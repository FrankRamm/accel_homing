# Beacon accelerometer X/Y homing endstop for Klipper and Kalico
#
# Detects the bump of the toolhead against the X/Y stop with the Beacon's
# accelerometer and fires the stepper MCU trsync from the host. One touch per
# home. The homing axis is watched in either direction until the toolhead
# travels, then only against the travel (or in a configured direction). A
# legacy Beacon path recognizes pressed starts from commanded versus measured
# motion. Legacy configured batch sensors require startup motion proof. Simple
# calibration and its saved models use XYZ magnitude and commanded travel instead.
#
# Experimental. The inherited upstream Beacon implementation reports use on
# three printers; these generic adapters have only been tested synthetically.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections
import json
import logging
import math
import struct
import sys
import threading

import mcu

try:
    from .accel_homing_backends import (ConfiguredAccelBackend, motion_proof_window,
                                        motion_startup_timeout, select_sensor)
except ImportError:  # Standalone offline replay harness
    from accel_homing_backends import (ConfiguredAccelBackend, motion_proof_window,
                                       motion_startup_timeout, select_sensor)

SAMPLE = struct.Struct("<hhh")
CLIPPED = 0x7FFF
NATIVE_LE = sys.byteorder == "little"
BEACON_ACCEL_DATA = "beacon_accel_data start_clock=%u delta_clock=%u data=%*s"
TRIGGER_CMD = "trsync_trigger oid=%c reason=%c"
MM_PER_S2_PER_G = 9806.65
WATCHDOG_INTERVAL = 0.010
STREAM_START_TIMEOUT = 0.5
BASELINE_MIN_SAMPLES = 50
BASELINE_MAX_SAMPLES = 65536  # bounded pre-arm history, never silently evicted
IMPACT_WINDOW = 0.015      # s after the onset in which the impact peak is kept
IDLE_CLEANUP = 3.0         # s, closes a homing session left open by an error
SECOND_TOUCH_TOLERANCE = 1.5   # mm, only used if a second touch is configured
MIN_SPEED = 50.0           # mm/s, below this impacts were too soft in testing
AUTO_DIRECTION_SPEED = 30.0    # mm/s of travel that fixes which way an impact points
PRESSED_MIN_SPEED = 40.0   # mm/s commanded before missing motion counts as pressed
PRESSED_RATIO = 0.25       # measured below this share of commanded: not moving
MOVING_RATIO = 0.5         # measured above this share: moving freely
MOTION_LAG = 0.015         # s, commanded speed is taken this much earlier, so step
                           # and sensor timing may be off by 11 ms without harm
COMMAND_WINDOW = 0.004     # s, window for the commanded speed
EARLY_INTERVAL = 0.003     # s, watchdog interval until the toolhead is seen moving
GAP_SAMPLES = 5            # missing samples that make the integrated speed untrustworthy
ENABLE_SETTLE = 0.25       # s, motors switched on before homing settle before the reading
BASELINE_NOISE_RATIO = 0.20  # all-axis peak-to-peak / current impact threshold
NATIVE_BASELINE_NOISE_FLOOR = 0.5  # g peak-to-peak, also capped by the impact threshold
NATIVE_PROOF_HISTORY_SAMPLES = 4096  # bounded startup history, never silently evicted
NATIVE_PROOF_SCAN_BUDGET = 1024      # same-window comparisons per reactor event
STEP_EPOCH_PRECISION = 0.000050      # s, bound on the first commanded-step search
AXES = {"x": 0, "y": 1}
FAILURES = {
    "no_data": "no fresh accelerometer data",
    "no_baseline": "no fresh quiet accelerometer baseline before the move",
    "baseline_overflow": "stationary baseline exceeded its bounded sample capacity",
    "travel": "no impact within the expected travel",
    "frozen": "accelerometer readings did not change while the steps ran",
    "scale": "another Beacon accelerometer client changed the range during homing",
    "stale_data": "accelerometer sample timestamps are stale or in the future",
    "invalid_data": "invalid or repeated accelerometer samples",
    "data_error": "accelerometer reported errors or overflows",
    "sample_gap": "accelerometer samples are missing or irregular",
    "sample_rate": "accelerometer sample rate is too low",
    "no_motion": "toolhead motion was not verified before the startup deadline",
    "proof_overflow": "native startup motion-proof history exceeded its bounded capacity",
}


def validate_threshold(value, name, error, full_scale=None):
    if not math.isfinite(value) or value <= 0.0:
        raise error("accel_homing: %s must be finite and greater than 0 g" % name)
    if full_scale is not None and value > 0.8 * full_scale:
        raise error("accel_homing: %s must be at most %.12g g (80%% of the"
                    " accelerometer range %.12g g)" % (name, 0.8 * full_scale, full_scale))


class QuietReference:
    """Bounded contiguous reference, optionally quiet, protected by the owner lock."""

    def __init__(self, axis, cutoff, duration, limit, threshold_g=None,
                 require_quiet=True):
        self.axis, self.cutoff, self.duration, self.limit = axis, cutoff, duration, limit
        self.threshold_g = threshold_g
        self.require_quiet = require_quiet
        self.first = self.last = None
        self.count = 0
        self.totals = [0.0, 0.0, 0.0]
        self.low = self.high = None
        self._samples = collections.deque()
        self._minimum = [collections.deque() for _ in range(3)]
        self._maximum = [collections.deque() for _ in range(3)]
        self.overflow = False
        # Diagnostic summaries only; none of these fields admit a baseline.
        self.accepted = self.pre_cutoff = self.resets = 0
        self.observed_low = self.observed_high = None
        self.best_count, self.best_span = 0, 0.0
        self.best_peak_to_peak = (0.0, 0.0, 0.0)
        self.oldest_batch_age = self.newest_batch_age = None

    def add(self, when, values):
        values = tuple(values)
        self.accepted += 1
        self.last = when
        self.observed_low = (list(values) if self.observed_low is None else
                             [min(a, b) for a, b in zip(self.observed_low, values)])
        self.observed_high = (list(values) if self.observed_high is None else
                              [max(a, b) for a, b in zip(self.observed_high, values)])
        if self.overflow:
            return "baseline_overflow"
        self._samples.append((self.accepted, when, values))
        self.totals = [a + b for a, b in zip(self.totals, values)]
        for value, minimum, maximum in zip(values, self._minimum, self._maximum):
            while minimum and minimum[-1][1] >= value:
                minimum.pop()
            while maximum and maximum[-1][1] <= value:
                maximum.pop()
            minimum.append((self.accepted, value))
            maximum.append((self.accepted, value))
        if self.require_quiet and any(high[0][1] - low[0][1] > self.limit
                                      for low, high in zip(self._minimum, self._maximum)):
            # Keep the longest contiguous suffix that is quiet on every axis.
            # Resetting to only the new sample would discard valid overlapping
            # windows whenever an old extremum is the sole offending sample.
            self.resets += 1
            while any(high[0][1] - low[0][1] > self.limit
                      for low, high in zip(self._minimum, self._maximum)):
                self._drop_first()
        # History older than a complete qualifying window is redundant. Keep
        # its boundary sample: dropping it must still leave both the minimum
        # count and the full duration, with the same tolerance as baseline().
        while (len(self._samples) > BASELINE_MIN_SAMPLES
               and when - self._samples[1][1] + 0.000000001 >= self.duration):
            self._drop_first()
        if len(self._samples) > BASELINE_MAX_SAMPLES:
            # At most one extra sample is held while pruning. Never silently
            # evict history needed to prove a complete window, or recover this
            # acquisition after its bounded capacity has been exceeded.
            self.overflow = True
            self._samples.clear()
            for queue in self._minimum + self._maximum:
                queue.clear()
            self.first, self.count = None, 0
            self.totals = [0.0, 0.0, 0.0]
            self.low = self.high = None
            return "baseline_overflow"
        self.first, self.count = self._samples[0][1], len(self._samples)
        self.low = [queue[0][1] for queue in self._minimum]
        self.high = [queue[0][1] for queue in self._maximum]
        span = self.last - self.first
        if (span, self.count) > (self.best_span, self.best_count):
            self.best_count, self.best_span = self.count, span
            self.best_peak_to_peak = tuple(b - a for a, b in zip(self.low, self.high))

    def _drop_first(self):
        number, _when, values = self._samples.popleft()
        self.totals = [a - b for a, b in zip(self.totals, values)]
        for queue in self._minimum + self._maximum:
            if queue[0][0] == number:
                queue.popleft()

    def baseline(self):
        if (self.overflow or self.count < BASELINE_MIN_SAMPLES or self.first is None
                or self.last - self.first + 0.000000001 < self.duration):
            return None
        return tuple(value / self.count for value in self.totals)


def clock_after(clock, reference):
    """32-bit clock comparison that survives wraparound."""
    return ((clock - reference) & 0xFFFFFFFF) < 0x80000000


def axis_values(data, index):
    """Raw samples of one sensor axis of a Beacon batch, without a Python loop."""
    if isinstance(data, tuple):
        return data[index::3]
    if NATIVE_LE:
        return memoryview(data).cast("h")[index::3]
    return [sample[index] for sample in SAMPLE.iter_unpack(data)]


def sample_count(data):
    return len(data) // (3 if isinstance(data, tuple) else 6)


def sample_slice(data, count):
    return data[:count * (3 if isinstance(data, tuple) else 6)]


def decode_clips(raw, previous, clips):
    """Beacon sends a clip in either direction as 0x7FFF. As in beacon_klipper,
    a clip takes the sign of the sample before it on the same axis."""
    out = list(raw)
    last = list(previous)
    for i, value in enumerate(out):
        axis = i % 3
        if value == CLIPPED:
            value = out[i] = clips[0] if last[axis] >= 0 else clips[1]
        last[axis] = value
    return out


def describe(r):
    """One line per homing pass, for the status command and klippy.log."""
    return ("%s pass %d: %s, onset %s g, peak %s g, noise +%.1f/-%.1f g, travel %s mm%s"
            % (r["axis"].upper(), r["pass"], r["result"], r["onset_g"], r["impact_peak_g"],
               r["noise_pos_g"], r["noise_neg_g"], r.get("travel_mm"),
               ", origin kept" if r.get("kept_origin") else ""))


def describe_reference(status):
    """Bounded, machine-axis diagnostic; raw sensor samples are not included."""
    def xyz(values):
        return "/".join("%.3f" % values[axis] for axis in ("x", "y", "z"))
    def age(value):
        return "unknown" if value is None else "%.2f" % value
    require_quiet = status.get("quiet_range_enforced", True)
    limit = ("limit %.3f g" % status["quiet_limit_g"] if require_quiet else
             "quiet range not required for calibration")
    result = ("baseline %s: %s (%s); best %s %d/%d samples, %.2f/%.2f ms,"
            " p-p XYZ %s g (%s); observed p-p XYZ %s g;"
            " accepted %d, pre-cutoff %d, resets %d; sample age %s ms"
            " (max %.2f ms), batch oldest/newest %s/%s ms"
            % (status["axis"].upper(), status["result"].replace("_", " "), status["phase"].replace("_", " "),
               "quiet" if require_quiet else "reference",
               status["best_quiet_samples"], status["required_samples"],
               status["best_quiet_span_ms"], status["required_span_ms"],
               xyz(status["best_quiet_peak_to_peak_g"]), limit,
               xyz(status["observed_peak_to_peak_g"]), status["accepted_samples"],
               status["pre_cutoff_samples"], status["quiet_resets"],
               age(status["sample_age_ms"]), status["max_sample_age_ms"],
               age(status["oldest_batch_age_ms"]), age(status["newest_batch_age_ms"])))
    if status["motion_proof_noise_bound_g"] is not None:
        result += "; motion-proof noise bound XYZ %s g" % xyz(status["motion_proof_noise_bound_g"])
    if status.get("impact_threshold_g") is not None:
        result += "; trial impact threshold %.6g g" % status["impact_threshold_g"]
    return result


def describe_motion_proof(proof):
    """Short generic proof summary for a stopped move and STATUS."""
    def value(name):
        item = proof.get(name)
        return "unknown" if item is None else "%.3f" % item
    noise = proof.get("noise_bound_xyz_g")
    noise = "none" if noise is None else "/".join(
        "unknown" if item is None else "%.4f" % item for item in noise)
    return ("motion proof %s (%s, %s); raw/lower/upper %s/%s/%s mm/s,"
            " peak %s mm/s; command %s mm/s eligible %s;"
            " sample/request/deadline %s/%s/%s ms after arm; noise XYZ %s g"
            % (proof["result"], proof["backend"], proof["method"],
               value("raw_speed_mm_s"), value("lower_speed_mm_s"), value("upper_speed_mm_s"),
               value("full_arm_peak_mm_s"), value("commanded_speed_mm_s"), proof["command_eligible"],
               value("evaluated_sample_after_arm_ms"), value("request_after_arm_ms"),
               value("deadline_after_arm_ms"), noise))


def homing_warnings(name, info):
    """Rail settings known to weaken detection. Advice only, never an error."""
    warnings = []
    retract = info.retract_dist
    rehome = getattr(info, "min_home_dist", 0.0) or 0.0
    speeds = [info.speed] + ([info.second_homing_speed] if retract or rehome else [])
    if min(speeds) < MIN_SPEED:
        warnings.append(
            "beacon_accel_homing: %s homes at %.0f mm/s. Impacts below about %.0f mm/s"
            " can be too soft to detect, and the travel stop then ends homing with an error."
            % (name, min(speeds), MIN_SPEED))
    if (retract or rehome) and info.second_homing_speed < 0.6 * info.speed:
        warnings.append(
            "beacon_accel_homing: %s second_homing_speed is much lower than homing_speed."
            " A slow second touch can be too soft to detect. Use the same speed for both."
            % (name,))
    return warnings


class Attempt:
    """State of one armed homing move. The trigger path runs in the Beacon serial thread."""

    def __init__(self, endstop, pass_no, arm_time, arm_clock, deadline, deadline_clock,
                 index, map_sign, direction, limit_raw, count_g):
        self.endstop = endstop
        self.axis = endstop.axis
        self.pass_no = pass_no
        self.arm_time = arm_time
        self.arm_clock = arm_clock
        self.deadline = deadline
        self.deadline_clock = deadline_clock
        self.index = index
        self.map_sign = map_sign
        self.direction = direction
        self.limit_raw = limit_raw
        self.count_g = count_g
        self.baseline = None
        self.done = False
        self.reason = None  # "impact", "travel", "no_data", "no_baseline"
        self.last_data = 0.0
        self.last_samples = [0, 0, 0]  # previous sample per sensor axis, clips decoded
        self.noise_pos = self.noise_neg = 0.0
        self.noise_checked = 0.0   # highest reading compared with the threshold
        self.pending_noise = (0.0, 0.0, 0.0)
        self.bases = None           # resting reading of all three sensor axes
        self.sums = [0.0, 0.0, 0.0]  # integrated readings since the pass started
        self.vector_mode = False   # XYZ magnitude; no acceleration-derived motion proof
        self.vector_unchanged_since = None
        self.proof_noise = None    # measured uncertainty for calibration/expanded references
        self.proof_state = None    # atomic (sums, sample count, dt, end clock) for that proof
        self.proof_speed_peak = 0.0  # conservative lower bound, before the startup deadline
        self.proof_speed_peak_time = None
        self.proof_deadline = None  # configured-sensor deadline; no native step-history policy
        self.motion_deadline = None  # expanded native references must prove motion or pressed
        self.proof_history = []     # bounded coherent (time, sample count, XYZ prefix sums)
        self.proof_prefix = (0.0, 0.0, 0.0)
        self.proof_history_count = 0
        self.proof_overflow = False
        self.proof_step_positions = None
        self.proof_step_error = None
        self.proof_step_epoch = None
        self.proof_interval_peak = 0.0
        self.proof_interval = None
        self.proof_scan_end = self.proof_scan_start = 0
        self.proof_scan_event = None
        self.proof_scan_comparisons = 0
        self.proof_lower = self.proof_upper = self.proof_commanded = None
        self.proof_raw = None
        self.proof_command_eligible = False
        self.proof_measured_at = None
        self.proof_first_sample_time = self.proof_sample_time = None
        self.proof_method = None
        self.request_time = None
        self.sample_dt = 0.0
        self.first_clock = self.last_end_clock = None
        self.auto_direction = 0     # impact direction found from the travel
        self.moving = False         # toolhead seen moving with the steps
        self.motion_seen = False    # relative motion proof, including slow homes
        self.effective_speed = self.effective_accel = None
        self.varied = False         # readings changed at least once (sensor not frozen)
        self.gap = False            # samples were missing, so the speed is not known
        self.pressed_checks = 0
        self.checked_clock = None
        self.steppers = []
        self.halt_delta = None      # axis movement from start to halt, signed
        self.onset_batch = None
        self.onset_sample = None    # original captured packet and selected sample
        self.onset_raw = 0.0
        self.impact_peak = 0.0
        self.impact_until = None
        self.record = None
        self.start_position = self.expected_travel = None


class BeaconAccelEndstop:
    """Endstop object returned for endstop_pin: beacon_accel:x or beacon_accel:y."""

    def __init__(self, owner, axis):
        self._owner = owner
        self.axis = axis
        self._mcu = None
        self._dispatch = None
        self._triggers = []

    # Endstop interface used by the homing code
    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        # The trigger sync lives on the stepper MCU, taken from the first stepper.
        if self._dispatch is None:
            self._mcu = stepper.get_mcu()
            self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers() if self._dispatch is not None else []

    def setup_trigger(self, command_for):
        # One trsync per stepper MCU, each triggered on a dedicated command queue
        # of its MCU, so the stop never waits behind queued trsync or stepper
        # traffic (measured 20-74 ms delays on the shared queue).
        if self._dispatch is None:
            return
        trsyncs = getattr(self._dispatch, "_trsyncs", None)
        if trsyncs:
            self._triggers = [(command_for(t.get_mcu()), t.get_oid()) for t in trsyncs]
        else:
            self._triggers = [(command_for(self._mcu), self._dispatch.get_oid())]

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        attempt = self._owner.prepare(self, print_time)
        completion = self._dispatch.start(print_time)
        self._owner.arm(attempt)
        return completion

    def home_wait(self, home_end_time):
        try:
            self._dispatch.wait_end(home_end_time)
        finally:
            attempt = self._owner.disarm()
        result = self._dispatch.stop()
        return self._owner.finish(self, attempt, result)

    def query_endstop(self, print_time):
        return 0

    def trigger(self, reason):
        for command, oid in self._triggers:
            command.send([oid, reason])


class BeaconAccelHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        if not callable(getattr(mcu, "TriggerDispatch", None)):
            raise config.error("accel_homing: this fork/version has no MCU TriggerDispatch;"
                               " update to a compatible Klipper or Kalico version")
        other = "accel_homing" if self.name == "beacon_accel_homing" else "beacon_accel_homing"
        if config.has_section(other):
            raise config.error("Use only one [accel_homing] or [beacon_accel_homing] section")
        self.sensor_name = select_sensor(config)
        self.is_beacon = self.sensor_name.split()[0] == "beacon"
        self.beacon_name = self.sensor_name
        if config.get("mcu", None) is not None:
            config.deprecate("mcu")  # the stepper MCU now comes from the steppers
        threshold = config.getfloat("threshold", 8.0)
        self.threshold_g = {a: config.getfloat("%s_threshold" % a, threshold) for a in AXES}
        validate_threshold(threshold, "threshold", config.error)
        for axis, value in self.threshold_g.items():
            validate_threshold(value, axis + "_threshold", config.error)
        self.impact_sign = {a: config.getint("%s_impact_sign" % a, 0, minval=-1, maxval=1)
                            for a in AXES}
        self.homing_accel = config.getfloat("homing_accel", 5000.0, minval=0.0)
        self.back_off_dist = config.getfloat("back_off_dist", 5.0, minval=0.0, maxval=20.0)
        self.baseline_batches = config.getint("baseline_batches", 25, minval=10, maxval=200)
        self.baseline_time = config.getfloat("baseline_time", 0.1, minval=0.05, maxval=1.0)
        self.baseline_noise_floor = config.getfloat("baseline_noise_floor", 0.7,
                                                    minval=0.0, maxval=0.7)
        self.enable_settle_time = config.getfloat("enable_settle_time", ENABLE_SETTLE,
                                                  minval=0.0, maxval=2.0)
        self.stream_timeout = config.getfloat("stream_timeout", 0.05, minval=0.01, maxval=0.5)
        self.travel_margin = config.getfloat("travel_margin", 1.1, minval=1.0, maxval=2.0)
        self.watchdog_margin = config.getfloat("watchdog_margin", 0.15, minval=0.0, maxval=2.0)
        scalar_options = {"homing_accel": self.homing_accel,
                          "back_off_dist": self.back_off_dist, "baseline_time": self.baseline_time,
                          "baseline_noise_floor": self.baseline_noise_floor,
                          "enable_settle_time": self.enable_settle_time,
                          "stream_timeout": self.stream_timeout, "travel_margin": self.travel_margin,
                          "watchdog_margin": self.watchdog_margin}
        for option, value in scalar_options.items():
            if not math.isfinite(value):
                raise config.error("accel_homing: %s must be finite" % option)
        # Load the sensor first so its MCU is configured before our connect handler.
        self.sensor = self.printer.load_object(config, self.sensor_name)
        self.backend = None if self.is_beacon else ConfiguredAccelBackend(self, config, self.sensor)
        if self.backend is not None and config.get("threshold", None) is None:
            default_threshold = min(8.0, self.backend.full_scale * 0.5)
            for axis in AXES:
                if config.get(axis + "_threshold", None) is None:
                    self.threshold_g[axis] = default_threshold
        if self.backend is not None and self.stream_timeout > self.backend.max_age:
            self.stream_timeout = self.backend.max_age

        self.lock = threading.Lock()
        self.endstops = {}
        self.trigger_cmds = {}
        self.beacon = self.helper = self.toolhead = self.main_mcu = None
        self.impact_window = 0
        self.beacon_freq = 1.0
        self.saved_handler = None
        self.sensor_clock = self.clock_to_time = None
        self.sensor_fault = None
        self.sample_timing_fault = None
        self.scale = self.session_scale = None
        self.clip_values = (CLIPPED, -CLIPPED - 1)
        self.stream_alone = False
        self.full_scale_g = None
        self.recent = collections.deque(maxlen=self.baseline_batches)
        self.attempt = self.tail = self.pending_move = None
        self.homing_active = self.stream_on = False
        self.preparing = False
        self.preparation = None
        self.baseline_status = None
        self.session_axes = {}
        self.rails_by_axis = {}
        self.passes = 0
        self.saved_accel = None
        self.last_batch = None
        self.last_activity = 0.0
        self.homed_before = self.last_pass = None
        self.last = {}
        self.history = collections.deque(maxlen=20)
        self.capture_batches = None
        self.capture_samples = self.capture_dropped = 0
        self.capture_limit = 200000
        self.capture_baseline = None
        self.capture_attempt = None
        self.capture_attempts = 0
        self.beta_capture = self.beta_motion_capture = None
        self.beta_last_diagnostics = {}
        self.kinematics_home = None
        self.kinematics_home_owner = None
        self.ordinary_home = None
        self.operation_generation = 0
        self.noise_session = self.noise_window = None
        self.owned_sensor_clients = collections.deque(maxlen=8)
        self.watchdog = self.reactor.register_timer(self._watchdog)
        self.idle_timer = self.reactor.register_timer(self._idle_check)

        self.printer.lookup_object("pins").register_chip("beacon_accel", self)
        self.printer.lookup_object("pins").register_chip("accel_homing", self)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("homing:home_rails_begin", self._handle_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end", self._handle_rails_end)
        self.printer.register_event_handler("homing:homing_move_begin", self._handle_move_begin)
        self.printer.register_event_handler("homing:homing_move_end", self._handle_move_end)
        self.printer.register_event_handler("stepper_enable:motor_off", self._end_session)
        self.printer.register_event_handler("klippy:shutdown", self._end_session)
        for event in ("stepper_enable:motor_off", "klippy:shutdown", "klippy:disconnect"):
            self.printer.register_event_handler(event, self._cancel_pending_operations)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("BEACON_ACCEL_HOMING_STATUS", self.cmd_STATUS,
                               desc="Report the last Beacon accelerometer homing results")
        gcode.register_command("SET_BEACON_ACCEL_HOMING", self.cmd_SET,
                               desc="Change the threshold or impact sign until restart")
        gcode.register_command("SET_ACCEL_HOMING", self.cmd_SET,
                               desc="Change independent X/Y thresholds until restart")
        try:
            from .accel_homing_workflows import AccelHomingWorkflows
        except ImportError:
            from accel_homing_workflows import AccelHomingWorkflows
        self.workflows = AccelHomingWorkflows(self, config)

    # Pin chip
    def setup_pin(self, pin_type, pin_params):
        if pin_type != "endstop" or pin_params["pin"] not in AXES:
            raise self.printer.config_error("beacon_accel pins are 'x' and 'y' endstops only")
        if pin_params["invert"] or pin_params["pullup"]:
            raise self.printer.config_error("beacon_accel endstops cannot be inverted or pulled up")
        axis = pin_params["pin"]
        if axis in self.endstops:
            raise self.printer.config_error("beacon_accel:%s is used twice" % axis)
        self.endstops[axis] = BeaconAccelEndstop(self, axis)
        return self.endstops[axis]

    # Startup checks: problems show at restart, not in the middle of a home
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.main_mcu = self.printer.lookup_object("mcu")
        if self.is_beacon:
            self._attach_beacon()
            self.sensor_clock = self.beacon._mcu
            self.clock_to_time = self.beacon._clock32_to_time
        else:
            self.helper = self.backend.helper
            self.scale = {"name": "normalized_g", "id": None, "scale": 1.0}
            self.sensor_clock = self.backend
            self.clock_to_time = self.backend.clock_to_time
        self.impact_window = int(self.sensor_clock.seconds_to_clock(IMPACT_WINDOW))
        self.beacon_freq = float(self.sensor_clock.seconds_to_clock(1.0))
        full_scale = self._full_scale_g(self.scale)
        for axis, value in self.threshold_g.items():
            validate_threshold(value, axis + "_threshold", self.printer.config_error, full_scale)
        for endstop in self.endstops.values():
            endstop.setup_trigger(self._trigger_command)
        if self.homing_accel and not any(hasattr(self.toolhead, n) for n in (
                "set_max_velocities", "set_accel", "_calc_junction_deviation")):
            raise self.printer.config_error(
                "beacon_accel_homing: this Klipper version cannot change the homing"
                " acceleration. Update it or set homing_accel: 0")
        configfile = self.printer.lookup_object("configfile")
        for rail in getattr(self.toolhead.get_kinematics(), "rails", []):
            axes = [axis for axis, es in self.endstops.items()
                    if any(es is candidate for candidate, _name in rail.get_endstops())]
            if axes:
                for axis in axes:
                    self.rails_by_axis[axis] = rail
                if self.backend is not None:
                    self.backend.validate_rail(rail.get_homing_info(), self.printer.config_error)
                for msg in homing_warnings(rail.get_name(), rail.get_homing_info()):
                    configfile.runtime_warning(msg)
        kin = self.toolhead.get_kinematics()
        home = getattr(kin, "home", None)
        if (getattr(home, "__self__", None) is self
                and getattr(home, "__func__", None) is BeaconAccelHoming._home_with_retract):
            return  # A repeated connect must not stack the synchronous hook.
        if callable(home):
            self.kinematics_home_owner, self.kinematics_home = kin, home
            kin.home = self._home_with_retract

    def _cancel_pending_operations(self, *args):
        self.operation_generation += 1
        if self.ordinary_home is not None:
            self.ordinary_home["valid"] = False
        if self.noise_session is not None:
            self.stop_noise_measurement()
        if self.beta_capture is not None:
            self.beta_capture["cancelled"] = True
            self._stop_stream()

    def _home_with_retract(self, homing):
        """Retract synchronously after kinematics finishes final coordinate fixes.

        home_rails_end precedes Klipper/Kalico's adjust_pos correction. Moving
        from that event changes the correction itself. Public G28 handlers and
        safe_z_home/override chains remain untouched by this kinematics hook.
        """
        if self.ordinary_home is not None:
            self.ordinary_home["valid"] = False
            return self.kinematics_home(homing)
        suppressed = (self.workflows.session is not None or self.workflows.in_home
                      or self.noise_session is not None)
        context = {"homing": homing, "rails": {}, "valid": not suppressed}
        self.ordinary_home = context
        try:
            result = self.kinematics_home(homing)
            if context["valid"]:
                self._retract_completed_home(context)
            return result
        finally:
            context["valid"] = False
            self.ordinary_home = None

    def _retract_completed_home(self, context):
        distance = context.get("distance", self.back_off_dist)
        if (not distance or not context["rails"]
                or (hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown())):
            return
        if not math.isfinite(distance) or distance < 0.0:
            raise self.printer.command_error("accel_homing: invalid final back-off distance")
        position = self.toolhead.get_position()
        coord, speeds = [None, None, None], []
        for axis, (rail, record) in context["rails"].items():
            if self.last.get(axis) is not record or self._homed(axis) is not True:
                continue
            info = rail.get_homing_info()
            low, high = rail.get_range()
            speed = getattr(info, "retract_speed", None) or info.speed
            current = position[AXES[axis]]
            if (not all(math.isfinite(value) for value in
                        (low, high, info.position_endstop, speed, current))
                    or low >= high or speed <= 0.0):
                raise self.printer.command_error("accel_homing: invalid final back-off geometry/speed")
            away = -1.0 if info.positive_dir else 1.0
            target = min(max(info.position_endstop + away * distance, low), high)
            target_free = (target - info.position_endstop) * away
            free = (current - info.position_endstop) * away
            # Kalico may already have retracted. Never move back toward a stop
            # or add another configured distance to an existing final retract.
            if target_free <= 0.0 or free >= target_free - 0.000001:
                continue
            coord[AXES[axis]] = target
            speeds.append(speed)
        workflow_session = context.get("workflow_session")
        if (speeds and context["valid"] and (workflow_session is None
                                            or self.workflows.session is workflow_session)):
            generation = self.operation_generation
            def current():
                return (context["valid"] and self.operation_generation == generation
                        and (workflow_session is None or self.workflows.session is workflow_session)
                        and not (hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown())
                        and all(self.last.get(axis) is record and self._homed(axis) is True
                                for axis, (_rail, record) in context["rails"].items()))
            try:
                self._limit_accel()
                # Velocity setters can flush/yield. A motor-off or cancellation
                # during the cap change must not authorize a later move.
                if not current():
                    raise self.printer.command_error("accel_homing: final back-off was interrupted")
                self.toolhead.manual_move(coord, min(speeds))
                self.toolhead.wait_moves()
                if not current():
                    raise self.printer.command_error("accel_homing: final back-off was interrupted")
            finally:
                self._restore_accel()

    def release_workflow_home(self, axis, source_record, distance, session):
        """Release only a terminal workflow home after its caller qualifies it."""
        if (self.workflows.session is not session or session.get("state") != "running"
                or axis not in self.rails_by_axis or self.last.get(axis) is not source_record
                or self._homed(axis) is not True):
            raise self.printer.command_error("accel_homing: terminal workflow home is no longer current")
        context = {"rails": {axis: (self.rails_by_axis[axis], source_record)},
                   "valid": True, "distance": distance, "workflow_session": session}
        self._retract_completed_home(context)

    def _trigger_command(self, stepper_mcu):
        command = self.trigger_cmds.get(stepper_mcu)
        if command is None:
            queue = stepper_mcu.alloc_command_queue()
            command = stepper_mcu.lookup_command(TRIGGER_CMD, cq=queue)
            self.trigger_cmds[stepper_mcu] = command
        return command

    def _attach_beacon(self):
        error = self.printer.config_error
        beacon = self.printer.lookup_object(self.beacon_name)
        helper = getattr(beacon, "accel_helper", None)
        if helper is None:
            raise error("beacon_accel_homing: the Beacon reports no accelerometer."
                        " A Beacon RevH (or newer) with current firmware is required")
        # Internal names of beacon_klipper this module relies on.
        missing = [n for n, obj in (
            ("accel_stream_cmd", helper), ("_scale", helper), ("_stream_en", helper),
            ("config", helper), ("_handle_accel_data", helper), ("_mcu", beacon),
            ("compat_mcu_register_response", beacon), ("_clock32_to_time", beacon))
            if not hasattr(obj, n)]
        if missing or not hasattr(helper.config, "axes_map"):
            raise error("beacon_accel_homing: this beacon_klipper version is not supported"
                        " (missing %s)" % ", ".join(missing or ["axes_map"]))
        scales = list(getattr(helper, "_scales", {}).values()) or [helper._scale]
        widest = max(scales, key=lambda s: s.get("scale", 0.0))
        if not 0.01 < widest.get("scale", 0.0) < 100.0:
            raise error("beacon_accel_homing: implausible Beacon accelerometer scale %r" % (widest,))
        self.beacon, self.helper, self.scale = beacon, helper, widest
        clips = getattr(helper, "_clip_values", None)
        self.clip_values = tuple(clips) if clips else (CLIPPED, -CLIPPED - 1)
        serial = getattr(beacon._mcu, "_serial", None)
        self.saved_handler = getattr(serial, "handlers", {}).get(
            ("beacon_accel_data", None), helper._handle_accel_data)
        beacon.compat_mcu_register_response(self._batch_hook, BEACON_ACCEL_DATA)

    def _full_scale_g(self, scale):
        if self.backend is not None:
            return self.backend.full_scale
        clip = getattr(self.helper, "_clip_values", (None,))[0]
        return clip * scale["scale"] / 1000.0 if clip else None

    def _raw_axis(self, axis):
        """Sensor index and sign of a machine axis, from Beacon's accel_axes_map."""
        if self.backend is not None:
            return AXES[axis], 1  # The configured driver already applies axes_map.
        index, sign = self.helper.config.axes_map[AXES[axis]]
        return index, sign

    # Stream and acceleration, owned only for the duration of a homing session
    def _start_stream(self):
        self.sensor_fault = None
        self.sample_timing_fault = None
        if self.backend is not None:
            self.session_scale = self.scale
            self.full_scale_g = self.backend.full_scale
            self.backend.start()
            if not any(client is self.backend.client for client in self.owned_sensor_clients):
                self.owned_sensor_clients.append(self.backend.client)
            self.stream_on = True
            return
        helper = self.helper
        # Streaming alone at our own scale: a Beacon client starting later
        # (the web API dump) would switch the sensor range mid-move.
        self.stream_alone = not helper._stream_en
        if helper._stream_en:
            # Already streaming for someone else, at their scale.
            self.session_scale = helper._scale
        else:
            self.session_scale = self.scale
            helper.accel_stream_cmd.send([1, self.scale["id"]])
        self.stream_on = True
        self.full_scale_g = self._full_scale_g(self.session_scale)

    def _stop_stream(self):
        if self.backend is not None:
            self.stream_on = False
            self.backend.stop()
            return
        if self.stream_on:
            self.stream_on = False
            if not self.helper._stream_en:
                self.helper.accel_stream_cmd.send([0, 0])

    def _limit_accel(self):
        accel = self.toolhead.get_max_velocity()[1]
        if self.homing_accel and accel > self.homing_accel and self.saved_accel is None:
            self.saved_accel = accel
            self._set_accel(self.homing_accel)

    def _restore_accel(self):
        if self.saved_accel is not None:
            accel, self.saved_accel = self.saved_accel, None
            self._set_accel(accel)

    def _set_accel(self, accel):
        toolhead = self.toolhead
        if hasattr(toolhead, "set_max_velocities"):   # Klipper since 2024
            toolhead.set_max_velocities(None, accel, None, None)
        elif hasattr(toolhead, "set_accel"):          # Kalico
            toolhead.set_accel(accel)
        else:                                         # older Klipper
            toolhead.max_accel = accel
            toolhead._calc_junction_deviation()

    def _end_session(self, *args, beta_tail=False):
        """Stop the stream and restore the acceleration. Safe to call any time."""
        self.homing_active = False
        with self.lock:
            self.preparation = None
        self.preparing = False
        self.session_axes = {}
        self.reactor.update_timer(self.idle_timer, self.reactor.NEVER)
        state = self.beta_capture
        keep_tail = (beta_tail and state is not None and not state.get("cancelled")
                     and not state.get("fault") and self.sensor_fault is None
                     and self.capture_batches is not None
                     and self.capture_attempt is self.last_pass
                     and self.last_pass is not None and self.last_pass.reason == "impact"
                     and not (hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown()))
        if keep_tail:
            state["tail_retained"] = True
        elif self.helper is not None:
            self._stop_stream()
        if self.toolhead is not None:
            self._restore_accel()

    def _idle_check(self, eventtime):
        # Klipper sends no event when homing fails outside G28, so a session
        # left open closes itself once nothing has been armed for a while.
        if not self.homing_active:
            return self.reactor.NEVER
        if not self.preparing and self.attempt is None and eventtime - self.last_activity > IDLE_CLEANUP:
            logging.info("beacon_accel_homing: closing a homing session left open")
            self._end_session()
            return self.reactor.NEVER
        return eventtime + 0.5

    def preflight_motion_policy(self, axes, error=None):
        """Check configured X/Y plans against live limits without any effects.

        Workflow callers run this before their first motion. Rail-begin and
        prepare() still revalidate because limits may change after preflight.
        """
        if self.backend is None:
            return
        if error is None:
            error = self.printer.command_error
        if self.toolhead is None:
            raise error("accel_homing: printer is not connected")
        for axis in axes:
            if axis not in AXES:
                continue
            rail = self.rails_by_axis.get(axis)
            if rail is None or axis not in self.endstops:
                raise error("accel_homing: axis %s must use this module's accelerometer endstop"
                            % axis.upper())
            self.backend.validate_rail(rail.get_homing_info(), error)

    # Homing lifecycle
    def _handle_rails_begin(self, homing, rails):
        # Rails come from the homing call itself, so no kinematics or rail order
        # is assumed.
        axes = {}
        for rail in rails:
            for es, _name in rail.get_endstops():
                for axis, endstop in self.endstops.items():
                    if es is endstop:
                        axes[axis] = rail
        if not axes:
            return
        self.workflows.check_home_axes(axes)
        self._end_session()
        if len(axes) > 1:
            raise self.printer.command_error("beacon_accel_homing homes one axis at a time")
        if self.helper is None:
            raise self.printer.command_error("beacon_accel_homing: accelerometer is not available")
        if self.backend is not None:
            for rail in axes.values():
                self.backend.validate_rail(rail.get_homing_info(), self.printer.command_error)
        self.rails_by_axis.update(axes)
        self.session_axes = axes
        self.passes = 0
        self.recent.clear()
        self.tail = self.last_pass = None
        self.homed_before = self._homed_at_stop(*list(axes.items())[0])
        self.homing_active = True
        self.last_activity = started = self.reactor.monotonic()
        self.reactor.update_timer(self.idle_timer, started + 0.5)
        self.preparing = True
        try:
            self._limit_accel()
            self._enable_motors(list(axes)[0])
            self._back_off_others(list(axes)[0])
            if not self.homing_active or self.session_axes is not axes:
                raise self.printer.command_error("accel_homing: homing preparation cancelled")
            self.last_batch = None
            # Enabling/backoff may itself yield while the toolhead drains a
            # large queue. Sensor startup gets its own actual start time.
            self.last_activity = started = self.reactor.monotonic()
            self._start_stream()
        except Exception:
            self._end_session()
            raise
        finally:
            self.preparing = False
        # Wait in real time until accelerometer data actually flows.
        while self.last_batch is None or self.last_batch < started:
            now = self.reactor.monotonic()
            if self.sensor_fault is not None:
                reason = self.sensor_fault
                self._end_session()
                raise self.printer.command_error("accel_homing: %s" % FAILURES[reason])
            if now - started > STREAM_START_TIMEOUT:
                self._end_session()
                raise self.printer.command_error(
                    "beacon_accel_homing: accelerometer stream did not start")
            self.reactor.pause(now + 0.005)

    def _homed(self, axis):
        kin = self.toolhead.get_kinematics()
        try:
            return axis in kin.get_status(self.reactor.monotonic())["homed_axes"]
        except Exception:
            return None

    def _enable_motors(self, axis):
        """Switch on the motors of the homing move before the resting reading.
        Klipper switches a motor on with its first step, and the snap of motors
        that were off (after M84 or a failed home) fired the trigger at the start
        of the move."""
        stepper_enable = self.printer.lookup_object("stepper_enable", None)
        kin = self.toolhead.get_kinematics()
        if stepper_enable is None or kin is None:
            return
        lines = []
        for stepper in kin.get_steppers():
            try:
                if stepper.is_active_axis(axis):
                    line = stepper_enable.lookup_enable(stepper.get_name())
                    if not line.is_motor_enabled():
                        lines.append(line)
            except Exception:
                continue
        if not lines:
            return
        self.toolhead.flush_step_generation()
        print_time = self.toolhead.get_last_move_time()
        for line in lines:
            line.motor_enable(print_time)
        logging.info("beacon_accel_homing: switched on %d motors before homing %s",
                     len(lines), axis.upper())
        self.toolhead.dwell(self.enable_settle_time)

    def _back_off_others(self, homing_axis):
        """Move axes this module homed off their stops before homing another one.
        An axis left pushed into its stop keeps the belts and the gantry loaded,
        which disturbs the next axis (false triggers, lost steps)."""
        others = [(axis, rail) for axis, rail in self.rails_by_axis.items()
                  if axis != homing_axis and self.back_off_dist and self._homed(axis)]
        if not others:
            return
        coord, speeds = [None, None, None], []
        position = self.toolhead.get_position()
        for axis, rail in others:
            info = rail.get_homing_info()
            away = -1.0 if info.positive_dir else 1.0
            # Distance from the stop into the travel range, negative past the stop.
            free = (position[AXES[axis]] - info.position_endstop) * away
            if free >= self.back_off_dist - 0.01:
                continue
            low, high = rail.get_range()
            target = info.position_endstop + away * self.back_off_dist
            coord[AXES[axis]] = min(max(target, low), high)
            speeds.append(getattr(info, "retract_speed", None) or info.speed)
        if speeds:
            self.toolhead.manual_move(coord, min(speeds))
            # Let the gantry settle before the resting reading is taken.
            self.toolhead.dwell(self.baseline_time)

    def _homed_at_stop(self, axis, rail):
        """Position before homing if the axis is homed and Klipper already has the
        toolhead at the stop, or within 5 mm past it in the homing direction."""
        homed = self._homed(axis)
        if homed is None:
            return None
        info = rail.get_homing_info()
        position = self.toolhead.get_position()[AXES[axis]]
        past = position - info.position_endstop
        past = past if info.positive_dir else -past
        return position if homed and -0.01 <= past <= 5.0 else None

    def _handle_rails_end(self, homing, rails):
        if not self.homing_active:
            return
        attempt = self.last_pass
        if (attempt is not None and attempt.reason == "pressed" and self.homed_before is not None
                and attempt.halt_delta is not None):
            # Pressed start on an axis that was already homed at the stop, for
            # example G28 twice in a row. The accelerometer cannot measure how
            # far the belts were already stretched, so the previous origin is
            # kept instead of moving it by that stretch on every home.
            position = self.toolhead.get_position()
            position[AXES[attempt.axis]] = self.homed_before + attempt.halt_delta
            self.toolhead.set_position(position)
            attempt.record["kept_origin"] = True
        context = self.ordinary_home
        if (context is not None and context["valid"] and context["homing"] is homing
                and attempt is not None and attempt.axis in self.session_axes
                and any(rail is self.session_axes[attempt.axis] for rail in rails)):
            record = attempt.record
            accepted = (attempt.reason == "impact" and
                        (record.get("moving") or record.get("motion_seen")
                         or record.get("impact_metric") == "xyz_magnitude")) or (
                            attempt.reason == "pressed" and record.get("kept_origin"))
            if accepted and self.last.get(attempt.axis) is record:
                context["rails"][attempt.axis] = (self.session_axes[attempt.axis], record)
        # Only this successful, exact beta home transition may retain a stream
        # for diagnostics. Error/shutdown callers keep unconditional teardown.
        if (self.beta_capture is not None and attempt is not None
                and attempt is self.capture_attempt and attempt.reason == "impact"):
            self._end_session(beta_tail=True)
        else:
            self._end_session()

    def _handle_move_begin(self, hmove):
        # A standstill right before every pass (first touch and second touch)
        # gives the resting reading the impact is compared against.
        if self.homing_active and any(es in self.endstops.values() for es in hmove.get_mcu_endstops()):
            self._prepare_reference()

    def _prepare_reference(self):
        """Finish queued enable/backoff/retract, then acquire before trsync/motion.

        Stock homing captures the move's print time only after this event.
        A queued dwell alone does not establish that the physical enable time
        has passed or that delivered sensor data was sampled afterwards.
        """
        self.preparing = True
        session_axes = self.session_axes
        try:
            self.toolhead.flush_step_generation()
            self.toolhead.wait_moves()
            if not self.homing_active or self.session_axes is not session_axes:
                raise self.printer.command_error("accel_homing: homing preparation cancelled")
            started = self.reactor.monotonic()
            cutoff = self.main_mcu.estimated_print_time(started)
            axis = next(iter(self.session_axes))
            count_g = self.session_scale["scale"] / 1000.0
            reference = QuietReference(axis, cutoff, self.baseline_time,
                                       self._reference_noise_limit(axis) / count_g,
                                       threshold_g=self.threshold_g[axis],
                                       require_quiet=not (self.workflows.calibration_reference(axis)
                                                          or self._uses_vector_detector(axis)))
            with self.lock:
                self.recent.clear()
                self.tail = None
                self.preparation = reference
                if self.backend is not None:
                    self.backend.prepare_reference()
            while True:
                now = self.reactor.monotonic()
                self.last_activity = now
                if not self.homing_active or self.session_axes is not session_axes:
                    raise self.printer.command_error("accel_homing: homing preparation cancelled")
                if self.sensor_fault is not None:
                    sensor_now = self.main_mcu.estimated_print_time(now)
                    self._remember_reference(reference, sensor_now, "acquisition", self.sensor_fault)
                    raise self.printer.command_error("accel_homing: %s; %s"
                                                     % (FAILURES[self.sensor_fault],
                                                        describe_reference(self.baseline_status)))
                with self.lock:
                    ready = reference.baseline() is not None
                    last = reference.last
                sensor_now = self.main_mcu.estimated_print_time(now)
                if ready and last is not None and -0.005 <= sensor_now - last <= self._reference_max_age():
                    self._remember_reference(reference, sensor_now, "acquisition", "ready")
                    return
                if now - started > self.baseline_time + STREAM_START_TIMEOUT:
                    raise self._baseline_error(reference, sensor_now, "acquisition", "acquisition_timeout")
                self.reactor.pause(now + 0.005)
        except Exception:
            self._end_session()
            raise
        finally:
            self.preparing = False

    def _remember_reference(self, reference, sensor_now, phase, result, axis=None):
        """Keep one constant-size summary through workflow/session cleanup."""
        axis = reference.axis if reference is not None else axis
        count_g = self.session_scale["scale"] / 1000.0
        mapping = ([(0, 1), (1, 1), (2, 1)] if self.backend is not None
                   else self.helper.config.axes_map)
        def mapped(values):
            return {name: round(values[index] * count_g, 6)
                    for name, (index, sign) in zip(("x", "y", "z"), mapping)}
        with self.lock:
            span = (0.0 if reference is None or reference.first is None
                    else reference.last - reference.first)
            observed = ((0.0, 0.0, 0.0) if reference is None or reference.observed_low is None
                        else tuple(b - a for a, b in zip(reference.observed_low, reference.observed_high)))
            bases = None if reference is None else reference.baseline()
            proof_noise = self._reference_motion_noise(reference, bases, axis)
            self.baseline_status = {
                "axis": axis, "phase": phase, "result": result,
                "quiet_range_enforced": reference is None or reference.require_quiet,
                "motion_proof_noise_bound_g": None if proof_noise is None else mapped(proof_noise),
                "quiet_samples": 0 if reference is None else reference.count,
                "quiet_span_ms": round(span * 1000.0, 3),
                "required_samples": BASELINE_MIN_SAMPLES,
                "required_span_ms": self.baseline_time * 1000.0,
                "best_quiet_samples": 0 if reference is None else reference.best_count,
                "best_quiet_span_ms": 0.0 if reference is None else round(reference.best_span * 1000.0, 3),
                "best_quiet_peak_to_peak_g": mapped((0.0, 0.0, 0.0) if reference is None
                                                    else reference.best_peak_to_peak),
                "observed_peak_to_peak_g": mapped(observed),
                "quiet_limit_g": (self._reference_noise_limit(axis) if reference is None
                                  else reference.limit * count_g),
                "impact_threshold_g": None if reference is None else reference.threshold_g,
                "accepted_samples": 0 if reference is None else reference.accepted,
                "pre_cutoff_samples": 0 if reference is None else reference.pre_cutoff,
                "quiet_resets": 0 if reference is None else reference.resets,
                "sample_age_ms": (None if reference is None or reference.last is None
                                  else round((sensor_now - reference.last) * 1000.0, 3)),
                "max_sample_age_ms": self._reference_max_age() * 1000.0,
                "oldest_batch_age_ms": (None if reference is None or reference.oldest_batch_age is None
                                        else round(reference.oldest_batch_age * 1000.0, 3)),
                "newest_batch_age_ms": (None if reference is None or reference.newest_batch_age is None
                                        else round(reference.newest_batch_age * 1000.0, 3)),
            }

    def _baseline_error(self, reference, sensor_now, phase, result, axis=None):
        self._remember_reference(reference, sensor_now, phase, result, axis)
        detail = describe_reference(self.baseline_status)
        logging.info("accel_homing: %s", detail)
        message = (FAILURES["no_baseline"] if reference is None or reference.require_quiet else
                   "no fresh accelerometer reference before the move")
        return self.printer.command_error("accel_homing: %s; %s. Inspect stationary"
                                          " sensor readings and delivery timing; see ACCEL_HOMING_STATUS"
                                          % (message, detail))

    def _uses_vector_detector(self, axis):
        return self.workflows.simple_detection(axis)

    def _reference_noise_limit(self, axis):
        original = BASELINE_NOISE_RATIO * self.threshold_g[axis]
        floor = (min(NATIVE_BASELINE_NOISE_FLOOR, self.threshold_g[axis])
                 if self.backend is None else self.baseline_noise_floor)
        return max(original, floor)

    def _reference_motion_noise(self, reference, bases, axis):
        """Measured envelope for calibration and expanded quiet references."""
        if reference is None or bases is None or self._uses_vector_detector(axis):
            return None
        original = BASELINE_NOISE_RATIO * self.threshold_g[axis] / (self.session_scale["scale"] / 1000.0)
        if reference.require_quiet and not any(
                high - low > original for low, high in zip(reference.low, reference.high)):
            return None
        # Calibration omits the quiet-range gate, so its measured uncertainty
        # remains active even if a later impact candidate exceeds that range.
        # The mean need not lie at the midpoint, so half the range is unsafe.
        return tuple(max(abs(low - base), abs(high - base))
                     for low, high, base in zip(reference.low, reference.high, bases))

    def _reference_max_age(self):
        return self.stream_timeout if self.backend is None else self.backend.max_age

    def _collect_reference(self, params, data, count, now):
        """Validate sensor time and every axis while no homing move is armed."""
        interval = params["delta_clock"] / (count - 1.0) / self.beacon_freq
        start = self.clock_to_time(params["start_clock"])
        if not math.isfinite(interval) or interval <= 0.0 or not math.isfinite(start):
            return "invalid_data"
        sensor_now = self.main_mcu.estimated_print_time(now)
        raw = [axis_values(sample_slice(data, count), index) for index in range(3)]
        with self.lock:
            reference = self.preparation
            if reference is None:
                return None
            if (self.backend is None and self.helper._stream_en
                    and self.helper._scale.get("id") != self.session_scale.get("id")):
                return "scale"
            first_eligible = True
            for number in range(count):
                when = start + number * interval
                if when <= reference.cutoff:
                    reference.pre_cutoff += 1
                    continue
                if self.backend is None:
                    if first_eligible:
                        reference.oldest_batch_age = sensor_now - when
                        reference.newest_batch_age = sensor_now - (start + (count - 1) * interval)
                        first_eligible = False
                if not -0.005 <= sensor_now - when <= self._reference_max_age():
                    return "stale_data"
                if reference.last is not None:
                    if when <= reference.last:
                        return "invalid_data"
                    if when - reference.last > 5.0 * interval + 0.000003:
                        return "sample_gap"
                values = tuple(axis[number] for axis in raw)
                if (not all(math.isfinite(value) for value in values)
                        or (self.backend is None and CLIPPED in values)
                        or (self.full_scale_g is not None
                            and any(abs(value) * self.session_scale["scale"] / 1000.0
                                    >= self.full_scale_g for value in values))):
                    return "invalid_data"
                reference_error = reference.add(when, values)
                if reference_error is not None:
                    return reference_error
        return None

    def _handle_move_end(self, hmove):
        attempt, self.pending_move = self.pending_move, None
        if attempt is None or attempt.endstop not in hmove.get_mcu_endstops():
            return
        record = attempt.record
        positions = [sp for sp in getattr(hmove, "stepper_positions", []) if sp.trig_pos is not None]
        record["trigger_position_mm"] = {
            sp.stepper_name: round(sp.trig_pos * sp.stepper.get_step_dist(), 4) for sp in positions}
        travel = self._travel(positions, attempt.axis)
        record["travel_mm"] = None if travel is None else round(abs(travel), 3)
        record["travel_mm_exact"] = None if travel is None else abs(travel)
        record["trigger_travel_mm_signed"] = travel
        record["halt_travel_mm_signed"] = self._travel(positions, attempt.axis, halt=True)
        attempt.halt_delta = self._travel(positions, attempt.axis, halt=True)
        self.last_pass = attempt
        self.workflows.note_calibration_halt(record, positions)
        self.workflows.check_home_result(record)
        logging.info("beacon_accel_homing: %s, %.4f s after arm", describe(record),
                     record["trigger_after_arm_s"] or 0.0)
        if travel is not None:
            travel = abs(travel)
        if attempt.reason != "impact" or travel is None:
            return
        if attempt.pass_no >= 2:
            info = self.session_axes[attempt.axis].get_homing_info()
            # The second touch starts one retract away from the stop. A trigger
            # much later came from somewhere else, for example the end of a
            # second touch that followed an early first trigger.
            dists = [d for d in (info.retract_dist, getattr(info, "min_home_dist", 0.0)) if d]
            if dists and not (min(dists) - SECOND_TOUCH_TOLERANCE <= travel
                              <= max(dists) + SECOND_TOUCH_TOLERANCE):
                raise self.printer.command_error(
                    "beacon_accel_homing: second touch on %s triggered after %.2f mm instead"
                    " of about %.1f mm. The trigger was not at the stop, homing stopped"
                    % (attempt.axis.upper(), travel,
                       min(dists) if travel < min(dists) else max(dists)))

    def _travel(self, positions, axis, halt=False):
        """Signed distance moved along the homing axis until the trigger (or the
        halt), from the steppers."""
        try:
            kin = self.toolhead.get_kinematics()
            moved = {s.get_name(): 0.0 for s in kin.get_steppers()}
            for sp in positions:
                end = sp.halt_pos if halt else sp.trig_pos
                moved[sp.stepper_name] = (end - sp.start_pos) * sp.stepper.get_step_dist()
            return kin.calc_position(moved)[AXES[axis]]
        except Exception:
            logging.exception("beacon_accel_homing: travel calculation failed")
            return None

    def prepare(self, endstop, print_time):
        axis = endstop.axis
        rail = self.session_axes.get(axis)
        if rail is None or not self.homing_active:
            raise self.printer.command_error(
                "beacon_accel_homing: %s can only be homed with G28" % axis.upper())
        self.passes += 1
        info = rail.get_homing_info()
        max_velocity, accel = self.toolhead.get_max_velocity()
        if not all(math.isfinite(value) and value > 0 for value in
                   (max_velocity, accel, info.speed, info.second_homing_speed)):
            raise self.printer.command_error("accel_homing: homing speed, acceleration"
                                             " and threshold must be finite positive values")
        if self.passes == 1:
            low, high = rail.get_range()
            travel, speed = (high - low) * self.travel_margin, info.speed
        else:
            retract = max(info.retract_dist, getattr(info, "min_home_dist", 0.0) or 0.0)
            travel, speed = 2.0 * retract + 3.0, info.second_homing_speed
        # The toolhead never moves faster than its max_velocity.
        speed = min(speed, max_velocity)
        if self.backend is not None:
            self.backend.validate_motion_plan(speed, accel, self.printer.command_error)
        # Cruise time plus the time lost while ramping up to speed.
        deadline = print_time + travel / speed + speed / accel + self.watchdog_margin
        if self.backend is not None and not math.isfinite(deadline):
            raise self.printer.command_error("accel_homing: derived homing deadline must be finite")
        count_g = self.session_scale["scale"] / 1000.0
        threshold = self.threshold_g[axis]
        validate_threshold(threshold, axis + "_threshold", self.printer.command_error,
                           self.full_scale_g)
        if not math.isfinite(count_g) or count_g <= 0.0:
            raise self.printer.command_error("accel_homing: active accelerometer scale"
                                             " must be finite and positive")
        limit_raw = threshold / count_g
        if not math.isfinite(limit_raw) or limit_raw <= 0.0:
            raise self.printer.command_error("accel_homing: %s_threshold cannot produce a finite"
                                             " positive trigger limit at the active accelerometer scale"
                                             % axis)
        vector_mode = self._uses_vector_detector(axis)
        if vector_mode:
            limit_squared = limit_raw * limit_raw
            if not math.isfinite(limit_squared) or limit_squared <= 0.0:
                raise self.printer.command_error(
                    "accel_homing: %s_threshold cannot produce a finite positive squared XYZ"
                    " trigger limit at the active accelerometer scale" % axis)
        index, map_sign = self._raw_axis(axis)
        direction = self.impact_sign[axis]
        if self.sensor_fault is not None:
            raise self.printer.command_error("accel_homing: %s" % FAILURES[self.sensor_fault])
        now = self.main_mcu.estimated_print_time(self.reactor.monotonic())
        with self.lock:
            reference = self.preparation
            bases = reference.baseline() if reference is not None and reference.axis == axis else None
            result = ("missing_before_arm" if reference is None else
                      "calibration_reference_outside_trial" if not reference.require_quiet
                      and not (self.workflows.calibration_reference(axis)
                               or self._uses_vector_detector(axis)) else
                      "quiet_window_invalidated_before_arm" if bases is None else
                      "expired_before_arm" if reference.last is None
                      or not -0.005 <= now - reference.last <= self._reference_max_age() else "ready")
            if result != "ready":
                bases = None
            proof_noise = self._reference_motion_noise(reference, bases, axis)
            self.preparation = None
        if bases is None:
            error = self._baseline_error(reference, now, "before_arm", result, axis)
            self._end_session()
            raise error
        motion_deadline = None
        if self.backend is None and proof_noise is not None:
            timeout = motion_startup_timeout(speed, accel, self.stream_timeout, EARLY_INTERVAL)
            if not math.isfinite(timeout) or not math.isfinite(deadline):
                raise self.printer.command_error("accel_homing: derived native startup"
                                                 " and homing deadlines must be finite")
            motion_deadline = min(deadline, print_time + timeout)
        self._remember_reference(reference, now, "before_arm", "ready", axis)
        beacon_mcu = self.sensor_clock
        attempt = Attempt(endstop, self.passes, print_time,
                          beacon_mcu.print_time_to_clock(print_time) & 0xFFFFFFFF,
                          deadline, beacon_mcu.print_time_to_clock(deadline) & 0xFFFFFFFF,
                          index, map_sign, direction, limit_raw, count_g)
        attempt.vector_mode = vector_mode
        attempt.steppers = endstop.get_steppers()
        attempt.bases = bases
        attempt.proof_noise = proof_noise
        attempt.baseline = bases[index]
        attempt.last_samples = list(bases)
        attempt.effective_speed, attempt.effective_accel = speed, accel
        attempt.motion_deadline = motion_deadline
        if self.backend is not None and not attempt.vector_mode:
            attempt.proof_deadline = print_time + self.backend.motion_startup_timeout(speed, accel)
            attempt.proof_method = "full_arm_lower" if proof_noise is not None else "raw_integral"
        if motion_deadline is not None:
            try:
                attempt.proof_step_positions = tuple(
                    stepper.get_past_mcu_position(print_time) for stepper in attempt.steppers)
            except Exception as exc:
                attempt.proof_step_error = type(exc).__name__
        if hasattr(self.toolhead, "get_position"):
            attempt.start_position = self.toolhead.get_position()[AXES[axis]]
            attempt.expected_travel = abs(attempt.start_position - info.position_endstop)
        return attempt

    def arm(self, attempt):
        attempt.last_data = self.last_activity = self.reactor.monotonic()
        with self.lock:
            self.tail = None
            self.attempt = attempt
            if self.capture_batches is not None:
                # Bind the capture to this detector instance, never a previous
                # last_pass observed while an already-running sensor streams.
                if self.capture_attempt is not attempt:
                    self.capture_attempts += 1
                self.capture_attempt = attempt
        self.reactor.update_timer(self.watchdog, self.reactor.NOW)

    def disarm(self):
        with self.lock:
            attempt, self.attempt = self.attempt, None
            if attempt is not None:
                attempt.done = True
        self.reactor.update_timer(self.watchdog, self.reactor.NEVER)
        self.last_activity = self.reactor.monotonic()
        if attempt is not None:
            self._record(attempt)
            self.pending_move = attempt
            if attempt.onset_batch is not None:
                self.tail = attempt
        return attempt

    def finish(self, endstop, attempt, result):
        if result >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            raise self.printer.command_error("Communication timeout during homing")
        if attempt is not None and attempt.reason in FAILURES:
            detail = ("; " + describe_motion_proof(self._motion_proof_status(attempt))
                      if attempt.reason == "no_motion" else "")
            raise self.printer.command_error(
                "beacon_accel_homing: %s on %s, homing stopped%s"
                % (FAILURES[attempt.reason], endstop.axis.upper(), detail))
        if result != mcu.MCU_trsync.REASON_ENDSTOP_HIT or attempt is None:
            return 0.0
        return self.trigger_time(attempt)

    def trigger_time(self, attempt):
        """Time Klipper takes the trigger position from.

        A toolhead that never moved with the steps was already pressed against
        the stop, so the origin is where the move started. Otherwise it is the
        first accelerometer sample over the threshold."""
        if not attempt.vector_mode and attempt.reason == "impact" and self._motion_state(attempt) is True:
            attempt.reason = "pressed"
            if attempt.record is not None:
                attempt.record["result"] = "pressed"
        if attempt.record is not None:
            attempt.record["moving"] = attempt.moving
            attempt.record["motion_seen"] = attempt.motion_seen or attempt.moving
            if attempt.motion_deadline is not None or self.backend is not None:
                attempt.record["motion_proof"] = self._motion_proof_status(attempt)
        if attempt.reason == "pressed":
            return attempt.arm_time
        if attempt.onset_batch is None:
            return 0.0
        return self.onset_time(attempt)

    def _commanded_speed(self, attempt, print_time, window=COMMAND_WINDOW):
        """Stepper speed along the move at a print time, from the step history."""
        speed = 0.0
        for stepper in attempt.steppers:
            moved = (stepper.get_past_mcu_position(print_time)
                     - stepper.get_past_mcu_position(print_time - window))
            speed = max(speed, abs(moved) * stepper.get_step_dist() / window)
        return speed

    def _measured_speed(self, attempt, proof_state=None):
        """Speed of the toolhead in any direction, integrated from all three axes."""
        sums, dt = ((attempt.sums, attempt.sample_dt) if proof_state is None else
                    (proof_state[0], proof_state[2]))
        scale = attempt.count_g * MM_PER_S2_PER_G * dt
        return sum((v * scale) ** 2 for v in sums) ** 0.5

    def _motion_speed_bounds(self, attempt, proof_state):
        """Integrated speed bounds if disturbance stays within the pre-arm envelope."""
        sums, integrated_count, dt, _end_clock = proof_state
        scale = attempt.count_g * MM_PER_S2_PER_G * dt
        lower = sum((max(0.0, abs(value) - bound * integrated_count) * scale) ** 2
                    for value, bound in zip(sums, attempt.proof_noise)) ** 0.5
        upper = sum(((abs(value) + bound * integrated_count) * scale) ** 2
                    for value, bound in zip(sums, attempt.proof_noise)) ** 0.5
        return lower, upper

    def _append_motion_history(self, attempt, raw, count, start_clock, interval):
        """Keep sample-resolution prefixes; the serial thread never queries steps."""
        if attempt.motion_seen or attempt.moving or attempt.proof_interval_peak >= 0.5 * attempt.effective_speed:
            return True
        with self.lock:
            if attempt.done or attempt is not self.attempt:
                return True
            needed = count + (not attempt.proof_history)
            if len(attempt.proof_history) + needed > NATIVE_PROOF_HISTORY_SAMPLES:
                attempt.proof_overflow = True
                return False
            history = attempt.proof_history
            prefix = attempt.proof_prefix
            number = attempt.proof_history_count
            if not history:
                first = self.clock_to_time(start_clock) - attempt.sample_dt
                history.append((first, number, prefix))
            for position in range(count):
                offset = position * 3
                prefix = tuple(prefix[axis] + raw[offset + axis] - attempt.bases[axis]
                               for axis in range(3))
                number += 1
                when = self.clock_to_time(start_clock + int(round(position * interval)))
                history.append((when, number, prefix))
            attempt.proof_prefix, attempt.proof_history_count = prefix, number
        return True

    def _find_step_epoch(self, attempt, measured_at):
        """Freeze the first actual step-history transition, on the reactor only."""
        if attempt.proof_step_epoch is not None or attempt.proof_step_error is not None:
            return
        if not attempt.proof_step_positions:
            return
        latest = min(measured_at, self.main_mcu.estimated_print_time(self.reactor.monotonic()))
        epochs = []
        try:
            for stepper, initial in zip(attempt.steppers, attempt.proof_step_positions):
                if stepper.get_past_mcu_position(latest) == initial:
                    continue
                low, high = attempt.arm_time, latest
                while high - low > STEP_EPOCH_PRECISION:
                    middle = 0.5 * (low + high)
                    if stepper.get_past_mcu_position(middle) == initial:
                        low = middle
                    else:
                        high = middle
                epochs.append(high)
        except Exception as exc:
            attempt.proof_step_error = type(exc).__name__
            return
        if epochs:
            attempt.proof_step_epoch = min(epochs)

    def _scan_step_motion(self, attempt, measured_at, eventtime):
        """Additional startup evidence, bounded by first steps and the original deadline.

        Prefix differences remove stationary arm-to-motion dwell without
        treating an arbitrary later ringing interval as a new startup.
        Every lower bound uses one complete XYZ interval and its own count.
        """
        if attempt.proof_scan_event == eventtime:
            return
        attempt.proof_scan_event = eventtime
        self._find_step_epoch(attempt, measured_at)
        epoch = attempt.proof_step_epoch
        if (epoch is None or attempt.proof_step_error is not None or attempt.proof_overflow
                or attempt.proof_interval_peak >= 0.5 * attempt.effective_speed):
            return
        with self.lock:
            history = tuple(attempt.proof_history)
        start_min = max(attempt.arm_time, epoch - MOTION_LAG)
        start_max = epoch + MOTION_LAG
        end_max = min(attempt.motion_deadline, epoch + attempt.effective_speed / attempt.effective_accel
                      + MOTION_LAG + attempt.sample_dt)
        origins = [entry for entry in history if start_min <= entry[0] <= start_max]
        budget = NATIVE_PROOF_SCAN_BUDGET
        end_index, start_index = attempt.proof_scan_end, attempt.proof_scan_start
        scale = attempt.count_g * MM_PER_S2_PER_G * attempt.sample_dt
        bounds = attempt.proof_noise
        while end_index < len(history) and budget:
            end_time, end_count, end_sums = history[end_index]
            if end_time > end_max:
                break
            if end_time <= start_min:
                end_index, start_index = end_index + 1, 0
                continue
            while start_index < len(origins) and budget:
                start_time, start_count, start_sums = origins[start_index]
                if start_time >= end_time:
                    break
                number = end_count - start_count
                lower_squared = 0.0
                for axis in range(3):
                    lower = max(0.0, abs(end_sums[axis] - start_sums[axis]) - bounds[axis] * number)
                    lower_squared += (lower * scale) ** 2
                lower = lower_squared ** 0.5
                budget -= 1
                attempt.proof_scan_comparisons += 1
                start_index += 1
                if lower > attempt.proof_interval_peak:
                    attempt.proof_interval_peak = lower
                    attempt.proof_interval = (start_time, end_time, number)
                if lower >= 0.5 * attempt.effective_speed:
                    attempt.proof_scan_end, attempt.proof_scan_start = end_index, start_index
                    return
            if not budget:
                break
            end_index, start_index = end_index + 1, 0
        attempt.proof_scan_end, attempt.proof_scan_start = end_index, start_index

    def _motion_state(self, attempt, eventtime=None):
        """True when the steps run but the toolhead does not move, False once it
        moves with them, None while that cannot be told yet."""
        if attempt.moving:
            return False
        if (attempt.last_end_clock is None or attempt.gap
                or (attempt.motion_deadline is not None
                    and (attempt.proof_overflow or attempt.proof_step_error is not None))):
            # Missing samples leave the integrated speed too low, so a moving
            # toolhead could look pressed. Only an impact can end such a move.
            return None
        proof_state = attempt.proof_state
        if attempt.proof_noise is not None:
            if proof_state is None:
                return None
            _sums, _integrated_count, _dt, end_clock = proof_state
            measured_at = self.clock_to_time(end_clock)
            measured, measured_upper = self._motion_speed_bounds(attempt, proof_state)
            attempt.proof_raw = self._measured_speed(attempt, proof_state)
            attempt.proof_lower, attempt.proof_upper = measured, measured_upper
            attempt.proof_measured_at = measured_at
            if self.backend is None:
                self._scan_step_motion(attempt, measured_at,
                                       self.reactor.monotonic() if eventtime is None else eventtime)
                if attempt.proof_step_error is not None or attempt.proof_overflow:
                    return None
        else:
            measured_at = self.clock_to_time(attempt.last_end_clock)
            measured = self._measured_speed(attempt)
            measured_upper = measured
            if self.backend is not None:
                attempt.proof_raw = attempt.proof_lower = attempt.proof_upper = measured
                attempt.proof_measured_at = measured_at
        commanded = self._commanded_speed(attempt, measured_at - MOTION_LAG)
        # Workflow proof must also work when the configured rail or toolhead
        # velocity limit is below the legacy pressed-start cutoff.  A slightly
        # wider step-history window avoids single-step quantization at very low
        # speeds.  This does not alter Beacon's pressed-start decision below.
        effective = attempt.effective_speed
        if effective is None:
            effective = self.toolhead.get_max_velocity()[0] if self.backend is not None else PRESSED_MIN_SPEED
        proof_min_speed = min(PRESSED_MIN_SPEED, 0.5 * effective)
        window = motion_proof_window(effective)
        proof_commanded = (commanded if window == COMMAND_WINDOW else
                           self._commanded_speed(attempt, measured_at - MOTION_LAG, window))
        # Preserve valid early motion evidence across watchdog/report phases.
        # Unlike the instantaneous test, a historical lower bound must exceed
        # half the full effective speed, never half a later smaller command.
        full_arm_peak = (attempt.proof_noise is not None and effective > 0.0
                         and attempt.proof_speed_peak >= MOVING_RATIO * effective)
        native_interval = (self.backend is None and attempt.proof_noise is not None
                           and attempt.proof_interval_peak >= MOVING_RATIO * effective)
        if self.backend is not None or attempt.proof_noise is not None:
            attempt.proof_commanded = proof_commanded
            attempt.proof_command_eligible = proof_commanded >= proof_min_speed
        if (proof_commanded >= proof_min_speed
                and (measured >= MOVING_RATIO * proof_commanded or full_arm_peak or native_interval)):
            attempt.motion_seen = True
            if attempt.proof_noise is not None:
                attempt.proof_method = ("step_epoch_interval" if native_interval else
                                        "full_arm_peak" if full_arm_peak else "full_arm_lower")
        if self.backend is not None:
            # A generic stream may prove motion at its actual homing speed,
            # but missing motion is never a successful pressed-start home.
            if attempt.motion_seen:
                attempt.moving = True
                return False
            return None
        if commanded < PRESSED_MIN_SPEED:
            return None
        if measured >= MOVING_RATIO * commanded or full_arm_peak or native_interval:
            attempt.moving = True
            return False
        # An uncertain lower bound cannot establish a successful pressed home.
        return True if measured_upper < PRESSED_RATIO * commanded else None

    def _motion_startup_plan(self, attempt):
        speed, accel = self.toolhead.get_max_velocity()
        speed = attempt.effective_speed or speed
        accel = attempt.effective_accel or (min(accel, self.homing_accel)
                                           if self.homing_accel else accel)
        timeout = (motion_startup_timeout(speed, accel, self.stream_timeout, EARLY_INTERVAL)
                   if self.backend is None else self.backend.motion_startup_timeout(speed, accel))
        return speed, accel, timeout

    def _motion_proof_deadline(self, attempt):
        if attempt.motion_deadline is not None:
            return attempt.motion_deadline
        if attempt.proof_deadline is not None:
            return attempt.proof_deadline
        # Standalone replay attempts do not pass through prepare().
        return attempt.arm_time + self._motion_startup_plan(attempt)[2]

    def _check_no_motion(self, attempt, print_time, eventtime=None):
        """Fail-closed startup check for configured or expanded native references.

        The reporting/lag allowance leaves the complete acceleration ramp in
        fresh samples before proof is required.  If proof is still absent,
        stop with an error instead of pushing until the full-travel watchdog.
        """
        self._motion_state(attempt, eventtime)
        if attempt.motion_seen or attempt.moving:
            return False
        deadline = self._motion_proof_deadline(attempt)
        if print_time < deadline:
            return False
        self._fire(attempt, "no_motion", mcu.MCU_trsync.REASON_HOST_REQUEST)
        return True

    def _fire(self, attempt, reason, code):
        with self.lock:
            # A move that already ended must never stop the next one.
            if attempt.done or attempt is not self.attempt:
                return
            attempt.done = True
            attempt.reason = reason
            attempt.endstop.trigger(code)
        if attempt.motion_deadline is not None or self.backend is not None:
            # Diagnostics follow dispatch and cannot prevent the physical stop.
            try:
                attempt.request_time = self.main_mcu.estimated_print_time(self.reactor.monotonic())
            except Exception:
                attempt.request_time = None

    def _watchdog(self, eventtime):
        """Data-independent safety net: stops the move if the stream dies or travel runs out."""
        attempt = self.attempt
        if attempt is None or attempt.done:
            return self.reactor.NEVER
        print_time = self.main_mcu.estimated_print_time(eventtime)
        if print_time < attempt.arm_time:
            attempt.last_data = max(attempt.last_data, eventtime)
            return eventtime + WATCHDOG_INTERVAL
        if eventtime - attempt.last_data > self.stream_timeout:
            self._fire(attempt, "no_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return self.reactor.NEVER
        if self.backend is not None and self.backend.last_sample_time is not None:
            if print_time - self.backend.last_sample_time > self.backend.max_age:
                self._fire(attempt, "stale_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return self.reactor.NEVER
        if print_time > attempt.deadline + self.stream_timeout:
            self._fire(attempt, "travel", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return self.reactor.NEVER
        if attempt.vector_mode:
            if (self.backend is None and attempt.vector_unchanged_since is not None
                    and attempt.proof_sample_time is not None
                    and attempt.proof_sample_time - attempt.vector_unchanged_since > 0.025
                    and self._commanded_speed(attempt, attempt.proof_sample_time - MOTION_LAG) >= 40.0):
                self._fire(attempt, "frozen", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return self.reactor.NEVER
            return eventtime + WATCHDOG_INTERVAL
        if attempt.moving:
            return eventtime + WATCHDOG_INTERVAL
        if self.backend is not None:
            if self._check_no_motion(attempt, print_time, eventtime):
                return self.reactor.NEVER
            return eventtime + EARLY_INTERVAL
        if self._check_pressed(attempt, eventtime):
            return self.reactor.NEVER
        if attempt.proof_noise is not None and self._check_no_motion(attempt, print_time, eventtime):
            return self.reactor.NEVER
        return eventtime + EARLY_INTERVAL

    def _check_pressed(self, attempt, eventtime=None):
        """Pressed against the stop: the steps run but the toolhead does not
        move. Stops the move after two checks on different data."""
        if attempt.checked_clock == attempt.last_end_clock:
            return False
        attempt.checked_clock = attempt.last_end_clock
        if self._motion_state(attempt, eventtime) is not True:
            attempt.pressed_checks = 0
            return False
        attempt.pressed_checks += 1
        if attempt.pressed_checks < 2:
            return False
        if not attempt.varied:
            # A live sensor always shows some noise. Identical readings mean a
            # frozen stream, which would make a moving toolhead look pressed and
            # set the origin wherever the move started.
            self._fire(attempt, "frozen", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return True
        self._fire(attempt, "pressed", mcu.MCU_trsync.REASON_ENDSTOP_HIT)
        return True

    # Sample path: Beacon serial thread, every batch (about 400 per second).
    def _batch_hook(self, params):
        try:
            self._inspect(params)
        except Exception:
            logging.exception("beacon_accel_homing: batch inspection failed")
            self._sensor_fault("invalid_data")
        if self.helper._stream_en:
            self.saved_handler(params)

    def _inspect(self, params):
        try:
            self._inspect_batch(params)
        finally:
            # Optional raw references are retained only after detector/trigger
            # processing.  Conversion for charts is deferred to stop_capture.
            if self.capture_batches is not None:
                self._capture_batch(params)

    def _sensor_fault(self, reason):
        self.sensor_fault = reason
        attempt = self.attempt
        if attempt is not None and not attempt.done:
            self._fire(attempt, reason, mcu.MCU_trsync.REASON_HOST_REQUEST)
        # Stop dispatch keeps its original priority. Diagnostics are optional
        # and use snapshots because main-thread cleanup may replace the owner.
        state, token = self.beta_capture, self.beta_motion_capture
        try:
            if state is not None:
                quality = state["quality"]
                quality["fault"] = quality["fault"] or reason
            if token is not None:
                quality = token["window"]["beta_quality"]
                quality["fault"] = quality["fault"] or reason
        except Exception:
            logging.exception("accel_homing: beta fault diagnostics could not be retained")

    def _record_sample_timing_fault(self, phase, sensor_now, oldest, newest,
                                    cutoff=None, reason=None, sample_time=None, max_age=None):
        """Retain the first rejected timestamp separately from prior baseline data."""
        if self.sample_timing_fault is not None:
            return
        if max_age is None:
            max_age = self._reference_max_age()
        if reason is None:
            reason = "too_old" if sensor_now - oldest > max_age else "future"
        if sample_time is None:
            sample_time = oldest if reason == "too_old" else newest
        window = self.noise_window
        context = ("beta_motion_noise" if self.beta_motion_capture is not None else
                   "stationary_noise" if self.noise_session is not None else
                   "baseline" if self.preparation is not None else
                   "homing" if self.attempt is not None else "stream")
        self.sample_timing_fault = {
            "phase": phase, "context": context, "reason": reason,
            "window": None if window is None else window.get("index"),
            "sensor_now_s": sensor_now, "cutoff_time_s": cutoff,
            "oldest_sample_time_s": oldest, "newest_sample_time_s": newest,
            "rejected_sample_time_s": sample_time,
            "sample_age_ms": round((sensor_now - sample_time) * 1000.0, 3),
            "oldest_batch_age_ms": round((sensor_now - oldest) * 1000.0, 3),
            "newest_batch_age_ms": round((sensor_now - newest) * 1000.0, 3),
            "max_sample_age_ms": max_age * 1000.0,
            "max_future_ms": 5.0,
        }
        logging.info("accel_homing: sample timing fault %s",
                     json.dumps(self.sample_timing_fault, sort_keys=True))

    def _inspect_batch(self, params):
        if self.noise_session is not None:
            self._inspect_noise_batch(params)
            return
        data = params["data"]
        count = sample_count(data)
        if count < 2:
            return
        now = params.get("#receive_time") or self.reactor.monotonic()
        self.last_batch = now
        attempt = self.attempt
        if attempt is None:
            if self.tail is not None:
                self._track_impact(self.tail, params, data, count)
            if self.homing_active:
                if self.preparation is not None:
                    reason = self._collect_reference(params, data, count, now)
                    if reason is not None:
                        self._sensor_fault(reason)
                self.recent.append(data)
            return
        attempt.last_data = now
        if attempt.done:
            self._track_impact(attempt, params, data, count)
            return
        helper = self.helper
        if (self.backend is None and self.stream_alone and helper._stream_en
                and helper._scale.get("id") != self.session_scale.get("id")):
            self._fire(attempt, "scale", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return
        start_clock = params["start_clock"]
        interval = params["delta_clock"] / (count - 1.0)
        source_params, source_skip = params, 0
        expanded_native = self.backend is None and attempt.proof_noise is not None
        strict_native = self.backend is None and (expanded_native or attempt.vector_mode)
        if strict_native and (not math.isfinite(interval) or interval <= 0.0):
            self._fire(attempt, "invalid_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return
        if not clock_after(start_clock, attempt.arm_clock):
            # A delivered batch can contain both a pre-enable transient and
            # the first real impact. Preserve every sample at/after arm,
            # including a single remaining sample, instead of dropping it.
            skip = int(math.ceil(((attempt.arm_clock - start_clock) & 0xFFFFFFFF) / interval))
            if skip >= count:
                return
            width = 3 if isinstance(data, tuple) else 6
            data = data[skip * width:]
            count -= skip
            source_skip = skip
            start_clock = (start_clock + int(round(skip * interval))) & 0xFFFFFFFF
            params = dict(params, data=data, start_clock=start_clock,
                          delta_clock=int(round((count - 1) * interval)))
        if strict_native:
            start_time = self.clock_to_time(start_clock)
            end_time = self.clock_to_time(start_clock + params["delta_clock"])
            sensor_now = self.main_mcu.estimated_print_time(now)
            if not all(math.isfinite(value) for value in (start_time, end_time, sensor_now)):
                self._fire(attempt, "invalid_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return
            if (sensor_now - start_time > self.stream_timeout
                    or end_time - sensor_now > 0.005):
                self._fire(attempt, "stale_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return
            advance = (None if attempt.last_end_clock is None else
                       (start_clock - attempt.last_end_clock) & 0xFFFFFFFF)
            if advance is not None and not 0 < advance < 0x80000000:
                self._fire(attempt, "invalid_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return
        if attempt.first_clock is None:
            if attempt.bases is None:
                # Standalone detector replay uses no homing lifecycle. The
                # production prepare() path always supplies a validated one.
                attempt.bases = self._baseline()
            attempt.first_clock = start_clock
            attempt.sample_dt = interval / self.beacon_freq
            if strict_native or self.backend is not None:
                attempt.proof_first_sample_time = self.clock_to_time(start_clock)
            if attempt.bases is None:
                self._fire(attempt, "no_baseline", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return
            attempt.baseline = attempt.bases[attempt.index]
            attempt.last_samples = list(attempt.bases)
            if ((start_clock - attempt.arm_clock) & 0xFFFFFFFF
                    > params["delta_clock"] + GAP_SAMPLES * interval):
                attempt.gap = True
        elif (attempt.last_end_clock is not None
              and (start_clock - attempt.last_end_clock) & 0xFFFFFFFF > GAP_SAMPLES * interval):
            attempt.gap = True
        if clock_after(start_clock, attempt.deadline_clock):
            self._fire(attempt, "travel", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return
        data = sample_slice(data, count)
        raw = (data if isinstance(data, tuple) else memoryview(data).cast("h") if NATIVE_LE
               else [v for sample in SAMPLE.iter_unpack(data) for v in sample])
        if strict_native and not all(math.isfinite(value) for value in raw):
            self._fire(attempt, "invalid_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return
        if self.backend is None and CLIPPED in raw:
            raw = decode_clips(raw, attempt.last_samples, self.clip_values)
        if attempt.vector_mode:
            if attempt.gap:
                self._fire(attempt, "sample_gap", mcu.MCU_trsync.REASON_HOST_REQUEST)
                return
            self._inspect_vector(attempt, params, raw, count, source_params, source_skip)
            return
        values = raw[attempt.index::3]
        top, bottom = max(values), min(values)
        if top != bottom:
            attempt.varied = True
        base = attempt.baseline
        above, below = top - base, base - bottom
        pos, neg = (above, below) if attempt.map_sign > 0 else (below, above)
        direction = attempt.direction or attempt.auto_direction
        level = pos if direction > 0 else neg if direction < 0 else max(pos, neg)
        if level <= attempt.limit_raw:
            # Quiet batch: keep the cheap statistics only. Noise peaks count one
            # batch late, so the rising edge right before an impact is not noise.
            pending_pos, pending_neg, pending_level = attempt.pending_noise
            if pending_pos > attempt.noise_pos:
                attempt.noise_pos = pending_pos
            if pending_neg > attempt.noise_neg:
                attempt.noise_neg = pending_neg
            if pending_level > attempt.noise_checked:
                attempt.noise_checked = pending_level
            attempt.pending_noise = (pos, neg, level)
            sums, bases = attempt.sums, attempt.bases
            # One new list, so the watchdog never reads a half-updated set.
            sums = attempt.sums = [sums[0] + sum(raw[0::3]) - bases[0] * count,
                                   sums[1] + sum(raw[1::3]) - bases[1] * count,
                                   sums[2] + sum(raw[2::3]) - bases[2] * count]
            end_clock = start_clock + params["delta_clock"]
            if attempt.proof_noise is not None:
                previous = attempt.proof_state
                integrated_count = count + (0 if previous is None else previous[1])
                # Serial-reader and watchdog threads see one coherent sample
                # count and sums. N*dt includes the first integrated sample.
                attempt.proof_state = (tuple(sums), integrated_count, attempt.sample_dt, end_clock)
                attempt.proof_sample_time = self.clock_to_time(end_clock)
                peak_deadline = (attempt.motion_deadline if expanded_native else attempt.proof_deadline)
                if (not attempt.gap and peak_deadline is not None
                        and attempt.proof_sample_time <= peak_deadline):
                    lower, _upper = self._motion_speed_bounds(attempt, attempt.proof_state)
                    if lower > attempt.proof_speed_peak:
                        attempt.proof_speed_peak = lower
                        attempt.proof_speed_peak_time = attempt.proof_sample_time
                if expanded_native and not self._append_motion_history(attempt, raw, count, start_clock, interval):
                    self._fire(attempt, "proof_overflow", mcu.MCU_trsync.REASON_HOST_REQUEST)
                    return
            attempt.last_end_clock = end_clock
            if self.backend is not None and attempt.proof_noise is None:
                attempt.proof_sample_time = self.clock_to_time(end_clock)
            attempt.last_samples = [raw[-3], raw[-2], raw[-1]]
            if not direction:
                # The stop decelerates the toolhead, so once it travels the
                # impact can only point against the travel. A start-of-move
                # jolt in the travel direction is then ignored.
                speed = (sums[attempt.index] * attempt.map_sign * attempt.count_g
                         * MM_PER_S2_PER_G * attempt.sample_dt)
                if attempt.proof_noise is not None:
                    uncertainty = (attempt.proof_noise[attempt.index] * integrated_count
                                   * attempt.count_g * MM_PER_S2_PER_G * attempt.sample_dt)
                    if speed - uncertainty >= AUTO_DIRECTION_SPEED:
                        attempt.auto_direction = -1
                    elif speed + uncertainty <= -AUTO_DIRECTION_SPEED:
                        attempt.auto_direction = 1
                elif abs(speed) >= AUTO_DIRECTION_SPEED:
                    attempt.auto_direction = -1 if speed > 0 else 1
            return
        self._onset(attempt, params, values, count, source_params, source_skip)

    def _inspect_vector(self, attempt, params, raw, count, source_params, source_skip):
        """XYZ acceleration magnitude about a fixed resting offset, in raw units."""
        if not all(math.isfinite(value) for value in raw):
            self._fire(attempt, "invalid_data", mcu.MCU_trsync.REASON_HOST_REQUEST)
            return
        limit2 = attempt.limit_raw * attempt.limit_raw
        peak = 0.0
        previous = tuple(attempt.last_samples)
        for position in range(count):
            xyz = tuple(raw[3 * position:3 * position + 3])
            if xyz != previous or attempt.vector_unchanged_since is None:
                attempt.varied = attempt.varied or xyz != previous
                interval = params["delta_clock"] / max(1, count - 1)
                attempt.vector_unchanged_since = self.clock_to_time(
                    params["start_clock"] + int(round(position * interval)))
            previous = xyz
            level2 = sum((value - base) ** 2 for value, base in zip(xyz, attempt.bases))
            if level2 >= limit2:
                attempt.onset_batch = (params["start_clock"], params["delta_clock"], position, count)
                attempt.onset_sample = (source_params["start_clock"], source_params["delta_clock"],
                                        position + source_skip, sample_count(source_params["data"]))
                attempt.onset_raw = math.sqrt(level2)
                interval = params["delta_clock"] / max(1, count - 1)
                onset_clock = params["start_clock"] + int(round(position * interval))
                attempt.impact_until = (onset_clock + self.impact_window) & 0xFFFFFFFF
                self._fire(attempt, "impact", mcu.MCU_trsync.REASON_ENDSTOP_HIT)
                self._track_impact(attempt, params, raw, count)
                return
            peak = max(peak, math.sqrt(level2))
        attempt.noise_checked = max(attempt.noise_checked, peak)
        attempt.noise_pos = max(attempt.noise_pos, peak)
        attempt.last_end_clock = params["start_clock"] + params["delta_clock"]
        attempt.proof_sample_time = self.clock_to_time(attempt.last_end_clock)
        attempt.last_samples = list(previous)

    def _onset(self, attempt, params, values, count, source_params=None, source_skip=0):
        base, map_sign = attempt.baseline, attempt.map_sign
        direction = attempt.direction or attempt.auto_direction
        # Clips are already decoded, so they pass the same direction check.
        for position, raw in enumerate(values):
            value = (raw - base) * map_sign
            if (value * direction if direction else abs(value)) > attempt.limit_raw:
                break
        else:
            return
        start_clock = params["start_clock"]
        attempt.onset_batch = (start_clock, params["delta_clock"], position, count)
        # Constant-size sample identity only; conversion and capture inspection
        # remain after the trigger. Arm trimming must refer to the original
        # packet retained by _inspect's finally block, including its last row.
        source = params if source_params is None else source_params
        attempt.onset_sample = (source["start_clock"], source["delta_clock"],
                                position + source_skip, sample_count(source["data"]))
        attempt.onset_raw = value
        attempt.impact_until = (start_clock + self.impact_window) & 0xFFFFFFFF
        self._fire(attempt, "impact", mcu.MCU_trsync.REASON_ENDSTOP_HIT)
        self._track_impact(attempt, params, None, count, values)

    def _track_impact(self, attempt, params, data, count, values=None):
        """Largest reading shortly after the onset, for status."""
        if attempt.impact_until is None or not clock_after(attempt.impact_until, params["start_clock"]):
            return
        if attempt.vector_mode:
            raw = (data if isinstance(data, (tuple, list, memoryview)) else
                   [v for row in SAMPLE.iter_unpack(sample_slice(data, count)) for v in row])
            if self.backend is None and CLIPPED in raw:
                raw = decode_clips(raw, attempt.last_samples, self.clip_values)
            peak = max(math.sqrt(sum((raw[n + axis] - attempt.bases[axis]) ** 2
                                     for axis in range(3))) for n in range(0, count * 3, 3))
            if peak > attempt.impact_peak:
                attempt.impact_peak = peak
                if attempt.record is not None:
                    attempt.record["impact_peak_g"] = round(peak * attempt.count_g, 2)
            return
        if values is None:
            values = axis_values(sample_slice(data, count), attempt.index)
        top, bottom = max(values), min(values)
        peak = (self.full_scale_g / attempt.count_g
                if self.backend is None and top == CLIPPED and self.full_scale_g
                else max(top - attempt.baseline, attempt.baseline - bottom))
        if peak > attempt.impact_peak:
            attempt.impact_peak = peak
            if attempt.record is not None:
                attempt.record["impact_peak_g"] = round(peak * attempt.count_g, 2)

    def _baseline(self):
        totals, counts = [0, 0, 0], [0, 0, 0]
        for data in list(self.recent):
            for index in range(3):
                values = [v for v in axis_values(sample_slice(data, sample_count(data)), index)
                          if self.backend is not None or v != CLIPPED]
                totals[index] += sum(values)
                counts[index] += len(values)
        if min(counts) < BASELINE_MIN_SAMPLES:
            return None
        return tuple(t / c for t, c in zip(totals, counts))

    def onset_time(self, attempt):
        start_clock, delta_clock, position, count = attempt.onset_batch
        start = self.clock_to_time(start_clock)
        end = self.clock_to_time(start_clock + delta_clock)
        return start + (end - start) * position / max(count - 1, 1)

    def _velocity(self, attempt):
        """Speed along the homing axis before the impact, integrated from the
        accelerometer (machine frame of Beacon's axes map)."""
        if attempt.vector_mode or attempt.last_end_clock is None:
            return None
        return (attempt.sums[attempt.index] * attempt.map_sign * attempt.count_g
                * MM_PER_S2_PER_G * attempt.sample_dt)

    # Reporting
    def noise_sensor_context(self):
        """Stable, serializable sensor identity for a transient noise seed."""
        if self.helper is None or self.toolhead is None or self.main_mcu is None:
            raise self.printer.command_error("accel_homing: accelerometer is not connected")
        mapping = ([(0, 1), (1, 1), (2, 1)] if self.backend is not None
                   else self.helper.config.axes_map)
        if (not isinstance(mapping, (tuple, list)) or len(mapping) != 3
                or any(not isinstance(pair, (tuple, list)) or len(pair) != 2
                       or pair[0] not in (0, 1, 2) or pair[1] not in (-1, 1)
                       for pair in mapping)
                or len({pair[0] for pair in mapping}) != 3):
            raise self.printer.command_error("accel_homing: invalid accelerometer axes map")
        scale = self.scale
        count_g = scale["scale"] / 1000.0
        full_scale = self._full_scale_g(scale)
        if (not math.isfinite(count_g) or count_g <= 0.0
                or (full_scale is not None and
                    (not math.isfinite(full_scale) or full_scale <= 0.0))):
            raise self.printer.command_error("accel_homing: invalid accelerometer range")
        driver_map = (getattr(self.backend.sensor, "axes_map", None)
                      if self.backend is not None else mapping)
        context = {"sensor": self.sensor_name,
                   "backend": "beacon_raw" if self.backend is None else self.backend.kind,
                   "scale_id": scale.get("id"), "scale_name": scale.get("name"),
                   "count_g": count_g, "full_scale_g": full_scale,
                   "capture_axes_map": mapping, "sensor_axes_map": driver_map,
                   "sensor_rate_hz": (getattr(self.backend.sensor, "data_rate", None)
                                      if self.backend is not None else None)}
        try:
            return json.loads(json.dumps(context, allow_nan=False))
        except (TypeError, ValueError):
            raise self.printer.command_error("accel_homing: invalid accelerometer context")

    def _noise_shared_client(self):
        if self.backend is None:
            return bool(self.helper._stream_en)
        for name in ("clients", "client_cbs"):
            clients = getattr(self.helper, name, None)
            if isinstance(clients, dict):
                clients = list(clients)
            if isinstance(clients, (tuple, list, set)) and any(
                    not any(client is own for own in self.owned_sensor_clients)
                    and client is not self.backend.client and not getattr(client, "closed", False)
                    for client in clients):
                return True
        return False

    def start_noise_measurement(self):
        """Own only a stationary sensor subscription; never enable or move motors."""
        context = self.noise_sensor_context()
        if (self.homing_active or self.attempt is not None or self.stream_on
                or self.noise_session is not None or self.capture_batches is not None
                or (self.backend is not None and self.backend.active)):
            raise self.printer.command_error("accel_homing: sensor or homing capture is busy")
        if self._noise_shared_client():
            raise self.printer.command_error("accel_homing: noise measurement requires an exclusive sensor stream")
        session = {"context": context, "sensor": self.sensor, "helper": self.helper,
                   "helper_scale": dict(self.helper._scale) if self.backend is None else None,
                   "windows_started": 0}
        self.noise_session = session
        with self.lock:
            # A prior home's peak tail must not be extended by stationary data.
            self.tail = None
            self.recent.clear()
            self.preparation = None
        self.last_batch = None
        try:
            self._start_stream()
            if (self.session_scale != self.scale
                    or self.noise_sensor_context() != context):
                raise self.printer.command_error("accel_homing: sensor context changed while starting noise measurement")
        except Exception:
            self.stop_noise_measurement()
            raise
        return context

    def stop_noise_measurement(self):
        self.noise_session = None
        with self.lock:
            self.noise_window = None
        if self.helper is not None:
            self._stop_stream()

    def collect_noise_window(self, check_running, duration=1.0):
        """Collect one fresh bounded sensor-time window with no motion state writes."""
        session = self.noise_session
        if session is None or not math.isfinite(duration) or duration != 1.0:
            raise self.printer.command_error("accel_homing: noise measurement requires a one-second session")
        check_running()
        started = self.reactor.monotonic()
        session["windows_started"] += 1
        window = {"index": session["windows_started"],
                  "cutoff": self.main_mcu.estimated_print_time(started),
                  "duration": duration, "samples": [], "last": None,
                  "interval": None, "complete": False, "changed": False}
        with self.lock:
            self.noise_window = window
            if self.backend is not None:
                self.backend.prepare_reference()
        try:
            while True:
                check_running()
                now = self.reactor.monotonic()
                if (self.noise_session is not session or self.sensor is not session["sensor"]
                        or self.helper is not session["helper"]):
                    raise self.printer.command_error("accel_homing: noise measurement cancelled or disconnected")
                if (self._noise_shared_client() or self.noise_sensor_context() != session["context"]
                        or self.session_scale != self.scale
                        or (self.backend is None and self.helper._scale != session["helper_scale"])):
                    raise self.printer.command_error("accel_homing: sensor scale, mapping or client changed during noise measurement")
                if self.sensor_fault is not None:
                    raise self.printer.command_error("accel_homing: noise measurement: %s" % FAILURES[self.sensor_fault])
                with self.lock:
                    complete, last = window["complete"], window["last"]
                if complete:
                    if not window["changed"]:
                        raise self.printer.command_error("accel_homing: noise readings are identical; a live sensor cannot be established")
                    samples = window["samples"]
                    if len(samples) < BASELINE_MIN_SAMPLES:
                        raise self.printer.command_error("accel_homing: insufficient noise samples")
                    return {"samples": samples, "dropped": 0, "fault": None,
                            "cutoff_time_s": window["cutoff"],
                            "_noise_context": session["context"]}
                sensor_now = self.main_mcu.estimated_print_time(now)
                if last is not None and sensor_now - last > self.stream_timeout:
                    self._record_sample_timing_fault(
                        "noise_wait", sensor_now, last, last,
                        cutoff=window["cutoff"], reason="too_old", sample_time=last,
                        max_age=self.stream_timeout)
                    raise self.printer.command_error("accel_homing: noise measurement stopped receiving fresh samples")
                if now - started > duration + STREAM_START_TIMEOUT:
                    raise self.printer.command_error("accel_homing: noise measurement acquisition timeout")
                self.reactor.pause(now + 0.005)
        finally:
            with self.lock:
                if self.noise_window is window:
                    self.noise_window = None

    def _inspect_noise_batch(self, params):
        """Validate native and decoded stationary batches independently of homing."""
        window = self.noise_window
        if window is None or window["complete"] or self.sensor_fault is not None:
            return
        session = self.noise_session
        quality = window.get("beta_quality")
        if quality is not None:
            quality["packet_count"] += 1
        if (self._noise_shared_client() or self.noise_sensor_context() != session["context"]
                or self.session_scale != self.scale
                or self.sensor is not session["sensor"] or self.helper is not session["helper"]
                or (self.backend is None and self.helper._scale != session["helper_scale"])):
            self._sensor_fault("scale")
            return
        if any(params.get(key, 0) for key in ("errors", "overflows")):
            self._sensor_fault("data_error")
            return
        data = params["data"]
        width = 3 if isinstance(data, tuple) else 6
        count = sample_count(data)
        if count < 2 or len(data) % width:
            self._sensor_fault("invalid_data")
            return
        interval = params["delta_clock"] / (count - 1.0) / self.beacon_freq
        start = self.clock_to_time(params["start_clock"])
        minimum_rate = 800.0 if self.backend is None else self.backend.min_rate * 0.98
        if (not math.isfinite(start) or not math.isfinite(interval) or interval <= 0.0
                or 1.0 / interval < minimum_rate):
            self._sensor_fault("sample_rate")
            return
        now = self.reactor.monotonic()
        sensor_now = self.main_mcu.estimated_print_time(now)
        if quality is not None:
            self._update_beta_quality_times(quality, start, interval, count, sensor_now)
        max_age = self._reference_max_age()
        mapping = self.noise_session["context"]["capture_axes_map"]
        count_g = self.noise_session["context"]["count_g"]
        full_scale = self.noise_session["context"]["full_scale_g"]
        raw = [axis_values(sample_slice(data, count), index) for index in range(3)]
        with self.lock:
            if self.noise_window is not window:
                return
            for number in range(count):
                when = start + number * interval
                if when <= window["cutoff"]:
                    continue
                if not -0.005 <= sensor_now - when <= max_age:
                    self._record_sample_timing_fault(
                        "noise_batch", sensor_now, start, start + (count - 1) * interval,
                        cutoff=window["cutoff"],
                        reason="too_old" if sensor_now - when > max_age else "future",
                        sample_time=when)
                    self._sensor_fault("stale_data")
                    return
                if window["last"] is not None:
                    gap = when - window["last"]
                    if gap <= 0.0:
                        self._sensor_fault("invalid_data")
                        return
                    if (gap > 5.0 * interval + 0.000003
                            or (window["interval"] is not None
                                and abs(interval - window["interval"]) > max(0.000003, interval * 0.10))):
                        if quality is not None:
                            quality["gap_observations"] += 1
                        self._sensor_fault("sample_gap")
                        return
                values = tuple(axis[number] for axis in raw)
                if (not all(math.isfinite(value) for value in values)
                        or (self.backend is None and CLIPPED in values)
                        or (full_scale is not None
                            and any(abs(value * count_g) >= full_scale for value in values))):
                    if quality is not None:
                        quality["clipped_axis_values"] += sum(value == CLIPPED for value in values) if self.backend is None else 0
                        quality["saturation_axis_values"] += sum(
                            abs(value * count_g) >= full_scale for value in values) if full_scale is not None else 0
                    self._sensor_fault("invalid_data")
                    return
                if len(window["samples"]) >= self.capture_limit:
                    self._sensor_fault("proof_overflow")
                    return
                xyz = tuple(values[index] * sign * count_g for index, sign in mapping)
                if window["samples"] and xyz != window["samples"][0][1:]:
                    window["changed"] = True
                window["samples"].append((when,) + xyz)
                if quality is not None:
                    quality["sample_count"] += 1
                window["last"], window["interval"] = when, interval
                self.last_batch = now
                if (window["duration"] is not None
                        and when - window["samples"][0][0] + 0.000000001 >= window["duration"]):
                    window["complete"] = True
                    return

    @staticmethod
    def _new_beta_quality(context):
        return {"sensor_context": context, "packet_count": 0, "sample_count": 0,
                "min_sample_rate_hz": None, "max_sample_rate_hz": None,
                "maximum_packet_sample_age_s": None, "gap_observations": 0,
                "clipped_axis_values": 0, "saturation_axis_values": 0, "fault": None}

    @staticmethod
    def _update_beta_quality_times(quality, start, interval, count, sensor_now):
        if all(math.isfinite(value) for value in (start, interval, sensor_now)) and interval > 0:
            rate = 1.0 / interval
            quality["min_sample_rate_hz"] = min(quality["min_sample_rate_hz"] or rate, rate)
            quality["max_sample_rate_hz"] = max(quality["max_sample_rate_hz"] or rate, rate)
            quality["maximum_packet_sample_age_s"] = max(
                quality["maximum_packet_sample_age_s"] if quality["maximum_packet_sample_age_s"] is not None else
                sensor_now - start, sensor_now - start)

    def beta_capture_diagnostics(self):
        """Bounded current/last beta metadata for same-command console reports."""
        try:
            state, token = self.beta_capture, self.beta_motion_capture
            if state is not None:
                summary = {**state["quality"], "phase": "beta_home", "cancelled": state["cancelled"],
                           "first_sample_time_s": state["first_sample_time_s"],
                           "last_sample_time_s": state["last_sample_time_s"],
                           "stop_response": state["stop_response"], "capture_dropped": self.capture_dropped}
            elif token is not None:
                window = token["window"]
                summary = {**window["beta_quality"], "phase": "beta_motion",
                           "first_sample_time_s": window["samples"][0][0] if window["samples"] else None,
                           "last_sample_time_s": window["last"], "capture_dropped": 0,
                           "baseline_sample_count": len(token["baseline_samples"]),
                           "baseline_first_sample_time_s": token["baseline_samples"][0][0],
                           "baseline_last_sample_time_s": token["baseline_samples"][-1][0]}
            else:
                summary = self.beta_last_diagnostics
            return json.loads(json.dumps(summary, allow_nan=False))
        except Exception:
            return {"diagnostics_available": False}

    def start_beta_motion_capture(self, check_running):
        """Acquire a fresh offset, then an independent validated moving trace."""
        check_running()
        if self.beta_motion_capture is not None or self.beta_capture is not None:
            raise self.printer.command_error("accel_homing: beta capture is already active")
        self.beta_last_diagnostics = {"phase": "beta_motion_start"}
        context = self.start_noise_measurement()
        session = self.noise_session
        self.beta_last_diagnostics = {"phase": "beta_stationary_reference", "sensor_context": context}
        try:
            baseline = self.collect_noise_window(check_running, 1.0)
            check_running()
            rows = tuple(baseline["samples"])
            offset = tuple(math.fsum(row[index] for row in rows) / len(rows) for index in (1, 2, 3))
            cutoff = self.main_mcu.estimated_print_time(self.reactor.monotonic())
            window = {"index": session["windows_started"] + 1, "cutoff": cutoff,
                      "duration": None, "samples": [], "last": None, "interval": None,
                      "complete": False, "changed": False, "beta_quality": self._new_beta_quality(context)}
            token = {"session": session, "window": window, "sensor_context": context,
                     "generation": self.operation_generation, "baseline_samples": rows,
                     "baseline_g": offset, "capture_cutoff_time_s": cutoff,
                     "sample_rate_hz": (len(rows) - 1) / (rows[-1][0] - rows[0][0]),
                     "started": self.reactor.monotonic()}
            with self.lock:
                self.noise_window = window
                self.beta_motion_capture = token
            return token
        except Exception:
            self.beta_last_diagnostics.update(fault=self.sensor_fault,
                                              sample_timing_fault=self.sample_timing_fault)
            if self.noise_session is session:
                try:
                    self.stop_noise_measurement()
                except Exception:
                    logging.exception("accel_homing: beta reference stream cleanup failed")
            raise

    def _check_beta_motion_capture(self, token, check_running):
        check_running()
        session, window = token["session"], token["window"]
        if (self.beta_motion_capture is not token or self.noise_session is not session
                or self.noise_window is not window or self.operation_generation != token["generation"]
                or self._noise_shared_client() or self.noise_sensor_context() != token["sensor_context"]
                or self.session_scale != self.scale or self.sensor is not session["sensor"]
                or self.helper is not session["helper"]
                or (self.backend is None and self.helper._scale != session["helper_scale"])):
            raise self.printer.command_error("accel_homing: beta moving capture was interrupted or changed context")
        if self.sensor_fault is not None:
            raise self.printer.command_error("accel_homing: beta moving capture: %s" % FAILURES[self.sensor_fault])
        now = self.reactor.monotonic()
        last = window["last"]
        if ((last is None and now - token["started"] > STREAM_START_TIMEOUT)
                or (last is not None and self.main_mcu.estimated_print_time(now) - last > self.stream_timeout)):
            raise self.printer.command_error("accel_homing: beta moving capture stopped receiving fresh samples")

    def stop_beta_motion_capture(self, token, check_running):
        """Close only this beta subscription; return every validated XYZ row."""
        try:
            self._check_beta_motion_capture(token, check_running)
            window = token["window"]
            end = token.get("motion_end_time_s")
            if end is not None:
                if type(end) not in (int, float) or not math.isfinite(end):
                    raise self.printer.command_error("accel_homing: beta motion end time must be finite")
                deadline = self.reactor.monotonic() + STREAM_START_TIMEOUT
                while window["last"] is None or window["last"] < end:
                    self._check_beta_motion_capture(token, check_running)
                    now = self.reactor.monotonic()
                    if now >= deadline:
                        raise self.printer.command_error("accel_homing: beta moving capture does not cover motion end")
                    self.reactor.pause(now + 0.005)
            with self.lock:
                rows = tuple(window["samples"])
            if len(rows) < BASELINE_MIN_SAMPLES or not window["changed"]:
                raise self.printer.command_error("accel_homing: beta moving capture is insufficient or frozen")
            baseline = token["baseline_samples"]
            return {"samples": rows, "baseline_g": token["baseline_g"], "dropped": 0,
                    "fault": None, "sensor_context": token["sensor_context"],
                    "_noise_context": token["sensor_context"],
                    "baseline_first_sample_time_s": baseline[0][0],
                    "baseline_last_sample_time_s": baseline[-1][0],
                    "baseline_sample_count": len(baseline), "sample_rate_hz": token["sample_rate_hz"],
                    "capture_cutoff_time_s": token["capture_cutoff_time_s"],
                    "first_sample_time_s": rows[0][0], "last_sample_time_s": rows[-1][0],
                    "raw_samples_validated": True,
                    "beta_capture_diagnostics": self.beta_capture_diagnostics()}
        finally:
            if self.beta_motion_capture is token:
                self.beta_last_diagnostics = self.beta_capture_diagnostics()
                self.beta_motion_capture = None
                if self.noise_session is token["session"]:
                    failed = sys.exc_info()[0] is not None
                    try:
                        self.stop_noise_measurement()
                    except Exception:
                        if not failed:
                            raise
                        logging.exception("accel_homing: beta moving stream cleanup failed")

    def start_capture(self, beta=False):
        """Capture one requested workflow in memory; no streaming or motion."""
        beta_context = None
        if beta:
            if self._noise_shared_client() or self.beta_motion_capture is not None:
                raise self.printer.command_error("accel_homing: beta home requires an exclusive sensor capture")
            beta_context = self.noise_sensor_context()
        with self.lock:
            if self.capture_batches is not None:
                raise self.printer.command_error("accel_homing: capture already active")
            self.capture_batches = []
            self.capture_samples = self.capture_dropped = 0
            self.capture_baseline = None
            self.capture_attempt = None
            self.capture_attempts = 0
            if beta:
                self.beta_capture = {"context": beta_context, "generation": self.operation_generation,
                                     "sensor": self.sensor, "helper": self.helper,
                                     "helper_scale": dict(self.helper._scale) if self.backend is None else None,
                                     "first_sample_time_s": None, "last_sample_time_s": None,
                                     "interval_s": None, "fault": None, "cancelled": False,
                                     "tail_retained": False, "stop_response": None,
                                     "quality": self._new_beta_quality(beta_context)}

    def _inspect_beta_capture_batch(self, state, params):
        """Validate original raw rows after detection, before clip decoding."""
        reason = None
        quality = state["quality"]
        quality["packet_count"] += 1
        try:
            context = state["context"]
            if (state["cancelled"] or self.operation_generation != state["generation"]
                    or self.sensor is not state["sensor"] or self.helper is not state["helper"]
                    or self._noise_shared_client() or self.noise_sensor_context() != context
                    or self.session_scale != self.scale
                    or (self.backend is None and self.helper._scale != state["helper_scale"])):
                reason = "scale"
            elif any(params.get(key, 0) for key in ("errors", "overflows")):
                reason = "data_error"
            else:
                data = params["data"]
                count = sample_count(data)
                width = 3 if isinstance(data, tuple) else 6
                if count < 2 or len(data) % width:
                    reason = "invalid_data"
                else:
                    interval = params["delta_clock"] / (count - 1.0) / self.beacon_freq
                    start = self.clock_to_time(params["start_clock"])
                    now = self.main_mcu.estimated_print_time(self.reactor.monotonic())
                    self._update_beta_quality_times(quality, start, interval, count, now)
                    rate = 800.0 if self.backend is None else self.backend.min_rate * 0.98
                    if (not all(math.isfinite(value) for value in (start, interval, now))
                            or interval <= 0 or 1.0 / interval < rate):
                        reason = "sample_rate"
                    elif (now - start > self._reference_max_age()
                          or start + (count - 1) * interval - now > 0.005):
                        reason = "stale_data"
                    elif (state["last_sample_time_s"] is not None
                          and (start <= state["last_sample_time_s"]
                               or start - state["last_sample_time_s"] > 5 * interval + 0.000003
                               or abs(interval - state["interval_s"]) > max(0.000003, interval * 0.10))):
                        reason = "sample_gap"
                        quality["gap_observations"] += 1
                    else:
                        raw = [axis_values(sample_slice(data, count), index) for index in range(3)]
                        full_scale, count_g = context["full_scale_g"], context["count_g"]
                        quality["clipped_axis_values"] += sum(value == CLIPPED for channel in raw for value in channel) if self.backend is None else 0
                        quality["saturation_axis_values"] += sum(
                            abs(value * count_g) >= full_scale for channel in raw for value in channel) if full_scale is not None else 0
                        if any(not math.isfinite(value)
                               or (self.backend is None and value == CLIPPED)
                               or (full_scale is not None and abs(value * count_g) >= full_scale)
                               for channel in raw for value in channel):
                            reason = "invalid_data"
                        else:
                            if state["first_sample_time_s"] is None:
                                state["first_sample_time_s"] = start
                            state["last_sample_time_s"] = start + (count - 1) * interval
                            state["interval_s"] = interval
                            quality["sample_count"] += count
        except Exception:
            reason = "invalid_data"
        if reason is not None:
            state["fault"] = state["fault"] or reason
            quality["fault"] = quality["fault"] or reason
            # Never invoke _fire while holding the non-reentrant capture lock.
            if (self.beta_capture is state and self.capture_batches is not None
                    and self.operation_generation == state["generation"] and not state["cancelled"]):
                self._sensor_fault(reason)

    def finish_beta_home_capture(self, check_running):
        """Observe queue completion and collect a bounded diagnostic tail only."""
        state = self.beta_capture
        if state is None or self.capture_batches is None or not state["tail_retained"]:
            raise self.printer.command_error("accel_homing: no completed beta home capture for a response tail")
        boundary = self.main_mcu.estimated_print_time(self.reactor.monotonic())
        state["stop_response"] = {"boundary_source": "host_observed_queue_completion",
                                  "boundary_time_s": boundary, "window_s": IMPACT_WINDOW,
                                  "complete": False, "available": False}
        deadline = self.reactor.monotonic() + STREAM_START_TIMEOUT
        try:
            while True:
                check_running()
                if (self.beta_capture is not state or state["cancelled"]
                        or self.operation_generation != state["generation"]):
                    raise self.printer.command_error("accel_homing: beta response capture was interrupted")
                if state["fault"] or self.sensor_fault is not None:
                    raise self.printer.command_error("accel_homing: beta response capture has a sensor fault")
                last = state["last_sample_time_s"]
                if last is not None and last >= boundary + IMPACT_WINDOW:
                    state["stop_response"].update(available=True, complete=True)
                    break
                now = self.reactor.monotonic()
                if now >= deadline:
                    state["stop_response"].update(
                        available=last is not None and last >= boundary,
                        reason="bounded_tail_incomplete")
                    break
                self.reactor.pause(now + 0.005)
        except Exception:
            if self.beta_capture is state:
                try:
                    self._stop_stream()
                except Exception:
                    logging.exception("accel_homing: beta response stream cleanup failed")
            raise

    def _capture_batch(self, params):
        data = params["data"]
        count = sample_count(data)
        state = self.beta_capture
        if count < 2:
            if state is not None:
                state["fault"] = state["fault"] or "invalid_data"
                if (self.beta_capture is state and self.operation_generation == state["generation"]
                        and not state["cancelled"]):
                    self._sensor_fault("invalid_data")
            return
        if state is not None:
            self._inspect_beta_capture_batch(state, params)
        overflow = False
        with self.lock:
            if self.capture_batches is None:
                return
            if self.beta_capture is not state:
                return
            if self.capture_samples + count > self.capture_limit:
                self.capture_dropped += count
                overflow = True
                if state is not None:
                    state["fault"] = state["fault"] or "capture_overflow"
            else:
                self.capture_batches.append((params["start_clock"], params["delta_clock"],
                                             data, self.session_scale["scale"] / 1000.0))
                self.capture_samples += count
        if (overflow and state is not None and self.beta_capture is state
                and self.operation_generation == state["generation"] and not state["cancelled"]):
            self._sensor_fault("proof_overflow")

    def stop_capture(self):
        state = self.beta_capture
        if state is None:
            return self._stop_capture_data()
        try:
            captured = self._stop_capture_data()
            captured["beta_capture"] = {"version": 1, "sensor_context": state["context"],
                                        "raw_samples_validated": not state["fault"] and not state["cancelled"],
                                        "fault": state["fault"], "cancelled": state["cancelled"],
                                        "stop_response": state["stop_response"],
                                        "quality": self.beta_capture_diagnostics()}
            if state["fault"]:
                captured["fault"] = captured.get("fault") or state["fault"]
            return captured
        finally:
            if self.beta_capture is state:
                self.beta_last_diagnostics = self.beta_capture_diagnostics()
                self.beta_capture = None
                failed = sys.exc_info()[0] is not None
                try:
                    self._stop_stream()
                except Exception:
                    if not failed:
                        raise
                    logging.exception("accel_homing: beta home stream cleanup failed")

    def _stop_capture_data(self):
        """Convert bounded captured batches to machine-axis g off the trigger path.

        `samples` include gravity and bias; `baseline_g` must be subtracted when
        comparing channels to directional impact thresholds in charts.
        """
        with self.lock:
            batches, self.capture_batches = self.capture_batches or [], None
            dropped, baseline = self.capture_dropped, self.capture_baseline
            attempt, self.capture_attempt = self.capture_attempt, None
            attempt_count, self.capture_attempts = self.capture_attempts, 0
            fault = self.sensor_fault
        mapping = ([(0, 1), (1, 1), (2, 1)] if self.backend is not None
                   else self.helper.config.axes_map)
        detector = None
        onset_sample = None if attempt is None else attempt.onset_sample
        valid_onset_sample = (isinstance(onset_sample, (tuple, list)) and len(onset_sample) == 4
                              and all(isinstance(value, int) and not isinstance(value, bool)
                                      for value in onset_sample)
                              and 0 <= onset_sample[0] <= 0xFFFFFFFF
                              and 0 < onset_sample[1] <= 0xFFFFFFFF
                              and onset_sample[3] >= 2
                              and 0 <= onset_sample[2] < onset_sample[3])
        if attempt is not None and attempt.bases is not None:
            baseline = tuple(attempt.bases[index] * sign * attempt.count_g
                             for index, sign in mapping)
            detector = {"axis": attempt.axis, "arm_time_s": attempt.arm_time,
                        "onset_time_s": None,  # Filled only from a retained row below.
                        "trigger_onset_time_s": (attempt.record.get("onset_time_s")
                                                 if isinstance(attempt.record, dict) else None),
                        "onset_sample_version": 1,
                        "onset_sample": onset_sample,
                        "threshold_g": attempt.limit_raw * attempt.count_g,
                        "onset_g": (attempt.onset_raw * attempt.count_g
                                    if attempt.onset_batch is not None else None),
                        "impact_peak_g": (attempt.impact_peak * attempt.count_g
                                          if attempt.onset_batch is not None else None),
                        "direction": 0 if attempt.vector_mode else attempt.direction or attempt.auto_direction,
                        "impact_metric": "xyz_magnitude" if attempt.vector_mode else "selected_axis",
                        "homing_speed_mm_s": attempt.effective_speed,
                        "homing_accel_mm_s2": attempt.effective_accel,
                        "attempt_count": attempt_count}
        samples, previous = [], [0, 0, 0]
        onset_matches, onset_index = 0, None
        for start_clock, delta_clock, data, count_g in batches:
            count = sample_count(data)
            if valid_onset_sample and (start_clock, delta_clock, count) == (
                    onset_sample[0], onset_sample[1], onset_sample[3]):
                onset_matches += 1
                onset_index = len(samples) + onset_sample[2]
            raw = (list(data) if isinstance(data, tuple)
                   else [v for row in SAMPLE.iter_unpack(sample_slice(data, count)) for v in row])
            if self.backend is None and CLIPPED in raw:
                raw = decode_clips(raw, previous, self.clip_values)
            previous = raw[-3:]
            start, end = self.clock_to_time(start_clock), self.clock_to_time(start_clock + delta_clock)
            dt = (end - start) / (count - 1)
            for number in range(count):
                xyz = raw[number * 3:number * 3 + 3]
                samples.append((start + number * dt,) + tuple(
                    xyz[index] * sign * count_g for index, sign in mapping))
        if detector is not None:
            detector["onset_sample_matches"] = onset_matches
            detector["onset_sample_index"] = onset_index if onset_matches == 1 else None
            if onset_sample is not None and onset_matches == 1:
                # Use the actual exported row, avoiding both later MCU clock
                # adjustments and a different floating-point interpolation order.
                detector["onset_time_s"] = samples[onset_index][0]
        if baseline is None and samples:
            resting = samples[:min(50, len(samples))]
            baseline = tuple(sum(row[i] for row in resting) / len(resting) for i in (1, 2, 3))
        return {"samples": samples, "dropped": dropped, "baseline_g": baseline, "fault": fault,
                "detector": detector,
                # Used only for in-process provenance; workflows remove this
                # reference before exporting metadata or publishing status.
                "_record": None if attempt is None else attempt.record}

    def beta_home_telemetry(self, capture, record):
        """Exact beta diagnostics from retained rows; never alter legacy peaks."""
        finite = lambda value: type(value) in (int, float) and math.isfinite(value)
        beta = capture.get("beta_capture")
        detector = capture.get("detector")
        samples, baseline = capture.get("samples"), capture.get("baseline_g")
        if (not isinstance(beta, dict) or beta.get("raw_samples_validated") is not True
                or capture.get("fault") or capture.get("dropped", 0)
                or not isinstance(detector, dict) or detector.get("impact_metric") != "xyz_magnitude"
                or detector.get("axis") != record.get("axis") or detector.get("attempt_count") != 1
                or not isinstance(samples, (tuple, list)) or not 2 <= len(samples) <= self.capture_limit
                or not isinstance(baseline, (tuple, list)) or len(baseline) != 3
                or not all(finite(value) for value in baseline)):
            raise self.printer.command_error("accel_homing: beta event requires its complete raw XYZ capture")
        onset_index = detector.get("onset_sample_index")
        arm, threshold = detector.get("arm_time_s"), detector.get("threshold_g")
        if (type(onset_index) is not int or not 0 <= onset_index < len(samples)
                or detector.get("onset_sample_matches") != 1 or not finite(arm)
                or not finite(threshold) or threshold <= 0):
            raise self.printer.command_error("accel_homing: beta event lacks its exact first-crossing row")
        levels, deltas, previous = [], [], None
        for row in samples:
            if (not isinstance(row, (tuple, list)) or len(row) != 4
                    or not all(finite(value) for value in row)
                    or (previous is not None and row[0] <= previous)):
                raise self.printer.command_error("accel_homing: beta event has invalid raw sample times")
            previous = row[0]
            xyz = tuple(row[index + 1] - baseline[index] for index in range(3))
            level = math.sqrt(sum(value * value for value in xyz))
            if not finite(level):
                raise self.printer.command_error("accel_homing: beta event magnitude is not finite")
            deltas.append(xyz)
            levels.append(level)
        onset = samples[onset_index][0]
        if detector.get("onset_time_s") != onset or onset < arm:
            raise self.printer.command_error("accel_homing: beta first-crossing identity/time changed")
        tolerance = max(1e-12, threshold * 1e-12)
        if levels[onset_index] < threshold - tolerance:
            raise self.printer.command_error("accel_homing: beta first-crossing row is below its trigger")
        prefix = [index for index in range(onset_index) if samples[index][0] >= arm]
        if any(levels[index] > threshold + tolerance for index in prefix):
            raise self.printer.command_error("accel_homing: beta capture crossed its trigger before the selected row")

        def summary(indices):
            if not indices:
                return {"sample_count": 0, "first_sample_time_s": None, "last_sample_time_s": None,
                        "magnitude_peak_g": None, "peak_time_s": None, "axis_absolute_peak_g": None}
            peak = max(indices, key=lambda index: levels[index])
            return {"sample_count": len(indices), "first_sample_time_s": samples[indices[0]][0],
                    "last_sample_time_s": samples[indices[-1]][0],
                    "magnitude_peak_g": levels[peak], "peak_time_s": samples[peak][0],
                    "axis_absolute_peak_g": {axis: max(abs(deltas[index][channel]) for index in indices)
                                              for channel, axis in enumerate("xyz")}}

        noise = summary(prefix)
        noise.update(start_time_s=arm, end_time_s=onset, interval="[arm, first_crossing)")
        end = onset + IMPACT_WINDOW
        impact = summary([index for index in range(onset_index, len(samples)) if samples[index][0] <= end])
        impact.update(start_time_s=onset, end_time_s=end, interval="[first_crossing, window_end]",
                      complete=samples[-1][0] >= end)
        response = beta.get("stop_response")
        if (isinstance(response, dict) and response.get("boundary_source") == "host_observed_queue_completion"
                and finite(response.get("boundary_time_s")) and finite(response.get("window_s"))
                and response["window_s"] > 0):
            boundary, until = response["boundary_time_s"], response["boundary_time_s"] + response["window_s"]
            response = dict(response)
            response.update(summary([index for index, row in enumerate(samples) if boundary <= row[0] <= until]))
            response.update(start_time_s=boundary, end_time_s=until,
                            available=response["sample_count"] > 0,
                            complete=bool(response.get("complete") and samples[-1][0] >= until))
            if not response["available"]:
                response["reason"] = "no_samples_after_host_observed_queue_completion"
        else:
            response = {**summary([]), "available": False, "complete": False,
                        "boundary_source": "host_observed_queue_completion", "boundary_time_s": None,
                        "reason": "no_host_observed_queue_completion_boundary"}
        return {"version": 1, "clock_domain": "sensor_print_time_seconds", "impact_metric": "xyz_magnitude",
                "capture_diagnostics": beta.get("quality", {}),
                "pre_onset_noise": noise,
                "first_crossing": {"sample_index": onset_index, "time_s": onset,
                                   "magnitude_g": levels[onset_index], "xyz_delta_g": deltas[onset_index],
                                   "threshold_g": threshold},
                "onset_window": impact, "post_stop_response": response}

    def _record(self, attempt):
        to_g = attempt.count_g
        if attempt.reason != "impact":
            attempt.noise_pos = max(attempt.noise_pos, attempt.pending_noise[0])
            attempt.noise_neg = max(attempt.noise_neg, attempt.pending_noise[1])
            attempt.noise_checked = max(attempt.noise_checked, attempt.pending_noise[2])
        onset = attempt.onset_batch is not None
        recorded_onset = self.onset_time(attempt) if onset else None
        velocity = self._velocity(attempt)
        result = {
            "axis": attempt.axis,
            "pass": attempt.pass_no,
            "result": attempt.reason or "no_trigger",
            "threshold_g": round(attempt.limit_raw * to_g, 2),
            "impact_metric": "xyz_magnitude" if attempt.vector_mode else "selected_axis",
            "physical_position_verified": False,
            "direction": 0 if attempt.vector_mode else attempt.direction or attempt.auto_direction,
            "onset_g": round(attempt.onset_raw * to_g, 2) if onset else None,
            "impact_sign": (1 if attempt.onset_raw > 0 else -1 if attempt.onset_raw < 0 else None)
                           if onset else None,
            "impact_peak_g": round(attempt.impact_peak * to_g, 2) if onset else None,
            "noise_pos_g": round(attempt.noise_pos * to_g, 2),
            "noise_neg_g": round(attempt.noise_neg * to_g, 2),
            "noise_g": round(attempt.noise_checked * to_g, 2),
            "velocity_mm_s": None if velocity is None else round(velocity, 1),
            "moving": attempt.moving,
            "motion_seen": attempt.motion_seen or attempt.moving,
            "arm_time_s": attempt.arm_time,
            "onset_time_s": recorded_onset,
            "onset_sample": attempt.onset_sample,
            "start_position_mm": attempt.start_position,
            "expected_travel_mm": attempt.expected_travel,
            "backend": "beacon_raw" if self.backend is None else self.backend.kind,
            "trigger_after_arm_s": round(recorded_onset - attempt.arm_time, 4) if onset else None,
        }
        if attempt.motion_deadline is not None or self.backend is not None:
            result["motion_proof"] = self._motion_proof_status(attempt)
        attempt.record = result
        self.last[attempt.axis] = result
        self.history.append(result)

    def _motion_proof_status(self, attempt):
        """Constant-size diagnostics survive disarm, workflow errors and capture cleanup."""
        def number(value):
            return round(value, 6) if value is not None and math.isfinite(value) else None
        def elapsed(value):
            return number(None if value is None else (value - attempt.arm_time) * 1000.0)
        if attempt.vector_mode:
            return {"result": "not_required", "method": "commanded_travel",
                    "backend": "beacon_raw" if self.backend is None else self.backend.kind,
                    "command_eligible": False, "noise_bound_xyz_g": None,
                    "physical_position_verified": False}
        first, sample = attempt.proof_first_sample_time, attempt.proof_sample_time
        interval = attempt.proof_interval
        native = self.backend is None
        deadline = self._motion_proof_deadline(attempt)
        mapping = self.helper.config.axes_map if native else ((0, 1), (1, 1), (2, 1))
        noise = (None if attempt.proof_noise is None else
                 tuple(number(attempt.proof_noise[index] * attempt.count_g) for index, _sign in mapping))
        result = ("proved" if attempt.motion_seen or attempt.moving else
                  "gap" if attempt.gap else "history_overflow" if native and attempt.proof_overflow else
                  "step_history_error" if native and attempt.proof_step_error else
                  "no_first_step" if native and attempt.proof_step_epoch is None else
                  "no_samples" if sample is None else
                  "command_not_eligible" if not attempt.proof_command_eligible else "insufficient_bound")
        return {
            "result": result, "method": attempt.proof_method,
            "backend": "beacon_raw" if native else self.backend.kind,
            "gap": attempt.gap, "overflow": attempt.proof_overflow,
            "effective_speed_mm_s": number(attempt.effective_speed),
            "effective_accel_mm_s2": number(attempt.effective_accel),
            "noise_bound_xyz_g": noise,
            "arm_time_s": number(attempt.arm_time),
            "sample_time_s": number(sample),
            "evaluated_sample_time_s": number(attempt.proof_measured_at),
            "deadline_time_s": number(deadline),
            "request_time_s": number(attempt.request_time),
            "first_sample_after_arm_ms": elapsed(first),
            "first_step_after_arm_ms": elapsed(attempt.proof_step_epoch),
            "step_epoch_precision_ms": STEP_EPOCH_PRECISION * 1000.0 if native else None,
            "sample_after_arm_ms": elapsed(sample),
            "evaluated_sample_after_arm_ms": elapsed(attempt.proof_measured_at),
            "deadline_after_arm_ms": elapsed(deadline),
            "request_after_arm_ms": elapsed(attempt.request_time),
            "sample_age_at_request_ms": number(None if sample is None or attempt.request_time is None
                                                else (attempt.request_time - sample) * 1000.0),
            "raw_speed_mm_s": number(attempt.proof_raw),
            "lower_speed_mm_s": number(attempt.proof_lower),
            "upper_speed_mm_s": number(attempt.proof_upper),
            "full_arm_peak_mm_s": number(attempt.proof_speed_peak),
            "full_arm_peak_after_arm_ms": elapsed(attempt.proof_speed_peak_time),
            "step_epoch_peak_mm_s": number(attempt.proof_interval_peak) if native else None,
            "commanded_speed_mm_s": number(attempt.proof_commanded),
            "command_eligible": attempt.proof_command_eligible,
            "interval_start_after_arm_ms": elapsed(None if interval is None else interval[0]),
            "interval_end_after_arm_ms": elapsed(None if interval is None else interval[1]),
            "history_samples": attempt.proof_history_count if native else None,
            "history_capacity": NATIVE_PROOF_HISTORY_SAMPLES if native else None,
            "scan_comparisons": attempt.proof_scan_comparisons if native else None,
            "scan_budget_per_event": NATIVE_PROOF_SCAN_BUDGET if native else None,
            "step_history_error": attempt.proof_step_error,
        }

    def get_status(self, eventtime):
        return {
            "threshold_g": dict(self.threshold_g),
            "impact_sign": dict(self.impact_sign),
            "homing_accel": self.homing_accel,
            "back_off_dist": self.back_off_dist,
            "homing": self.homing_active,
            "baseline": (None if self.baseline_status is None else
                         {key: dict(value) if isinstance(value, dict) else value
                          for key, value in self.baseline_status.items()}),
            "sample_timing_fault": (None if self.sample_timing_fault is None else
                                    dict(self.sample_timing_fault)),
            "last": {axis: {key: dict(value) if isinstance(value, dict) else value
                            for key, value in record.items()} for axis, record in self.last.items()},
            "sensor": self.sensor_name,
            "backend": {"type": "beacon_raw", "sensor": self.sensor_name,
                        "latency_guaranteed": False} if self.backend is None else self.backend.get_status(),
            "workflows": self.workflows.get_status(eventtime),
        }

    def cmd_STATUS(self, gcmd):
        kind = "beacon_raw" if self.backend is None else self.backend.kind
        gcmd.respond_info("accel_homing: sensor %s, backend %s, X %.12g g, Y %.12g g"
                          % (self.sensor_name, kind, self.threshold_g["x"], self.threshold_g["y"]))
        gcmd.respond_info("accel_homing: baseline policy: legacy ordinary homing requires quiet XYZ;"
                          " simple calibration and saved XYZ-magnitude models require a fresh"
                          " %.2f ms / %d sample reference without a quiet-range limit;"
                          " no acceleration-derived motion proof for XYZ magnitude"
                          % (self.baseline_time * 1000.0, BASELINE_MIN_SAMPLES))
        if self.backend is not None:
            gcmd.respond_info("accel_homing: requested report interval %.4f s, sample-age limit"
                              " %.4f s, observed rate %s Hz, report-age speed ceiling %s;"
                              " total stop latency is not guaranteed"
                              % (self.backend.interval, self.backend.max_age, self.backend.observed_rate,
                                 "disabled" if self.backend.max_speed is None
                                 else "%.1f mm/s" % self.backend.max_speed))
        if self.backend is not None:
            gcmd.respond_info("accel_homing: max_report_travel %s, max_startup_travel %s;"
                              " optional commanded-travel limits, not physical stopping distances"
                              % ("disabled" if self.backend.report_travel is None else "%.3f mm" % self.backend.report_travel,
                                 "disabled" if self.backend.startup_travel is None else "%.3f mm" % self.backend.startup_travel))
        if self.baseline_status is not None:
            gcmd.respond_info("accel_homing: " + describe_reference(self.baseline_status))
        if self.sample_timing_fault is not None:
            gcmd.respond_info("accel_homing: sample timing fault " +
                              json.dumps(self.sample_timing_fault, sort_keys=True))
        if not self.history:
            gcmd.respond_info("beacon_accel_homing: no homes recorded since restart")
            return
        gcmd.respond_info("\n".join(describe(r) for r in self.history))
        for axis, record in sorted(self.last.items()):
            proof = record.get("motion_proof")
            if proof is not None:
                if proof.get("backend") != "beacon_raw":
                    gcmd.respond_info("accel_homing: %s %s" % (axis.upper(), describe_motion_proof(proof)))
                    continue
                gcmd.respond_info("accel_homing: %s motion proof %s; "
                                  "V/A %s/%s, first step %s ms, gap %s; "
                                  "lower/upper %s/%s, arm/step peak %s/%s mm/s; "
                                  "command %s eligible %s; request/deadline %s/%s ms; "
                                  "history %d/%d overflow %s" % (
                                      axis.upper(), proof["result"], proof["effective_speed_mm_s"],
                                      proof["effective_accel_mm_s2"], proof["first_step_after_arm_ms"],
                                      proof["gap"], proof["lower_speed_mm_s"], proof["upper_speed_mm_s"],
                                      proof["full_arm_peak_mm_s"], proof["step_epoch_peak_mm_s"],
                                      proof["commanded_speed_mm_s"], proof["command_eligible"],
                                      proof["request_after_arm_ms"], proof["deadline_after_arm_ms"],
                                      proof["history_samples"], proof["history_capacity"], proof["overflow"]))

    def cmd_SET(self, gcmd):
        self.workflows.check_settings_change()
        axis = gcmd.get("AXIS").lower()
        if axis not in AXES:
            raise gcmd.error("AXIS must be X or Y")
        threshold = gcmd.get_float("THRESHOLD", None)
        sign = gcmd.get_int("IMPACT_SIGN", None, minval=-1, maxval=1)
        if threshold is not None:
            scale = self.session_scale if self.stream_on else self.scale
            full_scale = self._full_scale_g(scale) if self.helper is not None else None
            validate_threshold(threshold, "THRESHOLD", gcmd.error, full_scale)
            self.threshold_g[axis] = threshold
        if sign is not None:
            self.impact_sign[axis] = sign
        gcmd.respond_info("beacon_accel_homing %s: threshold %.12g g, impact_sign %d (until restart)"
                          % (axis.upper(), self.threshold_g[axis], self.impact_sign[axis]))


def load_config(config):
    return BeaconAccelHoming(config)
