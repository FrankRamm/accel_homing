# Configured accelerometer adapters for accel_homing.
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Subscribe to decoded Klipper acceleration, never infer electrical wiring.

The bulk_sensor client contract and fields come from Klipper/Kalico.  Their
normal 100 ms reporting interval is intended for resonance measurements.  An
armed homing session requests a shorter interval and validates sensor time,
sample rate and error counters before passing a batch to the detector.  This
does not provide a real-time guarantee for Python, USB/CAN or MCU stopping.
"""

import math


GRAVITY = 9806.65
CLOCK_FREQ = 1000000.0
REPORT_TRAVEL_MIN = 0.1
REPORT_TRAVEL_MAX = 2.0
STARTUP_REQUEST_TRAVEL_LIMIT = None  # disabled unless explicitly configured
STARTUP_REQUEST_TRAVEL_MIN = 0.1
STARTUP_REQUEST_TRAVEL_MAX = 5.0


def motion_report_speed_limit(max_age, report_travel):
    # Report freshness limits commanded travel; startup proof independently
    # enforces its configured request budget at the actual speed/acceleration.
    return None if report_travel is None else report_travel / max_age


def motion_proof_window(speed):
    return max(0.004, min(0.050, 0.050 / speed))


def motion_startup_timeout(speed, accel, max_age, interval):
    """Pure policy shared by runtime and offline config installation."""
    return (speed / accel + 0.015 + max_age
            + max(0.010, 2.0 * interval, motion_proof_window(speed)))


def motion_startup_request_travel(speed, accel, max_age, interval):
    # The same trapezoid distance as speed * (timeout + .003) - v**2/(2a),
    # without subtracting two unbounded ramp terms at very small acceleration.
    margin = (0.018 + max_age
              + max(0.010, 2.0 * interval, motion_proof_window(speed)))
    return 0.5 * speed * (speed / accel) + speed * margin


SENSOR_TYPES = ("beacon", "adxl345", "lis2dw", "lis3dh", "mpu9250", "icm20948")
# The configured upstream drivers select these ranges themselves.  An unknown
# compatible fork must declare its real range instead of assuming 16 g.
DRIVER_RANGES = {"adxl345": 16.0, "lis2dw": 16.0, "lis3dh": 16.0,
                 "mpu9250": 8.0, "icm20948": 16.0}


def configured_sensors(config):
    names = set()
    for prefix in SENSOR_TYPES:
        if config.has_section(prefix):
            names.add(prefix)
        get_sections = getattr(config, "get_prefix_sections", None)
        if get_sections is not None:
            for section in get_sections(prefix):
                name = section.get_name()
                if name == prefix or name.startswith(prefix + " "):
                    names.add(name)
    return sorted(names)


def select_sensor(config):
    """Legacy Beacon configurations retain their original default selection."""
    selected = config.get("accel_chip", None)
    legacy_beacon = config.get("beacon", "beacon")
    if selected is None and config.get_name() == "beacon_accel_homing":
        if config.has_section(legacy_beacon):
            selected = legacy_beacon
    if selected is None or selected.strip().lower() == "auto":
        names = configured_sensors(config)
        if len(names) != 1:
            raise config.error(
                "accel_homing: set accel_chip to the full configured accelerometer"
                " section name; found %s" % (", ".join(names) or "none",))
        selected = names[0]
    selected = selected.strip()
    if not config.has_section(selected):
        raise config.error("accel_homing: no [%s] accelerometer section" % selected)
    return selected


class _LegacyClient:
    """Klipper v0.12 motion_report.APIDumpHelper web-request client contract."""
    def __init__(self, callback):
        self.callback = callback
        self.closed = False

    def get_client_connection(self):
        return self

    def get_dict(self, name, default):
        return default

    def is_closed(self):
        return self.closed

    def send(self, message):
        if not self.closed:
            self.closed = not self.callback(message["params"])


class ConfiguredAccelBackend:
    """Machine-mapped mm/s² batches, normalized to 0.001 g detector units."""
    kind = "klipper_batch"
    count_g = 0.001

    def __init__(self, owner, config, sensor):
        self.owner, self.sensor = owner, sensor
        self.name = owner.sensor_name
        self.interval = config.getfloat("stream_interval", 0.005,
                                        minval=0.001, maxval=0.020)
        self.max_age = config.getfloat("max_sample_age", 0.050,
                                       minval=0.005, maxval=0.050)
        self.min_rate = config.getfloat("min_sample_rate", 1000.0,
                                        minval=800.0, maxval=10000.0)
        self.report_travel = config.getfloat("max_report_travel", None,
                                             minval=REPORT_TRAVEL_MIN, maxval=REPORT_TRAVEL_MAX)
        self.startup_travel = config.getfloat("max_startup_travel", STARTUP_REQUEST_TRAVEL_LIMIT,
                                              minval=STARTUP_REQUEST_TRAVEL_MIN,
                                              maxval=STARTUP_REQUEST_TRAVEL_MAX)
        for option, value in (("stream_interval", self.interval), ("max_sample_age", self.max_age),
                              ("min_sample_rate", self.min_rate), ("max_report_travel", self.report_travel),
                              ("max_startup_travel", self.startup_travel)):
            if value is not None and not math.isfinite(value):
                raise config.error("accel_homing: %s must be finite" % option)
        self.max_speed = motion_report_speed_limit(self.max_age, self.report_travel)
        self.full_scale = config.getfloat(
            "sensor_range", DRIVER_RANGES.get(self.name.split()[0], 0.0),
            minval=0.0, maxval=1000.0)
        if not math.isfinite(self.full_scale) or self.full_scale <= 0.0:
            raise config.error("accel_homing: compatible sensor %s needs its actual"
                               " sensor_range in g" % self.name)
        if self.interval >= self.max_age:
            raise config.error("accel_homing: stream_interval must be less than"
                               " max_sample_age")
        rate = getattr(sensor, "data_rate", None)
        if rate is not None and not math.isfinite(rate):
            raise config.error("accel_homing: sensor data_rate must be finite")
        if rate is not None and rate < self.min_rate:
            raise config.error("accel_homing: %s rate %s is below min_sample_rate %.0f"
                               % (self.name, rate, self.min_rate))
        self.rate = float(rate) if rate is not None else None
        self.helper = getattr(sensor, "batch_bulk", None)
        self.legacy = False
        if self.helper is not None and hasattr(self.helper, "add_client"):
            self.interval_attr, self.timer_attr = "batch_interval", "batch_timer"
        else:
            # Verified old Klipper APIDumpHelper; callback-shaped private APIs
            # are deliberately not guessed from the method's name.
            self.helper = getattr(sensor, "api_dump", None)
            if (self.helper is None or not hasattr(self.helper, "add_client")
                    or not isinstance(getattr(self.helper, "clients", None), dict)):
                raise config.error("accel_homing: %s has no supported accelerometer"
                                   " batch_bulk or legacy api_dump API" % self.name)
            self.legacy = True
            self.interval_attr, self.timer_attr = "update_interval", "update_timer"
        if not hasattr(self.helper, self.interval_attr):
            raise config.error("accel_homing: %s cannot request a shorter reporting"
                               " interval; unsupported driver API" % self.name)
        self.active = False
        self.generation = 0
        self.saved_interval = self.applied_interval = None
        self.client = None
        self.last_sample_time = None
        self.last_age = self.observed_rate = None
        self.error_counts = None
        self.pending = []
        self.fault = None
        self.last_xyz = self.unchanged_since = None

    def seconds_to_clock(self, seconds):
        return seconds * CLOCK_FREQ

    def print_time_to_clock(self, print_time):
        return int(round(print_time * CLOCK_FREQ))

    def clock_to_time(self, clock):
        # Reconstruct the 32-bit pseudo clock against the synchronized MCU
        # print time, including printers which have run longer than 71 min.
        now = self.owner.main_mcu.estimated_print_time(self.owner.reactor.monotonic())
        reference = self.print_time_to_clock(now)
        delta = ((clock - reference + 0x80000000) & 0xFFFFFFFF) - 0x80000000
        return (reference + delta) / CLOCK_FREQ

    def start(self):
        if self.active:
            return
        self.generation += 1
        token = self.generation
        self.last_sample_time = self.last_age = self.observed_rate = None
        self.error_counts = None
        self.pending = []
        self.fault = None
        self.last_xyz = self.unchanged_since = None
        self.saved_interval = getattr(self.helper, self.interval_attr)
        if not math.isfinite(self.saved_interval) or self.saved_interval <= 0:
            raise self.owner.printer.command_error("accel_homing: driver reporting interval"
                                                   " must be finite and positive")
        self.applied_interval = min(float(self.saved_interval), self.interval)
        setattr(self.helper, self.interval_attr, self.applied_interval)
        self.active = True

        def callback(message):
            if not self.active or token != self.generation:
                return False
            try:
                self._receive(message)
            except Exception:
                self._fail("invalid_data")
            return self.active

        self.client = _LegacyClient(callback) if self.legacy else callback
        try:
            self.helper.add_client(self.client)
            timer = getattr(self.helper, self.timer_attr, None)
            if timer is not None:
                self.owner.reactor.update_timer(
                    timer, self.owner.reactor.monotonic() + self.applied_interval)
        except Exception:
            self.stop()
            raise

    def stop(self):
        self.active = False
        self.generation += 1
        if self.legacy and self.client is not None:
            self.client.closed = True
        # Returning False unregisters only our client on the next callback.
        # Existing resonance/web clients continue.  Do not overwrite a later
        # interval choice made by another extension.
        if (self.saved_interval is not None
                and getattr(self.helper, self.interval_attr) == self.applied_interval):
            setattr(self.helper, self.interval_attr, self.saved_interval)
        self.saved_interval = self.applied_interval = None
        self.pending = []

    def _fail(self, reason):
        self.fault = reason
        self.owner._sensor_fault(reason)

    def prepare_reference(self):
        # Start a new timestamp sequence after completed queued motion. Old
        # buffered rows are filtered by the owner's sensor-time cutoff.
        self.pending = []
        self.last_sample_time = None
        self.last_xyz = self.unchanged_since = None

    def _receive(self, message):
        rows = message.get("data", ())
        if not rows:
            return  # Never refresh the watchdog for an empty message.
        current = tuple(int(message.get(key, 0)) for key in ("errors", "overflows"))
        if min(current) < 0:
            return self._fail("invalid_data")
        # Drivers report cumulative counters; a new session starts from zero.
        if (self.error_counts is None and any(current)) or (
                self.error_counts is not None and current != self.error_counts):
            return self._fail("data_error")
        self.error_counts = current
        values = []
        reference = self.owner.preparation
        noise_session = self.owner.noise_session is not None
        noise_window = self.owner.noise_window if noise_session else None
        drain_noise = noise_session and (noise_window is None or noise_window["complete"])
        noise_cutoff = None if not noise_session or drain_noise else noise_window["cutoff"]
        if drain_noise:
            # No samples are being measured between stationary windows. Do not
            # let this unmeasured interval carry pending data or continuity into
            # the next window, but still validate rows and driver error counts.
            self.prepare_reference()
        elif noise_cutoff is not None:
            self.pending = [row for row in self.pending if row[0] > noise_cutoff]
            if self.last_sample_time is not None and self.last_sample_time <= noise_cutoff:
                self.last_sample_time = None
                self.last_xyz = self.unchanged_since = None
        # home_rails_begin can attach to an already streaming helper before
        # move_begin establishes its stricter post-settle cutoff. Drain old
        # valid backlog here rather than poisoning the upcoming preparation.
        # Active noise windows have their own cutoff; trimming by age there
        # would hide genuinely late samples belonging to the measured window.
        warmup_cutoff = None
        if reference is None and self.owner.attempt is None and not noise_session:
            warmup_cutoff = self.owner.main_mcu.estimated_print_time(
                self.owner.reactor.monotonic()) - self.max_age
            if self.last_sample_time is not None and self.last_sample_time < warmup_cutoff:
                # This history is outside the same interval being drained.
                # Comparing a retained row with it would report our own trim
                # as missing samples. Reset once, before checking retained rows.
                self.prepare_reference()
        for row in rows:
            if len(row) != 4:
                return self._fail("invalid_data")
            sample = tuple(float(value) for value in row)
            if not all(math.isfinite(value) for value in sample):
                return self._fail("invalid_data")
            if drain_noise or (noise_cutoff is not None and sample[0] <= noise_cutoff):
                continue
            if reference is not None and sample[0] <= reference.cutoff:
                reference.pre_cutoff += 1
                continue
            if warmup_cutoff is not None and sample[0] < warmup_cutoff:
                continue
            if self.last_sample_time is not None:
                dt = sample[0] - self.last_sample_time
                if dt <= 0.0:
                    return self._fail("invalid_data")
                if dt > 5.0 / (self.rate or self.min_rate):
                    return self._fail("sample_gap")
            self.last_sample_time = sample[0]
            xyz = sample[1:]
            if xyz != self.last_xyz:
                self.last_xyz, self.unchanged_since = xyz, sample[0]
            values.append(sample)
        values = self.pending + values
        self.pending = []
        if not values:
            return
        now = self.owner.reactor.monotonic()
        print_time = self.owner.main_mcu.estimated_print_time(now)
        self.last_age = print_time - values[-1][0]
        if reference is not None:
            reference.oldest_batch_age = print_time - values[0][0]
            reference.newest_batch_age = self.last_age
        attempt = self.owner.attempt
        relevant = [row for row in values if attempt is None or row[0] >= attempt.arm_time]
        if self.last_age < -0.005 or (relevant and print_time - relevant[0][0] > self.max_age):
            # Stop any armed move before diagnostic formatting or logging.
            self._fail("stale_data")
            record_timing = getattr(self.owner, "_record_sample_timing_fault", None)
            if record_timing is not None:
                oldest, newest = (relevant or values)[0][0], values[-1][0]
                future = self.last_age < -0.005
                cutoff = (noise_cutoff if noise_cutoff is not None else
                          reference.cutoff if reference is not None else
                          attempt.arm_time if attempt is not None else None)
                record_timing("backend_batch", print_time, oldest, newest, cutoff=cutoff,
                              reason="future" if future else "too_old",
                              sample_time=newest if future else oldest)
            return
        if (attempt is not None and not attempt.done
                and values[-1][0] - max(self.unchanged_since, attempt.arm_time) > 0.025
                and self.owner._commanded_speed(attempt, values[-1][0] - 0.015) >= 40.0):
            return self._fail("frozen")
        if len(values) < 2:
            self.pending = values
            return
        self.pending = []
        duration = values[-1][0] - values[0][0]
        self.observed_rate = (len(values) - 1) / duration
        if self.observed_rate < self.min_rate * 0.98:
            return self._fail("sample_rate")
        # A nonuniform batch would make the existing integrator and onset
        # interpolation inaccurate.  Tiny timestamp rounding is allowed.
        expected_dt = duration / (len(values) - 1)
        if any(abs((b[0] - a[0]) - expected_dt) > max(0.000003, expected_dt * 0.10)
               for a, b in zip(values, values[1:])):
            return self._fail("sample_gap")
        if attempt is not None and not attempt.done:
            before = [row for row in values if row[0] < attempt.arm_time]
            after = [row for row in values if row[0] >= attempt.arm_time]
            if before:
                self.owner.recent.append(tuple(v / GRAVITY / self.count_g
                                               for row in before for v in row[1:]))
            if len(after) < 2:
                self.pending = after
                return
            values = after
        data = tuple(value / GRAVITY / self.count_g for row in values for value in row[1:])
        self.owner._inspect({
            "start_clock": self.print_time_to_clock(values[0][0]) & 0xFFFFFFFF,
            "delta_clock": self.print_time_to_clock(values[-1][0] - values[0][0]),
            "data": data, "#receive_time": now,
        })

    def validate_rail(self, info, error):
        speeds = [info.speed]
        if info.retract_dist or getattr(info, "min_home_dist", 0.0):
            speeds.append(info.second_homing_speed)
        if not all(math.isfinite(speed) and speed > 0 for speed in speeds):
            raise error("accel_homing: homing speeds must be finite and positive")
        velocity, accel = self.owner.toolhead.get_max_velocity()
        if not all(math.isfinite(value) and value > 0 for value in (velocity, accel)):
            raise error("accel_homing: toolhead max_velocity/max_accel must be finite and positive")
        if self.owner.homing_accel:
            accel = min(accel, self.owner.homing_accel)
        speeds = [min(speed, velocity) for speed in speeds]
        for speed in speeds:
            self.validate_motion_plan(speed, accel, error)

    def motion_startup_timeout(self, speed, accel):
        # 15ms is the detector's existing command/sensor lag allowance.  The
        # sample-age and two-report margins ensure the ramp is already visible
        # when a host-time deadline requests a no-motion failure.
        return motion_startup_timeout(speed, accel, self.max_age, self.interval)

    def motion_startup_request_travel(self, speed, accel):
        return motion_startup_request_travel(speed, accel, self.max_age, self.interval)

    def motion_startup_speed_limit(self, accel):
        # Distance is strictly increasing for positive speed, including the
        # speed-dependent proof window. Bound the search by the acceleration
        # term so even very small positive accelerations keep useful precision.
        if self.startup_travel is None:
            return self.max_speed
        low = 0.0
        minimum_margin = 0.018 + self.max_age + max(0.010, 2.0 * self.interval)
        high = min(math.sqrt(2.0 * self.startup_travel) * math.sqrt(accel),
                   self.startup_travel / minimum_margin)
        if self.max_speed is not None:
            high = min(high, self.max_speed)
        for unused in range(64):
            middle = (low + high) * 0.5
            travel = self.motion_startup_request_travel(middle, accel)
            if math.isfinite(travel) and travel <= self.startup_travel - 0.000001:
                low = middle
            else:
                high = middle
        # Round down to six significant digits for an actionable diagnostic.
        unit = 10.0 ** (math.floor(math.log10(low)) - 5)
        return math.floor(low / unit) * unit

    def validate_motion_plan(self, speed, accel, error):
        if not all(math.isfinite(value) and value > 0 for value in (speed, accel)):
            raise error("accel_homing: effective homing speed/acceleration must be finite and positive")
        travel = self.motion_startup_request_travel(speed, accel)
        timeout = self.motion_startup_timeout(speed, accel)
        if not math.isfinite(travel) or not math.isfinite(timeout):
            raise error("accel_homing: derived startup timeout and nominal travel must be finite"
                        " at effective homing speed %.6g mm/s and acceleration %.6g mm/s^2"
                        % (speed, accel))
        report_failed = self.max_speed is not None and speed > self.max_speed
        startup_failed = self.startup_travel is not None and travel > self.startup_travel
        if report_failed or startup_failed:
            maximum = self.motion_startup_speed_limit(accel)
            report_setting = "disabled" if self.report_travel is None else "%.12g mm" % self.report_travel
            startup_setting = "disabled" if self.startup_travel is None else "%.12g mm" % self.startup_travel
            detail = ("effective homing speed %.6g mm/s and acceleration %.6g mm/s^2;"
                      " report needs %.12g mm at max_sample_age %.6g s"
                      " (max_report_travel %s); startup needs %.12g mm"
                      " (max_startup_travel %s, stream_interval %.6g s)."
                      " The current-setting homing_speed ceiling is at most %.6g mm/s."
                      " Review compatible settings explicitly; speed and acceleration"
                      " were not changed. This is not a guarantee of physical stopping travel"
                      % (speed, accel, speed * self.max_age, self.max_age, report_setting,
                         travel, startup_setting, self.interval, maximum))
            if report_failed:
                raise error("accel_homing: %s homing speed exceeds %.1f mm/s for the"
                            " sample-age/report-travel policy; %s"
                            % (self.name, self.max_speed, detail))
            raise error("accel_homing: startup motion proof needs %.2f mm of commanded travel"
                        " before the host stop request, above the %.1f mm policy; %s"
                        % (travel, self.startup_travel, detail))

    def get_status(self):
        status = {"type": self.kind, "sensor": self.name,
                "requested_interval_s": self.interval,
                "max_sample_age_s": self.max_age,
                "last_sample_age_s": self.last_age,
                "observed_rate_hz": self.observed_rate,
                "max_homing_speed": self.max_speed, "fault": self.fault,
                "max_report_travel_mm": self.report_travel,
                "startup_motion_request_limit_mm": self.startup_travel,
                "latency_guaranteed": False}
        if self.owner.toolhead is not None:
            velocity, accel = self.owner.toolhead.get_max_velocity()
            if self.owner.homing_accel:
                accel = min(accel, self.owner.homing_accel)
            attempt = self.owner.attempt
            speed = (attempt.effective_speed if attempt is not None and attempt.effective_speed is not None
                     else velocity if self.max_speed is None else min(velocity, self.max_speed))
            if attempt is not None and attempt.effective_accel is not None:
                accel = attempt.effective_accel
            valid = all(math.isfinite(value) and value > 0 for value in (speed, accel))
            timeout = self.motion_startup_timeout(speed, accel) if valid else None
            travel = self.motion_startup_request_travel(speed, accel) if valid else None
            status["startup_motion_evaluated_speed_mm_s"] = speed if valid else None
            status["startup_motion_timeout_s"] = timeout if timeout is not None and math.isfinite(timeout) else None
            # Nominal request calculation includes one 3ms early watchdog interval.
            # It excludes host scheduling stalls and physical MCU stop delay.
            status["startup_motion_request_travel_mm"] = travel if travel is not None and math.isfinite(travel) else None
        return status
