"""Offline state-machine tests: simulated G28 results never touch a printer."""
import collections
import contextlib
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import struct
import tempfile
import types
import unittest
import zlib
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import accel_homing_charts as charts_module
from accel_homing_charts import write_capture
from accel_homing_model import AccelHomingModel
from harness import CommandError, ConfigError, FakeReactor, plugin


class FakePNGRenderer:
    """Exercise payload/provenance integration without importing plotting libs."""
    def __init__(self):
        self.calls = []
        self.on_render = None

    def render(self, payload, check_running=None):
        if check_running:
            check_running()
        self.calls.append(payload)
        if self.on_render:
            self.on_render(payload)
        if check_running:
            check_running()
        def chunk(kind, content):
            return struct.pack(">I", len(content)) + kind + content + struct.pack(">I", zlib.crc32(kind + content) & 0xffffffff)
        return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 2, 1, 8, 6, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(b"\0" + b"\xff\xff\xff\xff" * 2)) + chunk(b"IEND", b""))


class ConsoleMessages(list):
    """Every exercised response path must stay free of popup protocol output."""
    def append(self, message):
        if "action:prompt_" in message:
            raise AssertionError("workflow emitted a popup action: " + message)
        super().append(message)


class Config:
    error = ConfigError

    def __init__(self, printer, options=None, name="accel_homing", sections=None):
        self.printer, self.options, self.name = printer, options or {}, name
        self.sections = sections or {"printer": {"kinematics": "corexy"}}

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, name, default=None):
        return self.options.get(name, default)

    def getfloat(self, name, default=None, minval=None, maxval=None):
        return self._number(float, name, default, minval, maxval)

    def getint(self, name, default=None, minval=None, maxval=None):
        return self._number(int, name, default, minval, maxval)

    def _number(self, kind, name, default, low, high):
        value = self.options.get(name, default)
        if value is None:
            return value
        value = kind(value)
        if (low is not None and value < low) or (high is not None and value > high):
            raise ConfigError(name + " out of range")
        return value

    def has_section(self, name):
        return name in self.sections

    def getsection(self, name):
        return Config(self.printer, self.sections[name], name, self.sections)


class GCmd:
    error = CommandError

    def __init__(self, command=None, commandline=None, **params):
        self.params, self.responses = params, ConsoleMessages()
        self.command, self.commandline = command, commandline

    def get_command(self):
        return self.command

    def get_commandline(self):
        return self.commandline

    def get_command_parameters(self):
        return dict(self.params)

    def get_raw_command_parameters(self):
        return self.commandline.partition(" ")[2] if self.commandline else ""

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        value = int(self.get(name, default))
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise CommandError("invalid " + name)
        return value

    def get_float(self, name, default=None, minval=None, maxval=None):
        try:
            value = float(self.get(name, default))
        except (TypeError, ValueError):
            raise CommandError("invalid " + name)
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise CommandError("invalid " + name)
        return value

    def respond_info(self, message):
        self.responses.append(message)


class StockHoming:
    """Synthetic stock owner; its bound handler emits the rig's homing events."""
    def __init__(self, printer):
        self.printer = printer

    def cmd_G28(self, gcmd):
        try:
            return self.printer.stock_home(gcmd)
        except CommandError:
            # Match stock PrinterHoming's fatal motor_off cleanup. Workflows
            # must propagate this error without promising Z stays enabled.
            for line in self.printer.lines.values():
                line.motor_disable(0.0)
            self.printer.toolhead.kin.clear_homing_state("xyz")
            raise


class SafeZHoming:
    """Synthetic built-in wrapper: z_hop deliberately precedes axis selection."""
    def __init__(self, printer):
        self.printer, self.gcode = printer, printer.gcode
        self.prev_G28 = printer.objects["homing"].cmd_G28
        self.z_hop, self.z_hop_speed = 50.0, 15.0
        self.home_x_pos, self.home_y_pos, self.speed = 140.0, 90.0, 50.0
        self.calls = []

    def cmd_G28(self, gcmd):
        self.calls.append(gcmd.get_commandline())
        toolhead = self.printer.toolhead
        homed = toolhead.kin.get_status(0)["homed_axes"]
        if self.z_hop:
            if "z" not in homed:
                position = toolhead.get_position()
                position[2] = 0.0
                toolhead.set_position(position, homing_axes="z")
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)
                toolhead.kin.clear_homing_state("z")
            elif toolhead.position[2] < self.z_hop:
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)
        axes = {axis: "0" for axis in "XYZ" if gcmd.get(axis, None) is not None}
        if not axes:
            axes = {axis: "0" for axis in "XYZ"}
        xy = {axis: value for axis, value in axes.items() if axis in "XY"}
        if xy:
            self.prev_G28(self.gcode.create_gcode_command("G28", "G28", xy))
        if "Z" in axes:
            homed = toolhead.kin.get_status(0)["homed_axes"]
            if not all(axis in homed for axis in "xy"):
                raise gcmd.error("Must home X and Y axes first")
            toolhead.manual_move([self.home_x_pos, self.home_y_pos], self.speed)
            self.prev_G28(self.gcode.create_gcode_command("G28", "G28", {"Z": "0"}))
            if self.z_hop and toolhead.position[2] < self.z_hop:
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)


class HomingOverride:
    """Native wrapper contract, including forced positions and script recursion."""
    def __init__(self, printer, axes="XYZ", start_pos=(None, None, 0.0)):
        self.printer, self.gcode = printer, printer.gcode
        self.prev_G28 = printer.objects["homing"].cmd_G28
        self.axes, self.start_pos = axes.upper(), list(start_pos)
        self.in_script, self.calls = False, []
        self.template = types.SimpleNamespace(
            create_template_context=mock.Mock(return_value={}),
            run_gcode_from_command=mock.Mock())

    def cmd_G28(self, gcmd):
        self.calls.append(gcmd.get_commandline())
        if self.in_script:
            return self.prev_G28(gcmd)
        no_axis = all(gcmd.get(axis, None) is None for axis in "XYZ")
        if not no_axis and not any(gcmd.get(axis, None) is not None for axis in self.axes):
            return self.prev_G28(gcmd)
        toolhead = self.printer.toolhead
        position, homing_axes = toolhead.get_position(), ""
        for index, location in enumerate(self.start_pos):
            if location is not None:
                position[index] = location
                homing_axes += "xyz"[index]
        toolhead.set_position(position, homing_axes=homing_axes)
        context = self.template.create_template_context()
        context["params"] = gcmd.get_command_parameters()
        context["rawparams"] = gcmd.get_raw_command_parameters()
        try:
            self.in_script = True
            self.template.run_gcode_from_command(context)
        finally:
            self.in_script = False


def load_workflow_fixture():
    """Exercise real relative identity imports using isolated synthetic modules."""
    package_name = "accel_homing_workflow_fixture"
    package = types.ModuleType(package_name)
    package.__path__ = [str(Path(__file__).resolve().parent.parent)]
    homing = types.ModuleType(package_name + ".homing")
    homing.PrinterHoming = StockHoming
    safe = types.ModuleType(package_name + ".safe_z_home")
    safe.SafeZHoming = SafeZHoming
    override = types.ModuleType(package_name + ".homing_override")
    override.HomingOverride = HomingOverride
    name = package_name + ".accel_homing_workflows"
    spec = importlib.util.spec_from_file_location(name, Path(package.__path__[0]) / "accel_homing_workflows.py")
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, {package_name: package, name: module,
                                     homing.__name__: homing, safe.__name__: safe,
                                     override.__name__: override,
                                     package_name + ".accel_homing_charts": charts_module}):
        spec.loader.exec_module(module)
    return module


workflow_module = load_workflow_fixture()
AccelHomingWorkflows = workflow_module.AccelHomingWorkflows


class Line:
    def __init__(self, name, disabled):
        self.name, self.disabled = name, disabled
        self.enable = object()
        self.enabled = True

    def motor_disable(self, stamp):
        self.enabled = False
        self.disabled.append(self.name)

    def is_motor_enabled(self):
        return self.enabled


class Stepper:
    def __init__(self, name, axes):
        self.name, self.axes = name, axes
        self.mcu_position = 0

    def get_name(self):
        return self.name

    def is_active_axis(self, axis):
        return axis in self.axes

    def get_step_dist(self):
        return 0.000001

    def get_mcu_position(self):
        return self.mcu_position


class Rail:
    def __init__(self, positive=False, bounds=(0.0, 300.0), endstop=None):
        self.bounds = bounds
        self.info = types.SimpleNamespace(
            retract_dist=0.0, min_home_dist=0.0, speed=100.0, positive_dir=positive,
            position_endstop=(bounds[1] if positive else bounds[0]) if endstop is None else endstop)

    def get_homing_info(self):
        return self.info

    def get_range(self):
        return self.bounds


class Kin:
    def __init__(self):
        self.limits = [(1.0, -1.0), (1.0, -1.0), (0.0, 500.0)]
        self.steppers = [Stepper("stepper_x", "xy"), Stepper("stepper_y", "xy"),
                         Stepper("stepper_z", "z"), Stepper("stepper_z1", "z")]
        self.dual_carriage_axis = None

    def get_steppers(self):
        return self.steppers

    def calc_position(self, positions):
        a, b = positions["stepper_x"], positions["stepper_y"]
        return [(a + b) * 0.5, (a - b) * 0.5, positions["stepper_z"]]

    def clear_homing_state(self, axes):
        for i, axis in enumerate("xyz"):
            if axis in axes:
                self.limits[i] = (1.0, -1.0)

    def get_status(self, stamp):
        return {"homed_axes": "".join(axis for axis, limits in zip("xyz", self.limits)
                                      if limits[0] <= limits[1])}


class Toolhead:
    def __init__(self, rig):
        self.rig, self.kin = rig, Kin()
        self.position = [150.0, 100.0, 40.0, 6.0]
        self.moves, self.dwells = [], []
        self.accel, self.accel_log = 50000.0, []
        self.busy = (0.0, 10.0, True)

    def get_position(self):
        return list(self.position)

    def set_position(self, position, homing_axes=""):
        self.position = list(position)
        for index, axis in enumerate("xyz"):
            if axis in homing_axes:
                self.kin.limits[index] = (0.0, 500.0)

    def get_kinematics(self):
        return self.kin

    def get_max_velocity(self):
        return 500.0, self.accel

    def set_max_velocities(self, velocity, accel, scv, mcr):
        if accel is not None:
            self.accel = accel
            self.accel_log.append(accel)

    def get_status(self, stamp):
        return self.kin.get_status(stamp)

    def check_busy(self, stamp):
        return self.busy

    def wait_moves(self):
        self.busy = (0.0, 10.0, True)

    def flush_step_generation(self):
        pass

    def dwell(self, seconds):
        self.dwells.append(seconds)
        self.rig.idle_state = "Printing"  # stock toolhead:sync_print_time lifecycle

    def get_last_move_time(self):
        return 0.0

    def manual_move(self, coords, speed):
        self.moves.append((list(coords), speed))
        delta = [(value - self.position[i]) if value is not None else 0.0
                 for i, value in enumerate(coords)]
        for stepper in self.kin.steppers:
            displacement = ((delta[0] + delta[1] if stepper.name == "stepper_x" else delta[0] - delta[1])
                            if stepper.axes == "xy" else delta["xyz".index(stepper.axes)])
            stepper.mcu_position += round(displacement / stepper.get_step_dist())
        for index, value in enumerate(coords):
            if value is not None:
                self.position[index] = value
        self.rig.idle_state = "Printing"


class Configfile:
    def __init__(self):
        self.saved = {}
        self.fail = False

    def set(self, section, option, value):
        if self.fail:
            raise CommandError("simulated config staging error")
        self.saved[(section, option)] = value


class Rig:
    config_error = ConfigError
    command_error = CommandError

    def __init__(self, options=None, sections=None):
        self.reactor = FakeReactor()
        self.handlers = collections.defaultdict(list)
        self.output, self.scripts, self.disabled, self.home_calls = ConsoleMessages(), [], [], []
        self.dispatched_scripts, self.created_commands, self.homing_events, self.z_homes = [], [], [], []
        self.end_sessions = 0
        self.idle_state, self.print_state = "Ready", "standby"
        self.paused, self.sd_active, self.shutdown = False, False, False
        self.save_error = False
        self.policy_checks, self.policy_error = [], None
        self.home_results = collections.deque()
        self.on_home = None
        self.capture_active, self.capture_stops = False, 0
        self.noise_calls, self.noise_window_count = [], 0
        self.on_noise, self.amplitudes, self.calibration_charts = None, (.01, .005, .002), None
        self.bad_capture = None
        self.toolhead = Toolhead(self)
        self.lines = {s.name: Line(s.name, self.disabled) for s in self.toolhead.kin.steppers}
        self.configfile = Configfile()
        self.gcode = types.SimpleNamespace(
            ready_gcode_handlers={}, register_command=self.register_command,
            respond_info=self.output.append, run_script_from_command=self.run_script,
            create_gcode_command=self.create_gcode_command)
        self.gcode.gcode_handlers = self.gcode.ready_gcode_handlers
        homing = StockHoming(self)
        self.gcode.ready_gcode_handlers["G28"] = homing.cmd_G28
        self.objects = {
            "gcode": self.gcode, "toolhead": self.toolhead,
            "homing": homing, "configfile": self.configfile,
            "stepper_enable": types.SimpleNamespace(lookup_enable=self.lines.__getitem__),
            "idle_timeout": types.SimpleNamespace(get_status=lambda t: {"state": self.idle_state}),
            "print_stats": types.SimpleNamespace(get_status=lambda t: {"state": self.print_state}),
            "pause_resume": types.SimpleNamespace(get_status=lambda t: {"is_paused": self.paused}),
            "virtual_sdcard": types.SimpleNamespace(is_active=lambda: self.sd_active)}
        self.config = Config(self, options, sections=sections)
        self.owner = types.SimpleNamespace(
            printer=self, reactor=self.reactor, toolhead=self.toolhead, name="accel_homing",
            threshold_g={"x": 8.0, "y": 9.0}, impact_sign={"x": 0, "y": 0},
            homing_accel=5000.0, back_off_dist=5.0, full_scale_g=16.0,
            homing_active=False, attempt=None, last={}, operation_generation=0, saved_accel=None,
            endstops={"x": object(), "y": object()},
            rails_by_axis={"x": Rail(), "y": Rail(bounds=(0.0, 200.0))},
            sensor_name="simulated", start_capture=self.start_capture, stop_capture=self.stop_capture,
            preflight_motion_policy=self.preflight_motion_policy,
            _end_session=self.end_session, cmd_STATUS=lambda g: g.respond_info("core status"))
        self.owner._homed = lambda axis: axis in self.toolhead.kin.get_status(0)["homed_axes"]
        self.owner.noise_session = None
        self.owner.noise_sensor_context = self.sensor_context
        self.owner.start_noise_measurement = self.start_noise
        self.owner.stop_noise_measurement = self.stop_noise
        self.owner.collect_noise_window = self.collect_noise
        for method in ("_retract_completed_home", "release_workflow_home", "_limit_accel",
                       "_restore_accel", "_set_accel"):
            setattr(self.owner, method, types.MethodType(getattr(plugin.BeaconAccelHoming, method), self.owner))
        self.workflow = AccelHomingWorkflows(self.owner, self.config)
        self.workflow.renderer = FakePNGRenderer()
        self.owner.workflows = self.workflow

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def load_object(self, config, name):
        if name not in self.objects and name.startswith("accel_homing_model "):
            self.objects[name] = AccelHomingModel(config.getsection(name))
        return self.objects[name]

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, callback):
        self.handlers[event].append(callback)

    def is_shutdown(self):
        return self.shutdown

    def register_command(self, name, callback, desc=None):
        if name in self.gcode.ready_gcode_handlers:
            raise ConfigError("duplicate command: " + name)
        self.gcode.ready_gcode_handlers[name] = callback

    def preflight_motion_policy(self, axes, error=None):
        self.policy_checks.append(tuple(axes))
        if self.policy_error is not None:
            raise (error or self.command_error)(self.policy_error)

    def end_session(self):
        self.end_sessions += 1
        self.owner.homing_active = False
        self.owner.attempt = None

    def start_capture(self):
        if self.capture_active:
            raise CommandError("capture already active")
        self.capture_active = True

    def stop_capture(self):
        self.capture_active = False
        self.capture_stops += 1
        if self.bad_capture is not None:
            return self.bad_capture
        axis = self.home_calls[-1]["axis"] if self.home_calls else "x"
        record = self.owner.last.get(axis)
        t0 = self.capture_stops - 1.0
        onset_sample = (round(t0 * 1000) & 0xFFFFFFFF, 3, 2, 4)
        if record is not None:
            record["onset_sample"] = onset_sample
        return {"samples": [(t0, 0.0, 0.0, 1.0), (t0 + 0.001, 0.1, 0.0, 1.0),
                            (t0 + 0.002, -4.0, -5.0, 1.0), (t0 + 0.003, 0.0, 0.0, 1.0)],
                "dropped": 0, "baseline_g": (0.0, 0.0, 1.0), "_record": record,
                "detector": None if record is None else {
                    "axis": axis, "attempt_count": 1, "arm_time_s": record["arm_time_s"],
                    "onset_time_s": record["onset_time_s"], "threshold_g": self.owner.threshold_g[axis],
                    "trigger_onset_time_s": record["onset_time_s"],
                    "onset_sample_version": 1, "onset_sample": onset_sample,
                    "onset_sample_matches": 1, "onset_sample_index": 2,
                    "onset_g": record.get("onset_g"), "impact_peak_g": record.get("impact_peak_g"),
                    "impact_metric": record.get("impact_metric", "selected_axis"),
                    "direction": record.get("direction", -1),
                    "homing_speed_mm_s": self.owner.rails_by_axis[axis].info.speed,
                    "homing_accel_mm_s2": self.owner.homing_accel}}

    def create_gcode_command(self, command, commandline, params):
        self.created_commands.append((command, commandline, dict(params)))
        return GCmd(command, commandline, **params)

    def sensor_context(self):
        return {"sensor": self.owner.sensor_name, "backend": "synthetic_noise",
                "scale_id": 0, "scale_name": "16g", "count_g": 0.001,
                "full_scale_g": self.owner.full_scale_g,
                "capture_axes_map": [[0, 1], [1, 1], [2, 1]],
                "sensor_axes_map": [[0, 1], [1, 1], [2, 1]], "sensor_rate_hz": 1000.0}

    def start_noise(self):
        self.noise_calls.append("start")
        self.noise_window_count = 0
        self.owner.noise_session = object()
        return self.sensor_context()

    def stop_noise(self):
        self.noise_calls.append("stop")
        self.owner.noise_session = None

    def collect_noise(self, check_running, duration):
        check_running()
        self.noise_window_count += 1
        iteration = self.noise_window_count
        self.noise_calls.append(iteration)
        if self.on_noise:
            self.on_noise(iteration)
        check_running()
        start = 10.0 + iteration * 1.1
        baseline = (0.25 * iteration, -0.2 * iteration, 1.0)
        samples = [(start + number / 1000.0,) + tuple(
            bias + amplitude * (0 if number == 1000 else 1 if number % 2 else -1)
            for bias, amplitude in zip(baseline, self.amplitudes)) for number in range(1001)]
        return {"samples": samples, "dropped": 0, "fault": None,
                "cutoff_time_s": start - 0.001, "_noise_context": self.sensor_context()}

    def use_safe_z_home(self):
        safe = SafeZHoming(self)
        self.objects["safe_z_home"] = safe
        self.gcode.ready_gcode_handlers["G28"] = safe.cmd_G28
        return safe

    def use_homing_override(self, axes="XYZ", start_pos=(None, None, 0.0)):
        override = HomingOverride(self, axes, start_pos)
        self.objects["homing_override"] = override
        self.gcode.ready_gcode_handlers["G28"] = override.cmd_G28
        return override

    def run_script(self, script):
        self.dispatched_scripts.append(script)
        if script.startswith("G28"):
            params = {axis: "" for axis in script.split()[1:]}
            return self.gcode.gcode_handlers["G28"](self.create_gcode_command("G28", script, params))
        self.scripts.append(script)
        if script == "SAVE_CONFIG":
            if self.save_error:
                raise CommandError("simulated SAVE_CONFIG include conflict")
            return
        raise AssertionError("unexpected command " + script)

    def stock_home(self, gcmd):
        axes = [axis.lower() for axis in "XYZ" if gcmd.get(axis, None) is not None]
        if not axes:
            raise AssertionError("unexpected all-axis stock G28")
        if axes == ["z"] and self.workflow.session is None:
            self.z_homes.append(gcmd)
            self.toolhead.position[2] = 0.0
            self.toolhead.kin.limits[2] = (0.0, 500.0)
            return
        if len(axes) != 1 or axes[0] not in "xy":
            raise AssertionError("unexpected stock G28 axes " + str(axes))
        axis = axes[0]
        for stepper in self.toolhead.kin.get_steppers():
            if stepper.is_active_axis(axis):
                self.lines[stepper.name].enabled = True
        self.scripts.append("G28 " + axis.upper())
        if self.workflow.session is None:
            self.toolhead.kin.limits["xy".index(axis)] = self.owner.rails_by_axis[axis].get_range()
            return
        self.homing_events.append(("begin", axis))
        for callback in self.handlers["homing:home_rails_begin"]:
            callback(axis)
        self.workflow.check_home_axes({axis: self.owner.rails_by_axis[axis]})
        self.home_calls.append({"axis": axis, "threshold": self.owner.threshold_g[axis],
                                "homed_axes": self.toolhead.kin.get_status(0)["homed_axes"],
                                "position": list(self.toolhead.position),
                                "attempt": self.workflow.session["attempts"],
                                "candidate_qualified": self.workflow.session["candidate_qualified"],
                                "successes": self.workflow.session["successes"]})
        if self.on_home:
            self.on_home(axis)
        result = self.home_results.popleft() if self.home_results else {}
        if isinstance(result, Exception):
            raise result
        _center, expected = self.workflow._geometry(axis)
        record = {"axis": axis, "pass": 1, "result": "impact", "travel_mm": expected,
                  "onset_g": 4.0, "impact_peak_g": 7.0, "moving": True,
                  "threshold_g": self.owner.threshold_g[axis], "arm_time_s": self.capture_stops + 0.001,
                  "onset_time_s": self.capture_stops + 0.002, "direction": -1}
        if self.workflow.session.get("mode") == "calibration":
            record["impact_metric"] = "xyz_magnitude"
        record.update(result)
        value = record.get("travel_mm_exact", record.get("travel_mm"))
        if (self.workflow.session.get("mode") == "calibration"
                and type(value) in (int, float) and math.isfinite(value) and value >= 0
                and (value <= 20 or value < expected * (1. - self.workflow.travel_tolerance))):
            record.setdefault("travel_mm_exact", value)
        sign = 1.0 if self.owner.rails_by_axis[axis].info.positive_dir else -1.0
        record.setdefault("trigger_travel_mm_signed", (None if type(value) not in (int, float) else
                                                       value * sign))
        if "halt_travel_mm_signed" not in record:
            value = record.get("travel_mm")
            record["halt_travel_mm_signed"] = (None if not isinstance(value, (int, float)) else
                                               value * sign)
        self.owner.last[axis] = record
        index = "xy".index(axis)
        self.toolhead.kin.limits[index] = self.owner.rails_by_axis[axis].get_range()
        self.toolhead.position[index] = self.owner.rails_by_axis[axis].info.position_endstop
        trigger, halt = record["trigger_travel_mm_signed"], record["halt_travel_mm_signed"]
        if all(type(value) in (int, float) and math.isfinite(value) for value in (trigger, halt)):
            self.toolhead.position[index] += halt - trigger
        positions = []
        for stepper in self.toolhead.kin.get_steppers():
            if stepper.is_active_axis(axis):
                factor = -1 if axis == "y" and stepper.name == "stepper_y" and stepper.axes == "xy" else 1
                ticks = lambda value: (round(value * factor / stepper.get_step_dist())
                                       if type(value) in (int, float) and math.isfinite(value) else 0)
                start = stepper.mcu_position
                stepper.mcu_position = start + ticks(halt)
                positions.append(types.SimpleNamespace(stepper=stepper, stepper_name=stepper.name,
                                                        start_pos=start, trig_pos=start + ticks(trigger),
                                                        halt_pos=stepper.mcu_position))
        self.workflow.note_calibration_halt(record, positions)
        self.workflow.check_home_result(record)
        self.homing_events.append(("end", axis))
        for callback in self.handlers["homing:home_rails_end"]:
            callback(axis)
        self.idle_state = "Printing"

    def calibrate(self, axis="X", save=None, **params):
        if "charts_directory" not in self.config.options and self.calibration_charts is None:
            self.calibration_charts = tempfile.TemporaryDirectory(prefix="accel-homing-calibration-")
            self.workflow.charts_directory = self.calibration_charts.name
        arguments = {"AXIS": axis, "CENTERED": 1}
        if save is not None:
            arguments["SAVE_CONFIG"] = save
        arguments.update(params)
        self.workflow.cmd_CALIBRATE(GCmd(**arguments))

    def wait_calibration(self, axis="X", save=None, **params):
        """Construct the retained manual-wait state without the automatic trial loop."""
        arguments = {"AXIS": axis, "CENTERED": 1, **params}
        if save is not None:
            arguments["SAVE_CONFIG"] = save
        self.workflow._begin(GCmd(**arguments), "calibration", axis.lower())
        self.workflow.session["state"] = "running"
        self.workflow._home(axis.lower(), self.workflow.session["candidate"])
        self.workflow._next_calibration_candidate("synthetic retained wait")
        self.workflow._wait_center("synthetic retained manual-centering wait")

    def effects(self):
        return copy.deepcopy({
            "scripts": self.scripts, "moves": self.toolhead.moves,
            "dispatched_scripts": self.dispatched_scripts, "created_commands": self.created_commands,
            "homing_events": self.homing_events, "z_homes": len(self.z_homes),
            "dwells": self.toolhead.dwells, "disabled": self.disabled,
            "enabled": {name: line.enabled for name, line in self.lines.items()},
            "position": self.toolhead.position, "limits": self.toolhead.kin.limits,
            "thresholds": self.owner.threshold_g, "back_off": self.owner.back_off_dist,
            "session": self.workflow.session, "result": self.workflow.last_result,
            "saved": self.configfile.saved, "output": self.output,
            "end_sessions": self.end_sessions, "capture_stops": self.capture_stops,
            "capture_active": self.capture_active, "timers": list(self.reactor.timers.values())})


class SafeZHomingWorkflowTests(unittest.TestCase):
    def test_nonzero_hop_is_omitted_for_each_workflow_and_both_z_homed_states(self):
        for mode in ("calibrate_x", "calibrate_y", "test", "charts"):
            for z_homed in (False, True):
                with self.subTest(mode=mode, z_homed=z_homed), tempfile.TemporaryDirectory() as temp:
                    rig = Rig({"charts_directory": temp})
                    rig.toolhead.kin.limits[2] = (20.0, 480.0) if z_homed else (1.0, -1.0)
                    safe = rig.use_safe_z_home()
                    registries = (rig.gcode.ready_gcode_handlers, rig.gcode.gcode_handlers)
                    registered = dict(registries[0])
                    previous = safe.prev_G28
                    z_limit = rig.toolhead.kin.limits[2]
                    events = []
                    observer = lambda axis: events.append(axis)
                    rig.register_event_handler("homing:home_rails_begin", observer)
                    if mode.startswith("calibrate"):
                        axis = mode[-1]
                        rig.calibrate(axis.upper(), save=0)
                        expected = [axis.upper()] * 11
                    else:
                        getattr(rig.workflow, "cmd_TEST" if mode == "test" else "cmd_CHARTS")(GCmd(CENTERED=1))
                        expected = ["X", "Y"]
                    self.assertEqual(safe.calls, [])
                    self.assertEqual(rig.dispatched_scripts, [])
                    self.assertEqual(rig.created_commands,
                                     [("G28", "G28 " + axis, {axis: ""}) for axis in expected])
                    self.assertEqual(events, [axis.lower() for axis in expected])
                    self.assertIn(observer, rig.handlers["homing:home_rails_begin"])
                    self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
                    self.assertEqual(rig.toolhead.kin.limits[2], z_limit)
                    self.assertTrue(rig.lines["stepper_z"].enabled)
                    self.assertTrue(rig.lines["stepper_z1"].enabled)
                    self.assertTrue(all(coords[2] is None for coords, _speed in rig.toolhead.moves))
                    self.assertEqual(rig.z_homes, [])
                    self.assertIs(safe.prev_G28, previous)
                    self.assertEqual(safe.z_hop, 50.0)
                    self.assertIs(rig.objects["safe_z_home"], safe)
                    self.assertIs(rig.gcode.ready_gcode_handlers, registries[0])
                    self.assertIs(rig.gcode.gcode_handlers, registries[1])
                    self.assertEqual(rig.gcode.ready_gcode_handlers, registered)
                    self.assertIsNone(rig.workflow.session)

    def test_normal_xy_g28_still_runs_wrapper_hop_for_homed_and_unhomed_z(self):
        for z_homed in (False, True):
            rig = Rig()
            if not z_homed:
                rig.toolhead.kin.limits[2] = (1.0, -1.0)
            safe = rig.use_safe_z_home()
            rig.run_script("G28 X")
            self.assertEqual(safe.calls, ["G28 X"])
            self.assertEqual(rig.toolhead.moves, [([None, None, 50.0], 15.0)])
            self.assertEqual(rig.toolhead.position[2], 50.0)
            self.assertEqual("z" in rig.toolhead.kin.get_status(0)["homed_axes"], z_homed)

    def test_normal_z_path_and_probe_object_remain_available_after_workflow(self):
        rig = Rig()
        probe = object()  # Probe implementation is external to this synthetic fixture.
        rig.objects["probe"] = rig.objects["cartographer"] = probe
        safe = rig.use_safe_z_home()
        handler = rig.gcode.gcode_handlers["G28"]
        rig.workflow.cmd_TEST(GCmd(CENTERED=1))
        rig.run_script("G28 Z")
        self.assertEqual(safe.calls, ["G28 Z"])
        self.assertEqual(len(rig.z_homes), 1)
        self.assertEqual(rig.z_homes[0].params, {"Z": "0"})
        self.assertEqual(rig.toolhead.moves[2:], [([None, None, 50.0], 15.0),
                                                ([140.0, 90.0], 50.0),
                                                ([None, None, 50.0], 15.0)])
        self.assertEqual(rig.gcode.gcode_handlers["G28"], handler)
        self.assertIs(rig.objects["probe"], probe)
        self.assertIs(rig.objects["cartographer"], probe)

    def test_unknown_and_nested_predecessors_are_rejected_before_side_effects(self):
        for kind in ("function", "other_owner", "nested", "missing"):
            with self.subTest(kind=kind):
                rig = Rig()
                safe = rig.use_safe_z_home()
                if kind == "function":
                    safe.prev_G28 = lambda gcmd: None
                elif kind == "other_owner":
                    safe.prev_G28 = StockHoming(rig).cmd_G28
                elif kind == "nested":
                    safe.prev_G28 = SafeZHoming(rig).cmd_G28
                else:
                    del safe.prev_G28
                before = rig.effects()
                with self.assertRaisesRegex(CommandError, "G28 predecessor"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.effects(), before)
                self.assertEqual(safe.calls, [])

    def test_custom_safe_wrapper_owner_type_and_methods_are_rejected(self):
        class SafeSubclass(SafeZHoming):
            pass

        for kind in ("subclass", "object", "owner", "dispatcher", "instance_method", "class_method"):
            with self.subTest(kind=kind):
                rig = Rig()
                safe = rig.use_safe_z_home()
                if kind == "subclass":
                    safe = SafeSubclass(rig)
                    rig.objects["safe_z_home"] = safe
                    rig.gcode.ready_gcode_handlers["G28"] = safe.cmd_G28
                elif kind == "object":
                    rig.objects["safe_z_home"] = types.SimpleNamespace(
                        printer=rig, gcode=rig.gcode, cmd_G28=safe.cmd_G28, prev_G28=safe.prev_G28)
                elif kind == "owner":
                    safe.printer = object()
                elif kind == "dispatcher":
                    safe.gcode = object()
                elif kind == "instance_method":
                    safe.cmd_G28 = lambda gcmd: None
                before = rig.effects()
                context = mock.patch.object(SafeZHoming, "cmd_G28", lambda obj, gcmd: None) \
                    if kind == "class_method" else contextlib.nullcontext()
                with context, self.assertRaisesRegex(CommandError, "G28"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.effects(), before)
                self.assertEqual(safe.calls, [])

    def test_unknown_stock_owners_and_patched_stock_methods_are_rejected(self):
        class StockSubclass(StockHoming):
            pass

        for kind in ("subclass", "object", "instance_method", "class_method", "missing_import"):
            with self.subTest(kind=kind):
                rig = Rig()
                if kind == "subclass":
                    rig.objects["homing"] = StockSubclass(rig)
                elif kind == "object":
                    rig.objects["homing"] = types.SimpleNamespace(printer=rig, cmd_G28=rig.stock_home)
                elif kind == "instance_method":
                    rig.objects["homing"].cmd_G28 = rig.stock_home
                before = rig.effects()
                if kind == "class_method":
                    context = mock.patch.object(StockHoming, "cmd_G28", lambda obj, gcmd: None)
                elif kind == "missing_import":
                    context = mock.patch.object(workflow_module, "_PrinterHoming", None)
                else:
                    context = contextlib.nullcontext()
                with context, self.assertRaisesRegex(CommandError, "stock G28"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.effects(), before)

    def test_missing_or_divergent_handler_registries_fail_closed(self):
        for wrapped in (False, True):
            for kind in ("ready_missing", "active_missing", "ready_empty", "active_empty",
                         "unknown_ready", "unknown_active", "inactive_wrapper"):
                with self.subTest(wrapped=wrapped, kind=kind):
                    rig = Rig()
                    if wrapped:
                        rig.use_safe_z_home()
                    if kind == "ready_missing":
                        del rig.gcode.ready_gcode_handlers
                    elif kind == "active_missing":
                        del rig.gcode.gcode_handlers
                    elif kind == "ready_empty":
                        rig.gcode.ready_gcode_handlers = {}
                    elif kind == "active_empty":
                        rig.gcode.gcode_handlers = {}
                    elif kind == "unknown_ready":
                        rig.gcode.ready_gcode_handlers = {"G28": lambda gcmd: None}
                    elif kind == "unknown_active":
                        rig.gcode.gcode_handlers = {"G28": lambda gcmd: None}
                    else:
                        rig.use_safe_z_home()
                        rig.gcode.ready_gcode_handlers["G28"] = rig.objects["homing"].cmd_G28
                    before = rig.effects()
                    with self.assertRaisesRegex(CommandError, "G28"):
                        rig.calibrate(save=0)
                    self.assertEqual(rig.effects(), before)

    def test_chain_changes_during_capture_setup_or_command_creation_abort_before_home(self):
        for stage in ("capture", "command"):
            with self.subTest(stage=stage):
                rig = Rig()
                safe = rig.use_safe_z_home()
                def mutate_after(callback, *args):
                    result = callback(*args)
                    safe.prev_G28 = lambda gcmd: None
                    return result
                if stage == "capture":
                    callback = rig.owner.start_capture
                    rig.owner.start_capture = lambda: mutate_after(callback)
                else:
                    callback = rig.gcode.create_gcode_command
                    rig.gcode.create_gcode_command = lambda *args: mutate_after(callback, *args)
                with self.assertRaisesRegex(CommandError, "G28 predecessor"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(safe.calls, [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.capture_stops, 1)
                self.assertFalse(rig.capture_active)
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                self.assertEqual(rig.owner.back_off_dist, 5.0)
                self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_chain_is_revalidated_between_trials_and_after_manual_center_wait(self):
        for mode in ("calibration", "test", "waiting"):
            for mutation in ("previous", "active", "override"):
                with self.subTest(mode=mode, mutation=mutation):
                    rig = Rig()
                    safe = rig.use_safe_z_home()
                    def mutate(axis=None):
                        if mutation == "previous":
                            safe.prev_G28 = lambda gcmd: None
                        elif mutation == "active":
                            rig.gcode.gcode_handlers = {"G28": lambda gcmd: None}
                        else:
                            rig.objects["homing_override"] = object()
                    if mode == "waiting":
                        rig.home_results.append({"travel_mm": 2.0})
                        rig.wait_calibration(save=0)
                        mutate()
                        resume = lambda: rig.calibrate(save=0)
                    elif mode == "test":
                        rig.on_home = mutate
                        resume = lambda: rig.workflow.cmd_TEST(GCmd(CENTERED=1))
                    else:
                        rig.on_home = mutate
                        resume = lambda: rig.calibrate(save=0)
                    with self.assertRaisesRegex(CommandError, "G28"):
                        resume()
                    self.assertEqual(len(rig.home_calls), 1)
                    self.assertEqual(safe.calls, [])
                    self.assertFalse(rig.capture_active)
                    self.assertIsNone(rig.workflow.session)
                    self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                    self.assertEqual(rig.owner.back_off_dist, 5.0)
                    self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_direct_stock_error_propagates_and_cleanup_preserves_registered_wrapper(self):
        for error_type in (CommandError, RuntimeError):
            with self.subTest(error_type=error_type):
                rig = Rig()
                safe = rig.use_safe_z_home()
                registered = rig.gcode.gcode_handlers["G28"]
                error = error_type("synthetic stock homing failure")
                rig.home_results.append(error)
                with self.assertRaises(error_type) as caught:
                    rig.calibrate(save=0)
                self.assertIs(caught.exception, error)
                self.assertFalse(rig.capture_active)
                self.assertEqual(rig.capture_stops, 1)
                self.assertIsNone(rig.workflow.session)
                self.assertFalse(rig.workflow.in_home)
                self.assertIsNone(rig.workflow.home_axis)
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                self.assertEqual(rig.owner.back_off_dist, 5.0)
                self.assertEqual(rig.gcode.gcode_handlers["G28"], registered)
                self.assertEqual(safe.calls, [])
                self.assertEqual(rig.toolhead.moves, [])
                if error_type is CommandError:
                    # Discovery now invokes stock G28, just like validation.
                    # Preserve its fatal all-motor-off policy; do not suppress
                    # the firmware's cleanup to keep Z enabled after a fault.
                    self.assertIn("stepper_z", rig.disabled)
                    self.assertFalse(rig.lines["stepper_z"].enabled)

    def test_invalid_internal_home_axis_is_rejected_without_dispatch_or_capture(self):
        for axis in ("", "z", "xy", "X", None):
            rig = Rig()
            before = rig.effects()
            with self.assertRaisesRegex(CommandError, "exactly one X or Y"):
                rig.workflow._home(axis)
            self.assertEqual(rig.effects(), before)


class HomingOverrideWorkflowTests(unittest.TestCase):
    def test_native_override_is_bypassed_for_each_workflow_and_axes_setting(self):
        cases = (("calibrate_x", "XYZ"), ("calibrate_y", "Z"),
                 ("test", "XY"), ("charts", "X"), ("accuracy", "Y"))
        for mode, axes in cases:
            for z_homed in (False, True):
                with self.subTest(mode=mode, axes=axes, z_homed=z_homed), tempfile.TemporaryDirectory() as temp:
                    rig = Rig({"charts_directory": temp})
                    rig.toolhead.kin.limits[2] = (20.0, 480.0) if z_homed else (1.0, -1.0)
                    override = rig.use_homing_override(axes, start_pos=(11.0, 22.0, 33.0))
                    registries = (rig.gcode.ready_gcode_handlers, rig.gcode.gcode_handlers)
                    registered, previous = dict(registries[0]), override.prev_G28
                    z_limit = rig.toolhead.kin.limits[2]
                    with mock.patch.object(rig.toolhead, "set_position", wraps=rig.toolhead.set_position) as force:
                        if mode.startswith("calibrate"):
                            rig.calibrate(mode[-1].upper(), save=0)
                            expected = [mode[-1].upper()] * 11
                        elif mode == "accuracy":
                            rig.workflow.cmd_ACCURACY(GCmd(CENTERED=1, SAMPLES=2))
                            expected = ["X"] * 3 + ["Y"] * 3
                        else:
                            getattr(rig.workflow, "cmd_TEST" if mode == "test" else "cmd_CHARTS")(
                                GCmd(CENTERED=1))
                            expected = ["X", "Y"]
                        force.assert_not_called()
                    self.assertEqual(override.calls, [])
                    override.template.create_template_context.assert_not_called()
                    override.template.run_gcode_from_command.assert_not_called()
                    self.assertEqual(rig.dispatched_scripts, [])
                    self.assertEqual(rig.created_commands,
                                     [("G28", "G28 " + axis, {axis: ""}) for axis in expected])
                    self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
                    self.assertEqual(rig.toolhead.kin.limits[2], z_limit)
                    self.assertTrue(all(coords[2] is None for coords, _speed in rig.toolhead.moves))
                    self.assertTrue(rig.lines["stepper_z"].enabled)
                    self.assertTrue(rig.lines["stepper_z1"].enabled)
                    self.assertEqual(rig.z_homes, [])
                    self.assertFalse(override.in_script)
                    self.assertIs(override.prev_G28, previous)
                    self.assertIs(rig.objects["homing_override"], override)
                    self.assertIs(rig.gcode.ready_gcode_handlers, registries[0])
                    self.assertIs(rig.gcode.gcode_handlers, registries[1])
                    self.assertEqual(rig.gcode.ready_gcode_handlers, registered)
                    self.assertIsNone(rig.workflow.session)
                    self.assertFalse(rig.capture_active)

    def test_normal_public_override_still_forces_position_and_runs_its_sync_z_sequence(self):
        rig = Rig()
        override = rig.use_homing_override(start_pos=(None, None, 7.5))
        handler = rig.gcode.gcode_handlers["G28"]
        rig.workflow.cmd_TEST(GCmd(CENTERED=1))

        def user_sequence(context):
            self.assertTrue(override.in_script)
            self.assertEqual(context["params"], {})
            self.assertEqual(context["rawparams"], "")
            self.assertEqual(rig.toolhead.position[2], 7.5)
            rig.run_script("G28 X")
            rig.run_script("G28 Y")
            rig.scripts.append("SYNC_MOTORS")
            rig.toolhead.manual_move([140.0, 90.0, None], 50.0)
            rig.run_script("G28 Z")
            rig.toolhead.manual_move([None, None, 12.0], 15.0)

        override.template.run_gcode_from_command.side_effect = user_sequence
        with mock.patch.object(rig.toolhead, "set_position", wraps=rig.toolhead.set_position) as force:
            rig.run_script("G28")
            force.assert_called_once()
        self.assertEqual(override.calls, ["G28", "G28 X", "G28 Y", "G28 Z"])
        override.template.create_template_context.assert_called_once_with()
        override.template.run_gcode_from_command.assert_called_once()
        self.assertEqual(rig.scripts[-3:], ["G28 X", "G28 Y", "SYNC_MOTORS"])
        self.assertEqual(len(rig.z_homes), 1)
        self.assertEqual(rig.toolhead.position, [140.0, 90.0, 12.0, 6.0])
        self.assertFalse(override.in_script)
        self.assertEqual(rig.gcode.gcode_handlers["G28"], handler)
        self.assertEqual(rig.gcode.ready_gcode_handlers["G28"], handler)

    def test_unknown_and_nested_override_predecessors_fail_before_effects(self):
        for kind in ("function", "other_owner", "override", "safe_z_home", "missing"):
            with self.subTest(kind=kind):
                rig = Rig()
                override = rig.use_homing_override()
                if kind == "function":
                    override.prev_G28 = lambda gcmd: None
                elif kind == "other_owner":
                    override.prev_G28 = StockHoming(rig).cmd_G28
                elif kind == "override":
                    override.prev_G28 = HomingOverride(rig).cmd_G28
                elif kind == "safe_z_home":
                    override.prev_G28 = SafeZHoming(rig).cmd_G28
                else:
                    del override.prev_G28
                before = rig.effects()
                with self.assertRaisesRegex(CommandError, "stock G28 predecessor"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.effects(), before)
                self.assertEqual(override.calls, [])
                override.template.run_gcode_from_command.assert_not_called()

    def test_custom_override_owner_methods_and_missing_native_import_fail_closed(self):
        class OverrideSubclass(HomingOverride):
            pass

        for kind in ("subclass", "object", "owner", "dispatcher", "instance_method",
                     "class_method", "missing_import"):
            with self.subTest(kind=kind):
                rig = Rig()
                override = rig.use_homing_override()
                if kind == "subclass":
                    override = OverrideSubclass(rig)
                    rig.objects["homing_override"] = override
                    rig.gcode.ready_gcode_handlers["G28"] = override.cmd_G28
                elif kind == "object":
                    rig.objects["homing_override"] = types.SimpleNamespace(
                        printer=rig, gcode=rig.gcode, cmd_G28=override.cmd_G28,
                        prev_G28=override.prev_G28, in_script=False)
                elif kind == "owner":
                    override.printer = object()
                elif kind == "dispatcher":
                    override.gcode = object()
                elif kind == "instance_method":
                    override.cmd_G28 = lambda gcmd: None
                if kind == "class_method":
                    context = mock.patch.object(HomingOverride, "cmd_G28", lambda obj, gcmd: None)
                elif kind == "missing_import":
                    context = mock.patch.object(workflow_module, "_HomingOverride", None)
                else:
                    context = contextlib.nullcontext()
                before = rig.effects()
                with context, self.assertRaisesRegex(CommandError, "homing_override.*stock G28 predecessor"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.effects(), before)
                self.assertEqual(override.calls, [])

    def test_override_and_safe_z_home_cannot_be_combined_in_either_order(self):
        for first in ("safe_z_home", "homing_override"):
            rig = Rig()
            if first == "safe_z_home":
                rig.use_safe_z_home()
                override = rig.use_homing_override()
            else:
                override = rig.use_homing_override()
                rig.use_safe_z_home()
            before = rig.effects()
            with self.assertRaisesRegex(CommandError, "cannot combine safe_z_home and homing_override"):
                rig.calibrate(save=0)
            self.assertEqual(rig.effects(), before)
            override.template.run_gcode_from_command.assert_not_called()

    def test_active_missing_or_divergent_override_registrations_are_rejected(self):
        for kind in ("ready_missing", "active_missing", "ready_empty", "active_empty",
                     "unknown_ready", "unknown_active", "stock_ready", "stock_active"):
            with self.subTest(kind=kind):
                rig = Rig()
                override = rig.use_homing_override()
                if kind == "ready_missing":
                    del rig.gcode.ready_gcode_handlers
                elif kind == "active_missing":
                    del rig.gcode.gcode_handlers
                elif kind == "ready_empty":
                    rig.gcode.ready_gcode_handlers = {}
                elif kind == "active_empty":
                    rig.gcode.gcode_handlers = {}
                elif kind == "unknown_ready":
                    rig.gcode.ready_gcode_handlers = {"G28": lambda gcmd: None}
                elif kind == "unknown_active":
                    rig.gcode.gcode_handlers = {"G28": lambda gcmd: None}
                elif kind == "stock_ready":
                    rig.gcode.ready_gcode_handlers = {"G28": rig.objects["homing"].cmd_G28}
                else:
                    rig.gcode.gcode_handlers = {"G28": rig.objects["homing"].cmd_G28}
                before = rig.effects()
                with self.assertRaisesRegex(CommandError, "G28"):
                    rig.calibrate(save=0)
                self.assertEqual(rig.effects(), before)
                self.assertEqual(override.calls, [])

    def test_active_or_unverifiable_override_recursion_is_refused_even_with_valid_xy(self):
        for state in (True, None, 0, "", "missing"):
            for xy_homed in (False, True):
                with self.subTest(state=state, xy_homed=xy_homed):
                    rig = Rig()
                    override = rig.use_homing_override()
                    if state == "missing":
                        del override.in_script
                    else:
                        override.in_script = state
                    if xy_homed:
                        rig.toolhead.kin.limits[:2] = [(0.0, 300.0), (0.0, 200.0)]
                    before = rig.effects()
                    with self.assertRaisesRegex(CommandError, "active or unverifiable homing_override G28 script"):
                        rig.calibrate(save=0)
                    self.assertEqual(rig.effects(), before)
                    override.template.run_gcode_from_command.assert_not_called()

    def test_override_is_revalidated_after_capture_setup_and_command_creation(self):
        for stage in ("capture", "command"):
            for mutation in ("previous", "active", "recursion"):
                with self.subTest(stage=stage, mutation=mutation):
                    rig = Rig()
                    override = rig.use_homing_override()
                    def mutate_after(callback, *args):
                        result = callback(*args)
                        if mutation == "previous":
                            override.prev_G28 = lambda gcmd: None
                        elif mutation == "active":
                            rig.gcode.gcode_handlers = {"G28": lambda gcmd: None}
                        else:
                            override.in_script = True
                        return result
                    if stage == "capture":
                        callback = rig.owner.start_capture
                        rig.owner.start_capture = lambda: mutate_after(callback)
                    else:
                        callback = rig.gcode.create_gcode_command
                        rig.gcode.create_gcode_command = lambda *args: mutate_after(callback, *args)
                    with self.assertRaisesRegex(CommandError, "G28"):
                        rig.calibrate(save=0)
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(override.calls, [])
                    override.template.run_gcode_from_command.assert_not_called()
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.capture_stops, 1)
                    self.assertFalse(rig.capture_active)
                    self.assertIsNone(rig.workflow.session)
                    self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                    self.assertEqual(rig.owner.back_off_dist, 5.0)
                    self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_override_chain_is_revalidated_between_trials_and_manual_center_wait(self):
        for mode in ("calibration", "test", "waiting"):
            for mutation in ("previous", "active", "recursion"):
                with self.subTest(mode=mode, mutation=mutation):
                    rig = Rig()
                    override = rig.use_homing_override()
                    def mutate(axis=None):
                        if mutation == "previous":
                            override.prev_G28 = lambda gcmd: None
                        elif mutation == "active":
                            rig.gcode.gcode_handlers = {"G28": lambda gcmd: None}
                        else:
                            override.in_script = True
                    if mode == "waiting":
                        rig.home_results.append({"travel_mm": 2.0})
                        rig.wait_calibration(save=0)
                        mutate()
                        resume = lambda: rig.calibrate(save=0)
                    else:
                        rig.on_home = mutate
                        resume = (lambda: rig.workflow.cmd_TEST(GCmd(CENTERED=1))) \
                            if mode == "test" else (lambda: rig.calibrate(save=0))
                    with self.assertRaisesRegex(CommandError, "G28"):
                        resume()
                    self.assertEqual(len(rig.home_calls), 1)
                    self.assertEqual(override.calls, [])
                    override.template.run_gcode_from_command.assert_not_called()
                    self.assertTrue(all(coords[2] is None for coords, _speed in rig.toolhead.moves))
                    self.assertFalse(rig.capture_active)
                    self.assertIsNone(rig.workflow.session)
                    self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                    self.assertEqual(rig.owner.back_off_dist, 5.0)
                    self.assertEqual(rig.workflow.last_result["result"], "aborted")


class CalibrationTests(unittest.TestCase):
    def test_first_trial_is_discovery_impact_and_is_excluded_from_validation(self):
        rig = Rig()
        # Unhomed reported coordinates do not establish the physical center.
        rig.toolhead.position[:2] = [12.0, 19.0]
        rig.calibrate(save=0)
        first = rig.home_calls[0]
        self.assertEqual(first["threshold"], 1.0)
        self.assertEqual(first["homed_axes"], "z")
        self.assertEqual(first["position"], [12.0, 19.0, 40.0, 6.0])
        self.assertEqual(first["attempt"], 1)
        self.assertFalse(first["candidate_qualified"])
        self.assertTrue(rig.home_calls[1]["candidate_qualified"])
        self.assertEqual(rig.home_calls[1]["attempt"], 2)
        self.assertEqual(rig.home_calls[1]["successes"], 0)
        self.assertEqual(rig.scripts, ["G28 X"] * 11)
        self.assertEqual(rig.disabled, [])
        self.assertEqual(rig.toolhead.dwells, [.25] * 10)
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
        self.assertFalse(any("action:prompt_begin" in line for line in rig.output))
        self.assertFalse(any("TOKEN" in line for line in rig.output))

    def test_ten_independent_axis_homes_are_saved_and_original_settings_restored(self):
        for axis in "XY":
            rig = Rig()
            rig.calibrate(axis)
            self.assertEqual([call["threshold"] for call in rig.home_calls], [1.0] * 11)
            self.assertEqual(rig.scripts, ["G28 " + axis] * 11 + ["SAVE_CONFIG"])
            self.assertEqual(len(rig.toolhead.moves), 11)
            index = "XY".index(axis)
            for coords, speed in rig.toolhead.moves[:-1]:
                self.assertEqual(coords[2], None)
                self.assertEqual(coords[1 - index], None)
                self.assertEqual(coords[index], (150.0, 100.0)[index])
                self.assertEqual(speed, 50.0)
            terminal = [None, None, None]
            terminal[index] = 5.0
            self.assertEqual(rig.toolhead.moves[-1], (terminal, 100.0))
            values = rig.configfile.saved
            self.assertEqual(values[("accel_homing_model default", axis.lower() + "_successes")], "10")
            self.assertNotIn(("accel_homing_model default", "XY"[1 - index].lower() + "_threshold"), values)
            self.assertNotIn(("accel_homing", "x_threshold"), values)
            self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
            self.assertEqual(rig.owner.back_off_dist, 5.0)
            self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
            self.assertFalse(rig.capture_active)
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.workflow.last_result["successes"], 10)
            metadata = json.loads(values[("accel_homing_model default", axis.lower() + "_metadata")])
            self.assertEqual(len(metadata["onset_g"]), 10)
            self.assertFalse(metadata["physical_position_verified"])

    def test_short_false_home_returns_then_same_command_retries_without_manual_wait(self):
        for short in (0.0, 10.0, 19.0, 19.999):
            rig = Rig()
            rig.home_results.append({"travel_mm": short})
            rig.calibrate()
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.home_calls[0]["threshold"], 1.0)
            self.assertEqual(rig.disabled, [])
            self.assertTrue(rig.lines["stepper_z"].enabled)
            self.assertTrue(rig.lines["stepper_z1"].enabled)
            self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
            self.assertFalse(any("Move XY manually" in text for text in rig.output))
            if short:
                self.assertEqual(rig.toolhead.moves[0], ([short, None, None], 50.))
            self.assertEqual(rig.home_calls[1]["threshold"], 1.25)
            self.assertEqual(rig.home_calls[1]["attempt"], 2)
            self.assertEqual(rig.home_calls[1]["homed_axes"], "z")
            self.assertEqual(rig.scripts, ["G28 X"] * 12 + ["SAVE_CONFIG"])
            self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)

    def test_pressed_result_is_never_a_success_even_with_apparent_step_travel(self):
        rig = Rig()
        rig.home_results.append({"result": "pressed", "travel_mm": 150.0, "moving": False})
        with self.assertRaisesRegex(CommandError, "sensor/homing failure"):
            rig.calibrate()
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.toolhead.moves, [])
        self.assertIsNone(rig.workflow.session)

    def test_success_count_resets_after_threshold_changes(self):
        rig = Rig()
        rig.home_results.extend([{}, {}, {"travel_mm": 8.0}])
        rig.calibrate()
        self.assertEqual(len(rig.toolhead.moves), 14)
        self.assertEqual(rig.home_calls[3]["successes"], 0)
        self.assertFalse(rig.home_calls[3]["candidate_qualified"])
        self.assertEqual(rig.home_calls[3]["attempt"], 4)
        self.assertEqual(rig.workflow.last_result["successes"], 10)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
        self.assertEqual(len(rig.scripts), 15)

    def test_faults_or_missing_step_travel_abort_without_threshold_escalation(self):
        for bad in ({"travel_mm": None}, {"result": "no_data"}, {"travel_mm": math.nan}):
            rig = Rig()
            rig.home_results.append(bad)
            with self.assertRaises(CommandError):
                rig.calibrate()
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.owner.threshold_g["x"], 8.0)
            self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")

    def test_sensor_headroom_and_attempt_budget_abort(self):
        for ceiling in ("sensor_scale", "attempts"):
            rig = Rig()
            if ceiling == "sensor_scale":
                rig.owner.full_scale_g = 0.625
            if ceiling == "attempts":
                rig.workflow.max_attempts = 1
            rig.home_results.append({"travel_mm": 2.0})
            with self.assertRaisesRegex(CommandError, "headroom|maximum|attempt budget"):
                rig.calibrate()
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.toolhead.moves, [])

    def test_no_data_and_incomplete_capture_abort_and_close_capture(self):
        for result in (CommandError("no accelerometer data"), None):
            rig = Rig()
            if result:
                rig.home_results.append(result)
            else:
                rig.bad_capture = {"samples": [], "dropped": 0}
            with self.assertRaises(CommandError):
                rig.calibrate()
            self.assertFalse(rig.capture_active)
            self.assertEqual(rig.owner.back_off_dist, 5.0)
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.toolhead.moves, [])

    def test_unhomed_positive_and_negative_axes_use_actual_geometry_without_touching_z(self):
        for axis in "xy":
            for positive in (False, True):
                rig = Rig()
                rig.owner.rails_by_axis[axis] = Rail(positive=positive, bounds=(-40.0, 220.0))
                rig.calibrate(axis.upper())
                coords = [None, None, None]
                coords["xy".index(axis)] = 90.0
                self.assertEqual(rig.toolhead.moves[0][0], coords)
                self.assertEqual(rig.home_calls[0]["homed_axes"], "z")
                self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
                self.assertEqual(rig.workflow.last_result["result"], "calibrated")

    def test_configured_start_remains_editable_above_minimum_and_is_used_first(self):
        for threshold in (0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 4.0):
            with self.subTest(threshold=threshold):
                rig = Rig({"calibration_start_threshold": threshold})
                rig.calibrate(save=0)
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [threshold] * 11)
        for threshold in (0.499, 0.0, -1.0,
                          float("nan"), float("inf"), float("-inf")):
            with self.subTest(threshold=threshold), self.assertRaises(ConfigError):
                Rig({"calibration_start_threshold": threshold})

    def test_successful_calibration_keeps_unhomed_z_unhomed(self):
        rig = Rig()
        rig.toolhead.kin.limits[2] = (1.0, -1.0)
        rig.calibrate(save=0)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "x")
        self.assertEqual(rig.toolhead.kin.limits[2], (1.0, -1.0))
        self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
        self.assertTrue(rig.lines["stepper_z"].enabled)
        self.assertTrue(rig.lines["stepper_z1"].enabled)

    def test_save_config_opt_out_stages_but_never_restarts(self):
        rig = Rig()
        rig.calibrate(save=0)
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertEqual(rig.workflow.last_result["result"], "calibrated")
        self.assertTrue(any("staged only" in text for text in rig.output))

    def test_save_config_error_restores_settings_and_clears_active_state(self):
        rig = Rig()
        rig.save_error = True
        with self.assertRaisesRegex(CommandError, "SAVE_CONFIG"):
            rig.calibrate()
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.owner.threshold_g["x"], 8.0)
        self.assertEqual(rig.owner.back_off_dist, 5.0)
        self.assertFalse(rig.capture_active)
        self.assertEqual(rig.workflow.last_result["result"], "save_failed")

    def test_config_staging_error_restores_everything(self):
        rig = Rig()
        rig.configfile.fail = True
        with self.assertRaisesRegex(CommandError, "staging"):
            rig.calibrate()
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.owner.threshold_g["x"], 8.0)
        self.assertFalse(rig.capture_active)


class CalibrationRecenterTests(unittest.TestCase):
    def machine(self, axis="x", mode="calibration", settle=.25, live_accel=60000., cap=5000.):
        rig = Rig({"calibration_settle_time": settle})
        rig.toolhead.accel = live_accel
        rig.owner.homing_accel = cap
        rig.workflow._begin(GCmd(SAVE_CONFIG=0), mode, axis)
        rig.workflow.session["state"] = "running"
        rig.toolhead.kin.limits["xy".index(axis)] = rig.owner.rails_by_axis[axis].get_range()
        if mode == "calibration":
            center = rig.workflow._geometry(axis)[0]
            record = {"axis": axis, "result": "impact", "travel_mm": center,
                      "impact_metric": "xyz_magnitude"}
            rig.owner.last[axis] = record
            rig.workflow.session["terminal_home"] = (axis, record, record)
        return rig

    def test_calibration_and_accuracy_return_with_same_normal_acceleration_and_settle(self):
        for mode in ("calibration", "accuracy"):
            for axis in ("x", "y"):
                for live, cap in ((60000., 5000.), (1000., 5000.), (60000., 0.)):
                    with self.subTest(mode=mode, axis=axis, live=live, cap=cap):
                        rig = self.machine(axis, mode=mode, live_accel=live, cap=cap)
                        toolhead, events = rig.toolhead, []
                        move, wait, dwell = toolhead.manual_move, toolhead.wait_moves, toolhead.dwell
                        def moving(coordinates, speed):
                            events.append(("move", toolhead.accel))
                            move(coordinates, speed)
                        def waiting():
                            events.append(("wait", toolhead.accel))
                            wait()
                        def dwelling(seconds):
                            events.append(("dwell", seconds, toolhead.accel))
                            dwell(seconds)
                        with mock.patch.object(toolhead, "manual_move", side_effect=moving), \
                                mock.patch.object(toolhead, "wait_moves", side_effect=waiting), \
                                mock.patch.object(toolhead, "dwell", side_effect=dwelling), \
                                mock.patch.object(rig.owner, "_limit_accel") as limit, \
                                mock.patch.object(rig.owner, "_restore_accel") as restore:
                            rig.workflow._recenter(axis)
                        self.assertEqual(events, [("move", live), ("wait", live),
                                                  ("dwell", .25, live), ("wait", live)])
                        limit.assert_not_called()
                        restore.assert_not_called()
                        self.assertEqual(toolhead.accel, live)
                        self.assertEqual(toolhead.accel_log, [])
                        self.assertIsNone(rig.owner.saved_accel)
                        coordinates = [None, None, None]
                        coordinates["xy".index(axis)] = (150., 100.)["xy".index(axis)]
                        self.assertEqual(toolhead.moves, [(coordinates, 50.)])

    def test_test_and_chart_returns_keep_live_acceleration_without_added_dwell(self):
        for mode in ("test", "charts"):
            with self.subTest(mode=mode):
                rig = self.machine(mode=mode)
                with mock.patch.object(rig.owner, "_limit_accel") as limit, \
                        mock.patch.object(rig.owner, "_restore_accel") as restore:
                    rig.workflow._recenter("x")
                limit.assert_not_called()
                restore.assert_not_called()
                self.assertEqual(rig.toolhead.moves, [([150., None, None], 50.)])
                self.assertEqual(rig.toolhead.dwells, [])
                self.assertEqual(rig.toolhead.accel, 60000.)

    def test_zero_settle_keeps_normal_acceleration_and_setting_is_bounded_and_reported(self):
        rig = self.machine(settle=0.)
        with mock.patch.object(rig.toolhead, "wait_moves", wraps=rig.toolhead.wait_moves) as wait:
            rig.workflow._recenter("x")
        self.assertEqual(wait.call_count, 1)
        self.assertEqual(rig.toolhead.dwells, [])
        self.assertEqual(rig.toolhead.accel_log, [])
        self.assertEqual(rig.workflow.get_status(0)["active"]["calibration_settle_time_s"], 0.)
        for value in (-.001, 2.001, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                Rig({"calibration_settle_time": value})
        self.assertEqual(Rig({"calibration_settle_time": 2.}).workflow.calibration_settle_time, 2.)
        rig = Rig({"calibration_settle_time": .4})
        rig.calibrate(save=0)
        self.assertEqual(rig.toolhead.dwells, [.4] * 10)
        self.assertEqual(rig.workflow.last_result["calibration_settle_time_s"], .4)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertEqual(metadata["calibration_settle_time_s"], .4)

    def test_error_or_cancellation_at_return_yield_boundaries_prevents_next_trial(self):
        for failure in ("error", "cancel"):
            for stage in ("move", "return_wait", "dwell", "settle_wait"):
                with self.subTest(failure=failure, stage=stage):
                    rig = Rig()
                    rig.toolhead.accel = 60000.
                    inside, waits, reached = [False], [0], []
                    recenter, move, wait, dwell, end_session = (rig.workflow._recenter,
                        rig.toolhead.manual_move, rig.toolhead.wait_moves, rig.toolhead.dwell,
                        rig.owner._end_session)
                    def interrupt(here):
                        reached.append(here)
                        if here != stage:
                            return
                        if failure == "error":
                            raise RuntimeError("synthetic return failure")
                        rig.workflow.cmd_CANCEL(GCmd())
                    def centering(axis):
                        inside[0] = True
                        try:
                            return recenter(axis)
                        finally:
                            inside[0] = False
                    def moving(coordinates, speed):
                        move(coordinates, speed)
                        if inside[0]:
                            interrupt("move")
                    def waiting():
                        wait()
                        if inside[0]:
                            waits[0] += 1
                            interrupt("return_wait" if waits[0] == 1 else "settle_wait")
                    def dwelling(seconds):
                        dwell(seconds)
                        if inside[0]:
                            interrupt("dwell")
                    def ending():
                        end_session()
                        # Real core _end_session restores the homing cap before
                        # _recenter; cancellation must not apply another cap.
                        rig.owner._restore_accel()
                    with mock.patch.object(rig.workflow, "_recenter", side_effect=centering), \
                            mock.patch.object(rig.toolhead, "manual_move", side_effect=moving), \
                            mock.patch.object(rig.toolhead, "wait_moves", side_effect=waiting), \
                            mock.patch.object(rig.toolhead, "dwell", side_effect=dwelling), \
                            mock.patch.object(rig.owner, "_end_session", side_effect=ending), \
                            self.assertRaises(RuntimeError if failure == "error" else CommandError):
                        rig.calibrate(save=0)
                    order = ["move", "return_wait", "dwell", "settle_wait"]
                    self.assertEqual(reached, order[:order.index(stage) + 1])
                    self.assertEqual(rig.scripts, ["G28 X"])
                    self.assertEqual(rig.toolhead.moves, [([150., None, None], 50.)])
                    self.assertEqual(rig.toolhead.accel, 60000.)
                    self.assertIsNone(rig.owner.saved_accel)
                    self.assertEqual(rig.toolhead.accel_log, [])
                    self.assertIsNone(rig.workflow.session)
                    self.assertEqual(rig.configfile.saved, {})

    def test_synthetic_settling_reduces_decaying_reference_without_accepting_persistent_noise(self):
        # Tie the actual recenter dwell/wait lifecycle to the actual generic
        # reference and motion checks. This models ringdown, not printer data.
        from test_backends import generic, feed
        from test_motion_policy import SignedRamp
        from test_session import FakeRail, info
        from harness import FakeEndstop
        g, rate = 9806.65, 3200.
        outcomes = {}
        for profile in ("healthy", "decaying", "persistent"):
            for settling in (0., .25):
                rig = self.machine(settle=settling)
                queued_until = [10.]  # Synthetic completed recenter time.
                def waiting():
                    rig.reactor.now = queued_until[0]
                def dwelling(seconds):
                    queued_until[0] += seconds
                with mock.patch.object(rig.toolhead, "wait_moves", side_effect=waiting), \
                        mock.patch.object(rig.toolhead, "dwell", side_effect=dwelling):
                    rig.workflow._recenter("x")
                self.assertEqual(rig.reactor.now, 10. + settling)
                obj = generic()
                obj.threshold_g["x"] = 1.5
                obj.toolhead.get_max_velocity = lambda: (500., 5000.)
                obj.reactor.now = rig.reactor.now
                obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x"}
                obj.workflows.in_home, obj.workflows.home_axis = True, "x"
                endstop = FakeEndstop("x")
                obj.session_axes["x"] = FakeRail(endstop, info(speed=50., retract=0, second=50.))
                def vibration(when):
                    elapsed = when - 10.
                    amplitude = .02 if profile == "healthy" else .70
                    if profile == "decaying":
                        amplitude *= math.exp(-elapsed / .080)
                    return amplitude * math.sin(2 * math.pi * 80 * elapsed)
                def reference_batch():
                    first = (obj.preparation.cutoff if obj.backend.last_sample_time is None
                             else obj.backend.last_sample_time) + 1 / rate
                    obj.backend.helper.emit({"data": [
                        (first + j / rate, vibration(first + j / rate) * g, 0., g) for j in range(16)]})
                obj.reactor.on_pause = reference_batch
                obj._prepare_reference()
                bound = obj.baseline_status["motion_proof_noise_bound_g"]["x"]
                self.assertGreaterEqual(obj.preparation.last - obj.preparation.first + 1e-9, .1)
                obj.reactor.on_pause = None
                arm = round(obj.backend.last_sample_time + 1 / rate, 6)
                endstop.get_steppers = lambda: [SignedRamp(arm, 5000., 50., 1)]
                attempt = obj.prepare(endstop, arm)
                obj.arm(attempt)
                for number in range(0, 480, 16):
                    rows = []
                    for j in range(16):
                        offset = (number + j) / rate
                        when = arm + offset
                        acceleration = 5000. if offset < .010 else 0.
                        rows.append((when, acceleration + vibration(when) * g, 0., g))
                    feed(obj, rows, age=.001)
                    obj._watchdog(obj.reactor.monotonic())
                    if attempt.done:
                        break
                self.assertIsNone(obj.sensor_fault)
                self.assertFalse(attempt.gap)
                outcomes[profile, settling] = (attempt.motion_seen, attempt.reason, bound)
        for settling in (0., .25):
            self.assertEqual(outcomes["healthy", settling][:2], (True, None))
            self.assertEqual(outcomes["persistent", settling][:2], (False, "no_motion"))
        self.assertEqual(outcomes["decaying", 0.][:2], (False, "no_motion"))
        self.assertEqual(outcomes["decaying", .25][:2], (True, None))
        self.assertGreater(outcomes["decaying", 0.][2], .5)
        self.assertLess(outcomes["decaying", .25][2], .1)
        self.assertAlmostEqual(outcomes["persistent", 0.][2], outcomes["persistent", .25][2])


class CalibrationGridTests(unittest.TestCase):
    def test_default_grid_and_explicit_configuration_apply_only_to_requested_axis(self):
        for axis in ("X", "Y"):
            for options, start, step in (({}, 1.0, .25),
                    ({"calibration_start_threshold": .5, "calibration_step": .5}, .5, .5)):
                with self.subTest(axis=axis, options=options):
                    rig = Rig(options)
                    rig.home_results.append({"travel_mm": 2.0})
                    rig.wait_calibration(axis, save=0)
                    self.assertEqual(rig.scripts, ["G28 " + axis])
                    self.assertEqual(rig.home_calls[0]["threshold"], start)
                    self.assertEqual(rig.workflow.session["candidate"], start + step)
                    self.assertEqual(rig.toolhead.moves, [])
                    status = rig.workflow.get_status(0)["active"]
                    self.assertEqual(status["start_threshold_g"], start)
                    self.assertEqual(status["threshold_step_g"], step)
                    self.assertEqual(status["attempt_limit"], 40)
                    self.assertFalse(status["quiet_range_enforced"])
                    self.assertFalse(status["explicit_calibration_grid"])
                    self.assertFalse(rig.workflow.calibration_reference(axis.lower()))

    def test_explicit_grid_survives_automatic_retries_and_does_not_change_configuration(self):
        for axis in ("X", "Y"):
            with self.subTest(axis=axis):
                rig = Rig({"calibration_start_threshold": .5, "calibration_step": .5})
                rig.home_results.extend([{"travel_mm": 2.0}, {"travel_mm": 2.0}])
                contexts = []
                def observe(current):
                    contexts.append((rig.workflow.calibration_reference(current),
                                     rig.workflow.calibration_reference("y" if current == "x" else "x")))
                rig.on_home = observe
                rig.calibrate(axis, save=0, START_THRESHOLD=1, STEP=.25)
                self.assertIsNone(rig.workflow.session)
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [1.0, 1.25] + [1.5] * 11)
                self.assertEqual(rig.scripts, ["G28 " + axis] * 13)
                self.assertEqual(contexts, [(True, False)] * 13)
                self.assertEqual(rig.workflow.start_threshold, .5)
                self.assertEqual(rig.workflow.threshold_step, .5)
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                self.assertFalse(rig.workflow.calibration_reference(axis.lower()))
                other = "y" if axis == "X" else "x"
                self.assertNotIn(("accel_homing_model default", other + "_threshold"), rig.configfile.saved)
                for coordinates, _speed in rig.toolhead.moves:
                    self.assertIsNone(coordinates["xy".index(other)])
                    self.assertIsNone(coordinates[2])
                result = rig.workflow.last_result
                self.assertEqual(result["start_threshold_g"], 1.0)
                self.assertEqual(result["threshold_step_g"], .25)
                self.assertFalse(result["quiet_range_enforced"])
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", axis.lower() + "_metadata")])
                self.assertEqual(metadata["calibration_start_threshold_g"], 1.0)
                self.assertEqual(metadata["calibration_step_g"], .25)
                self.assertFalse(metadata["calibration_quiet_range_enforced"])

    def test_invalid_grid_is_rejected_before_preflight_or_workflow_effects(self):
        bad = [{"START_THRESHOLD": value} for value in (.499, 0, -1, 12.800001, 16, "nan", "inf", "bad")]
        bad += [{"STEP": value} for value in (.099, 0, -1, 3.01, "nan", "inf", "bad")]
        for parameters in bad:
            with self.subTest(parameters=parameters):
                rig = Rig()
                before = rig.effects()
                with mock.patch.object(rig.workflow, "_preflight") as preflight, self.assertRaises(CommandError):
                    rig.calibrate(save=0, **parameters)
                preflight.assert_not_called()
                self.assertEqual(rig.effects(), before)
                self.assertIsNone(rig.workflow.session)
                self.assertFalse(rig.capture_active)
        rig = Rig()
        rig.owner.full_scale_g = 2.0
        with self.assertRaisesRegex(CommandError, "ceiling"):
            rig.calibrate(save=0, START_THRESHOLD=1.7)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.scripts, [])

    def test_resume_cannot_change_original_grid_even_to_current_candidate(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 2.0})
        rig.wait_calibration(save=0, START_THRESHOLD=1, STEP=.25)
        session = rig.workflow.session
        for parameters, message in (({"START_THRESHOLD": 1.25}, "START_THRESHOLD cannot change"),
                                    ({"STEP": .5}, "STEP cannot change")):
            before = rig.effects()
            with self.assertRaisesRegex(CommandError, message):
                rig.calibrate(**parameters)
            self.assertEqual(rig.effects(), before)
            self.assertIs(rig.workflow.session, session)
        rig.calibrate(START_THRESHOLD=1, STEP=.25)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)

    def test_calibration_reference_requires_every_trial_context_condition(self):
        rig = Rig()
        workflow = rig.workflow
        valid = {"mode": "calibration", "state": "running", "axis": "x"}
        workflow.session, workflow.in_home, workflow.home_axis = dict(valid), True, "x"
        self.assertIs(workflow.calibration_reference("x"), True)
        for key, values in (("mode", ("test", "charts", "accuracy", "noise")),
                            ("state", ("starting", "waiting")), ("axis", ("y", None))):
            for value in values:
                workflow.session = dict(valid, **{key: value})
                self.assertIs(workflow.calibration_reference("x"), False)
        workflow.session = dict(valid)
        for in_home, home_axis in ((False, "x"), (1, "x"), (True, "y"), (True, None)):
            workflow.in_home, workflow.home_axis = in_home, home_axis
            self.assertIs(workflow.calibration_reference("x"), False)
        workflow.in_home, workflow.home_axis, workflow.session = True, "x", None
        self.assertIs(workflow.calibration_reference("x"), False)

    def test_failed_trial_clears_bypass_and_reports_chosen_grid(self):
        rig = Rig()
        error = CommandError("synthetic valid-data failure")
        rig.home_results.append(error)
        rig.on_home = lambda axis: self.assertTrue(rig.workflow.calibration_reference(axis))
        with self.assertRaises(CommandError) as caught:
            rig.calibrate("Y", save=0, START_THRESHOLD=1, STEP=.25)
        self.assertIs(caught.exception, error)
        self.assertFalse(rig.workflow.calibration_reference("y"))
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.workflow.last_result["result"], "aborted")
        self.assertEqual(rig.workflow.last_result["start_threshold_g"], 1.0)
        self.assertEqual(rig.workflow.last_result["threshold_step_g"], .25)
        self.assertFalse(rig.workflow.last_result["quiet_range_enforced"])
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
        self.assertEqual(rig.configfile.saved, {})


class RepeatabilityCalibrationTests(unittest.TestCase):
    def test_exact_two_mm_range_passes_including_decimal_endpoints(self):
        for axis, low, high in (("X", 150.0, 152.0), ("X", 150.1, 152.1),
                                ("X", 127.1, 129.1), ("Y", 100.0, 102.0)):
            with self.subTest(axis=axis, low=low, high=high):
                rig = Rig()
                rig.home_results.extend([{}] + [{"travel_mm": low}] * 9 + [{"travel_mm": high}])
                rig.calibrate(axis, save=0)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1.0] * 11)
                self.assertEqual(rig.workflow.last_result["repeatability_spread_mm"], 2.0)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["result"], "calibrated")
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                self.assertTrue(rig.lines["stepper_z"].enabled)
                self.assertFalse(rig.capture_active)

    def test_just_below_limit_passes_all_ten_and_records_spread_for_both_save_modes(self):
        high = 151.999
        for save in (0, 1):
            with self.subTest(save=save):
                rig = Rig()
                rig.home_results.extend([{}] + [{"travel_mm": 150.0}] * 9 + [{"travel_mm": high}])
                rig.calibrate(save=save)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertEqual(len(rig.toolhead.moves), 11)
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
                self.assertEqual(metadata["min_commanded_travel_mm"], 150.0)
                self.assertEqual(metadata["max_commanded_travel_mm"], high)
                self.assertLess(metadata["repeatability_spread_mm"], 2.0)
                self.assertEqual(metadata["repeatability_limit_mm"], 2.0)
                self.assertEqual(rig.workflow.last_result["repeatability_spread_mm"],
                                 metadata["repeatability_spread_mm"])
                self.assertEqual(rig.workflow.last_result["repeatability_limit_mm"], 2.0)
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), save)
                self.assertTrue(any("<= 2.00 mm" in message for message in rig.output))

    def test_ninth_check_outlier_is_rejected_immediately_then_restarts_discovery(self):
        for axis, center in (("X", 150.0), ("Y", 100.0)):
            for offset in (-2.001, 2.001):
                with self.subTest(axis=axis, offset=offset):
                    rig = Rig()
                    rig.home_results.extend([{}] + [{"travel_mm": center}] * 8 +
                                            [{"travel_mm": center + offset}, {}] +
                                            [{"travel_mm": center}] * 10)
                    rig.calibrate(axis, save=0)
                    self.assertEqual([c["threshold"] for c in rig.home_calls], [1.0] * 10 + [1.25] * 11)
                    self.assertFalse(rig.home_calls[10]["candidate_qualified"])
                    self.assertEqual(rig.home_calls[10]["successes"], 0)
                    self.assertEqual(rig.home_calls[11]["successes"], 0)
                    self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
                    self.assertEqual(rig.workflow.last_result["restarts"], 1)

    def test_reported_three_point_six_two_five_range_restarts_with_an_empty_window(self):
        travels = [152.0, 152.034, 153.713, 153.075, 151.981,
                   153.728, 155.606, 154.919, 153.347, 152.003]
        self.assertEqual(AccelHomingWorkflows._repeatability_spread(
            [{"travel_mm": value} for value in travels]), 3.625)
        rig = Rig()
        # The seventh validation sample first takes the running spread above
        # 2 mm; the remaining historic samples must never run at this threshold.
        rig.home_results.extend([{}] + [{"travel_mm": value} for value in travels[:7]] +
                                [{}] + [{"travel_mm": 149.0}] * 10)
        rig.calibrate(save=0)
        self.assertEqual(len(rig.home_calls), 19)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
        self.assertEqual(rig.workflow.last_result["repeatability_spread_mm"], 0.0)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertEqual(metadata["min_commanded_travel_mm"], 149.0)
        self.assertEqual(metadata["max_commanded_travel_mm"], 149.0)

    def test_discovery_and_manual_center_are_excluded_from_ten_distinct_validation_homes(self):
        for axis, travel in (("X", 150.0), ("Y", 100.0)):
            with self.subTest(axis=axis):
                rig = Rig()
                rig.toolhead.position[:2] = [12.0, 19.0]
                rig.home_results.extend([{"travel_mm": travel + 4.0}] +
                                         [{"travel_mm": travel}] * 10)
                save_model = rig.workflow._save_model
                inspected = []

                def inspect_then_save():
                    session = rig.workflow.session
                    qualified = session["validation_records"]
                    self.assertEqual(len(qualified), 10)
                    self.assertEqual(len({id(record) for record in qualified}), 10)
                    self.assertEqual([record["workflow_attempt"] for record in qualified],
                                     list(range(2, 12)))
                    self.assertEqual(session["candidate_attempt"], 1)
                    self.assertIsNot(qualified[0], session["records"][0])
                    inspected.append(True)
                    save_model()

                with mock.patch.object(rig.workflow, "_save_model", side_effect=inspect_then_save):
                    rig.calibrate(axis, save=0)
                self.assertEqual(inspected, [True])
                self.assertEqual([call["successes"] for call in rig.home_calls], [0, 0] + list(range(1, 10)))
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", axis.lower() + "_metadata")])
                self.assertEqual(metadata["min_commanded_travel_mm"], travel)
                self.assertEqual(metadata["max_commanded_travel_mm"], travel)
                self.assertEqual(metadata["repeatability_spread_mm"], 0.0)

    def test_axes_keep_independent_windows_and_saved_repeatability_metadata(self):
        rig = Rig()
        rig.home_results.extend([{}] + [{"travel_mm": 150.0}] * 9 + [{"travel_mm": 151.4}])
        rig.calibrate("X", save=0)
        x_metadata = rig.configfile.saved[("accel_homing_model default", "x_metadata")]
        rig.workflow._disable_xy()
        rig.idle_state = "Ready"
        rig.home_results.extend([{}] + [{"travel_mm": 100.0}] * 9 + [{"travel_mm": 100.5}])
        rig.calibrate("Y", save=0)
        self.assertEqual(rig.configfile.saved[("accel_homing_model default", "x_metadata")], x_metadata)
        self.assertEqual(json.loads(x_metadata)["repeatability_spread_mm"], 1.4)
        y_metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "y_metadata")])
        self.assertEqual(y_metadata["repeatability_spread_mm"], 0.5)
        self.assertEqual(rig.workflow.last_result["axis"], "y")
        self.assertEqual(rig.scripts, ["G28 X"] * 11 + ["G28 Y"] * 11)

    def test_early_hit_resets_window_before_new_candidate_and_fresh_discovery(self):
        for early in (8.0, 19.999):
            with self.subTest(early=early):
                rig = Rig()
                rig.home_results.extend([{}, {"travel_mm": 151.0}, {"travel_mm": early}]
                                        + [{"travel_mm": 148.0}] * 11)
                rig.calibrate(save=0)
                restart = rig.home_calls[3]
                self.assertEqual(restart["threshold"], 1.25)
                self.assertEqual(restart["successes"], 0)
                self.assertFalse(restart["candidate_qualified"])
                self.assertEqual(restart["homed_axes"], "z")
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
                self.assertEqual(metadata["min_commanded_travel_mm"], 148.0)
                self.assertEqual(metadata["max_commanded_travel_mm"], 148.0)
                self.assertEqual(metadata["repeatability_spread_mm"], 0.0)
                self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1.0] * 3 + [1.25] * 11)

    def test_repeatability_failure_after_early_retry_increments_and_restarts_everything(self):
        rig = Rig()
        rig.home_results.extend([{}, {"travel_mm": 150.0}, {"travel_mm": 8.0}])
        rig.home_results.extend([{}] + [{"travel_mm": 150.0}] * 9 + [{"travel_mm": 152.01}])
        rig.calibrate(save=0)
        self.assertEqual([c["threshold"] for c in rig.home_calls],
                         [1.0] * 3 + [1.25] * 11 + [1.5] * 11)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.5)
        self.assertEqual(rig.workflow.last_result["restarts"], 2)
        self.assertEqual(rig.workflow.last_result["successes"], 10)

    def test_status_reports_current_validation_spread_and_fixed_limit(self):
        rig = Rig()
        states = []
        rig.on_home = lambda axis: states.append(rig.workflow.get_status(0)["active"])
        rig.home_results.extend([{}] + [{"travel_mm": 150.0 + 0.1 * index} for index in range(10)])
        rig.calibrate(save=0)
        self.assertEqual(states[1]["successes"], 0)
        self.assertIsNone(states[1]["repeatability_spread_mm"])
        self.assertEqual(states[2]["successes"], 1)
        self.assertEqual(states[2]["repeatability_spread_mm"], 0.0)
        self.assertEqual(states[-1]["successes"], 9)
        self.assertEqual(states[-1]["repeatability_spread_mm"], 0.8)
        self.assertTrue(all(status["repeatability_limit_mm"] == 2.0 for status in states))

    def test_save_guard_rejects_malformed_or_unqualified_windows_before_any_write(self):
        mutations = {
            "missing session": lambda w: setattr(w, "session", None),
            "missing validation window": lambda w: w.session.pop("validation_records"),
            "too few successes": lambda w: w.session.update(successes=9),
            "too few homes": lambda w: w.session["validation_records"].pop(),
            "missing discovery": lambda w: w.session.update(candidate_attempt=None),
            "unqualified discovery": lambda w: w.session.update(candidate_qualified=False),
            "wrong axis": lambda w: w.session["validation_records"][-1].update(axis="y"),
            "wrong candidate": lambda w: w.session["validation_records"][-1].update(workflow_threshold_g=2.0),
            "wrong run": lambda w: w.session["validation_records"][-1].update(run_id="other-run"),
            "discovery counted": lambda w: w.session["validation_records"][-1].update(workflow_phase="discovery"),
            "repeated attempt": lambda w: w.session["validation_records"][-1].update(workflow_attempt=2),
            "unobserved home": lambda w: w.session["validation_records"].__setitem__(
                -1, dict(w.session["validation_records"][-1])),
            "bad geometry": lambda w: w.session["validation_records"][-1].update(travel_mm=1.0),
            "nonfinite distance": lambda w: w.session["validation_records"][-1].update(travel_mm=math.nan),
            "forged spread summary": lambda w: w.session["validation_records"][-1].update(travel_mm=152.001),
            "missing save mode": lambda w: w.session.pop("save_config"),
        }
        for name, mutate in mutations.items():
            with self.subTest(mutation=name):
                rig = Rig()
                with mock.patch.object(rig.workflow, "_save_model"):
                    rig.calibrate(save=0)
                mutate(rig.workflow)
                before = rig.effects()
                with self.assertRaisesRegex(CommandError, "cannot save calibration"):
                    rig.workflow._save_model()
                self.assertEqual(rig.effects(), before)
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)

    def test_save_guard_rejects_one_home_reused_as_multiple_validation_records(self):
        rig = Rig()
        with mock.patch.object(rig.workflow, "_save_model"):
            rig.calibrate(save=0)
        session = rig.workflow.session
        session["validation_records"][-1] = session["records"][-1] = session["validation_records"][0]
        with self.assertRaisesRegex(CommandError, "ten distinct"):
            rig.workflow._save_model()
        self.assertEqual(rig.configfile.saved, {})
        self.assertNotIn("SAVE_CONFIG", rig.scripts)

    def test_exact_workflow_threshold_supports_rounded_sensor_record_threshold(self):
        rig = Rig({"calibration_start_threshold": 0.755})
        rig.home_results.extend([{"threshold_g": 0.76}] * 11)
        rig.calibrate(save=0)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 0.755)
        self.assertEqual(rig.workflow.last_result["repeatability_spread_mm"], 0.0)


class GuardTests(unittest.TestCase):
    def test_runtime_policy_refusal_precedes_workflow_effects_with_stock_or_safe_z(self):
        for safe in (False, True):
            for command, requested, axes in (("cmd_CALIBRATE", "X", ("x",)),
                                             ("cmd_TEST", "X", ("x", "y")),
                                             ("cmd_CHARTS", None, ("x", "y")),
                                             ("cmd_CHARTS", "X", ("x",)),
                                             ("cmd_CHARTS", "Y", ("y",))):
                with self.subTest(safe=safe, command=command, requested=requested):
                    rig = Rig()
                    if safe:
                        rig.use_safe_z_home()
                    rig.policy_error = "synthetic unsafe runtime homing policy"
                    before = rig.effects()
                    params = {} if requested is None else {"AXIS": requested}
                    with self.assertRaisesRegex(CommandError, "unsafe runtime homing policy"):
                        getattr(rig.workflow, command)(GCmd(CENTERED=1, **params))
                    self.assertEqual(rig.policy_checks, [axes])
                    self.assertEqual(rig.effects(), before)

    def test_resume_rechecks_runtime_policy_before_another_home(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 8.0})
        rig.wait_calibration(save=0)
        rig.policy_error = "synthetic runtime policy changed"
        with self.assertRaisesRegex(CommandError, "runtime policy changed"):
            rig.calibrate()
        self.assertEqual(rig.policy_checks, [("x",), ("x",)])
        self.assertEqual(rig.scripts, ["G28 X"])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.configfile.saved, {})
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})

    def test_missing_zero_or_malformed_centered_has_no_start_or_resume_side_effects(self):
        for waiting in (False, True):
            for params in ({}, {"CENTERED": 0}, {"CENTERED": "invalid"},
                           {"CENTERED": "1.0"}, {"CENTERED": 2}, {"CENTERED": -1}):
                with self.subTest(waiting=waiting, params=params):
                    rig = Rig()
                    if waiting:
                        rig.home_results.append({"travel_mm": 2.0})
                        rig.wait_calibration()
                    before = rig.effects()
                    with self.assertRaisesRegex(CommandError, "CENTERED=1"):
                        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", **params))
                    self.assertEqual(rig.effects(), before)

    def test_homed_x_or_y_is_rejected_without_motion_release_or_setting_changes(self):
        for homed in ("x", "y", "xy", "xz", "yz", "xyz"):
            for axis in "XY":
                rig = Rig()
                rig.toolhead.kin.limits = [(0.0, 300.0) if a in homed else (1.0, -1.0)
                                           for a in "xyz"]
                before = rig.effects()
                with self.assertRaisesRegex(CommandError, "both X and Y unhomed"):
                    rig.calibrate(axis)
                self.assertEqual(rig.effects(), before)

    def test_release_clears_only_xy_across_integer_and_letter_clear_state_apis(self):
        class IntegerKin(Kin):
            def clear_homing_state(self, axes):
                for index in range(3):
                    if index in axes:
                        self.limits[index] = (1.0, -1.0)

        for kind in (Kin, IntegerKin):
            for z_homed in (False, True):
                rig = Rig()
                kin = kind()
                kin.limits[2] = (20.0, 480.0) if z_homed else (1.0, -1.0)
                kin.clear_homing_state = mock.Mock(wraps=kin.clear_homing_state)
                rig.toolhead.kin = kin
                z_limit = kin.limits[2]
                rig.home_results.append({"travel_mm": 2.0})
                rig.wait_calibration()
                self.assertEqual(kin.get_status(0)["homed_axes"], "z" if z_homed else "")
                self.assertEqual(kin.limits[2], z_limit)
                self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
                self.assertTrue(rig.lines["stepper_z"].enabled)
                self.assertTrue(rig.lines["stepper_z1"].enabled)
                kin.clear_homing_state.assert_not_called()

    def test_unknown_limits_layout_is_rejected_before_side_effects(self):
        for limits in (None, ((1.0, -1.0),) * 3, [(1.0, -1.0)] * 2):
            rig = Rig()
            rig.toolhead.kin.limits = limits
            before = rig.effects()
            with self.assertRaisesRegex(CommandError, "invalidate XY"):
                rig.calibrate()
            self.assertEqual(rig.effects(), before)

    def test_nonfinite_workflow_configuration_is_rejected(self):
        for name in ("calibration_start_threshold", "calibration_step",
                     "calibration_confirm_timeout", "calibration_travel_tolerance",
                     "workflow_reposition_speed"):
            with self.assertRaises(ConfigError):
                Rig({name: float("nan")})

    def test_charts_default_tracks_custom_main_configuration_directory(self):
        with mock.patch.object(Rig, "get_start_args", create=True,
                               return_value={"config_file": "C:/custom_instance/config/printer-main.cfg"}):
            rig = Rig()
            expected = Path("C:/custom_instance/config/accel_homing_charts").absolute()
            self.assertEqual(Path(rig.workflow.charts_directory), expected)

    def test_busy_print_pause_and_pending_queue_reject_before_motion(self):
        for setup in (lambda r: setattr(r, "print_state", "printing"),
                      lambda r: setattr(r, "print_state", "paused"),
                      lambda r: setattr(r, "paused", True),
                      lambda r: setattr(r, "sd_active", True),
                      lambda r: setattr(r, "idle_state", "Printing"),
                      lambda r: setattr(r.toolhead, "busy", (11.0, 10.0, True)),
                      lambda r: setattr(r.toolhead, "busy", (0.0, 10.0, False))):
            rig = Rig()
            setup(rig)
            with self.assertRaises(CommandError):
                rig.calibrate()
            self.assertEqual(rig.scripts, [])
            self.assertEqual(rig.disabled, [])

    def test_current_command_does_not_mark_idle_printing_and_own_retry_release_can_continue(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 2.0})
        rig.wait_calibration()
        self.assertEqual(rig.idle_state, "Printing")
        rig.calibrate()
        self.assertEqual(rig.workflow.last_result["result"], "calibrated")

    def test_unrecognized_overrides_macros_and_unrelated_g28_wrappers_rejected(self):
        for key in ("homing_override", "gcode_macro G28", "wrapped_handler"):
            rig = Rig()
            if key == "wrapped_handler":
                rig.gcode.ready_gcode_handlers["G28"] = lambda g: None
            else:
                rig.objects[key] = object()
            with self.assertRaisesRegex(CommandError, "G28"):
                rig.calibrate()
            self.assertEqual(rig.scripts, [])

    def test_retract_or_minimum_second_home_are_rejected_before_initial_g28(self):
        for field in ("retract_dist", "min_home_dist"):
            rig = Rig()
            setattr(rig.owner.rails_by_axis["x"].info, field, 5.0)
            with self.assertRaisesRegex(CommandError, "homing_retract_dist"):
                rig.calibrate()
            self.assertEqual(rig.scripts, [])

    def test_unsupported_z_coupled_kinematics_and_shared_enable_lines_rejected(self):
        for kind in ("corexz", "shared_line", "coupled_stepper", "dual_carriage"):
            rig = Rig()
            if kind == "corexz":
                rig.workflow.kinematics_name = "corexz"
            elif kind == "shared_line":
                rig.lines["stepper_x"].enable = rig.lines["stepper_z"].enable
            elif kind == "coupled_stepper":
                rig.toolhead.kin.steppers[0].axes = "xz"
            else:
                rig.toolhead.kin.dual_carriage_axis = 0
            with self.assertRaises(CommandError):
                rig.calibrate()
            self.assertEqual(rig.scripts, [])
            self.assertEqual(rig.disabled, [])

    def test_new_workflow_external_home_and_setting_change_are_blocked_while_waiting(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 2.0})
        rig.wait_calibration()
        before = rig.effects()
        with self.assertRaisesRegex(CommandError, "already active"):
            rig.calibrate("Y")
        with self.assertRaisesRegex(CommandError, "already active"):
            rig.workflow.cmd_TEST(GCmd(CENTERED=1))
        with self.assertRaisesRegex(CommandError, "already active"):
            rig.workflow.cmd_CHARTS(GCmd(CENTERED=1))
        with self.assertRaisesRegex(CommandError, "workflow"):
            rig.workflow.check_home_axes({"x": rig.owner.rails_by_axis["x"]})
        with self.assertRaisesRegex(CommandError, "workflow"):
            rig.workflow.check_settings_change()
        self.assertEqual(rig.effects(), before)

    def test_calibration_cannot_replace_another_workflow_or_reenter_a_running_one(self):
        for mode in ("test", "charts", "calibration"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temp:
                rig = Rig({"charts_directory": temp})
                checked = []
                def check_reentrant(axis):
                    rig.on_home = None
                    before = rig.effects()
                    for callback in (lambda: rig.calibrate(),
                                     lambda: rig.workflow.cmd_TEST(GCmd(CENTERED=1)),
                                     lambda: rig.workflow.cmd_CHARTS(GCmd(CENTERED=1))):
                        with self.assertRaisesRegex(CommandError, "already active"):
                            callback()
                        self.assertEqual(rig.effects(), before)
                    checked.append(axis)
                rig.on_home = check_reentrant
                if mode == "calibration":
                    rig.calibrate(save=0)
                else:
                    getattr(rig.workflow, "cmd_TEST" if mode == "test" else "cmd_CHARTS")(GCmd(CENTERED=1))
                self.assertEqual(checked, ["x"])
                self.assertIsNone(rig.workflow.session)

    def test_calibration_resume_keeps_save_config_choice_and_rejects_conflicting_override(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 2.0})
        rig.wait_calibration(save=0)
        before = rig.effects()
        with self.assertRaisesRegex(CommandError, "SAVE_CONFIG cannot change"):
            rig.calibrate(save=1)
        self.assertEqual(rig.effects(), before)
        rig.calibrate()
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertFalse(rig.workflow.last_result["save_config"])

    def test_stale_position_settings_homed_state_or_print_state_abort_calibration_resume(self):
        for setup in (lambda r: r.toolhead.position.__setitem__(0, 10.0),
                      lambda r: r.owner.impact_sign.__setitem__("x", -1),
                      lambda r: r.owner.threshold_g.__setitem__("x", 7.0),
                      lambda r: setattr(r.owner, "homing_accel", 4000.0),
                      lambda r: setattr(r.owner, "back_off_dist", 3.0),
                      lambda r: r.toolhead.kin.limits.__setitem__(0, (0.0, 300.0)),
                      lambda r: setattr(r, "print_state", "printing"),
                      lambda r: setattr(r, "paused", True),
                      lambda r: setattr(r.toolhead, "busy", (20.0, 10.0, True))):
            rig = Rig()
            rig.home_results.append({"travel_mm": 2.0})
            rig.wait_calibration()
            setup(rig)
            with self.assertRaises(CommandError):
                rig.calibrate()
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.scripts, ["G28 X"])
            self.assertEqual(rig.toolhead.moves, [])

    def test_late_resume_is_rejected_even_if_timeout_callback_has_not_run(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 2.0})
        rig.wait_calibration()
        rig.reactor.now = rig.workflow.session["expires"]
        with self.assertRaisesRegex(CommandError, "expired"):
            rig.calibrate()
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.scripts, ["G28 X"])
        self.assertEqual(rig.toolhead.moves, [])

    def test_cancel_and_timeout_restore_without_more_motion(self):
        for method in ("cancel", "timeout"):
            rig = Rig()
            rig.home_results.append({"travel_mm": 2.0})
            rig.wait_calibration()
            if method == "cancel":
                rig.workflow.cmd_CANCEL(GCmd())
            else:
                when = rig.workflow.session["expires"]
                self.assertEqual(rig.workflow._timeout(when), rig.reactor.NEVER)
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.owner.threshold_g["x"], 8.0)
            self.assertEqual(rig.scripts, ["G28 X"])
            self.assertEqual(rig.toolhead.moves, [])


class ModelTests(unittest.TestCase):
    def test_default_and_early_trigger_calibrations_reload_with_their_saved_thresholds(self):
        for axis in "xy":
            for early in (False, True):
                with self.subTest(axis=axis, early_trigger=early):
                    calibrated = Rig()
                    if early:
                        calibrated.home_results.append({"travel_mm": 2.0})
                    calibrated.calibrate(axis.upper())
                    threshold = 1.25 if early else 1.0
                    self.assertEqual(calibrated.home_calls[0]["threshold"], 1.0)
                    if early:
                        self.assertEqual(calibrated.home_calls[1]["threshold"], 1.25)
                    self.assertEqual(calibrated.scripts[-1], "SAVE_CONFIG")
                    options = {key: value for (section, key), value
                               in calibrated.configfile.saved.items()}
                    sections = {"printer": {"kinematics": "corexy"},
                                "accel_homing_model default": options}
                    reloaded = Rig(sections=sections)
                    reloaded.workflow._validate_model_connection()
                    self.assertEqual(reloaded.workflow.model.thresholds, {axis: threshold})
                    self.assertEqual(reloaded.workflow.model.successes[axis], 10)
                    self.assertEqual(reloaded.owner.threshold_g[axis], threshold)
                    self.assertEqual(reloaded.scripts, [])

    def test_selected_model_rejects_changed_sensor_geometry_speed_or_mapping(self):
        calibrated = Rig()
        calibrated.calibrate(save=0)
        options = {key: value for (section, key), value in calibrated.configfile.saved.items()}
        sections = {"printer": {"kinematics": "corexy"},
                    "accel_homing_model default": options}
        rig = Rig(sections=sections)
        rig.workflow._validate_model_connection()
        for mutate in (lambda r: setattr(r.owner, "sensor_name", "other_sensor"),
                       lambda r: setattr(r.owner.rails_by_axis["x"].info, "position_endstop", 2.0),
                       lambda r: setattr(r.owner.rails_by_axis["x"], "bounds", (-10.0, 300.0)),
                       lambda r: setattr(r.owner.rails_by_axis["x"].info, "speed", 80.0),
                       lambda r: setattr(r.owner, "helper", types.SimpleNamespace(
                           config=types.SimpleNamespace(axes_map=[(1, 1), (0, 1), (2, 1)])))):
            rig = Rig(sections=sections)
            mutate(rig)
            with self.assertRaisesRegex(ConfigError, "does not match"):
                rig.workflow._validate_model_connection()
            override = Rig({"x_threshold": 8.0}, sections=sections)
            mutate(override)
            override.workflow._validate_model_connection()

    def test_saved_models_supply_defaults_but_manual_controls_win(self):
        sections = {"printer": {"kinematics": "corexy"},
                    "accel_homing_model default": {"x_threshold": 4.0, "x_successes": 10,
                                                   "y_threshold": 5.0, "y_successes": 10}}
        rig = Rig(sections=sections)
        self.assertEqual(rig.owner.threshold_g, {"x": 4.0, "y": 5.0})
        rig = Rig({"x_threshold": 8.0}, sections=sections)
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 5.0})
        rig = Rig({"threshold": 8.0}, sections=sections)
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})

    def test_saved_models_keep_accepting_previous_thresholds(self):
        for axis in "xy":
            for threshold in (1.0, 1.5, 2.0, 2.5, 4.0, 15.0, 15.1, 20.0, 1e308):
                with self.subTest(axis=axis, threshold=threshold):
                    model = AccelHomingModel(Config(None, {
                        axis + "_threshold": threshold, axis + "_successes": 10},
                        "accel_homing_model test"))
                    self.assertEqual(model.thresholds, {axis: threshold})

    def test_model_rejects_unqualified_threshold_or_bad_metadata(self):
        for axis in "xy":
            invalid = [{axis + "_threshold": 4.0, axis + "_successes": 9},
                       {axis + "_metadata": "[]"}, {axis + "_metadata": "invalid"},
                       {axis + "_metadata": '{"invalid": NaN}'},
                       {axis + "_metadata": '{"nested": [1e999]}'}]
            invalid.extend({axis + "_threshold": threshold, axis + "_successes": 10}
                           for threshold in (0.499, 0.0, -1.0,
                                             float("nan"), float("inf"), float("-inf")))
            for options in invalid:
                with self.subTest(axis=axis, options=options), self.assertRaises(ConfigError):
                    AccelHomingModel(Config(None, options, "accel_homing_model test"))

    def test_saving_one_axis_keeps_other_axis_model(self):
        rig = Rig()
        rig.configfile.saved[("accel_homing_model default", "y_threshold")] = "4.5"
        rig.calibrate()
        self.assertEqual(rig.configfile.saved[("accel_homing_model default", "y_threshold")], "4.5")


class TestAndChartWorkflowTests(unittest.TestCase):
    def test_chart_axis_is_validated_before_preflight_or_waiting_session_mutation(self):
        for waiting in (False, True):
            for requested in ("", " ", "Z", "XY", "YX", "X,Y", "X Y", "XX", "1"):
                with self.subTest(waiting=waiting, requested=requested):
                    rig = Rig()
                    if waiting:
                        rig.home_results.append({"travel_mm": 2.0})
                        rig.wait_calibration(save=0)
                    before = rig.effects()
                    with mock.patch.object(rig.workflow, "_require_centered") as centered, \
                            mock.patch.object(rig.workflow, "_chart_preflight") as preflight, \
                            mock.patch.object(rig.workflow, "_begin") as begin:
                        with self.assertRaisesRegex(CommandError, "AXIS must be X or Y"):
                            rig.workflow.cmd_CHARTS(GCmd(AXIS=requested))
                        centered.assert_not_called()
                        preflight.assert_not_called()
                        begin.assert_not_called()
                    self.assertEqual(rig.effects(), before)

    def test_single_axis_charts_home_capture_and_release_only_the_requested_axis(self):
        for requested, axis in (("X", "x"), ("Y", "y"), (" x ", "x"), ("y", "y")):
            for safe in (False, True):
                with self.subTest(requested=requested, safe=safe), tempfile.TemporaryDirectory() as temp:
                    rig = Rig({"charts_directory": temp})
                    rig.toolhead.position[:2] = [12.0, 19.0]
                    other = "y" if axis == "x" else "x"
                    previous = {"axis": other, "result": "preexisting"}
                    rig.owner.last[other] = previous
                    wrapper = rig.use_safe_z_home() if safe else None
                    with mock.patch.object(rig.workflow, "_release_terminal_home",
                                           wraps=rig.workflow._release_terminal_home) as release:
                        rig.workflow.cmd_CHARTS(GCmd(AXIS=requested, CENTERED=1))
                        result = rig.workflow.last_result
                        release.assert_called_once_with(axis, result["records"][0])
                    self.assertEqual(rig.scripts, ["G28 " + axis.upper()])
                    self.assertEqual([call["axis"] for call in rig.home_calls], [axis])
                    self.assertEqual(rig.policy_checks, [(axis,)])
                    target = [None, None, None]
                    target["xy".index(axis)] = 5.0
                    self.assertEqual(rig.toolhead.moves, [(target, 100.0)])
                    self.assertEqual(rig.toolhead.position["xy".index(other)],
                                     19.0 if other == "y" else 12.0)
                    self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
                    self.assertFalse(rig.owner._homed(other))
                    self.assertIs(rig.owner.last[other], previous)
                    self.assertEqual(result["result"], "complete")
                    self.assertEqual([item["axis"] for item in result["paths"]], [axis])
                    payloads = [item for item in rig.workflow.renderer.calls if item["kind"] == "capture"]
                    self.assertEqual(len(payloads), 1)
                    self.assertEqual(payloads[0]["axis"], axis)
                    self.assertEqual(payloads[0]["record"]["axis"], axis)
                    self.assertEqual(payloads[0]["record"]["run_id"], result["run_id"])
                    self.assertEqual(payloads[0]["record"]["workflow_attempt"], 1)
                    self.assertTrue(Path(result["paths"][0]["png"]).is_file())
                    self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                    self.assertEqual(rig.owner.back_off_dist, 5.0)
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertEqual(rig.disabled, [])
                    self.assertIsNone(rig.workflow.session)
                    if wrapper is not None:
                        self.assertEqual(wrapper.calls, [])

    def test_single_axis_charts_do_not_require_an_accelerometer_endstop_on_the_other_axis(self):
        for axis in "xy":
            with self.subTest(axis=axis), tempfile.TemporaryDirectory() as temp:
                rig = Rig({"charts_directory": temp})
                other = "y" if axis == "x" else "x"
                del rig.owner.endstops[other]
                del rig.owner.rails_by_axis[other]
                rig.workflow.cmd_CHARTS(GCmd(AXIS=axis.upper(), CENTERED=1))
                self.assertEqual(rig.scripts, ["G28 " + axis.upper()])
                self.assertEqual(rig.workflow.last_result["result"], "complete")

    def test_single_axis_charts_still_require_manual_xy_preparation_and_both_axes_unhomed(self):
        for axis in "xy":
            for centered in (None, 0, "invalid"):
                with self.subTest(axis=axis, centered=centered):
                    rig = Rig()
                    before = rig.effects()
                    params = {} if centered is None else {"CENTERED": centered}
                    with self.assertRaisesRegex(CommandError, "CENTERED=1"):
                        rig.workflow.cmd_CHARTS(GCmd(AXIS=axis.upper(), **params))
                    self.assertEqual(rig.effects(), before)
            for homed in ("x", "y", "xy"):
                with self.subTest(axis=axis, homed=homed):
                    rig = Rig()
                    rig.toolhead.kin.limits = [(0.0, 300.0) if item in homed else (1.0, -1.0)
                                               for item in "xyz"]
                    before = rig.effects()
                    with self.assertRaisesRegex(CommandError, "both X and Y unhomed"):
                        rig.workflow.cmd_CHARTS(GCmd(AXIS=axis.upper(), CENTERED=1))
                    self.assertEqual(rig.effects(), before)

    def test_single_axis_chart_rejection_or_export_failure_never_releases_or_homes_another_axis(self):
        for axis in "xy":
            for failure in ("suspect", "export"):
                with self.subTest(axis=axis, failure=failure), tempfile.TemporaryDirectory() as temp:
                    rig = Rig({"charts_directory": temp})
                    if failure == "suspect":
                        rig.home_results.append({"travel_mm": 15.0})
                    context = mock.patch.object(workflow_module, "write_capture",
                                                side_effect=OSError("synthetic export error")) \
                        if failure == "export" else contextlib.nullcontext()
                    with context, mock.patch.object(rig.workflow, "_release_terminal_home") as release:
                        error = OSError if failure == "export" else CommandError
                        message = "synthetic export error" if failure == "export" else "no automatic move"
                        with self.assertRaisesRegex(error, message):
                            rig.workflow.cmd_CHARTS(GCmd(AXIS=axis.upper(), CENTERED=1))
                        release.assert_not_called()
                    self.assertEqual(rig.scripts, ["G28 " + axis.upper()])
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.workflow.last_result["result"],
                                     "aborted" if failure == "export" else "rejected")
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                    self.assertEqual(rig.owner.back_off_dist, 5.0)
                    self.assertFalse(rig.capture_active)
                    self.assertIsNone(rig.workflow.session)

    def test_missing_zero_or_malformed_centered_cannot_start_or_mutate_a_waiting_session(self):
        for callback in ("cmd_TEST", "cmd_CHARTS"):
            for waiting in (False, True):
                for params in ({}, {"CENTERED": 0}, {"CENTERED": "invalid"},
                               {"CENTERED": "1.0"}, {"CENTERED": 2}, {"CENTERED": -1}):
                    with self.subTest(callback=callback, waiting=waiting, params=params):
                        rig = Rig()
                        if waiting:
                            rig.home_results.append({"travel_mm": 2.0})
                            rig.wait_calibration(save=0)
                        before = rig.effects()
                        with self.assertRaisesRegex(CommandError, "CENTERED=1"):
                            getattr(rig.workflow, callback)(GCmd(**params))
                        self.assertEqual(rig.effects(), before)

    def test_separate_confirmation_command_and_method_are_removed(self):
        rig = Rig()
        self.assertNotIn("ACCEL_HOMING_CONFIRM", rig.gcode.ready_gcode_handlers)
        self.assertFalse(hasattr(rig.workflow, "cmd_CONFIRM"))
        for command in ("ACCEL_HOMING_CALIBRATE", "ACCEL_HOMING_TEST",
                        "ACCEL_HOMING_CHARTS_GENERATE", "ACCEL_HOMING_CANCEL", "ACCEL_HOMING_STATUS"):
            self.assertIn(command, rig.gcode.ready_gcode_handlers)

    def test_old_extra_token_is_ignored_for_compatibility(self):
        rig = Rig()
        rig.workflow.cmd_TEST(GCmd(CENTERED=1, TOKEN="obsolete-token"))
        self.assertEqual(rig.scripts, ["G28 X", "G28 Y"])
        self.assertEqual(rig.workflow.last_result["result"], "complete")

    def test_prehomed_xy_are_rejected_before_release_motion_or_setting_changes(self):
        for callback in ("cmd_TEST", "cmd_CHARTS"):
            for homed in ("x", "y", "xy", "xz", "yz", "xyz"):
                with self.subTest(callback=callback, homed=homed):
                    rig = Rig()
                    rig.toolhead.kin.limits = [(0.0, 300.0) if axis in homed else (1.0, -1.0)
                                               for axis in "xyz"]
                    before = rig.effects()
                    with self.assertRaisesRegex(CommandError, "both X and Y unhomed"):
                        getattr(rig.workflow, callback)(GCmd(CENTERED=1))
                    self.assertEqual(rig.effects(), before)

    def test_busy_printing_paused_shutdown_and_other_homing_block_direct_commands(self):
        for callback in ("cmd_TEST", "cmd_CHARTS"):
            for setup in (lambda r: setattr(r, "print_state", "printing"),
                          lambda r: setattr(r, "print_state", "paused"),
                          lambda r: setattr(r, "paused", True),
                          lambda r: setattr(r, "sd_active", True),
                          lambda r: setattr(r, "idle_state", "Printing"),
                          lambda r: setattr(r, "shutdown", True),
                          lambda r: setattr(r.owner, "homing_active", True),
                          lambda r: setattr(r.owner, "attempt", object()),
                          lambda r: setattr(r.toolhead, "busy", (0.0, 10.0, False))):
                rig = Rig()
                setup(rig)
                before = rig.effects()
                with self.assertRaises(CommandError):
                    getattr(rig.workflow, callback)(GCmd(CENTERED=1))
                self.assertEqual(rig.effects(), before)

    def test_direct_commands_home_xy_without_preliminary_home_release_or_wait(self):
        for callback in ("cmd_TEST", "cmd_CHARTS"):
            with self.subTest(callback=callback), tempfile.TemporaryDirectory() as temp:
                rig = Rig({"charts_directory": temp})
                rig.toolhead.position[:2] = [12.0, 19.0]
                getattr(rig.workflow, callback)(GCmd(CENTERED=1))
                self.assertEqual(rig.scripts, ["G28 X", "G28 Y"])
                self.assertEqual([call["axis"] for call in rig.home_calls], ["x", "y"])
                self.assertEqual(rig.home_calls[0]["position"], [12.0, 19.0, 40.0, 6.0])
                self.assertEqual(rig.home_calls[0]["homed_axes"], "z")
                self.assertEqual(rig.toolhead.moves, [([150.0, None, None], 50.0),
                                                     ([None, 5.0, None], 100.0)])
                self.assertEqual(rig.disabled, [])
                self.assertEqual(rig.toolhead.dwells, [])
                self.assertEqual(rig.workflow.last_result["result"], "complete")
                self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                self.assertEqual(rig.owner.back_off_dist, 5.0)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.dispatched_scripts, [])
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.reactor.timers[rig.workflow.timer], rig.reactor.NEVER)

    def test_chart_exports_current_run_samples_and_explicit_paths(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = Rig({"charts_directory": temp})
            rig.owner.last["x"] = {"axis": "x", "result": "impact", "travel_mm": 1234}
            rig.workflow.cmd_CHARTS(GCmd(CENTERED=1))
            self.assertEqual(rig.scripts, ["G28 X", "G28 Y"])
            result = rig.workflow.last_result
            run = result["run_id"]
            self.assertEqual(len(result["paths"]), 2)
            for item in result["paths"]:
                path = Path(item["png"])
                self.assertTrue(path.is_absolute())
                self.assertTrue(path.is_file())
                self.assertNotIn(run, path.name)
            payloads = [item for item in rig.workflow.renderer.calls if item["kind"] == "capture"]
            self.assertTrue(all(item["record"]["run_id"] == run for item in payloads))
            self.assertNotEqual(payloads[0]["record"]["travel_mm"], 1234)
            self.assertTrue(all(path.suffix == ".png" for path in Path(temp).rglob("*") if path.is_file()))

    def test_suspect_xy_results_finalize_rejection_without_more_homes_or_model_saving(self):
        for callback in ("cmd_TEST", "cmd_CHARTS"):
            for bad_axis in "xy":
                for bad in ({"travel_mm": 15.0}, {"result": "pressed"},
                            {"result": "no_data"}, {"travel_mm": 500.0}, {"moving": False}):
                    with self.subTest(callback=callback, axis=bad_axis, bad=bad), tempfile.TemporaryDirectory() as temp:
                        rig = Rig({"charts_directory": temp})
                        if bad_axis == "y":
                            rig.home_results.append({})
                        rig.home_results.append(bad)
                        with self.assertRaisesRegex(CommandError, "no automatic move"):
                            getattr(rig.workflow, callback)(GCmd(CENTERED=1))
                        expected = ["G28 X"] if bad_axis == "x" else ["G28 X", "G28 Y"]
                        self.assertEqual(rig.scripts, expected)
                        self.assertEqual(rig.toolhead.moves, [] if bad_axis == "x" else
                                         [([150.0, None, None], 50.0)])
                        self.assertEqual(rig.workflow.last_result["result"], "rejected")
                        self.assertEqual(len(rig.workflow.last_result["paths"]),
                                         len(expected) if callback == "cmd_CHARTS" else 0)
                        self.assertIsNone(rig.workflow.session)
                        self.assertEqual(rig.configfile.saved, {})
                        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                        self.assertEqual(rig.owner.back_off_dist, 5.0)
                        self.assertFalse(rig.capture_active)

    def test_execution_failures_abort_restore_and_never_save_a_model(self):
        for callback in ("cmd_TEST", "cmd_CHARTS"):
            for failure in ("homing", "capture", "unexpected", "export"):
                if failure == "export" and callback != "cmd_CHARTS":
                    continue
                with self.subTest(callback=callback, failure=failure), tempfile.TemporaryDirectory() as temp:
                    rig = Rig({"charts_directory": temp})
                    if failure == "homing":
                        rig.home_results.append(CommandError("synthetic homing error"))
                    elif failure == "capture":
                        rig.bad_capture = {"samples": [], "dropped": 0}
                    elif failure == "unexpected":
                        rig.home_results.append(RuntimeError("synthetic unexpected error"))
                    context = mock.patch.object(workflow_module, "write_capture", side_effect=OSError("synthetic export error")) \
                        if failure == "export" else contextlib.nullcontext()
                    with context, self.assertRaises(Exception):
                        getattr(rig.workflow, callback)(GCmd(CENTERED=1))
                    self.assertEqual(rig.scripts, ["G28 X"])
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.workflow.last_result["result"], "aborted")
                    self.assertIsNone(rig.workflow.session)
                    self.assertFalse(rig.capture_active)
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                    self.assertEqual(rig.owner.back_off_dist, 5.0)

    def test_retry_cancel_timeout_shutdown_and_status_use_only_console_messages(self):
        for ending in ("cancel", "timeout", "shutdown"):
            with self.subTest(ending=ending):
                rig = Rig()
                rig.home_results.append({"travel_mm": 2.0})
                rig.wait_calibration(save=0)
                session = rig.workflow.session
                self.assertEqual(session["state"], "waiting")
                self.assertTrue(any("ACCEL_HOMING_CALIBRATE AXIS=X CENTERED=1" in line for line in rig.output))
                self.assertEqual(rig.workflow._timeout(rig.reactor.now), session["expires"])
                status = GCmd()
                rig.workflow.cmd_STATUS(status)
                self.assertEqual(json.loads(status.responses[-1])["active"]["state"], "waiting")
                if ending == "cancel":
                    rig.workflow.cmd_CANCEL(GCmd())
                elif ending == "timeout":
                    rig.workflow._timeout(session["expires"])
                else:
                    rig.shutdown = True
                    for callback in rig.handlers["klippy:shutdown"]:
                        callback()
                self.assertEqual(rig.scripts, ["G28 X"])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
                self.assertEqual(rig.owner.back_off_dist, 5.0)
                self.assertEqual(rig.workflow._timeout(rig.reactor.now), rig.reactor.NEVER)
                rig.workflow.cmd_CANCEL(GCmd())
                rig.workflow.cmd_STATUS(GCmd())


class MoonrakerChartDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.proc = self.base / "proc"
        self.proc.mkdir()
        self.uid = self.base.stat().st_uid
        self.data = self.base / "selected_data"
        self.socket = (self.base / "selected_klippy.sock").resolve()

    def configure(self, data, text=None):
        root = data / "config"
        root.mkdir(parents=True, exist_ok=True)
        config = root / "moonraker.conf"
        config.write_bytes((text if text is not None else
                            "[server]\nklippy_uds_address: %s\n" % self.socket).encode())
        return root

    def process(self, argv=None, environment=None, pid="123"):
        process = self.proc / pid
        (process / "cwd").mkdir(parents=True, exist_ok=True)
        argv = argv if argv is not None else ["python", "-m", "moonraker"]
        (process / "cmdline").write_bytes(b"\0".join(value.encode() for value in argv) + b"\0")
        (process / "environ").write_bytes(b"\0".join(
            (key + "=" + value).encode() for key, value in (environment or {}).items()) + b"\0")
        return process

    def discover(self, socket=None, uid=None):
        start = {"apiserver": str(self.socket if socket is None else socket),
                 "config_file": str(self.base / "unrelated/nested/printer.cfg")}
        with mock.patch.object(workflow_module.os, "getuid",
                               return_value=self.uid if uid is None else uid, create=True):
            return workflow_module.verified_moonraker_config_root(start, self.proc)

    def test_env_only_module_launch_maps_the_socket_matched_data_config_root(self):
        root = self.configure(self.data)
        alternate = self.base / "outside_config/moonraker.conf"
        alternate.parent.mkdir()
        alternate.write_bytes((root / "moonraker.conf").read_bytes())
        self.process(environment={"HOME": str(self.base / "process_home"),
                                  "MOONRAKER_DATA_PATH": str(self.data),
                                  "MOONRAKER_CONFIG_PATH": str(alternate)})
        self.assertEqual(self.discover(), str(root.resolve()))
        self.assertIsNone(self.discover(self.base / "wrong_klippy.sock"))
        self.assertNotEqual(root.resolve(), alternate.parent.resolve())

    def test_path_defaults_use_process_home_and_argv_overrides_environment_in_process_cwd(self):
        home = self.base / "moonraker_home"
        root = self.configure(home / "printer_data")
        process = self.process(environment={"HOME": str(home),
                                            "MOONRAKER_CONFIG_PATH": "~/printer_data/config/moonraker.conf"})
        self.assertEqual(self.discover(), str(root.resolve()))
        data = process / "cwd/selected"
        selected = self.configure(data)
        alternate = selected / "alternate.conf"
        alternate.write_bytes((selected / "moonraker.conf").read_bytes())
        self.process(argv=["python", "moonraker.py", "--datapath=unused", "-d", "selected",
                           "--configfile=selected/config/alternate.conf"],
                     environment={"HOME": str(home), "MOONRAKER_DATA_PATH": str(root),
                                  "MOONRAKER_CONFIG_PATH": str(self.base / "missing.conf")})
        self.assertEqual(self.discover(), str(selected.resolve()))
        unsupported = ["-dunused", "-cunused"]
        unsupported.extend(option[:length] + "=unused"
                           for option in ("--datapath", "--configfile")
                           for length in range(3, len(option)))
        for argument in unsupported:
            with self.subTest(argument=argument):
                self.process(argv=["python", "-m", "moonraker", argument],
                             environment={"HOME": str(home), "MOONRAKER_DATA_PATH": str(data)})
                self.assertIsNone(self.discover())

    def test_missing_home_uses_the_same_uid_passwd_home_without_host_home_substitution(self):
        home = self.base / "passwd_home"
        root = self.configure(home / "printer_data")
        self.process()
        lookup = mock.Mock(return_value=types.SimpleNamespace(pw_dir=str(home)))
        with mock.patch.dict(sys.modules, {"pwd": types.SimpleNamespace(getpwuid=lookup)}):
            self.assertEqual(self.discover(), str(root.resolve()))
        lookup.assert_called_once_with(self.uid)

    def test_omitted_socket_uses_official_default_and_unresolved_config_stays_unmapped(self):
        root = self.configure(self.data, "[server]\n")
        self.process(argv=["python", "-m", "moonraker", "-d", str(self.data)])
        self.assertEqual(self.discover(Path("/tmp/klippy_uds").resolve()), str(root.resolve()))
        self.assertIsNone(self.discover(self.data / "comms/klippy.sock"))
        for text in ("[server]\nklippy_uds_address:\n",
                     "[server]\nklippy_uds_address: selected_klippy.sock\n",
                     "[server]\nklippy_uds_address: {data_path}/comms/klippy.sock\n",
                     "[other]\nklippy_uds_address: %s\n" % self.socket,
                     "[server]\nklippy_uds_address: %s\n[include extra.conf]\n" % self.socket,
                     "[server]\nklippy_uds_address: %s\n[include\textra.conf]\n" % self.socket):
            with self.subTest(text=text):
                self.configure(self.data, text)
                self.assertIsNone(self.discover())

    def test_environment_reads_follow_uid_and_entrypoint_checks_and_remain_bounded_allowlisted(self):
        root = self.configure(self.data)
        process = self.process(argv=["python", "-m", "other"],
                               environment={"MOONRAKER_DATA_PATH": str(self.data)})
        with mock.patch.object(workflow_module, "_moonraker_path_environment",
                               side_effect=AssertionError("environment read before identity")) as read:
            self.assertIsNone(self.discover())
            read.assert_not_called()
        self.process(environment={"MOONRAKER_DATA_PATH": str(self.data)})
        with mock.patch.object(workflow_module, "_moonraker_path_environment",
                               side_effect=AssertionError("foreign UID environment read")) as read:
            self.assertIsNone(self.discover(uid=self.uid + 1))
            read.assert_not_called()
        valid = b"MOONRAKER_DATA_PATH=" + str(self.data).encode() + b"\0"
        (process / "environ").write_bytes(valid + b"SECRET_TOKEN=\xff\0")
        self.assertEqual(self.discover(), str(root.resolve()))
        for invalid in (valid + b"HOME=/first\0HOME=/second\0", b"x" * 65537):
            with self.subTest(environment_case=len(invalid)):
                (process / "environ").write_bytes(invalid)
                self.assertIsNone(self.discover())
        (process / "environ").unlink()
        self.assertIsNone(self.discover())

    def test_multiple_matching_pids_are_ambiguous_even_when_their_roots_are_identical(self):
        root = self.configure(self.data)
        self.process(environment={"MOONRAKER_DATA_PATH": str(self.data)})
        self.process(environment={"MOONRAKER_DATA_PATH": str(self.data)}, pid="456")
        self.assertIsNone(self.discover())
        other = self.base / "other_data"
        self.configure(other)
        self.process(environment={"MOONRAKER_DATA_PATH": str(other)}, pid="456")
        self.assertIsNone(self.discover())
        self.configure(other, "[server]\nklippy_uds_address: %s\n" % (self.base / "other.sock"))
        self.assertEqual(self.discover(), str(root.resolve()))


class PortableChartTests(unittest.TestCase):
    def test_failed_chart_write_publishes_nothing_and_previous_runs_are_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = {"samples": [(0.0, 0.0, 0.0, 1.0), (0.001, 0.0, 0.0, 1.0)],
                       "baseline_g": (0.0, 0.0, 1.0), "dropped": 0}
            with self.assertRaises(TypeError):
                write_capture(temp, "failed", "x", capture, {"unserializable": object()}, renderer=FakePNGRenderer())
            self.assertEqual(list(Path(temp).iterdir()), [])
            original = write_capture(temp, "completed", "x", capture, {}, renderer=FakePNGRenderer())
            before = Path(original["png"]).read_bytes()
            repeated = write_capture(temp, "completed", "x", capture, {}, renderer=FakePNGRenderer())
            self.assertNotEqual(repeated["png"], original["png"])
            self.assertEqual(Path(original["png"]).read_bytes(), before)

    def test_png_payload_has_baseline_subtracted_signed_samples_and_precise_threshold(self):
        with tempfile.TemporaryDirectory() as temp:
            record = {"onset_g": 4.0, "impact_peak_g": 6.0, "threshold_g": 3.0,
                      "direction": 0, "onset_time_s": 1.002}
            capture = {"samples": [(1.0, 1.0, 2.0, 3.0), (1.001, 1.0, 2.0, 3.0),
                                   (1.002, -5.0, 2.0, 3.0)],
                       "baseline_g": (1.0, 2.0, 3.0), "dropped": 0}
            renderer = FakePNGRenderer()
            paths = write_capture(temp, "synthetic_test", "x", capture, record,
                                  simulated=True, renderer=renderer)
            self.assertEqual(set(paths), {"png"})
            self.assertTrue(Path(paths["png"]).read_bytes().startswith(b"\x89PNG"))
            payload = renderer.calls[-1]
            self.assertEqual(payload["trace"][-1][1], -6.0)
            self.assertEqual(payload["record"]["threshold_g"], 3.0)
            self.assertTrue(payload["simulated"])
            self.assertIn("timestamp", payload)

    def test_chart_rejects_empty_stale_overflow_or_invalid_data_and_path_injection(self):
        for capture in ({"samples": []}, {"samples": [(1.0, 0, 0, 0)], "dropped": 1},
                        {"samples": [(1.0, 0, 0, 0), (1.0, 0, 0, 0)]},
                        {"samples": [(1.0, float("nan"), 0, 0)]}):
            with tempfile.TemporaryDirectory() as temp:
                with self.assertRaises(ValueError):
                    write_capture(temp, "run", "x", capture, {}, renderer=FakePNGRenderer())
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                write_capture(temp, "../outside", "x", {}, {})
            with self.assertRaises(ValueError):
                write_capture(temp, "run", "../../outside", {}, {})


if __name__ == "__main__":
    unittest.main()
