"""Shared in-memory fixtures for the axis limits calibration tests.

The native move and configfile are synthetic. They check the workflow's
accounting and state boundaries, not actual impacts or printer behavior.
"""
import collections
import configparser
import copy
import io
import json
import types

from harness import CommandError
from test_workflows import GCmd, Rail, Rig


class NativeConfigfile:
    def __init__(self, rig):
        self.rig = rig
        self.autosave = types.SimpleNamespace(fileconfig=configparser.RawConfigParser())
        self.status_save_pending, self.save_config_pending = {}, False
        self.source = "[stepper_x]\nposition_max: 300\nposition_endstop: 0\n" \
                      "[stepper_y]\nposition_max: 200\nposition_endstop: 0\n"
        self.included = {}
        self.set_calls, self.native_calls, self.saved_snapshots = [], 0, []
        self.fail_set_at = None
        self.save_error = None
        self.on_save = None

    @property
    def saved(self):
        return {(section, name): value for section in self.autosave.fileconfig.sections()
                for name, value in self.autosave.fileconfig.items(section, raw=True)}

    def get_status(self, eventtime):
        return {"settings": {}, "save_config_pending": self.save_config_pending,
                "save_config_pending_items": self.status_save_pending}

    def _read_config_file(self, filename):
        return self.source

    def _find_autosave_data(self, data):
        return data, ""

    def _strip_duplicates(self, data, autosave):
        parser = configparser.RawConfigParser()
        parser.read_string(data)
        for section in autosave.fileconfig.sections():
            if parser.has_section(section):
                for option in autosave.fileconfig.options(section):
                    parser.remove_option(section, option)
        output = io.StringIO()
        parser.write(output)
        return output.getvalue()

    def _build_config_wrapper(self, data, filename):
        parser = configparser.RawConfigParser()
        parser.read_string(data)
        for section, options in self.included.items():
            if not parser.has_section(section):
                parser.add_section(section)
            for option, value in options.items():
                parser.set(section, option, value)
        return types.SimpleNamespace(fileconfig=parser)

    def set(self, section, option, value):
        self.set_calls.append((section, option, value))
        if self.fail_set_at == len(self.set_calls):
            raise CommandError("synthetic staging failure")
        if not self.autosave.fileconfig.has_section(section):
            self.autosave.fileconfig.add_section(section)
        self.autosave.fileconfig.set(section, option, value)
        pending = copy.deepcopy(self.status_save_pending)
        pending.setdefault(section, {})[option] = value
        self.status_save_pending, self.save_config_pending = pending, True

    def cmd_SAVE_CONFIG(self, gcmd):
        self.native_calls += 1
        if self.on_save:
            self.on_save()
        if self.save_error:
            raise self.save_error
        self.saved_snapshots.append(copy.deepcopy(self.saved))


class NativeState:
    def __init__(self, printer):
        self.rig = printer

    def set_axes(self, axes):
        self.axes = list(axes)

    def _set_homing_accel(self, accel, pre_homing):
        self.rig.native_accel_calls.append((accel, pre_homing))
        # Match Kalico's configured-default reset, which is not necessarily
        # the live pre-command limit that the wizard must restore.
        self.rig.toolhead.accel = accel if pre_homing else 50000.0


class NativeMove:
    def __init__(self, printer, endstops):
        self.rig, self.endstops = printer, endstops
        self.stepper_positions = []

    def homing_move(self, target, speed, probe_pos=False):
        rig, owner, session = self.rig, self.rig.owner, self.rig.workflow.session
        axis = session["axis"]
        index = "xy".index(axis)
        direction = 1 if target[index] > 0 else -1
        phase = session["phase"]
        rig.measure_calls.append({"axis": axis, "phase": phase, "direction": direction,
                                  "target": list(target), "speed": speed, "probe_pos": probe_pos,
                                  "back_off_dist": dict(owner.back_off_dist),
                                  "accel": rig.toolhead.accel})
        if rig.on_measure:
            rig.on_measure(axis, phase)
        rig.workflow.axis_limits._check(session)
        item = rig.measure_results.popleft() if rig.measure_results else {}
        if isinstance(item, Exception):
            raise item
        distance = item.get("distance", rig.spans[axis] * (0.5 if phase == "initial_positioning" else 1.0))
        if distance > abs(target[index]):
            raise CommandError("No trigger after full finite movement")
        trigger = direction * distance
        halt = trigger + direction * item.get("overshoot", .25)
        for stepper in rig.toolhead.kin.get_steppers():
            if not stepper.is_active_axis(axis):
                continue
            factor = -1 if axis == "y" and stepper.name == "stepper_y" and stepper.axes == "xy" else 1
            ticks = lambda distance: round(distance * factor / stepper.get_step_dist())
            start = stepper.mcu_position
            stepper.mcu_position += ticks(halt)
            self.stepper_positions.append(types.SimpleNamespace(
                stepper=stepper, stepper_name=stepper.name, start_pos=start,
                trig_pos=start + ticks(trigger), halt_pos=stepper.mcu_position))
        if item.get("drop_stepper"):
            self.stepper_positions.pop()
        if item.get("duplicate_stepper"):
            self.stepper_positions[-1] = self.stepper_positions[0]
        metric = ("mapped_axis" if rig.workflow.detection_axis_mapping(axis) is not None
                  else "xyz_magnitude" if owner._uses_vector_detector(axis) else "selected_axis")
        owner.last[axis] = {"axis": axis, "pass": 1, "result": item.get("result", "impact"),
                            "impact_metric": metric,
                            "polarity_policy": "absolute_after_reference_subtraction_v1",
                            "trigger_travel_mm_signed": trigger, "halt_travel_mm_signed": halt,
                            "onset_g": 8., "impact_peak_g": 9.}
        rig.toolhead.position[index] += halt
        return list(rig.toolhead.position)


class AxisLimitsFixture(Rig):
    def __init__(self):
        super().__init__()
        self.spans = {"x": 299., "y": 199.}
        self.measure_results = collections.deque()
        self.measure_calls, self.native_accel_calls, self.current_calls = [], [], []
        self.on_measure, self.on_begin, self.on_current = None, None, None
        self.stop_requests = []
        self.owner.sensor_fault = None
        self.owner._uses_vector_detector = lambda axis: self.workflow.simple_detection(axis)
        self.owner._sensor_fault = self.sensor_fault
        self.owner._handle_rails_begin = self.begin
        self.owner._end_session = self.end_limits_session
        self.workflow._beta_current_context = lambda axis, purpose="Beta": (
            self.current_helper if self.use_current else None,
            {"configured": {}, "physical_current_verified": False})
        self.use_current = False
        self.mapped_axes = set()
        self.workflow.detection_axis_mapping = lambda axis: (
            {"stub": True} if axis in self.mapped_axes else None)
        self.configfile = NativeConfigfile(self)
        self.objects["configfile"] = self.configfile
        self.get_start_args = lambda: {"config_file": "/synthetic/printer.cfg"}
        for axis in "xy":
            self.set_rail(axis)

    def set_rail(self, axis, low=0., positive=False, endstop=None):
        span = 300. if axis == "x" else 200.
        rail = Rail(positive=positive, bounds=(low, low + span), endstop=endstop)
        rail.info.second_homing_speed, rail.info.accel = rail.info.speed, None
        steppers = [s for s in self.toolhead.kin.get_steppers() if s.is_active_axis(axis)]
        endstop_object = types.SimpleNamespace(get_steppers=lambda: steppers)
        self.owner.endstops[axis] = endstop_object
        rail.get_endstops = lambda: [(endstop_object, "accelerometer_" + axis)]
        self.owner.rails_by_axis[axis] = rail

    def sensor_fault(self, reason):
        self.owner.sensor_fault = reason
        self.stop_requests.append(reason)

    def begin(self, state, rails):
        axis = self.workflow.session["axis"]
        self.workflow.check_home_axes({axis: rails[0]})
        self.owner._end_session()
        self.owner.rails_by_axis[axis] = rails[0]
        self.owner.homing_active = True
        self.owner._limit_accel()
        if self.on_begin:
            self.on_begin(axis)

    def end_limits_session(self):
        self.end_session()
        self.owner._restore_accel()

    def current_helper(self, axes, enabled):
        self.current_calls.append((tuple(axes), enabled))
        if self.on_current:
            self.on_current(enabled)

    def model(self, axis):
        metadata = self.workflow._model_context(axis)
        metadata.update(impact_metric="mapped_axis", polarity_policy="absolute_after_reference_subtraction_v1",
                        original_calibration_evidence="retained")
        self.workflow.model = types.SimpleNamespace(thresholds={axis: 7.}, metadata={axis: metadata})
        self.owner.threshold_g[axis] = 7.
        # A saved mapped-axis model: stand in for its validated mapping profile.
        self.mapped_axes.add(axis)
        parser = self.configfile.autosave.fileconfig
        parser.add_section(self.workflow.model_section)
        parser.set(self.workflow.model_section, axis + "_threshold", "7")
        parser.set(self.workflow.model_section, axis + "_successes", "10")
        parser.set(self.workflow.model_section, axis + "_metadata", json.dumps(metadata))
        return copy.deepcopy(metadata)
