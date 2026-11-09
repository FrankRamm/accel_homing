"""Source-only CENTER regressions. Do not execute without operator approval.

The native move and configfile below are in-memory fixtures. They check the
workflow's accounting/state boundaries, not actual impacts or printer behavior.
"""
import collections
import configparser
import copy
import io
import itertools
import json
import math
import types
import unittest
from unittest import mock

from harness import CommandError
from test_workflows import BACK_OFF, GCmd, Rail, Rig


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
        # the live pre-command limit that CENTER must restore.
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
        rig.workflow.center._check(session)
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
        metric = "xyz_magnitude" if owner._uses_vector_detector(axis) else "selected_axis"
        owner.last[axis] = {"axis": axis, "pass": 1, "result": item.get("result", "impact"),
                            "impact_metric": metric,
                            "polarity_policy": "absolute_after_reference_subtraction_v1",
                            "trigger_travel_mm_signed": trigger, "halt_travel_mm_signed": halt,
                            "onset_g": 8., "impact_peak_g": 9.}
        rig.toolhead.position[index] += halt
        return list(rig.toolhead.position)


class CenterRig(Rig):
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
        self.owner._end_session = self.end_center_session
        self.workflow.center._native_classes = lambda: (NativeState, NativeMove)
        self.workflow._beta_current_context = lambda axis, purpose="Beta": (
            self.current_helper if self.use_current else None,
            {"configured": {}, "physical_current_verified": False})
        self.use_current = False
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

    def end_center_session(self):
        self.end_session()
        self.owner._restore_accel()

    def current_helper(self, axes, enabled):
        self.current_calls.append((tuple(axes), enabled))
        if self.on_current:
            self.on_current(enabled)

    def center(self, **params):
        self.workflow.center.command(GCmd(CENTERED=1, **params))

    def model(self, axis):
        metadata = self.workflow._model_context(axis)
        metadata.update(impact_metric="xyz_magnitude", polarity_policy="absolute_after_reference_subtraction_v1",
                        original_calibration_evidence="retained")
        self.workflow.model = types.SimpleNamespace(thresholds={axis: 7.}, metadata={axis: metadata})
        self.owner.threshold_g[axis] = 7.
        parser = self.configfile.autosave.fileconfig
        parser.add_section(self.workflow.model_section)
        parser.set(self.workflow.model_section, axis + "_threshold", "7")
        parser.set(self.workflow.model_section, axis + "_successes", "10")
        parser.set(self.workflow.model_section, axis + "_metadata", json.dumps(metadata))
        return copy.deepcopy(metadata)


class CenterCalibrationTests(unittest.TestCase):
    def test_default_reports_both_axes_ten_direct_traverses_and_no_extra_moves(self):
        rig = CenterRig()
        rails = dict(rig.owner.rails_by_axis)
        z_limit = rig.toolhead.kin.limits[2]
        original_ze = rig.toolhead.position[2:]
        rig.center()
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "center_report")
        self.assertEqual(result["axes"], ["x", "y"])
        self.assertEqual(len(rig.measure_calls), 22)
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.dispatched_scripts, [])
        self.assertEqual(rig.scripts, [])
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.configfile.native_calls, 0)
        self.assertEqual(rig.owner.back_off_dist, BACK_OFF)
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertEqual(rig.owner.rails_by_axis, rails)
        self.assertEqual(rig.toolhead.position[2:], original_ze)
        self.assertEqual(rig.toolhead.kin.limits[2], z_limit)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
        for axis in "xy":
            item = result["axis_results"][axis]
            self.assertEqual(len(item["samples"]), 10)
            self.assertEqual(item["directions"]["+"]["count"], 5)
            self.assertEqual(item["directions"]["-"]["count"], 5)
            self.assertEqual(item["search"]["extra_allowance_mm"], 20.)
            calls = [call for call in rig.measure_calls if call["axis"] == axis]
            self.assertEqual([call["direction"] for call in calls], [-1, 1] * 5 + [-1])
            self.assertTrue(all(not any(call["back_off_dist"].values()) and call["probe_pos"] for call in calls))
        self.assertFalse(result["physical_position_verified"])
        self.assertFalse(result["save_config"])
        json.dumps(result, allow_nan=False)

    def test_axes_signs_and_nonzero_minima_produce_consistent_endpoints(self):
        for axis, positive, minimum in itertools.product("xy", (False, True), (-17.2, 0., 38.4)):
            with self.subTest(axis=axis, positive=positive, minimum=minimum):
                rig = CenterRig()
                prior_endstop = None if positive else minimum + 2.
                rig.set_rail(axis, low=minimum, positive=positive, endstop=prior_endstop)
                rig.center(AXIS=axis.upper())
                result = rig.workflow.last_result
                self.assertEqual(result["axes"], [axis])
                proposed = result["axis_results"][axis]["proposed"]
                self.assertEqual(proposed["position_min"], minimum)
                self.assertEqual(proposed["position_max"], minimum + rig.spans[axis])
                self.assertEqual(proposed["position_endstop"], proposed["position_max"] if positive else minimum)
                self.assertEqual(proposed["midpoint"], (minimum + proposed["position_max"]) * .5)
                self.assertEqual(len(rig.measure_calls), 11)
                self.assertEqual(rig.toolhead.moves, [])

    def test_primary_span_does_not_subtract_previous_stalled_step_overshoot(self):
        rig = CenterRig()
        rig.measure_results.extend([{"overshoot": .75}, {"overshoot": .4}])
        rig.center(AXIS="X")
        item = rig.workflow.last_result["axis_results"]["x"]
        first = item["samples"][0]
        self.assertEqual(first["commanded_halt_to_trigger_mm"], 299.)
        self.assertEqual(first["previous_overshoot_mm_signed"], -.75)
        self.assertEqual(first["alternate_commanded_trigger_to_trigger_mm"], 298.25)
        self.assertAlmostEqual(first["overshoot_mm_signed"], .4)
        self.assertEqual(item["proposed"]["position_max"], 299.)
        self.assertFalse(first["physical_position_verified"])

    def test_finite_default_allowance_discovers_larger_span_and_zero_can_refuse(self):
        rig = CenterRig()
        rig.spans["x"] = 303.
        rig.center(AXIS="X")
        self.assertEqual(rig.workflow.last_result["axis_results"]["x"]["proposed"]["position_max"], 303.)
        self.assertTrue(all(abs(call["target"][0]) == 320. for call in rig.measure_calls))
        limited = CenterRig()
        limited.spans["x"] = 303.
        with self.assertRaisesRegex(CommandError, "finite movement"):
            limited.center(AXIS="X", SEARCH_MARGIN=0)
        self.assertTrue(all(abs(call["target"][0]) == 300. for call in limited.measure_calls))
        self.assertEqual(limited.workflow.last_result["axis_results"], {})
        self.assertEqual(limited.configfile.set_calls, [])
        self.assertEqual(len(limited.measure_calls), 2)

    def test_native_currents_and_rail_accel_restore_prior_live_limit(self):
        rig = CenterRig()
        rig.use_current = True
        rig.owner.rails_by_axis["x"].info.accel = 1000.
        rig.toolhead.accel = 2000.  # Below module cap; no owner.saved_accel is created.
        rig.center(AXIS="X")
        self.assertEqual(rig.toolhead.accel, 2000.)
        self.assertTrue(all(call["accel"] == 1000. and call["speed"] == 100. for call in rig.measure_calls))
        self.assertEqual(rig.current_calls, [((0,), True), ((0,), False)] * 11)
        self.assertEqual(rig.native_accel_calls, [(1000., True), (1000., False)] * 11)
        self.assertIsNone(rig.owner.saved_accel)

    def test_invalid_parameters_and_homed_start_do_not_move(self):
        for arguments in ({"AXIS": "Z"}, {"SEARCH_MARGIN": math.nan}, {"SEARCH_MARGIN": math.inf},
                          {"SEARCH_MARGIN": -1}, {"BACKOFF": 0}, {"SAVE_CONFIG": 2}):
            with self.subTest(arguments=arguments):
                rig = CenterRig()
                with self.assertRaises(CommandError):
                    rig.center(**arguments)
                self.assertEqual(rig.measure_calls, [])
                self.assertEqual(rig.configfile.set_calls, [])
        rig = CenterRig()
        rig.toolhead.kin.limits[0] = (0., 300.)
        with self.assertRaisesRegex(CommandError, "both X and Y unhomed"):
            rig.center(AXIS="Y")
        self.assertEqual(rig.measure_calls, [])

    def test_missing_pressed_early_or_nonrepeatable_evidence_never_proposes(self):
        cases = ([{}, {"drop_stepper": True}], [{}, {"duplicate_stepper": True}],
                 [{}, {"result": "pressed"}], [{"distance": 20.}],
                 [{}, {"distance": 40.}],
                 [{}] + [{"distance": 298. if number % 2 else 301.} for number in range(10)])
        for records in cases:
            with self.subTest(records=records):
                rig = CenterRig()
                rig.measure_results.extend(records)
                with self.assertRaises(CommandError):
                    rig.center(AXIS="X", SAVE_CONFIG=1)
                self.assertEqual(rig.workflow.last_result["axis_results"], {})
                self.assertEqual(rig.workflow.last_result["proposed_config"], {})
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertIsNone(rig.workflow.session)

    def test_second_axis_failure_discards_first_axis_proposal_and_all_saves(self):
        rig = CenterRig()
        rig.measure_results.extend([{}] * 11 + [CommandError("synthetic Y sensor failure")])
        with self.assertRaisesRegex(CommandError, "Y sensor failure"):
            rig.center(SAVE_CONFIG=1)
        self.assertEqual(rig.workflow.last_result["completed_traverses"], {"x": 10, "y": 0})
        self.assertEqual(rig.workflow.last_result["axis_results"], {})
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.configfile.native_calls, 0)

    def test_cancellation_disconnect_shutdown_and_motor_release_stop_progression(self):
        for ending in ("cancel", "disconnect", "shutdown", "motor_off", "sensor"):
            with self.subTest(ending=ending):
                rig = CenterRig()

                def stop(axis, phase):
                    if phase != "traverse_3":
                        return
                    if ending == "cancel":
                        rig.workflow.cmd_CANCEL(GCmd())
                    elif ending == "disconnect":
                        rig.workflow._disconnect()
                    elif ending == "shutdown":
                        rig.shutdown = True
                        rig.workflow._shutdown()
                    elif ending == "sensor":
                        rig.sensor_fault("sample_gap")
                    else:
                        rig.owner.operation_generation += 1
                        rig.owner._end_session()

                rig.on_measure = stop
                with self.assertRaises(CommandError):
                    rig.center(AXIS="X", SAVE_CONFIG=1)
                self.assertEqual(len(rig.measure_calls), 4)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.owner.back_off_dist, BACK_OFF)
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.workflow.last_result["axis_results"], {})

    def test_interrupt_during_begin_or_current_helper_prevents_the_move(self):
        for boundary in ("begin", "current"):
            with self.subTest(boundary=boundary):
                rig = CenterRig()
                if boundary == "begin":
                    rig.on_begin = lambda axis: rig.workflow.cmd_CANCEL(GCmd())
                else:
                    rig.use_current = True
                    rig.on_current = lambda enabled: rig.workflow.cmd_CANCEL(GCmd()) if enabled else None
                with self.assertRaises(CommandError):
                    rig.center(AXIS="X", SAVE_CONFIG=1)
                self.assertEqual(rig.measure_calls, [])
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.owner.back_off_dist, BACK_OFF)

    def test_pending_config_and_include_conflicts_refuse_save_before_motion(self):
        for conflict in ("pending", "include"):
            with self.subTest(conflict=conflict):
                rig = CenterRig()
                if conflict == "pending":
                    rig.configfile.set("extruder", "pid_kp", "42")
                    rig.configfile.set_calls.clear()
                else:
                    rig.configfile.included = {"stepper_x": {"position_max": "300"}}
                before = copy.deepcopy(rig.configfile.saved)
                with self.assertRaises(CommandError):
                    rig.center(AXIS="X", SAVE_CONFIG=1)
                self.assertEqual(rig.measure_calls, [])
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.configfile.saved, before)
                # Report-only does not invoke save preflight or touch pending config.
                rig.center(AXIS="X")
                self.assertEqual(rig.configfile.saved, before)

    def test_native_save_happens_after_all_cleanup_without_macro_dispatch(self):
        rig = CenterRig()
        rails = dict(rig.owner.rails_by_axis)
        macro_calls = []
        rig.gcode.ready_gcode_handlers["SAVE_CONFIG"] = lambda gcmd: macro_calls.append(gcmd)

        def assert_clean():
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.owner.rails_by_axis, rails)
            self.assertEqual(rig.owner.back_off_dist, BACK_OFF)
            self.assertEqual(rig.toolhead.accel, 50000.)
            self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
            self.assertFalse(rig.owner.homing_active)
            self.assertEqual(len(rig.measure_calls), 22)

        rig.configfile.on_save = assert_clean
        rig.center(SAVE_CONFIG=1)
        self.assertTrue(rig.workflow.last_result["save_config"])
        self.assertEqual(rig.configfile.native_calls, 1)
        self.assertEqual(macro_calls, [])
        self.assertEqual(rig.dispatched_scripts, [])
        self.assertTrue(all(option in ("position_max", "position_endstop")
                            for section, option, value in rig.configfile.set_calls))

    def test_staging_and_native_save_failures_restore_previous_autosave(self):
        for failure in ("stage", "save"):
            with self.subTest(failure=failure):
                rig = CenterRig()
                parser = rig.configfile.autosave.fileconfig
                parser.add_section("bed_mesh default")
                parser.set("bed_mesh default", "version", "1")
                before = copy.deepcopy(rig.configfile.saved)
                if failure == "stage":
                    rig.configfile.fail_set_at = 2
                else:
                    rig.configfile.save_error = CommandError("synthetic native I/O failure")
                with self.assertRaises(CommandError):
                    rig.center(SAVE_CONFIG=1)
                self.assertEqual(rig.configfile.saved, before)
                self.assertEqual(rig.configfile.status_save_pending, {})
                self.assertFalse(rig.configfile.save_config_pending)
                self.assertEqual(rig.workflow.last_result["result"], "save_failed")
                self.assertFalse(rig.workflow.last_result["save_config"])

    def test_pending_config_introduced_by_final_report_is_not_saved_or_discarded(self):
        rig = CenterRig()

        def report(message):
            rig.output.append(message)
            if message.startswith("accel_homing: Saving measured coordinate proposal"):
                rig.configfile.set("extruder", "pid_kp", "42")

        rig.gcode.respond_info = report
        with self.assertRaisesRegex(CommandError, "unrelated SAVE_CONFIG"):
            rig.center(AXIS="X", SAVE_CONFIG=1)
        self.assertEqual(rig.configfile.native_calls, 0)
        self.assertEqual(rig.configfile.saved, {("extruder", "pid_kp"): "42"})
        self.assertEqual(rig.configfile.status_save_pending, {"extruder": {"pid_kp": "42"}})

    def test_cleanup_failure_prevents_proposal_and_staging(self):
        rig = CenterRig()
        cleanup = rig.workflow.center._cleanup

        def fail_after_cleanup(session, plans):
            cleanup(session, plans)
            raise CommandError("synthetic final cleanup failure")

        with mock.patch.object(rig.workflow.center, "_cleanup", side_effect=fail_after_cleanup):
            with self.assertRaisesRegex(CommandError, "final cleanup failure"):
                rig.center(AXIS="X", SAVE_CONFIG=1)
        self.assertEqual(rig.workflow.last_result["axis_results"], {})
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.configfile.native_calls, 0)

    def test_saved_model_rebase_preserves_calibration_and_survives_temporary_override_restart(self):
        for temporary in (False, True):
            with self.subTest(temporary=temporary):
                rig = CenterRig()
                rig.set_rail("x", low=-17.2, positive=True)
                original = rig.model("x")
                if temporary:
                    rig.owner.threshold_g["x"] = 8.
                    rig.workflow.note_manual_threshold("x", 8.)
                rig.center(AXIS="X", SAVE_CONFIG=1)
                metadata = json.loads(rig.configfile.saved[(rig.workflow.model_section, "x_metadata")])
                self.assertEqual(metadata["calibration_geometry"], original["geometry"])
                self.assertEqual(metadata["center_coordinate_rebase"]["previous_geometry"], original["geometry"])
                self.assertFalse(metadata["center_coordinate_rebase"]["threshold_calibration_rerun"])
                self.assertEqual(metadata["original_calibration_evidence"], "retained")
                self.assertEqual(rig.configfile.saved[(rig.workflow.model_section, "x_threshold")], "7")
                self.assertEqual(rig.configfile.saved[(rig.workflow.model_section, "x_successes")], "10")
                rail = rig.owner.rails_by_axis["x"]
                rail.bounds = (metadata["geometry"]["minimum"], metadata["geometry"]["maximum"])
                rail.info.position_endstop = metadata["geometry"]["endstop"]
                rig.workflow.model.metadata["x"] = metadata
                rig.workflow.temporary_manual_thresholds.clear()
                rig.owner.threshold_g["x"] = 7.
                rig.workflow._validate_model_connection()
                self.assertEqual(metadata["geometry"], rig.workflow._model_context("x")["geometry"])

    def test_report_only_does_not_rebase_model_or_stage_anything(self):
        rig = CenterRig()
        original = rig.model("x")
        before = copy.deepcopy(rig.configfile.saved)
        rig.center(AXIS="X")
        self.assertEqual(rig.workflow.model.metadata["x"], original)
        self.assertEqual(rig.configfile.saved, before)
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.workflow.last_result["model_metadata_rebase_proposed"], ["x"])


if __name__ == "__main__":
    unittest.main()
