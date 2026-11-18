"""Offline bidirectional mapping, cleanup and persistence contracts."""
import copy
import json
import math
import types
import unittest
from unittest import mock

from harness import CommandError, ConfigError
from test_workflows import AccelHomingWorkflows, Config, GCmd, Rig, workflow_module
from accel_homing_model import AccelHomingModel


class MappingRig(Rig):
    def __init__(self, axis="x", source=2, sign=-1):
        super().__init__()
        self.axis, self.source, self.sign = axis, source, sign
        self.clock, self.profile, self.events = 10., [], []
        self.fail, self.interrupt = None, None
        self.toolhead.min_cruise_ratio = .5
        self.toolhead.get_last_move_time = lambda: self.clock
        self.owner.capture_limit, self.owner.stream_timeout = 100000, .05
        self.owner._enable_motors = lambda axis: self.event("enable")
        self.owner.beta_motion_capture = None
        self.owner.start_beta_motion_capture = self.start_mapping_capture
        self.owner.stop_beta_motion_capture = self.stop_mapping_capture
        self.objects["motion_report"] = types.SimpleNamespace(trapqs={
            "toolhead": types.SimpleNamespace(extract_trapq=self.extract_profile)})
        original_move = self.toolhead.manual_move

        def move(target, speed):
            current_axis = self.workflow.session["axis"]
            direction = 1 if target["xy".index(current_axis)] > self.toolhead.position["xy".index(current_axis)] else -1
            distance = abs(target["xy".index(current_axis)] - self.toolhead.position["xy".index(current_axis)])
            accel, start = self.toolhead.accel, self.clock + .01
            ramp, cruise = speed / accel, distance / speed - speed / accel
            vector = [0., 0., 0.]
            vector["xy".index(current_axis)] = direction
            self.profile = [types.SimpleNamespace(print_time=t, move_t=d, start_v=v, accel=a,
                                                  x_r=vector[0], y_r=vector[1], z_r=0.)
                            for t, d, v, a in ((start, ramp, 0., accel),
                                (start + ramp, cruise, speed, 0.),
                                (start + ramp + cruise, ramp, speed, -accel))]
            self.clock = start + ramp * 2 + cruise
            self.direction = direction
            original_move(target, speed)
            self.event("move")

        self.toolhead.manual_move = move
        self.workflow._calibration_axis_mapping = types.MethodType(
            AccelHomingWorkflows._calibration_axis_mapping, self.workflow)

    def event(self, stage):
        self.events.append(stage)
        if self.interrupt is not None:
            self.interrupt(stage)
        if self.fail == stage:
            raise CommandError("synthetic " + stage + " failure")

    def extract_profile(self, start, end):
        self.event("profile")
        return self.profile, None

    def start_mapping_capture(self, check):
        check()
        self.event("reference")
        token = {"sample_rate_hz": 1000., "capture_cutoff_time_s": self.clock}
        self.owner.beta_motion_capture = token
        return token

    def stop_mapping_capture(self, token, check):
        try:
            check()
            self.event("capture")
            start, end = token["capture_cutoff_time_s"], token["motion_end_time_s"]
            rows = []
            for number in range(math.ceil((end - start) * 1000.) + 2):
                t = start + number / 1000.
                values = [0., 0., 1.]
                for segment in self.profile:
                    if segment.print_time <= t < segment.print_time + segment.move_t:
                        values[self.source] += self.sign * self.direction * segment.accel / 9806.65
                rows.append((t,) + tuple(values))
            return {"samples": rows, "baseline_g": (0., 0., 1.), "dropped": 0, "fault": None,
                    "sensor_context": self.sensor_context(), "raw_samples_validated": True}
        finally:
            self.owner.beta_motion_capture = None

    def begin_mapping(self):
        self.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", self.axis)
        return self.workflow.session


class AxisMappingWorkflowTests(unittest.TestCase):
    def test_both_axes_all_channels_both_polarities_preserve_other_coordinates_and_normal_limits(self):
        for axis in "xy":
            for source in range(3):
                for sign in (-1, 1):
                    with self.subTest(axis=axis, source=source, sign=sign):
                        rig = MappingRig(axis, source, sign)
                        initial, limits, accel = rig.toolhead.get_position(), list(rig.toolhead.kin.limits), rig.toolhead.accel
                        session = rig.begin_mapping()
                        rig.workflow._calibration_axis_mapping(session)
                        self.assertEqual(session["axis_mapping"]["source_axis"], source)
                        self.assertEqual(session["axis_mapping"]["sign"], sign)
                        self.assertEqual(rig.toolhead.position, initial)
                        self.assertEqual(rig.toolhead.kin.limits, limits)
                        self.assertEqual(rig.toolhead.accel, accel)
                        index, center = "xy".index(axis), rig.workflow._geometry(axis)[0]
                        self.assertEqual([move[index] for move, speed in rig.toolhead.moves],
                                         [center + 15., center, center - 15., center])
                        self.assertTrue(all(speed == 20. for move, speed in rig.toolhead.moves))
                        self.assertTrue(all(move[2] is None and move[1 - index] is None
                                            for move, speed in rig.toolhead.moves))
                        self.assertEqual(rig.owner.rails_by_axis[axis].info.speed, 100.)
                        self.assertEqual(rig.owner.homing_accel, 5000.)
                        self.assertEqual(rig.home_calls, [])

    def test_mapping_precedes_noise_then_homing_and_original_homing_settings_remain(self):
        rig = MappingRig()
        session = rig.begin_mapping()
        observed = []
        def noise(current):
            self.assertEqual(current["axis_mapping_phase"], "qualified")
            self.assertEqual(rig.toolhead.accel, 50000.)
            observed.append("noise")
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(rig.workflow, "_calibration_noise_preflight", side_effect=noise), \
                mock.patch.object(rig.workflow, "_calibrate_trials", side_effect=lambda: observed.append("home")):
            rig.workflow._run_calibration()
        self.assertEqual(observed, ["noise", "home"])
        self.assertEqual(rig.events.count("capture"), 2)

    def test_failure_never_homes_saves_or_returns_after_uncertain_move(self):
        for stage in ("enable", "reference", "move", "profile", "capture"):
            with self.subTest(stage=stage):
                rig = MappingRig()
                rig.fail = stage
                limits = list(rig.toolhead.kin.limits)
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
                    rig.workflow._run_calibration()
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertLessEqual(len(rig.toolhead.moves), 1)
                self.assertEqual(rig.toolhead.kin.limits, limits)
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertIsNone(rig.owner.beta_motion_capture)
                self.assertIsNone(rig.workflow.session)

    def test_cancel_motor_release_and_sensor_change_at_yields_do_not_commit_mapping(self):
        for stage in ("enable", "reference", "move", "profile", "capture"):
            for fault in ("cancel", "motor_off", "sensor"):
                with self.subTest(stage=stage, fault=fault):
                    rig = MappingRig()
                    fired = []
                    def interrupt(current):
                        if current != stage or fired:
                            return
                        fired.append(True)
                        if fault == "cancel": rig.workflow.cmd_CANCEL(GCmd())
                        elif fault == "motor_off": rig.owner.operation_generation += 1
                        else: rig.owner.sensor_name = "different"
                    rig.interrupt = interrupt
                    rig.begin_mapping()
                    with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
                        rig.workflow._run_calibration()
                    self.assertTrue(fired)
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertLessEqual(len(rig.toolhead.moves), 1)
                    self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
                    self.assertEqual(rig.toolhead.position[2:], [40., 6.])
                    self.assertEqual(rig.toolhead.accel, 50000.)

    def test_ambiguous_or_clipped_evidence_aborts_before_noise_or_home(self):
        for failure in ("ambiguous", "clipped"):
            with self.subTest(failure=failure):
                rig = MappingRig()
                original = rig.owner.stop_beta_motion_capture
                def corrupted(token, check):
                    capture = original(token, check)
                    if failure == "clipped": capture["amplitude_limited"] = True
                    else: capture["samples"] = [(t, z - 1., y, z) for t, x, y, z in capture["samples"]]
                    return capture
                rig.owner.stop_beta_motion_capture = corrupted
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises((ValueError, CommandError)):
                    rig.workflow._run_calibration()
                self.assertEqual(rig.noise_calls, [])
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_noise_uses_mapped_component_even_if_another_component_is_larger(self):
        rig = Rig()
        original = rig.qualify_axis_mapping
        def mapped(session):
            original(session)
            session["axis_mapping"].update(source_axis=2, sign=-1)
        rig.workflow._calibration_axis_mapping = mapped
        rig.amplitudes = (5., 7., .6)
        with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
            rig.calibrate(save=0)
        self.assertEqual(rig.home_calls[0]["threshold"], 1.)
        result = rig.workflow.last_result["stationary_noise"]
        self.assertEqual(result["impact_metric"], "mapped_axis")
        self.assertAlmostEqual(result["noise_axis_peak_g"], .6)
        self.assertGreater(result["noise_magnitude_peak_g"], 8.)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertEqual(metadata["axis_mapping"]["source_axis"], 2)
        self.assertEqual(metadata["axis_mapping_context"], rig.sensor_context())

    def test_legacy_model_rejected_even_with_manual_threshold_and_mapped_model_keeps_orientation(self):
        rig = Rig({"x_threshold": 7.})
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 4.}, metadata={"x": {"impact_metric": "xyz_magnitude"}})
        with self.assertRaisesRegex(CommandError, "recalibrate"):
            rig.workflow.check_home_axes(["x"])
        session = {"axis": "x"}
        rig.qualify_axis_mapping(session)
        rig.workflow.model.metadata["x"] = {"impact_metric": "mapped_axis", **session}
        rig.workflow.check_home_axes(["x"])
        self.assertEqual(rig.workflow.detection_axis_mapping("x")["source_axis"], 0)
        self.assertTrue(rig.workflow.simple_detection("x"))
        rig.owner.sensor_name = "changed"
        with self.assertRaisesRegex(CommandError, "context changed"):
            rig.workflow.check_home_axes(["x"])

    def test_malformed_mapping_never_loads_and_cleanup_failure_does_not_stage(self):
        rig = Rig()
        session = {"axis": "x"}
        rig.qualify_axis_mapping(session)
        metadata = {"impact_metric": "mapped_axis", **session}
        for key, value in (("source_axis", True), ("sign", 0), ("coordinate_frame", "raw"), ("axis", "y")):
            bad = copy.deepcopy(metadata)
            bad["axis_mapping"][key] = value
            with self.subTest(key=key), self.assertRaises(ConfigError):
                AccelHomingModel(Config(None, {"x_threshold": 2., "x_successes": 10,
                                               "x_metadata": json.dumps(bad)}, "accel_homing_model test"))
        with mock.patch.object(rig.workflow, "_finish", side_effect=CommandError("cleanup failed")), \
                self.assertRaises(CommandError):
            rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, {})

    def test_frozen_mapping_receipts_must_match_on_every_trial(self):
        for target in ("record", "detector", "capture", "frame"):
            with self.subTest(target=target):
                rig = Rig()
                original = rig.owner.stop_capture
                def changed():
                    capture = original()
                    if target == "frame":
                        capture["coordinate_frame"] = "raw_sensor"
                    else:
                        item = capture["_record"] if target == "record" else capture["detector"] if target == "detector" else capture
                        item["axis_mapping"] = dict(item["axis_mapping"], source_axis=2)
                    return capture
                rig.owner.stop_capture = changed
                with self.assertRaisesRegex(CommandError, "axis mapping|calibration homing"):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.configfile.saved, {})

    def test_actual_terminal_cleanup_failure_reports_aborted_and_preserves_pending_model(self):
        rig = Rig()
        previous = {("accel_homing_model default", "x_threshold"): "3.5"}
        rig.configfile.saved.update(previous)
        original = rig.owner._end_session
        def cleanup():
            original()
            if rig.workflow.session.get("finishing"):
                raise CommandError("synthetic cleanup failure")
        with mock.patch.object(rig.owner, "_end_session", side_effect=cleanup), self.assertRaises(CommandError):
            rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, previous)
        self.assertEqual(rig.workflow.last_result["result"], "aborted")
        self.assertFalse(rig.workflow.last_result["save_config"])
        self.assertIsNone(rig.workflow.session)

    def saved_pair(self, rig):
        metadata = {}
        for axis in "xy":
            session = {"axis": axis}
            rig.qualify_axis_mapping(session)
            metadata[axis] = {**rig.workflow._model_context(axis), **session, "impact_metric": "mapped_axis"}
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 8., "y": 9.}, metadata=metadata)
        return copy.deepcopy(metadata)

    def test_ninety_degree_remount_can_recalibrate_both_axes_without_restart_or_stale_overwrite(self):
        rig = MappingRig(source=1, sign=1)
        original = self.saved_pair(rig)
        with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
            rig.calibrate("X", save=0)
            first_x = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            first_y = json.loads(rig.configfile.saved[("accel_homing_model default", "y_metadata")])
            self.assertEqual(first_x["axis_mapping"]["source_axis"], 1)
            self.assertTrue(first_y["axis_mapping_invalidated"])
            self.assertEqual(first_y["axis_mapping"], original["y"]["axis_mapping"])
            with self.assertRaisesRegex(CommandError, "invalidated"):
                rig.workflow.check_home_axes(["y"])
            with self.assertRaisesRegex(CommandError, "staged"):
                rig.workflow.check_home_axes(["x"])
            rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
            rig.idle_state = "Ready"
            rig.axis, rig.source, rig.sign = "y", 0, -1
            rig.calibrate("Y", save=0)
        final_x = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        final_y = json.loads(rig.configfile.saved[("accel_homing_model default", "y_metadata")])
        self.assertEqual(final_x, first_x)
        self.assertNotIn("axis_mapping_invalidated", final_y)
        self.assertEqual(final_y["axis_mapping"]["source_axis"], 0)
        self.assertEqual(final_y["axis_mapping"]["sign"], -1)
        self.assertEqual(rig.workflow.model.metadata, original)

    def test_failed_remount_calibration_preserves_other_axis_and_pending_pair(self):
        rig = MappingRig(source=1, sign=1)
        original = self.saved_pair(rig)
        rig.home_results.append(CommandError("synthetic home failure"))
        with self.assertRaises(CommandError):
            rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.workflow.pending_model_metadata, {})
        self.assertEqual(rig.workflow.model.metadata, original)

    def test_native_pending_configuration_rolls_back_whole_pair_on_partial_set_failure(self):
        # The workflow fixture isolates sys.modules while importing; use its
        # stdlib parser instance so discovery order cannot create two classes.
        parser = workflow_module.configparser.RawConfigParser()
        parser.add_section("accel_homing_model default")
        parser.set("accel_homing_model default", "y_metadata", "previous")
        config = types.SimpleNamespace(autosave=types.SimpleNamespace(fileconfig=parser),
                                       status_save_pending={"other": {"value": "retained"}}, save_config_pending=True)
        original = copy.deepcopy(config.status_save_pending)
        def setting(section, option, value):
            config.autosave.fileconfig.set(section, option, value)
            config.status_save_pending = {"mutated": value}
            if option == "y_metadata":
                raise CommandError("synthetic partial staging failure")
        config.set = setting
        with self.assertRaises(CommandError):
            AccelHomingWorkflows._stage_model_settings(config, [
                ("accel_homing_model default", "x_metadata", "new"),
                ("accel_homing_model default", "y_metadata", "invalidated")])
        self.assertEqual(dict(config.autosave.fileconfig.items("accel_homing_model default")), {"y_metadata": "previous"})
        self.assertEqual(config.status_save_pending, original)
        self.assertTrue(config.save_config_pending)

    def test_orientation_change_invalidates_peer_without_source_collision_and_without_pingpong(self):
        for source, sign in ((2, 1), (0, -1)):
            with self.subTest(source=source, sign=sign):
                rig = MappingRig(source=source, sign=sign)
                self.saved_pair(rig)
                with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
                    rig.calibrate("X", save=0)
                    peer = rig.workflow.pending_model_metadata["y"]
                    self.assertTrue(peer["axis_mapping_invalidated"])
                    rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
                    rig.idle_state = "Ready"
                    rig.axis, rig.source, rig.sign = "y", 1, 1
                    rig.calibrate("Y", save=0)
                self.assertNotIn("axis_mapping_invalidated", rig.workflow.pending_model_metadata["x"])
                self.assertNotIn("axis_mapping_invalidated", rig.workflow.pending_model_metadata["y"])

    def test_cancel_on_each_return_or_second_stroke_never_continues_motion(self):
        for move_number in (2, 3, 4):
            with self.subTest(move_number=move_number):
                rig = MappingRig()
                def cancel(stage):
                    if stage == "move" and rig.events.count("move") == move_number:
                        rig.workflow.cmd_CANCEL(GCmd())
                rig.interrupt = cancel
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
                    rig.workflow._run_calibration()
                self.assertEqual(len(rig.toolhead.moves), move_number)
                self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_acceleration_restore_failure_does_not_qualify_mapping_or_start_homing(self):
        rig = MappingRig()
        original = rig.owner._set_accel
        def setting(value):
            if value == 50000.:
                raise CommandError("synthetic acceleration restore failure")
            original(value)
        rig.owner._set_accel = setting
        session = rig.begin_mapping()
        with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaisesRegex(CommandError, "cleanup failed"):
            rig.workflow._run_calibration()
        self.assertNotIn("axis_mapping", session)
        self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.configfile.saved, {})


if __name__ == "__main__":
    unittest.main()
