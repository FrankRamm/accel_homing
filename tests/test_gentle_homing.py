"""Gentle native dispatch, model guards and contact qualification; offline only."""
import itertools
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, ConfigError, plugin
import test_auto_notch_workflow as auto_tests
from test_calibration_config import NativeShape
from test_gentle_auto_workflow import GentleAutoRig
from test_mapped_detector import deliver, mapping, prepared
from test_workflows import AccelHomingModel, Config, GCmd, Rig, workflow_module


class GentleHomingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="gentle-homing-")
        self.addCleanup(self.directory.cleanup)

    def rig(self):
        rig = Rig({"gentle_homing": True, "charts_directory": self.directory.name})
        rig.owner.homing_accel = 1000.
        for rail in rig.owner.rails_by_axis.values():
            rail.info.speed = 40.
        return rig

    def qualified_model(self, rig, axis="x"):
        threshold = rig.owner.threshold_g[axis]
        metadata = dict(rig.workflow._model_context(axis), gentle_homing=True,
                        impact_metric="mapped_axis", saved_threshold_tested=True,
                        tested_threshold_g=threshold, saved_threshold_g=threshold,
                        signal_filter={"type": "disabled"}, axis_mapping=mapping(axis, "xy".index(axis)),
                        axis_mapping_context=rig.sensor_context())
        if rig.workflow.model is None:
            rig.workflow.model = types.SimpleNamespace(thresholds={}, metadata={})
        rig.workflow.model.thresholds[axis] = threshold
        rig.workflow.model.metadata[axis] = metadata
        return metadata

    def observe_saved_seek(self, rig, attempt, home_end):
        # The workflow fixture is loaded under an artificial Klipper package.
        # Supply its real sibling module, without replacing planner validation.
        name = workflow_module.__package__ + ".accel_homing_auto"
        with mock.patch.dict(sys.modules, {name: auto_tests.auto}):
            rig.workflow.observe_saved_gentle_seek(attempt, home_end)

    def test_native_centered_dispatch_bounds_both_axes_and_directions(self):
        for axis, positive, mode in itertools.product("xy", (False, True),
                                                     ("calibration", "accuracy", "troubleshoot", "test", "charts")):
            with self.subTest(axis=axis, positive=positive, mode=mode):
                rig = self.rig()
                rail = rig.owner.rails_by_axis[axis]
                rail.info.positive_dir = positive
                rail.info.position_endstop = rail.bounds[1] if positive else rail.bounds[0]
                workflow = rig.workflow
                workflow.session = {"mode": mode, "axis": axis}
                workflow.in_home, workflow.home_axis = True, axis
                owner = rig.owner
                owner.ordinary_home = None
                owner.kinematics_home = mock.Mock(side_effect=AssertionError("unbounded kinematics dispatch"))
                homing = types.SimpleNamespace(get_axes=lambda: ["xy".index(axis)],
                                               home_rails=mock.Mock(return_value="native single rail"))
                before = rig.toolhead.get_position()
                result = plugin.BeaconAccelHoming._home_with_retract(owner, homing)
                self.assertEqual(result, "native single rail")
                owner.kinematics_home.assert_not_called()
                homing.home_rails.assert_called_once()
                rails, forcepos, target = homing.home_rails.call_args.args
                self.assertEqual(rails, [rail])
                center, expected = workflow._geometry(axis)
                index = "xy".index(axis)
                self.assertEqual(forcepos[index], center - 2. if positive else center + 2.)
                self.assertEqual(target[index], rail.info.position_endstop)
                self.assertEqual(abs(target[index] - forcepos[index]), expected + 2.)
                self.assertTrue(all(value is None for n, value in enumerate(forcepos) if n != index))
                self.assertTrue(all(value is None for n, value in enumerate(target) if n != index))
                self.assertEqual(rig.toolhead.get_position(), before)
                self.assertIsNone(owner.ordinary_home)

    def test_centered_dispatch_rejects_extra_or_wrong_axis(self):
        for axes in ([0, 1], [1], [], [2]):
            rig = self.rig()
            rig.workflow.home_axis = "x"
            homing = types.SimpleNamespace(get_axes=lambda: axes, home_rails=mock.Mock())
            with self.subTest(axes=axes), self.assertRaises(CommandError):
                rig.workflow.centered_gentle_home(homing)
            homing.home_rails.assert_not_called()

    def test_ordinary_homing_keeps_native_kinematics_dispatch(self):
        for enabled in (False, True):
            rig = self.rig()
            rig.workflow.gentle_homing = enabled
            owner = rig.owner
            owner.ordinary_home = None
            owner.kinematics_home = mock.Mock(return_value="ordinary native result")
            homing = types.SimpleNamespace(get_axes=lambda: [0], home_rails=mock.Mock())
            self.assertEqual(plugin.BeaconAccelHoming._home_with_retract(owner, homing), "ordinary native result")
            owner.kinematics_home.assert_called_once_with(homing)
            homing.home_rails.assert_not_called()
            self.assertIsNone(owner.ordinary_home)

    def test_missing_or_stale_model_blocks_before_core_preparation(self):
        mutations = {
            "missing": lambda rig, saved: setattr(rig.workflow, "model", None),
            "legacy": lambda rig, saved: saved.pop("gentle_homing"),
            "untested": lambda rig, saved: saved.update(saved_threshold_tested=False),
            "tested_threshold": lambda rig, saved: saved.update(tested_threshold_g=1.),
            "saved_threshold": lambda rig, saved: saved.update(saved_threshold_g=1.),
            "model_threshold": lambda rig, saved: rig.workflow.model.thresholds.update(x=1.),
            "runtime_threshold": lambda rig, saved: rig.owner.threshold_g.update(x=1.),
            "speed": lambda rig, saved: saved.update(homing_speed_mm_s=125.),
            "accel": lambda rig, saved: saved.update(homing_accel_mm_s2=5000.),
            "filter": lambda rig, saved: saved.update(signal_filter={"type": "different"}),
            "invalidated_mapping": lambda rig, saved: saved.update(axis_mapping_invalidated=True),
            "sensor_context": lambda rig, saved: saved.update(axis_mapping_context={"sensor": "different"}),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                rig = self.rig()
                saved = self.qualified_model(rig)
                mutate(rig, saved)
                rail = rig.owner.rails_by_axis["x"]
                rail.get_endstops = lambda: [(rig.owner.endstops["x"], "stepper_x")]
                before = rig.effects()
                with mock.patch.object(rig.owner, "_end_session") as prepare, self.assertRaises(CommandError):
                    plugin.BeaconAccelHoming._handle_rails_begin(rig.owner, object(), [rail])
                prepare.assert_not_called()
                self.assertEqual(rig.effects(), before)

    def test_valid_model_allows_ordinary_home_and_calibration_allows_no_model(self):
        for axis in "xy":
            rig = self.rig()
            self.qualified_model(rig, axis)
            rig.workflow.check_home_axes({axis: rig.owner.rails_by_axis[axis]})
            rig.workflow.model = None
            rig.workflow.session = {"mode": "calibration", "axis": axis}
            rig.workflow.in_home, rig.workflow.home_axis = True, axis
            rig.workflow.check_home_axes({axis: rig.owner.rails_by_axis[axis]})
            self.assertEqual(rig.toolhead.moves, [])

    def test_test_and_charts_abort_first_bad_contact_without_release_or_second_axis(self):
        for mode in ("TEST", "CHARTS"):
            with self.subTest(mode=mode):
                rig = self.rig()
                for axis in "xy":
                    self.qualified_model(rig, axis)
                rig.home_results.append({"travel_mm": 80.})
                with self.assertRaises(CommandError):
                    getattr(rig.workflow, "cmd_" + mode)(GCmd(CENTERED=1))
                self.assertEqual([call["axis"] for call in rig.home_calls], ["x"])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.workflow.last_result["result"], "rejected")

    def test_ordinary_contact_accepts_short_start_but_checks_post_trigger_budget(self):
        for overshoot in (.5, .5001, None):
            with self.subTest(overshoot=overshoot):
                rig = self.rig()
                record = {"axis": "x", "result": "impact", "travel_mm": .01,
                          "trigger_travel_mm_signed": -.01,
                          "halt_travel_mm_signed": None if overshoot is None else -.01 - overshoot}
                if overshoot == .5:
                    rig.workflow.check_home_result(record)
                else:
                    with self.assertRaises(CommandError):
                        rig.workflow.check_home_result(record)
                self.assertEqual(rig.toolhead.moves, [])

    def test_saved_phase_policy_checks_original_onset_against_native_seek_clock(self):
        for axis, positive, onset in itertools.product(
                "xy", (False, True), (10.03, 10.1, 10.25, 10.27)):
            # The first two onsets are acceleration/cruise; the exact brake
            # boundary and a later braking onset must both be rejected.
            accepted = onset < 10.25
            with self.subTest(axis=axis, positive=positive, onset=onset):
                rig = self.rig()
                rail = rig.owner.rails_by_axis[axis]
                rail.info.positive_dir = positive
                rail.info.position_endstop = rail.bounds[1] if positive else rail.bounds[0]
                metadata = self.qualified_model(rig, axis)
                metadata["gentle_contact_phase_policy"] = "prebrake_v1"
                vector = [0., 0., 0.]
                vector["xy".index(axis)] = 1. if positive else -1.
                moves = [types.SimpleNamespace(print_time=start, move_t=duration, start_v=speed,
                                               accel=accel, x_r=vector[0], y_r=vector[1], z_r=vector[2])
                         for start, duration, speed, accel in (
                             (10.01, .04, 0., 1000.), (10.05, .2, 40., 0.), (10.25, .04, 40., -1000.))]
                extractor = mock.Mock(return_value=(moves, []))
                rig.objects["motion_report"] = types.SimpleNamespace(trapqs={
                    "toolhead": types.SimpleNamespace(extract_trapq=extractor)})
                rig.workflow.check_home_axes({axis: rail})
                record = {"axis": axis, "result": "impact", "travel_mm": .1,
                          "trigger_travel_mm_signed": .1, "halt_travel_mm_signed": .2,
                          "onset_time_s": onset}
                attempt = types.SimpleNamespace(axis=axis, pass_no=1, reason="impact", arm_time=10., record=record)
                self.observe_saved_seek(rig, attempt, 10.3)
                receipt = record["gentle_contact_phase"]
                self.assertEqual(receipt["accepted"], accepted, receipt)
                self.assertEqual(receipt["trigger_onset_time_s"], onset)
                self.assertEqual(receipt["brake_start_time_s"], 10.25)
                self.assertEqual(receipt["phase"], "acceleration" if onset < 10.05 else
                                 "cruise" if onset < 10.25 else "braking")
                self.assertFalse(receipt["physical_contact_verified"])
                extractor.assert_called_once_with(10., rig.reactor.NEVER)
                if accepted:
                    rig.workflow.check_home_result(record)
                else:
                    with self.assertRaisesRegex(CommandError, "before commanded braking"):
                        rig.workflow.check_home_result(record)
                self.assertEqual(rig.toolhead.moves, [])

    def test_saved_phase_policy_fails_closed_when_native_timing_is_unavailable(self):
        for missing in (True, False):
            with self.subTest(missing_extractor=missing):
                rig = self.rig()
                self.qualified_model(rig)["gentle_contact_phase_policy"] = "prebrake_v1"
                extractor = mock.Mock(side_effect=ValueError("native schedule was discarded"))
                if not missing:
                    rig.objects["motion_report"] = types.SimpleNamespace(dtrapqs={
                        "toolhead": types.SimpleNamespace(extract_trapq=extractor)})
                else:
                    with self.assertRaisesRegex(CommandError, "trapq extractor"):
                        rig.workflow.check_home_axes({"x": rig.owner.rails_by_axis["x"]})
                record = {"axis": "x", "result": "impact", "travel_mm": .1,
                          "trigger_travel_mm_signed": .1, "halt_travel_mm_signed": .2,
                          "onset_time_s": 10.1}
                attempt = types.SimpleNamespace(axis="x", pass_no=1, reason="impact", arm_time=10., record=record)
                self.observe_saved_seek(rig, attempt, 10.3)
                self.assertFalse(record["gentle_contact_phase"]["accepted"])
                with self.assertRaisesRegex(CommandError, "before commanded braking"):
                    rig.workflow.check_home_result(record)
                self.assertEqual(extractor.call_count, 0 if missing else 1)
                self.assertEqual(rig.toolhead.moves, [])

    def test_previous_saved_models_do_not_require_the_new_native_phase_receipt(self):
        rig = self.rig()
        self.qualified_model(rig)
        extractor = mock.Mock(side_effect=AssertionError("legacy model must not acquire a new phase requirement"))
        rig.objects["motion_report"] = types.SimpleNamespace(trapqs={
            "toolhead": types.SimpleNamespace(extract_trapq=extractor)})
        rig.workflow.check_home_axes({"x": rig.owner.rails_by_axis["x"]})
        record = {"axis": "x", "result": "impact", "travel_mm": .1,
                  "trigger_travel_mm_signed": .1, "halt_travel_mm_signed": .2,
                  "onset_time_s": None}
        attempt = types.SimpleNamespace(axis="x", pass_no=1, reason="impact", arm_time=10., record=record)
        self.observe_saved_seek(rig, attempt, 10.3)
        self.assertNotIn("gentle_contact_phase", record)
        rig.workflow.check_home_result(record)
        extractor.assert_not_called()

    def test_saved_phase_policy_uses_real_core_record_and_does_not_reuse_a_previous_receipt(self):
        rig = self.rig()
        self.qualified_model(rig)["gentle_contact_phase_policy"] = "prebrake_v1"
        core, attempt, _mapping = prepared()
        deliver(core, attempt, [(.1, 0., 0.)] * 3 + [(1.25, 0., 0.)] * 5)
        core.disarm()  # Exercise the actual runtime record constructor.
        record = attempt.record
        record.update(travel_mm=.1, trigger_travel_mm_signed=.1, halt_travel_mm_signed=.2)
        self.assertNotIn("trigger_onset_time_s", record)
        arm = attempt.arm_time
        moves = [types.SimpleNamespace(print_time=arm + offset, move_t=duration, start_v=speed,
                                       accel=accel, x_r=-1., y_r=0., z_r=0.)
                 for offset, duration, speed, accel in (
                     (0., .04, 0., 1000.), (.04, .2, 40., 0.), (.24, .04, 40., -1000.))]
        extractor = mock.Mock(return_value=(moves, []))
        rig.objects["motion_report"] = types.SimpleNamespace(trapqs={
            "toolhead": types.SimpleNamespace(extract_trapq=extractor)})
        self.observe_saved_seek(rig, attempt, arm + .1)
        self.assertTrue(record["gentle_contact_phase"]["accepted"], record["gentle_contact_phase"])
        self.assertEqual(record["gentle_contact_phase"]["trigger_onset_time_s"], record["onset_time_s"])
        rig.workflow.check_home_result(record)

        # A following home gets its own core record. The previous accepted
        # receipt cannot qualify missing telemetry or a missing observation.
        core, following, _mapping = prepared()
        deliver(core, following, [(1.25, 0., 0.)] * 8)
        core.disarm()
        fresh = following.record
        fresh.update(travel_mm=.1, trigger_travel_mm_signed=.1, halt_travel_mm_signed=.2)
        self.assertNotIn("gentle_contact_phase", fresh)
        with self.assertRaisesRegex(CommandError, "missing native seek timing"):
            rig.workflow.check_home_result(fresh)
        extractor.side_effect = ValueError("current seek unavailable")
        self.observe_saved_seek(rig, following, following.arm_time + .1)
        self.assertFalse(fresh["gentle_contact_phase"]["accepted"])
        with self.assertRaisesRegex(CommandError, "current seek unavailable"):
            rig.workflow.check_home_result(fresh)
        self.assertTrue(record["gentle_contact_phase"]["accepted"])

    def test_saved_phase_policy_reload_accepts_supported_or_absent_and_rejects_unknown_values(self):
        for axis in "xy":
            rig = self.rig()
            original = self.qualified_model(rig, axis)
            for policy in (None, "prebrake_v1"):
                with self.subTest(axis=axis, valid_policy=policy):
                    metadata = dict(original)
                    if policy is not None:
                        metadata["gentle_contact_phase_policy"] = policy
                    model = AccelHomingModel(Config(None, {axis + "_threshold": original["tested_threshold_g"], axis + "_successes": 10,
                        axis + "_metadata": json.dumps(metadata)}, "accel_homing_model test"))
                    self.assertEqual(model.metadata[axis].get("gentle_contact_phase_policy"), policy)
            for policy in ("prebrake_v2", "", None, False, 1, [], {}):
                with self.subTest(axis=axis, invalid_policy=policy):
                    metadata = dict(original, gentle_contact_phase_policy=policy)
                    with self.assertRaisesRegex(ConfigError, "gentle_contact_phase_policy"):
                        AccelHomingModel(Config(None, {axis + "_threshold": original["tested_threshold_g"], axis + "_successes": 10,
                            axis + "_metadata": json.dumps(metadata)}, "accel_homing_model test"))

    def test_gentle_limits_reject_overrides_retraction_and_reduced_toolhead_caps(self):
        mutations = {
            "fast": lambda rig: setattr(rig.owner.rails_by_axis["x"].info, "speed", 40.001),
            "accel": lambda rig: setattr(rig.owner, "homing_accel", 1000.001),
            "nan_speed": lambda rig: setattr(rig.owner.rails_by_axis["x"].info, "speed", float("nan")),
            "retract": lambda rig: setattr(rig.owner.rails_by_axis["x"].info, "retract_dist", .1),
            "min_home_dist": lambda rig: setattr(rig.owner.rails_by_axis["x"].info, "min_home_dist", .1),
            "rail_accel": lambda rig: setattr(rig.owner.rails_by_axis["x"].info, "accel", 500.),
            "live_velocity": lambda rig: setattr(rig.toolhead, "get_max_velocity", lambda: (39., 50000.)),
            "live_accel": lambda rig: setattr(rig.toolhead, "accel", 999.),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                rig = self.rig()
                mutate(rig)
                with self.assertRaises(CommandError):
                    rig.workflow._check_gentle_limits("x", CommandError)
                with self.assertRaises(ConfigError):
                    rig.workflow._validate_model_connection()
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.toolhead.moves, [])

    def test_premature_trigger_aborts_without_retry_return_or_threshold_increase(self):
        for travel in (8., 80.):
            with self.subTest(travel=travel):
                rig = GentleAutoRig(self.directory.name)
                rig.home_results.append({"travel_mm": travel})
                initial = dict(rig.owner.threshold_g)
                moves_at_contact = []
                rig.on_home = lambda _axis: moves_at_contact.append(list(rig.toolhead.moves))
                with rig.helpers(), self.assertRaises(CommandError):
                    rig.command(START_THRESHOLD=2.25)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25])
                self.assertEqual(rig.toolhead.moves, moves_at_contact[0])
                self.assertEqual(len([event for event in rig.beta_events if event[0] == "central_move"]), 6)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.owner.threshold_g, initial)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_post_trigger_travel_boundary_aborts_before_another_home_or_recenter(self):
        for positive, overshoot in itertools.product((False, True), (.5, .5001, None)):
            with self.subTest(positive=positive, overshoot=overshoot):
                rig = GentleAutoRig(self.directory.name)
                rail = rig.owner.rails_by_axis["x"]
                rail.info.positive_dir = positive
                rail.info.position_endstop = 300. if positive else 0.
                direction = 1. if positive else -1.
                rig.home_results.append({"travel_mm": 150., "trigger_travel_mm_signed": direction * 150.,
                                         "halt_travel_mm_signed": (None if overshoot is None else
                                                                    direction * (150. + overshoot))})
                moves_at_contact = []
                rig.on_home = lambda _axis: moves_at_contact.append(list(rig.toolhead.moves))
                with rig.helpers():
                    if overshoot == .5:
                        rig.command(START_THRESHOLD=2.25)
                        self.assertEqual(len(rig.home_calls), 11)
                        self.assertEqual(rig.workflow.last_result["result"], "auto_notch_proposed")
                    else:
                        with self.assertRaises(CommandError):
                            rig.command(START_THRESHOLD=2.25)
                        self.assertEqual(len(rig.home_calls), 1)
                        self.assertEqual(rig.toolhead.moves, moves_at_contact[0])
                        self.assertEqual(rig.configfile.saved, {})
                        self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_recenter_and_final_release_use_gentle_caps_and_preserve_lower_retract_speed(self):
        for retract_speed in (20., 125.):
            with self.subTest(retract_speed=retract_speed):
                rig = GentleAutoRig(self.directory.name)
                rig.owner.rails_by_axis["x"].info.retract_speed = retract_speed
                observed = []
                move = rig.toolhead.manual_move

                def observe(coords, speed):
                    observed.append((list(coords), speed, rig.toolhead.accel))
                    move(coords, speed)

                rig.toolhead.manual_move = observe
                with rig.helpers():
                    rig.command(START_THRESHOLD=2.25)
                self.assertTrue(observed)
                self.assertTrue(all(speed <= 40. and accel == 1000. for _, speed, accel in observed))
                self.assertEqual(observed[-1], ([5., None, None], min(retract_speed, 40.), 1000.))
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertIsNone(rig.owner.saved_accel)

    def test_gentle_save_preserves_exact_threshold_qualified_by_all_eleven_homes(self):
        for axis, save in itertools.product("xy", (0, 1)):
            with self.subTest(axis=axis, save=save):
                rig = GentleAutoRig(self.directory.name)
                config = auto_tests.AutoNotchWorkflowTests().native_config(rig, Path(self.directory.name), NativeShape)
                with rig.helpers():
                    rig.command(axis=axis.upper(), bare=True, SAVE_CONFIG=save, START_THRESHOLD=2.25)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25] * 11)
                saved = config.read_main_config().fileconfig if save else config.autosave.fileconfig
                self.assertEqual(saved.getfloat("accel_homing", axis + "_threshold"), 2.25)
                self.assertEqual(saved.getfloat("accel_homing_model default", axis + "_threshold"), 2.25)
                metadata = json.loads(saved.get("accel_homing_model default", axis + "_metadata"))
                self.assertTrue(metadata["gentle_homing"])
                self.assertTrue(metadata["saved_threshold_tested"])
                self.assertEqual(metadata["tested_threshold_g"], 2.25)
                self.assertEqual(metadata["saved_threshold_g"], 2.25)
                self.assertEqual(metadata["threshold_margin_g"], 0.)
                self.assertEqual(metadata["homing_speed_mm_s"], 40.)
                self.assertEqual(metadata["homing_accel_mm_s2"], 1000.)
                self.assertEqual(metadata["signal_filter"], rig.home_profiles[0]["candidate"]["profile"])
                self.assertTrue(metadata["auto_notch_selection"]["candidate_frozen_before_contact"])
                self.assertEqual(rig.gcode.request_restart.call_count, save)
                self.assertEqual(rig.workflow.last_result["persistence"],
                                 "native_save_requested" if save else "staged_not_saved")
                self.assertEqual(rig.toolhead.position[2:], [40., 6.])


if __name__ == "__main__":
    unittest.main()
