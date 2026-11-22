"""Gentle native dispatch, model guards and contact qualification; offline only."""
import contextlib
import itertools
import json
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, ConfigError, plugin
from test_mapped_detector import mapping
from test_workflows import GCmd, Rig


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

    @contextlib.contextmanager
    def stationary_seed(self, rig):
        # The independent gentle seed suite exercises actual central strokes.
        # Keep real stationary seeding here, isolate only its moving input.
        def motion_peak(session):
            session["gentle_motion_noise"] = {"motion_peak_g": .01}
            return .01
        with mock.patch.object(rig.workflow, "_gentle_collect_motion_noise"), \
                mock.patch.object(rig.workflow, "_gentle_motion_seed_peak", side_effect=motion_peak), \
                mock.patch.object(rig.workflow, "_gentle_validate_motion_seed"):
            yield

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
                rig = self.rig()
                rig.home_results.append({"travel_mm": travel})
                initial = dict(rig.owner.threshold_g)
                with self.stationary_seed(rig), self.assertRaises(CommandError):
                    rig.calibrate("X", save=0, START_THRESHOLD=2.25)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.owner.threshold_g, initial)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_post_trigger_travel_boundary_aborts_before_another_home_or_recenter(self):
        for positive, overshoot in itertools.product((False, True), (.5, .5001, None)):
            with self.subTest(positive=positive, overshoot=overshoot):
                rig = self.rig()
                rail = rig.owner.rails_by_axis["x"]
                rail.info.positive_dir = positive
                rail.info.position_endstop = 300. if positive else 0.
                direction = 1. if positive else -1.
                rig.home_results.append({"travel_mm": 150., "trigger_travel_mm_signed": direction * 150.,
                                         "halt_travel_mm_signed": (None if overshoot is None else
                                                                    direction * (150. + overshoot))})
                with self.stationary_seed(rig):
                    if overshoot == .5:
                        rig.calibrate("X", save=0, START_THRESHOLD=2.25)
                        self.assertEqual(len(rig.home_calls), 11)
                        self.assertEqual(rig.workflow.last_result["result"], "calibrated")
                    else:
                        with self.assertRaises(CommandError):
                            rig.calibrate("X", save=0, START_THRESHOLD=2.25)
                        self.assertEqual(len(rig.home_calls), 1)
                        self.assertEqual(rig.toolhead.moves, [])
                        self.assertEqual(rig.configfile.saved, {})
                        self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_recenter_and_final_release_use_gentle_caps_and_preserve_lower_retract_speed(self):
        for retract_speed in (20., 125.):
            with self.subTest(retract_speed=retract_speed):
                rig = self.rig()
                rig.owner.rails_by_axis["x"].info.retract_speed = retract_speed
                observed = []
                move = rig.toolhead.manual_move

                def observe(coords, speed):
                    observed.append((list(coords), speed, rig.toolhead.accel))
                    move(coords, speed)

                rig.toolhead.manual_move = observe
                with self.stationary_seed(rig):
                    rig.calibrate("X", save=0, START_THRESHOLD=2.25)
                self.assertTrue(observed)
                self.assertTrue(all(speed <= 40. and accel == 1000. for _, speed, accel in observed))
                self.assertEqual(observed[-1], ([5., None, None], min(retract_speed, 40.), 1000.))
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertIsNone(rig.owner.saved_accel)

    def test_gentle_save_preserves_exact_threshold_qualified_by_all_eleven_homes(self):
        for axis, save in itertools.product("xy", (0, 1)):
            with self.subTest(axis=axis, save=save):
                rig = self.rig()
                with self.stationary_seed(rig):
                    rig.calibrate(axis.upper(), save=save, START_THRESHOLD=2.25)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25] * 11)
                saved = rig.configfile.saved
                self.assertEqual(float(saved[("accel_homing", axis + "_threshold")]), 2.25)
                self.assertEqual(float(saved[("accel_homing_model default", axis + "_threshold")]), 2.25)
                metadata = json.loads(saved[("accel_homing_model default", axis + "_metadata")])
                self.assertTrue(metadata["gentle_homing"])
                self.assertTrue(metadata["saved_threshold_tested"])
                self.assertEqual(metadata["tested_threshold_g"], 2.25)
                self.assertEqual(metadata["saved_threshold_g"], 2.25)
                self.assertEqual(metadata["threshold_margin_g"], 0.)
                self.assertEqual(metadata["homing_speed_mm_s"], 40.)
                self.assertEqual(metadata["homing_accel_mm_s2"], 1000.)
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), save)
                self.assertEqual(rig.toolhead.position[2:], [40., 6.])


if __name__ == "__main__":
    unittest.main()
