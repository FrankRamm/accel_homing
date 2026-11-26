"""Calibration stops, saved-threshold cap and bounded workflow seeks; synthetic only."""
import itertools
import json
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, plugin
from test_accuracy_recovery import RecoveryRig
from test_accuracy_workflow import AccuracyRig
from test_workflows import GCmd, Rig


def metadata(rig, axis="x"):
    return json.loads(rig.configfile.saved[("accel_homing_model default", axis + "_metadata")])


def saved_sections(rig):
    options = {key: value for (_section, key), value in rig.configfile.saved.items()}
    return {"printer": {"kinematics": "corexy"}, "accel_homing_model default": options}


class RepeatedEarlyContactTests(unittest.TestCase):
    def test_three_matching_premature_contacts_stop_after_the_third_return(self):
        for axis, travels in itertools.product("xy", ((40., 40., 40.), (40., 41.5, 42.), (60., 59., 58.5))):
            with self.subTest(axis=axis, travels=travels):
                rig = Rig()
                rig.home_results.extend([{"travel_mm": travel} for travel in travels] + [{}] * 11)
                with self.assertRaisesRegex(CommandError, "hitting something repeatably at about") as cm:
                    rig.calibrate(axis.upper(), save=0)
                message = str(cm.exception)
                self.assertIn("Check that the toolhead was centred", message)
                self.assertIn("restart with a higher START_THRESHOLD", message)
                self.assertIn("at 1, 1.25, 1.5 g", message)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1., 1.25, 1.5])
                # Every premature contact, including the third, is reversed once; nothing else moves.
                self.assertEqual(len(rig.toolhead.moves), 3)
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "aborted")
                self.assertEqual(result["candidate_g"], 1.5)
                self.assertEqual(result["recovery"]["count"], 3)
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                # Like other early-hit aborts, only the selected axis is invalidated.
                self.assertEqual(rig.disabled, [])
                self.assertNotIn(axis, rig.toolhead.kin.get_status(0)["homed_axes"])

    def test_transients_non_matching_hits_and_good_discovery_keep_escalating(self):
        cases = {
            # At or below 20 mm: start-up transients reset the count.
            "transient": [40., 40., 8., 40., 40., 20.] * 3,
            # Beyond 20 mm but not within 2 mm of each other.
            "non_matching": [40., 45., 40., 45., 40., 45., 40.],
            # A good discovery resets the count; the validation hit starts a new one.
            "discovery": [40., 40., None, 40., 40.],
        }
        for name, travels in cases.items():
            with self.subTest(case=name):
                rig = Rig()
                rig.home_results.extend([{} if travel is None else {"travel_mm": travel} for travel in travels]
                                        + [{}] * 11)
                rig.calibrate(save=0)
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "calibrated")
                restarts = sum(travel is not None for travel in travels)
                self.assertEqual(result["restarts"], restarts)
                self.assertEqual(result["tested_threshold_g"], 1. + .25 * restarts)


class SpreadRejectionTests(unittest.TestCase):
    def test_third_spread_rejection_stops_with_time_at_homing_speed(self):
        for axis in "xy":
            with self.subTest(axis=axis):
                rig = Rig()
                expected = rig.workflow._geometry(axis)[1]
                rig.home_results.extend([{}, {"travel_mm": expected - 1.}, {"travel_mm": expected + 1.5}] * 4)
                with self.assertRaisesRegex(CommandError, "spread rejection 3 in this run") as cm:
                    rig.calibrate(axis.upper(), save=0)
                message = str(cm.exception)
                self.assertIn("2.5 mm is 25.0 ms of travel at 100 mm/s", message)
                self.assertIn("Lower homing_speed", message)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1.] * 3 + [1.25] * 3 + [1.5] * 3)
                self.assertEqual(rig.workflow.last_result["restarts"], 2)
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertTrue(any("commanded travel spread 2.5 mm (25.0 ms at 100 mm/s)" in line
                                    for line in rig.output))
                self.assertEqual(rig.configfile.saved, {})
                self.assertIsNone(rig.workflow.session)

    def test_two_spread_rejections_still_restart_and_calibrate(self):
        rig = Rig()
        expected = rig.workflow._geometry("x")[1]
        rig.home_results.extend([{}, {"travel_mm": expected}, {"travel_mm": expected + 2.5}] * 2 + [{}] * 11)
        rig.calibrate(save=0)
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "calibrated")
        self.assertEqual(result["restarts"], 2)
        self.assertEqual(result["tested_threshold_g"], 1.5)


class TravelExcessTests(unittest.TestCase):
    def test_validation_excess_is_recorded_summarized_and_explained(self):
        for excess, notice in ((.8, True), (.4, False), (-.3, False)):
            with self.subTest(excess=excess):
                rig = Rig()
                expected = rig.workflow._geometry("x")[1]
                rig.home_results.extend([{}] + [{"travel_mm": expected + excess}] * 10)
                rig.calibrate(save=0)
                records = rig.workflow.last_result
                self.assertEqual(records["result"], "calibrated")
                saved = metadata(rig)
                for key in ("min_travel_excess_mm", "median_travel_excess_mm", "max_travel_excess_mm"):
                    self.assertAlmostEqual(saved[key], excess, places=9)
                self.assertAlmostEqual(records["travel_excess_mm"]["median_mm"], excess, places=9)
                self.assertTrue(any("travel %+.3f mm beyond the midpoint-to-stop distance" % excess in line
                                    for line in rig.output))
                lines = [line for line in rig.output if "further than the configured midpoint-to-stop" in line]
                self.assertEqual(len(lines), 1 if notice else 0)
                if notice:
                    self.assertIn("about 0.80 mm", lines[0])
                    self.assertIn("lower homing_speed reduces it", lines[0])

    def test_accuracy_reports_excess_of_measured_homes(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            expected = rig.workflow._geometry("x")[1]
            rig.home_results.extend([{}, {"travel_mm": expected + .6}, {"travel_mm": expected + 1.}])
            rig.accuracy("X", samples=2)
            trials = rig.workflow.last_result["axis_results"]["x"]["trials"]
            self.assertEqual([round(record["travel_excess_mm"], 9) for record in trials], [.6, 1.])
            summary = rig.workflow.last_result["axis_results"]["x"]["travel_excess_mm"]
            self.assertAlmostEqual(summary["median_mm"], .8, places=9)
            self.assertTrue(any("travel +0.600/+0.800/+1.000 mm (min/median/max)" in line for line in rig.output))
            self.assertTrue(any("X measured homes travelled about 0.80 mm further" in line for line in rig.output))


class SavedThresholdCapTests(unittest.TestCase):
    def test_margin_is_reduced_to_eighty_percent_of_the_weakest_impact(self):
        for peaks, saved, tested in (
                ([1.3] * 11, 1.04, False),
                ([1.1] * 11, 1., True),
                ([1.2] + [7.] * 10, 1., True),
                ([7.] * 10 + [1.6], 1.28, False)):
            with self.subTest(peaks=peaks):
                rig = Rig()
                rig.home_results.extend([{"impact_peak_g": peak} for peak in peaks])
                rig.calibrate(save=0)
                result, saved_metadata = rig.workflow.last_result, metadata(rig)
                self.assertEqual(result["tested_threshold_g"], 1.)
                self.assertAlmostEqual(result["saved_threshold_g"], saved, places=12)
                self.assertAlmostEqual(float(rig.configfile.saved[("accel_homing_model default", "x_threshold")]),
                                       saved, places=12)
                for report in (result, saved_metadata):
                    self.assertTrue(report["threshold_margin_reduced"])
                    self.assertEqual(report["weakest_impact_peak_g"], min(peaks))
                    self.assertAlmostEqual(report["threshold_margin_g"], saved - 1., places=12)
                    self.assertIs(report["saved_threshold_tested"], tested)
                self.assertEqual(saved_metadata["impact_threshold_fraction"], .8)
                self.assertTrue(any("weakest qualifying impact peaked at %.6g g" % min(peaks) in line
                                    for line in rig.output))
                self.assertTrue(any(("is the tested one" if tested else "not tested by those homes") in line
                                    for line in rig.output))

    def test_strong_impacts_and_missing_peaks_keep_the_full_margin(self):
        rig = Rig()
        rig.calibrate(save=0)
        result = rig.workflow.last_result
        self.assertEqual(result["saved_threshold_g"], 1.5)
        self.assertEqual(result["threshold_margin_g"], .5)
        self.assertFalse(result["threshold_margin_reduced"])
        self.assertEqual(result["weakest_impact_peak_g"], 7.)
        self.assertFalse(any("weakest qualifying impact" in line for line in rig.output))
        workflow = rig.workflow
        self.assertEqual(workflow._calibration_saved_threshold(1., None), 1.5)
        self.assertEqual(workflow._calibration_saved_threshold(1., 1.3), 1.04)
        workflow.session = {"records": [{"impact_peak_g": 7.}, {"impact_peak_g": None}], "candidate_attempt": 1}
        self.assertIsNone(workflow._weakest_impact_peak([{"impact_peak_g": 7.}] * 9 + [{}]))
        self.assertIsNone(workflow._weakest_impact_peak([{"impact_peak_g": float("nan")}] * 10))
        workflow.session["records"][0]["impact_peak_g"] = "7"
        self.assertIsNone(workflow._weakest_impact_peak([{"impact_peak_g": 7.}] * 10))
        workflow.session = None

    def test_impacts_at_the_sensor_range_save_at_the_headroom_ceiling(self):
        rig = Rig()
        rig.home_results.extend([{"impact_peak_g": 16.}] * 11)
        rig.calibrate(save=0, START_THRESHOLD=12.5)
        self.assertEqual(rig.workflow.last_result["saved_threshold_g"], 12.8)
        self.assertAlmostEqual(rig.workflow.last_result["threshold_margin_g"], .3, places=12)

    def test_accuracy_recovery_still_raises_the_active_threshold(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = RecoveryRig({"charts_directory": directory, "calibration_step": .5, "x_threshold": 2.25})
            rig.owner.threshold_g["x"] = 2.25
            # The recovery homes measure weak impacts, so the margin is reduced.
            rig.home_results.extend([{"travel_mm": 51.175}, {"travel_mm": 8.}] + [{"impact_peak_g": 3.2}] * 11)
            rig.accuracy(axis="X")
            result = rig.workflow.last_result
            self.assertEqual(result["tested_threshold_g"], 3.)
            self.assertEqual(result["saved_threshold_g"], 3.)
            self.assertGreater(result["saved_threshold_g"], 2.25)
            self.assertEqual(float(rig.configfile.saved[(rig.config.get_name(), "x_threshold")]), 3.)


class UnknownRangeCandidateTests(unittest.TestCase):
    def test_unknown_range_refuses_a_next_candidate_above_sixteen_g(self):
        rig = Rig()
        rig.owner.full_scale_g = None
        rig.home_results.extend([{"travel_mm": 8.}] * 80)
        with self.assertRaisesRegex(CommandError, "16.25 g would exceed 16 g and the accelerometer range is unknown"):
            rig.calibrate(save=0, START_THRESHOLD=1.)
        self.assertEqual(rig.home_calls[-1]["threshold"], 16.)
        self.assertEqual(len(rig.home_calls), 61)
        self.assertEqual(len(rig.toolhead.moves), 60)
        self.assertEqual(rig.configfile.saved, {})

    def test_known_range_keeps_its_headroom_limit(self):
        rig = Rig()
        rig.owner.full_scale_g = 32.
        rig.home_results.extend([{"travel_mm": 8.}] * 70 + [{"impact_peak_g": 32.}] * 11)
        rig.calibrate(save=0, START_THRESHOLD=1.)
        self.assertEqual(rig.workflow.last_result["tested_threshold_g"], 18.5)
        self.assertEqual(rig.workflow.last_result["saved_threshold_g"], 19.)


class StagedOnlyMessageTests(unittest.TestCase):
    def test_staged_only_message_says_homing_keeps_the_previous_threshold(self):
        for axis in "xy":
            with self.subTest(axis=axis):
                rig = Rig()
                rig.calibrate(axis.upper(), save=0)
                self.assertTrue(any("Threshold and model staged only. Homing, including ACCEL_HOMING_TEST and G28, "
                                    "keeps using the previous %s threshold until you run SAVE_CONFIG and Klipper "
                                    "restarts" % axis.upper() in line for line in rig.output))
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})


class EffectiveMotionContextTests(unittest.TestCase):
    def reloaded(self, mutate=None):
        calibrated = Rig()
        calibrated.calibrate(save=0)
        saved = metadata(calibrated)
        self.assertEqual(saved["effective_homing_speed_mm_s"], 100.)
        self.assertEqual(saved["effective_homing_accel_mm_s2"], 5000.)
        sections = saved_sections(calibrated)
        if mutate is not None:
            mutate(saved)
            sections["accel_homing_model default"]["x_metadata"] = json.dumps(saved)
        rig = Rig(sections=sections)
        rig.workflow._validate_model_connection()
        self.assertEqual(rig.workflow.stale_model_axes, {})
        return rig

    def notices(self, rig):
        return [line for line in rig.output if "used for calibration" in line]

    def test_matching_limits_print_no_notice(self):
        rig = self.reloaded()
        rig.workflow.check_home_axes(["x"])
        self.assertEqual(self.notices(rig), [])

    def test_lower_speed_or_changed_acceleration_print_one_notice(self):
        for velocity, accel, speed_notice, accel_notice in (
                (80., 50000., True, False), (500., 3000., False, True), (60., 2000., True, True)):
            with self.subTest(velocity=velocity, accel=accel):
                rig = self.reloaded()
                rig.toolhead.get_max_velocity = lambda: (velocity, accel)
                rig.workflow.check_home_axes(["x"])
                notices = self.notices(rig)
                self.assertEqual(len(notices), 1)
                self.assertEqual("X homes at %g mm/s, below the 100 mm/s" % velocity in notices[0], speed_notice)
                self.assertEqual("instead of the 5000 mm/s² used for calibration" in notices[0], accel_notice)
                self.assertIn("Restore the toolhead limits or recalibrate this axis", notices[0])
                self.assertEqual(rig.toolhead.moves, [])

    def test_kalico_rail_acceleration_counts_as_the_effective_value(self):
        rig = self.reloaded()
        rig.owner.rails_by_axis["x"].info.accel = 2500.
        rig.workflow.check_home_axes(["x"])
        self.assertIn("with 2500 mm/s² acceleration", self.notices(rig)[0])

    def test_models_without_the_new_keys_are_not_stale_and_compare_only_speed(self):
        def legacy(saved):
            del saved["effective_homing_speed_mm_s"], saved["effective_homing_accel_mm_s2"]
        rig = self.reloaded(legacy)
        rig.toolhead.get_max_velocity = lambda: (500., 3000.)
        rig.workflow.check_home_axes(["x"])
        self.assertEqual(self.notices(rig), [])
        rig.toolhead.get_max_velocity = lambda: (90., 3000.)
        rig.workflow.check_home_axes(["x"])
        notices = self.notices(rig)
        self.assertEqual(len(notices), 1)
        self.assertIn("X homes at 90 mm/s, below the 100 mm/s used for calibration", notices[0])
        self.assertNotIn("acceleration", notices[0])

    def test_manual_threshold_prints_no_notice(self):
        calibrated = Rig()
        calibrated.calibrate(save=0)
        rig = Rig({"x_threshold": 8.}, sections=saved_sections(calibrated))
        rig.workflow._validate_model_connection()
        rig.toolhead.get_max_velocity = lambda: (50., 1000.)
        rig.workflow.check_home_axes(["x"])
        self.assertEqual(self.notices(rig), [])


class BoundedWorkflowSeekTests(unittest.TestCase):
    def dispatch(self, rig, axis, mode="calibration", home_rails=None):
        workflow, owner = rig.workflow, rig.owner
        workflow.session = {"mode": mode, "axis": axis}
        workflow.in_home, workflow.home_axis = True, axis
        owner.ordinary_home = None
        owner.kinematics_home = mock.Mock(side_effect=AssertionError("unbounded kinematics dispatch"))
        homing = types.SimpleNamespace(get_axes=lambda: ["xy".index(axis)],
                                       home_rails=home_rails or mock.Mock(return_value="native single rail"))
        try:
            return plugin.BeaconAccelHoming._home_with_retract(owner, homing), homing
        finally:
            workflow.session, workflow.in_home, workflow.home_axis = None, False, None

    def test_every_workflow_mode_seeks_the_accepted_band_plus_braking(self):
        modes = ("calibration", "accuracy", "troubleshoot", "test", "charts")
        for axis, positive, mode in itertools.product("xy", (False, True), modes):
            with self.subTest(axis=axis, positive=positive, mode=mode):
                rig = Rig()
                rail = rig.owner.rails_by_axis[axis]
                rail.info.positive_dir = positive
                rail.info.position_endstop = rail.bounds[1] if positive else rail.bounds[0]
                before = rig.toolhead.get_position()
                result, homing = self.dispatch(rig, axis, mode)
                self.assertEqual(result, "native single rail")
                rig.owner.kinematics_home.assert_not_called()
                rails, forcepos, target = homing.home_rails.call_args.args
                self.assertEqual(rails, [rail])
                center, expected = rig.workflow._geometry(axis)
                # 100 mm/s, min(50000, 5000) mm/s²: 1 mm of braking after the band edge.
                bound = expected * .2 + 1.
                index = "xy".index(axis)
                self.assertAlmostEqual(forcepos[index], center - bound if positive else center + bound, places=9)
                self.assertEqual(target[index], rail.info.position_endstop)
                self.assertAlmostEqual(abs(target[index] - forcepos[index]), expected * 1.2 + 1., places=9)
                self.assertTrue(all(value is None for n, value in enumerate(forcepos) if n != index))
                self.assertEqual(rig.toolhead.get_position(), before)
                self.assertIsNone(rig.owner.ordinary_home)

    def test_braking_uses_live_speed_limit_and_rail_acceleration(self):
        for velocity, rail_accel, tolerance, braking in ((50., None, .2, .25), (500., 2000., .2, 2.5),
                                                         (500., None, .4, 1.)):
            with self.subTest(velocity=velocity, rail_accel=rail_accel, tolerance=tolerance):
                rig = Rig({"calibration_travel_tolerance": tolerance})
                rig.toolhead.get_max_velocity = lambda: (velocity, 50000.)
                if rail_accel is not None:
                    rig.owner.rails_by_axis["x"].info.accel = rail_accel
                _result, homing = self.dispatch(rig, "x")
                _rails, forcepos, target = homing.home_rails.call_args.args
                expected = rig.workflow._geometry("x")[1]
                self.assertAlmostEqual(abs(target[0] - forcepos[0]), expected * (1 + tolerance) + braking, places=9)

    def test_missed_contact_explains_the_bounded_seek_and_keeps_the_native_error(self):
        rig = Rig()
        native = mock.Mock(side_effect=CommandError("No trigger on x after full movement"))
        with self.assertRaisesRegex(CommandError, "^No trigger on x after full movement$"):
            self.dispatch(rig, "x", home_rails=native)
        notices = [line for line in rig.output if "seek without a detected impact" in line]
        self.assertEqual(len(notices), 1)
        self.assertIn("X workflow home moved its full 181.0 mm seek", notices[0])
        self.assertIn("Klipper switches the motors off after this error", notices[0])
        other = Rig()
        with self.assertRaisesRegex(CommandError, "different failure"):
            self.dispatch(other, "x", home_rails=mock.Mock(side_effect=CommandError("different failure")))
        self.assertFalse(any("seek without a detected impact" in line for line in other.output))

    def test_other_axis_inside_a_workflow_home_keeps_stock_kinematics(self):
        rig = Rig()
        workflow, owner = rig.workflow, rig.owner
        workflow.session = {"mode": "calibration", "axis": "x"}
        workflow.in_home, workflow.home_axis = True, "x"
        owner.ordinary_home = None
        owner.kinematics_home = mock.Mock(return_value="stock other axis")
        homing = types.SimpleNamespace(get_axes=lambda: [2], home_rails=mock.Mock())
        try:
            self.assertEqual(plugin.BeaconAccelHoming._home_with_retract(owner, homing), "stock other axis")
        finally:
            workflow.session, workflow.in_home, workflow.home_axis = None, False, None
        owner.kinematics_home.assert_called_once_with(homing)
        homing.home_rails.assert_not_called()

    def test_ordinary_homing_keeps_stock_kinematics(self):
        rig = Rig()
        owner = rig.owner
        owner.ordinary_home = None
        owner.kinematics_home = mock.Mock(return_value="ordinary native result")
        homing = types.SimpleNamespace(get_axes=lambda: [0], home_rails=mock.Mock())
        self.assertEqual(plugin.BeaconAccelHoming._home_with_retract(owner, homing), "ordinary native result")
        owner.kinematics_home.assert_called_once_with(homing)
        homing.home_rails.assert_not_called()

    def test_missed_contact_aborts_calibration_like_other_native_errors(self):
        rig = Rig()
        rig.home_results.extend([{}, CommandError("No trigger on x after full movement")])
        with self.assertRaisesRegex(CommandError, "No trigger on x after full movement"):
            rig.calibrate(save=0)
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "aborted")
        self.assertIsNone(rig.workflow.session)
        self.assertFalse(rig.capture_active)
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "")


class GentleBrakingWindowTests(unittest.TestCase):
    def test_braking_that_consumes_the_centring_tolerance_is_rejected(self):
        for speed, accel, accepted in ((40., 400., False), (40., 533., False), (40., 534., True),
                                       (40., 1000., True), (30., 300., True), (30., 299., False)):
            with self.subTest(speed=speed, accel=accel):
                rig = Rig({"gentle_homing": True})
                rig.owner.homing_accel = accel
                rig.owner.rails_by_axis["x"].info.speed = speed
                check = lambda: rig.workflow._check_gentle_limits("x", CommandError)
                if accepted:
                    check()
                    continue
                with self.assertRaisesRegex(CommandError, "brakes over") as cm:
                    check()
                minimum = {40.: 534, 30.: 300}[speed]
                self.assertIn("homing_accel of at least %d mm/s²" % minimum, str(cm.exception))

    def test_braking_window_does_not_stop_klipper_from_starting(self):
        rig = Rig({"gentle_homing": True})
        rig.owner.homing_accel = 400.
        rig.owner.rails_by_axis["x"].info.speed = 40.
        rig.workflow._check_gentle_limits("x", rig.workflow.printer.config_error)
        with self.assertRaisesRegex(CommandError, "brakes over"):
            rig.workflow._check_gentle_limits("x", rig.workflow.printer.command_error)


if __name__ == "__main__":
    unittest.main()
