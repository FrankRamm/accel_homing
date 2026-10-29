"""Source-only calibration-limit regressions; execution requires operator approval."""
import itertools
import json
import math
import tempfile
import unittest
from unittest import mock

from harness import CommandError, ConfigError, plugin
from test_calibration_auto_return import machine
from test_retract_noise import NoiseRig
from test_workflows import GCmd, Rig, workflow_module


class CalibrationRecoveryLimitTests(unittest.TestCase):
    def test_twelve_measured_returns_reach_four_g_within_default_attempt_budget(self):
        for axis, positive, cartesian in itertools.product("xy", (False, True), (False, True)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian):
                rig = machine(axis, positive, cartesian)
                rig.home_results.extend([{"travel_mm": 4.}] * 12 + [{}] * 11)
                rig.calibrate(axis.upper(), save=0)
                result = rig.workflow.last_result
                self.assertEqual([item["threshold"] for item in rig.home_calls],
                                 [1. + .25 * number for number in range(12)] + [4.] * 11)
                self.assertEqual(result["threshold_g"], 4.)
                self.assertEqual(result["successes"], 10)
                self.assertEqual(result["restarts"], 12)
                self.assertEqual(result["attempts"], 23)
                self.assertEqual(result["attempt_limit"], 40)
                self.assertEqual(result["attempts_remaining"], 17)
                self.assertEqual(result["candidate_attempts_required"], 11)
                self.assertEqual(result["recovery"], {
                    "count": 12, "last_return_mm_signed": -4. if positive else 4.,
                    "accounting": "completed_commanded_returns", "physical_position_verified": False})
                self.assertFalse(rig.home_calls[12]["candidate_qualified"])
                self.assertEqual(rig.home_calls[12]["successes"], 0)
                json.dumps(result, allow_nan=False)

    def test_reported_eight_return_state_accepts_ninth_halt_then_increases_to_3_25_g(self):
        rig = machine("y")
        rail = rig.owner.rails_by_axis["y"]
        rail.bounds, rail.info.position_endstop = (0., 218.5), 0.
        workflow = rig.workflow
        workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "y")
        session = workflow.session
        # Reproduce the reported previous-return count without accumulating
        # or reconstructing the individual previous halted distances.
        session.update(state="running", candidate=3., attempts=8, restarts=8)
        session["recovery"]["count"] = 8
        # Trigger travel and the signed start-to-halt displacement are distinct.
        rig.home_results.append({"travel_mm": 13.062, "halt_travel_mm_signed": -13.83125})
        record, _capture = workflow._home("y", 3.)
        self.assertEqual(workflow._next_calibration_candidate("early impact", apply=False), 3.25)
        workflow._return_early_calibration("y", record)
        self.assertEqual(session["candidate"], 3.)
        self.assertEqual(session["recovery"]["count"], 9)
        self.assertEqual(session["recovery"]["last_return_mm_signed"], 13.83125)
        self.assertEqual(len(rig.toolhead.moves), 1)
        workflow._invalidate_calibration_axis(session)
        self.assertEqual(workflow._next_calibration_candidate("early impact"), 3.25)
        self.assertEqual(session["candidate"], 3.25)
        self.assertEqual(session["restarts"], 9)
        self.assertEqual(workflow.get_status(0)["active"]["attempts_remaining"], 31)

    def test_exactly_eleven_future_homes_are_admitted_but_ten_abort_before_next_return(self):
        for early_count in (1, 2):
            with self.subTest(early_count=early_count):
                rig = Rig({"calibration_max_attempts": 12})
                rig.home_results.extend([{"travel_mm": 4.}] * early_count + [{}] * 11)
                if early_count == 1:
                    rig.calibrate(save=0)
                    self.assertEqual(rig.workflow.last_result["result"], "calibrated")
                    self.assertEqual(rig.workflow.last_result["attempts"], 12)
                    self.assertEqual([item["threshold"] for item in rig.home_calls], [1.] + [1.25] * 11)
                else:
                    with self.assertRaisesRegex(CommandError, "10 attempts remain, 11 required"):
                        rig.calibrate(save=0)
                    self.assertEqual(len(rig.home_calls), 2)
                    self.assertEqual(len(rig.toolhead.moves), 1)
                    self.assertEqual(rig.workflow.last_result["attempts_remaining"], 10)
                    self.assertEqual(rig.configfile.saved, {})
                result = rig.workflow.last_result
                self.assertEqual(result["candidate_g"], 1.25)
                self.assertEqual(result["restarts"], 1)
                self.assertEqual(result["recovery"]["count"], 1)
                self.assertEqual(result["recovery"]["last_return_mm_signed"], 4.)
                self.assertEqual(result["candidate_attempts_required"], 11)

    def test_default_budget_can_finish_after_twenty_nine_zero_returns_and_blocks_thirtieth_return(self):
        for early_count in (29, 30):
            with self.subTest(early_count=early_count):
                rig = Rig()
                rig.home_results.extend([{"travel_mm": 0.}] * early_count + [{}] * 11)
                if early_count == 29:
                    rig.calibrate(save=0)
                    self.assertEqual(rig.workflow.last_result["threshold_g"], 8.25)
                    self.assertEqual(len(rig.home_calls), 40)
                else:
                    with self.assertRaisesRegex(CommandError, "10 attempts remain, 11 required"):
                        rig.calibrate(save=0)
                    self.assertEqual(len(rig.home_calls), 30)
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.workflow.last_result["candidate_g"], 8.25)
                    self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 29)
                self.assertEqual(rig.workflow.last_result["recovery"]["last_return_mm_signed"], 0.)


class CalibrationThresholdLimitTests(unittest.TestCase):
    def test_configured_and_command_starts_above_old_maxima_save_and_reload(self):
        for axis, configured, (threshold, sensor_range) in itertools.product(
                "xy", (False, True), ((12.5, 16.), (20., 32.))):
            with self.subTest(axis=axis, configured=configured, threshold=threshold):
                rig = Rig({"calibration_start_threshold": threshold} if configured else {})
                rig.owner.full_scale_g = sensor_range
                rig.calibrate(axis.upper(), save=0, **({} if configured else {"START_THRESHOLD": threshold}))
                self.assertEqual([item["threshold"] for item in rig.home_calls], [threshold] * 11)
                options = {key: value for (_section, key), value in rig.configfile.saved.items()}
                sections = {"printer": {"kinematics": "corexy"}, "accel_homing_model default": options}
                reloaded = Rig(sections=sections)
                reloaded.owner.full_scale_g = sensor_range
                reloaded.workflow._validate_model_connection()
                self.assertEqual(reloaded.workflow.model.thresholds, {axis: threshold})
                self.assertEqual(reloaded.owner.threshold_g[axis], threshold)
                plugin.validate_threshold(threshold, axis + "_threshold", ConfigError, sensor_range)
                json.dumps(reloaded.workflow.model.get_status(0), allow_nan=False)

    def test_noise_hint_and_fresh_xyz_seed_above_fifteen_use_only_sensor_headroom(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp, "calibration_max_threshold": 1.},
                           amplitudes=(13., 0., 0.))
            rig.owner.full_scale_g = 32.
            rig.workflow.cmd_NOISE(GCmd())
            recommendation = rig.workflow.last_result["recommendation"]
            self.assertEqual(recommendation["threshold_limit_g"], 25.6)
            self.assertEqual(recommendation["start_threshold_g"]["x"], 15.75)
            rig.calibrate(save=0)
            self.assertEqual(rig.workflow.last_result["threshold_g"], 15.75)
            metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            self.assertEqual(metadata["stationary_noise_seed"]["start_threshold_g"], 15.75)
            self.assertEqual(metadata["stationary_noise_seed"]["threshold_limit_g"], 25.6)

    def test_retired_maximum_is_consumed_ignored_and_absent_from_noise_context(self):
        for legacy in (0., .5, 12., 15., -1., "obsolete", "nan", "inf"):
            with self.subTest(legacy=legacy), mock.patch.object(workflow_module.logging, "warning") as warning:
                rig = Rig({"calibration_max_threshold": legacy})
                warning.assert_called_once()
                self.assertIn("deprecated and ignored", warning.call_args[0][0])
                self.assertFalse(hasattr(rig.workflow, "max_threshold"))
                self.assertNotIn("calibration_max_threshold", rig.workflow._noise_context())
                rig.home_results.append({"travel_mm": 4.})
                rig.calibrate(save=0)
                self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)

    def test_known_sensor_headroom_still_rejects_before_motion(self):
        rig = Rig()
        before = rig.effects()
        with self.assertRaisesRegex(CommandError, "sensor-range headroom.*80%"):
            rig.calibrate(save=0, START_THRESHOLD=12.800001)
        self.assertEqual(rig.effects(), before)
        self.assertIsNone(rig.workflow.session)

    def test_unknown_range_explicit_start_and_model_use_none_without_fixed_g_cap(self):
        rig = Rig()
        rig.owner.full_scale_g = None
        rig.calibrate(save=0, START_THRESHOLD=100.)
        self.assertIsNone(rig.workflow._threshold_limit())
        self.assertEqual(rig.workflow.last_result["threshold_g"], 100.)
        options = {key: value for (_section, key), value in rig.configfile.saved.items()}
        reloaded = Rig(sections={"printer": {"kinematics": "corexy"},
                                 "accel_homing_model default": options})
        reloaded.owner.full_scale_g = None
        reloaded.workflow._validate_model_connection()
        self.assertEqual(reloaded.workflow.model.thresholds, {"x": 100.})
        encoded = json.dumps(reloaded.workflow.model.get_status(0), allow_nan=False)
        self.assertNotIn("Infinity", encoded)

    def test_unrepresentable_increment_or_invalid_home_threshold_does_not_mutate_state(self):
        rig = Rig()
        rig.owner.full_scale_g = None
        workflow = rig.workflow
        workflow._begin(GCmd(SAVE_CONFIG=0, START_THRESHOLD=1e308), "calibration", "x")
        session = workflow.session
        session["state"] = "running"
        with self.assertRaisesRegex(CommandError, "finite and increase"):
            workflow._next_calibration_candidate("early impact")
        self.assertEqual(session["candidate"], 1e308)
        self.assertEqual(session["attempts"], 0)
        self.assertEqual(session["restarts"], 0)
        for threshold in (0., .499, math.nan, math.inf):
            with self.subTest(threshold=threshold), self.assertRaisesRegex(CommandError, "finite and at least"):
                workflow._home("x", threshold)
            self.assertEqual(session["attempts"], 0)
            self.assertEqual(session["recovery"]["count"], 0)
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.configfile.saved, {})
