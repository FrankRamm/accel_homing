"""Source-only calibration-limit regressions; execution requires operator approval."""
import itertools
import json
import math
import tempfile
import unittest
from unittest import mock

from harness import CommandError, ConfigError, plugin
from test_beta_calibration import BetaRig
from test_calibration_auto_return import machine
from test_retract_noise import NoiseRig
from test_workflows import GCmd, Rig, workflow_module


class CalibrationRecoveryLimitTests(unittest.TestCase):
    def test_twelve_measured_returns_reach_four_g_without_an_attempt_limit(self):
        for axis, positive, cartesian in itertools.product("xy", (False, True), (False, True)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian):
                rig = machine(axis, positive, cartesian)
                rig.home_results.extend([{"travel_mm": 4.}] * 12 + [{}] * 11)
                rig.calibrate(axis.upper(), save=0)
                result = rig.workflow.last_result
                self.assertEqual([item["threshold"] for item in rig.home_calls],
                                 [1. + .25 * number for number in range(12)] + [4.] * 11)
                self.assertEqual(result["tested_threshold_g"], 4.)
                self.assertEqual(result["threshold_g"], 4.5)
                self.assertEqual(result["successes"], 10)
                self.assertEqual(result["restarts"], 12)
                self.assertEqual(result["attempts"], 23)
                self.assertIsNone(result["attempt_limit"])
                self.assertIsNone(result["attempts_remaining"])
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
        self.assertIsNone(workflow.get_status(0)["active"]["attempts_remaining"])


class CalibrationUnlimitedAttemptsTests(unittest.TestCase):
    def rig(self, beta, options=None):
        return (BetaRig if beta else Rig)(options)

    def run_calibration(self, rig, beta, axis):
        if beta:
            rig.beta(axis.upper(), START_THRESHOLD=1.)
        else:
            rig.calibrate(axis.upper(), save=0)

    def test_discovery_and_all_ten_checks_complete_beyond_forty_homes(self):
        for beta, axis, early_count in itertools.product((False, True), "xy", (34, 39, 41)):
            with self.subTest(beta=beta, axis=axis, early_count=early_count):
                rig = self.rig(beta, {"calibration_max_attempts": 12})
                # Includes the reported 9.5 g candidate, discovery on home 40,
                # and early-return/threshold progression after home 40.
                rig.home_results.extend([{"travel_mm": 0.}] * early_count + [{}] * 12)
                states = []
                rig.on_home = lambda _axis: states.append(rig.workflow.get_status(0)["active"])
                self.run_calibration(rig, beta, axis)
                result = rig.workflow.last_result
                candidate = 1. + .25 * early_count
                self.assertEqual(result["result"], "beta_proposed" if beta else "calibrated")
                self.assertEqual(result["successes"], 10)
                self.assertEqual(result["attempts"], early_count + 11)
                self.assertIsNone(result["attempt_limit"])
                self.assertIsNone(result["attempts_remaining"])
                self.assertEqual(result["candidate_attempts_required"], 11)
                self.assertEqual([call["attempt"] for call in rig.home_calls],
                                 list(range(1, early_count + 12)))
                self.assertEqual([call["threshold"] for call in rig.home_calls[-11:]], [candidate] * 11)
                self.assertFalse(rig.home_calls[-11]["candidate_qualified"])
                self.assertEqual([call["successes"] for call in rig.home_calls[-11:]],
                                 [0, 0] + list(range(1, 10)))
                self.assertEqual(result["restarts"], early_count)
                self.assertEqual(result["recovery"]["count"], early_count)
                self.assertTrue(all(state["attempt_limit"] is None and
                                    state["attempts_remaining"] is None for state in states))
                self.assertGreater(states[-1]["attempts"], 40)
                self.assertEqual(len(rig.home_results), 1)
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                if beta:
                    self.assertEqual(result["proposed_threshold_g"], candidate)
                    self.assertEqual(rig.configfile.saved, {})
                    trials = result["beta_diagnostics"]["trials"]
                    self.assertEqual([trial["workflow_attempt"] for trial in trials[-11:]],
                                     list(range(early_count + 1, early_count + 12)))
                else:
                    self.assertEqual(result["tested_threshold_g"], candidate)
                    self.assertEqual(result["threshold_g"], candidate + .5)
                    self.assertIn(("accel_homing_model default", axis + "_threshold"), rig.configfile.saved)
                json.dumps(result, allow_nan=False)

    def test_deprecated_maximum_is_consumed_without_numeric_validation(self):
        for beta, legacy in itertools.product((False, True),
                                              (0, 1, 12, 40, 1000, -1, "obsolete", "nan", "inf")):
            with self.subTest(beta=beta, legacy=legacy), \
                    mock.patch.object(workflow_module.logging, "warning") as warning:
                rig = self.rig(beta, {"calibration_max_attempts": legacy})
                warning.assert_called_once()
                self.assertIn("calibration_max_attempts is deprecated and ignored", warning.call_args[0][0])
                self.assertFalse(hasattr(rig.workflow, "max_attempts"))
                self.run_calibration(rig, beta, "x")
                self.assertEqual(len(rig.home_calls), 11)
                self.assertEqual(rig.workflow.last_result["successes"], 10)

    def test_rejected_validation_restarts_discovery_and_ten_checks_past_forty(self):
        for beta, axis in itertools.product((False, True), "xy"):
            with self.subTest(beta=beta, axis=axis):
                rig = self.rig(beta)
                expected = rig.workflow._geometry(axis)[1]
                rig.home_results.extend([{"travel_mm": 0.}] * 27 + [{}] * 6 +
                                        [{"travel_mm": expected + 2.001}] + [{}] * 11)
                self.run_calibration(rig, beta, axis)
                result = rig.workflow.last_result
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [1. + .25 * number for number in range(27)] + [7.75] * 7 + [8.] * 11)
                self.assertFalse(rig.home_calls[34]["candidate_qualified"])
                self.assertEqual([call["successes"] for call in rig.home_calls[34:]],
                                 [0, 0] + list(range(1, 10)))
                self.assertEqual(result["attempts"], 45)
                self.assertEqual(result["successes"], 10)
                self.assertEqual(result["restarts"], 28)
                self.assertEqual(result["recovery"]["count"], 28 if beta else 27)

    def test_cancel_disconnect_shutdown_and_errors_still_stop_after_forty(self):
        for beta, ending in itertools.product((False, True), ("cancel", "disconnect", "shutdown", "error")):
            with self.subTest(beta=beta, ending=ending):
                rig = self.rig(beta)
                rig.home_results.extend([{"travel_mm": 0.}] * 40 + [{}] * 12)
                moves_at_stop = []

                def stop(_axis):
                    if len(rig.home_calls) != 42:
                        return
                    moves_at_stop.append(len(rig.toolhead.moves))
                    if ending == "cancel":
                        rig.workflow.cmd_CANCEL(GCmd())
                    elif ending == "error":
                        raise CommandError("synthetic sensor error after forty homes")
                    else:
                        getattr(rig.workflow, "_" + ending)()

                rig.on_home = stop
                with self.assertRaises(CommandError):
                    self.run_calibration(rig, beta, "x")
                result = rig.workflow.last_result
                self.assertIn(result["result"], ("cancelled", "aborted"))
                self.assertEqual(result["attempts"], 42)
                self.assertEqual(result["candidate_g"], 11.)
                self.assertEqual(result["restarts"], 40)
                self.assertIsNone(result["attempt_limit"])
                self.assertIsNone(result["attempts_remaining"])
                self.assertEqual(len(rig.home_calls), 42)
                self.assertEqual(len(rig.toolhead.moves), moves_at_stop[0])
                self.assertIsNone(rig.workflow.session)
                self.assertFalse(rig.capture_active)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                if beta:
                    self.assertEqual(result["beta_home_history"], result["beta_diagnostics"]["home_history"])
                    self.assertEqual(result["beta_home_history"]["total_homes"], 42)


class CalibrationThresholdLimitTests(unittest.TestCase):
    def test_configured_and_command_starts_above_old_maxima_save_and_reload(self):
        for axis, configured, (threshold, sensor_range) in itertools.product(
                "xy", (False, True), ((12.3, 16.), (20., 32.))):
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
                self.assertEqual(reloaded.workflow.model.thresholds, {axis: threshold + .5})
                self.assertEqual(reloaded.owner.threshold_g[axis], threshold + .5)
                plugin.validate_threshold(threshold + .5, axis + "_threshold", ConfigError, sensor_range)
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
            self.assertEqual(rig.workflow.last_result["tested_threshold_g"], 15.75)
            self.assertEqual(rig.workflow.last_result["threshold_g"], 16.25)
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
                self.assertEqual(rig.workflow.last_result["tested_threshold_g"], 1.25)
                self.assertEqual(rig.workflow.last_result["threshold_g"], 1.75)

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
        self.assertEqual(rig.workflow.last_result["tested_threshold_g"], 100.)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 100.5)
        options = {key: value for (_section, key), value in rig.configfile.saved.items()}
        reloaded = Rig(sections={"printer": {"kinematics": "corexy"},
                                 "accel_homing_model default": options})
        reloaded.owner.full_scale_g = None
        reloaded.workflow._validate_model_connection()
        self.assertEqual(reloaded.workflow.model.thresholds, {"x": 100.5})
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
