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
    def test_twelve_measured_returns_reach_four_g_within_default_attempt_budget(self):
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


class CalibrationAttemptBudgetTests(unittest.TestCase):
    def rig(self, beta, limit=40):
        return (BetaRig if beta else Rig)({"calibration_max_attempts": limit})

    def run_calibration(self, rig, beta, axis):
        if beta:
            rig.beta(axis.upper(), START_THRESHOLD=1.)
        else:
            rig.calibrate(axis.upper(), save=0)

    def track_moves_before_home(self, rig):
        counts = []
        rig.on_home = lambda _axis: counts.append(len(rig.toolhead.moves))
        return counts

    def assert_exhausted(self, rig, limit):
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "aborted")
        self.assertEqual(result["attempts"], limit)
        self.assertEqual(result["attempt_limit"], limit)
        self.assertEqual(result["attempts_remaining"], 0)
        # This describes a complete qualification window, not a retry reserve.
        self.assertEqual(result["candidate_attempts_required"], 11)
        self.assertEqual([call["attempt"] for call in rig.home_calls], list(range(1, limit + 1)))
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertNotIn("SAVE_CONFIG", rig.scripts)

    def test_all_six_remaining_homes_run_before_incomplete_validation_aborts_at_forty(self):
        for beta, axis in itertools.product((False, True), "xy"):
            with self.subTest(beta=beta, axis=axis):
                rig = self.rig(beta)
                # Reject a running window on home 34, exactly where the former
                # reserve guard reported six attempts left. Use all six for new
                # discovery and five checks, then abort without a 41st home.
                expected = rig.workflow._geometry(axis)[1]
                rig.home_results.extend([{"travel_mm": 0.}] * 27 + [{}] * 6 +
                                        [{"travel_mm": expected + 2.001}] + [{}] * 7)
                moves_before_home = self.track_moves_before_home(rig)
                with self.assertRaisesRegex(CommandError, "attempt budget exhausted"):
                    self.run_calibration(rig, beta, axis)
                self.assert_exhausted(rig, 40)
                result = rig.workflow.last_result
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [1. + .25 * number for number in range(27)] + [7.75] * 7 + [8.] * 6)
                self.assertFalse(rig.home_calls[34]["candidate_qualified"])
                self.assertEqual([call["successes"] for call in rig.home_calls[34:]], [0, 0, 1, 2, 3, 4])
                self.assertEqual(result["candidate_g"], 8.)
                self.assertEqual(result["restarts"], 28)
                self.assertEqual(result["recovery"]["count"], 28 if beta else 27)
                self.assertEqual(len(rig.home_results), 1)
                self.assertEqual(len(rig.toolhead.moves), moves_before_home[-1])
                json.dumps(result, allow_nan=False)

    def test_tenth_validation_at_the_configured_cap_can_complete(self):
        for beta, axis, limit in itertools.product((False, True), "xy", (12, 40)):
            with self.subTest(beta=beta, axis=axis, limit=limit):
                rig = self.rig(beta, limit)
                early_count = limit - 11
                rig.home_results.extend([{"travel_mm": 0.}] * early_count + [{}] * 12)
                moves_before_home = self.track_moves_before_home(rig)
                self.run_calibration(rig, beta, axis)
                result = rig.workflow.last_result
                candidate = 1. + .25 * early_count
                self.assertEqual(result["result"], "beta_proposed" if beta else "calibrated")
                self.assertEqual(result["successes"], 10)
                self.assertEqual(result["attempts"], limit)
                self.assertEqual(result["attempts_remaining"], 0)
                self.assertEqual(len(rig.home_calls), limit)
                self.assertEqual([call["threshold"] for call in rig.home_calls[-11:]], [candidate] * 11)
                self.assertEqual(result["restarts"], early_count)
                self.assertEqual(result["recovery"]["count"], early_count)
                # Successful qualification still performs the normal final
                # release; it does not recenter for an additional home.
                self.assertEqual(len(rig.toolhead.moves), moves_before_home[-1] + 1)
                self.assertEqual(len(rig.home_results), 1)
                self.assertIsNone(rig.workflow.session)
                if beta:
                    self.assertEqual(result["proposed_threshold_g"], candidate)
                    self.assertEqual(rig.configfile.saved, {})
                else:
                    self.assertEqual(result["tested_threshold_g"], candidate)
                    self.assertEqual(result["threshold_g"], candidate + .5)
                    self.assertIn(("accel_homing_model default", axis + "_threshold"), rig.configfile.saved)

    def test_final_discovery_does_not_recenter_without_another_attempt(self):
        for beta, axis in itertools.product((False, True), "xy"):
            with self.subTest(beta=beta, axis=axis):
                rig = self.rig(beta, 12)
                rig.home_results.extend([{"travel_mm": 0.}] * 11 + [{}, {}])
                moves_before_home = self.track_moves_before_home(rig)
                with self.assertRaisesRegex(CommandError, "attempt budget exhausted"):
                    self.run_calibration(rig, beta, axis)
                self.assert_exhausted(rig, 12)
                result = rig.workflow.last_result
                self.assertEqual(result["candidate_g"], 3.75)
                self.assertEqual(rig.home_calls[-1]["successes"], 0)
                self.assertEqual(result["restarts"], 11)
                self.assertEqual(result["recovery"]["count"], 11)
                self.assertFalse(rig.home_calls[-1]["candidate_qualified"])
                self.assertEqual(len(rig.toolhead.moves), moves_before_home[-1])
                self.assertEqual(len(rig.home_results), 1)

    def test_last_rejected_home_does_not_return_recenter_or_raise_threshold(self):
        for beta, axis, rejection in itertools.product((False, True), "xy", ("early", "spread")):
            with self.subTest(beta=beta, axis=axis, rejection=rejection):
                rig = self.rig(beta, 12)
                expected = rig.workflow._geometry(axis)[1]
                rejected = {"travel_mm": 4. if rejection == "early" else expected + 2.001}
                rig.home_results.extend([{"travel_mm": 0.}] + [{}] * 10 + [rejected, {}])
                moves_before_home = self.track_moves_before_home(rig)
                with self.assertRaisesRegex(CommandError, "attempt budget exhausted"):
                    self.run_calibration(rig, beta, axis)
                self.assert_exhausted(rig, 12)
                result = rig.workflow.last_result
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1.] + [1.25] * 11)
                self.assertEqual(result["candidate_g"], 1.25)
                self.assertEqual(rig.home_calls[-1]["successes"], 9)
                self.assertEqual(result["restarts"], 1)
                self.assertEqual(result["recovery"]["count"], 1)
                self.assertEqual(result["recovery"]["last_return_mm_signed"], 0.)
                self.assertEqual(len(rig.toolhead.moves), moves_before_home[-1])
                self.assertEqual(len(rig.home_results), 1)
                if beta:
                    last = result["beta_diagnostics"]["trials"][-1]
                    self.assertEqual(last["beta_decision"], "rejected")
                    self.assertNotIn("beta_rejected_return_mm_signed", last)


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
