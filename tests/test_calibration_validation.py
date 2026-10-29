"""Source-only calibration validation regressions; execution requires operator approval."""
import itertools
import json
import unittest
from unittest import mock

from harness import CommandError
from test_calibration_auto_return import machine
from test_workflows import GCmd, Rig


class ImmediateValidationTests(unittest.TestCase):
    def test_first_over_limit_check_restarts_discovery_and_all_ten_trials(self):
        for axis, failed_check, offset in itertools.product("xy", (2, 3, 8), (-2.0001, 2.0001)):
            with self.subTest(axis=axis, failed_check=failed_check, offset=offset):
                rig = Rig()
                expected = rig.workflow._geometry(axis)[1]
                rejected_count = failed_check + 1  # Discovery plus the failed validation prefix.
                rig.home_results.extend([{}] + [{"travel_mm": expected}] * (failed_check - 1) +
                                        [{"travel_mm": expected + offset}] + [{}] * 11)
                saved = rig.workflow._save_model
                accepted_attempts = []

                def saving():
                    accepted_attempts.extend(record["workflow_attempt"]
                                             for record in rig.workflow.session["validation_records"])
                    saved()

                with mock.patch.object(rig.workflow, "_save_model", side_effect=saving):
                    rig.calibrate(axis.upper(), save=0)
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [1.] * rejected_count + [1.25] * 11)
                restarted = rig.home_calls[rejected_count]
                self.assertFalse(restarted["candidate_qualified"])
                self.assertEqual(restarted["successes"], 0)
                self.assertEqual(accepted_attempts, list(range(rejected_count + 2, rejected_count + 12)))
                self.assertEqual(rig.workflow.last_result["successes"], 10)
                self.assertEqual(rig.workflow.last_result["restarts"], 1)
                self.assertTrue(any("after %d checks" % failed_check in line for line in rig.output))

    def test_immediate_rejection_respects_sensor_headroom_and_attempts_before_return(self):
        for ceiling in ("sensor-headroom", "attempts"):
            with self.subTest(ceiling=ceiling):
                rig = Rig({"calibration_max_attempts": 12} if ceiling == "attempts" else {})
                if ceiling == "sensor-headroom":
                    rig.owner.full_scale_g = 1.25
                rig.home_results.extend([{}, {"travel_mm": 150.}, {"travel_mm": 152.0001}])
                with self.assertRaisesRegex(CommandError, "headroom|attempt budget"):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 3)
                self.assertEqual(len(rig.toolhead.moves), 2)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1.] * 3)
                self.assertEqual(rig.configfile.saved, {})


class CalibrationTravelPlausibilityTests(unittest.TestCase):
    def test_discovery_and_validation_use_inclusive_geometry_limits_and_exact_travel(self):
        for axis, positive, tolerance, phase in itertools.product(
                "xy", (False, True), (.05, .2, .4), ("discovery", "validation")):
            with self.subTest(axis=axis, positive=positive, tolerance=tolerance, phase=phase):
                rig = machine(axis, positive, options={"calibration_travel_tolerance": tolerance})
                workflow = rig.workflow
                workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", axis)
                workflow.session["state"] = "running"
                expected = workflow._geometry(axis)[1]
                low, high = expected * (1. - tolerance), expected * (1. + tolerance)
                for travel, quality in ((low, "good"), (high, "good"),
                                        (low - .0001, "early"), (high + .0001, "implausible")):
                    record = {"result": "impact", "impact_metric": "xyz_magnitude",
                              "travel_mm": expected, "travel_mm_exact": travel, "workflow_phase": phase}
                    self.assertEqual(workflow._quality(axis, record)[0], quality)

    def test_twenty_mm_recovery_rule_precedes_geometry_on_a_short_supported_axis(self):
        rig = Rig()
        rig.owner.rails_by_axis["x"].bounds = (0., 40.1)
        workflow = rig.workflow
        workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
        workflow.session["state"] = "running"
        for travel, quality in ((20., "early"), (20.0001, "good")):
            record = {"result": "impact", "impact_metric": "xyz_magnitude",
                      "travel_mm": 20., "travel_mm_exact": travel}
            self.assertEqual(workflow._quality("x", record)[0], quality)

    def test_implausible_hits_abort_without_motion_retry_or_releasing_other_axes(self):
        for axis, positive, cartesian, validation in itertools.product(
                "xy", (False, True), (False, True), (False, True)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian,
                              validation=validation):
                rig = machine(axis, positive, cartesian)
                other = 1 - "xy".index(axis)
                other_limits, z_limits = (5., 195.), (20., 480.)
                rig.toolhead.kin.limits[2] = z_limits
                initial = rig.toolhead.get_position()

                def preserve_other_home(_axis):
                    # Establish state after the required initial unhomed-XY check.
                    rig.toolhead.kin.limits[other] = other_limits

                rig.on_home = preserve_other_home
                prefix = [{}, {}] if validation else []
                expected = rig.workflow._geometry(axis)[1]
                rig.home_results.extend(prefix + [{"travel_mm": expected * 1.3}])
                with mock.patch.object(rig.workflow, "_return_early_calibration") as reverse, \
                        self.assertRaisesRegex(CommandError, "Manually recenter XY.*new ACCEL_HOMING_CALIBRATE"):
                    rig.calibrate(axis.upper(), save=0)
                reverse.assert_not_called()
                self.assertEqual(len(rig.home_calls), len(prefix) + 1)
                self.assertEqual(len(rig.toolhead.moves), len(prefix))
                self.assertTrue(all(call["threshold"] == 1. for call in rig.home_calls))
                self.assertEqual(rig.toolhead.kin.limits["xy".index(axis)], (1., -1.))
                self.assertEqual(rig.toolhead.kin.limits[other], other_limits)
                self.assertEqual(rig.toolhead.kin.limits[2], z_limits)
                self.assertEqual(rig.toolhead.position[other], initial[other])
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertEqual(rig.disabled, [])
                self.assertTrue(all(line.enabled for line in rig.lines.values()))
                self.assertEqual(rig.configfile.saved, {})
                self.assertIsNone(rig.workflow.session)

    def test_implausible_halt_marks_axis_only_cleanup_before_capture_or_stream_failure(self):
        for failure in ("capture-data", "capture-stop", "stream-stop", "post-home-wait"):
            with self.subTest(failure=failure):
                rig = Rig()
                other_limits, z_limits = (5., 195.), (20., 480.)
                rig.toolhead.kin.limits[2] = z_limits
                rig.on_home = lambda axis: rig.toolhead.kin.limits.__setitem__(1, other_limits)
                rig.home_results.append({"travel_mm": 196.})
                stop_capture, end_session, wait = (rig.owner.stop_capture, rig.owner._end_session,
                                                   rig.toolhead.wait_moves)
                injected = [False]

                def fail_once(original, *args, **kwargs):
                    result = original(*args, **kwargs)
                    if not injected[0] and rig.owner.last.get("x") is not None:
                        injected[0] = True
                        raise CommandError("synthetic cleanup fault")
                    return result

                if failure == "capture-data":
                    rig.bad_capture = {"samples": [], "dropped": 0}
                    patch = mock.patch.object(rig.owner, "stop_capture", wraps=stop_capture)
                elif failure == "capture-stop":
                    patch = mock.patch.object(rig.owner, "stop_capture", side_effect=lambda: fail_once(stop_capture))
                elif failure == "stream-stop":
                    patch = mock.patch.object(rig.owner, "_end_session", side_effect=lambda: fail_once(end_session))
                else:
                    patch = mock.patch.object(rig.toolhead, "wait_moves", side_effect=lambda: fail_once(wait))
                with patch, self.assertRaisesRegex(CommandError, "cleanup fault|empty or incomplete"):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.toolhead.kin.limits, [(1., -1.), other_limits, z_limits])
                self.assertEqual(rig.disabled, [])
                self.assertTrue(all(line.enabled for line in rig.lines.values()))
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(sum("Manually recenter XY" in line for line in rig.output), 1)

    def test_plausible_and_repeatable_calibration_still_does_not_prove_contact(self):
        rig = Rig()
        rig.home_results.extend([{"travel_mm": 145.}] + [{"travel_mm": 130.}] * 10)
        rig.calibrate(save=0)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertTrue(metadata["travel_plausibility_required"])
        self.assertFalse(metadata["physical_position_verified"])
        self.assertEqual(metadata["recenter_acceleration_policy"], "restored_normal_limit")
        self.assertEqual(metadata["travel_tolerance_fraction"], .2)
        self.assertEqual(metadata["repeatability_spread_mm"], 0.)


class RecenterAccelerationGuardTests(unittest.TestCase):
    def test_active_homing_or_unrestored_cap_refuses_return_without_changing_acceleration(self):
        for mode, fault in itertools.product(("calibration", "accuracy"),
                                              ("homing", "attempt", "saved-accel")):
            with self.subTest(mode=mode, fault=fault):
                rig = Rig()
                workflow = rig.workflow
                workflow._begin(GCmd(SAVE_CONFIG=0), mode, "x")
                workflow.session["state"] = "running"
                rig.toolhead.kin.limits[0] = (0., 300.)
                rig.toolhead.accel = 5000.
                if fault == "homing":
                    rig.owner.homing_active = True
                elif fault == "attempt":
                    rig.owner.attempt = object()
                else:
                    rig.owner.saved_accel = 60000.
                with mock.patch.object(rig.owner, "_limit_accel") as limit, \
                        mock.patch.object(rig.owner, "_restore_accel") as restore, \
                        self.assertRaisesRegex(CommandError, "completed homing with restored normal acceleration"):
                    workflow._recenter("x")
                limit.assert_not_called()
                restore.assert_not_called()
                self.assertEqual(rig.toolhead.accel, 5000.)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.toolhead.dwells, [])
