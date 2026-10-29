"""Source-only premature-trigger regressions; execution requires operator approval."""
import itertools
import json
import unittest
from unittest import mock

from harness import CommandError
from test_calibration_auto_return import intercept_return, machine
from test_workflows import GCmd, Rig


class MidtravelRecoveryTests(unittest.TestCase):
    def test_51_175_mm_trigger_reverses_halt_then_discovers_at_2_5_before_ten_fresh_checks(self):
        for axis, positive, cartesian, validation, overshoot in itertools.product(
                "xy", (False, True), (False, True), (False, True), (0., 2., 5.)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian,
                              validation=validation, overshoot=overshoot):
                rig = machine(axis, positive, cartesian)
                rail = rig.owner.rails_by_axis[axis]
                rail.bounds = (0., 218.5)
                rail.info.position_endstop = rail.bounds[1 if positive else 0]
                index, sign = "xy".index(axis), 1 if positive else -1
                initial = rig.toolhead.get_position()
                prefix = [{}, {}] if validation else []
                halt = 51.175 + overshoot
                rig.home_results.extend(prefix + [{"travel_mm": 51.175,
                                                   "halt_travel_mm_signed": sign * halt}] + [{}] * 11)
                moves, returning, accepted = [], [], []
                move, save = rig.toolhead.manual_move, rig.workflow._save_model

                def moving(coordinates, speed):
                    moves.append((rig.toolhead.get_position(), list(coordinates)))
                    return move(coordinates, speed)

                def evidence(r, selected, record):
                    captured = r.workflow.session["early_halt"]
                    self.assertEqual(selected, axis)
                    self.assertIs(captured["source"], r.owner.last[axis])
                    self.assertEqual(captured["attempt"], len(prefix) + 1)
                    self.assertEqual(record["travel_mm_exact"], 51.175)
                    self.assertTrue(record["early_capture_valid"])
                    returning.append(record)

                def saving():
                    accepted.extend(record["workflow_attempt"]
                                    for record in rig.workflow.session["validation_records"])
                    return save()

                with intercept_return(rig, evidence), \
                        mock.patch.object(rig.toolhead, "manual_move", side_effect=moving), \
                        mock.patch.object(rig.toolhead, "set_position", side_effect=AssertionError("coordinate rebase")), \
                        mock.patch.object(rig.workflow, "_save_model", side_effect=saving):
                    rig.calibrate(axis.upper(), save=0, START_THRESHOLD=2.25, STEP=.25)
                before, coordinates = moves[len(prefix)]
                self.assertAlmostEqual(coordinates[index] - before[index], -sign * halt)
                self.assertEqual(coordinates[index], rail.info.position_endstop - sign * 51.175)
                self.assertTrue(all(value is None for i, value in enumerate(coordinates) if i != index))
                restarted = rig.home_calls[len(prefix) + 1]
                self.assertEqual(restarted["position"][index], coordinates[index])
                self.assertEqual(restarted["homed_axes"], "z")
                self.assertFalse(restarted["candidate_qualified"])
                self.assertEqual(restarted["successes"], 0)
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [2.25] * (len(prefix) + 1) + [2.5] * 11)
                self.assertEqual(accepted, list(range(len(prefix) + 3, len(prefix) + 13)))
                self.assertEqual(len(returning), 1)
                self.assertEqual(returning[0]["early_return_mm_signed"], -sign * halt)
                self.assertFalse(returning[0]["early_return_physical_position_verified"])
                self.assertEqual(rig.toolhead.position[1 - index], initial[1 - index])
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertEqual(rig.disabled, [])
                result = rig.workflow.last_result
                self.assertEqual(result["threshold_g"], 2.5)
                self.assertEqual(result["successes"], 10)
                self.assertEqual(result["recovery"]["count"], 1)
                self.assertEqual(result["recovery"]["last_return_mm_signed"], -sign * halt)
                self.assertFalse(result["recovery"]["physical_position_verified"])
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", axis + "_metadata")])
                self.assertEqual(metadata["recovery"], result["recovery"])

    def test_two_51_175_mm_returns_continue_without_the_old_98_325_mm_budget(self):
        for axis, positive in itertools.product("xy", (False, True)):
            with self.subTest(axis=axis, positive=positive):
                rig = machine(axis, positive)
                rail = rig.owner.rails_by_axis[axis]
                rail.bounds = (0., 218.5)
                rail.info.position_endstop = rail.bounds[1 if positive else 0]
                other, z_limits = 1 - "xy".index(axis), (20., 480.)
                other_limits = (5., 195.)
                rig.toolhead.kin.limits[2] = z_limits
                initial, home = rig.toolhead.get_position(), rig.workflow._home

                def home_with_other_axis_state(selected, threshold=None):
                    # Retain unrelated state while the selected axis retries.
                    rig.toolhead.kin.limits[other] = other_limits
                    return home(selected, threshold)

                rig.home_results.extend([{"travel_mm": 51.175}] * 2 + [{}] * 11)
                with mock.patch.object(rig.workflow, "_home", side_effect=home_with_other_axis_state):
                    rig.calibrate(axis.upper(), save=0, START_THRESHOLD=2.25, STEP=.25)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25, 2.5] + [2.75] * 11)
                self.assertEqual(rig.workflow.last_result["threshold_g"], 2.75)
                self.assertEqual(rig.workflow.last_result["restarts"], 2)
                self.assertEqual(rig.workflow.last_result["recovery"], {
                    "count": 2, "last_return_mm_signed": -51.175 if positive else 51.175,
                    "accounting": "completed_commanded_returns", "physical_position_verified": False})
                self.assertEqual(rig.toolhead.kin.limits[other], other_limits)
                self.assertEqual(rig.toolhead.kin.limits[2], z_limits)
                self.assertEqual(rig.toolhead.position[other], initial[other])
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertEqual(rig.disabled, [])
                self.assertIsNone(rig.workflow.session)

    def test_repeated_returns_have_no_accumulated_distance_limit(self):
        for extra in (0., .000001):
            with self.subTest(extra=extra):
                rig = machine("y")
                rig.home_results.extend([{"travel_mm": 45.},
                                         {"travel_mm": 45., "halt_travel_mm_signed": -45. - extra}] + [{}] * 11)
                rig.calibrate("Y", save=0)
                self.assertEqual(rig.workflow.last_result["threshold_g"], 1.5)
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 2)
                self.assertEqual(rig.workflow.last_result["recovery"]["last_return_mm_signed"], 45. + extra)

    def test_twelve_zero_halts_have_no_independent_return_count_limit(self):
        rig = Rig()
        rig.home_results.extend([{"travel_mm": 0.}] * 12 + [{}] * 11)
        rig.calibrate(save=0)
        self.assertEqual([call["threshold"] for call in rig.home_calls],
                         [1. + .25 * i for i in range(12)] + [4.] * 11)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 4.)
        self.assertEqual(rig.workflow.last_result["restarts"], 12)
        recovery = rig.workflow.last_result["recovery"]
        self.assertEqual(recovery["count"], 12)
        self.assertEqual(recovery["last_return_mm_signed"], 0.)
        self.assertEqual(rig.disabled, [])

    def test_completed_return_telemetry_survives_spread_candidate_reset(self):
        rig = Rig()
        rig.home_results.extend([{"travel_mm": 8.}, {"travel_mm": 51.175}, {},
                                 {"travel_mm": 150.}, {"travel_mm": 152.0001}, {"travel_mm": 80.}] + [{}] * 11)
        states = []
        rig.on_home = lambda axis: states.append(rig.workflow.get_status(0)["active"])
        rig.calibrate(save=0)
        self.assertEqual([call["threshold"] for call in rig.home_calls],
                         [1., 1.25, 1.5, 1.5, 1.5, 1.75] + [2.] * 11)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 2.)
        self.assertEqual(rig.workflow.last_result["restarts"], 4)
        for status in (states[2], states[5]):
            self.assertEqual(status["recovery"]["count"], 2)
            self.assertEqual(status["recovery"]["last_return_mm_signed"], 51.175)
        self.assertEqual(rig.workflow.last_result["recovery"]["count"], 3)
        self.assertEqual(rig.workflow.last_result["recovery"]["last_return_mm_signed"], 80.)
        self.assertFalse(states[6]["candidate_qualified"])
        self.assertEqual(states[6]["successes"], 0)

    def test_initial_return_telemetry_has_no_geometry_or_limit_fields(self):
        for positive, bounds, endstop in (
                (False, (10., 310.), 20.),
                (True, (10., 310.), 300.),
                (False, (0., 42.), 0.)):
            with self.subTest(positive=positive, bounds=bounds):
                rig = machine(positive=positive)
                rail = rig.owner.rails_by_axis["x"]
                rail.bounds, rail.info.position_endstop = bounds, endstop
                rig.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
                self.assertEqual(rig.workflow.get_status(0)["active"]["recovery"], {
                    "count": 0, "last_return_mm_signed": None,
                    "accounting": "completed_commanded_returns", "physical_position_verified": False})
                self.assertEqual(rig.toolhead.moves, [])

    def test_same_run_manual_continuation_retains_completed_return_count_without_a_budget(self):
        rig = Rig()
        workflow = rig.workflow
        workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
        workflow.session["state"] = "running"
        rig.home_results.append({"travel_mm": 51.175})
        record, _capture = workflow._home("x", 1.)
        workflow._return_early_calibration("x", record)
        workflow._invalidate_calibration_axis(workflow.session)
        workflow._next_calibration_candidate("synthetic retained wait after recovery")
        session, run_id = workflow.session, workflow.session["run_id"]
        workflow._wait_center("synthetic retained manual-centering wait")
        self.assertIs(workflow.session, session)
        self.assertEqual(workflow.get_status(0)["active"]["recovery"]["count"], 1)
        rig.home_results.extend([{"travel_mm": 90.}] + [{}] * 11)
        rig.calibrate(save=0)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.5)
        self.assertEqual(rig.workflow.last_result["restarts"], 2)
        self.assertEqual(rig.workflow.last_result["recovery"]["count"], 2)
        self.assertEqual(rig.workflow.last_result["recovery"]["last_return_mm_signed"], 90.)
        self.assertEqual(session["run_id"], run_id)

    def test_sensor_headroom_and_attempt_rejections_leave_candidate_and_return_count_unchanged(self):
        for fault in ("sensor-headroom", "attempts"):
            with self.subTest(fault=fault):
                rig = Rig()
                if fault == "sensor-headroom":
                    rig.owner.full_scale_g = 2.8125
                if fault == "attempts":
                    rig.workflow.max_attempts = 1
                rig.home_results.append({"travel_mm": 51.175, "halt_travel_mm_signed": -51.175})
                with self.assertRaises(CommandError):
                    rig.calibrate(save=0, START_THRESHOLD=2.25, STEP=.25)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.moves, [])
                result = rig.workflow.last_result
                self.assertEqual(result["candidate_g"], 2.25)
                self.assertEqual(result["restarts"], 0)
                self.assertEqual(result["recovery"]["count"], 0)
                self.assertIsNone(result["recovery"]["last_return_mm_signed"])
                self.assertEqual(rig.disabled, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_midtravel_cleanup_evidence_is_marked_before_capture_stream_or_wait_failures(self):
        for failure in ("capture-data", "capture-stop", "stream-stop", "post-home-wait"):
            with self.subTest(failure=failure):
                rig = Rig()
                other_limits, z_limits = (5., 195.), (20., 480.)
                rig.toolhead.kin.limits[2] = z_limits
                rig.on_home = lambda axis: rig.toolhead.kin.limits.__setitem__(1, other_limits)
                rig.home_results.append({"travel_mm": 51.175})
                target, method = {"capture-data": (rig.owner, "stop_capture"),
                                  "capture-stop": (rig.owner, "stop_capture"),
                                  "stream-stop": (rig.owner, "_end_session"),
                                  "post-home-wait": (rig.toolhead, "wait_moves")}[failure]
                original, checked = getattr(target, method), []

                def fail_after_halt(*args, **kwargs):
                    result = original(*args, **kwargs)
                    source = rig.owner.last.get("x")
                    if source is not None and not checked:
                        session = rig.workflow.session
                        self.assertTrue(session["early_return_cleanup"])
                        self.assertIs(session["early_halt"]["source"], source)
                        self.assertEqual(session["early_halt"]["halt_travel_mm_signed"], -51.175)
                        checked.append(True)
                        if failure == "capture-data":
                            result["samples"] = []
                        else:
                            raise CommandError("synthetic cleanup fault")
                    return result

                with mock.patch.object(target, method, side_effect=fail_after_halt), \
                        self.assertRaisesRegex(CommandError, "cleanup fault|empty or incomplete"):
                    rig.calibrate(save=0)
                self.assertEqual(checked, [True])
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.toolhead.kin.limits, [(1., -1.), other_limits, z_limits])
                self.assertEqual(rig.disabled, [])
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 0)
                self.assertEqual(rig.configfile.saved, {})
