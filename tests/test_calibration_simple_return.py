"""Source-only simple-return regressions; execution requires operator approval."""
import contextlib
import itertools
import json
import math
import unittest
from unittest import mock

from harness import CommandError
from test_calibration_auto_return import intercept_return, machine
from test_workflows import GCmd, Rig


class SimpleRecordedReturnTests(unittest.TestCase):
    def test_two_mm_early_halt_returns_two_mm_before_step_and_fresh_discovery(self):
        for axis, positive, cartesian in itertools.product("xy", (False, True), (False, True)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian):
                rig = machine(axis, positive, cartesian)
                sign, index = (1 if positive else -1), "xy".index(axis)
                rig.home_results.extend([{"travel_mm": 2., "halt_travel_mm_signed": sign * 2.}] + [{}] * 11)
                before, move = [], rig.toolhead.manual_move

                def moving(coordinates, speed):
                    if not before:
                        before.append(rig.toolhead.get_position())
                        self.assertEqual(rig.workflow.session["candidate"], 1.)
                        self.assertEqual(rig.workflow.session["recovery"]["count"], 0)
                        self.assertAlmostEqual(coordinates[index] - before[0][index], -sign * 2.)
                        self.assertTrue(all(value is None for i, value in enumerate(coordinates) if i != index))
                    return move(coordinates, speed)

                with mock.patch.object(rig.toolhead, "manual_move", side_effect=moving):
                    rig.calibrate(axis.upper(), save=0)
                self.assertEqual([item["threshold"] for item in rig.home_calls], [1.] + [1.25] * 11)
                self.assertFalse(rig.home_calls[1]["candidate_qualified"])
                self.assertEqual(rig.home_calls[1]["successes"], 0)
                self.assertEqual(rig.workflow.last_result["recovery"], {
                    "count": 1, "last_return_mm_signed": -sign * 2.,
                    "accounting": "completed_commanded_returns", "physical_position_verified": False})

    def test_short_axis_and_more_than_two_mm_post_trigger_travel_have_no_extra_envelope(self):
        rig = machine("x")
        rig.owner.rails_by_axis["x"].bounds = (0., 42.)
        rig.home_results.extend([{"travel_mm": 20., "halt_travel_mm_signed": -25.}] + [{}] * 11)
        rig.calibrate(save=0)
        self.assertEqual(rig.toolhead.moves[0], ([20., None, None], 50.))
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
        self.assertEqual(rig.workflow.last_result["recovery"]["last_return_mm_signed"], 25.)

    def test_return_reads_current_commanded_coordinate_without_arrival_or_motor_verification(self):
        rig = Rig()
        workflow = rig.workflow
        workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
        session = workflow.session
        session["state"] = "running"
        rig.home_results.append({"travel_mm": 2., "halt_travel_mm_signed": -5.})
        record, _capture = workflow._home("x", 1.)
        rig.toolhead.position[0] = 100.
        with contextlib.ExitStack() as stack:
            position = stack.enter_context(mock.patch.object(
                rig.toolhead, "get_position", wraps=rig.toolhead.get_position))
            move = stack.enter_context(mock.patch.object(rig.toolhead, "manual_move"))
            stack.enter_context(mock.patch.object(
                rig.toolhead, "check_busy", side_effect=AssertionError("extra idle-queue verification")))
            for stepper in rig.toolhead.kin.steppers:
                stack.enter_context(mock.patch.object(
                    stepper, "get_mcu_position", side_effect=AssertionError("MCU return verification")))
            for line in rig.lines.values():
                stack.enter_context(mock.patch.object(
                    line, "is_motor_enabled", side_effect=AssertionError("enable-line return verification")))
            workflow._return_early_calibration("x", record)
            move.assert_called_once_with([105., None, None], 50.)
            position.assert_called_once_with()
        # The accepted firmware command is sufficient; no post-return position
        # or motor-counter equality is used to claim physical arrival.
        self.assertEqual(rig.toolhead.position[0], 100.)
        self.assertEqual(session["recovery"], {"count": 1, "last_return_mm_signed": 5.})
        workflow._invalidate_calibration_axis(session)
        self.assertEqual(workflow._next_calibration_candidate("early trigger"), 1.25)

    def test_stock_return_wait_or_settle_errors_never_increase_threshold_or_count(self):
        for stage in ("move", "wait", "settle", "restore"):
            with self.subTest(stage=stage):
                rig = Rig()
                rig.home_results.append({"travel_mm": 2.})
                target, method = {
                    "move": (rig.toolhead, "manual_move"), "wait": (rig.toolhead, "wait_moves"),
                    "settle": (rig.toolhead, "dwell"), "restore": (rig.owner, "_restore_accel")}[stage]
                original, armed = getattr(target, method), [False]

                def failing(*args, **kwargs):
                    value = original(*args, **kwargs)
                    if armed[0]:
                        armed[0] = False
                        raise CommandError("ordinary firmware return failure")
                    return value

                def activate(_rig, _axis, _record):
                    armed[0] = True

                with intercept_return(rig, activate), mock.patch.object(target, method, side_effect=failing), \
                        self.assertRaisesRegex(CommandError, "ordinary firmware return failure"):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.workflow.last_result["candidate_g"], 1.)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 0)
                self.assertIsNone(rig.workflow.last_result["recovery"]["last_return_mm_signed"])
                self.assertEqual(rig.configfile.saved, {})

    def test_nonfinite_current_coordinate_or_sensor_fault_aborts_before_return(self):
        for fault in ("coordinate", "sensor"):
            with self.subTest(fault=fault):
                rig = Rig()
                rig.home_results.append({"travel_mm": 2.})

                def change(current, _axis, _record):
                    if fault == "coordinate":
                        current.toolhead.position[0] = math.nan
                    else:
                        current.owner.sensor_fault = "stale_data"

                with intercept_return(rig, change), self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.workflow.last_result["candidate_g"], 1.)
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 0)

    def test_historical_accumulated_98_50625_mm_metadata_is_readable_and_does_not_gate_new_return(self):
        template = Rig()
        metadata = {**template.workflow._model_context("x"), "impact_metric": "xyz_magnitude",
                    "recovery": {"count": 12, "distance_mm": 98.50625, "distance_limit_mm": 98.325,
                                 "count_limit": None, "count_remaining": None}}
        options = {"x_threshold": 3., "x_successes": 10, "x_metadata": json.dumps(metadata)}
        rig = Rig(sections={"printer": {"kinematics": "corexy"},
                            "accel_homing_model default": options})
        rig.workflow._validate_model_connection()
        rig.home_results.extend([{"travel_mm": 2.}] + [{}] * 11)
        rig.calibrate(save=0, START_THRESHOLD=3.)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 3.25)
        self.assertEqual(rig.workflow.last_result["recovery"], {
            "count": 1, "last_return_mm_signed": 2.,
            "accounting": "completed_commanded_returns", "physical_position_verified": False})
        self.assertEqual(rig.workflow.model.metadata["x"]["recovery"]["distance_mm"], 98.50625)
