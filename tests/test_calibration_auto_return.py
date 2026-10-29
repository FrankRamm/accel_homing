"""Source-only early-impact recovery regressions; execution requires operator approval."""
import itertools
import math
import unittest
from unittest import mock

from harness import CommandError
from test_workflows import GCmd, Kin, Rig


class CartesianKin(Kin):
    def __init__(self):
        super().__init__()
        self.steppers[0].axes, self.steppers[1].axes = "x", "y"

    def calc_position(self, positions):
        return [positions["stepper_x"], positions["stepper_y"], positions["stepper_z"]]


def machine(axis="x", positive=False, cartesian=False, options=None):
    rig = Rig(options)
    if cartesian:
        rig.toolhead.kin = CartesianKin()
    rail = rig.owner.rails_by_axis[axis]
    rail.info.positive_dir = positive
    rail.info.position_endstop = rail.get_range()[1 if positive else 0]
    return rig


def intercept_return(rig, change):
    returning = rig.workflow._return_early_calibration

    def altered(axis, record):
        change(rig, axis, record)
        return returning(axis, record)

    return mock.patch.object(rig.workflow, "_return_early_calibration", side_effect=altered)


class AutomaticEarlyReturnTests(unittest.TestCase):
    def test_xy_both_directions_corexy_and_cartesian_reverse_halt_including_overshoot(self):
        for axis, positive, cartesian in itertools.product("xy", (False, True), (False, True)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian):
                rig = machine(axis, positive, cartesian)
                initial = rig.toolhead.get_position()
                other = 1 - "xy".index(axis)
                other_limit, z_limit = rig.toolhead.kin.limits[other:other + 1], rig.toolhead.kin.limits[2]
                sign = 1 if positive else -1
                rig.home_results.append({"travel_mm": 8., "halt_travel_mm_signed": sign * 8.375})
                movements = []
                move = rig.toolhead.manual_move

                def moving(coordinates, speed):
                    movements.append((rig.toolhead.get_position(), list(coordinates), speed))
                    return move(coordinates, speed)

                with mock.patch.object(rig.toolhead, "manual_move", side_effect=moving), \
                        mock.patch.object(rig.toolhead, "set_position", side_effect=AssertionError("coordinate rebase")):
                    rig.calibrate(axis.upper(), save=0)
                before, coordinates, speed = movements[0]
                index = "xy".index(axis)
                self.assertAlmostEqual(coordinates[index] - before[index], -sign * 8.375)
                self.assertEqual(coordinates[index], rig.owner.rails_by_axis[axis].info.position_endstop - sign * 8.)
                self.assertTrue(all(value is None for i, value in enumerate(coordinates) if i != index))
                self.assertEqual(speed, 50.)
                self.assertEqual(rig.home_calls[1]["position"][index], coordinates[index])
                self.assertEqual(rig.home_calls[1]["homed_axes"], "z")
                self.assertEqual(rig.home_calls[1]["threshold"], 1.25)
                self.assertEqual(rig.toolhead.position[other], initial[other])
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertEqual(rig.toolhead.kin.limits[other:other + 1], other_limit)
                self.assertEqual(rig.toolhead.kin.limits[2], z_limit)
                self.assertEqual(rig.disabled, [])
                self.assertEqual(len(rig.home_calls), 12)

    def test_multiple_early_validation_hits_restart_discovery_and_all_ten_checks(self):
        rig = Rig()
        rig.home_results.extend([{}, {}, {"travel_mm": 8.}, {"travel_mm": 10.}] + [{}] * 11)
        rig.calibrate(save=0, STEP=.25)
        self.assertEqual([call["threshold"] for call in rig.home_calls], [1.] * 3 + [1.25] + [1.5] * 11)
        for offset in (3, 4):
            self.assertFalse(rig.home_calls[offset]["candidate_qualified"])
            self.assertEqual(rig.home_calls[offset]["successes"], 0)
            self.assertEqual(rig.home_calls[offset]["homed_axes"], "z")
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.5)
        self.assertEqual(rig.workflow.last_result["successes"], 10)
        self.assertEqual(rig.workflow.last_result["repeatability_limit_mm"], 2.)

    def test_zero_halt_is_noop_with_settle_and_same_command_retry(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 0.})
        rig.calibrate(save=0)
        self.assertEqual(rig.toolhead.moves[0][0], [150., None, None])
        self.assertEqual(rig.toolhead.dwells[0], rig.workflow.calibration_settle_time)
        self.assertEqual(rig.home_calls[1]["threshold"], 1.25)
        self.assertEqual(rig.home_calls[1]["homed_axes"], "z")

    def test_bad_signed_displacements_abort_without_return_or_new_home(self):
        for fields in ({"halt_travel_mm_signed": None}, {"halt_travel_mm_signed": math.nan},
                       {"halt_travel_mm_signed": math.inf}, {"halt_travel_mm_signed": True},
                       {"pass": 2}):
            with self.subTest(fields=fields):
                rig = Rig()
                rig.home_results.append({"travel_mm": 8., **fields})
                with self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
                self.assertEqual(rig.disabled, [])

    def test_sensor_headroom_and_attempt_budget_are_checked_before_any_reverse(self):
        for kind in ("sensor-headroom", "attempts"):
            with self.subTest(kind=kind):
                rig = Rig()
                if kind == "sensor-headroom":
                    rig.owner.full_scale_g = 1.25
                if kind == "attempts":
                    rig.workflow.max_attempts = 1
                rig.home_results.append({"travel_mm": 20., "halt_travel_mm_signed": -20.})
                with self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_stale_or_changed_halt_evidence_fails_closed_and_preserves_other_motors(self):
        changes = {
            "source": lambda r, a, rec: r.owner.last.__setitem__(a, dict(r.owner.last[a])),
            "generation": lambda r, a, rec: setattr(r.owner, "operation_generation", r.owner.operation_generation + 1),
            "sensor-fault": lambda r, a, rec: setattr(r.owner, "sensor_fault", "stale_data"),
            "missing": lambda r, a, rec: r.workflow.session.pop("early_halt"),
            "changed-delta": lambda r, a, rec: rec.update(halt_travel_mm_signed=-9.),
        }
        for travel, (label, change) in itertools.product((8., 51.175), changes.items()):
            with self.subTest(travel=travel, label=label):
                rig = Rig()
                rig.home_results.append({"travel_mm": travel})
                with intercept_return(rig, change), self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
                self.assertEqual(rig.disabled, [])
                self.assertTrue(rig.lines["stepper_y"].enabled)
                self.assertTrue(rig.lines["stepper_z"].enabled)
                self.assertEqual(rig.workflow.last_result["candidate_g"], 1.)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 0)

    def test_mismatched_or_faulted_capture_aborts_without_return(self):
        for travel, kind in itertools.product((8., 51.175), (
                "capture-identity", "capture-fault", "capture-error", "capture-metric")):
            with self.subTest(travel=travel, kind=kind):
                rig = Rig()
                rig.home_results.append({"travel_mm": travel})
                capture = rig.owner.stop_capture

                def captured():
                    result = capture()
                    if kind == "capture-identity":
                        result["_record"] = dict(result["_record"])
                    elif kind in ("capture-fault", "capture-error"):
                        result[kind.split("-", 1)[1]] = "transport failure"
                    elif kind == "capture-metric":
                        result["detector"]["impact_metric"] = "selected_axis"
                    return result

                with mock.patch.object(rig.owner, "stop_capture", side_effect=captured), self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.disabled, [])
                self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
                self.assertEqual(rig.workflow.last_result["candidate_g"], 1.)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 0)

    def test_interruptions_at_each_yield_prevent_any_later_motion(self):
        for travel, stage, fault in itertools.product((8., 51.175), ("cap", "move", "wait", "dwell", "restore"),
                                                      ("cancel", "shutdown", "disconnect", "motor-off")):
            with self.subTest(travel=travel, stage=stage, fault=fault):
                rig = Rig()
                rig.home_results.append({"travel_mm": travel})
                obj, method = {"cap": (rig.owner, "_limit_accel"), "move": (rig.toolhead, "manual_move"),
                               "wait": (rig.toolhead, "wait_moves"), "dwell": (rig.toolhead, "dwell"),
                               "restore": (rig.owner, "_restore_accel")}[stage]
                original = getattr(obj, method)
                armed = [False]

                def fail(*args, **kwargs):
                    value = original(*args, **kwargs)
                    if armed[0]:
                        armed[0] = False
                        if fault == "cancel":
                            rig.workflow.cmd_CANCEL(GCmd())
                        elif fault in ("shutdown", "disconnect"):
                            getattr(rig.workflow, "_" + fault)()
                        else:
                            rig.owner.operation_generation += 1
                            rig.lines["stepper_x"].enabled = False
                    return value

                def activate(r, axis, record):
                    armed[0] = True

                with intercept_return(rig, activate), mock.patch.object(obj, method, side_effect=fail), \
                        self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertLessEqual(len(rig.toolhead.moves), 0 if stage == "cap" else 1)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
                self.assertEqual(rig.disabled, [])
                self.assertEqual(rig.workflow.last_result["candidate_g"], 1.)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.workflow.last_result["recovery"]["count"], 0)
                self.assertIsNone(rig.workflow.last_result["recovery"]["last_return_mm_signed"])

    def test_failed_return_and_stock_homing_exception_never_retry(self):
        for kind in ("move-error", "stock-error"):
            with self.subTest(kind=kind):
                rig = Rig()
                rig.home_results.append(CommandError("transport") if kind == "stock-error" else {"travel_mm": 8.})
                move = rig.toolhead.manual_move

                def broken(coordinates, speed):
                    if kind == "move-error":
                        raise CommandError("movement failed")
                    return move(coordinates, speed)

                with mock.patch.object(rig.toolhead, "manual_move", side_effect=broken), self.assertRaises(CommandError):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
                self.assertEqual(rig.toolhead.accel, 50000.)

    def test_cartesian_recovery_does_not_require_the_unrelated_motor_enabled(self):
        rig = machine(cartesian=True)
        rig.lines["stepper_y"].enabled = False
        rig.home_results.append({"travel_mm": 8.})
        rig.calibrate(save=0)
        self.assertFalse(rig.lines["stepper_y"].enabled)
        self.assertEqual(rig.disabled, [])
        self.assertEqual(rig.workflow.last_result["result"], "calibrated")
