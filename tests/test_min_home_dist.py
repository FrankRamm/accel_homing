"""min_home_dist: short G28 homes are backed off and re-homed."""
import types
import unittest

from harness import CommandError, make_homing, plugin
from test_retract_noise import PositionedToolhead
from test_session import FakeRail, info, with_endstops
from test_workflows import Kin


class MinHomeDistTests(unittest.TestCase):
    def machine(self, axes="xy", travels=None, minimum=10.0, bounds=(0.0, 300.0),
                positive=False, reason="impact", kept_origin=False, extra=None,
                reasons=None, retract=0.0):
        reason_default = reason
        """travels: {axis: [first, rehome1, ...]} scripted travel_mm_exact per home.

        A (first pass, second pass) pair runs both homing moves of one home
        through the real homing_move_end handler, as with homing_retract_dist.
        """
        travels = {a: list((travels or {}).get(a, [150.0])) for a in axes}
        kin = Kin()
        state = types.SimpleNamespace(rehomes=[], homes={a: 0 for a in axes}, on_rehome=None)
        objects = {}

        def run_passes(obj, axis, rail, reason, script):
            sign = 1.0 if positive else -1.0
            stepper = types.SimpleNamespace(get_step_dist=lambda: 0.0125)
            for pass_no, travel in enumerate(script, 1):
                steps = int(round(sign * travel / 0.0125))
                record = {"axis": axis, "pass": pass_no, "result": reason, "moving": True,
                          "motion_seen": True, "onset_g": 9.0, "impact_peak_g": 12.0,
                          "noise_pos_g": 1.0, "noise_neg_g": 1.0, "trigger_after_arm_s": 0.1}
                obj.last[axis] = record
                obj.pending_move = types.SimpleNamespace(
                    endstop=rail.endstop, axis=axis, pass_no=pass_no, reason=reason,
                    record=record, effective_speed=100.0)
                # CoreXY: X turns both motors one way, Y turns them apart.
                moved = {"stepper_x": steps, "stepper_y": steps if axis == "x" else -steps}
                obj._handle_move_end(types.SimpleNamespace(
                    get_mcu_endstops=lambda: [rail.endstop],
                    stepper_positions=[types.SimpleNamespace(
                        stepper_name=name, stepper=stepper, start_pos=0, trig_pos=count,
                        halt_pos=count + int(sign)) for name, count in moved.items()]))
            return record

        def run_home(homing, axis):
            obj = objects["owner"]
            rail = obj.rails_by_axis[axis]
            index = "xy".index(axis)
            script = travels[axis]
            travel = script[min(state.homes[axis], len(script) - 1)]
            reason = (reasons or {}).get(axis, [reason_default])[
                min(state.homes[axis], len((reasons or {}).get(axis, [0])) - 1)]
            state.homes[axis] += 1
            obj.homing_active = True
            obj.session_axes = {axis: rail}
            obj.homed_before = None
            obj.first_pass = None
            if isinstance(travel, tuple):
                record = run_passes(obj, axis, rail, reason, travel)
            else:
                record = {"axis": axis, "result": reason, "moving": True, "motion_seen": True,
                          "travel_mm_exact": travel}
                obj.last[axis] = record
                obj.last_pass = types.SimpleNamespace(axis=axis, reason=reason, record=record)
            if kept_origin:
                record["kept_origin"] = True
            kin.limits[index] = bounds
            obj.toolhead.position[index] = rail.get_homing_info().position_endstop
            obj._handle_rails_end(homing, [rail])

        def home(homing):
            for axis in axes:
                run_home(homing, axis)
            return "stock result"

        class Homing:
            def home_rails(self_, rails, forcepos, homepos):
                state.rehomes.append((list(rails), list(forcepos), list(homepos)))
                axis = next(a for a in axes if objects["owner"].rails_by_axis[a] is rails[0])
                run_home(self_, axis)
                if state.on_rehome:
                    state.on_rehome()

        kin.home = home
        options = {"min_home_dist": str(minimum)}
        options.update(extra or {})
        obj = make_homing(kin=kin, toolhead_cls=PositionedToolhead, options=options)
        objects["owner"] = obj
        endstops = with_endstops(obj, tuple(axes))
        for axis in axes:
            homing_info = info(speed=100.0, retract=retract)._replace(
                positive_dir=positive, position_endstop=bounds[1] if positive else bounds[0])
            obj.rails_by_axis[axis] = FakeRail(endstops[axis], homing_info, rng=bounds)
        state.homing = Homing()
        return obj, kin, state

    def go(self, kin, state):
        return kin.home(state.homing)

    def test_options_fall_back_to_shared_and_default_off(self):
        self.assertEqual(make_homing().min_home_dist, {"x": 0.0, "y": 0.0})
        obj = make_homing(options={"min_home_dist": "10", "y_min_home_dist": "4"})
        self.assertEqual(obj.min_home_dist, {"x": 10.0, "y": 4.0})
        self.assertEqual(obj.get_status(0.0)["min_home_dist"], obj.min_home_dist)
        with self.assertRaises(Exception) as ctx:
            make_homing(options={"x_min_home_dist": "-1"})
        self.assertEqual(type(ctx.exception).__name__, "ConfigError")

    def test_long_home_is_accepted_without_rehome(self):
        obj, kin, state = self.machine("x", {"x": [150.0]})
        self.go(kin, state)
        self.assertEqual(state.rehomes, [])
        self.assertEqual(obj.toolhead.moves, [([5.0, None, None], 100.0)])

    def test_short_genuine_home_rehomes_and_accepts(self):
        obj, kin, state = self.machine("y", {"y": [3.0, 9.0]})
        self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)
        rails, forcepos, homepos = state.rehomes[0]
        self.assertEqual(rails, [obj.rails_by_axis["y"]])
        self.assertEqual(homepos, [None, 0.0, None, None])
        self.assertEqual(forcepos[1], 450.0)
        # back-off to endstop + minimum, then the final back-off (5 mm) after re-home
        self.assertEqual(obj.toolhead.moves[0][0], [None, 10.0, None])
        self.assertEqual(obj.toolhead.moves[-1][0], [None, 5.0, None])
        self.assertEqual(len(obj.toolhead.moves), 2)

    def test_false_early_trigger_recovered_by_long_rehome(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 140.0]})
        self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)

    def test_rehome_logs_each_attempt_and_acceptance(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 2.0, 9.0]})
        with self.assertLogs(level="INFO") as logs:
            self.go(kin, state)
        lines = [r.getMessage() for r in logs.records if "accel_homing: X" in r.getMessage()]
        self.assertEqual(lines, [
            "accel_homing: X homed short (travels 3.00 mm, need 10.0 mm), "
            "backing off 10.0 mm for re-home 1 of 2",
            "accel_homing: X homed short (travels 3.00/2.00 mm, need 8.5 mm), "
            "backing off 10.0 mm for re-home 2 of 2",
            "accel_homing: X re-home accepted, stop confirmed (travels 3.00/2.00/9.00 mm)",
        ])

    def test_repeated_short_homes_raise_and_never_back_off(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 2.0, 2.5]})
        with self.assertRaisesRegex(CommandError, "less than min_home_dist 10.0 mm, 3 times"):
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 2)
        self.assertNotIn([5.0, None, None], [m[0] for m in obj.toolhead.moves])

    def test_zero_disables(self):
        obj, kin, state = self.machine("x", {"x": [0.5]}, minimum=0.0)
        self.go(kin, state)
        self.assertEqual(state.rehomes, [])

    def test_gentle_retry_backoff_respects_the_axis_speed(self):
        obj, kin, state = self.machine("y", {"y": [3., 10.]})
        rail = obj.rails_by_axis["y"]
        rail.info = rail.info._replace(speed=40., retract_speed=125.)
        obj.workflows.gentle_homing = True
        self.go(kin, state)
        self.assertEqual(obj.toolhead.moves, [([None, 10., None], 40.), ([None, 5., None], 40.)])

    def test_cancellation_during_an_otherwise_accepted_retry_is_reported(self):
        obj, kin, state = self.machine("y", {"y": [3., 10.]})
        state.on_rehome = obj._cancel_pending_operations
        with self.assertRaisesRegex(CommandError, "re-home was interrupted"):
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)
        self.assertEqual(len(obj.toolhead.moves), 1)

    def test_missing_travel_counts_as_short(self):
        obj, kin, state = self.machine("x", {"x": [None, 9.5]})
        self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)

    def test_pressed_kept_origin_record_rehomes(self):
        obj, kin, state = self.machine("x", {"x": [0.2, 150.0]}, reason="pressed",
                                       kept_origin=True)
        obj.homed_before = None
        self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)

    def test_only_short_axis_rehomed_in_xy_home(self):
        obj, kin, state = self.machine("xy", {"x": [150.0], "y": [2.0, 10.0]})
        self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)
        self.assertEqual(state.rehomes[0][0], [obj.rails_by_axis["y"]])
        finals = [m[0] for m in obj.toolhead.moves if m[0] == [5.0, 5.0, None]]
        self.assertEqual(len(finals), 1)

    def test_back_off_out_of_range_raises_before_move(self):
        obj, kin, state = self.machine("x", {"x": [3.0]}, bounds=(0.0, 8.0))
        with self.assertRaisesRegex(CommandError, "out of range"):
            self.go(kin, state)
        self.assertEqual(obj.toolhead.moves, [])
        self.assertEqual(state.rehomes, [])

    def test_workflow_session_never_rehomes(self):
        obj, kin, state = self.machine("x", {"x": [3.0]})
        obj.workflows.session = object()
        self.go(kin, state)
        self.assertEqual(state.rehomes, [])

    def test_cancel_between_attempts_raises_without_back_off(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 2.0, 2.5]})
        state.on_rehome = obj._cancel_pending_operations
        with self.assertRaisesRegex(CommandError, "re-home was interrupted"):
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)
        self.assertEqual(len(obj.toolhead.moves), 1)

    def test_shutdown_before_back_off_raises_without_move(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 2.0, 2.5]})
        state.on_rehome = lambda: setattr(obj.printer, "is_shutdown", lambda: True)
        with self.assertRaisesRegex(CommandError, "re-home was interrupted"):
            self.go(kin, state)
        self.assertEqual(len(obj.toolhead.moves), 1)

    def test_rehome_not_accepted_raises(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 150.0]}, reasons={"x": ["impact", "pressed"]})
        with self.assertRaisesRegex(CommandError, "X re-home was not accepted"):
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)

    def test_unaccepted_home_is_rehomed_when_min_set(self):
        obj, kin, state = self.machine("x", {"x": [150.0, 150.0]},
                                       reasons={"x": ["pressed", "impact"]})
        self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)
        self.assertEqual(obj.toolhead.moves[-1][0], [5.0, None, None])

    def test_unaccepted_home_untouched_when_min_zero(self):
        obj, kin, state = self.machine("x", {"x": [0.5]}, minimum=0.0,
                                       reasons={"x": ["pressed"]})
        self.go(kin, state)
        self.assertEqual(state.rehomes, [])
        self.assertEqual(obj.toolhead.moves, [])

    def test_positive_dir_backs_off_toward_lower_values(self):
        obj, kin, state = self.machine("x", {"x": [3.0, 150.0]}, positive=True)
        self.go(kin, state)
        self.assertEqual(obj.toolhead.moves[0][0], [290.0, None, None])
        self.assertEqual(state.rehomes[0][1][0], 300.0 + -1.5 * 300.0)
        self.assertEqual(obj.toolhead.moves[-1][0], [295.0, None, None])

    def test_retract_second_touch_does_not_count_as_a_short_home(self):
        for axis in "xy":
            with self.subTest(axis=axis):
                obj, kin, state = self.machine(axis, {axis: [(150.0, 5.0)]}, retract=5.0)
                self.go(kin, state)
                self.assertEqual(state.rehomes, [])
                record = obj.last[axis]
                self.assertEqual(record["pass"], 2)
                self.assertAlmostEqual(record["travel_mm_exact"], 5.0)
                target = [5.0 if axis == "x" else None, 5.0 if axis == "y" else None, None]
                self.assertEqual(obj.toolhead.moves, [(target, 100.0)])

    def test_short_first_touch_is_rehomed_even_when_second_touch_is_long_enough(self):
        obj, kin, state = self.machine("x", {"x": [(2.0, 5.0), (4.0, 5.0)]}, minimum=4.0,
                                       retract=5.0)
        with self.assertLogs(level="INFO") as logs:
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 1)
        self.assertEqual(obj.toolhead.moves[0][0], [4.0, None, None])
        self.assertEqual(obj.toolhead.moves[-1][0], [5.0, None, None])
        lines = [r.getMessage() for r in logs.records if r.getMessage().startswith("accel_homing: X")]
        self.assertEqual(lines, [
            "accel_homing: X homed short (travels 2.00 mm, need 4.0 mm), "
            "backing off 4.0 mm for re-home 1 of 2",
            "accel_homing: X re-home accepted, stop confirmed (travels 2.00/4.00 mm)",
        ])

    def test_two_pass_rehome_loop_judges_each_first_touch(self):
        obj, kin, state = self.machine("x", {"x": [(3.0, 5.0), (2.0, 5.0), (9.0, 5.0)]},
                                       retract=5.0)
        with self.assertLogs(level="INFO") as logs:
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 2)
        self.assertIn("accel_homing: X re-home accepted, stop confirmed (travels 3.00/2.00/9.00 mm)",
                      [r.getMessage() for r in logs.records])
        self.assertEqual(obj.toolhead.moves[-1][0], [5.0, None, None])
        obj, kin, state = self.machine("x", {"x": [(3.0, 5.0), (2.0, 5.0), (2.5, 5.0)]},
                                       retract=5.0)
        with self.assertRaisesRegex(CommandError, r"after 3\.00/2\.00/2\.50 mm, less than "
                                                  r"min_home_dist 10\.0 mm, 3 times"):
            self.go(kin, state)
        self.assertEqual(len(state.rehomes), 2)
        self.assertNotIn([5.0, None, None], [m[0] for m in obj.toolhead.moves])

    def test_none_travel_renders_question_mark_in_error(self):
        obj, kin, state = self.machine("x", {"x": [None, 2.0, 2.5]})
        with self.assertRaisesRegex(CommandError, r"after \?/2\.00/2\.50 mm"):
            self.go(kin, state)


if __name__ == "__main__":
    unittest.main()
