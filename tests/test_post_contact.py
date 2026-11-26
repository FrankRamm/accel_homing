"""Post-contact push, stop-request latency and start-position diagnostics; offline only."""
import types
import unittest
from unittest import mock

from harness import arm_axis, batch, make_homing, plugin, stationary_stream
from test_retract_noise import PositionedToolhead
from test_session import FakeRail, info, start_session, with_endstops
from test_trigger import feed_baseline, still, x_sample
from test_workflows import Kin

STEP = 0.0125


def positions(travel, push, names=("stepper_x",)):
    stepper = types.SimpleNamespace(get_step_dist=lambda: STEP)
    trig = -int(round(travel / STEP))
    halt = trig - int(round(push / STEP))
    return [types.SimpleNamespace(stepper_name=name, stepper=stepper, start_pos=0,
                                  trig_pos=trig, halt_pos=halt) for name in names]


def line_kin(obj):
    obj.toolhead.kin = types.SimpleNamespace(
        get_steppers=lambda: [types.SimpleNamespace(get_name=lambda: "stepper_x")],
        calc_position=lambda moved: [moved["stepper_x"], 0.0, 0.0])


class PostContactTests(unittest.TestCase):
    def impact(self, obj, now=.2135):
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        attempt.effective_speed = 100.0
        obj.reactor.now = now
        hit = still()
        hit[3] = x_sample(-12.0)
        obj._inspect(batch(208, hit))
        self.assertEqual(attempt.reason, "impact")
        obj.disarm()
        return attempt, trigger

    def finish_move(self, obj, attempt, travel=120.0, push=2.5):
        messages = []
        obj.fake_printer.objects["gcode"].respond_info = messages.append
        line_kin(obj)
        obj._handle_move_end(types.SimpleNamespace(get_mcu_endstops=lambda: [attempt.endstop],
                                                   stepper_positions=positions(travel, push)))
        return messages

    def test_native_mapped_path_without_observer_records_stop_request(self):
        obj = make_homing()
        self.assertIsNone(getattr(obj, "contact_timing_observer", None))
        attempt, trigger = self.impact(obj)
        self.assertEqual(trigger.messages, [[plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT]])
        self.assertIsNone(attempt.motion_deadline)
        self.assertEqual(attempt.request_time, .2135)
        # Onset sample 3 of the packet starting at clock 208, one tick per sample.
        self.assertAlmostEqual(attempt.record["onset_time_s"], .211)
        self.assertAlmostEqual(attempt.record["stop_request_after_onset_ms"], 2.5, places=6)

    def test_stop_request_is_omitted_without_a_finite_comparable_time(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        obj.main_mcu.estimated_print_time = mock.Mock(side_effect=RuntimeError("clock"))
        hit = still()
        hit[3] = x_sample(-12.0)
        obj._inspect(batch(208, hit))
        obj.disarm()
        self.assertIsNone(attempt.request_time)
        self.assertIsNone(attempt.record["stop_request_after_onset_ms"])
        for reason in ("no_data", "travel"):
            obj = make_homing()
            feed_baseline(obj)
            attempt, _trigger = arm_axis(obj, "x", 200, 9000)
            obj._fire(attempt, reason, plugin.mcu.MCU_trsync.REASON_HOST_REQUEST)
            obj.disarm()
            self.assertIsNotNone(attempt.request_time)
            self.assertIsNone(attempt.record["stop_request_after_onset_ms"])

    def test_push_after_contact_is_recorded_described_and_reported_once(self):
        obj = make_homing()
        attempt, _trigger = self.impact(obj)
        position_calls = []
        obj.toolhead.set_position = position_calls.append
        messages = self.finish_move(obj, attempt, travel=120.0, push=2.5)
        record = attempt.record
        self.assertEqual(record["post_trigger_mm"], 2.5)
        self.assertAlmostEqual(abs(record["halt_travel_mm_signed"] - record["trigger_travel_mm_signed"]), 2.5)
        line = plugin.describe(record)
        self.assertIn("post-trigger 2.500 mm", line)
        self.assertIn("stop request 2.5 ms after onset", line)
        self.assertEqual(messages, [
            "accel_homing: X pass 1: after contact the motor was still commanded 2.50 mm into"
            " the stop (about 25 ms at 100 mm/s). This is where steps are lost. A lower"
            " homing_speed reduces it."])
        self.assertEqual(position_calls, [])
        self.assertEqual(obj.get_status(0.)["last"]["x"]["post_trigger_mm"], 2.5)
        self.assertEqual(obj.get_status(0.)["last"]["x"]["stop_request_after_onset_ms"], 2.5)

    def test_second_touch_names_second_homing_speed(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=5.0)))
        attempt, _trigger = self.impact(obj)
        attempt.endstop, attempt.pass_no = endstop, 2
        messages = self.finish_move(obj, attempt, travel=5.0, push=1.5)
        self.assertEqual(len(messages), 1)
        self.assertIn("X pass 2:", messages[0])
        self.assertIn("A lower second_homing_speed reduces it.", messages[0])

    def test_no_report_at_or_below_limit_for_other_results_or_missing_values(self):
        obj = make_homing()
        attempt, _trigger = self.impact(obj)
        self.assertEqual(self.finish_move(obj, attempt, push=1.0), [])
        self.assertEqual(attempt.record["post_trigger_mm"], 1.0)
        obj = make_homing()
        attempt, _trigger = self.impact(obj)
        attempt.record["result"] = attempt.reason = "pressed"
        self.assertEqual(self.finish_move(obj, attempt, push=4.0), [])
        self.assertEqual(attempt.record["post_trigger_mm"], 4.0)
        obj = make_homing()
        attempt, _trigger = self.impact(obj)
        with mock.patch.object(obj, "_travel", return_value=None):
            self.assertEqual(self.finish_move(obj, attempt), [])
        self.assertIsNone(attempt.record["post_trigger_mm"])
        self.assertNotIn("post-trigger", plugin.describe(attempt.record))

    def test_console_failure_never_fails_the_home(self):
        obj = make_homing()
        attempt, _trigger = self.impact(obj)
        line_kin(obj)
        with self.assertLogs(level="ERROR"):
            obj._handle_move_end(types.SimpleNamespace(get_mcu_endstops=lambda: [attempt.endstop],
                                                       stepper_positions=positions(120.0, 3.0)))
        self.assertEqual(attempt.record["post_trigger_mm"], 3.0)
        self.assertIs(obj.last_pass, attempt)


class StartPositionTests(unittest.TestCase):
    def session(self, homed):
        kin = Kin()
        obj = make_homing(kin=kin, toolhead_cls=PositionedToolhead)
        endstop = with_endstops(obj)["x"]
        rail = FakeRail(endstop, info(speed=100.0, retract=0.0))
        if homed:
            kin.limits[0] = (0.0, 300.0)
        obj.toolhead.position[0] = 150.0
        start_session(obj, rail)
        # Stock homing forces the start position only after home_rails_begin.
        obj.toolhead.position[0] = 450.0
        stationary_stream(obj)
        obj._prepare_reference()
        attempt = obj.prepare(endstop, obj.preparation.last + .001)
        obj._record(attempt)
        return obj, endstop, attempt

    def finish(self, obj, endstop, attempt, travel):
        obj.pending_move = attempt
        obj._handle_move_end(types.SimpleNamespace(
            get_mcu_endstops=lambda: [endstop],
            stepper_positions=positions(travel, 0.1, ("stepper_x", "stepper_y"))))

    def test_homed_axis_reports_believed_distance_and_its_error(self):
        obj, endstop, attempt = self.session(homed=True)
        record = attempt.record
        self.assertIs(record["axis_was_homed"], True)
        self.assertEqual(record["believed_distance_to_stop_mm"], 150.0)
        self.assertEqual(record["start_position_mm"], 450.0)
        self.assertEqual(record["expected_travel_mm"], 450.0)
        self.finish(obj, endstop, attempt, 149.2)
        self.assertAlmostEqual(record["believed_minus_trigger_travel_mm"], 0.8)
        self.assertAlmostEqual(record["travel_mm"], 149.2)

    def test_unhomed_axis_has_no_believed_distance(self):
        obj, endstop, attempt = self.session(homed=False)
        record = attempt.record
        self.assertIs(record["axis_was_homed"], False)
        self.assertIsNone(record["believed_distance_to_stop_mm"])
        self.finish(obj, endstop, attempt, 149.2)
        self.assertIsNone(record["believed_minus_trigger_travel_mm"])

    def test_only_the_first_pass_gets_a_believed_distance(self):
        obj, endstop, first = self.session(homed=True)
        obj._prepare_reference()
        second = obj.prepare(endstop, obj.preparation.last + .001)
        self.assertEqual(first.believed_distance, 150.0)
        self.assertIsNone(second.believed_distance)

    def test_rails_begin_clears_the_previous_first_pass(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        obj.first_pass = object()
        start_session(obj, FakeRail(endstop, info(retract=5.0)))
        self.assertIsNone(obj.first_pass)


if __name__ == "__main__":
    unittest.main()
