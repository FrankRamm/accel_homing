"""Steady-travel vibration recorded on every first-pass home, without the contact build-up."""
import types
import unittest

from harness import arm_axis, batch, counts, make_homing, plugin
from test_cruise_threshold import HIT, armed, still, with_x
from test_mapped_detector import deliver, prepared
from test_session import FakeRail, start_session, with_endstops


def feed(obj, levels, first=400, size=8):
    """Contiguous 8-sample batches, one X reading per batch at the given level (g)."""
    for number, level in enumerate(levels):
        obj._inspect(batch(first + number * size, with_x(level) if level else still()))


class CruisePeakTests(unittest.TestCase):
    def record(self, obj, attempt):
        obj._record(attempt)
        return obj.last["x"]

    def test_peak_counts_cruise_batches_older_than_20_ms_before_the_trigger_batch(self):
        obj, attempt, trigger = armed()
        attempt.cruise_clock = 400
        obj._inspect(batch(300, with_x(6.0)))  # start-up ramp, before the cruise window
        # Batches start at 400, 408, ... and end 7 ticks (ms) later. The trigger
        # batch starts at 456, so only batches ending by 436 count.
        feed(obj, [2.0, 1.0, 1.5, 2.5, 3.0, 0.0, 5.0, 12.0])
        self.assertEqual(trigger.messages, [[HIT]])
        record = self.record(obj, attempt)
        self.assertEqual(record["result"], "impact")
        self.assertEqual(record["cruise_peak_g"], round(counts(2.5) * attempt.count_g, 2))
        self.assertIsNone(record["cruise_threshold_g"])
        self.assertIn(", cruise peak %s g" % record["cruise_peak_g"], plugin.describe(record))

    def test_peak_is_the_trigger_metric_in_either_direction(self):
        obj, attempt, _trigger = armed()
        attempt.cruise_clock = 400
        feed(obj, [-3.5, 1.0, 0.0, 0.0, 0.0, 12.0])
        self.assertAlmostEqual(self.record(obj, attempt)["cruise_peak_g"], 3.5, delta=0.01)

    def test_no_window_gives_no_peak(self):
        cases = {
            "trigger within 20 ms of the cruise start": ([2.0, 1.0, 12.0], 400),
            "no cruise window (second pass or experiment)": ([2.0, 1.0, 1.0, 1.0, 1.0, 12.0], None),
            "no trigger": ([2.0, 1.0, 1.0, 1.0, 1.0, 1.0], 400),
        }
        for name, (levels, cruise_clock) in cases.items():
            with self.subTest(name):
                obj, attempt, _trigger = armed()
                attempt.cruise_clock = cruise_clock
                feed(obj, levels)
                self.assertIsNone(self.record(obj, attempt)["cruise_peak_g"])

    def test_window_and_exclusion_survive_the_32_bit_clock_wrap(self):
        obj = make_homing()
        start = 0xFFFFFF00
        for i in range(12):
            obj._inspect(batch(start + i * 8, still()))
        attempt, trigger = arm_axis(obj, "x", start + 100, start + 90000)
        attempt.cruise_clock = (start + 0xC0) & 0xFFFFFFFF
        feed(obj, [6.0], first=start + 0xB8)  # just before the cruise start
        feed(obj, [2.0, 1.0, 1.0, 2.8, 1.0, 1.0, 1.0, 4.0, 12.0], first=start + 0xC0)
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertGreater(start + 0xC0 + 8 * 8, 0xFFFFFFFF)
        self.assertAlmostEqual(self.record(obj, attempt)["cruise_peak_g"], 2.8, delta=0.01)

    def test_held_back_batches_are_bounded(self):
        obj, attempt, _trigger = armed()
        attempt.cruise_clock = 400
        obj.cruise_exclusion = 10 ** 6  # nothing is ever old enough
        feed(obj, [3.0] + [1.0] * 200)
        self.assertEqual(len(attempt.cruise_queue), 65)
        feed(obj, [12.0], first=400 + 201 * 8)
        # Overflowing batches are counted early: a higher, never lower, peak.
        self.assertAlmostEqual(self.record(obj, attempt)["cruise_peak_g"], 3.0, delta=0.01)

    def test_cruise_threshold_and_peak_measure_the_same_window(self):
        obj, attempt, trigger = armed(3.0)
        obj._inspect(batch(200, with_x(-5.0)))
        feed(obj, [2.5, 1.0, 1.0, 1.0, 0.0, 0.0, 4.0])
        self.assertEqual(trigger.messages, [[HIT]])
        record = self.record(obj, attempt)
        self.assertTrue(record["cruise_threshold_active"])
        self.assertAlmostEqual(record["cruise_peak_g"], 2.5, delta=0.01)

    def test_mapped_detector_uses_its_component_metric(self):
        obj, attempt, _profile = prepared()
        attempt.cruise_clock = attempt.arm_clock
        quiet = [(0.0, 0.0, 0.0)] * 5
        offsets = iter(range(100))

        def send(level, axis=0):
            rows = list(quiet)
            row = [0.0, 0.0, 0.0]
            row[axis] = level
            rows[2] = tuple(row)
            deliver(obj, attempt, rows, offset=next(offsets) * 0.005)

        send(0.3)
        send(0.6)
        send(0.9, axis=1)  # another sensor channel is not the mapped component
        for _ in range(4):
            send(0.0)
        send(0.8)  # 15 ms before the trigger batch: contact build-up
        send(0.0)
        send(0.0)
        send(1.5)
        self.assertEqual(attempt.reason, "impact")
        obj._record(attempt)
        self.assertAlmostEqual(obj.last["x"]["cruise_peak_g"], 0.6, delta=0.01)


class CruiseWindowPlanTests(unittest.TestCase):
    def test_every_first_pass_has_a_window_and_later_passes_none(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 1000})
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        move = types.SimpleNamespace(get_mcu_endstops=lambda: [endstop])
        obj._handle_move_begin(move)
        first = obj.prepare(endstop, 100.0)
        self.assertEqual(first.cruise_clock, obj.sensor_clock.print_time_to_clock(100.4) & 0xFFFFFFFF)
        self.assertIsNone(first.cruise_limit_raw)
        obj._handle_move_begin(move)
        self.assertIsNone(obj.prepare(endstop, 200.0).cruise_clock)


if __name__ == "__main__":
    unittest.main()
