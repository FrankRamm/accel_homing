"""A lower trigger limit that only applies once the homing move is cruising."""
import types
import unittest

from harness import ConfigError, arm_axis, batch, counts, make_homing, plugin
from test_session import FakeRail, start_session, with_endstops

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
ONE_G_Z = (0, 0, counts(1.0))


def still(n=8):
    return [ONE_G_Z] * n


def with_x(g, position=3):
    samples = still()
    samples[position] = (counts(g), 0, counts(1.0))
    return samples


def armed(cruise_g=None, cruise_clock=400):
    obj = make_homing()
    for i in range(12):
        obj._inspect(batch(i * 8, still()))
    attempt, trigger = arm_axis(obj, "x", 200, 90000)
    if cruise_g is not None:
        attempt.cruise_limit_raw = cruise_g / attempt.count_g
        attempt.cruise_clock = cruise_clock
    return obj, attempt, trigger


class CruiseTriggerTests(unittest.TestCase):
    def test_start_of_move_keeps_the_configured_threshold(self):
        obj, attempt, trigger = armed(3.0)
        obj._inspect(batch(200, with_x(-5.0)))
        obj._inspect(batch(208, with_x(5.0)))
        self.assertEqual(trigger.messages, [])
        self.assertFalse(attempt.cruise_active)

    def test_cruise_limit_triggers_on_a_softer_contact(self):
        obj, attempt, trigger = armed(3.0)
        obj._inspect(batch(200, with_x(-5.0)))
        obj._inspect(batch(400, with_x(-2.5)))
        self.assertEqual(trigger.messages, [])
        self.assertTrue(attempt.cruise_active)
        obj._inspect(batch(408, with_x(-4.0, position=5)))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(attempt.reason, "impact")
        self.assertEqual(attempt.onset_batch[2], 5)
        obj._record(attempt)
        record = obj.last["x"]
        self.assertEqual(record["threshold_g"], 8.0)
        self.assertEqual(record["cruise_threshold_g"], 3.0)
        self.assertTrue(record["cruise_threshold_active"])

    def test_without_a_cruise_limit_the_same_contact_does_not_trigger(self):
        obj, attempt, trigger = armed()
        obj._inspect(batch(200, still()))
        obj._inspect(batch(408, with_x(-4.0)))
        self.assertEqual(trigger.messages, [])
        obj._record(attempt)
        self.assertIsNone(obj.last["x"]["cruise_threshold_g"])
        self.assertFalse(obj.last["x"]["cruise_threshold_active"])

    def test_hard_impact_before_cruise_still_triggers(self):
        obj, attempt, trigger = armed(3.0)
        obj._inspect(batch(200, with_x(-12.0)))
        self.assertEqual(trigger.messages, [[HIT]])
        obj._record(attempt)
        self.assertEqual(obj.last["x"]["cruise_threshold_g"], 3.0)
        self.assertFalse(obj.last["x"]["cruise_threshold_active"])


class CruisePlanTests(unittest.TestCase):
    def prepared(self, options, session=None):
        obj = make_homing(accel=20000.0, options=dict({"homing_accel": 1000}, **options))
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        if session is not None:
            obj.workflows.session = session
            obj._detection_mapping = lambda axis: None
        move = types.SimpleNamespace(get_mcu_endstops=lambda: [endstop])
        obj._handle_move_begin(move)
        return obj, endstop, move, obj.prepare(endstop, 100.0)

    def test_cruise_starts_after_the_ramp_and_settle(self):
        obj, endstop, move, attempt = self.prepared({"x_cruise_threshold": 3.0})
        self.assertAlmostEqual(attempt.cruise_limit_raw * attempt.count_g, 3.0)
        # 150 mm/s at 1000 mm/s^2 ramps for 0.15 s, then 0.25 s of settle.
        expected = obj.sensor_clock.print_time_to_clock(100.0 + 0.15 + 0.25) & 0xFFFFFFFF
        self.assertEqual(attempt.cruise_clock, expected)
        self.assertEqual(attempt.limit_raw, attempt.configured_limit_raw)
        obj._handle_move_begin(move)
        second = obj.prepare(endstop, 200.0)
        self.assertIsNone(second.cruise_limit_raw)

    def test_shared_option_applies_to_both_axes_and_axis_option_overrides(self):
        obj = make_homing(options={"cruise_threshold": 3.0, "y_cruise_threshold": 0.0})
        self.assertEqual(obj.cruise_threshold_g, {"x": 3.0, "y": 0.0})
        self.assertEqual(make_homing().cruise_threshold_g, {"x": 0.0, "y": 0.0})

    def test_disabled_when_not_below_the_threshold_or_in_calibration(self):
        for options, session in (({}, None), ({"x_cruise_threshold": 8.0}, None),
                                 ({"x_cruise_threshold": 3.0}, {"mode": "calibration", "state": "running"})):
            with self.subTest(options=options, session=session):
                attempt = self.prepared(options, session)[3]
                self.assertIsNone(attempt.cruise_limit_raw)
                self.assertIsNone(attempt.cruise_clock)

    def test_negative_value_is_a_config_error(self):
        with self.assertRaises(ConfigError):
            make_homing(options={"x_cruise_threshold": -1.0})


if __name__ == "__main__":
    unittest.main()
