"""Synthetic motion/step histories for reviewer P1/P2; no printer access."""
import json
import math
import types
import unittest
from unittest import mock

from harness import (CommandError, ConfigError, FakeEndstop, acquire_reference, batch,
                     counts, make_homing, plugin)
from test_backends import baseline, feed, generic
from test_session import FakeRail, info, with_endstops
from test_single_touch import RampStepper
from test_workflows import Rig

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST
G = 9806.65


class SignedRamp(RampStepper):
    def __init__(self, start, accel, speed, direction):
        super().__init__(start, accel, speed)
        self.direction = direction

    def get_past_mcu_position(self, stamp):
        return self.direction * super().get_past_mcu_position(stamp)


def prepared_generic(speed=50.0, velocity=500.0, accel=5000.0, direction=1,
                     options=None, start=10.060):
    obj = generic(options={"homing_accel": accel, **(options or {})})
    obj.toolhead.get_max_velocity = lambda: (velocity, accel)
    baseline(obj, start - 0.060)
    endstop = FakeEndstop("x")
    stepper = SignedRamp(start, accel, min(speed, velocity), direction)
    endstop.get_steppers = lambda: [stepper]
    obj.session_axes["x"] = FakeRail(endstop, info(speed=speed, retract=0, second=speed))
    acquire_reference(obj, endstop, start)
    attempt = obj.prepare(endstop, start)
    obj.arm(attempt)
    return obj, attempt, endstop.trigger_log


def prepared_fast_generic(direction=1, start=10.060):
    """An explicitly admitted 75 mm/s plan with synthetic fresh delivery."""
    speed, accel, age, rate = 75.0, 5360.0, 0.0001, 3200.0
    obj = generic(options={"homing_accel": 0.0, "max_sample_age": 0.005,
                           "stream_interval": 0.001, "max_report_travel": 1.0,
                           "max_startup_travel": 3.0})
    obj.toolhead.get_max_velocity = lambda: (500.0, accel)
    endstop = FakeEndstop("x")
    endstop.get_steppers = lambda: [SignedRamp(start, accel, speed, direction)]
    obj.session_axes["x"] = FakeRail(endstop, info(speed=speed, retract=0, second=speed))
    obj.reactor.now = start - obj.baseline_time - 0.020

    def stationary():
        now = obj.reactor.monotonic()
        # All 16 uniformly spaced rows are younger than the explicit 5 ms
        # policy; this models the fixture's delivery, not real-device latency.
        first = now - age - 15.0 / rate
        feed(obj, [(first + number / rate, 0.0, 0.0, G) for number in range(16)], age=age)

    obj.reactor.on_pause = stationary
    obj._prepare_reference()
    while obj.reactor.monotonic() + 0.005 <= start + 0.000000001:
        obj.reactor.pause(obj.reactor.monotonic() + 0.005)
    attempt = obj.prepare(endstop, start)
    obj.arm(attempt)
    return obj, attempt, endstop.trigger_log


def stream_ramp(obj, attempt, duration=0.150, direction=1, parked=False,
                response_lag=0.0, age=0.001, size=16):
    dt = 1.0 / 3200.0
    number = 0
    while number * dt < duration and not attempt.done:
        rows = []
        for index in range(size):
            offset = (number + index) * dt
            acceleration = 0.0
            if not parked and response_lag <= offset < response_lag + (
                    attempt.effective_speed / attempt.effective_accel):
                acceleration = direction * attempt.effective_accel
            # Balanced, fresh, varying noise avoids the separate frozen check.
            noise = 0.0002 * G * (1 if (number + index) % 2 else -1)
            rows.append((attempt.arm_time + offset, acceleration + noise, 0.0, G))
        feed(obj, rows, age=age)
        obj._watchdog(obj.reactor.monotonic())
        number += size
    return obj.reactor.monotonic()


class GenericBoundedEvidenceTests(unittest.TestCase):
    """Preserve actual pre-deadline lower bounds across report/command phases."""

    rate = 3200.0

    def machine(self, spread=0.32, direction=1):
        from test_backends import RaisedBaselineFloorTest
        obj, endstop = RaisedBaselineFloorTest().machine(
            threshold=1.5, spreads=(spread, 0.04, 0.02), options={"max_sample_age": 0.020})
        obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x"}
        obj.workflows.in_home, obj.workflows.home_axis = True, "x"
        obj.toolhead.get_max_velocity = lambda: (500.0, 5000.0)
        obj.session_axes["x"].info = info(speed=50.0, retract=0, second=50.0)
        obj._prepare_reference()
        when = obj.backend.last_sample_time + 16 / self.rate
        stepper = SignedRamp(when, 5000.0, 50.0, direction)
        stepper.get_past_mcu_position = mock.Mock(wraps=stepper.get_past_mcu_position)
        endstop.get_steppers = lambda: [stepper]
        attempt = obj.prepare(endstop, when)
        stepper.get_past_mcu_position.assert_not_called()
        self.assertIsNone(attempt.motion_deadline)
        self.assertIsNone(attempt.proof_step_positions)
        self.assertEqual(attempt.proof_history, [])
        self.assertAlmostEqual(attempt.proof_deadline - when, 0.055)
        obj.arm(attempt)
        return obj, attempt, endstop

    def stream(self, obj, attempt, phase=16, acceleration=None, direction=1,
               watch_from=0.0, duration=0.13, age=0.001):
        if acceleration is None:
            acceleration = lambda elapsed: direction * 5000.0 if 0 <= elapsed < 0.010 else 0.0
        number, size, observations = 0, 15 + phase, []
        while not attempt.done and not attempt.motion_seen and number / self.rate < duration:
            rows = []
            for index in range(size):
                offset = number + index - 15
                elapsed = offset / self.rate
                noise = 0.0002 * G * (1 if offset % 2 else -1)
                rows.append((attempt.arm_time + elapsed, acceleration(elapsed) + noise, 0.0, G))
            feed(obj, rows, age=age)
            if attempt.proof_state is not None:
                endpoint = obj.clock_to_time(attempt.proof_state[3])
                lower, _upper = obj._motion_speed_bounds(attempt, attempt.proof_state)
                observations.append((endpoint, lower))
            if obj.reactor.now - attempt.arm_time >= watch_from:
                obj._watchdog(obj.reactor.now)
            number += size
            size = 16
        return observations

    def test_same_real_ramp_proves_motion_at_both_report_phases(self):
        for phase in (8, 16):
            for direction in (-1, 1):
                with self.subTest(phase=phase, direction=direction):
                    obj, attempt, endstop = self.machine(direction=direction)
                    self.stream(obj, attempt, phase=phase, direction=direction)
                    self.assertTrue(attempt.motion_seen)
                    self.assertTrue(attempt.moving)
                    self.assertEqual(attempt.proof_method, "full_arm_peak")
                    self.assertGreaterEqual(attempt.proof_speed_peak, 25.0)
                    self.assertLessEqual(attempt.proof_speed_peak_time, attempt.proof_deadline)
                    self.assertTrue(attempt.proof_command_eligible)
                    self.assertFalse(attempt.gap)
                    self.assertIsNone(obj.sensor_fault)
                    self.assertIsNone(attempt.reason)
                    self.assertEqual(endstop.trigger_log.messages, [])
                    self.assertLess(obj.reactor.now, attempt.proof_deadline)

    def test_stationary_noise_bias_and_insufficient_margin_still_fail(self):
        cases = ((0.32, "parked"), (0.32, "noise"), (0.32, "bias"),
                 (0.581, "ramp"), (1.313, "ramp"))
        for spread, profile in cases:
            with self.subTest(spread=spread, profile=profile):
                obj, attempt, endstop = self.machine(spread)
                acceleration = (None if profile == "ramp" else
                                (lambda elapsed: 0.0) if profile == "parked" else
                                (lambda elapsed: 0.8 * attempt.proof_noise[0] * attempt.count_g * G)
                                if profile == "bias" else
                                (lambda elapsed: 0.15 * G * (1 if round(elapsed * self.rate) % 2 else -1)))
                self.stream(obj, attempt, acceleration=acceleration)
                self.assertFalse(attempt.motion_seen)
                self.assertFalse(attempt.moving)
                self.assertLess(attempt.proof_speed_peak, 25.0)
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])
                self.assertLessEqual(obj.reactor.now, attempt.proof_deadline + 0.005)

    def test_peak_needs_half_full_speed_and_command_eligibility(self):
        for profile in ("small_impulse", "uncommanded_ramp"):
            with self.subTest(profile=profile):
                obj, attempt, endstop = self.machine()
                acceleration = None
                if profile == "small_impulse":
                    # Four subthreshold samples exceed half the first eligible
                    # 25 mm/s command, but never half the full 50 mm/s plan.
                    acceleration = lambda elapsed: 14000.0 if 0 <= elapsed < 4 / self.rate else 0.0
                else:
                    attempt.steppers = [SignedRamp(attempt.arm_time + 0.2, 5000.0, 50.0, 1)]
                self.stream(obj, attempt, phase=4, acceleration=acceleration)
                if profile == "small_impulse":
                    self.assertGreater(attempt.proof_speed_peak, 12.5)
                    self.assertLess(attempt.proof_speed_peak, 25.0)
                else:
                    self.assertGreater(attempt.proof_speed_peak, 25.0)
                    self.assertFalse(attempt.proof_command_eligible)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_peak_cutoff_uses_sample_endpoint_and_cannot_be_minted_after_deadline(self):
        obj, attempt, endstop = self.machine()
        def late_impulse(elapsed):
            return 14000.0 if 0.060 <= elapsed < 0.080 else -14000.0 if 0.080 <= elapsed < 0.100 else 0.0
        # Defer the watchdog only in this fixture. A post-deadline jolt and its
        # reversal leave no current lower bound when it runs; a wrongly minted
        # late peak would otherwise rescue this failed startup.
        observations = self.stream(obj, attempt, acceleration=late_impulse, watch_from=0.110)
        self.assertGreater(max(lower for _when, lower in observations), 25.0)
        self.assertEqual(attempt.proof_speed_peak, 0.0)
        self.assertIsNone(attempt.proof_speed_peak_time)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

        obj, attempt, endstop = self.machine()
        observations = self.stream(obj, attempt,
                                   acceleration=lambda elapsed: 14000.0 if 0.040 <= elapsed < 0.050 else 0.0,
                                   watch_from=0.060, age=0.011)
        self.assertGreater(obj.reactor.now, attempt.proof_deadline)
        self.assertTrue(attempt.motion_seen)
        self.assertGreater(attempt.proof_speed_peak, 25.0)
        self.assertLessEqual(attempt.proof_speed_peak_time, attempt.proof_deadline)
        self.assertIsNone(attempt.reason)
        self.assertEqual(endstop.trigger_log.messages, [])

    def test_generic_failure_diagnostics_are_coherent_bounded_and_survive_cleanup(self):
        obj, attempt, endstop = self.machine(spread=1.313)
        self.stream(obj, attempt)
        self.assertEqual(attempt.reason, "no_motion")
        snapshot = attempt.proof_state
        expected_raw = obj._measured_speed(attempt, snapshot)
        original_bounds, original_sums = obj._motion_speed_bounds, attempt.sums
        def concurrent_sums(current, state):
            result = original_bounds(current, state)
            current.sums = [1e9, 1e9, 1e9]
            return result
        with mock.patch.object(obj, "_motion_speed_bounds", side_effect=concurrent_sums):
            obj._motion_state(attempt)
        attempt.sums = original_sums
        self.assertAlmostEqual(attempt.proof_raw, expected_raw)
        obj.disarm()
        obj._end_session()
        obj.workflows.session = None
        obj.workflows.in_home, obj.workflows.home_axis = False, None
        proof = obj.get_status(obj.reactor.now)["last"]["x"]["motion_proof"]
        self.assertEqual(proof["backend"], "klipper_batch")
        self.assertEqual(proof["method"], "full_arm_lower")
        self.assertEqual(proof["result"], "insufficient_bound")
        self.assertAlmostEqual(proof["raw_speed_mm_s"], expected_raw, places=5)
        self.assertGreater(proof["raw_speed_mm_s"], 0.9 * attempt.effective_speed)
        self.assertEqual(proof["lower_speed_mm_s"], 0.0)
        self.assertGreater(proof["upper_speed_mm_s"], proof["raw_speed_mm_s"])
        self.assertEqual(proof["deadline_after_arm_ms"], 55.0)
        self.assertGreaterEqual(proof["request_after_arm_ms"], 55.0)
        self.assertLessEqual(proof["request_after_arm_ms"], 60.0)
        self.assertEqual(proof["first_sample_after_arm_ms"], 0.0)
        self.assertIsNone(proof["first_step_after_arm_ms"])
        self.assertIsNone(proof["history_capacity"])
        self.assertEqual(proof["noise_bound_xyz_g"], tuple(round(value * attempt.count_g, 6)
                                                         for value in attempt.proof_noise))
        self.assertNotIn("samples", proof)
        json.dumps(proof, allow_nan=False)
        proof["noise_bound_xyz_g"] = (999.0, 999.0, 999.0)
        self.assertLess(obj.get_status(obj.reactor.now)["last"]["x"]["motion_proof"]["noise_bound_xyz_g"][0], 1.0)
        messages = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=messages.append))
        summary = next(message for message in messages if "X motion proof" in message)
        self.assertIn("raw/lower/upper", summary)
        self.assertIn("noise XYZ", summary)
        self.assertNotIn("first step", summary)
        self.assertNotIn("history", summary)
        with self.assertRaisesRegex(CommandError, "raw/lower/upper.*deadline.*55.000"):
            obj.finish(endstop, attempt, HOST)


class GenericNoMotionPolicyTests(unittest.TestCase):
    def test_parked_stop_with_varying_noise_fails_first_and_repeated_home(self):
        obj, attempt, trigger = prepared_generic(options={"max_sample_age": 0.020})
        for pass_number in (1, 2):
            if pass_number == 2:
                # Reuse the module/endstop with a new ordinary G28 session;
                # a previous failure must never turn the next home into a hit.
                rail = obj.session_axes["x"]
                obj._end_session()
                obj.homing_active = True
                obj.session_axes["x"] = rail
                obj.passes = 0
                obj.recent.clear()
                obj._start_stream()
                baseline(obj, 11.0)
                attempt.endstop.get_steppers = lambda: [SignedRamp(11.060, 5000.0, 50.0, 1)]
                acquire_reference(obj, attempt.endstop, 11.060)
                attempt = obj.prepare(attempt.endstop, 11.060)
                obj.arm(attempt)
                trigger.messages.clear()
            stopped = stream_ramp(obj, attempt, parked=True)
            self.assertTrue(attempt.varied)
            self.assertEqual(attempt.reason, "no_motion")
            self.assertEqual(trigger.messages, [[HOST]])
            self.assertLess(stopped - attempt.arm_time, 0.065)
            self.assertLess(stopped, attempt.deadline - 6.0)
            obj.disarm()
            self.assertFalse(attempt.record["motion_seen"])
            with self.assertRaisesRegex(CommandError, "motion was not verified"):
                obj.finish(attempt.endstop, attempt, HOST)

    def test_positive_negative_and_delayed_real_ramps_are_not_false_stopped(self):
        for speed, accel in ((50.0, 5000.0), (20.0, 100.0), (30.0, 5000.0)):
            for direction in (-1, 1):
                with self.subTest(speed=speed, accel=accel, direction=direction):
                    obj, attempt, trigger = prepared_generic(speed=speed, accel=accel,
                                                              direction=direction)
                    stream_ramp(obj, attempt, duration=0.300, direction=direction,
                                response_lag=0.010, age=0.014)
                    self.assertIsNone(attempt.reason)
                    self.assertEqual(trigger.messages, [])
                    self.assertTrue(attempt.motion_seen)

    def test_admitted_75_positive_and_negative_ramps_prove_motion(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                obj, attempt, trigger = prepared_fast_generic(direction)
                stream_ramp(obj, attempt, direction=direction, age=0.0001)
                self.assertEqual(attempt.effective_speed, 75.0)
                self.assertEqual(attempt.effective_accel, 5360.0)
                self.assertTrue(attempt.motion_seen)
                self.assertIsNone(attempt.reason)
                self.assertEqual(trigger.messages, [])
                self.assertIsNone(obj.sensor_fault)

    def test_admitted_75_parked_noisy_traces_still_stop_within_request_budget(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                obj, attempt, trigger = prepared_fast_generic(direction)
                stopped = stream_ramp(obj, attempt, direction=direction, parked=True, age=0.0001)
                timeout = obj.backend.motion_startup_timeout(75.0, 5360.0)
                self.assertTrue(attempt.varied)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(trigger.messages, [[HOST]])
                self.assertLessEqual(stopped - attempt.arm_time, timeout + 0.003)
                commanded_travel = 75.0 * (stopped - attempt.arm_time) - 0.5 * 75.0 ** 2 / 5360.0
                self.assertLessEqual(commanded_travel, 3.0)
                self.assertIsNone(obj.sensor_fault)

    def test_one_mm_s_uses_full_proof_history_with_19ms_sample_age(self):
        for direction in (-1, 1):
            obj, attempt, trigger = prepared_generic(
                speed=1.0, accel=1000.0, direction=direction, options={"stream_interval": 0.001})
            stream_ramp(obj, attempt, duration=0.150, direction=direction,
                        age=0.019, size=3)
            self.assertIsNone(attempt.reason)
            self.assertEqual(trigger.messages, [])
            self.assertTrue(attempt.motion_seen)
            self.assertAlmostEqual(obj.backend.motion_startup_timeout(1.0, 1000.0), 0.116)

    def test_effective_velocity_and_lower_policy_cap_qualify_motion(self):
        for speed, velocity, options, expected in (
                (50.0, 20.0, {}, 20.0),
                (25.0, 500.0, {"max_report_travel": 0.5, "max_sample_age": 0.020}, 25.0)):
            obj, attempt, trigger = prepared_generic(speed=speed, velocity=velocity, options=options)
            self.assertEqual(attempt.effective_speed, expected)
            obj.backend.validate_rail(obj.session_axes["x"].get_homing_info(), CommandError)
            stream_ramp(obj, attempt)
            self.assertEqual(trigger.messages, [])
            self.assertTrue(attempt.motion_seen)

    def test_low_acceleration_rejected_before_enable_or_backoff(self):
        obj = generic(options={"homing_accel": 100.0, "max_startup_travel": 3.0})
        rail = FakeRail(FakeEndstop("x"), info(speed=50, retract=0))
        obj.endstops["x"] = rail.endstop
        calls = []
        obj._limit_accel = lambda: calls.append("limit")
        obj._enable_motors = lambda axis: calls.append("enable")
        obj._back_off_others = lambda axis: calls.append("backoff")
        with self.assertRaisesRegex(CommandError, "3.0 mm policy"):
            obj._handle_rails_begin(None, [rail])
        self.assertEqual(calls, [])
        self.assertEqual(obj.toolhead.moves, [])

    def test_prepare_rechecks_budget_before_dispatch(self):
        obj, attempt, _trigger = prepared_generic(options={"max_startup_travel": 3.0, "max_sample_age": 0.020})
        obj.disarm()
        obj.toolhead.get_max_velocity = lambda: (500.0, 100.0)
        with self.assertRaisesRegex(CommandError, "host stop request"):
            obj.prepare(attempt.endstop, 11.0)

    def test_default_deadline_and_nominal_command_request_budget_are_explicit(self):
        obj, _attempt, _trigger = prepared_generic()
        status = obj.backend.get_status()
        self.assertAlmostEqual(status["startup_motion_timeout_s"], 0.085)
        self.assertAlmostEqual(status["startup_motion_request_travel_mm"], 4.15)
        self.assertIsNone(status["startup_motion_request_limit_mm"])
        self.assertFalse(status["latency_guaranteed"])


class MotionPolicyPreflightTests(unittest.TestCase):
    def machine(self, speeds=(50.0,), options=None, accel=20000.0, connect=True):
        obj = generic(options=options, connect=False)
        axes = "xy"[:len(speeds)]
        endstops = with_endstops(obj, axes)
        rails = [FakeRail(endstops[axis], info(speed=speed, retract=0), "stepper_" + axis)
                 for axis, speed in zip(axes, speeds)]
        toolhead = obj.fake_printer.objects["toolhead"]
        toolhead.accel = accel
        toolhead.kin = types.SimpleNamespace(rails=rails)
        if connect:
            obj.fake_printer.send_event("klippy:connect")
        return obj, toolhead, rails

    def assert_no_effects(self, obj, toolhead):
        self.assertFalse(obj.homing_active)
        self.assertFalse(obj.stream_on)
        self.assertFalse(obj.backend.active)
        self.assertEqual(toolhead.moves, [])
        self.assertEqual(toolhead.dwells, [])
        self.assertEqual(toolhead.accel_log, [])
        self.assertIsNone(obj.saved_accel)
        self.assertEqual(obj.reactor.monotonic(), 0.0)
        self.assertEqual(obj.fake_printer.scripts, [])
        self.assertTrue(all(not endstop._dispatch.started for endstop in obj.endstops.values()))

    def test_static_inadmissible_plan_is_rejected_at_connect_without_motion(self):
        obj, toolhead, _rails = self.machine(accel=1500.0, options={"max_startup_travel": 3.0, "max_sample_age": 0.020}, connect=False)
        with self.assertRaisesRegex(ConfigError, "3.23 mm"):
            obj.fake_printer.send_event("klippy:connect")
        self.assert_no_effects(obj, toolhead)

    def test_runtime_preflight_rejects_lower_live_accel_before_any_effect(self):
        obj, toolhead, _rails = self.machine(options={"max_startup_travel": 3.0, "max_sample_age": 0.020})
        toolhead.accel = 1500.0
        with self.assertRaisesRegex(CommandError, "effective homing speed 50 mm/s and acceleration 1500"):
            obj.preflight_motion_policy("x")
        self.assert_no_effects(obj, toolhead)
        self.assertEqual(toolhead.accel, 1500.0)

    def test_preflight_uses_live_velocity_and_does_not_change_it(self):
        obj, toolhead, _rails = self.machine()
        toolhead.get_max_velocity = lambda: (20.0, 1500.0)
        obj.preflight_motion_policy("x")
        self.assertEqual(toolhead.get_max_velocity(), (20.0, 1500.0))
        self.assert_no_effects(obj, toolhead)

    def test_configured_homing_accel_cap_and_zero_are_respected(self):
        for cap, live, expected in ((1500.0, 20000.0, 1500), (0.0, 1500.0, 1500)):
            with self.subTest(cap=cap):
                obj, toolhead, rails = self.machine(speeds=(30.0,), options={"homing_accel": cap, "max_startup_travel": 3.0})
                toolhead.accel = live
                rails[0].info = info(speed=50.0, retract=0)
                with self.assertRaisesRegex(CommandError, "acceleration %s mm/s" % expected):
                    obj.preflight_motion_policy("x")
                self.assert_no_effects(obj, toolhead)
                self.assertEqual(toolhead.accel, live)

    def test_all_selected_xy_plans_are_checked_and_z_is_ignored(self):
        obj, toolhead, rails = self.machine(speeds=(30.0, 30.0), options={"max_startup_travel": 3.0, "max_sample_age": 0.020})
        toolhead.accel = 1500.0
        rails[1].info = info(speed=50.0, retract=0)
        obj.preflight_motion_policy("xz")
        with self.assertRaisesRegex(CommandError, "3.23 mm"):
            obj.preflight_motion_policy("xy")
        self.assert_no_effects(obj, toolhead)

    def test_runtime_75_preserves_speed_and_requires_explicit_compatible_limits(self):
        obj, toolhead, rails = self.machine(
            speeds=(75.0,), accel=5360.0,
            options={"homing_accel": 0.0, "max_sample_age": 0.005, "stream_interval": 0.001,
                     "max_report_travel": 1.0, "max_startup_travel": 3.0})
        self.assertEqual(obj.backend.max_speed, 200.0)
        obj.preflight_motion_policy("x")
        self.assertLess(obj.backend.motion_startup_request_travel(75.0, 5360.0), 3.0)
        self.assertEqual(rails[0].get_homing_info().speed, 75.0)
        self.assert_no_effects(obj, toolhead)
        # The unchanged 3 mm check still rejects this same configured speed
        # when a later command lowers global acceleration to 5000.
        toolhead.accel = 5000.0
        with self.assertRaisesRegex(CommandError, "needs 3.04 mm"):
            obj.preflight_motion_policy("x")
        self.assertEqual(rails[0].get_homing_info().speed, 75.0)
        self.assertEqual(toolhead.accel, 5000.0)
        self.assert_no_effects(obj, toolhead)

    def test_missing_selected_rail_fails_with_the_requested_error_type(self):
        obj, toolhead, _rails = self.machine()
        del obj.rails_by_axis["x"]
        with self.assertRaisesRegex(ConfigError, "axis X must use"):
            obj.preflight_motion_policy("x", error=ConfigError)
        self.assert_no_effects(obj, toolhead)

    def test_legacy_beacon_preflight_preserves_its_existing_motion_policy(self):
        obj = make_homing(connect=False)
        obj.preflight_motion_policy("xy")
        self.assertIsNone(obj.toolhead)
        self.assertEqual(obj.fake_printer.objects["toolhead"].moves, [])


class MotionPolicyDiagnosticTests(unittest.TestCase):
    def test_reported_three_point_two_three_budget_has_effective_values_and_a_safe_speed(self):
        obj = generic(options={"max_report_travel": 1.0, "max_startup_travel": 3.0, "max_sample_age": 0.020})
        budget = obj.backend.motion_startup_request_travel(50.0, 1500.0)
        self.assertAlmostEqual(budget, 3.2333333333333334)
        maximum = obj.backend.motion_startup_speed_limit(1500.0)
        self.assertGreater(maximum, 47.0)
        self.assertLess(maximum, 48.0)
        with self.assertRaises(CommandError) as caught:
            obj.backend.validate_motion_plan(50.0, 1500.0, CommandError)
        message = str(caught.exception)
        for text in ("3.23 mm", "3.0 mm policy", "speed 50 mm/s", "acceleration 1500 mm/s^2",
                     "at most %.6g mm/s" % maximum, "not a guarantee of physical stopping travel"):
            self.assertIn(text, message)
        obj.backend.validate_motion_plan(maximum, 1500.0, CommandError)

    def test_speed_limit_is_admissible_across_proof_windows_and_extreme_acceleration(self):
        for options in ({"max_sample_age": 0.005, "stream_interval": 0.001}, {},
                        {"max_sample_age": 0.050, "stream_interval": 0.020}):
            obj = generic(options={"max_report_travel": 1.0, "max_startup_travel": 3.0, **options}, connect=False)
            for accel in (5e-324, 0.001, 0.1, 1.0, 100.0, 1500.0, 5000.0, 1e308):
                with self.subTest(options=options, accel=accel):
                    maximum = obj.backend.motion_startup_speed_limit(accel)
                    self.assertTrue(math.isfinite(maximum))
                    self.assertGreater(maximum, 0.0)
                    self.assertLessEqual(maximum, obj.backend.max_speed)
                    obj.backend.validate_motion_plan(maximum, accel, CommandError)
                    if maximum * 1.00002 < obj.backend.max_speed:
                        with self.assertRaisesRegex(CommandError, "3.0 mm policy"):
                            obj.backend.validate_motion_plan(maximum * 1.00002, accel, CommandError)

    def test_unbounded_derived_travel_fails_closed(self):
        obj = generic()
        with self.assertRaisesRegex(CommandError, "derived startup timeout and nominal travel must be finite"):
            obj.backend.validate_motion_plan(50.0, 5e-324, CommandError)

    def test_nonfinite_or_nonpositive_live_limits_are_rejected(self):
        obj = generic()
        for limits in ((float("nan"), 5000.0), (float("inf"), 5000.0), (0.0, 5000.0),
                       (500.0, float("nan")), (500.0, float("inf")), (500.0, 0.0)):
            with self.subTest(limits=limits):
                obj.toolhead.get_max_velocity = lambda: limits
                with self.assertRaisesRegex(CommandError, "max_velocity/max_accel"):
                    obj.backend.validate_rail(info(speed=50.0, retract=0), CommandError)


class ConfigurableStartupBudgetTests(unittest.TestCase):
    """Explicit modeled budgets do not qualify a physical stopping distance."""
    options = {"max_report_travel": 1.5, "max_startup_travel": 4.5, "max_sample_age": 0.020}

    def test_explicit_legacy_limits_refuse_75_with_complete_unchanged_settings(self):
        obj, toolhead, rails = MotionPolicyPreflightTests().machine(
            speeds=(75.0,), accel=5000.0,
            options={"max_report_travel": 1.0, "max_startup_travel": 3.0, "max_sample_age": 0.020}, connect=False)
        with self.assertRaises(ConfigError) as caught:
            obj.fake_printer.send_event("klippy:connect")
        message = str(caught.exception)
        for part in ("exceeds 50.0", "effective homing speed 75 mm/s", "acceleration 5000 mm/s^2",
                     "report needs 1.5 mm", "max_sample_age 0.02 s", "max_report_travel 1 mm",
                     "startup needs 4.1625 mm", "max_startup_travel 3 mm", "stream_interval 0.005 s",
                     "speed and acceleration were not changed", "not a guarantee of physical stopping travel"):
            self.assertIn(part, message)
        self.assertEqual(rails[0].info.speed, 75.0)
        self.assertEqual(toolhead.accel, 5000.0)
        self.assertEqual(toolhead.moves, [])
        self.assertEqual(toolhead.accel_log, [])
        self.assertEqual(obj.backend.startup_travel, 3.0)
        self.assertEqual(obj.backend.report_travel, 1.0)

    def test_explicit_75_budgets_preserve_timing_and_allow_real_motion(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                obj, attempt, trigger = prepared_generic(speed=75.0, options=self.options,
                                                          direction=direction)
                self.assertEqual(attempt.effective_speed, 75.0)
                self.assertEqual(attempt.effective_accel, 5000.0)
                self.assertEqual(obj.backend.max_age, 0.020)
                self.assertEqual(obj.backend.interval, 0.005)
                self.assertAlmostEqual(obj.backend.motion_startup_timeout(75.0, 5000.0), 0.060)
                self.assertAlmostEqual(obj.backend.motion_startup_request_travel(75.0, 5000.0), 4.1625)
                stream_ramp(obj, attempt, direction=direction)
                self.assertTrue(attempt.motion_seen)
                self.assertIsNone(attempt.reason)
                self.assertEqual(trigger.messages, [])

    def test_explicit_75_parked_request_keeps_deadline_and_configured_budget(self):
        obj, attempt, trigger = prepared_generic(speed=75.0, options=self.options)
        stopped = stream_ramp(obj, attempt, parked=True)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(trigger.messages, [[HOST]])
        elapsed = stopped - attempt.arm_time
        self.assertLessEqual(elapsed, obj.backend.motion_startup_timeout(75.0, 5000.0) + 0.003)
        travel = 75.0 * elapsed - 0.5 * 75.0 ** 2 / 5000.0
        self.assertLessEqual(travel, obj.backend.startup_travel)

    def test_same_75_budgets_refuse_lower_live_accel_before_workflow_effects(self):
        obj, toolhead, rails = MotionPolicyPreflightTests().machine(
            speeds=(75.0,), accel=5000.0, options=self.options)
        toolhead.accel = 1500.0
        with self.assertRaises(CommandError) as caught:
            obj.preflight_motion_policy(["x"])
        message = str(caught.exception)
        for part in ("startup needs 5.475 mm", "max_startup_travel 4.5 mm",
                     "speed 75 mm/s", "acceleration 1500 mm/s^2"):
            self.assertIn(part, message)
        self.assertEqual(toolhead.accel, 1500.0)
        self.assertEqual(toolhead.accel_log, [])
        self.assertEqual(toolhead.moves, [])
        self.assertEqual(rails[0].info.speed, 75.0)
        self.assertFalse(obj.backend.active)

    def test_budget_bounds_and_nonfinite_values_are_rejected(self):
        for value in (0.0, 0.099999, 5.000001, float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                generic(options={"max_startup_travel": value}, connect=False)
        for value in (0.1, 3.0, 4.5, 5.0):
            obj = generic(options={"max_startup_travel": value}, connect=False)
            self.assertEqual(obj.backend.startup_travel, value)
            self.assertEqual(obj.backend.get_status()["startup_motion_request_limit_mm"], value)
            self.assertAlmostEqual(obj.backend.motion_startup_timeout(75.0, 5000.0), 0.090)

    def test_exact_startup_and_report_boundaries_match_strict_installer_policy(self):
        required = 4.1625
        for configured, accepted in ((required - 0.0000001, False),
                                     (required, True), (required + 0.0000001, True)):
            obj = generic(options={"max_report_travel": 1.5, "max_startup_travel": configured, "max_sample_age": 0.020})
            if accepted:
                obj.backend.validate_motion_plan(75.0, 5000.0, CommandError)
            else:
                with self.assertRaises(CommandError):
                    obj.backend.validate_motion_plan(75.0, 5000.0, CommandError)
        for configured, accepted in ((1.5 - 0.000000001, False), (1.5, True)):
            obj = generic(options={"max_report_travel": configured, "max_startup_travel": 4.5, "max_sample_age": 0.020})
            if accepted:
                obj.backend.validate_motion_plan(75.0, 5000.0, CommandError)
            else:
                with self.assertRaisesRegex(CommandError, "sample-age/report-travel"):
                    obj.backend.validate_motion_plan(75.0, 5000.0, CommandError)

    def test_speed_ceiling_uses_each_configured_budget_at_extreme_accelerations(self):
        for budget in (0.1, 3.0, 4.5, 5.0):
            obj = generic(options={"max_report_travel": 2.0, "max_startup_travel": budget})
            for accel in (5e-324, 1500.0, 5000.0, 1e308):
                with self.subTest(budget=budget, accel=accel):
                    maximum = obj.backend.motion_startup_speed_limit(accel)
                    self.assertTrue(math.isfinite(maximum))
                    self.assertGreater(maximum, 0.0)
                    self.assertLessEqual(maximum, obj.backend.max_speed)
                    obj.backend.validate_motion_plan(maximum, accel, CommandError)
                    # The diagnostic includes a fixed 1 micrometre interior
                    # margin; a 0.1% overshoot also clears it at the 0.1 mm cap.
                    if maximum * 1.001 < obj.backend.max_speed:
                        with self.assertRaises(CommandError):
                            obj.backend.validate_motion_plan(maximum * 1.001, accel, CommandError)

    def test_final_prepare_rechecks_reporting_even_when_startup_would_fit(self):
        obj, attempt, _trigger = prepared_generic(speed=75.0, options=self.options)
        obj.disarm()
        obj.session_axes["x"].info = info(speed=80.0, second=80.0, retract=0)
        self.assertLess(obj.backend.motion_startup_request_travel(80.0, 5000.0), 4.5)
        with self.assertRaisesRegex(CommandError, "exceeds 75.0"):
            obj.prepare(attempt.endstop, 11.0)

    def test_second_pass_speed_uses_both_configured_limits(self):
        obj = generic(options=self.options)
        obj.backend.validate_rail(info(speed=50.0, second=75.0, retract=5.0), CommandError)
        plain = generic(options={"max_report_travel": 1.0, "max_startup_travel": 3.0, "max_sample_age": 0.020})
        with self.assertRaisesRegex(CommandError, "speed 75 mm/s"):
            plain.backend.validate_rail(info(speed=50.0, second=75.0, retract=5.0), CommandError)

    def test_status_and_console_expose_configured_nominal_limits(self):
        obj = generic(options=self.options)
        status = obj.backend.get_status()
        self.assertEqual(status["startup_motion_request_limit_mm"], 4.5)
        self.assertEqual(status["max_report_travel_mm"], 1.5)
        self.assertEqual(status["max_homing_speed"], 75.0)
        self.assertFalse(status["latency_guaranteed"])
        messages = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=messages.append))
        self.assertTrue(any("max_report_travel 1.500 mm, max_startup_travel 4.500 mm" in line
                            and "not physical stopping distances" in line for line in messages))
        self.assertTrue(any("report-age speed ceiling 75.0 mm/s" in line for line in messages))


class OptionalTravelPolicyTests(unittest.TestCase):
    """Configured speeds have no default plugin travel ceiling; live guards remain."""

    def test_absent_policies_accept_75_and_100_connect_and_preflight_without_edits(self):
        obj, toolhead, rails = MotionPolicyPreflightTests().machine(speeds=(75.0, 100.0), accel=5000.0)
        obj.preflight_motion_policy("xy")
        self.assertEqual([rail.info.speed for rail in rails], [75.0, 100.0])
        self.assertIsNone(obj.backend.report_travel)
        self.assertIsNone(obj.backend.startup_travel)
        self.assertIsNone(obj.backend.max_speed)
        self.assertIsNone(obj.backend.motion_startup_speed_limit(5000.0))
        self.assertEqual(obj.backend.max_age, 0.050)
        self.assertEqual(obj.backend.interval, 0.005)
        MotionPolicyPreflightTests().assert_no_effects(obj, toolhead)

    def test_absent_policies_prepare_real_75_and_100_in_both_directions(self):
        for speed in (75.0, 100.0):
            for direction in (-1, 1):
                with self.subTest(speed=speed, direction=direction):
                    obj, attempt, trigger = prepared_generic(speed=speed, direction=direction)
                    self.assertEqual(attempt.effective_speed, speed)
                    self.assertEqual(attempt.effective_accel, 5000.0)
                    self.assertEqual(obj.session_axes["x"].info.speed, speed)
                    stream_ramp(obj, attempt, direction=direction)
                    self.assertTrue(attempt.motion_seen)
                    self.assertIsNone(attempt.reason)
                    self.assertEqual(trigger.messages, [])

    def test_absent_policies_parked_75_and_100_still_stop_at_same_deadline(self):
        for speed in (75.0, 100.0):
            for direction in (-1, 1):
                with self.subTest(speed=speed, direction=direction):
                    obj, attempt, trigger = prepared_generic(speed=speed, direction=direction)
                    stopped = stream_ramp(obj, attempt, parked=True)
                    self.assertFalse(attempt.motion_seen)
                    self.assertEqual(attempt.reason, "no_motion")
                    self.assertEqual(trigger.messages, [[HOST]])
                    self.assertLessEqual(stopped - attempt.arm_time,
                                         obj.backend.motion_startup_timeout(speed, 5000.0) + 0.003)
                    with self.assertRaisesRegex(CommandError, "motion was not verified"):
                        obj.finish(attempt.endstop, attempt, HOST)

    def test_each_explicit_policy_is_enforced_independently(self):
        for name in ("max_report_travel", "max_startup_travel"):
            for invalid in (None, "None"):
                with self.subTest(option=name, invalid=invalid), self.assertRaises((TypeError, ValueError)):
                    generic(options={name: invalid})
        report_only = generic(options={"max_report_travel": 1.0})
        self.assertIsNone(report_only.backend.startup_travel)
        with self.assertRaisesRegex(CommandError, "sample-age/report-travel"):
            report_only.backend.validate_motion_plan(75.0, 5000.0, CommandError)
        self.assertEqual(report_only.backend.motion_startup_speed_limit(5000.0), 20.0)
        startup_only = generic(options={"max_startup_travel": 3.0})
        self.assertIsNone(startup_only.backend.max_speed)
        with self.assertRaisesRegex(CommandError, "3.0 mm policy"):
            startup_only.backend.validate_motion_plan(75.0, 5000.0, CommandError)
        maximum = startup_only.backend.motion_startup_speed_limit(5000.0)
        self.assertGreater(maximum, 36.0)
        self.assertLess(maximum, 37.0)
        startup_only.backend.validate_motion_plan(maximum, 5000.0, CommandError)
        generic(options={"max_startup_travel": 4.5}).backend.validate_motion_plan(50.0, 5000.0, CommandError)
        generic(options={"max_report_travel": 1.5}).backend.validate_motion_plan(30.0, 1500.0, CommandError)

    def test_startup_only_ceiling_is_bounded_without_reporting_cap(self):
        for accel in (5e-324, 100.0, 5000.0, 1e308):
            obj = generic(options={"max_startup_travel": 3.0})
            maximum = obj.backend.motion_startup_speed_limit(accel)
            self.assertTrue(math.isfinite(maximum))
            self.assertGreater(maximum, 0.0)
            obj.backend.validate_motion_plan(maximum, accel, CommandError)

    def test_disabled_status_is_json_safe_and_console_explicit(self):
        obj, attempt, _trigger = prepared_generic(speed=100.0)
        status = obj.get_status(obj.reactor.now)
        backend = status["backend"]
        self.assertIsNone(backend["max_homing_speed"])
        self.assertIsNone(backend["max_report_travel_mm"])
        self.assertIsNone(backend["startup_motion_request_limit_mm"])
        self.assertEqual(backend["startup_motion_evaluated_speed_mm_s"], 100.0)
        self.assertAlmostEqual(backend["startup_motion_timeout_s"], 0.095)
        json.dumps(status, allow_nan=False)
        messages = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=messages.append))
        self.assertTrue(any("report-age speed ceiling disabled" in text for text in messages))
        self.assertTrue(any("max_report_travel disabled, max_startup_travel disabled" in text for text in messages))
        attempt.effective_speed = attempt.effective_accel = None  # Legacy detector replay has no plan.
        speed, accel, timeout = obj._motion_startup_plan(attempt)
        self.assertEqual(speed, 500.0)
        self.assertEqual(accel, 5000.0)
        self.assertTrue(math.isfinite(timeout))
        self.assertIsNone(obj._motion_state(attempt))

    def test_disabled_policies_still_reject_nonfinite_derived_timeout_or_travel(self):
        obj = generic()
        for speed, accel in ((50.0, 5e-324), (1.7e308, 5e307)):
            with self.subTest(speed=speed, accel=accel), self.assertRaisesRegex(
                    CommandError, "derived startup timeout and nominal travel must be finite"):
                obj.backend.validate_motion_plan(speed, accel, CommandError)
        with self.assertRaisesRegex(CommandError, "derived homing deadline must be finite"):
            prepared_generic(speed=1e-320)
        obj.homing_accel = 0.0
        for speed, accel in ((float("nan"), 5000.0), (1.7e308, 5e307)):
            obj.toolhead.get_max_velocity = lambda: (speed, accel)
            status = obj.get_status(obj.reactor.now)
            json.dumps(status, allow_nan=False)
            self.assertIsNone(status["backend"]["startup_motion_request_travel_mm"])

    def test_absent_policies_do_not_disable_missing_stale_or_frozen_faults(self):
        for fault in ("no_data", "stale_data", "frozen"):
            with self.subTest(fault=fault):
                obj, attempt, trigger = prepared_generic(speed=75.0)
                if fault == "no_data":
                    obj.reactor.now = max(attempt.arm_time, attempt.last_data) + obj.stream_timeout + 0.001
                    obj._watchdog(obj.reactor.now)
                elif fault == "stale_data":
                    feed(obj, [(attempt.arm_time + number / 3200.0, 0.0, 0.0, G)
                               for number in range(16)], age=0.050)
                else:
                    while not attempt.done and obj.reactor.now < attempt.arm_time + 0.060:
                        first = obj.backend.last_sample_time + 1 / 3200.0
                        feed(obj, [(first + number / 3200.0, 0.0, 0.0, G) for number in range(16)])
                        obj._watchdog(obj.reactor.now)
                self.assertEqual(attempt.reason, fault)
                self.assertEqual(trigger.messages, [[HOST]])
                self.assertFalse(attempt.motion_seen)

    def test_firmware_velocity_cap_is_preserved_without_editing_rail_speed(self):
        obj, attempt, _trigger = prepared_generic(speed=1000.0, velocity=100.0)
        self.assertEqual(obj.session_axes["x"].info.speed, 1000.0)
        self.assertEqual(attempt.effective_speed, 100.0)
        self.assertEqual(obj.toolhead.get_max_velocity()[0], 100.0)


class BeaconSlowMotionProofTests(unittest.TestCase):
    def test_slow_beacon_motion_is_proved_without_altering_legacy_pressed_gate(self):
        for speed in (20.0, 30.0):
            for direction in (-1, 1):
                obj = make_homing(impact_sign=0)
                for start in range(0, 96, 8):
                    obj._inspect(batch(start, [(0, 0, counts(1.0))] * 8))
                endstop = FakeEndstop("x")
                endstop.get_steppers = lambda: [SignedRamp(0.2, 5000.0, speed, direction)]
                obj.session_axes["x"] = FakeRail(endstop, info(speed=speed, retract=0, second=speed))
                acquire_reference(obj, endstop, 0.2)
                attempt = obj.prepare(endstop, 0.2)
                obj.arm(attempt)
                for start in range(200, 296, 8):
                    data = []
                    for offset in range(8):
                        elapsed = (start + offset) / 1000.0 - attempt.arm_time
                        accel = direction * 5000.0 if 0.0 <= elapsed < speed / 5000.0 else 0.0
                        data.append((counts(accel / G) + offset % 2, 0, counts(1.0)))
                    obj._inspect(batch(start, data))
                    obj._check_pressed(attempt)
                self.assertTrue(attempt.motion_seen)
                self.assertFalse(attempt.moving)  # Legacy pressed gate remains40mm/s.
                self.assertEqual(endstop.trigger_log.messages, [])
                obj._inspect(batch(296, [(counts(-12 * direction), 0, counts(1.0))] * 8))
                obj.disarm()
                self.assertEqual(obj.finish(endstop, attempt, HIT), 0.296)
                self.assertFalse(attempt.record["moving"])
                self.assertTrue(attempt.record["motion_seen"])


class NativeExpandedReferenceTests(unittest.TestCase):
    """Native raw Beacon traces; every sensor, stepper and clock is synthetic."""

    rate = 4000.0
    clock_hz = 1000000.0
    low = (-23, -27, 101)
    high = (24, 28, 155)

    def machine(self, axis="x", threshold=0.5, speed=75.0, accel=5000.0,
                asymmetric=False, quiet=None, impact_sign=0, epoch=10.0):
        obj = make_homing(threshold=threshold, impact_sign=impact_sign,
                          accel=accel, options={"homing_accel": 0})
        obj.sensor_clock.print_time_to_clock = lambda when: round(when * self.clock_hz)
        obj.sensor_clock.seconds_to_clock = lambda interval: interval * self.clock_hz
        def clock_to_time(clock):
            current = round(obj.reactor.now * self.clock_hz)
            return (current + ((clock - current + 0x80000000) & 0xFFFFFFFF)
                    - 0x80000000) / self.clock_hz
        obj.clock_to_time = clock_to_time
        obj.beacon_freq = self.clock_hz
        obj.impact_window = round(plugin.IMPACT_WINDOW * self.clock_hz)
        endstop = FakeEndstop(axis)
        obj.session_axes[axis] = FakeRail(endstop, info(speed=speed, second=speed, retract=0))
        obj.reactor.now = epoch

        def stationary():
            now = obj.reactor.now
            rows = []
            size = max(2, round(self.rate * 0.005))
            for number in range(size):
                high = number % (10 if asymmetric else 2) == 0
                rows.append((self.high if high else self.low) if quiet is None
                            else tuple(counts(value) for value in quiet))
            obj._inspect(batch(round((now - (size - 1) / self.rate) * self.clock_hz), rows,
                               delta_clock=round((size - 1) / self.rate * self.clock_hz),
                               receive_time=now))

        obj.reactor.on_pause = stationary
        obj._prepare_reference()
        return obj, endstop

    def prepare(self, obj, endstop, direction=1, step_delay=0.0):
        start = obj.reactor.now + 0.005
        speed = obj.session_axes[endstop.axis].info.speed
        accel = obj.toolhead.accel
        endstop.get_steppers = lambda: [SignedRamp(start + step_delay, accel, speed, direction)]
        attempt = obj.prepare(endstop, start)
        obj.arm(attempt)
        return attempt

    def send(self, obj, first, rows, age=0.001, interval=None):
        interval = 1 / self.rate if interval is None else interval
        end = first + (len(rows) - 1) * interval
        obj.reactor.now = end + age
        obj._inspect(batch(round(first * self.clock_hz), rows,
                           delta_clock=round((len(rows) - 1) * interval * self.clock_hz),
                           receive_time=obj.reactor.now))

    def stream(self, obj, attempt, duration=0.125, direction=1, parked=False,
               biased=False, sample_offset=0.0, watchdog_phase=0.0, frozen=False,
               step_delay=0.0, sensor_lag=0.0, size=10, acceleration=None):
        next_watchdog = attempt.arm_time + watchdog_phase
        index = 0
        while index / self.rate < duration and not attempt.done:
            first = attempt.arm_time + sample_offset + index / self.rate
            rows = []
            for offset in range(size):
                number = index + offset
                when = first + offset / self.rate
                if frozen:
                    row = [round(value) for value in attempt.bases]
                elif biased:
                    row = [value - number % 2 for value in self.high]
                else:
                    row = list(self.high if number % 2 else self.low)
                if acceleration is not None:
                    row[attempt.index] += counts(acceleration(when - attempt.arm_time) / G)
                elif (not parked and 0.0 <= when - attempt.arm_time - step_delay - sensor_lag
                        < attempt.effective_speed / attempt.effective_accel):
                    row[attempt.index] += counts(direction * attempt.effective_accel / G)
                rows.append(row)
            receive = first + (size - 1) / self.rate + 0.001
            while next_watchdog < receive and not attempt.done:
                obj.reactor.now = next_watchdog
                obj._watchdog(next_watchdog)
                next_watchdog += plugin.EARLY_INTERVAL
            if not attempt.done:
                self.send(obj, first, rows)
            index += size

    def test_reported_stationary_xyz_noise_admits_half_g_without_changing_trigger(self):
        for axis in ("x", "y"):
            with self.subTest(axis=axis):
                obj, endstop = self.machine(axis=axis)
                attempt = self.prepare(obj, endstop)
                status = obj.baseline_status
                self.assertEqual(status["quiet_limit_g"], 0.5)
                self.assertEqual(status["observed_peak_to_peak_g"],
                                 {"x": 0.367188, "y": 0.429688, "z": 0.421875})
                self.assertGreaterEqual(status["quiet_samples"], 50)
                self.assertGreaterEqual(status["quiet_span_ms"], 100.0)
                self.assertEqual(status["quiet_resets"], 0)
                self.assertEqual(attempt.limit_raw * attempt.count_g, 0.5)
                self.assertIsNotNone(attempt.proof_noise)
                self.assertAlmostEqual(attempt.motion_deadline - attempt.arm_time, 0.09)
                self.assertEqual(endstop.trigger_log.messages, [])
                self.assertEqual(obj.toolhead.accel_log, [])
                self.assertEqual(obj.toolhead.moves, [])

    def test_native_floor_caps_small_manual_thresholds_and_preserves_large_thresholds(self):
        obj = make_homing()
        for threshold, expected in ((0.000001, 0.000001), (0.1, 0.1), (0.5, 0.5),
                                    (1.0, 0.5), (2.5, 0.5), (3.0, 0.6), (5.0, 1.0)):
            obj.threshold_g["x"] = threshold
            self.assertAlmostEqual(obj._reference_noise_limit("x"), expected)
        with self.assertRaisesRegex(CommandError, "fresh quiet"):
            self.machine(threshold=0.1)

    def test_every_axis_rejects_stationary_noise_above_native_floor(self):
        for axis in range(3):
            with self.subTest(axis=axis):
                previous_low, previous_high = self.low, self.high
                try:
                    self.low = tuple(-34 if index == axis else value
                                     for index, value in enumerate(previous_low))
                    self.high = tuple(34 if index == axis else value
                                      for index, value in enumerate(previous_high))
                    with self.assertRaisesRegex(CommandError, "fresh quiet"):
                        self.machine()
                finally:
                    self.low, self.high = previous_low, previous_high

    def test_balanced_biased_and_asymmetric_parked_noise_cannot_succeed(self):
        for asymmetric, biased in ((False, False), (False, True), (True, False), (True, True)):
            for axis in ("x", "y"):
                with self.subTest(axis=axis, asymmetric=asymmetric, biased=biased):
                    obj, endstop = self.machine(axis=axis, asymmetric=asymmetric)
                    attempt = self.prepare(obj, endstop)
                    if asymmetric:
                        self.assertGreater(attempt.proof_noise[1],
                                           (self.high[1] - self.low[1]) / 2)
                    self.stream(obj, attempt, parked=True, biased=biased)
                    self.assertFalse(attempt.moving)
                    self.assertFalse(attempt.motion_seen)
                    self.assertEqual(attempt.auto_direction, 0)
                    self.assertEqual(attempt.proof_speed_peak, 0.0)
                    self.assertEqual(attempt.reason, "no_motion")
                    self.assertEqual(endstop.trigger_log.messages, [[HOST]])
                    self.assertLessEqual(obj.reactor.now, attempt.motion_deadline + 0.003)
                    self.assertGreaterEqual(obj.reactor.now, attempt.motion_deadline)
                    with self.assertRaisesRegex(CommandError, "motion was not verified"):
                        obj.finish(endstop, attempt, HOST)

    def test_real_75mm_ramps_and_contacts_survive_batch_watchdog_phases(self):
        for axis in ("x", "y"):
            for direction in (-1, 1):
                for phase in (0.0, 0.001, 0.002):
                    with self.subTest(axis=axis, direction=direction, phase=phase):
                        obj, endstop = self.machine(axis=axis, threshold=1.0)
                        attempt = self.prepare(obj, endstop, direction)
                        self.stream(obj, attempt, direction=direction,
                                    sample_offset=phase / 4, watchdog_phase=phase)
                        self.assertTrue(attempt.motion_seen)
                        self.assertTrue(attempt.moving)
                        self.assertGreaterEqual(attempt.proof_speed_peak, 37.5)
                        self.assertEqual(attempt.auto_direction, -direction)
                        self.assertIsNone(attempt.reason)
                        first = obj.clock_to_time(attempt.last_end_clock) + 1 / self.rate
                        row = [round(value) for value in attempt.bases]
                        row[attempt.index] += counts(-1.1 * direction)
                        self.send(obj, first, [row, row])
                        self.assertEqual(attempt.reason, "impact")
                        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
                        obj.disarm()
                        self.assertAlmostEqual(obj.finish(endstop, attempt, HIT), first, places=6)
                        self.assertEqual(attempt.record["result"], "impact")

    def test_lower_acceleration_without_reliable_motion_proof_aborts_instead_of_pressed(self):
        obj, endstop = self.machine(threshold=1.0, accel=1000.0)
        attempt = self.prepare(obj, endstop)
        self.stream(obj, attempt, duration=0.2)
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_nonfinite_derived_native_deadline_fails_before_arming(self):
        obj, endstop = self.machine(accel=5e-324)
        with self.assertRaisesRegex(CommandError, "deadlines must be finite"):
            self.prepare(obj, endstop)
        self.assertIsNone(obj.attempt)
        self.assertEqual(endstop.trigger_log.messages, [])

    def test_half_g_still_reports_real_startup_acceleration_without_masking_it(self):
        obj, endstop = self.machine()
        attempt = self.prepare(obj, endstop)
        self.stream(obj, attempt)
        self.assertEqual(attempt.reason, "impact")
        self.assertFalse(attempt.motion_seen)
        self.assertAlmostEqual(obj.onset_time(attempt), attempt.arm_time + 1 / self.rate)
        self.assertGreater(attempt.onset_raw * attempt.count_g, 0.5)
        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
        # Workflow quality rejects this startup trigger instead of raising the
        # actual detector threshold or granting calibration a successful home.
        quality, _message = Rig().workflow._quality("x", {
            "result": "impact", "travel_mm": 0.0, "moving": False, "motion_seen": False})
        self.assertNotEqual(quality, "good")

    def test_first_contact_survives_old_prearm_transient_and_single_retained_sample(self):
        for sign in (-1, 1):
            obj, endstop = self.machine()
            attempt = self.prepare(obj, endstop)
            before = [0, 0, counts(1)]
            before[attempt.index] = counts(-15)
            contact = [round(value) for value in attempt.bases]
            contact[attempt.index] += counts(sign * 0.55)
            self.send(obj, attempt.arm_time - 0.010, [before, contact], interval=0.010)
            self.assertEqual(attempt.reason, "impact")
            self.assertAlmostEqual(obj.onset_time(attempt), attempt.arm_time)
            self.assertEqual(endstop.trigger_log.messages, [[HIT]])

    def test_stationary_bias_cannot_choose_direction_or_hide_either_first_contact(self):
        for axis in ("x", "y"):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    obj, endstop = self.machine(axis=axis)
                    attempt = self.prepare(obj, endstop)
                    self.stream(obj, attempt, duration=0.040, parked=True, biased=True)
                    self.assertGreater(obj._measured_speed(attempt), 30.0)
                    self.assertFalse(attempt.motion_seen)
                    self.assertEqual(attempt.auto_direction, 0)
                    self.assertIsNone(attempt.reason)
                    first = obj.clock_to_time(attempt.last_end_clock) + 1 / self.rate
                    contact = [round(value) for value in attempt.bases]
                    contact[attempt.index] += counts(sign * 0.55)
                    self.send(obj, first, [contact, contact])
                    self.assertEqual(attempt.reason, "impact")
                    self.assertEqual(endstop.trigger_log.messages, [[HIT]])
                    obj.disarm()
                    self.assertAlmostEqual(obj.finish(endstop, attempt, HIT), first)
                    self.assertEqual(attempt.record["result"], "impact")

    def test_invalid_expanded_native_frames_cannot_update_motion_peak_or_return_home(self):
        for fault in ("stale", "future", "duplicate", "reverse", "zero_interval", "nonfinite"):
            with self.subTest(fault=fault):
                obj, endstop = self.machine(threshold=1.0)
                attempt = self.prepare(obj, endstop)
                row = [round(value) for value in attempt.bases]
                first = attempt.arm_time
                if fault in ("duplicate", "reverse"):
                    self.send(obj, first, [row] * 10)
                    first -= 1 / self.rate if fault == "reverse" else 0
                if fault == "nonfinite":
                    obj.reactor.now = first + 0.004
                    obj._inspect({"start_clock": round(first * self.clock_hz), "delta_clock": 2250,
                                  "data": tuple([float("nan"), 0, 128] * 10),
                                  "#receive_time": obj.reactor.now})
                else:
                    self.send(obj, first, [row] * 10,
                              age=0.051 if fault == "stale" else -0.010 if fault == "future" else 0.001,
                              interval=0 if fault == "zero_interval" else None)
                self.assertEqual(attempt.reason, "stale_data" if fault in ("stale", "future")
                                 else "invalid_data")
                self.assertEqual(attempt.proof_speed_peak, 0.0)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_gap_invalidates_early_motion_peak_and_noisy_frozen_stream_fails_closed(self):
        obj, endstop = self.machine(threshold=1.0)
        attempt = self.prepare(obj, endstop)
        self.stream(obj, attempt, duration=0.0175, watchdog_phase=0.100)
        self.assertGreaterEqual(attempt.proof_speed_peak, 37.5)
        self.assertFalse(attempt.motion_seen)
        first = obj.clock_to_time(attempt.last_end_clock) + 0.005
        self.send(obj, first, [self.low, self.high] * 5)
        self.assertTrue(attempt.gap)
        obj._watchdog(obj.reactor.now)
        self.assertFalse(attempt.motion_seen)
        obj.reactor.now = attempt.motion_deadline
        attempt.last_data = obj.reactor.now
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])
        obj, endstop = self.machine()
        attempt = self.prepare(obj, endstop)
        self.stream(obj, attempt, parked=True, frozen=True)
        self.assertFalse(attempt.varied)
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_expanded_native_continuity_and_exact_end_duplicates_across_clock_wrap(self):
        epoch = 0x100000000 / self.clock_hz - 0.120
        obj, endstop = self.machine(threshold=1.0, epoch=epoch)
        attempt = self.prepare(obj, endstop)
        self.stream(obj, attempt, duration=0.040)
        self.assertTrue(attempt.motion_seen)
        self.assertTrue(attempt.moving)
        self.assertFalse(attempt.gap)
        self.assertIsNone(attempt.reason)
        self.assertEqual(endstop.trigger_log.messages, [])
        # The last valid batch straddles the 32-bit boundary, so its stored end
        # clock is unmasked while a replay starts at the same masked clock.
        obj, endstop = self.machine(threshold=1.0, epoch=epoch)
        attempt = self.prepare(obj, endstop)
        wrap = 0x100000000 / self.clock_hz
        row = [round(value) for value in attempt.bases]
        self.send(obj, attempt.arm_time, [row] * 37)
        self.send(obj, wrap - 0.0005, [row] * 4)
        self.assertGreater(attempt.last_end_clock, 0xFFFFFFFF)
        peak = attempt.proof_speed_peak
        self.send(obj, obj.clock_to_time(attempt.last_end_clock), [row] * 2)
        self.assertEqual(attempt.reason, "invalid_data")
        self.assertEqual(attempt.proof_speed_peak, peak)
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_current_upper_bound_preserves_justified_expanded_native_pressed_start(self):
        previous_low, previous_high = self.low, self.high
        try:
            self.low, self.high = (0, 0, 121), (1, 0, 135)
            obj, endstop = self.machine(speed=100.0)
            attempt = self.prepare(obj, endstop)
            self.assertIsNotNone(attempt.proof_noise)
            self.stream(obj, attempt, parked=True)
            self.assertTrue(attempt.varied)
            self.assertFalse(attempt.motion_seen)
            self.assertFalse(attempt.moving)
            lower, upper = obj._motion_speed_bounds(attempt, attempt.proof_state)
            commanded = obj._commanded_speed(
                attempt, obj.clock_to_time(attempt.last_end_clock) - plugin.MOTION_LAG)
            self.assertEqual(lower, 0.0)
            self.assertLess(upper, plugin.PRESSED_RATIO * commanded)
            self.assertEqual(attempt.reason, "pressed")
            self.assertEqual(endstop.trigger_log.messages, [[HIT]])
            self.assertEqual(obj.finish(endstop, attempt, HIT), attempt.arm_time)
        finally:
            self.low, self.high = previous_low, previous_high

    def test_quiet_native_pressed_start_and_no_data_guard_remain_effective(self):
        obj, endstop = self.machine(quiet=(0.0, 0.0, 1.0))
        attempt = self.prepare(obj, endstop)
        self.assertIsNone(attempt.proof_noise)
        self.assertIsNone(attempt.motion_deadline)
        for index in range(20):
            first = attempt.arm_time + index * 10 / self.rate
            rows = [[number % 2, 0, counts(1)] for number in range(10)]
            self.send(obj, first, rows)
            obj._watchdog(obj.reactor.now)
            if attempt.done:
                break
        self.assertEqual(attempt.reason, "pressed")
        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
        self.assertEqual(obj.finish(endstop, attempt, HIT), attempt.arm_time)
        obj, endstop = self.machine()
        attempt = self.prepare(obj, endstop)
        obj.reactor.now = attempt.arm_time + obj.stream_timeout + 0.001
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "no_data")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])


class NativeStepEpochTests(unittest.TestCase):
    """Model stock/Kalico arm -> homing dwell -> drip scan dwell -> real steps."""

    def machine(self, rate=3300.0, **options):
        fixture = NativeExpandedReferenceTests()
        fixture.rate = rate
        fixture.low, fixture.high = (-24, -25, 98), (24, 25, 158)
        obj, endstop = fixture.machine(threshold=1.5, speed=100.0, **options)
        return fixture, obj, endstop

    def test_real_step_delays_and_watchdog_phases_keep_original_deadline(self):
        for rate in (3300.0, 4000.0):
            for axis in ("x", "y"):
                for direction in (-1, 1):
                    for delay in (0.0, 0.010, 0.015, 0.030, 0.050):
                        for phase in (0.0, 0.001, 0.002):
                            with self.subTest(rate=rate, axis=axis, direction=direction,
                                              delay=delay, phase=phase):
                                fixture, obj, endstop = self.machine(rate=rate, axis=axis)
                                attempt = fixture.prepare(obj, endstop, direction, step_delay=delay)
                                deadline = attempt.motion_deadline
                                fixture.stream(obj, attempt, direction=direction, step_delay=delay,
                                               watchdog_phase=phase, size=8)
                                self.assertTrue(attempt.motion_seen)
                                self.assertTrue(attempt.moving)
                                self.assertFalse(attempt.gap)
                                self.assertIsNone(attempt.reason)
                                self.assertEqual(attempt.motion_deadline, deadline)
                                self.assertAlmostEqual(deadline - attempt.arm_time, 0.095)
                                self.assertGreaterEqual(attempt.proof_step_epoch,
                                                        attempt.arm_time + delay)
                                # First discrete step follows the motion start;
                                # this search precision is independent of sample rate.
                                self.assertLess(attempt.proof_step_epoch,
                                                attempt.arm_time + delay + 0.003)
                                self.assertEqual(endstop.trigger_log.messages, [])

    def test_sensor_step_skew_is_separate_from_pre_motion_dwell(self):
        for delay in (0.015, 0.030, 0.050):
            for lag in (-0.015, 0.0, 0.015):
                for phase in (0.0, 0.001, 0.002):
                    with self.subTest(delay=delay, lag=lag, phase=phase):
                        fixture, obj, endstop = self.machine(axis="y")
                        attempt = fixture.prepare(obj, endstop, -1, step_delay=delay)
                        fixture.stream(obj, attempt, direction=-1, step_delay=delay,
                                       sensor_lag=lag, watchdog_phase=phase, size=8)
                        self.assertTrue(attempt.motion_seen)
                        self.assertIsNone(attempt.reason)
                        self.assertEqual(endstop.trigger_log.messages, [])

    def test_prearm_missing_sensor_ramp_is_not_reconstructed_as_motion(self):
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop, -1)
        fixture.stream(obj, attempt, direction=-1, sensor_lag=-0.015, size=8)
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_stationary_within_envelope_bias_never_qualifies_a_step_interval(self):
        for biased in (False, True):
            for phase in (0.0, 0.001, 0.002):
                fixture, obj, endstop = self.machine(axis="y")
                attempt = fixture.prepare(obj, endstop, -1, step_delay=0.030)
                fixture.stream(obj, attempt, parked=True, biased=biased,
                               watchdog_phase=phase, size=8)
                self.assertFalse(attempt.motion_seen)
                self.assertAlmostEqual(attempt.proof_interval_peak, 0.0)
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_later_jolts_and_subthreshold_ringing_do_not_create_startup_proof(self):
        for profile in ("late_jolt", "startup_ringing"):
            fixture, obj, endstop = self.machine(axis="y")
            delay = 0.015
            attempt = fixture.prepare(obj, endstop, -1, step_delay=delay)
            def acceleration(elapsed):
                since_steps = elapsed - delay
                if profile == "late_jolt" and 0.050 <= since_steps < 0.062:
                    return -9806.65
                if profile == "startup_ringing" and 0.0 <= since_steps < 0.025:
                    return 0.8 * G * (1 if int(since_steps / 0.001) % 2 else -1)
                return 0.0
            fixture.stream(obj, attempt, acceleration=acceleration, size=8)
            self.assertFalse(attempt.motion_seen)
            self.assertLess(attempt.proof_interval_peak, 50.0)
            self.assertEqual(attempt.reason, "no_motion")
            self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_delayed_motion_keeps_first_and_later_real_contact_samples(self):
        for early in (False, True):
            fixture, obj, endstop = self.machine(axis="y")
            attempt = fixture.prepare(obj, endstop, -1, step_delay=0.030)
            if not early:
                fixture.stream(obj, attempt, duration=0.080, direction=-1, step_delay=0.030,
                               sensor_lag=0.015, size=8)
                self.assertTrue(attempt.motion_seen)
                first = obj.clock_to_time(attempt.last_end_clock) + 1 / fixture.rate
            else:
                first = attempt.arm_time
            row = [round(value) for value in attempt.bases]
            row[attempt.index] += counts(1.6)
            fixture.send(obj, first, [row, row])
            self.assertEqual(attempt.reason, "impact")
            self.assertAlmostEqual(obj.onset_time(attempt), first, places=6)
            self.assertEqual(endstop.trigger_log.messages, [[HIT]])

    def test_history_overflow_and_post_dispatch_diagnostic_failure_fail_safely(self):
        with mock.patch.object(plugin, "NATIVE_PROOF_HISTORY_SAMPLES", 10):
            fixture, obj, endstop = self.machine(axis="y")
            attempt = fixture.prepare(obj, endstop, -1, step_delay=0.030)
            fixture.stream(obj, attempt, parked=True, size=8)
            self.assertEqual(attempt.reason, "proof_overflow")
            self.assertTrue(attempt.proof_overflow)
            self.assertEqual(len(attempt.proof_history), 9)
            self.assertEqual(endstop.trigger_log.messages, [[HOST]])
            obj.disarm()
            self.assertTrue(attempt.record["motion_proof"]["overflow"])
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop)
        obj.main_mcu.estimated_print_time = mock.Mock(side_effect=RuntimeError("diagnostic clock"))
        obj._fire(attempt, "impact", HIT)
        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
        self.assertIsNone(attempt.request_time)

    def test_each_real_watchdog_callback_shares_one_scan_budget(self):
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop, -1, step_delay=0.015)
        fixture.stream(obj, attempt, duration=0.045, parked=True, watchdog_phase=0.200, size=8)
        clock_calls = [0]
        def moving_clock():
            clock_calls[0] += 1
            return obj.reactor.now + clock_calls[0] * 0.000001
        obj.reactor.monotonic = moving_clock
        with mock.patch.object(plugin, "NATIVE_PROOF_SCAN_BUDGET", 7):
            for eventtime in (obj.reactor.now, obj.reactor.now + 0.003):
                before = attempt.proof_scan_comparisons
                obj._watchdog(eventtime)
                self.assertEqual(attempt.proof_scan_comparisons - before, 7)
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(endstop.trigger_log.messages, [])

    def test_failed_proof_diagnostics_survive_session_and_capture_cleanup(self):
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop, -1, step_delay=0.030)
        fixture.stream(obj, attempt, parked=True, biased=True, size=8)
        obj.disarm()
        obj._end_session()
        status = obj.get_status(obj.reactor.now)
        proof = status["last"]["y"]["motion_proof"]
        self.assertEqual(status["last"]["y"]["result"], "no_motion")
        self.assertEqual(proof["effective_speed_mm_s"], 100.0)
        self.assertEqual(proof["effective_accel_mm_s2"], 5000.0)
        self.assertGreater(proof["first_step_after_arm_ms"], 30.0)
        self.assertEqual(proof["deadline_after_arm_ms"], 95.0)
        self.assertGreaterEqual(proof["request_after_arm_ms"], 95.0)
        self.assertLessEqual(proof["request_after_arm_ms"], 98.0)
        self.assertEqual(proof["step_epoch_peak_mm_s"], 0.0)
        self.assertTrue(proof["command_eligible"])
        self.assertFalse(proof["gap"])
        self.assertFalse(proof["overflow"])
        json.dumps(status, allow_nan=False)
        proof["gap"] = True
        self.assertFalse(obj.get_status(obj.reactor.now)["last"]["y"]["motion_proof"]["gap"])
        messages = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=messages.append))
        self.assertTrue(any("Y motion proof" in message and "95.0" in message for message in messages))

    def test_serial_prefix_path_never_queries_step_history(self):
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop, -1, step_delay=0.030)
        stepper = attempt.steppers[0]
        original = stepper.get_past_mcu_position
        in_serial = [False]
        def checked_position(stamp):
            self.assertFalse(in_serial[0], "step history queried by serial sample processing")
            return original(stamp)
        stepper.get_past_mcu_position = checked_position
        inspect = obj._inspect
        def serial_inspect(params):
            in_serial[0] = True
            try:
                inspect(params)
            finally:
                in_serial[0] = False
        obj._inspect = serial_inspect
        fixture.stream(obj, attempt, direction=-1, step_delay=0.030, size=8)
        self.assertTrue(attempt.motion_seen)

    def test_broadened_matched_step_and_sensor_ramp_still_fails_conservatively(self):
        fixture, obj, endstop = self.machine(axis="y")
        delay = 0.015
        attempt = fixture.prepare(obj, endstop, -1, step_delay=delay)
        actual_accel = 2500.0  # A 40 ms physical/history ramp, despite the nominal 5000 plan.
        attempt.steppers = [SignedRamp(attempt.arm_time + delay, actual_accel, 100.0, -1)]
        def acceleration(elapsed):
            return -actual_accel if delay <= elapsed < delay + 100.0 / actual_accel else 0.0
        fixture.stream(obj, attempt, acceleration=acceleration, size=8)
        self.assertFalse(attempt.motion_seen)
        self.assertLess(attempt.proof_interval_peak, 50.0)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])
        obj.disarm()
        proof = attempt.record["motion_proof"]
        self.assertEqual(proof["effective_accel_mm_s2"], 5000.0)  # Planned, not measured.
        self.assertEqual(proof["deadline_after_arm_ms"], 95.0)

    def test_step_epoch_history_survives_clock_wrap_and_cannot_leak_into_next_attempt(self):
        fixture, obj, endstop = self.machine(axis="y", epoch=0x100000000 / 1000000.0 - 0.120)
        attempt = fixture.prepare(obj, endstop, -1, step_delay=0.015)
        fixture.stream(obj, attempt, direction=-1, step_delay=0.015, size=8)
        self.assertTrue(attempt.motion_seen)
        self.assertFalse(attempt.gap)
        self.assertGreater(attempt.proof_step_epoch, 0x100000000 / fixture.clock_hz)
        old_history = attempt.proof_history
        obj.disarm()
        obj._end_session()
        obj.homing_active = True
        obj.session_axes["y"] = FakeRail(endstop, info(speed=100.0, second=100.0, retract=0))
        obj.passes = 0
        obj.reactor.now += 0.5
        obj._prepare_reference()
        new_attempt = fixture.prepare(obj, endstop, -1, step_delay=0.015)
        self.assertIsNot(new_attempt.proof_history, old_history)
        self.assertEqual(new_attempt.proof_interval_peak, 0.0)
        obj._fire(attempt, "impact", HIT)
        self.assertEqual(endstop.trigger_log.messages, [])
        fixture.stream(obj, new_attempt, parked=True, biased=True, size=8)
        self.assertFalse(new_attempt.motion_seen)
        self.assertEqual(new_attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_disconnect_after_verified_delayed_start_still_stops_immediately(self):
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop, -1, step_delay=0.030)
        fixture.stream(obj, attempt, duration=0.080, direction=-1, step_delay=0.030, size=8)
        self.assertTrue(attempt.motion_seen)
        obj.reactor.now = attempt.last_data + obj.stream_timeout + 0.001
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "no_data")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_step_epoch_query_failure_cannot_fall_through_to_full_arm_success(self):
        fixture, obj, endstop = self.machine(axis="y")
        attempt = fixture.prepare(obj, endstop, -1)
        fixture.stream(obj, attempt, duration=0.030, direction=-1, watchdog_phase=0.200, size=8)
        self.assertGreater(attempt.proof_speed_peak, 50.0)
        self.assertFalse(attempt.motion_seen)
        stepper = attempt.steppers[0]
        original = stepper.get_past_mcu_position
        calls = [0]
        def unreliable_history(stamp):
            calls[0] += 1
            if calls[0] == 1:
                raise RuntimeError("one failed epoch-search query")
            return original(stamp)
        stepper.get_past_mcu_position = unreliable_history
        obj._watchdog(obj.reactor.now)
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(attempt.proof_step_error, "RuntimeError")
        while not attempt.done:
            first = obj.clock_to_time(attempt.last_end_clock) + 1 / fixture.rate
            fixture.send(obj, first, [fixture.low, fixture.high] * 4)
            obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])
        obj.disarm()
        proof = attempt.record["motion_proof"]
        self.assertEqual(proof["result"], "step_history_error")
        self.assertLess(proof["evaluated_sample_after_arm_ms"], proof["sample_after_arm_ms"])


class WorkflowMotionProofIntegrationTests(unittest.TestCase):
    def test_relative_motion_proof_is_accepted_when_legacy_moving_is_false(self):
        rig = Rig()
        quality, _message = rig.workflow._quality("x", {
            "result": "impact", "travel_mm": 150.0, "moving": False, "motion_seen": True})
        self.assertEqual(quality, "good")

    def test_initial_no_motion_failure_aborts_calibration_without_reposition(self):
        rig = Rig()
        rig.home_results.append(CommandError("toolhead motion was not verified"))
        with self.assertRaisesRegex(CommandError, "motion was not verified"):
            rig.calibrate()
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.workflow.last_result["result"], "aborted")
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.configfile.saved, {})
        self.assertFalse(rig.capture_active)


if __name__ == "__main__":
    unittest.main()
