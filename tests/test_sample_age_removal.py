"""Delayed current-window generic data versus receipt silence; offline only."""
import unittest

from harness import CommandError, make_homing, plugin
import test_backends as backends
from test_backends import G, HIT, HOST, arm, baseline, feed, generic, samples
from test_warmup_timing import AFTER_PARSE, BEFORE_PARSE, OLDEST, reported_rows, DelayedRows


def delayed_stream(obj, delay=.080):
    """Advance sensor and receipt clocks independently at each real fake pause."""
    emitted = []

    def flow():
        now = obj.reactor.monotonic()
        first = now - delay - 15 / 3200.
        rows = [(first + number / 3200., (.02 if number % 2 else -.02) * G, -.1 * G, G)
                for number in range(16)]
        emitted.append((now, rows))
        obj.backend.helper.emit({"data": rows})

    obj.reactor.on_pause = flow
    return emitted


class GenericSampleDelayTests(unittest.TestCase):
    def test_continuous_delayed_receipts_and_watchdog_preserve_late_post_arm_hit(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = generic(legacy=legacy)
                baseline(obj)
                attempt, trigger = arm(obj)
                attempt.vector_mode = True  # Current calibrated XYZ detector.
                for number in range(5):
                    start = 10.060 + number * .005
                    receipt = 10.075 + number * .027
                    # Poll the actual watchdog before and after each receipt;
                    # no test writes last_data or bypasses its stream timeout.
                    obj.reactor.now = receipt - .010
                    obj._watchdog(obj.reactor.now)
                    self.assertIsNone(attempt.reason)
                    rows = samples(start)
                    if number == 4:
                        rows[4] = (rows[4][0], -12. * G, 0., G)
                    obj.reactor.now = receipt
                    obj.backend.helper.emit({"data": rows})
                    obj._watchdog(receipt)
                    self.assertIsNone(obj.sensor_fault)
                    if number < 4:
                        self.assertIsNone(attempt.reason)
                        self.assertEqual(attempt.last_data, receipt)
                    else:
                        self.assertGreater(receipt - rows[4][0], .080)
                        self.assertEqual(attempt.reason, "impact")
                        self.assertAlmostEqual(obj.onset_time(attempt), rows[4][0], places=6)
                        self.assertEqual(trigger.messages, [[HIT]])

    def test_logged_71_432_8_943_ms_packet_keeps_its_early_post_arm_impact(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = generic(legacy=legacy)
                baseline(obj, OLDEST - .060)
                attempt, trigger = arm(obj, when=OLDEST, deadline=OLDEST + 10.)
                rows = reported_rows()
                rows[2] = (rows[2][0], -12. * G, 0., G)
                obj.reactor.now = BEFORE_PARSE
                obj.backend.helper.emit({"data": DelayedRows(obj, rows)})
                self.assertEqual(attempt.reason, "impact")
                self.assertEqual(trigger.messages, [[HIT]])
                self.assertAlmostEqual(obj.onset_time(attempt), rows[2][0], places=6)
                self.assertAlmostEqual((AFTER_PARSE - rows[0][0]) * 1000., 71.43246153846)
                self.assertAlmostEqual((AFTER_PARSE - rows[-1][0]) * 1000., 8.94346153846)
                self.assertIsNone(obj.sample_timing_fault)

    def test_pre_arm_only_receipts_cannot_hit_or_keep_the_armed_watchdog_alive(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = generic(legacy=legacy)
                baseline(obj)
                attempt, trigger = arm(obj, when=10.200)
                attempt.vector_mode = True
                # The poll before arm establishes the start of receipt liveness.
                obj.reactor.now = attempt.arm_time - .001
                obj._watchdog(obj.reactor.now)
                last_valid = attempt.last_data
                for number in range(3):
                    rows = samples(10.060 + number * .005, x=-15.)
                    obj.reactor.now = attempt.arm_time + number * .020
                    obj.backend.helper.emit({"data": rows})
                    self.assertIsNone(attempt.reason)
                    self.assertEqual(attempt.last_data, last_valid)
                    self.assertEqual(trigger.messages, [])
                obj._watchdog(last_valid + obj.stream_timeout + .000001)
                self.assertEqual(attempt.reason, "no_data")
                self.assertEqual(trigger.messages, [[HOST]])

    def test_delayed_reference_can_arm_when_recently_received_but_silence_cannot(self):
        for legacy in (False, True):
            for silent in (False, True):
                with self.subTest(legacy=legacy, silent=silent):
                    fixture = backends.GenericPreparationTest()
                    obj, endstop = fixture.session(legacy=legacy)
                    delayed_stream(obj)
                    fixture.begin(obj, endstop)
                    reference = obj.preparation
                    self.assertGreater(obj.reactor.now - reference.last, .079)
                    self.assertLess(obj.reactor.now - reference.last_receipt_time, obj.stream_timeout)
                    self.assertIsNone(obj.baseline_status["max_sample_age_ms"])
                    self.assertFalse(obj.baseline_status["sample_age_limit_enforced"])
                    if silent:
                        obj.reactor.now += obj.stream_timeout + .001
                        with self.assertRaises(CommandError):
                            endstop.home_start(obj.reactor.now + .010, 0, 0, 0)
                        self.assertEqual(obj.baseline_status["result"], "no_data_before_arm")
                        self.assertEqual(endstop._dispatch.started, [])
                    else:
                        endstop.home_start(obj.reactor.now + .010, 0, 0, 0)
                        self.assertIsNotNone(obj.attempt)
                        self.assertEqual(len(endstop._dispatch.started), 1)

    def test_exact_5_ms_future_boundary_and_over_boundary_remain_distinct(self):
        for future, accepted in ((.005, True), (.005001, False)):
            with self.subTest(future=future):
                obj = generic()
                rows = samples(future - 15 / 3200.)
                obj.reactor.now = 0.
                obj.backend.helper.emit({"data": rows})
                self.assertEqual(obj.sensor_fault, None if accepted else "stale_data")
                if not accepted:
                    self.assertEqual(obj.sample_timing_fault["reason"], "future")

    def test_stream_restart_uses_generation_cutoff_instead_of_timestamp_age(self):
        obj = generic()
        feed(obj, samples(.990))
        obj._stop_stream()
        obj.reactor.now = 1.
        obj._start_stream()
        before = obj.last_batch
        obj.reactor.now = 1.100
        obj.backend.helper.emit({"data": samples(.990)})
        self.assertIsNone(obj.sensor_fault)
        self.assertEqual(obj.last_batch, before)
        current = samples(1.001)
        obj.backend.helper.emit({"data": current})
        self.assertIsNone(obj.sensor_fault)
        self.assertEqual(obj.last_batch, 1.100)
        self.assertEqual(obj.backend.last_sample_time, current[-1][0])
        self.assertGreater(obj.reactor.now - current[-1][0], .080)
        obj.backend.helper.emit({"data": samples(current[-1][0] + 1 / 3200., size=2)
                                 + samples(.990, size=1)
                                 + samples(current[-1][0] + 3 / 3200., size=2)})
        self.assertIn(obj.sensor_fault, ("invalid_data", "sample_gap"))


class GenericDelayedWindowTests(unittest.TestCase):
    def sensor(self, legacy=False):
        obj = generic(legacy=legacy)
        obj._end_session()
        obj.reactor.now = 10.
        delayed_stream(obj)
        return obj

    def test_three_stationary_windows_accept_80_ms_initial_and_continuous_delay(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = self.sensor(legacy)
                obj.start_noise_measurement()
                started = obj.reactor.now
                captures = [obj.collect_noise_window(lambda: None) for _ in range(3)]
                for capture in captures:
                    self.assertGreater(capture["samples"][0][0], capture["cutoff_time_s"])
                    self.assertGreaterEqual(capture["samples"][-1][0] - capture["samples"][0][0] + 1e-9, 1.)
                self.assertGreater(obj.reactor.now - captures[-1]["samples"][-1][0], .079)
                self.assertGreater(obj.reactor.now - started, 3.2)
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNone(obj.sample_timing_fault)
                obj.stop_noise_measurement()
                self.assertEqual(obj.toolhead.moves, [])

    def test_beta_stationary_reference_moving_window_and_home_accept_delayed_rows(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = self.sensor(legacy)
                token = obj.start_beta_motion_capture(lambda: None)
                self.assertGreater(obj.reactor.now - token["baseline_samples"][-1][0], .079)
                end = obj.reactor.now + .150
                token["motion_end_time_s"] = end
                while obj.reactor.now < end:
                    obj.reactor.pause(obj.reactor.now + .005)
                    obj._check_beta_motion_capture(token, lambda: None)
                captured = obj.stop_beta_motion_capture(token, lambda: None)
                self.assertTrue(captured["raw_samples_validated"])
                self.assertGreaterEqual(captured["samples"][-1][0] + 1e-6, end)
                self.assertGreater(captured["samples"][0][0], captured["capture_cutoff_time_s"])
                self.assertGreater(obj.reactor.now - captured["samples"][-1][0], .079)
                self.assertIsNone(obj.sample_timing_fault)
                # Beta home capture uses its own independent validated stream.
                obj.homing_active = True
                obj._start_stream()
                obj.start_capture(beta=True)
                start = obj.reactor.now + .001
                rows = samples(start)
                obj.reactor.now = rows[-1][0] + .080
                obj.backend.helper.emit({"data": rows})
                home = obj.stop_capture()
                self.assertIsNone(home["fault"])
                self.assertEqual(len(home["samples"]), len(rows))
                self.assertTrue(home["beta_capture"]["quality"]["maximum_packet_sample_age_s"] > .080)
                obj._stop_stream()

    def test_continuous_delayed_noise_then_real_silence_uses_receipt_timeout(self):
        obj = self.sensor()
        obj.start_noise_measurement()
        flow = obj.reactor.on_pause

        def stop_after_valid_rows():
            flow()
            if obj.noise_window and len(obj.noise_window["samples"]) >= 16:
                obj.reactor.on_pause = None

        obj.reactor.on_pause = stop_after_valid_rows
        faults = []
        with self.assertRaisesRegex(CommandError, "stopped receiving validated samples"):
            obj.collect_noise_window(lambda: None, on_acquisition_fault=faults.append)
        self.assertIsNone(obj.sample_timing_fault)
        self.assertEqual(len(faults), 1)
        fault = faults[0]
        self.assertEqual(fault["reason"], "no_fresh_data")
        self.assertEqual(fault["phase"], "noise_wait")
        self.assertGreater(fault["receipt_now_s"] - fault["last_receipt_time_s"], fault["receipt_timeout_s"])
        self.assertGreater(fault["last_receipt_time_s"] - fault["last_sample_time_s"], .079)
        obj.stop_noise_measurement()

    def test_native_beacon_keeps_its_50_ms_reference_and_beta_timing(self):
        obj = make_homing()
        self.assertEqual(obj._reference_max_age(), .050)
        obj.reactor.now = 1.
        obj.start_capture(beta=True)
        from harness import batch, counts
        obj._capture_batch(batch(900, [(0, 0, counts(1.)), (1, 0, counts(1.))], delta_clock=1))
        captured = obj.stop_capture()
        self.assertEqual(captured["fault"], "stale_data")

    def test_beta_home_keeps_original_times_beyond_signed_half_clock_wrap(self):
        obj = generic()
        obj.start_capture(beta=True)
        rows = samples(1400.)
        obj.reactor.now = 5000.  # Valid current-generation data, 3600 s in transit.
        obj.backend.helper.emit({"data": rows})
        captured = obj.stop_capture()
        self.assertIsNone(obj.sensor_fault)
        self.assertIsNone(captured["fault"])
        self.assertIsNone(obj.sample_timing_fault)
        self.assertEqual(len(captured["samples"]), len(rows))
        self.assertEqual(captured["samples"][0][0], rows[0][0])
        self.assertEqual(captured["samples"][-1][0], rows[-1][0])
        self.assertGreater(captured["beta_capture"]["quality"]["maximum_packet_sample_age_s"], 3599.)


if __name__ == "__main__":
    unittest.main()
