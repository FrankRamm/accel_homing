"""Reported Beacon packet periods and retained timing guards; offline only."""
import unittest

from harness import FakeEndstop, batch, counts, make_homing, plugin
from test_backends import feed, generic, samples


PERIODS = (.0003022, .0003450)
BOUNDARY_GAP = .000311
CLOCK_HZ = 10000000.


def timed_sensor(decoded=False):
    obj = generic() if decoded else make_homing()
    obj._end_session()
    obj.reactor.now = 10.
    if not decoded:
        # Preserve the reported sub-microsecond periods in integer MCU ticks.
        obj.beacon_freq = CLOCK_HZ
        obj.clock_to_time = lambda clock: clock / CLOCK_HZ
        obj.sensor_clock.print_time_to_clock = lambda when: round(when * CLOCK_HZ)
        obj.sensor_clock.seconds_to_clock = lambda duration: duration * CLOCK_HZ
        obj.impact_window = round(plugin.IMPACT_WINDOW * CLOCK_HZ)
    return obj


def packet(obj, start, interval, count=8):
    count_g = obj.noise_sensor_context()["count_g"]
    rows = [(1 if number % 2 else -1, 0, round(1. / count_g))
            for number in range(count)]
    params = batch(round(start * obj.beacon_freq), rows,
                   delta_clock=round((count - 1) * interval * obj.beacon_freq))
    if obj.backend is not None:
        params.update(data=tuple(value for row in rows for value in row),
                      _sample_start_time_s=start,
                      _sample_end_time_s=start + (count - 1) * interval)
    return params


def window_sensor(decoded=False, moving=False):
    obj = timed_sensor(decoded)
    context = obj.start_noise_measurement()
    obj.noise_window = {"index": 1, "cutoff": 10.,
                        "duration": None if moving else 1., "samples": [],
                        "last": None, "interval": None, "complete": False,
                        "changed": False, "last_receipt_time": None}
    if moving:
        obj.noise_window["beta_quality"] = obj._new_beta_quality(context)
    return obj


class BeaconPacketTimingTests(unittest.TestCase):
    def deliver(self, obj, mode, start, interval):
        params = packet(obj, start, interval)
        obj.reactor.now = start + 7 * interval + .001
        if mode == "capture":
            obj._capture_batch(params)
        else:
            obj._inspect_noise_batch(params)
        return params

    def owner(self, mode, decoded=False):
        if mode != "capture":
            return window_sensor(decoded, moving=mode == "moving")
        obj = timed_sensor(decoded)
        obj.start_capture(beta=True)
        return obj

    def test_reported_native_jitter_is_accepted_in_both_directions_and_capture_paths(self):
        for mode in ("stationary", "moving", "capture"):
            for periods in (PERIODS, PERIODS[::-1]):
                with self.subTest(mode=mode, periods=periods):
                    obj = self.owner(mode)
                    first = 10.001
                    self.deliver(obj, mode, first, periods[0])
                    second = first + 7 * periods[0] + BOUNDARY_GAP
                    self.deliver(obj, mode, second, periods[1])
                    self.assertIsNone(obj.sensor_fault)
                    if mode == "capture":
                        capture = obj.stop_capture()
                        self.assertIsNone(capture["fault"])
                        self.assertEqual(len(capture["samples"]), 16)
                        quality = capture["beta_capture"]["quality"]
                        self.assertEqual(quality["gap_observations"], 0)
                    else:
                        self.assertEqual(len(obj.noise_window["samples"]), 16)
                        if mode == "moving":
                            self.assertEqual(obj.noise_window["beta_quality"]["gap_observations"], 0)
                        obj.stop_noise_measurement()

    def test_full_stationary_window_with_alternating_periods_needs_no_retry(self):
        obj = timed_sensor()
        start = [10.0001]
        sequence = [0]

        def emit():
            while True:
                interval = PERIODS[sequence[0] % 2]
                end = start[0] + 7 * interval
                if end > obj.reactor.now - .0001:
                    return
                obj._inspect(packet(obj, start[0], interval))
                start[0] = end + BOUNDARY_GAP
                sequence[0] += 1

        obj.reactor.on_pause = emit
        obj.start_noise_measurement()
        capture = obj.collect_noise_window(lambda: None, recover_transient=True)
        self.assertNotIn("acquisition_faults", capture)
        self.assertGreaterEqual(capture["samples"][-1][0] - capture["samples"][0][0], 1.)
        self.assertGreater(len(capture["samples"]), 3000)
        self.assertEqual(obj.noise_session["retries"], 0)
        obj.stop_noise_measurement()

    def test_decoded_cross_packet_period_gate_remains_ten_percent(self):
        for mode in ("stationary", "moving", "capture"):
            with self.subTest(mode=mode):
                obj = self.owner(mode, decoded=True)
                # Both rates exceed the configured minimum. A 13.3% period
                # difference must still fail for non-Beacon decoded streams.
                previous, current = .000300, .000260
                self.deliver(obj, mode, 10.001, previous)
                self.deliver(obj, mode, 10.001 + 7 * previous + current, current)
                self.assertEqual(obj.sensor_fault, "sample_gap")
                if mode != "capture":
                    details = obj.noise_window["fault_details"]
                    self.assertAlmostEqual(details["interval_change_limit_s"], current * .10)

    def test_gaps_over_five_samples_and_excessive_period_changes_remain_rejected(self):
        for mode in ("stationary", "moving", "capture"):
            for failure in ("dropout", "period"):
                with self.subTest(mode=mode, failure=failure):
                    obj = self.owner(mode)
                    previous = PERIODS[0]
                    current = previous if failure == "dropout" else .000450
                    gap = 5 * current + .0000031 if failure == "dropout" else BOUNDARY_GAP
                    self.deliver(obj, mode, 10.001, previous)
                    self.deliver(obj, mode, 10.001 + 7 * previous + gap, current)
                    self.assertEqual(obj.sensor_fault, "sample_gap")
                    if mode != "capture":
                        details = obj.noise_window["fault_details"]
                        self.assertAlmostEqual(details["gap_limit_s"], 5 * current + .000003)
                        self.assertAlmostEqual(details["interval_change_limit_s"], current * .25)
                        if failure == "period":
                            self.assertLess(details["gap_s"], details["gap_limit_s"])

    def test_overlapping_packets_remain_rejected(self):
        for mode in ("stationary", "moving", "capture"):
            for gap in (0., -.000100):
                with self.subTest(mode=mode, gap=gap):
                    obj = self.owner(mode)
                    self.deliver(obj, mode, 10.001, PERIODS[0])
                    self.deliver(obj, mode, 10.001 + 7 * PERIODS[0] + gap, PERIODS[1])
                    self.assertEqual(obj.sensor_fault, "sample_gap" if mode == "capture" else "invalid_data")

    def test_native_one_tick_advance_is_not_confused_with_a_duplicate(self):
        for mode in ("stationary", "moving", "capture"):
            with self.subTest(mode=mode):
                obj = self.owner(mode)
                self.deliver(obj, mode, 10.001, PERIODS[0])
                self.deliver(obj, mode, 10.001 + 7 * PERIODS[0] + 1. / CLOCK_HZ, PERIODS[1])
                self.assertIsNone(obj.sensor_fault)

    def test_decoded_nonuniform_rows_still_fail_inside_one_packet(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = generic(legacy=legacy)
                rows = samples(10.001, size=8)
                # Keep endpoints and mean period unchanged, but perturb two
                # adjacent gaps by 20%; this is actual row nonuniformity.
                rows[3] = (rows[3][0] + .2 / 3200.,) + rows[3][1:]
                feed(obj, rows)
                self.assertEqual(obj.sensor_fault, "sample_gap")

    def test_startup_reference_keeps_jitter_tolerance_and_missing_sample_gate(self):
        for gap, expected in ((BOUNDARY_GAP, None), (.002, "sample_gap"), (-1. / CLOCK_HZ, "invalid_data")):
            with self.subTest(gap=gap):
                obj = timed_sensor()
                obj.preparation = plugin.QuietReference("x", 10., .1, 100.)
                first = PERIODS[0]
                starts = (10.001, 10.001 + 7 * first + gap)
                reasons = []
                for start, interval in zip(starts, PERIODS):
                    params = packet(obj, start, interval)
                    now = start + 7 * interval + .001
                    reasons.append(obj._collect_reference(params, params["data"], 8, now))
                self.assertIsNone(reasons[0])
                self.assertEqual(reasons[1], expected)

    def test_armed_detector_keeps_jitter_tolerance_and_missing_sample_gate(self):
        for gap, expected in ((BOUNDARY_GAP, None), (.002, "sample_gap"), (0., "invalid_data")):
            with self.subTest(gap=gap):
                obj = timed_sensor()
                endstop = FakeEndstop("x")
                count_g = obj.scale["scale"] / 1000.
                attempt = plugin.Attempt(endstop, 1, 10.001, round(10.001 * CLOCK_HZ),
                                         11., round(11. * CLOCK_HZ), 0, 1, -1,
                                         8. / count_g, count_g)
                attempt.bases = (0, 0, counts(1.))
                attempt.vector_mode = True
                obj.arm(attempt)
                for start, interval in zip((10.001, 10.001 + 7 * PERIODS[0] + gap), PERIODS):
                    obj.reactor.now = start + 7 * interval + .001
                    obj._inspect(packet(obj, start, interval))
                self.assertEqual(attempt.reason, expected)
                if expected is None:
                    self.assertFalse(attempt.done)
                    self.assertEqual(endstop.trigger_log.messages, [])
                else:
                    self.assertTrue(attempt.done)
                    self.assertEqual(endstop.trigger_log.messages, [[plugin.mcu.MCU_trsync.REASON_HOST_REQUEST]])


if __name__ == "__main__":
    unittest.main()
