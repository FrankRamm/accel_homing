"""Exact rolling pre-arm references; all traces are synthetic, not printer data."""
import random
import types
import unittest
from unittest import mock

from harness import CommandError, make_homing, plugin, stationary_stream
from test_backends import G, feed, generic, samples
from test_session import FakeRail, info, with_endstops


class QuietReferenceTest(unittest.TestCase):
    def reference(self, duration=0.1, limit=700.0):
        return plugin.QuietReference("x", -1.0, duration, limit, threshold_g=0.5)

    def oracle(self, rows, duration, limit):
        """Search all latest-ending suffixes independently of the deque state."""
        low, high = [float("inf")] * 3, [-float("inf")] * 3
        earliest = len(rows) - 1
        for start in range(len(rows) - 1, -1, -1):
            low = [min(a, b) for a, b in zip(low, rows[start][1])]
            high = [max(a, b) for a, b in zip(high, rows[start][1])]
            if any(b - a > limit for a, b in zip(low, high)):
                break
            earliest = start
            if (len(rows) - start >= plugin.BASELINE_MIN_SAMPLES
                    and rows[-1][0] - rows[start][0] + 1e-9 >= duration):
                return rows[start:]
        return rows[earliest:]

    def assert_window(self, reference, rows):
        self.assertEqual(reference.count, len(rows))
        self.assertEqual(reference.first, rows[0][0])
        self.assertEqual(reference.last, rows[-1][0])
        self.assertEqual([(when, values) for _number, when, values in reference._samples], rows)
        for axis in range(3):
            values = [row[1][axis] for row in rows]
            self.assertEqual(reference.low[axis], min(values))
            self.assertEqual(reference.high[axis], max(values))
            self.assertAlmostEqual(reference.totals[axis], sum(values), places=7)
        ready = (len(rows) >= plugin.BASELINE_MIN_SAMPLES
                 and rows[-1][0] - rows[0][0] + 1e-9 >= reference.duration)
        if ready:
            for axis, value in enumerate(reference.baseline()):
                self.assertAlmostEqual(value, sum(row[1][axis] for row in rows) / len(rows), places=9)
        else:
            self.assertIsNone(reference.baseline())

    def test_old_extremum_does_not_discard_a_complete_overlapping_window(self):
        # The former reset-to-newest algorithm retained only 66.875 ms before
        # the jump and 33.125 ms after it. Samples 1..321 are a valid 100 ms
        # interval: only sample 0 needs removal, on whichever axis conflicts.
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    reference, rows = self.reference(), []
                    for number in range(322):
                        values = [0.0, 40.0, 1000.0]
                        values[axis] += sign * (-700.0 if number == 0 else
                                               700.0 if number >= 215 else 0.0)
                        row = (number / 3200.0, tuple(values))
                        rows.append(row)
                        self.assertIsNone(reference.add(*row))
                        if number < 321:
                            self.assertIsNone(reference.baseline())
                    self.assert_window(reference, rows[1:])
                    self.assertEqual(reference.resets, 1)
                    self.assertEqual(reference.accepted, 322)
                    self.assertEqual(reference.observed_high[axis] - reference.observed_low[axis], 1400.0)
                    self.assertEqual(reference.best_count, 321)
                    self.assertAlmostEqual(reference.best_span, 0.1)

    def test_minimum_count_and_full_duration_are_both_required(self):
        reference = self.reference()
        for number in range(49):
            reference.add(number / 480.0, (0.0, 0.0, 1000.0))
        self.assertEqual(reference.count, 49)
        self.assertAlmostEqual(reference.last - reference.first, 0.1)
        self.assertIsNone(reference.baseline())
        reference.add(49 / 480.0, (0.0, 0.0, 1000.0))
        self.assertEqual(reference.count, 50)
        self.assertIsNotNone(reference.baseline())
        reference.add(50 / 480.0, (0.0, 0.0, 1000.0))
        self.assertEqual(reference.count, 50)
        self.assertEqual(reference.first, 1 / 480.0)

        for shortfall, ready in ((2e-9, False), (0.5e-9, True), (0.0, True)):
            with self.subTest(shortfall=shortfall):
                reference = self.reference()
                for number in range(50):
                    reference.add(number * (0.1 - shortfall) / 49, (0.0, 0.0, 1000.0))
                self.assertEqual(reference.baseline() is not None, ready)
                self.assertEqual(reference.count, 50)

    def test_duplicate_extrema_and_conflicts_on_different_axes_keep_a_contiguous_suffix(self):
        values = [(0, 0, 0), (0, 0, 0), (700, 0, 0), (700, 700, 0),
                  (700, 700, 700), (701, 700, 700), (701, 701, 700),
                  (701, 701, 701), (-1, 701, 701), (-1, -1, 701),
                  (-1, -1, -1), (-1, -1, -1), (699, 699, 699)]
        reference, rows = self.reference(), []
        for number, sample in enumerate(values):
            rows.append((number / 3200.0, sample))
            self.assertIsNone(reference.add(*rows[-1]))
            self.assert_window(reference, self.oracle(rows, reference.duration, reference.limit))
        self.assertGreater(reference.resets, 1)

    def test_retained_mean_extrema_and_proof_bound_use_the_same_boundary_inclusive_rows(self):
        reference, rows = self.reference(), []
        for number in range(601):
            values = ((-350.0, -150.0, 800.0) if number < 100 else
                      (0.0 if number % 4 == 0 else 300.0,
                       50.0 if number % 3 == 0 else 100.0,
                       900.0 if number % 5 == 0 else 1100.0))
            rows.append((number / 3200.0, values))
            reference.add(*rows[-1])
        retained = rows[-321:]
        self.assert_window(reference, retained)
        self.assertEqual(reference.first, (600 - 320) / 3200.0)
        self.assertEqual(reference.low, [0.0, 50.0, 900.0])
        obj = generic()
        obj.threshold_g["x"] = 0.5
        means = reference.baseline()
        expected = tuple(max(abs(row[1][axis] - means[axis]) for row in retained)
                         for axis in range(3))
        self.assertEqual(obj._reference_motion_noise(reference, means, "x"), expected)
        self.assertGreater(expected[0], (reference.high[0] - reference.low[0]) / 2.0)

    def test_window_matches_independent_suffix_search_after_every_sample(self):
        for rate, duration in ((1000.0, 0.05), (1000.0, 0.1), (3200.0, 0.1)):
            with self.subTest(rate=rate, duration=duration):
                rng = random.Random(715)
                reference, rows = self.reference(duration), []
                for number in range(850):
                    # Quiet plateaus, tied extrema and changes near the range
                    # limit produce both overlapping successes and refusals.
                    center = (number // 170) % 3 * 500
                    values = tuple(center + rng.choice((-200, -150, 0, 150, 200))
                                   for _ in range(3))
                    rows.append((123456.0 + number / rate, values))
                    self.assertIsNone(reference.add(*rows[-1]))
                    self.assert_window(reference, self.oracle(rows, duration, 700.0))

    def test_long_quiet_history_retains_only_a_complete_window_and_boundary_sample(self):
        reference = self.reference(duration=1.0)
        for number in range(6401):
            reference.add(number / 3200.0, (number % 7, number % 11, 1000.0))
        self.assertEqual(reference.count, 3201)
        self.assertEqual(reference.first, 1.0)
        self.assertEqual(reference.last, 2.0)
        self.assertIsNotNone(reference.baseline())
        self.assertFalse(reference.overflow)
        self.assertEqual(len(reference._samples), 3201)
        self.assertTrue(all(len(queue) <= reference.count
                            for queue in reference._minimum + reference._maximum))

    def test_capacity_is_fail_closed_and_latched_without_silent_eviction(self):
        self.assertEqual(plugin.BASELINE_MAX_SAMPLES, 65536)
        with mock.patch.object(plugin, "BASELINE_MAX_SAMPLES", 50):
            reference = self.reference()
            for number in range(50):
                self.assertIsNone(reference.add(number / 1000.0, (number, 0, 1000)))
            self.assertEqual(reference.count, 50)
            self.assertEqual(reference.first, 0.0)
            self.assertEqual(reference.add(0.050, (50, 0, 1000)), "baseline_overflow")
            self.assertTrue(reference.overflow)
            self.assertEqual(reference.accepted, 51)
            self.assertEqual(reference.best_count, 50)
            self.assertEqual(reference.last, 0.050)
            self.assertEqual(reference.count, 0)
            self.assertIsNone(reference.first)
            self.assertIsNone(reference.baseline())
            self.assertEqual(len(reference._samples), 0)
            self.assertTrue(all(not queue for queue in reference._minimum + reference._maximum))
            for number in range(51, 301):
                self.assertEqual(reference.add(number / 1000.0, (0, 0, 1000)), "baseline_overflow")
            self.assertEqual(reference.last, 0.3)
            self.assertIsNone(reference.baseline())
            self.assertEqual(len(reference._samples), 0)

            # A removable old boundary must be pruned before checking capacity.
            reference = self.reference(duration=0.05)
            for number in range(150):
                self.assertIsNone(reference.add(number * 0.05 / 49, (0, 0, 1000)))
            self.assertEqual(reference.count, 50)
            self.assertFalse(reference.overflow)
            self.assertIsNotNone(reference.baseline())

    def test_calibration_reference_keeps_all_noisy_rows_and_full_count_duration_and_capacity(self):
        reference = plugin.QuietReference("x", -1.0, 0.1, 700.0, require_quiet=False)
        rows = []
        for number in range(641):
            sign = -1 if number % 4 == 0 else 1
            rows.append((number / 3200.0, (sign * 900, sign * 1100, 1000 + sign * 1300)))
            self.assertIsNone(reference.add(*rows[-1]))
            if number < 320:
                self.assertIsNone(reference.baseline())
        self.assert_window(reference, rows[-321:])
        self.assertEqual(reference.resets, 0)
        self.assertTrue(all(high - low > reference.limit
                            for low, high in zip(reference.low, reference.high)))
        sparse = plugin.QuietReference("x", -1.0, 0.1, 700.0, require_quiet=False)
        for number in range(49):
            sparse.add(number / 480.0, (number * 900, 0, 1000))
        self.assertIsNone(sparse.baseline())
        with mock.patch.object(plugin, "BASELINE_MAX_SAMPLES", 50):
            bounded = plugin.QuietReference("x", -1.0, 0.1, 700.0, require_quiet=False)
            for number in range(50):
                self.assertIsNone(bounded.add(number / 3200.0, (number * 900, 0, 1000)))
            self.assertEqual(bounded.add(50 / 3200.0, (0, 0, 1000)), "baseline_overflow")
            self.assertTrue(bounded.overflow)
            self.assertIsNone(bounded.baseline())

    def test_calibration_keeps_measured_envelope_when_threshold_exceeds_reference_range(self):
        for factory in (make_homing, generic):
            for require_quiet in (False, True):
                with self.subTest(backend=factory.__name__, require_quiet=require_quiet):
                    obj = factory()
                    count_g = obj.session_scale["scale"] / 1000.0
                    reference = plugin.QuietReference("x", -1.0, 0.1, 2.0 / count_g,
                                                      require_quiet=require_quiet)
                    for number in range(321):
                        sign = -1 if number % 4 == 0 else 1
                        reference.add(number / 3200.0,
                                      tuple(value / count_g for value in
                                            (sign * 0.65, sign * 0.3, 1.0 + sign * 0.14)))
                    means = reference.baseline()
                    expected = tuple(max(abs(low - mean), abs(high - mean))
                                     for low, high, mean in zip(reference.low, reference.high, means))
                    for threshold in (1.0, 1.25, 8.0):
                        obj.threshold_g["x"] = threshold
                        envelope = obj._reference_motion_noise(reference, means, "x")
                        if require_quiet and threshold == 8.0:
                            self.assertIsNone(envelope)
                        else:
                            self.assertEqual(envelope, expected)
                    self.assertGreater(expected[0], (reference.high[0] - reference.low[0]) / 2.0)


class RollingReferenceIntegrationTest(unittest.TestCase):
    def machine(self, legacy=False):
        obj = generic(legacy=legacy, connect=False)
        obj.fake_printer.send_event("klippy:connect")
        obj.threshold_g["x"] = 0.5
        obj.reactor.now = 10.0
        endstop = with_endstops(obj)["x"]
        stationary_stream(obj)
        obj._handle_rails_begin(None, [FakeRail(endstop, info(speed=50, retract=0))])
        number = [0]
        def flow():
            rows = []
            for offset in range(16):
                index = number[0] + offset
                x = -0.69 if index == 0 else 0.69 if index >= 215 else 0.0
                when = obj.preparation.cutoff + (index + 1) / 3200.0
                rows.append((when, x * G, 0.0, G))
            number[0] += 16
            obj.backend.helper.emit({"data": rows})
        obj.reactor.on_pause = flow
        return obj, endstop

    def prepare(self, obj, endstop):
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))

    def test_overlap_acquires_before_arm_and_keeps_raw_contact_and_noise_envelope(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, endstop = self.machine(legacy)
                self.prepare(obj, endstop)
                reference = obj.preparation
                self.assertLessEqual(obj.reactor.now - reference.cutoff, 0.110)
                self.assertEqual(reference.count, 321)
                self.assertGreater(reference.first, reference.cutoff)
                self.assertGreaterEqual(reference.last - reference.first + 1e-9, 0.1)
                self.assertAlmostEqual(obj.baseline_status["quiet_limit_g"], 0.7)
                self.assertEqual(endstop._dispatch.started, [])
                self.assertEqual(endstop.get_mcu().log, [])
                self.assertEqual(obj.toolhead.moves, [])
                means = reference.baseline()
                bound = tuple(max(abs(low - mean), abs(high - mean))
                              for low, high, mean in zip(reference.low, reference.high, means))
                when = obj.backend.last_sample_time + 1 / 3200.0
                attempt = obj.prepare(endstop, when)
                self.assertEqual(attempt.bases, means)
                self.assertEqual(attempt.proof_noise, bound)
                self.assertAlmostEqual(attempt.limit_raw * attempt.count_g, 0.5)
                obj.arm(attempt)
                rows = samples(when, x=means[0] * attempt.count_g)
                rows[0] = (when, (means[0] * attempt.count_g + 0.51) * G, 0.0, G)
                feed(obj, rows)
                self.assertEqual(attempt.reason, "impact")
                self.assertAlmostEqual(obj.onset_time(attempt), when, delta=1e-6)

    def test_acquired_rolling_window_still_expires_or_is_invalidated_before_arm(self):
        for mode in ("stale", "new_noise"):
            with self.subTest(mode=mode):
                obj, endstop = self.machine()
                self.prepare(obj, endstop)
                if mode == "stale":
                    obj.reactor.now += obj.backend.max_age + 0.010
                else:
                    feed(obj, samples(obj.backend.last_sample_time + 1 / 3200.0, x=-0.8))
                with self.assertRaisesRegex(CommandError, "fresh quiet"):
                    endstop.home_start(obj.reactor.now + 0.010, 0, 0, 0)
                expected = ("expired_before_arm" if mode == "stale"
                            else "quiet_window_invalidated_before_arm")
                self.assertEqual(obj.baseline_status["result"], expected)
                self.assertEqual(endstop._dispatch.started, [])
                self.assertEqual(endstop.get_mcu().log, [])
                self.assertEqual(obj.toolhead.moves, [])
                self.assertFalse(obj.homing_active)

    def test_capacity_failure_is_reported_during_acquisition_before_dispatch(self):
        with mock.patch.object(plugin, "BASELINE_MAX_SAMPLES", 50):
            obj, endstop = self.machine()
            with self.assertRaisesRegex(CommandError, "baseline.*bounded.*capacity"):
                self.prepare(obj, endstop)
            self.assertEqual(obj.sensor_fault, "baseline_overflow")
            self.assertEqual(obj.baseline_status["result"], "baseline_overflow")
            self.assertEqual(obj.baseline_status["best_quiet_samples"], 50)
            self.assertEqual(obj.baseline_status["accepted_samples"], 51)
            self.assertEqual(endstop._dispatch.started, [])
            self.assertEqual(endstop.get_mcu().log, [])
            self.assertEqual(obj.toolhead.moves, [])
            self.assertFalse(obj.homing_active)


if __name__ == "__main__":
    unittest.main()
