"""Offline current-window and receipt timing for configured-sensor warmup."""
import unittest
from unittest import mock

from harness import plugin, stationary_stream
from test_backends import G, HOST, arm, baseline, generic, samples


OLDEST = 35.020921
NEWEST = 35.083410
BEFORE_PARSE = 35.090500
AFTER_PARSE = 35.09235346153846


def reported_rows():
    # A dense, uniform ~3200 Hz packet, with the exact reported endpoints.
    return [(OLDEST + (NEWEST - OLDEST) * n / 200, 0., 0., G)
            for n in range(201)]


def measured_window():
    return {"cutoff": 35., "complete": False, "index": 1, "samples": [],
            "duration": 1., "last": None, "interval": None, "changed": False,
            "last_receipt_time": None}


class DelayedRows(list):
    """Account for parsing time without altering the sensor's timestamps."""
    def __init__(self, obj, rows, after=AFTER_PARSE):
        super().__init__(rows)
        self.obj, self.after = obj, after

    def __iter__(self):
        self.obj.reactor.now = self.after
        return super().__iter__()


class WarmupTimingTest(unittest.TestCase):
    def owner(self, legacy=False):
        obj = generic(legacy=legacy)
        obj.reactor.now = BEFORE_PARSE
        return obj

    def delayed(self, obj, rows=None, after=AFTER_PARSE, **fields):
        obj.backend.helper.emit({"data": DelayedRows(
            obj, reported_rows() if rows is None else rows, after), **fields})

    def test_reported_71_432_8_943_ms_packet_keeps_every_current_capture_row(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = self.owner(legacy)
                obj.start_capture()  # Ordinary CALIBRATE opens this before G28.
                self.delayed(obj)
                expected = reported_rows()
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNone(obj.sample_timing_fault)
                self.assertEqual(obj.backend.max_age, .070)
                self.assertAlmostEqual(obj.backend.last_age, .00894346153846)
                captured = obj.stop_capture()
                self.assertEqual(len(captured["samples"]), len(expected))
                for actual, original in zip(captured["samples"], expected):
                    self.assertAlmostEqual(actual[0], original[0], delta=.000001)
                self.assertAlmostEqual(captured["samples"][0][0], OLDEST, delta=.000001)
                self.assertGreater(AFTER_PARSE - captured["samples"][0][0], .070)
                # move_begin starts a clean, strictly post-settle reference.
                obj.session_axes = {"x": None}
                stationary_stream(obj)
                obj._prepare_reference()
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNotNone(obj.preparation.baseline())
                self.assertGreater(obj.preparation.first, obj.preparation.cutoff)

    def test_processing_delay_preserves_wholly_old_valid_warmup_and_receipt(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = self.owner(legacy)
                self.delayed(obj, samples(35.017200, size=16))
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(obj.last_batch, AFTER_PARSE)
                self.assertAlmostEqual(obj.backend.last_sample_time, samples(35.017200, size=16)[-1][0])
                self.assertEqual(obj.backend.pending, [])

    def test_pending_singleton_survives_parse_delay_in_same_contiguous_sequence(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = self.owner(legacy)
                first = samples(OLDEST, size=1)
                obj.backend.helper.emit({"data": first})
                self.assertEqual(obj.backend.pending, first)
                rows = samples(OLDEST + 1 / 3200., size=200)
                self.delayed(obj, rows)
                expected = first + rows
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(obj.backend.pending, [])
                self.assertEqual(len(obj.recent[-1]), 3 * len(expected))
                self.assertEqual(obj.backend.unchanged_since, first[0][0])

    def test_warmup_does_not_trim_at_exact_70_or_80_ms(self):
        for age in (.070, .070000001, .080):
            with self.subTest(age=age):
                obj = self.owner()
                now = 1.0  # Every row belongs to this stream generation.
                rows = samples(now - age, size=8)
                obj.reactor.now = now
                self.delayed(obj, rows, after=now)
                expected = rows
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(len(obj.recent[-1]), 3 * len(expected))

    def test_measured_scopes_accept_reported_delay_without_timestamp_age_fault(self):
        for legacy in (False, True):
            for scope in ("armed", "reference", "noise", "beta_moving", "beta_home", "beta_tail", "tail",
                          "pending_move", "bound_capture"):
                with self.subTest(legacy=legacy, scope=scope):
                    obj = self.owner(legacy)
                    attempt = None
                    if scope == "armed":
                        baseline(obj, OLDEST - .060)
                        attempt, trigger = arm(obj, when=OLDEST, deadline=OLDEST + 10.)
                    elif scope == "reference":
                        obj.preparation = plugin.QuietReference("x", 35., .1, 100.)
                    elif scope == "noise":
                        obj._end_session()
                        obj.start_noise_measurement()
                        obj.noise_window = measured_window()
                    elif scope == "beta_moving":
                        obj._end_session()
                        obj.reactor.now = 35.
                        with mock.patch.object(obj, "collect_noise_window", return_value={
                                "samples": [(34.8 + n / 3200., 0., 0., 1.) for n in range(641)]}):
                            obj.start_beta_motion_capture(lambda: None)
                        obj.reactor.now = BEFORE_PARSE
                    elif scope in ("beta_home", "beta_tail"):
                        obj.start_capture(beta=True)
                        obj.beta_capture["tail_retained"] = scope == "beta_tail"
                    else:
                        if scope == "bound_capture":
                            obj.start_capture()
                            obj.capture_attempt = object()
                        elif scope == "tail":
                            baseline(obj, OLDEST - .060)
                            tail, _trigger = arm(obj, when=OLDEST, deadline=OLDEST + 10.)
                            tail.bases = obj._baseline()
                            tail.baseline = tail.bases[tail.index]
                            tail.last_samples = list(tail.bases)
                            tail.impact_until = obj.backend.print_time_to_clock(OLDEST + .2)
                            tail.record = {"impact_peak_g": 0.}
                            obj.attempt, obj.tail = None, tail
                        else:
                            setattr(obj, scope, object())
                    self.delayed(obj)
                    self.assertIsNone(obj.sensor_fault)
                    self.assertIsNone(obj.sample_timing_fault)
                    self.assertEqual(obj.last_batch, AFTER_PARSE)
                    self.assertAlmostEqual(obj.backend.last_age * 1000., 8.94346153846)
                    if attempt is not None:
                        self.assertIs(obj.attempt, attempt)
                        self.assertIsNone(attempt.reason)
                        self.assertEqual(trigger.messages, [])

    def test_measured_pending_singleton_is_not_trimmed(self):
        for scope in ("armed", "reference", "noise"):
            with self.subTest(scope=scope):
                obj = self.owner()
                if scope == "armed":
                    baseline(obj, OLDEST - .060)
                    attempt, trigger = arm(obj, when=OLDEST, deadline=OLDEST + 10.)
                elif scope == "reference":
                    obj.preparation = plugin.QuietReference("x", 35., .1, 100.)
                else:
                    obj._end_session()
                    obj.start_noise_measurement()
                    obj.noise_window = measured_window()
                obj.backend.helper.emit({"data": samples(OLDEST, size=1)})
                self.assertEqual(len(obj.backend.pending), 1)
                self.delayed(obj, samples(OLDEST + 1 / 3200., size=16))
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNone(obj.sample_timing_fault)
                self.assertEqual(obj.backend.pending, [])
                self.assertEqual(obj.last_batch, AFTER_PARSE)

    def test_warmup_does_not_hide_a_stale_interior_replay(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj = self.owner(legacy)
                rows = samples(35.0, size=2) + samples(35.080, size=2)
                rows += samples(35.0, size=1) + samples(35.080625, size=2)
                self.delayed(obj, rows)
                self.assertEqual(obj.sensor_fault, "sample_gap")
                self.assertIsNone(obj.last_batch)

    def test_warmup_keeps_future_shape_counters_and_retained_gaps_fatal(self):
        for mode in ("future", "shape", "nonfinite", "errors", "overflows", "gap", "rate"):
            with self.subTest(mode=mode):
                obj = self.owner()
                rows, fields = reported_rows(), {}
                if mode == "future":
                    rows = samples(AFTER_PARSE + .006)
                elif mode == "shape":
                    rows.insert(0, (35., 0., G))
                elif mode == "nonfinite":
                    rows.insert(0, (35., float("nan"), 0., G))
                elif mode in ("errors", "overflows"):
                    fields[mode] = 1
                elif mode == "gap":
                    rows = samples(35.080, size=2) + samples(35.084, size=2)
                else:
                    rows = samples(35.080, size=4, hz=800.)
                self.delayed(obj, rows, **fields)
                expected = ("stale_data" if mode == "future" else "invalid_data"
                            if mode in ("shape", "nonfinite") else "data_error"
                            if mode in ("errors", "overflows") else "sample_" + mode)
                self.assertEqual(obj.sensor_fault, expected)

    def test_receipt_watchdog_still_stops_after_50_ms_without_data(self):
        obj = self.owner()
        attempt, trigger = arm(obj, when=BEFORE_PARSE)
        obj.backend.helper.emit({"data": []})
        obj._watchdog(attempt.last_data + .050001)
        self.assertEqual(obj.stream_timeout, .050)
        self.assertEqual(attempt.reason, "no_data")
        self.assertEqual(trigger.messages, [[HOST]])


if __name__ == "__main__":
    unittest.main()
