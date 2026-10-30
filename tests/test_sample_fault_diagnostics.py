"""Bounded fault provenance and unchanged rejection rules; no printer access."""
import json
import math
import unittest
from unittest import mock

from accel_homing_filters import CausalImpactDetector
from harness import CommandError, arm_axis, batch, counts, make_homing, plugin
from test_noise_recovery import sensor
from test_workflows import GCmd


class HomingFaultDiagnosticsTests(unittest.TestCase):
    def machine(self):
        obj = make_homing(threshold=1.)
        attempt, sent = arm_axis(obj, "x", 1000, 2000)
        attempt.vector_mode, attempt.detector_type = True, "filtered"
        attempt.bases = (0., 0., 128.)
        attempt.filter_engine = CausalImpactDetector(1., (0., 0., 1.))
        obj.reactor.now = 1.008
        return obj, attempt, sent

    def packet(self, obj, rows=None, **changes):
        params = batch(1001, rows or [(0, 0, 128)] * 8)
        params.update(changes)
        obj._inspect(params)
        return params

    def test_native_sentinel_and_individual_fullscale_report_different_exact_causes(self):
        for value, cause in ((plugin.CLIPPED, "native_clip_sentinel"),
                             (counts(16.), "axis_full_scale")):
            with self.subTest(cause=cause):
                obj, attempt, sent = self.machine()
                self.packet(obj, [(0, 0, 128), (0, value, 128)] + [(0, 0, 128)] * 6)
                self.assertEqual(attempt.reason, "invalid_data")
                self.assertEqual(attempt.sample_fault["cause"], cause)
                self.assertEqual(attempt.sample_fault["phase"], "homing_xyz")
                self.assertEqual(attempt.sample_fault["sample_position"], 1)
                self.assertEqual(attempt.sample_fault["sensor_axis"], "y")
                self.assertEqual(attempt.sample_fault["raw_value"], value)
                self.assertEqual(len(sent.messages), 1)
                self.assertEqual(sent.messages[0][0], plugin.mcu.MCU_trsync.REASON_HOST_REQUEST)
                json.dumps(attempt.sample_fault, allow_nan=False)

    def test_vector_magnitude_at_or_above_sensor_axis_range_is_not_called_clipping(self):
        for axis_g in (10., 11.3, 12.5):
            with self.subTest(axis_g=axis_g):
                obj, attempt, _sent = self.machine()
                row = (counts(axis_g), counts(axis_g), 128)
                self.packet(obj, [(0, 0, 128)] + [row] * 7)
                self.assertEqual(attempt.reason, "impact")
                self.assertIsNone(attempt.sample_fault)
                self.assertLess(abs(row[0] * attempt.count_g), obj.full_scale_g)
                self.assertGreater(attempt.impact_peak * attempt.count_g, 14.)

    def test_native_filtered_reports_the_first_unusable_row_chronologically(self):
        obj, attempt, _sent = self.machine()
        raw = tuple([counts(16.), 0., 128., 0., float("nan"), 128.] + [0., 0., 128.] * 6)
        self.packet(obj, data=raw)
        self.assertEqual(attempt.sample_fault["cause"], "axis_full_scale")
        self.assertEqual(attempt.sample_fault["sample_position"], 0)
        self.assertEqual(attempt.sample_fault["sensor_axis"], "x")
        self.assertEqual(attempt.sample_fault["raw_value"], counts(16.))

    def test_zero_or_nonfinite_packet_interval_still_stops_with_precise_fault(self):
        for delta in (0, float("nan")):
            with self.subTest(delta=delta):
                obj, attempt, _sent = self.machine()
                self.packet(obj, delta_clock=delta)
                self.assertEqual(attempt.reason, "invalid_data")
                self.assertEqual(attempt.sample_fault["cause"], "nonpositive_or_nonfinite_interval")
                json.dumps(attempt.sample_fault, allow_nan=False)

    def test_repeated_and_backward_native_packet_clocks_remain_rejected(self):
        for previous in (1001, 1002):
            obj, attempt, _sent = self.machine()
            attempt.last_end_clock = previous
            self.packet(obj)
            self.assertEqual(attempt.reason, "invalid_data")
            self.assertEqual(attempt.sample_fault["cause"], "nonmonotonic_packet_clock")
            self.assertEqual(attempt.sample_fault["previous_end_clock"], previous)

    def test_nonfinite_clock_conversion_is_reported_without_nonfinite_json(self):
        obj, attempt, _sent = self.machine()
        obj.clock_to_time = lambda _clock: float("nan")
        self.packet(obj)
        self.assertEqual(attempt.sample_fault["cause"], "nonfinite_packet_time")
        self.assertIsNone(attempt.sample_fault["first_sample_time_s"])
        json.dumps(attempt.sample_fault, allow_nan=False)

    def test_filter_nonmonotonic_time_and_real_gap_keep_same_stop_behavior(self):
        for previous, cause in ((1.002, "filter_nonmonotonic_time"), (.990, "filter_sample_gap")):
            obj, attempt, _sent = self.machine()
            attempt.filter_engine.previous_time = previous
            self.packet(obj)
            self.assertEqual(attempt.reason, "invalid_data")
            self.assertEqual(attempt.sample_fault["cause"], cause)
            self.assertEqual(attempt.sample_fault["previous_sample_time_s"], previous)
            self.assertIn("sample_dt_s", attempt.sample_fault)

    def test_filter_timestamp_boundary_and_rejected_state_are_unchanged(self):
        engine = CausalImpactDetector(1., (0., 0., 1.))
        engine.previous_time = 0.
        engine.feed(.00625, (0., 0., 1.))
        self.assertEqual(engine.previous_time, .00625)
        for when in (.00625, .006, .012500000001):
            with self.subTest(when=when), self.assertRaises(ValueError):
                engine.feed(when, (0., 0., 1.))
            self.assertEqual(engine.previous_time, .00625)

    def test_stop_dispatch_precedes_fault_metadata_conversion_json_and_logging(self):
        obj, attempt, sent = self.machine()
        class BadPacket(dict):
            def get(self, *args):
                self_test.assertEqual(len(sent.messages), 1)
                return super().get(*args)
        self_test = self
        original_json = plugin.json.dumps
        def checked_json(*args, **kwargs):
            self.assertEqual(len(sent.messages), 1)
            return original_json(*args, **kwargs)
        def checked_log(*args, **kwargs):
            self.assertEqual(len(sent.messages), 1)
        with mock.patch.object(plugin.json, "dumps", side_effect=checked_json), \
                mock.patch.object(plugin.logging, "info", side_effect=checked_log):
            obj._reject_homing_sample(attempt, "batch_callback", "inspection_exception",
                                       BadPacket(start_clock=1001, delta_clock=7, data=b"0" * 48))
        self.assertIsNotNone(attempt.sample_fault)

    def test_missing_packet_fields_keep_base_cause_and_callback_exception(self):
        obj, attempt, sent = self.machine()
        obj.saved_handler = lambda _params: None
        obj._batch_hook({"data": b"0" * 48})
        self.assertEqual(attempt.reason, "invalid_data")
        self.assertEqual(len(sent.messages), 1)
        self.assertEqual(attempt.sample_fault["cause"], "inspection_exception")
        self.assertEqual(attempt.sample_fault["exception_type"], "KeyError")
        self.assertIsNone(attempt.sample_fault["batch_start_clock"])

    def test_diagnostic_conversion_or_logger_failures_cannot_suppress_the_stop(self):
        for failure in ("json", "logger", "exception_text"):
            with self.subTest(failure=failure):
                obj, attempt, sent = self.machine()
                class BadError(ValueError):
                    def __str__(self):
                        raise RuntimeError("unavailable text")
                context = (mock.patch.object(plugin.json, "dumps", side_effect=RuntimeError("bad JSON"))
                           if failure == "json" else
                           mock.patch.object(plugin.logging, "info", side_effect=RuntimeError("bad logger")))
                with context:
                    obj._reject_homing_sample(attempt, "batch_callback", "inspection_exception",
                                               error=BadError() if failure == "exception_text" else ValueError("fault"))
                self.assertEqual(attempt.reason, "invalid_data")
                self.assertEqual(len(sent.messages), 1)

    def test_first_fault_is_immutable_and_survives_record_console_and_status(self):
        obj, attempt, sent = self.machine()
        self.packet(obj, [(0, plugin.CLIPPED, 128)] + [(0, 0, 128)] * 7)
        first = dict(attempt.sample_fault)
        obj._reject_homing_sample(attempt, "homing_packet", "nonmonotonic_packet_clock")
        self.assertEqual(attempt.sample_fault, first)
        self.assertEqual(len(sent.messages), 1)
        obj.disarm()
        self.assertEqual(attempt.record["sample_fault"], first)
        with self.assertRaisesRegex(CommandError, "sensor reported a clipped axis sample"):
            obj.finish(attempt.endstop, attempt, plugin.mcu.MCU_trsync.REASON_HOST_REQUEST)
        command = GCmd()
        obj.cmd_STATUS(command)
        self.assertTrue(any("native_clip_sentinel" in line for line in command.responses))


class StationaryCadenceDiagnosticsTests(unittest.TestCase):
    CASES = ((.000301857142857, .000343495535714, .000313125109641),
             (.000301486607142, .000345035714285, .00030196656644))

    def owner_packet(self, decoded, previous, current, gap):
        obj = sensor(decoded)
        obj.start_noise_measurement()
        frequency = obj.beacon_freq
        start_clock = round(10.01 * frequency)
        start = obj.clock_to_time(start_clock)
        delta = round(current * 7 * frequency)
        current = delta / 7. / frequency
        obj.reactor.now = start + 7 * current + .001
        obj.noise_window = {"index": 1, "complete": False, "cutoff": start - .1,
                            "last": start - gap, "interval": previous, "duration": 1.,
                            "samples": [(start - gap, 0., 0., 1.)], "changed": False}
        params = {"start_clock": start_clock, "delta_clock": delta,
                  "data": tuple(v for _ in range(8) for v in (0., 0., 128.))
                  if decoded else batch(start_clock, [(0, 0, 128)] * 8, delta_clock=delta)["data"]}
        return obj, params, current

    def test_both_operator_numeric_cases_still_reject_cadence_change_and_label_it(self):
        for decoded in (False, True):
            for previous, current, gap in self.CASES:
                with self.subTest(decoded=decoded, current=current):
                    # Native fake clock normally uses ms; use the installed
                    # Beacon's 32 MHz clock to reproduce the observed ratios.
                    obj = sensor(decoded)
                    if not decoded:
                        obj.beacon_freq = 32000000.
                        obj.clock_to_time = lambda clock: clock / 32000000.
                    obj.start_noise_measurement()
                    freq = obj.beacon_freq
                    start_clock, delta = round(10.01 * freq), round(current * 7 * freq)
                    start, actual_interval = obj.clock_to_time(start_clock), delta / 7. / freq
                    obj.reactor.now = start + 7 * actual_interval + .001
                    obj.noise_window = {"index": 1, "complete": False, "cutoff": start - .1,
                                        "last": start - gap, "interval": previous, "duration": 1.,
                                        "samples": [(start - gap, 0., 0., 1.)], "changed": False}
                    data = (tuple(v for _ in range(8) for v in (0., 0., 1000.)) if decoded else
                            batch(start_clock, [(0, 0, 128)] * 8, delta_clock=delta)["data"])
                    obj._inspect_noise_batch({"start_clock": start_clock, "delta_clock": delta, "data": data})
                    detail = obj.noise_window["fault_details"]
                    self.assertEqual(obj.sensor_fault, "sample_gap")
                    self.assertEqual(detail["cause"], "interval_change")
                    self.assertAlmostEqual(detail["gap_s"], gap, places=11)
                    self.assertLess(detail["gap_s"], detail["gap_limit_s"])
                    self.assertGreater(detail["interval_change_s"], detail["interval_change_limit_s"])
                    self.assertEqual(obj.toolhead.moves, [])
                    obj.stop_noise_measurement()

    def test_real_gap_duplicate_stale_and_low_rate_remain_rejected(self):
        for decoded in (False, True):
            for failure in ("gap", "duplicate", "stale", "low_rate"):
                with self.subTest(decoded=decoded, failure=failure):
                    current = .002 if failure == "low_rate" else .0003125
                    gap = 6 * current if failure == "gap" else 0. if failure == "duplicate" else current
                    obj, params, _interval = self.owner_packet(decoded, current, current, gap)
                    if failure == "stale": obj.reactor.now += .1
                    obj._inspect_noise_batch(params)
                    self.assertEqual(obj.sensor_fault, {"gap": "sample_gap", "duplicate": "invalid_data",
                                                       "stale": "stale_data", "low_rate": "sample_rate"}[failure])
                    if failure == "gap":
                        self.assertEqual(obj.noise_window["fault_details"]["cause"], "actual_gap")
                    obj.stop_noise_measurement()

    def test_beta_preserves_interval_change_real_gap_repeats_stale_low_rate_and_clip_guards(self):
        for failure in ("interval_change", "gap", "duplicate", "stale", "low_rate", "clip"):
            with self.subTest(failure=failure):
                obj = make_homing()
                obj.beacon_freq = 32000000.
                obj.clock_to_time = lambda clock: clock / 32000000.
                obj.start_capture(beta=True)
                previous = .000301857142857
                interval = .000343495535714 if failure == "interval_change" else .002 if failure == "low_rate" else previous
                gap = 6 * interval if failure == "gap" else 0. if failure == "duplicate" else previous
                start_clock, delta = round(10.01 * obj.beacon_freq), round(interval * 7 * obj.beacon_freq)
                start = obj.clock_to_time(start_clock)
                obj.beta_capture.update(last_sample_time_s=start - gap, interval_s=previous)
                obj.reactor.now = start + delta / obj.beacon_freq + (.1 if failure == "stale" else .001)
                rows = [(0, 0, 128)] * 8
                if failure == "clip": rows[1] = (plugin.CLIPPED, 0, 128)
                obj._capture_batch(batch(start_clock, rows, delta_clock=delta))
                self.assertEqual(obj.beta_capture["fault"], {"interval_change": "sample_gap", "gap": "sample_gap",
                                                             "duplicate": "sample_gap", "stale": "stale_data",
                                                             "low_rate": "sample_rate", "clip": "invalid_data"}[failure])
                obj.stop_capture()


if __name__ == "__main__":
    unittest.main()
