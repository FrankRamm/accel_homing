"""Offline regression evidence for range limits and the host delivery pipeline."""
from array import array
import copy
import csv
import gc
import gzip
import json
from pathlib import Path
import tempfile
import threading
import types
import unittest
from unittest import mock

import harness  # Load the plugin with offline Klipper stubs.
import accel_homing_troubleshoot as diagnostics


def adxl_context(rate=3200):
    return {"sensor": {"configured_sensor_class": "extras.adxl345.ADXL345",
        "cached_backend": {"full_scale": 16.}, "configured_sensor_cached": {
            "data_rate": rate, "axes_map": [[1, -37.0102971], [0, 37.0102971], [2, 38.3047749]]}}}


class SaturationDiagnosticsTests(unittest.TestCase):
    def home(self):
        return {"record": {"arm_time_s": 1., "onset_time_s": 2., "amplitude_limited": False,
                           "peak_semantics": "reported_sensor_derived"},
                "capture": {"amplitude_limited": False, "dropped": 0, "raw_rows_omitted": 0}}

    def test_native_limits_below_16g_with_signed_mapping_and_phases(self):
        profile = diagnostics.adxl_saturation_profile(adxl_context())
        raw = array("d")
        for when, counts in ((.9, [4094, 0, 0]), (1.5, [0, -4096, 0]),
                             (2., [0, 0, 4094]), (2.1, [0, 0, -4096]), (2.2, [0, 0, 0])):
            raw.extend([when] + [count * pair[1] / 9806.65 for count, pair in zip(counts, profile["axes_map"])])
        self.assertLess(max(abs(value) for value in raw), 16.)
        public = self.home()
        original_record, original_capture = public["record"], public["capture"]
        before = copy.deepcopy(public)
        raw_before = list(raw)
        diagnostics.diagnose_saturation(public, raw, profile, 2.05)
        quality = public["saturation_diagnostic"]
        self.assertEqual(quality["range_limit_axis_counts"], {"x": 1, "y": 1, "z": 2})
        self.assertEqual([quality["phases"][phase][axis] for phase, axis in
                         (("before_arm", "x"), ("armed_before_onset", "y"),
                          ("after_onset_before_stop", "z"), ("after_stop", "z"))], [1, 1, 1, 1])
        self.assertTrue(quality["complete_capture"])
        self.assertFalse(quality["physical_peak_recoverable"])
        self.assertTrue(public["record"]["amplitude_limited"])
        self.assertFalse(public["record"]["runtime_signal_quality"]["amplitude_limited"])
        self.assertEqual(original_record, before["record"])
        self.assertEqual(original_capture, before["capture"])
        self.assertEqual(list(raw), raw_before)

    def test_rate_changes_endpoint_and_nearby_samples_do_not_clip(self):
        for rate, counts, expected in ((3200, 4094, True), (1600, 4094, True),
                                       (800, 4094, False), (800, 4095, True),
                                       (3200, 4092, False), (3200, -4094, False)):
            with self.subTest(rate=rate, counts=counts):
                profile = diagnostics.adxl_saturation_profile(adxl_context(rate))
                raw = array("d", [1.5, counts * profile["axes_map"][0][1] / 9806.65, 0, 0])
                public = self.home()
                diagnostics.diagnose_saturation(public, raw, profile, None)
                self.assertEqual(public["saturation_diagnostic"]["amplitude_limited"], expected)

    def test_unknown_sensor_range_or_mapping_never_guesses_limits(self):
        for modify in (lambda s: s.update(configured_sensor_class="extras.lis2dw.LIS2DW"),
                       lambda s: s["cached_backend"].update(full_scale=8.),
                       lambda s: s["configured_sensor_cached"].update(data_rate=None),
                       lambda s: s["configured_sensor_cached"].update(axes_map=[[0, 0]] * 3)):
            context = adxl_context()
            modify(context["sensor"])
            profile = diagnostics.adxl_saturation_profile(context)
            public = self.home()
            original = copy.deepcopy(public["record"])
            diagnostics.diagnose_saturation(public, array("d", [1., 16., 16., 16.]), profile, None)
            self.assertFalse(public["saturation_diagnostic"]["available"])
            self.assertEqual(public["record"], original)

    def test_invalid_or_omitted_raw_is_not_a_complete_negative(self):
        for flag in ("dropped", "raw_rows_omitted", "invalid_rows", "retention_error", "unavailable"):
            public = self.home()
            public["capture"][flag] = 1
            diagnostics.diagnose_saturation(public, array("d", [float("nan"), 15.458304, 0, 0]),
                                           diagnostics.adxl_saturation_profile(adxl_context()), None)
            quality = public["saturation_diagnostic"]
            self.assertFalse(quality["complete_capture"])
            self.assertEqual(quality["phases"]["unattributed"]["x"], 1)
            self.assertIsNone(quality["first_print_time_s"])
            json.dumps(public, allow_nan=False)


class DeliveryDiagnosticsTests(unittest.TestCase):
    def fixture(self):
        home = {"home_id": 2, "axis": "y", "phase": "holdout", "batches_omitted": 7,
                "stop_timing": {"onset_print_time_s": 10., "host_dispatch_estimated_print_time_s": 10.01061,
                                "onset_to_host_dispatch_estimate_s": .01061}}
        events = [{"home_id": 2, "kind": "stop_request", "fields": {
            "request_before_host_monotonic_s": 20.0106, "request_after_host_monotonic_s": 20.0107,
            "request_estimated_print_time_s": 10.01061, "arm_print_time_s": 9.9,
            "onset_sample": [1234, 240, 4, 10]}}]
        common = {"arm_print_time_s": 9.9, "sample_time_domain": "main_mcu_print_time",
                  "original_first_print_time_s": 9.998, "original_last_print_time_s": 10.006,
                  "original_sample_count": 10}
        detector = {"home_id": 2, "home_packet_sequence": 8, "source": "klipper_batch_detector", "fields": {
            **common, "original_start_clock": 1234, "original_delta_clock": 240,
            "callback_entry_host_monotonic_s": 20.0104, "callback_exit_host_monotonic_s": 20.0108}}
        adapter = {"home_id": 2, "home_packet_sequence": 9, "source": "klipper_batch", "fields": {
            **common, "timestamp_provenance": "configured_driver_decoded_batch_callback",
            "callback_entry_host_monotonic_s": 20.010, "callback_exit_host_monotonic_s": 20.011,
            "callback_entry_estimated_print_time_s": 10.010}}
        return home, events, [detector, adapter]

    def test_decomposition_uses_named_clocks_and_preserves_evidence(self):
        args = self.fixture()
        before = copy.deepcopy(args)
        result = diagnostics.delivery_diagnostic(*args)
        self.assertEqual(args, before)
        self.assertEqual(result["status"], "complete")
        for key, seconds in (("onset_to_batch_last", .006), ("batch_last_to_adapter", .004),
                             ("adapter_to_detector", .0004), ("detector_to_request", .0002)):
            self.assertAlmostEqual(result["segments"][key]["duration_s"], seconds)
            self.assertEqual(result["segments"][key]["status"], "available")
        self.assertAlmostEqual(result["decomposition_residual_s"], .00001)
        self.assertAlmostEqual(result["stop_call_duration_s"], .0001)
        self.assertIn("clocksync estimate", result["segments"]["batch_last_to_adapter"]["clock_basis"])
        self.assertEqual(result["segments"]["detector_to_request"]["clock_basis"], "host_monotonic")
        self.assertIsNone(result["mcu_halt_time_s"])
        self.assertIsNone(result["physical_contact_time_s"])
        self.assertEqual(result["packet_retention_omitted"], 7)
        json.dumps(result, allow_nan=False)

    def test_pending_source_gap_and_missing_adapter_remain_unknown(self):
        for pending_gap in (False, True):
            with self.subTest(pending_gap=pending_gap):
                home, events, packets = self.fixture()
                if pending_gap:
                    packets[1]["fields"].update(original_first_print_time_s=10.001,
                        pending_before_first_print_time_s=9.998, pending_before_last_print_time_s=10.,
                        delivered_first_print_time_s=9.998, delivered_last_print_time_s=10.006)
                else:
                    packets.pop()
                result = diagnostics.delivery_diagnostic(home, events, packets)
                self.assertEqual(result["status"], "partial")
                self.assertAlmostEqual(result["segments"]["detector_to_request"]["duration_s"], .0002)
                self.assertIsNone(result["segments"]["batch_last_to_adapter"]["duration_s"])
                self.assertIsNone(result["decomposition_residual_s"])

    def test_identity_wrong_home_arm_clock_and_ambiguous_callbacks_are_not_guessed(self):
        mutations = (
            lambda h, e, p: p[0]["fields"].update(original_start_clock=4321),
            lambda h, e, p: p[0].update(home_id=3),
            lambda h, e, p: p[0]["fields"].update(arm_print_time_s=9.8),
            lambda h, e, p: p[0]["fields"].update(sample_time_domain="sensor_clock"),
            lambda h, e, p: p.append(copy.deepcopy(p[0])),
            lambda h, e, p: h.update(stop_timing={"reason": "clock mapping unavailable"}),
            lambda h, e, p: e[0]["fields"].update(request_before_host_monotonic_s=None))
        for mutate in mutations:
            args = self.fixture()
            mutate(*args)
            result = diagnostics.delivery_diagnostic(*args)
            self.assertEqual(result["status"], "unavailable")
            self.assertTrue(all(item["duration_s"] is None for item in result["segments"].values()))
        home, events, packets = self.fixture()
        packets.append(copy.deepcopy(packets[1]))
        result = diagnostics.delivery_diagnostic(home, events, packets)
        self.assertEqual(result["status"], "partial")
        self.assertIn("ambiguous", result["segments"]["adapter_to_detector"]["reason"])

    def test_reversed_or_missing_clocksync_estimate_is_not_zero(self):
        for estimate, status in ((9.99, "inconsistent"), (None, "unavailable")):
            home, events, packets = self.fixture()
            packets[1]["fields"]["callback_entry_estimated_print_time_s"] = estimate
            result = diagnostics.delivery_diagnostic(home, events, packets)
            self.assertEqual(result["status"], "partial")
            segment = result["segments"]["batch_last_to_adapter"]
            self.assertEqual(segment["status"], status)
            self.assertIsNone(segment["duration_s"])
            self.assertIsNone(result["decomposition_residual_s"])

    def test_native_only_reports_observed_detector_span_without_fabricating_adapter(self):
        home, events, packets = self.fixture()
        packets[0]["source"] = "beacon_native"
        result = diagnostics.delivery_diagnostic(home, events, packets[:1])
        self.assertEqual(result["status"], "partial")
        self.assertEqual(result["evidence"]["detector_source"], "beacon_native")
        self.assertIsNone(result["segments"]["adapter_to_detector"]["duration_s"])

    def test_portable_exports_keep_unknowns_null_or_blank_and_name_clock_bases(self):
        home, events, packets = self.fixture()
        home["delivery_diagnostic"] = diagnostics.delivery_diagnostic(home, events, packets[:1])
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            diagnostics.export_delivery(folder, [home])
            report = json.loads((folder / "delivery.json").read_text(encoding="utf-8"))
            with (folder / "delivery.csv").open(newline="", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))
        self.assertIsNone(report["homes"][0]["segments"]["batch_last_to_adapter"]["duration_s"])
        self.assertEqual(row["batch_last_to_adapter_ms"], "")
        self.assertEqual(row["mcu_halt_time_s"], "")
        self.assertAlmostEqual(float(row["detector_to_request_ms"]), .2)
        self.assertEqual(row["detector_to_request_clock_basis"], "host_monotonic")


class Clock:
    NEVER = 1e30
    def __init__(self):
        self.now, self.timers = 10., []
    def monotonic(self):
        return self.now
    def register_timer(self, callback, when):
        timer = (callback, when)
        self.timers.append(timer)
        return timer
    def unregister_timer(self, timer):
        self.timers.remove(timer)


class Queue:
    def __init__(self):
        self.data = []
    def _handle_data(self, params):
        self.data.append(params)
    def pull_queue(self):
        result, self.data = self.data, []
        return result


class Pipeline:
    """The verified BatchBulkHelper/FixedFreqReader/serial handler contracts."""
    def __init__(self):
        self.clock, self.queue = Clock(), Queue()
        self.failure, self.delay, self.queries = None, .087, 0
        self.params = {"sequence": 123, "#receive_time": 10.002, "data": b"abc"}
        self.query_result = {"#sent_time": 10., "#receive_time": 10.087,
                             "clock": 1000, "buffered": 0, "next_sequence": 124}
        self.serial = types.SimpleNamespace(handlers={("sensor_bulk_data", 7): self.queue._handle_data}, lock=threading.Lock())
        self.query = types.SimpleNamespace(send=self.send)
        self.reader = types.SimpleNamespace(query_status_cmd=self.query, bulk_queue=self.queue,
            mcu=types.SimpleNamespace(_serial=self.serial), oid=7, pull_samples=self.pull)
        self.sensor = types.SimpleNamespace(ffreader=self.reader, _convert_samples=lambda rows: rows)
        self.helper = types.SimpleNamespace(batch_cb=self.batch)
        self.owner = types.SimpleNamespace(_inspect=lambda rows: rows)
        self.owner.backend = types.SimpleNamespace(helper=self.helper, sensor=self.sensor,
            _receive=lambda rows: self.owner._inspect(rows), _troubleshoot_receive=lambda *args: None)
        self.recorder = types.SimpleNamespace(workflow=types.SimpleNamespace(owner=self.owner, reactor=self.clock),
            current_home={"home_id": 1}, record_batch=lambda *a, **k: None, record_event=lambda *a, **k: None)
        self.trace = diagnostics.TimingTrace(self.recorder)

    def send(self, args):
        self.queries += 1
        self.clock.now += .003
        # Serial messages can continue arriving while the existing query yields.
        with self.serial.lock:
            self.serial.handlers[("sensor_bulk_data", 7)](self.params)
        self.clock.now += self.delay - .003
        if self.failure:
            raise self.failure
        return self.query_result

    def pull(self):
        self.reader.query_status_cmd.send([7])
        return self.reader.bulk_queue.pull_queue()

    def batch(self, eventtime):
        return {"data": self.sensor._convert_samples(self.reader.pull_samples())}

    def run(self):
        result = self.helper.batch_cb(self.clock.now)
        return self.owner.backend._receive(result)

    def rows(self):
        return list(self.trace.groups.get(1, {}).get("rows", ()))


class PipelineTimingTests(unittest.TestCase):
    def pipeline(self):
        pipeline = Pipeline()
        self.addCleanup(pipeline.trace.close)
        return pipeline

    def test_delayed_existing_query_is_localized_without_extra_queries(self):
        p = self.pipeline()
        reference = self.pipeline()
        expected = reference.run()
        p.trace.start()
        self.assertEqual(p.run(), expected)
        self.assertEqual(p.queries, 1)
        self.assertEqual(p.clock.now, reference.clock.now)
        rows = {row["stage"]: row for row in p.rows()}
        self.assertAlmostEqual(rows["sensor_status_query"]["duration_s"], .087)
        self.assertAlmostEqual(rows["driver_batch"]["duration_s"], .087)
        self.assertEqual(rows["driver_decode"]["duration_s"], 0.)
        self.assertEqual(rows["detector_with_observer"]["duration_s"], 0.)
        self.assertEqual(rows["mcu_bulk_handler"]["details"]["serial_receive_host_monotonic_s"], 10.002)
        self.assertEqual(rows["driver_queue_drain"]["details"]["first_sequence"], 123)
        self.assertEqual(rows["sensor_status_query"]["details"]["next_sequence"], 124)

    def test_late_driver_dispatch_is_distinct_from_query_time(self):
        p = self.pipeline()
        p.delay = .003
        p.trace.start()
        p.run()
        p.clock.now += .084
        p.run()
        driver = [row for row in p.rows() if row["stage"] == "driver_batch"][-1]
        query = [row for row in p.rows() if row["stage"] == "sensor_status_query"][-1]
        self.assertAlmostEqual(driver["entry_gap_s"], .087)
        self.assertAlmostEqual(query["duration_s"], .003)

    def test_heartbeat_and_gc_distinguish_host_pause_and_restore_without_gc_control(self):
        p = self.pipeline()
        before = (gc.isenabled(), gc.get_threshold(), list(gc.callbacks))
        p.trace.start()
        self.assertEqual(len(p.clock.timers), 1)
        p.trace._gc("start", {"generation": 2})
        p.clock.now += .087
        p.trace._gc("stop", {"generation": 2, "collected": 1, "uncollectable": 0})
        next_wake = p.clock.timers[0][0](p.clock.now)
        heartbeat = [row for row in p.rows() if row["stage"] == "reactor_heartbeat"][-1]
        self.assertAlmostEqual(heartbeat["details"]["lateness_s"], .067)
        self.assertAlmostEqual(next_wake - p.clock.now, .020)
        self.assertTrue(any(row["stage"] == "gc" and abs(row["duration_s"] - .087) < 1e-8 for row in p.rows()))
        p.trace.close()
        p.trace.close()
        self.assertEqual(p.clock.timers, [])
        self.assertEqual((gc.isenabled(), gc.get_threshold(), list(gc.callbacks)), before)

    def test_observer_failure_preserves_original_objects_exceptions_and_cleanup(self):
        p = self.pipeline()
        originals = (p.helper.batch_cb, p.query.send, p.serial.handlers[("sensor_bulk_data", 7)])
        p.trace.start()
        with mock.patch.object(p.trace, "note", side_effect=RuntimeError("optional observer broken")):
            result = p.run()
            self.assertIs(result["data"][0], p.params)
            p.failure = ValueError("driver query failed")
            with self.assertRaises(ValueError) as caught:
                p.run()
            self.assertIs(caught.exception, p.failure)
        p.trace.close()
        self.assertEqual((p.helper.batch_cb, p.query.send, p.serial.handlers[("sensor_bulk_data", 7)]), originals)
        self.assertNotIn("pull_queue", p.queue.__dict__)
        count = len(p.rows())
        p.serial.handlers[("sensor_bulk_data", 7)](p.params)
        self.assertEqual(len(p.rows()), count)

    def test_cleanup_does_not_overwrite_other_clients_handler_or_method(self):
        p = self.pipeline()
        p.trace.start()
        replacement = lambda *args: None
        p.helper.batch_cb = replacement
        p.serial.handlers[("sensor_bulk_data", 7)] = replacement
        p.trace.close()
        self.assertIs(p.helper.batch_cb, replacement)
        self.assertIs(p.serial.handlers[("sensor_bulk_data", 7)], replacement)

    def test_failed_partial_install_detaches_and_marks_unavailable(self):
        p = self.pipeline()
        original = p.helper.batch_cb
        with mock.patch.object(p.trace, "_serial", side_effect=ValueError("incompatible driver")):
            p.trace.start()
        self.assertIs(p.helper.batch_cb, original)
        self.assertTrue(p.trace.closed)
        self.assertEqual(p.trace.available["startup_error"], "ValueError")
        self.assertNotIn(p.trace.gc_callback, gc.callbacks)

    def test_closing_during_a_yielded_query_retains_inflight_evidence_and_result(self):
        p = self.pipeline()
        original = p.query.send
        def cancel_during_query(args):
            p.trace.close()
            return original(args)
        p.query.send = cancel_during_query
        p.trace.start()
        self.assertIs(p.run()["data"][0], p.params)
        self.assertEqual({row["stage"] for row in p.trace.active_at_close}, {"driver_batch", "sensor_status_query"})
        self.assertTrue(all(row["home_id"] == 1 for row in p.trace.active_at_close))
        self.assertIs(p.query.send, cancel_during_query)
        self.assertEqual(p.clock.timers, [])

    def test_bounded_rows_keep_slow_call_and_report_all_omissions(self):
        p = self.pipeline()
        with mock.patch.object(diagnostics, "MAX_TIMING_ROWS", 4), mock.patch.object(diagnostics, "MAX_SLOW_TIMING_ROWS", 2):
            p.trace.note("query", (10., 1.), (10.087, 1.001))
            for index in range(10):
                p.trace.note("query", (10.09 + index * .005, 1.), (10.091 + index * .005, 1.001))
        p.trace.close()
        with tempfile.TemporaryDirectory() as folder:
            report = p.trace.export(Path(folder))
            with gzip.open(Path(folder) / "timing.jsonl.gz", "rt") as stream:
                rows = [json.loads(line) for line in stream]
        self.assertLessEqual(len(rows), 6)
        self.assertTrue(any(abs(row["duration_s"] - .087) < 1e-8 for row in rows))
        home = report["homes"]["1"]
        self.assertEqual(home["observed"], 11)
        self.assertEqual(home["omitted"], 11 - len(rows))
        self.assertAlmostEqual(home["stages"]["query"]["max_duration_s"], .087)


if __name__ == "__main__":
    unittest.main()
