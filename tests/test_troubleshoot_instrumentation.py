"""Passive observations preserve the real offline detector and stop contract."""
import types
import unittest
from unittest import mock

from harness import arm_axis, batch, counts, make_homing, plugin
from test_backends import arm, baseline, feed, generic, samples
from test_trigger import feed_baseline, still, x_sample
from accel_homing_troubleshoot import TimingTrace

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST


class Recorder:
    def __init__(self, log=None, raising=False):
        self.events, self.batches = [], []
        self.log = [] if log is None else log
        self.raising = raising

    def record_event(self, kind, **fields):
        self.log.append(kind)
        self.events.append((kind, fields))
        if self.raising:
            raise RuntimeError("optional recorder failed")

    def record_batch(self, source, **fields):
        self.log.append("batch:" + source)
        self.batches.append((source, fields))
        if self.raising:
            raise RuntimeError("optional recorder failed")

    def event(self, kind):
        return next(fields for name, fields in self.events if name == kind)

    def packet(self, source):
        return next(fields for name, fields in reversed(self.batches) if name == source)


class StopObservationTest(unittest.TestCase):
    def native(self, recorder=None, vector=False):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        attempt.vector_mode = vector
        attempt.bases = obj._baseline()
        attempt.last_samples = list(attempt.bases)
        attempt.moving = True
        obj.reactor.now = .210
        obj.troubleshoot = recorder
        return obj, attempt, trigger

    def test_native_scalar_and_vector_results_unchanged_with_raising_sink(self):
        for vector in (False, True):
            results = []
            for recorder in (None, Recorder(), Recorder(raising=True)):
                obj, attempt, trigger = self.native(recorder, vector)
                packet = batch(198, still(3) + [x_sample(-12)] + still(4),
                               receive_time=.210)
                packet["#sent_time"] = .209
                obj._batch_hook(packet)
                result = obj.finish(attempt.endstop, attempt, HIT)
                results.append((attempt.reason, attempt.onset_batch, attempt.onset_sample,
                                result, trigger.messages, attempt.impact_peak))
                if recorder is not None:
                    row = recorder.packet("beacon_native")
                    self.assertEqual(row["original_sample_count"], 8)
                    self.assertEqual(row["filtered_sample_count"], 6)
                    self.assertAlmostEqual(row["filtered_first_print_time_s"], .200)
                    self.assertEqual(row["driver_timestamp_fields"],
                                     {"#receive_time": .210, "#sent_time": .209})
                    self.assertFalse(row["hardware_receipt_time_available"])
                    self.assertEqual(row["callback_entry_estimated_print_time_s"], .210)
                    self.assertEqual(row["arm_print_time_s"], attempt.arm_time)
                    self.assertEqual(row["stream_generation"], obj.native_stream_generation)
                    self.assertIsNone(row["backend_generation"])
                    self.assertEqual(recorder.event("stop_request")["onset_sample"],
                                     attempt.onset_sample)
                    self.assertAlmostEqual(recorder.event("threshold_crossing")["onset_print_time_s"], .201)
            self.assertEqual(results[0], results[1])
            self.assertEqual(results[0], results[2])

    def test_configured_modern_legacy_scalar_vector_results_unchanged(self):
        for legacy in (False, True):
            for vector in (False, True):
                results = []
                for recorder in (None, Recorder(), Recorder(raising=True)):
                    obj = generic(legacy=legacy)
                    baseline(obj)
                    attempt, trigger = arm(obj)
                    attempt.vector_mode = vector
                    attempt.bases = obj._baseline()
                    attempt.last_samples = list(attempt.bases)
                    attempt.moving = True
                    obj.troubleshoot = recorder
                    rows = samples(10.060, size=20, x=12)
                    feed(obj, rows)
                    result = obj.finish(attempt.endstop, attempt, HIT)
                    results.append((attempt.reason, attempt.onset_sample, result,
                                    trigger.messages, attempt.request_time))
                    if recorder is not None:
                        row = recorder.packet("klipper_batch")
                        self.assertEqual(row["original_sample_count"], 20)
                        self.assertEqual(row["delivered_sample_count"], 20)
                        self.assertEqual(row["original_first_print_time_s"], rows[0][0])
                        self.assertEqual(row["original_last_print_time_s"], rows[-1][0])
                        self.assertEqual(row["driver_timestamp_fields"], {})
                        self.assertFalse(row["hardware_receipt_time_available"])
                        self.assertEqual(row["backend_generation"], obj.backend.generation)
                        self.assertEqual(row["arm_print_time_s"], attempt.arm_time)
                        detector = recorder.packet("klipper_batch_detector")
                        self.assertEqual(detector["callback_entry_estimated_print_time_s"], obj.reactor.now)
                        self.assertEqual(detector["arm_print_time_s"], attempt.arm_time)
                        self.assertEqual(detector["backend_generation"], obj.backend.generation)
                        self.assertEqual(recorder.event("stop_request")["dispatch_returned"], True)
                self.assertEqual(results[0], results[1])
                self.assertEqual(results[0], results[2])

    def test_stop_precedes_sink_and_print_clock_conversion_without_locking_sink(self):
        log = []
        recorder = Recorder(log, raising=True)
        obj, attempt, _trigger = self.native(recorder)
        attempt.motion_deadline = 1.0
        obj.main_mcu.estimated_print_time = lambda when: log.append("print_estimate") or when + 1234
        attempt.endstop.trigger = lambda code: log.append("stop")
        original = recorder.record_event
        def record(kind, **fields):
            self.assertFalse(obj.lock.locked())
            original(kind, **fields)
        recorder.record_event = record
        obj._fire(attempt, "no_data", HOST)
        self.assertEqual(log[0], "stop")
        self.assertLess(log.index("stop"), log.index("stop_request"))
        self.assertEqual(attempt.request_time, 1234.210)
        event = recorder.event("stop_request")
        self.assertEqual(event["arm_print_time_s"], attempt.arm_time)
        self.assertEqual(event["arm_sensor_clock"], attempt.arm_clock)
        self.assertEqual(event["request_estimated_print_time_s"], 1234.210)
        self.assertEqual(event["request_before_host_monotonic_s"], .210)
        self.assertEqual(event["request_after_host_monotonic_s"], .210)
        self.assertIsNone(event["mcu_halt_print_time_s"])

    def test_pipeline_timing_hooks_preserve_real_configured_detector_stop(self):
        for legacy in (False, True):
            results = []
            for enabled in (False, True):
                obj = generic(legacy=legacy)
                baseline(obj)
                attempt, trigger = arm(obj)
                attempt.vector_mode, attempt.moving = True, True
                attempt.bases = obj._baseline()
                attempt.last_samples = list(attempt.bases)
                recorder = Recorder()
                recorder.workflow = types.SimpleNamespace(owner=obj, reactor=obj.reactor)
                recorder.current_home = {"home_id": 1}
                trace = TimingTrace(recorder)
                obj.troubleshoot = recorder
                try:
                    if enabled:
                        trace.start()
                    feed(obj, samples(10.060, size=20, x=12))
                    result = obj.finish(attempt.endstop, attempt, HIT)
                    results.append((attempt.reason, attempt.onset_sample, result,
                                    trigger.messages, attempt.request_time))
                    if enabled:
                        self.assertIn("adapter_processing", trace.groups[1]["stages"])
                        self.assertIn("detector_with_observer", trace.groups[1]["stages"])
                finally:
                    trace.close()
            self.assertEqual(results[0], results[1])

    def test_trigger_exception_is_not_replaced_or_swallowed_by_sink(self):
        recorder = Recorder(raising=True)
        obj, attempt, _trigger = self.native(recorder)
        failure = ValueError("existing stop failure")
        def stop(code):
            raise failure
        attempt.endstop.trigger = stop
        with self.assertRaises(ValueError) as caught:
            obj._fire(attempt, "impact", HIT)
        self.assertIs(caught.exception, failure)
        self.assertFalse(recorder.event("stop_request")["dispatch_returned"])
        self.assertFalse(obj.lock.locked())

    def test_watchdog_records_deadline_after_dispatch_and_preserves_stop_with_failed_sink(self):
        results = []
        for recorder in (None, Recorder(), Recorder(raising=True)):
            obj, attempt, trigger = self.native(recorder, vector=True)
            attempt.last_data = obj.reactor.now
            obj.reactor.now += .087
            log = recorder.log if recorder else []
            original = attempt.endstop.trigger
            attempt.endstop.trigger = lambda code: (log.append("stop"), original(code))
            wake = obj._watchdog(obj.reactor.now)
            results.append((attempt.reason, trigger.messages, wake, attempt.request_time))
            if recorder:
                fields = recorder.event("watchdog_no_data")
                self.assertLess(log.index("stop"), log.index("watchdog_no_data"))
                self.assertAlmostEqual(fields["elapsed_s"], .087)
                self.assertEqual(fields["timeout_s"], obj.stream_timeout)
                self.assertEqual(fields["arm_print_time_s"], attempt.arm_time)
                self.assertEqual(fields["last_data_host_monotonic_s"], .210)
            self.assertEqual(trigger.messages, [[HOST]])
            self.assertEqual(wake, obj.reactor.NEVER)
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0], results[2])

    def test_stop_observation_keeps_dispatch_reason_if_final_reason_changes(self):
        recorder = Recorder()
        obj, attempt, _trigger = self.native(recorder)
        def estimate(when):
            # Model final-result reclassification on the other thread after
            # dispatch; the stop observation describes the dispatched request.
            attempt.reason = "pressed"
            return when
        obj.main_mcu.estimated_print_time = estimate
        obj._fire(attempt, "impact", HIT)
        self.assertEqual(recorder.event("stop_request")["reason"], "impact")
        self.assertEqual(attempt.reason, "pressed")

    def test_native_callback_local_clock_estimate_occurs_after_stop(self):
        log = []
        recorder = Recorder(log)
        obj, attempt, trigger = self.native(recorder)
        original = attempt.endstop.trigger
        attempt.endstop.trigger = lambda code: (log.append("stop"), original(code))
        obj.main_mcu.estimated_print_time = lambda when: log.append("print_estimate") or when + 1000.
        obj._batch_hook(batch(200, [x_sample(-12)] + still(7), receive_time=.210))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(log[0], "stop")
        self.assertEqual(recorder.packet("beacon_native")["callback_entry_estimated_print_time_s"], 1000.210)

    def test_native_late_callback_keeps_original_attempt_and_stream_generation(self):
        recorder = Recorder()
        obj, attempt, trigger = self.native(recorder)
        generation = obj.native_stream_generation
        obj.helper._stream_en = True
        def shared_handler(packet):
            obj.attempt = types.SimpleNamespace(axis="y", pass_no=1, arm_time=5., reason=None)
            obj.native_stream_generation += 1
        obj.saved_handler = shared_handler
        obj._batch_hook(batch(200, [x_sample(-12)] + still(7), receive_time=.210))
        self.assertEqual(trigger.messages, [[HIT]])
        row = recorder.packet("beacon_native")
        self.assertEqual(row["arm_print_time_s"], attempt.arm_time)
        self.assertEqual(row["axis"], "x")
        self.assertEqual(row["stream_generation"], generation)
        self.assertNotEqual(row["stream_generation"], obj.native_stream_generation)

    def test_disabled_diagnostics_do_not_call_observers(self):
        obj, attempt, trigger = self.native()
        with mock.patch.object(obj, "_troubleshoot_now", side_effect=AssertionError), \
                mock.patch.object(obj, "_troubleshoot_event", side_effect=AssertionError), \
                mock.patch.object(obj, "_troubleshoot_inspect", side_effect=AssertionError):
            obj._batch_hook(batch(200, [x_sample(-12)] + still(7), receive_time=.210))
        self.assertEqual(trigger.messages, [[HIT]])

    def test_watchdog_timeout_observed_after_original_stop(self):
        recorder = Recorder(raising=True)
        obj, attempt, trigger = self.native(recorder)
        attempt.last_data = .2
        obj.reactor.now = .3
        self.assertEqual(obj._watchdog(.3), obj.reactor.NEVER)
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(attempt.reason, "no_data")
        self.assertEqual(recorder.event("stop_request")["reason"], "no_data")


class BatchObservationTest(unittest.TestCase):
    def test_packet_sizes_and_pending_merge_preserve_full_precision_bounds(self):
        obj = generic()
        recorder = obj.troubleshoot = Recorder()
        first = 10.0
        for size in (1, 10, 20, 30):
            rows = samples(first, size=size)
            feed(obj, rows)
            row = recorder.packet("klipper_batch")
            self.assertEqual(row["original_sample_count"], size)
            self.assertEqual(row["original_first_print_time_s"], rows[0][0])
            self.assertEqual(row["original_last_print_time_s"], rows[-1][0])
            if size == 1:
                self.assertEqual(row["pending_after_sample_count"], 1)
                self.assertEqual(row["delivered_sample_count"], 0)
            elif size == 10:
                self.assertEqual(row["pending_before_sample_count"], 1)
                self.assertEqual(row["merged_sample_count"], 11)
                self.assertEqual(row["delivered_sample_count"], 11)
                self.assertEqual(row["delivered_first_print_time_s"], 10.0)
            else:
                self.assertEqual(row["delivered_sample_count"], size)
            first = rows[-1][0] + 1 / 3200.
        self.assertIsNone(obj.sensor_fault)

    def test_rejected_configured_batches_stop_before_optional_telemetry(self):
        for legacy in (False, True):
            for failure in ("errors", "overflows", "malformed", "future", "sample_gap"):
                with self.subTest(legacy=legacy, failure=failure):
                    obj = generic(legacy=legacy)
                    baseline(obj)
                    attempt, trigger = arm(obj)
                    log = []
                    recorder = obj.troubleshoot = Recorder(log, raising=True)
                    original = attempt.endstop.trigger
                    def stop(code):
                        log.append("stop")
                        original(code)
                    attempt.endstop.trigger = stop
                    rows, fields = samples(10.060), {}
                    if failure in ("errors", "overflows"):
                        fields[failure] = 1
                    elif failure == "malformed":
                        rows[3] = (rows[3][0], 1)
                    elif failure == "future":
                        rows = samples(10.160)
                    else:
                        rows = samples(10.080)
                    obj.reactor.now = 10.066
                    obj.backend.helper.emit({"data": rows, **fields})
                    self.assertEqual(trigger.messages, [[HOST]])
                    self.assertEqual(log[0], "stop")
                    row = recorder.packet("klipper_batch")
                    self.assertEqual(row["original_sample_count"], len(rows))
                    self.assertEqual(row["fault"], obj.sensor_fault)
                    self.assertEqual(row["delivered_sample_count"], 0)
                    if failure in ("errors", "overflows"):
                        self.assertEqual(row[failure], 1)
                    if failure == "malformed":
                        self.assertEqual(row["stage"], "callback_exception")

    def test_native_callback_exception_stops_before_rejected_batch_observation(self):
        obj = make_homing()
        feed_baseline(obj)
        attempt, trigger = arm_axis(obj, "x", 200, 9000)
        log = []
        recorder = obj.troubleshoot = Recorder(log, raising=True)
        original = attempt.endstop.trigger
        attempt.endstop.trigger = lambda code: (log.append("stop"), original(code))
        with mock.patch.object(obj, "_inspect_data", side_effect=ValueError("malformed")), \
                mock.patch.object(plugin.logging, "exception"):
            obj._batch_hook({"data": b"\x01"})
        self.assertEqual(trigger.messages, [[HOST]])
        self.assertEqual(log[0], "stop")
        row = recorder.packet("beacon_native")
        self.assertEqual(row["sensor_fault"], "invalid_data")
        self.assertEqual(row["detector_stage"], "callback_exception")
        self.assertFalse(row["original_bounds_available"])

    def test_pre_arm_rows_stay_excluded_with_or_without_observer(self):
        result = []
        for recorder in (None, Recorder(), Recorder(raising=True)):
            obj = generic()
            baseline(obj)
            attempt, trigger = arm(obj, when=10.061)
            obj.troubleshoot = recorder
            rows = samples(10.061 - 2 / 3200., size=10)
            feed(obj, rows)
            result.append((attempt.first_clock, attempt.last_end_clock, trigger.messages))
            if recorder is not None:
                row = recorder.packet("klipper_batch")
                self.assertEqual(row["original_sample_count"], 10)
                self.assertEqual(row["delivered_sample_count"], 8)
                self.assertEqual(row["delivered_first_print_time_s"], 10.061)
        self.assertEqual(result[0], result[1])
        self.assertEqual(result[0], result[2])


class LifecycleObservationTest(unittest.TestCase):
    def test_arm_uses_cached_reference_and_existing_clock(self):
        obj = make_homing()
        feed_baseline(obj)
        recorder = obj.troubleshoot = Recorder()
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        event = recorder.event("arm")
        self.assertEqual(event["arm_print_time_s"], .2)
        self.assertEqual(event["threshold_g"], obj.threshold_g["x"])
        self.assertEqual(event["backend"], "beacon_native")
        self.assertEqual(event["host_monotonic_s"], attempt.last_data)

    def test_home_end_includes_all_counts_without_fabricating_halt_time(self):
        obj = make_homing()
        recorder = obj.troubleshoot = Recorder()
        attempt, _trigger = arm_axis(obj, "x", 200, 9000)
        attempt.reason = "no_data"
        attempt.record = {"axis": "x", "pass": 1, "result": "no_data",
                          "onset_g": None, "impact_peak_g": None,
                          "noise_pos_g": 0., "noise_neg_g": 0.,
                          "travel_mm": None, "trigger_after_arm_s": None}
        obj.pending_move = attempt
        obj.reactor.now = 42.
        stepper = types.SimpleNamespace(get_step_dist=lambda: .01)
        position = types.SimpleNamespace(stepper=stepper, stepper_name="stepper_x",
                                         start_pos=100, trig_pos=140, halt_pos=148)
        missing = types.SimpleNamespace(stepper=stepper, stepper_name="stepper_y",
                                        start_pos=200, trig_pos=None, halt_pos=201)
        move = types.SimpleNamespace(get_mcu_endstops=lambda: [attempt.endstop],
                                     stepper_positions=[position, missing])
        with mock.patch.object(obj, "_travel", side_effect=[.4, .48, .48]), \
                mock.patch.object(plugin, "describe", return_value="synthetic test"):
            obj._handle_move_end(move)
        event = recorder.event("home_end")
        self.assertEqual(event["arm_print_time_s"], attempt.arm_time)
        self.assertEqual(event["arm_sensor_clock"], attempt.arm_clock)
        self.assertEqual(event["callback_host_monotonic_s"], 42.)
        self.assertEqual(len(event["steppers"]), 2)
        self.assertEqual(event["steppers"][0]["start_steps"], 100)
        self.assertEqual(event["steppers"][0]["trigger_steps"], 140)
        self.assertEqual(event["steppers"][0]["halt_steps"], 148)
        self.assertEqual(event["steppers"][0]["step_distance_mm"], .01)
        self.assertEqual(event["steppers"][0]["halt_mm"], 1.48)
        self.assertIsNone(event["steppers"][1]["trigger_steps"])
        self.assertIsNone(event["mcu_halt_print_time_s"])
        self.assertEqual(event["halt_travel_mm_signed"], .48)
        self.assertIs(obj.last_pass, attempt)
        self.assertIsNone(obj.pending_move)


if __name__ == "__main__":
    unittest.main()
