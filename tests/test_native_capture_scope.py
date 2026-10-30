"""Native FILTERED capture boundaries and callback handoffs; no hardware."""
import copy
import contextlib
import threading
import time
import unittest
from unittest import mock

from accel_homing_filters import compare_filter_capture, validate_filtered_capture_prefix
from harness import CommandError, FakeEndstop, acquire_reference, batch, counts, make_homing, plugin
from test_chronological_clipping import native_filtered_capture
from test_session import FakeRail, info, with_endstops


class NativeCaptureScopeTests(unittest.TestCase):
    def machine(self, real_endstop=False):
        obj = make_homing(threshold=1.)
        obj.reactor.now = 10.
        endstop = with_endstops(obj)["x"] if real_endstop else FakeEndstop("x")
        if not real_endstop:
            endstop.get_steppers = lambda: []
        rail = FakeRail(endstop, info(speed=100., retract=0., second=100.))
        obj.session_axes = obj.rails_by_axis = {"x": rail}
        obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x",
                                 "algorithm": "impact20_vector_v2", "detector_types": {"x": "filtered"}}
        obj.workflows.in_home, obj.workflows.home_axis = True, "x"
        obj.start_capture()
        obj._start_stream()
        return obj, endstop, rail

    def arm_machine(self):
        obj, endstop, rail = self.machine()
        acquire_reference(obj, endstop)
        when = obj.preparation.last + .001
        attempt = obj.prepare(endstop, when)
        obj.arm(attempt)
        obj.reactor.now = when + .008
        packet = batch(round(when * 1000.), [(counts(3.), 0, counts(1.))] * 4
                       + [(32767, 0, counts(1.))] + [(counts(3.), 0, counts(1.))] * 3,
                       receive_time=obj.reactor.now, delta_clock=7)
        return obj, attempt, rail, packet

    def test_preparation_gap_is_outside_exact_accepted_reference(self):
        original = plugin.BeaconAccelHoming._prepare_reference
        def with_warmup(obj):
            obj.reactor.now = 10.008
            obj._inspect(batch(10000, [(0, 0, counts(1.))] * 8, receive_time=10.008))
            obj.reactor.now += .25
            original(obj)
        with mock.patch.object(plugin.BeaconAccelHoming, "_prepare_reference", with_warmup):
            capture, record = native_filtered_capture()
        self.assertEqual(record["result"], "impact")
        self.assertEqual(capture["detector"]["reference_sample_indices"][0], 0)
        self.assertEqual(capture["samples"][0][0], capture["detector"]["reference_interval_s"][0])
        scope = capture["_native_capture_scope"]
        self.assertGreater(scope["excluded_pre_reference_samples"], 8)
        self.assertEqual(scope["first_sample"], list(capture["detector"]["reference_sample_bounds"][0]))
        self.assertIsNone(validate_filtered_capture_prefix(capture, record))

    def test_original_partial_first_packet_identity_and_period_are_preserved(self):
        capture, record = native_filtered_capture(clock_hz=32000000., sample_period=.000302, arm_straddle=2)
        source = capture["_native_source_batches"][0]
        stamp = capture["_native_capture_scope"]["first_sample"]
        self.assertEqual(source["first_sample_position"], stamp[2])
        self.assertEqual(len(source["data"]) // 6, stamp[3])
        self.assertEqual((source["start_clock"], source["delta_clock"]), tuple(stamp[:2]))
        result = compare_filter_capture(capture, 1., 1.)
        self.assertEqual(result["filtered_decision"]["confirmation_time_s"], record["confirmation_time_s"])

    def test_empty_packets_refresh_neither_reference_nor_armed_watchdog(self):
        obj, endstop, _rail = self.machine()
        before = obj.last_batch
        obj._inspect(batch(10000, [], receive_time=10., delta_clock=0))
        self.assertEqual(obj.last_batch, before)
        self.assertIsNone(obj.sensor_fault)
        acquire_reference(obj, endstop)
        when = obj.preparation.last + .001
        attempt = obj.prepare(endstop, when)
        obj.arm(attempt)
        state = (obj.last_batch, attempt.last_data, attempt.sample_dt, attempt.first_armed_sample)
        obj.reactor.now += .01
        obj._inspect(batch(round(when * 1000.), [], receive_time=obj.reactor.now, delta_clock=0))
        self.assertEqual((obj.last_batch, attempt.last_data, attempt.sample_dt, attempt.first_armed_sample), state)
        self.assertFalse(attempt.done)
        self.assertIsNone(obj.capture_token["fault"])

    def test_old_short_warmup_after_new_open_does_not_poison_fresh_reference(self):
        obj, endstop, _rail = self.machine()
        before = obj.last_batch
        obj._inspect(batch(1000, [(0, 0, counts(1.))], receive_time=10., delta_clock=0))
        obj._inspect({"start_clock": 1000, "delta_clock": 1, "data": b"x", "#receive_time": 10.})
        self.assertEqual(obj.last_batch, before)
        self.assertIsNone(obj.capture_token["fault"])
        acquire_reference(obj, endstop)
        self.assertGreaterEqual(obj.preparation.count, 50)

    def test_admitted_short_warmup_cannot_become_a_new_reference_fault(self):
        singleton = batch(1000, [(0, 0, counts(1.))], delta_clock=0)["data"]
        for data, ignored in ((singleton, True), (b"x", True), (singleton * 2 + b"x", False)):
            obj, endstop, _rail = self.machine()
            entered, release = threading.Event(), threading.Event()
            original = obj._capture_batch
            def delayed(params, admission=None):
                if params["data"] is data:
                    entered.set()
                    self.assertTrue(release.wait(1.))
                original(params, admission)
            obj._capture_batch = delayed
            packet = {"start_clock": 1000, "delta_clock": 0, "data": data, "#receive_time": 10.}
            worker = threading.Thread(target=lambda: obj._inspect(packet))
            worker.start()
            self.assertTrue(entered.wait(1.))
            # The old packet is still in finally while a fresh reference is
            # acquired. It never contributed rows to that reference.
            acquire_reference(obj, endstop)
            self.assertIsNotNone(obj.capture_token["reference"])
            release.set()
            worker.join(1.)
            self.assertFalse(worker.is_alive())
            self.assertIsNone(obj.sensor_fault)
            if ignored:
                self.assertIsNone(obj.capture_token["fault"])
                attempt = obj.prepare(endstop, obj.preparation.last + .001)
                obj.arm(attempt)
                self.assertIs(obj.attempt, attempt)
            else:
                self.assertEqual(obj.capture_token["fault"], "invalid_data")
                with self.assertRaises(CommandError):
                    obj.prepare(endstop, obj.preparation.last + .001)

    def test_wellformed_pre_cutoff_flush_is_excluded_but_unknown_scope_is_fatal(self):
        for malformed in (False, True):
            obj, endstop, _rail = self.machine()
            from harness import stationary_stream
            stationary_stream(obj)
            flow = obj.reactor.on_pause
            injected = []
            def emit():
                if not injected:
                    injected.append(True)
                    packet = batch(1000, [(0, 0, counts(1.))] * 8, receive_time=obj.reactor.now)
                    if malformed:
                        packet["data"] += b"x"
                    obj._inspect(packet)
                flow()
            obj.reactor.on_pause = emit
            if malformed:
                with self.assertRaises(CommandError):
                    obj._prepare_reference()
                self.assertEqual(obj.capture_token["fault"], "invalid_data")
            else:
                obj._prepare_reference()
                self.assertIsNone(obj.capture_token["fault"])
                self.assertTrue(all(source[0] > 1000 for source in obj.capture_batches))

    def test_ready_then_malformed_before_prepare_or_arm_fails_closed(self):
        for after_prepare in (False, True):
            obj, endstop, _rail = self.machine()
            acquire_reference(obj, endstop)
            when = obj.preparation.last + .001
            attempt = obj.prepare(endstop, when) if after_prepare else None
            obj._inspect(batch(round(when * 1000.), [(0, 0, counts(1.))],
                               receive_time=obj.reactor.now, delta_clock=0))
            self.assertEqual(obj.capture_token["fault"], "invalid_data")
            with self.assertRaises(CommandError):
                obj.arm(attempt) if after_prepare else obj.prepare(endstop, when)
            self.assertIsNone(obj.attempt)
            self.assertFalse(endstop.trigger_log.messages)

    def test_close_after_trigger_before_append_drains_the_admitted_packet(self):
        obj, attempt, rail, packet = self.arm_machine()
        entered, release, waiting = (threading.Event() for _ in range(3))
        original, wait = obj._capture_batch, obj.capture_condition.wait
        errors, sealed = [], []
        def delayed(params, admission=None):
            self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)
            entered.set()
            self.assertTrue(release.wait(1.))
            saved_scale, saved_converter = obj.session_scale, obj.clock_to_time
            obj.session_scale = obj.clock_to_time = None
            try:
                original(params, admission)
            finally:
                obj.session_scale, obj.clock_to_time = saved_scale, saved_converter
        def wait_for_callback(timeout):
            waiting.set()
            return wait(timeout)
        obj._capture_batch, obj.capture_condition.wait = delayed, wait_for_callback
        worker = threading.Thread(target=lambda: obj._inspect(packet))
        worker.start()
        self.assertTrue(entered.wait(1.))
        obj.disarm()
        obj._handle_rails_end(None, [rail])
        def seal():
            try: sealed.append(obj.stop_capture())
            except Exception as exc: errors.append(exc)
        sealer = threading.Thread(target=seal)
        sealer.start()
        self.assertTrue(waiting.wait(1.))
        self.assertFalse(sealed)
        release.set()
        worker.join(1.)
        sealer.join(1.)
        self.assertFalse(worker.is_alive() or sealer.is_alive())
        self.assertFalse(errors)
        self.assertEqual(sealed[0]["_native_capture_scope"]["closed_by"], "stream_stop")
        self.assertIsNone(validate_filtered_capture_prefix(sealed[0], attempt.record))

    def test_reference_ready_before_append_survives_prepare_and_invalid_packet_cannot_arm(self):
        for malformed in (False, True):
            obj, endstop, _rail = self.machine(real_endstop=True)
            # A 128ms packet is valid only with an explicitly larger delivery
            # age limit; production freshness checks themselves are unchanged.
            obj.stream_timeout = .2
            entered, release, waiting = (threading.Event() for _ in range(3))
            original, wait = obj._capture_batch, obj.capture_condition.wait
            workers, errors = [], []
            def delayed(params, admission=None):
                entered.set()
                self.assertTrue(release.wait(1.))
                original(params, admission)
            def emit_large_reference():
                if workers:
                    return
                start = obj.preparation.cutoff + .001
                obj.reactor.now = start + .127
                packet = batch(round(start * 1000.), [(0, 0, counts(1.))] * 128,
                               receive_time=obj.reactor.now, delta_clock=127)
                if malformed:
                    packet["data"] += b"x"
                worker = threading.Thread(target=lambda: obj._inspect(packet))
                workers.append(worker)
                worker.start()
                self.assertTrue(entered.wait(1.))
            def wait_for_callback(timeout):
                self.assertFalse(endstop._dispatch.started)
                waiting.set()
                return wait(timeout)
            obj._capture_batch, obj.capture_condition.wait = delayed, wait_for_callback
            obj.reactor.on_pause = emit_large_reference
            obj._prepare_reference()
            when = obj.preparation.last + .001
            def arm():
                try: endstop.home_start(when, 0, 0, 0)
                except Exception as exc: errors.append(exc)
            armer = threading.Thread(target=arm)
            armer.start()
            self.assertTrue(waiting.wait(1.))
            self.assertIsNone(obj.attempt)
            self.assertIsNotNone(obj.preparation)
            release.set()
            workers[0].join(1.)
            armer.join(1.)
            self.assertFalse(workers[0].is_alive() or armer.is_alive())
            if malformed:
                self.assertEqual(len(errors), 1)
                self.assertIsInstance(errors[0], CommandError)
                self.assertIsNone(obj.attempt)
                self.assertEqual(obj.capture_token["fault"], "invalid_data")
                self.assertFalse(endstop._dispatch.started)
            else:
                self.assertFalse(errors)
                self.assertIsNotNone(obj.attempt)
                self.assertEqual(len(endstop._dispatch.started), 1)
                self.assertEqual(len(obj.capture_batches), 1)
                self.assertEqual(obj.capture_samples, 128)

    def test_home_start_drain_timeout_never_starts_dispatch(self):
        obj, endstop, _rail = self.machine(real_endstop=True)
        acquire_reference(obj, endstop)
        when = obj.preparation.last + .001
        obj.stream_timeout = .01
        # A stalled admitted callback must fail before any trigger starts.
        obj.capture_token["pending"] = 1
        with self.assertRaisesRegex(CommandError, "capture_inflight_timeout"):
            endstop.home_start(when, 0, 0, 0)
        self.assertFalse(endstop._dispatch.started)
        self.assertIsNone(obj.attempt)

    def test_final_arm_failure_always_stops_dispatch_even_if_disarm_raises(self):
        for installed in (False, True):
            obj, endstop, _rail = self.machine(real_endstop=True)
            acquire_reference(obj, endstop)
            when = obj.preparation.last + .001
            original_arm = obj.arm
            def fail(attempt):
                if installed:
                    original_arm(attempt)
                raise CommandError("original arm error")
            obj.arm = fail
            with mock.patch.object(obj, "disarm", side_effect=ValueError("disarm failed")), \
                    mock.patch.object(endstop._dispatch, "stop", wraps=endstop._dispatch.stop) as stop:
                with self.assertLogs(level="ERROR") if installed else contextlib.nullcontext():
                    with self.assertRaisesRegex(CommandError, "original arm error"):
                        endstop.home_start(when, 0, 0, 0)
                self.assertEqual(len(endstop._dispatch.started), 1)
                stop.assert_called_once_with()

    def test_prepared_attempt_cannot_arm_a_replaced_capture_or_stream_epoch(self):
        for replacement in ("reference", "epoch", "capture"):
            obj, endstop, _rail = self.machine()
            acquire_reference(obj, endstop)
            attempt = obj.prepare(endstop, obj.preparation.last + .001)
            if replacement == "reference":
                acquire_reference(obj, endstop)
            elif replacement == "capture":
                obj.stop_capture()
                obj.start_capture()
            if replacement != "reference":
                obj._start_stream()
            with self.assertRaisesRegex(CommandError, "context changed"):
                obj.arm(attempt)
            self.assertIsNone(obj.attempt)

    def test_closed_epoch_malformed_packets_cannot_mutate_record_or_capture(self):
        original = plugin.BeaconAccelHoming.disarm
        def close_then_late_flush(obj):
            original(obj)
            obj._handle_rails_end(None, list(obj.session_axes.values()))
            previous = copy.deepcopy(obj.last["x"])
            obj.reactor.now += .25
            obj._batch_hook({})
            obj._inspect(batch(round(obj.reactor.now * 1000.) - 7,
                               [(0, 0, counts(1.))] * 8, receive_time=obj.reactor.now))
            self.assertEqual(obj.last["x"], previous)
            self.assertIsNone(obj.sensor_fault)
        with mock.patch.object(plugin.BeaconAccelHoming, "disarm", close_then_late_flush):
            capture, record = native_filtered_capture(time_offset=10.)
        self.assertIsNone(validate_filtered_capture_prefix(capture, record))

    def test_seal_timeout_old_append_and_exception_cannot_fault_new_capture(self):
        for raises in (False, True):
            obj, _endstop, _rail = self.machine()
            obj.stream_timeout = .01
            entered, release = threading.Event(), threading.Event()
            original = obj._capture_batch
            original_clock = obj.clock_to_time
            def delayed(params, admission=None):
                entered.set()
                self.assertTrue(release.wait(1.))
                original(params, admission)
            if raises:
                def failed_conversion(clock):
                    entered.set()
                    self.assertTrue(release.wait(1.))
                    raise ValueError("old capture conversion failed")
                obj.clock_to_time = failed_conversion
            else:
                obj._capture_batch = delayed
            worker = threading.Thread(target=lambda: obj._batch_hook(batch(10000, [(0, 0, counts(1.))] * 8)))
            worker.start()
            self.assertTrue(entered.wait(1.))
            started = time.monotonic()
            old = obj.stop_capture()
            self.assertLess(time.monotonic() - started, .2)
            self.assertEqual(old["fault"], "capture_inflight_timeout")
            obj.clock_to_time = original_clock
            obj.start_capture()
            obj._start_stream()
            release.set()
            worker.join(1.)
            self.assertFalse(worker.is_alive())
            self.assertIsNone(obj.capture_token["fault"])
            self.assertIsNone(obj.sensor_fault)
            self.assertEqual(obj.capture_samples, 0)

    def test_changed_scope_or_missing_duplicate_reference_fails_closed(self):
        capture, record = native_filtered_capture()
        for mutate in (
                lambda current: current["_native_source_batches"][0].update(first_sample_position=0),
                lambda current: current["_native_source_batches"][0].update(admission_epoch=99),
                lambda current: current["_native_capture_scope"].update(first_sample=[1, 7, 0, 8]),
                lambda current: current["_native_source_batches"].insert(0, copy.deepcopy(current["_native_source_batches"][0]))):
            current = copy.deepcopy(capture)
            mutate(current)
            with self.assertRaises(ValueError):
                validate_filtered_capture_prefix(current, record)

    def test_seal_rejects_missing_or_duplicate_exact_reference_boundary(self):
        for duplicate in (False, True):
            obj, attempt, _rail, packet = self.arm_machine()
            obj._inspect(packet)
            obj.disarm()
            first = attempt.reference_sample_bounds[0]
            index = next(i for i, source in enumerate(obj.capture_batches)
                         if (source[0], source[1], len(source[2]) // 6) == (first[0], first[1], first[3]))
            for rows in (obj.capture_batches, obj.capture_receive_times, obj.capture_packet_times):
                rows.insert(index, rows[index]) if duplicate else rows.pop(index)
            captured = obj.stop_capture()
            self.assertIn("unique current reference boundary", captured["fault"])
            with self.assertRaises(ValueError):
                validate_filtered_capture_prefix(captured, attempt.record)

    def test_real_in_scope_gap_remains_fatal_with_bounded_numeric_pair(self):
        capture, record = native_filtered_capture(split=4)
        source = capture["_native_source_batches"][-1]
        source["filter_start_time_s"] += .02
        source["filter_end_time_s"] += .02
        with self.assertRaisesRegex(ValueError, r"sample gap.*packet=.*previous_end=.*next_start=.*gap=.*limit=.*epoch="):
            validate_filtered_capture_prefix(capture, record)


if __name__ == "__main__":
    unittest.main()
