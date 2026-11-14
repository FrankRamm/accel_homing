"""Offline delivery comparisons and transparent existing-timer observation."""
import csv
import gzip
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import types
import unittest
from unittest import mock

import harness
import accel_homing_troubleshoot as diagnostics
from test_troubleshoot_timing import Pipeline


class TimerClock:
    NEVER = 1e30
    def __init__(self):
        self.now, self.timers = 10., []
        self.update_result = object()
        self.update_failure = None
    def monotonic(self):
        return self.now
    def register_timer(self, callback, when):
        timer = types.SimpleNamespace(callback=callback, waketime=when, timer_is_running=False)
        self.timers.append(timer)
        return timer
    def update_timer(self, timer_handler, waketime):
        if self.update_failure is not None:
            raise self.update_failure
        if not timer_handler.timer_is_running:
            timer_handler.waketime = waketime
        return self.update_result
    def unregister_timer(self, timer):
        timer.waketime = self.NEVER
        self.timers.remove(timer)
    def dispatch(self, timer, now):
        self.now = now
        timer.waketime = self.NEVER
        timer.timer_is_running = True
        try:
            result = timer.callback(now)
            timer.waketime = result
            return result
        finally:
            timer.timer_is_running = False


class ExistingTimerTests(unittest.TestCase):
    def fixture(self):
        pipeline = Pipeline()
        pipeline.clock = TimerClock()
        pipeline.recorder.workflow.reactor = pipeline.clock
        pipeline.owner.attempt = types.SimpleNamespace(done=False)
        pipeline.helper.batch_timer = None
        pipeline.helper.batch_interval = .005
        backend = pipeline.owner.backend
        backend.timer_attr = "batch_timer"
        backend.interval_attr = "batch_interval"
        backend.active = False
        pipeline.timer_calls = 0
        pipeline.timer_during = lambda: None
        pipeline.timer_result = None
        def callback(eventtime):
            pipeline.timer_calls += 1
            pipeline.timer_during()
            pipeline.run()
            return eventtime + .005 if pipeline.timer_result is None else pipeline.timer_result
        pipeline.original_timer = callback
        def start():
            backend.active = True
            if pipeline.helper.batch_timer is None:
                pipeline.helper.batch_timer = pipeline.clock.register_timer(callback, pipeline.clock.now + .005)
            pipeline.clock.update_timer(pipeline.helper.batch_timer, pipeline.clock.now + .005)
        backend.start = start
        pipeline.trace = diagnostics.TimingTrace(pipeline.recorder)
        self.addCleanup(pipeline.trace.close)
        pipeline.trace.start()
        backend.start()
        return pipeline

    def timer_row(self, pipeline):
        return [row for row in pipeline.rows() if row["stage"] == "driver_timer"][-1]

    def test_actual_initial_returned_and_updated_deadlines_are_observed_without_new_batch_timer(self):
        pipeline = self.fixture()
        pipeline.delay = 0.
        timer = pipeline.helper.batch_timer
        self.assertEqual(len(pipeline.clock.timers), 2)  # Existing batch + existing diagnostic heartbeat.
        self.assertAlmostEqual(pipeline.clock.dispatch(timer, 10.012), 10.017)
        first = self.timer_row(pipeline)
        self.assertAlmostEqual(first["details"]["scheduled_host_monotonic_s"], 10.005)
        self.assertAlmostEqual(first["details"]["lateness_s"], .007)
        pipeline.clock.dispatch(timer, 10.020)
        self.assertAlmostEqual(self.timer_row(pipeline)["details"]["scheduled_host_monotonic_s"], 10.017)
        pipeline.clock.update_timer(timer, 10.100)
        pipeline.clock.dispatch(timer, 10.102)
        self.assertAlmostEqual(self.timer_row(pipeline)["details"]["lateness_s"], .002)
        self.assertEqual(pipeline.timer_calls, 3)
        self.assertEqual(pipeline.queries, 3)
        pipeline.trace.close()
        self.assertIs(timer.callback, pipeline.original_timer)
        self.assertEqual(len(pipeline.clock.timers), 1)

    def test_ignored_running_update_does_not_fabricate_new_deadline(self):
        pipeline = self.fixture()
        pipeline.delay = 0.
        timer = pipeline.helper.batch_timer
        pipeline.timer_during = lambda: pipeline.clock.update_timer(timer, 90.)
        pipeline.clock.dispatch(timer, 10.005)
        pipeline.timer_during = lambda: None
        pipeline.clock.dispatch(timer, 10.014)
        self.assertAlmostEqual(self.timer_row(pipeline)["details"]["scheduled_host_monotonic_s"], 10.010)
        self.assertAlmostEqual(self.timer_row(pipeline)["details"]["lateness_s"], .004)

    def test_update_timer_forwards_original_positional_and_keyword_contract(self):
        pipeline = self.fixture()
        timer = pipeline.helper.batch_timer
        self.assertIs(pipeline.clock.update_timer(timer, 20.), pipeline.clock.update_result)
        self.assertEqual(timer.waketime, 20.)
        self.assertIs(pipeline.clock.update_timer(timer_handler=timer, waketime=21.), pipeline.clock.update_result)
        pipeline.clock.dispatch(timer, 21.003)
        self.assertAlmostEqual(self.timer_row(pipeline)["details"]["lateness_s"], .003)
        pipeline.clock.update_failure = RuntimeError("original update failure")
        with self.assertRaises(RuntimeError) as caught:
            pipeline.clock.update_timer(timer_handler=timer, waketime=22.)
        self.assertIs(caught.exception, pipeline.clock.update_failure)

    def test_never_unknown_and_original_exception_are_forwarded(self):
        pipeline = self.fixture()
        timer = pipeline.helper.batch_timer
        unknown = object()
        pipeline.timer_result = unknown
        self.assertIs(pipeline.clock.dispatch(timer, 10.005), unknown)
        self.assertIsNone(self.timer_row(pipeline)["details"]["next_deadline_s"])
        pipeline.timer_result = pipeline.clock.NEVER
        pipeline.clock.dispatch(timer, 10.010)
        self.assertIsNone(self.timer_row(pipeline)["details"]["scheduled_host_monotonic_s"])
        pipeline.failure = ValueError("original driver error")
        with self.assertRaises(ValueError) as caught:
            pipeline.clock.dispatch(timer, 10.015)
        self.assertIs(caught.exception, pipeline.failure)
        self.assertFalse(self.timer_row(pipeline)["details"]["returned"])

    def test_cancellation_reentrant_timer_replacement_and_foreign_callback_restoration(self):
        pipeline = self.fixture()
        timer = pipeline.helper.batch_timer
        def cancel():
            pipeline.trace.close()
        pipeline.timer_during = cancel
        self.assertAlmostEqual(pipeline.clock.dispatch(timer, 10.005), 10.010)
        self.assertIs(timer.callback, pipeline.original_timer)
        self.assertEqual(pipeline.timer_calls, 1)
        self.assertNotIn("driver_timer", [row["stage"] for row in pipeline.rows()])
        other = self.fixture()
        old = other.helper.batch_timer
        other.helper.batch_timer = None
        other.owner.backend.start()
        self.assertIs(old.callback, other.original_timer)
        replacement = lambda eventtime: eventtime + 1
        other.helper.batch_timer.callback = replacement
        other.trace.close()
        self.assertIs(other.helper.batch_timer.callback, replacement)

    def test_timer_observer_failure_does_not_change_original_return_or_exception(self):
        pipeline = self.fixture()
        timer = pipeline.helper.batch_timer
        with mock.patch.object(pipeline.trace, "note", side_effect=RuntimeError("observer failed")):
            self.assertAlmostEqual(pipeline.clock.dispatch(timer, 10.005), 10.010)
            pipeline.failure = ValueError("original")
            with self.assertRaises(ValueError) as caught:
                pipeline.clock.dispatch(timer, 10.010)
            self.assertIs(caught.exception, pipeline.failure)
        self.assertGreater(pipeline.trace.errors, 0)

    def test_armed_aggregates_block_metadata_and_bookkeeping_remain_bounded(self):
        pipeline = self.fixture()
        pipeline.delay = .004
        pipeline.queue.data = [dict(sequence=i, data=b"secret-payload", **{"#receive_time": 9.9}) for i in range(100)]
        with mock.patch.object(diagnostics, "MAX_TIMING_ROWS", 5), mock.patch.object(diagnostics, "MAX_SLOW_TIMING_ROWS", 2):
            pipeline.clock.dispatch(pipeline.helper.batch_timer, 10.006)
            pipeline.owner.attempt.done = True
            pipeline.clock.dispatch(pipeline.helper.batch_timer, 10.012)
        pipeline.trace.close()
        with tempfile.TemporaryDirectory() as directory:
            report = pipeline.trace.export(Path(directory))
            with gzip.open(Path(directory) / "timing.jsonl.gz", "rt") as stream:
                rows = [json.loads(line) for line in stream]
        stats = report["homes"]["1"]
        self.assertEqual(stats["armed_entry_stages"]["driver_timer"]["count"], 1)
        self.assertEqual(stats["stages"]["driver_timer"]["count"], 2)
        self.assertEqual(stats["queue_block_provenance"]["observed"], 102)
        self.assertEqual(stats["queue_block_provenance"]["omitted_per_call_limit"], 37)
        self.assertEqual(stats["queue_block_provenance"]["omitted_total"],
                         102 - stats["queue_block_provenance"]["retained_exported"])
        self.assertGreater(stats["bookkeeping_overhead"]["armed_entry_calls"]["count"], 0)
        serialized = json.dumps(rows)
        self.assertNotIn("secret-payload", serialized)
        for row in rows:
            self.assertLessEqual(len(row["details"].get("blocks", [])), diagnostics.MAX_QUEUE_BLOCKS)

    def test_native_detector_observes_real_inspect_data_entry(self):
        pipeline = Pipeline()
        pipeline.owner.backend = None
        calls = []
        pipeline.owner._inspect_data = lambda data: calls.append(data) or data
        trace = diagnostics.TimingTrace(pipeline.recorder)
        try:
            trace.start()
            payload = object()
            self.assertIs(pipeline.owner._inspect_data(payload), payload)
            self.assertEqual(calls, [payload])
            self.assertEqual(trace.groups[1]["stages"]["detector_with_observer"]["count"], 1)
        finally:
            trace.close()

    def test_overlapping_call_context_bound_reports_omissions(self):
        pipeline = self.fixture()
        tokens = [pipeline.trace._enter_call("sensor_pull", (10., 1.), 1) for _ in range(40)]
        self.assertEqual(sum(token is not None for token in tokens), 32)
        self.assertEqual(pipeline.trace.context_calls_omitted, 8)
        with tempfile.TemporaryDirectory() as directory:
            report = pipeline.trace.export(Path(directory))
        self.assertEqual(report["max_active_context_calls"], 32)
        self.assertEqual(report["context_calls_omitted"], 8)


def comparison_fixture():
    homes = []
    for identifier, variant, pair, delay in ((1, "control", None, .010), (12, "control", 1, .012),
                                           (13, "fast", 1, .008), (14, "fast", 2, None)):
        homes.append({"home_id": identifier, "axis": "y", "phase": "delivery_" + variant,
            "comparison_pair": pair, "comparison_order": 1 if variant == "control" else 2,
            "delivery_variant": variant, "delivery_planned_variant": variant, "delivery_supported": True,
            "delivery_control_interval_s": .005, "delivery_requested_interval_s": .002 if variant == "fast" else .005,
            "delivery_effective_interval_s": .002 if variant == "fast" else .005,
            "delivery_intervention_applied": variant == "fast", "quality": "good",
            "delivery_observation": {"count": 3, "consistent": True},
            "record": {"halt_travel_mm_signed": -10.5, "trigger_travel_mm_signed": -10.},
            "stop_timing": {"onset_to_host_dispatch_estimate_s": delay}, "delivery_diagnostic": {"segments": {}}})
    return {"homes": homes, "timing": {"homes": {}}, "experiment": {"mode": "COMPARE",
        "delivery_comparison": {"supported": True, "intervention_observed": True,
                                "restoration": {"complete": True}},
        "shadow_validation": {"status": "rejected", "live_use_authorized": False},
        "noise_margin_policy": {"fraction": .1, "minimum_g": .5}}}


class DeliveryComparisonTests(unittest.TestCase):
    def test_observation_uses_actual_active_intervals_and_mismatch_is_sticky(self):
        backend = types.SimpleNamespace(active=True, interval=.002, applied_interval=.002,
            interval_attr="batch_interval", helper=types.SimpleNamespace(batch_interval=.002))
        recorder = diagnostics.Recorder.__new__(diagnostics.Recorder)
        recorder.workflow = types.SimpleNamespace(owner=types.SimpleNamespace(backend=backend))
        home = {"delivery_variant": "fast", "delivery_control_interval_s": .005}
        recorder._delivery_observation(home)
        self.assertEqual(home["delivery_effective_interval_s"], .002)
        self.assertTrue(home["delivery_intervention_applied"])
        backend.active = False
        recorder._delivery_observation(home)
        self.assertEqual(home["delivery_observation"]["count"], 1)
        self.assertEqual(home["delivery_observation"]["inactive_count"], 1)
        backend.active = True
        backend.helper.batch_interval = .004
        recorder._delivery_observation(home)
        backend.helper.batch_interval = .002
        recorder._delivery_observation(home)
        self.assertFalse(home["delivery_observation"]["consistent"])
        self.assertIsNone(home["delivery_effective_interval_s"])
        self.assertFalse(home["delivery_intervention_applied"])

    def test_report_uses_verified_pairs_negative_delta_and_missing_not_zero(self):
        report = comparison_fixture()
        result = diagnostics.comparison_report(report)
        self.assertEqual(result["groups"]["control"]["homes"], 1)
        self.assertEqual(result["groups"]["fast"]["homes"], 2)
        self.assertEqual(result["groups"]["fast"]["metrics"]["onset_to_host_dispatch_estimate_ms"]["missing"], 1)
        self.assertEqual(result["pairs"][0]["delta_fast_minus_control"]["onset_to_host_dispatch_estimate_ms"], -4.)
        self.assertFalse(result["pairs"][1]["matched"])
        self.assertIsNone(result["pairs"][1]["delta_fast_minus_control"]["onset_to_host_dispatch_estimate_ms"])
        report["homes"][2]["delivery_observation"]["consistent"] = False
        self.assertFalse(diagnostics.comparison_report(report)["pairs"][0]["matched"])

    def test_unsupported_never_becomes_control_or_fast_comparison(self):
        report = comparison_fixture()
        for home in report["homes"]:
            home.update(delivery_variant="unavailable", delivery_supported=False, delivery_intervention_applied=False)
        report["experiment"]["delivery_comparison"]["supported"] = False
        result = diagnostics.comparison_report(report)
        self.assertEqual(result["groups"]["unavailable"]["homes"], 3)
        self.assertEqual(result["groups"]["control"]["homes"], 0)
        self.assertTrue(all(not pair["matched"] for pair in result["pairs"]))

    def test_export_blanks_and_standalone_margin_closest_pair_survive(self):
        report = comparison_fixture()
        margin = {"qualified": False, "startup": {"weighted_margin_g": .4,
            "closest_pair": {"evidence_sample_index": 17, "decision_sample_index": 18}}}
        experiment = report["experiment"]
        experiment["fit"] = {"shadow": {"training": [{"noise_margin": margin}], "reason": "insufficient margin"}}
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            diagnostics.export_comparison(folder, report)
            diagnostics.export_noise_margins(folder, experiment, [])
            with (folder / "comparison.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            margins = json.loads((folder / "noise-margin.json").read_text())
            summary = (folder / "tester-summary.txt").read_text()
        self.assertEqual(rows[3]["onset_to_host_dispatch_estimate_ms"], "")
        self.assertEqual(rows[4]["onset_to_host_dispatch_estimate_ms"], "-4.0")
        self.assertEqual(margins["assessments"][0]["assessment"]["startup"]["closest_pair"]["decision_sample_index"], 18)
        self.assertIn("not physical carriage travel", summary)
        self.assertIn("rejected", summary)

    def test_cached_environment_filters_secrets_bounds_objects_and_uses_no_mutators(self):
        calls = []
        status = types.SimpleNamespace(get_status=lambda now: calls.append(now) or {"temperature": 35., "target": 0., "secret": "hidden"})
        workflow = types.SimpleNamespace(printer=types.SimpleNamespace(lookup_objects=lambda: [
            ("temperature_sensor sensor%d" % index, status) for index in range(40)]))
        result = diagnostics._home_environment(workflow, 10.)
        self.assertEqual(result["objects_omitted"], 8)
        self.assertEqual(len(calls), 32)
        self.assertNotIn("secret", json.dumps(result))

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "matplotlib unavailable")
    def test_real_comparison_render_contains_negative_pair_and_unavailable_distribution(self):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from accel_homing_charts import _check_png
        report = comparison_fixture()
        snapshots = []
        original = Figure.savefig
        def save(figure, path, *args, **kwargs):
            snapshots.extend(figure.axes[1].lines[1].get_ydata())
            return original(figure, path, *args, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with mock.patch.object(Figure, "savefig", save):
                diagnostics._plot_comparison(folder, report, Figure, FigureCanvasAgg)
            payload = (folder / "comparison.png").read_bytes()
            self.assertGreater(len(payload), 10000)
            self.assertGreater(_check_png(payload)[0], 1000)
        self.assertEqual(snapshots, [-4.])


if __name__ == "__main__":
    unittest.main()
