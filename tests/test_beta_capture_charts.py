"""Beta capture/chart regression sources. Not executed during implementation."""
import copy
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, batch, counts, make_homing, plugin
from test_accuracy_charts import RecordingRenderer, charts


def beta_event():
    rows = [(index / 1000., 0., 0., 1.) for index in range(51)]
    for index, value in ((2, 15.), (7, .8), (9, .9), (10, 1.05), (12, 3.),
                         (25, 4.), (26, 12.), (30, 14.)):
        rows[index] = (index / 1000., value, 0., 1.)
    record = {"run_id": "beta_event", "axis": "x", "workflow_attempt": 1,
              "workflow_phase": "discovery", "result": "impact", "threshold_g": 1.,
              "noise_g": .25, "impact_peak_g": 15., "onset_g": 1.05, "travel_mm": 150.}
    capture = {"samples": rows, "baseline_g": (0., 0., 1.), "dropped": 0, "fault": None,
               "detector": {"axis": "x", "impact_metric": "xyz_magnitude", "attempt_count": 1,
                            "arm_time_s": .005, "threshold_g": 1., "onset_time_s": .010,
                            "onset_sample_index": 10, "onset_sample_matches": 1},
               "beta_capture": {"version": 1, "raw_samples_validated": True,
                                "quality": {"sensor_context": {"sensor": "simulated"}, "fault": None},
                                "stop_response": {"boundary_source": "host_observed_queue_completion",
                                                  "boundary_time_s": .028, "window_s": .015,
                                                  "available": True, "complete": True}}}
    return record, capture


def beta_motion(offset=0.):
    rows = [(offset + index / 5000., 0., 0., 1.) for index in range(5001)]
    rows[122] = (rows[122][0], 1., 0., 1.)
    rows[123] = (rows[123][0], .9, .9, 1.)
    rows[124] = (rows[124][0], 0., 1., 1.)
    rows[4500] = (rows[4500][0], 5., 0., 1.)
    record = {"axis": "x", "run_id": "beta_event", "configured_speed_mm_s": 100.,
              "effective_speed_mm_s": 75., "effective_accel_mm_s2": 2000.,
              "segments": [{"phase": phase, "axis": "x", "start_time_s": offset + first,
                            "end_time_s": offset + last, "start_velocity_mm_s": velocity,
                            "acceleration_mm_s2": accel}
                           for phase, first, last, velocity, accel in (
                               ("startup", 0., .1, 0., 0.), ("acceleration", .1, .2, 0., 750.),
                               ("cruise", .2, .8, 75., 0.), ("braking", .8, 1., 75., -375.))]}
    context = {"sensor": "simulated", "backend": "generic", "full_scale_g": 16.}
    capture = {"samples": rows, "baseline_g": (0., 0., 1.), "raw_samples_validated": True,
               "dropped": 0, "fault": None, "sensor_context": context, "_noise_context": context}
    return {"record": record, "capture": capture}


class BetaEventTelemetryTests(unittest.TestCase):
    def test_exact_prefix_crossing_and_window_clip_without_legacy_mutation(self):
        obj = make_homing()
        record, capture = beta_event()
        old_record = copy.deepcopy(record)
        telemetry = obj.beta_home_telemetry(capture, record)
        self.assertEqual(record, old_record)
        self.assertEqual(telemetry["pre_onset_noise"]["sample_count"], 5)
        self.assertAlmostEqual(telemetry["pre_onset_noise"]["magnitude_peak_g"], .9)
        self.assertEqual(telemetry["first_crossing"]["sample_index"], 10)
        self.assertAlmostEqual(telemetry["first_crossing"]["magnitude_g"], 1.05)
        self.assertEqual(telemetry["onset_window"]["first_sample_time_s"], .010)
        self.assertEqual(telemetry["onset_window"]["last_sample_time_s"], .025)
        self.assertEqual(telemetry["onset_window"]["magnitude_peak_g"], 4.)
        self.assertEqual(telemetry["post_stop_response"]["magnitude_peak_g"], 14.)
        self.assertEqual(telemetry["post_stop_response"]["boundary_source"],
                         "host_observed_queue_completion")

    def test_no_sample_after_observed_boundary_reports_unavailable(self):
        obj = make_homing()
        record, capture = beta_event()
        capture["beta_capture"]["stop_response"].update(boundary_time_s=.1, complete=False)
        telemetry = obj.beta_home_telemetry(capture, record)
        response = telemetry["post_stop_response"]
        self.assertFalse(response["available"])
        self.assertFalse(response["complete"])
        self.assertIsNone(response["magnitude_peak_g"])
        self.assertEqual(response["sample_count"], 0)

    def test_changed_onset_identity_and_earlier_crossing_are_rejected(self):
        for failure in ("identity", "earlier", "dropped", "fault"):
            with self.subTest(failure=failure):
                record, capture = beta_event()
                if failure == "identity": capture["detector"]["onset_sample_matches"] = 2
                elif failure == "earlier": capture["samples"][8] = (.008, 1.2, 0., 1.)
                elif failure == "dropped": capture["dropped"] = 1
                else: capture["fault"] = "invalid_data"
                with self.assertRaises(CommandError):
                    make_homing().beta_home_telemetry(capture, record)

    def test_ordinary_whole_packet_peak_behavior_is_unchanged(self):
        obj = make_homing()
        attempt = types.SimpleNamespace(impact_until=10, vector_mode=True, bases=(0., 0., 0.),
                                        last_samples=[0, 0, 0], impact_peak=0., count_g=1.,
                                        record={"impact_peak_g": 0.})
        obj._track_impact(attempt, {"start_clock": 0, "delta_clock": 100},
                          (.5, 0., 0., 2., 0., 0., 7., 0., 0.), 3)
        self.assertEqual(attempt.record["impact_peak_g"], 7.)
        self.assertIsNone(obj.beta_capture)


class BetaRawCaptureTests(unittest.TestCase):
    def owner(self):
        obj = make_homing()
        obj.reactor.now = .010
        obj.start_capture(beta=True)
        return obj

    def test_native_clipped_marker_is_faulted_before_decode_can_hide_it(self):
        obj = self.owner()
        obj._capture_batch(batch(8, [(plugin.CLIPPED, 0, counts(1.)), (0, 0, counts(1.))], delta_clock=1))
        captured = obj.stop_capture()
        self.assertEqual(captured["fault"], "invalid_data")
        self.assertGreater(captured["beta_capture"]["quality"]["clipped_axis_values"], 0)
        self.assertNotEqual(captured["samples"][0][1], plugin.CLIPPED * obj.scale["scale"] / 1000.)
        self.assertIsNone(obj.beta_capture)

    def test_stale_gap_context_and_overflow_produce_bounded_diagnostics(self):
        for failure in ("stale", "gap", "context", "overflow"):
            with self.subTest(failure=failure):
                obj = self.owner()
                if failure == "stale": obj.reactor.now = 1.
                elif failure == "context": obj.helper._scale = dict(obj.helper._scale, id=99)
                elif failure == "overflow": obj.capture_limit = 1
                if failure == "gap":
                    obj._capture_batch(batch(8, [(0, 0, counts(1.)), (1, 0, counts(1.))], delta_clock=1))
                    obj.reactor.now = .040
                obj._capture_batch(batch(38 if failure == "gap" else 8,
                                         [(0, 0, counts(1.)), (1, 0, counts(1.))], delta_clock=1))
                captured = obj.stop_capture()
                self.assertTrue(captured["fault"] or captured["dropped"])
                diagnostic = obj.beta_capture_diagnostics()
                self.assertNotIn("samples", diagnostic)
                self.assertIsNotNone(diagnostic["fault"])
                json.dumps(diagnostic, allow_nan=False)

    def test_disabled_capture_retains_ordinary_shape(self):
        obj = make_homing()
        obj.start_capture()
        obj._capture_batch(batch(8, [(0, 0, counts(1.)), (1, 0, counts(1.))], delta_clock=1))
        captured = obj.stop_capture()
        self.assertNotIn("beta_capture", captured)
        self.assertEqual(captured["dropped"], 0)
        self.assertIsNone(captured["fault"])

    def test_replaced_beta_owner_cannot_accept_unvalidated_old_packet(self):
        obj = make_homing()
        obj.start_capture()
        original_lock = obj.lock

        class ReplacementLock:
            def __enter__(self): obj.beta_capture = {"replacement": True}
            def __exit__(self, *_args): pass

        obj.lock = ReplacementLock()
        obj._capture_batch(batch(8, [(0, 0, counts(1.)), (1, 0, counts(1.))], delta_clock=1))
        obj.lock, obj.beta_capture = original_lock, None
        self.assertEqual(obj.capture_samples, 0)
        self.assertEqual(obj.capture_batches, [])
        obj.stop_capture()

    def test_tail_is_bounded_and_cancellation_closes_exact_stream(self):
        obj = self.owner()
        obj.stream_on = True
        obj.beta_capture["tail_retained"] = True
        obj.beta_capture["last_sample_time_s"] = .009
        start = obj.reactor.now
        obj.finish_beta_home_capture(lambda: None)
        self.assertLessEqual(obj.reactor.now - start, plugin.STREAM_START_TIMEOUT + .0051)
        self.assertFalse(obj.beta_capture["stop_response"]["available"])
        obj.stop_capture()
        self.assertFalse(obj.stream_on)
        obj = self.owner()
        obj.stream_on = True
        obj.beta_capture["tail_retained"] = True

        def cancelled(): raise CommandError("original cancellation")
        with mock.patch.object(obj, "_stop_stream", side_effect=RuntimeError("cleanup")):
            with self.assertRaisesRegex(CommandError, "original cancellation"):
                obj.finish_beta_home_capture(cancelled)
        obj.stop_capture()

    def test_moving_capture_waits_for_end_keeps_offset_and_closes_subscription(self):
        obj = make_homing()
        obj.homing_active = False

        def baseline(_check, _duration):
            obj.reactor.now = 2.
            return {"samples": [(1. + index / 1000., .2 + (.001 if index % 2 else -.001), -.1, 1.)
                                for index in range(1001)]}

        with mock.patch.object(obj, "collect_noise_window", side_effect=baseline):
            token = obj.start_beta_motion_capture(lambda: None)
        expected_baseline = token["baseline_g"]
        obj.reactor.now = 2.005
        obj._inspect(batch(2001, [(counts(.2), 0, counts(1.)), (counts(.4), 0, counts(1.)),
                                (counts(.2), 0, counts(1.)), (counts(.4), 0, counts(1.)),
                                (counts(.2), 0, counts(1.))], delta_clock=4))
        token["motion_end_time_s"] = 2.010

        def tail():
            obj._inspect(batch(2006, [(counts(.2), 0, counts(1.)), (counts(.4), 0, counts(1.)),
                                    (counts(.2), 0, counts(1.)), (counts(.4), 0, counts(1.)),
                                    (counts(.2), 0, counts(1.))], delta_clock=4))

        obj.reactor.on_pause = tail
        captured = obj.stop_beta_motion_capture(token, lambda: None)
        self.assertGreaterEqual(captured["last_sample_time_s"], 2.010)
        self.assertEqual(captured["baseline_g"], expected_baseline)
        self.assertEqual(len(captured["samples"]), 10)
        self.assertIsNone(obj.noise_session)
        self.assertIsNone(obj.beta_motion_capture)
        self.assertIsNone(obj.capture_batches)
        self.assertEqual(obj.toolhead.moves, [])


class BetaPNGTests(unittest.TestCase):
    def test_seed_excludes_braking_and_preserves_non_axis_vector_peak_in_png(self):
        iterations = [beta_motion(offset) for offset in (0., 2., 4.)]
        statistics = charts.beta_motion_statistics(iterations[0]["capture"], iterations[0]["record"])
        self.assertAlmostEqual(statistics["seed_magnitude_peak_g"], math.sqrt(.9 ** 2 * 2))
        self.assertEqual(statistics["all_phase_magnitude_peak_g"], 5.)
        self.assertFalse(statistics["phases"][-1]["seed_eligible"])
        record, capture = beta_event()
        record["beta_telemetry"] = make_homing().beta_home_telemetry(capture, record)
        compact = charts.compact_beta_home(capture, record)
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as temp:
            paths = charts.write_beta_motion(temp, "beta_event", "x", iterations, homes=[compact],
                                            result="beta_proposed", proposed_threshold_g=3.25,
                                            simulated=True, renderer=renderer)
            self.assertEqual(set(paths), {"png"})
            self.assertIn("Beta", Path(paths["png"]).name)
            self.assertIn("Suggested Threshold", Path(paths["png"]).name)
            self.assertEqual([path.suffix for path in Path(temp).iterdir()], [".png"])
            payload = renderer.payloads[0]
            self.assertEqual(payload["kind"], "beta")
            self.assertEqual(payload["result"], "beta_proposed")
            self.assertIn("Beta", payload["title"])
            self.assertEqual(payload["motions"][0]["record"]["configured_speed_mm_s"], 100.)
            self.assertIn((123 / 5000., .9, .9, 0.), payload["motions"][0]["trace"])
            self.assertEqual(payload["homes"][0]["record"]["beta_telemetry"],
                             json.loads(json.dumps(record["beta_telemetry"])))
            self.assertRegex(payload["timestamp"], r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}")

    def test_faulted_motion_never_exports_or_seeds(self):
        item = beta_motion()
        item["capture"]["fault"] = "invalid_data"
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                charts.write_beta_motion(temp, "beta_event", "x", [item], renderer=renderer)
            self.assertEqual(renderer.payloads, [])
            self.assertEqual(list(Path(temp).iterdir()), [])

    def test_failure_png_preserves_valid_homes_and_marks_incomplete_trial(self):
        record, capture = beta_event()
        record["beta_telemetry"] = make_homing().beta_home_telemetry(capture, record)
        valid = charts.compact_beta_home(capture, record)
        incomplete = {"record": {"run_id": "beta_event", "axis": "x", "workflow_attempt": 2,
                                  "result": "invalid_data", "beta_decision": "incomplete",
                                  "beta_reason": "simulated capture fault", "capture_valid": False},
                      "capture": {"samples": []}}
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as temp:
            paths = charts.write_beta_motion(temp, "beta_event", "x", [], homes=[valid, incomplete],
                                            result="aborted", reason="simulated capture fault",
                                            simulated=True, renderer=renderer)
            self.assertTrue(Path(paths["png"]).is_file())
            payload = renderer.payloads[0]
            self.assertEqual(len(payload["homes"]), 1)
            self.assertEqual(payload["incomplete_homes"][0]["workflow_attempt"], 2)
            self.assertNotIn("samples", payload["incomplete_homes"][0])


if __name__ == "__main__":
    unittest.main()
