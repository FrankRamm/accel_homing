"""Censored native response chart provenance. ALL DATA ARE SYNTHETIC."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import accel_homing_charts as charts
import accel_homing_filters as filters
from test_accuracy_charts import RecordingRenderer, png_bytes
from test_chronological_clipping import historical_censored_capture as native_filtered_capture


def private_native_keys(value):
    if isinstance(value, dict):
        return [key for key in value if key.startswith("_native_")] + [
            key for item in value.values() for key in private_native_keys(item)]
    if isinstance(value, (tuple, list)):
        return [key for item in value for key in private_native_keys(item)]
    return []


def synthetic_native_accuracy_axis(run_id, travels=(175., 175.2, 174.9, 175.1), censored_indices=(1, 3), filter_version=None):
    """Real software callbacks, generated native samples and commanded travels."""
    item = {"requested_samples": len(travels), "complete": True, "discovery": None,
            "trials": [], "statistics": None, "capture_paths": [],
            "discovery_capture_paths": None, "rejected": None}
    for index, travel in enumerate((174.8,) + tuple(travels)):
        capture, record = native_filtered_capture(clip_position=4 if index in censored_indices else None,
            filter_config=None if filter_version is None else filters.detector_config({"version": filter_version}))
        record.update(run_id=run_id, workflow_attempt=index + 1,
                      workflow_phase="measurement" if index else "discovery", accuracy_sample=index,
                      travel_mm=travel, travel_mm_exact=travel,
                      sensor_context={"sensor": "beacon synthetic", "backend": "beacon_raw",
                                      "axes_map": "x,y,z", "full_scale_g": 15.9921875},
                      geometry_context={"position_min": 0., "position_max": 350.,
                                        "position_endstop": 350., "homing_positive_dir": True})
        prepared = charts.prepare_capture("%s_a%03d" % (run_id, index + 1), "x", capture, record, simulated=True)
        if index:
            item["trials"].append(record)
            item["capture_paths"].append(prepared)
        else:
            item["discovery"], item["discovery_capture_paths"] = record, prepared
    return item


def synthetic_public_censored_report(count=2, filter_version=1):
    """Public receipt parser fixture, not retained native capture evidence."""
    travels = [175.0 + index * .001 for index in range(count)]
    response = {"trial_index": 1, "cause": "native_clip_sentinel",
                "first_invalid_sample": [100000, 700, 4, 8],
                "first_invalid_sample_index": 104, "first_invalid_time_s": 1.04,
                "original_sample_count": 108, "valid_prefix_sample_count": 104,
                "censored_sample_count": 4, "confirmation_sample_index": 102,
                "confirmation_sample": [100000, 700, 2, 8],
                "confirmation_time_s": 1.02, "observed_valid_prefix_peak_g": 2.5,
                "impact_peak_g": None, "raw_samples_validated": False,
                "decision_prefix_validated": True}
    context = {key: None for key in charts._ACCURACY_CONTEXT_KEYS}
    context.update(detector_type="filtered", impact_metric="xyz_magnitude",
                   detector_version=filter_version, detector_config=filters.detector_config({"version": filter_version}),
                   sensor_context={"sensor": "beacon synthetic", "backend": "beacon_raw"})
    return {"version": 3 if filter_version == 2 else 2, "kind": "completed_accuracy", "run_id": "synthetic_censored_receipt",
            "axis": "x", "result": "complete", "measurement": "commanded_homing_travel",
            "physical_position_verified": False, "simulated": True,
            "requested": count, "count": count, "threshold_g": 1., "context": context,
            "travels_mm": travels,
            "statistics": charts._accuracy_statistics([{"travel_mm": value} for value in travels]),
            "censored_response_count": 1, "censored_responses": [response]}


class CensoredAccuracyReceiptTests(unittest.TestCase):
    def test_v2_roundtrip_retains_null_peak_and_separate_prefix_peak(self):
        report = synthetic_public_censored_report()
        restored = charts._accuracy_report_from_png(charts._png_with_accuracy_report(png_bytes(), report))
        self.assertEqual(restored, report)
        self.assertIsNone(restored["censored_responses"][0]["impact_peak_g"])
        self.assertEqual(restored["censored_responses"][0]["observed_valid_prefix_peak_g"], 2.5)
        self.assertEqual(restored["statistics"]["count"], 2)

    def test_legacy_v1_absence_remains_unknown_not_inferred_zero(self):
        report = synthetic_public_censored_report()
        report.update(version=1)
        report["context"]["detector_type"] = "raw"
        del report["censored_response_count"], report["censored_responses"]
        restored = charts._accuracy_report_from_png(charts._png_with_accuracy_report(png_bytes(), report))
        self.assertEqual(restored, report)
        self.assertNotIn("censored_response_count", restored)
        self.assertNotIn("censored_responses", restored)

    def test_new_uncensored_v1_reports_zero_explicitly(self):
        report = synthetic_public_censored_report()
        report.update(version=1, censored_response_count=0, censored_responses=[])
        self.assertEqual(charts._validate_accuracy_report(report), report)

    def test_v1_censor_claim_and_v2_zero_or_missing_summary_are_rejected(self):
        for update, deleted in (({"version": 1}, ()),
                                ({"censored_response_count": 0, "censored_responses": []}, ()),
                                ({}, ("censored_response_count",)),
                                ({}, ("censored_responses",)),
                                ({}, ("censored_response_count", "censored_responses"))):
            report = synthetic_public_censored_report()
            report.update(update)
            for key in deleted:
                del report[key]
            with self.subTest(update=update, deleted=deleted), self.assertRaises(ValueError):
                charts._png_with_accuracy_report(png_bytes(), report)

    def test_count_duplicate_unordered_and_raw_censor_claims_are_rejected(self):
        original = synthetic_public_censored_report()
        changes = []
        for count in (True, -1, 2, 3):
            changes.append({**copy.deepcopy(original), "censored_response_count": count})
        for indices in ((1, 1), (2, 1)):
            report = copy.deepcopy(original)
            report["censored_response_count"] = 2
            report["censored_responses"] = [{**report["censored_responses"][0], "trial_index": index}
                                             for index in indices]
            changes.append(report)
        report = copy.deepcopy(original)
        report["context"]["detector_type"] = "raw"
        changes.append(report)
        for report in changes:
            with self.subTest(report=report), self.assertRaises(ValueError):
                charts._validate_accuracy_report(report)

    def test_impossible_native_packet_identities_are_rejected(self):
        for stamp in ([0x100000000, 700, 4, 8], [100000, 0, 4, 8],
                      [100000, 0x100000000, 4, 8], [100000, 700, 0, 1],
                      [100000, 700, 8, 8], [True, 700, 4, 8],
                      [100000, 700, 4, charts._MAX_CENSORED_NATIVE_SAMPLES + 1]):
            for key in ("first_invalid_sample", "confirmation_sample"):
                report = synthetic_public_censored_report()
                report["censored_responses"][0][key] = stamp
                with self.subTest(key=key, stamp=stamp), self.assertRaises(ValueError):
                    charts._validate_accuracy_report(report)

    def test_missing_or_mismatched_frozen_native_context_is_rejected(self):
        for key, values in {"detector_version": (None, True, 2),
                            "detector_config": (None, {}, {"version": 1}),
                            "impact_metric": (None, "selected_axis"),
                            "sensor_context": (None, {"backend": "generic"})}.items():
            for value in values:
                report = synthetic_public_censored_report()
                report["context"][key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    charts._validate_accuracy_report(report)

    def test_censor_limits_and_unknown_peak_cannot_be_forged(self):
        changes = {"trial_index": (0, 3, True), "cause": ("sample_gap", "nonfinite"),
                   "first_invalid_sample_index": (103, 105),
                   "original_sample_count": (107, charts._MAX_CENSORED_NATIVE_SAMPLES + 1),
                   "valid_prefix_sample_count": (1, 108), "censored_sample_count": (0, 5),
                   "confirmation_sample_index": (104, -1),
                   "confirmation_sample": ([100000, 700, 4, 8], [100000, 700, 5, 8]),
                   "confirmation_time_s": (1.04, 1.05, float("nan")),
                   "first_invalid_time_s": (1.02, float("inf")),
                   "observed_valid_prefix_peak_g": (-1, float("nan"), float("inf")),
                   "impact_peak_g": (2.5, float("nan")),
                   "raw_samples_validated": (True, None), "decision_prefix_validated": (False, None)}
        for key, values in changes.items():
            for value in values:
                report = synthetic_public_censored_report()
                report["censored_responses"][0][key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    charts._validate_accuracy_report(report)

    def test_maximum_trial_count_remains_inside_existing_receipt_bound(self):
        report = synthetic_public_censored_report(50)
        response = report["censored_responses"][0]
        response.update(first_invalid_sample=[0xffffffff, 0xffffffff, 199999, 200000],
                        confirmation_sample=[0xffffffff, 0xffffffff, 199998, 200000],
                        first_invalid_sample_index=199999, valid_prefix_sample_count=199999,
                        original_sample_count=200000, censored_sample_count=1,
                        confirmation_sample_index=199998, confirmation_time_s=123456.12345678901,
                        first_invalid_time_s=123456.12345678902, observed_valid_prefix_peak_g=15.1234567890123)
        report["censored_response_count"] = 50
        report["censored_responses"] = [{**response, "trial_index": index} for index in range(1, 51)]
        data = charts._png_with_accuracy_report(png_bytes(), report)
        self.assertEqual(charts._accuracy_report_from_png(data)["censored_response_count"], 50)
        self.assertLess(len(charts._json(report).encode("ascii")), charts._MAX_ACCURACY_REPORT_BYTES)


class CensoredNativeChartTests(unittest.TestCase):
    def test_finite_range_signs_keep_full_recorded_response_without_censoring(self):
        for raw_value in (-2048, -2047, 2047):
            capture, record = native_filtered_capture(clip_value=raw_value)
            prepared = charts.prepare_capture("synthetic_full_scale", "x", capture, record, simulated=True)
            _receipt, frozen, metadata = charts._prepared_capture(prepared)
            with self.subTest(raw_value=raw_value):
                self.assertNotIn("response_censoring", metadata)
                self.assertFalse(frozen["response_censored"])
                self.assertGreater(frozen["impact_peak_g"], 15.)
                self.assertTrue(metadata["amplitude_limited"])
                self.assertTrue(metadata["excess_amplitude_unknown"])
                self.assertEqual(max(abs(row[1]) for row in _receipt.trace), abs(raw_value) * .0078125)

    def test_only_validated_native_prefix_reaches_capture_renderer(self):
        capture, record = native_filtered_capture()
        summary = record["response_censoring"]
        prepared = charts.prepare_capture("synthetic_native_prefix", "x", capture, record, simulated=True)
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as temporary:
            charts.publish_capture(temporary, prepared, renderer)
            self.assertEqual([path.suffix for path in Path(temporary).iterdir()], [".png"])
        payload = renderer.payloads[0]
        self.assertEqual(payload["capture"]["sample_count"], summary["valid_prefix_sample_count"])
        self.assertLess(payload["capture"]["sample_count"], summary["original_sample_count"])
        self.assertLess(payload["capture"]["last_sample_time_s"], summary["first_invalid_time_s"])
        self.assertFalse(payload["capture"]["raw_integrity"]["raw_samples_validated"])
        self.assertIsNone(payload["record"]["impact_peak_g"])
        self.assertEqual(payload["record"]["observed_valid_prefix_peak_g"], 3.)
        self.assertEqual(private_native_keys(payload), [])
        self.assertIn("full impact peak unavailable", " ".join(charts._response_censoring_lines(
            payload["capture"]["response_censoring"], payload["capture"]["first_sample_time_s"])))

    def test_replay_observation_ends_at_clipping_before_host_stop_request(self):
        capture, record = native_filtered_capture()
        prepared = charts.prepare_filter_capture("synthetic_native_cutoff", "x", capture, record,
                                                  10., 1., simulated=True)
        payload = json.loads(prepared.receipt.payload_json)
        analysis = payload["analysis"]
        self.assertEqual(analysis["decision_cutoff_reason"], "first_invalid_sample")
        self.assertEqual(analysis["decision_cutoff_time_s"], record["response_censoring"]["first_invalid_time_s"])
        self.assertLess(analysis["decision_cutoff_time_s"], analysis["stop_request_interval_s"][0])
        self.assertTrue(analysis["raw_decision"]["censored"])
        self.assertFalse(analysis["raw_decision"]["decision_observed"])
        self.assertTrue(analysis["filtered_decision"]["decision_observed"])
        self.assertAlmostEqual(analysis["filtered_decision"]["confirmation_time_s"],
                               analysis["actual_confirmation_time_s"], places=12)
        self.assertTrue(all(row[0] < analysis["decision_cutoff_time_s"] for row in analysis["trace"]))
        self.assertEqual(analysis["sample_count"], len(capture["samples"]))
        self.assertFalse(analysis["raw_integrity"]["raw_samples_validated"])
        self.assertFalse(analysis["physical_accuracy_verified"])
        self.assertEqual(private_native_keys(payload), [])

    def test_remapped_capture_keeps_chart_boundaries_in_the_same_time_domain(self):
        for shift in (-.001, .001):
            capture, record = native_filtered_capture(clock_shift=shift)
            prepared = charts.prepare_filter_capture("synthetic_remapped_cutoff", "x", capture, record,
                                                      10., 1., simulated=True)
            analysis = json.loads(prepared.receipt.payload_json)["analysis"]
            with self.subTest(clock_shift=shift):
                self.assertAlmostEqual(analysis["arm_time_s"], .106 + shift, places=12)
                self.assertAlmostEqual(analysis["actual_confirmation_time_s"], .107 + shift, places=12)
                self.assertAlmostEqual(analysis["filtered_decision"]["confirmation_time_s"], .107 + shift, places=12)
                self.assertAlmostEqual(analysis["decision_cutoff_time_s"], .11 + shift, places=12)
                self.assertAlmostEqual(analysis["response_censoring"]["first_invalid_time_s"], .11 + shift, places=12)
                self.assertAlmostEqual(analysis["stop_request_interval_s"][0], .114 + shift, places=12)
                self.assertLess(analysis["trace"][-1][0], analysis["decision_cutoff_time_s"])
                self.assertEqual(analysis["decision_cutoff_reason"], "first_invalid_sample")

    def test_forged_prefix_and_record_proof_never_reach_renderer_or_disk(self):
        changes = ("source_missing", "prefix_shortened", "nonfinite_prefix", "wrong_record_cutoff",
                   "false_whole_integrity", "invented_full_peak")
        for change in changes:
            capture, record = native_filtered_capture()
            if change == "source_missing":
                capture.pop("_native_source_batches")
            elif change == "prefix_shortened":
                capture["samples"] = capture["samples"][:-1]
            elif change == "nonfinite_prefix":
                row = capture["samples"][5]
                capture["samples"][5] = (row[0], float("nan"), row[2], row[3])
            elif change == "wrong_record_cutoff":
                record["response_censoring"]["first_invalid_time_s"] += .001
            elif change == "false_whole_integrity":
                capture["raw_integrity"]["raw_samples_validated"] = True
            else:
                record["impact_peak_g"] = record["observed_valid_prefix_peak_g"]
            renderer = RecordingRenderer()
            with tempfile.TemporaryDirectory() as temporary:
                with self.subTest(change=change), self.assertRaises(ValueError):
                    charts.write_filter_capture(temporary, "synthetic_bad_prefix", "x", capture, record,
                                                1., 1., simulated=True, renderer=renderer)
                self.assertEqual(renderer.payloads, [])
                self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_frozen_prefix_metadata_must_still_match_its_record(self):
        capture, record = native_filtered_capture()
        prepared = charts.prepare_capture("synthetic_frozen_prefix", "x", capture, record, simulated=True)
        metadata = json.loads(prepared.receipt.metadata_json)
        metadata["response_censoring"]["first_invalid_time_s"] += .001
        forged = charts.CapturePaths({}, prepared.receipt._replace(metadata_json=charts._json(metadata)))
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(ValueError, "validated prefix receipt"):
                charts.publish_capture(temporary, forged, RecordingRenderer())
            self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_complete_accuracy_keeps_travel_statistics_separate_from_missing_peaks(self):
        run_id, renderer = "synthetic_native_accuracy", RecordingRenderer()
        item = synthetic_native_accuracy_axis(run_id)
        with tempfile.TemporaryDirectory() as temporary:
            paths = charts.write_accuracy(temporary, run_id, {"x": item}, "complete",
                                           simulated=True, renderer=renderer)
            report = charts.read_accuracy_baseline(temporary, run_id, "x", simulated=True)
            self.assertEqual(list(Path(temporary).iterdir()), [Path(paths["x_png"])])
        self.assertEqual(report["version"], 3)
        self.assertEqual(report["censored_response_count"], 2)
        self.assertEqual([entry["trial_index"] for entry in report["censored_responses"]], [1, 3])
        self.assertTrue(all(entry["impact_peak_g"] is None for entry in report["censored_responses"]))
        self.assertEqual(report["statistics"], charts._accuracy_statistics(item["trials"]))
        self.assertEqual(report["statistics"]["count"], 4)
        self.assertTrue(all(len(trace) <= charts._MAX_CAPTURE_TRACE_POINTS for trace in renderer.payloads[0]["traces"]))
        self.assertEqual(private_native_keys(renderer.payloads), [])

    def test_null_peak_without_native_proof_and_mismatched_trial_receipt_are_rejected(self):
        for change in ("unproven_null", "record_mismatch"):
            run_id = "synthetic_invalid_accuracy"
            item = synthetic_native_accuracy_axis(run_id)
            if change == "unproven_null":
                item["trials"][1]["impact_peak_g"] = None
            else:
                item["trials"][0]["response_censoring"]["first_invalid_time_s"] += .001
            with tempfile.TemporaryDirectory() as temporary:
                with self.subTest(change=change), self.assertRaises(ValueError):
                    charts.write_accuracy(temporary, run_id, {"x": item}, "complete",
                                          simulated=True, renderer=RecordingRenderer())
                self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_failed_and_incomplete_workflows_still_publish_no_chart(self):
        item = synthetic_native_accuracy_axis("synthetic_failed_accuracy")
        with tempfile.TemporaryDirectory() as temporary:
            for result in ("failed", "incomplete", "aborted", "cancelled"):
                with self.subTest(result=result), self.assertRaisesRegex(ValueError, "Only a complete"):
                    charts.write_accuracy(temporary, "synthetic_failed_accuracy", {"x": item}, result,
                                          simulated=True, renderer=RecordingRenderer())
            self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_raw_ordinary_capture_behavior_is_preserved(self):
        from test_filter_charts import synthetic_filter_home
        capture, record = synthetic_filter_home()
        capture["raw_integrity"] = {"raw_samples_validated": False, "clipped_axis_values": 1}
        prepared = charts.prepare_capture("synthetic_legacy_raw", "x", capture, record, simulated=True)
        self.assertIsInstance(prepared.receipt, charts._MemoryReceipt)
        with self.assertRaisesRegex(ValueError, "clipped"):
            charts.prepare_filter_capture("synthetic_legacy_raw", "x", capture, record, .8, .8, simulated=True)


if __name__ == "__main__":
    unittest.main()
