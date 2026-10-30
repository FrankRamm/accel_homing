"""Signed-v1 / rectified-v2 comparison checks. ALL DATA ARE SYNTHETIC."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import accel_homing_charts as charts
import accel_homing_filters as filters
from test_accuracy_charts import RecordingRenderer, png_bytes
from test_censored_response_charts import (native_filtered_capture, private_native_keys,
    synthetic_native_accuracy_axis, synthetic_public_censored_report)
from test_filter_charts import synthetic_filter_home
from test_rectified_filter import native_gap_remap_capture


def synthetic_alternating_filter_home():
    """Generated bipolar pulse; differing replay results are not accuracy data."""
    capture, record = synthetic_filter_home()
    capture["samples"] = [(row[0], row[1] - 2. + (2. if index % 2 == 0 else -2.), row[2], row[3])
                          if 1200 <= index < 1212 else row for index, row in enumerate(capture["samples"])]
    capture["detector"].update(stop_request_interval_s=[10.121, 10.1214], stop_request_time_s=10.1214)
    return capture, record


class RectifiedFilterComparisonTests(unittest.TestCase):
    def prepare(self, detector="raw", version=2, count=1450):
        capture, record = synthetic_filter_home(detector=detector, filter_version=version, count=count)
        prepared = charts.prepare_filter_capture("synthetic_signed_rectified", "x", capture, record,
                                                  .8, .8, simulated=True)
        return capture, record, prepared, json.loads(prepared.receipt.payload_json)

    def test_both_algorithms_share_original_signed_input_and_exact_causal_values(self):
        capture, _record, _prepared, payload = self.prepare("filtered")
        analysis = payload["analysis"]
        self.assertEqual(set(analysis["variants"]), {"signed_v1", "rectified_v2"})
        self.assertTrue(any(row[1] < 0 for row in analysis["trace"]))
        self.assertEqual(analysis["actual_variant"], "rectified_v2")
        by_time = {row[0]: row for row in analysis["trace"]}
        for key in charts._FILTER_VARIANTS:
            item = analysis["variants"][key]
            reference = item["reference"]
            engine = filters.CausalImpactDetector(.8, reference["offset_g"], reference["noise_sigma_g"],
                                                   reference["weights_xyz"], item["config"])
            engine.previous_time = 10.1 - .0001
            columns = analysis["trace_columns"][key]
            for source in (row for row in capture["samples"] if row[0] >= 10.1):
                engine.feed(source[0], source[1:])
                displayed = by_time[source[0]]
                for index, value in zip(columns["xyz"], engine.filtered_xyz):
                    self.assertAlmostEqual(displayed[index], value, places=11)
                self.assertAlmostEqual(displayed[columns["weighted_level"]], engine.level_g, places=11)
        rectified = analysis["trace_columns"]["rectified_v2"]["xyz"]
        self.assertTrue(all(row[index] >= 0 for row in analysis["trace"] for index in rectified))

    def test_active_version_is_explicit_and_raw_has_two_diagnostic_variants(self):
        for detector, version, active in (("raw", 2, None), ("filtered", 1, "signed_v1"),
                                          ("filtered", 2, "rectified_v2")):
            _capture, _record, _prepared, payload = self.prepare(detector, version)
            analysis = payload["analysis"]
            with self.subTest(detector=detector, version=version):
                self.assertEqual(analysis["actual_variant"], active)
                for key, item in analysis["variants"].items():
                    self.assertEqual(item["threshold_source"], "actual_active_detector" if key == active else
                                     "same_numeric_threshold_diagnostic_not_calibrated")
                    self.assertEqual(item["threshold_g"], .8)
                self.assertNotIn("signal_mode", analysis["variants"]["signed_v1"]["config"])
                self.assertEqual(analysis["variants"]["rectified_v2"]["config"]["signal_mode"], "rectified")

    def test_mean_removed_noise_does_not_hide_rectification_dc(self):
        capture, record = synthetic_filter_home()
        capture["samples"] = [(row[0], .2 + (.08 if index % 2 else -.08), -.1, 1.)
                              if index < 1000 else row for index, row in enumerate(capture["samples"])]
        prepared = charts.prepare_filter_capture("synthetic_rectification_dc", "x", capture, record,
                                                  .8, .8, simulated=True)
        noise = json.loads(prepared.receipt.payload_json)["analysis"]["noise"]
        signed, rectified = (noise["variants"][key] for key in charts._FILTER_VARIANTS)
        self.assertLess(abs(signed["mean_xyz_g"][0]), .002)
        self.assertGreater(rectified["mean_xyz_g"][0], .07)
        self.assertLess(rectified["rms_xyz_g"][0], .001)
        self.assertGreater(rectified["decision_level_g"]["mean"], .05)
        self.assertGreater(rectified["decision_level_g"]["rms"], .05)
        self.assertIn("without_mean_removal", noise["decision_level_policy"])

    def test_stop_censors_both_unconfirmed_variants_without_invented_delay(self):
        _capture, _record, _prepared, payload = self.prepare()
        analysis = payload["analysis"]
        for item in analysis["variants"].values():
            self.assertTrue(item["decision"]["censored"])
            self.assertFalse(item["decision"]["decision_observed"])
            self.assertIsNone(item["decision"].get("confirmation_time_s"))
            self.assertEqual(item["decision_cutoff_time_s"], analysis["decision_cutoff_time_s"])

    def test_bipolar_pulse_can_change_replay_outcome_without_physical_improvement_claim(self):
        capture, record = synthetic_alternating_filter_home()
        prepared = charts.prepare_filter_capture("synthetic_bipolar_pulse", "x", capture, record,
                                                  .8, .8, simulated=True)
        analysis = json.loads(prepared.receipt.payload_json)["analysis"]
        self.assertIsNone(analysis["actual_variant"])
        self.assertTrue(analysis["variants"]["signed_v1"]["decision"]["censored"])
        self.assertTrue(analysis["variants"]["rectified_v2"]["decision"]["decision_observed"])
        self.assertFalse(analysis["physical_accuracy_verified"])

    def test_original_times_not_display_estimates_prove_observed_confirmation(self):
        capture, _record = synthetic_alternating_filter_home()
        analysis = filters.compare_filter_capture(capture, .8, .8)
        decision = analysis["variants"]["rectified_v2"]["decision"]
        self.assertTrue(decision["decision_observed"])
        self.assertLess(decision["original_confirmation_time_s"], analysis["decision_cutoff_original_time_s"])
        # A historical nearest-packet display estimate could reverse marker
        # order across a remapped interpacket gap. It is not eligibility proof.
        estimate = decision["confirmation_time_s"] - .0001
        analysis["decision_cutoff_time_s"] = estimate
        for item in analysis["variants"].values():
            item["decision_cutoff_time_s"] = estimate
        variants, _primary = charts._filter_variants(analysis)
        self.assertTrue(variants["rectified_v2"]["decision"]["decision_observed"])
        variants["rectified_v2"]["decision"]["original_confirmation_time_s"] = analysis["decision_cutoff_original_time_s"]
        with self.assertRaisesRegex(ValueError, "proven comparison cutoff"):
            charts._filter_variants(analysis)

    def test_native_gap_remap_preserves_original_eligibility_in_both_variants(self):
        for version in (1, 2):
            capture, record = native_gap_remap_capture(version)
            prepared = charts.prepare_filter_capture("synthetic_native_gap", "x", capture, record,
                                                      1., record["threshold_g"], simulated=True)
            analysis = json.loads(prepared.receipt.payload_json)["analysis"]
            with self.subTest(version=version):
                self.assertTrue(analysis["decision_cutoff_display_is_approximate"])
                for item in analysis["variants"].values():
                    self.assertTrue(item["decision"]["decision_observed"])
                    self.assertLess(item["decision"]["original_confirmation_time_s"],
                                    analysis["decision_cutoff_original_time_s"])
                    self.assertLess(item["decision"]["confirmation_time_s"], analysis["decision_cutoff_time_s"])
                self.assertEqual(private_native_keys(analysis), [])

    def test_native_censored_prefix_preserves_signed_axes_and_original_identities(self):
        capture, record = native_filtered_capture(axes_map=((0, -1), (1, 1), (2, 1)))
        prepared = charts.prepare_filter_capture("synthetic_mapped_rectification", "x", capture, record,
                                                  10., 1., simulated=True)
        analysis = json.loads(prepared.receipt.payload_json)["analysis"]
        self.assertEqual(private_native_keys(analysis), [])
        self.assertEqual(analysis["response_censoring"], record["response_censoring"])
        self.assertTrue(any(row[1] < -2. for row in analysis["trace"]))
        columns = analysis["trace_columns"]["rectified_v2"]["xyz"]
        self.assertTrue(all(row[index] >= 0 for row in analysis["trace"] for index in columns))
        self.assertTrue(all(row[0] < analysis["decision_cutoff_time_s"] for row in analysis["trace"]))
        for item in analysis["variants"].values():
            self.assertEqual(item["decision_cutoff_reason"], "first_invalid_sample")
        self.assertIsNone(record["impact_peak_g"])

    def test_mismatched_source_identity_config_cutoff_or_observation_never_publishes(self):
        capture, record = synthetic_filter_home()
        original = filters.compare_filter_capture(capture, .8, .8)
        for change in ("clock", "reference", "mode", "threshold", "control_label", "cutoff", "fake_confirmation"):
            analysis = copy.deepcopy(original)
            item = analysis["variants"]["signed_v1"]
            if change == "clock":
                item["trace"][10][0] += .00001
            elif change == "reference":
                item["reference"] = {**item["reference"], "weights_xyz": [.5, 1., 1.]}
            elif change == "mode":
                item["signal_mode"] = "rectified"
            elif change == "threshold":
                item["threshold_g"] = .9
            elif change == "control_label":
                item["threshold_source"] = "actual_active_detector"
            elif change == "cutoff":
                item["decision_cutoff_original_time_s"] += .001
            else:
                item["decision"].update(decision_observed=True, censored=False,
                    confirmation_time_s=analysis["decision_cutoff_time_s"],
                    original_confirmation_time_s=analysis["decision_cutoff_original_time_s"])
            renderer = RecordingRenderer()
            with tempfile.TemporaryDirectory() as temporary:
                with mock.patch.object(filters, "compare_filter_capture", return_value=analysis):
                    with self.subTest(change=change), self.assertRaises(ValueError):
                        charts.write_filter_capture(temporary, "synthetic_bad_variant", "x", capture, record,
                                                    .8, .8, simulated=True, renderer=renderer)
                self.assertEqual(renderer.payloads, [])
                self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_joint_reduction_preserves_one_time_array_and_excludes_full_variant_arrays(self):
        capture, record = synthetic_filter_home(count=12000)
        capture["_native_private_test_token"] = "must not enter the public plotting payload"
        prepared = charts.prepare_filter_capture("synthetic_bounded_variants", "x", capture, record,
                                                  .8, .8, simulated=True)
        payload = json.loads(prepared.receipt.payload_json)
        analysis = payload["analysis"]
        self.assertEqual(analysis["sample_count"], 12000)
        self.assertLessEqual(len(analysis["trace"]), charts._MAX_CAPTURE_TRACE_POINTS)
        self.assertTrue(all(len(row) == 15 for row in analysis["trace"]))
        self.assertLess(len(prepared.receipt.payload_json.encode("utf-8")), charts._MAX_FILTER_RECEIPT_BYTES)
        self.assertEqual(private_native_keys(payload), [])
        spectrum = analysis["noise"]["spectrum"]
        for name in ("raw_xyz_g", "filtered_xyz_g", "comparison_xyz_g"):
            self.assertLessEqual(len(spectrum[name]), 4096)
        for item in analysis["variants"].values():
            self.assertNotIn("trace", item)
            self.assertNotIn("level", item)
            self.assertNotIn("weighted_level", item)

    def test_large_variant_payload_cancellation_and_render_failure_publish_nothing(self):
        _capture, _record, prepared, _payload = self.prepare(count=12000)
        for failure in ("cancel", "renderer"):
            renderer, calls = RecordingRenderer(), [0]
            def running():
                calls[0] += 1
                if failure == "cancel" and calls[0] == 2:
                    raise RuntimeError("synthetic cancellation after renderer received bounded variants")
            if failure == "renderer":
                renderer.render = mock.Mock(side_effect=ValueError("synthetic renderer failure"))
            with tempfile.TemporaryDirectory() as temporary:
                with self.subTest(failure=failure), self.assertRaises((RuntimeError, ValueError)):
                    charts.publish_filter_captures(temporary, [prepared], renderer, check_running=running)
                self.assertEqual(list(Path(temporary).iterdir()), [])


class RectifiedAccuracyReceiptTests(unittest.TestCase):
    def test_rectified_runs_require_report_v3_with_or_without_censored_response(self):
        for count in (0, 1):
            report = synthetic_public_censored_report(filter_version=2)
            if not count:
                report.update(censored_response_count=0, censored_responses=[])
            self.assertEqual(charts._accuracy_report_from_png(charts._png_with_accuracy_report(png_bytes(), report)), report)
            for old_version in (1, 2):
                changed = copy.deepcopy(report)
                changed["version"] = old_version
                with self.subTest(censored=count, version=old_version), self.assertRaises(ValueError):
                    charts._validate_accuracy_report(changed)

    def test_historical_signed_receipts_retain_exact_config_and_raw_stays_readable(self):
        for report_version in (1, 2):
            report = synthetic_public_censored_report(filter_version=1)
            report["version"] = report_version
            if report_version == 1:
                del report["censored_response_count"], report["censored_responses"]
            restored = charts._accuracy_report_from_png(charts._png_with_accuracy_report(png_bytes(), report))
            self.assertNotIn("signal_mode", restored["context"]["detector_config"])
            self.assertEqual(charts._algorithm_label(restored["context"]), "FILTERED v1 signed")
        report["version"] = 1
        report["context"]["detector_type"] = "raw"
        del report["censored_response_count"], report["censored_responses"]
        self.assertEqual(charts._validate_accuracy_report(report), report)

    def test_unknown_or_contradictory_persisted_filter_identity_fails_clearly(self):
        for change in ("unknown_report", "unknown_filter", "incomplete_config", "wrong_mode", "wrong_version"):
            report = synthetic_public_censored_report(filter_version=2)
            if change == "unknown_report":
                report["version"] = 4
            elif change == "unknown_filter":
                report["context"]["detector_version"] = 3
            elif change == "incomplete_config":
                report["context"]["detector_config"] = {"version": 2, "signal_mode": "rectified"}
            elif change == "wrong_mode":
                report["signal_mode"] = "signed"
            else:
                report["context"]["detector_version"] = 1
            with self.subTest(change=change), self.assertRaises(ValueError):
                charts._validate_accuracy_report(report)

    def test_actual_v1_v2_baselines_show_algorithm_treatment_without_pooling(self):
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as temporary:
            baseline = synthetic_native_accuracy_axis("synthetic_signed_actual", filter_version=1)
            current = synthetic_native_accuracy_axis("synthetic_rectified_actual", filter_version=2)
            charts.write_accuracy(temporary, "synthetic_signed_actual", {"x": baseline}, "complete",
                                  simulated=True, renderer=RecordingRenderer())
            charts.write_accuracy(temporary, "synthetic_rectified_actual", {"x": current}, "complete",
                simulated=True, renderer=renderer, baseline_run_id="synthetic_signed_actual")
        payload = renderer.payloads[0]
        self.assertEqual(payload["baseline"]["version"], 4)
        self.assertEqual(payload["report"]["version"], 4)
        self.assertTrue(payload["comparison"]["independent_runs"])
        self.assertTrue(payload["comparison"]["comparable_context"])
        self.assertIn("detector_version", payload["comparison"]["treatment_differences"])
        self.assertIn("detector_config", payload["comparison"]["treatment_differences"])
        self.assertEqual(payload["report"]["statistics"], payload["baseline"]["statistics"])


if __name__ == "__main__":
    unittest.main()
