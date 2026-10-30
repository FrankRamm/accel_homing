"""Offline experimental plots and baseline receipts. ALL DATA ARE SYNTHETIC."""
import copy
import json
import math
import os
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest
from unittest import mock
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import accel_homing_charts as charts
import accel_homing_filters as filters
from test_accuracy_charts import RecordingRenderer, png_bytes


def synthetic_filter_home(axis="x", detector="raw", count=1450, filter_version=None):
    """10 kHz stationary noise followed by a generated pulse, not stop evidence."""
    samples = []
    for index in range(count):
        when = 10. + index * .0001
        x = .2 + .065 * math.sin(index * .45)
        y = -.1 + .035 * math.sin(index * .71)
        z = 1. + .02 * math.cos(index * .61)
        if 1200 <= index < 1212:
            x += 2.
            y += .35
        if index >= 1212:
            x += 3.5 * math.exp(-(index - 1212) / 28.) * math.sin((index - 1212) * .7)
        samples.append((when, x, y, z))
    arm, crossing = 10.1, 10.12
    stop = [10.1202, 10.1206] if detector == "raw" else [10.121, 10.1214]
    config = filters.detector_config(None if filter_version is None else {"version": filter_version})
    reference = filters.robust_reference([row[1:] for row in samples if row[0] < arm], config)
    metadata = {"axis": axis, "detector_type": detector, "impact_metric": "xyz_magnitude",
                "arm_time_s": arm, "onset_time_s": crossing, "crossing_time_s": crossing,
                "reference_interval_s": [10., 10.0999],
                "reference_sample_count": 1000,
                "confirmation_time_s": crossing if detector == "raw" else crossing + .0005,
                "stop_request_interval_s": stop, "stop_request_time_s": stop[1],
                "detector_version": config["version"] if detector == "filtered" else 0,
                "threshold_g": .8, "detector_config": config, "filter_reference": reference}
    capture = {"samples": samples, "baseline_g": (.2, -.1, 1.), "raw_baseline_g": (.2, -.1, 1.),
               "dropped": 0, "fault": None, "detector": metadata}
    record = {"axis": axis, "result": "impact", "capture_valid": True, "travel_mm": 175.012,
              "travel_mm_exact": 175.01234, "threshold_g": .8, "onset_g": 2.,
              "impact_peak_g": 3.5, "onset_time_s": crossing, "arm_time_s": arm,
              "direction": 0, "impact_metric": "xyz_magnitude", "detector_type": detector,
              "detector_version": config["version"] if detector == "filtered" else 0,
              "detector_config": config if detector == "filtered" else None}
    return capture, record


def synthetic_accuracy_axis(run_id, detector="raw", travels=(175.0, 175.2, 174.9, 175.1),
                            threshold=1., speed=50., filter_version=None):
    """Independent generated commanded-travel series with complete contexts."""
    item = {"requested_samples": len(travels), "complete": True, "discovery": None,
            "trials": [], "statistics": None, "capture_paths": [], "discovery_capture_paths": None,
            "rejected": None}
    config = filters.detector_config(None if filter_version is None else {"version": filter_version})
    for index, travel in enumerate((174.8,) + tuple(travels)):
        origin = 20. + index
        record = {"run_id": run_id, "axis": "x", "workflow_attempt": index + 1,
                  "workflow_phase": "discovery" if index == 0 else "measurement", "accuracy_sample": index,
                  "result": "impact", "capture_valid": True, "travel_mm": travel, "travel_mm_exact": travel,
                  "threshold_g": threshold, "workflow_threshold_g": threshold, "direction": 0,
                  "onset_g": 2., "impact_peak_g": 3., "onset_time_s": origin + .002,
                  "impact_metric": "xyz_magnitude", "detector_type": detector,
                  "detector_version": config["version"] if detector == "filtered" else 0,
                  "detector_config": config if detector == "filtered" else None,
                  "homing_speed_mm_s": speed, "configured_homing_speed_mm_s": speed, "homing_accel_mm_s2": 5000.,
                  "sensor_context": {"sensor": "adxl345 synthetic", "backend": "generic",
                                     "axes_map": "x,y,z", "full_scale_g": 16.},
                  "geometry_context": {"position_min": 0., "position_max": 350., "position_endstop": 350.,
                                       "homing_positive_dir": True}}
        capture = {"samples": [(origin, .2, -.1, 1.), (origin + .001, .2, -.1, 1.),
                                (origin + .002, 2.2, -.1, 1.), (origin + .003, 3.2, -.1, 1.)],
                   "baseline_g": (.2, -.1, 1.), "dropped": 0}
        prepared = charts.prepare_capture("%s_a%03d" % (run_id, index + 1), "x", capture, record, simulated=True)
        if index == 0:
            item["discovery"], item["discovery_capture_paths"] = record, prepared
        else:
            item["trials"].append(record)
            item["capture_paths"].append(prepared)
    return item


class FilterComparisonTests(unittest.TestCase):
    def prepare(self, **kwargs):
        capture, record = synthetic_filter_home(**kwargs)
        prepared = charts.prepare_filter_capture("synthetic_filters", record["axis"], capture, record,
                                                  .8, .8, simulated=True)
        return capture, record, prepared, json.loads(prepared.receipt.payload_json)

    def test_same_sample_series_and_real_profile_initialization(self):
        capture, record, prepared, payload = self.prepare(detector="filtered")
        analysis = payload["analysis"]
        reference = capture["detector"]["filter_reference"]
        engine = filters.CausalImpactDetector(.8, reference["offset_g"], reference["noise_sigma_g"],
                                               reference["weights_xyz"], filters.detector_config())
        armed = [row for row in capture["samples"] if row[0] >= 10.1]
        period = .0001
        engine.previous_time = armed[0][0] - period
        by_time = {row[0]: row for row in analysis["trace"]}
        for row in armed:
            engine.feed(row[0], row[1:])
            displayed = by_time[row[0]]
            for first, second in zip(displayed[4:7], engine.filtered_xyz):
                self.assertAlmostEqual(first, second, places=11)
            self.assertAlmostEqual(displayed[9], engine.level_g, places=11)
        self.assertFalse(prepared)
        self.assertTrue(payload["experimental"])

    def test_noise_is_real_pre_arm_replay_not_zero_or_contact_response(self):
        capture, record, prepared, payload = self.prepare()
        noise = payload["analysis"]["noise"]
        self.assertTrue(noise["available"])
        self.assertGreater(noise["raw"]["rms_magnitude_g"], .01)
        self.assertGreater(noise["filtered"]["rms_magnitude_g"], .001)
        self.assertLess(noise["interval_s"][1], record["arm_time_s"])
        self.assertGreaterEqual(noise["interval_s"][0], 10.001)
        self.assertLess(noise["raw"]["p99_magnitude_g"], .15)
        self.assertTrue(noise["spectrum"]["available"])
        self.assertLessEqual(noise["spectrum"]["sample_count"], 4096)

    def test_saved_shadow_filter_configuration_is_used_and_frozen(self):
        capture, record = synthetic_filter_home()
        config = filters.detector_config({"lowpass_tau_s": .0006})
        prepared = charts.prepare_filter_capture("custom_shadow", "x", capture, record, .8, .8,
                                                  simulated=True, filter_config=config)
        analysis = json.loads(prepared.receipt.payload_json)["analysis"]
        self.assertEqual(analysis["filter_config"], config)
        config["lowpass_tau_s"] = 0.
        self.assertEqual(json.loads(prepared.receipt.payload_json)["analysis"]["filter_config"]["lowpass_tau_s"], .0006)
        replay = filters.compare_filter_capture(capture, .8, .8, {"lowpass_tau_s": .0006})
        self.assertEqual(analysis["trace"][1100][4:7], replay["filtered_trace"][1100][1:])

    def test_noise_excludes_motor_transients_before_validated_reference(self):
        capture, record = synthetic_filter_home()
        capture["detector"]["reference_interval_s"] = [10.05, 10.0999]
        capture["detector"]["reference_sample_count"] = 500
        capture["samples"] = [(row[0], row[1] + 4., row[2], row[3]) if row[0] < 10.05 else row
                              for row in capture["samples"]]
        prepared = charts.prepare_filter_capture("validated_interval", "x", capture, record, .8, .8, simulated=True)
        noise = json.loads(prepared.receipt.payload_json)["analysis"]["noise"]
        self.assertTrue(noise["available"])
        self.assertGreaterEqual(noise["interval_s"][0], 10.051)
        self.assertLess(noise["raw"]["p99_magnitude_g"], .15)

    def test_irregular_prearm_interval_has_no_spectrum(self):
        capture, _record = synthetic_filter_home()
        analysis = filters.compare_filter_capture(capture, .8, .8)
        for rows in [analysis["raw_trace"]] + [item["trace"] for item in analysis["variants"].values()]:
            for index in range(1, 998, 2):
                rows[index][0] += .00003
        noise = charts._filter_noise_summary(analysis)
        self.assertTrue(noise["available"])
        self.assertFalse(noise["spectrum"]["available"])
        self.assertIn("irregular", noise["spectrum"]["reason"])

    def test_zero_pre_arm_display_is_never_presented_as_noise_attenuation(self):
        capture, _record = synthetic_filter_home()
        analysis = filters.compare_filter_capture(capture, .8, .8)
        analysis["pre_arm_filter_replay"] = False
        self.assertFalse(charts._filter_noise_summary(analysis)["available"])

    def test_raw_actual_stop_censors_later_filtered_decision(self):
        capture, record, prepared, payload = self.prepare()
        analysis = payload["analysis"]
        self.assertTrue(analysis["raw_decision"]["decision_observed"])
        self.assertTrue(analysis["filtered_decision"]["censored"])
        self.assertFalse(analysis["filtered_decision"]["decision_observed"])
        self.assertIsNone(analysis["filtered_decision"].get("confirmation_time_s"))
        changed = copy.deepcopy(capture)
        changed["samples"] = [(row[0], row[1] + 6., row[2], row[3])
                              if row[0] >= analysis["decision_cutoff_time_s"] else row for row in capture["samples"]]
        replay = filters.compare_filter_capture(changed, .8, .8)
        self.assertEqual(replay["filtered_decision"], analysis["filtered_decision"])
        self.assertEqual(replay["moving_noise"], analysis["moving_noise"])

    def test_joint_reduction_remains_bounded_and_does_not_change_noise_metrics(self):
        capture, record, prepared, payload = self.prepare(count=8000)
        analysis = payload["analysis"]
        self.assertLessEqual(len(analysis["trace"]), charts._MAX_CAPTURE_TRACE_POINTS)
        self.assertEqual(analysis["sample_count"], 8000)
        replay = filters.compare_filter_capture(capture, .8, .8)
        self.assertEqual(analysis["noise"], charts._filter_noise_summary(replay))
        self.assertTrue(all(len(row) == 15 for row in analysis["trace"]))

    def test_fault_and_axis_mismatch_never_write_output(self):
        capture, record = synthetic_filter_home()
        with tempfile.TemporaryDirectory() as temporary:
            capture["fault"] = "synthetic fault"
            with self.assertRaisesRegex(ValueError, "fault"):
                charts.write_filter_capture(temporary, "bad", "x", capture, record, .8, .8,
                                            renderer=RecordingRenderer())
            capture["fault"] = None
            with self.assertRaisesRegex(ValueError, "axis"):
                charts.write_filter_capture(temporary, "bad", "y", capture, record, .8, .8,
                                            renderer=RecordingRenderer())
            self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_all_axes_render_before_any_publication(self):
        first = self.prepare()[2]
        second = self.prepare(axis="y")[2]
        renderer = RecordingRenderer()
        original = renderer.render
        def render(payload, check_running=None):
            if payload["axis"] == "y":
                raise ValueError("synthetic renderer failure")
            return original(payload, check_running)
        renderer.render = render
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(ValueError, "renderer failure"):
                charts.publish_filter_captures(temporary, [first, second], renderer)
            self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_success_saves_png_only(self):
        _capture, _record, prepared, _payload = self.prepare()
        with tempfile.TemporaryDirectory() as temporary:
            result = charts.publish_filter_captures(temporary, [prepared], RecordingRenderer())[0]
            self.assertEqual(set(result), {"png"})
            self.assertEqual([path.suffix for path in Path(temporary).iterdir()], [".png"])


class AccuracyBaselineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def write(self, run_id="raw_control", detector="raw", speed=50., renderer=None, baseline=None):
        item = synthetic_accuracy_axis(run_id, detector, threshold=1.5 if detector == "filtered" else 1., speed=speed)
        return charts.write_accuracy(self.root, run_id, {"x": item}, "complete", simulated=True,
                                      renderer=renderer or RecordingRenderer(), baseline_run_id=baseline)

    def test_completed_baseline_survives_re_read_and_embeds_no_sidecar(self):
        paths = self.write()
        receipt = charts.read_accuracy_baseline(self.root, "raw_control", "x", simulated=True)
        self.assertEqual(receipt["count"], 4)
        self.assertEqual(receipt["context"]["detector_type"], "raw")
        self.assertFalse(receipt["physical_position_verified"])
        self.assertEqual(list(self.root.iterdir()), [Path(paths["x_png"])])
        self.assertGreater(Path(paths["x_png"]).stat().st_size, len(png_bytes()))

    def test_actual_ab_series_are_separate_and_treatment_differences_explicit(self):
        self.write()
        renderer = RecordingRenderer()
        self.write("filtered_test", "filtered", renderer=renderer, baseline="raw_control")
        payload = renderer.payloads[0]
        comparison = payload["comparison"]
        self.assertTrue(comparison["comparable_context"])
        self.assertTrue(comparison["independent_runs"])
        self.assertFalse(comparison["paired_physical_measurement"])
        self.assertIn("threshold_g", comparison["treatment_differences"])
        self.assertIn("detector_type", comparison["treatment_differences"])
        self.assertEqual(payload["report"]["count"], 4)
        self.assertEqual(payload["baseline"]["count"], 4)

    def test_changed_speed_prohibits_comparable_context_claim(self):
        self.write()
        renderer = RecordingRenderer()
        self.write("different_speed", "filtered", speed=40., renderer=renderer, baseline="raw_control")
        comparison = renderer.payloads[0]["comparison"]
        self.assertFalse(comparison["comparable_context"])
        self.assertIn("homing_speed_mm_s", comparison["mismatched_context"])

    def test_changed_or_missing_acceleration_prohibits_comparable_claim(self):
        self.write()
        baseline = charts.read_accuracy_baseline(self.root, "raw_control", "x", simulated=True)
        for value in (None, 2500.):
            current = copy.deepcopy(baseline)
            current["run_id"] = "changed_acceleration"
            current["context"]["homing_accel_mm_s2"] = value
            comparison = charts._compare_accuracy_reports(current, baseline)
            self.assertFalse(comparison["comparable_context"])
            self.assertIn("homing_accel_mm_s2", comparison["mismatched_context"])
            if value is None:
                self.assertIn("homing_accel_mm_s2", comparison["missing_context"])

    def test_baseline_paths_wrong_axis_and_actual_synthetic_mix_rejected(self):
        self.write()
        for run_id in ("../raw_control", "raw/control", "x" * 101, "raw_control.png"):
            with self.subTest(run_id=run_id), self.assertRaisesRegex(ValueError, "run ID"):
                charts.read_accuracy_baseline(self.root, run_id, "x", simulated=True)
        with self.assertRaisesRegex(ValueError, "exactly one"):
            charts.read_accuracy_baseline(self.root, "raw_control", "y", simulated=True)
        with self.assertRaisesRegex(ValueError, "provenance"):
            charts.read_accuracy_baseline(self.root, "raw_control", "x", simulated=False)

    def test_duplicate_run_receipts_are_not_silently_selected(self):
        paths = self.write()
        original = Path(paths["x_png"])
        shutil.copyfile(original, self.root / (original.stem + "_duplicate.png"))
        with self.assertRaisesRegex(ValueError, "exactly one"):
            charts.read_accuracy_baseline(self.root, "raw_control", "x", simulated=True)

    def test_symlink_is_rejected_without_following_its_target(self):
        paths = self.write()
        original = Path(paths["x_png"])
        target = self.root / "outside.png"
        original.rename(target)
        try:
            original.symlink_to(target)
        except OSError as exc:
            self.skipTest("Symlink creation unavailable: %s" % exc)
        with self.assertRaisesRegex(ValueError, "symlink"):
            charts.read_accuracy_baseline(self.root, "raw_control", "x", simulated=True)

    def test_missing_corrupt_and_incomplete_receipts_rejected(self):
        path = self.root / "Accel Homing Accuracy X - Run corrupt - 2026-10-29.png"
        path.write_bytes(png_bytes())
        with self.assertRaisesRegex(ValueError, "no completed accuracy receipt"):
            charts.read_accuracy_baseline(self.root, "corrupt", "x", simulated=True)
        path.write_bytes(png_bytes()[:-5])
        with self.assertRaisesRegex(ValueError, "PNG"):
            charts.read_accuracy_baseline(self.root, "corrupt", "x", simulated=True)
        item = synthetic_accuracy_axis("incomplete")
        report = charts._accuracy_report("incomplete", "x", {**item, "statistics": charts._accuracy_statistics(item["trials"])}, True)
        report["count"] -= 1
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            charts._png_with_accuracy_report(png_bytes(), report)

    def test_tampered_statistics_and_large_text_fail_before_publish(self):
        self.write()
        receipt = charts.read_accuracy_baseline(self.root, "raw_control", "x", simulated=True)
        receipt["statistics"]["spread_mm"] += 1.
        with self.assertRaisesRegex(ValueError, "statistics"):
            charts._png_with_accuracy_report(png_bytes(), receipt)
        encoded = charts._ACCURACY_REPORT_KEY + b"\x00" + b" " * (charts._MAX_ACCURACY_REPORT_BYTES + 1)
        kind = b"tEXt"
        chunk = struct.pack(">I", len(encoded)) + kind + encoded + struct.pack(">I", zlib.crc32(kind + encoded) & 0xffffffff)
        data = png_bytes()[:-12] + chunk + png_bytes()[-12:]
        with self.assertRaisesRegex(ValueError, "oversized"):
            charts._accuracy_report_from_png(data)

    def test_baseline_render_failure_preserves_prior_png_without_scratch(self):
        self.write()
        before = list(self.root.iterdir())
        renderer = RecordingRenderer()
        renderer.render = mock.Mock(side_effect=ValueError("synthetic render failure"))
        with self.assertRaisesRegex(ValueError, "render failure"):
            self.write("failed_filtered", "filtered", renderer=renderer, baseline="raw_control")
        self.assertEqual(list(self.root.iterdir()), before)


if __name__ == "__main__":
    unittest.main()
