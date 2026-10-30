"""Bounded automatic diagnostic retention; entirely synthetic, no printer access."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import types
import tracemalloc
import unittest
from unittest import mock

import accel_homing_charts as charts
import accel_homing_filters as filters
from harness import counts, make_homing, plugin
from test_chronological_clipping import native_filtered_capture
from test_accuracy_workflow import AccuracyRig


def native_home(version=2, detector_type="filtered", **kwargs):
    capture, record = native_filtered_capture(clock_hz=32000000., sample_period=.000302,
        filter_config={"version": version}, mode="accuracy", detector_type=detector_type, finish_home_wait=True, **kwargs)
    record.update(run_id="telemetry_test", workflow_phase="measurement", accuracy_sample=1,
                  sensor_context={"full_scale_g": 2047 * .0078125})
    return capture, record


class EventWindowTests(unittest.TestCase):
    def test_exact_original_times_and_decision_survive_signed_axes_and_capture_remap(self):
        for version in (1, 2):
            for shift in (-.001, .00002):
                with self.subTest(version=version, shift=shift):
                    capture, record = native_home(version, clock_shift=shift,
                        axes_map=((1, 1), (0, -1), (2, 1)), packet_size=80, split=40,
                        clip_position=45, row_values=[(0, 0, counts(1.))] * 40 + [(counts(3.), 0, counts(1.))] * 40)
                    before = copy.deepcopy(record)
                    diagnostic = filters.prepare_accuracy_home_telemetry(capture, record)
                    home = filters.validate_accuracy_home_telemetry(diagnostic, record)
                    self.assertEqual(record, before)
                    self.assertIsNotNone(home["returned_trigger_time_s"])
                    self.assertEqual(home["window"]["axes_map"], [[1, 1], [0, -1], [2, 1]])
                    self.assertEqual(len(home["window"]["rows"]), 50)
                    self.assertFalse(home["window"]["downsampled"])
                    self.assertTrue(home["state_provenance"]["actual_decision_verified"])
                    self.assertTrue(home["state_provenance"]["replay_candidate_summary_matches_actual"])
                    self.assertFalse(home["state_provenance"]["online_snapshot"])
                    for event in ("onset", "crossing", "confirmation"):
                        self.assertEqual(home["events"][event]["original_time_s"], record["trigger_" + event + "_time_s"])
                    for row in home["window"]["rows"]:
                        self.assertAlmostEqual(row["capture_time_s"] - row["original_time_s"], shift)
                        if not row["processed_by_actual_filter"]:
                            self.assertIsNone(row["filtered_sensor_xyz_g"])
                    marker = next(row for row in home["window"]["rows"] if any(row["marker_axes"]))
                    self.assertEqual(marker["raw_sensor_xyz_counts"][0], 32767)
                    self.assertEqual(marker["decoded_sensor_xyz_counts"][0], 2047)
                    self.assertFalse(marker["valid_original_axes"][0])
                    self.assertLess(marker["decoded_sensor_xyz_g"][0], 16.)
                    capture["_native_filter_reference"]["offset_g"][0] = 99.
                    native_home(version, clip_position=None)
                    capture.clear()
                    record.clear()
                    self.assertEqual(json.loads(diagnostic), home)

    def test_reconstructed_state_replays_only_actual_processed_window_rows(self):
        capture, record = native_home(packet_size=80, split=40, clip_position=45,
            row_values=[(counts(.1), 0, counts(1.))] * 40 + [(counts(3.), 0, counts(1.))] * 40)
        home = json.loads(filters.prepare_accuracy_home_telemetry(capture, record))
        ref, state = home["filter_reference_sensor_frame"], home["state_before_window"]
        engine = filters.CausalImpactDetector(home["threshold_g"], ref["offset_g"], ref["noise_sigma_g"], ref["weights_xyz"], home["detector_config"])
        engine.previous_time, engine.filtered_xyz = state["previous_time_s"], tuple(state["filtered_sensor_xyz_g"])
        engine.history.extend((t, level, raw, tuple(identity)) for t, level, raw, identity in state["history"])
        engine.candidate = state["candidate"]
        for row in home["window"]["rows"]:
            if row["processed_by_actual_filter"]:
                decision = engine.feed(row["original_time_s"], row["decoded_sensor_xyz_g"], tuple(row["sample"]))
                self.assertEqual(list(engine.filtered_xyz), row["filtered_sensor_xyz_g"])
        self.assertEqual(decision["confirmation_time_s"], record["trigger_confirmation_time_s"])
        self.assertLessEqual(len(state["history"]), 128)

    def test_arm_straddling_does_not_feed_prearm_rows_and_labels_tail(self):
        capture, record = native_home(arm_straddle=2, packet_size=16, split=8, clip_position=2)
        home = json.loads(filters.prepare_accuracy_home_telemetry(capture, record))
        armed = home["state_provenance"]["first_armed_sample"]
        first_armed = next(row for row in home["window"]["rows"] if row["sample"] == armed)
        self.assertEqual(first_armed["sample"][2], 2)
        self.assertTrue(first_armed["processed_by_actual_filter"])
        self.assertTrue(any(not row["processed_by_actual_filter"] for row in home["window"]["rows"]))
        self.assertTrue(any(row["stop_relation"] == "after_upper" for row in home["window"]["rows"]))

    def test_raw_preserves_actual_controller_marker_seed_and_original_signed_codes(self):
        rows = [(-counts(3.), 0, counts(1.))] * 2 + [(32767, 0, counts(1.))] + [(counts(3.), 0, counts(1.))] * 5
        capture, record = native_home(detector_type="raw", row_values=rows, clip_position=None, arm_straddle=2)
        home = json.loads(filters.prepare_accuracy_home_telemetry(capture, record))
        marker = next(row for row in home["window"]["rows"] if any(row["marker_axes"]))
        self.assertEqual(marker["decoded_sensor_xyz_counts"][0], 2047)
        self.assertEqual(marker["decoder_previous_sensor_counts"][0], 0)
        self.assertEqual(home["impact_metric"], "xyz_magnitude")
        self.assertFalse(home["state_provenance"]["available"])
        self.assertEqual(home["events"]["crossing"]["sample"], home["events"]["onset"]["sample"])

    def test_unsupported_capture_reports_unavailable_and_preserves_legacy_direction(self):
        record = {"result": "impact", "run_id": "raw", "axis": "x", "detector_type": "raw",
                  "impact_metric": "selected_axis", "direction": -1}
        home = json.loads(filters.prepare_accuracy_home_telemetry({"samples": []}, record))
        self.assertFalse(home["window"]["available"])
        self.assertEqual(home["window"]["rows"], [])
        self.assertEqual(home["direction"], -1)
        self.assertIsNone(home["events"]["confirmation"]["original_time_s"])
        self.assertIsNone(home["planned_movement_start_time_s"])

    def test_forged_original_row_or_receipt_binding_rejected(self):
        capture, record = native_home()
        bad = copy.deepcopy(capture)
        data = bytearray(bad["_native_source_batches"][-1]["data"])
        data[0] ^= 1
        bad["_native_source_batches"][-1]["data"] = bytes(data)
        with self.assertRaises(ValueError):
            filters.prepare_accuracy_home_telemetry(bad, record)
        encoded = filters.prepare_accuracy_home_telemetry(capture, record)
        with self.assertRaises(ValueError):
            filters.validate_accuracy_home_telemetry(encoded, dict(record, threshold_g=9.))

    def test_stop_relations_are_bounds_not_measured_halt(self):
        self.assertEqual([filters._stop_relation(t, [1., 2.]) for t in (.9, 1., 1.5, 2., 2.1)],
                         ["before_lower", "within_request_interval", "within_request_interval", "within_request_interval", "after_upper"])
        self.assertEqual(filters._stop_relation(1., None), "unknown")

    def test_bounded_receipt_rejects_oversize_history_and_rows(self):
        record = {"result": "impact", "detector_type": "raw"}
        home = json.loads(filters.prepare_accuracy_home_telemetry({}, record))
        for key in ("rows", "history"):
            changed = copy.deepcopy(home)
            if key == "rows":
                changed["window"]["rows"] = [{}] * 51
            else:
                changed["state_before_window"] = {"history": [None] * 129}
            with self.assertRaises(ValueError):
                filters.validate_accuracy_home_telemetry(charts._json(changed), record)

    def test_large_single_packet_expands_only_selected_fifty_rows(self):
        count = 100000
        source = {"data": (0, 0, 128) * count, "start_clock": 1000, "delta_clock": count - 1,
            "first_sample_position": 0, "decode_previous_raw": [0, 0, 128], "clip_values": [2047, -2048],
            "count_g": .0078125, "filter_start_time_s": 1., "filter_end_time_s": 100.,
            "start_time_s": 1., "end_time_s": 100.}
        record = {"result": "impact", "detector_type": "raw", "onset_sample": [1000, count - 1, 50000, count]}
        capture = {"_accuracy_native_sources": [source], "_accuracy_native_mapping": [[0, 1], [1, 1], [2, 1]]}
        tracemalloc.start()
        try:
            home = json.loads(filters.prepare_accuracy_home_telemetry(capture, record))
            _current, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        self.assertEqual(len(home["window"]["rows"]), 50)
        self.assertLess(peak, 4 * 1024 * 1024)

    def test_raw_event_snapshot_returned_time_and_later_capture_are_separate_domains(self):
        capture, record = native_home(detector_type="raw", clock_shift=-.001)
        record["returned_trigger_time_s"] = record["trigger_onset_time_s"] + .00001
        home = json.loads(filters.prepare_accuracy_home_telemetry(capture, record))
        event = home["events"]["onset"]
        self.assertEqual(event["original_time_origin"], "raw_disarm_trigger_time_conversion")
        self.assertEqual(event["original_time_s"], record["trigger_onset_time_s"])
        self.assertNotEqual(event["capture_time_s"], event["original_time_s"])
        self.assertNotEqual(home["returned_trigger_time_s"], event["capture_time_s"])

    def test_accuracy_ownership_does_not_change_actual_native_decisions(self):
        for detector in ("raw", "filtered"):
            for version in (1, 2):
                options = dict(detector_type=detector, filter_config={"version": version}, clip_position=2,
                               clock_hz=32000000., sample_period=.000302)
                _base_capture, baseline = native_filtered_capture(mode="calibration", **options)
                capture, instrumented = native_filtered_capture(mode="accuracy", **options)
                for field in ("result", "onset_sample", "onset_g", "trigger_onset_time_s",
                              "trigger_crossing_time_s", "trigger_confirmation_time_s", "stop_request_interval_s"):
                    self.assertEqual(instrumented[field], baseline[field])
                self.assertTrue(capture["_accuracy_native_sources"])


class RuntimeAndPublicationTests(unittest.TestCase):
    def test_native_raw_and_filtered_complete_actual_accuracy_command_with_original_windows(self):
        from test_filter_engine import FilteredRig
        class NativeAccuracyRig(FilteredRig):
            def stop_capture(self):
                existing = super().stop_capture()
                source = existing.get("_record")
                if source is None:
                    return existing
                capture, native = native_filtered_capture(axis=source["axis"], mode="accuracy",
                    detector_type=source["detector_type"], filter_config=source.get("detector_config"),
                    time_offset=self.capture_stops * 10.,
                    clock_shift=-.001 if source["detector_type"] == "filtered" else .00002, arm_straddle=2)
                source.update(native)
                capture["_record"] = source
                return capture
        for detector in ("raw", "filtered"):
            with self.subTest(detector=detector), tempfile.TemporaryDirectory() as directory:
                rig = NativeAccuracyRig({"charts_directory": directory})
                rig.accuracy(DETECTOR=detector.upper(), **{detector.upper() + "_THRESHOLD": 1.})
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "complete")
                sidecar = json.loads(Path(result["telemetry_paths"]["x_json"]).read_text())
                self.assertEqual(len(sidecar["trials"]), 2)
                self.assertTrue(all(item["window"]["available"] for item in sidecar["trials"]))
                self.assertTrue(all(any(any(row["marker_axes"]) for row in item["window"]["rows"]) for item in sidecar["trials"]))
                self.assertEqual(rig.configfile.saved, {})

    def test_home_wait_records_the_same_single_evaluated_return_after_stop(self):
        obj = make_homing()
        attempt = types.SimpleNamespace(record={})
        endstop = plugin.BeaconAccelEndstop(obj, "x")
        order = []
        endstop._dispatch = types.SimpleNamespace(wait_end=lambda t: order.append("wait"), stop=lambda: order.append("stop") or 1)
        obj.disarm = lambda: attempt
        obj.finish = lambda *args: order.append("finish") or 1.23456789
        self.assertEqual(endstop.home_wait(2.), 1.23456789)
        self.assertEqual(order, ["wait", "stop", "finish"])
        self.assertEqual(attempt.record["returned_trigger_time_s"], 1.23456789)

    def test_successful_accuracy_automatically_saves_separate_discovery_and_30_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.accuracy(samples=30, DETECTOR="RAW")
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            data = json.loads(Path(result["telemetry_paths"]["x_json"]).read_text())
            png = Path(result["summary_paths"]["x_png"]).read_bytes()
            self.assertEqual(data["png_sha256"], hashlib.sha256(png).hexdigest())
            self.assertEqual(data["discovery"]["measurement_index"], 0)
            self.assertEqual([item["measurement_index"] for item in data["trials"]], list(range(1, 31)))
            self.assertEqual(charts._accuracy_report_from_png(png)["version"], 1)
            self.assertNotIn("window", json.dumps(result))
            self.assertNotIn("accuracy_telemetry", result)
            self.assertEqual(charts.read_accuracy_baseline(directory, result["run_id"], "x")["count"], 30)

    def test_json_publication_failure_rolls_back_png_and_keeps_existing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            old = Path(directory) / "previous.json"
            old.write_text("user data")
            rig = AccuracyRig({"charts_directory": directory})
            original = charts.os.link
            def fail_json(source, target):
                if str(target).endswith(".json"):
                    raise OSError("injected JSON publication error")
                return original(source, target)
            with mock.patch.object(charts.os, "link", side_effect=fail_json):
                with self.assertRaises(Exception):
                    rig.accuracy(DETECTOR="RAW")
            self.assertEqual(list(Path(directory).iterdir()), [old])
            self.assertEqual(old.read_text(), "user data")
            self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_failed_home_publishes_no_json_or_png(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.home_results.extend([{}, {"result": "invalid_data"}])
            with self.assertRaises(Exception):
                rig.accuracy(DETECTOR="RAW")
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_two_axes_fifty_measurements_stay_out_of_status_payloads(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.accuracy(axis="XY", samples=50, DETECTOR="RAW")
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertEqual(set(result["summary_paths"]), {"x_png", "y_png"})
            self.assertEqual(set(result["telemetry_paths"]), {"x_json", "y_json"})
            for filename in result["telemetry_paths"].values():
                data = json.loads(Path(filename).read_text())
                self.assertEqual(len(data["trials"]), 50)
                self.assertIsNotNone(data["discovery"])
                self.assertLess(Path(filename).stat().st_size, charts._MAX_ACCURACY_TELEMETRY_BYTES)
            self.assertNotIn("raw_sensor_xyz_counts", json.dumps(result))

    def test_cancellation_between_png_and_json_rolls_back_both(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            original = charts._publish
            def cancel_json(*args, **kwargs):
                if kwargs.get("_extension") == ".json":
                    raise RuntimeError("cancelled between paired publications")
                return original(*args, **kwargs)
            with mock.patch.object(charts, "_publish", side_effect=cancel_json):
                with self.assertRaises(Exception):
                    rig.accuracy(DETECTOR="RAW")
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_invalid_diagnostic_builder_aborts_actual_workflow_without_artifacts(self):
        from test_accuracy_workflow import workflow_module
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            with mock.patch.object(workflow_module, "prepare_accuracy_home_telemetry", side_effect=ValueError("native provenance mismatch")):
                with self.assertRaises(Exception):
                    rig.accuracy(DETECTOR="RAW")
            self.assertEqual(list(Path(directory).iterdir()), [])
            self.assertEqual(rig.workflow.last_result["result"], "aborted")


if __name__ == "__main__":
    unittest.main()
