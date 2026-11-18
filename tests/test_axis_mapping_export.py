"""Failed mapping evidence from simulated captures, without printer access."""
import copy
import json
from pathlib import Path
import statistics
import tempfile
import unittest
from unittest import mock
import zipfile

import accel_homing_charts as charts
from test_axis_mapping import motion_pass


class AxisMappingExportTests(unittest.TestCase):
    def evidence(self):
        passes = [motion_pass(1, (0., -1., 0.)), motion_pass(-1, (0., -1., 0.))]
        for item in passes:
            item["capture"].update(sensor_context={"sensor": "beacon", "backend": "beacon_raw"},
                                   baseline_first_sample_time_s=-1., baseline_last_sample_time_s=0.,
                                   baseline_sample_count=1000, capture_cutoff_time_s=.9)
        return passes, {"reason": "Simulated inference rejection", "simulated": True,
                        "inference": {"version": 1, "rejected_pass": 0}}

    def test_isolated_plot_and_zip_preserve_full_signed_replay_evidence(self):
        passes, diagnostic = self.evidence()
        original = copy.deepcopy(passes)
        with tempfile.TemporaryDirectory() as directory:
            paths = charts.write_axis_mapping_failure(directory, "x", passes, diagnostic)
            width, height = charts._check_png(Path(paths["axis_mapping_png"]).read_bytes())
            self.assertGreater(width, 1000)
            self.assertGreater(height, 600)
            with zipfile.ZipFile(paths["axis_mapping_zip"]) as zipped:
                self.assertEqual(set(zipped.namelist()), {"mapping.json", "mapping.png"})
                report = json.loads(zipped.read("mapping.json"))
                self.assertEqual(zipped.read("mapping.json"), Path(paths["axis_mapping_json"]).read_bytes())
            self.assertEqual(report["passes"], json.loads(json.dumps(original)))
            self.assertEqual(report["diagnostic"], diagnostic)
            self.assertEqual(report["coordinate_frame"], "configured_sensor_stream")
            self.assertIn("not native ADC", report["sample_semantics"])
            self.assertTrue(report["simulated"])
            self.assertEqual(passes, original)
            # The signed sensor-Y component remains negative for both
            # direction-normalized strokes; no guessed mapping is applied.
            for item in passes:
                trace = charts._mapping_velocity_trace(item)
                cruise = [row[2] for row in trace if 1.1 <= row[0] <= 1.4]
                self.assertAlmostEqual(statistics.median(cruise), -40., delta=1.)
            passes[0]["capture"]["samples"].clear()
            self.assertEqual(json.loads(Path(paths["axis_mapping_json"]).read_text())["passes"], report["passes"])

    def test_render_failure_still_exports_exact_samples_and_error_in_zip(self):
        passes, diagnostic = self.evidence()
        renderer = mock.Mock()
        renderer.render.side_effect = RuntimeError("plotting unavailable")
        with tempfile.TemporaryDirectory() as directory:
            paths = charts.write_axis_mapping_failure(directory, "y", passes, diagnostic, renderer=renderer)
            self.assertNotIn("axis_mapping_png", paths)
            self.assertIn("plotting unavailable", paths["chart_error"])
            with zipfile.ZipFile(paths["axis_mapping_zip"]) as zipped:
                self.assertEqual(zipped.namelist(), ["mapping.json"])
                report = json.loads(zipped.read("mapping.json"))
            self.assertEqual(report["passes"], json.loads(json.dumps(passes)))
            self.assertEqual(report["chart_error"], paths["chart_error"])

    def test_archive_failure_retains_json_without_claiming_a_zip(self):
        passes, diagnostic = self.evidence()
        renderer = mock.Mock()
        renderer.render.side_effect = RuntimeError("plotting unavailable")
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(charts.zipfile, "ZipFile", side_effect=OSError("archive unavailable")):
            paths = charts.write_axis_mapping_failure(directory, "x", passes, diagnostic, renderer=renderer)
            self.assertNotIn("axis_mapping_zip", paths)
            self.assertIn("archive unavailable", paths["archive_error"])
            self.assertTrue(Path(paths["axis_mapping_json"]).is_file())

    def test_failed_error_annotation_keeps_completed_json_path_and_attempts_zip(self):
        passes, diagnostic = self.evidence()
        renderer = mock.Mock()
        renderer.render.side_effect = RuntimeError("plotting unavailable")
        original_open = Path.open
        def fail_annotation(path, *args, **kwargs):
            if path.name == "mapping.json.tmp":
                raise OSError("no room for annotation")
            return original_open(path, *args, **kwargs)
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(Path, "open", fail_annotation):
            paths = charts.write_axis_mapping_failure(directory, "x", passes, diagnostic, renderer=renderer)
            self.assertIn("no room for annotation", paths["annotation_error"])
            self.assertIn("plotting unavailable", paths["chart_error"])
            self.assertTrue(Path(paths["axis_mapping_json"]).is_file())
            with zipfile.ZipFile(paths["axis_mapping_zip"]) as zipped:
                report = json.loads(zipped.read("mapping.json"))
            self.assertEqual(report["passes"], json.loads(json.dumps(passes)))


if __name__ == "__main__":
    unittest.main()
