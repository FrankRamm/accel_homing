"""Real isolated PNG rendering from synthetic offline diagnostic captures."""
import importlib.util
import csv
import gzip
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from test_accuracy_workflow import AccuracyRig
from test_workflows import GCmd, workflow_module
from accel_homing_charts import _check_png
import accel_homing_troubleshoot as diagnostics


@unittest.skipUnless(importlib.util.find_spec("matplotlib"), "matplotlib unavailable")
class TroubleshootChartTests(unittest.TestCase):
    def test_real_subprocess_creates_nonempty_valid_summary_and_impact_pngs(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            def events(axis):
                observer = rig.owner.troubleshoot
                arm = float(len(rig.home_calls))
                observer.record_event("arm", axis=axis, arm_print_time_s=arm)
                observer.record_event("stop_request", axis=axis, arm_print_time_s=arm,
                                      request_before_host_monotonic_s=rig.reactor.monotonic(),
                                      request_estimated_print_time_s=arm+.003,
                                      onset_print_time_s=arm+.002,
                                      timestamp_provenance="host_stop_dispatch; local_main_mcu_clocksync_estimate")
                observer.record_batch("synthetic", axis=axis, arm_print_time_s=arm,
                                      original_sample_count=4, last_sample_age_s=.007,
                                      callback_entry_host_monotonic_s=arm+.010,
                                      callback_exit_host_monotonic_s=arm+.011)
            rig.on_home = events
            with mock.patch.object(workflow_module, "TroubleshootRecorder", diagnostics.Recorder):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="Y", SAMPLES=2, CENTERED=1))
            result = rig.workflow.last_result
            self.assertEqual(result["artifacts"]["render"]["result"], "complete")
            folder = Path(result["artifacts"]["folder"])
            pngs = list(folder.glob("*.png"))
            self.assertEqual(len(pngs), 7)
            self.assertEqual(len(list(folder.glob("*-timing.png"))), 3)
            for path in pngs:
                self.assertGreater(path.stat().st_size, 10000)
                width, height = _check_png(path.read_bytes())
                self.assertGreater(width, 500)
                self.assertGreater(height, 400)
            manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(len(manifest["render"]["files"]), 7)
            self.assertTrue((folder / "delivery.json").exists())
            self.assertTrue((folder / "delivery.csv").exists())
            self.assertTrue(all(home["delivery_diagnostic"]["status"] == "unavailable" for home in manifest["homes"]))

    def test_shadow_charts_compare_original_reference_and_frozen_score_include_early_evidence(self):
        from matplotlib.figure import Figure
        profile = {"weights_xyz": [1.5, .75, .75], "threshold_g": 1.2,
                   "raw_threshold_g": 6., "shadow_only": True}
        home = {"home_id": 1, "axis": "y", "phase": "shadow", "begin_host_monotonic_s": 20.,
                "previous_home_id": None, "record": {"onset_time_s": .1, "threshold_g": 6., "result": "impact"},
                "capture": {"baseline_g": [1., 2., 3.], "raw_rows_omitted": 0},
                "stop_timing": {}, "raw_file": "home-001-y.csv.gz", "shadow_profile": profile,
                "shadow": {"route": "weighted", "evidence_time_s": .049, "decision_time_s": .050,
                           "reference_time_s": .1, "reference_delta_ms": -50., "compatible": False}}
        report = {"homes": [home], "axes": ["y"], "ending": {"result": "complete"}, "events": [],
                  "experiment": {"mode": "SHADOW", "gc_guard_requested": True,
                                 "fit": {"shadow": {"status": "ready", "profile": profile}}}}
        snapshots = {}
        original_save = Figure.savefig
        def save(figure, path, *args, **kwargs):
            snapshots[Path(path).name] = [
                {"title": axes.get_title(), "xlim": axes.get_xlim(), "lines": {line.get_label(): list(zip(line.get_xdata(), line.get_ydata()))
                    for line in axes.lines}} for axes in figure.axes]
            return original_save(figure, path, *args, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            diagnostics.write_json(folder / "manifest.json", report)
            with gzip.open(folder / "packets.jsonl.gz", "wt", encoding="utf-8"):
                pass
            with gzip.open(folder / home["raw_file"], "wt", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("sample_index", "print_time_s", "x_g", "y_g", "z_g"))
                for index in range(71):
                    when = .04 + index * .001
                    writer.writerow((index, when, 1. + (6. if when >= .1 else 2. if when >= .049 else 0.), 2., 3.))
            with mock.patch.object(Figure, "savefig", save), mock.patch.dict(os.environ):
                diagnostics._worker(folder, "")
            for name in ("home-001-y.png", "experiment-phases.png", "summary.png"):
                self.assertGreater((folder / name).stat().st_size, 10000)
                _check_png((folder / name).read_bytes())
        impact = snapshots["home-001-y.png"][0]
        self.assertIn("OFFLINE SHADOW", impact["title"])
        self.assertIn("actual stop uses raw XYZ", impact["title"])
        lines = impact["lines"]
        self.assertIn("Frozen weighted XYZ score (offline shadow)", lines)
        raw = lines["Original per-home XYZ reference"]
        self.assertLess(raw[0][0], -50.)
        self.assertEqual(raw[0][1], 0.)  # Subtract this home's original reference baseline.
        self.assertAlmostEqual(lines["Shadow decision (offline)"][0][0], -50.)
        self.assertEqual(impact["xlim"], (-25., 100.))
        overview = snapshots["home-001-y.png"][1]
        self.assertIn("Earlier OFFLINE SHADOW", overview["title"])
        self.assertAlmostEqual(overview["lines"]["Shadow decision (offline)"][0][0], -50.)
        self.assertIn("OFFLINE SHADOW", snapshots["experiment-phases.png"][3]["title"])


if __name__ == "__main__":
    unittest.main()
