"""Real isolated PNG rendering from synthetic offline diagnostic captures."""
import importlib.util
import json
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


if __name__ == "__main__":
    unittest.main()
