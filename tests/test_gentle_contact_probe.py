import gzip
import json
import tempfile
import unittest
import zipfile

from harness import CommandError
from test_auto_notch_workflow import AutoRig
from test_workflows import GCmd


class GentleContactProbeTests(unittest.TestCase):
    def test_one_contact_exports_trace_on_acceptance_and_rejection(self):
        for travel, accepted in ((150., True), (1., False)):
            with self.subTest(travel=travel), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory, {"gentle_homing": True})
                rig.owner.homing_accel = 1000.
                for rail in rig.owner.rails_by_axis.values():
                    rail.info.speed = 40.
                rig.home_results.append({"travel_mm": travel})
                original = rig.owner.notch_profiles["x"].copy()
                with rig.helpers():
                    if accepted:
                        rig.command(CONTACT_ONLY=1)
                    else:
                        with self.assertRaisesRegex(CommandError, "outside"):
                            rig.command(CONTACT_ONLY=1)
                result = rig.workflow.last_result
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.analysis_seen, [])
                self.assertEqual(rig.owner.notch_profiles["x"], original)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(result["result"], "contact_check_complete" if accepted else "aborted")
                with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                    report = json.loads(bundle.read("report.json"))
                    self.assertTrue(report["gentle_motion_noise"]["complete"])
                    self.assertEqual(len(report["retained_attempt_records"]), 1)
                    self.assertTrue(any("home" in item and item.endswith(".gz") for item in bundle.namelist()))
                    self.assertFalse(report["outcome"]["model_staged"])

    def test_no_trigger_keeps_contact_samples_when_native_home_exits_with_queue_busy(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory, {"gentle_homing": True})
            rig.owner.homing_accel = 1000.
            rig.owner.rails_by_axis["x"].info.speed = 40.
            stock_home = rig.stock_home
            message = "No trigger on x after full movement"

            def missed_contact(gcmd):
                stock_home(gcmd)
                rig.owner.last["x"].update(result="no_trigger", onset_g=None, impact_peak_g=None)
                rig.toolhead.busy = (20., 10., False)
                raise CommandError(message)

            rig.stock_home = missed_contact
            with rig.helpers(), self.assertRaisesRegex(CommandError, message):
                rig.command(CONTACT_ONLY=1)
            self.assertEqual(len(rig.home_calls), 1)
            self.assertEqual(rig.configfile.saved, {})
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                receipt = json.loads(bundle.read("run.json"))
                path = "raw/home-0001.json.gz"
                raw = json.loads(gzip.decompress(bundle.read(path)))
                self.assertEqual(raw["run_id"], receipt["run_id"])
                self.assertEqual(raw["phase"], "single_contact")
                self.assertEqual(raw["record"]["result"], "no_trigger")
                self.assertEqual(raw["error"]["message"], message)
                self.assertEqual(len(raw["capture"]["samples"]), 4)
                self.assertEqual([item["sample_count"] for item in receipt["captures"]
                                  if item["file"] == path], [4])
                self.assertEqual(receipt["errors"], [])

    def test_invalid_probe_options_fail_before_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory)
            with self.assertRaisesRegex(CommandError, "requires gentle_homing"):
                rig.command(CONTACT_ONLY=1)
            rig.workflow.gentle_homing = True
            with self.assertRaisesRegex(CommandError, "SAVE_CONFIG=0"):
                rig.command(CONTACT_ONLY=1, SAVE_CONFIG=1)
            with self.assertRaisesRegex(CommandError, "requires NOTCH=AUTO"):
                rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, CONTACT_ONLY=1))
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.toolhead.moves, [])
