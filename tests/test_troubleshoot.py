"""Offline diagnostic workflow tests. No printer, service or network access."""
import csv
import gzip
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
import zipfile

from harness import CommandError
from test_accuracy_workflow import AccuracyRig
from test_workflows import GCmd, workflow_module

import accel_homing_troubleshoot as diagnostics


class TroubleshootTests(unittest.TestCase):
    def setUp(self):
        # The synthetic native-homing package removes itself from sys.modules.
        # Use the public module instance so worker/cap patches target its globals.
        patch = mock.patch.object(workflow_module, "TroubleshootRecorder", diagnostics.Recorder)
        patch.start()
        self.addCleanup(patch.stop)

    def run_command(self, rig, axis="X", samples=2):
        with mock.patch.object(diagnostics, "render_bundle"):
            rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS=axis, SAMPLES=samples, CENTERED=1))

    def read_report(self, rig):
        return json.loads(Path(rig.workflow.last_result["artifacts"]["manifest"]).read_text(encoding="utf-8"))

    def restored(self, rig):
        self.assertIsNone(rig.workflow.session)
        self.assertIsNone(rig.owner.troubleshoot)
        self.assertFalse(rig.capture_active)
        self.assertFalse(rig.workflow.in_home)
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
        self.assertEqual(rig.owner.back_off_dist, {"x": 5.0, "y": 5.0})
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.noise_calls, [])
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])

    def test_registration_invalid_parameters_and_preflight_do_not_move(self):
        for params in ({}, {"CENTERED": 0}, {"CENTERED": 1, "AXIS": "Z"},
                       {"CENTERED": 1, "SAMPLES": 1}, {"CENTERED": 1, "SAMPLES": 51},
                       {"CENTERED": 1, "SAVE_CONFIG": 0}, {"CENTERED": 1, "SPEED": 50},
                       {"CENTERED": 1, "START_THRESHOLD": 2}):
            with self.subTest(params=params):
                rig = AccuracyRig()
                self.assertIn("ACCEL_HOMING_TROUBLESHOOT", rig.gcode.ready_gcode_handlers)
                before = rig.effects()
                with self.assertRaises(CommandError):
                    rig.workflow.cmd_TROUBLESHOOT(GCmd(**params))
                self.assertEqual(before, rig.effects())
        for change in (lambda r: setattr(r, "print_state", "printing"),
                       lambda r: setattr(r, "paused", True),
                       lambda r: setattr(r.owner.rails_by_axis["x"].info, "retract_dist", 2),
                       lambda r: r.objects.pop("homing")):
            rig = AccuracyRig()
            change(rig)
            before = rig.effects()
            with self.assertRaises(CommandError):
                self.run_command(rig)
            self.assertEqual(before, rig.effects())

    def test_same_motion_settings_and_sequence_as_accuracy(self):
        with tempfile.TemporaryDirectory() as directory:
            diagnostic = AccuracyRig({"charts_directory": directory})
            reference = AccuracyRig({"charts_directory": directory})
            # Summary publication is unrelated to motion and uses Linux filenames.
            with mock.patch.object(workflow_module, "write_accuracy"):
                reference.accuracy(axis="XY", samples=2)
            self.run_command(diagnostic, axis="XY")
            self.assertEqual(diagnostic.home_calls, reference.home_calls)
            self.assertEqual(diagnostic.toolhead.moves, reference.toolhead.moves)
            self.assertEqual(diagnostic.toolhead.dwells, reference.toolhead.dwells)
            self.assertEqual(diagnostic.toolhead.accel_log, reference.toolhead.accel_log)
            self.assertEqual(diagnostic.scripts, reference.scripts)
            self.assertEqual(diagnostic.owner.threshold_g, reference.owner.threshold_g)
            self.assertEqual(diagnostic.workflow.last_result["completed_samples"], {"x": 2, "y": 2})
            report = self.read_report(diagnostic)
            self.assertEqual(len(report["homes"]), 6)
            self.assertEqual([h["phase"] for h in report["homes"]], ["discovery", "measurement", "measurement"] * 2)
            self.assertEqual(report["homes"][2]["previous_home_id"], 2)
            self.assertTrue(report["homes"][2]["previous_return_completed"])
            self.assertEqual(report["homes"][0]["return_planned"]["restored_velocity_and_acceleration"], [500.0, 50000.0])
            self.restored(diagnostic)

    def test_default_50_samples_each_axis_excludes_discovery(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            with mock.patch.object(diagnostics, "render_bundle"):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(CENTERED=1))
            self.assertEqual(len(rig.home_calls), 102)
            self.assertEqual(rig.workflow.last_result["completed_samples"], {"x": 50, "y": 50})
            self.assertEqual(rig.workflow.last_result["statistics"]["x"]["count"], 50)
            self.assertEqual(len(self.read_report(rig)["homes"]), 102)
            self.restored(rig)

    def test_early_home_preserves_raw_and_stops_without_return_or_recalibration(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.home_results.append({"travel_mm": 50.0, "halt_travel_mm_signed": -52.0})
            with self.assertRaisesRegex(CommandError, "without automatic return"):
                self.run_command(rig)
            self.assertEqual(len(rig.home_calls), 1)
            self.assertEqual(rig.toolhead.moves, [])
            home = self.read_report(rig)["homes"][0]
            self.assertEqual(home["quality"], "early")
            self.assertEqual(home["capture"]["raw_rows_retained"], 4)
            self.assertEqual(home["record"]["travel_mm"], 50.0)
            self.restored(rig)

    def test_validation_failure_preserves_all_raw_rows_and_detector(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.capture_mutator = lambda capture: capture["detector"].update(onset_sample_matches=0)
            with self.assertRaisesRegex(CommandError, "measured impact"):
                self.run_command(rig)
            report = self.read_report(rig)
            home = report["homes"][0]
            self.assertFalse(home["record"]["capture_valid"])
            self.assertEqual(home["capture"]["detector"]["onset_sample_matches"], 0)
            folder = Path(rig.workflow.last_result["artifacts"]["folder"])
            with gzip.open(folder / home["raw_file"], "rt") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 4)
            self.assertEqual(float(rows[2]["y_g"]), -5.0)
            self.restored(rig)

    def test_native_fault_and_cancel_preserve_partial_evidence_after_capture_cleanup(self):
        for cancel in (False, True):
            with self.subTest(cancel=cancel), tempfile.TemporaryDirectory() as directory:
                rig = AccuracyRig({"charts_directory": directory})
                if cancel:
                    rig.on_home = lambda axis: rig.workflow.cmd_CANCEL(GCmd())
                else:
                    rig.home_results.append(CommandError("synthetic fault"))
                def export_after_cleanup(folder, renderer, reactor, check_running):
                    self.assertFalse(rig.capture_active)
                    self.assertFalse(rig.workflow.in_home)
                    self.assertIsNone(rig.owner.troubleshoot)
                    self.assertTrue(Path(folder, "manifest.json").exists())
                    self.assertEqual(rig.owner.back_off_dist, {"x": 5.0, "y": 5.0})
                with mock.patch.object(diagnostics, "render_bundle", side_effect=export_after_cleanup):
                    with self.assertRaises(CommandError):
                        rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="X", SAMPLES=2, CENTERED=1))
                self.assertEqual(len(self.read_report(rig)["homes"]), 1)
                self.assertEqual(rig.workflow.last_result["result"], "cancelled" if cancel else "aborted")
                self.restored(rig)

    def test_finite_json_and_explicit_raw_cap_omissions(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            with mock.patch.object(diagnostics, "MAX_RAW_ROWS", 5), mock.patch.object(diagnostics, "MAX_HOME_ROWS", 3):
                self.run_command(rig)
            report = self.read_report(rig)
            self.assertEqual([home["capture"]["raw_rows_retained"] for home in report["homes"]], [2, 2, 1])
            self.assertEqual([home["capture"]["raw_rows_omitted"] for home in report["homes"]], [2, 2, 3])
            folder = Path(rig.workflow.last_result["artifacts"]["folder"])
            for home in report["homes"]:
                with gzip.open(folder / home["raw_file"], "rt") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(rows[-1]["sample_index"], "3")
            json.dumps(report, allow_nan=False)
            self.assertEqual(diagnostics.clean({"bad": math.nan}), {"bad": None})
            self.restored(rig)

    def test_packet_caps_do_not_consume_critical_events(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            def packets(axis):
                observer = rig.owner.troubleshoot
                now = rig.reactor.monotonic()
                for index in range(10):
                    observer.record_batch("simulated", original_sample_count=index,
                                          callback_entry_host_monotonic_s=now, callback_exit_host_monotonic_s=now+.01)
                observer.record_event("stop_request", request_estimated_print_time_s=1.002)
            rig.on_home = packets
            with mock.patch.object(diagnostics, "MAX_HOME_BATCHES", 2), mock.patch.object(diagnostics, "INITIAL_HOME_BATCHES", 1):
                self.run_command(rig)
            report = self.read_report(rig)
            self.assertEqual(report["bounds"]["batches_retained"], 6)
            self.assertEqual(report["bounds"]["batches_omitted"], 24)
            self.assertEqual(report["homes"][0]["packet_retention"]["retained_sequence_ranges"], [[1, 1], [10, 10]])
            self.assertEqual(sum(event["kind"] == "stop_request" for event in report["events"]), 3)

    def test_renderer_failure_retains_json_csv_zip_and_status_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            with mock.patch.object(diagnostics, "render_bundle", side_effect=RuntimeError("missing plotting dependencies")):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="Y", SAMPLES=2, CENTERED=1))
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertEqual(result["artifacts"]["render"]["result"], "failed")
            self.assertEqual(rig.workflow.get_status(0)["last_result"]["artifacts"], result["artifacts"])
            with zipfile.ZipFile(result["artifacts"]["archive"]) as zipped:
                self.assertIsNone(zipped.testzip())
                self.assertTrue(any(name.endswith("manifest.json") for name in zipped.namelist()))
                self.assertEqual(sum(name.endswith(".csv.gz") for name in zipped.namelist()), 3)
            self.restored(rig)

    def test_raw_export_failure_detaches_observer_and_restores_state(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            with mock.patch.object(diagnostics.Recorder, "export", side_effect=OSError("disk full")):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="X", SAMPLES=2, CENTERED=1))
            self.assertIn("disk full", rig.workflow.last_result["export_error"])
            self.restored(rig)

    def test_installed_source_fingerprints_and_tailored_config(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.configfile.get_status = lambda now: {"settings": {
                "printer": {"max_accel": 50000}, "stepper_x": {"position_endstop": 0},
                "accel_homing": {"secret": "redact", "x_threshold": 8},
                "gcode_macro SECRET": {"gcode": "do not export"}}}
            self.run_command(rig)
            report = self.read_report(rig)
            sections = report["environment"]["effective_config_sections"]
            self.assertNotIn("gcode_macro SECRET", sections)
            self.assertEqual(sections["accel_homing"]["secret"], "<redacted>")
            identities = report["installed_identity"]["source_files"]
            self.assertTrue(any(Path(row["path"]).name == "accel_homing_troubleshoot.py" and len(row["sha256"]) == 64 for row in identities))

    def test_late_packet_uses_original_arm_and_closed_recorder_is_frozen(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            observed = []
            def packets(axis):
                observer = rig.owner.troubleshoot
                observed[:] = [observer]
                number = len(rig.home_calls)
                observer.record_event("arm", axis=axis, arm_print_time_s=float(number))
                if number == 2:
                    observer.record_batch("late_native", axis=axis, arm_print_time_s=1., original_sample_count=9)
                    observer.record_event("sensor_fault", axis=axis, arm_print_time_s=1., fault="late original observer")
            rig.on_home = packets
            def check_closed(folder, renderer, reactor, check_running):
                observer = observed[0]
                self.assertTrue(observer.closed)
                before = len(observer.events)
                observer.record_event("sensor_fault", axis="x", arm_print_time_s=1., fault="arrived after close")
                self.assertEqual(len(observer.events), before)
            with mock.patch.object(diagnostics, "render_bundle", side_effect=check_closed):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="X", SAMPLES=2, CENTERED=1))
            report = self.read_report(rig)
            self.assertEqual(report["homes"][0]["batches_retained"], 1)
            self.assertEqual(report["homes"][1]["batches_retained"], 0)
            faults = [event for event in report["events"] if event["kind"] == "sensor_fault"]
            self.assertEqual([event["home_id"] for event in faults], [1])

    def test_stop_delay_uses_same_domain_matched_onset_and_invalid_remains_null(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            def events(axis):
                observer = rig.owner.troubleshoot
                number = len(rig.home_calls)
                stamp = [number, 2, 1, 3]
                observer.record_event("arm", axis=axis, arm_print_time_s=float(number))
                observer.record_event("threshold_crossing", axis=axis, arm_print_time_s=float(number),
                                      onset_print_time_s=number + .00123, onset_sample=stamp,
                                      sample_time_provenance="configured_full_precision_sample_times")
                observer.record_event("stop_request", axis=axis, arm_print_time_s=float(number),
                                      onset_print_time_s=number + .001, onset_sample=stamp,
                                      request_estimated_print_time_s=number + (.02 if number < 3 else -1),
                                      timestamp_provenance="host_stop_dispatch; local_main_mcu_clocksync_estimate")
            rig.on_home = events
            self.run_command(rig)
            report = self.read_report(rig)
            self.assertAlmostEqual(report["homes"][0]["stop_timing"]["onset_to_host_dispatch_estimate_s"], .01877)
            self.assertIsNone(report["homes"][2]["stop_timing"]["onset_to_host_dispatch_estimate_s"])
            self.assertIn("reversed", report["homes"][2]["stop_timing"]["reason"])

    def test_cancel_during_renderer_yield_kills_child_preserves_evidence_and_ending(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            process = mock.Mock()
            process.poll.return_value = None
            rig.reactor.on_pause = lambda: rig.workflow.cmd_CANCEL(GCmd())
            with mock.patch.object(diagnostics.platform, "platform", return_value="simulated-platform"), \
                    mock.patch.object(diagnostics.subprocess, "Popen", return_value=process):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="X", SAMPLES=2, CENTERED=1))
            process.kill.assert_called_once()
            process.wait.assert_called_once()
            self.assertEqual(rig.workflow.last_result["result"], "cancelled")
            report = self.read_report(rig)
            self.assertEqual(report["ending"]["result"], "cancelled")
            self.assertEqual(report["render"]["result"], "failed")
            self.assertTrue(Path(rig.workflow.last_result["artifacts"]["archive"]).is_file())
            self.restored(rig)


if __name__ == "__main__":
    unittest.main()
