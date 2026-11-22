"""One experiment bundle retains run logs on success, failure and idle rejection."""
import json
import contextlib
import itertools
from pathlib import Path
import tempfile
import time
import types
import unittest
from unittest import mock
import zipfile

from harness import CommandError
from test_experiment_mapped_compare import MappedCompareRig
from test_workflows import GCmd, workflow_module
import accel_homing_experiment as model
import accel_homing_troubleshoot as diagnostics


class ExperimentDebugBundleTests(unittest.TestCase):
    def setUp(self):
        for name, value in (("TroubleshootRecorder", diagnostics.Recorder),
                            ("_load_troubleshoot_module", lambda: diagnostics),
                            ("_load_experiment_module", lambda: model)):
            patch = mock.patch.object(workflow_module, name, value)
            patch.start()
            self.addCleanup(patch.stop)
        patch = mock.patch.object(diagnostics, "render_bundle")
        patch.start()
        self.addCleanup(patch.stop)

    def fixture(self, directory):
        rig = MappedCompareRig(directory, filtered=True)
        logfile = Path(directory) / "klippy.log"
        logfile.write_text("OLD RUN MUST NOT BE REPORTED\n", encoding="utf-8")
        args = {"log_file": str(logfile), "software_version": "synthetic-debug-test"}
        rig.get_start_args = lambda: dict(args)
        rig.workflow.chart_start_args = dict(args)
        return rig, logfile

    def append(self, logfile, text):
        with logfile.open("a", encoding="utf-8") as stream:
            stream.write(text + "\n")

    def run_compare(self, rig, **parameters):
        rig.workflow.cmd_EXPERIMENT(GCmd(**{"AXIS": "X", "CENTERED": 1,
            "MODE": "COMPARE", "SAMPLES": 2, **parameters}))

    def bundle(self, rig):
        paths = rig.workflow.last_result["artifacts"]
        with zipfile.ZipFile(paths["archive"]) as archive:
            self.assertIsNone(archive.testzip())
            files = {name.split("/", 1)[1]: archive.read(name) for name in archive.namelist()}
        return files

    def test_success_and_partial_failure_include_debug_and_existing_timing_evidence(self):
        for failure in (False, True):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig, logfile = self.fixture(directory)
                rig.on_home = lambda axis: self.append(logfile, "CURRENT HOME DEBUG")
                if failure:
                    rig.capture_change = lambda capture: capture["detector"]["experimental_envelope"].update(
                        max_oldest_eligible_age_s=.2)
                    with mock.patch.object(diagnostics, "render_bundle", side_effect=ValueError("PNG FAILED")):
                        with self.assertRaisesRegex(CommandError, "age envelope"):
                            self.run_compare(rig)
                else:
                    self.run_compare(rig)
                files = self.bundle(rig)
                for name in ("debug/context.json", "manifest.json", "experiment.json",
                             "timing.jsonl.gz", "packets.jsonl.gz", "delivery.json", "comparison.json"):
                    self.assertIn(name, files)
                context = json.loads(files["debug/context.json"])
                logged = files[context["log"]["file"]].decode("utf-8")
                self.assertIn("CURRENT HOME DEBUG", logged)
                self.assertNotIn("OLD RUN", logged)
                self.assertEqual(context["run"]["ending"]["result"], "aborted" if failure else "complete")
                if failure:
                    manifest = json.loads(files["manifest.json"])
                    self.assertIn("Traceback", manifest["experiment"]["failure_exception"]["traceback"])
                    self.assertIn("PNG FAILED", manifest["render"]["exception"]["traceback"])
                self.assertEqual(sum(name.endswith(".csv.gz") for name in files), 1 if failure else 4)
                self.assertTrue(any("shareable ZIP" in message for message in rig.output))

    def test_idle_preflight_bundle_is_new_and_never_repackages_previous_run(self):
        with tempfile.TemporaryDirectory() as directory:
            rig, logfile = self.fixture(directory)
            rig.on_home = lambda axis: self.append(logfile, "PREVIOUS COMPLETE HOME")
            self.run_compare(rig)
            old = rig.workflow.last_result
            old_archive = Path(old["artifacts"]["archive"])
            contents = old_archive.read_bytes()
            def reject(command):
                self.append(logfile, "CURRENT PREFLIGHT FAILURE")
                raise CommandError("preflight rejected")
            with mock.patch.object(rig.workflow, "_cmd_EXPERIMENT", side_effect=reject):
                with self.assertRaisesRegex(CommandError, "preflight rejected"):
                    self.run_compare(rig)
            self.assertNotEqual(rig.workflow.last_result["run_id"], old["run_id"])
            self.assertEqual(old_archive.read_bytes(), contents)
            files = self.bundle(rig)
            self.assertFalse(any(name.startswith("home-") for name in files))
            context = json.loads(files["debug/context.json"])
            self.assertEqual(context["run"]["phase"], "preflight")
            self.assertEqual(context["run"]["arguments"]["SAMPLES"], 2)
            self.assertIn("Traceback", context["run"]["exception"]["traceback"])
            logged = files[context["log"]["file"]].decode("utf-8")
            self.assertIn("CURRENT PREFLIGHT FAILURE", logged)
            self.assertNotIn("PREVIOUS COMPLETE HOME", logged)

    def test_active_workflow_print_or_motion_skips_file_and_log_work(self):
        for state in ("workflow", "homing", "attempt", "print", "queued"):
            with self.subTest(state=state), tempfile.TemporaryDirectory() as directory:
                rig, _ = self.fixture(directory)
                previous = {"result": "previous result"}
                rig.workflow.last_result = previous
                if state == "workflow":
                    rig.workflow.session = {"mode": "accuracy", "run_id": "existing"}
                elif state == "homing":
                    rig.owner.homing_active = True
                elif state == "attempt":
                    rig.owner.attempt = object()
                elif state == "print":
                    rig.print_state = "printing"
                else:
                    rig.toolhead.busy = (10., 0., False)
                with mock.patch.object(diagnostics, "debug_log_cursor") as cursor, \
                        mock.patch.object(diagnostics, "export_experiment_failure") as export:
                    with self.assertRaises(CommandError):
                        self.run_compare(rig, AXIS="Z")
                cursor.assert_not_called()
                export.assert_not_called()
                self.assertIs(rig.workflow.last_result, previous)
                self.assertFalse((Path(directory) / "experiment").exists())
                self.assertEqual(rig.home_calls, [])

    def test_debug_unavailable_preserves_original_error_and_exports_source_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            rig, logfile = self.fixture(directory)
            logfile.unlink()
            with self.assertRaisesRegex(CommandError, "requires AXIS"):
                self.run_compare(rig, AXIS="Z")
            files = self.bundle(rig)
            context = json.loads(files["debug/context.json"])
            self.assertIn("unavailable", context["log"])
            self.assertIn("unavailable", context["started"])
            self.assertIn("requires AXIS", context["run"]["reason"])

    def test_partial_export_failure_is_packaged_with_written_captures_and_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            rig, logfile = self.fixture(directory)
            rig.on_home = lambda axis: self.append(logfile, "RUN BEFORE EXPORT FAILURE")
            with mock.patch.object(diagnostics, "export_delivery", side_effect=RuntimeError("DELIVERY EXPORT FAILED")):
                self.run_compare(rig)
            files = self.bundle(rig)
            self.assertEqual(sum(name.endswith(".csv.gz") for name in files), 4)
            self.assertIn("debug/context.json", files)
            context = json.loads(files["debug/context.json"])
            self.assertEqual(context["run"]["phase"], "export_failure")
            self.assertIn("DELIVERY EXPORT FAILED", context["run"]["exception"]["traceback"])
            self.assertEqual(len(list((Path(directory) / "experiment").glob("*.zip"))), 1)

    def test_handled_cancel_during_homing_or_render_keeps_single_failure_zip(self):
        for phase in ("homing", "render"):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as directory:
                rig, logfile = self.fixture(directory)
                def cancel():
                    self.append(logfile, "CANCELLED RUN DEBUG")
                    rig.workflow.cmd_CANCEL(GCmd())
                if phase == "homing":
                    rig.on_home = lambda axis: cancel() if len(rig.home_calls) == 2 else None
                    with self.assertRaises(CommandError):
                        self.run_compare(rig)
                else:
                    def render(folder, renderer, reactor, check):
                        cancel()
                        check()
                    with mock.patch.object(diagnostics, "render_bundle", side_effect=render):
                        self.run_compare(rig)
                files = self.bundle(rig)
                manifest = json.loads(files["manifest.json"])
                self.assertEqual(manifest["ending"]["result"], "cancelled")
                self.assertIn("debug/context.json", files)
                self.assertEqual(sum(name.endswith(".csv.gz") for name in files), 2 if phase == "homing" else 4)
                self.assertEqual(len(list((Path(directory) / "experiment").glob("*.zip"))), 1)

    def test_raw_capture_serialization_yields_and_retains_handled_cancel_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            rig, _ = self.fixture(directory)
            ticks = itertools.count(100., .030)
            clock = types.SimpleNamespace(monotonic=lambda: next(ticks),
                perf_counter=time.perf_counter, process_time=time.process_time, sleep=time.sleep)
            raw_open, raw_pauses = [False], []
            original_open, original_pause = diagnostics.gzip.open, rig.reactor.pause
            @contextlib.contextmanager
            def track_open(filename, *args, **kwargs):
                with original_open(filename, *args, **kwargs) as stream:
                    raw_open[0] = Path(filename).name.startswith("home-")
                    try:
                        yield stream
                    finally:
                        raw_open[0] = False
            def pause(when):
                if raw_open[0]:
                    self.assertFalse(rig.workflow.in_home)
                    self.assertTrue(rig.workflow.session["finishing"])
                    raw_pauses.append(when)
                    if len(raw_pauses) == 1:
                        rig.workflow.cmd_CANCEL(GCmd())
                return original_pause(when)
            with mock.patch.object(diagnostics, "time", clock), \
                    mock.patch.object(diagnostics.gzip, "open", side_effect=track_open), \
                    mock.patch.object(rig.reactor, "pause", side_effect=pause):
                self.run_compare(rig)
            self.assertGreaterEqual(len(raw_pauses), 4)
            files = self.bundle(rig)
            self.assertEqual(json.loads(files["manifest.json"])["ending"]["result"], "cancelled")
            self.assertEqual(sum(name.endswith(".csv.gz") for name in files), 4)
            self.assertIn("debug/context.json", files)


if __name__ == "__main__":
    unittest.main()
