"""Synthetic publication transactions; no printer, service or network access."""
import copy
import contextlib
import errno
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import accel_homing_charts as charts
from harness import CommandError
from test_accuracy_charts import RecordingRenderer, png_bytes, simulated_noise_run
from test_accuracy_recovery import RecoveryRig
from test_accuracy_workflow import AccuracyRig
from test_beta_calibration import BetaRig
from test_workflows import GCmd, Rig, workflow_module


def memory_axis(run_id, axis="x", attempt_start=1):
    item = {"requested_samples": 2, "complete": True, "trials": [], "capture_paths": [],
            "statistics": None, "rejected": None}
    for index, travel in enumerate((150., 150.001, 150.002)):
        attempt = attempt_start + index
        record = {"run_id": run_id, "axis": axis, "workflow_attempt": attempt,
                  "workflow_phase": "discovery" if index == 0 else "measurement",
                  "accuracy_sample": index, "result": "impact", "travel_mm": travel,
                  "onset_g": 2.5, "impact_peak_g": 5., "threshold_g": 2.25,
                  "workflow_threshold_g": 2.25, "direction": -1}
        capture = {"samples": [(1., .2, -.1, 1.), (1.001, 2.7, -.1, 1.),
                               (1.002, -4.8, .2, 1.)], "baseline_g": (.2, -.1, 1.)}
        prepared = charts.prepare_capture("%s_a%03d" % (run_id, attempt), axis,
                                          capture, record, simulated=True)
        if index == 0:
            item["discovery"], item["discovery_capture_paths"] = record, prepared
        else:
            item["trials"].append(record)
            item["capture_paths"].append(prepared)
    return item


class PublicationFixture(unittest.TestCase):
    def setUp(self):
        # These tests also run on Windows; production titles retain local time.
        self.clock = {"created_utc": "2026-10-29T21:40:28+00:00",
                      "created_local": "2026-10-29T18:40:28-03:00",
                      "timestamp": "2026-10-29 18:40:28", "timezone": "UTC-03:00",
                      "filename_time": "2026-10-29 18-40-28"}
        patcher = mock.patch.object(charts, "_clock", return_value=self.clock)
        patcher.start()
        self.addCleanup(patcher.stop)

    def assert_no_files(self, directory):
        self.assertEqual([path for path in Path(directory).rglob("*") if path.is_file()], [])


class DeferredCaptureTests(PublicationFixture):
    def test_prepare_and_validate_use_no_renderer_or_filesystem_scratch(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(charts, "PNGRenderer", side_effect=AssertionError("premature render")), \
                mock.patch.object(charts.tempfile, "mkstemp", side_effect=AssertionError("scratch file")), \
                mock.patch.object(charts.tempfile, "TemporaryDirectory", side_effect=AssertionError("scratch root")):
            item = memory_axis("memory")
            checked = charts.validate_accuracy(directory, "memory", {"x": item}, simulated=True)
            self.assertEqual(checked["x"]["statistics"]["count"], 2)
            self.assertFalse(item["capture_paths"][0])
            self.assertEqual(json.dumps(item["capture_paths"][0]), "{}")
            self.assert_no_files(directory)

    def test_memory_receipt_is_immutable_bounded_and_requires_current_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            item = memory_axis("provenance")
            prepared = item["capture_paths"][0]
            with self.assertRaises(AttributeError):
                prepared.receipt.axis = "y"
            with self.assertRaises(TypeError):
                prepared.receipt.trace[0][1] = 100.
            for mutation in ("record", "run", "plain", "simulated"):
                with self.subTest(mutation=mutation):
                    changed = copy.deepcopy(item)
                    if mutation == "record": changed["trials"][0]["onset_g"] += .1
                    elif mutation == "run":
                        changed["capture_paths"][0]._receipt = prepared.receipt._replace(run_id="other_a002")
                    elif mutation == "plain": changed["capture_paths"][0] = {}
                    with self.assertRaises(ValueError):
                        charts.validate_accuracy(directory, "provenance", {"x": changed},
                                                 simulated=mutation != "simulated")
            rows = [(number / 100000., (-1.) ** number, 0., 1.) for number in range(100001)]
            capture = charts.prepare_capture("bounded", "x",
                {"samples": rows, "baseline_g": (0., 0., 1.)}, {})
            self.assertLessEqual(len(capture.receipt.trace), 4096)
            rows[0] = (0., 999., 999., 999.)
            self.assertEqual(capture.receipt.trace[0], (0., 1., 0., 0.))
            self.assert_no_files(directory)

    def test_noise_preparation_keeps_png_in_memory_until_explicit_publication(self):
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory:
            iterations, recommendation = simulated_noise_run(directory, "noise")
            prepared = charts.prepare_noise("noise", iterations, recommendation, "complete",
                                             simulated=True, renderer=renderer)
            self.assert_no_files(directory)
            self.assertIsInstance(prepared.data, bytes)
            with self.assertRaises(AttributeError):
                prepared.data = b"changed"
            with mock.patch.object(renderer, "render", side_effect=AssertionError("render after save")):
                paths = charts.publish_noise(directory, prepared)
            self.assertTrue(Path(paths["png"]).is_file())
            self.assertEqual(len(list(Path(directory).iterdir())), 1)

    def test_failed_low_level_reports_do_not_render_or_publish(self):
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(charts, "_publish") as publish:
            for state in ("failed", "aborted", "incomplete", "cancelled"):
                with self.subTest(state=state):
                    for action in (
                            lambda: charts.write_accuracy(directory, "failed", {}, state, renderer=renderer),
                            lambda: charts.write_noise(directory, "failed", [], None, state, renderer=renderer),
                            lambda: charts.write_beta_motion(directory, "failed", "x", [],
                                                             result=state, renderer=renderer)):
                        with self.assertRaises(ValueError): action()
            publish.assert_not_called()
            self.assertEqual(renderer.payloads, [])
            self.assert_no_files(directory)

    def test_real_renderer_accepts_memory_capture_accuracy_and_noise(self):
        renderer = charts.PNGRenderer(timeout_s=30.)
        with tempfile.TemporaryDirectory() as directory:
            item = memory_axis("real_render")
            capture = charts.publish_capture(directory, item["discovery_capture_paths"], renderer=renderer)
            accuracy = charts.write_accuracy(directory, "real_render", {"x": item}, "complete",
                                              simulated=True, renderer=renderer)
            iterations, recommendation = simulated_noise_run(directory, "real_noise")
            noise = charts.write_noise(directory, "real_noise", iterations, recommendation, "complete",
                                       simulated=True, renderer=renderer)
            self.assertEqual(len(list(Path(directory).iterdir())), 3)
            for path in (capture["png"], accuracy["x_png"], noise["png"]):
                width, height = charts._check_png(Path(path).read_bytes())
                self.assertGreater(width, 100)
                self.assertGreater(height, 100)


class PublicationTransactionTests(PublicationFixture):
    def test_second_axis_render_failure_never_publishes_first_axis(self):
        for mode in ("accuracy", "captures"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                axes = {"x": memory_axis("xy"), "y": memory_axis("xy", "y", 4)}
                renderer = RecordingRenderer()
                original = renderer.render
                def render(payload, **kwargs):
                    if payload["axis"] == "y": raise ValueError("synthetic Y render failure")
                    return original(payload, **kwargs)
                with mock.patch.object(renderer, "render", side_effect=render), \
                        mock.patch.object(charts, "_publish", wraps=charts._publish) as publish:
                    with self.assertRaisesRegex(ValueError, "Y render failure"):
                        if mode == "accuracy":
                            charts.write_accuracy(directory, "xy", axes, "complete", simulated=True, renderer=renderer)
                        else:
                            charts.publish_captures(directory, [item["discovery_capture_paths"]
                                                               for item in axes.values()], renderer=renderer)
                    publish.assert_not_called()
                self.assert_no_files(directory)

    def test_second_publication_error_removes_only_new_owned_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            previous = root / ("Accel Homing Accuracy X - " + self.clock["filename_time"] + ".png")
            previous.write_bytes(b"existing user chart")
            axes = {"x": memory_axis("rollback"), "y": memory_axis("rollback", "y", 4)}
            link = charts.os.link
            def fail_y(stage, final):
                if "Accuracy Y" in str(final): raise OSError(errno.EIO, "synthetic publication failure")
                return link(stage, final)
            published = {}
            with mock.patch.object(charts.os, "link", side_effect=fail_y):
                with self.assertRaisesRegex(OSError, "publication failure"):
                    charts.write_accuracy(root, "rollback", axes, "complete", simulated=True,
                                          renderer=RecordingRenderer(), published_paths=published)
            self.assertEqual(published, {})
            self.assertEqual(list(root.iterdir()), [previous])
            self.assertEqual(previous.read_bytes(), b"existing user chart")

    def test_group_rollback_preserves_foreign_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            replacement = root / "foreign.png"
            replacement.write_bytes(b"another writer's file")
            axes = {"x": memory_axis("replacement"), "y": memory_axis("replacement", "y", 4)}
            link, created = charts.os.link, []
            def replace_x_then_fail_y(stage, final):
                if "Accuracy Y" in str(final):
                    os.replace(replacement, created[0])
                    raise OSError(errno.EIO, "synthetic second publication failure")
                result = link(stage, final)
                created.append(Path(final))
                return result
            with mock.patch.object(charts.os, "link", side_effect=replace_x_then_fail_y):
                with self.assertRaises(OSError):
                    charts.write_accuracy(root, "replacement", axes, "complete", simulated=True,
                                          renderer=RecordingRenderer())
            self.assertEqual(list(root.iterdir()), created)
            self.assertEqual(created[0].read_bytes(), b"another writer's file")

    def test_late_cancellation_and_stage_cleanup_failure_remove_owned_final(self):
        for failure in ("cancel", "stage_cleanup"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                linked, failed = [], []
                link, unlink = charts.os.link, charts.os.unlink
                def mark_link(stage, final):
                    result = link(stage, final)
                    linked.append(True)
                    return result
                def fail_stage_once(path, *args, **kwargs):
                    if failure == "stage_cleanup" and str(path).endswith(".tmp") and not failed:
                        failed.append(True)
                        raise OSError(errno.EIO, "synthetic staging cleanup failure")
                    return unlink(path, *args, **kwargs)
                def check():
                    if failure == "cancel" and linked: raise CommandError("synthetic late cancellation")
                with mock.patch.object(charts.os, "link", side_effect=mark_link), \
                        mock.patch.object(charts.os, "unlink", side_effect=fail_stage_once):
                    with self.assertRaises((CommandError, OSError)):
                        charts._publish(directory, "qualified", self.clock, png_bytes(), check)
                self.assert_no_files(directory)


class FailedWorkflowPublicationTests(PublicationFixture):
    def test_two_good_noise_windows_then_failure_or_cancel_publish_no_summary(self):
        for failure in ("sensor", "cancel", "disconnect"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = Rig({"charts_directory": directory})
                def interrupt(iteration):
                    if iteration != 3: return
                    if failure == "sensor": raise CommandError("synthetic hard sensor fault")
                    if failure == "cancel": rig.workflow.cmd_CANCEL(GCmd())
                    else: rig.workflow._disconnect()
                rig.on_noise = interrupt
                with mock.patch.object(workflow_module, "write_noise") as writer, \
                        mock.patch.object(workflow_module, "publish_noise") as publish, \
                        mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                    with self.assertRaises(CommandError): rig.workflow.cmd_NOISE(GCmd())
                    writer.assert_not_called()
                    publish.assert_not_called()
                    filesystem.assert_not_called()
                self.assertIsNone(rig.owner.noise_session)
                self.assertEqual(rig.workflow.noise_seeds, {})
                self.assert_no_files(directory)

    def test_complete_noise_followed_by_failed_calibration_saves_no_chart(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = Rig({"charts_directory": directory})
            rig.home_results.append(CommandError("synthetic first-home fault"))
            with mock.patch.object(workflow_module, "publish_noise") as publish, \
                    mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                with self.assertRaisesRegex(CommandError, "first-home fault"): rig.calibrate(save=0)
                publish.assert_not_called()
                filesystem.assert_not_called()
            self.assertEqual(rig.configfile.saved, {})
            self.assert_no_files(directory)

    def test_calibration_save_failure_does_not_publish_prepared_noise(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = Rig({"charts_directory": directory})
            rig.save_error = True
            with mock.patch.object(workflow_module, "publish_noise") as publish, \
                    mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                with self.assertRaisesRegex(CommandError, "SAVE_CONFIG include conflict"):
                    rig.calibrate(save=1)
                publish.assert_not_called()
                filesystem.assert_not_called()
            self.assertEqual(len(rig.home_calls), 11)
            self.assertEqual(rig.workflow.last_result["result"], "save_failed")
            self.assert_no_files(directory)

    def test_failed_accuracy_after_accepted_trial_never_writes_capture_or_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory})
            rig.home_results.extend([{}, {}, {"result": "sample_gap"}])
            with mock.patch.object(workflow_module, "write_capture") as capture_writer, \
                    mock.patch.object(workflow_module, "write_accuracy") as summary_writer, \
                    mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                with self.assertRaises(CommandError): rig.accuracy()
                capture_writer.assert_not_called()
                summary_writer.assert_not_called()
                filesystem.assert_not_called()
            self.assertEqual(len(rig.workflow.last_result["axis_results"]["x"]["trials"]), 1)
            self.assertEqual(rig.capture_directories, [])
            self.assert_no_files(directory)

    def test_early_accuracy_handoff_keeps_calibration_without_failed_accuracy_export(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = RecoveryRig({"charts_directory": directory})
            rig.owner.threshold_g["x"] = 2.25
            rig.home_results.extend([{"travel_mm": 1.175, "halt_travel_mm_signed": -1.4}] + [{}] * 11)
            with mock.patch.object(workflow_module, "write_accuracy") as summary_writer, \
                    mock.patch.object(workflow_module, "write_capture") as capture_writer:
                rig.accuracy()
                summary_writer.assert_not_called()
                capture_writer.assert_not_called()
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "calibrated")
            self.assertEqual(result["source_accuracy"]["result"], "aborted")
            self.assertEqual(result["source_accuracy"].get("summary_paths", {}), {})
            self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)
            self.assertEqual([item["threshold"] for item in rig.home_calls], [2.25] + [2.5] * 11)
            self.assertEqual(rig.capture_directories, [])
            files = list(Path(directory).iterdir())
            self.assertEqual(len(files), 1)
            self.assertTrue(files[0].name.startswith("Accel Homing Noise"))

    def test_rejected_charts_axis_or_failed_second_axis_produces_no_png(self):
        for axis, results in (("X", [{"travel_mm": 1.}]),
                              (None, [{}, {"result": "sample_gap"}])):
            with self.subTest(axis=axis), tempfile.TemporaryDirectory() as directory:
                rig = Rig({"charts_directory": directory})
                rig.home_results.extend(results)
                params = {"CENTERED": 1, **({"AXIS": axis} if axis else {})}
                with mock.patch.object(workflow_module, "write_capture") as capture_writer, \
                        mock.patch.object(workflow_module, "publish_captures") as publish, \
                        mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                    with self.assertRaises(CommandError): rig.workflow.cmd_CHARTS(GCmd(**params))
                    capture_writer.assert_not_called()
                    publish.assert_not_called()
                    filesystem.assert_not_called()
                self.assert_no_files(directory)

    def test_charts_keeps_finishing_lease_during_render_and_rejects_new_workflow(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = Rig({"charts_directory": directory})
            observed = []
            def attempt_reentry(payload):
                if payload["kind"] != "capture" or observed: return
                session = rig.workflow.session
                self.assertIsNotNone(session)
                observed.append(session)
                with self.assertRaises(CommandError):
                    rig.workflow.cmd_NOISE(GCmd())
                self.assertIs(rig.workflow.session, session)
            rig.workflow.renderer.on_render = attempt_reentry
            rig.workflow.cmd_CHARTS(GCmd(AXIS="X", CENTERED=1))
            self.assertEqual(len(observed), 1)
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.workflow.last_result["mode"], "charts")
            self.assertEqual(rig.workflow.last_result["result"], "complete")
            self.assertEqual(len(list(Path(directory).iterdir())), 1)

    def test_charts_cleanup_failure_or_render_cancellation_publishes_nothing(self):
        for failure in ("cleanup", "cancel_render"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = Rig({"charts_directory": directory})
                if failure == "cleanup":
                    rig.workflow._release_terminal_home = mock.Mock(
                        side_effect=CommandError("synthetic release failure"))
                else:
                    def cancel(payload):
                        if payload["kind"] == "capture": rig.workflow.cmd_CANCEL(GCmd())
                    rig.workflow.renderer.on_render = cancel
                with mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                    with self.assertRaises(CommandError):
                        rig.workflow.cmd_CHARTS(GCmd(AXIS="X", CENTERED=1))
                    filesystem.assert_not_called()
                self.assertNotEqual(rig.workflow.last_result["result"], "complete")
                self.assertIsNone(rig.workflow.session)
                self.assert_no_files(directory)

    def test_beta_failure_keeps_console_diagnostics_without_invoking_png_writer(self):
        for failure in ("baseline", "capture"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = BetaRig({"charts_directory": directory})
                rig.fail_at = failure
                with rig.beta_helpers(), mock.patch.object(workflow_module, "write_beta_motion") as writer, \
                        mock.patch.object(charts, "_publish", wraps=charts._publish) as filesystem:
                    with self.assertRaisesRegex(CommandError, "synthetic"):
                        rig.workflow.cmd_BETA_CALIBRATE(GCmd(AXIS="X", CENTERED=1))
                    writer.assert_not_called()
                    filesystem.assert_not_called()
                self.assertTrue(any("Beta diagnostic" in line for line in rig.output))
                self.assertIsNone(rig.workflow.last_result.get("proposed_threshold_g"))
                self.assertEqual(rig.configfile.saved, {})
                self.assert_no_files(directory)

    def test_optional_preview_error_preserves_committed_success_and_valid_png(self):
        for mode in ("noise", "accuracy", "charts", "beta"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory, \
                    contextlib.ExitStack() as stack:
                kind = AccuracyRig if mode == "accuracy" else BetaRig if mode == "beta" else Rig
                rig = kind({"charts_directory": directory})
                if mode == "beta":
                    stack.enter_context(rig.beta_helpers())
                    def publish_beta(destination, *args, **options):
                        return {"png": charts._publish(destination, "SIMULATED Beta", self.clock,
                                                       png_bytes(), options.get("check_running"))}
                    stack.enter_context(mock.patch.object(workflow_module, "write_beta_motion",
                                                          side_effect=publish_beta))
                stack.enter_context(mock.patch.object(rig.workflow, "_show_pngs",
                                                      side_effect=OSError("synthetic preview failure")))
                stack.enter_context(mock.patch.object(workflow_module.logging, "exception"))
                if mode == "noise": rig.workflow.cmd_NOISE(GCmd())
                elif mode == "accuracy": rig.accuracy()
                elif mode == "charts": rig.workflow.cmd_CHARTS(GCmd(AXIS="X", CENTERED=1))
                else: rig.workflow.cmd_BETA_CALIBRATE(GCmd(AXIS="X", CENTERED=1))
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "beta_proposed" if mode == "beta" else "complete")
                self.assertIn("preview failure", result["display_error"])
                files = list(Path(directory).iterdir())
                self.assertEqual(len(files), 1)
                self.assertEqual(charts._check_png(files[0].read_bytes()), (24, 16) if mode == "beta" else (2, 1))


if __name__ == "__main__":
    unittest.main()
