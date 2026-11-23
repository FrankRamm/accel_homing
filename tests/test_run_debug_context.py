import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import zipfile

import accel_homing_troubleshoot as diagnostics


class RunDebugContextTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.folder = Path(directory.name)
        self.log = self.folder / "klippy.log"
        self.log.write_bytes(b"prior-run-only\n")
        self.printer = Mock()
        self.printer.get_start_args.return_value = {"log_file": str(self.log), "software_version": "test-host"}
        self.printer.get_reactor.return_value.monotonic.return_value = 123.5
        settings = {"accel_homing": {"x_threshold": 2.3}, "gcode_macro unrelated": {"gcode": "private"},
                    "beacon": {"api_token": "private"}}
        self.printer.lookup_object.return_value.get_status.return_value = {"settings": settings}
        mcu = Mock()
        mcu.get_status.return_value = {"mcu_version": "test-firmware"}
        self.printer.lookup_objects.return_value = [("mcu cartographer", mcu)]

    def export(self, cursor):
        artifacts = diagnostics.write_debug_context(self.folder, self.printer,
            {"log_cursor": cursor, "axis": "x", "phase": "compare"})
        self.assertIn("debug/context.json", artifacts["files"])
        return json.loads((self.folder / "debug" / "context.json").read_text(encoding="utf-8"))

    def test_run_log_and_relevant_config_and_versions(self):
        cursor = diagnostics.debug_log_cursor(self.printer)
        with self.log.open("ab") as stream:
            stream.write(b"command fault\n")
        context = self.export(cursor)
        self.assertEqual((self.folder / context["log"]["file"]).read_bytes(), b"command fault\n")
        self.assertEqual(context["errors"], {})
        self.assertEqual(context["host"]["software_version"], "test-host")
        self.assertEqual(context["mcus"]["mcu cartographer"]["mcu_version"], "test-firmware")
        self.assertEqual(context["effective_config_sections"], {
            "accel_homing": {"x_threshold": 2.3}, "beacon": {"api_token": "<redacted>"}})
        self.assertEqual(len(context["installed_sha256"]["accel_homing_workflows.py"]), 64)

    def test_host_and_can_snapshots_are_scoped_to_the_run(self):
        host, bus = Mock(), Mock()
        host.get_status.side_effect = [{"sysload": .4}, {"sysload": 1.2}]
        bus.get_status.side_effect = [{"tx_retries": 31}, {"tx_retries": 57}]
        self.printer.lookup_objects.return_value.extend([
            ("system_stats", host), ("canbus_stats cartographer", bus)])
        cursor = diagnostics.debug_log_cursor(self.printer)
        with patch.object(diagnostics, "_host_board_model", return_value="Raspberry Pi 3 Model B"):
            context = self.export(cursor)
        start, end = (context["host_transport"][p] for p in ("start", "end"))
        self.assertEqual([start["system_stats"]["sysload"], end["system_stats"]["sysload"]], [.4, 1.2])
        self.assertEqual([p["canbus_stats"]["canbus_stats cartographer"]["tx_retries"]
                          for p in (start, end)], [31, 57])
        self.assertEqual(context["host"]["board_model"], "Raspberry Pi 3 Model B")

    def test_rotation_and_overflow_preserve_start_and_final_exception(self):
        cursor = diagnostics.debug_log_cursor(self.printer)
        with self.log.open("ab") as stream:
            stream.write(b"begin-old-log")
        self.log.rename(self.log.with_name("klippy.log.2026-11-21"))
        self.log.with_name("klippy.log.1").write_bytes(b"unrelated older log")
        self.log.write_bytes(b"new-log-error")
        with patch.object(diagnostics, "LOG_LIMIT", 10):
            context = self.export(cursor)
        output = (self.folder / context["log"]["file"]).read_bytes()
        self.assertTrue(output.startswith(b"begin"))
        self.assertTrue(output.endswith(b"error"))
        self.assertTrue(context["log"]["gaps"])
        self.assertEqual(sum(s["bytes"] for s in context["log"]["segments"]), 10)

    def test_missing_log_preserves_failure_context(self):
        cursor = diagnostics.debug_log_cursor(self.printer)
        self.log.unlink()
        context = self.export(cursor)
        self.assertIn("klippy_log", context["errors"])
        self.assertEqual(context["run"]["phase"], "compare")

    def test_cooperative_zip_keeps_previous_archive_when_interrupted(self):
        root = self.folder / "run"
        (root / "debug").mkdir(parents=True)
        payload = (b"test binary" * 32000)
        (root / "debug" / "capture.gz").write_bytes(payload)
        archive = self.folder / "run.zip"
        archive.write_bytes(b"previous archive")
        with self.assertRaisesRegex(RuntimeError, "interrupted"):
            diagnostics.write_run_archive(root, archive, Mock(side_effect=RuntimeError("interrupted")))
        self.assertEqual(archive.read_bytes(), b"previous archive")
        self.assertEqual(list(self.folder.glob(".*.tmp")), [])
        tick = Mock()
        with patch.object(diagnostics.time, "monotonic", side_effect=range(1000)):
            diagnostics.write_run_archive(root, archive, tick, prefix="run")
        self.assertGreaterEqual(tick.call_count, 5)
        with zipfile.ZipFile(archive) as saved:
            member = saved.getinfo("run/debug/capture.gz")
            self.assertEqual(member.compress_type, zipfile.ZIP_STORED)
            self.assertEqual(saved.read(member), payload)
