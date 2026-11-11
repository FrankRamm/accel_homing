"""Offline save/rollback matrix against pinned native Klipper and Kalico APIs.

All motion and sensor operations are stubs; configuration files are temporary.
The upstream parsing, include-conflict, save and reload methods run unchanged.
"""
import configparser
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from harness import CommandError
from test_center_calibration import CenterRig
from test_travel_calibration import TravelRig
from native_configfile_fixtures import KlipperConfig, KalicoConfig


class ConfigPersistenceTests(unittest.TestCase):
    config_class = KlipperConfig

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "axes").mkdir()
        (self.root / "printer.cfg").write_text("# keep main comment\n[include axes/*.cfg]\n")
        (self.root / "axes/10-x.cfg").write_bytes(
            b"# keep include comment\r\n[stepper_x]\r\nposition_min: 0\r\nposition_max: 300\r\n"
            b"position_endstop: 0\r\nhoming_speed: 100\r\n[stepper_x]\r\nposition_max: 301\r\n"
            b"[include ../limits.cfg]\r\n")
        (self.root / "limits.cfg").write_text("[stepper_x]\nposition_max: 300\n[gcode_macro KEEP]\ngcode:\n  G4 P1\n")

    def rig(self, travel=True):
        rig = TravelRig() if travel else CenterRig()
        rig.get_start_args = lambda: {"config_file": str(self.root / "printer.cfg")}
        self.restarts = []
        rig.gcode.request_restart = self.restarts.append
        rig.gcode.error = CommandError
        config = self.config_class(rig)
        if hasattr(config, "_build_config_wrapper"):
            config.autosave = config._build_config_wrapper("", rig.get_start_args()["config_file"])
        rig.configfile = config
        rig.objects["configfile"] = config
        return rig

    def prepare(self, rig):
        rig.measure()
        rig.origin()

    def files(self):
        return {path: path.read_bytes() for path in self.root.rglob("*.cfg")}

    def test_round_trip_migrates_nested_glob_split_duplicate_options_only_after_opt_in(self):
        for travel in (True, False):
            with self.subTest(travel=travel):
                # Restore every original input for the second workflow.
                original = self.files()
                rig = self.rig(travel)
                if travel:
                    self.prepare(rig)
                    self.assertEqual(self.files(), original)
                    rig.command("SAVE", CONFIRM=1)
                else:
                    rig.center(AXIS="X")
                    self.assertEqual(self.files(), original)
                    rig = self.rig(travel)
                    rig.center(AXIS="X", SAVE_CONFIG=1)
                result = rig.workflow.last_result
                config = rig.configfile.read_main_config().fileconfig
                expected = result["proposed_config"]["stepper_x"]
                for option in ("position_min", "position_max", "position_endstop"):
                    self.assertAlmostEqual(config.getfloat("stepper_x", option), expected[option])
                self.assertEqual(config.getfloat("stepper_x", "homing_speed"), 100.)
                self.assertEqual(config.get("gcode_macro KEEP", "gcode").strip(), "G4 P1")
                self.assertEqual(self.restarts, ["restart"])
                backup = Path(result["config_backup_directory"])
                self.assertTrue((backup / "manifest.json").is_file())
                self.assertEqual((backup / "000.bak").read_bytes(), original[self.root / "printer.cfg"])
                included = (self.root / "axes/10-x.cfg").read_bytes()
                self.assertIn(b"# keep include comment\r\n", included)
                self.assertIn(b"#position_max: 300\r\n", included)
                self.assertIn(b"#position_max: 301\r\n", included)
                for path, data in original.items():
                    path.write_bytes(data)

    def test_save_failure_matrix_restores_inputs_and_pending(self):
        for failure in ("stage", "include_write", "before_main_write", "after_main_write"):
            with self.subTest(failure=failure):
                rig = self.rig()
                self.prepare(rig)
                original = self.files()
                center_globals = rig.workflow.center._save.__globals__
                real_write = center_globals["_atomic_config_write"]
                save_owner = rig.configfile if hasattr(rig.configfile, "cmd_SAVE_CONFIG") else rig.configfile.autosave

                def fail_write(path, data, mode):
                    if path == str(self.root / "limits.cfg"):
                        raise OSError("synthetic include write failure")
                    return real_write(path, data, mode)

                if failure == "stage":
                    patch = mock.patch.object(rig.configfile, "set", side_effect=CommandError("synthetic stage failure"))
                elif failure == "include_write":
                    patch = mock.patch.dict(center_globals, {"_atomic_config_write": fail_write})
                elif failure == "before_main_write":
                    patch = mock.patch.object(type(save_owner), "cmd_SAVE_CONFIG", side_effect=CommandError("synthetic native failure"))
                else:
                    patch = mock.patch.object(rig.gcode, "request_restart", side_effect=CommandError("synthetic restart failure"))
                with patch, self.assertRaises((CommandError, OSError)):
                    rig.command("SAVE", CONFIRM=1)
                for path, data in original.items():
                    self.assertEqual(path.read_bytes(), data, str(path))
                self.assertFalse(save_owner.save_config_pending)
                self.assertEqual(save_owner.status_save_pending, {})
                self.assertFalse(rig.workflow.last_result["save_config"])

    def test_comment_only_include_edit_pending_settings_and_cycles_prevent_save(self):
        for change in ("comment", "pending", "cycle", "glob"):
            with self.subTest(change=change):
                rig = self.rig()
                self.prepare(rig)
                path = self.root / "limits.cfg"
                source = path.read_text()
                if change == "pending":
                    rig.configfile.set("extruder", "pid_kp", "42")
                elif change == "glob":
                    (self.root / "axes/20-added.cfg").write_text("[unrelated]\nvalue: 1\n")
                else:
                    path.write_text(source + ("# edited\n" if change == "comment" else "[include printer.cfg]\n"))
                original = self.files()
                with self.assertRaises((CommandError, configparser.Error)):
                    rig.command("SAVE", CONFIRM=1)
                self.assertEqual(self.files(), original)
                self.assertEqual(self.restarts, [])
                if change == "pending":
                    self.assertEqual(rig.configfile.autosave.fileconfig.get("extruder", "pid_kp"), "42")
                path.write_text(source)
                if change == "glob":
                    (self.root / "axes/20-added.cfg").unlink()

    def test_include_symlink_preserved_and_retarget_rejected(self):
        include = self.root / "axes/10-x.cfg"
        target = self.root / "axes/original.txt"
        original = include.read_bytes()
        target.write_bytes(original)
        include.unlink()
        try:
            include.symlink_to(target)
        except OSError:
            self.skipTest("creating symlinks is unavailable")
        rig = self.rig()
        self.prepare(rig)
        rig.command("SAVE", CONFIRM=1)
        self.assertTrue(include.is_symlink())
        self.assertIn(b"#position_max: 300", target.read_bytes())
        # A different target with identical source text is still a config change.
        (self.root / "printer.cfg").write_text("[include axes/*.cfg]\n")
        target.write_bytes(original)
        rig = self.rig()
        self.prepare(rig)
        replacement = self.root / "axes/replacement.txt"
        replacement.write_bytes(original)
        include.unlink()
        include.symlink_to(replacement)
        with self.assertRaises(CommandError):
            rig.command("SAVE", CONFIRM=1)
        self.assertEqual(replacement.read_bytes(), original)
        self.assertEqual(self.restarts, [])

    def test_external_main_edit_is_retained_on_native_failure(self):
        rig = self.rig()
        self.prepare(rig)
        main = self.root / "printer.cfg"
        edited = main.read_text() + "# external writer\n"
        save_owner = rig.configfile if hasattr(rig.configfile, "cmd_SAVE_CONFIG") else rig.configfile.autosave

        def external_edit(*args):
            main.write_text(edited)
            raise CommandError("synthetic concurrent edit")

        with mock.patch.object(type(save_owner), "cmd_SAVE_CONFIG", external_edit), self.assertRaises(CommandError):
            rig.command("SAVE", CONFIRM=1)
        self.assertEqual(main.read_text(), edited)
        self.assertIn("unconfirmed", rig.workflow.last_result["persistence"])
        self.assertTrue(rig.workflow.last_result["config_rollback_errors"])

    def test_external_include_removal_is_retained_on_native_failure(self):
        rig = self.rig()
        self.prepare(rig)
        include = self.root / "limits.cfg"
        save_owner = rig.configfile if hasattr(rig.configfile, "cmd_SAVE_CONFIG") else rig.configfile.autosave

        def remove_include(*args):
            include.unlink()
            raise CommandError("synthetic concurrent removal")

        with mock.patch.object(type(save_owner), "cmd_SAVE_CONFIG", remove_include), self.assertRaises(CommandError):
            rig.command("SAVE", CONFIRM=1)
        self.assertFalse(include.exists())
        self.assertIn("unconfirmed", rig.workflow.last_result["persistence"])
        self.assertTrue(rig.workflow.last_result["config_rollback_errors"])


class KalicoConfigPersistenceTests(ConfigPersistenceTests):
    config_class = KalicoConfig


if __name__ == "__main__":
    unittest.main()
