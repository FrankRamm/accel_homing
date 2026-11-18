"""Disposable configuration trees and public SAVE_CONFIG chaining only."""
import configparser
import glob
import io
import os
from pathlib import Path
import re
import tempfile
import types
import unittest
from unittest import mock

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import accel_homing_config as persistence
from accel_homing_config import CalibrationConfigSave


HEADER = "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n"


class SaveError(Exception):
    pass


class NativeShape:
    def __init__(self, root, modern=False):
        self.root = root
        self.autosave = types.SimpleNamespace(fileconfig=configparser.RawConfigParser())
        self.state = self.autosave if modern else self
        self.state.status_save_pending, self.state.save_config_pending = {}, False
        self.before, self.after = None, None
        self.calls = 0

    def set(self, section, option, value):
        parser = self.autosave.fileconfig
        if not parser.has_section(section):
            parser.add_section(section)
        parser.set(section, option, str(value))
        self.state.status_save_pending.setdefault(section, {})[option] = str(value)
        self.state.save_config_pending = True

    @staticmethod
    def _find_autosave_data(text):
        if HEADER not in text:
            return text, ""
        regular, generated = text.split(HEADER, 1)
        return regular, "\n".join(line[4:] for line in generated.split("\n") if line.startswith("#*# "))

    @staticmethod
    def _resolve_include(filename, specification, parser, visited):
        pattern = os.path.join(os.path.dirname(filename), specification)
        names = sorted(glob.glob(pattern))
        if not names and not glob.has_magic(pattern):
            raise SaveError("missing include")
        return names

    @staticmethod
    def _strip_duplicates(data, config):
        # Native retained host behavior deliberately ignores indented keys.
        section = None
        lines = data.split("\n")
        for index, line in enumerate(lines):
            pruned = re.sub(r"[#;].*$", "", line).rstrip()
            if not pruned or pruned[0].isspace():
                continue
            if pruned.startswith("["):
                section = pruned[1:-1].strip()
            elif config.fileconfig.has_option(section, re.sub(r"[^A-Za-z0-9_].*$", "", pruned)):
                lines[index] = "#" + line
        return "\n".join(lines)

    def save(self, command):
        self.calls += 1
        if self.before:
            self.before()
        parser = self.autosave.fileconfig
        regular, _old = self._find_autosave_data(self.root.read_text())
        # Minimal serializer fixture; separate retained-native smoke exercises
        # actual Klipper/Kalico implementations rather than this test helper.
        keys = {(section, option) for section in parser.sections() for option in parser.options(section)}
        regular = persistence._comment_selected(regular.encode(), keys).decode()
        buffer = io.StringIO()
        parser.write(buffer)
        generated = "\n".join("#*# " + line for line in buffer.getvalue().split("\n"))
        self.root.write_text(regular.rstrip() + HEADER + generated)
        if self.after:
            self.after()
        return "native result"


class PublicGCode:
    def __init__(self, native):
        self.handlers = {"SAVE_CONFIG": native}
        self.fail_install = False

    def register_command(self, name, callback):
        if callback is None:
            return self.handlers.pop(name, None)
        if self.fail_install:
            self.fail_install = False
            raise SaveError("installation failed")
        if name in self.handlers:
            raise SaveError("already registered")
        self.handlers[name] = callback


class Rig:
    command_error = SaveError

    def __init__(self, directory, modern=False):
        self.root = Path(directory) / "printer.cfg"
        self.include = Path(directory) / "plugin.cfg"
        self.root.write_bytes(b"[include plugin.cfg]\n[printer]\nkinematics: corexy\n")
        self.include.write_bytes(b"# retained comment\r\n[accel_homing]\r\nx_threshold: 8\r\n"
                                 b"y_threshold: 7\r\nthreshold: 8\r\nhoming_accel: 5000\r\n"
                                 b"[other]\r\nx_threshold: 42\r\n")
        self.config = NativeShape(self.root, modern)
        self.gcode = PublicGCode(self.config.save)
        self.helper = CalibrationConfigSave(self, "accel_homing")

    def lookup_object(self, name):
        return {"configfile": self.config, "gcode": self.gcode}[name]

    def get_start_args(self):
        return {"config_file": str(self.root)}

    def stage(self, axis="x", value="2.75"):
        rollback = self.helper.register_threshold(axis, value)
        self.config.set("accel_homing", axis + "_threshold", value)
        self.config.set("accel_homing_model default", axis + "_threshold", value)
        self.config.set("accel_homing_model default", axis + "_successes", "10")
        return rollback

    def save(self):
        return self.gcode.handlers["SAVE_CONFIG"](object())


class CalibrationConfigTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.rig = Rig(self.directory.name)

    def test_registration_and_staging_make_no_file_changes(self):
        root, included = self.rig.root.read_bytes(), self.rig.include.read_bytes()
        before = self.rig.include.stat().st_mtime_ns
        self.rig.stage()
        self.assertEqual(self.rig.root.read_bytes(), root)
        self.assertEqual(self.rig.include.read_bytes(), included)
        self.assertEqual(self.rig.include.stat().st_mtime_ns, before)
        self.assertEqual(self.rig.config.calls, 0)

    def test_save_comments_only_selected_included_key_and_keeps_exact_other_bytes(self):
        original = self.rig.include.read_bytes()
        self.rig.stage()
        self.assertEqual(self.rig.save(), "native result")
        self.assertEqual(self.rig.include.read_bytes(), original.replace(b"\r\nx_threshold: 8", b"\r\n#x_threshold: 8"))
        self.assertIn("x_threshold = 2.75", self.rig.root.read_text())
        self.assertEqual(self.rig.helper._targets, {})
        self.assertFalse(list(Path(self.directory.name).glob(".accel_homing_save_*")))

    def test_case_insensitive_selected_option_preserves_uppercase_other_axis(self):
        original = b"[accel_homing]\nX_THRESHOLD: 8\nY_THRESHOLD: 7\n"
        self.rig.include.write_bytes(original)
        self.rig.stage()
        self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), original.replace(b"X_THRESHOLD", b"#X_THRESHOLD"))

    def test_indented_options_are_commented_without_matching_multiline_value_text(self):
        original = (b"[accel_homing]\n  X_THRESHOLD: 8\n  Y_THRESHOLD: 7\n"
                    b"  note: preserved\n    x_threshold: 123\n"
                    b"    [accel_homing]\n    y_threshold: 456\n")
        self.rig.include.write_bytes(original)
        self.rig.stage()
        self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), original.replace(b"  X_THRESHOLD", b"#  X_THRESHOLD"))

    def test_unrelated_pending_multiline_beacon_model_serializes_normally(self):
        self.rig.stage()
        self.rig.config.set("beacon model default", "model_coef", "1.0,\n  2.0,\n  3.0")
        self.rig.save()
        self.assertEqual(self.rig.helper._targets, {})
        self.assertIn("model_coef = 1.0,", self.rig.root.read_text())

    def test_native_pending_owner_supported_on_modern_layout(self):
        self.rig = Rig(self.directory.name, modern=True)
        self.rig.stage("y", "3.25")
        self.rig.save()
        self.assertIn(b"#y_threshold: 7", self.rig.include.read_bytes())
        self.assertIn(b"\nx_threshold: 8", self.rig.include.read_bytes())

    def test_sequential_axis_registry_and_rollback_preserve_prior_target(self):
        self.rig.stage("x", "2.75")
        rollback = self.rig.helper.register_threshold("y", "3.25")
        rollback()
        rollback()
        self.assertEqual(set(self.rig.helper._targets), {("accel_homing", "x_threshold")})
        self.rig.stage("y", "3.25")
        self.rig.save()
        self.assertIn(b"#x_threshold: 8", self.rig.include.read_bytes())
        self.assertIn(b"#y_threshold: 7", self.rig.include.read_bytes())

    def test_empty_registry_forwards_unrelated_save_unchanged(self):
        rollback = self.rig.helper.register_threshold("x", "2.75")
        rollback()
        original = self.rig.include.read_bytes()
        self.rig.config.set("input_shaper", "shaper_freq_x", "50.0")
        with mock.patch.object(persistence, "_snapshot", side_effect=AssertionError("unrelated read")):
            self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), original)
        self.assertEqual(self.rig.config.calls, 1)

    def test_failed_handler_install_restores_original_public_command(self):
        original = self.rig.gcode.handlers["SAVE_CONFIG"]
        self.rig.gcode.fail_install = True
        with self.assertRaisesRegex(SaveError, "installation failed"):
            self.rig.helper.register_threshold("x", "2.75")
        self.assertEqual(self.rig.gcode.handlers["SAVE_CONFIG"], original)
        self.assertEqual(self.rig.helper._targets, {})

    def test_changed_pending_parser_or_status_is_rejected_before_edits(self):
        for target in ("parser", "pending"):
            self.rig.stage()
            original = self.rig.include.read_bytes()
            if target == "parser":
                self.rig.config.autosave.fileconfig.set("accel_homing", "x_threshold", "4")
            else:
                self.rig.config.state.status_save_pending["accel_homing"]["x_threshold"] = "4"
            with self.subTest(target=target), self.assertRaisesRegex(SaveError, "changed after staging"):
                self.rig.save()
            self.assertEqual(self.rig.include.read_bytes(), original)
            self.assertEqual(self.rig.config.calls, 0)

    def test_native_exception_before_root_write_restores_include_and_allows_retry(self):
        original = self.rig.include.read_bytes()
        self.rig.stage()
        def fail():
            self.assertIn(b"#x_threshold", self.rig.include.read_bytes())
            raise SaveError("native save failed")
        self.rig.config.before = fail
        with self.assertRaisesRegex(SaveError, "edits were restored.*native save failed"):
            self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), original)
        self.assertTrue(self.rig.helper._targets)
        self.rig.config.before = None
        self.rig.save()
        self.assertEqual(self.rig.helper._targets, {})

    def test_restart_exception_after_root_write_preserves_comments_and_reports_uncertainty(self):
        self.rig.stage()
        self.rig.config.after = lambda: (_ for _ in ()).throw(SaveError("restart failed"))
        with self.assertRaisesRegex(SaveError, "main configuration changed.*state is unconfirmed"):
            self.rig.save()
        self.assertIn(b"#x_threshold: 8", self.rig.include.read_bytes())
        self.assertIn("x_threshold = 2.75", self.rig.root.read_text())
        self.assertTrue(self.rig.helper._targets)

    def test_native_error_preserves_concurrent_include_edits(self):
        self.rig.stage()
        changed = b"[accel_homing]\nx_threshold: 4\n# concurrent operator change\n"
        def fail():
            self.rig.include.write_bytes(changed)
            raise SaveError("native failed")
        self.rig.config.before = fail
        with self.assertRaisesRegex(SaveError, "concurrent edits were preserved"):
            self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), changed)

    def test_handler_return_without_native_save_restores_comments(self):
        original = self.rig.include.read_bytes()
        self.rig.gcode.handlers["SAVE_CONFIG"] = lambda command: None
        self.rig.stage()
        with self.assertRaisesRegex(SaveError, "did not confirm"):
            self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), original)
        self.assertTrue(self.rig.helper._targets)

    def test_nested_wildcards_and_repeated_include_only_modify_registered_key(self):
        directory = Path(self.directory.name)
        (directory / "nested").mkdir()
        first = directory / "nested" / "a.cfg"
        first.write_bytes(b"[include ../plugin.cfg]\n[include ../plugin.cfg]\n")
        (directory / "nested" / "b.cfg").write_bytes(b"[other]\nx_threshold: 99\n")
        self.rig.root.write_bytes(b"[include nested/*.cfg]\n[include absent*.cfg]\n")
        self.rig.stage()
        self.rig.save()
        self.assertIn(b"#x_threshold: 8", self.rig.include.read_bytes())
        self.assertEqual(first.read_bytes(), b"[include ../plugin.cfg]\n[include ../plugin.cfg]\n")
        self.assertEqual((directory / "nested" / "b.cfg").read_bytes(), b"[other]\nx_threshold: 99\n")

    def test_root_selected_key_is_left_for_native_serializer(self):
        self.rig.root.write_bytes(b"[accel_homing]\nx_threshold: 8\ny_threshold: 7\n")
        self.rig.stage()
        with mock.patch.object(persistence, "_replace", side_effect=AssertionError("root replacement")):
            self.rig.save()
        self.assertIn("#x_threshold: 8", self.rig.root.read_text())
        self.assertIn("y_threshold: 7", self.rig.root.read_text())

    def test_indented_root_target_is_rejected_even_when_value_already_matches(self):
        for previous in ("8", "2.75"):
            self.rig.root.write_text("[include plugin.cfg]\n[accel_homing]\n  x_threshold: %s\n" % previous)
            root, included = self.rig.root.read_bytes(), self.rig.include.read_bytes()
            self.rig.stage()
            with self.subTest(previous=previous), self.assertRaisesRegex(SaveError, "cannot comment selected main-file"):
                self.rig.save()
            self.assertEqual(self.rig.root.read_bytes(), root)
            self.assertEqual(self.rig.include.read_bytes(), included)
            self.assertEqual(self.rig.config.calls, 0)
            self.assertTrue(self.rig.helper._targets)

    def test_include_symlink_is_preserved_and_hardlink_is_refused(self):
        alias = Path(self.directory.name) / "alias.cfg"
        try:
            alias.symlink_to(self.rig.include.name)
        except OSError:
            self.skipTest("symlink creation unavailable")
        self.rig.root.write_bytes(b"[include alias.cfg]\n")
        self.rig.stage()
        self.rig.save()
        self.assertTrue(alias.is_symlink())
        self.assertIn(b"#x_threshold", alias.read_bytes())
        self.rig.include.write_bytes(b"[accel_homing]\nx_threshold: 8\n")
        hardlink = Path(self.directory.name) / "hard.cfg"
        os.link(self.rig.include, hardlink)
        self.rig.stage()
        with self.assertRaisesRegex(SaveError, "hard-linked"):
            self.rig.save()
        self.assertEqual(hardlink.read_bytes(), b"[accel_homing]\nx_threshold: 8\n")

    def test_file_change_before_atomic_replace_is_not_overwritten(self):
        self.rig.stage()
        original_replace = persistence._replace
        changed = b"[accel_homing]\nx_threshold: 6\n"
        def race(snapshot, data):
            self.rig.include.write_bytes(changed)
            return original_replace(snapshot, data)
        with mock.patch.object(persistence, "_replace", side_effect=race):
            with self.assertRaisesRegex(SaveError, "changed before save"):
                self.rig.save()
        self.assertEqual(self.rig.include.read_bytes(), changed)


if __name__ == "__main__":
    unittest.main()
