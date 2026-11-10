"""Offline fixtures: the standard installer must accept root and hosts without sudo/systemd."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest import mock

DIRECTORY = Path(__file__).resolve().parent.parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, DIRECTORY / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


installer = load("installer")


class RootInstallTest(unittest.TestCase):
    def test_main_does_not_refuse_root(self):
        # Root must get past the former "not root" refusal and reach target discovery.
        with mock.patch.object(installer.os, "geteuid", return_value=0, create=True), \
                mock.patch.object(installer, "discover_target", side_effect=installer.InstallError("reached discovery")), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(installer.InstallError, "reached discovery"):
                installer.main(["--dry-run"])

    def test_root_without_systemd_skips_automatic_restart(self):
        output = io.StringIO()
        with mock.patch.object(installer.os, "geteuid", return_value=0, create=True), \
                mock.patch.object(installer.shutil, "which", return_value=None), \
                contextlib.redirect_stdout(output):
            self.assertIsNone(installer.prepare_restart({"service": None}))
        self.assertIn("skipping automatic restart", output.getvalue())

    def test_non_root_without_sudo_still_reports_clear_error(self):
        with mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
                mock.patch.object(installer.shutil, "which", return_value=None):
            with self.assertRaisesRegex(installer.InstallError, "requires systemctl and sudo"):
                installer.prepare_restart({"service": "klipper.service"})

    def test_root_restart_permission_needs_no_sudo(self):
        self.assertIsNone(installer.restart_permission({"sudo": None, "service_command": ["x"]}, acquire=True))

    def test_skip_plotting_never_touches_python_environment(self):
        args = mock.Mock(skip_plotting=True, dry_run=False)
        called = []
        with mock.patch.object(installer, "inspect_plotting_python", side_effect=AssertionError("must not run")), \
                contextlib.redirect_stdout(io.StringIO()):
            result = installer.prepare_plotting({}, args, before_write=lambda: called.append(1))
        self.assertFalse(result["available"])
        self.assertEqual(called, [1])


if __name__ == "__main__":
    unittest.main()
