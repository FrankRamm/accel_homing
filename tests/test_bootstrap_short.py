"""Offline local-entry fixtures; a fake Python executable never installs a host."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


DIRECTORY = Path(__file__).resolve().parent.parent


class LocalShellEntryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        git_bash = Path("C:/Program Files/Git/bin/bash.exe")
        cls.bash = str(git_bash) if git_bash.is_file() else shutil.which("bash")
        if not cls.bash:
            raise unittest.SkipTest("Bash unavailable for offline local-entry fixtures")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.source = self.directory / "clone with spaces"
        self.source.mkdir()
        self.bin = self.directory / "bin"
        self.bin.mkdir()
        self.calls = self.directory / "python-calls.txt"
        fake = self.bin / "python3"
        fake.write_bytes(b'#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$PYTHON_CALLS_PATH"\nexit "${FAKE_PYTHON_STATUS:-0}"\n')
        fake.chmod(0o755)
        for name in ("install.sh", "uninstall.sh", "bootstrap.sh", "installer.py", "uninstaller.py",
                     "accel_homing_updater.py", "accel_homing.py", "beacon_accel_homing.py",
                     "accel_homing_backends.py", "accel_homing_workflows.py", "accel_homing_model.py",
                     "accel_homing_charts.py", "accel_homing_filters.py", "LICENSE", "NOTICE"):
            target = self.source / name
            target.write_bytes((DIRECTORY / name).read_bytes())
            if name.endswith(".sh"):
                target.chmod(0o755)

    def run_entry(self, name, *arguments, status=0, downloaded=False):
        environment = dict(os.environ, PYTHON_CALLS_PATH=self.calls.as_posix(), FAKE_PYTHON_STATUS=str(status))
        # Git Bash translates this explicit POSIX PATH; no global Python is run.
        command = ('if command -v cygpath >/dev/null 2>&1; then native_bin=$(cygpath -u "$1"); '
                   'else native_bin=$1; fi; export PATH="$native_bin:$PATH"; shift; exec "$@"')
        if downloaded:
            argv = [self.bash, "-c", (self.source / name).read_text(), "--", *arguments]
        else:
            argv = [self.bash, "-c", command, "--", self.bin.as_posix(),
                    (self.source / name).as_posix(), *arguments]
        return subprocess.run(argv, env=environment, cwd=self.directory, text=True,
                              encoding="utf-8", capture_output=True, timeout=10)

    def assert_installer_call(self, arguments):
        lines = self.calls.read_text().splitlines()
        self.assertTrue(lines[0].replace("\\", "/").endswith("/clone with spaces/installer.py"), lines)
        self.assertEqual(lines[1:], list(arguments))

    def test_install_runs_directly_from_an_unrelated_directory_and_preserves_arguments(self):
        arguments = ("--dry-run", "--no-restart", "--config", "/fixture path/printer.cfg", "literal $(printf danger)")
        result = self.run_entry("install.sh", *arguments)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_installer_call(arguments)

    def test_uninstall_forces_full_mode_and_forwards_target_selection_offline(self):
        arguments = ("--dry-run", "--config", "/fixture path/printer.cfg")
        result = self.run_entry("uninstall.sh", *arguments)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_installer_call(("--uninstall", *arguments))

    def test_bootstrap_is_a_local_alias_with_the_same_argument_forwarding(self):
        result = self.run_entry("bootstrap.sh", "--dry-run", "--no-restart")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_installer_call(("--dry-run", "--no-restart"))

    def test_python_failure_is_propagated_by_every_entry(self):
        for name in ("install.sh", "uninstall.sh", "bootstrap.sh"):
            with self.subTest(name=name):
                result = self.run_entry(name, "--dry-run", status=29)
                self.assertEqual(result.returncode, 29, result.stderr)

    def test_incomplete_source_refuses_before_python_or_network(self):
        for name, missing in (("install.sh", "installer.py"), ("bootstrap.sh", "installer.py"),
                              ("uninstall.sh", "NOTICE")):
            with self.subTest(name=name):
                path = self.source / missing
                original = path.read_bytes()
                path.unlink()
                try:
                    result = self.run_entry(name, "--dry-run")
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("complete", result.stderr)
                    self.assertFalse(self.calls.exists())
                finally:
                    path.write_bytes(original)

    def test_downloaded_bootstrap_without_a_local_checkout_refuses(self):
        result = self.run_entry("bootstrap.sh", "--dry-run", downloaded=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Clone https://github.com/FrankRamm/accel_homing.git", result.stderr)
        self.assertFalse(self.calls.exists())

    def test_all_shell_sources_have_linux_line_endings_and_valid_bash_syntax(self):
        for name in ("install.sh", "uninstall.sh", "bootstrap.sh"):
            with self.subTest(name=name):
                data = (DIRECTORY / name).read_bytes()
                self.assertNotIn(b"\r", data)
                self.assertTrue(data.startswith(b"#!/usr/bin/env bash\n"))
                result = subprocess.run([self.bash, "-n"], input=data, capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr.decode())


if __name__ == "__main__":
    unittest.main()
