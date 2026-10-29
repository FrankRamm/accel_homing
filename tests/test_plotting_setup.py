"""Mocked dependency setup contracts; no pip, services or printer access."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import types
import unittest
from unittest import mock

from test_installer import installer


class PlottingPythonTests(unittest.TestCase):
    def selected(self, executable=None):
        return {"executable": executable, "pid": None, "service": None,
                "repo": Path("selected-repo"), "config": Path("selected-printer.cfg"),
                "socket": None, "origin": "fixture"}

    def details(self, executable, prefix):
        return {"executable": executable, "prefix": str(prefix), "base_prefix": str(prefix.parent / "system"),
                "cache_tag": "cpython-311", "uid": 1000, "prefix_uid": 1000,
                "writable": True, "venv_config": True}

    def test_lexical_venv_command_is_preserved_and_bootstrap_python_is_never_used(self):
        with tempfile.TemporaryDirectory() as temp:
            prefix = Path(temp) / "klippy-env"
            executable = str(prefix / "bin/python")
            response = types.SimpleNamespace(returncode=0, stdout=json.dumps(self.details(executable, prefix)))
            with mock.patch.object(installer.os, "getuid", return_value=1000, create=True), \
                    mock.patch.object(installer.subprocess, "run", return_value=response) as run:
                found = installer.inspect_plotting_python(self.selected(executable))
            self.assertEqual(found["command"], executable)
            self.assertEqual(run.call_args.args[0][:3], [executable, "-I", "-c"])
            self.assertNotEqual(found["command"], installer.sys.executable)
            with self.assertRaisesRegex(installer.InstallError, "does not match"):
                installer.inspect_plotting_python(self.selected(executable), str(Path(temp) / "other/bin/python"))
            with self.assertRaisesRegex(installer.InstallError, "Cannot identify"):
                installer.inspect_plotting_python(self.selected())

    def test_global_foreign_unwritable_or_failed_interpreters_fail_before_pip(self):
        with tempfile.TemporaryDirectory() as temp:
            prefix = Path(temp) / "venv"
            executable = str(prefix / "bin/python")
            for change in ({"base_prefix": str(prefix)}, {"prefix_uid": 2000}, {"uid": 0},
                           {"writable": False}, {"venv_config": False}):
                with self.subTest(change=change):
                    details = {**self.details(executable, prefix), **change}
                    with mock.patch.object(installer.os, "getuid", return_value=1000, create=True), \
                            mock.patch.object(installer.subprocess, "run", return_value=types.SimpleNamespace(
                                returncode=0, stdout=json.dumps(details))) as run:
                        with self.assertRaises(installer.InstallError):
                            installer.inspect_plotting_python(self.selected(executable))
                    self.assertEqual(run.call_count, 1)
            with mock.patch.object(installer.subprocess, "run", side_effect=OSError("missing interpreter")):
                with self.assertRaisesRegex(installer.InstallError, "Cannot interrogate"):
                    installer.inspect_plotting_python(self.selected(executable))

    def test_live_process_identity_mismatch_cannot_override_selected_environment(self):
        selected = self.selected(str(Path.cwd() / "venv/bin/python"))
        selected.update(pid=123, start_time="original")
        with mock.patch.object(installer, "proc_instances", return_value=[{**selected, "start_time": "changed"}]), \
                mock.patch.object(installer.subprocess, "run") as run:
            with self.assertRaisesRegex(installer.InstallError, "changed"):
                installer.inspect_plotting_python(selected)
            run.assert_not_called()


class PrivatePlottingSetupTests(unittest.TestCase):
    @contextlib.contextmanager
    def fixture(self, available=False, dry_run=False, failure=None):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "release"
            source.mkdir()
            target = source / ".plot-deps/cpython-311-envkey"
            events = []
            details = {"command": str(Path(temp) / "venv/bin/python"),
                       "prefix": str(Path(temp) / "venv"), "cache_tag": "cpython-311"}
            selected = {"executable": details["command"], "pid": None, "service": None}
            def verify(renderer=None, **kwargs):
                events.append(("render", renderer.directory))
                if failure == "existing" and renderer.directory == target:
                    raise RuntimeError("existing cache is invalid")
                if renderer.directory == target and not available and not target.exists():
                    raise RuntimeError("matplotlib unavailable")
                if failure == "render" and renderer.directory != target:
                    raise RuntimeError("wheel import failed")
                return True
            charts = types.SimpleNamespace(
                default_dependency_directory=lambda prefix, tag: target,
                PNGRenderer=lambda executable, directory: types.SimpleNamespace(
                    executable=executable, directory=Path(directory)), preflight_renderer=verify)
            def pip(command, **kwargs):
                events.append(("pip", command))
                if failure == "timeout":
                    raise subprocess.TimeoutExpired(command, 300)
                if failure == "pip":
                    return types.SimpleNamespace(returncode=1, stderr="No matching binary wheel")
                (Path(command[command.index("--target") + 1]) / "private-wheel.txt").write_text("fixture wheel")
                return types.SimpleNamespace(returncode=0, stderr="")
            with mock.patch.object(installer, "__file__", str(source / "installer.py")), \
                    mock.patch.object(installer, "inspect_plotting_python", return_value=details), \
                    mock.patch.object(installer, "assert_python_instance", side_effect=lambda *a: events.append(("identity", None))), \
                    mock.patch.object(installer, "load_plotting", return_value=charts), \
                    mock.patch.object(installer.subprocess, "run", side_effect=pip) as run, \
                    contextlib.redirect_stdout(io.StringIO()):
                args = types.SimpleNamespace(dry_run=dry_run, klipper_python=None)
                guard = lambda: events.append(("idle", None))
                yield selected, args, guard, target, events, run

    def test_existing_real_render_is_reused_without_pip_or_private_writes(self):
        with self.fixture(available=True) as (selected, args, guard, target, events, run):
            result = installer.prepare_plotting(selected, args, guard)
            self.assertTrue(result["available"])
            run.assert_not_called()
            self.assertFalse(target.exists())
            self.assertEqual(events[0][0], "idle")

    def test_private_binary_wheels_are_guarded_verified_and_atomically_published(self):
        with self.fixture() as (selected, args, guard, target, events, run):
            result = installer.prepare_plotting(selected, args, guard)
            self.assertTrue(result["available"])
            self.assertTrue((target / "private-wheel.txt").is_file())
            self.assertEqual(len(list(target.parent.iterdir())), 1)
            command = run.call_args.args[0]
            self.assertEqual(command[:5], [selected["executable"], "-I", "-m", "pip", "--isolated"])
            self.assertIn("--only-binary=:all:", command)
            self.assertNotIn("--upgrade", command)
            self.assertNotIn("--user", command)
            self.assertGreaterEqual([event[0] for event in events].count("idle"), 3)
            self.assertTrue(any(event[0] == "render" and event[1].name.startswith(".prepare-") for event in events))

    def test_missing_wheel_timeout_or_failed_render_leaves_no_dependencies_or_printer_writes(self):
        for failure in ("pip", "timeout", "render"):
            with self.subTest(failure=failure), self.fixture(failure=failure) as fixture:
                selected, args, guard, target, events, run = fixture
                with self.assertRaisesRegex(installer.InstallError, "failed"):
                    installer.prepare_plotting(selected, args, guard)
                self.assertFalse(target.exists())
                self.assertEqual(list(target.parent.iterdir()), [])

    def test_dry_run_missing_renderer_reports_without_install_or_directory_creation(self):
        with self.fixture(dry_run=True) as (selected, args, guard, target, events, run):
            result = installer.prepare_plotting(selected, args)
            self.assertFalse(result["available"])
            run.assert_not_called()
            self.assertFalse(target.parent.exists())

    def test_existing_invalid_private_cache_is_preserved_without_in_place_pip(self):
        with self.fixture(failure="existing") as (selected, args, guard, target, events, run):
            target.mkdir(parents=True)
            existing = target / "retain-user-cache.txt"
            existing.write_text("preserve this cache")
            with self.assertRaisesRegex(installer.InstallError, "Preserve/review"):
                installer.prepare_plotting(selected, args, guard)
            self.assertEqual(existing.read_text(), "preserve this cache")
            self.assertEqual(list(target.parent.iterdir()), [target])
            run.assert_not_called()

    def test_dependency_failure_precedes_printer_file_transaction_or_restart(self):
        selected = {"repo": Path("fixture-repo"), "config": Path("fixture.cfg"),
                    "origin": "fixture", "service": None, "executable": None}
        with mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
                mock.patch.object(installer, "discover_target", return_value=selected), \
                mock.patch.object(installer, "make_plan", return_value=({}, [], "beacon")), \
                mock.patch.object(installer, "prepare_plotting", side_effect=installer.InstallError("dependency failure")), \
                mock.patch.object(installer, "apply_plan") as apply, \
                mock.patch.object(installer, "activate_install") as activate, \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(installer.InstallError, "dependency failure"):
                installer.main(["--no-restart", "--no-update-manager"])
        apply.assert_not_called()
        activate.assert_not_called()


if __name__ == "__main__":
    unittest.main()
