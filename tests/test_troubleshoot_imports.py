"""Offline extras-package upgrades, using the actual former symlink layout."""
import contextlib
import importlib
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, FakeConfig, FakePrinter, FakeToolhead, make_beacon
from test_accuracy_workflow import AccuracyRig
from test_workflows import GCmd, workflow_module
import test_workflows as workflow_fixture


SOURCE = Path(__file__).resolve().parent.parent
OLD_MODULES = ("accel_homing.py", "beacon_accel_homing.py", "accel_homing_backends.py",
               "accel_homing_workflows.py", "accel_homing_model.py", "accel_homing_charts.py")
DIAGNOSTICS = "accel_homing_troubleshoot.py"


class TroubleshootImportTests(unittest.TestCase):
    @contextlib.contextmanager
    def installation(self, diagnostic_source=True, diagnostic_link=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            checkout, host = root / "checkout", root / "host"
            extras = host / "extras"
            checkout.mkdir()
            extras.mkdir(parents=True)
            (extras / "__init__.py").write_text("", encoding="utf-8")
            (extras / "homing.py").write_text(
                "class PrinterHoming:\n    def cmd_G28(self, gcmd):\n        pass\n", encoding="utf-8")
            for name in OLD_MODULES + ((DIAGNOSTICS,) if diagnostic_source else ()):
                shutil.copyfile(SOURCE / name, checkout / name)
            for name in OLD_MODULES + ((DIAGNOSTICS,) if diagnostic_link else ()):
                try:
                    (extras / name).symlink_to(checkout / name)
                except OSError as exc:
                    self.skipTest("real symlink creation unavailable: %s" % exc)
            search = [str(host)] + [entry for entry in sys.path if Path(entry or ".").resolve() != SOURCE]
            with mock.patch.dict(sys.modules), mock.patch.object(sys, "path", search):
                # The offline harness adds the checkout to sys.path. Remove that
                # convenience to exercise the same package-only import as Klippy.
                for name in list(sys.modules):
                    if name == "extras" or name.startswith(("extras.", "accel_homing", "beacon_accel_homing")):
                        sys.modules.pop(name)
                importlib.invalidate_caches()
                yield checkout, extras

    def load_owner(self):
        entry = importlib.import_module("extras.accel_homing")
        printer = FakePrinter(FakeToolhead(), make_beacon())
        owner = entry.load_config(FakeConfig(printer, {}, name="accel_homing"))
        return owner, printer

    def test_git_only_update_with_old_six_links_boots_ordinary_commands(self):
        with self.installation() as (checkout, extras):
            self.assertTrue((checkout / DIAGNOSTICS).is_file())
            self.assertFalse((extras / DIAGNOSTICS).exists())
            owner, printer = self.load_owner()
            self.assertIsNotNone(owner.workflows)
            for command in ("ACCEL_HOMING_ACCURACY", "ACCEL_HOMING_CALIBRATE", "SET_ACCEL_HOMING",
                            "ACCEL_HOMING_TROUBLESHOOT"):
                self.assertIn(command, printer.commands)
            self.assertEqual(printer.scripts, [])
            self.assertEqual(printer.objects["toolhead"].moves, [])
            self.assertNotIn("extras.accel_homing_troubleshoot", sys.modules)

    def test_old_links_load_exact_resolved_checkout_sibling_and_cache_without_path_changes(self):
        with self.installation() as (checkout, extras):
            self.load_owner()
            module = sys.modules["extras.accel_homing_workflows"]
            before = list(sys.path)
            diagnostics = module._load_troubleshoot_module()
            self.assertEqual(Path(diagnostics.__file__), checkout / DIAGNOSTICS)
            self.assertIs(diagnostics, module._load_troubleshoot_module())
            self.assertIs(sys.modules["extras.accel_homing_troubleshoot"], diagnostics)
            self.assertFalse((extras / DIAGNOSTICS).exists())
            self.assertEqual(sys.path, before)

    def test_new_link_uses_normal_package_import(self):
        with self.installation(diagnostic_link=True) as (checkout, extras):
            self.load_owner()
            module = sys.modules["extras.accel_homing_workflows"]
            diagnostics = module._load_troubleshoot_module()
            self.assertEqual(Path(diagnostics.__file__), extras / DIAGNOSTICS)
            self.assertEqual(Path(diagnostics.__file__).resolve(), checkout / DIAGNOSTICS)

    def test_missing_optional_source_is_command_only_actionable_error_before_preflight(self):
        with self.installation(diagnostic_source=False):
            owner, printer = self.load_owner()
            with mock.patch.object(owner.workflows, "_begin") as begin:
                with self.assertRaisesRegex(CommandError, "diagnostics could not load.*install.sh"):
                    owner.workflows.cmd_TROUBLESHOOT(GCmd(CENTERED=1, AXIS="X", SAMPLES=2))
                begin.assert_not_called()
            self.assertIsNone(owner.workflows.session)
            self.assertEqual(printer.scripts, [])
            self.assertEqual(printer.objects["toolhead"].moves, [])

    def test_broken_optional_dependency_keeps_original_cause_and_cleans_fallback_cache(self):
        for linked in (False, True):
            with self.subTest(linked=linked), self.installation(diagnostic_link=linked) as (checkout, extras):
                (checkout / DIAGNOSTICS).write_text("import absent_diagnostic_dependency\n", encoding="utf-8")
                owner, printer = self.load_owner()
                with mock.patch.object(owner.workflows, "_begin") as begin:
                    with self.assertRaisesRegex(CommandError, "absent_diagnostic_dependency") as caught:
                        owner.workflows.cmd_TROUBLESHOOT(GCmd(CENTERED=1, AXIS="X", SAMPLES=2))
                    begin.assert_not_called()
                self.assertIsInstance(caught.exception.__cause__, ModuleNotFoundError)
                self.assertEqual(caught.exception.__cause__.name, "absent_diagnostic_dependency")
                self.assertNotIn("extras.accel_homing_troubleshoot", sys.modules)
                self.assertIsNone(sys.modules["extras.accel_homing_workflows"]._TROUBLESHOOT_MODULE)
                self.assertEqual(printer.objects["toolhead"].moves, [])
                shutil.copyfile(SOURCE / DIAGNOSTICS, checkout / DIAGNOSTICS)
                importlib.invalidate_caches()
                recovered = sys.modules["extras.accel_homing_workflows"]._load_troubleshoot_module()
                self.assertTrue(callable(recovered.Recorder))

    def test_old_six_link_upgrade_runs_complete_synthetic_diagnostic_without_installation(self):
        with self.installation() as (checkout, extras):
            self.load_owner()
            module = sys.modules["extras.accel_homing_workflows"]
            self.assertIsNone(module._TROUBLESHOOT_MODULE)
            with mock.patch.object(workflow_fixture, "AccelHomingWorkflows", module.AccelHomingWorkflows), \
                    mock.patch.object(module, "_PRINTER_G28", workflow_fixture.StockHoming.cmd_G28):
                rig = AccuracyRig({"charts_directory": str(checkout / "test-evidence")})
                rig.workflow.cmd_TROUBLESHOOT(GCmd(CENTERED=1, AXIS="X", SAMPLES=2))
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertEqual(result["completed_samples"], {"x": 2})
            self.assertTrue(Path(result["artifacts"]["manifest"]).is_file())
            self.assertTrue(Path(result["artifacts"]["archive"]).is_file())
            self.assertEqual(len(rig.home_calls), 3)
            self.assertIsNone(rig.workflow.session)
            self.assertIsNone(rig.owner.troubleshoot)
            self.assertEqual(Path(module._TROUBLESHOOT_MODULE.__file__), checkout / DIAGNOSTICS)
            self.assertFalse((extras / DIAGNOSTICS).exists())

    def test_invalid_source_exports_remove_only_loader_owned_cache_entry(self):
        with self.installation() as (checkout, extras):
            (checkout / DIAGNOSTICS).write_text("unrelated = 1\n", encoding="utf-8")
            self.load_owner()
            module = sys.modules["extras.accel_homing_workflows"]
            with self.assertRaisesRegex(ImportError, "provide Recorder and snapshot"):
                module._load_troubleshoot_module()
            self.assertNotIn("extras.accel_homing_troubleshoot", sys.modules)
            foreign = types.ModuleType("extras.accel_homing_troubleshoot")
            sys.modules[foreign.__name__] = foreign
            with self.assertRaisesRegex(ImportError, "provide Recorder and snapshot"):
                module._load_troubleshoot_module()
            self.assertIs(sys.modules[foreign.__name__], foreign)

    def test_existing_core_dependency_failures_are_not_replaced_by_standalone_fallback(self):
        for filename in ("accel_homing_backends.py", "accel_homing_workflows.py", "accel_homing_charts.py"):
            with self.subTest(filename=filename), self.installation() as (checkout, extras):
                (checkout / filename).write_text("import absent_core_dependency\n", encoding="utf-8")
                with self.assertRaises(ModuleNotFoundError) as caught:
                    self.load_owner()
                self.assertEqual(caught.exception.name, "absent_core_dependency")

    def test_sibling_fallback_refuses_source_outside_resolved_checkout(self):
        with self.installation() as (checkout, extras):
            diagnostic = checkout / DIAGNOSTICS
            outside = checkout.parent / "outside_diagnostic.py"
            diagnostic.replace(outside)
            diagnostic.symlink_to(outside)
            self.load_owner()
            with self.assertRaisesRegex(ImportError, "beside the installed workflow source"):
                sys.modules["extras.accel_homing_workflows"]._load_troubleshoot_module()
            self.assertNotIn("extras.accel_homing_troubleshoot", sys.modules)

    def test_standalone_module_import_remains_supported(self):
        with self.installation() as (checkout, extras):
            with mock.patch.object(sys, "path", [str(checkout)] + sys.path):
                module = importlib.import_module("accel_homing_workflows")
                self.assertFalse(module.__package__)
                self.assertIsNone(module._PRINTER_G28)
                diagnostics = module._load_troubleshoot_module()
                self.assertEqual(diagnostics.__name__, "accel_homing_troubleshoot")
                self.assertEqual(Path(diagnostics.__file__), checkout / DIAGNOSTICS)

    def test_removed_synthetic_package_can_load_trusted_sibling_without_sys_path_changes(self):
        name = workflow_module.__package__ + ".accel_homing_troubleshoot"
        with mock.patch.dict(sys.modules), mock.patch.object(workflow_module, "_TROUBLESHOOT_MODULE", None):
            sys.modules.pop(workflow_module.__package__, None)
            sys.modules.pop(name, None)
            before = list(sys.path)
            diagnostics = workflow_module._load_troubleshoot_module()
            self.assertEqual(Path(diagnostics.__file__), SOURCE / DIAGNOSTICS)
            self.assertEqual(sys.path, before)


if __name__ == "__main__":
    unittest.main()
