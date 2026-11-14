"""Real symlink-upgrade coverage for the optional experimental command."""
import shutil
import sys
import unittest
from unittest import mock

from harness import CommandError
import test_troubleshoot_imports as import_fixture
from test_accuracy_workflow import AccuracyRig
import test_workflows as workflow_fixture
from test_workflows import GCmd


class ExperimentImportTests(unittest.TestCase):
    def test_missing_module_does_not_break_boot_or_start_motion(self):
        with import_fixture.TroubleshootImportTests.installation(self):
            owner, printer = import_fixture.TroubleshootImportTests.load_owner(self)
            self.assertIn("ACCEL_HOMING_EXPERIMENT", printer.commands)
            self.assertIn("ACCEL_HOMING_TEST", printer.commands)
            self.assertNotIn("extras.accel_homing_experiment", sys.modules)
            module = sys.modules["extras.accel_homing_workflows"]
            with mock.patch.object(workflow_fixture, "AccelHomingWorkflows", module.AccelHomingWorkflows), \
                    mock.patch.object(module, "_PRINTER_G28", workflow_fixture.StockHoming.cmd_G28):
                rig = AccuracyRig()
                with mock.patch.object(rig.workflow, "_begin") as begin:
                    with self.assertRaisesRegex(CommandError, "EXPERIMENT modules could not load.*install.sh"):
                        rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, GC_GUARD=0))
                    begin.assert_not_called()
                self.assertEqual(rig.home_calls, [])
            self.assertEqual(printer.scripts, [])
            self.assertEqual(printer.objects["toolhead"].moves, [])

    def test_git_update_loads_exact_source_sibling_without_extra_link(self):
        with import_fixture.TroubleshootImportTests.installation(self) as (checkout, extras):
            shutil.copyfile(import_fixture.SOURCE / "accel_homing_experiment.py", checkout / "accel_homing_experiment.py")
            import_fixture.TroubleshootImportTests.load_owner(self)
            workflows = sys.modules["extras.accel_homing_workflows"]
            before = list(sys.path)
            model = workflows._load_experiment_module()
            self.assertEqual(model.__file__, str(checkout / "accel_homing_experiment.py"))
            self.assertTrue(callable(model.fit_experimental_profile))
            self.assertIs(model, workflows._load_experiment_module())
            self.assertFalse((extras / "accel_homing_experiment.py").exists())
            self.assertEqual(sys.path, before)


if __name__ == "__main__":
    unittest.main()
