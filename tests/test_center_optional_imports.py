"""Optional center/travel workflow modules in the real former symlink layout."""
import shutil
import unittest

from harness import CommandError
import test_troubleshoot_imports as troubleshoot_imports
from test_workflows import GCmd


CENTER = "accel_homing_center.py"
TRAVEL = "accel_homing_travel.py"


class CenterOptionalImportTests(unittest.TestCase):
    # Reuse the fixture methods without collecting the troubleshooting tests twice.
    installation = troubleshoot_imports.TroubleshootImportTests.installation
    load_owner = troubleshoot_imports.TroubleshootImportTests.load_owner

    def link(self, checkout, extras, name):
        if not (checkout / name).exists():
            shutil.copyfile(troubleshoot_imports.SOURCE / name, checkout / name)
        (extras / name).symlink_to(checkout / name)

    def test_six_legacy_links_boot_with_center_and_travel_unavailable(self):
        with self.installation() as (_checkout, _extras):
            owner, printer = self.load_owner()
            self.assertIsNone(owner.workflows.center)
            self.assertIsNone(owner.workflows.travel)
            self.assertEqual(printer.commands["ACCEL_HOMING_CALIBRATE_CENTER"],
                             owner.workflows._center_unavailable)
            self.assertEqual(printer.commands["ACCEL_HOMING_TRAVEL"], owner.workflows._travel_unavailable)
            for command in ("ACCEL_HOMING_CALIBRATE_CENTER", "ACCEL_HOMING_TRAVEL"):
                with self.subTest(command=command), \
                        self.assertRaisesRegex(CommandError, "needs its new installed module"):
                    printer.commands[command](GCmd())
            self.assertIsNone(owner.workflows.session)
            self.assertEqual(printer.scripts, [])
            self.assertEqual(printer.objects["toolhead"].moves, [])

    def test_center_link_without_travel_link_keeps_travel_unavailable(self):
        with self.installation() as (checkout, extras):
            self.link(checkout, extras, CENTER)
            owner, printer = self.load_owner()
            self.assertIsNotNone(owner.workflows.center)
            self.assertIsNone(owner.workflows.travel)
            self.assertEqual(printer.commands["ACCEL_HOMING_CALIBRATE_CENTER"], owner.workflows.center.command)

    def test_center_and_travel_links_load_both_workflows(self):
        with self.installation() as (checkout, extras):
            for name in (CENTER, TRAVEL):
                self.link(checkout, extras, name)
            owner, printer = self.load_owner()
            self.assertIsNotNone(owner.workflows.center)
            self.assertIsNotNone(owner.workflows.travel)
            self.assertEqual(printer.commands["ACCEL_HOMING_TRAVEL"], owner.workflows.travel.command)

    def test_center_dependency_failure_propagates_its_original_error(self):
        with self.installation() as (checkout, extras):
            (checkout / CENTER).write_text("import absent_center_dependency\n", encoding="utf-8")
            self.link(checkout, extras, CENTER)
            with self.assertRaises(ModuleNotFoundError) as caught:
                self.load_owner()
            self.assertEqual(caught.exception.name, "absent_center_dependency")


if __name__ == "__main__":
    unittest.main()
