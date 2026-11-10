"""Offline fixtures for the Creality K1 installer. Never touches a printer or service."""
import importlib.util
import os
from pathlib import Path
import shutil
import tempfile
import unittest

DIRECTORY = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("k1_installer", DIRECTORY / "k1_installer.py")
k1 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(k1)

PRINTER_CFG = """[include sensorless.cfg]

[stepper_x]
endstop_pin: tmc2209_stepper_x:virtual_endstop  # sensorless
homing_retract_dist: 0

[tmc2209 stepper_x]
diag_pin: ^PA1
driver_SGTHRS: 80

[stepper_y]
endstop_pin: tmc2209_stepper_y:virtual_endstop
homing_retract_dist: 5
min_home_dist: 3

[tmc2209 stepper_y]
diag_pin: ^PA2
home_current: 0.4

[adxl345]
cs_pin: nozzle_mcu:PA3

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [input_shaper]
#*# shaper_type_x = mzv
"""


class K1InstallerTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.root, True)
        self.repo = self.root / "usr/data/klipper"
        (self.repo / "klippy/extras").mkdir(parents=True)
        (self.repo / "klippy/klippy.py").write_text("")
        (self.repo / "klippy/mcu.py").write_text("class TriggerDispatch: pass\n")
        (self.repo / "klippy/extras/adxl345.py").write_text("batch_bulk = 1\n")
        self.config_dir = self.root / "usr/data/printer_data/config"
        self.config_dir.mkdir(parents=True)
        self.printer = self.config_dir / "printer.cfg"
        self.printer.write_text(PRINTER_CFG)
        (self.config_dir / "sensorless.cfg").write_text("[gcode_macro S]\ngcode:\n  G4 P1\n")
        self.args = ["--klipper-path", str(self.repo), "--config", str(self.printer),
                     "--init-script", str(self.root / "missing-init"), "--no-restart"]
        self.state = k1.STATE_FILE
        self.addCleanup(lambda: os.path.exists(self.state) and os.remove(self.state))

    def run_main(self, *extra):
        return k1.main(list(extra) + self.args)

    def test_install_edits_and_links(self):
        self.assertEqual(self.run_main(), 0)
        text = self.printer.read_text()
        self.assertIn("endstop_pin: accel_homing:x  # accel_homing_k1", text)
        self.assertIn("endstop_pin: accel_homing:y  # accel_homing_k1", text)
        self.assertIn("#k1ah# diag_pin: ^PA1", text)
        self.assertIn("#k1ah# min_home_dist: 3", text)
        self.assertIn("#k1ah# home_current: 0.4", text)
        self.assertEqual(text.count("homing_retract_dist: 0"), 2)
        self.assertTrue(text.startswith("# accel_homing_k1\n[include accel_homing.cfg]\n"))
        self.assertIn("[accel_homing]", (self.config_dir / "accel_homing.cfg").read_text())
        for module in k1.MODULES:
            link = self.repo / "klippy/extras" / module
            self.assertTrue(link.is_symlink())
        self.assertTrue(list((self.config_dir / ".accel_homing_backups").iterdir()))

    def test_install_is_idempotent(self):
        self.run_main()
        first = self.printer.read_text()
        self.assertEqual(self.run_main(), 0)
        self.assertEqual(self.printer.read_text(), first)

    def test_dry_run_writes_nothing(self):
        self.assertEqual(self.run_main("--dry-run"), 0)
        self.assertEqual(self.printer.read_text(), PRINTER_CFG)
        self.assertFalse((self.config_dir / "accel_homing.cfg").exists())
        self.assertFalse(list((self.repo / "klippy/extras").glob("accel_homing*")))

    def test_uninstall_restores_config_and_keeps_later_save_config(self):
        self.run_main()
        with open(self.printer, "a") as handle:
            handle.write("#*# [accel_homing_model default]\n#*# x_threshold = 1.5\n")
        self.assertEqual(self.run_main("--uninstall"), 0)
        self.assertEqual(self.printer.read_text(), PRINTER_CFG)
        self.assertFalse((self.config_dir / "accel_homing.cfg").exists())
        self.assertFalse(list((self.repo / "klippy/extras").glob("accel_homing*")))

    def test_conflicting_regular_file_is_refused_without_force(self):
        (self.repo / "klippy/extras/accel_homing.py").write_text("# other\n")
        self.assertEqual(self.run_main(), 1)
        self.assertEqual(self.printer.read_text(), PRINTER_CFG)
        self.assertEqual(self.run_main("--force"), 0)
        self.assertTrue((self.repo / "klippy/extras/accel_homing.py.k1bak").exists())

    def test_missing_stepper_section_aborts_before_writes(self):
        self.printer.write_text(PRINTER_CFG.replace("[stepper_y]", "[stepper_z]"))
        self.assertEqual(self.run_main(), 1)
        self.assertFalse(list((self.repo / "klippy/extras").glob("accel_homing*")))

    def test_edits_in_included_file(self):
        stepper = "[stepper_x]\nendstop_pin: PA1\n\n[stepper_y]\nendstop_pin: PA2\n"
        (self.config_dir / "sensorless.cfg").write_text(stepper)
        self.printer.write_text("[include sensorless.cfg]\n[adxl345]\ncs_pin: PA3\n")
        self.assertEqual(self.run_main(), 0)
        self.assertIn("accel_homing:x", (self.config_dir / "sensorless.cfg").read_text())
        self.assertEqual(self.run_main("--uninstall"), 0)
        self.assertEqual((self.config_dir / "sensorless.cfg").read_text(), stepper)


if __name__ == "__main__":
    unittest.main()
