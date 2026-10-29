"""Offline synthetic host/config fixtures. Never contact a printer or a service."""
import base64
import contextlib
import hashlib
import io
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

DIRECTORY = Path(__file__).resolve().parent.parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, DIRECTORY / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


installer = load("installer")
bootstrap = load("bootstrap")

CONFIG = """# Keep my printer settings
[printer]
kinematics: corexy
max_velocity: 300
max_accel: 5000
[include axes.cfg]
[beacon]
serial: /dev/serial/by-id/fixture-only
[gcode_macro PRINT_START]
gcode:
    M118 Leave this exact macro alone
"""
AXES = """[stepper_x]
endstop_pin: tmc2209_stepper_x:virtual_endstop
position_endstop: 0
position_max: 350
homing_speed: 150
homing_retract_dist: 5
homing_accel: 1000
min_home_dist: 10
[stepper_y]
endstop_pin: !PA0
position_endstop: 350
position_max: 350
homing_speed: 35
homing_retract_dist: 3
[tmc2209 stepper_x]
diag_pin: ^PB0
run_current: 1.2
home_current: 0.6
driver_SGTHRS: 50
[tmc5160 stepper_y]
diag0_pin: PB1
run_current: 1.5
[tmc2209 stepper_z]
diag_pin: PB2
run_current: 0.8
"""
SAVED = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*# [beacon model default]
#*# model_coef = 1.234
#*# [accel_homing_model default]
#*# x_threshold = 6.5
"""
TRAVEL_LIMITS = "\n[accel_homing]\nmax_sample_age: 0.020\nmax_report_travel: 1.0\nmax_startup_travel: 3.0\n"
SAFE_Z_HOME = """# Preserve the operator's probe homing setup
[safe_z_home]
home_xy_position: 175, 175
speed: 90
z_hop: 15
z_hop_speed: 8
move_to_previous: True
"""
HOMING_OVERRIDE = """# Preserve the operator's complete normal homing sequence
[homing_override]
axes: xyz
set_position_z: 0
gcode:
    SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.7
    SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1
    SYNC_MOTORS
    G28 X Y
    G28 Z
    BEACON_AUTO_CALIBRATE
"""


class HostFixture(unittest.TestCase):
    def setUp(self):
        # Environment/pip behavior has independent mocked contract tests. Host
        # lifecycle fixtures must never invoke a real interpreter or installer.
        plotting = mock.patch.object(installer, "prepare_plotting", return_value={"available": True})
        plotting.start()
        self.addCleanup(plotting.stop)
        updater = mock.patch.object(installer, "load_updater", return_value=None)
        updater.start()
        self.addCleanup(updater.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name)
        self.repo = self.home / "klipper"
        self.extras = self.repo / "klippy/extras"
        self.extras.mkdir(parents=True)
        (self.repo / "klippy/klippy.py").write_text("# fixture\n")
        (self.extras / "beacon.py").write_text("# fixture driver\n")
        (self.extras / "adxl345.py").write_text("# fixture driver\n")
        (self.repo / ".git/info").mkdir(parents=True)
        (self.repo / ".git/info/exclude").write_text("# existing\n")
        self.config = self.home / "printer_data/config/printer.cfg"
        self.config.parent.mkdir(parents=True)
        self.config.write_bytes((CONFIG + SAVED).encode())
        self.axes = self.config.parent / "axes.cfg"
        self.axes.write_bytes(AXES.encode())
        self.source = self.home / "package"
        self.source.mkdir()
        for name in installer.MODULES:
            (self.source / name).write_text("# fixture source\n")
        self.selected = dict(repo=self.repo.resolve(), config=self.config.resolve(), socket=None,
                             pid=None, service=None, origin="fixture", executable=None)

    def plan(self, *flags):
        return installer.make_plan(self.source, self.selected, installer.arguments(list(flags)))

    def cartographer_safe_z_fixture(self):
        probe_home = self.config.parent / "probe-home.cfg"
        probe_home.write_bytes(SAFE_Z_HOME.replace("\n", "\r\n").encode())
        config = CONFIG.replace("[beacon]", "[scanner]")
        config += ("\n[adxl345 toolhead]\ncs_pin: fixture:None\n"
                   "[include probe-home.cfg]\n")
        saved = SAVED.replace("[beacon model default]", "[scanner model default]")
        self.config.write_bytes((config + saved).encode())
        self.axes.write_bytes(AXES.replace("homing_speed: 150", "homing_speed: 50").encode())
        return probe_home, saved

    def retained_y_policy_fixture(self, options, speed=50, startup=4.5):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[accel_homing]\nmax_sample_age: 0.020\nmax_report_travel: 1.5\nmax_startup_travel: "
                               + str(startup) + "\n" + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50")
                             .replace("homing_speed: 35", "homing_speed: " + str(speed))
                             .replace("endstop_pin: !PA0", "endstop_pin: accel_homing:y")
                             .replace("homing_retract_dist: 3", options))

    def require_symlinks(self):
        link = self.home / "symlink-test"
        try:
            os.symlink(self.source, link, target_is_directory=True)
            link.unlink()
        except OSError:
            self.skipTest("Host does not allow symlink creation")

    def test_common_directory_detection(self):
        chosen = installer.discover_target(installer.arguments([]), self.home, [], [])
        self.assertEqual(chosen["repo"], self.repo)
        self.assertEqual(chosen["config"], self.config)

    def test_running_process_takes_priority_and_executable_retained(self):
        live = dict(self.selected, pid=123, executable="/venv/bin/python", origin="running", service="custom.service")
        chosen = installer.discover_target(installer.arguments([]), self.home, [live], [])
        self.assertEqual(chosen["executable"], "/venv/bin/python")
        self.assertEqual(chosen["pid"], 123)

    def test_multiple_running_instances_require_selection(self):
        one = dict(self.selected, pid=1)
        two = dict(self.selected, pid=2, config=self.home / "other.cfg")
        with self.assertRaisesRegex(installer.InstallError, "Multiple running"):
            installer.discover_target(installer.arguments([]), self.home, [one, two], [])
        selected = installer.discover_target(installer.arguments(["--config", str(self.config)]), self.home, [one, two], [])
        self.assertEqual(selected["pid"], 1)

    def test_multiple_stopped_checkouts_rejected(self):
        other = self.home / "kalico/klippy"
        other.mkdir(parents=True)
        (other / "klippy.py").write_text("# fixture")
        with self.assertRaisesRegex(installer.InstallError, "Cannot choose"):
            installer.discover_target(installer.arguments([]), self.home, [], [])

    def test_explicit_wrong_service_cannot_bypass_live_instance(self):
        live = dict(self.selected, pid=10, service="klipper.service")
        args = installer.arguments(["--klipper-path", str(self.repo), "--config", str(self.config), "--service", "wrong"])
        with self.assertRaises(installer.InstallError):
            installer.discover_target(args, self.home, [live], [])

    def test_command_parser_extracts_config_socket_and_python(self):
        parsed = installer.command_instance(["/venv/bin/python", str(self.repo / "klippy/klippy.py"),
                    str(self.config), "-l", "/tmp/klippy.log", "-a", "/tmp/klippy.sock"], self.home, "fixture")
        self.assertEqual(parsed["repo"], self.repo)
        self.assertEqual(parsed["socket"], Path("/tmp/klippy.sock").resolve())
        self.assertEqual(parsed["executable"], "/venv/bin/python")

    def test_proc_fixture_extracts_service_without_execution(self):
        proc = self.home / "proc/101"
        proc.mkdir(parents=True)
        argv = ["python", str(self.repo / "klippy/klippy.py"), str(self.config), "-a", str(self.home / "uds")]
        (proc / "cmdline").write_bytes(b"\0".join(v.encode() for v in argv) + b"\0")
        (proc / "cgroup").write_text("0::/system.slice/klipper-two.service\n")
        found = installer.proc_instances(proc.parent)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]["service"], "klipper-two.service")

    def test_plan_activates_included_endstops_and_preserves_saved_block(self):
        changes, messages, sensor = self.plan()
        self.assertEqual(sensor, "beacon")
        axes = changes[self.axes]["data"].decode()
        self.assertIn("endstop_pin: accel_homing:x", axes)
        self.assertIn("endstop_pin: accel_homing:y", axes)
        self.assertIn("homing_retract_dist: 0", axes)
        self.assertIn("# accel_homing previous: diag_pin: ^PB0", axes)
        self.assertIn("driver_SGTHRS: 50", axes)
        self.assertIn("[tmc2209 stepper_z]\ndiag_pin: PB2", axes)
        self.assertIn("homing_speed: 150", axes)
        main = changes[self.config]["data"].decode()
        self.assertTrue(main.endswith(SAVED))
        self.assertIn("M118 Leave this exact macro alone", main)
        self.assertLess(main.index("[include accel_homing.cfg]"), main.index(installer.SAVE_MARKER))
        created = changes[self.config.parent / installer.MANAGED]["data"].decode()
        self.assertIn("#x_threshold: 8.0\n#y_threshold: 8.0", created)
        self.assertNotRegex(created, r"(?m)^x_threshold:")

    def test_generic_default_preserves_speed_text_and_slower_axis(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        speed_line = b"  homing_speed = 050.000 # operator speed\r\n"
        self.axes.write_bytes(AXES.replace("homing_speed: 150", speed_line.decode().rstrip("\r\n"))
                              .replace("\n", "\r\n").encode())
        original = self.axes.read_bytes()
        changes, messages, sensor = self.plan()
        graph = changes[self.axes]["data"]
        self.assertIn(speed_line, graph)
        self.assertIn(b"homing_speed: 35\r\n", graph)
        self.assertNotIn(b"previous: homing_speed", graph)
        self.assertIn("adxl345 toolhead", sensor)
        self.assertFalse(any("Capped" in message for message in messages))
        self.assertEqual(self.axes.read_bytes(), original)
        managed = changes[self.config.parent / installer.MANAGED]["data"]
        self.assertNotIn(b"max_report_travel", managed)
        self.assertNotIn(b"max_startup_travel", managed)

    def test_default_distance_policies_preserve_75_100_and_higher_configured_speeds(self):
        for x, y, acceleration in ((75, 50, 5000), (100, 100, 1500), (500, 35, 1000)):
            with self.subTest(x=x, y=y, acceleration=acceleration):
                self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]")
                                       .replace("max_accel: 5000", "max_accel: " + str(acceleration)) + SAVED)
                self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: " + str(x))
                                     .replace("homing_speed: 35", "homing_speed: " + str(y)))
                before = self.config.read_bytes(), self.axes.read_bytes()
                changes = self.plan()[0]
                self.assertEqual(self.planned_speed(changes, "x"), x)
                self.assertEqual(self.planned_speed(changes, "y"), y)
                self.assertNotIn(b"previous: homing_speed", changes[self.axes]["data"])
                managed = changes[self.config.parent / installer.MANAGED]["data"]
                self.assertNotIn(b"max_report_travel", managed)
                self.assertNotIn(b"max_startup_travel", managed)
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_default_distance_policies_allow_explicit_speed_edit_without_adding_caps(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        changes = self.plan("--homing-speed", "100")[0]
        self.assertEqual(self.planned_speed(changes, "x"), 100)
        self.assertEqual(self.planned_speed(changes, "y"), 100)
        managed = changes[self.config.parent / installer.MANAGED]["data"]
        self.assertNotIn(b"max_report_travel", managed)
        self.assertNotIn(b"max_startup_travel", managed)

    def test_targeted_x_75_update_preserves_y_50_and_saved_model_without_budget_flags(self):
        saved_y = SAVED.replace("#*# x_threshold = 6.5", "#*# y_threshold = 6.5")
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]")
                               .replace("max_accel: 5000", "max_accel: 5360")
                               + "\n[accel_homing]\naccel_chip: adxl345 toolhead\n" + saved_y)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50")
                             .replace("homing_speed: 35", "homing_speed: 50")
                             .replace("endstop_pin: !PA0", "endstop_pin: accel_homing:y")
                             .replace("homing_retract_dist: 3", "homing_retract_dist: 0"))
        config_before = self.config.read_bytes()
        y_before = self.axes.read_bytes().split(b"[stepper_y]", 1)[1].split(b"[tmc2209", 1)[0]
        changes = self.plan("--axes", "x", "--homing-speed", "75", "--replace-existing-links")[0]
        self.assertNotIn(self.config, changes)
        self.assertNotIn(self.config.parent / installer.MANAGED, changes)
        installer.apply_plan({path: state for path, state in changes.items() if state["kind"] != "symlink"}, self.config)
        self.assertEqual(self.config.read_bytes(), config_before)
        self.assertTrue(self.config.read_text().endswith(saved_y))
        self.assertEqual(self.axes.read_bytes().split(b"[stepper_y]", 1)[1].split(b"[tmc2209", 1)[0], y_before)
        self.assertIn(b"homing_speed: 75", self.axes.read_bytes().split(b"[stepper_y]", 1)[0])
        self.assertNotIn(b"max_report_travel", self.config.read_bytes())
        self.assertNotIn(b"max_startup_travel", self.config.read_bytes())

    def test_existing_individual_distance_limit_is_honored_without_adding_the_other(self):
        for option, value, conflict in (("max_report_travel", "1", "report-speed limit 20 mm/s"),
                                        ("max_startup_travel", "3", "3 mm max_startup_travel limit")):
            with self.subTest(option=option):
                self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                                       "\n[accel_homing]\naccel_chip: adxl345 toolhead\n"
                                       + option + ": " + value + "\n" + SAVED)
                self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
                before = self.config.read_bytes()
                with self.assertRaisesRegex(installer.InstallError, conflict):
                    self.plan()
                changes = self.plan("--homing-speed", "20")[0]
                self.assertNotIn(self.config, changes)
                self.assertEqual(self.config.read_bytes(), before)

    def test_absent_distance_policies_still_reject_invalid_timing_before_staging(self):
        settings = ("max_sample_age: 0.004", "max_sample_age: nan", "stream_interval: 0.021",
                    "stream_interval: inf")
        for setting in settings:
            with self.subTest(setting=setting):
                self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                                       "\n[accel_homing]\n" + setting + "\n" + SAVED)
                before = self.config.read_bytes(), self.axes.read_bytes()
                with mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "disable") as disable, \
                     self.assertRaisesRegex(installer.InstallError, "Generic settings require"):
                    self.plan()
                replace.assert_not_called()
                disable.assert_not_called()
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_absent_distance_policies_still_reject_nonfinite_derived_startup_timing(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]")
                               .replace("max_accel: 5000", "max_accel: 1e-308") + SAVED)
        with mock.patch.object(installer.Section, "replace") as replace, \
             mock.patch.object(installer.Section, "disable") as disable, \
             self.assertRaisesRegex(installer.InstallError, "non-finite startup timing/travel"):
            self.plan()
        replace.assert_not_called()
        disable.assert_not_called()
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_absent_distance_policies_still_require_finite_positive_homing_speed(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        for speed in ("nan", "inf", "0", "-1"):
            with self.subTest(speed=speed):
                self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: " + speed))
                with mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "disable") as disable, \
                     self.assertRaisesRegex(installer.InstallError, "finite and positive"):
                    self.plan()
                replace.assert_not_called()
                disable.assert_not_called()

    def test_generic_explicit_configured_limits_refuse_75_before_staging_or_writes(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + TRAVEL_LIMITS + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75")
                             .replace("homing_speed: 35", "homing_speed: 75"))
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        with mock.patch.object(installer.Section, "replace") as replace, \
             mock.patch.object(installer.Section, "disable") as disable, \
             self.assertRaises(installer.InstallError) as caught:
            self.plan()
        replace.assert_not_called()
        disable.assert_not_called()
        for text in ("X configured homing_speed 75 mm/s", "effective speed 75 mm/s",
                     "acceleration 5000 mm/s^2", "report-speed limit 50 mm/s",
                     "4.1625 mm", "3 mm max_startup_travel limit", "No files changed"):
            self.assertIn(text, str(caught.exception))
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_generic_opted_in_limits_refuse_requested_75_and_preserve_current_50(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + TRAVEL_LIMITS + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50"))
        before = self.axes.read_bytes(), self.config.read_bytes()
        with self.assertRaises(installer.InstallError) as caught:
            self.plan("--homing-speed", "75")
        for text in ("requested --homing-speed 75", "current configured homing_speed 50", "4.1625 mm"):
            self.assertIn(text, str(caught.exception))
        self.assertEqual((self.axes.read_bytes(), self.config.read_bytes()), before)
        self.assertFalse((self.config.parent / installer.MANAGED).exists())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_cli_speed_conflict_precedes_idle_checks_writes_and_activation(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + TRAVEL_LIMITS + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        policy = installer.load_motion_policy()
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        with mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
             mock.patch.object(installer, "load_motion_policy", return_value=policy), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             mock.patch.object(installer, "assert_idle") as idle, \
             mock.patch.object(installer, "prepare_restart") as prepare, \
             mock.patch.object(installer, "apply_plan") as apply, \
             mock.patch.object(installer, "activate_install") as activate, \
             contextlib.redirect_stdout(io.StringIO()), \
             self.assertRaisesRegex(installer.InstallError, "4.1625 mm"):
            installer.main([])
        for operation in (idle, prepare, apply, activate):
            operation.assert_not_called()
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_later_selected_axis_policy_conflict_precedes_any_staging(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + TRAVEL_LIMITS + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 35")
                             .replace("homing_speed: 35\nhoming_retract_dist: 3", "homing_speed: 75\nhoming_retract_dist: 3"))
        before = self.axes.read_bytes()
        with mock.patch.object(installer.Section, "replace") as replace, \
             mock.patch.object(installer.Section, "disable") as disable, \
             self.assertRaisesRegex(installer.InstallError, "Y configured homing_speed 75"):
            self.plan()
        replace.assert_not_called()
        disable.assert_not_called()
        self.assertEqual(self.axes.read_bytes(), before)

    def test_default_does_not_insert_an_omitted_homing_speed(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(re.sub(r"(?m)^homing_speed:.*\n", "", AXES))
        changes = self.plan()[0]
        self.assertNotRegex(changes[self.axes]["data"].decode(), r"(?m)^\s*homing_speed\s*[:=]")

    def test_explicit_speed_changes_only_selected_axis(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75")
                             .replace("homing_speed: 35", "homing_speed: 75"))
        changes = self.plan("--axes", "x", "--homing-speed", "40")[0]
        self.assertEqual(self.planned_speed(changes, "x"), 40.0)
        self.assertEqual(self.planned_speed(changes, "y"), 75.0)
        self.assertIn(b"endstop_pin: !PA0", changes[self.axes]["data"])

    def planned_speed(self, changes, axis):
        text = changes[self.axes]["data"].decode().replace("\r\n", "\n")
        match = re.search(r"(?ms)^\[stepper_" + axis + r"\].*?^homing_speed:\s*([\d.eE+-]+)", text)
        return float(match.group(1))

    def test_generic_low_accel_explicit_startup_limit_refuses_without_lowering_speed(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]").replace("max_accel: 5000", "max_accel: 1000")
                               + "\n[accel_homing]\nmax_sample_age: 0.020\nmax_startup_travel: 3\n" + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50"))
        before = self.config.read_bytes(), self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "startup stop request needs 3.65 mm"):
            self.plan()
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_explicit_generic_speed_above_startup_budget_refuses_without_writes(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]").replace("max_accel: 5000", "max_accel: 1000")
                               + "\n[accel_homing]\nmax_sample_age: 0.020\nmax_startup_travel: 3\n" + SAVED)
        before = self.config.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "startup stop request"):
            self.plan("--homing-speed", "50")
        self.assertEqual(self.config.read_bytes(), before)
        self.assertFalse((self.config.parent / installer.MANAGED).exists())
        changes = self.plan("--homing-speed", "40")[0]
        self.assertEqual(self.planned_speed(changes, "x"), 40.0)

    def test_generic_startup_budget_uses_plugin_acceleration_and_zero_override(self):
        policy = installer.load_motion_policy()
        for homing_accel, effective_accel, speed in ((100.0, 100.0, 20.0), (0.0, 5000.0, 50.0)):
            self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                                   "\n[accel_homing]\nmax_sample_age: 0.020\nmax_startup_travel: 3\nhoming_accel: " + str(homing_accel) + "\n" + SAVED)
            self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: " + str(speed))
                                 .replace("homing_speed: 35", "homing_speed: " + str(speed)))
            changes, _, _ = self.plan()
            x = self.planned_speed(changes, "x")
            self.assertEqual(x, speed)
            self.assertLessEqual(policy.motion_startup_request_travel(x, effective_accel, 0.020, 0.005), 3.0)
            self.assertIn(("homing_accel: " + str(homing_accel)).encode(), changes[self.config]["data"])

    def test_generic_velocity_limit_preserves_configured_75(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]").replace("max_velocity: 300", "max_velocity: 40")
                               .replace("max_accel: 5000", "max_accel: 1000") + TRAVEL_LIMITS)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        changes = self.plan()[0]
        self.assertEqual(self.planned_speed(changes, "x"), 75.0)
        self.assertEqual(self.planned_speed(changes, "y"), 35.0)
        # Runtime sees min(rail speed, max_velocity)=40, not the raw rail setting.
        changes = self.plan("--homing-speed", "50")[0]
        self.assertEqual(self.planned_speed(changes, "x"), 50.0)

    def test_configured_75_with_opt_in_startup_limit_uses_timing_and_acceleration(self):
        config = CONFIG.replace("[beacon]", "[adxl345 toolhead]").replace("max_accel: 5000", "max_accel: 5360")
        settings = "\n[accel_homing]\nmax_startup_travel: 3\nhoming_accel: 0\nmax_sample_age: 0.005\nstream_interval: 0.001\n"
        self.config.write_text(config + settings + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75")
                             .replace("homing_speed: 35", "homing_speed: 75"))
        before = self.config.read_bytes(), self.axes.read_bytes()
        changes = self.plan()[0]
        self.assertEqual(self.planned_speed(changes, "x"), 75.0)
        self.assertEqual(self.planned_speed(changes, "y"), 75.0)
        self.assertIn(settings.strip().encode(), changes[self.config]["data"].replace(b"\r\n", b"\n"))
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)
        # Leaving the ordinary plugin cap at 5000 still exceeds 3 mm even
        # with these explicitly provided fast timing settings.
        self.config.write_text(config + settings.replace("homing_accel: 0", "homing_accel: 5000") + SAVED)
        with self.assertRaisesRegex(installer.InstallError, "3.0375 mm"):
            self.plan()

    def test_explicit_policy_overrides_admit_30_without_changing_speed_or_timing(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 30")
                             .replace("homing_speed: 35", "homing_speed: 30"))
        before = self.config.read_bytes(), self.axes.read_bytes()
        changes = self.plan("--max-report-travel", "1.5", "--max-startup-travel", "4.5")[0]
        managed = changes[self.config.parent / installer.MANAGED]["data"]
        self.assertIn(b"max_report_travel: 1.5\nmax_startup_travel: 4.5\n", managed)
        for option in (b"max_sample_age:", b"stream_interval:", b"homing_accel:"):
            self.assertNotIn(option, managed)
        self.assertEqual(self.planned_speed(changes, "x"), 30.0)
        self.assertEqual(self.planned_speed(changes, "y"), 30.0)
        self.assertNotIn(b"previous: homing_speed", changes[self.axes]["data"])
        self.assertIn(b"max_accel: 5000", changes[self.config]["data"])
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_each_distance_policy_can_be_enabled_independently(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 30")
                             .replace("homing_speed: 35", "homing_speed: 30"))
        for flags, conflict in ((("--max-report-travel", "1"), "report-speed limit 20 mm/s"),
                                (("--max-startup-travel", "2"), "2 mm max_startup_travel limit")):
            with self.subTest(flags=flags), self.assertRaisesRegex(installer.InstallError, conflict):
                self.plan(*flags)
        for flags, present, absent in ((("--max-report-travel", "1.5"), b"max_report_travel", b"max_startup_travel"),
                                      (("--max-startup-travel", "4.5"), b"max_startup_travel", b"max_report_travel")):
            with self.subTest(flags=flags):
                changes = self.plan(*flags)[0]
                managed = changes[self.config.parent / installer.MANAGED]["data"]
                self.assertIn(present, managed)
                self.assertNotIn(absent, managed)

    def test_existing_policy_limits_preserved_without_explicit_override(self):
        settings = ("\n[accel_homing]\naccel_chip: adxl345 toolhead\n"
                    "max_sample_age: 0.020\n"
                    "  max_report_travel = 01.500 # chosen reporting limit\n"
                    "max_startup_travel: 4.500 # chosen startup limit\n")
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + settings + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        before = self.config.read_bytes()
        changes = self.plan()[0]
        self.assertNotIn(self.config, changes)
        self.assertEqual(self.config.read_bytes(), before)
        self.assertEqual(self.planned_speed(changes, "x"), 75.0)

    def test_policy_overrides_edit_actual_current_or_legacy_include(self):
        policy_file = self.config.parent / "sensor-policy.cfg"
        for section in ("accel_homing", "beacon_accel_homing"):
            with self.subTest(section=section):
                self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                                       "\n[include sensor-policy.cfg]\n" + SAVED)
                policy_file.write_bytes(("# Preserve this included policy\r\n[" + section + "]\r\n"
                                         "accel_chip: adxl345 toolhead\r\nmax_sample_age: 0.020\r\nmax_report_travel: 1.0\r\n"
                                         "max_startup_travel: 3.0\r\nx_threshold: 6.1\r\n").encode())
                self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
                before = policy_file.read_bytes()
                changes = self.plan("--max-report-travel", "1.5", "--max-startup-travel", "4.5")[0]
                updated = changes[policy_file]["data"]
                self.assertIn(("[" + section + "]\r\n").encode(), updated)
                self.assertIn(b"max_report_travel: 1.5\r\n", updated)
                self.assertIn(b"max_startup_travel: 4.5\r\n", updated)
                self.assertIn(b"x_threshold: 6.1\r\n", updated)
                self.assertNotIn(self.config, changes)
                self.assertNotIn(self.config.parent / installer.MANAGED, changes)
                prefix = b"beacon_accel:x" if section.startswith("beacon_") else b"accel_homing:x"
                self.assertIn(prefix, changes[self.axes]["data"])
                self.assertEqual(policy_file.read_bytes(), before)

    def test_explicit_policy_replaces_invalid_previous_value_before_validation(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[accel_homing]\nmax_sample_age: 0.020\nmax_report_travel: old-invalid\n"
                               "max_startup_travel: nan\n" + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        changes = self.plan("--max-report-travel", "1.5", "--max-startup-travel", "4.5")[0]
        self.assertIn(b"max_report_travel: 1.5", changes[self.config]["data"])
        self.assertIn(b"max_startup_travel: 4.5", changes[self.config]["data"])

    def test_invalid_policy_limits_refuse_before_config_staging_or_writes(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50"))
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        invalid = {"max-report-travel": ("nan", "inf", "-inf", "0", "0.09999", "2.00001"),
                   "max-startup-travel": ("nan", "inf", "-inf", "0", "0.09999", "5.00001")}
        for option, values in invalid.items():
            for value in values:
                with self.subTest(option=option, value=value), \
                     mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "disable") as disable, \
                     self.assertRaisesRegex(installer.InstallError, "Generic settings require"):
                    self.plan("--" + option + "=" + value)
                replace.assert_not_called()
                disable.assert_not_called()
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_policy_override_bounds_and_exact_required_startup_boundary(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 30")
                             .replace("homing_speed: 35", "homing_speed: 30"))
        required = installer.load_motion_policy().motion_startup_request_travel(30.0, 5000.0, 0.050, 0.005)
        for limit in (required, required + 0.0000001):
            with self.subTest(limit=limit):
                changes = self.plan("--max-report-travel", "1.5", "--max-startup-travel", repr(limit))[0]
                text = changes[self.config.parent / installer.MANAGED]["data"].decode()
                persisted = float(re.search(r"(?m)^max_startup_travel:\s*(.+)$", text).group(1))
                self.assertEqual(persisted, limit)
                self.assertGreaterEqual(persisted, required)
        with self.assertRaisesRegex(installer.InstallError, "startup stop request needs 2.43 mm"):
            self.plan("--max-report-travel", "1.5", "--max-startup-travel", repr(required - 0.0000001))
        for report, startup in (("0.1", "0.1"), ("2", "5")):
            with self.subTest(report=report, startup=startup):
                self.plan("--homing-speed", "0.5", "--max-report-travel", report, "--max-startup-travel", startup)

    def test_policy_override_checks_retained_active_axis_without_editing_it(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[accel_homing]\nmax_sample_age: 0.020\nmax_report_travel: 1.5\nmax_startup_travel: 4.5\n" + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50")
                             .replace("homing_speed: 35", "homing_speed: 75")
                             .replace("endstop_pin: !PA0", "endstop_pin: beacon_accel:y"))
        before = self.config.read_bytes(), self.axes.read_bytes()
        for flags in (("--max-report-travel", "1"), ("--max-startup-travel", "3")):
            with self.subTest(flags=flags), \
                 mock.patch.object(installer.Section, "replace") as replace, \
                 mock.patch.object(installer.Section, "disable") as disable, \
                 self.assertRaisesRegex(installer.InstallError, "Y configured homing_speed 75"):
                self.plan("--axes", "x", "--homing-speed", "20", *flags)
            replace.assert_not_called()
            disable.assert_not_called()
        changes = self.plan("--axes", "x", "--max-startup-travel", "4.5")[0]
        self.assertIn(b"endstop_pin: beacon_accel:y", changes[self.axes]["data"])
        self.assertEqual(self.planned_speed(changes, "y"), 75.0)
        self.assertIn(b"homing_retract_dist: 3", changes[self.axes]["data"])
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_shared_policy_checks_retained_second_pass_before_any_staging(self):
        for distances in ("homing_retract_dist: 3\n", "homing_retract_dist: 0\nmin_home_dist: 3\n", ""):
            with self.subTest(distances=distances):
                self.retained_y_policy_fixture(distances + "second_homing_speed: 75")
                before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
                with mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "disable") as disable, \
                     self.assertRaises(installer.InstallError) as caught:
                    self.plan("--axes", "x", "--max-startup-travel", "3")
                for text in ("Y configured second_homing_speed 75", "current configured homing_speed 50",
                             "startup stop request needs 4.1625 mm", "3 mm max_startup_travel limit"):
                    self.assertIn(text, str(caught.exception))
                replace.assert_not_called()
                disable.assert_not_called()
                self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_shared_policy_retained_default_second_passes_are_bounded_by_first(self):
        for options in ("homing_retract_dist: 3", "homing_retract_dist: 3\nuse_sensorless_homing: true",
                        "homing_retract_dist: 0\nmin_home_dist: 3"):
            with self.subTest(options=options):
                self.retained_y_policy_fixture(options)
                original_y = self.axes.read_bytes().split(b"[stepper_y]", 1)[1].split(b"[tmc2209", 1)[0]
                changes = self.plan("--axes", "x", "--max-startup-travel", "3")[0]
                planned_y = changes[self.axes]["data"].split(b"[stepper_y]", 1)[1].split(b"[tmc2209", 1)[0]
                self.assertEqual(planned_y, original_y)
                self.assertNotIn(b"second_homing_speed", planned_y)

    def test_shared_policy_retained_second_speed_requires_numeric_finite_positive_value(self):
        for second in ("bad", "nan", "inf", "0", "-1"):
            with self.subTest(second=second):
                self.retained_y_policy_fixture("homing_retract_dist: 0\nmin_home_dist: 3\nsecond_homing_speed: " + second)
                before = self.config.read_bytes(), self.axes.read_bytes()
                with mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "disable") as disable, \
                     self.assertRaisesRegex(installer.InstallError, "Y second_homing_speed must be"):
                    self.plan("--axes", "x", "--max-startup-travel", "3")
                replace.assert_not_called()
                disable.assert_not_called()
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_shared_policy_checks_retained_rail_acceleration_before_any_staging(self):
        self.retained_y_policy_fixture("homing_retract_dist: 0\nhoming_accel: 3000", speed=75, startup=5)
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        with mock.patch.object(installer.Section, "replace") as replace, \
             mock.patch.object(installer.Section, "disable") as disable, \
             self.assertRaises(installer.InstallError) as caught:
            self.plan("--axes", "x", "--max-startup-travel", "4.5")
        for text in ("Y configured homing_speed 75", "acceleration 3000 mm/s^2",
                     "startup stop request needs 4.5375 mm", "4.5 mm max_startup_travel limit"):
            self.assertIn(text, str(caught.exception))
        replace.assert_not_called()
        disable.assert_not_called()
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_shared_policy_retained_rail_acceleration_requires_numeric_finite_positive_value(self):
        for acceleration in ("bad", "nan", "inf", "0", "-1"):
            with self.subTest(acceleration=acceleration):
                self.retained_y_policy_fixture("homing_retract_dist: 0\nhoming_accel: " + acceleration)
                before = self.config.read_bytes(), self.axes.read_bytes()
                with mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "disable") as disable, \
                     self.assertRaisesRegex(installer.InstallError, "Y retained homing"):
                    self.plan("--axes", "x", "--max-startup-travel", "3")
                replace.assert_not_called()
                disable.assert_not_called()
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_shared_policy_selected_conversion_excludes_disabled_rail_settings(self):
        self.retained_y_policy_fixture("homing_retract_dist: 3")
        self.axes.write_text(self.axes.read_text().replace("homing_speed: 50\nhoming_retract_dist: 5",
                                                         "homing_speed: 50\nsecond_homing_speed: 150\nhoming_retract_dist: 5")
                             .replace("homing_accel: 1000", "homing_accel: 100"))
        changes = self.plan("--axes", "x", "--max-startup-travel", "3")[0]
        planned = changes[self.axes]["data"]
        self.assertIn(b"second_homing_speed: 150", planned)
        self.assertIn(b"# accel_homing previous: homing_accel: 100", planned)
        self.assertIn(b"homing_retract_dist: 0", planned)
        self.assertIn(b"# accel_homing previous: min_home_dist: 10", planned)

    def test_cli_retained_second_pass_conflict_precedes_service_checks_and_writes(self):
        self.retained_y_policy_fixture("homing_retract_dist: 3\nsecond_homing_speed: 75")
        policy = installer.load_motion_policy()
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        with mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
             mock.patch.object(installer, "load_motion_policy", return_value=policy), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             mock.patch.object(installer, "assert_idle") as idle, \
             mock.patch.object(installer, "prepare_restart") as prepare, \
             mock.patch.object(installer, "apply_plan") as apply, \
             mock.patch.object(installer, "activate_install") as activate, \
             contextlib.redirect_stdout(io.StringIO()), \
             self.assertRaisesRegex(installer.InstallError, "Y configured second_homing_speed 75"):
            installer.main(["--axes", "x", "--max-startup-travel", "3"])
        for operation in (idle, prepare, apply, activate):
            operation.assert_not_called()
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_cli_policy_conflict_precedes_service_checks_backups_and_writes(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50"))
        policy = installer.load_motion_policy()
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        for flags in (("--max-report-travel=nan",), ("--max-startup-travel=5.01",)):
            with self.subTest(flags=flags), \
                 mock.patch.object(installer, "discover_target", return_value=self.selected), \
                 mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
                 mock.patch.object(installer, "load_motion_policy", return_value=policy), \
                 mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
                 mock.patch.object(installer, "assert_idle") as idle, \
                 mock.patch.object(installer, "prepare_restart") as prepare, \
                 mock.patch.object(installer, "apply_plan") as apply, \
                 mock.patch.object(installer, "activate_install") as activate, \
                 contextlib.redirect_stdout(io.StringIO()), \
                 self.assertRaisesRegex(installer.InstallError, "Generic settings require"):
                installer.main(list(flags))
            for operation in (idle, prepare, apply, activate):
                operation.assert_not_called()
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_cli_explicit_policy_output_reports_limits_and_preserved_speed(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 30")
                             .replace("homing_speed: 35", "homing_speed: 30"))
        policy, output = installer.load_motion_policy(), io.StringIO()
        before = {path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}
        with mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
             mock.patch.object(installer, "load_motion_policy", return_value=policy), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             mock.patch.object(installer, "assert_idle") as idle, \
             mock.patch.object(installer, "prepare_restart") as prepare, \
             mock.patch.object(installer, "apply_plan") as apply, \
             contextlib.redirect_stdout(output):
            self.assertEqual(installer.main(["--dry-run", "--max-report-travel", "1.5",
                                             "--max-startup-travel", "4.5"]), 0)
        for message in ("Explicit generic policy overrides: max_report_travel=1.5 mm, max_startup_travel=4.5 mm",
                        "nominal request-travel limits are not physical stopping guarantees",
                        "Preserved configured homing_speed: X=30 mm/s, Y=30 mm/s"):
            self.assertIn(message, output.getvalue())
        for operation in (idle, prepare, apply):
            operation.assert_not_called()
        self.assertEqual({path: path.read_bytes() for path in self.home.rglob("*") if path.is_file()}, before)

    def test_explicit_speed_and_policy_output_does_not_claim_preserved_speed(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]")
                               + "\n[accel_homing]\nmax_sample_age: 0.020\n" + SAVED)
        messages = self.plan("--homing-speed", "75", "--max-report-travel", "1.5",
                             "--max-startup-travel", "4.5")[1]
        text = "\n".join(messages)
        self.assertIn("Explicitly selected homing_speed: X=75 mm/s, Y=75 mm/s", text)
        self.assertNotIn("Preserved configured homing_speed", text)

    def test_native_beacon_policy_flags_are_rejected_before_staging(self):
        before = self.config.read_bytes(), self.axes.read_bytes()
        for flag in ("--max-report-travel=1.5", "--max-startup-travel=4.5"):
            with self.subTest(flag=flag), \
                 mock.patch.object(installer.Section, "replace") as replace, \
                 mock.patch.object(installer.Section, "disable") as disable, \
                 self.assertRaisesRegex(installer.InstallError, "not native Beacon"):
                self.plan(flag)
            replace.assert_not_called()
            disable.assert_not_called()
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_policy_flags_cannot_be_silently_ignored_by_restoration_commands(self):
        for mode in (("--uninstall",), ("--rollback", "offline-backup")):
            with self.subTest(mode=mode), contextlib.redirect_stderr(io.StringIO()), \
                 self.assertRaises(SystemExit):
                installer.arguments(list(mode) + ["--max-startup-travel", "4.5"])

    def test_legacy_section_thresholds_and_pin_alias_preserved(self):
        self.config.write_text(CONFIG + "\n[beacon_accel_homing]\nx_threshold: 6.1\ny_threshold: 7.2\n" + SAVED)
        changes, _, _ = self.plan()
        text = changes[self.config]["data"].decode().replace("\r\n", "\n")
        self.assertIn("x_threshold: 6.1\ny_threshold: 7.2", text)
        self.assertNotIn("[include accel_homing.cfg]", text)
        self.assertIn("beacon_accel:x", changes[self.axes]["data"].decode())

    def test_multiple_sensors_requires_full_name(self):
        self.config.write_text(CONFIG + "\n[adxl345 toolhead]\ncs_pin: host:None\n")
        with self.assertRaisesRegex(installer.InstallError, "Expected one configured"):
            self.plan()
        self.assertEqual(self.plan("--accel-chip", "adxl345 toolhead", "--homing-speed", "50")[2], "adxl345 toolhead")

    def test_cartographer_probe_alone_is_not_a_sensor(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[scanner]"))
        with self.assertRaisesRegex(installer.InstallError, "Cartographer/scanner"):
            self.plan()

    def test_custom_g28_macros_still_require_acknowledgement(self):
        macro = "\n[gcode_macro G28]\nrename_existing: G28_BASE\ngcode:\n    SYNC_MOTORS\n    G28_BASE {rawparams}\n"
        self.config.write_bytes((CONFIG + macro + SAVED).encode())
        before = self.config.read_bytes(), self.axes.read_bytes()
        with mock.patch.object(installer.Section, "replace") as replace, \
                mock.patch.object(installer.Section, "disable") as disable, \
                self.assertRaisesRegex(installer.InstallError, "Custom homing path.*gcode_macro g28"):
            self.plan()
        replace.assert_not_called()
        disable.assert_not_called()
        changes, _, _ = self.plan("--acknowledge-homing-override")
        self.assertIn(macro.encode(), changes[self.config]["data"])
        self.assertTrue(changes[self.config]["data"].endswith(SAVED.encode()))
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_standard_homing_override_needs_no_acknowledgement_and_preserves_the_complete_script(self):
        for placement in ("main", "include"):
            with self.subTest(placement=placement):
                override_path = self.config.parent / "normal-homing.cfg"
                if placement == "main":
                    self.config.write_bytes((CONFIG + "\n" + HOMING_OVERRIDE + SAVED).encode())
                else:
                    override_path.write_bytes(HOMING_OVERRIDE.encode())
                    self.config.write_bytes((CONFIG + "\n[include normal-homing.cfg]\n" + SAVED).encode())
                paths = (self.config, self.axes) if placement == "main" else (self.config, self.axes, override_path)
                before = {path: path.read_bytes() for path in paths}
                changes, messages, _ = self.plan()
                if placement == "main":
                    self.assertIn(HOMING_OVERRIDE.encode(), changes[self.config]["data"])
                else:
                    self.assertNotIn(override_path, changes)
                    self.assertIn(b"[include normal-homing.cfg]", changes[self.config]["data"])
                self.assertTrue(changes[self.config]["data"].endswith(SAVED.encode()))
                self.assertTrue(any("Preserved [homing_override]" in message and
                                    "bypass all override G-code" in message for message in messages))
                self.assertEqual({path: path.read_bytes() for path in paths}, before)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_safe_z_home_and_homing_override_conflict_even_after_acknowledgement(self):
        self.config.write_bytes((CONFIG + SAFE_Z_HOME + "\n" + HOMING_OVERRIDE + SAVED).encode())
        before = self.config.read_bytes(), self.axes.read_bytes()
        for flags in ((), ("--acknowledge-homing-override",)):
            with self.subTest(flags=flags), \
                    mock.patch.object(installer.Section, "replace") as replace, \
                    mock.patch.object(installer.Section, "disable") as disable, \
                    self.assertRaisesRegex(installer.InstallError, "cannot be used together"):
                self.plan(*flags)
            replace.assert_not_called()
            disable.assert_not_called()
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_duplicate_homing_override_sections_fail_without_changes(self):
        self.config.write_bytes((CONFIG + HOMING_OVERRIDE + "\n" + HOMING_OVERRIDE + SAVED).encode())
        before = self.config.read_bytes(), self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "exactly one \\[homing_override\\]"):
            self.plan()
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_safe_z_home_with_cartographer_and_accelerometer_needs_no_bypass_flag(self):
        probe_home, saved = self.cartographer_safe_z_fixture()
        original = {path: path.read_bytes() for path in (self.config, self.axes, probe_home)}
        changes, messages, sensor = self.plan()
        self.assertEqual(sensor, "adxl345 toolhead")
        self.assertNotIn(probe_home, changes)
        main = changes[self.config]["data"].decode()
        self.assertIn("[scanner]\nserial: /dev/serial/by-id/fixture-only", main)
        self.assertTrue(main.endswith(saved))
        self.assertIn("[include probe-home.cfg]", main)
        self.assertTrue(any("[safe_z_home]" in message and "z_hop" in message for message in messages))
        for path, content in original.items():
            self.assertEqual(path.read_bytes(), content)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_safe_z_home_does_not_exempt_custom_g28_and_duplicate_sections(self):
        self.config.write_text(CONFIG + SAFE_Z_HOME + "\n[gcode_macro G28]\ngcode:\n    G28\n")
        with self.assertRaisesRegex(installer.InstallError, "Custom homing path"):
            self.plan()
        self.config.write_text(CONFIG + SAFE_Z_HOME + "\n" + SAFE_Z_HOME)
        with self.assertRaisesRegex(installer.InstallError, "exactly one \\[safe_z_home\\]"):
            self.plan()

    def test_incomplete_package_fails_without_changes(self):
        (self.source / installer.MODULES[-1]).unlink()
        with self.assertRaisesRegex(installer.InstallError, "Incomplete package"):
            self.plan()
        self.assertEqual(self.config.read_bytes(), (CONFIG + SAVED).encode())

    def test_duplicate_sections_options_and_recursive_includes_rejected(self):
        self.axes.write_text(AXES + "\n[stepper_x]\nendstop_pin: PA1\n")
        with self.assertRaisesRegex(installer.InstallError, "exactly one"):
            self.plan()
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 150\nhoming_speed: 10"))
        with self.assertRaisesRegex(installer.InstallError, "Duplicate"):
            self.plan()
        self.axes.write_text(AXES + "\n[include printer.cfg]\n")
        with self.assertRaisesRegex(installer.InstallError, "Recursive"):
            self.plan()

    def test_existing_unincluded_config_not_overwritten(self):
        (self.config.parent / installer.MANAGED).write_text("# User file")
        with self.assertRaisesRegex(installer.InstallError, "not included"):
            self.plan()

    def test_conflicting_regular_plugin_file_rejected(self):
        (self.extras / installer.MODULES[0]).write_text("# custom plugin")
        with self.assertRaisesRegex(installer.InstallError, "non-symlink"):
            self.plan()

    def test_transaction_idempotence_and_rollback(self):
        before = {self.config: self.config.read_bytes(), self.axes: self.axes.read_bytes(),
                  self.repo / ".git/info/exclude": (self.repo / ".git/info/exclude").read_bytes()}
        changes = {p: v for p, v in self.plan()[0].items() if v["kind"] != "symlink"}
        backup = installer.apply_plan(changes, self.config)
        manifest = json.loads((backup / "manifest.json").read_text())
        self.assertEqual(manifest["status"], "installed")
        with mock.patch.object(installer, "MODULES", ()):
            self.assertEqual(self.plan()[0], {})
        installer.rollback(backup, selected=self.selected)
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)
        self.assertFalse((self.config.parent / installer.MANAGED).exists())
        self.assertFalse((self.extras / installer.MODULES[0]).is_symlink())

    def test_rollbacks_refuse_later_edits(self):
        changes = {p: v for p, v in self.plan()[0].items() if v["kind"] != "symlink"}
        backup = installer.apply_plan(changes, self.config)
        self.axes.write_text(self.axes.read_text() + "# later user edit\n")
        with self.assertRaisesRegex(installer.InstallError, "changed after"):
            installer.rollback(backup)
        self.assertIn("later user edit", self.axes.read_text())

    def test_failed_write_restores_prior_bytes(self):
        initial = self.config.read_bytes()
        calls = []
        def fail_second(path, value):
            calls.append(path)
            if len(calls) == 2:
                raise OSError("synthetic disk failure")
            installer.write_state(path, value)
        with self.assertRaisesRegex(installer.InstallError, "restored"):
            installer.apply_plan({self.config: {"kind": "file", "data": b"changed"},
                                  self.axes: {"kind": "file", "data": b"changed"}}, self.config, fail_second)
        self.assertEqual(self.config.read_bytes(), initial)
        self.assertEqual(self.axes.read_bytes(), AXES.encode())

    def test_edit_after_plan_fails_before_backup_or_writes(self):
        changes = self.plan()[0]
        self.config.write_text(self.config.read_text() + "# concurrent edit\n")
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            installer.apply_plan(changes, self.config)
        self.assertIn("concurrent edit", self.config.read_text())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_failure_rollback_preserves_concurrent_edit_to_applied_file(self):
        count = []
        def concurrent_failure(path, value):
            count.append(path)
            if len(count) == 2:
                self.config.write_bytes(b"concurrent editor/SAVE_CONFIG data")
                raise OSError("synthetic later write failure")
            installer.write_state(path, value)
        changes = {self.config: {"kind": "file", "data": b"installed A"},
                   self.axes: {"kind": "file", "data": b"installed B"}}
        with self.assertRaisesRegex(installer.InstallError, "rollback needs repair"):
            installer.apply_plan(changes, self.config, concurrent_failure)
        self.assertEqual(self.config.read_bytes(), b"concurrent editor/SAVE_CONFIG data")
        manifest = next((self.config.parent / ".accel_homing_backups").glob("*/manifest.json"))
        self.assertEqual(json.loads(manifest.read_text())["status"], "rollback_incomplete")

    def test_explicit_rollback_guards_restore_and_failure_rollforward(self):
        changes = {self.config: {"kind": "file", "data": b"installed A"},
                   self.axes: {"kind": "file", "data": b"installed B"}}
        backup = installer.apply_plan(changes, self.config)
        real_writer = installer.write_state
        def editing_writer(path, value):
            real_writer(path, value)
            if path == self.axes:
                self.axes.write_bytes(b"new edit during rollback rollforward")
                self.config.write_bytes(b"new edit before next rollback restore")
        with mock.patch.object(installer, "write_state", side_effect=editing_writer):
            with self.assertRaisesRegex(installer.InstallError, "concurrent changes were preserved"):
                installer.rollback(backup)
        self.assertEqual(self.axes.read_bytes(), b"new edit during rollback rollforward")
        self.assertEqual(self.config.read_bytes(), b"new edit before next rollback restore")
        self.assertEqual(json.loads((backup / "manifest.json").read_text())["status"], "rollback_incomplete")

    def test_idle_requires_correct_instance_and_not_printing(self):
        selected = dict(self.selected, pid=123, socket=self.home / "fake-socket")
        def query(path, method, params=None):
            if method == "info":
                return dict(state="ready", config_file=str(self.config), klipper_path=str(self.repo))
            if method == "objects/list":
                return dict(objects=["idle_timeout", "print_stats", "pause_resume"])
            return dict(status={"idle_timeout": {"state": "Ready"}, "print_stats": {"state": "standby"}, "pause_resume": {"is_paused": False}})
        self.assertIn("ready/idle", installer.assert_idle(selected, query, lambda _: None))
        def wrong(path, method, params=None):
            return dict(state="ready", config_file=str(self.home / "wrong.cfg")) if method == "info" else query(path, method, params)
        with self.assertRaisesRegex(installer.InstallError, "different/unknown config"):
            installer.assert_idle(selected, wrong, lambda _: None)
        def printing(path, method, params=None):
            response = query(path, method, params)
            if method == "objects/query":
                response["status"]["print_stats"]["state"] = "printing"
            return response
        with self.assertRaisesRegex(installer.InstallError, "Refusing writes"):
            installer.assert_idle(selected, printing, lambda _: None)

    def test_running_unknown_socket_rejected_stopped_host_allowed(self):
        with self.assertRaisesRegex(installer.InstallError, "no known API socket"):
            installer.assert_idle(dict(self.selected, pid=1), service_state=lambda _: None)
        self.assertIn("stopped", installer.assert_idle(self.selected, service_state=lambda _: None))

    def test_mixed_case_sensor_suffix_preserved(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 ToolHead]"))
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50"))
        changes, _, sensor = self.plan("--accel-chip", "adxl345 ToolHead")
        self.assertEqual(sensor, "adxl345 ToolHead")
        self.assertIn(b"accel_chip: adxl345 ToolHead", changes[self.config.parent / installer.MANAGED]["data"])

    def test_wildcard_include_does_not_get_duplicate_explicit_include(self):
        self.config.write_bytes((CONFIG + "\n[include accel_*.cfg]\n" + SAVED).encode())
        (self.config.parent / "accel_existing.cfg").write_text("# Existing include fixture\n")
        changes, _, _ = self.plan()
        self.assertNotIn(self.config, changes)
        file_changes = {p: v for p, v in changes.items() if v["kind"] != "symlink"}
        installer.apply_plan(file_changes, self.config)
        with mock.patch.object(installer, "MODULES", ()):
            self.assertEqual(self.plan()[0], {})

    def test_cli_dry_run_writes_nothing_and_does_not_query_live_status(self):
        before = sorted(p.relative_to(self.home).as_posix() for p in self.home.rglob("*"))
        with mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             mock.patch.object(installer, "assert_idle", side_effect=AssertionError("must not query")), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(installer.main(["--dry-run"]), 0)
        after = sorted(p.relative_to(self.home).as_posix() for p in self.home.rglob("*"))
        self.assertEqual(before, after)

    def test_real_plugin_symlinks_where_supported(self):
        self.require_symlinks()
        changes = {p: v for p, v in self.plan()[0].items() if v["kind"] == "symlink"}
        backup = installer.apply_plan(changes, self.config)
        self.assertEqual((self.extras / installer.MODULES[0]).resolve(), self.source / installer.MODULES[0])
        installer.rollback(backup)
        self.assertFalse((self.extras / installer.MODULES[0]).is_symlink())


class RestartFixture(unittest.TestCase):
    """All systemctl/sudo/API operations are synthetic; only temp files change."""
    plan = HostFixture.plan

    def setUp(self):
        HostFixture.setUp(self)
        (self.source / "uninstaller.py").write_bytes((DIRECTORY / "uninstaller.py").read_bytes())
        self.selected.update(pid=123, start_time="1000", service="kalico-two.service",
                             service_scope="system", socket=self.home / "klippy.sock",
                             executable=(self.home / "venv/bin/python").as_posix())
        self.processes = [dict(self.selected)]
        executable = self.selected["executable"]
        command = " ".join(shlex.quote(value) for value in (
            executable, (self.repo / "klippy/klippy.py").as_posix(), self.config.as_posix(),
            "-a", self.selected["socket"].as_posix()))
        self.unit = dict(Id=self.selected["service"], LoadState="loaded", ActiveState="active",
                         SubState="running", MainPID="123", InvocationID="old-invocation",
                         ControlGroup="/system.slice/" + self.selected["service"],
                         ExecStart="{ path=%s ; argv[]=%s ; ignore_errors=no ; pid=123 ; }" % (executable, command),
                         WorkingDirectory=self.home.as_posix(), FragmentPath="/offline/kalico-two.service",
                         DropInPaths="", NeedDaemonReload="no", User="fixture", Group="fixture",
                         CanStart="yes", CanStop="yes", RefuseManualStart="no", RefuseManualStop="no")
        self.events, self.commands, self.idle_checks = [], [], 0
        self.states = ["standby"]
        self.auth_return = self.list_return = self.acquire_return = self.restart_return = 0
        self.restart_error, self.after_idle = None, None
        self.info_config = self.config
        self.start_process = True
        self.real_plan = installer.make_plan
        patches = (
            mock.patch.object(installer, "discover_target", side_effect=lambda _: dict(self.selected)),
            mock.patch.object(installer, "__file__", str(self.source / "installer.py")),
            mock.patch.object(installer.os, "geteuid", return_value=1000, create=True),
            mock.patch.object(installer.shutil, "which", side_effect=lambda name: "/offline/" + name),
            mock.patch.object(installer, "proc_instances", side_effect=lambda: list(self.processes)),
            mock.patch.object(installer, "service_instances", side_effect=lambda _: [dict(self.selected)]),
            mock.patch.object(installer, "webhook_query", side_effect=self.query),
            mock.patch.object(installer.subprocess, "run", side_effect=self.command),
            mock.patch.object(installer, "make_plan", side_effect=self.file_plan),
        )
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def file_plan(self, source, selected, args):
        changes, messages, sensor = self.real_plan(source, selected, args)
        # Real config/history transaction, with platform-specific symlinks covered
        # separately by HostFixture and UninstallFixture.
        return {path: change for path, change in changes.items() if change["kind"] != "symlink"}, messages, sensor

    def command(self, command, **kwargs):
        self.commands.append(list(command))
        self.assertNotIn("shell", kwargs)
        self.assertGreater(kwargs["timeout"], 0)
        if command[0] == "/offline/systemctl":
            self.events.append("unit")
            output = (self.unit["ActiveState"] + "\n" if "--property=ActiveState" in command else
                      "\n".join("%s=%s" % pair for pair in self.unit.items()))
            return subprocess.CompletedProcess(command, 0, output, "")
        self.assertEqual(command[0], "/offline/sudo")
        if command[1:] == ["-n", "-v"]:
            self.events.append("auth")
            return subprocess.CompletedProcess(command, self.auth_return, "", "auth unavailable")
        if command[1:] == ["-v"]:
            self.events.append("credentials")
            if self.acquire_return == 0:
                self.auth_return = 0
            return subprocess.CompletedProcess(command, self.acquire_return, "", "credentials denied")
        if command[1:4] == ["-n", "-l", "--"]:
            self.events.append("permission")
            self.assertEqual(command[4:], ["/offline/systemctl", "--system", "--no-ask-password",
                                           "restart", "--", self.selected["service"]])
            return subprocess.CompletedProcess(command, self.list_return, "", "restart denied")
        self.assertEqual(command, ["/offline/sudo", "-n", "--", "/offline/systemctl", "--system",
                                   "--no-ask-password", "restart", "--", self.selected["service"]])
        self.assertEqual(json.loads((self.backup() / "manifest.json").read_text())["status"], "installed")
        self.events.append("restart")
        if self.restart_error:
            raise self.restart_error
        if self.restart_return == 0 and self.start_process:
            self.unit.update(ActiveState="active", SubState="running", MainPID="124", InvocationID="new-invocation")
            self.processes = [dict(self.selected, pid=124, start_time="1001")]
        return subprocess.CompletedProcess(command, self.restart_return, "", "synthetic restart failure")

    def query(self, path, method, params=None):
        self.assertEqual(path, self.selected["socket"])
        if method == "info":
            return dict(state="ready", config_file=str(self.info_config), klipper_path=str(self.repo), process_id=self.selected["pid"])
        if method == "objects/list":
            return dict(objects=["idle_timeout", "print_stats", "pause_resume"])
        self.events.append("idle")
        state = self.states[min(self.idle_checks, len(self.states) - 1)]
        self.idle_checks += 1
        if self.after_idle:
            self.after_idle(self.idle_checks)
        return dict(status={"idle_timeout": {"state": "Ready"}, "print_stats": {"state": state},
                            "pause_resume": {"is_paused": False}})

    def cli(self, *flags):
        output = io.StringIO()
        try:
            with contextlib.redirect_stdout(output):
                return installer.main(list(flags))
        finally:
            self.output = output.getvalue()

    def backup(self):
        return next((self.config.parent / ".accel_homing_backups").iterdir())

    def receipt(self):
        return json.loads((self.backup() / "activation.json").read_text())

    def test_default_install_restarts_exact_instance_after_commit_and_fresh_idle(self):
        self.auth_return = 1  # Listing policy alone must never bypass authentication.
        self.assertEqual(self.cli(), 0)
        self.assertEqual(self.events.count("restart"), 1)
        self.assertEqual(self.events.count("idle"), 2)
        self.assertLess(self.events.index("credentials"), self.events.index("idle"))
        self.assertEqual(self.events[self.events.index("restart") - 1], "idle")
        self.assertEqual(self.receipt()["status"], "restarted")
        self.assertEqual(self.receipt()["pid"], 124)
        self.assertIn("Ready/module behavior", self.output)
        self.assertEqual(json.loads((self.backup() / "manifest.json").read_text())["status"], "installed")

    def test_live_environment_based_unit_uses_process_provenance_without_shell(self):
        self.unit["ExecStart"] = "{ path=%s ; argv[]=%s $KLIPPER_ARGS ; ignore_errors=no ; pid=123 ; }" % ((self.selected["executable"],) * 2)
        self.unit["EnvironmentFiles"] = "/offline/klipper.env (ignore_errors=no)"
        self.assertEqual(self.cli(), 0)
        self.assertEqual(self.receipt()["status"], "restarted")

    def test_unavailable_auth_or_exact_command_permission_refuses_before_idle_and_writes(self):
        for auth, acquire, permission in ((1, 1, 0), (0, 0, 1)):
            with self.subTest(auth=auth, permission=permission):
                self.auth_return, self.acquire_return, self.list_return = auth, acquire, permission
                with self.assertRaises(installer.InstallError):
                    self.cli()
                self.assertNotIn("idle", self.events)
                self.assertNotIn("restart", self.events)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_prewrite_printing_and_failed_plan_or_write_never_restart(self):
        self.states = ["printing"]
        with self.assertRaisesRegex(installer.InstallError, "printing/paused/unknown"):
            self.cli()
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
        self.assertNotIn("restart", self.events)
        self.events.clear()
        with mock.patch.object(installer, "make_plan", side_effect=installer.InstallError("invalid plan")):
            with self.assertRaisesRegex(installer.InstallError, "invalid plan"):
                self.cli()
        self.assertEqual(self.events, [])
        self.states = ["standby"]
        with mock.patch.object(installer, "apply_plan", side_effect=installer.InstallError("disk transaction failed")):
            with self.assertRaisesRegex(installer.InstallError, "disk transaction failed"):
                self.cli()
        self.assertNotIn("restart", self.events)

    def test_print_started_after_install_blocks_restart_and_preserves_installed_receipt(self):
        self.states = ["standby", "printing"]
        with self.assertRaisesRegex(installer.InstallError, "Files are installed.*was blocked"):
            self.cli()
        self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
        self.assertEqual(self.receipt()["status"], "blocked")
        self.assertFalse(self.receipt()["restart_attempted"])
        self.assertNotIn("restart", self.events)

    def test_auth_expiry_after_install_is_noninteractive_and_retains_changes(self):
        self.after_idle = lambda count: setattr(self, "auth_return", 1) if count == 1 else None
        with self.assertRaisesRegex(installer.InstallError, "Files are installed.*blocked"):
            self.cli()
        self.assertEqual(self.receipt()["status"], "blocked")
        self.assertNotIn("credentials", self.events)
        self.assertNotIn("restart", self.events)

    def test_postwrite_socket_identity_or_pause_rejection_preserves_installed_files(self):
        good_query = self.query
        for change in ("config", "pause"):
            with self.subTest(change=change):
                self.idle_checks = 0
                self.info_config = self.config
                self.events.clear()
                self.axes.write_text(self.axes.read_text().replace("homing_retract_dist: 0", "homing_retract_dist: 2"))
                def query(path, method, params=None):
                    response = good_query(path, method, params)
                    if method == "objects/query":
                        if change == "config" and self.idle_checks == 1:
                            self.info_config = self.home / "wrong-socket-config.cfg"
                        elif change == "pause" and self.idle_checks == 2:
                            response["status"]["pause_resume"]["is_paused"] = True
                    return response
                with mock.patch.object(installer, "webhook_query", side_effect=query):
                    with self.assertRaisesRegex(installer.InstallError, "Files are installed.*blocked") as failure:
                        self.cli()
                self.assertNotIn("no files changed", str(failure.exception))
                self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
                self.assertNotIn("restart", self.events)

    def test_pid_reuse_or_unit_change_after_install_blocks_activation(self):
        for change in ("start_time", "definition", "state"):
            with self.subTest(change=change):
                self.events.clear()
                self.idle_checks = 0
                self.processes = [dict(self.selected)]
                self.unit.update(User="fixture", ActiveState="active")
                def change_identity(count):
                    if count == 1:
                        if change == "start_time":
                            self.processes[0]["start_time"] = "reused-pid"
                        elif change == "definition":
                            self.unit["User"] = "changed-user"
                        else:
                            self.unit["ActiveState"] = "deactivating"
                self.after_idle = change_identity
                # Each subcase gets a real additional file transaction.
                self.axes.write_text(self.axes.read_text().replace("homing_retract_dist: 0", "homing_retract_dist: 2"))
                with self.assertRaisesRegex(installer.InstallError, "Files are installed.*blocked"):
                    self.cli()
                self.assertNotIn("restart", self.events)

    def test_restart_failure_or_timeout_keeps_transaction_and_truthful_activation_failure(self):
        self.restart_return = 1
        with self.assertRaisesRegex(installer.InstallError, "Files are installed.*failed"):
            self.cli()
        backup = self.backup()
        manifest = (backup / "manifest.json").read_bytes()
        self.assertEqual(self.receipt()["status"], "failed")
        self.assertTrue(self.receipt()["restart_attempted"])
        self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
        self.restart_error = subprocess.TimeoutExpired(["mocked-sudo"], 45)
        with self.assertRaisesRegex(installer.InstallError, "no automatic rollback"):
            self.cli()  # Unchanged existing installation still attempts activation.
        self.assertEqual((backup / "manifest.json").read_bytes(), manifest)
        self.assertEqual(self.receipt()["status"], "failed")
        self.restart_error, self.restart_return = None, 0
        self.assertEqual(self.cli(), 0)
        self.assertEqual(self.receipt()["status"], "restarted")
        self.assertEqual(list((self.config.parent / ".accel_homing_backups").iterdir()), [backup])

    def test_zero_exit_without_new_active_process_does_not_claim_activation(self):
        self.start_process = False
        with mock.patch.object(installer.time, "monotonic", side_effect=[0, 11]):
            with self.assertRaisesRegex(installer.InstallError, "new matching active host process"):
                self.cli()
        self.assertEqual(self.receipt()["status"], "failed")

    def test_postrestart_wait_handles_type_simple_exec_delay(self):
        plan = installer.prepare_restart(self.selected)
        after = dict(plan["identity"], pid=124, start_time="1001")
        with mock.patch.object(installer, "service_identity", side_effect=[installer.ServiceStarting("exec pending"), after]), \
             mock.patch.object(installer.time, "sleep") as sleep:
            self.assertEqual(installer.restarted_process(self.selected, plan), after)
        sleep.assert_called_once_with(0.1)

    def test_wrong_effective_instance_user_manager_or_pending_reload_refuses_before_writes(self):
        original = dict(self.unit)
        for change in ("config", "socket", "user", "reload", "option", "unknown"):
            with self.subTest(change=change):
                self.unit = dict(original)
                self.selected.update(service_scope="system", service="kalico-two.service")
                if change in ("config", "socket"):
                    path = self.config if change == "config" else self.selected["socket"]
                    self.unit["ExecStart"] = self.unit["ExecStart"].replace(path.as_posix(), (self.home / "wrong").as_posix())
                elif change == "user":
                    self.selected["service_scope"] = "user"
                elif change == "reload":
                    self.unit["NeedDaemonReload"] = "yes"
                elif change == "option":
                    self.selected["service"] = "--wrong.service"
                else:
                    self.selected["service"] = None
                with self.assertRaises(installer.InstallError):
                    self.cli()
                self.assertNotIn("restart", self.events)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_stopped_literal_service_starts_but_stopped_environment_unit_requires_opt_out(self):
        self.selected.update(pid=None, start_time=None)
        self.processes = []
        self.unit.update(ActiveState="inactive", SubState="dead", MainPID="0", InvocationID="")
        literal = self.unit["ExecStart"]
        self.unit["ExecStart"] = "{ path=%s ; argv[]=%s $KLIPPER_ARGS ; ignore_errors=no ; }" % ((self.selected["executable"],) * 2)
        with self.assertRaisesRegex(installer.InstallError, "matching live main process"):
            self.cli()
        self.assertNotIn("restart", self.events)
        self.unit["ExecStart"] = literal
        self.assertEqual(self.cli(), 0)
        self.assertEqual(self.receipt()["status"], "restarted")
        self.assertEqual(self.idle_checks, 0)

    def test_dry_run_no_restart_uninstall_and_rollback_never_dispatch_automatic_activation(self):
        self.assertEqual(self.cli("--dry-run"), 0)
        self.assertEqual(self.events, [])
        self.selected["service_scope"] = "user"
        self.unit["ActiveState"] = "inactive"  # User unit is absent from the system manager.
        self.assertEqual(self.cli("--no-restart"), 0)
        self.assertEqual(self.receipt()["status"], "skipped")
        self.assertIn("systemctl --user restart -- kalico-two.service", self.output)
        self.assertNotIn("sudo systemctl restart", self.output)
        backup = self.backup()
        self.assertEqual(self.cli("--rollback", str(backup)), 0)
        self.assertNotIn("restart", self.events)
        self.assertNotIn("auth", self.events)
        self.assertEqual(self.cli("--no-restart"), 0)
        self.assertEqual(self.cli("--uninstall"), 0)
        self.assertNotIn("restart", self.events)
        self.assertNotIn("auth", self.events)

    def test_malformed_live_status_and_socket_pid_must_not_prove_idle(self):
        good_query = self.query
        for broken in ("print_stats", "pause_resume", "paused_type", "paused_true", "print_paused", "process_id"):
            with self.subTest(broken=broken):
                def query(path, method, params=None):
                    response = good_query(path, method, params)
                    if method == "info" and broken == "process_id":
                        response["process_id"] = 999
                    elif method == "objects/query" and broken != "process_id":
                        if broken == "paused_type":
                            response["status"]["pause_resume"]["is_paused"] = 0
                        elif broken == "paused_true":
                            response["status"]["pause_resume"]["is_paused"] = True
                        elif broken == "print_paused":
                            response["status"]["print_stats"]["state"] = "paused"
                        else:
                            response["status"].pop(broken)
                    return response
                with mock.patch.object(installer, "webhook_query", side_effect=query):
                    with self.assertRaises(installer.InstallError):
                        self.cli()
                self.assertNotIn("restart", self.events)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())


class BootstrapFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / "accel_homing"
        self.source.mkdir()
        for name in bootstrap.FILES:
            (self.source / name).write_bytes((DIRECTORY / name).read_bytes())

    def test_local_wrapper_forwards_exact_arguments_and_return_status_without_downloads(self):
        for status in (0, 17):
            with self.subTest(status=status), \
                    mock.patch.object(bootstrap, "__file__", str(self.source / "bootstrap.py")), \
                    mock.patch.object(bootstrap.subprocess, "run", return_value=mock.Mock(returncode=status)) as run:
                self.assertEqual(bootstrap.main(["--dry-run", "--config", "/fixture path/printer.cfg"]), status)
            run.assert_called_once_with([bootstrap.sys.executable, str(self.source / "installer.py"),
                                         "--dry-run", "--config", "/fixture path/printer.cfg"])

    def test_complete_package_retains_runtime_license_and_notice(self):
        self.assertTrue(set(installer.MODULES).issubset(bootstrap.FILES))
        for name in ("LICENSE", "NOTICE"):
            self.assertEqual((self.source / name).read_bytes(), (DIRECTORY / name).read_bytes())

    def test_lone_downloaded_wrapper_refuses_arbitrary_sibling_installer(self):
        for path in self.source.iterdir():
            if path.name != "installer.py":
                path.unlink()
        with mock.patch.object(bootstrap, "__file__", str(self.source / "bootstrap.py")), \
                mock.patch.object(bootstrap.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "complete local checkout"):
                bootstrap.main(["--dry-run"])
        run.assert_not_called()

    def test_missing_file_refuses_before_dispatch_and_preserves_every_existing_file(self):
        (self.source / "NOTICE").unlink()
        before = {path.name: path.read_bytes() for path in self.source.iterdir()}
        with mock.patch.object(bootstrap, "__file__", str(self.source / "bootstrap.py")), \
                mock.patch.object(bootstrap.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "complete local checkout"):
                bootstrap.main(["--uninstall", "--dry-run"])
        run.assert_not_called()
        self.assertEqual({path.name: path.read_bytes() for path in self.source.iterdir()}, before)

    def test_no_explicit_argv_preserves_process_arguments(self):
        with mock.patch.object(bootstrap, "__file__", str(self.source / "bootstrap.py")), \
                mock.patch.object(bootstrap.sys, "argv", ["bootstrap.py", "--uninstall", "--dry-run"]), \
                mock.patch.object(bootstrap.subprocess, "run", return_value=mock.Mock(returncode=0)) as run:
            self.assertEqual(bootstrap.main(), 0)
        self.assertEqual(run.call_args.args[0][2:], ["--uninstall", "--dry-run"])


class UninstallFixture(unittest.TestCase):
    """Real temp config files, with managed symlinks emulated on Windows too."""
    plan = HostFixture.plan

    def setUp(self):
        HostFixture.setUp(self)
        self.links = {}
        self.real_snapshot, self.real_write = installer.snapshot, installer.write_state
        def snapshot(path):
            path = Path(path)
            return dict(self.links.get(path, {"kind": "absent"})) if self.is_module(path) else self.real_snapshot(path)
        self.snapshot = snapshot
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(installer, "snapshot", side_effect=snapshot).start()
        mock.patch.object(installer, "write_state", side_effect=self.write).start()

    def is_module(self, path):
        return path.parent == self.extras and path.name in installer.MODULES

    def write(self, path, state):
        path = Path(path)
        if self.is_module(path):
            self.links[path] = dict(state)
        else:
            self.real_write(path, state)

    def install(self, release="package", *flags):
        self.source = self.home / release
        self.source.mkdir(exist_ok=True)
        for name in installer.MODULES:
            (self.source / name).write_text("# offline source " + release + "\n")
        changes = self.plan(*flags)[0]
        return installer.apply_plan(changes, self.config, self.write, self.selected)

    def uninstall(self):
        plan = installer.make_uninstall_plan(self.selected)
        backup = installer.apply_uninstall_plan(plan, self.selected, self.write)
        return plan, backup

    def capture(self):
        return {str(path): self.snapshot(path) for path in
                (self.config, self.axes, self.config.parent / installer.MANAGED,
                 self.repo / ".git/info/exclude", *(self.extras / name for name in installer.MODULES))}

    def manifest(self, backup):
        return json.loads((backup / "manifest.json").read_text())

    def _assert_manual_thresholds_preserved_by_install_and_update(self, section):
        values = "threshold: 0.25\nx_threshold: 0.5\ny_threshold: 0.001\n"
        self.config.write_bytes((CONFIG + "\n[" + section + "]\n" + values + SAVED).encode())
        self.install("manual-first")
        installed = self.config.read_bytes()
        self.assertIn(values.encode(), installed)
        self.assertTrue(installed.endswith(SAVED.encode()))
        self.install("manual-update")
        self.assertEqual(self.config.read_bytes(), installed)
        graph = installer.ConfigGraph(self.config)
        plugin = graph.one(section)
        self.assertEqual([plugin.get(name) for name in ("threshold", "x_threshold", "y_threshold")],
                         ["0.25", "0.5", "0.001"])

    def test_manual_values_below_three_survive_current_section_install_and_update(self):
        self._assert_manual_thresholds_preserved_by_install_and_update("accel_homing")

    def test_manual_values_below_three_survive_legacy_section_install_and_update(self):
        self._assert_manual_thresholds_preserved_by_install_and_update("beacon_accel_homing")

    def test_install_and_uninstall_preserve_safe_z_probe_and_saved_config_bytes(self):
        probe_home, _saved = HostFixture.cartographer_safe_z_fixture(self)
        original = self.capture()
        probe_original = probe_home.read_bytes()
        self.install()
        self.assertEqual(probe_home.read_bytes(), probe_original)
        self.assertIn(b"[scanner]", self.config.read_bytes())
        self.uninstall()
        self.assertEqual(self.capture(), original)
        self.assertEqual(probe_home.read_bytes(), probe_original)

    def test_explicit_policy_install_upgrade_and_uninstall_restore_original_state(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 30")
                             .replace("homing_speed: 35", "homing_speed: 30"))
        original = self.capture()
        first = self.install("package", "--max-report-travel", "1.5", "--max-startup-travel", "4.5")
        managed = self.config.parent / installer.MANAGED
        self.assertIn(b"max_report_travel: 1.5\nmax_startup_travel: 4.5\n", managed.read_bytes())
        self.assertEqual(self.axes.read_bytes().count(b"homing_speed: 30"), 2)
        saved = next(entry for entry in self.manifest(first)["changes"] if entry["path"] == str(managed))
        self.assertEqual(saved["before"], {"kind": "absent"})
        self.assertIn(b"max_startup_travel: 4.5", base64.b64decode(saved["after"]["data"]))
        self.install("upgrade-one", "--max-startup-travel", "5")
        self.assertIn(b"max_report_travel: 1.5", managed.read_bytes())
        self.assertIn(b"max_startup_travel: 5.0", managed.read_bytes())
        plan, _ = self.uninstall()
        self.assertEqual(len(plan["history"]), 2)
        self.assertEqual(self.capture(), original)

    def test_included_legacy_policy_override_backups_rollback_and_uninstall(self):
        policy_file = self.config.parent / "sensor-policy.cfg"
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[include sensor-policy.cfg]\n" + SAVED)
        policy_file.write_bytes(b"# Existing included legacy section\r\n[beacon_accel_homing]\r\n"
                                b"accel_chip: adxl345 toolhead\r\nmax_sample_age: 0.020\r\nmax_report_travel: 1.0\r\n"
                                b"max_startup_travel: 3.0\r\nx_threshold: 6.1\r\n")
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        original, original_policy = self.capture(), policy_file.read_bytes()
        first = self.install("package", "--max-report-travel", "1.5", "--max-startup-travel", "4.5")
        saved = next(entry for entry in self.manifest(first)["changes"] if entry["path"] == str(policy_file))
        self.assertEqual(base64.b64decode(saved["before"]["data"]), original_policy)
        installed_policy = policy_file.read_bytes()
        self.assertIn(b"max_startup_travel: 4.5\r\n", installed_policy)
        upgrade = self.install("upgrade-one", "--max-startup-travel", "5")
        installer.rollback(upgrade, selected=self.selected)
        self.assertEqual(policy_file.read_bytes(), installed_policy)
        self.uninstall()
        self.assertEqual(policy_file.read_bytes(), original_policy)
        self.assertEqual(self.capture(), original)

    def test_included_current_policy_restore_preserves_unrelated_later_edits(self):
        policy_file = self.config.parent / "sensor-policy.cfg"
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[include sensor-policy.cfg]\n" + SAVED)
        original_policy = (b"# Existing current section\n[accel_homing]\n"
                           b"accel_chip: adxl345 toolhead\nmax_sample_age: 0.020\nmax_report_travel: 1.0\n"
                           b"max_startup_travel: 3.0\nx_threshold: 6.1\n")
        policy_file.write_bytes(original_policy)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        self.install("package", "--max-report-travel", "1.5", "--max-startup-travel", "4.5")
        policy_file.write_bytes(policy_file.read_bytes().replace(b"x_threshold: 6.1", b"x_threshold: 6.4"))
        self.uninstall()
        self.assertEqual(policy_file.read_bytes(), original_policy.replace(b"x_threshold: 6.1", b"x_threshold: 6.4"))

    def test_policy_history_rejects_unexpected_options_and_invalid_values_without_writes(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[accel_homing]\naccel_chip: adxl345 toolhead\n"
                               "max_sample_age: 0.020\nmax_report_travel: 1.0\nmax_startup_travel: 3.0\n" + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        first = self.install("package", "--max-report-travel", "1.5", "--max-startup-travel", "4.5")
        original_manifest = (first / "manifest.json").read_bytes()
        original_config = self.config.read_bytes()
        for option, value in (("max_report_travel", "nan"), ("max_report_travel", "2.0001"),
                              ("max_startup_travel", "inf"), ("max_startup_travel", "0.0999"),
                              ("max_startup_travel", "5.0001"), ("unexpected_option", "1")):
            with self.subTest(option=option, value=value):
                manifest = json.loads(original_manifest)
                entry = next(item for item in manifest["changes"] if item["path"] == str(self.config))
                altered = re.sub(rb"(?m)^" + option.encode() + rb": [^\r\n]+",
                                 (option + ": " + value).encode(), original_config)
                if option == "unexpected_option":
                    altered = re.sub(rb"(?m)^(max_startup_travel: 4\.5)(\r?\n)",
                                     rb"\1\2unexpected_option: 1\2", original_config)
                self.assertNotEqual(altered, original_config)
                entry["after"]["data"] = base64.b64encode(altered).decode("ascii")
                (first / "manifest.json").write_text(json.dumps(manifest))
                self.config.write_bytes(altered)
                current = self.capture()
                backups = set((self.config.parent / ".accel_homing_backups").iterdir())
                with self.assertRaisesRegex(installer.InstallError, "Unrecognized max_|outside installer ownership"):
                    installer.make_uninstall_plan(self.selected)
                self.assertEqual(self.capture(), current)
                self.assertEqual(set((self.config.parent / ".accel_homing_backups").iterdir()), backups)
        (first / "manifest.json").write_bytes(original_manifest)
        self.config.write_bytes(original_config)

    def test_uninstall_preserves_later_safe_z_settings_and_probe_model_edits(self):
        probe_home, _saved = HostFixture.cartographer_safe_z_fixture(self)
        self.install()
        edited_probe = probe_home.read_bytes().replace(b"z_hop: 15", b"z_hop: 20")
        probe_home.write_bytes(edited_probe)
        self.config.write_bytes(self.config.read_bytes().replace(b"model_coef = 1.234", b"model_coef = 9.876"))
        self.uninstall()
        self.assertEqual(probe_home.read_bytes(), edited_probe)
        self.assertIn(b"[scanner model default]", self.config.read_bytes())
        self.assertIn(b"model_coef = 9.876", self.config.read_bytes())
        self.assertNotIn(b"[include accel_homing.cfg]", self.config.read_bytes())

    def test_full_history_two_upgrades_preserves_unrelated_edits_and_is_idempotent(self):
        original = self.capture()
        first = self.install()
        self.axes.write_text(self.axes.read_text().replace("run_current: 1.2", "run_current: 1.25"))
        second = self.install("upgrade-one")
        self.config.write_text(self.config.read_text().replace("Keep my printer settings", "Keep edited printer settings"))
        third = self.install("upgrade-two")
        plan, receipt = self.uninstall()
        self.assertEqual(len(plan["history"]), 3)
        self.assertIn("run_current: 1.25", self.axes.read_text())
        self.assertIn("Keep edited printer settings", self.config.read_text())
        self.assertIn("endstop_pin: tmc2209_stepper_x:virtual_endstop", self.axes.read_text())
        self.assertIn("diag_pin: ^PB0", self.axes.read_text())
        self.assertIn("home_current: 0.6", self.axes.read_text())
        self.assertFalse((self.config.parent / installer.MANAGED).exists())
        self.assertEqual(self.capture()[str(self.repo / ".git/info/exclude")], original[str(self.repo / ".git/info/exclude")])
        self.assertTrue(all(value["kind"] == "absent" for value in self.links.values()))
        self.assertEqual(self.manifest(receipt)["operation"], "uninstall")
        self.assertTrue(all(self.manifest(item)["status"] == "installed" for item in (first, second, third)))
        again, no_backup = self.uninstall()
        self.assertEqual(again["entries"], [])
        self.assertIsNone(no_backup)

    def test_routine_calibration_restores_original_plugin_model_and_preserves_other_saved_data(self):
        self.install()
        text = self.config.read_text().replace("#*# x_threshold = 6.5", "#*# x_threshold = 9.2")
        text += ("#*# [accel_homing_model new_run]\n#*# y_threshold = 7.1\n"
                 "#*# [bed_mesh default]\n#*# points = 0.1, 0.2\n")
        text = text.replace("model_coef = 1.234", "model_coef = 9.876")
        self.config.write_text(text)
        self.uninstall()
        restored = self.config.read_text()
        self.assertIn("#*# x_threshold = 6.5", restored)
        self.assertNotIn("accel_homing_model new_run", restored)
        self.assertIn("model_coef = 9.876", restored)
        self.assertIn("#*# [bed_mesh default]\n#*# points = 0.1, 0.2\n", restored)

    def test_composed_config_upgrades_restore_both_axes_and_nested_speed_edits(self):
        self.install("package", "--axes", "x", "--homing-speed", "80")
        self.axes.write_bytes(self.axes.read_text().replace("run_current: 1.2", "run_current: 1.25").encode())
        self.install("upgrade-one", "--axes", "y", "--homing-speed", "60")
        self.install("upgrade-two", "--axes", "x", "--homing-speed", "90")
        plan, _ = self.uninstall()
        self.assertEqual(len(plan["history"]), 3)
        expected = AXES.replace("run_current: 1.2", "run_current: 1.25")
        self.assertEqual(self.axes.read_text(), expected)
        self.assertEqual(self.config.read_bytes(), (CONFIG + SAVED).encode())

    def test_explicit_75_to_50_upgrade_keeps_50_and_uninstall_restores_recorded_75(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75"))
        original = self.capture()
        first = self.install("package", "--axes", "x", "--homing-speed", "50")
        installed = self.axes.read_bytes()
        self.assertIn(b"homing_speed: 50", installed)
        before = next(entry["before"] for entry in self.manifest(first)["changes"]
                      if entry["path"] == str(self.axes))
        self.assertIn(b"homing_speed: 75", base64.b64decode(before["data"]))
        self.install("upgrade-one", "--axes", "x")
        self.assertEqual(self.axes.read_bytes(), installed)
        self.uninstall()
        self.assertEqual(self.capture(), original)

    def test_calibration_when_originally_no_saved_block_removes_only_plugin_models(self):
        self.config.write_text(CONFIG)
        self.install()
        self.config.write_text(self.config.read_text() + SAVED.replace("#*# [beacon model default]\n#*# model_coef = 1.234\n", ""))
        self.uninstall()
        self.assertEqual(self.config.read_text(), CONFIG)

    def test_generated_config_tuning_is_owned_but_other_sections_fail_closed(self):
        self.install()
        managed = self.config.parent / installer.MANAGED
        managed.write_text(managed.read_text().replace("#x_threshold: 8.0", "x_threshold: 7.2"))
        before = self.capture()
        managed.write_text(managed.read_text() + "\n[gcode_macro MY_MACRO]\ngcode: M118 mine\n")
        with self.assertRaisesRegex(installer.InstallError, "unowned sections"):
            installer.make_uninstall_plan(self.selected)
        self.assertIn("MY_MACRO", managed.read_text())
        managed.write_bytes(base64.b64decode(before[str(managed)]["data"]))
        self.uninstall()
        self.assertFalse(managed.exists())

    def test_preexisting_legacy_config_links_models_and_commented_active_models_restored(self):
        active_model = "\n[accel_homing_model old]\nx_threshold: 6.1\nx_successes: 10\n"
        self.config.write_text(CONFIG + "\n[beacon_accel_homing]\nx_threshold: 6.2\ny_threshold: 6.3\n" + active_model + SAVED)
        old_config = self.config.read_bytes()
        original_source = self.home / "original-legacy-source"
        original_source.mkdir()
        for name in installer.MODULES:
            (original_source / name).write_text("# Original legacy module\n")
            self.links[self.extras / name] = {"kind": "symlink", "target": str(original_source / name)}
        original_links = {path: dict(state) for path, state in self.links.items()}
        self.install()
        self.config.write_text(self.config.read_text().replace("x_threshold: 6.1", "#x_threshold: 6.1").replace("x_successes: 10", "#x_successes: 10") +
                               "#*# [accel_homing_model old]\n#*# x_threshold = 8.5\n#*# x_successes = 10\n")
        self.uninstall()
        self.assertEqual(self.config.read_bytes(), old_config)
        self.assertEqual(self.links, original_links)

    def test_managed_line_edit_or_changed_section_refuses_before_writes(self):
        self.install()
        for old, new in (("endstop_pin: accel_homing:x", "endstop_pin: !PA4"),
                         ("[stepper_x]", "[stepper_edited]")):
            original = self.axes.read_text()
            self.axes.write_text(original.replace(old, new))
            current = self.capture()
            with self.assertRaisesRegex(installer.InstallError, "Managed lines|Managed section"):
                installer.make_uninstall_plan(self.selected)
            self.assertEqual(self.capture(), current)
            self.axes.write_text(original)

    def test_real_save_config_normalizes_crlf_and_separator_blank_lines(self):
        original = CONFIG + "\n[beacon_accel_homing]\nx_threshold: 6.2\n" + (
            "\n[accel_homing_model old]\nx_threshold: 6.1\nx_successes: 10\n") + SAVED
        self.config.write_bytes(original.replace("\n", "\r\n").encode())
        self.install()
        active = self.config.read_text().split(installer.SAVE_MARKER)[0].rstrip()
        active = active.replace("x_threshold: 6.1", "#x_threshold: 6.1").replace("x_successes: 10", "#x_successes: 10")
        saved = SAVED.lstrip("\n") + "#*# [accel_homing_model old]\n#*# x_threshold = 8.5\n#*# x_successes = 10\n"
        self.config.write_bytes((active + "\n" + saved).encode())
        self.uninstall()
        text = self.config.read_bytes().decode().replace("\r\n", "\n")
        self.assertIn("[accel_homing_model old]\nx_threshold: 6.1\nx_successes: 10\n", text)
        self.assertNotIn("#*# [accel_homing_model old]", text)
        self.assertNotIn("accel_chip:", text)

    def test_new_active_model_or_plugin_section_and_broken_restored_include_refuse(self):
        self.install()
        current = self.config.read_bytes()
        for added in ("\n[accel_homing_model manual]\nx_threshold: 6.1\n",
                      "\n[beacon_accel_homing]\nx_threshold: 6.2\n",
                      "\n[include accel_*.cfg]\n"):
            text = current.decode().replace(installer.SAVE_MARKER, added + installer.SAVE_MARKER)
            self.config.write_bytes(text.encode())
            before = self.capture()
            with self.assertRaisesRegex(installer.InstallError, "New/unowned active|Missing include"):
                installer.make_uninstall_plan(self.selected)
            self.assertEqual(self.capture(), before)
        self.config.write_bytes(current)

    def test_missing_original_link_source_refuses_but_deleted_intermediate_release_does_not(self):
        for name in installer.MODULES:
            self.links[self.extras / name] = {"kind": "symlink", "target": "../../../legacy/" + name}
        self.install()
        current = self.capture()
        with self.assertRaisesRegex(installer.InstallError, "Original plugin source is missing"):
            installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.capture(), current)
        # Recover the original source and use two later releases; intermediate
        # source deletion does not matter when its link is absent in the result.
        legacy = self.repo / "klippy/extras/../../../legacy"
        legacy.mkdir(parents=True, exist_ok=True)
        for name in installer.MODULES:
            (legacy / name).write_text("# recovered original\n")
        self.install("upgrade")
        shutil.rmtree(self.home / "package")
        self.uninstall()
        self.assertTrue(all(state.get("target", "").startswith("../../../legacy/") for state in self.links.values()))

    def test_unchanged_included_context_is_rechecked_before_uninstall_mutation(self):
        untouched = self.config.parent / "unchanged.cfg"
        untouched.write_text("[gcode_macro USER_MACRO]\ngcode: M118 mine\n")
        self.config.write_bytes((CONFIG + "\n[include unchanged.cfg]\n" + SAVED).encode())
        self.install()
        plan = installer.make_uninstall_plan(self.selected)
        self.assertNotIn(str(untouched), {entry["path"] for entry in plan["entries"]})
        untouched.write_text("[accel_homing_model new_unreviewed]\nx_threshold: 7.1\n")
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            installer.apply_uninstall_plan(plan, self.selected, self.write)

    def test_changed_link_refuses_and_missing_history_fails_before_writes(self):
        first = self.install()
        second = self.install("upgrade")
        self.links[self.extras / installer.MODULES[0]]["target"] = "/somewhere/manual.py"
        current = self.capture()
        with self.assertRaisesRegex(installer.InstallError, "Managed link"):
            installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.capture(), current)
        shutil.rmtree(first)
        with self.assertRaisesRegex(installer.InstallError, "gapped"):
            installer.make_uninstall_plan(self.selected)
        self.assertTrue(second.is_dir())

    def test_uninstall_wrong_checkout_unknown_path_and_edited_manifest_refuse(self):
        first = self.install()
        with self.assertRaisesRegex(installer.InstallError, "different Klipper checkout"):
            installer.make_uninstall_plan(dict(self.selected, repo=self.home / "other-klipper"))
        manifest = self.manifest(first)
        manifest["changes"][0]["path"] = str(self.home / "unrelated.cfg")
        (first / "manifest.json").write_text(json.dumps(manifest))
        current = self.capture()
        with self.assertRaisesRegex(installer.InstallError, "outside selected instance"):
            installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.capture(), current)

    def test_v2_unchanged_main_baseline_handles_wildcard_calibration_and_v1_refuses_unknown_models(self):
        self.config.write_text(CONFIG + "\n[include accel_*.cfg]\n" + SAVED)
        (self.config.parent / "accel_existing.cfg").write_text("# Original include\n")
        first = self.install()
        self.assertNotIn(str(self.config), {entry["path"] for entry in self.manifest(first)["changes"]})
        self.config.write_text(self.config.read_text() + "#*# [accel_homing_model later]\n#*# x_threshold = 7.1\n")
        plan = installer.make_uninstall_plan(self.selected)
        self.assertTrue(plan["entries"])
        manifest = self.manifest(first)
        for key in ("context", "repo", "parent", "previous_uninstall", "operation"):
            manifest.pop(key, None)
        manifest["version"] = 1
        (first / "manifest.json").write_text(json.dumps(manifest))
        current = self.capture()
        with self.assertRaisesRegex(installer.InstallError, "no original main-config/model baseline"):
            installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.capture(), current)

    def test_v1_complete_install_and_upgrade_restores_original_state(self):
        before = self.capture()
        for name in ("package", "upgrade"):
            directory = self.install(name)
            manifest = self.manifest(directory)
            for key in ("context", "repo", "parent", "previous_uninstall", "operation"):
                manifest.pop(key, None)
            manifest["version"] = 1
            (directory / "manifest.json").write_text(json.dumps(manifest))
        self.uninstall()
        self.assertEqual(self.capture(), before)

    def test_uninstall_failure_recovers_entire_active_state_and_retry_works(self):
        self.install()
        self.install("upgrade")
        before = self.capture()
        plan = installer.make_uninstall_plan(self.selected)
        calls = []
        def fail_second(path, state):
            calls.append(path)
            if len(calls) == 2:
                raise OSError("offline restore failure")
            self.write(path, state)
        with self.assertRaisesRegex(installer.InstallError, "Uninstall failed; applied changes restored"):
            installer.apply_uninstall_plan(plan, self.selected, fail_second)
        self.assertEqual(self.capture(), before)
        failed = sorted((self.config.parent / ".accel_homing_backups").iterdir())[-1]
        self.assertEqual(self.manifest(failed)["status"], "rolled_back_after_failure")
        self.uninstall()

    def test_partial_failure_and_concurrent_edit_report_recovery_without_overwrite(self):
        self.install()
        plan = installer.make_uninstall_plan(self.selected)
        first_path = Path(plan["entries"][0]["path"])
        calls = []
        def fail(path, state):
            calls.append(path)
            if len(calls) == 2:
                if self.is_module(first_path):
                    self.links[first_path] = {"kind": "symlink", "target": "/concurrent/manual.py"}
                else:
                    first_path.write_text("# concurrent edit during failed uninstall\n")
                raise OSError("offline restore failure")
            self.write(path, state)
        with self.assertRaisesRegex(installer.InstallError, "rollback needs repair"):
            installer.apply_uninstall_plan(plan, self.selected, fail)
        failed = sorted((self.config.parent / ".accel_homing_backups").iterdir())[-1]
        self.assertEqual(self.manifest(failed)["status"], "rollback_incomplete")
        with self.assertRaisesRegex(installer.InstallError, "Incomplete/unknown transaction"):
            installer.make_uninstall_plan(self.selected)

    def test_successful_uninstall_can_be_precisely_rolled_back_then_uninstalled_again(self):
        self.install()
        self.install("upgrade")
        installed = self.capture()
        _, backup = self.uninstall()
        installer.rollback(backup, selected=self.selected)
        self.assertEqual(self.capture(), installed)
        self.assertEqual(len(installer.make_uninstall_plan(self.selected)["history"]), 2)
        self.uninstall()

    def test_new_install_after_uninstall_has_new_baseline(self):
        self.install()
        self.uninstall()
        self.axes.write_text(self.axes.read_text().replace("run_current: 1.2", "run_current: 1.3"))
        self.install("new-installation")
        plan, _ = self.uninstall()
        self.assertEqual(len(plan["history"]), 1)
        self.assertIn("run_current: 1.3", self.axes.read_text())

    def test_uninstall_plan_checks_concurrent_config_history_and_instance_before_writes(self):
        self.install()
        plan = installer.make_uninstall_plan(self.selected)
        with self.assertRaisesRegex(installer.InstallError, "different instance"):
            installer.apply_uninstall_plan(plan, dict(self.selected, config=self.home / "wrong.cfg"), self.write)
        self.axes.write_text(self.axes.read_text() + "# concurrent edit\n")
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            installer.apply_uninstall_plan(plan, self.selected, self.write)

    def test_cli_uninstall_dry_run_and_idle_failure_do_not_write_or_install(self):
        self.install()
        before = self.capture()
        with mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             mock.patch.object(installer, "assert_idle", side_effect=installer.InstallError("printing")) as idle, \
             mock.patch.object(installer, "make_plan", side_effect=AssertionError("must never install")), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(installer.main(["--uninstall", "--dry-run"]), 0)
            idle.assert_not_called()
            with self.assertRaisesRegex(installer.InstallError, "printing"):
                installer.main(["--uninstall"])
        self.assertEqual(self.capture(), before)

    def test_other_shared_checkout_plugin_consumer_refuses_before_writes(self):
        self.install()
        peer_config = self.home / "printer_b/config/printer.cfg"
        peer_config.parent.mkdir(parents=True)
        peer_config.write_text("[printer]\nkinematics: corexy\n[accel_homing]\naccel_chip: beacon\n")
        peer = dict(self.selected, config=peer_config, pid=456, service="klipper-b.service", origin="offline peer")
        before = self.capture()
        with mock.patch.object(installer, "proc_instances", return_value=[peer]), \
             mock.patch.object(installer, "service_instances", return_value=[]):
            with self.assertRaisesRegex(installer.InstallError, "shared checkout module links.*printer_b"):
                installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.capture(), before)
        peer_config.write_text("[include missing-config.cfg]\n")
        with mock.patch.object(installer, "proc_instances", return_value=[]), \
             mock.patch.object(installer, "service_instances", return_value=[dict(peer, pid=None)]):
            with self.assertRaisesRegex(installer.InstallError, "cannot resolve this shared checkout consumer"):
                installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.capture(), before)

    def test_peer_without_plugin_is_safe_but_new_consumer_after_planning_refuses(self):
        self.install()
        peer_config = self.home / "plain-printer.cfg"
        peer_config.write_text("[printer]\nkinematics: corexy\n")
        peer = dict(self.selected, config=peer_config, pid=None, service="klipper-b.service", origin="offline service")
        with mock.patch.object(installer, "proc_instances", return_value=[]), \
             mock.patch.object(installer, "service_instances", return_value=[peer]):
            plan = installer.make_uninstall_plan(self.selected)
            peer_config.write_text("[accel_homing_model old]\nx_threshold: 6.1\n")
            before = self.capture()
            with self.assertRaisesRegex(installer.InstallError, "shared checkout module links"):
                installer.apply_uninstall_plan(plan, self.selected, self.write)
            self.assertEqual(self.capture(), before)

    def test_pristine_host_without_any_history_is_already_uninstalled(self):
        plan, backup = self.uninstall()
        self.assertEqual(plan["entries"], [])
        self.assertIsNone(backup)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())


if __name__ == "__main__":
    unittest.main()
