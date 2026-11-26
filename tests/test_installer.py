"""Offline synthetic host/config fixtures. Never contact a printer or a service."""
import base64
import configparser
import contextlib
import glob
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


def klipper_parse(path, parser=None):
    """Read a config like Klipper: comments stripped, includes split the buffer."""
    parser = parser or configparser.RawConfigParser(strict=False)
    lines = []
    def flush():
        if lines:
            parser.read_string("\n".join(lines), str(path))
            del lines[:]
    for line in Path(path).read_text().replace("\r\n", "\n").split("\n"):
        line = line.split("#", 1)[0]
        match = configparser.RawConfigParser.SECTCRE.match(line)
        header = match and match.group("header")
        if header and header.startswith("include "):
            flush()
            for found in sorted(glob.glob(os.path.join(os.path.dirname(str(path)), header[8:].strip()))):
                klipper_parse(found, parser)
        else:
            lines.append(line)
    flush()
    return parser


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

    def gentle_homing_fixture(self, sensor="adxl345 toolhead", acceleration="5000"):
        plugin = "\n[accel_homing]\naccel_chip: " + sensor + "\nmax_sample_age: 0.020\n"
        if acceleration is not None:
            plugin += "homing_accel: " + acceleration + "\n"
        self.config.write_bytes((CONFIG.replace("[beacon]", "[" + sensor + "]") + plugin + SAVED).encode())
        self.axes.write_bytes(AXES.replace("tmc2209_stepper_x:virtual_endstop", "accel_homing:x")
                             .replace("endstop_pin: !PA0", "endstop_pin: accel_homing:y")
                             .replace("homing_speed: 150", "homing_speed: 125")
                             .replace("homing_speed: 35", "homing_speed: 25")
                             .replace("homing_retract_dist: 5", "homing_retract_dist: 0")
                             .replace("homing_retract_dist: 3", "homing_retract_dist: 0")
                             .replace("homing_accel: 1000\n", "")
                             .replace("min_home_dist: 10", "min_home_dist: 0").encode())

    def test_gentle_homing_caps_installed_rails_and_preserves_hardware_and_saved_models(self):
        self.gentle_homing_fixture()
        before = self.config.read_bytes(), self.axes.read_bytes()
        changes, messages, sensor = self.plan("--gentle-homing")
        self.assertEqual(sensor, "adxl345 toolhead")
        axes = changes[self.axes]["data"].decode()
        self.assertEqual(self.planned_speed(changes, "x"), 40.)
        self.assertEqual(self.planned_speed(changes, "y"), 25.)
        for line in ("home_current: 0.6", "diag_pin: ^PB0", "diag0_pin: PB1",
                     "run_current: 1.2", "min_home_dist: 0", "endstop_pin: accel_homing:x"):
            self.assertIn("\n" + line + "\n", axes)
            self.assertNotIn("# " + line, axes)
        self.assertEqual(axes.count("homing_retract_dist: 0"), 2)
        main = changes[self.config]["data"].decode()
        self.assertIn("gentle_homing: True", main)
        self.assertIn("\nhoming_accel: 1000\n", main)
        self.assertTrue(main.endswith(SAVED))
        self.assertIn("Recalibrate all selected axes", "\n".join(messages))
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_gentle_homing_caps_never_raise_lower_positive_acceleration(self):
        for original, expected in ((None, "1000"), ("0", "1000"), ("500", "500"),
                                   ("1000", "1000"), ("5000", "1000")):
            with self.subTest(original=original):
                self.gentle_homing_fixture(acceleration=original)
                changes, _, _ = self.plan("--gentle-homing")
                self.assertIn("\nhoming_accel: " + expected + "\n", changes[self.config]["data"].decode())

    def test_gentle_homing_requires_exact_existing_plugin_axis_scope(self):
        self.gentle_homing_fixture()
        before = self.config.read_bytes(), self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "all and only existing plugin-owned axes"):
            self.plan("--gentle-homing", "--axes", "x")
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)
        self.axes.write_text(self.axes.read_text().replace("endstop_pin: accel_homing:y", "endstop_pin: !PA0"))
        before_y = self.axes.read_text().split("[stepper_y]", 1)[1]
        with self.assertRaisesRegex(installer.InstallError, "cannot convert physical endstops"):
            self.plan("--gentle-homing")
        changes, _, _ = self.plan("--gentle-homing", "--axes", "x")
        self.assertEqual(changes[self.axes]["data"].decode().replace("\r\n", "\n").split("[stepper_y]", 1)[1], before_y)

    def test_gentle_homing_rejects_fresh_installs_and_missing_owned_rails(self):
        with self.assertRaisesRegex(installer.InstallError, "requires an existing"):
            self.plan("--gentle-homing")
        self.gentle_homing_fixture()
        self.axes.write_text(AXES)
        with self.assertRaisesRegex(installer.InstallError, "cannot convert physical endstops"):
            self.plan("--gentle-homing")

    def test_gentle_homing_rejects_second_pass_and_rail_acceleration_without_mutation(self):
        for old, new, message in (
                ("homing_retract_dist: 0", "homing_retract_dist: 3", "active second pass"),
                ("min_home_dist: 0", "min_home_dist: 3", "active second pass"),
                ("min_home_dist: 0", "min_home_dist: 0\nhoming_accel: 5000", "rail homing_accel")):
            with self.subTest(setting=new):
                self.gentle_homing_fixture()
                self.axes.write_text(self.axes.read_text().replace(old, new, 1))
                before = self.config.read_bytes(), self.axes.read_bytes()
                with self.assertRaisesRegex(installer.InstallError, message):
                    self.plan("--gentle-homing")
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_gentle_homing_validates_policies_using_both_resulting_caps(self):
        self.gentle_homing_fixture()
        changes, _, _ = self.plan("--gentle-homing", "--max-report-travel", "1", "--max-startup-travel", "2.8")
        self.assertEqual(self.planned_speed(changes, "x"), 40.)
        before = self.config.read_bytes(), self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "acceleration 1000.*2.72 mm"):
            self.plan("--gentle-homing", "--max-report-travel", "1", "--max-startup-travel", "2.5")
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_gentle_homing_rejects_invalid_configured_motion_values(self):
        for option, value in (("homing_speed", "nan"), ("homing_speed", "inf"), ("homing_speed", "0"),
                              ("homing_accel", "nan"), ("homing_accel", "-1")):
            with self.subTest(option=option, value=value):
                self.gentle_homing_fixture()
                path, old = (self.axes, "125") if option == "homing_speed" else (self.config, "5000")
                path.write_text(path.read_text().replace(option + ": " + old, option + ": " + value))
                with self.assertRaisesRegex(installer.InstallError, "finite"):
                    self.plan("--gentle-homing")

    def test_gentle_homing_is_idempotent_for_existing_configuration(self):
        self.gentle_homing_fixture()
        changes, _, _ = self.plan("--gentle-homing")
        for path, change in changes.items():
            if change["kind"] == "file":
                path.write_bytes(change["data"])
        changes, _, _ = self.plan("--gentle-homing")
        self.assertNotIn(self.config, changes)
        self.assertNotIn(self.axes, changes)

    def test_gentle_homing_accepts_native_beacon_without_changing_hardware(self):
        self.gentle_homing_fixture(sensor="beacon")
        changes, _, sensor = self.plan("--gentle-homing")
        self.assertEqual(sensor, "beacon")
        self.assertEqual(self.planned_speed(changes, "x"), 40.)
        self.assertIn(b"\nhome_current: 0.6\n", changes[self.axes]["data"])

    def test_persisted_gentle_homing_keeps_hardware_and_prevents_implicit_conversion(self):
        self.gentle_homing_fixture()
        self.config.write_bytes(self.config.read_bytes().replace(b"[accel_homing]\n", b"[accel_homing]\ngentle_homing: True\n"))
        changes, _, _ = self.plan()
        self.assertEqual(self.planned_speed(changes, "x"), 40.)
        self.assertIn(b"\nhome_current: 0.6\n", changes[self.axes]["data"])
        with self.assertRaisesRegex(installer.InstallError, "cannot be combined with --homing-speed"):
            self.plan("--homing-speed", "100")
        self.axes.write_bytes(self.axes.read_bytes().replace(b"endstop_pin: accel_homing:y", b"endstop_pin: !PA0"))
        before = self.config.read_bytes(), self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "cannot convert physical endstops"):
            self.plan()
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)
        changes, _, _ = self.plan("--axes", "x")
        self.assertIn(b"\nendstop_pin: !PA0\n", changes[self.axes]["data"])

    def test_gentle_homing_cli_rejects_conflicting_actions(self):
        for flags in (("--homing-speed", "40"), ("--uninstall",), ("--rollback", "fixture")):
            with self.subTest(flags=flags), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                installer.arguments(["--gentle-homing", *flags])

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
        self.assertIn("# diag_pin: ^PB0", axes)
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

    def test_split_sensor_headers_select_exact_name_without_editing_sensor_options(self):
        fragment = self.config.parent / "sensor-options.cfg"
        for sensor in ("beacon", "adxl345 ToolHead"):
            for flags in ((), ("--accel-chip", sensor)):
                with self.subTest(sensor=sensor, flags=flags):
                    original = (CONFIG.replace("[beacon]", "[" + sensor + "]")
                                + "\n[include sensor-options.cfg]\n" + SAVED).encode()
                    self.config.write_bytes(original)
                    fragment.write_bytes(("# Preserve sensor options and CRLF\r\n[" + sensor
                                          + "]\r\naxes_map: x, -y, z\r\n").encode())
                    sensor_options = fragment.read_bytes()
                    self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 50"))
                    changes, _, selected = self.plan(*flags)
                    self.assertEqual(selected, sensor)
                    self.assertIn(("accel_chip: " + sensor + "\n").encode(),
                                  changes[self.config.parent / installer.MANAGED]["data"])
                    self.assertNotIn(fragment, changes)
                    self.assertEqual(fragment.read_bytes(), sensor_options)
                    self.assertEqual(self.config.read_bytes(), original)
                    self.assertTrue(changes[self.config]["data"].endswith(SAVED.encode()))
                    with self.assertRaisesRegex(installer.InstallError, "exactly one"):
                        installer.ConfigGraph(self.config).one(sensor)

    def test_split_named_beacon_uses_existing_legacy_sensor_selection(self):
        self.config.write_bytes((CONFIG + "\n[beacon sensor Second]\nserial: fixture-second\n"
                                 "[beacon sensor Second]\nx_offset: -20\n"
                                 "[beacon_accel_homing]\nbeacon: beacon sensor Second\nx_threshold: 6.1\n"
                                 + SAVED).encode())
        changes, _, sensor = self.plan()
        self.assertEqual(sensor, "beacon sensor Second")
        self.assertIn(b"accel_chip: beacon sensor Second\n", changes[self.config]["data"])
        self.assertIn(b"beacon: beacon sensor Second\nx_threshold: 6.1\n", changes[self.config]["data"])

    def test_active_beacon_models_do_not_count_as_accelerometers(self):
        for sensor in ("beacon", "beacon sensor ToolHead"):
            with self.subTest(sensor=sensor):
                original = (CONFIG.replace("[beacon]", "[" + sensor + "]")
                            + "\n[" + sensor + "]\nx_offset: -20\n[" + sensor
                            + " model default]\nmodel_coef: 1.234\n" + SAVED).encode()
                self.config.write_bytes(original)
                changes, _, selected = self.plan()
                self.assertEqual(selected, sensor)
                self.assertIn(("[" + sensor + " model default]\nmodel_coef: 1.234\n").encode(),
                              changes[self.config]["data"])
                self.assertEqual(self.config.read_bytes(), original)

    def test_split_sensor_does_not_accept_invalid_beacon_names_or_padded_headers(self):
        for header in ("BEACON", "beacon imaginary", "beacon Sensor ToolHead", "beacon "):
            with self.subTest(header=header):
                self.config.write_text(CONFIG.replace("[beacon]", "[" + header + "]")
                                       + "\n[" + header + "]\nx_offset: -20\n" + SAVED)
                with self.assertRaisesRegex(installer.InstallError, "exactly one"):
                    self.plan("--accel-chip", header.strip())

    def test_split_sensor_does_not_hide_distinct_sensors_or_case_collisions(self):
        self.config.write_text(CONFIG + "\n[beacon]\nx_offset: -20\n"
                               "[beacon sensor second]\nserial: fixture-second\n" + SAVED)
        with self.assertRaisesRegex(installer.InstallError, "configured accelerometer, found 2"):
            self.plan()
        for sensor, conflicting in (("beacon", "Beacon"),
                                    ("adxl345 ToolHead", "adxl345 toolhead")):
            self.config.write_text(CONFIG.replace("[beacon]", "[" + sensor + "]")
                                   + "\n[" + conflicting + "]\naxes_map: x, -y, z\n" + SAVED)
            for flags in ((), ("--accel-chip", sensor)):
                with self.subTest(sensor=sensor, flags=flags), \
                     self.assertRaisesRegex(installer.InstallError, "exactly one"):
                    self.plan(*flags)

    def test_case_collision_does_not_hide_another_sensor_based_on_header_order(self):
        for first, second in (("beacon", "BEACON"), ("BEACON", "beacon")):
            with self.subTest(first=first):
                self.config.write_text(CONFIG.replace("[beacon]", "[" + first + "]")
                                       + "\n[" + second + "]\nx_offset: -20\n"
                                       "[adxl345 toolhead]\ncs_pin: fixture:None\n" + SAVED)
                with self.assertRaisesRegex(installer.InstallError, "configured accelerometer, found 2"):
                    self.plan()

    def test_explicit_unique_third_party_sensor_keeps_physical_uniqueness_check(self):
        custom = "custom_accel ThirdParty"
        self.config.write_text(CONFIG + "\n[" + custom + "]\nserial: fixture-custom\n" + SAVED)
        self.assertEqual(installer.choose_sensor(installer.ConfigGraph(self.config), custom), custom)
        self.config.write_text(CONFIG + ("\n[" + custom + "]\nserial: fixture-custom\n") * 2 + SAVED)
        with self.assertRaisesRegex(installer.InstallError, "exactly one"):
            installer.choose_sensor(installer.ConfigGraph(self.config), custom)

    def test_repeated_physical_include_is_still_rejected(self):
        self.config.write_text(CONFIG + "\n[include axes.cfg]\n" + SAVED)
        with self.assertRaisesRegex(installer.InstallError, "included more than once"):
            self.plan()

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
        self.axes.write_text(AXES.replace("min_home_dist: 10\n", "").replace("homing_speed: 150", "homing_speed: 50")
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
        for option, value, conflict in (("max_report_travel", "1", "report-speed limit 14.2857142857 mm/s"),
                                        ("max_startup_travel", "3", "3 mm max_startup_travel limit")):
            with self.subTest(option=option):
                self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                                       "\n[accel_homing]\naccel_chip: adxl345 toolhead\n"
                                       + option + ": " + value + "\n" + SAVED)
                self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75").replace("min_home_dist: 10\n", ""))
                before = self.config.read_bytes()
                with self.assertRaisesRegex(installer.InstallError, conflict):
                    self.plan()
                changes = self.plan("--homing-speed", "10")[0]
                self.assertNotIn(self.config, changes)
                self.assertEqual(self.config.read_bytes(), before)

    def test_explicit_sample_age_20_50_and_70_ms_is_accepted_and_preserved(self):
        for age in ("0.020", "0.050", "0.070"):
            with self.subTest(age=age):
                self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                                       "\n[accel_homing]\nmax_sample_age: " + age + "\n" + SAVED)
                before = self.config.read_bytes(), self.axes.read_bytes()
                changes = self.plan()[0]
                planned = changes.get(self.config, {"data": before[0]})["data"]
                self.assertEqual(re.findall(rb"(?m)^max_sample_age:\s*(\S+)", planned), [age.encode()])
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)
                self.assertIn(("max_sample_age: " + age).encode(), self.config.read_bytes())

    def test_absent_distance_policies_still_reject_invalid_timing_before_staging(self):
        settings = ("max_sample_age: 0.004", "max_sample_age: nan", "max_sample_age: 0.070001",
                    "stream_interval: 0.021",
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

    def test_explicit_policy_overrides_admit_20_without_changing_speed_or_timing(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 20")
                             .replace("homing_speed: 35", "homing_speed: 20"))
        before = self.config.read_bytes(), self.axes.read_bytes()
        changes = self.plan("--max-report-travel", "1.5", "--max-startup-travel", "4.5")[0]
        managed = changes[self.config.parent / installer.MANAGED]["data"]
        self.assertIn(b"max_report_travel: 1.5\nmax_startup_travel: 4.5\n", managed)
        for option in (b"max_sample_age:", b"stream_interval:", b"homing_accel:"):
            self.assertNotIn(option, managed)
        self.assertEqual(self.planned_speed(changes, "x"), 20.0)
        self.assertEqual(self.planned_speed(changes, "y"), 20.0)
        self.assertNotIn(b"previous: homing_speed", changes[self.axes]["data"])
        self.assertIn(b"max_accel: 5000", changes[self.config]["data"])
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_each_distance_policy_can_be_enabled_independently(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") + SAVED)
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 20")
                             .replace("homing_speed: 35", "homing_speed: 20"))
        for flags, conflict in ((("--max-report-travel", "1"), "report-speed limit 14.2857142857 mm/s"),
                                (("--max-startup-travel", "1.5"), "1.5 mm max_startup_travel limit")):
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
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 75").replace("min_home_dist: 10\n", ""))
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
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 20")
                             .replace("homing_speed: 35", "homing_speed: 20"))
        required = installer.load_motion_policy().motion_startup_request_travel(20.0, 5000.0, 0.070, 0.005)
        for limit in (required, required + 0.0000001):
            with self.subTest(limit=limit):
                changes = self.plan("--max-report-travel", "1.5", "--max-startup-travel", repr(limit))[0]
                text = changes[self.config.parent / installer.MANAGED]["data"].decode()
                persisted = float(re.search(r"(?m)^max_startup_travel:\s*(.+)$", text).group(1))
                self.assertEqual(persisted, limit)
                self.assertGreaterEqual(persisted, required)
        with self.assertRaisesRegex(installer.InstallError, "startup stop request needs 2 mm"):
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
        self.assertIn(b"# homing_accel: 100", planned)
        self.assertIn(b"homing_retract_dist: 0", planned)
        self.assertIn(b"# min_home_dist: 10", planned)

    def test_min_home_dist_migrates_into_plugin_section(self):
        changes = self.plan()[0]
        managed = changes[self.config.parent / installer.MANAGED]["data"]
        self.assertIn(b"x_min_home_dist: 10\n", managed)
        self.assertNotIn(b"y_min_home_dist", managed)
        self.assertIn(b"# min_home_dist: 10", changes[self.axes]["data"])

    def test_min_home_dist_zero_or_absent_not_migrated(self):
        for value in ("min_home_dist: 0", "min_home_dist: 0.0"):
            with self.subTest(value=value):
                self.axes.write_text(AXES.replace("min_home_dist: 10", value))
                managed = self.plan()[0][self.config.parent / installer.MANAGED]["data"]
                self.assertNotIn(b"_min_home_dist:", managed)
        self.axes.write_text(AXES.replace("min_home_dist: 10\n", ""))
        managed = self.plan()[0][self.config.parent / installer.MANAGED]["data"]
        self.assertNotIn(b"_min_home_dist:", managed)

    def test_min_home_dist_invalid_value_changes_nothing(self):
        for value in ("abc", "-1", "nan", "inf"):
            with self.subTest(value=value):
                self.axes.write_text(AXES.replace("min_home_dist: 10", "min_home_dist: " + value))
                before = self.config.read_bytes(), self.axes.read_bytes()
                with self.assertRaisesRegex(installer.InstallError, "No files changed"):
                    self.plan()
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)

    def test_min_home_dist_rerun_does_not_duplicate(self):
        self.require_symlinks()
        changes = self.plan()[0]
        installer.apply_plan({path: state for path, state in changes.items() if state["kind"] != "symlink"}, self.config)
        again = self.plan()[0]
        managed = again.get(self.config.parent / installer.MANAGED, {}).get("data", self.config.parent.joinpath(installer.MANAGED).read_bytes())
        self.assertEqual(managed.count(b"x_min_home_dist:"), 1)
        self.assertIn(b"x_min_home_dist: 10\n", managed)
        self.assertEqual(managed.count(b"y_min_home_dist:"), 0)

    def test_min_home_dist_existing_section_gets_value(self):
        self.config.write_text(CONFIG.replace("[beacon]", "[adxl345 toolhead]") +
                               "\n[accel_homing]\nmax_sample_age: 0.020\n" + SAVED)
        changes = self.plan()[0]
        planned = changes[self.config]["data"]
        self.assertIn(b"x_min_home_dist: 10", planned)
        self.assertNotIn(self.config.parent / installer.MANAGED, changes)

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
        self.axes.write_text(AXES.replace("homing_speed: 150", "homing_speed: 20")
                             .replace("homing_speed: 35", "homing_speed: 20"))
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
                        "Preserved configured homing_speed: X=20 mm/s, Y=20 mm/s"):
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

    def test_generic_names_are_install_only(self):
        for mode in (("--uninstall",), ("--rollback", "offline-backup")):
            with self.subTest(mode=mode), contextlib.redirect_stderr(io.StringIO()), \
                 self.assertRaises(SystemExit):
                installer.arguments(list(mode) + ["--generic-names"])
        self.assertTrue(installer.arguments(["--generic-names", "--dry-run", "--no-restart"]).generic_names)

    def test_generic_names_rename_both_pin_aliases_with_only_one_selected_axis(self):
        legacy = ("\n [beacon_accel_homing] ; Preserve header comment\nbeacon: beacon\naccel_chip: beacon\n"
                  "x_threshold: 6.1\ny_threshold: 7.2\n# printer.beacon_accel_homing is historical\n")
        self.config.write_bytes((CONFIG + legacy + SAVED).encode())
        self.axes.write_bytes(AXES.replace("min_home_dist: 10\n", "").replace("tmc2209_stepper_x:virtual_endstop", "beacon_accel:x")
                              .replace("!PA0", "beacon_accel:y").encode())
        changes, _, sensor = self.plan("--generic-names", "--axes", "x")
        self.assertEqual(sensor, "beacon")
        self.assertEqual(changes[self.config]["data"],
                         (CONFIG + legacy.replace("[beacon_accel_homing]", "[accel_homing]") + SAVED).encode())
        y_before = self.axes.read_bytes().split(b"[stepper_y]", 1)[1].split(b"[tmc2209 stepper_x]", 1)[0]
        y_after = changes[self.axes]["data"].split(b"[stepper_y]", 1)[1].split(b"[tmc2209 stepper_x]", 1)[0]
        self.assertEqual(y_after, y_before.replace(b"beacon_accel:y", b"accel_homing:y"))
        self.assertIn(b"endstop_pin: accel_homing:x", changes[self.axes]["data"])
        self.assertNotIn(b"previous: endstop_pin: beacon_accel", changes[self.axes]["data"])

    def test_generic_names_preserve_explicit_legacy_sensor_selection(self):
        self.config.write_bytes((CONFIG + "\n[beacon second]\nserial: fixture-second\n"
                                 "[beacon_accel_homing]\nbeacon: beacon second\nx_threshold: 6.1\n" + SAVED).encode())
        changes, _, sensor = self.plan("--generic-names")
        self.assertEqual(sensor, "beacon second")
        self.assertIn(b"[accel_homing]\nbeacon: beacon second\nx_threshold: 6.1\n", changes[self.config]["data"])
        plugin = next(section for section in installer.ConfigDocument(self.config, changes[self.config]["data"]).sections
                      if section.name == "accel_homing")
        self.assertEqual(plugin.get("accel_chip"), "beacon second")

    def test_generic_names_refuse_active_legacy_references_before_staging(self):
        for reference in ("    SET_BEACON_ACCEL_HOMING AXIS=X THRESHOLD=1\n",
                          "    BEACON_ACCEL_HOMING_STATUS\n",
                          "    {% set state = printer.beacon_accel_homing %}\n",
                          '    {% set state = printer["beacon_accel_homing"] %}\n',
                          "    M118 beacon_accel:x\n"):
            with self.subTest(reference=reference):
                self.config.write_bytes((CONFIG + reference + "\n[beacon_accel_homing]\n" + SAVED).encode())
                original = self.config.read_bytes(), self.axes.read_bytes()
                with mock.patch.object(installer.Section, "replace") as replace, \
                     mock.patch.object(installer.Section, "generic_name") as rename, \
                     self.assertRaisesRegex(installer.InstallError, "legacy macro/object/command reference"):
                    self.plan("--generic-names")
                replace.assert_not_called()
                rename.assert_not_called()
                self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), original)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_generic_names_already_generic_header_normalizes_remaining_pin_and_is_idempotent(self):
        self.require_symlinks()
        self.config.write_bytes((CONFIG + "\n[accel_homing]\naccel_chip: beacon\n" + SAVED).encode())
        self.axes.write_bytes(AXES.replace("tmc2209_stepper_x:virtual_endstop", "accel_homing:x")
                              .replace("!PA0", "beacon_accel:y").encode())
        first = self.plan("--generic-names")[0]
        installer.apply_plan(first, self.config, selected=self.selected)
        current = self.config.read_bytes(), self.axes.read_bytes()
        self.assertIn(b"endstop_pin: accel_homing:y", current[1])
        self.assertEqual(self.plan("--generic-names")[0], {})
        self.assertEqual(self.plan()[0], {})
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), current)

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

    def test_transaction_idempotence_without_persistent_backups(self):
        changes = {p: v for p, v in self.plan()[0].items() if v["kind"] != "symlink"}
        self.assertIsNone(installer.apply_plan(changes, self.config))
        with mock.patch.object(installer, "MODULES", ()):
            self.assertEqual(self.plan()[0], {})
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
        self.assertNotIn("# accel_homing previous:", self.axes.read_text())

    def test_deprecated_rollback_never_reads_paths_or_changes_current_edits(self):
        self.axes.write_text(self.axes.read_text() + "# later user edit\n")
        expected = self.axes.read_bytes()
        with mock.patch.object(Path, "read_text", side_effect=AssertionError("must not read backup")), \
             self.assertRaisesRegex(installer.InstallError, "deprecated and unsupported"):
            installer.rollback(self.home / "missing-backup")
        self.assertEqual(self.axes.read_bytes(), expected)

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
        with self.assertRaisesRegex(installer.InstallError, "recovery needs manual repair"):
            installer.apply_plan(changes, self.config, concurrent_failure)
        self.assertEqual(self.config.read_bytes(), b"concurrent editor/SAVE_CONFIG data")
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_deprecated_rollback_cli_precedes_discovery_and_service_checks(self):
        with mock.patch.object(installer, "discover_target", side_effect=AssertionError("must not discover")), \
             self.assertRaisesRegex(installer.InstallError, "deprecated and unsupported"):
            installer.main(["--rollback", str(self.home / "missing-backup")])

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

    def test_empty_wildcard_includes_allow_install_and_remain_in_graph(self):
        (self.config.parent / "empty").mkdir()
        for pattern in ("K-ShakeTune/*.cfg", "empty/*.cfg", "empty/optional?.cfg"):
            with self.subTest(pattern=pattern):
                original = (CONFIG + "\n[include " + pattern + "]\n" + SAVED).encode()
                self.config.write_bytes(original)
                graph = installer.ConfigGraph(self.config)
                self.assertEqual(set(graph.documents), {self.config, self.axes})
                self.assertIn(str(self.config.parent / pattern), graph.include_patterns)
                changes = self.plan()[0]
                self.assertIn(self.config.parent / installer.MANAGED, changes)
                self.assertEqual(self.config.read_bytes(), original)

    def test_missing_literal_include_still_refuses_before_install_writes(self):
        for pattern in ("missing.cfg", "missing-directory/required.cfg"):
            with self.subTest(pattern=pattern):
                original = (CONFIG + "\n[include " + pattern + "]\n" + SAVED).encode()
                self.config.write_bytes(original)
                with self.assertRaisesRegex(installer.InstallError, "Include has no matches"):
                    self.plan()
                self.assertEqual(self.config.read_bytes(), original)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

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
        installer.apply_plan(self.plan()[0], self.config)
        for name in installer.MODULES:
            self.assertEqual((self.extras / name).resolve(), self.source / name)
        self.assertEqual(self.plan()[0], {})
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def install_files(self, *flags):
        changes = {path: value for path, value in self.plan(*flags)[0].items() if value["kind"] != "symlink"}
        self.assertIsNone(installer.apply_plan(changes, self.config))

    def test_include_follows_options_added_to_last_stepper_section(self):
        head = ("[printer]\nkinematics: corexy\nmax_velocity: 300\nmax_accel: 5000\n"
                "[adxl345 toolhead]\ncs_pin: fixture:None\n"
                "[stepper_x]\nendstop_pin: !PA0\nhoming_speed: 60\n")
        cases = (("no SAVE_CONFIG", "[stepper_y]\nendstop_pin: !PA1\nhoming_speed: 60\n", "", ()),
                 ("SAVE_CONFIG", "[stepper_y]\nendstop_pin: !PA1\nhoming_speed: 60\n", SAVED, ()),
                 ("homing speed", "[stepper_y]\nendstop_pin: !PA1\n", SAVED, ("--homing-speed", "60")))
        for label, last, tail, flags in cases:
            with self.subTest(label):
                managed = self.config.parent / installer.MANAGED
                if managed.exists():
                    managed.unlink()
                self.config.write_text(head + last + tail)
                self.install_files(*flags)
                parsed = klipper_parse(self.config)
                self.assertEqual(parsed.get("stepper_y", "homing_retract_dist"), "0")
                self.assertEqual(parsed.get("stepper_y", "homing_speed"), "60")
                self.assertEqual(parsed.get("accel_homing", "accel_chip"), "adxl345 toolhead")
                text = self.config.read_text()
                self.assertLess(text.rindex("homing_retract_dist: 0"), text.index("[include accel_homing.cfg]"))
                if tail:
                    self.assertTrue(text.endswith(tail))
                    self.assertLess(text.index("[include accel_homing.cfg]"), text.index(installer.SAVE_MARKER))

    def test_new_plugin_section_follows_options_added_to_retained_include(self):
        managed = self.config.parent / installer.MANAGED
        managed.write_text("[stepper_y]\nendstop_pin: !PA1\nhoming_speed: 60\n")
        self.config.write_text("[printer]\nkinematics: corexy\nmax_velocity: 300\nmax_accel: 5000\n"
                               "[adxl345 toolhead]\ncs_pin: fixture:None\n"
                               "[stepper_x]\nendstop_pin: !PA0\nhoming_speed: 60\n"
                               "[include accel_homing.cfg]\n" + SAVED)
        self.install_files()
        parsed = klipper_parse(self.config)
        self.assertEqual(parsed.get("stepper_y", "homing_retract_dist"), "0")
        self.assertFalse(parsed.has_option("accel_homing", "homing_retract_dist"))
        self.assertEqual(parsed.get("accel_homing", "accel_chip"), "adxl345 toolhead")

    def test_module_links_are_written_before_printer_configuration(self):
        order = list(self.plan()[0])
        links = [order.index(self.extras / name) for name in installer.MODULES]
        self.assertLess(max(links), order.index(self.config))
        self.assertLess(max(links), order.index(self.axes))

    def test_interrupted_write_restores_prior_bytes_and_propagates(self):
        for interruption in (KeyboardInterrupt, SystemExit):
            with self.subTest(interruption.__name__):
                calls = []
                def interrupt_second(path, value):
                    calls.append(path)
                    if len(calls) == 2:
                        raise interruption()
                    installer.write_state(path, value)
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr), self.assertRaises(interruption):
                    installer.apply_plan({self.config: {"kind": "file", "data": b"changed"},
                                          self.axes: {"kind": "file", "data": b"changed"}},
                                         self.config, interrupt_second)
                self.assertEqual(self.config.read_bytes(), (CONFIG + SAVED).encode())
                self.assertEqual(self.axes.read_bytes(), AXES.encode())
                self.assertIn("applied changes restored", stderr.getvalue())

    def test_missing_git_info_directory_is_created_for_exclude(self):
        self.require_symlinks()
        shutil.rmtree(self.repo / ".git/info")
        exclude = self.repo / ".git/info/exclude"
        self.assertIn(exclude, self.plan()[0])
        with mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer, "__file__", str(self.source / "installer.py")), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             mock.patch.object(installer, "assert_idle", return_value="idle fixture"), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(installer.main(["--no-restart", "--no-update-manager"]), 0)
        self.assertEqual(exclude.read_text().splitlines(), ["klippy/extras/" + name for name in installer.MODULES])
        shutil.rmtree(self.repo / ".git")
        (self.repo / ".git").write_text("gitdir: /elsewhere\n")
        self.assertNotIn(exclude, self.plan()[0])

    def test_symlinked_main_config_into_another_directory_is_refused(self):
        self.require_symlinks()
        elsewhere = self.home / "config-repo"
        elsewhere.mkdir()
        os.replace(self.config, elsewhere / "printer.cfg")
        os.symlink(elsewhere / "printer.cfg", self.config)
        before = sorted(p.relative_to(self.home).as_posix() for p in self.home.rglob("*"))
        argv = ["python", str(self.repo / "klippy/klippy.py"), str(self.config), "-a", str(self.home / "uds")]
        live = installer.command_instance(argv, self.home, "fixture", pid=5, service="klipper.service")
        for label, args, processes in (("common directories", installer.arguments([]), []),
                                       ("explicit", installer.arguments(["--config", str(self.config)]), []),
                                       ("running", installer.arguments([]), [live])):
            with self.subTest(label), \
                 self.assertRaisesRegex(installer.InstallError, "symlink into another directory"):
                installer.discover_target(args, self.home, processes, [])
        self.assertEqual(sorted(p.relative_to(self.home).as_posix() for p in self.home.rglob("*")), before)
        chosen = installer.discover_target(installer.arguments(["--config", str(elsewhere / "printer.cfg")]),
                                           self.home, [], [])
        self.assertEqual(chosen["config"], (elsewhere / "printer.cfg").resolve())

    def test_symlinked_config_directory_keeps_include_location(self):
        self.require_symlinks()
        linked = self.home / "linked-config"
        os.symlink(self.config.parent, linked, target_is_directory=True)
        argv = ["python", str(self.repo / "klippy/klippy.py"), str(linked / "printer.cfg")]
        live = installer.command_instance(argv, self.home, "fixture", pid=5)
        chosen = installer.discover_target(installer.arguments([]), self.home, [live], [])
        self.assertEqual(chosen["config"], self.config.resolve())


class SudoPolicyFixture(unittest.TestCase):
    def test_legacy_nss_source_must_be_absent_or_unambiguously_files_only(self):
        for content, allowed in (("passwd: files\n", True), ("sudoers: files # local\n", True),
                                 ("sudoers: files sss\n", False), ("sudoers: ldap\n", False),
                                 ("sudoers: files [SUCCESS=return]\n", False),
                                 ("sudoers: files\nsudoers: files\n", False)):
            with self.subTest(content=content), mock.patch.object(Path, "read_text", return_value=content):
                self.assertEqual(installer.sudo_files_policy(), allowed)
        with mock.patch.object(Path, "read_text", side_effect=PermissionError("unreadable")):
            self.assertFalse(installer.sudo_files_policy())

    def test_controlling_terminal_is_checked_even_when_standard_input_is_redirected(self):
        with mock.patch.object(installer.sys, "stdin", io.StringIO()), \
             mock.patch.object(installer.os, "open", return_value=7) as opening, \
             mock.patch.object(installer.os, "close") as closing:
            self.assertTrue(installer.sudo_has_terminal())
        opening.assert_called_once_with("/dev/tty", os.O_RDWR | os.O_NOCTTY)
        closing.assert_called_once_with(7)
        with mock.patch.object(installer.os, "open", side_effect=OSError("SSH without terminal")):
            self.assertFalse(installer.sudo_has_terminal())

    def test_legacy_executable_alias_identity_is_checked_and_unknown_identity_refuses(self):
        command = ["/usr/bin/systemctl", "--system", "restart", "--", "klipper.service"]
        rule = "/bin/systemctl --system restart -- klipper.service"
        with mock.patch.object(installer.os.path, "samefile", return_value=True) as same:
            self.assertTrue(installer.sudo_literal_matches(rule, command))
        same.assert_called_once_with("/bin/systemctl", "/usr/bin/systemctl")
        with mock.patch.object(installer.os.path, "samefile", side_effect=OSError("missing")):
            with self.assertRaisesRegex(ValueError, "executable alias"):
                installer.sudo_literal_matches(rule, command)


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
        self.terminal = True
        self.restart_argv = ["/offline/systemctl", "--system", "--no-ask-password",
                             "restart", "--", self.selected["service"]]
        self.policy_output = self.policy_entry(passwordless=False, matched=True)
        self.full_policy_output = ""
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
            mock.patch.object(installer, "sudo_has_terminal", side_effect=lambda: self.terminal),
            mock.patch.object(installer, "sudo_files_policy", return_value=True),
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
        if command[1:] == ["-n", "-u", "root", "-v"]:
            self.events.append("auth")
            return subprocess.CompletedProcess(command, self.auth_return, "", "auth unavailable")
        if command[1:] == ["-u", "root", "-v"]:
            self.events.append("credentials")
            if self.acquire_return == 0:
                self.auth_return = 0
            return subprocess.CompletedProcess(command, self.acquire_return, "", "credentials denied")
        if command[1:5] == ["-n", "-u", "root", "-ll"]:
            self.assertEqual(kwargs["env"]["LC_ALL"], "C")
            self.assertEqual(kwargs["env"]["LANG"], "C")
            if len(command) == 5:
                self.events.append("policy-list")
                output = self.full_policy_output
            else:
                self.events.append("policy")
                self.assertEqual(command[5:], ["--"] + self.restart_argv)
                output = self.policy_output
            return subprocess.CompletedProcess(command, self.list_return, output, "restart denied")
        if command[1:6] == ["-n", "-u", "root", "-l", "--"]:
            self.events.append("permission")
            self.assertEqual(command[6:], ["/offline/systemctl", "--system", "--no-ask-password",
                                           "restart", "--", self.selected["service"]])
            return subprocess.CompletedProcess(command, self.list_return, "", "restart denied")
        self.assertEqual(command, ["/offline/sudo", "-n", "-u", "root", "--", "/offline/systemctl", "--system",
                                   "--no-ask-password", "restart", "--", self.selected["service"]])
        self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
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

    def assert_no_backups(self):
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def receipt(self):
        return json.loads(installer.activation_path(self.selected).read_text())

    def policy_entry(self, command=None, passwordless=True, matched=False, runas="root", groups=None):
        command = " ".join(self.restart_argv) if command is None else command
        output = "Sudoers entry:\n    RunAsUsers: %s\n" % runas
        if groups is not None:
            output += "    RunAsGroups: %s\n" % groups
        output += "    Options: %sauthenticate\n    Commands:\n\t%s\n" % ("!" if passwordless else "", command)
        if matched:
            output += "    Matched: " + " ".join(self.restart_argv) + "\n"
        return output

    def legacy_policy(self, entries):
        self.policy_output = " ".join(self.restart_argv) + "\n"
        self.full_policy_output = ("Matching Defaults entries for fixture on host:\n"
                                   "    env_reset, mail_badpass, secure_path=/usr/bin:/bin\n\n"
                                   "User fixture may run the following commands on host:\n\n" + "\n".join(entries))

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
        self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_valid_cached_auth_still_requires_exact_permission_without_prompt(self):
        self.assertEqual(self.cli(), 0)
        self.assertNotIn("credentials", self.events)
        self.assertNotIn("policy", self.events)
        self.assertEqual(self.events.count("permission"), 3)

    def test_modern_exact_nopasswd_all_runas_works_when_validation_fails_without_tty(self):
        self.auth_return, self.terminal = 1, False
        self.policy_output = self.policy_entry(matched=True, runas="ALL", groups="ALL")
        self.assertEqual(self.cli(), 0)
        self.assertNotIn("credentials", self.events)
        self.assertEqual(self.events.count("policy"), 3)
        self.assertEqual(self.events.count("restart"), 1)
        self.assertEqual(self.events[self.events.index("restart") - 1], "idle")

    def test_modern_exact_policy_uses_sudos_root_match_for_mixed_runas_selectors(self):
        self.auth_return, self.terminal = 1, False
        self.policy_output = self.policy_entry(matched=True, runas="root, operator", groups="root")
        self.assertEqual(self.cli(), 0)
        self.assertNotIn("credentials", self.events)

    def test_passwordless_permission_revoked_before_first_write_stops_without_backup(self):
        self.auth_return, self.terminal = 1, False
        self.policy_output = self.policy_entry(matched=True)
        plan = installer.prepare_restart(self.selected)
        self.policy_output = self.policy_entry(passwordless=False, matched=True)
        changes = self.file_plan(self.source, self.selected, installer.arguments([]))[0]
        before = self.config.read_bytes(), self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "does not explicitly permit NOPASSWD"):
            installer.apply_plan(changes, self.config, selected=self.selected,
                                 before_write=lambda: installer.assert_restart_safe(self.selected, plan))
        self.assertEqual((self.config.read_bytes(), self.axes.read_bytes()), before)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
        self.assertNotIn("restart", self.events)
        self.assertNotIn("idle", self.events)

    def test_legacy_mixed_policy_uses_last_matching_rule_with_all_runas(self):
        self.auth_return, self.terminal = 1, False
        self.legacy_policy([self.policy_entry("ALL", passwordless=False, runas="ALL", groups="ALL"),
                            self.policy_entry(runas="ALL", groups="ALL"),
                            self.policy_entry("/usr/bin/true", passwordless=False)])
        self.assertEqual(self.cli(), 0)
        self.assertEqual(self.events.count("policy-list"), 3)
        self.assertNotIn("credentials", self.events)

    def test_legacy_later_passwd_or_executable_alias_blocks_before_writes(self):
        self.auth_return, self.terminal = 1, False
        for later in ("ALL", " ".join(self.restart_argv).replace("/offline/", "/alias/")):
            with self.subTest(later=later), mock.patch.object(installer.os.path, "samefile", return_value=True):
                self.legacy_policy([self.policy_entry(), self.policy_entry(later, passwordless=False)])
                with self.assertRaisesRegex(installer.InstallError, "does not explicitly permit NOPASSWD"):
                    self.cli()
                self.assertNotIn("credentials", self.events)
                self.assertNotIn("restart", self.events)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_unknown_ambiguous_or_mismatched_policy_never_proves_passwordless(self):
        self.auth_return, self.terminal = 1, False
        valid = self.policy_entry(matched=True)
        cases = ("", valid.replace("!authenticate", "authenticate"),
                 valid.replace("!authenticate", "!authenticate, authenticate"), valid + valid,
                 valid.replace("Matched: " + " ".join(self.restart_argv), "Matched: /usr/bin/true"),
                 valid.replace("RunAsUsers: root", "RunAsUsers:"),
                 valid.replace("Options:", "Unexpected:"), valid.replace("Sudoers entry:", "Entrada sudoers:"))
        for output in cases:
            with self.subTest(output=output):
                self.policy_output = output
                with self.assertRaisesRegex(installer.InstallError, "Cannot verify noninteractive sudo"):
                    self.cli()
                self.assertNotIn("credentials", self.events)
                self.assertNotIn("restart", self.events)
                self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_legacy_unknown_patterns_defaults_and_policy_sources_fail_closed(self):
        self.auth_return, self.terminal = 1, False
        for pattern in ("/offline/systemctl *", "!/offline/systemctl", "^/offline/systemctl$", "SYSTEM_COMMANDS"):
            with self.subTest(pattern=pattern):
                self.legacy_policy([self.policy_entry(), self.policy_entry(pattern)])
                with self.assertRaisesRegex(installer.InstallError, "unsupported sudoers command pattern"):
                    self.cli()
        self.legacy_policy([self.policy_entry()])
        self.full_policy_output = "Runas and Command-specific defaults for fixture:\n    Defaults!ALL authenticate\n" + self.full_policy_output
        with self.assertRaisesRegex(installer.InstallError, "unsupported sudoers defaults"):
            self.cli()
        for option in ("runchroot=/isolated", "runcwd=/isolated"):
            with self.subTest(option=option):
                self.legacy_policy([self.policy_entry()])
                self.full_policy_output = self.full_policy_output.replace("env_reset,", option + ", env_reset,")
                with self.assertRaisesRegex(installer.InstallError, "command filesystem context"):
                    self.cli()
        self.legacy_policy([self.policy_entry()])
        with mock.patch.object(installer, "sudo_files_policy", return_value=False):
            with self.assertRaisesRegex(installer.InstallError, "files-only"):
                self.cli()
        self.assertNotIn("credentials", self.events)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_no_terminal_does_not_attempt_credential_acquisition(self):
        self.auth_return, self.terminal = 1, False
        with self.assertRaisesRegex(installer.InstallError, "No credential prompt.*--no-restart"):
            self.cli()
        self.assertNotIn("credentials", self.events)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_passwordless_policy_revoked_before_activation_retains_files_without_restart(self):
        self.auth_return = 1
        self.policy_output = self.policy_entry(matched=True)
        def revoke(count):
            if count == 1:
                self.policy_output = self.policy_entry(passwordless=False, matched=True)
        self.after_idle = revoke
        with self.assertRaisesRegex(installer.InstallError, "Files are installed.*blocked"):
            self.cli()
        self.assertNotIn("credentials", self.events)
        self.assertNotIn("restart", self.events)
        self.assertEqual(self.receipt()["status"], "blocked")
        self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

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

    def test_restart_failure_or_timeout_keeps_files_and_truthful_activation_failure(self):
        self.restart_return = 1
        with self.assertRaisesRegex(installer.InstallError, "Files are installed.*failed"):
            self.cli()
        current = self.axes.read_bytes()
        self.assertEqual(self.receipt()["status"], "failed")
        self.assertTrue(self.receipt()["restart_attempted"])
        self.assertIn("endstop_pin: accel_homing:x", self.axes.read_text())
        self.restart_error = subprocess.TimeoutExpired(["mocked-sudo"], 45)
        with self.assertRaisesRegex(installer.InstallError, "no automatic undo"):
            self.cli()
        self.assertEqual(self.axes.read_bytes(), current)
        self.assertEqual(self.receipt()["status"], "failed")
        self.restart_error, self.restart_return = None, 0
        self.assertEqual(self.cli(), 0)
        self.assertEqual(self.receipt()["status"], "restarted")
        self.assert_no_backups()

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
        with self.assertRaisesRegex(installer.InstallError, "deprecated and unsupported"):
            self.cli("--rollback", str(self.home / "obsolete"))
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
        self.assertIn("accel_homing_axes.py", installer.MODULES)
        self.assertEqual((self.source / "accel_homing_axes.py").read_bytes(),
                         (DIRECTORY / "accel_homing_axes.py").read_bytes())
        self.assertIn("accel_homing_config.py", installer.MODULES)
        self.assertEqual((self.source / "accel_homing_config.py").read_bytes(),
                         (DIRECTORY / "accel_homing_config.py").read_bytes())
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
    """Current-state removal: no archived baseline, host operations or hardware."""
    plan = HostFixture.plan

    def setUp(self):
        HostFixture.setUp(self)
        self.real_consumer_check = installer.assert_uninstall_consumers
        patch = mock.patch.object(installer, "assert_uninstall_consumers")
        self.consumers = patch.start()
        self.addCleanup(patch.stop)

    def install(self, *flags):
        changes, _, _ = self.plan(*flags)
        # Real symlinks have a separate Linux test; all other writes are real.
        for path in list(changes):
            if changes[path]["kind"] == "symlink":
                changes.pop(path)
        self.assertIsNone(installer.apply_plan(changes, self.config))

    def remove(self):
        plan = installer.make_uninstall_plan(self.selected)
        self.assertIsNone(installer.apply_uninstall_plan(plan, self.selected))
        return plan

    def test_uninstall_removes_current_plugin_state_and_preserves_user_configuration(self):
        self.install()
        managed = self.config.parent / installer.MANAGED
        managed.write_text(managed.read_text() + "# user note\n[gcode_macro USER_MACRO]\ngcode: M118 Keep\n")
        self.axes.write_text(self.axes.read_text().replace("homing_speed: 150", "homing_speed: 73") +
                             "# accel_homing previous: endstop_pin: NEVER_RESTORE\n")
        self.config.write_text(self.config.read_text().replace("model_coef = 1.234", "model_coef = 9.876"))
        plan = self.remove()
        self.assertEqual(len(plan["manual_config"]), 2)
        self.assertIn("homing_speed: 73", self.axes.read_text())
        self.assertIn("homing_retract_dist: 0", self.axes.read_text())
        self.assertIn("# accel_homing previous: endstop_pin: NEVER_RESTORE", self.axes.read_text())
        graph = installer.ConfigGraph(self.config)
        self.assertIsNone(graph.one("stepper_x").get("endstop_pin"))
        self.assertIsNone(graph.one("stepper_y").get("endstop_pin"))
        self.assertIsNone(graph.one("accel_homing", False))
        self.assertIn("[gcode_macro USER_MACRO]", managed.read_text())
        self.assertIn("# user note", managed.read_text())
        self.assertIn("model_coef = 9.876", self.config.read_text())
        self.assertNotIn("[accel_homing_model", self.config.read_text())
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())
        self.assertEqual(self.remove()["entries"], [])

    def test_install_uninstall_install_reuses_retained_included_file(self):
        self.install()
        managed = self.config.parent / installer.MANAGED
        managed.write_text(managed.read_text() + "\n[gcode_macro USER_MACRO]\ngcode: M118 Keep\n")
        self.remove()
        self.install()
        graph = installer.ConfigGraph(self.config)
        self.assertIsNotNone(graph.one("accel_homing"))
        self.assertIsNotNone(graph.one("gcode_macro user_macro"))
        self.assertEqual(graph.one("stepper_x").get("endstop_pin"), "accel_homing:x")
        self.assertEqual(self.config.read_text().count("[include accel_homing.cfg]"), 1)

    def test_uninstall_without_history_handles_legacy_and_active_models(self):
        self.config.write_text(CONFIG + "\n[beacon_accel_homing]\naccel_chip: beacon\nx_threshold: 12\n"
                               "[accel_homing_model tuned]\nx_threshold: 17\n" + SAVED)
        self.axes.write_text(AXES.replace("endstop_pin: !PA0", "endstop_pin: beacon_accel:y"))
        self.remove()
        graph = installer.ConfigGraph(self.config)
        self.assertEqual(graph.one("stepper_x").get("endstop_pin"), "tmc2209_stepper_x:virtual_endstop")
        self.assertIsNone(graph.one("stepper_y").get("endstop_pin"))
        self.assertIsNone(graph.one("beacon_accel_homing", False))
        self.assertIsNone(graph.one("accel_homing_model tuned", False))
        self.assertIn("[beacon model default]", self.config.read_text())

    def test_malformed_and_tampered_old_backups_are_never_read_or_changed(self):
        archive = self.config.parent / ".accel_homing_backups"
        archive.mkdir()
        (archive / "manifest.json").write_bytes(b"not JSON\x00")
        (archive / "tampered").write_bytes(b"[old impossible section changes]")
        before = {p: p.read_bytes() for p in archive.iterdir()}
        read_bytes, read_text, iterdir = Path.read_bytes, Path.read_text, Path.iterdir
        def forbidden(path):
            if archive == path or archive in path.parents:
                raise AssertionError("obsolete backup directory was accessed")
        def safe_bytes(path):
            forbidden(path)
            return read_bytes(path)
        def safe_text(path, *args, **kwargs):
            forbidden(path)
            return read_text(path, *args, **kwargs)
        def safe_iterdir(path):
            forbidden(path)
            return iterdir(path)
        with mock.patch.object(Path, "read_bytes", safe_bytes), mock.patch.object(Path, "read_text", safe_text), \
             mock.patch.object(Path, "iterdir", safe_iterdir):
            self.install()
            self.install("--generic-names")
            self.remove()
        self.assertEqual({p: p.read_bytes() for p in archive.iterdir()}, before)

    def test_edited_config_reinstall_preserves_thresholds_models_macros_and_homing_wrapper(self):
        self.install()
        managed = self.config.parent / installer.MANAGED
        managed.write_text(managed.read_text().replace("#x_threshold: 8.0", "x_threshold: 19.25"))
        self.config.write_text(self.config.read_text().replace("[gcode_macro PRINT_START]", HOMING_OVERRIDE + "[gcode_macro PRINT_START]")
                               .replace("x_threshold = 6.5", "x_threshold = 21.75"))
        self.axes.write_text(self.axes.read_text().replace("homing_speed: 150", "homing_speed: 73"))
        before = {p: p.read_bytes() for p in (managed, self.config, self.axes)}
        self.install("--generic-names")
        self.assertEqual({p: p.read_bytes() for p in before}, before)
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_unchanged_include_edited_after_install_planning_refuses_before_writes(self):
        extra = self.config.parent / "unchanged.cfg"
        extra.write_text("[gcode_macro EXTRA]\ngcode: M118 initial\n")
        self.config.write_text(CONFIG + "[include unchanged.cfg]\n" + SAVED)
        changes, _, _ = self.plan()
        before = self.axes.read_bytes()
        extra.write_text("[gcode_macro EXTRA]\ngcode: M118 concurrent\n")
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            installer.apply_plan(changes, self.config)
        self.assertEqual(self.axes.read_bytes(), before)

    def test_unchanged_include_edited_during_transaction_recovers_prior_writes(self):
        extra = self.config.parent / "unchanged.cfg"
        extra.write_text("[gcode_macro EXTRA]\ngcode: M118 initial\n")
        self.config.write_text(CONFIG + "[include unchanged.cfg]\n" + SAVED)
        changes, _, _ = self.plan()
        for path in list(changes):
            if changes[path]["kind"] == "symlink":
                changes.pop(path)
        before = {p: installer.snapshot(p) for p in changes}
        def writer(path, state):
            installer.write_state(path, state)
            extra.write_text("[gcode_macro EXTRA]\ngcode: M118 concurrent\n")
        with self.assertRaisesRegex(installer.InstallError, "applied changes restored"):
            installer.apply_plan(changes, self.config, writer=writer)
        self.assertEqual({p: installer.snapshot(p) for p in before}, before)
        self.assertIn("concurrent", extra.read_text())

    def test_uninstall_plan_guards_unchanged_include_and_instance(self):
        self.install()
        extra = self.config.parent / "unchanged.cfg"
        extra.write_text("[gcode_macro EXTRA]\ngcode: M118 initial\n")
        self.config.write_text(self.config.read_text().replace("[include axes.cfg]", "[include axes.cfg]\n[include unchanged.cfg]"))
        plan = installer.make_uninstall_plan(self.selected)
        before = self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "different instance"):
            installer.apply_uninstall_plan(plan, dict(self.selected, repo=self.home / "other"))
        extra.write_text("[gcode_macro EXTRA]\ngcode: M118 changed\n")
        with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
            installer.apply_uninstall_plan(plan, self.selected)
        self.assertEqual(self.axes.read_bytes(), before)

    def test_uninstall_write_failure_recovers_in_memory_and_retry_succeeds(self):
        self.install()
        plan = installer.make_uninstall_plan(self.selected)
        before = {Path(e["path"]): installer.snapshot(Path(e["path"])) for e in plan["entries"]}
        calls = []
        def writer(path, state):
            calls.append(path)
            if len(calls) == 2:
                raise OSError("synthetic failure")
            installer.write_state(path, state)
        with self.assertRaisesRegex(installer.InstallError, "applied changes restored"):
            installer.apply_uninstall_plan(plan, self.selected, writer=writer)
        self.assertEqual({p: installer.snapshot(p) for p in before}, before)
        self.remove()
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_uninstall_partial_failure_preserves_concurrent_edit(self):
        self.install()
        plan = installer.make_uninstall_plan(self.selected)
        first = Path(plan["entries"][0]["path"])
        calls = []
        def writer(path, state):
            calls.append(path)
            if len(calls) == 2:
                first.write_bytes(b"concurrent operator contents")
                raise OSError("synthetic failure")
            installer.write_state(path, state)
        with self.assertRaisesRegex(installer.InstallError, "recovery needs manual repair"):
            installer.apply_uninstall_plan(plan, self.selected, writer=writer)
        self.assertEqual(first.read_bytes(), b"concurrent operator contents")
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_regular_module_file_and_unsupported_pin_refuse_without_writes(self):
        self.install()
        before = self.axes.read_bytes()
        path = self.extras / installer.MODULES[0]
        path.write_text("# caller-owned regular file")
        with self.assertRaisesRegex(installer.InstallError, "non-symlink"):
            installer.make_uninstall_plan(self.selected)
        path.unlink()
        self.axes.write_text(self.axes.read_text().replace("accel_homing:x", "accel_homing:z"))
        current = self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "Unsupported plugin endstop"):
            installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.axes.read_bytes(), current)
        self.assertNotEqual(before, current)

    def test_real_verified_links_removed_but_foreign_or_changed_links_preserved(self):
        HostFixture.require_symlinks(self)
        self.install()
        for name in installer.MODULES:
            os.symlink(self.source / name, self.extras / name)
        with mock.patch.object(installer, "__file__", str(self.source / "installer.py")):
            # Loader path is also current source; provide the real uninstaller.
            (self.source / "uninstaller.py").write_bytes((DIRECTORY / "uninstaller.py").read_bytes())
            plan = installer.make_uninstall_plan(self.selected)
            link = self.extras / installer.MODULES[0]
            link.unlink()
            os.symlink(self.home / "foreign.py", link)
            with self.assertRaisesRegex(installer.InstallError, "changed after planning"):
                installer.apply_uninstall_plan(plan, self.selected)
            with self.assertRaisesRegex(installer.InstallError, "Unrecognized plugin link"):
                installer.make_uninstall_plan(self.selected)
            link.unlink()
            os.symlink(self.source / installer.MODULES[0], link)
            self.remove()
        self.assertFalse(any((self.extras / name).is_symlink() for name in installer.MODULES))
        self.assertTrue(all((self.source / name).is_file() for name in installer.MODULES))

    def test_shared_checkout_consumer_is_checked_before_and_after_planning(self):
        self.install()
        plan = installer.make_uninstall_plan(self.selected)
        self.consumers.assert_called_once()
        self.consumers.side_effect = installer.InstallError("shared checkout consumer")
        before = self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "shared checkout"):
            installer.apply_uninstall_plan(plan, self.selected)
        self.assertEqual(self.axes.read_bytes(), before)

    def test_actual_shared_checkout_peer_using_plugin_refuses_before_writes(self):
        peer_config = self.config.parent / "peer.cfg"
        peer_config.write_text("[accel_homing]\naccel_chip: beacon\n")
        peer = dict(self.selected, config=peer_config, service="klipper-peer.service")
        plan = {"entries": [{"path": str(self.extras / installer.MODULES[0])}]}
        before = self.axes.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "shared checkout module links"):
            self.real_consumer_check(self.selected, plan, processes=[peer], services=[])
        self.assertEqual(self.axes.read_bytes(), before)

    def test_actual_peer_without_plugin_is_allowed_but_new_dependency_refuses(self):
        peer_config = self.config.parent / "peer.cfg"
        peer_config.write_text("[printer]\nkinematics: corexy\n")
        peer = dict(self.selected, config=peer_config, service="klipper-peer.service")
        plan = {"entries": [{"path": str(self.extras / installer.MODULES[0])}]}
        self.real_consumer_check(self.selected, plan, processes=[], services=[peer])
        peer_config.write_text(peer_config.read_text() + "[stepper_x]\nendstop_pin: accel_homing:x\n")
        with self.assertRaisesRegex(installer.InstallError, "shared checkout module links"):
            self.real_consumer_check(self.selected, plan, processes=[], services=[peer])

    def test_updater_unchanged_input_race_recovers_all_applied_install_writes(self):
        moon_include = self.config.parent / "moon-unchanged.cfg"
        moon_include.write_bytes(b"[history]\nenabled: True\n")
        expected = installer.snapshot(moon_include)
        original = self.axes.read_bytes()
        def writer(path, state):
            installer.write_state(path, state)
            moon_include.write_bytes(b"concurrent Moonraker edit")
        with self.assertRaisesRegex(installer.InstallError, "applied changes restored"):
            installer.apply_plan({self.axes: {"kind": "file", "data": b"replacement"}}, self.config,
                                 writer=writer, updater={"expected": {moon_include: expected}})
        self.assertEqual(self.axes.read_bytes(), original)
        self.assertEqual(moon_include.read_bytes(), b"concurrent Moonraker edit")
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_missing_literal_or_recursive_include_fails_without_mutation(self):
        self.install()
        self.config.write_text(self.config.read_text().replace("[include axes.cfg]", "[include missing.cfg]"))
        before = self.config.read_bytes()
        with self.assertRaisesRegex(installer.InstallError, "Include has no matches"):
            installer.make_uninstall_plan(self.selected)
        self.assertEqual(self.config.read_bytes(), before)

    def test_optional_wildcard_and_split_sensor_documents_survive_removal(self):
        self.config.write_text(CONFIG.replace("[beacon]\nserial: /dev/serial/by-id/fixture-only\n", "[beacon]\nserial: fixture\n[beacon]\nx_offset: 0\n") +
                               "[include optional/*.cfg]\n" + SAVED)
        self.install()
        self.remove()
        graph = installer.ConfigGraph(self.config)
        self.assertEqual(len(graph.sections["beacon"]), 2)
        self.assertIn("[include optional/*.cfg]", self.config.read_text())

    def test_clean_host_without_plugin_state_is_idempotent(self):
        self.config.write_text(CONFIG)
        self.assertEqual(self.remove()["entries"], [])
        self.assertFalse((self.config.parent / ".accel_homing_backups").exists())

    def test_uninstall_removes_klipper_activation_receipt(self):
        self.install()
        receipt = installer.activation_path(self.selected)
        installer.save_activation(receipt, self.selected, "skipped", reason="--no-restart")
        plan = self.remove()
        self.assertIn(str(receipt), [entry["path"] for entry in plan["entries"]])
        self.assertFalse(receipt.exists())
        self.assertEqual(self.remove()["entries"], [])
        receipt.mkdir()
        with self.assertRaises(installer.InstallError):
            installer.make_uninstall_plan(self.selected)

    def test_no_update_manager_uninstall_skips_moonraker_for_stopped_klipper(self):
        self.install()
        receipt = self.config.parent / ".accel_homing_updater.json"
        receipt.write_text("{}\n")
        unreachable = mock.Mock()
        unreachable.make_uninstall_plan.side_effect = installer.InstallError(
            "Moonraker updater: the selected Klipper has no verified API socket")
        with mock.patch.object(installer, "load_updater", return_value=unreachable):
            with self.assertRaisesRegex(installer.InstallError, "no verified API socket"):
                installer.make_uninstall_plan(self.selected)
        output = io.StringIO()
        with mock.patch.object(installer, "load_updater", side_effect=AssertionError("Moonraker was consulted")), \
             mock.patch.object(installer, "discover_target", return_value=self.selected), \
             mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
             contextlib.redirect_stdout(output):
            self.assertEqual(installer.main(["--uninstall", "--no-update-manager"]), 0)
        self.assertIn("Selected Klipper is stopped", output.getvalue())
        self.assertIn("delete that section by hand", output.getvalue())
        self.assertIn("[update_manager accel_homing]", output.getvalue())
        graph = installer.ConfigGraph(self.config)
        self.assertIsNone(graph.one("accel_homing", False))
        self.assertIsNone(graph.one("stepper_x").get("endstop_pin"))
        self.assertEqual(receipt.read_text(), "{}\n")


if __name__ == "__main__":
    unittest.main()
