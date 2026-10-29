# Supervised calibration and measured charts for accelerometer homing. GPLv3.
"""Bounded X/Y workflows. No workflow command ever requests a Z move.

An accelerometer and motor step count cannot establish physical position.
The operator confirms the initial center. Premature calibration triggers reverse
the recorded signed start-to-halt commanded travel, assuming no skipped steps
or slip. Qualified homes return to the configured midpoint.
"""
import datetime
import configparser
from decimal import Decimal
import html
import json
import logging
import math
import os
from pathlib import Path
import re
import types
import sys
import tempfile
import urllib.parse
import uuid

try:
    from .accel_homing_charts import (_accuracy_statistics as accuracy_statistics,
                                      PNGRenderer, default_dependency_directory, preflight_renderer,
                                      validate_accuracy, write_accuracy, write_capture, write_noise,
                                      write_noise_capture, beta_motion_statistics,
                                      compact_beta_home, write_beta_motion)
except ImportError:
    from accel_homing_charts import (_accuracy_statistics as accuracy_statistics,
                                     PNGRenderer, default_dependency_directory, preflight_renderer,
                                     validate_accuracy, write_accuracy, write_capture, write_noise,
                                     write_noise_capture, beta_motion_statistics,
                                     compact_beta_home, write_beta_motion)

try:
    from .homing import PrinterHoming as _PrinterHoming
    from .safe_z_home import SafeZHoming as _SafeZHoming
except ImportError:
    # Standalone analysis can import this module without Klipper. Workflows
    # fail closed until the installed stock classes can be established.
    _PrinterHoming = _SafeZHoming = None

try:
    from .homing_override import HomingOverride as _HomingOverride
except ImportError:
    _HomingOverride = None

_PRINTER_G28 = _PrinterHoming.cmd_G28 if _PrinterHoming is not None else None
_SAFE_Z_G28 = _SafeZHoming.cmd_G28 if _SafeZHoming is not None else None
_OVERRIDE_G28 = _HomingOverride.cmd_G28 if _HomingOverride is not None else None

AXES = {"x": 0, "y": 1}
CALIBRATION_MIN_TRAVEL_MM = 20.0
CALIBRATION_REPEATABILITY_MM = 2.0
SIMPLE_CALIBRATION = "impact20_vector_v2"
REQUIRED_SUCCESSES = 10
REPEATABILITY_LIMIT_MM = 1.5
MIN_TRAVEL = 10.0
NOISE_ITERATIONS = 3
NOISE_DURATION = 1.0


def _moonraker_read_bytes(path, limit):
    with path.open("rb") as stream:
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError("Moonraker discovery input exceeds its byte bound")
    return data


def _moonraker_path_environment(process):
    allowed = {b"HOME", b"MOONRAKER_DATA_PATH", b"MOONRAKER_CONFIG_PATH"}
    environment = {}
    for field in _moonraker_read_bytes(process / "environ", 65536).split(b"\0"):
        key, separator, value = field.partition(b"=")
        if separator and key in allowed:
            name = key.decode("ascii")
            if name in environment:
                raise ValueError("ambiguous Moonraker path environment")
            environment[name] = value.decode("utf-8")
    return environment


def _moonraker_expand_path(value, environment, uid):
    if value.startswith("~"):
        home = environment.get("HOME")
        if home is None:
            try:
                import pwd
                home = pwd.getpwuid(uid).pw_dir
            except (ImportError, KeyError):
                home = None
        if (not home or not Path(home).is_absolute()
                or (value != "~" and not value.startswith("~/"))):
            raise ValueError("cannot establish Moonraker user-home expansion")
        value = home.rstrip("/") + value[1:] or "/"
    return Path(value)


def verified_moonraker_config_root(start_args, proc_root="/proc"):
    """Correlate one same-user Moonraker data path to this Klippy Unix socket.

    Main-config containment alone does not identify Moonraker's config root.
    Unresolved includes, wrappers or paths deliberately return no web mapping.
    """
    socket = start_args.get("apiserver")
    if not socket or not hasattr(os, "getuid"):
        return None
    expected, roots = Path(socket).expanduser().resolve(), []
    try:
        processes = list(Path(proc_root).iterdir())
    except OSError:
        return None
    for process in processes:
        if not process.name.isdigit():
            continue
        try:
            uid = process.stat().st_uid
            if uid != os.getuid():
                continue
            data = _moonraker_read_bytes(process / "cmdline", 65536)
            argv = [part.decode("utf-8") for part in data.split(b"\0") if part]
            indexes = [index for index, value in enumerate(argv) if Path(value).name == "moonraker.py"]
            indexes += [index + 1 for index, value in enumerate(argv[:-1])
                        if value == "-m" and argv[index + 1] == "moonraker"]
            if len(indexes) != 1:
                continue
            environment = _moonraker_path_environment(process)
            options = {}
            iterator = iter(argv[indexes[0] + 1:])
            for value in iterator:
                key, separator, inline = value.partition("=")
                if key in ("-d", "--datapath", "-c", "--configfile"):
                    name = "data" if key in ("-d", "--datapath") else "config"
                    options[name] = inline if separator else next(iterator, "")
                elif (key.startswith(("-d", "-c")) or
                      (key.startswith("--") and any(option.startswith(key)
                                                     for option in ("--datapath", "--configfile")))):
                    raise ValueError("unsupported Moonraker path argument form")
            raw_data = options.get("data", environment.get("MOONRAKER_DATA_PATH"))
            cwd = (process / "cwd").resolve()
            if not cwd.is_dir():
                continue
            data_path = _moonraker_expand_path(raw_data or "~/printer_data", environment, uid)
            data_path = (data_path if data_path.is_absolute() else cwd / data_path).resolve()
            raw_config = options.get("config", environment.get("MOONRAKER_CONFIG_PATH"))
            config_path = (_moonraker_expand_path(raw_config, environment, uid)
                           if raw_config is not None else data_path / "config/moonraker.conf")
            config_path = config_path if config_path.is_absolute() else cwd / config_path
            encoded = _moonraker_read_bytes(config_path, 262144)
            parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=("#", ";"))
            parser.read_string(encoded.decode("utf-8"))
            if not parser.has_section("server") or any(
                    re.match(r"include\s", section, re.I) for section in parser.sections()):
                continue
            raw_socket = parser.get("server", "klippy_uds_address", fallback="/tmp/klippy_uds")
            if any(character in raw_socket for character in ("{", "}", "$", "%", "\n")):
                continue
            actual_socket = _moonraker_expand_path(raw_socket, environment, uid)
            if (not (actual_socket.is_absolute() or raw_socket.startswith("/"))
                    or actual_socket.resolve() != expected):
                continue
            root = (data_path / "config").resolve()
            if root.is_dir():
                roots.append(str(root))
        except (OSError, UnicodeError, ValueError, configparser.Error):
            continue
    return next(iter(roots)) if len(roots) == 1 else None


class AccelHomingWorkflows:
    def __init__(self, owner, config):
        self.owner, self.config = owner, config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.model_name = config.get("model", "default")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self.model_name):
            raise config.error("model must contain only letters, numbers, _ or -")
        self.model_section = "accel_homing_model " + self.model_name
        self.manual_axes = {a for a in AXES
                            if config.get(a + "_threshold", None) is not None}
        if config.get("threshold", None) is not None:
            self.manual_axes.update(AXES)
        self.model = None
        if config.has_section(self.model_section):
            self.model = self.printer.load_object(config, self.model_section)
            for axis, threshold in self.model.thresholds.items():
                if axis not in self.manual_axes:
                    owner.threshold_g[axis] = threshold
        self.start_threshold = config.getfloat("calibration_start_threshold", 1.0,
                                               minval=0.5)
        self.threshold_step = config.getfloat("calibration_step", 0.25,
                                              minval=0.1, maxval=3.0)
        # Consume the retired option so an existing printer.cfg remains valid.
        # Its value never limits calibration or affects the noise context.
        if config.get("calibration_max_threshold", None) is not None:
            logging.warning("accel_homing: calibration_max_threshold is deprecated and ignored; "
                            "calibration retains the sensor-range headroom policy")
        self.max_attempts = config.getint("calibration_max_attempts", 40,
                                          minval=12, maxval=100)
        self.confirm_timeout = config.getfloat("calibration_confirm_timeout", 300.0,
                                               minval=30.0, maxval=1800.0)
        self.travel_tolerance = config.getfloat("calibration_travel_tolerance", 0.20,
                                                minval=0.05, maxval=0.40)
        self.reposition_speed = config.getfloat("workflow_reposition_speed", 50.0,
                                                minval=1.0, maxval=150.0)
        self.calibration_settle_time = config.getfloat("calibration_settle_time", 0.25,
                                                        minval=0.0, maxval=2.0)
        if not all(math.isfinite(value) for value in (
                self.start_threshold, self.threshold_step,
                self.confirm_timeout, self.travel_tolerance, self.reposition_speed,
                self.calibration_settle_time)):
            raise config.error("Accelerometer homing workflow settings must be finite")
        start_args = self.printer.get_start_args() if hasattr(self.printer, "get_start_args") else {}
        main_config = start_args.get("config_file")
        default_charts = (os.path.join(os.path.dirname(os.path.abspath(os.path.expanduser(main_config))),
                                      "accel_homing_charts") if main_config else
                          "~/printer_data/config/accel_homing_charts")
        self.charts_directory = os.path.abspath(os.path.expanduser(config.get(
            "charts_directory", default_charts)))
        self.renderer = PNGRenderer(sys.executable, default_dependency_directory(), self.reactor)
        self.chart_start_args = dict(start_args)
        self.chart_config_root = None
        self.preflight_generation = 0
        self.chart_preflight_check = None
        self.kinematics_name = None
        if hasattr(config, "getsection"):
            self.kinematics_name = config.getsection("printer").get("kinematics")
        self.session = None
        self.last_result = None
        self.noise_recommendation = self.noise_seed_context = None
        self.noise_seeds = {}
        self.in_home = False
        self.home_axis = None
        self.timer = self.reactor.register_timer(self._timeout)
        self.printer.register_event_handler("klippy:shutdown", self._shutdown)
        self.printer.register_event_handler("klippy:disconnect", self._disconnect)
        self.printer.register_event_handler("klippy:connect", self._validate_model_connection)
        self.printer.register_event_handler("klippy:connect", self._find_chart_root)
        for name, callback, description in (
                ("ACCEL_HOMING_CALIBRATE", self.cmd_CALIBRATE,
                 "Calibrate one X or Y homing threshold with ten qualified homes"),
                ("ACCEL_HOMING_BETA_CALIBRATE", self.cmd_BETA_CALIBRATE,
                 "Beta motion-noise experiment: PNG and threshold proposal only"),
                ("ACCEL_HOMING_CALIBRATE_NOISE", self.cmd_NOISE,
                 "Measure three stationary one-second noise windows and suggest calibration starts"),
                ("ACCEL_HOMING_CANCEL", self.cmd_CANCEL,
                 "Cancel the current accelerometer homing workflow"),
                ("ACCEL_HOMING_TEST", self.cmd_TEST,
                 "Test X then Y from an explicitly confirmed center"),
                ("ACCEL_HOMING_CHARTS_GENERATE", self.cmd_CHARTS,
                 "Export acceleration PNG charts; optional AXIS=X or AXIS=Y, default both"),
                ("ACCEL_HOMING_ACCURACY", self.cmd_ACCURACY,
                 "Measure repeated X/Y commanded homing travel and accelerometer captures"),
                ("ACCEL_HOMING_STATUS", self.cmd_STATUS,
                 "Show calibration progress, model and chart output paths")):
            self.gcode.register_command(name, callback, desc=description)

    def _error(self, message):
        return self.printer.command_error("accel_homing: " + message)

    def _info(self, message):
        self.gcode.respond_info("accel_homing: " + message)

    def _find_chart_root(self):
        self.chart_config_root = verified_moonraker_config_root(self.chart_start_args)

    def _chart_check(self, session):
        if session.get("finish_interrupted"):
            raise self._error(session["finish_interrupted"])
        if session.get("finishing") and session.get("finish_result") != "complete":
            if self.session is not session:
                raise self._error("partial PNG export was interrupted")
            # A shutdown/print interruption is reportable after cleanup. This
            # branch exports only already-qualified data and cannot seed a
            # model, resume a trial or authorize movement.
            return
        if session["mode"] == "noise" and not session.get("sensor_stopped"):
            self._check_noise_running(session)
        else:
            self._check_running(session)
            if session["mode"] == "noise":
                self._check_stopped_noise_context(session)

    def _chart_options(self, session):
        return {"renderer": self.renderer, "check_running": lambda: self._chart_check(session)}

    def _chart_preflight(self):
        generation, operation = self.preflight_generation, self.owner.operation_generation
        def check():
            if (self.preflight_generation != generation or self.owner.operation_generation != operation):
                raise self._error("PNG renderer preflight was cancelled or disconnected")
            if hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown():
                raise self._error("printer is shut down")
            if self._printing():
                raise self._error("cannot prepare PNG charts while printing or paused")
        try:
            preflight_renderer(renderer=self.renderer, check_running=check)
            check()
            self.chart_preflight_check = check
        except Exception as exc:
            raise self._error("PNG renderer is unavailable before motion/acquisition: %s. "
                              "Run the package installer for this Klipper Python environment" % exc)

    def _show_pngs(self, paths, description):
        root = Path(self.chart_config_root).resolve() if self.chart_config_root else None
        for label, filename in paths.items():
            path = Path(filename).resolve()
            if path.suffix.lower() != ".png":
                raise self._error("chart writer returned a non-PNG output")
            self._info(html.escape("%s %s: %s" % (description, label.upper(), path), quote=False))
            try:
                relative = path.relative_to(root) if root is not None else None
            except ValueError:
                relative = None
            if relative is None:
                self._info("PNG saved locally; the Moonraker config root is unverified for this path. "
                           "Open it through this printer's file browser when available")
                continue
            url = "/server/files/config/" + urllib.parse.quote(relative.as_posix(), safe="/")
            def attribute(value):
                # Fluidd linkifies known uppercase G-code tokens before its
                # HTML sanitizer, including tokens inside attributes. Numeric
                # references keep the owned URL/alt intact through that pass.
                return re.sub(r"[A-Z_]", lambda match: "&#%d;" % ord(match.group()),
                              html.escape(value, quote=True))
            escaped = attribute(url)
            alt = attribute("%s: %s" % (description, path.name))
            self.gcode.respond_info('<a href="%s">View PNG: %s<br><img src="%s" alt="%s" '
                                    'style="max-width:100%%;height:auto"></a>' % (
                                        escaped, alt, escaped, alt))

    def _status(self, name):
        obj = self.printer.lookup_object(name, None)
        if obj is not None and hasattr(obj, "get_status"):
            return obj.get_status(self.reactor.monotonic())
        return {}

    def _printing(self):
        if self._status("print_stats").get("state") in ("printing", "paused"):
            return True
        if self._status("pause_resume").get("is_paused", False):
            return True
        virtual_sd = self.printer.lookup_object("virtual_sdcard", None)
        return (virtual_sd is not None and hasattr(virtual_sd, "is_active")
                and virtual_sd.is_active())

    @staticmethod
    def _bound_handler(handler, owner, function):
        return (type(handler) is types.MethodType and handler.__self__ is owner
                and handler.__func__ is function)

    def _stock_g28(self):
        """Resolve stock G28 directly or beneath one verified built-in wrapper."""
        if self.printer.lookup_object("gcode_macro G28", None) is not None:
            raise self._error("workflows require stock G28; a G28 macro is configured")
        normal = self.printer.lookup_object("homing", None)
        stock = getattr(normal, "cmd_G28", None)
        if (_PrinterHoming is None or type(normal) is not _PrinterHoming
                or getattr(normal, "printer", None) is not self.printer
                or not self._bound_handler(stock, normal, _PRINTER_G28)):
            raise self._error("cannot verify the stock G28 homing handler")
        registries = [getattr(self.gcode, name, None)
                      for name in ("ready_gcode_handlers", "gcode_handlers")]
        if any(not isinstance(handlers, dict) or "G28" not in handlers
               for handlers in registries):
            raise self._error("cannot verify the active and ready G28 handlers")
        safe = self.printer.lookup_object("safe_z_home", None)
        override = self.printer.lookup_object("homing_override", None)
        if safe is not None and override is not None:
            raise self._error("workflows cannot combine safe_z_home and homing_override G28 wrappers")
        if override is not None:
            if (_HomingOverride is None or type(override) is not _HomingOverride
                    or getattr(override, "printer", None) is not self.printer
                    or getattr(override, "gcode", None) is not self.gcode
                    or not self._bound_handler(getattr(override, "cmd_G28", None),
                                               override, _OVERRIDE_G28)
                    or not all(self._bound_handler(handlers["G28"], override, _OVERRIDE_G28)
                               for handlers in registries)
                    or not self._bound_handler(getattr(override, "prev_G28", None),
                                               normal, _PRINTER_G28)):
                raise self._error("workflows require built-in homing_override with an unmodified "
                                  "stock G28 predecessor; custom or nested wrappers are unsupported")
            if getattr(override, "in_script", None) is not False:
                raise self._error("workflows cannot run from an active or unverifiable homing_override G28 script")
        elif safe is None:
            if not all(self._bound_handler(handlers["G28"], normal, _PRINTER_G28)
                       for handlers in registries):
                raise self._error("workflows require the unmodified stock G28 handler")
        elif (_SafeZHoming is None or type(safe) is not _SafeZHoming
              or getattr(safe, "printer", None) is not self.printer
              or getattr(safe, "gcode", None) is not self.gcode
              or not self._bound_handler(getattr(safe, "cmd_G28", None), safe, _SAFE_Z_G28)
              or not all(self._bound_handler(handlers["G28"], safe, _SAFE_Z_G28)
                         for handlers in registries)
              or not self._bound_handler(getattr(safe, "prev_G28", None), normal, _PRINTER_G28)):
            raise self._error("workflows require built-in safe_z_home with an unmodified stock "
                              "G28 predecessor; custom or nested wrappers are unsupported")
        if not callable(getattr(self.gcode, "create_gcode_command", None)):
            raise self._error("cannot create a verified X/Y G28 command")
        return stock

    def _preflight(self, axes, continuation=False):
        if hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown():
            raise self._error("printer is shut down")
        if self._printing():
            raise self._error("cannot run while printing or paused")
        if not continuation and self._status("idle_timeout").get("state") == "Printing":
            raise self._error("printer is busy; wait until it is idle")
        if self.owner.homing_active or self.owner.attempt is not None:
            raise self._error("another homing operation is active")
        self._stock_g28()
        toolhead = self.owner.toolhead
        if toolhead is None:
            raise self._error("printer is not connected")
        self.owner.preflight_motion_policy(axes, error=self.printer.command_error)
        kin = toolhead.get_kinematics()
        name = self.kinematics_name
        if name is None:
            name = kin.__class__.__module__.rsplit(".", 1)[-1]
        if name not in ("cartesian", "corexy"):
            raise self._error("supervised workflows currently require cartesian or corexy "
                              "kinematics with Z independent of XY")
        if getattr(kin, "dual_carriage_axis", None) is not None:
            raise self._error("dual-carriage workflows are not supported")
        if not (isinstance(getattr(kin, "limits", None), list) and len(kin.limits) == 3):
            raise self._error("this kinematics cannot invalidate XY while retaining Z")
        self._xy_enable_lines()
        for axis in axes:
            rail = self.owner.rails_by_axis.get(axis)
            if rail is None or axis not in self.owner.endstops:
                raise self._error("axis %s must use this module's accelerometer endstop" % axis.upper())
            info = rail.get_homing_info()
            if info.retract_dist or getattr(info, "min_home_dist", 0.0):
                raise self._error("workflow axes require homing_retract_dist: 0 and min_home_dist: 0")
            low, high = rail.get_range()
            center = (low + high) * 0.5
            distance = (info.position_endstop - center) * (1.0 if info.positive_dir else -1.0)
            if not all(math.isfinite(v) for v in (low, high, info.position_endstop)) \
                    or high <= low or distance <= MIN_TRAVEL \
                    or not low <= info.position_endstop <= high:
                raise self._error("axis %s has invalid endstop, direction or insufficient center travel"
                                  % axis.upper())
            if not math.isfinite(info.speed) or info.speed <= 0.0:
                raise self._error("axis homing speed must be positive and finite")
        if hasattr(toolhead, "check_busy"):
            queued, estimated, empty = toolhead.check_busy(self.reactor.monotonic())
            if not empty or queued > estimated + 0.1:
                raise self._error("queued motion is still active")

    def _xy_enable_lines(self):
        toolhead = self.owner.toolhead
        manager = self.printer.lookup_object("stepper_enable", None)
        if toolhead is None or manager is None:
            raise self._error("stepper enable controls are unavailable")
        lines, z_lines = [], []
        for stepper in toolhead.get_kinematics().get_steppers():
            if not hasattr(stepper, "is_active_axis"):
                raise self._error("cannot establish which steppers affect Z")
            xy = any(stepper.is_active_axis(a) for a in "xy")
            z = stepper.is_active_axis("z")
            line = manager.lookup_enable(stepper.get_name())
            if xy and z:
                raise self._error("an XY stepper also drives Z; selective release is unsafe")
            if xy:
                lines.append(line)
            if z:
                z_lines.append(line)
        if not lines:
            raise self._error("no XY stepper enable lines were found")
        if any(line is other or getattr(line, "enable", object()) is getattr(other, "enable", None)
               for line in lines for other in z_lines):
            raise self._error("XY and Z share an enable line; cannot release XY independently")
        if any(hasattr(line, "has_dedicated_enable") and not line.has_dedicated_enable()
               and getattr(getattr(line, "enable", None), "mcu_enable", True) is None
               for line in lines):
            raise self._error("an XY motor has no controllable enable pin")
        return list(dict.fromkeys(lines))

    def _disable_xy(self):
        toolhead = self.owner.toolhead
        kin = toolhead.get_kinematics()
        limits = getattr(kin, "limits", None)
        if not isinstance(limits, list) or len(limits) != 3:
            raise self._error("this kinematics cannot invalidate XY while retaining Z")
        lines = self._xy_enable_lines()
        z_limit = limits[2]
        z_homed = "z" in kin.get_status(self.reactor.monotonic()).get("homed_axes", "")
        toolhead.wait_moves()
        toolhead.flush_step_generation()
        toolhead.dwell(0.1)
        print_time = toolhead.get_last_move_time()
        for line in lines:
            line.motor_disable(print_time)
        toolhead.dwell(0.1)
        # Stock cartesian/corexy use these ranges for homed state. The public
        # clear_homing_state API differs between Klipper (letters) and Kalico
        # (integer indices), so invalidate only the two verified XY ranges.
        limits[0] = limits[1] = (1.0, -1.0)
        homed = kin.get_status(self.reactor.monotonic()).get("homed_axes", "")
        if any(axis in homed for axis in AXES) or ("z" in homed) != z_homed \
                or kin.limits[2] != z_limit:
            raise self._error("could not invalidate XY while retaining Z state")
        toolhead.wait_moves()

    def _geometry(self, axis):
        rail = self.owner.rails_by_axis[axis]
        low, high = rail.get_range()
        center = (low + high) * 0.5
        return center, abs(center - rail.get_homing_info().position_endstop)

    @staticmethod
    def _calibration_recovery_status(session):
        return {"count": session["recovery"]["count"],
                "last_return_mm_signed": session["recovery"]["last_return_mm_signed"],
                "accounting": "completed_commanded_returns", "physical_position_verified": False}

    def _calibration_attempt_status(self, session):
        return {"attempt_limit": self.max_attempts,
                "attempts_remaining": max(0, self.max_attempts - session["attempts"]),
                "candidate_attempts_required": REQUIRED_SUCCESSES + 1}

    def _require_calibration_candidate_attempts(self, session):
        budget = self._calibration_attempt_status(session)
        if budget["attempts_remaining"] < budget["candidate_attempts_required"]:
            raise self._error(
                "calibration attempt budget cannot complete another candidate: %d attempts remain, "
                "%d required for fresh impact discovery and ten checks. No automatic return or "
                "threshold increase" % (budget["attempts_remaining"], budget["candidate_attempts_required"]))

    def _model_context(self, axis):
        rail = self.owner.rails_by_axis[axis]
        low, high = rail.get_range()
        info = rail.get_homing_info()
        backend = getattr(self.owner, "backend", None)
        sensor = getattr(backend, "sensor", None)
        helper_config = getattr(getattr(self.owner, "helper", None), "config", None)
        mapping = getattr(sensor, "axes_map", None) if backend is not None else \
            getattr(helper_config, "axes_map", None)
        # JSON normalizes tuples from drivers so saved and live mapping compare.
        mapping = json.loads(json.dumps(mapping, allow_nan=False))
        return {"sensor": getattr(self.owner, "sensor_name", None),
                "backend": "beacon_raw" if backend is None else backend.kind,
                "sensor_axes_map": mapping,
                "geometry": {"minimum": low, "maximum": high,
                             "endstop": info.position_endstop,
                             "positive_dir": bool(info.positive_dir), "center": (low + high) * 0.5},
                "homing_speed_mm_s": info.speed,
                "homing_accel_mm_s2": self.owner.homing_accel,
                "impact_sign": self.owner.impact_sign[axis]}

    def _validate_model_connection(self):
        if self.model is None:
            return
        for axis in self.model.thresholds:
            if axis in self.manual_axes:
                continue
            if axis not in self.owner.rails_by_axis:
                raise self.printer.config_error("Saved %s model has no configured accelerometer endstop"
                                                % axis.upper())
            saved = self.model.metadata[axis]
            current = self._model_context(axis)
            for key, value in current.items():
                if key not in saved or saved[key] != value:
                    raise self.printer.config_error(
                        "Saved %s homing model does not match current %s; recalibrate "
                        "or set an explicit %s_threshold override" % (axis.upper(), key, axis))

    def _snapshot(self):
        return (tuple(self.owner.toolhead.get_position()),
                self._status("toolhead").get("homed_axes", ""),
                tuple(sorted(self.owner.threshold_g.items())),
                tuple(sorted(self.owner.impact_sign.items())),
                self.owner.homing_accel, self.owner.back_off_dist)

    def _restore(self):
        session = self.session
        if session is None:
            return
        if session["mode"] == "noise":
            if session.get("sensor_started"):
                self.owner.stop_noise_measurement()
            session["sensor_stopped"] = True
            return
        try:
            self.owner._end_session()
        finally:
            self.owner.threshold_g.update(session["saved_thresholds"])
            self.owner.back_off_dist = session["saved_back_off"]
            self.in_home, self.home_axis = False, None

    def _finish(self, result, release=False):
        session = self.session
        if session is not None and session.get("finishing"):
            session["finish_interrupted"] = result.get("reason", "workflow interrupted during PNG export")
            return
        if session is not None:
            if session.get("accuracy_recovery") is not None:
                result = dict(result, accuracy_recovery=dict(session["accuracy_recovery"]),
                              source_accuracy=session["source_accuracy"])
            if session["mode"] == "calibration":
                result = dict(result, **{key: session[key] for key in (
                    "start_threshold_g", "threshold_step_g", "quiet_range_enforced",
                    "calibration_settle_time_s")})
                result.update(candidate_g=session["candidate"], restarts=session["restarts"],
                              attempts=session["attempts"], **self._calibration_attempt_status(session),
                              recovery=self._calibration_recovery_status(session))
                if session.get("beta"):
                    result.update(beta=True, beta_phase=session.get("beta_phase"),
                                  motion_noise=session.get("beta_motion_summary"),
                                  motion_plan=session.get("beta_motion_plan"),
                                  repeatability_spread_mm=session.get("repeatability_spread_mm"))
                if session.get("stationary_noise") is not None:
                    result["stationary_noise"] = dict(session["stationary_noise"])
            session["finishing"] = True
            session["finish_result"] = result.get("result")
        try:
            if (session is not None and session["mode"] == "calibration"
                    and result.get("result") != "calibrated"):
                self._invalidate_calibration_axis(session)
            self._restore()
            if release:
                try:
                    self._disable_xy()
                except Exception:
                    logging.exception("accel_homing: unable to release/invalidate XY during cleanup")
        except Exception as exc:
            if session is not None and session.get("beta"):
                session["beta_cleanup_error"] = str(exc)
                result = dict(result, result="aborted", proposed_threshold_g=None,
                              operation_reason=result.get("reason"),
                              reason="%sBeta cleanup failed: %s" % (
                                  (str(result["reason"]) + "; ") if result.get("reason") else "", exc))
            elif session is not None and session.get("accuracy_recovery") is not None:
                result = dict(result, result="aborted", save_config=False,
                              operation_reason=result.get("reason"),
                              reason="recovery calibration cleanup failed: %s" % exc)
            elif session is not None and session["mode"] in ("accuracy", "noise"):
                result = {"result": "aborted", "reason": "workflow cleanup failed: %s" % exc}
            raise
        finally:
            try:
                if session is not None:
                    session["finish_result"] = result.get("result")
                if session is not None and session["mode"] == "accuracy":
                    result = self._accuracy_result(session, result)
                elif session is not None and session["mode"] == "noise":
                    result = self._noise_result(session, result)
                if session is not None and session.get("beta"):
                    # The command finalizes after central-frame/current cleanup
                    # unwinds too, including cancellation inside a yielding call.
                    session["beta_pending_result"] = result
            finally:
                self.last_result = result
                if session is None or not (session.get("beta")
                                           or session.get("accuracy_recovery_pending")):
                    self.session = None
                self.reactor.update_timer(self.timer, self.reactor.NEVER)

    def _shutdown(self, *args):
        self.preflight_generation += 1
        self._clear_noise_seeds()
        if self.session is not None:
            self._finish({"result": "aborted", "reason": "printer shutdown"})

    def _disconnect(self, *args):
        self.preflight_generation += 1
        self._clear_noise_seeds()
        if self.session is not None:
            self._finish({"result": "aborted", "reason": "printer disconnected"})

    def _timeout(self, eventtime):
        if self.session is None or self.session.get("state") != "waiting":
            return self.reactor.NEVER
        if eventtime >= self.session["expires"]:
            self._finish({"result": "cancelled", "reason": "manual-centering wait expired"})
            self._info("manual-centering wait expired; workflow cancelled")
            return self.reactor.NEVER
        return self.session["expires"]

    def check_home_axes(self, axes):
        """Called by core before an accelerometer homing session can move."""
        if self.session is not None and (not self.in_home or set(axes) != {self.home_axis}):
            raise self._error("a supervised workflow is active; resume calibration with its "
                              "command or use CANCEL")

    def check_home_result(self, record):
        # Single-touch geometry is enforced before any workflow motion. Record
        # qualification therefore happens immediately after stock G28 returns;
        # it does not force stock G28's all-motor fault cleanup for an early hit.
        if self.in_home and record.get("axis", "").lower() != self.home_axis:
            raise self._error("unexpected homing result axis")

    def _invalidate_calibration_axis(self, session):
        if session.get("axis_touched"):
            # Cartesian/CoreXY limits were verified by workflow preflight.
            # Do not certify a rejected trigger, or release the other motors.
            self.owner.toolhead.get_kinematics().limits[AXES[session["axis"]]] = (1.0, -1.0)

    def _mark_implausible_calibration(self, axis, source_record, reason):
        session = self.session
        if (not isinstance(session, dict) or session.get("mode") != "calibration"
                or session.get("state") != "running" or session.get("axis") != axis
                or not session.get("axis_touched") or self.owner.last.get(axis) is not source_record
                or source_record is session.get("home_old_record")
                or session.get("home_generation") != self.owner.operation_generation):
            raise self._error("implausible travel has no current calibration home evidence")
        message = (reason + "; calibration aborted without an automatic return or threshold "
                   "increase; selected-axis home is invalid. Manually recenter XY and "
                   "start a new ACCEL_HOMING_CALIBRATE command with CENTERED=1")
        announce = not session.get("axis_only_cleanup", False)
        session["axis_only_cleanup"] = {"source": source_record, "attempt": session["attempts"],
                                        "generation": session["home_generation"]}
        if announce:
            # Retain this instruction even if later capture/stream cleanup fails.
            self._info(message)
        return message

    def note_calibration_halt(self, record, _positions):
        """Retain the current recorded displacement before stock G28 cleanup."""
        axis = record.get("axis")
        session = self.session
        if (session is not None and session.get("mode") == "accuracy"
                and self.in_home and self.home_axis == axis):
            session["accuracy_halt"] = {
                "source": record, "generation": self.owner.operation_generation,
                "attempt": session["attempts"],
                "trigger": record.get("trigger_travel_mm_signed"),
                "halt": record.get("halt_travel_mm_signed"),
                "positions": tuple((sp.stepper, sp.start_pos, sp.trig_pos, sp.halt_pos)
                                   for sp in _positions)}
            return
        if not self.calibration_reference(axis):
            return
        quality, reason = self._quality(axis, record)
        if quality == "implausible":
            self._mark_implausible_calibration(axis, record, reason)
            return
        if quality != "early":
            return
        session = self.session
        session["early_return_cleanup"] = True
        session["early_halt"] = {
            "source": record, "generation": self.owner.operation_generation,
            "attempt": session["attempts"],
            "halt_travel_mm_signed": record.get("halt_travel_mm_signed")}

    def check_settings_change(self):
        if self.session is not None:
            raise self._error("cancel the active workflow before changing homing settings")
        self._clear_noise_seeds()

    def _require_centered(self, gcmd):
        if str(gcmd.get("CENTERED", "")).strip() != "1":
            raise gcmd.error("Manually center XY first, then run this command with CENTERED=1")

    def _require_unhomed_xy(self):
        homed = self.owner.toolhead.get_kinematics().get_status(
            self.reactor.monotonic()).get("homed_axes", "")
        if any(axis in homed for axis in AXES):
            raise self._error("workflow requires both X and Y unhomed; release XY and "
                              "manually center the toolhead before running with CENTERED=1")

    def calibration_reference(self, axis):
        """Permit amplitude-independent reference acquisition only inside this trial."""
        session = self.session
        return bool(axis in AXES and isinstance(session, dict)
                    and session.get("mode") == "calibration" and session.get("state") == "running"
                    and session.get("axis") == axis and self.in_home is True and self.home_axis == axis)

    def simple_detection(self, axis):
        if (self.calibration_reference(axis)
                and self.session.get("algorithm") == SIMPLE_CALIBRATION):
            return True
        model = self.model
        return bool(model is not None and axis not in self.manual_axes
                    and axis in model.thresholds
                    and model.metadata[axis].get("impact_metric") == "xyz_magnitude")

    def _calibration_grid(self, gcmd, session=None):
        defaults = session or {"start_threshold_g": self.start_threshold,
                               "threshold_step_g": self.threshold_step}
        grid = {}
        for parameter, key, minimum, maximum in (
                ("START_THRESHOLD", "start_threshold_g", 0.5, None),
                ("STEP", "threshold_step_g", 0.1, 3.0)):
            value = gcmd.get_float(parameter, defaults[key], minval=minimum, maxval=maximum)
            if not math.isfinite(value):
                raise gcmd.error("%s must be finite" % parameter)
            if session is not None and value != session[key]:
                raise gcmd.error("%s cannot change while resuming calibration" % parameter)
            grid[key] = value
        self._validate_calibration_threshold(grid["start_threshold_g"], "START_THRESHOLD", gcmd.error)
        grid["explicit_calibration_grid"] = any(gcmd.get(key, None) is not None
                                                for key in ("START_THRESHOLD", "STEP"))
        grid["explicit_start_threshold"] = gcmd.get("START_THRESHOLD", None) is not None
        return grid

    def _begin(self, gcmd, mode, axis=None, axes=None, calibration_grid=None, save_config_default=1,
               previous_accuracy=None):
        if self.session is not None and self.session is not previous_accuracy:
            raise gcmd.error("A workflow is already active; use ACCEL_HOMING_CANCEL first")
        if previous_accuracy is not None:
            self._check_accuracy_recovery(previous_accuracy)
        if mode == "calibration" and calibration_grid is None:
            calibration_grid = self._calibration_grid(gcmd)
        axes = axes if axes is not None else [axis] if axis else ["x", "y"]
        self._preflight(axes, continuation=previous_accuracy is not None)
        if mode == "calibration" and self._geometry(axis)[1] <= CALIBRATION_MIN_TRAVEL_MM:
            raise gcmd.error("center-to-endstop travel must exceed 20 mm for impact discovery")
        self._require_unhomed_xy()
        if mode in ("charts", "accuracy") and self.chart_preflight_check is not None:
            self.chart_preflight_check()
        if previous_accuracy is not None:
            self._check_accuracy_recovery(previous_accuracy)
        self.session = {"mode": mode, "axis": axis, "state": "starting",
                        "run_id": uuid.uuid4().hex, "successes": 0, "attempts": 0,
                        "candidate_qualified": False, "candidate_attempt": None,
                        "validation_records": [], "repeatability_spread_mm": None,
                        "records": [], "paths": [], "candidate": self.start_threshold,
                        "saved_thresholds": dict(self.owner.threshold_g),
                        "saved_back_off": self.owner.back_off_dist,
                        "save_config": (mode == "calibration" and bool(gcmd.get_int(
                            "SAVE_CONFIG", save_config_default, minval=0, maxval=1)))}
        if mode == "calibration":
            self.session.update(calibration_grid, candidate=calibration_grid["start_threshold_g"],
                                algorithm=SIMPLE_CALIBRATION, restarts=0,
                                quiet_range_enforced=False,
                                calibration_settle_time_s=self.calibration_settle_time,
                                recovery={"count": 0, "last_return_mm_signed": None})
            self._clear_noise_seeds()

    def _announce_calibration(self, session):
        self._info("%s calibration starts at %.6g g with %.6g g steps; XYZ magnitude; "
                   "impact after >20 mm then ten centered endstop checks with spread <= 2 mm; "
                   "no quietness or acceleration-derived motion proof; returns settle for %.3f s; "
                   "early triggers reverse their recorded signed start-to-halt travel; "
                   "%d total homing attempts, with %d required before another candidate" % (
                       session["axis"].upper(), session["candidate"], session["threshold_step_g"],
                        session["calibration_settle_time_s"],
                        self.max_attempts, REQUIRED_SUCCESSES + 1))

    def _wait_center(self, reason):
        self._restore()
        self._disable_xy()
        session = self.session
        session["state"] = "waiting"
        session["expires"] = self.reactor.monotonic() + self.confirm_timeout
        session["snapshot"] = self._snapshot()
        self.reactor.update_timer(self.timer, session["expires"])
        centers = {session["axis"]: self._geometry(session["axis"])[0]}
        instruction = "Move XY manually to the center of their travel, then repeat the command. "
        instruction += "Expected centers: " + ", ".join("%s=%.2f mm" % (a.upper(), v)
                                                       for a, v in centers.items())
        command = "ACCEL_HOMING_CALIBRATE AXIS=%s CENTERED=1" % session["axis"].upper()
        self._info("%s. %s Command: %s (expires in %.0f s)" % (
            reason, instruction, command, self.confirm_timeout))

    def _threshold_limit(self):
        scale = getattr(self.owner, "full_scale_g", None)
        if scale is None and getattr(self.owner, "helper", None) is not None:
            scale = self.owner._full_scale_g(self.owner.scale)
        if scale is None:
            return None
        if not math.isfinite(scale) or scale <= 0.0:
            raise self._error("known accelerometer range must be finite and positive")
        # Match the core threshold validator's existing conservative headroom
        # policy. This is separate from actual clipped/saturated-sample checks.
        return 0.8 * scale

    def _validate_calibration_threshold(self, value, name, error=None):
        error = error or self._error
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0.5:
            raise error("%s must be finite and at least 0.5 g" % name)
        limit = self._threshold_limit()
        if limit is not None and value > limit:
            raise error("%s exceeds the calibration sensor-range headroom ceiling %.6g g "
                        "(80%% of the known accelerometer range)" % (name, limit))

    def _clear_noise_seeds(self):
        self.noise_recommendation = self.noise_seed_context = None
        self.noise_seeds = {}

    def _noise_context(self):
        return {"sensor": self.owner.noise_sensor_context(),
                "calibration_start_threshold": self.start_threshold,
                "calibration_step": self.threshold_step,
                "calibration_settle_time_s": self.calibration_settle_time,
                "axes": {axis: self._model_context(axis)
                         for axis in AXES if axis in self.owner.rails_by_axis}}

    def _consume_noise_seed(self, axis):
        if self.noise_seed_context is None:
            return None
        if self._noise_context() != self.noise_seed_context:
            self._clear_noise_seeds()
            self._info("Stationary noise context changed; using the configured calibration start")
            return None
        seed = self.noise_seeds.pop(axis, None)
        if seed is not None:
            self._info("%s calibration starts at %.6g g from stationary noise run %s. "
                       "This is an initial candidate; fresh-sample checks and ten-home "
                       "qualification remain required" % (
                           axis.upper(), seed, self.noise_recommendation["run_id"]))
        return seed

    def _noise_snapshot(self):
        manager = self.printer.lookup_object("stepper_enable", None)
        kin = self.owner.toolhead.get_kinematics()
        if manager is None or not callable(getattr(kin, "get_steppers", None)):
            raise self._error("cannot establish stationary motor enable state")
        motors = []
        for stepper in kin.get_steppers():
            name = stepper.get_name()
            line = manager.lookup_enable(name)
            if not callable(getattr(line, "is_motor_enabled", None)):
                raise self._error("cannot read motor enable state for %s" % name)
            motors.append((name, bool(line.is_motor_enabled())))
        return (tuple(self.owner.toolhead.get_position()),
                self._status("toolhead").get("homed_axes", ""),
                tuple(sorted(self.owner.threshold_g.items())),
                tuple(sorted(self.owner.impact_sign.items())),
                self.owner.homing_accel, self.owner.back_off_dist, tuple(sorted(motors)))

    def _preflight_noise(self, session=None):
        if self.session is not None and self.session is not session:
            raise self._error("a workflow is already active; use ACCEL_HOMING_CANCEL first")
        if session is not None:
            self._check_running(session)
        if hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown():
            raise self._error("printer is shut down")
        continuation = session is not None and bool(session.get("accuracy_recovery"))
        if self._printing() or (not continuation
                               and self._status("idle_timeout").get("state") == "Printing"):
            raise self._error("noise measurement requires an idle printer, not printing or paused")
        if self.owner.toolhead is None:
            raise self._error("printer is not connected")
        if self.owner.homing_active or self.owner.attempt is not None:
            raise self._error("another homing operation is active")
        self._noise_idle_queue()

    def _noise_idle_queue(self):
        toolhead = self.owner.toolhead
        if not callable(getattr(toolhead, "check_busy", None)):
            raise self._error("cannot establish that the stationary motion queue is empty")
        queued, estimated, empty = toolhead.check_busy(self.reactor.monotonic())
        if (not all(math.isfinite(value) for value in (queued, estimated))
                or not empty or queued > estimated + 0.000001):
            raise self._error("queued motion is active during stationary noise measurement")

    def _check_noise_running(self, session):
        self._check_running(session)
        if self.owner.operation_generation != session["noise_generation"]:
            raise self._error("stationary noise was interrupted")
        if (session.get("sensor_token") is None
                or self.owner.noise_session is not session["sensor_token"]):
            raise self._error("stationary sensor session was cancelled or disconnected")
        self._noise_idle_queue()
        if (self._noise_snapshot() != session["snapshot"]
                or self._noise_context() != session["context"]):
            raise self._error("printer or sensor context changed during stationary noise measurement")

    def _check_stopped_noise_context(self, session):
        self._noise_idle_queue()
        if (self.owner.operation_generation != session["noise_generation"]
                or self._noise_snapshot() != session["snapshot"]
                or self._noise_context() != session["context"]):
            raise self._error("stationary context was interrupted or changed during PNG export")

    def _noise_record(self, session, iteration, capture):
        samples = capture.get("samples", ())
        context = session["context"]["sensor"]
        cutoff = capture.get("cutoff_time_s")
        if (capture.get("dropped", 0) or capture.get("fault") or capture.get("error")
                or capture.pop("_noise_context", None) != context
                or not isinstance(cutoff, (int, float)) or not math.isfinite(cutoff)
                or len(samples) < 50):
            raise self._error("stationary noise capture is incomplete or lacks current sensor provenance")
        previous = None
        for sample in samples:
            if (not isinstance(sample, (tuple, list)) or len(sample) != 4
                    or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                               and math.isfinite(value) for value in sample)
                    or sample[0] <= cutoff or (previous is not None and sample[0] <= previous)):
                raise self._error("invalid or stale stationary noise samples")
            previous = sample[0]
        if (samples[-1][0] - samples[0][0] + 0.000000001 < NOISE_DURATION
                or (session["iterations"] and
                    samples[0][0] <= session["iterations"][-1]["record"]["last_sample_time_s"])):
            raise self._error("stationary noise windows must be disjoint and span one second each")
        baseline = {axis: math.fsum(row[index + 1] for row in samples) / len(samples)
                    for index, axis in enumerate("xyz")}
        peaks, rms, spread = {}, {}, {}
        for index, axis in enumerate("xyz", 1):
            values = [row[index] for row in samples]
            deltas = [value - baseline[axis] for value in values]
            peaks[axis] = max(abs(value) for value in deltas)
            rms[axis] = math.sqrt(math.fsum(value * value for value in deltas) / len(deltas))
            spread[axis] = max(values) - min(values)
        capture["baseline_g"] = tuple(baseline[axis] for axis in "xyz")
        magnitude_peak = max(math.sqrt(math.fsum((row[index] - baseline[axis]) ** 2
                                                for index, axis in enumerate("xyz", 1)))
                             for row in samples)
        return {"run_id": session["run_id"], "noise_iteration": iteration,
                "duration_s": samples[-1][0] - samples[0][0], "sample_count": len(samples),
                "first_sample_time_s": samples[0][0], "last_sample_time_s": samples[-1][0],
                "baseline_g": baseline, "noise_peak_g": peaks,
                "noise_magnitude_peak_g": magnitude_peak,
                "noise_peak_to_peak_g": spread, "noise_rms_g": rms,
                "sensor": context["sensor"], "backend": context["backend"],
                "full_scale_g": context["full_scale_g"],
                "axis_context": "machine-mapped XYZ after configured axes_map",
                "sensor_context": context}

    def _noise_candidate(self, peak):
        required = max(self.start_threshold, peak * 1.2, peak + 0.1)
        # Keep the existing start+n*step candidate grid; never round a measured
        # peak downward or clamp an unsafe candidate back under the ceiling.
        steps = math.ceil((required - self.start_threshold) / self.threshold_step)
        return self.start_threshold + steps * self.threshold_step

    def _noise_recommend(self, session):
        peaks = {axis: max(item["record"]["noise_peak_g"][axis]
                           for item in session["iterations"]) for axis in "xyz"}
        full_scale = session["context"]["sensor"]["full_scale_g"]
        limit = 0.8 * full_scale if full_scale else None
        starts = {axis: self._noise_candidate(peaks[axis]) for axis in AXES}
        unavailable = [axis for axis, value in starts.items()
                       if limit is None or not math.isfinite(value) or value > limit or value <= peaks[axis]]
        for axis in unavailable:
            starts[axis] = None
        return {"source": "stationary_noise", "method": "initial_candidate_heuristic",
                "noise_peak_g": peaks, "start_threshold_g": starts,
                "margin_factor": 1.2, "margin_g": 0.1,
                "candidate_step_g": self.threshold_step,
                "configured_start_g": self.start_threshold,
                "threshold_limit_g": limit, "unavailable_axes": unavailable, "clamped": False}

    def _noise_result(self, session, ending):
        complete = ending.get("result") == "complete" and len(session["iterations"]) == NOISE_ITERATIONS
        recommendation = session.get("recommendation") if complete else None
        result = {"mode": "noise", "run_id": session["run_id"],
                  "iterations_requested": NOISE_ITERATIONS,
                  "iterations": list(session["iterations"]),
                  "recommendation": recommendation, **ending}
        if session.get("calibration_preflight"):
            result.update(purpose="calibration_preflight", calibration_axis=session["axis"])
        timing_fault = getattr(self.owner, "sample_timing_fault", None)
        if (not complete and timing_fault is not None
                and timing_fault is not session.get("timing_fault_before")):
            result["timing_fault"] = dict(timing_fault)
        try:
            result["summary_paths"] = write_noise(
                self.charts_directory, session["run_id"], session["iterations"],
                recommendation, result["result"], result.get("reason"), **self._chart_options(session))
            self._show_pngs(result["summary_paths"], "Stationary noise")
        except Exception as exc:
            logging.exception("accel_homing: unable to export stationary noise summary")
            result.update(chart_error=str(exc), recommendation=None)
            if result["result"] == "complete":
                result.update(result="aborted", reason="noise summary export failed: %s" % exc)
        return result

    def _begin_noise(self, session=None):
        context = self._noise_context()
        if session is None:
            session = {"run_id": uuid.uuid4().hex}
        else:
            self._check_running(session)
        session.update(mode="noise", state="running", iterations=[], context=context,
                       snapshot=self._noise_snapshot(), noise_generation=self.owner.operation_generation,
                       timing_fault_before=getattr(self.owner, "sample_timing_fault", None),
                       iteration=0, recommendation=None, sensor_started=False, sensor_stopped=True)
        self.session = session
        return session

    def _measure_noise(self, session):
        self._check_running(session)
        self._check_stopped_noise_context(session)
        session["sensor_stopped"] = False
        session["sensor_started"] = True
        if self.owner.start_noise_measurement() != session["context"]["sensor"]:
            raise self._error("sensor context changed before stationary noise measurement")
        session["sensor_token"] = self.owner.noise_session
        self._info("Recording three stationary one-second windows with the current motor/fan state. "
                   "Keep the toolhead still; no homing or motor-state changes are requested")
        for iteration in range(1, NOISE_ITERATIONS + 1):
            session["iteration"] = iteration
            capture = self.owner.collect_noise_window(lambda: self._check_noise_running(session),
                                                      NOISE_DURATION)
            self._check_noise_running(session)
            record = self._noise_record(session, iteration, capture)
            try:
                paths = write_noise_capture(self.charts_directory, session["run_id"],
                                            iteration, capture, record, **self._chart_options(session))
            finally:
                del capture
            session["iterations"].append({"record": record, "capture_paths": paths})
            self._check_noise_running(session)
            self._info("Stationary noise window %d/3 complete: X %.6g g, Y %.6g g, Z %.6g g; "
                       "XYZ magnitude %.6g g after subtracting this window's mean" % (
                           iteration, *(record["noise_peak_g"][axis] for axis in "xyz"),
                           record["noise_magnitude_peak_g"]))
        session["recommendation"] = self._noise_recommend(session)
        self._check_noise_running(session)

    def _calibration_noise_preflight(self, session):
        # _begin has validated CENTERED, the command parameters and the motion
        # policy without moving or changing motor state. Keep this exact active
        # dictionary through stationary acquisition, PNG yields and transition.
        self._begin_noise(session)
        session["calibration_preflight"] = True
        self._chart_preflight()
        continuation = bool(session.get("accuracy_recovery"))
        self._preflight([session["axis"]], continuation=continuation)
        self._require_unhomed_xy()
        self._preflight_noise(session)
        self._check_stopped_noise_context(session)
        self._measure_noise(session)
        self._restore()
        session.update(finishing=True, finish_result="complete")
        try:
            noise_result = self._noise_result(session, {"result": "complete"})
            if noise_result["result"] != "complete":
                raise self._error(noise_result.get("reason", "noise export failed"))
            self._chart_check(session)
        finally:
            session.pop("finishing", None)
            session.pop("finish_result", None)
        # Renderer and sensor setup can yield. The initial manual-centering
        # acknowledgement only remains valid while the full context is intact.
        self._check_running(session)
        self._preflight([session["axis"]], continuation=continuation)
        self._require_unhomed_xy()
        self._check_stopped_noise_context(session)
        peaks = [item["record"].get("noise_magnitude_peak_g") for item in session["iterations"]]
        if (len(peaks) != NOISE_ITERATIONS or any(
                not isinstance(peak, (int, float)) or isinstance(peak, bool)
                or not math.isfinite(peak) or peak < 0.0 for peak in peaks)):
            raise self._error("calibration requires three measured XYZ-magnitude noise peaks")
        peak = max(peaks)
        start, step = session["start_threshold_g"], session["threshold_step_g"]
        # Decimal avoids an extra grid step from binary roundoff at an exact
        # boundary. A noisy candidate must never be rounded or clamped downward.
        start_decimal, step_decimal = Decimal(str(start)), Decimal(str(step))
        peak_decimal = Decimal(str(peak))
        required_decimal = max(start_decimal, Decimal("1.2") * peak_decimal,
                               peak_decimal + Decimal("0.1"))
        required = float(required_decimal)
        steps = math.ceil((required_decimal - start_decimal) / step_decimal)
        candidate = float(start_decimal + steps * step_decimal)
        full_scale = session["context"]["sensor"]["full_scale_g"]
        limit = 0.8 * full_scale if full_scale else None
        explicit = session["explicit_start_threshold"]
        session["stationary_noise"] = {
            "run_id": session["run_id"], "iterations": NOISE_ITERATIONS,
            "impact_metric": "xyz_magnitude", "noise_magnitude_peak_g": peak,
            "initial_threshold_source": "explicit_start_threshold" if explicit else "stationary_noise",
            "start_threshold_g": start if explicit else candidate,
            "summary_paths": dict(noise_result["summary_paths"])}
        if not explicit:
            if limit is None:
                raise self._error("stationary XYZ-magnitude noise cannot select a calibration start "
                                  "without a known sensor range; specify START_THRESHOLD explicitly")
            if not math.isfinite(candidate) or candidate > limit or candidate <= peak:
                raise self._error("stationary XYZ-magnitude noise requires %.6g g, outside the "
                                  "calibration sensor-range headroom ceiling %.6g g "
                                  "(80%% of the known range); calibration aborted" % (
                                      candidate, limit))
            session["noise_seed"] = {
                "run_id": session["run_id"], "source": "stationary_noise",
                "impact_metric": "xyz_magnitude", "noise_magnitude_peak_g": peak,
                "required_threshold_g": required, "start_threshold_g": candidate,
                "configured_start_g": start, "candidate_step_g": step, "threshold_limit_g": limit,
                "margin_factor": 1.2, "margin_g": 0.1}
            session.update(start_threshold_g=candidate, candidate=candidate)
            self._info("Fresh stationary XYZ-magnitude peak %.6g g selects initial candidate %.6g g; "
                       "ten-home qualification remains required" % (peak, candidate))
        else:
            self._info("Fresh stationary XYZ-magnitude peak %.6g g measured; explicit START_THRESHOLD "
                       "%.6g g remains the initial candidate" % (peak, start))
        self._announce_calibration(session)
        self._check_running(session)
        self._check_stopped_noise_context(session)
        session.update(mode="calibration", state="running")

    def cmd_NOISE(self, gcmd):
        self._clear_noise_seeds()
        self._preflight_noise()
        self._chart_preflight()
        self._preflight_noise()
        if gcmd.get("SAVE_CONFIG", None) is not None:
            raise gcmd.error("SAVE_CONFIG is not supported by ACCEL_HOMING_CALIBRATE_NOISE")
        session = self._begin_noise()
        try:
            self._measure_noise(session)
            self._finish({"result": "complete"})
            if self.last_result["result"] != "complete":
                raise self._error(self.last_result.get("reason", "noise export failed"))
            self._check_stopped_noise_context(session)
            self.noise_recommendation = {"run_id": session["run_id"], **session["recommendation"]}
            self.noise_seed_context = session["context"]
            self.noise_seeds = {axis: value for axis, value in
                                session["recommendation"]["start_threshold_g"].items()
                                if value is not None}
            self._info("Noise measurement complete. Per-axis stationary threshold hints: %s. Ordinary "
                       "homing thresholds are unchanged. Each fresh calibration measures its own "
                       "XYZ-magnitude noise; these component hints are not reused" %
                       ", ".join("%s=%s" % (axis.upper(), "unavailable above the safe ceiling" if value is None
                                             else "%.6g g" % value)
                                 for axis, value in session["recommendation"]["start_threshold_g"].items()))
        except Exception as exc:
            self._clear_noise_seeds()
            if self.session is session:
                self._finish({"result": "aborted", "reason": str(exc)})
            raise

    def _home(self, axis, threshold=None):
        if axis not in AXES:
            raise self._error("workflow homing requires exactly one X or Y axis")
        session = self.session
        if session["attempts"] >= session.get("attempt_limit", self.max_attempts):
            raise self._error("maximum homing attempts reached")
        self._check_running(session)
        if threshold is not None:
            self._validate_calibration_threshold(threshold, "candidate")
        session["attempts"] += 1
        if threshold is not None:
            self.owner.threshold_g[axis] = threshold
        self.owner.back_off_dist = 0.0
        old_record = self.owner.last.get(axis)
        if session["mode"] == "accuracy":
            reference = self._accuracy_position_reference()
            session.update(home_old_record=old_record,
                           home_generation=self.owner.operation_generation,
                           accuracy_home_start=reference,
                           accuracy_started_centered=(reference == session.get("accuracy_center_reference")))
            session.pop("accuracy_halt", None)
        if session["mode"] == "calibration":
            session.update(axis_touched=True, early_return_cleanup=False,
                           axis_only_cleanup=False,
                           home_old_record=old_record,
                           home_generation=self.owner.operation_generation)
            session.pop("early_halt", None)
        self.in_home, self.home_axis = True, axis
        capture_started = False
        try:
            try:
                if session.get("beta"):
                    self.owner.start_capture(beta=True)
                else:
                    self.owner.start_capture()
                capture_started = True
                letter = axis.upper()
                gcmd = self.gcode.create_gcode_command("G28", "G28 " + letter, {letter: ""})
                # Public wrappers may hop Z, force coordinates, run SYNC_MOTORS
                # or execute arbitrary homing G-code. Invoke only verified stock
                # X/Y here, retaining public handlers and normal homing events.
                # Recheck after capture setup and command creation can yield.
                self._check_running(session)
                self._stock_g28()(gcmd)
                self.owner.toolhead.wait_moves()
                if session.get("beta"):
                    self.owner.finish_beta_home_capture(lambda: self._check_running(session))
            finally:
                self.in_home, self.home_axis = False, None
                try:
                    self.owner._end_session()
                finally:
                    try:
                        capture = self.owner.stop_capture() if capture_started else {"samples": [], "dropped": 0}
                    finally:
                        if session["mode"] == "calibration" and self.session is not session:
                            self._invalidate_calibration_axis(session)
        except Exception:
            if session["mode"] == "accuracy" and self.session is session:
                self._remember_accuracy_record(session, axis, old_record)
            raise
        self._check_running(session)
        record = self.owner.last.get(axis)
        if record is None or record is old_record:
            raise self._error("G28 returned without a new accelerometer homing result")
        if not isinstance(record, dict) or record.get("axis") != axis:
            raise self._error("G28 returned an unexpected homing result axis")
        source_record = record
        detector_record = capture.pop("_record", None)
        record = dict(record)
        record["run_id"] = session["run_id"]
        record["workflow_attempt"] = session["attempts"]
        record["workflow_threshold_g"] = (threshold if threshold is not None
                                          else self.owner.threshold_g[axis])
        record["workflow_phase"] = ("validation" if session["candidate_qualified"] else "discovery") \
            if session["mode"] == "calibration" else session["mode"]
        calibration_quality, calibration_reason = (self._quality(axis, record)
                                                   if session["mode"] == "calibration" else (None, None))
        early_calibration = calibration_quality == "early"
        if early_calibration:
            session["early_return_cleanup"] = True
        elif calibration_quality == "implausible":
            # Stock G28 assigned an endstop coordinate to an implausible hit.
            # Preserve other-axis/Z state even if capture validation now fails.
            self._mark_implausible_calibration(axis, source_record, calibration_reason)
        if session["mode"] == "accuracy":
            record = self._remember_accuracy_record(session, axis, old_record, record)
        if session["mode"] == "calibration" and capture.get("fault"):
            raise self._error("sensor fault during calibration capture: %s" % capture["fault"])
        if len(capture.get("samples", ())) < 2 or capture.get("dropped", 0):
            raise self._error("homing capture is empty or incomplete; sensor data is not usable")
        previous = None
        for sample in capture["samples"]:
            if len(sample) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v)
                                           for v in sample) \
                    or (previous is not None and sample[0] <= previous):
                raise self._error("homing capture has invalid or stale sensor samples")
            previous = sample[0]
        if session["mode"] in ("accuracy", "charts"):
            self._validate_accuracy_capture(session, axis, source_record, detector_record,
                                            record, capture)
            record["capture_valid"] = True
        elif early_calibration:
            if not isinstance(capture.get("detector"), dict) or capture["detector"].get("impact_metric") != "xyz_magnitude":
                raise self._error("early return requires the current XYZ-magnitude detector capture")
            self._validate_accuracy_capture(session, axis, source_record, detector_record,
                                            record, capture, record["workflow_threshold_g"])
            record["early_capture_valid"] = True
        if session.get("beta"):
            if not early_calibration:
                self._validate_accuracy_capture(session, axis, source_record, detector_record,
                                                record, capture, record["workflow_threshold_g"])
            record["beta_telemetry"] = self.owner.beta_home_telemetry(capture, record)
            session["beta_homes"].append(compact_beta_home(capture, record))
        if session["mode"] != "accuracy":
            session["records"].append(record)
        self._info("%s home: result %s; onset %s g; peak %s g; threshold %s g; "
                   "commanded travel %s mm" % (
            axis.upper(), record.get("result"), record.get("onset_g"),
            record.get("impact_peak_g"), record.get("threshold_g"), record.get("travel_mm")))
        session["terminal_home"] = (axis, record, source_record)
        return record, capture

    def _release_terminal_home(self, axis, record):
        session = self.session
        self._check_running(session)
        current = session.get("terminal_home")
        if (current is None or current[0] != axis or current[1] is not record
                or self._quality(axis, record)[0] != "good"
                or self.owner.last.get(axis) is not current[2]):
            raise self._error("terminal home is not the current qualified result")
        generation = self.owner.operation_generation
        self.owner.release_workflow_home(axis, current[2], session["saved_back_off"], session)
        self.owner.toolhead.wait_moves()
        self._check_running(session)
        if (self.owner.last.get(axis) is not current[2] or self.owner._homed(axis) is not True
                or self.owner.operation_generation != generation):
            raise self._error("terminal home changed or motors were released during final release")

    def _check_running(self, session):
        if session is None or self.session is not session:
            raise self._error("workflow cancelled or interrupted")
        if session.get("beta") and session.get("finishing"):
            raise self._error(session.get("finish_interrupted", "Beta is finishing or was cancelled"))
        if hasattr(self.printer, "is_shutdown") and self.printer.is_shutdown():
            raise self._error("printer is shut down")
        if self._printing():
            raise self._error("printing or pause detected during workflow")

    def _remember_accuracy_record(self, session, axis, old_record, record=None):
        if record is None:
            source = self.owner.last.get(axis)
            if not isinstance(source, dict) or source is old_record:
                return None
            record = dict(source)
        record.update(run_id=session["run_id"], workflow_attempt=session["attempts"],
                      workflow_threshold_g=session["saved_thresholds"][axis],
                      workflow_phase=("discovery" if session["sample_index"] == 0
                                      else "measurement"),
                      accuracy_sample=session["sample_index"], capture_valid=False)
        invalid = [key for key, value in record.items()
                   if isinstance(value, float) and not math.isfinite(value)]
        if invalid:
            # An invalid observed value must never prevent an honest partial
            # failure report from retaining earlier measurements as strict JSON.
            record.update((key, None) for key in invalid)
            record["invalid_numeric_fields"] = invalid
        session["records"].append(record)
        session["pending_record"] = record
        return record

    def _validate_accuracy_capture(self, session, axis, source_record, detector_record,
                                   record, capture, expected_threshold=None):
        detector = capture.get("detector")
        baseline = capture.get("baseline_g")
        arm, onset = record.get("arm_time_s"), record.get("onset_time_s")
        finite = lambda value: (isinstance(value, (int, float)) and not isinstance(value, bool)
                                and math.isfinite(value))
        if expected_threshold is None:
            expected_threshold = session["saved_thresholds"][axis]
        if (detector_record is not source_record or not isinstance(detector, dict)
                or capture.get("fault") or capture.get("error")
                or detector.get("axis") != axis or detector.get("attempt_count") != 1
                or not isinstance(baseline, (tuple, list)) or len(baseline) != 3
                or not all(finite(value) for value in baseline)
                or not finite(arm) or detector.get("arm_time_s") != arm
                or not finite(detector.get("threshold_g")) or detector["threshold_g"] <= 0
                or detector.get("direction") not in (-1, 0, 1)
                or not finite(detector.get("homing_speed_mm_s"))
                or detector["homing_speed_mm_s"] <= 0
                or not finite(detector.get("homing_accel_mm_s2"))
                or detector["homing_accel_mm_s2"] <= 0
                or not math.isclose(detector["threshold_g"], expected_threshold,
                                    rel_tol=1e-12, abs_tol=1e-12)
                or (session.get("last_arm_time") is not None
                    and arm <= session["last_arm_time"])):
            raise self._error("homing capture does not belong to the current detector/reference")
        samples = capture["samples"]
        if not samples[0][0] < arm <= samples[-1][0]:
            raise self._error("homing capture lacks the current pre-arm reference or armed samples")
        if record.get("result") == "impact":
            captured_onset = detector.get("onset_time_s")
            def stamp(value):
                if (not isinstance(value, (tuple, list)) or len(value) != 4
                        or not all(isinstance(item, int) and not isinstance(item, bool) for item in value)
                        or not 0 <= value[0] <= 0xFFFFFFFF or not 0 < value[1] <= 0xFFFFFFFF
                        or value[3] < 2 or not 0 <= value[2] < value[3]):
                    return None
                return tuple(value)
            def missing_impact(reason):
                number = lambda value: value if finite(value) else None
                diagnostics = {"reason": reason, "axis": axis, "sample_count": len(samples),
                               "first_sample_time_s": samples[0][0], "last_sample_time_s": samples[-1][0],
                               "arm_time_s": arm, "recorded_onset_time_s": number(onset),
                               "capture_onset_time_s": number(captured_onset),
                               "onset_sample": stamp(source_record.get("onset_sample")),
                               "capture_onset_sample": stamp(detector.get("onset_sample")),
                               "onset_sample_matches": number(detector.get("onset_sample_matches")),
                               "onset_sample_index": number(detector.get("onset_sample_index")),
                               "onset_g": number(detector.get("onset_g")),
                               "impact_peak_g": number(detector.get("impact_peak_g"))}
                record["capture_diagnostics"] = diagnostics
                detail = lambda value: "unknown" if value is None else "%.17g" % value
                raise self._error("homing capture lacks the current measured impact "
                                  "(%s; recorded onset=%s, capture onset=%s, last sample=%s; "
                                  "matches=%s, index=%s, samples=%d)" % (
                                      reason, detail(diagnostics["recorded_onset_time_s"]),
                                      detail(diagnostics["capture_onset_time_s"]),
                                      detail(diagnostics["last_sample_time_s"]),
                                      detail(diagnostics["onset_sample_matches"]),
                                      detail(diagnostics["onset_sample_index"]), len(samples)))
            if (not finite(onset) or not finite(detector.get("onset_g"))
                    or not finite(detector.get("impact_peak_g")) or detector["impact_peak_g"] < 0):
                missing_impact("invalid measured impact values")
            source_stamp = stamp(source_record.get("onset_sample"))
            capture_stamp = stamp(detector.get("onset_sample"))
            if (type(detector.get("onset_sample_version")) is not int
                    or detector["onset_sample_version"] != 1
                    or source_stamp is None or capture_stamp != source_stamp):
                missing_impact("missing or changed raw onset sample identity")
            index = detector.get("onset_sample_index")
            if (type(detector.get("onset_sample_matches")) is not int
                    or detector["onset_sample_matches"] != 1
                    or type(index) is not int or not 0 <= index < len(samples)):
                missing_impact("onset packet/sample was not retained exactly once")
            if (not finite(captured_onset) or detector.get("trigger_onset_time_s") != onset
                    or samples[index][0] != captured_onset):
                missing_impact("retained onset does not match the current detector/sample")
            if not arm <= onset or not arm <= captured_onset <= samples[-1][0]:
                missing_impact("onset lies outside the armed capture")
            # A later MCU clock adjustment may change chart timestamps.
            # Keep the original homing timestamp and align only this copied
            # report to its proven retained row; the detector is untouched.
            record["trigger_onset_time_s"] = onset
            record["onset_time_s"] = captured_onset
            record["capture_onset_sample_index"] = index
            record["capture_clock_shift_s"] = captured_onset - onset
        record["threshold_g"] = detector["threshold_g"]
        record["onset_g"] = detector.get("onset_g")
        record["impact_peak_g"] = detector.get("impact_peak_g")
        record["direction"] = detector.get("direction")
        record["configured_homing_speed_mm_s"] = self.owner.rails_by_axis[axis].get_homing_info().speed
        record["homing_speed_mm_s"] = detector.get("homing_speed_mm_s")
        record["homing_accel_mm_s2"] = detector.get("homing_accel_mm_s2")
        session["last_arm_time"] = arm

    def _quality(self, axis, record):
        result = record.get("result")
        travel = record.get("travel_mm")
        session = self.session
        if session is not None and session.get("mode") == "accuracy" and "travel_mm_exact" in record:
            exact = record["travel_mm_exact"]
            zero_hit = exact == 0 and travel == 0 and result in ("impact", "pressed")
            if (type(exact) not in (int, float) or not math.isfinite(exact) or exact < 0
                    or (exact == 0 and not zero_hit)):
                return "fatal", "invalid exact commanded travel measurement"
        if session is not None and session.get("mode") == "calibration":
            travel = record.get("travel_mm_exact", travel)
            if (not isinstance(travel, (int, float)) or isinstance(travel, bool)
                    or not math.isfinite(travel) or travel < 0):
                return "fatal", "missing commanded travel measurement"
            if record.get("impact_metric") != "xyz_magnitude":
                return "fatal", "calibration requires the XYZ-magnitude detector"
            if result != "impact":
                return "fatal", "sensor/homing failure (%s)" % result
            if travel <= CALIBRATION_MIN_TRAVEL_MM:
                return "early", "early trigger at %.3f mm (impact must occur after 20 mm)" % travel
            _center, expected = self._geometry(axis)
            minimum = expected * (1.0 - self.travel_tolerance)
            maximum = expected * (1.0 + self.travel_tolerance)
            if math.isfinite(expected) and travel < minimum:
                return "early", ("premature trigger at %.6f mm is below plausible midpoint-to-stop "
                                 "minimum %.6f mm (expected %.6f mm, tolerance %.1f%%)" % (
                                     travel, minimum, expected, self.travel_tolerance * 100.0))
            if not math.isfinite(expected) or travel > maximum:
                return "implausible", ("commanded travel %.6f mm is outside the plausible "
                                       "midpoint-to-stop range %.6f..%.6f mm "
                                       "(expected %.6f mm, tolerance %.1f%%)" % (
                                           travel, minimum, maximum, expected,
                                           self.travel_tolerance * 100.0))
            # Step travel and nominal geometry do not prove physical contact.
            return "good", "impact after 20 mm with plausible midpoint-to-stop commanded travel"
        if result not in ("impact", "pressed"):
            return "fatal", "sensor/homing failure (%s)" % result
        if not isinstance(travel, (int, float)) or not math.isfinite(travel):
            return "fatal", "missing commanded travel measurement"
        if result == "pressed" or travel <= MIN_TRAVEL:
            return "early", "pressed start or early trigger at %.2f mm (must exceed 10 mm)" % travel
        if not (record.get("moving") or record.get("motion_seen")
                or record.get("impact_metric") == "xyz_magnitude"):
            return "fatal", "accelerometer did not establish motion before impact"
        _center, expected = self._geometry(axis)
        if travel < expected * (1.0 - self.travel_tolerance):
            return "early", "trigger at %.2f mm is materially short of expected center travel %.2f mm" % (
                travel, expected)
        if travel > expected * (1.0 + self.travel_tolerance):
            return "fatal", "travel %.2f mm exceeds expected center travel %.2f mm" % (travel, expected)
        return "good", "qualified center-to-stop home"

    @staticmethod
    def _repeatability_spread(records):
        # Remove binary multiplication noise at sub-picometer precision, not
        # the 0.001 mm rounding used for console display. Thus a true 2 mm
        # step difference passes without admitting a 2.0001 mm spread.
        travels = [round(record["travel_mm_exact"], 12) if "travel_mm_exact" in record
                   else record["travel_mm"] for record in records]
        return float(Decimal(str(max(travels))) - Decimal(str(min(travels))))

    def _recenter(self, axis):
        # Called only after _quality accepted this exact axis's current home.
        session = self.session
        self._check_running(session)
        if (self.owner.homing_active or self.owner.attempt is not None
                or self.owner.saved_accel is not None):
            raise self._error("return to center requires completed homing with restored normal acceleration")
        if axis not in self.owner.toolhead.get_kinematics().get_status(
                self.reactor.monotonic()).get("homed_axes", ""):
            raise self._error("axis lost homing before return to center")
        center, _expected = self._geometry(axis)
        coordinates = [None, None, None]
        if session["mode"] == "calibration":
            terminal = session.get("terminal_home")
            if (terminal is None or terminal[0] != axis
                    or self.owner.last.get(axis) is not terminal[2]):
                raise self._error("calibration return has no current measured move")
            if self._quality(axis, terminal[1])[0] != "good":
                raise self._error("calibration return requires a current impact after 20 mm "
                                  "with plausible midpoint-to-stop travel")
            # Stock homing has assigned the selected axis's endstop coordinate.
            # Use its configured midpoint, not the distance from the manually
            # estimated discovery start. The other coordinates remain unchanged.
        coordinates[AXES[axis]] = center
        info = self.owner.rails_by_axis[axis].get_homing_info()
        toolhead = self.owner.toolhead
        # _home -> _end_session restored the ordinary acceleration limit.
        # Calibration and accuracy must return and settle under the same policy.
        # Never restore an unexplained active cap here or reduce homing speed.
        toolhead.manual_move(coordinates, min(self.reposition_speed, info.speed))
        self._check_running(session)
        toolhead.wait_moves()
        self._check_running(session)
        settle = (self.calibration_settle_time
                  if session["mode"] in ("calibration", "accuracy") else 0.0)
        if settle:
            toolhead.dwell(settle)
            self._check_running(session)
            toolhead.wait_moves()
            self._check_running(session)
        if session["mode"] == "accuracy":
            start = session["accuracy_home_start"][0]
            position = toolhead.get_position()
            trusted = (session["accuracy_started_centered"]
                       and self.owner.operation_generation == session["accuracy_generation"]
                       and self.preflight_generation == session["accuracy_preflight_generation"]
                       and all(position[index] == value for index, value in enumerate(start)
                               if index != AXES[axis]))
            session["accuracy_center_reference"] = self._accuracy_position_reference() if trusted else None

    def _next_calibration_candidate(self, reason, apply=True):
        session = self.session
        self._check_running(session)
        candidate = float(Decimal(str(session["candidate"])) + Decimal(str(session["threshold_step_g"])))
        if not math.isfinite(candidate) or candidate <= session["candidate"]:
            raise self._error(reason + "; next calibration candidate must be finite and increase the threshold")
        self._validate_calibration_threshold(candidate, "next calibration candidate")
        self._require_calibration_candidate_attempts(session)
        if apply:
            session.update(candidate=candidate, successes=0, candidate_qualified=False,
                           candidate_attempt=None, validation_records=[], repeatability_spread_mm=None,
                           restarts=session["restarts"] + 1)
        return candidate

    def _return_early_calibration(self, axis, record):
        """Reverse the current recorded commanded displacement; no arrival proof."""
        session = self.session
        self._check_running(session)
        terminal, evidence = session.get("terminal_home"), session.get("early_halt")
        toolhead = self.owner.toolhead
        index = AXES[axis]
        finite = lambda value: (type(value) in (int, float) and math.isfinite(value))
        if (terminal is None or terminal[0] != axis or terminal[1] is not record
                or not isinstance(evidence, dict)
                or evidence.get("source") is not terminal[2]
                or evidence.get("attempt") != session["attempts"]
                or record.get("run_id") != session["run_id"]
                or record.get("workflow_attempt") != session["attempts"]
                or record.get("early_capture_valid") is not True
                or record.get("pass") != 1
                or self._quality(axis, record)[0] != "early"):
            raise self._error("early return lacks current complete halt/capture evidence")
        signed_halt = record.get("halt_travel_mm_signed")
        if not finite(signed_halt) or signed_halt != evidence.get("halt_travel_mm_signed"):
            raise self._error("early return has missing, nonfinite or changed recorded halt displacement")
        generation = session["home_generation"]

        def current():
            self._check_running(session)
            if (self.owner.operation_generation != generation or evidence["generation"] != generation
                    or self.owner.last.get(axis) is not terminal[2]
                    or self.owner.homing_active or self.owner.attempt is not None
                    or getattr(self.owner, "sensor_fault", None) is not None):
                raise self._error("early return was interrupted or its current result changed")

        current()
        self._require_calibration_candidate_attempts(session)
        info = self.owner.rails_by_axis[axis].get_homing_info()
        try:
            self.owner._limit_accel()
            current()
            position = toolhead.get_position()[index]
            if not finite(position) or not finite(position - signed_halt):
                raise self._error("early return requires a finite commanded coordinate and target")
            coordinates = [None, None, None]
            coordinates[index] = position - signed_halt
            # A zero-step early hit is a no-op. The normal firmware move checks
            # govern every nonzero return; no extra recovery envelope is applied.
            if signed_halt:
                toolhead.manual_move(coordinates, min(self.reposition_speed, info.speed))
                current()
                toolhead.wait_moves()
                current()
            if session["calibration_settle_time_s"]:
                toolhead.dwell(session["calibration_settle_time_s"])
                current()
                toolhead.wait_moves()
                current()
        finally:
            self.owner._restore_accel()
        current()
        record["early_return_mm_signed"] = -signed_halt
        record["early_return_physical_position_verified"] = False
        session["recovery"].update(count=session["recovery"]["count"] + 1,
                                   last_return_mm_signed=-signed_halt)

    def _calibrate_trials(self):
        session = self.session
        axis = session["axis"]
        while True:
            discovery = not session["candidate_qualified"]
            record, _capture = self._home(axis, session["candidate"])
            quality, reason = self._quality(axis, record)
            if quality == "implausible":
                raise self._error(self._mark_implausible_calibration(
                    axis, session["terminal_home"][2], reason))
            if quality == "fatal":
                raise self._error(reason + "; calibration aborted without raising the threshold")
            if quality == "early":
                session["early_return_cleanup"] = True
                try:
                    # Threshold/attempt checks must not change the candidate
                    # if the recorded displacement or ordinary firmware move fails.
                    self._next_calibration_candidate(reason, apply=False)
                    self._return_early_calibration(axis, record)
                    candidate = self._next_calibration_candidate(reason)
                finally:
                    self._invalidate_calibration_axis(session)
                self._info(reason + "; reversed commanded start-to-halt travel to the estimated "
                           "starting point; restart impact discovery at %.2f g; completed "
                           "%d returns, last signed return %.6f mm; %d attempts remain" % (
                                candidate, session["recovery"]["count"], session["recovery"]["last_return_mm_signed"],
                                self._calibration_attempt_status(session)["attempts_remaining"]))
                continue
            if discovery:
                session["candidate_qualified"] = True
                session["candidate_attempt"] = record["workflow_attempt"]
                self._info("%s %.2f g candidate selected by impact after %.3f mm (>20 mm); "
                           "centering this axis for ten endstop checks; discovery hit is excluded" % (
                               axis.upper(), session["candidate"],
                               record.get("travel_mm_exact", record["travel_mm"])))
                self._recenter(axis)
                continue
            session["validation_records"].append(record)
            session["successes"] = len(session["validation_records"])
            spread = session["repeatability_spread_mm"] = self._repeatability_spread(session["validation_records"])
            self._info("%s calibration %.2f g: %d/10 endstop checks; commanded travel spread %s mm" % (
                axis.upper(), session["candidate"], session["successes"], spread))
            if spread > CALIBRATION_REPEATABILITY_MM:
                # No later trial can reduce this running max-min spread.
                # Reject immediately, then require fresh discovery and ten checks.
                reason = "%s validation spread %s mm exceeds 2 mm after %d checks" % (
                    axis.upper(), spread, session["successes"])
                candidate = self._next_calibration_candidate(reason)
                session["last_rejected_spread_mm"] = spread
                self._info(reason + "; restart from impact discovery at %.2f g" % candidate)
                self._recenter(axis)
                continue
            if session["successes"] < REQUIRED_SUCCESSES:
                self._recenter(axis)
                continue
            self._save_model()
            return

    def _qualified_validation_records(self):
        """Recheck the entire current-candidate window before any config writes."""
        session = self.session
        message = ("cannot save calibration without a discovery impact after 20 mm and ten distinct "
                   "current-candidate validation homes with commanded travel spread "
                   "<= %.2f mm" % CALIBRATION_REPEATABILITY_MM)
        if not isinstance(session, dict):
            raise self._error(message)
        axis, threshold = session.get("axis"), session.get("candidate")
        threshold_limit = self._threshold_limit()
        candidate_attempt = session.get("candidate_attempt")
        records, qualified = session.get("records"), session.get("validation_records")
        if session.get("mode") != "calibration" or session.get("state") != "running" \
                or not isinstance(axis, str) or axis not in AXES \
                or session.get("candidate_qualified") is not True \
                or session.get("successes") != REQUIRED_SUCCESSES \
                or not isinstance(threshold, (int, float)) or isinstance(threshold, bool) \
                or not math.isfinite(threshold) or threshold < 0.5 \
                or (threshold_limit is not None and threshold > threshold_limit) \
                or type(candidate_attempt) is not int or candidate_attempt < 1 \
                or type(session.get("attempts")) is not int \
                or session.get("attempts") != candidate_attempt + REQUIRED_SUCCESSES \
                or not isinstance(records, list) or len(records) != session["attempts"] \
                or not isinstance(qualified, list) or len(qualified) != REQUIRED_SUCCESSES \
                or not isinstance(session.get("run_id"), str) or not session["run_id"] \
                or type(session.get("save_config")) is not bool:
            raise self._error(message)
        discovery = records[candidate_attempt - 1]
        window = [discovery] + qualified
        if len({id(record) for record in window}) != REQUIRED_SUCCESSES + 1 \
                or any(actual is not expected for actual, expected in
                       zip(qualified, records[-REQUIRED_SUCCESSES:])):
            raise self._error(message)
        for index, record in enumerate(window):
            if not isinstance(record, dict) or record.get("axis") != axis \
                    or record.get("run_id") != session["run_id"] \
                    or type(record.get("workflow_attempt")) is not int \
                    or record.get("workflow_attempt") != candidate_attempt + index \
                    or record.get("workflow_threshold_g") != threshold \
                    or record.get("workflow_phase") != ("discovery" if index == 0 else "validation") \
                    or self._quality(axis, record)[0] != "good":
                raise self._error(message)
        if self._repeatability_spread(qualified) > CALIBRATION_REPEATABILITY_MM:
            raise self._error(message)
        return qualified

    def _save_model(self):
        qualified = self._qualified_validation_records()
        session = self.session
        axis, threshold = session["axis"], session["candidate"]
        minimum = min(r.get("travel_mm_exact", r["travel_mm"]) for r in qualified)
        maximum = max(r.get("travel_mm_exact", r["travel_mm"]) for r in qualified)
        spread = self._repeatability_spread(qualified)
        self._release_terminal_home(axis, qualified[-1])
        # Final release may yield while its motion completes. Recheck all ten
        # accepted trials before staging any model settings.
        self._qualified_validation_records()
        metadata = {"run_id": session["run_id"], **self._model_context(axis),
                    "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "expected_center_travel_mm": self._geometry(axis)[1],
                    "travel_tolerance_fraction": self.travel_tolerance,
                    "min_commanded_travel_mm": minimum,
                    "max_commanded_travel_mm": maximum,
                    "repeatability_spread_mm": spread,
                    "repeatability_limit_mm": CALIBRATION_REPEATABILITY_MM,
                    "calibration_algorithm": SIMPLE_CALIBRATION, "impact_metric": "xyz_magnitude",
                    "minimum_impact_travel_mm": CALIBRATION_MIN_TRAVEL_MM,
                    "discovery_impact_required": True, "recenter_policy": "selected_axis_midpoint",
                    "travel_plausibility_required": True,
                    "recenter_acceleration_policy": "restored_normal_limit",
                    "onset_g": [r.get("onset_g") for r in qualified],
                    "impact_peak_g": [r.get("impact_peak_g") for r in qualified],
                    "calibration_start_threshold_g": session["start_threshold_g"],
                    "calibration_step_g": session["threshold_step_g"],
                    "calibration_quiet_range_enforced": session["quiet_range_enforced"],
                    "calibration_settle_time_s": session["calibration_settle_time_s"],
                    "recovery": self._calibration_recovery_status(session),
                    "physical_position_verified": False}
        if session.get("noise_seed") is not None:
            metadata["stationary_noise_seed"] = dict(session["noise_seed"])
        if session.get("stationary_noise") is not None:
            metadata["stationary_noise_measurement"] = dict(session["stationary_noise"])
        encoded_metadata = json.dumps(metadata, separators=(",", ":"), allow_nan=False)
        configfile = self.printer.lookup_object("configfile")
        recovery = session.get("accuracy_recovery")
        encoded_threshold = ("%.17g" if recovery is not None else "%.6f") % threshold
        configfile.set(self.model_section, "version", "1")
        configfile.set(self.model_section, axis + "_threshold", encoded_threshold)
        configfile.set(self.model_section, axis + "_successes", str(REQUIRED_SUCCESSES))
        configfile.set(self.model_section, axis + "_metadata", encoded_metadata)
        if recovery is not None and axis in self.manual_axes:
            # A model alone loses to explicit shared/axis thresholds on reload.
            # Let native SAVE_CONFIG replace the selected override or report its
            # own include conflict; never edit config/include files directly.
            configfile.set(self.config.get_name(), axis + "_threshold", encoded_threshold)
        result = {"result": "calibrated", "axis": axis, "threshold_g": threshold,
                  "successes": REQUIRED_SUCCESSES, "model": self.model_name,
                  "run_id": session["run_id"], "save_config": session["save_config"],
                  "repeatability_spread_mm": spread,
                  "repeatability_limit_mm": CALIBRATION_REPEATABILITY_MM,
                  "algorithm": SIMPLE_CALIBRATION, "restarts": session["restarts"]}
        if recovery is not None:
            result.update(accuracy_recovery=dict(recovery), source_accuracy=session["source_accuracy"])
        save = session["save_config"]
        self._finish(result)
        self._info("%s model %s calibrated at %.2f g after ten qualified homes with commanded "
                   "travel spread %s mm <= %.2f mm. %s" % (
            axis.upper(), self.model_name, threshold, spread, CALIBRATION_REPEATABILITY_MM,
            "The selected manual override is staged with this model." if recovery is not None and axis in self.manual_axes
            else "Explicit manual threshold overrides remain active." if axis in self.manual_axes
            else "The saved model supplies this axis threshold after restart."))
        if save:
            if recovery is not None:
                self._info("After SAVE_CONFIG succeeds and Klipper restarts, release and manually center XY, "
                           "then rerun: %s" % recovery["rerun_command"])
            self._info("Saving model to printer.cfg SAVE_CONFIG block; Klipper will restart")
            try:
                self.gcode.run_script_from_command("SAVE_CONFIG")
            except Exception as exc:
                self.last_result["result"] = "save_failed"
                self.last_result["save_config"] = False
                if recovery is not None:
                    self.last_result["reason"] = "Recovery calibration passed, but SAVE_CONFIG failed: %s" % exc
                    self._info("Recovery calibration passed, but SAVE_CONFIG failed; persistence and restart "
                               "were not confirmed. Resolve the reported save error before rerunning accuracy")
                raise
        else:
            self._info("Model staged only. Run SAVE_CONFIG to write printer.cfg and restart")

    def cmd_CALIBRATE(self, gcmd):
        if self.session is None:
            self._clear_noise_seeds()
        axis = gcmd.get("AXIS", "").lower()
        if axis not in AXES:
            raise gcmd.error("AXIS must be X or Y")
        self._require_centered(gcmd)
        session = self.session
        if session is not None:
            if session["mode"] != "calibration" or session["axis"] != axis \
                    or session.get("state") != "waiting":
                raise gcmd.error("A workflow is already active; resume calibration on its "
                                 "same axis or use ACCEL_HOMING_CANCEL first")
            if gcmd.get("SAVE_CONFIG", None) is not None and bool(gcmd.get_int(
                    "SAVE_CONFIG", minval=0, maxval=1)) != session["save_config"]:
                raise gcmd.error("SAVE_CONFIG cannot change while resuming calibration")
            self._calibration_grid(gcmd, session)
            self._continue_calibration()
            return
        grid = self._calibration_grid(gcmd)
        self._begin(gcmd, "calibration", axis, calibration_grid=grid)
        self._run_calibration()

    def _run_calibration(self):
        session = self.session
        try:
            self._calibration_noise_preflight(session)
            self._calibrate_trials()
        except Exception as exc:
            if self.session is session:
                self._finish({"result": "aborted", "reason": str(exc)},
                             release=(session["mode"] == "calibration"
                                      and session.get("axis_touched", False)
                                      and not session.get("early_return_cleanup", False)
                                      and not session.get("axis_only_cleanup", False)))
            raise

    def _beta_extractor(self):
        report = self.printer.lookup_object("motion_report", None)
        for name in ("dtrapqs", "trapqs"):
            queues = getattr(report, name, {})
            queue = queues.get("toolhead") if isinstance(queues, dict) else None
            extractor = getattr(queue, "extract_trapq", None)
            if callable(extractor):
                return extractor
        raise self._error("Beta requires the actual toolhead motion_report trapq extractor before motion")

    def _beta_current_context(self, axis):
        settings = self._status("configfile").get("settings", {})
        steppers = {stepper.get_name() for stepper in self.owner.toolhead.get_kinematics().get_steppers()
                    if stepper.is_active_axis(axis)}
        configured = {}
        changed = False
        for name, values in (settings.items() if isinstance(settings, dict) else ()):
            if name.split(" ", 1)[-1] not in steppers or not isinstance(values, dict):
                continue
            currents = {key: values[key] for key in ("run_current", "hold_current", "homing_current")
                        if key in values}
            if currents:
                configured[name] = currents
                if values.get("homing_current") is not None \
                        and values.get("homing_current") != values.get("run_current"):
                    changed = True
        native = getattr(_PRINTER_G28, "__globals__", {}).get("Homing")
        factory = native if isinstance(native, type) else None
        helper = None
        if factory is not None and callable(getattr(factory, "_set_homing_current", None)):
            if not callable(getattr(self.owner.toolhead, "get_active_rails_for_axis", None)):
                raise self._error("Beta native homing-current helper lacks its active-rail API before motion")
            state = factory(self.printer)
            helper = state._set_homing_current
        context = {"configured": configured, "settings_available": isinstance(settings, dict) and bool(settings),
                   "native_helper": None if helper is None else native.__module__ + ".Homing._set_homing_current",
                   "matching": "native_homing_current_helper" if helper is not None else
                               "no_changed_homing_current_exposed", "physical_current_verified": False}
        if self.session is not None and self.session.get("beta"):
            self.session["beta_driver_current_context"] = context
        if changed and helper is None:
            context["matching"] = "unsupported_changed_homing_current"
            raise self._error("Beta found changed homing_current but the installed native homing-current "
                              "helper is unavailable; current conditions cannot be matched before motion")
        return helper, context

    def _beta_motion_plan(self, session):
        axis = session["axis"]
        toolhead = self.owner.toolhead
        extractor = self._beta_extractor()
        for name in ("start_beta_motion_capture", "stop_beta_motion_capture",
                     "finish_beta_home_capture", "beta_home_telemetry", "_enable_motors", "_set_accel"):
            if not callable(getattr(self.owner, name, None)):
                raise self._error("Beta capture capability %s is unavailable before motion" % name)
        if not callable(getattr(toolhead, "set_position", None)):
            raise self._error("Beta requires a temporary selected-axis coordinate before motion")
        current_helper, current_context = self._beta_current_context(axis)
        velocity, normal_accel = toolhead.get_max_velocity()
        info = self.owner.rails_by_axis[axis].get_homing_info()
        speed = min(info.speed, velocity)
        accel = min(normal_accel, self.owner.homing_accel) if self.owner.homing_accel else normal_accel
        rail_accel = getattr(info, "accel", None)
        if rail_accel is not None:
            accel = rail_accel
        low, high = self.owner.rails_by_axis[axis].get_range()
        center = low + (high - low) * 0.5
        sign = 1.0 if info.positive_dir else -1.0
        clearance = (info.position_endstop - center) * sign
        distance = clearance * 0.5
        target = center + sign * distance
        if (not all(math.isfinite(value) for value in (center, clearance, distance, target))
                or clearance <= 0 or not min(center, info.position_endstop) < target < max(center, info.position_endstop)):
            raise self._error("Beta requires a finite central stroke strictly before the configured directional endstop")
        status = toolhead.get_status(self.reactor.monotonic())
        ratio = status.get("minimum_cruise_ratio", getattr(toolhead, "min_cruise_ratio", None))
        if ratio is None and hasattr(toolhead, "max_accel_to_decel"):
            ratio = 1.0 - toolhead.max_accel_to_decel / normal_accel
        if (not all(type(value) in (int, float) and math.isfinite(value) and value > 0
                    for value in (velocity, normal_accel, speed, accel, distance))
                or type(ratio) not in (int, float) or not math.isfinite(ratio) or not 0 <= ratio < 1):
            raise self._error("Beta cannot establish finite speed, acceleration and planner cruise settings")
        # Include the planner's smoothing/minimum-cruise constraint, not merely
        # the unsmoothed v^2/a ramp distance. Actual segments are checked again.
        ramp_distance = speed * speed / (accel * (1.0 - ratio))
        duration = distance / speed + speed / accel
        if not math.isfinite(ramp_distance) or distance <= ramp_distance or not math.isfinite(duration):
            raise self._error("Beta central stroke cannot reach the existing effective homing speed; "
                              "speed was not reduced")
        return {"axis": axis, "center_mm": center,
                "target_mm": target, "selected_endstop_mm": info.position_endstop,
                "center_to_selected_endstop_mm": clearance,
                "distance_mm": distance, "configured_speed_mm_s": info.speed,
                "effective_speed_mm_s": speed, "effective_accel_mm_s2": accel,
                "rail_homing_accel_override_mm_s2": rail_accel,
                "original_accel_mm_s2": normal_accel, "minimum_cruise_ratio": ratio,
                "minimum_duration_s": duration, "extractor": extractor,
                "current_helper": current_helper, "driver_current_context": current_context}

    def _beta_check(self, session):
        self._check_running(session)
        if session.get("finishing"):
            raise self._error(session.get("finish_interrupted", "Beta is finishing or was cancelled"))
        if (self.owner.operation_generation != session["beta_generation"]
                or getattr(self.owner, "sensor_fault", None) is not None):
            raise self._error("Beta was interrupted or the accelerometer reported a fault")

    def _beta_profile(self, plan, start, end, cutoff):
        extracted, references = plan["extractor"](start, end)
        segments = []
        index = AXES[plan["axis"]]
        sign = 1.0 if plan["target_mm"] > plan["center_mm"] else -1.0
        for move in extracted:
            values = (move.print_time, move.move_t, move.start_v, move.accel,
                      move.x_r, move.y_r, move.z_r)
            current_motion = self.session.get("beta_current_motion") if self.session else None
            if isinstance(current_motion, dict):
                observed = current_motion.setdefault("observed_planner_segments", [])
                if len(observed) < 32:
                    observed.append(dict(zip(("start_time_s", "duration_s", "start_velocity_mm_s",
                                              "acceleration_mm_s2", "x_direction", "y_direction", "z_direction"), values)))
            if not all(math.isfinite(value) for value in values) or move.move_t <= 0:
                raise self._error("Beta received an invalid actual planner segment")
            direction = (move.x_r, move.y_r, move.z_r)
            if move.start_v == 0 and move.accel == 0:
                continue
            left, right = max(start, move.print_time), min(end, move.print_time + move.move_t)
            if right <= left:
                continue
            if (not math.isclose(direction[index], sign, abs_tol=1e-9)
                    or any(abs(value) > 1e-9 for i, value in enumerate(direction) if i != index)):
                raise self._error("Beta actual planner profile contains another axis or direction")
            velocity = move.start_v + move.accel * (left - move.print_time)
            phase = "acceleration" if move.accel > 0 else "braking" if move.accel < 0 else "cruise"
            segments.append({"phase": phase, "axis": plan["axis"],
                             "start_time_s": left, "end_time_s": right,
                             "start_velocity_mm_s": velocity, "acceleration_mm_s2": move.accel})
        # Keep the CFFI backing references alive until all segments are copied.
        del references
        segments.sort(key=lambda item: item["start_time_s"])
        phases = {item["phase"] for item in segments}
        if phases != {"acceleration", "cruise", "braking"}:
            raise self._error("Beta actual motion did not contain acceleration, cruise and braking")
        for previous, current in zip(segments, segments[1:]):
            if not math.isclose(previous["end_time_s"], current["start_time_s"], abs_tol=1e-6):
                raise self._error("Beta actual planner profile is discontinuous")
        for item in segments:
            if item["phase"] in ("acceleration", "braking") and not math.isclose(
                    abs(item["acceleration_mm_s2"]), plan["effective_accel_mm_s2"], rel_tol=1e-6, abs_tol=1e-6):
                raise self._error("Beta actual central motion did not match the effective homing acceleration")
            if item["phase"] == "cruise" and not math.isclose(
                    item["start_velocity_mm_s"], plan["effective_speed_mm_s"], rel_tol=1e-6, abs_tol=1e-6):
                raise self._error("Beta actual central motion did not reach the effective homing speed")
        if cutoff < segments[0]["start_time_s"]:
            segments.insert(0, {"phase": "startup", "axis": plan["axis"],
                                "start_time_s": cutoff, "end_time_s": segments[0]["start_time_s"],
                                "start_velocity_mm_s": 0.0, "acceleration_mm_s2": 0.0,
                                "description": "pre-motion startup before commanded acceleration"})
        return segments

    def _beta_motion_noise(self, session, plan):
        toolhead = self.owner.toolhead
        kin = toolhead.get_kinematics()
        index = AXES[session["axis"]]
        original_position = toolhead.get_position()[index]
        original_limit = kin.limits[index]
        completed = False
        failure = None
        current_attempted = False
        restore = session.setdefault("beta_restore", {})
        restore.update(central_selected_frame_restored=False, central_selected_limit_restored=False,
                       central_profile_accel_restored=False,
                       central_driver_current_restored=plan["current_helper"] is None)
        session["beta_phase"] = "motion_noise"
        try:
            # CENTERED=1 supplies this estimated coordinate. No Z or other-axis
            # coordinate/limit is rewritten, including during cleanup.
            position = toolhead.get_position()
            position[index] = plan["center_mm"]
            toolhead.set_position(position)
            self._beta_check(session)
            kin.limits[index] = self.owner.rails_by_axis[session["axis"]].get_range()
            self.owner._set_accel(plan["effective_accel_mm_s2"])
            self._beta_check(session)
            self.owner._enable_motors(session["axis"])
            self._beta_check(session)
            if plan["current_helper"] is not None:
                current_attempted = True
                plan["current_helper"]([index], True)
                self._beta_check(session)
            toolhead.wait_moves()
            self._beta_check(session)
            session["beta_motor_settle_complete_time_s"] = toolhead.get_last_move_time()
            for iteration in range(1, NOISE_ITERATIONS + 1):
                session["beta_current_motion"] = {"iteration": iteration, "stage": "baseline",
                                                   "observed_planner_segments": []}
                token = None
                capture = None
                capture_failure = None
                try:
                    token = self.owner.start_beta_motion_capture(lambda: self._beta_check(session))
                    self._beta_check(session)
                    rate = token.get("sample_rate_hz")
                    baseline = token.get("baseline_samples", ())
                    if (type(rate) not in (int, float) or not math.isfinite(rate) or rate <= 0
                            or len(baseline) < 50):
                        raise self._error("Beta cannot establish moving-capture sample rate/capacity")
                    duration = plan["minimum_duration_s"] + 1.0 + 2.0 * self.owner.stream_timeout
                    if (not math.isfinite(duration * rate)
                            or math.ceil(duration * rate) >= self.owner.capture_limit):
                        raise self._error("Beta central stroke exceeds the existing capture capacity")
                    start = toolhead.get_last_move_time()
                    session["beta_current_motion"].update(stage="forward_motion", start_time_s=start,
                        capture_cutoff_time_s=token["capture_cutoff_time_s"], sample_rate_hz=rate)
                    target = [None, None, None]
                    target[index] = plan["target_mm"]
                    toolhead.manual_move(target, plan["effective_speed_mm_s"])
                    self._beta_check(session)
                    end = toolhead.get_last_move_time()
                    token["motion_end_time_s"] = end
                    session["beta_current_motion"].update(end_time_s=end, stage="waiting_for_motion")
                    toolhead.wait_moves()
                    self._beta_check(session)
                    segments = self._beta_profile(plan, start, end, token["capture_cutoff_time_s"])
                    session["beta_current_motion"].update(stage="capture_completion", segments=segments)
                    self._beta_check(session)
                    closing_token = token
                    token = None
                    capture = self.owner.stop_beta_motion_capture(closing_token, lambda: self._beta_check(session))
                    self._beta_check(session)
                    samples = capture.get("samples", ())
                    interval = 1.0 / rate
                    if (len(samples) < 2
                            or samples[0][0] > segments[0]["start_time_s"] + 2.0 * interval
                            or samples[-1][0] < end - interval):
                        raise self._error("Beta capture does not cover the complete forward motion interval")
                    record = {key: value for key, value in plan.items() if key not in ("extractor", "current_helper")}
                    record.update(run_id=session["run_id"], beta=True, iteration=iteration,
                                  segments=segments, phase_time_source="actual_commanded_trapq",
                                  motor_settle_complete_time_s=session["beta_motor_settle_complete_time_s"])
                    record.update(beta_motion_statistics(capture, record))
                    cruises = [item for item in segments if item["phase"] == "cruise"]
                    if not any(sum(item["start_time_s"] <= sample[0] < item["end_time_s"]
                                   for sample in capture["samples"]) >= 2 for item in cruises):
                        raise self._error("Beta capture does not cover actual full-speed cruise")
                    session["beta_motion_iterations"].append({"record": record, "capture": capture})
                    session["beta_current_motion"]["stage"] = "return_to_assumed_center"
                except Exception as exc:
                    capture_failure = exc
                    raise
                finally:
                    if token is not None and getattr(self.owner, "beta_motion_capture", None) is token:
                        try:
                            self.owner.stop_beta_motion_capture(token, lambda: self._beta_check(session))
                        except Exception as exc:
                            session["beta_capture_cleanup_error"] = str(exc)
                            if capture_failure is None:
                                raise
                self._beta_check(session)
                target = [None, None, None]
                target[index] = plan["center_mm"]
                toolhead.manual_move(target, plan["effective_speed_mm_s"])
                self._beta_check(session)
                toolhead.wait_moves()
                self._beta_check(session)
                if self.calibration_settle_time:
                    toolhead.dwell(self.calibration_settle_time)
                    self._beta_check(session)
                    toolhead.wait_moves()
                    self._beta_check(session)
                session["beta_current_motion"]["stage"] = "completed"
            completed = True
        except Exception as exc:
            failure = exc
            raise
        finally:
            cleanup_errors = []
            try:
                if current_attempted:
                    try:
                        plan["current_helper"]([index], False)
                        restore["central_driver_current_restored"] = True
                    except Exception as exc:
                        cleanup_errors.append("homing current: %s" % exc)
                try:
                    self.owner._set_accel(plan["original_accel_mm_s2"])
                    restore["central_profile_accel_restored"] = True
                    if completed and not cleanup_errors:
                        self._beta_check(session)
                        position = toolhead.get_position()
                        position[index] = original_position
                        toolhead.set_position(position)
                        self._beta_check(session)
                        restore["central_selected_frame_restored"] = True
                except Exception as exc:
                    cleanup_errors.append("acceleration/frame: %s" % exc)
            finally:
                # Failure never restores a guessed coordinate over an unknown
                # displacement, nor leaves the temporary selected axis homed.
                kin.limits[index] = original_limit
                restore["central_selected_limit_restored"] = kin.limits[index] == original_limit
            if cleanup_errors:
                restore["cleanup_errors"] = cleanup_errors
                if failure is None:
                    raise self._error("Beta cleanup failed: " + "; ".join(cleanup_errors))

    def _beta_seed(self, session):
        peaks = [item["record"]["seed_magnitude_peak_g"] for item in session["beta_motion_iterations"]]
        if len(peaks) != NOISE_ITERATIONS or any(
                type(value) not in (int, float) or not math.isfinite(value) or value < 0 for value in peaks):
            raise self._error("Beta requires three valid independent forward-motion noise recordings")
        peak = max(peaks)
        start = Decimal(str(session["start_threshold_g"]))
        step = Decimal(str(session["threshold_step_g"]))
        required = max(start, Decimal("1.2") * Decimal(str(peak)), Decimal(str(peak)) + Decimal("0.1"))
        candidate = float(start + math.ceil((required - start) / step) * step)
        explicit = session["explicit_start_threshold"]
        selected = session["start_threshold_g"] if explicit else candidate
        if not explicit and self._threshold_limit() is None:
            raise self._error("Beta automatic motion-noise seed requires a known sensor range; "
                              "specify START_THRESHOLD explicitly")
        self._validate_calibration_threshold(selected, "Beta initial candidate")
        if not explicit and selected <= peak:
            raise self._error("Beta motion-noise seed must exceed the measured forward-motion peak")
        session["beta_motion_summary"] = {
            "source": "independent_forward_motion_noise", "iterations": NOISE_ITERATIONS,
            "seed_phases": ["startup", "acceleration", "cruise"],
            "excluded_seed_phases": ["braking", "return"], "seed_peak_g": peak,
            "initial_threshold_source": "explicit_start_threshold" if explicit else "motion_noise",
            "required_threshold_g": float(required), "initial_threshold_g": selected,
            "step_g": session["threshold_step_g"], "margin_factor": 1.2, "margin_g": 0.1,
            "phase_records": [item["record"] for item in session["beta_motion_iterations"]]}
        session.update(candidate=selected, start_threshold_g=selected, beta_phase="homing")
        self._info("Beta forward-motion peak %.6g g; initial candidate %.6g g (%s). "
                   "Braking/return peaks are diagnostic only" % (
                       peak, selected, session["beta_motion_summary"]["initial_threshold_source"]))

    def _beta_return_rejected(self, axis, record):
        session = self.session
        terminal = session.get("terminal_home")
        if (terminal is None or terminal[0] != axis or terminal[1] is not record
                or self.owner.last.get(axis) is not terminal[2]
                or record.get("run_id") != session["run_id"]
                or record.get("workflow_attempt") != session["attempts"]):
            raise self._error("Beta rejection has no current recorded home")
        halt = record.get("halt_travel_mm_signed")
        if (type(halt) not in (int, float) or not math.isfinite(halt)
                or halt != terminal[2].get("halt_travel_mm_signed")):
            raise self._error("Beta rejection requires its current finite signed halt displacement")
        self._require_calibration_candidate_attempts(session)
        def current():
            self._beta_check(session)
            if (self.owner.last.get(axis) is not terminal[2]
                    or terminal[2].get("halt_travel_mm_signed") != halt):
                raise self._error("Beta rejected-return result changed")
        current()
        toolhead = self.owner.toolhead
        index = AXES[axis]
        target = toolhead.get_position()[index] - halt
        if not math.isfinite(target):
            raise self._error("Beta rejected-return target must be finite")
        coordinates = [None, None, None]
        coordinates[index] = target
        info = self.owner.rails_by_axis[axis].get_homing_info()
        try:
            self.owner._limit_accel()
            current()
            if halt:
                toolhead.manual_move(coordinates, min(self.reposition_speed, info.speed))
                current()
                toolhead.wait_moves()
                current()
            if self.calibration_settle_time:
                toolhead.dwell(self.calibration_settle_time)
                current()
                toolhead.wait_moves()
                current()
        finally:
            self.owner._restore_accel()
        current()
        record["beta_rejected_return_mm_signed"] = -halt
        session["recovery"].update(count=session["recovery"]["count"] + 1, last_return_mm_signed=-halt)

    def _beta_trials(self):
        session = self.session
        axis = session["axis"]
        while True:
            discovery = not session["candidate_qualified"]
            try:
                record, capture = self._home(axis, session["candidate"])
            except Exception as exc:
                source = self.owner.last.get(axis)
                partial = (dict(source) if isinstance(source, dict)
                           and source is not session.get("home_old_record") else {"axis": axis})
                partial.update(axis=axis, run_id=session["run_id"], workflow_attempt=session["attempts"],
                               workflow_threshold_g=session["candidate"],
                               workflow_phase="discovery" if discovery else "validation",
                               beta_decision="incomplete", beta_reason=str(exc), capture_valid=False)
                if session["beta_homes"] and session["beta_homes"][-1]["record"].get(
                        "workflow_attempt") == session["attempts"]:
                    session["beta_homes"][-1]["record"].update(partial)
                else:
                    session["beta_homes"].append({"record": partial, "capture": {"samples": []}})
                raise
            del capture
            quality, reason = self._quality(axis, record)
            record.update(beta_decision="pending", beta_quality=quality, beta_reason=reason)
            if quality not in ("good", "early"):
                record["beta_decision"] = "aborted"
                session["beta_homes"][-1]["record"].update(record)
                raise self._error(reason + "; Beta aborted without a threshold proposal")
            reject = quality == "early"
            if not discovery and not reject:
                session["validation_records"].append(record)
                session["successes"] = len(session["validation_records"])
                session["repeatability_spread_mm"] = self._repeatability_spread(session["validation_records"])
                reject = session["repeatability_spread_mm"] > CALIBRATION_REPEATABILITY_MM
                if reject:
                    reason = "Beta running validation spread %.6g mm exceeds 2 mm" % session["repeatability_spread_mm"]
            if reject:
                record.update(beta_decision="rejected", beta_reason=reason,
                              beta_running_spread_mm=session.get("repeatability_spread_mm"))
                session["beta_homes"][-1]["record"].update(record)
                session["early_return_cleanup"] = True
                try:
                    self._next_calibration_candidate(reason, apply=False)
                    self._beta_return_rejected(axis, record)
                    self._next_calibration_candidate(reason)
                finally:
                    session["beta_homes"][-1]["record"].update(record)
                    self._invalidate_calibration_axis(session)
                continue
            if discovery:
                record["beta_decision"] = "discovery_excluded"
                session["beta_homes"][-1]["record"].update(record)
                session.update(candidate_qualified=True, candidate_attempt=record["workflow_attempt"])
                self._recenter(axis)
                continue
            record.update(beta_decision="accepted_validation",
                          beta_running_spread_mm=session["repeatability_spread_mm"])
            session["beta_homes"][-1]["record"].update(record)
            if session["successes"] < REQUIRED_SUCCESSES:
                self._recenter(axis)
                continue
            qualified = self._qualified_validation_records()
            self._release_terminal_home(axis, qualified[-1])
            self._qualified_validation_records()
            self._finish({"result": "beta_proposed", "beta": True,
                          "proposed_threshold_g": session["candidate"], "axis": axis,
                          "successes": REQUIRED_SUCCESSES, "save_config": False,
                          "model_staged": False, "run_id": session["run_id"]})
            return

    def _beta_result(self, session, result):
        if session.get("beta_exported"):
            return result
        session["beta_exported"] = True
        result = dict(result, beta=True, save_config=False, model_staged=False)
        if session.get("finish_interrupted"):
            primary = result.get("reason")
            result.update(result="aborted", operation_reason=primary,
                          reason=(str(primary) + "; " if primary else "") + session["finish_interrupted"],
                          proposed_threshold_g=None)
        report_generation, operation = self.preflight_generation, self.owner.operation_generation
        def check_report():
            if (self.session is not session or self.preflight_generation != report_generation
                    or self.owner.operation_generation != operation):
                raise self._error("Beta report export was cancelled or interrupted")
        try:
            paths = write_beta_motion(
                self.charts_directory, session["run_id"], session["axis"],
                session["beta_motion_iterations"], homes=session["beta_homes"],
                result=result.get("result", "aborted"), reason=result.get("reason"),
                proposed_threshold_g=result.get("proposed_threshold_g"), renderer=self.renderer,
                check_running=check_report)
            check_report()
            result["summary_paths"] = paths
            self._show_pngs(paths, "Beta calibration experiment")
        except Exception as exc:
            logging.exception("accel_homing: Beta PNG export failed")
            result.update(chart_error=str(exc), result="aborted", proposed_threshold_g=None)
            result.setdefault("reason", "Beta PNG export failed: %s" % exc)
        try:
            diagnostics = self._beta_diagnostics(session, result)
            result["beta_diagnostics"] = diagnostics
            self._beta_print_diagnostics(diagnostics)
        except Exception as exc:
            # Reporting is optional cleanup. It must never hide a motion,
            # capture, cancellation or restore failure.
            result["diagnostic_error"] = str(exc)
            logging.exception("accel_homing: Beta console diagnostics failed")
        return result

    def _beta_diagnostics(self, session, result):
        errors = []
        def optional(label, getter):
            try:
                return getter()
            except Exception as exc:
                errors.append("%s: %s" % (label, exc))
                return None
        def source_hashes():
            import hashlib
            hashes = {}
            for name in ("accel_homing_workflows.py", "beacon_accel_homing.py", "accel_homing_charts.py",
                         "accel_homing_backends.py"):
                with Path(__file__).with_name(name).open("rb") as stream:
                    content = stream.read(2 * 1024 * 1024 + 1)
                if len(content) > 2 * 1024 * 1024:
                    raise ValueError("plugin source exceeds diagnostic hash bound")
                hashes[name] = hashlib.sha256(content).hexdigest()
            return hashes
        backend = getattr(self.owner, "backend", None)
        firmware = optional("firmware", lambda: {
            key: value for key, value in self.printer.get_start_args().items()
            if key in ("software_version", "git_version")})
        restore = dict(session.get("beta_restore", {}))
        restore.update(thresholds_restored=self.owner.threshold_g == session["saved_thresholds"],
                       back_off_restored=self.owner.back_off_dist == session["saved_back_off"],
                       model_unchanged=self.model is session.get("beta_model_object"),
                       cleanup_error=session.get("beta_cleanup_error"),
                       capture_cleanup_error=session.get("beta_capture_cleanup_error"))
        restore["capture_closed"] = optional("capture restoration", lambda: all((
            getattr(self.owner, "beta_motion_capture", None) is None,
            getattr(self.owner, "beta_capture", None) is None,
            getattr(self.owner, "noise_session", None) is None,
            getattr(self.owner, "capture_batches", None) is None)))
        plan = session.get("beta_motion_plan", {})
        observed = optional("final acceleration", lambda: self.owner.toolhead.get_max_velocity()[1])
        restore["final_observed_accel_mm_s2"] = observed
        initial = plan.get("original_accel_mm_s2")
        restore["final_accel_matches_initial"] = (math.isclose(observed, initial, rel_tol=1e-9)
            if type(observed) in (int, float) and type(initial) in (int, float) else None)
        expected = self._geometry(session["axis"])[1]
        summary = dict(session.get("beta_motion_summary") or {})
        summary.pop("phase_records", None)
        trial_keys = ("axis", "pass", "result", "workflow_attempt", "workflow_phase",
                      "workflow_threshold_g", "threshold_g", "arm_time_s", "onset_time_s",
                      "trigger_onset_time_s", "capture_clock_shift_s", "travel_mm", "travel_mm_exact",
                      "trigger_travel_mm_signed", "halt_travel_mm_signed", "beta_decision", "beta_quality",
                      "beta_reason", "beta_running_spread_mm", "beta_rejected_return_mm_signed",
                      "beta_telemetry", "capture_valid")
        trials = [{key: item["record"].get(key) for key in trial_keys if key in item["record"]}
                  for item in session["beta_homes"][:self.max_attempts]]
        motions = []
        for item in session["beta_motion_iterations"][:NOISE_ITERATIONS]:
            record = dict(item["record"])
            record["capture_quality"] = item["capture"].get("beta_capture_diagnostics")
            record["sensor_context"] = item["capture"].get("sensor_context")
            motions.append(record)
        diagnostics = {
            "schema": "accel_homing_beta/v1", "run_id": session["run_id"],
            "initial_arguments": session["beta_arguments"],
            "environment": {"kinematics": self.kinematics_name, "firmware": firmware,
                            "on_disk_plugin_sha256": optional("source identity", source_hashes),
                            "source_identity_scope": "on-disk bytes; loaded runtime bytes may differ until restart"},
            "sensor": optional("sensor context", self.owner.noise_sensor_context),
            "timing_policy": {"stream_timeout_s": getattr(self.owner, "stream_timeout", None),
                              "reference_max_age_s": optional("reference age", lambda: self.owner._reference_max_age()),
                              "homing_batch_max_age_s": getattr(backend, "max_age", getattr(self.owner, "stream_timeout", None)),
                              "max_future_s": 0.005, "report_interval_s": getattr(backend, "interval", None),
                              "capture_sample_limit": getattr(self.owner, "capture_limit", None)},
            "motion_plan": plan, "driver_current_context": session.get("beta_driver_current_context"),
            "seed": summary, "motions": motions, "trials": trials,
            "current_motion": session.get("beta_current_motion"),
            "capture_health": optional("capture diagnostics", lambda: self.owner.beta_capture_diagnostics()),
            "qualification": {"expected_center_travel_mm": expected,
                              "plausible_min_mm": expected * (1.0 - self.travel_tolerance),
                              "plausible_max_mm": expected * (1.0 + self.travel_tolerance),
                              "discovery_excluded": True, "checks_required": REQUIRED_SUCCESSES,
                              "spread_limit_mm": CALIBRATION_REPEATABILITY_MM,
                              "running_spread_mm": session.get("repeatability_spread_mm")},
            "restoration": restore,
            "outcome": {key: result.get(key) for key in ("result", "reason", "operation_reason", "proposed_threshold_g", "attempts",
                                                        "attempt_limit", "attempts_remaining", "restarts",
                                                        "successes", "summary_paths", "chart_error")},
            "diagnostic_collection_errors": errors, "physical_position_verified": False,
            "threshold_applied": False, "model_staged": False}
        def clean(value, depth=0):
            if depth > 14:
                return "diagnostic nesting bound reached"
            if value is None or type(value) in (bool, int):
                return value
            if isinstance(value, float):
                return value if math.isfinite(value) else None
            if isinstance(value, str):
                return value[:2048]
            if isinstance(value, dict):
                return {str(key): clean(item, depth + 1) for key, item in list(value.items())[:128]
                        if key not in ("samples", "baseline_samples", "trace", "extractor", "current_helper")}
            if isinstance(value, (tuple, list)):
                return [clean(item, depth + 1) for item in value[:128]]
            return "unavailable diagnostic type"
        return clean(diagnostics)

    def _beta_print_diagnostics(self, diagnostics):
        outcome = diagnostics["outcome"]
        self._info("Beta complete: %s; proposed threshold %s g; no threshold/model saved. PNG %s" % (
            outcome.get("result"), outcome.get("proposed_threshold_g"), outcome.get("summary_paths")))
        def emit(section, data):
            self._info("Beta diagnostic " + json.dumps({
                "schema": diagnostics["schema"], "run_id": diagnostics["run_id"],
                "section": section, "data": data}, separators=(",", ":"), allow_nan=False))
        for key, value in diagnostics.items():
            if key in ("schema", "run_id", "motions", "trials"):
                continue
            emit(key, value)
        for index, motion in enumerate(diagnostics["motions"], 1):
            phases = motion.get("phases", [])
            emit("motion.%d" % index, {key: value for key, value in motion.items() if key != "phases"})
            for number, phase in enumerate(phases, 1):
                emit("motion.%d.phase.%d" % (index, number), phase)
        for index, trial in enumerate(diagnostics["trials"], 1):
            metrics = trial.get("beta_telemetry", {})
            emit("attempt.%d" % index, {key: value for key, value in trial.items() if key != "beta_telemetry"})
            if isinstance(metrics, dict):
                for key, value in metrics.items():
                    emit("attempt.%d.%s" % (index, key), value)

    def cmd_BETA_CALIBRATE(self, gcmd):
        axis = gcmd.get("AXIS", "").lower()
        if axis not in AXES:
            raise gcmd.error("AXIS must be X or Y")
        self._require_centered(gcmd)
        if gcmd.get_int("SAVE_CONFIG", 0, minval=0, maxval=1):
            raise gcmd.error("Beta produces a PNG and threshold suggestion only; SAVE_CONFIG=1 is unsupported")
        if self.session is not None:
            raise gcmd.error("A workflow is already active; cancel it before starting Beta")
        grid = self._calibration_grid(gcmd)
        self._preflight([axis])
        self._require_unhomed_xy()
        # No beta path calls _save_model or configfile.set.
        self._begin(gcmd, "calibration", axis, calibration_grid=grid, save_config_default=0)
        session = self.session
        session.update(beta=True, save_config=False, state="running", beta_phase="preflight",
                       beta_generation=self.owner.operation_generation,
                       beta_motion_iterations=[], beta_homes=[], beta_model_object=self.model,
                       beta_arguments={key: gcmd.get(key, None) for key in (
                           "AXIS", "CENTERED", "START_THRESHOLD", "STEP", "SAVE_CONFIG")})
        try:
            self._chart_preflight()
            self._beta_check(session)
            self._preflight([axis])
            self._require_unhomed_xy()
            plan = self._beta_motion_plan(session)
            session["beta_motion_plan"] = {key: value for key, value in plan.items()
                                           if key not in ("extractor", "current_helper")}
            self._info("Beta %s: configured homing speed %.6g mm/s, effective %.6g mm/s, "
                       "acceleration %.6g mm/s^2; three central forward recordings, no model save" % (
                           axis.upper(), plan["configured_speed_mm_s"], plan["effective_speed_mm_s"],
                           plan["effective_accel_mm_s2"]))
            self._info("Beta driver/current context: %s; physical current is not measured" %
                       plan["driver_current_context"]["matching"])
            self._beta_motion_noise(session, plan)
            self._beta_check(session)
            self._require_unhomed_xy()
            self._beta_seed(session)
            self._beta_trials()
        except Exception as exc:
            if self.session is session:
                self._finish({"result": "aborted", "reason": str(exc)},
                             release=bool(session.get("axis_touched")
                                          and not session.get("early_return_cleanup")
                                          and not session.get("axis_only_cleanup")))
            raise
        finally:
            pending = session.get("beta_pending_result", {"result": "aborted", "reason": "Beta did not finish"})
            if self.session is session:
                try:
                    self.last_result = self._beta_result(session, pending)
                finally:
                    if self.session is session:
                        self.session = None

    def _continue_calibration(self):
        session = self.session
        try:
            if self.reactor.monotonic() >= session["expires"]:
                raise self._error("manual-centering wait expired")
            self._preflight([session["axis"]], continuation=True)
            if self._snapshot() != session["snapshot"]:
                raise self._error("printer state changed while awaiting manual recentering; restart workflow")
            self._require_unhomed_xy()
            session["state"] = "running"
            self.reactor.update_timer(self.timer, self.reactor.NEVER)
            self._calibrate_trials()
        except Exception as exc:
            if self.session is not None:
                self._finish({"result": "aborted", "reason": str(exc)},
                             release=(not self.session.get("early_return_cleanup", False)
                                      and not self.session.get("axis_only_cleanup", False)))
            raise

    def cmd_CANCEL(self, gcmd):
        self.preflight_generation += 1
        self._clear_noise_seeds()
        if self.session is None:
            gcmd.respond_info("No accelerometer homing workflow is active")
            return
        self._finish({"result": "cancelled", "reason": "operator cancelled"})
        gcmd.respond_info("Accelerometer homing workflow cancelled; settings restored")

    def _start_xy(self, gcmd, mode, axes=("x", "y")):
        self._require_centered(gcmd)
        if mode == "charts":
            self._chart_preflight()
        self._begin(gcmd, mode, axes=axes)
        try:
            self.session.update(state="running", axes=axes)
            self._test_xy()
        except Exception as exc:
            if self.session is not None:
                self._finish({"result": "aborted", "reason": str(exc)}, release=True)
            raise

    def cmd_TEST(self, gcmd):
        self._start_xy(gcmd, "test")

    def cmd_CHARTS(self, gcmd):
        requested = gcmd.get("AXIS", None)
        axes = ("x", "y")
        if requested is not None:
            axis = str(requested).strip().lower()
            if axis not in AXES:
                raise gcmd.error("AXIS must be X or Y; omit AXIS to chart both axes")
            axes = (axis,)
        self._start_xy(gcmd, "charts", axes=axes)

    def _accuracy_capture_directory(self):
        roots = [Path(self.charts_directory).resolve()]
        if self.chart_config_root is not None:
            roots.append(Path(self.chart_config_root).resolve())
        main_config = self.chart_start_args.get("config_file")
        if main_config:
            roots.append(Path(main_config).expanduser().resolve().parent)
        # TMPDIR may point into Moonraker's watched config tree. Check each
        # parent before creating a directory there; /tmp and /var/tmp are
        # explicit Linux fallbacks when the environment selects that tree.
        for candidate in dict.fromkeys((tempfile.gettempdir(), "/tmp", "/var/tmp")):
            parent = Path(candidate).resolve()
            if any(parent == root or root in parent.parents for root in roots):
                continue
            try:
                return tempfile.TemporaryDirectory(prefix="accel_homing_accuracy_", dir=str(parent))
            except OSError:
                continue
        raise self._error("cannot prepare temporary accuracy captures outside the chart/config directories")

    @staticmethod
    def _public_accuracy_results(session):
        return {axis: {key: value for key, value in item.items()
                       if key not in ("capture_paths", "discovery_capture_paths", "rejection_capture_paths")}
                for axis, item in session["axis_results"].items()}

    def _cleanup_accuracy_captures(self, temporary, session, error_active):
        if session is not None:
            session.pop("capture_directory", None)
            for item in session.get("axis_results", {}).values():
                for key in ("capture_paths", "discovery_capture_paths", "rejection_capture_paths"):
                    item.pop(key, None)
        try:
            temporary.cleanup()
        except Exception as exc:
            logging.exception("accel_homing: unable to remove temporary accuracy captures")
            if (session is not None and self.last_result is not None
                    and self.last_result.get("run_id") == session["run_id"]):
                self.last_result["capture_cleanup_error"] = str(exc)
                if self.last_result["result"] == "complete":
                    self.last_result.update(result="aborted", reason="temporary accuracy capture cleanup failed: %s" % exc)
            # An original motion/restoration/cancellation error keeps priority.
            if not error_active:
                raise self._error("temporary accuracy capture cleanup failed: %s" % exc) from exc

    def _accuracy_position_reference(self):
        toolhead = self.owner.toolhead
        return (tuple(toolhead.get_position()),
                tuple((stepper, stepper.get_mcu_position())
                      for stepper in toolhead.get_kinematics().get_steppers()),
                tuple(line.is_motor_enabled() for line in self._xy_enable_lines()))

    def _accuracy_recovery_context(self):
        # Backoff is deliberately restored by _finish; no movement is permitted
        # between this snapshot and replacement of the owned accuracy session.
        snapshot = self._noise_snapshot()
        return (self._accuracy_position_reference(), snapshot[1:5], snapshot[6:])

    def _prepare_accuracy_recovery(self, session, axis, record, reason):
        """Return to the confirmed start using current step/halt evidence only."""
        self._check_running(session)
        finite = lambda value: type(value) in (int, float) and math.isfinite(value)
        threshold = session["saved_thresholds"][axis]
        candidate = float(Decimal(str(threshold)) + Decimal("0.25"))
        self._validate_calibration_threshold(candidate, "accuracy recovery START_THRESHOLD")
        if candidate <= threshold or self.max_attempts < REQUIRED_SUCCESSES + 1:
            raise self._error("accuracy recovery cannot complete a fresh calibration candidate")
        center, expected = self._geometry(axis)
        if expected <= CALIBRATION_MIN_TRAVEL_MM:
            raise self._error("accuracy recovery requires center-to-endstop travel greater than 20 mm")
        if not callable(getattr(self.printer.lookup_object("configfile", None), "set", None)):
            raise self._error("accuracy recovery cannot stage a saved calibration")
        source = self.owner.last.get(axis)
        terminal, halt = session.get("terminal_home"), session.get("accuracy_halt")
        if (not isinstance(halt, dict) or terminal is None or terminal[0] != axis
                or terminal[1] is not record or terminal[2] is not source
                or halt.get("source") is not source or source is session.get("home_old_record")
                or halt.get("attempt") != session["attempts"]
                or not session["accuracy_started_centered"]
                or record.get("capture_valid") is not True or record.get("pass") != 1
                or record.get("run_id") != session["run_id"]
                or record.get("workflow_attempt") != session["attempts"]):
            raise self._error("early accuracy recovery lacks current halt/capture or confirmed center evidence")
        travel = source.get("travel_mm_exact")
        trigger, displacement = halt["trigger"], halt["halt"]
        info = self.owner.rails_by_axis[axis].get_homing_info()
        sign = 1 if info.positive_dir else -1
        if (not all(finite(value) for value in (travel, trigger, displacement)) or travel < 0
                or travel >= expected * (1.0 - self.travel_tolerance)
                or trigger * sign < 0 or displacement * sign < trigger * sign
                or displacement * sign >= expected * (1.0 - self.travel_tolerance)
                or not math.isclose(abs(trigger), travel, rel_tol=1e-9, abs_tol=1e-6)
                or record.get("halt_travel_mm_signed") != displacement
                or source.get("halt_travel_mm_signed") != displacement
                or source.get("trigger_travel_mm_signed") != trigger):
            raise self._error("early accuracy recovery requires consistent short signed trigger/halt travel")
        start, start_steps, _enabled = session["accuracy_home_start"]
        positions = halt["positions"]
        active = {stepper for stepper, _position in start_steps if stepper.is_active_axis(axis)}
        if (len(positions) != len(active) or {item[0] for item in positions} != active
                or any(not all(finite(value) for value in item[1:]) for item in positions)):
            raise self._error("early accuracy recovery lacks complete current stepper halt positions")
        step_positions = {stepper: (first, triggered, stopped)
                          for stepper, first, triggered, stopped in positions}
        moved = {stepper.get_name(): 0.0 for stepper, _position in start_steps}
        triggered = dict(moved)
        for stepper, first in start_steps:
            if stepper in active:
                recorded_start, trig, stopped = step_positions[stepper]
                if first != recorded_start or stepper.get_mcu_position() != stopped:
                    raise self._error("early accuracy recovery stepper start/halt evidence changed")
                distance = stepper.get_step_dist()
                moved[stepper.get_name()] = (stopped - first) * distance
                triggered[stepper.get_name()] = (trig - first) * distance
            elif stepper.get_mcu_position() != first:
                raise self._error("unrelated axis moved during the rejected accuracy home")
        index = AXES[axis]
        kin = self.owner.toolhead.get_kinematics()
        for measured, reported in ((kin.calc_position(moved)[index], displacement),
                                   (kin.calc_position(triggered)[index], trigger)):
            if not finite(measured) or not math.isclose(measured, reported, rel_tol=1e-9, abs_tol=1e-6):
                raise self._error("early accuracy recovery travel disagrees with stepper halt evidence")
        toolhead = self.owner.toolhead
        before = tuple(toolhead.get_position())
        expected_position = info.position_endstop + displacement - trigger
        if (not finite(before[index])
                or not math.isclose(before[index], expected_position, rel_tol=1e-9, abs_tol=1e-6)
                or any(value != start[number] for number, value in enumerate(before) if number != index)
                or self.owner._homed(axis) is not True or self.owner.saved_accel is not None):
            raise self._error("rejected accuracy home lost its commanded start/stop coordinate reference")
        halted_reference = self._accuracy_position_reference()
        returning = False

        def current():
            self._check_running(session)
            if (self.owner.operation_generation != session["accuracy_generation"]
                    or halt["generation"] != session["accuracy_generation"]
                    or self.preflight_generation != session["accuracy_preflight_generation"]
                    or self.owner.last.get(axis) is not source
                    or source.get("halt_travel_mm_signed") != displacement
                    or source.get("trigger_travel_mm_signed") != trigger
                    or source.get("travel_mm_exact") != travel
                    or self.owner.homing_active or self.owner.attempt is not None
                    or getattr(self.owner, "sensor_fault", None) is not None
                    or (not returning and self._accuracy_position_reference() != halted_reference)
                    or any(value != before[number] for number, value in enumerate(toolhead.get_position())
                           if number != index)):
                raise self._error("early accuracy recovery was cancelled or its halt/center reference changed")

        current()
        target = before[index] - displacement
        if not finite(target):
            raise self._error("early accuracy recovery requires a finite return target")
        coordinates = [None, None, None]
        coordinates[index] = target
        try:
            self.owner._limit_accel()
            current()
            if displacement:
                returning = True
                toolhead.manual_move(coordinates, min(self.reposition_speed, info.speed))
                current()
                toolhead.wait_moves()
                current()
            if self.calibration_settle_time:
                toolhead.dwell(self.calibration_settle_time)
                current()
                toolhead.wait_moves()
                current()
        finally:
            self.owner._restore_accel()
        current()
        if toolhead.get_position()[index] != target or any(
                abs(stepper.get_mcu_position() - position) > 1 for stepper, position in start_steps):
            raise self._error("early accuracy recovery did not complete its commanded return")
        # Discard G28's rejected coordinate assignment without releasing motors
        # or inventing a midpoint coordinate. Discovery will establish it anew.
        kin.limits[0] = kin.limits[1] = (1.0, -1.0)
        session["accuracy_recovery_pending"] = {
            "axis": axis, "source_run_id": session["run_id"], "reason": reason,
            "previous_threshold_g": threshold, "start_threshold_g": candidate,
            "initial_increment_g": 0.25, "threshold_step_g": self.threshold_step,
            "return_mm_signed": -displacement,
            "physical_position_verified": False,
            "rerun_command": "ACCEL_HOMING_ACCURACY AXIS=%s SAMPLES=%d CENTERED=1" % (
                "".join(session["axes"]).upper(), session["samples_requested"])}
        session["accuracy_recovery_context"] = self._accuracy_recovery_context()
        self._info(reason + "; returned by reversing recorded commanded halt travel; "
                   "finalizing accuracy before one calibration starting at %.6g g" % candidate)

    def _check_accuracy_recovery(self, session):
        self._check_running(session)
        pending = session.get("accuracy_recovery_pending")
        result = self.last_result
        if (not pending or session.get("finish_interrupted")
                or self.preflight_generation != session["accuracy_preflight_generation"]
                or self.owner.operation_generation != session["accuracy_generation"]
                or getattr(self.owner, "sensor_fault", None) is not None
                or self._accuracy_recovery_context() != session["accuracy_recovery_context"]
                or self.owner.last.get(pending["axis"]) is not session["accuracy_halt"]["source"]
                or result is None or result.get("run_id") != session["run_id"]
                or result.get("result") != "aborted" or result.get("chart_error")
                or result.get("capture_cleanup_error")):
            raise self._error("accuracy recovery interrupted during finalization; manually recenter before retrying")

    def _start_accuracy_recalibration(self, session):
        self._check_accuracy_recovery(session)
        pending = session["accuracy_recovery_pending"]
        source_accuracy = self.last_result
        params = {"AXIS": pending["axis"].upper(), "START_THRESHOLD": str(pending["start_threshold_g"]),
                  "SAVE_CONFIG": "1"}
        gcmd = self.gcode.create_gcode_command("ACCEL_HOMING_CALIBRATE", "ACCEL_HOMING_CALIBRATE", params)
        grid = self._calibration_grid(gcmd)
        self._begin(gcmd, "calibration", pending["axis"], calibration_grid=grid, previous_accuracy=session)
        self.session.update(accuracy_recovery=dict(pending), source_accuracy=source_accuracy)
        self._run_calibration()

    def cmd_ACCURACY(self, gcmd):
        requested = str(gcmd.get("AXIS", "XY")).strip().lower()
        if requested not in ("x", "y", "xy"):
            raise gcmd.error("AXIS must be X, Y or XY")
        samples = gcmd.get_int("SAMPLES", 10, minval=2, maxval=50)
        # Saving belongs only to a fully successful recovery calibration.
        if gcmd.get("SAVE_CONFIG", None) is not None:
            raise gcmd.error("SAVE_CONFIG is not supported by ACCEL_HOMING_ACCURACY")
        self._require_centered(gcmd)
        self._chart_preflight()
        axes = list(requested)
        temporary = self._accuracy_capture_directory()
        session = None
        recovery = False
        try:
            self._begin(gcmd, "accuracy", axes=axes)
            session = self.session
            session.update(state="running", axes=axes, samples_requested=samples,
                           capture_directory=str(Path(temporary.name).resolve()),
                           attempt_limit=len(axes) * (samples + 1), sample_index=0,
                           configured_speeds={axis: self.owner.rails_by_axis[axis].get_homing_info().speed
                                              for axis in axes},
                            pending_record=None, last_arm_time=None,
                            accuracy_generation=self.owner.operation_generation,
                            accuracy_preflight_generation=self.preflight_generation,
                            accuracy_center_reference=None,
                           axis_results={axis: {"requested_samples": samples, "complete": False,
                                                "discovery": None, "discovery_capture_paths": None,
                                                "trials": [], "statistics": None,
                                                "capture_paths": [], "rejected": None}
                                         for axis in axes})
            self._info("Accuracy uses the configured homing speed and threshold. CENTERED=1 "
                       "confirms manually centered XY with nozzle clearance; each axis's first "
                       "home is discovery only and is excluded from measured statistics")
            try:
                self._check_running(session)
                session["accuracy_center_reference"] = self._accuracy_position_reference()
                self._check_running(session)
                self._accuracy_trials(session)
                self._check_running(session)
                if session.get("accuracy_recovery_pending"):
                    self._finish({"result": "aborted", "reason": session["accuracy_recovery_pending"]["reason"],
                                  "recovery": dict(session["accuracy_recovery_pending"])})
                    recovery = True
                    return
                # Validate temporary capture bytes before terminal release.
                # Final publication follows restoration, so release failures
                # receive an aborted summary retaining accepted measurements.
                try:
                    validate_accuracy(self.charts_directory, session["run_id"], session["axis_results"],
                                      capture_directory=session["capture_directory"],
                                      **self._chart_options(session))
                except (ValueError, OSError) as exc:
                    raise self._error("accuracy summary validation failed: %s" % exc) from exc
                axis, record, _source = session["terminal_home"]
                self._release_terminal_home(axis, record)
                self._finish({"result": "complete"})
                if self.last_result["result"] != "complete":
                    raise self._error(self.last_result.get("reason", "accuracy export failed"))
            except Exception as exc:
                if self.session is session:
                    self._finish({"result": "aborted", "reason": str(exc)}, release=True)
                raise
        finally:
            error_active = sys.exc_info()[0] is not None
            try:
                self._cleanup_accuracy_captures(temporary, session, error_active)
                if recovery and not error_active:
                    self._start_accuracy_recalibration(session)
            finally:
                if self.session is session and session is not None and session.get("accuracy_recovery_pending"):
                    self.session = None
                    self.reactor.update_timer(self.timer, self.reactor.NEVER)

    @staticmethod
    def _accuracy_statistics(records):
        return accuracy_statistics(records)

    def _accuracy_trials(self, session):
        for axis_index, axis in enumerate(session["axes"]):
            session["axis"] = axis
            axis_result = session["axis_results"][axis]
            for sample_index in range(session["samples_requested"] + 1):
                self._check_running(session)
                session["sample_index"] = sample_index
                session["pending_record"] = None
                record, capture = self._home(axis)
                quality, reason = self._quality(axis, record)
                if quality != "good":
                    axis_result["rejected"] = record
                elif sample_index == 0:
                    axis_result["discovery"] = record
                else:
                    axis_result["trials"].append(record)
                    axis_result["capture_paths"].append(None)
                    axis_result["statistics"] = self._accuracy_statistics(axis_result["trials"])
                    axis_result["complete"] = len(axis_result["trials"]) == session["samples_requested"]
                try:
                    paths = write_capture(session["capture_directory"],
                                          "%s_a%03d" % (session["run_id"], session["attempts"]),
                                          axis, capture, record, **self._chart_options(session))
                except (ValueError, OSError) as exc:
                    raise self._error("accuracy capture export failed: %s" % exc) from exc
                finally:
                    # Bound memory to one capture, even on export rejection.
                    del capture
                self._check_running(session)
                if quality != "good":
                    axis_result["rejection_capture_paths"] = paths
                    if quality == "early":
                        try:
                            self._prepare_accuracy_recovery(session, axis, record, reason)
                        except self.printer.command_error as exc:
                            raise self._error("%s; no automatic move follows this error; manually recenter" % exc) from exc
                        return
                    raise self._error(reason + "; no automatic move follows this result; manually recenter")
                if sample_index == 0:
                    axis_result["discovery_capture_paths"] = paths
                    self._info("%s discovery home qualified; excluded from accuracy statistics" % axis.upper())
                else:
                    axis_result["capture_paths"][-1] = paths
                    self._info("%s accuracy measurement %d/%d retained" % (
                        axis.upper(), sample_index, session["samples_requested"]))
                # Only a qualified current home authorizes a return to center.
                # X must also return after its final trial before Y discovery.
                if sample_index < session["samples_requested"] or axis_index < len(session["axes"]) - 1:
                    self._recenter(axis)

    def _accuracy_result(self, session, ending):
        pending = session.get("pending_record")
        if pending is not None:
            axis_result = session["axis_results"][session["axis"]]
            if pending is not axis_result["discovery"] and not any(
                    pending is record for record in axis_result["trials"]):
                axis_result["rejected"] = pending
        for axis, axis_result in session["axis_results"].items():
            stats = axis_result["statistics"] = self._accuracy_statistics(axis_result["trials"])
            count = len(axis_result["trials"])
            axis_result["complete"] = count == session["samples_requested"]
            detail = ("count %d/%d; no qualified measured homes" % (count, session["samples_requested"])
                      if stats is None else
                      "count %d/%d; min %.3f mm; max %.3f mm; mean %.6f mm; spread %.3f mm; "
                      "stddev %s mm; variance %s mm^2 (%s)" % (
                          count, session["samples_requested"], stats["min_mm"], stats["max_mm"],
                          stats["mean_mm"], stats["spread_mm"],
                          "unavailable" if stats["stddev_mm"] is None else "%.6f" % stats["stddev_mm"],
                          "unavailable" if stats["variance_mm2"] is None else "%.9f" % stats["variance_mm2"],
                          stats["variance_definition"]))
            self._info("%s accuracy %s: %s" % (axis.upper(),
                                              "complete" if axis_result["complete"] else "incomplete", detail))
            if stats is not None and stats["spread_mm"] >= REPEATABILITY_LIMIT_MM:
                self._info("%s observed spread is at or above the existing %.2f mm calibration "
                           "criterion; all qualified accuracy trials remain in this report" % (
                               axis.upper(), REPEATABILITY_LIMIT_MM))
        result = {"mode": "accuracy", "run_id": session["run_id"],
                  "axes": list(session["axes"]), "samples_requested": session["samples_requested"],
                  "fixed_thresholds_g": {axis: session["saved_thresholds"][axis]
                                         for axis in session["axes"]},
                  "configured_homing_speeds_mm_s": dict(session["configured_speeds"]),
                  "attempts": session["attempts"], "axis_results": self._public_accuracy_results(session),
                  "records": list(session["records"]), "paths": list(session["paths"]), **ending}
        paths = {}
        try:
            write_accuracy(self.charts_directory, session["run_id"],
                           session["axis_results"], result["result"], result.get("reason"),
                           capture_directory=session["capture_directory"], published_paths=paths,
                           **self._chart_options(session))
            self._show_pngs(paths, "Accuracy")
        except Exception as exc:
            logging.exception("accel_homing: unable to export accuracy summary")
            result["chart_error"] = str(exc)
            if result["result"] == "complete":
                result["result"] = "aborted"
                result["reason"] = "accuracy summary export failed: %s" % exc
        if paths:
            result["summary_paths"] = dict(paths)
        self._info("Accuracy run %s: %s. Commanded travel is reported to 0.001 mm; "
                   "these measurements do not certify physical stop position" % (
                       session["run_id"], result["result"]))
        return result

    def _test_xy(self):
        session = self.session
        axes = session["axes"]
        for index, axis in enumerate(axes):
            record, capture = self._home(axis)
            quality, reason = self._quality(axis, record)
            if session["mode"] == "charts":
                paths = write_capture(self.charts_directory, session["run_id"], axis, capture, record,
                                      **self._chart_options(session))
                session["paths"].append({"axis": axis, **paths})
                self._show_pngs(paths, axis.upper() + " measured acceleration")
            if quality != "good":
                paths = list(session["paths"])
                self._finish({"result": "rejected", "reason": reason, "paths": paths}, release=True)
                raise self._error(reason + "; no automatic move follows this result; manually recenter")
            if index + 1 < len(axes):
                # Return only the trusted axis to center when another axis
                # follows; the next axis retains its manually confirmed center.
                self._recenter(axis)
        self._release_terminal_home(axis, record)
        self._finish({"result": "complete", "mode": session["mode"],
                      "run_id": session["run_id"], "records": list(session["records"]),
                      "paths": list(session["paths"])})
        self._info("%s %s complete; these measurements do not certify physical stop position"
                   % (" and ".join(axis.upper() for axis in axes), session["mode"]))

    def get_status(self, eventtime):
        session = self.session
        active = None if session is None else {
            key: session.get(key) for key in ("mode", "state", "axis", "candidate", "successes",
                                             "candidate_qualified", "attempts", "run_id", "expires",
                                             "repeatability_spread_mm")}
        if session is not None and session["mode"] == "accuracy":
            active = {key: session.get(key) for key in ("mode", "state", "axis", "attempts", "run_id")}
            active.update(axes=list(session["axes"]),
                          samples_requested=session["samples_requested"],
                          sample_index=session["sample_index"],
                          fixed_thresholds_g={axis: session["saved_thresholds"][axis]
                                              for axis in session["axes"]},
                          configured_homing_speeds_mm_s=dict(session["configured_speeds"]),
                          completed_samples={axis: len(item["trials"])
                                             for axis, item in session["axis_results"].items()},
                          axis_results=self._public_accuracy_results(session))
        elif session is not None and session["mode"] == "noise":
            active = {"mode": "noise", "state": session["state"], "run_id": session["run_id"],
                      "iteration": session["iteration"], "iterations_requested": NOISE_ITERATIONS,
                      "iterations_completed": len(session["iterations"]),
                      "duration_per_iteration_s": NOISE_DURATION}
            if session.get("calibration_preflight"):
                active.update(purpose="calibration_preflight", calibration_axis=session["axis"])
        elif active is not None:
            active["repeatability_limit_mm"] = (CALIBRATION_REPEATABILITY_MM
                                                  if session["mode"] == "calibration" else REPEATABILITY_LIMIT_MM)
            if session["mode"] == "calibration":
                active.update({key: session[key] for key in (
                    "start_threshold_g", "threshold_step_g", "explicit_calibration_grid", "quiet_range_enforced",
                    "calibration_settle_time_s")})
                active.update(self._calibration_attempt_status(session))
                active["recovery"] = self._calibration_recovery_status(session)
                active.update(algorithm=session.get("algorithm"), restarts=session.get("restarts", 0),
                              minimum_impact_travel_mm=CALIBRATION_MIN_TRAVEL_MM,
                              discovery_impact_required=True, recenter_policy="selected_axis_midpoint",
                              phase="validation" if session["candidate_qualified"] else "discovery")
                if session.get("stationary_noise") is not None:
                    active["stationary_noise"] = dict(session["stationary_noise"])
        return {"model": self.model_name, "manual_threshold_overrides": sorted(self.manual_axes),
                "active": active, "last_result": self.last_result,
                "noise_recommendation": self.noise_recommendation,
                "noise_seed_axes": sorted(self.noise_seeds),
                "charts_directory": self.charts_directory}

    def cmd_STATUS(self, gcmd):
        self.owner.cmd_STATUS(gcmd)
        gcmd.respond_info(json.dumps(self.get_status(self.reactor.monotonic()), sort_keys=True))
