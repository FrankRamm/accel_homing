#!/usr/bin/env python3
"""Transactional Klipper/Kalico installer with guarded host-service activation."""
import argparse
import base64
import datetime
import glob
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import socket
import stat
import subprocess
import sys
import tempfile
import time
import uuid

MODULES = ("accel_homing.py", "beacon_accel_homing.py", "accel_homing_backends.py",
           "accel_homing_workflows.py", "accel_homing_model.py", "accel_homing_charts.py")
SENSORS = {"beacon", "adxl345", "lis2dw", "lis3dh", "mpu9250", "icm20948"}
SAVE_MARKER = "#*# <---------------------- SAVE_CONFIG ---------------------->"
MANAGED = "accel_homing.cfg"
SECTION = re.compile(r"^\s*\[([^]]+)\]\s*(?:[#;].*)?$")
OPTION = re.compile(r"^(\s*)([\w]+)(\s*[:=]\s*)(.*?)(\r?\n)?$")
LEGACY_REFERENCE = re.compile(r"\b(?:beacon_accel_homing|beacon_accel_homing_status|set_beacon_accel_homing)\b|\bbeacon_accel\s*:", re.I)
SERVICE_NAME = re.compile(r"[A-Za-z0-9_:][A-Za-z0-9_.:@\\-]*\.service\Z")
SERVICE_PROPERTIES = ("Id", "LoadState", "ActiveState", "SubState", "MainPID", "InvocationID",
                      "ControlGroup", "ExecStart", "WorkingDirectory", "FragmentPath", "DropInPaths",
                      "NeedDaemonReload", "User", "Group", "CanStart", "CanStop",
                      "RefuseManualStart", "RefuseManualStop", "Environment", "EnvironmentFiles")


def load_motion_policy():
    # The backend module has only definitions/math imports at module scope.
    # Load the sibling file, never a possibly different installed Klipper copy.
    path = Path(__file__).resolve().with_name("accel_homing_backends.py")
    spec = importlib.util.spec_from_file_location("accel_homing_install_policy", path)
    policy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(policy)
    return policy


def load_uninstaller():
    path = Path(__file__).resolve().with_name("uninstaller.py")
    spec = importlib.util.spec_from_file_location("accel_homing_uninstaller", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_updater():
    path = Path(__file__).resolve().with_name("accel_homing_updater.py")
    spec = importlib.util.spec_from_file_location("accel_homing_installer_updater", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def uninstall_api():
    # Pass the current module's helpers explicitly, including in offline tests
    # which load installer.py under an isolated import name.
    import types
    return types.SimpleNamespace(**globals())


class InstallError(Exception):
    pass


class ServiceStarting(InstallError):
    pass


def canonical(value, base=None):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else Path(base or Path.cwd()) / path).resolve()


def command_instance(argv, cwd, origin, pid=None, service=None):
    index = next((i for i, arg in enumerate(argv) if Path(arg).name == "klippy.py"), None)
    if index is None:
        return None
    script = canonical(argv[index], cwd)
    if script.parent.name != "klippy":
        return None
    positional, api_socket = [], None
    values = {"-l", "-a", "-I", "-i", "-d", "-o", "--logfile", "--api-server", "--input-tty", "--dictionary"}
    arguments = iter(argv[index + 1:])
    for arg in arguments:
        if arg in values:
            value = next(arguments, None)
            if value is None:
                return None
            if arg in ("-a", "--api-server"):
                api_socket = canonical(value, cwd)
        elif arg.startswith("--api-server="):
            api_socket = canonical(arg.split("=", 1)[1], cwd)
        elif not arg.startswith("-"):
            positional.append(arg)
    if len(positional) != 1:
        return None
    return dict(repo=script.parent.parent, config=canonical(positional[0], cwd), socket=api_socket,
                pid=pid, service=service, origin=origin, executable=argv[0])


def proc_instances(proc_root=Path("/proc")):
    instances = []
    if not proc_root.is_dir():
        return instances
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            argv = [p.decode("utf-8", "surrogateescape") for p in
                    (process / "cmdline").read_bytes().split(b"\0") if p]
            if not any(Path(arg).name == "klippy.py" for arg in argv):
                continue
            service, scope = None, None
            if (process / "cgroup").is_file():
                groups = (process / "cgroup").read_text()
                names = re.findall(r"/([^/\n]+\.service)(?:/|$)", groups)
                service = names[-1] if names else None
                scope = "user" if "/user.slice/" in groups else "system"
            found = command_instance(argv, (process / "cwd").resolve(),
                                     "running process " + process.name, int(process.name), service)
            if found:
                found["service_scope"] = scope
                # Field 22 follows the parenthesized command, which may contain spaces.
                try:
                    fields = (process / "stat").read_text().rsplit(") ", 1)[1].split()
                    found["start_time"] = fields[19] if fields[19].isdigit() else None
                except (OSError, IndexError):
                    found["start_time"] = None
                instances.append(found)
        except (OSError, ValueError):
            continue
    return instances


def service_instances(home):
    directories = (Path("/etc/systemd/system"), Path("/lib/systemd/system"),
                   Path("/usr/lib/systemd/system"), home / ".config/systemd/user")
    instances, seen = [], set()
    for directory in directories:
        if not directory.is_dir():
            continue
        for unit in sorted(directory.glob("*.service")):
            if unit.name in seen:
                continue
            seen.add(unit.name)
            try:
                text = unit.read_text()
                if "klippy.py" not in text:
                    continue
                # Drop-ins/environment expansion require actual live arguments, not guessing.
                if any((directory / (unit.name + ".d")).glob("*.conf")):
                    continue
                text = re.sub(r"\\\r?\n\s*", " ", text)
                command = re.findall(r"^\s*ExecStart\s*=\s*(.+)$", text, re.M)
                working = re.findall(r"^\s*WorkingDirectory\s*=\s*(.+)$", text, re.M)
                if len(command) != 1 or "$" in command[0] or "%" in command[0]:
                    continue
                found = command_instance(shlex.split(command[0]), working[-1].strip() if working else "/",
                                         str(unit), service=unit.name)
                if found:
                    found["service_scope"] = "user" if directory == directories[-1] else "system"
                    instances.append(found)
            except (OSError, ValueError):
                continue
    return instances


def active_service(service):
    systemctl = shutil.which("systemctl")
    if not service or systemctl is None:
        return None
    try:
        result = subprocess.run([systemctl, "--no-ask-password", "show", "--property=ActiveState", "--value", "--", service],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def describe(instances):
    return "\n".join("  %s: %s ; %s ; service=%s" %
                     (p["origin"], p["repo"], p["config"], p["service"] or "unknown") for p in instances)


def discover_target(args, home=None, processes=None, services=None):
    home = canonical(home or Path.home())
    processes = proc_instances() if processes is None else processes
    services = service_instances(home) if services is None else services
    repo = canonical(args.klipper_path) if args.klipper_path else None
    config = canonical(args.config) if args.config else None
    service = args.service
    if service and not service.endswith(".service"):
        service += ".service"

    def matches(item):
        return ((repo is None or item["repo"] == repo) and (config is None or item["config"] == config)
                and (service is None or item["service"] == service))

    running = [p for p in processes if matches(p)]
    if len(running) > 1:
        raise InstallError("Multiple running instances match. Set --klipper-path, --config and --service.\n" + describe(running))
    if running:
        selected = dict(running[0])
    else:
        configured = {(p["repo"], p["config"], p["service"]): p for p in services if matches(p)}
        if len(configured) > 1:
            raise InstallError("Multiple services match. Set --klipper-path, --config and --service.\n" + describe(configured.values()))
        if configured:
            selected = dict(next(iter(configured.values())))
        else:
            if service:
                raise InstallError("Cannot resolve service %s. Supply the actual instance arguments; "
                                   "environment-based units/drop-ins require a readable running /proc command." % service)
            repos = [repo] if repo else sorted({canonical(p) for pattern in ("klipper", "kalico", "klipper-*", "kalico-*")
                                               for p in home.glob(pattern) if (p / "klippy/klippy.py").is_file()})
            configs = [config] if config else sorted({canonical(p) for pattern in (
                "printer_data/config/printer.cfg", "printer_data*/config/printer.cfg", "printer_*/config/printer.cfg", "printer.cfg")
                for p in home.glob(pattern) if p.is_file()})
            if len(repos) != 1 or len(configs) != 1:
                raise InstallError("Cannot choose one checkout/main config. Set --klipper-path /path/to/klipper "
                                   "--config /path/to/printer.cfg (and --service on multi-instance hosts). "
                                   "Checkouts: %s; configs: %s" % (repos, configs))
            selected = dict(repo=repos[0], config=configs[0], socket=None, service=None, pid=None,
                            executable=None, origin="explicit paths/common directories (stopped host)")
        overlap = [p for p in processes if p["repo"] == selected["repo"] or p["config"] == selected["config"]]
        if overlap:
            raise InstallError("Selected paths overlap a live instance with different arguments/service. "
                               "Use exact --klipper-path, --config and --service.\n" + describe(overlap))
    if args.klippy_socket:
        selected["socket"] = canonical(args.klippy_socket)
    if not (selected["repo"] / "klippy/klippy.py").is_file() or not (selected["repo"] / "klippy/extras").is_dir():
        raise InstallError("Not a compatible checkout (requires klippy/klippy.py and klippy/extras): %s" % selected["repo"])
    if not selected["config"].is_file():
        raise InstallError("Main config not found: %s" % selected["config"])
    return selected


class Section:
    def __init__(self, document, name, start, end):
        self.document, self.name, self.start, self.end = document, name, start, end
        self.options = {}
        for index in range(start + 1, end):
            if document.lines[index].lstrip().startswith(("#", ";")):
                continue
            match = OPTION.match(document.lines[index])
            if match:
                self.options.setdefault(match.group(2).lower(), []).append((index, match))

    def get(self, name, default=None):
        options = self.options.get(name.lower(), [])
        if len(options) > 1:
            raise InstallError("Duplicate %s in [%s], %s; resolve before installation." % (name, self.name, self.document.path))
        return re.split(r"\s*[#;]", options[0][1].group(4), maxsplit=1)[0].strip() if options else default

    def replace(self, name, value):
        if self.get(name) == str(value):
            return
        entries = self.options.get(name.lower(), [])
        line = "%s: %s%s" % (name, value, self.document.newline)
        if entries:
            index, match = entries[0]
            old = self.document.lines[index].lstrip().rstrip("\r\n")
            self.document.replacements[index] = "# accel_homing previous: " + old + self.document.newline + match.group(1) + line
        else:
            self.document.insertions.setdefault(self.end, []).append(line)

    def disable(self, name):
        if self.get(name) is not None:
            index = self.options[name.lower()][0][0]
            self.document.replacements[index] = "# accel_homing previous: " + self.document.lines[index].lstrip()

    def generic_name(self):
        if self.name.lower() != "beacon_accel_homing":
            return
        line = self.document.lines[self.start]
        match = SECTION.match(line)
        name = re.sub("beacon_accel_homing", "accel_homing", match.group(1), flags=re.I)
        self.document.replacements[self.start] = line[:match.start(1)] + name + line[match.end(1):]

    def generic_pin(self, axis):
        if self.get("endstop_pin") != "beacon_accel:" + axis:
            return False
        index, match = self.options["endstop_pin"][0]
        line = self.document.lines[index]
        self.document.replacements[index] = (line[:match.start(4)]
            + match.group(4).replace("beacon_accel:" + axis, "accel_homing:" + axis, 1)
            + line[match.end(4):])
        return True


class ConfigDocument:
    def __init__(self, path, original=None):
        self.path = canonical(path)
        self.original = self.path.read_bytes() if original is None else original
        self.text = self.original.decode("utf-8")
        self.newline = "\r\n" if "\r\n" in self.text else "\n"
        self.lines = self.text.splitlines(keepends=True)
        self.replacements, self.insertions, self.sections = {}, {}, []
        heads, self.active_end = [], len(self.lines)
        for index, line in enumerate(self.lines):
            if line.startswith(SAVE_MARKER) or line.startswith("#*# DO NOT EDIT"):
                self.active_end = index
                break
            match = SECTION.match(line)
            if match:
                heads.append((index, match.group(1).strip()))
        for i, (start, name) in enumerate(heads):
            end = heads[i + 1][0] if i + 1 < len(heads) else self.active_end
            self.sections.append(Section(self, name, start, end))

    def rendered(self):
        output = []
        for index in range(len(self.lines) + 1):
            additions = self.insertions.get(index, [])
            if additions and output and not output[-1].endswith(("\n", "\r")):
                output.append(self.newline)
            output.extend(additions)
            if index < len(self.lines):
                output.append(self.replacements.get(index, self.lines[index]))
        return "".join(output).encode("utf-8")


class ConfigGraph:
    def __init__(self, main):
        self.main, self.documents, self.sections, self.include_patterns = canonical(main), {}, {}, []
        self._visit(self.main, [])

    def _visit(self, path, stack):
        path = canonical(path)
        if path in stack:
            raise InstallError("Recursive config include: " + " -> ".join(map(str, stack + [path])))
        if path in self.documents:
            raise InstallError("Config included more than once: %s. Resolve duplicate includes." % path)
        try:
            document = ConfigDocument(path)
        except (OSError, UnicodeError) as exc:
            raise InstallError("Cannot read UTF-8 config %s: %s" % (path, exc))
        self.documents[path] = document
        for section in document.sections:
            if section.name.lower().startswith("include "):
                pattern = os.path.expanduser(section.name[8:].strip())
                if not Path(pattern).is_absolute():
                    pattern = str(path.parent / pattern)
                self.include_patterns.append(pattern)
                matches = sorted(glob.glob(pattern))
                if not matches:
                    raise InstallError("Include has no matches in %s: %s" % (path, section.name))
                for match in matches:
                    self._visit(match, stack + [path])
            else:
                self.sections.setdefault(section.name.lower(), []).append(section)

    def one(self, name, required=True):
        sections = self.sections.get(name.lower(), [])
        if not sections and not required:
            return None
        if len(sections) != 1:
            raise InstallError("Expected exactly one [%s], found %d. Resolve duplicate/missing sections." % (name, len(sections)))
        return sections[0]


def choose_sensor(graph, selected=None, existing=None):
    configured = [name for name in graph.sections if name.split()[0] in SENSORS]
    preferred = selected
    if preferred is None and existing:
        preferred = existing.get("accel_chip")
        if not preferred and existing.name.lower() == "beacon_accel_homing":
            preferred = existing.get("beacon", "beacon")
    if preferred and preferred.lower() != "auto":
        preferred = preferred.lower().strip()
        if preferred not in graph.sections:
            raise InstallError("Accelerometer [%s] is not configured. Configure its sensor/firmware first, "
                               "or select --accel-chip from: %s" % (preferred, ", ".join(configured) or "none"))
        return graph.one(preferred).name
    if len(configured) != 1:
        raise InstallError("Expected one configured accelerometer, found %d (%s). Use --accel-chip "
                           "'adxl345 toolhead' (full existing section). A Cartographer/scanner probe alone "
                           "is not an accelerometer; configure its separate [adxl345 ...] first." %
                           (len(configured), ", ".join(configured) or "none"))
    return graph.one(configured[0]).name


def validate_generic_names(graph, existing):
    """Only the known plugin header and X/Y endstop aliases can be renamed."""
    recognized = set()
    if existing and existing.name.lower() == "beacon_accel_homing":
        recognized.add((existing.document.path, existing.start))
    for axis in ("x", "y"):
        rail = graph.one("stepper_" + axis, False)
        if rail and rail.get("endstop_pin") == "beacon_accel:" + axis:
            recognized.add((rail.document.path, rail.options["endstop_pin"][0][0]))
    for document in graph.documents.values():
        include_headers = {section.start for section in document.sections
                           if section.name.lower().startswith("include ")}
        for index, line in enumerate(document.lines[:document.active_end]):
            if index in include_headers or (document.path, index) in recognized:
                continue
            active = re.split(r"[#;]", line, maxsplit=1)[0]
            if LEGACY_REFERENCE.search(active):
                raise InstallError("--generic-names cannot safely migrate a legacy macro/object/command "
                                   "reference in %s:%d. Adapt that active reference explicitly first; "
                                   "no files changed." % (document.path, index + 1))


def validate_homing_speeds(graph, args, family, existing):
    """Validate selected rails and retained users of explicit shared limits."""
    overrides = {name: getattr(args, name) for name in
                 ("max_report_travel", "max_startup_travel")
                 if getattr(args, name) is not None}
    if family == "beacon" and overrides:
        raise InstallError("%s applies only to configured accelerometer backends, not native Beacon. "
                           "Remove the unsupported override. No files changed."
                           % ", ".join("--" + name.replace("_", "-") for name in overrides))
    if family != "beacon":
        policy = load_motion_policy()
        printer = graph.one("printer")
        try:
            velocity = float(printer.get("max_velocity", "nan"))
            acceleration = float(printer.get("max_accel", "nan"))
            homing_accel = float(existing.get("homing_accel", "5000")) if existing else 5000.0
            age = float(existing.get("max_sample_age", "0.070")) if existing else 0.070
            travel = (overrides["max_report_travel"] if "max_report_travel" in overrides else
                      existing.get("max_report_travel") if existing else None)
            startup = (overrides["max_startup_travel"] if "max_startup_travel" in overrides else
                       existing.get("max_startup_travel") if existing else None)
            travel = float(travel) if travel is not None else None
            startup = float(startup) if startup is not None else None
            interval = float(existing.get("stream_interval", "0.005")) if existing else 0.005
        except ValueError:
            raise InstallError("Generic homing requires numeric printer velocity/acceleration and plugin motion-policy settings.")
        if not all(math.isfinite(v) and v > 0 for v in (velocity, acceleration)) or not math.isfinite(homing_accel) or homing_accel < 0:
            raise InstallError("Generic homing requires finite positive printer max_velocity/max_accel and nonnegative homing_accel.")
        if homing_accel:
            acceleration = min(acceleration, homing_accel)
        if (not all(math.isfinite(v) for v in (age, interval))
                or not 0.005 <= age <= 0.070
                or (travel is not None and (not math.isfinite(travel)
                    or not policy.REPORT_TRAVEL_MIN <= travel <= policy.REPORT_TRAVEL_MAX))
                or (startup is not None and (not math.isfinite(startup)
                    or not policy.STARTUP_REQUEST_TRAVEL_MIN <= startup <= policy.STARTUP_REQUEST_TRAVEL_MAX))
                or not 0.001 <= interval <= 0.020 or interval >= age):
            raise InstallError("Generic settings require max_sample_age 0.005..0.070s, "
                               "optional max_report_travel %.12g..%.12gmm, optional max_startup_travel %.12g..%.12gmm, "
                               "stream_interval 0.001..0.020s and stream_interval < max_sample_age; "
                               "all values must be finite. No files changed."
                               % (policy.REPORT_TRAVEL_MIN, policy.REPORT_TRAVEL_MAX,
                                  policy.STARTUP_REQUEST_TRAVEL_MIN, policy.STARTUP_REQUEST_TRAVEL_MAX))
        cap = policy.motion_report_speed_limit(age, travel) if travel is not None else None
    axes = list(args.axes)
    if overrides:
        # These settings belong to the whole backend. A partial update must
        # not invalidate another rail already connected to this plugin.
        for axis in ("x", "y"):
            rail = graph.one("stepper_" + axis, False)
            if (axis not in axes and rail is not None
                    and rail.get("endstop_pin", "").lstrip("!^~") in
                    ("accel_homing:" + axis, "beacon_accel:" + axis)):
                axes.append(axis)
    speeds = {}
    for axis in axes:
        rail = graph.one("stepper_" + axis)
        try:
            original_speed = float(rail.get("homing_speed", "5"))
        except ValueError:
            raise InstallError("%s homing_speed must be numeric. No files changed." % axis.upper())
        requested = args.homing_speed is not None and axis in args.axes
        speed = args.homing_speed if requested else original_speed
        if not math.isfinite(speed) or speed <= 0:
            raise InstallError("Homing speeds must be finite and positive. No files changed.")
        effective = min(speed, velocity) if family != "beacon" else speed
        if family != "beacon":
            rail_acceleration = acceleration
            passes = [("requested --homing-speed" if requested else "configured homing_speed", speed)]
            if axis not in args.axes:
                # Selected rails lose retract/min_home_dist/homing_accel during
                # conversion. Retained plugin rails keep every active pass and
                # Kalico's per-rail acceleration, so validate them before edits.
                try:
                    retract = float(rail.get("homing_retract_dist", "5"))
                    minimum = float(rail.get("min_home_dist", str(retract)))
                    rail_accel = rail.get("homing_accel")
                    rail_accel = float(rail_accel) if rail_accel is not None else None
                except ValueError:
                    raise InstallError("%s retained homing distances/acceleration must be numeric. No files changed."
                                       % axis.upper())
                if not all(math.isfinite(value) and value >= 0 for value in (retract, minimum)):
                    raise InstallError("%s retained homing distances must be finite and nonnegative. No files changed."
                                       % axis.upper())
                if rail_accel is not None:
                    if not math.isfinite(rail_accel) or rail_accel <= 0:
                        raise InstallError("%s retained homing_accel must be finite and positive. No files changed."
                                           % axis.upper())
                    rail_acceleration = min(rail_acceleration, rail_accel)
                if retract or minimum:
                    second = rail.get("second_homing_speed")
                    if second is not None:
                        try:
                            second = float(second)
                        except ValueError:
                            raise InstallError("%s second_homing_speed must be numeric. No files changed."
                                               % axis.upper())
                        if not math.isfinite(second) or second <= 0:
                            raise InstallError("%s second_homing_speed must be finite and positive. No files changed."
                                               % axis.upper())
                        passes.append(("configured second_homing_speed", second))
                    # Known Klipper/Kalico defaults are first/2 or first; the
                    # validated first pass conservatively covers both defaults.
            for setting, pass_speed in passes:
                pass_effective = min(pass_speed, velocity)
                timeout = policy.motion_startup_timeout(pass_effective, rail_acceleration, age, interval)
                budget = policy.motion_startup_request_travel(pass_effective, rail_acceleration, age, interval)
                if not all(math.isfinite(value) for value in (timeout, budget)):
                    raise InstallError("%s %s produces non-finite startup timing/travel at effective speed %.12g mm/s "
                                       "and acceleration %.12g mm/s^2. No files changed."
                                       % (axis.upper(), setting, pass_effective, rail_acceleration))
                if ((cap is not None and pass_effective > cap)
                        or (startup is not None and budget > startup)):
                    report_policy = ("report-speed limit %.12g mm/s for %.12g s max_sample_age / %.12g mm max_report_travel"
                                     % (cap, age, travel) if cap is not None else "report-distance cap disabled")
                    startup_policy = ("startup stop request needs %.12g mm of commanded travel (%.12g mm max_startup_travel limit)"
                                      % (budget, startup) if startup is not None else "startup-distance cap disabled")
                    raise InstallError(
                        "%s %s %.12g mm/s (current configured homing_speed %.12g mm/s; "
                        "effective speed %.12g mm/s, acceleration %.12g mm/s^2) exceeds generic "
                        "policies: %s; %s. Homing speed is preserved by "
                        "default; choose compatible settings explicitly. No files changed."
                        % (axis.upper(), setting, pass_speed, original_speed, pass_effective, rail_acceleration,
                           report_policy, startup_policy))
        if axis in args.axes:
            speeds[axis] = rail, speed, effective
    return speeds


def assert_migration_state(args):
    for path, expected in getattr(args, "_migration_expected", {}).items():
        if not state_matches(path, expected):
            raise InstallError("%s changed after migration ownership validation. No printer transaction files changed; retry." % path)


def make_plan(source, selected, args):
    assert_migration_state(args)
    source = canonical(source)
    for module in MODULES:
        if not (source / module).is_file():
            raise InstallError("Incomplete package: missing %s in %s" % (module, source))
    graph = ConfigGraph(selected["config"])
    safe_z_home = graph.one("safe_z_home", False)
    homing_override = graph.one("homing_override", False)
    if safe_z_home is not None and homing_override is not None:
        raise InstallError("[safe_z_home] and [homing_override] cannot be used together. "
                           "Resolve the conflicting built-in homing sections. No files changed.")
    custom_g28 = [name for name in graph.sections if re.match(r"^gcode_macro\s+g28$", name, re.I)]
    if custom_g28 and not args.acknowledge_homing_override:
        raise InstallError("Custom homing path: %s. No files changed. Review/adapt its G28 sequence, "
                           "motor current and enable setup, and moves; only then use --acknowledge-homing-override "
                           "to acknowledge installation. The installer cannot validate custom G28 "
                           "macro behavior, and supervised workflows still reject that macro path."
                           % ", ".join(custom_g28))
    plugins = [graph.one(name, False) for name in ("accel_homing", "beacon_accel_homing")]
    plugins = [section for section in plugins if section]
    if len(plugins) > 1:
        raise InstallError("Use either [accel_homing] or [beacon_accel_homing], not both.")
    existing = plugins[0] if plugins else None
    generic_names = getattr(args, "generic_names", False)
    if generic_names:
        load_uninstaller().validate_installation_history(selected, uninstall_api())
        validate_generic_names(graph, existing)
    sensor = choose_sensor(graph, args.accel_chip, existing)
    family = sensor.split()[0].lower()
    if not any((selected["repo"] / "klippy" / location / (family + ".py")).is_file()
               or (selected["repo"] / "klippy" / location / family / "__init__.py").is_file()
               for location in ("extras", "plugins")):
        raise InstallError("Configured [%s] has no installed driver in klippy/extras or klippy/plugins. "
                           "Install its normal driver first." % sensor)
    speeds = validate_homing_speeds(graph, args, family, existing)
    changes, messages = {}, []
    if safe_z_home is not None:
        messages.append("Preserved [safe_z_home]. Supervised X/Y workflows verify its stock G28 "
                        "predecessor and omit the wrapper's z_hop; ordinary G28 remains unchanged.")
    if homing_override is not None:
        messages.append("Preserved [homing_override] and its normal G28 sequence. Supervised X/Y "
                        "workflows require its verified stock G28 predecessor and bypass all override "
                        "G-code, including current/enable setup, SYNC_MOTORS and Z/Beacon actions. "
                        "Prepare suitable test currents/enables and nozzle clearance independently.")
    prefix = ("beacon_accel" if existing and existing.name.lower() == "beacon_accel_homing"
              and not generic_names else "accel_homing")
    if generic_names:
        if existing:
            existing.generic_name()
        for axis in ("x", "y"):
            rail = graph.one("stepper_" + axis, False)
            if rail:
                rail.generic_pin(axis)
        messages.append("Generic plugin section and X/Y pin names requested; active calibration and SAVE_CONFIG preserved.")
    overrides = {name: repr(getattr(args, name)) for name in
                 ("max_report_travel", "max_startup_travel")
                 if getattr(args, name) is not None}
    if overrides:
        messages.append("Explicit generic policy overrides: %s. These nominal request-travel "
                        "limits are not physical stopping guarantees."
                        % ", ".join("%s=%s mm" % item for item in overrides.items()))
        messages.append("%s homing_speed: %s."
                        % ("Preserved configured" if args.homing_speed is None else "Explicitly selected",
                           ", ".join("%s=%.12g mm/s" % (axis.upper(), speed)
                                     for axis, (_rail, speed, _effective) in speeds.items())))
    if existing:
        existing.replace("accel_chip", sensor)
        for name, value in overrides.items():
            existing.replace(name, value)
    else:
        managed = selected["config"].parent / MANAGED
        if managed.exists() or managed.is_symlink():
            raise InstallError("%s exists but is not included; review/include or rename it. It will not be overwritten." % managed)
        content = ("# Managed by the accelerometer homing installer. Edit this file normally.\n"
                   "# Independent overrides; leave commented to use SAVE_CONFIG calibration models.\n"
                   "[accel_homing]\naccel_chip: " + sensor + "\n"
                    "#x_threshold: 8.0\n#y_threshold: 8.0\n#model: default\n"
                    + "".join("%s: %s\n" % item for item in overrides.items()))
        changes[managed] = {"kind": "file", "data": content.encode("utf-8")}
        if not any(managed.match(pattern) for pattern in graph.include_patterns):
            main = graph.documents[selected["config"]]
            main.insertions.setdefault(main.active_end, []).append(main.newline + "[include " + MANAGED + "]" + main.newline + main.newline)
    for axis, (rail, speed, actual_speed) in speeds.items():
        if not (generic_names and rail.get("endstop_pin") == "beacon_accel:" + axis):
            rail.replace("endstop_pin", prefix + ":" + axis)
        rail.replace("homing_retract_dist", "0")
        for key in ("homing_accel", "min_home_dist"):
            rail.disable(key)
        if args.homing_speed is not None:
            rail.replace("homing_speed", "%.12g" % speed)
        if actual_speed < 50:
            messages.append("%s speed %.3g mm/s may produce soft, indistinguishable impacts. "
                            "Supervised calibration is required." % (axis.upper(), actual_speed))
        for name in graph.sections:
            if re.match(r"^tmc\w+ stepper_" + axis + r"\d*$", name):
                driver = graph.one(name)
                for key in ("diag_pin", "diag0_pin", "diag1_pin", "home_current"):
                    driver.disable(key)
    for document in graph.documents.values():
        data = document.rendered()
        if data != document.original:
            changes[document.path] = {"kind": "file", "data": data}
    for module in MODULES:
        link, wanted = selected["repo"] / "klippy/extras" / module, str(source / module)
        if link.is_symlink():
            if canonical(link) == canonical(wanted):
                continue
            if not args.replace_existing_links:
                raise InstallError("%s links to %s. Review it, then --replace-existing-links to back up/replace." % (link, os.readlink(link)))
        elif link.exists():
            raise InstallError("Refusing to overwrite non-symlink plugin %s. Resolve manually." % link)
        changes[link] = {"kind": "symlink", "target": wanted}
        if link in getattr(args, "_migration_expected", {}):
            changes[link]["expected"] = dict(args._migration_expected[link])
    exclude = selected["repo"] / ".git/info/exclude"
    if exclude.parent.is_dir():
        before = exclude.read_bytes() if exclude.exists() else b""
        text = before.decode("utf-8")
        missing = ["klippy/extras/" + name for name in MODULES if "klippy/extras/" + name not in text.splitlines()]
        if missing:
            text += ("\n" if text and not text.endswith("\n") else "") + "\n".join(missing) + "\n"
            changes[exclude] = {"kind": "file", "data": text.encode("utf-8"),
                                "expected": snapshot(exclude)}
            if changes[exclude]["expected"].get("kind") == "file":
                changes[exclude]["expected"]["data"] = base64.b64encode(before).decode("ascii")
    for path, change in changes.items():
        change.setdefault("expected", snapshot(path))
        if path in graph.documents:
            change["expected"]["data"] = base64.b64encode(graph.documents[path].original).decode("ascii")
    return changes, messages, sensor


def webhook_query(api_path, method, params=None):
    request = {"id": 1, "method": method, "params": params or {}}
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(3.0)
        client.connect(str(api_path))
        client.sendall(json.dumps(request).encode("utf-8") + b"\x03")
        data = b""
        while len(data) < 1024 * 1024:
            chunk = client.recv(65536)
            if not chunk:
                break
            data += chunk
            while b"\x03" in data:
                packet, data = data.split(b"\x03", 1)
                response = json.loads(packet)
                if response.get("id") == 1:
                    if "error" in response:
                        raise InstallError("Klipper status query failed: %s" % response["error"])
                    return response.get("result", {})
    raise InstallError("No complete reply from Klipper API socket %s" % api_path)


def assert_idle(selected, query=None, service_state=None, require_process_identity=False):
    query = webhook_query if query is None else query
    service_state = active_service if service_state is None else service_state
    state = service_state(selected["service"])
    if require_process_identity and selected["pid"] is not None and selected["service"] and state != "active":
        raise InstallError("Selected live service is no longer verifiably active (%s); refusing writes/restart." % state)
    active = selected["pid"] is not None or state in ("active", "activating", "reloading")
    if not active:
        if selected["service"] and state is None:
            raise InstallError("Cannot verify %s is stopped. Inspect/stop it and retry with readable systemd status." % selected["service"])
        return "Selected Klipper is stopped (no live process/active service detected)."
    if not selected["socket"]:
        raise InstallError("Running Klipper has no known API socket. Set --klippy-socket /path/from/-a "
                           "or stop the selected service. Config writes require idle evidence.")
    try:
        info = query(selected["socket"], "info")
        if not info.get("config_file") or canonical(info["config_file"]) != selected["config"]:
            raise InstallError("API socket belongs to a different/unknown config (%s). "
                               "Use the selected instance's --klippy-socket." % info.get("config_file"))
        if info.get("klipper_path") and canonical(info["klipper_path"]) != selected["repo"]:
            raise InstallError("API socket Klipper checkout does not match selected --klipper-path.")
        if require_process_identity and (info.get("process_id") != selected["pid"]
                or not info.get("klipper_path") or canonical(info["klipper_path"]) != selected["repo"]):
            raise InstallError("API socket process/checkout identity does not match the selected service MainPID.")
        if info.get("state") != "ready":
            raise InstallError("Klipper is not ready/idle (%s); stop it before repairing config." % info.get("state", "unknown"))
        objects = query(selected["socket"], "objects/list").get("objects", [])
        names = [name for name in ("idle_timeout", "print_stats", "pause_resume") if name in objects]
        if "idle_timeout" not in names:
            raise InstallError("No idle_timeout evidence; stop the selected service first.")
        status = query(selected["socket"], "objects/query", {"objects": {name: None for name in names}}).get("status", {})
        idle = status.get("idle_timeout", {}).get("state")
        printing = status.get("print_stats", {}).get("state") if "print_stats" in names else "standby"
        paused = status.get("pause_resume", {}).get("is_paused") if "pause_resume" in names else False
        if (idle not in ("Idle", "Ready") or printing not in ("standby", "complete", "cancelled", "error")
                or paused is not False):
            raise InstallError("Refusing writes: printing/paused/unknown state (idle_timeout=%s, print_stats=%s, paused=%s)." % (idle, printing, paused))
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        raise InstallError("Cannot obtain read-only idle evidence from %s: %s. Stop the selected service and retry." % (selected["socket"], exc))
    return "Selected live Klipper reports ready/idle (read-only API query)."


def service_command(command, timeout=5, capture=True):
    """Every service-manager command is bounded, argv-only, and non-shell."""
    try:
        return subprocess.run(command, timeout=timeout, text=True,
                              stdout=subprocess.PIPE if capture else None,
                              stderr=subprocess.PIPE if capture else None)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise InstallError("Host service command did not complete: %s" % exc)


def read_service_unit(selected, systemctl):
    service = selected.get("service")
    if not service or not SERVICE_NAME.fullmatch(service):
        raise InstallError("Automatic restart needs one exact, valid detected systemd service. "
                           "Select --service or use --no-restart with the actual service manager.")
    if selected.get("service_scope") == "user":
        raise InstallError("Selected Klipper uses a user systemd manager. Use --no-restart and "
                           "restart that user's service manually; system services will not be guessed.")
    result = service_command([systemctl, "--system", "--no-ask-password", "show", "--all",
                              "--property=" + ",".join(SERVICE_PROPERTIES), "--", service])
    properties = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if result.returncode != 0 or properties.get("Id") != service or properties.get("LoadState") != "loaded":
        raise InstallError("Cannot verify the loaded systemd unit %s; no service will be guessed. "
                           "Use --no-restart for another service manager. %s" % (service, result.stderr.strip()))
    if properties.get("NeedDaemonReload") != "no":
        raise InstallError("%s has an unknown/pending unit reload. Reconcile its effective service definition first." % service)
    if (properties.get("CanStart") != "yes" or properties.get("CanStop") != "yes"
            or properties.get("RefuseManualStart") != "no" or properties.get("RefuseManualStop") != "no"):
        raise InstallError("%s does not permit a verified manual service restart." % service)
    if "/user.slice/" in properties.get("ControlGroup", ""):
        raise InstallError("Refusing to control a user service through the system manager.")
    return properties


def effective_service_instance(properties, live=None):
    # `show` supplies the manager's effective command, including loaded drop-ins.
    # A standard live `python $KLIPPER_ARGS` unit is identified by its MainPID's
    # actual /proc arguments, not by interpreting an environment file as a shell.
    # Stopped units require a literal command because that live proof is absent.
    command = properties.get("ExecStart", "")
    arguments = re.findall(r"(?:^|;\s*)argv\[\]=(.*?)\s*;\s*ignore_errors=", command)
    if len(arguments) != 1 or any(token in arguments[0] for token in ("%", "\\x")):
        raise InstallError("Cannot resolve the effective systemd ExecStart to one Klipper command. "
                           "Environment-based/wrapped units require --no-restart and manual activation.")
    try:
        argv = shlex.split(arguments[0])
    except ValueError as exc:
        raise InstallError("Cannot parse effective systemd ExecStart: %s" % exc)
    working = properties.get("WorkingDirectory") or "/"
    variables = [arg for arg in argv if "$" in arg]
    if variables:
        if (live is None or not argv or not Path(argv[0]).is_absolute()
                or canonical(argv[0]) != canonical(live["executable"])
                or any(not re.fullmatch(r"\$[A-Za-z_]\w*|\$\{[A-Za-z_]\w*\}", value) for value in variables)):
            raise InstallError("Environment-based ExecStart needs a matching live main process; "
                               "use --no-restart for stopped/unresolved units.")
        found = live
    else:
        found = command_instance(argv, working, "effective systemd ExecStart", service=properties["Id"])
    if not found:
        raise InstallError("Effective systemd ExecStart does not identify one Klipper/Kalico instance; use --no-restart.")
    definition = {key: properties.get(key, "") for key in (
        "Id", "WorkingDirectory", "FragmentPath", "DropInPaths", "User", "Group", "Environment", "EnvironmentFiles")}
    definition["argv"] = argv
    return found, definition


def service_identity(selected, systemctl):
    properties = read_service_unit(selected, systemctl)
    try:
        pid = int(properties["MainPID"])
    except (KeyError, ValueError):
        raise InstallError("Cannot verify the selected service MainPID.")
    processes = proc_instances()
    relevant = [item for item in processes if item["config"] == selected["config"]
                or (item.get("service") == selected["service"] and item.get("service_scope") != "user")]
    active = properties.get("ActiveState")
    live = None
    if active in ("inactive", "failed") and pid == 0 and not relevant:
        pid = None
    else:
        if active != "active" or properties.get("SubState") != "running" or pid <= 0 or len(relevant) != 1:
            error = ServiceStarting if active in ("active", "activating") and not relevant else InstallError
            raise error("Selected service/process is changing or ambiguous (%s/%s, MainPID=%s); retry when stable." %
                        (active, properties.get("SubState"), pid))
        live = relevant[0]
        if (live["pid"] != pid or live.get("service") != selected["service"]
                or live.get("service_scope") != "system" or not live.get("start_time")):
            raise InstallError("Selected systemd MainPID/process creation identity cannot be verified.")
    effective, definition = effective_service_instance(properties, live)
    for key in ("repo", "config", "socket"):
        if effective[key] != selected[key]:
            raise InstallError("Effective service %s does not match the selected %s." % (selected["service"], key))
        if live and live[key] != selected[key]:
            raise InstallError("Live service process does not match the selected %s." % key)
    return dict(definition=definition, pid=pid, start_time=live["start_time"] if live else None,
                invocation=properties.get("InvocationID", ""), state=active)


def restart_permission(plan, acquire=False):
    # sudoers may permit command *listing* without authenticating execution.
    # Validate the timestamp independently before checking this exact command.
    validation = service_command([plan["sudo"], "-n", "-v"])
    if validation.returncode and acquire:
        print("Authorize the detected host-service restart before any idle check or file writes.")
        validation = service_command([plan["sudo"], "-v"], timeout=60, capture=False)
        if validation.returncode:
            raise InstallError("Cannot obtain sudo authorization for the selected service restart; no files changed.")
        validation = service_command([plan["sudo"], "-n", "-v"])
    if validation.returncode:
        raise InstallError("Noninteractive sudo authentication is unavailable. Authorize it before retrying, "
                           "or use --no-restart.")
    command = [plan["sudo"], "-n", "-l", "--"] + plan["service_command"]
    result = service_command(command)
    if result.returncode:
        raise InstallError("Noninteractive sudo authorization for the exact service restart is unavailable. "
                           "Authorize it before retrying, or use --no-restart. %s" % result.stderr.strip())


def prepare_restart(selected):
    systemctl, sudo = shutil.which("systemctl"), shutil.which("sudo")
    if not systemctl or not sudo:
        raise InstallError("Automatic restart requires systemctl and sudo. Use --no-restart for manual host activation.")
    identity = service_identity(selected, systemctl)
    if (identity["pid"] != selected.get("pid")
            or (identity["pid"] and identity["start_time"] != selected.get("start_time"))):
        raise InstallError("Selected service/process changed since detection. Retry before installing.")
    plan = dict(systemctl=systemctl, sudo=sudo, identity=identity,
                service_command=[systemctl, "--system", "--no-ask-password", "restart", "--", selected["service"]])
    restart_permission(plan, acquire=True)
    return plan


def assert_restart_safe(selected, plan):
    # These commands cannot ask for credentials. Permission checks always precede
    # the final identity/API evidence, including after a slow file transaction.
    restart_permission(plan)
    if service_identity(selected, plan["systemctl"]) != plan["identity"]:
        raise InstallError("Selected service/process identity changed; refusing writes/restart.")
    print(assert_idle(selected, require_process_identity=True))


def activation_path(selected, backup=None):
    if backup is None:
        _, active, _ = load_uninstaller().history(selected["config"], uninstall_api())
        if active:
            backup = active[-1]["directory"]
    return (backup / "activation.json" if backup else
            selected["config"].parent / ".accel_homing_activation.json")


def save_activation(path, selected, status, attempted=False, **details):
    receipt = dict(version=1, status=status, restart_attempted=attempted, repo=str(selected["repo"]),
                   config=str(selected["config"]), service=selected.get("service"),
                   updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), **details)
    data = json.dumps(receipt, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    write_state(path, {"kind": "file", "mode": 0o600, "data": base64.b64encode(data).decode("ascii")})


def restarted_process(selected, plan):
    deadline = time.monotonic() + 10
    before = plan["identity"]
    while True:
        try:
            after = service_identity(selected, plan["systemctl"])
        except ServiceStarting:
            after = None  # A Type=simple process may not have exec'd Python yet.
        if after:
            if after["definition"] != before["definition"] or after["state"] != "active" or after["pid"] is None:
                raise InstallError("Restart did not produce the selected matching active host service.")
            if (after["pid"], after["start_time"]) != (before["pid"], before["start_time"]):
                return after
        if time.monotonic() >= deadline:
            raise InstallError("Restart returned successfully but a new matching active host process was not verified within 10 seconds.")
        time.sleep(0.1)


def activate_install(selected, plan, backup=None):
    receipt = activation_path(selected, backup)
    attempted = False
    try:
        save_activation(receipt, selected, "pending")
        assert_restart_safe(selected, plan)
        # No interactive authentication, filesystem writes or waits occur between
        # the last idle proof and sending this restart request.
        attempted = True
        result = service_command([plan["sudo"], "-n", "--"] + plan["service_command"], timeout=45)
        if result.returncode:
            raise InstallError("systemctl restart failed (exit %s): %s" % (result.returncode, result.stderr.strip()))
        after = restarted_process(selected, plan)
        before = plan["identity"]
        save_activation(receipt, selected, "restarted", attempted=True,
                        previous_pid=before["pid"], pid=after["pid"], start_time=after["start_time"])
    except (InstallError, OSError, ValueError) as exc:
        try:
            save_activation(receipt, selected, "failed" if attempted else "blocked", attempted=attempted, error=str(exc))
        except (InstallError, OSError, ValueError) as receipt_error:
            print("Activation receipt could not be saved: %s" % receipt_error, file=sys.stderr)
        raise InstallError("Files are installed; automatic host-service activation %s. "
                           "Installed changes and backups were retained; no automatic rollback was attempted. "
                           "The service may have changed state. Backup: %s; activation receipt: %s. Cause: %s" %
                           ("failed" if attempted else "was blocked", backup or "existing installation", receipt, exc))
    print("Host service %s restarted; a new matching process was verified (PID %s)." % (selected["service"], after["pid"]))
    print("Klipper Ready/module behavior and physical homing/calibration have not been verified. Activation receipt: %s" % receipt)


def snapshot(path):
    path = Path(path)
    if path.is_symlink():
        return {"kind": "symlink", "target": os.readlink(path)}
    if path.exists():
        if not path.is_file():
            raise InstallError("Expected file/symlink, found directory/device: %s" % path)
        return {"kind": "file", "data": base64.b64encode(path.read_bytes()).decode("ascii"), "mode": stat.S_IMODE(path.stat().st_mode)}
    return {"kind": "absent"}


def state_matches(path, expected):
    return {k: v for k, v in snapshot(path).items() if k != "mode"} == {k: v for k, v in expected.items() if k != "mode"}


def write_state(path, state):
    path = Path(path)
    if state["kind"] == "absent":
        if path.exists() or path.is_symlink():
            path.unlink()
        return
    temporary = path.parent / ("." + path.name + ".accel-" + uuid.uuid4().hex)
    try:
        if state["kind"] == "symlink":
            os.symlink(state["target"], temporary)
        elif state["kind"] == "file":
            with open(temporary, "xb") as stream:
                os.chmod(temporary, state.get("mode", 0o600))
                stream.write(base64.b64decode(state["data"]))
                stream.flush()
                os.fsync(stream.fileno())
        else:
            raise InstallError("Invalid transaction state")
        os.replace(temporary, path)
    finally:
        if temporary.exists() or temporary.is_symlink():
            temporary.unlink()


def save_manifest(directory, manifest):
    data = json.dumps(manifest, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    write_state(directory / "manifest.json", {"kind": "file", "mode": 0o600, "data": base64.b64encode(data).decode("ascii")})


def apply_plan(changes, config, writer=write_state, selected=None, before_write=None, updater=None):
    if not changes:
        return None
    entries = []
    for path, change in changes.items():
        before, after = snapshot(path), dict(change)
        expected = after.pop("expected", before)
        if not state_matches(path, expected):
            raise InstallError("%s changed after planning. Retry after editor/SAVE_CONFIG finishes." % path)
        if not os.access(path.parent, os.W_OK):
            raise InstallError("No write access to %s. Run as Klipper service user; files are never written with sudo." % path.parent)
        if after["kind"] == "file":
            after["data"], after["mode"] = base64.b64encode(after["data"]).decode("ascii"), before.get("mode", 0o600)
        entries.append(dict(path=str(path), before=before, after=after))
    uninstaller = load_uninstaller()
    _, active, receipt = uninstaller.history(config, uninstall_api())
    context = {str(path): snapshot(path) for path in ConfigGraph(config).documents}
    metadata = dict(operation="install", repo=str(selected["repo"]) if selected else None,
                    parent=active[-1]["ref"] if active else None,
                    previous_uninstall=receipt["ref"] if receipt else None, context=context)
    if updater is not None:
        metadata["updater"] = updater["context"]
        metadata["source_update"] = load_updater().source_metadata(updater)
    return execute_transaction(entries, config, metadata, writer, before_write)


def execute_transaction(entries, config, metadata, writer=write_state, before_write=None):
    for entry in entries:
        path = Path(entry["path"])
        if not state_matches(path, entry["before"]):
            raise InstallError("%s changed after planning. No printer transaction files changed; a private plotting cache "
                               "may have been prepared. Retry after editor/SAVE_CONFIG finishes." % path)
        if not os.access(path.parent, os.W_OK):
            raise InstallError("No write access to %s. Run as Klipper service user; files are never written with sudo." % path.parent)
    if before_write:
        before_write()
    parent = config.parent / ".accel_homing_backups"
    if parent.is_symlink():
        raise InstallError("Refusing symlink backup directory: %s" % parent)
    parent.mkdir(mode=0o700, exist_ok=True)
    os.chmod(parent, 0o700)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    backup = parent / (stamp + "-" + uuid.uuid4().hex[:8])
    backup.mkdir(mode=0o700)
    manifest = dict(version=2, config=str(config), status="prepared", created_utc=stamp, changes=entries, **metadata)
    save_manifest(backup, manifest)
    applied = []
    try:
        for entry in entries:
            path = Path(entry["path"])
            if not state_matches(path, entry["before"]):
                raise InstallError("%s changed during installation. Retry after editor/SAVE_CONFIG finishes." % path)
            applied.append(entry)
            writer(path, entry["after"])
        manifest["status"] = "installed"
        save_manifest(backup, manifest)
    except Exception as exc:
        errors = []
        for entry in reversed(applied):
            try:
                if state_matches(Path(entry["path"]), entry["before"]):
                    continue
                if not state_matches(Path(entry["path"]), entry["after"]):
                    raise InstallError("changed concurrently; preserving current contents")
                write_state(Path(entry["path"]), entry["before"])
            except Exception as undo:
                errors.append("%s: %s" % (entry["path"], undo))
        manifest.update(status="rollback_incomplete" if errors else "rolled_back_after_failure", failure=str(exc), rollback_errors=errors)
        save_manifest(backup, manifest)
        raise InstallError("%s failed; %s. Backup %s. Cause: %s%s" %
                           ("Uninstall" if metadata.get("operation") == "uninstall" else "Installation",
                            "rollback needs repair" if errors else "applied changes restored", backup, exc, "; ".join(errors)))
    return backup


def make_uninstall_plan(selected):
    try:
        plan = load_uninstaller().make_plan(selected, uninstall_api())
        assert_uninstall_consumers(selected, plan)
        return plan
    except InstallError as error:
        raise InstallError("%s History/backups: %s" % (error, selected["config"].parent / ".accel_homing_backups"))


def assert_uninstall_consumers(selected, plan, processes=None, services=None):
    """A checkout's global extras links may serve more than one config."""
    extras = selected["repo"] / "klippy/extras"
    if not any(Path(entry["path"]).parent == extras for entry in plan["entries"]):
        return
    processes = proc_instances() if processes is None else processes
    services = service_instances(Path.home()) if services is None else services
    peers = {}
    for peer in processes + services:
        if peer["repo"] != selected["repo"]:
            continue
        same_instance = (peer["config"] == selected["config"]
                         and (not peer.get("service") or not selected.get("service") or peer["service"] == selected["service"])
                         and (not peer.get("pid") or not selected.get("pid") or peer["pid"] == selected["pid"]))
        if not same_instance:
            peers[(peer["config"], peer.get("service"))] = peer
    conflicts = []
    for peer in peers.values():
        try:
            graph = ConfigGraph(peer["config"])
            uses_plugin = any(name in ("accel_homing", "beacon_accel_homing")
                              or name.startswith("accel_homing_model ") for name in graph.sections)
            uses_plugin |= any(re.search(r"(?mi)^#\*#\s+\[accel_homing_model\s+", doc.text)
                               or re.search(r"(?mi)^\s*endstop_pin\s*[:=]\s*(?:accel_homing|beacon_accel):", doc.text)
                               for doc in graph.documents.values())
            if uses_plugin or peer["config"] == selected["config"]:
                conflicts.append(str(peer["config"]) + " (" + (peer.get("service") or peer["origin"]) + ")")
        except (InstallError, OSError, ValueError):
            conflicts.append(str(peer["config"]) + " (cannot resolve this shared checkout consumer)")
    if conflicts:
        raise InstallError("Uninstall would change shared checkout module links used by another instance: %s. "
                           "Give that instance a separate checkout or reconcile its plugin dependency first. No files changed." % "; ".join(conflicts))


def apply_uninstall_plan(plan, selected, writer=write_state):
    if plan["config"] != selected["config"] or plan["repo"] != selected["repo"]:
        raise InstallError("Uninstall plan belongs to a different instance. No files changed.")
    records, _, _ = load_uninstaller().history(selected["config"], uninstall_api())
    if [item["ref"] for item in records] != plan["records"]:
        raise InstallError("Installation history changed after uninstall planning. No files changed; retry.")
    for path, expected in plan.get("expected", {}).items():
        if not state_matches(Path(path), expected):
            raise InstallError("%s changed after planning. No files changed; retry after editor/SAVE_CONFIG finishes." % path)
    if not plan["entries"]:
        return None
    assert_uninstall_consumers(selected, plan)
    metadata = dict(operation="uninstall", repo=str(selected["repo"]), history=plan["history"])
    if plan.get("updater") is not None:
        metadata["updater"] = plan["updater"]
    return execute_transaction(plan["entries"], selected["config"], metadata, writer)


def rollback(backup, dry_run=False, selected=None):
    backup = canonical(backup)
    if backup.is_file():
        backup = backup.parent
    manifest = json.loads((backup / "manifest.json").read_text())
    if manifest.get("version") not in (1, 2) or manifest.get("status") != "installed":
        raise InstallError("Not an installed transaction: %s" % backup)
    if selected and canonical(manifest["config"]) != selected["config"]:
        raise InstallError("Backup belongs to a different config: %s" % manifest["config"])
    if selected and manifest.get("repo") and canonical(manifest["repo"]) != selected["repo"]:
        raise InstallError("Backup belongs to a different checkout: %s" % manifest["repo"])
    if manifest.get("updater") is not None:
        if selected is None:
            raise InstallError("Updater rollback requires the exact selected instance.")
        load_updater().validate_record(manifest, selected, uninstall_api())
    source_restore = load_updater().validate_source_rollback(manifest, selected, uninstall_api()) if manifest.get("source_update") else None
    for entry in manifest["changes"]:
        if not state_matches(Path(entry["path"]), entry["after"]):
            raise InstallError("%s changed after installation. Refusing to overwrite later edits; restore selected "
                               "before states from %s manually." % (entry["path"], backup / "manifest.json"))
    if dry_run:
        return
    done = []
    try:
        for entry in reversed(manifest["changes"]):
            if not state_matches(Path(entry["path"]), entry["after"]):
                raise InstallError("%s changed during rollback; preserving current contents" % entry["path"])
            write_state(Path(entry["path"]), entry["before"])
            done.append(entry)
        if source_restore:
            load_updater().restore_publication(source_restore, uninstall_api())
    except Exception as exc:
        errors = []
        for entry in reversed(done):
            try:
                if not state_matches(Path(entry["path"]), entry["before"]):
                    raise InstallError("changed concurrently; preserving current contents")
                write_state(Path(entry["path"]), entry["after"])
            except Exception as repair:
                errors.append("%s: %s" % (entry["path"], repair))
        intact = all(state_matches(Path(e["path"]), e["after"]) for e in manifest["changes"])
        manifest.update(status="installed" if intact else "rollback_incomplete", failure=str(exc), rollback_errors=errors)
        save_manifest(backup, manifest)
        raise InstallError("Rollback failed; concurrent changes were preserved. Inspect backup %s. Cause: %s%s" %
                           (backup, exc, "; ".join(errors)))
    manifest["status"] = "rolled_back"
    save_manifest(backup, manifest)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--klipper-path", default=os.environ.get("KLIPPER_PATH"), help="Klipper/Kalico/compatible checkout")
    parser.add_argument("--config", default=os.environ.get("PRINTER_CONFIG"), help="main printer.cfg")
    parser.add_argument("--service", default=os.environ.get("KLIPPER_SERVICE"), help="exact systemd instance")
    parser.add_argument("--klippy-socket", default=os.environ.get("KLIPPY_SOCKET"), help="read-only Klipper API socket (-a)")
    parser.add_argument("--klipper-python", help="exact selected Klipper venv interpreter when it cannot be detected; never bootstrap/global Python")
    parser.add_argument("--no-update-manager", action="store_true", help="install without managed git source/Moonraker registration (offline/development opt-out)")
    parser.add_argument("--moonraker-config", help="select the actual Moonraker config; must match this Klippy API socket")
    parser.add_argument("--accel-chip", help="full configured section, e.g. 'adxl345 toolhead'")
    parser.add_argument("--generic-names", action="store_true",
                        help="rename a legacy plugin section and both X/Y pin aliases to generic names; refuse legacy macro/object/command references")
    parser.add_argument("--axes", default="x,y", help="x,y (default), x or y")
    parser.add_argument("--homing-speed", type=float,
                        help="explicitly replace selected homing speeds; default preserves configured values and rejects policy conflicts")
    parser.add_argument("--max-report-travel", type=float,
                        help="opt into generic max_report_travel in mm (finite 0.1..2); otherwise preserve an explicit configured limit or leave disabled")
    parser.add_argument("--max-startup-travel", type=float,
                        help="opt into generic max_startup_travel in mm (finite 0.1..5); otherwise preserve an explicit configured limit or leave disabled")
    parser.add_argument("--acknowledge-homing-override", action="store_true",
                        help="acknowledge reviewed gcode_macro G28 behavior for installation only; "
                        "not needed for built-in homing_override/safe_z_home and does not enable workflows")
    parser.add_argument("--replace-existing-links", action="store_true", help="back up/replace plugin links from another source")
    parser.add_argument("--no-restart", action="store_true", help="install/update without automatic host-service restart; activate manually")
    parser.add_argument("--dry-run", action="store_true", help="validate/print plan, no writes")
    parser.add_argument("--rollback", metavar="BACKUP_DIRECTORY", help="restore one unchanged installation transaction")
    parser.add_argument("-u", "--uninstall", action="store_true", help="restore the state before the first active managed installation, including upgrades")
    args = parser.parse_args(argv)
    args.axes = [axis.strip().lower() for axis in args.axes.split(",")]
    if not args.axes or len(set(args.axes)) != len(args.axes) or any(axis not in ("x", "y") for axis in args.axes):
        parser.error("--axes must be x,y, x or y")
    if args.rollback and args.uninstall:
        parser.error("choose --rollback or --uninstall")
    if (args.rollback or args.uninstall) and (args.max_report_travel is not None or args.max_startup_travel is not None):
        parser.error("--max-report-travel and --max-startup-travel apply only to installation/update")
    if (args.rollback or args.uninstall) and args.generic_names:
        parser.error("--generic-names applies only to installation/update")
    return args


def print_restart_advice(selected):
    print("After reviewing the changes, while the printer is idle, restart its Klipper host "
          "process to load the changed Python modules.")
    service = selected.get("service")
    if service and selected.get("service_scope") == "user":
        print("Run this host command as the Klipper service user: systemctl --user restart -- %s" % shlex.quote(str(service)))
    elif service:
        print("Run this host command yourself: sudo systemctl restart -- %s" % shlex.quote(str(service)))
    else:
        print("Use this printer's actual host service manager to restart the Klipper process; "
              "the installer could not determine a service name.")
    print("G-code RESTART and FIRMWARE_RESTART alone do not reload already-imported Python modules. "
          "No restart, homing, firmware flashing, or calibration was performed.")


def load_plotting(source=None):
    path = (Path(source) if source is not None else Path(__file__).resolve().parent) / "accel_homing_charts.py"
    spec = importlib.util.spec_from_file_location("accel_homing_plotting_setup", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _lexical_python(value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise InstallError("Klipper Python must be an absolute venv executable, not %r. "
                           "A relative service command cannot establish its environment." % value)
    # Resolving bin/python's symlink would silently select the system prefix.
    return os.path.abspath(str(path))


def assert_python_instance(selected, executable):
    if selected.get("pid"):
        matches = [item for item in proc_instances() if item.get("pid") == selected["pid"]]
        if (len(matches) != 1 or any(matches[0].get(key) != selected.get(key)
                                     for key in ("repo", "config", "socket", "start_time"))
                or _lexical_python(matches[0]["executable"]) != executable):
            raise InstallError("Selected Klipper process/interpreter changed before printer transaction writes; "
                               "a private plotting cache may already have been prepared.")
    elif selected.get("service"):
        matches = [item for item in service_instances(Path.home())
                   if item.get("service") == selected["service"]
                   and item.get("repo") == selected["repo"] and item.get("config") == selected["config"]]
        if len(matches) != 1 or _lexical_python(matches[0]["executable"]) != executable:
            raise InstallError("Cannot reverify the selected stopped service's exact Python environment.")


def inspect_plotting_python(selected, explicit=None):
    detected = selected.get("executable")
    if explicit and detected and _lexical_python(explicit) != _lexical_python(detected):
        raise InstallError("--klipper-python does not match the selected instance's executable %s; "
                           "it cannot override another instance's environment." % detected)
    if not detected and not explicit:
        raise InstallError("Cannot identify this selected instance's Klipper Python. Supply "
                           "--klipper-python /absolute/path/to/its/venv/bin/python; bootstrap/system Python will not be used.")
    executable = _lexical_python(detected or explicit)
    assert_python_instance(selected, executable)
    script = ("import json,os,sys; p=sys.prefix; "
              "print(json.dumps(dict(executable=sys.executable,prefix=p,base_prefix=sys.base_prefix,"
              "cache_tag=sys.implementation.cache_tag,uid=os.getuid(),prefix_uid=os.stat(p).st_uid,"
              "writable=os.access(p,os.W_OK),venv_config=os.path.isfile(os.path.join(p,'pyvenv.cfg')))))")
    try:
        result = subprocess.run([executable, "-I", "-c", script], capture_output=True,
                                text=True, timeout=15)
        details = json.loads(result.stdout) if result.returncode == 0 else None
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        raise InstallError("Cannot interrogate selected Klipper Python %s: %s" % (executable, exc))
    uid = os.getuid() if hasattr(os, "getuid") else None
    if (not isinstance(details, dict) or details.get("uid") != uid or uid == 0
            or details.get("prefix_uid") != uid or details.get("writable") is not True
            or details.get("venv_config") is not True or not details.get("cache_tag")
            or details.get("prefix") == details.get("base_prefix")
            or _lexical_python(details.get("executable", "")) != executable):
        raise InstallError("Selected Python %s is not a verified writable venv owned by the Klipper service user. "
                           "No global/root pip fallback is permitted." % executable)
    details["command"] = executable
    return details


def prepare_plotting(selected, args, before_write=None, source=None):
    """Reuse Agg or atomically prepare private wheels, never mutate venv pins."""
    try:
        details = inspect_plotting_python(selected, getattr(args, "klipper_python", None))
    except InstallError as exc:
        if not args.dry_run:
            raise
        print("PNG dependency preflight unresolved (dry run): %s" % exc)
        return {"available": False, "reason": str(exc)}
    charts = load_plotting() if source is None else load_plotting(source)
    target = Path(charts.default_dependency_directory(details["prefix"], details["cache_tag"]))
    owned_root = (Path(source).resolve() if source is not None else Path(__file__).resolve().parent) / ".plot-deps"
    try:
        relative = target.resolve().relative_to(owned_root)
        if not relative.parts:
            raise ValueError("empty environment key")
    except ValueError:
        raise InstallError("Refusing a plotting dependency target outside this source release.")
    if before_write is not None:
        before_write()
    def verify(directory):
        renderer = charts.PNGRenderer(details["command"], directory)
        charts.preflight_renderer(renderer=renderer)
    try:
        verify(target)
        assert_python_instance(selected, details["command"])
        print("PNG Agg renderer verified with selected Klipper Python: %s" % details["command"])
        return {"available": True, "python": details["command"], "target": str(target)}
    except Exception as original:
        if args.dry_run:
            print("PNG renderer needs private binary matplotlib dependencies for %s (dry run; no installation): %s" %
                  (details["command"], original))
            return {"available": False, "python": details["command"], "target": str(target)}
        if target.exists():
            raise InstallError("Existing private plotting dependencies failed a real render: %s. "
                               "Preserve/review %s before replacing it; no printer files changed." % (original, target))
    assert_python_instance(selected, details["command"])
    if before_write is not None:
        before_write()
    target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    stage = Path(tempfile.mkdtemp(prefix=".prepare-", dir=str(target.parent)))
    try:
        command = [details["command"], "-I", "-m", "pip", "--isolated", "install",
                   "--disable-pip-version-check", "--no-warn-script-location", "--only-binary=:all:",
                   "--index-url", "https://pypi.org/simple", "--target", str(stage), "matplotlib>=3.3,<4"]
        print("Preparing private PNG plotting wheels with %s; existing environment packages are unchanged." % details["command"])
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=300)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise InstallError("Private plotting wheel installation failed: %s. No printer files changed." % exc)
        if result.returncode:
            raise InstallError("Private matplotlib binary-wheel installation failed (including missing pip/unsupported wheels): %s. "
                               "No printer files changed; no global pip or source-build fallback was attempted." %
                               result.stderr[-3000:].strip())
        verify(stage)
        assert_python_instance(selected, details["command"])
        if before_write is not None:
            before_write()
        if target.exists():
            verify(target)  # Another verified installer may have published first.
        else:
            os.rename(stage, target)
        print("Private PNG dependencies verified: %s. Removing this cache reverses only plotting setup; "
              "uninstall retains it with the source cache." % target)
        return {"available": True, "python": details["command"], "target": str(target)}
    except Exception as exc:
        if isinstance(exc, InstallError):
            raise
        raise InstallError("Private PNG dependency preparation failed before printer writes: %s" % exc)
    finally:
        if stage.exists():
            shutil.rmtree(stage)


def main(argv=None):
    args = arguments(argv)
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        raise InstallError("Run as the Klipper service user, not root. Only the detected host-service restart uses sudo.")
    selected = discover_target(args)
    print("Checkout: %s\nMain config: %s\nDetection: %s\nService: %s\nExecutable: %s" %
          (selected["repo"], selected["config"], selected["origin"], selected["service"] or "none", selected["executable"] or "stopped host"))
    updater_helper = load_updater()
    if updater_helper:
        # Known legacy ownership needs its old uninstaller, including when this
        # entry point was invoked with --uninstall, --rollback, or an opt-out.
        updater_helper.reject_legacy_installation(selected, uninstall_api())
    if args.uninstall:
        plan = make_uninstall_plan(selected)
        for entry in plan["entries"]:
            print("Restore %s (%s -> %s)" % (entry["path"], entry["before"]["kind"], entry["after"]["kind"]))
        if args.dry_run:
            print("Uninstall dry run: %d installation transaction(s); no writes, restart, or movement." % len(plan["history"]))
            return 0
        if not plan["entries"]:
            print("Already uninstalled; no changes, restart, or movement.")
            return 0
        print(assert_idle(selected))
        backup = apply_uninstall_plan(plan, selected)
        print("Uninstalled. Current-state backup: %s" % backup)
        print("Restored the pre-install activation/config state across %d installation transaction(s). "
              "Source caches, charts and original backups were retained." % len(plan["history"]))
        print("Review the restored config.")
        print_restart_advice(selected)
        if plan.get("updater") is not None:
            load_updater().print_activation_advice(plan["updater"], selected, uninstall_api())
        print("Undo this uninstall (only while its restored files remain unchanged): bash %s --config %s --klipper-path %s --rollback %s" %
              tuple(shlex.quote(str(v)) for v in (Path(__file__).resolve().parent / "install.sh", selected["config"], selected["repo"], backup)))
        return 0
    if args.rollback:
        rollback(args.rollback, True, selected)
        if not args.dry_run:
            print(assert_idle(selected))
            rollback(args.rollback, False, selected)
        print(("Would restore: " if args.dry_run else "Restored: ") + args.rollback)
        if not args.dry_run:
            print_restart_advice(selected)
            manifest = json.loads((canonical(args.rollback) / "manifest.json").read_text())
            if manifest.get("updater") is not None:
                load_updater().print_activation_advice(manifest["updater"], selected, uninstall_api())
        return 0
    source = Path(__file__).resolve().parent
    updater_receipt = selected["config"].parent / ".accel_homing_updater.json"
    if args.no_update_manager and (updater_receipt.exists() or updater_receipt.is_symlink()):
        raise InstallError("This instance has a managed Moonraker updater. Uninstall its managed registration "
                           "before switching to --no-update-manager; otherwise UI updates would target code "
                           "that Klipper no longer loads. No printer files changed.")
    updater_api = None if args.no_update_manager else load_updater()
    updater = None
    planning_args = updater_api.migration_args(selected, args, uninstall_api()) if updater_api else args
    # Validate printer/custom-G28/foreign-link policy before source/cache writes.
    changes, messages, sensor = make_plan(source, selected, planning_args)
    if updater_api:
        updater = updater_api.make_plan(source, selected, args, uninstall_api())
        assert_uninstall_consumers(selected, {"entries": [
            {"path": str(selected["repo"] / "klippy/extras" / name)} for name in MODULES]})
        for module in MODULES:
            link = selected["repo"] / "klippy/extras" / module
            changes[link] = dict(kind="symlink", target=str(updater["source"] / module),
                                 expected=getattr(planning_args, "_migration_expected", {}).get(
                                     link, changes.get(link, {}).get("expected", snapshot(link))))
        changes.update(updater["changes"])
        print("Managed update source: %s\nMoonraker config: %s" % (updater["repo"], updater["moon"]["config"]))
    print("Accelerometer: [%s] (physical mounting/orientation not inferred)" % sensor)
    for message in messages:
        print("Note: " + message)
    for path, change in changes.items():
        print(("Link " if change["kind"] == "symlink" else "Edit ") + str(path))
    if args.dry_run:
        prepare_plotting(selected, args)
        print("Dry run: no writes, restart, or movement.")
        return 0
    restart = None if args.no_restart else prepare_restart(selected)
    moon_restart = (updater_api.prepare_restart(updater["moon"], selected, uninstall_api())
                    if updater and updater["restart_needed"] and not args.no_restart else None)
    def before_write():
        assert_migration_state(planning_args)
        executable = selected.get("executable") or args.klipper_python
        if executable:
            assert_python_instance(selected, _lexical_python(executable))
        if restart:
            assert_restart_safe(selected, restart)
        else:
            print(assert_idle(selected))
        if updater:
            updater_api.assert_source_identity(updater, uninstall_api())
            if moon_restart:
                updater_api.assert_restart_safe(updater["moon"], selected, moon_restart, uninstall_api())
            else:
                updater_api.assert_identity(updater["moon"], selected, uninstall_api())
    candidate = updater_api.prepare_source(updater, uninstall_api(), before_write) if updater else source
    prepare_plotting(selected, args, before_write, source=candidate) if updater else prepare_plotting(selected, args, before_write)
    if updater:
        changes, _messages, _sensor = make_plan(candidate, selected, planning_args)
        for module in MODULES:
            path = selected["repo"] / "klippy/extras" / module
            desired = dict(kind="symlink", target=str(updater["source"] / module))
            desired["expected"] = getattr(planning_args, "_migration_expected", {}).get(
                path, changes.get(path, {}).get("expected", snapshot(path)))
            if state_matches(path, {key: value for key, value in desired.items() if key != "expected"}):
                changes.pop(path, None)
            else:
                changes[path] = desired
        changes.update(updater["changes"])
    backup = None
    try:
        if updater:
            updater_api.publish_source(updater, uninstall_api(), before_write)
        if changes:
            options = {"updater": updater} if updater else {}
            backup = apply_plan(changes, selected["config"], selected=selected, before_write=before_write, **options)
    except Exception as exc:
        if updater:
            try:
                updater_api.restore_publication(updater, uninstall_api())
            except Exception as recovery:
                raise InstallError("Installer file transaction failed and prior source restoration also failed. "
                                   "Retained source backup: %s. Original cause: %s. Recovery: %s" %
                                   (updater.get("source_backup"), exc, recovery))
        raise
    if changes:
        print("Installed. Backup: %s" % backup)
        print("Review the config and sensor mounting/orientation.")
        print("Restore: bash %s --config %s --klipper-path %s --rollback %s" %
              tuple(shlex.quote(str(v)) for v in (Path(__file__).resolve().parent / "install.sh", selected["config"], selected["repo"], backup)))
    else:
        print("Already installed; no installation files changed.")
    if updater and updater["restart_needed"]:
        if moon_restart:
            updater_api.activate(updater, selected, moon_restart, uninstall_api(), backup)
        else:
            updater_api.record_activation(updater, selected, uninstall_api(), backup, "skipped", reason="--no-restart")
            print("Moonraker updater entry is written but not loaded (--no-restart). Restart the matching "
                  "Moonraker service once from its UI/service manager; refreshing the browser alone does not load it.")
            updater_api.print_activation_advice(updater["context"], selected, uninstall_api())
    if restart:
        activate_install(selected, restart, backup)
    else:
        receipt = activation_path(selected, backup)
        save_activation(receipt, selected, "skipped", reason="--no-restart")
        print("Automatic activation skipped by --no-restart. Activation receipt: %s" % receipt)
        print_restart_advice(selected)
    print("Full uninstall (including upgrades): bash %s --config %s --klipper-path %s" %
          tuple(shlex.quote(str(v)) for v in (Path(__file__).resolve().parent / "uninstall.sh", selected["config"], selected["repo"])))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (InstallError, OSError, ValueError) as error:
        print("Installation aborted: %s" % error, file=sys.stderr)
        sys.exit(1)
