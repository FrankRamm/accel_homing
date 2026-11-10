#!/usr/bin/env python3
"""Installer for the Creality K1 family (Buildroot/BusyBox, runs as root).

The standard installer.py targets a Linux host with systemd, sudo and a
non-root Klipper service user. The K1 differs in all three points: everything
runs as root from /usr/data, services are started by /etc/init.d scripts, and
there is normally no sudo, systemctl, git or bash. This script performs the
same essential steps for that environment:

* link (or copy) the plugin modules into klippy/extras,
* switch the X/Y endstops to accel_homing:x / accel_homing:y,
* add an [include accel_homing.cfg] to the main printer.cfg,
* restart Klipper through its init script.

Every config line it changes is tagged so --uninstall can reverse the edit
without discarding anything saved later by SAVE_CONFIG. Original files are also
copied to .accel_homing_backups/ beside printer.cfg first.

Only the Python standard library is used; Python 3.7+ is sufficient.
"""
import argparse
import datetime
import glob
import json
import os
import re
import shutil
import subprocess
import sys

MODULES = ("accel_homing.py", "beacon_accel_homing.py", "accel_homing_backends.py",
           "accel_homing_workflows.py", "accel_homing_model.py", "accel_homing_charts.py",
           "accel_homing_troubleshoot.py")
SOURCE = os.path.dirname(os.path.realpath(__file__))
STATE_FILE = os.path.join(SOURCE, ".k1_install_state.json")
MANAGED_CFG = "accel_homing.cfg"
TAG = "# accel_homing_k1"
ORIG = "#k1ah# "
SAVE_MARKER = "#*# <---------------------- SAVE_CONFIG ---------------------->"
SECTION = re.compile(r"^\[([^\]]+)\]")
INCLUDE = re.compile(r"^\[include\s+([^\]]+)\]")
OPTION = re.compile(r"^([A-Za-z_][\w.]*)\s*[:=]\s*(.*?)\s*$")
TMC = re.compile(r"^tmc\d+\s+stepper_([xy])$")
KLIPPER_CANDIDATES = ("/usr/data/klipper", "/usr/data/kalico", "/usr/share/klipper")
CONFIG_CANDIDATES = ("/usr/data/printer_data/config/printer.cfg",)
INIT_CANDIDATES = ("/etc/init.d/S55klipper_service", "/opt/etc/init.d/S55klipper_service")


class K1Error(Exception):
    pass


# ---------------------------------------------------------------- detection

def running_klippy():
    """Return info about running klippy.py processes (best effort, via /proc)."""
    found = []
    for path in glob.glob("/proc/[0-9]*/cmdline"):
        pid_dir = os.path.dirname(path)
        try:
            with open(path, "rb") as handle:
                argv = [part.decode("utf-8", "replace") for part in handle.read().split(b"\0") if part]
            cwd = os.readlink(os.path.join(pid_dir, "cwd"))
        except OSError:
            continue
        for index, value in enumerate(argv):
            if os.path.basename(value) != "klippy.py":
                continue
            script = value if os.path.isabs(value) else os.path.join(cwd, value)
            config = None
            if index + 1 < len(argv) and not argv[index + 1].startswith("-"):
                config = argv[index + 1]
                config = config if os.path.isabs(config) else os.path.join(cwd, config)
            python = argv[0] if index > 0 else None
            if python and not os.path.isabs(python):
                python = shutil.which(python)
            found.append({"script": os.path.realpath(script), "config": config, "python": python})
            break
    return found


def pick_klipper(arg):
    if arg:
        candidates = [os.path.realpath(arg)]
    else:
        candidates = [os.path.dirname(os.path.dirname(item["script"])) for item in running_klippy()]
        candidates += [path for path in KLIPPER_CANDIDATES]
    for path in candidates:
        if os.path.isfile(os.path.join(path, "klippy", "klippy.py")) and \
                os.path.isdir(os.path.join(path, "klippy", "extras")):
            return path
    raise K1Error("Klipper checkout not found. Pass --klipper-path /path/to/klipper "
                  "(the directory that contains klippy/klippy.py).")


def pick_config(arg):
    if arg:
        candidates = [arg]
    else:
        candidates = [item["config"] for item in running_klippy() if item["config"]]
        candidates += list(CONFIG_CANDIDATES)
    for path in candidates:
        if path and os.path.isfile(path):
            return os.path.realpath(path)
    raise K1Error("printer.cfg not found. Pass --config /path/to/printer.cfg.")


def pick_init(arg):
    if arg:
        return arg if os.path.isfile(arg) else None
    for path in INIT_CANDIDATES:
        if os.path.isfile(path):
            return path
    matches = sorted(glob.glob("/etc/init.d/S*klipper*"))
    return matches[0] if len(matches) == 1 else None


def klippy_python():
    for item in running_klippy():
        if item["python"]:
            return item["python"]
    return None


# ---------------------------------------------------------------- config files

class ConfigFile:
    def __init__(self, path):
        self.path = path
        with open(path, "r", encoding="utf-8", errors="surrogateescape", newline="") as handle:
            text = handle.read()
        parts = text.split("\n")
        self.lines = [part + "\n" for part in parts[:-1]] + ([parts[-1]] if parts[-1] else [])
        self.eol = "\r\n" if self.lines and self.lines[0].endswith("\r\n") else "\n"
        self.dirty = False

    def end(self):
        for index, line in enumerate(self.lines):
            if line.startswith(SAVE_MARKER):
                return index
        return len(self.lines)

    def sections(self):
        end, result, current = self.end(), [], None
        for index in range(end):
            match = SECTION.match(self.lines[index])
            if match:
                if current:
                    current[2] = index
                    result.append(tuple(current))
                current = [match.group(1).strip(), index, end]
        if current:
            result.append(tuple(current))
        return result

    def options(self, start, stop):
        found = []
        for index in range(start + 1, stop):
            match = OPTION.match(self.lines[index].rstrip("\r\n"))
            if match:
                found.append((index, match.group(1), match.group(2).split("#")[0].strip()))
        return found

    def newline(self, text):
        return text + self.eol

    def ensure_eol(self, index):
        if not self.lines[index].endswith("\n"):
            self.lines[index] += self.eol

    def write(self, backup_dir, root):
        if not self.dirty:
            return
        relative = os.path.relpath(self.path, root)
        relative = os.path.basename(self.path) if relative.startswith("..") else relative
        target = os.path.join(backup_dir, relative)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        shutil.copy2(self.path, target)
        temporary = self.path + ".k1tmp"
        with open(temporary, "w", encoding="utf-8", errors="surrogateescape", newline="") as handle:
            handle.write("".join(self.lines))
        shutil.copymode(self.path, temporary)
        os.replace(temporary, self.path)


def load_tree(main):
    """Main config plus every readable [include] file, in order, without cycles."""
    files, seen = [], set()

    def visit(path, depth):
        real = os.path.realpath(path)
        if real in seen or depth > 6 or not os.path.isfile(real):
            return
        seen.add(real)
        cf = ConfigFile(real)
        files.append(cf)
        for index in range(cf.end()):
            match = INCLUDE.match(cf.lines[index])
            if match:
                pattern = match.group(1).split("#")[0].strip()
                pattern = pattern if os.path.isabs(pattern) else os.path.join(os.path.dirname(real), pattern)
                for item in sorted(glob.glob(pattern)):
                    visit(item, depth + 1)

    visit(main, 0)
    return files


def find_sections(files, predicate):
    return [(cf, name, start, stop) for cf in files for name, start, stop in cf.sections()
            if predicate(name)]


def replace_option(cf, section_name, name, value, check=None):
    """Make `name: value` active in section_name; keep the former line as a comment."""
    section = next((s for s in cf.sections() if s[0] == section_name), None)
    start, stop = section[1], section[2]
    matches = [item for item in cf.options(start, stop) if item[1] == name]
    if len(matches) > 1:
        raise K1Error("Duplicate option %s in [%s] (%s)." % (name, section_name, cf.path))
    if matches:
        index, _, current = matches[0]
        if (check(current) if check else current == value):
            return False
        cf.ensure_eol(index)
        cf.lines[index] = ORIG + cf.lines[index]
        cf.lines.insert(index + 1, cf.newline("%s: %s  %s" % (name, value, TAG)))
    else:
        cf.ensure_eol(start)
        cf.lines.insert(start + 1, cf.newline("%s: %s  %s" % (name, value, TAG)))
    cf.dirty = True
    return True


def comment_options(cf, section_name, names):
    section = next((s for s in cf.sections() if s[0] == section_name), None)
    changed = []
    for index, name, _ in cf.options(section[1], section[2]):
        if name in names:
            cf.lines[index] = ORIG + cf.lines[index]
            cf.dirty = True
            changed.append(name)
    return changed


def is_zero(value):
    try:
        return float(value) == 0.0
    except ValueError:
        return False


def configure(files, main_cf, axes, accel_chip, notes, warnings):
    names = {name.lower() for cf in files for name, _, _ in cf.sections()}
    for risky in ("homing_override", "safe_z_home", "gcode_macro g28"):
        if risky in names:
            warnings.append("[%s] is configured. Review it: it can interfere with the X/Y homing sequence "
                            "(see docs/COMPATIBILITY.md)." % risky)
    for axis in axes:
        found = find_sections(files, lambda name, axis=axis: name == "stepper_" + axis)
        if len(found) != 1:
            raise K1Error("Expected exactly one [stepper_%s] section in printer.cfg and its includes, found %d."
                          % (axis, len(found)))
        cf, name, _, _ = found[0]
        wanted = "accel_homing:" + axis
        if replace_option(cf, name, "endstop_pin", wanted):
            notes.append("[%s] endstop_pin -> %s" % (name, wanted))
        if replace_option(cf, name, "homing_retract_dist", "0", check=is_zero):
            notes.append("[%s] homing_retract_dist -> 0" % name)
        for option in comment_options(cf, name, {"min_home_dist", "homing_accel"}):
            notes.append("[%s] commented %s" % (name, option))
        for tmc_cf, tmc_name, _, _ in find_sections(
                files, lambda section, axis=axis: (TMC.match(section) and TMC.match(section).group(1) == axis)):
            for option in comment_options(tmc_cf, tmc_name, {"diag_pin", "diag0_pin", "diag1_pin", "home_current"}):
                notes.append("[%s] commented %s" % (tmc_name, option))
    existing = any(name == "accel_homing" or name.startswith("beacon_accel_homing")
                   for cf in files for name, _, _ in cf.sections())
    included = any(INCLUDE.match(line) and MANAGED_CFG in line for cf in files for line in cf.lines[:cf.end()])
    created = None
    if not existing and not included:
        created = os.path.join(os.path.dirname(main_cf.path), MANAGED_CFG)
        sections = main_cf.sections()
        position = sections[0][1] if sections else main_cf.end()
        main_cf.lines[position:position] = [main_cf.newline(TAG), main_cf.newline("[include %s]" % MANAGED_CFG)]
        main_cf.dirty = True
        notes.append("added [include %s] to %s" % (MANAGED_CFG, os.path.basename(main_cf.path)))
        if os.path.exists(created):
            warnings.append("%s already exists and was left untouched; make sure it contains [accel_homing]."
                            % created)
            created = None
    else:
        notes.append("an [accel_homing] section/include already exists; no new include added")
    return created, accel_chip


def managed_cfg_text(accel_chip):
    return "\n".join([
        TAG + " file: created by install_k1.sh and removed by uninstall_k1.sh unless you edit it.",
        "# All options: %s" % os.path.join(SOURCE, "examples", "accel_homing.cfg"),
        "[accel_homing]",
        "accel_chip: %s" % accel_chip,
        "#x_threshold: 8.0",
        "#y_threshold: 8.0",
        ""])


# ---------------------------------------------------------------- uninstall edits

def revert_lines(lines, keep_model, is_main):
    out, index, changed, removed_model = [], 0, False, False
    while index < len(lines):
        line = lines[index]
        if line.strip() == TAG:
            index += 1
            if index < len(lines) and INCLUDE.match(lines[index]) and MANAGED_CFG in lines[index]:
                index += 1
            changed = True
            continue
        if line.rstrip().endswith(TAG):
            index += 1
            changed = True
            continue
        if line.startswith(ORIG):
            out.append(line[len(ORIG):])
            index += 1
            changed = True
            continue
        if is_main and not keep_model and line.startswith("#*# [accel_homing_model"):
            index += 1
            while index < len(lines) and not lines[index].startswith("#*# ["):
                index += 1
            changed = removed_model = True
            continue
        out.append(line)
        index += 1
    return out, changed, removed_model


# ---------------------------------------------------------------- modules

def plan_links(repo, copy, force):
    extras = os.path.join(repo, "klippy", "extras")
    actions = []
    for module in MODULES:
        source, target = os.path.join(SOURCE, module), os.path.join(extras, module)
        if not os.path.isfile(source):
            raise K1Error("Missing module in checkout: %s" % source)
        if os.path.islink(target):
            if os.path.realpath(target) == os.path.realpath(source) and not copy:
                continue
            if not force:
                raise K1Error("%s is a link to another file. Use --force to replace it." % target)
            actions.append(("replace", source, target))
        elif os.path.exists(target):
            if copy and open(target, "rb").read() == open(source, "rb").read():
                continue
            if not force:
                raise K1Error("%s already exists as a regular file. Use --force to back it up and replace it." % target)
            actions.append(("replace", source, target))
        else:
            actions.append(("new", source, target))
    return actions


def apply_links(actions, copy):
    for kind, source, target in actions:
        if kind == "replace":
            backup = target + ".k1bak"
            if os.path.lexists(backup):
                raise K1Error("%s already exists; move it away first." % backup)
            os.rename(target, backup)
        if copy:
            shutil.copy2(source, target)
        else:
            try:
                os.symlink(source, target)
            except OSError as error:
                raise K1Error("Cannot create symlink %s (%s). Re-run with --copy." % (target, error))


def check_klipper_apis(repo, warnings):
    def read(relative):
        try:
            with open(os.path.join(repo, relative), "r", encoding="utf-8", errors="replace") as handle:
                return handle.read()
        except OSError:
            return ""
    if "TriggerDispatch" not in read("klippy/mcu.py"):
        warnings.append("This Klipper has no MCU TriggerDispatch. The plugin requires it "
                        "(docs/COMPATIBILITY.md) and will probably refuse to load.")
    adxl = read("klippy/extras/adxl345.py")
    if adxl and "batch_bulk" not in adxl and "api_dump" not in adxl:
        warnings.append("adxl345.py offers neither batch_bulk nor api_dump; no supported data path found.")


def check_plotting(warnings):
    python = klippy_python()
    if not python:
        warnings.append("Klippy is not running, so its Python could not be checked for matplotlib.")
        return
    result = subprocess.run([python, "-c", "import matplotlib, numpy"], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    if result.returncode:
        warnings.append("matplotlib/numpy are not importable by Klippy's Python (%s). Ordinary G28 homing "
                        "works, but CALIBRATE, CHARTS, ACCURACY, NOISE and TROUBLESHOOT stop at their PNG "
                        "preflight. Set x_threshold/y_threshold manually (see docs/K1.md)." % python)


# ---------------------------------------------------------------- main

def confirm_restart(args):
    if args.yes:
        return True
    if sys.stdin.isatty():
        reply = input("Restart Klipper now? Any running print is aborted. The printer must be idle. [y/N] ")
        return reply.strip().lower() in ("y", "yes")
    print("Not a terminal and no --yes: Klipper was not restarted.")
    return False


def do_restart(init, args):
    if args.no_restart or args.dry_run:
        print("Restart skipped. Restart Klipper while the printer is idle: %s restart"
              % (init or "/etc/init.d/S55klipper_service"))
        return
    if not init:
        print("No Klipper init script found; restart Klipper manually.")
        return
    if not confirm_restart(args):
        print("Restart skipped. Run when idle: %s restart" % init)
        return
    result = subprocess.run([init, "restart"])
    print("Klipper restart %s." % ("requested" if result.returncode == 0 else "FAILED (exit %d)" % result.returncode))


def backup_dir_for(main):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    return os.path.join(os.path.dirname(main), ".accel_homing_backups", "k1-" + stamp)


def save_state(state):
    try:
        with open(STATE_FILE, "w") as handle:
            json.dump(state, handle)
    except OSError:
        pass


def load_state():
    try:
        with open(STATE_FILE) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def install(args):
    repo, main = pick_klipper(args.klipper_path), pick_config(args.config)
    init = pick_init(args.init_script)
    axes = [a.strip().lower() for a in args.axes.split(",") if a.strip()]
    if not axes or any(a not in ("x", "y") for a in axes):
        raise K1Error("--axes must be x, y or x,y")
    warnings, notes = [], []
    print("Klipper:  %s\nconfig:   %s\ninit:     %s\nsource:   %s" % (repo, main, init or "(not found)", SOURCE))
    if sys.version_info < (3, 7):
        raise K1Error("Python 3.7 or newer is required to run this installer.")
    if not SOURCE.startswith("/usr/data") and os.path.exists("/usr/data"):
        warnings.append("This checkout is outside /usr/data. Links into it break if it is moved or if the "
                        "volatile root file system is reset; keep it under /usr/data/accel_homing.")
    check_klipper_apis(repo, warnings)
    check_plotting(warnings)
    actions = plan_links(repo, args.copy, args.force)
    files = load_tree(main)
    main_cf = files[0]
    created, chip = configure(files, main_cf, axes, args.accel_chip, notes, warnings)
    for action in actions:
        notes.append("%s %s" % ("copy" if args.copy else "link", action[2]))
    print("\nPlanned changes:")
    for note in notes or ["(nothing to change)"]:
        print("  - " + note)
    for warning in warnings:
        print("WARNING: " + warning)
    if args.dry_run:
        print("\nDry run: no files were written.")
        return 0
    backups = backup_dir_for(main)
    apply_links(actions, args.copy)
    root = os.path.dirname(main)
    for cf in files:
        cf.write(backups, root)
    if created:
        with open(created, "w") as handle:
            handle.write(managed_cfg_text(chip))
    save_state({"klipper": repo, "config": main, "init": init, "copy": args.copy, "created": created})
    print("\nInstalled. Originals saved under %s" % backups)
    do_restart(init, args)
    print("\nNext: read docs/K1.md. Do NOT home the printer yet; first set thresholds as described there.")
    return 0


def uninstall(args):
    state = load_state()
    repo = pick_klipper(args.klipper_path or state.get("klipper"))
    main = pick_config(args.config or state.get("config"))
    init = pick_init(args.init_script or state.get("init"))
    files = load_tree(main)
    notes = []
    backups = backup_dir_for(main) + "-uninstall"
    root = os.path.dirname(main)
    managed = os.path.join(root, MANAGED_CFG)
    for cf in files:
        if os.path.realpath(cf.path) == os.path.realpath(managed):
            continue
        lines, changed, model = revert_lines(cf.lines, args.keep_model, cf is files[0])
        if changed:
            cf.lines, cf.dirty = lines, True
            notes.append("reverted %s" % cf.path)
        if model:
            notes.append("removed saved [accel_homing_model ...] from %s (needed: Klipper rejects unknown sections)"
                         % cf.path)
    extras = os.path.join(repo, "klippy", "extras")
    for module in MODULES:
        target, source = os.path.join(extras, module), os.path.join(SOURCE, module)
        if os.path.islink(target) and os.path.realpath(target) == os.path.realpath(source):
            notes.append("remove link %s" % target)
        elif os.path.isfile(target) and os.path.isfile(source) and \
                open(target, "rb").read() == open(source, "rb").read():
            notes.append("remove copy %s" % target)
    if os.path.isfile(managed):
        with open(managed, "r", errors="replace") as handle:
            if handle.readline().startswith(TAG):
                notes.append("remove %s (backed up first)" % managed)
    print("Planned changes:")
    for note in notes or ["(nothing to undo)"]:
        print("  - " + note)
    if args.dry_run:
        print("Dry run: no files were written.")
        return 0
    for cf in files:
        cf.write(backups, root)
    for module in MODULES:
        target, source = os.path.join(extras, module), os.path.join(SOURCE, module)
        if (os.path.islink(target) and os.path.realpath(target) == os.path.realpath(source)) or \
                (os.path.isfile(target) and not os.path.islink(target) and os.path.isfile(source) and
                 open(target, "rb").read() == open(source, "rb").read()):
            os.remove(target)
            if os.path.exists(target + ".k1bak"):
                os.rename(target + ".k1bak", target)
    if os.path.isfile(managed):
        with open(managed, "r", errors="replace") as handle:
            ours = handle.readline().startswith(TAG)
        if ours:
            os.makedirs(backups, exist_ok=True)
            shutil.copy2(managed, os.path.join(backups, MANAGED_CFG))
            os.remove(managed)
    try:
        os.remove(STATE_FILE)
    except OSError:
        pass
    print("Uninstalled. Backups: %s" % backups)
    do_restart(init, args)
    return 0


def parser():
    p = argparse.ArgumentParser(description="Install/uninstall accel_homing on a Creality K1 (root, init.d).")
    p.add_argument("--uninstall", action="store_true")
    p.add_argument("--klipper-path", help="directory containing klippy/klippy.py (auto-detected)")
    p.add_argument("--config", help="main printer.cfg (auto-detected)")
    p.add_argument("--init-script", help="Klipper init script, e.g. /etc/init.d/S55klipper_service")
    p.add_argument("--accel-chip", default="auto", help="e.g. 'adxl345' (default: auto)")
    p.add_argument("--axes", default="x,y", help="x, y or x,y (default)")
    p.add_argument("--copy", action="store_true", help="copy modules instead of symlinking")
    p.add_argument("--force", action="store_true", help="back up and replace conflicting module files")
    p.add_argument("--keep-model", action="store_true", help="uninstall: keep saved calibration models")
    p.add_argument("--no-restart", action="store_true")
    p.add_argument("-y", "--yes", action="store_true", help="restart without asking")
    p.add_argument("--dry-run", action="store_true")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        return uninstall(args) if args.uninstall else install(args)
    except (K1Error, OSError) as error:
        print("Aborted: %s" % error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
