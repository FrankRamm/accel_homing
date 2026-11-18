"""Remove recognized current plugin state without reading installation archives.

Plans contain only in-memory current-state snapshots. Physical endstop
configuration must be supplied by the operator after removing virtual pins.
"""
import base64
from pathlib import Path
import re

PLUGIN = {"accel_homing", "beacon_accel_homing"}
MODEL = re.compile(r"^accel_homing_model\s+\S.*$", re.I)
PIN = re.compile(r"^(?:accel_homing|beacon_accel):[xy]$", re.I)
SAVED_SECTION = re.compile(r"^#\*#\s+\[([^]]+)\]\s*(?:[#;].*)?$")
CORE = ("accel_homing.py", "beacon_accel_homing.py", "accel_homing_backends.py",
        "accel_homing_workflows.py", "accel_homing_model.py", "accel_homing_charts.py")


def _fail(api, message):
    raise api.InstallError(message + " No files changed; reconcile the current configuration manually.")


def _plugin_section(name):
    return name.lower() in PLUGIN or bool(MODEL.fullmatch(name))


def _remove_section(document, section):
    # Preserve comments, including old installer markers, without interpreting
    # them as instructions to restore any previous value.
    for index in range(section.start, section.end):
        line = document.lines[index].strip()
        if index == section.start or (line and not line.startswith(("#", ";"))):
            document.replacements[index] = ""


def _remove_saved_models(document):
    removing = False
    for index in range(document.active_end, len(document.lines)):
        line = document.lines[index]
        match = SAVED_SECTION.match(line.rstrip("\r\n"))
        if match:
            removing = _plugin_section(match.group(1).strip())
        if removing and line.startswith("#*#"):
            document.replacements[index] = ""


def _module_links(selected, api):
    extras = selected["repo"] / "klippy/extras"
    expected, links, roots = {}, [], set()
    current_source = Path(api.__file__).resolve().parent
    for name in api.MODULES:
        path = extras / name
        state = api.snapshot(path)
        expected[str(path)] = state
        if state["kind"] == "absent":
            continue
        if state["kind"] != "symlink":
            _fail(api, "Refusing to remove non-symlink plugin file %s" % path)
        target = Path(state["target"])
        target = target if target.is_absolute() else path.parent / target
        if target.name != name or ".." in target.parts:
            _fail(api, "Unrecognized plugin link target for %s" % path)
        source = target.parent.resolve()
        if source != current_source and source.name not in ("accel_homing", "Beacon_Accel_Homing"):
            _fail(api, "Foreign plugin source for %s: %s" % (path, source))
        if not all((source / module).is_file() and not (source / module).is_symlink() for module in CORE):
            _fail(api, "Cannot verify plugin source %s; keep its links until manually reconciled" % source)
        if not target.is_file() or target.is_symlink():
            _fail(api, "Missing or indirect plugin target %s" % target)
        roots.add(source)
        links.append(path)
    if len(roots) > 1:
        _fail(api, "Plugin links point to multiple source checkouts")
    return links, expected


def make_plan(selected, api):
    """Return cleanup entries; caller checks idleness and checkout consumers."""
    config, repo = selected["config"], selected["repo"]
    graph = api.ConfigGraph(config)
    changes, expected, manual = {}, {}, []
    for path, document in graph.documents.items():
        state = api.snapshot(path)
        state["data"] = base64.b64encode(document.original).decode("ascii")
        expected[str(path)] = state
        for section in document.sections:
            if _plugin_section(section.name):
                graph.one(section.name)
                _remove_section(document, section)
            elif "endstop_pin" in section.options:
                pin = section.get("endstop_pin")
                if pin and pin.lower().startswith(("accel_homing:", "beacon_accel:")):
                    if not PIN.fullmatch(pin) or section.name.lower() not in ("stepper_x", "stepper_y"):
                        _fail(api, "Unsupported plugin endstop in [%s], %s" % (section.name, path))
                    graph.one(section.name)
                    document.replacements[section.options["endstop_pin"][0][0]] = ""
                    manual.append(str(path) + " [" + section.name + "] endstop_pin")
        _remove_saved_models(document)
        rendered = document.rendered()
        if rendered != document.original:
            changes[path] = dict(kind="file", data=rendered, expected=state)

    links, link_expected = _module_links(selected, api)
    expected.update(link_expected)
    for path in links:
        changes[path] = dict(kind="absent", expected=expected[str(path)])

    updater = api.load_updater()
    update_plan = updater.make_uninstall_plan(selected, api) if updater else {}
    for path, change in update_plan.get("changes", {}).items():
        path = Path(path)
        if path in changes:
            _fail(api, "Printer and Moonraker configuration overlap at %s" % path)
        changes[path] = change
    expected.update({str(path): state for path, state in update_plan.get("expected", {}).items()})

    entries = []
    for path, change in changes.items():
        before = change.get("expected", api.snapshot(path))
        after = {key: value for key, value in change.items() if key != "expected"}
        if after["kind"] == "file":
            after["data"] = base64.b64encode(after["data"]).decode("ascii")
            after["mode"] = before.get("mode", 0o600)
        expected.setdefault(str(path), before)
        if before != after:
            entries.append(dict(path=str(path), before=before, after=after))
    return dict(entries=entries, expected=expected, config=config, repo=repo,
                updater=update_plan.get("context"), manual_config=manual)
