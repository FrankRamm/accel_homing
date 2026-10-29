"""Offline restore planning. The caller supplies the installer state/IO helpers.

No mutation belongs here: even SAVE_CONFIG cleanup is part of the verified plan.
"""
import base64
import difflib
import glob
import hashlib
import json
import math
import os
from pathlib import Path
import re


MODEL = re.compile(r"^accel_homing_model\s+\S", re.I)


def _fail(api, message, backup=None):
    suffix = (" Inspect %s/manifest.json; all recorded before states and any current-state "
              "recovery backup are retained. No files changed." % backup) if backup else " No files changed."
    raise api.InstallError(message + suffix)


def _path(value, api):
    if not isinstance(value, str) or not value or "\x00" in value:
        _fail(api, "Invalid transaction path")
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts or path.parent.resolve() != path.parent:
        _fail(api, "Suspicious transaction path: %s" % value)
    return path


def _state(value, api):
    if not isinstance(value, dict):
        _fail(api, "Invalid transaction state")
    kind = value.get("kind")
    fields = {"absent": {"kind"}, "symlink": {"kind", "target"},
              "file": {"kind", "data", "mode"}}.get(kind)
    if fields is None or set(value) != fields:
        _fail(api, "Unknown transaction state/fields")
    if kind == "file":
        try:
            base64.b64decode(value["data"], validate=True)
        except (TypeError, ValueError):
            _fail(api, "Invalid base64 transaction contents")
        if not isinstance(value["mode"], int) or not 0 <= value["mode"] <= 0o7777:
            _fail(api, "Invalid transaction file permissions")
    elif kind == "symlink":
        target = value["target"]
        if not isinstance(target, str) or not target or "\x00" in target or "\n" in target:
            _fail(api, "Invalid transaction symlink")
    return value


def _same(one, two):
    return {k: v for k, v in one.items() if k != "mode"} == {k: v for k, v in two.items() if k != "mode"}


def _bytes(state):
    return base64.b64decode(state["data"], validate=True)


def _file(data, mode=0o600):
    return {"kind": "file", "data": base64.b64encode(data).decode("ascii"), "mode": mode}


def history(config, api):
    """Read complete history without trusting manifest paths or following links."""
    parent = config.parent / ".accel_homing_backups"
    if parent.is_symlink() or (parent.exists() and not parent.is_dir()):
        _fail(api, "Suspicious backup directory: %s" % parent)
    records = []
    if not parent.exists():
        return records, [], None
    for directory in sorted(parent.iterdir()):
        if directory.is_symlink() or not directory.is_dir():
            _fail(api, "Unknown item in installer history: %s" % directory)
        path = directory / "manifest.json"
        if path.is_symlink() or not path.is_file():
            _fail(api, "Missing/unsafe installer manifest: %s" % path)
        try:
            raw = path.read_bytes()
            manifest = json.loads(raw)
        except (OSError, ValueError) as exc:
            _fail(api, "Unreadable installer manifest %s: %s" % (path, exc))
        if not isinstance(manifest, dict) or manifest.get("version") not in (1, 2):
            _fail(api, "Unknown installer history format", directory)
        recorded_config = _path(manifest.get("config"), api)
        if recorded_config != config:
            # Other instances can share a config directory. They never supply
            # ownership for this instance, even if a checkout happens to overlap.
            continue
        status = manifest.get("status")
        if status not in ("installed", "rolled_back", "rolled_back_after_failure"):
            _fail(api, "Incomplete/unknown transaction (%s)" % status, directory)
        operation = manifest.get("operation", "install")
        if operation not in ("install", "uninstall") or (manifest["version"] == 1 and operation != "install"):
            _fail(api, "Unknown installer operation", directory)
        entries = manifest.get("changes")
        if not isinstance(entries, list) or not entries:
            _fail(api, "Empty/invalid installer transaction", directory)
        seen = set()
        for entry in entries:
            if not isinstance(entry, dict) or set(entry) != {"path", "before", "after"}:
                _fail(api, "Invalid transaction entry", directory)
            entry_path = _path(entry["path"], api)
            if entry_path in seen:
                _fail(api, "Duplicate transaction destination: %s" % entry_path, directory)
            seen.add(entry_path)
            _state(entry["before"], api)
            _state(entry["after"], api)
        records.append(dict(directory=directory, manifest=manifest,
                            ref={"transaction": directory.name, "sha256": hashlib.sha256(raw).hexdigest()}))
    active, receipt = [], None
    for record in records:
        data = record["manifest"]
        if data["status"] != "installed":
            continue
        if data.get("operation", "install") == "uninstall":
            if data.get("history") != [item["ref"] for item in active]:
                _fail(api, "Missing/gapped/changed history referenced by uninstall", record["directory"])
            active, receipt = [], record
        else:
            if data["version"] == 2:
                expected_parent = active[-1]["ref"] if active else None
                expected_receipt = receipt["ref"] if receipt else None
                if data.get("parent") != expected_parent or data.get("previous_uninstall") != expected_receipt:
                    _fail(api, "Missing/gapped/changed installation history", record["directory"])
            active.append(record)
    return records, active, receipt


def _split_saved(data, api):
    text = data.decode("utf-8")
    lines = text.splitlines(keepends=True)
    indices = [i for i, line in enumerate(lines) if line.startswith(api.SAVE_MARKER)]
    if len(indices) > 1 or (not indices and any(line.startswith("#*# DO NOT EDIT") for line in lines)):
        _fail(api, "Ambiguous SAVE_CONFIG block")
    index = indices[0] if indices else len(lines)
    return "".join(lines[:index]), "".join(lines[index:])


def _saved_sections(saved, api):
    lines = saved.splitlines(keepends=True)
    heads = []
    for i, line in enumerate(lines):
        if not line.startswith("#*#"):
            _fail(api, "Unrecognized data below SAVE_CONFIG; restore it manually after reviewing the backup")
        match = re.match(r"^#\*#\s+\[([^]]+)\]\s*(?:\r?\n)?$", line)
        if match:
            heads.append((i, match.group(1).strip()))
    result, seen = [], set()
    for index, (start, name) in enumerate(heads):
        if name.lower() in seen:
            _fail(api, "Duplicate SAVE_CONFIG section: %s" % name)
        seen.add(name.lower())
        end = heads[index + 1][0] if index + 1 < len(heads) else len(lines)
        result.append((name, "".join(lines[start:end])))
    preamble = "".join(lines[:heads[0][0]]) if heads else saved
    return preamble, result


def _restore_models(baseline, current, api):
    """Restore old plugin models, remove new ones; other saved bytes are opaque."""
    _, old = _saved_sections(baseline, api)
    preamble, now = _saved_sections(current, api)
    originals = {name.lower(): body for name, body in old if MODEL.match(name)}
    output, used = [], set()
    for name, body in now:
        if MODEL.match(name):
            if name.lower() in originals:
                output.append(originals[name.lower()])
                used.add(name.lower())
        else:
            output.append(body)
    output.extend(body for name, body in originals.items() if name not in used)
    if not output and not baseline:
        return ""
    # Preserve the current header when other saved data exists. An old saved
    # model can also be recovered if somebody removed the whole saved block.
    return (preamble or _saved_sections(baseline, api)[0]) + "".join(output)


def _section_names(lines, api):
    names, current = [], None
    for line in lines:
        match = api.SECTION.match(line)
        if match:
            current = match.group(1).strip().lower()
        names.append(current)
    return names


def _undo_text(installed, original, current, path, api):
    """Apply only inverse installer hunks through unchanged, unique anchors."""
    if current == installed:
        return original
    base, wanted, live = (value.splitlines(keepends=True) for value in (installed, original, current))
    base_keys, wanted_keys, live_keys = ([line.rstrip("\r\n") for line in lines] for lines in (base, wanted, live))
    matches = difflib.SequenceMatcher(None, base_keys, live_keys, autojunk=False).get_matching_blocks()
    sections_base, sections_live = _section_names(base, api), _section_names(live, api)
    edits = []
    for tag, first, last, old_first, old_last in difflib.SequenceMatcher(None, base_keys, wanted_keys, autojunk=False).get_opcodes():
        if tag == "equal":
            continue
        if first == last:
            _fail(api, "Cannot safely anchor insertion while restoring %s" % path)
        candidates = [block.b + first - block.a for block in matches
                      if block.a <= first and last <= block.a + block.size]
        if len(candidates) != 1:
            _fail(api, "Managed lines changed in %s; refusing to overwrite later edits" % path)
        start = candidates[0]
        if sections_base[first:last] != sections_live[start:start + last - first]:
            _fail(api, "Managed section changed in %s" % path)
        # Identical managed text in another section cannot be a replacement for
        # the intended hunk after its original was edited/deleted.
        occurrences = [i for i in range(len(live) - (last - first) + 1)
                       if live_keys[i:i + last - first] == base_keys[first:last]
                       and sections_live[i:i + last - first] == sections_base[first:last]]
        if occurrences != [start]:
            _fail(api, "Ambiguous managed lines in %s" % path)
        newline = "\r\n" if "\r\n" in current else "\n"
        replacement = [line.rstrip("\r\n") + newline if line.endswith(("\n", "\r")) else line
                       for line in wanted[old_first:old_last]]
        edits.append((start, start + last - first, replacement))
    for first, last, replacement in reversed(edits):
        live[first:last] = replacement
    return "".join(live)


def _valid_config_change(before, after, path, main, api):
    """Re-render all allowed installer edits; reject arbitrary manifest writes."""
    document = api.ConfigDocument(path, before)
    installed = api.ConfigDocument(path, after)
    old_names = [section.name.lower() for section in document.sections]
    new_names = [section.name.lower() for section in installed.sections]
    expected = old_names[:]
    if path == main and len(new_names) == len(expected) + 1 and new_names[-1] == "include " + api.MANAGED:
        expected.append("include " + api.MANAGED)
        document.insertions.setdefault(document.active_end, []).append(
            document.newline + "[include " + api.MANAGED + "]" + document.newline + document.newline)
    if new_names != expected or len(set(old_names)) != len(old_names):
        _fail(api, "Unrecognized section changes in installation backup: %s" % path)
    for original, changed in zip(document.sections, installed.sections):
        name = original.name.lower()
        replace, disable = {}, ()
        if name in ("stepper_x", "stepper_y"):
            axis = name[-1]
            pin = changed.get("endstop_pin")
            if pin != original.get("endstop_pin"):
                if pin not in ("accel_homing:" + axis, "beacon_accel:" + axis):
                    _fail(api, "Unrecognized endstop change in %s" % path)
                replace["endstop_pin"] = pin
            if changed.get("homing_retract_dist") != original.get("homing_retract_dist"):
                if changed.get("homing_retract_dist") != "0":
                    _fail(api, "Unrecognized retract change in %s" % path)
                replace["homing_retract_dist"] = "0"
            if changed.get("homing_speed") != original.get("homing_speed"):
                value = changed.get("homing_speed")
                try:
                    speed = float(value)
                    if not 0 < speed < float("inf"):
                        raise ValueError()
                except (TypeError, ValueError):
                    _fail(api, "Unrecognized homing speed change in %s" % path)
                replace["homing_speed"] = value
            disable = ("homing_accel", "min_home_dist")
        elif re.match(r"^tmc\w+ stepper_[xy]\d*$", name):
            disable = ("diag_pin", "diag0_pin", "diag1_pin", "home_current")
        elif name in ("accel_homing", "beacon_accel_homing"):
            value = changed.get("accel_chip")
            if value != original.get("accel_chip"):
                if not value or value.split()[0].lower() not in api.SENSORS:
                    _fail(api, "Unrecognized accelerometer change in %s" % path)
                replace["accel_chip"] = value
            policy = None
            for option in ("max_report_travel", "max_startup_travel"):
                value = changed.get(option)
                if value == original.get(option):
                    continue
                if policy is None:
                    policy = api.load_motion_policy()
                low, high = ((policy.REPORT_TRAVEL_MIN, policy.REPORT_TRAVEL_MAX)
                             if option == "max_report_travel" else
                             (policy.STARTUP_REQUEST_TRAVEL_MIN, policy.STARTUP_REQUEST_TRAVEL_MAX))
                try:
                    travel = float(value)
                    if not math.isfinite(travel) or not low <= travel <= high:
                        raise ValueError()
                except (TypeError, ValueError):
                    _fail(api, "Unrecognized %s change in %s" % (option, path))
                replace[option] = value
        for key, value in replace.items():
            original.replace(key, value)
        for key in disable:
            if original.get(key) is not None and changed.get(key) is None:
                original.disable(key)
    if document.rendered() != after:
        _fail(api, "Backup contains changes outside installer ownership: %s" % path)


def _config_paths(main, states, api):
    documents, stack = {}, []
    def visit(path):
        path = _path(str(path), api)
        if path in documents or path in stack:
            _fail(api, "Ambiguous config include graph in installation history: %s" % path)
        state = states[path] if path in states else api.snapshot(path)
        if state["kind"] != "file":
            _fail(api, "Missing/unsafe config context for %s" % path)
        document = api.ConfigDocument(path, _bytes(state))
        documents[path] = state
        stack.append(path)
        for section in document.sections:
            if not section.name.lower().startswith("include "):
                continue
            pattern = os.path.expanduser(section.name[8:].strip())
            if not Path(pattern).is_absolute():
                pattern = str(path.parent / pattern)
            matches = {Path(p).resolve() for p in glob.glob(pattern)}
            matches.update(p for p in states if p.match(pattern))
            matches = sorted(p for p in matches if states.get(p, {}).get("kind") != "absent")
            if not matches:
                _fail(api, "Missing include context for %s: %s" % (path, pattern))
            for match in matches:
                visit(match)
        stack.pop()
    visit(main)
    return documents


def _generated(data, path, api, strict=False):
    document = api.ConfigDocument(path, data)
    return (not _split_saved(data, api)[1]
            and [section.name.lower() for section in document.sections] == ["accel_homing"]
            and (not strict or data.startswith(b"# Managed by the accelerometer homing installer.")))


def _restore_active_models(baseline, current, path, api):
    """Recognize only Klipper's exact '#' comment-out of original model options."""
    before = api.ConfigDocument(path, baseline.encode())
    live = api.ConfigDocument(path, current.encode())
    originals = {section.name.lower(): section for section in before.sections if MODEL.match(section.name)}
    sections = {section.name.lower(): section for section in live.sections}
    replacements = {}
    for name, section in originals.items():
        target = sections.get(name)
        if target is None:
            _fail(api, "Preexisting active plugin model section changed in %s" % path)
        for key, values in section.options.items():
            if len(values) != 1:
                _fail(api, "Ambiguous preexisting model option in %s" % path)
            index = values[0][0]
            old = before.lines[index]
            if target.get(key) == section.get(key):
                continue
            candidates = [i for i in range(target.start + 1, target.end)
                          if live.lines[i].rstrip("\r\n") in (
                              "#" + old.rstrip("\r\n"), "#" + old.lstrip().rstrip("\r\n"),
                              "# " + old.lstrip().rstrip("\r\n"))]
            if target.get(key) is not None or len(candidates) != 1:
                _fail(api, "Preexisting active plugin model option changed in %s: %s" % (path, key))
            newline = "\r\n" if "\r\n" in current else "\n"
            replacements[candidates[0]] = old.rstrip("\r\n") + newline
    return "".join(replacements.get(i, line) for i, line in enumerate(live.lines))


def validate_installation_history(selected, api):
    """Validate every archived transaction without trying to restore live config."""
    config, repo = selected["config"], selected["repo"]
    records, active, receipt = history(config, api)
    if not active:
        return dict(records=records, active=active, receipt=receipt, validated=[],
                    baseline=None, updater_context=None, updater_paths=set())
    extras, exclude = repo / "klippy/extras", repo / ".git/info/exclude"
    validated, baseline = [], None
    updater_context, updater_paths = None, set()
    for record in active:
        manifest, directory = record["manifest"], record["directory"]
        if manifest.get("repo") is not None and _path(manifest["repo"], api) != repo:
            _fail(api, "Backup belongs to a different Klipper checkout: %s" % manifest["repo"], directory)
        states = {}
        context = manifest.get("context", {})
        if not isinstance(context, dict):
            _fail(api, "Invalid config baseline context", directory)
        for name, state in context.items():
            path = _path(name, api)
            states[path] = _state(state, api)
            if state["kind"] != "file":
                _fail(api, "Unsafe config baseline context: %s" % path, directory)
        for entry in manifest["changes"]:
            path = Path(entry["path"])
            if path in states and not _same(states[path], entry["before"]):
                _fail(api, "Config baseline differs from transaction before state: %s" % path, directory)
            states[path] = entry["before"]
        documents = _config_paths(config, states, api)
        if baseline is None:
            # Never use today's unchanged config as evidence of old saved models.
            baseline = {path: state for path, state in documents.items() if path in states}
        generated = config.parent / api.MANAGED
        owned_updater = {}
        if manifest.get("updater") is not None:
            helper = api.load_updater()
            if helper is None:
                _fail(api, "Updater history requires its verified helper", directory)
            owned_updater = helper.validate_record(manifest, selected, api)
            updater_context = manifest["updater"]
            updater_paths.update(owned_updater)
        for entry in manifest["changes"]:
            path, before, after = Path(entry["path"]), entry["before"], entry["after"]
            if path.parent == extras and path.name in api.MODULES:
                if after["kind"] != "symlink" or before["kind"] not in ("absent", "symlink"):
                    _fail(api, "Invalid managed module ownership: %s" % path, directory)
                target = Path(after["target"])
                if not target.is_absolute() or target.name != path.name or ".." in target.parts:
                    _fail(api, "Suspicious managed module target: %s" % path, directory)
                kind = "link"
            elif path == exclude:
                if after["kind"] != "file" or before["kind"] not in ("file", "absent"):
                    _fail(api, "Invalid Git exclude history", directory)
                old = _bytes(before) if before["kind"] == "file" else b""
                missing = ["klippy/extras/" + name for name in api.MODULES if "klippy/extras/" + name not in old.decode("utf-8").splitlines()]
                expected = old + (b"\n" if old and not old.endswith(b"\n") else b"") + ("\n".join(missing) + "\n").encode()
                if not missing or _bytes(after) != expected:
                    _fail(api, "Git exclude backup contains unowned changes", directory)
                kind = "exclude"
            elif path == generated and before["kind"] == "absent":
                if after["kind"] != "file" or not _generated(_bytes(after), path, api, strict=True):
                    _fail(api, "Invalid generated plugin config ownership: %s" % path, directory)
                kind = "generated"
            elif path in owned_updater:
                kind = owned_updater[path]
            elif path in documents:
                if before["kind"] != "file" or after["kind"] != "file":
                    _fail(api, "Unsafe config history: %s" % path, directory)
                _valid_config_change(_bytes(before), _bytes(after), path, config, api)
                kind = "config"
            else:
                _fail(api, "Backup path is outside selected instance/installer ownership: %s" % path, directory)
            validated.append((entry, kind, directory))
    first = active[0]
    if first["manifest"]["version"] == 1:
        # v1 has no linked journal. A first snapshot still containing installer
        # markers/owned pins cannot prove that an older transaction is present.
        for entry in first["manifest"]["changes"]:
            before = entry["before"]
            if before["kind"] == "file" and (b"# accel_homing previous:" in _bytes(before)
                    or b"[include accel_homing.cfg]" in _bytes(before)):
                _fail(api, "Ambiguous/gapped legacy history; earliest snapshot is already managed", first["directory"])
    return dict(records=records, active=active, receipt=receipt, validated=validated,
                baseline=baseline, updater_context=updater_context, updater_paths=updater_paths)


def validate_migration(selected, api):
    """Prove current module ownership without inverse-merging user configuration."""
    checked = validate_installation_history(selected, api)
    validated = checked["validated"]
    owned_links = {Path(entry["path"]) for entry, kind, _ in validated if kind == "link"}
    if not owned_links:
        return {}
    modules = {selected["repo"] / "klippy/extras" / name for name in api.MODULES}
    owned_files = {Path(entry["path"]) for entry, kind, _ in validated
                   if kind in ("moon_generated", "updater_receipt")}
    initial = {path: api.snapshot(path) for path in modules | owned_files}
    for path in modules - owned_links:
        if initial[path]["kind"] != "absent":
            _fail(api, "Unowned module link cannot be migrated automatically: %s" % path)
    virtual = {path: dict(state) for path, state in initial.items()}
    for entry, kind, directory in reversed(validated):
        if kind not in ("link", "moon_generated", "updater_receipt"):
            continue
        path = Path(entry["path"])
        if not _same(virtual[path], entry["after"]):
            message = ("Managed link changed/missing: %s" if kind == "link"
                       else "Owned updater file changed/missing: %s")
            _fail(api, message % path, directory)
        virtual[path] = entry["before"]
    return initial


def make_plan(selected, api):
    """Return encoded before/after entries and history refs, without writes."""
    config, repo = selected["config"], selected["repo"]
    checked = validate_installation_history(selected, api)
    records, active, receipt = checked["records"], checked["active"], checked["receipt"]
    if not active:
        if receipt:
            for entry in receipt["manifest"]["changes"]:
                if entry["after"]["kind"] != "file" and not api.state_matches(Path(entry["path"]), entry["after"]):
                    _fail(api, "Restored path changed after uninstall: %s" % entry["path"], receipt["directory"])
            return dict(entries=[], history=[], records=[item["ref"] for item in records], config=config, repo=repo)
        # With no history, absence of activation is an ordinary idempotent no-op.
        graph = api.ConfigGraph(config)
        links = [repo / "klippy/extras" / name for name in api.MODULES]
        evidence = [str(p) for p in links if p.exists() or p.is_symlink()]
        evidence += [str(p) for p, document in graph.documents.items()
                     if "# accel_homing previous:" in document.text
                     or any(section.name.lower() in ("accel_homing", "beacon_accel_homing") for section in document.sections)]
        if evidence:
            _fail(api, "No complete installation history for existing plugin state: %s. Recover the original .accel_homing_backups directory" % ", ".join(evidence))
        return dict(entries=[], history=[], records=[item["ref"] for item in records], config=config, repo=repo)
    extras, exclude = repo / "klippy/extras", repo / ".git/info/exclude"
    validated, baseline = checked["validated"], checked["baseline"]
    updater_context, updater_paths = checked["updater_context"], checked["updater_paths"]
    first = active[0]
    paths = {Path(entry["path"]) for entry, _, _ in validated} | set(baseline or {}) | {config}
    initial = {path: api.snapshot(path) for path in paths}
    virtual = {path: dict(state) for path, state in initial.items()}
    for entry, kind, directory in reversed(validated):
        path, before, after = Path(entry["path"]), entry["before"], entry["after"]
        live = virtual[path]
        if kind == "link":
            if not _same(live, after):
                _fail(api, "Managed link changed/missing: %s" % path, directory)
            virtual[path] = before
        elif kind in ("moon_generated", "updater_receipt"):
            if not _same(live, after):
                _fail(api, "Owned updater file changed/missing: %s" % path, directory)
            virtual[path] = before
        elif kind == "generated":
            if live["kind"] != "file" or not _generated(_bytes(live), path, api):
                _fail(api, "Generated plugin config has unowned sections/changes: %s" % path, directory)
            virtual[path] = before
        else:
            if live["kind"] != "file":
                _fail(api, "Managed file replaced/missing: %s" % path, directory)
            old_data = _bytes(before) if before["kind"] == "file" else b""
            if kind == "config":
                old, saved_old = _split_saved(old_data, api)
                installed, saved_installed = _split_saved(_bytes(after), api)
                current, saved = _split_saved(_bytes(live), api)
                if saved_old != saved_installed:
                    _fail(api, "Installer backup unexpectedly changes SAVE_CONFIG: %s" % path, directory)
                if saved or saved_old:
                    # Klipper normalizes newlines and uses regular_data.rstrip()
                    # when saving. Its removal of separator blank lines must not
                    # make an unchanged installer include look like a conflict.
                    suffix = old[len(old.rstrip("\r\n")):]
                    merged_active = _undo_text(installed.rstrip("\r\n") + "\n", old.rstrip("\r\n") + "\n",
                                               current.rstrip("\r\n") + "\n", path, api)
                    merged = (merged_active.rstrip("\r\n") + (suffix or "\n") + saved).encode()
                else:
                    merged = _undo_text(installed, old, current, path, api).encode()
            else:
                merged = _undo_text(_bytes(after).decode(), old_data.decode(), _bytes(live).decode(), path, api).encode()
            virtual[path] = _file(merged, live["mode"]) if merged or before["kind"] == "file" else {"kind": "absent"}
    # All ordinary SAVE_CONFIG edits are considered only here, after all history
    # has been checked. Non-plugin saved sections remain byte-for-byte current.
    for path in paths:
        state = virtual[path]
        if state["kind"] != "file" or path == exclude or path in updater_paths:
            continue
        current, saved = _split_saved(_bytes(state), api)
        if path not in (baseline or {}):
            if any(MODEL.match(name) for name, _ in _saved_sections(saved, api)[1]):
                _fail(api, "Legacy history has no original main-config/model baseline for %s; cannot determine which plugin models predate installation" % path, first["directory"])
            continue
        original, original_saved = _split_saved(_bytes(baseline[path]), api)
        current = _restore_active_models(original, current, path, api)
        saved = _restore_models(original_saved, saved, api)
        if not saved and not original_saved and current.rstrip() == original.rstrip():
            current = original
        virtual[path] = _file((current + saved).encode(), state["mode"])
    final_documents = _config_paths(config, virtual, api)
    original_plugin_sections = set()
    original_plugin_pins = set()
    for path, state in (baseline or {}).items():
        document = api.ConfigDocument(path, _bytes(state))
        for section in document.sections:
            name = section.name.lower()
            if name in ("accel_homing", "beacon_accel_homing") or MODEL.match(name):
                original_plugin_sections.add((path, name))
            pin = section.get("endstop_pin")
            if pin and pin.startswith(("accel_homing:", "beacon_accel:")):
                original_plugin_pins.add((path, name, pin))
    for path, state in final_documents.items():
        document = api.ConfigDocument(path, _bytes(state))
        for section in document.sections:
            name = section.name.lower()
            if ((name in ("accel_homing", "beacon_accel_homing") or MODEL.match(name))
                    and (path, name) not in original_plugin_sections):
                _fail(api, "New/unowned active plugin or model section [%s] in %s; reconcile it against the original baseline" %
                      (section.name, path), first["directory"])
            pin = section.get("endstop_pin")
            if pin and pin.startswith(("accel_homing:", "beacon_accel:")) and (path, name, pin) not in original_plugin_pins:
                _fail(api, "New/unowned plugin endstop reference in %s [%s]" % (path, section.name), first["directory"])
    for path, state in virtual.items():
        if path.parent == extras and state["kind"] == "symlink":
            target = Path(state["target"])
            target = target if target.is_absolute() else path.parent / target
            if not target.is_file():
                _fail(api, "Original plugin source is missing for %s: %s. Recover that original source before retrying" %
                      (path, target), first["directory"])
    entries = [dict(path=str(path), before=initial[path], after=virtual[path]) for path in sorted(paths)
               if not _same(initial[path], virtual[path])]
    if not entries:
        _fail(api, "Installation history has no verifiable activation to restore", first["directory"])
    return dict(entries=entries, history=[item["ref"] for item in active],
                records=[item["ref"] for item in records], config=config, repo=repo,
                updater=updater_context,
                expected={str(path): initial[path] if path in initial else final_documents[path]
                          for path in set(initial) | set(final_documents)})
