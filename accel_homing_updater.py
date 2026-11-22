"""Managed Moonraker git transport and narrowly owned installer configuration.

All commands are argv-only. The installer supplies idle/identity guards before
source/config writes. Importing this module never contacts or changes a host.
"""
import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import time


def _current_users():
    if not hasattr(os, "getuid"):
        return None
    import pwd
    uid = os.getuid()
    return {str(uid), pwd.getpwuid(uid).pw_name}

ORIGIN = "https://github.com/FrankRamm/accel_homing.git"
BRANCH = "main"
LEGACY_ORIGIN = "https://github.com/FrankRamm/MISC.git"
LEGACY_BRANCH = "codex/universal-accel-homing"
SECTION = "update_manager accel_homing"
INCLUDE = "accel_homing_update.cfg"
RECEIPT = ".accel_homing_updater.json"
ACTIVATION_RECEIPT = ".accel_homing_moonraker_activation.json"
PATH_ENVIRONMENT = frozenset(("HOME", "MOONRAKER_DATA_PATH", "MOONRAKER_CONFIG_PATH"))


class _MoonrakerDiscovery(list):
    def __init__(self):
        super().__init__()
        self.diagnostics = []


def _diagnostic_value(value):
    return json.dumps(str(value)[:240], ensure_ascii=True)


def _discovery_note(notes, source, reason, config=None):
    if notes is None or len(notes) >= 8:
        return
    text = "%s: %s" % (_diagnostic_value(source), reason)
    if config is not None:
        text += "; config=" + _diagnostic_value(config)
    notes.append(text)


def _config_rejection(error):
    message = str(error)
    if message.startswith("Include has no matches"):
        return "configuration include pattern has no matches"
    if message.startswith("Config included more than once"):
        return "configuration includes the same file more than once"
    if message.startswith("Recursive config include"):
        return "configuration has a recursive include"
    if message.startswith("Expected exactly one [server]"):
        return "configuration needs one unambiguous [server] section"
    if message.startswith("Duplicate "):
        return "configuration has an ambiguous duplicate option"
    return "configuration could not be read by the safe configuration parser"


def _fail(api, message):
    raise api.InstallError("Moonraker updater: " + message)


def instance_key(selected):
    value = str(selected["repo"]) + "\0" + str(selected["config"])
    return hashlib.sha256(value.encode()).hexdigest()[:16]


def managed_path(selected):
    # The caller owns this checkout. Never rename, replace, reset, or remove it.
    # A fixed path identifies the documented checkout for native updates.
    return Path.home() / "accel_homing"


def _moon_index(argv):
    indexes = [i for i, value in enumerate(argv) if Path(value).name == "moonraker.py"]
    indexes += [i + 1 for i, value in enumerate(argv[:-1]) if value == "-m" and argv[i + 1] == "moonraker"]
    return indexes[0] if len(indexes) == 1 else None


def _path_environment(process):
    # Retain only Moonraker's path defaults, never credentials or other env.
    with (process / "environ").open("rb") as stream:
        raw = stream.read(65537)
    if len(raw) > 65536:
        raise ValueError("Moonraker path environment exceeds the discovery bound")
    result = {}
    allowed = {name.encode("ascii") for name in PATH_ENVIRONMENT}
    for field in raw.split(b"\0"):
        key, separator, value = field.partition(b"=")
        if separator and key in allowed:
            name = key.decode("ascii")
            if name in result:
                raise ValueError("ambiguous Moonraker path environment")
            result[name] = value.decode("utf-8")
    return result


def _owner_home(uid):
    try:
        import pwd
        return pwd.getpwuid(uid).pw_dir
    except (ImportError, KeyError):
        return None


def _expand_moon_path(value, environment, owner_home):
    if not value.startswith("~"):
        return value
    if environment is None:
        # Legacy literal stopped-unit discovery has no live environment proof.
        return os.path.expanduser(value)
    home = environment.get("HOME", owner_home)
    if not home or not Path(home).is_absolute() or (value != "~" and not value.startswith("~/")):
        raise ValueError("cannot establish Moonraker's user-home expansion")
    return home.rstrip("/") + value[1:] or "/"


def _moon_command(argv, cwd, api, pid=None, service=None, scope=None, start=None,
                  path_environment=None, owner_home=None, diagnostics=None):
    index = _moon_index(argv)
    if index is None:
        return None
    environment = ({key: value for key, value in path_environment.items() if key in PATH_ENVIRONMENT}
                   if path_environment is not None else None)
    options, iterator = {}, iter(argv[index + 1:])
    for value in iterator:
        name, separator, inline = value.partition("=")
        if name in ("-d", "--datapath", "-c", "--configfile"):
            options["data" if name in ("-d", "--datapath") else "config"] = inline if separator else next(iterator, "")
    # Moonraker server.main: argv overrides MOONRAKER_*; absent/empty data
    # falls back to ~/printer_data, but an empty config is not a default.
    defaults = environment or {}
    raw_data = options.get("data", defaults.get("MOONRAKER_DATA_PATH"))
    source = "PID %s" % pid if pid is not None else "service %s" % (service or "unknown")
    if not raw_data and environment is None:
        _discovery_note(diagnostics, source, "data path unavailable without explicit argv or verified live environment")
        return None
    config = None
    try:
        data = api.canonical(_expand_moon_path(raw_data or "~/printer_data", environment, owner_home), cwd)
        raw_config = options.get("config", defaults.get("MOONRAKER_CONFIG_PATH"))
        config = (api.canonical(_expand_moon_path(raw_config, environment, owner_home), cwd)
                  if raw_config is not None else data / "config/moonraker.conf")
        graph = api.ConfigGraph(config)
        server = graph.one("server")
        # Moonraker defaults this option to /tmp/klippy_uds. An explicit empty
        # value is a configured relative path, not permission to use a default.
        raw_socket = server.get("klippy_uds_address", "/tmp/klippy_uds")
        raw_socket = re.sub(r"\{\s*data_path\s*\}", lambda _match: str(data), raw_socket)
        if any(token in raw_socket for token in ("$", "%", "{", "}")):
            _discovery_note(diagnostics, source, "socket expression is unsupported; only literal paths and {data_path} are resolved", config)
            return None
        # ConfigHelper.getpath resolves configured relative paths against the
        # Moonraker process working directory, after template/home expansion.
        socket = api.canonical(_expand_moon_path(raw_socket, environment, owner_home), cwd)
    except api.InstallError as exc:
        _discovery_note(diagnostics, source, _config_rejection(exc), config)
        return None
    except (OSError, ValueError) as exc:
        _discovery_note(diagnostics, source, "path/configuration resolution failed (%s)" % type(exc).__name__, config)
        return None
    return dict(config=config, data=data, socket=socket.resolve(), pid=pid,
                service=service, service_scope=scope, start_time=start,
                executable=argv[0], argv=list(argv), cwd=str(cwd), graph=graph,
                path_environment=environment, owner_home=owner_home)


def moonraker_instances(api, proc_root=Path("/proc"), unit_directories=None):
    found = _MoonrakerDiscovery()
    skipped_uid = 0
    if proc_root.is_dir():
        for process in proc_root.iterdir():
            if not process.name.isdigit():
                continue
            recognized = False
            try:
                uid = process.stat().st_uid
                if hasattr(os, "getuid") and uid != os.getuid():
                    skipped_uid += 1
                    continue
                raw = (process / "cmdline").read_bytes()
                if len(raw) > 65536:
                    continue
                argv = [part.decode("utf-8") for part in raw.split(b"\0") if part]
                if _moon_index(argv) is None:
                    continue
                recognized = True
                try:
                    environment = _path_environment(process)
                except (OSError, UnicodeError, ValueError) as exc:
                    _discovery_note(found.diagnostics, "PID " + process.name,
                                    "bounded path environment unavailable (%s)" % type(exc).__name__)
                    continue
                groups = (process / "cgroup").read_text() if (process / "cgroup").is_file() else ""
                services = re.findall(r"/([^/\n]+\.service)(?:/|$)", groups)
                fields = (process / "stat").read_text().rsplit(") ", 1)[1].split()
                item = _moon_command(argv, (process / "cwd").resolve(), api, int(process.name),
                                     services[-1] if services else None,
                                     "user" if "/user.slice/" in groups else "system", fields[19],
                                     path_environment=environment, owner_home=_owner_home(uid),
                                     diagnostics=found.diagnostics)
                if item:
                    found.append(item)
            except (OSError, UnicodeError, ValueError, IndexError) as exc:
                if recognized:
                    _discovery_note(found.diagnostics, "PID " + process.name,
                                    "process identity metadata unavailable (%s)" % type(exc).__name__)
                continue
    if skipped_uid:
        _discovery_note(found.diagnostics, "process scan",
                        "skipped %s processes owned by other UIDs; only installer UID %s was inspected" % (skipped_uid, os.getuid()))
    directories = unit_directories or (Path("/etc/systemd/system"), Path("/lib/systemd/system"),
                                       Path("/usr/lib/systemd/system"), Path.home() / ".config/systemd/user")
    seen = {item["service"] for item in found if item["service"]}
    for directory in directories:
        for unit in sorted(directory.glob("*.service")) if directory.is_dir() else ():
            if unit.name in seen:
                continue
            seen.add(unit.name)
            recognized = "moonraker" in unit.name.lower()
            try:
                text = re.sub(r"\\\r?\n\s*", " ", unit.read_text())
                if not re.search(r"\bmoonraker(?:\.py)?\b", text, re.I):
                    continue
                recognized = True
                if any((directory / (unit.name + ".d")).glob("*.conf")):
                    _discovery_note(found.diagnostics, unit.name, "service has drop-ins and needs a readable matching live process")
                    continue
                users = re.findall(r"^\s*User\s*=\s*(\S+)\s*$", text, re.M)
                allowed_users = _current_users()
                if allowed_users is not None and (len(users) != 1 or users[0] not in allowed_users):
                    _discovery_note(found.diagnostics, unit.name, "service user does not match the installer user")
                    continue
                starts = re.findall(r"^\s*ExecStart\s*=\s*(.+)$", text, re.M)
                working = re.findall(r"^\s*WorkingDirectory\s*=\s*(.+)$", text, re.M)
                if len(starts) != 1 or "$" in starts[0] or "%" in starts[0]:
                    _discovery_note(found.diagnostics, unit.name, "service arguments need a readable matching live process")
                    continue
                item = _moon_command(shlex.split(starts[0]), working[-1] if working else "/", api,
                                     service=unit.name,
                                     scope="user" if directory == Path.home() / ".config/systemd/user" else "system",
                                     diagnostics=found.diagnostics)
                if item:
                    found.append(item)
            except (OSError, UnicodeError, ValueError) as exc:
                if recognized:
                    _discovery_note(found.diagnostics, unit.name, "service definition unavailable (%s)" % type(exc).__name__)
                continue
    return found


def discover_moonraker(selected, api, explicit=None, instances=None):
    if not selected.get("socket"):
        _fail(api, "the selected Klipper has no verified API socket; specify --klippy-socket or use --no-update-manager")
    instances = moonraker_instances(api) if instances is None else instances
    config = api.canonical(explicit) if explicit else None
    candidates = [item for item in instances if item["socket"] == selected["socket"]
                  and (config is None or item["config"] == config)]
    live = [item for item in candidates if item.get("pid")]
    if len(live) > 1:
        _fail(api, "multiple live Moonraker processes match this Klippy socket; no instance was guessed")
    candidates = live or candidates
    unique = {(item["config"], item.get("service")): item for item in candidates}
    if len(unique) != 1:
        details = ["Selected Klipper config=%s; socket=%s; installer UID=%s." % (
            _diagnostic_value(selected["config"]), _diagnostic_value(selected["socket"]),
            os.getuid() if hasattr(os, "getuid") else "unavailable")]
        for item in instances[:4]:
            reason = ("socket mismatch" if item["socket"] != selected["socket"] else
                      "explicit config selector mismatch" if config is not None and item["config"] != config else
                      "matches selected socket; selection is ambiguous")
            details.append("Verified Moonraker candidate PID=%s service=%s config=%s socket=%s: %s." % (
                item.get("pid"), _diagnostic_value(item.get("service")), _diagnostic_value(item["config"]),
                _diagnostic_value(item["socket"]), reason))
        details.extend(getattr(instances, "diagnostics", [])[:8])
        if not instances and not getattr(instances, "diagnostics", []):
            details.append("No readable recognized Moonraker process or literal service candidate was found for this user.")
        _fail(api, "cannot select exactly one Moonraker for this Klippy socket (%d matches). "
              "Run as the selected service user; --moonraker-config "
              "only filters verified candidates and cannot supply missing process/socket evidence.\n%s" %
              (len(unique), "\n".join(details)))
    return next(iter(unique.values()))


def _fingerprint(moon):
    return {key: str(moon[key]) if isinstance(moon[key], Path) else moon.get(key)
            for key in ("config", "data", "socket", "pid", "start_time", "service", "service_scope", "argv",
                        "cwd", "path_environment", "owner_home")}


def assert_identity(moon, selected, api):
    current = discover_moonraker(selected, api, moon["config"])
    if _fingerprint(current) != _fingerprint(moon):
        _fail(api, "selected Moonraker process/service/config/socket changed; stop and retry")
    if not current.get("pid"):
        systemctl = shutil.which("systemctl")
        if not systemctl or not current.get("service") or current.get("service_scope") != "system":
            _fail(api, "a stopped Moonraker requires a verifiable loaded system service even with --no-restart")
        service_identity(moon, selected, api, systemctl)
    for path, document in moon["graph"].documents.items():
        if api.snapshot(path).get("data") != base64.b64encode(document.original).decode():
            _fail(api, "Moonraker configuration changed after planning: %s" % path)


def stanza(repo, branch=BRANCH):
    return ("# Managed by the accelerometer homing installer.\n"
            "[update_manager accel_homing]\n"
            "type: git_repo\nchannel: dev\npath: %s\norigin: %s\nprimary_branch: %s\n"
            "is_system_service: False\nmanaged_services: klipper\n" % (repo, ORIGIN, branch)).encode()


def main_suffix(add_base, add_include):
    # Compatibility helper for existing include-based configuration layouts.
    text = "\n# accel_homing managed Moonraker update registration\n"
    if add_base:
        text += "[update_manager]\n"
    if add_include:
        text += "[include %s]\n" % INCLUDE
    return text.encode() if add_base or add_include else b""


def direct_suffix(repo, add_base, add_stanza, branch=BRANCH):
    if not add_base and not add_stanza:
        return b""
    text = b"\n# accel_homing managed Moonraker update registration\n"
    if add_base:
        text += b"[update_manager]\n"
    return text + (stanza(repo, branch) if add_stanza else b"")


def _origin(value):
    return re.sub(r"\.git$", "", str(value or "").rstrip("/"), flags=re.I).lower()


def registration_path(section, moon, api):
    configured = section.get("path")
    if not configured or any(token in configured for token in ("$", "%", "{", "}")):
        return None
    environment = moon.get("path_environment")
    if configured.startswith("~") and (not moon.get("pid") or environment is None):
        _fail(api, "Moonraker's live home is unavailable for path: %s; use an absolute checkout path" % configured)
    try:
        expanded = _expand_moon_path(configured, environment, moon.get("owner_home"))
        return api.canonical(expanded, moon["cwd"])
    except (OSError, ValueError, RuntimeError):
        return None


def matching_stanza(section, repo, api, moon):
    # Operator settings, comments, whitespace and include layout are authoritative.
    # Only transport identity must still identify the checkout being installed.
    for name in section.options:
        section.get(name)
    if (section.get("type", "").lower() != "git_repo"
            or _origin(section.get("origin")) != _origin(ORIGIN)):
        return False
    resolved = registration_path(section, moon, api)
    if resolved != repo.resolve() and str(section.get("path", "")).startswith("~"):
        _fail(api, "path: %s does not identify the installed clone under Moonraker's verified home; use path: %s" %
              (section.get("path"), repo))
    return resolved == repo.resolve()


def _metadata(path):
    # Old/corrupt metadata never supplies configuration or historical ownership.
    try:
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 16384:
            return {}
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError, UnicodeError):
        return {}


def _registration_digest(section, base, path):
    value = dict(path=str(path), options={key: section.get(key) for key in section.options},
                 base={key: base.get(key) for key in base.options} if base else {})
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def activation_needed(selected, context, api, moon=None):
    value = _metadata(selected["config"].parent / ACTIVATION_RECEIPT)
    current = moon or discover_moonraker(selected, api, context["moon_config"])
    process = {key: current.get(key) for key in ("pid", "start_time", "service", "service_scope")}
    return not (current.get("pid") and value.get("status") == "restarted"
                and value.get("config") == context["moon_config"]
                and value.get("instance") == instance_key(selected)
                and value.get("registration_sha256") == context["registration_sha256"]
                and value.get("process") == process)


def source_commit(source, api):
    if (source / ".source.json").exists() or (source / ".source.json").is_symlink():
        _fail(api, "archive source manifests are no longer supported by the standalone updater; "
              "use the complete ~/accel_homing Git clone, or --no-update-manager for offline development")
    identity = repo_identity(source, api)
    required = tuple(api.MODULES) + ("install.sh", "uninstall.sh", "installer.py", "uninstaller.py",
                                     "accel_homing_updater.py", "LICENSE", "NOTICE")
    if any(not (source / name).is_file() or (source / name).is_symlink() for name in required):
        _fail(api, "incomplete standalone checkout; runtime, installer, LICENSE and NOTICE must be regular tracked files")
    git(source, ["ls-files", "--error-unmatch", "--"] + list(required), api)
    return identity["commit"]


def legacy_context(value):
    return isinstance(value, dict) and _origin(value.get("origin")) in {
        _origin(LEGACY_ORIGIN), _origin("https://github.com/Monolith3D/MISC.git")}


def legacy_installation(selected, api):
    """Identify current legacy links/registration without consulting old records."""
    for name in api.MODULES:
        link = selected["repo"] / "klippy/extras" / name
        state = api.snapshot(link)
        if state["kind"] == "symlink" and Path(state["target"]).parent.name == "Beacon_Accel_Homing":
            return True
    if selected.get("socket"):
        for moon in moonraker_instances(api):
            if moon["socket"] != selected["socket"]:
                continue
            section = moon["graph"].one(SECTION, False)
            if section and legacy_context({"origin": section.get("origin")}):
                return True
    return False


def migration_args(selected, args, api):
    result = copy.copy(args)
    if legacy_installation(selected, api):
        result._legacy_migration = True
        result.replace_existing_links = True
        paths = [selected["repo"] / "klippy/extras" / name for name in api.MODULES]
        result._migration_expected = {path: api.snapshot(path) for path in paths}
    return result


def assert_clone_consumers(source, selected, api):
    """One native updater must not silently change another detected host."""
    for peer in api.proc_instances() + api.service_instances(Path.home()):
        if peer["repo"] == selected["repo"] and peer["config"] == selected["config"]:
            continue
        extras = peer["repo"] / "klippy/extras"
        if any((extras / name).is_symlink() and api.canonical(extras / name) == source / name
               for name in api.MODULES):
            _fail(api, "another detected Klipper instance uses this source clone: %s. "
                  "The native updater supports one managed instance per ~/accel_homing checkout; "
                  "preserve the other instance before installing" % peer["config"])


def remove_registration(data, path, api):
    """Remove only this updater's active section; retain comments and other sections."""
    document = api.ConfigDocument(path, data)
    sections = [section for section in document.sections if section.name.lower() == SECTION]
    if len(sections) > 1:
        _fail(api, "duplicate [%s] sections in %s" % (SECTION, path))
    for section in sections:
        for index in range(section.start, section.end):
            line = document.lines[index].strip()
            if index == section.start or (line and not line.startswith(("#", ";"))):
                document.replacements[index] = ""
    return document.rendered()


def _replace_registration_option(section, name, value):
    entries = section.options.get(name, [])
    section.get(name)  # Reject duplicate options before preparing any writes.
    if entries:
        index, match = entries[0]
        line = section.document.lines[index]
        comment = re.search(r"(\s*[#;].*)$", match.group(4))
        suffix = comment.group(1) if comment else ""
        section.document.replacements[index] = line[:match.start(4)] + str(value) + suffix + line[match.end(4):]
    else:
        section.document.insertions.setdefault(section.end, []).append(
            "%s: %s%s" % (name, value, section.document.newline))


def _context(moon, repo, section, base, branch):
    path = section.document.path
    return dict(moon_config=str(moon["config"]), moon_data=str(moon["data"]),
                moon_socket=str(moon["socket"]), managed_repo=str(repo), origin=ORIGIN,
                branch=branch, registration_path=str(path),
                registration_sha256=_registration_digest(section, base, path))


def _graph_expected(graph, api):
    expected = {}
    for path, document in graph.documents.items():
        state = api.snapshot(path)
        original = base64.b64encode(document.original).decode()
        if state.get("kind") != "file" or state.get("data") != original:
            _fail(api, "Moonraker configuration changed after planning: %s" % path)
        # Expectations must describe the bytes used to render the change, never
        # a later editor's bytes observed after the configuration was parsed.
        expected[str(path)] = dict(state, data=original)
    return expected


def make_plan(source, selected, args, api):
    repo = managed_path(selected)
    if source != repo or source.is_symlink():
        _fail(api, "install the standalone Git checkout at %s and run its install.sh; "
              "use --no-update-manager for another local development source" % repo)
    commit = source_commit(source, api)
    identity = repo_identity(source, api)
    if identity["commit"] != commit:
        _fail(api, "standalone checkout changed during source validation; retry")
    assert_clone_consumers(source, selected, api)
    moon = discover_moonraker(selected, api, getattr(args, "moonraker_config", None))
    assert_identity(moon, selected, api)
    if any(token in str(repo) for token in ("\n", "\r", "#", ";")):
        _fail(api, "managed checkout path cannot be represented safely in Moonraker configuration")
    graph, changes = moon["graph"], {}
    existing = graph.one(SECTION, False)
    branch = identity["branch"]
    migrating = legacy_installation(selected, api)
    if existing:
        for name in existing.options:
            existing.get(name)
        registration_edits = {}
        current_branch = existing.get("primary_branch", BRANCH)
        if legacy_context({"origin": existing.get("origin")}):
            if existing.get("type", "").lower() != "git_repo":
                _fail(api, "conflicting existing [%s] transport; preserve/reconcile it before installing" % SECTION)
            registration_edits = {"path": repo, "origin": ORIGIN, "primary_branch": branch}
        elif not matching_stanza(existing, repo, api, moon):
            _fail(api, "conflicting existing [%s] path/origin/type; preserve/reconcile it instead of overwriting" % SECTION)
        elif current_branch in (BRANCH, "dev") and current_branch != branch:
            registration_edits["primary_branch"] = branch
        if registration_edits:
            document = api.ConfigDocument(existing.document.path, existing.document.original)
            edited = next(section for section in document.sections if section.name.lower() == SECTION)
            for name, value in registration_edits.items():
                _replace_registration_option(edited, name, value)
            rendered = document.rendered()
            changes[document.path] = dict(kind="file", data=rendered)
            existing = next(section for section in api.ConfigDocument(document.path, rendered).sections
                            if section.name.lower() == SECTION)
    base = graph.one("update_manager", False)
    if base:
        for name in base.options:
            base.get(name)
    suffix = direct_suffix(repo, base is None, existing is None, branch)
    if suffix:
        original = changes.get(moon["config"], {}).get("data", graph.documents[moon["config"]].original)
        changes[moon["config"]] = dict(kind="file", data=original + suffix)
    if existing is None:
        existing = api.ConfigDocument(moon["config"], stanza(repo, branch)).sections[0]
    context = _context(moon, repo, existing, base, branch)
    receipt = selected["config"].parent / RECEIPT
    receipt_state = api.snapshot(receipt)
    if receipt_state["kind"] not in ("absent", "file"):
        _fail(api, "refusing unsafe updater receipt")
    encoded = (json.dumps(dict(version=2, instance=instance_key(selected), commit=commit,
                               **context), sort_keys=True, indent=2) + "\n").encode()
    if receipt_state.get("data") != base64.b64encode(encoded).decode():
        changes[receipt] = dict(kind="file", data=encoded)
    expected = _graph_expected(graph, api)
    expected[str(receipt)] = receipt_state
    for path, change in changes.items():
        change["expected"] = expected[str(path)]
    return dict(moon=moon, repo=repo, source=repo, commit=commit, source_identity=identity, selected=selected,
                changes=changes, expected=expected, context=context, source_input=source, published=False,
                restart_needed=any(path != receipt for path in changes) or activation_needed(selected, context, api, moon),
                receipt=receipt, legacy_migration=migrating)


def make_uninstall_plan(selected, api):
    """Remove the currently configured plugin registration, never restore files."""
    changes, expected = {}, {}
    receipt = selected["config"].parent / RECEIPT
    value = _metadata(receipt)
    candidates = [moon for moon in moonraker_instances(api)
                  if selected.get("socket") and moon["socket"] == selected["socket"]]
    context = None
    if candidates or value.get("instance") == instance_key(selected):
        moon = discover_moonraker(selected, api, instances=candidates)
        assert_identity(moon, selected, api)
        graph = moon["graph"]
        expected.update(_graph_expected(graph, api))
        section = graph.one(SECTION, False)
        if section:
            # An explicit uninstall owns this plugin section, including edits
            # made since installation. Its other sections and base remain intact.
            path = section.document.path
            changes[path] = dict(kind="file", data=remove_registration(section.document.original, path, api))
            context = dict(moon_config=str(moon["config"]), moon_data=str(moon["data"]),
                           moon_socket=str(moon["socket"]))
    for path in (receipt, selected["config"].parent / ACTIVATION_RECEIPT):
        state = api.snapshot(path)
        expected[str(path)] = state
        if state["kind"] == "file":
            changes[path] = dict(kind="absent")
        elif state["kind"] != "absent":
            _fail(api, "refusing unsafe updater metadata during uninstall: %s" % path)
    for path, change in changes.items():
        change["expected"] = expected[str(path)]
    return dict(changes=changes, expected=expected, context=context)


def git(repo, arguments, api, timeout=120):
    command = ["git", "-C", str(repo)] + list(arguments)
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout,
                                env={**os.environ, "GIT_TERMINAL_PROMPT": "0"})
    except (OSError, subprocess.TimeoutExpired) as exc:
        _fail(api, "git command failed: %s" % exc)
    if result.returncode:
        _fail(api, "git %s failed: %s" % (arguments[0], result.stderr[-1500:].strip()))
    return result.stdout.strip()


def repo_identity(repo, api):
    locks = [repo / ".git" / name for name in ("index.lock", "HEAD.lock", "config.lock", "packed-refs.lock")]
    locks += list((repo / ".git/refs").rglob("*.lock")) if (repo / ".git/refs").is_dir() else []
    if (repo.is_symlink() or (repo / ".git").is_symlink() or not (repo / ".git").is_dir()
            or any(path.exists() for path in locks)):
        _fail(api, "managed source is not an unlocked dedicated checkout: %s" % repo)
    if Path(git(repo, ["rev-parse", "--show-toplevel"], api)).resolve() != repo.resolve():
        _fail(api, "managed source must be the Git checkout root")
    origin = git(repo, ["remote", "get-url", "origin"], api)
    branch = git(repo, ["symbolic-ref", "--short", "HEAD"], api)
    upstream = git(repo, ["rev-parse", "--abbrev-ref", "@{upstream}"], api)
    if origin != ORIGIN or branch not in (BRANCH, "dev") or upstream != "origin/" + branch:
        _fail(api, "managed checkout origin/branch/upstream differs; nothing was reset or stashed")
    if git(repo, ["status", "--porcelain=v1", "--untracked-files=all"], api):
        _fail(api, "managed checkout is dirty; preserve/reconcile those changes before updating")
    commit = git(repo, ["rev-parse", "HEAD"], api)
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        _fail(api, "standalone checkout did not return a full commit SHA")
    git(repo, ["merge-base", "--is-ancestor", commit, "origin/" + branch], api)
    return dict(commit=commit, origin=origin, branch=branch, upstream=upstream)


def assert_source_identity(plan, api):
    if repo_identity(plan["repo"], api) != plan["source_identity"]:
        _fail(api, "standalone checkout changed after planning; no source reset or replacement was attempted")
    if plan.get("selected"):
        assert_clone_consumers(plan["source"], plan["selected"], api)


def prepare_source(plan, api, guard):
    guard()
    assert_source_identity(plan, api)
    plan["candidate"] = plan["repo"]
    return plan["source"]


def publish_source(plan, api, guard):
    guard()
    assert_source_identity(plan, api)


def restore_publication(plan, api):
    # This installer only changes owned links/configuration. The source clone
    # remains caller-owned even after a failed installation or later Git pull.
    if plan.get("published"):
        _fail(api, "legacy source publication requires its previous installer for recovery")


def prepare_restart(moon, selected, api):
    systemctl, sudo = shutil.which("systemctl"), shutil.which("sudo")
    if not systemctl or not sudo or not moon.get("service") or moon.get("service_scope") != "system":
        _fail(api, "automatic activation needs the unique system Moonraker service; use --no-restart for manual activation")
    identity = service_identity(moon, selected, api, systemctl)
    plan = dict(systemctl=systemctl, sudo=sudo, identity=identity,
                service_command=[systemctl, "--system", "--no-ask-password", "restart", "--", moon["service"]])
    api.restart_permission(plan, acquire=True)
    return plan


def service_identity(moon, selected, api, systemctl):
    properties = api.read_service_unit(moon, systemctl)
    users = _current_users()
    if users is not None and properties.get("User") not in users:
        _fail(api, "Moonraker service is not owned by the installer/Klipper user")
    current = discover_moonraker(selected, api, moon["config"])
    active = properties.get("ActiveState")
    pid = int(properties.get("MainPID", "-1"))
    if current.get("pid"):
        if (active != "active" or properties.get("SubState") != "running"
                or current["pid"] != pid or current.get("service") != moon["service"]
                or current.get("service_scope") != "system"):
            _fail(api, "Moonraker service MainPID/process identity is ambiguous")
    elif active not in ("inactive", "failed") or pid != 0:
        _fail(api, "Moonraker is starting or cannot be established as stopped")
    commands = re.findall(r"(?:^|;\s*)argv\[\]=(.*?)\s*;\s*ignore_errors=", properties.get("ExecStart", ""))
    if len(commands) != 1 or any(token in commands[0] for token in ("%", "\\x")):
        _fail(api, "cannot establish one effective Moonraker ExecStart")
    argv = shlex.split(commands[0])
    variables = [arg for arg in argv if "$" in arg]
    if variables:
        if (not current.get("pid") or not argv or argv[0] != current["executable"]
                or any(not re.fullmatch(r"\$[A-Za-z_]\w*|\$\{[A-Za-z_]\w*\}", value) for value in variables)):
            _fail(api, "environment-based Moonraker unit needs a matching live main process")
    else:
        effective = _moon_command(argv, properties.get("WorkingDirectory") or "/", api,
                                  path_environment=current.get("path_environment"),
                                  owner_home=current.get("owner_home"))
        if effective is None or any(effective[key] != current[key] for key in ("config", "data", "socket", "executable")):
            _fail(api, "effective Moonraker service command differs from the selected instance")
    # Loaded definitions and live arguments are both retained; no env expansion.
    return dict(instance=_fingerprint(current), unit={key: properties.get(key, "") for key in
                ("Id", "ExecStart", "WorkingDirectory", "FragmentPath", "DropInPaths", "User", "Group",
                 "Environment", "EnvironmentFiles", "MainPID", "InvocationID", "ActiveState")})


def assert_restart_safe(moon, selected, restart, api, check_files=True):
    api.restart_permission(restart)
    if service_identity(moon, selected, api, restart["systemctl"]) != restart["identity"]:
        _fail(api, "Moonraker service/process identity changed before writes/restart")
    if check_files:
        assert_identity(moon, selected, api)
    print(api.assert_idle(selected, require_process_identity=bool(selected.get("pid"))))


def record_activation(plan, selected, api, backup, status, **extra):
    # The ignored backup argument keeps older local callers compatible. This
    # fixed receipt stores current activation metadata, never configuration data.
    path = selected["config"].parent / ACTIVATION_RECEIPT
    if path.is_symlink() or (path.exists() and not path.is_file()):
        _fail(api, "refusing unsafe Moonraker activation receipt")
    current = extra.get("identity", {}).get("instance", plan["moon"])
    data = dict(version=1, status=status, service=plan["moon"]["service"], config=str(plan["moon"]["config"]),
                instance=instance_key(selected), registration_sha256=plan["context"]["registration_sha256"],
                process={key: current.get(key) for key in ("pid", "start_time", "service", "service_scope")})
    data.update({key: str(extra[key]) for key in ("reason", "error") if key in extra})
    api.write_state(path, dict(kind="file", mode=0o600,
                    data=base64.b64encode((json.dumps(data, indent=2) + "\n").encode()).decode()))
    return path


def print_activation_advice(context, selected, api):
    moon = discover_moonraker(selected, api, context["moon_config"])
    service = moon.get("service")
    if service and api.SERVICE_NAME.fullmatch(service):
        command = ("systemctl --user restart -- " if moon.get("service_scope") == "user"
                   else "sudo systemctl restart -- ") + shlex.quote(service)
        print("Moonraker registration changed. While this printer is idle, restart its detected Moonraker "
              "service from the UI or run: %s" % command)
    else:
        print("Moonraker registration changed in %s. Restart that actual Moonraker host process once; "
              "browser refresh alone does not reload configuration." % context["moon_config"])


def activate(plan, selected, restart, api, backup):
    path = record_activation(plan, selected, api, backup, "pending")
    try:
        assert_restart_safe(plan["moon"], selected, restart, api, check_files=False)
        for destination, expected in plan["changes"].items():
            state = dict(expected)
            state.pop("expected", None)
            if state["kind"] == "file":
                state["data"] = base64.b64encode(state["data"]).decode()
            if not api.state_matches(destination, state):
                _fail(api, "installed updater files changed before activation")
        result = api.service_command([restart["sudo"], "-n", "-u", "root", "--"] + restart["service_command"], timeout=60)
        if result.returncode:
            _fail(api, "Moonraker restart command failed: %s" % result.stderr.strip())
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                current = service_identity(plan["moon"], selected, api, restart["systemctl"])
                old = restart["identity"]["instance"]
                if (current["instance"].get("pid") and current["instance"]["config"] == old["config"]
                        and current["instance"]["socket"] == old["socket"]
                        and (current["instance"]["pid"], current["instance"]["start_time"]) !=
                            (old.get("pid"), old.get("start_time"))):
                    record_activation(plan, selected, api, backup, "restarted", identity=current)
                    print("Moonraker updater registration activated: %s" % plan["moon"]["service"])
                    return
            except api.InstallError:
                pass
            time.sleep(0.2)
        _fail(api, "Moonraker did not return with a new verified process")
    except Exception as exc:
        record_activation(plan, selected, api, backup, "failed", error=str(exc))
        _fail(api, "files are installed but Moonraker updater activation remains pending; Klipper activation "
              "has not run. Activation receipt: %s. Cause: %s" % (path, exc))
