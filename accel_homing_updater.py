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
    # A fixed path also keeps journal validation scoped to the documented clone.
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


def stanza(repo):
    return ("# Managed by the accelerometer homing installer.\n"
            "[update_manager accel_homing]\n"
            "type: git_repo\nchannel: dev\npath: %s\norigin: %s\nprimary_branch: %s\n"
            "is_system_service: False\nmanaged_services: klipper\n" % (repo, ORIGIN, BRANCH)).encode()


def main_suffix(add_base, add_include):
    # Keep the original bytes for existing include-layout transaction journals.
    text = "\n# accel_homing managed Moonraker update registration\n"
    if add_base:
        text += "[update_manager]\n"
    if add_include:
        text += "[include %s]\n" % INCLUDE
    return text.encode() if add_base or add_include else b""


def direct_suffix(repo, add_base, add_stanza):
    if not add_base and not add_stanza:
        return b""
    text = b"\n# accel_homing managed Moonraker update registration\n"
    if add_base:
        text += b"[update_manager]\n"
    return text + (stanza(repo) if add_stanza else b"")


def matching_stanza(section, repo, api):
    expected = api.ConfigDocument(section.document.path, stanza(repo)).sections[0]
    header = section.document.lines[section.start]
    if (not header.startswith("[") or api.SECTION.match(header).group(1) != SECTION
            or set(section.options) != set(expected.options)):
        return False
    for line in section.document.lines[section.start + 1:section.end]:
        if line.strip() and not line.lstrip().startswith(("#", ";")):
            match = api.OPTION.match(line)
            if not match or match.group(1):
                return False
    return all(section.get(name) == expected.get(name) for name in expected.options)


def journal_layout(context, selected, api):
    common = {"moon_config", "moon_data", "moon_socket", "managed_repo", "origin", "branch", "append"}
    direct = isinstance(context, dict) and context.get("layout") == "direct"
    keys = common | ({"layout"} if direct else {"include_file"})
    if not isinstance(context, dict) or set(context) != keys:
        _fail(api, "invalid updater journal context")
    repo = managed_path(selected)
    if not isinstance(context["moon_config"], str):
        _fail(api, "invalid updater journal main configuration")
    main = Path(context["moon_config"])
    include = main.parent / INCLUDE
    if (not main.is_absolute() or (not direct and context["include_file"] != str(include))
            or context["managed_repo"] != str(repo) or context["origin"] != ORIGIN or context["branch"] != BRANCH
            or context["moon_socket"] != str(selected["socket"])):
        _fail(api, "updater journal is outside selected instance ownership")
    try:
        suffix = base64.b64decode(context["append"], validate=True)
    except (ValueError, TypeError):
        _fail(api, "invalid updater main-config journal")
    suffixes = ({direct_suffix(repo, base, entry) for base in (False, True) for entry in (False, True)}
                if direct else {main_suffix(base, inc) for base in (False, True) for inc in (False, True)})
    if suffix not in suffixes:
        _fail(api, "unowned Moonraker main-config edit")
    return dict(keys=keys, main=main, include=include, suffix=suffix, direct=direct)


def receipt_context(value, selected, api):
    if not isinstance(value, dict):
        _fail(api, "unowned updater receipt contents")
    context = {key: item for key, item in value.items() if key not in ("version", "instance", "commit")}
    layout = journal_layout(context, selected, api)
    if (set(value) != {"version", "instance", "commit"} | layout["keys"]
            or value["version"] != 1 or value["instance"] != instance_key(selected)
            or not isinstance(value["commit"], str) or not re.fullmatch(r"[0-9a-f]{40}", value["commit"])):
        _fail(api, "unowned updater receipt contents")
    return context


def activation_needed(selected, context, api):
    _records, active, _receipt = api.load_uninstaller().history(selected["config"], api.uninstall_api())
    for record in reversed(active):
        if record["manifest"].get("updater", {}).get("managed_repo") != context["managed_repo"]:
            continue
        path = record["directory"] / "moonraker-activation.json"
        if not path.exists():
            continue
        if path.is_symlink():
            _fail(api, "refusing symlink Moonraker activation receipt")
        value = json.loads(path.read_text())
        return not (value.get("status") == "restarted" and value.get("config") == context["moon_config"]
                    and value.get("instance") == instance_key(selected)
                    and value.get("registration_sha256") == hashlib.sha256(stanza(Path(context["managed_repo"]))).hexdigest())
    return True


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


def reject_legacy_installation(selected, api):
    """Reject old updater ownership before attempting native history validation."""
    def legacy(value):
        return isinstance(value, dict) and (value.get("origin") == LEGACY_ORIGIN
                                             or value.get("branch") == LEGACY_BRANCH)
    receipt = selected["config"].parent / RECEIPT
    contexts = []
    if receipt.is_file() and not receipt.is_symlink():
        try:
            contexts.append(json.loads(receipt.read_text()))
        except (OSError, ValueError):
            pass  # The ordinary receipt validator reports unknown ownership.
    _records, active, _receipt = api.load_uninstaller().history(selected["config"], api.uninstall_api())
    contexts.extend(record["manifest"].get("updater") for record in active)
    if any(legacy(context) for context in contexts):
        _fail(api, "legacy MISC updater installation detected. Back up custom thresholds and SAVE_CONFIG "
              "models, then run the previous installation's uninstall.sh successfully before installing "
              "this standalone clone. Keep its old source and backups until that uninstall succeeds. "
              "No printer or Moonraker files changed.")


def migration_args(selected, args, api):
    reject_legacy_installation(selected, api)
    result = copy.copy(args)
    if args.replace_existing_links:
        return result
    expected = api.load_uninstaller().validate_migration(selected, api.uninstall_api())
    if expected:
        # Archive ownership and exact current states authorize only these links;
        # current user configuration need not be invertible as an uninstall.
        result._migration_expected = expected
        result.replace_existing_links = True
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


def make_plan(source, selected, args, api):
    reject_legacy_installation(selected, api)
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
    include = moon["config"].parent / INCLUDE
    graph, changes = moon["graph"], {}
    original = moon["config"].read_bytes()
    receipt = selected["config"].parent / RECEIPT
    receipt_state = api.snapshot(receipt)
    previous = None
    if receipt_state["kind"] != "absent":
        if receipt_state["kind"] != "file":
            _fail(api, "refusing unsafe updater receipt")
        previous = receipt_context(json.loads(base64.b64decode(receipt_state["data"])), selected, api)
        if any(previous[key] != str(moon[name]) for key, name in
               (("moon_config", "config"), ("moon_data", "data"), ("moon_socket", "socket"))):
            _fail(api, "existing updater receipt belongs to another Moonraker instance")
        owned = base64.b64decode(previous["append"])
        if owned and original.count(owned) != 1:
            _fail(api, "owned Moonraker main-config registration changed/missing; preserve/reconcile it before updating")
    existing = graph.one(SECTION, False)
    direct = True
    if existing:
        if existing.document.path == include and include.read_bytes() == stanza(repo):
            direct = False
        elif existing.document.path != moon["config"] or not matching_stanza(existing, repo, api):
            _fail(api, "conflicting existing [%s]; preserve/reconcile it instead of overwriting" % SECTION)
    elif previous:
        _fail(api, "owned Moonraker updater registration is missing; preserve/reconcile it before updating")
    if direct and (include.exists() or include.is_symlink()):
        _fail(api, "%s exists without the unique managed updater stanza" % include)
    if previous and (previous.get("layout") == "direct") != direct:
        _fail(api, "existing updater layout changed; preserve its recorded registration instead of migrating it")
    base = graph.one("update_manager", False)
    if base:
        for name in base.options:
            base.get(name)  # Reject duplicate base options without changing them.
    add_base = base is None
    suffix = direct_suffix(repo, add_base, not existing) if direct else main_suffix(add_base, False)
    if suffix and previous and previous["append"]:
        _fail(api, "Moonraker base updater configuration changed; reconcile it before updating")
    if suffix:
        changes[moon["config"]] = dict(kind="file", data=original + suffix)
    context = dict(moon_config=str(moon["config"]), moon_data=str(moon["data"]),
                   moon_socket=str(moon["socket"]), managed_repo=str(repo), origin=ORIGIN,
                   branch=BRANCH, append=base64.b64encode(suffix).decode())
    context.update(dict(layout="direct") if direct else dict(include_file=str(include)))
    if previous and not suffix:
        context["append"] = previous["append"]
    encoded = (json.dumps(dict(version=1, instance=instance_key(selected), commit=commit,
                               **context), sort_keys=True, indent=2) + "\n").encode()
    if receipt_state.get("data") != base64.b64encode(encoded).decode():
        changes[receipt] = dict(kind="file", data=encoded)
    for path, change in changes.items():
        change["expected"] = receipt_state if path == receipt else api.snapshot(path)
    pending = activation_needed(selected, context, api)
    return dict(moon=moon, repo=repo, source=repo, commit=commit, source_identity=identity, selected=selected,
                changes=changes, context=context, source_input=source, published=False,
                restart_needed=any(path != receipt for path in changes) or pending, receipt=receipt)


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
    if origin != ORIGIN or branch != BRANCH or upstream != "origin/" + BRANCH:
        _fail(api, "managed checkout origin/branch/upstream differs; nothing was reset or stashed")
    if git(repo, ["status", "--porcelain=v1", "--untracked-files=all"], api):
        _fail(api, "managed checkout is dirty; preserve/reconcile those changes before updating")
    commit = git(repo, ["rev-parse", "HEAD"], api)
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        _fail(api, "standalone checkout did not return a full commit SHA")
    git(repo, ["merge-base", "--is-ancestor", commit, "origin/" + BRANCH], api)
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


def source_metadata(plan):
    return dict(repo=str(plan["repo"]), commit=plan["commit"],
                old_commit=None, backup=None, published=False)


def validate_source_rollback(manifest, selected, api):
    value = manifest.get("source_update")
    if value is None:
        return None
    if (not isinstance(value, dict) or set(value) != {"repo", "commit", "old_commit", "backup", "published"}
            or value["repo"] != str(managed_path(selected)) or type(value["published"]) is not bool
            or not re.fullmatch(r"[0-9a-f]{40}", value.get("commit", ""))):
        _fail(api, "invalid source-update rollback ownership")
    if not value["published"]:
        if value["backup"] is not None or value["old_commit"] is not None:
            _fail(api, "invalid caller-owned source rollback metadata")
        return None
    _fail(api, "legacy source publication requires its previous installer for rollback")


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
    if backup is None:
        _records, active, _receipt = api.load_uninstaller().history(selected["config"], api.uninstall_api())
        if not active:
            _fail(api, "no owned installation transaction for the activation receipt")
        backup = active[-1]["directory"]
    path = backup / "moonraker-activation.json"
    data = dict(status=status, service=plan["moon"]["service"], config=str(plan["moon"]["config"]),
                instance=instance_key(selected), registration_sha256=hashlib.sha256(stanza(plan["repo"])).hexdigest(), **extra)
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
        result = api.service_command([restart["sudo"], "-n", "--"] + restart["service_command"], timeout=60)
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
              "has not run. Backup/activation receipt: %s. Cause: %s" % (path, exc))


def validate_record(manifest, selected, api, verify_live=True):
    context = manifest.get("updater")
    if context is None:
        return {}
    layout = journal_layout(context, selected, api)
    repo = managed_path(selected)
    main, include, suffix = layout["main"], layout["include"], layout["suffix"]
    if verify_live:
        moon = discover_moonraker(selected, api, main)
        if str(moon["data"]) != context["moon_data"]:
            _fail(api, "updater journal Moonraker data path differs from actual instance")
    allowed = {main: "moon_main", selected["config"].parent / RECEIPT: "updater_receipt"}
    if not layout["direct"]:
        allowed[include] = "moon_generated"
    checked = {}
    for entry in manifest["changes"]:
        path = Path(entry["path"])
        if path not in allowed:
            continue
        before, after = entry["before"], entry["after"]
        if manifest.get("operation") == "uninstall":
            before, after = after, before
        if path == include:
            if before["kind"] != "absent" or after["kind"] != "file" or base64.b64decode(after["data"]) != stanza(repo):
                _fail(api, "unowned generated Moonraker updater edit")
        elif path == main:
            if not suffix or before["kind"] != "file" or after["kind"] != "file":
                _fail(api, "unsafe Moonraker main configuration journal")
            old, new = base64.b64decode(before["data"]), base64.b64decode(after["data"])
            if manifest.get("operation") == "uninstall":
                if new.count(suffix) != 1 or new.replace(suffix, b"", 1) != old:
                    _fail(api, "uninstall contains unowned Moonraker text changes")
            elif new != old + suffix:
                _fail(api, "updater journal changes unrelated Moonraker settings")
        else:
            for state in (before, after):
                if state["kind"] == "absent":
                    continue
                if state["kind"] != "file":
                    _fail(api, "unsafe updater receipt journal")
                value = json.loads(base64.b64decode(state["data"]))
                value_context = receipt_context(value, selected, api)
                if set(value_context) != layout["keys"]:
                    _fail(api, "unowned updater receipt contents")
        checked[path] = allowed[path]
    return checked
