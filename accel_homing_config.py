# Narrow calibrated-threshold persistence support. Distributed under GNU GPLv3.
"""Defer included threshold replacement to the public SAVE_CONFIG command.

Only successfully staged selected-axis keys are registered. The native host
owns main-file serialization and restart; this helper only comments those keys
in included files. It never enables Kalico's global autosave_includes policy.
"""
import configparser
import inspect
import io
import os
import re
import stat
import sys
import tempfile
import types


MAX_CONFIG_FILES = 256
MAX_CONFIG_BYTES = 16 * 1024 * 1024
MAX_INCLUDE_DEPTH = 32
_AUTOSAVE_MARKER = "#*# <---------------------- SAVE_CONFIG ---------------------->"
_COMMENT = re.compile(r"[#;].*$")
_OPTION = re.compile(r"(?P<option>.*?)\s*(?:=|:)\s*.*$")


def _parser():
    return configparser.RawConfigParser(strict=False, inline_comment_prefixes=(";", "#"))


def _identity(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns,
            info.st_ctime_ns, stat.S_IMODE(info.st_mode))


def _snapshot(path):
    logical = os.path.abspath(path)
    resolved = os.path.realpath(logical)
    with open(resolved, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ValueError("configuration path is not a regular file: %s" % logical)
        data = stream.read(MAX_CONFIG_BYTES + 1)
    if len(data) > MAX_CONFIG_BYTES:
        raise ValueError("configuration file exceeds the bounded read limit")
    return {"path": resolved, "aliases": {logical}, "data": data,
            "identity": _identity(info), "links": info.st_nlink}


def _matches(snapshot):
    try:
        if any(os.path.realpath(alias) != snapshot["path"] for alias in snapshot["aliases"]):
            return False
        current = _snapshot(snapshot["path"])
        return current["identity"] == snapshot["identity"] and current["data"] == snapshot["data"]
    except (OSError, ValueError):
        return False


def _replace(snapshot, data):
    """Same-directory atomic replacement with checks immediately before rename."""
    if snapshot["links"] != 1:
        raise ValueError("cannot replace a hard-linked included configuration: %s" % snapshot["path"])
    if not _matches(snapshot):
        raise ValueError("included configuration changed before save: %s" % snapshot["path"])
    descriptor, temporary = tempfile.mkstemp(prefix=".accel_homing_save_", suffix=".tmp",
                                              dir=os.path.dirname(snapshot["path"]))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            if hasattr(os, "fchmod"):
                os.fchmod(stream.fileno(), snapshot["identity"][-1])
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if not hasattr(os, "fchmod"):
            os.chmod(temporary, snapshot["identity"][-1])
        if not _matches(snapshot):
            raise ValueError("included configuration changed during save: %s" % snapshot["path"])
        os.replace(temporary, snapshot["path"])
        try:
            current = _snapshot(snapshot["path"])
        except (OSError, ValueError):
            # The replacement happened; retain it in rollback accounting even
            # when a concurrent edit/removal prevents observing its identity.
            current = dict(snapshot, identity=None)
        current["aliases"] = set(snapshot["aliases"])
        # Expected bytes, not a concurrent user's bytes. A mismatch makes the
        # next verification fail and prevents rollback from replacing that edit.
        current["data"] = data
        return current
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _comment_selected(data, keys):
    # Retain every original byte/newline except '#' on selected fields and
    # their continuation lines. ConfigParser also permits indented options;
    # greater indentation continues the previous value instead of a new key.
    text = data.decode("utf-8")
    lines = text.split("\n")
    section, duplicate, option_indent = None, False, None
    for index, line in enumerate(lines):
        pruned = _COMMENT.sub("", line).rstrip()
        if not pruned:
            continue
        content = pruned.lstrip()
        indent = len(pruned) - len(content)
        if option_indent is not None and indent > option_indent:
            if duplicate:
                lines[index] = "#" + line
            continue
        duplicate = False
        header = configparser.RawConfigParser.SECTCRE.match(content)
        if header:
            section, option_indent = header.group("header"), None
            continue
        option = _OPTION.match(content)
        if option is None:
            continue
        option_indent = indent
        if (section, option.group("option").rstrip().lower()) in keys:
            duplicate = True
            lines[index] = "#" + line
    return "\n".join(lines).encode("utf-8")


class _NativeReader:
    """Read-only adapters for the retained Klipper and Kalico native layouts."""
    def __init__(self, configfile):
        if callable(getattr(configfile, "_resolve_include", None)):
            self.reader = configfile
            self.find_autosave = configfile._find_autosave_data
            self.strip_duplicates = lambda data, parser: configfile._strip_duplicates(
                data, types.SimpleNamespace(fileconfig=parser))
        else:
            autosave = getattr(configfile, "autosave", None)
            module = sys.modules.get(type(autosave).__module__)
            reader_type = getattr(module, "ConfigFileReader", None)
            if not callable(reader_type) or not callable(getattr(autosave, "_find_autosave_data", None)):
                raise ValueError("native SAVE_CONFIG include-reader API is unsupported")
            self.reader = reader_type()
            self.find_autosave = autosave._find_autosave_data
            self.strip_duplicates = autosave._strip_duplicates
        if not callable(getattr(self.reader, "_resolve_include", None)):
            raise ValueError("native SAVE_CONFIG include resolver is unavailable")
        try:
            signature = inspect.signature(self.reader._resolve_include)
            self.include_keywords = {"include_vars": None} if "include_vars" in signature.parameters else {}
            signature.bind(None, None, None, None, **self.include_keywords)
        except (TypeError, ValueError) as exc:
            raise ValueError("native SAVE_CONFIG include resolver signature is unsupported") from exc

    def included(self, filename, specification):
        # Native resolution includes sorted globs and nested syntax validation.
        # Kalico enables recursive '**'; retained Klipper does not.
        return self.reader._resolve_include(filename, specification, _parser(), set(), **self.include_keywords)

    def check_plain_includes(self, data, filename):
        if not self.include_keywords:
            return
        pending = None
        for line in data.split("\n"):
            line = line.split("#", 1)[0]
            match = configparser.RawConfigParser.SECTCRE.match(line)
            header = match and match.group("header")
            if header:
                pending = header[8:].strip() if header.startswith("include ") else None
            elif pending and line.strip():
                # Native Kalico renders the included file with these variables
                # and inherits them in nested includes. Literal-file edits
                # cannot safely reproduce that generated configuration graph.
                raise ValueError("parameterized include [%s] in %s is unsupported for calibrated SAVE_CONFIG; "
                                 "use plain includes before retrying; no files were changed" %
                                 ("include " + pending, filename))

    def checked_root_data(self, data):
        """Reject an unreadable existing envelope before any save-side edits."""
        text = data.decode("utf-8").replace("\r\n", "\n")
        regular, generated = self.find_autosave(text)
        lines = text.split("\n")
        # Native rejection returns an empty suffix. Both retained hosts return
        # newline-only data for valid empty/no blocks; also allow compatible
        # readers to return empty for ordinary files with no reserved lines.
        # Kalico may accept contiguous repeated headers, unlike Klipper.
        if (lines.count(_AUTOSAVE_MARKER) > 1
                or (generated == "" and any(line.startswith("#*# ") for line in lines))):
            raise ValueError("existing malformed SAVE_CONFIG block; no files were changed; "
                             "repair or restore the existing autosave block before retrying")
        return regular

    def check_root_targets(self, data, targets):
        """Native serialization must actually remove selected root literals.

        Some native versions leave indented options untouched and Kalico may
        accept an equal-valued duplicate. That would discard generated origin
        on reload, so reject even equal values instead of rewriting the root.
        """
        regular, _generated = self.find_autosave(data.decode("utf-8").replace("\r\n", "\n"))
        proposed = _parser()
        for (section, option), target in targets.items():
            if not proposed.has_section(section):
                proposed.add_section(section)
            proposed.set(section, option, target["value"])
        remaining = _parser()
        remaining.read_string(self.strip_duplicates(regular, proposed))
        conflicts = ["[%s] %s" % key for key in targets if remaining.has_option(*key)]
        if conflicts:
            raise ValueError("native SAVE_CONFIG cannot comment selected main-file thresholds %s; "
                             "remove indentation from those option lines before saving" % ", ".join(conflicts))


class CalibrationConfigSave:
    def __init__(self, printer, section):
        if section not in ("accel_homing", "beacon_accel_homing"):
            raise ValueError("unsupported calibrated threshold section")
        self.printer, self.section = printer, section
        self._targets = {}
        self._original = None
        self._saving = False

    def _error(self, message):
        return self.printer.command_error("accel_homing: " + message)

    def _install(self):
        if self._original is not None:
            return
        gcode = self.printer.lookup_object("gcode")
        original = gcode.register_command("SAVE_CONFIG", None)
        if not callable(original):
            raise self._error("native SAVE_CONFIG handler is unavailable; calibration was not staged")
        try:
            gcode.register_command("SAVE_CONFIG", self.cmd_SAVE_CONFIG)
        except Exception:
            gcode.register_command("SAVE_CONFIG", original)
            raise
        self._original = original

    def register_threshold(self, axis, encoded_value):
        return self.register_settings(axis, [(self.section, axis + "_threshold", encoded_value)])

    def register_settings(self, axis, entries, model_section=None):
        """Register only the qualified axis and model; never write files here."""
        allowed = {(self.section, axis + suffix) for suffix in
                   ("_threshold", "_notch_frequency", "_notch_q")}
        if model_section is not None:
            if not model_section.startswith("accel_homing_model "):
                raise ValueError("unsupported homing model section")
            allowed.update((model_section, key) for key in
                           ("version", axis + "_threshold", axis + "_successes", axis + "_metadata",
                            ("y" if axis == "x" else "x") + "_metadata"))
        if (axis not in ("x", "y") or not entries or any(
                (section, option) not in allowed or not isinstance(value, str)
                for section, option, value in entries)):
            raise ValueError("register only qualified homing axis settings")
        self._install()
        previous, current = {}, {}
        for section, option, value in entries:
            key = (section, option)
            previous[key] = self._targets.get(key)
            current[key] = {"value": value}
            self._targets[key] = current[key]

        def rollback():
            for key, target in current.items():
                if self._targets.get(key) is target:
                    if previous[key] is None:
                        self._targets.pop(key, None)
                    else:
                        self._targets[key] = previous[key]
        return rollback

    def _pending(self, targets):
        configfile = self.printer.lookup_object("configfile")
        autosave = getattr(configfile, "autosave", None)
        parser = getattr(autosave, "fileconfig", None)
        state = (configfile if hasattr(configfile, "status_save_pending") else autosave)
        pending = getattr(state, "status_save_pending", None)
        if not isinstance(parser, configparser.RawConfigParser) or not isinstance(pending, dict):
            raise ValueError("native SAVE_CONFIG pending-state API is unsupported")
        for (section, option), target in targets.items():
            staged = pending.get(section)
            if (not isinstance(staged, dict) or staged.get(option) != target["value"]
                    or not parser.has_option(section, option)
                    or parser.get(section, option, raw=True) != target["value"]):
                raise ValueError("calibrated [%s] %s changed after staging; no include edits made" % (section, option))
        # Native serialization normalizes continuation indentation (Beacon
        # model coefficients are a common unrelated multiline autosave value).
        serialized = io.StringIO()
        parser.write(serialized)
        canonical = _parser()
        canonical.read_string(serialized.getvalue())
        expected = {section: dict(canonical.items(section, raw=True)) for section in canonical.sections()}
        return configfile, expected

    def _configuration(self, reader, filename):
        root = _snapshot(filename)
        regular_root = reader.checked_root_data(root["data"])
        files, edges, total = {root["path"]: root}, [], [len(root["data"])]
        visited, parsed = set(), set()

        def walk(logical, snapshot, depth):
            logical = os.path.abspath(logical)
            if depth > MAX_INCLUDE_DEPTH or logical in visited:
                raise ValueError("recursive or excessively deep configuration includes")
            if logical in parsed:
                return
            visited.add(logical)
            data = snapshot["data"].decode("utf-8")
            if depth == 0:
                data = regular_root
            reader.check_plain_includes(data, logical)
            for line in data.split("\n"):
                line = line.split("#", 1)[0]
                match = configparser.RawConfigParser.SECTCRE.match(line)
                header = match and match.group("header")
                if not header or not header.startswith("include "):
                    continue
                specification = header[8:].strip()
                names = tuple(reader.included(logical, specification))
                edges.append((logical, specification, names))
                for name in names:
                    child = _snapshot(name)
                    previous = files.get(child["path"])
                    if previous is None:
                        total[0] += len(child["data"])
                        if len(files) >= MAX_CONFIG_FILES or total[0] > MAX_CONFIG_BYTES:
                            raise ValueError("configuration includes exceed bounded save limits")
                        files[child["path"]] = child
                    else:
                        if previous["data"] != child["data"] or previous["identity"] != child["identity"]:
                            raise ValueError("configuration changed while resolving includes")
                        previous["aliases"].update(child["aliases"])
                        child = previous
                    walk(name, child, depth + 1)
            visited.remove(logical)
            parsed.add(logical)

        walk(filename, root, 0)
        return root, files, edges

    @staticmethod
    def _verify_configuration(reader, files, edges):
        if any(not _matches(snapshot) for snapshot in files.values()):
            raise ValueError("configuration changed before native SAVE_CONFIG")
        if any(tuple(reader.included(filename, specification)) != names
               for filename, specification, names in edges):
            raise ValueError("configuration include matches changed before native SAVE_CONFIG")

    @staticmethod
    def _persisted(reader, filename, expected):
        _regular, generated = reader.find_autosave(_snapshot(filename)["data"].decode("utf-8").replace("\r\n", "\n"))
        parsed = _parser()
        parsed.read_string(generated)
        return bool(generated.strip()) and all(
            parsed.has_option(section, option) and parsed.get(section, option, raw=True) == value
            for section, values in expected.items() for option, value in values.items())

    @staticmethod
    def _rollback(edits):
        conflicts = []
        for before, after in reversed(edits):
            if not _matches(after):
                conflicts.append(before["path"])
                continue
            try:
                _replace(after, before["data"])
            except (OSError, ValueError):
                conflicts.append(before["path"])
        return conflicts

    def cmd_SAVE_CONFIG(self, gcmd):
        if not self._targets:
            return self._original(gcmd)
        if self._saving:
            raise self._error("recursive SAVE_CONFIG while calibrated thresholds are pending")
        self._saving = True
        targets, edits, root = dict(self._targets), [], None
        try:
            configfile, expected = self._pending(targets)
            reader = _NativeReader(configfile)
            filename = self.printer.get_start_args()["config_file"]
            root, files, edges = self._configuration(reader, filename)
            reader.check_root_targets(root["data"], targets)
            self._verify_configuration(reader, files, edges)
            for path, before in list(files.items()):
                if path == root["path"]:
                    continue  # The native serializer owns the main file.
                commented = _comment_selected(before["data"], targets)
                if commented != before["data"]:
                    after = _replace(before, commented)
                    edits.append((before, after))
                    files[path] = after
            self._verify_configuration(reader, files, edges)
            self._pending(targets)
            result = self._original(gcmd)
            if _matches(root) or not self._persisted(reader, filename, expected):
                raise ValueError("SAVE_CONFIG handler did not confirm the staged native configuration on disk")
            for key, target in targets.items():
                if self._targets.get(key) is target:
                    del self._targets[key]
            return result
        except Exception as exc:
            if root is not None and not _matches(root):
                raise self._error("SAVE_CONFIG failed after the main configuration changed; included calibrated "
                                  "thresholds remain commented and persistence/restart state is unconfirmed: %s" % exc) from exc
            conflicts = self._rollback(edits)
            if conflicts:
                raise self._error("SAVE_CONFIG failed; include rollback could not replace changed files %s; "
                                  "concurrent edits were preserved: %s" % (", ".join(conflicts), exc)) from exc
            raise self._error("SAVE_CONFIG failed; calibrated include edits were restored: %s" % exc) from exc
        finally:
            self._saving = False
