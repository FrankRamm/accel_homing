"""Pinned native config reader/save excerpts for offline integration tests.

Klipper3d/klipper af81d23fec795ba64882d2206bc9cbc019e4501b,
klippy/configfile.py, Copyright (C) 2016-2024 Kevin O'Connor.
KalicoCrew/kalico 107302c468c2e692e7bcb1e8c66032782c4ad555, klippy/configfile.py,
Copyright (C) 2016-2021 Kevin O'Connor.
https://github.com/Klipper3d/klipper/blob/af81d23fec795ba64882d2206bc9cbc019e4501b/klippy/configfile.py
https://github.com/KalicoCrew/kalico/blob/107302c468c2e692e7bcb1e8c66032782c4ad555/klippy/configfile.py
GNU GPLv3; upstream method bodies below are unchanged. Only class selection,
Kalico's class name and explicit firmware setup stubs differ. No MCU is used.
"""
import sys, os, glob, re, time, logging, configparser, io, pathlib
import types
error = configparser.Error
AUTOSAVE_INCLUDES = False
def get_danger_options():
    return types.SimpleNamespace(autosave_includes=AUTOSAVE_INCLUDES)


class ConfigWrapper:
    error = configparser.Error

    def __init__(self, printer, fileconfig, access_tracking, section):
        self.printer = printer
        self.fileconfig = fileconfig
        self.access_tracking = access_tracking
        self.section = section


class ConfigFileReader:
    def read_config_file(self, filename):
        try:
            f = open(filename, 'r')
            data = f.read()
            f.close()
        except:
            msg = "Unable to open config file %s" % (filename,)
            logging.exception(msg)
            raise error(msg)
        return data.replace('\r\n', '\n')
    def build_config_string(self, fileconfig):
        sfile = io.StringIO()
        fileconfig.write(sfile)
        return sfile.getvalue().strip()
    def append_fileconfig(self, fileconfig, data, filename):
        if not data:
            return
        # Strip trailing comments
        lines = data.split('\n')
        for i, line in enumerate(lines):
            pos = line.find('#')
            if pos >= 0:
                lines[i] = line[:pos]
        sbuffer = io.StringIO('\n'.join(lines))
        if sys.version_info.major >= 3:
            fileconfig.read_file(sbuffer, filename)
        else:
            fileconfig.readfp(sbuffer, filename)
    def _create_fileconfig(self):
        if sys.version_info.major >= 3:
            fileconfig = configparser.RawConfigParser(
                strict=False, inline_comment_prefixes=(';', '#'))
        else:
            fileconfig = configparser.RawConfigParser()
        return fileconfig
    def build_fileconfig(self, data, filename):
        fileconfig = self._create_fileconfig()
        self.append_fileconfig(fileconfig, data, filename)
        return fileconfig
    def _resolve_include(self, source_filename, include_spec, fileconfig,
                         visited):
        dirname = os.path.dirname(source_filename)
        include_spec = include_spec.strip()
        include_glob = os.path.join(dirname, include_spec)
        include_filenames = glob.glob(include_glob)
        if not include_filenames and not glob.has_magic(include_glob):
            # Empty set is OK if wildcard but not for direct file reference
            raise error("Include file '%s' does not exist" % (include_glob,))
        include_filenames.sort()
        for include_filename in include_filenames:
            include_data = self.read_config_file(include_filename)
            self._parse_config(include_data, include_filename, fileconfig,
                               visited)
        return include_filenames
    def _parse_config(self, data, filename, fileconfig, visited):
        path = os.path.abspath(filename)
        if path in visited:
            raise error("Recursive include of config file '%s'" % (filename))
        visited.add(path)
        lines = data.split('\n')
        # Buffer lines between includes and parse as a unit so that overrides
        # in includes apply linearly as they do within a single file
        buf = []
        for line in lines:
            # Strip trailing comment
            pos = line.find('#')
            if pos >= 0:
                line = line[:pos]
            # Process include or buffer line
            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group('header')
            if header and header.startswith('include '):
                self.append_fileconfig(fileconfig, '\n'.join(buf), filename)
                del buf[:]
                include_spec = header[8:].strip()
                self._resolve_include(filename, include_spec, fileconfig,
                                      visited)
            else:
                buf.append(line)
        self.append_fileconfig(fileconfig, '\n'.join(buf), filename)
        visited.remove(path)
    def build_fileconfig_with_includes(self, data, filename):
        fileconfig = self._create_fileconfig()
        self._parse_config(data, filename, fileconfig, set())
        return fileconfig

AUTOSAVE_HEADER = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
"""

class ConfigAutoSave:
    def __init__(self, printer):
        self.printer = printer
        self.fileconfig = None
        self.status_save_pending = {}
        self.save_config_pending = False
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SAVE_CONFIG", self.cmd_SAVE_CONFIG,
                               desc=self.cmd_SAVE_CONFIG_help)

    def _find_autosave_data(self, data):
        regular_data = data
        autosave_data = ""
        pos = data.find(AUTOSAVE_HEADER)
        if pos >= 0:
            regular_data = data[:pos]
            autosave_data = data[pos + len(AUTOSAVE_HEADER):].strip()
        # Check for errors and strip line prefixes
        if "\n#*# " in regular_data or autosave_data.find(AUTOSAVE_HEADER) >= 0:
            logging.warning("Can't read autosave from config file"
                            " - autosave state corrupted")
            return data, ""
        out = [""]
        for line in autosave_data.split('\n'):
            if ((not line.startswith("#*#")
                 or (len(line) >= 4 and not line.startswith("#*# ")))
                and autosave_data):
                logging.warning("Can't read autosave from config file"
                                " - modifications after header")
                return data, ""
            out.append(line[4:])
        out.append("")
        return regular_data, "\n".join(out)

    comment_r = re.compile('[#;].*$')

    value_r = re.compile('[^A-Za-z0-9_].*$')

    def _strip_duplicates(self, data, fileconfig):
        # Comment out fields in 'data' that are defined in 'config'
        lines = data.split('\n')
        section = None
        is_dup_field = False
        for lineno, line in enumerate(lines):
            pruned_line = self.comment_r.sub('', line).rstrip()
            if not pruned_line:
                continue
            if pruned_line[0].isspace():
                if is_dup_field:
                    lines[lineno] = '#' + lines[lineno]
                continue
            is_dup_field = False
            if pruned_line[0] == '[':
                section = pruned_line[1:-1].strip()
                continue
            field = self.value_r.sub('', pruned_line)
            if fileconfig.has_option(section, field):
                is_dup_field = True
                lines[lineno] = '#' + lines[lineno]
        return "\n".join(lines)

    def load_main_config(self):
        filename = self.printer.get_start_args()['config_file']
        cfgrdr = ConfigFileReader()
        data = cfgrdr.read_config_file(filename)
        regular_data, autosave_data = self._find_autosave_data(data)
        regular_fileconfig = cfgrdr.build_fileconfig_with_includes(
            regular_data, filename)
        autosave_data = self._strip_duplicates(autosave_data,
                                               regular_fileconfig)
        self.fileconfig = cfgrdr.build_fileconfig(autosave_data, filename)
        cfgrdr.append_fileconfig(regular_fileconfig,
                                 autosave_data, '*AUTOSAVE*')
        return regular_fileconfig, self.fileconfig

    def set(self, section, option, value):
        if not self.fileconfig.has_section(section):
            self.fileconfig.add_section(section)
        svalue = str(value)
        self.fileconfig.set(section, option, svalue)
        pending = dict(self.status_save_pending)
        if not section in pending or pending[section] is None:
            pending[section] = {}
        else:
            pending[section] = dict(pending[section])
        pending[section][option] = svalue
        self.status_save_pending = pending
        self.save_config_pending = True
        logging.info("save_config: set [%s] %s = %s", section, option, svalue)

    def _disallow_include_conflicts(self, regular_fileconfig):
        for section in self.fileconfig.sections():
            for option in self.fileconfig.options(section):
                if regular_fileconfig.has_option(section, option):
                    msg = ("SAVE_CONFIG section '%s' option '%s' conflicts "
                           "with included value" % (section, option))
                    raise self.printer.command_error(msg)

    cmd_SAVE_CONFIG_help = "Overwrite config file and restart"

    def cmd_SAVE_CONFIG(self, gcmd):
        if not self.fileconfig.sections():
            return
        # Create string containing autosave data
        cfgrdr = ConfigFileReader()
        autosave_data = cfgrdr.build_config_string(self.fileconfig)
        lines = [('#*# ' + l).strip()
                 for l in autosave_data.split('\n')]
        lines.insert(0, "\n" + AUTOSAVE_HEADER.rstrip())
        lines.append("")
        autosave_data = '\n'.join(lines)
        # Read in and validate current config file
        cfgname = self.printer.get_start_args()['config_file']
        try:
            data = cfgrdr.read_config_file(cfgname)
        except error as e:
            msg = "Unable to read existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        regular_data, old_autosave_data = self._find_autosave_data(data)
        regular_data = self._strip_duplicates(regular_data, self.fileconfig)
        data = regular_data.rstrip() + autosave_data
        new_regular_data, new_autosave_data = self._find_autosave_data(data)
        if not new_autosave_data:
            raise gcmd.error(
                "Existing config autosave is corrupted."
                " Can't complete SAVE_CONFIG")
        try:
            regular_fileconfig = cfgrdr.build_fileconfig_with_includes(
                new_regular_data, cfgname)
        except error as e:
            msg = "Unable to parse existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        self._disallow_include_conflicts(regular_fileconfig)
        # Determine filenames
        datestr = time.strftime("-%Y%m%d_%H%M%S")
        backup_name = cfgname + datestr
        temp_name = cfgname + "_autosave"
        if cfgname.endswith(".cfg"):
            backup_name = cfgname[:-4] + datestr + ".cfg"
            temp_name = cfgname[:-4] + "_autosave.cfg"
        # Create new config file with temporary name and swap with main config
        logging.info("SAVE_CONFIG to '%s' (backup in '%s')",
                     cfgname, backup_name)
        try:
            f = open(temp_name, 'w')
            f.write(data)
            f.close()
            os.rename(cfgname, backup_name)
            os.rename(temp_name, cfgname)
        except:
            msg = "Unable to write config file during SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        # Request a restart
        gcode = self.printer.lookup_object('gcode')
        gcode.request_restart('restart')


class KlipperConfig:
    # The real modern PrinterConfig delegates persistence to ConfigAutoSave.
    def __init__(self, printer):
        self.printer = printer
        self.autosave = ConfigAutoSave(printer)
        self.autosave.fileconfig = ConfigFileReader().build_fileconfig("", "empty")
    def set(self, section, option, value):
        self.autosave.set(section, option, value)
    def read_main_config(self):
        regular, saved = self.autosave.load_main_config()
        return types.SimpleNamespace(fileconfig=regular)


_INCLUDERE = re.compile(r"!!include (?P<file>.*)")

def _fix_include_path(source_file: str, match: re.Match) -> pathlib.Path:
    new_path = pathlib.Path(source_file).parent.absolute() / match.group("file")
    if not new_path.is_file():
        raise error(f"Attempted to include non-existent file {new_path}")
    return f"!!include {new_path}"

class SectionInterpolation(configparser.Interpolation):
    """
    variable interpolation replacing ${[section.]option}
    """

    _KEYCRE = re.compile(
        r"(?<!\\)?\$\{(?:(?P<section>[^.:${}]+)[.:])?(?P<option>[^${}]+)\}"
    )

    def __init__(self, access_tracking):
        self.access_tracking = access_tracking

    def before_get(self, parser, section, option, value, defaults):
        if not isinstance(value, str):
            return value
        depth = configparser.MAX_INTERPOLATION_DEPTH
        while depth:
            depth -= 1

            match = self._KEYCRE.search(value)
            if not match:
                break

            sect = match.group("section") or section
            opt = match.group("option")

            const = parser.get(sect, opt)
            self.access_tracking.setdefault((sect, opt), const)

            value = value[: match.start()] + const + value[match.end() :]

        return value.replace("\\${", "${")

class KalicoConfig:
    def __init__(self, printer):
        self.printer = printer
        self.autosave = None
        self.deprecated = {}
        self.runtime_warnings = []
        self.deprecate_warnings = []
        self.status_raw_config = {}
        self.status_save_pending = {}
        self.status_settings = {}
        self.status_warnings = []
        self.unused_sections = []
        self.unused_options = []
        self.save_config_pending = False
        gcode = self.printer.lookup_object("gcode")
        if "SAVE_CONFIG" not in gcode.ready_gcode_handlers:
            gcode.register_command(
                "SAVE_CONFIG",
                self.cmd_SAVE_CONFIG,
                desc=self.cmd_SAVE_CONFIG_help,
            )

    def _read_config_file(self, filename):
        try:
            f = open(filename, "r")
            data = f.read()
            f.close()
        except:
            msg = "Unable to open config file %s" % (filename,)
            logging.exception(msg)
            raise error(msg)
        return data.replace("\r\n", "\n")

    def _find_autosave_data(self, data):
        regular_data = data
        autosave_data = ""
        pos = data.find(AUTOSAVE_HEADER)
        if pos >= 0:
            regular_data = data[:pos]
            autosave_data = data[pos + len(AUTOSAVE_HEADER) :].strip()
        # Check for errors and strip line prefixes
        if "\n#*# " in regular_data:
            logging.warning(
                "Can't read autosave from config file"
                " - autosave state corrupted"
            )
            return data, ""
        out = [""]
        for line in autosave_data.split("\n"):
            if (
                not line.startswith("#*#")
                or (len(line) >= 4 and not line.startswith("#*# "))
            ) and autosave_data:
                logging.warning(
                    "Can't read autosave from config file"
                    " - modifications after header"
                )
                return data, ""
            out.append(line[4:])
        out.append("")
        return regular_data, "\n".join(out)

    comment_r = re.compile("[#;].*$")

    value_r = re.compile("[^A-Za-z0-9_].*$")

    def _strip_duplicates(self, data, config):
        # Comment out fields in 'data' that are defined in 'config'
        lines = data.split("\n")
        section = None
        is_dup_field = False
        for lineno, line in enumerate(lines):
            pruned_line = self.comment_r.sub("", line).rstrip()
            if not pruned_line:
                continue
            if pruned_line[0].isspace():
                if is_dup_field:
                    lines[lineno] = "#" + lines[lineno]
                continue
            is_dup_field = False
            if pruned_line[0] == "[":
                section = pruned_line[1:-1].strip()
                continue
            field = self.value_r.sub("", pruned_line)
            if config.fileconfig.has_option(section, field):
                is_dup_field = True
                lines[lineno] = "#" + lines[lineno]
        return "\n".join(lines)

    def _parse_config_buffer(self, buffer, filename, fileconfig):
        if not buffer:
            return
        data = "\n".join(buffer)
        del buffer[:]
        sbuffer = io.StringIO(data)
        if sys.version_info.major >= 3:
            fileconfig.read_file(sbuffer, filename)
        else:
            fileconfig.readfp(sbuffer, filename)

    def _resolve_include(
        self, source_filename, include_spec, fileconfig, visited
    ):
        dirname = os.path.dirname(source_filename)
        include_spec = include_spec.strip()
        include_glob = os.path.join(dirname, include_spec)
        if sys.version_info >= (3, 5):
            include_filenames = glob.glob(include_glob, recursive=True)
        else:
            include_filenames = glob.glob(include_glob)
        if not include_filenames and not glob.has_magic(include_glob):
            # Empty set is OK if wildcard but not for direct file reference
            raise error("Include file '%s' does not exist" % (include_glob,))
        include_filenames.sort()
        for include_filename in include_filenames:
            include_data = self._read_config_file(include_filename)
            self._parse_config(
                include_data, include_filename, fileconfig, visited
            )
        return include_filenames

    def _parse_config(self, data, filename, fileconfig, visited):
        path = os.path.abspath(filename)
        if path in visited:
            raise error("Recursive include of config file '%s'" % (filename))
        visited.add(path)
        lines = data.split("\n")
        # Buffer lines between includes and parse as a unit so that overrides
        # in includes apply linearly as they do within a single file
        buffer = []
        for line in lines:
            # Strip trailing comment
            pos = line.find("#")
            if pos >= 0:
                line = line[:pos]
            # Process include or buffer line
            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group("header")
            if header and header.startswith("include "):
                self._parse_config_buffer(buffer, filename, fileconfig)
                include_spec = header[8:].strip()
                self._resolve_include(
                    filename, include_spec, fileconfig, visited
                )
            else:
                line = _INCLUDERE.sub(
                    lambda match: _fix_include_path(filename, match),
                    line,
                )
                buffer.append(line)
        self._parse_config_buffer(buffer, filename, fileconfig)
        visited.remove(path)

    def _build_config_wrapper(self, data, filename):
        access_tracking = {}
        fileconfig = configparser.RawConfigParser(
            strict=False,
            inline_comment_prefixes=(";", "#"),
            interpolation=SectionInterpolation(access_tracking),
        )

        self._parse_config(data, filename, fileconfig, set())
        return ConfigWrapper(
            self.printer, fileconfig, access_tracking, "printer"
        )

    def _build_config_string(self, config):
        sfile = io.StringIO()
        config.fileconfig.write(sfile)
        return sfile.getvalue().strip()

    def read_main_config(self):
        filename = self.printer.get_start_args()["config_file"]
        data = self._read_config_file(filename)
        regular_data, autosave_data = self._find_autosave_data(data)
        regular_config = self._build_config_wrapper(regular_data, filename)
        autosave_data = self._strip_duplicates(autosave_data, regular_config)
        self.autosave = self._build_config_wrapper(autosave_data, filename)
        cfg = self._build_config_wrapper(regular_data + autosave_data, filename)
        return cfg

    def set(self, section, option, value):
        if not self.autosave.fileconfig.has_section(section):
            self.autosave.fileconfig.add_section(section)
        svalue = str(value)
        self.autosave.fileconfig.set(section, option, svalue)
        pending = dict(self.status_save_pending)
        if section not in pending or pending[section] is None:
            pending[section] = {}
        else:
            pending[section] = dict(pending[section])
        pending[section][option] = svalue
        self.status_save_pending = pending
        self.save_config_pending = True
        logging.info("save_config: set [%s] %s = %s", section, option, svalue)

    def _disallow_include_conflicts(self, regular_data, cfgname, gcode):
        config = self._build_config_wrapper(regular_data, cfgname)
        for section in self.autosave.fileconfig.sections():
            for option in self.autosave.fileconfig.options(section):
                if config.fileconfig.has_option(section, option):
                    # They conflict only if they are not the same value
                    included_value = config.fileconfig.get(section, option)
                    autosave_value = self.autosave.fileconfig.get(
                        section, option
                    )
                    if included_value != autosave_value:
                        msg = (
                            "SAVE_CONFIG section '%s' option '%s' value '%s' conflicts "
                            "with included value '%s' "
                            % (section, option, autosave_value, included_value)
                        )
                        raise gcode.error(msg)

    cmd_SAVE_CONFIG_help = "Overwrite config file and restart"

    def _write_backup(self, cfgpath, cfgdata, gcode):
        printercfg = self.printer.get_start_args()["config_file"]
        configdir = os.path.dirname(printercfg)
        # Define a directory for configuration backups so that include blocks
        # using a wildcard to reference all files in a directory don't throw
        # errors
        backupdir = os.path.join(configdir, "config_backups")
        # Create the backup directory if it doesn't already exist
        if not os.path.exists(backupdir):
            os.mkdir(backupdir)

        # Generate the name of the backup file by stripping the leading path in
        # `cfgpath` and appending to it. Then add it to the config_backups dir
        datestr = time.strftime("-%Y%m%d_%H%M%S")
        cfgname = os.path.basename(cfgpath)
        backup_path = backupdir + "/" + cfgname + datestr
        if cfgpath.endswith(".cfg"):
            backup_path = backupdir + "/" + cfgname[:-4] + datestr + ".cfg"
        logging.info(
            "SAVE_CONFIG to '%s' (backup in '%s')", cfgpath, backup_path
        )
        try:
            # Read the current config into the backup before making changes to
            # the original file
            currentconfig = open(cfgpath, "r")
            backupconfig = open(backup_path, "w")
            backupconfig.write(currentconfig.read())
            backupconfig.close()
            currentconfig.close()
            # With the backup created, write the new data to the original file
            currentconfig = open(cfgpath, "w")
            currentconfig.write(cfgdata)
            currentconfig.close()
        except:
            msg = "Unable to write config file during SAVE_CONFIG"
            logging.exception(msg)
            raise gcode.error(msg)

    def _save_includes(self, cfgpath, data, visitedpaths, gcode):
        # Prevent an infinite loop in the event of configs circularly
        # referencing each other
        if cfgpath in visitedpaths:
            return

        visitedpaths.add(cfgpath)
        dirname = os.path.dirname(cfgpath)
        # Read the data as individual lines so we can find include blocks
        lines = data.split("\n")
        for line in lines:
            # Strip trailing comment
            pos = line.find("#")
            if pos >= 0:
                line = line[:pos]

            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group("header")
            if header and header.startswith("include "):
                include_spec = header[8:].strip()
                include_glob = os.path.join(dirname, include_spec)
                # retrieve all filenames associated with the absolute path of
                # the include header
                include_filenames = glob.glob(include_glob)
                if not include_filenames and not glob.has_magic(include_glob):
                    # Empty set is OK if wildcard but not for direct file
                    # reference
                    raise error(
                        "Include file '%s' does not exist" % (include_glob,)
                    )
                include_filenames.sort()
                # Read the include files and check them against autosave data.
                # If autosave data overwites anything we'll update the file
                # and create a backup.
                for include_filename in include_filenames:
                    # Recursively check for includes. No need to check for looping
                    # includes as klipper checks this at startup.
                    include_predata = self._read_config_file(include_filename)
                    self._save_includes(
                        include_filename, include_predata, visitedpaths, gcode
                    )

                    include_postdata = self._strip_duplicates(
                        include_predata, self.autosave
                    )
                    # Only write and backup data that's been changed
                    if include_predata != include_postdata:
                        self._write_backup(
                            include_filename, include_postdata, gcode
                        )

    def cmd_SAVE_CONFIG(self, gcmd):
        if not self.autosave.fileconfig.sections():
            return
        gcode = self.printer.lookup_object("gcode")
        # Create string containing autosave data
        autosave_data = self._build_config_string(self.autosave)
        lines = [("#*# " + l).strip() for l in autosave_data.split("\n")]
        lines.insert(0, "\n" + AUTOSAVE_HEADER.rstrip())
        lines.append("")
        autosave_data = "\n".join(lines)
        # Read in and validate current config file
        cfgname = self.printer.get_start_args()["config_file"]
        try:
            data = self._read_config_file(cfgname)
            regular_data, old_autosave_data = self._find_autosave_data(data)
            config = self._build_config_wrapper(regular_data, cfgname)
        except error as e:
            msg = "Unable to parse existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcode.error(msg)
        regular_data = self._strip_duplicates(regular_data, self.autosave)

        if get_danger_options().autosave_includes:
            self._save_includes(cfgname, data, set(), gcode)

        # NOW we're safe to check for conflicts
        self._disallow_include_conflicts(regular_data, cfgname, gcode)
        data = regular_data.rstrip() + autosave_data
        self._write_backup(cfgname, data, gcode)

        # If requested restart or no restart just flag config saved
        require_restart = gcmd.get_int("RESTART", 1, minval=0, maxval=1)
        if require_restart:
            # Request a restart
            gcode.request_restart("restart")
        else:
            # flag config updated to false since config saved with no restart
            self.save_config_pending = False
            gcode.respond_info("Config update without restart successful")

