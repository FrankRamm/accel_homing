# Opt-in accelerometer homing evidence recorder. GPLv3.
"""Bounded, passive observation; file export and plotting only after cleanup.

Positions and travel in this report are commanded/MCU step accounting. They do
not measure carriage position or prove absence of lost steps. Host stop dispatch
is not a measurement of hardware receipt. Missing evidence is never zero.
"""
from array import array
from collections import deque
import csv
import datetime
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import threading
import time
import zipfile

SCHEMA = "accel_homing_troubleshoot_v1"
MAX_RAW_ROWS = 1000000  # 32 MB in four-double arrays, across the whole run.
MAX_HOME_ROWS = 200000  # Matches the core capture ceiling.
MAX_HOME_BATCHES = 512
INITIAL_HOME_BATCHES = 128
MAX_EVENTS = 3000
MAX_HOMES = 102
LOG_LIMIT = 2 * 1024 * 1024


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def clean(value, depth=0):
    """Small, finite JSON snapshots; never stringify arbitrary firmware objects."""
    if value is None or type(value) in (str, bool, int):
        return value[:4096] if isinstance(value, str) else value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if depth >= 8:
        return {"omitted": "snapshot depth bound"}
    if isinstance(value, dict):
        result = {str(key)[:128]: clean(item, depth + 1)
                  for key, item in list(value.items())[:128] if str(key) != "_record"}
        if len(value) > 128:
            result["_omitted_keys"] = len(value) - 128
        return result
    if isinstance(value, (tuple, list)):
        result = [clean(item, depth + 1) for item in value[:256]]
        if len(value) > 256:
            result.append({"omitted_items": len(value) - 256})
        return result
    return {"unavailable_type": type(value).__name__}


def _observe(obj, method, *args):
    try:
        callback = getattr(obj, method, None)
        return clean(callback(*args)) if callable(callback) else {"unavailable": method}
    except Exception as exc:
        return {"unavailable": method, "error": str(exc)[:512]}


def _snapshot(workflow):
    """Cached status and position accessors only; no MCU register queries."""
    owner, now = workflow.owner, workflow.reactor.monotonic()
    toolhead = owner.toolhead
    kin = toolhead.get_kinematics()
    steppers = []
    for stepper in kin.get_steppers():
        steppers.append({"name": _observe(stepper, "get_name"),
                         "mcu_position_steps": _observe(stepper, "get_mcu_position"),
                         "commanded_position_mm": _observe(stepper, "get_commanded_position"),
                         "step_distance_mm": _observe(stepper, "get_step_dist")})
    objects = {}
    for name, obj in workflow.printer.lookup_objects():
        if name == "mcu" or name.startswith(("mcu ", "tmc")):
            objects[name] = {"status": _observe(obj, "get_status", now)}
            if name == "mcu" or name.startswith("mcu "):
                objects[name]["stats"] = _observe(obj, "stats", now)
    return {"host_monotonic_s": now, "logical_commanded_position_mm": clean(toolhead.get_position()),
            "kinematic_limits": clean(getattr(kin, "limits", None)),
            "kinematic_status": _observe(kin, "get_status", now),
            "toolhead_status": _observe(toolhead, "get_status", now),
            "max_velocity_and_acceleration": _observe(toolhead, "get_max_velocity"),
            "backend_cached_status": _observe(getattr(owner, "backend", None), "get_status"),
            "steppers": steppers, "cached_mcu_tmc": objects,
            "runtime": clean({name: getattr(owner, name, None) for name in (
                "threshold_g", "impact_sign", "homing_accel", "back_off_dist", "saved_accel",
                "operation_generation", "sensor_fault", "scale", "session_scale", "full_scale_g",
                "sensor_name", "baseline_status", "baseline_time", "baseline_batches",
                "stream_timeout", "stream_on", "native_stream_generation", "last")})}


def snapshot(workflow):
    try:
        return _snapshot(workflow)
    except Exception as exc:
        return {"unavailable": "cached state snapshot", "error": str(exc)}


def environment(workflow):
    owner = workflow.owner
    settings = workflow._status("configfile").get("settings", {})
    # Never include macros, network/auth settings or arbitrary complete configs.
    allowed = lambda name: name in ("printer", "accel_homing", "beacon_accel_homing", "beacon", "cartographer", "mcu") or name.startswith((
        "stepper_", "tmc", "mcu ", "accel_homing_model ", "adxl345", "lis2dw", "lis3dh", "icm20948", "mpu9250"))
    sections = {name: clean(values) for name, values in settings.items()
                if allowed(name) or name == getattr(owner, "sensor_name", None)}
    for name, values in sections.items():
        if isinstance(values, dict):
            for key in list(values):
                if any(word in key.lower() for word in ("password", "secret", "token", "credential")):
                    values[key] = "<redacted>"
    backend = getattr(owner, "backend", None)
    helper = getattr(owner, "helper", None)
    sensor = getattr(backend, "sensor", getattr(owner, "sensor", None))
    return {"python": sys.version, "platform": platform.platform(),
            "effective_config_sections": sections,
            "config_available": bool(sections),
            "host_status": workflow._status("webhooks"),
            "sensor": {"name": getattr(owner, "sensor_name", None),
                       "backend": getattr(backend, "kind", "beacon_native"),
                       "backend_type": type(backend).__name__, "helper_type": type(helper).__name__,
                       "configured_sensor_class": type(sensor).__module__ + "." + type(sensor).__name__,
                       "configured_sensor_cached": clean({key: getattr(sensor, key, None) for key in (
                           "name", "data_rate", "rate", "axes_map", "full_scale", "rest_ticks")}),
                       "cached_backend": clean({key: getattr(backend, key, None) for key in (
                           "name", "sensor_name", "chip", "data_rate", "rate", "scale", "axes_map")})},
            "model": clean({"name": workflow.model_name, "section": workflow.model_section,
                             "metadata": getattr(workflow.model, "metadata", None),
                             "thresholds": getattr(workflow.model, "thresholds", None)}),
            "workflow": {"reposition_speed_mm_s": workflow.reposition_speed,
                         "settle_time_s": workflow.calibration_settle_time,
                         "travel_tolerance": workflow.travel_tolerance}}


class Recorder:
    def __init__(self, workflow, session):
        self.workflow, self.session = workflow, session
        self.events, self.batches, self.homes = [], [], []
        self.current_home = None
        self.raw_rows = self.events_omitted = self.batches_omitted = 0
        self.observer_errors = 0
        self.observer_lock = threading.Lock()
        self.home_by_arm = {}
        self.closed = False
        self.closed_at = None
        self.started_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
        self.context = environment(workflow)
        self.before = snapshot(workflow)
        self.log_start = self._log_position()
        self.folder = None

    def _log_position(self):
        args = self.workflow.chart_start_args
        path = args.get("log_file")
        try:
            stat = Path(path).stat()
            return {"path": str(path), "offset": stat.st_size, "inode": stat.st_ino}
        except (TypeError, OSError) as exc:
            return {"unavailable": str(exc)}

    def _home_for(self, kind, fields):
        arm, axis = fields.get("arm_print_time_s"), fields.get("axis")
        if finite(arm):
            key = (axis, arm)
            if kind == "arm" and self.current_home is not None:
                self.home_by_arm[key] = self.current_home
            home = self.home_by_arm.get(key)
            # Configured outer batch telemetry need not carry axis, but arm time
            # is unique within the current sequential run.
            if home is None and axis is None:
                matches = [home for (_axis, stamp), home in self.home_by_arm.items() if stamp == arm]
                home = matches[0] if len(matches) == 1 else None
            return home, "arm_print_time" if home is not None else "unresolved_arm_print_time"
        stamp = next((fields[key] for key in ("callback_entry_host_monotonic_s", "callback_host_monotonic_s",
                                             "request_before_host_monotonic_s", "host_monotonic_s")
                      if finite(fields.get(key))), None)
        if stamp is not None:
            matches = [home for home in self.homes if home["begin_host_monotonic_s"] <= stamp]
            return (matches[-1], "home_interval_at_callback_entry") if matches else (None, "before_first_home")
        workflow_kinds = ("workflow_", "home_begin", "home_observed_end", "return_", "terminal_release_")
        if kind.startswith(workflow_kinds):
            return self.current_home, "synchronous_workflow_observation"
        return None, "unresolved_missing_arm_and_entry_time"

    def record_event(self, kind, **fields):
        try:
            row = {"kind": str(kind)[:128], "host_observed_monotonic_s": self.workflow.reactor.monotonic(),
                   "fields": clean(fields)}
            with self.observer_lock:
                if self.closed:
                    return
                if len(self.events) >= MAX_EVENTS:
                    self.events_omitted += 1
                    return
                home, attribution = self._home_for(kind, fields)
                row.update(home_id=home["home_id"] if home else None, attribution=attribution)
                self.events.append(row)
        except Exception:
            self.observer_errors += 1

    def record_batch(self, source, **fields):
        try:
            entry, exit_time = (fields.get("callback_entry_host_monotonic_s"),
                                fields.get("callback_exit_host_monotonic_s"))
            if finite(entry) and finite(exit_time):
                fields["callback_duration_s"] = exit_time - entry
            estimate, last = fields.get("callback_entry_estimated_print_time_s"), fields.get("original_last_print_time_s")
            if finite(estimate) and finite(last):
                fields["original_last_sample_age_at_callback_s"] = estimate - last
            row = {"source": str(source)[:128], "fields": clean(fields)}
            with self.observer_lock:
                if self.closed:
                    return
                home, attribution = self._home_for("packet", fields)
                row.update(home_id=home["home_id"] if home else None, attribution=attribution)
                if home is None:
                    if len(self.batches) >= MAX_HOME_BATCHES:
                        self.batches_omitted += 1
                    else:
                        self.batches.append(row)
                    return
                home["batches_observed"] += 1
                row["home_packet_sequence"] = home["batches_observed"]
                if len(home["packet_prefix"]) < INITIAL_HOME_BATCHES:
                    home["packet_prefix"].append(row)
                else:
                    tail = home["packet_tail"]
                    if len(tail) == tail.maxlen:
                        self.batches_omitted += 1
                        home["batches_omitted"] += 1
                    tail.append(row)
                home["batches_retained"] = len(home["packet_prefix"]) + len(home["packet_tail"])
        except Exception:
            self.observer_errors += 1

    def close(self):
        """Freeze evidence after stream/command unwind, before any file writes."""
        with self.observer_lock:
            self.closed = True
            self.closed_at = self.workflow.reactor.monotonic()

    def begin_home(self, axis, sample):
        previous = self.current_home
        if len(self.homes) >= MAX_HOMES:
            raise ValueError("troubleshoot home evidence limit reached")
        home = {"home_id": len(self.homes) + 1, "axis": axis, "sample": sample,
                "begin_host_monotonic_s": self.workflow.reactor.monotonic(),
                "phase": "discovery" if sample == 0 else "measurement",
                "previous_home_id": previous["home_id"] if previous else None,
                "previous_return_completed": previous.get("return_completed", False) if previous else None,
                "before": snapshot(self.workflow), "raw": array("d"), "filtered": array("d"),
                "capture": {"unavailable": "capture did not complete"},
                "packet_prefix": [], "packet_tail": deque(maxlen=MAX_HOME_BATCHES - INITIAL_HOME_BATCHES),
                "batches_observed": 0, "batches_retained": 0, "batches_omitted": 0}
        self.current_home = home
        self.homes.append(home)
        self.record_event("home_begin", axis=axis, sample=sample)

    def retain_capture(self, capture):
        """Called after sensor stop, before validation or any chart decimation."""
        home = self.current_home
        if home is None:
            return
        try:
            samples = capture.get("samples", ())
            expected_homes = self.session["attempt_limit"]
            share, remainder = divmod(MAX_RAW_ROWS, expected_homes)
            budget = min(MAX_HOME_ROWS, share + int(home["home_id"] <= remainder))
            maximum = min(len(samples), budget)
            prefix = min(1024, maximum // 4) if len(samples) > maximum else maximum
            ranges = [(0, prefix)] if prefix else []
            if maximum > prefix:
                ranges.append((len(samples) - (maximum - prefix), len(samples)))
            home["raw_ranges"] = ranges
            raw, invalid = home["raw"], 0
            for first, end in ranges:
                for row in samples[first:end]:
                    if not isinstance(row, (tuple, list)) or len(row) != 4:
                        invalid += 1
                        raw.extend((float("nan"),) * 4)
                    else:
                        invalid += int(not all(finite(value) for value in row))
                        raw.extend(value if finite(value) else float("nan") for value in row)
            retained = len(raw) // 4
            metadata = capture.get("signal_filter", {})
            filtered = capture.get("filtered_samples", ())
            start = metadata.get("first_sample_index")
            if type(start) is int and filtered:
                for first, end in ranges:
                    for index in range(first, end):
                        row = filtered[index - start] if start <= index < start + len(filtered) else None
                        home["filtered"].extend(row[1:] if row is not None else (float("nan"),) * 3)
            self.raw_rows += retained
            home["capture"] = clean({key: value for key, value in capture.items()
                                     if key not in ("samples", "filtered_samples", "_record")})
            home["capture"].update(raw_rows_received=len(samples), raw_rows_retained=retained,
                                   raw_rows_omitted=len(samples) - retained,
                                   invalid_rows=invalid, retention="first reference prefix and last impact tail; original sample indices; no averaging",
                                   reserved_home_row_budget=budget,
                                   retained_sample_ranges=clean(ranges),
                                   omitted_sample_ranges=([[prefix, len(samples) - maximum + prefix]] if len(samples) > maximum else []),
                                   raw_units="print_time seconds, machine-axis g including gravity/bias",
                                   source="owner.stop_capture before workflow validation",
                                   truncation=bool(len(samples) > retained or capture.get("dropped")))
            if filtered:
                home["capture"].update(filtered_rows_retained=len(home["filtered"]) // 3,
                                       filtered_retention="same original indices as raw; blank before first armed sample",
                                       filtered_replay_complete=not bool(len(samples) > retained or capture.get("dropped")))
        except Exception as exc:
            home["capture"]["retention_error"] = str(exc)
            self.observer_errors += 1

    def finish_home(self, record=None, error=None):
        if self.current_home is not None:
            self.current_home.update(record=clean(record), error=str(error) if error else None,
                                     after=snapshot(self.workflow))
            self.record_event("home_observed_end", record=record, error=str(error) if error else None)

    def _source_identity(self):
        paths = set(Path(__file__).resolve().parent.glob("accel_homing*.py"))
        paths.add(Path(__file__).resolve().parent / "beacon_accel_homing.py")
        owner = self.workflow.owner
        backend = getattr(owner, "backend", None)
        sensor = getattr(backend, "sensor", getattr(owner, "sensor", None))
        objects = [owner, owner.toolhead, owner.toolhead.get_kinematics(), backend, sensor,
                   getattr(owner, "helper", None), getattr(backend, "helper", None)]
        objects += list(owner.toolhead.get_kinematics().get_steppers())
        objects += [obj for name, obj in self.workflow.printer.lookup_objects()
                    if name == "mcu" or name.startswith("mcu ") or name == getattr(owner, "sensor_name", None)]
        loaded = {type(obj).__module__ for obj in objects if obj is not None}
        dependencies = {"klippy", "homing", "bulk_sensor", "mcu", "stepper", "toolhead", "clocksync"}
        loaded.update(name for name in list(sys.modules) if name.rsplit(".", 1)[-1] in dependencies)
        module_files = {}
        for name in sorted(loaded):
            module = sys.modules.get(name)
            filename = getattr(module, "__file__", None)
            if filename:
                source = Path(filename).resolve()
                paths.add(source)
                module_files[name] = str(source)
            else:
                module_files[name] = {"unavailable": "loaded module has no source file"}
        fingerprints = []
        for path in sorted(paths):
            try:
                data = path.read_bytes()
                fingerprints.append({"path": str(path), "size": len(data),
                                     "sha256": hashlib.sha256(data).hexdigest()})
            except OSError as exc:
                fingerprints.append({"path": str(path), "unavailable": str(exc)})
        # Fingerprints describe installed bytes even for copies/symlinks or dirty trees.
        args = self.workflow.chart_start_args
        return {"source_files": fingerprints, "loaded_modules": module_files,
                "host_version": clean({key: args.get(key) for key in ("software_version", "git_version", "cpu_info")}),
                "git_commit_dirty": "not queried; installed source SHA256 fingerprints supplied"}

    def _export_log(self, folder):
        start = self.log_start
        if "path" not in start:
            return start
        try:
            source = Path(start["path"])
            stat = source.stat()
            rotated = stat.st_ino != start["inode"] or stat.st_size < start["offset"]
            offset = 0 if rotated else start["offset"]
            with source.open("rb") as stream:
                # Startup lines are selected, not a full potentially secret config dump.
                head = stream.read(min(stat.st_size, 262144)).decode("utf-8", "replace")
                startup = [line for line in head.splitlines() if line.startswith((
                    "Git version:", "CPU:", "Python:", "Start printer at", "Starting Klippy", "Loaded MCU", "MCU "))]
                stream.seek(offset)
                data = stream.read(LOG_LIMIT)
            (folder / "klippy-run.log").write_bytes(data)
            (folder / "klippy-startup.txt").write_text("\n".join(startup), encoding="utf-8")
            return {"source": str(source), "offset": offset, "bytes": len(data),
                    "truncated": stat.st_size - offset > len(data), "rotated": rotated,
                    "startup_metadata_lines": len(startup), "startup_scan_bytes": min(stat.st_size, 262144)}
        except OSError as exc:
            return {"unavailable": str(exc)}

    def export(self, ending):
        """After motion/cleanup only. Raw JSON/CSV survive any plotting failure."""
        self.close()
        root = Path(self.workflow.charts_directory) / "troubleshoot"
        root.mkdir(parents=True, exist_ok=True)
        folder = root / self.session["run_id"]
        folder.mkdir()  # Unique UUID; deliberately never overwrite another report.
        self.folder = str(folder)
        homes = []
        for home in self.homes:
            public = {key: value for key, value in home.items()
                      if key not in ("raw", "filtered", "packet_prefix", "packet_tail")}
            public["stop_timing"] = self._stop_timing(home["home_id"])
            public["packet_retention"] = {"policy": "first prefix and rolling last tail; omitted middle",
                "prefix_limit": INITIAL_HOME_BATCHES, "tail_limit": MAX_HOME_BATCHES - INITIAL_HOME_BATCHES,
                "retained_sequence_ranges": [[rows[0]["home_packet_sequence"], rows[-1]["home_packet_sequence"]]
                                             for rows in (home["packet_prefix"], home["packet_tail"]) if rows]}
            name = "home-%03d-%s.csv.gz" % (home["home_id"], home["axis"])
            public["raw_file"] = name
            with gzip.open(folder / name, "wt", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                filtered = home.get("filtered", ())
                columns = ("sample_index", "print_time_s", "x_g", "y_g", "z_g")
                if filtered:
                    columns += ("filtered_x_g", "filtered_y_g", "filtered_z_g", "filtered_magnitude_g")
                writer.writerow(columns)
                raw = home["raw"]
                offset = 0
                for first, end in home.get("raw_ranges", ()):
                    for original_index in range(first, end):
                        row = [original_index] + [value if finite(value) else "" for value in raw[offset:offset + 4]]
                        if filtered:
                            xyz = filtered[(offset // 4) * 3:(offset // 4) * 3 + 3]
                            row += ([*xyz, math.sqrt(sum(value * value for value in xyz))]
                                    if all(finite(value) for value in xyz) else [""] * 4)
                        writer.writerow(row)
                        offset += 4
            homes.append(public)
        with gzip.open(folder / "packets.jsonl.gz", "wt", encoding="utf-8") as stream:
            for group in [self.batches] + [rows for home in self.homes for rows in (home["packet_prefix"], home["packet_tail"])]:
                for row in group:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
        report = {"schema": SCHEMA, "run_id": self.session["run_id"], "created_utc": self.started_utc,
                  "ending": clean(ending), "axes": self.session["axes"],
                  "samples_requested": self.session["samples_requested"],
                  "fixed_thresholds_g": clean(self.session["saved_thresholds"]),
                  "configured_speeds_mm_s": clean(self.session["configured_speeds"]),
                  "environment": self.context, "installed_identity": self._source_identity(),
                  "before": self.before, "after": snapshot(self.workflow), "homes": homes,
                  "events": self.events, "log": self._export_log(folder),
                  "collector_closed_host_monotonic_s": self.closed_at,
                  "bounds": {"max_raw_rows": MAX_RAW_ROWS, "max_home_rows": MAX_HOME_ROWS,
                             "raw_rows_retained": self.raw_rows, "max_events": MAX_EVENTS,
                             "events_omitted": self.events_omitted, "max_batches": (MAX_HOMES + 1) * MAX_HOME_BATCHES,
                             "max_batches_per_home": MAX_HOME_BATCHES,
                             "batches_retained": len(self.batches) + sum(home["batches_retained"] for home in self.homes),
                             "batches_omitted": self.batches_omitted, "observer_errors": self.observer_errors},
                  "interpretation": ["Commanded positions and step counts are not measured physical carriage positions.",
                      "Host stop dispatch does not establish MCU receipt time.",
                      "Clock domains are named in fields; do not subtract host monotonic from print_time.",
                      "callback_duration_s ends before recorder formatting/append and excludes that observer overhead; it is not full callback occupancy or transport latency.",
                      "Late callbacks holding an old recorder reference after collector_closed_host_monotonic_s are ignored; exported evidence is frozen.",
                      "Discovery is retained and excluded from measured-trial statistics.",
                      "Rejected/partial homes are retained; no automatic recalibration or threshold change.",
                      "Empty CSV cells represent invalid/missing numbers; see capture invalid_rows.",
                      "Optional filtered columns are signed baseline-subtracted XYZ, with causal zero state at the exact first armed sample. Raw columns are unchanged. Replay requires every intervening raw sample; omitted middle ranges prevent full replay.",
                      "Charts use impact windows; raw retained CSV rows are not decimated."]}
        write_json(folder / "manifest.json", report)
        (folder / "README.txt").write_text(
            "ACCEL_HOMING_TROUBLESHOOT evidence\n\n"
            "Share the entire folder or ZIP. Read manifest.json first.\n"
            "home-*.csv.gz: every retained raw decoded XYZ sample before graph reduction, in g including gravity/bias.\n"
            "Optional filtered_* columns: causal notch after resting-offset subtraction; blank before arm. Coefficients/rate/start identity are in manifest.json.\n"
            "packets.jsonl.gz: bounded callback timing, pending/filtered packet context.\n"
            "manifest.json: home/return/next-home linkage, detector/reference/quality metadata, errors, config, installed source SHA256, bounds.\n"
            "Each home records capture/core drops and recorder omissions. Missing is not zero.\n"
            "No physical-position sensor is present; travel is commanded step accounting.\n"
            "Stop markers distinguish sensor print_time from host dispatch. Hardware receipt is unknown.\n"
            "PNG failures do not remove raw evidence. Discovery/rejections/outliers remain.\n",
            encoding="utf-8")
        try:
            def check_render():
                if self.session.get("finish_interrupted"):
                    raise ValueError("PNG export interrupted after motion cleanup: " + self.session["finish_interrupted"])
            render_bundle(folder, self.workflow.renderer, self.workflow.reactor, check_render)
            report["render"] = {"result": "complete", "files": sorted(path.name for path in folder.glob("*.png"))}
        except Exception as exc:
            report["render"] = {"result": "failed", "error": str(exc)}
        if self.session.get("troubleshoot_late_ending"):
            ending.update(self.session["troubleshoot_late_ending"])
            report["ending"] = clean(ending)
        write_json(folder / "manifest.json", report)
        archive = folder.with_suffix(".zip")
        try:
            with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED) as zipped:
                for path in sorted(folder.iterdir()):
                    if path.is_file():
                        zipped.write(path, folder.name + "/" + path.name)
        except Exception as exc:
            report["zip_error"] = str(exc)
            write_json(folder / "manifest.json", report)
            archive = None
        return {"folder": str(folder), "manifest": str(folder / "manifest.json"),
                "archive": str(archive) if archive else None, "render": report["render"],
                "retained_homes": len(homes), "bounds": report["bounds"]}

    def _stop_timing(self, home_id):
        result = {"onset_to_host_dispatch_estimate_s": None,
                  "domain": "main_mcu_print_time",
                  "hardware_receipt_or_halt_time_available": False,
                  "reason": "matching onset/host dispatch evidence unavailable"}
        stops = [event["fields"] for event in self.events if event["home_id"] == home_id and event["kind"] == "stop_request"]
        crossings = [event["fields"] for event in self.events if event["home_id"] == home_id and event["kind"] == "threshold_crossing"]
        for stop in reversed(stops):
            dispatch = stop.get("request_estimated_print_time_s")
            onset, provenance = stop.get("onset_print_time_s"), "stop event detector onset"
            for crossing in crossings:
                if crossing.get("onset_sample") == stop.get("onset_sample") and finite(crossing.get("onset_print_time_s")):
                    onset, provenance = crossing["onset_print_time_s"], crossing.get("sample_time_provenance")
                    break
            if "local_main_mcu_clocksync_estimate" not in stop.get("timestamp_provenance", ""):
                result["reason"] = "stop dispatch clock mapping not established"
                continue
            if not all(finite(value) for value in (dispatch, onset)) or dispatch < onset:
                result["reason"] = "missing/nonfinite or reversed onset/dispatch clock estimate"
                continue
            result.update(onset_to_host_dispatch_estimate_s=dispatch - onset,
                          onset_print_time_s=onset, host_dispatch_estimated_print_time_s=dispatch,
                          onset_provenance=provenance, dispatch_returned=stop.get("dispatch_returned"),
                          reason=None)
            break
        return result


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def render_bundle(folder, renderer, reactor=None, check_running=None):
    """One isolated worker, using the existing selected Python/dependency setup."""
    python = getattr(renderer, "python_executable", sys.executable)
    dependencies = getattr(renderer, "dependency_directory", "")
    if check_running:
        check_running()
    with (Path(folder) / "renderer.log").open("wb") as log:
        process = subprocess.Popen([python, "-B", "-I", str(Path(__file__).resolve()),
                                    "--render-worker", str(folder), dependencies],
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        deadline = time.monotonic() + 300.0
        try:
            while process.poll() is None:
                if check_running:
                    check_running()
                if time.monotonic() >= deadline or os.fstat(log.fileno()).st_size > 1024 * 1024:
                    raise ValueError("troubleshoot PNG worker exceeded time/log bound")
                if reactor is None:
                    time.sleep(0.02)
                else:
                    reactor.pause(reactor.monotonic() + 0.02)
            if process.returncode:
                raise ValueError("troubleshoot PNG worker failed; see renderer.log")
            if check_running:
                check_running()
        finally:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=2)


def _worker(folder, dependencies):
    folder = Path(folder)
    if Path(dependencies).is_dir():
        sys.path.insert(0, str(Path(dependencies)))
    with tempfile.TemporaryDirectory(prefix="accel_troubleshoot_plot_") as cache:
        os.environ["MPLCONFIGDIR"] = cache
        os.environ["MPLBACKEND"] = "Agg"
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib import rcParams
        rcParams["text.usetex"] = False
        if "text.parse_math" in rcParams:
            rcParams["text.parse_math"] = False
        manifest = folder / "manifest.json"
        if manifest.stat().st_size > 32 * 1024 * 1024:
            raise ValueError("troubleshoot plotting manifest exceeds byte bound")
        report = json.loads(manifest.read_text(encoding="utf-8"))
        homes = report["homes"]
        if len(homes) > MAX_HOMES:
            raise ValueError("troubleshoot plotting home limit exceeded")
        figure = Figure(figsize=(12, 14), dpi=110)
        FigureCanvasAgg(figure)
        plots = figure.subplots(4, 1)
        for axis in report["axes"]:
            group = [home for home in homes if home["axis"] == axis]
            ids = [home["home_id"] for home in group]
            number = lambda home, key: (home.get("record") or {}).get(key)
            plots[0].plot(ids, [abs(number(h, "trigger_travel_mm_signed")) if finite(number(h, "trigger_travel_mm_signed")) else math.nan for h in group], "o-", label=axis.upper() + " trigger")
            plots[0].plot(ids, [abs(number(h, "halt_travel_mm_signed")) if finite(number(h, "halt_travel_mm_signed")) else math.nan for h in group], "x--", label=axis.upper() + " halt")
            for left, right in zip(group, group[1:]):
                a, b = number(left, "trigger_travel_mm_signed"), number(left, "halt_travel_mm_signed")
                travel = number(right, "travel_mm_exact")
                if not finite(travel):
                    travel = number(right, "travel_mm")
                expected = right.get("expected_travel_mm")
                if (all(finite(v) for v in (a, b, travel, expected)) and left.get("return_completed")
                        and right.get("quality") == "good" and right["previous_home_id"] == left["home_id"]):
                    plots[1].scatter(abs(b) - abs(a), travel - expected, label=axis.upper() if left is group[0] else None)
                    plots[1].annotate(str(right["home_id"]), (abs(b) - abs(a), travel - expected), fontsize=7)
                    delay = left["stop_timing"].get("onset_to_host_dispatch_estimate_s")
                    if finite(delay):
                        plots[2].scatter(delay * 1000, travel - expected)
                        plots[2].annotate(str(right["home_id"]), (delay * 1000, travel - expected), fontsize=7)
        plots[0].set(title="Commanded trigger and halt travel (discovery and rejected homes retained)", xlabel="Home ID", ylabel="Travel mm")
        plots[0].legend()
        plots[1].set(title="Previous commanded overshoot vs next-home travel error", xlabel="abs(halt) - abs(trigger), mm", ylabel="Next trigger travel - expected, mm")
        plots[2].set(title="Previous onset to host stop dispatch estimate vs next-home travel error", xlabel="Host dispatch estimate minus onset, ms (hardware receipt unknown)", ylabel="Next trigger travel - expected, mm")
        with gzip.open(folder / "packets.jsonl.gz", "rt", encoding="utf-8") as stream:
            packets = [json.loads(line) for line in stream]
        # Exact numeric field names are retained in the legend; host and sensor
        # timestamps are never subtracted across clock domains.
        age = plots[3].twinx()
        for key in sorted({key for row in packets for key in row["fields"]}):
            if key.endswith("_count") or "age" in key or "duration" in key:
                pairs = [(index, row["fields"].get(key)) for index, row in enumerate(packets)
                         if finite(row["fields"].get(key))]
                if pairs:
                    target = plots[3] if key.endswith("_count") else age
                    target.plot([p[0] for p in pairs], [p[1] for p in pairs], ".", markersize=2, label=key)
        plots[3].set(title="Packet counts and observed age/processing time (excludes recorder overhead)", xlabel="Retained packet row; see manifest for omitted middle ranges", ylabel="Count")
        age.set_ylabel("Seconds (field names identify timing semantics)")
        for plot in (plots[3], age):
            if plot.lines:
                plot.legend(fontsize=6, loc="upper left" if plot is plots[3] else "upper right")
        for plot in plots:
            plot.grid(True, alpha=.25)
        figure.suptitle("ACCEL_HOMING_TROUBLESHOOT: " + report["ending"].get("result", "unknown"))
        figure.tight_layout(rect=(0, 0, 1, .97))
        figure.savefig(folder / "summary.png")
        figure.clear()
        for home in homes:
            record, capture = home.get("record") or {}, home["capture"]
            onset = record.get("onset_time_s")
            if not finite(onset):
                onset = (capture.get("detector") or {}).get("onset_time_s")
            baseline = capture.get("baseline_g")
            has_baseline = isinstance(baseline, list) and len(baseline) == 3 and all(finite(v) for v in baseline)
            baseline = baseline if has_baseline else [0, 0, 0]
            rows = []
            with gzip.open(folder / home["raw_file"], "rt", encoding="utf-8") as stream:
                for row_index, row in enumerate(csv.DictReader(stream)):
                    if row_index >= MAX_HOME_ROWS:
                        raise ValueError("raw capture plot input exceeds per-home bound")
                    try:
                        sample = [float(row[key]) for key in ("print_time_s", "x_g", "y_g", "z_g")]
                    except ValueError:
                        continue
                    if not finite(onset) or onset - .025 <= sample[0] <= onset + .100:
                        try:
                            sample += [float(row[key]) for key in ("filtered_x_g", "filtered_y_g", "filtered_z_g")]
                        except (KeyError, ValueError):
                            sample += [float("nan")] * 3
                        rows.append(sample)
            # Window retains all samples/outliers. Bound unknown-onset plots by
            # displaying a prefix explicitly; raw files remain unchanged.
            omitted = max(0, len(rows) - 20000)
            rows = rows[:20000]
            figure = Figure(figsize=(11, 5), dpi=110)
            FigureCanvasAgg(figure)
            plot = figure.subplots()
            origin = onset if finite(onset) else rows[0][0] if rows else 0
            times = [(row[0] - origin) * 1000 for row in rows]
            for index, label in enumerate("XYZ"):
                plot.plot(times, [row[index + 1] - baseline[index] for row in rows], label=label, linewidth=.8)
            plot.plot(times, [math.sqrt(sum((row[index + 1] - baseline[index]) ** 2 for index in range(3))) for row in rows], label="XYZ norm", color="black", linewidth=1)
            if any(all(finite(value) for value in row[4:7]) for row in rows):
                plot.plot(times, [math.sqrt(sum(value * value for value in row[4:7])) for row in rows],
                          label="Filtered XYZ magnitude", color="#d17b00", linewidth=1.2)
                threshold = record.get("threshold_g")
                if finite(threshold):
                    plot.axhline(threshold, color="#d17b00", linestyle="--", label="Filtered trigger")
            if finite(onset):
                plot.axvline(0, color="red", label="retained sensor onset")
            for event in report["events"]:
                if event["home_id"] == home["home_id"] and event["kind"] == "stop_request":
                    fields = event["fields"]
                    # Use only a documented estimate in sensor/main-MCU print time.
                    for key in ("request_estimated_print_time_s",):
                        stamp = fields.get(key)
                        if finite(stamp):
                            plot.axvline((stamp - origin) * 1000, color="purple", linestyle="--", label="host dispatch, estimated print_time")
                            break
            plot.set(title="Home %03d %s %s; %s; raw omitted %s; plot omitted %d" % (
                home["home_id"], home["axis"].upper(), home["phase"], home.get("error") or record.get("result", "partial"), capture.get("raw_rows_omitted", "unknown"), omitted),
                xlabel="ms from retained onset" if finite(onset) else "ms from first retained sample (onset unavailable)",
                ylabel="g after reference subtraction" if has_baseline else "raw g; reference unavailable (gravity/bias retained)")
            plot.legend(fontsize=7)
            plot.grid(True, alpha=.25)
            figure.tight_layout()
            figure.savefig(folder / ("home-%03d-%s.png" % (home["home_id"], home["axis"])))
            figure.clear()


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--render-worker":
        _worker(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit("Internal troubleshoot plotting worker")
