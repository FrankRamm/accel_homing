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
import gc
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
MAX_TIMING_ROWS = 1024  # Per home, independently of packet/raw retention.
MAX_SLOW_TIMING_ROWS = 64
TIMING_INTERVAL = .020
SLOW_TIMING = .020


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
                           "name", "sensor_name", "chip", "data_rate", "rate", "scale", "axes_map", "full_scale")})},
            "model": clean({"name": workflow.model_name, "section": workflow.model_section,
                             "metadata": getattr(workflow.model, "metadata", None),
                             "thresholds": getattr(workflow.model, "thresholds", None)}),
            "workflow": {"reposition_speed_mm_s": workflow.reposition_speed,
                         "settle_time_s": workflow.calibration_settle_time,
                         "travel_tolerance": workflow.travel_tolerance}}


class TimingTrace:
    """Opt-in timing of existing calls; never query, reschedule or suppress them.

    The serial hook forwards the original object before recording scalar metadata.
    No payloads, file IO, stack walks or JSON formatting run in these hooks.
    CPU time is process-wide (including other threads), not exclusive stage CPU.
    """
    def __init__(self, recorder):
        self.recorder = recorder
        self.reactor = recorder.workflow.reactor
        self.lock = threading.RLock()  # GC callbacks can re-enter this observer.
        self.groups, self.hooks, self.available = {}, [], {}
        self.closed = False
        self.errors = 0
        self.serial_hook = self.timer = None
        self.gc_started = {}
        self.gc_callback = self._gc
        self.active_calls, self.active_at_close, self.next_call = {}, [], 0

    def _stamp(self):
        try:
            return self.reactor.monotonic(), time.process_time()
        except Exception:
            self.errors += 1
            return None, None

    def _home(self):
        try:
            home = self.recorder.current_home
            return home["home_id"] if home is not None else 0
        except Exception:
            self.errors += 1
            return 0

    def note(self, stage, start, end, details=None, home_id=None):
        try:
            if self.closed or not finite(start[0]) or not finite(end[0]):
                return
            home_id = self._home() if home_id is None else home_id
            with self.lock:
                if self.closed:
                    return
                if home_id not in self.groups:
                    if len(self.groups) >= MAX_HOMES + 1:
                        self.errors += 1
                        return
                    self.groups[home_id] = {"rows": deque(maxlen=MAX_TIMING_ROWS),
                        "slow": deque(maxlen=MAX_SLOW_TIMING_ROWS), "observed": 0,
                        "slow_observed": 0, "stages": {}}
                group = self.groups[home_id]
                stages = group["stages"]
                if stage not in stages:
                    stages[stage] = {"count": 0, "last_entry_s": None, "max_duration_s": 0.,
                                     "max_entry_gap_s": 0., "total_duration_s": 0.}
                stat = stages[stage]
                gap = None if stat["last_entry_s"] is None else start[0] - stat["last_entry_s"]
                duration = end[0] - start[0]
                if duration >= stat["max_duration_s"]:
                    stat["max_duration_entry_s"] = start[0]
                if gap is not None and gap >= stat["max_entry_gap_s"]:
                    stat["max_entry_gap_entry_s"] = start[0]
                stat.update(count=stat["count"] + 1, last_entry_s=start[0],
                            max_duration_s=max(stat["max_duration_s"], duration),
                            max_entry_gap_s=max(stat["max_entry_gap_s"], gap or 0.),
                            total_duration_s=stat["total_duration_s"] + duration)
                group["observed"] += 1
                row = {"sequence": group["observed"], "home_id": home_id, "stage": stage,
                       "entry_host_monotonic_s": start[0], "exit_host_monotonic_s": end[0],
                       "duration_s": duration, "entry_gap_s": gap,
                       "process_cpu_s": (end[1] - start[1] if finite(start[1]) and finite(end[1]) else None),
                       "details": details or {}}
                group["rows"].append(row)
                slow_gap = ((details or {}).get("lateness_s", 0.) if stage == "reactor_heartbeat" else gap)
                if duration >= SLOW_TIMING or (finite(slow_gap) and slow_gap >= SLOW_TIMING):
                    group["slow_observed"] += 1
                    group["slow"].append(row)
        except Exception:
            self.errors += 1

    def _wrap(self, obj, attr, stage, details=None):
        original = getattr(obj, attr, None)
        if not callable(original):
            self.available[stage] = "unavailable: compatible method not exposed"
            return
        own = attr in getattr(obj, "__dict__", {})

        def observed(*args, **kwargs):
            if self.closed:
                return original(*args, **kwargs)
            start, home_id = self._stamp(), self._home()
            token = self._enter_call(stage, start, home_id)
            result, returned = None, False
            try:
                result = original(*args, **kwargs)
                returned = True
                return result
            finally:
                end = self._stamp()
                try:
                    if token is not None:
                        with self.lock:
                            self.active_calls.pop(token, None)
                    fields = details(args, result) if details is not None and returned else {}
                    fields["returned"] = returned
                    self.note(stage, start, end, fields, home_id)
                except Exception:
                    self.errors += 1

        try:
            setattr(obj, attr, observed)
            self.hooks.append((obj, attr, original, observed, own))
            self.available[stage] = "enabled"
        except Exception as exc:
            self.available[stage] = "unavailable: " + type(exc).__name__

    def _enter_call(self, stage, start, home_id):
        # A query can yield while cancellation unwinds the workflow. Retain its
        # start even if its wrapper has not returned when collection closes.
        if stage not in ("sensor_status_query", "driver_batch"):
            return None
        try:
            with self.lock:
                if self.closed or len(self.active_calls) >= 16:
                    return None
                self.next_call += 1
                self.active_calls[self.next_call] = {"stage": stage, "home_id": home_id,
                    "entry_host_monotonic_s": start[0], "entry_process_cpu_s": start[1]}
                return self.next_call
        except Exception:
            self.errors += 1
            return None

    @staticmethod
    def _query_details(args, result):
        # Driver response timestamps are from the host serial queue, not MCU halt.
        return {key: result.get(key) for key in ("#sent_time", "#receive_time", "clock",
                "query_ticks", "next_sequence", "buffered", "possible_overflows")}

    @staticmethod
    def _queue_details(args, result):
        return {"messages": len(result),
                "first_sequence": result[0].get("sequence") if result else None,
                "last_sequence": result[-1].get("sequence") if result else None,
                "oldest_receive_host_monotonic_s": result[0].get("#receive_time") if result else None,
                "newest_receive_host_monotonic_s": result[-1].get("#receive_time") if result else None}

    def _serial(self, reader):
        serial = getattr(getattr(reader, "mcu", None), "_serial", None)
        handlers, lock = getattr(serial, "handlers", None), getattr(serial, "lock", None)
        key = ("sensor_bulk_data", getattr(reader, "oid", None))
        queue = getattr(reader, "bulk_queue", None)
        if not isinstance(handlers, dict) or lock is None or queue is None:
            self.available["mcu_bulk_handler"] = "unavailable: serial/BulkDataQueue contract"
            return
        with lock:
            original = handlers.get(key)
            if not callable(original) or getattr(original, "__self__", None) is not queue:
                self.available["mcu_bulk_handler"] = "unavailable: handler not owned by sensor queue"
                return

            def observed(params):
                if self.closed:
                    return original(params)
                start, home_id = self._stamp(), self._home()
                try:
                    return original(params)
                finally:
                    end = self._stamp()
                    try:
                        self.note("mcu_bulk_handler", start, end,
                            {"serial_receive_host_monotonic_s": params.get("#receive_time"),
                             "serial_sent_host_monotonic_s": params.get("#sent_time"),
                             "mcu_sequence": params.get("sequence"),
                             "payload_bytes": len(params.get("data", ()))}, home_id)
                    except Exception:
                        self.errors += 1
            handlers[key] = observed
            self.serial_hook = (handlers, lock, key, original, observed)
            self.available["mcu_bulk_handler"] = "enabled"

    def start(self):
        if self.closed or self.available:
            return
        try:
            owner = self.recorder.workflow.owner
            backend = getattr(owner, "backend", None)
            helper = getattr(backend, "helper", None)
            sensor = getattr(backend, "sensor", None)
            reader = getattr(sensor, "ffreader", None)
            # Captures driver entry, including empty batches, before plugin clients.
            self._wrap(helper, "batch_cb", "driver_batch",
                lambda args, result: {"reactor_eventtime_s": args[0] if args else None,
                                     "sample_count": len(result.get("data", ())) if result else 0})
            self._wrap(reader, "pull_samples", "sensor_pull")
            self._wrap(getattr(reader, "query_status_cmd", None), "send", "sensor_status_query", self._query_details)
            self._wrap(getattr(reader, "bulk_queue", None), "pull_queue", "driver_queue_drain", self._queue_details)
            self._wrap(sensor, "_convert_samples", "driver_decode")
            self._wrap(owner, "_inspect", "detector_with_observer")
            self._wrap(backend, "_receive", "adapter_processing")
            self._wrap(backend, "_troubleshoot_receive", "adapter_observer")
            self._wrap(self.recorder, "record_batch", "recorder_batch")
            self._wrap(self.recorder, "record_event", "recorder_event")
            self._serial(reader)
            gc.callbacks.append(self.gc_callback)
            self.available["gc"] = "enabled"
            if callable(getattr(self.reactor, "unregister_timer", None)):
                self.expected = self.reactor.monotonic() + TIMING_INTERVAL
                self.previous_heartbeat = self._stamp()
                self.timer = self.reactor.register_timer(self._heartbeat, self.expected)
                self.available["reactor_heartbeat"] = "enabled"
            else:
                self.available["reactor_heartbeat"] = "unavailable: unregister_timer not exposed"
        except Exception as exc:
            self.available["startup_error"] = type(exc).__name__
            self.close()  # A partially attached observer must not outlive the run.

    def _heartbeat(self, eventtime):
        if self.closed:
            return self.reactor.NEVER
        start = self._stamp()
        try:
            expected = self.expected
            self.expected = start[0] + TIMING_INTERVAL
            previous = self.previous_heartbeat
            self.previous_heartbeat = start
            self.note("reactor_heartbeat", start, start,
                      {"scheduled_host_monotonic_s": expected,
                       "lateness_s": max(0., start[0] - expected), "eventtime_s": eventtime,
                       "interval_s": start[0] - previous[0],
                       "interval_process_cpu_s": start[1] - previous[1]})
            return self.expected
        except Exception:
            self.errors += 1
            return eventtime + TIMING_INTERVAL

    def _gc(self, phase, info):
        if self.closed:
            return
        try:
            generation = info.get("generation")
            if phase == "start":
                self.gc_started[generation] = (self._stamp(), self._home())
            elif phase == "stop":
                before = self.gc_started.pop(generation, None)
                if before is not None:
                    self.note("gc", before[0], self._stamp(),
                              {key: info.get(key) for key in ("generation", "collected", "uncollectable")}, before[1])
        except Exception:
            self.errors += 1

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.active_at_close = list(self.active_calls.values())
        if self.timer is not None:
            try:
                self.reactor.unregister_timer(self.timer)
            except Exception:
                self.errors += 1
        try:
            if self.gc_callback in gc.callbacks:
                gc.callbacks.remove(self.gc_callback)
        except Exception:
            self.errors += 1
        if self.serial_hook is not None:
            handlers, lock, key, original, wrapper = self.serial_hook
            try:
                with lock:
                    if handlers.get(key) is wrapper:
                        handlers[key] = original
            except Exception:
                self.errors += 1
        for obj, attr, original, wrapper, own in reversed(self.hooks):
            try:
                if getattr(obj, attr, None) is wrapper:
                    if own:
                        setattr(obj, attr, original)
                    else:
                        delattr(obj, attr)
            except Exception:
                self.errors += 1

    def export(self, folder):
        summaries = {}
        with gzip.open(folder / "timing.jsonl.gz", "wt", encoding="utf-8") as stream:
            for home_id, group in sorted(self.groups.items()):
                retained = {row["sequence"]: row for row in list(group["slow"]) + list(group["rows"])}
                for _, row in sorted(retained.items()):
                    stream.write(json.dumps(clean(row), allow_nan=False) + "\n")
                summaries[str(home_id)] = {"observed": group["observed"], "retained": len(retained),
                    "omitted": group["observed"] - len(retained),
                    "slow_observed": group["slow_observed"], "slow_retained": len(group["slow"]),
                    "stages": clean(group["stages"])}
        return {"schema": "accel_homing_timing_v1", "file": "timing.jsonl.gz", "hooks": self.available,
                "in_flight_at_close": clean(self.active_at_close),
                "errors": self.errors, "max_tail_rows_per_home": MAX_TIMING_ROWS,
                "max_slow_rows_per_home": MAX_SLOW_TIMING_ROWS, "slow_threshold_s": SLOW_TIMING,
                "heartbeat_interval_s": TIMING_INTERVAL, "homes": summaries,
                "clock_domain": "host_monotonic", "cpu_scope": "process including other threads",
                "interpretation": "Nested durations overlap; do not sum stages. Serial receive is a host transport timestamp, not hardware receipt. Recorder/adapter observer costs exclude timing-hook bookkeeping itself. Hooks add overhead and do not locate delays inside firmware or the kernel."}


def adxl_saturation_profile(context):
    """Diagnostic-only endpoints from the configured ADXL345 driver's axes_map.

    Full-resolution +/-16 g uses signed 13-bit counts. At 1600/3200 Hz the
    least significant bit is zero (ADXL345 data sheet, Data Formatting).
    Driver scales are signed mm/s2 per count, already mapped to machine axes.
    """
    sensor = context.get("sensor", {})
    cached = sensor.get("configured_sensor_cached", {})
    cls = sensor.get("configured_sensor_class", "")
    rate, mapping = cached.get("data_rate"), cached.get("axes_map")
    if (not cls.endswith("adxl345.ADXL345") or
            sensor.get("cached_backend", {}).get("full_scale") != 16.0 or
            rate not in (25, 50, 100, 200, 400, 800, 1600, 3200) or
            not isinstance(mapping, (list, tuple)) or len(mapping) != 3):
        return {"available": False, "reason": "verified ADXL345 range/rate/mapping unavailable"}
    try:
        if sorted(pair[0] for pair in mapping) != [0, 1, 2] or not all(
                finite(pair[1]) and pair[1] != 0 for pair in mapping):
            raise ValueError()
        upper = 4094 if rate in (1600, 3200) else 4095
        return {"available": True, "method": "adxl345_mapped_native_endpoints_v1", "rate_hz": rate,
                "range_source": "configured full-resolution ADXL345 driver contract; no register query",
                "native_min_count": -4096, "native_max_count": upper,
                "count_tolerance": .01, "axes_map": mapping,
                "axis_endpoints_g": {axis: sorted([-4096 * pair[1] / 9806.65, upper * pair[1] / 9806.65])
                                     for axis, pair in zip("xyz", mapping)}}
    except (ValueError, TypeError, IndexError):
        return {"available": False, "reason": "invalid ADXL345 mapping"}


def diagnose_saturation(public, raw, profile, stop_time):
    """Correct exported evidence only, after motion; never mutate detector/model state."""
    if not profile.get("available"):
        public["saturation_diagnostic"] = profile
        return
    record, capture = public.get("record") or {}, public.get("capture") or {}
    arm, onset = record.get("arm_time_s"), record.get("onset_time_s")
    phases = {key: dict.fromkeys("xyz", 0) for key in
              ("before_arm", "armed_before_onset", "after_onset_before_stop", "after_stop", "unattributed")}
    counts, first, last = dict.fromkeys("xyz", 0), None, None
    for offset in range(0, len(raw), 4):
        when = raw[offset]
        phase = ("unattributed" if not finite(when) else
                 "before_arm" if finite(arm) and when < arm else
                 "after_stop" if finite(stop_time) and when >= stop_time else
                 "after_onset_before_stop" if finite(onset) and when >= onset else
                 "armed_before_onset" if finite(arm) and when >= arm else "unattributed")
        for index, (axis, mapping) in enumerate(zip("xyz", profile["axes_map"])):
            value = raw[offset + index + 1]
            if not finite(value):
                continue
            native = value * 9806.65 / mapping[1]
            if (native <= profile["native_min_count"] + profile["count_tolerance"] or
                    native >= profile["native_max_count"] - profile["count_tolerance"]):
                counts[axis] += 1
                phases[phase][axis] += 1
                if finite(when):
                    first, last = (when if first is None else first), when
    limited = bool(sum(counts.values()))
    public["saturation_diagnostic"] = {**profile, "scope": "retained_raw_capture",
        "range_limit_axis_counts": counts, "range_limit_axis_values": sum(counts.values()),
        "amplitude_limited": limited, "phases": phases, "first_print_time_s": first, "last_print_time_s": last,
        "complete_capture": bool(raw) and not any(capture.get(key) for key in
            ("dropped", "raw_rows_omitted", "invalid_rows", "truncation", "unavailable", "retention_error")),
        "physical_peak_recoverable": False if limited else None}
    # Retain the original runtime report verbatim. Calibration/acceptance has
    # already finished; flags here describe the raw capture, including braking.
    if limited:
        for key in ("capture", "record"):
            if not isinstance(public.get(key), dict):
                continue
            original = public[key]
            public[key] = {**original, "runtime_signal_quality": {name: original.get(name) for name in (
                "amplitude_limited", "excess_amplitude_unknown", "range_limit_axis_values", "range_limit_axis_counts", "peak_semantics", "signal_scope")},
                "amplitude_limited": True, "excess_amplitude_unknown": True,
                "range_limit_axis_values": sum(counts.values()), "range_limit_axis_counts": dict(counts),
                "peak_semantics": "reported_sensor_derived_range_limited",
                "signal_scope": "retained_raw_capture", "signal_quality_source": profile["method"]}


DELIVERY_SEGMENTS = (
    ("onset_to_batch_last", "Onset to source batch end", "main_mcu_print_time"),
    ("batch_last_to_adapter", "Batch end to adapter (delivery / driver)",
     "main_mcu_print_time; adapter entry is a local clocksync estimate"),
    ("adapter_to_detector", "Adapter entry to detector entry", "host_monotonic"),
    ("detector_to_request", "Detector entry to stop request", "host_monotonic"))


def delivery_diagnostic(home, events, packets):
    """Decompose retained timestamps after cleanup; never infer missing stages.

    Match the source batch by its sample bounds AND enclosing callback. A
    pending/merged sample from an earlier source batch cannot be attributed to
    the current callback's source batch just because it contains the stop call.
    """
    segments = {key: {"duration_s": None, "clock_basis": basis,
                      "status": "unavailable", "reason": "matching retained evidence unavailable"}
                for key, _label, basis in DELIVERY_SEGMENTS}
    timing = home.get("stop_timing") or {}
    result = {"version": 1, "status": "unavailable", "segments": segments,
              "onset_to_host_dispatch_estimate_s": timing.get("onset_to_host_dispatch_estimate_s"),
              "decomposition_residual_s": None,
              "stop_call_duration_s": None, "stop_call_clock_basis": "host_monotonic",
              "hardware_receipt_time_s": None, "mcu_halt_time_s": None,
              "physical_contact_time_s": None,
              "packet_retention_omitted": home.get("batches_omitted"),
              "evidence": {},
              "interpretation": "Delivery / driver is an aggregate sample-age estimate, not pure transport latency. "
                  "Mixed-clock component sum is approximate; the residual records clocksync differences. "
                  "No MCU receipt, physical contact or physical halt timestamp is available."}
    hid = home["home_id"]
    stops = [event.get("fields", {}) for event in events
             if event.get("home_id") == hid and event.get("kind") == "stop_request"]
    onset, dispatch = timing.get("onset_print_time_s"), timing.get("host_dispatch_estimated_print_time_s")
    stop = next((item for item in reversed(stops)
                 if finite(dispatch) and item.get("request_estimated_print_time_s") == dispatch), None)
    if stop is None or not finite(onset):
        result["reason"] = "matching onset and clocksync-qualified stop request unavailable"
        return result
    request = stop.get("request_before_host_monotonic_s")
    if not finite(request):
        result["reason"] = "stop request host monotonic timestamp unavailable"
        return result
    result["evidence"].update(onset_print_time_s=onset,
        request_before_host_monotonic_s=request,
        request_estimated_print_time_s=dispatch,
        onset_sample=stop.get("onset_sample"))
    after = stop.get("request_after_host_monotonic_s")
    if finite(after) and after >= request:
        result["stop_call_duration_s"] = after - request

    def contains(fields, value, prefix):
        first, last = fields.get(prefix + "_first_print_time_s"), fields.get(prefix + "_last_print_time_s")
        return all(finite(v) for v in (first, last, value)) and first - 1e-9 <= value <= last + 1e-9

    def encloses(fields, first, last):
        entry, exit_time = fields.get("callback_entry_host_monotonic_s"), fields.get("callback_exit_host_monotonic_s")
        return all(finite(v) for v in (entry, exit_time, first, last)) and entry <= first <= last <= exit_time

    def same_attempt(fields):
        arm = stop.get("arm_print_time_s")
        return finite(arm) and fields.get("arm_print_time_s") == arm

    retained = [row for row in packets if row.get("home_id") == hid]
    detectors = [row for row in retained if row.get("source") in ("klipper_batch_detector", "beacon_native")
                 and same_attempt(row.get("fields", {}))
                 and row["fields"].get("sample_time_domain") == "main_mcu_print_time"
                 and contains(row["fields"], onset, "original")
                 and encloses(row["fields"], request, request)]
    identity = stop.get("onset_sample")
    if isinstance(identity, (tuple, list)) and len(identity) == 4:
        detectors = [row for row in detectors if
                     (row["fields"].get("original_start_clock"), row["fields"].get("original_delta_clock"),
                      row["fields"].get("original_sample_count")) == (identity[0], identity[1], identity[3])]
    if len(detectors) != 1:
        result["reason"] = ("ambiguous detector callback attribution" if detectors else
                            "matching detector callback unavailable; packet omissions or source gaps are possible")
        return result
    detector = detectors[0]
    df = detector["fields"]

    def duration(key, first, last):
        item = segments[key]
        if not all(finite(v) for v in (first, last)):
            item["reason"] = "required timestamp unavailable"
        elif last < first:
            item.update(status="inconsistent", reason="reversed timestamps; clock estimate or attribution unresolved",
                        observed_delta_s=last - first)
        else:
            item.update(duration_s=last - first, status="available", reason=None)

    detector_entry = df.get("callback_entry_host_monotonic_s")
    duration("detector_to_request", detector_entry, request)
    result["evidence"].update(detector_packet_sequence=detector.get("home_packet_sequence"),
        detector_source=detector.get("source"), detector_entry_host_monotonic_s=detector_entry)
    adapters = [row for row in retained
                if row.get("fields", {}).get("timestamp_provenance") == "configured_driver_decoded_batch_callback"
                and same_attempt(row["fields"])
                and row["fields"].get("sample_time_domain") == "main_mcu_print_time"
                and contains(row["fields"], onset, "original")
                and encloses(row["fields"], detector_entry, df.get("callback_exit_host_monotonic_s"))]
    if len(adapters) == 1:
        adapter = adapters[0]
        af = adapter["fields"]
        last, entry = af.get("original_last_print_time_s"), af.get("callback_entry_host_monotonic_s")
        estimate = af.get("callback_entry_estimated_print_time_s")
        duration("onset_to_batch_last", onset, last)
        duration("batch_last_to_adapter", last, estimate)
        duration("adapter_to_detector", entry, detector_entry)
        result["evidence"].update(adapter_packet_sequence=adapter.get("home_packet_sequence"),
            adapter_source=adapter.get("source"), source_batch_last_print_time_s=last,
            source_batch_first_print_time_s=af.get("original_first_print_time_s"),
            source_batch_sample_count=af.get("original_sample_count"),
            adapter_entry_host_monotonic_s=entry, adapter_entry_estimated_print_time_s=estimate)
    else:
        reason = ("ambiguous source batch attribution" if adapters else
                  "source adapter callback unavailable or onset outside current original batch; pending/source gap not inferred")
        for key in ("onset_to_batch_last", "batch_last_to_adapter", "adapter_to_detector"):
            segments[key]["reason"] = reason
    available = sum(item["status"] == "available" for item in segments.values())
    result["status"] = "complete" if available == len(segments) else "partial" if available else "unavailable"
    if result["status"] == "complete" and finite(result["onset_to_host_dispatch_estimate_s"]):
        result["decomposition_residual_s"] = (result["onset_to_host_dispatch_estimate_s"]
            - sum(item["duration_s"] for item in segments.values()))
    result["reason"] = None if result["status"] == "complete" else "see unavailable or inconsistent segments"
    return result


def export_delivery(folder, homes):
    """Portable per-home summary; CSV blanks retain unknown timing values."""
    write_json(folder / "delivery.json", {"version": 1, "homes": [
        {"home_id": home["home_id"], "axis": home["axis"], "phase": home["phase"],
         **home["delivery_diagnostic"]} for home in homes]})
    keys = [key for key, _label, _basis in DELIVERY_SEGMENTS]
    fields = ["home_id", "axis", "phase", "status", "onset_to_host_dispatch_estimate_ms"]
    fields += [key + "_ms" for key in keys]
    fields += ["decomposition_residual_ms", "stop_call_duration_ms", "packet_retention_omitted"]
    fields += [key + suffix for key in keys for suffix in ("_status", "_clock_basis", "_reason")]
    fields += ["mcu_halt_time_s", "physical_contact_time_s"]
    with (folder / "delivery.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for home in homes:
            item = home["delivery_diagnostic"]
            row = {key: home[key] for key in ("home_id", "axis", "phase")}
            row.update(status=item["status"], packet_retention_omitted=item["packet_retention_omitted"])
            for key in ("onset_to_host_dispatch_estimate", "decomposition_residual", "stop_call_duration"):
                value = item.get(key + "_s")
                row[key + "_ms"] = value * 1000 if finite(value) else None
            for key in keys:
                segment = item["segments"][key]
                value = segment["duration_s"]
                row[key + "_ms"] = value * 1000 if finite(value) else None
                row.update({key + suffix: segment[suffix[1:]] for suffix in ("_status", "_clock_basis", "_reason")})
            writer.writerow(row)


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
        self.timing = TimingTrace(self)

    def start(self):
        self.timing.start()

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
        self.timing.close()
        with self.observer_lock:
            if self.closed:
                return
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
                "before": snapshot(self.workflow), "raw": array("d"),
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
            self.raw_rows += retained
            home["capture"] = clean({key: value for key, value in capture.items() if key not in ("samples", "_record")})
            home["capture"].update(raw_rows_received=len(samples), raw_rows_retained=retained,
                                   raw_rows_omitted=len(samples) - retained,
                                   invalid_rows=invalid, retention="first reference prefix and last impact tail; original sample indices; no averaging",
                                   reserved_home_row_budget=budget,
                                   retained_sample_ranges=clean(ranges),
                                   omitted_sample_ranges=([[prefix, len(samples) - maximum + prefix]] if len(samples) > maximum else []),
                                   raw_units="print_time seconds, machine-axis g including gravity/bias",
                                   source="owner.stop_capture before workflow validation",
                                   truncation=bool(len(samples) > retained or capture.get("dropped")))
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
        dependencies = {"klippy", "homing", "bulk_sensor", "mcu", "stepper", "toolhead", "clocksync", "serialhdl", "reactor"}
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
        experimental = self.session.get("experiment")
        command = "ACCEL_HOMING_EXPERIMENT" if experimental else "ACCEL_HOMING_TROUBLESHOOT"
        root = Path(self.workflow.charts_directory) / ("experiment" if experimental else "troubleshoot")
        root.mkdir(parents=True, exist_ok=True)
        folder = root / self.session["run_id"]
        folder.mkdir()  # Unique UUID; deliberately never overwrite another report.
        self.folder = str(folder)
        homes = []
        profile = adxl_saturation_profile(self.context)
        for home in self.homes:
            public = {key: value for key, value in home.items() if key not in ("raw", "packet_prefix", "packet_tail")}
            public["stop_timing"] = self._stop_timing(home["home_id"])
            public["delivery_diagnostic"] = delivery_diagnostic(public, self.events,
                list(home["packet_prefix"]) + list(home["packet_tail"]))
            stop = next((event["fields"].get("request_estimated_print_time_s") for event in self.events
                         if event["home_id"] == home["home_id"] and event["kind"] == "stop_request"), None)
            diagnose_saturation(public, home["raw"], profile, stop)
            public["packet_retention"] = {"policy": "first prefix and rolling last tail; omitted middle",
                "prefix_limit": INITIAL_HOME_BATCHES, "tail_limit": MAX_HOME_BATCHES - INITIAL_HOME_BATCHES,
                "retained_sequence_ranges": [[rows[0]["home_packet_sequence"], rows[-1]["home_packet_sequence"]]
                                             for rows in (home["packet_prefix"], home["packet_tail"]) if rows]}
            name = "home-%03d-%s.csv.gz" % (home["home_id"], home["axis"])
            public["raw_file"] = name
            with gzip.open(folder / name, "wt", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("sample_index", "print_time_s", "x_g", "y_g", "z_g"))
                raw = home["raw"]
                offset = 0
                for first, end in home.get("raw_ranges", ()):
                    for original_index in range(first, end):
                        writer.writerow([original_index] + [value if finite(value) else "" for value in raw[offset:offset + 4]])
                        offset += 4
            homes.append(public)
        export_delivery(folder, homes)
        with gzip.open(folder / "packets.jsonl.gz", "wt", encoding="utf-8") as stream:
            for group in [self.batches] + [rows for home in self.homes for rows in (home["packet_prefix"], home["packet_tail"])]:
                for row in group:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
        report = {"schema": SCHEMA, "command": command,
                  "run_id": self.session["run_id"], "created_utc": self.started_utc,
                  "ending": clean(ending), "axes": self.session["axes"],
                  "samples_requested": self.session["samples_requested"],
                  "fixed_thresholds_g": clean(self.session["saved_thresholds"]),
                  "configured_speeds_mm_s": clean(self.session["configured_speeds"]),
                  "environment": self.context, "installed_identity": self._source_identity(),
                  "before": self.before, "after": snapshot(self.workflow), "homes": homes,
                  "events": self.events, "log": self._export_log(folder),
                  "timing": self.timing.export(folder),
                  "collector_closed_host_monotonic_s": self.closed_at,
                  "bounds": {"max_raw_rows": MAX_RAW_ROWS, "max_home_rows": MAX_HOME_ROWS,
                             "raw_rows_retained": self.raw_rows, "max_events": MAX_EVENTS,
                             "events_omitted": self.events_omitted, "max_batches": (MAX_HOMES + 1) * MAX_HOME_BATCHES,
                             "max_batches_per_home": MAX_HOME_BATCHES,
                             "batches_retained": len(self.batches) + sum(home["batches_retained"] for home in self.homes),
                             "batches_omitted": self.batches_omitted, "observer_errors": self.observer_errors},
                  "interpretation": ["Commanded positions and step counts are not measured physical carriage positions.",
                      "Saturation corrections apply to homes[].record/capture only. runtime_signal_quality, events and ending preserve the runtime report; detector and calibration decisions are unchanged.",
                      "Host stop dispatch does not establish MCU receipt time.",
                      "Clock domains are named in fields; do not subtract host monotonic from print_time.",
                      "callback_duration_s ends before recorder formatting/append and excludes that observer overhead; it is not full callback occupancy or transport latency.",
                      "Late callbacks holding an old recorder reference after collector_closed_host_monotonic_s are ignored; exported evidence is frozen.",
                      "Discovery is retained and excluded from measured-trial statistics.",
                      "Rejected/partial homes are retained; no automatic recalibration or threshold change.",
                      "Empty CSV cells represent invalid/missing numbers; see capture invalid_rows.",
                      "Charts use impact windows; raw retained CSV rows are not decimated."]}
        if experimental:
            report["experiment"] = clean(experimental)
            shadow_mode = experimental.get("mode") == "SHADOW"
            report["interpretation"] += [
                ("OFFLINE SHADOW: every physical home uses raw XYZ magnitude; frozen profile replay never controls the stop."
                 if shadow_mode else "Training and heldout reference homes use raw XYZ magnitude; candidates use a frozen fitted profile with raw bypass."),
                ("SAMPLES counts additional raw shadow homes only; discovery plus ten reference homes are additional."
                 if shadow_mode else "SAMPLES counts candidate homes only; discovery plus ten reference homes are additional."),
                "The same requested GC policy applies to every phase; this sequential comparison does not isolate GC causality.",
                "Profile fit and commanded repeatability do not prove physical contact or absence of lost steps.",
                "No automatic model, threshold, speed or configuration persistence. Normal G28 is unchanged.",
                "Freshness limits and no-data timeouts run on the host reactor and provide no MCU-independent stop deadline."]
            write_json(folder / "experiment.json", report["experiment"])
        write_json(folder / "manifest.json", report)
        (folder / "README.txt").write_text(
            command + " evidence\n\n"
            "Share the entire folder or ZIP. Read manifest.json first.\n"
            "home-*.csv.gz: every retained raw decoded XYZ sample before graph reduction, in g including gravity/bias.\n"
            "packets.jsonl.gz: bounded callback timing, pending/filtered packet context.\n"
            "timing.jsonl.gz: MCU host receive, driver/query/decode, detector/observer, reactor and GC timing. See manifest timing hook availability and omissions.\n"
            "delivery.json / delivery.csv: per-home source-batch tail, aggregate delivery/driver, adapter/detector and host request timing; clock bases and missing evidence are explicit.\n"
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


def _plot_timing(folder, home, report, rows, Figure, FigureCanvasAgg):
    """Host-clock view of the stop window; never mix in sensor print_time."""
    stops = [event["fields"].get("request_before_host_monotonic_s") for event in report["events"]
             if event["home_id"] == home["home_id"] and event["kind"] == "stop_request"]
    origin = next((stamp for stamp in stops if finite(stamp)), None)
    has_stop = origin is not None
    if origin is None:
        origin = max((row["exit_host_monotonic_s"] for row in rows), default=home["begin_host_monotonic_s"])
    # The full bounded trace is in timing.jsonl.gz; the picture focuses on the
    # dispatch window, so the 5 ms normal batches do not conceal an 87 ms pause.
    selected = [row for row in rows if row["exit_host_monotonic_s"] >= origin - .250
                and row["entry_host_monotonic_s"] <= origin + .100]
    figure = Figure(figsize=(12, 13), dpi=110)
    FigureCanvasAgg(figure)
    plots = figure.subplots(4, 1)
    def series(stage, value, plot, label):
        pairs = [((row["entry_host_monotonic_s"] - origin) * 1000, value(row))
                 for row in selected if row["stage"] == stage]
        pairs = [(x, y * 1000) for x, y in pairs if finite(y)]
        if pairs:
            plot.plot([x for x, _ in pairs], [y for _, y in pairs], ".-", markersize=3, linewidth=.7, label=label)
    for stage, label in (("mcu_bulk_handler", "Bulk handler entry gap"),
                         ("driver_batch", "Driver batch entry gap"),
                         ("adapter_processing", "Plugin entry gap")):
        series(stage, lambda row: row["entry_gap_s"], plots[0], label)
    def serial_lag(row):
        received = row["details"].get("serial_receive_host_monotonic_s")
        return row["entry_host_monotonic_s"] - received if finite(received) else None
    series("mcu_bulk_handler", serial_lag, plots[0], "Host serial receive to handler")
    series("reactor_heartbeat", lambda row: row["details"].get("lateness_s"), plots[0], "Reactor timer lateness")
    plots[0].set(title="Delivery gaps and host scheduling (missing stages are unavailable)", ylabel="ms")
    stages = ("sensor_status_query", "driver_queue_drain", "sensor_pull", "driver_decode",
              "driver_batch", "adapter_processing", "detector_with_observer", "gc")
    for index, stage in enumerate(stages):
        intervals = [((row["entry_host_monotonic_s"] - origin) * 1000,
                      max(row["duration_s"] * 1000, .05)) for row in selected if row["stage"] == stage]
        plots[1].broken_barh(intervals, (index - .35, .7), facecolors="C%d" % index)
    plots[1].set_yticks(range(len(stages)))
    plots[1].set_yticklabels(stages, fontsize=8)
    plots[1].invert_yaxis()
    plots[1].set_title("Existing call occupancy; nested intervals overlap (minimum drawn width 0.05 ms)")
    for stage in ("recorder_batch", "recorder_event", "adapter_observer"):
        series(stage, lambda row: row["duration_s"], plots[2], stage + " duration")
    series("sensor_status_query", lambda row: row["process_cpu_s"], plots[2], "Process CPU during status query")
    series("reactor_heartbeat", lambda row: row["details"].get("interval_process_cpu_s"), plots[2], "Process CPU since previous timer")
    plots[2].set(title="Observer cost and process-wide CPU (includes other threads)", ylabel="ms",
                 xlabel="Host monotonic ms from stop dispatch" if has_stop else "Host monotonic ms from last observation; stop unavailable")
    for plot in plots[:3]:
        plot.set_xlim(-250, 100)
        plot.grid(True, alpha=.25)
        if has_stop:
            plot.axvline(0, color="purple", linestyle="--", label="Host stop dispatch")
        if plot.lines:
            plot.legend(fontsize=7, loc="upper left")
    delivery = home.get("delivery_diagnostic") or {}
    for index, (key, label, _basis) in enumerate(DELIVERY_SEGMENTS):
        item = delivery.get("segments", {}).get(key, {})
        duration = item.get("duration_s")
        if finite(duration):
            plots[3].barh(index, duration * 1000, color="C%d" % index)
            plots[3].annotate(" %.3f ms" % (duration * 1000), (duration * 1000, index),
                              va="center", fontsize=8)
        else:
            plots[3].text(.01, index, "unavailable: " + str(item.get("reason") or "no retained evidence"),
                          transform=plots[3].get_yaxis_transform(), va="center", fontsize=7)
    plots[3].set_yticks(range(len(DELIVERY_SEGMENTS)))
    plots[3].set_yticklabels([label for _key, label, _basis in DELIVERY_SEGMENTS], fontsize=8)
    plots[3].invert_yaxis()
    plots[3].margins(x=.25)
    residual = delivery.get("decomposition_residual_s")
    plots[3].set(title="Per-home decomposition (%s); clock-mapping residual %s\n"
                      "First two: main-MCU print time / clocksync estimate; last two: host monotonic" % (
                          delivery.get("status", "unavailable"),
                          "%.6f ms" % (residual * 1000) if finite(residual) else "unavailable"),
                 xlabel="Duration (ms); delivery / driver includes sample age, not pure transport latency")
    plots[3].grid(True, axis="x", alpha=.25)
    retention = report.get("timing", {}).get("homes", {}).get(str(home["home_id"]), {})
    figure.suptitle("Home %03d %s: host timing; retained %s, omitted %s\n"
                   "No MCU halt timestamp; timing-hook bookkeeping is not included in observer cost" % (
                       home["home_id"], home["axis"].upper(), retention.get("retained", 0), retention.get("omitted", 0)))
    figure.tight_layout(rect=(0, 0, 1, .94))
    figure.savefig(folder / ("home-%03d-%s-timing.png" % (home["home_id"], home["axis"])))
    figure.clear()


def _plot_experiment(folder, report, Figure, FigureCanvasAgg):
    """Separate sequential phases; never label commanded travel as accuracy."""
    has_shadow = any(home.get("shadow") for home in report["homes"])
    figure = Figure(figsize=(12, 15 if has_shadow else 12), dpi=110)
    FigureCanvasAgg(figure)
    plots = figure.subplots(4 if has_shadow else 3, 1)
    for phase, marker in (("discovery", "x"), ("training", "o"),
                          ("holdout", "s"), ("candidate", "^"), ("shadow", "D")):
        group = [home for home in report["homes"] if home["phase"] == phase]
        ids = [home["home_id"] for home in group]
        records = [home.get("record") or {} for home in group]
        numeric = lambda values: [value if finite(value) else math.nan for value in values]
        travel = [record.get("travel_mm_exact", record.get("travel_mm")) for record in records]
        overshoot = [abs(record["halt_travel_mm_signed"]) - abs(record["trigger_travel_mm_signed"])
                     if all(finite(record.get(key)) for key in ("halt_travel_mm_signed", "trigger_travel_mm_signed"))
                     else None for record in records]
        delay = [home.get("stop_timing", {}).get("onset_to_host_dispatch_estimate_s") for home in group]
        for plot, values in zip(plots, (travel, overshoot,
                [value * 1000 if finite(value) else None for value in delay])):
            plot.plot(ids, numeric(values), marker + "-", label=phase)
        if has_shadow:
            plots[3].plot(ids, numeric([(home.get("shadow") or {}).get("reference_delta_ms") for home in group]),
                          marker + "-", label=phase)
    plots[0].set(title="Commanded travel by sequential phase (not measured physical position)", ylabel="mm")
    plots[1].set(title="Commanded halt minus trigger distance", ylabel="mm")
    plots[2].set(title="Sensor onset to host dispatch estimate (MCU receipt unknown)", ylabel="ms", xlabel="Home ID")
    if has_shadow:
        plots[3].axhline(0, color="black", linewidth=.8)
        plots[3].set(title="OFFLINE SHADOW decision relative to original raw software reference\n"
                          "Negative values are earlier software signals; physical contact is unverified",
                    ylabel="Decision - reference, ms", xlabel="Home ID")
    home_ids = [home["home_id"] for home in report["homes"]]
    for plot in plots:
        plot.grid(True, alpha=.25)
        plot.legend(fontsize=8)
        if home_ids:
            plot.set_xlim(min(home_ids) - .5, max(home_ids) + .5)
    if not any(finite(home.get("stop_timing", {}).get("onset_to_host_dispatch_estimate_s"))
               for home in report["homes"]):
        plots[2].text(.5, .5, "Timing unavailable: stop dispatch clock mapping not established",
                      transform=plots[2].transAxes, ha="center", va="center", fontsize=10)
    experiment = report["experiment"]
    fit = experiment.get("fit") or {}
    profile = ((fit.get("shadow") or {}).get("profile") if has_shadow else fit.get("profile")) or {}
    weights = profile.get("weights_xyz")
    weight_text = (", ".join("%.3g" % value for value in weights)
                   if isinstance(weights, (tuple, list)) and all(finite(value) for value in weights)
                   else "unavailable")
    threshold_text = "%.3g" % profile["threshold_g"] if finite(profile.get("threshold_g")) else "unavailable"
    figure.suptitle("ACCEL_HOMING_EXPERIMENT: %sfrozen profile comparison\n"
                   "Weights XYZ: %s; weighted threshold: %s g; GC guard all phases: %s\n"
                   "Sequential comparison; no independent GC or physical-contact inference" % (
                       "OFFLINE SHADOW / " if has_shadow else "", weight_text, threshold_text,
                       experiment.get("gc_guard_requested", "unknown")), fontsize=11)
    figure.tight_layout(rect=(0, 0, 1, .92))
    figure.savefig(folder / "experiment-phases.png")
    figure.clear()


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
        timing = {}
        if report.get("timing"):
            with gzip.open(folder / "timing.jsonl.gz", "rt", encoding="utf-8") as stream:
                for number, line in enumerate(stream):
                    if number >= (MAX_HOMES + 1) * (MAX_TIMING_ROWS + MAX_SLOW_TIMING_ROWS):
                        raise ValueError("timing plot input exceeds bound")
                    row = json.loads(line)
                    timing.setdefault(row["home_id"], []).append(row)
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
        figure.suptitle(report.get("command", "ACCEL_HOMING_TROUBLESHOOT") + ": " + report["ending"].get("result", "unknown"))
        figure.tight_layout(rect=(0, 0, 1, .97))
        figure.savefig(folder / "summary.png")
        figure.clear()
        if report.get("experiment"):
            _plot_experiment(folder, report, Figure, FigureCanvasAgg)
        for home in homes:
            record, capture = home.get("record") or {}, home["capture"]
            shadow = home.get("shadow") or {}
            onset = record.get("onset_time_s")
            if not finite(onset):
                onset = (capture.get("detector") or {}).get("onset_time_s")
            dispatch = next((event["fields"].get("request_estimated_print_time_s")
                for event in report["events"] if event["home_id"] == home["home_id"]
                and event["kind"] == "stop_request"), None)
            anchor = onset if finite(onset) else dispatch
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
                    before = .025 if finite(onset) else .125
                    first = anchor - before if finite(anchor) else None
                    evidence = shadow.get("evidence_time_s")
                    if finite(first) and finite(evidence):
                        first = min(first, evidence - .005)
                    if not finite(anchor) or first <= sample[0] <= anchor + .100:
                        rows.append(sample)
            # Window retains all samples/outliers. Bound unknown-onset plots by
            # displaying a prefix explicitly; raw files remain unchanged.
            omitted = max(0, len(rows) - 20000)
            # Preserve both the early shadow evidence and the original onset
            # when a wide replay span exceeds the plot-only bound.
            rows = rows[:10000] + [[math.nan] * 4] + rows[-10000:] if omitted else rows
            wide_shadow = finite(onset) and finite(shadow.get("evidence_time_s")) and shadow["evidence_time_s"] < onset - .025
            figure = Figure(figsize=(11, 9 if wide_shadow else 5), dpi=110)
            FigureCanvasAgg(figure)
            if wide_shadow:
                plot, overview = figure.subplots(2, 1)
            else:
                plot, overview = figure.subplots(), None
            origin = anchor if finite(anchor) else rows[0][0] if rows else 0
            times = [(row[0] - origin) * 1000 for row in rows]
            for index, label in enumerate("XYZ"):
                plot.plot(times, [row[index + 1] - baseline[index] for row in rows], label=label, linewidth=.8)
            plot.plot(times, [math.sqrt(sum((row[index + 1] - baseline[index]) ** 2 for index in range(3))) for row in rows], label="Original per-home XYZ reference" if shadow else "XYZ norm", color="black", linewidth=1)
            experimental = (capture.get("detector") or {}).get("experimental") or {}
            profile = home.get("shadow_profile") or ((report.get("experiment") or {}).get("fit") or {}).get("profile") or {}
            weights = profile.get("weights_xyz")
            if (home["phase"] == "candidate" or shadow) and has_baseline and isinstance(weights, list) and len(weights) == 3 \
                    and all(finite(value) and value >= 0 for value in weights):
                plot.plot(times, [math.sqrt(sum(weights[index] * (row[index + 1] - baseline[index]) ** 2
                                                 for index in range(3))) for row in rows],
                          label="Frozen weighted XYZ score (offline shadow)" if shadow else "Frozen weighted XYZ score", color="#007b83", linewidth=1)
                if finite(profile.get("threshold_g")):
                    plot.axhline(profile["threshold_g"], color="#007b83", linestyle="--",
                                 label="Frozen weighted threshold")
                for key, label in (("capture_evidence_time_s", "weighted first evidence, retained row"),
                                   ("capture_decision_time_s", "experimental decision, retained row")):
                    if finite(experimental.get(key)):
                        plot.axvline((experimental[key] - origin) * 1000, linestyle=":", label=label)
                for key, label in (("evidence_time_s", "Shadow first evidence (offline)"),
                                   ("decision_time_s", "Shadow decision (offline)")):
                    if finite(shadow.get(key)):
                        plot.axvline((shadow[key] - origin) * 1000, linestyle=":", label=label)
            threshold = record.get("threshold_g")
            if finite(threshold):
                plot.axhline(threshold, color="#d17b00", linestyle="--",
                             label="Raw bypass threshold" if home["phase"] == "candidate" else "Active detector threshold")
            quality = home.get("saturation_diagnostic", {})
            if quality.get("available"):
                for index, axis in enumerate("xyz"):
                    lower, upper = quality["axis_endpoints_g"][axis]
                    tolerance = abs(quality["axes_map"][index][1]) / 9806.65 * quality["count_tolerance"]
                    limited = [(t, row[index + 1] - baseline[index]) for t, row in zip(times, rows)
                               if row[index + 1] <= lower + tolerance or row[index + 1] >= upper - tolerance]
                    if limited:
                        plot.scatter([t for t, _ in limited], [v for _, v in limited], marker="x", s=20,
                                     label=axis.upper() + " native range limit", zorder=5)
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
            plot.set(title="Home %03d %s %s; %s\nRaw omitted %s; plot omitted %d; capture range-limited axis values %s%s" % (
                home["home_id"], home["axis"].upper(), home["phase"], record.get("result", "partial; see manifest error"),
                capture.get("raw_rows_omitted", "unknown"), omitted, quality.get("range_limit_axis_values", "unknown"),
                "\nOFFLINE SHADOW: actual stop uses raw XYZ; physical contact unverified" if shadow else ""),
                xlabel=("ms from retained onset" if finite(onset) else
                        "ms from host dispatch estimate (onset unavailable)" if finite(dispatch) else
                        "ms from first retained sample (onset/stop unavailable)"),
                ylabel="g after reference subtraction" if has_baseline else "raw g; reference unavailable (gravity/bias retained)")
            plot.legend(fontsize=7)
            plot.grid(True, alpha=.25)
            if overview is not None:
                plot.set_xlim(-25, 100)
                overview.plot(times, [math.sqrt(sum((row[index + 1] - baseline[index]) ** 2 for index in range(3)))
                                     for row in rows], label="Original per-home XYZ reference", color="black", linewidth=.8)
                if has_baseline and isinstance(weights, list) and len(weights) == 3 \
                        and all(finite(value) and value >= 0 for value in weights):
                    overview.plot(times, [math.sqrt(sum(weights[index] * (row[index + 1] - baseline[index]) ** 2
                                                       for index in range(3))) for row in rows],
                                  label="Frozen weighted XYZ score (offline shadow)", color="#007b83", linewidth=.8)
                if finite(profile.get("threshold_g")):
                    overview.axhline(profile["threshold_g"], color="#007b83", linestyle="--", label="Frozen weighted threshold")
                if finite(threshold):
                    overview.axhline(threshold, color="#d17b00", linestyle="--", label="Active raw threshold")
                for key, label in (("evidence_time_s", "Shadow first evidence (offline)"),
                                   ("decision_time_s", "Shadow decision (offline)"),
                                   ("reference_time_s", "Original software reference")):
                    if finite(shadow.get(key)):
                        overview.axvline((shadow[key] - origin) * 1000, linestyle=":", label=label)
                overview.set(title="Earlier OFFLINE SHADOW evidence overview; phase: %s; compatibility: %s\n"
                                   "Upper panel keeps original-onset detail; software signals do not establish physical contact" % (
                                       shadow.get("phase", "unknown"), shadow.get("compatible", "unknown")),
                             xlabel="ms from original retained onset", ylabel=plot.get_ylabel())
                overview.grid(True, alpha=.25)
                overview.legend(fontsize=7)
            figure.tight_layout()
            figure.savefig(folder / ("home-%03d-%s.png" % (home["home_id"], home["axis"])))
            figure.clear()
            if report.get("timing"):
                _plot_timing(folder, home, report, timing.get(home["home_id"], []), Figure, FigureCanvasAgg)


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--render-worker":
        _worker(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit("Internal troubleshoot plotting worker")
