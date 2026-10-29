# PNG-only accelerometer homing charts. GPLv3.
"""Render completed PNGs in an isolated process; keep provenance in memory."""
import collections
import datetime
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import re
import statistics
import struct
import subprocess
import sys
import tempfile
import textwrap
import time
import zlib


_MAX_PNG_BYTES = 32 * 1024 * 1024
_MAX_PNG_PIXELS = 12 * 1024 * 1024
_MAX_INPUT_BYTES = 32 * 1024 * 1024
_MAX_SAMPLES = 500000
_POPULATION = "population (divide by N)"
_NOISE_WINDOWS = 3
_NOISE_WINDOW_SECONDS = 1.0
_Receipt = collections.namedtuple("_Receipt", (
    "kind", "run_id", "axis", "iteration", "record_json", "metadata_json",
    "trace", "png_path", "png_sha256", "simulated"))


class CapturePaths(dict):
    """Public file paths plus a private immutable, non-JSON provenance receipt."""
    __slots__ = ("_receipt",)

    def __init__(self, paths, receipt):
        super().__init__(paths)
        self._receipt = receipt

    @property
    def receipt(self):
        return self._receipt


def default_dependency_directory(prefix=None, cache_tag=None):
    prefix = sys.prefix if prefix is None else prefix
    cache_tag = sys.implementation.cache_tag if cache_tag is None else cache_tag
    if not isinstance(cache_tag, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", cache_tag):
        raise ValueError("Invalid chart Python cache tag")
    identity = hashlib.sha256(os.path.abspath(prefix).encode("utf-8")).hexdigest()[:12]
    return Path(__file__).resolve().parent / ".plot-deps" / (cache_tag + "-" + identity)


def _json(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":"))


def _check_running(check_running):
    if check_running is not None:
        check_running()


def _check_png(data, decode_pixels=True):
    """Check complete bounded, noninterlaced PNG bytes without importing Pillow."""
    if not isinstance(data, bytes) or not 45 <= len(data) <= _MAX_PNG_BYTES \
            or data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("Invalid or oversized PNG chart")
    offset, header, ended, compressed = 8, None, False, []
    seen_idat = False
    while offset < len(data):
        if offset + 12 > len(data):
            raise ValueError("Truncated PNG chart")
        size, kind = struct.unpack(">I4s", data[offset:offset + 8])
        end = offset + 12 + size
        if end > len(data):
            raise ValueError("Truncated PNG chart chunk")
        chunk = data[offset + 8:offset + 8 + size]
        checksum = struct.unpack(">I", data[offset + 8 + size:end])[0]
        if zlib.crc32(kind + chunk) & 0xffffffff != checksum:
            raise ValueError("PNG chart CRC mismatch")
        if kind == b"IHDR":
            if header is not None or offset != 8 or size != 13:
                raise ValueError("Invalid PNG chart header")
            header = struct.unpack(">IIBBBBB", chunk)
            width, height, depth, color, compression, filtering, interlace = header
            if not width or not height or width * height > _MAX_PNG_PIXELS \
                    or depth != 8 or color not in (0, 2, 4, 6) \
                    or compression or filtering or interlace:
                raise ValueError("Unsupported or oversized PNG chart dimensions")
        elif kind == b"IDAT":
            if header is None:
                raise ValueError("PNG data precedes its header")
            seen_idat = True
            compressed.append(chunk)
        elif kind == b"IEND":
            if size or not seen_idat or end != len(data):
                raise ValueError("Invalid PNG chart ending")
            ended = True
        elif kind[0] & 32 == 0 and kind != b"PLTE":
            raise ValueError("Unsupported critical PNG chart chunk")
        offset = end
        if ended:
            break
    if header is None or not ended:
        raise ValueError("Incomplete PNG chart")
    if not decode_pixels:
        return header[0], header[1]
    channels = {0: 1, 2: 3, 4: 2, 6: 4}[header[3]]
    row_bytes = header[0] * channels + 1
    expected = row_bytes * header[1]
    decoder = zlib.decompressobj()
    raw = decoder.decompress(b"".join(compressed), expected + 1)
    if len(raw) != expected or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail \
            or any(raw[index] > 4 for index in range(0, len(raw), row_bytes)):
        raise ValueError("Invalid PNG chart pixel stream")
    return header[0], header[1]


class PNGRenderer:
    """Run only the selected Python plotting worker while yielding to Klippy."""
    def __init__(self, python_executable=None, dependency_directory=None, reactor=None,
                 timeout_s=90.0):
        self.python_executable = str(python_executable or sys.executable)
        self.dependency_directory = str(dependency_directory or default_dependency_directory())
        self.reactor = reactor
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("Invalid PNG rendering deadline")

    def render(self, payload, check_running=None):
        encoded = _json(payload).encode("utf-8")
        if len(encoded) > _MAX_INPUT_BYTES:
            raise ValueError("PNG chart render input exceeds its size limit")
        _check_running(check_running)
        with tempfile.TemporaryDirectory(prefix="accel_homing_png_worker_") as temporary:
            folder = Path(temporary)
            source, target = folder / "input.json", folder / "output.png"
            source.write_bytes(encoded)
            with open(folder / "worker.log", "wb") as log:
                process = subprocess.Popen(
                    [self.python_executable, "-B", "-I", str(Path(__file__).resolve()),
                     "--render-worker", str(source), str(target), self.dependency_directory],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=log)
                deadline = time.monotonic() + self.timeout_s
                try:
                    while process.poll() is None:
                        _check_running(check_running)
                        if os.fstat(log.fileno()).st_size > 1024 * 1024:
                            raise ValueError("PNG renderer exceeded its diagnostic-log limit")
                        if time.monotonic() >= deadline:
                            raise ValueError("PNG renderer exceeded its deadline")
                        if self.reactor is None:
                            time.sleep(0.02)
                        else:
                            self.reactor.pause(self.reactor.monotonic() + 0.02)
                    _check_running(check_running)
                    if process.returncode:
                        log.flush()
                        with open(folder / "worker.log", "rb") as detail_stream:
                            detail = detail_stream.read(8000).decode("utf-8", "replace")
                        raise ValueError("PNG renderer failed; plotting dependencies must be available "
                                         "for the selected Klipper Python: " + detail.strip())
                    if not target.is_file() or target.stat().st_size > _MAX_PNG_BYTES:
                        raise ValueError("PNG renderer did not produce a bounded completed image")
                    data = target.read_bytes()
                    _check_png(data, decode_pixels=False)
                    _check_running(check_running)
                    return data
                finally:
                    if process.poll() is None:
                        process.kill()
                    process.wait(timeout=2.0)


def preflight_renderer(renderer=None, check_running=None):
    """Exercise the actual worker before any chart-producing workflow moves."""
    data = (renderer or PNGRenderer()).render({"kind": "preflight"}, check_running=check_running)
    _check_png(data, decode_pixels=False)
    _check_running(check_running)
    return True


def _clock():
    instant = datetime.datetime.now(datetime.timezone.utc)
    local = instant.astimezone()
    offset = local.strftime("%z")
    return {"created_utc": instant.isoformat(),
            "created_local": local.isoformat(),
            "timestamp": local.strftime("%Y-%m-%d %H:%M:%S"),
            "timezone": "UTC%s:%s" % (offset[:3], offset[3:]),
            "filename_time": local.strftime("%Y-%m-%d %H:%M:%S")}


def _identity(run_id, axis):
    if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
        raise ValueError("Invalid capture run ID")
    if axis not in ("x", "y"):
        raise ValueError("Capture axis must be x or y")


def _envelope(samples, buckets=512):
    """Keep every bucket's first/last and all signed XYZ extrema."""
    stride = max(1, int(math.ceil(len(samples) / float(buckets))))
    result = []
    for offset in range(0, len(samples), stride):
        bucket = samples[offset:offset + stride]
        chosen = {0, len(bucket) - 1}
        for channel in (1, 2, 3):
            chosen.update((min(range(len(bucket)), key=lambda i: bucket[i][channel]),
                           max(range(len(bucket)), key=lambda i: bucket[i][channel])))
        result.extend(tuple(bucket[index]) for index in sorted(chosen))
    return tuple(result)


def _capture_data(capture, record):
    if not isinstance(capture, dict) or capture.get("dropped", 0) \
            or capture.get("fault") or capture.get("error"):
        raise ValueError("Sensor capture fault or buffer overflow; refusing an incomplete PNG")
    samples = capture.get("samples", ())
    if not isinstance(samples, (tuple, list)) or not 2 <= len(samples) <= _MAX_SAMPLES:
        raise ValueError("No bounded accelerometer capture for this home")
    previous = None
    for sample in samples:
        if not isinstance(sample, (tuple, list)) or len(sample) != 4 \
                or not all(_finite(value) for value in sample) \
                or (previous is not None and sample[0] <= previous):
            raise ValueError("Invalid or non-increasing accelerometer sample")
        previous = sample[0]
    baseline = capture.get("baseline_g")
    if baseline is None:
        arm = record.get("arm_time_s", samples[0][0])
        quiet = [row for row in samples if row[0] < arm]
        if not quiet:
            raise ValueError("No pre-arm baseline for measured acceleration")
        baseline = tuple(math.fsum(row[index] for row in quiet) / len(quiet) for index in (1, 2, 3))
    if not isinstance(baseline, (tuple, list)) or len(baseline) != 3 \
            or not all(_finite(value) for value in baseline):
        raise ValueError("Invalid accelerometer baseline")
    delta = [(row[0] - samples[0][0],) + tuple(row[i + 1] - baseline[i] for i in range(3))
             for row in samples]
    metadata = {"sample_count": len(samples), "baseline_g": list(baseline),
                "first_sample_time_s": samples[0][0], "last_sample_time_s": samples[-1][0],
                "absolute_peak_g": {axis: max(abs(row[index]) for row in delta)
                                    for index, axis in enumerate("xyz", 1)}}
    _json(record)
    return _envelope(delta), metadata


def _publish(directory, label, clock, data, check_running=None):
    _check_png(data, decode_pixels=False)
    _check_running(check_running)
    destination = Path(directory).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    stem = label + " - " + clock["filename_time"]
    # Moonraker adds watches to new directories asynchronously. Keep this root
    # durable and publish files only; a directory created then renamed/removed
    # here can disappear before its queued watch registration runs.
    descriptor, temporary = tempfile.mkstemp(prefix=".accel_homing_png_", suffix=".tmp",
                                              dir=str(destination))
    identity = os.fstat(descriptor)
    identity = (identity.st_dev, identity.st_ino)
    def owned(path):
        try:
            current = os.lstat(path)
        except FileNotFoundError:
            return False
        return (current.st_dev, current.st_ino) == identity
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
        for collision in range(100):
            _check_running(check_running)
            name = stem if collision == 0 else stem + "_%02d" % collision
            published = destination / (name + ".png")
            try:
                # Staging and final files share a directory/filesystem. link()
                # atomically exposes completed bytes and refuses an existing
                # name, unlike POSIX rename(). TMPDIR cannot relocate staging.
                os.link(temporary, published)
            except OSError as exc:
                if exc.errno == errno.EEXIST or os.path.lexists(published):
                    continue
                raise OSError("Atomic no-overwrite PNG publication requires same-directory "
                              "hard links: %s" % exc) from exc
            try:
                # Moonraker completes a CREATE event on CLOSE_WRITE for that
                # name. A link of an already-closed inode supplies CREATE only.
                # Open/close the owned final name without creating, truncating
                # or writing bytes, before cancellation can skip completion.
                flags = os.O_WRONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_BINARY", 0)
                completion = os.open(published, flags)
                try:
                    current = os.fstat(completion)
                    if (current.st_dev, current.st_ino) != identity:
                        raise OSError(errno.ESTALE, "PNG publication path changed before completion")
                finally:
                    os.close(completion)
            except Exception:
                if owned(published):
                    os.unlink(published)
                raise
            _check_running(check_running)
            return str(published)
        raise ValueError("PNG chart filename collisions exceeded their bounded limit")
    finally:
        if owned(temporary):
            os.unlink(temporary)


def write_capture(directory, run_id, axis, capture, record, simulated=False,
                  renderer=None, check_running=None):
    _identity(run_id, axis)
    trace, metadata = _capture_data(capture, record)
    clock = _clock()
    phase = record.get("workflow_phase")
    sample = record.get("accuracy_sample")
    suffix = (" - Qualifier" if phase == "discovery" else " - Home %02d" % sample
              if type(sample) is int and sample > 0 else "")
    title = "Accel Homing %s%s" % (axis.upper(), suffix)
    payload = {"kind": "capture", "axis": axis, "record": record, "trace": trace,
               "capture": metadata, "title": title, "simulated": bool(simulated), **clock}
    data = (renderer or PNGRenderer()).render(payload, check_running=check_running)
    path = _publish(directory, title, clock, data, check_running)
    receipt = _Receipt("capture", run_id, axis, record.get("workflow_attempt"),
                       _json(record), _json(metadata), _envelope(trace, 64), path,
                       hashlib.sha256(data).hexdigest(), bool(simulated))
    _check_running(check_running)
    return CapturePaths({"png": path}, receipt)


def _capture_reference(directory, paths, run_id, axis, record, simulated, check_running=None):
    _check_running(check_running)
    if paths is None:
        return None
    expected_id = "%s_a%03d" % (run_id, record["workflow_attempt"])
    if not isinstance(paths, CapturePaths) or set(paths) != {"png"}:
        raise ValueError("Accuracy PNG requires its immutable current-run capture receipt")
    receipt = paths.receipt
    if not isinstance(receipt, _Receipt) or receipt.kind != "capture" \
            or receipt.run_id != expected_id or receipt.axis != axis \
            or receipt.iteration != record["workflow_attempt"] \
            or receipt.record_json != _json(record) or receipt.simulated != bool(simulated):
        raise ValueError("Accuracy PNG capture receipt does not match this current-run record")
    path = paths.get("png")
    if not isinstance(path, str) or path != receipt.png_path:
        raise ValueError("Accuracy PNG capture path does not match its receipt")
    candidate = Path(path).expanduser().resolve(strict=True)
    if not candidate.is_file() or candidate.suffix.lower() != ".png" \
            or candidate.parent != directory:
        raise ValueError("Accuracy PNG capture path does not belong to this output directory")
    if candidate.stat().st_size > _MAX_PNG_BYTES:
        raise ValueError("Accuracy PNG capture exceeds its size limit")
    data = candidate.read_bytes()
    if hashlib.sha256(data).hexdigest() != receipt.png_sha256:
        raise ValueError("Accuracy PNG capture bytes changed after export")
    _check_png(data, decode_pixels=False)
    _check_running(check_running)
    return paths


def validate_accuracy(directory, run_id, axis_results, result="complete", reason=None,
                      simulated=False, renderer=None, check_running=None,
                      capture_directory=None):
    _identity(run_id, "x")
    def cooperate():
        _check_running(check_running)
        reactor = getattr(renderer, "reactor", None)
        if reactor is not None:
            reactor.pause(reactor.monotonic() + 0.001)
        _check_running(check_running)
    source = Path(directory if capture_directory is None else capture_directory).expanduser().resolve()
    axes = _validate_accuracy(source, run_id, axis_results,
                              result, reason, simulated, check_running=cooperate)
    _check_running(check_running)
    return axes


def write_accuracy(directory, run_id, axis_results, result, reason=None, simulated=False,
                   renderer=None, check_running=None, capture_directory=None,
                   published_paths=None):
    axes = validate_accuracy(directory, run_id, axis_results, result, reason, simulated,
                             renderer=renderer, check_running=check_running,
                             capture_directory=capture_directory)
    clock = _clock()
    # Retain already-published axes if a later render/publication fails.
    paths = {} if published_paths is None else published_paths
    if not isinstance(paths, dict) or paths:
        raise ValueError("Accuracy published paths must start as an empty mapping")
    for axis, item in axes.items():
        traces = [reference.receipt.trace for reference in item["capture_paths"] if reference is not None]
        payload = {"kind": "accuracy", "axis": axis, "result": result, "reason": reason,
                   "requested": item["requested_samples"], "trials": item["trials"],
                   "statistics": item["statistics"], "traces": traces,
                   "discovery": item.get("discovery"), "simulated": bool(simulated), **clock}
        data = (renderer or PNGRenderer()).render(payload, check_running=check_running)
        paths[axis + "_png"] = _publish(directory, "Accel Homing Accuracy " + axis.upper(),
                                        clock, data, check_running)
    _check_running(check_running)
    return paths


def write_noise_capture(directory, run_id, iteration, capture, record, simulated=False,
                        renderer=None, check_running=None):
    """Freeze a validated window in memory; never create an intermediate file."""
    _noise_identity(run_id, iteration)
    samples, baseline, delta = _validate_noise_capture(run_id, iteration, capture, record)
    elapsed = [(row[0] - samples[0][0], *row[1:]) for row in delta]
    metadata = {"sample_count": len(samples), "baseline_g": list(baseline)}
    receipt = _Receipt("noise", run_id, None, iteration, _json(record), _json(metadata),
                       _envelope(elapsed), None, None, bool(simulated))
    _check_running(check_running)
    return CapturePaths({}, receipt)


def _noise_reference(run_id, iteration, item, simulated):
    record = item.get("record")
    _check_noise_record(run_id, iteration, record)
    paths = item.get("capture_paths")
    if not isinstance(paths, CapturePaths) or paths:
        raise ValueError("Stationary noise requires its in-memory capture receipt")
    receipt = paths.receipt
    if not isinstance(receipt, _Receipt) or receipt.kind != "noise" or receipt.run_id != run_id \
            or receipt.iteration != iteration or receipt.record_json != _json(record) \
            or receipt.simulated != bool(simulated):
        raise ValueError("Stationary noise capture receipt does not match its current-run window")
    return receipt


def write_noise(directory, run_id, iterations, recommendation, result, reason=None, simulated=False,
                renderer=None, check_running=None):
    _noise_identity(run_id, 1)
    if result not in ("complete", "aborted", "incomplete", "failed", "cancelled") \
            or not isinstance(iterations, list) or len(iterations) > 3 \
            or (reason is not None and (not isinstance(reason, str) or len(reason) > 4000)):
        raise ValueError("Invalid stationary noise PNG report")
    if result == "complete" and len(iterations) != 3:
        raise ValueError("A complete stationary noise report requires three valid windows")
    if result != "complete" and recommendation is not None:
        raise ValueError("Incomplete stationary noise cannot recommend a calibration candidate")
    receipts, context, previous_end = [], None, None
    for iteration, item in enumerate(iterations, 1):
        if not isinstance(item, dict):
            raise ValueError("Invalid stationary noise iteration")
        receipt = _noise_reference(run_id, iteration, item, simulated)
        record = item["record"]
        if previous_end is not None and record["first_sample_time_s"] <= previous_end:
            raise ValueError("Stationary noise windows overlap or repeat earlier samples")
        previous_end = record["last_sample_time_s"]
        current = _json({key: record.get(key) for key in
                         ("sensor", "backend", "axis_context", "full_scale_g", "sensor_context")})
        if context is not None and current != context:
            raise ValueError("Stationary noise sensor context changed between windows")
        context = current
        receipts.append(receipt)
    if result == "complete":
        _validate_noise_recommendation(recommendation, iterations)
    clock = _clock()
    payload = {"kind": "noise", "result": result, "reason": reason,
               "records": [item["record"] for item in iterations],
               "traces": [receipt.trace for receipt in receipts],
               "recommendation": recommendation, "simulated": bool(simulated), **clock}
    data = (renderer or PNGRenderer()).render(payload, check_running=check_running)
    path = _publish(directory, "Accel Homing Noise", clock, data, check_running)
    _check_running(check_running)
    return {"png": path}


def beta_motion_statistics(capture, record):
    """Analyze full beta rows by actual planner phase, never a plot envelope."""
    _capture_data(capture, record)
    if capture.get("raw_samples_validated") is not True:
        raise ValueError("Beta moving noise requires a validated independent raw capture")
    samples, baseline = capture["samples"], capture["baseline_g"]
    segments = record.get("segments")
    if not isinstance(segments, (tuple, list)) or not segments:
        raise ValueError("Beta moving noise requires actual planner segments")
    delta = [tuple(row[index + 1] - baseline[index] for index in range(3)) for row in samples]
    magnitude = [math.sqrt(sum(value * value for value in row)) for row in delta]
    if not all(_finite(value) for value in magnitude):
        raise ValueError("Beta moving magnitudes must be finite")
    seed_indices, phase_indices, phases, previous_end = set(), set(), [], None
    for number, segment in enumerate(segments):
        if not isinstance(segment, dict) or segment.get("phase") not in (
                "startup", "acceleration", "cruise", "braking", "return"):
            raise ValueError("Invalid beta planner phase")
        start, end = segment.get("start_time_s"), segment.get("end_time_s")
        if (not _finite(start) or not _finite(end) or end < start
                or (previous_end is not None and start < previous_end - 1e-9)):
            raise ValueError("Beta planner segment times overlap or are invalid")
        previous_end = end
        indices = [index for index, row in enumerate(samples)
                   if start <= row[0] < end or (number == len(segments) - 1 and row[0] == end)]
        eligible = segment["phase"] in ("startup", "acceleration", "cruise")
        phase_indices.update(indices)
        if eligible:
            seed_indices.update(indices)
        peak = max(indices, key=lambda index: magnitude[index]) if indices else None
        phases.append({**segment, "seed_eligible": eligible, "sample_count": len(indices),
                       "magnitude_peak_g": None if peak is None else magnitude[peak],
                       "peak_time_s": None if peak is None else samples[peak][0],
                       "axis_absolute_peak_g": None if not indices else
                           {axis: max(abs(delta[index][channel]) for index in indices)
                            for channel, axis in enumerate("xyz")}})
    if not seed_indices:
        raise ValueError("Beta moving capture has no startup/acceleration/cruise samples")
    peak = max(seed_indices, key=lambda index: magnitude[index])
    phase_peak = max(phase_indices, key=lambda index: magnitude[index])
    result = {"clock_domain": "sensor_print_time_seconds", "sample_count": len(samples),
              "first_sample_time_s": samples[0][0], "last_sample_time_s": samples[-1][0],
              "seed_phases": ["startup", "acceleration", "cruise"],
              "excluded_seed_phases": ["braking", "return"],
              "seed_sample_count": len(seed_indices), "seed_magnitude_peak_g": magnitude[peak],
              "seed_peak_time_s": samples[peak][0],
              "all_phase_magnitude_peak_g": max(magnitude[index] for index in phase_indices),
              "all_phase_peak_time_s": samples[phase_peak][0],
              "capture_magnitude_peak_g": max(magnitude), "phases": phases}
    _json(result)
    return result


def compact_beta_home(capture, record):
    """Keep a bounded display trace after exact full-row beta event analysis."""
    trace, metadata = _capture_data(capture, record)
    telemetry = record.get("beta_telemetry")
    if not isinstance(telemetry, dict) or telemetry.get("version") != 1:
        raise ValueError("Beta home must retain its separately validated exact event telemetry")
    samples, baseline = capture["samples"], capture["baseline_g"]
    marked_times = {telemetry["first_crossing"]["time_s"]}
    for key in ("pre_onset_noise", "onset_window", "post_stop_response"):
        marked_times.update(telemetry[key].get(field) for field in
                            ("peak_time_s", "start_time_s", "end_time_s"))
    # XYZ extrema alone need not preserve a vector-magnitude extremum. Retain
    # every exact event/peak row in addition to the ordinary display envelope.
    extra = [(row[0] - samples[0][0],) + tuple(row[index + 1] - baseline[index] for index in range(3))
             for row in samples if row[0] in marked_times]
    trace = sorted({tuple(row) for row in tuple(trace) + tuple(extra)})
    onset = telemetry["first_crossing"]["time_s"]
    onset_span = telemetry["onset_window"]["end_time_s"] - onset
    response_end = telemetry["post_stop_response"].get("end_time_s")
    event_end = max(onset + onset_span, response_end if _finite(response_end) else onset + onset_span)
    event = [(row[0] - samples[0][0],) + tuple(row[index + 1] - baseline[index] for index in range(3))
             for row in samples if onset - onset_span <= row[0] <= event_end]
    event_trace = sorted({tuple(row) for row in tuple(_envelope(event, 256)) + tuple(extra)
                          if onset - onset_span <= row[0] + samples[0][0] <= event_end})
    return {"record": json.loads(_json(record)),
            "capture": {**metadata, "trace": trace,
                        "event_trace": event_trace,
                        "plot_data": "extrema_envelope_after_full_sample_beta_analysis",
                        "beta_compact": True}}


def write_beta_motion(directory, run_id, axis, iterations, homes=(), result="complete", reason=None,
                      proposed_threshold_g=None, simulated=False, renderer=None, check_running=None):
    """Publish one Beta-branded suggestion PNG; never produce other formats."""
    _identity(run_id, axis)
    if (not isinstance(iterations, (tuple, list)) or len(iterations) > 3
            or not isinstance(homes, (tuple, list)) or len(homes) > 100
            or result not in ("complete", "beta_proposed", "aborted", "incomplete", "failed", "cancelled")
            or (reason is not None and (not isinstance(reason, str) or len(reason) > 4000))
            or (proposed_threshold_g is not None and
                (not _finite(proposed_threshold_g) or proposed_threshold_g <= 0))):
        raise ValueError("Invalid Beta calibration PNG report")
    motions, home_traces, incomplete_homes = [], [], []
    previous_end, motion_context = None, None
    for item in iterations:
        if not isinstance(item, dict) or not isinstance(item.get("record"), dict):
            raise ValueError("Invalid beta motion item")
        record, capture = item["record"], item["capture"]
        if record.get("axis") != axis or record.get("run_id", run_id) != run_id:
            raise ValueError("Beta motion does not belong to this axis/run")
        statistics = beta_motion_statistics(capture, record)
        context = _json(capture.get("sensor_context", capture.get("_noise_context")))
        if ((previous_end is not None and capture["samples"][0][0] <= previous_end)
                or (motion_context is not None and context != motion_context)):
            raise ValueError("Beta moving captures overlap or changed sensor context")
        previous_end, motion_context = capture["samples"][-1][0], context
        trace, metadata = _capture_data(capture, record)
        marked = {statistics["seed_peak_time_s"], statistics["all_phase_peak_time_s"]}
        marked.update(phase["peak_time_s"] for phase in statistics["phases"])
        rows, baseline = capture["samples"], capture["baseline_g"]
        extra = [(row[0] - rows[0][0],) + tuple(row[index + 1] - baseline[index] for index in range(3))
                 for row in rows if row[0] in marked]
        trace = sorted({tuple(row) for row in tuple(trace) + tuple(extra)})
        motions.append({"record": record, "statistics": statistics,
                        "trace": trace, "capture": metadata})
    for item in homes:
        if not isinstance(item, dict) or not isinstance(item.get("record"), dict):
            raise ValueError("Invalid beta home item")
        record, capture = item["record"], item["capture"]
        if not isinstance(record.get("beta_telemetry"), dict):
            if (result not in ("complete", "beta_proposed")
                    and record.get("beta_decision") == "incomplete" and record.get("capture_valid") is False
                    and record.get("axis") == axis and record.get("run_id", run_id) == run_id):
                incomplete_homes.append({key: record.get(key) for key in
                                         ("workflow_attempt", "result", "beta_decision", "beta_reason")})
                continue
            raise ValueError("Beta home has no valid event telemetry or incomplete-trial marker")
        if (record.get("axis") != axis or record.get("run_id") != run_id
                or not isinstance(record.get("beta_telemetry"), dict)):
            raise ValueError("Beta home does not belong to this axis/run")
        compact = item if capture.get("beta_compact") is True else compact_beta_home(capture, record)
        plot = compact["capture"]
        trace = plot.get("trace")
        if (not isinstance(trace, (tuple, list)) or not 2 <= len(trace) <= 5000
                or any(not isinstance(row, (tuple, list)) or len(row) != 4
                       or not all(_finite(value) for value in row) for row in trace)
                or any(first[0] >= second[0] for first, second in zip(trace, trace[1:]))
                or not _finite(plot.get("first_sample_time_s"))):
            raise ValueError("Invalid beta display envelope")
        home_traces.append(compact)
    clock = _clock()
    title = "Accel Homing Beta %s - Suggested Threshold" % axis.upper()
    payload = {"kind": "beta", "axis": axis, "motions": motions, "homes": home_traces,
               "incomplete_homes": incomplete_homes,
               "proposed_threshold_g": proposed_threshold_g, "result": result, "reason": reason,
               "title": title, "simulated": bool(simulated), **clock}
    data = (renderer or PNGRenderer()).render(payload, check_running=check_running)
    path = _publish(directory, title, clock, data, check_running)
    _check_running(check_running)
    return {"png": path}


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _measured_travel(record):
    # Older records only have the console-rounded field. An invalid exact
    # measurement must be rejected, rather than hidden by that fallback.
    return record.get("travel_mm_exact", record.get("travel_mm"))


def _accuracy_statistics(trials):
    values = [_measured_travel(trial) for trial in trials]
    if not values:
        return None
    if any(not _finite(value) or value <= 0 for value in values):
        raise ValueError("Accuracy trial has invalid commanded travel")
    variance = statistics.pvariance(values) if len(values) >= 2 else None
    result = {"count": len(values), "min_mm": min(values), "max_mm": max(values),
              "mean_mm": statistics.mean(values), "spread_mm": max(values) - min(values),
              "stddev_mm": math.sqrt(variance) if variance is not None else None,
              "variance_mm2": variance,
              "variance_definition": _POPULATION if variance is not None else
              _POPULATION + "; at least 2 measured homes required"}
    if any(not _finite(value) for key, value in result.items()
           if key != "variance_definition" and value is not None):
        raise ValueError("Accuracy statistics are not finite")
    return result

def _check_accuracy_record(record, run_id, axis, phase, index, attempts, measured=True):
    if not isinstance(record, dict) or record.get("run_id") != run_id \
            or record.get("axis") != axis or record.get("workflow_phase") != phase \
            or record.get("accuracy_sample") != index:
        raise ValueError("Accuracy trial identity does not match this run")
    attempt = record.get("workflow_attempt")
    if type(attempt) is not int or attempt <= 0 or attempt in attempts:
        raise ValueError("Accuracy trial attempt is invalid or duplicated")
    attempts.add(attempt)
    if measured:
        travel = _measured_travel(record)
        if record.get("result") != "impact" or record.get("capture_valid") is False \
                or not _finite(record.get("travel_mm")) or record["travel_mm"] <= 0 \
                or not _finite(travel) or travel <= 0 \
                or not _finite(record.get("onset_g")) \
                or not _finite(record.get("impact_peak_g")) or record["impact_peak_g"] < 0 \
                or not _finite(record.get("threshold_g")) or record["threshold_g"] <= 0 \
                or record.get("direction") not in (-1, 0, 1):
            raise ValueError("Accuracy trial has invalid travel or acceleration measurements")

def _validate_accuracy(directory, run_id, axis_results, result, reason, simulated, check_running=None):
    if result not in ("complete", "aborted", "incomplete", "failed", "cancelled"):
        raise ValueError("Invalid accuracy report result")
    if reason is not None and (not isinstance(reason, str) or len(reason) > 4000):
        raise ValueError("Invalid accuracy report reason")
    if not isinstance(axis_results, dict) or not axis_results \
            or any(axis not in ("x", "y") for axis in axis_results):
        raise ValueError("Accuracy report requires X and/or Y results")
    # Validate before opening any output; also reject NaN hidden in metadata.
    if len(json.dumps(axis_results, allow_nan=False).encode("utf-8")) > 2 * 1024 * 1024:
        raise ValueError("Accuracy metadata exceeds its report size limit")
    checked, attempts = {}, set()
    for axis in (key for key in "xy" if key in axis_results):
        _check_running(check_running)
        item = axis_results[axis]
        if not isinstance(item, dict):
            raise ValueError("Invalid accuracy axis result")
        requested, trials = item.get("requested_samples"), item.get("trials")
        if type(requested) is not int or not 2 <= requested <= 50 \
                or not isinstance(trials, list) or len(trials) > requested \
                or type(item.get("complete")) is not bool \
                or item["complete"] != (len(trials) == requested):
            raise ValueError("Accuracy trial count does not match requested samples")
        capture_paths = item.get("capture_paths", [])
        if not isinstance(capture_paths, list) or len(capture_paths) != len(trials):
            raise ValueError("Accuracy capture paths are not aligned with measured trials")
        discovery = item.get("discovery")
        if discovery is not None:
            _check_accuracy_record(discovery, run_id, axis, "discovery", 0, attempts)
        if trials and discovery is None:
            raise ValueError("Measured accuracy homes require a separate qualifying home")
        refs = []
        for index, (record, paths) in enumerate(zip(trials, capture_paths), 1):
            _check_accuracy_record(record, run_id, axis, "measurement", index, attempts)
            if discovery is not None and (not math.isclose(record["threshold_g"], discovery["threshold_g"],
                                                           rel_tol=1e-12, abs_tol=1e-12)
                    or record.get("workflow_threshold_g") != discovery.get("workflow_threshold_g")):
                raise ValueError("Accuracy threshold changed during the measured run")
            refs.append(_capture_reference(directory, paths, run_id, axis, record, simulated, check_running))
        discovery_ref = _capture_reference(directory, item.get("discovery_capture_paths"),
                                           run_id, axis, discovery, simulated, check_running) \
            if discovery is not None else None
        rejected = item.get("rejected")
        rejection_ref = None
        if rejected is not None:
            if not isinstance(rejected, dict) or rejected.get("run_id") != run_id \
                    or rejected.get("axis") != axis \
                    or type(rejected.get("workflow_attempt")) is not int \
                    or rejected["workflow_attempt"] <= 0 \
                    or rejected["workflow_attempt"] in attempts:
                raise ValueError("Rejected accuracy trial identity is invalid")
            attempts.add(rejected["workflow_attempt"])
            rejection_ref = _capture_reference(directory, item.get("rejection_capture_paths"),
                                               run_id, axis, rejected, simulated, check_running)
        computed = _accuracy_statistics(trials)
        supplied = item.get("statistics")
        if supplied is not None:
            if computed is None or not isinstance(supplied, dict):
                raise ValueError("Accuracy statistics do not match measured trials")
            for key, value in computed.items():
                other = supplied.get(key)
                if value is None or isinstance(value, str):
                    equal = other == value
                else:
                    equal = _finite(other) and math.isclose(value, other, rel_tol=1e-9, abs_tol=1e-12)
                if not equal:
                    raise ValueError("Accuracy statistics do not match measured trials")
        if result == "complete" and (not item["complete"] or discovery_ref is None
                                      or any(ref is None for ref in refs) or rejected is not None):
            raise ValueError("A complete accuracy report requires every requested valid capture")
        checked[axis] = {**item, "statistics": computed, "capture_paths": refs,
                         "discovery_capture_paths": discovery_ref,
                         "rejection_capture_paths": rejection_ref}
    return checked

def _noise_identity(run_id, iteration):
    if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
        raise ValueError("Invalid noise run ID")
    if type(iteration) is not int or not 1 <= iteration <= _NOISE_WINDOWS:
        raise ValueError("Invalid stationary noise iteration")
    return "accel_homing_noise_%s_i%02d" % (run_id, iteration)

def _noise_vector(value, label):
    if not isinstance(value, dict) or set(value) != set("xyz") \
            or not all(_finite(number) for number in value.values()):
        raise ValueError("Invalid stationary noise %s" % label)
    return value

def _same_noise_number(first, second):
    return _finite(first) and _finite(second) and math.isclose(
        first, second, rel_tol=1e-9, abs_tol=1e-12)

def _check_noise_record(run_id, iteration, record):
    _noise_identity(run_id, iteration)
    if not isinstance(record, dict) or record.get("run_id") != run_id \
            or type(record.get("noise_iteration")) is not int or record["noise_iteration"] != iteration \
            or type(record.get("sample_count")) is not int or record["sample_count"] < 50 \
            or not _finite(record.get("duration_s")) \
            or record["duration_s"] < _NOISE_WINDOW_SECONDS - 1e-9 \
            or not _finite(record.get("first_sample_time_s")) \
            or not _finite(record.get("last_sample_time_s")) \
            or not _same_noise_number(record["duration_s"],
                                      record["last_sample_time_s"] - record["first_sample_time_s"]):
        raise ValueError("Stationary noise record is not a complete current-run window")
    for key in ("baseline_g", "noise_peak_g", "noise_peak_to_peak_g", "noise_rms_g"):
        vector = _noise_vector(record.get(key), key)
        if key != "baseline_g" and any(value < 0 for value in vector.values()):
            raise ValueError("Stationary noise magnitudes must not be negative")
    if "noise_magnitude_peak_g" in record and (
            not _finite(record["noise_magnitude_peak_g"]) or record["noise_magnitude_peak_g"] < 0):
        raise ValueError("Invalid stationary XYZ-magnitude noise peak")
    for key in ("sensor", "backend", "axis_context"):
        if not isinstance(record.get(key), str) or not record[key] or len(record[key]) > 400:
            raise ValueError("Invalid stationary noise sensor context")
    full_scale = record.get("full_scale_g")
    if full_scale is not None and (not _finite(full_scale) or full_scale <= 0):
        raise ValueError("Invalid stationary noise sensor range")
    json.dumps(record, allow_nan=False)

def _validate_noise_capture(run_id, iteration, capture, record):
    _check_noise_record(run_id, iteration, record)
    if not isinstance(capture, dict) or capture.get("dropped", 0) \
            or capture.get("fault") or capture.get("error"):
        raise ValueError("Stationary noise sensor capture is incomplete or faulted")
    samples = capture.get("samples", ())
    if not isinstance(samples, (list, tuple)) or len(samples) != record["sample_count"]:
        raise ValueError("Stationary noise capture count does not match its record")
    previous = None
    for sample in samples:
        if not isinstance(sample, (list, tuple)) or len(sample) != 4 \
                or not all(_finite(value) for value in sample) \
                or (previous is not None and sample[0] <= previous):
            raise ValueError("Invalid stationary noise accelerometer samples")
        previous = sample[0]
    if not _same_noise_number(samples[0][0], record["first_sample_time_s"]) \
            or not _same_noise_number(samples[-1][0], record["last_sample_time_s"]):
        raise ValueError("Stationary noise sample times do not match their record")
    baseline = tuple(math.fsum(sample[index] for sample in samples) / len(samples)
                     for index in (1, 2, 3))
    supplied = capture.get("baseline_g")
    if not isinstance(supplied, (list, tuple)) or len(supplied) != 3 \
            or not all(_same_noise_number(value, supplied[index])
                       for index, value in enumerate(baseline)):
        raise ValueError("Stationary noise capture must use its full-window mean")
    delta = [(sample[0],) + tuple(sample[index + 1] - baseline[index] for index in range(3))
             for sample in samples]
    for index, axis in enumerate("xyz", 1):
        values = [sample[index] for sample in delta]
        computed = {"baseline_g": baseline[index - 1],
                    "noise_peak_g": max(abs(value) for value in values),
                    "noise_peak_to_peak_g": max(values) - min(values),
                    "noise_rms_g": math.sqrt(math.fsum(value * value for value in values) / len(values))}
        if any(not _same_noise_number(record[key][axis], value) for key, value in computed.items()):
            raise ValueError("Stationary noise statistics do not match measured samples")
    if "noise_magnitude_peak_g" in record:
        magnitude_peak = max(math.sqrt(math.fsum(value * value for value in sample[1:]))
                             for sample in delta)
        if not _same_noise_number(record["noise_magnitude_peak_g"], magnitude_peak):
            raise ValueError("Stationary XYZ-magnitude noise peak does not match measured samples")
    return samples, baseline, delta

def _validate_noise_recommendation(recommendation, iterations):
    if not isinstance(recommendation, dict) or recommendation.get("source") != "stationary_noise" \
            or recommendation.get("method") != "initial_candidate_heuristic" \
            or recommendation.get("margin_factor") != 1.2 or recommendation.get("margin_g") != 0.1 \
            or recommendation.get("clamped") is not False:
        raise ValueError("Invalid stationary noise initial-candidate heuristic")
    for key in ("candidate_step_g", "configured_start_g"):
        if not _finite(recommendation.get(key)) or recommendation[key] <= 0:
            raise ValueError("Invalid stationary noise candidate bounds")
    limit = recommendation.get("threshold_limit_g")
    if limit is not None and (not _finite(limit) or limit <= 0):
        raise ValueError("Invalid stationary noise candidate ceiling")
    observed = _noise_vector(recommendation.get("noise_peak_g"), "maximum peak")
    candidates = recommendation.get("start_threshold_g")
    if not isinstance(candidates, dict) or set(candidates) != set("xy"):
        raise ValueError("Invalid stationary noise axis candidates")
    for axis in "xyz":
        if not _same_noise_number(observed[axis], max(item["record"]["noise_peak_g"][axis]
                                                     for item in iterations)):
            raise ValueError("Stationary noise recommendation peak does not match its windows")
    if limit is None:
        if any(item["record"]["full_scale_g"] is not None for item in iterations) \
                or candidates != {"x": None, "y": None} \
                or recommendation.get("unavailable_axes") != ["x", "y"]:
            raise ValueError("Unknown sensor range cannot supply a stationary noise calibration candidate")
        json.dumps(recommendation, allow_nan=False)
        return
    unavailable = []
    start, step = recommendation["configured_start_g"], recommendation["candidate_step_g"]
    for axis in "xy":
        minimum = max(start, observed[axis] * 1.2, observed[axis] + 0.1)
        candidate = start + max(0, math.ceil((minimum - start) / step)) * step
        if not _finite(candidate) or candidate > limit:
            unavailable.append(axis)
            if candidates[axis] is not None:
                raise ValueError("Stationary noise candidate exceeds its ceiling; do not clamp it downward")
        elif not _same_noise_number(candidates[axis], candidate):
            raise ValueError("Stationary noise candidate is not rounded upward on the calibration grid")
    if recommendation.get("unavailable_axes") != unavailable:
        raise ValueError("Stationary noise unavailable candidates do not match the ceiling")
    json.dumps(recommendation, allow_nan=False)


def _worker_main(source, target, dependency_directory):
    # This function runs only as a separate selected-Python process. Plotting
    # packages and their font/config cache are never loaded into Klippy.
    private = Path(dependency_directory)
    if private.is_dir():
        sys.path.insert(0, str(private))
    os.environ["MPLCONFIGDIR"] = str(Path(source).parent / "matplotlib-cache")
    os.environ["MPLBACKEND"] = "Agg"
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib import rcParams
    rcParams["text.usetex"] = False
    if "text.parse_math" in rcParams:
        rcParams["text.parse_math"] = False
    payload = json.loads(Path(source).read_text(encoding="utf-8"))
    kind = payload["kind"]
    figure = Figure(figsize=(10, 6) if kind == "capture" else (10, 10) if kind == "accuracy" else (10, 8), dpi=120)
    try:
        canvas = FigureCanvasAgg(figure)
        if kind == "preflight":
            figure.set_size_inches(2, 1)
            figure.subplots().plot([0, 1], [0, 1])
        elif kind == "capture":
            _plot_capture(figure, payload)
        elif kind == "accuracy":
            _plot_accuracy(figure, payload)
        elif kind == "noise":
            _plot_noise(figure, payload)
        elif kind == "beta":
            _plot_beta(figure, payload)
        else:
            raise ValueError("Unknown PNG chart render kind")
        canvas.print_png(target, metadata={
            "Software": "Beacon Accelerometer Homing",
            "Creation Time": payload.get("created_utc", ""),
            "Local Time": payload.get("created_local", ""),
            "Description": "SIMULATED - NOT PRINTER MEASUREMENT" if payload.get("simulated")
                           else "Captured accelerometer measurements"})
        _check_png(Path(target).read_bytes())
    finally:
        figure.clear()


def _title(figure, payload, title):
    status = payload.get("result", "complete")
    if status != "complete":
        title = "%s - %s" % (title, status.upper())
    if payload.get("simulated"):
        title = "SIMULATED / NOT PRINTER MEASUREMENT\n" + title
    figure.suptitle("%s\n%s" % (title, payload["timestamp"]), fontsize=12)


def _number(value, significant=3):
    return ("%.*g" % (significant, value)) if _finite(value) else "n/a"


def _axes_summary(axis, lines):
    axis.set_title("\n".join(lines), loc="left", fontsize=8, pad=6)


def _finish_layout(figure, footer, reason=None):
    lines = textwrap.wrap(footer, 100)
    if reason:
        reason_lines = textwrap.wrap(" ".join(str(reason).split()), 100)
        lines.extend(reason_lines[:2])
        if len(reason_lines) > 2:
            lines[-1] = lines[-1].rstrip(".") + "..."
    figure.text(.5, .012, "\n".join(lines), ha="center", va="bottom", fontsize=8, color=".3")
    bottom = .025 + len(lines) * 11.0 / 72.0 / figure.get_figheight()
    figure.tight_layout(pad=1.1, h_pad=1.4, rect=(0, bottom, 1, 1))


def _axes_style(axis, ylabel="Acceleration (g)", zero=True):
    axis.minorticks_on()
    axis.grid(which="major", color="grey", linewidth=.6, alpha=.4)
    axis.grid(which="minor", color="lightgrey", linewidth=.4, alpha=.4)
    axis.tick_params(labelsize=9)
    axis.set_ylabel(ylabel)
    axis.set_xlabel("Time (s)")
    if zero:
        axis.axhline(0, color="grey", linewidth=.5)
    axis.margins(x=.02, y=.12)


def _traces(axis, trace, alpha=1.0, labels=True):
    for index, (letter, color) in enumerate(zip("XYZ", ("red", "green", "blue")), 1):
        axis.plot([row[0] for row in trace], [row[index] for row in trace], color=color,
                  linewidth=.75, alpha=alpha, label=letter if labels else None)


def _threshold(axis, record, label=True):
    threshold = record.get("threshold_g")
    if not _finite(threshold) or threshold <= 0:
        return
    direction = record.get("direction", 0)
    vector = record.get("impact_metric") == "xyz_magnitude"
    limits = ([threshold] if vector else
              [direction * threshold] if direction in (-1, 1) else [-threshold, threshold])
    name = ("XYZ magnitude trigger %s g" % _number(threshold, 4) if vector else
            "Trigger %+.4g g" % limits[0] if len(limits) == 1 else
            "Trigger ±%s g" % _number(threshold, 4))
    for index, value in enumerate(limits):
        axis.axhline(value, color="#744b98", linestyle="--", linewidth=1.1,
                     label=name if label and index == 0 else None)


def _plot_capture(figure, payload):
    record, trace, captured = payload["record"], payload["trace"], payload["capture"]
    title = payload.get("title", "Accel Homing " + payload["axis"].upper())
    _title(figure, payload, title)
    axis = figure.subplots()
    _axes_style(axis)
    _traces(axis, trace)
    vector = record.get("impact_metric") == "xyz_magnitude"
    magnitude = [(row[0], math.sqrt(sum(value * value for value in row[1:4]))) for row in trace]
    if vector:
        axis.plot([row[0] for row in magnitude], [row[1] for row in magnitude],
                  linewidth=1.2, label="XYZ magnitude")
    _threshold(axis, record)
    onset = record.get("onset_time_s", record.get("onset_time"))
    elapsed = None
    if _finite(onset):
        elapsed = onset - captured["first_sample_time_s"]
        if trace[0][0] <= elapsed <= trace[-1][0]:
            axis.axvline(elapsed, color="#5e4c77", linestyle=":", linewidth=1.1, label="Detected onset")
        else:
            elapsed = None
    channel = {"x": 1, "y": 2}[payload["axis"]]
    peak = (max(magnitude, key=lambda row: row[1]) if vector else
            max(trace, key=lambda row: abs(row[channel])))
    peak_value = peak[1] if vector else peak[channel]
    peak_name = "XYZ magnitude" if vector else "|%s|" % payload["axis"].upper()
    axis.plot([peak[0]], [peak_value], "o", color="#192d45", markersize=4)
    _axes_summary(axis, ["Samples: %d  |  Travel: %s mm  |  Onset: %s s / %s g" % (
                            captured["sample_count"], _number(_measured_travel(record), 6),
                            _number(elapsed, 4), _number(record.get("onset_g"))),
                         "Detector peak: %s g  |  Captured %s peak: %s g" % (
                            _number(record.get("impact_peak_g")), peak_name, _number(abs(peak_value)))])
    axis.legend(loc="best", fontsize=8)
    _finish_layout(figure, "Stationary baseline removed. Commanded travel is not physical stop accuracy.")


def _plot_beta(figure, payload):
    """Beta-only combined diagnostic; ordinary chart styles remain untouched."""
    figure.set_size_inches(12, 17)
    suggestion = _number(payload.get("proposed_threshold_g"), 6)
    _title(figure, payload, "%s: %s g" % (payload["title"], suggestion))
    axes = figure.subplots(7, 1)
    colors = {"startup": "#b5c8e3", "acceleration": "#f0cb87", "cruise": "#9bcbb8",
              "braking": "#d7b2ca", "return": "#c5c5c5"}
    for number, axis in enumerate(axes[:3]):
        _axes_style(axis)
        if number >= len(payload["motions"]):
            axis.text(.5, .5, "No valid beta motion capture", ha="center", transform=axis.transAxes)
            _axes_summary(axis, ["Central motion %d" % (number + 1)])
            continue
        item = payload["motions"][number]
        record, stats, trace, captured = item["record"], item["statistics"], item["trace"], item["capture"]
        _traces(axis, trace)
        axis.plot([row[0] for row in trace],
                  [math.sqrt(sum(value * value for value in row[1:])) for row in trace],
                  color="#202832", linewidth=.8, alpha=.8, label="XYZ magnitude")
        labels = set()
        for segment in stats["phases"]:
            phase = segment["phase"]
            first = segment["start_time_s"] - captured["first_sample_time_s"]
            last = segment["end_time_s"] - captured["first_sample_time_s"]
            label = phase.capitalize() if phase not in labels else None
            axis.axvspan(first, last, color=colors[phase], alpha=.25, label=label)
            labels.add(phase)
        axis.plot([stats["seed_peak_time_s"] - captured["first_sample_time_s"]],
                  [stats["seed_magnitude_peak_g"]], "o", color="#172b4d", markersize=4,
                  label="Seed-eligible peak")
        configured = record.get("configured_speed_mm_s", record.get("configured_homing_speed_mm_s"))
        effective = record.get("effective_speed_mm_s", record.get("homing_speed_mm_s"))
        _axes_summary(axis, ["Central motion %d | Configured speed: %s mm/s | Planner speed: %s mm/s" % (
                                number + 1, _number(configured, 6), _number(effective, 6)),
                             "Seed-eligible peak: %s g | Braking/return excluded from seed | %d full samples" % (
                                _number(stats["seed_magnitude_peak_g"], 6), stats["sample_count"])])
        axis.legend(loc="upper right", ncol=4, fontsize=7)
    events = axes[3]
    _axes_style(events, "XYZ magnitude (g)")
    events.set_xlabel("Beta home, including rejected candidates")
    homes = payload["homes"]
    if homes:
        indices = list(range(1, len(homes) + 1))
        for key, label, color in (("pre_onset_noise", "Noise strictly before crossing", "#577d34"),
                                  ("onset_window", "Peak in exact onset window", "#b66b24"),
                                  ("post_stop_response", "Response after observed queue completion", "#9a5083")):
            values = [item["record"]["beta_telemetry"][key].get("magnitude_peak_g") for item in homes]
            events.plot(indices, values, "o-", color=color, linewidth=.8, markersize=3, label=label)
        events.plot(indices, [item["record"]["beta_telemetry"]["first_crossing"]["magnitude_g"]
                              for item in homes], "o-", color="#2462a6", linewidth=.8,
                    markersize=3, label="Exact first crossing")
        events.plot(indices, [item["record"]["beta_telemetry"]["first_crossing"]["threshold_g"]
                              for item in homes], "--", color="#333333", linewidth=.8,
                    label="Temporary candidate threshold")
        events.legend(loc="best", fontsize=7, ncol=2)
    else:
        events.text(.5, .5, "No valid beta home event capture", ha="center", transform=events.transAxes)
    _axes_summary(events, ["Suggested threshold: %s g | Proposal only | %d captured beta homes" % (
                            suggestion, len(homes)),
                          "Event statistics are computed per sample before any display reduction."] +
                         ["Incomplete attempt %s: %s" % (
                             _number(item.get("workflow_attempt")), str(item.get("beta_reason") or item.get("result"))[:160])
                          for item in payload.get("incomplete_homes", [])[:2]])
    for channel, (axis, letter, color) in enumerate(zip(axes[4:], "XYZ", ("red", "green", "blue")), 1):
        _axes_style(axis, "%s acceleration (g)" % letter)
        axis.set_xlabel("Time from each beta home's exact first crossing (s)")
        for number, item in enumerate(homes):
            record, captured = item["record"], item["capture"]
            onset = record["beta_telemetry"]["first_crossing"]["time_s"]
            trace = captured.get("event_trace") or captured["trace"]
            last = number == len(homes) - 1
            axis.plot([row[0] + captured["first_sample_time_s"] - onset for row in trace],
                      [row[channel] for row in trace], color=color,
                      linewidth=1.0 if last else .6, alpha=.95 if last else .23,
                      label="Last beta home" if last else None)
        axis.axvline(0, color="#2462a6", linestyle=":", linewidth=.8, label="Exact first crossing")
        if homes:
            telemetry = homes[-1]["record"]["beta_telemetry"]
            onset = telemetry["first_crossing"]["time_s"]
            axis.axvline(telemetry["pre_onset_noise"]["start_time_s"] - onset,
                         color="#65747e", linestyle=":", linewidth=.7, label="Last detector arm")
            axis.axvspan(0, telemetry["onset_window"]["end_time_s"] - onset,
                         color="#f0cb87", alpha=.15, label="Onset window")
            response = telemetry["post_stop_response"]
            if _finite(response.get("boundary_time_s")):
                boundary = response["boundary_time_s"] - onset
                axis.axvline(boundary, color="#9a5083", linestyle="--", linewidth=.7,
                             label="Last host-observed queue completion")
                if _finite(response.get("end_time_s")):
                    axis.axvspan(boundary, response["end_time_s"] - onset,
                                 color="#d7b2ca", alpha=.12)
            _axes_summary(axis, ["Beta home XYZ event traces | Last home emphasized | Resting offset removed"])
        else:
            axis.text(.5, .5, "No valid beta home trace", ha="center", transform=axis.transAxes)
        axis.legend(loc="best", fontsize=7)
    _finish_layout(figure,
        "Beta suggestion only; no model staged or saved. Runtime-restoration outcome is in the command diagnostics. "
        "Exact event metrics use full samples; plotted curves are extrema envelopes. "
        "Queue completion is host observed, not a measurement of mechanical stopping.", payload.get("reason"))


def _plot_accuracy(figure, payload):
    stats = payload.get("statistics") or {}
    trials, traces = payload["trials"], payload["traces"]
    summary = ["Accepted homes: %d / %d" % (len(trials), payload["requested"])]
    if stats:
        summary[0] += "  |  Mean: %s mm  |  Range: %s mm" % (_number(stats["mean_mm"], 6), _number(stats["spread_mm"]))
        summary += ["Min: %s mm  |  Max: %s mm  |  Std dev: %s mm  |  Variance: %s mm²" % (
                    _number(stats["min_mm"], 6), _number(stats["max_mm"], 6),
                    _number(stats["stddev_mm"]), _number(stats.get("variance_mm2")))]
    _title(figure, payload, "Accel Homing Accuracy " + payload["axis"].upper())
    vector = bool(trials and trials[0].get("impact_metric") == "xyz_magnitude")
    axes = figure.subplots(5 if vector else 4, 1)
    travel = axes[0]
    _axes_style(travel, "Travel (mm)", zero=False)
    travel.set_xlabel("Accepted measured home")
    travel.ticklabel_format(axis="y", style="plain", useOffset=False)
    if trials:
        travel.plot(range(1, len(trials) + 1), [_measured_travel(row) for row in trials], "o-",
                    color="#1769aa", markersize=4)
        travel.set_xlim(.5, max(1.5, len(trials) + .5))
    else:
        travel.text(.5, .5, "No accepted measured homes", ha="center", transform=travel.transAxes)
    _axes_summary(travel, summary)
    for channel, (axis, color, letter) in enumerate(zip(axes[1:],
                    ("red", "green", "blue"), "XYZ"), 1):
        _axes_style(axis, "%s acceleration (g)" % letter)
        for index, trace in enumerate(traces):
            axis.plot([row[0] for row in trace], [row[channel] for row in trace], color=color,
                      alpha=.35 if len(traces) > 1 else 1., linewidth=.7,
                      label="%s (%d homes)" % (letter, len(traces)) if index == 0 else None)
        if not vector and channel == {"x": 1, "y": 2}[payload["axis"]] and trials:
            _threshold(axis, trials[0])
        if traces:
            axis.legend(fontsize=8, loc="best")
    if vector:
        axis = axes[4]
        _axes_style(axis, "XYZ magnitude (g)")
        for index, trace in enumerate(traces):
            axis.plot([row[0] for row in trace],
                      [math.sqrt(sum(value * value for value in row[1:4])) for row in trace],
                      linewidth=.7, alpha=.35 if len(traces) > 1 else 1.,
                      label="XYZ magnitude" if index == 0 else None)
        _threshold(axis, trials[0])
        axis.legend(fontsize=8, loc="best")
    _finish_layout(figure, "Commanded travel only; not physical stop accuracy. Qualifier excluded. Population statistics.", payload.get("reason"))


def _plot_noise(figure, payload):
    recommendation, records, traces = payload["recommendation"], payload["records"], payload["traces"]
    note = ""
    if recommendation:
        candidates = recommendation["start_threshold_g"]
        fmt = lambda value: "unavailable" if value is None else "%s g" % _number(value, 4)
        note = "Per-axis diagnostic hints: X %s; Y %s (not XYZ-magnitude calibration starts)." % (
            fmt(candidates["x"]), fmt(candidates["y"]),
            )
    else:
        note = "Incomplete measurement: no starting candidate."
    _title(figure, payload, "Accel Homing Noise")
    axes = figure.subplots(3, 1, sharex=True)
    peak = max(.001, max((abs(value) for trace in traces for row in trace for value in row[1:]), default=0.)) * 1.1
    for index, axis in enumerate(axes):
        _axes_style(axis)
        axis.set_ylim(-peak, peak)
        if index >= len(traces):
            axis.set_title("Window %d - no valid data" % (index + 1), fontsize=10)
            continue
        record = records[index]
        _traces(axis, traces[index])
        summary = ["Window %d (%s s)  |  Peak XYZ: %s g" % (
                                index + 1, _number(record["duration_s"]),
                                " / ".join(_number(record["noise_peak_g"][key]) for key in "xyz")),
                   "RMS XYZ: %s g" % " / ".join(_number(record["noise_rms_g"][key]) for key in "xyz")]
        if "noise_magnitude_peak_g" in record:
            summary.append("XYZ-magnitude peak: %s g (calibration metric)" %
                           _number(record["noise_magnitude_peak_g"], 4))
        _axes_summary(axis, summary)
        axis.legend(loc="best", ncol=3, fontsize=8)
    for axis in axes[:-1]:
        axis.set_xlabel("")
    _finish_layout(figure, "Window means removed. " + note, payload.get("reason"))


if __name__ == "__main__":
    if len(sys.argv) != 5 or sys.argv[1] != "--render-worker":
        raise SystemExit("This module is a private PNG rendering worker")
    _worker_main(sys.argv[2], sys.argv[3], sys.argv[4])
