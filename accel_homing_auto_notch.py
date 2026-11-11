# This file may be distributed under the terms of the GNU GPLv3 license.
"""Bounded offline notch proposal; no motion, configuration writes, or SciPy.

Central strokes supply independently commanded free-travel labels. Qualified
raw homes supply *reference stop events*, not independently observed contacts.
Their causal replay ends at the raw trigger, inclusive: later samples already
depend on the original stop command and cannot prove a counterfactual crossing.
All settings are frozen on training data before held-out acceptance. Fresh
whole-home qualification at the exact returned threshold remains mandatory.
"""
import hashlib
import math

try:
    from .accel_homing_filter import CausalNotch, DISABLED, RATE_TOLERANCE, profile
except ImportError:
    from accel_homing_filter import CausalNotch, DISABLED, RATE_TOLERANCE, profile


VERSION = "independent_motion_reference_stop_notch_v1"
MIN_USABLE_CRUISE_SECONDS = 0.25
CRUISE_EDGE_SECONDS = 0.020
MIN_CRUISE_SECONDS = MIN_USABLE_CRUISE_SECONDS + 2 * CRUISE_EDGE_SECONDS
Q_GRID = (2.0, 5.0, 10.0, 20.0)
MAX_PEAKS = 3
MAX_CANDIDATES = MAX_PEAKS * len(Q_GRID)
MAX_CAPTURE_SAMPLES = 200000
MAX_TOTAL_SAMPLES = 600000
MAX_TRACE_POINTS = 1024
MAX_WELCH_SEGMENT = 2048
MAX_REFERENCE_AGE_SECONDS = 0.25
TRAIN_REDUCTION = 0.20
HOLDOUT_REDUCTION = 0.15
REFERENCE_RETENTION = 0.85
MIN_FREQUENCY_HZ = 30.0
MAX_FREQUENCY_HZ = 1000.0
MIN_PEAK_PROMINENCE = 8.0
MIN_PEAK_POWER_FRACTION = 0.10
MIN_CYCLES = 8.0
MIN_BANDWIDTH_BINS = 2.0


class InsufficientData(ValueError):
    """Invalid or insufficient evidence, distinct from a valid disabled choice."""


def preflight():
    """Call before collection so a missing offline dependency cannot start motion."""
    import numpy
    return {"numpy_version": numpy.__version__, "analysis_policy": VERSION,
            "minimum_cruise_seconds": MIN_CRUISE_SECONDS}


def _check(check_running):
    if check_running is not None:
        check_running()  # Cancellation/context exceptions deliberately propagate.


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _require(condition, reason):
    if not condition:
        raise InsufficientData(reason)


def _policy():
    return {"version": VERSION,
            "split": {"motion_train": 2, "motion_holdout": 1,
                      "home_train": 6, "home_holdout": 4},
            "caps": {"captures": 13, "samples_per_capture": MAX_CAPTURE_SAMPLES,
                     "total_samples": MAX_TOTAL_SAMPLES, "spectral_peaks": MAX_PEAKS,
                     "candidate_profiles": MAX_CANDIDATES,
                     "welch_segment_samples": MAX_WELCH_SEGMENT,
                     "trace_points_per_capture": MAX_TRACE_POINTS},
            "q_grid": list(Q_GRID), "cruise_edge_exclusion_s": CRUISE_EDGE_SECONDS,
            "minimum_usable_cruise_s": MIN_USABLE_CRUISE_SECONDS,
            "minimum_frequency_cycles": MIN_CYCLES,
            "minimum_bandwidth_rule": "digital_minus_3dB_bandwidth >= max(2 / usable_cruise_duration, 2 * actual_Welch_bin_width)",
            "minimum_bandwidth_bins": MIN_BANDWIDTH_BINS,
            "spectral_metric": "Welch sum of signed baseline-subtracted XYZ power densities",
            "spectral_prominence_minimum": MIN_PEAK_PROMINENCE,
            "spectral_peak_power_fraction_minimum": MIN_PEAK_POWER_FRACTION,
            "noise_metric": "maximum XYZ magnitude, causal zero-state replay",
            "train_cruise_peak_reduction_minimum": TRAIN_REDUCTION,
            "heldout_cruise_peak_reduction_minimum": HOLDOUT_REDUCTION,
            "startup_acceleration_peak_increase_allowed": 0.0,
            "reference_stop_level_retention_minimum": REFERENCE_RETENTION,
            "reference_lead_rule": "at most two source sample periods and 0.25 mm commanded travel",
            "positive_reference_delay_allowed_s": 0.0,
            "threshold_rule": "max(minimum, 1.2 * training_noise_peak, training_noise_peak + 0.1 g)",
            "selection": "best worst-training cruise reduction, then reference retention; freeze before holdout",
            "holdout_failure": "disabled only if its preregistered raw threshold passes every independent stroke and reference; otherwise rejected",
            "disabled_control": "unchanged raw qualified threshold, no free-travel crossing, exact original reference crossings",
            "motion_labels": "actual commanded trapq segments; no physical-position proof",
            "home_labels": "qualified raw trigger reference_stop_events; not independent contact labels",
            "home_replay_end": "original raw trigger sample inclusive; post-trigger tail never scored",
            "requires_fresh_full_homing_validation": True,
            "rationale": {
                "welch": "overlapping Hann segments reduce single-periodogram variance; only recurring narrow peaks propose filters",
                "bounds": "three recurring peaks and four Q values bound replay cost without searching held-out data",
                "cruise": "trim acceleration/braking boundaries; require duration, cycles and resolvable digital bandwidth",
                "noise": "peak criteria protect the first-crossing detector; an RMS improvement alone is insufficient",
                "retention": "retain at least 85 percent of each raw reference-stop level, with no delayed counterfactual crossing",
                "threshold": "20 percent or 0.1 g measured-noise margin, including causal startup, then fresh exact-threshold homes"}}


def _reference_rate(meta, first, *, motion):
    if motion:
        rate = meta.get("sample_rate_hz")
        count = meta.get("baseline_sample_count")
        start, end = meta.get("baseline_first_sample_time_s"), meta.get("baseline_last_sample_time_s")
    else:
        _require(meta.get("sample_rate_source") == "fresh_reference_timestamps",
                 "home sample rate lacks fresh-reference provenance")
        _require(meta.get("baseline_source") == "armed_detector_reference",
                 "home baseline is not the exact armed detector reference")
        rate = meta.get("sample_rate_hz")
        count = meta.get("reference_sample_count")
        start, end = meta.get("reference_first_time_s"), meta.get("reference_last_time_s")
    _require(_finite(rate) and rate > 0 and type(count) is int and count >= 32
             and _finite(start) and _finite(end) and start < end,
             "missing or invalid measured reference sample rate")
    measured = (count - 1) / (end - start)
    _require(abs(measured / rate - 1) <= RATE_TOLERANCE,
             "reference sample-rate provenance mismatch")
    _require(-1.02 / rate <= first - end <= MAX_REFERENCE_AGE_SECONDS,
             "reference is stale or later than the replay arm")
    return float(rate)


def _home_eligibility(meta, first_time, arm_time, rate):
    """Keep native source-clock eligibility separate from later clock mapping."""
    source = meta.get("eligibility_source")
    if source == "full_precision_source_time":
        _require(-1e-9 <= first_time - arm_time <= 1.02 / rate,
                 "first armed source time does not match generic eligibility")
        return
    _require(source == "native_sensor_clock", "missing exact arm eligibility time domain")
    identity, arm = meta.get("first_sample"), meta.get("arm_clock")
    _require(isinstance(identity, (list, tuple)) and len(identity) == 4
             and all(type(value) is int for value in identity)
             and type(arm) is int and 0 <= arm < 0x100000000,
             "native arm requires exact source packet/sample identity")
    start, delta, index, count = identity
    _require(0 <= start < 0x100000000 and 0 < delta < 0x100000000
             and count >= 2 and 0 <= index < count, "invalid native source sample identity")
    relative = (start - arm) & 0xffffffff
    if relative >= 0x80000000:
        relative -= 0x100000000
    # Integer rational arithmetic avoids subtracting rounded 32-bit clocks.
    elapsed_numerator = relative * (count - 1) + index * delta
    _require(0 <= elapsed_numerator < delta,
             "native first source sample is not the first eligible arm sample")


def _prepare(item, kind, index, raw_threshold, np, check_running):
    _check(check_running)
    _require(isinstance(item, dict) and isinstance(item.get("record"), dict)
             and isinstance(item.get("capture"), dict), "capture entry requires record and capture")
    record, capture = item["record"], item["capture"]
    role = "train" if index < (2 if kind == "motion" else 6) else "holdout"
    number = record.get("iteration" if kind == "motion" else "workflow_attempt")
    run_id, axis = record.get("run_id"), record.get("axis")
    _require(isinstance(run_id, str) and 0 < len(run_id) <= 128 and axis in ("x", "y")
             and type(number) is int and number > 0, "capture lacks run/axis/attempt identity")
    capture_id = "%s:%s:%s:%d" % (run_id, axis, kind, number)
    _require(record.get("physical_position_verified") is False,
             "commanded/plausibility labels must explicitly disclaim physical-position verification")
    _require(not any(capture.get(key) for key in ("dropped", "fault", "error", "amplitude_limited",
                                                 "excess_amplitude_unknown", "clipped_axis_values",
                                                 "range_limit_axis_values", "saturation_axis_values")),
             capture_id + ": incomplete, faulty or amplitude-limited raw capture")
    quality = capture.get("reference_signal_quality") or {}
    _require(isinstance(quality, dict) and not any(quality.get(key) for key in (
        "amplitude_limited", "excess_amplitude_unknown", "clipped_axis_values", "range_limit_axis_values")),
        capture_id + ": amplitude-limited baseline")
    decoding = capture.get("native_clip_decode") or {}
    _require(isinstance(decoding, dict) and not decoding.get("marker_axis_values"),
             capture_id + ": native marker amplitudes cannot train a filter")
    rows, baseline = capture.get("samples"), capture.get("baseline_g")
    _require(isinstance(rows, (list, tuple)) and 128 <= len(rows) <= MAX_CAPTURE_SAMPLES,
             capture_id + ": missing, short or oversized complete raw XYZ capture")
    _require(isinstance(baseline, (tuple, list)) and len(baseline) == 3
             and all(_finite(value) for value in baseline), capture_id + ": missing exact signed XYZ baseline")
    for offset in range(0, len(rows), 2048):
        _check(check_running)
        _require(all(isinstance(row, (list, tuple)) and len(row) == 4
                     and all(_finite(value) for value in row) for row in rows[offset:offset + 2048]),
                 capture_id + ": malformed or nonfinite raw XYZ samples")
    array = np.asarray(rows, dtype=float)
    with np.errstate(over="ignore", invalid="ignore"):
        times, signed = array[:, 0], array[:, 1:] - np.asarray(baseline)
        raw_levels = np.sqrt(np.sum(signed ** 2, axis=1))
    _require(bool(np.all(np.isfinite(raw_levels))), capture_id + ": nonfinite baseline-subtracted magnitudes")
    _require(bool(np.all(np.diff(times) > 0)), capture_id + ": non-increasing sample times")
    result = {"id": capture_id, "kind": kind, "role": role, "axis": axis, "run_id": run_id,
              "record": record, "capture": capture, "times": times, "signed": signed}
    if kind == "motion":
        _require(capture.get("raw_samples_validated") is True
                 and record.get("phase_time_source") == "actual_commanded_trapq",
                 capture_id + ": moving labels require complete validated actual-commanded-trapq capture")
        segments = record.get("segments")
        _require(isinstance(segments, list) and 3 <= len(segments) <= 32
                 and all(isinstance(segment, dict) for segment in segments),
                 capture_id + ": missing commanded motion phases")
        phases = [segment.get("phase") for segment in segments if isinstance(segment, dict)]
        _require(phases in (["acceleration", "cruise", "braking"],
                            ["startup", "acceleration", "cruise", "braking"]),
                 capture_id + ": ambiguous or incomplete motion phase labels")
        previous = None
        for segment in segments:
            left, right = segment.get("start_time_s"), segment.get("end_time_s")
            _require(_finite(left) and _finite(right) and left < right and segment.get("axis") == axis,
                     capture_id + ": invalid commanded phase bounds")
            _require(previous is None or abs(left - previous) <= 1e-6,
                     capture_id + ": discontinuous commanded phase labels")
            previous = right
        acceleration = next(segment for segment in segments if segment["phase"] == "acceleration")
        cruise = next(segment for segment in segments if segment["phase"] == "cruise")
        braking = next(segment for segment in segments if segment["phase"] == "braking")
        arm = acceleration["start_time_s"]
        rate = _reference_rate(capture, arm, motion=True)
        start = int(np.searchsorted(times, arm, side="left"))
        end = int(np.searchsorted(times, braking["start_time_s"], side="left"))
        lo, hi = cruise["start_time_s"] + CRUISE_EDGE_SECONDS, cruise["end_time_s"] - CRUISE_EDGE_SECONDS
        _require(hi - lo >= MIN_USABLE_CRUISE_SECONDS - 1e-9,
                 capture_id + ": insufficient cruise duration after boundary exclusions")
        _require(times[0] <= arm and times[-1] >= braking["end_time_s"] - 1.02 / rate,
                 capture_id + ": capture truncates the commanded stroke")
        cruise_start, cruise_end = int(np.searchsorted(times, lo)), int(np.searchsorted(times, hi))
        _require(0 <= start < cruise_start < cruise_end <= end < len(times)
                 and times[start] - arm <= 1.02 / rate and cruise_end - cruise_start >= 128,
                 capture_id + ": incomplete eligible virtual-arm or cruise samples")
        speed = record.get("effective_speed_mm_s")
        accel = record.get("effective_accel_mm_s2")
        _require(_finite(speed) and speed > 0 and _finite(accel) and accel > 0,
                 capture_id + ": missing effective commanded motion conditions")
        result.update(start=start, end=end, cruise_start=cruise_start, cruise_end=cruise_end,
                      startup_end=cruise_start, duration=hi - lo, speed=float(speed), accel=float(accel),
                      arm=arm, arm_source="actual_commanded_acceleration_start")
    else:
        _require(record.get("auto_notch_reference") == "qualified_raw_trigger_reference"
                 and record.get("result") == "impact",
                 capture_id + ": home lacks independent commanded/plausibility qualification")
        meta, detector = capture.get("auto_notch_replay"), capture.get("detector")
        _require(isinstance(meta, dict) and isinstance(detector, dict),
                 capture_id + ": missing exact live arm/reference provenance")
        start = meta.get("first_sample_index")
        _require(type(start) is int and 0 <= start < len(rows) and meta.get("first_sample_matches") == 1
                 and meta.get("first_sample_time_s") == rows[start][0]
                 and meta.get("initial_state") == "zero_at_first_armed_sample"
                 and meta.get("complete") is True,
                 capture_id + ": incomplete or ambiguous first armed sample identity")
        rate = _reference_rate(meta, times[start], motion=False)
        onset = detector.get("onset_sample_index")
        arm = detector.get("arm_time_s")
        _require(type(onset) is int and start + 3 <= onset < len(rows)
                 and detector.get("onset_sample_matches") == 1
                 and detector.get("onset_time_s") == rows[onset][0]
                 and detector.get("attempt_count") == 1 and detector.get("axis") == axis
                 and detector.get("impact_metric") == "xyz_magnitude"
                 and detector.get("threshold_g") == raw_threshold and _finite(arm),
                 capture_id + ": missing exact raw reference-trigger identity")
        _home_eligibility(meta, times[start], arm, rate)
        _require(capture.get("signal_filter", DISABLED) == DISABLED,
                 capture_id + ": reference homes must use the disabled detector")
        levels = np.sqrt(np.sum(signed[start:onset + 1] ** 2, axis=1))
        _require(float(levels[-1]) >= raw_threshold * (1 - 1e-12)
                 and bool(np.all(levels[:-1] < raw_threshold * (1 + 1e-12))),
                 capture_id + ": recorded onset is not the first raw threshold crossing")
        speed, accel = detector.get("homing_speed_mm_s"), detector.get("homing_accel_mm_s2")
        _require(_finite(speed) and speed > 0 and _finite(accel) and accel > 0,
                 capture_id + ": missing matched commanded home conditions")
        result.update(start=start, end=onset + 1, onset=onset, speed=float(speed), accel=float(accel),
                      arm=arm, arm_source="recorded_live_first_eligible_sample",
                      lead_limit=min(2.0 / rate, 0.25 / speed))
    intervals = np.diff(times)
    _require(bool(np.all(np.abs(intervals * rate - 1) <= RATE_TOLERANCE + 1e-8)),
             capture_id + ": sample-rate drift or missing/nonuniform samples")
    result["rate"] = rate
    result["raw"] = raw_levels
    # A new ID or time translation must not make the same capture independent.
    fingerprint = hashlib.sha256()
    fingerprint.update(np.asarray(signed[result["start"]:result["end"]], dtype="<f8").tobytes())
    fingerprint.update(("%.6g" % rate).encode("ascii"))
    result["fingerprint"] = fingerprint.hexdigest()
    return result


def _welch(signed, rate, np):
    length = min(MAX_WELCH_SEGMENT, len(signed) // 2)
    _require(length >= 64, "insufficient cruise samples for three Welch segments")
    step = length // 2
    window = np.hanning(length)
    scale = rate * float(np.sum(window ** 2))
    powers = []
    for start in range(0, len(signed) - length + 1, step):
        segment = signed[start:start + length]
        centered = segment - np.mean(segment, axis=0)
        with np.errstate(over="ignore", invalid="ignore"):
            transformed = np.fft.rfft(centered * window[:, None], axis=0)
            density = np.sum(np.abs(transformed) ** 2, axis=1) / scale
        _require(bool(np.all(np.isfinite(density))), "nonfinite signed XYZ spectral power")
        density[1:-1 if length % 2 == 0 else None] *= 2
        powers.append(density)
    _require(len(powers) >= 3, "insufficient independent Welch coverage")
    return np.fft.rfftfreq(length, 1 / rate), np.mean(powers, axis=0), length, len(powers)


def _spectral_peaks(frequencies, power, np):
    maximum = min(MAX_FREQUENCY_HZ, float(frequencies[-1]) * 0.9)
    eligible = (frequencies >= MIN_FREQUENCY_HZ) & (frequencies <= maximum)
    band_power = float(np.sum(power[eligible]))
    if band_power <= 0:
        return []
    peaks, spacing = [], float(frequencies[1])
    for index in range(2, len(power) - 2):
        if not eligible[index] or not power[index] > max(power[index - 1], power[index + 1]):
            continue
        neighbors = np.concatenate((power[max(1, index - 8):index - 1], power[index + 2:index + 9]))
        background = max(float(np.median(neighbors)), band_power * 1e-15)
        prominence = float(power[index]) / background
        fraction = float(np.sum(power[index - 1:index + 2])) / band_power
        if prominence < MIN_PEAK_PROMINENCE or fraction < MIN_PEAK_POWER_FRACTION:
            continue
        logs = np.log(np.maximum(power[index - 1:index + 2], band_power * 1e-15))
        denominator = float(logs[0] - 2 * logs[1] + logs[2])
        delta = 0 if denominator == 0 else max(-0.5, min(0.5, float(0.5 * (logs[0] - logs[2]) / denominator)))
        left = right = index
        while left > 1 and power[left - 1] >= power[index] * 0.5:
            left -= 1
        while right < len(power) - 2 and power[right + 1] >= power[index] * 0.5:
            right += 1
        width = (right - left + 1) * spacing
        if width <= max(3 * spacing, 0.2 * float(frequencies[index])):
            peaks.append({"frequency_hz": float(frequencies[index] + delta * spacing),
                          "bin_width_hz": spacing, "half_power_width_hz": width,
                          "prominence": prominence, "band_power_fraction": fraction})
    return sorted(peaks, key=lambda item: (-item["band_power_fraction"], item["frequency_hz"]))[:6]


def digital_bandwidth(frequency, q, rate):
    """Exact full -3 dB bandwidth of this RBJ notch, in Hz.

    |H|^2=cos-distance^2/(cos-distance^2 + alpha^2*sin(w)^2).
    The two half-power roots differ by 2*atan(alpha), alpha=sin(w0)/(2Q).
    """
    return rate / math.pi * math.atan(math.sin(2 * math.pi * frequency / rate) / (2 * q))


def _resolution(frequency, q, rate, duration, bin_width):
    bandwidth = digital_bandwidth(frequency, q, rate)
    required = max(2 / duration, MIN_BANDWIDTH_BINS * bin_width)
    return {"digital_bandwidth_hz": bandwidth, "minimum_resolved_bandwidth_hz": required,
            "welch_bin_width_hz": bin_width, "cycles": frequency * duration,
            "resolved": frequency * duration >= MIN_CYCLES and bandwidth >= required}


def _replay(data, settings, np, check_running):
    start, end = data["start"], data["end"]
    if settings == DISABLED:
        return data["signed"][start:end].copy()
    filt = CausalNotch(settings, data["rate"])
    result = np.empty((end - start, 3))
    for offset, row in enumerate(data["signed"][start:end]):
        if offset % 1024 == 0:
            _check(check_running)
        result[offset] = filt.process(row)
    return result


def _metrics(data, filtered, np):
    levels = np.sqrt(np.sum(filtered ** 2, axis=1))
    raw = data["raw"][data["start"]:data["end"]]
    result = {"capture_id": data["id"], "kind": data["kind"], "role": data["role"]}
    if data["kind"] == "motion":
        a, b = data["cruise_start"] - data["start"], data["cruise_end"] - data["start"]
        raw_peak, filtered_peak = float(np.max(raw[a:b])), float(np.max(levels[a:b]))
        startup_raw, startup_filtered = float(np.max(raw[:a])), float(np.max(levels[:a]))
        result.update(raw_cruise_peak_g=raw_peak, filtered_cruise_peak_g=filtered_peak,
                      cruise_peak_reduction=(1 - filtered_peak / raw_peak) if raw_peak else 0.0,
                      raw_startup_acceleration_peak_g=startup_raw,
                      filtered_startup_acceleration_peak_g=startup_filtered,
                      noise_peak_g=float(np.max(levels)), raw_noise_peak_g=float(np.max(raw)))
    else:
        times = data["times"][data["start"]:data["end"]]
        before = times < times[-1] - data["lead_limit"] - 1e-9 / data["rate"]
        _require(bool(np.any(before)), "reference event lacks a complete armed noise prefix")
        result.update(raw_reference_stop_g=float(raw[-1]), filtered_reference_stop_g=float(levels[-1]),
                      reference_stop_retention=float(levels[-1] / raw[-1]),
                      noise_peak_g=float(np.max(levels[before])), raw_noise_peak_g=float(np.max(raw[before])),
                      reference_stop_time_s=float(times[-1]), reference_lead_limit_s=data["lead_limit"])
    return result, levels


def _crossing(data, levels, threshold, np):
    hits = np.flatnonzero(levels >= threshold)
    return None if not len(hits) else float(data["times"][data["start"] + int(hits[0])])


def _evaluate(dataset, settings, threshold, np, check_running):
    measured = []
    for data in dataset:
        filtered = _replay(data, settings, np, check_running)
        metrics, levels = _metrics(data, filtered, np)
        if threshold is not None and data["kind"] == "home":
            crossing = _crossing(data, levels, threshold, np)
            raw = data["raw"][data["start"]:data["end"]]
            metrics.update(raw_crossing_time_s=_crossing(data, raw, threshold, np),
                           filtered_crossing_time_s=crossing,
                           reference_crossing_shift_s=None if crossing is None else crossing - metrics["reference_stop_time_s"])
        measured.append(metrics)
    return measured


def _acceptance(metrics, threshold, role):
    reasons = []
    for item in metrics:
        prefix = item["capture_id"] + ": "
        if item["kind"] == "motion":
            reduction = TRAIN_REDUCTION if role == "train" else HOLDOUT_REDUCTION
            if item["cruise_peak_reduction"] < reduction:
                reasons.append(prefix + "insufficient cruise maximum reduction")
            if item["filtered_startup_acceleration_peak_g"] > item["raw_startup_acceleration_peak_g"] + 1e-10:
                reasons.append(prefix + "causal startup/acceleration peak increased")
            if item["noise_peak_g"] >= threshold:
                reasons.append(prefix + "free-travel first crossing at frozen threshold")
        else:
            if item["reference_stop_retention"] < REFERENCE_RETENTION:
                reasons.append(prefix + "reference-stop level retention below minimum")
            shift = item.get("reference_crossing_shift_s")
            if shift is None:
                reasons.append(prefix + "no causal crossing by original raw stop event")
            elif shift > 1e-12 or shift < -item["reference_lead_limit_s"] - 1e-9:
                reasons.append(prefix + "first crossing outside allowed reference-stop alignment")
    return reasons


def _control_acceptance(metrics, threshold):
    """Disabled is a preregistered candidate, not an unconditional fallback."""
    reasons = []
    for item in metrics:
        if item["kind"] == "motion":
            if item["raw_noise_peak_g"] >= threshold:
                reasons.append(item["capture_id"] + ": disabled control crosses during independent free travel")
        elif item["raw_crossing_time_s"] != item["reference_stop_time_s"]:
            reasons.append(item["capture_id"] + ": disabled control does not reproduce the exact raw reference stop")
    return reasons


def _trace_indices(raw, filtered, cap, np):
    count = len(raw)
    if count <= cap:
        return np.arange(count)
    # Four extrema per bucket plus both endpoints; do not erase a short impulse.
    indices = {0, count - 1}
    edges = np.linspace(0, count, (cap - 2) // 4 + 1, dtype=int)
    for a, b in zip(edges[:-1], edges[1:]):
        for values in (raw, filtered):
            indices.add(a + int(np.argmin(values[a:b])))
            indices.add(a + int(np.argmax(values[a:b])))
    return np.asarray(sorted(indices), dtype=int)


def _plot(dataset, settings, threshold, np, check_running):
    result = {"spectra": [], "traces": [], "profile": dict(settings), "threshold_g": threshold}
    for data in dataset:
        filtered = _replay(data, settings, np, check_running)
        raw = data["raw"][data["start"]:data["end"]]
        levels = np.sqrt(np.sum(filtered ** 2, axis=1))
        indices = _trace_indices(raw, levels, MAX_TRACE_POINTS, np)
        times = data["times"][data["start"]:data["end"]]
        trace = {"capture_id": data["id"], "kind": data["kind"], "role": data["role"],
                 "time_s": times[indices].tolist(), "raw_magnitude_g": raw[indices].tolist(),
                 "filtered_magnitude_g": levels[indices].tolist(), "threshold_g": threshold,
                 "raw_crossing_time_s": _crossing(data, raw, threshold, np),
                 "filtered_crossing_time_s": _crossing(data, levels, threshold, np),
                 "original_sample_count": len(times), "reference_stop_time_s": None}
        if data["kind"] == "home":
            trace["reference_stop_time_s"] = float(times[-1])
        else:
            a, b = data["cruise_start"] - data["start"], data["cruise_end"] - data["start"]
            frequencies, raw_power, _, _ = _welch(data["signed"][data["cruise_start"]:data["cruise_end"]], data["rate"], np)
            _, filtered_power, _, _ = _welch(filtered[a:b], data["rate"], np)
            result["spectra"].append({"capture_id": data["id"], "role": data["role"],
                                      "frequency_hz": frequencies.tolist(),
                                      "raw_power_g2_hz": raw_power.tolist(),
                                      "filtered_power_g2_hz": filtered_power.tolist()})
        result["traces"].append(trace)
    return result


def analyze(motions, homes, *, raw_threshold_g, threshold_min_g=0.5,
            threshold_max_g, check_running=None):
    """Return JSON-ready evidence or an explicit insufficient_data rejection.

    Inputs are three central stroke and ten qualified raw-home {record,capture}
    entries in acquisition order. The first 2/6 train; the final 1/4 are held out.
    Raw home captures require auto_notch_replay exact arm/reference metadata.
    Callback cancellation errors propagate without disguising partial results.
    """
    _check(check_running)
    preflight()
    import numpy as np
    report = {"policy": _policy(), "status": "insufficient_data", "decision": None,
              "profile": None, "threshold_g": None, "reasons": [],
              "requires_full_homing_validation": True, "physical_position_verified": False,
              "evidence": {"captures": [], "spectral": [], "candidates": [],
                           "frozen_candidate": None, "holdout": None},
              "plot": {"spectra": [], "traces": []}}
    try:
        _require(all(_finite(value) for value in (raw_threshold_g, threshold_min_g, threshold_max_g))
                 and 0 < threshold_min_g <= raw_threshold_g <= threshold_max_g,
                 "invalid raw threshold or candidate threshold bounds")
        _require(isinstance(motions, (list, tuple)) and len(motions) == 3
                 and isinstance(homes, (list, tuple)) and len(homes) == 10,
                 "require three independent motion captures and ten qualified raw reference homes")
        all_items = list(motions) + list(homes)
        _require(all(isinstance(item, dict) and isinstance(item.get("capture"), dict)
                     and isinstance(item["capture"].get("samples"), (list, tuple)) for item in all_items),
                 "capture entries require complete raw sample lists")
        _require(sum(len(item["capture"]["samples"]) for item in all_items) <= MAX_TOTAL_SAMPLES,
                 "total raw capture sample cap exceeded")
        dataset = ([_prepare(item, "motion", index, raw_threshold_g, np, check_running)
                    for index, item in enumerate(motions)]
                   + [_prepare(item, "home", index, raw_threshold_g, np, check_running)
                      for index, item in enumerate(homes)])
        _require(len({item["id"] for item in dataset}) == len(dataset)
                 and len({item["fingerprint"] for item in dataset}) == len(dataset),
                 "reused or duplicate captures are not independent training/held-out evidence")
        _require(all(right["times"][0] > left["times"][-1] for left, right in zip(dataset, dataset[1:])),
                 "acquisition times must be ordered and nonoverlapping")
        for group, key in ((dataset[:3], "iteration"), (dataset[3:], "workflow_attempt")):
            _require(all(right["record"][key] > left["record"][key] for left, right in zip(group, group[1:])),
                     "acquisition identities must increase within each independent capture group")
        _require(len({(item["run_id"], item["axis"]) for item in dataset}) == 1,
                 "capture run or axis changed")
        first = dataset[0]
        _require(all(abs(item["rate"] / first["rate"] - 1) <= RATE_TOLERANCE
                     and math.isclose(item["speed"], first["speed"], rel_tol=1e-6)
                     and math.isclose(item["accel"], first["accel"], rel_tol=1e-6) for item in dataset),
                 "sample rate or commanded speed/acceleration changed between captures")
        for item in dataset:
            report["evidence"]["captures"].append({
                "capture_id": item["id"], "kind": item["kind"], "role": item["role"],
                "scored_signed_xyz_sha256": item["fingerprint"], "sample_rate_hz": item["rate"],
                "sample_count": len(item["times"]), "first_sample_index": item["start"],
                "first_sample_time_s": float(item["times"][item["start"]]),
                "last_scored_sample_index": item["end"] - 1,
                "last_scored_sample_time_s": float(item["times"][item["end"] - 1]),
                "arm_source": item["arm_source"], "initial_state": "zero_at_first_eligible_sample",
                "baseline_g": list(item["capture"]["baseline_g"]),
                "source_first_sample_identity": ((item["capture"].get("auto_notch_replay") or {}).get("first_sample")
                                                 if item["kind"] == "home" else None),
                "source_arm_eligibility_domain": ((item["capture"].get("auto_notch_replay") or {}).get("eligibility_source")
                                                  if item["kind"] == "home" else "actual_commanded_acceleration_start"),
                "post_reference_stop_samples_scored": 0 if item["kind"] == "home" else None})
        train, heldout = ([item for item in dataset if item["role"] == role] for role in ("train", "holdout"))
        spectral_peaks = []
        for item in train[:2]:
            frequencies, powers, length, count = _welch(item["signed"][item["cruise_start"]:item["cruise_end"]], item["rate"], np)
            peaks = _spectral_peaks(frequencies, powers, np)
            item["welch_bin_width"] = float(frequencies[1])
            spectral_peaks.append(peaks)
            report["evidence"]["spectral"].append({"capture_id": item["id"], "role": "train",
                "usable_cruise_duration_s": item["duration"], "welch_segment_samples": length,
                "welch_segments": count, "peaks": peaks})
        recurring = []
        for peak in spectral_peaks[0]:
            matches = [other for other in spectral_peaks[1]
                       if abs(peak["frequency_hz"] - other["frequency_hz"]) <= max(
                           peak["bin_width_hz"], other["bin_width_hz"], 0.03 * peak["frequency_hz"])]
            if matches:
                other = min(matches, key=lambda value: abs(value["frequency_hz"] - peak["frequency_hz"]))
                frequency = (peak["frequency_hz"] + other["frequency_hz"]) / 2
                if not any(abs(frequency - old) < peak["bin_width_hz"] for old in recurring):
                    recurring.append(frequency)
            if len(recurring) == MAX_PEAKS:
                break
        report["evidence"]["recurring_frequency_hz"] = recurring
        control_train = _evaluate(train, DISABLED, raw_threshold_g, np, check_running)
        report["evidence"]["disabled_control"] = {"profile": dict(DISABLED), "threshold_g": raw_threshold_g,
            "preregistered": True, "train_metrics": control_train,
            "train_reasons": _control_acceptance(control_train, raw_threshold_g)}
        passing = []
        for frequency in recurring:
            for q in Q_GRID:
                _check(check_running)
                settings = profile(frequency, q)
                candidate = {"profile": settings, "threshold_g": None, "train_accepted": False,
                             "reasons": [], "metrics": []}
                report["evidence"]["candidates"].append(candidate)
                resolution = [_resolution(frequency, q, item["rate"], item["duration"], item["welch_bin_width"])
                              for item in train[:2]]
                candidate["resolution"] = resolution
                if any(not item["resolved"] for item in resolution):
                    candidate["reasons"].append("insufficient cycles or unresolved digital notch bandwidth")
                    continue
                metrics = _evaluate(train, settings, None, np, check_running)
                peak = max(item["noise_peak_g"] for item in metrics)
                threshold = max(threshold_min_g, peak * 1.2, peak + 0.1)
                candidate["threshold_g"] = threshold
                if threshold > threshold_max_g:
                    candidate.update(metrics=metrics, reasons=["training noise margin exceeds threshold upper bound"])
                    continue
                metrics = _evaluate(train, settings, threshold, np, check_running)
                reasons = _acceptance(metrics, threshold, "train")
                candidate.update(metrics=metrics, reasons=reasons, train_accepted=not reasons)
                if not reasons:
                    score = (min(item["cruise_peak_reduction"] for item in metrics if item["kind"] == "motion"),
                             min(item["reference_stop_retention"] for item in metrics if item["kind"] == "home"), q)
                    passing.append((score, candidate))
        if recurring and all(not item["metrics"] for item in report["evidence"]["candidates"]):
            raise InsufficientData("recurring peaks lack enough cycles or resolvable notch bandwidth")
        chosen, threshold = dict(DISABLED), raw_threshold_g
        plot_settings, plot_threshold = chosen, threshold
        reasons = ["no recurring narrow cruise spectral peak"] if not recurring else ["no candidate improves disabled control while preserving reference-stop crossings"]
        if passing:
            winner = max(passing, key=lambda entry: entry[0])[1]
            frozen = {"profile": dict(winner["profile"]), "threshold_g": winner["threshold_g"],
                      "selection_data": "train_only", "retuned_after_holdout": False}
            report["evidence"]["frozen_candidate"] = frozen
            metrics = _evaluate(heldout, frozen["profile"], frozen["threshold_g"], np, check_running)
            failures = _acceptance(metrics, frozen["threshold_g"], "holdout")
            report["evidence"]["holdout"] = {"accepted": not failures, "reasons": failures, "metrics": metrics}
            plot_settings, plot_threshold = frozen["profile"], frozen["threshold_g"]
            if failures:
                reasons = ["frozen candidate failed independent held-out acceptance; disabled selected without retuning"] + failures
            else:
                chosen, threshold = frozen["profile"], frozen["threshold_g"]
                reasons = ["frozen candidate improved independent free-travel peaks and retained causal reference-stop events"]
        control_holdout = _evaluate(heldout, DISABLED, raw_threshold_g, np, check_running)
        control = report["evidence"]["disabled_control"]
        control["holdout_metrics"] = control_holdout
        control["holdout_reasons"] = _control_acceptance(control_holdout, raw_threshold_g)
        control["accepted"] = not (control["train_reasons"] or control["holdout_reasons"])
        selected = chosen != DISABLED
        report.update(status="selected" if selected else "no_benefit", decision="selected" if selected else "disabled",
                      profile=chosen, threshold_g=threshold, reasons=reasons)
        if not selected and not control["accepted"]:
            report.update(status="rejected", decision=None, profile=None, threshold_g=None,
                          reasons=["no accepted frozen notch and preregistered disabled control failed"]
                          + reasons + control["train_reasons"] + control["holdout_reasons"])
        report["plot"] = _plot(dataset, plot_settings, plot_threshold, np, check_running)
        report["plot"]["description"] = ("frozen candidate comparison" if passing else "disabled control; no acceptable trained candidate")
        _check(check_running)
    except InsufficientData as exc:
        report.update(status="insufficient_data", decision=None, profile=None, threshold_g=None, reasons=[str(exc)])
    return report
