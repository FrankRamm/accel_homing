# Automatic homing-axis identification. Distributed under GNU GPLv3.
"""Identify one physical axis in the already configured sensor stream.

This is an offline calibration calculation, not a live impact filter. Signed
velocity during known bidirectional motion identifies a channel; actual homing
uses that channel without this integration or smoothing. No driver axes_map is
applied here: capture adapters have already applied it.
"""
import bisect
import copy
import math
import statistics


FRAME = "configured_sensor_stream"
METHOD = "bidirectional_velocity_v1"
CONFIGURED_METHOD = "configured_axes_map_v1"
GRAVITY_MM_S2 = 9806.65
MIN_ALIGNMENT = 0.85
MIN_DIRECTION_AGREEMENT = 0.95
MIN_STABLE_CRUISE_FRACTION = 0.95
CRUISE_WINDOW_S = 0.020
MIN_LOCAL_MOTION_RATIO = 0.25
MAX_DIAGNOSTIC_PASSES = 8


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _finite_tree(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(isinstance(key, str) and _finite_tree(item)
                   for key, item in value.items())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(item) for item in value)
    return value is None or isinstance(value, (str, bool, int))


def normalize_configured_axes_map(value):
    """Normalize an explicit driver option, never substitute a default map."""
    if isinstance(value, str):
        value = value.split(",")
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(not isinstance(token, str) for token in value)):
        raise ValueError("Configured axes map must contain three signed axes")
    tokens = [token.strip() for token in value]
    if (any(token not in ("x", "y", "z", "-x", "-y", "-z") for token in tokens)
            or len({token[-1] for token in tokens}) != 3):
        raise ValueError("Configured axes map must be a signed permutation of x, y, z")
    return tokens


def validate_axis_mapping(mapping, axis=None):
    """Validate persisted coordinate semantics; return an independent copy."""
    if not isinstance(mapping, dict) or not _finite_tree(mapping):
        raise ValueError("Axis mapping must contain finite JSON metadata")
    if type(mapping.get("version")) is not int or mapping["version"] != 1:
        raise ValueError("Unsupported axis mapping version")
    if mapping.get("axis") not in ("x", "y") or (
            axis is not None and mapping["axis"] != axis):
        raise ValueError("Axis mapping does not match the requested machine axis")
    if (type(mapping.get("source_axis")) is not int
            or mapping["source_axis"] not in (0, 1, 2)):
        raise ValueError("Axis mapping source_axis must be 0, 1 or 2")
    if type(mapping.get("sign")) is not int or mapping["sign"] not in (-1, 1):
        raise ValueError("Axis mapping sign must be +1 or -1")
    if (mapping.get("coordinate_frame") != FRAME
            or mapping.get("method") not in (METHOD, CONFIGURED_METHOD)):
        raise ValueError("Unsupported axis mapping coordinate frame or method")
    if mapping["method"] == CONFIGURED_METHOD:
        if mapping["source_axis"] != "xy".index(mapping["axis"]) or mapping["sign"] != 1:
            raise ValueError("Configured mapping must select the machine axis in the configured sensor stream")
        if any(key in mapping for key in ("confidence", "passes", "direction_agreement")):
            raise ValueError("Configured mapping cannot claim measured inference evidence")
        declaration = mapping.get("configuration")
        if not isinstance(declaration, dict):
            raise ValueError("Configured mapping requires its explicit sensor configuration")
        section = declaration.get("sensor_section")
        if not isinstance(section, str) or not section or section != section.strip():
            raise ValueError("Configured mapping requires the selected sensor section")
        option = "accel_axes_map" if section.split()[0] == "beacon" else "axes_map"
        if declaration.get("option") != option:
            raise ValueError("Configured mapping has the wrong axes-map option for its sensor")
        tokens = declaration.get("axes_map")
        if not isinstance(tokens, list) or normalize_configured_axes_map(tokens) != tokens:
            raise ValueError("Configured mapping requires a normalized signed axes-map permutation")
    if "confidence" in mapping and (not _number(mapping["confidence"])
                                    or not MIN_ALIGNMENT <= mapping["confidence"] <= 1):
        raise ValueError("Axis mapping is ambiguous or has invalid confidence")
    return copy.deepcopy(mapping)


class MappedCaptureError(ValueError):
    """The retained measurements cannot be attributed to the requested axis."""


def mapped_axis_capture(capture, mapping, check_running=None):
    """Retain the signed machine-axis component in g, before baseline subtraction.

    Filtered samples, when present, are already baseline-subtracted. A compact
    capture already includes the mapping sign and must never be mapped again.
    """
    try:
        mapping = validate_axis_mapping(mapping)
    except ValueError as exc:
        raise MappedCaptureError(str(exc)) from exc
    if not isinstance(capture, dict) or capture.get("axis_mapping", mapping) != mapping:
        raise MappedCaptureError("Capture axis mapping differs from the selected axis")
    if capture.get("coordinate_frame", FRAME) != FRAME:
        raise MappedCaptureError("Unexpected capture coordinate frame")
    layout = capture.get("sample_layout")
    compact = layout == "mapped_axis_v1"
    columns = ["time_s", "mapped_axis_g"]
    if layout is not None and not compact:
        raise MappedCaptureError("Unknown accelerometer sample layout")
    if compact and (capture.get("axis_mapping") != mapping
                    or list(capture.get("sample_columns") or ()) != columns):
        raise MappedCaptureError("Incomplete mapped capture provenance")
    source, sign = mapping["source_axis"], mapping["sign"]
    if capture.get("impact_metric", "mapped_axis") != "mapped_axis":
        raise MappedCaptureError("Capture uses a different detector metric")
    result = dict(capture, impact_metric="mapped_axis", axis_mapping=mapping, coordinate_frame=FRAME,
                  sample_layout="mapped_axis_v1", sample_columns=columns)
    baseline = capture.get("baseline_g")
    if baseline is not None:
        if compact:
            if not _number(baseline):
                raise MappedCaptureError("Missing exact signed mapped-axis baseline")
        elif (not isinstance(baseline, (list, tuple)) or len(baseline) != 3
              or not _number(baseline[source])):
            raise MappedCaptureError("Missing exact signed mapped-axis baseline")
        else:
            result["baseline_g"] = sign * baseline[source]
    for field in ("samples", "filtered_samples"):
        if field not in capture:
            continue
        rows = capture[field]
        if not isinstance(rows, (list, tuple)):
            raise MappedCaptureError("Malformed accelerometer sample list")
        selected = [] if not compact else None
        width, channel = (2, 1) if compact else (4, source + 1)
        for index, row in enumerate(rows):
            if index % 2048 == 0 and check_running is not None:
                check_running()
            if (not isinstance(row, (list, tuple)) or len(row) != width
                    or not _number(row[0]) or not _number(row[channel])):
                raise MappedCaptureError("Malformed or nonfinite mapped-axis samples")
            if not compact:
                selected.append((row[0], sign * row[channel]))
        if not compact:
            result[field] = selected
    return result


def _interpolate(times, values, time):
    index = bisect.bisect_left(times, time)
    if index < len(times) and times[index] == time:
        return values[index]
    if index == 0 or index == len(times):
        raise ValueError("Mapping capture does not cover the commanded motion")
    fraction = (time - times[index - 1]) / (times[index] - times[index - 1])
    return tuple(a + fraction * (b - a)
                 for a, b in zip(values[index - 1], values[index]))


def _diagnostic_number(value):
    return value if _number(value) else None


def _diagnostic_vector(values):
    return [_diagnostic_number(value) for value in values[:3]]


def _cruise_windows(times, vectors, left, right):
    """Time-weighted signed means, with full coverage and no long final bin.

    Pool XYZ before taking a norm or projection: rectifying first could turn
    opposing motion into false evidence of a channel. Integrate the piecewise
    linear velocity at exact time boundaries, independent of the sample rate.
    """
    count = max(1, math.ceil((right - left) / CRUISE_WINDOW_S))
    width = (right - left) / count
    result = []
    for index in range(count):
        start = left + index * width
        end = right if index == count - 1 else left + (index + 1) * width
        first, last = bisect.bisect_right(times, start), bisect.bisect_left(times, end)
        points = [(start, _interpolate(times, vectors, start))]
        points.extend(zip(times[first:last], vectors[first:last]))
        points.append((end, _interpolate(times, vectors, end)))
        mean = tuple(math.fsum((v0[channel] + v1[channel]) * .5 * (t1 - t0)
                               for (t0, v0), (t1, v1) in zip(points, points[1:])) / (end - start)
                     for channel in range(3))
        result.append((end - start, mean))
    return result


def _pass_velocity(axis, item, diagnostics=None):
    if not isinstance(item, dict) or type(item.get("direction")) is not int or item["direction"] not in (-1, 1):
        raise ValueError("Mapping pass requires a signed commanded direction")
    if diagnostics is not None:
        diagnostics["direction"] = item["direction"]
    capture = item.get("capture", {})
    if not isinstance(capture, dict):
        raise ValueError("Mapping pass has no capture")
    if (capture.get("fault") or capture.get("dropped", 0)
            or capture.get("amplitude_limited") or capture.get("excess_amplitude_unknown")
            or capture.get("reference_signal_quality", {}).get("amplitude_limited")):
        raise ValueError("Mapping capture is faulty, incomplete or amplitude limited")
    baseline = capture.get("baseline_g")
    if not isinstance(baseline, (list, tuple)) or len(baseline) != 3 or not all(_number(v) for v in baseline):
        raise ValueError("Mapping capture requires a finite stationary XYZ reference")
    if diagnostics is not None:
        diagnostics["baseline_xyz_g"] = _diagnostic_vector(baseline)
    samples = capture.get("samples", ())
    if not isinstance(samples, (list, tuple)) or len(samples) < 50:
        raise ValueError("Mapping capture contains too few samples")
    if any(not isinstance(row, (list, tuple)) or len(row) != 4 or not all(_number(v) for v in row)
           for row in samples):
        raise ValueError("Mapping capture contains invalid samples")
    times = [row[0] for row in samples]
    intervals = [right - left for left, right in zip(times, times[1:])]
    if diagnostics is not None:
        diagnostics.update(sample_count=len(samples), sample_start_time_s=times[0],
                           sample_end_time_s=times[-1],
                           minimum_sample_interval_s=_diagnostic_number(min(intervals)),
                           maximum_sample_interval_s=_diagnostic_number(max(intervals)))
    if min(intervals) <= 0:
        raise ValueError("Mapping samples must have strictly increasing timestamps")
    interval = statistics.median(intervals)
    if diagnostics is not None:
        diagnostics.update(median_sample_interval_s=_diagnostic_number(interval),
                           sample_rate_hz=_diagnostic_number(1.0 / interval))
    if interval > 0.01 or max(intervals) > 3.0 * interval:
        raise ValueError("Mapping capture has insufficient rate or a sample gap")
    segments = item.get("segments", ())
    motion = []
    for segment in segments:
        if not isinstance(segment, dict):
            raise ValueError("Invalid mapping planner segment")
        if segment.get("phase") == "startup":
            continue
        phase = segment.get("phase")
        values = [segment.get(key) for key in ("start_time_s", "end_time_s",
                                              "start_velocity_mm_s", "acceleration_mm_s2")]
        if (phase not in ("acceleration", "cruise", "braking")
                or not all(_number(v) for v in values) or values[1] <= values[0]
                or segment.get("axis", axis) != axis):
            raise ValueError("Invalid mapping commanded-motion profile")
        velocity, accel = values[2:]
        if (velocity < 0 or (phase == "acceleration" and accel <= 0)
                or (phase == "braking" and accel >= 0)
                or (phase == "cruise" and accel != 0)):
            raise ValueError("Mapping planner phase disagrees with velocity/acceleration")
        motion.append(segment)
    if not motion or {s["phase"] for s in motion} != {"acceleration", "cruise", "braking"}:
        raise ValueError("Mapping needs acceleration, cruise and braking evidence")
    motion.sort(key=lambda segment: segment["start_time_s"])
    if any(not math.isclose(left["end_time_s"], right["start_time_s"], abs_tol=1e-6, rel_tol=0)
           for left, right in zip(motion, motion[1:])):
        raise ValueError("Mapping commanded-motion profile is discontinuous")
    for left, right in zip(motion, motion[1:]):
        end_velocity = left["start_velocity_mm_s"] + left["acceleration_mm_s2"] * (
            left["end_time_s"] - left["start_time_s"])
        if not math.isclose(end_velocity, right["start_velocity_mm_s"], abs_tol=1e-5, rel_tol=1e-6):
            raise ValueError("Mapping planner velocity is discontinuous")
    last = motion[-1]
    final_velocity = last["start_velocity_mm_s"] + last["acceleration_mm_s2"] * (
        last["end_time_s"] - last["start_time_s"])
    if (not math.isclose(motion[0]["start_velocity_mm_s"], 0., abs_tol=1e-5)
            or not math.isclose(final_velocity, 0., abs_tol=1e-5)):
        raise ValueError("Mapping motion must start and finish at rest")
    start, end = motion[0]["start_time_s"], motion[-1]["end_time_s"]
    if diagnostics is not None:
        diagnostics.update(profile_start_time_s=start, profile_end_time_s=end,
                           profile_duration_s=_diagnostic_number(end - start),
                           profile_segment_count=len(motion),
                           capture_lead_time_s=_diagnostic_number(start - times[0]),
                           capture_tail_time_s=_diagnostic_number(times[-1] - end))
    if end - start < 0.05:
        raise ValueError("Mapping stroke is too short to identify an axis")
    if times[0] > start or times[-1] < end:
        raise ValueError("Mapping capture does not cover the commanded motion")
    values = [tuple((row[i + 1] - baseline[i]) * GRAVITY_MM_S2 for i in range(3))
              for row in samples]
    middle = [(time, value) for time, value in zip(times, values) if start < time < end]
    points = [(start, _interpolate(times, values, start))] + middle + [
        (end, _interpolate(times, values, end))]
    integrated = [(start, (0.0, 0.0, 0.0))]
    velocity = [0.0, 0.0, 0.0]
    for (t0, a0), (t1, a1) in zip(points, points[1:]):
        velocity = [v + (left + right) * 0.5 * (t1 - t0)
                    for v, left, right in zip(velocity, a0, a1)]
        integrated.append((t1, tuple(velocity)))
    # Both endpoints are commanded rest. Remove a constant acceleration bias
    # as a linear velocity drift. This does not run in the homing detector.
    drift = tuple(velocity)
    if diagnostics is not None:
        diagnostics.update(endpoint_drift_xyz_mm_s=_diagnostic_vector(drift),
                           removed_bias_xyz_mm_s2=_diagnostic_vector(
                               tuple(value / (end - start) for value in drift)))
    integrated_times = [time for time, _ in integrated]
    signed_vectors = [tuple((value - bias * (time - start) / (end - start)) * item["direction"]
                            for value, bias in zip(vector, drift))
                      for time, vector in integrated]
    cruise_vectors, cruise_weights, expected_speeds, windows = [], [], [], []
    for segment in motion:
        if segment["phase"] != "cruise":
            continue
        speed = segment["start_velocity_mm_s"]
        if speed <= 0 or segment["acceleration_mm_s2"] != 0:
            raise ValueError("Mapping cruise speed must be finite and positive")
        trim = 0.1 * (segment["end_time_s"] - segment["start_time_s"])
        left, right = segment["start_time_s"] + trim, segment["end_time_s"] - trim
        first, last = bisect.bisect_left(integrated_times, left), bisect.bisect_right(integrated_times, right)
        for index in range(first, last):
            time = integrated_times[index]
            # Assign each raw observation its local time coverage, so changing
            # the sampling density cannot conceal conflicting directions.
            before = left if index == first else (integrated_times[index - 1] + time) * .5
            after = right if index == last - 1 else (time + integrated_times[index + 1]) * .5
            cruise_weights.append(after - before)
            cruise_vectors.append(signed_vectors[index])
            expected_speeds.append(speed)
            if diagnostics is not None:
                diagnostics.setdefault("cruise_start_time_s", time)
                diagnostics["cruise_end_time_s"] = time
        windows.extend(_cruise_windows(integrated_times, signed_vectors, left, right))
    if diagnostics is not None:
        diagnostics["cruise_samples"] = len(cruise_vectors)
    if len(cruise_vectors) < 10:
        raise ValueError("Mapping capture has insufficient full-speed cruise samples")
    vector = tuple(statistics.median(row[i] for _, row in windows) for i in range(3))
    norm = math.sqrt(math.fsum(v * v for v in vector))
    speed = statistics.median(expected_speeds)
    if diagnostics is not None:
        diagnostics.update(velocity_xyz_mm_s=_diagnostic_vector(vector),
                           velocity_norm_mm_s=_diagnostic_number(norm),
                           expected_speed_mm_s=_diagnostic_number(speed),
                           velocity_ratio=_diagnostic_number(norm / speed))
    if not math.isfinite(norm) or not 0.5 * speed <= norm <= 1.5 * speed:
        raise ValueError("Mapping motion does not match commanded speed "
                         "(measured %.3f, commanded %.3f mm/s); check sensor and free travel" % (norm, speed))
    source = max(range(3), key=lambda i: abs(vector[i]))
    alignment = abs(vector[source]) / norm
    if diagnostics is not None:
        diagnostics.update(source_axis=source, sign=1 if vector[source] > 0 else -1,
                           alignment=_diagnostic_number(alignment), alignment_limit=MIN_ALIGNMENT)
    if alignment < MIN_ALIGNMENT:
        raise ValueError("Mapping is ambiguous: best channel %s alignment %.3f is below %.3f; "
                         "sensor tilt or transverse motion is too large" % (
                             "XYZ"[source], alignment, MIN_ALIGNMENT))
    # Test the property we use for homing: the same signed sensor channel
    # should identify the stroke even as the estimated velocity fluctuates.
    unit = tuple(value / norm for value in vector)
    projections = [math.fsum(value * direction for value, direction in zip(row, unit))
                   for row in cruise_vectors]
    parallel = [abs(value - norm) for value in projections]
    # The stroke median has already passed the commanded-speed check.
    # Compare individual samples to that median to detect stops and surges.
    longitudinal_limit = 0.50 * norm
    sign = 1 if vector[source] > 0 else -1
    aligned = [
        sign * row[source] > 0 and sign * row[source] >= MIN_ALIGNMENT * math.hypot(*row)
        for row in cruise_vectors
    ]
    stable_count = sum(on_axis and error <= longitudinal_limit
                       for on_axis, error in zip(aligned, parallel))
    size = len(cruise_vectors)
    stable_fraction = stable_count / size
    cruise_duration = math.fsum(duration for duration, _ in windows)
    raw_coherent_fraction = math.fsum(weight for weight, on_axis in zip(cruise_weights, aligned)
                                     if on_axis) / math.fsum(cruise_weights)
    pooled_projections = [math.fsum(value * direction for value, direction in zip(row, unit))
                          for _, row in windows]
    pooled_stable_duration = math.fsum(
        duration for (duration, row), projection in zip(windows, pooled_projections)
        if sign * row[source] > 0 and sign * row[source] >= MIN_ALIGNMENT * math.hypot(*row)
        and abs(projection - norm) <= longitudinal_limit)
    pooled_stable_fraction = min(1., pooled_stable_duration / cruise_duration)
    minimum_motion = MIN_LOCAL_MOTION_RATIO * norm
    local_motion = min(pooled_projections) >= minimum_motion
    if diagnostics is not None:
        perpendicular = [math.hypot(*(value - projection * direction
                                      for value, direction in zip(row, unit)))
                         for row, projection in zip(cruise_vectors, projections)]
        deviation = statistics.median(math.hypot(a, b) for a, b in zip(parallel, perpendicular))
        near_zero = 0.10 * norm
        diagnostics.update(
            total_median_deviation_mm_s=_diagnostic_number(deviation),
            total_deviation_ratio=_diagnostic_number(deviation / norm),
            parallel_median_deviation_mm_s=_diagnostic_number(statistics.median(parallel)),
            longitudinal_sample_tolerance_mm_s=_diagnostic_number(longitudinal_limit),
            perpendicular_median_deviation_mm_s=_diagnostic_number(statistics.median(perpendicular)),
            aligned_cruise_fraction=sum(aligned) / size,
            in_speed_band_fraction=sum(error <= longitudinal_limit for error in parallel) / size,
            stable_cruise_fraction=stable_fraction,
            minimum_stable_cruise_fraction=MIN_STABLE_CRUISE_FRACTION,
            sign_agreement_fraction=sum(row[source] * vector[source] > 0 for row in cruise_vectors) / size,
            nonpositive_projection_fraction=sum(value <= 0 for value in projections) / size,
            near_zero_projection_fraction=sum(abs(value) <= near_zero for value in projections) / size,
            near_zero_projection_limit_mm_s=_diagnostic_number(near_zero),
            cruise_pooling_method="signed_time_mean",
            velocity_estimate_method="median_signed_time_means",
            cruise_window_limit_s=CRUISE_WINDOW_S,
            cruise_window_count=len(windows),
            minimum_cruise_window_s=_diagnostic_number(min(duration for duration, _ in windows)),
            maximum_cruise_window_s=_diagnostic_number(max(duration for duration, _ in windows)),
            pooled_cruise_duration_s=_diagnostic_number(cruise_duration),
            raw_coherent_cruise_time_fraction=_diagnostic_number(raw_coherent_fraction),
            pooled_stable_cruise_fraction=_diagnostic_number(pooled_stable_fraction),
            minimum_pooled_projection_mm_s=_diagnostic_number(min(pooled_projections)),
            minimum_local_motion_mm_s=_diagnostic_number(minimum_motion),
            local_motion_supported=local_motion)
    # High-frequency longitudinal ripple may cross the sample speed band while
    # its signed time means consistently show motion. Raw angular/sign evidence
    # must still agree: averaging cannot invent an axis from opposing vectors.
    # The local floor catches a real stop/reversal even on a stroke so long that
    # it would occupy less than the allowed 5% of total cruise time.
    if (raw_coherent_fraction < MIN_STABLE_CRUISE_FRACTION
            or pooled_stable_fraction < MIN_STABLE_CRUISE_FRACTION or not local_motion):
        raise ValueError("Mapping cruise response is unstable (direction %+d, channel %s: "
                         "%.1f%% raw signed-channel support, %.1f%% stable cruise time in "
                         "<=%.0f ms windows, need %.0f%%; minimum forward speed %.3g, "
                         "need %.3g mm/s)" % (
                             item["direction"], "XYZ"[source], 100 * raw_coherent_fraction,
                             100 * pooled_stable_fraction, 1000 * CRUISE_WINDOW_S,
                             100 * MIN_STABLE_CRUISE_FRACTION, min(pooled_projections), minimum_motion))
    return {"direction": item["direction"], "source_axis": source,
            "sign": sign, "alignment": alignment,
            "velocity_mm_s": list(vector), "expected_speed_mm_s": speed,
            "velocity_ratio": norm / speed, "cruise_samples": len(cruise_vectors),
            "start_time_s": start, "end_time_s": end,
            "sample_rate_hz": 1.0 / interval}


def infer_axis_mapping(axis, passes, diagnostics=None):
    """Identify an unambiguous signed channel from both commanded directions.

    An optional output dictionary receives bounded, detached numeric summaries,
    including the first failed pass. Raw samples and unprocessed passes are not
    retained. Diagnostics do not change the mapping or its acceptance criteria.
    """
    if diagnostics is not None:
        if not isinstance(diagnostics, dict):
            raise TypeError("Mapping diagnostics must be a dictionary")
        diagnostics.clear()
        diagnostics.update(version=3, axis=axis if axis in ("x", "y") else None,
                           status="evaluating", passes=[], evaluated_pass_count=0,
                           requested_pass_count=len(passes) if isinstance(passes, (list, tuple)) else 0,
                           unevaluated_pass_count=len(passes) if isinstance(passes, (list, tuple)) else 0,
                           omitted_pass_report_count=0, rejected_pass_index=None)
    current = None
    try:
        if axis not in ("x", "y") or not isinstance(passes, (list, tuple)) or len(passes) < 2:
            raise ValueError("Axis mapping requires X or Y and bidirectional captures")
        reports = []
        for index, item in enumerate(passes):
            current = None
            if diagnostics is not None:
                current = {"pass_index": index, "status": "evaluating"}
                if index < MAX_DIAGNOSTIC_PASSES:
                    diagnostics["passes"].append(current)
                else:
                    diagnostics["omitted_pass_report_count"] += 1
                diagnostics["evaluated_pass_count"] = index + 1
                diagnostics["unevaluated_pass_count"] = len(passes) - index - 1
            reports.append(_pass_velocity(axis, item, current))
            if current is not None:
                current["status"] = "passed"
            current = None
        if {r["direction"] for r in reports} != {-1, 1}:
            raise ValueError("Axis mapping needs both positive and negative movement")
        first = reports[0]
        if any((r["source_axis"], r["sign"]) != (first["source_axis"], first["sign"]) for r in reports):
            raise ValueError("Mapping disagrees between movement directions")
        units = []
        for report in reports:
            vector = report["velocity_mm_s"]
            norm = math.sqrt(math.fsum(v * v for v in vector))
            units.append(tuple(v / norm for v in vector))
        agreement = min(math.fsum(a * b for a, b in zip(left, right))
                        for i, left in enumerate(units) for right in units[i + 1:])
        if diagnostics is not None:
            diagnostics.update(direction_agreement=_diagnostic_number(agreement),
                               direction_agreement_limit=MIN_DIRECTION_AGREEMENT)
        if agreement < MIN_DIRECTION_AGREEMENT:
            raise ValueError("Mapping orientation is inconsistent between passes")
        result = validate_axis_mapping({"version": 1, "axis": axis,
            "source_axis": first["source_axis"], "sign": first["sign"],
            "coordinate_frame": FRAME, "method": METHOD,
            "confidence": min(r["alignment"] for r in reports),
            "direction_agreement": min(1.0, agreement), "passes": reports}, axis)
        if diagnostics is not None:
            diagnostics["status"] = "accepted"
        return result
    except Exception as exc:
        if diagnostics is not None:
            diagnostics.update(status="rejected", reason=str(exc)[:512])
            if current is not None:
                current.update(status="rejected", reason=str(exc)[:512])
                diagnostics["rejected_pass_index"] = current["pass_index"]
                if current["pass_index"] >= MAX_DIAGNOSTIC_PASSES:
                    # Preserve the actual failure even when earlier reports
                    # have filled the bounded output budget.
                    diagnostics["passes"][-1] = current
        raise
