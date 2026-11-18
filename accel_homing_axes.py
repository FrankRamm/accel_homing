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
GRAVITY_MM_S2 = 9806.65
MIN_ALIGNMENT = 0.85
MIN_DIRECTION_AGREEMENT = 0.95
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
    if mapping.get("coordinate_frame") != FRAME or mapping.get("method") != METHOD:
        raise ValueError("Unsupported axis mapping coordinate frame or method")
    if "confidence" in mapping and (not _number(mapping["confidence"])
                                    or not MIN_ALIGNMENT <= mapping["confidence"] <= 1):
        raise ValueError("Axis mapping is ambiguous or has invalid confidence")
    return copy.deepcopy(mapping)


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
    cruise_vectors, expected_speeds = [], []
    for segment in motion:
        if segment["phase"] != "cruise":
            continue
        speed = segment["start_velocity_mm_s"]
        if speed <= 0 or segment["acceleration_mm_s2"] != 0:
            raise ValueError("Mapping cruise speed must be finite and positive")
        trim = 0.1 * (segment["end_time_s"] - segment["start_time_s"])
        left, right = segment["start_time_s"] + trim, segment["end_time_s"] - trim
        for time, vector in integrated:
            if left <= time <= right:
                fraction = (time - start) / (end - start)
                cruise_vectors.append(tuple((v - d * fraction) * item["direction"]
                                            for v, d in zip(vector, drift)))
                expected_speeds.append(speed)
                if diagnostics is not None:
                    diagnostics.setdefault("cruise_start_time_s", time)
                    diagnostics["cruise_end_time_s"] = time
    if diagnostics is not None:
        diagnostics["cruise_samples"] = len(cruise_vectors)
    if len(cruise_vectors) < 10:
        raise ValueError("Mapping capture has insufficient full-speed cruise samples")
    vector = tuple(statistics.median(row[i] for row in cruise_vectors) for i in range(3))
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
    # A median alone must not hide unstable velocity magnitude or direction.
    deviations = [math.sqrt(math.fsum((row[i] - vector[i]) ** 2 for i in range(3)))
                  for row in cruise_vectors]
    deviation = statistics.median(deviations)
    limit = 0.20 * norm
    if diagnostics is not None:
        # Diagnostic decomposition only. Acceptance continues to use the
        # original total Euclidean deviation, including longitudinal ripple.
        unit = tuple(value / norm for value in vector)
        projections = [math.fsum(value * direction for value, direction in zip(row, unit))
                       for row in cruise_vectors]
        parallel = [abs(value - norm) for value in projections]
        perpendicular = [math.hypot(*(value - projection * direction
                                      for value, direction in zip(row, unit)))
                         for row, projection in zip(cruise_vectors, projections)]
        size = len(cruise_vectors)
        near_zero = 0.10 * norm
        diagnostics.update(
            total_median_deviation_mm_s=_diagnostic_number(deviation),
            total_deviation_limit_mm_s=_diagnostic_number(limit),
            total_deviation_ratio=_diagnostic_number(deviation / norm),
            total_deviation_limit_ratio=0.20,
            total_deviation_to_limit_ratio=_diagnostic_number(deviation / limit),
            parallel_median_deviation_mm_s=_diagnostic_number(statistics.median(parallel)),
            perpendicular_median_deviation_mm_s=_diagnostic_number(statistics.median(perpendicular)),
            sign_agreement_fraction=sum(row[source] * vector[source] > 0 for row in cruise_vectors) / size,
            nonpositive_projection_fraction=sum(value <= 0 for value in projections) / size,
            near_zero_projection_fraction=sum(abs(value) <= near_zero for value in projections) / size,
            near_zero_projection_limit_mm_s=_diagnostic_number(near_zero))
    if deviation > limit:
        raise ValueError("Mapping cruise response is unstable (direction %+d, channel %s: "
                         "median deviation %.6g mm/s exceeds limit %.6g mm/s)" % (
                             item["direction"], "XYZ"[source], deviation, limit))
    return {"direction": item["direction"], "source_axis": source,
            "sign": 1 if vector[source] > 0 else -1, "alignment": alignment,
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
        diagnostics.update(version=1, axis=axis if axis in ("x", "y") else None,
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
