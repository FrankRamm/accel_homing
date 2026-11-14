# Runtime-only experiment models. Part of the GPLv3 homing extension.
"""Deterministic, bounded XYZ fitting against software reference events.

The six training events choose the candidate. The following four events can
only accept or reject it: they never tune it. Reference onsets are detector
events, not independently measured contact, and no model is persisted here.
"""
import itertools
import math

VERSION = 1
CONFIRMATION_S = .001
COMPATIBILITY_S = .001
NEGATIVE_GUARD_S = .020
STARTUP_S = .100


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_profile(profile, axis, raw_threshold_g):
    """Return an independent, JSON-safe snapshot or reject before arming."""
    if not isinstance(profile, dict) or profile.get("version") != VERSION:
        raise ValueError("unsupported experimental profile")
    weights = profile.get("weights_xyz")
    if (profile.get("axis") != axis or axis not in ("x", "y")
            or not isinstance(weights, (tuple, list)) or len(weights) != 3
            or not all(_number(w) and .25 <= w <= 2.5 for w in weights)
            or abs(sum(weights) - 3.) > 1e-9):
        raise ValueError("experimental profile has invalid axis or bounded XYZ weights")
    threshold = profile.get("threshold_g")
    raw = profile.get("raw_threshold_g")
    window = profile.get("confirmation_s")
    age = profile.get("max_age_s")
    if (not _number(threshold) or not _number(raw) or not 0 < threshold < raw
            or not _number(raw_threshold_g)
            or abs(raw - raw_threshold_g) > max(1e-9, raw * 1e-9)
            or not _number(window) or not 0 < window <= CONFIRMATION_S
            or not _number(age) or not .001 <= age <= .5):
        raise ValueError("experimental profile has invalid threshold, freshness or confirmation")
    return {"version": VERSION, "axis": axis, "weights_xyz": list(weights),
            "threshold_g": threshold, "raw_threshold_g": raw,
            "confirmation_s": window, "max_age_s": age,
            "physical_contact_verified": False,
            "reference_kind": "software_raw_xyz_first_crossing"}


def _event(capture, record, axis, checkpoint=None):
    if not isinstance(capture, dict) or not isinstance(record, dict):
        raise ValueError("capture and record must be objects")
    detector = capture.get("detector")
    if (capture.get("fault") or capture.get("dropped", 0)
            or record.get("result") != "impact" or record.get("axis") != axis
            or not isinstance(detector, dict) or detector.get("axis") != axis
            or detector.get("impact_metric") != "xyz_magnitude"
            or detector.get("experimental") is not None
            or detector.get("attempt_count") != 1
            or detector.get("onset_sample_matches") != 1):
        raise ValueError("reference needs one complete, unclipped raw XYZ impact capture")
    baseline, samples = capture.get("baseline_g"), capture.get("samples")
    index = detector.get("onset_sample_index")
    arm = detector.get("capture_arm_time_s", detector.get("arm_time_s"))
    threshold = detector.get("threshold_g")
    if (not isinstance(baseline, (tuple, list)) or len(baseline) != 3
            or not all(_number(v) for v in baseline)
            or not isinstance(samples, (tuple, list)) or not 2 <= len(samples) <= 200000
            or type(index) is not int or not 0 <= index < len(samples)
            or not _number(arm) or not _number(threshold) or threshold <= 0
            or not isinstance(detector.get("onset_sample"), (tuple, list))
            or tuple(record.get("onset_sample") or ()) != tuple(detector["onset_sample"])):
        raise ValueError("reference lacks exact sample provenance or a finite baseline")
    rows, previous = [], None
    # The experiment stops at the reference raw crossing at the latest.
    # Post-stop recoil, saturation and transport cadence are not fit inputs.
    for number, row in enumerate(samples[:index + 1]):
        if checkpoint is not None and number % 4096 == 0:
            checkpoint()
        if (not isinstance(row, (tuple, list)) or len(row) != 4
                or not all(_number(v) for v in row)
                or (previous is not None and row[0] <= previous)):
            raise ValueError("invalid or unordered reference samples")
        previous = row[0]
        xyz = tuple(row[n + 1] - baseline[n] for n in range(3))
        if not all(math.isfinite(v * v) for v in xyz):
            raise ValueError("reference energy overflow")
        rows.append((row[0], xyz[0], xyz[1], xyz[2]))
    onset = rows[index][0]
    if detector.get("onset_time_s") != onset or onset < arm:
        raise ValueError("reference onset time differs from its retained sample")
    if "first_eligible_sample_index" in detector:
        first = detector["first_eligible_sample_index"]
        envelope = detector.get("experimental_envelope", {})
        if (type(first) is not int or not 0 <= first <= index
                or detector.get("first_eligible_sample_matches") != 1
                or not isinstance(detector.get("first_eligible_sample"), (tuple, list))
                or tuple(detector["first_eligible_sample"]) != tuple(envelope.get("first_eligible_sample") or ())):
            raise ValueError("reference first eligible row lacks exact original-packet provenance")
        eligible = list(range(first, index + 1))
    else:
        eligible = [n for n, row in enumerate(rows) if row[0] >= arm]
    reference_quality = capture.get("reference_signal_quality", detector.get("reference_signal_quality"))
    limited = lambda q: bool(q.get("amplitude_limited") or q.get("clipped_axis_values", 0)
                            or q.get("range_limit_axis_values", 0))
    if isinstance(reference_quality, dict) and limited(reference_quality):
        raise ValueError("stationary reference amplitude is limited")
    prefix = detector.get("reference_prefix_quality")
    if prefix is not None:
        if (not isinstance(prefix, dict) or prefix.get("version") != 1
                or prefix.get("available") is not True
                or prefix.get("source") != "original_sensor_values_before_native_clip_decoding"
                or prefix.get("signal_scope") != "armed_prefix_through_reference_decision"
                or prefix.get("sample_count") != len(eligible)
                or prefix.get("first_sample_index") != eligible[0]
                or prefix.get("last_sample_index") != index
                or tuple(prefix.get("onset_sample") or ()) != tuple(detector["onset_sample"])
                or limited(prefix)):
            raise ValueError("armed prefix has limited amplitude or invalid original-code provenance")
    if limited(capture) and (prefix is None or not isinstance(reference_quality, dict)
                             or reference_quality.get("amplitude_limited") is not False):
        raise ValueError("capture amplitude is limited without proven clean reference and armed prefix")
    crossings = [n for n in eligible if sum(v * v for v in rows[n][1:]) >= threshold * threshold]
    if not crossings or crossings[0] != index:
        raise ValueError("reference is not the first raw XYZ threshold crossing")
    negative = [n for n in eligible if rows[n][0] < onset - NEGATIVE_GUARD_S]
    startup = [n for n in negative if rows[n][0] < arm + STARTUP_S]
    cruise = [n for n in negative if rows[n][0] >= arm + STARTUP_S]
    if not startup or not cruise or rows[eligible[0]][0] - arm > .001:
        raise ValueError("reference must include both startup and cruise negatives")
    gaps = [rows[b][0] - rows[a][0] for a, b in zip(eligible, eligible[1:])]
    typical = sorted(gaps)[len(gaps) // 2]
    if typical > CONFIRMATION_S + 1e-12 or max(gaps) > typical * 1.75 + 1e-9:
        raise ValueError("reference cadence cannot support continuous <=1ms evidence")
    return {"rows": rows, "eligible": eligible, "onset": onset,
            "onset_index": index, "arm": arm, "threshold": threshold,
            "negative": negative, "startup": startup, "cruise": cruise}


def replay_profile(profile, event, checkpoint=None, weighted2=None):
    """Offline counterpart of the scalar runtime rule; decision is never backdated."""
    weights, threshold2 = profile["weights_xyz"], profile["threshold_g"] ** 2
    raw2, pending = profile["raw_threshold_g"] ** 2, None
    for index in event["eligible"]:
        if checkpoint is not None and index % 4096 == 0:
            checkpoint()
        when, x, y, z = event["rows"][index]
        x2, y2, z2 = x * x, y * y, z * z
        if x2 + y2 + z2 >= raw2:
            route, evidence = "raw_bypass", index
        elif (weighted2[index] if weighted2 is not None else
              weights[0] * x2 + weights[1] * y2 + weights[2] * z2) >= threshold2:
            if pending is None or when - event["rows"][pending][0] > profile["confirmation_s"] + 1e-12:
                pending = index
                continue
            route, evidence = "weighted_confirmed", pending
        else:
            pending = None
            continue
        return {"route": route, "sample_index": index,
                "evidence_sample_index": evidence, "decision_time_s": when,
                "evidence_time_s": event["rows"][evidence][0],
                "reference_time_s": event["onset"],
                "reference_delta_ms": (when - event["onset"]) * 1000.}
    return None


def _compatible(result):
    return result is not None and abs(result["reference_delta_ms"]) <= COMPATIBILITY_S * 1000. + 1e-9


def _replay_thresholds(event, levels, thresholds, checkpoint):
    """Scan once for all ascending thresholds, preserving raw-first ordering."""
    rows, onset = event["rows"], event["onset_index"]
    remaining, previous, results = 0, None, []
    for index in event["eligible"]:
        if checkpoint is not None and index % 4096 == 0:
            checkpoint()
        if index == onset:
            break
        if previous is not None and 0 < rows[index][0] - rows[previous][0] <= CONFIRMATION_S + 1e-12:
            pair2 = min(levels[previous], levels[index])
            while remaining < len(thresholds) and pair2 >= thresholds[remaining] ** 2:
                results.append({"route": "weighted_confirmed", "sample_index": index,
                    "evidence_sample_index": previous, "decision_time_s": rows[index][0],
                    "evidence_time_s": rows[previous][0], "reference_time_s": event["onset"],
                    "reference_delta_ms": (rows[index][0] - event["onset"]) * 1000.})
                remaining += 1
            if remaining == len(thresholds):
                break
        previous = index
    while len(results) < len(thresholds):
        results.append({"route": "raw_bypass", "sample_index": onset,
            "evidence_sample_index": onset, "decision_time_s": event["onset"],
            "evidence_time_s": event["onset"], "reference_time_s": event["onset"],
            "reference_delta_ms": 0.})
    return results


def fit_experimental_profile(captures, records, *, axis=None, max_age_s=.040, checkpoint=None):
    """Fit six references, then validate exactly four untouched references.

    Negatives end 20ms before the software event to avoid labelling the rising
    impact as motion noise. Replay still examines EVERY post-arm sample, with
    a strict +/-1ms software-event compatibility bound on training and holdout.
    """
    report = {"accepted": False, "profile": None, "reason": None,
              "train_count": 6, "holdout_count": 4, "training": [], "holdout": [],
              "reference_kind": "software_raw_xyz_first_crossing",
              "physical_contact_verified": False, "negative_guard_s": NEGATIVE_GUARD_S,
              "compatibility_tolerance_s": COMPATIBILITY_S, "candidate_count": 0}
    try:
        if len(captures) != 10 or len(records) != 10:
            raise ValueError("exactly ten reference homes are required (six train, four holdout)")
        axis = records[0].get("axis") if axis is None else axis
        # Do not inspect heldout samples until the complete candidate is frozen.
        training = [_event(captures[n], records[n], axis, checkpoint) for n in range(6)]
        raw = training[0]["threshold"]
        if any(event["threshold"] != raw for event in training):
            raise ValueError("reference raw thresholds differ")
        report["negative_samples"] = {"startup": sum(len(e["startup"]) for e in training),
                                      "cruise": sum(len(e["cruise"]) for e in training)}
        best = None
        seen = set()
        for unscaled in itertools.product((.5, 1., 2.), repeat=3):
            weights = tuple(3. * v / sum(unscaled) for v in unscaled)
            if weights in seen:
                continue
            seen.add(weights)
            weighted = []
            wx, wy, wz = weights
            for event in training:
                levels = []
                for number, (_t, x, y, z) in enumerate(event["rows"]):
                    if checkpoint is not None and number % 4096 == 0:
                        checkpoint()
                    levels.append(wx * x * x + wy * y * y + wz * z * z)
                weighted.append(levels)
            negative_peak = math.sqrt(max(levels[n] for event, levels in zip(training, weighted)
                                          for n in event["negative"]))
            lower = max(negative_peak * 1.05, .01)
            # Fixed threshold grid, trained only on the first six homes.
            if lower > raw * .95:
                continue
            thresholds = [lower + (raw * .95 - lower) * step / 16. for step in range(17)]
            by_event = [_replay_thresholds(event, levels, thresholds, checkpoint)
                        for event, levels in zip(training, weighted)]
            for number, threshold in enumerate(thresholds):
                profile = validate_profile({"version": VERSION, "axis": axis,
                    "weights_xyz": weights, "threshold_g": threshold,
                    "raw_threshold_g": raw, "confirmation_s": CONFIRMATION_S,
                    "max_age_s": max_age_s}, axis, raw)
                outcomes = [outcomes[number] for outcomes in by_event]
                report["candidate_count"] += 1
                weighted_count = sum(o is not None and o["route"] == "weighted_confirmed" for o in outcomes)
                if weighted_count < 3 or len(outcomes) != 6 or not all(_compatible(o) for o in outcomes):
                    continue
                # Prefer measured sample-time improvement, then stronger noise
                # separation, then a fixed lexicographic weight tie breaker.
                rank = (sum(o["reference_delta_ms"] for o in outcomes),
                        -threshold / max(negative_peak, .01), weights, -threshold)
                if best is None or rank < best[0]:
                    best = rank, profile, outcomes
        if best is None:
            raise ValueError("training impacts and motion negatives are inseparable within the compatibility bound")
        _, profile, report["training"] = best
        report["frozen_candidate"] = profile
        holdout = [_event(captures[n], records[n], axis, checkpoint) for n in range(6, 10)]
        if any(e["threshold"] != raw for e in holdout):
            raise ValueError("heldout raw threshold differs from frozen candidate")
        report["holdout"] = [replay_profile(profile, event, checkpoint) for event in holdout]
        if not all(_compatible(o) for o in report["holdout"]):
            raise ValueError("frozen candidate failed heldout +/-1ms compatibility")
        if not any(o["route"] == "weighted_confirmed" for o in report["holdout"]):
            raise ValueError("frozen candidate has no heldout weighted detections")
        report.update(accepted=True, profile=profile, reason="frozen candidate passed software-event compatibility")
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        report["reason"] = str(exc)
    return report
