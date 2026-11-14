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
MAX_CANDIDATE_DIAGNOSTICS = 12
_CANDIDATE_CATEGORIES = (
    "compatible_enough_weighted", "compatible_insufficient_weighted",
    "incompatible_enough_weighted", "incompatible_insufficient_weighted")
_REFERENCE_ERRORS = (ValueError, TypeError, KeyError, IndexError, OverflowError)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_profile(profile, axis, raw_threshold_g):
    """Return an independent, JSON-safe snapshot or reject before arming."""
    if not isinstance(profile, dict) or profile.get("version") != VERSION:
        raise ValueError("unsupported experimental profile")
    if (profile.get("shadow_only") or profile.get("live_use_authorized") is False
            or "noise_margin_policy" in profile):
        raise ValueError("shadow-only profile is not authorized for live use")
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
                or not isinstance(envelope, dict)
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


def _classified(outcome, event):
    """Classify software decisions without claiming that they measure contact."""
    if outcome is None:
        return None
    result = dict(outcome)
    when = result["decision_time_s"]
    before_guard = when < event["onset"] - NEGATIVE_GUARD_S - 1e-12
    if before_guard:
        phase = "startup" if when < event["arm"] + STARTUP_S else "cruise"
    else:
        phase = "reference" if result["sample_index"] == event["onset_index"] else "near_event"
    result.update(phase=phase, compatible=_compatible(result),
                  within_negative_guard=not before_guard,
                  no_later_than_reference=when <= event["onset"] + 1e-12,
                  physical_contact_verified=False, live_use_authorized=False)
    return result


def _noise_margin_policy(fraction, minimum_g):
    if (not _number(fraction) or not 0 < fraction < 1
            or not _number(minimum_g) or minimum_g <= 0):
        raise ValueError("noise margin needs a finite fraction between zero and one and positive g")
    return {"version": 1, "fraction": fraction, "minimum_g": minimum_g,
            "interpretation": "engineering_noise_margin_not_reliability_probability",
            "raw_bypass_assessed": True}


def _profile_margin_policy(profile):
    if "noise_margin_policy" not in profile:
        return None
    policy = profile["noise_margin_policy"]
    if not isinstance(policy, dict):
        raise ValueError("invalid noise margin policy")
    canonical = _noise_margin_policy(policy.get("fraction"), policy.get("minimum_g"))
    if (type(policy.get("version")) is not int or policy.get("raw_bypass_assessed") is not True
            or policy != canonical):
        raise ValueError("invalid noise margin policy")
    return canonical


def _negative_noise(event, weights, confirmation_s, checkpoint, weighted2=None):
    """Observe all negatives, including those after a hypothetical early stop.

    A pair is the minimum of two adjacent original samples within the actual
    confirmation interval. Both are negative samples; the decision sample sets
    its phase, so a startup-to-cruise pair is retained in cruise diagnostics.
    """
    phases = {phase: {"sample_count": 0, "pair_count": 0,
                     "raw_max_g": None, "weighted_isolated_max_g": None,
                     "weighted_pair_max_g": None, "closest_pair": None,
                     "raw_closest_sample": None, "weighted_isolated_sample": None}
              for phase in ("startup", "cruise")}
    previous = None
    for number, index in enumerate(event["negative"]):
        if checkpoint is not None and number % 4096 == 0:
            checkpoint()
        when, x, y, z = event["rows"][index]
        raw = math.sqrt(x * x + y * y + z * z)
        level2 = (weighted2[index] if weighted2 is not None else
                  weights[0] * x * x + weights[1] * y * y + weights[2] * z * z)
        level = math.sqrt(level2)
        if not math.isfinite(raw) or not math.isfinite(level):
            raise ValueError("negative-window energy overflow")
        phase = "startup" if when < event["arm"] + STARTUP_S else "cruise"
        stats = phases[phase]
        stats["sample_count"] += 1
        if stats["raw_max_g"] is None or raw > stats["raw_max_g"]:
            stats["raw_max_g"] = raw
            stats["raw_closest_sample"] = {"sample_index": index, "time_s": when}
        if stats["weighted_isolated_max_g"] is None or level > stats["weighted_isolated_max_g"]:
            stats["weighted_isolated_max_g"] = level
            stats["weighted_isolated_sample"] = {"sample_index": index, "time_s": when}
        if (previous is not None and index == previous[0] + 1
                and 0 < when - previous[1] <= confirmation_s + 1e-12):
            pair = min(previous[2], level)
            stats["pair_count"] += 1
            if stats["weighted_pair_max_g"] is None or pair > stats["weighted_pair_max_g"]:
                stats["weighted_pair_max_g"] = pair
                stats["closest_pair"] = {"evidence_sample_index": previous[0],
                    "decision_sample_index": index, "evidence_time_s": previous[1],
                    "decision_time_s": when, "separation_s": when - previous[1],
                    "evidence_weighted_g": previous[2], "decision_weighted_g": level}
        previous = index, when, level
    return phases


def _assess_noise_margin(profile, phases, policy):
    """Apply the frozen engineering policy to weighted and raw bypass routes."""
    result = {"qualified": False, "reason_codes": [], "policy": dict(policy),
              "physical_contact_verified": False, "live_use_authorized": False,
              "negative_guard_s": NEGATIVE_GUARD_S, "confirmation_s": profile["confirmation_s"]}
    threshold, raw = profile["threshold_g"], profile["raw_threshold_g"]
    required = max(policy["minimum_g"], policy["fraction"] * threshold)
    raw_required = max(policy["minimum_g"], policy["fraction"] * raw)
    for phase in ("startup", "cruise"):
        stats = result[phase] = dict(phases[phase])
        peak, raw_peak = stats["weighted_pair_max_g"], stats["raw_max_g"]
        margin = None if peak is None else threshold - peak
        raw_margin = None if raw_peak is None else raw - raw_peak
        weighted_ok = margin is not None and margin + 1e-12 >= required
        raw_ok = raw_margin is not None and raw_margin + 1e-12 >= raw_required
        stats.update(weighted_margin_g=margin,
                     weighted_margin_fraction=None if margin is None else margin / threshold,
                     required_weighted_margin_g=required, weighted_qualified=weighted_ok,
                     raw_bypass_margin_g=raw_margin,
                     raw_bypass_margin_fraction=None if raw_margin is None else raw_margin / raw,
                     required_raw_bypass_margin_g=raw_required, raw_bypass_qualified=raw_ok,
                     qualified=weighted_ok and raw_ok)
        if peak is None:
            result["reason_codes"].append(phase + "_missing_confirmable_negative_pair")
        elif not weighted_ok:
            result["reason_codes"].append(phase + "_insufficient_weighted_margin")
        if not raw_ok:
            result["reason_codes"].append(phase + "_insufficient_raw_bypass_margin")
    result["qualified"] = not result["reason_codes"]
    return result


def _validated_shadow(profile, capture, record, axis, checkpoint):
    if (not isinstance(profile, dict) or profile.get("shadow_only") is not True
            or profile.get("live_use_authorized") is not False):
        raise ValueError("replay requires an explicitly shadow-only profile")
    policy = _profile_margin_policy(profile)
    axis = profile.get("axis") if axis is None else axis
    event = _event(capture, record, axis, checkpoint)
    parameters = {key: value for key, value in profile.items()
                  if key not in ("shadow_only", "live_use_authorized", "noise_margin_policy")}
    return validate_profile(parameters, axis, event["threshold"]), event, policy


def assess_shadow_margin(profile, capture, record, *, axis=None, checkpoint=None):
    """Assess a frozen shadow policy without retuning or permitting live use."""
    try:
        validated, event, policy = _validated_shadow(profile, capture, record, axis, checkpoint)
        if policy is None:
            raise ValueError("shadow profile has no frozen noise margin policy")
        phases = _negative_noise(event, validated["weights_xyz"], validated["confirmation_s"], checkpoint)
        return _assess_noise_margin(validated, phases, policy)
    except _REFERENCE_ERRORS as exc:
        raise ValueError(str(exc)) from exc


def replay_shadow_capture(profile, capture, record, *, axis=None, checkpoint=None):
    """Validate one captured RAW home, then replay a frozen shadow-only profile.

    The caller must finish capture before calling this helper. Invalid data
    raises ValueError; an early negative-window decision is returned as evidence
    with within_negative_guard=False. Nothing is armed or installed here.
    """
    try:
        validated, event, policy = _validated_shadow(profile, capture, record, axis, checkpoint)
        result = _classified(replay_profile(validated, event, checkpoint), event)
        if policy is not None:
            phases = _negative_noise(event, validated["weights_xyz"], validated["confirmation_s"], checkpoint)
            result["noise_margin"] = _assess_noise_margin(validated, phases, policy)
        return result
    except _REFERENCE_ERRORS as exc:
        raise ValueError(str(exc)) from exc


def _diagnose_candidate(report, profile, outcomes, training):
    """Count every candidate; retain at most three examples per category."""
    classified = [_classified(o, e) for o, e in zip(outcomes, training)]
    weighted = sum(o is not None and o["route"] == "weighted_confirmed" for o in outcomes)
    compatible = all(_compatible(o) for o in outcomes)
    category = (("compatible" if compatible else "incompatible")
                + ("_enough_weighted" if weighted >= 3 else "_insufficient_weighted"))
    codes = []
    if weighted < 3:
        codes.append("insufficient_weighted_decisions")
    if not compatible:
        codes.append("outside_live_compatibility")
    for phase in ("startup", "cruise"):
        if any(o is not None and o["phase"] == phase for o in classified):
            codes.append(phase + "_trigger")
    if any(o is not None and o["phase"] == "near_event" and not o["compatible"]
           for o in classified):
        codes.append("near_event_early")
    diagnostics = report["diagnostics"]
    diagnostics["candidate_counts"][category] += 1
    for code in codes:
        diagnostics["rejection_counts"][code] += 1
    for outcome in classified:
        if outcome is not None:
            diagnostics["decision_phase_counts"][outcome["phase"]] += 1
    # Each example carries six deltas, not sensor rows or an unbounded trace.
    examples = diagnostics["candidate_examples"]
    if sum(e["category"] == category for e in examples) < MAX_CANDIDATE_DIAGNOSTICS // 4:
        examples.append({"candidate_index": report["candidate_count"], "category": category,
                         "weights_xyz": list(profile["weights_xyz"]),
                         "threshold_g": profile["threshold_g"], "weighted_decisions": weighted,
                         "reason_codes": codes, "phases": [o["phase"] for o in classified],
                         "reference_delta_ms": [o["reference_delta_ms"] for o in outcomes]})
    return weighted, compatible, classified


def fit_experimental_profile(captures, records, *, axis=None, max_age_s=.040,
                             checkpoint=None, shadow=False, noise_margin_fraction=None,
                             noise_margin_g=None):
    """Fit six references, then assess exactly four untouched references.

    Live acceptance retains its strict +/-1ms bound. Optional shadow fitting
    studies decisions within 20ms before the software event; it never authorizes
    live use or verifies physical contact. Both candidates freeze before any
    holdout data is inspected, and a failed holdout never causes a refit.
    Supplying both noise-margin options selects only a shadow candidate using
    confirmed negative pairs and also qualifies the single-sample raw bypass.
    """
    report = {"accepted": False, "profile": None, "reason": None, "reason_codes": [],
              "train_count": 6, "holdout_count": 4, "training": [], "holdout": [],
              "reference_kind": "software_raw_xyz_first_crossing",
              "physical_contact_verified": False, "negative_guard_s": NEGATIVE_GUARD_S,
              "compatibility_tolerance_s": COMPATIBILITY_S, "candidate_count": 0,
              "diagnostics": {
                  "count_scope": "training_candidates_only",
                  "candidate_counts": dict.fromkeys(_CANDIDATE_CATEGORIES, 0),
                  "rejection_counts": dict.fromkeys(("insufficient_weighted_decisions",
                      "outside_live_compatibility", "startup_trigger", "cruise_trigger",
                      "near_event_early"), 0),
                  "decision_phase_counts": dict.fromkeys(("startup", "cruise", "near_event", "reference"), 0),
                  "candidate_examples": [], "candidate_examples_limit": MAX_CANDIDATE_DIAGNOSTICS,
                  "weight_sets_without_threshold_margin": 0}}
    shadow_report = None
    if shadow:
        shadow_report = report["shadow"] = {
            "status": "rejected", "profile": None, "training": [], "holdout": [],
            "reason": None, "reason_codes": [], "live_use_authorized": False,
            "physical_contact_verified": False, "negative_guard_s": NEGATIVE_GUARD_S,
            "train_count": 6, "holdout_count": 4,
            "selection": "most_weighted_training_homes_then_earliest_sum_then_negative_margin",
            "interpretation": "Exploratory software-event timing only; not authorized for live use. "
                              "Earlier selection may trade sensitivity; independent contact truth is required."}
    stage = "invalid_training_reference"
    try:
        if type(shadow) is not bool:
            raise ValueError("shadow must be a boolean")
        policy = None
        if noise_margin_fraction is not None or noise_margin_g is not None:
            stage = "invalid_noise_margin_policy"
            if not shadow:
                raise ValueError("noise margin qualification requires shadow-only fitting")
            policy = _noise_margin_policy(noise_margin_fraction, noise_margin_g)
            shadow_report["noise_margin_policy"] = policy
            shadow_report["selection"] += "_with_frozen_engineering_noise_margin"
            report["diagnostics"]["noise_margin_rejection_counts"] = {}
        stage = "invalid_training_reference"
        if len(captures) != 10 or len(records) != 10:
            raise ValueError("exactly ten reference homes are required (six train, four holdout)")
        if not isinstance(records[0], dict):
            raise ValueError("reference record must be an object")
        axis = records[0].get("axis") if axis is None else axis
        # No heldout sample is accessed during either selection.
        training = [_event(captures[n], records[n], axis, checkpoint) for n in range(6)]
        raw = training[0]["threshold"]
        if any(event["threshold"] != raw for event in training):
            raise ValueError("reference raw thresholds differ")
        report["negative_samples"] = {"startup": sum(len(e["startup"]) for e in training),
                                      "cruise": sum(len(e["cruise"]) for e in training)}
        best, best_shadow = None, None
        seen = set()
        stage = "invalid_candidate_parameters"
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
            noise = None
            if policy is not None:
                noise = [_negative_noise(event, weights, CONFIRMATION_S, checkpoint, levels)
                         for event, levels in zip(training, weighted)]
                peaks = [phases[phase]["weighted_pair_max_g"] for phases in noise
                         for phase in ("startup", "cruise")]
                if any(peak is None for peak in peaks):
                    counts = report["diagnostics"]["noise_margin_rejection_counts"]
                    code = "missing_confirmable_negative_pair"
                    counts[code] = counts.get(code, 0) + 1
                    continue
                negative_peak = max(peaks)
                lower = max(negative_peak + policy["minimum_g"],
                            negative_peak / (1. - policy["fraction"]), .01)
            # Fixed threshold grid, trained only on the first six homes.
            if lower > raw * .95:
                report["diagnostics"]["weight_sets_without_threshold_margin"] += 1
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
                weighted_count, compatible, classified = _diagnose_candidate(report, profile, outcomes, training)
                margin_ok = True
                if policy is not None:
                    margins = [_assess_noise_margin(profile, phases, policy) for phases in noise]
                    margin_ok = all(margin["qualified"] for margin in margins)
                    counts = report["diagnostics"]["noise_margin_rejection_counts"]
                    margin_codes = sorted({code for margin in margins for code in margin["reason_codes"]})
                    for code in margin_codes:
                        counts[code] = counts.get(code, 0) + 1
                    examples = report["diagnostics"]["candidate_examples"]
                    if examples and examples[-1]["candidate_index"] == report["candidate_count"]:
                        examples[-1].update(noise_margin_qualified=margin_ok,
                                            noise_margin_reason_codes=margin_codes)
                # This live ranking and +/-1ms acceptance are unchanged.
                rank = (sum(o["reference_delta_ms"] for o in outcomes),
                        -threshold / max(negative_peak, .01), weights, -threshold)
                if policy is None and weighted_count >= 3 and compatible and (best is None or rank < best[0]):
                    best = rank, profile, outcomes
                # Prefer repeatable weighted coverage before exploratory early
                # timing. This is not evidence that an earlier signal is contact.
                shadow_rank = (-weighted_count,) + rank
                if (shadow and margin_ok and weighted_count >= 3
                        and all(o["within_negative_guard"] and o["no_later_than_reference"] for o in classified)
                        and (best_shadow is None or shadow_rank < best_shadow[0])):
                    best_shadow = shadow_rank, profile, classified
        if best is None:
            counts = report["diagnostics"]["candidate_counts"]
            report["reason"] = ("no training candidate meets both live +/-1ms compatibility and minimum "
                "weighted coverage: " + ", ".join("%s=%d" % (key, counts[key]) for key in _CANDIDATE_CATEGORIES))
            report["reason_codes"] = [code for code, count in report["diagnostics"]["rejection_counts"].items() if count]
            if not report["candidate_count"]:
                report["reason_codes"] = ["no_candidates_after_negative_margin"]
            if policy is not None:
                report.update(reason="noise-margin fitting is shadow-only; live profile selection disabled",
                              reason_codes=["noise_margin_shadow_only"])
        else:
            _, profile, report["training"] = best
            report["frozen_candidate"] = profile
        if shadow:
            if best_shadow is None:
                shadow_report.update(reason="no training candidate has three weighted decisions within the 20ms guard",
                                     reason_codes=["no_shadow_training_candidate"])
                if policy is not None:
                    shadow_report["reason"] += " and the required weighted/raw noise margin"
                    shadow_report["reason_codes"].append("no_noise_margin_training_candidate")
            else:
                _, shadow_profile, shadow_report["training"] = best_shadow
                shadow_profile = dict(shadow_profile, shadow_only=True, live_use_authorized=False)
                if policy is not None:
                    shadow_profile["noise_margin_policy"] = dict(policy)
                    for event, outcome in zip(training, shadow_report["training"]):
                        phases = _negative_noise(event, shadow_profile["weights_xyz"], CONFIRMATION_S, checkpoint)
                        outcome["noise_margin"] = _assess_noise_margin(shadow_profile, phases, policy)
                shadow_report["profile"] = shadow_profile
        if best is None and best_shadow is None:
            return report

        # Freeze both choices above before validating all four holdouts. Retain
        # valid siblings even if one reference is malformed, without refitting.
        stage = "invalid_holdout_reference"
        holdout, errors = [], []
        for n in range(6, 10):
            try:
                event = _event(captures[n], records[n], axis, checkpoint)
                if event["threshold"] != raw:
                    raise ValueError("heldout raw threshold differs from frozen candidate")
                holdout.append(event)
            except _REFERENCE_ERRORS as exc:
                holdout.append(None)
                errors.append({"reference_index": n, "reason": str(exc), "reason_code": stage})
        if errors:
            report["holdout_errors"] = errors
        if best is not None:
            profile = best[1]
            report["holdout"] = [replay_profile(profile, event, checkpoint) if event is not None else None
                                  for event in holdout]
            if errors:
                report.update(reason="invalid heldout reference", reason_codes=[stage])
            elif not all(_compatible(o) for o in report["holdout"]):
                report.update(reason="frozen candidate failed heldout +/-1ms compatibility",
                              reason_codes=["holdout_outside_live_compatibility"])
            elif not any(o["route"] == "weighted_confirmed" for o in report["holdout"]):
                report.update(reason="frozen candidate has no heldout weighted detections",
                              reason_codes=["holdout_no_weighted_decisions"])
            else:
                report.update(accepted=True, profile=profile, reason_codes=[],
                              reason="frozen candidate passed software-event compatibility")
        if best_shadow is not None:
            shadow_outcomes = shadow_report["holdout"] = [
                _classified(replay_profile(shadow_profile, event, checkpoint), event) if event is not None else None
                for event in holdout]
            if policy is not None:
                for event, outcome in zip(holdout, shadow_outcomes):
                    if event is not None:
                        phases = _negative_noise(event, shadow_profile["weights_xyz"], CONFIRMATION_S, checkpoint)
                        outcome["noise_margin"] = _assess_noise_margin(shadow_profile, phases, policy)
            codes = []
            if errors:
                shadow_report["holdout_errors"] = errors
                codes.append("invalid_holdout_reference")
            for phase in ("startup", "cruise"):
                if any(o is not None and o["phase"] == phase for o in shadow_outcomes):
                    codes.append("holdout_" + phase + "_trigger")
            if any(o is not None and not o["no_later_than_reference"] for o in shadow_outcomes):
                codes.append("holdout_later_than_raw_reference")
            if not any(o is not None and o["route"] == "weighted_confirmed" for o in shadow_outcomes):
                codes.append("holdout_no_weighted_decisions")
            if policy is not None and any(o is not None and not o["noise_margin"]["qualified"]
                                          for o in shadow_outcomes):
                codes.append("holdout_insufficient_noise_margin")
            shadow_report.update(status="rejected" if codes else "ready", reason_codes=codes,
                reason=("frozen shadow candidate rejected: " + ", ".join(codes)) if codes else
                       "frozen shadow candidate passed heldout software-window checks; not authorized for live use")
    except _REFERENCE_ERRORS as exc:
        report.update(reason=str(exc), reason_codes=[stage])
        if shadow_report is not None:
            shadow_report.update(status="rejected", reason=str(exc), reason_codes=[stage])
    return report
