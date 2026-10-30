# Experimental causal accelerometer detector. GPLv3.
"""Bounded signal processing shared by the armed detector and offline replay.

No parameter below is a measured printer optimum. Filtering and confirmation can
miss narrow contacts and add stopping latency. Thresholds belong to this metric
and must never be borrowed implicitly from a raw-magnitude calibration.
"""
import collections
import math
import statistics

DETECTOR_VERSION = 1
MAX_REFERENCE_SAMPLES = 65536
MAX_HISTORY_SAMPLES = 128
MAX_MOVING_SAMPLES = 256
DEFAULT_CONFIG = {
    "version": DETECTOR_VERSION,
    "lowpass_tau_s": 0.0002,
    "confirmation_min_s": 0.0003,
    "confirmation_max_s": 0.0015,
    "excess_energy_s": 0.00015,
    "support_ratio": 0.65,
    "release_ratio": 0.45,
    "onset_ratio": 0.25,
    "onset_lookback_s": 0.00075,
    "weight_floor": 0.75,
    "noise_floor_g": 0.005,
}


def detector_config(config=None):
    """Validate the entire versioned configuration; unknown fields fail closed."""
    result = dict(DEFAULT_CONFIG)
    if config is not None:
        if not isinstance(config, dict) or set(config) - set(result):
            raise ValueError("unknown filtered-detector configuration")
        result.update(config)
    if type(result["version"]) is not int or result["version"] != DETECTOR_VERSION:
        raise ValueError("unsupported filtered-detector version")
    bounds = {"lowpass_tau_s": (0., .001), "confirmation_min_s": (0., .0015),
              "confirmation_max_s": (.0001, .003), "excess_energy_s": (0., .002),
              "support_ratio": (.5, 1.), "release_ratio": (.1, .6),
              "onset_ratio": (.1, .5), "onset_lookback_s": (0., .0015),
              "weight_floor": (.75, 1.), "noise_floor_g": (.001, .1)}
    for key, (low, high) in bounds.items():
        value = result[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError("invalid filtered-detector %s" % key)
    if (result["confirmation_min_s"] > result["confirmation_max_s"]
            or result["release_ratio"] >= result["support_ratio"]):
        raise ValueError("inconsistent filtered-detector confirmation bounds")
    return result


def _xyz(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError("invalid finite XYZ values")
    return tuple(values)


def robust_reference(rows_xyz_g, config=None):
    """Median/MAD reference, learned only before arm and frozen during motion."""
    config = detector_config(config)
    rows = list(rows_xyz_g)
    if not 2 <= len(rows) <= MAX_REFERENCE_SAMPLES:
        raise ValueError("reference sample count outside bound")
    rows = [_xyz(row) for row in rows]
    channels = list(zip(*rows))
    offset = tuple(statistics.median(channel) for channel in channels)
    sigma = tuple(1.4826 * statistics.median(abs(v - center) for v in channel)
                  for channel, center in zip(channels, offset))
    floor = config["noise_floor_g"]
    quiet = max(floor, min(sigma))
    # Every channel retains at least 75% of its contribution. No projection,
    # sign exclusion or zero-weight axis may hide a transverse contact.
    weights = tuple(max(config["weight_floor"], min(1., quiet / max(floor, s))) for s in sigma)
    return {"method": "stationary_median_mad_v1", "sample_count": len(rows),
            "offset_g": list(offset), "noise_sigma_g": list(sigma),
            "weights_xyz": list(weights), "learning_policy": "frozen_before_arm",
            "threshold_adaptation": False}


class CausalImpactDetector:
    """O(1) processing/sample and bounded history; no learning during contact."""
    def __init__(self, threshold_g, offset_g, noise_sigma_g=(0., 0., 0.),
                 weights_xyz=(1., 1., 1.), config=None):
        self.config = detector_config(config)
        if type(threshold_g) not in (int, float) or not math.isfinite(threshold_g) or threshold_g <= 0.:
            raise ValueError("filtered threshold must be finite and positive")
        self.threshold_g = threshold_g
        self.offset_g = _xyz(offset_g)
        self.noise_sigma_g = _xyz(noise_sigma_g)
        self.weights_xyz = _xyz(weights_xyz)
        if any(s < 0 for s in self.noise_sigma_g) or any(
                not self.config["weight_floor"] <= w <= 1. for w in self.weights_xyz):
            raise ValueError("invalid frozen noise/weight model")
        self.previous_time = None
        self.filtered_xyz = (0., 0., 0.)
        self.history = collections.deque(maxlen=MAX_HISTORY_SAMPLES)
        self.moving = []
        self.moving_frozen = False
        self.candidate = self.decision = None
        self.candidates = self.rejected_candidates = 0
        self.raw_level_g = self.level_g = self.magnitude_g = 0.

    def feed(self, when, xyz_g, identity=None):
        if type(when) not in (int, float) or not math.isfinite(when):
            raise ValueError("invalid filtered sample timestamp")
        xyz = _xyz(xyz_g)
        previous = self.previous_time
        dt = None if previous is None else when - previous
        if dt is not None and (dt <= 0 or dt > .00625):
            raise ValueError("noncontiguous filtered sample timestamps")
        self.previous_time = when
        delta = tuple(v - b for v, b in zip(xyz, self.offset_g))
        tau = self.config["lowpass_tau_s"]
        # The first row is an actual armed sample; initialize from the frozen
        # resting state rather than treating its acceleration as a new offset.
        if dt is None:
            # Standalone callers may omit a preceding epoch. An explicit first
            # row initializes the filter directly; production/replay always
            # seed previous_time with the validated sampling period instead.
            period = 0.
        else:
            period = dt
        alpha = 1. if tau == 0 or dt is None else period / (tau + period)
        self.filtered_xyz = tuple(old + alpha * (v - old)
                                  for old, v in zip(self.filtered_xyz, delta))
        self.raw_level_g = math.sqrt(sum((v * w) ** 2 for v, w in zip(delta, self.weights_xyz)))
        self.magnitude_g = math.sqrt(sum(v*v for v in self.filtered_xyz))
        self.level_g = math.sqrt(sum((v*w)**2 for v, w in zip(self.filtered_xyz, self.weights_xyz)))
        row = (when, self.level_g, self.raw_level_g, identity)
        self.history.append(row)
        cutoff = when - self.config["onset_lookback_s"]
        while len(self.history) > 1 and self.history[0][0] < cutoff:
            self.history.popleft()
        if self.decision is not None:
            return self.decision
        threshold = self.threshold_g
        if self.candidate is None and self.level_g >= threshold:
            self.candidates += 1
            self.moving_frozen = True
            gate = max(self.config["onset_ratio"] * threshold,
                       3. * math.sqrt(sum((s*w)**2 for s, w in zip(self.noise_sigma_g, self.weights_xyz))))
            onset = row
            # Search only the final contiguous rising region, at most .75 ms.
            for old in reversed(self.history):
                if old[1] < gate:
                    break
                onset = old
            self.candidate = {"crossing_time_s": when, "crossing_identity": identity,
                              "onset_time_s": onset[0], "onset_identity": onset[3],
                              "onset_level_g": onset[1], "energy": 0., "support_samples": 0,
                              "peak_level_g": self.level_g}
        candidate = self.candidate
        if candidate is not None:
            elapsed = when - candidate["crossing_time_s"]
            if elapsed > self.config["confirmation_max_s"] + 1e-12:
                self.rejected_candidates += 1
                self.candidate = None
            else:
                candidate["peak_level_g"] = max(candidate["peak_level_g"], self.level_g)
                if self.raw_level_g >= self.config["support_ratio"] * threshold:
                    candidate["support_samples"] += 1
                # Capping each contribution prevents one giant row from paying
                # for the entire event. Two distinct raw support rows are still
                # mandatory even if the causal filter rings after a spike.
                contribution = min(3., max(0., (self.level_g / threshold) ** 2 - 1.))
                candidate["energy"] += contribution * min(period, .00025)
                if (elapsed + 1e-12 >= self.config["confirmation_min_s"]
                        and candidate["support_samples"] >= 2
                        and candidate["energy"] + 1e-12 >= self.config["excess_energy_s"]
                        and self.level_g >= self.config["release_ratio"] * threshold):
                    self.decision = {key: value for key, value in candidate.items() if key != "energy"}
                    self.decision.update(confirmation_time_s=when, confirmation_identity=identity,
                                         excess_energy_s=candidate["energy"],
                                         confirmation_delay_s=elapsed,
                                         threshold_g=threshold, level_g=self.level_g)
                    return self.decision
                if self.level_g < self.config["release_ratio"] * threshold:
                    self.rejected_candidates += 1
                    self.candidate = None
        # Moving-noise characterization is deliberately observation only. It
        # never updates the frozen offset, weights, or threshold. The first
        # candidate permanently freezes even this bounded diagnostic history.
        if (not self.moving_frozen and len(self.moving) < MAX_MOVING_SAMPLES
                and self.raw_level_g < self.config["support_ratio"] * threshold):
            self.moving.append(delta)
        return None

    def diagnostics(self):
        moving = None
        if len(self.moving) >= 2:
            moving = robust_reference(self.moving, self.config)
            moving.update(method="moving_low_level_median_mad_observation_only",
                          accepted_below_g=self.config["support_ratio"] * self.threshold_g,
                          censored=True, used_for_detection=False)
        return {"candidate_count": self.candidates, "rejected_candidates": self.rejected_candidates,
                "moving_noise": moving, "moving_learning_frozen": self.moving_frozen,
                "learning_policy": "frozen_before_arm", "threshold_adaptation": False,
                "history_samples": len(self.history), "moving_samples": len(self.moving)}


def compare_filter_capture(capture, raw_threshold_g, filtered_threshold_g, filter_config=None):
    """Counterfactual paired replay, censored before the actual stop request.

    Complete traces remain available for display. Rows after the conservative
    dispatch lower bound cannot establish a shadow detection or improvement.
    """
    if (not isinstance(capture, dict) or capture.get("fault") or capture.get("dropped", 0)):
        raise ValueError("comparison requires a complete valid capture")
    integrity = capture.get("raw_integrity")
    if integrity is not None and (not isinstance(integrity, dict) or integrity.get("raw_samples_validated") is not True):
        raise ValueError("comparison rejects clipped, saturated or nonfinite original raw samples")
    detector = capture.get("detector") or {}
    samples = capture.get("samples", ())
    arm = detector.get("arm_time_s")
    if (not isinstance(samples, (tuple, list)) or not 2 <= len(samples) <= 200000
            or type(arm) not in (int, float) or not math.isfinite(arm)):
        raise ValueError("comparison requires current arm time and bounded samples")
    for value in (raw_threshold_g, filtered_threshold_g):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("comparison thresholds must be finite and positive")
    previous = None
    for row in samples:
        if (not isinstance(row, (tuple, list)) or len(row) != 4
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in row)
                or (previous is not None and (row[0] <= previous or row[0] - previous > .00625))):
            raise ValueError("comparison contains invalid or gapped samples")
        previous = row[0]
    periods = [samples[index][0] - samples[index - 1][0] for index in range(1, len(samples))]
    period = statistics.median(periods)
    if period <= 0 or any(dt > 5. * period + .000003 for dt in periods):
        raise ValueError("comparison has a sample gap")
    armed_period = detector.get("sample_period_s", period)
    if type(armed_period) not in (int, float) or not math.isfinite(armed_period) or armed_period <= 0:
        raise ValueError("invalid armed sampling period")
    if detector.get("detector_type", "raw") == "raw" and detector.get("impact_metric") == "selected_axis":
        raise ValueError("raw XYZ comparison requires a vector-mode capture and its own threshold")
    reference_interval = detector.get("reference_interval_s")
    if (not isinstance(reference_interval, (list, tuple)) or len(reference_interval) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in reference_interval)
            or not samples[0][0] <= reference_interval[0] < reference_interval[1] < arm):
        raise ValueError("comparison requires the retained validated stationary-reference interval")
    reference_rows = [row[1:] for row in samples if reference_interval[0] <= row[0] <= reference_interval[1]]
    expected_count = detector.get("reference_sample_count")
    if expected_count is not None and (type(expected_count) is not int or len(reference_rows) != expected_count):
        raise ValueError("comparison stationary-reference rows were not retained exactly")
    config = detector_config(filter_config or detector.get("detector_config"))
    reference = detector.get("filter_reference") if detector.get("detector_type") == "filtered" else None
    if not isinstance(reference, dict):
        reference = robust_reference(reference_rows, config)
    raw_base = _xyz(capture.get("raw_baseline_g") or capture.get("baseline_g"))
    engine = CausalImpactDetector(filtered_threshold_g, reference["offset_g"],
                                  reference["noise_sigma_g"], reference["weights_xyz"], config)
    diagnostic = CausalImpactDetector(filtered_threshold_g, reference["offset_g"],
                                      reference["noise_sigma_g"], reference["weights_xyz"], config)
    diagnostic_reset = False
    interval = detector.get("stop_request_interval_s")
    stop = detector.get("stop_request_time_s")
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    if (not isinstance(interval, (list, tuple)) or len(interval) != 2
            or not all(finite(v) for v in interval) or interval[0] > interval[1]):
        interval = None
    # Unknown dispatch timing means every shadow result is censored. Never use
    # the end of a retained packet as a surrogate for an actual stop request.
    cutoff = interval[0] if interval is not None else None
    raw_trace, filtered_trace, raw_level, filtered_level, weighted_level = [], [], [], [], []
    raw_decision = filtered_decision = None
    pre_stop_candidate = pre_stop_diagnostics = None
    frozen = False
    for row in samples:
        when = row[0]
        if cutoff is not None and when >= cutoff and not frozen:
            pre_stop_candidate = None if engine.candidate is None else dict(engine.candidate)
            pre_stop_diagnostics = engine.diagnostics()
            frozen = True
        raw_xyz = tuple(v - b for v, b in zip(row[1:], raw_base))
        level = math.sqrt(sum(v*v for v in raw_xyz))
        raw_trace.append([when] + list(raw_xyz))
        raw_level.append(level)
        if when >= arm:
            if engine.previous_time is None:
                engine.previous_time = when - armed_period
            decision = engine.feed(when, row[1:])
            display = engine
        else:
            if when >= reference_interval[0] and not diagnostic_reset:
                diagnostic = CausalImpactDetector(filtered_threshold_g, reference["offset_g"],
                                                  reference["noise_sigma_g"], reference["weights_xyz"], config)
                diagnostic_reset = True
            if diagnostic.previous_time is None:
                diagnostic.previous_time = when - period
            diagnostic.feed(when, row[1:])
            display, decision = diagnostic, None
        filtered_trace.append([when] + list(display.filtered_xyz))
        filtered_level.append(display.magnitude_g)
        weighted_level.append(display.level_g)
        if when >= arm and cutoff is not None and when < cutoff:
            if raw_decision is None and level >= raw_threshold_g:
                raw_decision = {"crossing_time_s": when, "onset_time_s": when,
                                "confirmation_time_s": when, "level_g": level}
            if filtered_decision is None and decision is not None and decision["confirmation_time_s"] < cutoff:
                filtered_decision = {key: value for key, value in decision.items() if "identity" not in key}
    if cutoff is not None and not frozen:
        pre_stop_candidate = None if engine.candidate is None else dict(engine.candidate)
        pre_stop_diagnostics = engine.diagnostics()
    def verdict(decision):
        result = dict(decision or {})
        result.update(decision_observed=decision is not None, censored=decision is None,
                      evidence="counterfactual_pre_stop_replay", physical_accuracy=False)
        return result
    filtered_verdict = verdict(filtered_decision)
    if filtered_decision is None and pre_stop_candidate is not None:
        filtered_verdict.update({key: value for key, value in pre_stop_candidate.items()
                                if "identity" not in key and key != "energy"})
        filtered_verdict["confirmation_time_s"] = None
    return {"version": 1, "actual_detector": detector.get("detector_type", "raw"),
            "arm_time_s": arm, "stop_request_time_s": stop,
            "stop_request_interval_s": interval, "decision_cutoff_time_s": cutoff,
            "censoring_policy": "strictly_before_dispatch_lower_bound",
            "raw_metric": "xyz_magnitude", "decision_metric": "causal_weighted_xyz",
            "pre_arm_filter_replay": True, "pre_arm_warmup_s": 5. * config["lowpass_tau_s"],
            "raw_trace": raw_trace, "filtered_trace": filtered_trace,
            "raw_level": raw_level, "filtered_level": filtered_level,
            "weighted_level": weighted_level, "raw_threshold_g": raw_threshold_g,
            "filtered_threshold_g": filtered_threshold_g,
            "raw_decision": verdict(raw_decision), "filtered_decision": filtered_verdict,
            "reference_interval_s": list(reference_interval),
            "reference": reference, "filter_config": config,
            "actual_onset_time_s": detector.get("onset_time_s"),
            "actual_crossing_time_s": detector.get("crossing_time_s"),
            "actual_confirmation_time_s": detector.get("confirmation_time_s"),
            "moving_noise": None if pre_stop_diagnostics is None else pre_stop_diagnostics["moving_noise"],
            "physical_accuracy_verified": False, "counterfactual": True}
