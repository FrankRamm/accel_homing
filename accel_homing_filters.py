# Experimental causal accelerometer detector. GPLv3.
"""Bounded signal processing shared by the armed detector and offline replay.

No parameter below is a measured printer optimum. Filtering and confirmation can
miss narrow contacts and add stopping latency. Thresholds belong to this metric
and must never be borrowed implicitly from a raw-magnitude calibration.
"""
import collections
import math
import statistics
import struct

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


class FilteredSampleError(ValueError):
    """Precise rejection reason without changing ValueError compatibility."""
    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


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
            raise FilteredSampleError("invalid_time", "invalid filtered sample timestamp")
        try:
            xyz = _xyz(xyz_g)
        except ValueError:
            raise FilteredSampleError("invalid_xyz", "invalid finite XYZ values")
        previous = self.previous_time
        dt = None if previous is None else when - previous
        if dt is not None and (dt <= 0 or dt > .00625):
            raise FilteredSampleError("nonmonotonic_time" if dt <= 0 else "sample_gap",
                                      "noncontiguous filtered sample timestamps")
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


def _stamp(value):
    if (not isinstance(value, (tuple, list)) or len(value) != 4
            or any(type(v) is not int for v in value) or not 0 <= value[0] <= 0xffffffff
            or not 0 < value[1] <= 0xffffffff or not 2 <= value[3] <= 200000 or not 0 <= value[2] < value[3]):
        raise ValueError("invalid original packet/sample identity")
    return tuple(value)


def validate_censored_peak_record(record):
    """Structural check; publishers must additionally bind the capture receipt."""
    summary = record.get("response_censoring")
    if summary is None:
        if record.get("response_censored"):
            raise ValueError("censored response lacks validated prefix provenance")
        return False
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    if (not isinstance(summary, dict) or record.get("detector_type") != "filtered"
            or record.get("backend", "beacon_raw") != "beacon_raw" or record.get("result") != "impact"
            or record.get("decision_prefix_validated") is not True or record.get("response_censored") is not True
            or record.get("impact_peak_g") is not None or summary.get("decision_prefix_validated") is not True
            or type(summary.get("version")) is not int or summary["version"] != 1 or summary.get("kind") != "native_saturation_after_confirmation"
            or summary.get("cause") not in ("native_clip_sentinel", "axis_full_scale")
            or not finite(record.get("observed_valid_prefix_peak_g")) or record["observed_valid_prefix_peak_g"] < 0):
        raise ValueError("invalid censored native FILTERED impact record")
    first, confirm = summary.get("first_invalid_sample_index"), summary.get("confirmation_sample_index")
    total, prefix, censored = (summary.get(key) for key in (
        "original_sample_count", "valid_prefix_sample_count", "censored_sample_count"))
    if (any(type(v) is not int for v in (first, confirm, total, prefix, censored))
            or not 0 <= confirm < first == prefix < total <= 200000 or censored != total - prefix
            or not finite(summary.get("confirmation_time_s")) or not finite(summary.get("first_invalid_time_s"))
            or not summary["confirmation_time_s"] < summary["first_invalid_time_s"]
            or record.get("confirmation_time_s") != summary["confirmation_time_s"]
            or record.get("observed_valid_prefix_peak_g") != summary.get("observed_valid_prefix_peak_g")):
        raise ValueError("censored impact does not precede the unavailable response")
    _stamp(summary.get("first_invalid_sample"))
    _stamp(summary.get("confirmation_sample"))
    if type(record.get("detector_version")) is not int or record["detector_version"] != DETECTOR_VERSION or detector_config(record.get("detector_config")) != record.get("detector_config"):
        raise ValueError("censored impact lacks its exact detector configuration")
    return True


def validate_filtered_capture_prefix(capture, record=None):
    """Prove a native confirmation from original raw rows before saturation.

    Whole-capture integrity remains false. The private bounded original batches
    are checked in full, including faults after the censor boundary, and never
    become chart payload or persisted raw scratch. Replay uses original online
    sensor-order values/times; exported sample times may have been remapped.
    """
    detector = capture.get("detector") or {}
    summary = capture.get("response_censoring")
    integrity = capture.get("raw_integrity") or {}
    if summary is None:
        if (detector.get("detector_type") == "filtered" and integrity.get("raw_samples_validated") is False):
            raise ValueError("invalid FILTERED capture has no eligible censor proof")
        if record is not None and record.get("response_censored"):
            raise ValueError("claimed censored record has no capture proof")
        return None
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    if (capture.get("fault") or capture.get("error") or capture.get("dropped", 0)
            or not isinstance(summary, dict) or detector.get("detector_type") != "filtered"
            or detector.get("backend") != "beacon_raw" or detector.get("result") != "impact"
            or detector.get("axis") not in ("x", "y") or type(detector.get("attempt_count")) is not int
            or detector["attempt_count"] != 1 or integrity.get("raw_samples_validated") is not False
            or type(summary.get("version")) is not int or summary["version"] != 1
            or summary.get("kind") != "native_saturation_after_confirmation"
            or summary.get("cause") not in ("native_clip_sentinel", "axis_full_scale")):
        raise ValueError("only post-confirmation native saturation permits a censored prefix")
    config = detector_config(detector.get("detector_config"))
    if type(detector.get("detector_version")) is not int or detector["detector_version"] != DETECTOR_VERSION or config != detector.get("detector_config"):
        raise ValueError("censored capture configuration changed")
    samples, sources = capture.get("samples"), capture.get("_native_source_batches")
    mapping, reference = capture.get("_native_mapping"), capture.get("_native_filter_reference")
    full_scale = summary.get("full_scale_g")
    if (not isinstance(samples, (list, tuple)) or not 2 <= len(samples) <= 200000
            or not isinstance(sources, (list, tuple)) or not 1 <= len(sources) <= 100000
            or not isinstance(mapping, (list, tuple)) or len(mapping) != 3
            or any(not isinstance(pair, (list, tuple)) or len(pair) != 2 or pair[0] not in (0, 1, 2)
                   or pair[1] not in (-1, 1) for pair in mapping)
            or len({pair[0] for pair in mapping}) != 3 or not isinstance(reference, dict)
            or not finite(full_scale) or full_scale <= 0):
        raise ValueError("censored capture lacks bounded original native data/reference")
    rows, identities, online_times = [], [], []
    scope = capture.get("_native_capture_scope")
    bounds = detector.get("reference_sample_bounds")
    if (not isinstance(scope, dict) or type(scope.get("version")) is not int or scope["version"] != 1
            or scope.get("kind") != "accepted_reference_through_stream_close"
            or type(scope.get("admission_epoch")) is not int or scope["admission_epoch"] <= 0
            or type(detector.get("capture_admission_epoch")) is not int
            or detector.get("capture_admission_epoch") != scope["admission_epoch"]
            or scope.get("closed_by") not in ("stream_stop", "capture_seal")
            or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or _stamp(scope.get("first_sample")) != _stamp(bounds[0])
            or any(type(scope.get(key)) is not int or scope[key] < 0 for key in (
                "excluded_pre_reference_samples", "excluded_previous_epoch_samples"))):
        raise ValueError("censored capture lacks its exact current reference/stream scope")
    found = None
    counts = {"clipped_axis_values": 0, "saturated_axis_values": 0, "nonfinite_axis_values": 0}
    last, last_clock, original_last = None, None, None
    for source_index, source in enumerate(sources):
        data = source.get("data")
        if isinstance(data, tuple):
            if len(data) % 3:
                raise ValueError("malformed native tuple in censored response")
            raw = list(data)
        elif isinstance(data, (bytes, bytearray, memoryview)) and len(data) % 6 == 0:
            raw = [v for row in struct.iter_unpack("<hhh", data) for v in row]
        else:
            raise ValueError("malformed original native response packet")
        count = len(raw) // 3
        first = source.get("first_sample_position")
        if (type(first) is not int or not 0 <= first < count
                or type(source.get("admission_epoch")) is not int
                or source["admission_epoch"] != scope["admission_epoch"]
                or (source_index and first != 0)):
            raise ValueError("native response changed its admitted sample scope")
        start_clock, delta = source.get("start_clock"), source.get("delta_clock")
        start, end, received = (source.get(key) for key in ("start_time_s", "end_time_s", "received_time_s"))
        scale = source.get("count_g")
        if (type(start_clock) is not int or not 0 <= start_clock <= 0xffffffff or type(delta) is not int
                or not 0 < delta <= 0xffffffff or not 2 <= count <= 200000 or len(rows) + count - first > 200000
                or not all(finite(v) for v in (start, end, received, scale)) or scale <= 0 or end <= start):
            raise ValueError("invalid original native packet metadata")
        period = (end - start) / (count - 1)
        if (not source_index and (start_clock, delta, first, count) != _stamp(scope["first_sample"])):
            raise ValueError("native response did not start at its accepted reference identity")
        if period > 1. / 800. or received - (start + first * period) > capture.get("_native_max_age_s", .05) or end - received > .005:
            raise ValueError("stale/future/low-rate native response cannot be censored")
        original_start, original_end = source.get("filter_start_time_s"), source.get("filter_end_time_s")
        if not finite(original_start) or not finite(original_end) or original_end <= original_start:
            raise ValueError("invalid original response timing cannot be censored")
        original_period = (original_end - original_start) / (count - 1)
        retained_start = original_start + first * original_period
        if original_last is not None and (retained_start <= original_last or retained_start - original_last > 5. * original_period + .000003):
            raise ValueError("original response has a sample gap or repeated timestamp"
                             " (packet=%d, previous_end=%.9f, next_start=%.9f, gap=%.9f, limit=%.9f, epoch=%d, phase=%s)"
                             % (source_index, original_last, retained_start, retained_start - original_last,
                                5. * original_period + .000003, scope["admission_epoch"],
                                "reference" if len(rows) < detector.get("reference_sample_count", 0) else "response"))
        original_last = original_end
        if last_clock is not None and not 0 < ((start_clock - last_clock) & 0xffffffff) < 0x80000000:
            raise ValueError("repeated/backward packet in censored response"
                             " (packet=%d, previous_end_clock=%d, next_start_clock=%d, epoch=%d)"
                             % (source_index, last_clock, start_clock, scope["admission_epoch"]))
        last_clock = (start_clock + delta) & 0xffffffff
        for position in range(first, count):
            when = start + position * period
            if last is not None and (when <= last or when - last > 5. * period + .000003):
                raise ValueError("missing/repeated response samples cannot be censored"
                                 " (packet=%d, position=%d, previous=%.9f, next=%.9f, gap=%.9f, limit=%.9f, epoch=%d)"
                                 % (source_index, position, last, when, when - last,
                                    5. * period + .000003, scope["admission_epoch"]))
            last = when
            xyz = tuple(raw[3 * position:3 * position + 3])
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in xyz):
                raise ValueError("nonfinite native response cannot be censored")
            stamp = (start_clock, delta, position, count)
            invalid = None
            for axis, value in enumerate(xyz):
                counts["clipped_axis_values"] += value == 32767
                counts["saturated_axis_values"] += abs(value * scale) >= full_scale
                if invalid is None and (value == 32767 or abs(value * scale) >= full_scale):
                    invalid = axis
            if invalid is not None and found is None:
                found = (len(rows), stamp, when, invalid, xyz[invalid])
            if found is None:
                expected = (when,) + tuple(xyz[index] * sign * scale for index, sign in mapping)
                if len(rows) >= len(samples) or tuple(samples[len(rows)]) != expected:
                    raise ValueError("exported valid prefix differs from original native rows")
            rows.append(tuple(value * scale for value in xyz))
            identities.append(stamp)
            a, b = source.get("filter_start_time_s"), source.get("filter_end_time_s")
            online_times.append(None if not finite(a) or not finite(b) else a + (b - a) * position / (count - 1))
    if found is None or any(type(integrity.get(key)) is not int or integrity[key] != value for key, value in counts.items()):
        raise ValueError("whole-capture integrity does not match original native values")
    index, stamp, when, axis, value = found
    if (index != len(samples) or _stamp(summary.get("first_invalid_sample")) != stamp
            or summary.get("first_invalid_sample_index") != index or summary.get("first_invalid_time_s") != when
            or summary.get("sensor_axis") != "xyz"[axis] or summary.get("raw_value") != value
            or summary.get("cause") != ("native_clip_sentinel" if value == 32767 else "axis_full_scale")
            or summary.get("original_sample_count") != len(rows)
            or summary.get("valid_prefix_sample_count") != index
            or summary.get("censored_sample_count") != len(rows) - index
            or summary.get("original_last_sample_time_s") != last):
        raise ValueError("first saturation identity/count differs from original response")
    positions = {}
    for event in ("onset", "crossing", "confirmation", "first_armed"):
        event_stamp = _stamp(detector.get(event + "_sample"))
        matches = [i for i, identity in enumerate(identities) if identity == event_stamp]
        event_index = detector.get(event + "_sample_index")
        if (len(matches) != 1 or detector.get(event + "_sample_matches") != 1
                or event_index != matches[0] or not 0 <= event_index < index
                or detector.get(event + "_time_s") != samples[event_index][0]):
            raise ValueError("censored capture lacks a unique retained %s identity" % event)
        positions[event] = event_index
    arm_clock = detector.get("arm_clock")
    if type(arm_clock) is not int or not 0 <= arm_clock <= 0xffffffff:
        raise ValueError("censored capture lost its original arm clock")
    first_eligible = None
    for i, identity in enumerate(identities[:index]):
        relative = (identity[0] - arm_clock) & 0xffffffff
        if relative >= 0x80000000:
            relative -= 0x100000000
        if relative * (identity[3] - 1) + identity[2] * identity[1] >= 0:
            first_eligible = i
            break
    if first_eligible != positions["first_armed"]:
        raise ValueError("censored replay skipped an eligible armed sample")
    if not positions["first_armed"] <= positions["onset"] <= positions["crossing"] < positions["confirmation"] < index:
        raise ValueError("native saturation is not strictly after valid confirmation")
    bounds = detector.get("reference_sample_bounds")
    ref_indices = detector.get("reference_sample_indices")
    if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not isinstance(ref_indices, (list, tuple)) or len(ref_indices) != 2
            or any(type(v) is not int for v in ref_indices)
            or not 0 <= ref_indices[0] < ref_indices[1] < positions["first_armed"]
            or detector.get("reference_sample_matches") != [1, 1]
            or detector.get("reference_sample_count") != ref_indices[1] - ref_indices[0] + 1
            or ref_indices[0] != 0
            or identities[ref_indices[0]] != _stamp(bounds[0]) or identities[ref_indices[1]] != _stamp(bounds[1])
            or detector.get("reference_interval_s") != [samples[ref_indices[0]][0], samples[ref_indices[1]][0]]):
        raise ValueError("censored impact reference provenance changed")
    learned = robust_reference(rows[ref_indices[0]:ref_indices[1] + 1], config)
    for key in ("offset_g", "noise_sigma_g", "weights_xyz"):
        if _xyz(reference.get(key)) != tuple(learned[key]):
            raise ValueError("censored impact frozen reference differs from retained rows")
        public = (detector.get("filter_reference") or {}).get(key)
        expected = [reference[key][raw_index] * (sign if key == "offset_g" else 1) for raw_index, sign in mapping]
        if any(not math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12) for a, b in zip(_xyz(public), expected)):
            raise ValueError("public filtered reference differs from the original sensor model")
    if reference.get("sample_count") != detector["reference_sample_count"] or detector.get("filter_reference", {}).get("sample_count") != detector["reference_sample_count"]:
        raise ValueError("censored reference sample count changed")
    baseline = _xyz(capture.get("baseline_g"))
    if any(not math.isclose(value, reference["offset_g"][raw_index] * sign, rel_tol=1e-12, abs_tol=1e-12)
           for value, (raw_index, sign) in zip(baseline, mapping)):
        raise ValueError("censored capture baseline differs from frozen reference")
    threshold, period = detector.get("threshold_g"), detector.get("sample_period_s")
    if not finite(threshold) or threshold <= 0 or not finite(period) or period <= 0:
        raise ValueError("invalid censored detector threshold/period")
    engine = CausalImpactDetector(threshold, reference["offset_g"], reference["noise_sigma_g"], reference["weights_xyz"], config)
    start_index = positions["first_armed"]
    if online_times[start_index] is None:
        raise ValueError("censored prefix lacks original online filter timing")
    engine.previous_time = online_times[start_index] - period
    decision = None
    for i in range(start_index, index):
        if online_times[i] is None:
            raise ValueError("censored prefix lost an online packet timestamp")
        decision = engine.feed(online_times[i], rows[i], i)
        if decision is not None:
            break
    if decision is None or any(decision[event + "_identity"] != positions[event] for event in ("onset", "crossing", "confirmation")):
        raise ValueError("retained valid prefix does not reproduce actual filtered confirmation")
    for event in ("onset", "crossing", "confirmation"):
        trigger_time = detector.get("trigger_" + event + "_time_s")
        if trigger_time != decision[event + "_time_s"]:
            raise ValueError("original trigger timestamp differs from online prefix replay")
    if summary.get("confirmation_sample_index") != positions["confirmation"] or _stamp(summary.get("confirmation_sample")) != identities[positions["confirmation"]] \
            or summary.get("confirmation_time_s") != detector["confirmation_time_s"]:
        raise ValueError("censor summary changed its retained confirmation")
    if summary.get("first_invalid_original_time_s") != online_times[index]:
        raise ValueError("censor summary lost its original saturation timestamp")
    peak = max(math.sqrt(sum((v - b) ** 2 for v, b in zip(row[1:], baseline))) for row in samples)
    if summary.get("observed_valid_prefix_peak_g") != peak:
        raise ValueError("censored prefix peak is not an observed valid value")
    result = dict(summary, decision_prefix_validated=True,
                  first_invalid_sample=list(stamp), confirmation_sample=list(identities[positions["confirmation"]]))
    if record is not None:
        if (record.get("result") != "impact" or record.get("detector_type") != "filtered"
                or record.get("detector_config") != config
                or type(record.get("detector_version")) is not int or record["detector_version"] != detector["detector_version"]
                or record.get("axis") != detector.get("axis") or record.get("arm_time_s") != detector.get("arm_time_s")
                or record.get("arm_clock") != arm_clock
                or _stamp(record.get("onset_sample")) != _stamp(detector.get("onset_sample"))
                or record.get("trigger_onset_time_s", record.get("onset_time_s")) != detector.get("trigger_onset_time_s")
                or not math.isclose(record.get("workflow_threshold_g", record.get("threshold_g", 0)), threshold, rel_tol=1e-12, abs_tol=1e-12)
                or record.get("impact_peak_g") is not None
                or record.get("trigger_confirmation_time_s") != detector.get("trigger_confirmation_time_s")):
            raise ValueError("censored prefix does not belong to current filtered record")
        existing = record.get("response_censoring")
        if existing is not None and existing != result:
            raise ValueError("censored record differs from validated capture proof")
    return result


def compare_filter_capture(capture, raw_threshold_g, filtered_threshold_g, filter_config=None):
    """Counterfactual paired replay, censored before the actual stop request.

    Complete traces remain available for display. Rows after the conservative
    dispatch lower bound cannot establish a shadow detection or improvement.
    """
    if (not isinstance(capture, dict) or capture.get("fault") or capture.get("dropped", 0)):
        raise ValueError("comparison requires a complete valid capture")
    censor = validate_filtered_capture_prefix(capture)
    integrity = capture.get("raw_integrity")
    if censor is None and integrity is not None and (not isinstance(integrity, dict) or integrity.get("raw_samples_validated") is not True):
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
    armed_index = detector.get("first_armed_sample_index")
    armed_boundary = (samples[armed_index][0] if type(armed_index) is int and 0 <= armed_index < len(samples) else arm)
    if (not isinstance(reference_interval, (list, tuple)) or len(reference_interval) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in reference_interval)
            or not samples[0][0] <= reference_interval[0] < reference_interval[1] < armed_boundary):
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
    native_inputs, native_mapping = [], capture.get("_native_mapping")
    native_reference = capture.get("_native_filter_reference")
    if detector.get("detector_type") == "filtered" and isinstance(native_reference, dict) and capture.get("_native_source_batches"):
        for source in capture["_native_source_batches"]:
            raw = (source["data"] if isinstance(source["data"], tuple) else
                   tuple(v for row in struct.iter_unpack("<hhh", source["data"]) for v in row))
            count = len(raw) // 3
            for position in range(source.get("first_sample_position", 0), count):
                if len(native_inputs) >= len(samples):
                    break
                a, b = source.get("filter_start_time_s"), source.get("filter_end_time_s")
                online = (samples[len(native_inputs)][0] if a is None or b is None else
                          a + (b - a) * position / (count - 1))
                native_inputs.append((online, tuple(v * source["count_g"] for v in raw[position * 3:position * 3 + 3])))
    engine_reference = native_reference if native_inputs else reference
    engine = CausalImpactDetector(filtered_threshold_g, engine_reference["offset_g"],
                                  engine_reference["noise_sigma_g"], engine_reference["weights_xyz"], config)
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
    cutoff_reason = "stop_dispatch_lower_bound" if cutoff is not None else "unknown_stop_time"
    if censor is not None and cutoff is not None and censor["first_invalid_original_time_s"] < cutoff:
        cutoff, cutoff_reason = censor["first_invalid_original_time_s"], "first_invalid_sample"
    def display_time(original):
        if original is None or not native_inputs:
            return original
        sources = [source for source in capture["_native_source_batches"]
                   if type(source.get("filter_start_time_s")) in (int, float)
                   and type(source.get("filter_end_time_s")) in (int, float)]
        if not sources:
            return original
        containing = [s for s in sources if s["filter_start_time_s"] <= original <= s["filter_end_time_s"]]
        source = (containing[0] if containing else min(sources, key=lambda s: min(
            abs(s["filter_start_time_s"] - original), abs(s["filter_end_time_s"] - original))))
        return source["start_time_s"] + (original - source["filter_start_time_s"]) * (
            source["end_time_s"] - source["start_time_s"]) / (source["filter_end_time_s"] - source["filter_start_time_s"])
    raw_trace, filtered_trace, raw_level, filtered_level, weighted_level = [], [], [], [], []
    raw_decision = filtered_decision = None
    pre_stop_candidate = pre_stop_diagnostics = None
    frozen = False
    first_armed = detector.get("first_armed_sample_index")
    if type(first_armed) is not int or not 0 <= first_armed < len(samples):
        first_armed = next((i for i, row in enumerate(samples) if row[0] >= arm), len(samples))
    def mapped_decision(decision):
        if decision is None:
            return None
        result = dict(decision)
        for event in ("onset", "crossing", "confirmation"):
            index = decision.get(event + "_identity")
            if type(index) is int and 0 <= index < len(samples):
                result["original_" + event + "_time_s"] = decision.get(event + "_time_s")
                result[event + "_time_s"] = samples[index][0]
        return result
    for sample_index, row in enumerate(samples):
        when = row[0]
        evidence_time = native_inputs[sample_index][0] if native_inputs else when
        if cutoff is not None and evidence_time >= cutoff and not frozen:
            pre_stop_candidate = None if engine.candidate is None else dict(engine.candidate)
            pre_stop_diagnostics = engine.diagnostics()
            frozen = True
        raw_xyz = tuple(v - b for v, b in zip(row[1:], raw_base))
        level = math.sqrt(sum(v*v for v in raw_xyz))
        raw_trace.append([when] + list(raw_xyz))
        raw_level.append(level)
        if sample_index >= first_armed:
            feed_time, feed_xyz = native_inputs[sample_index] if native_inputs else (when, row[1:])
            if engine.previous_time is None:
                engine.previous_time = feed_time - armed_period
            decision = mapped_decision(engine.feed(feed_time, feed_xyz, sample_index))
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
        display_xyz = ([display.filtered_xyz[index] * sign for index, sign in native_mapping]
                       if native_inputs and sample_index >= first_armed else list(display.filtered_xyz))
        filtered_trace.append([when] + list(display_xyz))
        filtered_level.append(display.magnitude_g)
        weighted_level.append(display.level_g)
        if sample_index >= first_armed and cutoff is not None and evidence_time < cutoff:
            if raw_decision is None and level >= raw_threshold_g:
                raw_decision = {"crossing_time_s": when, "onset_time_s": when,
                                "confirmation_time_s": when, "original_confirmation_time_s": evidence_time, "level_g": level}
            if (filtered_decision is None and decision is not None
                    and decision.get("original_confirmation_time_s", decision["confirmation_time_s"]) < cutoff):
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
        pre_stop_candidate = mapped_decision(pre_stop_candidate)
        filtered_verdict.update({key: value for key, value in pre_stop_candidate.items()
                                if "identity" not in key and key != "energy"})
        filtered_verdict["confirmation_time_s"] = None
    return {"version": 1, "actual_detector": detector.get("detector_type", "raw"),
            "arm_time_s": display_time(arm), "stop_request_time_s": display_time(stop),
            "stop_request_interval_s": None if interval is None else [display_time(v) for v in interval],
            "original_arm_time_s": arm, "original_stop_request_time_s": stop,
            "original_stop_request_interval_s": interval,
            "decision_cutoff_time_s": (censor["first_invalid_time_s"] if cutoff_reason == "first_invalid_sample" else display_time(cutoff)),
            "decision_cutoff_original_time_s": cutoff,
            "decision_time_domain": "original_online_print_time" if native_inputs else "capture_print_time",
            "display_stop_request_interval_s": None if interval is None else [display_time(v) for v in interval],
            "display_stop_request_time_s": display_time(stop),
            "decision_cutoff_reason": cutoff_reason, "response_censoring": censor,
            "raw_integrity": integrity,
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
