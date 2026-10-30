# Experimental causal accelerometer detector. GPLv3.
"""Bounded signal processing shared by the armed detector and offline replay.

No parameter below is a measured printer optimum. Filtering and confirmation can
miss narrow contacts and add stopping latency. Thresholds belong to this metric
and must never be borrowed implicitly from a raw-magnitude calibration.
"""
import collections
import copy
import hashlib
import json
import math
import statistics
import struct

DETECTOR_VERSION = 2
SUPPORTED_DETECTOR_VERSIONS = (1, 2)
FILTERED_POSITION_POLICY = "accepted_threshold_crossing_v1"
LEGACY_FILTERED_POSITION_POLICY = "filtered_lookback_v1"
HYBRID_POSITION_POLICY = "associated_raw_threshold_crossing_v1"
MAX_HYBRID_ROWS = 256
MAX_REFERENCE_SAMPLES = 65536
MAX_HISTORY_SAMPLES = 128
MAX_MOVING_SAMPLES = 256
NATIVE_CLIP_POLICY = "beacon_previous_sensor_sign_v1"
ACCURACY_EVENT_ROWS = 50
MAX_HOME_TELEMETRY_BYTES = 128 * 1024


def _stop_relation(when, bounds):
    if (type(when) not in (int, float) or not math.isfinite(when)
            or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds)):
        return "unknown"
    return "before_lower" if when < bounds[0] else "after_upper" if when > bounds[1] else "within_request_interval"


def _filter_state(engine):
    return {"previous_time_s": engine.previous_time, "filtered_sensor_xyz_g": list(engine.filtered_xyz),
            "history": [list(row) for row in engine.history], "candidate": copy.deepcopy(engine.candidate),
            "candidate_count": engine.candidates, "rejected_candidates": engine.rejected_candidates,
            "rejection_reasons": dict(engine.rejection_reasons), "moving_learning_frozen": engine.moving_frozen}


def prepare_accuracy_home_telemetry(capture, record, check_running=None):
    """Freeze <=50 original consecutive rows after the completed home, never during motion.

    FILTERED state is reconstructed from the retained exact prefix and checked
    against the actual decision. It is not an online snapshot. Unsupported
    captures retain explicit unavailable fields instead of inventing input data.
    The canonical JSON string owns no capture/attempt references.
    """
    if record.get("result") != "impact" or record.get("capture_valid") is False:
        raise ValueError("only qualified impact captures have accuracy telemetry")
    if check_running is not None:
        check_running()
    detector = capture.get("detector") or {}
    original_times = {event: record.get("trigger_" + event + "_time_s", record.get(event + "_time_s"))
                      for event in ("onset", "crossing", "confirmation")}
    events = {event: {"sample": detector.get(event + "_sample", record.get(event + "_sample")),
                      "original_time_s": original_times[event],
                      "original_time_origin": ("filtered_online_decision" if record.get("detector_type") == "filtered" else
                          "raw_disarm_trigger_time_conversion" if event == "onset" else "raw_controller_threshold_crossing_conversion"),
                      "capture_time_s": detector.get(event + "_time_s"),
                      "unavailable_reason": None if original_times[event] is not None else "not_recorded_by_actual_controller"}
              for event in original_times}
    record_json = json.dumps(record, sort_keys=True, allow_nan=False, separators=(",", ":"))
    home = {"version": 1, "kind": "accuracy_home_telemetry", "run_id": record.get("run_id"),
            "record_sha256": hashlib.sha256(record_json.encode("utf-8")).hexdigest(),
            "axis": record.get("axis"), "workflow_attempt": record.get("workflow_attempt"),
            "phase": record.get("workflow_phase"), "measurement_index": record.get("accuracy_sample"),
            "result": record["result"], "physical_position_verified": False,
            "threshold_g": record.get("threshold_g"), "impact_metric": record.get("impact_metric"),
            "direction": record.get("direction"), "selected_machine_axis": record.get("axis"),
            "detector_type": record.get("detector_type", "raw"), "detector_version": record.get("detector_version"),
            "control_detector": record.get("control_detector", record.get("detector_type", "raw")),
            "requested_detector": record.get("requested_detector", record.get("detector_type", "raw")),
            "position_selection": copy.deepcopy(record.get("position_selection")),
            "position_capture_time_s": record.get("position_time_s"),
            "decision_semantics": "causal_temporal_confirmation" if record.get("detector_type") == "filtered" else "immediate_raw_threshold_decision",
            "detector_config": record.get("detector_config"), "sensor_context": record.get("sensor_context"),
            "geometry_context": record.get("geometry_context"),
            "homing_speed_mm_s": record.get("homing_speed_mm_s"),
            "homing_accel_mm_s2": record.get("homing_accel_mm_s2"),
            "arm_time_s": record.get("arm_time_s"), "arm_clock": record.get("arm_clock"),
            "planned_movement_start_time_s": None, "first_step_time_s": None,
            "movement_time_unavailable_reason": "not_exposed_by_existing_homing_interface",
            "returned_trigger_time_s": record.get("returned_trigger_time_s"),
            "returned_trigger_time_unavailable_reason": None if record.get("returned_trigger_time_s") is not None else "not_recorded",
            "events": events, "stop_request_interval_s": record.get("stop_request_interval_s"),
            "stop_request_is_mcu_halt": False,
            "travel_mm_exact": record.get("travel_mm_exact", record.get("travel_mm")),
            "trigger_travel_mm_signed": record.get("trigger_travel_mm_signed"),
            "halt_travel_mm_signed": record.get("halt_travel_mm_signed"),
            "native_clip_decode": capture.get("native_clip_decode"),
            "amplitude_limited": capture.get("amplitude_limited", False),
            "response_censoring": capture.get("response_censoring"),
            "filter_reference_sensor_frame": capture.get("_accuracy_sensor_reference", capture.get("_native_filter_reference")),
            "baseline_machine_xyz_g": capture.get("baseline_g"),
            "reference_sample_count": detector.get("reference_sample_count"),
            "reference_sample_bounds": detector.get("reference_sample_bounds"),
            "filter_diagnostics": record.get("filter_diagnostics"),
            "effective_onset_gate_g": None, "state_before_window": None,
            "state_provenance": {"available": False, "reason": "original_native_packets_not_retained_for_verified_state_replay"
                                 if record.get("detector_type") == "filtered" else "actual_controller_has_no_filter_state"},
            "window": {"available": False, "reason": "original_native_packets_not_retained",
                       "limit_rows": ACCURACY_EVENT_ROWS, "downsampled": False, "rows": []}}
    if home["detector_type"] == "filtered":
        home["position_policy"] = filtered_position_policy(record)
        if filtered_position_policy(detector) != home["position_policy"]:
            raise ValueError("accuracy telemetry position policy differs from its actual detector")
        home["lookback_onset_time_s"] = record.get("lookback_onset_time_s")
        if home["position_policy"] == HYBRID_POSITION_POLICY:
            validate_position_selection(home["position_selection"], record)
    sources = capture.get("_accuracy_native_sources", capture.get("_native_source_batches"))
    mapping = capture.get("_accuracy_native_mapping", capture.get("_native_mapping"))
    if sources:
        filtered = home["detector_type"] == "filtered"
        if filtered:
            validate_filtered_capture_prefix(capture, record, check_running)
        # First locate packet identities arithmetically. Do not expand the full
        # capture into rich diagnostic row dictionaries before selecting 50.
        total, found = 0, {event: [] for event in events}
        armed_index, armed_time = None, None
        for source in sources:
            count = len(source["data"]) // (3 if isinstance(source["data"], tuple) else 6)
            first_row = source["first_sample_position"]
            packet = (source["start_clock"], source["delta_clock"], count)
            for event, value in events.items():
                stamp = value["sample"]
                if stamp is not None and (stamp[0], stamp[1], stamp[3]) == packet:
                    found[event].append(total + stamp[2] - first_row)
                    value["row_acquisition_snapshot_time_s"] = source["filter_start_time_s"] + (source["filter_end_time_s"] - source["filter_start_time_s"]) * stamp[2] / (count - 1)
                    value["row_time_origin"] = source.get("time_origin", "detector_or_capture_packet_snapshot")
            stamp = detector.get("first_armed_sample")
            if stamp is not None and (stamp[0], stamp[1], stamp[3]) == packet:
                armed_index = total + stamp[2] - first_row
                armed_time = source["filter_start_time_s"] + (source["filter_end_time_s"] - source["filter_start_time_s"]) * stamp[2] / (count - 1)
            total += count - first_row
        if len(found["onset"]) != 1:
            raise ValueError("accuracy telemetry lost the unique original onset row")
        onset_index = found["onset"][0]
        first = max(armed_index if filtered else 0, onset_index - 16)
        last = min(total, first + ACCURACY_EVENT_ROWS)
        selected = []
        home["window"] = {"available": True, "limit_rows": ACCURACY_EVENT_ROWS,
            "selection": "up_to_16_rows_before_onset_excluding_filtered_prearm_then_consecutive_until_limit_or_capture_end",
            "downsampled": False, "source_frame": "sensor_xyz", "source_row_count": total,
            "first_source_row_index": first, "last_source_row_index": last - 1,
            "omitted_before": first, "omitted_after": total - last,
            "pre_onset_rows_requested": 16, "pre_onset_rows_retained": onset_index - first,
            "edge_truncated": onset_index - first < 16 or last - first < ACCURACY_EVENT_ROWS,
            "events_outside_window": [event for event in events if found[event] and not first <= found[event][0] < last],
            "axes_map": mapping, "rows": selected}
        for event, value in events.items():
            value["retained_matches"] = len(found[event])
        engine = decision = None
        if filtered:
            reference = home["filter_reference_sensor_frame"]
            config = validate_detector_identity(home["detector_version"], home["detector_config"])
            home["effective_onset_gate_g"] = max(config["onset_ratio"] * home["threshold_g"],
                3. * math.sqrt(sum((s * w) ** 2 for s, w in zip(reference["noise_sigma_g"], reference["weights_xyz"]))))
            engine = CausalImpactDetector(home["threshold_g"], reference["offset_g"], reference["noise_sigma_g"],
                                          reference["weights_xyz"], config, home["position_policy"])
            engine.previous_time = armed_time - detector["sample_period_s"]
        offset = 0
        for source_number, source in enumerate(sources):
            if check_running is not None:
                check_running()
            data = source["data"]
            count = len(data) // (3 if isinstance(data, tuple) else 6)
            raw_rows = (zip(*[iter(data)] * 3) if isinstance(data, tuple) else struct.iter_unpack("<hhh", data))
            seed, limits = tuple(source["decode_previous_raw"]), source.get("clip_values")
            control_seed, control_first = source.get("control_decode_previous_raw"), source.get("control_first_sample_position")
            a, b = source["filter_start_time_s"], source["filter_end_time_s"]
            period = (b - a) / (count - 1)
            previous_decoded = seed
            for position, raw in enumerate(raw_rows):
                if check_running is not None and position % 256 == 0:
                    check_running()
                if not filtered and control_seed is not None and position == control_first:
                    previous_decoded = tuple(control_seed)
                prior = previous_decoded
                decoded_xyz = decode_native_row(raw, prior, limits) if limits is not None else raw
                previous_decoded = decoded_xyz
                if position < source["first_sample_position"]:
                    continue
                index = offset + position - source["first_sample_position"]
                if index >= last and (engine is None or decision is not None):
                    break
                when = a + (b - a) * position / (count - 1)
                stamp = [source["start_clock"], source["delta_clock"], position, count]
                if engine is not None and index == first:
                    home["state_before_window"] = _filter_state(engine)
                processed = engine is not None and index >= armed_index and decision is None
                if processed:
                    decision = engine.feed(when, tuple(v * source["count_g"] for v in decoded_xyz), tuple(stamp))
                if not first <= index < last:
                    continue
                full_scale = (record.get("sensor_context") or {}).get("full_scale_g")
                unknown_decode = limits is None and 32767 in raw
                row = {"sample": stamp, "original_time_s": when, "time_origin": source.get("time_origin", "detector_or_capture_packet_snapshot"),
                       "capture_time_s": source["start_time_s"] + (source["end_time_s"] - source["start_time_s"]) * position / (count - 1),
                       "raw_sensor_xyz_counts": raw, "marker_axes": [v == 32767 for v in raw],
                       "valid_original_axes": [v != 32767 and math.isfinite(v) for v in raw],
                       "range_limit_axes": [bool(full_scale is not None and v != 32767 and abs(v * source["count_g"]) >= full_scale) for v in raw],
                       "decoded_sensor_xyz_counts": None if unknown_decode else list(decoded_xyz),
                       "decoded_sensor_xyz_g": None if unknown_decode else [v * source["count_g"] for v in decoded_xyz],
                       "stop_relation": _stop_relation(when, home["stop_request_interval_s"]),
                       "processed_by_actual_filter": processed, "filtered_sensor_xyz_g": list(engine.filtered_xyz) if processed else None,
                       "weighted_filtered_level_g": engine.level_g if processed else None, "decoder_previous_sensor_counts": list(prior),
                       "period_s": period, "clip_values": limits,
                       "control_seed_convention": "continuous_sensor_seed" if filtered else "legacy_raw_controller_seed_where_recorded"}
                selected.append(row)
            offset += count - source["first_sample_position"]
            if offset >= last and (engine is None or decision is not None):
                break
        if filtered:
            if decision is None or any(tuple(events[event]["sample"]) != decision[event + "_identity"]
                    or original_times[event] != decision[event + "_time_s"] for event in events):
                raise ValueError("accuracy telemetry replay does not match actual original decision")
            home["state_provenance"] = {"available": True, "origin": "post_stop_exact_prefix_replay",
                "actual_decision_verified": True, "online_snapshot": False,
                "initial_filtered_sensor_xyz_g": [0., 0., 0.],
                "first_armed_sample": detector["first_armed_sample"],
                "first_armed_previous_time_s": armed_time - detector["sample_period_s"],
                "reference_sample_bounds": detector.get("reference_sample_bounds"),
                "reference_sample_count": detector.get("reference_sample_count")}
            replay_counts = {"candidate_count": engine.candidates, "rejected_candidates": engine.rejected_candidates,
                             "rejection_reasons": dict(engine.rejection_reasons)}
            actual_counts = record.get("filter_diagnostics")
            home["state_provenance"].update(replay_candidate_summary=replay_counts,
                replay_candidate_summary_matches_actual=(None if not isinstance(actual_counts, dict) else
                    all(actual_counts.get(key) == value for key, value in replay_counts.items())))
    encoded = json.dumps(home, sort_keys=True, allow_nan=False, separators=(",", ":"))
    if len(encoded.encode("utf-8")) > MAX_HOME_TELEMETRY_BYTES:
        raise ValueError("accuracy home telemetry exceeds bounded metadata limit")
    if check_running is not None:
        check_running()
    return encoded


def validate_accuracy_home_telemetry(encoded, record):
    """Bind an immutable, bounded diagnostic receipt to its actual home record."""
    if not isinstance(encoded, str) or len(encoded.encode("utf-8")) > MAX_HOME_TELEMETRY_BYTES:
        raise ValueError("invalid bounded accuracy home telemetry")
    home = json.loads(encoded)
    canonical = json.dumps(home, sort_keys=True, allow_nan=False, separators=(",", ":"))
    expected = hashlib.sha256(json.dumps(record, sort_keys=True, allow_nan=False, separators=(",", ":")).encode("utf-8")).hexdigest()
    if (canonical != encoded or type(home.get("version")) is not int or home["version"] != 1
            or home.get("kind") != "accuracy_home_telemetry" or home.get("record_sha256") != expected
            or any(home.get(key) != record.get(other) for key, other in (
                ("run_id", "run_id"), ("axis", "axis"), ("workflow_attempt", "workflow_attempt"),
                ("phase", "workflow_phase"), ("measurement_index", "accuracy_sample"),
                ("threshold_g", "threshold_g"), ("detector_type", "detector_type")))):
        raise ValueError("accuracy telemetry does not belong to the current home")
    window = home.get("window") or {}
    state = home.get("state_before_window") or {}
    if (window.get("limit_rows") != ACCURACY_EVENT_ROWS or window.get("downsampled") is not False
            or not isinstance(window.get("rows"), list) or len(window["rows"]) > ACCURACY_EVENT_ROWS
            or len(state.get("history", ())) > MAX_HISTORY_SAMPLES):
        raise ValueError("accuracy diagnostic window/state exceeds its limits")
    if record.get("detector_type") == "filtered" and filtered_position_policy(home) != filtered_position_policy(record):
        raise ValueError("accuracy telemetry position policy differs from its home record")
    if filtered_position_policy(record) == HYBRID_POSITION_POLICY:
        validate_position_selection(home.get("position_selection"), record)
        if home.get("position_selection") != record.get("position_selection"):
            # JSON roundtrips reference-bound tuples into lists.
            if json.dumps(home.get("position_selection"), sort_keys=True) != json.dumps(record.get("position_selection"), sort_keys=True):
                raise ValueError("accuracy telemetry changed its frozen HYBRID selection")
    return home


def native_clip_limits(values):
    """Require actual configured endpoints; the native marker is never a limit."""
    if (not isinstance(values, (tuple, list)) or len(values) != 2
            or any(type(v) is not int for v in values)
            or not 0 < values[0] < 32767 or not -32768 <= values[1] < 0):
        raise ValueError("native marker decoding requires configured signed clip endpoints")
    return tuple(values)


def decode_native_row(values, previous, clips):
    """Beacon convention: infer a marker's endpoint from prior SENSOR code sign.

    This is a bounded substitution, never an observed sign or true amplitude.
    Zero (including a new stream's explicit zero seed) selects the positive end.
    """
    if (len(values) != 3 or len(previous) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values + tuple(previous))):
        raise ValueError("nonfinite native decoder input")
    if 32767 not in values:
        return tuple(values)
    clips = native_clip_limits(clips)
    return tuple(clips[0 if last >= 0 else 1] if value == 32767 else value
                 for value, last in zip(values, previous))


def decode_native_values(raw, previous, clips):
    out = []
    for index in range(0, len(raw), 3):
        previous = decode_native_row(tuple(raw[index:index + 3]), previous, clips)
        out.extend(previous)
    return out


SIGNED_CONFIG = {
    "version": 1,
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
DEFAULT_CONFIG = dict(SIGNED_CONFIG, version=DETECTOR_VERSION, signal_mode="rectified")


class FilteredSampleError(ValueError):
    """Precise rejection reason without changing ValueError compatibility."""
    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


def detector_config(config=None):
    """Validate the entire versioned configuration; unknown fields fail closed."""
    if config is not None and not isinstance(config, dict):
        raise ValueError("unknown filtered-detector configuration")
    version = DETECTOR_VERSION if config is None else config.get("version", DETECTOR_VERSION)
    if type(version) is not int or version not in SUPPORTED_DETECTOR_VERSIONS:
        raise ValueError("unsupported filtered-detector version")
    result = dict(SIGNED_CONFIG if version == 1 else DEFAULT_CONFIG)
    if config is not None:
        if set(config) - set(result):
            raise ValueError("unknown filtered-detector configuration")
        result.update(config)
    if version == 2 and result["signal_mode"] != "rectified":
        raise ValueError("unsupported filtered-detector signal mode")
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


def signal_mode(config):
    return "signed" if config["version"] == 1 else config["signal_mode"]


def filtered_position_policy(metadata):
    """Absent persisted identity denotes the historical lookback attribution."""
    policy = metadata.get("position_policy", LEGACY_FILTERED_POSITION_POLICY)
    if policy not in (FILTERED_POSITION_POLICY, LEGACY_FILTERED_POSITION_POLICY, HYBRID_POSITION_POLICY):
        raise ValueError("unsupported filtered position policy")
    return policy


def validate_detector_identity(version, config, metadata=None):
    """Persisted models/receipts require a complete exact versioned config."""
    if type(version) is not int or version not in SUPPORTED_DETECTOR_VERSIONS or not isinstance(config, dict):
        raise ValueError("filtered detector lacks its exact version/configuration")
    canonical = detector_config(config)
    if canonical != config or canonical["version"] != version:
        raise ValueError("filtered detector must persist every exact parameter for its version")
    if metadata is not None and (not isinstance(metadata, dict)
            or ("signal_mode" in metadata and metadata["signal_mode"] != signal_mode(canonical))):
        raise ValueError("filtered detector signal-mode label contradicts its configuration")
    if metadata is not None:
        filtered_position_policy(metadata)
    return canonical


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
                 weights_xyz=(1., 1., 1.), config=None, position_policy=FILTERED_POSITION_POLICY):
        self.config = detector_config(config)
        self.signal_mode = signal_mode(self.config)
        self.position_policy = filtered_position_policy({"position_policy": position_policy})
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
        self.rejection_reasons = {"timeout": 0, "release": 0}
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
        filter_input = tuple(abs(value) for value in delta) if self.signal_mode == "rectified" else delta
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
                                  for old, v in zip(self.filtered_xyz, filter_input))
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
            lookback = row
            # Search only the final contiguous rising region, at most .75 ms.
            for old in reversed(self.history):
                if old[1] < gate:
                    break
                lookback = old
            # HYBRID keeps every FILTERED control event unchanged. Its separate
            # RAW position receipt must never replace the control onset here.
            onset = lookback if self.position_policy == LEGACY_FILTERED_POSITION_POLICY else row
            self.candidate = {"crossing_time_s": when, "crossing_identity": identity,
                              "onset_time_s": onset[0], "onset_identity": onset[3],
                              "lookback_onset_time_s": lookback[0], "lookback_onset_identity": lookback[3],
                              "lookback_onset_level_g": lookback[1],
                              "onset_level_g": onset[1], "energy": 0., "support_samples": 0,
                              "peak_level_g": self.level_g}
        candidate = self.candidate
        if candidate is not None:
            elapsed = when - candidate["crossing_time_s"]
            if elapsed > self.config["confirmation_max_s"] + 1e-12:
                self.rejected_candidates += 1
                self.rejection_reasons["timeout"] += 1
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
                    self.rejection_reasons["release"] += 1
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
                "rejection_reasons": dict(self.rejection_reasons),
                "moving_noise": moving, "moving_learning_frozen": self.moving_frozen,
                "learning_policy": "frozen_before_arm", "threshold_adaptation": False,
                "history_samples": len(self.history), "moving_samples": len(self.moving)}


class AssociatedRawPosition:
    """Bounded observation only; FILTERED alone accepts and dispatches the stop.

    The RAW comparison is the production unweighted squared count-space XYZ
    metric about QuietReference.baseline(), not the FILTERED robust reference.
    A position requires one observed below-to-above threshold edge inside the
    accepted candidate's contiguous above-gate region. Rejected candidates,
    marker substitutions, missing brackets, and multiple edges cannot supply it.
    Association is evaluated after dispatch, on this frozen pre-confirmation
    state. No row after FILTERED confirmation is ever observed.
    """
    def __init__(self, threshold_g, baseline_counts, count_g, config, arm_time_s, profile=None,
                 reference_original_valid=True, clip_values=None):
        if (type(threshold_g) not in (int, float) or not math.isfinite(threshold_g) or threshold_g <= 0
                or type(count_g) not in (int, float) or not math.isfinite(count_g) or count_g <= 0
                or type(arm_time_s) not in (int, float) or not math.isfinite(arm_time_s)):
            raise ValueError("invalid hybrid RAW reference/threshold")
        self.baseline_counts = _xyz(baseline_counts)
        self.threshold_g, self.count_g = threshold_g, count_g
        self.limit_squared = (threshold_g / count_g) ** 2
        if not math.isfinite(self.limit_squared):
            raise ValueError("invalid hybrid RAW squared threshold")
        self.config = detector_config(config)
        self.arm_time = arm_time_s
        self.profile = copy.deepcopy(profile)
        self.reference_original_valid = bool(reference_original_valid)
        self.clip_values = None if clip_values is None else native_clip_limits(clip_values)
        self.rows = collections.deque(maxlen=MAX_HYBRID_ROWS)
        self.previous = None
        self.last_rejection_time = None
        self.rejected_candidates = 0
        self.evicted_through_time = None
        self.frozen = False
        self.receipt = None

    def observe(self, when, xyz_counts, identity, engine, original_valid=True, original_counts=None):
        if self.frozen:
            return
        level2 = sum((value - base) ** 2 for value, base in zip(xyz_counts, self.baseline_counts))
        time_valid = bool(math.isfinite(level2) and type(when) in (int, float)
                          and math.isfinite(when) and identity is not None)
        amplitude_valid = bool(original_valid and time_valid)
        marker_proven_above = False
        if not original_valid and original_counts is not None and self.clip_values is not None:
            # This is a bound on possible native RAW decoded endpoint codes,
            # never an observed amplitude or a mechanical acceleration bound.
            minimum2 = sum(min((endpoint - base) ** 2 for endpoint in self.clip_values)
                           if value == 32767 else (value - base) ** 2
                           for value, base in zip(original_counts, self.baseline_counts))
            marker_proven_above = bool(time_valid and 32767 in original_counts and minimum2 >= self.limit_squared)
        valid = amplitude_valid or marker_proven_above
        above = valid and (marker_proven_above or level2 >= self.limit_squared)
        previous = self.previous
        crossing = bool(above and previous is not None and previous[3] and not previous[1])
        row = (when, above, identity, valid, level2, crossing,
               None if previous is None else previous[2], None if previous is None else previous[0],
               tuple(xyz_counts), None if previous is None else previous[4],
               None if previous is None else previous[8], amplitude_valid, marker_proven_above)
        if len(self.rows) == MAX_HYBRID_ROWS:
            self.evicted_through_time = self.rows[0][0]
        self.rows.append(row)
        self.previous = row
        if engine.rejected_candidates != self.rejected_candidates:
            self.last_rejection_time = when
            self.rejected_candidates = engine.rejected_candidates
        if engine.decision is not None:
            self.frozen = True

    def select(self, decision):
        if self.receipt is not None:
            return copy.deepcopy(self.receipt)
        crossing = None if decision is None else decision["crossing_time_s"]
        confirmation = None if decision is None else decision["confirmation_time_s"]
        lower = (None if decision is None else max(self.arm_time, decision["lookback_onset_time_s"],
                    crossing - self.config["onset_lookback_s"],
                    self.last_rejection_time if self.last_rejection_time is not None else self.arm_time))
        rows = [] if lower is None else [row for row in self.rows if lower <= row[0] <= confirmation
            and (self.last_rejection_time is None or row[0] > self.last_rejection_time)]
        edges = [row for row in rows if row[5]]
        reason = ("filtered_not_accepted" if decision is None else
                  "raw_reference_contains_marker_estimates" if not self.reference_original_valid else
                  "raw_association_history_truncated" if self.evicted_through_time is not None
                    and self.evicted_through_time >= lower else
                  "raw_window_has_unverifiable_samples" if any(not row[3] for row in rows) else
                  "multiple_raw_threshold_crossings" if len(edges) > 1 else
                  "no_associated_raw_threshold_crossing" if not edges else
                  "raw_crossing_has_unverifiable_amplitude" if not edges[0][11] else None)
        selected = edges[0] if reason is None else None
        result = {"version": 1, "policy": HYBRID_POSITION_POLICY,
            "control_detector": "filtered", "control_position_policy": FILTERED_POSITION_POLICY,
            "actual_source": "raw_threshold_crossing" if selected else "filtered_threshold_crossing_fallback",
            "fallback_reason": reason, "raw_crossing_count": len(edges), "ambiguous": len(edges) > 1,
            "selected_time_s": selected[0] if selected else crossing,
            "selected_sample": list(selected[2]) if selected else
                None if decision is None else list(decision["crossing_identity"]),
            "selected_raw_level_g": math.sqrt(selected[4]) * self.count_g if selected else None,
            "selected_raw_sensor_counts": list(selected[8]) if selected else None,
            "selected_filtered_level_g": None if selected or decision is None else decision["onset_level_g"],
            "raw_previous_time_s": selected[7] if selected else None,
            "raw_previous_sample": list(selected[6]) if selected else None,
            "raw_previous_level_g": math.sqrt(selected[9]) * self.count_g if selected else None,
            "raw_previous_sensor_counts": list(selected[10]) if selected else None,
            "filtered_crossing_time_s": crossing, "filtered_confirmation_time_s": confirmation,
            "filtered_crossing_sample": None if decision is None else list(decision["crossing_identity"]),
            "arm_time_s": self.arm_time,
            "raw_minus_filtered_time_s": None if selected is None else selected[0] - crossing,
            "association_window_s": [lower, confirmation], "association_lookback_limit_s": self.config["onset_lookback_s"],
            "association_rule": "unique_observed_raw_rise_within_accepted_filtered_contiguous_gate_region_through_confirmation",
            "rejected_candidate_cutoff_s": self.last_rejection_time, "rejected_cutoff_exclusive": True,
            "observed_rows_in_window": len(rows), "history_limit_rows": MAX_HYBRID_ROWS,
            "marker_rows_with_proven_above_code_state": sum(row[12] for row in rows),
            "marker_crossings_ineligible_for_position": sum(not row[11] for row in edges),
            "raw_threshold_g": self.threshold_g, "raw_threshold_source": "saved_compatible_raw_xyz_profile",
            "raw_metric": "unweighted_xyz_squared_counts_about_arithmetic_quiet_baseline",
            "raw_reference": {"method": "quiet_reference_arithmetic_mean_counts_v1",
                "baseline_sensor_counts": list(self.baseline_counts), "count_g": self.count_g,
                "admission": "shared_filtered_qualified_prearm_window",
                "original_reference_values_verified": self.reference_original_valid,
                "standalone_raw_lifecycle_equivalent": False},
            "raw_profile": copy.deepcopy(self.profile),
            "marker_policy": "marker_cannot_supply_position_or_amplitude;native_endpoint_bound_may_prove_continuation_above",
            "native_clip_values": None if self.clip_values is None else list(self.clip_values),
            "time_source": "original_online_packet_snapshot", "physical_position_verified": False}
        self.receipt = result
        return copy.deepcopy(result)


def validate_position_selection(selection, record):
    """Check public receipt semantics; native capture validation also replays it."""
    if filtered_position_policy(record) != HYBRID_POSITION_POLICY:
        if selection is not None or record.get("requested_detector") == "hybrid":
            raise ValueError("hybrid receipt requires its explicit position policy")
        return None
    finite = lambda value: type(value) in (int, float) and math.isfinite(value)
    if (not isinstance(selection, dict) or selection.get("version") != 1
            or selection.get("policy") != HYBRID_POSITION_POLICY
            or selection.get("control_detector") != "filtered"
            or selection.get("control_position_policy") != FILTERED_POSITION_POLICY
            or record.get("detector_type") != "filtered" or record.get("requested_detector") != "hybrid"
            or record.get("control_detector") != "filtered"
            or selection.get("actual_source") not in ("raw_threshold_crossing", "filtered_threshold_crossing_fallback")
            or not finite(selection.get("raw_threshold_g")) or selection["raw_threshold_g"] <= 0
            or selection.get("raw_threshold_source") != "saved_compatible_raw_xyz_profile"
            or type(selection.get("raw_crossing_count")) is not int or selection["raw_crossing_count"] < 0
            or selection.get("ambiguous") is not (selection["raw_crossing_count"] > 1)
            or selection.get("history_limit_rows") != MAX_HYBRID_ROWS
            or not isinstance(selection.get("raw_reference"), dict)):
        raise ValueError("invalid HYBRID position receipt")
    profile = selection.get("raw_profile")
    reference = selection["raw_reference"]
    if (not isinstance(profile, dict) or profile.get("threshold_g") != selection["raw_threshold_g"]
            or profile.get("source") != "saved_compatible_raw_xyz_profile"
            or profile.get("detector_type") != "raw" or profile.get("impact_metric") != "xyz_magnitude"
            or not isinstance(profile.get("model_context"), dict)
            or reference.get("method") != "quiet_reference_arithmetic_mean_counts_v1"
            or reference.get("admission") != "shared_filtered_qualified_prearm_window"
            or reference.get("standalone_raw_lifecycle_equivalent") is not False
            or type(reference.get("original_reference_values_verified")) is not bool
            or selection.get("time_source") != "original_online_packet_snapshot"):
        raise ValueError("HYBRID position receipt contradicts its saved RAW XYZ threshold/reference source")
    if record.get("result") != "impact":
        if selection.get("fallback_reason") != "filtered_not_accepted" or selection.get("selected_time_s") is not None:
            raise ValueError("unaccepted HYBRID home cannot claim a position")
        return selection
    crossing = record.get("trigger_crossing_time_s", record.get("crossing_time_s"))
    confirmation = record.get("trigger_confirmation_time_s", record.get("confirmation_time_s"))
    bounds = selection.get("association_window_s")
    if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(finite(value) for value in (*bounds, crossing, confirmation, selection.get("selected_time_s"), record.get("arm_time_s")))
            or not record.get("arm_time_s") <= bounds[0] <= crossing <= confirmation == bounds[1]
            or selection.get("filtered_crossing_time_s") != crossing
            or selection.get("filtered_confirmation_time_s") != confirmation
            or not bounds[0] <= selection["selected_time_s"] <= confirmation
            or _stamp(selection.get("selected_sample")) is None):
        raise ValueError("HYBRID position is outside its accepted causal candidate")
    if selection["actual_source"] == "raw_threshold_crossing":
        if (selection.get("fallback_reason") is not None or selection["raw_crossing_count"] != 1
                or not finite(selection.get("selected_raw_level_g"))
                or selection["selected_raw_level_g"] + 1e-12 < selection["raw_threshold_g"]
                or selection.get("selected_filtered_level_g") is not None
                or _stamp(selection.get("raw_previous_sample")) is None
                or not finite(selection.get("raw_previous_level_g"))
                or not selection["raw_previous_level_g"] < selection["raw_threshold_g"]
                or not finite(selection.get("raw_previous_time_s"))
                or not selection["raw_previous_time_s"] < selection["selected_time_s"]
                or selection.get("raw_minus_filtered_time_s") != selection["selected_time_s"] - crossing):
            raise ValueError("HYBRID RAW position lacks its unique observed threshold bracket")
        reference = selection["raw_reference"]
        baseline = _xyz(reference.get("baseline_sensor_counts"))
        scale = reference.get("count_g")
        if not finite(scale) or scale <= 0 or reference.get("original_reference_values_verified") is not True:
            raise ValueError("HYBRID RAW position lacks its original arithmetic reference")
        level2 = []
        for key in ("raw_previous_sensor_counts", "selected_raw_sensor_counts"):
            values = _xyz(selection.get(key))
            if any(type(value) is not int or value == 32767 for value in values):
                raise ValueError("HYBRID cannot substitute marker amplitudes for original RAW samples")
            level2.append(sum((value - base) ** 2 for value, base in zip(values, baseline)))
        if not level2[0] < (selection["raw_threshold_g"] / scale) ** 2 <= level2[1]:
            raise ValueError("HYBRID RAW receipt does not cross its exact squared count-space threshold")
        if any(not math.isclose(selection[key], math.sqrt(value) * scale, rel_tol=1e-12, abs_tol=1e-12)
               for key, value in zip(("raw_previous_level_g", "selected_raw_level_g"), level2)):
            raise ValueError("HYBRID RAW amplitudes differ from original count-space norms")
    elif (selection.get("fallback_reason") not in (
            "raw_reference_contains_marker_estimates", "raw_association_history_truncated",
            "raw_window_has_unverifiable_samples", "multiple_raw_threshold_crossings", "no_associated_raw_threshold_crossing",
            "raw_crossing_has_unverifiable_amplitude")
            or selection["selected_time_s"] != crossing
            or _stamp(selection["selected_sample"]) != _stamp(record.get("crossing_sample"))
            or selection.get("selected_raw_level_g") is not None or selection.get("raw_minus_filtered_time_s") is not None):
        raise ValueError("HYBRID fallback must retain the current FILTERED crossing")
    return selection


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
    validate_detector_identity(record.get("detector_version"), record.get("detector_config"), record)
    return True


def validate_filtered_capture_prefix(capture, record=None, check_running=None):
    """Prove a native confirmation from original raw rows before saturation.

    Whole-capture integrity remains false. The private bounded original batches
    are checked in full, including faults after the censor boundary, and never
    become chart payload or persisted raw scratch. Replay uses original online
    sensor-order values/times; exported sample times may have been remapped.
    """
    detector = capture.get("detector") or {}
    summary = capture.get("response_censoring")
    integrity = capture.get("raw_integrity") or {}
    decoding = capture.get("native_clip_decode")
    decoded = (isinstance(decoding, dict) and detector.get("detector_type") == "filtered"
               and detector.get("backend") == "beacon_raw" and summary is None)
    if summary is None and not decoded:
        if detector.get("position_policy") == HYBRID_POSITION_POLICY:
            raise ValueError("HYBRID requires retained original native count-space provenance")
        if detector.get("detector_type") == "filtered" and integrity.get("clipped_axis_values", 0):
            raise ValueError("native marker rows require their frozen decoder provenance")
        if (detector.get("detector_type") == "filtered" and integrity.get("raw_samples_validated") is False):
            raise ValueError("invalid FILTERED capture has no eligible censor proof")
        if record is not None and record.get("response_censored"):
            raise ValueError("claimed censored record has no capture proof")
        return None
    finite = lambda v: type(v) in (int, float) and math.isfinite(v)
    if (capture.get("fault") or capture.get("error") or capture.get("dropped", 0)
            or (not decoded and not isinstance(summary, dict)) or detector.get("detector_type") != "filtered"
            or detector.get("backend") != "beacon_raw" or detector.get("result") != "impact"
            or detector.get("axis") not in ("x", "y") or type(detector.get("attempt_count")) is not int
            or detector["attempt_count"] != 1
            or integrity.get("raw_samples_validated") is not bool(decoded)
            or (not decoded and (type(summary.get("version")) is not int or summary["version"] != 1
                or summary.get("kind") != "native_saturation_after_confirmation"
                or summary.get("cause") not in ("native_clip_sentinel", "axis_full_scale")))):
        raise ValueError("only post-confirmation native saturation permits a censored prefix")
    config = validate_detector_identity(detector.get("detector_version"), detector.get("detector_config"), detector)
    samples, sources = capture.get("samples"), capture.get("_native_source_batches")
    mapping, reference = capture.get("_native_mapping"), capture.get("_native_filter_reference")
    clips = native_clip_limits(decoding.get("clip_values")) if decoded else None
    full_scale = clips[0] * decoding.get("count_g", 0.) if decoded else summary.get("full_scale_g")
    if (not isinstance(samples, (list, tuple)) or not 2 <= len(samples) <= 200000
            or not isinstance(sources, (list, tuple)) or not 1 <= len(sources) <= 100000
            or not isinstance(mapping, (list, tuple)) or len(mapping) != 3
            or any(not isinstance(pair, (list, tuple)) or len(pair) != 2 or pair[0] not in (0, 1, 2)
                   or pair[1] not in (-1, 1) for pair in mapping)
            or len({pair[0] for pair in mapping}) != 3 or not isinstance(reference, dict)
            or not finite(full_scale) or full_scale <= 0):
        raise ValueError("censored capture lacks bounded original native data/reference")
    rows, identities, online_times = [], [], []
    hybrid_counts, hybrid_original_valid, hybrid_original_counts = [], [], []
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
    last_decoded, marker_first, marker_last = None, None, None
    counts = {"clipped_axis_values": 0, "saturated_axis_values": 0, "nonfinite_axis_values": 0}
    last, last_clock, original_last = None, None, None
    for source_index, source in enumerate(sources):
        if check_running is not None:
            check_running()
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
        original_raw = raw
        if decoded:
            seed = _xyz(source.get("decode_previous_raw"))
            if (decoding.get("policy") != NATIVE_CLIP_POLICY
                    or scope.get("decoder_policy") != NATIVE_CLIP_POLICY
                    or native_clip_limits(source.get("clip_values")) != clips
                    or native_clip_limits(scope.get("clip_values")) != clips
                    or 32767 in seed
                    or (last_decoded is not None and seed != last_decoded)
                    or (last_decoded is None and seed != _xyz(scope.get("decode_previous_raw")))):
                raise ValueError("native decoder limits/seed/epoch provenance changed")
            if check_running is None:
                raw = decode_native_values(raw, seed, clips)
            else:
                decoded_rows, previous_decoded = [], seed
                for offset in range(0, len(raw), 768):
                    check_running()
                    chunk = decode_native_values(raw[offset:offset + 768], previous_decoded, clips)
                    previous_decoded = tuple(chunk[-3:])
                    decoded_rows.extend(chunk)
                raw = decoded_rows
            last_decoded = tuple(raw[-3:])
        first = source.get("first_sample_position")
        if (type(first) is not int or not 0 <= first < count
                or type(source.get("admission_epoch")) is not int
                or source["admission_epoch"] != scope["admission_epoch"]
                or (source_index and first != 0)):
            raise ValueError("native response changed its admitted sample scope")
        start_clock, delta = source.get("start_clock"), source.get("delta_clock")
        start, end, received = (source.get(key) for key in ("start_time_s", "end_time_s", "received_time_s"))
        scale = source.get("count_g")
        if decoded and scale != decoding.get("count_g"):
            raise ValueError("native decoder scale changed")
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
            if check_running is not None and position % 256 == 0:
                check_running()
            when = start + position * period
            if last is not None and (when <= last or when - last > 5. * period + .000003):
                raise ValueError("missing/repeated response samples cannot be censored"
                                 " (packet=%d, position=%d, previous=%.9f, next=%.9f, gap=%.9f, limit=%.9f, epoch=%d)"
                                 % (source_index, position, last, when, when - last,
                                    5. * period + .000003, scope["admission_epoch"]))
            last = when
            xyz = tuple(raw[3 * position:3 * position + 3])
            original_xyz = tuple(original_raw[3 * position:3 * position + 3])
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in xyz):
                raise ValueError("nonfinite native response cannot be censored")
            stamp = (start_clock, delta, position, count)
            invalid = None
            for axis, value in enumerate(original_xyz):
                counts["clipped_axis_values"] += value == 32767
                counts["saturated_axis_values"] += abs(value * scale) >= full_scale
                if decoded and value == 32767:
                    marker_first, marker_last = marker_first or stamp, stamp
                if not decoded and invalid is None and value == 32767:
                    invalid = axis
            if invalid is not None and found is None:
                found = (len(rows), stamp, when, invalid, xyz[invalid])
            if found is None:
                expected = (when,) + tuple(xyz[index] * sign * scale for index, sign in mapping)
                if len(rows) >= len(samples) or tuple(samples[len(rows)]) != expected:
                    raise ValueError("exported valid prefix differs from original native rows")
            rows.append(tuple(value * scale for value in xyz))
            hybrid_counts.append(xyz)
            hybrid_original_valid.append(32767 not in original_xyz)
            hybrid_original_counts.append(original_xyz)
            identities.append(stamp)
            a, b = source.get("filter_start_time_s"), source.get("filter_end_time_s")
            online_times.append(None if not finite(a) or not finite(b) else a + (b - a) * position / (count - 1))
    if (not decoded and found is None) or any(type(integrity.get(key)) is not int or integrity[key] != value for key, value in counts.items()):
        raise ValueError("whole-capture integrity does not match original native values")
    index = len(rows) if decoded else found[0]
    if decoded:
        if (len(samples) != index or decoding.get("marker_axis_values") != counts["clipped_axis_values"]
                or decoding.get("first_sample") != (None if marker_first is None else list(marker_first))
                or decoding.get("last_sample") != (None if marker_last is None else list(marker_last))
                or decoding.get("sign_inferred") is not bool(counts["clipped_axis_values"])
                or decoding.get("true_amplitude_unknown") is not bool(counts["clipped_axis_values"])):
            raise ValueError("native marker substitution summary differs from original rows")
    else:
        index, stamp, when, axis, value = found
    if not decoded and (index != len(samples) or _stamp(summary.get("first_invalid_sample")) != stamp
            or summary.get("first_invalid_sample_index") != index or summary.get("first_invalid_time_s") != when
            or summary.get("sensor_axis") != "xyz"[axis] or summary.get("raw_value") != value
            or summary.get("cause") != "native_clip_sentinel"
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
    engine = CausalImpactDetector(threshold, reference["offset_g"], reference["noise_sigma_g"], reference["weights_xyz"],
                                  config, filtered_position_policy(detector))
    start_index = positions["first_armed"]
    if online_times[start_index] is None:
        raise ValueError("censored prefix lacks original online filter timing")
    engine.previous_time = online_times[start_index] - period
    hybrid = None
    if filtered_position_policy(detector) == HYBRID_POSITION_POLICY:
        supplied = validate_position_selection(detector.get("position_selection"), detector)
        raw_baseline = tuple(sum(row[axis] for row in hybrid_counts[ref_indices[0]:ref_indices[1] + 1])
                             / detector["reference_sample_count"] for axis in range(3))
        hybrid = AssociatedRawPosition(supplied["raw_threshold_g"], raw_baseline, sources[0]["count_g"],
            config, detector["arm_time_s"], supplied.get("raw_profile"),
            reference_original_valid=all(hybrid_original_valid[ref_indices[0]:ref_indices[1] + 1]), clip_values=clips)
    decision = None
    for i in range(start_index, index):
        if check_running is not None and i % 256 == 0:
            check_running()
        if online_times[i] is None:
            raise ValueError("censored prefix lost an online packet timestamp")
        decision = engine.feed(online_times[i], rows[i], i)
        if hybrid is not None:
            original = hybrid_counts[i] if hybrid_original_valid[i] else None
            if original is None:
                # Original marker positions must remain separate from decoded
                # endpoints; finite channels retain their exact source codes.
                original = hybrid_original_counts[i]
            hybrid.observe(online_times[i], hybrid_counts[i], identities[i], engine, hybrid_original_valid[i], original)
        if decision is not None:
            break
    if decision is None or any(decision[event + "_identity"] != positions[event] for event in ("onset", "crossing", "confirmation")):
        raise ValueError("retained valid prefix does not reproduce actual filtered confirmation")
    for event in ("onset", "crossing", "confirmation"):
        trigger_time = detector.get("trigger_" + event + "_time_s")
        if trigger_time != decision[event + "_time_s"]:
            raise ValueError("original trigger timestamp differs from online prefix replay")
    if hybrid is not None:
        mapped = dict(decision, crossing_identity=identities[decision["crossing_identity"]])
        expected_selection = hybrid.select(mapped)
        expected_selection["raw_reference"].update(sample_count=detector["reference_sample_count"],
            sample_bounds=detector["reference_sample_bounds"], interval_s=detector["trigger_reference_interval_s"])
        canonical = lambda value: json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":"))
        if canonical(expected_selection) != canonical(detector["position_selection"]):
            raise ValueError("HYBRID RAW position differs from exact original count-space candidate replay")
        if record is not None:
            validate_position_selection(record.get("position_selection"), record)
            if (canonical(record["position_selection"]) != canonical(expected_selection)
                    or record.get("returned_trigger_time_s") != expected_selection["selected_time_s"]):
                raise ValueError("HYBRID returned position differs from its frozen accepted receipt")
    if not decoded and (summary.get("confirmation_sample_index") != positions["confirmation"] or _stamp(summary.get("confirmation_sample")) != identities[positions["confirmation"]] \
            or summary.get("confirmation_time_s") != detector["confirmation_time_s"]):
        raise ValueError("censor summary changed its retained confirmation")
    if not decoded and summary.get("first_invalid_original_time_s") != online_times[index]:
        raise ValueError("censor summary lost its original saturation timestamp")
    peak = max(math.sqrt(sum((v - b) ** 2 for v, b in zip(row[1:], baseline))) for row in samples)
    if not decoded and summary.get("observed_valid_prefix_peak_g") != peak:
        raise ValueError("censored prefix peak is not an observed valid value")
    result = (None if decoded else dict(summary, decision_prefix_validated=True,
                  first_invalid_sample=list(stamp), confirmation_sample=list(identities[positions["confirmation"]])))
    if record is not None:
        if (record.get("result") != "impact" or record.get("detector_type") != "filtered"
                or record.get("detector_config") != config
                or filtered_position_policy(record) != filtered_position_policy(detector)
                or type(record.get("detector_version")) is not int or record["detector_version"] != detector["detector_version"]
                or record.get("axis") != detector.get("axis") or record.get("arm_time_s") != detector.get("arm_time_s")
                or record.get("arm_clock") != arm_clock
                or _stamp(record.get("onset_sample")) != _stamp(detector.get("onset_sample"))
                or record.get("trigger_onset_time_s", record.get("onset_time_s")) != detector.get("trigger_onset_time_s")
                or not math.isclose(record.get("workflow_threshold_g", record.get("threshold_g", 0)), threshold, rel_tol=1e-12, abs_tol=1e-12)
                or (not decoded and record.get("impact_peak_g") is not None)
                or (decoded and (not finite(record.get("impact_peak_g")) or record.get("response_censored")))
                or (decoded and counts["clipped_axis_values"] and record.get("native_clip_decode") != decoding)
                or record.get("trigger_confirmation_time_s") != detector.get("trigger_confirmation_time_s")):
            raise ValueError("censored prefix does not belong to current filtered record")
        existing = record.get("response_censoring")
        if existing is not None and existing != result:
            raise ValueError("censored record differs from validated capture proof")
    return result


def _compare_filter_variant(capture, raw_threshold_g, filtered_threshold_g, filter_config=None):
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
            if capture.get("native_clip_decode") is not None:
                raw = decode_native_values(raw, _xyz(source.get("decode_previous_raw")),
                                           native_clip_limits(source.get("clip_values")))
            count = len(raw) // 3
            for position in range(source.get("first_sample_position", 0), count):
                if len(native_inputs) >= len(samples):
                    break
                a, b = source.get("filter_start_time_s"), source.get("filter_end_time_s")
                online = (samples[len(native_inputs)][0] if a is None or b is None else
                          a + (b - a) * position / (count - 1))
                native_inputs.append((online, tuple(v * source["count_g"] for v in raw[position * 3:position * 3 + 3])))
    engine_reference = native_reference if native_inputs else reference
    policy = filtered_position_policy(detector)
    engine = CausalImpactDetector(filtered_threshold_g, engine_reference["offset_g"],
                                  engine_reference["noise_sigma_g"], engine_reference["weights_xyz"], config, policy)
    diagnostic = CausalImpactDetector(filtered_threshold_g, reference["offset_g"],
                                      reference["noise_sigma_g"], reference["weights_xyz"], config, policy)
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
        if not containing:
            previous = max((s for s in sources if s["filter_end_time_s"] < original),
                           key=lambda s: s["filter_end_time_s"], default=None)
            following = min((s for s in sources if s["filter_start_time_s"] > original),
                            key=lambda s: s["filter_start_time_s"], default=None)
            if previous is not None and following is not None:
                # A host cutoff between packets has no captured sensor row.
                # Keep its presentation monotone between surrounding mapped
                # identities; eligibility always uses original times above.
                fraction = ((original - previous["filter_end_time_s"]) /
                            (following["filter_start_time_s"] - previous["filter_end_time_s"]))
                return previous["end_time_s"] + fraction * (following["start_time_s"] - previous["end_time_s"])
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
        for event in ("onset", "crossing", "confirmation", "lookback_onset"):
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
                                                  reference["noise_sigma_g"], reference["weights_xyz"], config, policy)
                diagnostic_reset = True
            if diagnostic.previous_time is None:
                diagnostic.previous_time = when - period
            diagnostic.feed(when, row[1:])
            display, decision = diagnostic, None
        display_xyz = ([display.filtered_xyz[index] * (sign if signal_mode(config) == "signed" else 1)
                        for index, sign in native_mapping]
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
            "decision_cutoff_display_is_approximate": bool(native_inputs and cutoff is not None and
                                                            cutoff_reason != "first_invalid_sample"),
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
            "reference": reference, "filter_config": config, "position_policy": policy,
            "actual_onset_time_s": detector.get("onset_time_s"),
            "actual_crossing_time_s": detector.get("crossing_time_s"),
            "actual_confirmation_time_s": detector.get("confirmation_time_s"),
            "moving_noise": None if pre_stop_diagnostics is None else pre_stop_diagnostics["moving_noise"],
            "physical_accuracy_verified": False, "counterfactual": True}


def compare_filter_capture(capture, raw_threshold_g, filtered_threshold_g, filter_config=None):
    """Replay signed-v1 and rectified-v2 on identical original valid samples.

    The alternate uses the same numerical threshold only to compare signal
    processing. Its threshold is not independently calibrated for that metric.
    """
    if not isinstance(capture, dict):
        raise ValueError("comparison requires a complete valid capture")
    detector = capture.get("detector") or {}
    config = detector_config(filter_config or detector.get("detector_config"))
    actual_config = (validate_detector_identity(detector.get("detector_version"), detector.get("detector_config"), detector)
                     if detector.get("detector_type") == "filtered" else None)
    if actual_config is not None and config != actual_config:
        raise ValueError("comparison cannot replace the actual filtered configuration")
    if actual_config is not None and (type(filtered_threshold_g) not in (int, float)
            or not math.isfinite(filtered_threshold_g)
            or type(detector.get("threshold_g")) not in (int, float)
            or not math.isfinite(detector["threshold_g"])
            or not math.isclose(filtered_threshold_g, detector["threshold_g"], rel_tol=1e-12, abs_tol=1e-12)):
        raise ValueError("comparison cannot replace the actual filtered threshold")
    selected_key = "signed_v1" if config["version"] == 1 else "rectified_v2"
    actual_key = None if actual_config is None else ("signed_v1" if actual_config["version"] == 1 else "rectified_v2")
    alternate = dict(config)
    if config["version"] == 1:
        alternate.update(version=2, signal_mode="rectified")
        alternate_key = "rectified_v2"
    else:
        alternate.pop("signal_mode")
        alternate["version"] = 1
        alternate_key = "signed_v1"
    selected = _compare_filter_variant(capture, raw_threshold_g, filtered_threshold_g, config)
    other = _compare_filter_variant(capture, raw_threshold_g, filtered_threshold_g, alternate)
    variants = {}
    for key, analysis in ((selected_key, selected), (alternate_key, other)):
        current = analysis["filter_config"]
        variants[key] = {"detector_version": current["version"], "signal_mode": signal_mode(current),
                         "position_policy": analysis["position_policy"],
                         "config": current, "trace": analysis["filtered_trace"],
                         "level": analysis["filtered_level"], "weighted_level": analysis["weighted_level"],
                         "decision": analysis["filtered_decision"], "reference": analysis["reference"],
                         "threshold_g": filtered_threshold_g,
                         "threshold_source": "actual_active_detector" if key == actual_key else
                                             "same_numeric_threshold_diagnostic_not_calibrated",
                         "pre_arm_warmup_s": analysis["pre_arm_warmup_s"],
                         "moving_noise": analysis["moving_noise"],
                         "decision_cutoff_time_s": analysis["decision_cutoff_time_s"],
                         "decision_cutoff_original_time_s": analysis["decision_cutoff_original_time_s"],
                         "decision_cutoff_reason": analysis["decision_cutoff_reason"]}
    selected.update(version=2, variants=variants, selected_variant=selected_key, actual_variant=actual_key,
                    actual_detector_version=None if actual_config is None else actual_config["version"],
                    actual_position_policy=None if actual_config is None else filtered_position_policy(detector),
                    actual_signal_mode=None if actual_config is None else signal_mode(actual_config))
    return selected
