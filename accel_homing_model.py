# Saved accelerometer-homing models. Part of the GPLv3 homing extension.
"""Named SAVE_CONFIG data, separate from editable homing thresholds."""
import json
import math
import re

try:
    from .accel_homing_filters import validate_detector_identity
except ImportError:
    from accel_homing_filters import validate_detector_identity


def validate_paired_certificates(certificates):
    if not isinstance(certificates, dict) or len(certificates) > 2:
        raise ValueError("invalid paired calibration certificates")
    for pair_id, certificate in certificates.items():
        if (not re.fullmatch(r"[a-f0-9]{32}", pair_id) or not isinstance(certificate, dict)
                or set(certificate) != {"pair_id", "raw", "filtered", "total_valid_checks"}
                or certificate["pair_id"] != pair_id or certificate["total_valid_checks"] != 10):
            raise ValueError("incomplete paired calibration certificate")
        proofs = [certificate[kind] for kind in ("raw", "filtered")]
        for kind, proof in zip(("raw", "filtered"), proofs):
            if not isinstance(proof, dict):
                raise ValueError("missing paired calibration proof")
            if set(proof) != {"pair_id", "stage_id", "detector_type", "valid_checks", "discovery_excluded",
                    "discovery_attempt", "validation_attempts", "tested_threshold_g", "saved_threshold_g",
                    "threshold_margin_g", "saved_threshold_tested", "repeatability_spread_mm", "model_context",
                    "sensor_context", "run_id", "detector_config"}:
                raise ValueError("unknown or incomplete paired calibration proof")
            attempts = proof.get("validation_attempts")
            discovery = proof.get("discovery_attempt")
            tested, saved = proof.get("tested_threshold_g"), proof.get("saved_threshold_g")
            if (proof.get("pair_id") != pair_id or proof.get("detector_type") != kind
                    or proof.get("valid_checks") != 5 or proof.get("discovery_excluded") is not True
                    or not re.fullmatch(r"[a-f0-9]{32}", str(proof.get("stage_id", "")))
                    or type(discovery) is not int or discovery < 1
                    or not isinstance(attempts, list) or attempts != list(range(discovery + 1, discovery + 6))
                    or any(type(value) not in (int, float) or not math.isfinite(value) or value < .5 for value in (tested, saved))
                    or not math.isclose(saved - tested, .5, rel_tol=1e-12, abs_tol=0.)
                    or proof.get("threshold_margin_g") != .5 or proof.get("saved_threshold_tested") is not False
                    or type(proof.get("repeatability_spread_mm")) not in (int, float)
                    or not 0 <= proof["repeatability_spread_mm"] <= 2.
                    or not re.fullmatch(r"[a-f0-9]{32}", str(proof.get("run_id", "")))
                    or not isinstance(proof.get("model_context"), dict)
                    or set(proof["model_context"]) != {"sensor", "backend", "sensor_axes_map", "geometry", "homing_speed_mm_s", "homing_accel_mm_s2", "impact_sign"}
                    or proof["model_context"].get("backend") != "beacon_raw"
                    or not isinstance(proof.get("sensor_context"), dict) or not proof["sensor_context"]):
                raise ValueError("paired calibration requires five qualified same-candidate checks and exact margin")
        if (proofs[0]["stage_id"] == proofs[1]["stage_id"]
                or proofs[0]["validation_attempts"][-1] + 1 != proofs[1]["discovery_attempt"]
                or proofs[0]["model_context"] != proofs[1]["model_context"]
                or proofs[0]["sensor_context"] != proofs[1]["sensor_context"]
                or proofs[0]["run_id"] != proofs[1]["run_id"]):
            raise ValueError("paired calibration lineage/context differs between its two series")
        if (proofs[0]["detector_config"] is not None
                or not isinstance(proofs[1]["detector_config"], dict)
                or proofs[1]["detector_config"].get("version") != 2
                or proofs[1]["detector_config"].get("signal_mode") != "rectified"):
            raise ValueError("paired native calibration requires RAW XYZ magnitude and rectified FILTERED v2")
    return certificates


def _validate_detector_metadata(metadata, certificates=None):
    own_certificates = metadata.get("paired_calibrations", {})
    validate_paired_certificates(own_certificates)
    certificates = dict(certificates or {}, **own_certificates)
    kind = metadata.get("detector_type", "raw")
    if kind not in ("raw", "filtered"):
        raise ValueError("unknown saved detector type")
    if kind == "filtered":
        if metadata.get("impact_metric") != "xyz_magnitude":
            raise ValueError("filtered model must use the XYZ vector metric")
        validate_detector_identity(metadata.get("detector_version"), metadata.get("detector_config"), metadata)
    proof = metadata.get("paired_qualification")
    if ("qualification_successes" in metadata and metadata["qualification_successes"] not in (5, 10)) or (metadata.get("qualification_successes") == 5 and proof is None):
        raise ValueError("five-check profile requires its complete paired qualification provenance")
    if proof is not None:
        if (not isinstance(proof, dict) or proof.get("pair_id") not in certificates
                or certificates[proof["pair_id"]].get(kind) != proof
                or metadata.get("tested_threshold_g") != proof.get("tested_threshold_g")
                or metadata.get("saved_threshold_g") != proof.get("saved_threshold_g")
                or metadata.get("qualification_successes") != 5
                or metadata.get("run_id") != proof.get("run_id")
                or metadata.get("detector_config") != proof.get("detector_config")
                or metadata.get("calibration_sensor_context") != proof.get("sensor_context")
                or metadata.get("repeatability_spread_mm") != proof.get("repeatability_spread_mm")
                or metadata.get("threshold_margin_g") != .5 or metadata.get("saved_threshold_tested") is not False
                or any(metadata.get(key) != value for key, value in proof.get("model_context", {}).items())):
            raise ValueError("five-check profile lacks its complete same-pair calibration certificate")
        if (metadata.get("minimum_impact_travel_mm") != 20. or metadata.get("discovery_impact_required") is not True
                or metadata.get("impact_metric") != "xyz_magnitude"
                or metadata.get("native_polarity_policy") != "absolute_after_reference_subtraction_v1"
                or any(not isinstance(metadata.get(key), list) or len(metadata[key]) != 5 for key in (
                    "onset_g", "impact_peak_g", "amplitude_limited", "excess_amplitude_unknown", "native_clip_decode",
                    "response_censored", "observed_valid_prefix_peak_g"))
                or (kind == "filtered" and (metadata.get("filter_sensor_context") != proof["sensor_context"]
                    or not isinstance(metadata.get("filter_reference_summaries"), list)
                    or len(metadata["filter_reference_summaries"]) != 5))):
            raise ValueError("paired profile metadata does not describe its five qualified checks")
    profiles = metadata.get("detector_profiles", {})
    if not isinstance(profiles, dict) or set(profiles) - {"raw", "filtered"}:
        raise ValueError("invalid saved detector profiles")
    for profile_kind, profile in profiles.items():
        if (not isinstance(profile, dict) or set(profile) != {"threshold_g", "metadata"}
                or type(profile["threshold_g"]) not in (int, float)
                or not math.isfinite(profile["threshold_g"]) or profile["threshold_g"] < .5
                or not isinstance(profile["metadata"], dict)
                or "detector_profiles" in profile["metadata"]
                or profile["metadata"].get("detector_type", "raw") != profile_kind):
            raise ValueError("invalid saved %s detector profile" % profile_kind)
        _validate_detector_metadata(profile["metadata"], certificates)
        if profile["metadata"].get("paired_qualification") is not None and profile["threshold_g"] != profile["metadata"]["paired_qualification"]["saved_threshold_g"]:
            raise ValueError("paired profile threshold differs from its qualified saved margin")
    if proof is not None and profiles:
        selected = profiles.get(kind, {}).get("metadata")
        if not isinstance(selected, dict) or any(metadata.get(key) != value for key, value in selected.items()):
            raise ValueError("selected paired profile differs from its nested qualification")


def _finite_metadata(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_metadata(item) for item in value.values())
    if isinstance(value, list):
        return all(_finite_metadata(item) for item in value)
    return True


class AccelHomingModel:
    def __init__(self, config):
        self.name = config.get_name().split(" ", 1)[1]
        self.thresholds = {}
        self.successes = {}
        self.metadata = {}
        self.version = config.getint("version", 1, minval=1, maxval=3)
        for axis in "xy":
            threshold = config.getfloat(axis + "_threshold", None,
                                        minval=0.5)
            self.successes[axis] = config.getint(axis + "_successes", 0,
                                                minval=0, maxval=10)
            encoded = config.get(axis + "_metadata", "{}")
            try:
                metadata = json.loads(encoded)
                if not isinstance(metadata, dict):
                    raise ValueError("metadata must be an object")
                if not _finite_metadata(metadata):
                    raise ValueError("metadata numbers must be finite")
                _validate_detector_metadata(metadata)
                if self.version < 3 and (metadata.get("paired_calibrations") or metadata.get("paired_qualification")
                        or any(profile["metadata"].get("paired_qualification") for profile in metadata.get("detector_profiles", {}).values())):
                    raise ValueError("paired five-check profiles require model schema version 3")
                if metadata.get("detector_type", "raw") == "filtered" and self.version < 2:
                    raise ValueError("active filtered profiles require model schema version 2")
            except (ValueError, TypeError) as exc:
                raise config.error("Invalid %s model metadata: %s" % (axis, exc))
            self.metadata[axis] = metadata
            if threshold is not None:
                if not math.isfinite(threshold):
                    raise config.error("Saved %s homing threshold must be finite" % axis)
                paired = metadata.get("paired_qualification")
                required = 5 if self.version == 3 and paired is not None else 10
                if self.successes[axis] != required:
                    raise config.error("Saved %s homing model needs %s successes" % (axis, "five" if required == 5 else "ten"))
                if paired is not None and threshold != paired["saved_threshold_g"]:
                    raise config.error("Saved paired threshold differs from its qualified profile")
                self.thresholds[axis] = threshold

    def get_status(self, eventtime):
        return {"name": self.name, "version": self.version,
                "threshold_g": dict(self.thresholds),
                "successes": dict(self.successes),
                "metadata": {a: dict(m) for a, m in self.metadata.items()}}


def load_config_prefix(config):
    return AccelHomingModel(config)
