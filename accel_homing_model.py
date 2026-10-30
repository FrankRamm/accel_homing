# Saved accelerometer-homing models. Part of the GPLv3 homing extension.
"""Named SAVE_CONFIG data, separate from editable homing thresholds."""
import json
import math

try:
    from .accel_homing_filters import validate_detector_identity
except ImportError:
    from accel_homing_filters import validate_detector_identity


def _validate_detector_metadata(metadata):
    kind = metadata.get("detector_type", "raw")
    if kind not in ("raw", "filtered"):
        raise ValueError("unknown saved detector type")
    if kind == "filtered":
        if metadata.get("impact_metric") != "xyz_magnitude":
            raise ValueError("filtered model must use the XYZ vector metric")
        validate_detector_identity(metadata.get("detector_version"), metadata.get("detector_config"), metadata)
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
        _validate_detector_metadata(profile["metadata"])


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
        self.version = config.getint("version", 1, minval=1, maxval=2)
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
                if metadata.get("detector_type", "raw") == "filtered" and self.version < 2:
                    raise ValueError("active filtered profiles require model schema version 2")
            except (ValueError, TypeError) as exc:
                raise config.error("Invalid %s model metadata: %s" % (axis, exc))
            self.metadata[axis] = metadata
            if threshold is not None:
                if not math.isfinite(threshold):
                    raise config.error("Saved %s homing threshold must be finite" % axis)
                if self.successes[axis] != 10:
                    raise config.error("Saved %s homing model needs ten successes" % axis)
                self.thresholds[axis] = threshold

    def get_status(self, eventtime):
        return {"name": self.name, "version": self.version,
                "threshold_g": dict(self.thresholds),
                "successes": dict(self.successes),
                "metadata": {a: dict(m) for a, m in self.metadata.items()}}


def load_config_prefix(config):
    return AccelHomingModel(config)
