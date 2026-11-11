# Saved accelerometer-homing models. Part of the GPLv3 homing extension.
"""Named SAVE_CONFIG data, separate from editable homing thresholds."""
import json
import math

if __package__:
    from .accel_homing_backends import load_filter_module
else:
    from accel_homing_backends import load_filter_module

_filter = load_filter_module()
DISABLED, coefficients, profile = _filter.DISABLED, _filter.coefficients, _filter.profile


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
                settings = metadata.get("signal_filter", DISABLED)
                if not isinstance(settings, dict):
                    raise ValueError("signal_filter must be an object")
                if settings != DISABLED:
                    if (self.version != 2 or metadata.get("impact_metric") != "xyz_magnitude"
                            or profile(settings.get("frequency_hz", float("nan")),
                                       settings.get("q", float("nan"))) != settings):
                        raise ValueError("notch model requires version 2 and an exact XYZ filter identity")
                    rate = metadata.get("filter_sample_rate_hz")
                    if not isinstance(rate, (int, float)) or isinstance(rate, bool):
                        raise ValueError("notch model requires its measured sensor sample rate")
                    coefficients(settings, rate)
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
