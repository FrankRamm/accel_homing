# Saved accelerometer-homing models. Part of the GPLv3 homing extension.
"""Named SAVE_CONFIG data, separate from editable homing thresholds."""
import json
import math

if __package__:
    from .accel_homing_backends import _ah_compat_sibling_module, load_filter_module
else:
    from accel_homing_backends import _ah_compat_sibling_module, load_filter_module

validate_axis_mapping = _ah_compat_sibling_module("accel_homing_axes", __package__, __file__).validate_axis_mapping

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
                if ("gentle_contact_phase_policy" in metadata
                        and metadata["gentle_contact_phase_policy"] != "prebrake_v1"):
                    raise ValueError("unknown gentle_contact_phase_policy in saved model")
                if metadata.get("impact_metric") == "mapped_axis":
                    metadata["axis_mapping"] = validate_axis_mapping(metadata.get("axis_mapping"), axis)
                    if not isinstance(metadata.get("axis_mapping_context"), dict):
                        raise ValueError("mapped-axis model requires its sensor context")
                settings = metadata.get("signal_filter", DISABLED)
                if not isinstance(settings, dict):
                    raise ValueError("signal_filter must be an object")
                if settings != DISABLED:
                    if (self.version != 2 or metadata.get("impact_metric") not in ("mapped_axis", "xyz_magnitude")
                            or profile(settings.get("frequency_hz", float("nan")),
                                       settings.get("q", float("nan"))) != settings):
                        raise ValueError("notch model requires version 2 and an exact filter identity and supported impact metric")
                    rate = metadata.get("filter_sample_rate_hz")
                    if not isinstance(rate, (int, float)) or isinstance(rate, bool):
                        raise ValueError("notch model requires its measured sensor sample rate")
                    coefficients(settings, rate)
                cruise = metadata.get("cruise_threshold_g")
                if cruise is not None and (type(cruise) not in (int, float) or not cruise > 0.0
                                           or threshold is not None and not cruise < threshold):
                    raise ValueError("cruise_threshold_g must be positive and below the saved threshold")
            except (ValueError, TypeError) as exc:
                raise config.error("Invalid %s model metadata: %s" % (axis, exc))
            self.metadata[axis] = metadata
            if threshold is not None:
                if not math.isfinite(threshold):
                    raise config.error("Saved %s homing threshold must be finite" % axis)
                if self.successes[axis] != 10:
                    raise config.error("Saved %s homing model needs ten successes" % axis)
                self.thresholds[axis] = threshold
        mapped = [self.metadata[axis] for axis in self.thresholds
                  if self.metadata[axis].get("impact_metric") == "mapped_axis"
                  and not self.metadata[axis].get("axis_mapping_invalidated")]
        if (len(mapped) == 2 and mapped[0]["axis_mapping_context"] == mapped[1]["axis_mapping_context"]
                and mapped[0]["axis_mapping"]["source_axis"] == mapped[1]["axis_mapping"]["source_axis"]):
            raise config.error("Saved X and Y homing mappings select the same sensor channel")

    def get_status(self, eventtime):
        return {"name": self.name, "version": self.version,
                "threshold_g": dict(self.thresholds),
                "successes": dict(self.successes),
                "metadata": {a: dict(m) for a, m in self.metadata.items()}}


def load_config_prefix(config):
    return AccelHomingModel(config)
