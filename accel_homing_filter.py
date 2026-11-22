# Optional causal accelerometer notch. GPLv3.
"""Small scalar biquads; no look-ahead, resampling, or runtime dependencies."""
import math


POLICY = "causal_xyz_notch_zero_at_arm_v1"
RATE_TOLERANCE = 0.02
DISABLED = {"type": "disabled"}


def profile(frequency, q):
    if not math.isfinite(frequency) or frequency < 0:
        raise ValueError("notch_frequency must be finite and nonnegative")
    if not math.isfinite(q) or not 0.5 <= q <= 1000:
        raise ValueError("notch_q must be finite and between 0.5 and 1000")
    return (dict(DISABLED) if frequency == 0 else
            {"type": POLICY, "frequency_hz": frequency, "q": q})


def coefficients(settings, rate):
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("notch requires a finite positive sensor sample rate")
    frequency, q = settings["frequency_hz"], settings["q"]
    if not 0 < frequency < rate * 0.5:
        raise ValueError("notch_frequency %.12g Hz must be below sensor Nyquist %.12g Hz"
                         % (frequency, rate * 0.5))
    omega = 2 * math.pi * frequency / rate
    alpha = math.sin(omega) / (2 * q)
    scale = 1 / (1 + alpha)
    b0, b1, b2 = scale, -2 * math.cos(omega) * scale, scale
    a1, a2 = b1, (1 - alpha) * scale
    # Strict Jury stability checks also reject unresolvable near-DC/Nyquist
    # settings whose rounded coefficients would have a pole on the unit circle.
    if (not all(math.isfinite(v) for v in (b0, b1, b2, a1, a2))
            or not abs(a2) < 1 or not 1 + a1 + a2 > 0 or not 1 - a1 + a2 > 0):
        raise ValueError("notch settings cannot produce numerically stable coefficients")
    return b0, b1, b2, a1, a2


def same_rate(first, second):
    return (isinstance(first, (int, float)) and not isinstance(first, bool)
            and math.isfinite(first) and first > 0
            and isinstance(second, (int, float)) and not isinstance(second, bool)
            and math.isfinite(second) and second > 0
            and abs(first / second - 1) <= RATE_TOLERANCE)


class CausalNotch:
    """Transposed direct form II, two delay values per signed XYZ channel."""
    def __init__(self, settings, rate):
        self.settings, self.rate = dict(settings), rate
        self.coefficients = coefficients(settings, rate)
        self.state = [[0.0, 0.0] for _ in range(3)]

    def process(self, xyz):
        b0, b1, b2, a1, a2 = self.coefficients
        result = []
        for value, state in zip(xyz, self.state):
            output = b0 * value + state[0]
            first = b1 * value - a1 * output + state[1]
            second = b2 * value - a2 * output
            if not all(math.isfinite(v) for v in (output, first, second)):
                raise ValueError("non-finite notch state")
            state[:] = first, second
            result.append(output)
        return tuple(result)

    def metadata(self):
        return dict(self.settings, sample_rate_hz=self.rate,
                    coefficients=list(self.coefficients), initial_state="zero_at_first_armed_sample",
                    rate_tolerance_fraction=RATE_TOLERANCE)


def replay(samples, baseline, metadata, start_index):
    filt = CausalNotch(metadata, metadata["sample_rate_hz"])
    if list(filt.coefficients) != metadata.get("coefficients"):
        raise ValueError("notch coefficient provenance mismatch")
    return [(row[0],) + filt.process(tuple(row[i + 1] - baseline[i] for i in range(3)))
            for row in samples[start_index:]]
