"""Synthetic orientation inference; never communicates with a printer."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from accel_homing_axes import GRAVITY_MM_S2, infer_axis_mapping, validate_axis_mapping


def motion_pass(direction, vector=(1., 0., 0.), noise=0., rate=1000., offset=(.1, -.2, 1.)):
    start, ramp, cruise_end, end = 1., 1.04, 1.5, 1.54
    segments = [
        {"phase": "acceleration", "start_time_s": start, "end_time_s": ramp,
         "start_velocity_mm_s": 0., "acceleration_mm_s2": 1000.},
        {"phase": "cruise", "start_time_s": ramp, "end_time_s": cruise_end,
         "start_velocity_mm_s": 40., "acceleration_mm_s2": 0.},
        {"phase": "braking", "start_time_s": cruise_end, "end_time_s": end,
         "start_velocity_mm_s": 40., "acceleration_mm_s2": -1000.},
    ]
    samples = []
    for i in range(int(.7 * rate) + 1):
        time = .95 + i / rate
        accel = 1000. if start <= time < ramp else -1000. if cruise_end <= time < end else 0.
        row = [time]
        for channel in range(3):
            vibration = noise * math.sin(2. * math.pi * (137. + channel * 20.) * time)
            row.append(offset[channel] + direction * vector[channel] * accel / GRAVITY_MM_S2 + vibration)
        samples.append(row)
    return {"direction": direction, "segments": segments,
            "capture": {"samples": samples, "baseline_g": offset, "fault": None,
                        "dropped": 0, "amplitude_limited": False}}


class AxisMappingInferenceTests(unittest.TestCase):
    def test_all_source_channels_and_signs_for_both_machine_axes(self):
        for axis in "xy":
            for source in range(3):
                for sign in (-1, 1):
                    with self.subTest(axis=axis, source=source, sign=sign):
                        vector = [0., 0., 0.]
                        vector[source] = sign
                        result = infer_axis_mapping(axis, [motion_pass(1, vector), motion_pass(-1, vector)])
                        self.assertEqual((result["axis"], result["source_axis"], result["sign"]), (axis, source, sign))
                        self.assertGreater(result["confidence"], .99)

    def test_integration_identifies_motion_under_high_frequency_transverse_noise(self):
        result = infer_axis_mapping("y", [motion_pass(1, (0., 0., -1.), noise=.25),
                                          motion_pass(-1, (0., 0., -1.), noise=.25)])
        self.assertEqual((result["source_axis"], result["sign"]), (2, -1))

    def test_stationary_bias_does_not_determine_orientation(self):
        positive = motion_pass(1, (0., -1., 0.))
        negative = motion_pass(-1, (0., -1., 0.))
        for item in (positive, negative):
            # A constant residual bias is removed by rest-to-rest integration.
            for row in item["capture"]["samples"]:
                row[1] += .03
                row[3] -= .04
        self.assertEqual(infer_axis_mapping("x", [positive, negative])["source_axis"], 1)

    def test_rejects_diagonal_sensor_orientation(self):
        component = math.sqrt(.5)
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            infer_axis_mapping("x", [motion_pass(1, (component, component, 0.)),
                                      motion_pass(-1, (component, component, 0.))])

    def test_rejects_disagreeing_channel_and_sign(self):
        for vector in ((0., 1., 0.), (-1., 0., 0.)):
            with self.subTest(vector=vector), self.assertRaisesRegex(ValueError, "disagrees"):
                infer_axis_mapping("x", [motion_pass(1), motion_pass(-1, vector)])

    def test_rejects_orientation_change_with_same_dominant_channel(self):
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            infer_axis_mapping("x", [motion_pass(1, (.94, .34, 0.)),
                                      motion_pass(-1, (.94, -.34, 0.))])

    def test_rejects_no_motion_or_wrong_scale(self):
        for scale in (0., .1, 2.):
            with self.subTest(scale=scale), self.assertRaisesRegex(ValueError, "commanded speed"):
                infer_axis_mapping("y", [motion_pass(1, (scale, 0., 0.)),
                                          motion_pass(-1, (scale, 0., 0.))])

    def test_rejects_missing_direction(self):
        with self.assertRaisesRegex(ValueError, "both positive and negative"):
            infer_axis_mapping("x", [motion_pass(1), motion_pass(1)])

    def test_rejects_corrupt_or_partial_samples_and_clipping(self):
        changes = {
            "nan": lambda c: c["samples"][100].__setitem__(2, float("nan")),
            "reorder": lambda c: c["samples"][100].__setitem__(0, c["samples"][99][0]),
            "gap": lambda c: c["samples"].__delitem__(slice(100, 110)),
            "partial": lambda c: c["samples"].__delitem__(slice(0, 100)),
            "clipped": lambda c: c.__setitem__("amplitude_limited", True),
            "reference_clip": lambda c: c.__setitem__("reference_signal_quality", {"amplitude_limited": True}),
            "dropped": lambda c: c.__setitem__("dropped", 1),
            "fault": lambda c: c.__setitem__("fault", "sensor_error"),
        }
        for name, change in changes.items():
            with self.subTest(name=name):
                positive = motion_pass(1)
                change(positive["capture"])
                with self.assertRaises(ValueError):
                    infer_axis_mapping("x", [positive, motion_pass(-1)])

    def test_rejects_incomplete_planner_phases(self):
        positive = motion_pass(1)
        positive["segments"].pop()
        with self.assertRaisesRegex(ValueError, "acceleration, cruise and braking"):
            infer_axis_mapping("x", [positive, motion_pass(-1)])

    def test_rejects_non_rest_or_discontinuous_planner_velocity(self):
        for index, field, value in ((0, "start_velocity_mm_s", 1.),
                                    (1, "start_velocity_mm_s", 50.),
                                    (2, "acceleration_mm_s2", -900.)):
            positive = motion_pass(1)
            positive["segments"][index][field] = value
            with self.subTest(index=index, field=field), self.assertRaises(ValueError):
                infer_axis_mapping("x", [positive, motion_pass(-1)])

    def test_result_does_not_retain_raw_captures_or_mutable_input(self):
        positive, negative = motion_pass(1), motion_pass(-1)
        result = infer_axis_mapping("x", [positive, negative])
        saved = copy.deepcopy(result)
        positive["capture"]["samples"].clear()
        self.assertEqual(result, saved)
        self.assertNotIn("samples", result["passes"][0])


class AxisMappingValidationTests(unittest.TestCase):
    def setUp(self):
        self.mapping = infer_axis_mapping("x", [motion_pass(1), motion_pass(-1)])

    def test_rejects_ambiguous_or_unsupported_metadata(self):
        for field, value in (("version", True), ("version", 2), ("source_axis", True),
                             ("source_axis", 3), ("sign", 1.), ("sign", 0),
                             ("axis", "z"), ("method", "unknown"),
                             ("coordinate_frame", "raw_sensor"), ("confidence", .5),
                             ("diagnostics", {"bad": float("inf")})):
            mapping = dict(self.mapping, **{field: value})
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_axis_mapping(mapping, "x")

    def test_wrong_machine_axis_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_axis_mapping(self.mapping, "y")

    def test_validation_returns_frozen_copy_for_attempt(self):
        result = validate_axis_mapping(self.mapping, "x")
        self.mapping["passes"][0]["velocity_mm_s"][0] = 999.
        self.assertNotEqual(self.mapping, result)


if __name__ == "__main__":
    unittest.main()
