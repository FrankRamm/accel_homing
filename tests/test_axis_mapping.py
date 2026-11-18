"""Synthetic orientation inference; never communicates with a printer."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from accel_homing_axes import (CONFIGURED_METHOD, FRAME, GRAVITY_MM_S2,
                               infer_axis_mapping, normalize_configured_axes_map,
                               validate_axis_mapping)


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


def quantized_mapping_pulse(direction, vibration_channel=None):
    """15 mm / 20 mm/s / 500 mm/s2, with optional single-channel ripple.

    This is a constructed waveform, not a capture from a printer. Quantization
    models a 0.0078125 g step, allowing a repeatable longitudinal/transverse
    comparison without changing the commanded trajectory or inference gates.
    """
    baseline = [.125, -.25, 1.]
    segments = [
        {"phase": "acceleration", "start_time_s": 1., "end_time_s": 1.04,
         "start_velocity_mm_s": 0., "acceleration_mm_s2": 500.},
        {"phase": "cruise", "start_time_s": 1.04, "end_time_s": 1.75,
         "start_velocity_mm_s": 20., "acceleration_mm_s2": 0.},
        {"phase": "braking", "start_time_s": 1.75, "end_time_s": 1.79,
         "start_velocity_mm_s": 20., "acceleration_mm_s2": -500.},
    ]
    samples = []
    for index in range(831):
        time = .98 + index * .001
        acceleration = 500. if 1. <= time < 1.04 else -500. if 1.75 <= time < 1.79 else 0.
        row = [time]
        for channel in range(3):
            value = baseline[channel]
            if channel == 0:
                value += direction * acceleration / GRAVITY_MM_S2
            if channel == vibration_channel:
                value += .5 * math.cos(2. * math.pi * 100. * (time - 1.))
            row.append(round(value / .0078125) * .0078125)
        samples.append(row)
    return {"direction": direction, "segments": segments,
            "capture": {"samples": samples, "baseline_g": baseline,
                        "fault": None, "dropped": 0}}


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


class AxisMappingDiagnosticsTests(unittest.TestCase):
    def test_clean_diagnostics_preserve_mapping_and_report_numeric_bounds(self):
        passes = [quantized_mapping_pulse(1), quantized_mapping_pulse(-1)]
        expected = infer_axis_mapping("x", passes)
        diagnostics = {"stale": "discarded"}
        result = infer_axis_mapping("x", passes, diagnostics)
        self.assertEqual(result, expected)
        self.assertNotIn("stale", diagnostics)
        self.assertEqual(diagnostics["status"], "accepted")
        self.assertEqual(diagnostics["evaluated_pass_count"], 2)
        self.assertEqual(diagnostics["unevaluated_pass_count"], 0)
        self.assertIsNone(diagnostics["rejected_pass_index"])
        for index, report in enumerate(diagnostics["passes"]):
            self.assertEqual(report["pass_index"], index)
            self.assertEqual(report["status"], "passed")
            self.assertEqual((report["source_axis"], report["sign"]), (0, 1))
            self.assertEqual(report["sample_count"], 831)
            self.assertAlmostEqual(report["sample_rate_hz"], 1000.)
            self.assertAlmostEqual(report["profile_duration_s"], .79)
            self.assertAlmostEqual(report["capture_lead_time_s"], .02)
            self.assertAlmostEqual(report["capture_tail_time_s"], .02)
            self.assertGreaterEqual(report["cruise_start_time_s"], 1.111)
            self.assertLessEqual(report["cruise_end_time_s"], 1.679)
            self.assertEqual(report["expected_speed_mm_s"], 20.)
            self.assertEqual(report["alignment"], 1.)
            self.assertLess(report["total_deviation_to_limit_ratio"], 1.)
            for drift, bias in zip(report["endpoint_drift_xyz_mm_s"], report["removed_bias_xyz_mm_s2"]):
                self.assertAlmostEqual(drift / .79, bias)
        json.dumps(diagnostics, allow_nan=False)

    def test_longitudinal_ripple_is_rejected_with_parallel_metrics_and_no_transverse_error(self):
        passes = [quantized_mapping_pulse(1, 0), quantized_mapping_pulse(-1, 0)]
        diagnostics = {}
        for output in (None, diagnostics):
            with self.assertRaisesRegex(ValueError, r"unstable \(direction \+1, channel X:.*mm/s exceeds limit.*mm/s"):
                infer_axis_mapping("x", passes, output)
        report = diagnostics["passes"][0]
        self.assertEqual(report["alignment"], 1.)
        self.assertGreater(report["parallel_median_deviation_mm_s"], report["total_deviation_limit_mm_s"])
        self.assertAlmostEqual(report["parallel_median_deviation_mm_s"], report["total_median_deviation_mm_s"])
        self.assertEqual(report["perpendicular_median_deviation_mm_s"], 0.)
        self.assertEqual(report["sign_agreement_fraction"], 1.)
        self.assertEqual(report["nonpositive_projection_fraction"], 0.)
        self.assertEqual(report["near_zero_projection_fraction"], 0.)
        self.assertGreater(report["total_deviation_ratio"], report["total_deviation_limit_ratio"])
        self.assertGreater(report["total_deviation_to_limit_ratio"], 1.)
        self.assertAlmostEqual(report["velocity_norm_mm_s"], 20.092, delta=.002)
        self.assertAlmostEqual(report["total_median_deviation_mm_s"], 4.491, delta=.002)

    def test_transverse_ripple_is_rejected_with_perpendicular_metrics(self):
        diagnostics = {}
        with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
            infer_axis_mapping("x", [quantized_mapping_pulse(1, 1), quantized_mapping_pulse(-1, 1)], diagnostics)
        report = diagnostics["passes"][0]
        self.assertEqual(report["source_axis"], 0)
        self.assertGreater(report["perpendicular_median_deviation_mm_s"], report["total_deviation_limit_mm_s"])
        self.assertGreater(report["perpendicular_median_deviation_mm_s"], report["parallel_median_deviation_mm_s"])
        self.assertAlmostEqual(report["perpendicular_median_deviation_mm_s"], 4.444, delta=.002)
        json.dumps(diagnostics, allow_nan=False)

    def test_first_failure_retains_only_evaluated_pass_and_original_reason(self):
        diagnostics = {}
        with self.assertRaisesRegex(ValueError, "cruise response is unstable") as caught:
            infer_axis_mapping("x", [quantized_mapping_pulse(1, 0), None], diagnostics)
        self.assertEqual(diagnostics["status"], "rejected")
        self.assertEqual(diagnostics["reason"], str(caught.exception))
        self.assertEqual(diagnostics["rejected_pass_index"], 0)
        self.assertEqual(diagnostics["evaluated_pass_count"], 1)
        self.assertEqual(diagnostics["unevaluated_pass_count"], 1)
        self.assertEqual(len(diagnostics["passes"]), 1)
        self.assertEqual(diagnostics["passes"][0]["status"], "rejected")
        self.assertEqual(diagnostics["passes"][0]["reason"], diagnostics["reason"])

    def test_second_pass_failure_keeps_first_pass_and_identifies_negative_direction(self):
        diagnostics = {}
        with self.assertRaisesRegex(ValueError, "direction -1, channel X"):
            infer_axis_mapping("x", [quantized_mapping_pulse(1), quantized_mapping_pulse(-1, 0)], diagnostics)
        self.assertEqual(diagnostics["rejected_pass_index"], 1)
        self.assertEqual(diagnostics["evaluated_pass_count"], 2)
        self.assertEqual(diagnostics["unevaluated_pass_count"], 0)
        self.assertEqual([report["status"] for report in diagnostics["passes"]], ["passed", "rejected"])
        self.assertEqual(diagnostics["passes"][1]["direction"], -1)

    def test_cross_pass_rejection_does_not_falsely_reject_an_individual_pass(self):
        diagnostics = {}
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            infer_axis_mapping("x", [motion_pass(1, (.94, .34, 0.)),
                                      motion_pass(-1, (.94, -.34, 0.))], diagnostics)
        self.assertEqual(diagnostics["status"], "rejected")
        self.assertIsNone(diagnostics["rejected_pass_index"])
        self.assertTrue(all(report["status"] == "passed" for report in diagnostics["passes"]))
        self.assertLess(diagnostics["direction_agreement"], diagnostics["direction_agreement_limit"])

    def test_diagnostics_are_detached_from_input_and_result(self):
        passes = [quantized_mapping_pulse(1), quantized_mapping_pulse(-1)]
        diagnostics = {}
        result = infer_axis_mapping("x", passes, diagnostics)
        snapshot = copy.deepcopy(diagnostics)
        passes[0]["capture"]["baseline_g"][0] = 99.
        passes[0]["capture"]["samples"].clear()
        passes[0]["segments"][0]["start_time_s"] = 99.
        result["passes"][0]["velocity_mm_s"][0] = 99.
        self.assertEqual(diagnostics, snapshot)
        for report in diagnostics["passes"]:
            self.assertNotIn("samples", report)
            self.assertNotIn("capture", report)
            self.assertNotIn("segments", report)
            self.assertTrue(all(not isinstance(value, list) or len(value) == 3 for value in report.values()))

    def test_invalid_nonfinite_inputs_still_produce_finite_json(self):
        for target in ("baseline", "samples", "profile"):
            passes = [quantized_mapping_pulse(1), quantized_mapping_pulse(-1)]
            if target == "baseline":
                passes[0]["capture"]["baseline_g"][0] = float("nan")
            elif target == "samples":
                passes[0]["capture"]["samples"][0][2] = float("inf")
            else:
                passes[0]["segments"][0]["end_time_s"] = float("inf")
            diagnostics = {}
            with self.subTest(target=target), self.assertRaises(ValueError):
                infer_axis_mapping("x", passes, diagnostics)
            json.dumps(diagnostics, allow_nan=False)
            self.assertEqual(diagnostics["rejected_pass_index"], 0)
            self.assertEqual(diagnostics["unevaluated_pass_count"], 1)

    def test_diagnostic_pass_reports_are_bounded_without_limiting_inference(self):
        passes = [quantized_mapping_pulse(1 if index % 2 else -1) for index in range(10)]
        diagnostics = {}
        result = infer_axis_mapping("x", passes, diagnostics)
        self.assertEqual(len(result["passes"]), 10)
        self.assertEqual(len(diagnostics["passes"]), 8)
        self.assertEqual(diagnostics["evaluated_pass_count"], 10)
        self.assertEqual(diagnostics["omitted_pass_report_count"], 2)
        self.assertEqual(diagnostics["unevaluated_pass_count"], 0)
        passes[9] = quantized_mapping_pulse(1, 0)
        with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
            infer_axis_mapping("x", passes, diagnostics)
        self.assertEqual(len(diagnostics["passes"]), 8)
        self.assertEqual(diagnostics["omitted_pass_report_count"], 2)
        self.assertEqual(diagnostics["rejected_pass_index"], 9)
        self.assertEqual(diagnostics["passes"][-1]["pass_index"], 9)
        self.assertEqual(diagnostics["passes"][-1]["status"], "rejected")
        self.assertIn("total_median_deviation_mm_s", diagnostics["passes"][-1])
        json.dumps(diagnostics, allow_nan=False)


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


class ConfiguredAxisMappingValidationTests(unittest.TestCase):
    def setUp(self):
        self.mapping = {"version": 1, "axis": "x", "source_axis": 0, "sign": 1,
                        "coordinate_frame": FRAME, "method": CONFIGURED_METHOD,
                        "configuration": {"sensor_section": "beacon toolhead",
                                          "option": "accel_axes_map", "axes_map": ["y", "-x", "z"]}}

    def test_accepts_declared_identity_without_fabricated_inference_evidence(self):
        for axis in "xy":
            self.mapping.update(axis=axis, source_axis="xy".index(axis))
            result = validate_axis_mapping(self.mapping, axis)
            self.assertEqual(result, self.mapping)
            for field in ("confidence", "passes", "direction_agreement"):
                self.assertNotIn(field, result)
            json.dumps(result, allow_nan=False)

    def test_rejects_raw_axis_transform_or_measured_evidence_in_configured_frame(self):
        for key, value in (("source_axis", 1), ("sign", -1),
                           ("confidence", 1.), ("passes", []), ("direction_agreement", 1.)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_axis_mapping(dict(self.mapping, **{key: value}), "x")

    def test_requires_sensor_option_and_normalized_permutation_provenance(self):
        for key, value in (("sensor_section", ""), ("sensor_section", " beacon"),
                           ("option", "axes_map"), ("axes_map", "y,-x,z"),
                           ("axes_map", ["y", "y", "z"]), ("axes_map", [" y", "-x", "z"]),
                           ("axes_map", ["y", "x", "+z"]), ("axes_map", ["y", "-x"]),
                           ("axes_map", ["y", None, "z"])):
            candidate = copy.deepcopy(self.mapping)
            candidate["configuration"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_axis_mapping(candidate, "x")
        candidate = dict(self.mapping)
        candidate.pop("configuration")
        with self.assertRaises(ValueError):
            validate_axis_mapping(candidate, "x")

    def test_generic_sensor_uses_axes_map_option_and_returns_detached_provenance(self):
        self.mapping["configuration"].update(sensor_section="adxl345 toolhead", option="axes_map")
        result = validate_axis_mapping(self.mapping, "x")
        self.mapping["configuration"]["axes_map"][0] = "z"
        self.assertEqual(result["configuration"]["axes_map"], ["y", "-x", "z"])

    def test_raw_option_normalization_never_supplies_an_implicit_map(self):
        self.assertEqual(normalize_configured_axes_map(" y, -x,\n z "), ["y", "-x", "z"])
        for invalid in (None, "", "x,y", "x,y,z,x", "x,-x,z", "X,y,z", "x,+y,z", 0):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                normalize_configured_axes_map(invalid)


if __name__ == "__main__":
    unittest.main()
