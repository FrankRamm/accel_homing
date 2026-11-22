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


def quantized_mapping_pulse(direction, vibration_channel=None, ripple_g=.5, motion_gain=1.):
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
                value += motion_gain * direction * acceleration / GRAVITY_MM_S2
            if channel == vibration_channel:
                value += ripple_g * math.cos(2. * math.pi * 100. * (time - 1.))
            row.append(round(value / .0078125) * .0078125)
        samples.append(row)
    return {"direction": direction, "segments": segments,
            "capture": {"samples": samples, "baseline_g": baseline,
                        "fault": None, "dropped": 0}}


def coherent_ripple_pass(direction, rate=3300., frequency=100., phase=0.,
                         ripple_mm_s=9.35, motion_gain=.9, vector=(0., 1., 0.),
                         ripple_vector=None, cruise_duration=.71, jitter=0.):
    """Synthetic velocity ripple with smooth rest endpoints, not recorded data.

    The derivative of the enveloped velocity ripple is added to acceleration,
    so its size is specified in mm/s without depending on sensor sample rate.
    """
    item = quantized_mapping_pulse(direction)
    ramp = .04
    cruise_end, end = 1.04 + cruise_duration, 1.08 + cruise_duration
    item["segments"][1]["end_time_s"] = cruise_end
    item["segments"][2].update(start_time_s=cruise_end, end_time_s=end)
    baseline = item["capture"]["baseline_g"]
    ripple_vector = vector if ripple_vector is None else ripple_vector
    samples = []
    for index in range(math.ceil((end + .02 - .98) * rate) + 1):
        time = .98 + (index + jitter * math.sin(index * 1.7)) / rate
        acceleration = 500. if 1. <= time < 1.04 else -500. if cruise_end <= time < end else 0.
        envelope, derivative = 0., 0.
        if 1. < time < 1.04:
            angle = math.pi * (time - 1.) / ramp
            envelope, derivative = .5 - .5 * math.cos(angle), .5 * math.pi / ramp * math.sin(angle)
        elif 1.04 <= time <= cruise_end:
            envelope = 1.
        elif cruise_end < time < end:
            angle = math.pi * (time - cruise_end) / ramp
            envelope, derivative = .5 + .5 * math.cos(angle), -.5 * math.pi / ramp * math.sin(angle)
        angle = 2. * math.pi * frequency * (time - 1.) + phase
        ripple = ripple_mm_s * (derivative * math.sin(angle)
                               + envelope * 2. * math.pi * frequency * math.cos(angle))
        samples.append([time] + [baseline[channel] + direction * (
            vector[channel] * acceleration * motion_gain + ripple_vector[channel] * ripple) / GRAVITY_MM_S2
            for channel in range(3)])
    item["capture"]["samples"] = samples
    return item


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
        self.assertEqual(diagnostics["version"], 3)
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
            self.assertEqual(report["stable_cruise_fraction"], 1.)
            for drift, bias in zip(report["endpoint_drift_xyz_mm_s"], report["removed_bias_xyz_mm_s2"]):
                self.assertAlmostEqual(drift / .79, bias)
        json.dumps(diagnostics, allow_nan=False)

    def test_reported_scale_of_ripple_can_preserve_the_same_signed_channel(self):
        for channel in (0, 1):
            with self.subTest(vibration_channel=channel):
                passes = [quantized_mapping_pulse(d, channel, ripple_g=.71, motion_gain=1.08)
                          for d in (1, -1)]
                diagnostics = {}
                result = infer_axis_mapping("x", passes, diagnostics)
                self.assertEqual(result, infer_axis_mapping("x", passes))
                self.assertEqual((result["source_axis"], result["sign"]), (0, 1))
                for report in diagnostics["passes"]:
                    self.assertAlmostEqual(report["total_median_deviation_mm_s"], 6.34, delta=.15)
                    self.assertGreater(report["total_deviation_ratio"], .20)
                    self.assertEqual(report["stable_cruise_fraction"], 1.)
                    self.assertAlmostEqual(report["parallel_median_deviation_mm_s"],
                                           6.34 if channel == 0 else 0., delta=.15)
                    self.assertAlmostEqual(report["perpendicular_median_deviation_mm_s"],
                                           6.34 if channel == 1 else 0., delta=.15)

    def test_coherent_ripple_near_reported_beacon_scale_passes_temporal_checks(self):
        # Representative of the separate 82.56% / 99.6% / 100% report, not a
        # replay or reconstruction of its unavailable raw capture.
        vector = (math.sqrt(1. - .996 ** 2), .996, 0.)
        diagnostics = {}
        result = infer_axis_mapping("y", [coherent_ripple_pass(d, frequency=97., vector=vector)
                                          for d in (1, -1)], diagnostics)
        self.assertEqual((result["source_axis"], result["sign"]), (1, 1))
        for report in diagnostics["passes"]:
            self.assertAlmostEqual(report["in_speed_band_fraction"], .8256, delta=.035)
            self.assertAlmostEqual(report["total_median_deviation_mm_s"], 6.62, delta=.15)
            self.assertAlmostEqual(report["alignment"], .996)
            self.assertEqual(report["sign_agreement_fraction"], 1.)
            self.assertLess(report["stable_cruise_fraction"], report["minimum_stable_cruise_fraction"])
            self.assertEqual(report["pooled_stable_cruise_fraction"], 1.)
            self.assertEqual(report["raw_coherent_cruise_time_fraction"], 1.)
            self.assertTrue(report["local_motion_supported"])

    def test_coherent_ripple_uses_time_not_fixed_sample_blocks(self):
        for rate, frequency, phase, jitter in ((800., 87., .7, 0.), (1000., 113., 1.3, .2),
                                               (3200., 97., 2.1, .2), (6400., 137., .4, 0.)):
            with self.subTest(rate=rate, frequency=frequency, phase=phase, jitter=jitter):
                passes = [coherent_ripple_pass(d, rate, frequency, phase, jitter=jitter,
                                               vector=(0., -1., 0.), ripple_mm_s=11.) for d in (1, -1)]
                diagnostics = {}
                result = infer_axis_mapping("y", passes, diagnostics)
                self.assertEqual((result["source_axis"], result["sign"]), (1, -1))
                for report in diagnostics["passes"]:
                    self.assertLess(report["in_speed_band_fraction"], .95)
                    self.assertEqual(report["pooled_stable_cruise_fraction"], 1.)
                    self.assertAlmostEqual(report["pooled_cruise_duration_s"], .71 * .8)
                    self.assertLessEqual(report["maximum_cruise_window_s"], .020 + 1e-12)
                    self.assertGreater(report["minimum_cruise_window_s"], .019)
                json.dumps(diagnostics, allow_nan=False)

    def test_pooling_cannot_manufacture_direction_from_opposing_or_cross_channel_vectors(self):
        for ripple_vector, ripple in (((0., 1., 0.), 30.), ((1., 0., 0.), 18.)):
            with self.subTest(ripple_vector=ripple_vector):
                diagnostics = {}
                passes = [coherent_ripple_pass(d, ripple_vector=ripple_vector, ripple_mm_s=ripple)
                          for d in (1, -1)]
                with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
                    infer_axis_mapping("y", passes, diagnostics)
                report = diagnostics["passes"][0]
                self.assertGreater(report["alignment"], .99)
                self.assertEqual(report["pooled_stable_cruise_fraction"], 1.)
                self.assertLess(report["raw_coherent_cruise_time_fraction"], .95)

    def test_low_frequency_speed_variation_is_not_excused_by_coherent_direction(self):
        diagnostics = {}
        passes = [coherent_ripple_pass(d, frequency=5., ripple_mm_s=13.) for d in (1, -1)]
        with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
            infer_axis_mapping("y", passes, diagnostics)
        report = diagnostics["passes"][0]
        self.assertEqual(report["sign_agreement_fraction"], 1.)
        self.assertLess(report["pooled_stable_cruise_fraction"], .95)

    def test_roughly_half_raw_support_does_not_prove_direction_after_pooling(self):
        # The separate 824/1779 report contains no angular/sign evidence.
        # A similar aggregate fraction can instead hide cross-channel motion.
        diagnostics = {}
        passes = [coherent_ripple_pass(d, frequency=97., ripple_vector=(1., 0., 0.), ripple_mm_s=16.8)
                  for d in (1, -1)]
        with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
            infer_axis_mapping("y", passes, diagnostics)
        report = diagnostics["passes"][0]
        self.assertAlmostEqual(report["stable_cruise_fraction"], 824 / 1779, delta=.03)
        self.assertEqual(report["pooled_stable_cruise_fraction"], 1.)
        self.assertLess(report["raw_coherent_cruise_time_fraction"], .95)

    def test_signed_pooling_does_not_turn_pure_vibration_into_commanded_motion(self):
        with self.assertRaisesRegex(ValueError, "commanded speed"):
            infer_axis_mapping("y", [coherent_ripple_pass(d, motion_gain=0., ripple_mm_s=30.)
                                      for d in (1, -1)])

    def test_local_80ms_stop_or_reversal_rejects_even_below_five_percent_of_long_cruise(self):
        for acceleration in (900., 1800.):
            for stop_start in (1.60, 2.30, 5.10):
                with self.subTest(acceleration=acceleration, stop_start=stop_start):
                    positive = coherent_ripple_pass(1, ripple_mm_s=0., cruise_duration=4.71, rate=1000.)
                    # Rest or reverse for 80 ms between two 20 ms ramps at
                    # several locations within the trimmed cruise interval.
                    for row in positive["capture"]["samples"]:
                        if stop_start <= row[0] < stop_start + .02:
                            row[2] -= acceleration / GRAVITY_MM_S2
                        elif stop_start + .10 <= row[0] < stop_start + .12:
                            row[2] += acceleration / GRAVITY_MM_S2
                    diagnostics = {}
                    with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
                        infer_axis_mapping("y", [positive, coherent_ripple_pass(-1)], diagnostics)
                    report = diagnostics["passes"][0]
                    self.assertGreater(report["pooled_stable_cruise_fraction"], .95)
                    self.assertFalse(report["local_motion_supported"])
                    self.assertAlmostEqual(report["pooled_cruise_duration_s"], .8 * 4.71)
                    self.assertLess(report["minimum_pooled_projection_mm_s"], report["minimum_local_motion_mm_s"])

    def test_brief_stop_during_cruise_is_not_hidden_by_the_stroke_median(self):
        positive = quantized_mapping_pulse(1)
        # Slow to rest for 80 ms, then resume. The net velocity change is zero.
        for row in positive["capture"]["samples"]:
            if 1.30 <= row[0] < 1.32:
                row[1] -= 1000. / GRAVITY_MM_S2
            elif 1.40 <= row[0] < 1.42:
                row[1] += 1000. / GRAVITY_MM_S2
        diagnostics = {}
        with self.assertRaisesRegex(ValueError, "cruise response is unstable"):
            infer_axis_mapping("x", [positive, quantized_mapping_pulse(-1)], diagnostics)
        report = diagnostics["passes"][0]
        self.assertLess(report["total_deviation_ratio"], .20)
        self.assertLess(report["in_speed_band_fraction"], report["minimum_stable_cruise_fraction"])

    def test_rejects_inconsistent_sample_speed_or_orientation_even_with_aligned_median(self):
        for channel, failed_check in ((0, "in_speed_band_fraction"), (1, "aligned_cruise_fraction")):
            with self.subTest(vibration_channel=channel):
                diagnostics = {}
                passes = [quantized_mapping_pulse(d, channel, ripple_g=2.) for d in (1, -1)]
                with self.assertRaisesRegex(ValueError, "unstable .*direction \\+1, channel X"):
                    infer_axis_mapping("x", passes, diagnostics)
                report = diagnostics["passes"][0]
                self.assertGreater(report["alignment"], .99)
                self.assertLess(report[failed_check], report["minimum_stable_cruise_fraction"])
                self.assertLess(report["stable_cruise_fraction"], report["minimum_stable_cruise_fraction"])
                json.dumps(diagnostics, allow_nan=False)

    def test_first_failure_retains_only_evaluated_pass_and_original_reason(self):
        diagnostics = {}
        with self.assertRaisesRegex(ValueError, "cruise response is unstable") as caught:
            infer_axis_mapping("x", [quantized_mapping_pulse(1, 0, ripple_g=2.), None], diagnostics)
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
            infer_axis_mapping("x", [quantized_mapping_pulse(1), quantized_mapping_pulse(-1, 0, ripple_g=2.)], diagnostics)
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
        passes[9] = quantized_mapping_pulse(1, 0, ripple_g=2.)
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
