"""Versioned signed/rectified experiments; synthetic signals are not contact labels."""
import copy
import json
import tempfile
import unittest
from unittest import mock

from accel_homing_filters import (CausalImpactDetector, compare_filter_capture, detector_config,
                                  robust_reference, signal_mode, validate_detector_identity,
                                  validate_filtered_capture_prefix)
from accel_homing_model import AccelHomingModel
from harness import ConfigError, plugin
from test_chronological_clipping import native_filtered_capture
from test_filter_engine import FilteredRig, comparison_capture
import test_filter_engine
from test_workflows import Config, GCmd, Rig


def trace_engine(version, rows, threshold=1., offset=(0., 0., 1.), rate=12500.):
    engine = CausalImpactDetector(threshold, offset, config={"version": version})
    engine.previous_time = -1. / rate
    for index, row in enumerate(rows):
        engine.feed(index / rate, row, index)
    return engine


def native_gap_remap_capture(version=2):
    """Valid original gap with a compressed display clock; synthetic only."""
    original_inspect, original_disarm = plugin.BeaconAccelHoming._inspect, plugin.BeaconAccelHoming.disarm
    shifted = []
    def gap_before_censor(obj, params):
        if obj.capture_attempt is not None and obj.capture_attempt.done and not shifted:
            shifted.append(True)
            obj.reactor.now += .0011
            params = dict(params, start_clock=(params["start_clock"] + round(.0011 * obj.beacon_freq)) & 0xffffffff,
                          **{"#receive_time": params["#receive_time"] + .0011})
        return original_inspect(obj, params)
    def compress_display_gap(obj):
        result = original_disarm(obj)
        previous_end = list(obj.capture_attempt.filter_packet_times.values())[-1][1]
        following_start = obj.clock_to_time(obj.capture_batches[-1][0])
        converter = obj.clock_to_time
        def mapped(clock):
            when = converter(clock)
            if when <= previous_end:
                return when
            if when >= following_start:
                return when - .0011
            fraction = (when - previous_end) / (following_start - previous_end)
            return previous_end + fraction * (following_start - .0011 - previous_end)
        obj.clock_to_time = mapped
        return result
    with mock.patch.object(plugin.BeaconAccelHoming, "_inspect", gap_before_censor), \
            mock.patch.object(plugin.BeaconAccelHoming, "disarm", compress_display_gap):
        return native_filtered_capture(filter_config={"version": version, "confirmation_min_s": .0009},
                                       clock_hz=32000000., sample_period=.0003, split=4)


class RectifiedSignalTests(unittest.TestCase):
    def test_default_identity_is_rectified_and_legacy_config_remains_exact(self):
        current, old = detector_config(), detector_config({"version": 1})
        self.assertEqual((current["version"], current["signal_mode"]), (2, "rectified"))
        self.assertEqual(signal_mode(old), "signed")
        self.assertNotIn("signal_mode", old)
        self.assertEqual(validate_detector_identity(1, old), old)
        self.assertEqual(validate_detector_identity(2, current), current)
        for version, config, metadata in (
                (1, current, None), (2, old, None), (True, old, None),
                (2, {"version": 2}, None), (1, dict(old, signal_mode="signed"), None),
                (2, dict(current, signal_mode="signed"), None),
                (1, old, {"signal_mode": "rectified"}),
                (2, current, {"signal_mode": "signed"})):
            with self.assertRaises(ValueError):
                validate_detector_identity(version, config, metadata)

    def test_opposite_polarities_cancel_only_in_signed_axis_filter(self):
        rows = [(2. * (-1 if i % 2 else 1), 0., 1.) for i in range(80)]
        signed, rectified = [trace_engine(version, rows) for version in (1, 2)]
        self.assertLess(signed.level_g, .5)
        self.assertIsNone(signed.decision)
        self.assertGreater(rectified.level_g, 1.99)
        self.assertIsNotNone(rectified.decision)
        self.assertEqual(signed.raw_level_g, rectified.raw_level_g)
        # An oscillation could be noise or contact. Rectification's higher
        # response is not evidence that this new decision is physically right.

    def test_rectification_increases_stationary_oscillation_envelope(self):
        rows = [(.8 * (-1 if i % 2 else 1), 0., 1.) for i in range(100)]
        signed, rectified = [trace_engine(version, rows) for version in (1, 2)]
        self.assertLess(signed.level_g, .2)
        self.assertGreater(rectified.level_g, .79)
        self.assertIsNone(rectified.decision)
        self.assertGreater(rectified.level_g, signed.level_g)

    def test_constant_negative_pulse_preserves_magnitudes_and_decision_times(self):
        offset = (-.4, .3, 1.03)
        engines = [CausalImpactDetector(1., offset, weights_xyz=(.75, 1., .8333333333333333),
                                        config={"version": version}) for version in (1, 2)]
        for engine in engines:
            engine.previous_time = -1. / 12500.
        rows = [offset] * 10 + [(-2.4, -.7, .03)] * 50
        for index, row in enumerate(rows):
            for engine in engines:
                engine.feed(index / 12500., row, index)
            self.assertAlmostEqual(engines[0].level_g, engines[1].level_g)
            self.assertAlmostEqual(engines[0].magnitude_g, engines[1].magnitude_g)
        self.assertLess(engines[0].filtered_xyz[0], 0)
        self.assertGreater(engines[1].filtered_xyz[0], 0)
        for event in ("onset", "crossing", "confirmation"):
            self.assertEqual(engines[0].decision[event + "_time_s"], engines[1].decision[event + "_time_s"])

    def test_gravity_and_signed_bias_are_subtracted_before_rectification(self):
        offset = (-.4, .3, 1.03)
        rows = [offset] * 20
        original = copy.deepcopy(rows)
        engine = trace_engine(2, rows, offset=offset)
        self.assertEqual(engine.filtered_xyz, (0., 0., 0.))
        self.assertEqual(engine.level_g, 0.)
        self.assertIsNone(engine.decision)
        when = 20. / 12500.
        engine.feed(when, (offset[0] - 2., offset[1], offset[2]), 20)
        alpha = (1. / 12500.) / (.0002 + 1. / 12500.)
        self.assertAlmostEqual(engine.filtered_xyz[0], 2. * alpha)
        self.assertEqual(engine.filtered_xyz[1:], (0., 0.))
        self.assertEqual(rows, original)

    def test_moving_noise_and_stationary_reference_keep_original_signed_values(self):
        rows = [(.1 * (-1 if i % 2 else 1), .02 * (-1 if i % 3 else 1), 1.) for i in range(80)]
        signed, rectified = [trace_engine(version, rows, threshold=5.) for version in (1, 2)]
        self.assertEqual(signed.moving, rectified.moving)
        self.assertTrue(any(row[0] < 0 for row in rectified.moving))
        self.assertTrue(any(row[0] > 0 for row in rectified.moving))
        self.assertEqual(robust_reference(rows, {"version": 1}), robust_reference(rows, {"version": 2}))
        self.assertEqual(signed.diagnostics()["moving_noise"], rectified.diagnostics()["moving_noise"])
        self.assertFalse(rectified.diagnostics()["threshold_adaptation"])

    def test_both_versions_require_two_support_rows_and_bound_buffers(self):
        for version in (1, 2):
            rows = [(0., 0., 1.)] * 8 + [(-20., 0., 1.)] + [(0., 0., 1.)] * 100
            engine = trace_engine(version, rows)
            self.assertIsNone(engine.decision)
            self.assertGreater(engine.rejected_candidates, 0)
            for index in range(20000):
                engine.feed((109 + index) / 12500., (.01, -.01, 1.), 109 + index)
            self.assertLessEqual(len(engine.history), 128)
            self.assertLessEqual(len(engine.moving), 256)


class RectifiedReplayTests(unittest.TestCase):
    def test_native_gap_display_cutoff_is_monotone_but_original_time_qualifies_decisions(self):
        capture, record = native_gap_remap_capture()
        analysis = compare_filter_capture(capture, 1., 1.)
        original_cutoff = analysis["decision_cutoff_original_time_s"]
        following = capture["_native_source_batches"][-1]
        old_extrapolation = following["start_time_s"] + original_cutoff - following["filter_start_time_s"]
        self.assertLess(old_extrapolation, record["confirmation_time_s"])
        self.assertLess(record["trigger_confirmation_time_s"], original_cutoff)
        self.assertLess(record["confirmation_time_s"], analysis["decision_cutoff_time_s"])
        self.assertLess(analysis["decision_cutoff_time_s"], following["start_time_s"])
        self.assertTrue(analysis["decision_cutoff_display_is_approximate"])
        self.assertTrue(all(variant["decision"]["decision_observed"] for variant in analysis["variants"].values()))

    def test_same_signed_raw_rows_give_separate_censored_variant_decisions(self):
        capture = comparison_capture(stop=.010)
        for index in range(50, 125):
            capture["samples"][index] = (capture["samples"][index][0],
                                         2. * (-1 if index % 2 else 1), 0., 1.)
        original = copy.deepcopy(capture["samples"])
        result = compare_filter_capture(capture, 1., 1.)
        self.assertIsNone(result["actual_variant"])
        self.assertTrue(result["variants"]["signed_v1"]["decision"]["censored"])
        self.assertTrue(result["variants"]["rectified_v2"]["decision"]["decision_observed"])
        self.assertEqual(capture["samples"], original)
        self.assertTrue(any(row[1] < 0 for row in result["raw_trace"]))
        for variant in result["variants"].values():
            self.assertEqual([row[0] for row in variant["trace"]], [row[0] for row in result["raw_trace"]])
            self.assertEqual(variant["threshold_source"], "same_numeric_threshold_diagnostic_not_calibrated")
            self.assertFalse(variant["decision"]["physical_accuracy"])

    def test_native_both_versions_preserve_proof_and_nonnegative_mapped_envelope(self):
        for version in (1, 2):
            capture, record = native_filtered_capture(filter_config={"version": version},
                clock_hz=32000000., sample_period=.000302, split=4, clock_shift=-.001,
                axes_map=((2, -1), (0, -1), (1, 1)))
            self.assertEqual(record["detector_version"], version)
            self.assertIsNone(validate_filtered_capture_prefix(capture, record))
            result = compare_filter_capture(capture, 1., 1.)
            key = "signed_v1" if version == 1 else "rectified_v2"
            self.assertEqual(result["actual_variant"], key)
            self.assertEqual(result["variants"][key]["decision"]["confirmation_time_s"], record["confirmation_time_s"])
            self.assertEqual(result["variants"][key]["threshold_source"], "actual_active_detector")
            self.assertTrue(all(value >= 0 for row in result["variants"]["rectified_v2"]["trace"] for value in row[1:]))
            self.assertTrue(any(value < 0 for row in result["variants"]["signed_v1"]["trace"] for value in row[1:]))
            for variant in result["variants"].values():
                self.assertEqual(variant["decision_cutoff_reason"], result["decision_cutoff_reason"])
                self.assertEqual(variant["reference"], result["reference"])

    def test_actual_profile_label_cannot_be_reused_with_other_threshold_or_config(self):
        capture, _record = native_filtered_capture()
        with self.assertRaisesRegex(ValueError, "actual filtered threshold"):
            compare_filter_capture(capture, 1., 1.25)
        with self.assertRaisesRegex(ValueError, "actual filtered configuration"):
            compare_filter_capture(capture, 1., 1., {"version": 1})

    def test_marker_before_confirmation_is_bounded_and_can_confirm_in_both_versions(self):
        for version in (1, 2):
            for value, cause in ((32767, "native_clip_sentinel"),):
                capture, record = native_filtered_capture(clip_position=0, clip_value=value,
                    filter_config={"version": version}, clock_hz=32000000., sample_period=.000302)
                self.assertEqual(record["result"], "impact")
                self.assertIsNone(record["sample_fault"])
                self.assertEqual(record["native_clip_decode"]["marker_axis_values"], 1)
                self.assertIsNotNone(record["confirmation_time_s"])
                self.assertIsNone(validate_filtered_capture_prefix(capture, record))


class RectifiedProfileTests(unittest.TestCase):
    def model(self, rig, version, threshold=2.25):
        metadata = test_filter_engine.ProfileTests().metadata(rig, "filtered", version=version)
        model = AccelHomingModel(Config(None, {"version": 2, "x_threshold": threshold,
            "x_successes": 10, "x_metadata": json.dumps(metadata)}, "accel_homing_model default"))
        rig.workflow.model = model
        rig.owner.threshold_g["x"] = threshold
        return model

    def test_both_models_roundtrip_without_upgrading_legacy_parameters(self):
        for version in (1, 2):
            rig = Rig()
            model = self.model(rig, version)
            self.assertEqual(rig.workflow.filter_config("x"), detector_config({"version": version}))
            self.assertEqual(model.metadata["x"]["detector_version"], version)
            self.assertEqual("signal_mode" in model.metadata["x"]["detector_config"], version == 2)
            metadata = dict(model.metadata["x"], signal_mode="rectified" if version == 1 else "signed")
            with self.assertRaises(ConfigError):
                AccelHomingModel(Config(None, {"version": 2, "x_threshold": 2.25,
                    "x_successes": 10, "x_metadata": json.dumps(metadata)}, "accel_homing_model default"))

    def test_user_calibration_inferred_or_explicit_filtered_always_uses_current_version(self):
        for explicit in (False, True):
            rig = Rig()
            self.model(rig, 1)
            self.assertEqual(rig.workflow.filter_config("x")["version"], 1)
            command = GCmd(SAVE_CONFIG=0, **({"DETECTOR": "FILTERED"} if explicit else {}))
            rig.workflow._begin(command, "calibration", "x")
            self.assertEqual(rig.workflow.session["detector_types"]["x"], "filtered")
            self.assertEqual(rig.workflow.filter_config("x")["version"], 2)
            self.assertTrue(any("FILTERED v2 (rectified)" in line for line in rig.output))

    def test_inferred_raw_calibration_does_not_switch_detector_type(self):
        rig = Rig()
        rig.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
        self.assertEqual(rig.workflow.session["detector_types"]["x"], "raw")

    def test_saved_legacy_accuracy_uses_signed_algorithm_and_reports_it(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = FilteredRig({"charts_directory": directory})
            self.model(rig, 1)
            rig.accuracy(DETECTOR="FILTERED")
            trials = rig.workflow.last_result["axis_results"]["x"]["trials"]
            self.assertEqual({trial["detector_version"] for trial in trials}, {1})
            self.assertTrue(all(trial["detector_config"] == detector_config({"version": 1}) for trial in trials))
            self.assertTrue(any("FILTERED v1 (signed)" in line for line in rig.output))

    def test_accuracy_recovery_preserves_source_algorithm_and_single_margin(self):
        for version in (1, 2):
            with tempfile.TemporaryDirectory() as directory:
                rig = FilteredRig({"charts_directory": directory})
                self.model(rig, version)
                rig.home_results.append({"travel_mm": 51.175, "halt_travel_mm_signed": -53.175})
                rig.accuracy(DETECTOR="FILTERED")
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
                self.assertEqual(metadata["detector_version"], version)
                self.assertEqual(metadata["detector_config"], detector_config({"version": version}))
                self.assertEqual(metadata["signal_mode"], "signed" if version == 1 else "rectified")
                self.assertEqual(metadata["tested_threshold_g"], 2.5)
                self.assertEqual(metadata["saved_threshold_g"], 3.)
                self.assertEqual(metadata["threshold_margin_g"], .5)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25] + [2.5] * 11)


if __name__ == "__main__":
    unittest.main()
