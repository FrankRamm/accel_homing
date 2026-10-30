"""Synthetic causal-signal semantics; no claims about physical contact or mm."""
import copy
import json
import math
import types
import unittest
import tempfile
from unittest import mock

from accel_homing_filters import (CausalImpactDetector, robust_reference,
                                  detector_config, compare_filter_capture)
from harness import CommandError, ConfigError, arm_axis, batch, make_homing, plugin
from test_simple_calibration import prepared
from test_workflows import Config, GCmd, Rig
from accel_homing_model import AccelHomingModel
from test_accuracy_recovery import RecoveryRig


def run_trace(rate, values, config=None):
    engine = CausalImpactDetector(1., (0., 0., 1.), config=config)
    engine.previous_time = -1. / rate
    for index, xyz in enumerate(values):
        engine.feed(index / rate, xyz)
    return engine


class SignalTests(unittest.TestCase):
    def test_robust_reference_resists_sparse_offset_contamination_and_bounds_weights(self):
        rows = [(.01 * (-1 if i % 2 else 1), .20 * (-1 if i % 2 else 1), 1.) for i in range(101)]
        rows[50] = (30., 30., 30.)
        ref = robust_reference(rows)
        self.assertAlmostEqual(ref["offset_g"][0], .01)
        self.assertEqual(ref["offset_g"][2], 1.)
        self.assertGreater(sum(row[2] for row in rows) / len(rows), 1.2)
        self.assertTrue(all(.75 <= w <= 1. for w in ref["weights_xyz"]))
        self.assertFalse(ref["threshold_adaptation"])

    def test_filter_is_causal_on_xyz_before_norm_and_ablation_changes_metric(self):
        values = [(2. * (-1 if i % 2 else 1), 0., 1.) for i in range(30)]
        filtered = run_trace(12500., values, {"version": 1})
        unfiltered = run_trace(12500., values, {"version": 1, "lowpass_tau_s": 0.})
        self.assertLess(filtered.level_g, .5)
        self.assertEqual(unfiltered.level_g, 2.)
        prefix = run_trace(12500., values[:12], {"version": 1})
        alternate = run_trace(12500., values[:12] + [(10., 10., 10.)] * 8, {"version": 1})
        same_row = next(row for row in alternate.history if row[0] == prefix.history[-1][0])
        self.assertAlmostEqual(prefix.history[-1][1], same_row[1])

    def test_single_large_sample_is_rejected_even_when_filter_tail_exceeds_threshold(self):
        for rate in (800., 3200., 12500., 25600.):
            with self.subTest(rate=rate):
                engine = run_trace(rate, [(0., 0., 1.)] * 8 + [(20., 0., 1.)] + [(0., 0., 1.)] * 60)
                self.assertIsNone(engine.decision)
                self.assertGreaterEqual(engine.rejected_candidates, 1)
                # Such a narrow pulse could also be real contact. Rejecting it
                # is a limitation of this experiment, not a truth label.

    def test_sustained_contact_confirms_in_bounded_sensor_time_at_several_rates(self):
        for rate in (800., 3200., 12500., 25600.):
            for channel in range(3):
                values = [(0., 0., 1.)] * 8
                impact = [0., 0., 1.]
                impact[channel] += 2.
                values += [tuple(impact)] * 100
                engine = run_trace(rate, values)
                self.assertIsNotNone(engine.decision)
                event = engine.decision
                self.assertLessEqual(event["onset_time_s"], event["crossing_time_s"])
                self.assertGreater(event["confirmation_time_s"], event["crossing_time_s"])
                self.assertLessEqual(event["confirmation_delay_s"], .0015 + 1e-12)
                self.assertGreaterEqual(event["support_samples"], 2)

    def test_offset_weights_threshold_frozen_after_candidate_and_buffers_bounded(self):
        engine = run_trace(12500., [(0.01, 0., 1.)] * 400 + [(2., 0., 1.)] * 30)
        before = (engine.offset_g, engine.weights_xyz, engine.threshold_g, list(engine.moving))
        for i in range(20000):
            engine.feed((430 + i) / 12500., (10., 8., 4.))
        self.assertEqual(before, (engine.offset_g, engine.weights_xyz, engine.threshold_g, list(engine.moving)))
        self.assertLessEqual(len(engine.history), 128)
        self.assertLessEqual(len(engine.moving), 256)
        self.assertTrue(engine.diagnostics()["moving_noise"]["censored"])

    def test_config_rejects_unknown_version_nonfinite_and_unbounded_confirmation(self):
        for config in ({"version": True}, {"version": 3}, {"lowpass_tau_s": float("nan")},
                       {"confirmation_max_s": .1}, {"weight_floor": .5}, {"foo": 1}):
            with self.assertRaises(ValueError):
                detector_config(config)


class RuntimeTests(unittest.TestCase):
    def filtered(self):
        obj, attempt = prepared()
        attempt.detector_type = "filtered"
        attempt.filter_engine = CausalImpactDetector(1., tuple(v * attempt.count_g for v in attempt.bases))
        attempt.filter_reference = robust_reference([tuple(v * attempt.count_g for v in attempt.bases)] * 50)
        return obj, attempt

    def packet(self, obj, attempt, start, rows):
        params = {"start_clock": round(start * obj.beacon_freq),
                  "delta_clock": round((len(rows) - 1) / 3200. * obj.beacon_freq),
                  "data": tuple(v for row in rows for v in row)}
        obj.reactor.now = start + (len(rows) - 1) / 3200. + .001
        obj._inspect(params)
        return params

    def test_confirmation_spans_packets_and_onset_identity_refers_to_earlier_packet(self):
        obj, attempt = self.filtered()
        low = tuple(attempt.bases)
        high = tuple(v + (2. / attempt.count_g if i == 1 else 0) for i, v in enumerate(low))
        t = attempt.arm_time + .0001
        first = self.packet(obj, attempt, t, [low] * 7 + [high])
        self.assertFalse(attempt.done)
        self.packet(obj, attempt, t + 8 / 3200., [high] * 8)
        self.assertEqual(attempt.reason, "impact")
        self.assertEqual(attempt.onset_sample[0], first["start_clock"])
        self.assertLess(attempt.filter_decision["crossing_time_s"], attempt.confirmation_time)
        self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)

    def test_filtered_reference_is_real_prepare_choice_and_fixed_during_motion(self):
        # The production preparation path, rather than an unused helper.
        from test_backends import RaisedBaselineFloorTest
        obj, endstop = RaisedBaselineFloorTest().machine(threshold=1.)
        obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x",
                                 "algorithm": "impact20_vector_v2", "detector_types": {"x": "filtered"}}
        obj.workflows.in_home, obj.workflows.home_axis = True, "x"
        obj._prepare_reference()
        endstop.get_steppers = lambda: []
        attempt = obj.prepare(endstop, obj.backend.last_sample_time + .001)
        self.assertEqual(attempt.detector_type, "filtered")
        self.assertIsNotNone(attempt.filter_engine)
        self.assertEqual(attempt.filter_engine.offset_g, tuple(attempt.filter_reference["offset_g"]))

    def test_configured_backend_capture_preserves_exact_reference_and_replays_same_decision(self):
        from test_backends import RaisedBaselineFloorTest, feed
        obj, endstop = RaisedBaselineFloorTest().machine(threshold=1.)
        obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x",
                                 "algorithm": "impact20_vector_v2", "detector_types": {"x": "filtered"}}
        obj.workflows.in_home, obj.workflows.home_axis = True, "x"
        obj.start_capture()
        obj._prepare_reference()
        endstop.get_steppers = lambda: []
        when = obj.backend.last_sample_time + 1 / 3200.
        attempt = obj.prepare(endstop, when)
        obj.arm(attempt)
        base = tuple(v * attempt.count_g for v in attempt.bases)
        rows = [(when + index / 3200., base[0] * 9806.65,
                 (base[1] + (2. if index >= 8 else 0.)) * 9806.65, base[2] * 9806.65)
                for index in range(32)]
        feed(obj, rows)
        self.assertEqual(attempt.reason, "impact")
        obj.disarm()
        capture = obj.stop_capture()
        detector = capture["detector"]
        self.assertEqual(detector["reference_sample_matches"], [1, 1])
        self.assertEqual(detector["reference_sample_count"], detector["reference_sample_indices"][1]
                         - detector["reference_sample_indices"][0] + 1)
        result = compare_filter_capture(capture, 1., 1.)
        self.assertTrue(result["filtered_decision"]["decision_observed"])
        self.assertAlmostEqual(result["filtered_decision"]["onset_time_s"], detector["onset_time_s"], places=10)
        self.assertAlmostEqual(result["filtered_decision"]["confirmation_time_s"], detector["confirmation_time_s"], places=10)

    def test_native_marker_is_decoded_without_unconditional_filtered_impact(self):
        obj = make_homing(threshold=1.)
        attempt, sent = arm_axis(obj, "x", 1000, 2000)
        attempt.vector_mode, attempt.detector_type = True, "filtered"
        attempt.bases = (0., 0., 128.)
        attempt.filter_engine = CausalImpactDetector(1., (0., 0., 1.))
        obj.reactor.now = 1.008
        obj._inspect(batch(1001, [(0, 0, 128), (32767, 0, 128)] + [(0, 0, 128)] * 6))
        self.assertFalse(attempt.done)
        self.assertIsNone(attempt.sample_fault)
        self.assertEqual(attempt.marker_axis_values, 1)
        self.assertEqual(len(sent.messages), 0)

    def test_single_armed_row_from_straddling_packet_retains_original_period(self):
        obj, attempt = self.filtered()
        low = tuple(attempt.bases)
        start = attempt.arm_time - 7 / 3200.
        self.packet(obj, attempt, start, [low] * 8)
        self.assertFalse(attempt.done, attempt.reason)
        self.assertAlmostEqual(attempt.sample_dt, 1. / 3200., delta=.000001)
        self.packet(obj, attempt, attempt.arm_time + 1 / 3200., [low] * 8)
        self.assertFalse(attempt.done, attempt.reason)

    def test_dispatch_occurs_before_even_failing_timestamp_diagnostics(self):
        obj, attempt = self.filtered()
        def stamp(_when):
            self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)
            raise RuntimeError("diagnostic clock unavailable")
        obj.main_mcu.estimated_print_time = stamp
        obj._fire(attempt, "impact", plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT)
        self.assertTrue(attempt.done)
        self.assertIsNone(attempt.stop_request_upper)

    def test_decoded_raw_marker_capture_still_needs_reference_before_filtered_shadow(self):
        obj = make_homing(threshold=1.)
        obj.start_capture()
        attempt, _sent = arm_axis(obj, "x", 1000, 2000)
        attempt.vector_mode = True
        attempt.bases = (0., 0., 128.)
        obj.reactor.now = 1.008
        obj._inspect(batch(1001, [(0, 0, 128), (32767, 0, 128)] + [(0, 0, 128)] * 6))
        self.assertEqual(attempt.reason, "impact")
        obj.disarm()
        capture = obj.stop_capture()
        self.assertTrue(capture["raw_integrity"]["raw_samples_validated"])
        self.assertEqual(capture["raw_integrity"]["clipped_axis_values"], 1)
        self.assertEqual(capture["native_clip_decode"]["marker_axis_values"], 1)
        with self.assertRaisesRegex(ValueError, "reference"):
            compare_filter_capture(capture, 1., 1.)


def comparison_capture(stop=.006):
    rows = [(i / 12500., .01 * (-1 if i % 2 else 1), 0., 1.) for i in range(200)]
    for i in range(50, 75):
        rows[i] = (i / 12500., 2., 0., 1.)
    return {"samples": rows, "baseline_g": (0., 0., 1.), "raw_baseline_g": (0., 0., 1.),
            "detector": {"arm_time_s": .003, "detector_type": "raw", "impact_metric": "xyz_magnitude",
                         "reference_interval_s": [0., .00296], "reference_sample_count": 38,
                         "stop_request_time_s": stop + .0001, "stop_request_interval_s": [stop, stop + .0001]}}


class ReplayTests(unittest.TestCase):
    def test_same_sample_signed_traces_prearm_are_real_warmed_data(self):
        result = compare_filter_capture(comparison_capture(), 1., 1.)
        self.assertEqual(len(result["raw_trace"]), len(result["filtered_trace"]))
        self.assertTrue(any(abs(row[1]) > 0 for row in result["filtered_trace"] if row[0] < .003))
        self.assertTrue(result["pre_arm_filter_replay"])
        self.assertTrue(result["filtered_decision"]["decision_observed"])
        self.assertFalse(result["physical_accuracy_verified"])

    def test_no_post_request_samples_can_confirm_shadow_event(self):
        capture = comparison_capture(stop=.0044)
        result = compare_filter_capture(capture, 1., 1.)
        self.assertTrue(result["raw_decision"]["decision_observed"])
        self.assertFalse(result["filtered_decision"]["decision_observed"])
        self.assertTrue(result["filtered_decision"]["censored"])
        self.assertIn("crossing_time_s", result["filtered_decision"])
        self.assertIsNone(result["filtered_decision"]["confirmation_time_s"])

    def test_unknown_stop_or_invalid_samples_are_not_shadow_evidence(self):
        capture = comparison_capture()
        capture["detector"].pop("stop_request_interval_s")
        result = compare_filter_capture(capture, 1., 1.)
        self.assertFalse(result["raw_decision"]["decision_observed"])
        for mutate in (lambda c: c.update(fault="sample_gap"),
                       lambda c: c["samples"].__setitem__(5, c["samples"][4]),
                       lambda c: c["detector"].update(impact_metric="selected_axis")):
            invalid = comparison_capture()
            mutate(invalid)
            with self.assertRaises(ValueError):
                compare_filter_capture(invalid, 1., 1.)

    def test_raw_shadow_uses_original_mean_instead_of_filtered_median(self):
        capture = comparison_capture()
        capture["baseline_g"] = (5., 0., 1.)
        result = compare_filter_capture(capture, 1., 1.)
        self.assertAlmostEqual(result["raw_trace"][0][1], .01)


class ProfileTests(unittest.TestCase):
    def metadata(self, rig, kind, version=2):
        data = rig.workflow._model_context("x")
        data.update(detector_type=kind, impact_metric="xyz_magnitude")
        if kind == "filtered":
            data.update(detector_version=version, detector_config=detector_config({"version": version}),
                        filter_sensor_context=rig.owner.noise_sensor_context())
        return data

    def test_legacy_models_stay_raw_and_filtered_needs_own_threshold(self):
        rig = Rig()
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 3.}, metadata={"x": self.metadata(rig, "raw")})
        self.assertEqual(rig.workflow.detector_type("x"), "raw")
        with self.assertRaisesRegex(CommandError, "No calibrated FILTERED"):
            rig.workflow._begin(GCmd(DETECTOR="FILTERED"), "accuracy", axes=["x"])
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.owner.threshold_g["x"], 8.)

    def test_raw_and_filtered_profiles_select_independent_thresholds_and_restore(self):
        rig = Rig()
        meta = self.metadata(rig, "filtered")
        meta["detector_profiles"] = {"raw": {"threshold_g": 3., "metadata": self.metadata(rig, "raw")}}
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": meta})
        rig.owner.threshold_g["x"] = 2.
        self.assertEqual(rig.workflow.detector_type("x"), "filtered")
        rig.workflow._begin(GCmd(DETECTOR="RAW"), "charts", axes=["x"])
        self.assertEqual(rig.owner.threshold_g["x"], 3.)
        self.assertEqual(rig.workflow.detector_type("x"), "raw")
        rig.workflow._finish({"result": "aborted"})
        self.assertEqual(rig.owner.threshold_g["x"], 2.)
        self.assertEqual(rig.workflow.detector_type("x"), "filtered")

    def test_omitted_calibration_preserves_active_filtered_detector(self):
        rig = Rig()
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": self.metadata(rig, "filtered")})
        rig.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
        self.assertEqual(rig.workflow.session["detector_types"]["x"], "filtered")

    def test_saved_filtered_metadata_rejects_missing_config_or_recursive_profiles(self):
        for meta in ({"detector_type": "filtered"},
                     {"detector_type": "filtered", "detector_version": 2, "detector_config": detector_config()},
                     {"detector_profiles": {"raw": {"threshold_g": 2., "metadata": {"detector_profiles": {}}}}}):
            with self.assertRaises(ConfigError):
                AccelHomingModel(Config(None, {"x_threshold": 2., "x_successes": 10,
                                               "x_metadata": json.dumps(meta)}, "accel_homing_model default"))

    def test_filtered_saved_metric_and_full_exact_config_are_required(self):
        rig = Rig()
        for update in ({"impact_metric": "selected_axis"}, {"impact_metric": None},
                       {"detector_config": None}, {"detector_config": {"version": 1}},
                       {"detector_version": True}):
            metadata = self.metadata(rig, "filtered")
            metadata.update(update)
            with self.assertRaises(ConfigError):
                AccelHomingModel(Config(None, {"x_threshold": 2., "x_successes": 10,
                                               "x_metadata": json.dumps(metadata)}, "accel_homing_model default"))

    def test_filter_chart_missing_shadow_threshold_rejects_before_home(self):
        rig = Rig()
        with self.assertRaisesRegex(CommandError, "XYZ-calibrated RAW"):
            rig.workflow.cmd_FILTER_CHARTS(GCmd(AXIS="X", CENTERED=1))
        self.assertEqual(rig.home_calls, [])

    def test_raw_legacy_profile_retains_selected_axis_metric(self):
        rig = Rig()
        meta = self.metadata(rig, "raw")
        meta["impact_metric"] = "selected_axis"
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 3.}, metadata={"x": meta})
        rig.workflow._begin(GCmd(DETECTOR="RAW"), "test", axes=["x"])
        self.assertFalse(rig.workflow.simple_detection("x"))

    def test_manual_config_override_blocks_filtered_calibration_before_motion(self):
        rig = Rig({"x_threshold": 3.})
        with self.assertRaisesRegex(CommandError, "removing the explicit threshold"):
            rig.calibrate(save=0, DETECTOR="FILTERED")
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.noise_calls, [])


class FilteredRig(RecoveryRig):
    """Workflow fixture with explicit experimental detector evidence."""
    def stock_home(self, gcmd):
        if self.workflow.session is not None:
            axis = next(a.lower() for a in "XY" if gcmd.get(a, None) is not None)
            if not self.home_results:
                self.home_results.append({})
            if isinstance(self.home_results[0], dict):
                kind = self.workflow.detector_type(axis)
                self.home_results[0].update(detector_type=kind)
                if self.workflow.simple_detection(axis):
                    self.home_results[0].update(impact_metric="xyz_magnitude", direction=0)
                if kind == "filtered":
                    config = self.workflow.filter_config(axis)
                    self.home_results[0].update(detector_version=config["version"], detector_config=config,
                                                filter_reference=robust_reference([(0., 0., 1.)] * 50, config))
        return super().stock_home(gcmd)

    def stop_capture(self):
        capture = super().stop_capture()
        detector = capture.get("detector")
        if detector and capture["_record"].get("detector_type") == "filtered":
            onset = detector["onset_time_s"]
            config = self.workflow.filter_config(capture["_record"]["axis"])
            detector.update(detector_type="filtered", detector_version=config["version"],
                            detector_config=config,
                            filter_reference=robust_reference([(0., 0., 1.)] * 50, config),
                            crossing_time_s=onset + .0001, confirmation_time_s=onset + .0005,
                            stop_request_time_s=onset + .001,
                            stop_request_interval_s=[onset + .0007, onset + .001])
        return capture


class FilteredWorkflowTests(unittest.TestCase):
    def test_filtered_then_other_raw_axis_staged_before_restart_retains_schema_two(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = FilteredRig({"charts_directory": directory})
            rig.calibrate(save=0, DETECTOR="FILTERED")
            self.assertEqual(rig.configfile.saved[("accel_homing_model default", "version")], "2")
            rig.toolhead.kin.limits[0] = rig.toolhead.kin.limits[1] = (1., -1.)
            rig.toolhead.position[:2] = [150., 100.]
            rig.idle_state = "Ready"
            rig.calibrate(axis="Y", save=0, DETECTOR="RAW")
            saved = rig.configfile.saved
            self.assertEqual(saved[("accel_homing_model default", "version")], "2")
            options = {key: value for (section, key), value in saved.items() if section == "accel_homing_model default"}
            loaded = AccelHomingModel(Config(None, options, "accel_homing_model default"))
            self.assertEqual(loaded.version, 2)
            self.assertEqual(loaded.metadata["x"]["detector_type"], "filtered")
            self.assertEqual(loaded.metadata["y"]["detector_type"], "raw")
            self.assertEqual(loaded.thresholds, {"x": 1.5, "y": 1.5})

    def test_active_filtered_schema_one_is_rejected_before_threshold_can_load(self):
        rig = Rig()
        metadata = ProfileTests().metadata(rig, "filtered")
        with self.assertRaisesRegex(ConfigError, "schema version 2"):
            AccelHomingModel(Config(None, {"version": 1, "x_threshold": 2., "x_successes": 10,
                                           "x_metadata": json.dumps(metadata)}, "accel_homing_model default"))

    def test_filtered_calibration_archives_raw_profile_and_persists_one_margin(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = FilteredRig({"charts_directory": directory})
            raw = ProfileTests().metadata(rig, "raw")
            rig.workflow.model = types.SimpleNamespace(thresholds={"x": 3.25}, metadata={"x": raw})
            rig.calibrate(save=0, DETECTOR="FILTERED")
            saved = rig.configfile.saved
            metadata = json.loads(saved[("accel_homing_model default", "x_metadata")])
            self.assertEqual(metadata["detector_type"], "filtered")
            self.assertEqual(metadata["detector_profiles"]["raw"]["threshold_g"], 3.25)
            self.assertEqual(float(saved[("accel_homing_model default", "x_threshold")]), 1.5)
            self.assertFalse(metadata["saved_threshold_tested"])
            self.assertEqual(metadata["tested_threshold_g"], 1.)
            self.assertEqual(metadata["threshold_margin_g"], .5)
            self.assertEqual(len(metadata["filter_reference_summaries"]), 10)
            self.assertEqual([c["threshold"] for c in rig.home_calls], [1.] * 11)
            loaded = AccelHomingModel(Config(None, {
                "version": 2,
                "x_threshold": saved[("accel_homing_model default", "x_threshold")], "x_successes": 10,
                "x_metadata": saved[("accel_homing_model default", "x_metadata")]}, "accel_homing_model default"))
            rig.workflow.model = loaded
            self.assertEqual(rig.workflow._profile("x", "raw")["threshold_g"], 3.25)
            self.assertEqual(rig.workflow._profile("x", "filtered")["threshold_g"], 1.5)

    def test_explicit_filtered_accuracy_restores_raw_settings_and_uses_same_mode_series(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = FilteredRig({"charts_directory": directory})
            rig.accuracy(DETECTOR="FILTERED", FILTERED_THRESHOLD=2.25)
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertEqual(result["fixed_thresholds_g"], {"x": 2.25})
            self.assertEqual(result["detector_types"], {"x": "filtered"})
            self.assertEqual({r["detector_type"] for r in result["axis_results"]["x"]["trials"]}, {"filtered"})
            self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
            self.assertEqual(rig.workflow.detector_type("x"), "raw")

    def test_filtered_early_accuracy_recovery_keeps_detector_and_quarter_then_margin(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = FilteredRig({"charts_directory": directory})
            rig.home_results.append({"travel_mm": 51.175, "halt_travel_mm_signed": -53.175})
            rig.accuracy(DETECTOR="FILTERED", FILTERED_THRESHOLD=2.25)
            self.assertEqual([c["threshold"] for c in rig.home_calls], [2.25] + [2.5] * 11)
            metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            self.assertEqual(metadata["detector_type"], "filtered")
            self.assertEqual(metadata["tested_threshold_g"], 2.5)
            self.assertEqual(metadata["saved_threshold_g"], 3.)
            self.assertEqual(rig.workflow.last_result["accuracy_recovery"]["detector_type"], "filtered")

    def test_missing_accuracy_baseline_fails_before_homing(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = FilteredRig({"charts_directory": directory})
            with self.assertRaisesRegex(CommandError, "BASELINE is not an available completed"):
                rig.accuracy(BASELINE="a" * 32)
            self.assertEqual(rig.home_calls, [])


class ComparisonRig(FilteredRig):
    def stop_capture(self):
        capture = super().stop_capture()
        detector = capture.get("detector")
        if detector is None:
            return capture
        record = capture["_record"]
        base, count, rate = self.sensor_time, 2000, 12500.
        rows = [(base + i / rate, .01 * (-1 if i % 2 else 1), 0., 1.) for i in range(count)]
        for index in range(1563, 1610):
            rows[index] = (rows[index][0], 4., 0., 1.)
        onset = rows[1563][0]
        stamp = (round(base * 1e6), 159920, 1563, count)
        record.update(arm_time_s=base + .12, onset_time_s=onset, onset_sample=stamp,
                      onset_g=4., impact_peak_g=4., direction=0, impact_metric="xyz_magnitude")
        detector.update(arm_time_s=record["arm_time_s"], onset_time_s=onset,
                        trigger_onset_time_s=onset, onset_sample=stamp, onset_sample_index=1563,
                        onset_sample_matches=1, onset_g=4., impact_peak_g=4., direction=0,
                        impact_metric="xyz_magnitude", detector_type=record["detector_type"],
                        stop_request_time_s=base + .127,
                        stop_request_interval_s=[base + .1265, base + .127],
                        reference_interval_s=[rows[125][0], rows[1375][0]],
                        reference_sample_count=1251, sample_period_s=1. / rate)
        capture.update(samples=rows, raw_baseline_g=(0., 0., 1.), baseline_g=(0., 0., 1.),
                       raw_integrity={"raw_samples_validated": True})
        return capture


class ComparisonWorkflowTests(unittest.TestCase):
    def test_manual_raw_axis_requires_explicit_vector_selection_before_any_home(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ComparisonRig({"charts_directory": directory, "x_threshold": 3.})
            raw, filtered = ProfileTests().metadata(rig, "raw"), ProfileTests().metadata(rig, "filtered")
            raw["detector_profiles"] = {"filtered": {"threshold_g": 1., "metadata": filtered}}
            rig.workflow.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": raw})
            with self.assertRaisesRegex(CommandError, "Active RAW comparison needs an XYZ detector"):
                rig.workflow.cmd_FILTER_CHARTS(GCmd(AXIS="X", CENTERED=1))
            self.assertEqual(rig.home_calls, [])

    def test_filter_command_publishes_reviewable_paired_capture_after_full_success(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ComparisonRig({"charts_directory": directory})
            rig.workflow.cmd_FILTER_CHARTS(GCmd(AXIS="X", CENTERED=1, RAW_THRESHOLD=2., FILTERED_THRESHOLD=1.))
            self.assertEqual(rig.workflow.last_result["result"], "complete")
            payload = next(item for item in rig.workflow.renderer.calls if item["kind"] == "filter")
            self.assertEqual(payload["analysis"]["actual_detector"], "raw")
            self.assertTrue(payload["analysis"]["raw_decision"]["decision_observed"])
            self.assertTrue(payload["analysis"]["filtered_decision"]["decision_observed"])
            self.assertEqual(payload["record"]["actual_repeatability"]["accepted"], 1)
            self.assertIsNone(payload["record"]["actual_repeatability"]["spread_mm"])
            self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})

    def test_filter_command_validates_unselected_profile_context_and_uses_saved_config(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ComparisonRig({"charts_directory": directory})
            raw, filtered = ProfileTests().metadata(rig, "raw"), ProfileTests().metadata(rig, "filtered")
            filtered["detector_config"] = detector_config({"lowpass_tau_s": .0003})
            raw["detector_profiles"] = {"filtered": {"threshold_g": 1., "metadata": filtered}}
            rig.workflow.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": raw})
            rig.owner.threshold_g["x"] = 2.
            rig.workflow.cmd_FILTER_CHARTS(GCmd(AXIS="X", CENTERED=1))
            payload = next(item for item in rig.workflow.renderer.calls if item["kind"] == "filter")
            self.assertEqual(payload["analysis"]["filter_config"]["lowpass_tau_s"], .0003)
            # A changed unselected sensor profile must fail before another home.
            rig.toolhead.kin.limits[0] = (1., -1.)
            rig.home_calls.clear()
            filtered["filter_sensor_context"]["scale_id"] = 42
            with self.assertRaisesRegex(CommandError, "comparison FILTERED profile sensor"):
                rig.workflow.cmd_FILTER_CHARTS(GCmd(AXIS="X", CENTERED=1))
            self.assertEqual(rig.home_calls, [])


if __name__ == "__main__":
    unittest.main()
