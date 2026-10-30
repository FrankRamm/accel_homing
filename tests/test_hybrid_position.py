"""Causal HYBRID attribution; synthetic signals never label physical contact."""
import copy
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

import accel_homing_charts as charts
import accel_homing_filters as filters
from accel_homing_workflows import AccelHomingWorkflows
from harness import CommandError, counts, plugin
from test_chronological_clipping import native_filtered_capture
import test_filter_engine as filter_tests
import test_chronological_clipping as native_tests
from test_workflows import GCmd, Rig
from test_censored_response_charts import png_bytes


def observed(values, raw_threshold=10., config=None, original_valid=None):
    config = dict(lowpass_tau_s=0., confirmation_min_s=.0003,
                  confirmation_max_s=.0015, excess_energy_s=0., **(config or {}))
    engine = filters.CausalImpactDetector(5., (0., 0., 0.), config=config)
    engine.previous_time = -.0001
    observer = filters.AssociatedRawPosition(raw_threshold, (0., 0., 0.), 1., config, 0.)
    for i, value in enumerate(values):
        stamp = (1000, 100, i, len(values))
        decision = engine.feed(i * .0001, (value, 0., 0.), stamp)
        observer.observe(i * .0001, (value, 0, 0), stamp, engine,
                         True if original_valid is None else original_valid[i])
        if decision is not None:
            return engine, observer, observer.select(decision)
    raise AssertionError("synthetic FILTERED event not accepted")


def native_hybrid_capture(raw_threshold=1.8, profile=None, **kwargs):
    profile = profile or {"threshold_g": raw_threshold, "detector_type": "raw", "impact_metric": "xyz_magnitude",
                         "source": "saved_compatible_raw_xyz_profile", "model_context": {}}
    options = dict(mode="accuracy", finish_home_wait=True, clip_position=None,
                   clock_hz=32000000., sample_period=.0003, packet_size=16,
                   row_values=[(0, 0, counts(1.))] * 3 + [(counts(3.), 0, counts(1.))] * 13,
                   filter_config={"confirmation_min_s": .0006})
    options.update(kwargs)
    with mock.patch.object(AccelHomingWorkflows, "hybrid_raw_profile", return_value=profile):
        return native_filtered_capture(**options)


def install_profiles(rig, axes=("x",), raw_threshold=4.25):
    metadata, thresholds = {}, {}
    for axis in axes:
        filtered = filter_tests.ProfileTests().metadata(rig, "filtered")
        filtered.update(rig.workflow._model_context(axis))
        raw = filter_tests.ProfileTests().metadata(rig, "raw")
        raw.update(rig.workflow._model_context(axis), impact_metric="xyz_magnitude")
        filtered["detector_profiles"] = {"raw": {"threshold_g": raw_threshold, "metadata": raw}}
        metadata[axis], thresholds[axis] = filtered, 2.25
    rig.workflow.model = types.SimpleNamespace(thresholds=thresholds, metadata=metadata)


class HybridWorkflowRig(filter_tests.FilteredRig):
    def stop_capture(self):
        existing = super().stop_capture()
        source = existing.get("_record")
        if source is None:
            return existing
        axis = source["axis"]
        threshold = self.owner.threshold_g[axis]
        make = native_tests.make_homing
        def machine(**options):
            options["threshold"] = threshold
            return make(**options)
        with mock.patch.object(native_tests, "make_homing", side_effect=machine):
            capture, record = native_hybrid_capture(axis=axis,
                profile=self.workflow.hybrid_raw_profile(axis),
                time_offset=self.capture_stops * 10.,
                row_values=[(0, 0, counts(1.))] * 3 + [(counts(6.), 0, counts(1.))] * 13,
                filter_config=self.workflow.filter_config(axis))
        preserved = {key: source[key] for key in ("start_position_mm", "expected_travel_mm", "travel_mm",
            "travel_mm_exact", "trigger_travel_mm_signed", "halt_travel_mm_signed") if key in source}
        source.update(record)
        source.update(preserved, workflow_threshold_g=threshold)
        capture["_record"] = source
        return capture


class AssociationTests(unittest.TestCase):
    def test_unique_finite_crossing_and_late_crossing_use_original_identity(self):
        for values, index in (([0, 0, 12, 12, 12, 12], 2), ([0, 0, 6, 6, 12, 12], 4)):
            engine, observer, receipt = observed(values)
            self.assertEqual(receipt["actual_source"], "raw_threshold_crossing")
            self.assertEqual(receipt["selected_sample"][2], index)
            self.assertLessEqual(receipt["selected_time_s"], engine.decision["confirmation_time_s"])
            self.assertEqual(receipt["selected_raw_sensor_counts"], [values[index], 0, 0])
            self.assertTrue(observer.frozen)

    def test_filtered_control_is_identical_with_observer(self):
        values = [0, 12, 0, 0, 12, 12, 12, 12]
        engine, _observer, receipt = observed(values)
        plain = filters.CausalImpactDetector(5., (0., 0., 0.), config=engine.config)
        plain.previous_time = -.0001
        for i, value in enumerate(values):
            decision = plain.feed(i * .0001, (value, 0., 0.), (1000, 100, i, len(values)))
            if decision is not None:
                break
        self.assertEqual(engine.decision, plain.decision)
        self.assertEqual(engine.diagnostics(), plain.diagnostics())
        self.assertEqual(receipt["selected_sample"][2], 4)
        self.assertEqual(receipt["rejected_candidate_cutoff_s"], .0002)

    def test_isolated_spike_and_rejected_candidate_never_supply_later_position(self):
        engine, _observer, receipt = observed([0, 12, 0, 0, 6, 6, 6, 6])
        self.assertEqual(engine.rejected_candidates, 1)
        self.assertEqual(receipt["fallback_reason"], "no_associated_raw_threshold_crossing")
        self.assertEqual(receipt["selected_time_s"], engine.decision["crossing_time_s"])
        # A small early RAW spike can remain under the FILTERED threshold, but
        # cannot be reused after the accepted candidate's above-gate gap.
        _engine, _observer, receipt = observed([0, 4, 0, 0, 6, 6, 6, 6], raw_threshold=3.)
        self.assertEqual(receipt["selected_sample"][2], 4)

    def test_multiple_edges_are_ambiguous_and_already_high_has_no_invented_edge(self):
        _engine, _observer, receipt = observed([0, 6, 12, 6, 12, 12, 12])
        self.assertEqual(receipt["raw_crossing_count"], 2)
        self.assertEqual(receipt["fallback_reason"], "multiple_raw_threshold_crossings")
        self.assertTrue(receipt["ambiguous"])
        _engine, _observer, receipt = observed([12, 12, 12, 12, 12])
        self.assertEqual(receipt["fallback_reason"], "no_associated_raw_threshold_crossing")

    def test_marker_missing_amplitude_and_future_rows_cannot_change_frozen_selection(self):
        engine, observer, receipt = observed([0, 0, 12, 12, 12, 12])
        snapshot = copy.deepcopy(receipt)
        observer.observe(.1, (32767, 0, 0), (3000, 100, 0, 8), engine, False)
        self.assertEqual(observer.select(engine.decision), snapshot)
        _engine, _observer, receipt = observed([0, 0, 12, 12, 12, 12], original_valid=[True] * 4 + [False, True])
        self.assertEqual(receipt["fallback_reason"], "raw_window_has_unverifiable_samples")

    def test_bounded_history_truncation_falls_back_without_reusing_old_edge(self):
        engine, observer, _receipt = observed([0, 0, 12, 12, 12, 12])
        observer.receipt = None
        observer.evicted_through_time = engine.decision["crossing_time_s"]
        self.assertEqual(observer.select(engine.decision)["fallback_reason"], "raw_association_history_truncated")
        self.assertLessEqual(len(observer.rows), filters.MAX_HYBRID_ROWS)

    def test_raw_squared_counts_boundary_uses_arithmetic_reference_not_filter_weights(self):
        engine = types.SimpleNamespace(rejected_candidates=0, decision=None)
        observer = filters.AssociatedRawPosition(1.25, (3.5, -1., 0.), .5, {}, 0.)
        observer.observe(0., (5, -1, 0), (1, 100, 0, 8), engine)
        observer.observe(.0003, (6, -1, 0), (1, 100, 1, 8), engine)
        decision = {"crossing_time_s": 0., "confirmation_time_s": .0003, "lookback_onset_time_s": 0.,
                    "crossing_identity": (1, 100, 0, 8), "onset_level_g": 1.}
        receipt = observer.select(decision)
        self.assertEqual(receipt["actual_source"], "raw_threshold_crossing")
        self.assertEqual(receipt["selected_raw_level_g"], 1.25)

    def test_wrapping_original_packet_identity_uses_monotone_original_time(self):
        engine = filters.CausalImpactDetector(5., (0., 0., 0.), config={"lowpass_tau_s": 0., "excess_energy_s": 0.})
        observer = filters.AssociatedRawPosition(10., (0., 0., 0.), 1., engine.config, 0.)
        engine.previous_time = -.0001
        for index, value in enumerate([0, 0, 12, 12, 12, 12]):
            stamp = ((0xfffffff0 + index * 32) & 0xffffffff, 100, 0, 8)
            decision = engine.feed(index * .0001, (value, 0., 0.), stamp)
            observer.observe(index * .0001, (value, 0, 0), stamp, engine)
            if decision is not None:
                break
        receipt = observer.select(decision)
        self.assertEqual(receipt["selected_sample"][0], 48)
        self.assertEqual(receipt["selected_time_s"], .0002)

    def test_native_marker_continuation_never_supplies_crossing_or_amplitude(self):
        def marker_case(values, originals, baseline=(0., 0., 0.), threshold=10.):
            engine = types.SimpleNamespace(rejected_candidates=0, decision=None)
            observer = filters.AssociatedRawPosition(threshold, baseline, 1., {}, 0., clip_values=(2047, -2048))
            for i, (value, original) in enumerate(zip(values, originals)):
                observer.observe(i * .0001, (value, 0, 0), (1000, 100, i, len(values)), engine,
                                 original != 32767, (original, 0, 0))
            decision = {"crossing_time_s": 0., "confirmation_time_s": (len(values) - 1) * .0001,
                        "lookback_onset_time_s": 0., "crossing_identity": (1000, 100, 0, len(values)), "onset_level_g": 5.}
            return observer.select(decision)
        selected = marker_case([0, 12, 2047, 12], [0, 12, 32767, 12])
        self.assertEqual(selected["actual_source"], "raw_threshold_crossing")
        self.assertEqual(selected["selected_sample"][2], 1)
        self.assertEqual(selected["marker_rows_with_proven_above_code_state"], 1)
        first = marker_case([0, 2047, 12], [0, 32767, 12])
        self.assertEqual(first["fallback_reason"], "raw_crossing_has_unverifiable_amplitude")
        self.assertEqual(first["raw_crossing_count"], 1)
        self.assertIsNone(first["selected_raw_level_g"])
        unknown = marker_case([2047, -2048, 2047], [2047, 32767, 2047], baseline=(2047., 0., 0.))
        self.assertEqual(unknown["fallback_reason"], "raw_window_has_unverifiable_samples")
        multiple = marker_case([0, 12, 2047, 0, 12], [0, 12, 32767, 0, 12])
        self.assertEqual(multiple["fallback_reason"], "multiple_raw_threshold_crossings")

    def test_mixed_xyz_marker_bound_and_finite_native_rail_codes(self):
        engine = types.SimpleNamespace(rejected_candidates=0, decision=None)
        decision = {"crossing_time_s": 0., "confirmation_time_s": .0002, "lookback_onset_time_s": 0.,
                    "crossing_identity": (1000, 100, 0, 8), "onset_level_g": 5.}
        for finite in (2047, -2048):
            observer = filters.AssociatedRawPosition(1000., (20., 5., -8.), 1., {}, 0., clip_values=(2047, -2048))
            observer.observe(0., (20, 5, -8), (1000, 100, 0, 8), engine)
            observer.observe(.0001, (finite, 305, -8), (1000, 100, 1, 8), engine)
            observer.observe(.0002, (2047, 305, -2048), (1000, 100, 2, 8), engine, False, (32767, 305, 32767))
            receipt = observer.select(decision)
            self.assertEqual(receipt["actual_source"], "raw_threshold_crossing")
            self.assertEqual(receipt["selected_raw_sensor_counts"], [finite, 305, -8])
            self.assertEqual(receipt["marker_rows_with_proven_above_code_state"], 1)


class NativeHybridTests(unittest.TestCase):
    def test_actual_home_wait_returns_earlier_or_later_raw_time_with_identical_filtered_stop(self):
        cases = ((.75, [.8, 3., 3.], .0006, -1), (2.5, [1.8, 2., 3.], .0009, 1))
        for raw_threshold, rise, minimum, direction in cases:
            rows = [(0, 0, counts(1.))] * 3 + [(counts(value), 0, counts(1.)) for value in rise]
            rows += [(counts(3.), 0, counts(1.))] * (16 - len(rows))
            common = dict(mode="accuracy", finish_home_wait=True, clip_position=None,
                clock_hz=32000000., sample_period=.0003, packet_size=16, row_values=rows,
                filter_config={"confirmation_min_s": minimum}, clock_shift=.00002)
            capture, record = native_hybrid_capture(raw_threshold, **common)
            plain_capture, plain = native_filtered_capture(**common)
            selected = record["position_selection"]
            self.assertEqual(selected["actual_source"], "raw_threshold_crossing")
            self.assertGreater(direction * (record["returned_trigger_time_s"] - plain["returned_trigger_time_s"]), 0.)
            self.assertEqual(record["returned_trigger_time_s"], selected["selected_time_s"])
            self.assertLessEqual(selected["selected_time_s"], record["trigger_confirmation_time_s"])
            for key in ("onset_sample", "crossing_sample", "confirmation_sample", "trigger_onset_time_s",
                        "trigger_crossing_time_s", "trigger_confirmation_time_s", "stop_request_interval_s",
                        "impact_peak_g", "filter_diagnostics", "lookback_onset_time_s"):
                self.assertEqual(record[key], plain[key], key)
            self.assertEqual(capture["samples"], plain_capture["samples"])
            filters.validate_filtered_capture_prefix(capture, record)
            detector = capture["detector"]
            self.assertEqual(detector["position_time_s"], capture["samples"][detector["position_sample_index"]][0])
            self.assertAlmostEqual(detector["position_time_s"] - selected["selected_time_s"], .00002)

    def test_native_original_prefix_proves_position_while_control_and_window_stay_same(self):
        capture, record = native_hybrid_capture()
        options = dict(mode="accuracy", finish_home_wait=True, clip_position=None,
                       clock_hz=32000000., sample_period=.0003, packet_size=16,
                       row_values=[(0, 0, counts(1.))] * 3 + [(counts(3.), 0, counts(1.))] * 13,
                       filter_config={"confirmation_min_s": .0006})
        pure_capture, pure = native_filtered_capture(**options)
        for key in ("onset_sample", "crossing_sample", "confirmation_sample", "trigger_onset_time_s",
                    "trigger_crossing_time_s", "trigger_confirmation_time_s", "stop_request_interval_s",
                    "impact_peak_g", "filter_diagnostics", "lookback_onset_time_s"):
            self.assertEqual(record[key], pure[key], key)
        self.assertEqual(capture["samples"], pure_capture["samples"])
        self.assertEqual(record["position_policy"], filters.HYBRID_POSITION_POLICY)
        self.assertEqual(record["returned_trigger_time_s"], record["position_selection"]["selected_time_s"])
        filters.validate_filtered_capture_prefix(capture, record)

    def test_cross_packet_wrap_and_remap_retain_original_position_and_capture_row(self):
        for options in ({"split": 5},
                        {"split": 5, "clock_shift": .00002}):
            capture, record = native_hybrid_capture(**options)
            filters.validate_filtered_capture_prefix(capture, record)
            detector = capture["detector"]
            self.assertEqual(detector["position_sample_matches"], 1)
            self.assertEqual(detector["position_time_s"], capture["samples"][detector["position_sample_index"]][0])
            self.assertEqual(detector["position_sample"], record["position_selection"]["selected_sample"])
            self.assertEqual(record["returned_trigger_time_s"], record["position_selection"]["selected_time_s"])

    def test_fallback_receipt_and_tampering_are_checked_against_original_counts(self):
        capture, record = native_hybrid_capture(raw_threshold=4.25)
        self.assertEqual(record["position_selection"]["fallback_reason"], "no_associated_raw_threshold_crossing")
        self.assertEqual(record["returned_trigger_time_s"], record["trigger_crossing_time_s"])
        filters.validate_filtered_capture_prefix(capture, record)
        for target, key, value in ((capture["detector"]["position_selection"], "selected_time_s", record["returned_trigger_time_s"] + .0001),
                                  (record, "returned_trigger_time_s", record["returned_trigger_time_s"] + .0001)):
            old = target[key]
            target[key] = value
            with self.assertRaises(ValueError):
                filters.validate_filtered_capture_prefix(capture, record)
            target[key] = old

    def test_immediate_completion_records_complete_selection_after_dispatch(self):
        original = plugin.BeaconAccelHoming._fire
        receipts = []
        def fire(owner, attempt, reason, code):
            original(owner, attempt, reason, code)
            if reason == "impact":
                self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)
                owner._record(attempt)
                receipts.append(copy.deepcopy(attempt.record["position_selection"]))
        with mock.patch.object(plugin.BeaconAccelHoming, "_fire", fire):
            capture, record = native_hybrid_capture()
        self.assertEqual(receipts, [record["position_selection"]])
        filters.validate_filtered_capture_prefix(capture, record)

    def test_reference_marker_validity_tracks_retained_rows_only(self):
        reference = plugin.QuietReference("x", 0., .01, 100000., require_quiet=False)
        for index in range(120):
            reference.add(index * .001, (0, 0, 128), (index, 1, 0, 2), original_valid=index != 0)
        self.assertEqual(len(reference._original_valid), reference.count)
        self.assertTrue(all(reference._original_valid))
        self.assertTrue(reference.baseline())


class HybridPreflightTests(unittest.TestCase):
    def test_compatible_saved_profiles_select_filtered_control_and_restore_on_cancel(self):
        rig = Rig()
        install_profiles(rig)
        original = dict(rig.owner.threshold_g)
        model = copy.deepcopy(rig.workflow.model.metadata)
        rig.workflow._begin(GCmd(DETECTOR="HYBRID"), "accuracy", axes=["x"])
        self.assertEqual(rig.workflow.detector_type("x"), "filtered")
        self.assertEqual(rig.owner.threshold_g["x"], 2.25)
        self.assertEqual(rig.workflow.hybrid_raw_profile("x")["threshold_g"], 4.25)
        rig.workflow._restore()
        self.assertEqual(rig.owner.threshold_g, original)
        self.assertEqual(rig.workflow.model.metadata, model)
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.noise_calls, [])

    def test_all_axes_and_both_profiles_preflight_before_motion(self):
        rig = Rig()
        install_profiles(rig)
        for axes in (["x", "y"], ["y"]):
            with self.assertRaisesRegex(CommandError, "compatible saved RAW XYZ and FILTERED"):
                rig.workflow._begin(GCmd(DETECTOR="HYBRID"), "test", axes=axes)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.noise_calls, [])

    def test_bad_context_metric_identity_and_options_reject_without_new_calibration(self):
        for change in ("raw_metric", "raw_context", "filtered_sensor", "filtered_identity", "raw_missing", "backend"):
            rig = Rig()
            install_profiles(rig)
            filtered = rig.workflow.model.metadata["x"]
            raw = filtered["detector_profiles"]["raw"]["metadata"]
            if change == "raw_metric":
                raw["impact_metric"] = "selected_axis"
            elif change == "raw_context":
                raw["homing_speed_mm_s"] += 1.
            elif change == "filtered_sensor":
                filtered["filter_sensor_context"] = {}
            elif change == "filtered_identity":
                filtered["detector_config"] = {"version": 2}
            elif change == "raw_missing":
                filtered["detector_profiles"] = {}
            else:
                rig.owner.backend = types.SimpleNamespace(kind="adxl345")
            with self.subTest(change=change), self.assertRaises(CommandError):
                rig.workflow._begin(GCmd(DETECTOR="HYBRID"), "accuracy", axes=["x"])
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.home_calls, [])
        rig = Rig()
        install_profiles(rig)
        for mode in ("calibration", "noise"):
            with self.assertRaises(CommandError):
                rig.workflow._select_detectors(GCmd(DETECTOR="HYBRID"), mode, ["x"])
        for option in ("RAW_THRESHOLD", "FILTERED_THRESHOLD", "HYBRID_THRESHOLD", "POSITION_POLICY", "LOOKBACK"):
            with self.assertRaisesRegex(CommandError, "Unsupported HYBRID"):
                rig.workflow._select_detectors(GCmd(DETECTOR="HYBRID", **{option: 1.}), "accuracy", ["x"])
        for command in (rig.workflow.cmd_FILTER_CHARTS, rig.workflow.cmd_NOISE):
            with self.assertRaises(CommandError):
                command(GCmd(DETECTOR="HYBRID", CENTERED=1))

    def test_early_quality_aborts_without_recovery_or_publication_and_preserves_z(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = HybridWorkflowRig({"charts_directory": directory})
            install_profiles(rig)
            original = copy.deepcopy(rig.workflow.model.metadata)
            thresholds = dict(rig.owner.threshold_g)
            z_position, z_limit = rig.toolhead.position[2:], rig.toolhead.kin.limits[2]
            rig.home_results.append({"travel_mm": 51.175, "halt_travel_mm_signed": -53.175})
            with self.assertRaisesRegex(CommandError, "HYBRID accuracy rejected.*manually recenter"):
                rig.accuracy(DETECTOR="HYBRID")
            self.assertEqual(len(rig.home_calls), 1)
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
            self.assertEqual(rig.toolhead.position[2:], z_position)
            self.assertEqual(rig.toolhead.kin.limits[2], z_limit)
            self.assertEqual(rig.owner.threshold_g, thresholds)
            self.assertEqual(rig.workflow.model.metadata, original)
            self.assertEqual(rig.configfile.saved, {})
            self.assertNotIn("SAVE_CONFIG", rig.scripts)
            self.assertEqual(rig.noise_calls, [])
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_complete_hybrid_workflow_exports_json_and_png_with_actual_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = HybridWorkflowRig({"charts_directory": directory})
            install_profiles(rig)
            original = copy.deepcopy(rig.workflow.model.metadata)
            rig.accuracy(DETECTOR="HYBRID")
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            paths = result["summary_paths"]
            diagnostic = json.loads(Path(result["telemetry_paths"]["x_json"]).read_text())
            self.assertEqual(diagnostic["hybrid_position_summary"]["raw_selected_count"], 2)
            self.assertEqual(diagnostic["hybrid_position_summary"]["fallback_count"], 0)
            report = charts._accuracy_report_from_png(Path(paths["x_png"]).read_bytes())
            self.assertEqual(report["version"], 5)
            self.assertEqual(report["hybrid_position_summary"], diagnostic["hybrid_position_summary"])
            self.assertEqual(rig.workflow.model.metadata, original)
            self.assertEqual(rig.configfile.saved, {})

    def test_cancel_during_hybrid_publication_restores_and_removes_successful_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = HybridWorkflowRig({"charts_directory": directory})
            install_profiles(rig)
            thresholds = dict(rig.owner.threshold_g)
            rig.workflow.renderer.on_render = lambda _payload: rig.workflow.cmd_CANCEL(GCmd())
            with self.assertRaises(CommandError):
                rig.accuracy(DETECTOR="HYBRID")
            self.assertEqual(rig.owner.threshold_g, thresholds)
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(list(Path(directory).iterdir()), [])
            self.assertIsNone(rig.workflow.session)


class HybridReceiptTests(unittest.TestCase):
    def report(self):
        capture, record = native_hybrid_capture()
        record.update(capture_valid=True, sensor_context={"backend": "beacon_raw"},
                      geometry_context={"minimum": 0., "maximum": 350.},
                      configured_homing_speed_mm_s=100., travel_mm=175., travel_mm_exact=175.)
        other = copy.deepcopy(record)
        other.update(travel_mm=175.01, travel_mm_exact=175.01)
        trials = [record, other]
        item = {"requested_samples": 2, "discovery": copy.deepcopy(record), "trials": trials,
                "statistics": charts._accuracy_statistics(trials)}
        return capture, record, item, charts._accuracy_report("synthetic_hybrid", "x", item, True)

    def test_v5_roundtrip_and_counts_sources_reference_receipts_are_bound(self):
        _capture, _record, _item, report = self.report()
        self.assertEqual(report["version"], 5)
        self.assertEqual(report["hybrid_position_summary"]["raw_selected_count"], 2)
        encoded = charts._png_with_accuracy_report(png_bytes(), report)
        self.assertEqual(charts._accuracy_report_from_png(encoded), report)
        for changed in ("counts", "source", "profile", "counts_raw", "coordinated_profile", "raw_level"):
            other = copy.deepcopy(report)
            if changed == "counts":
                other["hybrid_position_summary"]["fallback_count"] = 1
            elif changed == "source":
                other["position_selections"][0]["actual_source"] = "filtered_threshold_crossing_fallback"
            elif changed == "profile":
                other["context"]["raw_position_profile"]["threshold_g"] = 4.25
            elif changed == "counts_raw":
                other["position_selections"][0]["selected_raw_sensor_counts"] = [0, 0, 128]
            elif changed == "coordinated_profile":
                other["context"]["raw_position_profile"]["threshold_g"] = 4.25
                other["discovery_position_selection"]["raw_profile"]["threshold_g"] = 4.25
                for selection in other["position_selections"]:
                    selection["raw_profile"]["threshold_g"] = 4.25
            else:
                other["position_selections"][0]["selected_raw_level_g"] += 1.
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                charts._validate_accuracy_report(other)

    def test_maximum_fifty_home_receipt_is_bounded_and_old_versions_remain_readable(self):
        _capture, _record, _item, report = self.report()
        report.update(count=50, requested=50, travels_mm=[175.] * 50,
                      position_selections=[copy.deepcopy(report["position_selections"][0]) for _ in range(50)])
        report["statistics"] = charts._accuracy_statistics([{"travel_mm": value} for value in report["travels_mm"]])
        report["hybrid_position_summary"] = charts._hybrid_position_summary(report["position_selections"])
        encoded = charts._png_with_accuracy_report(png_bytes(), report)
        self.assertEqual(charts._accuracy_report_from_png(encoded), report)
        self.assertLess(len(charts._json(report)), charts._MAX_HYBRID_REPORT_BYTES)

    def test_hybrid_telemetry_keeps_control_events_and_separate_original_selection(self):
        capture, record, _item, _report = self.report()
        encoded = filters.prepare_accuracy_home_telemetry(capture, record)
        home = filters.validate_accuracy_home_telemetry(encoded, record)
        self.assertEqual(home["position_policy"], filters.HYBRID_POSITION_POLICY)
        self.assertEqual(home["requested_detector"], "hybrid")
        self.assertEqual(home["events"]["onset"]["original_time_s"], record["trigger_crossing_time_s"])
        self.assertEqual(home["position_selection"], json.loads(json.dumps(record["position_selection"])))
        self.assertLessEqual(len(home["window"]["rows"]), 50)


if __name__ == "__main__":
    unittest.main()
