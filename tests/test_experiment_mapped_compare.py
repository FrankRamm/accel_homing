"""Mapped delivery comparison through the real workflow/backend/capture checks."""
import copy
import csv
import gc
import gzip
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
import zipfile

from harness import CommandError, acquire_reference
from test_backends import generic
from test_experiment_compare import CompareRig
from test_mapped_detector import deliver, mapping
from test_session import FakeRail, info, with_endstops
from test_workflows import GCmd, workflow_module
import accel_homing_experiment as experiment_module
from accel_homing_axes import mapped_axis_capture
import accel_homing_filter as signal_filter
import accel_homing_troubleshoot as diagnostics


class MappedCompareRig(CompareRig):
    def __init__(self, directory, filtered=False):
        super().__init__({"charts_directory": directory})
        self.mapping = mapping("x", source=2, sign=-1)
        self.filter = signal_filter.profile(100. if filtered else 0., 3.)
        self.owner.filter_settings = lambda axis: dict(self.filter)
        metadata = dict(self.workflow._model_context("x"), impact_metric="mapped_axis",
                        axis_mapping=copy.deepcopy(self.mapping),
                        axis_mapping_context=self.owner.noise_sensor_context(),
                        signal_filter=dict(self.filter), filter_sample_rate_hz=2000. if filtered else None)
        self.workflow.model = types.SimpleNamespace(thresholds={"x": 8.}, metadata={"x": metadata})
        self.capture_change = None
        self.observed_mappings = []

    def stock_home(self, gcmd):
        super().stock_home(gcmd)
        # Record the actual workflow detector selector while its native-home
        # scope is active. This must not take the historical experiment bypass.
        self.observed_mappings.append(self.workflow.detection_axis_mapping("x"))

    def stop_capture(self):
        if len(self.home_calls) == self.capture_home_before:
            return super().stop_capture()
        self.capture_active = False
        self.capture_stops += 1
        arm = self.sensor_time
        # Non-mapped axes exceed the threshold well before the selected channel.
        rows = [(arm - .0005, 0., 0., 0.)] + [
            (arm + number / 2000., 12., -12., .1 if number < 40 else -12.)
            for number in range(80)]
        first = 1
        metadata = signal_filter.CausalNotch(self.filter, 2000.).metadata() if self.filter["type"] != "disabled" else None
        filtered = signal_filter.replay(rows, [0., 0., 0.], metadata, first) if metadata else None
        values = filtered if filtered else rows[first:]
        index = first + next(index for index, row in enumerate(values) if abs(row[3]) >= 8.)
        onset, level = rows[index][0], abs(values[index - first][3])
        identity = (int(arm * 1000), len(rows) - 1, index, len(rows))
        record = self.owner.last["x"]
        record.update(impact_metric="mapped_axis", axis_mapping=copy.deepcopy(self.mapping),
                      arm_time_s=arm, onset_time_s=onset, onset_sample=identity,
                      onset_g=level, impact_peak_g=12., signal_filter=metadata or dict(self.filter))
        detector = {"axis": "x", "attempt_count": 1, "impact_metric": "mapped_axis",
                    "axis_mapping": copy.deepcopy(self.mapping), "arm_time_s": arm,
                    "onset_time_s": onset, "trigger_onset_time_s": onset,
                    "onset_sample": identity, "onset_sample_version": 1,
                    "onset_sample_matches": 1, "onset_sample_index": index,
                    "threshold_g": 8., "onset_g": level, "impact_peak_g": 12., "direction": -1,
                    "homing_speed_mm_s": self.owner.rails_by_axis["x"].info.speed,
                    "homing_accel_mm_s2": self.owner.homing_accel,
                    "signal_filter": metadata or dict(self.filter),
                    "experimental_envelope": {"max_oldest_eligible_age_s": .010,
                        "max_newest_eligible_age_s": .005, "eligible_batches": 20, "future_sample_seen": False,
                        "sample_age_limit_enforced": True, "max_sample_age_s": .040,
                        "detector_metric": "mapped_axis", "age_observation": "current_host_dispatch"}}
        capture = {"samples": rows, "dropped": 0, "baseline_g": [0., 0., 0.],
                   "coordinate_frame": "configured_sensor_stream", "axis_mapping": copy.deepcopy(self.mapping),
                   "_record": record, "detector": detector}
        if metadata:
            capture.update(signal_filter=dict(metadata, first_sample_index=first, first_sample_matches=1),
                           filtered_samples=filtered)
        if self.capture_change:
            self.capture_change(capture)
        self.owner.troubleshoot.record_event("arm", axis="x", arm_print_time_s=arm)
        self.owner.troubleshoot.record_event("stop_request", axis="x", arm_print_time_s=arm,
            onset_print_time_s=onset, onset_sample=identity, request_estimated_print_time_s=onset + .002,
            request_before_host_monotonic_s=self.reactor.monotonic())
        return capture


class MappedComparisonTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(workflow_module, "TroubleshootRecorder", diagnostics.Recorder)
        patch.start()
        self.addCleanup(patch.stop)

    def run_compare(self, rig, **parameters):
        with mock.patch.object(workflow_module, "_load_experiment_module", return_value=experiment_module), \
                mock.patch.object(diagnostics, "render_bundle"), \
                mock.patch.object(experiment_module, "fit_experimental_profile", side_effect=AssertionError("XYZ fit used")), \
                mock.patch.object(experiment_module, "replay_shadow_capture", side_effect=AssertionError("XYZ replay used")):
            rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, MODE="COMPARE", SAMPLES=2, **parameters))

    def report(self, rig):
        return json.loads(Path(rig.workflow.last_result["artifacts"]["manifest"]).read_text(encoding="utf-8"))

    def assert_restored(self, rig, previous=None):
        self.assertIsNone(rig.workflow.session)
        self.assertIsNone(rig.owner.troubleshoot)
        self.assertIs(rig.owner.experimental_profile, previous)
        self.assertFalse(rig.capture_active)
        self.assertEqual(rig.owner.backend.interval, .005)
        self.assertEqual(rig.helper.batch_interval, .100)
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertEqual(rig.configfile.saved, {})

    def test_cartographer_detector_uses_real_workflow_mapping_and_frozen_filter(self):
        for filtered, threshold in ((False, 1.), (True, 1.), (False, 2.3), (True, 2.3),
                                    (False, .6234), (True, .6234), (False, 1.237), (True, 1.237)):
            with self.subTest(filtered=filtered, threshold=threshold):
                obj = generic("lis2dw cartographer", options={"threshold": threshold,
                    "y_notch_frequency": 100. if filtered else 0.})
                endstop = with_endstops(obj, ("y",))["y"]
                obj.session_axes = {"y": FakeRail(endstop, info(retract=0.))}
                obj.reactor.now = 10.
                acquire_reference(obj, endstop)
                selected = mapping("y", source=2, sign=-1)
                obj.workflows.model = types.SimpleNamespace(thresholds={"y": threshold}, metadata={"y": {
                    "impact_metric": "mapped_axis", "axis_mapping": selected,
                    "axis_mapping_context": obj.noise_sensor_context(),
                    "signal_filter": obj.filter_settings("y"), "filter_sample_rate_hz": 3200.}})
                obj.workflows.session = {"mode": "troubleshoot", "axis": "y", "experiment": {
                    "mode": "COMPARE", "impact_metric": "mapped_axis", "max_sample_age_s": .020}}
                obj.workflows.in_home, obj.workflows.home_axis = True, "y"
                when = obj.backend.last_sample_time + 1. / obj.backend.rate
                attempt = obj.prepare(endstop, when)
                obj.start_capture()
                obj.arm(attempt)
                self.assertEqual(attempt.axis_mapping, selected)
                self.assertEqual(attempt.index, 2)
                self.assertAlmostEqual(attempt.limit_raw * attempt.count_g, threshold)
                self.assertIsNone(attempt.experimental)
                self.assertEqual(attempt.mapped_compare_max_age_s, .020)
                self.assertEqual(attempt.notch is not None, filtered)
                deliver(obj, attempt, [(5., 6., .1)] * 8)
                self.assertFalse(attempt.done)
                deliver(obj, attempt, [(5., 6., -3.)] * 8, offset=8 / 3200.)
                self.assertEqual(attempt.reason, "impact")
                obj.disarm()
                capture = obj.stop_capture()
                self.assertEqual(capture["detector"]["impact_metric"], "mapped_axis")
                self.assertEqual(capture["detector"]["axis_mapping"], selected)
                self.assertTrue(capture["detector"]["experimental_envelope"]["sample_age_limit_enforced"])
                self.assertAlmostEqual(capture["detector"]["threshold_g"], threshold)
                if threshold == 2.3:
                    self.assertNotEqual(capture["detector"]["threshold_g"], threshold)
                frozen = {"axis": "y", "axis_mapping": selected, "threshold_g": threshold,
                          "signal_filter": obj.filter_settings("y")}
                # The homing record reports 0.01 g; the detector keeps the exact frozen value.
                self.assertEqual(attempt.record["threshold_g"], round(threshold, 2))
                receipt = experiment_module.validate_mapped_comparison_capture(capture, attempt.record,
                    frozen, max_age_s=.020)
                self.assertTrue(receipt["verified"])
                for record_threshold, detector_threshold in ((round(threshold, 2) + .01, None),
                                                             (None, threshold + .001)):
                    changed_record, changed_capture = dict(attempt.record), copy.deepcopy(capture)
                    if record_threshold is not None:
                        changed_record["threshold_g"] = record_threshold
                    else:
                        changed_capture["detector"]["threshold_g"] = detector_threshold
                    with self.assertRaisesRegex(ValueError, "frozen calibrated mapped threshold"):
                        experiment_module.validate_mapped_comparison_capture(
                            changed_capture, changed_record, frozen, max_age_s=.020)
                obj._end_session()

    def test_compact_capture_reproduces_mapped_crossing_and_exports_only_its_signal(self):
        for filtered in (False, True):
            with self.subTest(filtered=filtered), tempfile.TemporaryDirectory() as directory:
                rig = MappedCompareRig(directory, filtered)
                rig.workflow.gentle_homing = True
                rig.owner.rails_by_axis["x"].info.speed = 40.
                rig.owner.homing_accel = 1000.
                rig.workflow.model.metadata["x"].update(rig.workflow._model_context("x"),
                    gentle_homing=True, saved_threshold_tested=True, saved_threshold_g=8., tested_threshold_g=8.)
                rig.capture_change = lambda capture: capture.update(mapped_axis_capture(capture, rig.mapping))
                self.run_compare(rig)
                report = self.report(rig)
                home = report["homes"][0]
                self.assertTrue(home["mapped_comparison_receipt"]["verified"])
                self.assertEqual(home["capture"]["invalid_rows"], 0)
                folder = Path(rig.workflow.last_result["artifacts"]["folder"])
                with gzip.open(folder / home["raw_file"], "rt") as stream:
                    reader = csv.DictReader(stream)
                    self.assertEqual(reader.fieldnames, ["sample_index", "print_time_s", "mapped_axis_g"]
                                     + (["filtered_mapped_axis_g"] if filtered else []))
                    rows = list(reader)
                    self.assertEqual(len(rows), home["capture"]["raw_rows_retained"])
                    self.assertTrue(all(row["mapped_axis_g"] != "" for row in rows))
                    if filtered:
                        self.assertTrue(any(row["filtered_mapped_axis_g"] != "" for row in rows))

    def test_four_homes_keep_actual_mapping_threshold_and_fixed_notch_and_export_zip(self):
        for filtered in (False, True):
            with self.subTest(filtered=filtered), tempfile.TemporaryDirectory() as directory:
                rig = MappedCompareRig(directory, filtered)
                previous = {"previous_live_profile": True}
                rig.owner.experimental_profile = previous
                guard = (gc.isenabled(), rig.reactor._check_gc)
                self.run_compare(rig)
                report = self.report(rig)
                self.assertEqual(len(rig.home_calls), 4)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [8.] * 4)
                self.assertEqual(rig.observed_mappings, [rig.mapping] * 4)
                self.assertEqual([start[0] for start in rig.starts], [.005, .002, .002, .005])
                self.assertEqual([start[3] for start in rig.starts], [None] * 4)
                self.assertEqual([h["delivery_variant"] for h in report["homes"]], ["control", "fast", "fast", "control"])
                self.assertTrue(all(h["mapped_comparison_receipt"]["verified"] for h in report["homes"]))
                self.assertTrue(all(h["record"]["impact_metric"] == "mapped_axis" for h in report["homes"]))
                self.assertEqual(report["experiment"]["baseline_samples"], 0)
                self.assertIsNone(report["experiment"]["fit"])
                self.assertFalse(report["experiment"]["candidate_activated"])
                self.assertEqual(report["ending"]["completed_samples"], {"x": 4})
                self.assertEqual(report["experiment"]["statistics"]["delivery_control"]["count"], 2)
                self.assertTrue(all(report["experiment"]["runtime_restored"].values()))
                comparison = diagnostics.comparison_report(report)
                self.assertEqual(comparison["impact_metric"], "mapped_axis")
                self.assertEqual(comparison["bounds"]["reference_homes_excluded_from_pairs"], 0)
                self.assertTrue(all(pair["matched"] for pair in comparison["pairs"]))
                with zipfile.ZipFile(rig.workflow.last_result["artifacts"]["archive"]) as archive:
                    self.assertIsNone(archive.testzip())
                    self.assertEqual(sum(path.endswith(".csv.gz") for path in archive.namelist()), 4)
                    for name in ("comparison.json", "delivery.json", "experiment.json", "tester-summary.txt"):
                        self.assertTrue(any(path.endswith(name) for path in archive.namelist()))
                self.assertEqual(guard, (gc.isenabled(), rig.reactor._check_gc))
                self.assert_restored(rig, previous)

    def test_unqualified_mapped_detector_rejects_before_capture_or_motion(self):
        mutations = {
            "pending": lambda r: r.workflow.pending_model_metadata.update(x=dict(r.workflow.model.metadata["x"], run_id="new")),
            "mapping invalidated": lambda r: r.workflow.model.metadata["x"].update(axis_mapping_invalidated={"reason": "changed"}),
            "manual": lambda r: r.workflow.manual_axes.add("x"),
            "temporary": lambda r: r.workflow.note_manual_threshold("x", 8.),
            "threshold": lambda r: r.owner.threshold_g.update(x=7.),
            "filter": lambda r: r.filter.update(signal_filter.profile(120., 2.)),
            "context": lambda r: r.workflow.model.metadata["x"].update(homing_speed_mm_s=50.),
            "sensor": lambda r: r.workflow.model.metadata["x"].update(axis_mapping_context={}),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                rig = MappedCompareRig(directory)
                mutate(rig)
                with self.assertRaises(CommandError):
                    self.run_compare(rig)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.starts, [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertIsNone(rig.workflow.session)

    def test_wrong_metric_threshold_mapping_or_age_cannot_pass_or_return(self):
        mutations = {
            "XYZ metric": lambda c: c["detector"].update(impact_metric="xyz_magnitude"),
            "threshold": lambda c: c["detector"].update(threshold_g=7.),
            "mapping": lambda c: c["detector"]["axis_mapping"].update(source_axis=0),
            "stale": lambda c: c["detector"]["experimental_envelope"].update(max_oldest_eligible_age_s=.2),
            "missing ages": lambda c: c["detector"].pop("experimental_envelope"),
            "future": lambda c: c["detector"]["experimental_envelope"].update(future_sample_seen=True),
            "age guard disabled": lambda c: c["detector"]["experimental_envelope"].update(sample_age_limit_enforced=False),
            "wrong crossing": lambda c: c["samples"].__setitem__(2, (c["samples"][2][0], 12., -12., 10.)),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                rig = MappedCompareRig(directory)
                rig.capture_change = mutate
                with self.assertRaises(CommandError):
                    self.run_compare(rig)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(self.report(rig)["ending"]["result"], "aborted")
                self.assertFalse(self.report(rig)["homes"][0].get("return_completed", False))
                self.assert_restored(rig)

    def test_cancel_and_fast_registration_failure_restore_before_export(self):
        for cause in ("cancel", "start"):
            with self.subTest(cause=cause), tempfile.TemporaryDirectory() as directory:
                rig = MappedCompareRig(directory)
                if cause == "cancel":
                    rig.on_home = lambda axis: rig.workflow.cmd_CANCEL(GCmd()) if len(rig.home_calls) == 2 else None
                else:
                    add_client = rig.helper.add_client
                    def fail_fast(client):
                        if rig.owner.backend.interval == .002:
                            raise RuntimeError("fast registration failed")
                        add_client(client)
                    rig.helper.add_client = fail_fast
                with self.assertRaises(Exception):
                    self.run_compare(rig)
                self.assertEqual(len(rig.toolhead.moves), 1)
                self.assertFalse(self.report(rig)["homes"][-1].get("return_completed", False))
                self.assert_restored(rig)

    def test_changed_frozen_filter_or_driver_cleanup_is_not_reported_as_success(self):
        for cause in ("filter", "helper", "stop"):
            with self.subTest(cause=cause), tempfile.TemporaryDirectory() as directory:
                rig = MappedCompareRig(directory)
                if cause == "stop":
                    rig.backend_stop_error = RuntimeError("stop failed")
                elif cause == "filter":
                    rig.on_home = lambda axis: rig.filter.update(signal_filter.profile(120., 3.))
                else:
                    rig.on_home = lambda axis: setattr(rig.helper, "batch_interval", .003)
                with self.assertRaises(Exception):
                    self.run_compare(rig)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(self.report(rig)["ending"]["result"], "aborted")
                if cause in ("helper", "stop"):
                    self.assertFalse(self.report(rig)["experiment"]["runtime_restored"]["interval"])
                if cause == "helper":
                    self.assertEqual(rig.helper.batch_interval, .003)


if __name__ == "__main__":
    unittest.main()
