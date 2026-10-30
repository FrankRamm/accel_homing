"""Position attribution changes only; every fixture is synthetic, no hardware."""
import contextlib
import copy
import json
import unittest
from unittest import mock

from harness import counts, plugin, CommandError
import accel_homing_charts as charts
import accel_homing_filters as filters
from accel_homing_model import AccelHomingModel
from test_chronological_clipping import native_filtered_capture
from test_censored_response_charts import synthetic_public_censored_report
import test_filter_engine as filter_tests
from test_accuracy_charts import png_bytes
from test_workflows import Config, GCmd, Rig


CURRENT = filters.FILTERED_POSITION_POLICY
LEGACY = filters.LEGACY_FILTERED_POSITION_POLICY
ONSET_FIELDS = {"onset_time_s", "onset_identity", "onset_level_g"}


@contextlib.contextmanager
def runtime_policy(policy):
    def factory(*args, **kwargs):
        return filters.CausalImpactDetector(*args, **dict(kwargs, position_policy=policy))
    with mock.patch.object(plugin, "CausalImpactDetector", factory):
        yield


def native_case(policy, detector="filtered", version=2, clip_position=None):
    snapshots = []
    original_fire = plugin.BeaconAccelHoming._fire
    def fire(owner, attempt, reason, code):
        original_fire(owner, attempt, reason, code)
        snapshots.append({"reason": reason, "code": code, "impact_until": attempt.impact_until,
                          "messages": copy.deepcopy(attempt.endstop.trigger_log.messages),
                          "confirmation_time_s": attempt.confirmation_time,
                          "confirmation_sample": attempt.confirmation_sample,
                          "diagnostics": None if attempt.filter_engine is None else attempt.filter_engine.diagnostics()})
    # A smooth pre-threshold rise gives the old lookback an earlier sample.
    values = [.05] * 16 + [.4, .6, .8, 1., 1.4, 2., 3.] + [3.] * 41
    with runtime_policy(policy), mock.patch.object(plugin.BeaconAccelHoming, "_fire", fire):
        capture, record = native_filtered_capture(mode="accuracy", detector_type=detector,
            finish_home_wait=True, filter_config={"version": version}, clock_hz=32000000.,
            sample_period=.000302, packet_size=len(values), split=20, clip_position=clip_position,
            row_values=[(counts(value), 0, counts(1.)) for value in values])
    return capture, record, snapshots


class CrossingEngineTests(unittest.TestCase):
    def paired(self, times, values, version=2, config=None):
        engines = [filters.CausalImpactDetector(1., (0., 0., 1.),
                    config=dict(config or {}, version=version), position_policy=policy)
                   for policy in (LEGACY, CURRENT)]
        for engine in engines:
            engine.previous_time = times[0] - (times[1] - times[0])
        for index, (when, value) in enumerate(zip(times, values)):
            results = [engine.feed(when, (value, 0., 1.), index) for engine in engines]
            self.assertEqual(results[0] is None, results[1] is None)
            self.assertEqual(engines[0].diagnostics(), engines[1].diagnostics())
            self.assertEqual(list(engines[0].history), list(engines[1].history))
            self.assertEqual(engines[0].filtered_xyz, engines[1].filtered_xyz)
            self.assertEqual(engines[0].moving, engines[1].moving)
            for field in ("candidate", "decision"):
                left, right = (getattr(engine, field) for engine in engines)
                self.assertEqual(None if left is None else {k: v for k, v in left.items() if k not in ONSET_FIELDS},
                                 None if right is None else {k: v for k, v in right.items() if k not in ONSET_FIELDS})
        return engines

    def test_same_confirmation_samples_counters_and_energy_at_multiple_rates(self):
        values = [0.] * 12 + [.3, .5, .7, .9, 1.1, 1.5, 2., 3.] + [3.] * 30
        earlier_lookbacks = 0
        for version in (1, 2):
            for rate in (800., 3200., 12500., 25600.):
                with self.subTest(version=version, rate=rate):
                    old, new = self.paired([index / rate for index in range(len(values))], values, version)
                    event = new.decision
                    self.assertIsNotNone(event)
                    self.assertEqual(event["onset_identity"], event["crossing_identity"])
                    self.assertEqual(event["onset_time_s"], event["crossing_time_s"])
                    self.assertGreaterEqual(event["onset_level_g"], 1.)
                    self.assertEqual(event["lookback_onset_time_s"], old.decision["onset_time_s"])
                    earlier_lookbacks += old.decision["onset_time_s"] < event["onset_time_s"]
        self.assertGreater(earlier_lookbacks, 0)

    def test_rejected_first_candidate_does_not_supply_accepted_onset(self):
        values = [0.] * 8 + [3.] + [0.] * 12 + [.4, .7, .9, 1.1, 2., 3.] + [3.] * 20
        old, new = self.paired([index / 12500. for index in range(len(values))], values,
                               config={"lowpass_tau_s": 0.})
        self.assertEqual(new.candidates, 2)
        self.assertEqual(new.rejected_candidates, 1)
        self.assertEqual(new.rejection_reasons, {"release": 1, "timeout": 0})
        self.assertGreater(new.decision["onset_identity"], 20)
        self.assertEqual(new.decision["onset_identity"], new.decision["crossing_identity"])
        self.assertLess(old.decision["onset_identity"], new.decision["onset_identity"])
        self.assertEqual(new.decision["onset_level_g"], values[new.decision["crossing_identity"]])

    def test_timeout_and_new_candidate_confirm_on_identical_exact_row(self):
        old, new = self.paired([0., .001, .002, .003, .0035, .004, .0045, .005], [2.] * 8,
            config={"lowpass_tau_s": 0., "confirmation_min_s": .0015, "confirmation_max_s": .0015})
        self.assertEqual(new.candidates, 2)
        self.assertEqual(new.rejection_reasons, {"release": 0, "timeout": 1})
        self.assertEqual(new.decision["crossing_identity"], 3)
        self.assertEqual(new.decision["confirmation_identity"], 6)
        self.assertEqual(new.decision["confirmation_time_s"], .0045)
        self.assertEqual(old.decision["confirmation_identity"], 6)

    def test_unknown_position_policy_fails_without_changing_numerical_config(self):
        self.assertEqual(filters.detector_config()["version"], 2)
        self.assertNotIn("position_policy", filters.detector_config())
        self.assertEqual(filters.filtered_position_policy({}), LEGACY)
        self.assertEqual(filters.CausalImpactDetector(1., (0., 0., 1.)).position_policy, CURRENT)
        for invalid in (None, "crossing", 3, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                filters.CausalImpactDetector(1., (0., 0., 1.), position_policy=invalid)


class CrossingRuntimeTests(unittest.TestCase):
    def test_native_home_wait_crossing_keeps_dispatch_response_peak_and_capture(self):
        for version in (1, 2):
            with self.subTest(version=version):
                before, old, old_stop = native_case(LEGACY, version=version)
                after, new, new_stop = native_case(CURRENT, version=version)
                self.assertEqual(old_stop, new_stop)
                self.assertEqual(len(new_stop), 1)
                self.assertEqual(len(new_stop[0]["messages"]), 1)
                self.assertEqual(new_stop[0]["reason"], "impact")
                self.assertEqual(new_stop[0]["code"], plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT)
                for key in ("confirmation_sample", "crossing_sample", "trigger_confirmation_time_s",
                            "trigger_crossing_time_s", "stop_request_interval_s", "impact_peak_g",
                            "observed_valid_prefix_peak_g", "filter_diagnostics", "response_censored"):
                    self.assertEqual(old[key], new[key], key)
                self.assertEqual(before["samples"], after["samples"])
                self.assertEqual(before["raw_integrity"], after["raw_integrity"])
                self.assertEqual(before["native_clip_decode"], after["native_clip_decode"])
                self.assertEqual(old["returned_trigger_time_s"], old["trigger_onset_time_s"])
                self.assertLess(old["returned_trigger_time_s"], new["returned_trigger_time_s"])
                self.assertEqual(new["returned_trigger_time_s"], new["trigger_crossing_time_s"])
                self.assertEqual(new["onset_sample"], new["crossing_sample"])
                self.assertEqual(new["onset_g"], round(after["detector"]["onset_g"], 2))
                self.assertGreaterEqual(new["onset_g"], new["threshold_g"])
                self.assertEqual(new["lookback_onset_time_s"], old["trigger_onset_time_s"])
                self.assertEqual(new["position_policy"], CURRENT)
                self.assertEqual(after["detector"]["position_policy"], CURRENT)
                replay = filters.compare_filter_capture(after, 1., 1.)
                self.assertEqual(replay["filtered_decision"]["onset_time_s"], new["crossing_time_s"])

    def test_raw_runtime_outputs_and_stop_are_identical_for_both_defaults(self):
        old_capture, old_record, old_stop = native_case(LEGACY, detector="raw")
        new_capture, new_record, new_stop = native_case(CURRENT, detector="raw")
        self.assertEqual(old_record, new_record)
        self.assertEqual(old_capture, new_capture)
        self.assertEqual(old_stop, new_stop)
        self.assertNotIn("position_policy", new_record)

    def test_old_capture_without_policy_replays_lookback_without_relabelling(self):
        capture, record, _ = native_case(LEGACY)
        del capture["detector"]["position_policy"], record["position_policy"]
        original = copy.deepcopy(record)
        self.assertIsNone(filters.validate_filtered_capture_prefix(capture, record))
        replay = filters.compare_filter_capture(capture, 1., 1.)
        self.assertEqual(replay["position_policy"], LEGACY)
        self.assertEqual(replay["filtered_decision"]["onset_time_s"], record["onset_time_s"])
        self.assertLess(replay["filtered_decision"]["onset_time_s"], replay["filtered_decision"]["crossing_time_s"])
        self.assertEqual(record, original)
        self.assertNotIn("position_policy", capture["detector"])

    def test_new_and_legacy_home_telemetry_reconstruct_the_recorded_policy(self):
        for policy in (LEGACY, CURRENT):
            capture, record, _ = native_case(policy)
            encoded = filters.prepare_accuracy_home_telemetry(capture, record)
            home = filters.validate_accuracy_home_telemetry(encoded, record)
            self.assertEqual(home["position_policy"], policy)
            self.assertTrue(home["state_provenance"]["actual_decision_verified"])
            self.assertTrue(home["state_provenance"]["replay_candidate_summary_matches_actual"])
            self.assertEqual(home["events"]["onset"]["sample"], list(record["onset_sample"]))
            self.assertEqual(home["lookback_onset_time_s"], record["lookback_onset_time_s"])

    def test_changed_capture_position_policy_cannot_prove_wrong_onset(self):
        capture, record, _ = native_case(CURRENT)
        capture["detector"]["position_policy"] = LEGACY
        with self.assertRaises(ValueError):
            filters.validate_filtered_capture_prefix(capture, record)
        with self.assertRaises(ValueError):
            filters.prepare_accuracy_home_telemetry(capture, record)


class CrossingCompatibilityTests(unittest.TestCase):
    def test_saved_raw_and_filtered_thresholds_remain_usable_without_recalibration(self):
        for version in (1, 2):
            rig = Rig()
            metadata = filter_tests.ProfileTests().metadata(rig, "filtered", version=version)
            metadata["detector_profiles"] = {"raw": {"threshold_g": 4.25,
                "metadata": filter_tests.ProfileTests().metadata(rig, "raw")}}
            original = copy.deepcopy(metadata)
            model = AccelHomingModel(Config(None, {"version": 2, "x_threshold": 2.25,
                "x_successes": 10, "x_metadata": json.dumps(metadata)}, "accel_homing_model default"))
            rig.workflow.model = model
            self.assertEqual(rig.workflow._profile("x", "raw")["threshold_g"], 4.25)
            self.assertEqual(rig.workflow._profile("x", "filtered")["threshold_g"], 2.25)
            if version == 1:
                with self.assertRaisesRegex(CommandError, "rectified v2"):
                    rig.workflow._begin(GCmd(DETECTOR="FILTERED"), "test", axes=["x"])
                self.assertEqual(model.metadata["x"], original)
                continue
            rig.workflow._begin(GCmd(DETECTOR="FILTERED"), "test", axes=["x"])
            self.assertEqual(rig.owner.threshold_g["x"], 2.25)
            self.assertEqual(rig.workflow.filter_config("x"), filters.detector_config({"version": version}))
            engine = filters.CausalImpactDetector(2.25, (0., 0., 1.), config=rig.workflow.filter_config("x"))
            self.assertEqual(engine.position_policy, CURRENT)
            self.assertEqual(model.metadata["x"], original)
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.noise_calls, [])

    def test_unknown_saved_position_policy_is_rejected(self):
        rig = Rig()
        metadata = dict(filter_tests.ProfileTests().metadata(rig, "filtered"), position_policy="unknown")
        with self.assertRaises(ValueError):
            filters.validate_detector_identity(2, metadata["detector_config"], metadata)

    def test_schema_four_comparison_labels_policy_treatment_and_preserves_old_png_receipt(self):
        baseline = synthetic_public_censored_report(filter_version=2)
        baseline["context"].update(geometry_context={"position_min": 0., "position_max": 350.},
                                   homing_speed_mm_s=100., configured_homing_speed_mm_s=100.,
                                   homing_accel_mm_s2=5000.)
        original = copy.deepcopy(baseline)
        current = copy.deepcopy(baseline)
        current.update(version=4, run_id="synthetic_crossing_current")
        current["context"]["position_policy"] = CURRENT
        restored = charts._accuracy_report_from_png(charts._png_with_accuracy_report(
            png_bytes(), baseline))
        self.assertEqual(restored, original)
        comparison = charts._compare_accuracy_reports(current, restored)
        self.assertTrue(comparison["comparable_context"])
        self.assertEqual(comparison["treatment_differences"], ["position_policy"])
        self.assertEqual(charts._position_label(restored["context"]), "Estimated filtered lookback onset")
        self.assertEqual(charts._position_label(current["context"]), "Accepted filtered threshold crossing")
        self.assertEqual(baseline, original)

    def test_raw_schema_and_legacy_filtered_shadows_stay_explicitly_unchanged(self):
        baseline = synthetic_public_censored_report(filter_version=2)
        baseline.update(version=1, censored_response_count=0, censored_responses=[])
        baseline["context"].update(detector_type="raw", detector_version=None, detector_config=None)
        current = copy.deepcopy(baseline)
        current["run_id"] = "synthetic_raw_current"
        self.assertEqual(charts._compare_accuracy_reports(current, baseline)["treatment_differences"], [])
        self.assertNotIn("position_policy", charts._accuracy_context(dict(baseline["context"])))
        analysis = filters.compare_filter_capture(filter_tests.comparison_capture(), 1., 1.)
        self.assertIsNone(analysis["actual_position_policy"])
        for item in analysis["variants"].values():
            self.assertEqual(item["position_policy"], LEGACY)
            self.assertEqual(charts._position_label(dict(item, detector_type="filtered")),
                             "Estimated filtered lookback onset")

    def test_schema_four_requires_policy_and_rejects_policy_changes_within_run(self):
        report = synthetic_public_censored_report(filter_version=2)
        report["version"] = 4
        with self.assertRaises(ValueError):
            charts._validate_accuracy_report(report)
        report["context"]["position_policy"] = "unknown"
        with self.assertRaises(ValueError):
            charts._validate_accuracy_report(report)
        report["context"]["position_policy"] = CURRENT
        charts._validate_accuracy_report(report)
        item = {"trials": [dict(report["context"], threshold_g=1.),
                           dict(report["context"], threshold_g=1., position_policy=LEGACY)]}
        with self.assertRaisesRegex(ValueError, "context changed"):
            charts._accuracy_report("synthetic_mixed_policy", "x", item, True)


if __name__ == "__main__":
    unittest.main()
