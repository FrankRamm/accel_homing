"""Offline experiment lifecycle and evidence; never connects to a printer."""
import gc
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
import zipfile

from harness import CommandError
from test_accuracy_workflow import AccuracyRig
from test_workflows import GCmd, workflow_module
import accel_homing_troubleshoot as diagnostics
import accel_homing_experiment as model


class RealFitRig(AccuracyRig):
    """Software homes and clocks with full raw captures and the real fitter."""
    def __init__(self, options=None):
        super().__init__(options)
        self.reactor._check_gc = True
        self.reactor._check_timers = lambda *args: None
        self.reactor.get_gc_stats = lambda: (0., 0., 0.)
        self.owner.experimental_profile = None

    def stop_capture(self):
        self.capture_active = False
        self.capture_stops += 1
        axis = self.home_calls[-1]["axis"]
        record = self.owner.last[axis]
        arm = self.sensor_time
        rows = [(arm - .0005, 0., 0., 0.)]
        for n in range(420):
            x, z = (.1 if n % 2 else -.1), (3. if n < 80 else .3)
            if n == 397:
                x, z = 1., 0.
            elif n == 398:
                x, z = 3., 0.
            elif n == 399:
                x, z = 5., 0.
            elif n >= 400:
                x, z = 10., 0.
            rows.append((arm + n * .0005, x, 0., z))
        raw_onset = rows[401][0]
        profile = self.owner.experimental_profile
        outcome = (model.replay_profile(profile, {"eligible": list(range(1, len(rows))),
                    "rows": rows, "onset": raw_onset}) if profile else None)
        index = outcome["sample_index"] if outcome else 401
        source_onset = rows[index][0]
        identity = (int(arm * 1000), 210, index, len(rows))
        record.update(arm_time_s=arm, onset_time_s=source_onset, onset_sample=identity,
                      onset_g=math.sqrt(sum(value * value for value in rows[index][1:])),
                      impact_peak_g=10., impact_metric="xyz_magnitude")
        detector = {"axis": axis, "attempt_count": 1, "arm_time_s": arm,
                    "onset_time_s": source_onset, "trigger_onset_time_s": source_onset,
                    "onset_sample_version": 1, "onset_sample": identity,
                    "onset_sample_matches": 1, "onset_sample_index": index,
                    "threshold_g": self.owner.threshold_g[axis], "onset_g": record["onset_g"],
                    "impact_peak_g": 10., "impact_metric": "xyz_magnitude", "direction": -1,
                    "homing_speed_mm_s": self.owner.rails_by_axis[axis].info.speed,
                    "homing_accel_mm_s2": self.owner.homing_accel,
                    "experimental_envelope": {"max_oldest_eligible_age_s": .010,
                        "max_newest_eligible_age_s": .005, "eligible_batches": 30, "future_sample_seen": False}}
        if outcome:
            # Deliberately simulate a changed timestamp conversion after home.
            # Candidate plot markers must use the retained-row clock domain.
            shift = .0001
            rows = [(row[0] + shift,) + row[1:] for row in rows]
            detector["onset_time_s"] = rows[index][0]
            original = dict(profile, **outcome, decision_sample=identity,
                            evidence_sample=(identity[0], identity[1], outcome["evidence_sample_index"], len(rows)))
            meta = dict(original,
                        capture_decision_time_s=rows[index][0], capture_decision_sample_index=index,
                        capture_evidence_time_s=rows[outcome["evidence_sample_index"]][0],
                        capture_evidence_sample_index=outcome["evidence_sample_index"],
                        capture_evidence_sample_matches=1)
            record["experimental"] = original
            detector["experimental"] = meta
        observer = self.owner.troubleshoot
        observer.record_event("arm", axis=axis, arm_print_time_s=arm)
        observer.record_event("stop_request", axis=axis, arm_print_time_s=arm,
                              onset_print_time_s=source_onset, onset_sample=identity,
                              request_estimated_print_time_s=source_onset + .002,
                              request_before_host_monotonic_s=self.reactor.monotonic())
        return {"samples": rows, "dropped": 0, "baseline_g": [0., 0., 0.],
                "_record": record, "detector": detector, "amplitude_limited": False}


class ExperimentRig(AccuracyRig):
    def __init__(self, options=None):
        super().__init__(options)
        self.reactor._check_gc = True
        self.reactor._check_timers = lambda *args: None
        self.reactor.get_gc_stats = lambda: (0., 0., 0.)
        self.owner.experimental_profile = None
        self.envelope = {"max_oldest_eligible_age_s": .010, "max_newest_eligible_age_s": .005,
                         "eligible_batches": 3, "future_sample_seen": False}
        self.receipt_mutator = None

    def stop_capture(self):
        capture = super().stop_capture()
        if self.envelope is not None and "detector" in capture:
            capture["detector"]["experimental_envelope"] = dict(self.envelope)
        profile = self.owner.experimental_profile
        if profile and "detector" in capture:
            detector = capture["detector"]
            identity = detector["onset_sample"]
            capture["samples"][1] = (capture["samples"][1][0], -4., -5., 1.)
            original = dict(profile, route="weighted_confirmed", decision_sample=identity,
                            evidence_sample=(identity[0], identity[1], 1, identity[3]),
                            evidence_time_s=capture["samples"][1][0], decision_time_s=capture["samples"][2][0])
            capture["_record"]["experimental"] = original
            detector["experimental"] = dict(original, capture_decision_sample_index=2,
                capture_decision_time_s=capture["samples"][2][0], capture_evidence_sample_index=1,
                capture_evidence_sample_matches=1, capture_evidence_time_s=capture["samples"][1][0])
            if self.receipt_mutator:
                self.receipt_mutator(capture)
        return capture


class EarlyShadowRig(RealFitRig):
    """Raw impact with an unverified, smaller signal ten milliseconds earlier."""
    def __init__(self, options=None):
        super().__init__(options)
        self.unsafe_home = None

    def stop_capture(self):
        capture = super().stop_capture()
        rows = capture["samples"]
        for index in (380, 381):
            rows[index] = (rows[index][0], 6., 0., 0.)
        if len(self.home_calls) == self.unsafe_home:
            for index in (100, 101):
                rows[index] = (rows[index][0], 6., 0., 0.)
        return capture


class ExperimentWorkflowTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(workflow_module, "TroubleshootRecorder", diagnostics.Recorder)
        patch.start()
        self.addCleanup(patch.stop)
        self.profile = {"version": 1, "axis": "x", "weights_xyz": [.75, 1.5, .75],
                        "threshold_g": 3., "raw_threshold_g": 8.,
                        "confirmation_s": .001, "max_age_s": .040}
        self.fit_calls = []

    def fit(self, captures, records, *, axis, checkpoint=None, max_age_s=.040):
        if checkpoint:
            checkpoint()
        self.fit_calls.append(([record["workflow_phase"] for record in records], len(captures), axis))
        return {"accepted": True, "profile": dict(self.profile, axis=axis, max_age_s=max_age_s),
                "reason": "synthetic fit for lifecycle test", "train_count": 6, "holdout_count": 4}

    def run_command(self, rig, render=False, **params):
        args = {"AXIS": "X", "CENTERED": 1, "SAMPLES": 2, **params}
        with mock.patch.object(workflow_module, "_load_experiment_module",
                               return_value=types.SimpleNamespace(fit_experimental_profile=self.fit)):
            if render:
                rig.workflow.cmd_EXPERIMENT(GCmd(**args))
            else:
                with mock.patch.object(diagnostics, "render_bundle"):
                    rig.workflow.cmd_EXPERIMENT(GCmd(**args))

    def restored(self, rig, old_profile=None):
        self.assertIsNone(rig.workflow.session)
        self.assertIsNone(rig.owner.troubleshoot)
        self.assertIs(rig.owner.experimental_profile, old_profile)
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertEqual(rig.owner.back_off_dist, 5.)
        self.assertFalse(rig.capture_active)
        self.assertFalse(rig.workflow.in_home)
        self.assertEqual(rig.configfile.saved, {})
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertEqual(rig.toolhead.position[2:], [40., 6.])
        self.assertTrue(all(coords[2] is None for coords, _speed in rig.toolhead.moves))

    def report(self, rig):
        return json.loads(Path(rig.workflow.last_result["artifacts"]["manifest"]).read_text(encoding="utf-8"))

    def test_invalid_parameters_and_unknown_initial_state_never_move(self):
        cases = ({"AXIS": ""}, {"AXIS": "XY"}, {"AXIS": "Z"}, {"CENTERED": 0},
                 {"SAVE_CONFIG": 0}, {"SAVE_CONFIG": 1}, {"SAMPLES": 1}, {"SAMPLES": 41},
                 {"SPEED": 50}, {"SPEED": 0}, {"SPEED": float("nan")}, {"THRESHOLD": 4},
                 {"GC_GUARD": 2}, {"MODE": "UNKNOWN"})
        for params in cases:
            with self.subTest(params=params):
                rig = ExperimentRig()
                before = rig.effects()
                with self.assertRaises(CommandError):
                    self.run_command(rig, **params)
                self.assertEqual(before, rig.effects())
        for setter in (lambda r: r.toolhead.kin.limits.__setitem__(0, (0., 300.)),
                       lambda r: setattr(r, "print_state", "printing"),
                       lambda r: r.objects.pop("homing"),
                       lambda r: delattr(r.reactor, "_check_gc")):
            rig = ExperimentRig()
            setter(rig)
            before = rig.effects()
            with self.assertRaises(CommandError):
                self.run_command(rig)
            self.assertEqual(before, rig.effects())

    def test_frozen_fit_sequence_separate_statistics_and_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            previous = {"previous_profile": True}
            rig.owner.experimental_profile = previous
            observed = []
            def check_home(axis):
                observed.append(rig.owner.experimental_profile)
                self.assertFalse(gc.isenabled())
                self.assertFalse(rig.reactor._check_gc)
                self.assertTrue(rig.workflow.simple_detection(axis))
            rig.on_home = check_home
            was_enabled = gc.isenabled()
            self.run_command(rig, SPEED=rig.owner.rails_by_axis["x"].info.speed)
            self.assertEqual(gc.isenabled(), was_enabled)
            self.assertTrue(rig.reactor._check_gc)
            self.assertEqual(len(rig.home_calls), 13)
            self.assertEqual(observed[:11], [None] * 11)
            self.assertEqual(observed[11:], [self.profile, self.profile])
            self.assertIs(observed[11], observed[12])
            self.assertEqual(self.fit_calls, [(["training"] * 6 + ["holdout"] * 4, 10, "x")])
            report = self.report(rig)
            self.assertEqual(report["command"], "ACCEL_HOMING_EXPERIMENT")
            self.assertEqual([home["phase"] for home in report["homes"]],
                             ["discovery"] + ["training"] * 6 + ["holdout"] * 4 + ["candidate"] * 2)
            self.assertTrue(all(home["gc_guard"]["restored"] for home in report["homes"]))
            self.assertTrue(report["homes"][11]["previous_return_completed"])
            stats = report["experiment"]["statistics"]
            self.assertEqual({phase: item["count"] for phase, item in stats.items()},
                             {"training": 6, "holdout": 4, "candidate": 2})
            result = rig.workflow.last_result
            self.assertEqual(result["completed_samples"], {"x": 2})
            self.assertEqual(result["statistics"]["x"]["count"], 2)
            self.assertTrue(all(result["experiment"]["runtime_restored"].values()))
            with zipfile.ZipFile(result["artifacts"]["archive"]) as archive:
                self.assertIsNone(archive.testzip())
                self.assertTrue(any(path.endswith("experiment.json") for path in archive.namelist()))
                self.assertEqual(sum(path.endswith(".csv.gz") for path in archive.namelist()), 13)
            self.restored(rig, previous)

    def test_rejected_fit_does_not_activate_or_return(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            self.fit = lambda *args, **kwargs: {"accepted": False, "profile": None, "reason": "heldout mismatch"}
            with self.assertRaisesRegex(CommandError, "heldout mismatch.*without automatic return"):
                self.run_command(rig)
            self.assertEqual(len(rig.home_calls), 11)
            self.assertEqual(len(rig.toolhead.moves), 10)
            report = self.report(rig)
            self.assertFalse(report["experiment"]["candidate_activated"])
            self.assertFalse(report["homes"][-1].get("return_completed", False))
            self.restored(rig)

    def test_failed_candidate_does_not_return_and_restores_disabled_gc(self):
        for fault in ({"travel_mm": 20.}, CommandError("native dispatch failed")):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                rig = ExperimentRig({"charts_directory": directory})
                rig.home_results.extend([{}] * 11 + [fault])
                was_enabled = gc.isenabled()
                gc.disable()
                rig.reactor._check_gc = False
                try:
                    with self.assertRaises(CommandError):
                        self.run_command(rig)
                    self.assertFalse(gc.isenabled())
                    self.assertFalse(rig.reactor._check_gc)
                finally:
                    if was_enabled:
                        gc.enable()
                self.assertEqual(len(rig.home_calls), 12)
                self.assertEqual(len(rig.toolhead.moves), 11)
                report = self.report(rig)
                self.assertEqual(report["ending"]["result"], "aborted")
                self.assertFalse(report["homes"][-1].get("return_completed", False))
                self.assertTrue(report["homes"][-1]["gc_guard"]["restored"])
                self.restored(rig)

    def test_cancel_during_home_restores_before_export(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            was_enabled = gc.isenabled()
            def cancel(axis):
                if len(rig.home_calls) == 12:
                    rig.workflow.cmd_CANCEL(GCmd())
            rig.on_home = cancel
            original = diagnostics.Recorder.export
            def export(recorder, ending):
                self.assertEqual(gc.isenabled(), was_enabled)
                self.assertTrue(rig.reactor._check_gc)
                self.assertIsNone(rig.owner.experimental_profile)
                self.assertFalse(rig.capture_active)
                return original(recorder, ending)
            with mock.patch.object(diagnostics.Recorder, "export", export):
                with self.assertRaisesRegex(CommandError, "finishing|cancelled"):
                    self.run_command(rig)
            self.assertEqual(rig.workflow.last_result["result"], "cancelled")
            self.assertEqual(len(rig.toolhead.moves), 11)
            self.restored(rig)

    def test_cancel_during_fit_has_no_candidate_or_automatic_return(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            def fit(captures, records, *, checkpoint, **kwargs):
                rig.workflow.cmd_CANCEL(GCmd())
                checkpoint()
            self.fit = fit
            with self.assertRaisesRegex(CommandError, "finishing|cancelled"):
                self.run_command(rig)
            self.assertEqual(len(rig.home_calls), 11)
            self.assertEqual(len(rig.toolhead.moves), 10)
            self.restored(rig)

    def test_guard_off_is_explicit_and_stationary_collection_failure_restores(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            del rig.reactor._check_gc
            with mock.patch.object(workflow_module.gc, "collect", side_effect=AssertionError("unexpected GC")):
                self.run_command(rig, GC_GUARD=0)
            self.assertTrue(all(not home["gc_guard"]["requested"] for home in self.report(rig)["homes"]))
            self.restored(rig)
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            was_enabled = gc.isenabled()
            with mock.patch.object(workflow_module.gc, "collect", side_effect=RuntimeError("collection failed")):
                with self.assertRaisesRegex(RuntimeError, "collection failed"):
                    self.run_command(rig)
            self.assertEqual(gc.isenabled(), was_enabled)
            self.assertTrue(rig.reactor._check_gc)
            self.assertEqual(rig.home_calls, [])
            self.assertTrue(self.report(rig)["homes"][0]["gc_guard"]["restored"])
            self.restored(rig)

    def test_export_failure_preserves_restored_runtime_and_error(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            with mock.patch.object(diagnostics.Recorder, "export", side_effect=OSError("disk full")):
                self.run_command(rig)
            self.assertIn("disk full", rig.workflow.last_result["export_error"])
            self.restored(rig)

    def test_reference_oldest_age_missing_future_and_excess_age_abort_before_candidate(self):
        for envelope in (None, {"max_oldest_eligible_age_s": .071, "max_newest_eligible_age_s": .001,
                                "eligible_batches": 3, "future_sample_seen": False},
                         {"max_oldest_eligible_age_s": .010, "max_newest_eligible_age_s": .005,
                          "eligible_batches": 3, "future_sample_seen": True}):
            with self.subTest(envelope=envelope), tempfile.TemporaryDirectory() as directory:
                rig = ExperimentRig({"charts_directory": directory})
                rig.envelope = envelope
                with self.assertRaisesRegex(CommandError, "freshness preflight rejected"):
                    self.run_command(rig)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertEqual(len(rig.toolhead.moves), 10)
                report = self.report(rig)
                self.assertFalse(report["experiment"]["freshness_preflight"]["accepted"])
                self.assertIsNone(report["experiment"]["fit"])
                self.assertEqual(self.fit_calls, [])
                self.restored(rig)

    def test_explicit_age_envelope_is_frozen_without_automatic_increase(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            rig.envelope["max_oldest_eligible_age_s"] = .071
            self.run_command(rig, MAX_SAMPLE_AGE=.080)
            report = self.report(rig)
            self.assertTrue(report["experiment"]["freshness_preflight"]["accepted"])
            self.assertEqual(report["experiment"]["fit"]["profile"]["max_age_s"], .080)
            self.restored(rig)

    def test_real_fitter_produces_candidates_with_exact_retained_sample_clock(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = RealFitRig({"charts_directory": directory})
            with mock.patch.object(workflow_module, "_load_experiment_module", return_value=model), \
                    mock.patch.object(diagnostics, "render_bundle"):
                rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, SAMPLES=2))
            report = self.report(rig)
            self.assertTrue(report["experiment"]["fit"]["accepted"])
            self.assertEqual(len(report["homes"]), 13)
            candidate = report["homes"][11]
            record, detector = candidate["record"], candidate["capture"]["detector"]
            self.assertEqual(record["experimental"]["route"], "weighted_confirmed")
            self.assertEqual(detector["experimental"]["capture_decision_time_s"], record["onset_time_s"])
            self.assertAlmostEqual(record["capture_clock_shift_s"], .0001)
            self.assertLess(detector["experimental"]["capture_evidence_time_s"], record["onset_time_s"])
            self.restored(rig)

    def run_shadow(self, rig):
        with mock.patch.object(workflow_module, "_load_experiment_module", return_value=model), \
                mock.patch.object(diagnostics, "render_bundle"):
            rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, SAMPLES=2, MODE="SHADOW"))

    def test_shadow_near_event_replay_never_arms_profile_and_exports_raw_homes(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = EarlyShadowRig({"charts_directory": directory})
            previous = {"previous_profile": True}
            rig.owner.experimental_profile = previous
            active = []
            rig.on_home = lambda axis: active.append(rig.owner.experimental_profile)
            self.run_shadow(rig)
            report = self.report(rig)
            experiment = report["experiment"]
            self.assertEqual(active, [None] * 13)
            self.assertFalse(experiment["candidate_activated"])
            self.assertFalse(experiment["fit"]["accepted"])
            self.assertIsNone(experiment["fit"]["profile"])
            self.assertEqual(experiment["fit"]["shadow"]["status"], "ready")
            self.assertEqual([h["phase"] for h in report["homes"]],
                             ["discovery"] + ["training"] * 6 + ["holdout"] * 4 + ["shadow"] * 2)
            for home in report["homes"][1:]:
                self.assertTrue(home["shadow_profile"]["shadow_only"])
                self.assertLess(home["shadow"]["reference_delta_ms"], -1.)
                self.assertTrue(home["shadow"]["within_negative_guard"])
                self.assertIsNone(home["record"].get("experimental"))
            self.assertEqual(experiment["statistics"]["shadow"]["count"], 2)
            self.assertEqual(rig.workflow.last_result["completed_samples"], {"x": 2})
            self.assertEqual(len(experiment["shadow_results"]), 2)
            self.restored(rig, previous)

    def test_shadow_rejects_heldout_startup_without_retraining_or_extra_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = EarlyShadowRig({"charts_directory": directory})
            rig.unsafe_home = 8
            with self.assertRaisesRegex(CommandError, "shadow profile rejected.*without automatic return"):
                self.run_shadow(rig)
            report = self.report(rig)
            self.assertEqual(len(rig.home_calls), 11)
            self.assertEqual(len(rig.toolhead.moves), 10)
            self.assertEqual(report["experiment"]["fit"]["shadow"]["status"], "rejected")
            self.assertFalse(report["homes"][7]["shadow"]["within_negative_guard"])
            self.assertFalse(report["experiment"]["candidate_activated"])
            self.restored(rig)

    def test_shadow_fresh_home_startup_retains_evidence_and_stops_without_return(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = EarlyShadowRig({"charts_directory": directory})
            rig.unsafe_home = 12
            with self.assertRaisesRegex(CommandError, "outside the near-event window"):
                self.run_shadow(rig)
            report = self.report(rig)
            self.assertEqual(len(rig.home_calls), 12)
            self.assertEqual(len(rig.toolhead.moves), 11)
            self.assertFalse(report["homes"][-1]["shadow"]["within_negative_guard"])
            self.assertFalse(report["homes"][-1].get("return_completed", False))
            self.assertEqual(rig.workflow.last_result["completed_samples"], {"x": 0})
            self.assertIsNone(report["experiment"]["statistics"]["shadow"])
            self.restored(rig)

    def test_cancel_during_shadow_replay_restores_before_any_return(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = EarlyShadowRig({"charts_directory": directory})
            def cancel(*args, **kwargs):
                rig.workflow.cmd_CANCEL(GCmd())
                kwargs["checkpoint"]()
            with mock.patch.object(model, "replay_shadow_capture", side_effect=cancel):
                with self.assertRaisesRegex(CommandError, "finishing|cancelled"):
                    self.run_shadow(rig)
            self.assertEqual(len(rig.home_calls), 12)
            self.assertEqual(len(rig.toolhead.moves), 11)
            self.assertEqual(rig.workflow.last_result["result"], "cancelled")
            self.restored(rig)

    def test_shadow_missing_helper_rejects_before_motion(self):
        rig = ExperimentRig()
        before = rig.effects()
        with self.assertRaisesRegex(CommandError, "does not support MODE=SHADOW"):
            self.run_command(rig, MODE="SHADOW")
        self.assertEqual(before, rig.effects())

    def test_bad_candidate_receipt_stops_without_return(self):
        changes = (lambda meta: meta.update(weights_xyz=[1., 1., 1.]),
                   lambda meta: meta.update(route="unknown"),
                   lambda meta: meta.update(capture_evidence_sample_matches=0),
                   lambda meta: meta.update(capture_evidence_sample_index=0),
                   lambda meta: meta.update(decision_sample=(0, 1, 0, 2)),
                   lambda meta: meta.update(capture_evidence_time_s=0.))
        for change in changes:
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                rig = ExperimentRig({"charts_directory": directory})
                rig.receipt_mutator = lambda capture: change(capture["detector"]["experimental"])
                with self.assertRaisesRegex(CommandError, "experimental capture receipt invalid"):
                    self.run_command(rig)
                self.assertEqual(len(rig.home_calls), 12)
                self.assertEqual(len(rig.toolhead.moves), 11)
                report = self.report(rig)
                self.assertIsNone(report["experiment"]["statistics"]["candidate"])
                self.assertFalse(report["homes"][-1].get("return_completed", False))
                self.restored(rig)

    def test_raw_bypass_receipt_uses_same_evidence_and_decision_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            def raw(capture):
                capture["samples"][2] = (capture["samples"][2][0], 10., 0., 1.)
                meta = capture["detector"]["experimental"]
                original = capture["_record"]["experimental"]
                for item in (meta, original):
                    item.update(route="raw_bypass", evidence_sample=item["decision_sample"],
                                evidence_time_s=item["decision_time_s"])
                meta.update(capture_evidence_sample_index=2,
                            capture_evidence_time_s=capture["samples"][2][0])
            rig.receipt_mutator = raw
            self.run_command(rig)
            report = self.report(rig)
            self.assertTrue(all(home["record"]["experimental_capture_verified"]
                                for home in report["homes"] if home["phase"] == "candidate"))
            self.restored(rig)

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "matplotlib unavailable")
    def test_real_phase_chart_and_candidate_trace_are_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ExperimentRig({"charts_directory": directory})
            self.run_command(rig, render=True)
            result = rig.workflow.last_result
            self.assertEqual(result["artifacts"]["render"]["result"], "complete")
            self.assertIn("experiment-phases.png", result["artifacts"]["render"]["files"])
            self.assertGreater((Path(result["artifacts"]["folder"]) / "experiment-phases.png").stat().st_size, 10000)
            self.restored(rig)


if __name__ == "__main__":
    unittest.main()
