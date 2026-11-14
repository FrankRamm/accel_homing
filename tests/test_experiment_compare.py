"""RAW paired delivery comparison, with the real backend stream lifecycle."""
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError
from test_experiment_workflow import RealFitRig
from test_workflows import GCmd, workflow_module
from accel_homing_backends import ConfiguredAccelBackend
import accel_homing_experiment as model
import accel_homing_troubleshoot as diagnostics


class SharedBatchHelper:
    def __init__(self, interval=.100):
        self.batch_interval = interval
        self.batch_timer = object()
        self.clients = [lambda message: True]  # Existing unrelated client.
        self.add_error = None

    def add_client(self, client):
        self.clients.append(client)
        if self.add_error:
            raise self.add_error


class CompareRig(RealFitRig):
    def __init__(self, options=None, helper_interval=.100):
        super().__init__(options)
        self.owner.sensor_name = "adxl345"
        self.owner.stream_on = False
        self.owner.main_mcu = types.SimpleNamespace(estimated_print_time=lambda now: now)
        self.helper = SharedBatchHelper(helper_interval)
        sensor = types.SimpleNamespace(batch_bulk=self.helper, data_rate=2000.)
        self.owner.backend = ConfiguredAccelBackend(self.owner, self.config, sensor)
        self.starts = []
        self.backend_stop_error = None
        self.observe_interval = True
        self.after_start = None

    def start_capture(self):
        super().start_capture()
        self.capture_home_before = len(self.home_calls)

    def stock_home(self, gcmd):
        self.owner.backend.start()
        self.owner.stream_on = True
        self.starts.append((self.owner.backend.interval, self.helper.batch_interval,
                            self.reactor.timers[self.helper.batch_timer] - self.reactor.monotonic(),
                            self.owner.experimental_profile))
        if self.after_start:
            self.after_start()
        if self.observe_interval:
            self.owner.troubleshoot.record_batch("klipper_batch", callback_entry_host_monotonic_s=self.reactor.monotonic())
        super().stock_home(gcmd)

    def stop_capture(self):
        if len(self.home_calls) == self.capture_home_before:
            self.capture_active = False
            self.capture_stops += 1
            return {"samples": [], "dropped": 0}
        return super().stop_capture()

    def end_session(self):
        super().end_session()
        backend = getattr(self.owner, "backend", None)
        if backend is not None:
            if self.backend_stop_error:
                raise self.backend_stop_error
            self.owner.stream_on = False
            backend.stop()


class CompareWorkflowTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(workflow_module, "TroubleshootRecorder", diagnostics.Recorder)
        patch.start()
        self.addCleanup(patch.stop)
        self.fit_calls = []
        self.fit_status = "ready"
        self.replays = []
        self.invalid_replay = False
        self.fit_exception = None

    def fit(self, captures, records, **kwargs):
        kwargs["checkpoint"]()
        self.fit_calls.append((len(captures), [r["workflow_phase"] for r in records], kwargs))
        if self.fit_exception:
            raise self.fit_exception
        return {"accepted": False, "profile": None, "reason": "diagnostic only",
                "shadow": {"status": self.fit_status, "reason": "synthetic shadow fit",
                           "profile": {"shadow_only": True, "threshold_g": 3., "axis": kwargs["axis"]},
                           "training": [], "holdout": []}}

    def replay(self, profile, capture, record, **kwargs):
        kwargs["checkpoint"]()
        self.replays.append(profile)
        return {"within_negative_guard": True, "no_later_than_reference": True,
                "noise_margin": {"qualified": not self.invalid_replay,
                                 "reason_codes": ["startup_insufficient_raw_bypass_margin"] if self.invalid_replay else []}}

    def run_compare(self, rig, **params):
        module = types.SimpleNamespace(fit_experimental_profile=self.fit, replay_shadow_capture=self.replay)
        with mock.patch.object(workflow_module, "_load_experiment_module", return_value=module), \
                mock.patch.object(diagnostics, "render_bundle"):
            rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, MODE="COMPARE", **{"SAMPLES": 2, **params}))

    def report(self, rig):
        return json.loads(Path(rig.workflow.last_result["artifacts"]["manifest"]).read_text(encoding="utf-8"))

    def restored(self, rig, previous=None):
        self.assertIsNone(rig.workflow.session)
        self.assertIsNone(rig.owner.troubleshoot)
        self.assertIs(rig.owner.experimental_profile, previous)
        self.assertFalse(rig.capture_active)
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertEqual(rig.owner.back_off_dist, 5.)
        self.assertEqual(rig.configfile.saved, {})
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertTrue(all(coords[2] is None for coords, _speed in rig.toolhead.moves))

    def test_real_backend_pairs_are_balanced_and_restore_before_returns(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = CompareRig({"charts_directory": directory})
            previous = {"previous_live_profile": True}
            rig.owner.experimental_profile = previous
            returns = rig.workflow._diagnostic_recenter
            def recenter(*args):
                self.assertEqual(rig.owner.backend.interval, .005)
                self.assertEqual(rig.helper.batch_interval, .100)
                self.assertFalse(rig.owner.stream_on)
                return returns(*args)
            with mock.patch.object(rig.workflow, "_diagnostic_recenter", side_effect=recenter):
                self.run_compare(rig)
            report = self.report(rig)
            homes, experiment = report["homes"], report["experiment"]
            self.assertEqual(len(homes), 15)
            self.assertEqual([h["delivery_variant"] for h in homes[11:]], ["control", "fast", "fast", "control"])
            self.assertEqual([(h["comparison_pair"], h["comparison_order"]) for h in homes[11:]],
                             [(1, 1), (1, 2), (2, 1), (2, 2)])
            self.assertEqual([h["phase"] for h in homes[11:]],
                             ["delivery_control", "delivery_fast", "delivery_fast", "delivery_control"])
            expected = [.005] * 12 + [.002, .002, .005]
            self.assertEqual([start[0] for start in rig.starts], expected)
            self.assertEqual([start[1] for start in rig.starts], expected)
            for start, interval in zip(rig.starts, expected):
                self.assertAlmostEqual(start[2], interval)
                self.assertIsNone(start[3])
            self.assertTrue(all(h["delivery_interval_restored"] for h in homes))
            self.assertEqual([h["delivery_intervention_applied"] for h in homes[11:]], [False, True, True, False])
            self.assertEqual(rig.workflow.last_result["completed_samples"], {"x": 4})
            self.assertEqual(experiment["statistics"]["delivery_control"]["count"], 2)
            self.assertEqual(experiment["statistics"]["delivery_fast"]["count"], 2)
            self.assertTrue(all(experiment["runtime_restored"].values()))
            self.assertFalse(experiment["candidate_activated"])
            self.assertEqual(self.fit_calls[0][:2], (10, ["training"] * 6 + ["holdout"] * 4))
            self.assertEqual({k: self.fit_calls[0][2][k] for k in ("shadow", "noise_margin_fraction", "noise_margin_g")},
                             {"shadow": True, "noise_margin_fraction": .10, "noise_margin_g": .5})
            self.assertTrue(all(p is self.replays[0] for p in self.replays))
            self.restored(rig, previous)
            # An ordinary subsequent stream still requests the original policy.
            backend = rig.owner.backend
            timeout = backend.motion_startup_timeout(100., 5000.)
            backend.start()
            self.assertEqual(backend.applied_interval, .005)
            self.assertEqual(backend.motion_startup_timeout(100., 5000.), timeout)
            backend.stop()
            self.assertEqual(rig.helper.batch_interval, .100)
            self.assertTrue(rig.helper.clients[0]({}))
            self.assertTrue(all(client({}) is False for client in rig.helper.clients[1:]))

    def test_rejected_fit_and_fresh_margin_failure_do_not_stop_raw_delivery(self):
        for rejected in (True, False):
            with self.subTest(rejected=rejected), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                self.fit_status = "rejected" if rejected else "ready"
                self.invalid_replay = True
                self.replays.clear()
                self.run_compare(rig)
                report = self.report(rig)
                self.assertEqual(len(rig.home_calls), 15)
                self.assertEqual(report["ending"]["result"], "complete")
                self.assertEqual(report["experiment"]["shadow_validation"]["status"], "rejected" if rejected else "invalid")
                self.assertEqual(len(self.replays), 0 if rejected else 4)
                self.assertFalse(report["experiment"]["candidate_activated"])
                self.restored(rig)

    def test_fitting_failure_or_freshness_rejection_retains_raw_comparison(self):
        for failure in ("fit", "freshness"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                self.fit_exception = ValueError("unusable shadow evidence") if failure == "fit" else None
                if failure == "freshness":
                    stop = rig.stop_capture
                    def stale():
                        capture = stop()
                        capture["detector"]["experimental_envelope"]["max_oldest_eligible_age_s"] = .2
                        return capture
                    rig.owner.stop_capture = stale
                self.replays.clear()
                self.run_compare(rig)
                self.assertEqual(len(rig.home_calls), 15)
                self.assertEqual(self.report(rig)["experiment"]["shadow_validation"]["status"], "rejected")
                self.assertEqual(self.replays, [])
                self.restored(rig)

    def test_cancel_and_raw_failure_stop_without_return_and_restore_intervals(self):
        for action in ("cancel", "raw_failure"):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                if action == "cancel":
                    rig.on_home = lambda axis: rig.workflow.cmd_CANCEL(GCmd()) if len(rig.home_calls) == 13 else None
                else:
                    rig.home_results.extend([{}] * 12 + [{"travel_mm": 20.}])
                with self.assertRaises(CommandError):
                    self.run_compare(rig)
                self.assertEqual(len(rig.home_calls), 13)
                self.assertEqual(len(rig.toolhead.moves), 12)
                self.assertEqual(rig.owner.backend.interval, .005)
                self.assertEqual(rig.helper.batch_interval, .100)
                self.assertFalse(rig.owner.stream_on)
                self.assertFalse(self.report(rig)["homes"][-1].get("return_completed", False))
                self.restored(rig)

    def test_backend_start_and_stop_failures_cannot_report_success(self):
        for action in ("start", "stop"):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                if action == "start":
                    rig.helper.add_error = RuntimeError("start failed")
                else:
                    rig.backend_stop_error = RuntimeError("stop failed")
                with self.assertRaises(Exception):
                    self.run_compare(rig)
                report = self.report(rig)
                self.assertEqual(report["ending"]["result"], "aborted")
                self.assertFalse(report["homes"][-1].get("return_completed", False))
                if action == "start":
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(rig.owner.backend.interval, .005)
                    self.assertEqual(rig.helper.batch_interval, .100)
                else:
                    self.assertFalse(report["experiment"]["runtime_restored"]["interval"])

    def test_fast_start_failure_restores_request_before_export(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = CompareRig({"charts_directory": directory})
            add_client = rig.helper.add_client
            def fail_fast(client):
                if rig.owner.backend.interval == .002:
                    raise RuntimeError("fast stream registration failed")
                add_client(client)
            rig.helper.add_client = fail_fast
            with self.assertRaisesRegex(RuntimeError, "fast stream registration failed"):
                self.run_compare(rig)
            self.assertEqual(len(rig.home_calls), 12)
            self.assertEqual(len(rig.toolhead.moves), 12)
            self.assertEqual(rig.owner.backend.interval, .005)
            self.assertEqual(rig.helper.batch_interval, .100)
            self.assertTrue(self.report(rig)["experiment"]["runtime_restored"]["interval"])
            self.restored(rig)

    def test_cancel_during_fit_or_replay_is_never_converted_to_diagnostic_rejection(self):
        for phase, homes in (("fit", 11), ("replay", 12)):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                def cancel(*args, **kwargs):
                    rig.workflow.cmd_CANCEL(GCmd())
                    kwargs["checkpoint"]()
                with mock.patch.object(self, phase, side_effect=cancel):
                    with self.assertRaisesRegex(CommandError, "finishing|cancelled"):
                        self.run_compare(rig)
                self.assertEqual(len(rig.home_calls), homes)
                self.assertEqual(len(rig.toolhead.moves), homes - 1)
                self.assertEqual(rig.workflow.last_result["result"], "cancelled")
                self.assertEqual(rig.owner.backend.interval, .005)
                self.restored(rig)

    def test_external_helper_or_backend_change_is_preserved_and_aborts(self):
        for target in ("helper", "backend"):
            with self.subTest(target=target), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                def external(axis):
                    if len(rig.home_calls) == 13:
                        setattr(rig.helper if target == "helper" else rig.owner.backend,
                                "batch_interval" if target == "helper" else "interval", .003)
                rig.on_home = external
                with self.assertRaisesRegex(CommandError, "interval changed"):
                    self.run_compare(rig)
                self.assertEqual(len(rig.home_calls), 13)
                self.assertEqual(len(rig.toolhead.moves), 12)
                self.assertEqual(getattr(rig.helper if target == "helper" else rig.owner.backend,
                                         "batch_interval" if target == "helper" else "interval"), .003)
                self.assertFalse(self.report(rig)["experiment"]["runtime_restored"]["interval"])
                self.restored(rig)

    def test_missing_or_inconsistent_delivery_observation_aborts(self):
        for action in ("missing", "changed"):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                if action == "missing":
                    rig.observe_interval = False
                else:
                    def change(axis):
                        rig.helper.batch_interval = .004
                        rig.owner.troubleshoot.record_batch("klipper_batch", callback_entry_host_monotonic_s=rig.reactor.monotonic())
                        rig.helper.batch_interval = rig.owner.backend.applied_interval
                    rig.on_home = change
                with self.assertRaisesRegex(CommandError, "not consistently observed"):
                    self.run_compare(rig)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.toolhead.moves, [])
                self.restored(rig)

    def test_invalid_driver_shape_or_dirty_stream_exports_preflight_without_motion(self):
        for mutate in (lambda r: setattr(r.owner.backend, "interval_attr", "unknown"),
                       lambda r: setattr(r.owner.backend, "interval", float("nan")),
                       lambda r: setattr(r.owner.backend, "active", True),
                       lambda r: setattr(r.owner.backend, "saved_interval", .1)):
            with self.subTest(mutate=mutate), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                mutate(rig)
                with self.assertRaisesRegex(CommandError, "preflight rejected"):
                    self.run_compare(rig)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertFalse(self.report(rig)["experiment"]["delivery_comparison"]["preflight_safe"])

    def test_beacon_and_no_effective_shortening_are_truthfully_unavailable(self):
        for kind in ("beacon", "normal_2ms", "helper_1ms", "normal_10ms"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                options = {"charts_directory": directory}
                if kind == "beacon":
                    rig = RealFitRig(options)
                else:
                    if kind == "normal_2ms":
                        options["stream_interval"] = .002
                    elif kind == "normal_10ms":
                        options["stream_interval"] = .010
                    rig = CompareRig(options, helper_interval=.001 if kind == "helper_1ms" else .100)
                self.run_compare(rig)
                report = self.report(rig)
                comparison = report["experiment"]["delivery_comparison"]
                self.assertFalse(comparison["supported"])
                self.assertFalse(comparison["intervention_requested"])
                self.assertFalse(comparison["intervention_observed"])
                self.assertTrue(all(h["delivery_variant"] == "unavailable" for h in report["homes"]))
                self.assertTrue(all(not h["delivery_intervention_applied"] for h in report["homes"]))
                self.assertEqual(len(rig.home_calls), 15)
                self.restored(rig)

    def test_real_margin_fitter_never_arms_live_and_replays_new_raw_homes(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = CompareRig({"charts_directory": directory})
            with mock.patch.object(workflow_module, "_load_experiment_module", return_value=model), \
                    mock.patch.object(diagnostics, "render_bundle"):
                rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, SAMPLES=2, MODE="COMPARE"))
            experiment = self.report(rig)["experiment"]
            self.assertFalse(experiment["fit"]["accepted"])
            self.assertEqual(experiment["fit"]["shadow"]["status"], "ready")
            self.assertEqual(len(experiment["shadow_results"]), 4)
            self.assertTrue(all(r["noise_margin"]["qualified"] for r in experiment["shadow_results"]))
            self.assertTrue(all(start[3] is None for start in rig.starts))
            self.restored(rig)

    def test_pair_count_bounds_fit_within_retained_home_budget(self):
        for pairs in (10, 40):
            with self.subTest(pairs=pairs), tempfile.TemporaryDirectory() as directory:
                rig = CompareRig({"charts_directory": directory})
                self.run_compare(rig, SAMPLES=pairs)
                self.assertEqual(len(rig.home_calls), 11 + 2 * pairs)
                self.assertEqual(len(self.report(rig)["homes"]), 11 + 2 * pairs)
                self.assertEqual(rig.workflow.last_result["completed_samples"], {"x": 2 * pairs})
                self.restored(rig)


if __name__ == "__main__":
    unittest.main()
