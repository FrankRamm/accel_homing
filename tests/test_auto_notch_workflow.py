"""AUTO lifecycle, exact threshold, transactional persistence, and worker isolation."""
import contextlib
import copy
import importlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, FakeReactor, plugin
from accel_homing_filter import CausalNotch, DISABLED, profile, replay
from accel_homing_charts import PNGRenderer
import accel_homing_auto as auto
from test_accuracy_workflow import AccuracyRig
from test_beta_calibration import BetaRig
from test_workflows import GCmd, workflow_module
from native_configfile_fixtures import KlipperConfig, KalicoConfig
import test_troubleshoot_imports as upgrade_tests


class AutoRig(BetaRig):
    def __init__(self, directory, options=None):
        super().__init__({"charts_directory": directory, **(options or {})})
        self.sensor_time = 0.
        self.capture_mutator = None
        self.owner.notch_profiles = {"x": profile(150., 5.), "y": dict(DISABLED)}
        self.owner.filter_settings = lambda axis: dict(self.owner.notch_profiles[axis])
        self.selected = profile(100., 10.)
        self.selection_threshold = 1.25
        self.analysis_seen = []
        self.original_pending = {}
        self.workflow.renderer.reactor = self.reactor

    def stock_home(self, gcmd):
        self.sensor_time += 1.
        super().stock_home(gcmd)
        record = self.owner.last[self.home_calls[-1]["axis"]]
        record.update(arm_time_s=self.sensor_time + .001, onset_time_s=self.sensor_time + .002)

    def stop_capture(self):
        capture = AccuracyRig.stop_capture(self)
        record = capture["_record"]
        settings = self.owner.filter_settings(record["axis"])
        metadata = dict(DISABLED) if settings == DISABLED else CausalNotch(settings, 1000.).metadata()
        record.update(signal_filter=dict(metadata), impact_metric="xyz_magnitude")
        capture["detector"].update(signal_filter=dict(metadata), impact_metric="xyz_magnitude")
        capture["auto_notch_replay"] = {
            "sample_rate_hz": 1000., "sample_rate_source": "fresh_reference_timestamps",
            "reference_sample_count": 101, "reference_first_time_s": self.sensor_time - .1,
            "reference_last_time_s": self.sensor_time, "baseline_source": "armed_detector_reference",
            "first_sample_index": 1, "first_sample_matches": 1, "first_sample_time_s": self.sensor_time + .001,
            "initial_state": "zero_at_first_armed_sample", "complete": True}
        if settings != DISABLED:
            metadata.update(first_sample_index=1, first_sample_matches=1)
            capture["signal_filter"] = metadata
            capture["filtered_samples"] = replay(capture["samples"], capture["baseline_g"], metadata, 1)
        return capture

    def analyze(self, payload, renderer, check):
        check()
        if payload.get("preflight"):
            return {"numpy_version": "synthetic", "minimum_cruise_seconds": .29}
        self.analysis_seen.append(payload)
        if len(payload["motions"]) != 3 or len(payload["homes"]) != 10:
            raise AssertionError("analysis must follow complete independent collection")
        if getattr(self.configfile, "saved", {}) != self.original_pending:
            raise AssertionError("raw phase staged configuration")
        return {"status": "no_benefit" if self.selected == DISABLED else "selected",
                "decision": "disabled" if self.selected == DISABLED else "selected",
                "profile": dict(self.selected), "threshold_g": self.selection_threshold,
                "evidence": {}, "policy": {}, "plot": {"spectra": [], "traces": []}}

    @contextlib.contextmanager
    def helpers(self):
        with self.beta_helpers(), mock.patch.object(auto, "run_analysis", side_effect=self.analyze):
            yield

    def command(self, **options):
        package = workflow_module.__package__
        parent = types.ModuleType(package)
        parent.__path__ = [str(Path(auto.__file__).parent)]
        import accel_homing_backends
        with mock.patch.dict(sys.modules, {package: parent,
                package + ".accel_homing_backends": accel_homing_backends}):
            self.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, NOTCH="AUTO", **options))


class AutoNotchWorkflowTests(unittest.TestCase):
    def test_complete_two_phase_run_is_report_only_and_tests_exact_fixed_threshold(self):
        for disabled, retries in ((False, 0), (True, 0), (False, 45)):
            with self.subTest(disabled=disabled, retries=retries), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory)
                if disabled:
                    rig.selected = dict(DISABLED)
                if retries:
                    rig.home_results.extend([{"travel_mm": 2.}] +
                                            [{}, {"travel_mm": 150.}, {"travel_mm": 153.}] * retries)
                before = copy.deepcopy(rig.owner.notch_profiles)
                rig.configfile.set("extruder", "pid_kp", "42")
                pending = dict(rig.configfile.saved)
                rig.original_pending = pending
                with rig.helpers():
                    rig.command(START_THRESHOLD=.5, STEP=.1)
                self.assertEqual(len(rig.home_calls), 22 + retries * 3 + (1 if retries else 0))
                self.assertEqual([call["threshold"] for call in rig.home_calls[-11:]], [1.25] * 11)
                self.assertEqual(len([e for e in rig.beta_events if e[0] == "central_move"]), 6)
                self.assertEqual(rig.owner.notch_profiles, before)
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertEqual(rig.configfile.saved, pending)
                result = rig.workflow.last_result
                if retries:
                    self.assertGreater(result["attempts"], 64)
                    self.assertEqual(result["auto_notch"]["superseded_reference_captures"], 2 * retries)
                    self.assertEqual(result["auto_notch"]["omitted_attempt_records"], result["attempts"] - 128)
                self.assertEqual(result["result"], "auto_notch_proposed")
                self.assertFalse(result["model_staged"])
                report = json.loads(Path(result["artifacts"]["report"]).read_text())
                self.assertEqual(len(report["sources"]), 24)
                self.assertEqual(report["proposed_model"]["saved_threshold_g"], 1.25)
                self.assertEqual(report["proposed_model"]["threshold_margin_g"], 0.)
                self.assertTrue(report["proposed_model"]["saved_threshold_tested"])
                self.assertTrue(Path(result["artifacts"]["png"]).is_file())

    def test_invalid_cli_manual_override_and_short_stroke_fail_before_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            for options in ({"x_threshold": 2.}, {"threshold": 2.}):
                rig = AutoRig(directory, options)
                with self.assertRaisesRegex(CommandError, "explicit threshold"):
                    rig.command(SAVE_CONFIG=1)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.toolhead.moves, [])
            rig = AutoRig(directory)
            with self.assertRaisesRegex(CommandError, "NOTCH must be AUTO"):
                rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, NOTCH="BAD"))
            rig.owner.rails_by_axis["x"].info.speed = 400.
            before = copy.deepcopy(rig.owner.notch_profiles)
            with rig.helpers(), self.assertRaisesRegex(CommandError, "central stroke"):
                rig.command()
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.owner.notch_profiles, before)
            rig = AutoRig(directory)
            rig.owner.full_scale_g = None
            with self.assertRaisesRegex(CommandError, "known finite sensor range"):
                rig.command()
            self.assertEqual(rig.toolhead.moves, [])

    def test_cancel_analysis_failure_and_failed_fixed_qualification_restore_without_staging(self):
        for fault in ("cancel", "insufficient", "early", "noise_cancel"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory)
                before = copy.deepcopy(rig.owner.notch_profiles)
                if fault == "early":
                    rig.home_results.extend([{}] * 11 + [{"travel_mm": 2.}])
                elif fault == "noise_cancel":
                    rig.on_noise = lambda *_: rig.workflow.cmd_CANCEL(GCmd())
                original = rig.analyze
                def analyze(payload, renderer, check):
                    if not payload.get("preflight"):
                        if fault == "cancel":
                            rig.workflow.cmd_CANCEL(GCmd())
                            check()
                        if fault == "insufficient":
                            return {"status": "insufficient_data", "decision": None, "reasons": ["ambiguous spectrum"]}
                    return original(payload, renderer, check)
                with rig.helpers(), mock.patch.object(auto, "run_analysis", side_effect=analyze), self.assertRaises(CommandError):
                    rig.command()
                self.assertEqual(rig.owner.notch_profiles, before)
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertEqual(rig.configfile.saved, {})
                self.assertIsNone(rig.workflow.session)
                if fault == "early":
                    self.assertEqual(len(rig.home_calls), 12)
                    self.assertEqual(rig.home_calls[-1]["threshold"], 1.25)

    def native_config(self, rig, root, config_class):
        (root / "printer.cfg").write_text("[include homing.cfg]\n")
        (root / "homing.cfg").write_text("[accel_homing]\nx_notch_frequency: 150\nx_notch_q: 5\ny_notch_frequency: 0\n")
        rig.get_start_args = lambda: {"config_file": str(root / "printer.cfg")}
        rig.gcode.request_restart = mock.Mock()
        rig.gcode.error = CommandError
        config = config_class(rig)
        if hasattr(config, "_build_config_wrapper"):
            config.autosave = config._build_config_wrapper("", str(root / "printer.cfg"))
        rig.configfile = rig.objects["configfile"] = config
        return config

    def test_native_save_round_trip_and_failure_restore_files_and_pending_for_both_forks(self):
        for config_class in (KlipperConfig, KalicoConfig):
            for failure in (None, "native", "render", "pre_save", "final_report"):
                with self.subTest(fork=config_class.__name__, failure=failure), tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    rig = AutoRig(directory)
                    config = self.native_config(rig, root, config_class)
                    before = {p: p.read_bytes() for p in root.glob("*.cfg")}
                    owner = config if hasattr(config, "cmd_SAVE_CONFIG") else config.autosave
                    center = auto.load_companion_module("accel_homing_center")
                    original_snapshot, original_report = center.CenterCalibration._save_snapshot, auto.AutoNotchCalibration._write_report
                    snapshots = 0
                    def snapshot(controller, keys):
                        nonlocal snapshots
                        snapshots += 1
                        if snapshots == 2 and failure == "pre_save":
                            raise CommandError("pre-save snapshot failure")
                        return original_snapshot(controller, keys)
                    def report(controller, session, result):
                        if failure == "final_report" and result.get("persistence") == "native_save_requested":
                            raise OSError("final report refresh failure")
                        return original_report(controller, session, result)
                    def render(payload):
                        if failure == "render" and payload.get("kind") == "notch_auto":
                            raise OSError("comparison render failure")
                    rig.workflow.renderer.on_render = render
                    with rig.helpers(), contextlib.ExitStack() as stack:
                        stack.enter_context(mock.patch.object(center.CenterCalibration, "_save_snapshot", snapshot))
                        stack.enter_context(mock.patch.object(auto.AutoNotchCalibration, "_write_report", report))
                        if failure == "native":
                            stack.enter_context(mock.patch.object(type(owner), "cmd_SAVE_CONFIG", side_effect=CommandError("native save failure")))
                        if failure in ("native", "render", "pre_save"):
                            with self.assertRaisesRegex((CommandError, OSError), "failure"):
                                rig.command(SAVE_CONFIG=1)
                        else:
                            rig.command(SAVE_CONFIG=1)
                    self.assertEqual(rig.owner.notch_profiles["x"], profile(150., 5.))
                    if failure in ("native", "render", "pre_save"):
                        self.assertEqual({p: p.read_bytes() for p in root.glob("*.cfg")}, before)
                        self.assertFalse(owner.save_config_pending)
                        self.assertEqual(owner.status_save_pending, {})
                        self.assertEqual(rig.workflow.last_result["result"], "save_failed" if failure == "native" else "aborted")
                        self.assertFalse(rig.workflow.last_result["save_config"])
                        self.assertIsNone(rig.workflow.last_result["saved_threshold_g"])
                        self.assertFalse(rig.workflow.last_result["model_staged"])
                        self.assertNotIn("x", rig.toolhead.kin.get_status(0)["homed_axes"])
                    else:
                        self.assertTrue(rig.workflow.last_result["save_config"])
                        loaded = config.read_main_config().fileconfig
                        self.assertEqual(loaded.getfloat("accel_homing", "x_notch_frequency"), 100.)
                        self.assertEqual(loaded.getfloat("accel_homing_model default", "x_threshold"), 1.25)
                        self.assertEqual(loaded.get("accel_homing_model default", "version"), "2")
                        self.assertEqual(loaded.getfloat("accel_homing", "y_notch_frequency"), 0.)
                        rig.gcode.request_restart.assert_called_once()
                        if failure == "final_report":
                            self.assertIn("final report refresh failure", rig.workflow.last_result["artifact_report_error"])

    def test_old_links_load_auto_companions_and_worker_uses_private_numpy(self):
        fixture = upgrade_tests.TroubleshootImportTests()
        for entrypoint in ("accel_homing", "beacon_accel_homing"):
            with self.subTest(entrypoint=entrypoint), fixture.installation() as (checkout, extras):
                for name in ("accel_homing_auto.py", "accel_homing_auto_notch.py", "accel_homing_center.py"):
                    shutil.copyfile(Path(auto.__file__).with_name(name), checkout / name)
                importlib.import_module("extras." + entrypoint)
                backends = importlib.import_module("extras.accel_homing_backends")
                helper = backends.load_companion_module("accel_homing_auto")
                analyzer = backends.load_companion_module("accel_homing_auto_notch")
                self.assertEqual(Path(helper.__file__), checkout / "accel_homing_auto.py")
                self.assertEqual(Path(analyzer.__file__), checkout / "accel_homing_auto_notch.py")
                self.assertFalse((extras / "accel_homing_auto.py").exists())
        with tempfile.TemporaryDirectory() as directory:
            # A sentinel private-only dependency proves worker import precedence
            # without importing NumPy into Klippy or using another interpreter.
            (Path(directory) / "numpy.py").write_text("__version__ = 'private-only-sentinel'\n")
            renderer = PNGRenderer(sys.executable, directory)
            result = auto.run_analysis({"preflight": True}, renderer, lambda: None)
            self.assertEqual(result["numpy_version"], "private-only-sentinel")
        # Real numerical selection and PNG rendering run in the configured
        # subprocess, exercising JSON serialization and the new payload kind.
        from test_auto_notch import dataset, RAW_THRESHOLD
        import accel_homing_charts as charts
        motions, homes = dataset()
        payload = {"motions": motions, "homes": homes, "raw_threshold_g": RAW_THRESHOLD, "threshold_max_g": 8.}
        with tempfile.TemporaryDirectory() as directory:
            renderer = PNGRenderer(sys.executable, directory)
            report = auto.run_analysis(payload, renderer, lambda: None)
            self.assertEqual(report["decision"], "selected")
            paths = charts.write_notch_auto(directory, "simulated", "x", report, renderer=renderer, simulated=True)
            self.assertGreater(Path(paths["png"]).stat().st_size, 10000)
            processes = []
            def cancel():
                if processes:
                    raise CommandError("cancelled during worker")
            original_launch = auto.subprocess.Popen
            def launch_worker(*args, **kwargs):
                process = original_launch(*args, **kwargs)
                processes.append(process)
                return process
            with mock.patch.object(auto.subprocess, "Popen", side_effect=launch_worker) as launch:
                with self.assertRaisesRegex(CommandError, "cancelled during worker"):
                    auto.run_analysis(payload, renderer, cancel)
                self.assertEqual(launch.call_count, 1)
                self.assertIsNotNone(processes[0].poll())

    def test_artifact_exports_yield_are_atomic_and_reused_after_cleanup(self):
        reactor = FakeReactor()
        renderer = PNGRenderer(sys.executable, reactor=reactor)
        workflow = types.SimpleNamespace(renderer=renderer, session=None)
        controller = auto.AutoNotchCalibration(workflow)
        item = {"record": {"iteration": 1}, "capture": {"samples": [(i / 1000., .1, .2, 1.) for i in range(20000)]}}
        session = {"run_id": "synthetic", "axis": "x", "save_config": False,
                   "auto_notch_phase": "raw_reference", "auto_notch_homes": [], "auto_notch_fresh": [],
                   "beta_motion_iterations": [item], "validation_records": [], "records": [],
                   "auto_notch_record_offset": 0, "auto_notch_original_profile": dict(DISABLED)}
        with tempfile.TemporaryDirectory() as directory:
            controller.folder = Path(directory)
            with mock.patch.object(reactor, "pause", wraps=reactor.pause) as pause:
                controller._write_report(session, {"result": "aborted"})
                self.assertGreater(pause.call_count, 10)
            raw = controller.folder / "motion-01.json.gz"
            before = raw.read_bytes()
            with mock.patch.object(auto.gzip, "open", side_effect=AssertionError("cached capture was rewritten")):
                controller._write_report(session, {"result": "aborted", "reason": "updated outcome"})
            self.assertEqual(raw.read_bytes(), before)
            controller.folder = Path(directory) / "cancelled"
            controller.folder.mkdir()
            controller.exported_sources.clear()
            workflow.session = session
            checks = 0
            def check(active):
                nonlocal checks
                checks += 1
                if checks > 5:
                    raise CommandError("cancelled during export")
            controller.check = check
            with self.assertRaisesRegex(CommandError, "cancelled during export"):
                controller._write_report(session, {"result": "pending"})
            self.assertEqual(list(controller.folder.iterdir()), [])
            self.assertEqual(controller.exported_sources, {})


if __name__ == "__main__":
    unittest.main()
