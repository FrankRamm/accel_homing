"""AUTO lifecycle, exact threshold, transactional persistence, and worker isolation."""
import contextlib
import copy
import importlib
import json
import gzip
import zipfile
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
from test_calibration_config import NativeShape, ModernNativeShape
import configparser
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
        record.update(signal_filter=dict(metadata), impact_metric="mapped_axis")
        capture["detector"].update(signal_filter=dict(metadata), impact_metric="mapped_axis")
        capture["sensor_context"] = self.sensor_context()
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

    def analyze(self, payload, renderer, check, **options):
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
                def analyze(payload, renderer, check, **options):
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
        config = config_class(root / "printer.cfg")
        config.cmd_SAVE_CONFIG = lambda command: (config.save(command), rig.gcode.request_restart(command))
        def read_config():
            parser = configparser.RawConfigParser()
            parser.read(root / "homing.cfg")
            regular, generated = config._find_autosave_data((root / "printer.cfg").read_text())
            parser.read_string(generated)
            return types.SimpleNamespace(fileconfig=parser)
        config.read_main_config = read_config
        rig.gcode.ready_gcode_handlers["SAVE_CONFIG"] = lambda command: config.cmd_SAVE_CONFIG(command)
        rig.configfile = rig.objects["configfile"] = config
        return config

    def test_native_save_round_trip_and_failure_restore_files_and_pending_for_both_forks(self):
        for config_class in (NativeShape, ModernNativeShape):
            for failure in (None, "native", "render", "pre_save", "final_report"):
                with self.subTest(fork=config_class.__name__, failure=failure), tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    rig = AutoRig(directory)
                    config = self.native_config(rig, root, config_class)
                    before = {p: p.read_bytes() for p in root.glob("*.cfg")}
                    owner = config.state
                    original_snapshot, original_report = auto.AutoNotchCalibration._save_snapshot, auto.AutoNotchCalibration._write_report
                    snapshots = 0
                    def snapshot(controller):
                        nonlocal snapshots
                        snapshots += 1
                        if snapshots == 2 and failure == "pre_save":
                            raise CommandError("pre-save snapshot failure")
                        return original_snapshot(controller)
                    def report(controller, session, result):
                        if failure == "final_report" and result.get("persistence") == "native_save_requested":
                            raise OSError("final report refresh failure")
                        return original_report(controller, session, result)
                    def render(payload):
                        if failure == "render" and payload.get("kind") == "notch_auto":
                            raise OSError("comparison render failure")
                    rig.workflow.renderer.on_render = render
                    with rig.helpers(), contextlib.ExitStack() as stack:
                        stack.enter_context(mock.patch.object(auto.AutoNotchCalibration, "_save_snapshot", snapshot))
                        stack.enter_context(mock.patch.object(auto.AutoNotchCalibration, "_write_report", report))
                        if failure == "native":
                            stack.enter_context(mock.patch.object(config, "cmd_SAVE_CONFIG", side_effect=CommandError("native save failure")))
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
                            self.assertNotIn("artifact_report_error", rig.workflow.last_result)
                            self.assertEqual(rig.workflow.last_result["artifacts"]["archive_snapshot"], "before_native_save")
                            self.assertTrue((Path(rig.workflow.last_result["artifacts"]["directory"]) / "completion.json").is_file())

    def test_pending_unrelated_values_reject_save_before_mapping_or_motion(self):
        for config_class in (NativeShape, ModernNativeShape):
            with self.subTest(fork=config_class.__name__), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory)
                config = self.native_config(rig, Path(directory), config_class)
                config.set("extruder", "pid_kp", "42")
                before = copy.deepcopy(config.state.status_save_pending)
                with rig.helpers(), self.assertRaisesRegex(CommandError, "unrelated pending"):
                    rig.command(SAVE_CONFIG=1)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(config.state.status_save_pending, before)

    def test_saved_native_threshold_model_pair_can_be_replaced_by_auto(self):
        from test_workflows import Config, AccelHomingWorkflows
        for config_class in (NativeShape, ModernNativeShape):
            with self.subTest(fork=config_class.__name__), tempfile.TemporaryDirectory() as directory, \
                    mock.patch.object(workflow_module, "configparser", configparser):
                rig = AutoRig(directory)
                config = self.native_config(rig, Path(directory), config_class)
                with rig.helpers():
                    rig.command(SAVE_CONFIG=1)
                loaded = config.read_main_config().fileconfig
                sections = {"printer": {"kinematics": "corexy"},
                            "accel_homing_model default": dict(loaded.items("accel_homing_model default"))}
                runtime = dict(loaded.items("accel_homing"))
                runtime["charts_directory"] = directory
                rig.config = Config(rig, runtime, sections=sections)
                config.state.status_save_pending, config.state.save_config_pending = {}, False
                renderer = rig.workflow.renderer
                with mock.patch.object(rig.gcode, "register_command"):
                    rig.workflow = AccelHomingWorkflows(rig.owner, rig.config)
                rig.workflow.renderer = renderer
                rig.workflow._calibration_axis_mapping = rig.qualify_axis_mapping
                rig.owner.workflows = rig.workflow
                self.assertNotIn("x", rig.workflow.manual_axes)
                self.assertEqual(rig.owner.threshold_g["x"], 1.25)
                rig.workflow._disable_xy()
                rig.toolhead.position[:2] = [150., 100.]
                rig.idle_state = "Ready"
                with rig.helpers():
                    rig.command(SAVE_CONFIG=1)
                self.assertEqual(rig.workflow.last_result["persistence"], "native_save_requested")
                self.assertEqual(len(rig.home_calls), 44)

    def test_auto_qualifies_a_new_map_and_rejects_map_change_during_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory)
            qualify = rig.workflow._calibration_axis_mapping
            calls = []
            def fresh_mapping(session):
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.beta_events, [])
                calls.append(1)
                qualify(session)
            rig.workflow._calibration_axis_mapping = fresh_mapping
            original = rig.analyze
            def changed_mapping(payload, renderer, check, **options):
                result = original(payload, renderer, check)
                if not payload.get("preflight"):
                    rig.workflow.session["axis_mapping"]["sign"] = -1
                return result
            with rig.helpers(), mock.patch.object(auto, "run_analysis", side_effect=changed_mapping):
                with self.assertRaisesRegex(CommandError, "mapped detector"):
                    rig.command()
            self.assertEqual(calls, [1])
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(rig.owner.notch_profiles["x"], profile(150., 5.))

    def test_old_links_load_auto_companions_and_worker_uses_private_numpy(self):
        fixture = upgrade_tests.TroubleshootImportTests()
        for entrypoint in ("accel_homing", "beacon_accel_homing"):
            with self.subTest(entrypoint=entrypoint), fixture.installation() as (checkout, extras):
                for name in ("accel_homing_auto.py", "accel_homing_auto_notch.py", "accel_homing_config.py"):
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
            result = auto.run_analysis({"preflight": True}, renderer, lambda: None,
                                       debug_directory=Path(directory) / "worker-debug")
            self.assertEqual(result["numpy_version"], "private-only-sentinel")
            receipt = json.loads((Path(directory) / "worker-debug/worker.json").read_text())
            self.assertEqual(receipt["returncode"], 0)
            (Path(directory) / "numpy.py").write_text("raise RuntimeError('dependency exploded')\n")
            with self.assertRaisesRegex(ValueError, "dependency exploded"):
                auto.run_analysis({"preflight": True}, renderer, lambda: None,
                                  debug_directory=Path(directory) / "worker-failed")
            self.assertIn("dependency exploded", (Path(directory) / "worker-failed/worker.log").read_text())
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

    def test_debug_archive_contains_all_stopped_acquisitions_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory)
            with rig.helpers():
                rig.command()
            archive = Path(rig.workflow.last_result["artifacts"]["archive"])
            with zipfile.ZipFile(archive) as bundle:
                names = set(bundle.namelist())
                self.assertTrue({"run.json", "report.json", "debug/context.json"} <= names)
                self.assertEqual(len([name for name in names if name.startswith("raw/home-")]), 22)
                self.assertEqual(len([name for name in names if name.startswith("raw/motion-")]), 3)
                self.assertEqual(len([name for name in names if name.startswith("raw/stationary-")]), 3)
                self.assertTrue(any(name.endswith(".png") for name in names))
                first = json.loads(gzip.decompress(bundle.read("raw/home-0001.json.gz")))
                self.assertEqual(len(first["capture"]["samples"]), 4)
                self.assertEqual(first["phase"], "raw_reference_homes")
                receipt = json.loads(bundle.read("run.json"))
                self.assertIsNone(receipt["exception"])
                self.assertEqual(receipt["phase"], "complete")
            self.assertTrue(any("AUTO diagnostic directory:" in message for message in rig.output))
            self.assertTrue(any("AUTO diagnostic ZIP:" in message for message in rig.output))

    def test_debug_archive_survives_preflight_cancel_and_failed_final_home(self):
        for failure in ("preflight", "cancel", "home", "mapping"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory)
                log = Path(directory) / "klippy.log"
                log.write_text("before command\n")
                rig.get_start_args = lambda: {"log_file": str(log)}
                options = {"SAVE_CONFIG": 1} if failure == "preflight" else {}
                if failure == "preflight":
                    rig.workflow.manual_axes.add("x")
                elif failure == "home":
                    rig.home_results.extend([{}] * 11 + [{"travel_mm": 2.}])
                elif failure == "mapping":
                    def mapping_failure(session):
                        session["auto_notch_controller"].retain_debug_capture(session, "mapping", 1,
                            {"samples": [(0., 0., 0., 1.), (.001, .1, 0., 1.)]}, record={"direction": 1})
                        raise CommandError("mapping reference failed")
                    rig.workflow._calibration_axis_mapping = mapping_failure
                original = rig.analyze
                def analyze(payload, renderer, check, **kwargs):
                    if failure == "cancel" and not payload.get("preflight"):
                        with log.open("a") as stream:
                            stream.write("cancelled during selection\n")
                        rig.workflow.cmd_CANCEL(GCmd())
                        check()
                    return original(payload, renderer, check, **kwargs)
                original_pause = rig.reactor.pause
                def pause(when):
                    # Moonraker status polling must be safe even for a synthetic
                    # reservation created after an early preflight rejection.
                    rig.workflow.get_status(rig.reactor.monotonic())
                    return original_pause(when)
                with rig.helpers(), mock.patch.object(auto, "run_analysis", side_effect=analyze), \
                        mock.patch.object(rig.reactor, "pause", side_effect=pause), self.assertRaises(CommandError):
                    rig.command(**options)
                result = rig.workflow.last_result
                self.assertIsNone(rig.workflow.session)
                with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                    receipt = json.loads(bundle.read("run.json"))
                    self.assertIsNotNone(receipt["exception"])
                    self.assertIn("Traceback", receipt["exception"]["traceback"])
                    self.assertLessEqual(len(receipt["exception"]["traceback"]), 32768)
                    if failure == "home":
                        self.assertIn("raw/home-0012.json.gz", bundle.namelist())
                        self.assertEqual(receipt["exception"]["phase"], "fixed_profile_homes")
                    elif failure == "mapping":
                        self.assertIn("raw/mapping-0001.json.gz", bundle.namelist())
                    elif failure == "preflight":
                        self.assertEqual(receipt["phase"], "preflight")
                        self.assertEqual(rig.home_calls, [])
                    else:
                        self.assertIn("raw/home-0011.json.gz", bundle.namelist())
                        self.assertIn(b"cancelled during selection", bundle.read("debug/klippy.log"))
                        self.assertNotIn(b"before command", bundle.read("debug/klippy.log"))

    def test_debug_preflight_refuses_overlap_or_printing_without_io(self):
        for condition in ("workflow", "printing", "homing", "queue"):
            with self.subTest(condition=condition), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory)
                before = rig.workflow.last_result = {"previous": True}
                if condition == "workflow":
                    rig.workflow.session = {"mode": "other", "state": "running"}
                elif condition == "printing":
                    rig.print_state = "printing"
                elif condition == "homing":
                    rig.owner.homing_active = True
                else:
                    rig.toolhead.busy = (20., 10., False)
                with mock.patch.object(Path, "mkdir", side_effect=AssertionError("unexpected disk work")), \
                        self.assertRaisesRegex(CommandError, "no AUTO run was started"):
                    rig.command()
                self.assertIs(rig.workflow.last_result, before)
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_raw_debug_serialization_cooperates_and_stops_when_motion_starts(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory)
            controller = auto.AutoNotchCalibration(rig.workflow)
            controller.folder = Path(directory)
            controller.debug_phase = "synthetic"
            session = {"run_id": "debug", "axis": "x"}
            capture = {"samples": [(i / 1000., .1, .2, 1.) for i in range(20000)]}
            with mock.patch.object(rig.reactor, "pause", wraps=rig.reactor.pause) as pause:
                controller.retain_debug_capture(session, "home", 1, capture)
                self.assertGreater(pause.call_count, 20)
            with gzip.open(Path(directory) / "raw/home-0001.json.gz", "rt") as stream:
                self.assertEqual(len(json.load(stream)["capture"]["samples"]), 20000)
            def start_print(when):
                rig.print_state = "printing"
            with mock.patch.object(rig.reactor, "pause", side_effect=start_print):
                controller.retain_debug_capture(session, "home", 2, capture)
            self.assertFalse((Path(directory) / "raw/home-0002.json.gz").exists())
            self.assertTrue(any("another workflow is active" in error for error in controller.debug_errors))
            self.assertFalse(list((Path(directory) / "raw").glob("*.tmp")))

    def test_zip_failure_blocks_save_and_failed_refresh_keeps_previous_archive(self):
        diagnostics = auto.load_companion_module("accel_homing_troubleshoot")
        for scenario in ("before_save", "after_failed_save"):
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory)
                config = self.native_config(rig, Path(directory), NativeShape)
                before = {p: p.read_bytes() for p in Path(directory).glob("*.cfg")}
                original = diagnostics.write_run_archive
                saved_archive = []
                def archive(folder, target, **kwargs):
                    if scenario == "before_save" or saved_archive:
                        raise OSError("injected ZIP failure")
                    result = original(folder, target, **kwargs)
                    saved_archive.append(Path(target).read_bytes())
                    return result
                with rig.helpers(), mock.patch.object(diagnostics, "write_run_archive", side_effect=archive), \
                        mock.patch.object(config, "cmd_SAVE_CONFIG", side_effect=CommandError("native failed")), \
                        self.assertRaises(CommandError):
                    rig.command(SAVE_CONFIG=1)
                self.assertEqual({p: p.read_bytes() for p in Path(directory).glob("*.cfg")}, before)
                rig.gcode.request_restart.assert_not_called()
                result = rig.workflow.last_result
                if scenario == "before_save":
                    self.assertIn("diagnostic ZIP", result["reason"])
                    self.assertNotIn("archive", result["artifacts"])
                else:
                    target = Path(result["artifacts"]["archive"])
                    self.assertEqual(target.read_bytes(), saved_archive[0])
                    self.assertEqual(result["artifacts"]["archive_snapshot"], "previous_completed_snapshot")
                    with zipfile.ZipFile(target) as bundle:
                        self.assertIsNone(bundle.testzip())

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
