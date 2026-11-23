"""Gentle pre-contact selection and exact-profile qualification, offline only."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock
import zipfile

import accel_homing_auto as auto
import accel_homing_auto_notch as analysis
from accel_homing_filter import DISABLED, replay
from harness import CommandError
import test_auto_notch_workflow as auto_tests
from test_auto_notch_workflow import AutoRig
from test_calibration_config import NativeShape, ModernNativeShape
from test_workflows import GCmd, workflow_module


class GentleAutoRig(AutoRig):
    """Use real offline motion selection with synthetic independent captures."""
    def __init__(self, directory, options=None):
        super().__init__(directory, {"gentle_homing": True, **(options or {})})
        self.owner.homing_accel = 1000.
        for rail in self.owner.rails_by_axis.values():
            rail.info.speed = 40.
        self.motion_mutator = None
        self.selection_mutator = None
        self.home_profiles = []
        self.observe_seek = True
        self.native_plan_mutator = None
        analysis.preflight()  # Preserve dependency imports across isolated module mocks.

    def stop_motion(self, token, check):
        capture = super().stop_motion(token, check)
        session = self.workflow.session
        iteration = session["beta_current_motion"]["iteration"]
        source = session["axis_mapping"]["source_axis"]
        capture.update(baseline_sample_count=len(token["baseline_samples"]),
                       baseline_first_sample_time_s=token["baseline_samples"][0][0],
                       baseline_last_sample_time_s=token["baseline_samples"][-1][0])
        samples = []
        for row in capture["samples"]:
            values = [0., 0., 1.]
            values[source] += (.8 + .002 * iteration) * math.sin(2 * math.pi * 100. * row[0] + .01 * iteration)
            values[source] += .025 * math.sin(2 * math.pi * 31. * row[0])
            samples.append((row[0], *values))
        capture["samples"] = samples
        if self.motion_mutator:
            self.motion_mutator(capture, iteration)
        return capture

    def analyze(self, payload, renderer, check, **options):
        check()
        if payload.get("preflight"):
            return analysis.preflight()
        if not payload.get("gentle_candidate") or "homes" in payload or self.home_calls:
            raise AssertionError("gentle candidate must be chosen before any contact, from motion only")
        if len(payload["motions"]) != 3:
            raise AssertionError("gentle selection requires exactly three controls")
        if self.owner.filter_settings(self.workflow.session["axis"]) != DISABLED:
            raise AssertionError("motion inputs must precede candidate installation")
        self.analysis_seen.append(copy.deepcopy(payload))
        result = analysis.analyze_gentle(**{key: value for key, value in payload.items()
                                          if key != "gentle_candidate"}, check_running=check)
        if self.selection_mutator:
            self.selection_mutator(result)
        return result

    def stock_home(self, gcmd):
        session = self.workflow.session
        self.home_profiles.append({"profile": self.owner.filter_settings(session["axis"]),
                                   "candidate": copy.deepcopy(session["gentle_frozen_candidate"]),
                                   "phase": session["auto_notch_phase"]})
        result = super().stock_home(gcmd)
        axis = session["axis"]
        record = self.owner.last[axis]
        info = self.owner.rails_by_axis[axis].get_homing_info()
        speed, accel = info.speed, self.owner.homing_accel
        ramp = speed / accel
        distance = self.workflow._geometry(axis)[1] + 2.
        cruise = distance / speed - ramp
        arm = record["arm_time_s"]
        # A synthetic150mm stop during the152mm bounded seek's cruise.
        onset = arm + record["travel_mm"] / speed + ramp / 2.
        record["onset_time_s"] = onset
        direction = 1. if info.positive_dir else -1.
        self.profile = []
        for offset, duration, velocity, acceleration in ((0., ramp, 0., accel),
                (ramp, cruise, speed, 0.), (ramp + cruise, ramp, speed, -accel)):
            vector = [0., 0., 0.]
            vector["xy".index(axis)] = direction
            self.profile.append(types.SimpleNamespace(print_time=arm + offset, move_t=duration,
                start_v=velocity, accel=acceleration, x_r=vector[0], y_r=vector[1], z_r=vector[2]))
        attempt = types.SimpleNamespace(axis=axis, pass_no=1, arm_time=arm,
                                        request_time=onset + .001, reason="impact")
        home_end = onset + .01
        if self.native_plan_mutator:
            home_end = self.native_plan_mutator(attempt, home_end, self.profile, record)
        observer = getattr(self.owner, "contact_timing_observer", None)
        if self.observe_seek and observer is not None:
            observer(attempt, home_end)
        return result

    def stop_capture(self):
        capture = super().stop_capture()
        record = capture["_record"]
        axis = record["axis"]
        start = self.sensor_time
        onset_index = round((record["onset_time_s"] - start) * 1000.)
        source = self.workflow.session["axis_mapping"]["source_axis"]
        rows = []
        for index in range(onset_index + 31):
            values = [0., 0., 1.]
            values[source] += 4. if index == onset_index else .01
            rows.append((start + index / 1000., *values))
        identity = (round(start * 1000), len(rows) - 1, onset_index, len(rows))
        record.update(onset_sample=identity, onset_time_s=rows[onset_index][0])
        capture["samples"] = rows
        capture["auto_notch_replay"]["eligibility_source"] = "full_precision_source_time"
        capture["detector"].update(onset_sample=identity, onset_sample_index=onset_index,
            onset_time_s=rows[onset_index][0], trigger_onset_time_s=record["onset_time_s"])
        if capture.get("signal_filter", DISABLED) != DISABLED:
            capture["filtered_samples"] = replay(rows, capture["baseline_g"], capture["signal_filter"], 1)
        return capture

    def command(self, axis="X", bare=False, **options):
        package = workflow_module.__package__
        parent = types.ModuleType(package)
        parent.__path__ = [str(Path(auto.__file__).parent)]
        import accel_homing_backends
        with mock.patch.dict(sys.modules, {package: parent,
                package + ".accel_homing_backends": accel_homing_backends}):
            params = {"AXIS": axis, "CENTERED": 1, **options}
            if not bare:
                params["NOTCH"] = "AUTO"
            self.workflow.cmd_CALIBRATE(GCmd(**params))


class GentleAutoWorkflowTests(unittest.TestCase):
    def test_full_run_qualifies_first_frozen_profile_without_raw_reference_homes(self):
        for axis in "XY":
            with self.subTest(axis=axis), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory)
                original = copy.deepcopy(rig.owner.notch_profiles)
                with rig.helpers():
                    rig.command(axis=axis)
                result = rig.workflow.last_result
                self.assertEqual(len(rig.analysis_seen), 1)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertTrue(all(item["phase"] == "qualification" for item in rig.home_profiles))
                candidate = rig.home_profiles[0]["candidate"]
                self.assertNotEqual(candidate["profile"], DISABLED)
                self.assertTrue(all(item["profile"] == candidate["profile"] for item in rig.home_profiles))
                self.assertEqual([item["threshold"] for item in rig.home_calls], [candidate["threshold_g"]] * 11)
                self.assertEqual(len([event for event in rig.beta_events if event[0] == "central_move"]), 6)
                self.assertEqual(rig.owner.notch_profiles, original)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(result["auto_notch"]["reference_homes"], 0)
                self.assertEqual(result["auto_notch"]["fresh_homes"], 11)
                self.assertEqual(result["restarts"], 0)
                with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                    report = json.loads(bundle.read("report.json"))
                saved = report["proposed_model"]
                self.assertTrue(saved["gentle_homing"])
                self.assertTrue(saved["auto_notch_selection"]["candidate_frozen_before_contact"])
                self.assertEqual(saved["auto_notch_selection"]["raw_reference_homes"], 0)
                self.assertEqual(saved["signal_filter"], candidate["profile"])
                self.assertEqual(saved["saved_threshold_g"], candidate["threshold_g"])
                self.assertEqual(saved["tested_threshold_g"], candidate["threshold_g"])
                self.assertEqual(saved["threshold_margin_g"], 0.)

    def test_rejected_or_incomplete_motion_evidence_never_starts_contact(self):
        for fault in ("holdout", "missing", "worker_rejection", "threshold"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory)
                if fault in ("holdout", "missing"):
                    def mutate(capture, iteration):
                        if iteration != 3:
                            return
                        if fault == "missing":
                            capture["dropped"] = 1
                        else:
                            capture["samples"] = [(row[0], row[1] * 20., row[2], row[3])
                                                  for row in capture["samples"]]
                    rig.motion_mutator = mutate
                else:
                    rig.selection_mutator = (lambda report: report.update(status="rejected", reasons=["holdout failed"])) \
                        if fault == "worker_rejection" else (lambda report: report.update(threshold_g=.5))
                original = copy.deepcopy(rig.owner.notch_profiles)
                with rig.helpers(), self.assertRaises(CommandError):
                    rig.command()
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.owner.notch_profiles, original)
                self.assertEqual(rig.configfile.saved, {})
                self.assertIsNone(rig.workflow.session)
                self.assertIn("archive", rig.workflow.last_result["artifacts"])

    def test_early_contact_and_overtravel_stop_after_one_frozen_attempt(self):
        for record in ({"travel_mm": 1.}, {"travel_mm": 80.},
                       {"trigger_travel_mm_signed": -150., "halt_travel_mm_signed": -150.5001}):
            with self.subTest(record=record), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory)
                rig.home_results.append(record)
                original = copy.deepcopy(rig.owner.notch_profiles)
                with rig.helpers(), self.assertRaises(CommandError):
                    rig.command()
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.workflow.last_result["restarts"], 0)
                self.assertEqual(rig.owner.notch_profiles, original)
                self.assertEqual(rig.configfile.saved, {})
                self.assertIn("archive", rig.workflow.last_result["artifacts"])

    def test_native_save_replaces_selected_threshold_and_preserves_shared_peer(self):
        for native in (NativeShape, ModernNativeShape):
            with self.subTest(native=native.__name__), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory, {"threshold": 8.})
                root = Path(directory)
                config = auto_tests.AutoNotchWorkflowTests().native_config(rig, root, native)
                with (root / "homing.cfg").open("a") as stream:
                    stream.write("threshold: 8\nx_threshold: 7\n")
                with rig.helpers():
                    rig.command(SAVE_CONFIG=1)
                loaded = config.read_main_config().fileconfig
                candidate = rig.home_profiles[0]["candidate"]
                self.assertEqual(loaded.getfloat("accel_homing", "threshold"), 8.)
                self.assertEqual(loaded.getfloat("accel_homing", "x_threshold"), candidate["threshold_g"])
                self.assertEqual(loaded.getfloat("accel_homing", "x_notch_frequency"), candidate["profile"]["frequency_hz"])
                saved = json.loads(loaded.get("accel_homing_model default", "x_metadata"))
                self.assertEqual(saved["signal_filter"], candidate["profile"])
                self.assertEqual(saved["saved_threshold_g"], candidate["threshold_g"])
                self.assertTrue(rig.workflow.last_result["save_config"])

    def test_bare_gentle_preserves_default_save_and_explicit_stage_only(self):
        for save in (None, 0):
            with self.subTest(save=save), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory)
                root = Path(directory)
                config = auto_tests.AutoNotchWorkflowTests().native_config(rig, root, NativeShape)
                before = {path.name: path.read_bytes() for path in root.glob("*.cfg")}
                with rig.helpers():
                    rig.command(bare=True, **({} if save is None else {"SAVE_CONFIG": save}))
                result = rig.workflow.last_result
                self.assertEqual(len(rig.home_calls), 11)
                self.assertTrue(result["model_staged"])
                self.assertEqual(result["save_config"], save is None)
                self.assertEqual(result["persistence"], "native_save_requested" if save is None else "staged_not_saved")
                if save == 0:
                    self.assertEqual({path.name: path.read_bytes() for path in root.glob("*.cfg")}, before)
                    self.assertEqual(float(config.state.status_save_pending["accel_homing"]["x_threshold"]),
                                     rig.home_calls[0]["threshold"])
                    self.assertTrue(rig.workflow.calibration_config_save._targets)

    def test_changed_frozen_detector_is_rejected_before_a_second_home(self):
        for field in ("threshold", "profile", "mapping", "sensor"):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory)
                recenter = rig.workflow._recenter

                def changed(axis):
                    recenter(axis)
                    session = rig.workflow.session
                    if field == "threshold": session["candidate"] += .25
                    elif field == "profile": rig.owner.notch_profiles[axis] = dict(DISABLED)
                    elif field == "mapping": session["axis_mapping"]["sign"] *= -1
                    else: rig.owner.sensor_name = "changed"

                with rig.helpers(), mock.patch.object(rig.workflow, "_recenter", side_effect=changed), \
                        self.assertRaises(CommandError):
                    rig.command()
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.workflow.last_result["attempts"], 1)
                self.assertEqual(rig.configfile.saved, {})

    def test_staging_failure_rolls_back_pending_settings_and_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleAutoRig(directory)
            root = Path(directory)
            config = auto_tests.AutoNotchWorkflowTests().native_config(rig, root, NativeShape)
            config.set("extruder", "pid_kp", "42")
            pending = copy.deepcopy(config.state.status_save_pending)
            before = {path.name: path.read_bytes() for path in root.glob("*.cfg")}
            stage = rig.workflow._stage_model_settings

            def fail_after_staging(*args):
                stage(*args)
                raise CommandError("synthetic staging failure")

            with rig.helpers(), mock.patch.object(rig.workflow, "_stage_model_settings", side_effect=fail_after_staging), \
                    self.assertRaisesRegex(CommandError, "staging failure"):
                rig.command(bare=True, SAVE_CONFIG=0)
            self.assertEqual(config.state.status_save_pending, pending)
            self.assertEqual({path.name: path.read_bytes() for path in root.glob("*.cfg")}, before)
            self.assertEqual(rig.workflow.calibration_config_save._targets, {})
            self.assertFalse(rig.workflow.last_result["model_staged"])
            self.assertIn("archive", rig.workflow.last_result["artifacts"])

    def test_isolated_worker_routes_gentle_motion_only_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleAutoRig(directory)
            with rig.helpers():
                rig.command(CONTACT_ONLY=1)
            worker = types.SimpleNamespace(python_executable=sys.executable, dependency_directory="", reactor=None)
            result = auto.run_analysis(rig.analysis_seen[0], worker, lambda: None)
            self.assertEqual(result["status"], "selected")
            self.assertEqual(result["profile"], rig.home_profiles[0]["candidate"]["profile"])
            self.assertEqual(result["threshold_g"], rig.home_calls[0]["threshold"])


if __name__ == "__main__":
    unittest.main()
