"""Automatic paired 5+5 calibration and command-local HYBRID defaults."""
import configparser
import copy
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

from accel_homing_model import AccelHomingModel
from harness import CommandError, ConfigError
from test_filter_engine import FilteredRig
from test_workflows import Config, GCmd
import test_hybrid_position as hybrid_tests
import accel_homing_charts as charts


class NativePendingConfig:
    def __init__(self, parser_factory=configparser.RawConfigParser):
        self.autosave = types.SimpleNamespace(fileconfig=parser_factory())
        self.status_save_pending = {}
        self.save_config_pending = False
        self.on_set = None
        self.calls = 0

    @property
    def saved(self):
        parser = self.autosave.fileconfig
        return {(section, key): value for section in parser.sections() for key, value in parser.items(section)}

    def set(self, section, option, value):
        self.calls += 1
        if self.on_set:
            self.on_set(section, option, value)
        parser = self.autosave.fileconfig
        if not parser.has_section(section):
            parser.add_section(section)
        parser.set(section, option, str(value))
        self.status_save_pending.setdefault(section, {})[option] = str(value)
        self.save_config_pending = True


class PairRig(hybrid_tests.HybridWorkflowRig):
    def __init__(self, options=None):
        super().__init__(options)
        # test_workflows imports its relative workflow module under a temporary
        # sys.modules patch; use that module's own stdlib class identity.
        parser_factory = self.workflow._paired_config_snapshot.__globals__["configparser"].RawConfigParser
        self.configfile = NativePendingConfig(parser_factory)
        self.objects["configfile"] = self.configfile

    def run_pair(self, **params):
        arguments = dict(AXIS="X", CENTERED=1, SAVE_CONFIG=0)
        arguments.update(params)
        self.workflow.cmd_CALIBRATE(GCmd(**arguments))

    def stop_capture(self):
        if (self.workflow.session or {}).get("hybrid_raw_profiles"):
            return super().stop_capture()
        return FilteredRig.stop_capture(self)

    def reprepare(self, axis="x"):
        self.workflow._disable_xy()
        self.idle_state = "Ready"
        self.toolhead.position["xy".index(axis)] = self.workflow._geometry(axis)[0]
        self.capture_stops = 0  # Each native fixture owns a fresh synthetic clock epoch.

    def reload_model(self):
        options = {key: value for (section, key), value in self.configfile.saved.items()
                   if section == self.workflow.model_section}
        return AccelHomingModel(Config(None, options, self.workflow.model_section))


class HybridDefaultTests(unittest.TestCase):
    def test_untagged_schema_two_xyz_and_rectified_profiles_supply_default_accuracy_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            hybrid_tests.install_profiles(rig)
            metadata = copy.deepcopy(rig.workflow.model.metadata["x"])
            self.assertNotIn("native_polarity_policy", metadata)
            self.assertNotIn("native_polarity_policy", metadata["detector_profiles"]["raw"]["metadata"])
            rig.workflow.model = AccelHomingModel(Config(None, {"version": 2, "x_threshold": 2.25,
                "x_successes": 10, "x_metadata": json.dumps(metadata)}, rig.workflow.model_section))
            original = copy.deepcopy(rig.workflow.model.metadata)
            rig.workflow.cmd_ACCURACY(GCmd(AXIS="X", CENTERED=1, SAMPLES=2))
            self.assertEqual(rig.workflow.last_result["result"], "complete")
            trial = rig.workflow.last_result["axis_results"]["x"]["trials"][0]
            self.assertEqual(trial["position_selection"]["raw_threshold_g"], 4.25)
            self.assertEqual(trial["threshold_g"], 2.25)
            self.assertEqual(rig.workflow.model.metadata, original)
            self.assertEqual(rig.configfile.saved, {})

    def test_prior_filtered_selection_and_pair_progress_resume_remain_truthful(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            hybrid_tests.install_profiles(rig)
            progress = []
            rig.on_home = lambda _axis: progress.append(copy.deepcopy(rig.workflow.get_status(0)["active"]))
            rig.run_pair()
            self.assertEqual(rig.reload_model().metadata["x"]["detector_type"], "filtered")
            self.assertEqual(progress[0]["qualification_counts"], {"raw": 0, "filtered": 0})
            self.assertEqual(progress[6]["actual_series_detector"], "filtered")
            self.assertEqual(progress[6]["qualification_counts"], {"raw": 5, "filtered": 0})
            self.assertEqual(progress[6]["required_checks_per_detector"], 5)
        rig = PairRig()
        rig.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x", default_detector="hybrid")
        rig.workflow.session["state"] = "waiting"
        with mock.patch.object(rig.workflow, "_continue_calibration") as continuation:
            rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, DETECTOR="HYBRID"))
            continuation.assert_called_once()
            with self.assertRaisesRegex(CommandError, "DETECTOR cannot change"):
                rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, DETECTOR="RAW"))

    def test_omitted_and_explicit_hybrid_calibrate_fresh_pair_with_honest_reload(self):
        for params in ({}, {"DETECTOR": "HYBRID"}):
            with tempfile.TemporaryDirectory() as directory:
                rig = PairRig({"charts_directory": directory})
                old = dict(rig.owner.threshold_g)
                rig.run_pair(**params)
                self.assertEqual(len(rig.home_calls), 12)
                self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
                model = rig.reload_model()
                self.assertEqual(model.version, 3)
                self.assertEqual(model.successes["x"], 5)
                self.assertEqual(model.metadata["x"]["detector_type"], "raw")
                profiles = model.metadata["x"]["detector_profiles"]
                for kind in ("raw", "filtered"):
                    self.assertEqual(profiles[kind]["metadata"]["qualification_successes"], 5)
                    self.assertEqual(len(profiles[kind]["metadata"]["onset_g"]), 5)
                    self.assertEqual(profiles[kind]["threshold_g"], 1.5)
                self.assertEqual(rig.owner.threshold_g, old)
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertEqual(rig.workflow.last_result["qualification_counts"], {"raw": 5, "filtered": 5})

    def test_default_accuracy_requires_pair_and_explicit_overrides_keep_old_semantics(self):
        rig = PairRig()
        with self.assertRaisesRegex(CommandError, "compatible saved RAW XYZ and FILTERED"):
            rig.workflow.cmd_ACCURACY(GCmd(AXIS="X", CENTERED=1, SAMPLES=2))
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.noise_calls, [])
        for kind in ("RAW", "FILTERED"):
            with tempfile.TemporaryDirectory() as directory:
                rig = PairRig({"charts_directory": directory})
                rig.run_pair(DETECTOR=kind)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertEqual(rig.reload_model().successes["x"], 10)

    def test_new_pair_supplies_default_accuracy_and_fifty_home_receipt_with_bounded_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            rig.run_pair()
            rig.workflow.model = rig.reload_model()
            rig.reprepare()
            rig.workflow.cmd_ACCURACY(GCmd(AXIS="X", CENTERED=1, SAMPLES=2))
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            report = charts._accuracy_report_from_png(Path(result["summary_paths"]["x_png"]).read_bytes())
            self.assertEqual(report["context"]["requested_detector"], "hybrid")
            profile = report["context"]["raw_position_profile"]
            self.assertLess(len(json.dumps(profile)), 4096)
            self.assertNotIn("paired_calibrations", profile)
            report.update(count=50, requested=50, travels_mm=[175.] * 50,
                          position_selections=[copy.deepcopy(report["position_selections"][0]) for _ in range(50)])
            report["statistics"] = charts._accuracy_statistics([{"travel_mm": value} for value in report["travels_mm"]])
            report["hybrid_position_summary"] = charts._hybrid_position_summary(report["position_selections"])
            charts._validate_accuracy_report(report)
            self.assertLess(len(charts._json(report)), charts._MAX_HYBRID_REPORT_BYTES)

    def test_failure_in_either_series_discards_both_and_preserves_only_failing_candidate_progress(self):
        for failed_attempt, kind in ((6, "raw"), (12, "filtered")):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                rig = PairRig({"charts_directory": directory})
                rig.home_results.extend([{}] * (failed_attempt - 1) + [{"travel_mm": 8.}])
                rig.run_pair()
                self.assertEqual(len(rig.home_calls), failed_attempt + 12)
                model = rig.reload_model()
                profiles = model.metadata["x"]["detector_profiles"]
                self.assertEqual(profiles[kind]["threshold_g"], 1.75)
                self.assertEqual(profiles["filtered" if kind == "raw" else "raw"]["threshold_g"], 1.5)
                proof = profiles["raw"]["metadata"]["paired_qualification"]
                self.assertEqual(proof["discovery_attempt"], failed_attempt + 1)
                self.assertEqual(rig.workflow.last_result["paired_restarts"], 1)
                self.assertTrue(any("Discarding BOTH" in message for message in rig.output))

    def test_filtered_spread_failure_requalifies_raw_five_from_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            rig.home_results.extend([{}] * 8 + [{"travel_mm": 152.001}])
            rig.run_pair()
            self.assertEqual(len(rig.home_calls), 21)
            profiles = rig.reload_model().metadata["x"]["detector_profiles"]
            self.assertEqual(profiles["raw"]["metadata"]["paired_qualification"]["discovery_attempt"], 10)
            self.assertEqual(profiles["filtered"]["threshold_g"], 1.75)

    def test_budget_reserves_complete_pair_before_any_retry_return(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory, "calibration_max_attempts": 12})
            rig.home_results.extend([{}] * 11 + [{"travel_mm": 8.}])
            with self.assertRaisesRegex(CommandError, "12 remaining"):
                rig.run_pair()
            self.assertEqual(len(rig.home_calls), 12)
            self.assertEqual(len(rig.toolhead.moves), 11)
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_transition_context_change_aborts_without_pair_or_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            original = rig.workflow._recenter
            def recenter(axis):
                original(axis)
                if len(rig.home_calls) == 6:
                    rig.owner.homing_accel += 1.
            rig.workflow._recenter = recenter
            with self.assertRaisesRegex(CommandError, "context changed"):
                rig.run_pair()
            self.assertEqual(len(rig.home_calls), 6)
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(rig.workflow.staged_detector_profiles, {})
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_pending_staging_failure_cancel_and_save_failure_roll_back_whole_pair(self):
        for failure in ("set", "cancel_set", "save", "cancel_cleanup"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = PairRig({"charts_directory": directory})
                rig.configfile.set("unrelated", "value", "preserve")
                before = copy.deepcopy(rig.configfile.saved), copy.deepcopy(rig.configfile.status_save_pending)
                if failure in ("set", "cancel_set"):
                    def setting(section, option, value):
                        if option == "x_successes":
                            if failure == "set":
                                raise CommandError("staging failed")
                            rig.workflow.cmd_CANCEL(GCmd())
                    rig.configfile.on_set = setting
                elif failure == "save":
                    rig.save_error = True
                else:
                    ending = rig.owner._end_session
                    def end():
                        if (rig.workflow.session or {}).get("finishing"):
                            rig.workflow.cmd_CANCEL(GCmd())
                        ending()
                    rig.owner._end_session = end
                with self.assertRaises(CommandError):
                    rig.run_pair(SAVE_CONFIG=1 if failure == "save" else 0)
                self.assertEqual(rig.configfile.saved, before[0])
                self.assertEqual(rig.configfile.status_save_pending, before[1])
                self.assertEqual(rig.workflow.staged_detector_profiles, {})
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(list(Path(directory).iterdir()), [])

    def test_later_pure_calibration_retains_archived_five_certificate_and_schema_three(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            rig.run_pair()
            rig.workflow.model = rig.reload_model()
            rig.reprepare()
            rig.run_pair(DETECTOR="RAW")
            model = rig.reload_model()
            self.assertEqual(model.version, 3)
            self.assertEqual(model.successes["x"], 10)
            self.assertEqual(model.metadata["x"]["detector_profiles"]["filtered"]["metadata"]["qualification_successes"], 5)
            self.assertEqual(len(model.metadata["x"]["paired_calibrations"]), 1)
            options = {key: value for (section, key), value in rig.configfile.saved.items() if section == rig.workflow.model_section}
            metadata = json.loads(options["x_metadata"])
            del metadata["detector_profiles"]["filtered"]["metadata"]["paired_qualification"]
            options["x_metadata"] = json.dumps(metadata)
            with self.assertRaisesRegex(ConfigError, "five-check"):
                AccelHomingModel(Config(None, options, rig.workflow.model_section))

    def test_pending_pair_on_x_cannot_be_downgraded_by_later_pure_y_save(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            rig.run_pair()
            rig.reprepare("y")
            rig.run_pair(AXIS="Y", DETECTOR="RAW")
            model = rig.reload_model()
            self.assertEqual(model.version, 3)
            self.assertEqual(model.successes, {"x": 5, "y": 10})

    def test_model_rejects_partial_mixed_or_altered_pair_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = PairRig({"charts_directory": directory})
            rig.run_pair()
            original = {key: value for (section, key), value in rig.configfile.saved.items() if section == rig.workflow.model_section}
            for change in ("four", "context", "lineage", "schema", "top", "gap"):
                options = dict(original)
                metadata = json.loads(options["x_metadata"])
                if change == "four":
                    metadata["detector_profiles"]["raw"]["metadata"]["paired_qualification"]["valid_checks"] = 4
                elif change == "context":
                    metadata["detector_profiles"]["filtered"]["metadata"]["homing_speed_mm_s"] += 1.
                elif change == "lineage":
                    metadata["detector_profiles"]["filtered"]["metadata"]["paired_qualification"]["pair_id"] = "0" * 32
                elif change == "schema":
                    options["version"] = "2"
                elif change == "top":
                    metadata["tested_threshold_g"] += .25
                else:
                    pair_id = next(iter(metadata["paired_calibrations"]))
                    proof = metadata["paired_calibrations"][pair_id]["filtered"]
                    proof["discovery_attempt"] += 1
                    proof["validation_attempts"] = [number + 1 for number in proof["validation_attempts"]]
                options["x_metadata"] = json.dumps(metadata)
                with self.subTest(change=change), self.assertRaises(ConfigError):
                    AccelHomingModel(Config(None, options, rig.workflow.model_section))


if __name__ == "__main__":
    unittest.main()
