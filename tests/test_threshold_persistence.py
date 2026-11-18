"""Qualified runtime thresholds, native autosave provenance and transactions."""
import copy
import contextlib
import json
import tempfile
import types
import unittest
from unittest import mock

import test_workflows as workflow_tests
from harness import CommandError, ConfigError
from test_accuracy_workflow import AccuracyRig
from test_workflows import Rig, workflow_module


class NativePendingConfig:
    """Exercise both supported native pending-state layouts, without files."""

    def __init__(self, modern=False, values=None):
        parser = workflow_module.configparser.RawConfigParser()
        self.autosave = types.SimpleNamespace(fileconfig=parser)
        self.state = self.autosave if modern else self
        self.state.status_save_pending = {}
        self.state.save_config_pending = False
        self.fail_key = None
        for (section, option), value in (values or {}).items():
            if not parser.has_section(section):
                parser.add_section(section)
            parser.set(section, option, str(value))

    @property
    def saved(self):
        parser = self.autosave.fileconfig
        return {(section, option): value for section in parser.sections()
                for option, value in parser.items(section, raw=True)}

    def set(self, section, option, value):
        parser = self.autosave.fileconfig
        if not parser.has_section(section):
            parser.add_section(section)
        parser.set(section, option, str(value))
        pending = copy.deepcopy(self.state.status_save_pending)
        pending.setdefault(section, {})[option] = str(value)
        self.state.status_save_pending = pending
        self.state.save_config_pending = True
        if self.fail_key == (section, option):
            raise CommandError("synthetic native partial set failure")


class ThresholdPersistenceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="accel-homing-threshold-")
        self.addCleanup(self.directory.cleanup)
        publication = mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"})
        publication.start()
        self.addCleanup(publication.stop)

    def rig(self, options=None, sections=None, native=None, rig_type=Rig):
        options = dict(options or {}, charts_directory=self.directory.name)
        factory = (mock.patch.object(workflow_tests, "Configfile", return_value=native)
                   if native is not None else contextlib.nullcontext())
        with factory:
            rig = rig_type(options, sections=sections)
        # The generic motion fixture supplies fixed defaults; reproduce the
        # owner's per-axis-over-shared configuration precedence for overrides.
        for axis in rig.workflow.manual_axes:
            value = options.get(axis + "_threshold", options.get("threshold"))
            if value is not None:
                rig.owner.threshold_g[axis] = float(value)
        return rig

    def calibrated_values(self):
        rig = self.rig({"x_threshold": 8.})
        rig.calibrate("X", save=0, START_THRESHOLD=2.25)
        return copy.deepcopy(rig.configfile.saved)

    def reload(self, values, modern=False, options=None, remove_origin=None, rig_type=Rig):
        native = NativePendingConfig(modern, values)
        if remove_origin is not None:
            native.autosave.fileconfig.remove_option(*remove_origin)
        sections = {"printer": {"kinematics": "corexy"}, "accel_homing_model default": {
            option: value for (section, option), value in values.items()
            if section == "accel_homing_model default"}}
        runtime = {option: value for (section, option), value in values.items() if section == "accel_homing"}
        runtime.update(options or {})
        return self.rig(runtime, sections, native, rig_type)

    def test_accuracy_after_reload_uses_calibrated_axis_threshold_instead_of_old_shared_eight(self):
        values = self.calibrated_values()
        for modern in (False, True):
            with self.subTest(modern=modern):
                rig = self.reload(values, modern, {"threshold": 8.}, rig_type=AccuracyRig)
                rig.accuracy("X", samples=2)
                self.assertEqual(rig.workflow.last_result["fixed_thresholds_g"], {"x": 2.75})
                self.assertEqual([home["threshold"] for home in rig.home_calls], [2.75] * 3)
                self.assertEqual(rig.owner.threshold_g, {"x": 2.75, "y": 8.})
                self.assertEqual(rig.workflow.last_result["result"], "complete")
                self.assertNotIn("SAVE_CONFIG", rig.scripts)

    def test_selected_threshold_and_model_stage_together_for_every_override_style(self):
        for axis in "xy":
            other = "y" if axis == "x" else "x"
            for source in ("none", "axis", "shared"):
                for legacy in (False, True):
                    with self.subTest(axis=axis, source=source, legacy=legacy):
                        options = ({axis + "_threshold": 8.} if source == "axis" else
                                   {"threshold": 8.} if source == "shared" else {})
                        rig = self.rig(options)
                        if legacy:
                            rig.config.name = "beacon_accel_homing"
                        before = dict(rig.owner.threshold_g)
                        rig.calibrate(axis.upper(), save=0, START_THRESHOLD=2.25)
                        saved = rig.configfile.saved
                        self.assertEqual(saved[(rig.config.get_name(), axis + "_threshold")], "2.75")
                        self.assertEqual(saved[("accel_homing_model default", axis + "_threshold")], "2.75")
                        self.assertNotIn((rig.config.get_name(), other + "_threshold"), saved)
                        self.assertNotIn((rig.config.get_name(), "threshold"), saved)
                        metadata = json.loads(saved[("accel_homing_model default", axis + "_metadata")])
                        self.assertEqual(metadata["runtime_threshold_setting"], {
                            "version": 1, "section": rig.config.get_name(), "option": axis + "_threshold",
                            "threshold_g": 2.75, "run_id": metadata["run_id"]})
                        self.assertEqual(rig.owner.threshold_g, before)
                        self.assertEqual([home["threshold"] for home in rig.home_calls], [2.25] * 11)
                        self.assertNotIn("SAVE_CONFIG", rig.scripts)
                        self.assertEqual(rig.toolhead.position[2:], [40., 6.])

    def test_native_generated_pair_keeps_model_guards_even_with_shared_fallback(self):
        values = self.calibrated_values()
        for modern in (False, True):
            for shared in (False, True):
                for changed in ("none", "speed", "geometry"):
                    with self.subTest(modern=modern, shared=shared, changed=changed):
                        rig = self.reload(values, modern, {"threshold": 8.} if shared else None)
                        self.assertNotIn("x", rig.workflow.manual_axes)
                        self.assertEqual("y" in rig.workflow.manual_axes, shared)
                        self.assertEqual(rig.owner.threshold_g["x"], 2.75)
                        self.assertEqual(rig.owner.threshold_g["y"], 8. if shared else 9.)
                        if changed == "speed":
                            rig.owner.rails_by_axis["x"].info.speed = 80.
                        elif changed == "geometry":
                            rig.owner.rails_by_axis["x"].bounds = (-10., 300.)
                        if changed == "none":
                            rig.workflow._validate_model_connection()
                        else:
                            with self.assertRaisesRegex(ConfigError, "does not match"):
                                rig.workflow._validate_model_connection()

    def test_literal_override_keeps_manual_semantics_even_when_its_value_matches(self):
        values = self.calibrated_values()
        for modern in (False, True):
            for threshold in (2.75, 8.):
                with self.subTest(modern=modern, threshold=threshold):
                    # Native config loading strips a duplicated autosave option
                    # when a regular or included literal defines the same key.
                    rig = self.reload(values, modern, {"x_threshold": threshold, "threshold": 7.},
                                      ("accel_homing", "x_threshold"))
                    self.assertEqual(rig.workflow.manual_axes, {"x", "y"})
                    self.assertEqual(rig.owner.threshold_g, {"x": threshold, "y": 7.})
                    rig.owner.rails_by_axis["x"].info.speed = 80.
                    rig.workflow._validate_model_connection()

    def test_generated_classification_requires_complete_matching_provenance(self):
        values = self.calibrated_values()
        key = ("accel_homing_model default", "x_metadata")
        mutations = {
            "legacy": lambda meta: meta.pop("runtime_threshold_setting"),
            "run": lambda meta: meta["runtime_threshold_setting"].update(run_id="other"),
            "section": lambda meta: meta["runtime_threshold_setting"].update(section="beacon_accel_homing"),
            "option": lambda meta: meta["runtime_threshold_setting"].update(option="y_threshold"),
            "value": lambda meta: meta["runtime_threshold_setting"].update(threshold_g=3.),
            "bool": lambda meta: meta["runtime_threshold_setting"].update(version=True),
        }
        for name, mutate in mutations.items():
            with self.subTest(mutation=name):
                altered = copy.deepcopy(values)
                metadata = json.loads(altered[key])
                mutate(metadata)
                altered[key] = json.dumps(metadata)
                rig = self.reload(altered)
                self.assertIn("x", rig.workflow.manual_axes)
        for missing in (("accel_homing", "x_threshold"), key,
                        ("accel_homing_model default", "x_threshold")):
            with self.subTest(missing=missing):
                rig = self.reload(values, remove_origin=missing)
                self.assertIn("x", rig.workflow.manual_axes)

    def test_absent_runtime_option_still_allows_saved_model_loading(self):
        values = self.calibrated_values()
        values.pop(("accel_homing", "x_threshold"))
        rig = self.reload(values)
        self.assertNotIn("x", rig.workflow.manual_axes)
        self.assertEqual(rig.owner.threshold_g["x"], 2.75)
        rig.workflow._validate_model_connection()

    def test_two_axis_pending_settings_survive_partial_second_axis_failure_on_both_hosts(self):
        for modern in (False, True):
            for fail_option in (None, "y_metadata", "y_threshold"):
                with self.subTest(modern=modern, fail_option=fail_option):
                    native = NativePendingConfig(modern, {("other", "setting"): "keep"})
                    native.state.status_save_pending = {"other": {"setting": "keep"}}
                    native.state.save_config_pending = True
                    rig = self.rig({"threshold": 8.}, native=native)
                    rig.calibrate("X", save=0, START_THRESHOLD=2.25)
                    first_values = native.saved
                    first_pending = copy.deepcopy(native.state.status_save_pending)
                    first_models = copy.deepcopy(rig.workflow.pending_model_metadata)
                    first_targets = copy.deepcopy(rig.workflow.calibration_config_save._targets)
                    self.assertEqual(first_targets, {("accel_homing", "x_threshold"): {"value": "2.75"}})
                    rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
                    rig.idle_state = "Ready"
                    if fail_option:
                        native.fail_key = (("accel_homing_model default" if fail_option.endswith("metadata")
                                            else "accel_homing"), fail_option)
                        with self.assertRaisesRegex(CommandError, "partial set failure"):
                            rig.calibrate("Y", save=0, START_THRESHOLD=2.75)
                        self.assertEqual(native.saved, first_values)
                        self.assertEqual(native.state.status_save_pending, first_pending)
                        self.assertEqual(rig.workflow.pending_model_metadata, first_models)
                        self.assertEqual(rig.workflow.calibration_config_save._targets, first_targets)
                    else:
                        rig.calibrate("Y", save=0, START_THRESHOLD=2.75)
                        self.assertEqual(native.saved[("accel_homing", "x_threshold")], "2.75")
                        self.assertEqual(native.saved[("accel_homing", "y_threshold")], "3.25")
                        self.assertEqual(native.saved[("accel_homing_model default", "x_metadata")],
                                         first_values[("accel_homing_model default", "x_metadata")])
                        self.assertEqual(native.saved[("other", "setting")], "keep")
                        self.assertEqual(rig.workflow.calibration_config_save._targets, {
                            ("accel_homing", "x_threshold"): {"value": "2.75"},
                            ("accel_homing", "y_threshold"): {"value": "3.25"}})
                    self.assertTrue(native.state.save_config_pending)
                    self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 8.})
                    self.assertNotIn("SAVE_CONFIG", rig.scripts)
                    self.assertEqual(rig.toolhead.position[2:], [40., 6.])

    def test_missing_native_save_handler_rejects_before_staging_threshold_or_model(self):
        native = NativePendingConfig(True, {("other", "setting"): "keep"})
        rig = self.rig({"x_threshold": 8.}, native=native)
        rig.gcode.register_command("SAVE_CONFIG", None)
        before = native.saved
        with self.assertRaisesRegex(CommandError, "SAVE_CONFIG handler is unavailable"):
            rig.calibrate("X", save=0, START_THRESHOLD=2.25)
        self.assertEqual(native.saved, before)
        self.assertEqual(native.state.status_save_pending, {})
        self.assertFalse(native.state.save_config_pending)
        self.assertEqual(rig.workflow.pending_model_metadata, {})
        self.assertEqual(rig.workflow.calibration_config_save._targets, {})
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertEqual(rig.workflow.last_result["result"], "aborted")


if __name__ == "__main__":
    unittest.main()
