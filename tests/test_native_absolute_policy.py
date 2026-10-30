"""Native impact metrics use ABS after reference subtraction, preserving input."""
import copy
import json
import types
import unittest
from unittest import mock

import accel_homing_filters as filters
from accel_homing_model import AccelHomingModel
from harness import CommandError, arm_axis, batch, counts, make_homing, plugin
import test_filter_engine as filter_tests
from test_workflows import Config, GCmd, Rig
from test_chronological_clipping import native_filtered_capture
from test_hybrid_position import native_hybrid_capture


class SetGCmd(GCmd):
    def get_float(self, name, default=None, **bounds):
        return None if name not in self.params and default is None else super().get_float(name, default, **bounds)

    def get_int(self, name, default=None, **bounds):
        return None if name not in self.params and default is None else super().get_int(name, default, **bounds)


class AbsoluteNativeTests(unittest.TestCase):
    def test_mirrored_centered_xyz_decisions_and_hybrid_position_preserve_signed_rows(self):
        baseline = (100, -50, counts(1.))
        for kind in ("raw", "filtered", "hybrid"):
            records, captures = [], []
            for sign in (-1, 1):
                high = (baseline[0] + sign * counts(3.), baseline[1], baseline[2])
                options = dict(reference_rows=[baseline] * 16,
                    row_values=[baseline] * 3 + [high] * 13, packet_size=16, clip_position=None,
                    clock_hz=32000000., sample_period=.0003, finish_home_wait=True, mode="accuracy")
                capture, record = (native_hybrid_capture(2.5, **options) if kind == "hybrid" else
                                   native_filtered_capture(detector_type=kind, **options))
                captures.append(capture)
                records.append(record)
            for key in ("onset_sample", "crossing_sample", "confirmation_sample", "returned_trigger_time_s", "onset_g", "impact_peak_g"):
                self.assertEqual(records[0][key], records[1][key], (kind, key))
            self.assertLess(min(row[1] for row in captures[0]["samples"]), 0.)
            self.assertGreater(max(row[1] for row in captures[1]["samples"]), 0.)
            if kind == "hybrid":
                self.assertEqual(records[0]["position_selection"]["selected_time_s"], records[1]["position_selection"]["selected_time_s"])
                self.assertLess(records[0]["position_selection"]["selected_raw_sensor_counts"][0], 0)
                self.assertGreater(records[1]["position_selection"]["selected_raw_sensor_counts"][0], 0)

    def test_selected_axis_ignores_configured_and_automatic_direction_after_nonzero_reference(self):
        baseline = (100, -50, 128)
        for configured in (-1, 0, 1):
            for automatic in (-1, 1):
                for sign in (-1, 1):
                    obj = make_homing(impact_sign=configured)
                    for first in range(0, 96, 8):
                        obj._inspect(batch(first, [baseline] * 8))
                    attempt, trigger = arm_axis(obj, "x", 200, 9000)
                    attempt.auto_direction = automatic
                    rows = [baseline] * 8
                    rows[3] = (100 + sign * counts(12.), -50, 128)
                    obj._inspect(batch(200, rows))
                    self.assertEqual(trigger.messages, [[plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT]])
                    self.assertEqual(attempt.onset_sample[2], 3)
                    self.assertGreater(attempt.onset_raw, 0.)
                    obj.disarm()
                    self.assertEqual(attempt.record["direction"], 0)
                    self.assertEqual(attempt.record["impact_sign"], 0)

    def test_signed_v1_is_readable_offline_but_rejected_before_native_homing_entry(self):
        rig = Rig()
        metadata = filter_tests.ProfileTests().metadata(rig, "filtered", version=1)
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": metadata})
        rig.owner.workflows = rig.workflow
        rail = types.SimpleNamespace(get_endstops=lambda: [(rig.owner.endstops["x"], "x")])
        with mock.patch.object(rig.owner, "_end_session", side_effect=AssertionError("pre-motion hook ran")), self.assertRaisesRegex(CommandError, "rectified v2"):
            plugin.BeaconAccelHoming._handle_rails_begin(rig.owner, None, [rail])
        with self.assertRaisesRegex(CommandError, "rectified v2"):
            rig.workflow._select_detectors(GCmd(DETECTOR="FILTERED"), "accuracy", ["x"])
        loaded = AccelHomingModel(Config(None, {"version": 2, "x_threshold": 2., "x_successes": 10,
            "x_metadata": json.dumps(metadata)}, "accel_homing_model default"))
        self.assertEqual(loaded.metadata["x"]["detector_config"]["version"], 1)
        engine = filters.CausalImpactDetector(1., (0., 0., 1.), config=metadata["detector_config"])
        self.assertEqual(engine.signal_mode, "signed")

    def test_legacy_selected_axis_profile_needs_requalification_but_manual_override_reaches_entry(self):
        rig = Rig()
        metadata = filter_tests.ProfileTests().metadata(rig, "raw")
        metadata["impact_metric"] = "selected_axis"
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 3.}, metadata={"x": metadata})
        with self.assertRaisesRegex(CommandError, "legacy/unknown sign-gated"):
            rig.workflow.check_home_axes({"x": None})
        rig.workflow._begin(GCmd(DETECTOR="RAW", RAW_THRESHOLD=3.), "accuracy", axes=["x"])
        rig.workflow.in_home, rig.workflow.home_axis = True, "x"
        rig.workflow.check_home_axes({"x": None})
        rig.workflow._restore()
        rig.workflow.session = None
        rig.workflow.manual_axes.add("x")
        rig.workflow.check_home_axes({"x": None})

    def test_successful_set_threshold_is_manual_provenance_and_sign_only_is_not(self):
        obj = make_homing()
        metadata = {"detector_type": "raw", "impact_metric": "selected_axis"}
        obj.workflows.model = types.SimpleNamespace(thresholds={"x": 8.}, metadata={"x": metadata})
        obj.workflows.manual_axes.clear()
        obj.cmd_SET(SetGCmd(AXIS="X", IMPACT_SIGN=1))
        self.assertFalse(obj.workflows.temporary_manual_thresholds)
        with self.assertRaisesRegex(CommandError, "legacy/unknown"):
            obj.workflows.check_home_axes({"x": None})
        obj.cmd_SET(SetGCmd(AXIS="X", THRESHOLD=7.))
        self.assertEqual(obj.workflows.detector_type("x"), "raw")
        obj.workflows.check_home_axes({"x": None})
        with self.assertRaises(CommandError):
            obj.cmd_SET(SetGCmd(AXIS="X", THRESHOLD=-1.))
        self.assertEqual(obj.workflows.temporary_manual_thresholds, {"x": 7.})

    def test_manual_provenance_survives_workflow_restore_without_bypassing_signed_filter_guard(self):
        rig = Rig()
        metadata = filter_tests.ProfileTests().metadata(rig, "raw")
        metadata["impact_metric"] = "selected_axis"
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 3.}, metadata={"x": metadata})
        rig.workflow.note_manual_threshold("x", 8.)
        rig.workflow._begin(GCmd(DETECTOR="RAW", RAW_THRESHOLD=3.), "accuracy", axes=["x"])
        rig.workflow.in_home, rig.workflow.home_axis = True, "x"
        rig.workflow.check_home_axes({"x": None})
        rig.workflow._restore()
        rig.workflow.session = None
        self.assertTrue(rig.workflow._temporary_manual_threshold("x"))
        rig.owner.workflows = rig.workflow
        rail = types.SimpleNamespace(get_endstops=lambda: [(rig.owner.endstops["x"], "x")])
        with mock.patch.object(rig.owner, "_end_session", side_effect=RuntimeError("passed actual pre-motion polarity check")), self.assertRaisesRegex(RuntimeError, "passed actual"):
            plugin.BeaconAccelHoming._handle_rails_begin(rig.owner, None, [rail])
        rig.workflow.model.metadata["x"] = filter_tests.ProfileTests().metadata(rig, "filtered", version=1)
        with self.assertRaisesRegex(CommandError, "rectified v2"):
            rig.workflow.check_home_axes({"x": None})


if __name__ == "__main__":
    unittest.main()
