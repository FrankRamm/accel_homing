"""Post-qualification calibration margin; synthetic workflows only."""
import itertools
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from harness import CommandError, ConfigError, plugin
from test_accuracy_recovery import RecoveryRig
from test_workflows import GCmd, Rig, workflow_module


class CalibrationMarginTests(unittest.TestCase):
    def metadata(self, rig, axis="x"):
        return json.loads(rig.configfile.saved[("accel_homing_model default", axis + "_metadata")])

    def test_each_axis_and_save_mode_add_margin_only_after_qualification(self):
        for axis, save, tested in itertools.product("xy", (0, 1), (1., .755)):
            with self.subTest(axis=axis, save=save, tested=tested):
                rig = Rig()
                before = dict(rig.owner.threshold_g)
                rig.calibrate(axis.upper(), save=save, START_THRESHOLD=tested, DETECTOR="RAW")
                expected = 1.5 if tested == 1. else 1.255
                self.assertEqual([call["threshold"] for call in rig.home_calls], [tested] * 11)
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), save)
                self.assertEqual(rig.owner.threshold_g, before)
                result, metadata = rig.workflow.last_result, self.metadata(rig, axis)
                self.assertEqual(result["candidate_g"], tested)
                self.assertEqual(result["threshold_g"], expected)
                self.assertEqual(float(rig.configfile.saved[("accel_homing_model default", axis + "_threshold")]),
                                 expected)
                for report in (result, metadata):
                    self.assertEqual(report["tested_threshold_g"], tested)
                    self.assertEqual(report["saved_threshold_g"], expected)
                    self.assertEqual(report["threshold_margin_g"], .5)
                    self.assertFalse(report["saved_threshold_tested"])
                    json.dumps(report, allow_nan=False)
                other = "y" if axis == "x" else "x"
                self.assertNotIn(("accel_homing_model default", other + "_threshold"), rig.configfile.saved)
                self.assertTrue(any("tested at %g g" % tested in message
                                    and "staging %g g" % expected in message
                                    and "not tested by those homes" in message for message in rig.output))
                # This chart describes the initial stationary measurement. The
                # final margin must not rewrite its measured start/hint values.
                self.assertEqual(result["stationary_noise"]["start_threshold_g"], tested)
                payload = next(item for item in rig.workflow.renderer.calls if item["kind"] == "noise")
                self.assertNotIn("saved_threshold_g", payload["recommendation"])

    def test_rejected_candidates_keep_normal_steps_and_margin_applies_to_final_candidate(self):
        rig = Rig()
        rig.home_results.extend([{"travel_mm": 8.}, {}, {"travel_mm": 150.},
                                 {"travel_mm": 152.001}] + [{}] * 11)
        rig.calibrate(save=0, DETECTOR="RAW")
        self.assertEqual([call["threshold"] for call in rig.home_calls],
                         [1.] + [1.25] * 3 + [1.5] * 11)
        self.assertEqual(rig.workflow.last_result["restarts"], 2)
        self.assertEqual(rig.workflow.last_result["tested_threshold_g"], 1.5)
        self.assertEqual(rig.workflow.last_result["saved_threshold_g"], 2.)
        self.assertEqual(len(self.metadata(rig)["onset_g"]), 10)

    def test_staging_and_retried_native_save_do_not_add_margin_again(self):
        rig = Rig()
        rig.save_error = True
        with self.assertRaisesRegex(CommandError, "SAVE_CONFIG"):
            rig.calibrate(save=1, DETECTOR="RAW")
        staged = dict(rig.configfile.saved)
        self.assertEqual(rig.workflow.last_result["result"], "save_failed")
        self.assertEqual(float(staged[("accel_homing_model default", "x_threshold")]), 1.5)
        rig.save_error = False
        rig.gcode.run_script_from_command("SAVE_CONFIG")
        self.assertEqual(rig.configfile.saved, staged)
        self.assertEqual(len(rig.home_calls), 11)

    def test_sequential_axes_and_recalibration_preserve_other_axis_staged_data(self):
        rig = Rig()
        rig.configfile.saved[("accel_homing_model default", "y_threshold")] = "4.5"
        rig.configfile.saved[("accel_homing_model default", "y_metadata")] = '{"existing":true}'
        rig.calibrate("X", save=0, DETECTOR="RAW")
        self.assertEqual(rig.configfile.saved[("accel_homing_model default", "y_threshold")], "4.5")
        self.assertEqual(self.metadata(rig, "y"), {"existing": True})
        x_saved = {key: value for key, value in rig.configfile.saved.items() if key[1].startswith("x_")}
        rig.workflow._disable_xy()
        rig.idle_state = "Ready"
        rig.calibrate("Y", save=0, START_THRESHOLD=2., DETECTOR="RAW")
        self.assertEqual({key: value for key, value in rig.configfile.saved.items() if key[1].startswith("x_")},
                         x_saved)
        y_saved = {key: value for key, value in rig.configfile.saved.items() if key[1].startswith("y_")}
        rig.workflow._disable_xy()
        rig.idle_state = "Ready"
        rig.calibrate("X", save=0, DETECTOR="RAW")
        self.assertEqual(float(rig.configfile.saved[("accel_homing_model default", "x_threshold")]), 1.5)
        self.assertEqual({key: value for key, value in rig.configfile.saved.items() if key[1].startswith("y_")},
                         y_saved)

    def test_exact_sensor_ceiling_round_trips_without_adding_another_margin(self):
        rig = Rig()
        rig.calibrate(save=0, START_THRESHOLD=12.3, DETECTOR="RAW")
        value = float(rig.configfile.saved[("accel_homing_model default", "x_threshold")])
        self.assertEqual(value, 12.8)
        plugin.validate_threshold(value, "x_threshold", ConfigError, 16.)
        options = {option: configured for (section, option), configured in rig.configfile.saved.items()
                   if section == "accel_homing_model default"}
        reloaded = Rig(sections={"printer": {"kinematics": "corexy"},
                                 "accel_homing_model default": options})
        reloaded.workflow._validate_model_connection()
        self.assertEqual(reloaded.owner.threshold_g, {"x": 12.8, "y": 9.})
        self.assertEqual(reloaded.workflow.model.thresholds, {"x": 12.8})
        self.assertEqual(reloaded.configfile.saved, {})
        self.assertEqual(self.metadata(rig)["tested_threshold_g"], 12.3)

    def test_margin_exceeding_sensor_ceiling_releases_then_aborts_without_write_or_png(self):
        for save, tested in itertools.product((0, 1), (12.300000000000002, 12.5, 12.8)):
            with self.subTest(save=save, tested=tested), tempfile.TemporaryDirectory() as directory:
                rig = Rig({"charts_directory": directory})
                with mock.patch.object(rig.workflow, "_release_terminal_home",
                                       wraps=rig.workflow._release_terminal_home) as release, \
                        mock.patch.object(workflow_module, "prepare_noise") as render, \
                        self.assertRaisesRegex(CommandError, "saved calibration threshold.*headroom"):
                    rig.calibrate(save=save, START_THRESHOLD=tested, DETECTOR="RAW")
                release.assert_called_once()
                render.assert_not_called()
                self.assertEqual([call["threshold"] for call in rig.home_calls], [tested] * 11)
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertEqual(list(Path(directory).iterdir()), [])
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertIsNone(rig.workflow.session)

    def test_range_changed_during_terminal_release_checks_saved_value_before_write(self):
        rig = Rig()
        release = rig.workflow._release_terminal_home

        def reduced_range(axis, record):
            release(axis, record)
            # The tested 1 g candidate still fits; its 1.5 g saved value does not.
            rig.owner.full_scale_g = 1.5

        with mock.patch.object(rig.workflow, "_release_terminal_home", side_effect=reduced_range), \
                self.assertRaisesRegex(CommandError, "saved calibration threshold.*headroom"):
            rig.calibrate(save=0, DETECTOR="RAW")
        self.assertEqual(rig.configfile.saved, {})
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertEqual(rig.workflow.last_result["candidate_g"], 1.)

    def test_unrepresentable_half_g_increment_aborts_without_staging(self):
        for tested in (1e308, float(2 ** 52 + 1)):
            with self.subTest(tested=tested):
                rig = Rig()
                rig.owner.full_scale_g = None
                with self.assertRaisesRegex(CommandError, "preserve the full finite.*0.5 g margin"):
                    rig.calibrate(save=0, START_THRESHOLD=tested, DETECTOR="RAW")
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertIsNone(rig.workflow.session)

    def test_failed_or_cancelled_qualification_does_not_stage_a_margin(self):
        for fault in ("sensor", "cancel"):
            with self.subTest(fault=fault):
                rig = Rig()
                if fault == "sensor":
                    rig.home_results.extend([{}] * 10 + [{"result": "no_data"}])
                else:
                    stop = rig.owner.stop_capture

                    def cancel_after_last_capture():
                        capture = stop()
                        if len(rig.home_calls) == 11:
                            rig.workflow.cmd_CANCEL(GCmd())
                        return capture

                    rig.owner.stop_capture = cancel_after_last_capture
                with self.assertRaises(CommandError):
                    rig.calibrate(save=1, DETECTOR="RAW")
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertNotIn("saved_threshold_g", rig.workflow.last_result)

    def test_accuracy_recovery_keeps_quarter_start_and_adds_half_to_final_candidate(self):
        for axis in "xy":
            with self.subTest(axis=axis), tempfile.TemporaryDirectory() as directory:
                rig = RecoveryRig({"charts_directory": directory, "calibration_step": .5,
                                   axis + "_threshold": 2.25})
                rig.owner.threshold_g[axis] = 2.25
                rig.home_results.extend([{"travel_mm": 51.175}, {"travel_mm": 8.}] + [{}] * 11)
                rig.accuracy(axis=axis.upper(), DETECTOR="RAW")
                self.assertEqual([call["threshold"] for call in rig.home_calls],
                                 [2.25, 2.5] + [3.] * 11)
                result = rig.workflow.last_result
                self.assertEqual(result["accuracy_recovery"]["start_threshold_g"], 2.5)
                self.assertEqual(result["accuracy_recovery"]["initial_increment_g"], .25)
                self.assertEqual(result["tested_threshold_g"], 3.)
                self.assertEqual(result["saved_threshold_g"], 3.5)
                self.assertEqual(float(rig.configfile.saved[("accel_homing_model default", axis + "_threshold")]), 3.5)
                self.assertEqual(float(rig.configfile.saved[(rig.config.get_name(), axis + "_threshold")]), 3.5)
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)
                other = "y" if axis == "x" else "x"
                self.assertNotIn(("accel_homing_model default", other + "_threshold"), rig.configfile.saved)


if __name__ == "__main__":
    unittest.main()
