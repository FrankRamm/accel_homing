"""Source-only calibration noise regressions; execution requires operator approval."""
import contextlib
import itertools
import json
import math
import tempfile
import unittest
from unittest import mock

from harness import CommandError
from test_retract_noise import NoiseRig
from test_workflows import GCmd, workflow_module


class CalibrationNoisePreflightTests(unittest.TestCase):
    def assert_no_motion_or_config(self, rig):
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.toolhead.dwells, [])
        self.assertEqual(rig.disabled, [])
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
        self.assertIsNone(rig.workflow.session)

    def test_noise_precedes_each_axis_motion_and_uses_sample_xyz_magnitude(self):
        for axis in "XY":
            with self.subTest(axis=axis), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp, "calibration_start_threshold": .5},
                               amplitudes=(.6, .6, 0.))
                initial = (rig.toolhead.get_position(), list(rig.toolhead.kin.limits))
                identities, active_status = [], []

                def observe_noise(iteration):
                    identities.append(id(rig.workflow.session))
                    active_status.append(rig.workflow.get_status(0)["active"])
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.toolhead.dwells, [])
                    self.assertEqual(rig.disabled, [])
                    self.assertEqual((rig.toolhead.get_position(), rig.toolhead.kin.limits), initial)
                    with self.assertRaisesRegex(CommandError, "supervised workflow"):
                        rig.workflow.check_home_axes([axis.lower()])

                def observe_home(current_axis):
                    self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
                    identities.append(id(rig.workflow.session))
                    active = rig.workflow.get_status(0)["active"]
                    self.assertEqual(active["stationary_noise"]["impact_metric"], "xyz_magnitude")

                rig.on_noise, rig.on_home = observe_noise, observe_home
                rig.calibrate(axis, save=0, STEP=.2)
                self.assertEqual(len(set(identities)), 1)
                self.assertTrue(all(item["purpose"] == "calibration_preflight" for item in active_status))
                self.assertTrue(all(item["calibration_axis"] == axis.lower() for item in active_status))
                self.assertEqual([call["threshold"] for call in rig.home_calls], [1.1] * 11)
                result = rig.workflow.last_result
                measurement = result["stationary_noise"]
                self.assertAlmostEqual(measurement["noise_magnitude_peak_g"], math.sqrt(.6 ** 2 * 2))
                self.assertEqual(measurement["initial_threshold_source"], "stationary_noise")
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", axis.lower() + "_metadata")])
                self.assertEqual(metadata["stationary_noise_seed"]["candidate_step_g"], .2)
                self.assertEqual(metadata["stationary_noise_seed"]["start_threshold_g"], 1.1)
                self.assertEqual(metadata["stationary_noise_measurement"], measurement)
                payload = next(item for item in rig.workflow.renderer.calls if item["kind"] == "noise")
                self.assertAlmostEqual(payload["records"][0]["noise_magnitude_peak_g"],
                                       measurement["noise_magnitude_peak_g"])

    def test_explicit_start_below_noise_is_preserved_but_still_requires_measurement(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp}, amplitudes=(2., 2., 0.))
            rig.calibrate(save=0, START_THRESHOLD=.5, STEP=.2)
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
            self.assertEqual([call["threshold"] for call in rig.home_calls], [.5] * 11)
            measurement = rig.workflow.last_result["stationary_noise"]
            self.assertGreater(measurement["noise_magnitude_peak_g"], .5)
            self.assertEqual(measurement["initial_threshold_source"], "explicit_start_threshold")
            metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            self.assertNotIn("stationary_noise_seed", metadata)
            self.assertIn("stationary_noise_measurement", metadata)

    def test_auto_seed_uses_current_step_rounds_up_and_never_lowers_configured_start(self):
        for configured, step, peak, expected in ((1., .1, 1., 1.2), (.5, .3, 2., 2.6),
                                                  (3., .25, .1, 3.)):
            with self.subTest(configured=configured, step=step), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp, "calibration_start_threshold": configured},
                               amplitudes=(peak, 0., 0.))
                rig.calibrate(save=0, STEP=step)
                self.assertEqual(rig.home_calls[0]["threshold"], expected)
                self.assertEqual(rig.workflow.last_result["threshold_step_g"], step)
                self.assertEqual(rig.workflow.start_threshold, configured)

    def test_unknown_range_and_over_headroom_auto_seed_abort_without_clamp(self):
        for kind, explicit in itertools.product(("unknown-range", "over-headroom"), (False, True)):
            with self.subTest(kind=kind, explicit=explicit), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp},
                               amplitudes=(2., 2., 0.))
                rig.owner.full_scale_g = None if kind == "unknown-range" else 2.5
                if explicit:
                    rig.calibrate(save=0, START_THRESHOLD=.5)
                    self.assertEqual(rig.home_calls[0]["threshold"], .5)
                else:
                    with self.assertRaisesRegex(CommandError, "known sensor range|sensor-range headroom"):
                        rig.calibrate(save=0)
                    self.assert_no_motion_or_config(rig)
                    self.assertEqual(rig.workflow.last_result["purpose"], "calibration_preflight")
                self.assertEqual(rig.noise_calls[:4], ["start", 1, 2, 3])

    def test_retries_and_ten_home_validation_do_not_repeat_noise_but_fresh_commands_do(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp}, amplitudes=(.01, .01, .01))
            rig.home_results.extend([{"travel_mm": 8.}, {"travel_mm": 10.}] + [{}] * 11)
            rig.calibrate(save=0)
            first_run = rig.workflow.last_result["run_id"]
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
            self.assertEqual(rig.workflow.last_result["successes"], 10)
            self.assertEqual(rig.workflow.last_result["restarts"], 2)
            self.assertEqual(rig.workflow.last_result["threshold_g"], 1.5)
            self.assertEqual(rig.workflow.last_result["repeatability_limit_mm"], 2.)
            rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
            rig.idle_state = "Ready"
            rig.calibrate("Y", save=0)
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"] * 2)
            self.assertNotEqual(rig.workflow.last_result["run_id"], first_run)

    def test_standalone_hints_are_not_reused_by_fresh_calibration(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp}, amplitudes=(4., 0., 0.))
            rig.workflow.cmd_NOISE(GCmd())
            old_run = rig.workflow.last_result["run_id"]
            self.assertGreater(rig.workflow.noise_seeds["x"], 1.)
            rig.amplitudes = (.01, .01, .01)
            rig.calibrate(save=0)
            self.assertEqual(rig.home_calls[0]["threshold"], 1.)
            self.assertNotEqual(rig.workflow.last_result["stationary_noise"]["run_id"], old_run)
            self.assertEqual(rig.workflow.noise_seeds, {})

    def test_invalid_inputs_reject_before_noise_and_never_forward_save_to_noise_command(self):
        for kind in ("centered", "homed", "geometry", "stock", "policy", "grid", "save"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp})
                params = {"AXIS": "X", "CENTERED": 0 if kind == "centered" else 1, "SAVE_CONFIG": 0}
                if kind == "homed": rig.toolhead.kin.limits[0] = (0., 300.)
                elif kind == "geometry": rig.owner.rails_by_axis["x"].bounds = (0., 40.)
                elif kind == "stock": rig.gcode.gcode_handlers["G28"] = lambda g: None
                elif kind == "policy": rig.policy_error = "synthetic unsupported motion policy"
                elif kind == "grid": params["STEP"] = 0
                elif kind == "save": params["SAVE_CONFIG"] = 2
                with self.assertRaises(CommandError):
                    rig.workflow.cmd_CALIBRATE(GCmd(**params))
                self.assertEqual(rig.noise_calls, [])
                self.assert_no_motion_or_config(rig)
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp}, amplitudes=(.01, .01, .01))
            with mock.patch.object(rig.workflow, "cmd_NOISE", side_effect=AssertionError("nested command")):
                rig.calibrate(save=1)
            self.assertEqual(rig.scripts[-1], "SAVE_CONFIG")

    def test_sensor_and_export_faults_abort_before_first_home(self):
        for fault in ("capture", "record", "capture_export", "summary_export"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp})
                collect = rig.owner.collect_noise_window

                def bad_capture(check_running, duration):
                    capture = collect(check_running, duration)
                    if fault == "capture": capture["fault"] = "saturated"
                    elif fault == "record": capture["cutoff_time_s"] = capture["samples"][0][0]
                    return capture

                rig.owner.collect_noise_window = bad_capture
                target = "write_noise_capture" if fault == "capture_export" else "write_noise"
                exporter = mock.patch.object(workflow_module, target, side_effect=OSError("synthetic export failure")) \
                    if fault.endswith("export") else contextlib.nullcontext()
                with exporter, mock.patch.object(workflow_module.logging, "exception"), self.assertRaises(Exception):
                    rig.calibrate(save=0)
                self.assert_no_motion_or_config(rig)
                self.assertEqual(rig.workflow.noise_seeds, {})

    def test_interruptions_and_context_changes_at_each_noise_yield_never_allow_motion(self):
        for stage, fault in itertools.product(("preflight", "window", "capture_export", "stop", "summary", "display"),
                                              ("cancel", "disconnect", "motor-off", "position", "queue")):
            with self.subTest(stage=stage, fault=fault), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp})
                interrupted = [False]

                def interrupt():
                    if interrupted[0]: return
                    interrupted[0] = True
                    if fault == "cancel": rig.workflow.cmd_CANCEL(GCmd())
                    elif fault == "disconnect": rig.workflow._disconnect()
                    elif fault == "motor-off": rig.owner.operation_generation += 1
                    elif fault == "position": rig.toolhead.position[1] += 1.
                    elif fault == "queue": rig.toolhead.busy = (11., 10., True)

                def rendered(payload):
                    if payload["kind"] == ("preflight" if stage == "preflight" else "noise"):
                        interrupt()

                def at_window(iteration):
                    if iteration == 3: interrupt()

                def after_call(original):
                    def wrapped(*args, **kwargs):
                        result = original(*args, **kwargs)
                        interrupt()
                        return result
                    return wrapped

                if stage in ("preflight", "summary"): rig.workflow.renderer.on_render = rendered
                elif stage == "window": rig.on_noise = at_window
                target, method = {"capture_export": (workflow_module, "write_noise_capture"),
                                  "stop": (rig.owner, "stop_noise_measurement"),
                                  "display": (rig.workflow, "_show_pngs")}.get(stage, (None, None))
                wrapper = mock.patch.object(target, method, side_effect=after_call(getattr(target, method))) \
                    if target is not None else contextlib.nullcontext()
                with wrapper, mock.patch.object(workflow_module.logging, "exception"), self.assertRaises(Exception):
                    rig.calibrate(save=0)
                self.assertTrue(interrupted[0])
                self.assert_no_motion_or_config(rig)
                self.assertEqual(rig.workflow.noise_seeds, {})


if __name__ == "__main__":
    unittest.main()
