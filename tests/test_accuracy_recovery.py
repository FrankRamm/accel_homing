"""Synthetic accuracy-to-calibration recovery; no printer or service access."""
import copy
import itertools
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from harness import CommandError
from test_accuracy_workflow import AccuracyRig
from test_calibration_auto_return import CartesianKin
from test_workflows import AccelHomingWorkflows, Config, GCmd, workflow_module


class RecoveryRig(AccuracyRig):
    def stock_home(self, gcmd):
        if self.home_results and isinstance(self.home_results[0], dict):
            result = self.home_results[0]
            axis = next(axis.lower() for axis in "XY" if gcmd.get(axis, None) is not None)
            result.setdefault("start_position_mm", self.toolhead.position["xy".index(axis)])
            result.setdefault("travel_mm_exact", result.get("travel_mm", self.workflow._geometry(axis)[1]))
        return super().stock_home(gcmd)

    def stop_capture(self):
        capture = super().stop_capture()
        if capture.get("detector") is not None:
            capture["detector"]["impact_metric"] = capture["_record"].get("impact_metric", "selected_axis")
        return capture


class AccuracyRecoveryTests(unittest.TestCase):
    def rig(self, directory, axis="x", positive=False, cartesian=False, **options):
        rig = RecoveryRig(dict(options, charts_directory=directory))
        if cartesian:
            rig.toolhead.kin = CartesianKin()
            rig.workflow.kinematics_name = "cartesian"
        rail = rig.owner.rails_by_axis[axis]
        rail.info.positive_dir = positive
        rail.info.position_endstop = rail.bounds[1 if positive else 0]
        rig.owner.threshold_g[axis] = 2.25
        return rig

    def assert_no_calibration(self, rig):
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.noise_calls, [])
        self.assertIsNone(rig.workflow.session)
        self.assertFalse(rig.capture_active)
        self.assertTrue(all(not path.exists() for path in rig.capture_directories))

    def test_early_accuracy_restarts_full_calibration_at_effective_threshold_plus_quarter(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = RecoveryRig({"charts_directory": directory})
            rig.owner.threshold_g["x"] = 2.25
            rig.home_results.extend([{"travel_mm": 51.175, "halt_travel_mm_signed": -53.175}] + [{}] * 11)
            rig.accuracy()
            self.assertEqual([item["threshold"] for item in rig.home_calls], [2.25] + [2.5] * 11)
            self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)
            self.assertEqual(rig.workflow.last_result["result"], "calibrated")
            self.assertEqual(rig.workflow.last_result["successes"], 10)
            metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            self.assertEqual(metadata["calibration_start_threshold_g"], 2.5)
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
            self.assertTrue(all(not path.exists() for path in rig.capture_directories))
            self.assertTrue(any("ACCEL_HOMING_ACCURACY" in line and "restart" in line.lower()
                                for line in rig.output))

    def test_first_and_late_xy_both_directions_and_kinematics_preserve_reference(self):
        for axis, positive, cartesian, prior in itertools.product("xy", (False, True), (False, True), (0, 2)):
            with self.subTest(axis=axis, positive=positive, cartesian=cartesian, prior=prior), \
                    tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory, axis, positive, cartesian)
                index, sign = "xy".index(axis), 1 if positive else -1
                initial = list(rig.toolhead.position)
                halt = sign * 53.175
                prefix = [{"travel_mm": 150. if axis == "x" else 100.}] * prior
                rig.home_results.extend(prefix + [{"travel_mm": 51.175,
                                                   "halt_travel_mm_signed": halt}] + [{}] * 11)
                returns = []
                move = rig.toolhead.manual_move
                def moving(coordinates, speed):
                    returns.append((rig.toolhead.get_position(), list(coordinates), speed))
                    return move(coordinates, speed)
                with mock.patch.object(rig.toolhead, "manual_move", side_effect=moving), \
                        mock.patch.object(rig.toolhead, "set_position", side_effect=AssertionError("coordinate rebase")):
                    rig.accuracy(axis=axis.upper(), samples=4)
                before, coordinates, speed = returns[prior]
                self.assertAlmostEqual(coordinates[index] - before[index], -halt)
                self.assertEqual(coordinates[index], rig.owner.rails_by_axis[axis].info.position_endstop - sign * 51.175)
                self.assertEqual(speed, 50.)
                self.assertTrue(all(value is None for number, value in enumerate(coordinates) if number != index))
                first_calibration = rig.home_calls[prior + 1]
                self.assertEqual(first_calibration["position"][index], coordinates[index])
                self.assertEqual(first_calibration["homed_axes"], "z")
                self.assertFalse(first_calibration["candidate_qualified"])
                result = rig.workflow.last_result
                self.assertEqual(result["accuracy_recovery"]["return_mm_signed"], -halt)
                self.assertFalse(result["accuracy_recovery"]["physical_position_verified"])
                partial = result["source_accuracy"]
                self.assertEqual(partial["result"], "aborted")
                self.assertEqual(len(partial["axis_results"][axis]["trials"]), max(0, prior - 1))
                self.assertEqual(partial["axis_results"][axis]["rejected"]["travel_mm_exact"], 51.175)
                self.assertTrue(Path(partial["summary_paths"][axis + "_png"]).is_file())
                self.assertEqual(rig.toolhead.position[1 - index], initial[1 - index])
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertEqual(rig.disabled, [])
                self.assertEqual(len(rig.home_calls), prior + 12)
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)
                json.dumps(rig.workflow.get_status(0), allow_nan=False)

    def test_xy_failure_calibrates_only_failing_axis_and_keeps_completed_axis_report(self):
        for axis, prior in (("x", 0), ("x", 2), ("y", 0), ("y", 2)):
            with self.subTest(axis=axis, prior=prior), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory, axis)
                prefix = [{}] * ((3 if axis == "y" else 0) + prior)
                rig.home_results.extend(prefix + [{"travel_mm": 51.175}] + [{}] * 11)
                rig.accuracy(axis="XY")
                result = rig.workflow.last_result
                partial = result["source_accuracy"]
                self.assertEqual(result["axis"], axis)
                self.assertEqual([item["axis"] for item in rig.home_calls[-11:]], [axis] * 11)
                self.assertEqual(result["accuracy_recovery"]["rerun_command"],
                                 "ACCEL_HOMING_ACCURACY AXIS=XY SAMPLES=2 CENTERED=1")
                if axis == "y":
                    self.assertTrue(partial["axis_results"]["x"]["complete"])
                    self.assertEqual(len(partial["axis_results"]["x"]["trials"]), 2)
                self.assertFalse(partial["axis_results"][axis]["complete"])
                other = "y" if axis == "x" else "x"
                self.assertNotIn(("accel_homing_model default", other + "_threshold"), rig.configfile.saved)

    def test_exact_effective_threshold_and_normal_candidate_step_survive_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory, calibration_start_threshold=1., calibration_step=.5)
            rig.owner.threshold_g["x"] = 8.123456789
            rig.home_results.extend([{"travel_mm": 51.175}, {"travel_mm": 8.}] + [{}] * 11)
            rig.accuracy()
            thresholds = [item["threshold"] for item in rig.home_calls]
            self.assertEqual(thresholds, [8.123456789, 8.373456789] + [8.873456789] * 11)
            result = rig.workflow.last_result
            self.assertEqual(result["start_threshold_g"], 8.373456789)
            self.assertEqual(result["threshold_step_g"], .5)
            self.assertEqual(result["accuracy_recovery"]["initial_increment_g"], .25)
            self.assertEqual(float(rig.configfile.saved[("accel_homing_model default", "x_threshold")]),
                             result["threshold_g"])
            commands = [params for command, _line, params in rig.created_commands
                        if command == "ACCEL_HOMING_CALIBRATE"]
            self.assertEqual(len(commands), 1)
            self.assertNotIn("STEP", commands[0])
            self.assertNotIn("CENTERED", commands[0])
            self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)

    def test_manual_shared_axis_and_legacy_section_overrides_save_effective_selected_value(self):
        for axis, shared, legacy in itertools.product("xy", (False, True), (False, True)):
            with self.subTest(axis=axis, shared=shared, legacy=legacy), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory, axis, **({"threshold": 2.25} if shared else {axis + "_threshold": 2.25}))
                if legacy:
                    rig.config.name = "beacon_accel_homing"
                rig.home_results.extend([{"travel_mm": 51.175}] + [{}] * 11)
                rig.accuracy(axis=axis.upper())
                saved = rig.configfile.saved
                value = saved[(rig.config.get_name(), axis + "_threshold")]
                self.assertEqual(float(value), 2.5)
                self.assertNotIn((rig.config.get_name(), "threshold"), saved)
                self.assertNotIn((rig.config.get_name(), ("y" if axis == "x" else "x") + "_threshold"), saved)
                options = dict(rig.config.options, **{axis + "_threshold": value})
                sections = {"printer": {"kinematics": "corexy"}, "accel_homing_model default": {
                    option: configured for (section, option), configured in saved.items()
                    if section == "accel_homing_model default"}}
                rig.owner.threshold_g.update(x=float(options.get("x_threshold", options.get("threshold", 8.))),
                                             y=float(options.get("y_threshold", options.get("threshold", 9.))))
                config = Config(rig, options, rig.config.name, sections)
                rig.objects.pop("accel_homing_model default", None)
                rig.gcode.ready_gcode_handlers.clear()
                reloaded = AccelHomingWorkflows(rig.owner, config)
                self.assertIn(axis, reloaded.manual_axes)
                self.assertEqual(rig.owner.threshold_g[axis], 2.5)

    def test_saved_model_supplies_recovered_value_without_manual_override(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.extend([{"travel_mm": 51.175}] + [{}] * 11)
            rig.accuracy()
            sections = {"printer": {"kinematics": "corexy"}, "accel_homing_model default": {
                option: configured for (section, option), configured in rig.configfile.saved.items()
                if section == "accel_homing_model default"}}
            rig.owner.threshold_g["x"] = 8.
            rig.gcode.ready_gcode_handlers.clear()
            AccelHomingWorkflows(rig.owner, Config(rig, {"charts_directory": directory}, sections=sections))
            self.assertEqual(rig.owner.threshold_g["x"], 2.5)

    def test_save_error_reports_failed_persistence_without_claiming_restart_completed(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory, x_threshold=2.25)
            rig.save_error = True
            rig.home_results.extend([{"travel_mm": 51.175}] + [{}] * 11)
            with self.assertRaisesRegex(CommandError, "SAVE_CONFIG include conflict"):
                rig.accuracy()
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "save_failed")
            self.assertFalse(result["save_config"])
            self.assertIn("SAVE_CONFIG failed", result["reason"])
            self.assertTrue(any("persistence and restart were not confirmed" in line for line in rig.output))
            self.assertEqual(rig.owner.threshold_g["x"], 2.25)
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)

    def test_threshold_headroom_and_minimum_refuse_before_any_return(self):
        for threshold in (.001, .249999, 12.6, 12.8):
            with self.subTest(threshold=threshold), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.owner.threshold_g["x"] = threshold
                rig.home_results.append({"travel_mm": 51.175})
                with self.assertRaisesRegex(CommandError, "START_THRESHOLD"):
                    rig.accuracy()
                self.assertEqual(rig.toolhead.moves, [])
                self.assert_no_calibration(rig)

    def test_invalid_stale_wrong_sign_and_inconsistent_halt_provenance_never_returns(self):
        mutations = {
            "missing-halt": lambda r, c: r.workflow.session.pop("accuracy_halt"),
            "stale-source": lambda r, c: r.workflow.session["accuracy_halt"].update(source=copy.copy(c["_record"])),
            "old-attempt": lambda r, c: r.workflow.session["accuracy_halt"].update(attempt=0),
            "wrong-generation": lambda r, c: r.workflow.session["accuracy_halt"].update(generation=-1),
            "changed-generation": lambda r, c: setattr(r.owner, "operation_generation", 1),
            "changed-preflight": lambda r, c: setattr(r.workflow, "preflight_generation", 1),
            "wrong-halt": lambda r, c: c["_record"].update(halt_travel_mm_signed=53.175),
            "missing-exact": lambda r, c: c["_record"].pop("travel_mm_exact"),
            "missing-stepper": lambda r, c: r.workflow.session["accuracy_halt"].update(positions=()),
            "wrong-coordinate": lambda r, c: r.toolhead.position.__setitem__(0, 2.),
            "other-axis": lambda r, c: r.toolhead.position.__setitem__(1, 101.),
            "z-axis": lambda r, c: r.toolhead.position.__setitem__(2, 41.),
            "extruder": lambda r, c: r.toolhead.position.__setitem__(3, 7.),
            "stepper-changed": lambda r, c: setattr(r.toolhead.kin.steppers[0], "mcu_position", 0),
        }
        for fault, mutation in mutations.items():
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append({"travel_mm": 51.175, "halt_travel_mm_signed": -53.175})
                rig.capture_mutator = lambda capture: mutation(rig, capture)
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(rig.toolhead.moves, [])
                self.assert_no_calibration(rig)

    def test_missing_center_reference_after_external_move_does_not_fabricate_centered(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            original = rig.workflow._home
            def moved_before_home(axis, threshold=None):
                rig.toolhead.position[0] += 1.
                return original(axis, threshold)
            rig.home_results.append({"travel_mm": 51.175})
            with mock.patch.object(rig.workflow, "_home", side_effect=moved_before_home), \
                    self.assertRaisesRegex(CommandError, "confirmed center evidence"):
                rig.accuracy()
            self.assertEqual(rig.toolhead.moves, [])
            self.assert_no_calibration(rig)

    def test_initial_reference_read_failure_releases_session_without_moving(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            with mock.patch.object(rig.workflow, "_accuracy_position_reference",
                                   side_effect=OSError("synthetic MCU position read failure")), \
                    self.assertRaisesRegex(OSError, "MCU position read failure"):
                rig.accuracy()
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.toolhead.moves, [])
            self.assert_no_calibration(rig)
            rig.workflow.cmd_CANCEL(GCmd())

    def test_firmware_synthetic_homing_start_is_not_mistaken_for_center_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.extend([{"travel_mm": 51.175, "start_position_mm": 450.}] + [{}] * 11)
            rig.accuracy()
            self.assertEqual(rig.workflow.last_result["threshold_g"], 2.5)

    def test_missing_config_staging_interface_refuses_before_return(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.append({"travel_mm": 51.175})
            rig.configfile.set = None
            with self.assertRaisesRegex(CommandError, "cannot stage"):
                rig.accuracy()
            self.assertEqual(rig.toolhead.moves, [])
            self.assert_no_calibration(rig)

    def test_sensor_capture_and_native_faults_never_start_recovery(self):
        mutations = (lambda r, c: c.update(fault="stale_data"),
                     lambda r, c: c.update(error="invalid_data"),
                     lambda r, c: c.update(dropped=1),
                     lambda r, c: c["detector"].update(onset_sample_matches=0),
                     lambda r, c: setattr(r.owner, "sensor_fault", "sample_gap"))
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append({"travel_mm": 51.175})
                rig.capture_mutator = lambda capture: mutation(rig, capture)
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(rig.toolhead.moves, [])
                self.assert_no_calibration(rig)

    def test_pressed_or_impact_with_non_early_halt_is_not_recovered(self):
        for failure in ({"result": "pressed"}, {"travel_mm": 51., "halt_travel_mm_signed": -140.},
                        {"travel_mm": 51., "halt_travel_mm_signed": -50.},
                        {"travel_mm": 51., "trigger_travel_mm_signed": 51.},
                        {"result": "no_data", "travel_mm": 51.}):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append(failure)
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(rig.toolhead.moves, [])
                self.assert_no_calibration(rig)

    def test_zero_step_pressed_and_impact_preserve_start_without_a_return_move(self):
        for result in ("pressed", "impact"):
            with self.subTest(result=result), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.extend([{"result": result, "travel_mm": 0.}] + [{}] * 11)
                def no_pressed_onset(capture):
                    if rig.capture_stops == 1 and result == "pressed":
                        capture["_record"].update(onset_g=None, impact_peak_g=None, onset_time_s=None)
                        capture["detector"].update(onset_g=None, impact_peak_g=None, onset_time_s=None,
                                                   trigger_onset_time_s=None)
                rig.capture_mutator = no_pressed_onset
                rig.accuracy()
                # The first move is the discovery home's ordinary midpoint
                # return; the rejected zero-step trigger caused only settling.
                self.assertEqual(rig.toolhead.moves[0][0], [150., None, None])
                self.assertEqual(rig.workflow.last_result["accuracy_recovery"]["return_mm_signed"], 0.)
                self.assertEqual(rig.workflow.last_result["threshold_g"], 2.5)
                self.assertEqual(len(rig.home_calls), 12)

    def test_summary_and_temporary_cleanup_finish_before_calibration_noise_starts(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.extend([{"travel_mm": 51.175}] + [{}] * 11)
            def inspect_noise(_iteration):
                partial = rig.workflow.session["source_accuracy"]
                self.assertEqual(partial["result"], "aborted")
                self.assertTrue(Path(partial["summary_paths"]["x_png"]).is_file())
                self.assertTrue(all(not path.exists() for path in rig.capture_directories))
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.idle_state, "Printing")
            rig.on_noise = inspect_noise
            rig.accuracy()

    def test_cancel_shutdown_motor_off_and_printing_during_summary_block_handoff(self):
        for ending in ("cancel", "shutdown", "motor-off", "printing", "pause", "sd-active"):
            with self.subTest(ending=ending), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append({"travel_mm": 51.175})
                def interrupt(payload):
                    if payload["kind"] != "accuracy":
                        return
                    if ending == "cancel":
                        rig.workflow.cmd_CANCEL(GCmd())
                    elif ending == "shutdown":
                        rig.shutdown = True
                        rig.workflow._shutdown()
                    elif ending == "motor-off":
                        rig.owner.operation_generation += 1
                    elif ending == "pause":
                        rig.paused = True
                    elif ending == "sd-active":
                        rig.sd_active = True
                    else:
                        rig.print_state = "printing"
                rig.workflow.renderer.on_render = interrupt
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(len(rig.home_calls), 1)
                self.assert_no_calibration(rig)

    def test_cancel_and_external_motion_during_temporary_cleanup_block_handoff(self):
        for ending in ("cancel", "move", "preflight", "source"):
            with self.subTest(ending=ending), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append({"travel_mm": 51.175})
                create = rig.workflow._accuracy_capture_directory
                def directory_with_interrupt():
                    temporary = create()
                    cleanup = temporary.cleanup
                    def interrupt():
                        cleanup()
                        self.assertIsNotNone(rig.workflow.session)
                        if ending == "cancel":
                            rig.workflow.cmd_CANCEL(GCmd())
                        elif ending == "move":
                            rig.toolhead.position[0] += 1.
                        elif ending == "preflight":
                            rig.workflow.preflight_generation += 1
                        else:
                            rig.owner.last["x"] = copy.copy(rig.owner.last["x"])
                    temporary.cleanup = interrupt
                    return temporary
                with mock.patch.object(rig.workflow, "_accuracy_capture_directory", side_effect=directory_with_interrupt), \
                        self.assertRaises(CommandError):
                    rig.accuracy()
                self.assert_no_calibration(rig)

    def test_summary_or_temp_cleanup_failure_blocks_calibration_without_losing_primary_error(self):
        for stage in ("summary", "cleanup", "motion-and-cleanup"):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append({"travel_mm": 51.175})
                create = rig.workflow._accuracy_capture_directory
                def failing_directory():
                    temporary = create()
                    cleanup = temporary.cleanup
                    def fail():
                        cleanup()
                        raise OSError("synthetic temporary cleanup failure")
                    temporary.cleanup = fail
                    return temporary
                patch = (mock.patch.object(workflow_module, "write_accuracy", side_effect=OSError("summary fault"))
                         if stage == "summary" else
                         mock.patch.object(rig.workflow, "_accuracy_capture_directory", side_effect=failing_directory))
                if stage == "motion-and-cleanup":
                    rig.home_results.clear()
                    rig.home_results.append(CommandError("original homing fault"))
                with patch, self.assertRaises(CommandError) as caught:
                    rig.accuracy()
                if stage == "motion-and-cleanup":
                    self.assertIn("original homing fault", str(caught.exception))
                self.assert_no_calibration(rig)

    def test_cancel_during_calibration_command_creation_blocks_first_calibration_home(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.append({"travel_mm": 51.175})
            create = rig.gcode.create_gcode_command
            def cancelling(command, commandline, params):
                if command == "ACCEL_HOMING_CALIBRATE":
                    rig.workflow.cmd_CANCEL(GCmd())
                return create(command, commandline, params)
            with mock.patch.object(rig.gcode, "create_gcode_command", side_effect=cancelling), \
                    self.assertRaises(CommandError):
                rig.accuracy()
            self.assert_no_calibration(rig)

    def test_cancel_during_return_acceleration_flush_prevents_return_and_handoff(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.append({"travel_mm": 51.175})
            limit = rig.owner._limit_accel
            def cancelled():
                limit()
                rig.workflow.cmd_CANCEL(GCmd())
            with mock.patch.object(rig.owner, "_limit_accel", side_effect=cancelled), \
                    self.assertRaises(CommandError):
                rig.accuracy()
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.toolhead.accel, 50000.)
            self.assert_no_calibration(rig)

    def test_fresh_noise_failure_or_cancel_keeps_model_unstaged(self):
        for failure in ("sensor", "cancel", "busy"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = self.rig(directory)
                rig.home_results.append({"travel_mm": 51.175})
                def fail(_iteration):
                    if failure == "cancel":
                        rig.workflow.cmd_CANCEL(GCmd())
                    elif failure == "busy":
                        rig.toolhead.busy = (20., 10., False)
                    else:
                        raise CommandError("fresh stationary sensor fault")
                rig.on_noise = fail
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertIsNone(rig.workflow.session)
                self.assertIn("source_accuracy", rig.workflow.last_result)

    def test_calibration_sensor_failure_does_not_save_or_start_another_accuracy(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.extend([{"travel_mm": 51.175}, {"result": "no_data"}])
            with self.assertRaisesRegex(CommandError, "sensor/homing failure"):
                rig.accuracy()
            self.assertEqual(len(rig.home_calls), 2)
            self.assertEqual(rig.configfile.saved, {})
            self.assertNotIn("SAVE_CONFIG", rig.scripts)
            self.assertEqual(rig.workflow.last_result["result"], "aborted")
            self.assertEqual(rig.owner.threshold_g["x"], 2.25)

    def test_terminal_cleanup_failure_is_aborted_before_save_and_preserves_original_error(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.extend([{"travel_mm": 51.175}] + [{}] * 11)
            original = rig.owner._end_session
            fault = OSError("synthetic terminal recovery restoration failure")
            def fail_after_staging():
                original()
                if rig.configfile.saved:
                    raise fault
            with mock.patch.object(rig.owner, "_end_session", side_effect=fail_after_staging), \
                    self.assertRaises(OSError) as caught:
                rig.accuracy()
            self.assertIs(caught.exception, fault)
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            self.assertFalse(result["save_config"])
            self.assertIn("cleanup failed", result["reason"])
            self.assertIn("source_accuracy", result)
            self.assertNotIn("SAVE_CONFIG", rig.scripts)
            self.assertFalse(any("After SAVE_CONFIG succeeds" in line for line in rig.output))
            self.assertIsNone(rig.workflow.session)

    def test_recovery_calibration_keeps_full_repeatability_candidate_restarts(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = self.rig(directory)
            rig.home_results.extend([{"travel_mm": 51.175}, {}, {"travel_mm": 150.},
                                     {"travel_mm": 152.0001}] + [{}] * 11)
            rig.accuracy()
            result = rig.workflow.last_result
            self.assertEqual([item["threshold"] for item in rig.home_calls], [2.25] + [2.5] * 3 + [2.75] * 11)
            self.assertEqual(result["successes"], 10)
            self.assertEqual(result["threshold_g"], 2.75)
            self.assertEqual(result["restarts"], 1)
            self.assertEqual(result["repeatability_spread_mm"], 0.)
            self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)


if __name__ == "__main__":
    unittest.main()
