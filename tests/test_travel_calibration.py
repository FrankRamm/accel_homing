"""Offline wizard safety/persistence tests. No printer or host services used."""
import copy
import json
import math
import types
import unittest
from unittest import mock

from harness import CommandError
from test_workflows import GCmd, Stepper, workflow_module
from test_center_calibration import CenterRig, NativeState, NativeMove


class TravelRig(CenterRig):
    def __init__(self):
        super().__init__()
        self.workflow.travel._native_classes = lambda: (NativeState, NativeMove)
        self.prompts = []
        self.gcode.respond_raw = self.prompts.append
        self.callbacks = {}
        for name, line in self.lines.items():
            self.callbacks[name] = []
            line.register_state_callback = self.callbacks[name].append

    def command(self, action=None, **params):
        if action is not None:
            params["ACTION"] = action
        self.workflow.travel.command(GCmd(**params))

    def select(self, axis="X", **params):
        self.command(**params)
        self.command("SELECT", AXIS=axis)

    def measure(self, axis="X", **params):
        self.select(axis, **params)
        self.search()

    def search(self, first=73.0, second=None, negative_over=.25, positive_over=.25):
        axis = self.workflow.session["axis"]
        self.measure_results.extend([{"distance": first, "overshoot": negative_over},
                                     {"distance": second or self.spans[axis], "overshoot": positive_over}])
        self.command("MEASURE", CENTERED=1, CLEAR=1)

    def origin(self):
        self.command("JOG", DISTANCE=-10)
        self.command("ORIGIN", CONFIRM=1)

    def disable_one(self, name="stepper_x"):
        self.lines[name].enabled = False
        for callback in self.callbacks[name]:
            callback(0., False)


class TravelCalibrationTests(unittest.TestCase):
    def test_start_has_no_motion_and_center_is_operator_confirmation(self):
        rig = TravelRig()
        original = rig.toolhead.get_position()
        rig.select()
        self.assertEqual(rig.measure_calls, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.toolhead.get_position(), original)
        with self.assertRaisesRegex(CommandError, "CENTERED=1"):
            rig.command("MEASURE", CLEAR=1)
        self.assertEqual(rig.measure_calls, [])
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.owner.back_off_dist, 5.)

    def test_negative_then_positive_uses_configured_speed_no_retract_or_z(self):
        rig = TravelRig()
        ze, zlimit = rig.toolhead.position[2:], rig.toolhead.kin.limits[2]
        rig.set_rail("x", positive=True, low=-17.)
        rig.measure()
        self.assertEqual([c["direction"] for c in rig.measure_calls], [-1, 1])
        self.assertTrue(all(c["speed"] == 100 and c["probe_pos"] and c["back_off_dist"] == 0
                            for c in rig.measure_calls))
        self.assertEqual([c["target"][0] for c in rig.measure_calls], [-320, 320])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.toolhead.position[2:], ze)
        self.assertEqual(rig.toolhead.kin.limits[2], zlimit)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")

    def test_trigger_halt_origin_formula_is_independent_of_initial_midpoint(self):
        for positive in (False, True):
            rig = TravelRig()
            rig.set_rail("x", positive=positive, low=42.)
            rig.select()
            rig.search(first=46., second=300., negative_over=.75, positive_over=.5)
            item = rig.workflow.session["measurement"]
            self.assertAlmostEqual(item["span"], 299.25)
            self.assertAlmostEqual(item["coordinate"], 299.75)
            rig.origin()
            p = rig.workflow.session["results"]["x"]["proposed"]
            self.assertAlmostEqual(p["position_min"], -289.75)
            self.assertAlmostEqual(p["position_max"], 9.5)
            self.assertEqual(p["position_endstop"], p["position_max"] if positive else p["position_min"])
            self.assertAlmostEqual(p["position_max"] - p["position_min"], 299.25)
            self.assertEqual(rig.configfile.set_calls, [])

    def test_jog_is_bounded_slow_selected_axis_and_restores_unhomed_limits(self):
        rig = TravelRig()
        rig.measure("Y", JOG_SPEED=6.)
        original = rig.toolhead.get_position()
        rig.command("JOG", DISTANCE=-10)
        self.assertEqual(rig.toolhead.moves[-1], ([None, 189., None], 6.))
        self.assertEqual(rig.toolhead.position[0], original[0])
        self.assertEqual(rig.toolhead.position[2:], original[2:])
        self.assertEqual(rig.toolhead.accel, 50000.)
        self.assertIn(500., rig.toolhead.accel_log)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")

    def test_cartesian_stepper_frame(self):
        rig = TravelRig()
        rig.toolhead.kin.steppers = [Stepper("stepper_x", "x"), Stepper("stepper_y", "y"),
                                    Stepper("stepper_z", "z")]
        rig.toolhead.kin.calc_position = lambda p: [p["stepper_x"], p["stepper_y"], p["stepper_z"]]
        rig.workflow.kinematics_name = "cartesian"
        rig.set_rail("x")
        rig.set_rail("y")
        rig.measure()
        rig.origin()
        self.assertAlmostEqual(rig.workflow.session["results"]["x"]["proposed"]["position_min"], -289.)

    def test_prompts_offer_only_reachable_jogs_and_reopen_same_session(self):
        rig = TravelRig()
        rig.measure()
        beginning = max(i for i, text in enumerate(rig.prompts) if "prompt_begin" in text)
        buttons = [text for text in rig.prompts[beginning:] if "prompt_button" in text]
        self.assertEqual(len(buttons), 1)
        self.assertIn("DISTANCE=-10", buttons[0])
        session = rig.workflow.session
        rig.command()
        self.assertIs(rig.workflow.session, session)
        self.assertEqual(len(rig.measure_calls), 2)
        self.assertTrue(any("Cancel|ACCEL_HOMING_CANCEL" in text for text in rig.prompts))

    def test_console_mode_works_without_prompt_emission(self):
        rig = TravelRig()
        rig.measure(PROMPT=0)
        rig.origin()
        rig.command("DONE")
        self.assertEqual(rig.prompts, [])
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.workflow.last_result["result"], "travel_report")

    def test_complete_xy_sequence_keeps_motors_and_x_proposal(self):
        rig = TravelRig()
        rig.measure()
        rig.origin()
        x = copy.deepcopy(rig.workflow.session["results"]["x"])
        rig.command("NEXT")
        self.assertEqual(rig.workflow.session["axis"], "y")
        self.assertEqual(rig.workflow.session["stage"], "prepare")
        self.assertEqual(rig.workflow.session["results"]["x"], x)
        self.assertTrue(all(line.enabled for line in rig.lines.values()))
        self.assertEqual(rig.disabled, [])
        self.assertAlmostEqual(rig.toolhead.position[0], 298.75 / 2.)
        # Every center segment is <=10 mm and no Z coordinate is submitted.
        positions = [289.] + [move[0][0] for move in rig.toolhead.moves[1:]]
        self.assertTrue(all(abs(b - a) <= 10 for a, b in zip(positions, positions[1:])))
        self.assertTrue(all(move[0][2] is None for move in rig.toolhead.moves))
        rig.search()
        rig.origin()
        rig.command("SAVE", CONFIRM=1)
        self.assertEqual(rig.configfile.native_calls, 1)
        self.assertEqual(set(rig.workflow.last_result["proposed_config"]), {"stepper_x", "stepper_y"})
        self.assertEqual(rig.workflow.last_result["persistence"], "native_save_requested")
        self.assertTrue(rig.workflow.last_result["save_config"])
        self.assertEqual(len(rig.configfile.set_calls), 6)
        self.assertEqual(rig.owner.back_off_dist, 5.)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")

    def test_model_metadata_rebase_preserves_calibration_provenance(self):
        rig = TravelRig()
        rig.model("x")
        original = copy.deepcopy(rig.workflow.model.metadata["x"])
        rig.measure()
        rig.origin()
        rig.command("SAVE", CONFIRM=1)
        metadata = json.loads(rig.configfile.saved[(rig.workflow.model_section, "x_metadata")])
        self.assertEqual(metadata["calibration_geometry"], original["geometry"])
        self.assertEqual(metadata["original_calibration_evidence"], "retained")
        self.assertFalse(metadata["travel_coordinate_rebase"]["threshold_calibration_rerun"])
        self.assertFalse(metadata["travel_coordinate_rebase"]["physical_position_verified"])
        p = rig.workflow.last_result["proposed_config"]["stepper_x"]
        self.assertEqual(metadata["geometry"]["minimum"], p["position_min"])
        self.assertEqual(metadata["geometry"]["maximum"], p["position_max"])
        self.assertEqual(rig.workflow.last_result["model_metadata_rebased"], ["x"])
        self.assertEqual(rig.workflow.model.metadata["x"], original)

    def test_source_include_pending_and_metadata_conflicts_prevent_writes(self):
        for change in ("source", "include_min", "include_model", "pending", "included_source", "live_model"):
            with self.subTest(change=change):
                rig = TravelRig()
                rig.model("x")
                rig.measure()
                rig.origin()
                if change == "source":
                    rig.configfile.source += "# external edit\n"
                elif change == "include_min":
                    rig.configfile.included = {"stepper_x": {"position_min": "0"}}
                elif change == "include_model":
                    rig.configfile.included = {rig.workflow.model_section: {"x_metadata": "{}"}}
                elif change == "pending":
                    rig.configfile.save_config_pending = True
                elif change == "included_source":
                    rig.configfile.included = {"unrelated": {"value": "changed"}}
                else:
                    rig.workflow.model.metadata["x"]["external_change"] = True
                with self.assertRaises(CommandError):
                    rig.command("SAVE", CONFIRM=1)
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.configfile.native_calls, 0)
                self.assertIsNone(rig.workflow.session)

    def test_save_staging_and_native_failures_roll_back_pending_and_report_failure(self):
        for failure in ("stage", "native"):
            rig = TravelRig()
            rig.measure()
            rig.origin()
            if failure == "stage":
                rig.configfile.fail_set_at = 2
            else:
                rig.configfile.save_error = CommandError("native disk failure")
            with self.assertRaises(CommandError):
                rig.command("SAVE", CONFIRM=1)
            self.assertEqual(rig.configfile.saved, {})
            self.assertFalse(rig.configfile.save_config_pending)
            self.assertEqual(rig.configfile.status_save_pending, {})
            self.assertEqual(rig.workflow.last_result["result"], "save_failed")
            self.assertIn("unconfirmed", rig.workflow.last_result["persistence"])

    def test_faults_before_second_search_abort_without_moves_or_saves(self):
        for item in ({"distance": 20.}, {"distance": 500.}, {"distance": 100., "result": "pressed"},
                     {"distance": 100., "drop_stepper": True}, CommandError("sensor fault")):
            rig = TravelRig()
            rig.select()
            rig.measure_results.append(item)
            with self.assertRaises(CommandError):
                rig.command("MEASURE", CENTERED=1, CLEAR=1)
            self.assertEqual(len(rig.measure_calls), 1)
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.configfile.set_calls, [])
            self.assertIsNone(rig.workflow.session)

    def test_external_position_stepper_settings_and_homing_changes_invalidate(self):
        for change in ("position", "force_move", "z_stepper", "limit", "speed", "threshold", "generation"):
            with self.subTest(change=change):
                rig = TravelRig()
                rig.measure()
                if change == "position":
                    rig.toolhead.position[0] += .01
                elif change in ("force_move", "z_stepper"):
                    rig.toolhead.kin.steppers[2 if change == "z_stepper" else 0].mcu_position += 1
                elif change == "limit":
                    rig.toolhead.kin.limits[1] = (0., 200.)
                elif change == "speed":
                    rig.owner.rails_by_axis["x"].info.speed = 90.
                elif change == "threshold":
                    rig.owner.threshold_g["x"] += .1
                else:
                    rig.owner.operation_generation += 1
                with self.assertRaises(CommandError):
                    rig.command("JOG", DISTANCE=-10)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.workflow.last_result["proposed_config"], {})

    def test_individual_motor_disable_cannot_be_hidden_by_reenable(self):
        for name in ("stepper_x", "stepper_z", "stepper_z1"):
            rig = TravelRig()
            rig.measure()
            rig.disable_one(name)
            rig.lines[name].enabled = True
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.workflow.last_result["result"], "aborted")
            with self.assertRaises(CommandError):
                rig.command("JOG", DISTANCE=-10)

    def test_stale_next_after_both_axes_does_not_move(self):
        rig = TravelRig()
        rig.measure()
        rig.origin()
        rig.command("NEXT")
        rig.search()
        rig.origin()
        moves = len(rig.toolhead.moves)
        with self.assertRaisesRegex(CommandError, "both axes"):
            rig.command("NEXT")
        self.assertEqual(len(rig.toolhead.moves), moves)

    def test_cancel_timeout_command_error_shutdown_disconnect_and_external_home(self):
        for event in ("cancel", "timeout", "error", "shutdown", "disconnect", "home"):
            with self.subTest(event=event):
                rig = TravelRig()
                rig.measure()
                if event == "cancel":
                    rig.workflow.cmd_CANCEL(GCmd())
                elif event == "timeout":
                    rig.workflow._timeout(rig.workflow.session["expires"])
                elif event == "error":
                    rig.workflow.travel._command_error()
                elif event == "shutdown":
                    rig.workflow._shutdown()
                elif event == "disconnect":
                    rig.workflow._disconnect()
                else:
                    with self.assertRaises(CommandError):
                        rig.workflow.travel._external_home(None, [])
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.owner.back_off_dist, 5.)

    def test_cancel_during_search_or_first_recenter_chunk_stops_progression(self):
        rig = TravelRig()
        rig.select()
        rig.on_measure = lambda axis, phase: rig.workflow.cmd_CANCEL(GCmd())
        with self.assertRaises(CommandError):
            rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertEqual(len(rig.measure_calls), 1)
        self.assertIsNone(rig.workflow.session)
        rig = TravelRig()
        rig.measure()
        rig.origin()
        original_move = rig.toolhead.manual_move
        def cancel_after_chunk(coords, speed):
            original_move(coords, speed)
            rig.workflow.cmd_CANCEL(GCmd())
        rig.toolhead.manual_move = cancel_after_chunk
        with self.assertRaises(CommandError):
            rig.command("NEXT")
        self.assertEqual(len(rig.toolhead.moves), 2)  # origin jog + one recenter chunk
        self.assertEqual(len(rig.measure_calls), 2)  # Y was never measured
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.toolhead.accel, 50000.)

    def test_failed_jog_does_not_accept_origin_or_continue(self):
        rig = TravelRig()
        rig.measure()
        def failed(coords, speed):
            raise CommandError("move rejected")
        rig.toolhead.manual_move = failed
        with self.assertRaisesRegex(CommandError, "move rejected"):
            rig.command("JOG", DISTANCE=-10)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.toolhead.accel, 50000.)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
        self.assertEqual(rig.configfile.set_calls, [])

    def test_disable_during_save_cleanup_prevents_staging(self):
        rig = TravelRig()
        rig.measure()
        rig.origin()
        original = rig.toolhead.wait_moves
        def invalidate():
            original()
            rig.disable_one()
        rig.toolhead.wait_moves = invalidate
        with self.assertRaises(CommandError):
            rig.command("SAVE", CONFIRM=1)
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.configfile.native_calls, 0)

    def test_second_interrupt_during_idle_cleanup_does_not_reenter_cleanup(self):
        rig = TravelRig()
        rig.measure()
        original = rig.toolhead.wait_moves
        waits = []
        def invalidate():
            waits.append(True)
            original()
            rig.disable_one()
        rig.toolhead.wait_moves = invalidate
        rig.workflow.cmd_CANCEL(GCmd())
        self.assertEqual(len(waits), 1)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.configfile.set_calls, [])

    def test_missing_new_module_keeps_ordinary_workflows_registered(self):
        with mock.patch.object(workflow_module, "TravelCalibration", None):
            rig = CenterRig()
        self.assertIsNone(rig.workflow.travel)
        self.assertIn("ACCEL_HOMING_CALIBRATE", rig.gcode.ready_gcode_handlers)
        self.assertIn("ACCEL_HOMING_CALIBRATE_CENTER", rig.gcode.ready_gcode_handlers)
        with self.assertRaisesRegex(CommandError, "rerun the dev installer"):
            rig.gcode.ready_gcode_handlers["ACCEL_HOMING_TRAVEL"](GCmd())


if __name__ == "__main__":
    unittest.main()
