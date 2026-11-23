"""Offline wizard safety/persistence tests. No printer or host services used."""
import copy
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError
from test_workflows import BACK_OFF, GCmd, Stepper, workflow_module
from axis_limits_rig import AxisLimitsFixture, NativeState, NativeMove
from test_calibration_config import ModernNativeShape


class AxisLimitsRig(AxisLimitsFixture):
    def __init__(self):
        super().__init__()
        self.workflow.axis_limits._native_classes = lambda: (NativeState, NativeMove)
        self.prompts = []
        self.gcode.respond_raw = self.prompts.append
        self.callbacks = {}
        for name, line in self.lines.items():
            self.callbacks[name] = []
            line.register_state_callback = self.callbacks[name].append

    def command(self, action=None, **params):
        if action is not None:
            params["ACTION"] = action
        self.workflow.axis_limits.command(GCmd(**params))

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


class ModernConfigfile(ModernNativeShape):
    """Newer layout: pending state and autosave helpers live on autosave."""
    def __init__(self, root):
        super().__init__(root)
        self.set_calls, self.native_calls = [], 0

    def set(self, section, option, value):
        self.set_calls.append((section, option, value))
        super().set(section, option, value)

    def cmd_SAVE_CONFIG(self, gcmd):
        self.native_calls += 1
        return self.save(gcmd)


class ModernLayoutRig(AxisLimitsRig):
    def __init__(self, directory):
        super().__init__()
        self.root = Path(directory) / "printer.cfg"
        self.root.write_text("[stepper_x]\nposition_max: 300\nposition_endstop: 0\n"
                             "[stepper_y]\nposition_max: 200\nposition_endstop: 0\n")
        self.configfile = ModernConfigfile(self.root)
        self.objects["configfile"] = self.configfile
        self.get_start_args = lambda: {"config_file": str(self.root)}


class AxisLimitsCalibrationTests(unittest.TestCase):
    def test_start_has_no_motion_and_center_is_operator_confirmation(self):
        rig = AxisLimitsRig()
        original = rig.toolhead.get_position()
        rig.select()
        self.assertEqual(rig.measure_calls, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.toolhead.get_position(), original)
        with self.assertRaisesRegex(CommandError, "CENTERED=1"):
            rig.command("MEASURE", CLEAR=1)
        self.assertEqual(rig.measure_calls, [])
        self.assertEqual(rig.workflow.session["stage"], "prepare")
        rig.workflow.cmd_CANCEL(GCmd())
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.owner.back_off_dist, BACK_OFF)

    def test_negative_then_positive_uses_configured_speed_no_retract_or_z(self):
        rig = AxisLimitsRig()
        ze, zlimit = rig.toolhead.position[2:], rig.toolhead.kin.limits[2]
        rig.set_rail("x", positive=True, low=-17.)
        rig.measure()
        self.assertEqual([c["direction"] for c in rig.measure_calls], [-1, 1])
        self.assertTrue(all(c["speed"] == 100 and c["probe_pos"] and not any(c["back_off_dist"].values())
                            for c in rig.measure_calls))
        self.assertEqual([c["target"][0] for c in rig.measure_calls], [-320, 320])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.toolhead.position[2:], ze)
        self.assertEqual(rig.toolhead.kin.limits[2], zlimit)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")

    def test_trigger_halt_origin_formula_is_independent_of_initial_midpoint(self):
        for positive in (False, True):
            rig = AxisLimitsRig()
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
        rig = AxisLimitsRig()
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
        rig = AxisLimitsRig()
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
        rig = AxisLimitsRig()
        rig.measure()
        beginning = max(i for i, text in enumerate(rig.prompts) if "prompt_begin" in text)
        last = rig.prompts[beginning:]
        item = rig.workflow.session["measurement"]
        coordinate, span = item["coordinate"], item["span"]
        self.assertIn("// action:prompt_text X %.2f mm (measured span 0 -> %.2f mm)" % (coordinate, span),
                      last)
        distances, inside, pairs, count = [], False, 0, 0
        for text in last:
            if "prompt_button_group_start" in text:
                inside, count = True, 0
            elif "prompt_button_group_end" in text:
                inside = False
                self.assertGreater(count, 0)
                pairs += 1
            elif inside and "prompt_button " in text:
                count += 1
                distances.append(float(text.split("DISTANCE=")[1].split("|")[0]))
        self.assertGreaterEqual(pairs, 1)
        self.assertIn(-100.0, distances)
        self.assertIn(-10.0, distances)
        self.assertTrue(all(1.0 <= coordinate + d <= span - 1.0 for d in distances))
        self.assertTrue(any("Abort|ACCEL_HOMING_CANCEL|error" in text for text in last))
        self.assertFalse(any("Cancel|ACCEL_HOMING_CANCEL" in text for text in last))
        session = rig.workflow.session
        rig.command()
        self.assertIs(rig.workflow.session, session)
        self.assertEqual(len(rig.measure_calls), 2)

    def test_refresh_keeps_dialog_open(self):
        rig = AxisLimitsRig()
        rig.select()
        rig.search()
        self.assertIsNotNone(rig.workflow.session)
        self.assertFalse(any("action:prompt_end" in text for text in rig.prompts))
        self.assertGreaterEqual(sum("prompt_begin" in text for text in rig.prompts), 2)
        begins = sum("prompt_begin" in text for text in rig.prompts)
        rig.command("JOG", DISTANCE=-10)
        self.assertIsNotNone(rig.workflow.session)
        self.assertFalse(any("action:prompt_end" in text for text in rig.prompts))
        self.assertGreater(sum("prompt_begin" in text for text in rig.prompts), begins)

    def test_session_end_closes_dialog(self):
        rig = AxisLimitsRig()
        rig.measure()
        rig.workflow.cmd_CANCEL(GCmd())
        self.assertIsNone(rig.workflow.session)
        last = [text for text in rig.prompts if "action:" in text and "prompt_" in text][-1]
        self.assertEqual(last, "// action:prompt_end")

    def test_jog_distance_limit(self):
        rig = AxisLimitsRig()
        rig.measure()
        before = rig.workflow.session["measurement"]["coordinate"]
        rig.command("JOG", DISTANCE=-100)
        self.assertAlmostEqual(rig.workflow.session["measurement"]["coordinate"], before - 100)
        moves, position = len(rig.toolhead.moves), list(rig.toolhead.position)
        session = rig.workflow.session
        with self.assertRaises(CommandError):
            rig.command("JOG", DISTANCE=100.5)
        self.assertEqual(len(rig.toolhead.moves), moves)
        self.assertEqual(rig.toolhead.position, position)
        # A rejected command keeps the session, stage and measured coordinate.
        self.assertIs(rig.workflow.session, session)
        self.assertEqual(session["stage"], "origin")
        self.assertAlmostEqual(session["measurement"]["coordinate"], before - 100)

    def test_rejected_commands_keep_session_stage_and_show_dialog_again(self):
        cases = [("select", "ORIGIN", {"CONFIRM": 1}), ("select", "JOG", {"DISTANCE": -10}),
                 ("select", "MEASURE", {"CENTERED": 1, "CLEAR": 1}), ("select", "SAVE", {"CONFIRM": 1}),
                 ("select", "SELECT", {"AXIS": "Z"}), ("select", "BOGUS", {}),
                 ("prepare", "ORIGIN", {"CONFIRM": 1}), ("prepare", "SELECT", {"AXIS": "Y"}),
                 ("prepare", "MEASURE", {"CENTERED": 1}), ("prepare", "MEASURE", {"CLEAR": 1}),
                 ("origin", "MEASURE", {"CENTERED": 1, "CLEAR": 1}), ("origin", "ORIGIN", {}),
                 ("origin", "JOG", {"DISTANCE": 0}), ("origin", "JOG", {"DISTANCE": 300}),
                 ("origin", "JOG", {"DISTANCE": 10}), ("origin", "SAVE", {"CONFIRM": 1}),
                 ("review", "SAVE", {}), ("review", "JOG", {"DISTANCE": -10}),
                 ("review", "ORIGIN", {"CONFIRM": 1})]
        for stage, action, params in cases:
            with self.subTest(stage=stage, action=action, params=params):
                rig = AxisLimitsRig()
                if stage == "select":
                    rig.command()
                elif stage == "prepare":
                    rig.select()
                else:
                    rig.measure()
                    if stage == "review":
                        rig.origin()
                session, moves = rig.workflow.session, len(rig.toolhead.moves)
                calls, prompts = len(rig.measure_calls), len(rig.prompts)
                coordinate = session.get("measurement", {}).get("coordinate")
                with self.assertRaises(CommandError):
                    rig.command(action, **params)
                self.assertIs(rig.workflow.session, session)
                self.assertEqual(session["stage"], stage)
                self.assertEqual(len(rig.toolhead.moves), moves)
                self.assertEqual(len(rig.measure_calls), calls)
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(session.get("measurement", {}).get("coordinate"), coordinate)
                self.assertGreater(len(rig.prompts), prompts)
                self.assertNotIn("// action:prompt_end", rig.prompts)

    def test_selected_axis_impact_without_motion_is_rejected(self):
        rig = AxisLimitsRig()
        rig.select()
        rig.measure_results.append({"distance": 73., "motion_seen": False})
        with self.assertRaises(CommandError):
            rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertEqual(len(rig.measure_calls), 1)
        self.assertIsNone(rig.workflow.session)

    def test_mapped_impacts_do_not_need_motion_like_ordinary_homing(self):
        rig = AxisLimitsRig()
        rig.mapped_axes.add("x")
        rig.select()
        rig.measure_results.extend([{"distance": 73., "motion_seen": False},
                                    {"distance": 299., "motion_seen": False}])
        rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertEqual(rig.workflow.session["stage"], "origin")

    def test_bed_zero_button_only_when_coordinate_is_valid(self):
        rig = AxisLimitsRig()
        rig.measure()
        accept = "Accept bed zero|ACCEL_HOMING_CALIBRATE_LIMITS ACTION=ORIGIN CONFIRM=1|primary"
        item = rig.workflow.session["measurement"]
        for coordinate, shown in ((item["coordinate"], False), (0.5, False), (item["span"] - 0.5, False),
                                  (1.0, True), (item["span"] - 1.0, True)):
            with self.subTest(coordinate=coordinate):
                item["coordinate"] = coordinate
                rig.prompts.clear()
                rig.workflow.axis_limits._show(rig.workflow.session)
                self.assertEqual(any(accept in text for text in rig.prompts), shown)

    def test_newer_layout_snapshots_and_saves_through_native_save(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = ModernLayoutRig(directory)
            rig.measure()
            self.assertIsNotNone(rig.workflow.session["configuration"])
            rig.origin()
            rig.command("SAVE", CONFIRM=1)
            self.assertEqual(rig.configfile.native_calls, 1)
            self.assertEqual(len(rig.configfile.set_calls), 3)
            self.assertEqual(rig.workflow.last_result["result"], "axis_limits_report")
            self.assertEqual(rig.workflow.last_result["persistence"], "native_save_requested")
            self.assertTrue(rig.workflow.last_result["save_config"])
            self.assertIn("position_min", rig.root.read_text().split("SAVE_CONFIG")[-1])

    def test_newer_layout_refusals_prevent_writes_and_keep_report(self):
        for change in ("source", "pending", "include"):
            with self.subTest(change=change):
                with tempfile.TemporaryDirectory() as directory:
                    rig = ModernLayoutRig(directory)
                    rig.measure()
                    rig.origin()
                    if change == "source":
                        rig.root.write_text(rig.root.read_text() + "# external edit\n")
                    elif change == "pending":
                        rig.configfile.autosave.save_config_pending = True
                    else:
                        (Path(directory) / "extra.cfg").write_text("[stepper_x]\nposition_min: 0\n")
                        rig.root.write_text("[include extra.cfg]\n" + rig.root.read_text())
                        rig.workflow.session["configuration"] = \
                            rig.workflow.axis_limits._configuration_state()
                    with self.assertRaises(CommandError):
                        rig.command("SAVE", CONFIRM=1)
                    self.assertEqual(rig.configfile.set_calls, [])
                    self.assertEqual(rig.configfile.native_calls, 0)
                    self.assertIn("stepper_x", rig.workflow.last_result["proposed_config"])

    def test_refused_save_keeps_and_prints_the_complete_report(self):
        rig = AxisLimitsRig()
        rig.measure()
        rig.origin()
        rig.configfile.included = {"stepper_x": {"position_min": "0"}}
        rig.output.clear()
        with self.assertRaises(CommandError):
            rig.command("SAVE", CONFIRM=1)
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "axis_limits_report")
        self.assertIn("position_min", result["proposed_config"]["stepper_x"])
        self.assertIn("x", result["axis_results"])
        self.assertFalse(result["save_config"])
        self.assertEqual(result["persistence"], "save_refused")
        self.assertIn("included options", result["reason"])
        text = "\n".join(rig.output)
        self.assertIn("position_min", text)
        self.assertIn("not saved", text)
        self.assertIsNone(rig.workflow.session)

    def test_every_stage_footer_says_abort(self):
        rig = AxisLimitsRig()
        rig.command()
        rig.select()
        footers = [text for text in rig.prompts if "prompt_footer_button" in text]
        self.assertGreaterEqual(len(footers), 2)
        self.assertTrue(all(text == "// action:prompt_footer_button Abort|ACCEL_HOMING_CANCEL|error"
                            for text in footers))

    def test_console_mode_works_without_prompt_emission(self):
        rig = AxisLimitsRig()
        rig.measure(PROMPT=0)
        rig.origin()
        rig.command("DONE")
        self.assertEqual(rig.prompts, [])
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(rig.workflow.last_result["result"], "axis_limits_report")

    def test_complete_xy_sequence_keeps_motors_and_x_proposal(self):
        rig = AxisLimitsRig()
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
        self.assertEqual(rig.owner.back_off_dist, BACK_OFF)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")

    def test_model_metadata_rebase_preserves_calibration_provenance(self):
        rig = AxisLimitsRig()
        rig.model("x")
        original = copy.deepcopy(rig.workflow.model.metadata["x"])
        rig.measure()
        rig.origin()
        rig.command("SAVE", CONFIRM=1)
        metadata = json.loads(rig.configfile.saved[(rig.workflow.model_section, "x_metadata")])
        self.assertEqual(metadata["calibration_geometry"], original["geometry"])
        self.assertEqual(metadata["original_calibration_evidence"], "retained")
        self.assertFalse(metadata["axis_limits_rebase"]["threshold_calibration_rerun"])
        self.assertFalse(metadata["axis_limits_rebase"]["physical_position_verified"])
        p = rig.workflow.last_result["proposed_config"]["stepper_x"]
        self.assertEqual(metadata["geometry"]["minimum"], p["position_min"])
        self.assertEqual(metadata["geometry"]["maximum"], p["position_max"])
        self.assertEqual(rig.workflow.last_result["model_metadata_rebased"], ["x"])
        self.assertEqual(rig.workflow.model.metadata["x"], original)

    def test_source_include_pending_and_metadata_conflicts_prevent_writes(self):
        for change in ("source", "include_min", "include_model", "pending", "included_source", "live_model"):
            with self.subTest(change=change):
                rig = AxisLimitsRig()
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
            rig = AxisLimitsRig()
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
            rig = AxisLimitsRig()
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
                rig = AxisLimitsRig()
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
            rig = AxisLimitsRig()
            rig.measure()
            rig.disable_one(name)
            rig.lines[name].enabled = True
            self.assertIsNone(rig.workflow.session)
            self.assertEqual(rig.workflow.last_result["result"], "aborted")
            with self.assertRaises(CommandError):
                rig.command("JOG", DISTANCE=-10)

    def test_stale_next_after_both_axes_does_not_move(self):
        rig = AxisLimitsRig()
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
                rig = AxisLimitsRig()
                rig.measure()
                if event == "cancel":
                    rig.workflow.cmd_CANCEL(GCmd())
                elif event == "timeout":
                    rig.workflow._timeout(rig.workflow.session["expires"])
                elif event == "error":
                    rig.workflow.axis_limits._command_error()
                elif event == "shutdown":
                    rig.workflow._shutdown()
                elif event == "disconnect":
                    rig.workflow._disconnect()
                else:
                    with self.assertRaises(CommandError):
                        rig.workflow.axis_limits._external_home(None, [])
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.set_calls, [])
                self.assertEqual(rig.owner.back_off_dist, BACK_OFF)

    def test_cancel_during_search_or_first_recenter_chunk_stops_progression(self):
        rig = AxisLimitsRig()
        rig.select()
        rig.on_measure = lambda axis, phase: rig.workflow.cmd_CANCEL(GCmd())
        with self.assertRaises(CommandError):
            rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertEqual(len(rig.measure_calls), 1)
        self.assertIsNone(rig.workflow.session)
        rig = AxisLimitsRig()
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
        rig = AxisLimitsRig()
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
        rig = AxisLimitsRig()
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
        rig = AxisLimitsRig()
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
        with mock.patch.object(workflow_module, "AxisLimitsCalibration", None):
            rig = AxisLimitsFixture()
        self.assertIsNone(rig.workflow.axis_limits)
        self.assertIn("ACCEL_HOMING_CALIBRATE", rig.gcode.ready_gcode_handlers)
        with self.assertRaisesRegex(CommandError, "rerun the installer"):
            rig.gcode.ready_gcode_handlers["ACCEL_HOMING_CALIBRATE_LIMITS"](GCmd())


if __name__ == "__main__":
    unittest.main()
