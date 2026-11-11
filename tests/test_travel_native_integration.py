"""Offline integration with pinned Klipper homing bookkeeping, not hardware.

HomingMove, StepperPosition, multi_complete and their constants below are a
verbatim GPLv3 excerpt from Klipper3d/klipper at
af81d23fec795ba64882d2206bc9cbc019e4501b, klippy/extras/homing.py:
https://github.com/Klipper3d/klipper/blob/af81d23fec795ba64882d2206bc9cbc019e4501b/klippy/extras/homing.py

The CoreXY calc_position method is copied from that revision's
klippy/kinematics/corexy.py (Copyright 2017-2021 Kevin O'Connor, GPLv3).

This runs the actual native trigger/halt conversion and event lifecycle without
a network dependency. MCU motion, sensor detections and toolhead queueing remain
explicit stubs. It does not validate real MCU synchronization, impact detection,
physical motion, or every Kalico-specific extension.
"""
import copy
import unittest
from unittest import mock

from harness import CommandError
from test_center_calibration import NativeState
from test_travel_calibration import TravelRig
from test_workflows import BACK_OFF, Kin, Stepper, Toolhead, workflow_module


# BEGIN pinned upstream homing excerpt (unchanged).
# Helper code for implementing homing operations
#
# Copyright (C) 2016-2026  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0)
    return cp

# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None
    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)
    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            logging.warning(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(), self.start_pos, new_start_pos)

# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        self.printer = printer
        self.endstops = [es for es in endstops if es[0].get_steppers()]
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead = toolhead
        self.stepper_positions = []
    def get_mcu_endstops(self):
        return [es for es, name in self.endstops]
    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            return .001
        return move_t / max_steps
    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        cpos = kin.calc_position(kin_spos)
        return [cp if cp is not None else tp
                for cp, tp in zip(cpos, thpos[:3])] + thpos[3:]
    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True):
        # Notify start of homing/probing move
        self.printer.send_event("homing:homing_move_begin", self)
        # Note start location
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in self.endstops
                                   for s in es.get_steppers() ]
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        endstop_triggers = []
        for mcu_endstop, name in self.endstops:
            rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
            wait = mcu_endstop.home_start(print_time, ENDSTOP_SAMPLE_TIME,
                                          ENDSTOP_SAMPLE_COUNT, rest_time,
                                          triggered=triggered)
            endstop_triggers.append(wait)
        all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
        self.toolhead.dwell(HOMING_START_DELAY)
        # Issue move
        error = None
        try:
            self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)
        # Wait for endstops to trigger
        trigger_times = {}
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in self.endstops:
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
            except self.printer.command_error as e:
                if error is None:
                    error = "Error during homing %s: %s" % (name, str(e))
                continue
            if trigger_time > 0.:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = "No trigger on %s after full movement" % (name,)
        # Determine stepper halt positions
        self.toolhead.flush_step_generation()
        for sp in self.stepper_positions:
            tt = trigger_times.get(sp.endstop_name, move_end_print_time)
            sp.note_home_end(tt)
        if probe_pos:
            halt_steps = {sp.stepper_name: sp.halt_pos - sp.start_pos
                          for sp in self.stepper_positions}
            trig_steps = {sp.stepper_name: sp.trig_pos - sp.start_pos
                          for sp in self.stepper_positions}
            haltpos = trigpos = self.calc_toolhead_pos(kin_spos, trig_steps)
            if trig_steps != halt_steps:
                haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
            self.toolhead.set_position(haltpos)
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            haltpos = trigpos = movepos
            over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                          for sp in self.stepper_positions}
            if any(over_steps.values()):
                self.toolhead.set_position(movepos)
                halt_kin_spos = {s.get_name(): s.get_commanded_position()
                                 for s in kin.get_steppers()}
                haltpos = self.calc_toolhead_pos(halt_kin_spos, over_steps)
            self.toolhead.set_position(haltpos)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end", self)
        except self.printer.command_error as e:
            if error is None:
                error = str(e)
        if error is not None:
            raise self.printer.command_error(error)
        return trigpos
    def check_no_movement(self):
        if self.printer.get_start_args().get('debuginput') is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None

# END pinned upstream homing excerpt.


# Only construction and axis selection are needed from the native state object.
# Rail acceleration is absent in these fixtures; Kalico's helper is not tested.
Homing = NativeState


def native_g28_namespace(self, gcmd):
    # _native_classes must discover the classes through the actual native
    # handler namespace. The wizard must never execute G28 itself.
    raise AssertionError("travel wizard unexpectedly invoked G28")


class NativeStepper(Stepper):
    def __init__(self, name, axes, toolhead):
        super().__init__(name, axes)
        self.toolhead = toolhead
        self.trigger_position = 0

    def get_step_dist(self):
        return 0.0025

    def calc_position_from_coord(self, position):
        if self.name == "stepper_x":
            return position[0] + position[1]
        if self.name == "stepper_y":
            return position[0] - position[1]
        return position[2]

    def get_commanded_position(self):
        return self.calc_position_from_coord(self.toolhead.get_position())

    def get_mcu_position(self, commanded_position=None):
        if commanded_position is None:
            return self.mcu_position
        return self.mcu_position + round(
            (commanded_position - self.get_commanded_position()) / self.get_step_dist())

    def mcu_to_commanded_position(self, position):
        return self.get_commanded_position() + (
            position - self.mcu_position) * self.get_step_dist()

    def get_past_mcu_position(self, trigger_time):
        return self.trigger_position


class NativeCoreXY(Kin):
    def __init__(self, toolhead):
        super().__init__()
        self.steppers = [NativeStepper("stepper_x", "xy", toolhead),
                         NativeStepper("stepper_y", "xy", toolhead),
                         NativeStepper("stepper_z", "z", toolhead),
                         NativeStepper("stepper_z1", "z", toolhead)]
        self.rails = self.steppers[:3]

    # Verbatim upstream CoreXY conversion; the surrounding hardware is stubbed.
    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]


class NativeToolhead(Toolhead):
    def __init__(self, rig):
        super().__init__(rig)
        self.kin = NativeCoreXY(self)

    def drip_move(self, target, speed, completion):
        rig = self.rig
        axis = rig.workflow.session["axis"]
        index = "xy".index(axis)
        start = self.get_position()
        direction = 1 if target[index] > start[index] else -1
        item = rig.measure_results.popleft()
        rig.active_measurement = item
        distance = item["distance"]
        if not item.get("triggered", True):
            distance = abs(target[index] - start[index])
        trigger = direction * distance
        halt = trigger + direction * item.get("overshoot", 0.25)
        item["signed_trigger"], item["signed_halt"] = trigger, halt
        before = {s.name: s.get_mcu_position() for s in self.kin.steppers}
        for stepper in self.kin.steppers:
            factor = (1 if stepper.name == "stepper_x" or axis == "x" else -1)
            if not stepper.is_active_axis(axis):
                stepper.trigger_position = stepper.mcu_position
                continue
            stepper.trigger_position = stepper.mcu_position + round(
                trigger * factor / stepper.get_step_dist())
            stepper.mcu_position += round(halt * factor / stepper.get_step_dist())
        if item.get("cross_axis_steps"):
            self.kin.steppers[0].trigger_position += item["cross_axis_steps"]
            self.kin.steppers[0].mcu_position += item["cross_axis_steps"]
        # The planned coordinate is corrected by the actual HomingMove after
        # it reads MCU trigger and halt positions.
        self.position = list(target)
        rig.native_moves.append({
            "axis": axis, "target": list(target), "speed": speed,
            "before": before,
            "halt": {s.name: s.get_mcu_position() for s in self.kin.steppers}})
        rig.idle_state = "Printing"


class NativeTravelRig(TravelRig):
    def __init__(self):
        super().__init__()
        # Undo only TravelRig's shortcut: production lookup must run.
        del self.workflow.travel._native_classes
        self.toolhead = NativeToolhead(self)
        self.owner.toolhead = self.toolhead
        self.objects["toolhead"] = self.toolhead
        self.native_moves, self.native_events, self.endstop_calls = [], [], []
        self.active_measurement = None
        self.fail_end_event = False
        for axis in "xy":
            self.set_rail(axis)
            endstop = self.owner.endstops[axis]
            endstop.home_start = self.home_start
            endstop.home_wait = self.home_wait

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        self.endstop_calls.append(("start", sample_time, sample_count, rest_time, triggered))
        return object()

    def home_wait(self, move_end_print_time):
        self.endstop_calls.append(("wait", move_end_print_time))
        return 1.0 if self.active_measurement.get("triggered", True) else 0.0

    def send_event(self, event, move):
        axis = self.workflow.session["axis"]
        self.native_events.append({
            "event": event, "axis": axis,
            "position": self.toolhead.get_position(),
            "steps": [(sp.stepper_name, sp.start_pos, sp.trig_pos, sp.halt_pos)
                      for sp in move.stepper_positions]})
        if event == "homing:homing_move_end":
            item = self.active_measurement
            if item.get("triggered", True):
                # Sensor output is deliberately simulated independently of the
                # native StepperPosition conversion, and published only on END.
                self.owner.last[axis] = {
                    "axis": axis, "pass": 1, "result": "impact",
                    "impact_metric": ("xyz_magnitude" if self.owner._uses_vector_detector(axis)
                                      else "selected_axis"),
                    "polarity_policy": "absolute_after_reference_subtraction_v1",
                    "trigger_travel_mm_signed": item["signed_trigger"],
                    "halt_travel_mm_signed": item["signed_halt"],
                    "onset_g": 8.0, "impact_peak_g": 9.0}
            if self.fail_end_event:
                raise CommandError("synthetic sensor failure at native home_end")
        for callback in self.handlers[event]:
            callback(move)


class TravelNativeIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.native_handler = mock.patch.object(
            workflow_module, "_PRINTER_G28", native_g28_namespace)
        self.native_handler.start()
        self.addCleanup(self.native_handler.stop)

    def test_native_probe_returns_trigger_but_assigns_halt_for_both_corexy_axes(self):
        for axis in "xy":
            with self.subTest(axis=axis):
                rig = NativeTravelRig()
                rig.select(axis.upper())
                plan = rig.workflow.session["plans"][axis]
                self.assertIs(plan["move_class"], HomingMove)
                self.assertIsInstance(plan["state"], Homing)
                position = rig.toolhead.get_position()
                position["xy".index(axis)] = 0.0
                rig.toolhead.set_position(position)
                rig.measure_results.append({"distance": 73.0, "overshoot": 0.75})
                target = list(position)
                target["xy".index(axis)] = -plan["budget"]
                move = HomingMove(rig, plan["rail"].get_endstops())
                returned = move.homing_move(target, plan["speed"], probe_pos=True)
                index = "xy".index(axis)
                self.assertAlmostEqual(returned[index], -73.0)
                self.assertAlmostEqual(rig.toolhead.get_position()[index], -73.75)
                self.assertEqual(returned[2:], position[2:])
                deltas = {sp.stepper_name: sp.trig_pos - sp.start_pos
                          for sp in move.stepper_positions}
                self.assertEqual(deltas["stepper_x"], -29200)
                self.assertEqual(deltas["stepper_y"], -29200 if axis == "x" else 29200)
                self.assertEqual([e["event"] for e in rig.native_events],
                                 ["homing:homing_move_begin", "homing:homing_move_end"])
                self.assertEqual(rig.native_events[0]["steps"], [])
                self.assertAlmostEqual(rig.native_events[1]["position"][index], -73.75)

    def test_wizard_xy_uses_native_counts_across_rebases_and_preserves_x_origin(self):
        rig = NativeTravelRig()
        original_ze = rig.toolhead.get_position()[2:]
        rig.select("X", PROMPT=0)
        rig.search(first=46.0, second=300.0, negative_over=0.75, positive_over=0.5)
        item = rig.workflow.session["measurement"]
        self.assertAlmostEqual(item["span"], 299.25)
        self.assertAlmostEqual(item["coordinate"], 299.75)
        self.assertAlmostEqual(rig.native_events[1]["position"][0], -46.75)
        self.assertAlmostEqual(rig.native_events[3]["position"][0], 300.5)
        rig.origin()
        x_result = copy.deepcopy(rig.workflow.session["results"]["x"])
        self.assertAlmostEqual(x_result["proposed"]["position_min"], -289.75)
        self.assertAlmostEqual(x_result["proposed"]["position_max"], 9.5)
        rig.command("NEXT")
        x_center = rig.toolhead.get_position()[0]
        before_y = {s.name: s.get_mcu_position() for s in rig.toolhead.kin.steppers}
        rig.search(first=61.0, second=199.0, negative_over=0.25, positive_over=0.5)
        self.assertEqual(rig.native_moves[2]["before"], before_y)
        self.assertAlmostEqual(rig.toolhead.get_position()[0], x_center)
        self.assertAlmostEqual(rig.workflow.session["measurement"]["span"], 198.75)
        self.assertAlmostEqual(rig.workflow.session["measurement"]["coordinate"], 199.25)
        rig.origin()
        self.assertEqual(rig.workflow.session["results"]["x"], x_result)
        self.assertEqual([m["axis"] for m in rig.native_moves], ["x", "x", "y", "y"])
        for index, event in enumerate(rig.native_events):
            self.assertEqual(event["event"], "homing:homing_move_" + ("begin" if index % 2 == 0 else "end"))
        self.assertEqual(rig.toolhead.get_position()[2:], original_ze)
        self.assertEqual(rig.toolhead.kin.get_status(0)["homed_axes"], "z")
        rig.command("DONE")
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.workflow.last_result["result"], "travel_report")
        self.assertEqual(rig.configfile.set_calls, [])

    def test_native_missing_trigger_unwinds_without_accepting_endpoint(self):
        rig = NativeTravelRig()
        rig.select("X")
        rig.measure_results.append({"distance": 73.0, "overshoot": 0.0, "triggered": False})
        with self.assertRaisesRegex(CommandError, "No trigger"):
            rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertIsNone(rig.workflow.session)
        self.assertFalse(rig.owner.homing_active)
        self.assertEqual(rig.toolhead.accel, 50000.0)
        self.assertEqual(rig.owner.back_off_dist, BACK_OFF)
        self.assertEqual(rig.configfile.set_calls, [])
        self.assertEqual(len(rig.native_moves), 1)
        self.assertEqual(rig.native_events[-1]["event"], "homing:homing_move_end")

    def test_native_end_event_failure_discards_measurement(self):
        rig = NativeTravelRig()
        rig.select("X")
        rig.fail_end_event = True
        rig.measure_results.append({"distance": 73.0, "overshoot": 0.25})
        with self.assertRaisesRegex(CommandError, "synthetic sensor failure"):
            rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(len(rig.native_moves), 1)
        self.assertFalse(rig.owner.homing_active)
        self.assertEqual(rig.configfile.set_calls, [])

    def test_native_corexy_asymmetric_motor_evidence_is_rejected(self):
        rig = NativeTravelRig()
        rig.select("Y")
        rig.measure_results.append({"distance": 73.0, "overshoot": 0.25, "cross_axis_steps": 10})
        with self.assertRaisesRegex(CommandError, "cross-axis displacement"):
            rig.command("MEASURE", CENTERED=1, CLEAR=1)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(len(rig.native_moves), 1)
        self.assertEqual(rig.configfile.set_calls, [])


if __name__ == "__main__":
    unittest.main()
