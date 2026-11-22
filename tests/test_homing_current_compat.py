"""Offline native current API contracts; no printer connections or commands."""
import contextlib
import types
import unittest
from unittest import mock

from harness import CommandError
from test_axis_mapping_workflow import MappingRig
from test_beta_calibration import BetaRig
from test_workflows import workflow_module


class KalicoHoming:
    # These methods are copied from the pinned tester firmware, not inferred
    # from the plugin's adapter. Copyright (C) 2016-2024 Kevin O'Connor; GPLv3.
    # https://github.com/KalicoCrew/kalico/blob/4b15247f/klippy/extras/homing.py
    def __init__(self, printer, gcmd):
        self.printer = printer
        self.gcmd = gcmd
        self.toolhead = printer.lookup_object("toolhead")
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
        self.saved_accel = None
        self.homing_successful = False

    def _set_homing_current(self, homing_axes, pre_homing):
        print_time = self.toolhead.get_last_move_time()
        affected_rails = set()
        for axis in homing_axes:
            partial_rails = self.toolhead.get_active_rails_for_axis(axis)
            affected_rails = affected_rails | set(partial_rails)
        dwell_time = 0.0
        for rail in affected_rails:
            chs = rail.get_tmc_current_helpers()
            for ch in chs:
                if ch is not None:
                    current_dwell_time = ch.set_current_for_homing(
                        print_time, pre_homing
                    )
                    dwell_time = max(dwell_time, current_dwell_time)

        if dwell_time:
            self.toolhead.dwell(dwell_time)


class KalicoToolhead:
    # Same pinned revision; Copyright (C) 2016-2025 Kevin O'Connor; GPLv3.
    # https://github.com/KalicoCrew/kalico/blob/4b15247f/klippy/toolhead.py
    def get_active_rails_for_axis(self, axis):
        # axis is 'x,y,z'
        active_rails = []
        rails = self.kin.rails
        for rail in rails:
            for stepper in rail.get_steppers():
                if stepper.is_active_axis(axis):
                    active_rails.append(rail)
                    break
        return active_rails


class IndexedHoming(KalicoHoming):
    """Variant converts the helper's numeric axes before using the toolhead."""
    def _set_homing_current(self, homing_axes, pre_homing):
        return super()._set_homing_current(
            ["xyz"[axis] for axis in homing_axes], pre_homing)


class IndexedToolhead(KalicoToolhead):
    """Variant converts numeric axes in the toolhead instead of the helper."""
    def get_active_rails_for_axis(self, axis):
        return KalicoToolhead.get_active_rails_for_axis(self, "xyz"[axis])


class NativeRail:
    def __init__(self, names, current):
        self.names, self.current = names, current

    def get_steppers(self):
        # Real Kalico stepper.is_active_axis passes axis.encode() to its C
        # helper. An integer-only replacement would break this interface.
        return [types.SimpleNamespace(is_active_axis=lambda axis: axis.encode() in self.names)]

    def get_tmc_current_helpers(self):
        return [None, self.current]


@contextlib.contextmanager
def native_current(rig, homing=KalicoHoming, toolhead=KalicoToolhead, fail=None):
    calls, selections = [], []

    def set_current(print_time, pre_homing):
        calls.append((print_time, pre_homing))
        if fail is not None:
            fail(pre_homing)
        return .125 if pre_homing else .25

    current = types.SimpleNamespace(set_current_for_homing=set_current)
    selected_rail = NativeRail((b"x", b"y"), current)
    z_current = types.SimpleNamespace(set_current_for_homing=mock.Mock(
        side_effect=AssertionError("Z current changed")))
    # Duplicate references exercise the native affected_rails deduplication.
    rig.toolhead.kin.rails = [selected_rail, selected_rail, NativeRail((b"z",), z_current)]
    selector = types.MethodType(toolhead.get_active_rails_for_axis, rig.toolhead)

    def active_rails(axis):
        selections.append(axis)
        return selector(axis)

    with mock.patch.dict(workflow_module._PRINTER_G28.__globals__, {"Homing": homing}), \
            mock.patch.object(rig.toolhead, "get_active_rails_for_axis", active_rails, create=True):
        yield calls, selections
    z_current.set_current_for_homing.assert_not_called()


class HomingCurrentCompatibilityTests(unittest.TestCase):
    def test_pinned_and_indexed_apis_activate_and_restore_mapping_current(self):
        for axis in "xy":
            for homing, toolhead in ((KalicoHoming, KalicoToolhead),
                                     (IndexedHoming, KalicoToolhead),
                                     (KalicoHoming, IndexedToolhead)):
                with self.subTest(axis=axis, homing=homing.__name__, toolhead=toolhead.__name__):
                    rig = MappingRig(axis=axis)
                    with native_current(rig, homing, toolhead) as (calls, selections):
                        session = rig.begin_mapping()
                        rig.workflow._calibration_axis_mapping(session)
                    self.assertEqual([enabled for _, enabled in calls], [True, False])
                    self.assertEqual(selections, [axis, axis])
                    self.assertEqual(rig.toolhead.dwells,
                                     [.125, rig.workflow.calibration_settle_time,
                                      rig.workflow.calibration_settle_time, .25])
                    self.assertEqual(rig.events.count("capture"), 2)
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(rig.created_commands, [("G28", "G28 " + axis.upper(), {axis.upper(): ""})])
                    self.assertEqual(rig.dispatched_scripts, [])
                    self.assertTrue(all(position[2] is None for position, _ in rig.toolhead.moves))

    def test_pinned_and_indexed_apis_activate_and_restore_beta_current(self):
        for axis in "xy":
            for homing, toolhead in ((KalicoHoming, KalicoToolhead),
                                     (IndexedHoming, KalicoToolhead),
                                     (KalicoHoming, IndexedToolhead)):
                with self.subTest(axis=axis, homing=homing.__name__, toolhead=toolhead.__name__):
                    rig = BetaRig()
                    with native_current(rig, homing, toolhead) as (calls, selections):
                        rig.beta(axis.upper())
                    self.assertEqual([enabled for _, enabled in calls], [True, False])
                    self.assertEqual(selections, [axis, axis])
                    self.assertEqual(rig.workflow.last_result["result"], "beta_proposed")
                    self.assertEqual(rig.dispatched_scripts, [])
                    self.assertTrue(all(position[2] is None for position, _ in rig.toolhead.moves))

    def test_current_type_error_never_retries_activation_and_attempts_restore(self):
        for rig_type in (MappingRig, BetaRig):
            for restore_fails in (False, True):
                with self.subTest(workflow=rig_type.__name__, restore_fails=restore_fails):
                    rig = rig_type()
                    def fail(enabled):
                        if enabled:
                            raise TypeError("synthetic partial current activation")
                        if restore_fails:
                            raise TypeError("synthetic current restoration failure")
                    with native_current(rig, IndexedHoming, fail=fail) as (calls, _), \
                            self.assertRaisesRegex(CommandError, "native homing-current.*compatibility.*synthetic partial"):
                        if rig_type is MappingRig:
                            session = rig.begin_mapping()
                            rig.workflow._calibration_axis_mapping(session)
                        else:
                            rig.beta()
                    self.assertEqual([enabled for _, enabled in calls], [True, False])
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.home_calls, [])
                    if rig_type is MappingRig:
                        cleanup = session.get("axis_mapping_cleanup_errors", [])
                    else:
                        restoration = rig.workflow.last_result["beta_diagnostics"]["restoration"]
                        self.assertEqual(restoration["central_driver_current_restored"], not restore_fails)
                        cleanup = restoration.get("cleanup_errors", [])
                    self.assertEqual("synthetic current restoration failure" in str(cleanup), restore_fails)

    def test_strict_integer_only_helper_fails_actionably_before_motion(self):
        attempts = []

        class UnsupportedHoming(KalicoHoming):
            def _set_homing_current(self, homing_axes, pre_homing):
                attempts.append(pre_homing)
                if any(type(axis) is not int for axis in homing_axes):
                    raise TypeError("axis must have exact int type")
                return super()._set_homing_current(homing_axes, pre_homing)

        rig = MappingRig()
        with native_current(rig, UnsupportedHoming) as (calls, _), \
                self.assertRaisesRegex(CommandError, "native homing-current.*compatibility.*exact int"):
            rig.workflow._calibration_axis_mapping(rig.begin_mapping())
        self.assertEqual(attempts, [True, False])  # Cleanup, never alternate-axis activation.
        self.assertEqual(calls, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.home_calls, [])


if __name__ == "__main__":
    unittest.main()
