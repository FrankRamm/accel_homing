"""Selective release with shared pins and deferred per-driver callbacks."""
import types
import unittest
from unittest import mock
from test_workflows import CommandError, Rig, Stepper, workflow_module


class TMCCommandHelper:
    def __init__(self, rig, name, pending):
        self.stepper_name = name
        self.stepper_enable = rig.objects['stepper_enable']
        self.toff = self.driver_toff = 4
        self.pending = pending

    def _handle_stepper_enable(self, stamp, enabled):
        self.pending.append(lambda: setattr(self, 'driver_toff', self.toff if enabled else 0))

    def unrelated_callback(self, *args):
        pass


class Tracking:
    def __init__(self, name, pin, disabled):
        self.name, self.enable, self.disabled = name, pin, disabled
        self.enabled = True
        self.callbacks = []
        pin.enable_count += 1

    def has_dedicated_enable(self):
        return False

    def is_motor_enabled(self):
        return self.enabled

    def motor_disable(self, stamp):
        if self.enabled:
            for callback in self.callbacks:
                callback(stamp, False)
            self.enable.enable_count -= 1
            self.enabled = False
            self.disabled.append(self.name)


class VirtualEnableTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(workflow_module, '_TMCCommandHelper', TMCCommandHelper)
        patch.start()
        self.addCleanup(patch.stop)

    def rig(self, no_pin=False):
        rig = Rig()
        rig.toolhead.kin.steppers.extend([Stepper('stepper_z2', 'z'), Stepper('stepper_z3', 'z')])
        pin = types.SimpleNamespace(mcu_enable=None if no_pin else object(), enable_count=0)
        pending, helpers = [], {}
        for stepper in rig.toolhead.kin.steppers:
            name = stepper.get_name()
            line = rig.lines[name] = Tracking(name, pin, rig.disabled)
            helpers[name] = TMCCommandHelper(rig, name, pending)
            line.callbacks.append(helpers[name]._handle_stepper_enable)
        def wait():
            while pending:
                pending.pop(0)()
        rig.toolhead.wait_moves = wait
        return rig, pin, helpers

    def test_shared_and_pinless_release_both_xy_keep_four_z_held(self):
        for no_pin in (False, True):
            with self.subTest(no_pin=no_pin):
                rig, pin, helpers = self.rig(no_pin)
                rig.owner.endstops.pop('y')
                z_limit = rig.toolhead.kin.limits[2]
                rig.workflow._preflight(['x'])
                rig.workflow._disable_xy()
                self.assertEqual(rig.disabled, ['stepper_x', 'stepper_y'])
                self.assertEqual(pin.enable_count, 4)
                for name, helper in helpers.items():
                    is_z = name.startswith('stepper_z')
                    self.assertEqual(helper.driver_toff, 4 if is_z else 0)
                    self.assertEqual(rig.lines[name].enabled, is_z)
                self.assertEqual(rig.toolhead.kin.limits[2], z_limit)
                self.assertEqual(rig.toolhead.kin.get_status(0)['homed_axes'], 'z')
                rig.workflow._disable_xy()
                self.assertEqual(pin.enable_count, 4)

    def test_unverified_enable_rejected_before_side_effects(self):
        for failure in ('missing', 'unrelated', 'wrong_callback', 'wrong_stepper',
                        'wrong_tracker', 'none_toff', 'zero_toff', 'bad_toff', 'missing_api'):
            for no_pin in (False, True):
                with self.subTest(failure=failure, no_pin=no_pin):
                    rig, pin, helpers = self.rig(no_pin)
                    line, helper = rig.lines['stepper_y'], helpers['stepper_y']
                    if failure == 'missing':
                        line.callbacks = []
                    elif failure == 'unrelated':
                        class Unrelated:
                            toff = 4
                            def callback(self, *args): pass
                        line.callbacks = [Unrelated().callback]
                    elif failure == 'wrong_callback':
                        line.callbacks = [helper.unrelated_callback]
                    elif failure == 'wrong_stepper':
                        helper.stepper_name = 'stepper_z'
                    elif failure == 'wrong_tracker':
                        helper.stepper_enable = types.SimpleNamespace(lookup_enable=lambda name: object())
                    elif failure == 'none_toff':
                        helper.toff = None
                    elif failure == 'zero_toff':
                        helper.toff = 0
                    elif failure == 'bad_toff':
                        helper.toff = '4'
                    else:
                        line.has_dedicated_enable = None
                    with self.assertRaises(CommandError):
                        rig.workflow._disable_xy()
                    self.assertEqual(rig.disabled, [])
                    self.assertEqual(pin.enable_count, 6)

    def test_same_tracker_rejected_even_with_tmc(self):
        rig, _, _ = self.rig()
        rig.lines['stepper_z'] = rig.lines['stepper_x']
        with self.assertRaisesRegex(CommandError, 'enable tracker'):
            rig.workflow._disable_xy()
        self.assertEqual(rig.disabled, [])

    def test_dedicated_motor_can_coexist_with_virtual_enable(self):
        rig, _, _ = self.rig()
        line = rig.lines['stepper_y']
        line.enable = types.SimpleNamespace(mcu_enable=object(), enable_count=1)
        line.callbacks = []
        line.has_dedicated_enable = lambda: True
        rig.workflow._disable_xy()
        self.assertEqual(rig.disabled, ['stepper_x', 'stepper_y'])

    def test_missing_firmware_api_fails_closed(self):
        rig, _, _ = self.rig()
        with mock.patch.object(workflow_module, '_TMCCommandHelper', None):
            with self.assertRaises(CommandError):
                rig.workflow._disable_xy()
        self.assertEqual(rig.disabled, [])

    def test_existing_hardware_enable_setups_need_no_tmc_api(self):
        for kinematics in ('cartesian', 'corexy'):
            for shared_xy in (False, True):
                with self.subTest(kinematics=kinematics, shared_xy=shared_xy):
                    rig = Rig()
                    rig.workflow.kinematics_name = kinematics
                    if kinematics == 'cartesian':
                        for stepper in rig.toolhead.kin.steppers:
                            if stepper.name in ('stepper_x', 'stepper_y'):
                                stepper.axes = stepper.name[-1]
                    xy_pin = types.SimpleNamespace(mcu_enable=object(), enable_count=0)
                    for stepper in rig.toolhead.kin.steppers:
                        pin = xy_pin if shared_xy and stepper.name in (
                            'stepper_x', 'stepper_y') else types.SimpleNamespace(
                                mcu_enable=object(), enable_count=0)
                        line = rig.lines[stepper.name] = Tracking(stepper.name, pin, rig.disabled)
                        line.has_dedicated_enable = lambda shared=shared_xy: not shared
                    z_states = {name: line.is_motor_enabled() for name, line in rig.lines.items()
                                if name.startswith('stepper_z')}
                    with mock.patch.object(workflow_module, '_TMCCommandHelper', None):
                        rig.workflow._preflight(['x'])
                        rig.workflow._disable_xy()
                    self.assertEqual(rig.disabled, ['stepper_x', 'stepper_y'])
                    self.assertEqual(rig.toolhead.kin.get_status(0)['homed_axes'], 'z')
                    for name, enabled in z_states.items():
                        self.assertEqual(rig.lines[name].is_motor_enabled(), enabled)

    def test_coupled_xy_z_motor_still_rejected_with_virtual_enable(self):
        rig, _, _ = self.rig()
        rig.toolhead.kin.steppers[0].axes = 'xyz'
        with self.assertRaisesRegex(CommandError, 'also drives Z'):
            rig.workflow._disable_xy()
        self.assertEqual(rig.disabled, [])

    def test_shutdown_during_release_is_reported(self):
        rig, _, _ = self.rig()
        rig.lines['stepper_x'].callbacks.append(lambda *args: setattr(rig, 'shutdown', True))
        with self.assertRaisesRegex(CommandError, 'shut down'):
            rig.workflow._disable_xy()


if __name__ == '__main__':
    unittest.main()
