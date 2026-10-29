"""Offline tests for the homing session: stream, acceleration and checks."""
import collections
import types
import unittest

from harness import (CommandError, ConfigError, acquire_reference, batch, counts,
                     make_homing, plugin, stationary_stream)

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST
Info = collections.namedtuple("Info", "speed position_endstop retract_speed retract_dist "
                                      "positive_dir second_homing_speed")
KalicoInfo = collections.namedtuple("KalicoInfo", Info._fields + ("min_home_dist",))


def info(speed=150.0, retract=5.0, second=150.0, min_home_dist=None):
    if min_home_dist is None:
        return Info(speed, 0.0, speed, retract, False, second)
    return KalicoInfo(speed, 0.0, speed, retract, False, second, min_home_dist)


class FakeRail:
    def __init__(self, endstop, homing_info=None, name="stepper_x", rng=(0.0, 300.0)):
        self.endstop, self.info, self.name, self.rng = endstop, homing_info or info(), name, rng

    def get_endstops(self):
        return [(self.endstop, self.name)]

    def get_homing_info(self):
        return self.info

    def get_range(self):
        return self.rng

    def get_name(self):
        return self.name


def fake_stepper(name, mcu):
    return types.SimpleNamespace(get_mcu=lambda: mcu, get_name=lambda: name,
                                 get_step_dist=lambda: 0.0125)


class Board:
    """A stepper MCU that records trsync_trigger commands per dedicated queue."""

    def __init__(self, name="mcu", log=None):
        self.name, self.log, self.queues = name, [] if log is None else log, []

    def alloc_command_queue(self):
        self.queues.append(object())
        return self.queues[-1]

    def lookup_command(self, fmt, cq=None):
        assert cq is self.queues[-1], "trigger must use a dedicated queue"
        return types.SimpleNamespace(send=lambda args: self.log.append((self.name, list(args))))


def with_endstops(obj, axes=("x",), mcu_obj=None):
    stepper_mcu = mcu_obj or Board()
    endstops = {}
    for axis in axes:
        endstop = obj.setup_pin("endstop", {"pin": axis, "invert": False, "pullup": False})
        endstop.add_stepper(fake_stepper("stepper_" + axis, stepper_mcu))
        endstop.setup_trigger(obj._trigger_command)
        endstops[axis] = endstop
    return endstops


def stream_flows(obj):
    """Make the stream answer as soon as the session waits for it."""
    stationary_stream(obj)


def start_session(obj, rail):
    obj.homing_active = False
    stream_flows(obj)
    obj.fake_printer.reactor.now = 10.0
    obj._handle_rails_begin(None, [rail])


class AccelLimitTest(unittest.TestCase):
    def test_session_limits_accel_and_restores_it_at_rails_end(self):
        obj = make_homing(accel=20000.0)
        rail = FakeRail(with_endstops(obj)["x"])
        start_session(obj, rail)
        self.assertEqual(obj.toolhead.accel, 5000.0)
        self.assertEqual(obj.helper.accel_stream_cmd.messages, [[1, 0]])
        obj._handle_rails_end(None, [rail])
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertEqual(obj.helper.accel_stream_cmd.messages[-1], [0, 0])

    def test_accel_limit_works_on_kalico_and_older_klipper(self):
        from harness import KalicoToolhead, OldToolhead
        for cls in (KalicoToolhead, OldToolhead):
            obj = make_homing(accel=20000.0, toolhead_cls=cls)
            rail = FakeRail(with_endstops(obj)["x"])
            start_session(obj, rail)
            self.assertEqual(obj.toolhead.accel, 5000.0, cls.__name__)
            obj._handle_rails_end(None, [rail])
            self.assertEqual(obj.toolhead.accel_log, [5000.0, 20000.0], cls.__name__)

    def test_toolhead_without_accel_setter_is_a_config_error(self):
        from harness import BareToolhead
        with self.assertRaisesRegex(ConfigError, "homing_accel: 0"):
            make_homing(toolhead_cls=BareToolhead)
        make_homing(toolhead_cls=BareToolhead, options={"homing_accel": 0.0})

    def test_accel_is_never_raised(self):
        obj = make_homing(accel=3000.0)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertEqual(obj.toolhead.accel, 3000.0)
        self.assertEqual(obj.toolhead.accel_log, [])

    def test_homing_accel_zero_leaves_accel_alone(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 0})
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_failed_home_restores_accel_on_motor_off(self):
        # Klipper sends no rails_end after a failed home, G28 turns motors off.
        obj = make_homing(accel=20000.0)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        obj.fake_printer.send_event("stepper_enable:motor_off")
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertFalse(obj.homing_active)
        self.assertEqual(obj.helper.accel_stream_cmd.messages[-1], [0, 0])

    def test_session_left_open_closes_itself(self):
        obj = make_homing(accel=20000.0)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertNotEqual(obj._idle_check(11.0), obj.reactor.NEVER)
        self.assertEqual(obj._idle_check(10.0 + plugin.IDLE_CLEANUP + 1.0), obj.reactor.NEVER)
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_new_session_after_failure_keeps_the_original_accel(self):
        obj = make_homing(accel=20000.0)
        rail = FakeRail(with_endstops(obj)["x"])
        start_session(obj, rail)
        start_session(obj, rail)  # no cleanup in between
        obj._handle_rails_end(None, [rail])
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_stream_that_never_starts_restores_accel_and_fails(self):
        obj = make_homing(accel=20000.0)
        rail = FakeRail(with_endstops(obj)["x"])
        obj.homing_active = False
        with self.assertRaises(CommandError):
            obj._handle_rails_begin(None, [rail])
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertFalse(obj.homing_active)


class SessionTest(unittest.TestCase):
    def test_rails_without_beacon_endstops_are_ignored(self):
        obj = make_homing(accel=20000.0)
        with_endstops(obj)
        obj.homing_active = False
        obj._handle_rails_begin(None, [FakeRail(object(), name="stepper_z")])
        self.assertFalse(obj.homing_active)
        self.assertEqual(obj.toolhead.accel, 20000.0)
        self.assertEqual(obj.helper.accel_stream_cmd.messages, [])

    def test_two_beacon_axes_in_one_homing_call_are_rejected(self):
        obj = make_homing()
        endstops = with_endstops(obj, ("x", "y"))
        obj.homing_active = False
        with self.assertRaises(CommandError):
            obj._handle_rails_begin(None, [FakeRail(endstops["x"]), FakeRail(endstops["y"], name="stepper_y")])

    def test_fresh_baseline_before_every_pass_of_our_axis_only(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [object()]))
        self.assertEqual(len(obj.toolhead.waits), 1)
        self.assertGreaterEqual(obj.preparation.last - obj.preparation.first + 1e-9, 0.1)

    def test_home_start_outside_g28_fails_before_the_trsync_starts(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        obj.homing_active = False
        with self.assertRaises(CommandError):
            endstop.home_start(1.0, 0, 0, 0)
        self.assertEqual(endstop._dispatch.started, [])

    def test_deadline_covers_ramp_to_speed(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 1000})
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))
        attempt = obj.prepare(endstop, 100.0)
        # 330 mm at 150 mm/s, plus 0.15 s ramp at 1000 mm/s^2, plus the margin
        self.assertAlmostEqual(attempt.deadline - 100.0, 330.0 / 150.0 + 0.15 + 0.15, places=6)
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))
        second = obj.prepare(endstop, 200.0)
        self.assertAlmostEqual(second.deadline - 200.0, 13.0 / 150.0 + 0.15 + 0.15, places=6)
        self.assertEqual(second.pass_no, 2)

    def test_deadline_uses_max_velocity_when_lower_than_homing_speed(self):
        obj = make_homing(accel=20000.0, options={"homing_accel": 1000})
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        obj.toolhead.get_max_velocity = lambda: (50.0, 1000.0)
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))
        attempt = obj.prepare(endstop, 100.0)
        self.assertAlmostEqual(attempt.deadline - 100.0, 330.0 / 50.0 + 0.05 + 0.15, places=6)

    def test_full_home_reports_onset_time(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop))
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))
        board = endstop.get_mcu()
        arm_time = obj.reactor.now + 0.010
        endstop.home_start(arm_time, 0, 0, 0)
        obj._inspect(batch(round(arm_time * 1000.0), [(counts(-12.0), 0, counts(1.0))]
                           + [(0, 0, counts(1.0))] * 7))
        self.assertEqual(board.log, [("mcu", [7, HIT])])
        self.assertAlmostEqual(endstop.home_wait(arm_time + 1.0), arm_time, places=3)
        self.assertEqual(obj.last["x"]["result"], "impact")
        self.assertIsNone(obj.attempt)

    def test_failures_raise_instead_of_setting_an_origin(self):
        for reason in ("no_data", "no_baseline", "travel"):
            obj = make_homing()
            endstop = with_endstops(obj)["x"]
            attempt = types.SimpleNamespace(reason=reason, onset_batch=None)
            with self.assertRaises(CommandError):
                obj.finish(endstop, attempt, HOST)
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        with self.assertRaises(CommandError):
            obj.finish(endstop, types.SimpleNamespace(reason=None, onset_batch=None),
                       plugin.mcu.MCU_trsync.REASON_COMMS_TIMEOUT)
        self.assertEqual(obj.finish(endstop, types.SimpleNamespace(reason=None, onset_batch=None),
                                    plugin.mcu.MCU_trsync.REASON_PAST_END_TIME), 0.0)


class TriggerMcuTest(unittest.TestCase):
    def test_mcu_comes_from_the_first_stepper(self):
        obj = make_homing()
        board = Board()
        endstop = with_endstops(obj, mcu_obj=board)["x"]
        self.assertIs(endstop.get_mcu(), board)

    def test_trigger_reaches_every_stepper_mcu_on_its_own_queue(self):
        sent = []
        obj = make_homing(connect=False)
        main, extra = Board("mcu", sent), Board("extra", sent)
        endstop = obj.setup_pin("endstop", {"pin": "x", "invert": False, "pullup": False})
        endstop.add_stepper(fake_stepper("stepper_x", main))
        endstop.add_stepper(fake_stepper("stepper_y", extra))
        obj.fake_printer.send_event("klippy:connect")
        endstop.trigger(HIT)
        self.assertEqual(sent, [("mcu", [7, HIT]), ("extra", [8, HIT])])

    def test_old_mcu_option_still_loads(self):
        obj = make_homing(options={"mcu": "mcu"})
        self.assertTrue(obj.endstops is not None)


class ConnectChecksTest(unittest.TestCase):
    def test_missing_accelerometer_is_a_config_error_at_startup(self):
        obj = make_homing(connect=False)
        obj.fake_printer.objects["beacon"].accel_helper = None
        with self.assertRaises(ConfigError):
            obj.fake_printer.send_event("klippy:connect")

    def test_missing_beacon_internals_are_named(self):
        obj = make_homing(connect=False)
        del obj.fake_printer.objects["beacon"].accel_helper.accel_stream_cmd
        with self.assertRaisesRegex(ConfigError, "accel_stream_cmd"):
            obj.fake_printer.send_event("klippy:connect")

    def test_threshold_above_sensor_range_is_rejected(self):
        with self.assertRaises(ConfigError):
            make_homing(threshold=14.0)

    def test_missing_beacon_section_is_a_config_error(self):
        from harness import FakeConfig, FakePrinter, FakeToolhead, make_beacon
        printer = FakePrinter(FakeToolhead(), make_beacon())
        with self.assertRaises(ConfigError):
            plugin.BeaconAccelHoming(FakeConfig(printer, {}, sections=()))

    def test_rail_warnings_show_at_startup(self):
        obj = make_homing(connect=False)
        endstop = with_endstops(obj)["x"]
        rail = FakeRail(endstop, info(speed=30.0, retract=0.0))
        obj.fake_printer.objects["toolhead"].kin = types.SimpleNamespace(rails=[rail, FakeRail(object())])
        obj.fake_printer.send_event("klippy:connect")
        warnings = obj.fake_printer.objects["configfile"].warnings
        self.assertEqual(len(warnings), 1)
        self.assertIn("30 mm/s", warnings[0])

    def test_homing_warnings(self):
        self.assertEqual(plugin.homing_warnings("stepper_x", info()), [])
        # One touch per home is the design, so retract 0 is not a warning.
        self.assertEqual(plugin.homing_warnings("stepper_x", info(retract=0.0)), [])
        # Kalico min_home_dist gives the second touch for starts at the stop.
        self.assertEqual(plugin.homing_warnings("stepper_x", info(retract=0.0, min_home_dist=5.0)), [])
        self.assertEqual(len(plugin.homing_warnings("stepper_x", info(second=20.0))), 2)
        self.assertEqual(len(plugin.homing_warnings("stepper_x", info(speed=20.0, second=20.0))), 1)


class TravelCheckTest(unittest.TestCase):
    def _home(self, obj, endstop, pass_no, travel_mm):
        kin = types.SimpleNamespace(
            get_steppers=lambda: [types.SimpleNamespace(get_name=lambda: "stepper_x")],
            calc_position=lambda moved: [moved["stepper_x"], 0.0, 0.0])
        obj.toolhead.kin = kin
        stepper = types.SimpleNamespace(get_step_dist=lambda: 0.0125)
        trig = -int(round(travel_mm / 0.0125))
        sp = types.SimpleNamespace(stepper_name="stepper_x", stepper=stepper, start_pos=0,
                                   trig_pos=trig, halt_pos=trig - 30)
        record = {"axis": "x", "pass": pass_no, "result": "impact", "onset_g": -12.0,
                  "impact_peak_g": 16.0, "noise_pos_g": 3.0, "noise_neg_g": 3.0,
                  "trigger_after_arm_s": 0.05}
        attempt = types.SimpleNamespace(endstop=endstop, axis="x", pass_no=pass_no, reason="impact",
                                        record=record)
        obj.pending_move = attempt
        obj._handle_move_end(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop],
                                                   stepper_positions=[sp]))
        return attempt.record

    def test_second_touch_at_the_stop_passes(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=5.0)))
        record = self._home(obj, endstop, 2, 5.2)
        self.assertAlmostEqual(record["travel_mm"], 5.2)

    def test_early_second_touch_stops_homing(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=5.0)))
        with self.assertRaises(CommandError):
            self._home(obj, endstop, 2, 1.0)

    def test_late_second_touch_stops_homing(self):
        # An early first trigger left the second touch 10 mm
        # of commanded travel, and the end of that move was taken as the stop.
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=0.0, min_home_dist=5.0)))
        with self.assertRaises(CommandError):
            self._home(obj, endstop, 2, 9.881)

    def test_kalico_rehome_uses_min_home_dist(self):
        obj = make_homing()
        endstop = with_endstops(obj)["x"]
        start_session(obj, FakeRail(endstop, info(retract=0.0, min_home_dist=5.0)))
        self._home(obj, endstop, 2, 4.8)
        with self.assertRaises(CommandError):
            self._home(obj, endstop, 2, 2.0)

class StatusTest(unittest.TestCase):
    def test_calibration_reference_policy_is_visible_before_any_home(self):
        obj = make_homing()
        self.assertIsNone(obj.baseline_status)
        messages = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=messages.append))
        policy = next(message for message in messages if "baseline policy" in message)
        self.assertIn("ordinary homing requires quiet XYZ", policy)
        self.assertIn("fresh 100.00 ms / 50 sample reference", policy)
        self.assertIn("without a quiet-range limit", policy)
        self.assertIn("no acceleration-derived motion proof for XYZ magnitude", policy)

    def test_status_is_a_copy(self):
        obj = make_homing()
        self.assertIsNone(obj.get_status(0.0)["sample_timing_fault"])
        obj._record_sample_timing_fault("backend_batch", 0.1, 0.01, 0.02,
                                        cutoff=0.0, reason="too_old")
        obj.last["x"] = {"result": "impact", "impact_peak_g": 10.0}
        status = obj.get_status(0.0)
        obj.last["x"]["impact_peak_g"] = 12.0
        self.assertEqual(status["last"]["x"]["impact_peak_g"], 10.0)
        status["sample_timing_fault"]["sample_age_ms"] = 999.0
        self.assertEqual(obj.sample_timing_fault["sample_age_ms"], 90.0)
        obj.sample_timing_fault["reason"] = "future"
        self.assertEqual(status["sample_timing_fault"]["reason"], "too_old")
        self.assertEqual(set(status), {"threshold_g", "impact_sign", "homing_accel", "back_off_dist",
                                       "homing", "baseline", "sample_timing_fault", "last", "sensor",
                                       "backend", "workflows"})


if __name__ == "__main__":
    unittest.main()


class EnableLine:
    def __init__(self, log, name, enabled):
        self.log, self.name, self.enabled = log, name, enabled

    def is_motor_enabled(self):
        return self.enabled

    def motor_enable(self, print_time):
        self.log.append((self.name, print_time))
        self.enabled = True


class MotorEnableTest(unittest.TestCase):
    """Motors that were off are switched on and settle before the resting reading."""

    def _home_x(self, enabled, active=("stepper_x", "stepper_y")):
        obj = make_homing()
        log = []
        lines = {name: EnableLine(log, name, on) for name, on in enabled.items()}
        steppers = [types.SimpleNamespace(get_name=lambda n=name: n,
                                          is_active_axis=lambda a, n=name: a == "x" and n in active)
                    for name in enabled]
        toolhead = obj.toolhead
        toolhead.kin = types.SimpleNamespace(get_steppers=lambda: steppers,
                                             get_status=lambda t: {"homed_axes": ""})
        toolhead.flush_step_generation = lambda: None
        toolhead.get_last_move_time = lambda: 42.0
        toolhead.get_position = lambda: [150.0, 298.0, 40.0, 0.0]
        obj.fake_printer.objects["stepper_enable"] = types.SimpleNamespace(
            lookup_enable=lines.__getitem__)
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        return log, toolhead

    def test_motors_off_are_switched_on_before_the_move(self):
        # CoreXY: both motors drive X. Z is left alone.
        log, toolhead = self._home_x({"stepper_x": False, "stepper_y": False, "stepper_z": False})
        self.assertEqual(log, [("stepper_x", 42.0), ("stepper_y", 42.0)])
        self.assertIn(plugin.ENABLE_SETTLE, toolhead.dwells)

    def test_motors_already_on_add_no_delay(self):
        log, toolhead = self._home_x({"stepper_x": True, "stepper_y": True})
        self.assertEqual(log, [])
        self.assertNotIn(plugin.ENABLE_SETTLE, toolhead.dwells)

    def test_without_stepper_enable_nothing_happens(self):
        obj = make_homing()
        start_session(obj, FakeRail(with_endstops(obj)["x"]))
        self.assertNotIn(plugin.ENABLE_SETTLE, obj.toolhead.dwells)


class BackOffTest(unittest.TestCase):
    """An axis left pushed into its stop is moved off it before the next axis homes."""

    def _home_y(self, x_position, homed="xyz", x_info=None, options=None):
        obj = make_homing(options=options)
        endstops = with_endstops(obj, axes=("x", "y"))
        toolhead = obj.toolhead
        toolhead.position = [x_position, 150.0, 10.0, 0.0]
        toolhead.get_position = lambda: list(toolhead.position)
        toolhead.kin = types.SimpleNamespace(get_status=lambda t: {"homed_axes": homed})
        obj.rails_by_axis["x"] = FakeRail(endstops["x"], x_info or info())
        start_session(obj, FakeRail(endstops["y"], info(), name="stepper_y"))
        return toolhead

    def test_x_left_at_the_stop_backs_off_before_y_homes(self):
        toolhead = self._home_y(-0.4)
        self.assertEqual(toolhead.moves, [([5.0, None, None], 150.0)])
        self.assertIn(0.1, toolhead.dwells)

    def test_x_already_clear_of_the_stop_stays(self):
        self.assertEqual(self._home_y(50.0).moves, [])

    def test_back_off_can_be_switched_off(self):
        self.assertEqual(self._home_y(-0.4, options={"back_off_dist": 0.0}).moves, [])

    def test_unhomed_x_is_never_moved(self):
        self.assertEqual(self._home_y(-0.4, homed="").moves, [])

    def test_axis_homing_to_its_max_backs_off_downwards(self):
        x_info = Info(150.0, 300.0, 150.0, 0.0, True, 150.0)
        toolhead = self._home_y(300.3, x_info=x_info)
        self.assertEqual(toolhead.moves, [([295.0, None, None], 150.0)])


class EnablePreparationTest(unittest.TestCase):
    """Exercise the full pre-enable -> queued completion -> gate -> dispatch path."""

    def session(self, options=None, enabled=(False, False, False)):
        obj = make_homing(options=options)
        # This is the actual first calibration candidate, not a preparatory home.
        obj.threshold_g["x"] = 0.5
        endstop = with_endstops(obj)["x"]
        log = []
        lines = {name: EnableLine(log, name, state) for name, state in
                 zip(("stepper_x", "stepper_y", "stepper_z"), enabled)}
        steppers = [types.SimpleNamespace(get_name=lambda name=name: name,
                                         is_active_axis=lambda axis, name=name:
                                         axis in ("x", "y") and name != "stepper_z")
                    for name in lines]
        obj.toolhead.kin = types.SimpleNamespace(get_steppers=lambda: steppers,
                                                 get_status=lambda t: {"homed_axes": ""})
        obj.toolhead.get_position = lambda: [150.0, 150.0, 40.0, 0.0]
        obj.fake_printer.objects["stepper_enable"] = types.SimpleNamespace(
            lookup_enable=lines.__getitem__)
        start_session(obj, FakeRail(endstop, info(speed=100, retract=0)))
        return obj, endstop, log, lines

    def begin(self, obj, endstop):
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))

    def assert_unarmed_cleanup(self, obj, endstop):
        self.assertEqual(endstop._dispatch.started, [])
        self.assertEqual(endstop.get_mcu().log, [])
        self.assertEqual(obj.toolhead.moves, [])
        self.assertFalse(obj.homing_active)
        self.assertFalse(obj.stream_on)
        self.assertFalse(obj.preparing)
        self.assertIsNone(obj.preparation)
        self.assertEqual(obj.toolhead.accel, 20000.0)

    def test_corexy_enable_settle_really_completes_before_fresh_0_5g_trial(self):
        obj, endstop, log, lines = self.session()
        self.assertEqual(log, [("stepper_x", 10.0), ("stepper_y", 10.0)])
        self.assertFalse(lines["stepper_z"].enabled)
        self.assertLess(obj.reactor.now, 10.25)  # Dwell was only queued.
        self.begin(obj, endstop)
        self.assertEqual(obj.toolhead.waits, [10.25])
        self.assertGreater(obj.preparation.first, 10.25)
        self.assertGreaterEqual(obj.preparation.last - obj.preparation.first, 0.1)
        self.assertEqual(endstop._dispatch.started, [])
        when = round(obj.reactor.now * 1000.0) / 1000.0 + 0.005
        endstop.home_start(when, 0, 0, 0)
        self.assertEqual(len(endstop._dispatch.started), 1)
        self.assertEqual(obj.attempt.limit_raw * obj.attempt.count_g, 0.5)
        # The only post-arm sample is a real 0.6 g contact, below the old 1 g
        # floor. The pre-arm 15 g enable peak must neither trigger nor hide it.
        obj._inspect(batch(round(when * 1000) - 7,
                           [(counts(-15), 0, counts(1))] * 7
                           + [(counts(-0.6), 0, counts(1))]))
        self.assertEqual(obj.attempt.reason, "impact")
        self.assertAlmostEqual(obj.attempt.onset_raw * obj.attempt.count_g, -0.6,
                               delta=obj.attempt.count_g / 2)
        self.assertAlmostEqual(obj.onset_time(obj.attempt), when)

    def test_already_enabled_corexy_and_partly_enabled_keep_quiet_gate(self):
        for enabled, expected in (((True, True, True), []),
                                  ((True, False, True), [("stepper_y", 10.0)])):
            with self.subTest(enabled=enabled):
                obj, endstop, log, lines = self.session(enabled=enabled)
                self.begin(obj, endstop)
                self.assertEqual(log, expected)
                self.assertTrue(lines["stepper_z"].enabled)
                self.assertIsNotNone(obj.preparation.baseline())
                self.assertEqual(len(obj.toolhead.waits), 1)

    def test_half_g_candidate_has_bounded_half_g_all_axis_quiet_limit(self):
        for axis in range(3):
            for excursion, accepted in ((0.04, True), (0.06, True),
                                        (0.25, True), (0.26, False)):
                with self.subTest(axis=axis, excursion_g=excursion):
                    obj, endstop, _log, _lines = self.session()
                    plain = obj.reactor.on_pause
                    batches = [0]
                    def flow():
                        if obj.preparation is None:
                            plain()
                            return
                        count_g = obj.session_scale["scale"] / 1000.0
                        self.assertAlmostEqual(obj.preparation.limit * count_g, 0.5)
                        batches[0] += 1
                        values = [0, 0, counts(1)]
                        values[axis] += counts(excursion if batches[0] % 2 else -excursion)
                        obj._inspect(batch(round(obj.reactor.now * 1000) - 4,
                                           [values] * 5, delta_clock=4,
                                           receive_time=obj.reactor.now))
                    obj.reactor.on_pause = flow
                    if accepted:
                        self.begin(obj, endstop)
                        self.assertIsNotNone(obj.preparation.baseline())
                        self.assertEqual(endstop._dispatch.started, [])
                    else:
                        with self.assertRaisesRegex(CommandError, "fresh quiet"):
                            self.begin(obj, endstop)
                        self.assert_unarmed_cleanup(obj, endstop)

    def test_native_calibration_keeps_noisy_reference_envelope_and_first_raw_contact(self):
        from test_single_touch import RampStepper
        for threshold in (1.0, 8.0):
            for axis in range(3):
                with self.subTest(threshold=threshold, noisy_axis=axis):
                    obj, endstop, _log, _lines = self.session()
                    obj.threshold_g["x"] = threshold
                    obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x"}
                    obj.workflows.in_home, obj.workflows.home_axis = True, "x"
                    plain, number = obj.reactor.on_pause, [0]
                    def flow():
                        if obj.preparation is None:
                            plain()
                            return
                        number[0] += 1
                        values = [0, 0, counts(1)]
                        values[axis] += counts(0.7 if number[0] % 2 else -0.7)
                        obj._inspect(batch(round(obj.reactor.now * 1000) - 4,
                                           [values] * 5, delta_clock=4,
                                           receive_time=obj.reactor.now))
                    obj.reactor.on_pause = flow
                    self.begin(obj, endstop)
                    reference = obj.preparation
                    self.assertFalse(reference.require_quiet)
                    self.assertGreaterEqual(reference.last - reference.first + 1e-9, 0.1)
                    self.assertGreaterEqual(reference.count, 50)
                    self.assertAlmostEqual((reference.high[axis] - reference.low[axis])
                                           * obj.session_scale["scale"] / 1000.0, 1.40625)
                    means = reference.baseline()
                    envelope = tuple(max(abs(low - mean), abs(high - mean))
                                     for low, high, mean in zip(reference.low, reference.high, means))
                    when = round(obj.reactor.now * 1000.0) / 1000.0 + 0.005
                    endstop.get_steppers = lambda: [RampStepper(when, 5000.0, 100.0)]
                    attempt = obj.prepare(endstop, when)
                    self.assertEqual(attempt.bases, means)
                    self.assertEqual(attempt.proof_noise, envelope)
                    self.assertAlmostEqual(attempt.limit_raw * attempt.count_g, threshold)
                    self.assertFalse(obj.baseline_status["quiet_range_enforced"])
                    self.assertEqual(endstop._dispatch.started, [])
                    self.assertEqual(endstop.get_mcu().log, [])
                    obj.arm(attempt)
                    obj.reactor.now = when + 0.004
                    obj._inspect(batch(round(when * 1000),
                                       [(counts(-threshold - 0.1), 0, counts(1))]
                                       + [(0, 0, counts(1))] * 4,
                                       delta_clock=4, receive_time=obj.reactor.now))
                    self.assertEqual(attempt.reason, "impact")
                    self.assertAlmostEqual(obj.onset_time(attempt), when, places=6)
                    self.assertEqual(endstop.get_mcu().log, [("mcu", [7, HIT])])

    def test_native_calibration_still_rejects_clipped_changed_scale_and_bad_timestamps(self):
        for mode in ("clip", "scale", "stale", "future"):
            with self.subTest(mode=mode):
                obj, endstop, _log, _lines = self.session()
                obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x"}
                obj.workflows.in_home, obj.workflows.home_axis = True, "x"
                plain = obj.reactor.on_pause
                def flow():
                    if obj.preparation is None:
                        plain()
                        return
                    start = round(obj.reactor.now * 1000) - 4
                    if mode == "scale":
                        obj.helper._stream_en = 1
                        obj.helper._scale = dict(obj.helper._scale, id=99)
                    elif mode == "stale":
                        obj.reactor.now += 0.1
                    elif mode == "future":
                        start += 20
                    values = (0, plugin.CLIPPED if mode == "clip" else 0, counts(1))
                    obj._batch_hook(batch(start, [values] * 5, delta_clock=4,
                                          receive_time=obj.reactor.now))
                obj.reactor.on_pause = flow
                with self.assertRaises(CommandError):
                    self.begin(obj, endstop)
                self.assert_unarmed_cleanup(obj, endstop)
                self.assertFalse(obj.baseline_status["quiet_range_enforced"])
                self.assertEqual(obj.sensor_fault, {"clip": "invalid_data", "scale": "scale",
                                                    "stale": "stale_data", "future": "stale_data"}[mode])

    def test_old_delivered_peak_is_excluded_then_ringdown_must_be_quiet(self):
        obj, endstop, _log, _lines = self.session()
        plain = obj.reactor.on_pause
        old = batch(10000, [(counts(-15), 0, counts(1))] * 8)
        delivered = []
        def flow():
            if obj.preparation is None:
                plain()
                return
            old["#receive_time"] = obj.reactor.now
            obj._inspect(old)
            elapsed = obj.reactor.now - obj.preparation.cutoff
            x = (3.0 if len(delivered) % 2 else -3.0) if elapsed <= 0.030 else 0.0
            delivered.append(elapsed)
            obj._inspect(batch(round(obj.reactor.now * 1000) - 4,
                               [(counts(x), 0, counts(1))] * 5,
                               delta_clock=4, receive_time=obj.reactor.now))
        obj.reactor.on_pause = flow
        self.begin(obj, endstop)
        self.assertGreater(obj.preparation.first, obj.preparation.cutoff + 0.025)
        self.assertAlmostEqual(obj.preparation.baseline()[0], 0.0)
        self.assertEqual(endstop._dispatch.started, [])

    def test_all_axes_noise_and_absent_data_abort_before_dispatch(self):
        for mode in ("x_noise", "z_noise", "no_data", "old_only"):
            with self.subTest(mode=mode):
                obj, endstop, _log, _lines = self.session()
                plain = obj.reactor.on_pause
                n = [0]
                def flow():
                    if obj.preparation is None:
                        plain()
                    elif mode == "old_only":
                        obj._inspect(batch(10000, [(0, 0, counts(1))] * 8,
                                           receive_time=obj.reactor.now))
                    elif mode != "no_data":
                        n[0] += 1
                        values = [0, 0, counts(1)]
                        values[0 if mode == "x_noise" else 2] += counts(3 if n[0] % 2 else -3)
                        obj._inspect(batch(round(obj.reactor.now * 1000) - 4,
                                           [values] * 5, delta_clock=4,
                                           receive_time=obj.reactor.now))
                obj.reactor.on_pause = flow
                with self.assertRaisesRegex(CommandError, "fresh quiet"):
                    self.begin(obj, endstop)
                self.assert_unarmed_cleanup(obj, endstop)
                self.assertLessEqual(obj.reactor.now, 10.25 + obj.baseline_time + 0.506)

    def test_invalid_reference_streams_abort_before_dispatch(self):
        for mode in ("stale", "future", "duplicate", "out_of_order", "gap", "clip", "scale"):
            with self.subTest(mode=mode):
                obj, endstop, _log, _lines = self.session()
                plain = obj.reactor.on_pause
                seen = []
                def flow():
                    if obj.preparation is None:
                        plain()
                        return
                    start = round(obj.reactor.now * 1000) - 4
                    if mode == "stale":
                        obj.reactor.now += 0.1
                    elif mode == "future":
                        start += 20
                    elif mode in ("duplicate", "out_of_order") and seen:
                        start = seen[0] - (1 if mode == "out_of_order" else 0)
                    elif mode == "gap" and seen:
                        start += 20
                        obj.reactor.now += 0.020
                    elif mode == "scale":
                        obj.helper._stream_en = 1
                        obj.helper._scale = dict(obj.helper._scale, id=99)
                    values = (0, plugin.CLIPPED if mode == "clip" else 0, counts(1))
                    seen.append(start)
                    obj._batch_hook(batch(start, [values] * 5, delta_clock=4,
                                           receive_time=obj.reactor.now))
                obj.reactor.on_pause = flow
                with self.assertRaises(CommandError):
                    self.begin(obj, endstop)
                self.assert_unarmed_cleanup(obj, endstop)
                self.assertIn(obj.sensor_fault, ("stale_data", "invalid_data", "sample_gap", "scale"))

    def test_native_stale_oldest_retains_actual_fresh_newest_batch_age(self):
        obj, endstop, _log, _lines = self.session()
        plain = obj.reactor.on_pause
        def flow():
            if obj.preparation is None:
                plain()
                return
            obj.reactor.now += 0.080
            now = obj.reactor.now
            obj._inspect(batch(round(now * 1000) - 80, [(0, 0, counts(1))] * 80,
                               delta_clock=79, receive_time=now))
        obj.reactor.on_pause = flow
        with self.assertRaisesRegex(CommandError, "stale"):
            self.begin(obj, endstop)
        self.assert_unarmed_cleanup(obj, endstop)
        diagnostic = obj.get_status(obj.reactor.now)["baseline"]
        self.assertEqual(diagnostic["result"], "stale_data")
        self.assertGreater(diagnostic["oldest_batch_age_ms"], diagnostic["max_sample_age_ms"])
        self.assertAlmostEqual(diagnostic["newest_batch_age_ms"], 1.0)
        self.assertEqual(diagnostic["accepted_samples"], 0)

    def test_reference_cannot_be_reused_or_become_stale_before_trsync(self):
        for mode in ("missing", "stale", "new_noise", "reused"):
            with self.subTest(mode=mode):
                obj, endstop, _log, _lines = self.session()
                if mode != "missing":
                    self.begin(obj, endstop)
                if mode == "stale":
                    obj.reactor.now += 0.1
                elif mode == "new_noise":
                    obj.reactor.pause(obj.reactor.now + 0.005)
                    obj._inspect(batch(round(obj.reactor.now * 1000) + 1,
                                       [(counts(3), 0, counts(1))] * 5,
                                       delta_clock=4, receive_time=obj.reactor.now + 0.005))
                elif mode == "reused":
                    obj.prepare(endstop, obj.reactor.now + 0.010)
                with self.assertRaisesRegex(CommandError, "fresh quiet"):
                    endstop.home_start(obj.reactor.now + 0.010, 0, 0, 0)
                self.assert_unarmed_cleanup(obj, endstop)
                expected = {"missing": "missing_before_arm", "stale": "expired_before_arm",
                            "new_noise": "quiet_window_invalidated_before_arm", "reused": "missing_before_arm"}
                self.assertEqual(obj.baseline_status["result"], expected[mode])
                self.assertEqual(obj.baseline_status["phase"], "before_arm")

    def test_native_reference_diagnostics_use_machine_axes_and_survive_cleanup(self):
        obj, endstop, _log, _lines = self.session()
        obj.helper.config.axes_map = ((1, -1), (0, 1), (2, -1))
        plain = obj.reactor.on_pause
        n = [0]
        def flow():
            if obj.preparation is None:
                plain()
                return
            n[0] += 1
            sign = 1 if n[0] % 2 else -1
            obj._inspect(batch(round(obj.reactor.now * 1000) - 4,
                               [(counts(sign * 0.03125), counts(sign * 0.015625), counts(1))] * 5,
                               delta_clock=4, receive_time=obj.reactor.now))
        obj.reactor.on_pause = flow
        self.begin(obj, endstop)
        obj._end_session()
        diagnostic = obj.get_status(obj.reactor.now)["baseline"]
        self.assertEqual(diagnostic["result"], "ready")
        self.assertAlmostEqual(diagnostic["observed_peak_to_peak_g"]["x"], 0.03125, delta=0.001)
        self.assertAlmostEqual(diagnostic["observed_peak_to_peak_g"]["y"], 0.0625, delta=0.001)
        self.assertAlmostEqual(diagnostic["observed_peak_to_peak_g"]["z"], 0.0)
        diagnostic["observed_peak_to_peak_g"]["x"] = 999
        self.assertLess(obj.get_status(obj.reactor.now)["baseline"]["observed_peak_to_peak_g"]["x"], 1)
        messages = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=messages.append))
        self.assertTrue(any("best quiet" in message and "p-p XYZ" in message for message in messages))
        self.assertLess(len(next(message for message in messages if "best quiet" in message)), 600)

    def test_second_touch_gets_new_reference_after_retract_finishes(self):
        obj, endstop, _log, _lines = self.session()
        self.begin(obj, endstop)
        first = obj.preparation
        obj.prepare(endstop, obj.reactor.now + 0.010)
        obj.toolhead.dwell(0.050)  # queued retract/settle completion in the fake
        self.begin(obj, endstop)
        second = obj.preparation
        self.assertIsNot(second, first)
        self.assertGreater(second.cutoff, first.last)
        attempt = obj.prepare(endstop, obj.reactor.now + 0.010)
        self.assertEqual(attempt.pass_no, 2)
        self.assertEqual(endstop._dispatch.started, [])

    def test_long_baseline_does_not_depend_on_recent_batch_capacity(self):
        obj, endstop, _log, _lines = self.session({"baseline_time": 1.0, "baseline_batches": 10})
        self.begin(obj, endstop)
        self.assertGreaterEqual(obj.preparation.last - obj.preparation.first, 1.0)
        self.assertGreaterEqual(obj.preparation.count, 1000)
        self.assertEqual(len(obj.recent), 10)

    def test_long_queued_settle_does_not_trigger_idle_cleanup(self):
        obj, endstop, _log, _lines = self.session()
        obj.toolhead.print_time += 5.0
        plain = obj.reactor.on_pause
        def flow():
            self.assertNotEqual(obj._idle_check(obj.reactor.now), obj.reactor.NEVER)
            plain()
        obj.reactor.on_pause = flow
        self.begin(obj, endstop)
        self.assertTrue(obj.homing_active)
        self.assertGreater(obj.preparation.cutoff, 15.0)

    def test_stream_start_timeout_begins_after_enable_backoff_has_yielded(self):
        obj = make_homing(options={"enable_settle_time": 2.0, "baseline_time": 1.0})
        endstop = with_endstops(obj)["x"]
        # Real dwell/check_pause can yield on a long buffered enable/backoff.
        obj._enable_motors = lambda axis: obj.reactor.pause(obj.reactor.now + 0.75)
        start_session(obj, FakeRail(endstop))
        self.assertGreaterEqual(obj.reactor.now, 10.75)
        self.assertTrue(obj.homing_active)
        self.begin(obj, endstop)
        self.assertIsNotNone(obj.preparation.baseline())

    def test_motor_off_or_shutdown_during_wait_or_acquisition_cancels(self):
        for event in ("stepper_enable:motor_off", "klippy:shutdown"):
            for phase in ("wait", "collect"):
                with self.subTest(event=event, phase=phase):
                    obj, endstop, _log, _lines = self.session()
                    plain = obj.reactor.on_pause
                    def flow():
                        if phase == "wait" or obj.preparation is not None:
                            obj.fake_printer.send_event(event)
                        else:
                            plain()
                    obj.reactor.on_pause = flow
                    with self.assertRaisesRegex(CommandError, "cancelled"):
                        self.begin(obj, endstop)
                    self.assert_unarmed_cleanup(obj, endstop)

    def test_beacon_reference_uses_sensor_clock_conversion_and_wrap_epoch(self):
        for mode in ("separate_mcu", "wrap"):
            with self.subTest(mode=mode):
                obj = make_homing()
                endstop = with_endstops(obj)["x"]
                if mode == "separate_mcu":
                    obj.sensor_clock.print_time_to_clock = lambda t: round((t + 17.0) * 1000)
                    obj.clock_to_time = lambda clock: clock / 1000.0 - 17.0
                    obj.reactor.now = 10.0
                else:
                    obj.reactor.now = (2 ** 32 - 80) / 1000.0
                    def convert(clock):
                        now_clock = round(obj.reactor.now * 1000)
                        delta = ((clock - now_clock + 0x80000000) & 0xFFFFFFFF) - 0x80000000
                        return (now_clock + delta) / 1000.0
                    obj.clock_to_time = convert
                stationary_stream(obj)
                obj.homing_active = False
                obj._handle_rails_begin(None, [FakeRail(endstop)])
                self.begin(obj, endstop)
                self.assertGreater(obj.preparation.first, obj.preparation.cutoff)
                self.assertIsNotNone(obj.preparation.baseline())

    def test_enable_settle_setting_is_finite_bounded_and_optional(self):
        for value in (float("nan"), float("inf"), -0.01, 2.01):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                make_homing(options={"enable_settle_time": value})
        for value in (0.0, 0.75, 2.0):
            obj, endstop, _log, _lines = self.session({"enable_settle_time": value})
            self.begin(obj, endstop)
            self.assertIn(value, obj.toolhead.dwells)
            self.assertGreaterEqual(obj.preparation.cutoff, 10.0 + value)
