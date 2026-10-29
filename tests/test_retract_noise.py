"""Synthetic final-coordinate lifecycle and stationary-noise contracts only."""
import contextlib
import json
from html.parser import HTMLParser
from pathlib import Path
import tempfile
import re
import types
import unittest
from unittest import mock

from harness import CommandError, FakeToolhead, batch, counts, make_homing, plugin
from test_backends import generic
from test_session import FakeRail, info, with_endstops
from test_workflows import GCmd, Kin, Rig, workflow_module


class PositionedToolhead(FakeToolhead):
    def __init__(self, accel=20000.0, kin=None):
        super().__init__(accel, kin)
        self.position = [150.0, 100.0, 40.0, 6.0]

    def get_position(self):
        return list(self.position)

    def set_position(self, position):
        self.position = list(position)

    def manual_move(self, coord, speed):
        super().manual_move(coord, speed)
        for index, value in enumerate(coord):
            if value is not None:
                self.position[index] = value


class FinalRetractTests(unittest.TestCase):
    def machine(self, axes="xy", positive=False, bounds=(0.0, 300.0), final_free=0.0):
        kin = Kin()
        state = types.SimpleNamespace(cancel=None, fail=False, changed_record=False,
                                      calls=0, event_positions=[])
        objects = {}

        def home(homing):
            state.calls += 1
            obj = objects["owner"]
            for axis in axes:
                rail = obj.rails_by_axis[axis]
                homing_info = rail.get_homing_info()
                index = "xy".index(axis)
                obj.homing_active = True
                obj.session_axes = {axis: rail}
                obj.homed_before = None
                record = {"axis": axis, "result": "impact", "moving": True,
                          "motion_seen": True, "travel_mm": 150.0}
                obj.last[axis] = record
                obj.last_pass = types.SimpleNamespace(axis=axis, reason="impact", record=record)
                kin.limits[index] = bounds
                obj.toolhead.position[index] = homing_info.position_endstop + 0.25
                obj._handle_rails_end(homing, [rail])
                state.event_positions.append(list(obj.toolhead.position))
                # Emulate stock's post-event adjust_pos coordinate correction.
                away = -1.0 if homing_info.positive_dir else 1.0
                obj.toolhead.position[index] = homing_info.position_endstop + away * final_free
                if state.cancel is not None:
                    state.cancel(obj)
                if state.changed_record:
                    obj.last[axis] = dict(record)
            if state.fail:
                raise CommandError("later rail failed after accepted X")
            return "stock result"

        kin.home = home
        obj = make_homing(kin=kin, toolhead_cls=PositionedToolhead)
        objects["owner"] = obj
        endstops = with_endstops(obj, tuple(axes))
        for axis in axes:
            homing_info = info(speed=100.0, retract=0.0)
            homing_info = homing_info._replace(positive_dir=positive,
                                               position_endstop=bounds[1] if positive else bounds[0])
            obj.rails_by_axis[axis] = FakeRail(endstops[axis], homing_info, rng=bounds)
        return obj, kin, state

    def test_xy_release_waits_for_final_coordinate_assignment_and_preserves_records(self):
        obj, kin, state = self.machine(final_free=0.75)
        before = obj.toolhead.position[2:]
        self.assertEqual(kin.home(object()), "stock result")
        self.assertEqual(obj.toolhead.moves, [([5.0, 5.0, None], 100.0)])
        self.assertEqual(obj.toolhead.position[:2], [5.0, 5.0])
        self.assertEqual(obj.toolhead.position[2:], before)
        self.assertEqual(obj.last["x"]["travel_mm"], 150.0)
        self.assertEqual(len(state.event_positions), 2)

    def test_both_directions_bounds_and_existing_kalico_retract(self):
        for positive, bounds, expected in ((False, (0.0, 300.0), 5.0),
                                            (True, (0.0, 300.0), 295.0),
                                            (False, (0.0, 3.0), 3.0),
                                            (True, (0.0, 3.0), 0.0)):
            with self.subTest(positive=positive, bounds=bounds):
                obj, kin, _state = self.machine("x", positive, bounds)
                kin.home(object())
                self.assertEqual(obj.toolhead.moves, [([expected, None, None], 100.0)])
        obj, kin, _state = self.machine("x", final_free=5.0)
        kin.home(object())
        self.assertEqual(obj.toolhead.moves, [])

    def test_failures_interruptions_record_mismatch_and_workflow_entry_never_release(self):
        for failure in ("error", "motor_off", "shutdown", "disconnect", "record", "workflow"):
            with self.subTest(failure=failure):
                obj, kin, state = self.machine()
                if failure == "error":
                    state.fail = True
                elif failure in ("motor_off", "shutdown", "disconnect"):
                    event = "stepper_enable:motor_off" if failure == "motor_off" else "klippy:" + failure
                    state.cancel = lambda obj, event=event: obj.fake_printer.send_event(event)
                elif failure == "record":
                    state.changed_record = True
                else:
                    obj.workflows.session = {"mode": "calibration"}
                    state.cancel = lambda obj: setattr(obj.workflows, "session", None)
                if state.fail:
                    with self.assertRaises(CommandError):
                        kin.home(object())
                else:
                    kin.home(object())
                self.assertEqual(obj.toolhead.moves, [])
                self.assertIsNone(obj.ordinary_home)

    def test_zero_disables_and_repeated_connect_does_not_stack(self):
        obj, kin, state = self.machine("x")
        obj.back_off_dist = 0.0
        obj.fake_printer.send_event("klippy:connect")
        kin.home(object())
        self.assertEqual(state.calls, 1)
        self.assertEqual(obj.toolhead.moves, [])

    def test_final_release_reuses_homing_acceleration_cap_and_restores_after_draining(self):
        obj, kin, _state = self.machine("x")
        obj.toolhead.accel = 50000.0
        observed = []
        move, wait = obj.toolhead.manual_move, obj.toolhead.wait_moves
        def capped_move(coords, speed):
            observed.append(("move", obj.toolhead.accel))
            move(coords, speed)
        def capped_wait():
            observed.append(("wait", obj.toolhead.accel))
            wait()
        obj.toolhead.manual_move, obj.toolhead.wait_moves = capped_move, capped_wait
        kin.home(object())
        self.assertEqual(observed, [("move", 5000.0), ("wait", 5000.0)])
        self.assertEqual(obj.toolhead.accel_log, [5000.0, 50000.0])
        self.assertEqual(obj.toolhead.accel, 50000.0)
        self.assertIsNone(obj.saved_accel)

    def test_lower_live_acceleration_is_not_raised_and_release_error_restores_cap(self):
        obj, kin, _state = self.machine("x")
        obj.toolhead.accel = 3000.0
        kin.home(object())
        self.assertEqual(obj.toolhead.accel, 3000.0)
        self.assertEqual(obj.toolhead.accel_log, [])
        for stage in ("move", "wait"):
            with self.subTest(stage=stage):
                obj, kin, _state = self.machine("x")
                obj.toolhead.accel = 50000.0
                def fail(*args):
                    self.assertEqual(obj.toolhead.accel, 5000.0)
                    raise CommandError("synthetic final release failure")
                setattr(obj.toolhead, "manual_move" if stage == "move" else "wait_moves", fail)
                with self.assertRaises(CommandError):
                    kin.home(object())
                self.assertEqual(obj.toolhead.accel_log, [5000.0, 50000.0])
                self.assertEqual(obj.toolhead.accel, 50000.0)
                self.assertIsNone(obj.saved_accel)
        obj, kin, _state = self.machine("x")
        obj.toolhead.accel = 50000.0
        setter = obj.toolhead.set_max_velocities
        interrupted = [False]
        def motor_off_in_setter(velocity, accel, scv, mcr):
            setter(velocity, accel, scv, mcr)
            if not interrupted[0]:
                interrupted[0] = True
                obj.fake_printer.send_event("stepper_enable:motor_off")
        obj.toolhead.set_max_velocities = motor_off_in_setter
        with self.assertRaisesRegex(CommandError, "interrupted"):
            kin.home(object())
        self.assertEqual(obj.toolhead.moves, [])
        self.assertEqual(obj.toolhead.accel, 50000.0)
        self.assertIsNone(obj.saved_accel)


def noise_flow(obj, mode="valid"):
    previous = [None]
    emitted = [0]

    def flow():
        now = obj.main_mcu.estimated_print_time(obj.reactor.monotonic())
        emitted[0] += 1
        if mode == "missing":
            return
        rate = 1000.0 if obj.backend is None else 3200.0
        size = 5 if obj.backend is None else 16
        start = now - 0.001 - (size - 1) / rate
        if mode == "stale":
            start -= 0.1
        elif mode == "future":
            start += 0.1
        elif mode == "duplicate" and previous[0] is not None:
            start = previous[0]
        elif mode == "gap" and emitted[0] == 3:
            obj.reactor.now += 0.05
            start += 0.05
        previous[0] = start
        values = []
        for number in range(size):
            delta = 0.0 if mode == "frozen" else (0.02 if (emitted[0] * size + number) % 2 else -0.02)
            x = 16.0 if mode == "clip" else 0.25 + delta
            values.append((x, -0.1 - delta, 1.0 + delta))
        if mode == "map":
            if obj.backend is None:
                obj.helper.config.axes_map[0] = (1, 1)
            else:
                obj.backend.sensor.axes_map = [(0, 1), (1, 1), (2, 1)]
        if obj.backend is None:
            fields = batch(round(start * 1000.0),
                           [tuple(counts(value) for value in row) for row in values],
                           receive_time=obj.reactor.monotonic(), delta_clock=size - 1)
            if mode == "clip":
                fields["data"] = plugin.SAMPLE.pack(plugin.CLIPPED, 0, counts(1.0)) * size
            if mode == "error":
                fields["errors"] = 1
            obj._inspect(fields)
        else:
            rows = [(start + number / rate,) + tuple(value * 9806.65 for value in row)
                    for number, row in enumerate(values)]
            obj.backend.helper.emit({"data": rows, **({"errors": 1} if mode == "error" else {})})
    obj.reactor.on_pause = flow


class NoiseAcquisitionTests(unittest.TestCase):
    def machine(self, decoded=False, legacy=False):
        obj = generic(legacy=legacy) if decoded else make_homing()
        obj._end_session()
        obj.reactor.now = 10.0
        return obj

    def test_native_and_both_decoded_contracts_capture_three_disjoint_live_windows(self):
        for decoded, legacy in ((False, False), (True, False), (True, True)):
            with self.subTest(decoded=decoded, legacy=legacy):
                obj = self.machine(decoded, legacy)
                noise_flow(obj)
                context = obj.start_noise_measurement()
                captures = [obj.collect_noise_window(lambda: None) for _ in range(3)]
                obj.stop_noise_measurement()
                for capture in captures:
                    self.assertEqual(capture["_noise_context"], context)
                    self.assertGreaterEqual(capture["samples"][-1][0] - capture["samples"][0][0] + 1e-9, 1.0)
                    self.assertGreater(capture["samples"][0][0], capture["cutoff_time_s"])
                self.assertGreater(captures[1]["samples"][0][0], captures[0]["samples"][-1][0])
                self.assertGreater(captures[2]["samples"][0][0], captures[1]["samples"][-1][0])
                self.assertEqual(obj.toolhead.moves, [])
                self.assertEqual(obj.toolhead.dwells, [])
                self.assertFalse(obj.stream_on)
                self.assertIsNone(obj.noise_session)

    def test_missing_stale_future_duplicate_gap_clip_error_frozen_and_map_fail_without_motion(self):
        for decoded in (False, True):
            for mode in ("missing", "stale", "future", "duplicate", "gap", "clip", "error", "frozen", "map"):
                with self.subTest(decoded=decoded, mode=mode):
                    obj = self.machine(decoded)
                    noise_flow(obj, mode)
                    obj.start_noise_measurement()
                    with self.assertRaises(CommandError):
                        obj.collect_noise_window(lambda: None)
                    obj.stop_noise_measurement()
                    self.assertEqual(obj.toolhead.moves, [])
                    self.assertFalse(obj.stream_on)
                    self.assertIsNone(obj.noise_window)

    def test_shared_sensor_and_bounded_overflow_are_rejected(self):
        for decoded in (False, True):
            obj = self.machine(decoded)
            if decoded:
                obj.helper.add_client(lambda message: True)
            else:
                obj.helper._stream_en = 1
            with self.assertRaisesRegex(CommandError, "exclusive"):
                obj.start_noise_measurement()
            self.assertIsNone(obj.noise_session)
        obj = self.machine()
        obj.capture_limit = 10
        noise_flow(obj)
        obj.start_noise_measurement()
        with self.assertRaises(CommandError):
            obj.collect_noise_window(lambda: None)
        self.assertLessEqual(len(obj.noise_window["samples"]) if obj.noise_window else 0, 10)
        obj.stop_noise_measurement()

    def test_disconnect_cancels_acquisition_and_native_scale_switch_is_seen_in_batch(self):
        for mode in ("disconnect", "scale"):
            obj = self.machine()
            noise_flow(obj)
            flow = obj.reactor.on_pause
            def interrupt():
                if mode == "disconnect":
                    obj.fake_printer.send_event("klippy:disconnect")
                else:
                    obj.helper._scale = dict(obj.helper._scale, id=42)
                    flow()
                    obj.helper._scale = dict(obj.scale)
            obj.reactor.on_pause = interrupt
            obj.start_noise_measurement()
            with self.assertRaises(CommandError):
                obj.collect_noise_window(lambda: None)
            obj.stop_noise_measurement()
            self.assertEqual(obj.toolhead.moves, [])

    def test_second_window_timing_fault_retains_signed_ages_without_replacing_homing_baseline(self):
        for decoded in (False, True):
            for mode in ("stale", "future"):
                with self.subTest(decoded=decoded, mode=mode):
                    obj = self.machine(decoded)
                    count_g = obj.scale["scale"] / 1000.0
                    reference = plugin.QuietReference("x", 9.0, 0.1, 0.2 / count_g,
                                                      threshold_g=0.5)
                    reference.add(9.5, (0.0, 0.0, 1.0 / count_g))
                    obj._remember_reference(reference, 9.50878, "acquisition", "acquisition_timeout")
                    previous_baseline = dict(obj.baseline_status)
                    noise_flow(obj)
                    obj.start_noise_measurement()
                    capture = obj.collect_noise_window(lambda: None)
                    completed_samples = list(capture["samples"])
                    noise_flow(obj, mode)
                    with self.assertRaisesRegex(CommandError, "stale or in the future"):
                        obj.collect_noise_window(lambda: None)
                    obj.stop_noise_measurement()
                    fault = obj.get_status(obj.reactor.now)["sample_timing_fault"]
                    self.assertEqual(fault["context"], "stationary_noise")
                    self.assertEqual(fault["window"], 2)
                    self.assertEqual(fault["reason"], "too_old" if mode == "stale" else "future")
                    self.assertEqual(fault["phase"], "backend_batch" if decoded else "noise_batch")
                    self.assertGreater(fault["cutoff_time_s"], completed_samples[-1][0])
                    self.assertGreater(fault["newest_sample_time_s"], fault["oldest_sample_time_s"])
                    if mode == "stale":
                        self.assertGreater(fault["sample_age_ms"], fault["max_sample_age_ms"])
                    else:
                        self.assertLess(fault["sample_age_ms"], -fault["max_future_ms"])
                    self.assertGreaterEqual(fault["oldest_batch_age_ms"], fault["newest_batch_age_ms"])
                    self.assertEqual(obj.baseline_status, previous_baseline)
                    self.assertEqual(capture["samples"], completed_samples)
                    # Later callbacks and status callers cannot replace or mutate the first fault.
                    obj._record_sample_timing_fault("later_callback", 100.0, 1.0, 2.0)
                    self.assertEqual(obj.sample_timing_fault, fault)
                    fault["sample_age_ms"] = 9999
                    self.assertNotEqual(obj.sample_timing_fault["sample_age_ms"], 9999)
                    command = GCmd()
                    obj.cmd_STATUS(command)
                    self.assertTrue(any("sample timing fault" in message and '"window": 2' in message
                                        for message in command.responses))
                    self.assertEqual(obj.toolhead.moves, [])
                    self.assertEqual(obj.threshold_g, {"x": 8.0, "y": 8.0})
                    # A new stream has independent evidence; old diagnostics are not reused.
                    obj.start_noise_measurement()
                    self.assertIsNone(obj.sample_timing_fault)
                    obj.stop_noise_measurement()

    def test_noise_wait_diagnostic_uses_shorter_stream_timeout(self):
        obj = generic(options={"stream_timeout": 0.01})
        obj._end_session()
        obj.reactor.now = 10.0
        noise_flow(obj)
        first_batch = obj.reactor.on_pause
        def flow_once():
            first_batch()
            obj.reactor.on_pause = None
        obj.reactor.on_pause = flow_once
        obj.start_noise_measurement()
        with self.assertRaisesRegex(CommandError, "stopped receiving fresh samples"):
            obj.collect_noise_window(lambda: None)
        obj.stop_noise_measurement()
        fault = obj.sample_timing_fault
        self.assertEqual(fault["phase"], "noise_wait")
        self.assertEqual(fault["window"], 1)
        self.assertEqual(fault["max_sample_age_ms"], 10.0)
        self.assertGreater(fault["sample_age_ms"], 10.0)
        self.assertLess(fault["sample_age_ms"], obj.backend.max_age * 1000.0)


class NoiseRig(Rig):
    def __init__(self, options=None, amplitudes=(2.0, 0.05, 0.02)):
        super().__init__(options)
        self.amplitudes = amplitudes


class NoiseWorkflowTests(unittest.TestCase):
    def test_failed_second_window_exposes_its_timing_fault_and_keeps_completed_window(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            fault = {"phase": "backend_batch", "context": "stationary_noise",
                     "reason": "too_old", "window": 2, "sample_age_ms": 27.0,
                     "oldest_batch_age_ms": 27.0, "newest_batch_age_ms": 22.0,
                     "max_sample_age_ms": 20.0, "max_future_ms": 5.0,
                     "cutoff_time_s": 11.2}
            def fail_second(iteration):
                if iteration == 2:
                    rig.owner.sample_timing_fault = dict(fault)
                    raise CommandError("accelerometer sample timestamps are stale or in the future")
            rig.on_noise = fail_second
            with self.assertRaisesRegex(CommandError, "stale or in the future"):
                rig.workflow.cmd_NOISE(GCmd())
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            self.assertEqual(result["timing_fault"], fault)
            self.assertEqual(len(result["iterations"]), 1)
            self.assertEqual(result["iterations"][0]["record"]["sample_count"], 1001)
            self.assertNotIn("timing_fault", result["iterations"][0]["record"])
            self.assertIsNone(result["recommendation"])
            self.assertEqual(rig.workflow.noise_seeds, {})
            rig.owner.sample_timing_fault["sample_age_ms"] = 99.0
            self.assertEqual(result["timing_fault"]["sample_age_ms"], 27.0)
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.configfile.saved, {})

    def test_failure_before_stream_start_does_not_reuse_previous_timing_fault(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            rig.owner.sample_timing_fault = {"context": "stationary_noise", "window": 2}
            def fail_start():
                raise CommandError("sensor or homing capture is busy")
            rig.owner.start_noise_measurement = fail_start
            with self.assertRaisesRegex(CommandError, "busy"):
                rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.workflow.last_result["result"], "aborted")
            self.assertNotIn("timing_fault", rig.workflow.last_result)
            self.assertEqual(rig.workflow.last_result["iterations"], [])

    def test_three_window_charts_and_transient_candidate_seed_without_motion_or_config_writes(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            rig.toolhead.kin.limits[:2] = [(0.0, 300.0), (0.0, 200.0)]
            before = rig.effects()
            rig.workflow.cmd_NOISE(GCmd())
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
            self.assertEqual(result["recommendation"]["start_threshold_g"], {"x": 2.5, "y": 1.0})
            self.assertEqual(rig.workflow.noise_seeds, {"x": 2.5, "y": 1.0})
            self.assertEqual(rig.scripts, [])
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.toolhead.dwells, [])
            self.assertEqual(rig.disabled, [])
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(rig.owner.threshold_g, before["thresholds"])
            self.assertEqual(rig.toolhead.position, before["position"])
            self.assertEqual(rig.toolhead.kin.limits, before["limits"])
            for paths in [result["summary_paths"]]:
                self.assertTrue(all(Path(path).is_file() for path in paths.values()))
            self.assertTrue(all(not item["capture_paths"] for item in result["iterations"]))
            self.assertEqual(len([path for path in Path(temp).rglob("*") if path.is_file()]), 1)
            for item in result["iterations"]:
                self.assertAlmostEqual(item["record"]["noise_peak_g"]["z"], 0.02)
                self.assertAlmostEqual(item["record"]["baseline_g"]["z"], 1.0)
            rig.toolhead.kin.limits[:2] = [(1.0, -1.0), (1.0, -1.0)]
            rig.calibrate(save=0)
            self.assertEqual([call["threshold"] for call in rig.home_calls], [2.5] * 11)
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"] * 2)
            self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})
            self.assertEqual(rig.workflow.noise_seeds, {})

    def test_only_explicit_start_bypasses_fresh_magnitude_seed(self):
        for parameters, expected_start, expected_step in (
                ({"START_THRESHOLD": 1, "STEP": .25}, 1.0, .25),
                ({"START_THRESHOLD": 1}, 1.0, .5),
                ({"STEP": .25}, 2.5, .25)):
            with self.subTest(parameters=parameters), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp, "calibration_start_threshold": .5,
                                "calibration_step": .5})
                rig.workflow.cmd_NOISE(GCmd())
                seeds = dict(rig.workflow.noise_seeds)
                self.assertGreater(seeds["x"], .5)
                rig.calibrate(save=0, **parameters)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [expected_start] * 11)
                self.assertEqual(rig.workflow.last_result["start_threshold_g"], expected_start)
                self.assertEqual(rig.workflow.last_result["threshold_step_g"], expected_step)
                self.assertEqual(rig.workflow.noise_seeds, {})
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
                self.assertEqual("stationary_noise_seed" in metadata, "START_THRESHOLD" not in parameters)

    def test_idle_queue_preflight_rejects_even_fifty_ms_flushed_motion_and_no_stock_g28_needed(self):
        for busy in ((10.05, 10.0, True), (10.0, 10.0, False)):
            rig = NoiseRig()
            rig.toolhead.busy = busy
            with self.assertRaisesRegex(CommandError, "queued motion"):
                rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.noise_calls, [])
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            rig.objects["homing_override"] = object()
            rig.gcode.gcode_handlers["G28"] = lambda g: None
            rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.scripts, [])

    def test_abort_cancel_motor_change_disconnection_and_export_error_never_publish_seed(self):
        for failure in ("cancel", "disconnect", "motor", "token", "capture_export", "summary_export"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp})
                def interrupt(iteration):
                    if iteration != 3:
                        return
                    if failure == "cancel":
                        rig.workflow.cmd_CANCEL(GCmd())
                    elif failure == "disconnect":
                        rig.workflow._disconnect()
                    elif failure == "motor":
                        rig.lines["stepper_x"].enabled = False
                    elif failure == "token":
                        rig.owner.noise_session = None
                rig.on_noise = interrupt
                target = "write_noise_capture" if failure == "capture_export" else "write_noise"
                context = mock.patch.object(workflow_module, target, side_effect=OSError("synthetic export failure")) \
                    if failure.endswith("export") else contextlib.nullcontext()
                with context, mock.patch.object(workflow_module.logging, "exception"), self.assertRaises(Exception):
                    rig.workflow.cmd_NOISE(GCmd())
                self.assertEqual(rig.workflow.noise_seeds, {})
                self.assertIsNone(rig.workflow.noise_recommendation)
                self.assertIsNone(rig.workflow.session)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_context_change_before_calibration_discards_seed_and_over_ceiling_preserves_measurement(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            rig.workflow.cmd_NOISE(GCmd())
            rig.owner.rails_by_axis["x"].info.speed = 80.0
            rig.calibrate(save=0)
            self.assertEqual(rig.home_calls[0]["threshold"], 2.5)
            self.assertEqual(rig.workflow.noise_seeds, {})
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            rig.owner.full_scale_g = 2.5
            rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.workflow.last_result["result"], "complete")
            self.assertIsNone(rig.workflow.last_result["recommendation"]["start_threshold_g"]["x"])
            self.assertEqual(rig.workflow.noise_seeds, {"y": 1.0})
        for unknown_range in (False, True):
            with self.subTest(unknown_range=unknown_range), tempfile.TemporaryDirectory() as temp:
                rig = NoiseRig({"charts_directory": temp},
                               amplitudes=(2.0, 2.0, 0.02))
                rig.owner.full_scale_g = None if unknown_range else 2.5
                rig.workflow.cmd_NOISE(GCmd())
                self.assertEqual(rig.workflow.last_result["result"], "complete")
                self.assertEqual(rig.workflow.last_result["recommendation"]["start_threshold_g"],
                                 {"x": None, "y": None})
                self.assertEqual(rig.workflow.noise_seeds, {})

    def test_motor_off_after_last_capture_export_and_failed_rerun_clear_cache(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            export = workflow_module.write_noise_capture
            def interrupt_export(directory, run_id, iteration, capture, record, **kwargs):
                paths = export(directory, run_id, iteration, capture, record, **kwargs)
                if iteration == 3:
                    rig.owner.noise_session = None
                return paths
            with mock.patch.object(workflow_module, "write_noise_capture", side_effect=interrupt_export):
                with self.assertRaisesRegex(CommandError, "sensor session"):
                    rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.workflow.noise_seeds, {})
            self.assertEqual(rig.workflow.last_result["result"], "aborted")


class PNGIntegrationTests(unittest.TestCase):
    def test_motor_off_generation_during_noise_png_render_prevents_seed_even_if_motors_stay_disabled(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            for line in rig.lines.values():
                line.enabled = False
            def interrupt(payload):
                if payload["kind"] == "noise":
                    rig.owner.operation_generation += 1
            rig.workflow.renderer.on_render = interrupt
            with mock.patch.object(workflow_module.logging, "exception"), self.assertRaises(CommandError):
                rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.workflow.noise_seeds, {})
            self.assertIsNone(rig.workflow.noise_recommendation)
            self.assertEqual(rig.workflow.last_result["result"], "aborted")

    def test_missing_renderer_or_cancelled_preflight_never_homes_or_releases_motors(self):
        for mode in ("charts", "accuracy", "noise"):
            for failure in ("missing", "cancel"):
                with self.subTest(mode=mode, failure=failure):
                    rig = NoiseRig() if mode == "noise" else Rig()
                    before = rig.effects()
                    def fail(payload):
                        if failure == "cancel":
                            rig.workflow.cmd_CANCEL(GCmd())
                        else:
                            raise RuntimeError("missing matplotlib")
                    rig.workflow.renderer.on_render = fail
                    callback = {"charts": rig.workflow.cmd_CHARTS,
                                "accuracy": rig.workflow.cmd_ACCURACY,
                                "noise": rig.workflow.cmd_NOISE}[mode]
                    with self.assertRaisesRegex(CommandError, "PNG renderer"):
                        callback(GCmd(CENTERED=1))
                    self.assertEqual(rig.scripts, [])
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.disabled, [])
                    self.assertEqual(rig.toolhead.position, before["position"])
                    self.assertEqual(rig.toolhead.kin.limits, before["limits"])
                    self.assertIsNone(rig.workflow.session)

    def test_chart_uses_verified_precise_detector_threshold_and_rejects_wrong_source(self):
        for wrong in (False, True):
            with self.subTest(wrong_source=wrong), tempfile.TemporaryDirectory() as temp:
                rig = Rig({"charts_directory": temp})
                rig.owner.threshold_g.update(x=0.001, y=0.002)
                rig.home_results.extend([{"threshold_g": 0.0}, {"threshold_g": 0.0}])
                if wrong:
                    original = rig.owner.stop_capture
                    def substituted():
                        capture = original()
                        capture["_record"] = dict(capture["_record"])
                        return capture
                    rig.owner.stop_capture = substituted
                    with self.assertRaisesRegex(CommandError, "current detector"):
                        rig.workflow.cmd_CHARTS(GCmd(CENTERED=1))
                    self.assertEqual(rig.toolhead.moves, [])
                else:
                    rig.workflow.cmd_CHARTS(GCmd(CENTERED=1))
                    captures = [item for item in rig.workflow.renderer.calls if item["kind"] == "capture"]
                    self.assertEqual([item["record"]["threshold_g"] for item in captures], [0.001, 0.002])
                    self.assertEqual(rig.owner.threshold_g, {"x": 0.001, "y": 0.002})

    def test_same_user_socket_data_path_correlation_finds_root_and_ignores_nested_main_parent(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            data, proc = base / "custom_printer_data", base / "proc"
            (data / "config/nested").mkdir(parents=True)
            socket = data / "comms/klippy.sock"
            (data / "config/moonraker.conf").write_text(
                "[server]\nklippy_uds_address: %s\n" % socket, encoding="utf-8")
            process = proc / "123"
            (process / "cwd").mkdir(parents=True)
            (process / "cmdline").write_bytes(b"python\0moonraker.py\0-d\0" + str(data).encode() + b"\0")
            (process / "environ").write_bytes(b"")
            start = {"apiserver": str(socket), "config_file": str(data / "config/nested/printer.cfg")}
            with mock.patch.object(workflow_module.os, "getuid", return_value=process.stat().st_uid, create=True):
                found = workflow_module.verified_moonraker_config_root(start, proc)
                self.assertEqual(Path(found), (data / "config").resolve())
                self.assertIsNone(workflow_module.verified_moonraker_config_root(
                    {"apiserver": str(base / "different.sock")}, proc))
                (data / "config/moonraker.conf").write_text("[server]\n[include unknown.cfg]\n", encoding="utf-8")
                self.assertIsNone(workflow_module.verified_moonraker_config_root(start, proc))

    def test_inline_png_urls_are_escaped_and_unverified_outside_paths_have_no_img(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = Rig()
            root = Path(temp) / "config"
            root.mkdir()
            png = root / "plot & date.png"
            png.write_bytes(b"fixture")
            rig.workflow.chart_config_root = str(root)
            rig.workflow._show_pngs({"png": str(png)}, '<measured "chart">')
            markup = [line for line in rig.output if "<img" in line]
            self.assertEqual(len(markup), 1)
            self.assertIn('/server/files/config/plot%20%26%20date.png', markup[0])
            self.assertIn('&lt;measured &quot;chart&quot;&gt;', markup[0])
            self.assertNotIn('<measured', markup[0])
            rig.output.clear()
            rig.workflow._show_pngs({"png": str(Path(temp) / "outside.png")}, "outside")
            self.assertFalse(any("<img" in line for line in rig.output))

    def test_fluidd_known_command_linkification_cannot_corrupt_png_attributes(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = Rig()
            root = Path(temp) / "config"
            png = root / "PRINT_START/PRINT_START.png"
            png.parent.mkdir(parents=True)
            png.write_bytes(b"fixture")
            rig.workflow.chart_config_root = str(root)
            rig.workflow._show_pngs({"png": str(png)}, "PRINT_START")
            markup = next(line for line in rig.output if "<img" in line)
            linked = re.sub(r"([A-Z_][A-Z0-9_.]+)", lambda match:
                            '<a class="command">PRINT_START</a>' if match.group() == "PRINT_START"
                            else match.group(), markup)
            self.assertEqual(linked, markup)
            attributes = []
            class Parser(HTMLParser):
                def handle_starttag(self, tag, attrs):
                    attributes.append((tag, dict(attrs)))
            Parser().feed(linked)
            expected = "/server/files/config/PRINT_START/PRINT_START.png"
            self.assertEqual(attributes[0][1]["href"], expected)
            self.assertEqual(attributes[2][1]["src"], expected)
        with tempfile.TemporaryDirectory() as temp:
            rig = NoiseRig({"charts_directory": temp})
            rig.workflow.cmd_NOISE(GCmd())
            self.assertTrue(rig.workflow.noise_seeds)
            rig.toolhead.busy = (10.05, 10.0, True)
            with self.assertRaises(CommandError):
                rig.workflow.cmd_NOISE(GCmd())
            self.assertEqual(rig.workflow.noise_seeds, {})

    def test_terminal_release_failure_or_motor_off_cannot_stage_calibration(self):
        for interrupt in ("motor_off", "exception"):
            rig = Rig()
            wait = rig.toolhead.wait_moves
            def final_wait():
                wait()
                if len(rig.home_calls) == 11 and len(rig.toolhead.moves) == 11:
                    if interrupt == "exception":
                        raise CommandError("terminal release failed")
                    rig.owner.operation_generation += 1
            rig.toolhead.wait_moves = final_wait
            with mock.patch.object(workflow_module.logging, "exception"), self.assertRaises(CommandError):
                rig.calibrate(save=0)
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(rig.workflow.last_result["result"], "aborted")


if __name__ == "__main__":
    unittest.main()
