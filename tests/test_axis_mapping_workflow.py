"""Offline bidirectional mapping, cleanup and persistence contracts."""
import copy
import json
import math
import types
import unittest
from unittest import mock

from harness import CommandError, ConfigError
from test_workflows import AccelHomingWorkflows, Config, GCmd, Rig, workflow_module
from accel_homing_model import AccelHomingModel


class RealCaptureOwner:
    """Use actual acquisition/stream state with a simulated motion controller."""
    sensor_fields = {"sensor_fault", "sample_timing_fault", "stream_on", "backend", "noise_session",
                     "noise_window", "beta_motion_capture", "beta_capture", "capture_batches",
                     "sensor_name", "noise_sensor_context", "sensor_fault_lock", "_noise_shared_client", "helper",
                     "pending_move", "capture_attempt"}

    def __init__(self, owner, sensor):
        object.__setattr__(self, "owner", owner)
        object.__setattr__(self, "sensor", sensor)

    def __getattr__(self, name):
        return getattr(self.sensor if name in self.sensor_fields else self.owner, name)

    def __setattr__(self, name, value):
        setattr(self.sensor if name in self.sensor_fields else self.owner, name, value)


def real_reference_rig(decoded):
    from test_noise_recovery import sensor
    from test_retract_noise import noise_flow
    rig, obj = MappingRig(), sensor(decoded)
    obj.reactor = rig.reactor
    obj.reactor.now = 10.
    base = rig.owner
    rig.owner = RealCaptureOwner(base, obj)
    rig.workflow.owner = rig.owner
    rig.fresh_references = []

    def reference(check):
        try:
            token = obj.start_beta_motion_capture(check)
            rig.fresh_references.append(token)
            raise CommandError("stop after real fresh reference")
        finally:
            # This fixture deliberately stops before any stroke. Stream setup,
            # reference acquisition and its fault reset above remain real.
            obj.beta_motion_capture = None
            obj.stop_noise_measurement()

    base.start_beta_motion_capture = reference
    base.stop_beta_motion_capture = obj.stop_beta_motion_capture
    noise_flow(obj)
    return rig, obj


class MappingRig(Rig):
    def __init__(self, axis="x", source=2, sign=-1):
        super().__init__()
        self.axis, self.source, self.sign = axis, source, sign
        self.clock, self.profile, self.events = 10., [], []
        self.fail, self.interrupt = None, None
        self.toolhead.min_cruise_ratio = .5
        self.toolhead.get_last_move_time = lambda: self.clock
        self.owner.capture_limit, self.owner.stream_timeout = 100000, .05
        self.owner._enable_motors = lambda axis: self.event("enable")
        self.owner.beta_motion_capture = None
        self.owner.start_beta_motion_capture = self.start_mapping_capture
        self.owner.stop_beta_motion_capture = self.stop_mapping_capture
        self.objects["motion_report"] = types.SimpleNamespace(trapqs={
            "toolhead": types.SimpleNamespace(extract_trapq=self.extract_profile)})
        original_move = self.toolhead.manual_move

        def move(target, speed):
            current_axis = self.workflow.session["axis"]
            direction = 1 if target["xy".index(current_axis)] > self.toolhead.position["xy".index(current_axis)] else -1
            distance = abs(target["xy".index(current_axis)] - self.toolhead.position["xy".index(current_axis)])
            accel, start = self.toolhead.accel, self.clock + .01
            ramp, cruise = speed / accel, distance / speed - speed / accel
            vector = [0., 0., 0.]
            vector["xy".index(current_axis)] = direction
            self.profile = [types.SimpleNamespace(print_time=t, move_t=d, start_v=v, accel=a,
                                                  x_r=vector[0], y_r=vector[1], z_r=0.)
                            for t, d, v, a in ((start, ramp, 0., accel),
                                (start + ramp, cruise, speed, 0.),
                                (start + ramp + cruise, ramp, speed, -accel))]
            self.clock = start + ramp * 2 + cruise
            self.direction = direction
            original_move(target, speed)
            self.event("move")

        self.toolhead.manual_move = move
        self.workflow._calibration_axis_mapping = types.MethodType(
            AccelHomingWorkflows._calibration_axis_mapping, self.workflow)

    def event(self, stage):
        self.events.append(stage)
        if self.interrupt is not None:
            self.interrupt(stage)
        if self.fail == stage:
            raise CommandError("synthetic " + stage + " failure")

    def extract_profile(self, start, end):
        self.event("profile")
        return self.profile, None

    def start_mapping_capture(self, check):
        check()
        self.event("reference")
        token = {"sample_rate_hz": 1000., "capture_cutoff_time_s": self.clock}
        self.owner.beta_motion_capture = token
        return token

    def stop_mapping_capture(self, token, check):
        try:
            check()
            self.event("capture")
            start, end = token["capture_cutoff_time_s"], token["motion_end_time_s"]
            rows = []
            for number in range(math.ceil((end - start) * 1000.) + 2):
                t = start + number / 1000.
                values = [0., 0., 1.]
                for segment in self.profile:
                    if segment.print_time <= t < segment.print_time + segment.move_t:
                        values[self.source] += self.sign * self.direction * segment.accel / 9806.65
                rows.append((t,) + tuple(values))
            return {"samples": rows, "baseline_g": (0., 0., 1.), "dropped": 0, "fault": None,
                    "sensor_context": self.sensor_context(), "raw_samples_validated": True}
        finally:
            self.owner.beta_motion_capture = None

    def begin_mapping(self):
        self.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", self.axis)
        return self.workflow.session


class AxisMappingWorkflowTests(unittest.TestCase):
    def test_kalico_current_helper_receives_selected_axis_and_restores_current(self):
        rig = MappingRig(axis="y")
        command = GCmd("ACCEL_HOMING_CALIBRATE", AXIS="Y", CENTERED=1, SAVE_CONFIG=0)
        states, current_calls = [], []

        class Homing:
            def __init__(self, printer, gcmd):
                states.append((printer, gcmd.get_commandline(), gcmd.get_command_parameters()))

            def _set_homing_current(self, axes, pre_homing):
                current_calls.append((list(axes), pre_homing))

        with mock.patch.dict(workflow_module._PRINTER_G28.__globals__, {"Homing": Homing}), \
                mock.patch.object(rig.toolhead, "get_active_rails_for_axis", create=True), \
                mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(rig.workflow, "_calibration_noise_preflight",
                                  side_effect=CommandError("stop after mapping")), \
                self.assertRaisesRegex(CommandError, "stop after mapping"):
            rig.workflow.cmd_CALIBRATE(command)
        self.assertEqual(states, [(rig.workflow.printer, "G28 Y", {"Y": ""})])
        self.assertEqual(current_calls, [(["y"], True), (["y"], False)])
        self.assertEqual(rig.events.count("capture"), 2)
        self.assertEqual(rig.home_calls, [])

    def test_native_homing_init_type_error_aborts_before_motion_without_retry(self):
        rig = MappingRig()
        attempts = []

        class Homing:
            def __init__(self, printer, gcmd=None):
                attempts.append(gcmd.get_commandline())
                raise TypeError("native initialization failed")

            def _set_homing_current(self, axes, pre_homing):
                raise AssertionError("motor current changed")

        command = GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0)
        with mock.patch.dict(workflow_module._PRINTER_G28.__globals__, {"Homing": Homing}), \
                mock.patch.object(rig.toolhead, "get_active_rails_for_axis", create=True), \
                mock.patch.object(rig.workflow, "_chart_preflight"), \
                self.assertRaisesRegex(CommandError, "before motion: native initialization failed"):
            rig.workflow.cmd_CALIBRATE(command)
        self.assertEqual(attempts, ["G28 X"])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.home_calls, [])
        self.assertIsNone(rig.workflow.session)

    def use_configured_mapping(self, rig, tokens=("x", "y", "z")):
        rig.owner.sensor_name = "adxl345 toolhead"
        declaration = {"sensor_section": rig.owner.sensor_name, "option": "axes_map", "axes_map": list(tokens)}
        rig.axis_declaration = declaration
        def configured(axis):
            if rig.axis_declaration is None:
                return None
            return {"version": 1, "axis": axis, "source_axis": "xy".index(axis), "sign": 1,
                    "coordinate_frame": "configured_sensor_stream", "method": "configured_axes_map_v1",
                    "configuration": copy.deepcopy(rig.axis_declaration)}
        rig.owner.configured_axis_mapping = configured
        return declaration

    def test_explicit_identity_permutation_and_sign_skip_only_mapping_motion(self):
        for axis in "xy":
            for tokens in (("x", "y", "z"), ("z", "x", "y"), ("-y", "x", "-z")):
                with self.subTest(axis=axis, declaration=tokens):
                    rig = MappingRig(axis)
                    declaration = self.use_configured_mapping(rig, tokens)
                    initial, limits, accel = rig.toolhead.get_position(), list(rig.toolhead.kin.limits), rig.toolhead.accel
                    session = rig.begin_mapping()
                    del rig.objects["motion_report"]
                    del rig.owner.start_beta_motion_capture
                    del rig.owner.stop_beta_motion_capture
                    with mock.patch.object(rig.workflow, "_axis_mapping_plan", side_effect=AssertionError("mapping plan requested")):
                        rig.workflow._calibration_axis_mapping(session)
                    mapping = session["axis_mapping"]
                    self.assertEqual((mapping["source_axis"], mapping["sign"]), ("xy".index(axis), 1))
                    self.assertEqual(mapping["configuration"], declaration)
                    self.assertEqual(mapping["method"], "configured_axes_map_v1")
                    self.assertNotIn("confidence", mapping)
                    self.assertNotIn("passes", mapping)
                    self.assertEqual(rig.toolhead.position, initial)
                    self.assertEqual(rig.toolhead.kin.limits, limits)
                    self.assertEqual(rig.toolhead.accel, accel)
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.events, [])
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(session["axis_mapping_phase"], "qualified")
                    self.assertTrue(any("explicitly configured" in line for line in rig.output))
                    self.assertFalse(any("bidirectional free-motion evidence qualified" in line for line in rig.output))

    def test_absent_explicit_declaration_keeps_bidirectional_mapping(self):
        rig = MappingRig()
        rig.owner.configured_axis_mapping = mock.Mock(return_value=None)
        session = rig.begin_mapping()
        rig.workflow._calibration_axis_mapping(session)
        self.assertEqual(session["axis_mapping"]["method"], "bidirectional_velocity_v1")
        self.assertEqual(rig.events.count("reference"), 2)
        self.assertEqual(rig.events.count("capture"), 2)
        self.assertEqual(len(rig.toolhead.moves), 4)

    def test_configured_mapping_still_measures_fresh_noise_calibrates_and_persists(self):
        for axis in "xy":
            with self.subTest(axis=axis):
                rig = MappingRig(axis)
                declaration = self.use_configured_mapping(rig, ("-y", "x", "-z"))
                initial, limits = rig.toolhead.get_position(), list(rig.toolhead.kin.limits)
                rig.amplitudes = (.6, 7., 5.) if axis == "x" else (7., .6, 5.)
                original_noise = rig.workflow._calibration_noise_preflight
                def noise(session):
                    self.assertEqual(rig.events, [])
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.toolhead.position, initial)
                    self.assertEqual(rig.toolhead.kin.limits, limits)
                    original_noise(session)
                with mock.patch.object(rig.workflow, "_calibration_noise_preflight", side_effect=noise), \
                        mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
                    rig.calibrate(axis.upper(), save=0)
                self.addCleanup(rig.calibration_charts.cleanup)
                self.assertEqual(rig.noise_window_count, 3)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertTrue(all(home["axis"] == axis for home in rig.home_calls))
                self.assertEqual(rig.home_calls[0]["threshold"], 1.)
                self.assertEqual(rig.events.count("reference"), 0)
                self.assertEqual(rig.events.count("capture"), 0)
                self.assertEqual(rig.events.count("profile"), 0)
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertTrue(all(move[2] is None for move, speed in rig.toolhead.moves))
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", axis + "_metadata")])
                self.assertEqual(metadata["axis_mapping"]["configuration"], declaration)
                self.assertEqual(metadata["axis_mapping"]["source_axis"], "xy".index(axis))
                self.assertEqual(metadata["impact_metric"], "mapped_axis")
                self.assertAlmostEqual(metadata["stationary_noise_measurement"]["noise_axis_peak_g"], .6)
                AccelHomingModel(Config(None, {axis + "_threshold": rig.workflow.last_result["threshold_g"],
                                             axis + "_successes": 10, axis + "_metadata": json.dumps(metadata)},
                                       "accel_homing_model configured"))

    def test_configured_shortcut_preserves_center_unhomed_busy_and_fault_guards(self):
        for fault in ("center", "homed", "busy", "sensor"):
            with self.subTest(fault=fault):
                rig = MappingRig()
                self.use_configured_mapping(rig)
                if fault == "homed": rig.toolhead.kin.limits[0] = (0., 300.)
                elif fault == "busy": rig.idle_state = "Printing"
                elif fault == "sensor": rig.owner.sensor_fault = "sample_gap"
                initial, limits = rig.toolhead.get_position(), list(rig.toolhead.kin.limits)
                with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
                    rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=0 if fault == "center" else 1, SAVE_CONFIG=0))
                self.assertEqual(rig.events, [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.toolhead.position, initial)
                self.assertEqual(rig.toolhead.kin.limits, limits)
                self.assertEqual(rig.noise_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_configured_provenance_is_rechecked_after_noise_receipts_and_final_release(self):
        for boundary in ("noise", "receipt", "save"):
            with self.subTest(boundary=boundary):
                rig = MappingRig()
                self.use_configured_mapping(rig)
                if boundary == "noise":
                    rig.on_noise = lambda iteration: setattr(rig, "axis_declaration", None)
                elif boundary == "receipt":
                    original_capture = rig.owner.stop_capture
                    def capture():
                        result = original_capture()
                        rig.axis_declaration = None
                        return result
                    rig.owner.stop_capture = capture
                else:
                    original_release = rig.workflow._release_terminal_home
                    def release(axis, record):
                        original_release(axis, record)
                        rig.axis_declaration = None
                    rig.workflow._release_terminal_home = release
                with self.assertRaises((CommandError, ValueError)):
                    rig.calibrate(save=0)
                self.addCleanup(rig.calibration_charts.cleanup)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.workflow.pending_model_metadata, {})
                self.assertEqual(len(rig.home_calls), {"noise": 0, "receipt": 1, "save": 11}[boundary])
                self.assertEqual(rig.toolhead.position[2:], [40., 6.])

    def test_configured_model_requires_same_declaration_but_inferred_model_does_not(self):
        rig = MappingRig()
        self.use_configured_mapping(rig)
        mapping = rig.owner.configured_axis_mapping("x")
        metadata = {"impact_metric": "mapped_axis", "axis_mapping": mapping,
                    "axis_mapping_context": rig.sensor_context()}
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 4.}, metadata={"x": metadata})
        self.assertEqual(rig.workflow.detection_axis_mapping("x"), mapping)
        rig.owner.sensor_name = "different"
        with self.assertRaisesRegex(CommandError, "context changed"):
            rig.workflow.detection_axis_mapping("x")
        rig.owner.sensor_name = mapping["configuration"]["sensor_section"]
        for declaration in (None, {**mapping["configuration"], "axes_map": ["y", "x", "z"]}):
            rig.axis_declaration = declaration
            with self.assertRaisesRegex(CommandError, "changed|removed"):
                rig.workflow.detection_axis_mapping("x")
        inferred = {"axis": "x"}
        rig.qualify_axis_mapping(inferred)
        metadata.update(inferred)
        self.assertEqual(rig.workflow.detection_axis_mapping("x")["method"], "bidirectional_velocity_v1")
        rig.owner.sensor_name = "different"
        with self.assertRaisesRegex(CommandError, "context changed"):
            rig.workflow.detection_axis_mapping("x")

    def test_invalid_explicit_mapping_never_falls_back_to_motion_identification(self):
        rig = MappingRig()
        self.use_configured_mapping(rig)
        mapping = rig.owner.configured_axis_mapping("x")
        mapping["source_axis"] = 2
        rig.owner.configured_axis_mapping = lambda axis: mapping
        rig.begin_mapping()
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(rig.workflow, "_axis_mapping_plan", side_effect=AssertionError("fallback motion")), \
                self.assertRaises(CommandError):
            rig.workflow._run_calibration()
        self.assertEqual(rig.events, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.noise_calls, [])
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.configfile.saved, {})

    def test_inferred_to_configured_orientation_change_stages_peer_invalidation_atomically(self):
        for failure in (False, True):
            with self.subTest(failure=failure):
                rig = MappingRig()
                self.use_configured_mapping(rig)
                self.saved_pair(rig)
                rig.workflow.model.metadata["x"]["axis_mapping"]["source_axis"] = 2
                original = copy.deepcopy(rig.workflow.model.metadata)
                if failure:
                    rig.home_results.append(CommandError("synthetic home failure"))
                    with self.assertRaises(CommandError):
                        rig.calibrate(save=0)
                    self.addCleanup(rig.calibration_charts.cleanup)
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertEqual(rig.workflow.pending_model_metadata, {})
                else:
                    with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
                        rig.calibrate("X", save=0)
                        self.addCleanup(rig.calibration_charts.cleanup)
                        first_x = copy.deepcopy(rig.workflow.pending_model_metadata["x"])
                        self.assertTrue(rig.workflow.pending_model_metadata["y"]["axis_mapping_invalidated"])
                        with self.assertRaisesRegex(CommandError, "invalidated"):
                            rig.workflow.check_home_axes(["y"])
                        rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
                        rig.idle_state = "Ready"
                        rig.calibrate("Y", save=0)
                    self.assertEqual(rig.workflow.pending_model_metadata["x"], first_x)
                    self.assertNotIn("axis_mapping_invalidated", rig.workflow.pending_model_metadata["y"])
                    self.assertTrue(all(item["axis_mapping"]["method"] == "configured_axes_map_v1"
                                        for item in rig.workflow.pending_model_metadata.values()))
                self.assertEqual(rig.workflow.model.metadata, original)
                self.assertEqual(rig.events.count("reference"), 0)
                self.assertEqual(rig.toolhead.position[2:], [40., 6.])

    def test_manual_retry_after_closed_stream_fault_reaches_real_fresh_reference(self):
        for decoded in (False, True):
            with self.subTest(decoded=decoded):
                rig, obj = real_reference_rig(decoded)
                obj.sensor_fault = "sample_gap"
                self.assertFalse(obj.stream_on)
                self.assertIsNone(obj.noise_session)
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        self.assertRaisesRegex(CommandError, "stop after real fresh reference"):
                    rig.workflow._run_calibration()
                self.assertEqual(len(rig.fresh_references), 1)
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_real_reference_gap_is_fatal_and_preserved_before_manual_retry(self):
        from test_noise_recovery import gap_flow
        from test_retract_noise import noise_flow
        for decoded in (False, True):
            with self.subTest(decoded=decoded):
                rig, obj = real_reference_rig(decoded)
                gap_flow(obj, {1})
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        self.assertRaisesRegex(CommandError, "reference_positive.*sample_gap"):
                    rig.workflow._run_calibration()
                diagnostic = rig.workflow.last_result["axis_mapping_diagnostics"]
                self.assertEqual(diagnostic["phase"], "reference_positive")
                self.assertEqual(diagnostic["sensor_fault"], "sample_gap")
                self.assertTrue(diagnostic["capture_started"])
                if not decoded:
                    self.assertGreater(diagnostic["window_fault_details"]["gap_s"], .04)
                else:
                    self.assertEqual(diagnostic["backend_fault"], "sample_gap")
                self.assertIsNone(obj.noise_window)
                self.assertIsNone(obj.noise_session)
                self.assertFalse(obj.stream_on)
                self.assertEqual(rig.fresh_references, [])
                self.assertEqual(rig.toolhead.moves, [])
                saved = copy.deepcopy(diagnostic)
                # This is a second explicit operator invocation, not a retry
                # performed by either the workflow or acquisition loop.
                rig.idle_state = "Ready"
                noise_flow(obj)
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        self.assertRaisesRegex(CommandError, "stop after real fresh reference"):
                    rig.workflow._run_calibration()
                self.assertEqual(len(rig.fresh_references), 1)
                self.assertEqual(diagnostic, saved)
                retry = rig.workflow.last_result["axis_mapping_diagnostics"]
                self.assertEqual(retry["previous_closed_stream_fault"], "sample_gap")
                self.assertTrue(retry["previous_fault_cleared"])
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(rig.toolhead.moves, [])

    def test_new_same_code_native_fault_during_motor_enable_is_not_historical(self):
        rig, obj = real_reference_rig(False)
        obj.sensor_fault = "invalid_data"
        def malformed(stage):
            if stage == "enable":
                self.assertIsNone(obj.sensor_fault)
                obj._batch_hook({})
        rig.interrupt = malformed
        rig.begin_mapping()
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch("logging.exception"), \
                self.assertRaisesRegex(CommandError, "motor_enable.*invalid_data"):
            rig.workflow._run_calibration()
        diagnostic = rig.workflow.last_result["axis_mapping_diagnostics"]
        self.assertEqual(diagnostic["sensor_fault"], "invalid_data")
        self.assertEqual(diagnostic["previous_closed_stream_fault"], "invalid_data")
        self.assertTrue(diagnostic["previous_fault_cleared"])
        self.assertEqual(obj.sensor_fault, "invalid_data")
        self.assertEqual(rig.fresh_references, [])
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.toolhead.position[2:], [40., 6.])

    def test_existing_active_or_shared_sensor_ownership_never_clears_fault(self):
        for ownership in ("stream_on", "noise_session", "beta_motion_capture", "capture_batches",
                          "pending_move", "capture_attempt", "shared"):
            with self.subTest(ownership=ownership):
                rig, obj = real_reference_rig(False)
                obj.sensor_fault = "invalid_data"
                if ownership == "shared":
                    obj.helper._stream_en = True
                else:
                    setattr(obj, ownership, True if ownership == "stream_on" else object())
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        self.assertRaisesRegex(CommandError, "preflight.*invalid_data"):
                    rig.workflow._run_calibration()
                self.assertEqual(obj.sensor_fault, "invalid_data")
                self.assertFalse(rig.workflow.last_result["axis_mapping_diagnostics"]["previous_fault_cleared"])
                self.assertEqual(rig.fresh_references, [])
                self.assertEqual(rig.toolhead.moves, [])

    def test_new_faults_stop_each_stroke_return_and_capture_without_continuation(self):
        for stage, occurrence in (("reference", 1), ("move", 1), ("move", 2),
                                  ("move", 3), ("move", 4), ("capture", 1), ("capture", 2)):
            with self.subTest(stage=stage, occurrence=occurrence):
                rig = MappingRig()
                def fail(current):
                    if current == stage and rig.events.count(stage) == occurrence:
                        rig.owner.sensor_fault = "stale_data"
                rig.interrupt = fail
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        self.assertRaisesRegex(CommandError, "accelerometer fault: stale_data"):
                    rig.workflow._run_calibration()
                self.assertEqual(rig.workflow.last_result["axis_mapping_diagnostics"]["sensor_fault"], "stale_data")
                self.assertEqual(rig.events.count(stage), occurrence)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
                self.assertEqual(rig.toolhead.position[2:], [40., 6.])

    def test_mapping_fault_diagnostics_are_frozen_bounded_finite_json(self):
        rig = MappingRig()
        details = {str(index): {"rows": ["x" * 10000] * 30, "not_finite": float("inf")}
                   for index in range(100)}
        def fail(stage):
            if stage == "reference":
                rig.owner.sample_timing_fault = details
                rig.owner.sensor_fault = "sample_gap"
        rig.interrupt = fail
        rig.begin_mapping()
        with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
            rig.workflow._run_calibration()
        diagnostic = rig.workflow.last_result["axis_mapping_diagnostics"]
        encoded = json.dumps(diagnostic, allow_nan=False)
        self.assertLess(len(encoded), 45000)
        details.clear()
        self.assertEqual(json.dumps(diagnostic, allow_nan=False), encoded)

    def test_both_axes_all_channels_both_polarities_preserve_other_coordinates_and_normal_limits(self):
        for axis in "xy":
            for source in range(3):
                for sign in (-1, 1):
                    with self.subTest(axis=axis, source=source, sign=sign):
                        rig = MappingRig(axis, source, sign)
                        initial, limits, accel = rig.toolhead.get_position(), list(rig.toolhead.kin.limits), rig.toolhead.accel
                        session = rig.begin_mapping()
                        rig.workflow._calibration_axis_mapping(session)
                        self.assertEqual(session["axis_mapping"]["source_axis"], source)
                        self.assertEqual(session["axis_mapping"]["sign"], sign)
                        self.assertEqual(rig.toolhead.position, initial)
                        self.assertEqual(rig.toolhead.kin.limits, limits)
                        self.assertEqual(rig.toolhead.accel, accel)
                        index, center = "xy".index(axis), rig.workflow._geometry(axis)[0]
                        self.assertEqual([move[index] for move, speed in rig.toolhead.moves],
                                         [center + 15., center, center - 15., center])
                        self.assertTrue(all(speed == 20. for move, speed in rig.toolhead.moves))
                        self.assertTrue(all(move[2] is None and move[1 - index] is None
                                            for move, speed in rig.toolhead.moves))
                        self.assertEqual(rig.owner.rails_by_axis[axis].info.speed, 100.)
                        self.assertEqual(rig.owner.homing_accel, 5000.)
                        self.assertEqual(rig.home_calls, [])

    def test_mapping_precedes_noise_then_homing_and_original_homing_settings_remain(self):
        rig = MappingRig()
        session = rig.begin_mapping()
        observed = []
        def noise(current):
            self.assertEqual(current["axis_mapping_phase"], "qualified")
            self.assertEqual(rig.toolhead.accel, 50000.)
            observed.append("noise")
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(rig.workflow, "_calibration_noise_preflight", side_effect=noise), \
                mock.patch.object(rig.workflow, "_calibrate_trials", side_effect=lambda: observed.append("home")):
            rig.workflow._run_calibration()
        self.assertEqual(observed, ["noise", "home"])
        self.assertEqual(rig.events.count("capture"), 2)

    def test_inference_failure_exports_exact_evidence_after_cleanup_with_reserved_session(self):
        rig = MappingRig()
        session = rig.begin_mapping()
        limits, accel = list(rig.toolhead.kin.limits), rig.toolhead.accel
        current_context = {"configured": {"tmc5160 stepper_x": {"run_current": 1.2, "homing_current": .8}},
                           "native_helper": "simulated.Homing._set_homing_current",
                           "matching": "native_homing_current_helper", "physical_current_verified": False}
        expected_current_context = copy.deepcopy(current_context)
        current_calls = []
        current_helper = lambda axes, enabled: current_calls.append((axes, enabled))
        metrics = {"version": 1, "axis": "x", "passes": [
            {"direction": 1, "dispersion": {"median_mm_s": 5.25, "limit_mm_s": 4.0},
             "velocity_mm_s": [-20., 0., 0.]},
            {"direction": -1, "sample_count": 2000}], "rejected_pass": 0}
        captured = {}
        def reject(axis, passes, diagnostics=None):
            captured["passes"] = copy.deepcopy(passes)
            if diagnostics is not None:
                diagnostics.update(copy.deepcopy(metrics))
            raise ValueError("Mapping cruise response is unstable")
        paths = {"axis_mapping_json": "mapping.json", "axis_mapping_png": "mapping.png",
                 "axis_mapping_zip": "mapping.zip"}
        def export(directory, axis, passes, diagnostic, renderer=None):
            self.assertEqual(rig.end_sessions, 1)
            self.assertEqual(rig.toolhead.kin.limits, limits)
            self.assertEqual(rig.toolhead.accel, accel)
            self.assertIsNone(rig.owner.beta_motion_capture)
            self.assertIs(rig.workflow.session, session)
            self.assertTrue(session["finishing"])
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.noise_calls, [])
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(passes, captured["passes"])
            self.assertEqual(diagnostic["inference"], metrics)
            self.assertEqual(diagnostic["sensor_context"], rig.sensor_context())
            self.assertEqual(diagnostic["plan"]["distance_mm"], 15.)
            self.assertEqual(diagnostic["plan"]["driver_current_context"], expected_current_context)
            self.assertIs(diagnostic["plan"]["homing_current_applied"], True)
            self.assertEqual(diagnostic["plan"]["calibration_settle_time_s"], rig.workflow.calibration_settle_time)
            self.assertEqual(current_calls, [(["x"], True), (["x"], False)])
            self.assertIs(renderer, rig.workflow.renderer)
            with self.assertRaisesRegex(CommandError, "already active"):
                rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0))
            return paths
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(rig.workflow, "_beta_current_context", return_value=(current_helper, current_context)), \
                mock.patch.object(workflow_module, "infer_axis_mapping", side_effect=reject), \
                mock.patch.object(workflow_module, "write_axis_mapping_failure", side_effect=export, create=True) as writer, \
                self.assertRaisesRegex(CommandError, "Mapping cruise response is unstable"):
            rig.workflow._run_calibration()
        writer.assert_called_once()
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "aborted")
        self.assertEqual(result["attempts"], 0)
        self.assertEqual(result["axis_mapping_diagnostics"]["inference"], metrics)
        self.assertEqual(result["axis_mapping_diagnostics"]["paths"], paths)
        current_context["configured"]["tmc5160 stepper_x"]["homing_current"] = 5.
        self.assertEqual(result["axis_mapping_diagnostics"]["plan"]["driver_current_context"],
                         expected_current_context)
        self.assertTrue(any("mapping.zip" in line for line in rig.output))
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.workflow.pending_model_metadata, {})
        self.assertEqual(len(rig.toolhead.moves), 4)
        self.assertTrue(all(move[2] is None for move, speed in rig.toolhead.moves))

    def test_inference_failure_export_error_preserves_primary_abort_and_releases_session(self):
        rig = MappingRig()
        rig.begin_mapping()
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(workflow_module, "infer_axis_mapping", side_effect=ValueError("original inference rejection")), \
                mock.patch.object(workflow_module, "write_axis_mapping_failure", side_effect=OSError("archive disk full"), create=True), \
                self.assertLogs(level="ERROR"), self.assertRaisesRegex(CommandError, "original inference rejection"):
            rig.workflow._run_calibration()
        result = rig.workflow.last_result
        self.assertIn("original inference rejection", result["reason"])
        self.assertEqual(result["axis_mapping_diagnostics"]["export_error"], "archive disk full")
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.home_calls, [])

    def test_inference_failure_export_keeps_json_errors_and_abort_during_cancel(self):
        rig = MappingRig()
        rig.begin_mapping()
        paths = {"axis_mapping_json": "mapping.json", "chart_error": "render failed",
                 "archive_error": "archive failed"}
        def export(*args, **kwargs):
            rig.workflow.cmd_CANCEL(GCmd())
            self.assertIsNotNone(rig.workflow.session)
            with self.assertRaisesRegex(CommandError, "already active"):
                rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0))
            return paths
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(workflow_module, "infer_axis_mapping", side_effect=ValueError("original inference rejection")), \
                mock.patch.object(workflow_module, "write_axis_mapping_failure", side_effect=export), \
                self.assertRaisesRegex(CommandError, "original inference rejection"):
            rig.workflow._run_calibration()
        result = rig.workflow.last_result
        self.assertEqual(result["result"], "aborted")
        self.assertIn("original inference rejection", result["reason"])
        self.assertEqual(result["axis_mapping_diagnostics"]["paths"], paths)
        self.assertEqual(result["axis_mapping_diagnostics"]["chart_error"], "render failed")
        self.assertEqual(result["axis_mapping_diagnostics"]["archive_error"], "archive failed")
        self.assertTrue(any("mapping.json" in line for line in rig.output))
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.end_sessions, 1)
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.configfile.saved, {})

    def test_inference_failure_does_not_export_when_mapping_cleanup_fails(self):
        rig = MappingRig()
        rig.begin_mapping()
        original = rig.owner._set_accel
        def acceleration(value):
            if value == 50000.:
                raise CommandError("cannot restore acceleration")
            original(value)
        rig.owner._set_accel = acceleration
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(workflow_module, "infer_axis_mapping", side_effect=ValueError("original inference rejection")), \
                mock.patch.object(workflow_module, "write_axis_mapping_failure", create=True) as writer, \
                self.assertRaisesRegex(CommandError, "original inference rejection"):
            rig.workflow._run_calibration()
        writer.assert_not_called()
        diagnostic = rig.workflow.last_result["axis_mapping_diagnostics"]
        self.assertIn("cannot restore acceleration", " ".join(diagnostic["cleanup_errors"]))
        self.assertIn("cleanup", diagnostic["export_skipped"])
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.home_calls, [])

    def test_inference_failure_does_not_export_when_workflow_cleanup_fails(self):
        rig = MappingRig()
        rig.begin_mapping()
        rig.owner._end_session = mock.Mock(side_effect=CommandError("cleanup stopped"))
        with mock.patch.object(rig.workflow, "_chart_preflight"), \
                mock.patch.object(workflow_module, "infer_axis_mapping", side_effect=ValueError("original inference rejection")), \
                mock.patch.object(workflow_module, "write_axis_mapping_failure", create=True) as writer, \
                self.assertRaisesRegex(CommandError, "cleanup stopped"):
            rig.workflow._run_calibration()
        writer.assert_not_called()
        result = rig.workflow.last_result
        self.assertIn("cleanup stopped", result["reason"])
        self.assertIn("original inference rejection", result["axis_mapping_diagnostics"]["reason"])
        self.assertIn("cleanup", result["axis_mapping_diagnostics"]["export_skipped"])
        self.assertIsNone(rig.workflow.session)

    def test_failure_never_homes_saves_or_returns_after_uncertain_move(self):
        for stage in ("enable", "reference", "move", "profile", "capture"):
            with self.subTest(stage=stage):
                rig = MappingRig()
                rig.fail = stage
                limits = list(rig.toolhead.kin.limits)
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        mock.patch.object(workflow_module, "write_axis_mapping_failure") as writer, \
                        self.assertRaises(CommandError):
                    rig.workflow._run_calibration()
                writer.assert_not_called()
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})
                self.assertLessEqual(len(rig.toolhead.moves), 1)
                self.assertEqual(rig.toolhead.kin.limits, limits)
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertIsNone(rig.owner.beta_motion_capture)
                self.assertIsNone(rig.workflow.session)

    def test_cancel_motor_release_and_sensor_change_at_yields_do_not_commit_mapping(self):
        for stage in ("enable", "reference", "move", "profile", "capture"):
            for fault in ("cancel", "motor_off", "sensor"):
                with self.subTest(stage=stage, fault=fault):
                    rig = MappingRig()
                    fired = []
                    def interrupt(current):
                        if current != stage or fired:
                            return
                        fired.append(True)
                        if fault == "cancel": rig.workflow.cmd_CANCEL(GCmd())
                        elif fault == "motor_off": rig.owner.operation_generation += 1
                        else: rig.owner.sensor_name = "different"
                    rig.interrupt = interrupt
                    rig.begin_mapping()
                    with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
                        rig.workflow._run_calibration()
                    self.assertTrue(fired)
                    self.assertEqual(rig.home_calls, [])
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertLessEqual(len(rig.toolhead.moves), 1)
                    self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
                    self.assertEqual(rig.toolhead.position[2:], [40., 6.])
                    self.assertEqual(rig.toolhead.accel, 50000.)

    def test_ambiguous_or_clipped_evidence_aborts_before_noise_or_home(self):
        for failure in ("ambiguous", "clipped"):
            with self.subTest(failure=failure):
                rig = MappingRig()
                original = rig.owner.stop_beta_motion_capture
                def corrupted(token, check):
                    capture = original(token, check)
                    if failure == "clipped": capture["amplitude_limited"] = True
                    else: capture["samples"] = [(t, z - 1., y, z) for t, x, y, z in capture["samples"]]
                    return capture
                rig.owner.stop_beta_motion_capture = corrupted
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), \
                        mock.patch.object(workflow_module, "write_axis_mapping_failure", return_value={}) as writer, \
                        self.assertRaises((ValueError, CommandError)):
                    rig.workflow._run_calibration()
                self.assertEqual(writer.call_count, int(failure == "ambiguous"))
                self.assertEqual(rig.noise_calls, [])
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_noise_uses_mapped_component_even_if_another_component_is_larger(self):
        rig = Rig()
        original = rig.qualify_axis_mapping
        def mapped(session):
            original(session)
            session["axis_mapping"].update(source_axis=2, sign=-1)
        rig.workflow._calibration_axis_mapping = mapped
        rig.amplitudes = (5., 7., .6)
        with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
            rig.calibrate(save=0)
        self.assertEqual(rig.home_calls[0]["threshold"], 1.)
        result = rig.workflow.last_result["stationary_noise"]
        self.assertEqual(result["impact_metric"], "mapped_axis")
        self.assertAlmostEqual(result["noise_axis_peak_g"], .6)
        self.assertGreater(result["noise_magnitude_peak_g"], 8.)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertEqual(metadata["axis_mapping"]["source_axis"], 2)
        self.assertEqual(metadata["axis_mapping_context"], rig.sensor_context())

    def test_legacy_model_rejected_even_with_manual_threshold_and_mapped_model_keeps_orientation(self):
        rig = Rig({"x_threshold": 7.})
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 4.}, metadata={"x": {"impact_metric": "xyz_magnitude"}})
        with self.assertRaisesRegex(CommandError, "recalibrate"):
            rig.workflow.check_home_axes(["x"])
        session = {"axis": "x"}
        rig.qualify_axis_mapping(session)
        rig.workflow.model.metadata["x"] = {"impact_metric": "mapped_axis", **session}
        rig.workflow.check_home_axes(["x"])
        self.assertEqual(rig.workflow.detection_axis_mapping("x")["source_axis"], 0)
        self.assertTrue(rig.workflow.simple_detection("x"))
        rig.owner.sensor_name = "changed"
        with self.assertRaisesRegex(CommandError, "context changed"):
            rig.workflow.check_home_axes(["x"])

    def test_malformed_mapping_never_loads_and_cleanup_failure_does_not_stage(self):
        rig = Rig()
        session = {"axis": "x"}
        rig.qualify_axis_mapping(session)
        metadata = {"impact_metric": "mapped_axis", **session}
        for key, value in (("source_axis", True), ("sign", 0), ("coordinate_frame", "raw"), ("axis", "y")):
            bad = copy.deepcopy(metadata)
            bad["axis_mapping"][key] = value
            with self.subTest(key=key), self.assertRaises(ConfigError):
                AccelHomingModel(Config(None, {"x_threshold": 2., "x_successes": 10,
                                               "x_metadata": json.dumps(bad)}, "accel_homing_model test"))
        with mock.patch.object(rig.workflow, "_finish", side_effect=CommandError("cleanup failed")), \
                self.assertRaises(CommandError):
            rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, {})

    def test_frozen_mapping_receipts_must_match_on_every_trial(self):
        for target in ("record", "detector", "capture", "frame"):
            with self.subTest(target=target):
                rig = Rig()
                original = rig.owner.stop_capture
                def changed():
                    capture = original()
                    if target == "frame":
                        capture["coordinate_frame"] = "raw_sensor"
                    else:
                        item = capture["_record"] if target == "record" else capture["detector"] if target == "detector" else capture
                        item["axis_mapping"] = dict(item["axis_mapping"], source_axis=2)
                    return capture
                rig.owner.stop_capture = changed
                with self.assertRaisesRegex(CommandError, "axis mapping|calibration homing"):
                    rig.calibrate(save=0)
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.configfile.saved, {})

    def test_actual_terminal_cleanup_failure_reports_aborted_and_preserves_pending_model(self):
        rig = Rig()
        previous = {("accel_homing_model default", "x_threshold"): "3.5"}
        rig.configfile.saved.update(previous)
        original = rig.owner._end_session
        def cleanup():
            original()
            if rig.workflow.session.get("finishing"):
                raise CommandError("synthetic cleanup failure")
        with mock.patch.object(rig.owner, "_end_session", side_effect=cleanup), self.assertRaises(CommandError):
            rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, previous)
        self.assertEqual(rig.workflow.last_result["result"], "aborted")
        self.assertFalse(rig.workflow.last_result["save_config"])
        self.assertIsNone(rig.workflow.session)

    def saved_pair(self, rig):
        metadata = {}
        for axis in "xy":
            session = {"axis": axis}
            rig.qualify_axis_mapping(session)
            metadata[axis] = {**rig.workflow._model_context(axis), **session, "impact_metric": "mapped_axis"}
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 8., "y": 9.}, metadata=metadata)
        return copy.deepcopy(metadata)

    def test_ninety_degree_remount_can_recalibrate_both_axes_without_restart_or_stale_overwrite(self):
        rig = MappingRig(source=1, sign=1)
        original = self.saved_pair(rig)
        with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
            rig.calibrate("X", save=0)
            first_x = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            first_y = json.loads(rig.configfile.saved[("accel_homing_model default", "y_metadata")])
            self.assertEqual(first_x["axis_mapping"]["source_axis"], 1)
            self.assertTrue(first_y["axis_mapping_invalidated"])
            self.assertEqual(first_y["axis_mapping"], original["y"]["axis_mapping"])
            with self.assertRaisesRegex(CommandError, "invalidated"):
                rig.workflow.check_home_axes(["y"])
            with self.assertRaisesRegex(CommandError, "staged"):
                rig.workflow.check_home_axes(["x"])
            rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
            rig.idle_state = "Ready"
            rig.axis, rig.source, rig.sign = "y", 0, -1
            rig.calibrate("Y", save=0)
        final_x = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        final_y = json.loads(rig.configfile.saved[("accel_homing_model default", "y_metadata")])
        self.assertEqual(final_x, first_x)
        self.assertNotIn("axis_mapping_invalidated", final_y)
        self.assertEqual(final_y["axis_mapping"]["source_axis"], 0)
        self.assertEqual(final_y["axis_mapping"]["sign"], -1)
        self.assertEqual(rig.workflow.model.metadata, original)

    def test_failed_remount_calibration_preserves_other_axis_and_pending_pair(self):
        rig = MappingRig(source=1, sign=1)
        original = self.saved_pair(rig)
        rig.home_results.append(CommandError("synthetic home failure"))
        with self.assertRaises(CommandError):
            rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, {})
        self.assertEqual(rig.workflow.pending_model_metadata, {})
        self.assertEqual(rig.workflow.model.metadata, original)

    def test_native_pending_configuration_rolls_back_whole_pair_on_partial_set_failure(self):
        # The workflow fixture isolates sys.modules while importing; use its
        # stdlib parser instance so discovery order cannot create two classes.
        parser = workflow_module.configparser.RawConfigParser()
        parser.add_section("accel_homing_model default")
        parser.set("accel_homing_model default", "y_metadata", "previous")
        config = types.SimpleNamespace(autosave=types.SimpleNamespace(fileconfig=parser),
                                       status_save_pending={"other": {"value": "retained"}}, save_config_pending=True)
        original = copy.deepcopy(config.status_save_pending)
        def setting(section, option, value):
            config.autosave.fileconfig.set(section, option, value)
            config.status_save_pending = {"mutated": value}
            if option == "y_metadata":
                raise CommandError("synthetic partial staging failure")
        config.set = setting
        with self.assertRaises(CommandError):
            AccelHomingWorkflows._stage_model_settings(config, [
                ("accel_homing_model default", "x_metadata", "new"),
                ("accel_homing_model default", "y_metadata", "invalidated")])
        self.assertEqual(dict(config.autosave.fileconfig.items("accel_homing_model default")), {"y_metadata": "previous"})
        self.assertEqual(config.status_save_pending, original)
        self.assertTrue(config.save_config_pending)

    def test_orientation_change_invalidates_peer_without_source_collision_and_without_pingpong(self):
        for source, sign in ((2, 1), (0, -1)):
            with self.subTest(source=source, sign=sign):
                rig = MappingRig(source=source, sign=sign)
                self.saved_pair(rig)
                with mock.patch.object(workflow_module, "publish_noise", return_value={"png": "noise.png"}):
                    rig.calibrate("X", save=0)
                    peer = rig.workflow.pending_model_metadata["y"]
                    self.assertTrue(peer["axis_mapping_invalidated"])
                    rig.toolhead.kin.limits[:2] = [(1., -1.)] * 2
                    rig.idle_state = "Ready"
                    rig.axis, rig.source, rig.sign = "y", 1, 1
                    rig.calibrate("Y", save=0)
                self.assertNotIn("axis_mapping_invalidated", rig.workflow.pending_model_metadata["x"])
                self.assertNotIn("axis_mapping_invalidated", rig.workflow.pending_model_metadata["y"])

    def test_cancel_on_each_return_or_second_stroke_never_continues_motion(self):
        for move_number in (2, 3, 4):
            with self.subTest(move_number=move_number):
                rig = MappingRig()
                def cancel(stage):
                    if stage == "move" and rig.events.count("move") == move_number:
                        rig.workflow.cmd_CANCEL(GCmd())
                rig.interrupt = cancel
                rig.begin_mapping()
                with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaises(CommandError):
                    rig.workflow._run_calibration()
                self.assertEqual(len(rig.toolhead.moves), move_number)
                self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_acceleration_restore_failure_does_not_qualify_mapping_or_start_homing(self):
        rig = MappingRig()
        original = rig.owner._set_accel
        def setting(value):
            if value == 50000.:
                raise CommandError("synthetic acceleration restore failure")
            original(value)
        rig.owner._set_accel = setting
        session = rig.begin_mapping()
        with mock.patch.object(rig.workflow, "_chart_preflight"), self.assertRaisesRegex(CommandError, "cleanup failed"):
            rig.workflow._run_calibration()
        self.assertNotIn("axis_mapping", session)
        self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.)] * 2)
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.configfile.saved, {})


if __name__ == "__main__":
    unittest.main()
