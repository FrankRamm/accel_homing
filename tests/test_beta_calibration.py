"""Source-only beta workflow regressions; do not execute without operator approval."""
import contextlib
import json
import math
import types
import unittest
from unittest import mock

from harness import CommandError
from test_workflows import GCmd, Rig, workflow_module


class BetaRig(Rig):
    """Independent commanded trapezoids/captures; never a physical-motion model."""
    def __init__(self, options=None):
        super().__init__(options)
        self.clock, self.profile, self.beta_events = 10., [], []
        self.fail_at = None
        self.owner.capture_limit, self.owner.stream_timeout = 100000, .05
        self.toolhead.min_cruise_ratio = .5
        self.toolhead.get_last_move_time = lambda: self.clock
        self.objects["motion_report"] = types.SimpleNamespace(trapqs={
            "toolhead": types.SimpleNamespace(extract_trapq=self.extract_profile)})
        self.owner._enable_motors = self.enable_motors
        self.owner.start_beta_motion_capture = self.start_motion
        self.owner.stop_beta_motion_capture = self.stop_motion
        self.owner.finish_beta_home_capture = lambda check: check()
        self.owner.beta_home_telemetry = lambda capture, record: {
            "first_crossing": {"threshold_g": record["workflow_threshold_g"]},
            "pre_onset_noise": {"sample_count": 2}, "onset_window": {"sample_count": 2},
            "post_stop_response": {"available": False}}
        self.owner.beta_capture_diagnostics = lambda: {"fault": getattr(self.owner, "sensor_fault", None)}
        original_capture = self.owner.start_capture
        def capture(beta=False):
            self.beta_events.append(("home_capture", beta))
            return original_capture()
        self.owner.start_capture = capture
        original_move = self.toolhead.manual_move
        def move(coordinates, speed):
            session = self.workflow.session
            if session and session.get("beta_phase") == "motion_noise":
                axis = session["axis"]
                index = "xy".index(axis)
                distance = coordinates[index] - self.toolhead.position[index]
                direction = 1. if distance > 0 else -1.
                accel = self.toolhead.accel
                ramp = speed / accel
                cruise = abs(distance) / speed - ramp
                start = self.clock + .01
                self.profile = []
                for when, duration, velocity, acceleration in (
                        (start, ramp, 0., accel), (start + ramp, cruise, speed, 0.),
                        (start + ramp + cruise, ramp, speed, -accel)):
                    vector = [0., 0., 0.]
                    vector[index] = direction
                    self.profile.append(types.SimpleNamespace(
                        print_time=when, move_t=duration, start_v=velocity, accel=acceleration,
                        x_r=vector[0], y_r=vector[1], z_r=vector[2]))
                self.clock = start + 2 * ramp + cruise
                self.beta_events.append(("central_move", axis, distance, speed, accel))
            result = original_move(coordinates, speed)
            if self.fail_at == "move":
                raise CommandError("synthetic central move failure")
            return result
        self.toolhead.manual_move = move

    def extract_profile(self, start, end):
        if self.fail_at == "profile":
            raise CommandError("synthetic planner failure")
        return self.profile, None

    def enable_motors(self, axis):
        self.beta_events.append(("enable_and_settle", axis))
        for stepper in self.toolhead.kin.steppers:
            if stepper.is_active_axis(axis):
                self.lines[stepper.name].enabled = True
        self.toolhead.dwell(.1)

    def start_motion(self, check):
        check()
        self.beta_events.append(("baseline", self.workflow.session["axis"]))
        if self.fail_at == "baseline":
            raise CommandError("synthetic baseline failure")
        return {"baseline_samples": [(self.clock - 1. + number / 1000., 0., 0., 1.)
                                     for number in range(1001)],
                "sample_rate_hz": 1000., "capture_cutoff_time_s": self.clock}

    def stop_motion(self, token, check):
        check()
        if self.fail_at == "capture":
            raise CommandError("synthetic capture failure")
        start, end = token["capture_cutoff_time_s"], token["motion_end_time_s"]
        rows = [(start + number / 1000., (.2 if number % 2 else -.2), 0., 1.)
                for number in range(1, math.ceil((end - start) * 1000.) + 2)]
        return {"samples": rows, "baseline_g": (0., 0., 1.), "sample_rate_hz": 1000.,
                "capture_cutoff_time_s": start, "sensor_context": self.sensor_context(),
                "raw_samples_validated": True, "dropped": 0, "fault": None}

    @contextlib.contextmanager
    def beta_helpers(self):
        def stats(capture, record):
            return {"seed_magnitude_peak_g": 1., "all_phase_magnitude_peak_g": 8.,
                    "phases": [{"phase": segment["phase"], "magnitude_peak_g":
                                8. if segment["phase"] == "braking" else 1.}
                               for segment in record["segments"]]}
        def writer(directory, run_id, axis, iterations, **options):
            options["check_running"]()
            self.beta_events.append(("export", self.workflow.session is not None))
            return {"png": "Beta-calibration-%s.png" % run_id}
        with mock.patch.object(workflow_module, "beta_motion_statistics", side_effect=stats), \
                mock.patch.object(workflow_module, "compact_beta_home", side_effect=lambda capture, record: {
                    "record": dict(record), "capture": {"samples": []}}), \
                mock.patch.object(workflow_module, "write_beta_motion", side_effect=writer):
            yield

    def beta(self, axis="X", **params):
        with self.beta_helpers():
            return self.workflow.cmd_BETA_CALIBRATE(GCmd(AXIS=axis, CENTERED=1, **params))


class BetaCalibrationTests(unittest.TestCase):
    def test_beta_three_full_speed_recordings_then_eleven_homes_report_without_staging(self):
        for axis in "XY":
            with self.subTest(axis=axis):
                rig = BetaRig()
                rig.lines["stepper_x"].enabled = rig.lines["stepper_y"].enabled = False
                initial, acceleration = rig.toolhead.get_position(), rig.toolhead.accel
                rig.beta(axis)
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "beta_proposed")
                self.assertEqual(result["proposed_threshold_g"], 1.25)
                self.assertEqual(result["successes"], 10)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertEqual(rig.toolhead.accel, acceleration)
                self.assertEqual(rig.toolhead.position[2:], initial[2:])
                self.assertEqual(sum(event[0] == "baseline" for event in rig.beta_events), 3)
                central = [event for event in rig.beta_events if event[0] == "central_move"]
                self.assertEqual(len(central), 6)
                speed = rig.owner.rails_by_axis[axis.lower()].info.speed
                self.assertTrue(all(event[3] == speed and event[4] == 5000. for event in central))
                self.assertLess(next(i for i, event in enumerate(rig.beta_events) if event[0] == "enable_and_settle"),
                                next(i for i, event in enumerate(rig.beta_events) if event[0] == "baseline"))
                diagnostics = result["beta_diagnostics"]
                self.assertEqual(diagnostics["seed"]["source"], "independent_forward_motion_noise")
                self.assertEqual(len(diagnostics["motions"]), 3)
                self.assertEqual(len(diagnostics["trials"]), 11)
                self.assertTrue(diagnostics["restoration"]["thresholds_restored"])
                self.assertIn(("export", True), rig.beta_events)
                self.assertIsNone(rig.workflow.session)
                lines = [line.split("Beta diagnostic ", 1)[1] for line in rig.output if "Beta diagnostic " in line]
                parsed = [json.loads(line) for line in lines]
                self.assertTrue(any(item["section"] == "outcome" for item in parsed))
                self.assertTrue(any(item["section"].startswith("attempt.11") for item in parsed))
                self.assertFalse(any('"samples":' in line for line in lines))

    def test_explicit_start_is_preserved_and_seed_ignores_braking_peak(self):
        rig = BetaRig()
        rig.beta(START_THRESHOLD=.5)
        self.assertEqual(rig.workflow.last_result["proposed_threshold_g"], .5)
        self.assertEqual(rig.workflow.last_result["motion_noise"]["initial_threshold_source"], "explicit_start_threshold")
        self.assertEqual(rig.workflow.last_result["motion_noise"]["seed_peak_g"], 1.)
        self.assertEqual(rig.configfile.saved, {})

    def test_save_one_and_missing_planner_fail_before_any_movement(self):
        rig = BetaRig()
        with self.assertRaisesRegex(CommandError, "suggestion only"):
            rig.beta(SAVE_CONFIG=1)
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.beta_events, [])
        rig = BetaRig()
        del rig.objects["motion_report"]
        with self.assertRaisesRegex(CommandError, "trapq extractor"):
            rig.beta()
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.configfile.saved, {})
        self.assertIn("beta_diagnostics", rig.workflow.last_result)

    def test_central_stroke_stays_before_interior_endstop_in_both_directions(self):
        for positive, endstop, target in ((True, 200., 175.), (False, 100., 125.)):
            with self.subTest(positive=positive):
                rig = BetaRig()
                rail = rig.owner.rails_by_axis["x"]
                rail.bounds, rail.info.positive_dir, rail.info.position_endstop = (0., 300.), positive, endstop
                rig.workflow._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
                plan = rig.workflow._beta_motion_plan(rig.workflow.session)
                self.assertEqual(plan["center_mm"], 150.)
                self.assertEqual(plan["target_mm"], target)
                self.assertEqual(plan["distance_mm"], 25.)
                self.assertTrue(min(150., endstop) < target < max(150., endstop))
                self.assertEqual(rig.toolhead.moves, [])
        for positive, endstop in ((True, 100.), (False, 200.), (True, 150.)):
            rig = BetaRig()
            rail = rig.owner.rails_by_axis["x"]
            rail.info.positive_dir, rail.info.position_endstop = positive, endstop
            with self.assertRaises(CommandError):
                rig.beta()
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.beta_events, [])

    def test_central_failures_restore_acceleration_and_unhomed_limits_without_coordinate_rebase(self):
        for fault in ("baseline", "move", "profile", "capture"):
            with self.subTest(fault=fault):
                rig = BetaRig()
                rig.fail_at = fault
                before = rig.toolhead.get_position()
                rig.toolhead.position[0] = 17.
                with self.assertRaisesRegex(CommandError, "synthetic"):
                    rig.beta()
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.), (1., -1.)])
                self.assertEqual(rig.toolhead.position[1:], before[1:])
                self.assertNotEqual(rig.toolhead.position[0], 17.)
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertFalse(rig.workflow.last_result["beta_diagnostics"]["restoration"]["central_selected_frame_restored"])

    def test_running_spread_rejection_reverses_halt_before_new_discovery(self):
        rig = BetaRig()
        rig.home_results.extend([{}, {}, {"travel_mm": 152.001, "halt_travel_mm_signed": -155.}] + [{}] * 11)
        moves, original = [], rig.toolhead.manual_move
        def moving(coordinates, speed):
            if rig.workflow.session and rig.workflow.session.get("beta_phase") == "homing":
                moves.append((rig.toolhead.get_position(), list(coordinates), rig.workflow.session["candidate"]))
            return original(coordinates, speed)
        with mock.patch.object(rig.toolhead, "manual_move", side_effect=moving):
            rig.beta()
        rejected = rig.workflow.last_result["beta_diagnostics"]["trials"][2]
        self.assertEqual(rejected["beta_decision"], "rejected")
        self.assertEqual(rejected["beta_rejected_return_mm_signed"], 155.)
        self.assertAlmostEqual(moves[2][1][0] - moves[2][0][0], 155.)
        self.assertEqual(moves[2][2], 1.25)
        self.assertEqual([call["threshold"] for call in rig.home_calls], [1.25] * 3 + [1.5] * 11)
        self.assertEqual(rig.workflow.last_result["proposed_threshold_g"], 1.5)
        self.assertEqual(rig.configfile.saved, {})

    def test_planner_without_real_cruise_is_rejected_and_diagnostics_retain_failure(self):
        rig = BetaRig()
        rig.objects["motion_report"].trapqs["toolhead"].extract_trapq = lambda start, end: ([], None)
        with self.assertRaisesRegex(CommandError, "did not contain"):
            rig.beta()
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.workflow.last_result["result"], "aborted")
        self.assertEqual(rig.configfile.saved, {})
        self.assertTrue(any("Beta diagnostic" in line and "did not contain" in line for line in rig.output))

    def test_new_cancellation_during_export_keeps_ownership_and_cannot_produce_a_suggestion(self):
        rig = BetaRig()
        def export(*args, **kwargs):
            self.assertIsNotNone(rig.workflow.session)
            rig.workflow.cmd_CANCEL(GCmd())
            kwargs["check_running"]()
        with rig.beta_helpers(), mock.patch.object(workflow_module, "write_beta_motion", side_effect=export):
            rig.workflow.cmd_BETA_CALIBRATE(GCmd(AXIS="X", CENTERED=1))
        self.assertEqual(rig.workflow.last_result["result"], "aborted")
        self.assertIsNone(rig.workflow.last_result["proposed_threshold_g"])
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.configfile.saved, {})

    def test_cancellation_during_central_motion_or_accepted_recenter_never_starts_another_move(self):
        for stage in ("central", "recenter"):
            with self.subTest(stage=stage):
                rig = BetaRig()
                original = rig.toolhead.manual_move
                stopped = []
                def moving(coordinates, speed):
                    value = original(coordinates, speed)
                    session = rig.workflow.session
                    match = (session.get("beta_phase") == "motion_noise" if stage == "central" else
                             session.get("beta_phase") == "homing" and len(rig.home_calls) == 1)
                    if match and not stopped:
                        stopped.append(len(rig.toolhead.moves))
                        rig.workflow.cmd_CANCEL(GCmd())
                    return value
                with mock.patch.object(rig.toolhead, "manual_move", side_effect=moving), self.assertRaises(CommandError):
                    rig.beta()
                self.assertEqual(len(rig.toolhead.moves), stopped[0])
                self.assertEqual(len(rig.home_calls), 0 if stage == "central" else 1)
                self.assertEqual(rig.owner.threshold_g, {"x": 8., "y": 9.})
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertIsNone(rig.workflow.last_result.get("proposed_threshold_g"))
                self.assertIsNone(rig.workflow.session)

    def test_native_current_partial_activation_is_restored_even_when_cleanup_also_fails(self):
        for restore_fails in (False, True):
            with self.subTest(restore_fails=restore_fails):
                rig = BetaRig()
                calls = []
                class NativeHoming:
                    def __init__(self, printer):
                        pass
                    def _set_homing_current(self, axes, pre_homing):
                        calls.append((list(axes), pre_homing))
                        if pre_homing:
                            raise CommandError("primary current activation failure")
                        if restore_fails:
                            raise CommandError("secondary current restoration failure")
                rig.toolhead.get_active_rails_for_axis = lambda axis: set()
                with mock.patch.dict(workflow_module._PRINTER_G28.__globals__, {"Homing": NativeHoming}), \
                        self.assertRaisesRegex(CommandError, "primary current activation failure"):
                    rig.beta()
                self.assertEqual(calls, [([0], True), ([0], False)])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.toolhead.accel, 50000.)
                self.assertIn("primary current activation failure", rig.workflow.last_result["reason"])
                restoration = rig.workflow.last_result["beta_diagnostics"]["restoration"]
                self.assertEqual(restoration["central_driver_current_restored"], not restore_fails)
                if restore_fails:
                    self.assertIn("secondary current restoration failure", str(restoration["cleanup_errors"]))

    def test_changed_homing_current_without_native_helper_fails_before_motion(self):
        rig = BetaRig()
        rig.configfile.get_status = lambda eventtime: {"settings": {
            "tmc5160 stepper_x": {"run_current": 1.2, "homing_current": .8}}}
        with mock.patch.dict(workflow_module._PRINTER_G28.__globals__, {"Homing": None}), \
                self.assertRaisesRegex(CommandError, "homing_current.*unavailable"):
            rig.beta()
        self.assertEqual(rig.toolhead.moves, [])
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.configfile.saved, {})

    def test_failed_home_is_retained_as_incomplete_and_partial_beta_png_is_attempted(self):
        rig = BetaRig()
        rig.home_results.append(CommandError("synthetic home transport failure"))
        with self.assertRaisesRegex(CommandError, "home transport failure"):
            rig.beta()
        trials = rig.workflow.last_result["beta_diagnostics"]["trials"]
        self.assertEqual(len(trials), 1)
        self.assertEqual(trials[0]["beta_decision"], "incomplete")
        self.assertIn("home transport failure", trials[0]["beta_reason"])
        self.assertIn(("export", True), rig.beta_events)
        self.assertEqual(rig.configfile.saved, {})

    def test_ordinary_calibration_retains_legacy_capture_and_save_semantics(self):
        rig = BetaRig()
        rig.calibrate(save=0)
        self.assertTrue(rig.configfile.saved)
        self.assertTrue(all(event[1] is False for event in rig.beta_events if event[0] == "home_capture"))
        self.assertFalse(any("Beta diagnostic" in line for line in rig.output))
