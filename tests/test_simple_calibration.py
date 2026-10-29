"""Integrated impact-after-20 mm/vector detection regressions; no physical printer."""
import itertools
import json
import math
import types
import unittest
from unittest import mock

from harness import CommandError, arm_axis, batch, make_homing, plugin
import test_backends as backend_tests
from test_backends import feed
from test_motion_policy import SignedRamp
from test_workflows import AccelHomingWorkflows, GCmd, Rig, workflow_module

G = 9806.65


def prepared(axis="x"):
    obj, endstop = backend_tests.RaisedBaselineFloorTest().machine(
        threshold=1.0, axis=axis, spreads=(1.313, .604, .281))
    obj.workflows.session = {"mode": "calibration", "state": "running", "axis": axis,
                             "algorithm": "impact20_vector_v2"}
    obj.workflows.in_home, obj.workflows.home_axis = True, axis
    obj._prepare_reference()
    when = obj.backend.last_sample_time + 16 / 3200.0
    endstop.get_steppers = lambda: [SignedRamp(when, 5000.0, 50.0, 1)]
    attempt = obj.prepare(endstop, when)
    obj.arm(attempt)
    return obj, attempt


def packet(obj, attempt, xyz, position=0):
    rows = [tuple(attempt.bases)] * 8
    rows[position] = tuple(base + value / attempt.count_g for base, value in zip(attempt.bases, xyz))
    start = obj.backend.print_time_to_clock(attempt.arm_time + .001)
    params = {"start_clock": start, "delta_clock": 2188,
              "data": tuple(v for row in rows for v in row),
              "#receive_time": attempt.arm_time + .004}
    obj.reactor.now = attempt.arm_time + .004
    obj._inspect(params)
    return params


class VectorDetectorTests(unittest.TestCase):
    def test_real_prepare_keeps_offset_but_omits_noise_bounds_and_startup_deadline(self):
        for axis in "xy":
            obj, attempt = prepared(axis)
            self.assertTrue(attempt.vector_mode)
            self.assertIsNone(attempt.proof_noise)
            self.assertIsNone(attempt.proof_deadline)
            self.assertIsNone(attempt.motion_deadline)
            self.assertFalse(obj.baseline_status["quiet_range_enforced"])
            self.assertAlmostEqual(attempt.bases[2] * attempt.count_g, 1.0, delta=.002)
            self.assertEqual(obj._motion_proof_status(attempt)["result"], "not_required")

    def test_every_channel_sign_and_diagonal_can_trigger_either_movement_axis(self):
        vectors = [(1.25 * sign if n == axis else 0.0 for n in range(3))
                   for axis in range(3) for sign in (-1, 1)]
        vectors = [tuple(v) for v in vectors] + [(.8, .8, 0), (.6, -.6, .6)]
        for move_axis, xyz in itertools.product("xy", vectors):
            with self.subTest(move=move_axis, xyz=xyz):
                obj, attempt = prepared(move_axis)
                packet(obj, attempt, xyz, position=2)
                self.assertEqual(attempt.reason, "impact")
                self.assertEqual(attempt.onset_batch[2], 2)
                self.assertAlmostEqual(attempt.onset_raw * attempt.count_g,
                                       math.sqrt(sum(x*x for x in xyz)))
                self.assertFalse(attempt.motion_seen)
                with mock.patch.object(obj, "_motion_state", side_effect=AssertionError("velocity proof")):
                    self.assertEqual(obj.trigger_time(attempt), obj.onset_time(attempt))

    def test_resting_gravity_is_subtracted_and_subthreshold_signal_does_not_fire(self):
        obj, attempt = prepared()
        packet(obj, attempt, (.4, .4, .4))
        self.assertFalse(attempt.done)
        self.assertEqual(attempt.sums, [0.0, 0.0, 0.0])

    def test_first_crossing_not_peak_decides_stop_and_source_sample_identity(self):
        obj, attempt = prepared()
        # Exact boundary uses exactly representable raw count arithmetic.
        attempt.bases = (0.0, 0.0, 1000.0)
        attempt.baseline = 0.0
        rows = [(0., 0., 1000.), (600., 800., 1000.), (3000., 0., 1000.), (0., 0., 1000.)]
        start = obj.backend.print_time_to_clock(attempt.arm_time + .001)
        obj._inspect({"start_clock": start, "delta_clock": 938,
                      "data": tuple(v for row in rows for v in row), "#receive_time": attempt.arm_time + .003})
        self.assertEqual(attempt.reason, "impact")
        self.assertEqual(attempt.onset_batch[2], 1)
        self.assertEqual(attempt.onset_sample, (start, 938, 1, 4))
        self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 1.0)
        self.assertAlmostEqual(attempt.impact_peak * attempt.count_g, 3.0)
        self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)

    def test_live_stream_does_not_stop_at_twenty_mm_or_old_startup_deadline(self):
        obj, attempt = prepared()
        with mock.patch.object(obj, "_motion_state", side_effect=AssertionError("motion estimator called")):
            for number in range(-15, 1800, 16):
                rows = [(attempt.arm_time + n / 3200.,
                         (.30 if n % 2 else -.30) * G, .04 * G, G)
                        for n in range(number, number + 16)]
                feed(obj, rows)
                obj._watchdog(obj.reactor.now)
                self.assertFalse(attempt.done, attempt.reason)
        self.assertGreater(obj.reactor.now - attempt.arm_time, 20. / attempt.effective_speed)

    def test_stale_data_still_stops_the_simple_move(self):
        obj, attempt = prepared()
        rows = [(attempt.arm_time + n / 3200., .001 * (n % 2) * G, 0., G)
                for n in range(-15, 17)]
        feed(obj, rows, age=obj.backend.max_age + .001)
        self.assertTrue(attempt.done)
        self.assertEqual(attempt.reason, "stale_data")

    def test_missing_data_and_full_travel_watchdogs_remain_active(self):
        for reason in ("no_data", "travel"):
            obj, attempt = prepared()
            if reason == "no_data":
                obj.reactor.now = attempt.arm_time + .2
            else:
                obj.reactor.now = attempt.deadline + obj.stream_timeout + .01
                attempt.last_data = obj.reactor.now
                obj.backend.last_sample_time = obj.reactor.now
            obj._watchdog(obj.reactor.now)
            self.assertEqual(attempt.reason, reason)

    def test_native_raw_sensor_uses_xyz_magnitude_independent_of_axes_map(self):
        for mapping in (((0, 1), (1, 1), (2, 1)), ((2, -1), (0, -1), (1, 1))):
            obj = make_homing(axes_map=mapping, threshold=1.0)
            attempt, sent = arm_axis(obj, "x", 1000, 2000)
            attempt.vector_mode = True
            attempt.bases = (0., 0., 128.)
            obj.reactor.now = 1.008
            obj._inspect(batch(1001, [(0, 0, 128), (0, 164, 128)] + [(0, 0, 128)] * 6,
                               receive_time=1.008))
            self.assertEqual(attempt.reason, "impact")
            self.assertEqual(attempt.onset_batch[2], 1)
            self.assertEqual(len(sent.messages), 1)

    def test_native_gap_stops_without_fabricating_motion(self):
        obj = make_homing(threshold=1.0)
        attempt, _sent = arm_axis(obj, "x", 1000, 2000)
        attempt.vector_mode = True
        attempt.bases = (0., 0., 128.)
        obj.reactor.now = 1.108
        obj._inspect(batch(1101, [(0, 0, 128)] * 8, receive_time=1.108))
        self.assertEqual(attempt.reason, "sample_gap")

    def test_record_labels_metric_and_never_claims_measured_velocity_or_position(self):
        obj, attempt = prepared()
        packet(obj, attempt, (0., 1.3, 0.))
        obj.disarm()
        record = attempt.record
        self.assertEqual(record["impact_metric"], "xyz_magnitude")
        self.assertEqual(record["direction"], 0)
        self.assertFalse(record["physical_position_verified"])
        self.assertFalse(record["motion_seen"])
        self.assertEqual(record["motion_proof"]["result"], "not_required")


class ImpactDiscoveryTests(unittest.TestCase):
    def test_discovery_is_a_full_selected_axis_home_requiring_an_impact(self):
        for axis, positive in itertools.product("xy", (False, True)):
            with self.subTest(axis=axis, positive=positive):
                rig = Rig()
                rail = rig.owner.rails_by_axis[axis]
                rail.info.positive_dir = positive
                rail.info.position_endstop = rail.get_range()[1 if positive else 0]
                w = rig.workflow
                w._begin(GCmd(SAVE_CONFIG=0), "calibration", axis)
                w.session["state"] = "running"
                initial = rig.toolhead.get_position()
                z_limits = rig.toolhead.kin.limits[2]
                travel = w._geometry(axis)[1] * .95
                rig.home_results.append({"travel_mm": travel})
                record, capture = w._home(axis, 1.0)
                self.assertEqual(rig.scripts, ["G28 " + axis.upper()])
                self.assertEqual(record["result"], "impact")
                self.assertEqual(record["workflow_phase"], "discovery")
                self.assertEqual(record["travel_mm"], travel)
                self.assertEqual(w._quality(axis, record)[0], "good")
                self.assertFalse(w.session["candidate_qualified"])
                self.assertEqual(w.session["validation_records"], [])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertFalse(rig.capture_active)
                w._recenter(axis)
                expected = [None, None, None]
                expected["xy".index(axis)] = w._geometry(axis)[0]
                self.assertEqual(rig.toolhead.moves, [(expected, 50.)])
                for i in range(4):
                    if i != "xy".index(axis):
                        self.assertEqual(rig.toolhead.position[i], initial[i])
                self.assertEqual(rig.toolhead.kin.limits[2], z_limits)

    def test_reaching_twenty_mm_or_a_completed_endpoint_never_qualifies(self):
        for result in ("no_trigger", "screen_complete", "travel"):
            for travel in (20., 20.001, 106.188):
                with self.subTest(result=result, travel=travel):
                    rig = Rig()
                    rig.home_results.append({"result": result, "travel_mm": travel,
                                             "screen_completed": True, "screen_target_mm": 20.})
                    with self.assertRaisesRegex(CommandError, "failure"):
                        rig.calibrate(save=0)
                    self.assertEqual(len(rig.home_calls), 1)
                    self.assertEqual(rig.toolhead.moves, [])
                    self.assertEqual(rig.configfile.saved, {})
                    self.assertIsNone(rig.workflow.session)

    def test_twenty_mm_rule_and_geometry_both_use_unrounded_commanded_travel(self):
        for travel, quality in ((0., "early"), (19.9999, "early"), (20., "early"),
                                (20.0001, "early"), (106.188, "early"),
                                (120., "good"), (180., "good"), (180.0001, "implausible")):
            for phase in ("discovery", "validation"):
                rig = Rig()
                w = rig.workflow
                w._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
                w.session["state"] = "running"
                record = {"result": "impact", "impact_metric": "xyz_magnitude",
                          "travel_mm": round(travel, 3), "travel_mm_exact": travel,
                          "workflow_phase": phase}
                self.assertEqual(w._quality("x", record)[0], quality)

    def test_first_impact_is_excluded_then_only_selected_axis_centers(self):
        for axis, positive in itertools.product("xy", (False, True)):
            with self.subTest(axis=axis, positive=positive):
                rig = Rig()
                rail = rig.owner.rails_by_axis[axis]
                rail.info.positive_dir = positive
                rail.info.position_endstop = rail.get_range()[1 if positive else 0]
                rig.toolhead.position[:2] = [12., 19.]
                center = rig.workflow._geometry(axis)[0]
                rig.home_results.extend([{"travel_mm": center * .95}] + [{"travel_mm": center}] * 10)
                rig.calibrate(axis.upper(), save=0)
                self.assertEqual(len(rig.home_calls), 11)
                self.assertEqual(rig.home_calls[0]["position"][:2], [12., 19.])
                for call in rig.home_calls[1:]:
                    self.assertEqual(call["position"]["xy".index(axis)], center)
                    self.assertEqual(call["position"][1 - "xy".index(axis)], [19., 12.]["xy".index(axis)])
                for coords, _speed in rig.toolhead.moves:
                    self.assertTrue(all(value is None for i, value in enumerate(coords)
                                        if i != "xy".index(axis)))
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", axis + "_metadata")])
                self.assertEqual(metadata["min_commanded_travel_mm"], center)
                self.assertEqual(metadata["max_commanded_travel_mm"], center)
                self.assertEqual(metadata["repeatability_spread_mm"], 0.)
                self.assertEqual(metadata["recenter_policy"], "selected_axis_midpoint")

    def test_recenter_requires_a_current_qualified_impact(self):
        for result, travel in (("screen_complete", 20.), ("no_trigger", 100.), ("impact", 20.)):
            rig = Rig()
            w = rig.workflow
            w._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
            w.session["state"] = "running"
            record = {"result": result, "travel_mm": travel, "impact_metric": "xyz_magnitude"}
            rig.owner.last["x"] = record
            w.session["terminal_home"] = ("x", dict(record), record)
            rig.toolhead.kin.limits[0] = (0., 300.)
            with self.assertRaisesRegex(CommandError, "current impact"):
                w._recenter("x")
            self.assertEqual(rig.toolhead.moves, [])

    def test_changed_terminal_record_cannot_authorize_recenter(self):
        rig = Rig()
        w = rig.workflow
        w._begin(GCmd(SAVE_CONFIG=0), "calibration", "x")
        w.session["state"] = "running"
        record, _capture = w._home("x", 1.)
        rig.owner.last["x"] = dict(rig.owner.last["x"])
        with self.assertRaisesRegex(CommandError, "current measured move"):
            w._recenter("x")
        self.assertEqual(rig.toolhead.moves, [])

    def test_old_no_contact_discovery_cannot_be_saved(self):
        rig = Rig()
        save = rig.workflow._save_model
        def tamper_then_save():
            session = rig.workflow.session
            discovery = session["records"][session["candidate_attempt"] - 1]
            discovery.update(result="screen_complete", screen_completed=True)
            save()
        with mock.patch.object(rig.workflow, "_save_model", side_effect=tamper_then_save):
            with self.assertRaisesRegex(CommandError, "cannot save"):
                rig.calibrate(save=0)
        self.assertEqual(rig.configfile.saved, {})

    def test_exactly_twenty_mm_hit_returns_and_retries_in_the_same_command(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 20.})
        rig.calibrate(save=0, START_THRESHOLD=1, STEP=.25)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.toolhead.moves[0], ([20., None, None], 50.))
        self.assertEqual(rig.home_calls[1]["homed_axes"], "z")
        self.assertFalse(rig.home_calls[1]["candidate_qualified"])
        self.assertEqual(rig.home_calls[1]["successes"], 0)
        self.assertEqual([c["threshold"] for c in rig.home_calls], [1.] + [1.25] * 11)

    def test_failed_spread_requires_another_discovery_impact_before_ten_new_checks(self):
        rig = Rig()
        rig.home_results.extend([{"travel_mm": 145.}] + [{"travel_mm": 150.}] * 9 +
                                [{"travel_mm": 152.001}, {"result": "no_trigger", "travel_mm": 150.}])
        with self.assertRaisesRegex(CommandError, "failure"):
            rig.calibrate(save=0)
        self.assertEqual([c["threshold"] for c in rig.home_calls], [1.] * 11 + [1.25])
        self.assertFalse(rig.home_calls[-1]["candidate_qualified"])
        self.assertEqual(rig.home_calls[-1]["successes"], 0)
        self.assertEqual(rig.configfile.saved, {})

    def test_exactly_two_mm_passes_after_ten_checks_without_retry(self):
        rig = Rig()
        rig.home_results.extend([{"travel_mm": 145.}] + [{"travel_mm": 150.}] * 9 + [{"travel_mm": 152.}])
        rig.calibrate(save=0)
        self.assertEqual(len(rig.home_calls), 11)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.)
        self.assertEqual(rig.workflow.last_result["repeatability_spread_mm"], 2.)


class SimpleWorkflowTests(unittest.TestCase):
    def test_discovery_hit_is_excluded_and_ten_checks_need_no_motion_flag(self):
        rig = Rig()
        # Match this travel to the configured geometry instead of accepting a
        # repeatable but materially short hit on the default 300 mm axis.
        rig.owner.rails_by_axis["x"].bounds = (0., 212.376)
        rig.home_results.extend([{}] + [{"moving": False, "motion_seen": False, "travel_mm": 106.188}] * 10)
        rig.calibrate(save=0)
        self.assertEqual(len(rig.home_calls), 11)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertEqual(metadata["impact_metric"], "xyz_magnitude")
        self.assertEqual(metadata["minimum_impact_travel_mm"], 20.)
        self.assertEqual(metadata["repeatability_limit_mm"], 2.)
        self.assertEqual(metadata["min_commanded_travel_mm"], 106.188)
        self.assertEqual(len(metadata["onset_g"]), 10)

    def test_multiple_bad_sets_restart_discovery_with_ten_fresh_checks(self):
        rig = Rig()
        for _ in range(2):
            rig.home_results.extend([{}] + [{"travel_mm": 150.}] * 9 + [{"travel_mm": 152.001}])
        rig.home_results.extend([{}] + [{"travel_mm": 153.}] * 10)
        rig.calibrate(save=0)
        self.assertEqual([c["threshold"] for c in rig.home_calls],
                         [1.] * 11 + [1.25] * 11 + [1.5] * 11)
        for offset in (0, 11, 22):
            self.assertFalse(rig.home_calls[offset]["candidate_qualified"])
            self.assertEqual(rig.home_calls[offset]["successes"], 0)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.5)
        self.assertEqual(rig.workflow.last_result["restarts"], 2)

    def test_failed_set_cannot_exceed_sensor_headroom_or_global_attempt_budget(self):
        for ceiling in ("sensor-headroom", "attempts"):
            rig = Rig({"calibration_max_attempts": 12} if ceiling == "attempts" else {})
            if ceiling == "sensor-headroom":
                rig.owner.full_scale_g = 1.25
            rig.home_results.extend([{}] + [{"travel_mm": 150.}] * 9 + [{"travel_mm": 153.}])
            with self.assertRaisesRegex(CommandError, "headroom|attempt budget"):
                rig.calibrate(save=0)
            self.assertEqual(rig.configfile.saved, {})
            self.assertLessEqual(len(rig.home_calls), rig.workflow.max_attempts)

    def test_same_command_automatically_raises_threshold_after_early_impact(self):
        rig = Rig()
        rig.home_results.append({"travel_mm": 19.999})
        rig.calibrate(save=0, START_THRESHOLD=1, STEP=.25)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
        self.assertEqual([call["threshold"] for call in rig.home_calls], [1.] + [1.25] * 11)

    def test_saved_vector_model_uses_same_detector_but_legacy_models_stay_legacy(self):
        rig = Rig()
        w = rig.workflow
        self.assertFalse(w.simple_detection("x"))
        w.model = types.SimpleNamespace(thresholds={"x": 1.25}, metadata={"x": {"impact_metric": "xyz_magnitude"}})
        self.assertTrue(w.simple_detection("x"))
        self.assertFalse(w.simple_detection("y"))
        w.manual_axes.add("x")
        self.assertFalse(w.simple_detection("x"))
        w.manual_axes.clear()
        w.model.metadata["x"] = {}
        self.assertFalse(w.simple_detection("x"))

    def test_vector_calibrated_homes_are_accepted_by_test_and_chart_quality(self):
        rig = Rig()
        rig.workflow.session = {"mode": "test"}
        record = {"result": "impact", "travel_mm": 150., "moving": False,
                  "motion_seen": False, "impact_metric": "xyz_magnitude"}
        self.assertEqual(rig.workflow._quality("x", record)[0], "good")

    def test_too_short_axis_is_rejected_before_session_or_motion(self):
        rig = Rig()
        rig.owner.rails_by_axis["x"].bounds = (0., 40.)
        with self.assertRaisesRegex(CommandError, "exceed 20"):
            rig.calibrate(save=0)
        self.assertIsNone(rig.workflow.session)
        self.assertEqual(rig.home_calls, [])

    def test_spread_uses_unrounded_step_distance(self):
        rig = Rig()
        rig.home_results.extend([{}] + [{"travel_mm": 150., "travel_mm_exact": 150.}] * 9 +
                                [{"travel_mm": 152., "travel_mm_exact": 152.0001}])
        rig.calibrate(save=0)
        self.assertEqual(rig.workflow.last_result["threshold_g"], 1.25)
        self.assertEqual(rig.workflow.last_result["restarts"], 1)


class ReloadAndDiagnosticTests(unittest.TestCase):
    def test_saved_vector_model_prepares_ordinary_home_without_old_quiet_or_motion_guard(self):
        obj, endstop = backend_tests.RaisedBaselineFloorTest().machine(
            threshold=1.25, axis="x", spreads=(1.313, .604, .281))
        w = obj.workflows
        w.session = None
        w.in_home, w.home_axis = False, None
        w.model = types.SimpleNamespace(thresholds={"x": 1.25},
                                       metadata={"x": {"impact_metric": "xyz_magnitude"}})
        w.manual_axes.clear()
        obj._prepare_reference()
        self.assertFalse(obj.preparation.require_quiet)
        when = obj.backend.last_sample_time + 16 / 3200.
        endstop.get_steppers = lambda: [SignedRamp(when, 5000., 50., 1)]
        attempt = obj.prepare(endstop, when)
        self.assertTrue(attempt.vector_mode)
        self.assertIsNone(attempt.proof_noise)
        self.assertIsNone(attempt.proof_deadline)
        obj.arm(attempt)
        packet(obj, attempt, (0., 0., 1.5))
        self.assertEqual(attempt.reason, "impact")

    def test_discovery_keeps_full_travel_watchdog_without_motion_proof(self):
        obj, endstop = backend_tests.RaisedBaselineFloorTest().machine(threshold=1.)
        obj.workflows.session = {"mode": "calibration", "state": "running", "axis": "x",
                                 "algorithm": "impact20_vector_v2"}
        obj.workflows.in_home, obj.workflows.home_axis = True, "x"
        obj._prepare_reference()
        when = obj.backend.last_sample_time + 16 / 3200.
        endstop.get_steppers = lambda: [SignedRamp(when, 5000., 50., 1)]
        attempt = obj.prepare(endstop, when)
        low, high = obj.session_axes["x"].get_range()
        expected = ((high - low) * obj.travel_margin / attempt.effective_speed
                    + attempt.effective_speed / attempt.effective_accel + obj.watchdog_margin)
        self.assertGreater(attempt.deadline - when, 20. / attempt.effective_speed)
        self.assertAlmostEqual(attempt.deadline - when, expected)
        self.assertIsNone(attempt.proof_deadline)

    def test_exact_two_mm_step_difference_tolerates_only_binary_representation_noise(self):
        spread = AccelHomingWorkflows._repeatability_spread([
            {"travel_mm": 150., "travel_mm_exact": 150.00625},
            {"travel_mm": 152., "travel_mm_exact": 152.00625000000002}])
        self.assertEqual(spread, 2.)
        self.assertGreater(AccelHomingWorkflows._repeatability_spread([
            {"travel_mm": 150., "travel_mm_exact": 150.00625},
            {"travel_mm": 152., "travel_mm_exact": 152.006250001}]), 2.)

    def test_vector_chart_threshold_is_positive_magnitude_not_signed_channel(self):
        import accel_homing_charts as charts
        axis = mock.Mock()
        charts._threshold(axis, {"impact_metric": "xyz_magnitude", "threshold_g": 1.25, "direction": -1})
        self.assertEqual([call.args[0] for call in axis.axhline.call_args_list], [1.25])
        self.assertIn("XYZ magnitude", axis.axhline.call_args.kwargs["label"])

    def test_vector_capture_and_accuracy_plots_show_the_actual_detection_metric(self):
        from matplotlib.figure import Figure
        import accel_homing_charts as charts
        trace = [(0., 0., 0., 0.), (.001, .8, .8, 0.), (.002, .1, 0., 0.)]
        record = {"impact_metric": "xyz_magnitude", "threshold_g": 1., "direction": 0,
                  "travel_mm": 106., "onset_g": math.sqrt(1.28), "impact_peak_g": math.sqrt(1.28),
                  "onset_time_s": .001}
        payload = {"axis": "x", "timestamp": "SIMULATED", "simulated": True,
                   "record": record, "trace": trace,
                   "capture": {"first_sample_time_s": 0., "sample_count": 3}}
        figure = Figure(figsize=(10, 7))
        charts._plot_capture(figure, payload)
        line = next(line for line in figure.axes[0].lines if line.get_label() == "XYZ magnitude")
        self.assertAlmostEqual(line.get_ydata()[1], math.sqrt(1.28))
        figure.clear()
        payload.update(trials=[record], traces=[trace], statistics={}, requested=1)
        charts._plot_accuracy(figure, payload)
        self.assertEqual(len(figure.axes), 5)
        self.assertEqual(figure.axes[4].get_ylabel(), "XYZ magnitude (g)")
        self.assertFalse(any("trigger" in line.get_label().lower() for axis in figure.axes[1:4]
                             for line in axis.lines))
        figure.clear()
