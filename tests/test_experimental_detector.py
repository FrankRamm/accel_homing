"""Experimental-only detector contracts with synthetic clocks and no hardware."""
import copy
import random
import unittest
from unittest import mock

from harness import CommandError, arm_axis, batch, counts, make_homing, plugin
from test_trigger import feed_baseline, still, x_sample
import test_backends as backends
from accel_homing_experiment import (_replay_thresholds, fit_experimental_profile,
                                     replay_profile, validate_profile)

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST


def profile(axis="x", **changes):
    result = {"version": 1, "axis": axis, "weights_xyz": [2., .5, .5],
              "threshold_g": 2., "raw_threshold_g": 8.,
              "confirmation_s": .001, "max_age_s": .040}
    result.update(changes)
    return result


def reference(axis="x", shift=0., count=420):
    rows = []
    arm = 10. + shift
    for n in range(count):
        x = .1 if n % 2 else -.1
        z = 3. if n < 80 else .3
        if n == 397: x, z = 1., 0.
        if n == 398: x, z = 3., 0.
        if n == 399: x, z = 5., 0.
        if n >= 400: x, z = 10., 0.
        xyz = (x, 0., z) if axis == "x" else (0., x, z)
        rows.append((arm + n * .0005,) + xyz)
    identity = [0, 1, 0, 2]
    detector = {"axis": axis, "impact_metric": "xyz_magnitude", "attempt_count": 1,
                "onset_sample_matches": 1, "onset_sample_index": 400,
                "onset_sample": identity, "onset_time_s": rows[400][0],
                "arm_time_s": arm, "threshold_g": 8.}
    capture = {"samples": rows, "baseline_g": [0., 0., 0.], "detector": detector,
               "dropped": 0, "fault": None, "amplitude_limited": False}
    record = {"axis": axis, "result": "impact", "onset_sample": identity}
    return capture, record


def references(axis="x"):
    pairs = [reference(axis, n * 2.) for n in range(10)]
    return [p[0] for p in pairs], [p[1] for p in pairs]


def prove_clean_prefix(capture):
    detector = capture["detector"]
    capture["reference_signal_quality"] = {"amplitude_limited": False}
    detector["reference_prefix_quality"] = {
        "version": 1, "available": True,
        "source": "original_sensor_values_before_native_clip_decoding",
        "signal_scope": "armed_prefix_through_reference_decision",
        "sample_count": detector["onset_sample_index"] + 1,
        "first_sample_index": 0, "last_sample_index": detector["onset_sample_index"],
        "onset_sample": detector["onset_sample"], "amplitude_limited": False,
        "clipped_axis_values": 0, "range_limit_axis_values": 0}


class ExperimentalRuntimeTests(unittest.TestCase):
    def machine(self, **kwargs):
        obj = make_homing(**kwargs)
        feed_baseline(obj)
        obj.experimental_profile = profile()
        obj.reactor.now = .208
        return obj

    def test_weighted_confirmation_reports_second_row_and_original_capture_identity(self):
        obj = self.machine()
        obj.start_capture()
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        obj._inspect(batch(200, [x_sample(2.)] * 2 + still(6)))
        self.assertEqual(sent.messages, [[HIT]])
        self.assertEqual(attempt.onset_sample, (200, 7, 1, 8))
        self.assertEqual(attempt.experimental_evidence_sample, (200, 7, 0, 8))
        self.assertAlmostEqual(obj.onset_time(attempt), .201)
        obj.disarm()
        captured = obj.stop_capture()
        meta = captured["detector"]["experimental"]
        self.assertEqual(meta["route"], "weighted_confirmed")
        self.assertEqual(meta["capture_decision_sample_index"], 1)
        self.assertEqual(meta["capture_evidence_sample_index"], 0)
        self.assertEqual(meta["capture_evidence_sample_matches"], 1)
        self.assertEqual(captured["detector"]["threshold_g"], 8.)
        self.assertLess(captured["detector"]["onset_g"], 8.)
        self.assertFalse(attempt.record["experimental"]["decision_is_contact_measurement"])

    def test_strong_raw_is_immediate_on_any_axis_and_decodes_native_clips_honestly(self):
        for hit in ((0, counts(9.), counts(1.)), (0, 0, plugin.CLIPPED)):
            obj = self.machine()
            obj.start_capture()
            attempt, sent = arm_axis(obj, "x", 200, 1000)
            obj._inspect(batch(200, [hit] + still(7)))
            self.assertEqual(sent.messages, [[HIT]])
            self.assertEqual(attempt.onset_sample[2], 0)
            self.assertEqual(attempt.experimental_route, "raw_bypass")
            obj.disarm()
            captured = obj.stop_capture()
            self.assertEqual(captured["clipped_axis_values"], int(hit[2] == plugin.CLIPPED))
            if hit[2] == plugin.CLIPPED:
                self.assertTrue(captured["native_clip_decode"]["true_amplitude_unknown"])

    def test_single_sample_evidence_cannot_survive_quiet_or_new_home(self):
        obj = self.machine()
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        obj._inspect(batch(200, [x_sample(2.)] + still(7)))
        self.assertEqual(sent.messages, [])
        self.assertIsNone(attempt.experimental_pending_time)
        obj.disarm()
        obj.reactor.now = .216
        fresh, sent = arm_axis(obj, "x", 208, 1000)
        obj._inspect(batch(208, [x_sample(2.)] + still(7)))
        self.assertEqual(sent.messages, [])
        self.assertIsNone(fresh.experimental_route)

    def test_confirmation_crosses_batch_boundary_and_preserves_arm_trimming(self):
        obj = self.machine()
        obj.reactor.now = .201
        obj.start_capture()
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        obj._inspect(batch(193, still(7) + [x_sample(2.)]))
        self.assertEqual(sent.messages, [])
        obj.reactor.now = .209
        obj._inspect(batch(201, [x_sample(2.)] + still(7)))
        self.assertEqual(sent.messages, [[HIT]])
        self.assertEqual(attempt.experimental_evidence_sample, (193, 7, 7, 8))
        self.assertEqual(attempt.onset_sample, (201, 7, 0, 8))
        obj.disarm()
        meta = obj.stop_capture()["detector"]["experimental"]
        self.assertEqual(meta["capture_evidence_sample_index"], 7)
        self.assertEqual(meta["capture_decision_sample_index"], 8)

    def test_more_than_one_ms_cannot_confirm_and_gap_aborts(self):
        obj = self.machine()
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        obj.reactor.now = .214
        obj._inspect(batch(200, [x_sample(2.)] * 8, delta_clock=14))
        self.assertEqual(sent.messages, [])
        obj.reactor.now = .238
        obj._inspect(batch(230, still()))
        self.assertEqual(sent.messages, [[HOST]])
        self.assertEqual(attempt.reason, "sample_gap")

    def test_machine_weights_follow_mapping_signs_and_profile_snapshot(self):
        obj = self.machine(axes_map=((1, -1), (0, 1), (2, 1)))
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        obj.experimental_profile["weights_xyz"][:] = [.5, .5, 2.]
        self.assertEqual(attempt.experimental_weights, (.5, 2., .5))
        obj._inspect(batch(200, [(0, counts(-2.), counts(1.))] * 2 + still(6)))
        self.assertEqual(sent.messages, [[HIT]])
        self.assertEqual(attempt.experimental_route, "weighted_confirmed")

    def test_stale_data_fails_before_liveness_or_raw_success_using_dispatch_time(self):
        obj = self.machine()
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        previous = attempt.last_data
        obj.reactor.now = 1.
        obj._inspect(batch(200, [x_sample(10.)] * 8, receive_time=.208))
        self.assertEqual(sent.messages, [[HOST]])
        self.assertEqual(attempt.reason, "stale_data")
        self.assertEqual(attempt.last_data, previous)
        self.assertIsNone(attempt.onset_sample)

    def test_prearm_packets_do_not_refresh_experimental_liveness(self):
        obj = self.machine()
        attempt, sent = arm_axis(obj, "x", 300, 1000)
        before = attempt.last_data, obj.last_batch
        obj.reactor.now = .250
        obj._inspect(batch(200, still()))
        self.assertEqual((attempt.last_data, obj.last_batch), before)
        self.assertEqual(sent.messages, [])

    def test_configured_freshness_and_numeric_32767_are_not_native_markers(self):
        for age in (.001, .100):
            obj = backends.generic(options={"threshold": 8.})
            backends.baseline(obj)
            obj.experimental_profile = profile()
            obj.start_capture()
            attempt, sent = backends.arm(obj)
            backends.feed(obj, backends.samples(10.060, x=32.767), age=age)
            self.assertEqual(attempt.reason, "impact" if age < .040 else "stale_data")
            self.assertEqual(sent.messages, [[HIT if age < .040 else HOST]])
            obj.disarm()
            captured = obj.stop_capture()
            self.assertEqual(captured["clipped_axis_values"], 0)
            self.assertIsNone(captured["native_clip_decode"])

    def test_disabling_profile_restores_ordinary_selected_axis_behavior(self):
        obj = self.machine()
        obj.experimental_profile = None
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        obj._inspect(batch(200, [x_sample(2.)] * 8))
        self.assertFalse(attempt.vector_mode)
        self.assertEqual(sent.messages, [])
        obj.disarm()
        self.assertNotIn("experimental", attempt.record)

    def test_invalid_or_changed_reference_threshold_profile_is_rejected_before_arm(self):
        obj = self.machine()
        obj.experimental_profile["raw_threshold_g"] = 9.
        with self.assertRaises(CommandError):
            arm_axis(obj, "x", 200, 1000)
        self.assertIsNone(obj.attempt)

    def test_decision_timestamp_is_stable_after_clock_conversion_changes(self):
        obj = self.machine()
        attempt, _sent = arm_axis(obj, "x", 200, 1000)
        obj._inspect(batch(200, [x_sample(2.)] * 2 + still(6)))
        obj.clock_to_time = lambda clock: 100. + clock / 1000.
        self.assertAlmostEqual(obj.onset_time(attempt), .201)

    def test_baseline_observes_freshness_without_applying_experimental_age_gate(self):
        obj = self.machine()
        obj.experimental_profile = None
        obj.workflows._experimental_home = lambda axis: True
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        attempt.vector_mode = True
        obj.reactor.now = .245  # acceptable ordinary 50ms, beyond candidate 40ms
        obj._inspect(batch(200, [x_sample(9.)] + still(7), receive_time=.208))
        self.assertEqual(sent.messages, [[HIT]])
        obj.disarm()
        envelope = attempt.record["experimental_envelope"]
        self.assertEqual(envelope["eligible_batches"], 1)
        self.assertAlmostEqual(envelope["max_oldest_eligible_age_s"], .045)

    def test_capture_prefix_distinguishes_tail_and_onset_marker_or_numeric_rail(self):
        for marked in (plugin.CLIPPED, 2047):
            for at_onset in (False, True):
                obj = self.machine()
                obj.experimental_profile = None
                obj.workflows._experimental_home = lambda axis: True
                obj.start_capture()
                attempt, sent = arm_axis(obj, "x", 200, 1000)
                attempt.vector_mode = True
                attempt.reference_signal_quality = obj._signal_quality("stationary_reference").summary()
                rows = still()
                rows[3] = x_sample(9.)
                rows[3 if at_onset else 6] = (marked, 0, counts(1.))
                obj._inspect(batch(200, rows))
                self.assertEqual(sent.messages, [[HIT]])
                obj.disarm()
                capture = obj.stop_capture()
                prefix = capture["detector"]["reference_prefix_quality"]
                self.assertTrue(capture["amplitude_limited"])
                self.assertEqual(prefix["amplitude_limited"], at_onset)
                self.assertEqual(prefix["sample_count"], 4)
                self.assertEqual(prefix["clipped_axis_values"], int(at_onset and marked == plugin.CLIPPED))
                self.assertEqual(prefix["range_limit_axis_values"], int(at_onset and marked != plugin.CLIPPED))

    def test_nonfinite_and_energy_overflow_abort_configured_path(self):
        for value in (float("nan"), float("inf"), 1e200):
            obj = backends.generic(options={"threshold": 8.})
            backends.baseline(obj)
            obj.experimental_profile = profile()
            attempt, sent = backends.arm(obj)
            backends.feed(obj, backends.samples(10.060, x=value))
            self.assertEqual(sent.messages, [[HOST]])
            self.assertEqual(attempt.reason, "invalid_data")

    def test_negative_capture_clock_shift_preserves_actual_first_eligible_row(self):
        obj = self.machine()
        obj.experimental_profile = None
        obj.workflows._experimental_home = lambda axis: True
        obj.start_capture()
        attempt, _sent = arm_axis(obj, "x", 200, 1000)
        attempt.vector_mode = True
        obj._inspect(batch(200, still(3) + [x_sample(9.)] + still(4)))
        obj.disarm()
        obj.clock_to_time = lambda clock: clock / 1000. - .0002
        detector = obj.stop_capture()["detector"]
        self.assertEqual(detector["first_eligible_sample"], (200, 7, 0, 8))
        self.assertEqual(detector["first_eligible_sample_index"], 0)
        self.assertAlmostEqual(detector["capture_arm_time_s"], .1998)
        self.assertEqual(detector["reference_prefix_quality"]["sample_count"], 4)
        self.assertEqual(detector["reference_prefix_quality"]["first_sample_index"], 0)


class ExperimentalFitTests(unittest.TestCase):
    def test_optimized_threshold_scan_matches_scalar_replay_across_evidence_gaps(self):
        rng = random.Random(20261113)
        thresholds = [.5, 1., 2., 3., 5., 7.]
        for trial in range(64):
            when, rows = 10., []
            for _ in range(64):
                when += rng.choice((.0002, .0005, .001, .00100001, .0015))
                rows.append((when, rng.choice((0., .3, 1.1, 2.8, 4.4, 6.2, 7.7)), 0., 0.))
            rows[-1] = (when, 9., 0., 0.)
            event = {"rows": rows, "eligible": list(range(len(rows))),
                     "onset": when, "onset_index": len(rows) - 1}
            optimized = _replay_thresholds(event, [r[1] ** 2 for r in rows], thresholds, None)
            for threshold, outcome in zip(thresholds, optimized):
                candidate = profile(weights_xyz=[1., 1., 1.], threshold_g=threshold)
                with self.subTest(trial=trial, threshold=threshold):
                    self.assertEqual(outcome, replay_profile(candidate, event))

    def test_deterministic_fit_uses_startup_and_cruise_and_keeps_axis_specific_models(self):
        for axis in "xy":
            captures, records = references(axis)
            result = fit_experimental_profile(captures, records, axis=axis)
            self.assertTrue(result["accepted"], result["reason"])
            self.assertEqual(result, fit_experimental_profile(captures, records, axis=axis))
            self.assertGreater(result["negative_samples"]["startup"], 0)
            self.assertGreater(result["negative_samples"]["cruise"], 0)
            self.assertEqual(len(result["training"]), 6)
            self.assertEqual(len(result["holdout"]), 4)
            self.assertTrue(all(abs(o["reference_delta_ms"]) <= 1.0000001 for o in result["holdout"]))
            self.assertFalse(result["physical_contact_verified"])
            self.assertEqual(result["profile"]["raw_threshold_g"], 8.)

    def test_bad_holdout_rejects_frozen_candidate_without_refitting(self):
        captures, records = references()
        good = fit_experimental_profile(captures, records)
        self.assertTrue(good["accepted"], good["reason"])
        rows = captures[6]["samples"]
        for n in (300, 301): rows[n] = (rows[n][0], 7.9, 0., 0.)
        bad = fit_experimental_profile(captures, records)
        self.assertFalse(bad["accepted"])
        self.assertIsNone(bad["profile"])
        self.assertEqual(bad["frozen_candidate"], good["profile"])
        self.assertIn("heldout", bad["reason"])
        self.assertLess(bad["holdout"][0]["reference_delta_ms"], -1.)

    def test_indistinguishable_preimpact_and_motion_is_rejected(self):
        captures, records = references()
        for capture in captures:
            rows = capture["samples"]
            for target, source in zip(range(300, 303), range(397, 400)):
                rows[target] = (rows[target][0],) + rows[source][1:]
        result = fit_experimental_profile(captures, records)
        self.assertFalse(result["accepted"])
        self.assertIn("inseparable", result["reason"])

    def test_clipping_invalid_provenance_and_missing_startup_fail_closed(self):
        for failure in ("clipping", "marker", "identity", "onset", "missing_startup", "missing_cruise", "unordered"):
            captures, records = references()
            capture = captures[0]
            if failure == "clipping": capture["amplitude_limited"] = True
            if failure == "marker": capture["clipped_axis_values"] = 1
            if failure == "identity": records[0]["onset_sample"] = [0, 1, 1, 2]
            if failure == "onset": capture["detector"]["onset_time_s"] += .001
            if failure == "missing_startup": capture["detector"]["arm_time_s"] -= .01
            if failure == "missing_cruise": capture["detector"]["arm_time_s"] += .105
            if failure == "unordered": capture["samples"][2] = capture["samples"][1]
            with self.subTest(failure=failure):
                self.assertFalse(fit_experimental_profile(captures, records)["accepted"])

    def test_checkpoint_services_fit_and_cancellation_propagates(self):
        captures, records = references()
        check = mock.Mock()
        self.assertTrue(fit_experimental_profile(captures, records, checkpoint=check)["accepted"])
        self.assertGreater(check.call_count, 10)
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            fit_experimental_profile(captures, records, checkpoint=mock.Mock(side_effect=RuntimeError("cancelled")))

    def test_profile_bounds_and_exact_reference_threshold(self):
        for changes in ({"weights_xyz": [3., 0., 0.]}, {"confirmation_s": .00101},
                        {"threshold_g": 8.}, {"max_age_s": float("nan")},
                        {"raw_threshold_g": 7.}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_profile(profile(**changes), "x", 8.)

    def test_only_proven_postdecision_clipping_is_excluded_from_fit(self):
        captures, records = references()
        for capture in captures:
            prove_clean_prefix(capture)
            capture["amplitude_limited"] = True
            capture["clipped_axis_values"] = 2
            rows = capture["samples"]
            rows[405] = (rows[405][0] + .01, 16., 0., 0.)  # tail cadence irrelevant
        good = fit_experimental_profile(captures, records)
        self.assertTrue(good["accepted"], good["reason"])
        for source in ("prefix_marker", "prefix_rail", "reference", "unknown", "wrong_scope", "wrong_end"):
            damaged = copy.deepcopy(captures)
            q = damaged[0]["detector"]["reference_prefix_quality"]
            if source == "prefix_marker": q["clipped_axis_values"] = 1
            if source == "prefix_rail": q["range_limit_axis_values"] = 1
            if source == "reference": damaged[0]["reference_signal_quality"]["amplitude_limited"] = True
            if source == "unknown": damaged[0]["detector"].pop("reference_prefix_quality")
            if source == "wrong_scope": q["signal_scope"] = "whole_capture"
            if source == "wrong_end": q["last_sample_index"] -= 1
            with self.subTest(source=source):
                self.assertFalse(fit_experimental_profile(damaged, records)["accepted"])

    def test_shifted_first_sample_cannot_hide_early_weighted_pair_from_fit(self):
        captures, records = references()
        for capture in captures:
            rows, detector = capture["samples"], capture["detector"]
            for n, row in enumerate(rows): rows[n] = (row[0] - .0002,) + row[1:]
            rows[0] = (rows[0][0], 3., 0., 0.)
            rows[1] = (rows[1][0], 5., 0., 0.)
            detector["onset_time_s"] = rows[400][0]
            detector["capture_arm_time_s"] = rows[0][0]
            detector["first_eligible_sample"] = [0, 1, 0, 2]
            detector["first_eligible_sample_matches"] = 1
            detector["first_eligible_sample_index"] = 0
            detector["experimental_envelope"] = {"first_eligible_sample": [0, 1, 0, 2],
                                                   "first_eligible_after_arm_s": 0.}
            prove_clean_prefix(capture)
        result = fit_experimental_profile(captures, records)
        self.assertFalse(result["accepted"])
        self.assertIn("inseparable", result["reason"])


if __name__ == "__main__":
    unittest.main()
