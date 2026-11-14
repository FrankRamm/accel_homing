"""Frozen engineering noise margins; synthetic samples, no printer operations."""
import copy
import json
import math
import unittest
from unittest import mock

import accel_homing_experiment as experiment


def reference(shift=0., step=.0005):
    arm = 10. + shift
    rows = [(arm + n * step, .2, 0., 0.) for n in range(420)]
    for n in (388, 389):
        rows[n] = (rows[n][0], 5., 0., 0.)
    rows[400] = (rows[400][0], 9., 0., 0.)
    identity = [0, 1, 0, 2]
    capture = {"samples": rows, "baseline_g": [0., 0., 0.], "fault": None,
               "dropped": 0, "amplitude_limited": False,
               "detector": {"axis": "x", "impact_metric": "xyz_magnitude",
                            "attempt_count": 1, "onset_sample_matches": 1,
                            "onset_sample_index": 400, "onset_sample": identity,
                            "onset_time_s": rows[400][0], "arm_time_s": arm,
                            "threshold_g": 8.}}
    record = {"axis": "x", "result": "impact", "onset_sample": identity}
    return capture, record


def references():
    pairs = [reference(n * 2.) for n in range(10)]
    return [p[0] for p in pairs], [p[1] for p in pairs]


def profile(**changes):
    result = {"version": 1, "axis": "x", "weights_xyz": [1., 1., 1.],
              "threshold_g": 4., "raw_threshold_g": 8., "confirmation_s": .001,
              "max_age_s": .040, "shadow_only": True, "live_use_authorized": False,
              "noise_margin_policy": {"version": 1, "fraction": .10, "minimum_g": .5,
                  "interpretation": "engineering_noise_margin_not_reliability_probability",
                  "raw_bypass_assessed": True}}
    result.update(changes)
    return result


def set_x(capture, indices, value):
    for index in indices:
        row = capture["samples"][index]
        capture["samples"][index] = (row[0], value, 0., 0.)


def fit(captures, records, **kwargs):
    return experiment.fit_experimental_profile(captures, records, shadow=True,
        noise_margin_fraction=.10, noise_margin_g=.5, **kwargs)


class NoiseMarginTests(unittest.TestCase):
    def test_isolated_weighted_spike_is_not_a_confirmed_pair(self):
        capture, record = reference()
        set_x(capture, [10], 6.)
        set_x(capture, [20, 21], 3.4)
        frozen = profile()
        before = copy.deepcopy(frozen)
        outcome = experiment.replay_shadow_capture(frozen, capture, record)
        assessment = outcome["noise_margin"]
        self.assertTrue(assessment["qualified"])
        self.assertEqual(outcome["sample_index"], 389)
        startup = assessment["startup"]
        self.assertEqual(startup["raw_max_g"], 6.)
        self.assertEqual(startup["weighted_isolated_max_g"], 6.)
        self.assertEqual(startup["weighted_pair_max_g"], 3.4)
        self.assertAlmostEqual(startup["weighted_margin_g"], .6)
        self.assertAlmostEqual(startup["weighted_margin_fraction"], .15)
        self.assertEqual(startup["required_weighted_margin_g"], .5)
        self.assertEqual(startup["raw_bypass_margin_g"], 2.)
        pair = startup["closest_pair"]
        self.assertEqual((pair["evidence_sample_index"], pair["decision_sample_index"]), (20, 21))
        self.assertEqual(pair["evidence_time_s"], capture["samples"][20][0])
        self.assertEqual(pair["decision_time_s"], capture["samples"][21][0])
        self.assertEqual(assessment, experiment.assess_shadow_margin(frozen, capture, record))
        self.assertEqual(frozen, before)
        json.dumps(assessment, allow_nan=False)

    def test_tiny_positive_margin_rejects_even_without_false_trigger(self):
        capture, record = reference()
        set_x(capture, [300, 301], 3.99)
        outcome = experiment.replay_shadow_capture(profile(), capture, record)
        self.assertEqual(outcome["sample_index"], 389)
        self.assertTrue(outcome["within_negative_guard"])
        assessment = outcome["noise_margin"]
        self.assertFalse(assessment["qualified"])
        self.assertEqual(assessment["reason_codes"], ["cruise_insufficient_weighted_margin"])
        self.assertAlmostEqual(assessment["cruise"]["weighted_margin_g"], .01)

    def test_fraction_and_absolute_floor_both_apply(self):
        for threshold, pair, required, qualified in ((4., 3.5, .5, True),
                (4., 3.5001, .5, False), (6., 5.4, .6, True), (6., 5.45, .6, False)):
            capture, record = reference()
            set_x(capture, [20, 21], pair)
            result = experiment.assess_shadow_margin(profile(threshold_g=threshold), capture, record)
            with self.subTest(threshold=threshold, pair=pair):
                self.assertAlmostEqual(result["startup"]["required_weighted_margin_g"], required)
                self.assertEqual(result["qualified"], qualified)

    def test_raw_bypass_single_sample_margin_also_qualifies_conservatively(self):
        capture, record = reference()
        set_x(capture, [10], 7.5)
        assessment = experiment.assess_shadow_margin(
            profile(weights_xyz=[.5, 1.25, 1.25]), capture, record)
        self.assertTrue(assessment["startup"]["weighted_qualified"])
        self.assertFalse(assessment["qualified"])
        self.assertEqual(assessment["reason_codes"], ["startup_insufficient_raw_bypass_margin"])
        self.assertAlmostEqual(assessment["startup"]["required_raw_bypass_margin_g"], .8)
        self.assertAlmostEqual(assessment["startup"]["raw_bypass_margin_fraction"], .0625)

    def test_pairs_obey_actual_confirmation_chronology_and_phase_boundary(self):
        for gap, peak in ((.001, 3.9), (.00100001, .2)):
            capture, record = reference(step=.00075)
            rows = capture["samples"]
            adjustment = gap - .00075
            for index in range(21, len(rows)):
                rows[index] = (rows[index][0] + adjustment,) + rows[index][1:]
            capture["detector"]["onset_time_s"] = rows[400][0]
            set_x(capture, [20, 21], 3.9)
            assessment = experiment.assess_shadow_margin(profile(), capture, record)
            with self.subTest(gap=gap):
                self.assertAlmostEqual(assessment["startup"]["weighted_pair_max_g"], peak)
        capture, record = reference()
        set_x(capture, [199, 200], 3.9)
        assessment = experiment.assess_shadow_margin(profile(), capture, record)
        self.assertAlmostEqual(assessment["startup"]["weighted_pair_max_g"], .2)
        self.assertAlmostEqual(assessment["cruise"]["weighted_pair_max_g"], 3.9)
        self.assertEqual(assessment["cruise"]["closest_pair"]["evidence_sample_index"], 199)

    def test_guard_and_postreference_samples_do_not_enter_noise_windows(self):
        capture, record = reference()
        set_x(capture, [360, 361, 410, 411], 7.9)
        assessment = experiment.assess_shadow_margin(profile(), capture, record)
        self.assertTrue(assessment["qualified"])
        self.assertEqual(assessment["cruise"]["raw_max_g"], .2)

    def test_insufficient_pair_evidence_is_not_reported_as_zero_noise(self):
        capture, record = reference()
        assessment = experiment.assess_shadow_margin(profile(confirmation_s=.0001), capture, record)
        self.assertFalse(assessment["qualified"])
        for phase in ("startup", "cruise"):
            self.assertEqual(assessment[phase]["pair_count"], 0)
            self.assertIsNone(assessment[phase]["weighted_pair_max_g"])
            self.assertIsNone(assessment[phase]["weighted_margin_g"])
            self.assertIn(phase + "_missing_confirmable_negative_pair", assessment["reason_codes"])

    def test_heldout_small_margin_rejects_without_retuning_frozen_profile(self):
        captures, records = references()
        good = fit(captures, records)
        self.assertEqual(good["shadow"]["status"], "ready", good["shadow"]["reason"])
        self.assertFalse(good["accepted"])
        self.assertIsNone(good["profile"])
        frozen = good["shadow"]["profile"]
        amplitude = (frozen["threshold_g"] - .1) / math.sqrt(frozen["weights_xyz"][0])
        set_x(captures[6], [300, 301], amplitude)
        bad = fit(captures, records)
        self.assertEqual(bad["shadow"]["status"], "rejected")
        self.assertIn("holdout_insufficient_noise_margin", bad["shadow"]["reason_codes"])
        self.assertEqual(bad["shadow"]["profile"], frozen)
        self.assertEqual(bad["shadow"]["training"], good["shadow"]["training"])
        self.assertEqual(bad["diagnostics"], good["diagnostics"])
        heldout = bad["shadow"]["holdout"][0]
        self.assertTrue(heldout["within_negative_guard"])
        self.assertFalse(heldout["noise_margin"]["qualified"])
        fresh = experiment.replay_shadow_capture(frozen, captures[6], records[6])
        self.assertEqual(fresh, heldout)

    def test_invalid_holdout_preserves_frozen_profile_and_valid_sibling_margins(self):
        captures, records = references()
        good = fit(captures, records)
        captures[6]["dropped"] = 1
        bad = fit(captures, records)
        self.assertEqual(bad["shadow"]["status"], "rejected")
        self.assertEqual(bad["shadow"]["profile"], good["shadow"]["profile"])
        self.assertEqual(bad["shadow"]["training"], good["shadow"]["training"])
        self.assertEqual(bad["shadow"]["holdout"][1:], good["shadow"]["holdout"][1:])
        self.assertIsNone(bad["shadow"]["holdout"][0])
        self.assertEqual(bad["shadow"]["holdout_errors"][0]["reference_index"], 6)

    def test_fit_uses_confirmable_pair_instead_of_isolated_weighted_peak(self):
        captures, records = references()
        for capture in captures:
            set_x(capture, [20], 6.5)
        fitted = fit(captures, records)
        self.assertEqual(fitted["shadow"]["status"], "ready", fitted["shadow"]["reason"])
        self.assertTrue(all(o["noise_margin"]["qualified"] for o in fitted["shadow"]["training"]))
        # Legacy single-sample lower bound cannot detect a 5g pair after a 6.5g spike.
        legacy = experiment.fit_experimental_profile(captures, records, shadow=True)
        self.assertIsNone(legacy["shadow"]["profile"])

    def test_training_raw_margin_failure_does_not_access_holdouts(self):
        captures, records = references()
        for capture in captures[:6]:
            set_x(capture, [20], 7.9)
        class TrainingOnly(list):
            def __getitem__(self, index):
                if isinstance(index, int) and index >= 6:
                    raise AssertionError("heldout data was accessed during training failure")
                return super().__getitem__(index)
        fitted = fit(TrainingOnly(captures), TrainingOnly(records))
        self.assertEqual(fitted["shadow"]["status"], "rejected")
        self.assertIsNone(fitted["shadow"]["profile"])
        self.assertEqual(fitted["shadow"]["holdout"], [])
        self.assertIn("no_noise_margin_training_candidate", fitted["shadow"]["reason_codes"])
        self.assertGreater(fitted["diagnostics"]["noise_margin_rejection_counts"][
            "startup_insufficient_raw_bypass_margin"], 0)

    def test_malformed_policy_and_attempted_live_use_fail_closed(self):
        capture, record = reference()
        malformed = (None, [], {}, {"version": 1, "fraction": .1, "minimum_g": .5},
            dict(profile()["noise_margin_policy"], fraction=True),
            dict(profile()["noise_margin_policy"], fraction=float("nan")),
            dict(profile()["noise_margin_policy"], fraction=1.),
            dict(profile()["noise_margin_policy"], minimum_g=0.),
            dict(profile()["noise_margin_policy"], version=True),
            dict(profile()["noise_margin_policy"], raw_bypass_assessed=False))
        for policy in malformed:
            with self.subTest(policy=policy), self.assertRaises(ValueError):
                experiment.replay_shadow_capture(profile(noise_margin_policy=policy), capture, record)
        for changes in ({}, {"shadow_only": False, "live_use_authorized": True}):
            with self.assertRaisesRegex(ValueError, "not authorized"):
                experiment.validate_profile(profile(**changes), "x", 8.)
        captures, records = references()
        for fraction, minimum in ((None, .5), (.1, None), (0., .5), (True, .5), (.1, float("inf"))):
            result = experiment.fit_experimental_profile(captures, records, shadow=True,
                noise_margin_fraction=fraction, noise_margin_g=minimum)
            self.assertFalse(result["accepted"])
            self.assertEqual(result["shadow"]["reason_codes"], ["invalid_noise_margin_policy"])
        no_shadow = experiment.fit_experimental_profile(captures, records,
            noise_margin_fraction=.1, noise_margin_g=.5)
        self.assertEqual(no_shadow["reason_codes"], ["invalid_noise_margin_policy"])

    def test_legacy_defaults_unchanged_and_policy_work_is_bounded_cancellable(self):
        captures, records = references()
        legacy = experiment.fit_experimental_profile(captures, records, shadow=True)
        self.assertEqual(legacy, experiment.fit_experimental_profile(captures, records,
            shadow=True, noise_margin_fraction=None, noise_margin_g=None))
        frozen = legacy["shadow"]["profile"]
        self.assertNotIn("noise_margin", experiment.replay_shadow_capture(frozen, captures[0], records[0]))
        with self.assertRaisesRegex(ValueError, "no frozen noise margin policy"):
            experiment.assess_shadow_margin(frozen, captures[0], records[0])
        check = mock.Mock()
        fitted = fit(captures, records, checkpoint=check)
        self.assertEqual(fitted, fit(captures, records))
        self.assertLessEqual(fitted["candidate_count"], 27 * 17)
        self.assertLessEqual(len(fitted["diagnostics"]["candidate_examples"]), 12)
        self.assertGreater(check.call_count, 10)
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            fit(captures, records, checkpoint=mock.Mock(side_effect=RuntimeError("cancelled")))
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            experiment.assess_shadow_margin(profile(), captures[0], records[0],
                checkpoint=mock.Mock(side_effect=RuntimeError("cancelled")))


if __name__ == "__main__":
    unittest.main()
