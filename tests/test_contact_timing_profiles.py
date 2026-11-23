"""Contact diagnostics replay the supplied detector, including disabled."""
import copy
import json
import math
import unittest

import accel_homing_auto_notch as auto
from accel_homing_filter import CausalNotch, DISABLED, profile
from test_contact_timing_replay import BASELINE, RATE, retained_capture


class ContactTimingProfileTests(unittest.TestCase):
    def test_explicit_live_profile_replays_exact_signed_samples_from_arm(self):
        capture, planner = retained_capture()
        settings = profile(73., 20.)
        capture["detector"]["signal_filter"] = CausalNotch(settings, RATE).metadata()
        before = copy.deepcopy(capture)
        result, rows = auto.describe_contact_timing(capture, planner, settings=settings)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["replay_profile_source"], "explicit_profile")
        self.assertEqual(result["filter_label"], "recorded live detector replay")
        self.assertTrue(result["matches_recorded_live_filter"])
        self.assertEqual(result["filter"], CausalNotch(settings, RATE).metadata())
        expected = CausalNotch(settings, RATE)
        for row, source in zip(rows, capture["samples"][50:]):
            xyz = tuple(source[index + 1] - BASELINE[index] for index in range(3))
            self.assertEqual(row, (source[0], abs(xyz[2]), abs(expected.process(xyz)[2])))
        self.assertEqual(before, capture)
        historical, historical_rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(historical["filter"]["frequency_hz"], 200.)
        self.assertEqual(historical["filter"]["q"], 5.)
        self.assertEqual(historical["replay_profile_source"], "historical_fixed_diagnostic")
        self.assertFalse(historical["matches_recorded_live_filter"])
        self.assertNotEqual(rows, historical_rows)

    def test_disabled_retains_baseline_arm_and_phase_domain_at_low_sample_rate(self):
        capture, planner = retained_capture()
        stretch = lambda value: 10. + (value - 10.) * 2.5
        capture["samples"] = [(stretch(row[0]), *row[1:]) for row in capture["samples"]]
        meta = capture["auto_notch_replay"]
        for key in ("first_sample_time_s", "reference_first_time_s", "reference_last_time_s"):
            meta[key] = stretch(meta[key])
        meta["sample_rate_hz"] = 400.
        capture["detector"]["arm_time_s"] = stretch(capture["detector"]["arm_time_s"])
        capture["detector"]["signal_filter"] = dict(DISABLED)
        result, rows = auto.describe_contact_timing(capture, planner, settings=DISABLED)
        self.assertEqual(result["status"], "ok")
        self.assertTrue(result["matches_recorded_live_filter"])
        self.assertEqual(result["filter"]["type"], "disabled")
        self.assertEqual(result["filter"]["sample_rate_hz"], 400.)
        self.assertEqual(result["pre_arm_samples_excluded"], 50)
        self.assertEqual(len(rows), 551)
        for row, source in zip(rows, capture["samples"][50:]):
            self.assertEqual(row, (source[0], abs(source[3] - BASELINE[2]), abs(source[3] - BASELINE[2])))
        self.assertEqual(result["global_peak"]["raw_peak_g"], result["global_peak"]["filtered_peak_g"])
        historical, _ = auto.describe_contact_timing(capture, planner)
        self.assertEqual(historical["status"], "insufficient_data")
        self.assertIn("Nyquist", historical["reason"])

    def test_different_or_unproven_recorded_metadata_never_gets_live_label(self):
        for failure in ("missing", "frequency", "rate", "coefficients", "state", "unknown"):
            with self.subTest(failure=failure):
                capture, planner = retained_capture()
                settings = profile(100., 2.)
                recorded = CausalNotch(settings, RATE).metadata()
                if failure == "frequency": recorded["frequency_hz"] = 90.
                elif failure == "rate": recorded["sample_rate_hz"] *= 1.01
                elif failure == "coefficients": recorded["coefficients"][0] += .01
                elif failure == "state": recorded["initial_state"] = "prewarmed"
                elif failure == "unknown": recorded["type"] = "unknown"
                if failure != "missing": capture["detector"]["signal_filter"] = recorded
                result, rows = auto.describe_contact_timing(capture, planner, settings=settings)
                self.assertEqual(result["status"], "ok")
                self.assertEqual(result["filter_label"], "explicit-profile diagnostic")
                self.assertIs(result["matches_recorded_live_filter"], None if failure == "missing" else False)
                self.assertTrue(rows)

    def test_invalid_requested_profile_and_full_metadata_fail_closed(self):
        valid = CausalNotch(profile(100., 2.), RATE).metadata()
        invalid = ["disabled", {}, {"type": "unknown", "frequency_hz": 100., "q": 2.},
                   dict(DISABLED, q=2.), dict(profile(100., 2.), frequency_hz=math.nan),
                   profile(500., 2.), dict(valid, sample_rate_hz=999.),
                   dict(valid, coefficients=[0., 0., 0., 0., 0.])]
        for settings in invalid:
            with self.subTest(settings=settings):
                result, rows = auto.describe_contact_timing(*retained_capture(), settings=settings)
                self.assertEqual(result["status"], "insufficient_data")
                self.assertEqual(rows, [])
                json.dumps(result, allow_nan=False)
        result, rows = auto.describe_contact_timing(*retained_capture(), settings=valid)
        self.assertEqual(result["status"], "ok")
        self.assertTrue(rows)

    def test_disabled_still_rejects_gaps_and_baseline_subtraction_overflow(self):
        for failure in ("gap", "overflow"):
            with self.subTest(failure=failure):
                capture, planner = retained_capture()
                if failure == "gap": del capture["samples"][200]
                else:
                    row = capture["samples"][200]
                    capture["baseline_g"] = (BASELINE[0], BASELINE[1], -1e308)
                    capture["samples"][200] = (*row[:3], 1e308)
                result, rows = auto.describe_contact_timing(capture, planner, settings=DISABLED)
                self.assertEqual(result["status"], "insufficient_data")
                self.assertEqual(rows, [])


if __name__ == "__main__":
    unittest.main()
