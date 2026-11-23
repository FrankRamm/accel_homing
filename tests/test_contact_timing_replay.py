"""Retained-sample contact timing is descriptive and never changes detection."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import accel_homing_auto_notch as auto
from accel_homing_filter import coefficients, profile


RATE = 1000.0
BASELINE = (0.23, -0.18, 0.95)
MAPPING = {"version": 1, "axis": "y", "source_axis": 2, "sign": -1,
           "coordinate_frame": "configured_sensor_stream", "method": "bidirectional_velocity_v1"}


def retained_capture():
    rows = []
    for index in range(601):
        signed = (0.15 * math.sin(2 * math.pi * 200 * (index - 50) / RATE)
                  if index >= 50 else 100.0)
        if index == 390:
            signed -= 4.0
        if index == 550:
            signed += 20.0
        # A much larger signal on an unmapped component must not affect peaks.
        rows.append((10.0 + index / RATE, BASELINE[0] + 200.0, BASELINE[1], BASELINE[2] + signed))
    capture = {"samples": rows, "baseline_g": BASELINE, "dropped": 0, "fault": None,
               "reference_signal_quality": {"amplitude_limited": False},
               "coordinate_frame": "configured_sensor_stream", "axis_mapping": copy.deepcopy(MAPPING),
               "sensor_context": {"sensor": "synthetic", "backend": "test"},
               "auto_notch_replay": {
                   "first_sample_index": 50, "first_sample_matches": 1,
                   "first_sample_time_s": rows[50][0], "complete": True,
                   "initial_state": "zero_at_first_armed_sample",
                   "eligibility_source": "full_precision_source_time",
                   "sample_rate_hz": RATE, "sample_rate_source": "fresh_reference_timestamps",
                   "reference_sample_count": 101, "reference_first_time_s": 9.9,
                   "reference_last_time_s": 10.0, "baseline_source": "armed_detector_reference"},
               "detector": {"axis": "y", "arm_time_s": 10.05, "attempt_count": 1,
                            "impact_metric": "mapped_axis", "axis_mapping": copy.deepcopy(MAPPING),
                            "onset_sample_index": None, "onset_sample_matches": 0, "onset_time_s": None}}
    planner = {"status": "complete", "brake_start_time_s": 10.4,
               "segments": [{"phase": phase, "start_time_s": left, "end_time_s": right}
                            for phase, left, right in (("acceleration", 10.1, 10.2),
                                                      ("cruise", 10.2, 10.4),
                                                      ("braking", 10.4, 10.45))]}
    return capture, planner


class ContactTimingReplayTests(unittest.TestCase):
    def test_fixed_causal_signed_replay_mapping_baseline_and_no_trigger(self):
        capture, planner = retained_capture()
        # Arbitrary live filter metadata and retained live output are irrelevant.
        capture["signal_filter"] = profile(73.0, 20.0)
        capture["filtered_samples"] = [(0.0, 9000.0, 9000.0, 9000.0)]
        before = copy.deepcopy((capture, planner))
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "ok")
        self.assertEqual(summary["filter"]["frequency_hz"], 200.0)
        self.assertEqual(summary["filter"]["q"], 5.0)
        self.assertEqual(summary["filter"]["sample_rate_hz"], RATE)
        self.assertEqual(summary["pre_arm_samples_excluded"], 50)
        self.assertEqual(summary["time_domain"], "main_mcu_print_time")
        self.assertFalse(summary["physical_contact_verified"])
        self.assertEqual((capture, planner), before)
        # Independent scalar difference equation verifies signed filtering
        # before rectification, exact zero state, and no prearm filter warmup.
        b0, b1, b2, a1, a2 = coefficients(profile(200.0, 5.0), RATE)
        x1 = x2 = y1 = y2 = 0.0
        for actual, source in zip(rows, capture["samples"][50:]):
            value = -(source[3] - BASELINE[2])
            expected = b0 * value + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
            self.assertEqual(actual[0], source[0])
            self.assertEqual(actual[1], abs(value))
            self.assertAlmostEqual(actual[2], abs(expected), places=12)
            x2, x1, y2, y1 = x1, value, y1, expected
        self.assertEqual(len(rows), 551)
        self.assertEqual(summary["global_peak"]["raw_peak_time_s"], 10.55)
        self.assertEqual(summary["global_peak"]["filtered_peak_time_s"], 10.55)
        json.dumps(summary, allow_nan=False)

    def test_phase_boundaries_arm_before_motion_and_tail_are_separate(self):
        capture, planner = retained_capture()
        summary, rows = auto.describe_contact_timing(capture, planner)
        phases = summary["phases"]
        self.assertEqual([entry["phase"] for entry in phases],
                         ["armed_before_motion", "acceleration", "cruise", "braking", "tail"])
        self.assertEqual(sum(entry["sample_count"] for entry in phases), len(rows))
        self.assertEqual(phases[0]["covered_end_time_s"], 10.099)
        self.assertEqual(phases[1]["covered_start_time_s"], 10.1)
        self.assertEqual(phases[3]["covered_start_time_s"], 10.4)
        self.assertEqual(phases[4]["covered_start_time_s"], 10.45)
        for entry in phases:
            self.assertEqual(entry["coverage"], "complete")
            self.assertIsNotNone(entry["raw_peak_time_s"])
            self.assertIsNotNone(entry["filtered_peak_time_s"])

    def test_brake_candidate_uses_closed_100_ms_window_and_signed_delta(self):
        summary, _rows = auto.describe_contact_timing(*retained_capture())
        window = summary["brake_window"]
        self.assertEqual(window["coverage"], "complete")
        self.assertEqual(window["half_width_s"], .100)
        self.assertEqual(window["interval"], "closed")
        self.assertAlmostEqual(window["start_time_s"], 10.3)
        self.assertAlmostEqual(window["end_time_s"], 10.5)
        self.assertEqual(window["filtered_peak_time_s"], 10.39)
        self.assertAlmostEqual(window["peak_minus_brake_ms"], -10.0)
        self.assertNotEqual(window["filtered_peak_time_s"], summary["global_peak"]["filtered_peak_time_s"])
        capture, planner = retained_capture()
        source = capture["samples"][390]
        capture["samples"][390] = (*source[:3], BASELINE[2])
        source = capture["samples"][410]
        capture["samples"][410] = (*source[:3], BASELINE[2] - 4.0)
        summary, _rows = auto.describe_contact_timing(capture, planner)
        self.assertAlmostEqual(summary["brake_window"]["peak_minus_brake_ms"], 10.0)

    def test_partial_brake_window_reports_available_data_without_implying_complete_coverage(self):
        capture, planner = retained_capture()
        capture["samples"] = capture["samples"][:421]
        summary, _rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "ok")
        window = summary["brake_window"]
        self.assertEqual(window["coverage"], "partial")
        self.assertEqual(window["covered_end_time_s"], 10.42)
        self.assertEqual(window["filtered_peak_time_s"], 10.39)
        self.assertNotIn("tail", [entry["phase"] for entry in summary["phases"]])

    def test_empty_brake_window_has_no_candidate(self):
        capture, planner = retained_capture()
        planner.update(segments=[], status="unavailable", brake_start_time_s=12.0)
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertTrue(rows)
        self.assertEqual(summary["brake_window"]["coverage"], "none")
        self.assertIsNone(summary["brake_window"]["filtered_peak_time_s"])
        self.assertIsNone(summary["brake_window"]["peak_minus_brake_ms"])

    def test_missing_or_partial_planner_preserves_replay_without_inventing_tail(self):
        capture, planner = retained_capture()
        for unavailable in (None, {}, {"status": "unavailable", "segments": []}):
            with self.subTest(planner=unavailable):
                summary, rows = auto.describe_contact_timing(capture, unavailable)
                self.assertEqual(summary["status"], "ok")
                self.assertEqual(len(rows), 551)
                self.assertEqual([entry["phase"] for entry in summary["phases"]], ["unclassified"])
                self.assertEqual(summary["brake_window"]["coverage"], "unavailable")
        planner.update(status="partial", segments=planner["segments"][:2], brake_start_time_s=None)
        summary, _rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["phases"][-1]["phase"], "unclassified_after_planner")
        self.assertEqual(summary["brake_window"]["coverage"], "unavailable")
        # A matched stop request is useful even when detection preempted the
        # planner's braking phase. Its window is independent of scheduled brake.
        planner.update(axis="y", arm_time_s=10.05, clock_domain="main_mcu_print_time",
                       stop_request_observed_time_s=10.395)
        summary, _rows = auto.describe_contact_timing(capture, planner)
        stop = summary["stop_request_window"]
        self.assertEqual(stop["coverage"], "complete")
        self.assertEqual(stop["filtered_peak_time_s"], 10.39)
        self.assertAlmostEqual(stop["peak_minus_stop_request_ms"], -5.0)
        self.assertEqual(summary["brake_window"]["coverage"], "unavailable")
        planner["arm_time_s"] += 1.0
        summary, _rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["stop_request_window"]["coverage"], "unavailable")
        self.assertIsNone(summary["stop_request_window"]["peak_minus_stop_request_ms"])

    def test_invalid_planner_is_unavailable_without_losing_valid_capture(self):
        for change in (lambda planner: planner["segments"][1].update(start_time_s=10.201),
                       lambda planner: planner.update(brake_start_time_s=10.41),
                       lambda planner: planner.update(clock_domain="host_monotonic"),
                       lambda planner: planner.update(time_domain="host_monotonic")):
            capture, planner = retained_capture()
            change(planner)
            summary, rows = auto.describe_contact_timing(capture, planner)
            self.assertEqual(summary["status"], "ok")
            self.assertTrue(rows)
            self.assertEqual(summary["planner_status"], "unavailable")
            self.assertTrue(summary["planner_reason"])
            self.assertIsNone(summary["brake_window"]["peak_minus_brake_ms"])

    def test_bad_capture_provenance_returns_clean_insufficient_data(self):
        changes = {
            "dropped": lambda c: c.update(dropped=1),
            "fault": lambda c: c.update(fault="missing packet"),
            "saturation": lambda c: c.update(amplitude_limited=True),
            "reference_saturation": lambda c: c["reference_signal_quality"].update(amplitude_limited=True),
            "native_marker": lambda c: c.update(native_clip_decode={"marker_axis_values": 1}),
            "mapping": lambda c: c["axis_mapping"].update(source_axis=1),
            "baseline": lambda c: c.update(baseline_g=None),
            "ambiguous_first": lambda c: c["auto_notch_replay"].update(first_sample_matches=2),
            "wrong_first": lambda c: c["auto_notch_replay"].update(first_sample_time_s=10.051),
            "missing_arm": lambda c: c["detector"].pop("arm_time_s"),
            "incomplete": lambda c: c["auto_notch_replay"].update(complete=False),
            "rate_source": lambda c: c["auto_notch_replay"].pop("sample_rate_source"),
            "rate_mismatch": lambda c: c["auto_notch_replay"].update(sample_rate_hz=900.0),
            "first_eligibility": lambda c: c["detector"].update(arm_time_s=10.04),
            "multiple_attempts": lambda c: c["detector"].update(attempt_count=2),
            "time_domain": lambda c: c.update(sample_time_domain="host_monotonic"),
        }
        for name, change in changes.items():
            with self.subTest(case=name):
                capture, planner = retained_capture()
                change(capture)
                summary, rows = auto.describe_contact_timing(capture, planner)
                self.assertEqual(summary["status"], "insufficient_data")
                self.assertTrue(summary["reason"])
                self.assertEqual(rows, [])
                self.assertIsNone(summary["global_peak"])
                json.dumps(summary, allow_nan=False)

    def test_nonmonotonic_nonfinite_and_missing_samples_are_rejected(self):
        for failure in ("duplicate", "reversed", "nan", "gap", "overflow"):
            with self.subTest(failure=failure):
                capture, planner = retained_capture()
                row = capture["samples"][200]
                if failure == "duplicate":
                    capture["samples"][200] = (capture["samples"][199][0], *row[1:])
                elif failure == "reversed":
                    capture["samples"][200] = (10.0, *row[1:])
                elif failure == "nan":
                    capture["samples"][200] = (*row[:3], float("nan"))
                elif failure == "gap":
                    del capture["samples"][200]
                else:
                    capture["baseline_g"] = (BASELINE[0], BASELINE[1], -1e308)
                    capture["samples"][200] = (*row[:3], 1e308)
                summary, rows = auto.describe_contact_timing(capture, planner)
                self.assertEqual(summary["status"], "insufficient_data")
                self.assertEqual(rows, [])

    def test_discarded_prearm_gaps_do_not_invalidate_exact_armed_replay(self):
        capture, planner = retained_capture()
        expected, expected_rows = auto.describe_contact_timing(capture, planner)
        # Retained raw capture can include unrelated acquisition before its
        # exact reference and first armed sample. Those rows never warm state.
        del capture["samples"][10:20]
        capture["auto_notch_replay"]["first_sample_index"] -= 10
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "ok")
        self.assertEqual(rows, expected_rows)
        self.assertEqual(summary["global_peak"], expected["global_peak"])
        # The same missing-data pattern within the armed suffix invalidates it.
        del capture["samples"][100:110]
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "insufficient_data")
        self.assertEqual(rows, [])

    def test_bounded_packet_timestamp_jitter_is_reported_without_resampling(self):
        capture, planner = retained_capture()
        expected, expected_rows = auto.describe_contact_timing(capture, planner)
        # +/-14% alternating periods preserve the sample sequence and mean
        # rate. Real native packets need not satisfy instantaneous +/-2%.
        for index in range(51, len(capture["samples"]), 2):
            row = capture["samples"][index]
            capture["samples"][index] = (row[0] + .00014, *row[1:])
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "ok")
        self.assertEqual([row[1:] for row in rows], [row[1:] for row in expected_rows])
        self.assertEqual([row[0] for row in rows], [row[0] for row in capture["samples"][50:]])
        self.assertAlmostEqual(summary["sampling"]["maximum_interval_deviation_fraction"], .14)
        self.assertAlmostEqual(summary["sampling"]["armed_mean_sample_rate_hz"], RATE)
        self.assertEqual(summary["global_peak"]["filtered_peak_g"], expected["global_peak"]["filtered_peak_g"])

    def test_armed_mean_rate_drift_is_rejected_even_with_individually_plausible_intervals(self):
        capture, planner = retained_capture()
        for index in range(51, len(capture["samples"])):
            row = capture["samples"][index]
            capture["samples"][index] = (10.05 + (row[0] - 10.05) * 1.10, *row[1:])
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "insufficient_data")
        self.assertIn("mean sample rate", summary["reason"])
        self.assertEqual(rows, [])

    def test_measured_rate_must_allow_200_hz_notch(self):
        capture, planner = retained_capture()
        # Preserve all time/provenance relationships at a valid 400 Hz rate;
        # exactly Nyquist is still invalid for the fixed requested notch.
        stretch = lambda value: 10.0 + (value - 10.0) * 2.5
        capture["samples"] = [(stretch(row[0]), *row[1:]) for row in capture["samples"]]
        meta = capture["auto_notch_replay"]
        for key in ("first_sample_time_s", "reference_first_time_s", "reference_last_time_s"):
            meta[key] = stretch(meta[key])
        meta["sample_rate_hz"] = 400.0
        capture["detector"]["arm_time_s"] = stretch(capture["detector"]["arm_time_s"])
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "insufficient_data")
        self.assertIn("Nyquist", summary["reason"])
        self.assertEqual(rows, [])

    def test_native_first_identity_uses_original_clock_not_later_time_mapping(self):
        capture, planner = retained_capture()
        meta = capture["auto_notch_replay"]
        meta.update(eligibility_source="native_sensor_clock", arm_clock=1005,
                    first_sample=(1000, 15, 5, 16))
        capture["detector"]["arm_time_s"] += .000050
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "ok")
        self.assertTrue(rows)
        meta["arm_clock"] -= 1
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "insufficient_data")
        self.assertEqual(rows, [])

    def test_only_documented_reference_to_arm_gap_is_allowed(self):
        capture, planner = retained_capture()
        meta = capture["auto_notch_replay"]
        reference = [(9.9 + index / RATE, *BASELINE) for index in range(101)]
        armed = [(row[0] + .21, *row[1:]) for row in capture["samples"][50:]]
        capture["samples"] = reference + armed
        meta.update(first_sample_index=101, first_sample_time_s=armed[0][0])
        capture["detector"]["arm_time_s"] += .21
        meta["reference_validation"] = {
            "source": "live_prepare_reference_check", "clock_domain": "main_mcu_print_time",
            "result": "ready", "time_s": 10.01,
            "scheduled_arm_time_s": capture["detector"]["arm_time_s"]}
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "ok")
        self.assertEqual(rows[0][0], armed[0][0])
        del meta["reference_validation"]
        summary, rows = auto.describe_contact_timing(capture, planner)
        self.assertEqual(summary["status"], "insufficient_data")
        self.assertEqual(rows, [])

    def test_cancellation_and_context_errors_propagate(self):
        for kind in (RuntimeError, ValueError, auto.InsufficientData):
            for fail_at in (1, 2, 3, 4, 6, 8, 10, 12, 15):
                with self.subTest(kind=kind, fail_at=fail_at):
                    calls = []

                    def check():
                        calls.append(1)
                        if len(calls) == fail_at:
                            raise kind("cancelled or changed context")

                    with self.assertRaisesRegex(kind, "cancelled or changed context"):
                        auto.describe_contact_timing(*retained_capture(), check_running=check)


if __name__ == "__main__":
    unittest.main()
