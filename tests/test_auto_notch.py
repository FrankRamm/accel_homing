"""Offline automatic-notch evidence: independent strokes and raw stop events."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import accel_homing_auto_notch as auto
from accel_homing_filter import CausalNotch, DISABLED, coefficients, profile


RATE = 1000.0
RAW_THRESHOLD = 2.5


def synthetic_item(kind, index, *, frequency=100.0, amplitude=1.0, broadband=False):
    """Actual-scale time-aligned XYZ, 40 ms ramps, noise, and one-sample events.

    Every acquisition has a distinct offset, phase and random realization. The
    raw stop tail deliberately contains a much larger braking pulse: it must
    never contribute to reference retention or crossing acceptance.
    """
    offset = 10.0 * (index + (1 if kind == "motion" else 5))
    rng = np.random.default_rng(index + (10 if kind == "motion" else 20))
    n = np.arange(1401)
    relative = n / RATE
    times = offset + relative
    arm, cruise_start, cruise_end, motion_end = offset + .05, offset + .15, offset + 1.2, offset + 1.3
    envelope = np.clip((relative - .05) / .04, 0, 1)
    phase = .17 * index
    carrier = amplitude * np.sin(2 * np.pi * frequency * (relative - .05) + phase) * envelope
    noise = rng.normal(0, .012 if not broadband else .35, (len(n), 3))
    xyz = noise
    if not broadband:
        xyz[:, 0] += carrier
        xyz[:, 1] += .18 * amplitude * np.sin(2 * np.pi * frequency * (relative - .05) + .4 + phase) * envelope
    baseline = np.array((.23, -.18, .95))
    if kind == "home":
        xyz[1100, 2] += 6.0
        xyz[1101, 2] -= 1.0  # Short bipolar impact, but only first row precedes stop.
        xyz[1140, 0] += 12.0  # Stop response cannot be used as a retained contact.
    rows = [tuple(float(value) for value in row) for row in np.column_stack((times, xyz + baseline))]
    capture = {"samples": rows, "baseline_g": baseline.tolist(), "dropped": 0, "fault": None,
               "amplitude_limited": False, "clipped_axis_values": 0, "range_limit_axis_values": 0,
               "reference_signal_quality": {"amplitude_limited": False}}
    record = {"run_id": "offline_auto", "axis": "x", "physical_position_verified": False}
    if kind == "motion":
        record.update(iteration=index + 1, phase_time_source="actual_commanded_trapq",
                      effective_speed_mm_s=50.0, effective_accel_mm_s2=500.0,
                      segments=[{"phase": phase_name, "axis": "x", "start_time_s": left, "end_time_s": right}
                                for phase_name, left, right in (
                                    ("startup", offset, arm), ("acceleration", arm, cruise_start),
                                    ("cruise", cruise_start, cruise_end), ("braking", cruise_end, motion_end))])
        capture.update(raw_samples_validated=True, baseline_sample_count=101,
                       baseline_first_sample_time_s=offset - .1, baseline_last_sample_time_s=offset,
                       sample_rate_hz=RATE, capture_cutoff_time_s=offset)
    else:
        record.update(workflow_attempt=index + 2, auto_notch_reference="qualified_raw_trigger_reference",
                      result="impact", travel_mm_exact=50.0, threshold_g=RAW_THRESHOLD)
        start, onset = 50, 1100
        capture.update(signal_filter=dict(DISABLED),
                       auto_notch_replay={"first_sample_index": start, "first_sample_matches": 1,
                                         "first_sample_time_s": rows[start][0],
                                         "eligibility_source": "full_precision_source_time",
                                         "initial_state": "zero_at_first_armed_sample", "complete": True,
                                         "sample_rate_hz": RATE, "sample_rate_source": "fresh_reference_timestamps",
                                         "reference_sample_count": 101, "reference_first_time_s": offset - .1,
                                         "reference_last_time_s": offset, "baseline_source": "armed_detector_reference"},
                       detector={"axis": "x", "arm_time_s": arm, "onset_sample_index": onset,
                                 "onset_sample_matches": 1, "onset_time_s": rows[onset][0],
                                 "attempt_count": 1, "impact_metric": "xyz_magnitude",
                                 "threshold_g": RAW_THRESHOLD,
                                 "homing_speed_mm_s": 50.0, "homing_accel_mm_s2": 500.0})
    return {"record": record, "capture": capture}


def dataset(**options):
    return ([synthetic_item("motion", index, **options) for index in range(3)],
            [synthetic_item("home", index, **options) for index in range(10)])


def run_analysis(motions, homes, **options):
    return auto.analyze(motions, homes, raw_threshold_g=RAW_THRESHOLD,
                        threshold_min_g=.5, threshold_max_g=8.0, **options)


class AutoNotchTests(unittest.TestCase):
    def test_recurring_signed_spectrum_selects_frequency_q_and_frozen_threshold(self):
        motions, homes = dataset()
        report = run_analysis(motions, homes)
        self.assertEqual(report["status"], "selected", report["reasons"])
        self.assertAlmostEqual(report["profile"]["frequency_hz"], 100.0, delta=.3)
        self.assertIn(report["profile"]["q"], auto.Q_GRID)
        self.assertGreaterEqual(report["threshold_g"], .5)
        self.assertLess(report["threshold_g"], RAW_THRESHOLD)
        self.assertTrue(report["evidence"]["holdout"]["accepted"])
        self.assertLessEqual(len(report["evidence"]["candidates"]), auto.MAX_CANDIDATES)
        self.assertTrue(report["requires_full_homing_validation"])
        self.assertFalse(report["physical_position_verified"])
        self.assertEqual(report["evidence"]["frozen_candidate"]["selection_data"], "train_only")
        self.assertEqual(len(report["plot"]["spectra"]), 3)
        self.assertEqual(len(report["plot"]["traces"]), 13)
        for trace in report["plot"]["traces"]:
            self.assertLessEqual(len(trace["time_s"]), auto.MAX_TRACE_POINTS)
            if trace["kind"] == "home":
                self.assertEqual(trace["time_s"][-1], trace["reference_stop_time_s"])
                self.assertLessEqual(trace["filtered_crossing_time_s"], trace["reference_stop_time_s"])
                self.assertLess(max(trace["raw_magnitude_g"]), 7.0)
        json.dumps(report, allow_nan=False)

    def test_replay_exact_signed_zero_state_excludes_prewarm_and_post_stop(self):
        motions, homes = dataset()
        data = auto._prepare(motions[0], "motion", 0, RAW_THRESHOLD, np, None)
        settings = profile(100., 10.)
        actual = auto._replay(data, settings, np, None)
        filt = CausalNotch(settings, RATE)
        expected = [filt.process(row) for row in data["signed"][data["start"]:data["end"]]]
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(data["start"], 50)
        altered = copy.deepcopy(motions[0])
        altered["capture"]["samples"][:50] = [(row[0], row[1] + 10, row[2] - 5, row[3])
                                                for row in altered["capture"]["samples"][:50]]
        shifted = auto._prepare(altered, "motion", 0, RAW_THRESHOLD, np, None)
        np.testing.assert_array_equal(actual, auto._replay(shifted, settings, np, None))
        home = auto._prepare(homes[0], "home", 0, RAW_THRESHOLD, np, None)
        home_replay = auto._replay(home, settings, np, None)
        self.assertEqual(len(home_replay), 1100 - 50 + 1)
        self.assertEqual(home["times"][home["end"] - 1], homes[0]["capture"]["detector"]["onset_time_s"])
        native = copy.deepcopy(homes[0])
        meta = native["capture"]["auto_notch_replay"]
        meta.update(eligibility_source="native_sensor_clock", arm_clock=1005, first_sample=(1000, 15, 5, 16))
        # Later native clock synchronization shifts the exported print-time
        # rows relative to the original arm timestamp; source clocks stay exact.
        native["capture"]["detector"]["arm_time_s"] += .000050
        checked = auto._prepare(native, "home", 0, RAW_THRESHOLD, np, None)
        np.testing.assert_array_equal(home_replay, auto._replay(checked, settings, np, None))
        meta["arm_clock"] -= 1
        with self.assertRaisesRegex(auto.InsufficientData, "first eligible"):
            auto._prepare(native, "home", 0, RAW_THRESHOLD, np, None)

    def test_broadband_valid_data_recommends_disabled_distinct_from_invalid(self):
        motions, homes = dataset(broadband=True)
        report = run_analysis(motions, homes)
        self.assertEqual(report["status"], "no_benefit", report["reasons"])
        self.assertEqual(report["profile"], DISABLED)
        self.assertEqual(report["threshold_g"], RAW_THRESHOLD)
        self.assertFalse(report["evidence"]["recurring_frequency_hz"])

    def test_missing_rate_baseline_gap_nonfinite_clipping_and_circular_labels_reject(self):
        cases = ("rate", "rate_mismatch", "baseline", "gap", "nan", "clipping", "circular", "arm", "stale", "truncated")
        for case in cases:
            with self.subTest(case=case):
                motions, homes = dataset()
                target = homes[0]["capture"]
                if case == "rate": del target["auto_notch_replay"]["sample_rate_source"]
                elif case == "rate_mismatch": target["auto_notch_replay"]["sample_rate_hz"] = 900.
                elif case == "baseline": del target["baseline_g"]
                elif case == "gap": del target["samples"][300]
                elif case == "nan": target["samples"][300] = (target["samples"][300][0], float("nan"), 0., 0.)
                elif case == "clipping": target["range_limit_axis_values"] = 1
                elif case == "circular": homes[0]["record"]["auto_notch_reference"] = "threshold_only"
                elif case == "arm": target["auto_notch_replay"]["first_sample_matches"] = 2
                elif case == "stale":
                    target["auto_notch_replay"]["reference_first_time_s"] -= 1
                    target["auto_notch_replay"]["reference_last_time_s"] -= 1
                else: motions[0]["capture"]["samples"] = motions[0]["capture"]["samples"][:1200]
                report = run_analysis(motions, homes)
                self.assertEqual(report["status"], "insufficient_data", report)
                self.assertIsNone(report["decision"])
                self.assertIsNone(report["profile"])

    def test_holdout_failure_vetoes_frozen_winner_without_retuning(self):
        motions, homes = dataset()
        good = run_analysis(motions, homes)
        motions[2] = synthetic_item("motion", 2, frequency=180.)
        rejected = run_analysis(motions, homes)
        self.assertEqual(rejected["status"], "no_benefit")
        self.assertEqual(rejected["profile"], DISABLED)
        self.assertEqual(good["evidence"]["frozen_candidate"], rejected["evidence"]["frozen_candidate"])
        self.assertFalse(rejected["evidence"]["holdout"]["accepted"])
        self.assertFalse(rejected["evidence"]["frozen_candidate"]["retuned_after_holdout"])
        self.assertEqual(good["evidence"]["candidates"], rejected["evidence"]["candidates"])
        self.assertTrue(rejected["evidence"]["disabled_control"]["accepted"])
        # The preregistered raw fallback must itself pass held-out free travel.
        motions[2] = synthetic_item("motion", 2, frequency=180., amplitude=4.)
        unsafe_raw = run_analysis(motions, homes)
        self.assertEqual(unsafe_raw["status"], "rejected")
        self.assertIsNone(unsafe_raw["profile"])
        self.assertIsNone(unsafe_raw["threshold_g"])
        self.assertFalse(unsafe_raw["evidence"]["disabled_control"]["accepted"])
        self.assertEqual(good["evidence"]["frozen_candidate"], unsafe_raw["evidence"]["frozen_candidate"])

    def test_heldout_short_event_erosion_cannot_be_repaired_by_stop_tail(self):
        motions, homes = dataset()
        good = run_analysis(motions, homes)
        self.assertEqual(good["status"], "selected")
        # A one-sample raw event just clears the raw threshold; a much larger
        # post-trigger pulse is present but outside every candidate replay.
        rows = homes[9]["capture"]["samples"]
        baseline = homes[9]["capture"]["baseline_g"]
        rows[1100] = (rows[1100][0], baseline[0], baseline[1], baseline[2] + RAW_THRESHOLD)
        metadata = auto._prepare(homes[9], "home", 9, RAW_THRESHOLD, np, None)
        settings = good["profile"]
        filtered = auto._replay(metadata, settings, np, None)
        metrics, levels = auto._metrics(metadata, filtered, np)
        above_reference = max(RAW_THRESHOLD, float(levels[-1]) + .1)
        metrics.update(filtered_crossing_time_s=auto._crossing(metadata, levels, above_reference, np),
                       reference_crossing_shift_s=None)
        reasons = auto._acceptance([metrics], above_reference, "holdout")
        self.assertTrue(any("no causal crossing" in reason for reason in reasons))
        self.assertLess(float(np.max(filtered[:, 0])), 3.)

    def test_startup_ringing_increase_and_early_crossing_are_rejected(self):
        motions, homes = dataset()
        for item in motions:
            baseline, rows = item["capture"]["baseline_g"], item["capture"]["samples"]
            for index in range(50, 150):
                rows[index] = (rows[index][0], baseline[0] + 1.0, baseline[1], baseline[2])
        # A bounded 1 g commanded-acceleration step rings above its raw maximum
        # through the notch despite excellent 100 Hz cruise suppression.
        report = run_analysis(motions, homes)
        self.assertEqual(report["status"], "no_benefit")
        self.assertTrue(all(any("startup/acceleration peak increased" in reason for reason in candidate["reasons"])
                            for candidate in report["evidence"]["candidates"]))
        motions, homes = dataset()
        baseline, rows = homes[9]["capture"]["baseline_g"], homes[9]["capture"]["samples"]
        rows[60] = (rows[60][0], baseline[0], baseline[1], baseline[2] + 1.0)
        # This held-out startup impulse is below the raw 2.5 g threshold, but
        # above the frozen notch threshold. It must veto the trained candidate.
        report = run_analysis(motions, homes)
        self.assertEqual(report["status"], "no_benefit")
        self.assertTrue(any("outside allowed" in reason for reason in report["evidence"]["holdout"]["reasons"]))

    def test_true_digital_bandwidth_matches_frequency_response_near_nyquist(self):
        for frequency in (50., 400.):
            q = 5.
            b0, b1, b2, a1, a2 = coefficients(profile(frequency, q), RATE)
            frequencies = np.linspace(.001, RATE / 2 - .001, 250000)
            z = np.exp(-2j * np.pi * frequencies / RATE)
            power = np.abs((b0 + b1 * z + b2 * z ** 2) / (1 + a1 * z + a2 * z ** 2)) ** 2
            inside = frequencies[power <= .5]
            self.assertAlmostEqual(auto.digital_bandwidth(frequency, q, RATE), float(inside[-1] - inside[0]), delta=.005)
        self.assertLess(auto.digital_bandwidth(400., 5., RATE), 400. / 5. / 2)
        # Long captures cannot claim finer resolution than the capped Welch
        # segments: 10 s at 12.8 kHz still has 6.25 Hz bins with 2048 samples.
        rate, duration = 12800., 10.
        signal = np.zeros((int(rate * duration), 3))
        signal[:, 0] = np.sin(2 * np.pi * 30. * np.arange(len(signal)) / rate)
        frequency_bins, _, size, _ = auto._welch(signal, rate, np)
        self.assertEqual(size, auto.MAX_WELCH_SEGMENT)
        resolution = auto._resolution(30., 20., rate, duration, float(frequency_bins[1]))
        self.assertGreater(resolution["digital_bandwidth_hz"], 2 / duration)
        self.assertFalse(resolution["resolved"])
        self.assertEqual(resolution["minimum_resolved_bandwidth_hz"], 12.5)

    def test_duplicate_acquisition_and_short_cruise_are_insufficient(self):
        motions, homes = dataset()
        homes[9]["capture"] = copy.deepcopy(homes[8]["capture"])
        copied = homes[9]["capture"]
        copied["samples"] = [(row[0] + 10.,) + row[1:] for row in copied["samples"]]
        for key in ("first_sample_time_s", "reference_first_time_s", "reference_last_time_s"):
            copied["auto_notch_replay"][key] += 10.
        for key in ("arm_time_s", "onset_time_s"):
            copied["detector"][key] += 10.
        row = copied["samples"][1140]
        copied["samples"][1140] = (row[0], row[1] + .01, row[2], row[3])
        duplicate = run_analysis(motions, homes)
        self.assertEqual(duplicate["status"], "insufficient_data")
        self.assertIn("duplicate", duplicate["reasons"][0])
        motions, homes = dataset()
        segments = motions[0]["record"]["segments"]
        segments[2]["end_time_s"] = segments[2]["start_time_s"] + .20
        segments[3]["start_time_s"] = segments[2]["end_time_s"]
        short = run_analysis(motions, homes)
        self.assertEqual(short["status"], "insufficient_data")
        self.assertIn("cruise duration", short["reasons"][0])

    def test_cancel_propagates_inside_causal_replay(self):
        motions, homes = dataset()
        class Cancelled(Exception): pass
        calls = []
        def cancel():
            calls.append(1)
            if len(calls) > 90:
                raise Cancelled("operator cancelled")
        with self.assertRaisesRegex(Cancelled, "operator cancelled"):
            run_analysis(motions, homes, check_running=cancel)
        self.assertEqual(len(calls), 91)


if __name__ == "__main__":
    unittest.main()
