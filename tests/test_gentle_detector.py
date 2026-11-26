"""Pre-contact fitting, held-out vetoes and exact causal phase controls."""
import copy
import json
import math
import unittest

import numpy as np

import accel_homing_auto_notch as auto
from accel_homing_filter import CausalNotch, DISABLED, profile
from test_auto_notch import synthetic_item


def motions(**options):
    return [synthetic_item("motion", index, **options) for index in range(3)]


def analyze(items, **options):
    return auto.analyze_gentle(items, threshold_max_g=8., **options)


class GentleDetectorTests(unittest.TestCase):
    def test_decoded_batch_jitter_needs_intact_receipt_and_local_rate(self):
        items = motions()
        for item in items:
            item["record"]["axis_mapping_context"]["backend"] = "klipper_batch"
            cap = item["capture"]
            cap["sensor_context"]["backend"] = "klipper_batch"
            cap["beta_capture_diagnostics"] = {"sample_count": len(cap["samples"]),
                                               "capture_dropped": 0, "gap_observations": 0}
        original = analyze(items)
        self.assertEqual(original["status"], "selected")
        for case in ("isolated", "missing_sample", "sustained", "missing_receipt", "reported_gap"):
            with self.subTest(case=case):
                trial = copy.deepcopy(items)
                cap = trial[0]["capture"]
                # The tester had one 609 us interval at a nominal 622.82 us,
                # followed by normal spacing. Keep the original signal values.
                shift = .0222 / cap["sample_rate_hz"]
                times = np.array([row[0] for row in cap["samples"]])
                times[12:] -= shift
                if case == "sustained":
                    times[13:] -= shift
                cap["samples"] = [(t, *row[1:]) for t, row in zip(times.tolist(), cap["samples"])]
                if case == "missing_sample":
                    cap["samples"].pop(20)
                    cap["beta_capture_diagnostics"]["sample_count"] -= 1
                elif case == "missing_receipt":
                    del cap["beta_capture_diagnostics"]
                elif case == "reported_gap":
                    cap["beta_capture_diagnostics"]["gap_observations"] = 1
                result = analyze(trial)
                if case == "isolated":
                    self.assertEqual(result["status"], "selected", result["reasons"])
                    self.assertEqual(result["profile"]["type"], original["profile"]["type"])
                    self.assertEqual(result["profile"]["q"], original["profile"]["q"])
                    self.assertAlmostEqual(result["profile"]["frequency_hz"], original["profile"]["frequency_hz"], places=4)
                    self.assertAlmostEqual(result["threshold_g"], original["threshold_g"], places=5)
                    timing = result["evidence"]["captures"][0]["sample_timing"]
                    self.assertEqual(timing["isolated_jitter_intervals"], 1)
                else:
                    self.assertEqual(result["status"], "insufficient_data")
                    self.assertIn("missing/nonuniform samples at interval", result["reasons"][0])

    @staticmethod
    def beacon_motions():
        # Field Beacon packets alternate 302.2/345.0 us for 8 samples. Apply that
        # relative pattern to the fixture while keeping its first/last timestamps.
        low, high = .0003022 / .0003236, .0003450 / .0003236
        items = motions()
        for item in items:
            item["record"]["axis_mapping_context"]["backend"] = "beacon_raw"
            cap = item["capture"]
            cap["sensor_context"]["backend"] = "beacon_raw"
            rows = cap["samples"]
            steps = [low if (number // 8) % 2 == 0 else high for number in range(len(rows) - 1)]
            times = rows[0][0] + np.concatenate(([0.], np.cumsum(steps))) / cap["sample_rate_hz"]
            times = rows[0][0] + (times - rows[0][0]) * (rows[-1][0] - rows[0][0]) / (times[-1] - rows[0][0])
            cap["samples"] = [(float(t), *row[1:]) for t, row in zip(times, rows)]
            cap["beta_capture_diagnostics"] = {"sample_count": len(rows), "capture_dropped": 0,
                                               "gap_observations": 0}
        return items

    def test_beacon_packet_jitter_needs_continuity_mean_rate_and_intact_receipt(self):
        items = self.beacon_motions()
        original = analyze(motions())
        result = analyze(items)
        self.assertEqual(result["status"], "selected", result["reasons"])
        self.assertEqual(result["profile"]["type"], original["profile"]["type"])
        self.assertAlmostEqual(result["profile"]["frequency_hz"], original["profile"]["frequency_hz"], delta=1.)
        timing = result["evidence"]["captures"][0]["sample_timing"]
        self.assertGreater(timing["packet_jitter_intervals"], 100)
        self.assertAlmostEqual(timing["mean_rate_hz"], timing["reference_rate_hz"], delta=1.)
        for case in ("missing_sample", "missing_sample_with_receipt", "sustained", "reported_gap",
                     "missing_receipt", "dropped"):
            with self.subTest(case=case):
                trial = copy.deepcopy(items)
                cap = trial[0]["capture"]
                receipt = cap["beta_capture_diagnostics"]
                if case.startswith("missing_sample"):
                    cap["samples"].pop(400)
                    if case == "missing_sample":
                        receipt["sample_count"] -= 1
                elif case == "sustained":
                    # Second half 5% slower: every interval is within the packet
                    # tolerance, but the capture no longer has the reference rate.
                    times = np.array([row[0] for row in cap["samples"]])
                    times[700:] = times[699] + (times[700:] - times[699]) * 1.05
                    cap["samples"] = [(float(t), *row[1:]) for t, row in zip(times, cap["samples"])]
                elif case == "reported_gap":
                    receipt["gap_observations"] = 1
                elif case == "missing_receipt":
                    del cap["beta_capture_diagnostics"]
                else:
                    receipt["capture_dropped"] = 1
                report = analyze(trial)
                self.assertEqual(report["status"], "insufficient_data")
                self.assertIsNone(report["profile"])
                if case == "sustained":
                    self.assertIn("mean sample rate", report["reasons"][0])
                else:
                    self.assertIn("missing sample or nonuniform Beacon packet timing", report["reasons"][0])

    def test_training_only_frequency_generalizes_and_report_is_json(self):
        for frequency in (80., 330.):
            with self.subTest(frequency=frequency):
                report = analyze(motions(frequency=frequency))
                self.assertEqual(report["status"], "selected", report["reasons"])
                self.assertAlmostEqual(report["profile"]["frequency_hz"], frequency, delta=.2)
                self.assertEqual(report["run_id"], "offline_auto")
                self.assertEqual(report["axis"], "x")
                self.assertTrue(report["requires_full_homing_validation"])
                self.assertFalse(report["contact_sensitivity_verified"])
                self.assertEqual(report["evidence"]["frozen_candidate"]["selection_data"], "train_only")
                self.assertFalse(report["evidence"]["frozen_candidate"]["retuned_after_holdout"])
                json.dumps(report, allow_nan=False)

    def test_holdout_changes_cannot_change_frozen_profile_or_threshold(self):
        items = motions()
        good = analyze(items)
        items[2] = synthetic_item("motion", 2, frequency=180.)
        bad = analyze(items)
        self.assertEqual(good["status"], "selected", good["reasons"])
        self.assertEqual(bad["status"], "rejected", bad["reasons"])
        self.assertEqual(good["evidence"]["frozen_candidate"], bad["evidence"]["frozen_candidate"])
        self.assertFalse(bad["evidence"]["holdout"]["accepted"])
        self.assertIsNone(bad["profile"])
        self.assertIsNone(bad["threshold_g"])

    def test_no_crossing_without_absolute_margin_still_rejects_holdout(self):
        items = motions()
        good = analyze(items)
        frozen = good["evidence"]["frozen_candidate"]
        peak = good["evidence"]["holdout"]["metrics"][0]["noise_peak_g"]
        factor = (frozen["threshold_g"] - .05) / peak
        baseline = items[2]["capture"]["baseline_g"]
        items[2]["capture"]["samples"] = [
            (row[0], *(baseline[j] + (row[j + 1] - baseline[j]) * factor for j in range(3)))
            for row in items[2]["capture"]["samples"]]
        bad = analyze(items)
        self.assertEqual(bad["status"], "rejected")
        held_peak = bad["evidence"]["holdout"]["metrics"][0]["noise_peak_g"]
        self.assertLess(held_peak, frozen["threshold_g"])
        self.assertTrue(any("margin" in reason for reason in bad["reasons"]))
        self.assertEqual(good["evidence"]["frozen_candidate"], bad["evidence"]["frozen_candidate"])

    def test_quiet_signal_has_absolute_seed_and_acceptance_margins(self):
        report = analyze(motions(amplitude=.03), threshold_min_g=.001)
        frozen = report["evidence"]["frozen_candidate"]
        self.assertIsNotNone(frozen)
        peak = max(item["noise_peak_g"] for item in frozen["training_metrics"])
        self.assertGreaterEqual(frozen["threshold_g"] - peak, .125 - 1e-12)
        if report["evidence"]["holdout"]["accepted"]:
            self.assertGreaterEqual(frozen["threshold_g"] - report["evidence"]["holdout"]["metrics"][0]["noise_peak_g"], .1 - 1e-12)

    def test_stationary_or_explicit_lower_bound_is_frozen_before_holdout(self):
        report = analyze(motions(), threshold_min_g=3.)
        self.assertEqual(report["status"], "selected", report["reasons"])
        self.assertEqual(report["threshold_g"], 3.)

    def test_broadband_raw_control_is_measured_not_contact_qualified(self):
        report = analyze(motions(broadband=True), threshold_min_g=3.)
        self.assertEqual(report["status"], "no_benefit", report["reasons"])
        self.assertEqual(report["profile"], DISABLED)
        self.assertTrue(report["evidence"]["disabled_control"]["preregistered"])
        self.assertTrue(report["evidence"]["holdout"]["accepted"])
        self.assertFalse(report["contact_sensitivity_verified"])

    def test_threshold_cap_never_clamps_down_unsafe_candidate(self):
        report = auto.analyze_gentle(motions(), threshold_min_g=.001, threshold_max_g=.01)
        self.assertEqual(report["status"], "rejected")
        self.assertIsNone(report["profile"])
        self.assertIsNone(report["threshold_g"])

    def test_braking_is_diagnostic_not_a_fitting_noise_peak(self):
        items = motions()
        before = analyze(items)
        for item in items:
            brake = item["record"]["segments"][-1]
            rows = item["capture"]["samples"]
            for index, row in enumerate(rows):
                if brake["start_time_s"] <= row[0] < brake["end_time_s"]:
                    rows[index] = (row[0], row[1] + 10., row[2], row[3])
        after = analyze(items)
        for key in ("status", "profile", "threshold_g"):
            self.assertEqual(before[key], after[key])
        self.assertEqual(before["evidence"]["captures"], after["evidence"]["captures"])
        for item in after["evidence"]["frozen_candidate"]["training_metrics"]:
            brake = next(phase for phase in item["phases"] if phase["phase"] == "braking")
            self.assertFalse(brake["scored"])
            self.assertGreater(brake["filtered_peak_g"], item["noise_peak_g"])

    def test_mapped_component_ignores_large_orthogonal_noise(self):
        items = motions()
        before = analyze(items)
        for item in items:
            item["capture"]["samples"] = [(row[0], row[1], 100. * math.sin(row[0]), row[3])
                                          for row in item["capture"]["samples"]]
        after = analyze(items)
        self.assertEqual(before, after)
        for item in items:
            item["capture"] = auto.mapped_axis_capture(item["capture"], item["record"]["axis_mapping"])
        self.assertEqual(after, analyze(items))
        for receipt in after["evidence"]["captures"]:
            self.assertEqual(receipt["sample_layout"], "mapped_axis_v1")
            self.assertIsInstance(receipt["baseline_g"], float)
            self.assertIn("scored_signed_mapped_axis_sha256", receipt)

    def test_phase_peaks_cover_both_exact_causal_initial_states(self):
        item = synthetic_item("motion", 0)
        data = auto._prepare(item, "motion", 0, .5, np, None)
        settings = profile(100., 2.)
        measured = auto._gentle_metrics(data, settings, np, None)
        continuous = CausalNotch(settings, data["rate"])
        cold = CausalNotch(settings, data["rate"])
        cont_values, cold_values = [], []
        for index, row in enumerate(data["signed"]):
            cont_values.append(abs(continuous.process(row)[0]))
            cold_values.append(abs(cold.process(row)[0]) if index >= data["start"] else None)
        for phase in measured["phases"]:
            indices = [i for i, t in enumerate(data["times"])
                       if phase["start_time_s"] <= t < phase["end_time_s"]]
            cont_peak = max(cont_values[i] for i in indices)
            cold_peak = cont_peak if phase["phase"] == "startup" else max(cold_values[i] for i in indices)
            self.assertEqual(phase["continuous_startup_peak_g"], cont_peak)
            self.assertEqual(phase["acceleration_reset_peak_g"], cold_peak)
            self.assertEqual(phase["filtered_peak_g"], max(cont_peak, cold_peak))

    def test_continued_startup_state_can_exceed_both_separate_phase_peaks(self):
        item = synthetic_item("motion", 0)
        rate, offset = 3124., 10.
        count, startup_count, accel_count = 2200, 84, 125
        arm = offset + startup_count / rate
        cruise_start, cruise_end = arm + accel_count / rate, offset + 2000 / rate
        end = offset + 2125 / rate
        rows = []
        baseline = item["capture"]["baseline_g"]
        for i in range(count):
            phase = 0. if i < startup_count else 3 * math.pi / 4
            n = i if i < startup_count else i - startup_count
            value = math.sin(2 * math.pi * 175 * n / rate + phase)
            rows.append((offset + i / rate, baseline[0] + value, baseline[1], baseline[2]))
        item["capture"].update(samples=rows, sample_rate_hz=rate, baseline_sample_count=101,
                               baseline_first_sample_time_s=offset - 100 / rate,
                               baseline_last_sample_time_s=offset, capture_cutoff_time_s=offset)
        for segment, left, right in zip(item["record"]["segments"],
                                        (offset, arm, cruise_start, cruise_end),
                                        (arm, cruise_start, cruise_end, end)):
            segment.update(start_time_s=left, end_time_s=right)
        data = auto._prepare(item, "motion", 0, .5, np, None)
        measured = auto._gentle_metrics(data, profile(200., 2.), np, None)
        startup, acceleration = measured["phases"][:2]
        self.assertEqual(acceleration["sample_count"], 125)
        self.assertGreater(acceleration["continuous_startup_peak_g"], 1.4)
        self.assertLess(acceleration["acceleration_reset_peak_g"], .71)
        self.assertLess(startup["filtered_peak_g"], .71)
        self.assertGreater(measured["noise_peak_g"], 1.4)

    def test_raw_disabled_has_identical_continuous_and_cold_metrics(self):
        data = auto._prepare(synthetic_item("motion", 0), "motion", 0, .5, np, None)
        metrics = auto._gentle_metrics(data, DISABLED, np, None)
        for phase in metrics["phases"]:
            self.assertEqual(phase["raw_peak_g"], phase["filtered_peak_g"])
            self.assertEqual(phase["continuous_startup_peak_g"], phase["acceleration_reset_peak_g"])

    def test_invalid_or_nonindependent_controls_fail_closed(self):
        for failure in ("gap", "nan", "clipped", "rate", "stale", "mapping", "context", "duplicate", "order", "short"):
            with self.subTest(failure=failure):
                items = motions()
                capture = items[1]["capture"]
                if failure == "gap": del capture["samples"][250]
                elif failure == "nan": capture["samples"][250] = (capture["samples"][250][0], math.nan, 0., 0.)
                elif failure == "clipped": capture["range_limit_axis_values"] = 1
                elif failure == "rate": capture["sample_rate_hz"] = 900.
                elif failure == "stale":
                    capture["baseline_first_sample_time_s"] -= 1.
                    capture["baseline_last_sample_time_s"] -= 1.
                elif failure == "mapping": capture["axis_mapping"]["source_axis"] = 1
                elif failure == "context": capture["sensor_context"] = {"sensor": "changed"}
                elif failure == "duplicate": items[1] = copy.deepcopy(items[0])
                elif failure == "order": items.reverse()
                else: capture["samples"] = capture["samples"][:200]
                report = analyze(items)
                self.assertEqual(report["status"], "insufficient_data", report["reasons"])
                self.assertIsNone(report["profile"])

    def test_cancellation_propagates(self):
        def cancelled():
            raise RuntimeError("cancelled")
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            analyze(motions(), check_running=cancelled)


if __name__ == "__main__":
    unittest.main()
