"""Standalone contact-timing SVG diagnostics; all motion and samples are synthetic."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import accel_homing_charts as charts


class ContactTimingPlotTests(unittest.TestCase):
    def setUp(self):
        self.planner = {"brake_start_time_s": 14.9, "stop_request_observed_time_s": 14.903,
                        "segments": [
                            {"phase": "acceleration", "start_time_s": 10., "end_time_s": 10.04},
                            {"phase": "cruise", "start_time_s": 10.04, "end_time_s": 14.9},
                            {"phase": "braking", "start_time_s": 14.9, "end_time_s": 14.94}]}
        self.signal = {"filter": {"type": "causal_xyz_notch_zero_at_arm_v1", "frequency_hz": 200., "q": 5.},
                       "analysis_role": "historical_fixed_200Hz_Q5_diagnostic_only",
                       "brake_window": {"start_time_s": 14.8, "end_time_s": 15.,
                                       "filtered_peak_time_s": 14.895,
                                       "filtered_peak_g": 2., "peak_minus_brake_ms": -5.}}

    def test_svg_reports_distinct_timing_and_units_with_two_views(self):
        rows = [(10., .01, .01), (14.895, 3., 2.), (14.94, .1, .05)]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "contact-timing.svg"
            charts.write_contact_timing_plot(path, rows, self.planner, self.signal)
            root = ET.fromstring(path.read_text(encoding="utf-8"))
        labels = " ".join(root.itertext())
        for phrase in ("200 Hz, Q=5", "-5.000 ms", "MCU print_time (s)",
                       "Time relative to scheduled brake start (ms)", "Full scheduled seek",
                       "Zoom near scheduled brake", "does not verify physical contact"):
            self.assertIn(phrase, labels)
        elements = list(root.iter())
        for name in ("scheduled-brake", "stop-request-observed", "window-filtered-peak",
                     "raw-signal", "filtered-signal"):
            self.assertEqual(sum(item.get("class") == name for item in elements), 2)
        for phase in ("acceleration", "cruise", "braking"):
            self.assertTrue(any(item.get("class") == "phase-" + phase for item in elements))

    def test_display_reduction_preserves_channel_extrema_and_yields(self):
        rows = [(10. + index / 10000., 1., .5) for index in range(50000)]
        for index, raw, filtered in ((51, 10., .5), (52, 0., .5),
                                     (48951, 1., 2.), (48952, 1., 0.)):
            rows[index] = (rows[index][0], raw, filtered)
        calls = []
        full = charts._contact_timing_envelope(rows, rows[0][0], rows[-1][0], lambda: calls.append(True))
        zoom = charts._contact_timing_envelope(rows, 14.8, 15., lambda: calls.append(True))
        self.assertLessEqual(len(full), 3072)
        self.assertLessEqual(len(zoom), 3072)
        self.assertEqual(full[0], rows[0])
        self.assertEqual(full[-1], rows[-1])
        self.assertIn(rows[51], full)
        self.assertIn(rows[52], full)
        for trace in (full, zoom):
            self.assertIn(rows[48951], trace)
            self.assertIn(rows[48952], trace)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "contact-timing.svg"
            charts.write_contact_timing_plot(path, rows, self.planner, self.signal,
                                             cooperate=lambda: calls.append(True))
            root = ET.fromstring(path.read_text(encoding="utf-8"))
            self.assertLess(path.stat().st_size, 300000)
        self.assertGreater(len(calls), 150)
        for item in root.iter():
            if item.tag.endswith("polyline"):
                self.assertLessEqual(len(item.get("points").split()), 3072)

    def test_missing_timing_and_empty_capture_do_not_invent_markers(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "contact-timing.svg"
            charts.write_contact_timing_plot(path, [], {}, {})
            root = ET.fromstring(path.read_text(encoding="utf-8"))
        labels = " ".join(root.itertext())
        self.assertIn("timing unavailable", labels)
        self.assertIn("Brake zoom unavailable", labels)
        self.assertIn("No sensor samples", labels)
        self.assertIn("Profile unavailable", labels)
        self.assertNotIn("200 Hz", labels)
        self.assertFalse(any(item.get("class") in ("scheduled-brake", "window-filtered-peak")
                             for item in root.iter()))

    def test_rejects_invalid_rows_and_interrupts_before_output(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "contact-timing.svg"
            for rows in ([(1., float("nan"), 0.)], [(1., -1., 0.)],
                         [(2., 0., 0.), (1., 0., 0.)]):
                with self.assertRaises(ValueError):
                    charts.write_contact_timing_plot(path, rows, {}, {})
                self.assertFalse(path.exists())
            def cancel():
                raise RuntimeError("cancelled")
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                charts.write_contact_timing_plot(path, [(1., 0., 0.)], {}, {}, cooperate=cancel)
            self.assertFalse(path.exists())


if __name__ == "__main__":
    unittest.main()
