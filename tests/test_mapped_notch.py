"""Mapped-notch parity and mapped transport freshness on synthetic drivers."""
import itertools
import math
import unittest
from unittest import mock
from harness import acquire_reference, make_homing
from accel_homing_filter import DISABLED, profile
from accel_homing_charts import _capture_data, _plot_capture, _plot_notch_auto, _prepared_capture, prepare_capture
from test_backends import generic
from test_mapped_detector import mapping, deliver
from test_notch import oracle
from test_session import FakeRail, info, with_endstops


def prepared(source=2, sign=-1, backend=False, legacy=False, compare=False, notch=True):
    settings = {"threshold": .75, "y_notch_frequency": 100. if notch else 0.}
    obj = (generic("lis2dw cartographer", options=settings, legacy=legacy) if backend else
           make_homing(axes_map=((2, -1), (0, 1), (1, -1)), options=settings))
    selected = mapping("y", source, sign)
    obj.workflows.detection_axis_mapping = lambda axis: selected
    obj.workflows.simple_detection = lambda axis: True
    if compare:
        obj.workflows._experimental_home = lambda axis: axis == "y"
        obj.workflows.session = {"experiment": {"impact_metric": "mapped_axis", "mode": "COMPARE", "max_sample_age_s": .020}}
    endstop = with_endstops(obj, ("y",))["y"]
    obj.session_axes = {"y": FakeRail(endstop, info(retract=0.))}
    obj.reactor.now = 10.
    acquire_reference(obj, endstop)
    when = obj.backend.last_sample_time + 1. / obj.backend.rate if backend else obj.reactor.now + .001
    attempt = obj.prepare(endstop, when)
    obj.start_capture()
    obj.arm(attempt)
    return obj, attempt, selected


class MappedNotchTests(unittest.TestCase):
    def test_signed_mapped_component_runtime_capture_and_replay_match(self):
        for backend, legacy, source, sign in itertools.product((False, True), (False, True), range(3), (-1, 1)):
            if not backend and legacy:
                continue
            with self.subTest(backend=backend, legacy=legacy, source=source, sign=sign):
                obj, attempt, selected = prepared(source, sign, backend, legacy)
                self.assertEqual(obj.filter_settings("x"), DISABLED)
                self.assertIsNotNone(attempt.notch)
                rate, size, hit_at, ramp, width = (3200., 640, 608, 400., 16) if backend else (1000., 352, 320, 250., 8)
                rows = []
                for n in range(size):
                    selected_value = min(1., n / ramp) * math.sin(2 * math.pi * 100 * n / rate)
                    row = [4. + .1 * math.sin(n) for _ in range(3)]
                    row[attempt.index] = -3. if n == hit_at else selected_value
                    rows.append(tuple(row))
                for start in range(0, size, width):
                    deliver(obj, attempt, rows[start:start + width], offset=start / rate)
                self.assertEqual(attempt.reason, "impact")
                self.assertEqual(attempt.onset_sample[2], hit_at % width)
                obj.disarm()
                capture = obj.stop_capture()
                self.assertIsNone(capture["fault"])
                self.assertEqual(capture["detector"]["onset_sample_index"], hit_at)
                self.assertEqual(capture["detector"]["impact_metric"], "mapped_axis")
                self.assertEqual(capture["signal_filter"]["axis_mapping"], selected)
                self.assertEqual(capture["signal_filter"]["impact_metric"], "mapped_axis")
                self.assertEqual(capture["sensor_context"], attempt.capture_sensor_context)
                self.assertEqual(attempt.record["signal_filter"]["axis_mapping"], selected)
                self.assertEqual(capture["auto_notch_replay"]["first_sample_index"], 0)
                self.assertTrue(capture["auto_notch_replay"]["complete"])
                deltas = [tuple(row[n + 1] - capture["baseline_g"][n] for n in range(3)) for row in capture["samples"]]
                expected = oracle(deltas, profile(100., 10.), attempt.notch.rate)
                first = next(n for n, row in enumerate(expected) if abs(row[source]) >= .75)
                self.assertEqual(first, hit_at)
                self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, abs(expected[first][source]), places=10)
                for actual, wanted in zip(capture["filtered_samples"], expected):
                    for value, reference in zip(actual[1:], wanted):
                        self.assertAlmostEqual(value, reference, places=10)
                trace, metadata = _capture_data(capture, attempt.record)
                self.assertEqual(metadata["axis_mapping"], selected)
                self.assertTrue(metadata["raw_trace"])
                capture["signal_filter"]["axis_mapping"]["source_axis"] = 99
                self.assertEqual(attempt.axis_mapping["source_axis"], source)

    def test_filtered_chart_uses_mapped_raw_overlay_and_threshold(self):
        from matplotlib.figure import Figure
        obj, attempt, selected = prepared(source=1)
        deliver(obj, attempt, [(1.5, 4., 6.)] * 8)
        obj.disarm()
        capture = obj.stop_capture()
        figure = Figure()
        receipt, record, metadata = _prepared_capture(prepare_capture("b" * 32, "y", capture, attempt.record))
        _plot_capture(figure, {"record": record, "trace": receipt.trace, "capture": metadata,
                              "axis": "y", "timestamp": "Synthetic test"})
        labels = [line.get_label() for line in figure.axes[0].lines]
        self.assertIn("Raw mapped-axis magnitude", labels)
        self.assertFalse(any("XYZ magnitude" in label for label in labels))
        self.assertTrue(any("Physical Y trigger" in label for label in labels))

    def test_auto_chart_labels_selected_component_and_signed_spectrum(self):
        from matplotlib.figure import Figure
        selected = mapping("y", 2, -1)
        report = {"impact_metric": "mapped_axis", "axis_mapping": selected,
                  "decision": "filtered", "threshold_g": .8,
                  "plot": {"impact_metric": "mapped_axis", "axis_mapping": selected,
                           "spectra": [{"role": "train", "frequency_hz": [1., 2.],
                                        "raw_power_g2_hz": [1., 2.], "filtered_power_g2_hz": [.5, 1.]}],
                           "traces": [{"kind": "motion", "role": "holdout", "time_s": [0., 1.],
                                       "raw_magnitude_g": [.2, .3], "filtered_magnitude_g": [.1, .2]}]}}
        figure = Figure()
        _plot_notch_auto(figure, {"analysis": report, "axis": "y", "timestamp": "Synthetic test"})
        self.assertEqual(figure.axes[0].get_ylabel(), "Mapped signed component PSD (g²/Hz)")
        self.assertEqual(figure.axes[1].get_ylabel(), "Mapped component magnitude (g)")
        self.assertIn("Physical Y (= -stream Z)", figure.axes[1].get_title())

    def test_compare_keeps_mapping_notch_and_rejects_old_current_dispatch(self):
        for backend, notch in itertools.product((False, True), (False, True)):
            with self.subTest(backend=backend, notch=notch):
                obj, attempt, selected = prepared(backend=backend, compare=True, notch=notch)
                self.assertIsNone(attempt.experimental)
                self.assertEqual(attempt.mapped_compare_max_age_s, .020)
                obj.workflows.session["experiment"]["max_sample_age_s"] = .5
                obj._configure_experimental(attempt)
                self.assertEqual(attempt.mapped_compare_max_age_s, .020)
                inspect = obj._inspect
                def delayed(params):
                    obj.reactor.now += .030
                    return inspect(params)
                with mock.patch.object(obj, "_inspect", side_effect=delayed):
                    row = tuple(2. if n == attempt.index else 8. for n in range(3))
                    deliver(obj, attempt, [row] * 8)
                self.assertEqual(attempt.reason, "stale_data")
                self.assertIsNone(attempt.onset_sample)
                obj.disarm()
                capture = obj.stop_capture()
                self.assertEqual(capture["detector"]["axis_mapping"], selected)
                envelope = capture["detector"]["experimental_envelope"]
                self.assertTrue(envelope["sample_age_limit_enforced"])
                self.assertGreater(envelope["max_oldest_eligible_age_s"], .020)
                self.assertNotIn("experimental", capture["detector"])

    def test_compare_fresh_mapped_hit_retains_receipt_without_xyz_candidate(self):
        for backend in (False, True):
            obj, attempt, selected = prepared(backend=backend, compare=True)
            row = tuple(2. if n == attempt.index else 8. for n in range(3))
            deliver(obj, attempt, [row] * 8)
            self.assertEqual(attempt.reason, "impact")
            obj.disarm()
            detector = obj.stop_capture()["detector"]
            self.assertEqual(detector["first_eligible_sample_matches"], 1)
            self.assertEqual(detector["first_eligible_sample_index"], 0)
            self.assertEqual(detector["signal_filter"]["axis_mapping"], selected)
            self.assertLessEqual(detector["experimental_envelope"]["max_oldest_eligible_age_s"], .020)
            self.assertIsNone(attempt.experimental)


if __name__ == "__main__":
    unittest.main()
