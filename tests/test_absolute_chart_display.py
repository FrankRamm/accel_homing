"""Absolute chart presentation preserves signed inputs, timings and statistics."""
import copy
import json
import math
import random
import tempfile
import unittest
from unittest import mock

import accel_homing_charts as charts
from test_accuracy_charts import RecordingRenderer, png_bytes, simulated_noise_run
from test_beta_capture_charts import beta_event, beta_motion
from test_censored_response_charts import synthetic_public_censored_report
from test_rectified_filter_charts import synthetic_alternating_filter_home
from harness import make_homing

try:
    from matplotlib.figure import Figure
except ImportError:
    Figure = None


def clock(payload):
    return {**payload, **charts._clock(), 'simulated': True}


class AbsoluteDisplaySelectionTests(unittest.TestCase):
    def test_dense_bipolar_envelope_keeps_signed_peaks_zero_troughs_and_compact_bounds(self):
        samples = [(i / 10000., (-1 if i % 2 else 1) * (10. if i % 17 else 0.),
                    (-1 if i % 3 else 1) * (8. if i % 19 else .001),
                    (-1 if i % 5 else 1) * (7. if i % 23 else .002)) for i in range(20001)]
        saved = copy.deepcopy(samples)
        for buckets, limit in ((512, 4096), (64, 512)):
            selected = charts._envelope(samples, buckets)
            self.assertLessEqual(len(selected), limit)
            self.assertEqual(selected[0], samples[0])
            self.assertEqual(selected[-1], samples[-1])
            self.assertEqual(list(selected), sorted(set(selected)))
            for channel in (1, 2, 3):
                self.assertEqual(min(row[channel] for row in selected), min(row[channel] for row in samples))
                self.assertEqual(max(row[channel] for row in selected), max(row[channel] for row in samples))
                self.assertEqual(min(abs(row[channel]) for row in selected), min(abs(row[channel]) for row in samples))
            self.assertTrue(any(row[1] < 0 for row in selected))
        self.assertEqual(samples, saved)

    def test_joint_filter_selection_is_bounded_keeps_absolute_troughs_and_every_event(self):
        rng = random.Random(42)
        count = 16000
        raw = [(i * .0001, *(rng.uniform(-10, 10) for _ in range(3))) for i in range(count)]
        filtered = [(row[0], *(rng.uniform(-8, 8) for _ in range(3))) for row in raw]
        rectified = [(row[0], *(rng.uniform(0, 8) for _ in range(3))) for row in raw]
        times = [raw[index][0] for index in (211, 212, 213, 721, 722, 723, 912)]
        variants = {'signed_v1': {'decision': dict(zip(('onset_time_s', 'crossing_time_s', 'confirmation_time_s'), times[:3]))},
            'rectified_v2': {'trace': rectified, 'level': [3.] * count, 'weighted_level': [2.] * count,
                'decision': dict(zip(('onset_time_s', 'crossing_time_s', 'confirmation_time_s'), times[3:6]))}}
        analysis = {'raw_trace': raw, 'filtered_trace': filtered, 'raw_level': [4.] * count,
                    'filtered_level': [3.] * count, 'weighted_level': [2.] * count,
                    'raw_decision': {'crossing_time_s': times[-1]}, 'variants': variants}
        saved = copy.deepcopy(analysis)
        with mock.patch.object(charts, '_filter_variants', return_value=(variants, 'signed_v1')):
            rows = charts._filter_display_rows(analysis)
        self.assertLessEqual(len(rows), 4096)
        self.assertEqual([row[0] for row in rows], sorted(set(row[0] for row in rows)))
        selected = {row[0] for row in rows}
        self.assertTrue(set(times) <= selected)
        self.assertEqual(rows[0][0], raw[0][0])
        self.assertEqual(rows[-1][0], raw[-1][0])
        stride = math.ceil(count / ((4096 - 9) // 39))
        for start in range(0, count, stride):
            for source in (raw, filtered, rectified):
                for channel in (1, 2, 3):
                    minimum = min(source[start:start + stride], key=lambda row: abs(row[channel]))
                    self.assertIn(minimum[0], selected)
        self.assertTrue(any(row[1] < 0 for row in rows))
        self.assertEqual(analysis, saved)

    def test_difference_labels_preserve_direction_without_negative_magnitude(self):
        self.assertEqual(charts._display_difference(-.25, 'mm'), '0.25 mm (decrease)')
        self.assertEqual(charts._display_difference(.25, 'mm'), '0.25 mm (increase)')
        self.assertEqual(charts._display_difference(-.4, 'ms', 'later', 'earlier'), '0.4 ms (earlier)')
        self.assertEqual(charts._display_difference(0., 'mm'), '0 mm (unchanged)')

    def test_historical_accuracy_receipt_bytes_and_statistics_remain_compatible(self):
        for version in (1, 2):
            report = synthetic_public_censored_report(filter_version=version)
            original = copy.deepcopy(report)
            image = charts._png_with_accuracy_report(png_bytes(), report)
            loaded = charts._accuracy_report_from_png(image)
            self.assertEqual(loaded, original)
            self.assertEqual(report, original)
            self.assertEqual(charts._png_with_accuracy_report(png_bytes(), loaded), image)


@unittest.skipIf(Figure is None, 'matplotlib unavailable in this test runtime')
class AbsoluteFigureTests(unittest.TestCase):
    def check_amplitude_axes(self, figure):
        found = 0
        for axis in figure.axes:
            if not getattr(axis, '_absolute_acceleration_display', False):
                continue
            found += 1
            self.assertEqual(axis.get_ylim()[0], 0.)
            for line in axis.lines:
                values = [value for value in line.get_ydata() if value is not None and math.isfinite(value)]
                self.assertTrue(all(value >= 0 for value in values), (line.get_label(), values))
        self.assertGreater(found, 0)

    def test_capture_components_negative_peak_marker_and_threshold_use_absolute_display(self):
        trace = ((0., -3., 2., -1.), (.001, 4., -5., 0.), (.002, -6., 1., 2.))
        record = {'axis': 'x', 'onset_time_s': 1.001, 'onset_g': -1.25,
                  'impact_peak_g': 6., 'direction': -1, 'threshold_g': 2., 'travel_mm': 175.}
        payload = clock({'axis': 'x', 'trace': trace, 'record': record,
                         'capture': {'first_sample_time_s': 1., 'sample_count': 3}})
        original = copy.deepcopy(payload)
        figure = Figure()
        charts._plot_capture(figure, payload)
        axis = figure.axes[0]
        for channel, label in enumerate(('|X|', '|Y|', '|Z|'), 1):
            line = next(line for line in axis.lines if line.get_label() == label)
            self.assertEqual(list(line.get_ydata()), [abs(row[channel]) for row in trace])
        self.assertTrue(any(line.get_marker() == 'o' and list(line.get_ydata()) == [6.] for line in axis.lines))
        self.assertTrue(any('negative only' in line.get_label() and list(line.get_ydata()) == [2., 2.] for line in axis.lines))
        self.check_amplitude_axes(figure)
        self.assertNotIn('-1.25', axis.get_title(loc='left'))
        self.assertEqual(payload, original)

    def test_accuracy_overlay_and_censored_end_markers_are_positive_without_flattening_travel_zoom(self):
        trace = ((0., -3., 2., -1.), (.001, 4., -5., 0.), (.002, -6., 1., -2.))
        trials = [{'travel_mm': value, 'threshold_g': 2., 'direction': -1,
                   'response_censoring': {'display_only_fixture': True}} for value in (174.98, 175.02)]
        payload = clock({'axis': 'x', 'trials': trials, 'traces': [trace, trace], 'requested': 2,
                         'statistics': charts._accuracy_statistics(trials)})
        original = copy.deepcopy(payload)
        figure = Figure()
        charts._plot_accuracy(figure, payload)
        self.check_amplitude_axes(figure)
        self.assertGreater(figure.axes[0].get_ylim()[0], 170.)
        marks = [line for axis in figure.axes for line in axis.lines if line.get_marker() == 'x']
        self.assertTrue(marks)
        self.assertTrue(all(all(value >= 0 for value in line.get_ydata()) for line in marks))
        self.assertEqual(payload, original)

    def test_zero_variance_actual_ab_panel_has_no_negative_limits_and_keeps_travel_zoom(self):
        trace = ((0., -1., 0., 0.), (.001, 1., 0., 0.))
        trials = [{'travel_mm': 175., 'threshold_g': 2., 'direction': -1}] * 2
        statistics = charts._accuracy_statistics(trials)
        report = {'run_id': 'current', 'context': {'detector_type': 'raw'}, 'threshold_g': 2.,
                  'count': 2, 'statistics': statistics, 'travels_mm': [175., 175.], 'censored_response_count': 0}
        baseline = dict(report, run_id='baseline')
        comparison = {'spread_change_mm': 0., 'stddev_change_mm': 0., 'comparable_context': True,
                      'treatment_differences': []}
        payload = clock({'axis': 'x', 'trials': trials, 'traces': [trace, trace], 'requested': 2,
                         'statistics': statistics, 'report': report, 'baseline': baseline, 'comparison': comparison})
        original = copy.deepcopy(payload)
        figure = Figure()
        charts._plot_accuracy(figure, payload)
        self.assertEqual(figure.axes[1].get_ylim()[0], 0.)
        self.assertGreater(figure.axes[0].get_ylim()[0], 150.)  # unchanged Matplotlib zoom for a constant175mm series
        self.assertIn('unchanged', figure.axes[1].get_title(loc='left'))
        self.check_amplitude_axes(figure)
        self.assertEqual(payload, original)

    def test_filter_signed_v1_magnitudes_keep_negative_time_and_original_signed_psd(self):
        import numpy as np
        capture, record = synthetic_alternating_filter_home()
        prepared = charts.prepare_filter_capture('absolute_filter', 'x', capture, record, .8, .8, simulated=True)
        payload = json.loads(prepared.receipt.payload_json)
        payload.update(charts._clock())
        original_json = prepared.receipt.payload_json
        original = copy.deepcopy(payload)
        figure = Figure()
        charts._plot_filter(figure, payload)
        self.check_amplitude_axes(figure)
        analysis = payload['analysis']
        line = next(line for axis in figure.axes for line in axis.lines if line.get_label() == '|v1 signed output X|')
        column = analysis['trace_columns']['signed_v1']['xyz'][0]
        self.assertEqual(list(line.get_ydata()), [abs(row[column]) for row in analysis['trace']])
        self.assertLess(min(line.get_xdata()), 0.)
        spectrum = analysis['noise']['spectrum']
        self.assertTrue(spectrum['available'])
        axis = next(axis for axis in figure.axes if axis.get_ylabel() == 'Summed XYZ PSD (g²/Hz)')
        self.assertEqual(axis.get_yscale(), 'log')
        self.assertGreater(axis.get_ylim()[0], 0.)
        sources = [('Raw signed XYZ', 'raw_xyz_g')] + [
            ('v1 signed' if key == 'signed_v1' else 'v2 rectified envelope', series)
            for key, series in spectrum['variant_series'].items()]
        for label, series in sources:
            values = np.asarray(spectrum[series], dtype=float)
            window = np.hanning(spectrum['sample_count'])
            transformed = np.fft.rfft((values - values.mean(axis=0)) * window[:, None], axis=0)
            expected = (np.abs(transformed) ** 2).sum(axis=1) / (spectrum['sample_rate_hz'] * (window ** 2).sum())
            expected[1:-1 if spectrum['sample_count'] % 2 == 0 else None] *= 2.
            rendered = next(line for line in axis.lines if line.get_label() == label)
            np.testing.assert_allclose(rendered.get_ydata(), np.maximum(expected[1:], 1e-18), rtol=1e-14)
        self.assertEqual(payload, original)
        self.assertEqual(prepared.receipt.payload_json, original_json)

    def test_beta_motion_and_home_display_are_positive_while_event_times_stay_signed(self):
        motions = [beta_motion(offset) for offset in (0., 2., 4.)]
        for motion in motions:
            motion['capture']['samples'] = [(row[0], -row[1], row[2], row[3]) for row in motion['capture']['samples']]
        record, capture = beta_event()
        capture['samples'] = [(row[0], -row[1], row[2], row[3]) for row in capture['samples']]
        record['beta_telemetry'] = make_homing().beta_home_telemetry(capture, record)
        compact = charts.compact_beta_home(capture, record)
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory:
            charts.write_beta_motion(directory, 'beta_event', 'x', motions, homes=[compact],
                                    result='beta_proposed', proposed_threshold_g=3.25, simulated=True, renderer=renderer)
        payload = renderer.payloads[0]
        original = copy.deepcopy(payload)
        figure = Figure()
        charts._plot_beta(figure, payload)
        self.check_amplitude_axes(figure)
        self.assertTrue(any(min(line.get_xdata()) < 0 for axis in figure.axes[4:] for line in axis.lines if line.get_label() == 'Last beta home'))
        self.assertEqual(payload, original)

    def test_noise_display_is_positive_but_existing_centered_metrics_and_receipts_are_unchanged(self):
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory:
            iterations, recommendation = simulated_noise_run(directory, 'absolute_noise')
            records_before = copy.deepcopy([item['record'] for item in iterations])
            charts.write_noise(directory, 'absolute_noise', iterations, recommendation, 'complete', simulated=True, renderer=renderer)
        payload = renderer.payloads[0]
        original = copy.deepcopy(payload)
        self.assertTrue(any(value < 0 for trace in payload['traces'] for row in trace for value in row[1:]))
        figure = Figure()
        charts._plot_noise(figure, payload)
        self.check_amplitude_axes(figure)
        self.assertEqual(payload, original)
        self.assertEqual(payload['records'], records_before)
        self.assertTrue(any('original centered samples' in text.get_text() for text in figure.texts))


if __name__ == '__main__':
    unittest.main()
