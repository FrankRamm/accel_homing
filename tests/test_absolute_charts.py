"""Absolute PNG displays and scoped range provenance; all data is synthetic."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from test_accuracy_charts import RecordingRenderer, charts, simulated_noise_run, simulated_noise_window
from test_beta_capture_charts import beta_event, beta_motion
from harness import make_homing


def range_fields(markers=0, rails=0, scope="capture", policy="beacon_previous_sensor_sign_v1"):
    return {"amplitude_limited": bool(markers or rails),
            "excess_amplitude_unknown": bool(markers or rails),
            "peak_semantics": "bounded_native_endpoint_estimate" if markers else "reported_sensor_derived",
            "signal_scope": scope, "clipped_axis_values": markers, "range_limit_axis_values": rails,
            "clipped_axis_counts": {"x": markers, "y": 0, "z": 0},
            "range_limit_axis_counts": {"x": 0, "y": rails, "z": 0},
            "native_clip_decode": None if not markers else {
                "policy": policy, "marker_axis_values": markers, "scope": scope,
                "marker_axis_values_xyz": {"x": markers, "y": 0, "z": 0},
                "clip_values": [32767, -32768], "count_g": .001,
                "sign_inferred": True, "true_amplitude_unknown": True,
                "first_sample": [1, 0], "last_sample": [1, 1]}}


def figure_mock(count=1):
    figure = mock.Mock()
    figure.get_figheight.return_value = 10.
    axes = [mock.Mock() for _ in range(count)]
    figure.subplots.return_value = axes[0] if count == 1 else axes
    return figure, axes


def example_payloads():
    """The four supported MAIN renderers, with signed offsets and 16 g pulses."""
    clock = {"timestamp": "2026-10-30 12:00:00", "simulated": True}
    raw = {"samples": [(3., .2, -.1, -1.), (3.001, -15.8, 2.9, -5.),
                       (3.002, 2.2, -4.1, 0.)], "baseline_g": (.2, -.1, -1.),
           **range_fields(1, 1)}
    record = {"threshold_g": 1., "direction": -1, "onset_g": -16., "impact_peak_g": 16.,
              "onset_time_s": 3.001, "travel_mm": 175.000125,
              "travel_mm_exact": 175.000244140625, **range_fields(1, 1)}
    trace, captured = charts._capture_data(raw, record)
    capture = {"kind": "capture", "axis": "x", "record": record, "trace": trace,
               "capture": captured, **clock}
    trials = [dict(record), dict(record, travel_mm_exact=175.00048828125)]
    accuracy = {"kind": "accuracy", "axis": "x", "trials": trials, "traces": [trace, trace],
                "captures": [captured, captured], "requested": 2,
                "statistics": charts._accuracy_statistics(trials), **clock}
    noise_records = [{"duration_s": 1., "noise_peak_g": dict(x=16., y=4., z=4.),
                      "noise_rms_g": dict(x=4., y=1., z=1.), **range_fields(0, 1, "noise_window")}
                     for _ in range(3)]
    noise = {"kind": "noise", "records": noise_records, "traces": [trace] * 3,
             "recommendation": None, **clock}
    motions = []
    for offset in (0., 2., 4.):
        item = beta_motion(offset)
        rows = item["capture"]["samples"]
        rows[750] = (rows[750][0], -16., 0., 1.)
        item["capture"].update(range_fields(0, 1, "beta_motion"))
        stats = charts.beta_motion_statistics(item["capture"], item["record"])
        motion_trace, metadata = charts._capture_data(item["capture"], item["record"])
        motions.append({"record": item["record"], "statistics": stats,
                        "trace": motion_trace, "capture": metadata})
    home_record, home_capture = beta_event()
    home_capture["samples"] = [(row[0], -row[1], row[2], row[3]) for row in home_capture["samples"]]
    home_capture["samples"][2] = (.002, -16., 0., 1.)
    home_record["impact_peak_g"] = 16.
    home_record["beta_telemetry"] = make_homing().beta_home_telemetry(home_capture, home_record)
    home_capture.update(range_fields(1, 0))
    beta = {"kind": "beta", "axis": "x", "motions": motions,
            "homes": [charts.compact_beta_home(home_capture, home_record)],
            "title": "Accel Homing Beta X - Suggested Threshold", "proposed_threshold_g": 2., **clock}
    return [capture, accuracy, noise, beta]


class AbsoluteDisplayTests(unittest.TestCase):
    def test_subtract_reference_before_absolute_and_preserve_signed_receipt(self):
        capture = {"samples": [(4., -2., 3., -1.), (4.01, 2., -5., 2.)],
                   "baseline_g": (-1., 1., -2.)}
        original = copy.deepcopy(capture)
        prepared = charts.prepare_capture("signed_reference", "x", capture, {"result": "impact"})
        self.assertEqual(prepared.receipt.trace, ((0., -1., 2., 1.), (.009999999999999787, 3., -6., 4.)))
        self.assertEqual(json.loads(prepared.receipt.metadata_json)["baseline_g"], [-1., 1., -2.])
        axis = mock.Mock()
        charts._traces(axis, prepared.receipt.trace)
        self.assertEqual([call.args[1] for call in axis.plot.call_args_list], [[1., 3.], [2., 6.], [1., 4.]])
        self.assertEqual(capture, original)

    def test_dense_bipolar_envelopes_keep_signed_extrema_and_absolute_troughs(self):
        rows = [(index / 10000., (-1 if index % 2 else 1) * (10. + index % 7),
                 (-1 if index % 3 else 1) * (12. + index % 5),
                 (-1 if index % 5 else 1) * (14. + index % 3)) for index in range(40001)]
        for index, channel, value in ((1234, 1, .001), (1235, 2, -.002), (1236, 3, .003),
                                       (1241, 1, -25.), (1242, 1, 26.), (1243, 2, -27.),
                                       (1244, 2, 28.), (1245, 3, -29.), (1246, 3, 30.)):
            changed = list(rows[index])
            changed[channel] = value
            rows[index] = tuple(changed)
        prepared = charts.prepare_capture("bipolar", "x", {"samples": rows, "baseline_g": (0., 0., 0.)}, {})
        primary = prepared.receipt.trace
        legacy = charts._envelope(primary, 64)
        self.assertLessEqual(len(primary), 4096)
        self.assertLessEqual(len(legacy), 512)
        for reduced in (primary, legacy):
            for channel in (1, 2, 3):
                self.assertEqual(min(row[channel] for row in reduced), min(row[channel] for row in rows))
                self.assertEqual(max(row[channel] for row in reduced), max(row[channel] for row in rows))
                self.assertEqual(min(abs(row[channel]) for row in reduced), min(abs(row[channel]) for row in rows))
        self.assertEqual(primary[0], rows[0])
        self.assertEqual(primary[-1], rows[-1])
        charts._prepared_capture(prepared)

    def test_capture_peak_threshold_and_all_component_curves_are_nonnegative(self):
        payload = example_payloads()[0]
        record = copy.deepcopy(payload["record"])
        figure, (axis,) = figure_mock()
        charts._plot_capture(figure, payload)
        for call in axis.plot.call_args_list:
            self.assertTrue(all(value >= 0 for value in call.args[1]))
        self.assertEqual(axis.plot.call_args_list[-1].args[1], [16.])
        self.assertEqual([call.args[0] for call in axis.axhline.call_args_list], [0, 1.])
        self.assertEqual(axis.set_ylim.call_args.kwargs, {"bottom": 0})
        self.assertEqual(payload["record"], record)

    def test_accuracy_keeps_precise_travel_and_relative_times_while_folding_components(self):
        payload = example_payloads()[1]
        payload["traces"] = [[(-.005, -16., 3., -4.), (0., 2., -4., 1.)]] * 2
        figure, axes = figure_mock(4)
        charts._plot_accuracy(figure, payload)
        self.assertEqual(axes[0].plot.call_args.args[1], [175.000244140625, 175.00048828125])
        axes[0].set_ylim.assert_not_called()
        for axis in axes[1:]:
            self.assertEqual(axis.plot.call_args.args[0], [-.005, 0.])
            self.assertTrue(all(value >= 0 for call in axis.plot.call_args_list for value in call.args[1]))
            self.assertEqual(axis.set_ylim.call_args.kwargs, {"bottom": 0})

    def test_noise_has_common_nonnegative_scale_and_beta_keeps_negative_event_times(self):
        payloads = example_payloads()
        figure, axes = figure_mock(3)
        charts._plot_noise(figure, payloads[2])
        for axis in axes:
            self.assertEqual(axis.set_ylim.call_args.args, (0, 17.6))
            self.assertTrue(all(value >= 0 for call in axis.plot.call_args_list for value in call.args[1]))
        figure, axes = figure_mock(7)
        charts._plot_beta(figure, payloads[3])
        for axis in axes:
            self.assertTrue(all(value is None or value >= 0 for call in axis.plot.call_args_list for value in call.args[1]))
            self.assertEqual(axis.set_ylim.call_args.kwargs, {"bottom": 0})
        self.assertLess(min(axes[4].plot.call_args.args[0]), 0)

    def test_beta_exact_rows_remain_within_new_capture_budget(self):
        trace = [(index / 10000., (-1.) ** index * index, 1., -1.) for index in range(4096)]
        extra = [(5. + index / 1000., -.001 * index, -2., 3.) for index in range(15)]
        reduced = charts._marked_envelope(trace, extra)
        self.assertLessEqual(len(reduced), 4096)
        self.assertTrue(set(extra).issubset(reduced))
        self.assertEqual(min(row[1] for row in reduced), -4095.)
        self.assertEqual(max(row[1] for row in reduced), 4094.)


class RangeProvenanceTests(unittest.TestCase):
    def test_accuracy_counts_every_measured_home_and_excludes_qualifier_and_reference(self):
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory:
            item = {"requested_samples": 2, "complete": True, "trials": [], "capture_paths": [], "rejected": None}
            for index, fields in enumerate((range_fields(9), range_fields(1, 1), range_fields(2, 2))):
                attempt = index + 1
                reference = range_fields(3, 1, "stationary_reference")
                record = {"run_id": "all_homes", "axis": "x", "workflow_attempt": attempt,
                          "workflow_phase": "discovery" if not index else "measurement", "accuracy_sample": index,
                          "result": "impact", "travel_mm": 175. + index / 1000., "threshold_g": 1.,
                          "workflow_threshold_g": 1., "onset_g": -1., "impact_peak_g": 16., "direction": -1,
                          "reference_signal_quality": reference, **fields}
                capture = {"samples": [(1., -.2, .1, 1.), (1.001, -16.2, 1.1, 1.), (1.002, .8, .1, 1.)],
                           "baseline_g": (-.2, .1, 1.), "reference_signal_quality": reference, **fields}
                paths = charts.prepare_capture("all_homes_a%03d" % attempt, "x", capture, record, simulated=True)
                if not index:
                    item["discovery"], item["discovery_capture_paths"] = record, paths
                else:
                    item["trials"].append(record)
                    item["capture_paths"].append(paths)
            charts.write_accuracy(directory, "all_homes", {"x": item}, "complete", simulated=True, renderer=renderer)
        summary = renderer.payloads[-1]["range_summary"]
        self.assertEqual((summary["marker_axis_values"], summary["native_marker_items"]), (3, 2))
        self.assertEqual((summary["range_limit_axis_values"], summary["finite_rail_items"]), (3, 2))
        self.assertEqual((summary["reference_marker_axis_values"], summary["reference_range_limit_axis_values"]), (6, 2))
        self.assertEqual(summary["scope"], "measured_homes_qualifier_excluded")
        lines = " ".join(charts._range_observation_lines(summary))
        self.assertIn("3 axis values in 2/2 homes", lines)
        self.assertIn("counts may overlap capture", lines)

    def test_all_noise_windows_keep_exact_scope_and_separate_reference_uncertainty(self):
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory:
            _, recommendation = simulated_noise_run(directory, "all_windows", peaks=(.2, .2, .2))
            iterations = []
            for index in range(1, 4):
                item, capture = simulated_noise_window(directory, "all_windows", index, peak=.2)
                fields = range_fields(index, index - 1, "noise_window")
                item["record"].update(fields)
                capture.update(fields)
                item["capture_paths"] = charts.write_noise_capture(directory, "all_windows", index, capture,
                                                                    item["record"], simulated=True)
                iterations.append(item)
            charts.prepare_noise("all_windows", iterations, recommendation, "complete", simulated=True, renderer=renderer)
        payload = renderer.payloads[-1]
        self.assertEqual(payload["range_summary"]["marker_axis_values"], 6)
        self.assertEqual(payload["range_summary"]["range_limit_axis_values"], 3)
        self.assertTrue(all(item["native_clip_decode"]["scope"] == "noise_window" for item in payload["captures"]))
        self.assertTrue(all(item["baseline_g"][1] > 0 for item in payload["captures"]))

    def test_zero_marker_capture_still_reports_limited_reference_and_rail_only_beta(self):
        clean = range_fields()
        clean["reference_signal_quality"] = range_fields(2, 0, "stationary_reference")
        summary = charts._range_summary([({}, clean)], "home_capture", "captures")
        self.assertEqual(summary["marker_axis_values"], 0)
        self.assertEqual(summary["reference_marker_axis_values"], 2)
        self.assertIn("baseline/true peak uncertain", " ".join(charts._range_observation_lines(summary)))
        beta = example_payloads()[3]
        figure, axes = figure_mock(7)
        charts._plot_beta(figure, beta)
        for axis in axes[:3]:
            self.assertIn("Finite sensor rails", axis.set_title.call_args.args[0])
            self.assertIn("excess amplitude unknown", axis.set_title.call_args.args[0])
        self.assertIn("3 axis values in 3/3 motions", figure.text.call_args.args[2])

    def test_capture_metadata_takes_priority_and_mixed_legacy_policies_are_explicit(self):
        record = {**range_fields(99), "reference_signal_quality": range_fields(1, 0, "stationary_reference")}
        exact = range_fields(2, 1)
        captured = charts._range_metadata(exact, record)
        self.assertEqual(captured["native_clip_decode"]["marker_axis_values"], 2)
        self.assertEqual(captured["reference_signal_quality"]["native_clip_decode"]["marker_axis_values"], 1)
        summary = charts._range_summary([(record, captured), ({}, range_fields(3, policy="legacy_seed"))], "homes", "homes")
        self.assertEqual(summary["marker_axis_values"], 5)
        self.assertIn("mixed/legacy decode conventions", " ".join(charts._range_observation_lines(summary)))
        old_seed = range_fields(3)
        old_seed["native_clip_decode"]["armed_seed_convention"] = True
        summary = charts._range_summary([({}, captured), ({}, old_seed)], "homes", "homes")
        self.assertIn("mixed/legacy decode conventions", " ".join(charts._range_observation_lines(summary)))
        legacy = charts._range_summary([({}, {})], "homes", "homes")
        self.assertEqual(legacy["unknown_items"], 1)
        self.assertIn("provenance unavailable", " ".join(charts._range_observation_lines(legacy)))

    def test_beta_rail_only_home_is_counted_and_annotated_without_native_marker(self):
        home = example_payloads()[3]["homes"][0]
        home["capture"].update(range_fields(0, 2))
        renderer = RecordingRenderer()
        with tempfile.TemporaryDirectory() as directory:
            charts.write_beta_motion(directory, "beta_event", "x", [], homes=[home],
                                     result="beta_proposed", proposed_threshold_g=2.,
                                     simulated=True, renderer=renderer)
        payload = renderer.payloads[-1]
        summary = payload["home_range_summary"]
        self.assertEqual((summary["total_items"], summary["finite_rail_items"]), (1, 1))
        self.assertEqual(summary["range_limit_axis_values"], 2)
        self.assertEqual((summary["marker_axis_values"], summary["native_marker_items"]), (0, 0))
        figure, axes = figure_mock(7)
        charts._plot_beta(figure, payload)
        events = axes[3].set_title.call_args.args[0]
        self.assertIn("Finite sensor rails: 2 axis values in 1/1 homes", events)
        self.assertIn("excess amplitude unknown", events)
        self.assertNotIn("Native markers:", events)


class RealAbsoluteRendererTests(unittest.TestCase):
    def test_real_axes_include_full_peaks_and_pngs_render_for_all_main_types(self):
        payloads = example_payloads()
        script = """import json, sys
sys.path.insert(0, sys.argv[1])
from matplotlib.figure import Figure
import accel_homing_charts as charts
results = []
for payload in json.load(sys.stdin):
    figure = Figure(figsize=(12, 17))
    getattr(charts, '_plot_' + payload['kind'])(figure, payload)
    axes = figure.axes[1:] if payload['kind'] == 'accuracy' else figure.axes
    checked = []
    for axis in axes:
        values = [float(value) for line in axis.lines if line.get_transform() == axis.transData
                  for value in line.get_ydata()
                  if value is not None and float(value) == float(value)]
        checked.append([axis.get_ylim()[0], axis.get_ylim()[1], min(values), max(values)])
    results.append(checked)
json.dump(results, sys.stdout)
"""
        result = subprocess.run([sys.executable, "-B", "-I", "-c", script, str(Path(charts.__file__).parent)],
                                input=json.dumps(payloads), text=True, capture_output=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        for kind, axes in zip(("capture", "accuracy", "noise", "beta"), json.loads(result.stdout)):
            for bottom, top, minimum, peak in axes:
                with self.subTest(kind=kind, peak=peak):
                    self.assertEqual(bottom, 0.)
                    self.assertGreaterEqual(minimum, 0.)
                    self.assertGreater(top, peak)
            self.assertTrue(any(axis[3] >= 16. for axis in axes))
        renderer = charts.PNGRenderer(timeout_s=30.)
        for payload in payloads:
            with self.subTest(kind=payload["kind"]):
                width, height = charts._check_png(renderer.render(payload))
                self.assertGreater(width, 100)
                self.assertGreater(height, 100)


if __name__ == "__main__":
    unittest.main()
