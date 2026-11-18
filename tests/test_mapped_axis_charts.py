"""Physical-axis chart semantics from synthetic configured-stream captures."""
import copy
import json
import tempfile
import unittest
from unittest import mock

from test_accuracy_charts import charts, simulated_noise_window


def mapping(axis="x", source=2, sign=-1):
    return {"version": 1, "axis": axis, "source_axis": source, "sign": sign,
            "coordinate_frame": "configured_sensor_stream", "method": "bidirectional_velocity_v1",
            "confidence": .99, "diagnostics": {"fixture": "SIMULATED"}}


def payload(axis="x", source=2, sign=-1):
    learned = mapping(axis, source, sign)
    record = {"axis": axis, "result": "impact", "impact_metric": "mapped_axis", "axis_mapping": learned,
              "threshold_g": 1.5, "onset_time_s": 2.01, "onset_g": 2., "impact_peak_g": 3.,
              "travel_mm": 150.}
    values = [14., -12., 9.]
    values[source] = -3.
    raw = {"samples": [(2., 1., 2., 3.), (2.01,) + tuple(value + base for value, base in zip(values, (1., 2., 3.)))],
           "baseline_g": (1., 2., 3.), "impact_metric": "mapped_axis", "axis_mapping": copy.deepcopy(learned)}
    trace, metadata = charts._capture_data(raw, record)
    return {"kind": "capture", "axis": axis, "record": record, "capture": metadata, "trace": trace,
            "timestamp": "2026-11-18 12:00:00", "simulated": True}, raw


def figure_mock(count=1):
    figure = mock.Mock()
    figure.get_figheight.return_value = 10.
    axes = [mock.Mock() for _ in range(count)]
    figure.subplots.return_value = axes[0] if count == 1 else axes
    return figure, axes


class MappedAxisChartTests(unittest.TestCase):
    def test_capture_all_permutations_peak_and_threshold_use_physical_component(self):
        for machine in "xy":
            for source in range(3):
                for sign in (-1, 1):
                    with self.subTest(machine=machine, source=source, sign=sign):
                        data, raw = payload(machine, source, sign)
                        figure, (axis,) = figure_mock()
                        charts._plot_capture(figure, data)
                        plots = axis.plot.call_args_list
                        self.assertEqual(len(plots), 4)  # Three source components plus the mapped peak.
                        self.assertEqual(plots[-1].args[1], [3.])
                        self.assertEqual(plots[source].args[1], [0., 3.])
                        self.assertEqual(plots[source].kwargs["label"], charts._mapped_label(mapping(machine, source, sign)))
                        self.assertTrue(all("diagnostic" in call.kwargs["label"] for index, call in enumerate(plots[:3]) if index != source))
                        self.assertIn("Physical " + machine.upper(), axis.axhline.call_args.kwargs["label"])
                        self.assertNotIn("XYZ magnitude", str(axis.plot.call_args_list))

    def test_receipt_keeps_signed_source_rows_and_frozen_mapping(self):
        data, raw = payload()
        original = copy.deepcopy(raw)
        prepared = charts.prepare_capture("mapped", "x", raw, data["record"], simulated=True)
        self.assertEqual(raw, original)
        self.assertEqual(prepared.receipt.trace[-1][1:], (14., -12., -3.))
        metadata = json.loads(prepared.receipt.metadata_json)
        self.assertEqual(metadata["baseline_g"], [1., 2., 3.])
        self.assertEqual(metadata["axis_mapping"], mapping())
        raw["axis_mapping"]["source_axis"] = 0
        self.assertEqual(charts._prepared_capture(prepared)[2]["axis_mapping"], mapping())

    def test_missing_wrong_frame_wrong_axis_and_conflicting_maps_are_rejected(self):
        data, raw = payload()
        for change in ({"source_axis": 3}, {"sign": 0}, {"axis": "y"},
                       {"coordinate_frame": "native_sensor"}, {"confidence": .1}):
            with self.subTest(change=change):
                bad_record = copy.deepcopy(data["record"])
                bad_record["axis_mapping"].update(change)
                with self.assertRaises(ValueError):
                    charts.prepare_capture("bad_map", "x", raw, bad_record)
        for owner in (raw, data["record"]):
            owner.pop("axis_mapping")
        with self.assertRaisesRegex(ValueError, "requires one consistent axis mapping"):
            charts.prepare_capture("missing_map", "x", raw, data["record"])

    def test_accuracy_places_threshold_only_on_mapped_panel(self):
        data, raw = payload("y", 2, -1)
        accuracy = {"kind": "accuracy", "axis": "y", "trials": [data["record"]] * 2,
                    "captures": [data["capture"]] * 2, "traces": [data["trace"]] * 2,
                    "requested": 2, "statistics": {}, "timestamp": data["timestamp"], "simulated": True}
        figure, axes = figure_mock(4)
        charts._plot_accuracy(figure, accuracy)
        self.assertEqual([call.args[0] for call in axes[1].axhline.call_args_list], [0])
        self.assertEqual([call.args[0] for call in axes[2].axhline.call_args_list], [0])
        self.assertEqual([call.args[0] for call in axes[3].axhline.call_args_list], [0, 1.5])
        self.assertIn("Physical Y (= -stream Z)", axes[3].set_ylabel.call_args.args[0])
        self.assertEqual(axes[3].plot.call_args.args[1], [0., 3.])
        changed = copy.deepcopy(accuracy)
        changed["trials"] = [data["record"], dict(data["record"], impact_metric="xyz_magnitude")]
        with self.assertRaises(ValueError):
            charts._plot_accuracy(figure, changed)

    def test_mapped_noise_peak_is_validated_and_vector_peak_is_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            item, raw = simulated_noise_window(directory, "mapped_noise", 1, peak=2.)
            record = item["record"]
            record.update(impact_metric="mapped_axis", axis_mapping=mapping("y", 2, -1),
                          noise_axis_peak_g=record["noise_peak_g"]["z"])
            charts.write_noise_capture(directory, "mapped_noise", 1, raw, record, simulated=True)
            bad = dict(record, noise_axis_peak_g=record["noise_peak_g"]["x"])
            with self.assertRaisesRegex(ValueError, "Mapped-axis noise peak"):
                charts.write_noise_capture(directory, "mapped_noise", 1, raw, bad, simulated=True)
            trace, metadata = charts._capture_data(raw, record)
            record["noise_magnitude_peak_g"] = 2.3
            figure, axes = figure_mock(3)
            charts._plot_noise(figure, {"recommendation": None, "records": [record], "traces": [trace],
                                       "captures": [metadata], "timestamp": "2026-11-18", "simulated": True})
            summary = axes[0].set_title.call_args.args[0]
            self.assertIn("Physical Y noise peak: 0.5 g (calibration metric)", summary)
            self.assertIn("XYZ-magnitude peak: 2.3 g (diagnostic)", summary)

    def test_historical_xyz_capture_retains_vector_metric(self):
        data, raw = payload()
        for owner in (data["record"], raw):
            owner["impact_metric"] = "xyz_magnitude"
            owner.pop("axis_mapping")
        data["trace"], data["capture"] = charts._capture_data(raw, data["record"])
        figure, (axis,) = figure_mock()
        charts._plot_capture(figure, data)
        self.assertEqual(axis.plot.call_args_list[3].kwargs["label"], "XYZ magnitude")
        self.assertIn("XYZ magnitude trigger", axis.axhline.call_args.kwargs["label"])

    def test_real_isolated_worker_can_load_mapping_and_render_completed_png(self):
        data, raw = payload("y", 0, -1)
        image = charts.PNGRenderer(timeout_s=30.).render(data)
        width, height = charts._check_png(image)
        self.assertGreater(width, 500)
        self.assertGreater(height, 400)


if __name__ == "__main__":
    unittest.main()
