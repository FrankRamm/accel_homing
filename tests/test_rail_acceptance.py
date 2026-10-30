"""Finite range readings are observed data, not automatic impacts. Synthetic only."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import accel_homing_charts as charts
import accel_homing_filters as filters
import test_filter_engine as engine_tests
import test_chronological_clipping as native
import test_backends as backends
from harness import FakeEndstop, acquire_reference, arm_axis, batch, counts, make_homing, plugin, stationary_stream
from test_accuracy_charts import RecordingRenderer
from test_noise_recovery import sensor
from test_retract_noise import noise_flow
import test_beta_capture_charts as beta_tests


def endpoint_capture(value=2047, position=0, version=2, sentinel=None, time_offset=0.):
    rows = [(counts(3.), 0, counts(1.))] * 8
    rows[position] = (value, 0, counts(1.))
    return native.native_filtered_capture(clip_position=sentinel, row_values=rows, time_offset=time_offset,
        filter_config={"version": version}, clock_hz=32000000., sample_period=.000302)


class EndpointRuntimeTests(unittest.TestCase):
    def machine(self, version):
        obj = make_homing(threshold=1.)
        attempt, _sent = arm_axis(obj, "x", 1000, 2000)
        attempt.vector_mode, attempt.detector_type = True, "filtered"
        attempt.bases = (0., 0., 128.)
        attempt.filter_engine = filters.CausalImpactDetector(1., (0., 0., 1.), config={"version": version})
        return obj, attempt

    def packet(self, obj, start, rows):
        obj.reactor.now = start / 1000. + (len(rows) - 1) * .001 + .001
        obj._inspect({"start_clock": start, "delta_clock": len(rows) - 1,
                      "data": tuple(v for row in rows for v in row), "#receive_time": obj.reactor.now})

    def test_both_signed_endpoints_pass_actual_v1_v2_detection_and_same_source_replay(self):
        for version in (1, 2):
            for value in (2047, -2047, -2048):
                with self.subTest(version=version, value=value):
                    capture, record = endpoint_capture(value, version=version)
                    self.assertEqual(record["result"], "impact")
                    self.assertIsNone(record["sample_fault"])
                    self.assertFalse(record["response_censored"])
                    self.assertIsInstance(record["impact_peak_g"], float)
                    self.assertTrue(record["amplitude_limited"])
                    self.assertTrue(capture["excess_amplitude_unknown"])
                    self.assertTrue(capture["raw_integrity"]["raw_samples_validated"])
                    self.assertEqual(capture["raw_integrity"]["range_limit_axis_values"], 1)
                    self.assertNotIn("response_censoring", capture)
                    first = capture["detector"]["first_armed_sample_index"]
                    self.assertEqual(capture["samples"][first][1], value * .0078125)
                    analysis = filters.compare_filter_capture(capture, 1., 1.)
                    self.assertTrue(analysis["filtered_decision"]["decision_observed"])

    def test_finite_endpoint_after_confirmation_keeps_entire_response_and_numeric_peak(self):
        for value in (2047, -2048):
            capture, record = endpoint_capture(value, position=4)
            self.assertEqual(record["result"], "impact")
            self.assertFalse(record["response_censored"])
            self.assertGreater(record["impact_peak_g"], 15.)
            self.assertEqual(len(capture["samples"]), sum(
                len(source["data"]) // 6 - source["first_sample_position"]
                for source in capture["_native_source_batches"]))

    def test_rail_is_not_automatic_impact_and_support_guard_still_applies(self):
        for version in (1, 2):
            rows = [(-2048, 0, counts(1.))] + [(0, 0, counts(1.))] * 31
            capture, record = native.native_filtered_capture(clip_position=None, packet_size=32, row_values=rows,
                filter_config={"version": version}, clock_hz=32000000., sample_period=.000302)
            self.assertEqual(record["result"], "no_trigger")
            self.assertIsNone(record["sample_fault"])
            self.assertIsNone(record["confirmation_time_s"])
            self.assertGreater(record["filter_diagnostics"]["rejected_candidates"], 0)
            self.assertTrue(capture["raw_integrity"]["raw_samples_validated"])

    def test_finite_endpoint_before_later_sentinel_remains_in_verified_prefix(self):
        for version in (1, 2):
            capture, record = endpoint_capture(-2048, version=version, sentinel=4)
            summary = filters.validate_filtered_capture_prefix(capture, record)
            self.assertEqual(summary["cause"], "native_clip_sentinel")
            self.assertEqual(summary["first_invalid_sample"][2], 4)
            self.assertEqual(summary["raw_value"], 32767)
            self.assertTrue(record["response_censored"])
            self.assertIsNone(record["impact_peak_g"])
            self.assertEqual(capture["raw_integrity"]["range_limit_axis_values"], 1)
            self.assertGreater(summary["observed_valid_prefix_peak_g"], 15.)

    def test_accepted_rail_does_not_hide_subsequent_preconfirmation_sentinel_or_nan(self):
        for version in (1, 2):
            for bad, cause in ((32767, "native_clip_sentinel"), (float("nan"), "nonfinite_axis_value")):
                obj, attempt = self.machine(version)
                low = tuple(attempt.bases)
                high = (-2048, low[1], low[2])
                invalid = (low[0], bad, low[2])
                self.packet(obj, 1001, [high, invalid] + [low] * 6)
                self.assertEqual(attempt.reason, "invalid_data")
                self.assertEqual(attempt.sample_fault["cause"], cause)
                self.assertEqual(attempt.sample_fault["sample_position"], 1)
                self.assertIsNone(attempt.confirmation_time)
                self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)

    def test_nonfinite_later_tail_still_fails_retained_prefix_validation(self):
        capture, record = endpoint_capture(-2048, sentinel=4)
        last = capture["_native_source_batches"][-1]
        raw = [v for row in plugin.SAMPLE.iter_unpack(last["data"]) for v in row]
        raw[-2] = float("nan")
        last["data"] = tuple(raw)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            filters.validate_filtered_capture_prefix(capture, record)

    def test_earlier_endpoint_packet_and_no_capture_keep_monotone_status_flag(self):
        for version in (1, 2):
            obj, attempt = self.machine(version)
            low = tuple(attempt.bases)
            self.packet(obj, 1001, [(-2048, low[1], low[2])] + [low] * 15)
            self.assertFalse(attempt.done)
            high = (low[0], low[1] + 3. / attempt.count_g, low[2])
            self.packet(obj, 1017, [high] * 8)
            self.assertEqual(attempt.reason, "impact")
            obj.disarm()
            self.assertTrue(obj.get_status(obj.reactor.now)["last"]["x"]["amplitude_limited"])
            self.assertIsNone(obj.capture_batches)

    def test_raw_vector_and_legacy_axis_preserve_finite_endpoint_and_sentinel_policy(self):
        for vector in (False, True):
            for value in (2047, -2048, 32767):
                obj = make_homing(threshold=1.)
                attempt, sent = arm_axis(obj, "x", 1000, 2000)
                attempt.vector_mode = vector
                attempt.direction = 0
                attempt.bases, attempt.baseline = (0., 0., 128.), 0.
                obj.reactor.now = 1.008
                obj._inspect(batch(1001, [(value, 0, 128)] * 8))
                self.assertEqual(attempt.reason, "impact")  # legacy RAW sentinel policy is preserved
                self.assertIsNone(attempt.sample_fault)
                obj.disarm()
                self.assertEqual(attempt.record["amplitude_limited"], value != 32767)
                self.assertEqual(len(sent.messages), 1)

    def test_configured_sensor_endpoints_enter_both_filtered_versions(self):
        for version in (1, 2):
            for value in (16., -16.):
                obj, attempt = engine_tests.RuntimeTests().filtered()
                attempt.filter_engine = filters.CausalImpactDetector(1., tuple(v * attempt.count_g for v in attempt.bases),
                                                                    config={"version": version})
                high = (value / attempt.count_g, attempt.bases[1], attempt.bases[2])
                engine_tests.RuntimeTests().packet(obj, attempt, attempt.arm_time + .0001, [high] * 8)
                self.assertEqual(attempt.reason, "impact")
                self.assertIsNone(attempt.sample_fault)
                self.assertTrue(attempt.amplitude_limited)

    def test_all_original_sensor_axes_accept_both_endpoint_polarities(self):
        for version in (1, 2):
            for index in range(3):
                for value in (2047, -2048):
                    obj, attempt = self.machine(version)
                    high = list(attempt.bases)
                    high[index] = value
                    self.packet(obj, 1001, [tuple(high)] * 8)
                    self.assertEqual(attempt.reason, "impact")
                    self.assertIsNone(attempt.sample_fault)
                    self.assertTrue(attempt.amplitude_limited)

    def test_native_filtered_range_scan_never_looks_ahead_before_dispatch(self):
        obj, attempt = self.machine(2)
        original = obj._note_range_observation
        def note(current, raw):
            raw = tuple(raw)
            if len(raw) > 3:
                self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)
            return original(current, raw)
        with mock.patch.object(obj, "_note_range_observation", side_effect=note):
            self.packet(obj, 1001, [(counts(3.), 0, 128)] * 4 + [(-2048, 0, 128)] * 4)
        self.assertEqual(attempt.reason, "impact")

    def test_frozen_range_stream_still_stops_under_existing_watchdog(self):
        obj, attempt = self.machine(2)
        attempt.limit_raw = 12. / attempt.count_g
        attempt.filter_engine = filters.CausalImpactDetector(12., (0., 0., 1.))
        attempt.filter_engine.weights_xyz = (.75, .75, .75)
        # Baseline is the observed endpoint: no change means no impact.
        attempt.bases = (-2048., 0., 128.)
        attempt.filter_engine.offset_g = tuple(v * attempt.count_g for v in attempt.bases)
        for offset in (0, 8, 16, 24, 32):
            self.packet(obj, 1001 + offset, [attempt.bases] * 8)
        obj._commanded_speed = lambda *_: 100.
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "frozen")


class EndpointAcquisitionTests(unittest.TestCase):
    def test_native_and_generic_quiet_reference_can_contain_range_codes(self):
        for generic in (False, True):
            for value in (16., -16.):
                if generic:
                    obj, endstop = backends.GenericPreparationTest().session()
                else:
                    obj = make_homing()
                    endstop = FakeEndstop("x")
                    obj.session_axes = {"x": native.FakeRail(endstop, native.info(speed=100., retract=0.))}
                obj.reactor.now = 10.
                stationary_stream(obj, (value, 0., 1.))
                obj._prepare_reference()
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNotNone(obj.preparation.baseline())
                endstop.get_steppers = lambda: []
                attempt = obj.prepare(endstop, obj.preparation.last + .001)
                self.assertTrue(attempt.amplitude_limited)

    def test_native_and_generic_noise_accept_finite_range_but_keep_frozen_guard(self):
        for generic in (False, True):
            obj = sensor(generic)
            original = obj._inspect if not generic else obj.backend.helper.emit
            def incoming(params):
                params = dict(params)
                if generic:
                    params["data"] = [(row[0], -16. * backends.G, row[2], row[3]) for row in params["data"]]
                else:
                    rows = list(plugin.SAMPLE.iter_unpack(params["data"]))
                    params["data"] = b"".join(plugin.SAMPLE.pack(-2048, row[1], row[2]) for row in rows)
                original(params)
            if generic:
                obj.backend.helper.emit = incoming
            else:
                obj._inspect = incoming
            noise_flow(obj)
            obj.start_noise_measurement()
            result = obj.collect_noise_window(lambda: None)
            self.assertTrue(result["samples"])
            self.assertIsNone(obj.sensor_fault)
            session = {"context": {"sensor": result["_noise_context"]}, "iterations": [], "run_id": "range_noise"}
            record = obj.workflows._noise_record(session, 1, result)
            self.assertTrue(record["amplitude_limited"])
            self.assertTrue(record["excess_amplitude_unknown"])
            obj.stop_noise_measurement()

    def test_beta_acquisition_keeps_finite_endpoints_but_still_rejects_sentinel(self):
        for value in (2047, -2048, 32767):
            obj = beta_tests.BetaRawCaptureTests().owner()
            obj._capture_batch(batch(8, [(value, 0, counts(1.)), (0, 0, counts(1.))], delta_clock=1))
            capture = obj.stop_capture()
            self.assertEqual(capture["fault"], "invalid_data" if value == 32767 else None)
            if value != 32767:
                self.assertTrue(capture["raw_integrity"]["raw_samples_validated"])
                self.assertTrue(capture["amplitude_limited"])
                self.assertEqual(capture["samples"][0][1], value * .0078125)


class EndpointWorkflowRig(native.CensoredWorkflowRig):
    def stop_capture(self):
        existing = super(native.CensoredWorkflowRig, self).stop_capture()
        source = existing.get("_record")
        if source is None:
            return existing
        captured, record = endpoint_capture(-2048, time_offset=self.capture_stops * 10.)
        protected = {key: source[key] for key in ("start_position_mm", "expected_travel_mm") if key in source}
        source.update(record)
        source.update(protected)
        captured["_record"] = source
        return captured


class EndpointWorkflowChartTests(unittest.TestCase):
    def test_successful_calibration_keeps_margin_rules_and_observed_peak_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = EndpointWorkflowRig({"charts_directory": directory})
            rig.calibrate(save=0, DETECTOR="FILTERED")
            self.assertEqual(rig.workflow.last_result["result"], "calibrated")
            metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            self.assertEqual(metadata["amplitude_limited"], [True] * 10)
            self.assertTrue(all(value > 15 for value in metadata["impact_peak_g"]))
            self.assertEqual(metadata["response_censored"], [False] * 10)
            self.assertEqual(rig.workflow.last_result["saved_threshold_g"], 1.5)

    def test_accuracy_and_deferred_receipt_retain_informational_limit_without_abort(self):
        capture, record = endpoint_capture(-2048, position=4)
        prepared = charts.prepare_capture("endpoint", "x", capture, record, simulated=True)
        capture.clear()
        record.clear()
        _receipt, frozen, metadata = charts._prepared_capture(prepared)
        self.assertTrue(frozen["amplitude_limited"])
        self.assertTrue(metadata["excess_amplitude_unknown"])
        self.assertNotIn("response_censoring", metadata)
        with tempfile.TemporaryDirectory() as directory:
            renderer = RecordingRenderer()
            charts.publish_captures(directory, [prepared], renderer=renderer)
            rig = EndpointWorkflowRig({"charts_directory": directory})
            rig.accuracy(DETECTOR="FILTERED", FILTERED_THRESHOLD=1.)
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertTrue(all(r["impact_peak_g"] > 15 and not r["response_censored"]
                                for r in result["axis_results"]["x"]["trials"]))

    def test_filter_chart_keeps_signed_range_rows_both_replays_and_uncertainty(self):
        capture, record = endpoint_capture(-2048)
        prepared = charts.prepare_filter_capture("endpoint", "x", capture, record, 1., 1., simulated=True)
        payload = json.loads(prepared.receipt.payload_json)
        self.assertTrue(payload["analysis"]["amplitude_limited"])
        self.assertEqual(set(payload["analysis"]["variants"]), {"signed_v1", "rectified_v2"})
        self.assertTrue(any(row[1] < -15. for row in payload["analysis"]["trace"]))
        self.assertIn("true peak may be higher", charts._range_observation_lines(payload["analysis"])[0])

    def test_endpoint_then_unrelated_invalid_tail_still_aborts_without_chart_or_model(self):
        class InvalidTailRig(EndpointWorkflowRig):
            def stop_capture(self):
                capture = super().stop_capture()
                if capture.get("detector"):
                    capture["fault"] = "invalid_data"
                return capture
        with tempfile.TemporaryDirectory() as directory:
            rig = InvalidTailRig({"charts_directory": directory})
            with self.assertRaises(Exception):
                rig.calibrate(save=0, DETECTOR="FILTERED")
            self.assertEqual(list(Path(directory).rglob("*.png")), [])
            self.assertEqual(rig.configfile.saved, {})


if __name__ == "__main__":
    unittest.main()
