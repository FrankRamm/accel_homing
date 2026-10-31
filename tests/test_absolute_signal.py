"""Absolute centered RAW decisions and honest range observations; offline only."""
import json
import types
import unittest
from unittest import mock

from harness import (CommandError, FakeConfig, FakeEndstop, arm_axis, batch, counts,
                     make_homing, plugin, stationary_stream)
import test_backends as backends
from test_session import FakeRail, info, with_endstops
from test_trigger import FakeGcmd
from accel_homing_model import AccelHomingModel
import test_retract_noise as noise_tests
import test_beta_capture_charts as beta_tests
from test_workflows import GCmd, Rig

HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT


def native_baseline(obj, values=(32, -64, 128)):
    for first in range(0, 96, 8):
        obj._inspect(batch(first, [values] * 8))


class AbsoluteCenteredTests(unittest.TestCase):
    def test_native_signs_and_permutations_keep_centered_absolute_onset_and_signed_capture(self):
        for mapping in (((0, 1), (1, 1), (2, 1)), ((1, -1), (0, 1), (2, -1))):
            for axis in "xy":
                for configured in (-1, 0, 1):
                    for sign in (-1, 1):
                        with self.subTest(mapping=mapping, axis=axis, configured=configured, sign=sign):
                            obj = make_homing(axes_map=mapping, threshold=.5, impact_sign=configured)
                            base = (32, -64, 128)
                            native_baseline(obj, base)
                            obj.start_capture()
                            attempt, sent = arm_axis(obj, axis, 200, 1000)
                            attempt.auto_direction = -sign
                            rows = [base] * 8
                            index, map_sign = obj._raw_axis(axis)
                            hit = list(base)
                            hit[index] += sign * counts(1.)
                            rows[3] = tuple(hit)
                            obj._inspect(batch(200, rows))
                            self.assertEqual(sent.messages, [[HIT]])
                            self.assertEqual(attempt.onset_sample[2], 3)
                            self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 1.)
                            obj.disarm()
                            capture = obj.stop_capture()
                            channel = "xyz".index(axis) + 1
                            self.assertEqual(capture["samples"][3][channel], hit[index] * map_sign * attempt.count_g)
                            self.assertEqual(attempt.record["direction"], 0)
                            self.assertEqual(attempt.record["impact_sign"], 0)

    def test_every_configured_adapter_and_legacy_contract_accepts_both_centered_signs(self):
        names = ("adxl345 toolhead", "lis2dw", "lis3dh", "mpu9250", "icm20948", "custom_accel")
        for name in names:
            for legacy in (False, True):
                for axis in "xy":
                    for configured in (-1, 0, 1):
                        for sign in (-1, 1):
                            with self.subTest(name=name, legacy=legacy, axis=axis, configured=configured, sign=sign):
                                options = {"threshold": .5, "x_impact_sign": configured, "y_impact_sign": configured}
                                if name == "custom_accel":
                                    options["sensor_range"] = 8.
                                obj = backends.generic(name, options, legacy=legacy)
                                for number in range(12):
                                    backends.feed(obj, backends.samples(10. + number * .005, x=.75, y=-.5))
                                obj.start_capture()
                                attempt, sent = backends.arm(obj, axis)
                                attempt.auto_direction = -sign
                                rows = backends.samples(10.060, x=.75, y=-.5)
                                hit = list(rows[3])
                                hit["xy".index(axis) + 1] += sign * backends.G
                                rows[3] = tuple(hit)
                                backends.feed(obj, rows)
                                self.assertEqual(sent.messages, [[HIT]])
                                self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 1.)
                                self.assertAlmostEqual(obj.onset_time(attempt), rows[3][0], places=6)
                                obj.disarm()
                                capture = obj.stop_capture()
                                self.assertAlmostEqual(capture["samples"][3]["xy".index(axis) + 1], hit["xy".index(axis) + 1] / backends.G)

    def test_xyz_magnitude_first_crossing_is_unchanged_by_mirroring(self):
        identities = []
        for sign in (-1, 1):
            obj = make_homing(threshold=1.)
            base = (32, -64, 128)
            native_baseline(obj, base)
            attempt, sent = arm_axis(obj, "x", 200, 1000)
            attempt.vector_mode = True
            rows = [base] * 8
            rows[4] = (base[0], base[1], base[2] + sign * counts(2.))
            obj.reactor.now = .208
            obj._inspect(batch(200, rows))
            self.assertEqual(sent.messages, [[HIT]])
            identities.append((attempt.onset_sample, obj.onset_time(attempt), attempt.onset_raw))
        self.assertEqual(identities[0], identities[1])

    def test_range_metadata_scanning_follows_early_dispatch(self):
        for vector in (False, True):
            obj = make_homing(threshold=1.)
            native_baseline(obj)
            attempt, sent = arm_axis(obj, "x", 200, 1000)
            attempt.vector_mode = vector
            rows = [(counts(2.), -64, 128)] + [(2047, -64, 128)] * 7
            observe = obj._observe_response
            def after_dispatch(*args):
                self.assertEqual(sent.messages, [[HIT]])
                self.assertTrue(attempt.done)
                return observe(*args)
            with mock.patch.object(obj, "_observe_response", side_effect=after_dispatch):
                obj.reactor.now = .208
                obj._inspect(batch(200, rows))
            self.assertEqual(attempt.signal_quality.summary()["range_limit_axis_values"], 7)


class NativeDecoderTests(unittest.TestCase):
    def test_prearm_opposite_sign_and_consecutive_markers_match_live_and_capture(self):
        for mapping in (((0, 1), (1, 1), (2, 1)), ((1, -1), (0, 1), (2, 1))):
            obj = make_homing(axes_map=mapping, threshold=1.)
            base = (32, 32, 128)
            native_baseline(obj, base)
            obj.start_capture()
            attempt, sent = arm_axis(obj, "x", 200, 1000)
            index, sign = obj._raw_axis("x")
            preceding = list(base)
            preceding[index] = -32
            marked = list(base)
            marked[index] = plugin.CLIPPED
            obj._inspect(batch(193, [tuple(preceding)] + [tuple(marked)] * 7))
            self.assertEqual(sent.messages, [[HIT]])
            self.assertEqual(attempt.onset_sample, (193, 7, 7, 8))
            obj._inspect(batch(201, [tuple(marked)] * 8))
            obj.disarm()
            original = json.loads(json.dumps(attempt.record))
            capture = obj.stop_capture()
            expected = -2048 * sign * attempt.count_g
            self.assertEqual(capture["samples"][7][1], expected)
            self.assertTrue(all(row[1] == expected for row in capture["samples"][8:]))
            self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 2080 * attempt.count_g)
            self.assertEqual(capture["native_clip_decode"]["marker_axis_values"], 15)
            self.assertEqual(capture["native_clip_decode"]["scope"], "capture")
            self.assertEqual(capture["range_limit_axis_values"], 0)
            self.assertTrue(capture["native_clip_decode"]["true_amplitude_unknown"])
            self.assertEqual(json.loads(json.dumps(attempt.record)), original)

    def test_zero_and_negative_seeds_use_actual_endpoints_and_unknown_limits_do_not_guess(self):
        self.assertEqual(plugin.decode_native_values((32767, 32767, 32767), (0, -1, 1), (511, -512)), [511, -512, 511])
        with self.assertRaisesRegex(ValueError, "configured signed clip"):
            plugin.decode_native_values((32767, 0, 0), (0, 0, 0), None)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            plugin.decode_native_values((float("nan"), 0, 0), (0, 0, 0), (2047, -2048))

    def test_old_callback_cannot_replace_restarted_decoder_seed(self):
        obj = make_homing()
        obj._start_stream()
        def restart(_params):
            obj._stop_stream()
            obj._start_stream()
        with mock.patch.object(obj, "_inspect_batch", side_effect=restart):
            obj._inspect(batch(200, [(-100, 0, 128)] * 8))
        self.assertEqual(obj.native_decode_previous, (0, 0, 0))

    def test_generic_numeric_32767_is_never_a_beacon_marker(self):
        obj = backends.generic(options={"threshold": 1.})
        backends.baseline(obj)
        obj.start_capture()
        attempt, sent = backends.arm(obj)
        backends.feed(obj, backends.samples(10.060, x=32.767))
        self.assertEqual(sent.messages, [[HIT]])
        obj.disarm()
        capture = obj.stop_capture()
        self.assertEqual(capture["clipped_axis_values"], 0)
        self.assertIsNone(capture["native_clip_decode"])
        self.assertEqual(capture["range_limit_axis_values"], 16)


class RangeAndQualificationTests(unittest.TestCase):
    def test_finite_rails_and_native_markers_complete_live_noise_and_keep_exact_counts(self):
        for generic in (False, True):
            for value in ((-16., 16.) if generic else (-2048, 2047, 32767)):
                with self.subTest(generic=generic, value=value):
                    obj = noise_tests.NoiseAcquisitionTests().machine(decoded=generic)
                    if generic:
                        emit = obj.backend.helper.emit
                        def incoming(message):
                            message = dict(message, data=[(t, value * backends.G, y, z)
                                                         for t, _x, y, z in message["data"]])
                            emit(message)
                        obj.backend.helper.emit = incoming
                    else:
                        inspect = obj._inspect
                        def incoming(params):
                            rows = list(plugin.SAMPLE.iter_unpack(params["data"]))
                            replaced = [(value, y, z) for _x, y, z in rows]
                            if value == 32767:
                                replaced[0] = (-2048, replaced[0][1], replaced[0][2])
                            inspect(dict(params, data=b"".join(plugin.SAMPLE.pack(*row) for row in replaced)))
                        obj._inspect = incoming
                    noise_tests.noise_flow(obj)
                    context = obj.start_noise_measurement()
                    capture = obj.collect_noise_window(lambda: None)
                    obj.stop_noise_measurement()
                    self.assertIsNone(capture["fault"])
                    self.assertGreater(capture["range_limit_axis_values"], 0)
                    self.assertTrue(capture["amplitude_limited"])
                    if value == 32767:
                        self.assertGreater(capture["clipped_axis_values"], 0)
                        self.assertEqual(capture["native_clip_decode"]["scope"], "noise_window")
                        self.assertTrue(all(row[1] == -16. for row in capture["samples"]))
                    else:
                        self.assertEqual(capture["clipped_axis_values"], 0)
                    session = {"context": {"sensor": context}, "iterations": [], "run_id": "signal_noise"}
                    record = obj.workflows._noise_record(session, 1, capture)
                    self.assertEqual(record["range_limit_axis_values"], capture["range_limit_axis_values"])
                    self.assertEqual(record["clipped_axis_values"], capture["clipped_axis_values"])

    def test_beta_keeps_both_finite_endpoints_and_markers_but_later_timestamp_fault_is_fatal(self):
        for value in (-2048, 2047, 32767):
            obj = beta_tests.BetaRawCaptureTests().owner()
            obj._capture_batch(batch(8, [(value, 0, 128), (0, 0, 128)], delta_clock=1))
            obj.reactor.now = .020
            obj._capture_batch(batch(7, [(value, 0, 128), (0, 0, 128)], delta_clock=1))
            capture = obj.stop_capture()
            self.assertIsNotNone(capture["fault"])
            self.assertTrue(capture["amplitude_limited"])
            self.assertEqual(capture["clipped_axis_values"] > 0, value == 32767)
            self.assertEqual(capture["range_limit_axis_values"] > 0, value != 32767)

    def test_native_and_generic_finite_endpoints_are_valid_quiet_references(self):
        for generic in (False, True):
            for sign in (-1, 1):
                if generic:
                    obj, endstop = backends.GenericPreparationTest().session()
                    value = sign * obj.backend.full_scale
                else:
                    obj = make_homing()
                    endstop = FakeEndstop("x")
                    obj.session_axes = {"x": FakeRail(endstop, info(speed=100., retract=0.))}
                    value = obj.helper._clip_values[0 if sign > 0 else 1] * obj.scale["scale"] / 1000.
                obj.reactor.now = 10.
                stationary_stream(obj, (value, 0., 1.))
                obj._prepare_reference()
                endstop.get_steppers = lambda: []
                attempt = obj.prepare(endstop, obj.preparation.last + .001)
                self.assertIsNone(obj.sensor_fault)
                self.assertGreater(attempt.reference_signal_quality["range_limit_axis_values"], 0)
                self.assertTrue(attempt.reference_signal_quality["excess_amplitude_unknown"])

    def test_reference_only_uncertainty_survives_clean_response_refresh(self):
        obj = make_homing()
        native_baseline(obj)
        attempt, _sent = arm_axis(obj, "x", 200, 1000)
        attempt.reference_signal_quality = {"amplitude_limited": True, "native_clip_decode": {"marker_axis_values": 2}}
        obj.disarm()
        obj._observe_response(attempt, batch(210, [(0, 0, 128)] * 8), [0, 0, 128] * 8, 8)
        self.assertTrue(attempt.record["amplitude_limited"])
        self.assertTrue(attempt.record["excess_amplitude_unknown"])

    def test_unknown_selected_axis_qualification_is_guarded_for_all_adapters_before_motion(self):
        for generic in (False, True):
            for metric in (None, "selected_axis", "unexpected"):
                obj = backends.generic() if generic else make_homing()
                obj.workflows.manual_axes.clear()
                obj.workflows.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": {"impact_metric": metric}})
                endstop = with_endstops(obj, ("x",))["x"]
                with mock.patch.object(obj, "_end_session", side_effect=AssertionError("passed guard")):
                    with self.assertRaisesRegex(CommandError, "legacy/unknown"):
                        obj._handle_rails_begin(None, [FakeRail(endstop, info())])
                self.assertEqual(obj.toolhead.moves, [])
                obj.cmd_SET(FakeGcmd(AXIS="X", IMPACT_SIGN=1))
                with self.assertRaisesRegex(CommandError, "legacy/unknown"):
                    obj.workflows.check_home_axes({"x": None})
                obj.cmd_SET(FakeGcmd(AXIS="X", THRESHOLD=1.))
                obj.workflows.check_home_axes({"x": None})
                self.assertFalse(obj.workflows.simple_detection("x"))
                self.assertNotIn("polarity_policy", obj.workflows.model.metadata["x"])
                obj.workflows.session = {"mode": "accuracy", "saved_thresholds": dict(obj.threshold_g),
                                         "saved_back_off": obj.back_off_dist}
                obj.workflows._restore()
                obj.workflows.session = None
                obj.workflows.check_home_axes({"x": None})

    def test_supervised_calibration_can_replace_legacy_selected_axis_qualification(self):
        rig = Rig()
        rig.workflow.manual_axes.clear()
        rig.workflow.model = types.SimpleNamespace(thresholds={"x": 2.}, metadata={"x": {"impact_metric": "selected_axis"}})
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0))
        self.assertEqual(rig.workflow.last_result["result"], "calibrated")
        self.assertEqual(rig.workflow.last_result["successes"], 10)
        metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
        self.assertEqual(metadata["polarity_policy"], plugin.POLARITY_POLICY)
        self.assertEqual(metadata["impact_metric"], "xyz_magnitude")
        self.assertEqual(metadata["threshold_margin_g"], .5)
        self.assertEqual(rig.configfile.saved[("accel_homing_model default", "version")], "1")

    def test_xyz_schema1_context_and_ten_successes_remain_compatible(self):
        for generic in (False, True):
            obj = backends.generic() if generic else make_homing()
            endstop = with_endstops(obj, ("x",))["x"]
            obj.rails_by_axis["x"] = FakeRail(endstop, info())
            obj.workflows.manual_axes.clear()
            context = obj.workflows._model_context("x")
            metadata = dict(context, impact_metric="xyz_magnitude")
            cfg = FakeConfig(None, {"version": 1, "x_threshold": 2., "x_successes": 10,
                                    "x_metadata": json.dumps(metadata)}, name="accel_homing_model default")
            obj.workflows.model = AccelHomingModel(cfg)
            obj.workflows._validate_model_connection()
            obj.workflows.check_home_axes({"x": None})
            self.assertTrue(obj.workflows.simple_detection("x"))
            self.assertEqual(obj.workflows._model_context("x"), context)
            self.assertEqual(obj.workflows.model.version, 1)


if __name__ == "__main__":
    unittest.main()
