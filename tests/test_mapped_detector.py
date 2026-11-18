"""Mapped machine-axis impact decisions on native and configured batch streams."""
import copy
import itertools
import unittest
from unittest import mock

from harness import CommandError, acquire_reference, batch, counts, make_homing, plugin
import test_backends as backends
from test_session import FakeRail, info, with_endstops


def mapping(axis="x", source=0, sign=1):
    return {"version": 1, "axis": axis, "source_axis": source, "sign": sign,
            "coordinate_frame": "configured_sensor_stream", "method": "bidirectional_velocity_v1",
            "confidence": 0.99, "diagnostics": {"checked": [1, 2]}}


def prepared(axis="x", source=0, sign=1, backend=False, legacy=False,
             configured=((0, 1), (1, 1), (2, 1))):
    obj = (backends.generic(options={"threshold": 1.0}, legacy=legacy) if backend else
           make_homing(axes_map=configured, threshold=1.0))
    profile = mapping(axis, source, sign)
    obj.workflows.detection_axis_mapping = lambda selected: profile
    obj.workflows.simple_detection = lambda selected: True
    endstop = with_endstops(obj, (axis,))[axis]
    obj.session_axes = {axis: FakeRail(endstop, info(retract=0.))}
    obj.reactor.now = 10.0
    acquire_reference(obj, endstop)
    when = (obj.backend.last_sample_time + 1. / obj.backend.rate if backend
            else obj.reactor.now + .001)
    attempt = obj.prepare(endstop, when)
    obj.start_capture()
    obj.arm(attempt)
    return obj, attempt, profile


def deliver(obj, attempt, deltas, offset=0.0, age=.001):
    """Offsets are in the raw detector frame, g; use the real backend callback."""
    count_g = attempt.count_g
    start = attempt.arm_time + offset
    if obj.backend is not None:
        rows = [(start + n / 3200.,) + tuple((base * count_g + value) * backends.G
                                            for base, value in zip(attempt.bases, row))
                for n, row in enumerate(deltas)]
        backends.feed(obj, rows, age=age)
        return rows
    rows = [tuple(int(round(base + value / count_g))
                  for base, value in zip(attempt.bases, row)) for row in deltas]
    first = obj.sensor_clock.print_time_to_clock(start)
    obj.reactor.now = (first + len(rows) - 1) / obj.beacon_freq + age
    params = batch(first, rows, receive_time=obj.reactor.now, delta_clock=len(rows) - 1)
    obj._inspect(params)
    return params


class MappedDetectorTests(unittest.TestCase):
    def test_native_and_batch_only_mapped_component_can_trigger(self):
        configurations = (((0, 1), (1, 1), (2, 1)), ((2, -1), (0, 1), (1, -1)))
        for backend, axis, source, sign, configured in itertools.product(
                (False, True), "xy", range(3), (-1, 1), configurations):
            with self.subTest(backend=backend, axis=axis, source=source, sign=sign, configured=configured):
                obj, attempt, _profile = prepared(axis, source, sign, backend, configured=configured)
                expected_index = source if backend else configured[source][0]
                expected_sign = sign if backend else configured[source][1] * sign
                self.assertEqual((attempt.index, attempt.map_sign), (expected_index, expected_sign))
                self.assertTrue(attempt.vector_mode)
                self.assertIsNone(attempt.proof_noise)
                self.assertIsNone(attempt.proof_deadline)
                self.assertIsNone(attempt.motion_deadline)
                # Every non-selected component exceeds the threshold by 20x.
                noise = tuple(.25 if n == expected_index else 20. for n in range(3))
                deliver(obj, attempt, [noise] * 8)
                self.assertFalse(attempt.done, attempt.reason)
                self.assertAlmostEqual(attempt.noise_checked * attempt.count_g, .25)
                hit = list(noise)
                hit[expected_index] = -1.25
                rows = [noise] * 3 + [tuple(hit)] + [noise] * 4
                deliver(obj, attempt, rows, offset=.0025 if backend else .008)
                self.assertEqual(attempt.reason, "impact")
                self.assertEqual(attempt.onset_sample[2], 3)
                self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 1.25)
                self.assertAlmostEqual(attempt.impact_peak * attempt.count_g, 1.25)
                obj.disarm()
                self.assertEqual(attempt.record["impact_metric"], "mapped_axis")
                with mock.patch.object(obj, "_motion_state", side_effect=AssertionError("old motion proof")):
                    self.assertEqual(obj.trigger_time(attempt), obj.onset_time(attempt))

    def test_legacy_batch_api_does_not_reapply_driver_scaled_map(self):
        obj, attempt, _profile = prepared("y", 2, -1, backend=True, legacy=True)
        self.assertEqual(obj.backend.sensor.axes_map, [(1, -123), (0, 123), (2, 123)])
        deliver(obj, attempt, [(8., 9., -1.5)] * 8)
        self.assertEqual(attempt.reason, "impact")
        self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 1.5)

    def test_changed_upstream_batch_mapping_stops_before_interpreting_new_frame(self):
        for legacy in (False, True):
            obj, attempt, _profile = prepared(backend=True, legacy=legacy)
            obj.backend.sensor.axes_map[0] = (2, -123)
            deliver(obj, attempt, [(2., 0., 0.)] * 8)
            self.assertEqual(attempt.reason, "invalid_data")
            self.assertIsNone(attempt.onset_sample)

    def test_exact_threshold_and_onset_identity_are_selected_component_only(self):
        for backend in (False, True):
            obj, attempt, _profile = prepared("y", 1, backend=backend)
            rows = [(3., .75, 4.), (9., 1., 8.), (10., 2., 10.)] + [(5., 0., 6.)] * 5
            deliver(obj, attempt, rows)
            self.assertEqual(attempt.reason, "impact")
            self.assertEqual(attempt.onset_sample[2], 1)
            self.assertAlmostEqual(attempt.onset_raw * attempt.count_g, 1.)
            self.assertAlmostEqual(attempt.impact_peak * attempt.count_g, 2.)

    def test_prepared_mapping_and_capture_frame_are_detached_from_future_mutation(self):
        configured = ((2, -1), (0, 1), (1, -1))
        for backend in (False, True):
            obj, attempt, profile = prepared("x", 1, -1, backend=backend, configured=configured)
            original = copy.deepcopy(profile)
            profile["source_axis"] = 2
            profile["diagnostics"]["checked"].append(3)
            if not backend:
                obj.helper.config.axes_map = ((0, 1), (1, 1), (2, 1))
            rows = [(1.5 if n == attempt.index else .1) for n in range(3)]
            deliver(obj, attempt, [tuple(rows)] * 8)
            obj.disarm()
            captured = obj.stop_capture()
            self.assertEqual(attempt.axis_mapping, original)
            self.assertEqual(captured["axis_mapping"], original)
            self.assertEqual(captured["detector"]["axis_mapping"], original)
            self.assertEqual(attempt.record["axis_mapping"], original)
            self.assertEqual(captured["coordinate_frame"], "configured_sensor_stream")
            source = original["source_axis"] + 1
            self.assertAlmostEqual(abs(captured["samples"][0][source] - captured["baseline_g"][source - 1]), 1.5)
            captured["axis_mapping"]["diagnostics"]["checked"].append(7)
            self.assertEqual(attempt.axis_mapping, original)
            self.assertEqual(attempt.record["axis_mapping"], original)

    def test_unrelated_native_clip_marker_cannot_trigger_selected_axis(self):
        obj, attempt, _profile = prepared("x", 0)
        clock = obj.sensor_clock.print_time_to_clock(attempt.arm_time)
        rows = [(int(attempt.bases[0]), plugin.CLIPPED, int(attempt.bases[2]))] * 8
        obj.reactor.now = attempt.arm_time + .008
        obj._inspect(batch(clock, rows, receive_time=obj.reactor.now))
        self.assertFalse(attempt.done)
        self.assertGreater(attempt.signal_quality.summary()["clipped_axis_values"], 0)

    def test_stale_native_and_future_batches_fail_before_selected_impact(self):
        obj, attempt, _profile = prepared()
        deliver(obj, attempt, [(2., 0., 0.)] * 8, age=obj.stream_timeout + .01)
        self.assertEqual(attempt.reason, "stale_data")
        obj, attempt, _profile = prepared(backend=True)
        deliver(obj, attempt, [(2., 0., 0.)] * 8, age=-.020)
        self.assertEqual(attempt.reason, "stale_data")

    def test_late_configured_batch_preserves_existing_age_policy_and_silence_guard(self):
        obj, attempt, _profile = prepared(backend=True)
        deliver(obj, attempt, [(.1, 5., 6.)] * 8, age=.080)
        self.assertFalse(attempt.done)
        obj.reactor.now += obj.stream_timeout + .001
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "no_data")

    def test_invalid_profile_and_missing_general_calibration_map_fail_before_arm(self):
        for profile in (None, mapping("y"), dict(mapping(), source_axis=True), dict(mapping(), sign=0)):
            obj, attempt, _original = prepared()
            obj.disarm()
            obj.workflows.session = {"mode": "calibration", "axis": "x"}
            obj.workflows.detection_axis_mapping = lambda axis, profile=profile: profile
            with self.assertRaisesRegex(CommandError, "mapping"):
                obj.prepare(attempt.endstop, attempt.arm_time + .1)
            self.assertIsNone(obj.attempt)


if __name__ == "__main__":
    unittest.main()
