"""Exercise the gentle detector and its capture through real sensor callbacks."""
import unittest

from harness import CommandError, make_homing, plugin, stationary_stream
from test_backends import generic
from test_mapped_detector import deliver, mapping
from test_session import FakeRail, info, with_endstops


class GentleAxisCaptureTests(unittest.TestCase):
    def prepare_attempt(self, axis, source, sign, generic_driver, notch=True):
        options = {"threshold": 1., axis + "_notch_frequency": 100. if notch else 0.}
        obj = (generic("lis2dw cartographer", options=options) if generic_driver else
               make_homing(axes_map=((2, -1), (0, 1), (1, -1)), options=options))
        selected = mapping(axis, source, sign)
        obj.workflows.gentle_homing = True
        obj.workflows.home_axis = axis
        obj.workflows.simple_detection = lambda current: True
        obj.workflows.detection_axis_mapping = lambda current: selected
        stop = with_endstops(obj, (axis,))[axis]
        obj.session_axes = {axis: FakeRail(stop, info(retract=0.))}
        obj.reactor.now = 10.
        obj.start_capture()
        raw_index = obj._capture_axes_mapping()[source][0]
        resting = [16., 16., 16.]  # Other channels include full-scale observations.
        resting[raw_index] = .25
        stationary_stream(obj, resting)
        obj._prepare_reference()
        self.assertTrue(all(len(row[2]) == 1 for row in obj.preparation._samples))
        self.assertEqual(set(obj.baseline_status["observed_peak_to_peak_g"]), {axis})
        when = obj.backend.last_sample_time + 1. / obj.backend.rate if generic_driver else obj.reactor.now + .001
        attempt = obj.prepare(stop, when)
        self.assertFalse(attempt.reference_signal_quality["amplitude_limited"])
        obj.arm(attempt)
        return obj, attempt, selected

    def test_clipping_other_channels_cannot_trigger_invalidate_or_pollute_saved_trace(self):
        for axis, source, sign in (("x", 1, -1), ("y", 0, 1)):
            for batch_driver in (False, True):
                with self.subTest(axis=axis, batch=batch_driver):
                    obj, attempt, selected = self.prepare_attempt(axis, source, sign, batch_driver)
                    other = (16. if batch_driver else plugin.CLIPPED * attempt.count_g)
                    noise = [other, other, other]
                    noise[attempt.index] = 0.
                    deliver(obj, attempt, [noise] * 8)
                    self.assertIsNone(attempt.onset_sample)
                    self.assertFalse(attempt.varied)
                    self.assertFalse(attempt.signal_quality.summary()["amplitude_limited"])
                    hit = list(noise)
                    hit[attempt.index] = -2.
                    offset = 8 / (3200. if batch_driver else 1000.)
                    deliver(obj, attempt, [noise] * 2 + [hit] + [noise] * 5, offset=offset)
                    self.assertEqual(attempt.reason, "impact")
                    self.assertEqual(attempt.onset_sample[2], 2)
                    self.assertEqual(sum(len(packet[2]) for packet in obj.capture_batches), obj.capture_samples)
                    obj.disarm()
                    result = obj.stop_capture()
                    self.assertEqual(result["sample_layout"], "mapped_axis_v1")
                    self.assertEqual(result["axis_mapping"], selected)
                    self.assertFalse(result["amplitude_limited"])
                    self.assertEqual(result["clipped_axis_counts"], {"xyz"[source]: 0})
                    self.assertAlmostEqual(result["baseline_g"], .25 * attempt.map_sign)
                    self.assertTrue(all(len(row) == 2 for row in result["samples"] + result["filtered_samples"]))
                    filtered_index = result["detector"]["onset_sample_index"] - result["signal_filter"]["first_sample_index"]
                    self.assertAlmostEqual(result["filtered_samples"][filtered_index][1],
                                           -attempt.map_sign * attempt.onset_raw * attempt.count_g)

    def test_selected_native_clip_preserves_sign_and_is_flagged(self):
        obj, attempt, _ = self.prepare_attempt("y", 1, -1, False, notch=False)
        rows = [[0., 0., 0.] for _ in range(8)]
        rows[0][attempt.index] = -.5
        rows[1][attempt.index] = plugin.CLIPPED * attempt.count_g - .25
        deliver(obj, attempt, rows)
        obj.disarm()
        result = obj.stop_capture()
        self.assertTrue(result["amplitude_limited"])
        index = result["detector"]["onset_sample_index"]
        self.assertEqual(result["clipped_axis_counts"], {"y": 1})
        self.assertAlmostEqual(result["samples"][index][1],
                               obj.helper._clip_values[1] * attempt.count_g * attempt.map_sign)

    def test_gentle_home_without_mapping_refuses_capture(self):
        obj = make_homing()
        obj.workflows.gentle_homing = True
        obj.workflows.home_axis = "x"
        obj.workflows.detection_axis_mapping = lambda axis: None
        with self.assertRaisesRegex(CommandError, "requires a qualified axis mapping"):
            obj.start_capture()
        self.assertIsNone(obj.capture_batches)
