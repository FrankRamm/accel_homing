"""Known Beacon marker input policy. Synthetic software evidence, not contact accuracy."""
import copy
import json
import tempfile
import unittest
from unittest import mock

import accel_homing_charts as charts
import accel_homing_filters as filters
import test_rail_acceptance as rails
from harness import FakeEndstop, batch, counts, make_homing, plugin, stationary_stream
from test_chronological_clipping import native_filtered_capture
from test_noise_recovery import sensor
from test_retract_noise import noise_flow
import test_filter_engine as engine_tests
from accel_homing_model import AccelHomingModel
from test_workflows import Config


class NativeMarkerCodecTests(unittest.TestCase):
    def test_actual_configured_limits_and_zero_convention_match_beacon(self):
        for clips in ((2047, -2048), (511, -512), (127, -128)):
            self.assertEqual(filters.decode_native_row((32767, 32767, 32767), (0, -1, 2), clips),
                             (clips[0], clips[1], clips[0]))
        for clips in (None, (32767, -32768), (2047, 2048), (True, -2), (2047,), (float('nan'), -2048)):
            with self.assertRaises(ValueError):
                filters.decode_native_row((32767, 0, 0), (0, 0, 0), clips)

    def test_consecutive_multi_axis_markers_preserve_prior_sensor_sign(self):
        raw = (-2, 0, 3, 32767, 32767, 32767, 32767, 32767, 32767)
        self.assertEqual(filters.decode_native_values(raw, (0, 0, 0), (2047, -2048)),
                         [-2, 0, 3, -2048, 2047, 2047, -2048, 2047, 2047])

    def test_nonfinite_decoder_input_remains_invalid(self):
        for xyz in ((float('nan'), 0, 0), (0, float('inf'), 32767)):
            with self.assertRaises(ValueError):
                filters.decode_native_row(xyz, (0, 0, 0), (2047, -2048))


class NativeMarkerRuntimeTests(unittest.TestCase):
    def test_unowned_beacon_stream_has_no_extra_decoder_or_handler_mutation(self):
        obj = make_homing()
        obj.homing_active = obj.stream_on = False
        obj.helper._stream_en = 1
        obj.native_decode_previous = (-10, 20, 30)
        calls = []
        obj.saved_handler = calls.append
        params = batch(1001, [(32767, 0, 128)] * 8)
        with mock.patch.object(plugin, 'decode_native_values', side_effect=AssertionError('unowned decoder')) as decode:
            obj._batch_hook(params)
        decode.assert_not_called()
        self.assertEqual(obj.native_decode_previous, (-10, 20, 30))
        self.assertIs(calls[0], params)
        self.assertNotIn('_native_decode_previous', params)

    def capture(self, version=2, **options):
        return native_filtered_capture(filter_config={'version': version}, clock_hz=32000000.,
                                       sample_period=.000302, **options)

    def test_marker_before_at_after_confirmation_is_decoded_and_requires_real_confirmation(self):
        for version in (1, 2):
            for position in (0, 1, 4, 7):
                with self.subTest(version=version, position=position):
                    capture, record = self.capture(version, clip_position=position)
                    self.assertEqual(record['result'], 'impact')
                    self.assertIsNone(record['sample_fault'])
                    self.assertFalse(record['response_censored'])
                    self.assertIsNotNone(record['confirmation_time_s'])
                    self.assertTrue(capture['raw_integrity']['raw_samples_validated'])
                    self.assertNotIn('response_censoring', capture)
                    self.assertIsNone(filters.validate_filtered_capture_prefix(capture, record))
                    self.assertEqual(capture['native_clip_decode']['marker_axis_values'], 1)
                    first = capture['detector']['first_armed_sample_index']
                    self.assertEqual(capture['samples'][first + position][1], 2047 * .0078125)
                    self.assertLess(record['impact_peak_g'], 30.)
                    result = filters.compare_filter_capture(capture, 1., 1.)
                    self.assertTrue(result['filtered_decision']['decision_observed'])

    def test_one_marker_row_alone_is_not_an_impact(self):
        rows = [(32767, 0, counts(1.))] + [(0, 0, counts(1.))] * 31
        capture, record = self.capture(clip_position=None, row_values=rows, packet_size=32)
        self.assertEqual(record['result'], 'no_trigger')
        self.assertIsNone(record['confirmation_time_s'])
        self.assertIsNone(record['sample_fault'])
        with self.assertRaises(ValueError):
            charts.prepare_capture('failed_marker', 'x', capture, record)

    def test_sensor_sign_survives_packet_boundary_machine_axis_map_and_clock_remap(self):
        rows = [(0, 0, counts(1.))] * 7 + [(-counts(3.), 0, counts(1.))] + [(32767, 0, counts(1.))] * 8
        for version in (1, 2):
            for shift in (-.001, .00002):
                capture, record = self.capture(version, clip_position=None, row_values=rows, packet_size=16,
                    split=8, axes_map=((1, 1), (0, -1), (2, 1)), clock_shift=shift)
                self.assertEqual(record['result'], 'impact')
                self.assertIsNone(filters.validate_filtered_capture_prefix(capture, record))
                first = capture['detector']['first_armed_sample_index']
                self.assertEqual(capture['samples'][first + 8][2], 2048 * .0078125)
                result = filters.compare_filter_capture(capture, 1., 1.)
                self.assertEqual(result['filtered_decision']['original_confirmation_time_s'], record['trigger_confirmation_time_s'])

    def test_arm_trimming_keeps_prearm_sensor_sign_without_counting_prearm_support(self):
        rows = [(-counts(3.), 0, counts(1.))] * 2 + [(32767, 0, counts(1.))] + [(-counts(3.), 0, counts(1.))] * 5
        capture, record = self.capture(clip_position=None, row_values=rows, arm_straddle=2)
        first = capture['detector']['first_armed_sample_index']
        self.assertEqual(capture['samples'][first][1], -2048 * .0078125)
        self.assertGreater(record['trigger_confirmation_time_s'], record['arm_time_s'])
        self.assertIsNone(filters.validate_filtered_capture_prefix(capture, record))

    def test_decoding_never_scans_future_packet_rows_before_dispatch(self):
        obj, attempt = rails.EndpointRuntimeTests().machine(2)
        original = plugin.decode_native_values
        def checked(*args):
            self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)
            return original(*args)
        with mock.patch.object(plugin, 'decode_native_values', side_effect=checked):
            rails.EndpointRuntimeTests().packet(obj, 1001, [(counts(3.), 0, 128)] * 4 + [(32767, 0, 128)] * 4)
        self.assertEqual(attempt.reason, 'impact')

    def test_old_callback_cannot_overwrite_restarted_decoder_seed(self):
        obj, attempt = rails.EndpointRuntimeTests().machine(2)
        original = obj._capture_batch
        obj.start_capture()
        def restart(params, admission=None):
            obj._start_stream()
            return original(params, admission)
        with mock.patch.object(obj, '_capture_batch', side_effect=restart):
            rails.EndpointRuntimeTests().packet(obj, 1001, [(-counts(3.), 0, 128)] * 8)
        self.assertEqual(attempt.reason, 'impact')
        self.assertEqual(obj.native_decode_previous, (0, 0, 0))

    def test_unknown_or_changed_native_endpoints_fail_before_filtered_arm(self):
        for limits in (None, (32767, -32768), (511, -512)):
            obj, attempt = rails.EndpointRuntimeTests().machine(2)
            attempt.native_clip_values = (2047, -2048)
            obj.helper._clip_values = limits
            with self.assertRaises((ValueError, Exception)):
                obj.arm(attempt)

    def test_nonfinite_and_repeated_time_after_marker_still_abort(self):
        for failure in ('nonfinite', 'repeated'):
            obj, attempt = rails.EndpointRuntimeTests().machine(2)
            if failure == 'nonfinite':
                rails.EndpointRuntimeTests().packet(obj, 1001, [(32767, 0, 128), (0, float('nan'), 128)] + [(0, 0, 128)] * 6)
                self.assertEqual(attempt.reason, 'invalid_data')
            else:
                rails.EndpointRuntimeTests().packet(obj, 1001, [(32767, 0, 128)] + [(0, 0, 128)] * 7)
                self.assertFalse(attempt.done)
                rails.EndpointRuntimeTests().packet(obj, 1001, [(0, 0, 128)] * 8)
                self.assertEqual(attempt.reason, 'invalid_data')
            self.assertEqual(len(attempt.endstop.trigger_log.messages), 1)

    def test_frozen_marker_stream_uses_existing_frozen_guard(self):
        obj, attempt = rails.EndpointRuntimeTests().machine(2)
        attempt.bases = (2047., 0., 128.)
        attempt.filter_engine.offset_g = tuple(value * attempt.count_g for value in attempt.bases)
        obj.native_decode_previous = (2047, 0, 128)
        for offset in (0, 8, 16, 24, 32):
            rails.EndpointRuntimeTests().packet(obj, 1001 + offset, [(32767, 0, 128)] * 8)
        obj._commanded_speed = lambda *_: 100.
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, 'frozen')

    def test_generic_numeric_32767_is_not_a_native_marker(self):
        obj, attempt = engine_tests.RuntimeTests().filtered()
        high = (32767., attempt.bases[1], attempt.bases[2])
        engine_tests.RuntimeTests().packet(obj, attempt, attempt.arm_time + .0001, [high] * 8)
        self.assertEqual(attempt.reason, 'impact')
        self.assertEqual(attempt.marker_axis_values, 0)
        self.assertGreater(attempt.impact_peak * attempt.count_g, 30.)


class NativeMarkerProvenanceTests(unittest.TestCase):
    def test_mutated_summary_seed_limits_exported_row_or_reference_cannot_qualify(self):
        capture, record = native_filtered_capture(split=4)
        for mutate in (
            lambda c, r: c.pop('native_clip_decode'),
            lambda c, r: c['native_clip_decode'].update(marker_axis_values=2),
            lambda c, r: c['native_clip_decode'].update(clip_values=[511, -512]),
            lambda c, r: c['_native_source_batches'][0].update(decode_previous_raw=[-1, 0, 0]),
            lambda c, r: c['_native_source_batches'][-1].update(decode_previous_raw=[-1, 0, 0]),
            lambda c, r: c['samples'].__setitem__(-1, (c['samples'][-1][0], 0., 0., 1.)),
            lambda c, r: c['detector']['filter_reference']['offset_g'].__setitem__(0, 1.),
            lambda c, r: r.update(native_clip_decode=None)):
            changed, changed_record = copy.deepcopy(capture), copy.deepcopy(record)
            mutate(changed, changed_record)
            with self.assertRaises(ValueError):
                filters.validate_filtered_capture_prefix(changed, changed_record)

    def test_status_and_frozen_receipt_identify_estimates_without_original_raw_scratch(self):
        capture, record = native_filtered_capture()
        prepared = charts.prepare_capture('marker_estimate', 'x', capture, record, simulated=True)
        _receipt, frozen, metadata = charts._prepared_capture(prepared)
        capture['native_clip_decode']['clip_values'][0] = 1
        self.assertEqual(metadata['native_clip_decode']['clip_values'], [2047, -2048])
        self.assertEqual(frozen['peak_semantics'], 'bounded_native_endpoint_estimate')
        self.assertIn('true sign/peak unknown', charts._range_observation_lines(metadata)[0])
        self.assertLess(len(json.dumps(metadata)), 4000)

    def test_capture_tail_nonfinite_or_gap_remains_fatal_with_decoded_markers(self):
        capture, record = native_filtered_capture()
        for failure in ('nonfinite', 'gap'):
            current = copy.deepcopy(capture)
            source = current['_native_source_batches'][-1]
            if failure == 'nonfinite':
                raw = [v for row in plugin.SAMPLE.iter_unpack(source['data']) for v in row]
                raw[-1] = float('nan')
                source['data'] = tuple(raw)
            else:
                source['filter_start_time_s'] += .020
                source['filter_end_time_s'] += .020
            with self.assertRaises(ValueError):
                filters.validate_filtered_capture_prefix(current, record)


class NativeMarkerAcquisitionTests(unittest.TestCase):
    def test_beta_export_snapshots_marker_summary_with_the_exported_rows(self):
        obj = sensor()
        inspect = obj._inspect
        def incoming(params):
            params = dict(params)
            rows = list(plugin.SAMPLE.iter_unpack(params['data']))
            params['data'] = b''.join(plugin.SAMPLE.pack(32767, row[1], row[2]) for row in rows)
            inspect(params)
        obj._inspect = incoming
        noise_flow(obj)
        token = obj.start_beta_motion_capture(lambda: None)
        for _ in range(15):
            obj.reactor.pause(obj.reactor.now + .005)
        original_lock = obj.lock
        changed = []
        class SnapshotLock:
            def __enter__(self):
                original_lock.acquire()
            def __exit__(self, *_):
                original_lock.release()
                if not changed:
                    changed.append(True)
                    with original_lock:
                        window = token['window']
                        last = window['samples'][-1]
                        window['samples'].append((last[0] + .001, *last[1:]))
                        window['marker_axis_values'] += 1
                        window['marker_last_sample'] = [999999, 4, 0, 5]
        obj.lock = SnapshotLock()
        capture = obj.stop_beta_motion_capture(token, lambda: None)
        decoding = capture['native_clip_decode']
        self.assertTrue(changed)
        self.assertEqual(decoding['marker_axis_values'], decoding['baseline_marker_axis_values'] + len(capture['samples']))

    def test_native_reference_and_noise_decode_known_markers(self):
        obj = sensor()
        inspect = obj._inspect
        def incoming(params):
            params = dict(params)
            raw = list(plugin.SAMPLE.iter_unpack(params['data']))
            params['data'] = b''.join(plugin.SAMPLE.pack(32767, row[1], row[2]) for row in raw)
            inspect(params)
        obj._inspect = incoming
        noise_flow(obj)
        obj.start_noise_measurement()
        capture = obj.collect_noise_window(lambda: None)
        self.assertIsNone(obj.sensor_fault)
        self.assertTrue(all(row[1] == 2047 * .0078125 for row in capture['samples']))
        self.assertGreater(capture['native_clip_decode']['marker_axis_values'], 100)
        obj.stop_noise_measurement()

    def test_actual_accepted_reference_markers_and_negative_tail_seed_survive_into_arm(self):
        reference = [(counts(.05), 0, counts(1.))] * 16
        reference[5] = (32767, 0, counts(1.))
        reference[-1] = (-counts(.1), 0, counts(1.))
        for version in (1, 2):
            capture, record = native_filtered_capture(filter_config={'version': version},
                clock_hz=32000000., sample_period=.000302, reference_rows=reference, clip_position=0)
            self.assertEqual(record['result'], 'impact')
            self.assertGreater(capture['detector']['filter_reference']['offset_g'][0], .01)
            self.assertGreater(capture['native_clip_decode']['marker_axis_values'], 1)
            first = capture['detector']['first_armed_sample_index']
            self.assertEqual(capture['samples'][first][1], -2048 * .0078125)
            self.assertIsNone(filters.validate_filtered_capture_prefix(capture, record))

    def test_beta_moving_baseline_and_trace_keep_marker_metadata_and_endpoint_guard(self):
        obj = sensor()
        inspect = obj._inspect
        def incoming(params):
            params = dict(params)
            rows = list(plugin.SAMPLE.iter_unpack(params['data']))
            params['data'] = b''.join(plugin.SAMPLE.pack(32767, row[1], row[2]) for row in rows)
            inspect(params)
        obj._inspect = incoming
        noise_flow(obj)
        token = obj.start_beta_motion_capture(lambda: None)
        for _ in range(15):
            obj.reactor.pause(obj.reactor.now + .005)
        capture = obj.stop_beta_motion_capture(token, lambda: None)
        self.assertGreater(capture['native_clip_decode']['baseline_marker_axis_values'], 100)
        self.assertGreater(capture['native_clip_decode']['marker_axis_values'],
                           capture['native_clip_decode']['baseline_marker_axis_values'])
        self.assertTrue(capture['excess_amplitude_unknown'])
        self.assertFalse(obj.stream_on)
        obj = sensor()
        noise_flow(obj)
        token = obj.start_beta_motion_capture(lambda: None)
        obj.helper._clip_values = (2047, -1024)
        with self.assertRaises(Exception):
            obj.stop_beta_motion_capture(token, lambda: None)
        self.assertFalse(obj.stream_on)

    def test_beta_home_uses_unchanged_raw_controller_seed_after_opposite_prearm_sign(self):
        obj = make_homing(threshold=10.)
        obj.start_capture(beta=True)
        obj.reactor.now = 1.
        obj._inspect(batch(992, [(-1, 0, 128)] * 8, receive_time=1.))
        from harness import arm_axis
        attempt, _sent = arm_axis(obj, 'x', 1000, 2000)
        attempt.vector_mode = True
        attempt.bases = (counts(8.), 0., 128.)
        obj.reactor.now = 1.009
        obj._inspect(batch(1001, [(32767, 0, 128), (-2048, 0, 128)] + [(-counts(3.), 0, 128)] * 6,
                           receive_time=obj.reactor.now))
        self.assertEqual(attempt.reason, 'impact')
        self.assertEqual(attempt.onset_sample[2], 1)
        obj.disarm()
        capture = obj.stop_capture()
        record = dict(attempt.record)
        telemetry = obj.beta_home_telemetry(capture, record)
        index = capture['detector']['onset_sample_index']
        self.assertEqual(capture['samples'][index - 1][1], 2047 * .0078125)
        self.assertEqual(capture['native_clip_decode']['armed_seed_convention'], 'legacy_raw_controller_last_sensor_values')
        self.assertTrue(telemetry['onset_window']['sample_count'])


class MarkerWorkflowRig(engine_tests.FilteredRig):
    def __init__(self, options, shift=0., straddle=0):
        super().__init__(options)
        self.marker_shift, self.marker_straddle = shift, straddle

    def sensor_context(self):
        context = super().sensor_context()
        context.update(backend='beacon_raw', count_g=.0078125)
        return context

    def stop_capture(self):
        existing = super().stop_capture()
        source = existing.get('_record')
        if source is None:
            return existing
        capture, record = native_filtered_capture(axis=source['axis'],
            filter_config=source.get('detector_config'), time_offset=self.capture_stops * 10.,
            clock_shift=self.marker_shift, arm_straddle=self.marker_straddle)
        positions = {key: source[key] for key in ('start_position_mm', 'expected_travel_mm') if key in source}
        source.update(record)
        source.update(positions)
        capture['_record'] = source
        return capture


class NativeMarkerWorkflowTests(unittest.TestCase):
    def test_marker_calibration_and_accuracy_with_signed_clock_remaps_keep_confirmation_and_save_rules(self):
        for shift in (-.001, .00002):
            with tempfile.TemporaryDirectory() as directory:
                rig = MarkerWorkflowRig({'charts_directory': directory}, shift=shift, straddle=2)
                rig.calibrate(save=0, DETECTOR='FILTERED')
                self.assertEqual(rig.workflow.last_result['result'], 'calibrated')
                self.assertEqual(rig.workflow.last_result['saved_threshold_g'], 1.5)
                metadata = json.loads(rig.configfile.saved[('accel_homing_model default', 'x_metadata')])
                self.assertEqual(metadata['peak_semantics'], 'bounded_native_endpoint_estimate')
                self.assertTrue(all(item['marker_axis_values'] for item in metadata['native_clip_decode']))
            with tempfile.TemporaryDirectory() as directory:
                rig = MarkerWorkflowRig({'charts_directory': directory}, shift=shift, straddle=2)
                rig.accuracy(DETECTOR='FILTERED', FILTERED_THRESHOLD=1.)
                self.assertEqual(rig.workflow.last_result['result'], 'complete')
                self.assertEqual(len(rig.workflow.last_result['axis_results']['x']['trials']), 2)

    def test_existing_raw_schema1_thresholds_and_fingerprint_remain_selectable(self):
        rig = engine_tests.FilteredRig()
        values = {'version': 1, 'x_threshold': 4.25, 'y_threshold': 2.5, 'x_successes': 10, 'y_successes': 10}
        contexts = {}
        for axis in ('x', 'y'):
            contexts[axis] = rig.workflow._model_context(axis)
            metadata = dict(contexts[axis], detector_type='raw', impact_metric='xyz_magnitude')
            values[axis + '_metadata'] = json.dumps(metadata)
        rig.workflow.model = AccelHomingModel(Config(None, values, 'accel_homing_model default'))
        rig.owner.threshold_g.update(x=4.25, y=2.5)
        for axis, threshold in (('x', 4.25), ('y', 2.5)):
            self.assertEqual(rig.workflow.detector_type(axis), 'raw')
            self.assertEqual(rig.workflow._profile(axis, 'raw')['threshold_g'], threshold)
            self.assertEqual(rig.workflow._model_context(axis), contexts[axis])
        rig.accuracy(DETECTOR='RAW')
        self.assertEqual(rig.workflow.last_result['result'], 'complete')
        self.assertEqual(rig.owner.threshold_g, {'x': 4.25, 'y': 2.5})


if __name__ == '__main__':
    unittest.main()
