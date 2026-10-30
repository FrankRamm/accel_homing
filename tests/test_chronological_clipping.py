"""Native causal-prefix and censored-response fixtures; all signals synthetic."""
import copy
import math
import unittest
import tempfile
import types
from pathlib import Path
from unittest import mock

from accel_homing_filters import (validate_filtered_capture_prefix, validate_censored_peak_record,
                                  compare_filter_capture, detector_config)
from harness import FakeEndstop, acquire_reference, batch, counts, make_homing
from test_session import FakeRail, info
from test_filter_engine import FilteredRig


def native_filtered_capture(axis="x", clip_position=4, clip_value=32767, packet_size=8,
                            split=None, clock_shift=0., axes_map=((0, 1), (1, 1), (2, 1)),
                            time_offset=0., arm_straddle=0, clock_hz=1000., sample_period=.001,
                            post_clip_g=3., filter_config=None, row_values=None, reference_rows=None):
    """Exercise real prepare/filter/capture callbacks, with no real hardware."""
    obj = make_homing(threshold=1., axes_map=axes_map)
    obj.reactor.now = time_offset
    if clock_hz != 1000.:
        obj.beacon_freq = clock_hz
        obj.sensor_clock.print_time_to_clock = lambda t: round(t * clock_hz)
        obj.sensor_clock.seconds_to_clock = lambda t: round(t * clock_hz)
        obj.clock_to_time = lambda clock: clock / clock_hz
        obj.impact_window = round(.015 * clock_hz)
    endstop = FakeEndstop(axis)
    endstop.get_steppers = lambda: []
    rail = FakeRail(endstop, info(speed=100., retract=0., second=100.))
    obj.session_axes = obj.rails_by_axis = {axis: rail}
    obj.workflows.session = {"mode": "calibration", "state": "running", "axis": axis,
                             "algorithm": "impact20_vector_v2", "detector_types": {axis: "filtered"},
                             "detector_configs": {axis: detector_config(filter_config)}}
    obj.workflows.in_home, obj.workflows.home_axis = True, axis
    obj.start_capture()
    obj._start_stream()
    if clock_hz == 1000.:
        acquire_reference(obj, endstop)
    else:
        previous = [None]
        def reference_flow():
            start = (obj.preparation.cutoff if previous[0] is None else previous[0]) + sample_period
            ref_rows = [(0, 0, counts(1.))] * 16 if reference_rows is None else list(reference_rows)
            obj._inspect(batch(round(start * clock_hz), ref_rows,
                               receive_time=obj.reactor.now, delta_clock=round(15 * sample_period * clock_hz)))
            previous[0] = start + 15 * sample_period
        obj.reactor.on_pause = reference_flow
        obj._prepare_reference()
    when = obj.preparation.last + sample_period
    attempt = obj.prepare(endstop, when + arm_straddle * sample_period)
    obj.arm(attempt)
    rows = list(row_values) if row_values is not None else [(counts(3.), 0, counts(1.))] * packet_size
    if len(rows) != packet_size:
        raise ValueError("fixture rows must match packet size")
    if clip_position is not None:
        rows[clip_position] = (clip_value, 0, counts(1.))
        for i in range(clip_position + 1, packet_size):
            rows[i] = (counts(post_clip_g), 0, counts(1.))
    blocks = [(0, packet_size)] if split is None else [(0, split), (split, packet_size)]
    for first, last in blocks:
        current = rows[first:last]
        if len(current) < 2:
            raise ValueError("fixture packets need at least two rows")
        start = obj.sensor_clock.print_time_to_clock(when + first * sample_period)
        obj.reactor.now = when + (last - 1) * sample_period + .001
        obj._inspect(batch(start, current, receive_time=obj.reactor.now,
                           delta_clock=round((len(current) - 1) * sample_period * clock_hz)))
    obj.disarm()
    if clock_shift:
        original = obj.clock_to_time
        obj.clock_to_time = lambda clock: original(clock) + clock_shift
    capture = obj.stop_capture()
    record = dict(attempt.record)
    record.update(axis=axis, run_id="native_censored_fixture", workflow_attempt=1,
                  workflow_phase="charts", workflow_threshold_g=1., capture_valid=attempt.reason == "impact",
                  travel_mm=150., travel_mm_exact=150., homing_speed_mm_s=100., homing_accel_mm_s2=5000.,
                  configured_homing_speed_mm_s=100.)
    if capture.get("response_censoring") is not None and attempt.reason == "impact":
        summary = validate_filtered_capture_prefix(capture, record)
        record.update(response_censoring=summary, response_censored=True, decision_prefix_validated=True,
                      observed_valid_prefix_peak_g=summary["observed_valid_prefix_peak_g"],
                      confirmation_time_s=capture["detector"]["confirmation_time_s"],
                      crossing_time_s=capture["detector"]["crossing_time_s"],
                      onset_time_s=capture["detector"]["onset_time_s"])
    if attempt.reason == "impact":
        record.update(confirmation_time_s=capture["detector"]["confirmation_time_s"],
                      crossing_time_s=capture["detector"]["crossing_time_s"],
                      onset_time_s=capture["detector"]["onset_time_s"])
    return capture, record


def historical_censored_capture(*args, **kwargs):
    """Construct the former public/private archive shape, not current live policy.

    The current software callback supplies original bytes and the pre-marker
    confirmation. Only archive/parser compatibility tests use this conversion.
    """
    capture, record = native_filtered_capture(*args, **kwargs)
    marker = None
    offset = 0
    for source in capture["_native_source_batches"]:
        raw = (source["data"] if isinstance(source["data"], tuple) else
               tuple(v for row in native_plugin.SAMPLE.iter_unpack(source["data"]) for v in row))
        count = len(raw) // 3
        first = source["first_sample_position"]
        for position in range(first, count):
            xyz = raw[position * 3:position * 3 + 3]
            if 32767 in xyz and marker is None:
                marker = (offset + position - first, source, position, count, xyz.index(32767))
        offset += count - first
    if marker is None:
        return capture, record
    index, source, position, count, axis = marker
    capture.pop("native_clip_decode", None)
    capture["detector"].pop("native_clip_decode", None)
    record.pop("native_clip_decode", None)
    capture["raw_integrity"]["raw_samples_validated"] = False
    if index <= capture["detector"]["confirmation_sample_index"]:
        record.update(result="invalid_data", sample_fault={"cause": "native_clip_sentinel"})
        capture["detector"]["result"] = "invalid_data"
        return capture, record
    samples = capture["samples"][:index]
    dt = (source["end_time_s"] - source["start_time_s"]) / (count - 1)
    original_dt = (source["filter_end_time_s"] - source["filter_start_time_s"]) / (count - 1)
    peak = max(math.sqrt(sum((value - base) ** 2 for value, base in zip(row[1:], capture["baseline_g"]))) for row in samples)
    summary = {"version": 1, "kind": "native_saturation_after_confirmation", "cause": "native_clip_sentinel",
        "first_invalid_sample": [source["start_clock"], source["delta_clock"], position, count],
        "first_invalid_sample_index": index, "first_invalid_time_s": source["start_time_s"] + position * dt,
        "first_invalid_original_time_s": source["filter_start_time_s"] + position * original_dt,
        "sensor_axis": "xyz"[axis], "raw_value": 32767,
        "full_scale_g": source["clip_values"][0] * source["count_g"],
        "original_sample_count": offset, "valid_prefix_sample_count": index,
        "censored_sample_count": offset - index,
        "original_last_sample_time_s": capture["_native_source_batches"][-1]["end_time_s"],
        "observed_valid_prefix_peak_g": peak, "confirmation_sample": capture["detector"]["confirmation_sample"],
        "confirmation_sample_index": capture["detector"]["confirmation_sample_index"],
        "confirmation_time_s": capture["detector"]["confirmation_time_s"]}
    capture.update(samples=samples, response_censoring=summary)
    record.update(impact_peak_g=None, response_censored=True, observed_valid_prefix_peak_g=peak)
    record.update(confirmation_time_s=capture["detector"]["confirmation_time_s"],
                  crossing_time_s=capture["detector"]["crossing_time_s"], onset_time_s=capture["detector"]["onset_time_s"])
    capture["detector"].update(impact_peak_g=None, response_censored=True, observed_valid_prefix_peak_g=peak)
    summary = validate_filtered_capture_prefix(capture, record)
    record.update(response_censoring=summary, decision_prefix_validated=True)
    return capture, record


from harness import plugin as native_plugin


class HistoricalCensoredPrefixTests(unittest.TestCase):
    def test_actual_valid_confirmation_before_later_same_packet_clip_is_accepted(self):
        capture, record = historical_censored_capture()
        self.assertEqual(record["result"], "impact")
        self.assertIsNone(record["impact_peak_g"])
        self.assertTrue(validate_censored_peak_record(record))
        self.assertFalse(capture["raw_integrity"]["raw_samples_validated"])
        self.assertLess(record["response_censoring"]["confirmation_sample_index"], len(capture["samples"]))

    def test_clip_before_or_at_would_be_confirmation_still_aborts(self):
        for position in (0, 1):
            capture, record = historical_censored_capture(clip_position=position)
            self.assertEqual(record["result"], "invalid_data")
            self.assertEqual(record["sample_fault"]["cause"], "native_clip_sentinel")
            with self.assertRaises(ValueError):
                validate_filtered_capture_prefix(capture, record)

    def test_same_and_next_packet_saturation_have_equivalent_prefix_decision(self):
        first, a = historical_censored_capture()
        second, b = historical_censored_capture(split=4)
        self.assertEqual(a["trigger_confirmation_time_s"], b["trigger_confirmation_time_s"])
        self.assertEqual(a["response_censoring"]["valid_prefix_sample_count"], b["response_censoring"]["valid_prefix_sample_count"])
        self.assertEqual(a["observed_valid_prefix_peak_g"], b["observed_valid_prefix_peak_g"])

    def test_sentinel_and_actual_rate_synthetic_prefix_use_same_strict_order(self):
        for value in (32767,):
            for straddle in (0, 2):
                capture, record = historical_censored_capture(clip_value=value, arm_straddle=straddle,
                                                          clock_hz=32000000., sample_period=.000302)
                self.assertEqual(record["result"], "impact")
                self.assertEqual(record["response_censoring"]["first_invalid_sample"][2], 4)
                self.assertEqual(capture["detector"]["first_armed_sample"][2], straddle)
                self.assertTrue(validate_censored_peak_record(record))

    def test_signed_permuted_axes_and_clock_remap_preserve_original_confirmation(self):
        for shift in (-.001, .00002):
            for split in (None, 4):
                capture, record = historical_censored_capture(clock_shift=shift, split=split,
                                                          axes_map=((2, -1), (0, -1), (1, 1)))
                result = compare_filter_capture(capture, 1., 1.)
                self.assertTrue(result["filtered_decision"]["decision_observed"])
                self.assertEqual(result["filtered_decision"]["confirmation_time_s"], record["confirmation_time_s"])
                self.assertEqual(result["filtered_decision"]["original_confirmation_time_s"], record["trigger_confirmation_time_s"])

    def test_negative_remap_cannot_turn_after_stop_confirmation_into_observed_shadow(self):
        capture, _record = historical_censored_capture(clock_shift=-.001)
        original_confirmation = capture["detector"]["trigger_confirmation_time_s"]
        capture["detector"]["stop_request_interval_s"] = [original_confirmation - .0001, original_confirmation]
        result = compare_filter_capture(capture, 1., 1.)
        self.assertFalse(result["filtered_decision"]["decision_observed"])
        self.assertTrue(result["filtered_decision"]["censored"])

    def test_forged_proof_reference_identity_and_arm_anchor_fail_closed(self):
        capture, record = historical_censored_capture()
        for mutate in (
                lambda c, r: c["detector"].update(confirmation_sample_index=c["detector"]["confirmation_sample_index"] + 1),
                lambda c, r: c["response_censoring"].update(first_invalid_sample=[1, 1, 0, 2]),
                lambda c, r: c["detector"]["filter_reference"]["weights_xyz"].__setitem__(0, .75),
                lambda c, r: c.update(baseline_g=(1., 0., 1.)),
                lambda c, r: r.update(axis="y"),
                lambda c, r: r.update(workflow_threshold_g=2.),
                lambda c, r: r.update(impact_peak_g=r["observed_valid_prefix_peak_g"]),
                lambda c, r: c["detector"].update(first_armed_sample=c["detector"]["crossing_sample"],
                                                  first_armed_sample_index=c["detector"]["first_armed_sample_index"] + 1),
                lambda c, r: c["response_censoring"].update(version=True)):
            current, current_record = copy.deepcopy(capture), copy.deepcopy(record)
            mutate(current, current_record)
            with self.assertRaises(ValueError):
                validate_filtered_capture_prefix(current, current_record)

    def test_unrelated_tail_faults_after_saturation_are_not_hidden(self):
        for failure in ("nan", "malformed", "time", "stale"):
            capture, record = historical_censored_capture()
            source = capture["_native_source_batches"][-1]
            if failure == "nan":
                import struct
                raw = [value for row in struct.iter_unpack("<hhh", source["data"]) for value in row]
                raw[-3] = float("nan")
                source["data"] = tuple(raw)
            elif failure == "malformed": source["data"] += b"0"
            elif failure == "time": source["start_time_s"] = source["end_time_s"]
            else: source["received_time_s"] += 1.
            with self.assertRaises(ValueError):
                validate_filtered_capture_prefix(capture, record)

    def test_later_packets_cannot_update_peak_or_resume_the_censored_filter(self):
        capture, record = historical_censored_capture(packet_size=16, split=8, post_clip_g=12.)
        self.assertIsNone(record["impact_peak_g"])
        self.assertAlmostEqual(record["observed_valid_prefix_peak_g"], 3.)
        self.assertEqual(record["response_censoring"]["censored_sample_count"], 12)
        result = compare_filter_capture(capture, 1., 1.)
        self.assertEqual(len(result["filtered_trace"]), record["response_censoring"]["valid_prefix_sample_count"])

    def test_status_publication_reconciles_censor_that_arrives_while_record_is_built(self):
        from test_sample_fault_diagnostics import HomingFaultDiagnosticsTests
        obj, attempt, _sent = HomingFaultDiagnosticsTests().machine()
        attempt.onset_batch = (1001, 7, 0, 8)
        attempt.onset_sample = (1001, 7, 0, 8)
        attempt.reason = "impact"
        def during_record(_attempt):
            with obj.lock:
                attempt.impact_peak = 3. / attempt.count_g
                attempt.response_censoring = {"first_invalid_sample": (1001, 7, 4, 8)}
            return 1.001
        with mock.patch.object(obj, "onset_time", side_effect=during_record):
            obj._record(attempt)
        snapshot = obj.get_status(obj.reactor.now)["last"]["x"]
        self.assertTrue(snapshot["response_censored"])
        self.assertIsNone(snapshot["impact_peak_g"])
        self.assertEqual(snapshot["observed_valid_prefix_peak_g"], 3.)


class CensoredWorkflowRig(FilteredRig):
    def sensor_context(self):
        context = super().sensor_context()
        context.update(backend="beacon_raw", count_g=.0078125)
        return context

    def __init__(self, options):
        super().__init__(options)
        self.make_bad_tail = False

    def stop_capture(self):
        existing = super().stop_capture()
        source = existing.get("_record")
        if source is None:
            return existing
        captured, native_record = historical_censored_capture(axis=source["axis"], time_offset=self.capture_stops * 10.,
                                                         filter_config=source.get("detector_config"))
        protected = {key: source[key] for key in ("start_position_mm", "expected_travel_mm") if key in source}
        source.update(native_record)
        source.update(protected)
        captured["_record"] = source
        if self.make_bad_tail:
            captured["_native_source_batches"][-1]["received_time_s"] += 1.
        return captured


class CensoredWorkflowTests(unittest.TestCase):
    def test_complete_filtered_calibration_saves_only_qualified_censored_null_peaks(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = CensoredWorkflowRig({"charts_directory": directory})
            rig.calibrate(save=0, DETECTOR="FILTERED")
            self.assertEqual(rig.workflow.last_result["result"], "calibrated")
            import json
            metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "x_metadata")])
            self.assertEqual(metadata["impact_peak_g"], [None] * 10)
            self.assertEqual(metadata["response_censored"], [True] * 10)
            self.assertEqual([r["threshold"] for r in rig.home_calls], [1.] * 11)
            self.assertEqual(rig.workflow.last_result["saved_threshold_g"], 1.5)

    def test_complete_filtered_accuracy_accepts_censored_response_without_invented_peak(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = CensoredWorkflowRig({"charts_directory": directory})
            rig.accuracy(DETECTOR="FILTERED", FILTERED_THRESHOLD=1.)
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            self.assertEqual(len(result["axis_results"]["x"]["trials"]), 2)
            self.assertTrue(all(r["impact_peak_g"] is None and r["decision_prefix_validated"]
                                for r in result["axis_results"]["x"]["trials"]))

    def test_unrelated_tail_fault_aborts_without_png_or_model(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = CensoredWorkflowRig({"charts_directory": directory})
            rig.make_bad_tail = True
            with self.assertRaises(Exception):
                rig.accuracy(DETECTOR="FILTERED", FILTERED_THRESHOLD=1.)
            self.assertEqual(list(Path(directory).glob("*.png")), [])
            self.assertEqual(rig.configfile.saved, {})


if __name__ == "__main__":
    unittest.main()
