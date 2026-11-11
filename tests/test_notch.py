"""Causal notch signal, lifecycle, provenance and upgrade regressions; offline only."""
import importlib
import csv
import gzip
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, ConfigError, FakeEndstop, arm_axis, batch, counts, make_homing, plugin, stationary_stream
from accel_homing_filter import CausalNotch, DISABLED, coefficients, profile, replay
from accel_homing_charts import _capture_data, _prepared_capture, prepare_capture
from accel_homing_model import AccelHomingModel
from test_absolute_signal import native_baseline
from test_backends import arm, baseline, feed, generic, G
from test_session import FakeRail, info
from test_workflows import Config, GCmd, Rig, workflow_module
from test_accuracy_workflow import AccuracyRig
import accel_homing_troubleshoot as diagnostics
import test_troubleshoot_imports as upgrade_tests


def oracle(rows, settings, rate):
    """Independent direct-form-I recurrence, one channel at a time."""
    b0, b1, b2, a1, a2 = coefficients(settings, rate)
    result = [[0.0] * 3 for _ in rows]
    for axis in range(3):
        x1 = x2 = y1 = y2 = 0.0
        for index, row in enumerate(rows):
            value = row[axis]
            output = b0 * value + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
            result[index][axis] = output
            x2, x1, y2, y1 = x1, value, y1, output
    return result


class NotchTests(unittest.TestCase):
    def test_frequency_response_signed_channels_and_causal_impulse(self):
        settings, rate = profile(100., 10.), 3200.
        for frequency, lower, upper in ((100., 0., .0001), (20., .995, 1.005), (500., .995, 1.005)):
            rows = [(math.sin(2 * math.pi * frequency * n / rate), 0., 0.) for n in range(8000)]
            filt = CausalNotch(settings, rate)
            actual = [filt.process(row)[0] for row in rows]
            ratio = math.sqrt(sum(v * v for v in actual[-3200:]) / sum(row[0] ** 2 for row in rows[-3200:]))
            self.assertTrue(lower <= ratio <= upper, (frequency, ratio))
        rows = [(0., 0., 0.)] * 12 + [(1., -2., 3.)] + [(0., 0., 0.)] * 100
        filt = CausalNotch(settings, rate)
        actual = [filt.process(row) for row in rows]
        expected = oracle(rows, settings, rate)
        self.assertEqual(actual[:12], [(0., 0., 0.)] * 12)
        self.assertGreater(actual[12][0], 0.9)  # No look-ahead or confirmation interval.
        for observed, reference in zip(actual, expected):
            for a, b in zip(observed, reference):
                self.assertAlmostEqual(a, b, places=12)

    def native_run(self, widths, single=False, disabled=False):
        obj = make_homing(threshold=.75, options={"y_notch_frequency": 100.})
        base = (32, -64, 128)
        native_baseline(obj, base)
        obj.start_capture()
        attempt, sent = arm_axis(obj, "y", 207 if single else 200, 2000)
        attempt.vector_mode = True
        if not disabled:
            attempt.notch = CausalNotch(obj.filter_settings("y"), 1000.)
        if single:
            rows = [base] * 7 + [(32, -64, 128 + counts(2.))]
        else:
            rows = [(32 + counts(min(1., n / 250.) * math.sin(2 * math.pi * 100 * n / 1000.)), -64, 128)
                    for n in range(352)]
            rows[320] = (32, -64, 128 + counts(3.))
        offset = 0
        for width in widths:
            part = rows[offset:offset + width]
            if not part:
                break
            obj.reactor.now = (200 + offset + len(part)) / 1000.
            obj._inspect(batch(200 + offset, part, delta_clock=len(part) - 1))
            offset += len(part)
        obj.disarm()
        capture = obj.stop_capture()
        return obj, attempt, sent, capture, rows

    def test_native_packet_split_onset_tail_reset_and_single_armed_sample_capture(self):
        outcomes = []
        for widths in ([8] * 44, [16] * 22):
            obj, attempt, sent, capture, rows = self.native_run(widths)
            self.assertEqual(attempt.reason, "impact")
            self.assertEqual(len(sent.messages), 1)
            self.assertEqual(attempt.record["onset_time_s"], .520)
            self.assertEqual(capture["samples"][0][1], 32 * attempt.count_g)
            expected = oracle([tuple(row[i] - (32, -64, 128)[i] for i in range(3)) for row in rows],
                              profile(100., 10.), 1000.)
            first = next(i for i, row in enumerate(expected) if math.sqrt(sum(v * v for v in row)) >= attempt.limit_raw)
            self.assertEqual(first, 320)
            for observed, wanted in zip(capture["filtered_samples"], expected):
                for a, b in zip(observed[1:], wanted):
                    self.assertAlmostEqual(a, b * attempt.count_g, places=11)
            self.assertAlmostEqual(attempt.onset_raw, math.sqrt(sum(v * v for v in expected[first])), places=10)
            outcomes.append(capture["filtered_samples"])
        for first, second in zip(*outcomes):
            self.assertAlmostEqual(first[0], second[0], places=14)
            self.assertEqual(first[1:], second[1:])
        _, unfiltered, _, _, _ = self.native_run([8] * 44, disabled=True)
        self.assertLess(unfiltered.record["onset_time_s"], .520)
        # Keep the exact old disabled arithmetic at a one-ulp threshold edge.
        obj = make_homing()
        attempt, sent = arm_axis(obj, "x", 200, 1000)
        attempt.vector_mode = True
        attempt.bases = (0., 0., 0.)
        attempt.limit_raw = 880.4813247114332
        xyz = (-425.6749355421921, -428.6721490263683, -640.5375875511647)
        params = {"start_clock": 200, "delta_clock": 1, "data": xyz * 2}
        obj._inspect_vector(attempt, params, xyz * 2, 2, params, 0)
        self.assertEqual(sent.messages, [])
        _, reset, _, replayed, _ = self.native_run([8] * 44)
        self.assertEqual(outcomes[0], replayed["filtered_samples"])
        obj, attempt, _, capture, _ = self.native_run([8], single=True)
        self.assertEqual(len(capture["filtered_samples"]), 1)
        self.assertEqual(capture["signal_filter"]["first_sample_index"], 7)
        self.assertIsNone(capture["fault"])
        trace, metadata = _capture_data(capture, attempt.record)
        self.assertEqual(len(trace), 1)
        _prepared_capture(prepare_capture("a" * 32, "y", capture, attempt.record))

    def test_every_configured_sensor_contract_filters_xyz_before_norm(self):
        for name in ("adxl345 toolhead", "lis2dw", "lis3dh", "mpu9250", "icm20948", "custom_accel"):
            for legacy in (False, True):
                with self.subTest(sensor=name, legacy=legacy):
                    options = {"y_notch_frequency": 100., "y_threshold": .75}
                    if name == "custom_accel":
                        options["sensor_range"] = 8.
                    obj = generic(name, options, legacy=legacy)
                    baseline(obj)
                    obj.start_capture()
                    attempt, sent = arm(obj, "y")
                    attempt.vector_mode = obj._uses_vector_detector("y")
                    attempt.notch = CausalNotch(obj.filter_settings("y"), 3200.)
                    rows = [(10.06 + n / 3200., min(1., n / 400.) * math.sin(2 * math.pi * 100 * n / 3200.) * G, 0., G)
                            for n in range(640)]
                    rows[608] = (rows[608][0], 0., 0., -2 * G)
                    for start in range(0, len(rows), 16):
                        feed(obj, rows[start:start + 16])
                    obj.disarm()
                    captured = obj.stop_capture()
                    self.assertEqual(attempt.reason, "impact")
                    self.assertEqual(len(sent.messages), 1)
                    self.assertAlmostEqual(obj.onset_time(attempt), rows[608][0], places=6)
                    index = captured["detector"]["onset_sample_index"]
                    filtered = captured["filtered_samples"][index]
                    self.assertLess(filtered[3], 0.)
                    self.assertAlmostEqual(math.sqrt(sum(v * v for v in filtered[1:])),
                                           attempt.onset_raw * attempt.count_g, places=11)
                    self.assertEqual(obj.filter_settings("x"), DISABLED)
                    self.assertFalse(obj._uses_vector_detector("x"))

    def test_config_reference_rate_reset_and_raw_liveness_guards(self):
        invalid = ({"notch_frequency": float("nan")}, {"notch_frequency": -1},
                   {"notch_q": float("inf")}, {"x_notch_q": .49}, {"y_notch_q": 1001},
                   {"notch_frequency": 1600.}, {"notch_frequency": 1e-300})
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ConfigError):
                generic(options=options)
        obj = make_homing(options={"y_notch_frequency": 100.})
        obj.reactor.now = 10.
        endstop = FakeEndstop("y")
        endstop.get_steppers = lambda: []
        obj.session_axes = {"y": FakeRail(endstop, info(speed=100., retract=0.))}
        stationary_stream(obj)
        obj._prepare_reference()
        one = obj.prepare(endstop, obj.preparation.last + .001)
        one.notch.process((10., 20., 30.))
        obj._prepare_reference()
        two = obj.prepare(endstop, obj.preparation.last + .001)
        self.assertAlmostEqual(two.notch.rate, 1000.)
        self.assertEqual(two.notch.state, [[0., 0.]] * 3)
        # AUTO must retain the exact armed source row even when only the last
        # row of a native packet is eligible and clocksync later shifts time.
        two.notch = None
        two.limit_raw = counts(.5)
        obj.start_capture()
        obj.arm(two)
        obj.reactor.now = two.arm_time + .001
        obj._inspect(batch(two.arm_clock - 7, [(0, 0, counts(1.))] * 7
                           + [(0, 0, counts(3.))]))
        obj.disarm()
        clock_to_time = obj.clock_to_time
        obj.clock_to_time = lambda clock: clock_to_time(clock) - .000050
        captured = obj.stop_capture()
        provenance = captured["auto_notch_replay"]
        self.assertTrue(provenance["complete"])
        self.assertEqual(provenance["first_sample_index"], 7)
        self.assertEqual(provenance["first_sample_matches"], 1)
        self.assertEqual(provenance["first_sample"], (two.arm_clock - 7, 7, 7, 8))
        self.assertEqual(provenance["arm_clock"], two.arm_clock)
        self.assertEqual(provenance["eligibility_source"], "native_sensor_clock")
        self.assertEqual(provenance["first_sample_time_s"], captured["samples"][7][0])
        self.assertAlmostEqual(provenance["sample_rate_hz"], 1000.)
        for failure in ("changed_rate", "one_missing_sample", "frozen", "clocksync"):
            obj = make_homing(threshold=8., options={"notch_frequency": 100.})
            native_baseline(obj, (0, 0, 128))
            attempt, _ = arm_axis(obj, "x", 200, 2000)
            attempt.vector_mode = True
            attempt.notch = CausalNotch(obj.filter_settings("x"), 1000.)
            obj.reactor.now = .208
            obj._inspect(batch(200, [(counts(.5), 0, 128)] * 8))
            if failure == "changed_rate":
                obj.reactor.now = .224
                obj._inspect(batch(208, [(counts(.5), 0, 128)] * 8, delta_clock=14))
            elif failure == "one_missing_sample":
                obj.reactor.now = .217
                obj._inspect(batch(209, [(counts(.5), 0, 128)] * 8))
            elif failure == "clocksync":
                clock_to_time = obj.clock_to_time
                obj.clock_to_time = lambda clock: clock_to_time(clock) + .000050
                obj.reactor.now = .216
                obj._inspect(batch(208, [(counts(.5), 0, 128)] * 8))
                self.assertIsNone(attempt.reason)
                continue
            else:
                for start in (208, 216, 224, 232, 240):
                    obj.reactor.now = (start + 8) / 1000.
                    obj._inspect(batch(start, [(counts(.5), 0, 128)] * 8))
                self.assertEqual(attempt.vector_unchanged_since, .200)
                self.assertLess(attempt.filter_last_time - attempt.vector_unchanged_since, .050)
                with mock.patch.object(obj, "_commanded_speed", return_value=100.):
                    obj._watchdog(obj.reactor.now)
                self.assertEqual(attempt.reason, "frozen")
                continue
            self.assertEqual(attempt.reason, "filter_rate")

    def test_model_filter_identity_schema_and_recalibration_access(self):
        settings = profile(100., 10.)
        rig = Rig()
        metadata = dict(rig.workflow._model_context("y"), impact_metric="xyz_magnitude",
                        signal_filter=settings, filter_sample_rate_hz=3200.)
        options = {"version": 2, "y_threshold": 2., "y_successes": 10, "y_metadata": json.dumps(metadata)}
        model = AccelHomingModel(Config(None, options, "accel_homing_model test"))
        rig.workflow.model = model
        rig.owner.filter_settings = lambda axis: dict(DISABLED)
        rig.workflow._validate_model_connection()  # Restart stays usable for calibration.
        with self.assertRaisesRegex(CommandError, "different signal filter"):
            rig.workflow.check_home_axes(["y"])
        rig.workflow.session = {"mode": "calibration"}
        rig.workflow.in_home, rig.workflow.home_axis = True, "y"
        rig.workflow.check_home_axes(["y"])
        rig.workflow.session = None
        rig.owner.filter_settings = lambda axis: settings if axis == "y" else dict(DISABLED)
        rig.workflow.check_filter_rate("y", 3200.)
        with self.assertRaisesRegex(CommandError, "different sensor sample rate"):
            rig.workflow.check_filter_rate("y", 3000.)
        with self.assertRaisesRegex(CommandError, "does not support notch"):
            rig.workflow.cmd_BETA_CALIBRATE(GCmd(AXIS="Y", CENTERED=1))
        self.assertEqual(rig.toolhead.moves, [])
        rig.workflow.manual_axes.add("y")
        rig.workflow.check_filter_rate("y", 3000.)
        for malformed in (dict(options, version=1), dict(options, y_metadata=json.dumps(dict(metadata, filter_sample_rate_hz=100.))),
                          dict(options, y_metadata=json.dumps(dict(metadata, signal_filter={"type": "unknown"})))):
            with self.assertRaises(ConfigError):
                AccelHomingModel(Config(None, malformed, "accel_homing_model test"))

    def test_git_only_update_loads_helper_from_checkout_with_old_links(self):
        fixture = upgrade_tests.TroubleshootImportTests()
        for entrypoint in ("accel_homing", "beacon_accel_homing", "accel_homing_model"):
            with self.subTest(entrypoint=entrypoint), fixture.installation() as (checkout, extras):
                before = list(sys.path)
                importlib.import_module("extras." + entrypoint)
                helper = sys.modules["extras.accel_homing_filter"]
                self.assertEqual(Path(helper.__file__), checkout / "accel_homing_filter.py")
                self.assertFalse((extras / "accel_homing_filter.py").exists())
                self.assertEqual(sys.path, before)

    def test_filtered_calibration_staging_preserves_mixed_model_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = Rig({"charts_directory": directory})
            selected = {"x": dict(DISABLED), "y": profile(100., 10.)}
            rig.owner.filter_settings = lambda axis: selected[axis]
            filtered = CausalNotch(selected["y"], 3200.).metadata()
            rig.home_results.extend({"signal_filter": filtered} for _ in range(11))
            with mock.patch.object(workflow_module, "publish_noise", return_value={}):
                rig.calibrate("Y", save=0)
                self.assertEqual(rig.configfile.saved[("accel_homing_model default", "version")], "2")
                y_metadata = rig.configfile.saved[("accel_homing_model default", "y_metadata")]
                self.assertEqual(json.loads(y_metadata)["filter_sample_rate_hz"], 3200.)
                rig.workflow._disable_xy()
                rig.idle_state = "Ready"
                rig.calibrate("X", save=0)
                self.assertEqual(rig.configfile.saved[("accel_homing_model default", "version")], "2")
                self.assertEqual(rig.configfile.saved[("accel_homing_model default", "y_metadata")], y_metadata)
                selected["y"] = dict(DISABLED)
                rig.workflow._disable_xy()
                rig.idle_state = "Ready"
                rig.calibrate("Y", save=0)
                self.assertEqual(rig.configfile.saved[("accel_homing_model default", "version")], "1")

    def test_troubleshoot_exports_raw_and_filtered_same_indices_with_replay_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AccuracyRig({"charts_directory": directory, "x_threshold": 1.})
            settings = profile(100., 10.)
            rig.owner.filter_settings = lambda axis: settings if axis == "x" else dict(DISABLED)
            def filtered_capture(capture):
                metadata = CausalNotch(settings, 1000.).metadata()
                capture["detector"]["signal_filter"] = dict(metadata)
                capture["detector"]["impact_metric"] = "xyz_magnitude"
                capture["_record"].update(signal_filter=dict(metadata), impact_metric="xyz_magnitude")
                metadata.update(first_sample_index=1, first_sample_matches=1,
                                first_sample=(0, 3, 1, 4))
                capture["signal_filter"] = metadata
                capture["filtered_samples"] = replay(capture["samples"], capture["baseline_g"], metadata, 1)
            rig.capture_mutator = filtered_capture
            with mock.patch.object(workflow_module, "TroubleshootRecorder", diagnostics.Recorder), \
                    mock.patch.object(diagnostics, "render_bundle"):
                rig.workflow.cmd_TROUBLESHOOT(GCmd(AXIS="X", SAMPLES=2, CENTERED=1))
            manifest = Path(rig.workflow.last_result["artifacts"]["manifest"])
            report = json.loads(manifest.read_text(encoding="utf-8"))
            home = report["homes"][0]
            with gzip.open(manifest.parent / home["raw_file"], "rt", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["filtered_x_g"], "")
            self.assertEqual(float(rows[2]["x_g"]), -4.)
            self.assertLess(float(rows[2]["filtered_x_g"]), 0.)
            self.assertEqual([int(row["sample_index"]) for row in rows], list(range(4)))
            self.assertTrue(home["capture"]["filtered_replay_complete"])
            self.assertEqual(home["capture"]["signal_filter"]["first_sample_index"], 1)


if __name__ == "__main__":
    unittest.main()
