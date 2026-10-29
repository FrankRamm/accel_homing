"""Manual threshold parsing and range guards with synthetic sensor objects only."""
import struct
import types
import unittest
from unittest import mock

from harness import (CommandError, ConfigError, FakeConfig, FakePrinter, FakeToolhead,
                     MG_PER_COUNT_16G, make_beacon, plugin)
from test_backends import BatchHelper
from test_session import FakeRail, with_endstops
from test_trigger import FakeGcmd


def next_positive_float(value):
    """Immediate successor of a positive finite double, including Python 3.8."""
    bits, = struct.unpack(">Q", struct.pack(">d", value))
    return struct.unpack(">d", struct.pack(">Q", bits + 1))[0]


class BoundedGcmd(FakeGcmd):
    """Honor parser limits so a stale SET min/max cannot be hidden by the fake."""
    def get_float(self, name, default=None, minval=None, maxval=None):
        value = super().get_float(name, default, minval, maxval)
        if value is not None and ((minval is not None and value < minval)
                                  or (maxval is not None and value > maxval)):
            raise self.error("%s out of range" % name)
        return value


class ManualThresholdRangeTests(unittest.TestCase):
    sections = ("accel_homing", "beacon_accel_homing")
    backends = ("generic", "beacon")
    commands = ("SET_ACCEL_HOMING", "SET_BEACON_ACCEL_HOMING")

    def create(self, backend="generic", section="accel_homing", options=None,
               range_g=16.0, known_range=True):
        beacon = make_beacon()
        helper = beacon.accel_helper
        helper._scale["scale"] = MG_PER_COUNT_16G * range_g / 16.0
        helper._scales = {"test": helper._scale}
        if not known_range:
            del helper._clip_values
        printer = FakePrinter(FakeToolhead(), beacon)
        sensor = "beacon" if backend == "beacon" else "adxl345 toolhead"
        if backend == "generic":
            printer.objects[sensor] = types.SimpleNamespace(
                data_rate=3200, axes_map=[(0, 1), (1, 1), (2, 1)], batch_bulk=BatchHelper())
        controls = {"accel_chip": sensor}
        if backend == "generic":
            controls["sensor_range"] = range_g
        controls.update(options or {})
        config = FakeConfig(printer, controls, sections=(sensor,), name=section)
        obj = plugin.BeaconAccelHoming(config)
        obj.fake_printer = printer
        printer.send_event("klippy:connect")
        return obj

    def test_config_accepts_sub_three_and_sub_calibration_manual_values(self):
        for backend in self.backends:
            for section in self.sections:
                for option in ("threshold", "x_threshold", "y_threshold"):
                    for value in (0.5, 0.25, 0.001, 0.000001, 5e-324):
                        with self.subTest(backend=backend, section=section, option=option, value=value):
                            obj = self.create(backend, section, {option: value})
                            expected = {"x": 8.0, "y": 8.0}
                            expected.update({axis: value for axis in "xy"
                                             if option in ("threshold", axis + "_threshold")})
                            self.assertEqual(obj.threshold_g, expected)

    def test_config_and_set_allow_above_fifteen_when_sensor_range_permits(self):
        for backend in self.backends:
            for section in self.sections:
                for option in ("threshold", "x_threshold", "y_threshold"):
                    with self.subTest(backend=backend, section=section, option=option):
                        obj = self.create(backend, section, {option: 20.0}, range_g=32.0)
                        axes = "xy" if option == "threshold" else option[0]
                        self.assertTrue(all(obj.threshold_g[axis] == 20.0 for axis in axes))
                        for command in self.commands:
                            for axis in "xy":
                                obj.fake_printer.commands[command](BoundedGcmd(AXIS=axis, THRESHOLD=21.0))
                                self.assertEqual(obj.threshold_g[axis], 21.0)

    def test_config_rejects_nonpositive_or_nonfinite_shared_and_axis_values(self):
        for backend in self.backends:
            for section in self.sections:
                for option in ("threshold", "x_threshold", "y_threshold"):
                    for value in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
                        with self.subTest(backend=backend, section=section, option=option, value=value):
                            with self.assertRaisesRegex(ConfigError, option + " must be finite and greater than 0 g"):
                                self.create(backend, section, {option: value})

    def test_known_sensor_range_is_inclusive_at_eighty_percent_for_config_and_set(self):
        for backend in self.backends:
            for section in self.sections:
                reference = self.create(backend, section)
                limit = reference._full_scale_g(reference.scale) * 0.8
                above = next_positive_float(limit)
                for option in ("threshold", "x_threshold", "y_threshold"):
                    with self.subTest(backend=backend, section=section, option=option):
                        obj = self.create(backend, section, {option: limit})
                        with self.assertRaisesRegex(ConfigError, "80%"):
                            self.create(backend, section, {option: above})
                        for command in self.commands:
                            for axis in "xy":
                                callback = obj.fake_printer.commands[command]
                                callback(BoundedGcmd(AXIS=axis, THRESHOLD=limit))
                                with self.assertRaisesRegex(FakeGcmd.Error, "80%"):
                                    callback(BoundedGcmd(AXIS=axis, THRESHOLD=above, IMPACT_SIGN=-1))
                                self.assertEqual(obj.threshold_g[axis], limit)
                                self.assertEqual(obj.impact_sign[axis], 0)

    def test_both_set_aliases_accept_small_values_and_keep_axes_independent(self):
        for backend in self.backends:
            for section in self.sections:
                for command in self.commands:
                    for axis in "xy":
                        obj = self.create(backend, section)
                        for value in (0.5, 0.25, 0.001, 5e-324):
                            with self.subTest(backend=backend, section=section, command=command,
                                              axis=axis, value=value):
                                obj.fake_printer.commands[command](BoundedGcmd(AXIS=axis, THRESHOLD=value))
                                self.assertEqual(obj.threshold_g[axis], value)
                                self.assertEqual(obj.threshold_g["y" if axis == "x" else "x"], 8.0)

    def test_both_set_aliases_reject_nonpositive_and_nonfinite_without_partial_changes(self):
        for backend in self.backends:
            for command in self.commands:
                for axis in "xy":
                    obj = self.create(backend)
                    for value in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
                        with self.subTest(backend=backend, command=command, axis=axis, value=value):
                            with self.assertRaisesRegex(FakeGcmd.Error, "THRESHOLD must be finite and greater than 0 g"):
                                obj.fake_printer.commands[command](BoundedGcmd(
                                    AXIS=axis, THRESHOLD=value, IMPACT_SIGN=-1))
                            self.assertEqual(obj.threshold_g, {"x": 8.0, "y": 8.0})
                            self.assertEqual(obj.impact_sign, {"x": 0, "y": 0})

    def test_generic_defaults_follow_actual_range_without_a_three_g_floor(self):
        for range_g, expected in ((2.0, 1.0), (8.0, 4.0), (16.0, 8.0), (32.0, 8.0)):
            with self.subTest(range_g=range_g):
                obj = self.create(range_g=range_g)
                self.assertEqual(obj.threshold_g, {"x": expected, "y": expected})

    def test_set_and_prepare_reject_threshold_above_narrower_active_beacon_range(self):
        for command in self.commands:
            obj = self.create("beacon", options={"threshold": 4.0})
            obj.helper._stream_en = 1
            obj.helper._scale = {"name": "8g", "id": 1, "scale": MG_PER_COUNT_16G / 2.0}
            obj._start_stream()
            with self.assertRaisesRegex(FakeGcmd.Error, "80%"):
                obj.fake_printer.commands[command](BoundedGcmd(AXIS="X", THRESHOLD=7.0))
            self.assertEqual(obj.threshold_g["x"], 4.0)
            obj.threshold_g["x"] = 7.0
            endstop = with_endstops(obj)["x"]
            obj.session_axes["x"] = FakeRail(endstop)
            obj.homing_active = True
            with self.assertRaisesRegex(CommandError, "80%"):
                obj.prepare(endstop, 10.0)
            self.assertEqual(endstop._dispatch.started, [])
            self.assertIsNone(obj.attempt)

    def test_unknown_native_range_keeps_no_arbitrary_upper_cap(self):
        obj = self.create("beacon", options={"threshold": 20.0}, known_range=False)
        self.assertIsNone(obj._full_scale_g(obj.scale))
        obj.fake_printer.commands["SET_ACCEL_HOMING"](BoundedGcmd(AXIS="X", THRESHOLD=100.0))
        self.assertEqual(obj.threshold_g, {"x": 100.0, "y": 20.0})

    def test_prepare_rejects_unrepresentable_raw_limit_before_dispatch(self):
        for threshold, scale in ((1e308, MG_PER_COUNT_16G), (5e-324, 1e308)):
            with self.subTest(threshold=threshold, scale=scale):
                obj = self.create("beacon", options={"threshold": threshold}, known_range=False)
                obj.helper._stream_en = 1
                obj.helper._scale = {"name": "external", "id": 1, "scale": scale}
                obj._start_stream()
                endstop = with_endstops(obj)["x"]
                obj.session_axes["x"] = FakeRail(endstop)
                obj.homing_active = True
                with self.assertRaisesRegex(CommandError, "finite positive trigger limit"):
                    obj.prepare(endstop, 10.0)
                self.assertEqual(endstop._dispatch.started, [])
                self.assertIsNone(obj.attempt)

    def test_vector_prepare_rejects_unrepresentable_square_before_dispatch(self):
        for threshold, scale in ((1e160, MG_PER_COUNT_16G), (.5, 1e308)):
            with self.subTest(threshold=threshold, scale=scale):
                obj = self.create("beacon", options={"threshold": threshold}, known_range=False)
                obj.helper._stream_en = 1
                obj.helper._scale = {"name": "external", "id": 1, "scale": scale}
                obj._start_stream()
                endstop = with_endstops(obj)["x"]
                obj.session_axes["x"] = FakeRail(endstop)
                obj.homing_active = True
                with mock.patch.object(obj, "_uses_vector_detector", return_value=True), \
                        self.assertRaisesRegex(CommandError, "finite positive squared XYZ trigger limit"):
                    obj.prepare(endstop, 10.0)
                self.assertEqual(endstop._dispatch.started, [])
                self.assertIsNone(obj.attempt)

    def test_set_and_status_feedback_preserve_small_positive_values(self):
        obj = self.create()
        command = BoundedGcmd(AXIS="X", THRESHOLD=0.001)
        obj.fake_printer.commands["SET_ACCEL_HOMING"](command)
        self.assertIn("threshold 0.001 g", command.replies[0])
        status = BoundedGcmd()
        obj.cmd_STATUS(status)
        self.assertIn("X 0.001 g", status.replies[0])
        self.assertEqual(obj.get_status(0.0)["threshold_g"]["x"], 0.001)


if __name__ == "__main__":
    unittest.main()
