"""Explicit selected-sensor orientation; synthetic drivers only, no hardware."""
import configparser
import copy
import itertools
import types
import unittest

from harness import (CommandError, FakeConfig, FakePrinter, FakeToolhead,
                     acquire_reference, make_beacon, plugin)
from accel_homing_axes import CONFIGURED_METHOD
from test_backends import BatchHelper
from test_mapped_detector import deliver
from test_session import FakeRail, info, with_endstops


class LoadedSensorSection:
    """The ConfigWrapper presence/default and note_valid contract."""
    def __init__(self, parser, name, accessed, reads):
        self.parser, self.name = parser, name
        self.accessed, self.reads = accessed, reads

    def get(self, option, default=None, note_valid=True):
        self.reads.append((self.name, option, note_valid))
        if not self.parser.has_option(self.name, option):
            return default
        value = self.parser.get(self.name, option)
        if note_valid:
            self.accessed[(self.name, option)] = value
        return value


class LoadedConfig(FakeConfig):
    def __init__(self, printer, selected, parser):
        super().__init__(printer, {"accel_chip": selected, "threshold": 1.},
                         sections=parser.sections(), name="accel_homing")
        self.parser, self.reads = parser, []
        # Drivers record typed defaults even when absent in the raw parser.
        option = "accel_axes_map" if selected.split()[0] == "beacon" else "axes_map"
        self.accessed = {(selected, option): ("x", "y", "z")}

    def getsection(self, name):
        return LoadedSensorSection(self.parser, name, self.accessed, self.reads)


def machine(selected="beacon", declaration=None, tokens=("x", "y", "z"), other_sections=""):
    native = selected.split()[0] == "beacon"
    indexes = [("xyz".index(token[-1]), -1 if token.startswith("-") else 1) for token in tokens]
    beacon = make_beacon(indexes if native else ((0, 1), (1, 1), (2, 1)))
    printer = FakePrinter(FakeToolhead(), beacon)
    if native:
        printer.objects[selected] = beacon
    else:
        printer.objects[selected] = types.SimpleNamespace(
            data_rate=3200, axes_map=[(index, sign * (123. + row))
                                     for row, (index, sign) in enumerate(indexes)], batch_bulk=BatchHelper())
    option = "accel_axes_map" if native else "axes_map"
    parser = configparser.RawConfigParser()
    parser.read_string("[accel_homing]\naccel_chip: %s\n%s" % (selected, other_sections))
    # ConfigWrapper sees this same merged parser whether the selected section
    # appeared in printer.cfg or was loaded from a nested include.
    parser.read_string("[%s]\n%s" % (selected, "" if declaration is None else option + ": " + declaration))
    config = LoadedConfig(printer, selected, parser)
    owner = plugin.BeaconAccelHoming(config)
    owner.fake_printer = printer
    printer.send_event("klippy:connect")
    owner.session_scale = owner.scale
    owner.full_scale_g = owner._full_scale_g(owner.scale)
    config.reads.clear()
    return owner, config


class ConfiguredAxisMappingTests(unittest.TestCase):
    def test_explicit_identity_counts_but_runtime_identity_defaults_do_not(self):
        for selected in ("beacon", "adxl345 toolhead"):
            owner, config = machine(selected)
            self.assertIsNone(owner.configured_axis_mapping("x"))
            self.assertIsNone(owner.configured_axis_mapping("y"))
            owner, config = machine(selected, "x,y,z")
            for axis in "xy":
                mapping = owner.configured_axis_mapping(axis)
                self.assertEqual(mapping["method"], CONFIGURED_METHOD)
                self.assertEqual((mapping["source_axis"], mapping["sign"]), ("xy".index(axis), 1))
                self.assertEqual(mapping["configuration"]["sensor_section"], selected)
                self.assertNotIn("confidence", mapping)
                self.assertNotIn("passes", mapping)

    def test_all_signed_permutations_keep_configured_stream_identity(self):
        for native, order, signs in itertools.product((False, True), itertools.permutations("xyz"),
                                                       itertools.product((-1, 1), repeat=3)):
            tokens = [axis if sign > 0 else "-" + axis for axis, sign in zip(order, signs)]
            selected = "beacon secondary" if native else "lis2dw secondary"
            with self.subTest(selected=selected, tokens=tokens):
                owner, _config = machine(selected, ", ".join(tokens), tokens)
                for axis in "xy":
                    mapping = owner.configured_axis_mapping(axis)
                    self.assertEqual((mapping["source_axis"], mapping["sign"]), ("xy".index(axis), 1))
                    self.assertEqual(mapping["configuration"]["axes_map"], tokens)

    def test_other_sensor_option_and_nonidentity_runtime_do_not_count(self):
        for selected, other in (("beacon secondary", "[beacon]\naccel_axes_map: y,-x,z\n"),
                                ("adxl345 toolhead", "[adxl345 bed]\naxes_map: y,-x,z\n")):
            owner, config = machine(selected, tokens=("y", "-x", "z"), other_sections=other)
            self.assertIsNone(owner.configured_axis_mapping("x"))
            self.assertTrue(all(section == selected for section, _option, _valid in config.reads))

    def test_merged_named_section_normalizes_whitespace_without_overwriting_typed_settings(self):
        owner, config = machine("beacon secondary", " y, -x, z ", ("y", "-x", "z"))
        before = dict(config.accessed)
        context = owner.noise_sensor_context()
        mapping = owner.configured_axis_mapping("x")
        self.assertEqual(mapping["configuration"], {"sensor_section": "beacon secondary",
                         "option": "accel_axes_map", "axes_map": ["y", "-x", "z"]})
        self.assertEqual(config.reads, [("beacon secondary", "accel_axes_map", False)])
        self.assertEqual(config.accessed, before)
        self.assertEqual(owner.noise_sensor_context(), context)
        mapping["configuration"]["axes_map"][0] = "z"
        self.assertEqual(owner.configured_axis_mapping("x")["configuration"]["axes_map"], ["y", "-x", "z"])

    def test_invalid_explicit_declaration_never_falls_back_to_automatic(self):
        for selected, value in itertools.product(("beacon", "adxl345 toolhead"),
                                                  ("", "x,x,z", "x,y", "x,y,z,x", "x,+y,z", "X,y,z")):
            with self.subTest(selected=selected, value=value):
                owner, _config = machine(selected, value)
                with self.assertRaisesRegex(CommandError, "invalid explicit"):
                    owner.configured_axis_mapping("x")
                self.assertEqual(owner.toolhead.moves, [])

    def test_runtime_index_or_sign_mismatch_and_invalid_scale_are_rejected(self):
        for selected in ("beacon", "adxl345 toolhead"):
            for changed in (((1, 1), (0, 1), (2, 1)), ((0, -1), (1, 1), (2, 1)),
                            ((0, 0), (1, 1), (2, 1)), ((0, float("nan")), (1, 1), (2, 1))):
                owner, _config = machine(selected, "x,y,z")
                if owner.backend is None:
                    owner.helper.config.axes_map = changed
                else:
                    owner.backend.sensor.axes_map = changed
                with self.subTest(selected=selected, changed=changed), self.assertRaises(CommandError):
                    owner.configured_axis_mapping("x")
        owner, _config = machine("adxl345 toolhead", "x,y,z")
        del owner.backend.sensor.axes_map
        with self.assertRaisesRegex(CommandError, "valid axes map"):
            owner.configured_axis_mapping("x")

    def test_declaration_removal_or_change_is_visible_to_workflow_validation(self):
        owner, config = machine("beacon secondary", "y,-x,z", ("y", "-x", "z"))
        original = owner.configured_axis_mapping("x")
        config.parser.set("beacon secondary", "accel_axes_map", "x,y,z")
        with self.assertRaisesRegex(CommandError, "disagrees"):
            owner.configured_axis_mapping("x")
        config.parser.remove_option("beacon secondary", "accel_axes_map")
        self.assertIsNone(owner.configured_axis_mapping("x"))
        self.assertEqual(original["configuration"]["axes_map"], ["y", "-x", "z"])

    def test_native_and_generic_detectors_apply_explicit_orientation_exactly_once(self):
        for selected, axis in itertools.product(("beacon", "adxl345 toolhead"), "xy"):
            owner, _config = machine(selected, "y,-x,z", ("y", "-x", "z"))
            owner.workflows.detection_axis_mapping = owner.configured_axis_mapping
            owner.workflows.simple_detection = lambda selected_axis: True
            endstop = with_endstops(owner, (axis,))[axis]
            owner.session_axes = {axis: FakeRail(endstop, info(retract=0.))}
            owner.homing_active = True
            owner.reactor.now = 10.
            if owner.backend is not None:
                owner._start_stream()
            acquire_reference(owner, endstop)
            when = (owner.backend.last_sample_time + 1. / owner.backend.rate if owner.backend is not None
                    else owner.reactor.now + .001)
            attempt = owner.prepare(endstop, when)
            owner.arm(attempt)
            if owner.backend is None:
                expected = (1, 1) if axis == "x" else (0, -1)
            else:
                expected = ("xy".index(axis), 1)
            self.assertEqual((attempt.index, attempt.map_sign), expected)
            profile = copy.deepcopy(attempt.axis_mapping)
            noise = tuple(.1 if index == expected[0] else 5. for index in range(3))
            deliver(owner, attempt, [noise] * 8)
            self.assertFalse(attempt.done)
            hit = tuple(-1.5 if index == expected[0] else .1 for index in range(3))
            deliver(owner, attempt, [hit] * 8, offset=.0025 if owner.backend is not None else .008)
            self.assertEqual(attempt.reason, "impact")
            self.assertEqual(attempt.axis_mapping, profile)
            self.assertEqual(attempt.axis_mapping["source_axis"], "xy".index(axis))


if __name__ == "__main__":
    unittest.main()
