"""Synthetic accuracy trials; no hardware, service, network or motion access."""
import copy
import contextlib
import html
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from harness import CommandError, arm_axis, batch, counts, make_homing
from test_workflows import GCmd, Rig, workflow_module


class AccuracyRig(Rig):
    """Reuse the stock-handler fixture with current per-trial sensor evidence."""

    def __init__(self, options=None):
        super().__init__(options)
        self.sensor_time = 0.0
        self.capture_mutator = None
        self.capture_directories = []
        create_directory = self.workflow._accuracy_capture_directory
        def track_directory():
            temporary = create_directory()
            self.capture_directories.append(Path(temporary.name))
            return temporary
        self.workflow._accuracy_capture_directory = track_directory

    def stock_home(self, gcmd):
        self.sensor_time += 1.0
        super().stock_home(gcmd)
        if self.home_calls:
            axis = self.home_calls[-1]["axis"]
            record = self.owner.last[axis]
            record["arm_time_s"] = self.sensor_time + 0.001
            record["onset_time_s"] = self.sensor_time + 0.002

    def stop_capture(self):
        self.capture_active = False
        self.capture_stops += 1
        if self.bad_capture is not None:
            return self.bad_capture
        axis = self.home_calls[-1]["axis"] if self.home_calls else "x"
        record = self.owner.last.get(axis)
        if record is None:
            return {"samples": [], "dropped": 0}
        t0 = self.sensor_time
        onset_sample = (round(t0 * 1000) & 0xFFFFFFFF, 3, 2, 4)
        record["onset_sample"] = onset_sample
        capture = {"samples": [(t0, 0.0, 0.0, 1.0),
                               (t0 + 0.001, 0.1, 0.0, 1.0),
                               (t0 + 0.002, -4.0, -5.0, 1.0),
                               (t0 + 0.003, 0.0, 0.0, 1.0)],
                   "dropped": 0, "baseline_g": (0.0, 0.0, 1.0), "_record": record,
                   "detector": {"axis": axis, "attempt_count": 1,
                                "arm_time_s": record["arm_time_s"],
                                "onset_time_s": record["onset_time_s"],
                                "trigger_onset_time_s": record["onset_time_s"],
                                "onset_sample_version": 1, "onset_sample": onset_sample,
                                "onset_sample_matches": 1, "onset_sample_index": 2,
                                "threshold_g": self.home_calls[-1]["threshold"],
                                "onset_g": record.get("onset_g"),
                                "impact_peak_g": record.get("impact_peak_g"),
                                "direction": record.get("direction", -1),
                                "homing_speed_mm_s": self.owner.rails_by_axis[axis].info.speed,
                                "homing_accel_mm_s2": self.owner.homing_accel}}
        if self.capture_mutator is not None:
            self.capture_mutator(capture)
        return capture

    def accuracy(self, axis="X", samples=2, **params):
        arguments = {"CENTERED": 1, "SAMPLES": samples}
        if axis is not None:
            arguments["AXIS"] = axis
        arguments.update(params)
        self.workflow.cmd_ACCURACY(GCmd(**arguments))


class AccuracyWorkflowTests(unittest.TestCase):
    def assert_restored(self, rig, thresholds=None):
        self.assertIsNone(rig.workflow.session)
        self.assertFalse(rig.workflow.in_home)
        self.assertFalse(rig.capture_active)
        self.assertEqual(rig.owner.threshold_g, thresholds or {"x": 8.0, "y": 9.0})
        self.assertEqual(rig.owner.back_off_dist, 5.0)
        self.assertEqual(rig.configfile.saved, {})
        self.assertNotIn("SAVE_CONFIG", rig.scripts)
        self.assertEqual(rig.dispatched_scripts, [])
        self.assertEqual(rig.reactor.timers[rig.workflow.timer], rig.reactor.NEVER)
        self.assertEqual(rig.toolhead.position[2:], [40.0, 6.0])
        self.assertTrue(all(coords[2] is None for coords, _speed in rig.toolhead.moves))
        self.assertTrue(all(not directory.exists() for directory in rig.capture_directories))

    def test_registration_and_invalid_parameters_have_no_effects(self):
        cases = ({"AXIS": "Z", "CENTERED": 1}, {"AXIS": "YX", "CENTERED": 1},
                 {"AXIS": "", "CENTERED": 1}, {"CENTERED": 0}, {},
                 {"CENTERED": "1.0"}, {"CENTERED": 1, "SAMPLES": 1},
                 {"CENTERED": 1, "SAMPLES": 51}, {"CENTERED": 1, "SAVE_CONFIG": 0},
                 {"CENTERED": 1, "SAVE_CONFIG": 1})
        for params in cases:
            with self.subTest(params=params):
                rig = AccuracyRig()
                self.assertIn("ACCEL_HOMING_ACCURACY", rig.gcode.ready_gcode_handlers)
                before = rig.effects()
                with self.assertRaises(CommandError):
                    rig.workflow.cmd_ACCURACY(GCmd(**params))
                self.assertEqual(before, rig.effects())

    def test_preflight_reuses_idle_unhomed_stock_policy_and_geometry_checks(self):
        setters = (lambda r: setattr(r, "print_state", "printing"),
                   lambda r: setattr(r, "paused", True),
                   lambda r: setattr(r, "sd_active", True),
                   lambda r: setattr(r, "idle_state", "Printing"),
                   lambda r: setattr(r, "shutdown", True),
                   lambda r: setattr(r.owner, "homing_active", True),
                   lambda r: setattr(r.owner, "attempt", object()),
                   lambda r: setattr(r.toolhead, "busy", (0.0, 10.0, False)),
                   lambda r: setattr(r, "policy_error", "speed too high"),
                   lambda r: r.toolhead.kin.limits.__setitem__(0, (0.0, 300.0)),
                   lambda r: setattr(r.owner.rails_by_axis["x"].info, "retract_dist", 1.0),
                   lambda r: r.objects.__setitem__("homing_override", object()),
                   lambda r: r.objects.__setitem__("gcode_macro G28", object()),
                   lambda r: r.gcode.gcode_handlers.__setitem__("G28", lambda g: None))
        for setter in setters:
            with self.subTest(setter=setter):
                rig = AccuracyRig()
                setter(rig)
                before = rig.effects()
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(before, rig.effects())

    def test_discovery_is_excluded_population_statistics_match_known_values(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            rig.home_results.extend({"travel_mm": value} for value in (145.0, 149.0, 150.0, 153.0))
            rig.accuracy(samples=3)
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            axis = result["axis_results"]["x"]
            self.assertEqual(axis["discovery"]["travel_mm"], 145.0)
            self.assertEqual([r["travel_mm"] for r in axis["trials"]], [149.0, 150.0, 153.0])
            stats = axis["statistics"]
            self.assertEqual(stats["count"], 3)
            self.assertEqual((stats["min_mm"], stats["max_mm"], stats["spread_mm"]), (149.0, 153.0, 4.0))
            self.assertAlmostEqual(stats["mean_mm"], 452.0 / 3)
            self.assertAlmostEqual(stats["variance_mm2"], 26.0 / 9)
            self.assertAlmostEqual(stats["stddev_mm"], math.sqrt(26.0 / 9))
            self.assertEqual(stats["variance_definition"], "population (divide by N)")
            self.assertEqual([r["workflow_phase"] for r in result["records"]],
                             ["discovery", "measurement", "measurement", "measurement"])
            self.assertEqual(len(rig.toolhead.moves), 4)
            self.assertEqual(rig.toolhead.moves[-1], ([5.0, None, None], 100.0))
            self.assertEqual(result["paths"], [])
            summary = [item for item in rig.workflow.renderer.calls if item["kind"] == "accuracy"][-1]
            self.assertEqual(summary["result"], "complete")
            self.assertTrue(all(Path(path).suffix == ".png" for path in result["summary_paths"].values()))
            self.assertTrue(any("at or above" in line for line in rig.output))
            self.assertTrue(any("variance" in line and "mm^2" in line for line in rig.output))
            self.assert_restored(rig)

    def test_exact_travel_statistics_match_summary_when_console_values_are_equal(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            exact = (150.000244140625, 150.00048828125)
            rig.home_results.extend([{"travel_mm": 145.}] +
                                    [{"travel_mm": 150., "travel_mm_exact": value} for value in exact])
            rig.accuracy()
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "complete")
            axis = result["axis_results"]["x"]
            self.assertEqual([trial["travel_mm"] for trial in axis["trials"]], [150., 150.])
            expected = {"count": 2, "min_mm": exact[0], "max_mm": exact[1],
                        "mean_mm": 150.0003662109375, "spread_mm": .000244140625,
                        "stddev_mm": .0001220703125, "variance_mm2": 2. ** -26,
                        "variance_definition": "population (divide by N)"}
            self.assertEqual(axis["statistics"], expected)
            summary = [item for item in rig.workflow.renderer.calls if item["kind"] == "accuracy"][-1]
            self.assertEqual(summary["statistics"], expected)
            self.assert_restored(rig)

    @unittest.skipUnless(os.name == "posix", "Klipper temporary-directory fallbacks are Linux paths")
    def test_temporary_captures_stay_outside_watched_roots_until_summary_then_are_removed(self):
        with tempfile.TemporaryDirectory() as base:
            config = Path(base) / "config"
            output = config / "charts"
            output.mkdir(parents=True)
            previous = output / "previous.png"
            previous.write_bytes(b"previous user chart")
            rig = AccuracyRig({"charts_directory": str(output)})
            rig.workflow.chart_config_root = config
            rig.workflow.chart_start_args = {"config_file": str(config / "printer.cfg")}
            writer = workflow_module.write_accuracy
            def inspect_summary(*args, **kwargs):
                source = Path(kwargs["capture_directory"])
                self.assertNotIn(config, source.parents)
                self.assertNotIn(output, source.parents)
                captures = list(source.iterdir())
                self.assertEqual(len(captures), 3)
                self.assertTrue(all(path.suffix == ".png" for path in captures))
                self.assertEqual(list(output.iterdir()), [previous])
                paths = writer(*args, **kwargs)
                self.assertTrue(all(path.is_file() for path in captures))
                self.assertTrue(all(Path(path).is_file() for path in paths.values()))
                return paths
            with mock.patch.object(workflow_module.tempfile, "gettempdir", return_value=str(config / "tmp")), \
                    mock.patch.object(workflow_module, "write_accuracy", side_effect=inspect_summary):
                rig.accuracy()
            result = rig.workflow.last_result
            self.assertEqual(result["paths"], [])
            self.assertEqual(set(result["summary_paths"]), {"x_png"})
            self.assertEqual(set(output.iterdir()), {previous, Path(result["summary_paths"]["x_png"])})
            self.assertEqual(previous.read_bytes(), b"previous user chart")
            self.assertFalse((config / "tmp").exists())
            self.assert_restored(rig)

    def test_no_safe_temp_parent_and_begin_failure_leave_no_captures_or_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": str(Path(temp).anchor)})
            with mock.patch.object(workflow_module.tempfile, "TemporaryDirectory") as create, \
                    self.assertRaisesRegex(CommandError, "outside the chart/config"):
                rig.accuracy()
            create.assert_not_called()
            self.assertEqual(rig.scripts, [])
            self.assertEqual(rig.toolhead.moves, [])
            self.assert_restored(rig)
            rig = AccuracyRig({"charts_directory": temp})
            rig.policy_error = "synthetic begin failure"
            with self.assertRaisesRegex(CommandError, "begin failure"):
                rig.accuracy()
            self.assertEqual(len(rig.capture_directories), 1)
            self.assertEqual(rig.scripts, [])
            self.assert_restored(rig)

    def test_temporary_cleanup_failure_reports_error_without_replacing_original_motion_error(self):
        for motion_failure in (False, True):
            with self.subTest(motion_failure=motion_failure), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                create_directory = rig.workflow._accuracy_capture_directory
                def failing_cleanup_directory():
                    temporary = create_directory()
                    cleanup = temporary.cleanup
                    def fail_cleanup():
                        cleanup()
                        raise OSError("synthetic temporary cleanup failure")
                    temporary.cleanup = fail_cleanup
                    return temporary
                fault = OSError("synthetic original recenter failure")
                motion = mock.patch.object(rig.workflow, "_recenter", side_effect=fault) \
                    if motion_failure else contextlib.nullcontext()
                error_type = OSError if motion_failure else CommandError
                with mock.patch.object(rig.workflow, "_accuracy_capture_directory", side_effect=failing_cleanup_directory), \
                        motion, self.assertLogs(level="ERROR"), self.assertRaises(error_type) as caught:
                    rig.accuracy()
                if motion_failure:
                    self.assertIs(caught.exception, fault)
                else:
                    self.assertIn("temporary accuracy capture cleanup failed", str(caught.exception))
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertIn("capture_cleanup_error", rig.workflow.last_result)
                self.assert_restored(rig)

    def test_later_summary_failure_keeps_published_axis_and_cleans_all_intermediates(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            def fail_y(payload):
                if payload["kind"] == "accuracy" and payload["axis"] == "y":
                    raise ValueError("synthetic Y summary failure")
            rig.workflow.renderer.on_render = fail_y
            with self.assertLogs(level="ERROR"), self.assertRaisesRegex(CommandError, "Y summary failure"):
                rig.accuracy(axis="XY")
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            self.assertEqual(set(result["summary_paths"]), {"x_png"})
            self.assertEqual(list(Path(temp).iterdir()), [Path(result["summary_paths"]["x_png"])])
            self.assertEqual(result["paths"], [])
            self.assert_restored(rig)

    def test_invalid_exact_travel_is_rejected_without_losing_prior_measurements(self):
        for invalid in (None, True, "150.25", 0., -1., float("nan"), float("inf")):
            with self.subTest(exact=invalid), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                rig.home_results.extend([{}, {"travel_mm_exact": 150.25}, {"travel_mm_exact": invalid}])
                with self.assertRaisesRegex(CommandError, "invalid exact commanded travel"):
                    rig.accuracy()
                result = rig.workflow.last_result
                axis = result["axis_results"]["x"]
                self.assertEqual(result["result"], "aborted")
                self.assertEqual(len(axis["trials"]), 1)
                self.assertEqual(axis["statistics"]["mean_mm"], 150.25)
                self.assertIsNone(axis["statistics"]["variance_mm2"])
                self.assertIsNotNone(axis["rejected"])
                self.assertFalse(axis["complete"])
                self.assertEqual(len(rig.toolhead.moves), 2)
                self.assertIn("summary_paths", result)
                self.assert_restored(rig)

    def test_xy_default_and_explicit_order_recenter_x_before_y_preserve_z(self):
        for requested in (None, "XY"):
            for z_homed in (False, True):
                with self.subTest(requested=requested, z_homed=z_homed), tempfile.TemporaryDirectory() as temp:
                    rig = AccuracyRig({"charts_directory": temp})
                    if not z_homed:
                        rig.toolhead.kin.limits[2] = (1.0, -1.0)
                    safe = rig.use_safe_z_home()
                    rig.accuracy(axis=requested)
                    self.assertEqual(rig.scripts, ["G28 X"] * 3 + ["G28 Y"] * 3)
                    self.assertEqual(safe.calls, [])
                    self.assertEqual(len(rig.toolhead.moves), 6)
                    self.assertEqual(rig.toolhead.moves[-1], ([None, 5.0, None], 100.0))
                    self.assertEqual(rig.home_calls[3]["position"][0], 150.0)
                    self.assertEqual(rig.home_calls[3]["position"][1], 100.0)
                    result = rig.workflow.last_result
                    self.assertEqual(result["axes"], ["x", "y"])
                    self.assertEqual(result["axis_results"]["x"]["statistics"]["mean_mm"], 150.0)
                    self.assertEqual(result["axis_results"]["y"]["statistics"]["mean_mm"], 100.0)
                    self.assertEqual("z" in rig.toolhead.kin.get_status(0)["homed_axes"], z_homed)
                    self.assert_restored(rig)

    def test_single_y_and_exact_manual_threshold_are_used_without_tuning(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            rig.owner.threshold_g["y"] = 0.001
            rig.owner.rails_by_axis["y"].info.speed = 37.0
            rig.home_results.extend([{"threshold_g": 0.0, "onset_g": 0.0,
                                     "impact_peak_g": 0.0}] * 3)
            def exact_values(capture):
                capture["detector"].update(onset_g=-0.00125, impact_peak_g=0.00175)
            rig.capture_mutator = exact_values
            rig.accuracy(axis="Y")
            self.assertEqual([c["axis"] for c in rig.home_calls], ["y"] * 3)
            self.assertEqual([c["threshold"] for c in rig.home_calls], [0.001] * 3)
            for record in rig.workflow.last_result["records"]:
                self.assertEqual(record["threshold_g"], 0.001)
                self.assertEqual(record["onset_g"], -0.00125)
                self.assertEqual(record["impact_peak_g"], 0.00175)
                self.assertEqual(record["homing_speed_mm_s"], 37.0)
            self.assertEqual(rig.owner.last["y"]["threshold_g"], 0.0)
            self.assert_restored(rig, {"x": 8.0, "y": 0.001})

    def test_default_sample_count_and_maximum_xy_have_independent_exact_attempt_budget(self):
        for samples, expected in ((None, 22), (50, 102)):
            with self.subTest(samples=samples):
                rig = AccuracyRig({"calibration_max_attempts": 12})
                arguments = {"CENTERED": 1}
                if samples is not None:
                    arguments["SAMPLES"] = samples
                with mock.patch.object(workflow_module, "write_capture", return_value={}), \
                        mock.patch.object(workflow_module, "write_accuracy", return_value={}), \
                        mock.patch.object(workflow_module, "validate_accuracy", return_value={}):
                    rig.workflow.cmd_ACCURACY(GCmd(**arguments))
                self.assertEqual(len(rig.home_calls), expected)
                self.assertEqual(rig.capture_stops, expected)
                self.assertEqual(rig.workflow.last_result["attempts"], expected)
                self.assertEqual(rig.workflow.last_result["result"], "complete")
                self.assert_restored(rig)

    def test_bad_home_stops_without_retry_threshold_raise_or_unsafe_recenter(self):
        failures = ({"travel_mm": 2.0}, {"travel_mm": 119.0}, {"travel_mm": 181.0},
                    {"travel_mm": math.nan}, {"travel_mm": math.inf},
                    {"result": "pressed"}, {"result": "no_data"},
                    {"moving": False, "motion_seen": False})
        for preceding in (0, 2):
            for failure in failures:
                with self.subTest(preceding=preceding, failure=failure), tempfile.TemporaryDirectory() as temp:
                    rig = AccuracyRig({"charts_directory": temp})
                    rig.home_results.extend([{}] * preceding + [failure])
                    with self.assertRaisesRegex(CommandError, "no automatic move"):
                        rig.accuracy(axis="XY", samples=3)
                    self.assertEqual(len(rig.home_calls), preceding + 1)
                    self.assertEqual(len(rig.toolhead.moves), preceding)
                    result = rig.workflow.last_result
                    self.assertEqual(result["result"], "aborted")
                    self.assertEqual(len(result["axis_results"]["x"]["trials"]), max(0, preceding - 1))
                    self.assertIsNotNone(result["axis_results"]["x"]["rejected"])
                    self.assertEqual(result["axis_results"]["y"]["trials"], [])
                    self.assertIn("summary_paths", result)
                    if preceding == 2:
                        stats = result["axis_results"]["x"]["statistics"]
                        self.assertIsNone(stats["stddev_mm"])
                        self.assertIsNone(stats["variance_mm2"])
                    self.assert_restored(rig)

    def test_invalid_sensor_capture_or_provenance_aborts_and_closes_capture(self):
        def replace(capture, **values):
            capture.update(values)
        mutations = (lambda c: replace(c, samples=[]), lambda c: replace(c, dropped=1),
                     lambda c: c["samples"].__setitem__(2, (math.nan, 0, 0, 1)),
                     lambda c: c["samples"].__setitem__(2, c["samples"][1]),
                     lambda c: replace(c, baseline_g=None),
                     lambda c: replace(c, _record=copy.copy(c["_record"])),
                     lambda c: replace(c, detector=None),
                     lambda c: c["detector"].update(axis="y"),
                     lambda c: c["detector"].update(attempt_count=2),
                     lambda c: c["detector"].update(arm_time_s=0.001),
                     lambda c: c["detector"].update(threshold_g=8.01),
                     lambda c: replace(c, fault="stale_data"),
                     lambda c: replace(c, error="invalid_data"),
                     lambda c: c["detector"].update(direction=4),
                     lambda c: c["detector"].update(homing_speed_mm_s=math.nan),
                     lambda c: c["detector"].update(homing_accel_mm_s2=0),
                     lambda c: c["detector"].update(onset_time_s=12345.0),
                     lambda c: c["detector"].update(onset_g=math.nan),
                     lambda c: replace(c, samples=c["samples"][1:]))
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                rig.capture_mutator = mutation
                with self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(rig.scripts, ["G28 X"])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertEqual(rig.workflow.last_result["axis_results"]["x"]["trials"], [])
                self.assert_restored(rig)

    def test_current_retained_row_survives_a_later_clock_adjustment(self):
        for shift in (-0.00001, 0.00001):
            with self.subTest(shift=shift), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                def adjust_export_clock(capture):
                    capture["samples"] = [(row[0] + shift,) + row[1:] for row in capture["samples"]]
                    capture["detector"]["onset_time_s"] = capture["samples"][2][0]
                rig.capture_mutator = adjust_export_clock
                rig.accuracy()
                records = rig.workflow.last_result["records"]
                self.assertEqual(len(records), 3)
                for record in records:
                    self.assertTrue(record["capture_valid"])
                    self.assertEqual(record["capture_onset_sample_index"], 2)
                    self.assertAlmostEqual(record["capture_clock_shift_s"], shift)
                    self.assertNotEqual(record["onset_time_s"], record["trigger_onset_time_s"])
                self.assertEqual(rig.owner.last["x"]["onset_time_s"], records[-1]["trigger_onset_time_s"])
                self.assert_restored(rig)

    def test_last_retained_impact_row_is_used_without_a_timestamp_epsilon(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            def last_row_rounding(capture):
                last = capture["samples"][-1]
                capture["samples"][-1] = (last[0], -4.0, -5.0, 1.0)
                onset = math.nextafter(last[0], math.inf)
                stamp = capture["_record"]["onset_sample"]
                stamp = stamp[:2] + (3, stamp[3])
                capture["_record"].update(onset_time_s=onset, onset_sample=stamp)
                capture["detector"].update(onset_sample=stamp, onset_sample_index=3,
                                            trigger_onset_time_s=onset, onset_time_s=last[0])
            rig.capture_mutator = last_row_rounding
            rig.accuracy()
            for record in rig.workflow.last_result["records"]:
                self.assertTrue(record["capture_valid"])
                self.assertEqual(record["capture_onset_sample_index"], 3)
                self.assertGreater(record["trigger_onset_time_s"], record["onset_time_s"])
            self.assert_restored(rig)

    def test_missing_ambiguous_changed_or_downgraded_sample_proof_is_rejected(self):
        def omit_all_stamps(capture):
            capture["_record"].pop("onset_sample")
            for key in ("onset_sample", "onset_sample_version", "onset_sample_matches", "onset_sample_index"):
                capture["detector"].pop(key)
        mutations = (omit_all_stamps,
                     lambda c: c["detector"].update(onset_sample_version=0),
                     lambda c: c["detector"].update(onset_sample_version=True),
                     lambda c: c["detector"].update(onset_sample=None),
                     lambda c: c["_record"].update(onset_sample=(123, 3, 2, 4)),
                     lambda c: c["detector"].update(onset_sample_matches=0, onset_sample_index=None),
                     lambda c: c["detector"].update(onset_sample_matches=2, onset_sample_index=None),
                     lambda c: c["detector"].update(onset_sample_index=True),
                     lambda c: c["detector"].update(onset_sample_index=-1),
                     lambda c: c["detector"].update(onset_sample_index=len(c["samples"])),
                     lambda c: c["detector"].update(trigger_onset_time_s=c["_record"]["onset_time_s"] + 0.00001),
                     lambda c: c["detector"].update(onset_time_s=c["samples"][2][0] + 0.00001),
                     lambda c: c.update(samples=c["samples"][:2]))
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                rig.capture_mutator = mutation
                with self.assertRaisesRegex(CommandError, "lacks the current measured impact"):
                    rig.accuracy()
                self.assertEqual(rig.scripts, ["G28 X"])
                self.assertEqual(rig.toolhead.moves, [])
                self.assertFalse(rig.workflow.last_result["axis_results"]["x"]["complete"])
                self.assert_restored(rig)

    def test_missing_retained_impact_reports_bounded_diagnostics_and_keeps_prior_trials(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            def missing_third_measurement(capture):
                if rig.capture_stops == 4:
                    capture["detector"].update(onset_sample_matches=0, onset_sample_index=None,
                                                onset_time_s=None)
            rig.capture_mutator = missing_third_measurement
            with self.assertRaisesRegex(CommandError, "not retained exactly once.*matches=0"):
                rig.accuracy(axis="Y", samples=5)
            result = rig.workflow.last_result
            axis_result = result["axis_results"]["y"]
            self.assertEqual(len(axis_result["trials"]), 2)
            self.assertEqual(axis_result["statistics"]["count"], 2)
            self.assertEqual(result["paths"], [])
            rejected = axis_result["rejected"]
            self.assertFalse(rejected["capture_valid"])
            diagnostics = rejected["capture_diagnostics"]
            self.assertEqual(diagnostics["reason"], "onset packet/sample was not retained exactly once")
            self.assertEqual(diagnostics["onset_sample_matches"], 0)
            self.assertEqual(diagnostics["sample_count"], 4)
            self.assertIsNone(diagnostics["capture_onset_time_s"])
            self.assertLess(len(json.dumps(diagnostics, allow_nan=False)), 1500)
            json.dumps(rig.workflow.get_status(0), allow_nan=False)
            self.assert_restored(rig)

    def test_prior_completed_axis_and_paths_survive_fault_on_next_axis(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            rig.home_results.extend([{}, {}, {}, CommandError("sensor fault")])
            with self.assertRaisesRegex(CommandError, "sensor fault"):
                rig.accuracy(axis="XY")
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            self.assertTrue(result["axis_results"]["x"]["complete"])
            self.assertFalse(result["axis_results"]["y"]["complete"])
            self.assertEqual(result["paths"], [])
            self.assertIn("summary_paths", result)
            self.assert_restored(rig)

    def test_stale_result_or_cross_trial_sensor_times_cannot_be_reused(self):
        for reuse in ("record", "clock"):
            with self.subTest(reuse=reuse), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                original = rig.stock_home
                def stale(gcmd):
                    if not rig.home_calls:
                        original(gcmd)
                    elif reuse == "clock":
                        rig.sensor_time -= 1.0
                        original(gcmd)
                rig.stock_home = stale
                with self.assertRaisesRegex(CommandError, "new accelerometer|current detector"):
                    rig.accuracy()
                self.assertEqual(rig.workflow.last_result["result"], "aborted")
                self.assertEqual(rig.workflow.last_result["axis_results"]["x"]["trials"], [])
                self.assertEqual(len(rig.toolhead.moves), 1)
                self.assert_restored(rig)

    def test_cancel_shutdown_print_change_and_recenter_failure_cannot_continue_or_complete(self):
        for ending in ("cancel", "shutdown", "printing", "recenter"):
            with self.subTest(ending=ending), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                original = rig.toolhead.wait_moves
                def interrupted_wait():
                    original()
                    if not rig.toolhead.moves:
                        return
                    if ending == "cancel":
                        rig.workflow.cmd_CANCEL(GCmd())
                    elif ending == "shutdown":
                        rig.shutdown = True
                        for callback in rig.handlers["klippy:shutdown"]:
                            callback()
                    elif ending == "printing":
                        rig.print_state = "printing"
                    else:
                        raise CommandError("synthetic recenter failure")
                rig.toolhead.wait_moves = interrupted_wait
                logs = self.assertLogs(level="ERROR") if ending == "recenter" else contextlib.nullcontext()
                with logs, self.assertRaises(CommandError):
                    rig.accuracy()
                self.assertEqual(rig.scripts, ["G28 X"])
                self.assertEqual(len(rig.toolhead.moves), 1)
                result = rig.workflow.last_result
                self.assertIn(result["result"], ("cancelled", "aborted"))
                self.assertFalse(result["axis_results"]["x"]["complete"])
                self.assert_restored(rig)

    def test_cancel_during_capture_setup_prevents_dispatch(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            original = rig.start_capture
            def cancel_start():
                original()
                rig.workflow.cmd_CANCEL(GCmd())
            rig.owner.start_capture = cancel_start
            with self.assertRaisesRegex(CommandError, "cancelled or interrupted"):
                rig.accuracy()
            self.assertEqual(rig.scripts, [])
            self.assertEqual(rig.toolhead.moves, [])
            self.assertEqual(rig.workflow.last_result["result"], "cancelled")
            self.assert_restored(rig)

    def test_export_failure_preserves_trials_partial_status_and_removes_temporary_captures(self):
        for target, error_type in (("capture", ValueError), ("capture", OSError),
                                   ("summary", ValueError), ("summary", OSError)):
            with self.subTest(target=target, error_type=error_type), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                writer = workflow_module.write_capture
                count = 0
                def fail_second(*args, **kwargs):
                    nonlocal count
                    count += 1
                    if count == 2:
                        raise error_type("synthetic capture export failure")
                    return writer(*args, **kwargs)
                with mock.patch.object(workflow_module, "write_capture", side_effect=fail_second) \
                        if target == "capture" else mock.patch.object(
                            workflow_module, "write_accuracy", side_effect=error_type("synthetic summary export failure")):
                    logs = self.assertLogs(level="ERROR") if target == "summary" else contextlib.nullcontext()
                    with logs, self.assertRaisesRegex(CommandError, "synthetic %s export failure" % target):
                        rig.accuracy()
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "aborted")
                self.assertEqual(len(result["axis_results"]["x"]["trials"]), 1 if target == "capture" else 2)
                self.assertEqual(result["paths"], [])
                if target == "summary":
                    self.assertIn("chart_error", result)
                else:
                    self.assertIn("summary_paths", result)
                    self.assertNotIn("capture_paths", result["axis_results"]["x"])
                self.assert_restored(rig)

    def test_summary_validation_errors_are_command_errors_after_safe_cleanup(self):
        for error_type in (ValueError, OSError):
            with self.subTest(error_type=error_type), tempfile.TemporaryDirectory() as temp:
                rig = AccuracyRig({"charts_directory": temp})
                fault = error_type("synthetic summary validation failure")
                with mock.patch.object(workflow_module, "validate_accuracy", side_effect=fault), \
                        self.assertRaisesRegex(CommandError, "accuracy summary validation failed") as caught:
                    rig.accuracy()
                self.assertIs(caught.exception.__cause__, fault)
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "aborted")
                self.assertIn("synthetic summary validation failure", result["reason"])
                self.assertEqual(result["axis_results"]["x"]["statistics"]["count"], 2)
                self.assertEqual(len(rig.home_calls), 3)
                self.assertEqual(len(rig.toolhead.moves), 2)
                self.assertEqual(set(rig.disabled), {"stepper_x", "stepper_y"})
                self.assertIn("summary_paths", result)
                self.assert_restored(rig)

    def test_motion_and_restoration_faults_are_not_reclassified_as_chart_errors(self):
        for stage in ("recenter", "terminal_release", "restore"):
            for error_type in (ValueError, OSError, RuntimeError):
                with self.subTest(stage=stage, error_type=error_type), tempfile.TemporaryDirectory() as temp:
                    rig = AccuracyRig({"charts_directory": temp})
                    fault = error_type("synthetic %s failure" % stage)
                    if stage == "restore":
                        original = rig.owner._end_session
                        def fail_final_restore():
                            original()
                            if rig.capture_stops == 3:
                                raise fault
                        patch = mock.patch.object(rig.owner, "_end_session", side_effect=fail_final_restore)
                    else:
                        method = "_recenter" if stage == "recenter" else "_release_terminal_home"
                        patch = mock.patch.object(rig.workflow, method, side_effect=fault)
                    with patch, self.assertRaises(error_type) as caught:
                        rig.accuracy()
                    self.assertIs(caught.exception, fault)
                    self.assertEqual(rig.workflow.last_result["result"], "aborted")
                    self.assertIn(str(fault), rig.workflow.last_result["reason"])
                    self.assert_restored(rig)

    def test_restoration_failure_takes_precedence_over_controlled_validation_error(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            original = rig.owner._end_session
            fault = OSError("synthetic restoration failure during export-error cleanup")
            def fail_final_restore():
                original()
                if rig.capture_stops == 3:
                    raise fault
            with mock.patch.object(workflow_module, "validate_accuracy", side_effect=ValueError("bad statistics")), \
                    mock.patch.object(rig.owner, "_end_session", side_effect=fail_final_restore), \
                    self.assertRaises(OSError) as caught:
                rig.accuracy()
            self.assertIs(caught.exception, fault)
            self.assertIn("workflow cleanup failed", rig.workflow.last_result["reason"])
            self.assertEqual(rig.workflow.last_result["result"], "aborted")
            self.assert_restored(rig)

    def test_final_restoration_failure_is_aborted_before_summary_publication(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            original = rig.owner._end_session
            def fail_final_restore():
                original()
                if rig.capture_stops == 3:
                    raise CommandError("synthetic final restoration failure")
            rig.owner._end_session = fail_final_restore
            with self.assertRaisesRegex(CommandError, "final restoration failure"):
                rig.accuracy()
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            self.assertIn("cleanup failed", result["reason"])
            self.assertEqual(result["axis_results"]["x"]["statistics"]["count"], 2)
            self.assertTrue(result["axis_results"]["x"]["complete"])
            metadata = [item for item in rig.workflow.renderer.calls if item["kind"] == "accuracy"][-1]
            self.assertEqual(metadata["result"], "aborted")
            self.assert_restored(rig)

    def test_status_and_results_retain_metadata_without_raw_capture_arrays(self):
        with tempfile.TemporaryDirectory() as temp:
            rig = AccuracyRig({"charts_directory": temp})
            rig.workflow.chart_config_root = Path(temp)
            active = []
            rig.on_home = lambda axis: active.append(rig.workflow.get_status(0)["active"])
            rig.accuracy()
            self.assertEqual(active[-1]["sample_index"], 2)
            self.assertEqual(active[-1]["fixed_thresholds_g"], {"x": 8.0})
            self.assertEqual(active[-1]["completed_samples"], {"x": 1})
            self.assertEqual(active[-1]["configured_homing_speeds_mm_s"], {"x": 100.0})
            self.assertNotIn("candidate", active[-1])
            self.assertNotIn("successes", active[-1])
            self.assertNotIn("repeatability_limit_mm", active[-1])
            encoded = json.dumps(rig.workflow.get_status(0), allow_nan=False)
            self.assertNotIn('"samples":', encoded)
            self.assertNotIn('"_record":', encoded)
            self.assertNotIn('"detector":', encoded)
            self.assertNotIn('"capture_paths":', encoded)
            self.assertNotIn('"discovery_capture_paths":', encoded)
            self.assertNotIn("accel_homing_accuracy_", encoded)
            self.assertTrue(all("capture_paths" not in state["axis_results"]["x"] for state in active))
            self.assertIn("%3A", html.unescape("\n".join(rig.output)))
            self.assertNotIn("action:prompt_", "\n".join(rig.output))


class DetectorCaptureProvenanceTests(unittest.TestCase):
    def retained_impact(self, arm_clock=200, start_clock=250, position=2):
        obj = make_homing()
        obj.start_capture()
        obj._capture_batch(batch(arm_clock - 20, [(0, 0, counts(1))] * 8))
        attempt, trigger = arm_axis(obj, "x", arm_clock, arm_clock + 1000)
        attempt.bases, attempt.baseline = (0.0, 0.0, counts(1)), 0.0
        attempt.effective_speed, attempt.effective_accel = 100.0, 5000.0
        attempt.moving = attempt.motion_seen = True
        values = [(0, 0, counts(1))] * 8
        values[position] = (counts(-10), 0, counts(1))
        params = batch(start_clock, values)
        obj._inspect(params)
        self.assertEqual(attempt.reason, "impact")
        self.assertEqual(len(trigger.messages), 1)
        obj.disarm()
        return obj, attempt, params

    def validate_retained_impact(self, attempt, capture):
        rig = Rig()
        session = {"saved_thresholds": {"x": 8.0}, "last_arm_time": None}
        record = dict(attempt.record)
        rig.workflow._validate_accuracy_capture(session, "x", attempt.record,
                                                capture["_record"], record, capture)
        return record

    def test_export_clock_adjustment_uses_the_actual_retained_row_and_keeps_trigger_time(self):
        for shift in (-0.00001, 0.00001):
            with self.subTest(shift=shift):
                obj, attempt, _params = self.retained_impact()
                original = copy.deepcopy(attempt.record)
                obj.clock_to_time = lambda clock: clock / 1000.0 + shift
                capture = obj.stop_capture()
                detector = capture["detector"]
                self.assertEqual(detector["onset_sample_matches"], 1)
                self.assertEqual(detector["onset_sample_index"], 10)
                self.assertEqual(detector["onset_time_s"], capture["samples"][10][0])
                self.assertEqual(detector["trigger_onset_time_s"], original["onset_time_s"])
                self.assertNotEqual(detector["onset_time_s"], original["onset_time_s"])
                record = self.validate_retained_impact(attempt, capture)
                self.assertEqual(record["onset_time_s"], capture["samples"][10][0])
                self.assertEqual(record["trigger_onset_time_s"], original["onset_time_s"])
                self.assertEqual(attempt.record, original)

    def test_arm_trimmed_last_single_sample_refers_to_the_original_retained_packet(self):
        obj, attempt, params = self.retained_impact(start_clock=193, position=7)
        self.assertEqual(attempt.onset_batch, (200, 0, 0, 1))
        self.assertEqual(attempt.onset_sample, (193, 7, 7, 8))
        obj.clock_to_time = lambda clock: clock / 1000.0 + 0.00001
        capture = obj.stop_capture()
        self.assertEqual(capture["detector"]["onset_sample_index"], 15)
        self.assertEqual(capture["detector"]["onset_time_s"], capture["samples"][-1][0])
        self.assertEqual(len(capture["samples"]), 16)
        self.assertEqual(capture["samples"][-1][1], counts(-10) * attempt.count_g)
        self.assertEqual(params["start_clock"], 193)
        self.validate_retained_impact(attempt, capture)

    def test_original_packet_identity_and_sample_index_survive_clock_wrap(self):
        arm_clock = 2 ** 32 - 8
        obj, attempt, params = self.retained_impact(arm_clock, arm_clock + 4, position=4)
        obj.clock_to_time = lambda clock: (arm_clock + (
            ((clock - arm_clock + 0x80000000) & 0xFFFFFFFF) - 0x80000000)) / 1000.0
        capture = obj.stop_capture()
        self.assertEqual(attempt.onset_sample, (params["start_clock"], 7, 4, 8))
        self.assertEqual(capture["detector"]["onset_sample_matches"], 1)
        self.assertEqual(capture["detector"]["onset_sample_index"], 12)
        self.assertEqual(capture["detector"]["onset_time_s"], capture["samples"][12][0])
        self.validate_retained_impact(attempt, capture)

    def test_missing_or_ambiguous_retained_packet_never_synthesizes_an_impact_row(self):
        for mode in ("missing", "duplicate"):
            with self.subTest(mode=mode):
                obj, attempt, params = self.retained_impact()
                if mode == "missing":
                    obj.capture_batches = [row for row in obj.capture_batches if row[0] != params["start_clock"]]
                    obj._capture_batch(batch(300, [(0, 0, counts(1))] * 8))
                else:
                    obj._capture_batch(params)
                capture = obj.stop_capture()
                self.assertEqual(capture["detector"]["onset_sample_matches"], 0 if mode == "missing" else 2)
                self.assertIsNone(capture["detector"]["onset_sample_index"])
                self.assertIsNone(capture["detector"]["onset_time_s"])
                self.assertEqual(len(capture["samples"]), 16 if mode == "missing" else 24)
                with self.assertRaisesRegex(CommandError, "not retained exactly once"):
                    self.validate_retained_impact(attempt, capture)

    def test_changed_or_removed_backend_stamp_cannot_downgrade_to_timestamp_equality(self):
        for mode in ("changed", "removed"):
            with self.subTest(mode=mode):
                obj, attempt, _params = self.retained_impact()
                if mode == "changed":
                    attempt.onset_sample = (250, 7, 1, 8)
                else:
                    attempt.onset_sample = None
                    attempt.record.pop("onset_sample")
                capture = obj.stop_capture()
                with self.assertRaisesRegex(CommandError, "missing or changed raw onset sample identity"):
                    self.validate_retained_impact(attempt, capture)

    def test_previous_pass_cannot_supply_current_capture_reference(self):
        obj = make_homing()
        previous, _trigger = arm_axis(obj, "x", 100, 1000)
        previous.bases = (0.0, 0.0, counts(1))
        obj.attempt, obj.last_pass = None, previous
        obj.start_capture()
        obj._capture_batch(batch(150, [(counts(0.2), 0, counts(1))] * 8))
        current, _trigger = arm_axis(obj, "x", 200, 1000)
        current.bases = (counts(0.5), 0.0, counts(1))
        current.record = {"axis": "x"}
        obj._capture_batch(batch(200, [(counts(0.5), 0, counts(1))] * 8))
        captured = obj.stop_capture()
        self.assertEqual(captured["baseline_g"][0], 0.5)
        self.assertIs(captured["_record"], current.record)
        self.assertEqual(captured["detector"]["attempt_count"], 1)
        self.assertIsNone(obj.capture_attempt)
        self.assertEqual(obj.capture_attempts, 0)

    def test_negative_permuted_native_mapping_keeps_machine_frame_and_exact_values(self):
        obj = make_homing(axes_map=((1, -1), (0, 1), (2, 1)), threshold=0.001)
        obj.start_capture()
        current, _trigger = arm_axis(obj, "x", 200, 1000)
        current.bases = (0.0, 0.0, counts(1))
        current.baseline = 0.0
        current.effective_speed, current.effective_accel = 37.0, 4000.0
        params = batch(200, [(0, 1, counts(1))] * 8)
        obj._onset(current, params, [1] * 8, 8)
        current.record = {"axis": "x"}
        obj._capture_batch(params)
        capture = obj.stop_capture()
        detector = capture["detector"]
        self.assertEqual(capture["samples"][0][1], -0.0078125)
        self.assertEqual(detector["onset_g"], -0.0078125)
        self.assertEqual(detector["impact_peak_g"], 0.0078125)
        self.assertEqual(detector["direction"], -1)
        self.assertEqual(detector["threshold_g"], 0.001)
        self.assertEqual(detector["homing_speed_mm_s"], 37.0)
        self.assertIsNone(obj.capture_attempt)

    def test_multi_attempt_capture_is_explicit_and_pointer_is_released(self):
        obj = make_homing()
        obj.start_capture()
        first, _trigger = arm_axis(obj, "x", 100, 1000)
        first.bases = (0, 0, counts(1))
        second, _trigger = arm_axis(obj, "x", 200, 1000)
        second.bases = (0, 0, counts(1))
        obj._capture_batch(batch(200, [(0, 0, counts(1))] * 8))
        capture = obj.stop_capture()
        self.assertEqual(capture["detector"]["attempt_count"], 2)
        self.assertIsNone(obj.capture_attempt)


if __name__ == "__main__":
    unittest.main()
