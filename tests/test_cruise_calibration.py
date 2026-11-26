"""Calibration finds, checks and saves a cruise threshold; synthetic workflows only."""
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

from harness import CommandError, ConfigError, make_homing
from test_accuracy_recovery import RecoveryRig
import test_auto_notch_workflow as auto_tests
from test_beta_calibration import BetaRig
from test_calibration_config import NativeShape
from test_cruise_threshold import CruisePlanTests
from test_gentle_auto_workflow import GentleAutoRig
from test_workflows import AccelHomingModel, Config, GCmd, Rig

SECTION = "accel_homing_model default"
MODEL = {"cruise_peak_g": 2.0, "impact_peak_g": 20.0}
CHECK = {"cruise_peak_g": 1.0, "impact_peak_g": 20.0, "cruise_threshold_active": True}


def cruise_rig(*checks, start=7.5, **options):
    """Discovery and ten checks at START_THRESHOLD 7.5 g save 8 g; the rest are cruise checks."""
    rig = Rig(options)
    rig.home_results.extend([dict(MODEL) for _ in range(11)] + [dict(CHECK, **item) for item in checks])
    rig.owner.cruise_threshold_set, rig.owner.cruise_threshold_g = set(), {"x": 0.0, "y": 0.0}
    rig.seen = []
    rig.on_home = lambda axis: rig.seen.append(rig.workflow.cruise_threshold(axis))
    rig.start = start
    return rig


def metadata(rig, axis="x"):
    return json.loads(rig.configfile.saved[(SECTION, axis + "_metadata")])


def reload(rig, **options):
    saved = {option: value for (section, option), value in rig.configfile.saved.items() if section == SECTION}
    loaded = Rig(dict(options), sections={"printer": {"kinematics": "corexy"}, SECTION: saved})
    loaded.owner.cruise_threshold_set = {axis for axis in "xy" if "cruise_threshold" in options
                                         or axis + "_cruise_threshold" in options}
    loaded.owner.cruise_threshold_g = {axis: options.get(axis + "_cruise_threshold",
                                                         options.get("cruise_threshold", 0.0)) for axis in "xy"}
    loaded.workflow._validate_model_connection()
    return loaded


class CruiseCalibrationTests(unittest.TestCase):
    def calibrate(self, rig, save=0):
        with mock.patch.object(rig.owner, "release_workflow_home",
                               wraps=rig.owner.release_workflow_home) as release:
            rig.calibrate(save=save, START_THRESHOLD=rig.start)
        return release

    def test_five_passing_checks_save_the_candidate_and_release_from_the_last_one(self):
        rig = cruise_rig({}, {}, {}, {}, {})
        release = self.calibrate(rig)
        self.assertEqual([call["threshold"] for call in rig.home_calls], [7.5] * 16)
        # Grid 7.5 - n * 0.25 down to the first value >= max(1.2 * 2.0, 2.0 + 0.1).
        self.assertEqual(rig.seen, [0.0] * 11 + [2.5] * 5)
        result, saved = rig.workflow.last_result, metadata(rig)
        self.assertEqual(result["result"], "calibrated")
        self.assertEqual(result["saved_threshold_g"], 8.0)
        self.assertEqual(result["cruise_threshold_g"], 2.5)
        self.assertEqual(saved["cruise_threshold_g"], 2.5)
        evidence = saved["cruise_threshold_calibration"]
        self.assertEqual(evidence, dict(result["cruise_threshold_calibration"]))
        self.assertEqual((evidence["result"], evidence["worst_cruise_peak_g"], evidence["candidate_g"]),
                         ("verified", 2.0, 2.5))
        self.assertEqual((evidence["margin_factor"], evidence["margin_g"], evidence["max_threshold_fraction"]),
                         (1.2, 0.1, 0.75))
        self.assertEqual((evidence["verification_homes"], evidence["min_commanded_travel_mm"],
                          evidence["max_commanded_travel_mm"]), (5, 150.0, 150.0))
        self.assertEqual(len(saved["onset_g"]), 10)
        self.assertEqual(release.call_count, 1)
        self.assertIs(release.call_args[0][1], rig.owner.last["x"])
        self.assertIn("x", rig.toolhead.kin.get_status(0)["homed_axes"])
        self.assertTrue(any("cruise check 5/5 at 2.5 g: passed" in line for line in rig.output))
        self.assertTrue(any("cruise threshold 2.5 g passed 5 homes" in line for line in rig.output))
        self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})

    def test_saved_value_is_used_by_later_ordinary_homes_and_explicit_config_overrides_it(self):
        rig = cruise_rig({}, {}, {}, {}, {})
        self.calibrate(rig)
        loaded = reload(rig)
        self.assertEqual(loaded.workflow.stale_model_axes, {})
        self.assertEqual(loaded.workflow.ordinary_cruise_threshold("x"), (2.5, "model"))
        self.assertEqual(loaded.workflow.cruise_threshold("x"), 2.5)
        self.assertEqual(loaded.workflow.ordinary_cruise_threshold("y"), (0.0, None))
        for options, expected in (({"x_cruise_threshold": 0.0}, (0.0, "config")),
                                  ({"cruise_threshold": 3.0}, (3.0, "config")),
                                  ({"cruise_threshold": 3.0, "x_cruise_threshold": 0.0}, (0.0, "config"))):
            with self.subTest(options=options):
                self.assertEqual(reload(rig, **options).workflow.ordinary_cruise_threshold("x"), expected)
        # A manual or temporary threshold, or a stale model, never uses the saved cruise value.
        manual = reload(rig, x_threshold=6.0)
        manual.owner.threshold_g["x"] = 6.0
        self.assertEqual(manual.workflow.ordinary_cruise_threshold("x"), (0.0, None))
        loaded.owner.threshold_g["x"] = 8.0
        loaded.workflow.note_manual_threshold("x", 8.0)
        self.assertEqual(loaded.workflow.ordinary_cruise_threshold("x"), (0.0, None))
        stale = reload(rig)
        stale.workflow.stale_model_axes["x"] = {"key": "sensor", "saved": "a", "current": "b"}
        self.assertEqual(stale.workflow.ordinary_cruise_threshold("x"), (0.0, None))

    def test_ordinary_home_arms_the_model_value_and_calibration_ignores_it(self):
        plan = CruisePlanTests()
        model = types.SimpleNamespace(thresholds={"x": 8.0}, metadata={"x": {
            "cruise_threshold_g": 2.5, "polarity_policy": "absolute_after_reference_subtraction_v1"}})
        for options, session, expected in (({}, None, 2.5), ({"x_cruise_threshold": 0.0}, None, None),
                                           ({"x_cruise_threshold": 3.0}, None, 3.0),
                                           ({}, {"mode": "calibration", "state": "running"}, None)):
            with self.subTest(options=options, session=session):
                original = make_homing

                def with_model(*args, **kwargs):
                    obj = original(*args, **kwargs)
                    obj.workflows.model, obj.workflows.manual_axes = model, set()
                    return obj

                with mock.patch("test_cruise_threshold.make_homing", with_model):
                    attempt = plan.prepared(options, session)[3]
                value = None if attempt.cruise_limit_raw is None else attempt.cruise_limit_raw * attempt.count_g
                self.assertEqual(None if value is None else round(value, 9), expected)

    def test_status_shows_the_active_cruise_threshold_and_its_source(self):
        obj = make_homing(options={"x_cruise_threshold": 0.0})
        obj.workflows.model = types.SimpleNamespace(thresholds={"y": 8.0},
                                                    metadata={"y": {"cruise_threshold_g": 2.5}})
        obj.workflows.manual_axes = {"x"}
        lines = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=lines.append))
        self.assertIn("accel_homing: cruise threshold X off (config), Y 2.5 g (model)", lines)
        obj.cruise_threshold_set = set()
        lines = []
        obj.cmd_STATUS(types.SimpleNamespace(respond_info=lines.append))
        self.assertIn("accel_homing: cruise threshold X off, Y 2.5 g (model)", lines)

    def test_mid_travel_spike_is_not_offered_and_the_normal_model_is_saved(self):
        rig = cruise_rig()
        rig.home_results[4]["cruise_peak_g"] = 7.96
        self.calibrate(rig)
        self.assertEqual(len(rig.home_calls), 11)
        self.assertEqual(rig.seen, [0.0] * 11)
        saved = metadata(rig)
        self.assertNotIn("cruise_threshold_g", saved)
        evidence = saved["cruise_threshold_calibration"]
        self.assertEqual((evidence["result"], evidence["worst_cruise_peak_g"], evidence["candidate_g"],
                          evidence["verification_homes"]), ("not_offered", 7.96, 9.75, 0))
        self.assertIsNone(rig.workflow.last_result["cruise_threshold_g"])
        self.assertTrue(any("cruise threshold not offered: steady travel peaked at 7.96 g, so the candidate "
                            "9.75 g is not below 75% of the 8 g threshold (6 g)" in line for line in rig.output))
        self.assertIn("x", rig.toolhead.kin.get_status(0)["homed_axes"])

    def test_candidate_must_stay_below_three_quarters_of_the_saved_threshold(self):
        # 8 g saved: 6 g is not offered, 5.75 g is.
        for peak, homes in ((4.9, 11), (4.75, 16)):
            with self.subTest(peak=peak):
                rig = cruise_rig(*([{}] * 5))
                for result in list(rig.home_results)[:11]:
                    result["cruise_peak_g"] = peak
                self.calibrate(rig)
                self.assertEqual(len(rig.home_calls), homes)
                self.assertEqual(metadata(rig)["cruise_threshold_calibration"]["candidate_g"],
                                 6.0 if homes == 11 else 5.75)

    def test_candidate_lies_on_the_calibration_grid_and_never_below_one_g(self):
        for start, step, peak, expected in ((7.5, .25, 2.0, 2.5), (7.3, .25, 2.0, 2.55), (7.5, .3, 2.0, 2.4),
                                            (7.5, .25, 0.05, 1.0), (7.5, .25, 0.5, 1.0), (1.0, .25, 2.0, 2.5)):
            with self.subTest(start=start, step=step, peak=peak):
                rig = cruise_rig(*([{}] * 5), start=start, calibration_step=step)
                for result in list(rig.home_results)[:11]:
                    result["cruise_peak_g"] = peak
                with mock.patch.object(rig.workflow, "_calibration_saved_threshold", return_value=8.0):
                    self.calibrate(rig)
                self.assertEqual(metadata(rig)["cruise_threshold_calibration"]["candidate_g"], expected)

    def test_missing_measurement_saves_exactly_the_normal_model(self):
        for missing in (None, float("nan"), "2.0"):
            with self.subTest(missing=missing):
                rig = cruise_rig()
                rig.home_results[0]["cruise_peak_g"] = missing
                self.calibrate(rig)
                plain = Rig()
                plain.calibrate(save=0, START_THRESHOLD=7.5)
                self.assertEqual(len(rig.home_calls), 11)
                saved = metadata(rig)
                self.assertFalse(any(key.startswith("cruise") for key in saved))
                self.assertEqual(sorted(saved), sorted(metadata(plain)))
                self.assertTrue(any("steady travel was not measured on every qualified home" in line
                                    for line in rig.output))

    def assert_dropped(self, rig, homes, reason):
        self.assertEqual(len(rig.home_calls), homes)
        self.assertEqual(rig.seen, [0.0] * 11 + [2.5] * (homes - 11))
        result, saved = rig.workflow.last_result, metadata(rig)
        self.assertEqual(result["result"], "calibrated")
        self.assertEqual(float(rig.configfile.saved[(SECTION, "x_threshold")]), 8.0)
        self.assertEqual(float(rig.configfile.saved[("accel_homing", "x_threshold")]), 8.0)
        self.assertNotIn("cruise_threshold_g", saved)
        self.assertIsNone(result["cruise_threshold_g"])
        evidence = saved["cruise_threshold_calibration"]
        self.assertEqual((evidence["result"], evidence["verification_homes"]),
                         ("verification_failed", homes - 11))
        self.assertIn(reason, evidence["reason"])
        self.assertEqual(saved["max_commanded_travel_mm"], 150.0)
        self.assertNotIn("x", rig.toolhead.kin.get_status(0)["homed_axes"])
        self.assertTrue(any("cruise threshold dropped: " in line and "final back-off is skipped" in line
                            and "still saved, without a cruise threshold" in line for line in rig.output))
        self.assertIsNone(rig.workflow.session)

    def test_early_check_is_reversed_and_leaves_the_axis_unhomed_without_release(self):
        rig = cruise_rig({}, {"travel_mm": 60.0})
        moves = []
        rig.on_home = lambda axis: (rig.seen.append(rig.workflow.cruise_threshold(axis)),
                                    moves.append(len(rig.toolhead.moves)))
        release = self.calibrate(rig)
        self.assert_dropped(rig, 13, "premature trigger at 60")
        release.assert_not_called()
        # The last move reverses the recorded early travel from the endstop coordinate.
        self.assertEqual(len(rig.toolhead.moves), moves[-1] + 1)
        self.assertEqual(rig.toolhead.moves[-1][0], [60.0, None, None])
        self.assertEqual(rig.workflow.last_result["recovery"]["count"], 1)
        self.assertTrue(any("The early home was reversed by its recorded commanded travel and X is left unhomed"
                            in line for line in rig.output))

    def test_spread_or_inactive_cruise_leaves_the_carriage_where_it_stopped(self):
        for check, reason in (({"travel_mm": 152.5}, "spread 2.5 mm"),
                              ({"cruise_threshold_active": False}, "never became active")):
            with self.subTest(reason=reason):
                rig = cruise_rig({}, {}, check)
                moves = []
                rig.on_home = lambda axis: (rig.seen.append(rig.workflow.cruise_threshold(axis)),
                                            moves.append(len(rig.toolhead.moves)))
                release = self.calibrate(rig)
                self.assert_dropped(rig, 14, reason)
                release.assert_not_called()
                self.assertEqual(len(rig.toolhead.moves), moves[-1])
                self.assertTrue(any("X is left unhomed where that home stopped" in line for line in rig.output))

    def test_fatal_fault_or_cancel_during_checks_aborts_without_saving(self):
        for fault in ("sensor", "implausible", "cancel"):
            with self.subTest(fault=fault):
                rig = cruise_rig({}, {"result": "no_data"} if fault == "sensor" else
                                 {"travel_mm": 200.0} if fault == "implausible" else {})
                if fault == "cancel":
                    stop = rig.owner.stop_capture

                    def cancel_after_check():
                        capture = stop()
                        if len(rig.home_calls) == 13:
                            rig.workflow.cmd_CANCEL(GCmd())
                        return capture

                    rig.owner.stop_capture = cancel_after_check
                with self.assertRaises(CommandError):
                    rig.calibrate(save=1, START_THRESHOLD=7.5)
                self.assertEqual(len(rig.home_calls), 13)
                self.assertEqual(rig.configfile.saved, {})
                self.assertNotIn("SAVE_CONFIG", rig.scripts)
                self.assertNotEqual(rig.workflow.last_result["result"], "calibrated")
                self.assertEqual(rig.owner.threshold_g, {"x": 8.0, "y": 9.0})

    def test_qualification_window_still_rejects_changed_bookkeeping(self):
        def tamper_attempts(session):
            session["attempts"] -= 1

        def tamper_phase(session):
            session["cruise_verification"]["records"][-1]["workflow_phase"] = "validation"

        def tamper_order(session):
            records = session["records"]
            records[-6], records[-1] = records[-1], records[-6]

        def count_check_as_validation(session):
            session["validation_records"] = session["validation_records"][1:] + \
                [session["cruise_verification"]["records"][0]]

        def drop_check(session):
            session["cruise_verification"]["records"].pop()

        for tamper in (tamper_attempts, tamper_phase, tamper_order, count_check_as_validation, drop_check):
            with self.subTest(tamper=tamper.__name__):
                rig = cruise_rig({}, {}, {}, {}, {})
                verify = rig.workflow._verify_cruise_threshold

                def tampered():
                    verify()
                    tamper(rig.workflow.session)

                with mock.patch.object(rig.workflow, "_verify_cruise_threshold", side_effect=tampered), \
                        self.assertRaisesRegex(CommandError, "cannot save calibration"):
                    rig.calibrate(save=0, START_THRESHOLD=7.5)
                self.assertEqual(rig.configfile.saved, {})

    def test_recalibration_without_a_cruise_threshold_drops_the_old_one(self):
        rig = cruise_rig({}, {}, {}, {}, {})
        self.calibrate(rig)
        loaded = reload(rig)
        self.assertEqual(loaded.workflow.ordinary_cruise_threshold("x"), (2.5, "model"))
        loaded.owner.cruise_threshold_set, loaded.owner.cruise_threshold_g = set(), {"x": 0.0, "y": 0.0}
        loaded.home_results.extend([dict(MODEL, cruise_peak_g=7.96)] * 11)
        loaded.calibrate(save=0, START_THRESHOLD=7.5)
        self.assertEqual(len(loaded.home_calls), 11)
        self.assertNotIn("cruise_threshold_g", metadata(loaded))
        self.assertEqual(reload(loaded).workflow.ordinary_cruise_threshold("x"), (0.0, None))

    def test_models_saved_before_the_cruise_threshold_load_unchanged(self):
        rig = Rig()
        rig.calibrate(save=0, START_THRESHOLD=7.5)
        loaded = reload(rig)
        self.assertEqual(loaded.workflow.stale_model_axes, {})
        self.assertEqual(loaded.workflow.model.thresholds, {"x": 7.5})
        self.assertEqual(loaded.workflow.ordinary_cruise_threshold("x"), (0.0, None))

    def test_saved_value_must_be_positive_and_below_the_saved_threshold(self):
        rig = cruise_rig({}, {}, {}, {}, {})
        self.calibrate(rig)
        for value in (0, -1.0, 8.0, 9.0, True, "2.5"):
            with self.subTest(value=value):
                saved = {option: text for (section, option), text in rig.configfile.saved.items()
                         if section == SECTION}
                changed = dict(json.loads(saved["x_metadata"]), cruise_threshold_g=value)
                saved["x_metadata"] = json.dumps(changed)
                with self.assertRaisesRegex(ConfigError, "cruise_threshold_g"):
                    AccelHomingModel(Config(rig, saved, name=SECTION))

    def test_accuracy_recovery_rederives_or_drops_the_cruise_threshold(self):
        for peak, homes, expected in ((1.0, 17, 1.25), (2.5, 12, None)):
            with self.subTest(peak=peak), tempfile.TemporaryDirectory() as directory:
                rig = RecoveryRig({"charts_directory": directory})
                rig.owner.threshold_g["x"] = 2.25
                rig.home_results.extend([{"travel_mm": 51.175, "halt_travel_mm_signed": -53.175}] +
                                        [{"cruise_peak_g": peak} for _ in range(11)] +
                                        [dict(CHECK) for _ in range(5)])
                rig.accuracy()
                self.assertEqual(len(rig.home_calls), homes)
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "calibrated")
                self.assertEqual(result["saved_threshold_g"], 3.0)
                self.assertEqual(result["cruise_threshold_g"], expected)
                saved = json.loads(rig.configfile.saved[(SECTION, "x_metadata")])
                self.assertEqual(saved.get("cruise_threshold_g"), expected)
                self.assertEqual(saved["cruise_threshold_calibration"]["result"],
                                 "verified" if expected else "not_offered")
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)


class OtherCalibrationsTests(unittest.TestCase):
    def test_gentle_auto_and_beta_never_run_cruise_checks(self):
        def refuse():
            raise AssertionError("cruise threshold checks must not run")

        measured = {"cruise_peak_g": 0.2}
        with tempfile.TemporaryDirectory() as directory:
            rig = auto_tests.AutoRig(directory)
            rig.home_results.extend([dict(measured) for _ in range(30)])
            with mock.patch.object(rig.workflow, "_verify_cruise_threshold", side_effect=refuse), rig.helpers():
                rig.command(START_THRESHOLD=.5, STEP=.1)
            self.assertEqual(len(rig.home_calls), 22)
            self.assertEqual(rig.workflow.last_result["result"], "auto_notch_proposed")
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleAutoRig(directory)
            auto_tests.AutoNotchWorkflowTests().native_config(rig, Path(directory), NativeShape)
            rig.home_results.extend([dict(measured) for _ in range(30)])
            with mock.patch.object(rig.workflow, "_verify_cruise_threshold", side_effect=refuse), rig.helpers():
                rig.command(axis="X", bare=True, SAVE_CONFIG=0, START_THRESHOLD=2.25)
            self.assertEqual(len(rig.home_calls), 11)
            self.assertNotIn("cruise_threshold_calibration", rig.workflow.last_result)
        rig = BetaRig()
        rig.home_results.extend([dict(measured) for _ in range(11)])
        with mock.patch.object(rig.workflow, "_verify_cruise_threshold", side_effect=refuse), \
                rig.native_current():
            rig.beta("X")
        self.assertEqual(len(rig.home_calls), 11)
        self.assertEqual(rig.workflow.last_result["result"], "beta_proposed")

    def test_gentle_homing_ignores_every_cruise_threshold(self):
        rig = Rig({"gentle_homing": True})
        rig.owner.cruise_threshold_set, rig.owner.cruise_threshold_g = {"x"}, {"x": 3.0, "y": 0.0}
        self.assertEqual(rig.workflow.cruise_threshold("x"), 0.0)
        self.assertEqual(rig.workflow.ordinary_cruise_threshold("x"), (0.0, None))
        rig.workflow.session = {"mode": "calibration", "state": "running", "axis": "x",
                                "cruise_verification": {"threshold_g": 2.5, "active": False}}
        self.assertEqual(rig.workflow.cruise_threshold("x"), 0.0)


if __name__ == "__main__":
    unittest.main()
