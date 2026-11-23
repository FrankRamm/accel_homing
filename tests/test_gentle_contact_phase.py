"""Native-planner association for gentle qualification; simulated motion only."""
import copy
import gzip
import json
import tempfile
import unittest
from unittest import mock
import zipfile

from accel_homing_auto import _contact_phase_receipt
from harness import CommandError
from test_gentle_auto_workflow import GentleAutoRig


class GentleContactPhaseTests(unittest.TestCase):
    def test_native_interval_boundary_partial_history_and_identity(self):
        planner = {"verified_native_seek": True, "clock_domain": "main_mcu_print_time",
                   "axis": "x", "pass_number": 1, "arm_time_s": 10., "brake_start_time_s": 12.,
                   "native_home_end_time_s": 12.04,
                   "segments": [{"phase": phase, "start_time_s": start, "end_time_s": end}
                                for phase, start, end in (("acceleration", 10.01, 10.05),
                                    ("cruise", 10.05, 12.), ("braking", 12., 12.04))]}
        for onset, accepted, phase in ((10., False, None), (10.01, True, "acceleration"),
                (10.05, True, "cruise"), (11.999999999, True, "cruise"),
                (12., False, "braking"), (12.04, False, None)):
            with self.subTest(onset=onset):
                receipt = _contact_phase_receipt(planner, axis="x", pass_number=1, arm_time=10., onset_time=onset)
                self.assertEqual(receipt["accepted"], accepted)
                self.assertEqual(receipt["phase"], phase)
        partial = dict(planner, segments=planner["segments"][:2], brake_start_time_s=None)
        self.assertTrue(_contact_phase_receipt(partial, axis="x", pass_number=1, arm_time=10., onset_time=11.)["accepted"])
        for change in ({"axis": "y"}, {"pass_number": 2}, {"arm_time_s": 10.001},
                       {"verified_native_seek": False}, {"observation_count": 2}, {"segments": []}):
            with self.subTest(change=change):
                receipt = _contact_phase_receipt(dict(planner, **change), axis="x", pass_number=1,
                                                 arm_time=10., onset_time=11.)
                self.assertFalse(receipt["accepted"])

    def test_every_full_qualification_contact_has_matched_plan_and_candidate_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleAutoRig(directory)
            with rig.helpers():
                rig.command()
            result = rig.workflow.last_result
            self.assertEqual(len(result["artifacts"]["contact_timings"]), 11)
            self.assertFalse(hasattr(rig.owner, "contact_timing_observer"))
            with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                report = json.loads(bundle.read("report.json"))
                metadata = report["proposed_model"]
                self.assertEqual(metadata["gentle_contact_phase_policy"], "prebrake_v1")
                self.assertEqual(len(metadata["gentle_contact_timing"]), 11)
                self.assertEqual(len(report["gentle_contact_plans"]), 11)
                for number in range(1, 12):
                    receipt = json.loads(bundle.read("contact-%04d-timing.json" % number))
                    self.assertEqual(receipt["workflow_attempt"], number)
                    self.assertTrue(receipt["capture_matches_plan"])
                    self.assertTrue(receipt["qualification"]["accepted"])
                    self.assertEqual(receipt["qualification"]["phase"], "cruise")
                    self.assertEqual(receipt["signal"]["analysis_role"], "actual_candidate_replay")
                    self.assertTrue(receipt["signal"]["matches_recorded_live_filter"])
                    self.assertEqual(receipt["historical_signal"]["filter"]["frequency_hz"], 200.)
                    self.assertNotEqual(receipt["signal"]["filter"]["frequency_hz"], 200.)
                    raw = json.loads(gzip.decompress(bundle.read("raw/home-%04d.json.gz" % number)))
                    self.assertEqual(raw["native_seek"]["workflow_attempt"], number)
                    self.assertTrue(raw["native_seek"]["verified_native_seek"])
                    svg = bundle.read("contact-%04d-timing.svg" % number).decode()
                    self.assertIn("Recorded live detector replay", svg)
                    self.assertNotIn("Filtered 200 Hz / Q=5", svg)

    def test_missing_braking_mismatched_and_faulty_plans_abort_before_another_contact(self):
        for fault in ("missing", "braking", "tail", "before_motion", "pass", "arm", "fault", "duplicate"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                rig = GentleAutoRig(directory)
                original = copy.deepcopy(rig.owner.notch_profiles)
                if fault == "missing":
                    rig.observe_seek = False

                def mutate(attempt, home_end, moves, record):
                    if fault == "braking": record["onset_time_s"] = moves[-1].print_time + .005
                    elif fault == "tail": record["onset_time_s"] = moves[-1].print_time + moves[-1].move_t + .005
                    elif fault == "before_motion":
                        record["onset_time_s"] = attempt.arm_time + .001
                        for move in moves: move.print_time += .01
                    elif fault == "pass": attempt.pass_no = 2
                    elif fault == "arm": attempt.arm_time += .0005
                    elif fault == "fault": moves[1].x_r = 1.
                    elif fault == "duplicate":
                        rig.owner.contact_timing_observer(attempt, home_end)
                    return max(record["onset_time_s"] + .02, moves[0].print_time + .001)

                rig.native_plan_mutator = mutate
                with rig.helpers(), self.assertRaisesRegex(CommandError, "native|pre-braking|braking"):
                    rig.command()
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(rig.owner.notch_profiles, original)
                self.assertIsNone(rig.workflow.session)
                self.assertFalse(hasattr(rig.owner, "contact_timing_observer"))
                self.assertNotIn("x", rig.toolhead.kin.get_status(0)["homed_axes"])
                self.assertIn("archive", rig.workflow.last_result["artifacts"])

    def test_late_braking_crossing_preserves_prior_approach_and_never_retries(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleAutoRig(directory)

            def brake_second(attempt, home_end, moves, record):
                if rig.workflow.session["attempts"] == 2:
                    record["onset_time_s"] = moves[-1].print_time
                return record["onset_time_s"] + .02

            rig.native_plan_mutator = brake_second
            with rig.helpers(), self.assertRaisesRegex(CommandError, "braking"):
                rig.command()
            self.assertEqual(len(rig.home_calls), 2)
            self.assertEqual(rig.configfile.saved, {})
            with zipfile.ZipFile(rig.workflow.last_result["artifacts"]["archive"]) as bundle:
                report = json.loads(bundle.read("report.json"))
                self.assertTrue(report["gentle_contact_plans"][0]["qualification"]["accepted"])
                self.assertFalse(report["gentle_contact_plans"][1]["qualification"]["accepted"])
                self.assertEqual(len(report["sources"]), 4)

    def test_full_no_trigger_exports_failed_candidate_replay_and_root_peak_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleAutoRig(directory)
            home, capture = rig.stock_home, rig.owner.stop_capture

            def scheduled_end(attempt, home_end, moves, record):
                attempt.reason, attempt.request_time = None, None
                return moves[-1].print_time + moves[-1].move_t

            def missed(gcmd):
                home(gcmd)
                rig.owner.last["x"]["result"] = "no_trigger"
                raise CommandError("No trigger on x after full movement")

            def no_trigger_capture():
                data = capture()
                data["_record"].update(onset_time_s=None, onset_g=None, impact_peak_g=None)
                data["detector"].update(onset_time_s=None, trigger_onset_time_s=None, onset_g=None, impact_peak_g=None)
                arm = data["detector"]["arm_time_s"]
                data["samples"] = [(arm + (index - 1) / 1000., .6 if index == 3811 else .01, 0., 1.)
                                   for index in range(3901)]
                return data

            rig.native_plan_mutator = scheduled_end
            rig.stock_home, rig.owner.stop_capture = missed, no_trigger_capture
            with rig.helpers(), self.assertRaisesRegex(CommandError, "No trigger"):
                rig.command()
            self.assertEqual(len(rig.home_calls), 1)
            self.assertEqual(rig.configfile.saved, {})
            with zipfile.ZipFile(rig.workflow.last_result["artifacts"]["archive"]) as bundle:
                summary = json.loads(bundle.read("contact-summary.json"))["last_attempt"]
                timing = json.loads(bundle.read("contact-0001-timing.json"))
                self.assertEqual(summary["contact_result"], "no_trigger")
                self.assertEqual(summary["threshold_g"], rig.home_calls[0]["threshold"])
                self.assertIsNotNone(summary["peak_before_braking_g"])
                self.assertGreater(summary["peak_during_braking_g"], summary["peak_before_braking_g"])
                self.assertGreaterEqual(summary["peak_from_braking_through_tail_g"], summary["peak_during_braking_g"])
                self.assertTrue(summary["replay_matches_live_filter"])
                self.assertIsNone(summary["original_trigger_onset_time_s"])
                self.assertIn("contact-0001-timing.svg", bundle.namelist())
                self.assertEqual(timing["signal"]["analysis_role"], "actual_candidate_replay")


if __name__ == "__main__":
    unittest.main()
