import gzip
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
import zipfile

from harness import CommandError, arm_axis, make_homing, plugin
from accel_homing_auto import _contact_seek_segments
from test_gentle_auto_workflow import GentleAutoRig as AutoRig
from test_workflows import GCmd


class GentleContactProbeTests(unittest.TestCase):
    def test_one_contact_exports_trace_on_acceptance_and_rejection(self):
        for travel, accepted in ((150., True), (1., False)):
            with self.subTest(travel=travel), tempfile.TemporaryDirectory() as directory:
                rig = AutoRig(directory, {"gentle_homing": True})
                rig.owner.homing_accel = 1000.
                for rail in rig.owner.rails_by_axis.values():
                    rail.info.speed = 40.
                rig.home_results.append({"travel_mm": travel})
                original = rig.owner.notch_profiles["x"].copy()
                with rig.helpers():
                    if accepted:
                        rig.command(CONTACT_ONLY=1)
                    else:
                        with self.assertRaisesRegex(CommandError, "outside"):
                            rig.command(CONTACT_ONLY=1)
                result = rig.workflow.last_result
                self.assertEqual(len(rig.home_calls), 1)
                self.assertEqual(len(rig.analysis_seen), 1)
                self.assertTrue(rig.analysis_seen[0]["gentle_candidate"])
                self.assertNotIn("homes", rig.analysis_seen[0])
                self.assertEqual(rig.owner.notch_profiles["x"], original)
                self.assertEqual(rig.configfile.saved, {})
                self.assertEqual(result["result"], "contact_check_complete" if accepted else "aborted")
                with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                    report = json.loads(bundle.read("report.json"))
                    self.assertTrue(report["gentle_motion_noise"]["complete"])
                    self.assertEqual(len(report["retained_attempt_records"]), 1)
                    self.assertTrue(any("home" in item and item.endswith(".gz") for item in bundle.namelist()))
                    self.assertFalse(report["outcome"]["model_staged"])
                    candidate = report["gentle_frozen_candidate"]
                    self.assertEqual(rig.home_calls[0]["threshold"], candidate["threshold_g"])
                    self.assertEqual(rig.home_profiles[0]["profile"], candidate["profile"])

    def test_no_trigger_keeps_contact_samples_when_native_home_exits_with_queue_busy(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory, {"gentle_homing": True})
            rig.owner.homing_accel = 1000.
            rig.owner.rails_by_axis["x"].info.speed = 40.
            rig.observe_seek = False  # This test supplies the actual home_wait callback below.
            stock_home = rig.stock_home
            message = "No trigger on x after full movement"
            original_capture = rig.owner.stop_capture

            def capture():
                data = original_capture()
                data["auto_notch_replay"]["eligibility_source"] = "full_precision_source_time"
                data["detector"]["onset_time_s"] = None
                arm = data["detector"]["arm_time_s"]
                data["samples"] = [(arm + (i - 1) / 1000., 2. if i == 246 else 0., 0., 1.)
                                   for i in range(401)]
                return data
            rig.owner.stop_capture = capture

            def missed_contact(gcmd):
                stock_home(gcmd)
                record = rig.owner.last["x"]
                record.update(result="no_trigger", onset_g=None, impact_peak_g=None)
                arm = record["arm_time_s"]
                direction = 1. if rig.owner.rails_by_axis["x"].info.positive_dir else -1.
                def move(offset, duration, speed, accel):
                    return SimpleNamespace(print_time=arm + offset, move_t=duration, start_v=speed,
                                           accel=accel, x_r=direction, y_r=0., z_r=0.)
                rig.profile = [move(.01, .04, 0., 1000.), move(.05, .2, 40., 0.),
                               move(.25, .04, 40., -1000.)]
                attempt = SimpleNamespace(axis="x", pass_no=1, arm_time=arm, request_time=None, reason=None)
                rig.owner.disarm = lambda: attempt
                rig.owner.finish = lambda endstop, observed, result: 0.
                es = plugin.BeaconAccelEndstop(rig.owner, "x")
                es._dispatch = mock.Mock()
                self.assertEqual(es.home_wait(arm + .32), 0.)
                rig.profile = []  # later native position assignment can discard the seek
                rig.toolhead.busy = (20., 10., False)
                raise CommandError(message)

            rig.stock_home = missed_contact
            with rig.helpers(), self.assertRaisesRegex(CommandError, message):
                rig.command(CONTACT_ONLY=1)
            self.assertEqual(len(rig.home_calls), 1)
            self.assertEqual(rig.configfile.saved, {})
            result = rig.workflow.last_result
            self.assertEqual(result["result"], "aborted")
            with zipfile.ZipFile(result["artifacts"]["archive"]) as bundle:
                receipt = json.loads(bundle.read("run.json"))
                path = "raw/home-0001.json.gz"
                raw = json.loads(gzip.decompress(bundle.read(path)))
                self.assertEqual(raw["run_id"], receipt["run_id"])
                self.assertEqual(raw["phase"], "single_contact")
                self.assertEqual(raw["record"]["result"], "no_trigger")
                self.assertEqual(raw["error"]["message"], message)
                self.assertEqual(len(raw["capture"]["samples"]), 401)
                self.assertEqual([item["sample_count"] for item in receipt["captures"]
                                  if item["file"] == path], [401])
                self.assertEqual(receipt["errors"], [])
                diagnostic = json.loads(bundle.read("contact-timing.json"))
                self.assertEqual(diagnostic["signal"]["analysis_role"], "actual_candidate_replay")
                self.assertTrue(diagnostic["signal"]["matches_recorded_live_filter"])
                self.assertEqual(diagnostic["historical_signal"]["analysis_role"], "historical_fixed_200Hz_Q5_diagnostic_only")
                self.assertEqual(diagnostic["active_detector"]["candidate"]["profile"], rig.home_profiles[0]["profile"])
                self.assertTrue(diagnostic["capture_matches_plan"])
                self.assertEqual(diagnostic["contact_result"], "no_trigger")
                planner = diagnostic["planner"]
                self.assertEqual(planner["status"], "complete")
                self.assertEqual([s["phase"] for s in planner["segments"]], ["acceleration", "cruise", "braking"])
                self.assertIsNone(planner["stop_request_observed_time_s"])
                self.assertAlmostEqual(planner["brake_start_time_s"], planner["arm_time_s"] + .25)
                signal = diagnostic["signal"]
                self.assertEqual(signal["status"], "ok", signal)
                self.assertAlmostEqual(signal["brake_window"]["peak_minus_brake_ms"], -5.)
                self.assertIn("contact-timing.svg", bundle.namelist())
            self.assertFalse(hasattr(rig.owner, "contact_timing_observer"))

    def test_native_schedule_can_extend_past_trigger_and_rejects_unrelated_motion(self):
        def move(start, duration, speed, accel, direction=-1.):
            return SimpleNamespace(print_time=start, move_t=duration, start_v=speed, accel=accel,
                                   x_r=0., y_r=direction, z_r=0.)
        moves = [move(10., 0., 0., 0.), move(10.02, .04, 0., 1000.),
                 move(10.06, .2, 40., 0.), move(10.26, .04, 40., -1000.)]
        extractor = mock.Mock(return_value=(moves, []))
        segments = _contact_seek_segments(extractor, 10., 10.15, "y", -1., 9e9)
        self.assertEqual(segments[-1]["start_time_s"], 10.26)
        moves[-1].y_r = 1.
        with self.assertRaisesRegex(ValueError, "directional seek"):
            _contact_seek_segments(extractor, 10., 10.15, "y", -1., 9e9)

    def test_observer_failure_after_dispatch_stop_preserves_native_result(self):
        order = []
        def observe(attempt, end):
            order.append("observer")
            raise ValueError("unavailable")
        owner = SimpleNamespace(contact_timing_observer=observe, disarm=lambda: object(),
                                finish=lambda es, attempt, result: order.append("finish") or result)
        es = plugin.BeaconAccelEndstop(owner, "x")
        es._dispatch = SimpleNamespace(wait_end=lambda end: None, stop=lambda: order.append("stop") or 3)
        with self.assertLogs(level="ERROR"):
            self.assertEqual(es.home_wait(42.), 3)
        self.assertEqual(order, ["stop", "observer", "finish"])

    def test_beacon_observes_request_after_sending_the_same_trigger(self):
        code = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
        for enabled in (False, True):
            owner = make_homing()
            attempt, trigger = arm_axis(owner, "x", 200, 9000)
            if enabled:
                owner.contact_timing_observer = mock.Mock()
            def observed(now):
                self.assertEqual(trigger.messages, [[code]])
                return .215
            owner.main_mcu.estimated_print_time = observed
            owner._fire(attempt, "impact", code)
            self.assertEqual(trigger.messages, [[code]])
            self.assertEqual(attempt.request_time, .215 if enabled else None)

    def test_invalid_probe_options_fail_before_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = AutoRig(directory, {"gentle_homing": False})
            with self.assertRaisesRegex(CommandError, "requires gentle_homing"):
                rig.command(CONTACT_ONLY=1)
            rig.workflow.gentle_homing = True
            with self.assertRaisesRegex(CommandError, "SAVE_CONFIG=0"):
                rig.command(CONTACT_ONLY=1, SAVE_CONFIG=1)
            with self.assertRaisesRegex(CommandError, "requires NOTCH=AUTO"):
                rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, CONTACT_ONLY=1))
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.toolhead.moves, [])
