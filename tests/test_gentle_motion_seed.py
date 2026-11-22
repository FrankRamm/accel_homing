"""Simulated free-motion evidence and pre-contact gates for gentle calibration."""
import math
import tempfile
import unittest
from unittest import mock

from harness import CommandError
from test_beta_calibration import BetaRig
from test_workflows import GCmd


class GentleSeedRig(BetaRig):
    def __init__(self, directory):
        super().__init__({"charts_directory": directory})
        self.workflow.gentle_homing = True
        self.owner.homing_accel = 1000.
        for rail in self.owner.rails_by_axis.values():
            rail.info.speed = 40.
        self.motion_mutator = None

    def stop_motion(self, token, check):
        capture = super().stop_motion(token, check)
        iteration = self.workflow.session["beta_current_motion"]["iteration"]
        source = self.workflow.session["axis_mapping"]["source_axis"]
        baseline = [.2, -.3, .8]
        capture["baseline_g"] = baseline
        rows = []
        for row in capture["samples"]:
            phase = next((segment for segment in self.profile
                          if segment.print_time <= row[0] < segment.print_time + segment.move_t), None)
            level = (12. if phase is not None and phase.accel < 0 else
                     [2., 3., 2.5][iteration - 1] if phase is not None and phase.accel > 0 else 1.)
            xyz = list(baseline)
            xyz[(source + 1) % 3] += 11.  # Diagnostic channel must not seed this axis.
            xyz[source] -= level
            rows.append((row[0], *xyz))
        capture["samples"] = rows
        if self.motion_mutator is not None:
            self.motion_mutator(capture, iteration)
        return capture

    def preflight(self, **params):
        with self.beta_helpers(), mock.patch.object(self.workflow, "_calibrate_trials") as contact:
            self.calibrate(save=0, **params)
            contact.assert_called_once_with()
        return self.workflow.session


class GentleMotionSeedTests(unittest.TestCase):
    def test_three_actual_strokes_seed_only_mapped_startup_accel_cruise(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            original_mapping = rig.qualify_axis_mapping

            def mapping(session):
                original_mapping(session)
                session["axis_mapping"].update(source_axis=2, sign=-1)

            rig.workflow._calibration_axis_mapping = mapping
            original_accel = rig.toolhead.accel
            session = rig.preflight()
            evidence = session["gentle_motion_noise"]
            self.assertTrue(evidence["complete"])
            self.assertEqual(evidence["motion_peak_g"], 3.)
            self.assertEqual(session["candidate"], 3.75)
            self.assertEqual(evidence["required_threshold_g"], 3.6)
            self.assertEqual(evidence["axis_mapping"]["source_axis"], 2)
            self.assertEqual(len(evidence["iterations"]), 3)
            self.assertEqual(len([event for event in rig.beta_events if event[0] == "central_move"]), 6)
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.), (1., -1.)])
            self.assertEqual(rig.toolhead.accel, original_accel)

    def test_stationary_bound_and_explicit_start_use_same_margin(self):
        for stationary, explicit, expected in ((4., None, 5.), (.01, 4., 4.)):
            with self.subTest(stationary=stationary), tempfile.TemporaryDirectory() as directory:
                rig = GentleSeedRig(directory)
                rig.amplitudes = (stationary, .005, .002)
                session = rig.preflight(**({} if explicit is None else {"START_THRESHOLD": explicit}))
                self.assertEqual(session["candidate"], expected)
                self.assertAlmostEqual(session["gentle_motion_noise"]["stationary_peak_g"], stationary)

    def test_explicit_low_threshold_is_rejected_before_any_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            with rig.beta_helpers(), mock.patch.object(rig.workflow, "_calibrate_trials") as contact:
                with self.assertRaisesRegex(CommandError, "START_THRESHOLD.*moving-noise"):
                    rig.calibrate(save=0, START_THRESHOLD=1.)
                contact.assert_not_called()
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.configfile.saved, {})

    def test_partial_failed_gapped_or_nonfinite_capture_never_reaches_contact(self):
        for failure in ("dropped", "fault", "validated", "gap", "nan", "context", "amplitude_limited"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = GentleSeedRig(directory)

                def mutate(capture, iteration):
                    if iteration != 2:
                        return
                    if failure == "dropped": capture["dropped"] = 1
                    elif failure == "fault": capture["fault"] = "no_data"
                    elif failure == "validated": capture["raw_samples_validated"] = False
                    elif failure == "amplitude_limited": capture["amplitude_limited"] = True
                    elif failure == "context": capture["sensor_context"] = {}
                    elif failure == "gap": del capture["samples"][20:30]
                    else:
                        row = list(capture["samples"][20])
                        row[1] = math.nan
                        capture["samples"][20] = row

                rig.motion_mutator = mutate
                with rig.beta_helpers(), mock.patch.object(rig.workflow, "_calibrate_trials") as contact:
                    with self.assertRaises(Exception):
                        rig.calibrate(save=0)
                    contact.assert_not_called()
                self.assertEqual(rig.home_calls, [])
                self.assertEqual(rig.configfile.saved, {})

    def test_auto_owned_motion_is_reused_and_missing_auto_evidence_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            session = rig.preflight()
            session["auto_notch"] = True
            with mock.patch.object(rig.workflow, "_beta_motion_noise") as collect:
                rig.workflow._gentle_collect_motion_noise(session)
                collect.assert_not_called()
            self.assertEqual(rig.workflow._gentle_motion_seed_peak(session), 3.)
            session["beta_motion_iterations"].pop()
            with self.assertRaisesRegex(CommandError, "three complete"):
                rig.workflow._gentle_motion_seed_peak(session)

    def test_resume_rejects_missing_changed_or_lowered_evidence(self):
        for failure in ("missing", "threshold", "sensor", "mapping", "speed", "filter"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                rig = GentleSeedRig(directory)
                session = rig.preflight()
                rig.workflow._gentle_validate_motion_seed(session)
                if failure == "missing": session.pop("gentle_motion_noise")
                elif failure == "threshold": session["candidate"] = 1.
                elif failure == "sensor": rig.owner.sensor_name = "changed"
                elif failure == "mapping": session["axis_mapping"]["source_axis"] = 1
                elif failure == "speed": rig.owner.rails_by_axis["x"].info.speed = 30.
                else: rig.owner.filter_settings = lambda axis: {"type": "changed"}
                with self.assertRaisesRegex(CommandError, "gentle calibration"):
                    rig.workflow._gentle_validate_motion_seed(session)

    def test_fixed_filter_fails_before_free_strokes_and_legacy_path_is_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            rig.owner.filter_settings = lambda axis: {"type": "fixed"}
            with self.assertRaisesRegex(CommandError, "NOTCH=AUTO"):
                rig.preflight()
            self.assertEqual(rig.beta_events, [])
            self.assertEqual(rig.home_calls, [])
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            rig.workflow.gentle_homing = False
            session = rig.preflight(START_THRESHOLD=.5)
            self.assertNotIn("gentle_motion_noise", session)
            self.assertEqual(session["candidate"], .5)
            self.assertEqual(rig.beta_events, [])


if __name__ == "__main__":
    unittest.main()
