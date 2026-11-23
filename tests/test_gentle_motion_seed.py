"""Simulated free-motion evidence and pre-contact gates for gentle calibration."""
import math
import tempfile
import unittest
from unittest import mock

from harness import CommandError
from test_beta_calibration import BetaRig
from test_gentle_auto_workflow import GentleAutoRig


class GentleSeedRig(GentleAutoRig):
    def __init__(self, directory):
        super().__init__(directory)

    def stop_motion(self, token, check):
        capture = BetaRig.stop_motion(self, token, check)
        capture.update(baseline_sample_count=len(token["baseline_samples"]),
                       baseline_first_sample_time_s=token["baseline_samples"][0][0],
                       baseline_last_sample_time_s=token["baseline_samples"][-1][0])
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
        # Exercise the public AUTO entry, real acquisition and real selector;
        # stop immediately before its first contact attempt.
        with self.helpers(), mock.patch.object(self.workflow, "_calibrate_trials") as contact:
            self.command(**params)
            contact.assert_called_once_with()
        return self.workflow.session


class GentleMotionSeedTests(unittest.TestCase):
    def test_swapped_signed_axis_is_the_only_noise_measurement_for_each_calibration(self):
        for axis, source in (("x", 1), ("y", 0)):
            with self.subTest(axis=axis), tempfile.TemporaryDirectory() as directory:
                rig = GentleSeedRig(directory)
                rig.amplitudes = [10., 10., 10.]
                rig.amplitudes[source] = .25

                def qualify(session):
                    rig.qualify_axis_mapping(session)
                    session["axis_mapping"].update(source_axis=source, sign=-1)

                rig.workflow._calibration_axis_mapping = qualify
                with mock.patch.object(rig.owner, "start_noise_measurement",
                                       wraps=rig.owner.start_noise_measurement) as stationary, \
                     mock.patch.object(rig.owner, "start_beta_motion_capture",
                                       wraps=rig.owner.start_beta_motion_capture) as moving:
                    session = rig.preflight(axis=axis.upper())
                mapping = session["axis_mapping"]
                stationary.assert_called_once_with(axis_mapping=mapping)
                self.assertEqual([call.kwargs["axis_mapping"] for call in moving.call_args_list], [mapping] * 3)
                self.assertEqual(session["recommendation"]["noise_peak_g"], {axis: .25})
                self.assertEqual(set(session["recommendation"]["start_threshold_g"]), {axis})
                self.assertEqual(session["candidate"], 3.75)
                self.assertNotIn("noise_magnitude_peak_g", session["stationary_noise"])
                for iteration, item in enumerate(session["iterations"], 1):
                    record = item["record"]
                    expected_baseline = -(.25, -.2)[source] * iteration
                    self.assertAlmostEqual(record["baseline_g"][axis], expected_baseline)
                    for key in ("baseline_g", "noise_peak_g", "noise_peak_to_peak_g", "noise_rms_g"):
                        self.assertEqual(set(record[key]), {axis})
                    self.assertNotIn("noise_magnitude_peak_g", record)
                    self.assertEqual(record["axis_mapping"], mapping)
                for item in session["beta_motion_iterations"]:
                    capture = item["capture"]
                    self.assertEqual(capture["sample_layout"], "mapped_axis_v1")
                    self.assertEqual(capture["axis_mapping"], mapping)
                    self.assertAlmostEqual(capture["baseline_g"], -(.2, -.3)[source])
                    self.assertTrue(all(len(row) == 2 for row in capture["samples"]))

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
            self.assertAlmostEqual(evidence["required_threshold_g"], 3.6)
            self.assertEqual(evidence["source"], "precontact_mapped_controls")
            self.assertEqual(evidence["signal_filter"], {"type": "disabled"})
            self.assertEqual(session["gentle_frozen_candidate"]["threshold_g"], 3.75)
            self.assertEqual(len(rig.analysis_seen), 1)
            self.assertNotIn("homes", rig.analysis_seen[0])
            self.assertEqual(evidence["axis_mapping"]["source_axis"], 2)
            self.assertEqual(len(evidence["iterations"]), 3)
            self.assertEqual(len([event for event in rig.beta_events if event[0] == "central_move"]), 6)
            self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
            self.assertEqual(rig.home_calls, [])
            self.assertEqual(rig.configfile.saved, {})
            self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.), (1., -1.)])
            self.assertEqual(rig.toolhead.accel, original_accel)

    def test_stationary_bound_and_explicit_minimum_are_preserved_without_grid_rounding(self):
        for stationary, explicit, expected in ((4., None, 4.8), (.01, 4., 4.)):
            with self.subTest(stationary=stationary), tempfile.TemporaryDirectory() as directory:
                rig = GentleSeedRig(directory)
                rig.amplitudes = (stationary, .005, .002)
                session = rig.preflight(**({} if explicit is None else {"START_THRESHOLD": explicit}))
                self.assertAlmostEqual(session["candidate"], expected)
                self.assertAlmostEqual(session["gentle_motion_noise"]["stationary_peak_g"], stationary)
                self.assertEqual(session["candidate"], session["gentle_frozen_candidate"]["threshold_g"])

    def test_low_explicit_minimum_cannot_bypass_precontact_motion_margin(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            session = rig.preflight(START_THRESHOLD=1.)
            self.assertEqual(session["candidate"], 3.75)
            self.assertGreaterEqual(session["candidate"], session["gentle_motion_noise"]["required_threshold_g"])
            self.assertEqual(session["start_threshold_g"], session["candidate"])
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
                with rig.helpers(), mock.patch.object(rig.workflow, "_calibrate_trials") as contact:
                    with self.assertRaises(CommandError):
                        rig.command()
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
            self.assertEqual(session["gentle_motion_noise"]["motion_peak_g"], 3.)
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

    def test_existing_filter_is_disabled_for_controls_and_restored_after_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = GentleSeedRig(directory)
            original = rig.owner.notch_profiles["x"].copy()
            observed = []
            rig.motion_mutator = lambda *_: observed.append(rig.owner.filter_settings("x"))
            with rig.helpers():
                rig.command(CONTACT_ONLY=1)
            self.assertEqual(observed, [{"type": "disabled"}] * 3)
            self.assertEqual(rig.owner.notch_profiles["x"], original)
            self.assertEqual([home["threshold"] for home in rig.home_calls], [3.75])
            self.assertEqual(rig.configfile.saved, {})

    def test_legacy_calibration_keeps_its_stationary_only_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = BetaRig({"charts_directory": directory})
            with rig.beta_helpers(), mock.patch.object(rig.workflow, "_calibrate_trials") as contact:
                rig.calibrate(save=0, START_THRESHOLD=.5)
                contact.assert_called_once_with()
            session = rig.workflow.session
            self.assertNotIn("gentle_motion_noise", session)
            self.assertEqual(session["candidate"], .5)
            self.assertEqual(rig.beta_events, [])


if __name__ == "__main__":
    unittest.main()
