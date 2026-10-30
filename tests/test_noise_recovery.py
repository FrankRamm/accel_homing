"""Offline complete-window recovery and accuracy handoff; no printer access."""
import json
import tempfile
import unittest

from harness import CommandError, make_homing, plugin
from test_accuracy_recovery import RecoveryRig
from test_backends import generic, samples
from test_retract_noise import noise_flow
from test_retract_noise import NoiseRig
from test_workflows import GCmd


def sensor(decoded=False, legacy=False):
    obj = generic(legacy=legacy) if decoded else make_homing()
    obj._end_session()
    obj.reactor.now = 10.0
    return obj


def gap_flow(obj, failed_windows):
    """Insert a real timestamp discontinuity after some valid current data."""
    noise_flow(obj)
    valid = obj.reactor.on_pause
    failed = set()

    def emit():
        window = obj.noise_window
        if (window is not None and window["index"] in failed_windows
                and window["index"] not in failed and len(window["samples"]) >= 8):
            failed.add(window["index"])
            obj.reactor.now += .05
        valid()

    obj.reactor.on_pause = emit
    return failed


def install_noise_runtime(rig, decoded=False):
    """Keep the fake homing planner, but use the real stationary acquisition."""
    obj = sensor(decoded)
    obj.reactor = rig.reactor
    rig.owner.sensor_name = obj.sensor_name
    rig.owner.noise_sensor_context = obj.noise_sensor_context

    def start():
        rig.noise_calls.append("start")
        obj.operation_generation = rig.owner.operation_generation
        context = obj.start_noise_measurement()
        rig.owner.noise_session = obj.noise_session
        return context

    def collect(check_running, duration, **options):
        def current():
            obj.operation_generation = rig.owner.operation_generation
            check_running()
        result = obj.collect_noise_window(current, duration, **options)
        rig.noise_calls.append(len([call for call in rig.noise_calls if isinstance(call, int)]) + 1)
        return result

    def stop():
        rig.noise_calls.append("stop")
        obj.stop_noise_measurement()
        rig.owner.noise_session = None

    rig.owner.start_noise_measurement = start
    rig.owner.collect_noise_window = collect
    rig.owner.stop_noise_measurement = stop
    return obj


class StationaryNoiseRecoveryTests(unittest.TestCase):
    def test_third_window_gap_reacquires_whole_window_under_same_native_or_decoded_lease(self):
        for decoded, legacy in ((False, False), (True, False), (True, True)):
            with self.subTest(decoded=decoded, legacy=legacy):
                obj = sensor(decoded, legacy)
                gap_flow(obj, {3})
                obj.start_noise_measurement()
                token, generation = obj.noise_session, obj.operation_generation
                client = None if obj.backend is None else obj.backend.client
                captures = [obj.collect_noise_window(lambda: None, recover_transient=True)
                            for _ in range(3)]
                retry = captures[2]["acquisition_faults"][0]
                self.assertEqual(retry["reason"], "sample_gap")
                self.assertGreater(retry["discarded_sample_count"], 0)
                self.assertGreater(captures[2]["cutoff_time_s"], retry["last_sample_time_s"])
                self.assertGreater(captures[2]["samples"][0][0], captures[2]["cutoff_time_s"])
                self.assertGreaterEqual(captures[2]["samples"][-1][0] - captures[2]["samples"][0][0] + 1e-9, 1.)
                self.assertIs(obj.noise_session, token)
                self.assertEqual(obj.operation_generation, generation)
                self.assertEqual(token["windows_started"], 4)
                self.assertEqual(token["retries"], 1)
                self.assertIsNone(obj.sensor_fault)
                if obj.backend is None:
                    self.assertGreater(retry["gap_s"], retry["gap_limit_s"])
                    self.assertIn("batch_delta_clock", retry)
                else:
                    self.assertIs(obj.backend.client, client)
                    self.assertIsNone(obj.backend.fault)
                self.assertEqual(obj.toolhead.moves, [])
                self.assertEqual(obj.toolhead.dwells, [])
                obj.stop_noise_measurement()

    def test_per_window_and_shared_session_retry_budgets_are_finite(self):
        for decoded, failed_windows, complete, attempts in (
                (False, {1, 2, 3}, 0, 3), (True, {1, 2, 3}, 0, 3),
                (False, {1, 3, 5, 6}, 2, 6), (True, {1, 3, 5, 6}, 2, 6)):
            with self.subTest(decoded=decoded, failed_windows=failed_windows):
                obj = sensor(decoded)
                gap_flow(obj, failed_windows)
                obj.start_noise_measurement()
                accepted, faults = [], []
                with self.assertRaisesRegex(CommandError, "retry budget exhausted"):
                    for _ in range(3):
                        accepted.append(obj.collect_noise_window(lambda: None, recover_transient=True,
                                                                 on_acquisition_fault=faults.append))
                self.assertEqual(len(accepted), complete)
                self.assertEqual(obj.noise_session["windows_started"], attempts)
                self.assertFalse(faults[-1]["will_retry"])
                self.assertLessEqual(obj.noise_session["retries"], plugin.NOISE_SESSION_RETRIES)
                self.assertEqual(obj.sensor_fault, "sample_gap")
                self.assertIsNone(obj.noise_window)
                self.assertEqual(obj.toolhead.moves, [])
                obj.stop_noise_measurement()

    def test_missing_batches_can_resume_but_persistent_absence_aborts(self):
        for decoded in (False, True):
            for resume in (False, True):
                with self.subTest(decoded=decoded, resume=resume):
                    obj = sensor(decoded)
                    noise_flow(obj)
                    valid = obj.reactor.on_pause
                    obj.reactor.on_pause = lambda: valid() if resume and obj.noise_window["index"] > 1 else None
                    obj.start_noise_measurement()
                    faults = []
                    if resume:
                        capture = obj.collect_noise_window(lambda: None, recover_transient=True,
                                                           on_acquisition_fault=faults.append)
                        self.assertEqual(capture["acquisition_attempts"], 2)
                        self.assertEqual(faults[0]["discarded_sample_count"], 0)
                    else:
                        with self.assertRaisesRegex(CommandError, "retry budget exhausted"):
                            obj.collect_noise_window(lambda: None, recover_transient=True,
                                                     on_acquisition_fault=faults.append)
                        self.assertEqual(len(faults), 3)
                    self.assertEqual(faults[0]["reason"], "acquisition_timeout")
                    self.assertEqual(obj.toolhead.moves, [])
                    obj.stop_noise_measurement()

    def test_opt_in_never_retries_stale_future_duplicate_clipped_driver_or_frozen_data(self):
        for decoded in (False, True):
            for mode in ("stale", "future", "duplicate", "clip", "error", "frozen", "map"):
                if decoded and mode == "clip":
                    continue  # generic finite range readings are accepted; native 0x7FFF is not
                with self.subTest(decoded=decoded, mode=mode):
                    obj = sensor(decoded)
                    noise_flow(obj, mode)
                    obj.start_noise_measurement()
                    faults = []
                    with self.assertRaises(CommandError):
                        obj.collect_noise_window(lambda: None, recover_transient=True,
                                                 on_acquisition_fault=faults.append)
                    self.assertEqual(obj.noise_session["windows_started"], 1)
                    self.assertEqual(faults, [])
                    obj.stop_noise_measurement()

    def test_partial_stream_stalls_discard_the_window_and_require_fresh_live_data(self):
        for decoded in (False, True):
            for resume in (False, True):
                with self.subTest(decoded=decoded, resume=resume):
                    obj = sensor(decoded)
                    noise_flow(obj)
                    valid = obj.reactor.on_pause
                    emitted = set()

                    def flow():
                        index = obj.noise_window["index"]
                        if index not in emitted or (resume and index > 1):
                            emitted.add(index)
                            valid()

                    obj.reactor.on_pause = flow
                    obj.start_noise_measurement()
                    faults = []
                    if resume:
                        capture = obj.collect_noise_window(lambda: None, recover_transient=True,
                                                           on_acquisition_fault=faults.append)
                        self.assertEqual(capture["acquisition_attempts"], 2)
                        self.assertGreater(capture["cutoff_time_s"], faults[0]["last_sample_time_s"])
                    else:
                        with self.assertRaisesRegex(CommandError, "retry budget exhausted"):
                            obj.collect_noise_window(lambda: None, recover_transient=True,
                                                     on_acquisition_fault=faults.append)
                        self.assertEqual(len(faults), 3)
                    self.assertTrue(all(fault["reason"] == "no_fresh_data" for fault in faults))
                    self.assertTrue(all(fault["discarded_sample_count"] > 0 for fault in faults))
                    self.assertEqual(obj.toolhead.moves, [])
                    obj.stop_noise_measurement()

    def test_native_period_change_has_separate_numeric_gap_diagnostics(self):
        obj = sensor()
        noise_flow(obj)
        inspect = obj._inspect
        changed = [False]

        def alter_period(params):
            window = obj.noise_window
            if (window is not None and window["index"] == 1
                    and len(window["samples"]) >= 8 and not changed[0]):
                changed[0] = True
                params = dict(params, delta_clock=params["delta_clock"] * 1.2)
            inspect(params)

        obj._inspect = alter_period
        obj.start_noise_measurement()
        capture = obj.collect_noise_window(lambda: None, recover_transient=True)
        fault = capture["acquisition_faults"][0]
        self.assertLessEqual(fault["gap_s"], fault["gap_limit_s"])
        self.assertGreater(abs(fault["sample_interval_s"] - fault["previous_sample_interval_s"]),
                           fault["interval_change_limit_s"])
        self.assertEqual(capture["acquisition_attempts"], 2)
        obj.stop_noise_measurement()

    def test_cancel_context_generation_and_fatal_replacement_during_retry_are_never_cleared(self):
        for decoded in (False, True):
            for mode in ("cancel", "generation", "mapping", "client", "fatal", "homing", "beta"):
                with self.subTest(decoded=decoded, mode=mode):
                    obj = sensor(decoded)
                    gap_flow(obj, {1})
                    obj.start_noise_measurement()
                    token = obj.noise_session

                    def interrupted(diagnostic):
                        if mode == "cancel": obj.stop_noise_measurement()
                        elif mode == "generation": obj.operation_generation += 1
                        elif mode == "mapping":
                            if obj.backend is None: obj.helper.config.axes_map[0] = (1, 1)
                            else: obj.backend.sensor.axes_map = [(1, 1), (0, 1), (2, 1)]
                        elif mode == "client":
                            if obj.backend is None: obj.helper._stream_en = 1
                            else: obj.helper.add_client(lambda message: True)
                        elif mode == "fatal": obj._sensor_fault("data_error")
                        elif mode == "homing": obj.homing_active = True
                        else: obj.beta_capture = {}

                    with self.assertRaises(CommandError):
                        obj.collect_noise_window(lambda: None, recover_transient=True,
                                                 on_acquisition_fault=interrupted)
                    self.assertEqual(token["windows_started"], 1)
                    self.assertEqual(obj.sensor_fault, "data_error" if mode == "fatal" else "sample_gap")
                    self.assertEqual(obj.toolhead.moves, [])
                    obj.stop_noise_measurement()

    def test_beta_reference_and_default_collection_keep_strict_gap_behavior(self):
        for beta in (False, True):
            obj = sensor()
            gap_flow(obj, {1})
            with self.assertRaisesRegex(CommandError, "missing or irregular"):
                if beta:
                    obj.start_beta_motion_capture(lambda: None)
                else:
                    obj.start_noise_measurement()
                    obj.collect_noise_window(lambda: None)
            self.assertIsNone(obj.noise_window)
            self.assertEqual(obj.toolhead.moves, [])
            obj.stop_noise_measurement()

    def test_decoded_fatal_fault_cannot_be_overwritten_by_a_later_gap_before_polling(self):
        for fatal in ("invalid_data", "data_error", "stale_data"):
            with self.subTest(fatal=fatal):
                obj = sensor(decoded=True)
                noise_flow(obj)
                valid = obj.reactor.on_pause
                injected = [False]

                def emit():
                    valid()
                    if injected[0]: return
                    injected[0] = True
                    start = obj.backend.last_sample_time + 1. / 3200
                    if fatal == "invalid_data":
                        obj.helper.emit({"data": [(start, 0., 0.)]})
                    elif fatal == "data_error":
                        obj.helper.emit({"data": samples(start), "errors": 1})
                    else:
                        obj.reactor.now += .1
                        obj.helper.emit({"data": samples(start)})
                    self.assertEqual(obj.sensor_fault, fatal)
                    # Clear only the fake driver's cumulative count so the
                    # next packet actually exercises its timestamp gap path.
                    if fatal == "data_error": obj.backend.error_counts = (0, 0)
                    obj.helper.emit({"data": samples(obj.backend.last_sample_time + .05)})
                    self.assertEqual(obj.backend.fault, "sample_gap")

                obj.reactor.on_pause = emit
                obj.start_noise_measurement()
                faults = []
                with self.assertRaises(CommandError):
                    obj.collect_noise_window(lambda: None, recover_transient=True,
                                             on_acquisition_fault=faults.append)
                self.assertEqual(obj.sensor_fault, fatal)
                self.assertEqual(obj.noise_session["windows_started"], 1)
                self.assertEqual(faults, [])
                self.assertEqual(obj.toolhead.moves, [])
                obj.stop_noise_measurement()


class AccuracyStationaryRecoveryTests(unittest.TestCase):
    def test_context_change_during_committed_noise_announcement_clears_only_cached_hints(self):
        for ending in ("cancel", "generation"):
            with self.subTest(ending=ending), tempfile.TemporaryDirectory() as directory:
                rig = NoiseRig({"charts_directory": directory})
                show = rig.workflow._show_pngs

                def announce(paths, label):
                    show(paths, label)
                    if ending == "cancel": rig.workflow.cmd_CANCEL(GCmd())
                    else: rig.owner.operation_generation += 1

                rig.workflow._show_pngs = announce
                rig.workflow.cmd_NOISE(GCmd())
                self.assertEqual(rig.workflow.last_result["result"], "complete")
                self.assertTrue(rig.workflow.last_result["summary_paths"])
                self.assertEqual(rig.workflow.noise_seeds, {})
                self.assertIsNone(rig.workflow.noise_recommendation)
                self.assertIsNone(rig.workflow.session)

    def test_real_third_noise_window_gap_after_early_accuracy_reaches_ten_home_calibration(self):
        for decoded in (False, True):
            with self.subTest(decoded=decoded), tempfile.TemporaryDirectory() as directory:
                rig = RecoveryRig({"charts_directory": directory})
                rig.owner.threshold_g["y"] = 2.25
                rig.home_results.extend([{"travel_mm": 1.175, "halt_travel_mm_signed": -3.175}] + [{}] * 11)
                obj = install_noise_runtime(rig, decoded)
                gap_flow(obj, {3})
                captures = []
                collect = rig.owner.collect_noise_window

                def observed(check_running, duration, **options):
                    self.assertEqual(len(rig.home_calls), 1)
                    self.assertEqual(len(rig.toolhead.moves), 1)
                    capture = collect(check_running, duration, **options)
                    captures.append(capture)
                    return capture

                rig.owner.collect_noise_window = observed
                rig.accuracy(axis="Y", DETECTOR="RAW")
                result = rig.workflow.last_result
                self.assertEqual(result["result"], "calibrated")
                self.assertEqual(result["successes"], 10)
                self.assertEqual([call["threshold"] for call in rig.home_calls], [2.25] + [2.5] * 11)
                self.assertEqual(rig.scripts.count("SAVE_CONFIG"), 1)
                self.assertEqual(rig.noise_calls, ["start", 1, 2, 3, "stop"])
                self.assertEqual(len(captures), 3)
                faults = result["stationary_noise"]["acquisition_faults"]
                self.assertEqual(len(faults), 1)
                self.assertEqual(faults[0]["noise_iteration"], 3)
                self.assertEqual(faults[0]["reason"], "sample_gap")
                metadata = json.loads(rig.configfile.saved[("accel_homing_model default", "y_metadata")])
                self.assertEqual(metadata["stationary_noise_measurement"]["acquisition_faults"], faults)
                self.assertTrue(any("discarded" in line and "retry 1/2" in line for line in rig.output))
                self.assertIsNone(rig.workflow.session)
                self.assertIsNone(obj.noise_session)


if __name__ == "__main__":
    unittest.main()
