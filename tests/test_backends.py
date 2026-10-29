"""Offline contracts for configured sensor streams; no printer connection."""
import types
import unittest
from unittest import mock

from harness import (CommandError, ConfigError, FakeConfig, FakeEndstop, FakePrinter,
                     FakeToolhead, batch, counts, make_beacon, make_homing, plugin,
                     stationary_stream)
from accel_homing_backends import select_sensor
from test_session import FakeRail, info, with_endstops
from test_single_touch import RampStepper

G = 9806.65
HIT = plugin.mcu.MCU_trsync.REASON_ENDSTOP_HIT
HOST = plugin.mcu.MCU_trsync.REASON_HOST_REQUEST


class GenericPreparationTest(unittest.TestCase):
    """Real rails/move callbacks with decoded sensor delivery and fake clocks."""

    def session(self, legacy=False, backlog=False, options=None):
        obj = generic(legacy=legacy, options=options, connect=False)
        obj.fake_printer.send_event("klippy:connect")
        obj.threshold_g["x"] = 2.0
        endstop = with_endstops(obj)["x"]
        obj.reactor.now = 10.0
        stationary_stream(obj)
        plain = obj.reactor.on_pause
        called = [False]
        if backlog:
            def flow():
                if not called[0]:
                    called[0] = True
                    obj.backend.helper.emit({"data": samples(9.9, x=-15)})
                else:
                    plain()
            obj.reactor.on_pause = flow
        obj._handle_rails_begin(None, [FakeRail(endstop, info(speed=50, retract=0))])
        return obj, endstop

    def begin(self, obj, endstop):
        obj._handle_move_begin(types.SimpleNamespace(get_mcu_endstops=lambda: [endstop]))

    def test_existing_helper_backlog_is_drained_before_gate_without_poisoning(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, endstop = self.session(legacy=legacy, backlog=True)
                self.begin(obj, endstop)
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNotNone(obj.preparation.baseline())
                self.assertGreater(obj.preparation.first, obj.preparation.cutoff)
                self.assertEqual(endstop._dispatch.started, [])

    def test_late_pre_cutoff_peak_and_pending_are_excluded_from_reference(self):
        obj, endstop = self.session()
        obj.backend.pending = samples(9.9, size=1, x=-15)
        plain = obj.reactor.on_pause
        def flow():
            if obj.preparation is not None:
                obj.backend.helper.emit({"data": samples(9.9, x=-15)})
            plain()
        obj.reactor.on_pause = flow
        self.begin(obj, endstop)
        self.assertAlmostEqual(obj.preparation.baseline()[0], 0.0)
        self.assertIsNone(obj.sensor_fault)
        self.assertEqual(endstop._dispatch.started, [])

    def test_fault_or_noisy_generic_preparation_aborts_before_any_dispatch(self):
        for mode in ("noise", "no_data", "stale", "future", "duplicate", "gap",
                     "saturated", "nonfinite", "errors", "overflows"):
            with self.subTest(mode=mode):
                obj, endstop = self.session()
                sequence = [0]
                last = []
                def flow():
                    sequence[0] += 1
                    if mode == "no_data":
                        return
                    start = obj.reactor.now - 0.001 - 15 / 3200.0
                    x = (3 if sequence[0] % 2 else -3) if mode == "noise" else 0
                    fields = {}
                    if mode == "stale":
                        obj.reactor.now += 0.1
                    elif mode == "future":
                        start += 0.020
                    elif mode == "duplicate" and last:
                        start = last[0]
                    elif mode == "gap" and last:
                        start += 0.020
                        obj.reactor.now += 0.020
                    elif mode == "saturated":
                        x = 16.0
                    elif mode == "nonfinite":
                        x = float("nan")
                    elif mode in ("errors", "overflows"):
                        fields[mode] = 1
                    last.append(start)
                    obj.backend.helper.emit({"data": samples(start, x=x), **fields})
                obj.reactor.on_pause = flow
                with self.assertRaises(CommandError):
                    self.begin(obj, endstop)
                self.assertEqual(endstop._dispatch.started, [])
                self.assertEqual(endstop.get_mcu().log, [])
                self.assertEqual(obj.toolhead.moves, [])
                self.assertFalse(obj.homing_active)
                self.assertFalse(obj.backend.active)
                self.assertEqual(obj.backend.helper.batch_interval, 0.1)
                diagnostic = obj.get_status(obj.reactor.now)["baseline"]
                self.assertEqual(diagnostic["phase"], "acquisition")
                self.assertEqual(diagnostic["result"], obj.sensor_fault or "acquisition_timeout")
                if mode == "stale":
                    self.assertGreater(diagnostic["oldest_batch_age_ms"], diagnostic["max_sample_age_ms"])
                elif mode == "future":
                    self.assertLess(diagnostic["newest_batch_age_ms"], -5.0)

    def test_rounded_3200hz_reference_spans_small_deque_and_print_clock_offset(self):
        obj, endstop = self.session(options={"baseline_batches": 10})
        obj.threshold_g["x"] = 0.5
        # Decoded ADXL rows have microsecond-rounded print times, even when
        # host monotonic time and synchronized print time use different epochs.
        obj.main_mcu.estimated_print_time = lambda eventtime: eventtime + 123456.0
        def flow():
            now = obj.main_mcu.estimated_print_time(obj.reactor.now)
            rows = [(round(t, 6), x, y, z) for t, x, y, z in
                    samples(now - 0.001 - 15 / 3200.0, x=0.02, y=-0.03)]
            obj.backend.helper.emit({"data": rows})
        obj.reactor.on_pause = flow
        self.begin(obj, endstop)
        reference = obj.preparation
        self.assertGreaterEqual(reference.count, 321)
        self.assertGreaterEqual(reference.last - reference.first + 1e-9, 0.1)
        self.assertEqual(len(obj.recent), 10)
        self.assertAlmostEqual(reference.baseline()[0], 20.0)
        self.assertIsNone(obj.sensor_fault)
        self.assertEqual(endstop._dispatch.started, [])

    def test_timeout_retains_best_quiet_candidate_and_observed_noise(self):
        obj, endstop = self.session(options={"baseline_noise_floor": 0.0})
        obj.threshold_g["x"] = 0.5
        def flow():
            now = obj.reactor.now
            cutoff = obj.preparation.cutoff
            rows = [(round(t, 6), (0.2 if int((t - cutoff) / 0.090) % 2 else 0) * G, y, z)
                    for t, x, y, z in samples(now - 0.001 - 15 / 3200.0)]
            obj.backend.helper.emit({"data": rows})
        obj.reactor.on_pause = flow
        with self.assertRaisesRegex(CommandError, "acquisition timeout") as caught:
            self.begin(obj, endstop)
        diagnostic = obj.get_status(obj.reactor.now)["baseline"]
        self.assertEqual(diagnostic["result"], "acquisition_timeout")
        self.assertEqual(diagnostic["phase"], "acquisition")
        self.assertGreaterEqual(diagnostic["best_quiet_span_ms"], 80.0)
        self.assertLess(diagnostic["best_quiet_span_ms"], 100.0)
        self.assertGreater(diagnostic["best_quiet_samples"], diagnostic["quiet_samples"])
        self.assertGreater(diagnostic["quiet_resets"], 0)
        self.assertAlmostEqual(diagnostic["observed_peak_to_peak_g"]["x"], 0.2)
        self.assertAlmostEqual(diagnostic["quiet_limit_g"], 0.1)
        self.assertIn("g", str(caught.exception))
        self.assertIn("ms", str(caught.exception))
        self.assertFalse(obj.homing_active)
        self.assertIsNone(obj.preparation)
        self.assertEqual(endstop._dispatch.started, [])

    def test_timeout_differentiates_absent_from_pre_cutoff_only_data(self):
        for mode in ("absent", "pre_cutoff"):
            with self.subTest(mode=mode):
                obj, endstop = self.session()
                def flow():
                    if mode == "pre_cutoff":
                        obj.backend.helper.emit({"data": samples(obj.preparation.cutoff - 0.010)})
                obj.reactor.on_pause = flow
                with self.assertRaisesRegex(CommandError, "acquisition timeout"):
                    self.begin(obj, endstop)
                diagnostic = obj.get_status(obj.reactor.now)["baseline"]
                self.assertEqual(diagnostic["accepted_samples"], 0)
                self.assertEqual(diagnostic["quiet_samples"], 0)
                self.assertIsNone(diagnostic["sample_age_ms"])
                self.assertEqual(diagnostic["best_quiet_span_ms"], 0.0)
                self.assertEqual(diagnostic["pre_cutoff_samples"] > 0, mode == "pre_cutoff")
                self.assertEqual(endstop._dispatch.started, [])

    def test_ready_reference_expiry_before_arm_is_reported_separately(self):
        obj, endstop = self.session()
        self.begin(obj, endstop)
        obj.reactor.now += obj.backend.max_age + 0.010
        with self.assertRaisesRegex(CommandError, "expired before arm"):
            endstop.home_start(obj.reactor.now + 0.010, 0, 0, 0)
        diagnostic = obj.get_status(obj.reactor.now)["baseline"]
        self.assertEqual(diagnostic["result"], "expired_before_arm")
        self.assertEqual(diagnostic["phase"], "before_arm")
        self.assertGreaterEqual(diagnostic["best_quiet_span_ms"], 100.0)
        self.assertGreater(diagnostic["sample_age_ms"], diagnostic["max_sample_age_ms"])
        self.assertFalse(obj.homing_active)
        self.assertIsNone(obj.preparation)
        self.assertEqual(endstop._dispatch.started, [])

    def test_default_preparation_accepts_22_point_82_ms_sample_age(self):
        obj, endstop = self.session()
        def flow():
            start = obj.reactor.now - 0.02282 - 15 / 3200.0
            obj.backend.helper.emit({"data": samples(start)})
        obj.reactor.on_pause = flow
        self.begin(obj, endstop)
        self.assertIsNone(obj.sensor_fault)
        self.assertAlmostEqual(obj.baseline_status["sample_age_ms"], 22.82, delta=0.002)
        self.assertEqual(obj.baseline_status["max_sample_age_ms"], 50.0)
        self.assertEqual(obj.baseline_status["result"], "ready")
        self.assertEqual(endstop._dispatch.started, [])

    def test_first_single_post_arm_contact_survives_generic_batch_boundary(self):
        obj, endstop = self.session()
        self.begin(obj, endstop)
        when = obj.backend.last_sample_time + 16 / 3200.0
        endstop.home_start(when, 0, 0, 0)
        rows = samples(when - 15 / 3200.0, x=-15)
        rows[-1] = (when, -3 * G, 0, G)
        feed(obj, rows)
        self.assertIsNone(obj.attempt.reason)
        feed(obj, samples(when + 1 / 3200.0, size=1))
        self.assertEqual(obj.attempt.reason, "impact")
        self.assertAlmostEqual(obj.onset_time(obj.attempt), when, places=6)
        self.assertEqual(endstop.get_mcu().log, [("mcu", [7, HIT])])

    def test_pending_sample_age_is_rechecked_before_contact(self):
        obj, endstop = self.session()
        self.begin(obj, endstop)
        when = obj.backend.last_sample_time + 1 / 3200.0
        endstop.home_start(when, 0, 0, 0)
        feed(obj, samples(when, size=1, x=-3))
        self.assertIsNone(obj.attempt.reason)
        feed(obj, samples(when + 1 / 3200.0, size=2), age=obj.backend.max_age - 0.0003)
        self.assertEqual(obj.attempt.reason, "stale_data")
        self.assertEqual(endstop.get_mcu().log, [("mcu", [7, HOST])])

    def test_other_clients_survive_success_and_gate_failure(self):
        for failure in (False, True):
            with self.subTest(failure=failure):
                obj, endstop = self.session()
                seen = []
                other = lambda message: seen.append(message) or True
                obj.backend.helper.add_client(other)
                if failure:
                    obj.reactor.on_pause = lambda: obj.backend.helper.emit({"data": []})
                    with self.assertRaises(CommandError):
                        self.begin(obj, endstop)
                else:
                    self.begin(obj, endstop)
                    obj._end_session()
                obj.backend.helper.emit({"data": samples(obj.reactor.now)})
                self.assertIn(other, obj.backend.helper.client_cbs)
                self.assertTrue(seen)
                self.assertEqual(obj.backend.helper.batch_interval, 0.1)


class BaselineFloorTest(unittest.TestCase):
    """Constructed preparation and step histories, never a device recording."""

    def machine(self, profile="balanced", axes=(0,), options=None, epoch=10.0):
        # Keep the prior explicit 0.2 g policy covered after the default changes.
        obj = generic(options={"baseline_noise_floor": 0.2, **(options or {})})
        obj.threshold_g["x"] = 0.5
        obj.toolhead.get_max_velocity = lambda: (500.0, 5000.0)
        obj.reactor.now = epoch
        endstop = FakeEndstop("x")
        obj.session_axes["x"] = FakeRail(endstop, info(speed=50, retract=0, second=50))
        number = [0]
        def flow():
            now = obj.reactor.now
            start = now - 0.001 - 15 / 3200.0
            if profile == "pre_cutoff":
                start = obj.preparation.cutoff - 0.020
            elif profile == "stale":
                start -= obj.backend.max_age + 0.010
            elif profile == "future":
                start += 0.020
            rows = []
            for offset in range(16):
                sample = number[0] + offset
                sign = (1 if sample % 4 else -1) if profile == "asymmetric" else (
                    1 if (sample // 64 if profile == "slow" else sample) % 2 else -1)
                amplitude = 0.105 if profile == "excess" else 0.08 if profile == "slow" else 0.082
                values = [0.0, 0.0, 1.0]
                for axis in axes:
                    values[axis] += sign * amplitude
                rows.append((start + offset / 3200.0,) + tuple(value * G for value in values))
            number[0] += 16
            obj.backend.helper.emit({"data": rows})
        obj.reactor.on_pause = flow
        return obj, endstop

    def prepare_arm(self, obj, endstop, speed=50.0, accel=5000.0, direction=1):
        obj.toolhead.get_max_velocity = lambda: (500.0, accel)
        obj.session_axes["x"].info = info(speed=speed, retract=0, second=speed)
        obj._prepare_reference()
        self.assertGreaterEqual(obj.preparation.count, 321)
        self.assertGreaterEqual(obj.preparation.last - obj.preparation.first + 1e-9, 0.1)
        self.assertAlmostEqual(obj.baseline_status["quiet_limit_g"], 0.2)
        when = obj.backend.last_sample_time + 16 / 3200.0
        stepper = RampStepper(when, accel, speed)
        signed = types.SimpleNamespace(get_step_dist=stepper.get_step_dist,
                                       get_past_mcu_position=lambda stamp:
                                       direction * stepper.get_past_mcu_position(stamp))
        endstop.get_steppers = lambda: [signed]
        attempt = obj.prepare(endstop, when)
        obj.arm(attempt)
        self.assertAlmostEqual(attempt.limit_raw * attempt.count_g, 0.5)
        return attempt

    def parked(self, obj, attempt, bias=0.0, axes=(0,), max_samples=400):
        number = 0
        while number < max_samples and not attempt.done:
            first = obj.backend.last_sample_time + 1 / 3200.0
            rows = []
            for offset in range(16):
                values = [0.0, 0.0, 1.0]
                for axis in axes:
                    values[axis] += bias + (0.0002 if (number + offset) % 2 else -0.0002)
                rows.append((first + offset / 3200.0,) + tuple(value * G for value in values))
            feed(obj, rows)
            obj._watchdog(obj.reactor.now)
            number += 16

    def assert_startup_request_budget(self, obj, attempt):
        speed, accel = attempt.effective_speed, attempt.effective_accel
        timeout = obj.backend.motion_startup_timeout(speed, accel)
        elapsed = obj.reactor.now - attempt.arm_time
        # Delivery/polls remain 5 ms in this fixture; its actual request time
        # must still fit the existing deadline plus one 3 ms watchdog interval.
        self.assertLessEqual(elapsed, timeout + plugin.EARLY_INTERVAL + 0.000000001)
        travel = (0.5 * accel * elapsed ** 2 if elapsed < speed / accel
                  else speed * elapsed - 0.5 * speed ** 2 / accel)
        self.assertLessEqual(travel, 3.0)

    def test_floor_defaults_to_0_7_is_finite_bounded_and_preserves_explicit_values(self):
        default = generic()
        self.assertAlmostEqual(default.baseline_noise_floor, 0.7)
        default.threshold_g["x"] = 0.5
        self.assertAlmostEqual(default._reference_noise_limit("x"), 0.7)
        for value in (-0.001, 0.7001, float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                generic(options={"baseline_noise_floor": value})
        native = make_homing(options={"baseline_noise_floor": 0.7})
        native.threshold_g["x"] = 0.5
        self.assertAlmostEqual(native._reference_noise_limit("x"), 0.5)
        native.threshold_g["x"] = 0.3
        self.assertAlmostEqual(native._reference_noise_limit("x"), 0.3)
        for value in (0.0, 0.1, 0.2, 0.4, 0.7):
            obj = generic(options={"baseline_noise_floor": value})
            obj.threshold_g["x"] = 0.5
            self.assertAlmostEqual(obj._reference_noise_limit("x"), max(0.1, value))
            obj.threshold_g["x"] = 1.0
            self.assertAlmostEqual(obj._reference_noise_limit("x"), max(0.2, value))
            obj.threshold_g["x"] = 3.5
            self.assertAlmostEqual(obj._reference_noise_limit("x"), 0.7)
            obj.threshold_g["x"] = 8.0
            self.assertAlmostEqual(obj._reference_noise_limit("x"), 1.6)

    def test_balanced_0_164g_reference_completes_then_parked_trace_fails_motion(self):
        obj, endstop = self.machine(axes=(0, 1, 2))
        attempt = self.prepare_arm(obj, endstop)
        self.assertAlmostEqual(obj.baseline_status["observed_peak_to_peak_g"]["x"], 0.164)
        self.parked(obj, attempt, axes=(0, 1, 2))
        self.assertFalse(attempt.motion_seen)
        self.assertEqual(attempt.reason, "no_motion")
        self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_noisy_reference_retains_first_raw_sample_contact_above_half_g(self):
        obj, endstop = self.machine(axes=(0, 1, 2))
        attempt = self.prepare_arm(obj, endstop)
        first = obj.backend.last_sample_time + 1 / 3200.0
        rows = samples(first, size=32)
        contact = next(index for index, row in enumerate(rows) if row[0] >= attempt.arm_time)
        rows[contact] = (rows[contact][0], -0.51 * G, 0.0, G)
        feed(obj, rows)
        obj._watchdog(obj.reactor.now)
        self.assertEqual(attempt.reason, "impact")
        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
        self.assertAlmostEqual(obj.onset_time(attempt), rows[contact][0], places=6)

    def test_repeated_0_21g_excursions_still_reject_before_arm(self):
        obj, endstop = self.machine(profile="excess")
        with self.assertRaisesRegex(CommandError, "acquisition timeout"):
            obj._prepare_reference()
        self.assertFalse(obj.homing_active)
        self.assertEqual(endstop.trigger_log.messages, [])
        self.assertGreater(obj.baseline_status["quiet_resets"], 0)
        self.assertAlmostEqual(obj.baseline_status["quiet_limit_g"], 0.2)
        obj.threshold_g["x"] = 8.0  # Workflow cleanup restores ordinary settings.
        self.assertAlmostEqual(obj.baseline_status["impact_threshold_g"], 0.5)
        self.assertIn("trial impact threshold 0.5", plugin.describe_reference(obj.baseline_status))

    def test_half_g_floor_keeps_pre_cutoff_and_timestamp_rejections(self):
        for profile in ("pre_cutoff", "stale", "future"):
            with self.subTest(profile=profile):
                obj, endstop = self.machine(profile=profile)
                with self.assertRaises(CommandError):
                    obj._prepare_reference()
                self.assertFalse(obj.homing_active)
                self.assertEqual(endstop.trigger_log.messages, [])
                self.assertEqual(obj.baseline_status["accepted_samples"], 0)
                if profile == "pre_cutoff":
                    self.assertGreater(obj.baseline_status["pre_cutoff_samples"], 0)
                else:
                    self.assertEqual(obj.sensor_fault, "stale_data")

    def test_balanced_reference_then_changed_parked_bias_cannot_prove_motion(self):
        for axes in ((0,), (0, 1, 2)):
            with self.subTest(axes=axes):
                obj, endstop = self.machine(axes=axes)
                attempt = self.prepare_arm(obj, endstop)
                self.parked(obj, attempt, bias=0.08, axes=axes)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])
                self.assert_startup_request_budget(obj, attempt)

    def test_asymmetric_reference_uses_max_deviation_not_half_range(self):
        for axes in ((0,), (0, 1, 2)):
            with self.subTest(axes=axes):
                obj, endstop = self.machine(profile="asymmetric", axes=axes)
                attempt = self.prepare_arm(obj, endstop)
                self.assertGreater(attempt.proof_noise[0] * attempt.count_g, 0.12)
                self.assertGreater(obj.baseline_status["motion_proof_noise_bound_g"]["x"], 0.12)
                self.parked(obj, attempt, bias=-0.08, axes=axes)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(attempt.reason, "no_motion")

    def ramp(self, obj, attempt, direction=1, max_samples=400):
        number = 0
        while number < max_samples and not attempt.done:
            first = obj.backend.last_sample_time + 1 / 3200.0
            rows = []
            for offset in range(16):
                when = first + offset / 3200.0
                ramping = 0.0 <= when - attempt.arm_time < attempt.effective_speed / attempt.effective_accel
                acceleration = direction * attempt.effective_accel if ramping else 0.0
                noise = 0.0002 * G * (1 if (number + offset) % 2 else -1)
                rows.append((when, acceleration + noise, noise, G + noise))
            feed(obj, rows)
            obj._watchdog(obj.reactor.now)
            number += 16

    def test_real_50mm_s_ramps_prove_motion_in_both_directions_and_clock_epochs(self):
        # 4000 mm/s^2 is below .5 g in this ideal trace. A 5000 mm/s^2
        # ideal acceleration would itself exceed the unchanged .5 g detector.
        for direction in (-1, 1):
            for epoch in (10.0, 5000.0):
                with self.subTest(direction=direction, epoch=epoch):
                    obj, endstop = self.machine(axes=(0, 1, 2), epoch=epoch)
                    attempt = self.prepare_arm(obj, endstop, accel=4000.0, direction=direction)
                    self.ramp(obj, attempt, direction=direction)
                    self.assertTrue(attempt.motion_seen)
                    self.assertIsNone(attempt.reason)
                    self.assertEqual(endstop.trigger_log.messages, [])
                    self.assertEqual(attempt.auto_direction, -direction)

    def test_real_low_speed_with_insufficient_motion_evidence_fails_closed(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                obj, endstop = self.machine(axes=(0, 1, 2))
                attempt = self.prepare_arm(obj, endstop, speed=1.0, accel=1000.0, direction=direction)
                self.ramp(obj, attempt, direction=direction)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])
                self.assertLess(obj.reactor.now - attempt.arm_time, 0.1)

    def test_coherent_integration_snapshot_counts_first_and_odd_samples(self):
        obj, endstop = self.machine()
        attempt = self.prepare_arm(obj, endstop)
        rows = [(attempt.arm_time + (number - 15) / 3200.0, 0.0, 0.0, G)
                for number in range(32)]
        post_arm = sum(row[0] >= attempt.arm_time for row in rows)
        self.assertEqual(post_arm, 17)
        feed(obj, rows)
        previous = attempt.proof_state
        self.assertEqual(previous[1], post_arm)
        self.assertEqual(previous[0], tuple(attempt.sums))
        feed(obj, samples(obj.backend.last_sample_time + 1 / 3200.0, size=3))
        self.assertEqual(attempt.proof_state[1], post_arm + 3)
        self.assertEqual(previous[1], post_arm)
        self.assertIsNot(previous, attempt.proof_state)

    def test_uncertain_biased_speed_cannot_mask_either_first_contact_direction(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                obj, endstop = self.machine(profile="slow")
                attempt = self.prepare_arm(obj, endstop)
                self.parked(obj, attempt, bias=0.08, max_samples=160)
                self.assertGreater(obj._velocity(attempt), 30.0)
                self.assertFalse(attempt.motion_seen)
                self.assertEqual(attempt.auto_direction, 0)
                self.assertIsNone(attempt.reason)
                self.assertLess(obj.reactor.now - attempt.arm_time, 0.055)
                rows = samples(obj.backend.last_sample_time + 1 / 3200.0)
                rows[0] = (rows[0][0], (attempt.baseline * attempt.count_g + direction * 0.51) * G, 0.0, G)
                feed(obj, rows)
                self.assertEqual(attempt.reason, "impact")
                self.assertEqual(endstop.trigger_log.messages, [[HIT]])
                self.assertAlmostEqual(obj.onset_time(attempt), rows[0][0], places=6)

    def test_unexpanded_configured_references_keep_original_raw_motion_path(self):
        obj, endstop = self.machine()
        obj.threshold_g["x"] = 1.0
        obj._prepare_reference()
        when = obj.backend.last_sample_time + 16 / 3200.0
        endstop.get_steppers = lambda: [RampStepper(when, 5000.0, 50.0)]
        attempt = obj.prepare(endstop, when)
        self.assertIsNone(attempt.proof_noise)
        self.assertIsNone(attempt.proof_state)
        self.assertIsNone(obj.baseline_status["motion_proof_noise_bound_g"])

    def test_slow_reference_then_parked_bias_must_not_prove_motion(self):
        # This experiment intentionally checks the risk of admitting a wider
        # stationary range: preparation and parked samples contain no motion.
        for axes in ((0,), (0, 1, 2)):
            with self.subTest(axes=axes):
                obj, endstop = self.machine(profile="slow", axes=axes)
                attempt = self.prepare_arm(obj, endstop)
                self.parked(obj, attempt, bias=0.08, axes=axes)
                self.assertFalse(attempt.motion_seen,
                                 "parked signal falsely proved motion: measured %.2f mm/s, baseline %r"
                                 % (obj._measured_speed(attempt), attempt.bases))
                self.assertEqual(attempt.reason, "no_motion")
                self.assertEqual(endstop.trigger_log.messages, [[HOST]])
                self.assert_startup_request_budget(obj, attempt)


class RaisedBaselineFloorTest(unittest.TestCase):
    """Synthetic 0.7 g admission and unchanged impact/motion checks, not printer data."""

    def machine(self, threshold=0.5, axis="x", spreads=(0.581, 0.392, 0.258),
                asymmetric=False, legacy=False, options=None):
        obj = generic(legacy=legacy, options=options)
        obj.threshold_g[axis] = threshold
        obj.toolhead.get_max_velocity = lambda: (500.0, 5000.0)
        obj.reactor.now = 10.0
        endstop = FakeEndstop(axis)
        obj.session_axes[axis] = FakeRail(endstop, info(speed=50, retract=0, second=50))
        number = [0]
        obj.fixture_reference_rows = []
        def flow():
            start = (obj.preparation.cutoff if obj.backend.last_sample_time is None
                     else obj.backend.last_sample_time) + 1 / 3200.0
            rows = []
            for offset in range(16):
                sample = number[0] + offset
                sign = (1 if sample % 4 else -1) if asymmetric else (1 if sample % 2 else -1)
                values = [sign * spread / 2.0 for spread in spreads]
                values[2] += 1.0
                rows.append((start + offset / 3200.0,) + tuple(value * G for value in values))
            number[0] += 16
            obj.fixture_reference_rows.extend(rows)
            obj.backend.helper.emit({"data": rows})
        obj.reactor.on_pause = flow
        return obj, endstop

    def prepare_arm(self, obj, endstop, speed=50.0, accel=5000.0, direction=1):
        obj.toolhead.get_max_velocity = lambda: (500.0, accel)
        obj.session_axes[endstop.axis].info = info(speed=speed, retract=0, second=speed)
        obj._prepare_reference()
        self.assertGreaterEqual(obj.preparation.count, 321)
        self.assertGreaterEqual(obj.preparation.last - obj.preparation.first + 1e-9, 0.1)
        self.assertAlmostEqual(obj.baseline_status["quiet_limit_g"], 0.7)
        # Driver timestamps are rounded to microseconds. Select the independently
        # emitted rows belonging to the retained reference, whose odd sample
        # count need not leave an alternating waveform's mean at its midpoint.
        retained = [row for row in obj.fixture_reference_rows
                    if obj.preparation.first - 1e-6 <= row[0] <= obj.preparation.last + 1e-6]
        self.assertEqual(len(retained), obj.preparation.count)
        means = [sum(row[index + 1] / G for row in retained) / len(retained)
                 for index in range(3)]
        obj.fixture_reference_envelope = tuple(
            max(abs(row[index + 1] / G - mean) for row in retained)
            for index, mean in enumerate(means))
        when = obj.backend.last_sample_time + 16 / 3200.0
        stepper = RampStepper(when, accel, speed)
        endstop.get_steppers = lambda: [types.SimpleNamespace(
            get_step_dist=stepper.get_step_dist,
            get_past_mcu_position=lambda stamp: direction * stepper.get_past_mcu_position(stamp))]
        attempt = obj.prepare(endstop, when)
        obj.arm(attempt)
        self.assertAlmostEqual(attempt.limit_raw * attempt.count_g, obj.threshold_g[endstop.axis])
        self.assertIsNotNone(attempt.proof_noise)
        for index, mean in enumerate(means):
            self.assertAlmostEqual(attempt.bases[index] * attempt.count_g, mean)
        return attempt

    def test_reported_ranges_qualify_full_duration_without_changing_impact_threshold(self):
        for threshold in (0.5, 1.0):
            for legacy in (False, True):
                with self.subTest(threshold=threshold, legacy=legacy):
                    obj, endstop = self.machine(threshold=threshold, legacy=legacy)
                    attempt = self.prepare_arm(obj, endstop)
                    diagnostic = obj.baseline_status
                    self.assertEqual(diagnostic["result"], "ready")
                    self.assertEqual(diagnostic["quiet_resets"], 0)
                    self.assertAlmostEqual(diagnostic["impact_threshold_g"], threshold)
                    for index, (axis, spread) in enumerate(zip("xyz", (0.581, 0.392, 0.258))):
                        self.assertAlmostEqual(diagnostic["observed_peak_to_peak_g"][axis], spread)
                        self.assertAlmostEqual(attempt.proof_noise[index] * attempt.count_g,
                                               obj.fixture_reference_envelope[index])
                    self.assertEqual(endstop.trigger_log.messages, [])
                    self.assertEqual(obj.toolhead.moves, [])

    def test_above_0_7_on_each_axis_rejects_before_arm(self):
        for index in range(3):
            with self.subTest(axis=index):
                spreads = [0.581, 0.392, 0.258]
                spreads[index] = 0.702
                obj, endstop = self.machine(spreads=spreads)
                with self.assertRaisesRegex(CommandError, "acquisition timeout"):
                    obj._prepare_reference()
                self.assertIsNone(obj.attempt)
                self.assertIsNone(obj.preparation)
                self.assertFalse(obj.homing_active)
                self.assertFalse(obj.backend.active)
                self.assertGreater(obj.baseline_status["quiet_resets"], 0)
                self.assertAlmostEqual(obj.baseline_status["quiet_limit_g"], 0.7)
                self.assertEqual(endstop.trigger_log.messages, [])
                self.assertEqual(obj.toolhead.moves, [])

    def test_newly_admitted_reference_keeps_raw_half_and_one_g_contacts_in_both_directions(self):
        for threshold in (0.5, 1.0):
            for axis in ("x", "y"):
                for direction in (-1, 1):
                    with self.subTest(threshold=threshold, axis=axis, direction=direction):
                        obj, endstop = self.machine(threshold=threshold, axis=axis)
                        attempt = self.prepare_arm(obj, endstop)
                        bases = [value * attempt.count_g for value in attempt.bases]
                        first = obj.backend.last_sample_time + 1 / 3200.0
                        rows = samples(first, size=32, x=bases[0], y=bases[1], z=bases[2])
                        post = next(index for index, row in enumerate(rows) if row[0] >= attempt.arm_time)
                        row = list(rows[post])
                        row[attempt.index + 1] += direction * (threshold - 0.01) * G
                        rows[post] = tuple(row)
                        feed(obj, rows)
                        self.assertIsNone(attempt.reason)
                        rows = samples(obj.backend.last_sample_time + 1 / 3200.0,
                                       x=bases[0], y=bases[1], z=bases[2])
                        row = list(rows[0])
                        row[attempt.index + 1] += direction * (threshold + 0.01) * G
                        rows[0] = tuple(row)
                        feed(obj, rows)
                        self.assertEqual(attempt.reason, "impact")
                        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
                        self.assertAlmostEqual(obj.onset_time(attempt), rows[0][0], places=6)

    def test_stationary_reported_noise_and_asymmetric_bias_cannot_prove_motion(self):
        for threshold in (0.5, 1.0):
            for asymmetric in (False, True):
                for bias in (-0.8, 0.0, 0.8):
                    with self.subTest(threshold=threshold, asymmetric=asymmetric, bias=bias):
                        obj, endstop = self.machine(threshold=threshold, asymmetric=asymmetric)
                        attempt = self.prepare_arm(obj, endstop)
                        if asymmetric:
                            self.assertAlmostEqual(attempt.proof_noise[0] * attempt.count_g,
                                                   obj.fixture_reference_envelope[0])
                            self.assertGreater(attempt.proof_noise[0] * attempt.count_g, 0.581 / 2.0)
                        number = 0
                        while number < 512 and not attempt.done:
                            first = obj.backend.last_sample_time + 1 / 3200.0
                            rows = []
                            for offset in range(16):
                                sign = 1 if (number + offset) % 2 else -1
                                deltas = [bias * bound * attempt.count_g + sign * 0.0001
                                          for bound in attempt.proof_noise]
                                if bias == 0.0:
                                    deltas = [sign * spread / 2.0 for spread in (0.581, 0.392, 0.258)]
                                values = [(base * attempt.count_g + delta) * G
                                          for base, delta in zip(attempt.bases, deltas)]
                                rows.append((first + offset / 3200.0,) + tuple(values))
                            feed(obj, rows)
                            obj._watchdog(obj.reactor.now)
                            number += 16
                        self.assertFalse(attempt.motion_seen)
                        self.assertFalse(attempt.moving)
                        self.assertEqual(attempt.auto_direction, 0)
                        self.assertEqual(attempt.reason, "no_motion")
                        self.assertEqual(endstop.trigger_log.messages, [[HOST]])
                        timeout = obj.backend.motion_startup_timeout(50.0, 5000.0)
                        self.assertLessEqual(obj.reactor.now - attempt.arm_time,
                                             timeout + plugin.EARLY_INTERVAL + 1e-9)

    def test_true_ramp_with_sufficient_noise_margin_still_proves_motion(self):
        # These constructed 200 mm/s, 6000 mm/s^2 ramps have enough velocity
        # evidence after subtracting the admitted envelope. Lower-speed traces
        # may correctly remain ambiguous; this is not a hardware qualification.
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                obj, endstop = self.machine(threshold=1.0)
                attempt = self.prepare_arm(obj, endstop, speed=200.0, accel=6000.0, direction=direction)
                number = 0
                while number < 512 and not attempt.done and not attempt.motion_seen:
                    first = obj.backend.last_sample_time + 1 / 3200.0
                    rows = []
                    for offset in range(16):
                        when = first + offset / 3200.0
                        sign = 1 if (number + offset) % 2 else -1
                        values = [sign * spread / 2.0 * G for spread in (0.581, 0.392, 0.258)]
                        if 0.0 <= when - attempt.arm_time < 200.0 / 6000.0:
                            values[0] += direction * 6000.0
                        values[2] += G
                        rows.append((when,) + tuple(values))
                    feed(obj, rows)
                    obj._watchdog(obj.reactor.now)
                    number += 16
                self.assertTrue(attempt.motion_seen)
                self.assertTrue(attempt.moving)
                self.assertEqual(attempt.auto_direction, -direction)
                self.assertIsNone(attempt.reason)
                self.assertEqual(endstop.trigger_log.messages, [])


class CalibrationReferenceTest(unittest.TestCase):
    """Only active calibration omits the quiet-range requirement."""

    def context(self, obj, axis="x"):
        obj.workflows.session = {"mode": "calibration", "state": "running", "axis": axis}
        obj.workflows.in_home, obj.workflows.home_axis = True, axis

    def machine(self, threshold=1.0, axis="x", legacy=False):
        obj, endstop = RaisedBaselineFloorTest().machine(
            threshold=threshold, axis=axis, legacy=legacy, spreads=(1.313, 0.604, 0.281))
        self.context(obj, axis)
        return obj, endstop

    def prepare_arm(self, obj, endstop):
        obj._prepare_reference()
        reference = obj.preparation
        self.assertFalse(reference.require_quiet)
        self.assertGreaterEqual(reference.count, 321)
        self.assertGreaterEqual(reference.last - reference.first + 1e-9, 0.1)
        retained = [row for row in obj.fixture_reference_rows
                    if reference.first - 1e-6 <= row[0] <= reference.last + 1e-6]
        self.assertEqual(len(retained), reference.count)
        means = tuple(sum(row[index + 1] / G for row in retained) / len(retained)
                      for index in range(3))
        envelope = tuple(max(abs(row[index + 1] / G - means[index]) for row in retained)
                         for index in range(3))
        when = obj.backend.last_sample_time + 16 / 3200.0
        endstop.get_steppers = lambda: [RampStepper(when, 5000.0, 50.0)]
        attempt = obj.prepare(endstop, when)
        obj.arm(attempt)
        for index in range(3):
            self.assertAlmostEqual(attempt.bases[index] * attempt.count_g, means[index])
            self.assertAlmostEqual(attempt.proof_noise[index] * attempt.count_g, envelope[index])
        return attempt

    def test_noisy_calibration_retains_full_reference_and_envelope_at_each_candidate(self):
        for threshold in (1.0, 1.25, 8.0):
            for axis in ("x", "y"):
                for legacy in (False, True):
                    with self.subTest(threshold=threshold, axis=axis, legacy=legacy):
                        obj, endstop = self.machine(threshold, axis, legacy)
                        attempt = self.prepare_arm(obj, endstop)
                        diagnostic = obj.baseline_status
                        self.assertFalse(diagnostic["quiet_range_enforced"])
                        self.assertEqual(diagnostic["quiet_resets"], 0)
                        self.assertEqual(diagnostic["result"], "ready")
                        self.assertAlmostEqual(diagnostic["impact_threshold_g"], threshold)
                        self.assertAlmostEqual(attempt.limit_raw * attempt.count_g, threshold)
                        for name, spread in zip("xyz", (1.313, 0.604, 0.281)):
                            self.assertAlmostEqual(diagnostic["best_quiet_peak_to_peak_g"][name], spread)
                        detail = plugin.describe_reference(diagnostic)
                        self.assertIn("best reference", detail)
                        self.assertIn("quiet range not required for calibration", detail)
                        self.assertNotIn("best quiet", detail)
                        self.assertEqual(endstop.trigger_log.messages, [])
                        self.assertEqual(obj.toolhead.moves, [])

    def test_ordinary_and_other_workflow_contexts_still_require_quiet_xyz(self):
        for mode in ("ordinary", "test", "charts", "accuracy", "waiting",
                     "wrong_session_axis", "wrong_home_axis", "not_in_home"):
            with self.subTest(mode=mode):
                obj, endstop = self.machine()
                if mode == "ordinary":
                    obj.workflows.session = None
                elif mode in ("test", "charts", "accuracy"):
                    obj.workflows.session["mode"] = mode
                elif mode == "waiting":
                    obj.workflows.session["state"] = "waiting"
                elif mode == "wrong_session_axis":
                    obj.workflows.session["axis"] = "y"
                elif mode == "wrong_home_axis":
                    obj.workflows.home_axis = "y"
                else:
                    obj.workflows.in_home = False
                with self.assertRaisesRegex(CommandError, "acquisition timeout"):
                    obj._prepare_reference()
                self.assertTrue(obj.baseline_status["quiet_range_enforced"])
                self.assertIsNone(obj.attempt)
                self.assertFalse(obj.homing_active)
                self.assertEqual(endstop.trigger_log.messages, [])

    def test_calibration_reference_cannot_be_armed_after_scope_exit_or_expiry(self):
        for mode in ("scope_exit", "waiting", "expiry"):
            with self.subTest(mode=mode):
                obj, endstop = self.machine()
                obj._prepare_reference()
                if mode == "scope_exit":
                    obj.workflows.session = None
                    obj.workflows.in_home, obj.workflows.home_axis = False, None
                elif mode == "waiting":
                    obj.workflows.session["state"] = "waiting"
                else:
                    obj.reactor.now += obj.backend.max_age + 0.010
                with self.assertRaisesRegex(CommandError, "fresh accelerometer reference"):
                    obj.prepare(endstop, obj.reactor.now + 0.010)
                self.assertEqual(obj.baseline_status["result"],
                                 "expired_before_arm" if mode == "expiry"
                                 else "calibration_reference_outside_trial")
                self.assertFalse(obj.homing_active)
                self.assertIsNone(obj.preparation)
                self.assertEqual(endstop.trigger_log.messages, [])

    def test_noisy_calibration_cannot_turn_stationary_noise_or_bias_into_motion(self):
        for threshold in (1.0, 8.0):
            for bias in (0.0, 0.8):
                with self.subTest(threshold=threshold, bias=bias):
                    obj, endstop = self.machine(threshold)
                    attempt = self.prepare_arm(obj, endstop)
                    number = 0
                    while number < 512 and not attempt.done:
                        first = obj.backend.last_sample_time + 1 / 3200.0
                        rows = []
                        for offset in range(16):
                            sign = 1 if (number + offset) % 2 else -1
                            deltas = [bias * bound * attempt.count_g + sign * 0.0001
                                      for bound in attempt.proof_noise]
                            if bias == 0.0:
                                deltas = [sign * spread / 2.0 for spread in (1.313, 0.604, 0.281)]
                            values = [(base * attempt.count_g + delta) * G
                                      for base, delta in zip(attempt.bases, deltas)]
                            rows.append((first + offset / 3200.0,) + tuple(values))
                        feed(obj, rows)
                        obj._watchdog(obj.reactor.now)
                        number += 16
                    self.assertFalse(attempt.motion_seen)
                    self.assertFalse(attempt.moving)
                    self.assertEqual(attempt.auto_direction, 0)
                    self.assertEqual(attempt.reason, "no_motion")
                    self.assertEqual(endstop.trigger_log.messages, [[HOST]])

    def test_first_raw_contact_uses_selected_one_and_one_quarter_g_candidates(self):
        for threshold in (1.0, 1.25):
            for axis in ("x", "y"):
                for direction in (-1, 1):
                    with self.subTest(threshold=threshold, axis=axis, direction=direction):
                        obj, endstop = self.machine(threshold, axis)
                        attempt = self.prepare_arm(obj, endstop)
                        bases = [value * attempt.count_g for value in attempt.bases]
                        first = obj.backend.last_sample_time + 1 / 3200.0
                        rows = samples(first, size=32, x=bases[0], y=bases[1], z=bases[2])
                        post = next(index for index, row in enumerate(rows) if row[0] >= attempt.arm_time)
                        row = list(rows[post])
                        row[attempt.index + 1] += direction * (threshold - 0.01) * G
                        rows[post] = tuple(row)
                        feed(obj, rows)
                        self.assertIsNone(attempt.reason)
                        rows = samples(obj.backend.last_sample_time + 1 / 3200.0,
                                       x=bases[0], y=bases[1], z=bases[2])
                        row = list(rows[0])
                        row[attempt.index + 1] += direction * (threshold + 0.01) * G
                        rows[0] = tuple(row)
                        feed(obj, rows)
                        self.assertEqual(attempt.reason, "impact")
                        self.assertEqual(endstop.trigger_log.messages, [[HIT]])
                        self.assertAlmostEqual(obj.onset_time(attempt), rows[0][0], delta=1e-6)

    def test_calibration_data_faults_and_capacity_still_abort_before_dispatch(self):
        for mode in ("stale", "future", "duplicate", "gap", "saturated", "nonfinite",
                     "errors", "overflows", "capacity"):
            with self.subTest(mode=mode):
                obj, endstop = GenericPreparationTest().session()
                obj.threshold_g["x"] = 1.0
                self.context(obj)
                number, last = [0], []
                def flow():
                    number[0] += 1
                    start = obj.reactor.now - 0.001 - 15 / 3200.0
                    x, fields = 0.65 if number[0] % 2 else -0.65, {}
                    if mode == "stale":
                        obj.reactor.now += 0.1
                    elif mode == "future":
                        start += 0.020
                    elif mode == "duplicate" and last:
                        start = last[0]
                    elif mode == "gap" and last:
                        start += 0.020
                        obj.reactor.now += 0.020
                    elif mode == "saturated":
                        x = 16.0
                    elif mode == "nonfinite":
                        x = float("nan")
                    elif mode in ("errors", "overflows"):
                        fields[mode] = 1
                    last.append(start)
                    obj.backend.helper.emit({"data": samples(start, x=x), **fields})
                obj.reactor.on_pause = flow
                capacity = 50 if mode == "capacity" else plugin.BASELINE_MAX_SAMPLES
                with mock.patch.object(plugin, "BASELINE_MAX_SAMPLES", capacity):
                    with self.assertRaises(CommandError):
                        obj._prepare_reference()
                self.assertFalse(obj.baseline_status["quiet_range_enforced"])
                self.assertEqual(obj.baseline_status["result"], obj.sensor_fault)
                self.assertEqual(endstop._dispatch.started, [])
                self.assertEqual(endstop.get_mcu().log, [])
                self.assertEqual(obj.toolhead.moves, [])
                self.assertFalse(obj.homing_active)


class BatchHelper:
    """The real bulk_sensor callback/False-unsubscribe contract."""
    def __init__(self):
        self.batch_interval = 0.1
        self.batch_timer = object()
        self.client_cbs = []

    def add_client(self, callback):
        self.client_cbs.append(callback)

    def emit(self, message):
        for callback in list(self.client_cbs):
            if not callback(message):
                self.client_cbs.remove(callback)


class LegacyHelper:
    """Klipper v0.12's web request / connection interface, not add_client(cb)."""
    def __init__(self):
        self.update_interval = 0.1
        self.update_timer = object()
        self.clients = {}

    def add_client(self, request):
        connection = request.get_client_connection()
        self.clients[connection] = request.get_dict("response_template", {})

    def emit(self, message):
        for connection in list(self.clients):
            if connection.is_closed():
                del self.clients[connection]
            else:
                connection.send({"params": message})


def generic(name="adxl345 toolhead", options=None, rate=3200, legacy=False, connect=True):
    helper = LegacyHelper() if legacy else BatchHelper()
    sensor = types.SimpleNamespace(data_rate=rate, axes_map=[(1, -123), (0, 123), (2, 123)])
    setattr(sensor, "api_dump" if legacy else "batch_bulk", helper)
    printer = FakePrinter(FakeToolhead(), make_beacon())
    printer.objects[name] = sensor
    cfg = FakeConfig(printer, {"accel_chip": name, **(options or {})},
                     sections=(name,), name="accel_homing")
    obj = plugin.BeaconAccelHoming(cfg)
    obj.fake_printer = printer
    if connect:
        printer.send_event("klippy:connect")
        obj.homing_active = True
        obj._start_stream()
    return obj


def samples(start, size=16, hz=3200.0, x=0.0, y=0.0, z=1.0):
    return [(start + n / hz, x * G, y * G, z * G) for n in range(size)]


def feed(obj, rows, age=0.001, **fields):
    obj.fake_printer.reactor.now = rows[-1][0] + age
    obj.backend.helper.emit({"data": rows, **fields})


def baseline(obj, start=10.0):
    for n in range(12):
        feed(obj, samples(start + n * 0.005))


def arm(obj, axis="x", when=10.060, deadline=20.0):
    endstop = FakeEndstop(axis)
    clock = obj.backend.print_time_to_clock
    attempt = plugin.Attempt(endstop, 1, when, clock(when) & 0xFFFFFFFF,
                             deadline, clock(deadline) & 0xFFFFFFFF,
                             plugin.AXES[axis], 1, obj.impact_sign[axis],
                             obj.threshold_g[axis] / obj.backend.count_g, obj.backend.count_g)
    obj.arm(attempt)
    return attempt, endstop.trigger_log


class SensorSelectionTest(unittest.TestCase):
    def config(self, sections, options=None, name="accel_homing"):
        return FakeConfig(None, options or {}, sections=sections, name=name)

    def test_named_standard_sensor_is_selected_unambiguously(self):
        self.assertEqual(select_sensor(self.config(("adxl345 toolhead",))), "adxl345 toolhead")

    def test_ambiguous_auto_requires_explicit_full_section(self):
        cfg = self.config(("beacon", "adxl345 toolhead"))
        with self.assertRaisesRegex(ConfigError, "accel_chip"):
            select_sensor(cfg)
        cfg.options["accel_chip"] = "adxl345 toolhead"
        self.assertEqual(select_sensor(cfg), "adxl345 toolhead")

    def test_legacy_beacon_default_is_preserved_with_other_sensors(self):
        self.assertEqual(select_sensor(self.config(("beacon", "adxl345"),
                                                  name="beacon_accel_homing")), "beacon")

    def test_probe_only_cartographer_is_not_an_acceleration_stream(self):
        with self.assertRaisesRegex(ConfigError, "found none"):
            select_sensor(self.config(("cartographer", "scanner")))

    def test_cartographer_standard_adxl_section_is_supported(self):
        self.assertEqual(select_sensor(self.config(("cartographer", "adxl345"))), "adxl345")

    def test_missing_configured_sensor_rejected(self):
        with self.assertRaisesRegex(ConfigError, "no \\[adxl345 missing\\]"):
            select_sensor(self.config(("adxl345",), {"accel_chip": "adxl345 missing"}))

    def test_duplicate_old_and_new_sections_rejected(self):
        printer = FakePrinter(FakeToolhead(), make_beacon())
        with self.assertRaisesRegex(ConfigError, "only one"):
            plugin.BeaconAccelHoming(FakeConfig(printer, {}, sections=("beacon", "accel_homing")))

    def test_unknown_stream_driver_needs_actual_range(self):
        with self.assertRaisesRegex(ConfigError, "sensor_range"):
            generic("custom_accel toolhead")
        obj = generic("custom_accel toolhead", {"sensor_range": 16})
        self.assertEqual(obj.backend.full_scale, 16)


class GenericTriggerTest(unittest.TestCase):
    def test_normalized_mm_s2_triggers_with_independent_thresholds(self):
        for axis, amplitude, expected in (("x", -7, HIT), ("y", -7, None), ("y", -12, HIT)):
            obj = generic(options={"x_threshold": 6, "y_threshold": 11,
                                   "x_impact_sign": -1, "y_impact_sign": -1})
            baseline(obj)
            attempt, trigger = arm(obj, axis)
            rows = samples(10.060)
            row = list(rows[7])
            row[plugin.AXES[axis] + 1] = amplitude * G
            rows[7] = tuple(row)
            feed(obj, rows)
            self.assertEqual(trigger.messages, [] if expected is None else [[expected]])
            if expected:
                self.assertAlmostEqual(obj.onset_time(attempt), rows[7][0], places=6)

    def test_already_mapped_driver_data_is_not_mapped_twice(self):
        obj = generic(options={"x_impact_sign": -1})
        baseline(obj)
        _attempt, trigger = arm(obj)
        feed(obj, samples(10.060, x=12.0))
        self.assertEqual(trigger.messages, [])
        feed(obj, samples(10.065, x=-12.0))
        self.assertEqual(trigger.messages, [[HIT]])

    def test_biased_stationary_baseline_is_subtracted(self):
        obj = generic(options={"x_threshold": 5, "x_impact_sign": -1})
        for n in range(12):
            feed(obj, samples(10.0 + n * 0.005, x=-1.0))
        attempt, trigger = arm(obj)
        feed(obj, samples(10.060, x=-5.9))
        self.assertEqual(trigger.messages, [])
        feed(obj, samples(10.065, x=-6.1))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertAlmostEqual(attempt.baseline, -1000.0)

    def test_batch_straddling_arm_keeps_only_post_arm_trigger(self):
        obj = generic()
        baseline(obj)
        attempt, trigger = arm(obj, when=10.0625)
        rows = samples(10.060)
        rows[1] = (rows[1][0], -15 * G, 0, G)  # Before arming: never a hit.
        rows[12] = (rows[12][0], -12 * G, 0, G)
        feed(obj, rows)
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertAlmostEqual(obj.onset_time(attempt), rows[12][0], places=6)

    def test_clock_wrap_and_long_printer_uptime(self):
        obj = generic()
        base = 4294.94
        baseline(obj, base)
        attempt, trigger = arm(obj, when=base + 0.060, deadline=base + 10.0)
        feed(obj, samples(base + 0.060, x=-12))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertAlmostEqual(obj.onset_time(attempt), base + 0.060, places=6)

    def test_generic_never_infers_pressed_from_absent_motion(self):
        obj = generic()
        baseline(obj)
        attempt, trigger = arm(obj)
        attempt.steppers = [types.SimpleNamespace(
            get_past_mcu_position=lambda t: t * 5000, get_step_dist=lambda: 0.01)]
        for start in (10.060, 10.065, 10.070):
            feed(obj, samples(start))
            obj._check_pressed(attempt)
        self.assertEqual(trigger.messages, [])
        self.assertIsNone(obj._motion_state(attempt))

    def test_frozen_stream_fails_instead_of_homing_at_start(self):
        obj = generic()
        baseline(obj)
        attempt, trigger = arm(obj)
        attempt.steppers = [types.SimpleNamespace(
            get_past_mcu_position=lambda t: t * 5000, get_step_dist=lambda: 0.01)]
        for n in range(7):
            feed(obj, samples(10.060 + n * 0.005))
        self.assertEqual(attempt.reason, "frozen")
        self.assertEqual(trigger.messages, [[HOST]])


class WarmupContinuityTest(unittest.TestCase):
    """Discarded warmup history must not create or conceal a retained-data gap."""

    def seeded(self, legacy=False, singleton=False):
        # Preserve the original 20 ms warmup-cutoff scenarios explicitly.
        obj = generic(legacy=legacy, options={"max_sample_age": 0.020})
        rows = samples(9.990, size=1 if singleton else 16, x=2.0)
        feed(obj, rows)
        return obj, rows[-1][0]

    def test_continuous_delayed_batch_starts_new_retained_sequence(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, previous = self.seeded(legacy)
                # Every source interval is 0.3125 ms. Warmup intentionally
                # discards the prefix older than 10.010 s; comparing its fresh
                # suffix with 9.9946875 s formerly fabricated a sample_gap.
                rows = samples(previous + 1 / 3200.0, size=106)
                obj.reactor.now = 10.030
                cutoff = obj.reactor.now - obj.backend.max_age
                obj.backend.helper.emit({"data": rows})
                retained = [row for row in rows if row[0] >= cutoff]
                self.assertIsNone(obj.sensor_fault)
                self.assertAlmostEqual(obj.backend.observed_rate, 3200.0)
                self.assertEqual(len(obj.recent[-1]), 3 * len(retained))
                self.assertEqual(obj.backend.last_sample_time, rows[-1][0])
                # The former warmup fault poisoned an otherwise empty reference.
                obj.session_axes = {"x": None}
                stationary_stream(obj)
                obj._prepare_reference()
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNotNone(obj.preparation.baseline())

    def test_stale_pending_singleton_is_discarded_before_fresh_batch(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, previous = self.seeded(legacy, singleton=True)
                self.assertEqual(len(obj.backend.pending), 1)
                obj.reactor.now = 10.030
                rows = samples(10.0105)
                obj.backend.helper.emit({"data": rows})
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(obj.backend.pending, [])
                self.assertEqual(len(obj.recent[-1]), 3 * len(rows))
                self.assertEqual(obj.backend.last_xyz, rows[-1][1:])
                self.assertEqual(obj.backend.unchanged_since, rows[0][0])

    def test_all_old_batch_clears_obsolete_tracking_without_refreshing_data(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, previous = self.seeded(legacy)
                receipt = obj.last_batch
                obj.reactor.now = 10.030
                obj.backend.helper.emit({"data": samples(previous + 1 / 3200.0)})
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNone(obj.backend.last_sample_time)
                self.assertIsNone(obj.backend.last_xyz)
                self.assertIsNone(obj.backend.unchanged_since)
                self.assertEqual(obj.backend.pending, [])
                self.assertEqual(obj.last_batch, receipt)
                obj.reactor.now = 10.035
                obj.backend.helper.emit({"data": samples(10.016)})
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(obj.last_batch, 10.035)

    def test_old_prefix_cannot_clear_fresh_history_or_a_gap_between_retained_rows(self):
        for legacy in (False, True):
            for stale_history in (False, True):
                with self.subTest(legacy=legacy, stale_history=stale_history):
                    obj, previous = self.seeded(legacy)
                    if stale_history:
                        obj.reactor.now = 10.030
                        # Include an old row between two retained groups. A
                        # reset at every discarded row would conceal this gap.
                        rows = (samples(9.998, size=2) + samples(10.011, size=2)
                                + samples(9.8, size=1) + samples(10.016))
                    else:
                        obj.reactor.now = 10.005
                        rows = samples(9.98, size=2) + samples(previous + 0.002)
                    obj.backend.helper.emit({"data": rows})
                    self.assertEqual(obj.sensor_fault, "sample_gap")

    def test_warmup_drain_preserves_latched_faults_and_rejects_invalid_data(self):
        for legacy in (False, True):
            for mode in ("latched", "shape", "nonfinite", "errors", "overflows"):
                with self.subTest(legacy=legacy, mode=mode):
                    obj, previous = self.seeded(legacy)
                    obj.reactor.now = 10.030
                    rows, fields = samples(10.011), {}
                    if mode == "latched":
                        obj.backend._fail("sample_gap")
                    elif mode == "shape":
                        rows.insert(0, (9.8, 0.0, G))
                    elif mode == "nonfinite":
                        rows.insert(0, (9.8, float("nan"), 0.0, G))
                    else:
                        fields[mode] = 1
                    obj.backend.helper.emit({"data": rows, **fields})
                    expected = ("sample_gap" if mode == "latched" else "invalid_data"
                                if mode in ("shape", "nonfinite") else "data_error")
                    self.assertEqual(obj.sensor_fault, expected)


class NoiseWindowBoundaryTest(unittest.TestCase):
    """Decoded callbacks retain only samples belonging to the measured window."""

    def session(self, legacy=False, cutoff=9.990):
        # These regression timestamps straddle the original explicit 20 ms limit.
        obj = generic(legacy=legacy, options={"max_sample_age": 0.020})
        obj._end_session()
        obj.reactor.now = 10.0
        obj.start_noise_measurement()
        return obj, self.window(obj, cutoff)

    def window(self, obj, cutoff):
        window = {"cutoff": cutoff, "duration": 1.0, "samples": [], "last": None,
                  "interval": None, "complete": False, "changed": False}
        obj.noise_window = window
        obj.backend.prepare_reference()
        return window

    def test_mixed_boundary_batch_ignores_old_rows_before_age_and_continuity(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, window = self.session(legacy=legacy)

                class DelayedRows(list):
                    def __iter__(self):
                        # Reproduce 1 ms spent processing a callback: the old
                        # 9.9805 row ages from 19.5 to 20.5 ms, but every row
                        # belonging to the new window remains younger than 11 ms.
                        obj.reactor.now = 10.001
                        return super().__iter__()

                rows = samples(9.9805, size=61)
                obj.backend.helper.emit({"data": DelayedRows(rows)})
                expected = [row for row in rows if row[0] > window["cutoff"]]
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(len(window["samples"]), len(expected))
                self.assertGreater(window["samples"][0][0], window["cutoff"])
                self.assertAlmostEqual(obj.backend.last_age, 0.00175)

    def test_all_old_boundary_rows_and_exact_cutoff_are_ignored(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, window = self.session(legacy=legacy)
                # Repeated and out-of-order rows from before the new window
                # cannot set its continuity reference or become fresh data.
                rows = samples(9.7, size=2) + samples(9.990, size=1) + samples(9.6, size=2)
                obj.backend.helper.emit({"data": rows})
                self.assertIsNone(obj.sensor_fault)
                self.assertIsNone(obj.backend.last_sample_time)
                self.assertIsNone(obj.last_batch)
                self.assertEqual(window["samples"], [])
                obj.backend.helper.emit({"data": samples(9.9903125)})
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(len(window["samples"]), 16)

    def test_current_window_oldest_row_cannot_be_hidden_by_fresh_tail(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, window = self.session(legacy=legacy, cutoff=9.9)
                # Homing warmup trimming would discard the old prefix and
                # accept this batch, hiding lost data inside a measured window.
                obj.backend.helper.emit({"data": samples(9.970, size=94)})
                self.assertEqual(obj.sensor_fault, "stale_data")
                self.assertEqual(window["samples"], [])

    def test_current_window_future_duplicate_and_gap_still_fail(self):
        for legacy in (False, True):
            for mode in ("future", "duplicate", "gap"):
                with self.subTest(legacy=legacy, mode=mode):
                    obj, window = self.session(legacy=legacy)
                    rows = samples(9.9903125)
                    if mode == "future":
                        rows = samples(10.006)
                    else:
                        obj.backend.helper.emit({"data": rows})
                        start = rows[-1][0] + (0.0 if mode == "duplicate" else 0.002)
                        rows = samples(start)
                        obj.reactor.now = 10.005
                    obj.backend.helper.emit({"data": rows})
                    expected = {"future": "stale_data", "duplicate": "invalid_data",
                                "gap": "sample_gap"}[mode]
                    self.assertEqual(obj.sensor_fault, expected)

    def test_current_window_pending_sample_keeps_its_age_limit(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, window = self.session(legacy=legacy)
                obj.backend.helper.emit({"data": samples(9.9905, size=1)})
                self.assertEqual(len(obj.backend.pending), 1)
                obj.reactor.now = 10.011
                obj.backend.helper.emit({"data": samples(9.9908125, size=1)})
                self.assertEqual(obj.sensor_fault, "stale_data")
                self.assertEqual(window["samples"], [])

    def test_between_windows_drain_pending_and_continuity_before_next_window(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, window = self.session(legacy=legacy)
                obj.backend.helper.emit({"data": samples(9.9905, size=1)})
                self.assertEqual(len(obj.backend.pending), 1)
                for completed in (True, False):
                    window["complete"] = completed
                    obj.noise_window = window if completed else None
                    obj.reactor.now += 0.1
                    obj.backend.helper.emit({"data": samples(9.8)})
                    self.assertIsNone(obj.sensor_fault)
                    self.assertEqual(obj.backend.pending, [])
                    self.assertIsNone(obj.backend.last_sample_time)
                next_window = self.window(obj, 10.2)
                obj.reactor.now = 10.206
                obj.backend.helper.emit({"data": samples(10.2003125)})
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(len(next_window["samples"]), 16)

    def test_cutoff_filters_pending_before_continuity_even_without_callback_gap(self):
        obj, window = self.session()
        obj.backend.pending = samples(9.98, size=1)
        obj.backend.last_sample_time = 9.98
        obj.backend.helper.emit({"data": samples(9.9903125)})
        self.assertIsNone(obj.sensor_fault)
        self.assertEqual(len(window["samples"]), 16)
        self.assertEqual(obj.backend.pending, [])

    def test_draining_keeps_malformed_rows_and_driver_errors_fatal(self):
        for legacy in (False, True):
            for mode in ("shape", "nonfinite", "errors", "overflows"):
                with self.subTest(legacy=legacy, mode=mode):
                    obj, window = self.session(legacy=legacy)
                    obj.noise_window = None
                    rows, fields = samples(9.8), {}
                    if mode == "shape":
                        rows[0] = (9.8, 0.0, G)
                    elif mode == "nonfinite":
                        rows[0] = (9.8, float("nan"), 0.0, G)
                    else:
                        fields[mode] = 1
                    obj.backend.helper.emit({"data": rows, **fields})
                    self.assertEqual(obj.sensor_fault, "invalid_data" if mode in
                                     ("shape", "nonfinite") else "data_error")


class StreamIntegrityTest(unittest.TestCase):
    def test_default_armed_sample_age_accepts_22_point_82_ms_and_rejects_over_50_ms(self):
        for oldest_age, expected in ((0.02282, HIT), (0.04999, HIT), (0.05001, HOST)):
            with self.subTest(oldest_age=oldest_age):
                obj = generic()
                baseline(obj)
                attempt, trigger = arm(obj)
                rows = samples(10.060, x=-15)
                feed(obj, rows, age=oldest_age - (rows[-1][0] - rows[0][0]))
                self.assertEqual(obj.backend.max_age, 0.050)
                self.assertEqual(attempt.reason, "impact" if expected == HIT else "stale_data")
                self.assertEqual(trigger.messages, [[expected]])

    def check_fault(self, rows, reason, age=0.001, **fields):
        obj = generic()
        baseline(obj)
        attempt, trigger = arm(obj)
        feed(obj, rows, age=age, **fields)
        self.assertEqual(attempt.reason, reason)
        self.assertEqual(trigger.messages, [[HOST]])
        with self.assertRaises(CommandError):
            obj.finish(attempt.endstop, attempt, HOST)

    def test_stale_impact_never_sets_origin(self):
        self.check_fault(samples(10.060, x=-15), "stale_data", age=0.050)

    def test_oldest_threshold_sample_is_checked_not_only_latest(self):
        self.check_fault(samples(10.060, size=160, x=-15), "stale_data")

    def test_future_timestamp_never_sets_origin(self):
        self.check_fault(samples(10.060, x=-15), "stale_data", age=-0.050)

    def test_replayed_timestamps_fail_closed(self):
        self.check_fault(samples(10.055, x=-15), "invalid_data")

    def test_missing_samples_fail_closed(self):
        self.check_fault(samples(10.070, x=-15), "sample_gap")

    def test_nonfinite_and_wrong_shape_are_rejected(self):
        for row in ((10.060, float("nan"), 0, G), (10.060, 0, 0)):
            self.check_fault([row] + samples(10.0603125, size=15), "invalid_data")

    def test_sensor_errors_and_overflows_are_rejected(self):
        self.check_fault(samples(10.060, x=-15), "data_error", errors=1)
        self.check_fault(samples(10.060, x=-15), "data_error", overflows=1)

    def test_unconfigured_slow_rate_is_rejected_from_timestamps(self):
        obj = generic(rate=None)
        # Metadata absent, timestamp validation must still reject 800 Hz.
        obj.backend.rate = None
        obj.fake_printer.reactor.now = 10.01
        obj.backend.helper.emit({"data": samples(10.0, size=4, hz=800)})
        self.assertEqual(obj.sensor_fault, "sample_rate")

    def test_rate_and_range_are_checked_at_startup(self):
        with self.assertRaisesRegex(ConfigError, "below min_sample_rate"):
            generic(rate=800)
        with self.assertRaisesRegex(ConfigError, "at most 6.4 g"):
            generic("mpu9250 toolhead", {"threshold": 8})
        obj = generic("mpu9250 toolhead")
        self.assertEqual(obj.threshold_g, {"x": 4.0, "y": 4.0})

    def test_watchdog_checks_sensor_time_without_new_callback(self):
        obj = generic()
        baseline(obj)
        attempt, trigger = arm(obj)
        feed(obj, samples(10.060))
        attempt.last_data = 10.12  # Receipt is fresh but measurement is old.
        obj._watchdog(10.12)
        self.assertEqual(attempt.reason, "stale_data")
        self.assertEqual(trigger.messages, [[HOST]])

    def test_empty_batches_do_not_refresh_last_data(self):
        obj = generic()
        baseline(obj)
        attempt, trigger = arm(obj)
        last_data = attempt.last_data
        obj.backend.helper.emit({"data": []})
        self.assertEqual(attempt.last_data, last_data)
        obj._watchdog(10.1)
        self.assertEqual(trigger.messages, [[HOST]])


class SubscriptionAndPolicyTest(unittest.TestCase):
    def test_default_sample_age_and_explicit_shorter_limits(self):
        for options, expected_age, expected_timeout in (
                ({}, 0.050, 0.050), ({"max_sample_age": 0.020}, 0.020, 0.020),
                ({"stream_timeout": 0.010}, 0.050, 0.010)):
            with self.subTest(options=options):
                obj = generic(options=options)
                self.assertEqual(obj.backend.max_age, expected_age)
                self.assertEqual(obj.stream_timeout, expected_timeout)
                self.assertEqual(obj._reference_max_age(), expected_age)
        self.assertEqual(make_homing().stream_timeout, 0.050)

    def test_shared_clients_survive_and_interval_is_restored(self):
        obj = generic(connect=False)
        obj.fake_printer.send_event("klippy:connect")
        helper = obj.backend.helper
        seen = []
        other = lambda message: seen.append(message) or True
        helper.add_client(other)
        obj.homing_active = True
        obj._start_stream()
        self.assertEqual(helper.batch_interval, 0.005)
        self.assertEqual(obj.reactor.timers[helper.batch_timer], 0.005)
        obj._stop_stream()
        self.assertEqual(helper.batch_interval, 0.1)
        helper.emit({"data": samples(10)})
        self.assertEqual(helper.client_cbs, [other])
        self.assertEqual(len(seen), 1)

    def test_later_external_interval_change_is_preserved(self):
        obj = generic()
        obj.backend.helper.batch_interval = 0.003
        obj._stop_stream()
        self.assertEqual(obj.backend.helper.batch_interval, 0.003)

    def test_prior_session_callback_cannot_process_new_attempt(self):
        obj = generic()
        old = obj.backend.helper.client_cbs[0]
        obj._stop_stream()
        obj._start_stream()
        self.assertFalse(old({"data": samples(10, x=-15)}))
        self.assertIsNone(obj.last_batch)

    def test_legacy_web_request_contract_and_unsubscribe(self):
        obj = generic(legacy=True)
        baseline(obj)
        attempt, trigger = arm(obj)
        feed(obj, samples(10.060, x=-12))
        self.assertEqual(trigger.messages, [[HIT]])
        self.assertEqual(obj.backend.helper.update_interval, 0.005)
        obj._stop_stream()
        obj.backend.helper.emit({"data": []})
        self.assertFalse(obj.backend.helper.clients)
        self.assertEqual(obj.backend.helper.update_interval, 0.1)

    def test_excessive_rail_speed_rejected_before_any_motion(self):
        obj = generic(options={"max_report_travel": 1.0})
        rail = FakeRail(with_endstops(obj)["x"], info(speed=100, retract=0))
        with self.assertRaisesRegex(CommandError, "exceeds 20.0"):
            obj._handle_rails_begin(None, [rail])
        self.assertEqual(obj.toolhead.moves, [])
        self.assertEqual(obj.toolhead.accel_log, [])

    def test_speed_policy_tracks_report_travel_and_sample_age(self):
        obj = generic(options={"max_report_travel": 0.5})
        self.assertEqual(obj.backend.max_speed, 10)
        with self.assertRaises(ConfigError):
            obj.backend.validate_rail(info(speed=30, retract=0), ConfigError)

    def test_standard_command_and_pin_aliases_exist_once(self):
        obj = generic()
        self.assertIs(obj.fake_printer.chips["beacon_accel"], obj.fake_printer.chips["accel_homing"])
        self.assertIn("ACCEL_HOMING_STATUS", obj.fake_printer.commands)
        self.assertIn("BEACON_ACCEL_HOMING_STATUS", obj.fake_printer.commands)


class CaptureTest(unittest.TestCase):
    def test_beacon_capture_uses_mapping_and_retains_baseline(self):
        obj = make_homing(axes_map=((1, -1), (0, 1), (2, 1)))
        obj.start_capture()
        obj._inspect(batch(0, [(counts(0.5), counts(-0.25), counts(1))] * 8))
        capture = obj.stop_capture()
        self.assertEqual(capture["dropped"], 0)
        self.assertAlmostEqual(capture["samples"][0][1], 0.25)
        self.assertAlmostEqual(capture["samples"][0][2], 0.5)
        self.assertAlmostEqual(capture["baseline_g"][2], 1.0)

    def test_capture_is_bounded_and_reports_lost_samples(self):
        obj = make_homing()
        obj.capture_limit = 8
        obj.start_capture()
        for start in (0, 8, 16):
            obj._inspect(batch(start, [(0, 0, counts(1))] * 8))
        capture = obj.stop_capture()
        self.assertEqual(len(capture["samples"]), 8)
        self.assertEqual(capture["dropped"], 16)

    def test_generic_capture_records_g_and_timestamp_without_raw_clip_sentinel(self):
        obj = generic()
        obj.start_capture()
        baseline(obj)
        arm(obj)
        feed(obj, samples(10.060, x=-12))
        capture = obj.stop_capture()
        self.assertTrue(any(abs(row[1] + 12) < 0.0001 for row in capture["samples"]))
        self.assertEqual(capture["baseline_g"], (0, 0, 1))
        self.assertEqual(capture["dropped"], 0)

    def test_capture_failure_cannot_delay_or_prevent_trigger(self):
        obj = make_homing()
        from harness import arm_axis
        for start in range(0, 96, 8):
            obj._inspect(batch(start, [(0, 0, counts(1))] * 8))
        attempt, trigger = arm_axis(obj, "x", 200, 1000)
        obj.start_capture()
        def fail_after_trigger(params):
            self.assertEqual(trigger.messages, [[HIT]])
            raise ValueError("capture failure")
        obj._capture_batch = fail_after_trigger
        with self.assertLogs(level="ERROR"):
            obj._batch_hook(batch(200, [(counts(-12), 0, counts(1))] * 8))
        self.assertEqual(trigger.messages, [[HIT]])


class InvalidNumericOptionsTest(unittest.TestCase):
    def test_nonfinite_core_controls_are_rejected(self):
        for option in ("threshold", "x_threshold", "y_threshold", "homing_accel",
                       "back_off_dist", "baseline_time", "stream_timeout",
                       "travel_margin", "watchdog_margin"):
            with self.subTest(option=option):
                with self.assertRaises(ConfigError):
                    make_homing(options={option: float("nan")})

    def test_nonfinite_backend_controls_are_rejected(self):
        for option in ("stream_interval", "max_sample_age", "min_sample_rate",
                       "max_report_travel", "sensor_range"):
            with self.subTest(option=option):
                with self.assertRaises(ConfigError):
                    generic(options={option: float("nan")})
        with self.assertRaises(ConfigError):
            generic(rate=float("nan"))

    def test_runtime_set_rejects_nan_and_keeps_threshold(self):
        from test_trigger import FakeGcmd
        obj = generic()
        with self.assertRaisesRegex(FakeGcmd.Error, "finite"):
            obj.cmd_SET(FakeGcmd(AXIS="X", THRESHOLD=float("nan")))
        self.assertEqual(obj.threshold_g["x"], 8.0)

    def test_missing_trigger_dispatch_has_actionable_startup_error(self):
        with mock.patch.object(plugin.mcu, "TriggerDispatch", None):
            with self.assertRaisesRegex(ConfigError, "TriggerDispatch"):
                generic()

    def test_post_disarm_motion_assessment_updates_record(self):
        obj = generic()
        baseline(obj)
        attempt, _trigger = arm(obj)
        # Real pre-impact integration shows travel, but the watchdog has not
        # inspected the latest batch yet when dispatch finishes.
        feed(obj, samples(10.060, x=0.5))
        feed(obj, samples(10.065, x=-12))
        attempt.steppers = [types.SimpleNamespace(
            get_past_mcu_position=lambda t: t * 5000, get_step_dist=lambda: 0.01)]
        attempt.sums = [100000.0, 0.0, 0.0]
        obj.disarm()
        self.assertFalse(attempt.record["moving"])
        obj.trigger_time(attempt)
        self.assertTrue(attempt.record["moving"])
        self.assertTrue(attempt.record["motion_seen"])


if __name__ == "__main__":
    unittest.main()
