"""Generic 70 ms freshness bound; synthetic transport/reference data only."""
import math
import unittest

from harness import CommandError, ConfigError, make_homing, plugin
import test_backends as backends


class SampleAge70Tests(unittest.TestCase):
    def receive(self, oldest_age, options=None, legacy=False, newest_age=None):
        obj = backends.generic(options=options, legacy=legacy)
        # A dense, uniform packet retains the reported 44.794 ms span. Two
        # endpoints alone would correctly fail the independent rate/gap rules.
        now, size = .070, 145
        first = now - oldest_age
        last = first + .044794 if newest_age is None else now - newest_age
        rows = [(first + number * (last - first) / (size - 1), 0., 0., backends.G)
                for number in range(size)]
        reference = plugin.QuietReference("x", -1., .1, 1000., require_quiet=False)
        obj.preparation = reference
        obj.reactor.now = now
        obj.backend.helper.emit({"data": rows})
        return obj, reference

    def test_reported_oldest_51_876_and_newest_7_082_ms_pass_reference_admission(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                obj, reference = self.receive(.051876, legacy=legacy)
                self.assertIsNone(obj.sensor_fault)
                self.assertEqual(reference.count, 145)
                self.assertAlmostEqual(reference.oldest_batch_age * 1000., 51.876, places=6)
                self.assertAlmostEqual(reference.newest_batch_age * 1000., 7.082, places=6)
                self.assertEqual(obj._reference_max_age(), .070)

    def test_reported_packet_still_fails_explicit_50_ms_without_weakening_that_value(self):
        obj, reference = self.receive(.051876, {"max_sample_age": .050})
        self.assertEqual(obj.sensor_fault, "stale_data")
        self.assertEqual(reference.count, 0)
        self.assertEqual(obj.backend.max_age, .050)
        self.assertAlmostEqual(reference.oldest_batch_age * 1000., 51.876, places=6)
        self.assertAlmostEqual(reference.newest_batch_age * 1000., 7.082, places=6)

    def test_exact_70_ms_is_accepted_and_older_is_rejected(self):
        for legacy in (False, True):
            for age, accepted in ((.070, True), (.070001, False)):
                with self.subTest(legacy=legacy, age=age):
                    obj, reference = self.receive(age, legacy=legacy)
                    self.assertEqual(obj.sensor_fault, None if accepted else "stale_data")
                    self.assertEqual(reference.count, 145 if accepted else 0)

    def test_explicit_20_and_50_ms_remain_active_in_reference_and_runtime(self):
        for bound in (.020, .050):
            for age, accepted in ((bound, True), (bound + .000001, False)):
                # Use a short live packet entirely inside the selected bound.
                obj = backends.generic(options={"max_sample_age": bound})
                obj.preparation = plugin.QuietReference("x", -1., .1, 1000., require_quiet=False)
                obj.reactor.now = bound
                first = bound - age
                rows = backends.samples(first)
                obj.backend.helper.emit({"data": rows})
                self.assertEqual(obj.backend.max_age, bound)
                self.assertEqual(obj._reference_max_age(), bound)
                self.assertEqual(obj.sensor_fault, None if accepted else "stale_data")

    def test_configuration_accepts_70_and_rejects_above_70(self):
        self.assertEqual(backends.generic(options={"max_sample_age": .070}).backend.max_age, .070)
        for age in (.070001, .071, float("inf")):
            with self.subTest(age=age), self.assertRaises(ConfigError):
                backends.generic(options={"max_sample_age": age})

    def test_watchdog_and_native_beacon_age_remain_50_ms(self):
        obj = backends.generic()
        self.assertEqual(obj.backend.max_age, .070)
        self.assertEqual(obj.stream_timeout, .050)
        native = make_homing()
        self.assertEqual(native.stream_timeout, .050)
        self.assertEqual(native._reference_max_age(), .050)

    def test_future_samples_still_fail_the_independent_5_ms_guard(self):
        obj, reference = self.receive(.038794, newest_age=-.006)
        self.assertEqual(obj.sensor_fault, "stale_data")
        self.assertEqual(reference.count, 0)

    def test_explicit_report_and_startup_travel_guards_use_the_configured_age(self):
        reporting = backends.generic(options={"max_report_travel": 1.})
        self.assertEqual(reporting.backend.max_speed, 1. / .070)
        reporting.backend.validate_motion_plan(reporting.backend.max_speed, 5000., CommandError)
        with self.assertRaisesRegex(CommandError, "report-travel policy"):
            reporting.backend.validate_motion_plan(reporting.backend.max_speed + .001, 5000., CommandError)
        wider = backends.generic(options={"max_startup_travel": 3.})
        strict = backends.generic(options={"max_startup_travel": 3., "max_sample_age": .050})
        self.assertAlmostEqual(wider.backend.motion_startup_request_travel(30., 5000.), 3.03)
        self.assertAlmostEqual(strict.backend.motion_startup_request_travel(30., 5000.), 2.43)
        strict.backend.validate_motion_plan(30., 5000., CommandError)
        with self.assertRaisesRegex(CommandError, "startup motion proof"):
            wider.backend.validate_motion_plan(30., 5000., CommandError)

    def test_low_speed_without_motion_evidence_stops_after_each_configured_startup_deadline(self):
        fixture = backends.BaselineFloorTest()
        for age in (.050, .070):
            for direction in (-1, 1):
                with self.subTest(age=age, direction=direction):
                    obj, endstop = fixture.machine(axes=(0, 1, 2), options={"max_sample_age": age})
                    attempt = fixture.prepare_arm(obj, endstop, speed=1., accel=1000., direction=direction)
                    timeout = obj.backend.motion_startup_timeout(1., 1000.)
                    self.assertAlmostEqual(timeout, age + .066)
                    # The older regression's 400 rows stop before the new
                    # deadline. Cover that deadline and one delivery margin.
                    count = math.ceil((timeout + .020) * 3200.) + 16
                    fixture.ramp(obj, attempt, direction=direction, max_samples=count)
                    self.assertFalse(attempt.motion_seen)
                    self.assertEqual(attempt.reason, "no_motion")
                    self.assertEqual(endstop.trigger_log.messages,
                                     [[plugin.mcu.MCU_trsync.REASON_HOST_REQUEST]])
                    elapsed = obj.reactor.now - attempt.arm_time
                    self.assertGreaterEqual(elapsed, timeout)
                    self.assertLessEqual(elapsed, timeout + .010)


if __name__ == "__main__":
    unittest.main()
