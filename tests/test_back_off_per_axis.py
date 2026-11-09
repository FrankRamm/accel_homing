"""Per-axis back-off distances and Kalico-like out-of-range semantics."""
import unittest

from harness import CommandError, ConfigError, make_homing
from test_center_calibration import CenterRig
import test_retract_noise
from test_session import FakeRail, info, start_session, with_endstops
from test_workflows import Rig


def make_with_options(options):
    return make_homing(options=options)


class PerAxisBackOffTests(unittest.TestCase):
    machine = test_retract_noise.FinalRetractTests.machine

    def test_per_axis_options_fall_back_to_shared(self):
        obj = make_with_options({"back_off_dist": "7"})
        self.assertEqual((obj.back_off_dist["x"], obj.back_off_dist["y"]), (7.0, 7.0))
        obj = make_with_options({"back_off_dist": "0", "y_back_off_dist": "50"})
        self.assertEqual((obj.back_off_dist["x"], obj.back_off_dist["y"]), (0.0, 50.0))
        obj = make_with_options({})
        self.assertEqual((obj.back_off_dist["x"], obj.back_off_dist["y"]), (5.0, 5.0))

    def test_only_real_axes_have_back_off(self):
        obj = make_with_options({"back_off_dist": "7", "z_back_off_dist": "-1"})
        self.assertEqual(obj.back_off_dist, {"x": 7.0, "y": 7.0})
        self.assertEqual(obj.get_status(0.0)["back_off_dist"], {"x": 7.0, "y": 7.0})

    def test_large_values_accept_and_negative_rejects(self):
        self.assertEqual(make_with_options({"y_back_off_dist": "250"}).back_off_dist["y"], 250.0)
        self.assertEqual(make_with_options({"back_off_dist": "100"}).back_off_dist["x"], 100.0)
        with self.assertRaises(ConfigError):
            make_with_options({"x_back_off_dist": "-1"})

    def test_final_release_uses_axis_distance(self):
        obj, kin, _state = self.machine("y", positive=True, bounds=(0.0, 329.0))
        obj.back_off_dist["y"] = 50.0
        kin.home(object())
        self.assertEqual(obj.toolhead.moves, [([None, 279.0, None], 100.0)])

    def test_out_of_range_target_raises_like_kalico(self):
        obj, kin, _state = self.machine("y", bounds=(0.0, 30.0))
        obj.back_off_dist["y"] = 50.0
        with self.assertRaisesRegex(CommandError, "out of range"):
            kin.home(object())
        self.assertEqual(obj.toolhead.moves, [])

    def test_cross_axis_release_uses_homed_axis_distance(self):
        obj = make_homing(options={"y_back_off_dist": "50"})
        endstops = with_endstops(obj, axes=("x", "y"))
        toolhead = obj.toolhead
        toolhead.position = [150.0, 329.3, 10.0, 0.0]
        toolhead.get_position = lambda: list(toolhead.position)
        toolhead.kin = type("K", (), {"get_status": lambda self, t: {"homed_axes": "xyz"}})()
        y_info = info(speed=100.0, retract=0.0)._replace(positive_dir=True, position_endstop=329.0)
        obj.rails_by_axis["y"] = FakeRail(endstops["y"], y_info, name="stepper_y",
                                          rng=(0.0, 329.0))
        start_session(obj, FakeRail(endstops["x"], info(), name="stepper_x"))
        self.assertEqual(toolhead.moves[0], ([None, 279.0, None], 100.0))

    def test_cross_axis_out_of_range_raises(self):
        obj = make_homing(options={"y_back_off_dist": "50"})
        endstops = with_endstops(obj, axes=("x", "y"))
        toolhead = obj.toolhead
        toolhead.position = [150.0, 0.0, 10.0, 0.0]
        toolhead.get_position = lambda: list(toolhead.position)
        toolhead.kin = type("K", (), {"get_status": lambda self, t: {"homed_axes": "xyz"}})()
        obj.rails_by_axis["y"] = FakeRail(endstops["y"], info(), name="stepper_y",
                                          rng=(0.0, 30.0))
        with self.assertRaisesRegex(CommandError, "out of range"):
            start_session(obj, FakeRail(endstops["x"], info(), name="stepper_x"))
        self.assertEqual(toolhead.moves, [])

    def test_workflow_suppresses_and_restores_per_axis(self):
        for make_rig, run in ((Rig, lambda rig: rig.calibrate(save=0)),
                              (CenterRig, lambda rig: rig.center())):
            with self.subTest(rig=make_rig.__name__):
                rig = make_rig()
                rig.owner.back_off_dist = {"x": 5.0, "y": 50.0}
                run(rig)
                self.assertEqual(rig.owner.back_off_dist, {"x": 5.0, "y": 50.0})


if __name__ == "__main__":
    unittest.main()
