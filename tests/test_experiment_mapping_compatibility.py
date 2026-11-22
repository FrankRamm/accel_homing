"""A directional threshold must never silently become an XYZ experiment limit."""
import copy
import types
import unittest
from unittest import mock

from harness import CommandError
from test_experiment_workflow import ExperimentRig
from test_mapped_detector import mapping
from test_workflows import GCmd
from accel_homing_filter import profile as filter_profile


class ExperimentMappingCompatibilityTests(unittest.TestCase):
    def test_xyz_experiment_never_reinterprets_a_filtered_threshold(self):
        for mode in ("LIVE", "SHADOW", "COMPARE"):
            with self.subTest(mode=mode):
                rig = ExperimentRig()
                rig.owner.filter_settings = lambda axis: filter_profile(100., 3.)
                with mock.patch.object(rig.workflow, "_preflight") as preflight, \
                        mock.patch.object(rig.workflow, "_run_diagnostic") as run:
                    with self.assertRaisesRegex(CommandError, "active notch"):
                        rig.workflow.cmd_EXPERIMENT(GCmd(AXIS="X", CENTERED=1, MODE=mode))
                preflight.assert_not_called()
                run.assert_not_called()
                self.assertEqual(rig.home_calls, [])

    def test_loaded_pending_and_invalidated_directional_profiles_block_xyz_modes_before_effects(self):
        for axis in "xy":
            for mode in ("LIVE", "SHADOW"):
                for state in ("loaded", "pending", "invalidated"):
                    for manual in (False, True):
                        with self.subTest(axis=axis, mode=mode, state=state, manual=manual):
                            rig = ExperimentRig()
                            workflow = rig.workflow
                            profile = {"impact_metric": "mapped_axis", "axis_mapping": mapping(axis)}
                            if state == "invalidated":
                                profile["axis_mapping_invalidated"] = {"reason": "remounted"}
                            if state == "pending":
                                workflow.pending_model_metadata[axis] = profile
                            else:
                                workflow.model = types.SimpleNamespace(
                                    thresholds={axis: 1.5}, metadata={axis: profile})
                            if manual:
                                workflow.manual_axes.add(axis)
                            before = copy.deepcopy(profile)
                            with mock.patch.object(workflow, "_preflight") as preflight, \
                                    mock.patch.object(workflow, "_run_diagnostic") as run:
                                with self.assertRaisesRegex(CommandError, "cannot use a mapped-axis"):
                                    workflow.cmd_EXPERIMENT(GCmd(AXIS=axis.upper(), CENTERED=1, MODE=mode))
                            preflight.assert_not_called()
                            run.assert_not_called()
                            self.assertEqual(profile, before)
                            self.assertEqual(rig.home_calls, [])
                            self.assertEqual(rig.toolhead.moves, [])
                            self.assertIsNone(workflow.session)


if __name__ == "__main__":
    unittest.main()
