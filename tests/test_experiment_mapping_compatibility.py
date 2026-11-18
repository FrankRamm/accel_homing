"""A directional threshold must never silently become an XYZ experiment limit."""
import copy
import types
import unittest
from unittest import mock

from harness import CommandError
from test_experiment_workflow import ExperimentRig
from test_mapped_detector import mapping
from test_workflows import GCmd


class ExperimentMappingCompatibilityTests(unittest.TestCase):
    def test_loaded_pending_and_invalidated_directional_profiles_block_all_modes_before_effects(self):
        for axis in "xy":
            for mode in ("LIVE", "SHADOW", "COMPARE"):
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
