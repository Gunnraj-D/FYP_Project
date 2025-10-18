"""
Test to verify that poses from telemetry preserve their calculated orientations
while face-down enforcement only applies when explicitly requested.
"""

import logging
import numpy as np
from scipy.spatial.transform import Rotation as R

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_pose_orientation_preservation():
    """Test that poses from telemetry preserve their calculated orientations."""

    # Mock context with telemetry that returns poses with calculated orientations
    class MockTelemetry:
        def get_generated_grasp_pose(self):
            # Return a pose with calculated orientation (not face-down)
            calculated_orientation = [0.1, 0.2, 0.3]  # Some calculated RPY
            return [0.5, 0.0, 0.6] + calculated_orientation

        def get_generated_approach_pose(self):
            # Return a pose with calculated orientation (not face-down)
            calculated_orientation = [0.1, 0.2, 0.3]  # Some calculated RPY
            return [0.5, 0.0, 0.7] + calculated_orientation

    class MockContext:
        def __init__(self):
            self.telemetry = MockTelemetry()
            self.ik = None
            self.commands = None

    context = MockContext()

    # Test MoveToStrictState without face-down enforcement
    from states.move_to_strict_state import MoveToStrictState

    # Test grasp pose (should preserve calculated orientation)
    grasp_state = MoveToStrictState(
        context,
        pose_from_telemetry='generated_grasp_pose',
        enforce_face_down=False
    )

    grasp_pose = grasp_state._get_target_pose()
    expected_grasp_pose = [0.5, 0.0, 0.6, 0.1, 0.2, 0.3]

    assert np.allclose(
        grasp_pose, expected_grasp_pose), f"Grasp pose should preserve calculated orientation: {grasp_pose}"
    logger.info("✅ Grasp pose preserves calculated orientation")

    # Test approach pose (should preserve calculated orientation)
    approach_state = MoveToStrictState(
        context,
        pose_from_telemetry='generated_approach_pose',
        enforce_face_down=False
    )

    approach_pose = approach_state._get_target_pose()
    expected_approach_pose = [0.5, 0.0, 0.7, 0.1, 0.2, 0.3]

    assert np.allclose(
        approach_pose, expected_approach_pose), f"Approach pose should preserve calculated orientation: {approach_pose}"
    logger.info("✅ Approach pose preserves calculated orientation")


def test_face_down_enforcement():
    """Test that face-down enforcement works when explicitly requested."""

    class MockTelemetry:
        def get_generated_grasp_pose(self):
            # Return a pose with calculated orientation
            calculated_orientation = [0.1, 0.2, 0.3]  # Some calculated RPY
            return [0.5, 0.0, 0.6] + calculated_orientation

    class MockContext:
        def __init__(self):
            self.telemetry = MockTelemetry()
            self.ik = None
            self.commands = None

    context = MockContext()

    from states.move_to_strict_state import MoveToStrictState
    from kinematics.kinematics_solver import get_facing_down_orientation

    # Test with face-down enforcement (should override calculated orientation)
    enforced_state = MoveToStrictState(
        context,
        pose_from_telemetry='generated_grasp_pose',
        enforce_face_down=True
    )

    # Apply enforcement manually to test the method
    pose_before = enforced_state._get_target_pose()
    enforced_pose = enforced_state._apply_face_down_enforcement(pose_before)

    # Position should be preserved
    assert np.allclose(enforced_pose[:3], [
                       0.5, 0.0, 0.6]), f"Position should be preserved: {enforced_pose[:3]}"

    # Orientation should be face-down
    face_down_matrix = get_facing_down_orientation()
    face_down_rpy = R.from_matrix(face_down_matrix).as_euler('xyz')
    expected_orientation = face_down_rpy

    assert np.allclose(
        enforced_pose[3:6], expected_orientation), f"Orientation should be face-down: {enforced_pose[3:6]}"
    logger.info("✅ Face-down enforcement works correctly")


def test_pickup_sequencer_behavior():
    """Test that pickup sequencer uses correct orientation behavior."""

    class MockTelemetry:
        def get_generated_grasp_pose(self):
            return [0.5, 0.0, 0.6, 0.1, 0.2, 0.3]  # Calculated orientation

        def get_generated_approach_pose(self):
            return [0.5, 0.0, 0.7, 0.1, 0.2, 0.3]  # Calculated orientation

    class MockContext:
        def __init__(self):
            self.telemetry = MockTelemetry()
            self.ik = None
            self.commands = None

    class MockStateMachine:
        pass

    context = MockContext()
    state_machine = MockStateMachine()

    from states.pickup_task_sequencer import PickupTaskSequencer

    sequencer = PickupTaskSequencer(state_machine, context)
    states = list(sequencer.task_queue)

    # Check initial positioning (should enforce face-down)
    initial_state = states[0]
    assert initial_state.enforce_face_down == True, "Initial positioning should enforce face-down"
    logger.info("✅ Initial positioning enforces face-down")

    # Check approach pose (should NOT enforce face-down, use calculated orientation)
    approach_state = states[3]
    assert approach_state.enforce_face_down == False, "Approach pose should NOT enforce face-down"
    assert approach_state.pose_from_telemetry == 'generated_approach_pose', "Should use approach pose from telemetry"
    logger.info("✅ Approach pose uses calculated orientation")

    # Check grasp pose (should NOT enforce face-down, use calculated orientation)
    grasp_state = states[4]
    assert grasp_state.enforce_face_down == False, "Grasp pose should NOT enforce face-down"
    assert grasp_state.pose_from_telemetry == 'generated_grasp_pose', "Should use grasp pose from telemetry"
    logger.info("✅ Grasp pose uses calculated orientation")

    # Check lift movement (should NOT enforce face-down, use calculated orientation)
    lift_state = states[6]
    assert lift_state.enforce_face_down == False, "Lift movement should NOT enforce face-down"
    assert lift_state.pose_from_telemetry == 'generated_approach_pose', "Should use approach pose from telemetry"
    logger.info("✅ Lift movement uses calculated orientation")

    # Check return movement (should enforce face-down)
    return_state = states[7]
    assert return_state.enforce_face_down == True, "Return movement should enforce face-down"
    logger.info("✅ Return movement enforces face-down")


def run_pose_orientation_tests():
    """Run all pose orientation tests."""
    logger.info("=" * 60)
    logger.info("POSE ORIENTATION BEHAVIOR TESTS")
    logger.info("=" * 60)

    try:
        test_pose_orientation_preservation()
        test_face_down_enforcement()
        test_pickup_sequencer_behavior()

        logger.info("=" * 60)
        logger.info("✅ ALL POSE ORIENTATION TESTS PASSED")
        logger.info("=" * 60)

        return True

    except Exception as e:
        logger.error(f"❌ TEST FAILED: {e}")
        logger.error("=" * 60)
        return False


if __name__ == "__main__":
    success = run_pose_orientation_tests()
    exit(0 if success else 1)
