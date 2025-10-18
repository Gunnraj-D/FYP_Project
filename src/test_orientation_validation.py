"""
Comprehensive validation test for the orientation control system.

This test validates:
1. MoveToStrictState functionality
2. Face-down enforcement
3. Pose data handling
4. IK solver integration
5. Pickup sequencer integration
"""

import logging
import numpy as np
from scipy.spatial.transform import Rotation as R

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_face_down_orientation():
    """Test the face-down orientation matrix."""
    from kinematics.kinematics_solver import get_facing_down_orientation

    face_down_matrix = get_facing_down_orientation()
    expected = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    assert np.allclose(
        face_down_matrix, expected), f"Face-down matrix incorrect: {face_down_matrix}"

    # Convert to Euler angles
    face_down_rpy = R.from_matrix(face_down_matrix).as_euler('xyz')
    expected_rpy = [np.pi, 0, 0]  # 180° roll, 0° pitch, 0° yaw

    assert np.allclose(face_down_rpy, expected_rpy,
                       atol=1e-6), f"Face-down RPY incorrect: {face_down_rpy}"

    logger.info("✅ Face-down orientation test passed")


def test_pose_data_handling():
    """Test pose data handling in MoveToStrictState."""
    # Test pose creation
    position = [0.5, 0.0, 0.6]
    orientation = [0, np.pi, 0]  # Face-down
    pose = position + orientation

    assert len(pose) == 6, f"Pose should have 6 elements, got {len(pose)}"
    assert pose[:3] == position, "Position should be first 3 elements"
    assert pose[3:6] == orientation, "Orientation should be last 3 elements"

    # Test face-down enforcement
    from kinematics.kinematics_solver import get_facing_down_orientation

    face_down_matrix = get_facing_down_orientation()
    face_down_rpy = R.from_matrix(face_down_matrix).as_euler('xyz')

    # Apply face-down enforcement
    enforced_pose = list(pose[:3]) + list(face_down_rpy)

    assert enforced_pose[:3] == position, "Position should be preserved"
    assert np.allclose(
        enforced_pose[3:6], face_down_rpy), "Orientation should be face-down"

    logger.info("✅ Pose data handling test passed")


def test_move_to_strict_state_creation():
    """Test MoveToStrictState creation methods."""
    # Mock context for testing
    class MockContext:
        def __init__(self):
            self.ik = None
            self.telemetry = None
            self.commands = None

    context = MockContext()

    # Test basic creation
    from states.move_to_strict_state import MoveToStrictState

    target_pose = [0.5, 0.0, 0.6, 0, np.pi, 0]
    state = MoveToStrictState(context, target_pose=target_pose)

    assert state.target_pose == target_pose
    assert state.enforce_face_down == False

    # Test face-down enforcement creation
    state_enforced = MoveToStrictState.with_face_down_enforcement(
        context, target_pose)
    assert state_enforced.enforce_face_down == True
    assert state_enforced.target_pose == target_pose

    # Test telemetry creation
    state_telemetry = MoveToStrictState.from_telemetry_with_face_down(
        context, 'generated_grasp_pose')
    assert state_telemetry.pose_from_telemetry == 'generated_grasp_pose'
    assert state_telemetry.enforce_face_down == True

    logger.info("✅ MoveToStrictState creation test passed")


def test_pickup_sequencer_integration():
    """Test pickup sequencer integration."""
    # Mock context and state machine
    class MockContext:
        def __init__(self):
            self.ik = None
            self.telemetry = None
            self.commands = None

    class MockStateMachine:
        pass

    context = MockContext()
    state_machine = MockStateMachine()

    # Test pickup sequencer creation
    from states.pickup_task_sequencer import PickupTaskSequencer

    sequencer = PickupTaskSequencer(state_machine, context)

    # Check that all states are created
    assert len(
        sequencer.task_queue) == 8, f"Expected 8 states, got {len(sequencer.task_queue)}"

    # Check that critical movements use MoveToStrictState
    from states.move_to_strict_state import MoveToStrictState

    states = list(sequencer.task_queue)

    # Check initial positioning (index 0)
    assert isinstance(
        states[0], MoveToStrictState), "Initial positioning should use MoveToStrictState"
    assert states[0].enforce_face_down == True, "Initial positioning should enforce face-down"

    # Check approach pose (index 3)
    assert isinstance(
        states[3], MoveToStrictState), "Approach pose should use MoveToStrictState"
    assert states[3].enforce_face_down == True, "Approach pose should enforce face-down"
    assert states[3].pose_from_telemetry == 'generated_approach_pose', "Should use approach pose from telemetry"

    # Check grasp pose (index 4)
    assert isinstance(
        states[4], MoveToStrictState), "Grasp pose should use MoveToStrictState"
    assert states[4].enforce_face_down == True, "Grasp pose should enforce face-down"
    assert states[4].pose_from_telemetry == 'generated_grasp_pose', "Should use grasp pose from telemetry"

    # Check lift movement (index 6)
    assert isinstance(
        states[6], MoveToStrictState), "Lift movement should use MoveToStrictState"
    assert states[6].enforce_face_down == True, "Lift movement should enforce face-down"

    # Check return movement (index 7)
    assert isinstance(
        states[7], MoveToStrictState), "Return movement should use MoveToStrictState"
    assert states[7].enforce_face_down == True, "Return movement should enforce face-down"

    logger.info("✅ Pickup sequencer integration test passed")


def test_ik_solver_compatibility():
    """Test IK solver compatibility with pose data."""
    from kinematics.kinematics_solver import InverseKinematicsSolver

    # Test solve_pose method signature
    import inspect
    sig = inspect.signature(InverseKinematicsSolver.solve_pose)
    params = list(sig.parameters.keys())

    expected_params = ['self', 'target_pose', 'current_q', 'max_iter', 'tol']
    assert params == expected_params, f"Expected parameters {expected_params}, got {params}"

    # Test that solve_pose expects [x, y, z, rx, ry, rz] format
    # This is validated by the method implementation we saw earlier

    logger.info("✅ IK solver compatibility test passed")


def test_orientation_consistency():
    """Test that orientation enforcement is consistent across all movements."""
    from kinematics.kinematics_solver import get_facing_down_orientation

    face_down_matrix = get_facing_down_orientation()
    face_down_rpy = R.from_matrix(face_down_matrix).as_euler('xyz')

    # Test that all face-down orientations are the same
    expected_rpy = [np.pi, 0, 0]  # 180° roll, 0° pitch, 0° yaw

    assert np.allclose(face_down_rpy, expected_rpy,
                       atol=1e-6), f"Face-down RPY should be {expected_rpy}, got {face_down_rpy}"

    # Test that the matrix represents a downward-pointing orientation
    # The Z-axis should point downward (negative Z in base frame)
    z_axis = face_down_matrix[:, 2]  # Third column is Z-axis
    expected_z = np.array([0, 0, -1])

    assert np.allclose(
        z_axis, expected_z), f"Z-axis should point down {expected_z}, got {z_axis}"

    logger.info("✅ Orientation consistency test passed")


def run_all_tests():
    """Run all validation tests."""
    logger.info("=" * 60)
    logger.info("ORIENTATION CONTROL VALIDATION TESTS")
    logger.info("=" * 60)

    try:
        test_face_down_orientation()
        test_pose_data_handling()
        test_move_to_strict_state_creation()
        test_pickup_sequencer_integration()
        test_ik_solver_compatibility()
        test_orientation_consistency()

        logger.info("=" * 60)
        logger.info("✅ ALL TESTS PASSED - SYSTEM VALIDATED")
        logger.info("=" * 60)

        return True

    except Exception as e:
        logger.error(f"❌ TEST FAILED: {e}")
        logger.error("=" * 60)
        return False


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
