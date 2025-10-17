"""
Example demonstrating the new orientation control features in MoveToState.

This example shows how to:
1. Create MoveToState with custom orientation
2. Use face-down enforcement in pickup sequencer
3. Override orientations when needed
"""

import logging
import numpy as np
from scipy.spatial.transform import Rotation as R

from states.move_to_state import MoveToState
from kinematics.kinematics_solver import get_facing_down_orientation

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_custom_orientation_examples():
    """Demonstrate different ways to create MoveToState with orientation control."""

    # Example 1: Custom orientation matrix
    # Create a rotation matrix for gripper pointing forward (X-axis)
    forward_orientation = np.array([
        [0, 0, 1],   # X-axis points forward
        [0, 1, 0],   # Y-axis points right
        [-1, 0, 0]   # Z-axis points up
    ])

    # Example 2: Orientation from Euler angles
    # Create orientation from roll, pitch, yaw (in radians)
    euler_angles = [0, np.pi/4, 0]  # 45 degree pitch
    euler_orientation = R.from_euler('xyz', euler_angles).as_matrix()

    # Example 3: Face-down orientation (default)
    face_down_orientation = get_facing_down_orientation()

    logger.info("Custom orientation examples created:")
    logger.info(f"Forward orientation:\n{forward_orientation}")
    logger.info(f"Euler orientation (45° pitch):\n{euler_orientation}")
    logger.info(f"Face-down orientation:\n{face_down_orientation}")


def demonstrate_move_to_state_usage():
    """Show how to use the enhanced MoveToState."""

    # Mock context for demonstration
    class MockContext:
        def __init__(self):
            self.ik = None  # Would be actual IK solver
            self.telemetry = None  # Would be actual telemetry

    context = MockContext()

    # Example 1: Basic usage with custom orientation
    target_position = [0.5, 0.0, 0.6]
    custom_orientation = get_facing_down_orientation()

    move_state = MoveToState(
        context=context,
        target_location=target_position,
        target_orientation=custom_orientation
    )

    logger.info("Created MoveToState with custom orientation")

    # Example 2: Using factory method for custom orientation
    move_state_factory = MoveToState.with_custom_orientation(
        context=context,
        target_location=target_position,
        target_orientation=custom_orientation
    )

    logger.info("Created MoveToState using factory method")

    # Example 3: Face-down enforcement (overrides any custom orientation)
    move_state_enforced = MoveToState(
        context=context,
        target_location=target_position,
        target_orientation=custom_orientation,  # This will be overridden
        enforce_face_down=True
    )

    logger.info("Created MoveToState with face-down enforcement")

    # Example 4: Using factory method for face-down enforcement
    move_state_face_down = MoveToState.with_face_down_enforcement(
        context=context,
        target_location=target_position
    )

    logger.info("Created MoveToState with face-down enforcement using factory")


def demonstrate_pickup_sequencer_usage():
    """Show how the pickup sequencer now enforces face-down orientation."""

    logger.info("PickupTaskSequencer now enforces face-down orientation for:")
    logger.info("  - Approach pose movement")
    logger.info("  - Grasp pose movement")
    logger.info("  - Lift movement")
    logger.info("  - Return to pickup location")
    logger.info("")
    logger.info(
        "This ensures consistent gripper orientation throughout the pickup sequence,")
    logger.info(
        "regardless of the orientation calculated by the grasp detection system.")


def main():
    """Main demonstration function."""
    logger.info("=" * 60)
    logger.info("ORIENTATION CONTROL DEMONSTRATION")
    logger.info("=" * 60)

    create_custom_orientation_examples()
    logger.info("")

    demonstrate_move_to_state_usage()
    logger.info("")

    demonstrate_pickup_sequencer_usage()
    logger.info("")

    logger.info("=" * 60)
    logger.info("Key Features:")
    logger.info("1. MoveToState now accepts custom orientation matrices")
    logger.info("2. Face-down enforcement can override any custom orientation")
    logger.info(
        "3. PickupTaskSequencer enforces face-down orientation for all movements")
    logger.info("4. Factory methods provide convenient ways to create states")
    logger.info(
        "5. Backward compatibility maintained - existing code continues to work")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
