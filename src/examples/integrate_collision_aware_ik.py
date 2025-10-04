"""
Integration example showing how to replace standard kinematics solver
with collision-aware version in existing code.
"""
from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver
import sys
import os
import logging
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent.parent))


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_enhanced_kinematics_solver(table_id=None, use_gui=False):
    """
    Create an enhanced kinematics solver with collision awareness.

    Args:
        table_id: PyBullet body ID of the table (None if no table)
        use_gui: Whether to show PyBullet GUI

    Returns:
        CollisionAwareKinematicsSolver instance
    """
    solver = CollisionAwareKinematicsSolver(
        urdf_filepath=URDF_FILEPATH,
        base_elements=BASE_ELEMENT,
        active_links_mask=ACTIVE_LINKS,
        use_gui=use_gui,
        table_id=table_id
    )

    logger.info("Enhanced kinematics solver created with collision awareness")
    return solver


def example_drop_in_replacement():
    """
    Example showing how to use collision-aware solver as drop-in replacement.
    """
    logger.info("Demonstrating drop-in replacement for standard IK")

    # Create solver (replace your existing solver creation)
    solver = create_enhanced_kinematics_solver(use_gui=True)

    try:
        # Example 1: Simple position targeting (same API as before)
        target_position = [0.4, 0.0, 0.5]
        current_angles = [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0]

        logger.info("--- Standard IK call (now collision-aware) ---")
        final_angles = solver.solve_XYZ(
            target_position=target_position,
            current_joint_angles=current_angles
        )
        logger.info(f"Standard solve_XYZ result: {final_angles}")

        # Example 2: Enhanced collision-aware call with trajectory
        logger.info("--- Enhanced collision-aware IK call ---")
        final_angles, trajectory = solver.solve_XYZ_collision_aware(
            target_position=target_position,
            current_joint_angles=current_angles,
            use_pre_approach=True
        )
        logger.info(f"Collision-aware result: {final_angles}")
        logger.info(f"Trajectory waypoints: {len(trajectory)}")

        # Example 3: Pose-based targeting
        target_pose = [0.4, 0.0, 0.5, 0, 0, -1.57]  # [x, y, z, rx, ry, rz]

        logger.info("--- Pose-based collision-aware IK ---")
        final_angles, trajectory = solver.solve_pose_collision_aware(
            target_pose=target_pose,
            current_joint_angles=current_angles,
            use_pre_approach=True
        )
        logger.info(f"Pose-based result: {final_angles}")
        logger.info(f"Trajectory waypoints: {len(trajectory)}")

    except Exception as e:
        logger.error(f"Example failed: {e}")
        raise

    finally:
        solver.disconnect()


def example_with_table_collision():
    """
    Example with actual table collision detection.
    """
    logger.info("Demonstrating collision avoidance with table")

    # Create a simple table
    import pybullet as p
    table_shape = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.05])
    table_body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=table_shape,
        basePosition=[0.4, 0.0, 0.05]
    )

    # Create solver with table
    solver = create_enhanced_kinematics_solver(
        table_id=table_body, use_gui=True)

    try:
        # Test positions that would normally hit the table
        test_positions = [
            [0.3, 0.0, 0.2],   # Very low - should trigger collision avoidance
            [0.4, 0.0, 0.3],   # Low - should use pre-approach
            [0.5, 0.0, 0.4],   # Medium - should work fine
        ]

        current_angles = [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0]

        for i, target_pos in enumerate(test_positions):
            logger.info(f"\n--- Test {i+1}: {target_pos} ---")

            try:
                # Use collision-aware IK
                final_angles, trajectory = solver.solve_XYZ_collision_aware(
                    target_position=target_pos,
                    current_joint_angles=current_angles,
                    use_pre_approach=True
                )

                # Check clearance
                clearance = solver._compute_clearance_score(
                    final_angles.tolist())
                logger.info(
                    f"✓ Success - Clearance: {clearance:.4f}m, Waypoints: {len(trajectory)}")

                # Update for next iteration
                current_angles = final_angles.tolist()

            except Exception as e:
                logger.error(f"✗ Failed: {e}")

    finally:
        solver.disconnect()


def example_trajectory_execution():
    """
    Example showing how to execute trajectories safely.
    """
    logger.info("Demonstrating safe trajectory execution")

    solver = create_enhanced_kinematics_solver(use_gui=True)

    try:
        # Generate a collision-aware trajectory
        target_position = [0.4, 0.0, 0.5]
        current_angles = [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0]

        final_angles, trajectory = solver.solve_XYZ_collision_aware(
            target_position=target_position,
            current_joint_angles=current_angles,
            use_pre_approach=True
        )

        logger.info(f"Generated trajectory with {len(trajectory)} waypoints")

        # Mock execution function (replace with your actual robot control)
        def execute_waypoint(joint_angles):
            """Mock function to execute a waypoint on the robot."""
            logger.info(
                f"Executing waypoint: {[f'{a:.3f}' for a in joint_angles]}")

            # In real implementation, this would:
            # 1. Send joint angles to robot controller
            # 2. Wait for robot to reach position
            # 3. Check for errors
            # 4. Return success/failure

            return True  # Mock success

        # Execute trajectory safely
        success = solver.execute_trajectory_safely(
            trajectory, execute_waypoint)

        if success:
            logger.info("✓ Trajectory executed successfully")
        else:
            logger.error("✗ Trajectory execution failed")

    finally:
        solver.disconnect()


if __name__ == "__main__":
    print("Collision-Aware IK Integration Examples")
    print("=" * 50)

    try:
        # Example 1: Drop-in replacement
        print("\n1. Drop-in Replacement Example")
        print("-" * 30)
        example_drop_in_replacement()

        # Example 2: With table collision
        print("\n2. Table Collision Avoidance Example")
        print("-" * 30)
        example_with_table_collision()

        # Example 3: Trajectory execution
        print("\n3. Safe Trajectory Execution Example")
        print("-" * 30)
        example_trajectory_execution()

        print("\n" + "=" * 50)
        print("All examples completed successfully!")

    except KeyboardInterrupt:
        logger.info("Examples interrupted by user")
    except Exception as e:
        logger.error(f"Examples failed: {e}")
        raise
