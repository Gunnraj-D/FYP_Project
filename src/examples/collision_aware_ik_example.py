"""
Example demonstrating collision-aware inverse kinematics.
Shows how to use the CollisionAwareKinematicsSolver to avoid table collisions.
"""
from config import URDF_FILEPATH, get_facing_down_orientation
from kinematics.collision_aware_kinematics_solver import CollisionAwareKinematicsSolver
import sys
import os
import logging
import numpy as np
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent.parent))


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_table_in_pybullet():
    """
    Create a simple table in PyBullet for collision testing.
    Returns the table body ID.
    """
    import pybullet as p

    # Create a simple box table
    table_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.5, 0.5, 0.05]  # 1m x 1m x 10cm table
    )

    table_body = p.createMultiBody(
        baseMass=0,  # Static
        baseCollisionShapeIndex=table_shape,
        basePosition=[0.4, 0.0, 0.05]  # Position table at 0.4m height
    )

    return table_body


def example_collision_aware_ik():
    """Demonstrate collision-aware IK with table collision avoidance."""

    logger.info("Starting collision-aware IK example")

    # Create solver with GUI for visualization
    solver = CollisionAwareKinematicsSolver(
        urdf_filepath=URDF_FILEPATH,
        base_elements=None,
        active_links_mask=None,
        use_gui=True  # Enable GUI for visualization
    )

    # Create table for collision testing
    table_id = create_table_in_pybullet()
    solver.table_id = table_id

    try:
        # Define target positions (some might be near the table)
        test_positions = [
            [0.3, 0.0, 0.3],   # Low position near table
            [0.5, 0.2, 0.4],   # Medium height
            [0.4, 0.0, 0.6],   # High position
            # Very low position (should trigger collision avoidance)
            [0.6, -0.1, 0.2],
        ]

        current_angles = [0.0, -0.5, 0.0, 1.0,
                          0.0, 0.5, 0.0]  # Safe starting pose
        target_orientation = get_facing_down_orientation()

        for i, target_pos in enumerate(test_positions):
            logger.info(f"\n--- Test {i+1}: Target position {target_pos} ---")

            try:
                # Solve with collision awareness
                final_angles, trajectory = solver.solve_XYZ_collision_aware(
                    target_position=target_pos,
                    current_joint_angles=current_angles,
                    target_orientation=target_orientation,
                    use_pre_approach=True
                )

                logger.info(f"✓ Found collision-free solution")
                logger.info(f"  Final angles: {final_angles}")
                logger.info(f"  Trajectory waypoints: {len(trajectory)}")

                # Check clearance of final position
                clearance = solver._compute_clearance_score(
                    final_angles.tolist())
                logger.info(f"  Final clearance: {clearance:.4f}m")

                # Update current angles for next iteration
                current_angles = final_angles.tolist()

            except Exception as e:
                logger.error(f"✗ Failed to find solution: {e}")
                continue

        logger.info("\n--- Testing trajectory execution ---")

        # Test trajectory execution
        def mock_execution_callback(joint_angles):
            """Mock execution callback that just logs the waypoint."""
            logger.debug(f"Executing waypoint: {joint_angles}")
            return True

        # Generate a test trajectory
        test_target = [0.4, 0.0, 0.5]
        final_angles, trajectory = solver.solve_XYZ_collision_aware(
            target_position=test_target,
            current_joint_angles=current_angles,
            target_orientation=target_orientation,
            use_pre_approach=True
        )

        # Execute trajectory
        success = solver.execute_trajectory_safely(
            trajectory, mock_execution_callback)
        logger.info(
            f"Trajectory execution: {'✓ Success' if success else '✗ Failed'}")

    except Exception as e:
        logger.error(f"Example failed: {e}")
        raise

    finally:
        solver.disconnect()
        logger.info("Example completed")


def example_comparison_with_standard_ik():
    """Compare collision-aware IK with standard IK."""

    logger.info("Comparing collision-aware vs standard IK")

    # Create both solvers
    collision_solver = CollisionAwareKinematicsSolver(
        urdf_filepath=URDF_FILEPATH,
        base_elements=None,
        active_links_mask=None,
        use_gui=False
    )

    standard_solver = CollisionAwareKinematicsSolver(
        urdf_filepath=URDF_FILEPATH,
        base_elements=None,
        active_links_mask=None,
        use_gui=False
    )

    # Create table
    table_id = create_table_in_pybullet()
    collision_solver.table_id = table_id
    standard_solver.table_id = table_id

    try:
        target_pos = [0.4, 0.0, 0.3]  # Low position near table
        current_angles = [0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0]
        target_orientation = get_facing_down_orientation()

        # Standard IK
        logger.info("--- Standard IK ---")
        try:
            standard_angles = standard_solver.solve_XYZ(
                target_pos, current_angles, target_orientation
            )
            standard_clearance = standard_solver._compute_clearance_score(
                standard_angles.tolist())
            logger.info(f"Standard IK clearance: {standard_clearance:.4f}m")
        except Exception as e:
            logger.error(f"Standard IK failed: {e}")
            standard_clearance = 0.0

        # Collision-aware IK
        logger.info("--- Collision-aware IK ---")
        try:
            collision_angles, trajectory = collision_solver.solve_XYZ_collision_aware(
                target_pos, current_angles, target_orientation
            )
            collision_clearance = collision_solver._compute_clearance_score(
                collision_angles.tolist())
            logger.info(
                f"Collision-aware IK clearance: {collision_clearance:.4f}m")
            logger.info(f"Trajectory length: {len(trajectory)} waypoints")
        except Exception as e:
            logger.error(f"Collision-aware IK failed: {e}")
            collision_clearance = 0.0

        # Compare results
        logger.info(f"\n--- Comparison ---")
        logger.info(f"Standard IK clearance: {standard_clearance:.4f}m")
        logger.info(
            f"Collision-aware IK clearance: {collision_clearance:.4f}m")
        logger.info(
            f"Improvement: {collision_clearance - standard_clearance:.4f}m")

    finally:
        collision_solver.disconnect()
        standard_solver.disconnect()


if __name__ == "__main__":
    print("Collision-Aware IK Example")
    print("=" * 50)

    try:
        # Run basic example
        example_collision_aware_ik()

        print("\n" + "=" * 50)
        print("Comparison Example")
        print("=" * 50)

        # Run comparison example
        example_comparison_with_standard_ik()

    except KeyboardInterrupt:
        logger.info("Example interrupted by user")
    except Exception as e:
        logger.error(f"Example failed: {e}")
        raise
