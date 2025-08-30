#!/usr/bin/env python3
"""
Simple test script to verify the main fixes are working.
"""

import sys
import logging

# Configure basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_imports():
    """Test that all modules can be imported without errors."""
    try:
        logger.info("Testing imports...")

        # Test specific imports instead of wildcard
        from config import LOOP_RATE_MS, FRI_IP, FRI_PORT
        logger.info("✓ Config imported successfully")

        from shared_state_joints import SharedState
        logger.info("✓ SharedState imported successfully")

        from kinematics_solver import InverseKinematicsSolver
        logger.info("✓ KinematicsSolver imported successfully")

        from FRI_Interface import KukaFRIInterface
        logger.info("✓ FRI Interface imported successfully")

        from hand_detection_module import HandTracker
        logger.info("✓ HandTracker imported successfully")

        from camera_transform_module import transform_camera_to_base
        logger.info("✓ Camera transform imported successfully")

        return True

    except Exception as e:
        logger.error(f"✗ Import failed: {e}")
        return False


def test_shared_state():
    """Test SharedState functionality."""
    try:
        logger.info("Testing SharedState...")

        from shared_state_joints import SharedState

        state = SharedState()

        # Test camera vector update
        result = state.update_camera_vector([100, 200, 300], 0.8)
        assert result == True, "Camera vector update failed"

        # Test get camera vector
        vector = state.get_camera_vector()
        assert vector == [
            100, 200, 300], f"Expected [100, 200, 300], got {vector}"

        # Test target joints update
        result = state.update_target_joints(
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        assert result == True, "Target joints update failed"

        logger.info("✓ SharedState tests passed")
        return True

    except Exception as e:
        logger.error(f"✗ SharedState test failed: {e}")
        return False


def test_kinematics_solver():
    """Test KinematicsSolver functionality."""
    try:
        logger.info("Testing KinematicsSolver...")

        from kinematics_solver import InverseKinematicsSolver
        from config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS

        # Test initialization
        solver = InverseKinematicsSolver(
            URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)
        logger.info("✓ KinematicsSolver initialized")

        # Test joint validation
        test_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        result = solver.validate_joint_angles(test_joints)
        assert result == True, "Joint validation failed"

        logger.info("✓ KinematicsSolver tests passed")
        return True

    except Exception as e:
        logger.error(f"✗ KinematicsSolver test failed: {e}")
        return False


def test_camera_transform():
    """Test camera transform functionality."""
    try:
        logger.info("Testing camera transform...")

        from camera_transform_module import transform_camera_to_base
        import numpy as np

        # Test with dummy data
        camera_vector = [100, 200, 300]  # mm
        tcp_pose = np.eye(4)  # Identity matrix

        result = transform_camera_to_base(camera_vector, tcp_pose)
        assert result is not None, "Transform returned None"
        assert len(result) == 3, f"Expected 3 elements, got {len(result)}"

        logger.info("✓ Camera transform tests passed")
        return True

    except Exception as e:
        logger.error(f"✗ Camera transform test failed: {e}")
        return False


def main():
    """Run all tests."""
    logger.info("Starting tests...")

    tests = [
        test_imports,
        test_shared_state,
        test_kinematics_solver,
        test_camera_transform
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if test():
            passed += 1
        logger.info("")

    logger.info(f"Tests completed: {passed}/{total} passed")

    if passed == total:
        logger.info("🎉 All tests passed! The main fixes are working.")
        return 0
    else:
        logger.error("❌ Some tests failed. Please check the errors above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
