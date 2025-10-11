#!/usr/bin/env python3
"""
Test script to verify hand-eye matrix integration in the main application.
"""
from camera_management.camera_transform_module import transform_camera_to_base, transform_base_to_camera
from config import HAND_EYE_MATRIX, HAND_EYE_MATRIX_FILE
import numpy as np
import sys
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


def test_hand_eye_matrix_availability():
    """Test that hand-eye matrix is available in config."""
    print("🧪 Testing Hand-Eye Matrix Integration")
    print("=" * 50)

    # Test matrix from config
    print(f"✅ Hand-eye matrix from config:")
    print(f"   Shape: {HAND_EYE_MATRIX.shape}")
    print(
        f"   Translation magnitude: {np.linalg.norm(HAND_EYE_MATRIX[:3, 3]):.3f}m")
    print(f"   Determinant: {np.linalg.det(HAND_EYE_MATRIX[:3, :3]):.6f}")

    # Test matrix from file
    try:
        matrix_from_file = np.load(HAND_EYE_MATRIX_FILE)
        print(f"✅ Hand-eye matrix from file:")
        print(f"   Shape: {matrix_from_file.shape}")
        print(
            f"   Translation magnitude: {np.linalg.norm(matrix_from_file[:3, 3]):.3f}m")
        print(f"   Determinant: {np.linalg.det(matrix_from_file[:3, :3]):.6f}")

        # Verify they match
        if np.allclose(HAND_EYE_MATRIX, matrix_from_file):
            print("✅ Config and file matrices match!")
        else:
            print("❌ Config and file matrices differ!")

    except Exception as e:
        print(f"❌ Failed to load matrix from file: {e}")


def test_camera_transforms():
    """Test camera transformation functions."""
    print(f"\n🧪 Testing Camera Transform Functions")
    print("=" * 50)

    # Create a dummy TCP matrix (identity at origin for simplicity)
    tcp_matrix = np.eye(4)
    tcp_matrix[:3, 3] = [0.3, 0.4, 0.3]  # TCP at reasonable position

    # Test camera-to-base transformation
    camera_pos = [0.1, 0.05, 0.5]  # 50cm in front of camera
    print(f"📷 Testing camera-to-base transform:")
    print(f"   Camera position: {camera_pos}")

    try:
        base_pos = transform_camera_to_base(camera_pos, tcp_matrix)
        print(
            f"   Base position: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")
        print("   ✅ Camera-to-base transform successful")
    except Exception as e:
        print(f"   ❌ Camera-to-base transform failed: {e}")

    # Test base-to-camera transformation
    base_pos = [0.4, 0.45, 0.8]  # Reasonable base position
    print(f"\n🏠 Testing base-to-camera transform:")
    print(f"   Base position: {base_pos}")

    try:
        camera_pos = transform_base_to_camera(base_pos, tcp_matrix)
        print(
            f"   Camera position: [{camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}]")
        print("   ✅ Base-to-camera transform successful")
    except Exception as e:
        print(f"   ❌ Base-to-camera transform failed: {e}")

    # Test round-trip consistency
    print(f"\n🔄 Testing round-trip consistency:")
    original_camera_pos = [0.1, 0.05, 0.5]
    try:
        base_pos = transform_camera_to_base(original_camera_pos, tcp_matrix)
        back_to_camera_pos = transform_base_to_camera(base_pos, tcp_matrix)

        error = np.linalg.norm(
            np.array(original_camera_pos) - np.array(back_to_camera_pos))
        print(f"   Original camera pos: {original_camera_pos}")
        print(
            f"   Round-trip camera pos: [{back_to_camera_pos[0]:.3f}, {back_to_camera_pos[1]:.3f}, {back_to_camera_pos[2]:.3f}]")
        print(f"   Round-trip error: {error:.6f}m")

        if error < 1e-6:
            print("   ✅ Round-trip consistency excellent!")
        elif error < 1e-3:
            print("   ✅ Round-trip consistency good!")
        else:
            print("   ⚠️ Round-trip consistency poor - check matrix")

    except Exception as e:
        print(f"   ❌ Round-trip test failed: {e}")


def test_main_system_integration():
    """Test that main system components can import successfully."""
    print(f"\n🧪 Testing Main System Integration")
    print("=" * 50)

    try:
        from integrated_robot_control_system import IntegratedRobotControlSystem
        print("✅ IntegratedRobotControlSystem imports successfully")
    except Exception as e:
        print(f"❌ IntegratedRobotControlSystem import failed: {e}")

    try:
        from states.move_to_state import MoveToState
        print("✅ MoveToState imports successfully")
    except Exception as e:
        print(f"❌ MoveToState import failed: {e}")

    try:
        from camera_management.camera_manager import CameraManager
        print("✅ CameraManager imports successfully")
    except Exception as e:
        print(f"❌ CameraManager import failed: {e}")


if __name__ == "__main__":
    test_hand_eye_matrix_availability()
    test_camera_transforms()
    test_main_system_integration()

    print(f"\n🎉 Hand-Eye Matrix Integration Test Complete!")
    print("=" * 50)
