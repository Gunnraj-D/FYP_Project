"""
Test script to compare calibrated vs simple camera transform modes.
Shows the difference in transformation results between the two modes.
"""
import numpy as np
from config.config import (
    set_camera_transform_mode,
    get_camera_transform_info,
    print_camera_transform_info,
    HAND_EYE_MATRIX_CALIBRATED,
    HAND_EYE_MATRIX_SIMPLE
)
from camera_management.camera_transform_module import transform_camera_to_base


def create_sample_tcp_matrix():
    """Create a sample TCP transformation matrix for testing."""
    # Sample TCP position: x=0.4m, y=0.0m, z=0.5m (typical robot pose)
    # With identity rotation (end effector pointing down)
    tcp_matrix = np.array([
        [1.0, 0.0, 0.0, 0.4],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float32)
    return tcp_matrix


def test_transform_modes():
    """Compare transformations using both camera modes."""
    print("\n" + "="*70)
    print("📷 CAMERA TRANSFORM MODE COMPARISON TEST")
    print("="*70)

    # Sample camera position (object detected at these coordinates in camera frame)
    camera_positions = [
        [0.0, 0.0, 0.5],    # Object directly in front at 0.5m
        [0.1, 0.0, 0.4],    # Object slightly to the right
        [-0.1, 0.1, 0.6],   # Object to the left and up
    ]

    tcp_matrix = create_sample_tcp_matrix()

    print("\n📍 TCP Position in base frame:")
    print(
        f"   Position: [{tcp_matrix[0,3]:.3f}, {tcp_matrix[1,3]:.3f}, {tcp_matrix[2,3]:.3f}]")

    for i, cam_pos in enumerate(camera_positions):
        print(f"\n{'─'*70}")
        print(f"Test Case {i+1}: Camera position = {cam_pos}")
        print(f"{'─'*70}")

        # Test with CALIBRATED mode
        print("\n1️⃣  CALIBRATED MODE (with rotation):")
        set_camera_transform_mode('calibrated')
        base_pos_calibrated = transform_camera_to_base(cam_pos, tcp_matrix)
        print(
            f"   → Base position: [{base_pos_calibrated[0]:.4f}, {base_pos_calibrated[1]:.4f}, {base_pos_calibrated[2]:.4f}]")

        # Test with SIMPLE mode
        print("\n2️⃣  SIMPLE MODE (no rotation, identity):")
        set_camera_transform_mode('simple')
        base_pos_simple = transform_camera_to_base(cam_pos, tcp_matrix)
        print(
            f"   → Base position: [{base_pos_simple[0]:.4f}, {base_pos_simple[1]:.4f}, {base_pos_simple[2]:.4f}]")

        # Show difference
        diff = base_pos_calibrated - base_pos_simple
        print(f"\n📊 Difference (calibrated - simple):")
        print(f"   Δ = [{diff[0]:.4f}, {diff[1]:.4f}, {diff[2]:.4f}]")
        print(f"   Magnitude: {np.linalg.norm(diff):.4f}m")

    print("\n" + "="*70)
    print("✅ Comparison complete!")
    print("="*70)

    # Show matrix details
    print("\n📋 MATRIX DETAILS:")
    print("\n1️⃣  CALIBRATED HAND-EYE MATRIX:")
    print(HAND_EYE_MATRIX_CALIBRATED)
    print(f"   Translation: {HAND_EYE_MATRIX_CALIBRATED[:3, 3]}")
    print(f"   Has rotation: Yes")

    print("\n2️⃣  SIMPLE HAND-EYE MATRIX:")
    print(HAND_EYE_MATRIX_SIMPLE)
    print(f"   Translation: {HAND_EYE_MATRIX_SIMPLE[:3, 3]}")
    print(f"   Has rotation: No (Identity)")

    print("\n")


if __name__ == "__main__":
    test_transform_modes()

    # Reset to calibrated mode
    set_camera_transform_mode('calibrated')

