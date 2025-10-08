#!/usr/bin/env python3
"""
Test camera direction to understand the optical axis issue.
"""
from calibration.calibration_config import CalibrationConfig
from calibration.pose_generator import PoseGenerator
import numpy as np
import sys
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


def test_camera_direction():
    """Test camera direction to understand the optical axis."""
    print("🧪 Testing Camera Direction")
    print("=" * 30)

    # Initialize pose generator
    config = CalibrationConfig()
    pose_generator = PoseGenerator(config)

    # Load hand-eye matrix
    if not pose_generator.load_hand_eye_matrix():
        print("❌ Cannot load hand-eye matrix")
        return

    target_position = (0.4, 0.025, 0.22)
    print(f"Target position: {target_position}")

    # Test a simple case: camera directly above target
    print("\n1️⃣ Testing Camera Above Target")
    print("-" * 30)

    # Direction from target to camera (straight up)
    dir_vec = np.array([0, 0, 1])  # Points up from target to camera
    target = np.array(target_position)

    def normalize(v):
        return v / (np.linalg.norm(v) + 1e-12)

    # Test both camera optical axis conventions
    for convention, name in [(1, "Original"), (-1, "Flipped")]:
        print(f"\n{name} convention (z_cam = {convention} * dir_vec):")

        # Build camera rotation
        z_cam = normalize(convention * dir_vec)
        up_hint = np.array([0, 0, 1.0])
        if abs(np.dot(z_cam, up_hint)) > 0.99:
            up_hint = np.array([0.0, 1.0, 0.0])
        x_cam = normalize(np.cross(up_hint, z_cam))
        y_cam = np.cross(z_cam, x_cam)
        R_base_cam = np.column_stack([x_cam, y_cam, z_cam])

        print(
            f"Camera optical axis (z_cam): [{z_cam[0]:.3f}, {z_cam[1]:.3f}, {z_cam[2]:.3f}]")

        # Test with a reasonable camera distance
        r = 0.3
        p_cam = target + r * dir_vec
        print(
            f"Camera position: [{p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f}]")

        # Calculate TCP position
        T_cam_to_tcp = pose_generator.hand_eye_matrix
        t_cam_to_tcp = T_cam_to_tcp[:3, 3]
        R_cam_to_tcp = T_cam_to_tcp[:3, :3]

        offset = R_base_cam.dot(t_cam_to_tcp)
        p_tcp = p_cam + offset

        print(
            f"TCP position: [{p_tcp[0]:.3f}, {p_tcp[1]:.3f}, {p_tcp[2]:.3f}]")

        # Check validity
        y_valid = p_tcp[1] >= -0.5
        z_valid = 0.27 <= p_tcp[2] <= 0.5
        valid = y_valid and z_valid
        print(f"Valid: {valid} (Y: {y_valid}, Z: {z_valid})")

        # Show final TCP orientation
        R_base_tcp = R_base_cam.dot(R_cam_to_tcp)
        print(
            f"TCP Z-axis (orientation): [{R_base_tcp[0,2]:.3f}, {R_base_tcp[1,2]:.3f}, {R_base_tcp[2,2]:.3f}]")


if __name__ == "__main__":
    test_camera_direction()
