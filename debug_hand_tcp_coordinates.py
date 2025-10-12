"""
Debug script to understand the hand-to-TCP coordinate transformation.

This script will help diagnose why the TCP-relative coordinates don't match
the expected values when the hand is positioned relative to the TCP.
"""
from config import HAND_EYE_MATRIX, CAMERA_TRANSFORM_MODE
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))


print("=" * 70)
print("HAND-EYE CALIBRATION DIAGNOSTIC")
print("=" * 70)

print(f"\nCamera Transform Mode: {CAMERA_TRANSFORM_MODE}")
print(f"\nHAND_EYE_MATRIX (tcp_T_camera):")
print(HAND_EYE_MATRIX)

# Extract rotation and translation
rotation = HAND_EYE_MATRIX[:3, :3]
translation = HAND_EYE_MATRIX[:3, 3]

print(f"\nTranslation (camera origin in TCP frame): {translation}")
print(f"  X: {translation[0]*1000:.1f}mm")
print(f"  Y: {translation[1]*1000:.1f}mm")
print(f"  Z: {translation[2]*1000:.1f}mm")
print(f"  Magnitude: {np.linalg.norm(translation)*1000:.1f}mm")

print(f"\nRotation matrix:")
print(rotation)

# Check if it's a proper rotation matrix
det = np.linalg.det(rotation)
print(f"\nDeterminant (should be ~1.0): {det:.6f}")
print(f"Is orthogonal: {np.allclose(rotation @ rotation.T, np.eye(3))}")

# Extract axes
tcp_x_in_cam = rotation[:, 0]  # TCP X-axis expressed in camera frame
tcp_y_in_cam = rotation[:, 1]  # TCP Y-axis expressed in camera frame
tcp_z_in_cam = rotation[:, 2]  # TCP Z-axis expressed in camera frame

print(f"\nTCP axes in camera frame:")
print(f"  TCP X-axis: {tcp_x_in_cam}")
print(f"  TCP Y-axis: {tcp_y_in_cam}")
print(f"  TCP Z-axis: {tcp_z_in_cam}")

# Or equivalently, camera axes in TCP frame (rows of rotation matrix)
cam_x_in_tcp = rotation[0, :]  # Camera X-axis expressed in TCP frame
cam_y_in_tcp = rotation[1, :]  # Camera Y-axis expressed in TCP frame
cam_z_in_tcp = rotation[2, :]  # Camera Z-axis expressed in TCP frame

print(f"\nCamera axes in TCP frame:")
print(f"  Cam X-axis: {cam_x_in_tcp}")
print(f"  Cam Y-axis: {cam_y_in_tcp}")
print(f"  Cam Z-axis: {cam_z_in_tcp}")

print("\n" + "=" * 70)
print("TEST TRANSFORMATION")
print("=" * 70)

# Test: Hand at camera origin (0, 0, 0) in camera frame
print("\n[Test 1] Hand at camera optical center (0, 0, 0) in camera frame:")
cam_pos = np.array([0.0, 0.0, 0.0, 1.0])
tcp_pos = HAND_EYE_MATRIX @ cam_pos
print(f"  Camera frame: {cam_pos[:3]}")
print(f"  TCP frame: {tcp_pos[:3]}")
print(f"  (This should be the camera origin position in TCP frame)")

# Test: Hand 50cm in front of camera (along camera Z-axis)
print("\n[Test 2] Hand 50cm in front of camera (0, 0, 0.5) in camera frame:")
cam_pos = np.array([0.0, 0.0, 0.5, 1.0])
tcp_pos = HAND_EYE_MATRIX @ cam_pos
print(f"  Camera frame: {cam_pos[:3]}")
print(f"  TCP frame: {tcp_pos[:3]}")
print(f"  Distance from TCP origin: {np.linalg.norm(tcp_pos[:3])*1000:.1f}mm")

# Test: Hand 5cm below camera (0, 0.05, depth) - typical scenario
print("\n[Test 3] Hand 5cm down, 30cm forward (0, 0.05, 0.3) in camera frame:")
cam_pos = np.array([0.0, 0.05, 0.3, 1.0])
tcp_pos = HAND_EYE_MATRIX @ cam_pos
print(f"  Camera frame: {cam_pos[:3]}")
print(f"  TCP frame: {tcp_pos[:3]}")
tcp_pos_with_hack = tcp_pos[:3] - np.array([0, 0, 0.138])
print(f"  With -138mm hack: {tcp_pos_with_hack}")
print(
    f"  Distance from TCP origin (with hack): {np.linalg.norm(tcp_pos_with_hack)*1000:.1f}mm")

# Test: What camera position gives TCP = (0, 0, 0.05)?
print("\n[Test 4] Finding camera position for hand at TCP (0, 0, 0.05):")
print("  If hand is 5cm below TCP in real world, what should camera see?")
# Inverse transform: camera_pos = inv(HAND_EYE_MATRIX) @ tcp_pos
desired_tcp_pos = np.array([0.0, 0.0, 0.05, 1.0])
cam_pos_homogeneous = np.linalg.inv(HAND_EYE_MATRIX) @ desired_tcp_pos
print(f"  Desired TCP frame: {desired_tcp_pos[:3]}")
print(f"  Required Camera frame: {cam_pos_homogeneous[:3]}")
print(f"  (This is what camera should measure to get hand at 5cm below TCP)")

# Test: User's reported values
print("\n[Test 5] User's reported scenario:")
print("  Hand is physically ~5cm below TCP (aligned in X, Y)")
print("  User sees TCP coordinates: (0.0, 0.05, 0.25)")
print("  Expected TCP coordinates: (0.0, 0.0, 0.05)")
print(
    f"\n  → The Z value is {0.25 - 0.05:.2f}m ({(0.25-0.05)*1000:.0f}mm) too large!")
print(f"  → The Y value is {0.05:.2f}m ({0.05*1000:.0f}mm) off-center")
print("\n  Possible causes:")
print("  1. Hand-eye calibration issue (wrong origin or rotation)")
print("  2. Coordinate frame convention mismatch")
print("  3. The -138mm hack isn't being applied correctly")
print("  4. The hand detection is measuring wrong depth values")

print("\n" + "=" * 70)
print("COORDINATE FRAME CONVENTIONS")
print("=" * 70)
print("""
Camera Frame (RealSense convention):
  X+ = Right (in image)
  Y+ = Down (in image)
  Z+ = Forward (depth into scene)

Robot TCP Frame (typical for gripper facing down):
  X+ = Forward (along gripper approach)
  Y+ = Left (across gripper)
  Z+ = Down (gripper opening direction)
  
For a hand 5cm directly below the TCP (aligned in X and Y),
we expect TCP coordinates approximately:
  X ≈ 0 (aligned left-right)
  Y ≈ 0 (aligned front-back)
  Z ≈ 0.05 (5cm down from TCP)
""")

print("\nIf you're seeing different values, the hand-eye calibration")
print("may need adjustment or there's a frame convention mismatch.")
