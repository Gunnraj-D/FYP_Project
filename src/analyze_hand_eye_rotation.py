#!/usr/bin/env python3
"""
Analyze the rotation component of the hand-eye calibration matrix.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# New hand-eye matrix
HAND_EYE_MATRIX = np.array([
    [-0.9994,  0.0335, -0.0117,  0.0064],
    [-0.0333, -0.9993, -0.0174,  0.0697],
    [-0.0123, -0.0170,  0.9998,  0.0060],
    [0.0000,  0.0000,  0.0000,  1.0000]
])

print("🔍 Hand-Eye Matrix Rotation Analysis")
print("=" * 60)

# Extract rotation matrix
R = HAND_EYE_MATRIX[:3, :3]
print("\n📐 Rotation Matrix:")
print(R)

# Convert to Euler angles (XYZ convention, in degrees)
rotation = Rotation.from_matrix(R)
euler_xyz_deg = rotation.as_euler('xyz', degrees=True)
print(f"\n🔄 Euler Angles (XYZ convention):")
print(f"  Roll  (X): {euler_xyz_deg[0]:7.3f}°")
print(f"  Pitch (Y): {euler_xyz_deg[1]:7.3f}°")
print(f"  Yaw   (Z): {euler_xyz_deg[2]:7.3f}°")

# Convert to Euler angles (ZYX convention for comparison)
euler_zyx_deg = rotation.as_euler('zyx', degrees=True)
print(f"\n🔄 Euler Angles (ZYX convention):")
print(f"  Yaw   (Z): {euler_zyx_deg[0]:7.3f}°")
print(f"  Pitch (Y): {euler_zyx_deg[1]:7.3f}°")
print(f"  Roll  (X): {euler_zyx_deg[2]:7.3f}°")

# Convert to axis-angle representation
axis_angle = rotation.as_rotvec()
angle = np.linalg.norm(axis_angle)
axis = axis_angle / angle if angle > 1e-6 else np.array([0, 0, 1])
print(f"\n🎯 Axis-Angle Representation:")
print(f"  Rotation angle: {np.degrees(angle):.3f}°")
print(f"  Rotation axis: [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]")

# Check if close to identity (no rotation)
identity_error = np.linalg.norm(R - np.eye(3))
print(f"\n📊 Deviation from Identity:")
print(f"  Frobenius norm: {identity_error:.6f}")

# Check if close to 180° rotation around X (camera pointing down)
R_180_x = np.array([
    [1,  0,  0],
    [0, -1,  0],
    [0,  0, -1]
])
rotation_180_x_error = np.linalg.norm(R - R_180_x)
print(f"\n📊 Deviation from 180° X-rotation (camera pointing down):")
print(f"  Frobenius norm: {rotation_180_x_error:.6f}")

# Translation
translation = HAND_EYE_MATRIX[:3, 3]
translation_magnitude = np.linalg.norm(translation)
print(f"\n📏 Translation:")
print(
    f"  Vector: [{translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}]m")
print(
    f"  Magnitude: {translation_magnitude:.4f}m ({translation_magnitude*100:.2f}cm)")

print("\n" + "=" * 60)

# Interpretation
print("\n💡 Interpretation:")
if np.abs(euler_xyz_deg[0]) < 5 and np.abs(euler_xyz_deg[1]) < 5 and np.abs(euler_xyz_deg[2]) < 5:
    print("  ✅ Rotation is VERY SMALL (nearly identity)")
    print("     Camera is approximately aligned with TCP frame")
elif 175 < np.abs(euler_xyz_deg[0]) < 185 and np.abs(euler_xyz_deg[1]) < 5 and np.abs(euler_xyz_deg[2]) < 5:
    print("  ✅ Rotation is approximately 180° around X-axis")
    print("     Camera is pointing down (typical eye-in-hand configuration)")
else:
    print(f"  ⚠️  Rotation has significant angles:")
    print(
        f"     Roll: {euler_xyz_deg[0]:.1f}°, Pitch: {euler_xyz_deg[1]:.1f}°, Yaw: {euler_xyz_deg[2]:.1f}°")
    print("     Camera is NOT simply pointing down - it has a complex orientation")
