#!/usr/bin/env python3
"""
Test the camera-to-robot transformation to diagnose gripper movement issues.
"""
from config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
from kinematics.kinematics_solver import InverseKinematicsSolver
from camera_management.camera_transform_module import transform_camera_to_base
from config import HAND_EYE_MATRIX, CAMERA_TRANSFORM_MODE
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent))


print("🧪 Testing Camera-to-Robot Transform")
print("=" * 60)

print(f"\n⚙️ Camera Transform Mode: {CAMERA_TRANSFORM_MODE}")
print(f"\n📐 Hand-Eye Matrix being used:")
for i in range(4):
    print(f"  [{HAND_EYE_MATRIX[i,0]:8.4f} {HAND_EYE_MATRIX[i,1]:8.4f} {HAND_EYE_MATRIX[i,2]:8.4f} {HAND_EYE_MATRIX[i,3]:8.4f}]")

# Initialize kinematics
solver = InverseKinematicsSolver(URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

# Test with robot at zero position
test_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tcp_matrix, tcp_pose = solver.tcp_from_joints(test_joints)

print(f"\n🤖 Robot at zero position:")
print(
    f"TCP Position: [{tcp_pose[0]:.4f}, {tcp_pose[1]:.4f}, {tcp_pose[2]:.4f}] m")

# Test camera point: object 50cm in front of camera
camera_point = np.array([0.0, 0.0, 0.5])  # 50cm forward in camera frame
print(f"\n📷 Test Point in Camera Frame:")
print(
    f"Camera point: [{camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f}] m")
print(f"  (50cm in front of camera)")

# Transform to robot base frame
base_point = transform_camera_to_base(camera_point, tcp_matrix)
print(f"\n🗺️ Transformed to Robot Base Frame:")
print(
    f"Base point: [{base_point[0]:.3f}, {base_point[1]:.3f}, {base_point[2]:.3f}] m")

# Check if result makes sense
print(f"\n✓ Sanity Checks:")
# Point should be somewhere near the TCP
distance_from_tcp = np.linalg.norm(base_point - tcp_pose[:3])
print(f"  Distance from TCP: {distance_from_tcp:.3f} m")

if distance_from_tcp > 1.0:
    print(f"  ⚠️ WARNING: Point is very far from TCP!")
    print(f"     This might cause strange gripper movements!")
elif distance_from_tcp < 0.1:
    print(f"  ⚠️ WARNING: Point is very close to TCP!")
    print(f"     This might cause strange gripper movements!")
else:
    print(f"  ✅ Distance looks reasonable")

# Check Z height
if base_point[2] < 0:
    print(f"  ❌ ERROR: Point is below table (Z < 0)!")
elif base_point[2] > 2.0:
    print(f"  ⚠️ WARNING: Point is very high (Z > 2m)!")
else:
    print(f"  ✅ Z height looks reasonable")

# Test a few more points
print(f"\n🧪 Testing multiple points:")
test_points = [
    (np.array([0.0, 0.0, 0.3]), "30cm forward"),
    (np.array([0.1, 0.0, 0.3]), "30cm forward, 10cm right"),
    (np.array([0.0, 0.1, 0.3]), "30cm forward, 10cm down"),
]

for cam_pt, description in test_points:
    base_pt = transform_camera_to_base(cam_pt, tcp_matrix)
    dist_from_tcp = np.linalg.norm(base_pt - tcp_pose[:3])
    print(
        f"  {description:30s} -> [{base_pt[0]:6.3f}, {base_pt[1]:6.3f}, {base_pt[2]:6.3f}] (dist: {dist_from_tcp:.3f}m)")

# Check if using calibrated mode
print(f"\n⚙️ Configuration Check:")
if CAMERA_TRANSFORM_MODE == 'calibrated':
    print(f"  ✅ Using calibrated hand-eye matrix")
else:
    print(f"  ⚠️ Using simple mode (may not match calibration)")

# Check for common issues
print(f"\n� Common Issues Check:")

# Issue 1: Matrix might be inverted
R = HAND_EYE_MATRIX[:3, :3]
det = np.linalg.det(R)
if abs(det + 1) < 0.1:
    print(f"  ⚠️ Determinant is -1! Matrix might have wrong handedness")
elif abs(det - 1) < 0.1:
    print(f"  ✅ Determinant is +1 (correct)")

# Issue 2: Translation might be wrong scale
t = HAND_EYE_MATRIX[:3, 3]
t_mag = np.linalg.norm(t)
if t_mag > 0.5:
    print(f"  ⚠️ Translation is large ({t_mag:.3f}m)! Expected ~0.067m")
elif t_mag < 0.01:
    print(f"  ⚠️ Translation is very small ({t_mag:.3f}m)! Expected ~0.067m")
else:
    print(f"  ✅ Translation magnitude looks good ({t_mag:.3f}m)")

print("\n" + "=" * 60)

# Cleanup
solver.disconnect()

print("\n💡 If gripper is moving strangely:")
print("  1. Check that CAMERA_TRANSFORM_MODE = 'calibrated'")
print("  2. Verify the object detection depth values are reasonable (not too close/far)")
print("  3. Check if z_offset is being added in the grasping state")
print("  4. Verify joint limits aren't being violated")

