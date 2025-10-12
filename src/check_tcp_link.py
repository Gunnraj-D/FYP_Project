#!/usr/bin/env python3
"""
Check which link is being used as TCP in the kinematics solver.
"""
import pybullet as p
import numpy as np
from kinematics.kinematics_solver import InverseKinematicsSolver
from config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent))


print("🔍 Checking TCP Link Configuration")
print("=" * 60)

# Initialize kinematics solver
solver = InverseKinematicsSolver(URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

print(f"\n📐 URDF: {URDF_FILEPATH}")
print(f"End Effector Link Index: {solver.end_effector_link_index}")

# Get link name from PyBullet
link_info = p.getJointInfo(
    solver.robot_id, solver.end_effector_link_index, physicsClientId=solver.client)
link_name = link_info[12].decode('utf-8')

print(f"End Effector Link Name: '{link_name}'")

# Check the URDF structure
print("\n📋 URDF Link Chain:")
print("  link_7 -> tool0 (flange) [+12.6cm]")
print("  tool0 -> robotiq_85_base_link [+0cm]")
print("  robotiq_85_base_link -> tcp [+13.8cm]")
print("  TOTAL from flange to TCP: 13.8cm")

# Test forward kinematics at a known position
test_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tcp_matrix, tcp_pose = solver.tcp_from_joints(test_joints)

print(f"\n🧪 Test Forward Kinematics (all joints at 0°):")
print(
    f"TCP Position: [{tcp_pose[0]:.4f}, {tcp_pose[1]:.4f}, {tcp_pose[2]:.4f}] m")
print(f"TCP Z-height from base: {tcp_pose[2]:.4f} m")

# Expected height: 0.36 + 0.42 + 0.4 + 0.126 + 0.138 = 1.444m
expected_height = 0.36 + 0.42 + 0.4 + 0.126 + \
    0.138  # Sum of all joint offsets + tool0 + tcp
print(f"Expected Z-height (including TCP): {expected_height:.4f} m")
print(f"Difference: {abs(tcp_pose[2] - expected_height):.4f} m")

if link_name == 'tcp':
    print("\n✅ CORRECT: Using 'tcp' link (includes gripper length)")
    print("   This means the calibration is Camera-to-TCP (gripper tip)")
elif link_name == 'tool0':
    print("\n⚠️ WARNING: Using 'tool0' link (flange, EXCLUDES gripper)")
    print("   This means calibration is Camera-to-Flange")
    print("   Need to add 13.8cm to get actual gripper tip!")
else:
    print(f"\n❓ UNKNOWN: Using '{link_name}' link")

print("\n" + "=" * 60)

# Cleanup
solver.disconnect()
