#!/usr/bin/env python3
"""
Visualize the camera position relative to TCP based on hand-eye calibration.
"""
from config.camera_config import HAND_EYE_MATRIX_CALIBRATED
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent))


print("📷 Camera Position Relative to TCP (Gripper Tip)")
print("=" * 60)

# Extract translation from hand-eye matrix
translation = HAND_EYE_MATRIX_CALIBRATED[:3, 3]
distance = np.linalg.norm(translation)

print(f"\n📏 Translation Vector (Camera to TCP):")
print(f"  X: {translation[0]:7.4f} m = {translation[0]*1000:6.1f} mm")
print(f"  Y: {translation[1]:7.4f} m = {translation[1]*1000:6.1f} mm")
print(f"  Z: {translation[2]:7.4f} m = {translation[2]*1000:6.1f} mm")
print(
    f"\n  Total Distance: {distance:.4f} m = {distance*100:.2f} cm = {distance*1000:.1f} mm")

print(f"\n🎯 Physical Interpretation:")
print(
    f"  The camera is mounted approximately {distance*100:.1f} cm from the gripper tip.")
print(f"  ")
print(f"  Breakdown:")
print(f"  - Forward/Back (X): {abs(translation[0]*1000):.1f} mm")
print(
    f"  - Left/Right (Y):   {abs(translation[1]*1000):.1f} mm ← **DOMINANT**")
print(f"  - Up/Down (Z):      {abs(translation[2]*1000):.1f} mm")

print(f"\n📐 Visual Layout (Top View):")
print(f"")
print(f"        TCP (Gripper Tip)")
print(f"            ●")
print(f"            |")
print(f"            | ~{translation[1]*1000:.0f} mm (Y offset)")
print(f"            |")
print(f"            ●")
print(f"        Camera")

print(f"\n🔄 Rotation: ~178.5° yaw (camera facing opposite direction)")
print(f"   This means the camera is mounted roughly backwards/upside-down")
print(f"   relative to the gripper's forward direction.")

print("\n" + "=" * 60)
print(
    f"✅ Yes, your camera is approximately {distance*100:.1f} cm from the TCP!")

