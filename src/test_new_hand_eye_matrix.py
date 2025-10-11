#!/usr/bin/env python3
"""
Test script to verify the new hand-eye calibration matrix integration.
"""
import numpy as np
from config.config import (
    HAND_EYE_MATRIX_CALIBRATED,
    HAND_EYE_MATRIX,
    CAMERA_TRANSFORM_MODE,
    print_camera_transform_info
)

print("🎯 Hand-Eye Matrix Verification")
print("=" * 60)

# Check mode
print(f"\n📐 Camera Transform Mode: {CAMERA_TRANSFORM_MODE}")

# Check if calibrated matrix is active
if CAMERA_TRANSFORM_MODE == 'calibrated':
    print("✅ Using calibrated hand-eye matrix")
else:
    print("⚠️  Using simple hand-eye matrix")

# Display matrix info
print_camera_transform_info()

# Verify matrix values
print("\n📊 Matrix Details:")
print(f"Translation: {HAND_EYE_MATRIX[:3, 3]}")
translation_magnitude = np.linalg.norm(HAND_EYE_MATRIX[:3, 3])
print(
    f"Translation Magnitude: {translation_magnitude:.4f}m ({translation_magnitude*100:.2f}cm)")

# Check rotation matrix properties
R = HAND_EYE_MATRIX[:3, :3]
det = np.linalg.det(R)
orthogonality_error = np.linalg.norm(R @ R.T - np.eye(3))
print(f"\n🔍 Matrix Quality:")
print(f"Determinant: {det:.6f} (should be ±1)")
print(f"Orthogonality error: {orthogonality_error:.10f} (should be ~0)")

# Test transformation
test_point_camera = np.array([0.1, 0.2, 0.3, 1.0])  # Homogeneous coordinates
test_point_tcp = HAND_EYE_MATRIX @ test_point_camera
print(f"\n🧪 Test Transformation:")
print(f"Camera point: {test_point_camera[:3]}")
print(f"TCP point: {test_point_tcp[:3]}")

# Test inverse transformation
HAND_EYE_MATRIX_INV = np.linalg.inv(HAND_EYE_MATRIX)
test_point_camera_back = HAND_EYE_MATRIX_INV @ test_point_tcp
print(f"Round-trip back to camera: {test_point_camera_back[:3]}")
round_trip_error = np.linalg.norm(
    test_point_camera[:3] - test_point_camera_back[:3])
print(f"Round-trip error: {round_trip_error:.10f}m")

if round_trip_error < 1e-6:
    print("✅ Round-trip transformation successful!")
else:
    print("❌ Round-trip error too large!")

print("\n" + "=" * 60)
print("✅ Hand-eye matrix verification complete!")
