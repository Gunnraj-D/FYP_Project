#!/usr/bin/env python3
"""
Verify the new hand-eye calibration matrix is loaded correctly.
"""
from config.camera_config import (
    HAND_EYE_MATRIX_CALIBRATED,
    CAMERA_TRANSFORM_MODE,
    get_camera_transform_info,
    print_camera_transform_info
)
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent))


print("🔍 Verifying New Hand-Eye Calibration")
print("=" * 60)

# Print the loaded matrix
print("\n📐 Loaded Hand-Eye Matrix (Camera to TCP):")
for i in range(4):
    print(f"  [{HAND_EYE_MATRIX_CALIBRATED[i,0]:8.4f} {HAND_EYE_MATRIX_CALIBRATED[i,1]:8.4f} {HAND_EYE_MATRIX_CALIBRATED[i,2]:8.4f} {HAND_EYE_MATRIX_CALIBRATED[i,3]:8.4f}]")

# Extract and display components
R = HAND_EYE_MATRIX_CALIBRATED[:3, :3]
t = HAND_EYE_MATRIX_CALIBRATED[:3, 3]

print(f"\n📏 Translation (Camera to TCP):")
print(f"  X: {t[0]:.4f} m ({t[0]*1000:.1f} mm)")
print(f"  Y: {t[1]:.4f} m ({t[1]*1000:.1f} mm)")
print(f"  Z: {t[2]:.4f} m ({t[2]*1000:.1f} mm)")
print(
    f"  Magnitude: {np.linalg.norm(t):.4f} m ({np.linalg.norm(t)*100:.2f} cm)")

# Check rotation properties
det_R = np.linalg.det(R)
ortho_error = np.linalg.norm(R.T @ R - np.eye(3))

print(f"\n🔄 Rotation Matrix Validation:")
print(f"  Determinant: {det_R:.8f} (should be ±1)")
print(f"  Orthogonality error: {ortho_error:.8e} (should be ~0)")

# Show mode
print(f"\n⚙️ Camera Transform Mode: {CAMERA_TRANSFORM_MODE}")

# Print full info
print_camera_transform_info()

# Verify the matrix file exists
matrix_file = Path("src/hand_eye_matrix.npy")
if matrix_file.exists():
    loaded_matrix = np.load(matrix_file)
    if np.allclose(loaded_matrix, HAND_EYE_MATRIX_CALIBRATED):
        print("✅ Matrix file matches config!")
    else:
        print("⚠️ Matrix file differs from config!")
        print(
            f"  Max difference: {np.max(np.abs(loaded_matrix - HAND_EYE_MATRIX_CALIBRATED))}")
else:
    print("❌ Matrix file not found!")

print("\n" + "=" * 60)
print("✅ Verification Complete!")

