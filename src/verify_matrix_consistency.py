#!/usr/bin/env python3
"""
Verify that the hand-eye matrix in config matches the generated file
and check for any issues.
"""
from config.camera_config import CAMERA_TRANSFORM_MODE
from config.camera_config import HAND_EYE_MATRIX_CALIBRATED
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent))


print("🔍 Verifying Hand-Eye Matrix Consistency")
print("=" * 60)

# Load the generated matrix
matrix_file = Path("hand_eye_matrix_generated.npy")
if matrix_file.exists():
    generated_matrix = np.load(matrix_file)
    print("\n✅ Found generated matrix file")
else:
    print("\n❌ Generated matrix file not found!")
    generated_matrix = None

# Load the matrix from src directory
src_matrix_file = Path("src/hand_eye_matrix.npy")
if src_matrix_file.exists():
    src_matrix = np.load(src_matrix_file)
    print("✅ Found src matrix file")
else:
    print("❌ Src matrix file not found!")
    src_matrix = None

print("\n📐 Matrix in Config:")
for i in range(4):
    print(f"  [{HAND_EYE_MATRIX_CALIBRATED[i,0]:8.4f} {HAND_EYE_MATRIX_CALIBRATED[i,1]:8.4f} {HAND_EYE_MATRIX_CALIBRATED[i,2]:8.4f} {HAND_EYE_MATRIX_CALIBRATED[i,3]:8.4f}]")

if generated_matrix is not None:
    print("\n📐 Matrix in generated file:")
    for i in range(4):
        print(f"  [{generated_matrix[i,0]:8.4f} {generated_matrix[i,1]:8.4f} {generated_matrix[i,2]:8.4f} {generated_matrix[i,3]:8.4f}]")

    diff = np.max(np.abs(generated_matrix - HAND_EYE_MATRIX_CALIBRATED))
    print(f"\n📊 Max difference: {diff:.8f}")
    if diff < 1e-4:
        print("✅ Config matches generated file!")
    else:
        print(f"⚠️ Config differs from generated file!")

if src_matrix is not None:
    print("\n📐 Matrix in src/ file:")
    for i in range(4):
        print(
            f"  [{src_matrix[i,0]:8.4f} {src_matrix[i,1]:8.4f} {src_matrix[i,2]:8.4f} {src_matrix[i,3]:8.4f}]")

    diff = np.max(np.abs(src_matrix - HAND_EYE_MATRIX_CALIBRATED))
    print(f"\n📊 Max difference: {diff:.8f}")
    if diff < 1e-4:
        print("✅ Config matches src matrix file!")
    else:
        print(f"⚠️ Config differs from src matrix file!")

# Check matrix properties
print("\n🔍 Validating Matrix Properties:")
R = HAND_EYE_MATRIX_CALIBRATED[:3, :3]
t = HAND_EYE_MATRIX_CALIBRATED[:3, 3]

# Check determinant
det = np.linalg.det(R)
print(f"\nRotation determinant: {det:.8f}")
if abs(det - 1.0) < 0.01:
    print("✅ Determinant is ~1 (valid rotation)")
else:
    print(f"❌ Determinant is not 1! Matrix may be corrupted!")

# Check orthogonality
ortho_error = np.linalg.norm(R @ R.T - np.eye(3))
print(f"\nOrthogonality error: {ortho_error:.8e}")
if ortho_error < 0.01:
    print("✅ Matrix is orthogonal (valid rotation)")
else:
    print(f"❌ Matrix is not orthogonal! Matrix may be corrupted!")

# Check translation
t_magnitude = np.linalg.norm(t)
print(
    f"\nTranslation magnitude: {t_magnitude:.4f} m ({t_magnitude*100:.2f} cm)")
if 0.05 < t_magnitude < 0.2:  # Between 5cm and 20cm is reasonable
    print("✅ Translation is reasonable")
else:
    print(f"⚠️ Translation seems unusual (expected ~6-7cm)")

# Check if matrix is identity-ish (wrong!)
if np.allclose(HAND_EYE_MATRIX_CALIBRATED, np.eye(4), atol=0.1):
    print("\n❌ PROBLEM: Matrix is close to identity!")
    print("   This would mean no transformation, which is wrong!")

# Check camera transform mode
print(f"\n⚙️ Camera Transform Mode: {CAMERA_TRANSFORM_MODE}")
if CAMERA_TRANSFORM_MODE != 'calibrated':
    print("⚠️ WARNING: Not using calibrated mode!")
    print("   Set CAMERA_TRANSFORM_MODE = 'calibrated' to use the calibrated matrix")

print("\n" + "=" * 60)
print("✅ Verification complete!")


