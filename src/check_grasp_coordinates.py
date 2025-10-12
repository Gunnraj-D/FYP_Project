"""
Quick diagnostic script to check grasp coordinate transformation setup.

Run this to verify:
1. Hand-eye calibration is loaded
2. Camera transform mode is correct
3. Matrix values are reasonable

Usage:
    python src/check_grasp_coordinates.py
"""

from config import (
    HAND_EYE_MATRIX,
    CAMERA_TRANSFORM_MODE,
    CAMERA_TRANSLATION,
    CAMERA_ROTATION_EULER,
    print_camera_transform_info
)
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))


def check_matrix_is_valid(matrix, name="Matrix"):
    """Check if transformation matrix is reasonable."""
    issues = []

    # Check if identity
    if np.allclose(matrix, np.eye(4), atol=1e-3):
        issues.append(f"❌ {name} is identity matrix (no transformation!)")

    # Check if translation is zero
    translation = matrix[:3, 3]
    if np.allclose(translation, 0, atol=1e-3):
        issues.append(f"⚠️  {name} has zero translation")

    # Check if translation is reasonable (0.05m to 0.5m typical)
    trans_magnitude = np.linalg.norm(translation)
    if trans_magnitude < 0.03:
        issues.append(
            f"⚠️  {name} translation very small: {trans_magnitude:.3f}m")
    elif trans_magnitude > 1.0:
        issues.append(
            f"⚠️  {name} translation very large: {trans_magnitude:.3f}m")

    return issues, translation


def main():
    print("="*70)
    print("🔍 GRASP COORDINATE TRANSFORMATION DIAGNOSTIC")
    print("="*70)

    # 1. Check camera transform mode
    print(f"\n1️⃣  Camera Transform Mode: {CAMERA_TRANSFORM_MODE}")
    if CAMERA_TRANSFORM_MODE != 'calibrated':
        print(f"   ⚠️  WARNING: Not using 'calibrated' mode!")
        print(f"   ⚠️  This may cause incorrect transformations!")
    else:
        print(f"   ✅ Using calibrated mode (recommended)")

    # 2. Check hand-eye matrix
    print(f"\n2️⃣  Hand-Eye Calibration Matrix:")
    print(HAND_EYE_MATRIX)

    issues, translation = check_matrix_is_valid(
        HAND_EYE_MATRIX, "Hand-eye matrix")

    if issues:
        print(f"\n   Issues found:")
        for issue in issues:
            print(f"   {issue}")
    else:
        print(f"\n   ✅ Matrix looks reasonable")

    print(f"\n   Translation (camera offset from TCP):")
    print(
        f"   X: {translation[0]:+.3f}m, Y: {translation[1]:+.3f}m, Z: {translation[2]:+.3f}m")
    print(f"   Magnitude: {np.linalg.norm(translation):.3f}m")

    # 3. Check simple mode values (for comparison)
    print(f"\n3️⃣  Simple Mode Configuration (for reference):")
    print(f"   Translation: {CAMERA_TRANSLATION}")
    print(f"   Rotation (Euler): {CAMERA_ROTATION_EULER}")

    # 4. Test transformation with sample point
    print(f"\n4️⃣  Test Transformation:")
    print(f"   Testing point in camera frame: (0.2, 0.1, 0.5)")

    # Sample TCP matrix (robot arm extended forward)
    sample_tcp = np.array([
        [1, 0, 0, 0.5],    # TCP at X=0.5m
        [0, 1, 0, 0.0],    # Y=0.0m
        [0, 0, 1, 0.4],    # Z=0.4m
        [0, 0, 0, 1]
    ])

    # Point in camera frame
    point_camera = np.array([0.2, 0.1, 0.5, 1.0])  # Homogeneous

    # Transform: base = TCP @ hand_eye @ camera_point
    point_base = sample_tcp @ HAND_EYE_MATRIX @ point_camera

    print(
        f"\n   Sample TCP position: ({sample_tcp[0,3]:.3f}, {sample_tcp[1,3]:.3f}, {sample_tcp[2,3]:.3f})")
    print(f"   Point in camera: (0.200, 0.100, 0.500)")
    print(
        f"   Point in base:   ({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})")

    # Check if result is reasonable
    if np.allclose(point_base[:3], [0, 0, 0], atol=0.05):
        print(f"   ❌ PROBLEM: Result is at origin! Transform not working!")
    elif point_base[0] < 0.2:
        print(f"   ⚠️  WARNING: X coordinate very small, check calibration")
    else:
        print(f"   ✅ Result looks reasonable (not at origin)")

    # 5. Summary
    print(f"\n" + "="*70)
    print(f"📊 SUMMARY")
    print(f"="*70)

    all_good = True

    if CAMERA_TRANSFORM_MODE != 'calibrated':
        print(f"❌ Camera transform mode should be 'calibrated'")
        all_good = False
    else:
        print(f"✅ Camera transform mode: OK")

    if np.allclose(HAND_EYE_MATRIX, np.eye(4), atol=1e-3):
        print(f"❌ Hand-eye matrix is identity (NOT CALIBRATED!)")
        all_good = False
    else:
        print(f"✅ Hand-eye matrix: OK")

    trans_mag = np.linalg.norm(HAND_EYE_MATRIX[:3, 3])
    if trans_mag < 0.03:
        print(
            f"⚠️  Camera offset very small: {trans_mag:.3f}m (check calibration)")
        all_good = False
    else:
        print(f"✅ Camera offset: {trans_mag:.3f}m (reasonable)")

    if np.allclose(point_base[:3], [0, 0, 0], atol=0.05):
        print(f"❌ Test transformation produces origin (BAD!)")
        all_good = False
    else:
        print(f"✅ Test transformation: OK")

    print(f"\n" + "="*70)
    if all_good:
        print(f"✅ All checks passed! Coordinate transformation should work.")
    else:
        print(f"❌ Issues found! Grasp coordinates may be wrong.")
        print(f"\n🔧 Recommended actions:")
        print(f"   1. Check if hand_eye_matrix.npy file exists")
        print(f"   2. Verify camera transform mode is 'calibrated'")
        print(f"   3. Re-run hand-eye calibration if needed")
        print(f"\n📖 See GRASP_COORDINATE_DIAGNOSTIC.md for detailed troubleshooting")
    print(f"="*70)


if __name__ == "__main__":
    main()

