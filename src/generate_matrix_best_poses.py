#!/usr/bin/env python3
"""
Generate hand-eye matrix from only the best poses based on quality analysis.
"""
from calibration.calibration_config import CalibrationConfig
from kinematics.kinematics_solver import InverseKinematicsSolver
from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
import numpy as np
import cv2
import json
from pathlib import Path
import sys

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


def load_calibration_data_from_specific_poses():
    """
    Load calibration data from the best poses only.
    Based on the quality analysis, these are the best poses:
    """
    # Best poses based on quality analysis (excluding duplicates from different sessions)
    best_poses = [
        # From session 20251005_064821
        "calibration_captures/20251005_064821/pose_001",  # score: 0.899
        "calibration_captures/20251005_064821/pose_002",  # score: 0.898
        "calibration_captures/20251005_064821/pose_003",  # score: 0.832

        # From session 20251005_071304
        "calibration_captures/20251005_071304/pose_001",  # score: 0.807
        "calibration_captures/20251005_071304/pose_002",  # score: 0.852
        "calibration_captures/20251005_071304/pose_003",  # score: 0.785

        # From legacy poses
        "calibration_captures/pose_004",  # score: 0.856
        "calibration_captures/pose_005",  # score: 0.833
        "calibration_captures/pose_009",  # score: 0.816
        "calibration_captures/pose_011",  # score: 0.808
        "calibration_captures/pose_008",  # score: 0.802
        "calibration_captures/pose_013",  # score: 0.778
        "calibration_captures/pose_006",  # score: 0.725
    ]

    config = CalibrationConfig()
    chessboard_size = config.chessboard_size
    square_size = config.square_size

    # Prepare object points (3D points of the checkerboard)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0],
                           0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Camera matrix (approximate)
    camera_matrix = np.array(
        [[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1))

    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    print(f"🎯 Loading calibration data from {len(best_poses)} best poses...")

    for i, pose_path in enumerate(best_poses):
        pose_dir = Path(pose_path)
        if not pose_dir.exists():
            print(f"  ❌ Pose directory not found: {pose_path}")
            continue

        print(f"  Processing {pose_dir.name} ({i+1}/{len(best_poses)})...")

        # Load joint angles
        joints_path = pose_dir / "joint_angles.json"
        if not joints_path.exists():
            print(f"    ❌ No joint angles file")
            continue

        try:
            with open(joints_path, 'r') as f:
                joint_angles = np.array(json.load(f))
        except Exception as e:
            print(f"    ❌ Failed to load joint angles: {e}")
            continue

        # Get gripper pose from joint angles
        try:
            gripper_matrix, gripper_pose = kinematics_solver.tcp_from_joints(
                joint_angles)
            R_gripper2base = gripper_matrix[:3, :3]
            t_gripper2base = gripper_matrix[:3, 3]
        except Exception as e:
            print(f"    ❌ Failed to compute gripper pose: {e}")
            continue

        # Load and process image
        color_path = pose_dir / "color.png"
        if not color_path.exists():
            print(f"    ❌ No color image file")
            continue

        try:
            color_image = cv2.imread(str(color_path))
            if color_image is None:
                print(f"    ❌ Failed to load image")
                continue

            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Find checkerboard corners
            ret, corners = cv2.findChessboardCorners(
                gray, chessboard_size, None)
            if not ret:
                print(f"    ❌ Checkerboard not found")
                continue

            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)

            # Solve PnP to get target pose relative to camera
            ret, rvec, tvec = cv2.solvePnP(
                objp, corners, camera_matrix, dist_coeffs)
            if not ret:
                print(f"    ❌ PnP solve failed")
                continue

            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec.flatten()

            # Store the data
            R_gripper2base_list.append(R_gripper2base)
            t_gripper2base_list.append(t_gripper2base)
            R_target2cam_list.append(R_target2cam)
            t_target2cam_list.append(t_target2cam)

            print(f"    ✅ Successfully processed")

        except Exception as e:
            print(f"    ❌ Failed to process image: {e}")
            continue

    return R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list


def compute_hand_eye_calibration(R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list):
    """Compute hand-eye calibration matrix using OpenCV."""
    print(
        f"\n🧮 Computing hand-eye calibration from {len(R_gripper2base_list)} poses...")

    # Convert to numpy arrays
    R_gripper2base_array = np.array(R_gripper2base_list)
    t_gripper2base_array = np.array(t_gripper2base_list).reshape(-1, 3, 1)
    R_target2cam_array = np.array(R_target2cam_list)
    t_target2cam_array = np.array(t_target2cam_list).reshape(-1, 3, 1)

    # Compute hand-eye calibration
    result = cv2.calibrateHandEye(
        R_gripper2base_array, t_gripper2base_array,
        R_target2cam_array, t_target2cam_array,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    R_cam2gripper = result[0]
    t_cam2gripper = result[1]

    # Convert to 4x4 homogeneous matrix
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

    return T_cam2gripper


def analyze_calibration_quality(T_cam2gripper, R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list):
    """Analyze the quality of the calibration."""
    print(f"\n📈 Analyzing calibration quality...")

    # Check rotation matrix properties
    R = T_cam2gripper[:3, :3]
    t = T_cam2gripper[:3, 3]

    det = np.linalg.det(R)
    orthogonality_error = np.linalg.norm(R @ R.T - np.eye(3))

    print(f"Determinant of rotation matrix: {det:.6f} (should be ±1)")
    print(f"Orthogonality error: {orthogonality_error:.8f} (should be ~0)")

    # Compute calibration errors
    errors = []
    for i in range(len(R_gripper2base_list)):
        # Compute expected gripper pose from camera pose
        R_gripper2base = R_gripper2base_list[i]
        t_gripper2base = t_gripper2base_list[i]
        R_target2cam = R_target2cam_list[i]
        t_target2cam = t_target2cam_list[i]

        # Expected gripper pose: T_gripper = T_cam * T_cam2gripper
        T_cam = np.eye(4)
        T_cam[:3, :3] = R_target2cam.T  # Inverse of target2cam
        T_cam[:3, 3] = -R_target2cam.T @ t_target2cam

        T_gripper_expected = T_cam @ T_cam2gripper

        # Actual gripper pose
        T_gripper_actual = np.eye(4)
        T_gripper_actual[:3, :3] = R_gripper2base
        T_gripper_actual[:3, 3] = t_gripper2base

        # Compute error
        T_error = T_gripper_actual @ np.linalg.inv(T_gripper_expected)
        error = np.linalg.norm(T_error[:3, 3])
        errors.append(error)

    mean_error = np.mean(errors)
    max_error = np.max(errors)
    translation_magnitude = np.linalg.norm(t)

    print(f"Mean calibration error: {mean_error:.6f}")
    print(f"Max calibration error: {max_error:.6f}")
    print(
        f"Camera-to-gripper translation magnitude: {translation_magnitude:.4f} m")

    return mean_error, max_error, translation_magnitude


def main():
    """Main function to generate hand-eye matrix from best poses."""
    print("🎯 Hand-Eye Calibration Matrix Generation (Best Poses Only)")
    print("=" * 60)

    # Load calibration data from best poses
    R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list = load_calibration_data_from_specific_poses()

    if len(R_gripper2base_list) < 3:
        print(
            f"❌ Not enough valid poses ({len(R_gripper2base_list)}). Need at least 3.")
        return

    print(f"\n📊 Successfully loaded {len(R_gripper2base_list)} poses")

    # Compute hand-eye calibration
    T_cam2gripper = compute_hand_eye_calibration(
        R_gripper2base_list, t_gripper2base_list,
        R_target2cam_list, t_target2cam_list
    )

    # Analyze quality
    mean_error, max_error, translation_magnitude = analyze_calibration_quality(
        T_cam2gripper, R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list
    )

    # Display matrix
    print(f"\nHand-eye matrix (Camera to Gripper):")
    print(f"  [ {T_cam2gripper[0, 0]:7.4f} {T_cam2gripper[0, 1]:7.4f} {T_cam2gripper[0, 2]:7.4f} {T_cam2gripper[0, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[1, 0]:7.4f} {T_cam2gripper[1, 1]:7.4f} {T_cam2gripper[1, 2]:7.4f} {T_cam2gripper[1, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[2, 0]:7.4f} {T_cam2gripper[2, 1]:7.4f} {T_cam2gripper[2, 2]:7.4f} {T_cam2gripper[2, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[3, 0]:7.4f} {T_cam2gripper[3, 1]:7.4f} {T_cam2gripper[3, 2]:7.4f} {T_cam2gripper[3, 3]:7.4f}]")

    # Save matrix
    output_file = "src/hand_eye_matrix_best_poses.npy"
    np.save(output_file, T_cam2gripper)
    print(f"\n💾 Saved hand-eye matrix to: {output_file}")

    print(f"\n✅ Hand-eye calibration matrix generation completed!")
    print(f"   Used {len(R_gripper2base_list)} best poses out of 18 total")
    print(f"   Translation magnitude: {translation_magnitude:.4f}m")
    print(f"   Mean calibration error: {mean_error:.6f}")


if __name__ == "__main__":
    main()
