#!/usr/bin/env python3
"""
Generate hand-eye matrix from pruned high-quality poses.
"""
from calibration.calibration_config import CalibrationConfig
from kinematics.kinematics_solver import InverseKinematicsSolver
from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
import numpy as np
import cv2
import json
from pathlib import Path
import sys
from typing import List, Tuple

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


def load_calibration_data_from_poses(pose_dirs: List[Path]) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """
    Load calibration data from specific pose directories.

    Returns:
        R_gripper2base_list: List of rotation matrices (gripper to base)
        t_gripper2base_list: List of translation vectors (gripper to base)
        R_target2cam_list: List of rotation matrices (target to camera)
        t_target2cam_list: List of translation vectors (target to camera)
    """
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

    for pose_dir in pose_dirs:
        print(f"  Processing {pose_dir.name}...")

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


def compute_hand_eye_calibration(R_gripper2base_list: List[np.ndarray], t_gripper2base_list: List[np.ndarray],
                                 R_target2cam_list: List[np.ndarray], t_target2cam_list: List[np.ndarray]) -> np.ndarray:
    """
    Compute hand-eye calibration matrix using OpenCV.

    Returns:
        4x4 homogeneous transformation matrix (Camera to Gripper)
    """
    print(
        f"  Computing hand-eye calibration from {len(R_gripper2base_list)} poses...")

    # Convert to numpy arrays
    print(
        f"    Debug - Array lengths: R_gripper={len(R_gripper2base_list)}, t_gripper={len(t_gripper2base_list)}, R_target={len(R_target2cam_list)}, t_target={len(t_target2cam_list)}")

    R_gripper2base_array = np.array(R_gripper2base_list)
    t_gripper2base_array = np.array(t_gripper2base_list).reshape(-1, 3, 1)
    R_target2cam_array = np.array(R_target2cam_list)
    t_target2cam_array = np.array(t_target2cam_list).reshape(-1, 3, 1)

    print(
        f"    Debug - Array shapes: R_gripper={R_gripper2base_array.shape}, t_gripper={t_gripper2base_array.shape}, R_target={R_target2cam_array.shape}, t_target={t_target2cam_array.shape}")

    # Compute hand-eye calibration
    T_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base_array, t_gripper2base_array,
        R_target2cam_array, t_target2cam_array,
        method=cv2.CALIB_HAND_EYE_TSAI
    )[0]

    return T_cam2gripper


def generate_pruned_matrix():
    """Generate hand-eye matrix from the best poses."""
    print("🎯 Generating Hand-Eye Matrix from Pruned Poses")
    print("=" * 60)

    # Define the best poses based on the pruning analysis
    # These are the poses with the highest overall quality scores
    best_pose_names = [
        "pose_001",  # score: 0.899
        "pose_002",  # score: 0.898
        "pose_004",  # score: 0.856
        "pose_002",  # score: 0.852 (different session)
        "pose_005",  # score: 0.833
        "pose_003",  # score: 0.832
        "pose_009",  # score: 0.816
        "pose_011",  # score: 0.808
        "pose_001",  # score: 0.807 (different session)
        "pose_008",  # score: 0.802
        "pose_003",  # score: 0.796 (different session)
        "pose_003",  # score: 0.785 (different session)
        "pose_013",  # score: 0.778
        "pose_006",  # score: 0.725
    ]

    # Find all pose directories
    captures_base = Path("calibration_captures")
    if not captures_base.exists():
        print("❌ No calibration_captures directory found!")
        return

    # Collect all pose directories
    all_pose_dirs = []

    # Add timestamped sessions
    timestamped_dirs = [d for d in captures_base.iterdir(
    ) if d.is_dir() and d.name.startswith('2025')]
    for session_dir in timestamped_dirs:
        session_poses = [d for d in session_dir.iterdir(
        ) if d.is_dir() and d.name.startswith('pose_')]
        all_pose_dirs.extend(session_poses)
        print(
            f"📂 Found {len(session_poses)} poses in session {session_dir.name}")

    # Add legacy poses
    legacy_poses = [d for d in captures_base.iterdir(
    ) if d.is_dir() and d.name.startswith('pose_')]
    all_pose_dirs.extend(legacy_poses)
    print(f"📂 Found {len(legacy_poses)} legacy poses")

    print(f"📊 Total poses available: {len(all_pose_dirs)}")

    # Select the best poses
    selected_pose_dirs = []
    used_poses = set()  # Track which poses we've used to avoid duplicates

    for pose_name in best_pose_names:
        for pose_dir in all_pose_dirs:
            if pose_dir.name == pose_name and str(pose_dir) not in used_poses:
                selected_pose_dirs.append(pose_dir)
                used_poses.add(str(pose_dir))
                print(f"✅ Selected {pose_dir}")
                break

    print(f"\n📊 Selected {len(selected_pose_dirs)} poses for calibration")

    if len(selected_pose_dirs) < 3:
        print("❌ Not enough poses selected. Need at least 3.")
        return

    # Load calibration data
    print(f"\n📊 Loading calibration data...")
    R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list = load_calibration_data_from_poses(
        selected_pose_dirs)

    if len(R_gripper2base_list) < 3:
        print(
            f"❌ Not enough valid poses ({len(R_gripper2base_list)}). Need at least 3.")
        return

    # Compute hand-eye calibration
    print(f"\n🧮 Computing hand-eye calibration...")
    try:
        T_cam2gripper = compute_hand_eye_calibration(
            R_gripper2base_list, t_gripper2base_list,
            R_target2cam_list, t_target2cam_list
        )

        # Analyze quality
        translation_magnitude = np.linalg.norm(T_cam2gripper[:3, 3])
        rotation_det = np.linalg.det(T_cam2gripper[:3, :3])

        print(f"\n📊 Pruned Matrix Quality:")
        print(f"  Translation magnitude: {translation_magnitude:.3f}m")
        print(f"  Rotation determinant: {rotation_det:.6f}")
        print(f"  Number of poses used: {len(R_gripper2base_list)}")

        # Save pruned matrix
        output_file = "src/hand_eye_matrix_pruned.npy"
        np.save(output_file, T_cam2gripper)
        print(f"💾 Saved pruned hand-eye matrix to: {output_file}")

        # Display matrix
        print(f"\n📋 Pruned Hand-Eye Matrix:")
        print(f"  [ {T_cam2gripper[0, 0]:7.4f} {T_cam2gripper[0, 1]:7.4f} {T_cam2gripper[0, 2]:7.4f} {T_cam2gripper[0, 3]:7.4f}]")
        print(f"  [ {T_cam2gripper[1, 0]:7.4f} {T_cam2gripper[1, 1]:7.4f} {T_cam2gripper[1, 2]:7.4f} {T_cam2gripper[1, 3]:7.4f}]")
        print(f"  [ {T_cam2gripper[2, 0]:7.4f} {T_cam2gripper[2, 1]:7.4f} {T_cam2gripper[2, 2]:7.4f} {T_cam2gripper[2, 3]:7.4f}]")
        print(f"  [ {T_cam2gripper[3, 0]:7.4f} {T_cam2gripper[3, 1]:7.4f} {T_cam2gripper[3, 2]:7.4f} {T_cam2gripper[3, 3]:7.4f}]")

        print(f"\n🎉 Pruned calibration complete!")
        print(f"   Matrix saved to: {output_file}")
        print(f"   You can now update config.py with this improved matrix")

    except Exception as e:
        print(f"❌ Failed to compute hand-eye calibration: {e}")


if __name__ == "__main__":
    generate_pruned_matrix()
