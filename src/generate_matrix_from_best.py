#!/usr/bin/env python3
"""
Generate hand-eye matrix from best poses (excluding duplicates and low quality).
Does not delete any pose data - just skips them during matrix generation.
"""
import numpy as np
import cv2
import json
from pathlib import Path
import sys
from typing import List, Dict, Tuple

# Add src to path FIRST
sys.path.append(str(Path(__file__).parent))

from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS  # noqa: E402
from kinematics.kinematics_solver import InverseKinematicsSolver  # noqa: E402
from calibration.calibration_config import CalibrationConfig  # noqa: E402


def analyze_and_select_best_poses():
    """Analyze all poses and select the best ones, skipping duplicates."""
    print("🔍 Analyzing Calibration Poses")
    print("=" * 60)

    config = CalibrationConfig()
    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    # Find all sessions
    captures_base = Path("calibration_captures")
    all_sessions = [d for d in captures_base.iterdir(
    ) if d.is_dir() and d.name.startswith('2025')]

    print(f"📂 Found {len(all_sessions)} sessions")

    # Collect all poses with quality metrics
    all_pose_data = []

    for session in all_sessions:
        pose_dirs = sorted([d for d in session.iterdir()
                           if d.is_dir() and d.name.startswith('pose_')])
        print(f"  Session {session.name}: {len(pose_dirs)} poses")

        for pose_dir in pose_dirs:
            joints_path = pose_dir / "joint_angles.json"
            color_path = pose_dir / "color.png"

            if not joints_path.exists() or not color_path.exists():
                print(f"    ❌ {pose_dir.name}: Missing files")
                continue

            # Load joint angles
            with open(joints_path, 'r') as f:
                joint_angles = np.array(json.load(f))

            # Get TCP pose
            tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(
                joint_angles)
            tcp_position = tcp_matrix[:3, 3]

            # Analyze image quality
            color_image = cv2.imread(str(color_path))
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(
                gray, config.chessboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            reprojection_error = float('inf')
            if ret:
                # Refine corners
                criteria = (cv2.TERM_CRITERIA_EPS +
                            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)

                # Calculate reprojection error
                objp = np.zeros(
                    (config.chessboard_size[0] * config.chessboard_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:config.chessboard_size[0],
                                       0:config.chessboard_size[1]].T.reshape(-1, 2)
                objp *= config.square_size

                camera_matrix = np.array(
                    [[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
                dist_coeffs = np.zeros((4, 1))

                ret_pnp, rvec, tvec = cv2.solvePnP(
                    objp, corners, camera_matrix, dist_coeffs)
                if ret_pnp:
                    projected_points, _ = cv2.projectPoints(
                        objp, rvec, tvec, camera_matrix, dist_coeffs)
                    reprojection_error = cv2.norm(
                        corners, projected_points, cv2.NORM_L2) / len(corners)

            pose_data = {
                'path': pose_dir,
                'name': f"{session.name}/{pose_dir.name}",
                'joint_angles': joint_angles,
                'tcp_position': tcp_position,
                'tcp_matrix': tcp_matrix,
                'checkerboard_detected': ret,
                'reprojection_error': reprojection_error,
                'status': 'valid' if ret else 'no_checkerboard'
            }

            all_pose_data.append(pose_data)

    print(f"\n📊 Total poses: {len(all_pose_data)}")

    # Identify duplicates (similar joint configurations)
    print(f"\n🔍 Identifying duplicates...")
    unique_poses = []
    duplicate_count = 0

    for pose in all_pose_data:
        is_duplicate = False

        for unique_pose in unique_poses:
            # Check if joint angles are very similar (within 0.01 rad ≈ 0.57°)
            joint_diff = np.linalg.norm(
                pose['joint_angles'] - unique_pose['joint_angles'])

            if joint_diff < 0.05:  # 0.05 radians ≈ 2.86 degrees
                is_duplicate = True
                duplicate_count += 1
                pose['status'] = f'duplicate_of_{unique_pose["name"]}'
                print(
                    f"  ⚠️ {pose['name']} is duplicate of {unique_pose['name']} (joint diff: {joint_diff:.4f} rad)")
                break

        if not is_duplicate:
            unique_poses.append(pose)

    print(f"  Unique poses: {len(unique_poses)}")
    print(f"  Duplicates: {duplicate_count}")

    # Filter valid unique poses
    valid_unique_poses = [
        p for p in unique_poses if p['checkerboard_detected']]
    print(f"  Valid unique poses: {len(valid_unique_poses)}")

    # Sort by reprojection error (best first)
    valid_unique_poses.sort(key=lambda x: x['reprojection_error'])

    print(f"\n🏆 Best Poses (sorted by quality):")
    print(f"{'Rank':<5} {'Pose':<40} {'Reproj Error':<15} {'Status':<20}")
    print("-" * 85)

    for i, pose in enumerate(valid_unique_poses):
        print(
            f"{i+1:<5} {pose['name']:<40} {pose['reprojection_error']:<15.4f} {pose['status']:<20}")

    # Write status report
    print(f"\n📝 Writing pose status report...")
    with open('pose_quality_report.txt', 'w') as f:
        f.write("="*60 + "\n")
        f.write("Calibration Pose Quality Report\n")
        f.write("="*60 + "\n\n")
        f.write(f"Total poses: {len(all_pose_data)}\n")
        f.write(f"Unique poses: {len(unique_poses)}\n")
        f.write(f"Duplicates: {duplicate_count}\n")
        f.write(
            f"Valid poses with checkerboard: {len(valid_unique_poses)}\n\n")

        f.write("Duplicate Poses (SKIPPED):\n")
        for pose in all_pose_data:
            if pose['status'].startswith('duplicate'):
                f.write(f"  - {pose['name']}: {pose['status']}\n")

        f.write("\nInvalid Poses (SKIPPED):\n")
        for pose in all_pose_data:
            if pose['status'] == 'no_checkerboard':
                f.write(f"  - {pose['name']}: No checkerboard detected\n")

        f.write("\nBest Poses (USED for matrix generation):\n")
        for i, pose in enumerate(valid_unique_poses):
            f.write(
                f"  {i+1}. {pose['name']}: Reproj error = {pose['reprojection_error']:.4f}\n")

    print(f"💾 Saved detailed report to: pose_quality_report.txt")

    return valid_unique_poses


def generate_matrix_from_best_poses(best_poses: List[Dict]):
    """Generate hand-eye matrix from selected best poses."""
    print(f"\n🧮 Generating Hand-Eye Matrix from {len(best_poses)} Best Poses")
    print("=" * 60)

    config = CalibrationConfig()
    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    objp = np.zeros(
        (config.chessboard_size[0] * config.chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:config.chessboard_size[0],
                           0:config.chessboard_size[1]].T.reshape(-1, 2)
    objp *= config.square_size

    camera_matrix = np.array(
        [[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1))

    for i, pose_data in enumerate(best_poses):
        print(f"  Processing {pose_data['name']} ({i+1}/{len(best_poses)})...")

        pose_dir = pose_data['path']

        # Load joint angles
        joints_path = pose_dir / "joint_angles.json"
        with open(joints_path, 'r') as f:
            joint_angles = np.array(json.load(f))

        # Get TCP pose
        tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(joint_angles)
        R_gripper2base = tcp_matrix[:3, :3]
        t_gripper2base = tcp_matrix[:3, 3]

        # Load image and detect checkerboard
        color_path = pose_dir / "color.png"
        color_image = cv2.imread(str(color_path))
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(
            gray, config.chessboard_size, None)
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Solve PnP
        ret_pnp, rvec, tvec = cv2.solvePnP(
            objp, corners, camera_matrix, dist_coeffs)
        R_target2cam, _ = cv2.Rodrigues(rvec)
        t_target2cam = tvec.flatten()

        # Store data
        R_gripper2base_list.append(R_gripper2base)
        t_gripper2base_list.append(t_gripper2base)
        R_target2cam_list.append(R_target2cam)
        t_target2cam_list.append(t_target2cam)

        print(f"    ✅ Added to calibration")

    # Compute hand-eye calibration with Park method
    print(f"\n🧮 Computing hand-eye calibration using Park method...")

    R_gripper2base_array = np.array(R_gripper2base_list)
    t_gripper2base_array = np.array(t_gripper2base_list).reshape(-1, 3, 1)
    R_target2cam_array = np.array(R_target2cam_list)
    t_target2cam_array = np.array(t_target2cam_list).reshape(-1, 3, 1)

    result = cv2.calibrateHandEye(
        R_gripper2base_array, t_gripper2base_array,
        R_target2cam_array, t_target2cam_array,
        method=cv2.CALIB_HAND_EYE_PARK
    )

    R_cam2gripper = result[0]
    t_cam2gripper = result[1]

    # Convert to 4x4 homogeneous matrix
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

    # Analyze quality
    translation_magnitude = np.linalg.norm(T_cam2gripper[:3, 3])
    rotation_det = np.linalg.det(T_cam2gripper[:3, :3])

    print(f"\n📊 Calibration Quality:")
    print(
        f"  Translation magnitude: {translation_magnitude:.4f} m ({translation_magnitude*100:.2f} cm)")
    print(f"  Rotation determinant: {rotation_det:.6f} (should be ±1)")
    print(f"  Number of poses used: {len(R_gripper2base_list)}")
    print(f"  Method: Park")

    # Display matrix
    print(f"\n📋 Hand-Eye Matrix (Camera to TCP):")
    print(f"  [ {T_cam2gripper[0, 0]:7.4f} {T_cam2gripper[0, 1]:7.4f} {T_cam2gripper[0, 2]:7.4f} {T_cam2gripper[0, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[1, 0]:7.4f} {T_cam2gripper[1, 1]:7.4f} {T_cam2gripper[1, 2]:7.4f} {T_cam2gripper[1, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[2, 0]:7.4f} {T_cam2gripper[2, 1]:7.4f} {T_cam2gripper[2, 2]:7.4f} {T_cam2gripper[2, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[3, 0]:7.4f} {T_cam2gripper[3, 1]:7.4f} {T_cam2gripper[3, 2]:7.4f} {T_cam2gripper[3, 3]:7.4f}]")

    # Save matrix
    output_file = "hand_eye_matrix_from_best.npy"
    np.save(output_file, T_cam2gripper)
    print(f"\n💾 Saved hand-eye matrix to: {output_file}")

    return T_cam2gripper


def main():
    """Main function."""
    print("🎯 Hand-Eye Calibration from Best Poses")
    print("=" * 60 + "\n")

    # Analyze and select best poses
    best_poses = analyze_and_select_best_poses()

    if len(best_poses) < 3:
        print(
            f"\n❌ Not enough valid poses ({len(best_poses)}). Need at least 3.")
        return

    # Generate matrix
    T_cam2gripper = generate_matrix_from_best_poses(best_poses)

    print(f"\n✅ Calibration Complete!")
    print(f"   Matrix saved to: hand_eye_matrix_from_best.npy")
    print(f"   Quality report: pose_quality_report.txt")
    print(f"   All original pose data preserved (nothing deleted)")


if __name__ == "__main__":
    main()
