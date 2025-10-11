#!/usr/bin/env python3
"""
Analyze new calibration poses to identify duplicates and quality.
"""
import numpy as np
import cv2
import json
from pathlib import Path
import sys
from typing import List, Dict

# Add src to path
sys.path.append(str(Path(__file__).parent))

from config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS  # noqa: E402
from kinematics.kinematics_solver import InverseKinematicsSolver  # noqa: E402
from calibration.calibration_config import CalibrationConfig  # noqa: E402


def analyze_poses():
    """Analyze all poses for duplicates and quality."""
    print("🔍 Analyzing New Calibration Poses")
    print("=" * 60)

    config = CalibrationConfig()
    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    # Find all calibration sessions
    captures_base = Path("calibration_captures")
    all_sessions = [d for d in captures_base.iterdir(
    ) if d.is_dir() and d.name.startswith('2025')]

    print(f"📂 Found {len(all_sessions)} calibration sessions")

    # Collect all poses
    all_poses = []
    for session in all_sessions:
        pose_dirs = [d for d in session.iterdir() if d.is_dir()
                     and d.name.startswith('pose_')]
        for pose_dir in pose_dirs:
            joints_path = pose_dir / "joint_angles.json"
            color_path = pose_dir / "color.png"

            if not joints_path.exists() or not color_path.exists():
                continue

            with open(joints_path, 'r') as f:
                joint_angles = np.array(json.load(f))

            tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(
                joint_angles)

            # Detect checkerboard
            color_image = cv2.imread(str(color_path))
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(
                gray, config.chessboard_size, None)

            reprojection_error = None
            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS +
                            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)

                # Compute reprojection error
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

            pose_info = {
                'path': str(pose_dir),
                'name': f"{session.name}/{pose_dir.name}",
                'joint_angles': joint_angles,
                'tcp_position': tcp_matrix[:3, 3],
                'tcp_matrix': tcp_matrix,
                'checkerboard_detected': ret,
                'reprojection_error': reprojection_error if reprojection_error else float('inf')
            }

            all_poses.append(pose_info)

    print(f"📊 Total poses found: {len(all_poses)}")

    # Check for duplicates (similar joint angles)
    print(f"\n🔍 Checking for duplicate poses...")
    duplicates = []
    unique_poses = []

    for i, pose in enumerate(all_poses):
        is_duplicate = False
        for unique_pose in unique_poses:
            # Check if joint angles are very similar (within 0.01 radians = 0.57 degrees)
            if np.allclose(pose['joint_angles'], unique_pose['joint_angles'], atol=0.01):
                duplicates.append((pose['name'], unique_pose['name']))
                is_duplicate = True
                break

        if not is_duplicate:
            unique_poses.append(pose)

    print(f"  Unique poses: {len(unique_poses)}")
    print(f"  Duplicates found: {len(duplicates)}")

    if duplicates:
        print(f"\n📋 Duplicate Poses:")
        for dup_name, original_name in duplicates:
            print(f"  - {dup_name} (duplicate of {original_name})")

    # Analyze quality
    print(f"\n📊 Pose Quality Analysis:")
    valid_poses = [p for p in unique_poses if p['checkerboard_detected']]
    invalid_poses = [p for p in unique_poses if not p['checkerboard_detected']]

    print(f"  Valid poses (checkerboard detected): {len(valid_poses)}")
    print(f"  Invalid poses (no checkerboard): {len(invalid_poses)}")

    if invalid_poses:
        print(f"\n❌ Invalid Poses (no checkerboard):")
        for pose in invalid_poses:
            print(f"  - {pose['name']}")

    # Sort valid poses by reprojection error
    valid_poses.sort(key=lambda x: x['reprojection_error'])

    print(f"\n🏆 Top 15 Best Poses (by reprojection error):")
    print(f"{'Rank':<5} {'Pose':<35} {'Reproj Error':<15}")
    print("-" * 60)

    best_poses = valid_poses[:15] if len(valid_poses) >= 15 else valid_poses
    for i, pose in enumerate(best_poses):
        print(
            f"{i+1:<5} {pose['name']:<35} {pose['reprojection_error']:<15.4f}")

    # Save best pose paths
    best_pose_paths = [pose['path'] for pose in best_poses]

    with open('best_pose_paths.json', 'w') as f:
        json.dump(best_pose_paths, f, indent=2)

    print(
        f"\n💾 Saved {len(best_pose_paths)} best pose paths to: best_pose_paths.json")

    return best_pose_paths


if __name__ == "__main__":
    analyze_poses()
