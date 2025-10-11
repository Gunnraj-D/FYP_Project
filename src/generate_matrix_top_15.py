#!/usr/bin/env python3
"""
Generate hand-eye matrix from ONLY the top 15 best poses.
"""
import numpy as np
import cv2
import json
from pathlib import Path
import sys

# Add src to path FIRST
sys.path.append(str(Path(__file__).parent))

from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS  # noqa: E402
from kinematics.kinematics_solver import InverseKinematicsSolver  # noqa: E402
from calibration.calibration_config import CalibrationConfig  # noqa: E402


def generate_from_top_15():
    """Generate matrix from top 15 poses only."""
    print("🎯 Generating Matrix from Top 15 Best Poses")
    print("=" * 60)

    # Top 15 poses based on reprojection error
    top_15_paths = [
        "calibration_captures/20251010_215058/pose_015",
        "calibration_captures/20251010_215058/pose_016",
        "calibration_captures/20251010_215058/pose_012",
        "calibration_captures/20251010_215058/pose_014",
        "calibration_captures/20251010_215058/pose_002",
        "calibration_captures/20251010_205126/pose_012",
        "calibration_captures/20251010_205126/pose_013",
        "calibration_captures/20251010_215058/pose_004",
        "calibration_captures/20251010_215058/pose_001",
        "calibration_captures/20251010_205126/pose_003",
        "calibration_captures/20251010_215058/pose_003",
        "calibration_captures/20251010_215058/pose_011",
        "calibration_captures/20251010_205126/pose_011",
        "calibration_captures/20251010_215058/pose_013",
        "calibration_captures/20251010_205126/pose_015",
    ]

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

    for i, pose_path in enumerate(top_15_paths):
        pose_dir = Path(pose_path)
        print(f"  Processing {pose_dir.name} ({i+1}/15)...")

        # Load joint angles
        with open(pose_dir / "joint_angles.json", 'r') as f:
            joint_angles = np.array(json.load(f))

        # Get TCP pose
        tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(joint_angles)
        R_gripper2base = tcp_matrix[:3, :3]
        t_gripper2base = tcp_matrix[:3, 3]

        # Load image and detect checkerboard
        color_image = cv2.imread(str(pose_dir / "color.png"))
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

    # Compute hand-eye calibration with Park method
    print(f"\n🧮 Computing hand-eye calibration...")

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

    print(f"\n📊 Top 15 Calibration Quality:")
    print(
        f"  Translation magnitude: {translation_magnitude:.4f} m ({translation_magnitude*100:.2f} cm)")
    print(f"  Rotation determinant: {rotation_det:.6f}")
    print(f"  Number of poses: 15")
    print(f"  Method: Park")

    # Display matrix
    print(f"\n📋 Hand-Eye Matrix (Camera to TCP):")
    print(f"  [ {T_cam2gripper[0, 0]:7.4f} {T_cam2gripper[0, 1]:7.4f} {T_cam2gripper[0, 2]:7.4f} {T_cam2gripper[0, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[1, 0]:7.4f} {T_cam2gripper[1, 1]:7.4f} {T_cam2gripper[1, 2]:7.4f} {T_cam2gripper[1, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[2, 0]:7.4f} {T_cam2gripper[2, 1]:7.4f} {T_cam2gripper[2, 2]:7.4f} {T_cam2gripper[2, 3]:7.4f}]")
    print(f"  [ {T_cam2gripper[3, 0]:7.4f} {T_cam2gripper[3, 1]:7.4f} {T_cam2gripper[3, 2]:7.4f} {T_cam2gripper[3, 3]:7.4f}]")

    # Save matrix
    np.save("hand_eye_matrix_top15.npy", T_cam2gripper)
    print(f"\n💾 Saved to: hand_eye_matrix_top15.npy")

    return T_cam2gripper


if __name__ == "__main__":
    generate_from_top_15()
