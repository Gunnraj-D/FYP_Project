#!/usr/bin/env python3
"""
Generate hand-eye calibration matrix from existing captured images.
"""
import numpy as np
import cv2
import json
import sys
import argparse
from pathlib import Path
from typing import List, Tuple, Optional

# Add src directory to path FIRST
sys.path.append(str(Path(__file__).parent.parent.parent))

from config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS  # noqa: E402
from kinematics.kinematics_solver import InverseKinematicsSolver  # noqa: E402
from calibration.calibration_config import CalibrationConfig  # noqa: E402


def load_calibration_data(captures_dir: Path) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """
    Load calibration data from captured images.

    Returns:
        R_gripper2base_list: List of rotation matrices (gripper to base)
        t_gripper2base_list: List of translation vectors (gripper to base)
        R_target2cam_list: List of rotation matrices (target to camera)
        t_target2cam_list: List of translation vectors (target to camera)
    """
    print(f"📁 Loading calibration data from {captures_dir}")

    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    # Get all pose directories
    pose_dirs = sorted([d for d in captures_dir.iterdir()
                       if d.is_dir() and d.name.startswith('pose_')])

    print(f"Found {len(pose_dirs)} pose directories")

    config = CalibrationConfig()
    chessboard_size = config.chessboard_size
    square_size = config.square_size

    # Prepare object points (3D points of the checkerboard)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0],
                           0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    for i, pose_dir in enumerate(pose_dirs):
        print(f"Processing {pose_dir.name}...")

        # Load color image
        color_path = pose_dir / "color.png"
        if not color_path.exists():
            print(f"  ⚠️ No color image found in {pose_dir.name}")
            continue

        color_image = cv2.imread(str(color_path))
        if color_image is None:
            print(f"  ❌ Failed to load color image from {color_path}")
            continue

        # Load joint angles
        joints_path = pose_dir / "joint_angles.json"
        if not joints_path.exists():
            print(f"  ⚠️ No joint angles found in {pose_dir.name}")
            continue

        try:
            with open(joints_path, 'r') as f:
                joint_angles = np.array(json.load(f))
        except Exception as e:
            print(f"  ❌ Failed to load joint angles: {e}")
            continue

        # Load metadata
        metadata_path = pose_dir / "metadata.json"
        if not metadata_path.exists():
            print(f"  ⚠️ No metadata found in {pose_dir.name}")
            continue

        try:
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
        except Exception as e:
            print(f"  ❌ Failed to load metadata: {e}")
            continue

        # Find checkerboard corners
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if not ret:
            print(f"  ❌ Checkerboard not detected in {pose_dir.name}")
            continue

        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Solve PnP to get target pose in camera frame
        camera_matrix = np.array(
            [[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))

        ret, rvec, tvec = cv2.solvePnP(
            objp, corners, camera_matrix, dist_coeffs)

        if not ret:
            print(f"  ❌ PnP solve failed for {pose_dir.name}")
            continue

        # Convert rotation vector to rotation matrix
        R_target2cam, _ = cv2.Rodrigues(rvec)

        # Get gripper pose from joint angles
        kinematics_solver = InverseKinematicsSolver(
            URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)
        try:
            gripper_matrix, gripper_pose = kinematics_solver.tcp_from_joints(
                joint_angles)
            R_gripper2base = gripper_matrix[:3, :3]
            t_gripper2base = gripper_matrix[:3, 3]
        except Exception as e:
            print(f"  ❌ Failed to compute gripper pose: {e}")
            continue

        # Store the data
        R_gripper2base_list.append(R_gripper2base)
        t_gripper2base_list.append(t_gripper2base)
        R_target2cam_list.append(R_target2cam)
        t_target2cam_list.append(tvec.flatten())

        print(f"  ✅ Successfully processed {pose_dir.name}")

    print(f"📊 Loaded {len(R_gripper2base_list)} valid calibration poses")
    return R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list


def compute_hand_eye_calibration(R_gripper2base_list: List[np.ndarray],
                                 t_gripper2base_list: List[np.ndarray],
                                 R_target2cam_list: List[np.ndarray],
                                 t_target2cam_list: List[np.ndarray],
                                 method: int = cv2.CALIB_HAND_EYE_PARK) -> np.ndarray:
    """
    Compute hand-eye calibration matrix using OpenCV.

    Args:
        method: OpenCV calibration method (default: PARK)
    """
    print("🧮 Computing hand-eye calibration matrix...")

    # Convert lists to numpy arrays
    R_gripper2base = np.array(R_gripper2base_list)
    t_gripper2base = np.array(t_gripper2base_list)
    R_target2cam = np.array(R_target2cam_list)
    t_target2cam = np.array(t_target2cam_list)

    # Use OpenCV's calibrateHandEye function
    # This solves: R_gripper2base * R_cam2gripper = R_cam2gripper * R_target2cam
    # and: R_gripper2base * t_cam2gripper + t_gripper2base = R_cam2gripper * t_target2cam + t_cam2gripper

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=method
    )

    # Create homogeneous transformation matrix
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

    return T_cam2gripper


def analyze_calibration_quality(T_cam2gripper: np.ndarray,
                                R_gripper2base_list: List[np.ndarray],
                                t_gripper2base_list: List[np.ndarray],
                                R_target2cam_list: List[np.ndarray],
                                t_target2cam_list: List[np.ndarray]) -> None:
    """
    Analyze the quality of the hand-eye calibration.
    """
    print("\n📈 Analyzing calibration quality...")

    R_cam2gripper = T_cam2gripper[:3, :3]
    t_cam2gripper = T_cam2gripper[:3, 3]

    # Check rotation matrix properties
    det_R = np.linalg.det(R_cam2gripper)
    print(f"Determinant of rotation matrix: {det_R:.6f} (should be ±1)")

    orthogonality_error = np.linalg.norm(
        R_cam2gripper.T @ R_cam2gripper - np.eye(3))
    print(f"Orthogonality error: {orthogonality_error:.8f} (should be ~0)")

    # Compute reprojection errors
    reprojection_errors = []

    for i in range(len(R_gripper2base_list)):
        # Compute predicted gripper pose from camera pose
        R_pred = R_gripper2base_list[i] @ R_cam2gripper @ R_target2cam_list[i].T
        t_pred = R_gripper2base_list[i] @ t_cam2gripper + \
            t_gripper2base_list[i] - R_pred @ t_target2cam_list[i]

        # Compute error (simplified)
        R_error = np.linalg.norm(R_pred - np.eye(3))
        t_error = np.linalg.norm(t_pred)

        reprojection_errors.append(R_error + t_error)

    mean_error = np.mean(reprojection_errors)
    max_error = np.max(reprojection_errors)

    print(f"Mean calibration error: {mean_error:.6f}")
    print(f"Max calibration error: {max_error:.6f}")

    # Translation magnitude
    translation_magnitude = np.linalg.norm(t_cam2gripper)
    print(
        f"Camera-to-gripper translation magnitude: {translation_magnitude:.4f} m")

    print(f"\nHand-eye matrix (Camera to Gripper):")
    for i in range(4):
        print(f"  [{T_cam2gripper[i,0]:8.4f} {T_cam2gripper[i,1]:8.4f} {T_cam2gripper[i,2]:8.4f} {T_cam2gripper[i,3]:8.4f}]")


def main():
    """Main function to generate hand-eye matrix from existing images."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Generate hand-eye calibration matrix from captured images")
    parser.add_argument('--session', type=str, default=None,
                        help='Specific session directory to use (e.g., 20251011_013816)')
    parser.add_argument('--method', type=str, default='park',
                        choices=['tsai', 'park', 'horaud',
                                 'andreff', 'daniilidis'],
                        help='Hand-eye calibration method (default: park)')
    parser.add_argument('--best', type=int, default=None,
                        help='Use only the best N poses based on reprojection error')
    args = parser.parse_args()

    print("🎯 Hand-Eye Calibration Matrix Generation")
    print("=" * 50)
    if args.method:
        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS
        }
        calibration_method = method_map[args.method]
        print(f"📐 Using {args.method.upper()} method")
    else:
        calibration_method = cv2.CALIB_HAND_EYE_PARK
        print("📐 Using PARK method (default)")

    # Find the calibration captures directory
    captures_base = Path("calibration_captures")
    if not captures_base.exists():
        print("❌ No calibration_captures directory found!")
        return

    # Find all calibration sessions (timestamped directories and pose directories)
    session_dirs = []

    if args.session:
        # Use specific session
        session_path = captures_base / args.session
        if not session_path.exists():
            print(f"❌ Session directory not found: {args.session}")
            return
        session_dirs = [session_path]
        print(f"📁 Using specific session: {args.session}")
    else:
        # Add timestamped directories (new format)
        import re
        timestamped_dirs = [d for d in captures_base.iterdir(
        ) if d.is_dir() and re.match(r'^\d{8}_\d{6}$', d.name)]
        session_dirs.extend(timestamped_dirs)

        # Add legacy pose directories if they exist
        pose_dirs = [d for d in captures_base.iterdir() if d.is_dir()
                     and d.name.startswith('pose_')]
        if pose_dirs:
            # Use the captures_base directory directly for legacy poses
            session_dirs.append(captures_base)

        if not session_dirs:
            print("❌ No calibration sessions found!")
            return

        print(f"📁 Found {len(session_dirs)} calibration sessions:")
        for session in session_dirs:
            print(f"  - {session.name}")

    # Load calibration data from all sessions
    all_R_gripper2base = []
    all_t_gripper2base = []
    all_R_target2cam = []
    all_t_target2cam = []

    for session_dir in session_dirs:
        print(f"\n📂 Processing session: {session_dir.name}")
        try:
            R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list = load_calibration_data(
                session_dir)
            all_R_gripper2base.extend(R_gripper2base_list)
            all_t_gripper2base.extend(t_gripper2base_list)
            all_R_target2cam.extend(R_target2cam_list)
            all_t_target2cam.extend(t_target2cam_list)
            print(f"  ✅ Loaded {len(R_gripper2base_list)} poses")
        except Exception as e:
            print(f"  ❌ Failed to load session {session_dir.name}: {e}")

    if not all_R_gripper2base:
        print("❌ No valid poses loaded from any session!")
        return

    print(f"\n📊 Total poses loaded: {len(all_R_gripper2base)}")

    if len(all_R_gripper2base) < 3:
        print(
            f"❌ Not enough valid poses ({len(all_R_gripper2base)}). Need at least 3.")
        return

    # Prune to best poses if requested
    if args.best and args.best < len(all_R_gripper2base):
        print(
            f"\n🔍 Selecting best {args.best} poses based on reprojection error...")

        # First compute a preliminary calibration to evaluate pose quality
        T_preliminary = compute_hand_eye_calibration(
            all_R_gripper2base, all_t_gripper2base,
            all_R_target2cam, all_t_target2cam,
            method=calibration_method
        )

        R_cam2gripper = T_preliminary[:3, :3]
        t_cam2gripper = T_preliminary[:3, 3]

        # Compute reprojection error for each pose
        pose_errors = []
        for i in range(len(all_R_gripper2base)):
            R_pred = all_R_gripper2base[i] @ R_cam2gripper @ all_R_target2cam[i].T
            t_pred = all_R_gripper2base[i] @ t_cam2gripper + \
                all_t_gripper2base[i] - R_pred @ all_t_target2cam[i]

            R_error = np.linalg.norm(R_pred - np.eye(3))
            t_error = np.linalg.norm(t_pred)
            total_error = R_error + t_error
            pose_errors.append((i, total_error))

        # Sort by error and select best poses
        pose_errors.sort(key=lambda x: x[1])
        best_indices = [idx for idx, _ in pose_errors[:args.best]]

        print(f"  Selected poses: {best_indices}")
        print(
            f"  Error range: {pose_errors[0][1]:.6f} to {pose_errors[args.best-1][1]:.6f}")

        # Filter to best poses
        all_R_gripper2base = [all_R_gripper2base[i] for i in best_indices]
        all_t_gripper2base = [all_t_gripper2base[i] for i in best_indices]
        all_R_target2cam = [all_R_target2cam[i] for i in best_indices]
        all_t_target2cam = [all_t_target2cam[i] for i in best_indices]

        print(
            f"  Using {len(all_R_gripper2base)} best poses for final calibration")

    # Compute hand-eye calibration
    try:
        T_cam2gripper = compute_hand_eye_calibration(
            all_R_gripper2base, all_t_gripper2base,
            all_R_target2cam, all_t_target2cam,
            method=calibration_method
        )
    except Exception as e:
        print(f"❌ Failed to compute hand-eye calibration: {e}")
        return

    # Analyze quality
    analyze_calibration_quality(
        T_cam2gripper,
        all_R_gripper2base, all_t_gripper2base,
        all_R_target2cam, all_t_target2cam
    )

    # Save the matrix
    output_file = "hand_eye_matrix_generated.npy"
    np.save(output_file, T_cam2gripper)
    print(f"\n💾 Saved hand-eye matrix to: {output_file}")

    print("\n✅ Hand-eye calibration matrix generation completed!")


if __name__ == "__main__":
    main()
