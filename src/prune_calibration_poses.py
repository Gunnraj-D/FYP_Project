#!/usr/bin/env python3
"""
Prune calibration poses to remove the worst ones and generate a high-quality hand-eye matrix.
"""
from calibration.calibration_config import CalibrationConfig
from kinematics.kinematics_solver import InverseKinematicsSolver
from config.config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
import numpy as np
import cv2
import json
from pathlib import Path
import sys
from typing import List, Tuple, Dict
from scipy.spatial.transform import Rotation as R

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


def analyze_pose_quality(pose_dir: Path, config: CalibrationConfig, kinematics_solver: InverseKinematicsSolver) -> Dict:
    """
    Analyze the quality of a single calibration pose.

    Returns:
        Dictionary with quality metrics
    """
    quality_metrics = {
        'pose_dir': pose_dir.name,
        'valid': False,
        'reprojection_error': float('inf'),
        'corner_quality': 0.0,
        'pose_diversity': 0.0,
        'workspace_fitness': 0.0,
        'overall_score': 0.0
    }

    try:
        # Load joint angles
        joints_path = pose_dir / "joint_angles.json"
        if not joints_path.exists():
            return quality_metrics

        with open(joints_path, 'r') as f:
            joint_angles = np.array(json.load(f))

        # Get TCP pose
        try:
            tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(
                joint_angles)
            tcp_pos = tcp_matrix[:3, 3]
            tcp_rot = tcp_matrix[:3, :3]
        except Exception as e:
            print(f"  ❌ Failed to compute TCP pose for {pose_dir.name}: {e}")
            return quality_metrics

        # Load and analyze image
        color_path = pose_dir / "color.png"
        if not color_path.exists():
            return quality_metrics

        color_image = cv2.imread(str(color_path))
        if color_image is None:
            return quality_metrics

        # Find checkerboard
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        chessboard_size = config.chessboard_size
        square_size = config.square_size

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if not ret:
            return quality_metrics

        # Refine corners
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Prepare object points
        objp = np.zeros(
            (chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_size[0],
                               0:chessboard_size[1]].T.reshape(-1, 2)
        objp *= square_size

        # Camera matrix (approximate)
        camera_matrix = np.array(
            [[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))

        # Solve PnP
        ret, rvec, tvec = cv2.solvePnP(
            objp, corners, camera_matrix, dist_coeffs)
        if not ret:
            return quality_metrics

        # Calculate reprojection error
        projected_points, _ = cv2.projectPoints(
            objp, rvec, tvec, camera_matrix, dist_coeffs)
        reprojection_error = cv2.norm(
            corners, projected_points, cv2.NORM_L2) / len(corners)

        # Calculate corner quality (how well the corners are detected)
        corner_quality = 1.0 / (1.0 + reprojection_error)  # Higher is better

        # Calculate pose diversity (how different this pose is from others)
        # This will be calculated later when we have all poses

        # Calculate workspace fitness
        # Check if TCP is in a good position for calibration
        workspace_fitness = 1.0
        if tcp_pos[2] < 0.2 or tcp_pos[2] > 0.8:  # Too low or too high
            workspace_fitness *= 0.5
        if tcp_pos[1] < -0.5 or tcp_pos[1] > 0.5:  # Too far from center
            workspace_fitness *= 0.7
        if tcp_pos[0] < 0.0 or tcp_pos[0] > 0.8:  # Outside reasonable X range
            workspace_fitness *= 0.8

        quality_metrics.update({
            'valid': True,
            'reprojection_error': reprojection_error,
            'corner_quality': corner_quality,
            'workspace_fitness': workspace_fitness,
            'tcp_position': tcp_pos.tolist(),
            'tcp_rotation': tcp_rot.tolist(),
            'target_position': tvec.flatten().tolist(),
            'target_rotation': rvec.flatten().tolist()
        })

    except Exception as e:
        print(f"  ❌ Error analyzing {pose_dir.name}: {e}")

    return quality_metrics


def calculate_pose_diversity(quality_metrics_list: List[Dict]) -> List[Dict]:
    """
    Calculate pose diversity scores for all poses.
    """
    valid_poses = [qm for qm in quality_metrics_list if qm['valid']]

    if len(valid_poses) < 2:
        return quality_metrics_list

    # Extract TCP positions and rotations
    tcp_positions = np.array([qm['tcp_position'] for qm in valid_poses])
    tcp_rotations = np.array([qm['tcp_rotation'] for qm in valid_poses])

    # Calculate diversity for each pose
    for i, qm in enumerate(valid_poses):
        diversity_scores = []

        for j, other_qm in enumerate(valid_poses):
            if i == j:
                continue

            # Position diversity (distance)
            pos_diff = np.linalg.norm(
                np.array(qm['tcp_position']) - np.array(other_qm['tcp_position']))

            # Rotation diversity (angle between rotation matrices)
            rot_diff = np.arccos(np.clip(np.trace(
                qm['tcp_rotation'] @ np.array(other_qm['tcp_rotation']).T) / 2 - 0.5, -1, 1))

            # Combined diversity score
            diversity_score = pos_diff + rot_diff
            diversity_scores.append(diversity_score)

        # Average diversity score (higher is better - more different from others)
        avg_diversity = np.mean(diversity_scores) if diversity_scores else 0.0
        qm['pose_diversity'] = avg_diversity

    return quality_metrics_list


def calculate_overall_scores(quality_metrics_list: List[Dict]) -> List[Dict]:
    """
    Calculate overall quality scores for pose selection.
    """
    valid_poses = [qm for qm in quality_metrics_list if qm['valid']]

    if not valid_poses:
        return quality_metrics_list

    # Normalize scores to 0-1 range
    reprojection_errors = [qm['reprojection_error'] for qm in valid_poses]
    corner_qualities = [qm['corner_quality'] for qm in valid_poses]
    diversity_scores = [qm['pose_diversity'] for qm in valid_poses]
    workspace_fitnesses = [qm['workspace_fitness'] for qm in valid_poses]

    # Normalize reprojection error (lower is better)
    min_reproj = min(reprojection_errors)
    max_reproj = max(reprojection_errors)
    if max_reproj > min_reproj:
        for qm in valid_poses:
            qm['reprojection_score'] = 1.0 - \
                (qm['reprojection_error'] - min_reproj) / \
                (max_reproj - min_reproj)
    else:
        for qm in valid_poses:
            qm['reprojection_score'] = 1.0

    # Normalize diversity (higher is better)
    min_diversity = min(diversity_scores)
    max_diversity = max(diversity_scores)
    if max_diversity > min_diversity:
        for qm in valid_poses:
            qm['diversity_score'] = (
                qm['pose_diversity'] - min_diversity) / (max_diversity - min_diversity)
    else:
        for qm in valid_poses:
            qm['diversity_score'] = 1.0

    # Calculate overall score (weighted combination)
    weights = {
        'reprojection': 0.4,    # Most important - accuracy
        'corner_quality': 0.2,  # Detection quality
        'diversity': 0.2,       # Pose variety
        'workspace': 0.2        # Workspace fitness
    }

    for qm in valid_poses:
        overall_score = (
            weights['reprojection'] * qm['reprojection_score'] +
            weights['corner_quality'] * qm['corner_quality'] +
            weights['diversity'] * qm['diversity_score'] +
            weights['workspace'] * qm['workspace_fitness']
        )
        qm['overall_score'] = overall_score

    return quality_metrics_list


def prune_poses():
    """Main function to prune calibration poses and generate improved matrix."""
    print("🔍 Analyzing and Pruning Calibration Poses")
    print("=" * 60)

    # Initialize components
    config = CalibrationConfig()
    kinematics_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    # Find all calibration sessions
    captures_base = Path("calibration_captures")
    if not captures_base.exists():
        print("❌ No calibration_captures directory found!")
        return

    # Collect all pose directories
    pose_dirs = []

    # Add timestamped sessions
    timestamped_dirs = [d for d in captures_base.iterdir(
    ) if d.is_dir() and d.name.startswith('2025')]
    for session_dir in timestamped_dirs:
        session_poses = [d for d in session_dir.iterdir(
        ) if d.is_dir() and d.name.startswith('pose_')]
        pose_dirs.extend(session_poses)
        print(
            f"📂 Found {len(session_poses)} poses in session {session_dir.name}")

    # Add legacy poses
    legacy_poses = [d for d in captures_base.iterdir(
    ) if d.is_dir() and d.name.startswith('pose_')]
    pose_dirs.extend(legacy_poses)
    print(f"📂 Found {len(legacy_poses)} legacy poses")

    print(f"📊 Total poses to analyze: {len(pose_dirs)}")

    # Analyze each pose
    print(f"\n🔍 Analyzing pose quality...")
    quality_metrics_list = []

    for i, pose_dir in enumerate(pose_dirs):
        print(f"  Analyzing {pose_dir.name} ({i+1}/{len(pose_dirs)})...")
        quality_metrics = analyze_pose_quality(
            pose_dir, config, kinematics_solver)
        quality_metrics_list.append(quality_metrics)

        if quality_metrics['valid']:
            print(
                f"    ✅ Valid - Reprojection error: {quality_metrics['reprojection_error']:.4f}")
        else:
            print(f"    ❌ Invalid")

    # Calculate diversity scores
    print(f"\n📊 Calculating pose diversity...")
    quality_metrics_list = calculate_pose_diversity(quality_metrics_list)

    # Calculate overall scores
    print(f"📊 Calculating overall quality scores...")
    quality_metrics_list = calculate_overall_scores(quality_metrics_list)

    # Sort by overall score
    valid_poses = [qm for qm in quality_metrics_list if qm['valid']]
    valid_poses.sort(key=lambda x: x['overall_score'], reverse=True)

    print(f"\n📈 Pose Quality Analysis Results")
    print("=" * 60)
    print(f"Valid poses: {len(valid_poses)}/{len(pose_dirs)}")

    if not valid_poses:
        print("❌ No valid poses found!")
        return

    # Display top poses
    print(f"\n🏆 Top 10 poses by overall score:")
    print(f"{'Rank':<4} {'Pose':<12} {'Overall':<8} {'Reproj':<8} {'Corner':<8} {'Diversity':<10} {'Workspace':<10}")
    print("-" * 70)

    for i, qm in enumerate(valid_poses[:10]):
        print(f"{i+1:<4} {qm['pose_dir']:<12} {qm['overall_score']:<8.3f} "
              f"{qm['reprojection_error']:<8.4f} {qm['corner_quality']:<8.3f} "
              f"{qm['pose_diversity']:<10.3f} {qm['workspace_fitness']:<10.3f}")

    # Select best poses (keep top 80% or minimum 8 poses)
    num_to_keep = max(8, int(len(valid_poses) * 0.8))
    best_poses = valid_poses[:num_to_keep]

    print(f"\n✂️ Pruning Strategy:")
    print(f"  Original poses: {len(pose_dirs)}")
    print(f"  Valid poses: {len(valid_poses)}")
    print(f"  Selected poses: {len(best_poses)}")
    print(f"  Removed poses: {len(valid_poses) - len(best_poses)}")

    # Show which poses were removed
    removed_poses = valid_poses[len(best_poses):]
    if removed_poses:
        print(f"\n🗑️ Removed poses (worst quality):")
        for qm in removed_poses:
            print(f"  - {qm['pose_dir']} (score: {qm['overall_score']:.3f})")

    print(f"\n✅ Selected poses (best quality):")
    for qm in best_poses:
        print(f"  - {qm['pose_dir']} (score: {qm['overall_score']:.3f})")

    # Generate new matrix from selected poses
    print(
        f"\n🧮 Generating improved hand-eye matrix from {len(best_poses)} poses...")
    generate_improved_matrix(best_poses, config, kinematics_solver)


def generate_improved_matrix(selected_poses: List[Dict], config: CalibrationConfig, kinematics_solver: InverseKinematicsSolver):
    """Generate hand-eye matrix from selected high-quality poses."""
    print(f"🎯 Generating Hand-Eye Matrix from Pruned Poses")
    print("=" * 60)

    # Load calibration data from selected poses
    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    captures_base = Path("calibration_captures")
    chessboard_size = config.chessboard_size
    square_size = config.square_size

    # Prepare object points
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0],
                           0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    camera_matrix = np.array(
        [[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1))

    for i, pose_metrics in enumerate(selected_poses):
        print(
            f"  Processing {pose_metrics['pose_dir']} ({i+1}/{len(selected_poses)})...")

        # Find the pose directory - we need to match both the pose name and session
        pose_dir = None

        # The pose_metrics should contain the full path information
        # Let's reconstruct the pose directory path from the original pose_dirs
        for original_pose_dir in pose_dirs:
            if original_pose_dir.name == pose_metrics['pose_dir']:
                pose_dir = original_pose_dir
                break

        if pose_dir is None:
            print(
                f"    ⚠️ Could not find pose directory for {pose_metrics['pose_dir']}")
            continue

        # Load joint angles
        joints_path = pose_dir / "joint_angles.json"
        with open(joints_path, 'r') as f:
            joint_angles = np.array(json.load(f))

        # Get gripper pose
        tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(joint_angles)
        R_gripper2base = tcp_matrix[:3, :3]
        t_gripper2base = tcp_matrix[:3, 3]

        # Load image and detect checkerboard
        color_path = pose_dir / "color.png"
        color_image = cv2.imread(str(color_path))
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Solve PnP
        ret, rvec, tvec = cv2.solvePnP(
            objp, corners, camera_matrix, dist_coeffs)
        R_target2cam, _ = cv2.Rodrigues(rvec)
        t_target2cam = tvec.flatten()

        # Store data
        R_gripper2base_list.append(R_gripper2base)
        t_gripper2base_list.append(t_gripper2base)
        R_target2cam_list.append(R_target2cam)
        t_target2cam_list.append(t_target2cam)

        print(f"    ✅ Added to calibration data")

    if len(R_gripper2base_list) < 3:
        print(
            f"❌ Not enough valid poses ({len(R_gripper2base_list)}). Need at least 3.")
        return

    # Compute hand-eye calibration
    print(
        f"\n🧮 Computing hand-eye calibration from {len(R_gripper2base_list)} poses...")
    try:
        # Debug: Check shapes before conversion
        print(f"  Debug - Number of poses: {len(R_gripper2base_list)}")
        print(
            f"  Debug - R_gripper2base_list length: {len(R_gripper2base_list)}")
        print(
            f"  Debug - t_gripper2base_list length: {len(t_gripper2base_list)}")
        if len(t_gripper2base_list) > 0:
            print(
                f"  Debug - First t_gripper2base shape: {np.array(t_gripper2base_list[0]).shape}")

        # Convert lists to numpy arrays for OpenCV
        R_gripper2base_array = np.array(R_gripper2base_list)
        t_gripper2base_array = np.array(t_gripper2base_list).reshape(-1, 3, 1)
        R_target2cam_array = np.array(R_target2cam_list)
        t_target2cam_array = np.array(t_target2cam_list).reshape(-1, 3, 1)

        print(f"  Debug - Final array shapes:")
        print(f"    R_gripper2base_array: {R_gripper2base_array.shape}")
        print(f"    t_gripper2base_array: {t_gripper2base_array.shape}")
        print(f"    R_target2cam_array: {R_target2cam_array.shape}")
        print(f"    t_target2cam_array: {t_target2cam_array.shape}")

        T_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base_array, t_gripper2base_array,
            R_target2cam_array, t_target2cam_array,
            method=cv2.CALIB_HAND_EYE_TSAI
        )[0]

        # Analyze quality
        translation_magnitude = np.linalg.norm(T_cam2gripper[:3, 3])
        rotation_det = np.linalg.det(T_cam2gripper[:3, :3])

        print(f"\n📊 Improved Matrix Quality:")
        print(f"  Translation magnitude: {translation_magnitude:.3f}m")
        print(f"  Rotation determinant: {rotation_det:.6f}")
        print(f"  Number of poses used: {len(R_gripper2base_list)}")

        # Save improved matrix
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
        print(
            f"   Use the matrix in {output_file} to replace the current one in config.py")

    except Exception as e:
        print(f"❌ Failed to compute hand-eye calibration: {e}")


if __name__ == "__main__":
    prune_poses()
