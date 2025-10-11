#!/usr/bin/env python3
"""
Extract full transformation matrices for all calibration poses.
"""
from calibration.calibration_config import CalibrationConfig
from kinematics.kinematics_solver import InverseKinematicsSolver
from config import URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
import numpy as np
import json
from pathlib import Path
import sys
from typing import List, Dict, Tuple
from scipy.spatial.transform import Rotation as R

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


def extract_all_pose_matrices():
    """Extract transformation matrices for all calibration poses."""
    print("🎯 Extracting All Pose Transformation Matrices")
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

    print(f"📊 Total poses to analyze: {len(all_pose_dirs)}")

    # Extract matrices for each pose
    pose_data = []

    for i, pose_dir in enumerate(all_pose_dirs):
        print(f"  Processing {pose_dir.name} ({i+1}/{len(all_pose_dirs)})...")

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

        # Get TCP pose
        try:
            tcp_matrix, tcp_pose = kinematics_solver.tcp_from_joints(
                joint_angles)
            tcp_position = tcp_matrix[:3, 3]
            tcp_rotation_matrix = tcp_matrix[:3, :3]

            # Convert to euler angles
            tcp_euler = R.from_matrix(
                tcp_rotation_matrix).as_euler('xyz', degrees=True)

            # Create pose data dictionary
            pose_info = {
                'pose_name': pose_dir.name,
                'session': pose_dir.parent.name,
                'joint_angles': joint_angles.tolist(),
                'tcp_matrix': tcp_matrix.tolist(),
                'tcp_position': tcp_position.tolist(),
                'tcp_rotation_matrix': tcp_rotation_matrix.tolist(),
                'tcp_euler_xyz': tcp_euler.tolist(),
                'tcp_pose_6dof': tcp_pose  # [x, y, z, roll, pitch, yaw]
            }

            pose_data.append(pose_info)
            print(f"    ✅ Successfully processed")

        except Exception as e:
            print(f"    ❌ Failed to process: {e}")
            continue

    print(f"\n📊 Successfully extracted {len(pose_data)} pose matrices")

    # Analyze pose diversity
    analyze_pose_diversity(pose_data)

    return pose_data


def analyze_pose_diversity(pose_data: List[Dict]):
    """Analyze the diversity of poses."""
    print(f"\n📈 Analyzing Pose Diversity")
    print("=" * 60)

    if not pose_data:
        print("❌ No pose data to analyze")
        return

    # Extract positions and orientations
    positions = np.array([pose['tcp_position'] for pose in pose_data])
    euler_angles = np.array([pose['tcp_euler_xyz'] for pose in pose_data])

    # Position analysis
    print(f"Position Analysis:")
    print(
        f"  X range: {np.min(positions[:, 0]):.3f} to {np.max(positions[:, 0]):.3f} m")
    print(
        f"  Y range: {np.min(positions[:, 1]):.3f} to {np.max(positions[:, 1]):.3f} m")
    print(
        f"  Z range: {np.min(positions[:, 2]):.3f} to {np.max(positions[:, 2]):.3f} m")

    # Orientation analysis (in degrees)
    print(f"\nOrientation Analysis (degrees):")
    print(
        f"  Roll range:  {np.min(euler_angles[:, 0]):.1f} to {np.max(euler_angles[:, 0]):.1f}")
    print(
        f"  Pitch range: {np.min(euler_angles[:, 1]):.1f} to {np.max(euler_angles[:, 1]):.1f}")
    print(
        f"  Yaw range:   {np.min(euler_angles[:, 2]):.1f} to {np.max(euler_angles[:, 2]):.1f}")

    # Calculate diversity metrics
    print(f"\nDiversity Metrics:")

    # Position diversity
    pos_std = np.std(positions, axis=0)
    print(
        f"  Position std dev: X={pos_std[0]:.3f}, Y={pos_std[1]:.3f}, Z={pos_std[2]:.3f}")

    # Orientation diversity
    euler_std = np.std(euler_angles, axis=0)
    print(
        f"  Orientation std dev: Roll={euler_std[0]:.1f}°, Pitch={euler_std[1]:.1f}°, Yaw={euler_std[2]:.1f}°")

    # Yaw diversity specifically
    yaw_values = euler_angles[:, 2]
    yaw_std = np.std(yaw_values)
    print(f"\n🎯 Yaw Analysis:")
    print(f"  Yaw values: {yaw_values}")
    print(f"  Yaw std dev: {yaw_std:.1f}°")
    print(
        f"  Yaw range: {np.min(yaw_values):.1f}° to {np.max(yaw_values):.1f}°")

    # Identify gaps in yaw coverage
    yaw_sorted = np.sort(yaw_values)
    yaw_gaps = []
    for i in range(len(yaw_sorted) - 1):
        gap = yaw_sorted[i+1] - yaw_sorted[i]
        yaw_gaps.append(gap)

    if yaw_gaps:
        max_gap = max(yaw_gaps)
        print(f"  Largest yaw gap: {max_gap:.1f}°")

        # Suggest yaw values to fill gaps
        if max_gap > 30:  # If gap is larger than 30 degrees
            print(f"  ⚠️ Large yaw gap detected - more diverse yaw orientations needed")


def save_pose_data(pose_data: List[Dict]):
    """Save pose data to JSON file."""
    output_file = "calibration_poses_analysis.json"

    # Convert numpy arrays to lists for JSON serialization
    json_data = []
    for pose in pose_data:
        json_pose = {
            'pose_name': pose['pose_name'],
            'session': pose['session'],
            'joint_angles': pose['joint_angles'],
            'tcp_matrix': pose['tcp_matrix'],
            'tcp_position': pose['tcp_position'],
            'tcp_rotation_matrix': pose['tcp_rotation_matrix'],
            'tcp_euler_xyz': pose['tcp_euler_xyz'],
            'tcp_pose_6dof': pose['tcp_pose_6dof']
        }
        json_data.append(json_pose)

    with open(output_file, 'w') as f:
        json.dump(json_data, f, indent=2)

    print(f"\n💾 Saved pose analysis to: {output_file}")
    return output_file


def main():
    """Main function."""
    pose_data = extract_all_pose_matrices()

    if pose_data:
        output_file = save_pose_data(pose_data)
        print(f"\n✅ Pose matrix extraction complete!")
        print(f"   Total poses analyzed: {len(pose_data)}")
        print(f"   Data saved to: {output_file}")
    else:
        print("\n❌ No pose data extracted")


if __name__ == "__main__":
    main()
