#!/usr/bin/env python3
"""
Test script to demonstrate improved camera-centric pose generation.
"""
import numpy as np
import sys
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent))

from calibration.pose_generator import PoseGenerator
from calibration.calibration_config import CalibrationConfig


def test_pose_generation():
    """Test both legacy and improved pose generation methods."""
    print("🧪 Testing Pose Generation Methods")
    print("=" * 50)
    
    # Initialize pose generator
    config = CalibrationConfig()
    pose_generator = PoseGenerator(config)
    
    target_position = (0.4, 0.025, 0.22)
    num_poses = 15
    
    print(f"Target checkerboard position: {target_position}")
    print(f"Number of poses: {num_poses}")
    print()
    
    # Test 1: Legacy TCP-centric approach
    print("1️⃣ Testing Legacy TCP-Centric Approach")
    print("-" * 40)
    legacy_poses = pose_generator.generate_poses(
        num_poses, target_position, robot_base_z=0.202
    )
    print(f"Generated {len(legacy_poses)} legacy poses")
    
    # Analyze legacy poses
    if legacy_poses:
        positions = np.array([pose[:3, 3] for pose in legacy_poses])
        print(f"Position range - X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}]")
        print(f"Position range - Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
        print(f"Position range - Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}]")
    print()
    
    # Test 2: Try to load hand-eye matrix and use camera-centric approach
    print("2️⃣ Testing Camera-Centric Approach")
    print("-" * 40)
    
    # Try to load hand-eye matrix
    if pose_generator.load_hand_eye_matrix():
        camera_centric_poses = pose_generator.generate_camera_centric_poses(
            num_poses, target_position
        )
        print(f"Generated {len(camera_centric_poses)} camera-centric poses")
        
        # Analyze camera-centric poses
        if camera_centric_poses:
            positions = np.array([pose[:3, 3] for pose in camera_centric_poses])
            print(f"Position range - X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}]")
            print(f"Position range - Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
            print(f"Position range - Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}]")
            
            # Show first few poses for comparison
            print("\nFirst 3 camera-centric poses:")
            for i, pose in enumerate(camera_centric_poses[:3]):
                print(f"Pose {i+1}:")
                print(f"  Position: [{pose[0,3]:.3f}, {pose[1,3]:.3f}, {pose[2,3]:.3f}]")
                print(f"  Orientation (Z-axis): [{pose[0,2]:.3f}, {pose[1,2]:.3f}, {pose[2,2]:.3f}]")
    else:
        print("❌ No hand-eye matrix found - camera-centric approach not available")
    
    print()
    print("✅ Pose generation test completed!")


def analyze_hand_eye_matrix():
    """Analyze the hand-eye matrix if available."""
    print("🔍 Analyzing Hand-Eye Matrix")
    print("=" * 30)
    
    config = CalibrationConfig()
    matrix_path = config.hand_eye_matrix_file
    
    try:
        H = np.load(matrix_path)
        print(f"✅ Loaded hand-eye matrix from {matrix_path}")
        print(f"Matrix shape: {H.shape}")
        print()
        
        # Extract rotation and translation
        R_cam2tcp = H[:3, :3]
        t_cam2tcp = H[:3, 3]
        
        print("Rotation Matrix (Camera to TCP):")
        for i in range(3):
            print(f"  [{R_cam2tcp[i,0]:8.4f} {R_cam2tcp[i,1]:8.4f} {R_cam2tcp[i,2]:8.4f}]")
        
        print(f"\nTranslation Vector: [{t_cam2tcp[0]:8.4f} {t_cam2tcp[1]:8.4f} {t_cam2tcp[2]:8.4f}]")
        print(f"Translation magnitude: {np.linalg.norm(t_cam2tcp):.4f} m")
        
        # Check if rotation matrix is valid
        det = np.linalg.det(R_cam2tcp)
        print(f"Rotation matrix determinant: {det:.6f} (should be ±1)")
        
        # Check orthogonality
        should_be_identity = R_cam2tcp @ R_cam2tcp.T
        identity_error = np.linalg.norm(should_be_identity - np.eye(3))
        print(f"Orthogonality error: {identity_error:.8f} (should be ~0)")
        
    except FileNotFoundError:
        print(f"❌ Hand-eye matrix not found at {matrix_path}")
        print("Run calibration first to generate the matrix")
    except Exception as e:
        print(f"❌ Error loading hand-eye matrix: {e}")


if __name__ == "__main__":
    analyze_hand_eye_matrix()
    print()
    test_pose_generation()
