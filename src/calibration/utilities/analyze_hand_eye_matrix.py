#!/usr/bin/env python3
"""
Analyze the hand-eye matrix to understand the camera-TCP relationship.
"""
import numpy as np
import sys
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent))

from calibration.calibration_config import CalibrationConfig


def analyze_hand_eye_matrix():
    """Analyze the hand-eye matrix to understand camera-TCP relationship."""
    print("🔍 Analyzing Hand-Eye Matrix")
    print("=" * 40)
    
    config = CalibrationConfig()
    
    try:
        H = np.load(config.hand_eye_matrix_file)
        print(f"✅ Loaded hand-eye matrix from {config.hand_eye_matrix_file}")
        print(f"Matrix shape: {H.shape}")
        print()
        
        # Extract rotation and translation
        R_cam2tcp = H[:3, :3]
        t_cam2tcp = H[:3, 3]
        
        print("Hand-eye matrix:")
        print(H)
        print()
        
        print("Rotation matrix (Camera to TCP):")
        for i in range(3):
            print(f"  [{R_cam2tcp[i,0]:8.4f} {R_cam2tcp[i,1]:8.4f} {R_cam2tcp[i,2]:8.4f}]")
        
        print(f"\nTranslation vector: [{t_cam2tcp[0]:8.4f} {t_cam2tcp[1]:8.4f} {t_cam2tcp[2]:8.4f}]")
        print(f"Translation magnitude: {np.linalg.norm(t_cam2tcp):.4f} m")
        
        # Test some camera positions to see where TCP ends up
        print(f"\n🧪 Testing Camera Position → TCP Position Mapping")
        print("-" * 50)
        
        test_camera_positions = [
            [0.4, 0.0, 0.22],    # At target level
            [0.4, 0.0, 0.32],    # 10cm above target
            [0.4, 0.0, 0.42],    # 20cm above target
            [0.5, 0.0, 0.32],    # 10cm away, 10cm above
            [0.3, 0.0, 0.32],    # 10cm away (other side), 10cm above
        ]
        
        target_position = (0.4, 0.025, 0.22)
        
        for i, cam_pos in enumerate(test_camera_positions):
            # Create camera pose matrix (identity rotation for simplicity)
            camera_pose = np.eye(4)
            camera_pose[:3, 3] = cam_pos
            
            # Convert to TCP pose
            tcp_pose = camera_pose @ H
            tcp_pos = tcp_pose[:3, 3]
            
            print(f"Test {i+1}: Camera [{cam_pos[0]:.2f}, {cam_pos[1]:.2f}, {cam_pos[2]:.2f}]")
            print(f"        → TCP   [{tcp_pos[0]:.2f}, {tcp_pos[1]:.2f}, {tcp_pos[2]:.2f}]")
            
            # Check if this would be valid
            y_valid = tcp_pos[1] >= -0.5
            z_valid = 0.27 <= tcp_pos[2] <= 0.5
            valid = y_valid and z_valid
            print(f"        Valid: {valid} (Y: {y_valid}, Z: {z_valid})")
            print()
        
        # Try to find a good camera position
        print(f"🎯 Finding Good Camera Positions")
        print("-" * 35)
        
        # Try different camera positions around the target
        for cam_x in np.linspace(0.3, 0.5, 5):
            for cam_y in np.linspace(-0.1, 0.1, 3):
                for cam_z in np.linspace(0.15, 0.35, 5):
                    cam_pos = [cam_x, cam_y, cam_z]
                    
                    # Create camera pose matrix
                    camera_pose = np.eye(4)
                    camera_pose[:3, 3] = cam_pos
                    
                    # Convert to TCP pose
                    tcp_pose = camera_pose @ H
                    tcp_pos = tcp_pose[:3, 3]
                    
                    # Check validity
                    y_valid = tcp_pos[1] >= -0.5
                    z_valid = 0.27 <= tcp_pos[2] <= 0.5
                    
                    if y_valid and z_valid:
                        print(f"✅ Good camera position: [{cam_pos[0]:.2f}, {cam_pos[1]:.2f}, {cam_pos[2]:.2f}]")
                        print(f"   → TCP position: [{tcp_pos[0]:.2f}, {tcp_pos[1]:.2f}, {tcp_pos[2]:.2f}]")
                        break
                if y_valid and z_valid:
                    break
            if y_valid and z_valid:
                break
        
    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == "__main__":
    analyze_hand_eye_matrix()
