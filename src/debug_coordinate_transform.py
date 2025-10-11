"""
Debug script to trace coordinate transformations from camera to base frame.
This helps identify where sideways offsets are introduced.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from config import (
    HAND_EYE_MATRIX,
    HAND_EYE_MATRIX_CALIBRATED,
    HAND_EYE_MATRIX_SIMPLE,
    CAMERA_TRANSFORM_MODE,
    CAMERA_ROTATION_EULER,
    set_camera_transform_mode
)


def create_sample_tcp_matrix():
    """
    Create a sample TCP transformation matrix.
    Assume TCP is directly above table center, pointing down.
    """
    # TCP position: 0.5m forward, 0m left/right, 0.5m up
    # Orientation: Pointing down (-Z direction in base frame)
    position = np.array([0.5, 0.0, 0.5])

    # Standard "tool pointing down" orientation
    # This rotates the tool frame so Z points down
    rotation = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()

    tcp_matrix = np.eye(4)
    tcp_matrix[:3, :3] = rotation
    tcp_matrix[:3, 3] = position

    return tcp_matrix


def test_center_pixel_transform(camera_mode='simple', use_matrix=None):
    """
    Test what happens when we detect an object at the center of the camera frame.
    """
    print("\n" + "="*80)
    print(f"🔍 COORDINATE TRANSFORM DEBUG - MODE: {camera_mode.upper()}")
    print("="*80)

    # Simulate camera intrinsics (640x480 image)
    class FakeIntrinsics:
        width = 640
        height = 480
        ppx = 320.0  # Principal point X (image center)
        ppy = 240.0  # Principal point Y (image center)
        fx = 600.0   # Focal length X
        fy = 600.0   # Focal length Y

    intrinsics = FakeIntrinsics()

    # Test case: Object detected at image center
    pixel_u = 320  # Center X
    pixel_v = 240  # Center Y
    depth_m = 0.6  # 60cm away from camera

    print(f"\n📷 Input (Camera Image Space):")
    print(f"   Pixel coordinates: u={pixel_u}, v={pixel_v}")
    print(f"   Depth: {depth_m:.3f}m")
    print(f"   Image size: {intrinsics.width}x{intrinsics.height}")
    print(f"   Principal point: ({intrinsics.ppx}, {intrinsics.ppy})")

    # Step 1: Convert pixel to camera frame (standard pinhole projection)
    x_cam = (pixel_u - intrinsics.ppx) * depth_m / intrinsics.fx
    y_cam = (pixel_v - intrinsics.ppy) * depth_m / intrinsics.fy
    z_cam = depth_m

    print(f"\n1️⃣  Camera Frame Coordinates (from pixel projection):")
    print(
        f"   X_cam = ({pixel_u} - {intrinsics.ppx}) * {depth_m} / {intrinsics.fx} = {x_cam:.4f}m")
    print(
        f"   Y_cam = ({pixel_v} - {intrinsics.ppy}) * {depth_m} / {intrinsics.fy} = {y_cam:.4f}m")
    print(f"   Z_cam = {z_cam:.4f}m")
    print(
        f"   → Point in camera frame: [{x_cam:.4f}, {y_cam:.4f}, {z_cam:.4f}]")
    print(f"\n   Camera frame convention:")
    print(f"     X: Right (+u direction)")
    print(f"     Y: Down (+v direction)")
    print(f"     Z: Forward (into scene, depth)")

    # Step 2: Transform to TCP frame using hand-eye matrix
    # Use the matrix passed in, or get from config
    if use_matrix is None:
        if camera_mode == 'simple':
            current_matrix = HAND_EYE_MATRIX_SIMPLE
        else:
            current_matrix = HAND_EYE_MATRIX_CALIBRATED
    else:
        current_matrix = use_matrix

    camera_pos_homogeneous = np.array([x_cam, y_cam, z_cam, 1.0])
    tcp_pos_homogeneous = current_matrix @ camera_pos_homogeneous
    tcp_pos = tcp_pos_homogeneous[:3]

    print(f"\n2️⃣  TCP Frame Coordinates (via hand-eye matrix):")
    print(f"   Hand-eye matrix mode: {camera_mode}")
    print(f"   Rotation part:")
    for i in range(3):
        print(
            f"     [{current_matrix[i,0]:7.4f}, {current_matrix[i,1]:7.4f}, {current_matrix[i,2]:7.4f}]")
    print(
        f"   Translation: [{current_matrix[0,3]:.4f}, {current_matrix[1,3]:.4f}, {current_matrix[2,3]:.4f}]")
    print(
        f"   → Point in TCP frame: [{tcp_pos[0]:.4f}, {tcp_pos[1]:.4f}, {tcp_pos[2]:.4f}]")

    # Step 3: Transform to base frame using TCP pose
    tcp_matrix = create_sample_tcp_matrix()
    base_pos_homogeneous = tcp_matrix @ np.append(tcp_pos, 1.0)
    base_pos = base_pos_homogeneous[:3]

    print(f"\n3️⃣  Base Frame Coordinates (via TCP pose):")
    print(
        f"   TCP position in base: [{tcp_matrix[0,3]:.3f}, {tcp_matrix[1,3]:.3f}, {tcp_matrix[2,3]:.3f}]")
    print(
        f"   TCP rotation (first row): [{tcp_matrix[0,0]:7.4f}, {tcp_matrix[0,1]:7.4f}, {tcp_matrix[0,2]:7.4f}]")
    print(
        f"   → Point in base frame: [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}]")

    # Analysis
    print(f"\n" + "="*80)
    print("📊 ANALYSIS")
    print("="*80)

    print(f"\n✅ Expected behavior (object at center, camera pointing down):")
    print(f"   - Camera at center pixel → X_cam ≈ 0, Y_cam ≈ 0")
    print(f"   - Should map to point directly below TCP")
    print(f"   - Base X ≈ TCP X, Base Y ≈ TCP Y")

    offset_x = abs(base_pos[0] - tcp_matrix[0, 3])
    offset_y = abs(base_pos[1] - tcp_matrix[1, 3])

    print(f"\n📏 Actual offsets from TCP center:")
    print(f"   X offset: {offset_x:.4f}m ({offset_x*1000:.1f}mm)")
    print(f"   Y offset: {offset_y:.4f}m ({offset_y*1000:.1f}mm)")
    print(
        f"   Total lateral offset: {np.sqrt(offset_x**2 + offset_y**2)*1000:.1f}mm")

    if offset_x > 0.01 or offset_y > 0.01:
        print(f"\n⚠️  Lateral offset detected!")
        print(f"   This could explain sideways movement for center objects.")
        if camera_mode == 'simple':
            print(
                f"\n   Note: Simple mode now includes 180° rotation for camera pointing down.")
        else:
            print(f"\n   Calibrated mode uses full hand-eye matrix from calibration.")
    else:
        print(f"\n✅ Minimal offset - transformation looks correct!")

    print(f"\n" + "="*80)

    return {
        'camera': [x_cam, y_cam, z_cam],
        'tcp': tcp_pos.tolist(),
        'base': base_pos.tolist(),
        'tcp_position': tcp_matrix[:3, 3].tolist(),
        'offset_x': offset_x,
        'offset_y': offset_y
    }


def suggest_fix():
    """Suggest a corrected simple mode hand-eye matrix."""
    print("\n" + "="*80)
    print("🔧 SUGGESTED FIX FOR SIMPLE MODE")
    print("="*80)

    # Camera pointing down means 180° rotation around X axis
    rotation_cam_to_tcp = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()

    print(f"\nCamera physical orientation (from config):")
    print(f"  Roll (X): {CAMERA_ROTATION_EULER['roll']}°")
    print(f"  Pitch (Y): {CAMERA_ROTATION_EULER['pitch']}°")
    print(f"  Yaw (Z): {CAMERA_ROTATION_EULER['yaw']}°")

    print(f"\n💡 Corrected Simple Mode Hand-Eye Matrix:")
    print(f"   (Includes 180° rotation around X for camera pointing down)")

    corrected_matrix = np.eye(4, dtype=np.float32)
    corrected_matrix[:3, :3] = rotation_cam_to_tcp.astype(np.float32)

    print("\nHAND_EYE_MATRIX_SIMPLE = np.array([")
    for i in range(4):
        row_str = "    [" + \
            ", ".join(
                [f"{corrected_matrix[i,j]:7.4f}" for j in range(4)]) + "]"
        if i < 3:
            row_str += ","
        print(row_str)
    print("], dtype=np.float32)")

    print(f"\nThis matrix:")
    print(f"  • Rotates camera frame to align with TCP (camera looking down)")
    print(f"  • No translation offset (camera at TCP origin)")
    print(f"  • Should eliminate sideways movement for center objects")


if __name__ == "__main__":
    # Test both modes
    print("\n" + "="*80)
    print("🧪 TESTING COORDINATE TRANSFORMATIONS")
    print("="*80)

    print("\n\n" + "="*80)
    print("TEST 1: SIMPLE MODE (with 180° X rotation)")
    print("="*80)
    result_simple = test_center_pixel_transform(
        'simple', use_matrix=HAND_EYE_MATRIX_SIMPLE)

    print("\n\n" + "="*80)
    print("TEST 2: CALIBRATED MODE (from hand-eye calibration)")
    print("="*80)
    result_calibrated = test_center_pixel_transform(
        'calibrated', use_matrix=HAND_EYE_MATRIX_CALIBRATED)

    print("\n\n" + "="*80)
    print("COMPARISON")
    print("="*80)
    print(
        f"\nSimple mode offset: {result_simple['offset_x']*1000:.1f}mm X, {result_simple['offset_y']*1000:.1f}mm Y")
    print(
        f"Calibrated mode offset: {result_calibrated['offset_x']*1000:.1f}mm X, {result_calibrated['offset_y']*1000:.1f}mm Y")

    # Suggest fix
    suggest_fix()

    print("\n")
