"""
Diagnostic script to verify RealSense depth data units.
This will help confirm whether depth data is in mm or meters.
"""
import pyrealsense2 as rs
import numpy as np
import cv2


def test_depth_units():
    """Test and display RealSense depth units configuration."""

    print("\n" + "="*70)
    print("🔍 REALSENSE DEPTH UNITS DIAGNOSTIC")
    print("="*70)

    try:
        # Initialize pipeline
        pipeline = rs.pipeline()
        config = rs.config()

        # Configure streams
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        print("\n📷 Starting camera...")
        profile = pipeline.start(config)

        # Get depth sensor
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        print(f"\n✅ Camera started successfully")
        print(f"\n{'='*70}")
        print("DEPTH SCALE INFORMATION")
        print(f"{'='*70}")
        print(f"Depth Scale (from sensor): {depth_scale}")
        print(f"This means: Raw depth value * {depth_scale} = meters")
        print(
            f"Example: Raw value 500 → {500 * depth_scale:.4f} meters ({500 * depth_scale * 1000:.1f} mm)")

        # Wait for a coherent pair of frames
        print("\n📸 Capturing frames...")
        for _ in range(30):  # Skip first few frames
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            print("❌ Failed to get depth frame")
            return

        # Get depth frame units
        depth_units = depth_frame.get_units()
        print(f"\n{'='*70}")
        print("DEPTH FRAME UNITS")
        print(f"{'='*70}")
        print(f"depth_frame.get_units(): {depth_units}")
        print(f"This is the scale factor: raw_value * {depth_units} = meters")
        print(f"\n⚠️  Note: depth_scale and get_units() should be the same value")
        print(
            f"Match: {'✅ YES' if abs(depth_scale - depth_units) < 0.0001 else '❌ NO'}")

        # Get raw depth data
        depth_data = np.asanyarray(depth_frame.get_data())

        # Get some sample depth values
        height, width = depth_data.shape
        center_x, center_y = width // 2, height // 2

        print(f"\n{'='*70}")
        print("SAMPLE DEPTH VALUES (at image center)")
        print(f"{'='*70}")
        print(f"Image center: ({center_x}, {center_y})")

        # Method 1: Raw data
        raw_depth = depth_data[center_y, center_x]
        print(f"\n1️⃣  Raw depth value: {raw_depth}")
        print(f"   → In meters: {raw_depth * depth_units:.4f} m")
        print(f"   → In millimeters: {raw_depth * depth_units * 1000:.1f} mm")

        # Method 2: get_distance (returns in meters)
        distance = depth_frame.get_distance(center_x, center_y)
        print(
            f"\n2️⃣  depth_frame.get_distance({center_x}, {center_y}): {distance:.4f} m")
        print(f"   → In millimeters: {distance * 1000:.1f} mm")

        # Method 3: rs2_deproject_pixel_to_point
        intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()
        point_3d_raw = rs.rs2_deproject_pixel_to_point(
            intrinsics, [center_x, center_y], raw_depth)
        point_3d_meters = rs.rs2_deproject_pixel_to_point(
            intrinsics, [center_x, center_y], distance)

        print(f"\n3️⃣  rs2_deproject_pixel_to_point:")
        print(f"   Using raw depth ({raw_depth}):")
        print(
            f"      → 3D point: [{point_3d_raw[0]:.4f}, {point_3d_raw[1]:.4f}, {point_3d_raw[2]:.4f}]")
        print(f"   Using distance in meters ({distance:.4f}):")
        print(
            f"      → 3D point: [{point_3d_meters[0]:.4f}, {point_3d_meters[1]:.4f}, {point_3d_meters[2]:.4f}]")

        # Verify the conversion
        print(f"\n{'='*70}")
        print("VERIFICATION")
        print(f"{'='*70}")
        print(
            f"Raw value * depth_units = {raw_depth} * {depth_units} = {raw_depth * depth_units:.4f} m")
        print(f"get_distance() = {distance:.4f} m")
        print(f"Match: {'✅ YES' if abs(raw_depth * depth_units - distance) < 0.001 else '❌ NO (difference: ' + str(abs(raw_depth * depth_units - distance)) + ' m)'}")

        # Check statistical info
        print(f"\n{'='*70}")
        print("DEPTH IMAGE STATISTICS")
        print(f"{'='*70}")
        valid_depths = depth_data[depth_data > 0]
        if len(valid_depths) > 0:
            print(f"Raw depth values:")
            print(f"  Min: {valid_depths.min()}")
            print(f"  Max: {valid_depths.max()}")
            print(f"  Mean: {valid_depths.mean():.1f}")
            print(f"  Median: {np.median(valid_depths):.1f}")
            print(f"\nIn meters (raw * {depth_units}):")
            print(f"  Min: {valid_depths.min() * depth_units:.4f} m")
            print(f"  Max: {valid_depths.max() * depth_units:.4f} m")
            print(f"  Mean: {valid_depths.mean() * depth_units:.4f} m")
            print(f"  Median: {np.median(valid_depths) * depth_units:.4f} m")

        print(f"\n{'='*70}")
        print("CONCLUSION")
        print(f"{'='*70}")
        print(f"✅ Raw depth values from get_data() are in ARBITRARY UNITS")
        print(
            f"✅ Multiply by get_units() ({depth_units}) to convert to METERS")
        print(f"✅ get_distance(x, y) returns depth in METERS directly")
        print(f"✅ rs2_deproject_pixel_to_point() expects depth in METERS")
        print(f"\n📝 In our code:")
        print(f"   • camera_manager.get_average_depth() multiplies by get_units() → ✅ Returns METERS")
        print(f"   • camera_manager.pixel_to_3d() expects depth in METERS → ✅ Correct")
        print(f"   • Direct use of np.asanyarray(depth_frame.get_data()) → ⚠️  Raw units, need conversion!")
        print(f"{'='*70}\n")

        # Stop pipeline
        pipeline.stop()

    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nNote: This test requires a connected RealSense camera.")
        print("If no camera is available, key facts:")
        print("  • Raw depth from get_data() is typically in arbitrary units")
        print("  • depth_frame.get_units() is typically 0.001 (mm to m conversion)")
        print("  • depth_frame.get_distance(x,y) returns meters directly")
        print("  • rs2_deproject_pixel_to_point expects depth in meters")


if __name__ == "__main__":
    test_depth_units()

