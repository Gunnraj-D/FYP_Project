"""
Camera management module for robot hand tracking system.
Provides centralized camera initialization and frame capture functionality.
"""
import cv2
import pyrealsense2 as rs
import numpy as np
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    """Camera configuration parameters."""
    color_width: int = 640
    color_height: int = 480
    color_fps: int = 30
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30


class CameraManager:
    """Centralized camera management for RealSense camera."""

    def __init__(self, config: CameraConfig = None):
        self.config = config or CameraConfig()
        self.pipeline: Optional[rs.pipeline] = None
        self.align: Optional[rs.align] = None

        # Intrinsics for different streams
        # NOTE: After alignment, use aligned_color_intrinsics for deprojection
        # Color stream intrinsics
        self.color_intrinsics: Optional[rs.intrinsics] = None
        # Raw depth stream intrinsics
        self.depth_intrinsics: Optional[rs.intrinsics] = None
        # Intrinsics for aligned depth (same as color)
        self.aligned_color_intrinsics: Optional[rs.intrinsics] = None
        self.is_initialized = False

        # Cache for depth frame data to avoid repeated conversions
        self._depth_cache_frame_id: Optional[int] = None
        self._depth_cache_array: Optional[np.ndarray] = None
        self._depth_cache_scale: Optional[float] = None
        self._depth_cache_dims: Optional[Tuple[int, int]] = None

        # Depth scale (units per meter) - cached from sensor
        self._depth_scale: Optional[float] = None

    def initialize(self) -> bool:
        """Initialize the RealSense camera."""
        try:
            self.pipeline = rs.pipeline()
            cfg = rs.config()

            # Configure color stream
            cfg.enable_stream(
                rs.stream.color,
                self.config.color_width,
                self.config.color_height,
                rs.format.bgr8,
                self.config.color_fps
            )

            # Configure depth stream
            cfg.enable_stream(
                rs.stream.depth,
                self.config.depth_width,
                self.config.depth_height,
                rs.format.z16,
                self.config.depth_fps
            )

            # Start pipeline
            profile = self.pipeline.start(cfg)

            # Get intrinsics for coordinate transformation
            color_stream = profile.get_stream(rs.stream.color)
            depth_stream = profile.get_stream(rs.stream.depth)

            self.color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            self.depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

            # Validate that returned profiles match requested configuration
            if self.color_intrinsics.width != self.config.color_width or \
               self.color_intrinsics.height != self.config.color_height:
                logger.warning(
                    f"Color profile mismatch: requested {self.config.color_width}x{self.config.color_height}, "
                    f"got {self.color_intrinsics.width}x{self.color_intrinsics.height}")

            if self.depth_intrinsics.width != self.config.depth_width or \
               self.depth_intrinsics.height != self.config.depth_height:
                logger.warning(
                    f"Depth profile mismatch: requested {self.config.depth_width}x{self.config.depth_height}, "
                    f"got {self.depth_intrinsics.width}x{self.depth_intrinsics.height}")

            # After depth-to-color alignment, aligned depth has same intrinsics as color
            self.aligned_color_intrinsics = self.color_intrinsics

            # Create align object for depth-to-color alignment
            self.align = rs.align(rs.stream.color)

            # Cache depth scale from sensor
            depth_sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = depth_sensor.get_depth_scale()
            logger.info(f"Depth scale: {self._depth_scale} (meters per unit)")

            self.is_initialized = True
            logger.info("Camera initialized successfully")
            logger.info(
                f"Color stream: {self.color_intrinsics.width}x{self.color_intrinsics.height} @ {self.config.color_fps}fps, "
                f"fx={self.color_intrinsics.fx:.1f}, fy={self.color_intrinsics.fy:.1f}")
            logger.info(
                f"Depth stream: {self.depth_intrinsics.width}x{self.depth_intrinsics.height} @ {self.config.depth_fps}fps, "
                f"fx={self.depth_intrinsics.fx:.1f}, fy={self.depth_intrinsics.fy:.1f}")
            logger.info(
                f"Aligned depth uses color intrinsics for deprojection")
            return True

        except Exception as e:
            logger.error(f"Camera initialization failed: {e}")
            self.is_initialized = False
            return False

    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[rs.depth_frame]]:
        """
        Get aligned color and depth frames.

        Returns:
            Tuple of (color_array, depth_frame) where:
            - color_array: numpy array of color image (BGR format)
            - depth_frame: RealSense depth_frame object (aligned to color)

        Note: Depth frame data is cached internally to optimize performance when
        get_average_depth() is called multiple times with the same frame.
        """
        if not self.is_initialized or not self.pipeline:
            logger.warning("Camera not initialized")
            return None, None

        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None, None

            color_array = np.asanyarray(color_frame.get_data())
            return color_array, depth_frame

        except Exception as e:
            logger.error(f"Frame capture error: {e}")
            return None, None

    def pixel_to_3d(self, x: int, y: int, depth: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to 3D vector in meters.

        IMPORTANT: This method must be used with depth-to-color aligned frames:
        - Pixel coordinates (x, y) must be from the COLOR frame
        - Depth value must be from the ALIGNED depth frame at the same pixel
        - Depth must be in METERS (e.g., from get_average_depth())
        - Uses aligned_color_intrinsics which is correct for aligned depth

        DISTORTION HANDLING:
        - Uses RealSense SDK rs2_deproject_pixel_to_point() which handles lens
          distortion automatically using the Brown-Conrady model if distortion
          coefficients are present in the intrinsics
        - For RealSense color streams, distortion is typically minimal but is
          properly handled by the SDK function
        - This is more robust than manual pinhole projection formulas

        Args:
            x: Pixel x-coordinate in color frame
            y: Pixel y-coordinate in color frame
            depth: Depth value in METERS at pixel (x, y) in aligned depth frame

        Returns:
            (x, y, z) coordinates in camera frame (meters)
        """
        if not self.aligned_color_intrinsics or depth <= 0:
            return (0.0, 0.0, 0.0)

        # Sanity check: typical valid depth range for RealSense is 0.1m to 10m
        if depth > 100.0:
            logger.warning(
                f"Depth value {depth:.3f}m seems unusually large. "
                f"Ensure depth is in METERS, not raw units or millimeters."
            )

        try:
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.aligned_color_intrinsics, [x, y], depth
            )
            # Return coordinates in meters (RealSense already provides meters)
            return (point_3d[0], point_3d[1], point_3d[2])
        except Exception as e:
            logger.error(f"3D conversion error: {e}")
            return (0.0, 0.0, 0.0)

    def get_depth_scale(self) -> float:
        """
        Get depth scale (meters per unit) from the sensor.

        Returns:
            Depth scale factor (typically 0.001 for mm to m conversion), or 0.001 if not available
        """
        if self._depth_scale is not None:
            return self._depth_scale
        logger.warning("Depth scale not available, using default 0.001")
        return 0.001

    def get_depth_image_meters(self, depth_frame: rs.depth_frame) -> Optional[np.ndarray]:
        """
        Convert depth frame to float32 numpy array in meters.

        This is a convenience method that handles the conversion from raw depth units
        to meters in one step, reducing repeated scaling and unit mistakes.

        Args:
            depth_frame: RealSense depth frame (aligned to color)

        Returns:
            Depth image as float32 array in meters, or None if conversion fails
        """
        try:
            depth_array_raw = np.asanyarray(depth_frame.get_data())
            depth_scale = depth_frame.get_units()
            depth_array_meters = depth_array_raw.astype(
                np.float32) * depth_scale
            return depth_array_meters
        except Exception as e:
            logger.error(f"Failed to convert depth frame to meters: {e}")
            return None

    def get_average_depth(self, depth_frame: rs.depth_frame,
                          center: Tuple[int, int], radius: int,
                          min_valid_pixels: int = 5,
                          method: str = 'median',
                          percentile_low: float = 15.0) -> Tuple[Optional[float], dict]:
        """
        Get depth statistics in circular ROI.

        Returns:
            (median_depth_m, quality) where quality is dict with:
                - valid_ratio: fraction of ROI pixels with valid depth
                - depth_std_m: std-dev of valid depths (meters)
                - valid_count: number of valid pixels
                - median_depth_m: median depth (meters) or None if no valid pixels
        Notes:
            - Robust to invalid (0) depth values.
            - Uses percentile clipping (25-75) to ignore extreme tails within ROI.
        """
        try:
            # frame shape and units
            # assumed uint16 or similar
            depth_array = np.asanyarray(depth_frame.get_data())
            h, w = depth_array.shape
            depth_scale = depth_frame.get_units()

            # build circular mask
            mask = np.zeros((h, w), dtype=np.uint8)
            cx, cy = int(center[0]), int(center[1])
            cv2.circle(mask, (cx, cy), int(radius), 255, -1)

            # Extract ROI depths (vectorized)
            roi_depths = depth_array[mask == 255]

            # Count and filter valid depths (>0)
            valid_depths = roi_depths[roi_depths > 0]
            valid_count = int(valid_depths.size)
            total_count = int(roi_depths.size) if roi_depths.size > 0 else 0
            valid_ratio = (
                valid_count / total_count) if total_count > 0 else 0.0

            if valid_count == 0:
                # No valid depth in ROI
                quality = {
                    "valid_ratio": valid_ratio,
                    "depth_std_m": None,
                    "valid_count": 0,
                    "median_depth_m": None
                }
                return None, quality

            # convert to meters
            valid_depths_m = valid_depths.astype(np.float32) * depth_scale

            # percentile clipping to remove tails (robust to edges)
            if valid_count >= 10:
                lower = np.percentile(valid_depths_m, 25)
                upper = np.percentile(valid_depths_m, 75)
                clipped = valid_depths_m[(valid_depths_m >= lower) & (
                    valid_depths_m <= upper)]
                if clipped.size >= max(5, int(0.5 * valid_count)):
                    valid_depths_m = clipped

            if method == 'min':
                median_depth_m = float(np.min(valid_depths_m))
            elif method in ('p_low', 'percentile_low'):
                p = float(percentile_low)
                p = np.clip(p, 0.0, 50.0)
                median_depth_m = float(np.percentile(valid_depths_m, p))
            else:
                median_depth_m = float(np.median(valid_depths_m))

            depth_std_m = float(np.std(valid_depths_m))

            quality = {
                "valid_ratio": float(valid_ratio),
                "depth_std_m": float(depth_std_m),
                "valid_count": valid_count,
                "median_depth_m": median_depth_m
            }

            return median_depth_m, quality

        except Exception as e:
            logger.error(f"Depth calculation error: {e}")
            quality = {
                "valid_ratio": 0.0,
                "depth_std_m": None,
                "valid_count": 0,
                "median_depth_m": None
            }
            return None, quality

    def cleanup(self):
        """Clean up camera resources."""
        if self.pipeline and self.is_initialized:
            try:
                self.pipeline.stop()
            except Exception as e:
                logger.debug(f"Camera pipeline stop error (ignored): {e}")
        self.pipeline = None
        self.align = None
        self.color_intrinsics = None
        self.depth_intrinsics = None
        self.aligned_color_intrinsics = None
        # Clear depth cache
        self._depth_cache_frame_id = None
        self._depth_cache_array = None
        self._depth_cache_scale = None
        self._depth_cache_dims = None
        self.is_initialized = False
        logger.info("Camera cleanup complete")

    def compute_pixel_to_mm_at_depth(self, depth_m: float, image_width: int = None,
                                     image_height: int = None) -> Tuple[float, float]:
        """
        Compute pixel-to-mm conversion ratio at given depth using camera intrinsics.

        This is more accurate than FOV-based estimation because it uses the actual
        calibrated focal lengths from the camera.

        For a pinhole camera model:
            X = (u - ppx) * Z / fx
            Y = (v - ppy) * Z / fy

        Where (u,v) are pixel coords, (X,Y,Z) are 3D coords in meters, and
        (fx,fy) are focal lengths in pixels, (ppx,ppy) is principal point.

        The pixel-to-mm ratio at depth Z is: (Z / fx) * 1000 for horizontal,
        (Z / fy) * 1000 for vertical.

        If image dimensions are provided (e.g., for resized images), scales the
        focal lengths appropriately.

        Args:
            depth_m: Distance to object in meters
            image_width: Target image width (if resized from camera resolution)
            image_height: Target image height (if resized from camera resolution)

        Returns:
            Tuple of (mm_per_pixel_horizontal, mm_per_pixel_vertical)
        """
        if not self.aligned_color_intrinsics or depth_m <= 0:
            logger.warning(
                "Intrinsics not available or invalid depth, using fallback")
            # Fallback to approximate value
            return (0.8, 0.8)

        intrinsics = self.aligned_color_intrinsics
        fx = intrinsics.fx
        fy = intrinsics.fy

        # If target image dimensions are provided, scale focal lengths
        if image_width is not None and image_height is not None:
            scale_x = image_width / intrinsics.width
            scale_y = image_height / intrinsics.height
            fx = fx * scale_x
            fy = fy * scale_y
            logger.debug(
                f"Scaled intrinsics: fx={fx:.1f}, fy={fy:.1f} "
                f"for {image_width}x{image_height} (from {intrinsics.width}x{intrinsics.height})"
            )

        # Compute mm per pixel at this depth
        # At depth Z, moving 1 pixel corresponds to Z/f meters, or Z/f * 1000 mm
        mm_per_px_x = (depth_m / fx) * 1000.0
        mm_per_px_y = (depth_m / fy) * 1000.0

        return (mm_per_px_x, mm_per_px_y)

    def get_intrinsics_dict(self) -> dict:
        """
        Get camera intrinsics as a dictionary for easy inspection/logging.

        Returns:
            Dictionary with intrinsic parameters (fx, fy, ppx, ppy, width, height)
        """
        if not self.aligned_color_intrinsics:
            return {}

        intrinsics = self.aligned_color_intrinsics
        return {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'ppx': intrinsics.ppx,
            'ppy': intrinsics.ppy,
            'width': intrinsics.width,
            'height': intrinsics.height,
            'model': str(intrinsics.model)
        }

    def is_ready(self) -> bool:
        """Check if camera is ready for use."""
        return self.is_initialized
