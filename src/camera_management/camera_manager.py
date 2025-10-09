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

            # After depth-to-color alignment, aligned depth has same intrinsics as color
            self.aligned_color_intrinsics = self.color_intrinsics

            # Create align object for depth-to-color alignment
            self.align = rs.align(rs.stream.color)

            self.is_initialized = True
            logger.info("Camera initialized successfully")
            logger.info(
                f"Color intrinsics: {self.color_intrinsics.width}x{self.color_intrinsics.height}, "
                f"fx={self.color_intrinsics.fx:.1f}, fy={self.color_intrinsics.fy:.1f}")
            logger.info(
                f"Depth intrinsics: {self.depth_intrinsics.width}x{self.depth_intrinsics.height}, "
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

    def get_average_depth(self, depth_frame: rs.depth_frame,
                          center: Tuple[int, int], radius: int,
                          min_valid_pixels: int = 5) -> float:
        """
        Get median depth in circular region with robust outlier rejection.

        Uses caching to avoid repeated get_data() calls for the same frame,
        significantly improving performance when called multiple times per frame.

        Args:
            depth_frame: RealSense depth frame (aligned to color)
            center: (x, y) center pixel coordinates
            radius: Radius in pixels for circular sampling region
            min_valid_pixels: Minimum number of valid depth pixels required (default: 5)

        Returns:
            Median depth in meters, or 0.0 if insufficient valid data
        """
        try:
            # Early validation of radius
            if radius <= 0:
                logger.debug(f"Invalid radius {radius}, returning 0.0")
                return 0.0

            # Get frame metadata
            frame_number = depth_frame.get_frame_number()
            h, w = depth_frame.get_height(), depth_frame.get_width()
            cx, cy = center

            # Validate coordinates
            if cx < 0 or cx >= w or cy < 0 or cy >= h:
                logger.debug(
                    f"Center ({cx}, {cy}) out of bounds ({w}x{h}), returning 0.0")
                return 0.0

            # Check cache and populate if necessary
            if (self._depth_cache_frame_id != frame_number or
                    self._depth_cache_array is None):
                # Cache miss - convert frame data
                self._depth_cache_array = np.asanyarray(depth_frame.get_data())
                self._depth_cache_scale = depth_frame.get_units()
                self._depth_cache_dims = (h, w)
                self._depth_cache_frame_id = frame_number
                logger.debug(f"Depth cache updated for frame {frame_number}")

            # Use cached data
            depth_image = self._depth_cache_array
            depth_scale = self._depth_cache_scale

            # Create circular mask
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (cx, cy), radius, 255, -1)

            # Extract valid depths in circular region
            valid_depths = depth_image[mask == 255]
            valid_depths = valid_depths[valid_depths > 0]

            # Require minimum number of valid pixels
            if valid_depths.size < min_valid_pixels:
                logger.debug(
                    f"Insufficient valid pixels at ({cx}, {cy}): "
                    f"got {valid_depths.size}, need {min_valid_pixels}"
                )
                return 0.0

            # Use percentile clipping to reject outliers
            # This helps when there are a few noisy depth readings
            if valid_depths.size >= 10:
                # For larger samples, use interquartile range (25th to 75th percentile)
                # This is more robust than using all data
                lower = np.percentile(valid_depths, 25)
                upper = np.percentile(valid_depths, 75)
                clipped_depths = valid_depths[(
                    valid_depths >= lower) & (valid_depths <= upper)]

                # If clipping removes too much data, fall back to full dataset
                if clipped_depths.size >= min_valid_pixels:
                    valid_depths = clipped_depths
                    logger.debug(
                        f"Applied IQR clipping: {clipped_depths.size} pixels retained")

            # Return median depth in meters
            median_depth = np.median(valid_depths) * depth_scale
            return median_depth

        except Exception as e:
            logger.error(f"Depth calculation error: {e}")
            return 0.0

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

    def is_ready(self) -> bool:
        """Check if camera is ready for use."""
        return self.is_initialized
