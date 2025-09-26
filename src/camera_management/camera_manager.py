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
        self.intrinsics: Optional[rs.intrinsics] = None
        self.is_initialized = False

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
            self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

            # Create align object for depth-to-color alignment
            self.align = rs.align(rs.stream.color)

            self.is_initialized = True
            logger.info("Camera initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Camera initialization failed: {e}")
            self.is_initialized = False
            return False

    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[rs.depth_frame]]:
        """Get aligned color and depth frames."""
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
        """Convert pixel coordinates to 3D vector in meters."""
        if not self.intrinsics or depth <= 0:
            return (0.0, 0.0, 0.0)

        try:
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [x, y], depth
            )
            # Return coordinates in meters (RealSense already provides meters)
            return (point_3d[0], point_3d[1], point_3d[2])
        except Exception as e:
            logger.error(f"3D conversion error: {e}")
            return (0.0, 0.0, 0.0)

    def get_average_depth(self, depth_frame: rs.depth_frame,
                          center: Tuple[int, int], radius: int) -> float:
        """Get median (used to be average) depth in circular region."""
        try:
            h, w = depth_frame.get_height(), depth_frame.get_width()
            cx, cy = center

            if cx < 0 or cx >= w or cy < 0 or cy >= h:
                return 0.0

            # Create circular mask
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (cx, cy), radius, 255, -1)

            depth_image = np.asanyarray(depth_frame.get_data())
            valid_depths = depth_image[mask == 255]
            valid_depths = valid_depths[valid_depths > 0]

            if valid_depths.size == 0:
                return 0.0

            return np.median(valid_depths) * depth_frame.get_units()

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
        self.intrinsics = None
        self.is_initialized = False
        logger.info("Camera cleanup complete")

    def is_ready(self) -> bool:
        """Check if camera is ready for use."""
        return self.is_initialized
