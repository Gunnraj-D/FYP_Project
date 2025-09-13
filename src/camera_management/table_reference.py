"""
Table Reference Module for computing object heights above the table.

This module maintains a per-pixel reference depth map of the table surface and
provides functionality to compute object heights above the table, robust to flat objects.

Key Features:
- Automatic table reference initialization from empty table or first frames
- Per-pixel exponential moving average updates for table depth reference
- Height estimation for both single pixels and ROI regions
- Integration with grasp detection pipeline
- Debug visualization capabilities
"""
import cv2
import numpy as np
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass
from threading import RLock

from config.config import TABLE_REF_UPDATE_ALPHA, TABLE_REF_TOLERANCE

logger = logging.getLogger(__name__)


@dataclass
class TableReferenceConfig:
    """Configuration for table reference depth model."""
    update_alpha: float = TABLE_REF_UPDATE_ALPHA
    tolerance: float = TABLE_REF_TOLERANCE
    initialization_frames: int = 5
    min_valid_depth: float = 0.1  # meters
    max_valid_depth: float = 3.0  # meters


class TableReferenceModule:
    """
    Maintains a per-pixel reference depth map of the table surface.

    Responsibilities:
    - Initialize table reference from empty table or first few frames
    - Update table reference using exponential moving average
    - Compute object heights above table for single pixels or ROI regions
    - Provide debug visualization capabilities
    """

    def __init__(self, config: TableReferenceConfig = None):
        self.config = config or TableReferenceConfig()
        self._lock = RLock()

        # Table reference depth map (per-pixel)
        self._table_depth_ref: Optional[np.ndarray] = None

        # Initialization state
        self._initialization_frames: List[np.ndarray] = []
        self._is_initialized = False
        self._frame_count = 0

        # Debug visualization
        self._debug_mode = False

        logger.info("TableReferenceModule initialized")

    def update_table_reference(self, depth_frame: np.ndarray) -> None:
        """
        Update the table reference depth map with current frame.

        Args:
            depth_frame: Current depth frame (numpy array in meters)
        """
        with self._lock:
            try:
                # Validate input
                if depth_frame is None or depth_frame.size == 0:
                    logger.warning("Invalid depth frame provided")
                    return

                # Convert to numpy array if needed
                if hasattr(depth_frame, 'get_data'):
                    depth_array = np.asanyarray(depth_frame.get_data()).astype(
                        np.float32) / 1000.0  # Convert mm to m
                else:
                    depth_array = depth_frame.astype(np.float32)

                # Filter out invalid depths
                valid_mask = (depth_array > self.config.min_valid_depth) & \
                    (depth_array < self.config.max_valid_depth)

                if not self._is_initialized:
                    self._initialize_table_reference(depth_array, valid_mask)
                else:
                    self._update_existing_reference(depth_array, valid_mask)

                self._frame_count += 1

            except Exception as e:
                logger.error(f"Failed to update table reference: {e}")

    def _initialize_table_reference(self, depth_array: np.ndarray, valid_mask: np.ndarray) -> None:
        """
        Initialize table reference from first few frames.

        Args:
            depth_array: Current depth frame
            valid_mask: Mask of valid depth pixels
        """
        try:
            # Store frame for initialization
            self._initialization_frames.append(depth_array.copy())

            # Check if we have enough frames
            if len(self._initialization_frames) >= self.config.initialization_frames:
                # Compute per-pixel median as initial table reference
                frames_array = np.stack(self._initialization_frames, axis=0)

                # Compute median only for valid pixels
                self._table_depth_ref = np.full_like(depth_array, np.nan)

                for i in range(depth_array.shape[0]):
                    for j in range(depth_array.shape[1]):
                        pixel_depths = frames_array[:, i, j]
                        valid_depths = pixel_depths[
                            (pixel_depths > self.config.min_valid_depth) &
                            (pixel_depths < self.config.max_valid_depth)
                        ]

                        if len(valid_depths) > 0:
                            self._table_depth_ref[i, j] = np.median(
                                valid_depths)

                self._is_initialized = True
                self._initialization_frames.clear()  # Free memory

                logger.info(
                    f"Table reference initialized from {self.config.initialization_frames} frames")

            else:
                logger.debug(
                    f"Collecting initialization frames: {len(self._initialization_frames)}/{self.config.initialization_frames}")

        except Exception as e:
            logger.error(f"Failed to initialize table reference: {e}")

    def _update_existing_reference(self, depth_array: np.ndarray, valid_mask: np.ndarray) -> None:
        """
        Update existing table reference using exponential moving average.

        Args:
            depth_array: Current depth frame
            valid_mask: Mask of valid depth pixels
        """
        try:
            if self._table_depth_ref is None:
                logger.warning("Table reference not initialized")
                return

            # Compute delta between table reference and current depth
            delta = self._table_depth_ref - depth_array

            # Mark pixels as table surface where delta is within tolerance
            table_mask = np.abs(delta) < self.config.tolerance

            # Only update pixels that are both table surface and valid
            update_mask = table_mask & valid_mask & ~np.isnan(
                self._table_depth_ref)

            # Update table reference using exponential moving average
            alpha = self.config.update_alpha
            self._table_depth_ref[update_mask] = (
                alpha * depth_array[update_mask] +
                (1 - alpha) * self._table_depth_ref[update_mask]
            )

            # Log update statistics
            if self._frame_count % 100 == 0:  # Log every 100 frames
                update_ratio = np.sum(update_mask) / update_mask.size
                logger.debug(
                    f"Table reference updated: {update_ratio:.2%} of pixels updated")

        except Exception as e:
            logger.error(f"Failed to update existing table reference: {e}")

    def get_height_above_table(self, depth_frame: np.ndarray,
                               grasp_uv: Tuple[int, int],
                               roi_mask: Optional[np.ndarray] = None) -> float:
        """
        Compute height of object above table at given location.

        Args:
            depth_frame: Current depth frame
            grasp_uv: (u, v) pixel coordinates of grasp center
            roi_mask: Optional ROI mask for region-based height computation

        Returns:
            Height above table in meters (always >= 0)
        """
        with self._lock:
            try:
                if not self._is_initialized or self._table_depth_ref is None:
                    logger.warning("Table reference not initialized")
                    return 0.0

                # Convert depth frame to numpy array if needed
                if hasattr(depth_frame, 'get_data'):
                    depth_array = np.asanyarray(depth_frame.get_data()).astype(
                        np.float32) / 1000.0  # Convert mm to m
                else:
                    depth_array = depth_frame.astype(np.float32)

                # Validate grasp coordinates
                u, v = grasp_uv
                if (u < 0 or u >= depth_array.shape[1] or
                        v < 0 or v >= depth_array.shape[0]):
                    logger.warning(
                        f"Grasp coordinates out of bounds: {grasp_uv}")
                    return 0.0

                # Compute object depth
                if roi_mask is not None:
                    # Region-based computation
                    object_depth = self._compute_roi_depth(
                        depth_array, roi_mask)
                else:
                    # Single pixel computation
                    object_depth = depth_array[v, u]

                if np.isnan(object_depth) or object_depth <= 0:
                    logger.warning("Invalid object depth")
                    return 0.0

                # Compute table reference depth
                if roi_mask is not None:
                    # Region-based table reference
                    table_depth = self._compute_roi_depth(
                        self._table_depth_ref, roi_mask)
                else:
                    # Single pixel table reference
                    table_depth = self._table_depth_ref[v, u]

                if np.isnan(table_depth) or table_depth <= 0:
                    logger.warning("Invalid table reference depth")
                    return 0.0

                # Height = table_depth - object_depth
                height = table_depth - object_depth

                # Ensure non-negative height
                height = max(0.0, height)

                logger.debug(
                    f"Height above table: {height:.3f}m (table: {table_depth:.3f}m, object: {object_depth:.3f}m)")
                return height

            except Exception as e:
                logger.error(f"Failed to compute height above table: {e}")
                return 0.0

    def _compute_roi_depth(self, depth_map: np.ndarray, roi_mask: np.ndarray) -> float:
        """
        Compute median depth within ROI region.

        Args:
            depth_map: Depth map (table reference or current depth)
            roi_mask: Binary mask defining ROI

        Returns:
            Median depth within ROI
        """
        try:
            # Apply mask and get valid depths
            masked_depths = depth_map[roi_mask > 0]
            valid_depths = masked_depths[
                (masked_depths > self.config.min_valid_depth) &
                (masked_depths < self.config.max_valid_depth) &
                ~np.isnan(masked_depths)
            ]

            if len(valid_depths) == 0:
                return np.nan

            return np.median(valid_depths)

        except Exception as e:
            logger.error(f"Failed to compute ROI depth: {e}")
            return np.nan

    def create_grasp_roi_mask(self, grasp_center: Tuple[int, int],
                              grasp_width: float, grasp_angle: float,
                              image_shape: Tuple[int, int]) -> np.ndarray:
        """
        Create ROI mask for grasp rectangle.

        Args:
            grasp_center: (u, v) center of grasp
            grasp_width: Width of grasp in pixels
            grasp_angle: Angle of grasp in radians
            image_shape: (height, width) of image

        Returns:
            Binary mask for grasp ROI
        """
        try:
            height, width = image_shape
            mask = np.zeros((height, width), dtype=np.uint8)

            # Convert grasp width to pixels (assuming it's in normalized coordinates)
            # This is a simplified conversion - you may need to adjust based on your setup
            # Scale factor for pixel conversion
            pixel_width = int(grasp_width * 10)
            # Clamp to reasonable range
            pixel_width = max(5, min(pixel_width, 50))

            # Create rectangle around grasp center
            center_u, center_v = grasp_center

            # Calculate rectangle corners
            half_width = pixel_width // 2
            # Make rectangle more elongated
            half_height = max(2, pixel_width // 4)

            # Create rectangle points
            pts = np.array([
                [center_u - half_width, center_v - half_height],
                [center_u + half_width, center_v - half_height],
                [center_u + half_width, center_v + half_height],
                [center_u - half_width, center_v + half_height]
            ], dtype=np.int32)

            # Fill rectangle in mask
            cv2.fillPoly(mask, [pts], 255)

            return mask

        except Exception as e:
            logger.error(f"Failed to create grasp ROI mask: {e}")
            return np.zeros(image_shape, dtype=np.uint8)

    def is_initialized(self) -> bool:
        """Check if table reference is initialized."""
        with self._lock:
            return self._is_initialized

    def get_initialization_progress(self) -> float:
        """Get initialization progress as fraction (0.0 to 1.0)."""
        with self._lock:
            if self._is_initialized:
                return 1.0
            return len(self._initialization_frames) / self.config.initialization_frames

    def set_debug_mode(self, enabled: bool) -> None:
        """Enable/disable debug visualization mode."""
        with self._lock:
            self._debug_mode = enabled
            logger.info(f"Debug mode {'enabled' if enabled else 'disabled'}")

    def get_debug_visualization(self, depth_frame: np.ndarray) -> Optional[np.ndarray]:
        """
        Generate debug visualization of table reference.

        Args:
            depth_frame: Current depth frame

        Returns:
            Debug visualization image or None if not in debug mode
        """
        with self._lock:
            if not self._debug_mode or not self._is_initialized or self._table_depth_ref is None:
                return None

            try:
                # Convert depth frame to numpy array if needed
                if hasattr(depth_frame, 'get_data'):
                    depth_array = np.asanyarray(
                        depth_frame.get_data()).astype(np.float32) / 1000.0
                else:
                    depth_array = depth_frame.astype(np.float32)

                # Compute difference between table reference and current depth
                depth_diff = self._table_depth_ref - depth_array

                # Create visualization
                # Normalize difference to [0, 255] for visualization
                diff_normalized = np.clip(
                    (depth_diff + self.config.tolerance) /
                    (2 * self.config.tolerance),
                    0.0, 1.0
                )

                # Convert to color image
                diff_colored = cv2.applyColorMap(
                    (diff_normalized * 255).astype(np.uint8),
                    cv2.COLORMAP_JET
                )

                # Overlay valid/invalid regions
                valid_mask = (depth_array > self.config.min_valid_depth) & \
                    (depth_array < self.config.max_valid_depth)
                diff_colored[~valid_mask] = [128, 128, 128]  # Gray for invalid

                return diff_colored

            except Exception as e:
                logger.error(f"Failed to generate debug visualization: {e}")
                return None

    def reset(self) -> None:
        """Reset table reference to uninitialized state."""
        with self._lock:
            self._table_depth_ref = None
            self._initialization_frames.clear()
            self._is_initialized = False
            self._frame_count = 0
            logger.info("Table reference reset")

    def get_stats(self) -> dict:
        """Get statistics about table reference."""
        with self._lock:
            stats = {
                'initialized': self._is_initialized,
                'frame_count': self._frame_count,
                'initialization_progress': self.get_initialization_progress(),
                'config': {
                    'update_alpha': self.config.update_alpha,
                    'tolerance': self.config.tolerance,
                    'initialization_frames': self.config.initialization_frames
                }
            }

            if self._is_initialized and self._table_depth_ref is not None:
                valid_pixels = ~np.isnan(self._table_depth_ref)
                stats.update({
                    'table_depth_mean': float(np.nanmean(self._table_depth_ref)),
                    'table_depth_std': float(np.nanstd(self._table_depth_ref)),
                    'valid_pixel_ratio': float(np.sum(valid_pixels) / valid_pixels.size)
                })

            return stats
