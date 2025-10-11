"""
Debug visualization for grasp detection.

Provides:
- Input frame visualization (depth/RGB-D)
- Grasp output overlay (quality map + selected grasp)
"""

import cv2
import numpy as np
import logging
from typing import Dict, Optional

from config.config import DEBUG_MODE, DEBUG_CONFIG

logger = logging.getLogger(__name__)


class GraspVisualizer:
    """
    Debug visualization for grasp detection pipeline.

    Shows:
    - Input depth/color frames
    - Quality map overlays
    - Selected grasp rectangle and angle
    """

    def __init__(self, enabled: bool = DEBUG_MODE):
        """
        Args:
            enabled: Enable visualization (from DEBUG_MODE config)
        """
        self.enabled = enabled
        self.window_titles = set()

    def visualize_input_frame(self, depth_image: np.ndarray, title: str = "Input Depth"):
        """
        Show input depth frame.

        Args:
            depth_image: Depth array in meters
            title: Window title
        """
        if not self.enabled:
            return

        try:
            # Normalize depth for display
            depth_display = self._normalize_depth_for_display(depth_image)

            # Apply colormap
            depth_colored = cv2.applyColorMap(
                (depth_display * 255).astype(np.uint8), cv2.COLORMAP_JET
            )

            cv2.imshow(title, depth_colored)
            cv2.waitKey(1)
            self.window_titles.add(title)

        except Exception as e:
            logger.warning(f"Failed to visualize input frame: {e}")

    def visualize_grasp_output(self, depth_image: np.ndarray, grasp_2d: Dict,
                               title: str = "Grasp Output"):
        """
        Show grasp output overlaid on depth image.

        Displays:
        - Depth image as background
        - Selected grasp rectangle
        - Grasp angle arrow
        - Quality score text

        Args:
            depth_image: Depth array in meters
            grasp_2d: Dict with 'center', 'angle', 'width', 'quality'
            title: Window title
        """
        if not self.enabled:
            return

        try:
            # Normalize depth for display
            depth_display = self._normalize_depth_for_display(depth_image)

            # Convert to color for drawing
            depth_colored = cv2.applyColorMap(
                (depth_display * 255).astype(np.uint8), cv2.COLORMAP_JET
            )

            # Resize to 300x300 if needed (match network resolution)
            if depth_colored.shape[:2] != (300, 300):
                depth_colored = cv2.resize(depth_colored, (300, 300))

            # Draw grasp overlay
            depth_colored = self._draw_grasp_overlay(depth_colored, grasp_2d)

            # Enlarge for better visibility
            display_size = DEBUG_CONFIG.get('visualization_size', 600)
            depth_colored = cv2.resize(depth_colored, (display_size, display_size),
                                       interpolation=cv2.INTER_NEAREST)

            cv2.imshow(title, depth_colored)
            cv2.waitKey(1)
            self.window_titles.add(title)

        except Exception as e:
            logger.warning(f"Failed to visualize grasp output: {e}")

    def _normalize_depth_for_display(self, depth_image: np.ndarray) -> np.ndarray:
        """Normalize depth to [0,1] range for visualization."""
        # Clip to reasonable range (0.2m to 1.2m for table)
        depth_clipped = np.clip(depth_image, 0.2, 1.2)

        # Normalize to [0,1]
        depth_min, depth_max = depth_clipped.min(), depth_clipped.max()
        if depth_max - depth_min > 0:
            depth_normalized = (depth_clipped - depth_min) / \
                (depth_max - depth_min)
        else:
            depth_normalized = depth_clipped

        return depth_normalized

    def _draw_grasp_overlay(self, image: np.ndarray, grasp_2d: Dict) -> np.ndarray:
        """
        Draw grasp rectangle and angle arrow on image.

        Args:
            image: BGR image (H, W, 3)
            grasp_2d: Grasp parameters

        Returns:
            Image with grasp overlay
        """
        overlay = image.copy()

        # Extract grasp parameters
        center_u, center_v = grasp_2d["center"]
        angle = grasp_2d["angle"]
        width = grasp_2d.get("width", 50)  # pixels
        quality = grasp_2d["quality"]

        # Scale for visualization (resize from network resolution)
        scale_u = image.shape[1] / 300.0
        scale_v = image.shape[0] / 300.0

        center_u_scaled = int(center_u * scale_u)
        center_v_scaled = int(center_v * scale_v)
        width_scaled = width * scale_u

        # Draw grasp rectangle
        # Rectangle height is fixed (jaw width)
        rect_height = 20 * scale_v
        half_width = width_scaled / 2
        half_height = rect_height / 2

        # Compute rectangle corners
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        corners = np.array([
            [-half_width, -half_height],
            [half_width, -half_height],
            [half_width, half_height],
            [-half_width, half_height]
        ])

        # Rotate corners
        rot_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        corners_rotated = corners @ rot_matrix.T

        # Translate to grasp center
        corners_rotated[:, 0] += center_u_scaled
        corners_rotated[:, 1] += center_v_scaled
        corners_rotated = corners_rotated.astype(np.int32)

        # Draw rectangle
        cv2.polylines(overlay, [corners_rotated], True, (0, 255, 0), 2)

        # Draw center point
        cv2.circle(overlay, (center_u_scaled, center_v_scaled),
                   5, (0, 255, 0), -1)

        # Draw angle arrow
        arrow_length = 40 * scale_u
        arrow_end_u = int(center_u_scaled + arrow_length * cos_a)
        arrow_end_v = int(center_v_scaled + arrow_length * sin_a)
        cv2.arrowedLine(overlay, (center_u_scaled, center_v_scaled),
                        (arrow_end_u, arrow_end_v), (255, 255, 0), 2, tipLength=0.3)

        # Draw quality text
        text = f"Q: {quality:.3f}  Angle: {np.degrees(angle):.1f}°"
        cv2.putText(overlay, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)

        return overlay

    def cleanup(self):
        """Close all visualization windows."""
        if not self.enabled:
            return

        try:
            for title in self.window_titles:
                cv2.destroyWindow(title)
            self.window_titles.clear()
            logger.debug("Closed visualization windows")
        except Exception as e:
            logger.warning(f"Failed to cleanup visualization windows: {e}")
