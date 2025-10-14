"""
Temporal filtering for grasp angle consistency.

Reduces jitter by smoothing angles across frames.
Handles angle wraparound using circular statistics.
"""

import numpy as np
import logging
from typing import Optional, List

logger = logging.getLogger(__name__)


class TemporalAngleFilter:
    """
    Temporal filtering for angle stability across frames.

    Supports:
    - Circular mean (handles angle wrap-around)
    - Exponential moving average (EMA)
    - Median filter (robust to outliers)

    Includes 3-frame warmup to prevent first-frame bias.
    """

    def __init__(self, enabled: bool = True, window_size: int = 5,
                 filter_type: str = 'circular_mean', ema_alpha: float = 0.3,
                 outlier_threshold_deg: float = 30):
        """
        Args:
            enabled: Enable temporal filtering
            window_size: Number of frames to average (3-7 recommended)
            filter_type: 'circular_mean', 'median', or 'ema'
            ema_alpha: EMA smoothing factor (0.1-0.5, lower=smoother)
            outlier_threshold_deg: Outlier rejection threshold in degrees
        """
        self.enabled = enabled
        self.window_size = window_size
        self.filter_type = filter_type
        self.ema_alpha = ema_alpha
        self.outlier_threshold_deg = outlier_threshold_deg

        self.angle_history = []
        self.ema_angle = None

        if self.enabled:
            logger.info(
                f"Temporal filtering enabled: {filter_type}, window={window_size}")

    def filter(self, angle: float) -> float:
        """
        Apply temporal filter to angle.

        Args:
            angle: Raw angle prediction in radians

        Returns:
            Filtered angle in radians
        """
        if not self.enabled:
            return angle

        # Check for outliers (requires 3+ samples)
        if self._is_outlier(angle):
            logger.debug(
                f"⚠️  Angle outlier detected: {np.degrees(angle):.1f}° (rejecting)")
            if self.angle_history:
                return self.angle_history[-1]

        # Apply selected filter type
        if self.filter_type == 'ema':
            filtered_angle = self._ema_filter(angle)
        elif self.filter_type == 'median':
            filtered_angle = self._median_filter(angle)
        else:
            filtered_angle = self._circular_mean_filter(angle)

        logger.debug(f"🔄 Temporal filter: raw={np.degrees(angle):.1f}° → "
                     f"filtered={np.degrees(filtered_angle):.1f}° "
                     f"(history size: {len(self.angle_history)})")

        return filtered_angle

    def get_recent_angle(self) -> Optional[float]:
        """Get most recent filtered angle for temporal scoring."""
        return self.angle_history[-1] if self.angle_history else None

    def _circular_mean(self, angles: List[float]) -> float:
        """Compute circular mean of angles."""
        if not angles:
            return 0.0
        cos_sum = sum(np.cos(2 * a) for a in angles)
        sin_sum = sum(np.sin(2 * a) for a in angles)
        return 0.5 * np.arctan2(sin_sum, cos_sum)

    def _is_outlier(self, angle: float) -> bool:
        """Check if angle is an outlier."""
        if self.outlier_threshold_deg is None or len(self.angle_history) < 3:
            return False

        ref_mean = self._circular_mean(self.angle_history)
        diff = abs(angle - ref_mean)
        diff = min(diff, np.pi - diff)

        return diff > np.deg2rad(self.outlier_threshold_deg)

    def _circular_mean_filter(self, angle: float) -> float:
        """Circular mean filter (default)."""
        self.angle_history.append(angle)
        if len(self.angle_history) > self.window_size:
            self.angle_history.pop(0)
        return self._circular_mean(self.angle_history)

    def _median_filter(self, angle: float) -> float:
        """Median filter (robust to outliers)."""
        self.angle_history.append(angle)
        if len(self.angle_history) > self.window_size:
            self.angle_history.pop(0)
        return float(np.median(self.angle_history))

    def _ema_filter(self, angle: float) -> float:
        """Exponential moving average with circular interpolation."""
        if self.ema_angle is None:
            self.ema_angle = angle
        else:
            alpha = self.ema_alpha
            cos_new, sin_new = np.cos(2 * angle), np.sin(2 * angle)
            cos_ema, sin_ema = np.cos(
                2 * self.ema_angle), np.sin(2 * self.ema_angle)

            cos_result = alpha * cos_new + (1 - alpha) * cos_ema
            sin_result = alpha * sin_new + (1 - alpha) * sin_ema

            self.ema_angle = 0.5 * np.arctan2(sin_result, cos_result)

        return self.ema_angle

    def update(self, angle: float) -> float:
        """Alias for filter() - for backward compatibility."""
        return self.filter(angle)
