"""
Grasp candidate postprocessing and temporal filtering.

Handles:
- Candidate selection via NMS and scoring
- Temporal filtering for angle stability
- Quality thresholding and geometric penalties
- PCA-based geometric validation (for rectangular objects)
"""

import cv2
import numpy as np
import logging
from typing import Optional, Dict, List, Tuple
from collections import deque

from config import GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE

logger = logging.getLogger(__name__)


def compute_pca_angle(depth: np.ndarray, center: Tuple[int, int],
                      crop_size: int = 60, depth_thresh: float = 0.01) -> Optional[float]:
    """
    Compute principal axis angle of object using PCA on depth data.

    This provides a geometric ground-truth for rectangular objects.

    Args:
        depth: Depth image in meters (H, W)
        center: Grasp center (u, v) in pixels
        crop_size: Crop radius around center
        depth_thresh: Minimum depth variance to consider valid

    Returns:
        Principal axis angle in radians (image coords) or None if failed
    """
    try:
        h, w = depth.shape
        x, y = int(center[0]), int(center[1])

        # Crop around center
        x0, x1 = max(0, x - crop_size), min(w, x + crop_size)
        y0, y1 = max(0, y - crop_size), min(h, y + crop_size)
        crop = depth[y0:y1, x0:x1].copy()

        # Mask valid depth (object pixels)
        valid = (crop > 0) & np.isfinite(crop)

        if not np.any(valid) or np.sum(valid) < 10:
            return None

        # Get valid pixel coordinates
        ys, xs = np.where(valid)
        pts = np.stack([xs, ys], axis=1).astype(np.float32)

        # Center points
        pts -= pts.mean(axis=0)

        # Compute covariance and PCA
        cov = np.cov(pts, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # Principal component (long axis direction)
        principal = eigvecs[:, np.argmax(eigvals)]

        # Angle in image coordinates (x right, y down)
        angle = np.arctan2(principal[1], principal[0])

        return angle

    except Exception as e:
        logger.debug(f"PCA angle computation failed: {e}")
        return None


def angle_difference(angle1: float, angle2: float) -> float:
    """
    Compute angular distance between two angles (handles wrap-around and 180° symmetry).

    Args:
        angle1, angle2: Angles in radians

    Returns:
        Angular distance in radians [0, π/2]
    """
    diff = abs(angle1 - angle2)
    # Handle wrap-around at ±π/2
    diff = min(diff, np.pi - diff)
    # Handle 180° gripper symmetry
    diff = min(diff, np.pi/2)
    return diff


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
                f"✨ Temporal filtering enabled: {filter_type}, window={window_size}")

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

        # Check for outliers (requires 3+ samples to prevent first-frame bias)
        if self._is_outlier(angle):
            logger.debug(
                f"⚠️  Angle outlier detected: {np.degrees(angle):.1f}° (rejecting)")
            # Return previous filtered value instead
            if self.angle_history:
                return self.angle_history[-1]

        # Apply selected filter type
        if self.filter_type == 'ema':
            filtered_angle = self._ema_filter(angle)
        elif self.filter_type == 'median':
            filtered_angle = self._median_filter(angle)
        else:  # 'circular_mean' (default)
            filtered_angle = self._circular_mean_filter(angle)

        logger.debug(f"🔄 Temporal filter: raw={np.degrees(angle):.1f}° → "
                     f"filtered={np.degrees(filtered_angle):.1f}° "
                     f"(history size: {len(self.angle_history)})")

        return filtered_angle

    def _circular_mean(self, angles: List[float]) -> float:
        """Compute circular mean of angles (handles wrap-around)."""
        if not angles:
            return 0.0
        cos_sum = sum(np.cos(2 * a) for a in angles)
        sin_sum = sum(np.sin(2 * a) for a in angles)
        return 0.5 * np.arctan2(sin_sum, cos_sum)

    def _is_outlier(self, angle: float) -> bool:
        """Check if angle is an outlier (requires 3+ samples)."""
        if self.outlier_threshold_deg is None or len(self.angle_history) < 3:
            return False  # Warmup period

        ref_mean = self._circular_mean(self.angle_history)
        diff = abs(angle - ref_mean)
        diff = min(diff, np.pi - diff)  # Handle periodicity (gripper symmetry)

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
            # Circular interpolation for EMA
            alpha = self.ema_alpha
            cos_new, sin_new = np.cos(2 * angle), np.sin(2 * angle)
            cos_ema, sin_ema = np.cos(
                2 * self.ema_angle), np.sin(2 * self.ema_angle)

            cos_result = alpha * cos_new + (1 - alpha) * cos_ema
            sin_result = alpha * sin_new + (1 - alpha) * sin_ema

            self.ema_angle = 0.5 * np.arctan2(sin_result, cos_result)

        return self.ema_angle


class GraspPostprocessor:
    """
    Selects best grasp candidate from network outputs.

    Features:
    - Gaussian smoothing to reduce noise
    - NMS (Non-Maximum Suppression) for local peaks
    - Quality-weighted centroid for interior preference
    - Depth stability and width penalties
    - Temporal filtering for angle stability
    """

    def __init__(self, camera_manager, temporal_filter: Optional[TemporalAngleFilter] = None):
        """
        Args:
            camera_manager: CameraManager instance for depth sampling
            temporal_filter: Optional TemporalAngleFilter instance
        """
        self.camera_manager = camera_manager
        self.temporal_filter = temporal_filter or TemporalAngleFilter(
            enabled=False)
        self.depth_sample_radius = GRASP_DETECTION_CONFIG.get(
            'depth_sample_radius', 5)

    def postprocess(self, q_img, ang_img, width_img,
                    depth_image=None, original_depth_frame=None,
                    top_k: int = 8) -> Optional[Dict]:
        """
        Select best grasp candidate from network outputs.

        Pipeline:
        1. Smooth quality map (Gaussian blur)
        2. Mask by in-plane angle (optional, disabled by default)
        3. Local maxima detection (NMS)
        4. Quality-weighted centroid computation
        5. Candidate scoring (quality, distance, edge, width penalties)
        6. Temporal filtering on selected angle

        Args:
            q_img: Quality map tensor (B, 1, H, W)
            ang_img: Angle map tensor (B, 1, H, W)
            width_img: Width map tensor (B, 1, H, W)
            depth_image: Numpy depth array for scoring (meters)
            original_depth_frame: RealSense frame for accurate depth sampling
            top_k: Number of candidates to consider

        Returns:
            Dict with keys: center (u,v), angle, width, quality, score, depth_m, width_m
            or None if no valid grasp found
        """
        # Convert to numpy
        q_np = q_img.squeeze().cpu().numpy()
        ang_np = ang_img.squeeze().cpu().numpy()
        width_np = width_img.squeeze().cpu().numpy()

        # 1. Smooth quality map
        q_blur = cv2.GaussianBlur(q_np, (5, 5), 2)

        # 2. Optional in-plane angle filtering (default: disabled for free rotation)
        q_blur = self._apply_angle_mask(q_blur, ang_np)

        # 3. NMS: Find local maxima
        candidates = self._find_local_maxima(q_blur, top_k)

        if not candidates:
            logger.warning("No local maxima found in quality map")
            return None

        # 4. Compute quality-weighted centroid
        centroid = self._compute_centroid(q_blur)

        # 5. Score candidates
        best = self._score_candidates(
            candidates, q_blur, ang_np, width_np, centroid,
            depth_image, original_depth_frame
        )

        if best is None:
            logger.warning("No valid grasp candidate found after scoring")
            return None

        # 6. Apply temporal filtering to selected angle
        raw_angle = best['angle']
        filtered_angle = self.temporal_filter.filter(raw_angle)

        if self.temporal_filter.enabled:
            logger.info(f"📐 Angle: raw={np.degrees(raw_angle):.1f}° → "
                        f"filtered={np.degrees(filtered_angle):.1f}°")
            best['angle'] = filtered_angle

        return best

    def _apply_angle_mask(self, q_blur: np.ndarray, ang_np: np.ndarray) -> np.ndarray:
        """Optionally mask quality map by in-plane angle (default: disabled)."""
        topdown_tolerance = GRASP_DETECTION_CONFIG.get(
            'topdown_angle_tolerance_rad', None)

        if topdown_tolerance is None:
            logger.debug(
                "In-plane angle filtering DISABLED - all orientations allowed")
            return q_blur

        # Apply angle mask
        topdown_ref_angle = GRASP_DETECTION_CONFIG.get(
            'topdown_ref_angle', 0.0)
        angle_distance = np.abs(ang_np - topdown_ref_angle)
        angle_distance = np.minimum(angle_distance, np.pi - angle_distance)

        mask = angle_distance <= topdown_tolerance
        q_masked = q_blur * mask

        logger.info(f"In-plane angle filter ACTIVE: kept {mask.sum()}/{mask.size} pixels "
                    f"(ref: {np.degrees(topdown_ref_angle):.1f}°, "
                    f"tolerance: ±{np.degrees(topdown_tolerance):.1f}°)")

        return q_masked

    def _find_local_maxima(self, q_blur: np.ndarray, top_k: int) -> List[tuple]:
        """Find local maxima via NMS (Non-Maximum Suppression)."""
        thresh = max(GRASP_DETECTION_CONFIG.get(
            'min_quality_threshold', 0.15), 0.02)

        # Dilation-based NMS
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(q_blur, kernel)
        local_max_mask = (q_blur == dilated) & (q_blur > thresh)

        if local_max_mask.sum() == 0:
            # No local maxima, use top-k global
            flat_idxs = np.argpartition(q_blur.flatten(), -top_k)[-top_k:]
            cand_rows, cand_cols = np.unravel_index(flat_idxs, q_blur.shape)
        else:
            cand_rows, cand_cols = np.where(local_max_mask)
            if len(cand_rows) > top_k:
                qualities = q_blur[cand_rows, cand_cols]
                order = np.argsort(qualities)[-top_k:]
                cand_rows, cand_cols = cand_rows[order], cand_cols[order]

        return list(zip(cand_rows, cand_cols))

    def _compute_centroid(self, q_blur: np.ndarray) -> tuple:
        """Compute quality-weighted centroid (prefers interior grasps)."""
        thresh = max(GRASP_DETECTION_CONFIG.get(
            'min_quality_threshold', 0.15), 0.02)
        mask = q_blur.copy()
        mask[mask < thresh] = 0.0

        if mask.sum() > 0:
            rows_idx = np.arange(q_blur.shape[0])[:, None]
            cols_idx = np.arange(q_blur.shape[1])[None, :]
            com_v = float((mask * rows_idx).sum() / mask.sum())
            com_u = float((mask * cols_idx).sum() / mask.sum())
        else:
            # Fallback: image center
            com_v, com_u = q_blur.shape[0] / 2.0, q_blur.shape[1] / 2.0

        return (com_u, com_v)

    def _score_candidates(self, candidates: List[tuple], q_blur: np.ndarray,
                          ang_np: np.ndarray, width_np: np.ndarray, centroid: tuple,
                          depth_image, original_depth_frame) -> Optional[Dict]:
        """Score candidates with quality, distance, edge, and width penalties."""
        com_u, com_v = centroid
        h_resized, w_resized = q_blur.shape

        # Get intrinsics
        aligned_intr = getattr(self.camera_manager, "aligned_color_intrinsics", None) or \
            getattr(self.camera_manager, "color_intrinsics", None)
        fx = aligned_intr.fx if aligned_intr and hasattr(
            aligned_intr, "fx") else None
        depth_img_width = aligned_intr.width if aligned_intr and hasattr(
            aligned_intr, "width") else None

        # Gripper width limits
        grip_min = GRASP_EXECUTION_CONFIG.get('gripper_min_width_m', 0.02)
        grip_max = GRASP_EXECUTION_CONFIG.get('gripper_max_width_m', 0.12)

        best, best_score = None, -1e9
        all_candidates = []

        for r, c in candidates:
            quality = float(q_blur[r, c])

            # Distance to centroid (normalized)
            dist = np.hypot(c - com_u, r - com_v) / max(h_resized, w_resized)

            # Sample depth
            depth_m = self._sample_depth(
                c, r, depth_image, original_depth_frame)
            if depth_m is None or depth_m <= 0:
                logger.debug(
                    f"Skipping candidate at ({c}, {r}) - invalid depth")
                continue

            # Edge penalty (depth variance)
            edge_penalty = self._compute_edge_penalty(
                c, r, depth_m, depth_image, original_depth_frame)

            # Width penalty
            width_pixels = float(width_np[r, c])
            width_m = self._convert_width_to_meters(
                width_pixels, depth_m, fx, depth_img_width)
            width_penalty = self._compute_width_penalty(
                width_m, grip_min, grip_max)

            # Combined score (higher is better)
            score = (
                2.0 * quality          # Prioritize network quality
                - 0.4 * dist           # Allow off-center high-quality grasps
                - 0.9 * edge_penalty   # Avoid edges
                - 0.7 * width_penalty  # Gripper limits
            )

            candidate = {
                "center": (int(c), int(r)),
                "angle": float(ang_np[r, c]),
                "width": width_pixels,
                "quality": quality,
                "score": score,
                "depth_m": depth_m,
                "width_m": width_m
            }
            all_candidates.append(candidate)

            if score > best_score:
                best_score, best = score, candidate

        # Debug: Log top candidates
        if DEBUG_MODE and len(all_candidates) > 1:
            top = sorted(all_candidates,
                         key=lambda x: x['score'], reverse=True)[:5]
            logger.info(f"📊 Top {len(top)} grasp candidates:")
            for i, cand in enumerate(top):
                marker = "👑" if i == 0 else f" {i+1}."
                logger.info(f"  {marker} Angle: {np.degrees(cand['angle']):6.1f}° | "
                            f"Quality: {cand['quality']:.3f} | Score: {cand['score']:.3f}")

        # Final quality threshold check
        if best and best['quality'] < GRASP_DETECTION_CONFIG.get('min_quality_threshold', 0.15):
            logger.warning(
                f"Best candidate quality {best['quality']:.3f} below threshold")
            return None

        return best

    def _sample_depth(self, u: int, v: int, depth_image, original_depth_frame) -> Optional[float]:
        """Sample depth at (u,v) with nearest-valid fallback."""
        if depth_image is None and original_depth_frame is None:
            return None

        if original_depth_frame is not None:
            # Use RealSense frame (more accurate)
            h_orig = original_depth_frame.get_height()
            w_orig = original_depth_frame.get_width()
            su = int(u * w_orig / 300.0)
            sv = int(v * h_orig / 300.0)

            # Try primary location
            d = self.camera_manager.get_average_depth(
                original_depth_frame, (su, sv), radius=self.depth_sample_radius)
            if d and d > 0:
                return float(d)

            # 3x3 nearest-valid fallback
            for dv in (-1, 0, 1):
                for du in (-1, 0, 1):
                    if du == dv == 0:
                        continue
                    uu, vv = su + du, sv + dv
                    if 0 <= uu < w_orig and 0 <= vv < h_orig:
                        d2 = self.camera_manager.get_average_depth(
                            original_depth_frame, (uu, vv), radius=self.depth_sample_radius)
                        if d2 and d2 > 0:
                            return float(d2)
        else:
            # Use numpy array
            h_orig, w_orig = depth_image.shape
            su = int(u * w_orig / 300.0)
            sv = int(v * h_orig / 300.0)

            # Try primary location
            if 0 <= su < w_orig and 0 <= sv < h_orig and depth_image[sv, su] > 0:
                return float(depth_image[sv, su])

            # 3x3 nearest-valid fallback
            for dv in (-1, 0, 1):
                for du in (-1, 0, 1):
                    if du == dv == 0:
                        continue
                    uu, vv = su + du, sv + dv
                    if 0 <= uu < w_orig and 0 <= vv < h_orig and depth_image[vv, uu] > 0:
                        return float(depth_image[vv, uu])

        return None

    def _compute_edge_penalty(self, u: int, v: int, depth_m: float,
                              depth_image, original_depth_frame) -> float:
        """Compute edge penalty based on local depth variance."""
        if original_depth_frame is not None:
            # Sample neighbors
            vals = [depth_m]
            for dx, dy in [(3, 0), (-3, 0), (0, 3), (0, -3)]:
                d2 = self._sample_depth(
                    u + dx, v + dy, None, original_depth_frame)
                if d2:
                    vals.append(d2)

            if len(vals) > 1:
                depth_var = float(np.std(vals))
                return min(1.0, depth_var / 0.02)  # Normalize by 2cm variance
        elif depth_image is not None:
            # Use local patch
            try:
                h_orig, w_orig = depth_image.shape
                su = int(u * w_orig / 300.0)
                sv = int(v * h_orig / 300.0)

                ws = 5
                x0, x1 = max(0, su-ws), min(w_orig, su+ws)
                y0, y1 = max(0, sv-ws), min(h_orig, sv+ws)

                patch = depth_image[y0:y1, x0:x1]
                patch = patch[patch > 0]

                if patch.size > 0:
                    depth_var = float(np.std(patch))
                    return min(1.0, depth_var / 0.02)
            except Exception:
                pass

        return 0.2  # Default penalty if depth unknown

    def _convert_width_to_meters(self, width_pixels: float, depth_m: float,
                                 fx: Optional[float], depth_img_width: Optional[int]) -> Optional[float]:
        """Convert pixel width to meters using depth intrinsics."""
        if fx is None or depth_img_width is None or depth_m <= 0:
            return None

        fx_resized = fx * (300.0 / depth_img_width)
        return (width_pixels * depth_m) / max(fx_resized, 1e-6)

    def _compute_width_penalty(self, width_m: Optional[float],
                               grip_min: float, grip_max: float) -> float:
        """Compute width penalty based on gripper limits."""
        if width_m is None:
            return 0.05  # Small penalty if unknown

        if width_m < grip_min:
            penalty = (grip_min - width_m) / grip_min
        elif width_m > grip_max:
            penalty = (width_m - grip_max) / grip_max
        else:
            return 0.0  # Within limits

        return min(1.0, penalty)
