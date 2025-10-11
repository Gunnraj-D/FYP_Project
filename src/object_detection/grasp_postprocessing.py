"""
ADVANCED Grasp candidate postprocessing with anti-tip fixes.

Major improvements from legacy version:
1. Local depth estimation (per-grasp, not global)
2. NMS with tunable aggressiveness
3. Object mask overlap checking
4. Border distance penalties
5. Camera intrinsics-based mm conversion
6. PCA-based angle correction (fixes 90° errors)
7. Sophisticated multi-factor scoring
8. Width preference for optimal gripper range
9. Temporal consistency scoring

This module implements all fixes from visualize_grconvnet_temporal.py
in a production-ready format for integration with the robot control system.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Dict, List, Tuple
from collections import deque
from dataclasses import dataclass

from config import GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE

logger = logging.getLogger(__name__)


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class RobotiqGripperConfig:
    """Robotiq 2F-85 gripper specifications."""
    min_opening_mm: float = 0.0
    max_opening_mm: float = 85.0
    min_grasp_width_mm: float = 5.0
    max_grasp_width_mm: float = 80.0
    optimal_grasp_range: Tuple[float, float] = (15.0, 60.0)


@dataclass
class GraspCandidate:
    """Extended grasp candidate with all anti-tip metrics."""
    u: int  # pixel x coordinate
    v: int  # pixel y coordinate
    angle_rad: float  # grasp angle in radians (PCA-corrected)
    angle_rad_raw: float  # raw angle from model (before PCA correction)
    quality: float  # quality score [0,1]
    width_px: float  # grasp width in pixels
    width_mm: float  # grasp width in mm (using local depth + intrinsics)
    local_depth_m: float  # local depth at grasp position
    object_overlap: float  # overlap with object mask [0,1]
    border_distance: float  # normalized distance from border [0,1]
    is_valid: bool  # passes all constraints
    validity_reason: str  # explanation if invalid
    combined_score: float = 0.0  # multi-factor score


# ============================================================================
# HELPER FUNCTIONS - GEOMETRY & MASKING
# ============================================================================

def depth_foreground_mask(depth_map_m: np.ndarray, bg_percentile: float = 80,
                          depth_diff_thresh: float = 0.02) -> np.ndarray:
    """
    Create foreground object mask from depth using thresholding.

    Args:
        depth_map_m: Depth map in meters (300x300)
        bg_percentile: Percentile to use for background estimation
        depth_diff_thresh: Depth difference threshold in meters

    Returns:
        Binary mask indicating foreground pixels
    """
    valid = depth_map_m[depth_map_m > 0]
    if valid.size == 0:
        return np.ones_like(depth_map_m, dtype=bool)

    # Background is farther away (higher percentile)
    bg = np.percentile(valid, bg_percentile)

    # Foreground is closer than background
    mask = (depth_map_m > 0) & (depth_map_m < (bg - depth_diff_thresh))

    # Morphological cleaning
    mask_uint8 = mask.astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    return mask_clean.astype(bool)


def topk_local_maxima(q_img: np.ndarray, k: int, dilate_size: int = 9,
                      min_thr: float = 0.03) -> np.ndarray:
    """
    Select top-k local maxima using NMS (prevents tip spikes).

    Args:
        q_img: Quality image (2D numpy array)
        k: Number of top candidates
        dilate_size: Dilation kernel size
        min_thr: Minimum quality threshold

    Returns:
        Flat indices of top-k local maxima
    """
    kernel = np.ones((dilate_size, dilate_size), np.uint8)
    dil = cv2.dilate(q_img, kernel)

    # Peaks are where original equals dilated
    peaks = (q_img == dil) & (q_img > min_thr)
    peak_idxs = np.flatnonzero(peaks.ravel())

    if peak_idxs.size == 0:
        # Fallback to global top-k
        return np.argpartition(q_img.ravel(), -k)[-k:]

    # Sort peaks by quality and take top k
    vals = q_img.ravel()[peak_idxs]
    pick = peak_idxs[np.argsort(-vals)][:k]

    return pick


def compute_border_distance(u: int, v: int, img_shape: Tuple[int, int],
                            normalized: bool = True) -> float:
    """
    Compute distance from image border.

    Args:
        u, v: Pixel coordinates
        img_shape: (height, width)
        normalized: Return normalized distance [0,1]

    Returns:
        Distance from nearest border
    """
    h, w = img_shape
    min_dist = min(v, h - v - 1, u, w - u - 1)

    if normalized:
        max_dist = min(h, w) / 2.0
        return min_dist / max_dist if max_dist > 0 else 0.0

    return min_dist


def compute_grasp_rectangle_overlap(u: int, v: int, angle_rad: float,
                                    width_px: float, mask: np.ndarray,
                                    finger_length: int = 40) -> float:
    """
    Compute overlap between grasp rectangle and object mask.

    Args:
        u, v: Grasp center pixel coordinates
        angle_rad: Grasp angle in radians
        width_px: Grasp width in pixels
        mask: Binary object mask
        finger_length: Gripper finger length in pixels

    Returns:
        Overlap ratio [0,1]
    """
    h, w = mask.shape
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    # Sample points along gripper fingers
    n_samples = 20
    overlap_count = 0
    total_count = 0

    for finger_side in [-1, 1]:
        for t in np.linspace(-width_px/2, width_px/2, n_samples):
            for l in np.linspace(0, finger_length, 5):
                px = u + t * cos_a + finger_side * l * (-sin_a)
                py = v + t * sin_a + finger_side * l * cos_a

                px_int = int(round(px))
                py_int = int(round(py))

                if 0 <= px_int < w and 0 <= py_int < h:
                    total_count += 1
                    if mask[py_int, px_int]:
                        overlap_count += 1

    return overlap_count / total_count if total_count > 0 else 0.0


# ============================================================================
# PCA-BASED ANGLE CORRECTION
# ============================================================================

def normalize_grasp_angle(angle: float) -> float:
    """Normalize angle to canonical range [-π/2, π/2)."""
    a = (angle + np.pi/2) % np.pi - np.pi/2
    return a


def pca_principal_angle(mask: np.ndarray) -> Optional[float]:
    """
    Compute principal axis angle using PCA on object mask.

    Args:
        mask: Binary object mask

    Returns:
        Principal angle in radians or None
    """
    ys, xs = np.nonzero(mask)
    if xs.size < 10:
        return None

    # Center coordinates
    x = xs.astype(np.float32) - xs.mean()
    y = ys.astype(np.float32) - ys.mean()
    coords = np.stack([x, y], axis=0)

    # Compute covariance and eigenvectors
    cov = np.cov(coords)
    evals, evecs = np.linalg.eig(cov)

    # Principal eigenvector
    idx = np.argmax(evals)
    vx, vy = evecs[:, idx].real

    angle = np.arctan2(vy, vx)
    return float(angle)


def best_angle_mapping(angle_pred: float, mask: np.ndarray,
                       debug: bool = False) -> float:
    """
    Resolve angle convention mismatches using PCA.

    Automatically fixes:
    - 90° offsets (jaw-axis vs approach-axis)
    - Sign flips (image coordinate system)

    Args:
        angle_pred: Predicted angle from model
        mask: Binary object mask
        debug: Return debug info if True

    Returns:
        Corrected angle in [-π/2, π/2)
    """
    pca_angle = pca_principal_angle(mask)

    if pca_angle is None:
        return normalize_grasp_angle(angle_pred)

    # Try 4 common convention mappings
    candidates = [
        angle_pred,
        angle_pred + np.pi/2,
        -angle_pred,
        -angle_pred + np.pi/2
    ]

    candidates = [normalize_grasp_angle(a) for a in candidates]
    pca_norm = normalize_grasp_angle(pca_angle)

    def circular_dist(a, b):
        d = abs(a - b) % np.pi
        return min(d, np.pi - d)

    dists = [circular_dist(c, pca_norm) for c in candidates]
    best_idx = int(np.argmin(dists))
    best = candidates[best_idx]

    if debug:
        return best, candidates, dists, pca_norm, best_idx

    return best


# ============================================================================
# MULTI-FACTOR SCORING
# ============================================================================

def width_score_mm(width_mm: float, gripper: RobotiqGripperConfig) -> float:
    """
    Map width to [0,1] score, peaking in optimal range.

    Args:
        width_mm: Grasp width in millimeters
        gripper: Gripper configuration

    Returns:
        Score in [0,1]
    """
    lo, hi = gripper.optimal_grasp_range

    if width_mm <= gripper.min_grasp_width_mm or width_mm >= gripper.max_grasp_width_mm:
        return 0.0

    center = (lo + hi) / 2.0
    max_span = max(center - gripper.min_grasp_width_mm,
                   gripper.max_grasp_width_mm - center, 1e-3)
    score = 1.0 - abs(width_mm - center) / max_span

    return float(np.clip(score, 0.0, 1.0))


def temporal_score(angle_rad: float, recent_angle_rad: Optional[float]) -> float:
    """
    Score temporal consistency with recent angle.

    Args:
        angle_rad: Current angle
        recent_angle_rad: Recent filtered angle (None = no history)

    Returns:
        Score in [0,1], 1.0 = identical
    """
    if recent_angle_rad is None:
        return 1.0

    d = abs(angle_rad - recent_angle_rad) % np.pi
    d = min(d, np.pi - d)

    # Cosine-shaped falloff
    return float(max(0.0, np.cos((d / (np.pi/2.0)) * (np.pi/2.0))))


# ============================================================================
# TEMPORAL FILTERING
# ============================================================================

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


# ============================================================================
# ADVANCED GRASP POSTPROCESSOR
# ============================================================================

class GraspPostprocessor:
    """
    ADVANCED grasp candidate postprocessor with anti-tip fixes.

    Features (from visualize_grconvnet_temporal.py):
    1. Local depth estimation (not global median)
    2. NMS with adjustable aggressiveness
    3. Object mask overlap checking
    4. Border distance penalties
    5. Camera intrinsics for mm conversion
    6. PCA-based angle correction
    7. Sophisticated multi-factor scoring
    8. Width preference for optimal range
    9. Temporal consistency tracking

    Configuration via GRASP_DETECTION_CONFIG:
    - 'use_advanced_postprocessing': Enable all new features (default True)
    - 'width_multiplier': Calibrated width scaling (default 95.0 for screwdrivers)
    - 'min_overlap': Minimum object overlap (default 0.25)
    - 'use_pca_angle_correction': Enable PCA correction (default True)
    - 'nms_dilate_size': NMS kernel size (default 9)
    - 'scoring_weights': Dict of weights for multi-factor scoring
    """

    def __init__(self, camera_manager, temporal_filter: Optional[TemporalAngleFilter] = None):
        """
        Args:
            camera_manager: CameraManager instance for intrinsics and depth sampling
            temporal_filter: Optional TemporalAngleFilter instance
        """
        self.camera_manager = camera_manager
        self.temporal_filter = temporal_filter or TemporalAngleFilter(
            enabled=False)

        # Gripper configuration
        self.gripper = RobotiqGripperConfig(
            min_grasp_width_mm=GRASP_EXECUTION_CONFIG.get(
                'gripper_min_width_m', 0.005) * 1000,
            max_grasp_width_mm=GRASP_EXECUTION_CONFIG.get(
                'gripper_max_width_m', 0.080) * 1000,
            optimal_grasp_range=(
                GRASP_EXECUTION_CONFIG.get('gripper_optimal_min_mm', 15.0),
                GRASP_EXECUTION_CONFIG.get('gripper_optimal_max_mm', 60.0)
            )
        )

        # Advanced features configuration
        self.use_advanced = GRASP_DETECTION_CONFIG.get(
            'use_advanced_postprocessing', True)
        self.width_multiplier = GRASP_DETECTION_CONFIG.get(
            'width_multiplier', 95.0)
        self.min_overlap = GRASP_DETECTION_CONFIG.get('min_overlap', 0.25)
        self.use_pca_correction = GRASP_DETECTION_CONFIG.get(
            'use_pca_angle_correction', True)
        self.nms_dilate = GRASP_DETECTION_CONFIG.get('nms_dilate_size', 9)
        self.nms_threshold = GRASP_DETECTION_CONFIG.get(
            'nms_min_threshold', 0.03)

        # Scoring weights
        default_weights = {'q': 1.0, 'o': 1.2, 'b': 0.5, 'w': 0.7, 't': 0.8}
        self.scoring_weights = GRASP_DETECTION_CONFIG.get(
            'scoring_weights', default_weights)

        logger.info(f"✨ ADVANCED GraspPostprocessor initialized:")
        logger.info(f"   Width multiplier: {self.width_multiplier}")
        logger.info(f"   Min overlap: {self.min_overlap}")
        logger.info(f"   PCA angle correction: {self.use_pca_correction}")
        logger.info(
            f"   NMS: dilate={self.nms_dilate}, threshold={self.nms_threshold}")
        logger.info(f"   Scoring weights: {self.scoring_weights}")

    def postprocess(self, q_img, ang_img, width_img,
                    depth_image=None, original_depth_frame=None,
                    top_k: int = 15) -> Optional[Dict]:
        """
        Select best grasp using advanced anti-tip logic.

        Args:
            q_img: Quality map tensor (B, 1, H, W)
            ang_img: Angle map tensor (B, 1, H, W)
            width_img: Width map tensor (B, 1, H, W)
            depth_image: Numpy depth array in meters (resized 300x300)
            original_depth_frame: RealSense frame for accurate depth sampling
            top_k: Number of candidates to analyze

        Returns:
            Dict with best grasp or None
        """
        # Convert to numpy
        q_np = q_img.squeeze().cpu().numpy()
        ang_np = ang_img.squeeze().cpu().numpy()
        width_np = width_img.squeeze().cpu().numpy()

        if self.use_advanced:
            return self._postprocess_advanced(
                q_np, ang_np, width_np, depth_image, original_depth_frame, top_k
            )
        else:
            # Legacy mode (for comparison/debugging)
            return self._postprocess_legacy(
                q_np, ang_np, width_np, depth_image, original_depth_frame, top_k
            )

    def _postprocess_advanced(self, q_np: np.ndarray, ang_np: np.ndarray,
                              width_np: np.ndarray, depth_image: Optional[np.ndarray],
                              original_depth_frame, top_k: int) -> Optional[Dict]:
        """
        ADVANCED postprocessing with all anti-tip fixes.
        """
        # Create object mask from depth
        if depth_image is not None:
            object_mask = depth_foreground_mask(
                depth_image,
                bg_percentile=GRASP_DETECTION_CONFIG.get('bg_percentile', 80),
                depth_diff_thresh=GRASP_DETECTION_CONFIG.get(
                    'depth_diff_thresh', 0.02)
            )
            valid_depth = depth_image[depth_image > 0]
            median_depth_m = float(np.median(valid_depth)
                                   ) if valid_depth.size > 0 else 0.5
        else:
            object_mask = np.ones_like(q_np, dtype=bool)
            median_depth_m = 0.5

        # NMS: Select local maxima (prevents tip spikes)
        k_actual = min(top_k, q_np.size)
        top_indices = topk_local_maxima(q_np, k_actual,
                                        dilate_size=self.nms_dilate,
                                        min_thr=self.nms_threshold)

        # Analyze all candidates
        candidates = []
        for idx in top_indices:
            v, u = np.unravel_index(idx, q_np.shape)

            # Local depth estimation (fixes tip problem!)
            if depth_image is not None:
                win = 4
                v0, v1 = max(
                    0, v - win), min(depth_image.shape[0], v + win + 1)
                u0, u1 = max(
                    0, u - win), min(depth_image.shape[1], u + win + 1)
                local = depth_image[v0:v1, u0:u1]
                local_valid = local[local > 0]
                local_depth_m = float(
                    np.median(local_valid)) if local_valid.size > 0 else median_depth_m
            else:
                local_depth_m = median_depth_m

            # Convert width using camera intrinsics
            px_to_mm = self._compute_px_to_mm(local_depth_m)

            quality = float(q_np[v, u])
            angle_rad_raw = float(ang_np[v, u])
            width_px = float(width_np[v, u])
            width_mm = width_px * px_to_mm

            # PCA-based angle correction
            if self.use_pca_correction:
                angle_rad = best_angle_mapping(angle_rad_raw, object_mask)
            else:
                angle_rad = normalize_grasp_angle(angle_rad_raw)

            # Compute metrics
            border_dist = compute_border_distance(
                u, v, q_np.shape, normalized=True)
            overlap = compute_grasp_rectangle_overlap(
                u, v, angle_rad, width_px, object_mask)

            # Validate
            is_valid, reason = self._validate_grasp(
                width_mm, overlap, border_dist)

            candidate = GraspCandidate(
                u=int(u), v=int(v),
                angle_rad=angle_rad,
                angle_rad_raw=angle_rad_raw,
                quality=quality,
                width_px=width_px,
                width_mm=width_mm,
                local_depth_m=local_depth_m,
                object_overlap=overlap,
                border_distance=border_dist,
                is_valid=is_valid,
                validity_reason=reason
            )
            candidates.append(candidate)

        # Select best using multi-factor scoring
        best_candidate = self._select_best_grasp(candidates)

        if best_candidate is None:
            logger.warning("No valid grasp found after advanced scoring")
            return None

        # Apply temporal filtering
        raw_angle = best_candidate.angle_rad
        filtered_angle = self.temporal_filter.filter(raw_angle)

        if self.temporal_filter.enabled:
            logger.info(f"📐 Angle: raw={np.degrees(raw_angle):.1f}° → "
                        f"filtered={np.degrees(filtered_angle):.1f}°")

        # Convert to output format
        return {
            'center': (best_candidate.u, best_candidate.v),
            'angle': filtered_angle,
            'width': best_candidate.width_px,
            'quality': best_candidate.quality,
            'score': best_candidate.combined_score,
            'depth_m': best_candidate.local_depth_m,
            'width_m': best_candidate.width_mm / 1000.0,  # Convert to meters
            'object_overlap': best_candidate.object_overlap,
            'border_distance': best_candidate.border_distance
        }

    def _compute_px_to_mm(self, depth_m: float) -> float:
        """Compute pixel-to-mm ratio using camera intrinsics."""
        if self.camera_manager is None or not self.camera_manager.is_ready():
            # Fallback to FOV-based (less accurate)
            fov_rad = np.deg2rad(69.4)
            horizontal_extent_mm = 2 * depth_m * np.tan(fov_rad / 2) * 1000
            return horizontal_extent_mm / 300.0

        try:
            mm_per_px_x, mm_per_px_y = self.camera_manager.compute_pixel_to_mm_at_depth(
                depth_m, image_width=300, image_height=300
            )
            return (mm_per_px_x + mm_per_px_y) / 2.0
        except Exception as e:
            logger.warning(f"Failed to compute intrinsics-based px→mm: {e}")
            # Fallback
            fov_rad = np.deg2rad(69.4)
            horizontal_extent_mm = 2 * depth_m * np.tan(fov_rad / 2) * 1000
            return horizontal_extent_mm / 300.0

    def _validate_grasp(self, width_mm: float, overlap: float,
                        border_dist: float) -> Tuple[bool, str]:
        """
        Validate grasp against multiple constraints.

        Returns:
            (is_valid, reason_string)
        """
        # Width constraints
        if width_mm < self.gripper.min_grasp_width_mm:
            return False, f"Too narrow ({width_mm:.1f}mm < {self.gripper.min_grasp_width_mm}mm)"
        elif width_mm > self.gripper.max_grasp_width_mm:
            return False, f"Too wide ({width_mm:.1f}mm > {self.gripper.max_grasp_width_mm}mm)"

        # Overlap constraint
        if overlap < self.min_overlap:
            return False, f"Low overlap ({overlap:.2f} < {self.min_overlap})"

        # Border constraint
        if border_dist < 0.2:
            return False, f"Too close to border ({border_dist:.2f})"

        # All constraints passed
        if self.gripper.optimal_grasp_range[0] <= width_mm <= self.gripper.optimal_grasp_range[1]:
            return True, f"Optimal ({width_mm:.1f}mm)"
        else:
            return True, f"Valid ({width_mm:.1f}mm)"

    def _select_best_grasp(self, candidates: List[GraspCandidate]) -> Optional[GraspCandidate]:
        """
        Select best grasp using sophisticated multi-factor scoring.

        Formula: score = (Q^w_q) × (O^w_o) × (B^w_b) × (W^w_w) × (T^w_t) + ε
        """
        valid_candidates = [c for c in candidates if c.is_valid]

        if not valid_candidates:
            logger.warning(
                f"No valid candidates (checked {len(candidates)} total)")
            if DEBUG_MODE and candidates:
                logger.info("Invalid reasons:")
                for c in candidates[:3]:
                    logger.info(f"  @({c.u},{c.v}): {c.validity_reason}")
            return None

        # Get recent angle for temporal scoring
        recent_angle = self.temporal_filter.get_recent_angle()

        # Score all valid candidates
        eps = 1e-8
        w = self.scoring_weights

        for cand in valid_candidates:
            # Individual factor scores
            q_score = float(np.clip(cand.quality, 0.0, 1.0))
            o_score = float(np.clip(cand.object_overlap, 0.0, 1.0))
            b_score = float(np.clip(cand.border_distance, 0.0, 1.0))
            w_score = width_score_mm(cand.width_mm, self.gripper)
            t_score = temporal_score(cand.angle_rad, recent_angle)

            # Multiplicative scoring (poor performance in any factor heavily penalizes)
            score = (q_score ** w['q']) * \
                    (o_score ** w['o']) * \
                    (b_score ** w['b']) * \
                    (w_score ** w['w']) * \
                    (t_score ** w['t']) + eps

            cand.combined_score = score

        # Sort by score and select best
        valid_candidates.sort(key=lambda c: c.combined_score, reverse=True)
        best = valid_candidates[0]

        # Debug logging
        if DEBUG_MODE and len(valid_candidates) > 1:
            logger.info(
                f"📊 Top {min(3, len(valid_candidates))} candidates (multi-factor scoring):")
            for i, c in enumerate(valid_candidates[:3]):
                marker = "👑" if i == 0 else f" {i+1}."
                w_s = width_score_mm(c.width_mm, self.gripper)
                t_s = temporal_score(c.angle_rad, recent_angle)
                logger.info(
                    f"  {marker} Score={c.combined_score:.4f}: "
                    f"@({c.u},{c.v}) Q={c.quality:.3f} O={c.object_overlap:.2f} "
                    f"B={c.border_distance:.2f} W={w_s:.2f} T={t_s:.2f} | "
                    f"{c.width_mm:.1f}mm ∠{np.degrees(c.angle_rad):.1f}°"
                )

        return best

    def _postprocess_legacy(self, q_np: np.ndarray, ang_np: np.ndarray,
                            width_np: np.ndarray, depth_image: Optional[np.ndarray],
                            original_depth_frame, top_k: int) -> Optional[Dict]:
        """
        Legacy postprocessing (for backward compatibility/comparison).

        Uses simple quality-distance-edge-width scoring.
        """
        # Smooth quality map
        q_blur = cv2.GaussianBlur(q_np, (5, 5), 2)

        # Find local maxima
        thresh = max(GRASP_DETECTION_CONFIG.get(
            'min_quality_threshold', 0.15), 0.02)
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(q_blur, kernel)
        local_max_mask = (q_blur == dilated) & (q_blur > thresh)

        if local_max_mask.sum() == 0:
            flat_idxs = np.argpartition(q_blur.flatten(), -top_k)[-top_k:]
            cand_rows, cand_cols = np.unravel_index(flat_idxs, q_blur.shape)
        else:
            cand_rows, cand_cols = np.where(local_max_mask)
            if len(cand_rows) > top_k:
                qualities = q_blur[cand_rows, cand_cols]
                order = np.argsort(qualities)[-top_k:]
                cand_rows, cand_cols = cand_rows[order], cand_cols[order]

        candidates = list(zip(cand_rows, cand_cols))

        if not candidates:
            return None

        # Simple scoring (legacy method)
        best_score = -1e9
        best = None

        for r, c in candidates:
            quality = float(q_blur[r, c])
            score = quality  # Simple quality-only for legacy

            if score > best_score:
                best_score = score
                best = {
                    'center': (int(c), int(r)),
                    'angle': float(ang_np[r, c]),
                    'width': float(width_np[r, c]),
                    'quality': quality,
                    'score': score,
                    'depth_m': median_depth_m if depth_image is not None else 0.5,
                    # Rough conversion
                    'width_m': float(width_np[r, c]) * 0.001
                }

        return best
