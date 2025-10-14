"""
Grasp postprocessing package.

Exports:
- GraspPostprocessor: Main interface (backward compatible)
- TemporalAngleFilter: Temporal consistency
- GraspCandidate: Data structure
- RobotiqGripperConfig: Gripper specifications
"""

import numpy as np
import logging
from typing import Optional, Dict, List, Tuple

from config import GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE

# Import from sub-modules
from .candidate_selection import (
    GraspCandidate,
    RobotiqGripperConfig,
    depth_foreground_mask,
    topk_local_maxima,
    compute_border_distance,
    compute_grasp_rectangle_overlap,
    normalize_grasp_angle,
    compute_pca_angle
)
from .scoring import (
    GraspScorer,
    width_score_mm,
    temporal_score
)
from .temporal_filter import TemporalAngleFilter

logger = logging.getLogger(__name__)


# ============================================================================
# HELPER FUNCTION FOR PCA ANGLE CORRECTION
# ============================================================================

def best_angle_mapping(angle_raw: float, object_mask: np.ndarray,
                       depth_map: Optional[np.ndarray] = None,
                       u: Optional[int] = None, v: Optional[int] = None,
                       width_px: float = 50.0) -> float:
    """
    Choose best angle between raw and raw+90° using PCA.

    This wrapper provides backward compatibility with original API.
    """
    if depth_map is not None and u is not None and v is not None:
        pca_angle = compute_pca_angle(depth_map, object_mask, u, v, width_px)

        # Choose closest to PCA
        candidates = [angle_raw, angle_raw + np.pi/2, angle_raw - np.pi/2]
        candidates = [normalize_grasp_angle(a) for a in candidates]

        diffs = [abs(normalize_grasp_angle(a - pca_angle)) for a in candidates]
        best_idx = np.argmin(diffs)

        return candidates[best_idx]
    else:
        # Fallback to raw angle
        return normalize_grasp_angle(angle_raw)


# ============================================================================
# MAIN POSTPROCESSOR (BACKWARD COMPATIBLE)
# ============================================================================

class GraspPostprocessor:
    """
    Grasp candidate postprocessor with multi-factor scoring.

    Features:
    1. Local depth estimation (not global median)
    2. NMS with adjustable aggressiveness
    3. Object mask overlap checking
    4. Border distance penalties
    5. Camera intrinsics for mm conversion
    6. PCA-based angle correction
    7. Multi-factor scoring
    8. Width preference for optimal range
    9. Temporal consistency tracking

    Configuration via GRASP_DETECTION_CONFIG.
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

        # Create scorer
        self.scorer = GraspScorer(
            scoring_weights=self.scoring_weights, gripper=self.gripper)

        logger.info(f"GraspPostprocessor initialized (refactored):")
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
        Select best grasp using multi-factor scoring.

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

        return self._postprocess_advanced(
            q_np, ang_np, width_np, depth_image, original_depth_frame, top_k
        )

    def _postprocess_advanced(self, q_np: np.ndarray, ang_np: np.ndarray,
                              width_np: np.ndarray, depth_image: Optional[np.ndarray],
                              original_depth_frame, top_k: int) -> Optional[Dict]:
        """Advanced postprocessing with all features."""
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

        # NMS: Select local maxima
        k_actual = min(top_k, q_np.size)
        top_indices = topk_local_maxima(q_np, k_actual,
                                        dilate_size=self.nms_dilate,
                                        min_thr=self.nms_threshold)

        # Analyze all candidates
        candidates = []
        for idx in top_indices:
            v, u = np.unravel_index(idx, q_np.shape)

            # Local depth estimation
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
            if self.use_pca_correction and depth_image is not None:
                angle_rad = best_angle_mapping(angle_rad_raw, object_mask,
                                               depth_image, u, v, width_px)
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
            logger.warning("No valid grasp found after scoring")
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
        """Select best grasp using multi-factor scoring."""
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

        # Use scorer to select best
        best = self.scorer.select_best(valid_candidates, recent_angle)

        # Debug logging
        if DEBUG_MODE and len(valid_candidates) > 1:
            logger.info(f"📊 Top {min(3, len(valid_candidates))} candidates:")
            for i, c in enumerate(valid_candidates[:3]):
                marker = "👑" if i == 0 else f" {i+1}."
                logger.info(
                    f"  {marker} Score={c.combined_score:.4f}: "
                    f"@({c.u},{c.v}) Q={c.quality:.3f} O={c.object_overlap:.2f} "
                    f"B={c.border_distance:.2f} | "
                    f"{c.width_mm:.1f}mm ∠{np.degrees(c.angle_rad):.1f}°"
                )

        return best


# ============================================================================
# EXPORTS
# ============================================================================

__all__ = [
    'GraspPostprocessor',      # Main interface (backward compatible)
    'TemporalAngleFilter',     # Temporal filtering
    'GraspCandidate',          # Data structure
    'RobotiqGripperConfig',    # Gripper config
    'GraspScorer',             # Advanced: direct access to scorer
    'depth_foreground_mask',   # Advanced: helper functions
    'topk_local_maxima',
    'compute_border_distance',
    'compute_grasp_rectangle_overlap',
    'normalize_grasp_angle',
    'width_score_mm',
    'temporal_score',
]
