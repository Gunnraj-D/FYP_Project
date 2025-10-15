"""
Grasp postprocessing package.

Exports:
- GraspPostprocessor: Main interface (backward compatible)
- TemporalAngleFilter: Temporal consistency
- GraspCandidate: Data structure
- RobotiqGripperConfig: Gripper specifications
"""

import numpy as np
import cv2
import logging
from typing import Optional, Dict, List, Tuple

from config import GRASP_DETECTION_CONFIG, GRASP_EXECUTION_CONFIG, DEBUG_MODE

# Import from sub-modules
from .candidate_selection import (
    GraspCandidate,
    RobotiqGripperConfig,
    depth_foreground_mask,
    topk_local_maxima,
    topk_local_maxima_hybrid,
    compute_border_distance,
    compute_grasp_rectangle_overlap,
    normalize_grasp_angle,
    compute_pca_angle,
    compute_center_distance
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

        mode = GRASP_DETECTION_CONFIG.get('pca_align_mode', 'align')
        if mode == 'perpendicular':
            target = normalize_grasp_angle(pca_angle + np.pi/2)
        else:
            target = normalize_grasp_angle(pca_angle)

        candidates = [angle_raw, angle_raw + np.pi/2, angle_raw - np.pi/2]
        candidates = [normalize_grasp_angle(a) for a in candidates]

        def circ_dist(a, b):
            d = abs(a - b) % np.pi
            return min(d, np.pi - d)

        diffs = [circ_dist(a, target) for a in candidates]
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

    def __init__(self, camera_manager, temporal_filter: Optional[TemporalAngleFilter] = None, visualizer=None):
        """
        Args:
            camera_manager: CameraManager instance for intrinsics and depth sampling
            temporal_filter: Optional TemporalAngleFilter instance
        """
        self.camera_manager = camera_manager
        self.temporal_filter = temporal_filter or TemporalAngleFilter(
            enabled=False)
        self.visualizer = visualizer

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
        # More permissive defaults to recover weak signals
        self.nms_dilate = int(GRASP_DETECTION_CONFIG.get('nms_dilate_size', 5))
        self.nms_threshold = float(GRASP_DETECTION_CONFIG.get(
            'nms_min_threshold', 0.01))

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

        # Quality diagnostics (global and in-mask)
        try:
            logger.info(
                f"Quality map stats: min={q_np.min():.4f}, max={q_np.max():.4f}, "
                f"mean={q_np.mean():.4f}, median={np.median(q_np):.4f}")
            if object_mask.any():
                logger.info(
                    f"Quality in mask: max={q_np[object_mask].max():.4f}, "
                    f"mean={q_np[object_mask].mean():.4f}")
        except Exception:
            pass

        # Optional debug visualization of masks
        try:
            if self.visualizer is not None and DEBUG_MODE:
                self.visualizer.visualize_debug_masks(
                    depth_image if depth_image is not None else q_np, object_mask, q_np)
        except Exception:
            pass

        # Optional plane-based table suppression to avoid table being treated as object
        try:
            if not GRASP_DETECTION_CONFIG.get('use_plane_suppression', False):
                raise RuntimeError('disabled')
            if depth_image is not None and self.camera_manager is not None and self.camera_manager.is_ready():
                intr = self.camera_manager.aligned_color_intrinsics
                if intr:
                    # Scale intrinsics to 300x300
                    sx = 300.0 / float(intr.width)
                    sy = 300.0 / float(intr.height)
                    fx_s = intr.fx * sx
                    fy_s = intr.fy * sy
                    cx_s = intr.ppx * sx
                    cy_s = intr.ppy * sy

                    # Build subsampled 3D point cloud
                    zs = depth_image
                    h, w = zs.shape
                    step = 4
                    us, vs = np.meshgrid(
                        np.arange(0, w, step), np.arange(0, h, step))
                    z_samples = zs[vs, us]
                    valid = z_samples > 0
                    if np.count_nonzero(valid) >= 200:
                        us_v = us[valid].astype(np.float32)
                        vs_v = vs[valid].astype(np.float32)
                        z_v = z_samples[valid].astype(np.float32)
                        x_v = (us_v - cx_s) * z_v / fx_s
                        y_v = (vs_v - cy_s) * z_v / fy_s
                        pts = np.stack([x_v, y_v, z_v], axis=1)

                        # RANSAC plane fit
                        best_inliers = 0
                        best_n = None
                        best_d = None
                        N = pts.shape[0]
                        iters = 200
                        dist_thr = 0.01  # 1 cm
                        rng = np.random.default_rng(42)
                        for _ in range(iters):
                            idx = rng.choice(N, size=3, replace=False)
                            p1, p2, p3 = pts[idx]
                            v1 = p2 - p1
                            v2 = p3 - p1
                            n = np.cross(v1, v2)
                            norm = np.linalg.norm(n)
                            if norm < 1e-6:
                                continue
                            n = n / norm
                            d = -np.dot(n, p1)
                            dists = np.abs(pts @ n + d)
                            inliers = int(np.count_nonzero(dists <= dist_thr))
                            if inliers > best_inliers:
                                best_inliers = inliers
                                best_n = n
                                best_d = d

                        if best_n is not None and best_inliers >= 0.2 * N:
                            # Orient normal so deeper border points have positive distance
                            # Use border samples from full image
                            border_margin = max(6, min(h, w) // 12)
                            border_mask = np.zeros_like(zs, dtype=bool)
                            border_mask[:border_margin, :] = True
                            border_mask[-border_margin:, :] = True
                            border_mask[:, :border_margin] = True
                            border_mask[:, -border_margin:] = True
                            border_valid = border_mask & (zs > 0)
                            if np.count_nonzero(border_valid) > 50:
                                u_b, v_b = np.meshgrid(
                                    np.arange(w), np.arange(h))
                                u_b = u_b[border_valid].astype(np.float32)
                                v_b = v_b[border_valid].astype(np.float32)
                                z_b = zs[border_valid].astype(np.float32)
                                x_b = (u_b - cx_s) * z_b / fx_s
                                y_b = (v_b - cy_s) * z_b / fy_s
                                pts_b = np.stack([x_b, y_b, z_b], axis=1)
                                mean_border_sign = float(
                                    np.mean(pts_b @ best_n + best_d))
                                if mean_border_sign < 0:
                                    best_n = -best_n
                                    best_d = -best_d

                            # Compute distances for all pixels
                            u_full, v_full = np.meshgrid(
                                np.arange(w), np.arange(h))
                            z_full = zs
                            valid_full = z_full > 0
                            x_full = (u_full - cx_s) * z_full / fx_s
                            y_full = (v_full - cy_s) * z_full / fy_s
                            d_full = (
                                x_full * best_n[0] + y_full * best_n[1] + z_full * best_n[2] + best_d)

                            # Table mask: near plane
                            table_mask = np.abs(d_full) <= dist_thr

                            # Object mask: sufficiently above (closer to camera) than plane
                            # Use 10 mm offset as default
                            obj_offset = float(GRASP_DETECTION_CONFIG.get(
                                'object_above_table_offset_m', 0.010))
                            object_plane_mask = (
                                d_full < -obj_offset) & valid_full

                            # Combine: treat table as background, keep original foreground too
                            object_mask = (object_mask | object_plane_mask) & (
                                ~table_mask)
                            object_mask = object_mask.astype(bool)
        except Exception as e:
            logger.debug(f"Plane segmentation skipped: {e}")

        # Combine with quality-based mask to preserve high-quality edges
        if GRASP_DETECTION_CONFIG.get('use_quality_union', False):
            try:
                q_thr = max(self.nms_threshold, float(np.percentile(q_np, 60)))
                q_mask = (q_np >= q_thr)
                object_mask = np.logical_or(object_mask, q_mask)
            except Exception:
                pass

        # Interior mask using distance transform
        try:
            if not GRASP_DETECTION_CONFIG.get('use_interior_center_check', False):
                raise RuntimeError('disabled')
            edge_margin_px = int(
                GRASP_DETECTION_CONFIG.get('edge_margin_px', 4))
            dt = cv2.distanceTransform(
                (object_mask.astype(np.uint8) * 255), cv2.DIST_L2, 3)
            interior_mask = dt >= float(edge_margin_px)
            if interior_mask.sum() == 0:
                interior_mask = object_mask
        except Exception:
            interior_mask = object_mask
            dt = None

        # Boost quality inside object mask if configured
        if GRASP_DETECTION_CONFIG.get('boost_masked_quality', False):
            boost_factor = float(GRASP_DETECTION_CONFIG.get(
                'quality_boost_factor', 1.3))
            q_boosted = q_np.copy()
            needs_boost = (q_np < 0.5) & object_mask
            q_boosted[needs_boost] = np.clip(
                q_boosted[needs_boost] * boost_factor, 0.0, 0.95)
            logger.info(
                f"Quality boost: max={q_np.max():.3f} → {q_boosted.max():.3f}")
            q_np = q_boosted

        # NMS: Select local maxima (hybrid if available)
        k_actual = min(top_k, q_np.size)
        try:
            # width_np path aligns with ang/width decode upstream
            width_np = width_np
            top_indices = topk_local_maxima_hybrid(q_np, width_np, object_mask, k_actual,
                                                   dilate_size=self.nms_dilate,
                                                   min_thr=self.nms_threshold)
            if top_indices.size == 0:
                top_indices = topk_local_maxima(q_np, k_actual,
                                                dilate_size=self.nms_dilate,
                                                min_thr=self.nms_threshold)
        except Exception:
            top_indices = topk_local_maxima(q_np, k_actual,
                                            dilate_size=self.nms_dilate,
                                            min_thr=self.nms_threshold)

        def evaluate(use_interior: bool) -> Optional[GraspCandidate]:
            candidates = []
            for idx in top_indices:
                v, u = np.unravel_index(idx, q_np.shape)

                if use_interior and not interior_mask[v, u]:
                    continue

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

                px_to_mm = self._compute_px_to_mm(local_depth_m)
                quality = float(q_np[v, u])
                angle_rad_raw = float(ang_np[v, u])
                width_px = float(width_np[v, u])
                width_mm = width_px * px_to_mm

                cand_angles = [angle_rad_raw, angle_rad_raw +
                               np.pi/2, angle_rad_raw - np.pi/2]
                ov_scores = [
                    compute_grasp_rectangle_overlap(
                        u, v, normalize_grasp_angle(a), width_px, object_mask)
                    for a in cand_angles
                ]
                best_idx = int(np.argmax(ov_scores))
                angle_rad = normalize_grasp_angle(cand_angles[best_idx])

                border_dist = compute_border_distance(
                    u, v, q_np.shape, normalized=True)
                overlap = compute_grasp_rectangle_overlap(
                    u, v, angle_rad, width_px, object_mask)

                # Center distance and validation with slight relaxation near edge band
                center_dist = compute_center_distance(
                    u, v, object_mask, normalized=True)

                tmp_min_overlap = self.min_overlap
                try:
                    edge_band_px = int(
                        GRASP_DETECTION_CONFIG.get('edge_band_px', 6))
                    if dt is not None and dt[v, u] < float(edge_band_px):
                        self.min_overlap = max(
                            0.1, float(self.min_overlap) * 0.9)
                    is_valid, reason = self._validate_grasp(
                        width_mm, overlap, border_dist, center_dist)
                finally:
                    self.min_overlap = tmp_min_overlap

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
                    center_distance=center_dist,
                    is_valid=is_valid,
                    validity_reason=reason
                )
                candidates.append(candidate)

            # Debug: candidate counts and first few reasons
            try:
                logger.info(f"Postprocess candidates: total={len(candidates)}")
                for c in candidates[:5]:
                    logger.info(
                        f"  cand @({c.u},{c.v}) q={c.quality:.3f} ov={c.object_overlap:.2f} b={c.border_distance:.2f} w={c.width_mm:.1f}mm valid={c.is_valid} reason={c.validity_reason}")
            except Exception:
                pass

            return self._select_best_grasp(candidates)

        old_min_overlap = self.min_overlap
        old_border_thr = GRASP_DETECTION_CONFIG.get('border_threshold', 0.2)

        best_candidate = evaluate(use_interior=True)
        if best_candidate is None:
            best_candidate = evaluate(use_interior=False)
        if best_candidate is None:
            try:
                self.min_overlap = max(0.1, float(self.min_overlap) * 0.7)
                GRASP_DETECTION_CONFIG['border_threshold'] = max(
                    0.1, float(old_border_thr) * 0.8)
                best_candidate = evaluate(use_interior=False)
            finally:
                self.min_overlap = old_min_overlap
                GRASP_DETECTION_CONFIG['border_threshold'] = old_border_thr

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
                        border_dist: float, center_dist: float = 1.0) -> Tuple[bool, str]:
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

        # Overlap constraint (stricter if far from center)
        min_overlap_required = self.min_overlap
        if center_dist < 0.5:
            min_overlap_required *= 1.3
        if overlap < min_overlap_required:
            return False, f"Low overlap ({overlap:.2f} < {min_overlap_required:.2f})"

        # Border constraint
        border_thr = GRASP_DETECTION_CONFIG.get('border_threshold', 0.2)
        if border_dist < border_thr:
            return False, f"Too close to border ({border_dist:.2f})"

        # Prefer center grasps
        if center_dist < 0.3:
            return False, f"Too far from center ({center_dist:.2f})"

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
