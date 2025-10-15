"""
Grasp candidate selection and filtering.

Handles:
- NMS (Non-Maximum Suppression) for local maxima
- Foreground object masking
- Overlap checking
- Border distance penalties
- PCA-based angle correction
"""

import cv2
import numpy as np
import logging
from typing import Tuple
from dataclasses import dataclass

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
    """Grasp candidate with metrics."""
    u: int                      # pixel x
    v: int                      # pixel y
    angle_rad: float            # grasp angle (PCA-corrected)
    angle_rad_raw: float        # raw angle from model
    quality: float              # quality score [0,1]
    width_px: float             # width in pixels
    width_mm: float             # width in mm
    local_depth_m: float        # local depth
    object_overlap: float       # overlap [0,1]
    border_distance: float      # border distance [0,1]
    center_distance: float = 1.0  # distance from mask centroid [0,1], 1=center
    is_valid: bool = True
    validity_reason: str = ""
    combined_score: float = 0.0


# ============================================================================
# HELPER FUNCTIONS - GEOMETRY & MASKING
# ============================================================================

def depth_foreground_mask(depth_map_m: np.ndarray,
                          bg_percentile: float = 80,
                          depth_diff_thresh: float = 0.02) -> np.ndarray:
    """Create foreground object mask from depth using border-based background estimate."""
    h, w = depth_map_m.shape
    depth_valid = depth_map_m > 0
    if not depth_valid.any():
        return np.ones_like(depth_map_m, dtype=bool)

    # Estimate background from a border ring to avoid large foreground bias
    margin = max(6, min(h, w) // 12)  # ~8% of image size
    border_mask = np.zeros_like(depth_valid, dtype=bool)
    border_mask[:margin, :] = True
    border_mask[-margin:, :] = True
    border_mask[:, :margin] = True
    border_mask[:, -margin:] = True

    border_valid = depth_valid & border_mask
    if border_valid.sum() >= 50:
        bg_samples = depth_map_m[border_valid]
    else:
        bg_samples = depth_map_m[depth_valid]

    # Background is farther (larger depth)
    try:
        bg = float(np.percentile(bg_samples, bg_percentile))
    except Exception:
        bg = float(np.median(bg_samples))

    # Foreground is closer than background by threshold
    mask = depth_valid & (depth_map_m < (bg - depth_diff_thresh))

    # Morphological cleaning
    mask_uint8 = mask.astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    return mask_clean.astype(bool)


def topk_local_maxima(q_img: np.ndarray, k: int,
                      dilate_size: int = 9,
                      min_thr: float = 0.03) -> np.ndarray:
    """Select top-k local maxima using NMS."""
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


def topk_local_maxima_hybrid(q_img: np.ndarray, width_img: np.ndarray,
                             mask: np.ndarray, k: int,
                             dilate_size: int = 9,
                             min_thr: float = 0.03) -> np.ndarray:
    """
    Hybrid selection using quality + width + mask to preserve candidates when quality is noisy.
    """
    width_valid = (width_img > 10) & (width_img < 150)
    hybrid_score = q_img * mask.astype(float) * width_valid.astype(float)

    kernel = np.ones((dilate_size, dilate_size), np.uint8)
    dil = cv2.dilate(hybrid_score, kernel)
    peaks = (hybrid_score == dil) & (hybrid_score > min_thr * 0.5)

    peak_idxs = np.flatnonzero(peaks.ravel())
    if peak_idxs.size == 0:
        mask_center = np.argwhere(mask)
        if len(mask_center) > 0:
            center = mask_center.mean(axis=0).astype(int)
            r_flat = int(center[0]) * mask.shape[1] + int(center[1])
            return np.array([r_flat])
        return np.array([])

    vals = hybrid_score.ravel()[peak_idxs]
    pick = peak_idxs[np.argsort(-vals)][:k]
    return pick


def compute_border_distance(u: int, v: int, img_shape: Tuple[int, int],
                            normalized: bool = True) -> float:
    """Compute distance from image border."""
    h, w = img_shape
    min_dist = min(v, h - v - 1, u, w - u - 1)

    if normalized:
        max_dist = min(h, w) / 2.0
        return min_dist / max_dist if max_dist > 0 else 0.0

    return min_dist


def compute_grasp_rectangle_overlap(u: int, v: int, angle_rad: float,
                                    width_px: float, mask: np.ndarray,
                                    finger_length: int = 40) -> float:
    """Compute overlap between grasp rectangle and object mask."""
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


def compute_center_distance(u: int, v: int, mask: np.ndarray,
                            normalized: bool = True) -> float:
    """
    Compute distance score from mask centroid: 1.0 at center, 0.0 near edges.
    """
    coords = np.argwhere(mask)
    if coords.shape[0] < 10:
        return 0.5

    center_v, center_u = coords.mean(axis=0)
    dist = np.sqrt((u - center_u) ** 2 + (v - center_v) ** 2)

    max_dist = np.sqrt(
        ((coords[:, 0] - center_v) ** 2 + (coords[:, 1] - center_u) ** 2).max())
    if max_dist < 1:
        return 1.0

    score = 1.0 - (dist / max_dist)
    return float(np.clip(score, 0.0, 1.0))


# ============================================================================
# PCA-BASED ANGLE CORRECTION
# ============================================================================

def normalize_grasp_angle(angle: float) -> float:
    """Normalize angle to [-π/2, π/2]."""
    while angle > np.pi / 2:
        angle -= np.pi
    while angle < -np.pi / 2:
        angle += np.pi
    return angle


def compute_pca_angle(depth_map_m: np.ndarray, mask: np.ndarray,
                      u: int, v: int, width_px: float) -> float:
    """
    Compute principal component angle from local object points.

    Helps correct 90° rotation errors on cylindrical objects.
    """
    # Extract local region around grasp
    window_size = int(max(20, width_px * 1.5))
    u_min = max(0, u - window_size)
    u_max = min(mask.shape[1], u + window_size)
    v_min = max(0, v - window_size)
    v_max = min(mask.shape[0], v + window_size)

    local_mask = mask[v_min:v_max, u_min:u_max]
    local_depth = depth_map_m[v_min:v_max, u_min:u_max]

    # Get foreground points
    fg_points = np.column_stack(np.where(local_mask & (local_depth > 0)))

    if fg_points.shape[0] < 5:
        return 0.0  # Not enough points for PCA

    # Perform PCA
    centered = fg_points - fg_points.mean(axis=0)
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)

    # Principal component (major axis)
    principal_idx = np.argmax(eigenvalues)
    principal_vec = eigenvectors[:, principal_idx]

    # Convert to angle (note: image coordinates are (y,x))
    angle = np.arctan2(principal_vec[1], principal_vec[0])

    return normalize_grasp_angle(angle)
