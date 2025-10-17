"""
Multi-factor grasp scoring.

Combines multiple quality metrics:
- Network quality score
- Object overlap
- Border distance
- Width preference
- Temporal consistency
"""

import numpy as np
from typing import Optional, Dict
from .candidate_selection import GraspCandidate, RobotiqGripperConfig


# ============================================================================
# SCORING WEIGHT CONFIGURATION
# ============================================================================

def get_default_weights() -> Dict[str, float]:
    """Get default scoring weights."""
    return {
        'q': 1.0,   # Quality (baseline importance)
        'o': 1.5,   # Overlap (STRONG emphasis for anti-tip)
        'b': 0.5,   # Border (less critical if other factors good)
        'w': 0.7,   # Width preference (optimal range)
        't': 0.0,   # Temporal consistency (DISABLED for now)
        'c': 2.0    # Center distance (STRONG bias toward center grasps)
    }


# ============================================================================
# SCORING FUNCTIONS
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
# MULTI-FACTOR SCORER
# ============================================================================

class GraspScorer:
    """Scores grasp candidates using multiple factors."""

    def __init__(self,
                 scoring_weights: Optional[Dict[str, float]] = None,
                 gripper: Optional[RobotiqGripperConfig] = None):
        """
        Initialize scorer.

        Args:
            scoring_weights: Custom weights (or None for defaults)
            gripper: Gripper config (or None for default Robotiq 2F-85)
        """
        self.scoring_weights = scoring_weights or get_default_weights()
        self.gripper = gripper or RobotiqGripperConfig()

    def score_candidate(self, candidate: GraspCandidate,
                        recent_angle: Optional[float] = None) -> float:
        """
        Compute multi-factor score for single candidate.

        Formula: score = (Q^w_q) × (O^w_o) × (B^w_b) × (W^w_w) × (T^w_t) + ε

        Args:
            candidate: Grasp candidate to score
            recent_angle: Recent filtered angle for temporal scoring

        Returns:
            Combined score (higher is better)
        """
        eps = 1e-8
        w = self.scoring_weights

        # Individual factor scores [0,1]
        q_score = float(np.clip(candidate.quality, 0.0, 1.0))
        o_score = float(np.clip(candidate.object_overlap, 0.0, 1.0))
        b_score = float(np.clip(candidate.border_distance, 0.0, 1.0))
        w_score = width_score_mm(candidate.width_mm, self.gripper)
        t_score = temporal_score(candidate.angle_rad, recent_angle)

        # Multiplicative scoring (poor performance in any factor heavily penalizes)
        score = (q_score ** w['q']) * \
                (o_score ** w['o']) * \
                (b_score ** w['b']) * \
                (w_score ** w['w']) * \
                (t_score ** w['t']) * \
                (float(np.clip(candidate.center_distance, 0.0, 1.0))
                 ** w.get('c', 1.0)) + eps

        return score

    def score_candidates(self, candidates: list,
                         recent_angle: Optional[float] = None) -> list:
        """
        Compute combined scores for all candidates.

        Args:
            candidates: List of GraspCandidate objects
            recent_angle: Recent filtered angle for temporal scoring

        Returns:
            Same list with combined_score field populated, sorted by score
        """
        for candidate in candidates:
            candidate.combined_score = self.score_candidate(
                candidate, recent_angle)

        # Sort by score (highest first)
        candidates.sort(key=lambda c: c.combined_score, reverse=True)
        return candidates

    def select_best(self, candidates: list,
                    recent_angle: Optional[float] = None) -> Optional[GraspCandidate]:
        """
        Select best grasp from candidates using multi-factor scoring.

        Args:
            candidates: List of GraspCandidate objects
            recent_angle: Recent filtered angle for temporal scoring

        Returns:
            Best candidate or None if no valid candidates
        """
        # Filter to valid only
        valid_candidates = [c for c in candidates if c.is_valid]

        if not valid_candidates:
            return None

        # Score and sort
        scored = self.score_candidates(valid_candidates, recent_angle)

        return scored[0]  # Best candidate
