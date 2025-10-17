"""
Enhanced Hand Occlusion Detector using multi-factor scoring.

This module implements the sophisticated occlusion detection strategy from
occlusion_strategy.md with weighted indicators and temporal smoothing.

Features:
- Multi-factor weighted scoring (0.0-1.0)
- Temporal smoothing with sliding window
- Adaptive Z-filtering integration
- Gradual quality degradation detection
"""
import time
import logging
import numpy as np
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class OcclusionLevel(Enum):
    """Occlusion severity levels based on score ranges."""
    CLEAR = "clear"           # 0.0 - 0.2
    MINOR = "minor"           # 0.2 - 0.4
    MODERATE = "moderate"     # 0.4 - 0.6
    SEVERE = "severe"         # 0.6 - 0.8
    OCCLUDED = "occluded"     # 0.8 - 1.0


@dataclass
class EnhancedOcclusionConfig:
    """Configuration for enhanced occlusion detection."""
    # Score thresholds
    occlusion_threshold: float = 0.60          # Trigger fallback above this
    # Consecutive frames (2.0s at 10Hz)
    frames_threshold: int = 20

    # Hand detection presence (Weight: 0.5 max)
    max_lost_frames: int = 15                  # ~0.5 seconds at 30fps

    # Landmark visibility (Weight: 0.25)
    min_landmark_presence: float = 0.5         # Minimum presence score

    # Key landmark tracking (Weight: 0.15)
    key_landmark_indices: List[int] = None     # Critical landmarks to monitor

    # Depth quality (Weight: 0.3 total)
    min_depth_valid_ratio: float = 0.3         # Valid pixel ratio
    max_depth_std_dev: float = 0.03            # Depth standard deviation
    depth_std_normalization: Tuple[float, float] = (
        0.02, 0.05)  # (min, max) for normalization

    # Temporal smoothing
    history_size: int = 30                     # Sliding window size
    use_temporal_smoothing: bool = True        # Enable smoothing

    # Adaptive Z-filtering integration
    use_adaptive_z_filter: bool = True         # Use existing Z-filter state
    z_filter_alpha_good: float = 0.5          # Responsive when quality good
    z_filter_alpha_poor: float = 0.15         # Smooth when quality poor

    def __post_init__(self):
        if self.key_landmark_indices is None:
            # Critical landmarks: wrist, thumb tip, index tip, middle tip, ring tip, pinky tip
            self.key_landmark_indices = [0, 4, 8, 12, 16, 20]


@dataclass
class OcclusionMetrics:
    """Detailed occlusion metrics for analysis."""
    total_score: float
    indicators: Dict[str, float]
    level: OcclusionLevel
    smoothed_score: float
    consecutive_frames: int
    time_in_state: float


class EnhancedHandOcclusionDetector:
    """
    Enhanced occlusion detector with multi-factor scoring.

    Implements the sophisticated strategy from occlusion_strategy.md:
    - Weighted indicator scoring (0.0-1.0)
    - Temporal smoothing with sliding window
    - Adaptive Z-filtering integration
    - Gradual quality degradation detection
    """

    def __init__(self, config: Optional[EnhancedOcclusionConfig] = None):
        """Initialize the enhanced occlusion detector."""
        self.config = config or EnhancedOcclusionConfig()

        # State tracking
        self.occlusion_frames = 0
        self.occlusion_triggered = False
        self.last_update_time = time.time()

        # Temporal smoothing
        self.occlusion_history = []
        self.lost_frames_count = 0

        # Adaptive Z-filter state (if available)
        self.z_filter_quality_good = False

        logger.info(
            f"EnhancedHandOcclusionDetector initialized with config: {self.config}")

    def update(
        self,
        hand_landmarks: Optional[List] = None,
        depth_quality: Optional[Dict] = None,
        confidence: Optional[float] = None,
        landmark_count: Optional[int] = None,
        hand_position: Optional[List] = None,
        z_filter_quality: Optional[bool] = None
    ) -> OcclusionMetrics:
        """
        Update occlusion detection with multi-factor analysis.

        Args:
            hand_landmarks: MediaPipe hand landmarks with presence field
            depth_quality: Dict with 'valid_ratio' and 'depth_std_m'
            confidence: MediaPipe confidence score (0-1)
            landmark_count: Number of visible landmarks (0-21)
            hand_position: Hand position [x, y, z]
            z_filter_quality: Current Z-filter quality state

        Returns:
            OcclusionMetrics with detailed analysis
        """
        current_time = time.time()

        # Compute raw occlusion score
        raw_score, indicators = self._compute_occlusion_score(
            hand_landmarks, depth_quality, confidence, landmark_count,
            hand_position, z_filter_quality
        )

        # Apply temporal smoothing
        smoothed_score = self._apply_temporal_smoothing(raw_score)

        # Determine occlusion level
        level = self._classify_occlusion_level(smoothed_score)

        # Update state tracking
        self._update_occlusion_state(smoothed_score, current_time)

        # Create metrics
        metrics = OcclusionMetrics(
            total_score=raw_score,
            indicators=indicators,
            level=level,
            smoothed_score=smoothed_score,
            consecutive_frames=self.occlusion_frames,
            time_in_state=current_time - self.last_update_time
        )

        # Log significant changes
        if self.occlusion_frames > 0 and self.occlusion_frames % 10 == 0:
            logger.debug(f"Occlusion: score={smoothed_score:.3f}, "
                         f"frames={self.occlusion_frames}, "
                         f"indicators={indicators}")

        return metrics

    def _compute_occlusion_score(
        self,
        hand_landmarks: Optional[List],
        depth_quality: Optional[Dict],
        confidence: Optional[float],
        landmark_count: Optional[int],
        hand_position: Optional[List],
        z_filter_quality: Optional[bool]
    ) -> Tuple[float, Dict[str, float]]:
        """Compute occlusion score using weighted indicators."""
        indicators = {}

        # 1. Hand Detection Presence (Weight: 0.5 max)
        presence_score = self._compute_presence_score(
            hand_position, confidence, landmark_count)
        indicators['presence'] = presence_score

        # 2. Landmark Visibility (Weight: 0.25)
        visibility_score = self._compute_visibility_score(hand_landmarks)
        indicators['visibility'] = visibility_score

        # 3. Key Landmark Tracking (Weight: 0.15)
        key_landmark_score = self._compute_key_landmark_score(hand_landmarks)
        indicators['key_landmarks'] = key_landmark_score

        # 4. Depth Quality (Weight: 0.3 total)
        depth_scores = self._compute_depth_quality_score(depth_quality)
        indicators.update(depth_scores)

        # 5. Adaptive Z-Filter Integration (Weight: 0.1)
        z_filter_score = self._compute_z_filter_score(z_filter_quality)
        indicators['z_filter'] = z_filter_score

        # Compute total weighted score
        total_score = (
            presence_score * 0.5 +
            visibility_score * 0.25 +
            key_landmark_score * 0.15 +
            depth_scores.get('depth_ratio', 0.0) * 0.2 +
            depth_scores.get('depth_std', 0.0) * 0.1 +
            z_filter_score * 0.1
        )

        # Clamp to [0, 1]
        total_score = max(0.0, min(1.0, total_score))

        return total_score, indicators

    def _compute_presence_score(
        self,
        hand_position: Optional[List],
        confidence: Optional[float],
        landmark_count: Optional[int]
    ) -> float:
        """Compute hand detection presence score (0-1)."""
        # Check if hand is detected at all
        is_detected = True
        if hand_position is not None:
            if hand_position == [0.0, 0.0, 0.0] or all(x == 0 for x in hand_position):
                is_detected = False
        elif confidence is None and landmark_count is None:
            is_detected = False

        if not is_detected:
            self.lost_frames_count += 1
            if self.lost_frames_count >= self.config.max_lost_frames:
                return 1.0  # Fully occluded
            else:
                return (self.lost_frames_count / self.config.max_lost_frames) * 0.5
        else:
            self.lost_frames_count = 0
            return 0.0  # Hand detected

    def _compute_visibility_score(self, hand_landmarks: Optional[List]) -> float:
        """Compute landmark visibility score (0-1)."""
        if not hand_landmarks:
            return 1.0  # No landmarks = fully occluded

        # Extract presence scores from landmarks
        visibilities = []
        for landmark in hand_landmarks:
            if hasattr(landmark, 'presence'):
                visibilities.append(landmark.presence)
            else:
                # Fallback: assume good visibility if no presence field
                visibilities.append(1.0)

        if not visibilities:
            return 1.0

        avg_visibility = np.mean(visibilities)
        return (1.0 - avg_visibility) * 0.25

    def _compute_key_landmark_score(self, hand_landmarks: Optional[List]) -> float:
        """Compute key landmark tracking score (0-1)."""
        if not hand_landmarks:
            return 1.0  # No landmarks = fully occluded

        key_indices = self.config.key_landmark_indices
        visible_count = 0

        for i in key_indices:
            if i < len(hand_landmarks):
                landmark = hand_landmarks[i]
                if hasattr(landmark, 'presence'):
                    if landmark.presence > self.config.min_landmark_presence:
                        visible_count += 1
                else:
                    # Fallback: assume visible if no presence field
                    visible_count += 1

        key_ratio = visible_count / len(key_indices)
        return (1.0 - key_ratio) * 0.15

    def _compute_depth_quality_score(self, depth_quality: Optional[Dict]) -> Dict[str, float]:
        """Compute depth quality scores (0-1)."""
        scores = {'depth_ratio': 0.0, 'depth_std': 0.0}

        if not depth_quality:
            return scores

        # Valid pixel ratio (Weight: 0.2)
        valid_ratio = depth_quality.get('valid_ratio', 0.0)
        scores['depth_ratio'] = (1.0 - valid_ratio) * 0.2

        # Depth standard deviation (Weight: 0.1)
        depth_std = depth_quality.get('depth_std_m', float('inf'))
        if depth_std != float('inf'):
            min_std, max_std = self.config.depth_std_normalization
            std_normalized = min(
                1.0, max(0.0, (depth_std - min_std) / (max_std - min_std)))
            scores['depth_std'] = std_normalized * 0.1

        return scores

    def _compute_z_filter_score(self, z_filter_quality: Optional[bool]) -> float:
        """Compute Z-filter quality score (0-1)."""
        if not self.config.use_adaptive_z_filter or z_filter_quality is None:
            return 0.0  # Neutral if not available

        # Update internal state
        self.z_filter_quality_good = z_filter_quality

        # Return 0 if good quality, 0.1 if poor quality
        return 0.0 if z_filter_quality else 0.1

    def _apply_temporal_smoothing(self, raw_score: float) -> float:
        """Apply temporal smoothing using sliding window."""
        if not self.config.use_temporal_smoothing:
            return raw_score

        # Add to history
        self.occlusion_history.append(raw_score)
        if len(self.occlusion_history) > self.config.history_size:
            self.occlusion_history.pop(0)

        # Return smoothed score
        if not self.occlusion_history:
            return raw_score

        return np.mean(self.occlusion_history)

    def _classify_occlusion_level(self, score: float) -> OcclusionLevel:
        """Classify occlusion level based on score."""
        if score >= 0.8:
            return OcclusionLevel.OCCLUDED
        elif score >= 0.6:
            return OcclusionLevel.SEVERE
        elif score >= 0.4:
            return OcclusionLevel.MODERATE
        elif score >= 0.2:
            return OcclusionLevel.MINOR
        else:
            return OcclusionLevel.CLEAR

    def _update_occlusion_state(self, smoothed_score: float, current_time: float):
        """Update internal occlusion state tracking."""
        if smoothed_score >= self.config.occlusion_threshold:
            self.occlusion_frames += 1
            if self.occlusion_frames >= self.config.frames_threshold:
                self.occlusion_triggered = True
        else:
            self.occlusion_frames = 0
            self.occlusion_triggered = False

        self.last_update_time = current_time

    def is_occlusion_triggered(self) -> bool:
        """Check if occlusion fallback should be triggered."""
        return self.occlusion_triggered

    def get_statistics(self) -> Dict:
        """Get detailed statistics about occlusion detection."""
        return {
            'occlusion_frames': self.occlusion_frames,
            'occlusion_triggered': self.occlusion_triggered,
            'history_size': len(self.occlusion_history),
            'lost_frames_count': self.lost_frames_count,
            'z_filter_quality_good': self.z_filter_quality_good,
            'average_score': np.mean(self.occlusion_history) if self.occlusion_history else 0.0
        }

    def reset(self):
        """Reset the occlusion detector to initial state."""
        self.occlusion_frames = 0
        self.occlusion_triggered = False
        self.occlusion_history.clear()
        self.lost_frames_count = 0
        self.z_filter_quality_good = False
        logger.info("EnhancedHandOcclusionDetector reset")
