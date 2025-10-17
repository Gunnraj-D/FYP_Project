"""
Hand Occlusion Detector using MediaPipe confidence scores.

This module provides intelligent occlusion detection with:
- MediaPipe confidence score monitoring
- Visible landmark counting
- 2-second grace period for temporary occlusions
- Automatic failure after persistent occlusion
"""
import time
import logging
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class OcclusionState(Enum):
    """States for hand occlusion detection."""
    VISIBLE = "visible"              # Hand is clearly visible
    TEMPORARILY_OCCLUDED = "temp"    # Hand occluded but within grace period
    OCCLUDED_FAILED = "failed"       # Hand occluded beyond grace period
    NO_HAND = "no_hand"              # No hand detected at all


@dataclass
class OcclusionConfig:
    """Configuration for hand occlusion detection."""
    # MediaPipe confidence threshold (below this = occluded)
    min_confidence: float = 0.3

    # Minimum number of visible landmarks (out of 21)
    min_visible_landmarks: int = 15

    # Grace period to wait for hand to become unoccluded (seconds)
    grace_period_s: float = 2.0

    # Number of consecutive occluded frames to confirm occlusion
    consecutive_frames_threshold: int = 3

    # Timeout: if no hand appears within this time, fail (seconds)
    no_hand_timeout_s: float = 2.0


@dataclass
class OcclusionStatus:
    """Current occlusion status information."""
    state: OcclusionState
    confidence: Optional[float]
    visible_landmarks: Optional[int]
    time_in_state: float
    reason: str


class HandOcclusionDetector:
    """
    Detects hand occlusion using MediaPipe confidence scores.

    Features:
    - Tracks MediaPipe detection confidence
    - Counts visible landmarks (out of 21 total)
    - Implements 2-second grace period for temporary occlusions
    - Fails gracefully after persistent occlusion
    - Distinguishes between "no hand" and "occluded hand"

    Usage:
        detector = HandOcclusionDetector()
        status = detector.update(confidence=0.8, landmark_count=21)

        if status.state == OcclusionState.OCCLUDED_FAILED:
            # Handle failure
            pass
    """

    def __init__(self, config: Optional[OcclusionConfig] = None):
        """
        Initialize the occlusion detector.

        Args:
            config: Optional configuration. Uses defaults if None.
        """
        self.config = config or OcclusionConfig()

        # State tracking
        self.current_state = OcclusionState.NO_HAND
        self.state_start_time = time.time()
        self.last_update_time = time.time()

        # Occlusion frame counting
        self.consecutive_occluded_frames = 0
        self.consecutive_visible_frames = 0

        # History for debugging
        self.confidence_history = []
        self.max_history_size = 30  # Keep last 30 samples

        logger.info(
            f"HandOcclusionDetector initialized with config: {self.config}")

    def update(
        self,
        confidence: Optional[float] = None,
        landmark_count: Optional[int] = None,
        hand_position: Optional[list] = None
    ) -> OcclusionStatus:
        """
        Update occlusion state based on current hand detection data.

        Args:
            confidence: MediaPipe hand detection confidence (0-1)
            landmark_count: Number of visible landmarks (0-21)
            hand_position: Hand position [x, y, z] (used to detect "no hand")

        Returns:
            OcclusionStatus with current state and timing information
        """
        current_time = time.time()
        time_in_current_state = current_time - self.state_start_time

        # Determine if hand is visible based on available data
        is_hand_detected, is_good_quality = self._evaluate_hand_quality(
            confidence, landmark_count, hand_position
        )

        # Store confidence history
        if confidence is not None:
            self.confidence_history.append(confidence)
            if len(self.confidence_history) > self.max_history_size:
                self.confidence_history.pop(0)

        # State machine logic
        previous_state = self.current_state
        reason = ""

        if not is_hand_detected:
            # No hand detected at all
            if self.current_state == OcclusionState.NO_HAND:
                # Still no hand - check timeout
                if time_in_current_state > self.config.no_hand_timeout_s:
                    self.current_state = OcclusionState.OCCLUDED_FAILED
                    reason = f"No hand detected for {time_in_current_state:.1f}s (timeout)"
                else:
                    reason = f"No hand detected ({time_in_current_state:.1f}s elapsed)"
            else:
                # Transitioned from visible/occluded to no hand
                self._transition_to_state(OcclusionState.NO_HAND, current_time)
                reason = "Hand disappeared"

            self.consecutive_occluded_frames += 1
            self.consecutive_visible_frames = 0

        elif not is_good_quality:
            # Hand detected but quality is poor (occluded)
            self.consecutive_occluded_frames += 1
            self.consecutive_visible_frames = 0

            # Only transition to occluded if we have enough consecutive frames
            if self.consecutive_occluded_frames >= self.config.consecutive_frames_threshold:
                if self.current_state == OcclusionState.VISIBLE:
                    # Just became occluded - start grace period
                    self._transition_to_state(
                        OcclusionState.TEMPORARILY_OCCLUDED, current_time)
                    reason = f"Hand occluded (confidence={confidence:.2f}, landmarks={landmark_count})"

                elif self.current_state == OcclusionState.TEMPORARILY_OCCLUDED:
                    # Still occluded - check if grace period expired
                    if time_in_current_state > self.config.grace_period_s:
                        self._transition_to_state(
                            OcclusionState.OCCLUDED_FAILED, current_time)
                        reason = f"Hand occluded for {time_in_current_state:.1f}s (grace period expired)"
                    else:
                        reason = f"Hand temporarily occluded ({time_in_current_state:.1f}s / {self.config.grace_period_s:.1f}s)"

                elif self.current_state == OcclusionState.NO_HAND:
                    # Was no hand, now occluded - this is actually progress
                    self._transition_to_state(
                        OcclusionState.TEMPORARILY_OCCLUDED, current_time)
                    reason = "Hand appeared but occluded"

        else:
            # Hand is visible and good quality
            self.consecutive_visible_frames += 1
            self.consecutive_occluded_frames = 0

            if self.current_state != OcclusionState.VISIBLE:
                # Transitioned from occluded/no hand to visible
                if self.consecutive_visible_frames >= 2:  # Require 2 consecutive good frames
                    self._transition_to_state(
                        OcclusionState.VISIBLE, current_time)
                    reason = f"Hand visible (confidence={confidence:.2f}, landmarks={landmark_count})"
            else:
                reason = f"Hand tracking normally (confidence={confidence:.2f})"

        # Log state transitions
        if previous_state != self.current_state:
            logger.info(
                f"Occlusion state change: {previous_state.value} → {self.current_state.value} ({reason})")

        return OcclusionStatus(
            state=self.current_state,
            confidence=confidence,
            visible_landmarks=landmark_count,
            time_in_state=time_in_current_state,
            reason=reason
        )

    def _evaluate_hand_quality(
        self,
        confidence: Optional[float],
        landmark_count: Optional[int],
        hand_position: Optional[list]
    ) -> Tuple[bool, bool]:
        """
        Evaluate if hand is detected and if quality is good.

        Returns:
            (is_hand_detected, is_good_quality)
        """
        # Check if hand is detected at all
        is_hand_detected = True
        if hand_position is not None:
            # [0, 0, 0] means no hand detected
            if hand_position == [0.0, 0.0, 0.0] or all(x == 0 for x in hand_position):
                is_hand_detected = False
        elif confidence is None and landmark_count is None:
            is_hand_detected = False

        if not is_hand_detected:
            return False, False

        # Check quality using confidence and landmark count
        is_good_quality = True

        if confidence is not None:
            if confidence < self.config.min_confidence:
                is_good_quality = False

        if landmark_count is not None:
            if landmark_count < self.config.min_visible_landmarks:
                is_good_quality = False

        return is_hand_detected, is_good_quality

    def _transition_to_state(self, new_state: OcclusionState, current_time: float):
        """Transition to a new occlusion state."""
        self.current_state = new_state
        self.state_start_time = current_time

    def is_failed(self) -> bool:
        """Check if occlusion detector has failed."""
        return self.current_state == OcclusionState.OCCLUDED_FAILED

    def is_visible(self) -> bool:
        """Check if hand is currently visible."""
        return self.current_state == OcclusionState.VISIBLE

    def reset(self):
        """Reset the occlusion detector to initial state."""
        self.current_state = OcclusionState.NO_HAND
        self.state_start_time = time.time()
        self.consecutive_occluded_frames = 0
        self.consecutive_visible_frames = 0
        self.confidence_history.clear()
        logger.info("HandOcclusionDetector reset")

    def get_statistics(self) -> Dict:
        """Get statistics about occlusion detection."""
        avg_confidence = None
        if self.confidence_history:
            avg_confidence = sum(self.confidence_history) / \
                len(self.confidence_history)

        return {
            'current_state': self.current_state.value,
            'time_in_state': time.time() - self.state_start_time,
            'consecutive_occluded_frames': self.consecutive_occluded_frames,
            'consecutive_visible_frames': self.consecutive_visible_frames,
            'average_confidence': avg_confidence,
            'confidence_samples': len(self.confidence_history)
        }
