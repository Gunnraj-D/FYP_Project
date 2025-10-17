"""
Hybrid Hand Occlusion Detector - Combines simple and enhanced approaches.

This module provides a unified interface that can switch between:
1. Simple MediaPipe confidence-based detection (fast, reliable)
2. Enhanced multi-factor detection (sophisticated, comprehensive)

Features:
- Runtime switching between detection methods
- Backward compatibility with existing code
- Gradual migration path
- Performance monitoring
"""
import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

from hand_detection.hand_occlusion_detector import (
    HandOcclusionDetector,
    OcclusionConfig,
    OcclusionState as SimpleOcclusionState,
    OcclusionStatus as SimpleOcclusionStatus
)
from hand_detection.enhanced_occlusion_detector import (
    EnhancedHandOcclusionDetector,
    EnhancedOcclusionConfig,
    OcclusionLevel,
    OcclusionMetrics
)
from config import HAND_OCCLUSION_CONFIG, ENHANCED_OCCLUSION_CONFIG

logger = logging.getLogger(__name__)


class DetectionMode(Enum):
    """Detection mode for the hybrid detector."""
    SIMPLE = "simple"           # MediaPipe confidence only
    ENHANCED = "enhanced"       # Multi-factor scoring
    AUTO = "auto"              # Automatically choose based on data availability


@dataclass
class HybridOcclusionStatus:
    """Unified occlusion status from hybrid detector."""
    # Common fields
    is_occluded: bool
    is_failed: bool
    confidence: Optional[float]
    time_in_state: float
    reason: str

    # Mode-specific fields
    mode: DetectionMode
    simple_status: Optional[SimpleOcclusionStatus] = None
    enhanced_metrics: Optional[OcclusionMetrics] = None

    # Performance metrics
    detection_time_ms: float = 0.0


class HybridHandOcclusionDetector:
    """
    Hybrid occlusion detector supporting both simple and enhanced methods.

    Provides a unified interface that can switch between detection methods
    at runtime while maintaining backward compatibility.
    """

    def __init__(
        self,
        mode: DetectionMode = DetectionMode.AUTO,
        simple_config: Optional[OcclusionConfig] = None,
        enhanced_config: Optional[EnhancedOcclusionConfig] = None
    ):
        """
        Initialize the hybrid occlusion detector.

        Args:
            mode: Detection mode (SIMPLE, ENHANCED, or AUTO)
            simple_config: Configuration for simple detector
            enhanced_config: Configuration for enhanced detector
        """
        self.mode = mode

        # Initialize simple detector
        simple_config = simple_config or OcclusionConfig(
            **HAND_OCCLUSION_CONFIG)
        self.simple_detector = HandOcclusionDetector(simple_config)

        # Initialize enhanced detector
        enhanced_config = enhanced_config or EnhancedOcclusionConfig(
            **ENHANCED_OCCLUSION_CONFIG)
        self.enhanced_detector = EnhancedHandOcclusionDetector(enhanced_config)

        # State tracking
        self.current_mode = mode
        self.last_update_time = time.time()
        self.performance_stats = {
            'simple_detections': 0,
            'enhanced_detections': 0,
            'total_detection_time': 0.0,
            'mode_switches': 0
        }

        logger.info(
            f"HybridHandOcclusionDetector initialized with mode: {mode.value}")

    def update(
        self,
        # Simple detector parameters
        confidence: Optional[float] = None,
        landmark_count: Optional[int] = None,
        hand_position: Optional[list] = None,
        # Enhanced detector parameters
        hand_landmarks: Optional[list] = None,
        depth_quality: Optional[Dict] = None,
        z_filter_quality: Optional[bool] = None
    ) -> HybridOcclusionStatus:
        """
        Update occlusion detection using the current mode.

        Args:
            confidence: MediaPipe confidence (0-1)
            landmark_count: Number of visible landmarks (0-21)
            hand_position: Hand position [x, y, z]
            hand_landmarks: MediaPipe landmarks with presence field
            depth_quality: Dict with 'valid_ratio' and 'depth_std_m'
            z_filter_quality: Z-filter quality state

        Returns:
            HybridOcclusionStatus with unified results
        """
        start_time = time.time()
        current_time = time.time()

        # Determine detection mode
        actual_mode = self._determine_mode(hand_landmarks, depth_quality)

        # Switch mode if needed
        if actual_mode != self.current_mode:
            self._switch_mode(actual_mode)

        # Perform detection based on mode
        if actual_mode == DetectionMode.SIMPLE:
            status = self._update_simple(
                confidence, landmark_count, hand_position)
        else:  # ENHANCED
            status = self._update_enhanced(
                hand_landmarks, depth_quality, confidence,
                landmark_count, hand_position, z_filter_quality
            )

        # Update performance stats
        detection_time = (time.time() - start_time) * 1000  # Convert to ms
        self.performance_stats['total_detection_time'] += detection_time

        if actual_mode == DetectionMode.SIMPLE:
            self.performance_stats['simple_detections'] += 1
        else:
            self.performance_stats['enhanced_detections'] += 1

        # Create hybrid status
        hybrid_status = HybridOcclusionStatus(
            is_occluded=status.is_occluded if hasattr(
                status, 'is_occluded') else False,
            is_failed=status.is_failed if hasattr(
                status, 'is_failed') else False,
            confidence=confidence,
            time_in_state=current_time - self.last_update_time,
            reason=getattr(status, 'reason', 'Unknown'),
            mode=actual_mode,
            simple_status=status if actual_mode == DetectionMode.SIMPLE else None,
            enhanced_metrics=status if actual_mode == DetectionMode.ENHANCED else None,
            detection_time_ms=detection_time
        )

        self.last_update_time = current_time
        return hybrid_status

    def _determine_mode(
        self,
        hand_landmarks: Optional[list],
        depth_quality: Optional[Dict]
    ) -> DetectionMode:
        """Determine which detection mode to use."""
        if self.mode == DetectionMode.SIMPLE:
            return DetectionMode.SIMPLE
        elif self.mode == DetectionMode.ENHANCED:
            return DetectionMode.ENHANCED
        else:  # AUTO mode
            # Use enhanced if we have rich data, simple otherwise
            has_landmarks = hand_landmarks is not None and len(
                hand_landmarks) > 0
            has_depth = depth_quality is not None

            if has_landmarks and has_depth:
                return DetectionMode.ENHANCED
            else:
                return DetectionMode.SIMPLE

    def _switch_mode(self, new_mode: DetectionMode):
        """Switch to a new detection mode."""
        if new_mode != self.current_mode:
            logger.info(
                f"Switching occlusion detection mode: {self.current_mode.value} → {new_mode.value}")
            self.current_mode = new_mode
            self.performance_stats['mode_switches'] += 1

    def _update_simple(
        self,
        confidence: Optional[float],
        landmark_count: Optional[int],
        hand_position: Optional[list]
    ) -> SimpleOcclusionStatus:
        """Update using simple detector."""
        return self.simple_detector.update(confidence, landmark_count, hand_position)

    def _update_enhanced(
        self,
        hand_landmarks: Optional[list],
        depth_quality: Optional[Dict],
        confidence: Optional[float],
        landmark_count: Optional[int],
        hand_position: Optional[list],
        z_filter_quality: Optional[bool]
    ) -> OcclusionMetrics:
        """Update using enhanced detector."""
        return self.enhanced_detector.update(
            hand_landmarks, depth_quality, confidence,
            landmark_count, hand_position, z_filter_quality
        )

    def set_mode(self, mode: DetectionMode):
        """Manually set the detection mode."""
        self.mode = mode
        logger.info(f"Occlusion detection mode set to: {mode.value}")

    def is_occlusion_triggered(self) -> bool:
        """Check if occlusion fallback should be triggered."""
        if self.current_mode == DetectionMode.SIMPLE:
            return self.simple_detector.is_failed()
        else:
            return self.enhanced_detector.is_occlusion_triggered()

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        total_detections = (self.performance_stats['simple_detections'] +
                            self.performance_stats['enhanced_detections'])

        avg_detection_time = 0.0
        if total_detections > 0:
            avg_detection_time = (self.performance_stats['total_detection_time'] /
                                  total_detections)

        return {
            'current_mode': self.current_mode.value,
            'simple_detections': self.performance_stats['simple_detections'],
            'enhanced_detections': self.performance_stats['enhanced_detections'],
            'mode_switches': self.performance_stats['mode_switches'],
            'avg_detection_time_ms': avg_detection_time,
            'total_detection_time_ms': self.performance_stats['total_detection_time']
        }

    def get_detailed_stats(self) -> Dict[str, Any]:
        """Get detailed statistics from both detectors."""
        return {
            'performance': self.get_performance_stats(),
            'simple_stats': self.simple_detector.get_statistics(),
            'enhanced_stats': self.enhanced_detector.get_statistics()
        }

    def reset(self):
        """Reset both detectors to initial state."""
        self.simple_detector.reset()
        self.enhanced_detector.reset()
        self.last_update_time = time.time()
        self.performance_stats = {
            'simple_detections': 0,
            'enhanced_detections': 0,
            'total_detection_time': 0.0,
            'mode_switches': 0
        }
        logger.info("HybridHandOcclusionDetector reset")
