"""
Grasping State with Multi-Frame Best-of-N Selection.

This state:
1. Captures N frames (default 8) and collects grasp candidates
2. Uses multi-factor scoring to rank all candidates
3. Selects the best grasp across all frames
4. Supports object profile switching for different object types
5. Falls back to retry logic if no valid grasps found

Based on the sophisticated selection logic from visualize_grconvnet_temporal.py
"""
import logging
import time
import numpy as np
from typing import Optional, List, Dict, Any
from dataclasses import dataclass

from states.base_state import BaseState
from states.context import StateContext
from object_detection.grasp_detector import GraspDetector
from config import (
    GRCONVNET_MODEL_PATH,
    GRASP_EXECUTION_CONFIG,
    GRASP_DETECTION_CONFIG,
    OBJECT_PROFILES,
    DEBUG_MODE,
    DEBUG_CONFIG
)

logger = logging.getLogger(__name__)


@dataclass
class GraspCandidate:
    """Enhanced grasp candidate with scoring metrics."""
    # Core grasp data
    grasp_result: Dict[Any, Any]  # Original grasp result from detector
    joint_angles: List[float]
    pose: List[float]
    quality: float

    # Enhanced metrics (if available from advanced postprocessing)
    width_mm: float = 0.0
    object_overlap: float = 0.0
    border_distance: float = 1.0
    grasp_height: float = 0.0

    # Scoring
    multi_factor_score: float = 0.0
    frame_index: int = 0
    timestamp: float = 0.0


class GraspingState(BaseState):
    """
    Grasping state with multi-frame best-of-N selection.

    Multi-Frame Selection (Standard):
        - Captures N frames (default 8)
        - Scores all valid grasps using multi-factor scoring
        - Picks best across all frames
        - Robust to single-frame failures
        - More reliable than single-shot detection

    Object Profile Support:
        - Easy switching between screwdrivers, small objects, large flat, etc.
        - Automatically adjusts min_overlap, nms_dilate_size, etc.
        - Pass object_profile='small_objects' for different object types
    """

    def __init__(self,
                 context: StateContext,
                 auto_process: bool = False,
                 approach_z_offset: float = 0.05,
                 num_collection_frames: int = 8,
                 object_profile: Optional[str] = None,
                 enable_visuals: Optional[bool] = None):
        """
        Initialize grasping state.

        Args:
            context: State context with shared resources
            auto_process: If True, process automatically (sequencer mode)
            approach_z_offset: Z offset for approach pose above grasp (meters)
            num_collection_frames: Number of frames to collect (5-10 recommended)
            object_profile: Optional object profile name from OBJECT_PROFILES
                           (e.g., 'screwdriver_30mm', 'small_objects', 'large_flat')
        """
        super().__init__(context)

        # Configuration
        self.num_collection_frames = num_collection_frames
        self.approach_z_offset = approach_z_offset
        self.object_profile = object_profile

        # Grasp detector
        self.grasp_detector: Optional[GraspDetector] = None
        self.enable_visuals = enable_visuals

        # Retry logic (for fallback)
        self.max_attempts = GRASP_EXECUTION_CONFIG['retry_attempts']
        self.retry_delay = GRASP_EXECUTION_CONFIG.get('retry_delay', 2.0)

        # Multi-frame collection
        self.collected_candidates: List[GraspCandidate] = []
        self.frames_collected = 0
        self.collection_start_time = 0.0

        # State tracking
        self.best_grasp: Optional[GraspCandidate] = None
        self.current_attempt = 0
        self.last_attempt_time = 0.0
        self.state_start_time = 0.0
        self.grasp_start_time = 0.0

        # Debug mode
        self.debug_mode = DEBUG_MODE
        self.frame_selection_enabled = DEBUG_CONFIG.get(
            'frame_selection_enabled', True) and not auto_process
        self.selected_frame = None
        self.frame_selected = False
        self.collection_triggered = False  # Track if spacebar pressed to start collection

        logger.info(f"GraspingState initialized: frames={num_collection_frames}, "
                    f"profile={object_profile if object_profile else 'default'}")

    def enter(self):
        """Initialize grasp detector and reset state."""
        logger.info(
            f"Entering GraspingState (multi-frame mode, {self.num_collection_frames} frames)")

        try:
            # Apply object profile if specified
            if self.object_profile and self.object_profile in OBJECT_PROFILES:
                self._apply_object_profile(self.object_profile)

            # Initialize grasp detector
            self.grasp_detector = GraspDetector(
                model_path=str(GRCONVNET_MODEL_PATH),
                telemetry=self.context.telemetry,
                command_bus=self.context.commands,
                camera_manager=self.context.camera,
                kinematics_solver=self.context.ik,
                enable_visuals=self.enable_visuals
            )

            # Reset state
            self.collected_candidates = []
            self.frames_collected = 0
            self.best_grasp = None
            self.current_attempt = 0
            self.state_start_time = time.time()
            self.collection_start_time = time.time()
            self.last_attempt_time = 0.0
            self.grasp_start_time = 0.0
            self.collection_triggered = False  # Reset debug mode trigger

            logger.info("GraspingState initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize GraspingState: {e}")
            raise

    def execute(self):
        """Main execution loop."""
        try:
            # Check if we're in retry delay period
            if self.last_attempt_time > 0:
                elapsed = time.time() - self.last_attempt_time
                if elapsed < self.retry_delay:
                    return  # Still waiting
                else:
                    # Reset for next attempt
                    self.last_attempt_time = 0.0
                    self.collected_candidates = []
                    self.frames_collected = 0
                    self.collection_start_time = time.time()
                    self.collection_triggered = False  # Reset debug trigger for retry
                    logger.info(
                        f"Starting attempt {self.current_attempt + 1}/{self.max_attempts}")

            # Get frames
            color_frame, depth_frame = self.context.camera.get_frames()
            if depth_frame is None:
                logger.warning("No depth frame available")
                return

            # Handle debug mode (manual frame selection)
            if self.debug_mode and self.frame_selection_enabled:
                self._handle_debug_mode(color_frame, depth_frame)
                return

            # Execute multi-frame collection and selection
            self._execute_multi_frame(depth_frame, color_frame)

        except Exception as e:
            logger.error(
                f"Error in GraspingState execution: {e}", exc_info=True)
            self.current_attempt += 1

    def _execute_multi_frame(self, depth_frame, color_frame):
        """Multi-frame best-of-N selection."""

        if self.best_grasp is not None:
            return

        # Fast collection: grab remaining frames in a tight loop
        while self.frames_collected < self.num_collection_frames:
            if depth_frame is None or color_frame is None:
                color_frame, depth_frame = self.context.camera.get_frames()
                if depth_frame is None:
                    break

            grasp_result = self.grasp_detector.process_depth_frame(
                depth_frame, color_frame)

            if grasp_result is not None:
                candidate = self._create_candidate(
                    grasp_result, self.frames_collected)
                self.collected_candidates.append(candidate)
                logger.info(
                    f"📸 Frame {self.frames_collected + 1}/{self.num_collection_frames}: "
                    f"Quality={candidate.quality:.3f}, Score={candidate.multi_factor_score:.4f}")
            else:
                logger.debug(
                    f"Frame {self.frames_collected + 1}/{self.num_collection_frames}: No grasp found")

            self.frames_collected += 1

            # Force fetch next frame
            depth_frame = None
            color_frame = None

        if self.frames_collected >= self.num_collection_frames:
            self._select_best_grasp()

    def _create_candidate(self, grasp_result: Dict, frame_idx: int) -> GraspCandidate:
        """Create enhanced grasp candidate with scoring."""

        # Extract core data
        quality = grasp_result.get('quality', 0.0)
        joint_angles = grasp_result.get('joint_angles', [])
        # FIX: Key is 'pose', not 'grasp_pose_base'
        pose = grasp_result.get('pose', [0]*6)

        # Extract enhanced metrics (if available from advanced postprocessing)
        width_mm = grasp_result.get('width_mm', 0.0)
        object_overlap = grasp_result.get('object_overlap', 0.0)
        border_distance = grasp_result.get('border_distance', 1.0)
        grasp_height = grasp_result.get('grasp_height', 0.0)

        # Calculate multi-factor score
        score = self._calculate_multi_factor_score(
            quality, object_overlap, border_distance, width_mm
        )

        candidate = GraspCandidate(
            grasp_result=grasp_result,
            joint_angles=joint_angles,
            pose=pose,
            quality=quality,
            width_mm=width_mm,
            object_overlap=object_overlap,
            border_distance=border_distance,
            grasp_height=grasp_height,
            multi_factor_score=score,
            frame_index=frame_idx,
            timestamp=time.time()
        )

        return candidate

    def _calculate_multi_factor_score(self, quality: float, overlap: float,
                                      border: float, width_mm: float) -> float:
        """
        Calculate multi-factor score for grasp selection.

        Formula: score = (Q^w_q) × (O^w_o) × (B^w_b) × (W^w_w) + ε

        Based on sophisticated scoring from visualize_grconvnet_temporal.py
        """
        # Get weights from config
        weights = GRASP_DETECTION_CONFIG.get('scoring_weights', {
            'q': 1.0, 'o': 1.2, 'b': 0.5, 'w': 0.7
        })

        # Ensure values in [0, 1]
        q = np.clip(quality, 0.0, 1.0)
        o = np.clip(overlap if overlap > 0 else 0.5, 0.0,
                    1.0)  # Default 0.5 if not available
        b = np.clip(border, 0.0, 1.0)

        # Width score (triangular preference for optimal range)
        w = self._width_score(width_mm)

        # Multiplicative scoring (all factors must be reasonable)
        epsilon = 1e-8
        score = (
            (q ** weights['q']) *
            (o ** weights['o']) *
            (b ** weights['b']) *
            (w ** weights['w']) +
            epsilon
        )

        return float(score)

    def _width_score(self, width_mm: float) -> float:
        """
        Width preference score: peaks at optimal range center.

        Optimal range: 15-60mm (Robotiq 2F-85)
        """
        if width_mm <= 0:
            return 0.5  # Unknown width, neutral score

        # Get gripper specs
        min_width = GRASP_EXECUTION_CONFIG.get(
            'gripper_min_width_m', 0.005) * 1000
        max_width = GRASP_EXECUTION_CONFIG.get(
            'gripper_max_width_m', 0.080) * 1000
        optimal_min = GRASP_EXECUTION_CONFIG.get(
            'gripper_optimal_min_mm', 15.0)
        optimal_max = GRASP_EXECUTION_CONFIG.get(
            'gripper_optimal_max_mm', 60.0)

        # Reject invalid widths
        if width_mm < min_width or width_mm > max_width:
            return 0.0

        # Triangular score: peaks at center of optimal range
        center = (optimal_min + optimal_max) / 2.0
        max_span = max(center - min_width, max_width - center, 1e-3)
        score = 1.0 - abs(width_mm - center) / max_span

        return float(np.clip(score, 0.0, 1.0))

    def _select_best_grasp(self):
        """Select best grasp from all collected candidates."""

        if not self.collected_candidates:
            logger.warning(
                f"❌ No valid grasps found in {self.frames_collected} frames")
            self.current_attempt += 1
            self.last_attempt_time = time.time()
            return

        # Sort by multi-factor score
        self.collected_candidates.sort(
            key=lambda c: c.multi_factor_score, reverse=True)

        # Select best
        self.best_grasp = self.collected_candidates[0]
        self.current_attempt += 1

        # Log selection results
        logger.info(f"\n{'='*70}")
        logger.info(f"🎯 BEST-OF-{self.frames_collected} GRASP SELECTION")
        logger.info(f"{'='*70}")
        logger.info(f"Total candidates: {len(self.collected_candidates)}")
        logger.info(
            f"Collection time: {time.time() - self.collection_start_time:.2f}s")
        logger.info(
            f"\n🏆 BEST GRASP (Frame {self.best_grasp.frame_index + 1}):")
        logger.info(f"  Score:   {self.best_grasp.multi_factor_score:.4f}")
        logger.info(f"  Quality: {self.best_grasp.quality:.3f}")
        logger.info(f"  Width:   {self.best_grasp.width_mm:.1f}mm")
        logger.info(f"  Overlap: {self.best_grasp.object_overlap:.2f}")
        logger.info(f"  Border:  {self.best_grasp.border_distance:.2f}")

        # Show top 3 for comparison
        if len(self.collected_candidates) >= 2:
            logger.info(f"\n📊 Top 3 Alternatives:")
            for i, candidate in enumerate(self.collected_candidates[1:4], 2):
                logger.info(f"  {i}. Frame {candidate.frame_index + 1}: "
                            f"Score={candidate.multi_factor_score:.4f}, "
                            f"Q={candidate.quality:.3f}, "
                            f"W={candidate.width_mm:.1f}mm")

        logger.info(f"{'='*70}\n")

        # Store selected grasp
        self._store_selected_grasp(self.best_grasp)

    def _store_selected_grasp(self, candidate: GraspCandidate):
        """Store selected grasp in telemetry for sequencer."""
        try:
            # Extract pose (already in base frame)
            grasp_pose_base = list(candidate.pose)

            # Apply table height correction and safety offset
            # The calibrated transform doesn't account for table height properly,
            # so we correct it based on known table height
            table_height = GRASP_DETECTION_CONFIG.get(
                'table_height_base_frame', 0.142)
            depth_offset = GRASP_DETECTION_CONFIG.get(
                'grasp_depth_offset', 0.04)

            original_z = grasp_pose_base[2]
            # Correct Z: current Z is relative to some reference, add table height + offset
            corrected_z = original_z + table_height + depth_offset
            grasp_pose_base[2] = corrected_z

            logger.info(f"📏 Z correction: raw={original_z:.3f}m + table={table_height:.3f}m + "
                        f"offset={depth_offset:.3f}m = {corrected_z:.3f}m")

            # Orientation already determined by pipeline; no manual rotation

            # Store grasp height
            grasp_height = grasp_pose_base[2]
            self.context.telemetry.update_grasp_height(grasp_height)

            # Store grasp pose
            self.context.telemetry.set_generated_grasp_pose(grasp_pose_base)
            logger.info(f"✅ Stored grasp pose: pos={grasp_pose_base[:3]}")

            # Generate and store approach pose
            approach_pose = list(grasp_pose_base)
            approach_pose[2] += self.approach_z_offset
            self.context.telemetry.set_generated_approach_pose(approach_pose)
            logger.info(
                f"✅ Stored approach pose: Z offset={self.approach_z_offset*1000:.0f}mm")

            # Mark grasp start time for visualization delay
            self.grasp_start_time = time.time()

        except Exception as e:
            logger.error(f"Failed to store grasp: {e}", exc_info=True)

    def _apply_object_profile(self, profile_name: str):
        """Apply object-specific configuration profile."""
        try:
            profile = OBJECT_PROFILES[profile_name]

            logger.info(f"📋 Applying object profile: {profile_name}")

            # Update GRASP_DETECTION_CONFIG in-place
            for key, value in profile.items():
                if key in GRASP_DETECTION_CONFIG:
                    old_value = GRASP_DETECTION_CONFIG[key]
                    GRASP_DETECTION_CONFIG[key] = value
                    logger.info(f"  {key}: {old_value} → {value}")

            logger.info(f"✅ Profile '{profile_name}' applied successfully")

        except Exception as e:
            logger.error(f"Failed to apply profile '{profile_name}': {e}")

    def is_complete(self) -> bool:
        """Check if grasping is complete."""

        # Check max attempts (retry limit)
        if self.current_attempt >= self.max_attempts:
            if self.best_grasp is None:
                logger.warning(
                    f"⚠️ Max attempts ({self.max_attempts}) reached without valid grasp")
            return True

        # If we have a grasp, wait for visualization delay
        if self.best_grasp is not None:
            if self.grasp_start_time == 0.0:
                self.grasp_start_time = time.time()
                return False

            elapsed = time.time() - self.grasp_start_time
            if elapsed >= 3.0:
                logger.info("✅ Grasp selection complete")
                return True

        return False

    def exit(self):
        """Clean up resources."""
        logger.info("Exiting GraspingState")

        # Clean up visualization windows
        if self.debug_mode:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception as e:
                logger.warning(f"Failed to close windows: {e}")

        # Detector cleanup
        try:
            if self.grasp_detector:
                self.grasp_detector.cleanup()
        except Exception:
            pass

        # Reset state
        self.collected_candidates = []
        self.frames_collected = 0
        self.best_grasp = None
        self.current_attempt = 0
        self.collection_triggered = False

    def get_grasp_quality(self) -> float:
        """Get quality of selected grasp."""
        if self.best_grasp:
            return self.best_grasp.quality
        return 0.0

    def get_grasp_joint_angles(self) -> Optional[List[float]]:
        """Get joint angles of selected grasp."""
        if self.best_grasp:
            return self.best_grasp.joint_angles
        return None

    def get_best_grasp_info(self) -> Optional[Dict[str, Any]]:
        """Get complete info about best grasp."""
        if self.best_grasp:
            return {
                'quality': self.best_grasp.quality,
                'score': self.best_grasp.multi_factor_score,
                'width_mm': self.best_grasp.width_mm,
                'overlap': self.best_grasp.object_overlap,
                'border': self.best_grasp.border_distance,
                'height': self.best_grasp.grasp_height,
                'frame_index': self.best_grasp.frame_index,
                'total_frames': self.frames_collected,
                'total_candidates': len(self.collected_candidates)
            }
        return None

    # ========================================================================
    # DEBUG MODE (same as original)
    # ========================================================================

    def _handle_debug_mode(self, color_frame, depth_frame):
        """Handle debug mode with live feed display."""
        try:
            import cv2

            # If collection already triggered, continue collecting automatically
            if self.collection_triggered:
                self._execute_multi_frame(depth_frame, color_frame)
                return

            # Show live feed and wait for spacebar to trigger collection
            # Convert depth frame to displayable format
            if hasattr(depth_frame, 'get_data'):
                depth_array = np.asanyarray(depth_frame.get_data())
            else:
                depth_array = depth_frame

            depth_display = self._normalize_depth_for_display(depth_array)
            display_image = self._create_debug_display(
                depth_display, color_frame)

            window_title = DEBUG_CONFIG.get(
                'window_title', 'Debug Feed - Press SPACEBAR to start collection')
            cv2.imshow(window_title, display_image)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # Spacebar - trigger collection
                logger.info("🚀 Starting multi-frame collection!")
                self.collection_triggered = True
                self.selected_frame = depth_frame
                self.frame_selected = True

            elif key == ord('q') or key == 27:  # Quit
                logger.info("Debug mode quit requested")
                cv2.destroyAllWindows()

        except Exception as e:
            logger.error(f"Error in debug mode: {e}")

    def _normalize_depth_for_display(self, depth_array):
        """Normalize depth for display."""
        depth_clean = np.nan_to_num(depth_array, nan=0.0)
        if depth_clean.max() > depth_clean.min():
            return ((depth_clean - depth_clean.min()) /
                    (depth_clean.max() - depth_clean.min()) * 255).astype(np.uint8)
        return np.zeros_like(depth_clean, dtype=np.uint8)

    def _create_debug_display(self, depth_display, color_frame):
        """Create debug display with instructions."""
        try:
            import cv2
            depth_colored = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

            if self.collection_triggered:
                # Collection in progress
                instructions = [
                    "🔄 COLLECTING FRAMES...",
                    f"Progress: {self.frames_collected}/{self.num_collection_frames}",
                    f"Candidates: {len(self.collected_candidates)}",
                    "Please wait...",
                ]
            else:
                # Waiting for trigger
                instructions = [
                    "MULTI-FRAME GRASP SELECTION",
                    f"Ready to collect {self.num_collection_frames} frames",
                    "Press SPACEBAR to start collection",
                    "Press 'q' to quit"
                ]

            y_offset = 30
            for i, text in enumerate(instructions):
                color = (0, 255, 0) if i == 0 else (255, 255, 255)
                thickness = 2 if i == 0 else 1
                cv2.putText(depth_colored, text, (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, thickness)
                y_offset += 25

            return depth_colored
        except Exception as e:
            logger.error(f"Error creating display: {e}")
            return depth_display
