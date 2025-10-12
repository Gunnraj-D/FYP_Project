"""
Hand detection module using MediaPipe for robot hand tracking system.
Refactored to use shared CameraManager for camera operations.
"""
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Optional
import logging
import threading

from control.telemetry_store import Telemetry
from control.command_bus import CommandBus, Command
from camera_management.camera_manager import CameraManager
from config import HANDMODEL_FILEPATH

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class HandTrackingConfig:
    """Hand tracking configuration parameters."""
    model_path: str = HANDMODEL_FILEPATH
    max_hands: int = 1
    min_confidence: float = 0.5
    palm_indices: List[int] = None
    palm_flatness_threshold: float = 0.15

    def __post_init__(self):
        if self.palm_indices is None:
            self.palm_indices = [0, 1, 2, 5, 9, 13, 17]


class HandTracker:
    """Hand tracking using MediaPipe with shared camera management."""

    def __init__(self, telemetry: Telemetry, command_bus: CommandBus, camera_manager: CameraManager,
                 config: HandTrackingConfig = None):

        self.config = config or HandTrackingConfig()
        self.telemetry = telemetry
        self.command_bus = command_bus
        self.camera_manager = camera_manager
        self.is_running = False

        # MediaPipe components
        self.landmarker = None

        # Detection state
        self.latest_result = None
        self.latest_timestamp = 0

        # Visualization
        self.mp_drawing = solutions.drawing_utils
        self.mp_hands = solutions.hands

        # Processing thread
        self.processing_thread: Optional[threading.Thread] = None

        # Filtering state - keep last 5 frames for outlier detection
        self.position_history = []
        self.max_history_size = 5
        self.filtering_threshold = 0.20  # 20cm threshold

    def _setup_detector(self):
        """Setup MediaPipe hand detector."""
        try:
            base_options = python.BaseOptions(
                model_asset_path=self.config.model_path)
            options = vision.HandLandmarkerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.LIVE_STREAM,
                num_hands=self.config.max_hands,
                min_hand_detection_confidence=self.config.min_confidence,
                min_hand_presence_confidence=self.config.min_confidence,
                min_tracking_confidence=self.config.min_confidence,
                result_callback=self._detection_callback
            )

            self.landmarker = vision.HandLandmarker.create_from_options(
                options)
            logger.info("Hand detector initialized")
        except Exception as e:
            logger.error(f"Hand detector setup failed: {e}")
            raise

    def _detection_callback(self, result, output_image, timestamp_ms):
        """Callback for hand detection results."""
        self.latest_result = result
        self.latest_timestamp = timestamp_ms

    def _calculate_palm_centroid(self, landmarks):
        """Calculate palm centroid and radius."""
        try:
            data = np.array([[landmarks[i].x, landmarks[i].y, landmarks[i].z]
                             for i in self.config.palm_indices])

            centroid = np.mean(data, axis=0)
            recentered = data - centroid

            # Check if hand is flat enough
            _, singular_values, _ = np.linalg.svd(recentered)
            if np.min(singular_values) > self.config.palm_flatness_threshold:
                return None, None

            distances = np.linalg.norm(recentered, axis=1)
            radius = np.min(distances)

            return centroid, radius
        except Exception as e:
            logger.error(f"Palm calculation error: {e}")
            return None, None

    def _filter_hand_position(self, position):
        """
        Filter hand position to reject outliers.

        Args:
            position: [x, y, z] position in TCP frame

        Returns:
            Filtered position or None if rejected
        """
        # Always allow zero position (hand leaving frame)
        if position is None or np.array_equal(position, [0.0, 0.0, 0.0]):
            self.position_history = []  # Clear history when hand leaves
            return position

        position = np.array(position)

        # If we don't have enough history, accept the position
        if len(self.position_history) < 2:
            self.position_history.append(position.copy())
            if len(self.position_history) > self.max_history_size:
                self.position_history.pop(0)
            return position

        # Calculate average of recent positions
        recent_positions = np.array(self.position_history)
        avg_position = np.mean(recent_positions, axis=0)

        # Calculate distance from average
        distance_from_avg = np.linalg.norm(position - avg_position)

        # Accept if within threshold
        if distance_from_avg <= self.filtering_threshold:
            self.position_history.append(position.copy())
            if len(self.position_history) > self.max_history_size:
                self.position_history.pop(0)
            return position
        else:
            # Reject outlier - use last valid position instead
            logger.debug(
                f"Rejected outlier position: distance {distance_from_avg*1000:.1f}mm from average")
            if self.position_history:
                # Return last valid position
                return self.position_history[-1].copy()
            else:
                return position  # Fallback to current position

    def _draw_results(self, frame, landmarks, palm_pos, depth, vector_3d_cam):
        """Draw all visualization elements."""
        h, w = frame.shape[:2]

        # Draw hand landmarks
        if landmarks:
            for hand_landmarks in landmarks:
                # Create flipped landmarks for display
                flipped_landmarks = landmark_pb2.NormalizedLandmarkList()
                flipped_landmarks.landmark.extend([
                    landmark_pb2.NormalizedLandmark(x=1.0-lm.x, y=lm.y, z=lm.z)
                    for lm in hand_landmarks
                ])

                self.mp_drawing.draw_landmarks(
                    frame, flipped_landmarks, self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(
                        color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(
                        color=(0, 0, 255), thickness=2, circle_radius=2)
                )

        # Draw palm info
        if palm_pos and depth > 0:
            palm_x, palm_y, radius = palm_pos
            # Mirror X coordinate for display (common in camera apps for natural feel)
            # But use actual coordinates for depth calculation
            display_x = w - palm_x

            cv2.circle(frame, (display_x, palm_y), radius, (255, 255, 0), 2)
            cv2.putText(frame, f"Depth: {depth:.2f}m", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"Cam: ({vector_3d_cam[0]:.2f}, {vector_3d_cam[1]:.2f}, {vector_3d_cam[2]:.2f})",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            # Show TCP-relative coordinates (now stored as primary camera vector)
            try:
                vector_3d_tcp = self.telemetry.get_camera_vector()
                if vector_3d_tcp != [0.0, 0.0, 0.0]:
                    # Apply calibration offsets for visualization (same as in unified_hand_tracking_state)
                    vector_3d_tcp_corrected = np.array(vector_3d_tcp).copy()
                    # Subtract 64mm Y offset (camera to TCP)
                    vector_3d_tcp_corrected[1] -= 0.064
                    # Subtract 138mm Z offset (gripper extension)
                    vector_3d_tcp_corrected[2] -= 0.138

                    cv2.putText(frame, f"TCP (raw): ({vector_3d_tcp[0]:.3f}, {vector_3d_tcp[1]:.3f}, {vector_3d_tcp[2]:.3f})",
                                (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)
                    cv2.putText(frame, f"TCP (corrected): ({vector_3d_tcp_corrected[0]:.3f}, {vector_3d_tcp_corrected[1]:.3f}, {vector_3d_tcp_corrected[2]:.3f})",
                                (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            except Exception as e:
                logger.debug(f"Failed to display TCP coordinates: {e}")

    def start(self):
        """Start hand tracking in background thread."""
        if self.is_running:
            logger.warning("Hand Tracker is already running...")
            return

        logger.info("Starting HandTracker...")
        try:
            self.is_running = True
            self._setup_detector()

            self.processing_thread = threading.Thread(
                target=self.main_loop, daemon=True)
            self.processing_thread.start()
            logger.info(
                "HandTracker started successfully in a background thread.")
        except Exception as e:
            logger.error(f"Failed to start HandTracker: {e}")
            self._cleanup()
            self.is_running = False
            raise

    def stop(self):
        """Stop hand tracking and cleanup resources."""
        if not self.is_running:
            logger.warning("Hand Tracker is not running.")
            return

        logger.info("Stopping HandTracker...")
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join()
        self._cleanup()
        logger.info("HandTracker stopped.")

    def main_loop(self):
        """Main processing loop for hand tracking."""
        prev_time = time.time()

        while self.is_running:
            try:
                # Get frames from shared camera manager
                color_frame, depth_frame = self.camera_manager.get_frames()
                if color_frame is None or depth_frame is None:
                    continue

                # Process with MediaPipe
                rgb_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                timestamp_ms = int(time.time() * 1000)

                self.landmarker.detect_async(mp_image, timestamp_ms)

                # Process results
                palm_pos = None
                depth = 0
                vector_3d_cam = [0.0, 0.0, 0.0]

                if (self.latest_result and
                    self.latest_result.hand_landmarks and
                        len(self.latest_result.hand_landmarks) > 0):

                    landmarks = self.latest_result.hand_landmarks[0]
                    centroid, radius = self._calculate_palm_centroid(landmarks)

                    if centroid is not None:
                        h, w = color_frame.shape[:2]
                        palm_x = int(centroid[0] * w)
                        # MediaPipe Y (0=top, 1=bottom) matches OpenCV Y (0=top, h=bottom)
                        # No flip needed for pixel coordinates
                        palm_y = int(centroid[1] * h)
                        pixel_radius = int(radius * min(w, h))

                        # Use shared camera manager for depth calculation
                        depth = self.camera_manager.get_average_depth(
                            depth_frame, (palm_x, palm_y), pixel_radius)

                        if depth > 0:
                            # Use shared camera manager for 3D conversion
                            vector_3d_cam = self.camera_manager.pixel_to_3d(
                                palm_x, palm_y, depth)

                            # Calculate TCP-relative coordinates and store as primary camera vector
                            try:
                                from camera_management.camera_transform_module import transform_camera_to_tcp_frame
                                vector_3d_tcp = transform_camera_to_tcp_frame(
                                    vector_3d_cam)

                                # Apply filtering to reject outliers
                                filtered_tcp = self._filter_hand_position(
                                    vector_3d_tcp)

                                # Store filtered TCP-relative coordinates as the primary camera vector
                                self.telemetry.update_camera_vector(
                                    filtered_tcp)
                            except Exception as e:
                                logger.debug(
                                    f"Failed to calculate TCP-relative coordinates: {e}")
                                # Fallback to camera coordinates if transformation fails
                                self.telemetry.update_camera_vector(
                                    vector_3d_cam)

                            radius_vector = self.camera_manager.pixel_to_3d(
                                palm_x - pixel_radius, palm_y, depth)
                            actual_radius = vector_3d_cam[0] - radius_vector[0]
                            self.telemetry.update_radius(actual_radius)
                            palm_pos = (palm_x, palm_y, pixel_radius)
                else:
                    # No hand detected - apply filtering to zero vector
                    filtered_zero = self._filter_hand_position(vector_3d_cam)
                    self.telemetry.update_camera_vector(filtered_zero)

                # Create display frame
                display_frame = cv2.flip(color_frame, 1)

                # Draw everything
                landmarks = self.latest_result.hand_landmarks if self.latest_result else None
                self._draw_results(display_frame, landmarks,
                                   palm_pos, depth, vector_3d_cam)

                # FPS
                current_time = time.time()
                fps = int(1.0 / (current_time - prev_time))
                prev_time = current_time
                cv2.putText(display_frame, f"FPS: {fps}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Display
                cv2.imshow('Hand Tracking', display_frame)
                key = cv2.waitKey(1)
                # Check for Esc key to exit manually
                if key == 27:  # ESC key
                    self.is_running = False

            except Exception as e:
                logger.error(f"Error in hand tracking main loop: {e}")
                self.is_running = False
                break

        # Clean up OpenCV windows in the same thread that created them
        cv2.destroyAllWindows()

    def _cleanup(self):
        """Clean up resources."""
        if self.landmarker:
            self.landmarker.close()
        # Note: cv2.destroyAllWindows() is now called in main_loop() thread
        logger.info("HandTracker cleanup complete")


if __name__ == "__main__":
    # Test standalone functionality
    telemetry = Telemetry()
    camera_manager = CameraManager()

    if camera_manager.initialize():
        hand_tracker = HandTracker(telemetry, camera_manager)
        hand_tracker.start()
        try:
            print("Hand tracking running. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping hand tracking...")
            hand_tracker.stop()
            camera_manager.cleanup()
            print("Hand tracking stopped.")
    else:
        print("Failed to initialize camera")
