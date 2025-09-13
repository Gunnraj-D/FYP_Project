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
from config.config import HANDMODEL_FILEPATH

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

    def _draw_results(self, frame, landmarks, palm_pos, depth, vector_3d):
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
            flipped_x = w - palm_x

            cv2.circle(frame, (flipped_x, palm_y), radius, (255, 255, 0), 2)
            cv2.putText(frame, f"Depth: {depth:.2f}m", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"3D: ({vector_3d[0]:.2f}, {vector_3d[1]:.2f}, {vector_3d[2]:.2f})",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    def start(self):
        """Start hand tracking in background thread."""
        if self.is_running:
            logger.warning("Hand Tracker is already running...")
            return

        logger.info("Starting HandTracker...")
        try:
            self.is_running = True
            self._setup_detector()

            self.processing_thread = threading.Thread(target=self.main_loop)
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
            vector_3d = [0.0, 0.0, 0.0]

            if (self.latest_result and
                self.latest_result.hand_landmarks and
                    len(self.latest_result.hand_landmarks) > 0):

                landmarks = self.latest_result.hand_landmarks[0]
                centroid, radius = self._calculate_palm_centroid(landmarks)

                if centroid is not None:
                    h, w = color_frame.shape[:2]
                    palm_x = int(centroid[0] * w)
                    palm_y = int(centroid[1] * h)
                    pixel_radius = int(radius * min(w, h))

                    # Use shared camera manager for depth calculation
                    depth = self.camera_manager.get_average_depth(
                        depth_frame, (palm_x, palm_y), pixel_radius)

                    if depth > 0:
                        # Use shared camera manager for 3D conversion
                        vector_3d = self.camera_manager.pixel_to_3d(
                            palm_x, palm_y, depth)
                        self.telemetry.update_camera_vector(vector_3d)

                        radius_vector = self.camera_manager.pixel_to_3d(
                            palm_x - pixel_radius, palm_y, depth)
                        actual_radius = vector_3d[0] - radius_vector[0]
                        self.telemetry.update_radius(actual_radius)
                        palm_pos = (palm_x, palm_y, pixel_radius)
            else:
                self.telemetry.update_camera_vector(vector_3d)

            # Create display frame
            display_frame = cv2.flip(color_frame, 1)

            # Draw everything
            landmarks = self.latest_result.hand_landmarks if self.latest_result else None
            self._draw_results(display_frame, landmarks,
                               palm_pos, depth, vector_3d)

            # FPS
            current_time = time.time()
            fps = int(1.0 / (current_time - prev_time))
            prev_time = current_time
            cv2.putText(display_frame, f"FPS: {fps}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display
            cv2.imshow('Hand Tracking', display_frame)
            cv2.waitKey(1)

    def _cleanup(self):
        """Clean up resources."""
        if self.landmarker:
            self.landmarker.close()
        cv2.destroyAllWindows()
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
