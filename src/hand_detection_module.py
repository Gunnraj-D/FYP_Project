import cv2
import mediapipe as mp
import pyrealsense2 as rs
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple
import logging
import threading
from shared_state_joints import SharedState
from config import HANDMODEL_FILEPATH, MIN_HAND_CONFIDENCE, MAX_HAND_DISTANCE, MIN_HAND_DISTANCE

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class Config:
    """Simple configuration class"""
    model_path: str = HANDMODEL_FILEPATH
    max_hands: int = 1
    min_confidence: float = MIN_HAND_CONFIDENCE
    palm_indices: List[int] = None
    palm_flatness_threshold: float = 0.15
    max_tracking_distance: float = MAX_HAND_DISTANCE
    min_tracking_distance: float = MIN_HAND_DISTANCE

    def __post_init__(self):
        if self.palm_indices is None:
            self.palm_indices = [0, 1, 2, 5, 9, 13, 17]


class HandTracker:
    def __init__(self, shared_vector: SharedState, config: Config = None):
        self.config = config or Config()
        self.shared_vector = shared_vector
        self.is_running = False

        # Initialize components
        self.pipeline = None
        self.landmarker = None
        self.intrinsics = None
        self.align = None

        # Detection state
        self.latest_result = None
        self.latest_timestamp = 0
        self.detection_history = []
        self.max_history_length = 10

        # Visualization
        self.mp_drawing = solutions.drawing_utils
        self.mp_hands = solutions.hands

        # Performance tracking
        self.frame_count = 0
        self.detection_count = 0
        self.last_performance_log = time.time()

    def _setup_camera(self):
        """Setup RealSense camera with error handling"""
        try:
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            profile = self.pipeline.start(cfg)
            color_stream = profile.get_stream(rs.stream.color)
            self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            self.align = rs.align(rs.stream.color)

            logger.info("Camera initialized successfully")
        except Exception as e:
            logger.error(f"Camera setup failed: {e}")
            raise

    def _setup_detector(self):
        """Setup MediaPipe hand detector with error handling"""
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
            logger.info("Hand detector initialized successfully")
        except Exception as e:
            logger.error(f"Hand detector setup failed: {e}")
            raise

    def _detection_callback(self, result, output_image, timestamp_ms):
        """Callback for hand detection results with validation"""
        try:
            if result and hasattr(result, 'hand_landmarks') and result.hand_landmarks:
                self.latest_result = result
                self.latest_timestamp = timestamp_ms
                self.detection_count += 1

                # Process the detection
                self._process_detection_result(result)
            else:
                # No hand detected
                self._handle_no_detection()

        except Exception as e:
            logger.error(f"Error in detection callback: {e}")
            self._handle_no_detection()

    def _process_detection_result(self, result):
        """Process and validate detection result"""
        try:
            if not result.hand_landmarks:
                self._handle_no_detection()
                return

            # Get the first detected hand
            hand_landmarks = result.hand_landmarks[0]

            # Calculate confidence based on landmark quality
            confidence = self._calculate_detection_confidence(hand_landmarks)

            # Validate confidence threshold
            if confidence < self.config.min_confidence:
                logger.debug(
                    f"Hand detection confidence too low: {confidence:.3f}")
                self._handle_no_detection()
                return

            # Calculate palm centroid and radius
            palm_centroid, radius = self._calculate_palm_centroid(
                hand_landmarks)
            if palm_centroid is None:
                logger.debug("Palm centroid calculation failed")
                self._handle_no_detection()
                return

            # Get depth at palm position
            depth = self._get_depth_at_point(
                self.latest_depth_frame, palm_centroid, radius)
            if depth <= 0:
                logger.debug("Invalid depth value")
                self._handle_no_detection()
                return

            # Convert to 3D coordinates
            vector_3d = self._pixel_to_3d(
                palm_centroid[0], palm_centroid[1], depth)

            # Validate 3D vector
            if not self._validate_3d_vector(vector_3d):
                logger.debug(f"Invalid 3D vector: {vector_3d}")
                self._handle_no_detection()
                return

            # Update shared state with confidence
            self.shared_vector.update_camera_vector(vector_3d, confidence)
            self.shared_vector.update_radius(radius)

            # Add to detection history
            self._add_to_history(vector_3d, confidence, time.time())

        except Exception as e:
            logger.error(f"Error processing detection result: {e}")
            self._handle_no_detection()

    def _calculate_detection_confidence(self, landmarks) -> float:
        """Calculate confidence based on landmark quality and stability"""
        try:
            # Check landmark visibility
            visible_landmarks = sum(
                1 for lm in landmarks if lm.visibility > 0.5)
            visibility_score = visible_landmarks / len(landmarks)

            # Check landmark presence confidence
            presence_score = getattr(
                self.latest_result, 'hand_presence_confidence', [0.5])[0]

            # Check tracking confidence
            tracking_score = getattr(
                self.latest_result, 'hand_tracking_confidence', [0.5])[0]

            # Combine scores
            confidence = (visibility_score + presence_score +
                          tracking_score) / 3.0

            return min(confidence, 1.0)

        except Exception as e:
            logger.error(f"Error calculating confidence: {e}")
            return 0.0

    def _validate_3d_vector(self, vector_3d: Tuple[float, float, float]) -> bool:
        """Validate 3D vector values"""
        try:
            if len(vector_3d) != 3:
                return False

            x, y, z = vector_3d

            # Check for NaN or infinite values
            if any(not np.isfinite(val) for val in [x, y, z]):
                return False

            # Check distance limits
            distance = np.sqrt(x**2 + y**2 + z**2)
            if distance < self.config.min_tracking_distance or distance > self.config.max_tracking_distance:
                return False

            # Check reasonable bounds (in mm)
            if abs(x) > 5000 or abs(y) > 5000 or abs(z) > 5000:
                return False

            return True

        except Exception as e:
            logger.error(f"Error validating 3D vector: {e}")
            return False

    def _handle_no_detection(self):
        """Handle case when no hand is detected"""
        # Reset camera vector to indicate no detection
        self.shared_vector.update_camera_vector([0, 0, 0], 0.0)
        self.shared_vector.update_radius(0)

    def _add_to_history(self, vector_3d: Tuple[float, float, float], confidence: float, timestamp: float):
        """Add detection to history for stability analysis"""
        self.detection_history.append({
            'vector': vector_3d,
            'confidence': confidence,
            'timestamp': timestamp
        })

        # Keep only recent detections
        if len(self.detection_history) > self.max_history_length:
            self.detection_history.pop(0)

    def _get_frames(self):
        """Get aligned color and depth frames with error handling"""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                logger.warning("Failed to get valid frames")
                return None, None

            # Store depth frame for later use
            self.latest_depth_frame = depth_frame

            return np.asanyarray(color_frame.get_data()), depth_frame

        except Exception as e:
            logger.error(f"Frame capture error: {e}")
            return None, None

    def _calculate_palm_centroid(self, landmarks):
        """Calculate palm centroid and radius with improved validation"""
        try:
            data = np.array([[landmarks[i].x, landmarks[i].y, landmarks[i].z]
                             for i in self.config.palm_indices])

            centroid = np.mean(data, axis=0)
            recentered = data - centroid

            # Check if hand is flat enough
            _, singular_values, _ = np.linalg.svd(recentered)
            if np.min(singular_values) > self.config.palm_flatness_threshold:
                logger.debug("Hand not flat enough for palm detection")
                return None, None

            distances = np.linalg.norm(recentered, axis=1)
            radius = np.min(distances)

            # Validate radius
            if radius <= 0 or not np.isfinite(radius):
                logger.debug(f"Invalid radius calculated: {radius}")
                return None, None

            return centroid, radius

        except Exception as e:
            logger.error(f"Palm calculation error: {e}")
            return None, None

    def _get_depth_at_point(self, depth_frame, center, radius):
        """Get average depth in circular region with validation"""
        try:
            h, w = depth_frame.get_height(), depth_frame.get_width()
            cx, cy = int(center[0]), int(center[1])

            if cx < 0 or cx >= w or cy < 0 or cy >= h:
                logger.debug(
                    f"Center point outside frame bounds: ({cx}, {cy})")
                return 0.0

            # Create circular mask
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (cx, cy), int(radius), 255, -1)

            depth_image = np.asanyarray(depth_frame.get_data())
            valid_depths = depth_image[mask == 255]
            valid_depths = valid_depths[valid_depths > 0]

            if valid_depths.size == 0:
                logger.debug("No valid depth values in region")
                return 0.0

            depth_value = np.mean(valid_depths) * depth_frame.get_units()

            # Validate depth value
            if depth_value <= 0 or not np.isfinite(depth_value):
                logger.debug(f"Invalid depth value: {depth_value}")
                return 0.0

            return depth_value

        except Exception as e:
            logger.error(f"Depth calculation error: {e}")
            return 0.0

    def _pixel_to_3d(self, x, y, depth):
        """Convert pixel coordinates to 3D vector with validation"""
        try:
            if depth <= 0:
                return [0.0, 0.0, 0.0]

            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [x, y], depth)

            # Convert from meters to millimeters
            vector_3d = (point_3d[0] * 1000, point_3d[1]
                         * 1000, point_3d[2] * 1000)

            # Validate the result
            if not all(np.isfinite(val) for val in vector_3d):
                logger.debug(f"Non-finite 3D point: {vector_3d}")
                return [0.0, 0.0, 0.0]

            return vector_3d

        except Exception as e:
            logger.error(f"3D conversion error: {e}")
            return [0.0, 0.0, 0.0]

    def _log_performance_metrics(self):
        """Log performance metrics periodically"""
        current_time = time.time()
        if current_time - self.last_performance_log > 10.0:  # Log every 10 seconds
            fps = self.frame_count / (current_time - self.last_performance_log)
            detection_rate = self.detection_count / max(self.frame_count, 1)

            logger.info(
                f"Performance: FPS={fps:.1f}, Detection Rate={detection_rate:.2%}")

            self.frame_count = 0
            self.detection_count = 0
            self.last_performance_log = current_time

    def start(self):
        """Start hand tracking with error handling"""
        try:
            logger.info("Starting hand tracking...")

            self._setup_camera()
            self._setup_detector()

            self.is_running = True
            self.tracking_thread = threading.Thread(
                target=self._tracking_loop, daemon=True)
            self.tracking_thread.start()

            logger.info("Hand tracking started successfully")

        except Exception as e:
            logger.error(f"Failed to start hand tracking: {e}")
            self.is_running = False
            raise

    def stop(self):
        """Stop hand tracking safely"""
        logger.info("Stopping hand tracking...")
        self.is_running = False

        if hasattr(self, 'tracking_thread') and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)

        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception as e:
                logger.warning(f"Error stopping pipeline: {e}")

        logger.info("Hand tracking stopped")

    def _tracking_loop(self):
        """Main tracking loop with error handling"""
        while self.is_running:
            try:
                self.frame_count += 1

                # Get frames
                color_frame, depth_frame = self._get_frames()
                if color_frame is None or depth_frame is None:
                    time.sleep(0.01)
                    continue

                # Process frame with MediaPipe
                if self.landmarker:
                    # Convert BGR to RGB for MediaPipe
                    rgb_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
                    # Create MediaPipe image
                    mp_image = mp.Image(
                        image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                    self.landmarker.detect_async(
                        mp_image, int(time.time() * 1000))

                # Log performance metrics
                self._log_performance_metrics()

                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)

            except Exception as e:
                logger.error(f"Error in tracking loop: {e}")
                time.sleep(0.1)  # Longer delay on error
