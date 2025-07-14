import cv2
import mediapipe as mp
import pyrealsense2 as rs
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import time

# --- Configuration ---
MODEL_PATH = r"C:\Users\Raj\Documents\Year 4\FYP\FYP_Project\models\hand_landmarker.task" # Make sure this path is correct
CAP_DEVICE = 0 # 0 for default webcam, change if you have multiple cameras
MAX_NUM_HANDS = 1 # Detect up to 2 hands
MIN_HAND_DETECTION_CONFIDENCE = 0.5
MIN_HAND_PRESENCE_CONFIDENCE = 0.5
MIN_TRACKING_CONFIDENCE = 0.5

# --- Configure Camera ---
pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)

pipe.start(cfg)


# --- Global variable to store the latest detection results ---
latest_hand_detection_result = None
latest_frame_timestamp = 0

# --- Callback function for asynchronous detection ---
def result_callback(result: 'vision.HandLandmarkerResult', output_image: mp.Image, timestamp_ms: int):
    global latest_hand_detection_result, latest_frame_timestamp
    latest_hand_detection_result = result
    latest_frame_timestamp = timestamp_ms

def main():
    # --- Initialize MediaPipe Hand Landmarker ---
    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.LIVE_STREAM,
        num_hands=MAX_NUM_HANDS,
        min_hand_detection_confidence=MIN_HAND_DETECTION_CONFIDENCE,
        min_hand_presence_confidence=MIN_HAND_PRESENCE_CONFIDENCE,
        min_tracking_confidence=MIN_TRACKING_CONFIDENCE,
        result_callback=result_callback
    )

    landmarker = vision.HandLandmarker.create_from_options(options)

    # # --- Initialize OpenCV for webcam input ---
    # cap = cv2.VideoCapture(CAP_DEVICE)
    # if not cap.isOpened():
    #     print(f"Error: Could not open video device {CAP_DEVICE}")
    #     return

    # --- Drawing setup ---
    mp_drawing = solutions.drawing_utils
    mp_hands = solutions.hands

    # --- Frame processing loop ---
    prev_frame_time = 0
    new_frame_time = 0

    while True:
        frame = pipe.wait_for_frames()
        frame = frame.get_color_frame()
        frame = np.asanyarray(frame.get_data())

        # Flip the frame horizontally for a more natural mirror view
        frame = cv2.flip(frame, 1)

        # Convert the BGR image to RGB as MediaPipe expects RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create a MediaPipe Image object
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # Get current timestamp in milliseconds
        timestamp_ms = int(time.time() * 1000)

        # Send image to the landmarker asynchronously
        landmarker.detect_async(mp_image, timestamp_ms)

        # --- Draw landmarks on the frame (using the latest result) ---
        if latest_hand_detection_result and latest_hand_detection_result.hand_landmarks:
            normal_vector, centroid = None, None

            for hand_landmarks in latest_hand_detection_result.hand_landmarks:
                hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                hand_landmarks_proto.landmark.extend([
                    landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z)
                    for landmark in hand_landmarks
                ])
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks_proto,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2), # Green landmarks
                    mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2) # Red connections
                )

                normal_vector, centroid = open_flat_hand(hand_landmarks)

            if normal_vector is not None:
                h, w, _ = frame.shape

                # Convert normalized centroid to pixel coordinates
                start_point = np.array([centroid[0] * w, centroid[1] * h], dtype=int)

                # Define a scale for how long the arrow should be
                scale = 100  # adjust as needed
                direction = normal_vector[:2]  # Use only x, y for 2D drawing
                end_point = start_point + (direction * scale).astype(int)

                # Ensure types are tuples of ints
                start_point = tuple(start_point)
                end_point = tuple(end_point)

                vector_color = (0, 255, 0)
                vector_thickness = 2
                tip_length = 0.3

                cv2.arrowedLine(frame, start_point, end_point, vector_color, vector_thickness, cv2.LINE_AA, 0, tip_length)

        # --- Display FPS ---
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # --- Display the frame ---
        cv2.imshow('MediaPipe Hand Landmarks', frame)

        # --- Exit on 'q' press ---
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # --- Release resources ---
    landmarker.close()
    # cap.release()
    cv2.destroyAllWindows()

# flat hand detection
def open_flat_hand(landmarks):
    data = [[landmark.x, landmark.y, landmark.z] for landmark in landmarks]
    data = np.array(data)
    centroid = np.mean(data, axis=0)

    recentered_data = data - centroid

    _, D, V = np.linalg.svd(recentered_data)
    
    min_column = np.argmin(D)

    if D[min_column] > .07:
        return None, None
    
    normal_vector = V[:, min_column]
    return normal_vector, centroid


if __name__ == '__main__':
    main()