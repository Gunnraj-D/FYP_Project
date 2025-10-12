"""
Test script to empirically determine correct hand tracking target values.

Run this with your robot in a known position and your hand at a known location
to see what TCP coordinates are actually being reported.
"""
from control.command_bus import CommandBus
from hand_detection.hand_detection_module import HandTracker
from control.telemetry_store import Telemetry
from camera_management.camera_transform_module import transform_camera_to_tcp_frame
from camera_management.camera_manager import CameraManager
import numpy as np
import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))


print("=" * 70)
print("HAND TRACKING COORDINATE TEST")
print("=" * 70)
print("\nThis script will help you understand what TCP coordinates")
print("are being reported for your hand position.\n")
print("Instructions:")
print("1. Position your robot with TCP facing down")
print("2. Place your hand at a known position relative to TCP")
print("3. Observe the reported coordinates")
print("4. Use Ctrl+C to stop\n")
print("=" * 70)

# Initialize components
telemetry = Telemetry()
commands = CommandBus()
camera = CameraManager()

if not camera.initialize():
    print("❌ Failed to initialize camera!")
    sys.exit(1)

print("✓ Camera initialized")

# Start hand tracker
hand_tracker = HandTracker(
    telemetry=telemetry,
    command_bus=commands,
    camera_manager=camera
)

print("✓ Starting hand tracker...")
hand_tracker.start()
time.sleep(2)  # Let it stabilize

print("\n" + "=" * 70)
print("MONITORING HAND POSITION")
print("=" * 70)
print("\nPlace your hand at different positions and observe the coordinates:")
print("  - TCP (raw) = coordinates before -138mm hack")
print("  - TCP (corrected) = coordinates after -138mm hack")
print("\nFor a hand 5cm below TCP (aligned in X,Y), you should see:")
print("  TCP (corrected) ≈ (0.0, 0.0, 0.05)\n")

try:
    while True:
        # Get current hand position from telemetry
        hand_pos = telemetry.get_camera_vector()

        if hand_pos and hand_pos != [0.0, 0.0, 0.0]:
            hand_pos_tcp = np.array(hand_pos)

            # Apply calibration offsets (same as in unified_hand_tracking_state)
            hand_pos_corrected = hand_pos_tcp.copy()
            # Subtract 64mm Y offset (camera to TCP)
            hand_pos_corrected[1] -= 0.064
            # Subtract 138mm Z offset (gripper extension)
            hand_pos_corrected[2] -= 0.138

            # Calculate distance from ideal target
            target = np.array([0.0, 0.0, 0.05])  # 5cm below TCP
            distance_from_ideal = np.linalg.norm(hand_pos_corrected - target)

            # Clear screen (for cleaner output)
            print("\r" + " " * 100, end="")
            print(
                f"\rTCP (raw):       ({hand_pos_tcp[0]:>6.3f}, {hand_pos_tcp[1]:>6.3f}, {hand_pos_tcp[2]:>6.3f})", end="")
            print(
                f"  |  TCP (corrected): ({hand_pos_corrected[0]:>6.3f}, {hand_pos_corrected[1]:>6.3f}, {hand_pos_corrected[2]:>6.3f})", end="")
            print(
                f"  |  Δ from 5cm target: {distance_from_ideal*1000:>5.1f}mm", end="", flush=True)
        else:
            print("\r" + " " * 100, end="")
            print("\rNo hand detected" + " " * 80, end="", flush=True)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\n" + "=" * 70)
    print("STOPPING")
    print("=" * 70)
    hand_tracker.stop()
    camera.cleanup()
    print("\n✓ Stopped successfully")
