"""
Example script showing how to use the ZED Joint Receiver.

This demonstrates different usage patterns:
1. Basic usage with callback
2. Polling for latest frame
3. Extracting specific joint positions
4. Integration with robot control system
"""

from hand_detection.zed_joint_receiver import ZEDJointReceiver, FrameData
import logging
import time
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def example_1_callback():
    """Example 1: Using callback for real-time processing."""
    print("\n=== Example 1: Callback-based processing ===\n")

    def on_frame(frame_data: FrameData):
        """Process each frame as it arrives."""
        print(
            f"Frame {frame_data.frame}: {len(frame_data.skeletons)} skeleton(s)")

        for skeleton in frame_data.skeletons:
            # Get specific joint positions (e.g., right hand)
            right_hand = skeleton.get_joint_position("WRIST_RIGHT")
            if right_hand:
                print(f"  Right wrist: {right_hand}")

    receiver = ZEDJointReceiver(callback=on_frame)
    receiver.start()

    print("Waiting for Unity connection...")
    try:
        time.sleep(30)  # Run for 30 seconds
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


def example_2_polling():
    """Example 2: Polling for latest frame."""
    print("\n=== Example 2: Polling for latest data ===\n")

    receiver = ZEDJointReceiver()
    receiver.start()

    print("Waiting for Unity connection...")

    try:
        last_frame_number = -1

        while True:
            # Get latest frame data
            frame_data = receiver.get_latest_frame()

            if frame_data and frame_data.frame != last_frame_number:
                last_frame_number = frame_data.frame

                print(f"\nFrame {frame_data.frame}")
                print(f"  Skeletons tracked: {len(frame_data.skeletons)}")

                # Get first skeleton if available
                if frame_data.skeletons:
                    skeleton = frame_data.skeletons[0]
                    print(
                        f"  Skeleton {skeleton.skeleton_id} has {len(skeleton.joints)} joints")

                    # Print positions of key joints
                    key_joints = ["WRIST_RIGHT", "WRIST_LEFT",
                                  "HAND_RIGHT", "HAND_LEFT"]
                    for joint_name in key_joints:
                        position = skeleton.get_joint_position(joint_name)
                        if position:
                            print(
                                f"    {joint_name}: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")

            # Check connection status
            if not receiver.is_connected():
                print("Waiting for connection...")

            time.sleep(0.1)  # Poll at ~10Hz

    except KeyboardInterrupt:
        pass
    finally:
        stats = receiver.get_stats()
        print(f"\nFinal stats: {stats}")
        receiver.stop()


def example_3_context_manager():
    """Example 3: Using context manager."""
    print("\n=== Example 3: Context manager usage ===\n")

    with ZEDJointReceiver() as receiver:
        print("Receiver started in context manager")

        try:
            for i in range(50):  # Check 50 times
                frame_data = receiver.get_latest_frame()

                if frame_data:
                    print(
                        f"Frame {frame_data.frame}: {len(frame_data.skeletons)} skeleton(s)")

                time.sleep(0.2)

        except KeyboardInterrupt:
            pass

    print("Receiver stopped automatically")


def example_4_hand_position_tracking():
    """Example 4: Track hand position for robot interaction."""
    print("\n=== Example 4: Hand position tracking ===\n")

    def track_hand_positions(frame_data: FrameData):
        """Track hand positions for collision avoidance or interaction."""
        for skeleton in frame_data.skeletons:
            # Get both hand positions
            right_hand = skeleton.get_joint_by_name("WRIST_RIGHT")
            left_hand = skeleton.get_joint_by_name("WRIST_LEFT")

            # Check if hands are in workspace
            WORKSPACE_BOUNDS = {
                'x': (-0.5, 0.5),
                'y': (0.0, 1.0),
                'z': (0.0, 0.8)
            }

            def is_in_workspace(joint_data):
                if not joint_data:
                    return False
                return (WORKSPACE_BOUNDS['x'][0] <= joint_data.x <= WORKSPACE_BOUNDS['x'][1] and
                        WORKSPACE_BOUNDS['y'][0] <= joint_data.y <= WORKSPACE_BOUNDS['y'][1] and
                        WORKSPACE_BOUNDS['z'][0] <= joint_data.z <= WORKSPACE_BOUNDS['z'][1])

            if is_in_workspace(right_hand):
                print(
                    f"⚠️  Right hand in workspace at ({right_hand.x:.3f}, {right_hand.y:.3f}, {right_hand.z:.3f})")

            if is_in_workspace(left_hand):
                print(
                    f"⚠️  Left hand in workspace at ({left_hand.x:.3f}, {left_hand.y:.3f}, {left_hand.z:.3f})")

    receiver = ZEDJointReceiver(callback=track_hand_positions)
    receiver.start()

    print("Monitoring workspace for hand intrusion...")
    print("Press Ctrl+C to stop")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


def example_5_export_to_file():
    """Example 5: Record and export joint data."""
    print("\n=== Example 5: Recording joint data ===\n")

    import json

    recorded_frames = []

    def record_frame(frame_data: FrameData):
        """Record frame data to list."""
        recorded_frames.append(frame_data.to_dict())
        if len(recorded_frames) % 10 == 0:
            print(f"Recorded {len(recorded_frames)} frames...")

    receiver = ZEDJointReceiver(callback=record_frame)
    receiver.start()

    print("Recording for 10 seconds...")

    try:
        time.sleep(10)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()

        # Export to JSON file
        output_file = "zed_tracking_data.json"
        with open(output_file, 'w') as f:
            json.dump(recorded_frames, f, indent=2)

        print(f"\nRecorded {len(recorded_frames)} frames to {output_file}")


if __name__ == "__main__":
    print("ZED Joint Receiver Examples")
    print("=" * 50)
    print("Make sure Unity with ZEDJointSender is running!")
    print("=" * 50)

    # Choose which example to run
    examples = {
        '1': ("Callback-based processing", example_1_callback),
        '2': ("Polling for latest data", example_2_polling),
        '3': ("Context manager usage", example_3_context_manager),
        '4': ("Hand position tracking", example_4_hand_position_tracking),
        '5': ("Record and export data", example_5_export_to_file),
    }

    print("\nAvailable examples:")
    for key, (description, _) in examples.items():
        print(f"  {key}. {description}")

    choice = input("\nSelect example (1-5, or 'all' to run all): ").strip()

    if choice.lower() == 'all':
        for _, func in examples.values():
            func()
            print("\n" + "=" * 50 + "\n")
    elif choice in examples:
        examples[choice][1]()
    else:
        print("Invalid choice. Running example 2 (polling)...")
        example_2_polling()
