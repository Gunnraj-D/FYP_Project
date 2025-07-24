from hand_detection_module import HandTracker
from robot_communication import RobotCommunication
from shared_state import SharedState
import threading
from time import sleep

def main():
    shared_state = SharedState()
    hand_tracker = HandTracker(shared_state)
    robot_communicator = RobotCommunication(shared_state)
    
    hand_tracker.start()

    robot_thread = threading.Thread(target=robot_communicator.start)
    robot_thread.daemon = True
    robot_thread.start()

    try:
        print("System running. Press Ctrl+C to stop.")
        while True:
            sleep(1)

    except KeyboardInterrupt:
        print("\nStopping system...")
        hand_tracker.stop()
        robot_communicator.stop()
        print("System stopped.")

if __name__ == "__main__":
    main()