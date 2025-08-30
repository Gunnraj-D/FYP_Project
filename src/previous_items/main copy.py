from hand_detection_module import HandTracker
import numpy as np
from FRI_Interface import KukaFRIInterface
from shared_state_joints import SharedState
from config import FRI_IP, FRI_PORT, LOOP_RATE_MS, URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
from camera_transform_module import pose_to_homogeneous, transform_camera_to_base
from kinematics_solver import InverseKinematicsSolver
import time
# from Mock_FRI import RealisticMockKukaFRI


def main():
    shared_state = SharedState()
    hand_tracker = HandTracker(shared_state)
    robot = KukaFRIInterface(FRI_IP, FRI_PORT)
    # robot = RealisticMockKukaFRI()
    ik_solver = InverseKinematicsSolver(
        URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS)

    hand_tracker.start()
    robot.connect()

    loop_start_time = 0
    remaining_time = 0

    try:
        print("\n\nSystem running. Press Ctrl+C to stop.\n")

        while True:
            loop_start_time = time.time()

            camera_vector = shared_state.get_camera_vector()

            if camera_vector == [0.0, 0.0, 0.0]:
                continue

            tcp_pose = robot.get_cartesian_pose()
            current_joint_angles = robot.get_joint_positions()

            hand_location_in_base = transform_camera_to_base(
                camera_vector, pose_to_homogeneous(tcp_pose))
            
            current_joint_angles = np.insert(current_joint_angles, 0, 0.0)
            current_joint_angles = np.append(current_joint_angles, 0.0)

            target_joints = ik_solver.solve_XYZ(
                hand_location_in_base, current_joint_angles)
            
            target_joints = target_joints[1:8]

            robot.set_joint_positions(target_joints)

            remaining_time = LOOP_RATE_MS - (time.time() - loop_start_time)

            if remaining_time > 0:
                time.sleep(remaining_time / 1000)
            elif remaining_time < 0:
                print(
                    f"Loop exceeded {LOOP_RATE_MS} ms loop rate by {remaining_time} ms")

    except KeyboardInterrupt:
        print("\nStopping due to keyboard interrupt...")
    except Exception as e:
        print("\nStopping due to exception:")
        print(e)
    finally:
        print("\nStopping system...")
        hand_tracker.stop()
        robot.disconnect()
        print("System stopped.")


if __name__ == "__main__":
    main()
