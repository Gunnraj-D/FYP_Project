import pybullet as p
import time
import math

def get_required_joint_angles(robot_urdf_path, current_joint_angles, cartesian_displacement, end_effector_link_name):
    """
    Calculates the required joint angles to move a robot's end-effector by a Cartesian displacement.

    Args:
        robot_urdf_path (str): The file path to the robot's URDF file.
        current_joint_angles (list or tuple): The current angles of the robot's joints.
        cartesian_displacement (list or tuple): The desired change in position [dx, dy, dz].
        end_effector_link_name (str): The name of the end-effector link in the URDF.

    Returns:
        list: The calculated joint angles required to achieve the target pose, or None if no solution is found.
    """
    # --- 1. Setup the simulation environment ---
    # Use a 'DIRECT' connection to run pybullet without a GUI
    physicsClient = p.connect(p.DIRECT)

    # Load the robot from the URDF file
    # Use a fixed base so the robot arm doesn't fall over
    robot = p.loadURDF(robot_urdf_path, useFixedBase=1)

    # --- 2. Find the end-effector link index and identify actuated joints ---
    num_joints = p.getNumJoints(robot)
    end_effector_link_index = -1
    actuated_joint_indices = []

    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        joint_name = info[1].decode('UTF-8')
        link_name = info[12].decode('UTF-8')

        # Find the link index for the end-effector
        if link_name == end_effector_link_name:
            end_effector_link_index = i

        # Identify joints that can be controlled (not fixed)
        if info[2] != p.JOINT_FIXED:
            actuated_joint_indices.append(i)

    if end_effector_link_index == -1:
        raise Exception(f"Could not find a link named '{end_effector_link_name}' in the URDF.")
    
    if len(current_joint_angles) != len(actuated_joint_indices):
         raise Exception(f"Mismatch between number of provided angles ({len(current_joint_angles)}) and actuated joints ({len(actuated_joint_indices)}).")

    # --- 3. Set the robot to its current configuration ---
    for i, joint_index in enumerate(actuated_joint_indices):
        p.resetJointState(robot, joint_index, current_joint_angles[i])

    # --- 4. Calculate the current and target Cartesian poses ---
    # Get the current state of the end-effector (this is our FK step)
    # The result contains:
    # [0] Cartesian position (x,y,z)
    # [1] Cartesian orientation (quaternion x,y,z,w)
    current_link_state = p.getLinkState(robot, end_effector_link_index)
    current_position = current_link_state[0]
    current_orientation = current_link_state[1] # We will keep the orientation the same

    # Calculate the target position
    target_position = [
        current_position[0] + cartesian_displacement[0],
        current_position[1] + cartesian_displacement[1],
        current_position[2] + cartesian_displacement[2]
    ]

    print(f"Current End-Effector Position: {current_position}")
    print(f"Target End-Effector Position:  {target_position}")

    # --- 5. Solve for Inverse Kinematics ---
    # The IK solver needs a target position and optionally a target orientation.
    # We provide the current joint angles as the 'seed' for the solver.
    # PyBullet's IK solver is highly optimized and respects joint limits from the URDF.
    required_joint_angles = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=end_effector_link_index,
        targetPosition=target_position,
        targetOrientation=current_orientation, # Keep the same orientation
        currentPosition=current_joint_angles, # Use current angles as the starting point
        maxNumIterations=100,
        residualThreshold=.01
    )

    # --- 6. Clean up and return ---
    p.disconnect()

    return list(required_joint_angles)


# --- Example Usage ---
if __name__ == "__main__":
    # You MUST replace these with your actual file and parameters
    # For this example, let's assume a 6-joint robot arm
    MY_URDF_FILE = "path/to/your/robot.urdf" # IMPORTANT: CHANGE THIS
    MY_EE_LINK_NAME = "ee_link" # IMPORTANT: Find this name in your URDF file

    # Current state of the robot's 6 joints (in radians)
    current_angles = [0.1, -0.5, 0.2, 0.0, 1.0, 0.0]

    # Desired movement: move 10cm forward (X-axis) and 5cm up (Z-axis)
    # The coordinate system is defined by the URDF's base_link
    desired_displacement = [0.10, 0.0, 0.05] 

    try:
        # Calculate the required joint angles
        target_angles = get_required_joint_angles(
            robot_urdf_path=MY_URDF_FILE,
            current_joint_angles=current_angles,
            cartesian_displacement=desired_displacement,
            end_effector_link_name=MY_EE_LINK_NAME
        )

        print("\n--- Results ---")
        print(f"Current Joint Angles: {[round(a, 3) for a in current_angles]}")
        print(f"Required Joint Angles: {[round(a, 3) for a in target_angles]}")

    except Exception as e:
        print(f"An error occurred: {e}")