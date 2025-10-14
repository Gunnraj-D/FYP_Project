import pybullet as p
import pybullet_data
import time
import os

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Load your combined URDF
urdf_path = os.path.abspath(
    "src/resources/robot_models/iiwa14_with_robotiq85.urdf")
robot_id = p.loadURDF(
    urdf_path,
    basePosition=[0, 0, 0],
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE
)

# Print all joints to find gripper control joints
print("\n=== Joint Information ===")
num_joints = p.getNumJoints(robot_id)
gripper_joint_idx = None

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]  # 0=revolute, 1=prismatic, 4=fixed

    print(f"Joint {i}: {joint_name} (type: {joint_type})")

    # Find the main gripper control joint
    if joint_name == "finger_joint":
        gripper_joint_idx = i
        print(f"  --> GRIPPER CONTROL JOINT FOUND at index {i}")

# Test gripper control
if gripper_joint_idx is not None:
    print(f"\n=== Testing Gripper Control ===")

    # Open gripper (position = 0)
    print("Opening gripper...")
    p.setJointMotorControl2(
        robot_id,
        gripper_joint_idx,
        p.POSITION_CONTROL,
        targetPosition=0.0,
        force=20
    )

    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)

    # Close gripper (position = 0.725)
    print("Closing gripper...")
    p.setJointMotorControl2(
        robot_id,
        gripper_joint_idx,
        p.POSITION_CONTROL,
        targetPosition=0.725,
        force=20
    )

    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)

# Keep simulation running
print("\nSimulation running. Press Ctrl+C to exit.")
while True:
    p.stepSimulation()
    time.sleep(0.01)
