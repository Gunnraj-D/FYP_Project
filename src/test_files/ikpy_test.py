# from ikpy.chain import Chain
# from ikpy.inverse_kinematics import inverse_kinematic_optimization
import time
# import numpy as np

# my_chain = Chain.from_urdf_file(
#     "src/resources/models/iiwa14.urdf",
#     base_elements=["iiwa_link_0"], 
#     active_links_mask=[
#         False,  # base link - fixed
#         True,   # joint 1
#         True,   # joint 2
#         True,   # joint 3
#         True,   # joint 4
#         True,   # joint 5
#         True,   # joint 6
#         True,   # joint 7
#         False   # end-effector - fixed
#     ]
# )

# result = my_chain.inverse_kinematics([2,2,2])
# print(result)
# print()
# # start_time = time.perf_counter_ns()
# # inverse_kinematic_optimization(my_chain, )
# # end_time = time.perf_counter_ns()

# # inverse_kinematic_optimization(my_chain, )

# # print("Time taken for Optimisation:", (end_time - start_time)/1000000, "milliseconds")

# print(my_chain.forward_kinematics(result))
# # desired_direction = np.array([2, 2, 2])  # Specify your direction and a reasonable step
# # delta_q = np.linalg.pinv(jacobian[:3,:]) @ desired_direction
# # q1 = q0 + delta_q

# # print(q1)

from ikpy.chain import Chain
import numpy as np

# Load your robot chain (from URDF or DH params)
my_chain = Chain.from_urdf_file(
    "src/resources/models/iiwa14.urdf",
    base_elements=["iiwa_link_0"], 
    active_links_mask=[
        False,  # base link - fixed
        True,   # joint 1
        True,   # joint 2
        True,   # joint 3
        True,   # joint 4
        True,   # joint 5
        True,   # joint 6
        True,   # joint 7
        False   # end-effector - fixed
    ]
)

start_time = time.perf_counter_ns()
# Current joint angles streamed from the robot
q_current = [0.0, 0.0, 0.1, 0.05, 0, 0, 0, 0, 0]

# Current end-effector position (for reference)
current_ee_pos = my_chain.forward_kinematics(q_current)[:3, 3]

# Desired small displacement in Cartesian space (e.g. 1cm in x)
target_position = current_ee_pos + np.array([0.01, 0, 0])

# Call inverse kinematics with initial_position set to current joint angles
q_new = my_chain.inverse_kinematics(
    target_position,
    initial_position=q_current
)
end_time = time.perf_counter_ns()

print("New joint angles:", q_new)

print("Time taken for Optimisation:", (end_time - start_time)/1000000, "milliseconds")
