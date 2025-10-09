from enum import Enum, auto


class States(Enum):
    PICK_UP = auto()
    MOVE_TO = auto()
    HUMAN_AWARE_MOVE_TO = auto()  # Human-aware path planning state
    HUMAN_HANDOFF_APPROACH = auto()  # Approach human hand for handoff
    UNIFIED_HAND_TRACKING = auto()
    GRIPPER_CONTROL = auto()
    GRASPING = auto()  # New state for GGCNN2-based grasping
    GENERATE_PICKUP = auto()  # New state for generating grasp poses
    ERROR = auto()
