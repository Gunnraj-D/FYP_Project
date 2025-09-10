from enum import Enum, auto

class States(Enum):
    PICK_UP = auto()
    MOVE_TO = auto()
    TRACKING = auto()
    GRIPPER_CONTROL = auto()
    ERROR = auto()