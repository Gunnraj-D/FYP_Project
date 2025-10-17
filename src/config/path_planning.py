"""
Path Planning Configuration

This module contains skeleton joint positions extracted from ZED camera analysis.
These positions represent stable median locations for path planning and human-aware
robot control.

Data Source: 60-second skeleton capture with 20-second positioning delay
Analysis Date: 2025-10-18 02:42:20
Skeleton ID: 67
Sample Count: 302 frames analyzed (99.5% presence rate)

All positions are in meters relative to the ZED camera coordinate frame.
"""

from typing import Dict, Tuple, NamedTuple, List, Optional, Any
import time
from dataclasses import dataclass, field


# ============================================================================
# ZED COMPATIBLE DATA STRUCTURES
# ============================================================================

@dataclass
class ConfigJointData:
    """Represents a single joint position (compatible with ZED JointData format)."""
    joint_name: str
    x: float
    y: float
    z: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            'joint_name': self.joint_name,
            'x': self.x,
            'y': self.y,
            'z': self.z
        }

    def to_position(self) -> List[float]:
        """Return as [x, y, z] list."""
        return [self.x, self.y, self.z]


@dataclass
class ConfigSkeletonData:
    """Represents a tracked skeleton with all its joints (compatible with ZED SkeletonData format)."""
    skeleton_id: int
    joints: List[ConfigJointData] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'skeleton_id': self.skeleton_id,
            'joints': [joint.to_dict() for joint in self.joints]
        }

    def get_joint_by_name(self, joint_name: str) -> Optional[ConfigJointData]:
        """Get a specific joint by name."""
        for joint in self.joints:
            if joint.joint_name == joint_name:
                return joint
        return None

    def get_joint_position(self, joint_name: str) -> Optional[List[float]]:
        """Get joint position as [x, y, z] or None if not found."""
        joint = self.get_joint_by_name(joint_name)
        return joint.to_position() if joint else None


@dataclass
class ConfigFrameData:
    """Represents a complete frame of skeleton tracking data (compatible with ZED FrameData format)."""
    frame: int
    skeletons: List[ConfigSkeletonData] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'frame': self.frame,
            'timestamp': self.timestamp,
            'skeletons': [skeleton.to_dict() for skeleton in self.skeletons]
        }

    def get_skeleton(self, skeleton_id: int) -> Optional[ConfigSkeletonData]:
        """Get a specific skeleton by ID."""
        for skeleton in self.skeletons:
            if skeleton.skeleton_id == skeleton_id:
                return skeleton
        return None


# ============================================================================
# MEDIAN JOINT POSITIONS (from skeleton analysis)
# ============================================================================

# Core body joints (most stable for path planning)
CORE_BODY_JOINTS = {
    'PELVIS': ConfigJointData('PELVIS', 0.981, 0.264, -0.034),
    'SPINE_1': ConfigJointData('SPINE_1', 0.975, 0.262, 0.032),
    'SPINE_2': ConfigJointData('SPINE_2', 0.961, 0.260, 0.154),
    'SPINE_3': ConfigJointData('SPINE_3', 0.946, 0.260, 0.276),
    'NECK': ConfigJointData('NECK', 0.927, 0.263, 0.435),
}

# Head joints (good for head tracking)
HEAD_JOINTS = {
    'NOSE': ConfigJointData('NOSE', 0.854, 0.230, 0.565),
    'LEFT_EYE': ConfigJointData('LEFT_EYE', 0.885, 0.206, 0.609),
    'RIGHT_EYE': ConfigJointData('RIGHT_EYE', 0.876, 0.257, 0.616),
    'LEFT_EAR': ConfigJointData('LEFT_EAR', 0.970, 0.193, 0.574),
    'RIGHT_EAR': ConfigJointData('RIGHT_EAR', 0.948, 0.311, 0.589),
}

# Shoulder joints (good for arm tracking)
SHOULDER_JOINTS = {
    'LEFT_SHOULDER': ConfigJointData('LEFT_SHOULDER', 0.980, 0.093, 0.378),
    'RIGHT_SHOULDER': ConfigJointData('RIGHT_SHOULDER', 0.967, 0.435, 0.401),
    'LEFT_CLAVICLE': ConfigJointData('LEFT_CLAVICLE', 0.983, 0.197, 0.394),
    'RIGHT_CLAVICLE': ConfigJointData('RIGHT_CLAVICLE', 0.977, 0.331, 0.392),
}

# Arm joints (moderate stability)
ARM_JOINTS = {
    # Most stable arm joint
    'LEFT_ELBOW': ConfigJointData('LEFT_ELBOW', 1.007, 0.076, 0.134),
    'RIGHT_ELBOW': ConfigJointData('RIGHT_ELBOW', 0.857, 0.440, 0.180),
    # Most stable hand joint
    'LEFT_WRIST': ConfigJointData('LEFT_WRIST', 0.997, 0.051, -0.109),
    'RIGHT_WRIST': ConfigJointData('RIGHT_WRIST', 0.614, 0.433, 0.139),
}

# Hand joints (least stable, use with caution)
HAND_JOINTS = {
    'LEFT_HAND_INDEX_1': ConfigJointData('LEFT_HAND_INDEX_1', 0.931, 0.038, -0.190),
    'LEFT_HAND_MIDDLE_4': ConfigJointData('LEFT_HAND_MIDDLE_4', 0.912, 0.045, -0.264),
    'LEFT_HAND_PINKY_1': ConfigJointData('LEFT_HAND_PINKY_1', 0.972, 0.072, -0.212),
    'LEFT_HAND_THUMB_4': ConfigJointData('LEFT_HAND_THUMB_4', 0.892, 0.060, -0.173),
    'RIGHT_HAND_INDEX_1': ConfigJointData('RIGHT_HAND_INDEX_1', 0.508, 0.438, 0.137),
    'RIGHT_HAND_MIDDLE_4': ConfigJointData('RIGHT_HAND_MIDDLE_4', 0.441, 0.414, 0.109),
    'RIGHT_HAND_PINKY_1': ConfigJointData('RIGHT_HAND_PINKY_1', 0.524, 0.386, 0.102),
    'RIGHT_HAND_THUMB_4': ConfigJointData('RIGHT_HAND_THUMB_4', 0.499, 0.434, 0.182),
}

# Hip and leg joints (good for lower body tracking)
LEG_JOINTS = {
    'LEFT_HIP': ConfigJointData('LEFT_HIP', 0.981, 0.177, -0.092),
    'RIGHT_HIP': ConfigJointData('RIGHT_HIP', 0.983, 0.353, -0.092),
    'LEFT_KNEE': ConfigJointData('LEFT_KNEE', 0.883, 0.162, -0.527),
    'RIGHT_KNEE': ConfigJointData('RIGHT_KNEE', 0.887, 0.337, -0.524),
    'LEFT_ANKLE': ConfigJointData('LEFT_ANKLE', 0.922, 0.175, -0.929),
    'RIGHT_ANKLE': ConfigJointData('RIGHT_ANKLE', 0.930, 0.332, -0.926),
}

# Foot joints (moderate stability)
FOOT_JOINTS = {
    'LEFT_HEEL': ConfigJointData('LEFT_HEEL', 0.986, 0.185, -0.982),
    'RIGHT_HEEL': ConfigJointData('RIGHT_HEEL', 0.994, 0.340, -0.979),
    'LEFT_BIG_TOE': ConfigJointData('LEFT_BIG_TOE', 0.787, 0.195, -1.002),
    'RIGHT_BIG_TOE': ConfigJointData('RIGHT_BIG_TOE', 0.802, 0.284, -0.999),
    'LEFT_SMALL_TOE': ConfigJointData('LEFT_SMALL_TOE', 0.835, 0.133, -0.999),
    'RIGHT_SMALL_TOE': ConfigJointData('RIGHT_SMALL_TOE', 0.833, 0.357, -0.995),
}

# Combined dictionary of all joints
ALL_JOINTS = {
    **CORE_BODY_JOINTS,
    **HEAD_JOINTS,
    **SHOULDER_JOINTS,
    **ARM_JOINTS,
    **HAND_JOINTS,
    **LEG_JOINTS,
    **FOOT_JOINTS,
}

# ============================================================================
# PATH PLANNING CONFIGURATION
# ============================================================================

# Joint stability ratings (based on jitter analysis)
JOINT_STABILITY = {
    # Most stable joints (best for path planning)
    'EXCELLENT': ['PELVIS', 'SPINE_1', 'SPINE_2', 'SPINE_3', 'NECK'],

    # Good stability (reliable for path planning)
    'GOOD': ['LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_CLAVICLE', 'RIGHT_CLAVICLE',
             'LEFT_HIP', 'RIGHT_HIP'],

    # Moderate stability (use with caution)
    'MODERATE': ['LEFT_ELBOW', 'RIGHT_ELBOW', 'LEFT_WRIST', 'RIGHT_WRIST',
                 'LEFT_KNEE', 'RIGHT_KNEE', 'LEFT_ANKLE', 'RIGHT_ANKLE'],

    # Poor stability (avoid for critical path planning)
    'POOR': ['LEFT_HEEL', 'RIGHT_HEEL', 'LEFT_BIG_TOE', 'RIGHT_BIG_TOE',
             'LEFT_SMALL_TOE', 'RIGHT_SMALL_TOE'],

    # Very poor stability (hand joints - use only for rough tracking)
    'VERY_POOR': list(HAND_JOINTS.keys()) + ['LEFT_EYE', 'RIGHT_EYE', 'LEFT_EAR', 'RIGHT_EAR', 'NOSE']
}

# Recommended joints for different path planning tasks
RECOMMENDED_JOINTS = {
    'TORSO_TRACKING': ['PELVIS', 'SPINE_1', 'SPINE_2', 'SPINE_3'],
    'HEAD_TRACKING': ['NECK'],  # Most stable head reference
    'ARM_TRACKING': ['LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_ELBOW', 'RIGHT_ELBOW'],
    'HAND_TRACKING': ['LEFT_WRIST', 'RIGHT_WRIST'],  # Avoid finger joints
    'LOWER_BODY': ['LEFT_HIP', 'RIGHT_HIP', 'LEFT_KNEE', 'RIGHT_KNEE'],
    'FULL_BODY': ['PELVIS', 'NECK', 'LEFT_SHOULDER', 'RIGHT_SHOULDER',
                  'LEFT_WRIST', 'RIGHT_WRIST', 'LEFT_HIP', 'RIGHT_HIP']
}

# Safety margins for human-aware path planning (meters)
SAFETY_MARGINS = {
    'CRITICAL_JOINTS': 0.3,  # Core body (pelvis, spine, neck)
    'IMPORTANT_JOINTS': 0.2,  # Shoulders, hips
    'LIMB_JOINTS': 0.15,     # Elbows, knees, wrists
    'HAND_JOINTS': 0.1,      # Hands and fingers
    'FOOT_JOINTS': 0.1,      # Feet and toes
}

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================


def get_joint_position(joint_name: str) -> ConfigJointData:
    """
    Get the median position of a specific joint.

    Args:
        joint_name: Name of the joint (e.g., 'PELVIS', 'LEFT_WRIST')

    Returns:
        ConfigJointData with x, y, z coordinates

    Raises:
        KeyError: If joint_name is not found
    """
    if joint_name not in ALL_JOINTS:
        available = ', '.join(sorted(ALL_JOINTS.keys()))
        raise KeyError(
            f"Joint '{joint_name}' not found. Available joints: {available}")

    return ALL_JOINTS[joint_name]


def get_recommended_joints(task: str) -> list[str]:
    """
    Get recommended joints for a specific path planning task.

    Args:
        task: Task type ('TORSO_TRACKING', 'HEAD_TRACKING', 'ARM_TRACKING', etc.)

    Returns:
        List of recommended joint names
    """
    if task not in RECOMMENDED_JOINTS:
        available = ', '.join(RECOMMENDED_JOINTS.keys())
        raise KeyError(
            f"Task '{task}' not found. Available tasks: {available}")

    return RECOMMENDED_JOINTS[task]


def get_safety_margin(joint_name: str) -> float:
    """
    Get the recommended safety margin for a joint.

    Args:
        joint_name: Name of the joint

    Returns:
        Safety margin in meters
    """
    # Check if it's a critical joint (core body)
    if joint_name in JOINT_STABILITY['EXCELLENT']:
        return SAFETY_MARGINS['CRITICAL_JOINTS']

    # Check if it's an important joint (shoulders, hips)
    if joint_name in JOINT_STABILITY['GOOD']:
        return SAFETY_MARGINS['IMPORTANT_JOINTS']

    # Check if it's a limb joint
    if joint_name in JOINT_STABILITY['MODERATE']:
        return SAFETY_MARGINS['LIMB_JOINTS']

    # Check if it's a hand joint
    if joint_name in HAND_JOINTS:
        return SAFETY_MARGINS['HAND_JOINTS']

    # Check if it's a foot joint
    if joint_name in FOOT_JOINTS:
        return SAFETY_MARGINS['FOOT_JOINTS']

    # Default to hand joint margin for unknown joints
    return SAFETY_MARGINS['HAND_JOINTS']


def print_joint_info(joint_name: str):
    """
    Print detailed information about a joint.

    Args:
        joint_name: Name of the joint
    """
    if joint_name not in ALL_JOINTS:
        print(f"❌ Joint '{joint_name}' not found")
        return

    pos = ALL_JOINTS[joint_name]

    # Find stability level
    stability = "UNKNOWN"
    for level, joints in JOINT_STABILITY.items():
        if joint_name in joints:
            stability = level
            break

    # Find safety margin
    margin = get_safety_margin(joint_name)

    print(f"\n📍 Joint: {joint_name}")
    print(f"   Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) meters")
    print(f"   Stability: {stability}")
    print(f"   Safety Margin: {margin:.2f} meters")


def get_all_joint_positions(skeleton_id: int = 67, frame_number: int = 1) -> ConfigFrameData:
    """
    Get all joint positions in the same format as ZED joint receiver's get_latest_frame method.

    This method returns a ConfigFrameData object that is compatible with the ZED joint receiver
    format, containing all 38 joint positions as median values from the skeleton analysis.

    Args:
        skeleton_id: ID to assign to the skeleton (default: 67, from analysis)
        frame_number: Frame number to assign (default: 1)

    Returns:
        ConfigFrameData object with all joint positions in ZED-compatible format

    Example:
        # Get all joint positions
        frame_data = get_all_joint_positions()

        # Access skeleton
        skeleton = frame_data.skeletons[0]

        # Get specific joint position
        pelvis_pos = skeleton.get_joint_position('PELVIS')  # [0.981, 0.264, -0.034]

        # Convert to dict format
        frame_dict = frame_data.to_dict()
    """
    # Create all joint data objects (already in ConfigJointData format)
    joints = list(ALL_JOINTS.values())

    # Create skeleton data
    skeleton = ConfigSkeletonData(
        skeleton_id=skeleton_id,
        joints=joints
    )

    # Create frame data
    frame_data = ConfigFrameData(
        frame=frame_number,
        skeletons=[skeleton],
        timestamp=time.time()
    )

    return frame_data
