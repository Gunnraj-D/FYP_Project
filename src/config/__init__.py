"""
Main configuration package for robot hand tracking system.

This package imports and re-exports all configuration from modular config files.
This allows for cleaner organization while maintaining backward compatibility.

REFACTORED STRUCTURE:
- paths.py          → File paths and directories
- opc_config.py     → OPC UA communication settings
- robot_config.py   → Robot kinematics, safety, collision avoidance
- camera_config.py  → Camera and hand-eye calibration
- grasp_config.py   → Grasp detection (including advanced anti-tip features)
- system_config.py  → Debug, logging, hand tracking

USAGE (New, cleaner syntax):
    from config import GRASP_DETECTION_CONFIG, DEBUG_MODE, ROBOT_ID

LEGACY USAGE (still works):
    from config import GRASP_DETECTION_CONFIG  # But no longer needed!

All imports work - zero breaking changes, but the new syntax is cleaner!
"""

# ============================================================================
# IMPORT ALL CONFIGURATION MODULES
# ============================================================================

# Paths and directories
from config.paths import (
    SRC_DIR,
    GGCNN2_MODEL_PATH,
    GRCONVNET_MODEL_PATH,
    HANDMODEL_FILEPATH,
    URDF_FILEPATH,
    HAND_EYE_MATRIX_FILE
)

# OPC UA configuration
from config.opc_config import (
    OPC_SERVER_URL,
    OPC_OBJECTS_NAME,
    OPC_UPDATE_INTERVAL_SECONDS,
    OPC_POLL_INTERVAL_MS,
    OPC_COMMAND_BATCH_SIZE,
    OPC_SKIP_REDUNDANT_WRITES,
    OPC_CONNECTION_TIMEOUT_SECONDS,
    OPC_RECONNECT_DELAY_SECONDS,
    OPC_MAX_RECONNECT_ATTEMPTS,
    OPC_MODE,
    OPC_MOCK_SERVER_URL
)

# Robot configuration
from config.robot_config import (
    ROBOT_ID,
    get_robot_name,
    get_robot_namespace,
    BASE_ELEMENT,
    ACTIVE_LINKS,
    JOINT_LIMITS,
    GRIPPER_CONFIG,
    PICKUP_LOCATION,
    PRE_PICKUP_POSE,
    HANDOFF_APPROACH_POSE,
    PLACE_APPROACH_DISTANCE,
    PLACE_RELEASE_DISTANCE,
    SAFETY_CONFIG,
    COLLISION_AVOIDANCE_CONFIG,
    REST_POSES,
    COLLISION_CHECK_LINKS,
    ERROR_RECOVERY_CONFIG,
    HUMAN_MODEL_CONFIG,
    TRACKED_HUMAN_JOINTS,
    ZED_MANUAL_OFFSET,
    IK_EPSILON_MARGIN_DEG
)

# Path planning configuration
from config.path_planning import (
    PATH_PLANNING_CONFIG
)

# Camera configuration
from config.camera_config import (
    CAMERA_TRANSLATION,
    CAMERA_ROTATION_EULER,
    CAMERA_TRANSFORM_MODE,
    HAND_EYE_MATRIX_CALIBRATED,
    HAND_EYE_MATRIX_SIMPLE,
    HAND_EYE_MATRIX,
    set_camera_transform_mode,
    get_camera_transform_info,
    print_camera_transform_info
)

# Grasp detection configuration (including advanced features)
from config.grasp_config import (
    SIMPLE_GRASP_CONFIG,    # Simplified user-facing config
    GRASP_MODEL_TYPE,
    GRCONVNET_CONFIG,
    GRASP_DETECTION_CONFIG,
    GRASP_EXECUTION_CONFIG,
    OBJECT_PROFILES
)

# System configuration
from config.system_config import (
    LOOP_RATE_MS,
    DEBUG_MODE,
    DEBUG_CONFIG,
    LOGGING_CONFIG,
    DISTANCE_TO_REMAIN_M,
    HAND_STABILITY_THRESHOLD,
    HAND_STABILITY_TIME_THRESHOLD,
    HAND_OCCLUSION_CONFIG,
    ENHANCED_OCCLUSION_CONFIG,
    HAND_OCCLUSION_THRESHOLD,
    HAND_OCCLUSION_FRAMES_THRESHOLD,
    CONTROL_ESTIMATED_LATENCY_S,
    MAX_POSITION_CHANGE_PER_CYCLE,
    MAX_AXIAL_CHANGE_PER_CYCLE,
    STAGED_FULL_SPEED_DISTANCE,
    STAGED_FINE_DISTANCE,
    VELOCITY_DAMPING_K,
    DEAD_ZONE_ENTRY_M,
    DEAD_ZONE_EXIT_M,
    STABLE_VELOCITY_THRESHOLD,
    MIN_VELOCITY_DT
)

# Path planning configuration
from config.path_planning import (
    PATH_PLANNING_CONFIG
)

# ============================================================================
# EXPORT ALL (for "from config import *")
# ============================================================================

__all__ = [
    # Paths
    'SRC_DIR',
    'GGCNN2_MODEL_PATH',
    'GRCONVNET_MODEL_PATH',
    'HANDMODEL_FILEPATH',
    'URDF_FILEPATH',
    'HAND_EYE_MATRIX_FILE',

    # OPC UA
    'OPC_SERVER_URL',
    'OPC_OBJECTS_NAME',
    'OPC_UPDATE_INTERVAL_SECONDS',
    'OPC_POLL_INTERVAL_MS',
    'OPC_COMMAND_BATCH_SIZE',
    'OPC_SKIP_REDUNDANT_WRITES',
    'OPC_CONNECTION_TIMEOUT_SECONDS',
    'OPC_RECONNECT_DELAY_SECONDS',
    'OPC_MAX_RECONNECT_ATTEMPTS',
    'OPC_MODE',
    'OPC_MOCK_SERVER_URL',

    # Robot
    'ROBOT_ID',
    'get_robot_name',
    'get_robot_namespace',
    'BASE_ELEMENT',
    'ACTIVE_LINKS',
    'JOINT_LIMITS',
    'GRIPPER_CONFIG',
    'PICKUP_LOCATION',
    'PRE_PICKUP_POSE',
    'HANDOFF_APPROACH_POSE',
    'PLACE_APPROACH_DISTANCE',
    'PLACE_RELEASE_DISTANCE',
    'SAFETY_CONFIG',
    'COLLISION_AVOIDANCE_CONFIG',
    'REST_POSES',
    'COLLISION_CHECK_LINKS',
    'ERROR_RECOVERY_CONFIG',
    'PATH_PLANNING_CONFIG',
    'HUMAN_MODEL_CONFIG',
    'TRACKED_HUMAN_JOINTS',
    'ZED_MANUAL_OFFSET',

    # Camera
    'CAMERA_TRANSLATION',
    'CAMERA_ROTATION_EULER',
    'CAMERA_TRANSFORM_MODE',
    'HAND_EYE_MATRIX_CALIBRATED',
    'HAND_EYE_MATRIX_SIMPLE',
    'HAND_EYE_MATRIX',
    'set_camera_transform_mode',
    'get_camera_transform_info',
    'print_camera_transform_info',
    # Grasp detection
    'SIMPLE_GRASP_CONFIG',      # Simplified user-facing config
    'GRASP_MODEL_TYPE',
    'GRCONVNET_CONFIG',
    'GRASP_DETECTION_CONFIG',
    'GRASP_EXECUTION_CONFIG',
    'OBJECT_PROFILES',

    # System
    'LOOP_RATE_MS',
    'DEBUG_MODE',
    'DEBUG_CONFIG',
    'LOGGING_CONFIG',
    'DISTANCE_TO_REMAIN_M',
    'HAND_STABILITY_THRESHOLD',
    'HAND_STABILITY_TIME_THRESHOLD',
    'HAND_OCCLUSION_THRESHOLD',
    'HAND_OCCLUSION_FRAMES_THRESHOLD',
    'CONTROL_ESTIMATED_LATENCY_S',
    'MAX_POSITION_CHANGE_PER_CYCLE',
    'MAX_AXIAL_CHANGE_PER_CYCLE',
    'STAGED_FULL_SPEED_DISTANCE',
    'STAGED_FINE_DISTANCE',
    'VELOCITY_DAMPING_K',
    'DEAD_ZONE_ENTRY_M',
    'DEAD_ZONE_EXIT_M',
    'STABLE_VELOCITY_THRESHOLD',
    'MIN_VELOCITY_DT',

    # Path planning
    'PATH_PLANNING_CONFIG',
]

# ============================================================================
# CONVENIENCE FUNCTION
# ============================================================================


def print_config_summary():
    """Print a summary of the current configuration."""
    print("\n" + "="*70)
    print("⚙️  ROBOT HAND TRACKING SYSTEM CONFIGURATION")
    print("="*70)
    print(f"🤖 Robot ID: {ROBOT_ID}")
    print(f"📡 OPC Mode: {OPC_MODE}")
    print(f"🎯 Grasp Model: {GRASP_MODEL_TYPE.upper()}")
    print(f"📷 Camera Mode: {CAMERA_TRANSFORM_MODE}")
    print(f"🐛 Debug Mode: {DEBUG_MODE}")
    print(
        f"✨ Advanced Postprocessing: {GRASP_DETECTION_CONFIG['use_advanced_postprocessing']}")
    if GRASP_DETECTION_CONFIG['use_advanced_postprocessing']:
        print(
            f"   - Width multiplier: {GRASP_DETECTION_CONFIG['width_multiplier']}")
        print(f"   - Min overlap: {GRASP_DETECTION_CONFIG['min_overlap']}")
        print(
            f"   - PCA angle correction: {GRASP_DETECTION_CONFIG['use_pca_angle_correction']}")
        print(
            f"   - NMS dilate size: {GRASP_DETECTION_CONFIG['nms_dilate_size']}")
    print("="*70 + "\n")
