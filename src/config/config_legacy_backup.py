"""
Main configuration file for robot hand tracking system.

This file imports and re-exports all configuration from modular config files.
This allows for cleaner organization while maintaining backward compatibility.

REFACTORED STRUCTURE:
- paths.py          → File paths and directories
- opc_config.py     → OPC UA communication settings
- robot_config.py   → Robot kinematics, safety, collision avoidance
- camera_config.py  → Camera and hand-eye calibration
- grasp_config.py   → Grasp detection (including advanced anti-tip features)
- system_config.py  → Debug, logging, hand tracking

USAGE (unchanged):
    from config.config import GRASP_DETECTION_CONFIG, DEBUG_MODE, ...

All imports work exactly as before - zero breaking changes!
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
    HAND_EYE_MATRIX_FILE,
    RESOURCES_DIR,
    ML_MODELS_DIR,
    ROBOT_MODELS_DIR
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
    PATH_PLANNING_CONFIG,
    HUMAN_MODEL_CONFIG,
    TRACKED_HUMAN_JOINTS
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
    GRASP_MODEL_TYPE,
    GRCONVNET_CONFIG,
    GRASP_DETECTION_CONFIG,
    GRASP_EXECUTION_CONFIG,
    TABLE_REF_UPDATE_ALPHA,
    TABLE_REF_TOLERANCE,
    TABLE_REF_INITIALIZATION_FRAMES,
    TABLE_REF_MIN_DEPTH,
    TABLE_REF_MAX_DEPTH,
    OBJECT_PROFILES
)

# System configuration
from config.system_config import (
    LOOP_RATE_MS,
    DEBUG_MODE,
    DEBUG_CONFIG,
    LOGGING_CONFIG,
    HAND_LOCATION_TEMP,
    HAND_TRACKING_CONFIG,
    DISTANCE_TO_REMAIN_M,
    HAND_STABILITY_THRESHOLD,
    HAND_STABILITY_TIME_THRESHOLD
)

# ============================================================================
# EXPORT ALL (for "from config.config import *")
# ============================================================================

__all__ = [
    # Paths
    'SRC_DIR',
    'GGCNN2_MODEL_PATH',
    'GRCONVNET_MODEL_PATH',
    'HANDMODEL_FILEPATH',
    'URDF_FILEPATH',
    'HAND_EYE_MATRIX_FILE',
    'RESOURCES_DIR',
    'ML_MODELS_DIR',
    'ROBOT_MODELS_DIR',
    
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
    'GRASP_MODEL_TYPE',
    'GRCONVNET_CONFIG',
    'GRASP_DETECTION_CONFIG',
    'GRASP_EXECUTION_CONFIG',
    'TABLE_REF_UPDATE_ALPHA',
    'TABLE_REF_TOLERANCE',
    'TABLE_REF_INITIALIZATION_FRAMES',
    'TABLE_REF_MIN_DEPTH',
    'TABLE_REF_MAX_DEPTH',
    'OBJECT_PROFILES',
    
    # System
    'LOOP_RATE_MS',
    'DEBUG_MODE',
    'DEBUG_CONFIG',
    'LOGGING_CONFIG',
    'HAND_LOCATION_TEMP',
    'HAND_TRACKING_CONFIG',
    'DISTANCE_TO_REMAIN_M',
    'HAND_STABILITY_THRESHOLD',
    'HAND_STABILITY_TIME_THRESHOLD',
]

# ============================================================================
# CONVENIENCE INFO FUNCTION
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
    print(f"✨ Advanced Postprocessing: {GRASP_DETECTION_CONFIG['use_advanced_postprocessing']}")
    if GRASP_DETECTION_CONFIG['use_advanced_postprocessing']:
        print(f"   - Width multiplier: {GRASP_DETECTION_CONFIG['width_multiplier']}")
        print(f"   - Min overlap: {GRASP_DETECTION_CONFIG['min_overlap']}")
        print(f"   - PCA angle correction: {GRASP_DETECTION_CONFIG['use_pca_angle_correction']}")
        print(f"   - NMS dilate size: {GRASP_DETECTION_CONFIG['nms_dilate_size']}")
    print("="*70 + "\n")


# ============================================================================
# NOTE: CONFIGURATION ORGANIZATION
# ============================================================================
# 
# This config is now modular! Each section lives in its own file:
#
#   📁 config/
#   ├── paths.py          - File paths and directories
#   ├── opc_config.py     - OPC UA communication
#   ├── robot_config.py   - Robot, kinematics, safety, planning
#   ├── camera_config.py  - Camera and hand-eye calibration
#   ├── grasp_config.py   - Grasp detection (ADVANCED anti-tip features)
#   ├── system_config.py  - Debug, logging, hand tracking
#   └── config.py         - This file (imports and re-exports all)
#
# To modify settings:
# 1. Find the relevant config file above
# 2. Edit the specific parameter
# 3. Import still works: from config.config import GRASP_DETECTION_CONFIG
#
# Benefits:
# - ✅ Cleaner organization (logical grouping)
# - ✅ Easier to find settings (smaller files)
# - ✅ Better maintainability (separated concerns)
# - ✅ 100% backward compatible (same imports work)
# 
# ============================================================================
