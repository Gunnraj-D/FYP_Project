"""
System-level configuration: debug mode, logging, hand tracking.
"""
import numpy as np

# ============================================================================
# SYSTEM
# ============================================================================

# Main control loop rate (milliseconds)
LOOP_RATE_MS = 20  # 50Hz control loop

# ============================================================================
# DEBUG CONFIGURATION
# ============================================================================

# Debug mode for grasp detection and frame selection
DEBUG_MODE = True

# Debug mode settings
DEBUG_CONFIG = {
    'show_live_feed': True,           # Show live camera feed in debug mode
    'frame_selection_enabled': True,  # Allow spacebar to select frames for processing
    'window_title': 'Debug Feed - Press SPACEBAR to process frame',
    'display_quality_threshold': 0.1,  # Minimum quality to show in debug display
}

# ============================================================================
# LOGGING CONFIGURATION
# ============================================================================

LOGGING_CONFIG = {
    'level': 'INFO',
    'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    'file': 'robot_hand_tracking.log'
}

# ============================================================================
# HAND TRACKING CONFIGURATION
# ============================================================================

# Hand stability parameters
DISTANCE_TO_REMAIN_M = 0.40  # Distance to maintain from hand (meters)
HAND_STABILITY_THRESHOLD = 0.02  # How stable the hand should be (meters)
# How long hand needs to remain stable (seconds)
HAND_STABILITY_TIME_THRESHOLD = 2.0
