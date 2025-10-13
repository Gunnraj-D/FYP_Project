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
DISTANCE_TO_REMAIN_M = 0.2  # Distance to maintain from hand (meters)
HAND_STABILITY_THRESHOLD = 0.02  # How stable the hand should be (meters)
# How long hand needs to remain stable (seconds)
HAND_STABILITY_TIME_THRESHOLD = 2.0

# Z-coordinate filtering parameters (added for adaptive Z filtering)
# EMA alpha when depth quality is good (higher -> faster)
Z_FILTER_ALPHA_GOOD = 0.6
# EMA alpha when depth quality is poor (lower -> stronger smoothing)
Z_FILTER_ALPHA_POOR = 0.12
# Minimum fraction of valid pixels in ROI to consider depth "good"
MIN_DEPTH_VALID_RATIO = 0.30
# Depth std-dev in meters above which quality is poor (e.g., 30mm)
MAX_DEPTH_STD_DEV = 0.03
# Max absolute Z change allowed per detection frame (meters; 2cm)
MAX_Z_CHANGE_PER_FRAME = 0.02
# Hysteresis to avoid flipping quality on small changes
Z_FILTER_HYSTERESIS_FACTOR = 0.85
# Sec: allow initialization time where we accept measurement directly
Z_FILTER_INIT_TIMEOUT = 0.5
# Enable/disable debug logging for depth quality (for debug)
LOG_DEPTH_QUALITY = True
