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
Z_FILTER_ALPHA_GOOD = 0.5  # Reduced for smoother Z tracking
# EMA alpha when depth quality is poor (lower -> stronger smoothing)
Z_FILTER_ALPHA_POOR = 0.15  # Slightly increased to be less sluggish when quality poor
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

# Latency and predictive control
# Estimated end-to-end latency (s). Tune between 0.18-0.35
CONTROL_ESTIMATED_LATENCY_S = 0.30  # Increased to reduce overshoot

# Per-cycle safety clamps
# meters (max 3cm commanded change per 10Hz cycle) - reduced for smoother motion
MAX_POSITION_CHANGE_PER_CYCLE = 0.03
# meters (optional extra clamp per axis)
MAX_AXIAL_CHANGE_PER_CYCLE = 0.025

# Staged approach distances
# meters; >0.15 -> full scale (start slowing earlier)
STAGED_FULL_SPEED_DISTANCE = 0.15
# meters; <0.05 -> fine approach (low scale) - increased for smoother transition
STAGED_FINE_DISTANCE = 0.05

# Velocity damping (simple damping to reduce command at high robot velocities)
# unitless damping coefficient; higher -> more damping
VELOCITY_DAMPING_K = 2.0  # Increased for more aggressive damping

# Dead zone hysteresis & stability thresholds
# entry threshold (30mm) - precise for hand placement
DEAD_ZONE_ENTRY_M = 0.030
# exit threshold (20mm) - tight hold for accuracy
DEAD_ZONE_EXIT_M = 0.020
# m/s, robot must be slower than this to be "stable"
# Increased - allow slightly faster motion to be considered stable
STABLE_VELOCITY_THRESHOLD = 0.015

# Safety: minimum dt to compute velocity
MIN_VELOCITY_DT = 1e-3                    # seconds
