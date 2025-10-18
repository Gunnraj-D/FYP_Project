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

# Hand occlusion detection parameters (MediaPipe-based)
HAND_OCCLUSION_CONFIG = {
    # MediaPipe confidence threshold (below this = occluded)
    'min_confidence': 0.3,

    # Minimum number of visible landmarks (out of 21 total MediaPipe landmarks)
    'min_visible_landmarks': 15,

    # Grace period: wait this long for hand to become unoccluded (seconds)
    # If hand becomes visible within this time, continue normally
    'grace_period_s': 2.0,

    # Number of consecutive occluded frames to confirm occlusion
    # Prevents false positives from single bad frames
    'consecutive_frames_threshold': 3,

    # Timeout: if no hand appears within this time, fail (seconds)
    # This handles the case where operator never presents their hand
    'no_hand_timeout_s': 2.0,
}

# Enhanced occlusion detection parameters (multi-factor approach)
ENHANCED_OCCLUSION_CONFIG = {
    # Score thresholds
    # Trigger fallback above this (0.0-1.0)
    'occlusion_threshold': 0.60,
    'frames_threshold': 20,               # Consecutive frames (2.0s at 10Hz)

    # Hand detection presence (Weight: 0.5 max)
    'max_lost_frames': 15,                # ~0.5 seconds at 30fps

    # Landmark visibility (Weight: 0.25)
    'min_landmark_presence': 0.5,         # Minimum presence score

    # Key landmark tracking (Weight: 0.15)
    'key_landmark_indices': [0, 4, 8, 12, 16, 20],  # Critical landmarks

    # Depth quality (Weight: 0.3 total)
    'min_depth_valid_ratio': 0.3,         # Valid pixel ratio
    'max_depth_std_dev': 0.03,            # Depth standard deviation
    'depth_std_normalization': (0.02, 0.05),  # (min, max) for normalization

    # Temporal smoothing
    'history_size': 30,                   # Sliding window size
    'use_temporal_smoothing': True,       # Enable smoothing

    # Adaptive Z-filtering integration
    'use_adaptive_z_filter': True,        # Use existing Z-filter state
    'z_filter_alpha_good': 0.5,          # Responsive when quality good
    'z_filter_alpha_poor': 0.15,         # Smooth when quality poor
}

# Legacy parameters (kept for backward compatibility, consider deprecated)
HAND_OCCLUSION_THRESHOLD = 0.5  # Threshold for detecting hand occlusion
# Number of consecutive frames for occlusion detection
HAND_OCCLUSION_FRAMES_THRESHOLD = 3

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
# meters (max 1cm commanded change per 10Hz cycle) - reduced to minimize jitter
MAX_POSITION_CHANGE_PER_CYCLE = 0.010
# meters (optional extra clamp per axis)
MAX_AXIAL_CHANGE_PER_CYCLE = 0.008

# Staged approach distances
# meters; >0.15 -> full scale (start slowing earlier)
STAGED_FULL_SPEED_DISTANCE = 0.15
# meters; <0.05 -> fine approach (low scale) - increased for smoother transition
STAGED_FINE_DISTANCE = 0.05

# Velocity damping (simple damping to reduce command at high robot velocities)
# unitless damping coefficient; higher -> more damping
VELOCITY_DAMPING_K = 4.0  # Increased to 4.0 for smoother motion and less jitter

# Dead zone hysteresis & stability thresholds (slightly increased for more robustness)
# entry threshold (increase to 60mm) - more tolerant to noise
DEAD_ZONE_ENTRY_M = 0.06
# exit threshold (increase to 40mm) - maintain hysteresis
DEAD_ZONE_EXIT_M = 0.04
# m/s, robot must be slower than this to be "stable"
# Increased - allow slightly faster motion to be considered stable
STABLE_VELOCITY_THRESHOLD = 0.025

# Safety: minimum dt to compute velocity
MIN_VELOCITY_DT = 1e-3                    # seconds
