"""
Path Planning Configuration

Configuration for human-aware path planning including data source options
and static skeleton data for testing/development.
"""

# ============================================================================
# PATH PLANNING CONFIGURATION
# ============================================================================

PATH_PLANNING_CONFIG = {
    # Planner algorithm
    'planner_type': 'rrt_connect',     # 'rrt_connect', 'birrt', 'prm'
    # Number of joints to plan for (7-DOF arm)
    'planning_dof': 7,
    # Use GUI mode for debugging (False = DIRECT mode)
    'planning_gui': False,
    'check_self_collision': False,     # Enable self-collision checking
    'step_size': 0.15,                 # Joint space step size (radians)
    'goal_bias': 0.3,                  # Probability of sampling goal
    'max_iterations': 2000,            # Maximum planning iterations
    'planning_timeout': 0.5,           # Initial planning timeout (seconds)
    'replan_timeout': 0.1,             # Replanning timeout (seconds)
    # Post-processing smoothing passes (reduced from 20 for safety)
    'smoothing_iterations': 10,

    # Rolling horizon
    'horizon_time': 0.5,               # Plan ahead time (seconds)
    # Minimum time between replans (seconds) - INCREASED to reduce jitter
    'min_replan_interval': 1.0,
    # Replan if human moves >15cm - INCREASED to reduce jitter
    'replan_threshold_position': 0.15,
    'replan_threshold_velocity': 0.30,  # Replan if human speed >0.3m/s

    # SSM (Speed and Separation Monitoring) zones per ISO/TS 15066
    # INCREASED for real-world safety - robot maintains larger distance
    'comfort_distance': 0.60,          # ≥0.6m: normal speed (100%) - was 0.5m
    # ≥0.4m: reduced speed (50-100%) - was 0.3m
    'warning_distance': 0.40,
    # ≥0.2m: critical/stop (0-50%) - was 0.12m
    'hard_min_distance': 0.20,
    'emergency_stop_distance': 0.15,   # <0.15m: immediate stop - was 0.08m

    # Speed scaling
    'speed_scale_comfort': 1.0,        # 100% speed in comfort zone
    'speed_scale_warning': 0.5,        # 50% speed in warning zone
    'speed_scale_critical': 0.1,       # 10% speed near hard minimum

    # Safety behavior
    # Max time to wait when stopped (seconds)
    'max_safety_wait_time': 10.0,
    'collision_check_distance': 1.0,   # Distance to check collisions (meters)

    # ISO/TS 15066 SSM parameters
    'reaction_time': 0.2,              # System reaction time T_r (seconds)
    'stop_time_max': 1.0,              # Maximum stop time T_s (seconds)
    'intrusion_distance': 0.05,        # Intrusion tolerance C (meters)
    'position_uncertainty': 0.03,      # ZED position uncertainty Z_d (meters)
    # Robot position uncertainty Z_r (meters)
    'robot_uncertainty': 0.02,

    # Data source configuration
    'use_static_skeleton_data': True,  # Use static data instead of ZED receiver
    'static_skeleton_data': {
        # Static skeleton data for testing/development
        # Format: {joint_name: [x, y, z]} in meters
        'RIGHT_WRIST': [0.5, 0.3, 0.8],
        'RIGHT_HANDTIP': [0.55, 0.32, 0.75],
        'LEFT_WRIST': [0.4, 0.3, 0.8],
        'LEFT_HANDTIP': [0.45, 0.32, 0.75],
        'RIGHT_SHOULDER': [0.3, 0.2, 1.2],
        'RIGHT_ELBOW': [0.4, 0.25, 1.0],
        'LEFT_SHOULDER': [0.2, 0.2, 1.2],
        'LEFT_ELBOW': [0.3, 0.25, 1.0],
        'NECK': [0.25, 0.15, 1.4],
        'NOSE': [0.25, 0.15, 1.5],
        'SPINE_2': [0.25, 0.1, 1.1],
        'PELVIS': [0.25, 0.05, 0.9],
        'RIGHT_CLAVICLE': [0.28, 0.18, 1.3],
        'LEFT_CLAVICLE': [0.22, 0.18, 1.3],
    }
}

# ============================================================================
# HUMAN BODY MODEL CONFIGURATION
# ============================================================================

HUMAN_MODEL_CONFIG = {
    # Primitive radii following ISO/TS 15066 Speed and Separation Monitoring (SSM)
    # Guidelines: Minimum protective separation = 150mm for collaborative operations
    # Formula: radius = actual_body_part_size + safety_margin
    #
    # Actual human dimensions (95th percentile male):
    #   - Head: ~100mm radius
    #   - Torso: ~150mm radius
    #   - Upper arm: ~50mm radius
    #   - Shoulder: ~70mm radius
    #
    # Safety margins applied:
    #   - Static parts (head/torso): +100mm (ISO minimum)
    #   - Dynamic parts (arms): +100mm (ISO minimum, arms move fastest)
    #   - Shoulders: +100mm (high-risk articulation point)

    # 200mm = 100mm head + 100mm safety (ISO compliant)
    'head_radius': 0.20,
    # 250mm = 150mm torso + 100mm safety (conservative)
    'torso_radius': 0.25,
    # 150mm = 50mm arm + 100mm safety (ISO minimum)
    'arm_radius': 0.15,
    # 170mm = 70mm shoulder + 100mm safety (conservative)
    'shoulder_radius': 0.17,
}
