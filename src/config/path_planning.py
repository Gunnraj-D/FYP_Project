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

    # ========================================================================
    # IK SOLVER CONFIGURATION (NEW - for robust convergence)
    # ========================================================================
    # Number of yaw angles to sample (exploits redundancy in 7-DOF arm)
    # Higher = better IK success, but slower. Range: 8-16
    'ik_yaw_samples': 12,
    # Number of position perturbations to try per goal
    # Slightly perturbs target position to find reachable alternatives
    'ik_position_samples': 5,
    # XY perturbation range (meters) - typically ±20mm is safe
    'ik_xy_perturbation': 0.02,
    # Z perturbation range (meters) - typically ±30mm for vertical tolerance
    'ik_z_perturbation': 0.03,
    # Maximum IK iterations per attempt (higher = more robust, slower)
    'ik_max_iterations': 200,
    # IK convergence tolerance (meters) - 1mm is good balance
    'ik_tolerance': 1e-3,

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

    # Static data coordinate offset (applied to all static skeleton positions)
    'static_data_offset': {
        'x': 0.32,  # Subtract 23cm from X coordinate
        'y': -0.4,   # Subtract 40cm from Y coordinate
        'z': 0.0,    # No Z offset
    },

    'static_skeleton_data': {
        # Static skeleton data for testing/development
        # Format: {joint_name: [x, y, z]} in meters
        'RIGHT_WRIST': [0.139, 0.614, 0.433],
        'RIGHT_HANDTIP': [0.137, 0.508, 0.438],
        'LEFT_WRIST': [-0.109, 0.997, 0.051],
        'LEFT_HANDTIP': [-0.190, 0.931, 0.038],
        'RIGHT_SHOULDER': [0.401, 0.967, 0.435],
        'RIGHT_ELBOW': [0.180, 0.857, 0.440],
        'LEFT_SHOULDER': [0.378, 0.980, 0.093],
        'LEFT_ELBOW': [0.134, 1.007, 0.076],
        'NECK': [0.435, 0.927, 0.263],
        'NOSE': [0.565, 0.854, 0.230],
        'SPINE_2': [0.154, 0.961, 0.260],
        'PELVIS': [-0.034, 0.981, 0.264],
        'RIGHT_CLAVICLE': [0.392, 0.977, 0.331],
        'LEFT_CLAVICLE': [0.394, 0.983, 0.197],
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

    # 250mm = 100mm head + 150mm safety (increased for better head protection)
    'head_radius': 0.25,
    # 200mm = 150mm torso + 50mm safety (reduced from 250mm)
    'torso_radius': 0.20,
    # 120mm = 50mm arm + 70mm safety (reduced from 150mm)
    'arm_radius': 0.12,
    # 140mm = 70mm shoulder + 70mm safety (reduced from 170mm)
    'shoulder_radius': 0.14,
}
