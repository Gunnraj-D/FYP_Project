"""
Grasp detection configuration - ADVANCED anti-tip system.

Includes all settings for:
- GR-ConvNet / GGCNN2 models
- Advanced postprocessing (anti-tip fixes)
- Temporal filtering
- Multi-factor scoring
- PCA angle correction
- Gripper specifications
"""

# ============================================================================
# MODEL SELECTION
# ============================================================================

# Model selection: 'ggcnn2', 'grconvnet', or 'both'
GRASP_MODEL_TYPE = 'grconvnet'

# ============================================================================
# GR-CONVNET MODEL CONFIGURATION
# ============================================================================

# GR-ConvNet specific configuration (trained on Jacquard Dataset, RGB-D, 300x300)
GRCONVNET_CONFIG = {
    'input_size': 300,              # Input resolution
    'input_channels': 4,            # RGB-D input [D, R, G, B]
    'use_dropout': False,           # Disable for inference
    'dropout_prob': 0.0,
    'use_depth_inpainting': False,  # Enable if depth sparse (test first)
    'channel_size': 32,             # Base filter size (as trained)
}

# ============================================================================
# GRASP DETECTION - CORE PARAMETERS
# ============================================================================

GRASP_DETECTION_CONFIG = {
    # Quality thresholds
    'min_quality_threshold': 0.10,      # Minimum grasp quality to accept
    'max_grasp_width': 0.100,           # Maximum grasp width in meters (100mm)
    'min_grasp_width': 0.020,           # Minimum grasp width in meters (20mm)

    # Approach configuration
    'approach_height_offset': 0.050,    # Height offset for approach (meters)
    # Depth offset for grasp (meters) - raised by 15mm to compensate
    'grasp_depth_offset': 0.025,
    'vertical_approach': True,          # Use vertical approach angle
    'approach_angle': -90.0,            # Approach angle in degrees
    'frame_processing_interval': 0.5,   # Process frames every N seconds

    # Angle offset configuration
    # Set to 0.0 - we add the 90° rotation manually in the execution pipeline
    'grasp_angle_offset_rad': 0.0,      # No offset (rotation added manually)
    'compose_order': 'down_then_z',     # Rotation composition order

    # Depth sampling
    'depth_sample_radius': 5,           # Radius (pixels) for depth queries
    # In-plane angle filtering (None = disabled)
    'topdown_angle_tolerance_rad': None,
    # Reference angle (if filtering enabled)
    'topdown_ref_angle': 0.0,

    # ========================================================================
    # TEMPORAL FILTERING
    # ========================================================================
    'temporal_filter_enabled': True,          # Enable temporal filtering
    'temporal_window_size': 5,                # Number of frames to average
    'temporal_filter_type': 'circular_mean',  # 'circular_mean', 'median', or 'ema'
    'temporal_ema_alpha': 0.3,                # EMA smoothing factor
    'temporal_outlier_threshold_deg': 30,     # Outlier rejection threshold

    # ========================================================================
    # ADVANCED ANTI-TIP POSTPROCESSING 🎯
    # ========================================================================
    # Production-ready grasp selection with all fixes from research phase
    # Based on visualize_grconvnet_temporal.py

    # Enable all anti-tip fixes (HIGHLY RECOMMENDED)
    'use_advanced_postprocessing': True,

    # WIDTH CALIBRATION ⚖️
    # Calibrated for: RealSense D435 + GR-ConvNet (Jacquard) + 30mm screwdriver
    # Recalibrate for new objects: new_mult = 95.0 × (actual_mm / predicted_mm)
    'width_multiplier': 95.0,

    # OBJECT MASK OVERLAP 🎯
    # Minimum overlap between grasp rectangle and detected object
    # Prevents tip grasps by requiring contact with object body
    # Guidelines: Flat (0.3-0.4), Cylindrical (0.25-0.3), Small (0.2-0.25)
    'min_overlap': 0.25,  # Calibrated for cylindrical objects (screwdrivers)

    # FOREGROUND MASK SEGMENTATION 🔍
    # Parameters for depth-based object/background segmentation
    # Background depth percentile (higher = stricter)
    'bg_percentile': 80,
    'depth_diff_thresh': 0.02,   # Foreground depth threshold in meters (2cm)

    # NON-MAXIMUM SUPPRESSION (NMS) 🎲
    # Prevents selecting sharp quality spikes at tips
    # Kernel size: Larger = more aggressive, Smaller = more permissive
    # Screwdrivers: 9-11, Small objects: 7, Large objects: 11-13
    'nms_dilate_size': 9,
    'nms_min_threshold': 0.03,   # Minimum quality for local maxima

    # PCA-BASED ANGLE CORRECTION 📐
    # Automatically fixes 90° convention mismatches on axis-aligned objects
    # Works on horizontal, vertical, and diagonal orientations
    'use_pca_angle_correction': True,

    # BORDER PENALTY 🚫
    # Rejects grasps too close to image edges
    # Range: [0,1], where 0.2 = within 20% of edge
    'border_threshold': 0.20,

    # MULTI-FACTOR SCORING WEIGHTS ⚖️
    # Formula: score = (Q^w_q) × (O^w_o) × (B^w_b) × (W^w_w) × (T^w_t)
    # Higher weight = more important factor
    'scoring_weights': {
        'q': 1.0,   # Quality (baseline importance)
        'o': 1.2,   # Overlap (emphasized for anti-tip)
        'b': 0.5,   # Border (less critical if other factors good)
        'w': 0.7,   # Width preference (optimal range)
        't': 0.8    # Temporal consistency (reduces jitter)
    },
}

# ============================================================================
# GRASP EXECUTION
# ============================================================================

GRASP_EXECUTION_CONFIG = {
    # Timing
    'pre_grasp_delay': 1.0,              # Delay before grasping (seconds)
    'grasp_duration': 2.0,               # Time to hold grasp (seconds)
    'post_grasp_delay': 1.0,             # Delay after grasping (seconds)
    'lift_height': 0.100,                # Height to lift after grasp (meters)

    # Retry behavior
    'retry_attempts': 3,                 # Number of retry attempts
    'retry_delay': 2.0,                  # Delay between retries (seconds)
    # Timeout for grasp generation (seconds)
    'grasp_generation_timeout': 10.0,

    # Robotiq 2F-85 Gripper Specifications 🤖
    'gripper_min_width_m': 0.005,        # Minimum gripper width (5mm)
    # Maximum gripper width (80mm, 5mm safety margin)
    'gripper_max_width_m': 0.080,
    'gripper_optimal_min_mm': 15.0,      # Optimal range minimum
    'gripper_optimal_max_mm': 60.0,      # Optimal range maximum
}


# ============================================================================
# OBJECT-SPECIFIC CONFIGURATION PROFILES (Optional)
# ============================================================================
# Save successful configurations for different object types

OBJECT_PROFILES = {
    # Current default: Screwdrivers & long cylindrical tools
    'screwdriver_30mm': {
        'width_multiplier': 95.0,
        'min_overlap': 0.25,
        'nms_dilate_size': 9,
        'bg_percentile': 80,
    },

    # Small objects (USB drives, batteries)
    'small_objects': {
        'width_multiplier': 95.0,  # May need recalibration
        'min_overlap': 0.20,
        'nms_dilate_size': 7,
        'bg_percentile': 75,
    },

    # Large flat objects (books, boxes)
    'large_flat': {
        'width_multiplier': 95.0,  # May need recalibration
        'min_overlap': 0.30,
        'nms_dilate_size': 11,
        'bg_percentile': 82,
    },
}
