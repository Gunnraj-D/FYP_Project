"""
Grasp detection configuration.

Includes all settings for:
- GR-ConvNet / GGCNN2 models
- Advanced postprocessing
- Temporal filtering
- Multi-factor scoring
- PCA angle correction
- Gripper specifications
"""

# ============================================================================
# SIMPLIFIED GRASP CONFIGURATION (User-Facing)
# ============================================================================
# For most users, these are the ONLY parameters you need to tune.
# Advanced parameters are set to sensible defaults below.

SIMPLE_GRASP_CONFIG = {
    # Quality control
    'quality_threshold': 0.10,           # Minimum grasp quality [0-1]

    # Width calibration (object-specific!)
    # IMPORTANT: Calibrate for your objects (80-110 typical)
    # Formula: new = old * (actual_width_mm / logged_width_mm)
    # Example: 30mm screwdriver shows 25mm → new = 95 * (30/25) = 114
    'width_multiplier': 95.0,            # Calibrate using real objects
    'width_range_mm': (20, 100),         # Valid width range in mm
    # Optimal gripper range (Robotiq 2F-85)
    'gripper_optimal_mm': (15, 60),

    # Quality filters
    # Higher overlap = fewer bad grasps but harder to find grasps
    # Lower overlap = more grasps found but some may be unstable
    # Guidelines: Flat objects (0.3-0.4), Cylindrical (0.25-0.3), Small (0.2-0.25)
    'min_object_overlap': 0.25,          # Overlap with object mask [0-1]
    'avoid_borders': True,               # Penalize grasps near image borders

    # Temporal filtering (reduce jitter)
    'temporal_smoothing': True,          # Enable angle smoothing
    'temporal_window': 5,                # Number of frames to average (3-7)

    # Angle correction
    # Fixes 90° rotation errors on cylindrical objects
    'pca_angle_correction': True,        # Enable PCA-based correction
}

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

    # Table height and calibration correction
    # Table Z coordinate in base frame (meters)
    'table_height_base_frame': 0.001,
    # Grasp height correction offset (meters) - applied to final grasp Z
    # Positive value moves gripper UP, negative moves DOWN
    'grasp_height_offset': 0.105,  # 85mm correction for calibration offset
    'vertical_approach': True,          # Use vertical approach angle
    'approach_angle': -90.0,            # Approach angle in degrees
    'frame_processing_interval': 0.5,   # Process frames every N seconds

    # Angle offset configuration
    # Set to 0.0 - we add the 90° rotation manually in the execution pipeline
    'grasp_angle_offset_rad': 0.0,      # No offset (rotation added manually)
    'compose_order': 'down_then_z',     # Rotation composition order

    # Depth sampling
    'depth_sample_radius': 7,           # Radius (pixels) for depth queries
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
    # Optional heuristics (default disabled for stability)
    'use_plane_suppression': False,
    'use_quality_union': False,
    'use_interior_center_check': False,

    # WIDTH CALIBRATION ⚖️
    # Calibrated for: RealSense D435 + GR-ConvNet (Jacquard) + 30mm screwdriver
    # Recalibrate for new objects: new_mult = 95.0 × (actual_mm / predicted_mm)
    'width_multiplier': 95.0,

    # OBJECT MASK OVERLAP 🎯
    # Minimum overlap between grasp rectangle and detected object
    # Prevents tip grasps by requiring contact with object body
    # Guidelines: Flat (0.3-0.4), Cylindrical (0.25-0.3), Small (0.2-0.25)
    'min_overlap': 0.55,

    # FOREGROUND MASK SEGMENTATION 🔍
    # Parameters for depth-based object/background segmentation
    # Background depth percentile (higher = stricter)
    'bg_percentile': 72,
    'depth_diff_thresh': 0.012,   # Foreground depth threshold in meters (12mm)

    # NON-MAXIMUM SUPPRESSION (NMS) 🎲
    # Prevents selecting sharp quality spikes at tips
    # Kernel size: Larger = more aggressive, Smaller = more permissive
    # Screwdrivers: 9-11, Small objects: 7, Large objects: 11-13
    'nms_dilate_size': 5,
    'nms_min_threshold': 0.01,   # Minimum quality for local maxima

    # PCA-BASED ANGLE CORRECTION 📐
    # Automatically fixes 90° convention mismatches on axis-aligned objects
    # Works on horizontal, vertical, and diagonal orientations
    'use_pca_angle_correction': True,
    # PCA alignment mode: 'align' (match PCA axis) or 'perpendicular' (gripper closes across PCA axis)
    'pca_align_mode': 'align',

    # BORDER PENALTY 🚫
    # Rejects grasps too close to image edges
    # Range: [0,1], where 0.2 = within 20% of edge
    'border_threshold': 0.15,
    'boost_masked_quality': True,
    'denoise_quality': True,
    'use_geometric_fallback': True,
    # Depth sampling method for ROI: 'median' (default), 'min', or 'p_low' (percentile)
    'depth_sample_method': 'p_low',
    'depth_sample_percentile': 15.0,
    # Minimum pixel distance from object edge for interior center check
    'edge_margin_px': 6,

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

    # Retry behavior (if all frames in collection fail)
    'retry_attempts': 3,                 # Number of full collection retries
    # Delay between retry attempts (seconds)
    'retry_delay': 2.0,
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
        'bg_percentile': 78,
        'depth_diff_thresh': 0.015,
        'pca_align_mode': 'perpendicular',
        'scoring_weights': {
            'q': 1.0,
            'o': 1.1,
            'b': 0.5,
            'w': 0.5,
            't': 0.8
        }
    },

    # Rectangular box (generic)
    'rect_box': {
        'width_multiplier': 95.0,
        'min_overlap': 0.35,
        'nms_dilate_size': 11,
        'bg_percentile': 75,
        'depth_diff_thresh': 0.01,
        'pca_align_mode': 'perpendicular',
        'scoring_weights': {
            'q': 1.0,
            'o': 1.0,
            'b': 0.4,
            'w': 0.4,
            't': 0.8
        }
    },
}

# ============================================================================
# CONFIGURATION GUIDE
# ============================================================================
"""
Quick Tuning Guide for SIMPLE_GRASP_CONFIG:

1. START HERE:
   - Use SIMPLE_GRASP_CONFIG for 90% of use cases
   - Only change width_multiplier and min_object_overlap initially
   - Leave other parameters at defaults until you understand the system

2. WIDTH CALIBRATION (Most Important!):
   Step 1: Grasp a known object with calipers measurement
   Step 2: Check logged width in terminal (e.g., "width=25.0mm")
   Step 3: Calculate new multiplier: new = old * (actual_width / logged_width)
   Step 4: Update 'width_multiplier' in SIMPLE_GRASP_CONFIG
   
   Example:
   - Actual screwdriver: 30mm (measured with calipers)
   - Logged width: 25mm (from system output)
   - New multiplier: 95 * (30/25) = 114
   - Update: 'width_multiplier': 114.0

3. OVERLAP TUNING (Quality Control):
   Too many bad grasps (tips, edges)?
   → Increase 'min_object_overlap' to 0.30-0.40
   
   Too few grasps found?
   → Decrease 'min_object_overlap' to 0.15-0.20
   
   Guidelines by object type:
   - Flat objects (books, boxes): 0.30-0.40
   - Cylindrical objects (screwdrivers, markers): 0.25-0.30
   - Small objects (USB drives, batteries): 0.20-0.25

4. TEMPORAL SMOOTHING (Jitter Reduction):
   Robot movements jittery or unstable?
   → Increase 'temporal_window' to 7-10 frames
   
   Robot too slow to react to changes?
   → Decrease 'temporal_window' to 3-5 frames
   
   Best practice: Start with 5, only change if needed

5. OBJECT PROFILES (Advanced):
   Once you find working settings for an object:
   - Save them in OBJECT_PROFILES dictionary
   - Switch profiles based on task or detected object type
   - Reuse successful configurations

6. WHEN TO MODIFY ADVANCED SETTINGS:
   Most users should NEVER touch GRASP_DETECTION_CONFIG directly.
   Only modify if:
   - You understand the technical details
   - SIMPLE_GRASP_CONFIG doesn't provide enough control
   - You're debugging specific issues (e.g., NMS, PCA)

For debugging and visualization:
- Set DEBUG_MODE = True in system_config.py
- Check terminal logs for quality, overlap, width values
- Use visualization windows to see selected grasps

Common Issues:
- "No valid grasp found" → Lower min_object_overlap or quality_threshold
- "Grasp at tip" → Increase min_object_overlap
- "Width too large/small" → Recalibrate width_multiplier
- "Jittery angles" → Increase temporal_window
"""
