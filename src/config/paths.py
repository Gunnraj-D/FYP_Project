"""
File paths and directory configuration.
"""
from pathlib import Path

# Base directories
SRC_DIR = Path(__file__).parent.parent

# ============================================================================
# MODEL PATHS
# ============================================================================

# GGCNN2 model
GGCNN2_MODEL_PATH = SRC_DIR / "resources" / "ml_models" / \
    "ggcnn2_weights_cornell" / "epoch_50_cornell_statedict.pt"

# GR-ConvNet model (trained on Jacquard dataset)
GRCONVNET_MODEL_PATH = SRC_DIR / "resources" / "ml_models" / \
    "grconvnet_weights" / "grconvnet_jacquard.pt"

# MediaPipe hand detection model
HANDMODEL_FILEPATH = SRC_DIR / "resources" / "ml_models" / \
    "hand_landmarker.task"

# KUKA iiwa14 URDF model
URDF_FILEPATH = SRC_DIR / "resources" / "robot_models" / \
    "kuka_with_gripper.urdf"

# Hand-eye matrix file
HAND_EYE_MATRIX_FILE = "src/hand_eye_matrix.npy"
