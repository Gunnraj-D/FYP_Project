"""
Image preprocessing for grasp detection networks.

Handles preprocessing for both:
- GGCNN2: Depth-only (1 channel)
- GR-ConvNet: RGB-D (4 channels)
"""

import cv2
import numpy as np
import torch
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class GraspPreprocessor:
    """
    Preprocesses depth and color images for grasp detection networks.

    Supports model-specific preprocessing:
    - GGCNN2: Depth normalization to [0,1] range
    - GR-ConvNet: RGB-D with zero-centering and mean-centering
    """

    def __init__(self, model_type: str, resize_size: int = 300, device: str = 'cpu'):
        """
        Args:
            model_type: 'ggcnn2' or 'grconvnet'
            resize_size: Target size for network input (default 300x300)
            device: 'cpu' or 'cuda'
        """
        self.model_type = model_type
        self.resize_size = resize_size
        self.device = device
        self.use_rgbd = (model_type == 'grconvnet')

        logger.info(f"Initialized {model_type.upper()} preprocessor: "
                    f"input_size={resize_size}, "
                    f"mode={'RGB-D' if self.use_rgbd else 'Depth-only'}")

    def preprocess(self, depth_image: np.ndarray,
                   color_image: Optional[np.ndarray] = None) -> torch.Tensor:
        """
        Preprocess depth (and optionally color) for network inference.

        Args:
            depth_image: Depth image in meters, shape (H, W)
            color_image: Color image in BGR format, shape (H, W, 3) - required for GR-ConvNet

        Returns:
            Preprocessed tensor:
            - GGCNN2: (1, 1, 300, 300) depth-only
            - GR-ConvNet: (1, 4, 300, 300) RGB-D [D, R, G, B]
        """
        # 1. Center-crop to square (preserves aspect ratio)
        depth = self._crop_to_square(depth_image)

        # 2. Resize to network input size
        depth = cv2.resize(depth, (self.resize_size, self.resize_size))

        # 3. Model-specific normalization
        if self.use_rgbd and color_image is not None:
            return self._preprocess_rgbd(depth, color_image)
        else:
            return self._preprocess_depth_only(depth)

    def _crop_to_square(self, image: np.ndarray) -> np.ndarray:
        """Center-crop image to square aspect ratio."""
        h, w = image.shape[:2]
        min_dim = min(h, w)
        start_h = (h - min_dim) // 2
        start_w = (w - min_dim) // 2
        return image[start_h:start_h+min_dim, start_w:start_w+min_dim]

    def _preprocess_depth_only(self, depth: np.ndarray) -> torch.Tensor:
        """
        GGCNN2 depth preprocessing.

        Normalization:
        - Clip to [0.2, 1.2] meters (table working range)
        - Normalize to [0, 1]
        """
        # Clip and normalize
        depth_normalized = np.clip(depth, 0.2, 1.2)
        depth_normalized = (depth_normalized - 0.2) / 1.0  # [0,1]

        # Convert to tensor: (H, W) → (1, 1, H, W)
        depth_tensor = torch.from_numpy(
            depth_normalized).unsqueeze(0).unsqueeze(0).float()

        logger.debug(f"Preprocessed depth: {depth_tensor.shape}, "
                     f"range=[{depth_tensor.min():.3f}, {depth_tensor.max():.3f}]")

        return depth_tensor.to(self.device)

    def _preprocess_rgbd(self, depth: np.ndarray,
                         color_image: np.ndarray) -> torch.Tensor:
        """
        GR-ConvNet RGB-D preprocessing.

        RGB normalization (from GR-ConvNet training):
        - Scale to [0,1]
        - Zero-center by subtracting mean

        Depth normalization:
        - Mean-center
        - Clip to [-1, 1]

        Channel order: [D, R, G, B] (depth first!)
        """
        # 1. Crop and resize color to match depth
        color = self._crop_to_square(color_image)
        color = cv2.resize(color, (self.resize_size, self.resize_size))

        # 2. RGB normalization
        color_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        rgb_scaled = color_rgb.astype(np.float32) / 255.0
        rgb_normalized = rgb_scaled - rgb_scaled.mean()  # Zero-center

        # 3. Depth normalization
        depth_mean_centered = depth - depth.mean()
        depth_normalized = np.clip(depth_mean_centered, -1, 1)

        # 4. Stack: [D, R, G, B] - depth FIRST (GR-ConvNet convention)
        rgbd = np.dstack([depth_normalized[:, :, None], rgb_normalized])

        # 5. Convert to tensor: (H, W, 4) → (4, H, W) → (1, 4, H, W)
        rgbd_tensor = torch.from_numpy(rgbd).permute(
            2, 0, 1).unsqueeze(0).float()

        logger.debug(f"Preprocessed RGB-D: {rgbd_tensor.shape}, "
                     f"RGB=[{rgb_normalized.min():.3f}, {rgb_normalized.max():.3f}], "
                     f"Depth=[{depth_normalized.min():.3f}, {depth_normalized.max():.3f}]")

        return rgbd_tensor.to(self.device)
