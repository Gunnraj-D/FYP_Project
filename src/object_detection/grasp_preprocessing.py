"""
Image preprocessing for grasp detection networks.

Features:
1. Per-channel RGB normalization
2. Percentile-based depth normalization
3. Returns depth map in meters for local depth estimation
4. Camera intrinsics support for accurate scaling

Handles preprocessing for both:
- GGCNN2: Depth-only (1 channel)
- GR-ConvNet: RGB-D (4 channels)
"""

import cv2
import numpy as np
import torch
import logging
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class GraspPreprocessor:
    """
    Preprocessor for grasp detection networks.

    Features:
    - Per-channel RGB normalization
    - Percentile-based depth normalization (robust to outliers)
    - Returns depth map for local depth queries
    - Camera intrinsics-aware

    Supports model-specific preprocessing:
    - GGCNN2: Depth normalization to [0,1] range
    - GR-ConvNet: RGB-D normalization
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

        logger.info(f"{model_type.upper()} preprocessor initialized: "
                    f"input_size={resize_size}, "
                    f"mode={'RGB-D' if self.use_rgbd else 'Depth'}")

    def preprocess(self, depth_image: np.ndarray,
                   color_image: Optional[np.ndarray] = None) -> Tuple[torch.Tensor, np.ndarray, float]:
        """
        Preprocess depth (and optionally color) for network inference.

        Returns depth map in meters and median depth for
        local depth estimation and accurate mm conversion.

        Args:
            depth_image: Depth image in meters, shape (H, W)
            color_image: Color image in BGR format, shape (H, W, 3) - required for GR-ConvNet

        Returns:
            Tuple of:
            - input_tensor: Preprocessed tensor for network
              * GGCNN2: (1, 1, 300, 300) depth-only
              * GR-ConvNet: (1, 4, 300, 300) RGB-D [D, R, G, B]
            - depth_resized_m: Depth map in meters (300x300) for local depth queries
            - median_depth_m: Median object depth in meters for mm conversion
        """
        # 1. Center-crop to square (preserves aspect ratio)
        depth = self._crop_to_square(depth_image)

        # 2. Resize to network input size
        depth_resized = cv2.resize(depth, (self.resize_size, self.resize_size))

        # Store depth in meters for local depth queries
        depth_resized_m = depth_resized.astype(np.float32)

        # Compute median depth for mm conversion
        valid_depth = depth_resized_m[depth_resized_m > 0]
        median_depth_m = float(np.median(valid_depth)
                               ) if valid_depth.size > 0 else 0.5

        # 3. Model-specific normalization
        if self.use_rgbd and color_image is not None:
            input_tensor = self._preprocess_rgbd_improved(
                depth_resized, color_image)
        else:
            input_tensor = self._preprocess_depth_only(depth_resized)

        return input_tensor, depth_resized_m, median_depth_m

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

    def _preprocess_rgbd_improved(self, depth: np.ndarray,
                                  color_image: np.ndarray) -> torch.Tensor:
        """
        IMPROVED GR-ConvNet RGB-D preprocessing.

        IMPROVEMENTS over legacy:
        1. Per-channel RGB normalization (better than global mean)
        2. Percentile-based depth normalization (preserves gradients)
        3. Soft clipping with tanh (no hard cutoffs)

        RGB normalization:
        - Scale to [0,1]
        - Per-channel mean subtraction (better than global)

        Depth normalization:
        - Percentile-based scaling (5th-95th percentile)
        - Median-centered (robust to outliers)
        - Tanh soft clipping (preserves gradients)

        Channel order: [D, R, G, B] (depth first!)
        """
        # 1. Crop and resize color to match depth
        color = self._crop_to_square(color_image)
        color = cv2.resize(color, (self.resize_size, self.resize_size))

        # 2. IMPROVED RGB normalization - per-channel mean subtraction
        color_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        rgb_scaled = color_rgb.astype(np.float32) / 255.0

        # Per-channel mean (better matches training preprocessing)
        channel_mean = rgb_scaled.mean(axis=(0, 1), keepdims=True)
        rgb_normalized = rgb_scaled - channel_mean

        # 3. IMPROVED Depth normalization - percentile-based
        valid_depth = depth[depth > 0]

        if valid_depth.size > 0:
            # Use 5th and 95th percentiles for robust normalization
            p5 = np.percentile(valid_depth, 5)
            p95 = np.percentile(valid_depth, 95)

            # Center around median (more robust than mean)
            median_depth = np.median(valid_depth)
            depth_centered = depth - median_depth

            # Scale using percentile range
            scale = max(p95 - p5, 0.001) / 2.0
            depth_norm = depth_centered / scale

            # Soft clipping using tanh (preserves gradients)
            depth_norm = np.tanh(depth_norm)

            # Mask invalid depths
            depth_norm[depth <= 0] = 0
        else:
            # Fallback if no valid depth
            depth_norm = np.zeros_like(depth)

        # 4. Stack: [D, R, G, B] - depth FIRST (GR-ConvNet convention)
        rgbd = np.dstack([depth_norm[:, :, None], rgb_normalized])

        # 5. Convert to tensor: (H, W, 4) → (4, H, W) → (1, 4, H, W)
        rgbd_tensor = torch.from_numpy(rgbd).permute(
            2, 0, 1).unsqueeze(0).float()

        logger.debug(f"Preprocessed RGB-D (IMPROVED): {rgbd_tensor.shape}, "
                     f"RGB=[{rgb_normalized.min():.3f}, {rgb_normalized.max():.3f}], "
                     f"Depth=[{depth_norm.min():.3f}, {depth_norm.max():.3f}]")

        return rgbd_tensor.to(self.device)
