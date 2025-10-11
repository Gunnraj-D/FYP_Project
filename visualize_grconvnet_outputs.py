"""
Visualize GR-ConvNet output maps to understand grasp predictions.
Shows quality, angle, and width maps side-by-side.
"""

import matplotlib.pyplot as plt
import cv2
import numpy as np
import torch.nn.functional as F
import torch
import sys
import os

# Add src to path BEFORE importing project modules
src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
sys.path.insert(0, src_path)

# Standard library and third-party imports

# Project imports (must come AFTER sys.path modification)
# fmt: off
# isort: skip_file
from object_detection.grconvnet import GRConvNet
from camera_management.camera_manager import CameraManager
# fmt: on


# Initialize camera
camera = CameraManager()
camera.initialize()

# Get frames
color_frame, depth_frame = camera.get_frames()
color_array = color_frame if isinstance(
    color_frame, np.ndarray) else np.asanyarray(color_frame.get_data())
depth_array = depth_frame if isinstance(
    depth_frame, np.ndarray) else np.asanyarray(depth_frame.get_data())

if not isinstance(depth_array, np.ndarray) or depth_array.dtype != np.float32:
    if hasattr(depth_frame, 'get_units'):
        depth_units = depth_frame.get_units()
        depth_array = depth_array.astype(np.float32) * depth_units

# Preprocess
# GR-ConvNet: CENTER-CROP to exact 300x300 (NO RESIZE, matches training)
INPUT_SIZE = 300
h, w = depth_array.shape
left = (w - INPUT_SIZE) // 2
top = (h - INPUT_SIZE) // 2

depth_crop = depth_array[top:top+INPUT_SIZE,
                         left:left+INPUT_SIZE].astype(np.float32)
color_crop = color_array[top:top+INPUT_SIZE, left:left+INPUT_SIZE]

# Depth inpainting (if missing values)
if np.any(depth_crop == 0):
    scale = np.abs(depth_crop).max() if np.abs(depth_crop).max() > 0 else 1.0
    depth_scaled = (depth_crop / scale).astype(np.float32)
    depth_padded = cv2.copyMakeBorder(
        depth_scaled, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    mask = np.pad((depth_crop == 0).astype(np.uint8),
                  1, mode='constant', constant_values=0)
    depth_inpainted = cv2.inpaint(depth_padded, mask, 1, cv2.INPAINT_NS)
    depth_crop = depth_inpainted[1:-1, 1:-1] * scale
    print(f"📌 Inpainted {np.sum(mask)} missing depth pixels")

# RGB: /255 then zero-center
color_rgb = cv2.cvtColor(
    color_crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
rgb_norm = color_rgb - color_rgb.mean()

# Depth: Mean-center and clip to [-1, 1]
depth_mean_centered = depth_crop - depth_crop.mean()
depth_norm = np.clip(depth_mean_centered, -1, 1)

# Stack: [D, R, G, B] - Depth FIRST!
rgbd = np.dstack([depth_norm[:, :, None], rgb_norm])
rgbd_tensor = torch.from_numpy(rgbd).permute(2, 0, 1).unsqueeze(0).float()

print(f"\n🔍 Preprocessing check:")
print(
    f"   RGB normalized: [{rgb_norm.min():.3f}, {rgb_norm.max():.3f}] (zero-centered)")
print(
    f"   Depth normalized: [{depth_norm.min():.3f}, {depth_norm.max():.3f}] (mean-centered, clipped)")
print(f"   Channel order: [D, R, G, B]")

# Load model (use 300 for Jacquard-trained weights)
model = GRConvNet(input_channels=4, channel_size=32, input_size=INPUT_SIZE)
state_dict = torch.load("src/resources/ml_models/grconvnet_weights/grconvnet_cornell.pt",
                        map_location='cpu', weights_only=True)
model.load_state_dict(state_dict)
model.eval()

# Inference
with torch.no_grad():
    pos, cos, sin, width = model(rgbd_tensor)

# Decode (with GR-ConvNet width scaling: input_size / 2)
q_img = torch.sigmoid(pos).squeeze().cpu().numpy()
ang_img = (0.5 * torch.atan2(sin, cos)).squeeze().cpu().numpy()
width_img = (F.relu(width) * (INPUT_SIZE / 2.0)
             ).squeeze().cpu().numpy()  # 300/2 = 150

# Create visualization
fig, axes = plt.subplots(2, 3, figsize=(15, 10))

# Input color
axes[0, 0].imshow(color_rgb)
axes[0, 0].set_title('Input: Color (RGB)')
axes[0, 0].axis('off')

# Input depth
axes[0, 1].imshow(depth_norm, cmap='jet')
axes[0, 1].set_title(
    f'Input: Depth (normalized)\nRange: [{depth_norm.min():.2f}, {depth_norm.max():.2f}]')
axes[0, 1].axis('off')

# Quality map
im_q = axes[0, 2].imshow(q_img, cmap='hot', vmin=0, vmax=1)
axes[0, 2].set_title(
    f'Quality Map\nRange: [{q_img.min():.3f}, {q_img.max():.3f}]')
plt.colorbar(im_q, ax=axes[0, 2])

# Find best grasp
max_idx = np.unravel_index(np.argmax(q_img), q_img.shape)
best_v, best_u = max_idx
axes[0, 2].plot(best_u, best_v, 'gx', markersize=15, markeredgewidth=3)
axes[0, 2].set_title(
    f'Quality Map\nBest: ({best_u}, {best_v}) Q={q_img[best_v, best_u]:.3f}')

# Angle map
im_ang = axes[1, 0].imshow(np.degrees(ang_img), cmap='hsv', vmin=-90, vmax=90)
axes[1, 0].set_title(
    f'Angle Map (degrees)\nRange: [{np.degrees(ang_img.min()):.1f}°, {np.degrees(ang_img.max()):.1f}°]')
axes[1, 0].plot(best_u, best_v, 'wx', markersize=15, markeredgewidth=3)
plt.colorbar(im_ang, ax=axes[1, 0])

# Width map
im_w = axes[1, 1].imshow(width_img, cmap='viridis')
axes[1, 1].set_title(
    f'Width Map (pixels)\nRange: [{width_img.min():.2f}, {width_img.max():.2f}]')
axes[1, 1].plot(best_u, best_v, 'rx', markersize=15, markeredgewidth=3)
plt.colorbar(im_w, ax=axes[1, 1])

# Grasp visualization on color
axes[1, 2].imshow(color_rgb)
angle_at_best = ang_img[best_v, best_u]
width_at_best = width_img[best_v, best_u]

# Apply the same -90° offset that the actual grasping code uses
ANGLE_OFFSET = -1.5708  # -90° in radians (from config, corrected sign)
jaw_axis_angle = angle_at_best + ANGLE_OFFSET

# Draw grasp rectangle using JAW AXIS angle (after offset)
half_w = max(width_at_best / 2, 5)  # Minimum 5px for visibility
cos_a, sin_a = np.cos(jaw_axis_angle), np.sin(jaw_axis_angle)
corners = np.array([
    [-half_w, -10], [half_w, -10], [half_w, 10], [-half_w, 10]
])
rot_mat = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
corners_rot = corners @ rot_mat.T
corners_rot[:, 0] += best_u
corners_rot[:, 1] += best_v
axes[1, 2].plot(corners_rot[[0, 1, 2, 3, 0], 0],
                corners_rot[[0, 1, 2, 3, 0], 1], 'g-', linewidth=2)
axes[1, 2].plot(best_u, best_v, 'go', markersize=10)

# Draw angle arrow
arrow_len = 30
arrow_end_u = best_u + arrow_len * cos_a
arrow_end_v = best_v + arrow_len * sin_a
axes[1, 2].arrow(best_u, best_v, arrow_len*cos_a, arrow_len*sin_a,
                 color='yellow', width=3, head_width=10)

axes[1, 2].set_title(
    f'Best Grasp (WITH -90° offset)\n'
    f'Network: {np.degrees(angle_at_best):.1f}° → Jaw: {np.degrees(jaw_axis_angle):.1f}°\n'
    f'Width: {width_at_best:.1f}px')
axes[1, 2].axis('off')

plt.tight_layout()
plt.savefig('grconvnet_output_visualization.png', dpi=150)
print(f"\n✅ Visualization saved to: grconvnet_output_visualization.png")
print(f"\n📊 Summary:")
print(f"   Best grasp location: (u={best_u}, v={best_v})")
print(f"   Quality: {q_img[best_v, best_u]:.3f}")
print(f"   📐 Network angle (long axis): {np.degrees(angle_at_best):.1f}°")
print(
    f"   🔄 Jaw axis (after -90°): {np.degrees(jaw_axis_angle):.1f}° ← ACTUAL GRASP ANGLE")
print(f"   Width: {width_at_best:.1f} px")

plt.show()
camera.cleanup()
