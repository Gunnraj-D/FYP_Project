"""
Visualize GR-ConvNet output maps to understand grasp predictions.
Shows quality, angle, and width maps side-by-side.
"""


from object_detection.grconvnet import GRConvNet
from camera_management.camera_manager import CameraManager
import sys
import os
import torch
import torch.nn.functional as F
import numpy as np
import cv2
import matplotlib.pyplot as plt

src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
sys.path.insert(0, src_path)


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
h, w = depth_array.shape
min_dim = min(h, w)
start_h = (h - min_dim) // 2
start_w = (w - min_dim) // 2

depth_crop = depth_array[start_h:start_h+min_dim, start_w:start_w+min_dim]
color_crop = color_array[start_h:start_h+min_dim, start_w:start_w+min_dim]

depth_resized = cv2.resize(depth_crop, (300, 300))
color_resized = cv2.resize(color_crop, (300, 300))

# GR-ConvNet Normalization (from image.py)
# RGB: Scale to [0,1] then zero-center
color_rgb = cv2.cvtColor(color_resized, cv2.COLOR_BGR2RGB)
rgb_scaled = color_rgb.astype(np.float32) / 255.0
rgb_norm = rgb_scaled - rgb_scaled.mean()  # Zero-center

# Depth: Mean-center and clip to [-1, 1]
depth_mean_centered = depth_resized - depth_resized.mean()
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

# Load model
model = GRConvNet(input_channels=4, channel_size=32, input_size=300)
state_dict = torch.load("src/resources/ml_models/grconvnet_weights/grconvnet_jacquard.pt",
                        map_location='cpu', weights_only=True)
model.load_state_dict(state_dict)
model.eval()

# Inference
with torch.no_grad():
    pos, cos, sin, width = model(rgbd_tensor)

# Decode (with GR-ConvNet width scaling)
q_img = torch.sigmoid(pos).squeeze().cpu().numpy()
ang_img = (0.5 * torch.atan2(sin, cos)).squeeze().cpu().numpy()
width_img = (F.relu(width) * 150.0).squeeze().cpu().numpy()  # Scale by 150!

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

# Draw grasp rectangle
half_w = max(width_at_best / 2, 5)  # Minimum 5px for visibility
cos_a, sin_a = np.cos(angle_at_best), np.sin(angle_at_best)
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
    f'Best Grasp Visualization\nAngle: {np.degrees(angle_at_best):.1f}°, Width: {width_at_best:.1f}px')
axes[1, 2].axis('off')

plt.tight_layout()
plt.savefig('grconvnet_output_visualization.png', dpi=150)
print(f"\n✅ Visualization saved to: grconvnet_output_visualization.png")
print(f"\n📊 Summary:")
print(f"   Best grasp location: (u={best_u}, v={best_v})")
print(f"   Quality: {q_img[best_v, best_u]:.3f}")
print(f"   Angle: {np.degrees(angle_at_best):.1f}°")
print(f"   Width: {width_at_best:.1f} px")

plt.show()
camera.cleanup()
