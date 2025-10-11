"""
Debug script to analyze GR-ConvNet angle predictions across different object orientations.
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

# Project imports
# fmt: off
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
    depth_array = depth_array.astype(np.float32)
    if hasattr(depth_frame, 'get_units'):
        depth_units = depth_frame.get_units()
        depth_array = depth_array.astype(np.float32) * depth_units

# Preprocess (same as fixed preprocessing)
INPUT_SIZE = 300
h, w = depth_array.shape
left = (w - INPUT_SIZE) // 2
top = (h - INPUT_SIZE) // 2

depth_crop = depth_array[top:top+INPUT_SIZE,
                         left:left+INPUT_SIZE].astype(np.float32)
color_crop = color_array[top:top+INPUT_SIZE, left:left+INPUT_SIZE]

# Depth inpainting
if np.any(depth_crop == 0):
    scale = np.abs(depth_crop).max() if np.abs(depth_crop).max() > 0 else 1.0
    depth_scaled = (depth_crop / scale).astype(np.float32)
    depth_padded = cv2.copyMakeBorder(
        depth_scaled, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    mask = np.pad((depth_crop == 0).astype(np.uint8),
                  1, mode='constant', constant_values=0)
    depth_inpainted = cv2.inpaint(depth_padded, mask, 1, cv2.INPAINT_NS)
    depth_crop = depth_inpainted[1:-1, 1:-1] * scale

# RGB: /255 then zero-center
color_rgb = cv2.cvtColor(
    color_crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
rgb_norm = color_rgb - color_rgb.mean()

# Depth: Mean-center and clip to [-1, 1]
depth_norm = np.clip(depth_crop - depth_crop.mean(), -1, 1)

# Stack: [D, R, G, B]
rgbd = np.dstack([depth_norm[:, :, None], rgb_norm])
rgbd_tensor = torch.from_numpy(rgbd).permute(2, 0, 1).unsqueeze(0).float()

# Load model
model = GRConvNet(input_channels=4, channel_size=32, input_size=INPUT_SIZE)
state_dict = torch.load("src/resources/ml_models/grconvnet_weights/grconvnet_cornell.pt",
                        map_location='cpu', weights_only=True)
model.load_state_dict(state_dict)
model.eval()

# Inference
with torch.no_grad():
    pos, cos, sin, width = model(rgbd_tensor)

# Decode
q_img = torch.sigmoid(pos).squeeze().cpu().numpy()
ang_img = (0.5 * torch.atan2(sin, cos)).squeeze().cpu().numpy()
width_img = (F.relu(width) * (INPUT_SIZE / 2.0)).squeeze().cpu().numpy()

# Find top 10 candidates
q_flat = q_img.flatten()
top_10_indices = np.argsort(q_flat)[-10:][::-1]
top_10_2d = np.unravel_index(top_10_indices, q_img.shape)

print("\n" + "="*60)
print("TOP 10 GRASP CANDIDATES")
print("="*60)

for i, (v, u) in enumerate(zip(top_10_2d[0], top_10_2d[1])):
    network_angle = ang_img[v, u]
    quality = q_img[v, u]
    width_px = width_img[v, u]

    # Test different offsets
    jaw_plus_90 = network_angle + np.pi/2
    jaw_minus_90 = network_angle - np.pi/2
    jaw_no_offset = network_angle

    print(
        f"\n#{i+1}: Position ({u}, {v}), Quality: {quality:.3f}, Width: {width_px:.1f}px")
    print(f"  Network angle: {np.degrees(network_angle):6.1f}° (contact line)")
    print(f"  With NO offset: {np.degrees(jaw_no_offset):6.1f}°")
    print(f"  With +90° offset: {np.degrees(jaw_plus_90):6.1f}°")
    print(f"  With -90° offset: {np.degrees(jaw_minus_90):6.1f}°")

# Show angle distribution
print("\n" + "="*60)
print("ANGLE DISTRIBUTION STATISTICS")
print("="*60)
print(f"Min angle: {np.degrees(ang_img.min()):6.1f}°")
print(f"Max angle: {np.degrees(ang_img.max()):6.1f}°")
print(f"Mean angle: {np.degrees(ang_img.mean()):6.1f}°")
print(f"Median angle: {np.degrees(np.median(ang_img)):6.1f}°")

# Show histogram
angle_bins = np.linspace(-90, 90, 19)  # 10° bins
hist, _ = np.histogram(np.degrees(ang_img), bins=angle_bins)
print("\nAngle histogram (degrees):")
for i in range(len(hist)):
    bar = "█" * int(hist[i] / hist.max() * 40)
    print(
        f"  {angle_bins[i]:6.1f}° to {angle_bins[i+1]:6.1f}°: {bar} ({hist[i]})")

# Visualize top 3 with different offsets
fig, axes = plt.subplots(2, 4, figsize=(16, 8))

# Original image
axes[0, 0].imshow(color_rgb)
axes[0, 0].set_title('RGB Image')
axes[0, 0].axis('off')

# Quality map
im = axes[0, 1].imshow(q_img, cmap='hot')
plt.colorbar(im, ax=axes[0, 1])
axes[0, 1].set_title('Quality Map')
axes[0, 1].axis('off')

# Angle map
im = axes[0, 2].imshow(np.degrees(ang_img), cmap='hsv', vmin=-90, vmax=90)
plt.colorbar(im, ax=axes[0, 2])
axes[0, 2].set_title('Angle Map (degrees)')
axes[0, 2].axis('off')

# Width map
im = axes[0, 3].imshow(width_img, cmap='viridis')
plt.colorbar(im, ax=axes[0, 3])
axes[0, 3].set_title('Width Map')
axes[0, 3].axis('off')

# Top 3 candidates with different offsets
offsets = [0, np.pi/2, -np.pi/2]
offset_names = ['No offset', '+90°', '-90°']

for col, (offset, name) in enumerate(zip(offsets, offset_names)):
    ax = axes[1, col+1]
    ax.imshow(color_rgb)

    for i in range(min(3, len(top_10_indices))):
        v, u = top_10_2d[0][i], top_10_2d[1][i]
        angle = ang_img[v, u] + offset
        width_px = width_img[v, u]

        # Draw rectangle
        half_w = max(width_px / 2, 5)
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        corners = np.array([
            [-half_w, -10], [half_w, -10], [half_w, 10], [-half_w, 10]
        ])
        rot_mat = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        corners_rot = corners @ rot_mat.T
        corners_rot[:, 0] += u
        corners_rot[:, 1] += v

        color = ['green', 'yellow', 'cyan'][i]
        ax.plot(corners_rot[[0, 1, 2, 3, 0], 0], corners_rot[[0, 1, 2, 3, 0], 1],
                color=color, linewidth=2, alpha=0.7)
        ax.plot(u, v, 'o', color=color, markersize=8)

        # Draw arrow
        arrow_len = 30
        ax.arrow(u, v, arrow_len*cos_a, arrow_len*sin_a,
                 color=color, width=2, head_width=8, alpha=0.7)

    ax.set_title(f'{name} offset')
    ax.axis('off')

axes[1, 0].axis('off')

plt.tight_layout()
plt.savefig('grconvnet_angle_debug.png', dpi=150)
print(f"\n✅ Visualization saved to: grconvnet_angle_debug.png")
plt.show()

camera.cleanup()
