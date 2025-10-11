"""
Debug GR-ConvNet preprocessing to identify issues.
Checks RGB-D input format and network outputs.
"""

from camera_management.camera_manager import CameraManager
from object_detection.grconvnet import GRConvNet
import sys
import os
import torch
import torch.nn.functional as F
import numpy as np
import cv2

src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
sys.path.insert(0, src_path)


print("=" * 70)
print("GR-ConvNet Preprocessing Diagnostic")
print("=" * 70)

# Initialize camera
print("\n1. Initializing camera...")
camera = CameraManager()
if not camera.initialize():
    print("   ✗ Failed to initialize camera")
    exit(1)
print("   ✓ Camera initialized")

# Get frames
print("\n2. Capturing RGB-D frames...")
color_frame, depth_frame = camera.get_frames()
if color_frame is None or depth_frame is None:
    print("   ✗ Failed to get frames")
    exit(1)

# Convert to arrays (handle both RealSense frames and numpy arrays)
if hasattr(color_frame, 'get_data'):
    color_array = np.asanyarray(color_frame.get_data())
else:
    color_array = color_frame

if hasattr(depth_frame, 'get_data'):
    depth_array = np.asanyarray(depth_frame.get_data())
    depth_units = depth_frame.get_units()
    depth_array = depth_array.astype(np.float32) * depth_units
else:
    depth_array = depth_frame  # Already in meters

print(f"   ✓ Color shape: {color_array.shape} (BGR)")
print(f"   ✓ Depth shape: {depth_array.shape} (meters)")
print(f"   Color range: [{color_array.min()}, {color_array.max()}]")
print(
    f"   Depth range: [{depth_array[depth_array>0].min():.3f}m, {depth_array.max():.3f}m]")

# Preprocess as grasp_detector_module does
print("\n3. Preprocessing RGB-D...")

# Crop to square
h, w = depth_array.shape
min_dim = min(h, w)
start_h = (h - min_dim) // 2
start_w = (w - min_dim) // 2
depth_crop = depth_array[start_h:start_h+min_dim, start_w:start_w+min_dim]
color_crop = color_array[start_h:start_h+min_dim, start_w:start_w+min_dim]

# Resize
depth_resized = cv2.resize(depth_crop, (300, 300))
color_resized = cv2.resize(color_crop, (300, 300))

# Normalize depth
depth_normalized = np.clip(depth_resized, 0.2, 1.2)
depth_normalized = (depth_normalized - 0.2) / 1.0

# Normalize color
color_rgb = cv2.cvtColor(color_resized, cv2.COLOR_BGR2RGB)
color_normalized = color_rgb.astype(np.float32) / 255.0

print(
    f"   Depth normalized: [{depth_normalized.min():.3f}, {depth_normalized.max():.3f}]")
print(
    f"   Color normalized: [{color_normalized.min():.3f}, {color_normalized.max():.3f}]")

# Stack RGBD
rgbd = np.dstack([color_normalized, depth_normalized[:, :, None]])
print(f"   RGBD stacked shape: {rgbd.shape} (H, W, 4)")

# Convert to tensor
rgbd_tensor = torch.from_numpy(rgbd).permute(2, 0, 1).unsqueeze(0).float()
print(f"   ✓ RGBD tensor shape: {rgbd_tensor.shape} (B, C, H, W)")
print(
    f"   Channel 0 (R) range: [{rgbd_tensor[0,0].min():.3f}, {rgbd_tensor[0,0].max():.3f}]")
print(
    f"   Channel 1 (G) range: [{rgbd_tensor[0,1].min():.3f}, {rgbd_tensor[0,1].max():.3f}]")
print(
    f"   Channel 2 (B) range: [{rgbd_tensor[0,2].min():.3f}, {rgbd_tensor[0,2].max():.3f}]")
print(
    f"   Channel 3 (D) range: [{rgbd_tensor[0,3].min():.3f}, {rgbd_tensor[0,3].max():.3f}]")

# Load model
print("\n4. Loading GR-ConvNet model...")
model = GRConvNet(input_channels=4, channel_size=32, input_size=300)
weights_path = "src/resources/ml_models/grconvnet_weights/grconvnet_jacquard.pt"
state_dict = torch.load(weights_path, map_location='cpu', weights_only=True)
model.load_state_dict(state_dict)
model.eval()
print(f"   ✓ Model loaded")

# Run inference
print("\n5. Running inference...")
with torch.no_grad():
    pos, cos, sin, width = model(rgbd_tensor)

print(f"   ✓ Inference complete")
print(
    f"   Output shapes: pos={pos.shape}, cos={cos.shape}, sin={sin.shape}, width={width.shape}")

# Decode
q_img = torch.sigmoid(pos)
ang_img = 0.5 * torch.atan2(sin, cos)
width_img = F.relu(width)

print(f"\n6. Decoded outputs:")
print(
    f"   Quality:  [{q_img.min():.3f}, {q_img.max():.3f}], mean={q_img.mean():.3f}")
print(f"   Angle:    [{np.degrees(ang_img.min()):.1f}°, {np.degrees(ang_img.max()):.1f}°], mean={np.degrees(ang_img.mean()):.1f}°")
print(
    f"   Width:    [{width_img.min():.1f}, {width_img.max():.1f}], mean={width_img.mean():.1f}")

# Find best grasp
q_np = q_img.squeeze().cpu().numpy()
ang_np = ang_img.squeeze().cpu().numpy()
max_idx = np.unravel_index(np.argmax(q_np), q_np.shape)
row, col = max_idx

print(f"\n7. Best grasp candidate:")
print(f"   Location: (u={col}, v={row})")
print(f"   Quality: {q_np[row, col]:.3f}")
print(f"   Angle: {np.degrees(ang_np[row, col]):.1f}°")
print(f"   Width: {width_img.squeeze()[row, col]:.1f} px")

# Check if quality map is reasonable
if q_np.max() < 0.1:
    print(f"\n⚠️  WARNING: Very low quality scores!")
    print(f"   This suggests preprocessing mismatch or wrong normalization")
elif q_np.max() < 0.3:
    print(f"\n⚠️  WARNING: Low quality scores (max={q_np.max():.3f})")
    print(f"   Network may not be seeing objects correctly")
else:
    print(f"\n✓ Quality scores look reasonable (max={q_np.max():.3f})")

# Check angle distribution
ang_std = np.std(ang_np)
if ang_std < 0.1:
    print(
        f"\n⚠️  WARNING: Angles have very low variance (std={np.degrees(ang_std):.1f}°)")
    print(
        f"   All angles near {np.degrees(ang_np.mean()):.1f}° - suggests network issue")
else:
    print(
        f"\n✓ Angle distribution looks reasonable (std={np.degrees(ang_std):.1f}°)")

print("\n" + "=" * 70)
camera.stop()
