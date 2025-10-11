"""
Visualize GR-ConvNet with TEMPORAL FILTERING.
Captures multiple frames, then applies filtering AFTER collection.
This prevents first-frame bias where an outlier anchors the filter.
"""


import sys
import os
import torch
import torch.nn.functional as F
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque

src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
sys.path.insert(0, src_path)

from object_detection.grconvnet import GRConvNet
from camera_management.camera_manager import CameraManager

def preprocess_rgbd(depth_array, color_array):
    """Preprocess RGB-D frame (same as grasp_detector_module)."""
    h, w = depth_array.shape
    min_dim = min(h, w)
    start_h = (h - min_dim) // 2
    start_w = (w - min_dim) // 2

    depth_crop = depth_array[start_h:start_h+min_dim, start_w:start_w+min_dim]
    color_crop = color_array[start_h:start_h+min_dim, start_w:start_w+min_dim]

    depth_resized = cv2.resize(depth_crop, (300, 300))
    color_resized = cv2.resize(color_crop, (300, 300))

    # RGB normalization
    color_rgb = cv2.cvtColor(color_resized, cv2.COLOR_BGR2RGB)
    rgb_scaled = color_rgb.astype(np.float32) / 255.0
    rgb_norm = rgb_scaled - rgb_scaled.mean()

    # Depth normalization
    depth_mean_centered = depth_resized - depth_resized.mean()
    depth_norm = np.clip(depth_mean_centered, -1, 1)

    # Stack: [D, R, G, B]
    rgbd = np.dstack([depth_norm[:, :, None], rgb_norm])
    rgbd_tensor = torch.from_numpy(rgbd).permute(2, 0, 1).unsqueeze(0).float()

    return rgbd_tensor, color_rgb, depth_norm


def process_frame(model, camera):
    """Process one frame and return raw angle (filtering happens later)."""
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
    rgbd_tensor, color_rgb, depth_norm = preprocess_rgbd(
        depth_array, color_array)

    # Inference
    with torch.no_grad():
        pos, cos, sin, width = model(rgbd_tensor)

    # Decode
    q_img = torch.sigmoid(pos).squeeze().cpu().numpy()
    ang_img = (0.5 * torch.atan2(sin, cos)).squeeze().cpu().numpy()
    width_img = (F.relu(width) * 150.0).squeeze().cpu().numpy()

    # Find best grasp
    max_idx = np.unravel_index(np.argmax(q_img), q_img.shape)
    best_v, best_u = max_idx
    raw_angle = ang_img[best_v, best_u]
    quality = q_img[best_v, best_u]

    return {
        'raw_angle': raw_angle,
        'quality': quality,
        'position': (best_u, best_v),
        'color_rgb': color_rgb,
        'q_img': q_img,
        'ang_img': ang_img,
        'width_img': width_img,
    }


def circular_mean(angles):
    """Compute circular mean of angles."""
    if not angles:
        return 0.0
    cos_sum = sum(np.cos(2 * a) for a in angles)
    sin_sum = sum(np.sin(2 * a) for a in angles)
    return 0.5 * np.arctan2(sin_sum, cos_sum)


def apply_temporal_filtering_batch(raw_angles, window_size=5, outlier_threshold_deg=30):
    """
    Apply temporal filtering to a batch of angles AFTER collection.
    This prevents first-frame bias where an outlier anchors the filter.

    Strategy:
    1. Collect all raw angles first
    2. Apply sliding window filter with at least 3 samples before outlier detection
    3. Reject outliers based on deviation from recent history

    Args:
        raw_angles: List of raw angle predictions (radians)
        window_size: Number of frames to average
        outlier_threshold_deg: Threshold for outlier rejection

    Returns:
        filtered_angles: List of filtered angles
        outlier_mask: Boolean array marking outliers
    """
    filtered_angles = []
    outlier_mask = []
    angle_history = deque(maxlen=window_size)

    def is_outlier(new_angle, history, threshold_deg):
        # Need at least 3 samples for robust outlier detection
        if len(history) < 3:
            return False

        ref_mean = circular_mean(list(history))
        diff = abs(new_angle - ref_mean)
        # Handle periodicity: angles differ by π are the same (gripper symmetry)
        diff = min(diff, np.pi - diff)

        return diff > np.deg2rad(threshold_deg)

    for i, angle in enumerate(raw_angles):
        # Check if outlier based on existing history
        if is_outlier(angle, angle_history, outlier_threshold_deg):
            outlier_mask.append(True)
            # Use previous filtered value instead of outlier
            if filtered_angles:
                filtered_angles.append(filtered_angles[-1])
            else:
                # First frame is outlier (shouldn't happen with 3-sample min)
                filtered_angles.append(angle)
        else:
            outlier_mask.append(False)
            # Add to history and compute filtered angle
            angle_history.append(angle)
            filtered = circular_mean(list(angle_history))
            filtered_angles.append(filtered)

    return filtered_angles, outlier_mask


def main():
    print("🎥 GR-ConvNet Temporal Filtering Test")
    print("=" * 70)
    print("Strategy: Capture all frames FIRST, then apply filtering")
    print("This prevents first-frame outliers from anchoring the filter")
    print("=" * 70)

    # Initialize camera
    camera = CameraManager()
    camera.initialize()

    # Load model
    model = GRConvNet(input_channels=4, channel_size=32, input_size=300)
    state_dict = torch.load("src/resources/ml_models/grconvnet_weights/grconvnet_cornell.pt",
                            map_location='cpu', weights_only=True)
    model.load_state_dict(state_dict)
    model.eval()

    # Capture N frames (NO FILTERING YET)
    num_frames = 10
    results = []

    print(f"\n📸 Capturing {num_frames} frames (raw angles only)...")
    print(f"{'Frame':<8} {'Raw Angle':<12} {'Quality':<10}")
    print("-" * 35)

    for i in range(num_frames):
        result = process_frame(model, camera)
        results.append(result)
        raw_deg = np.degrees(result['raw_angle'])
        print(f"{i+1:<8} {raw_deg:>10.1f}° {result['quality']:>8.3f}")

    camera.cleanup()

    # Extract raw angles
    raw_angles = [r['raw_angle'] for r in results]

    # NOW apply temporal filtering to the batch
    print(
        f"\n🔄 Applying temporal filtering (window={5}, outlier_threshold=30°)...")
    filtered_angles, outlier_mask = apply_temporal_filtering_batch(
        raw_angles, window_size=5, outlier_threshold_deg=30)

    # Display results
    print(f"\n{'Frame':<8} {'Raw':<12} {'Filtered':<12} {'Diff':<10} {'Status':<15}")
    print("-" * 70)

    for i in range(num_frames):
        raw_deg = np.degrees(raw_angles[i])
        filt_deg = np.degrees(filtered_angles[i])
        diff_deg = abs(raw_deg - filt_deg)
        status = "⚠️ OUTLIER" if outlier_mask[i] else "✓ OK"

        print(
            f"{i+1:<8} {raw_deg:>10.1f}° {filt_deg:>10.1f}° {diff_deg:>8.1f}° {status:<15}")

    # Compute statistics
    raw_angles_deg = [np.degrees(a) for a in raw_angles]
    filtered_angles_deg = [np.degrees(a) for a in filtered_angles]

    # Compute std only on non-outlier frames for fair comparison
    raw_valid = [raw_angles_deg[i]
                 for i in range(len(raw_angles_deg)) if not outlier_mask[i]]

    raw_std = np.std(raw_angles_deg)
    raw_valid_std = np.std(raw_valid)
    filtered_std = np.std(filtered_angles_deg)
    num_outliers = sum(outlier_mask)

    print("\n" + "=" * 70)
    print(f"📊 RESULTS:")
    print(
        f"   Raw angles (all)   : mean={np.mean(raw_angles_deg):.1f}°, std={raw_std:.2f}°")
    print(
        f"   Raw angles (valid) : mean={np.mean(raw_valid):.1f}°, std={raw_valid_std:.2f}°")
    print(
        f"   Filtered angles    : mean={np.mean(filtered_angles_deg):.1f}°, std={filtered_std:.2f}°")
    print(f"   Outliers detected  : {num_outliers}/{num_frames} frames")
    if raw_valid_std > 0:
        print(
            f"   Improvement        : {(1 - filtered_std/raw_valid_std)*100:.1f}% reduction in variance")

    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    frames = list(range(1, num_frames + 1))

    # Top-left: Angle over time
    ax = axes[0, 0]
    ax.plot(frames, raw_angles_deg, 'o-', label='Raw Angles', color='lightcoral',
            linewidth=2, markersize=8, alpha=0.7)
    ax.plot(frames, filtered_angles_deg, 's-', label='Filtered Angles', color='seagreen',
            linewidth=2, markersize=8)

    # Mark outliers
    outlier_frames = [frames[i] for i in range(num_frames) if outlier_mask[i]]
    outlier_angles = [raw_angles_deg[i]
                      for i in range(num_frames) if outlier_mask[i]]
    if outlier_frames:
        ax.plot(outlier_frames, outlier_angles, 'rx', markersize=15, markeredgewidth=3,
                label='Outliers (rejected)', zorder=10)

    ax.axhline(np.mean(filtered_angles_deg), color='blue', linestyle='--', alpha=0.5,
               label='Filtered Mean')
    ax.fill_between(frames,
                    np.mean(filtered_angles_deg) - filtered_std,
                    np.mean(filtered_angles_deg) + filtered_std,
                    alpha=0.2, color='blue', label='±1σ (Filtered)')
    ax.set_xlabel('Frame Number', fontsize=12)
    ax.set_ylabel('Angle (degrees)', fontsize=12)
    ax.set_title('Temporal Filtering: Angle Stability Over Time',
                 fontsize=13, fontweight='bold')
    ax.legend(fontsize=9, loc='best')
    ax.grid(True, alpha=0.3)

    # Top-right: Deviation from mean
    ax = axes[0, 1]
    filt_mean = np.mean(filtered_angles_deg)
    raw_dev = [a - filt_mean for a in raw_angles_deg]
    filt_dev = [a - filt_mean for a in filtered_angles_deg]

    x_pos = np.arange(len(frames))
    ax.bar(x_pos - 0.2, raw_dev, width=0.4,
           label='Raw Deviation', color='lightcoral', alpha=0.7)
    ax.bar(x_pos + 0.2, filt_dev, width=0.4,
           label='Filtered Deviation', color='seagreen', alpha=0.7)
    ax.axhline(0, color='black', linestyle='-', linewidth=0.8)
    ax.set_xlabel('Frame Number', fontsize=12)
    ax.set_ylabel('Deviation from Filtered Mean (°)', fontsize=12)
    ax.set_title('Frame-by-Frame Deviation', fontsize=13, fontweight='bold')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(frames)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')

    # Bottom-left: Histogram
    ax = axes[1, 0]
    bins = np.linspace(min(raw_angles_deg + filtered_angles_deg) - 5,
                       max(raw_angles_deg + filtered_angles_deg) + 5, 20)
    ax.hist(raw_angles_deg, bins=bins, alpha=0.5, label='Raw',
            color='lightcoral', edgecolor='black')
    ax.hist(filtered_angles_deg, bins=bins, alpha=0.5,
            label='Filtered', color='seagreen', edgecolor='black')
    ax.axvline(np.mean(raw_angles_deg), color='red',
               linestyle='--', linewidth=2, label='Raw Mean')
    ax.axvline(np.mean(filtered_angles_deg), color='green',
               linestyle='--', linewidth=2, label='Filtered Mean')
    ax.set_xlabel('Angle (degrees)', fontsize=12)
    ax.set_ylabel('Frequency', fontsize=12)
    ax.set_title('Angle Distribution', fontsize=13, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')

    # Bottom-right: Statistics summary
    ax = axes[1, 1]
    ax.axis('off')

    stats_text = f"""
TEMPORAL FILTERING RESULTS
{'='*40}

Total Frames:        {num_frames}
Outliers Detected:   {num_outliers} ({num_outliers/num_frames*100:.0f}%)

RAW ANGLES (all frames):
  Mean:  {np.mean(raw_angles_deg):.1f}°
  Std:   {raw_std:.2f}°
  Range: [{min(raw_angles_deg):.1f}°, {max(raw_angles_deg):.1f}°]

FILTERED ANGLES:
  Mean:  {np.mean(filtered_angles_deg):.1f}°
  Std:   {filtered_std:.2f}°
  Range: [{min(filtered_angles_deg):.1f}°, {max(filtered_angles_deg):.1f}°]

IMPROVEMENT:
  Variance reduction: {(1 - filtered_std/raw_valid_std)*100:.1f}%
  (comparing filtered vs valid raw frames)
"""

    ax.text(0.05, 0.95, stats_text, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.savefig('temporal_filtering_results.png', dpi=150, bbox_inches='tight')
    print(f"\n✅ Results saved to: temporal_filtering_results.png")

    # Visualize last frame with both angles
    last_result = results[-1]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Quality map
    im_q = axes[0].imshow(last_result['q_img'], cmap='hot', vmin=0, vmax=1)
    axes[0].plot(last_result['position'][0], last_result['position'][1],
                 'gx', markersize=15, markeredgewidth=3)
    axes[0].set_title(
        f"Quality Map\nBest: Q={last_result['quality']:.3f}", fontsize=12)
    plt.colorbar(im_q, ax=axes[0])

    # Raw angle grasp
    axes[1].imshow(last_result['color_rgb'])
    u, v = last_result['position']
    raw_angle = raw_angles[-1]
    arrow_len = 40
    axes[1].arrow(u, v, arrow_len*np.cos(raw_angle), arrow_len*np.sin(raw_angle),
                  color='red', width=3, head_width=12, label='Raw')
    axes[1].plot(u, v, 'ro', markersize=10)
    status_text = " (OUTLIER)" if outlier_mask[-1] else ""
    axes[1].set_title(f"Raw Angle{status_text}\n{np.degrees(raw_angle):.1f}°",
                      color='red', fontweight='bold', fontsize=12)
    axes[1].axis('off')

    # Filtered angle grasp
    axes[2].imshow(last_result['color_rgb'])
    filtered_angle = filtered_angles[-1]
    axes[2].arrow(u, v, arrow_len*np.cos(filtered_angle), arrow_len*np.sin(filtered_angle),
                  color='green', width=3, head_width=12, label='Filtered')
    axes[2].plot(u, v, 'go', markersize=10)
    axes[2].set_title(f"Filtered Angle\n{np.degrees(filtered_angle):.1f}°",
                      color='green', fontweight='bold', fontsize=12)
    axes[2].axis('off')

    plt.tight_layout()
    plt.savefig('temporal_filtering_comparison.png',
                dpi=150, bbox_inches='tight')
    print(f"✅ Comparison saved to: temporal_filtering_comparison.png")
    plt.show()


if __name__ == "__main__":
    main()
