#!/usr/bin/env python3
"""
Temporary Skeleton Data Capture Script

This script captures skeleton data from the ZED camera, saves it to files,
and analyzes the data to find stable joint locations by filtering out
messy data from the start and end of recordings.

Usage:
    python skeleton_data_capture.py [--duration SECONDS] [--output-dir DIR]
    
Example:
    python skeleton_data_capture.py --duration 30 --output-dir ./skeleton_data
"""

# Add src directory to path to import our modules
import logging
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import numpy as np
import argparse
import json
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))


# Import after path setup to prevent linting from moving it
from hand_detection.zed_joint_receiver import ZEDJointReceiver, FrameData, SkeletonData, JointData  # noqa: E402


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SkeletonDataCapture:
    """Captures and analyzes skeleton data from ZED camera."""

    def __init__(self, output_dir: str = "./skeleton_data"):
        self.output_dir = output_dir
        self.captured_data: List[FrameData] = []
        self.receiver: Optional[ZEDJointReceiver] = None

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

    def start_capture(self, duration: int = 30) -> bool:
        """
        Start capturing skeleton data for specified duration.
        First waits for Unity connection, then captures data for the specified duration.

        Args:
            duration: Duration in seconds to capture data AFTER connection is established

        Returns:
            True if capture was successful, False otherwise
        """
        logger.info(
            f"Starting skeleton data capture for {duration} seconds...")

        # Initialize receiver
        self.receiver = ZEDJointReceiver(
            host='0.0.0.0',  # Accept connections from any network interface
            port=5005,
            callback=self._on_frame_received,
            smoothing_factor=0.1  # Less smoothing for raw data analysis
        )

        try:
            # Start receiver
            self.receiver.start()

            # Wait for connection (indefinitely until user cancels or connects)
            logger.info("Waiting for Unity connection...")
            logger.info("Press Ctrl+C to cancel")

            while not self.receiver.is_connected():
                time.sleep(0.1)

            logger.info("Connected! Starting data capture...")

            # Clear any data captured during connection wait
            self.captured_data.clear()

            # Wait 20 seconds for user to get into position
            logger.info("Waiting 20 seconds for you to get into position...")
            time.sleep(20)
            logger.info("Starting data capture now!")

            # Capture data for specified duration AFTER wait
            capture_start = time.time()
            while (time.time() - capture_start) < duration:
                time.sleep(0.1)

                # Print progress every 5 seconds
                elapsed = time.time() - capture_start
                if int(elapsed) % 5 == 0 and elapsed > 0:
                    logger.info(
                        f"Captured {len(self.captured_data)} frames ({elapsed:.1f}s elapsed)")

            logger.info(
                f"Capture complete! Collected {len(self.captured_data)} frames")
            return True

        except Exception as e:
            logger.error(f"Error during capture: {e}")
            return False
        finally:
            if self.receiver:
                self.receiver.stop()

    def _on_frame_received(self, frame_data: FrameData):
        """Callback for when a new frame is received."""
        self.captured_data.append(frame_data)

        # Print frame info every 10 frames
        if len(self.captured_data) % 10 == 0:
            skeleton_count = len(frame_data.skeletons)
            logger.info(
                f"Frame {frame_data.frame}: {skeleton_count} skeleton(s) detected")

    def save_data(self, filename: Optional[str] = None) -> str:
        """
        Save captured data to JSON file.

        Args:
            filename: Optional custom filename. If None, generates timestamp-based name.

        Returns:
            Path to saved file
        """
        if not self.captured_data:
            raise ValueError("No data to save")

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"skeleton_capture_{timestamp}.json"

        filepath = os.path.join(self.output_dir, filename)

        # Convert to serializable format
        data_to_save = {
            'capture_info': {
                'total_frames': len(self.captured_data),
                'capture_timestamp': datetime.now().isoformat(),
                'first_frame': self.captured_data[0].frame if self.captured_data else None,
                'last_frame': self.captured_data[-1].frame if self.captured_data else None
            },
            'frames': [frame.to_dict() for frame in self.captured_data]
        }

        with open(filepath, 'w') as f:
            json.dump(data_to_save, f, indent=2)

        logger.info(f"Data saved to: {filepath}")
        return filepath

    def analyze_joint_locations(self, filter_percentage: float = 0.1) -> Dict[str, Dict[str, float]]:
        """
        Analyze captured data to find stable joint locations with comprehensive jitter and tracking analysis.

        Args:
            filter_percentage: Percentage of data to filter from start and end (0.1 = 10%)

        Returns:
            Dictionary mapping skeleton_id -> joint_name -> analysis results
        """
        if not self.captured_data:
            raise ValueError("No data to analyze")

        logger.info(
            "Analyzing joint locations with jitter and tracking stability...")

        # Filter out messy data from start only (since we waited 20 seconds)
        total_frames = len(self.captured_data)
        filter_frames = int(total_frames * filter_percentage)

        if filter_frames > 0:
            # Only filter from start, keep all data to the end
            filtered_data = self.captured_data[filter_frames:]
            logger.info(
                f"Filtered out {filter_frames} frames from start ({filter_percentage*100:.1f}%)")
        else:
            filtered_data = self.captured_data

        # Track joint presence across all frames (including filtered data for tracking analysis)
        # {skeleton_id: {joint_name: [frame_indices_where_present]}}
        joint_tracking = {}
        joint_positions = {}  # {skeleton_id: {joint_name: [(x,y,z), ...]}}

        # First pass: collect all joint data and track presence
        for frame_idx, frame in enumerate(self.captured_data):
            for skeleton in frame.skeletons:
                skeleton_id = skeleton.skeleton_id

                if skeleton_id not in joint_tracking:
                    joint_tracking[skeleton_id] = {}
                    joint_positions[skeleton_id] = {}

                for joint in skeleton.joints:
                    joint_name = joint.joint_name
                    position = (joint.x, joint.y, joint.z)

                    # Track presence
                    if joint_name not in joint_tracking[skeleton_id]:
                        joint_tracking[skeleton_id][joint_name] = []
                    joint_tracking[skeleton_id][joint_name].append(frame_idx)

                    # Collect positions (only from filtered data)
                    if frame_idx >= filter_frames and frame_idx < (total_frames - filter_frames):
                        if joint_name not in joint_positions[skeleton_id]:
                            joint_positions[skeleton_id][joint_name] = []
                        joint_positions[skeleton_id][joint_name].append(
                            position)

        # Calculate comprehensive analysis for each joint
        stable_locations = {}

        for skeleton_id, joints in joint_positions.items():
            stable_locations[skeleton_id] = {}

            for joint_name, positions in joints.items():
                if len(positions) < 3:  # Need at least 3 samples for meaningful analysis
                    logger.warning(
                        f"Not enough samples for {skeleton_id}:{joint_name} ({len(positions)} samples)")
                    continue

                # Convert to numpy array for easier calculation
                positions_array = np.array(positions)

                # === BASIC STATISTICS ===
                # Calculate median for each axis (robust to outliers)
                median_x = np.median(positions_array[:, 0])
                median_y = np.median(positions_array[:, 1])
                median_z = np.median(positions_array[:, 2])

                # Calculate mean for comparison
                mean_x = np.mean(positions_array[:, 0])
                mean_y = np.mean(positions_array[:, 1])
                mean_z = np.mean(positions_array[:, 2])

                # === JITTER ANALYSIS ===
                # Standard deviation (overall spread)
                std_x = np.std(positions_array[:, 0])
                std_y = np.std(positions_array[:, 1])
                std_z = np.std(positions_array[:, 2])

                # Root Mean Square (RMS) - measures average magnitude of deviation
                rms_x = np.sqrt(np.mean(positions_array[:, 0]**2))
                rms_y = np.sqrt(np.mean(positions_array[:, 1]**2))
                rms_z = np.sqrt(np.mean(positions_array[:, 2]**2))

                # Maximum deviation from median
                max_dev_x = np.max(np.abs(positions_array[:, 0] - median_x))
                max_dev_y = np.max(np.abs(positions_array[:, 1] - median_y))
                max_dev_z = np.max(np.abs(positions_array[:, 2] - median_z))

                # Interquartile Range (IQR) - robust measure of spread
                q75_x, q25_x = np.percentile(positions_array[:, 0], [75, 25])
                q75_y, q25_y = np.percentile(positions_array[:, 1], [75, 25])
                q75_z, q25_z = np.percentile(positions_array[:, 2], [75, 25])
                iqr_x = q75_x - q25_x
                iqr_y = q75_y - q25_y
                iqr_z = q75_z - q25_z

                # === TRACKING STABILITY ANALYSIS ===
                # Calculate tracking presence rate
                total_frames_for_joint = len(
                    joint_tracking[skeleton_id][joint_name])
                tracking_presence_rate = total_frames_for_joint / total_frames

                # Calculate gaps in tracking (consecutive missing frames)
                present_frames = set(joint_tracking[skeleton_id][joint_name])
                gaps = []
                current_gap = 0
                for frame_idx in range(total_frames):
                    if frame_idx not in present_frames:
                        current_gap += 1
                    else:
                        if current_gap > 0:
                            gaps.append(current_gap)
                            current_gap = 0
                if current_gap > 0:  # Gap at the end
                    gaps.append(current_gap)

                # Gap statistics
                max_gap = max(gaps) if gaps else 0
                avg_gap = np.mean(gaps) if gaps else 0
                gap_count = len(gaps)

                # === VELOCITY ANALYSIS (for jitter assessment) ===
                if len(positions) > 1:
                    # Calculate frame-to-frame velocity
                    velocities = []
                    for i in range(1, len(positions)):
                        prev_pos = positions[i-1]
                        curr_pos = positions[i]
                        # Euclidean distance between consecutive positions
                        velocity = np.sqrt(
                            sum((a - b)**2 for a, b in zip(curr_pos, prev_pos)))
                        velocities.append(velocity)

                    velocity_array = np.array(velocities)
                    avg_velocity = np.mean(velocity_array)
                    max_velocity = np.max(velocity_array)
                    velocity_std = np.std(velocity_array)
                else:
                    avg_velocity = 0.0
                    max_velocity = 0.0
                    velocity_std = 0.0

                # === OVERALL JITTER SCORE ===
                # Combined metric: higher values = more jittery
                # Weighted combination of different jitter measures
                jitter_score = (
                    0.3 * (std_x + std_y + std_z) +  # Standard deviation
                    # Max deviation
                    0.2 * (max_dev_x + max_dev_y + max_dev_z) +
                    0.2 * (iqr_x + iqr_y + iqr_z) +  # IQR
                    0.3 * avg_velocity  # Average velocity
                )

                # === STABILITY RATING ===
                if jitter_score < 0.01:
                    stability_rating = "Excellent"
                elif jitter_score < 0.05:
                    stability_rating = "Good"
                elif jitter_score < 0.1:
                    stability_rating = "Fair"
                elif jitter_score < 0.2:
                    stability_rating = "Poor"
                else:
                    stability_rating = "Very Poor"

                stable_locations[skeleton_id][joint_name] = {
                    # Basic position data
                    'x': float(median_x),
                    'y': float(median_y),
                    'z': float(median_z),
                    'mean_x': float(mean_x),
                    'mean_y': float(mean_y),
                    'mean_z': float(mean_z),

                    # Jitter measurements
                    'std_x': float(std_x),
                    'std_y': float(std_y),
                    'std_z': float(std_z),
                    'rms_x': float(rms_x),
                    'rms_y': float(rms_y),
                    'rms_z': float(rms_z),
                    'max_dev_x': float(max_dev_x),
                    'max_dev_y': float(max_dev_y),
                    'max_dev_z': float(max_dev_z),
                    'iqr_x': float(iqr_x),
                    'iqr_y': float(iqr_y),
                    'iqr_z': float(iqr_z),

                    # Velocity analysis
                    'avg_velocity': float(avg_velocity),
                    'max_velocity': float(max_velocity),
                    'velocity_std': float(velocity_std),

                    # Tracking stability
                    'tracking_presence_rate': float(tracking_presence_rate),
                    'max_gap_frames': int(max_gap),
                    'avg_gap_frames': float(avg_gap),
                    'gap_count': int(gap_count),

                    # Overall assessment
                    'jitter_score': float(jitter_score),
                    'stability_rating': stability_rating,
                    'sample_count': len(positions),
                    'total_frames_analyzed': total_frames
                }

        logger.info(
            f"Analysis complete! Found stable locations for {len(stable_locations)} skeleton(s)")
        return stable_locations

    def save_analysis(self, stable_locations: Dict[str, Dict[str, float]], filename: Optional[str] = None) -> str:
        """
        Save analysis results to JSON file.

        Args:
            stable_locations: Results from analyze_joint_locations()
            filename: Optional custom filename

        Returns:
            Path to saved file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"skeleton_analysis_{timestamp}.json"

        filepath = os.path.join(self.output_dir, filename)

        with open(filepath, 'w') as f:
            json.dump(stable_locations, f, indent=2)

        logger.info(f"Analysis saved to: {filepath}")
        return filepath

    def print_analysis_summary(self, stable_locations: Dict[str, Dict[str, float]]):
        """Print a comprehensive summary of the analysis results."""
        print("\n" + "="*100)
        print("SKELETON JOINT LOCATION ANALYSIS SUMMARY")
        print("="*100)

        for skeleton_id, joints in stable_locations.items():
            print(f"\nSkeleton ID: {skeleton_id}")
            print("-" * 98)

            # Sort joints by name for consistent output
            for joint_name in sorted(joints.keys()):
                joint_data = joints[joint_name]

                print(f"\n  Joint: {joint_name}")
                print(
                    f"    Position (median): ({joint_data['x']:7.3f}, {joint_data['y']:7.3f}, {joint_data['z']:7.3f})")
                print(
                    f"    Position (mean):   ({joint_data['mean_x']:7.3f}, {joint_data['mean_y']:7.3f}, {joint_data['mean_z']:7.3f})")

                print(f"    Jitter Analysis:")
                print(
                    f"      Standard Dev:    ({joint_data['std_x']:6.4f}, {joint_data['std_y']:6.4f}, {joint_data['std_z']:6.4f})")
                print(
                    f"      Max Deviation:   ({joint_data['max_dev_x']:6.4f}, {joint_data['max_dev_y']:6.4f}, {joint_data['max_dev_z']:6.4f})")
                print(
                    f"      IQR:             ({joint_data['iqr_x']:6.4f}, {joint_data['iqr_y']:6.4f}, {joint_data['iqr_z']:6.4f})")
                print(
                    f"      RMS:             ({joint_data['rms_x']:6.4f}, {joint_data['rms_y']:6.4f}, {joint_data['rms_z']:6.4f})")

                print(f"    Velocity Analysis:")
                print(
                    f"      Avg Velocity:    {joint_data['avg_velocity']:8.4f} units/frame")
                print(
                    f"      Max Velocity:    {joint_data['max_velocity']:8.4f} units/frame")
                print(
                    f"      Velocity Std:    {joint_data['velocity_std']:8.4f} units/frame")

                print(f"    Tracking Stability:")
                print(
                    f"      Presence Rate:   {joint_data['tracking_presence_rate']:6.1%}")
                print(
                    f"      Max Gap:         {joint_data['max_gap_frames']:3d} frames")
                print(
                    f"      Avg Gap:         {joint_data['avg_gap_frames']:6.1f} frames")
                print(
                    f"      Gap Count:       {joint_data['gap_count']:3d} gaps")

                print(f"    Overall Assessment:")
                print(
                    f"      Jitter Score:    {joint_data['jitter_score']:8.4f}")
                print(
                    f"      Stability:       {joint_data['stability_rating']:12s}")
                print(
                    f"      Samples:         {joint_data['sample_count']:3d}/{joint_data['total_frames_analyzed']:3d} frames")

        print("\n" + "="*100)
        print("MEASUREMENT EXPLANATIONS:")
        print("="*100)
        print("• Standard Dev:     How spread out positions are around the median")
        print("• Max Deviation:    Largest single deviation from median position")
        print("• IQR:              Interquartile Range - robust measure of spread (75th - 25th percentile)")
        print("• RMS:              Root Mean Square - average magnitude of position values")
        print("• Avg Velocity:     Average frame-to-frame movement distance")
        print("• Max Velocity:     Largest single frame-to-frame movement")
        print("• Presence Rate:    Percentage of frames where joint was detected")
        print("• Max Gap:          Longest consecutive sequence of missing frames")
        print("• Avg Gap:          Average length of missing frame sequences")
        print("• Gap Count:        Number of separate missing frame sequences")
        print("• Jitter Score:     Combined metric (lower = more stable)")
        print("• Stability Rating: Overall assessment based on jitter score")
        print("="*100)


def main():
    """Main function to run the skeleton data capture and analysis."""
    parser = argparse.ArgumentParser(
        description="Capture and analyze skeleton data from ZED camera")
    parser.add_argument("--duration", type=int, default=30,
                        help="Duration to capture data (seconds)")
    parser.add_argument("--output-dir", type=str, default="./skeleton_data",
                        help="Output directory for saved files")
    parser.add_argument("--filter-percentage", type=float, default=0.1,
                        help="Percentage of data to filter from start/end (0.0-0.5)")

    args = parser.parse_args()

    # Validate arguments
    if args.duration <= 0:
        logger.error("Duration must be positive")
        return 1

    if not 0 <= args.filter_percentage <= 0.5:
        logger.error("Filter percentage must be between 0.0 and 0.5")
        return 1

    try:
        # Create capture instance
        capture = SkeletonDataCapture(args.output_dir)

        # Capture data
        if not capture.start_capture(args.duration):
            logger.error("Failed to capture data")
            return 1

        if not capture.captured_data:
            logger.error("No data was captured")
            return 1

        # Save raw data
        data_file = capture.save_data()

        # Analyze data
        stable_locations = capture.analyze_joint_locations(
            args.filter_percentage)

        # Save analysis
        analysis_file = capture.save_analysis(stable_locations)

        # Print summary
        capture.print_analysis_summary(stable_locations)

        print(f"\nFiles saved:")
        print(f"  Raw data: {data_file}")
        print(f"  Analysis: {analysis_file}")

        return 0

    except KeyboardInterrupt:
        logger.info("Capture interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
