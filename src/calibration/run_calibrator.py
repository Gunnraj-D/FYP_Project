"""
Main script to run hand-eye calibration.
"""
from .calibration_config import CalibrationConfig
from .hand_eye_calibrator import HandEyeCalibrator
import sys
import os
import logging
import argparse
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent.parent))


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """Main function to run hand-eye calibration."""
    parser = argparse.ArgumentParser(
        description='Hand-Eye Calibration for KUKA iiwa14')
    parser.add_argument('--mode', choices=['real', 'mock'], default='mock',
                        help='Robot communication mode (real or mock)')
    parser.add_argument('--poses', type=int, default=15,
                        help='Number of poses to collect for calibration')
    parser.add_argument('--manual', action='store_true',
                        help='Enable manual capture mode (SPACE to capture)')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to custom configuration file')
    parser.add_argument('--load-data', type=str, default=None,
                        help='Load existing calibration data from file')

    args = parser.parse_args()

    print("=" * 60)
    print("HAND-EYE CALIBRATION SYSTEM")
    print("=" * 60)
    print(f"Mode: {args.mode}")
    print(f"Poses: {args.poses}")
    print("=" * 60)

    try:
        # Create calibration configuration
        config = CalibrationConfig()
        if args.config:
            # Load custom config if provided
            # TODO: Implement config loading
            pass

        # Create calibrator
        calibrator = HandEyeCalibrator(config)

        # Initialize system
        print("Initializing calibration system...")
        if not calibrator.initialize(opc_mode=args.mode):
            print("❌ Failed to initialize calibration system")
            return 1

        print("✅ Calibration system initialized successfully")

        # Load existing data if specified
        if args.load_data:
            print(
                f"Loading existing calibration data from {args.load_data}...")
            if calibrator.load_calibration_data(args.load_data):
                print("✅ Calibration data loaded successfully")
            else:
                print("❌ Failed to load calibration data")
                return 1

        # Run calibration
        print(f"Starting calibration with {args.poses} poses...")
        print("Press Ctrl+C to cancel at any time")

        success = calibrator.run_calibration(
            num_poses=args.poses, manual=args.manual)

        if success:
            print("✅ Hand-eye calibration completed successfully!")

            # Display results
            H_cam2tcp = calibrator.get_hand_eye_matrix()
            if H_cam2tcp is not None:
                print("\nHand-Eye Transformation Matrix (Camera to TCP):")
                print(H_cam2tcp)
                print(f"\nMatrix saved to: {config.hand_eye_matrix_file}")
                print(f"Report saved to: {config.calibration_report_file}")
        else:
            print("❌ Hand-eye calibration failed!")
            return 1

        return 0

    except KeyboardInterrupt:
        print("\n👋 Calibration interrupted by user")
        return 0
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        logger.exception("Unexpected error during calibration")
        return 1
    finally:
        # Cleanup
        try:
            calibrator.cleanup()
        except:
            pass


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
