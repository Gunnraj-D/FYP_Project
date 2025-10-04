"""
Test script to verify collision-aware solver integration with calibration system.
"""
import logging
from calibration.calibration_config import CalibrationConfig
from calibration.hand_eye_calibrator import HandEyeCalibrator
import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent))


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_calibration_integration():
    """Test that calibration system uses collision-aware solver."""

    logger.info("Testing calibration system integration...")

    try:
        # Create calibration config
        config = CalibrationConfig()

        # Create calibrator
        calibrator = HandEyeCalibrator(config)

        # Check if it uses collision-aware solver
        if hasattr(calibrator, 'kinematics_solver'):
            solver_type = type(calibrator.kinematics_solver).__name__
            logger.info(f"Calibration system uses: {solver_type}")

            if "CollisionAware" in solver_type:
                logger.info(
                    "✅ SUCCESS: Calibration system uses collision-aware solver!")
                return True
            else:
                logger.warning(
                    f"❌ WARNING: Calibration system uses {solver_type}, not collision-aware")
                return False
        else:
            logger.error("❌ ERROR: No kinematics solver found in calibrator")
            return False

    except Exception as e:
        logger.error(f"❌ ERROR: Failed to test calibration integration: {e}")
        return False


if __name__ == "__main__":
    success = test_calibration_integration()
    if success:
        print("\n🎉 Calibration system is integrated with collision-aware solver!")
    else:
        print("\n⚠️  Calibration system integration needs attention.")
