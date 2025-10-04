#!/usr/bin/env python3
"""
Simple runner for semi-automated hand-eye calibration.
"""
from calibration.semi_automated_calibration import main
import sys
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent))


if __name__ == "__main__":
    main()
