#!/usr/bin/env python3
"""
Simple runner for manual hand-eye calibration.

Usage:
    python run_manual_calibration.py [--mode real|mock]
"""
from calibration.manual_calibration import main
import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent))


if __name__ == "__main__":
    main()
