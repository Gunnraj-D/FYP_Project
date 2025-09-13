#!/usr/bin/env python3
"""
Test runner for camera tests.

This script runs all camera-related tests including table reference functionality.
"""
from test_table_reference import TestTableReferenceModule, TestTableReferenceConfig
import sys
import unittest
from pathlib import Path

# Add src directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))


def run_camera_tests():
    """Run all camera tests."""
    # Create test suite
    test_suite = unittest.TestSuite()

    # Add table reference tests
    loader = unittest.TestLoader()
    test_suite.addTest(loader.loadTestsFromTestCase(TestTableReferenceModule))
    test_suite.addTest(loader.loadTestsFromTestCase(TestTableReferenceConfig))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)

    # Return exit code based on test results
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    exit_code = run_camera_tests()
    sys.exit(exit_code)
