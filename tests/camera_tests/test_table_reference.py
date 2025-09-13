"""
Unit tests for table reference module.

Tests the table reference depth model functionality including:
- Table reference initialization
- Exponential moving average updates
- Height computation for single pixels and ROI regions
- Debug visualization
"""
from camera_management.table_reference import TableReferenceModule, TableReferenceConfig
import unittest
import numpy as np
import sys
import os
from pathlib import Path

# Add src directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))


class TestTableReferenceModule(unittest.TestCase):
    """Test cases for TableReferenceModule."""

    def setUp(self):
        """Set up test fixtures."""
        self.config = TableReferenceConfig(
            update_alpha=0.1,  # Higher alpha for faster convergence in tests
            tolerance=0.005,   # Smaller tolerance for more precise tests
            initialization_frames=3,
            min_valid_depth=0.1,
            max_valid_depth=2.0
        )
        self.module = TableReferenceModule(self.config)

    def tearDown(self):
        """Clean up after tests."""
        self.module.reset()

    def test_initialization_state(self):
        """Test initial state of the module."""
        self.assertFalse(self.module.is_initialized())
        self.assertEqual(self.module.get_initialization_progress(), 0.0)
        self.assertEqual(len(self.module._initialization_frames), 0)

    def test_initialization_with_synthetic_frames(self):
        """Test table reference initialization with synthetic depth frames."""
        # Create synthetic depth frames with table at 1.0m and some noise
        height, width = 480, 640
        table_depth = 1.0

        for i in range(self.config.initialization_frames):
            # Create frame with table surface + noise
            frame = np.full((height, width), table_depth, dtype=np.float32)
            noise = np.random.normal(0, 0.001, (height, width))  # Small noise
            frame += noise

            # Add some invalid pixels
            frame[0:10, 0:10] = 0.0  # Invalid depths
            frame[10:20, 10:20] = 3.0  # Too far

            self.module.update_table_reference(frame)

            if i < self.config.initialization_frames - 1:
                self.assertFalse(self.module.is_initialized())
                self.assertLess(self.module.get_initialization_progress(), 1.0)

        # After initialization frames, should be initialized
        self.assertTrue(self.module.is_initialized())
        self.assertEqual(self.module.get_initialization_progress(), 1.0)

        # Check that table reference is approximately correct
        stats = self.module.get_stats()
        self.assertAlmostEqual(
            stats['table_depth_mean'], table_depth, places=2)

    def test_height_computation_single_pixel(self):
        """Test height computation for single pixel."""
        # Initialize with table reference
        self._initialize_with_table_depth(1.0)

        # Create depth frame with object at 0.8m (0.2m above table)
        depth_frame = np.full((480, 640), 0.8, dtype=np.float32)
        expected_height = 0.2

        # Test height computation at center pixel
        grasp_uv = (320, 240)
        computed_height = self.module.get_height_above_table(
            depth_frame, grasp_uv)

        self.assertAlmostEqual(computed_height, expected_height, places=3)

    def test_height_computation_roi(self):
        """Test height computation with ROI mask."""
        # Initialize with table reference
        self._initialize_with_table_depth(1.0)

        # Create depth frame with object at 0.7m (0.3m above table)
        depth_frame = np.full((480, 640), 0.7, dtype=np.float32)
        expected_height = 0.3

        # Create ROI mask (center region)
        roi_mask = np.zeros((480, 640), dtype=np.uint8)
        roi_mask[200:280, 280:360] = 255  # 80x80 region

        grasp_uv = (320, 240)
        computed_height = self.module.get_height_above_table(
            depth_frame, grasp_uv, roi_mask
        )

        self.assertAlmostEqual(computed_height, expected_height, places=3)

    def test_height_computation_edge_cases(self):
        """Test height computation edge cases."""
        # Initialize with table reference
        self._initialize_with_table_depth(1.0)

        # Test with invalid grasp coordinates
        depth_frame = np.full((480, 640), 0.8, dtype=np.float32)

        # Out of bounds coordinates
        height = self.module.get_height_above_table(depth_frame, (-1, 0))
        self.assertEqual(height, 0.0)

        height = self.module.get_height_above_table(depth_frame, (1000, 1000))
        self.assertEqual(height, 0.0)

        # Test with invalid depths
        invalid_frame = np.full(
            (480, 640), 0.0, dtype=np.float32)  # Invalid depth
        height = self.module.get_height_above_table(invalid_frame, (320, 240))
        self.assertEqual(height, 0.0)

    def test_exponential_moving_average_update(self):
        """Test exponential moving average update mechanism."""
        # Initialize with table reference
        self._initialize_with_table_depth(1.0)

        # Create frame with slightly different table depth
        new_table_depth = 0.98  # 2cm lower
        depth_frame = np.full((480, 640), new_table_depth, dtype=np.float32)

        # Update table reference multiple times
        for _ in range(10):
            self.module.update_table_reference(depth_frame)

        # Check that table reference has moved towards new depth
        stats = self.module.get_stats()
        # Allow for floating point precision
        self.assertLess(stats['table_depth_mean'], 1.001)
        self.assertGreater(stats['table_depth_mean'], new_table_depth)

    def test_grasp_roi_mask_creation(self):
        """Test grasp ROI mask creation."""
        # Initialize module
        self._initialize_with_table_depth(1.0)

        # Test ROI mask creation
        grasp_center = (320, 240)
        grasp_width = 50.0  # pixels
        grasp_angle = 0.0
        image_shape = (480, 640)

        roi_mask = self.module.create_grasp_roi_mask(
            grasp_center, grasp_width, grasp_angle, image_shape
        )

        # Check mask properties
        self.assertEqual(roi_mask.shape, image_shape)
        self.assertEqual(roi_mask.dtype, np.uint8)

        # Check that center region is marked
        self.assertGreater(roi_mask[240, 320], 0)

        # Check that mask has reasonable size
        mask_area = np.sum(roi_mask > 0)
        self.assertGreater(mask_area, 0)
        self.assertLess(mask_area, image_shape[0] * image_shape[1])

    def test_debug_visualization(self):
        """Test debug visualization functionality."""
        # Initialize with table reference
        self._initialize_with_table_depth(1.0)

        # Enable debug mode
        self.module.set_debug_mode(True)

        # Create depth frame with object
        depth_frame = np.full((480, 640), 0.8, dtype=np.float32)

        # Generate debug visualization
        debug_img = self.module.get_debug_visualization(depth_frame)

        # Check that debug image is generated
        self.assertIsNotNone(debug_img)
        self.assertEqual(debug_img.shape, (480, 640, 3))  # Color image

        # Test with debug mode disabled
        self.module.set_debug_mode(False)
        debug_img = self.module.get_debug_visualization(depth_frame)
        self.assertIsNone(debug_img)

    def test_reset_functionality(self):
        """Test module reset functionality."""
        # Initialize module
        self._initialize_with_table_depth(1.0)
        self.assertTrue(self.module.is_initialized())

        # Reset module
        self.module.reset()

        # Check that module is back to initial state
        self.assertFalse(self.module.is_initialized())
        self.assertEqual(self.module.get_initialization_progress(), 0.0)
        self.assertEqual(len(self.module._initialization_frames), 0)
        self.assertIsNone(self.module._table_depth_ref)

    def test_stats_reporting(self):
        """Test statistics reporting functionality."""
        # Test stats before initialization
        stats = self.module.get_stats()
        self.assertFalse(stats['initialized'])
        self.assertEqual(stats['frame_count'], 0)
        self.assertEqual(stats['initialization_progress'], 0.0)

        # Initialize and test stats after initialization
        self._initialize_with_table_depth(1.0)
        stats = self.module.get_stats()

        self.assertTrue(stats['initialized'])
        self.assertEqual(stats['frame_count'],
                         self.config.initialization_frames)
        self.assertEqual(stats['initialization_progress'], 1.0)
        self.assertIn('table_depth_mean', stats)
        self.assertIn('table_depth_std', stats)
        self.assertIn('valid_pixel_ratio', stats)

    def test_noise_robustness(self):
        """Test robustness to noisy depth measurements."""
        # Initialize with table reference
        self._initialize_with_table_depth(1.0)

        # Create noisy depth frame
        height, width = 480, 640
        table_depth = 1.0
        object_depth = 0.8
        expected_height = 0.2

        # Create frame with noise
        frame = np.full((height, width), object_depth, dtype=np.float32)
        noise = np.random.normal(0, 0.01, (height, width))  # 1cm noise
        frame += noise

        # Test height computation with noise
        grasp_uv = (320, 240)
        computed_height = self.module.get_height_above_table(frame, grasp_uv)

        # Should be close to expected height despite noise
        self.assertAlmostEqual(computed_height, expected_height, places=1)

    def _initialize_with_table_depth(self, table_depth: float):
        """Helper method to initialize module with given table depth."""
        height, width = 480, 640

        for _ in range(self.config.initialization_frames):
            frame = np.full((height, width), table_depth, dtype=np.float32)
            # Add small amount of noise
            noise = np.random.normal(0, 0.001, (height, width))
            frame += noise
            self.module.update_table_reference(frame)


class TestTableReferenceConfig(unittest.TestCase):
    """Test cases for TableReferenceConfig."""

    def test_default_config(self):
        """Test default configuration values."""
        config = TableReferenceConfig()

        self.assertEqual(config.update_alpha, 0.05)
        self.assertEqual(config.tolerance, 0.01)
        self.assertEqual(config.initialization_frames, 5)
        self.assertEqual(config.min_valid_depth, 0.1)
        self.assertEqual(config.max_valid_depth, 3.0)

    def test_custom_config(self):
        """Test custom configuration values."""
        config = TableReferenceConfig(
            update_alpha=0.1,
            tolerance=0.005,
            initialization_frames=3,
            min_valid_depth=0.2,
            max_valid_depth=2.0
        )

        self.assertEqual(config.update_alpha, 0.1)
        self.assertEqual(config.tolerance, 0.005)
        self.assertEqual(config.initialization_frames, 3)
        self.assertEqual(config.min_valid_depth, 0.2)
        self.assertEqual(config.max_valid_depth, 2.0)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
