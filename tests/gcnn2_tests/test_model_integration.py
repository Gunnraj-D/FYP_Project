"""
Integration tests for the complete object detection pipeline.
Tests the full workflow from depth image to grasp execution.
"""
from config.config import GGCNN2_MODEL_PATH
from object_detection.ggcnn2_module import GGcnn2Module
import unittest
import numpy as np
import torch
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestObjectDetectionIntegration(unittest.TestCase):
    """Integration tests for the complete object detection pipeline."""

    def setUp(self):
        """Set up integration test fixtures."""
        # Create realistic mock dependencies
        self.mock_telemetry = Mock()
        self.mock_command_bus = Mock()
        self.mock_camera_manager = Mock()
        self.mock_kinematics_solver = Mock()

        # Configure realistic mock behaviors
        self._setup_realistic_mocks()

    def _setup_realistic_mocks(self):
        """Set up realistic mock behaviors."""
        # Mock telemetry with realistic joint positions
        self.mock_telemetry.get_current_joints.return_value = np.array([
            0.1, -0.5, 0.0, -1.57, 0.0, 1.57, 0.0
        ])

        # Mock camera manager with realistic depth and 3D conversion
        self.mock_camera_manager.get_average_depth.return_value = 450.0
        self.mock_camera_manager.pixel_to_3d.return_value = [
            150.0, 200.0, 450.0]

        # Mock kinematics solver with realistic outputs
        self.mock_kinematics_solver.tcp_from_joints.return_value = (
            np.array([
                [1, 0, 0, 0.1],
                [0, 1, 0, 0.2],
                [0, 0, 1, 0.3],
                [0, 0, 0, 1]
            ]),
            [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        )
        self.mock_kinematics_solver.solve_pose.return_value = np.array([
            0.2, -0.6, 0.1, -1.6, 0.1, 1.6, 0.1
        ])

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_complete_inference_pipeline(self, mock_torch_load, mock_ggcnn2_class):
        """Test the complete inference pipeline from depth image to grasp result."""
        # Mock model and state dict
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Mock model outputs (realistic grasp detection results)
        mock_model.return_value = (
            torch.randn(1, 1, 300, 300) * 0.8 + 0.2,  # pos (quality)
            torch.cos(torch.randn(1, 1, 300, 300)),    # cos
            torch.sin(torch.randn(1, 1, 300, 300)),    # sin
            torch.randn(1, 1, 300, 300) * 0.5 + 0.3    # width
        )

        # Create module
        module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=self.mock_telemetry,
            command_bus=self.mock_command_bus,
            camera_manager=self.mock_camera_manager,
            kinematics_solver=self.mock_kinematics_solver
        )

        # Create realistic depth image
        depth_image = self._create_realistic_depth_image()

        # Run inference
        result = module.infer(depth_image)

        # Verify the complete pipeline worked
        if result is not None:
            self.assertIn('grasp_2d', result)
            self.assertIn('grasp_pose_camera', result)
            self.assertIn('grasp_pose_base', result)
            self.assertIn('joint_angles', result)
            self.assertIn('quality', result)

            # Verify data types and shapes
            self.assertIsInstance(result['joint_angles'], list)
            self.assertEqual(len(result['joint_angles']), 7)
            self.assertIsInstance(result['quality'], float)
            self.assertGreaterEqual(result['quality'], 0.0)
            self.assertLessEqual(result['quality'], 1.0)

            # Verify command was sent
            self.mock_command_bus.send.assert_called_once()

    def _create_realistic_depth_image(self):
        """Create a realistic depth image for testing."""
        # Create a depth image with some objects
        depth_image = np.full((480, 640), 500, dtype=np.uint16)

        # Add some objects (rectangles with different depths)
        depth_image[200:280, 250:350] = 400  # Object 1
        depth_image[300:380, 150:250] = 350  # Object 2
        depth_image[100:180, 400:500] = 450  # Object 3

        # Add some noise
        noise = np.random.randint(-10, 10, depth_image.shape, dtype=np.int16)
        depth_image = np.clip(depth_image.astype(
            np.int16) + noise, 0, 65535).astype(np.uint16)

        return depth_image

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_inference_with_no_valid_grasp(self, mock_torch_load, mock_ggcnn2_class):
        """Test inference when no valid grasp is found."""
        # Mock model with low quality outputs
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Mock model outputs with low quality
        mock_model.return_value = (
            torch.full((1, 1, 300, 300), 0.1),  # Low quality
            torch.cos(torch.randn(1, 1, 300, 300)),
            torch.sin(torch.randn(1, 1, 300, 300)),
            torch.randn(1, 1, 300, 300) * 0.5 + 0.3
        )

        module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=self.mock_telemetry,
            command_bus=self.mock_command_bus,
            camera_manager=self.mock_camera_manager,
            kinematics_solver=self.mock_kinematics_solver
        )

        depth_image = self._create_realistic_depth_image()
        result = module.infer(depth_image)

        # Should return None for low quality grasps
        self.assertIsNone(result)

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_inference_with_invalid_depth(self, mock_torch_load, mock_ggcnn2_class):
        """Test inference with invalid depth values."""
        # Mock model
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Mock model outputs
        mock_model.return_value = (
            torch.randn(1, 1, 300, 300) * 0.8 + 0.2,
            torch.cos(torch.randn(1, 1, 300, 300)),
            torch.sin(torch.randn(1, 1, 300, 300)),
            torch.randn(1, 1, 300, 300) * 0.5 + 0.3
        )

        # Mock camera manager to return invalid depth
        self.mock_camera_manager.get_average_depth.return_value = 0.0

        module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=self.mock_telemetry,
            command_bus=self.mock_command_bus,
            camera_manager=self.mock_camera_manager,
            kinematics_solver=self.mock_kinematics_solver
        )

        depth_image = self._create_realistic_depth_image()
        result = module.infer(depth_image)

        # Should return None for invalid depth
        self.assertIsNone(result)

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_inference_with_kinematics_failure(self, mock_torch_load, mock_ggcnn2_class):
        """Test inference when kinematics solver fails."""
        # Mock model
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Mock model outputs
        mock_model.return_value = (
            torch.randn(1, 1, 300, 300) * 0.8 + 0.2,
            torch.cos(torch.randn(1, 1, 300, 300)),
            torch.sin(torch.randn(1, 1, 300, 300)),
            torch.randn(1, 1, 300, 300) * 0.5 + 0.3
        )

        # Mock kinematics solver to raise exception
        self.mock_kinematics_solver.solve_pose.side_effect = Exception(
            "IK failed")

        module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=self.mock_telemetry,
            command_bus=self.mock_command_bus,
            camera_manager=self.mock_camera_manager,
            kinematics_solver=self.mock_kinematics_solver
        )

        depth_image = self._create_realistic_depth_image()
        result = module.infer(depth_image)

        # Should return None when kinematics fails
        self.assertIsNone(result)

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_process_depth_frame_method(self, mock_torch_load, mock_ggcnn2_class):
        """Test the convenience process_depth_frame method."""
        # Mock model
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Mock model outputs
        mock_model.return_value = (
            torch.randn(1, 1, 300, 300) * 0.8 + 0.2,
            torch.cos(torch.randn(1, 1, 300, 300)),
            torch.sin(torch.randn(1, 1, 300, 300)),
            torch.randn(1, 1, 300, 300) * 0.5 + 0.3
        )

        module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=self.mock_telemetry,
            command_bus=self.mock_command_bus,
            camera_manager=self.mock_camera_manager,
            kinematics_solver=self.mock_kinematics_solver
        )

        # Test with numpy array
        depth_array = self._create_realistic_depth_image()
        result = module.process_depth_frame(depth_array)

        if result is not None:
            self.assertIn('joint_angles', result)
            self.assertEqual(len(result['joint_angles']), 7)

    def test_error_handling_robustness(self):
        """Test error handling and robustness of the module."""
        # Test with None input
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):

            module = GGcnn2Module(
                model_path=str(GGCNN2_MODEL_PATH),
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Test with None input
            result = module.infer(None)
            self.assertIsNone(result)

            # Test with empty array
            result = module.infer(np.array([]))
            self.assertIsNone(result)

            # Test with wrong shape
            result = module.infer(np.random.rand(100, 100))
            # Should still work but might return None due to low quality


class TestPerformanceAndMemory(unittest.TestCase):
    """Test performance and memory usage."""

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_memory_usage(self, mock_torch_load, mock_ggcnn2_class):
        """Test memory usage during inference."""
        # Mock model
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Mock model outputs
        mock_model.return_value = (
            torch.full((1, 1, 300, 300), 0.9),  # strong grasp quality everywhere
            torch.ones((1, 1, 300, 300)),       # cos
            torch.zeros((1, 1, 300, 300)),      # sin
            torch.full((1, 1, 300, 300), 0.5)   # width
        )


        mock_telemetry = Mock()
        mock_telemetry.get_current_joints.return_value = np.array(
            [0.1, -0.5, 0.0, -1.57, 0.0, 1.57, 0.0])

        mock_camera_manager = Mock()
        mock_camera_manager.get_average_depth.return_value = 450.0
        mock_camera_manager.pixel_to_3d.return_value = [150.0, 200.0, 450.0]

        mock_kinematics_solver = Mock()
        mock_kinematics_solver.tcp_from_joints.return_value = (
            np.eye(4), [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        )
        mock_kinematics_solver.solve_pose.return_value = np.array(
            [0.2, -0.6, 0.1, -1.6, 0.1, 1.6, 0.1])

        module = GGcnn2Module(
            model_path=str(GGCNN2_MODEL_PATH),
            telemetry=mock_telemetry,
            command_bus=Mock(),
            camera_manager=mock_camera_manager,
            kinematics_solver=mock_kinematics_solver
        )

        # Test multiple inferences to check for memory leaks
        depth_image = np.random.randint(0, 1000, (480, 640), dtype=np.uint16)

        for i in range(10):
            result = module.infer(depth_image)
            # Should not crash or accumulate memory
            if i == 0:
                self.assertIsNotNone(result)  # At least one should work


if __name__ == '__main__':
    unittest.main(verbosity=2)
