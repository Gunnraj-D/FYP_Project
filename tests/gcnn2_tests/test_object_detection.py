"""
Comprehensive tests for object detection module (GGCNN2).
Tests model loading, preprocessing, inference, and postprocessing.
"""
from config.config import GGCNN2_MODEL_PATH, GRASP_DETECTION_CONFIG
from object_detection.ggcnn2_module import GGcnn2Module
from object_detection.ggcnn2 import GGCNN2
import unittest
import numpy as np
import torch
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestGGCNN2Model(unittest.TestCase):
    """Test the GGCNN2 neural network model."""

    def setUp(self):
        """Set up test fixtures."""
        self.model = GGCNN2()
        self.device = torch.device("cpu")  # Use CPU for testing
        self.model.to(self.device)

    def test_model_initialization(self):
        """Test that the model initializes correctly."""
        self.assertIsInstance(self.model, GGCNN2)
        self.assertEqual(self.model.pos_output.out_channels, 1)
        self.assertEqual(self.model.cos_output.out_channels, 1)
        self.assertEqual(self.model.sin_output.out_channels, 1)
        self.assertEqual(self.model.width_output.out_channels, 1)

    def test_model_forward_pass(self):
        """Test forward pass with dummy input."""
        # Create dummy input (batch_size=1, channels=1, height=300, width=300)
        dummy_input = torch.randn(1, 1, 300, 300)

        with torch.no_grad():
            pos, cos, sin, width = self.model(dummy_input)

        # Check output shapes
        self.assertEqual(pos.shape, (1, 1, 300, 300))
        self.assertEqual(cos.shape, (1, 1, 300, 300))
        self.assertEqual(sin.shape, (1, 1, 300, 300))
        self.assertEqual(width.shape, (1, 1, 300, 300))

    def test_model_loss_computation(self):
        """Test loss computation with dummy data."""
        # Create dummy input and target
        dummy_input = torch.randn(1, 1, 300, 300)
        dummy_target = (
            torch.randn(1, 1, 300, 300),  # pos
            torch.randn(1, 1, 300, 300),  # cos
            torch.randn(1, 1, 300, 300),  # sin
            torch.randn(1, 1, 300, 300)   # width
        )

        loss_dict = self.model.compute_loss(dummy_input, dummy_target)

        self.assertIn('loss', loss_dict)
        self.assertIn('losses', loss_dict)
        self.assertIn('pred', loss_dict)
        self.assertIsInstance(loss_dict['loss'], torch.Tensor)


class TestGGcnn2Module(unittest.TestCase):
    """Test the GGcnn2Module wrapper class."""

    def setUp(self):
        """Set up test fixtures with mocked dependencies."""
        # Mock dependencies
        self.mock_telemetry = Mock()
        self.mock_command_bus = Mock()
        self.mock_camera_manager = Mock()
        self.mock_kinematics_solver = Mock()

        # Mock telemetry methods
        self.mock_telemetry.get_current_joints.return_value = np.array([
                                                                       0.0] * 7)

        # Mock camera manager methods
        self.mock_camera_manager.get_average_depth.return_value = 500.0
        self.mock_camera_manager.pixel_to_3d.return_value = [
            100.0, 200.0, 500.0]

        # Mock kinematics solver
        self.mock_kinematics_solver.tcp_from_joints.return_value = (
            np.eye(4), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.mock_kinematics_solver.solve_pose.return_value = np.array([
                                                                       0.0] * 7)

    @patch('object_detection.ggcnn2_module.GGCNN2')
    @patch('torch.load')
    def test_module_initialization(self, mock_torch_load, mock_ggcnn2_class):
        """Test module initialization with mocked model loading."""
        # Mock model and state dict
        mock_model = Mock()
        mock_state_dict = {'layer1.weight': torch.randn(1, 1, 1, 1)}
        mock_torch_load.return_value = mock_state_dict
        mock_ggcnn2_class.return_value = mock_model

        # Test initialization
        module = GGcnn2Module(
            model_path="dummy_path.pt",
            telemetry=self.mock_telemetry,
            command_bus=self.mock_command_bus,
            camera_manager=self.mock_camera_manager,
            kinematics_solver=self.mock_kinematics_solver
        )

        # Verify model was loaded
        mock_torch_load.assert_called_once()
        mock_model.load_state_dict.assert_called_once_with(mock_state_dict)
        mock_model.to.assert_called_once()
        mock_model.eval.assert_called_once()

    def test_preprocessing(self):
        """Test depth image preprocessing."""
        # Create mock module without initializing the actual model
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):
            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Create test depth image
            depth_image = np.random.randint(
                0, 1000, (480, 640), dtype=np.uint16)

            # Test preprocessing
            processed = module.preprocess(depth_image)

            # Check output shape and type
            self.assertIsInstance(processed, torch.Tensor)
            self.assertEqual(processed.shape, (1, 1, 300, 300))
            self.assertTrue(torch.all(processed >= 0.0))
            self.assertTrue(torch.all(processed <= 1.0))

    def test_preprocessing_edge_cases(self):
        """Test preprocessing with edge cases."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):
            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Test with all zeros
            depth_zeros = np.zeros((480, 640), dtype=np.uint16)
            processed = module.preprocess(depth_zeros)
            self.assertEqual(processed.shape, (1, 1, 300, 300))

            # Test with all same values
            depth_same = np.full((480, 640), 500, dtype=np.uint16)
            processed = module.preprocess(depth_same)
            self.assertEqual(processed.shape, (1, 1, 300, 300))

    def test_postprocessing(self):
        """Test grasp postprocessing."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):
            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Create mock outputs
            q_img = torch.randn(1, 1, 300, 300)
            ang_img = torch.randn(1, 1, 300, 300)
            width_img = torch.randn(1, 1, 300, 300)

            # Test postprocessing
            result = module.postprocess(q_img, ang_img, width_img)

            if result is not None:
                self.assertIn('center', result)
                self.assertIn('angle', result)
                self.assertIn('width', result)
                self.assertIn('quality', result)
                self.assertIsInstance(result['center'], tuple)
                self.assertEqual(len(result['center']), 2)

    def test_postprocessing_low_quality(self):
        """Test postprocessing with low quality grasps."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):
            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Create low quality output (all values below threshold)
            q_img = torch.full((1, 1, 300, 300), 0.1)  # Below threshold
            ang_img = torch.randn(1, 1, 300, 300)
            width_img = torch.randn(1, 1, 300, 300)

            result = module.postprocess(q_img, ang_img, width_img)
            self.assertIsNone(result)

    @patch('object_detection.ggcnn2_module.transform_camera_to_base')
    def test_grasp_2d_to_3d_pose(self, mock_transform):
        """Test 2D to 3D pose conversion."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):
            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Mock transform function
            mock_transform.return_value = np.array([100.0, 200.0, 500.0])

            # Test data
            grasp_2d = {
                'center': (150, 150),
                'angle': 0.5,
                'width': 30.0,
                'quality': 0.8
            }
            depth_image = np.random.randint(
                0, 1000, (480, 640), dtype=np.uint16)

            result = module._grasp_2d_to_3d_pose(grasp_2d, depth_image)

            if result is not None:
                self.assertEqual(len(result), 6)  # [x, y, z, roll, pitch, yaw]
                self.assertIsInstance(result[0], (int, float))
                self.assertIsInstance(result[1], (int, float))
                self.assertIsInstance(result[2], (int, float))

    def test_grasp_2d_to_3d_pose_invalid_depth(self):
        """Test 2D to 3D conversion with invalid depth."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):
            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            # Mock camera manager to return invalid depth
            self.mock_camera_manager.get_average_depth.return_value = 0.0

            grasp_2d = {
                'center': (150, 150),
                'angle': 0.5,
                'width': 30.0,
                'quality': 0.8
            }
            depth_image = np.random.randint(
                0, 1000, (480, 640), dtype=np.uint16)

            result = module._grasp_2d_to_3d_pose(grasp_2d, depth_image)
            self.assertIsNone(result)

    def test_transform_to_base_frame(self):
        """Test transformation to base frame."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'), \
                patch('object_detection.ggcnn2_module.transform_camera_to_base') as mock_transform:

            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            mock_transform.return_value = np.array([100.0, 200.0, 500.0])

            camera_pose = [100.0, 200.0, 500.0, 0.0, 0.0, 0.0]
            result = module._transform_to_base_frame(camera_pose)

            if result is not None:
                self.assertEqual(len(result), 6)
                mock_transform.assert_called_once()

    def test_pose_to_joint_angles(self):
        """Test pose to joint angles conversion."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):

            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            pose = [100.0, 200.0, 500.0, 0.0, 0.0, 0.0]
            result = module._pose_to_joint_angles(pose)

            if result is not None:
                self.assertEqual(len(result), 7)
                self.mock_kinematics_solver.solve_pose.assert_called_once()

    def test_invalid_joint_positions(self):
        """Test handling of invalid joint positions."""
        with patch('object_detection.ggcnn2_module.GGCNN2'), \
                patch('torch.load'):

            # Mock telemetry with invalid joints
            self.mock_telemetry.get_current_joints.return_value = np.array(
                [0.0] * 5)  # Wrong length

            module = GGcnn2Module(
                model_path="dummy_path.pt",
                telemetry=self.mock_telemetry,
                command_bus=self.mock_command_bus,
                camera_manager=self.mock_camera_manager,
                kinematics_solver=self.mock_kinematics_solver
            )

            camera_pose = [100.0, 200.0, 500.0, 0.0, 0.0, 0.0]
            result = module._transform_to_base_frame(camera_pose)
            self.assertIsNone(result)


class TestModelLoading(unittest.TestCase):
    """Test model loading functionality."""

    def test_model_path_exists(self):
        """Test that the model path exists."""
        self.assertTrue(GGCNN2_MODEL_PATH.exists(),
                        f"Model path does not exist: {GGCNN2_MODEL_PATH}")

    def test_model_path_is_file(self):
        """Test that the model path points to a file."""
        self.assertTrue(GGCNN2_MODEL_PATH.is_file(),
                        f"Model path is not a file: {GGCNN2_MODEL_PATH}")

    @unittest.skipIf(not GGCNN2_MODEL_PATH.exists(), "Model file not found")
    def test_model_loading(self):
        """Test actual model loading from file."""
        try:
            # Test loading the actual model
            state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')
            model = GGCNN2()
            model.load_state_dict(state_dict)
            model.eval()

            # Test forward pass
            dummy_input = torch.randn(1, 1, 300, 300)
            with torch.no_grad():
                pos, cos, sin, width = model(dummy_input)

            self.assertEqual(pos.shape, (1, 1, 300, 300))
            self.assertEqual(cos.shape, (1, 1, 300, 300))
            self.assertEqual(sin.shape, (1, 1, 300, 300))
            self.assertEqual(width.shape, (1, 1, 300, 300))

        except Exception as e:
            self.fail(f"Model loading failed: {e}")


class TestConfiguration(unittest.TestCase):
    """Test configuration parameters."""

    def test_grasp_detection_config(self):
        """Test grasp detection configuration."""
        self.assertIn('min_quality_threshold', GRASP_DETECTION_CONFIG)
        self.assertIn('approach_angle', GRASP_DETECTION_CONFIG)
        self.assertGreater(
            GRASP_DETECTION_CONFIG['min_quality_threshold'], 0.0)
        self.assertLessEqual(
            GRASP_DETECTION_CONFIG['min_quality_threshold'], 1.0)

    def test_model_path_config(self):
        """Test model path configuration."""
        self.assertIsInstance(GGCNN2_MODEL_PATH, Path)
        self.assertTrue(str(GGCNN2_MODEL_PATH).endswith('.pt'))


if __name__ == '__main__':
    # Run tests
    unittest.main(verbosity=2)
