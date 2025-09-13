"""
Tests specifically for model loading and weight verification.
"""
from config.config import GGCNN2_MODEL_PATH
from object_detection.ggcnn2 import GGCNN2
import unittest
import torch
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


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

    def test_model_path_is_pytorch_file(self):
        """Test that the model path points to a PyTorch file."""
        self.assertTrue(str(GGCNN2_MODEL_PATH).endswith('.pt'),
                        f"Model path is not a .pt file: {GGCNN2_MODEL_PATH}")

    @unittest.skipIf(not GGCNN2_MODEL_PATH.exists(), "Model file not found")
    def test_load_state_dict(self):
        """Test loading state dict from file."""
        try:
            state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')
            self.assertIsInstance(state_dict, dict)
            self.assertGreater(len(state_dict), 0, "State dict is empty")

            # Check for expected keys (basic structure)
            expected_keys = ['pos_output.weight', 'cos_output.weight',
                             'sin_output.weight', 'width_output.weight']
            found_keys = [key for key in expected_keys if key in state_dict]
            self.assertGreater(len(
                found_keys), 0, f"Expected keys not found. Available keys: {list(state_dict.keys())}")

        except Exception as e:
            self.fail(f"Failed to load state dict: {e}")

    @unittest.skipIf(not GGCNN2_MODEL_PATH.exists(), "Model file not found")
    def test_model_loading_and_forward_pass(self):
        """Test complete model loading and forward pass."""
        try:
            # Load state dict
            state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')

            # Create model
            model = GGCNN2()

            # Load weights
            model.load_state_dict(state_dict)
            model.eval()

            # Test forward pass
            dummy_input = torch.randn(1, 1, 300, 300)
            with torch.no_grad():
                pos, cos, sin, width = model(dummy_input)

            # Verify outputs
            self.assertEqual(pos.shape, (1, 1, 300, 300))
            self.assertEqual(cos.shape, (1, 1, 300, 300))
            self.assertEqual(sin.shape, (1, 1, 300, 300))
            self.assertEqual(width.shape, (1, 1, 300, 300))

            # Verify output ranges (basic sanity checks)
            self.assertTrue(torch.all(torch.isfinite(pos)))
            self.assertTrue(torch.all(torch.isfinite(cos)))
            self.assertTrue(torch.all(torch.isfinite(sin)))
            self.assertTrue(torch.all(torch.isfinite(width)))

        except Exception as e:
            self.fail(f"Model loading and forward pass failed: {e}")

    @unittest.skipIf(not GGCNN2_MODEL_PATH.exists(), "Model file not found")
    def test_model_consistency(self):
        """Test model consistency across multiple forward passes."""
        try:
            # Load model
            state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')
            model = GGCNN2()
            model.load_state_dict(state_dict)
            model.eval()

            # Test with same input multiple times
            dummy_input = torch.randn(1, 1, 300, 300)

            with torch.no_grad():
                pos1, cos1, sin1, width1 = model(dummy_input)
                pos2, cos2, sin2, width2 = model(dummy_input)

            # Outputs should be identical for same input
            self.assertTrue(torch.allclose(pos1, pos2, atol=1e-6))
            self.assertTrue(torch.allclose(cos1, cos2, atol=1e-6))
            self.assertTrue(torch.allclose(sin1, sin2, atol=1e-6))
            self.assertTrue(torch.allclose(width1, width2, atol=1e-6))

        except Exception as e:
            self.fail(f"Model consistency test failed: {e}")

    @unittest.skipIf(not GGCNN2_MODEL_PATH.exists(), "Model file not found")
    def test_model_device_compatibility(self):
        """Test model loading on different devices."""
        try:
            # Load state dict
            state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')
            model = GGCNN2()
            model.load_state_dict(state_dict)
            model.eval()

            # Test on CPU
            dummy_input_cpu = torch.randn(1, 1, 300, 300)
            with torch.no_grad():
                pos_cpu, cos_cpu, sin_cpu, width_cpu = model(dummy_input_cpu)

            self.assertEqual(pos_cpu.device.type, 'cpu')

            # Test CUDA if available
            if torch.cuda.is_available():
                model_cuda = GGCNN2()
                model_cuda.load_state_dict(state_dict)
                model_cuda.to('cuda')
                model_cuda.eval()

                dummy_input_cuda = torch.randn(1, 1, 300, 300).to('cuda')
                with torch.no_grad():
                    pos_cuda, cos_cuda, sin_cuda, width_cuda = model_cuda(
                        dummy_input_cuda)

                self.assertEqual(pos_cuda.device.type, 'cuda')

                # Move back to CPU for comparison
                pos_cuda_cpu = pos_cuda.cpu()
                self.assertTrue(torch.allclose(
                    pos_cpu, pos_cuda_cpu, atol=1e-4))

        except Exception as e:
            self.fail(f"Device compatibility test failed: {e}")

    def test_model_architecture(self):
        """Test model architecture matches expected structure."""
        model = GGCNN2()

        # Check that model has expected components
        self.assertTrue(hasattr(model, 'features'))
        self.assertTrue(hasattr(model, 'pos_output'))
        self.assertTrue(hasattr(model, 'cos_output'))
        self.assertTrue(hasattr(model, 'sin_output'))
        self.assertTrue(hasattr(model, 'width_output'))

        # Check output layers are Conv2d
        self.assertIsInstance(model.pos_output, torch.nn.Conv2d)
        self.assertIsInstance(model.cos_output, torch.nn.Conv2d)
        self.assertIsInstance(model.sin_output, torch.nn.Conv2d)
        self.assertIsInstance(model.width_output, torch.nn.Conv2d)

        # Check output channels
        self.assertEqual(model.pos_output.out_channels, 1)
        self.assertEqual(model.cos_output.out_channels, 1)
        self.assertEqual(model.sin_output.out_channels, 1)
        self.assertEqual(model.width_output.out_channels, 1)

    def test_model_parameter_count(self):
        """Test model has reasonable number of parameters."""
        model = GGCNN2()
        total_params = sum(p.numel() for p in model.parameters())

        # GGCNN2 should have a reasonable number of parameters
        # This is a sanity check - adjust bounds based on actual model size
        self.assertGreater(total_params, 1000, "Model has too few parameters")
        self.assertLess(total_params, 10000000,
                        "Model has too many parameters")

        print(f"Model has {total_params:,} parameters")

    @unittest.skipIf(not GGCNN2_MODEL_PATH.exists(), "Model file not found")
    def test_model_weight_ranges(self):
        """Test that loaded model weights are in reasonable ranges."""
        try:
            state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')

            for name, param in state_dict.items():
                if 'weight' in name:
                    # Check for NaN or infinite values
                    self.assertTrue(torch.all(torch.isfinite(param)),
                                    f"Weight {name} contains NaN or infinite values")

                    # Check weight magnitudes (shouldn't be extremely large)
                    max_weight = torch.max(torch.abs(param)).item()
                    self.assertLess(max_weight, 100.0,
                                    f"Weight {name} has extremely large values: {max_weight}")

        except Exception as e:
            self.fail(f"Weight range test failed: {e}")


if __name__ == '__main__':
    unittest.main(verbosity=2)
