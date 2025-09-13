"""
Quick test script to verify object detection module setup.
Run this to check if everything is working before running full test suite.
"""
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def test_imports():
    """Test that all required modules can be imported."""
    try:
        import torch
        print(f"✓ PyTorch {torch.__version__} imported successfully")
    except ImportError as e:
        print(f"✗ PyTorch import failed: {e}")
        return False

    try:
        import numpy as np
        print(f"✓ NumPy {np.__version__} imported successfully")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")
        return False

    try:
        import cv2
        print(f"✓ OpenCV {cv2.__version__} imported successfully")
    except ImportError as e:
        print(f"✗ OpenCV import failed: {e}")
        return False

    try:
        from object_detection.ggcnn2 import GGCNN2
        print("✓ GGCNN2 model imported successfully")
    except ImportError as e:
        print(f"✗ GGCNN2 import failed: {e}")
        return False

    try:
        from config.config import GGCNN2_MODEL_PATH, GRASP_DETECTION_CONFIG
        print("✓ Configuration imported successfully")
    except ImportError as e:
        print(f"✗ Configuration import failed: {e}")
        return False

    return True


def test_model_path():
    """Test that model path exists and is valid."""
    try:
        from config.config import GGCNN2_MODEL_PATH

        if GGCNN2_MODEL_PATH.exists():
            print(f"✓ Model file exists: {GGCNN2_MODEL_PATH}")
        else:
            print(f"✗ Model file not found: {GGCNN2_MODEL_PATH}")
            return False

        if GGCNN2_MODEL_PATH.is_file():
            print(f"✓ Model path points to a file")
        else:
            print(f"✗ Model path does not point to a file")
            return False

        if str(GGCNN2_MODEL_PATH).endswith('.pt'):
            print(f"✓ Model file has .pt extension")
        else:
            print(f"✗ Model file does not have .pt extension")
            return False

        return True
    except Exception as e:
        print(f"✗ Model path test failed: {e}")
        return False


def test_model_loading():
    """Test basic model loading."""
    try:
        import torch
        from object_detection.ggcnn2 import GGCNN2
        from config.config import GGCNN2_MODEL_PATH

        print("Testing model loading...")

        # Load state dict
        state_dict = torch.load(GGCNN2_MODEL_PATH, map_location='cpu')
        print(f"✓ State dict loaded with {len(state_dict)} parameters")

        # Create model
        model = GGCNN2()
        model.load_state_dict(state_dict)
        model.eval()
        print("✓ Model created and weights loaded")

        # Test forward pass
        dummy_input = torch.randn(1, 1, 300, 300)
        with torch.no_grad():
            pos, cos, sin, width = model(dummy_input)

        print(
            f"✓ Forward pass successful - outputs: pos{pos.shape}, cos{cos.shape}, sin{sin.shape}, width{width.shape}")

        return True
    except Exception as e:
        print(f"✗ Model loading test failed: {e}")
        return False


def test_configuration():
    """Test configuration parameters."""
    try:
        from config.config import GRASP_DETECTION_CONFIG

        required_keys = ['min_quality_threshold',
                         'approach_angle', 'max_grasp_width', 'min_grasp_width']

        for key in required_keys:
            if key in GRASP_DETECTION_CONFIG:
                print(
                    f"✓ Config key '{key}' found: {GRASP_DETECTION_CONFIG[key]}")
            else:
                print(f"✗ Config key '{key}' missing")
                return False

        # Validate ranges
        if 0.0 <= GRASP_DETECTION_CONFIG['min_quality_threshold'] <= 1.0:
            print("✓ Quality threshold in valid range")
        else:
            print("✗ Quality threshold out of valid range")
            return False

        return True
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        return False


def main():
    """Run all quick tests."""
    print("Running quick tests for object detection module...")
    print("=" * 50)

    tests = [
        ("Import Test", test_imports),
        ("Model Path Test", test_model_path),
        ("Model Loading Test", test_model_loading),
        ("Configuration Test", test_configuration),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * 20)
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 50)
    print("QUICK TEST SUMMARY")
    print("=" * 50)

    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")
        if not result:
            all_passed = False

    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")

    if all_passed:
        print("\n✓ All quick tests passed! You can now run the full test suite.")
        print("Run: python tests/run_tests.py")
    else:
        print("\n✗ Some tests failed. Please fix the issues before running the full test suite.")

    return all_passed


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
