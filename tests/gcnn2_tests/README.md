# Object Detection Module Tests

This directory contains comprehensive tests for the object detection module (GGCNN2) in the robot hand tracking system.

## Test Structure

### Test Files

- **`test_object_detection.py`** - Unit tests for the GGCNN2 model and GGcnn2Module wrapper
- **`test_model_integration.py`** - Integration tests for the complete object detection pipeline
- **`test_model_loading.py`** - Tests specifically for model loading and weight verification
- **`quick_test.py`** - Quick verification script to check basic functionality
- **`run_tests.py`** - Test runner script for executing all tests

### Test Categories

#### Unit Tests (`test_object_detection.py`)

- Model initialization and architecture
- Forward pass functionality
- Loss computation
- Preprocessing pipeline
- Postprocessing and grasp selection
- 2D to 3D pose conversion
- Coordinate transformations
- Error handling

#### Integration Tests (`test_model_integration.py`)

- Complete inference pipeline
- End-to-end workflow testing
- Error scenarios and edge cases
- Performance and memory usage
- Robustness testing

#### Model Loading Tests (`test_model_loading.py`)

- Model file existence and validity
- State dict loading
- Weight verification
- Device compatibility
- Model consistency

## Running Tests

### Prerequisites

1. Install required dependencies:

   ```bash
   pip install torch torchvision opencv-python numpy scipy
   ```

2. Ensure the model file exists:
   ```
   src/resources/ml_models/ggcnn2_weights_cornell/epoch_50_cornell_statedict.pt
   ```

### Quick Test (Recommended First)

Run the quick verification script to check basic functionality:

```bash
cd tests
python quick_test.py
```

This will verify:

- All imports work correctly
- Model file exists and is valid
- Basic model loading works
- Configuration is correct

### Full Test Suite

Run all tests:

```bash
cd tests
python run_tests.py
```

Run specific test modules:

```bash
python run_tests.py test_object_detection
python run_tests.py test_model_integration
python run_tests.py test_model_loading
```

### Individual Test Files

Run individual test files:

```bash
python -m unittest test_object_detection.py -v
python -m unittest test_model_integration.py -v
python -m unittest test_model_loading.py -v
```

## Test Coverage

The tests cover:

### Core Functionality

- ✅ Model architecture and initialization
- ✅ Forward pass and inference
- ✅ Preprocessing (depth image normalization, resizing)
- ✅ Postprocessing (grasp selection, quality filtering)
- ✅ 2D to 3D pose conversion
- ✅ Coordinate transformations
- ✅ Inverse kinematics integration

### Error Handling

- ✅ Invalid inputs (None, empty arrays, wrong shapes)
- ✅ Low quality grasps
- ✅ Invalid depth values
- ✅ Kinematics failures
- ✅ Model loading errors

### Edge Cases

- ✅ All-zero depth images
- ✅ Uniform depth images
- ✅ Invalid joint positions
- ✅ Device compatibility (CPU/CUDA)
- ✅ Memory usage and leaks

### Integration

- ✅ Complete pipeline workflow
- ✅ Mock dependencies
- ✅ Realistic test data
- ✅ Performance testing

## Expected Results

### Successful Test Run

```
Tests run: 45
Failures: 0
Errors: 0
Skipped: 2
Overall result: PASS
```

### Common Issues and Solutions

#### Model File Not Found

```
✗ Model file not found: /path/to/model.pt
```

**Solution**: Ensure the model file exists at the correct path in `src/resources/ml_models/ggcnn2_weights_cornell/`

#### PyTorch Import Error

```
✗ PyTorch import failed: No module named 'torch'
```

**Solution**: Install PyTorch: `pip install torch torchvision`

#### CUDA Tests Skipped

```
Skipped: 2
```

**Solution**: This is normal if CUDA is not available. Tests will run on CPU.

## Test Data

The tests use:

- **Synthetic depth images** with realistic object shapes
- **Mock dependencies** for telemetry, camera manager, and kinematics
- **Realistic joint positions** and transformations
- **Edge case scenarios** for robustness testing

## Contributing

When adding new tests:

1. Follow the existing naming conventions
2. Add comprehensive docstrings
3. Include both positive and negative test cases
4. Use realistic mock data
5. Test edge cases and error conditions
6. Update this README if adding new test categories

## Troubleshooting

### Import Errors

If you get import errors, ensure you're running tests from the `tests/` directory and that `src/` is in the Python path.

### Model Loading Issues

If model loading fails, check:

1. Model file exists and is readable
2. PyTorch version compatibility
3. File permissions
4. Model file integrity

### Memory Issues

If tests fail due to memory issues:

1. Reduce batch sizes in test data
2. Use CPU instead of CUDA for testing
3. Clear GPU memory between tests
