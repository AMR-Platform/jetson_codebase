#!/bin/bash

# Build and Test Script for EKF Validation
echo "=== EKF Testing Build Script ==="
echo "This script builds and runs the EKF test programs"
echo ""

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found. Please run this from the project root."
    exit 1
fi

# Create build directory
echo "1. Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "2. Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Debug

if [ $? -ne 0 ]; then
    echo "Error: CMake configuration failed!"
    exit 1
fi

# Build the test programs
echo "3. Building test programs..."
make test_ekf_simple test_ekf_synthetic -j$(nproc)

if [ $? -ne 0 ]; then
    echo "Error: Build failed!"
    exit 1
fi

echo "4. Build successful! Test programs created:"
echo "   - test_ekf_simple: Basic validation tests"
echo "   - test_ekf_synthetic: Advanced synthetic data tests"
echo ""

# Run the simple tests
echo "5. Running simple EKF tests..."
echo "----------------------------------------"
./test_ekf_simple

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Simple tests completed successfully!"
else
    echo ""
    echo "✗ Simple tests had issues."
fi

echo ""
echo "=== NEXT STEPS ==="
echo "1. Review the test results above"
echo "2. Run './test_ekf_synthetic' for more detailed testing"
echo "3. Check generated CSV files for data analysis"
echo "4. If tests pass, your EKF should work correctly with real hardware"
echo ""
