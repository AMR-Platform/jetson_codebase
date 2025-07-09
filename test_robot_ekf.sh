#!/bin/bash
# Quick Robot EKF Test Script
# Run this on your Jetson robot

echo "=== Jetson Robot EKF Test Setup ==="

# 1. Build the test
echo "Building EKF test..."
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake ..
make test_ekf_real_robot

if [ $? -ne 0 ]; then
    echo "Build failed! Check your CMake setup."
    exit 1
fi

echo "Build successful!"

# 2. Create test directory
mkdir -p ../ekf_test_results
cd ../ekf_test_results

echo "=== Starting EKF Tests ==="
echo "Results will be saved in: $(pwd)"

# 3. Run different test scenarios
echo "1. Running Stationary Test (30 seconds)..."
timeout 30s ../build/test_ekf_real_robot > stationary_test.log 2>&1
mv ekf_test_log.csv stationary_test.csv

echo "2. Manual tests:"
echo "   - Drive robot forward 1m and check position estimate"
echo "   - Rotate robot 360° and check orientation"
echo "   - Lift wheels to test slip detection"

echo "3. To run continuous monitoring:"
echo "   ../build/test_ekf_real_robot"

echo "=== Test Files Created ==="
ls -la *.csv *.log

echo "=== Next Steps ==="
echo "1. Check stationary_test.csv - position should not drift"
echo "2. Run manual motion tests"
echo "3. Use Python/MATLAB to plot results"
echo "4. Adjust parameters in SensorFusion constructor if needed"

echo "=== Python Analysis Example ==="
cat << 'EOF'
# Run this Python code to analyze results:
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('stationary_test.csv')
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(data['timestamp'], data['x'], label='X position')
plt.plot(data['timestamp'], data['y'], label='Y position')
plt.title('Position During Stationary Test')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(data['timestamp'], data['vx'], label='vx')
plt.plot(data['timestamp'], data['vy'], label='vy')
plt.title('Velocities During Stationary Test')
plt.legend()
plt.show()
EOF

echo "=== Test Complete ==="
