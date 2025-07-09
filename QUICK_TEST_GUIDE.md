# Quick EKF Testing Commands

## Prerequisites
1. Make sure you're on your Jetson robot
2. Ensure all sensors are connected (encoders, IMU, accelerometer)
3. Check your serial port connection (usually `/dev/ttyUSB0` or `/dev/ttyACM0`)
4. Navigate to the project directory

## Step 1: Build the Tests

```bash
# Navigate to project directory
cd /path/to/jetson_codebase

# Create build directory
mkdir -p build
cd build

# Configure and build
cmake ..
make -j4

# Verify build was successful
ls -la main_exe test_ekf_*
```

Expected output: You should see executables like `main_exe`, `test_ekf_simple`, etc.

## Step 2: Check Hardware Connection

```bash
# Find available serial ports
ls /dev/tty*

# Common robot ports:
ls /dev/ttyUSB*  # USB-to-serial adapters
ls /dev/ttyACM*  # Arduino/microcontroller boards

# Test specific port (replace with your actual port)
sudo chmod 666 /dev/ttyUSB0
```

## Step 3: Quick Validation Tests (No Hardware Needed)

### Test A: Simple Logic Tests  
```bash
# Test basic EKF logic
./test_ekf_simple
```
**Expected**: 4 test scenarios with PASS/FAIL results. All should show "✓ PASS".

## Step 4: Real Robot Hardware Tests

### Test B: Real Robot Test (Main Test)
```bash
# Run the main executable - it will automatically run the EKF test harness
./main_exe
```

**What this does**:
- Automatically detects and connects to your robot via serial
- Tests 7 phases automatically (initialization, stationary, forward, constant velocity, turning, etc.)
- Creates detailed `ekf_test_log.csv` 
- Shows real-time EKF state and sensor readings
- Detects wheel slip/lift conditions
- Runs for about 2.5 minutes total

**Expected Real-Time Output**:
```
╔════════════════════════════════════════════════════════════╗
║                    ROBOT EKF TESTING                      ║
║              Real Hardware Validation                     ║
╚════════════════════════════════════════════════════════════╝

✅ Connected to robot on /dev/ttyUSB0
🔄 Starting phase: initialization (duration: 0.3s)

Current Test Phase: stationary
Time: 5.23s

--- EKF State ---
Position: (0.002, -0.001) m
Heading:  0.05 rad (2.9°)
Velocity: (0.01, 0.00) m/s
Ang. Vel: 0.00 rad/s (0.0°/s)

--- Raw Sensor Data ---
Encoders: L=0.00 R=0.00 m/s
         (Δ: L=0 R=2 ticks)
IMU:      θ=2.1° roll=0.5° pitch=-1.2°
Gyro:     ωz=0.02°/s
Accel:    (0.12, -0.05) m/s²

--- Status ---
Packet Valid: YES
Sensors: Gyro=OK IMU=OK Accel=OK Enc=OK
```

### Test C: Automated Test Script (Alternative)
```bash
# Return to project root
cd ..

# Make script executable
chmod +x test_robot_ekf.sh

# Run automated test suite
./test_robot_ekf.sh
```

## Step 5: Check Results During Test

### Monitor in Real-Time (Open Second Terminal)
```bash
# Watch the log file as it's being created
tail -f ekf_test_log.csv

# Or just see latest entries
tail -20 ekf_test_log.csv

# Count how many entries we have
wc -l ekf_test_log.csv
```

### Check for Issues
```bash
# Look for warnings or errors
grep -i "warning\|error\|slip\|invalid" *.log

# Check if test phases are running
grep -c "stationary" ekf_test_log.csv
grep -c "forward" ekf_test_log.csv
```

## Step 6: Analyze Results

### Quick Analysis Commands
```bash
# View first and last few lines
head -5 ekf_test_log.csv    # Headers and initial data
tail -10 ekf_test_log.csv   # Final results

# Check position range during test
cut -d',' -f2,3 ekf_test_log.csv | sort -n

# Look at velocity data
cut -d',' -f5,6 ekf_test_log.csv | tail -20
```

### Generate Quick Plot (if Python available)
```bash
python3 -c "
import pandas as pd
import matplotlib.pyplot as plt
data = pd.read_csv('ekf_test_log.csv')
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(data['x'], data['y'])
plt.title('Robot Trajectory')
plt.axis('equal')
plt.subplot(2,1,2)
plt.plot(data['timestamp'], data['vx'], label='vx')
plt.plot(data['timestamp'], data['leftVel'], label='left encoder')
plt.legend()
plt.title('Velocity Comparison')
plt.tight_layout()
plt.savefig('ekf_results.png')
print('Saved ekf_results.png')
"
```

## Step 7: Expected Test Results

### ✅ Good Results:
- **Stationary test**: Position drift < 1cm over 20 seconds
- **Forward test**: Position tracks movement, velocities match encoders
- **Constant velocity**: EKF velocity converges to encoder readings
- **Turning test**: Angular velocity tracking works
- **No "WHEEL SLIP" false positives during normal operation
- **Valid sensor packets throughout test**

### ❌ Problem Signs:
```
# Bad signs to watch for:
⚠️  Invalid sensor packet received
⚠️  WHEEL SLIP/LIFT DETECTED!
❌ Failed to connect to robot
WARNING: Position estimates very large!
WARNING: Velocity estimates very large!
```

## Step 8: Troubleshooting Commands

### If Connection Fails:
```bash
# Check available ports
ls /dev/tty* | grep -E "(USB|ACM)"

# Test port permissions
sudo chmod 666 /dev/ttyUSB0

# Check if robot is responsive
echo "test" > /dev/ttyUSB0

# Try different baud rate or port
./test_ekf_real_robot /dev/ttyACM0
```

### If Build Fails:
```bash
# Clean and rebuild
cd build
make clean
cd ..
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make VERBOSE=1
```

### If EKF Behaves Badly:
```bash
# Check robot physical parameters in the code
grep -n "WHEEL_RADIUS\|WHEEL_BASE" include/localization/robot_utils.hpp

# Edit parameters if needed
nano include/localization/robot_utils.hpp
# Then rebuild: make test_ekf_real_robot
```

### If Sensor Data Looks Wrong:
```bash
# Test just the serial communication
cd build
# Create simple test that just reads and prints sensor data
```

## Step 9: Manual Testing Scenarios

### Stationary Test (Manual)
```bash
# Place robot on flat surface, don't move it
./test_ekf_real_robot /dev/ttyUSB0
# Let it run for 1 minute, check position doesn't drift
```

### Movement Test (Manual)
```bash
# Start test, then:
# 1. Push robot forward 1 meter
# 2. Let it sit still
# 3. Turn robot 90 degrees
# 4. Push forward again
# Check that EKF tracks the movement
```

## Next Steps After Testing

### If Tests Pass ✅:
1. **Integrate with main robot code**: The EKF is ready for real use
2. **Tune parameters**: Adjust noise values if needed
3. **Run longer tests**: Test for hours to check stability

### If Tests Fail ❌:
1. **Check hardware**: Verify all sensors working
2. **Adjust parameters**: Robot wheel radius, wheelbase
3. **Check coordinate frames**: Make sure sensor orientations are correct
4. **Debug serial communication**: Test sensor data quality

### Integration Ready Checklist:
- [ ] All tests pass
- [ ] Position tracking accurate
- [ ] Velocity estimates reasonable  
- [ ] No excessive slip detection
- [ ] Stable over long runs
- [ ] Good performance at 100Hz

---

**Need Help?** 
- Check console output for specific error messages
- Verify robot physical parameters match the code
- Test serial communication separately first
- Look at the detailed `ROBOT_EKF_TESTING.md` for more troubleshooting
