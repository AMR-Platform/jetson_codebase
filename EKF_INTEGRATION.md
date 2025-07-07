# EKF Sensor Fusion Integration

## Overview

This integration adds a comprehensive Extended Kalman Filter (EKF) sensor fusion system to the robot codebase. The EKF fuses data from multiple sensors to provide accurate pose and velocity estimation.

## Robot Parameters

The system is configured for your specific robot with these parameters:
- **Wheel diameter**: 200mm (0.2m)
- **Wheel radius**: 100mm (0.1m)  
- **Wheel base**: 500mm (0.5m)
- **Encoder resolution**: 40,000 ticks per revolution
- **Wheel circumference**: π × 0.2m = 0.628m
- **Distance per tick**: 0.628m / 40,000 = 1.57 × 10⁻⁵ m/tick

## State Vector

The EKF maintains a 6-dimensional state vector:
```
[x, y, θ, vx, vy, ω]
```
Where:
- `x, y`: Position in global coordinates (meters)
- `θ`: Orientation angle (radians)
- `vx, vy`: Linear velocities in robot frame (m/s)
- `ω`: Angular velocity (rad/s)

## Sensor Inputs

### 1. Wheel Encoders
- **Input**: Encoder tick differences (`dEncL`, `dEncR`)
- **Frequency**: ~50Hz (same as main loop)
- **Purpose**: Primary motion estimation and velocity updates

### 2. IMU Orientation  
- **Input**: Yaw angle from IMU (`sensor.yaw`)
- **Units**: Degrees (converted to radians internally)
- **Purpose**: Absolute orientation correction

### 3. Gyroscope
- **Input**: Angular velocity (`sensor.gyroZ`)
- **Units**: Degrees/second (converted to radians/second)
- **Purpose**: Angular velocity measurement updates

### 4. Accelerometer (Optional)
- **Input**: 3-axis acceleration (`accelX`, `accelY`, `accelZ`)
- **Units**: m/s²
- **Purpose**: Zero Velocity Update (ZUPT) when robot is stationary

## Files Added/Modified

### New Files:
- `include/SensorFusion.hpp` - Main EKF class interface
- `src/SensorFusion.cpp` - EKF implementation 
- `include/robot_utils.hpp` - Robot parameter constants and utility functions
- `src/main_ekf.cpp` - Complete robot localization system
- `src/test_ekf.cpp` - EKF testing program

### Modified Files:
- `include/serial_com.hpp` - Added accelerometer/gyro fields to SensorPacket
- `src/serial_com.cpp` - Updated parser for additional IMU data
- `CMakeLists.txt` - Added new executables and dependencies

## Building

```bash
mkdir build && cd build
cmake ..
make -j4
```

This creates three executables:
1. `main_exe` - Original program (unchanged)
2. `robot_ekf` - Full robot with EKF localization
3. `test_ekf` - EKF testing program

## Usage

### Testing the EKF
```bash
./test_ekf
```
This runs a simulation to verify the EKF is working correctly.

### Running the Robot with EKF
```bash
./robot_ekf [serial_port]
```
Default serial port is `/dev/ttyUSB0`. Example:
```bash
./robot_ekf /dev/ttyACM0
```

### Serial Protocol Extension

If your MCU doesn't currently send accelerometer/gyro data, you can extend the telemetry message format. The current parser expects:

**Basic format** (existing):
```
yaw roll pitch encL encR vbat1 vbat2 cliffL cliffC cliffR emergency profileDone
```

**Extended format** (optional):
```
yaw roll pitch encL encR vbat1 vbat2 cliffL cliffC cliffR emergency profileDone accelX accelY accelZ gyroX gyroY gyroZ
```

The parser will work with both formats - it will use accelerometer/gyro data if available, otherwise it falls back to encoder + IMU yaw only.

## Configuration

### Noise Parameters

You can tune the EKF performance by adjusting noise parameters in `main_ekf.cpp`:

```cpp
SensorFusion ekf(
    RobotUtils::WHEEL_RADIUS,     // Physical parameters
    RobotUtils::WHEEL_BASE,       
    dt_,                          
    1e-4,  // processNoisePos     - Position process noise
    1e-5,  // processNoiseAng     - Angular process noise  
    1e-3,  // processNoiseVel     - Velocity process noise
    1e-4,  // gyroNoise           - Gyroscope measurement noise
    1e-3,  // imuNoise            - IMU yaw measurement noise
    1e-2   // accelNoise          - Accelerometer measurement noise
);
```

### Update Frequencies

The system is designed for:
- **Main loop**: 50Hz (20ms)
- **Encoder updates**: Every cycle (50Hz)
- **IMU updates**: 5-10Hz 
- **Gyro updates**: 10-20Hz
- **Accelerometer**: 5-10Hz

## API Usage

### Getting Robot Pose
```cpp
auto pose = robot.getPose();  // Returns [x, y, theta]
std::cout << "Position: (" << pose[0] << ", " << pose[1] << ")" << std::endl;
std::cout << "Heading: " << pose[2] * 180.0 / M_PI << " degrees" << std::endl;
```

### Getting Velocities
```cpp
auto velocities = robot.getVelocities();  // Returns [vx, vy, omega]
std::cout << "Forward velocity: " << velocities[0] << " m/s" << std::endl;
std::cout << "Angular velocity: " << velocities[2] << " rad/s" << std::endl;
```

### Sending Commands
```cpp
CommandPacket cmd;
cmd.mode = AUTONOMOUS;
cmd.distance = 1.0;  // Move 1 meter forward
cmd.angle = 0.0;     // No turning
robot.sendCommand(cmd);
```

## Data Logging

The system automatically logs pose data to CSV files named `robot_pose_YYYYMMDD_HHMMSS.csv` with columns:
- timestamp, x, y, theta, vx, vy, omega, leftVel, rightVel, imuYaw, gyroZ

## Troubleshooting

### Build Issues
- Ensure Eigen3 is installed: `sudo apt install libeigen3-dev`
- Check OpenCV installation: `pkg-config --modversion opencv4`
- Verify Boost libraries: `sudo apt install libboost-all-dev`

### Runtime Issues
- **Serial communication fails**: Check port permissions (`sudo chmod 666 /dev/ttyUSB0`)
- **EKF not initializing**: Verify encoder data is being received
- **Poor pose estimation**: Tune noise parameters or check sensor calibration

### Performance Tips
- Monitor the main loop frequency - should stay close to 50Hz
- Check sensor data quality (no NaN or excessive noise)
- Verify encoder tick counts are reasonable
- Ensure IMU yaw is in correct units (degrees)

## Integration with Path Planning

The EKF pose estimates can be directly used with your existing path planning system:

```cpp
auto currentPose = robot.getPose();
// Use currentPose[0], currentPose[1], currentPose[2] for path planning
// This provides much more accurate localization than encoder-only odometry
```

## Next Steps

1. **Test on real hardware** with your specific sensor setup
2. **Calibrate noise parameters** based on actual sensor performance  
3. **Add LIDAR integration** for even more robust localization
4. **Implement loop closure** detection for long-term accuracy
5. **Add GPS fusion** for outdoor applications (if applicable)
