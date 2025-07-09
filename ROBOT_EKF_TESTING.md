# Robot EKF Testing Protocol

## Hardware Requirements
- Jetson-based robot with wheel encoders
- IMU/Gyroscope sensor
- Accelerometer
- Known wheel radius and wheelbase measurements

## Test Setup

### 1. Robot Parameters (Update in test code)
```cpp
// Measure these from your actual robot
double wheelRadius = 0.05;    // meters (measure your wheel radius)
double wheelBase = 0.3;       // meters (distance between wheels)
double dt = 0.01;             // 100Hz update rate (10ms period)
```

### 2. Sensor Integration
Replace the placeholder functions in `readSensors()` with your actual sensor reading code:

```cpp
SensorData readSensors() {
    SensorData data;
    
    // YOUR ENCODER INTERFACE - example:
    data.leftVel = robot.getLeftWheelVelocity();    // m/s
    data.rightVel = robot.getRightWheelVelocity();  // m/s
    
    // YOUR IMU INTERFACE - example:
    data.gyroZ = imu.getAngularVelocityZ();         // rad/s
    data.imuYaw = imu.getYaw();                     // rad
    data.accelX = imu.getAccelerationX();           // m/s²
    data.accelY = imu.getAccelerationY();           // m/s²
    
    // Set sensor availability
    data.hasGyro = true;   // Set false if gyro not available
    data.hasIMU = true;    // Set false if IMU not available
    data.hasAccel = true;  // Set false if accelerometer not available
    
    return data;
}
```

## Constant Velocity Handling Analysis

### Current Behavior:
The EKF uses velocity smoothing (α = 0.8) which creates:

**Advantages:**
- Smooth velocity estimates (reduces noise)
- Stable position integration
- Natural inertia modeling

**Potential Issues:**
- **Lag during velocity changes**: 22ms time constant
- **Steady-state error during acceleration**: Temporary mismatch between encoder and state
- **Over-smoothing**: May miss rapid velocity changes

### Constant Velocity Performance:
- **Convergence time**: ~22ms to reach 95% of target velocity
- **Final accuracy**: Exactly matches encoder readings
- **Position integration**: Very accurate for constant motion

### Recommended Tuning for Different Scenarios:

1. **High-precision applications**: Reduce α to 0.6-0.7
2. **Noisy encoders**: Keep α at 0.8-0.9  
3. **Fast acceleration robots**: Use adaptive α based on acceleration

## Test Phases

### Phase 1: Stationary Test (2 minutes)
**Objective**: Verify ZUPT works correctly and position doesn't drift

**Procedure**:
1. Place robot on flat surface
2. Ensure wheels are not moving (encoders read 0)
3. Run EKF for 2 minutes
4. Check that x,y remain close to (0,0)

**Expected Results**:
- Position drift < 1cm over 2 minutes
- Velocities converge to ~0
- Covariance stabilizes

**Commands**:
```bash
# Compile and run
cd /path/to/jetson_codebase
mkdir -p build
cd build
cmake ..
make test_ekf_real_robot

# Run stationary test
./test_ekf_real_robot
```

### Phase 2: Forward Motion Test
**Objective**: Test motion model accuracy

**Procedure**:
1. Drive robot forward 1 meter at constant speed
2. Stop and measure actual distance traveled
3. Compare with EKF estimate

**Expected Results**:
- Position error < 5% of distance traveled
- Velocity estimates match encoder readings
- No significant lateral drift (y ≈ 0)

### Phase 2.5: Constant Velocity Test
**Objective**: Verify steady-state constant velocity handling

**Procedure**:
1. Accelerate robot to 0.5 m/s forward velocity
2. Maintain constant velocity for 30 seconds
3. Monitor velocity convergence and position accuracy

**Expected Results**:
- Velocity converges to encoder reading within 100ms
- Position error < 1% over 30 seconds
- Smooth velocity estimates (no oscillation)

**Analysis Commands**:
```python
# Check velocity convergence
data = pd.read_csv('ekf_test_log.csv')
constant_phase = data[data['test_phase'] == 'constant_velocity']

# Plot encoder vs EKF velocity
plt.plot(constant_phase['timestamp'], constant_phase['leftVel'], label='Left Encoder')
plt.plot(constant_phase['timestamp'], constant_phase['vx'], label='EKF vx')
plt.legend()
plt.title('Velocity Tracking During Constant Motion')
```

### Phase 3: Rotation Test  
**Objective**: Test angular motion and gyro integration

**Procedure**:
1. Rotate robot 360° in place
2. Check final orientation estimate
3. Verify position doesn't drift significantly

**Expected Results**:
- Final orientation close to initial (±5°)
- Position drift < 10cm during rotation
- Angular velocity tracking accurate

### Phase 4: Wheel Slip Test
**Objective**: Test wheel slip detection

**Procedure**:
1. Lift robot wheels off ground
2. Command wheel motion
3. Verify slip detection activates

**Expected Results**:
- Console shows "WHEEL SLIP/LIFT DETECTED"
- Position doesn't change despite encoder readings
- Uncertainty increases appropriately

### Phase 5: Mixed Motion Test
**Objective**: Test real-world scenarios

**Procedure**:
1. Drive robot in figure-8 pattern
2. Include stops and direction changes
3. Return to starting position

**Expected Results**:
- Final position within 20cm of start
- Smooth trajectory without jumps
- Reasonable velocity estimates

## Data Analysis

### 1. Real-time Monitoring
Watch console output for:
- State estimates (x, y, θ, vx, vy, ω)
- Covariance diagonal values
- Warning messages
- Test phase transitions

### 2. Log File Analysis
The test creates `ekf_test_log.csv` with columns:
- `timestamp`: Time since start
- `x,y,theta`: Position and orientation estimates
- `vx,vy,omega`: Velocity estimates  
- `leftVel,rightVel`: Encoder readings
- `gyroZ,imuYaw,accelX,accelY`: Sensor readings
- `P_xx,P_yy,P_theta,P_vx,P_vy,P_omega`: Covariance diagonal
- `test_phase`: Current test phase

### 3. Plotting (Python/MATLAB)
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
data = pd.read_csv('ekf_test_log.csv')

# Plot trajectory
plt.figure(figsize=(12, 8))
plt.subplot(2, 2, 1)
plt.plot(data['x'], data['y'])
plt.title('Robot Trajectory')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.axis('equal')

# Plot velocities
plt.subplot(2, 2, 2)
plt.plot(data['timestamp'], data['vx'], label='vx')
plt.plot(data['timestamp'], data['vy'], label='vy')
plt.title('Velocities')
plt.legend()

# Plot uncertainties
plt.subplot(2, 2, 3)
plt.plot(data['timestamp'], data['P_xx'], label='P_xx')
plt.plot(data['timestamp'], data['P_yy'], label='P_yy')
plt.title('Position Uncertainty')
plt.legend()

# Plot orientation
plt.subplot(2, 2, 4)
plt.plot(data['timestamp'], data['theta'] * 180/3.14159)
plt.title('Orientation (degrees)')

plt.tight_layout()
plt.show()
```

## Troubleshooting

### Common Issues:

**1. Large Position Drift**
- Check wheel radius/wheelbase measurements
- Verify encoder calibration
- Reduce process noise

**2. Jerky Motion Estimates**
- Increase velocity smoothing (α closer to 1.0)
- Check sensor timing/synchronization
- Verify measurement noise settings

**3. Slow Convergence**
- Increase Kalman gains (reduce measurement noise)
- Check initial covariance values
- Verify sensor data quality

**4. Wheel Slip False Positives**
- Adjust slip detection threshold (currently 0.5 m/s²)
- Check accelerometer calibration
- Verify coordinate frame alignment

**5. Constant Velocity Issues**
- **Slow convergence**: Reduce α (e.g., 0.6 instead of 0.8)
- **Velocity lag**: Consider adaptive smoothing based on acceleration
- **Over-smoothing**: Increase encoder measurement confidence (reduce R_encoder)
- **Position drift during constant motion**: Check for systematic encoder bias

### Adaptive Velocity Smoothing (Advanced):
For better constant velocity handling, consider implementing adaptive α:

```cpp
// In motionModel()
double accel_magnitude = std::abs(v_robot - vx) / dt_;
double adaptive_alpha = accel_magnitude > 0.1 ? 0.5 : 0.8;  // Less smoothing during acceleration
new_state(3) = adaptive_alpha * vx + (1.0 - adaptive_alpha) * v_robot;
```

## Performance Metrics

### Good Performance Indicators:
- Position error < 2% of distance traveled
- Velocity estimates within 10% of encoder readings
- Covariance convergence within 30 seconds
- No filter divergence over long runs
- Successful slip detection when wheels lifted

### Warning Signs:
- Covariance growing without bound
- Position estimates jumping between updates
- Velocity estimates inconsistent with motion
- Frequent false slip detections

## Next Steps

After basic testing, consider:
1. Integration with path planning
2. Multi-robot testing
3. Different terrain types
4. Sensor failure scenarios
5. Real-time performance optimization
