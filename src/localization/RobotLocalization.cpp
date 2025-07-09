#include "RobotLocalization.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>

RobotLocalization::RobotLocalization(const std::string& serialPort, bool enableLogging) 
    : dt_(DEFAULT_DT), enableLogging_(enableLogging) {
    
    // Initialize EKF with robot parameters - TUNED FOR BETTER PERFORMANCE
    ekf_ = std::make_unique<SensorFusion>(
        RobotUtils::WHEEL_RADIUS,     // wheelRadius
        RobotUtils::WHEEL_BASE,       // wheelBase  
        dt_,                          // dt
        1e-2,                         // processNoisePos (increased - less trust in prediction)
        1e-3,                         // processNoiseAng (increased - less trust in angular prediction)
        1e-2,                         // processNoiseVel (increased - less trust in velocity prediction)
        1e-3,                         // gyroNoise (increased - more realistic gyro noise)
        1e-2,                         // imuNoise (increased - more realistic IMU noise)
        1e-1                          // accelNoise (increased - less trust in accelerometer)
    );
    
    // Initialize serial communication
    try {
        serial_ = std::make_unique<Serial_Com>(serialPort, 115200);
        std::cout << "Serial communication initialized on " << serialPort << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize serial: " << e.what() << std::endl;
        throw;
    }
    
    // Initialize LIDAR (optional)
    try {
        lidar_ = std::make_unique<LidarHandler>();
        std::cout << "LIDAR initialized" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "LIDAR initialization failed: " << e.what() << std::endl;
        lidar_.reset();  // Continue without LIDAR
    }
    
    // Initialize timing
    lastUpdate_ = std::chrono::steady_clock::now();
    
    // Setup logging
    if (enableLogging_) {
        std::string timestamp = getCurrentTimestamp();
        logFile_.open("robot_pose_" + timestamp + ".csv");
        logFile_ << "timestamp,x,y,theta,vx,vy,omega,leftVel,rightVel,imuYaw,gyroZ,accelX,accelY\n";
    }
}

RobotLocalization::~RobotLocalization() {
    if (logFile_.is_open()) {
        logFile_.close();
    }
}

void RobotLocalization::spin(CommandPacket &cmd) {
    std::cout << "Starting robot localization loop..." << std::endl;
    
    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        dt_ = std::chrono::duration<double>(currentTime - lastUpdate_).count();
        lastUpdate_ = currentTime;
        
        // Limit dt to reasonable bounds
        if (dt_ > 0.1) dt_ = DEFAULT_DT;  // Reset if too large
        if (dt_ < 0.001) dt_ = 0.001;    // Minimum dt
        
        // Process serial data
        serial_->spinOnce(cmd);
        
        // Update global variables (same as basic mode)
        extern SensorPacket g_sensor;
        extern MotionDebugPacket g_debug;
        g_sensor = serial_->getSensor();
        g_debug = serial_->getDebug();
        
        // Perform EKF prediction and updates using global sensor data
        updateEKF(g_sensor);
        
        // Print status every 100 iterations (about 1Hz at 100Hz)
        static int counter = 0;
        if (++counter >= 100) {
            printStatus(g_sensor);
            counter = 0;
        }
        
        // Log data
        if (enableLogging_) {
            logData(g_sensor);
        }
        
        // Sleep to maintain approximate frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ~100Hz to match I2C
    }
}

void RobotLocalization::sendCommand(CommandPacket& cmd) {
    serial_->sendCommand(cmd);
}

std::array<double, 3> RobotLocalization::getPose() const {
    return ekf_->getPose();
}

std::array<double, 3> RobotLocalization::getVelocities() const {
    return ekf_->getVelocities();
}

void RobotLocalization::updateEKF(const SensorPacket& sensor) {
    // 1. Calculate wheel velocities
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
    
    // 2. Determine if ZUPT should be active
    double accelMagnitude = std::sqrt(sensor.accelX * sensor.accelX + sensor.accelY * sensor.accelY);
    double maxAccelThreshold = 0.5; // m/s² - reject above this (likely noise or significant motion)
    bool lowMotion = (std::abs(leftVel) < 0.05 && std::abs(rightVel) < 0.05);
    bool reasonableAccel = (accelMagnitude >= 0.0 && accelMagnitude < maxAccelThreshold);
    bool zuptCondition = (lowMotion && reasonableAccel && accelMagnitude < 0.05); // Use consistent 0.05 threshold
    
    // 3. Prediction step using ZUPT-aware prediction
    ekf_->predictWithZUPT(leftVel, rightVel, zuptCondition);
    
    // 4. Validate encoder velocities for reasonable bounds
    double maxReasonableVel = 2.0;  // 2 m/s max reasonable velocity
    bool encodersReliable = (std::abs(leftVel) < maxReasonableVel && std::abs(rightVel) < maxReasonableVel);
    
    // 5. Update with IMU yaw (if available and reasonable)
    if (sensor.yaw != 0.0f && std::abs(sensor.yaw) < 360.0f) {
        double yawRad = RobotUtils::degToRad(sensor.yaw);
        ekf_->updateWithIMU(yawRad);
    }
    
    // 6. Update with gyroscope Z (if reasonable)
    double maxReasonableOmega = 10.0;  // 10 rad/s max reasonable angular velocity
    if (sensor.gyroZ != 0.0f && std::abs(sensor.gyroZ) < maxReasonableOmega) {
        ekf_->updateWithGyro(sensor.gyroZ);
    }
    
    // 7. Zero Velocity Update (ZUPT) using accelerometer
    if (zuptCondition) {
        // The SensorFusion::updateWithAccelerometer method implements ZUPT:
        // - It detects when accel magnitude < 0.05 m/s² (stationary) 
        // - Then it forces the velocity states (vx, vy) to zero
        // - This is proper ZUPT implementation for your sensor range
        ekf_->updateWithAccelerometer(sensor.accelX, sensor.accelY);
    }
    
    // 8. Update with encoder velocities as measurement (only if reasonable and not in ZUPT mode)
    if (encodersReliable && !zuptCondition && (std::abs(leftVel) > 0.01 || std::abs(rightVel) > 0.01)) {
        ekf_->updateWithEncoders(leftVel, rightVel);
    }
}

void RobotLocalization::printStatus(const SensorPacket& sensor) {
    auto pose = ekf_->getPose();
    auto velocities = ekf_->getVelocities();
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
    
    // Calculate diagnostic values
    double accelMagnitude = std::sqrt(sensor.accelX * sensor.accelX + sensor.accelY * sensor.accelY);
    double avgWheelVel = (leftVel + rightVel) / 2.0;
    bool encodersReliable = (std::abs(leftVel) < 2.0 && std::abs(rightVel) < 2.0);
    bool lowMotion = (std::abs(leftVel) < 0.05 && std::abs(rightVel) < 0.05);
    bool zuptActive = (lowMotion && accelMagnitude >= 0.0 && accelMagnitude < 0.05); // Consistent ZUPT threshold
    
    std::cout << "\n========== ROBOT STATUS ==========\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "POSE: x=" << pose[0] << "m, y=" << pose[1] << "m, θ=" 
              << RobotUtils::radToDeg(pose[2]) << "°\n";
    std::cout << "VELOCITIES: vx=" << velocities[0] << "m/s, vy=" << velocities[1] 
              << "m/s, ω=" << RobotUtils::radToDeg(velocities[2]) << "°/s\n";
    std::cout << "ENCODERS: L=" << leftVel << "m/s, R=" << rightVel << "m/s (avg=" << avgWheelVel << ")\n";
    std::cout << "SENSORS: yaw=" << sensor.yaw << "°, gyroZ=" << sensor.gyroZ << "rad/s\n";
    std::cout << "ACCEL: X=" << sensor.accelX << "m/s², Y=" << sensor.accelY << "m/s² (mag=" << accelMagnitude << ")\n";
    std::cout << "DIAGNOSTICS: dt=" << dt_ << "s, Battery=" << sensor.vbat1 << "mV\n";
    std::cout << "STATUS: Encoders=" << (encodersReliable ? "OK" : "UNRELIABLE") 
              << ", Motion=" << (lowMotion ? "LOW" : "HIGH") 
              << ", ZUPT=" << (zuptActive ? "ACTIVE" : "INACTIVE") << "\n";
    std::cout << "=================================\n" << std::endl;
}

void RobotLocalization::logData(const SensorPacket& sensor) {
    if (!logFile_.is_open()) return;
    
    auto pose = ekf_->getPose();
    auto velocities = ekf_->getVelocities();
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
    
    // Use same timestamp format as LiDAR: 32-bit Unix timestamp in milliseconds
    auto now = std::chrono::system_clock::now();
    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    uint32_t timestamp = static_cast<uint32_t>(timestamp_ms & 0xFFFFFFFF);  // Truncate to 32-bit like LiDAR
    
    logFile_ << timestamp << ","
             << pose[0] << "," << pose[1] << "," << pose[2] << ","
             << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
             << leftVel << "," << rightVel << ","
             << sensor.yaw << "," << sensor.gyroZ << ","
             << sensor.accelX << "," << sensor.accelY << "\n";
    logFile_.flush();
}

std::string RobotLocalization::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}
