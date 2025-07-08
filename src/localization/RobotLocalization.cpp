#include "RobotLocalization.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>

RobotLocalization::RobotLocalization(const std::string& serialPort, bool enableLogging) 
    : dt_(DEFAULT_DT), enableLogging_(enableLogging) {
    
    // Initialize EKF with robot parameters
    ekf_ = std::make_unique<SensorFusion>(
        RobotUtils::WHEEL_RADIUS,     // wheelRadius
        RobotUtils::WHEEL_BASE,       // wheelBase  
        dt_,                          // dt
        1e-4,                         // processNoisePos
        1e-5,                         // processNoiseAng
        1e-3,                         // processNoiseVel
        1e-4,                         // gyroNoise
        1e-3,                         // imuNoise
        1e-2                          // accelNoise
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
    // 1. Prediction step using encoder data
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
    ekf_->predict(leftVel, rightVel);
    
    // 2. Update with IMU yaw (if available and reasonable)
    if (sensor.yaw != 0.0f) {
        double yawRad = RobotUtils::degToRad(sensor.yaw);
        ekf_->updateWithIMU(yawRad);
    }
    
    // 3. Update with gyroscope Z (if available)
    if (sensor.gyroZ != 0.0f) {
        ekf_->updateWithGyro(sensor.gyroZ);  // gyroZ already in rad/s from BNO055
    }
    
    // 4. Update with 2D accelerometer for Zero Velocity Update (if available)
    if (sensor.accelX != 0.0f || sensor.accelY != 0.0f) {
        ekf_->updateWithAccelerometer(sensor.accelX, sensor.accelY);
    }
    
    // 5. Update with encoder velocities as measurement
    if (leftVel != 0.0 || rightVel != 0.0) {
        ekf_->updateWithEncoders(leftVel, rightVel);
    }
}

void RobotLocalization::printStatus(const SensorPacket& sensor) {
    auto pose = ekf_->getPose();
    auto velocities = ekf_->getVelocities();
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
    
    std::cout << "\n========== ROBOT STATUS ==========\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "POSE: x=" << pose[0] << "m, y=" << pose[1] << "m, θ=" 
              << RobotUtils::radToDeg(pose[2]) << "°\n";
    std::cout << "VELOCITIES: vx=" << velocities[0] << "m/s, vy=" << velocities[1] 
              << "m/s, ω=" << RobotUtils::radToDeg(velocities[2]) << "°/s\n";
    std::cout << "ENCODERS: L=" << leftVel << "m/s, R=" << rightVel << "m/s\n";
    std::cout << "SENSORS: yaw=" << sensor.yaw << "°, gyroZ=" << sensor.gyroZ << "rad/s\n";
    std::cout << "ACCEL: X=" << sensor.accelX << "m/s², Y=" << sensor.accelY << "m/s²\n";
    std::cout << "dt=" << dt_ << "s, Battery: " << sensor.vbat1 << "mV\n";
    std::cout << "=================================\n" << std::endl;
}

void RobotLocalization::logData(const SensorPacket& sensor) {
    if (!logFile_.is_open()) return;
    
    auto pose = ekf_->getPose();
    auto velocities = ekf_->getVelocities();
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
    
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
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
