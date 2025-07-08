#include "SensorFusion.hpp"
#include "robot_utils.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

void testSensorFusion() {
    std::cout << "=== Testing SensorFusion EKF ===" << std::endl;
    
    // Initialize EKF with robot parameters for 100Hz operation
    SensorFusion ekf(
        RobotUtils::WHEEL_RADIUS,   // 0.1m
        RobotUtils::WHEEL_BASE,     // 0.5m  
        0.01                        // 100Hz (10ms) - matching I2C telemetry rate
    );
    
    std::cout << "Robot parameters:" << std::endl;
    std::cout << "  Wheel radius: " << RobotUtils::WHEEL_RADIUS << " m" << std::endl;
    std::cout << "  Wheel base: " << RobotUtils::WHEEL_BASE << " m" << std::endl;
    std::cout << "  Ticks per revolution: " << RobotUtils::TICKS_PER_REV << std::endl;
    std::cout << "  Meters per tick: " << RobotUtils::METERS_PER_TICK << std::endl;
    
    // Simulate robot moving forward at 0.5 m/s for 5 seconds at 100Hz
    std::cout << "\nSimulating forward motion at 0.5 m/s (100Hz telemetry)..." << std::endl;
    
    double leftVel = 0.5;   // m/s
    double rightVel = 0.5;  // m/s
    
    for (int i = 0; i < 500; i++) {  // 5 seconds at 100Hz
        // Prediction step (every iteration)
        ekf.predict(leftVel, rightVel);
        
        // All sensor updates every iteration (simulating 100Hz I2C packet)
        
        // IMU yaw measurement (should stay near 0 for straight motion)
        double imuYaw = 0.0 + (rand() % 100 - 50) * 0.001;  // Small noise
        ekf.updateWithIMU(imuYaw);
        
        // Encoder measurements  
        ekf.updateWithEncoders(leftVel, rightVel);
        
        // Gyroscope Z measurement
        double gyroZ = 0.0 + (rand() % 100 - 50) * 0.01;  // Small noise for straight motion
        ekf.updateWithGyro(gyroZ);
        
        // 2D Accelerometer measurements (small values for steady motion)
        double accelX = (rand() % 100 - 50) * 0.005;  // ±0.25 m/s²
        double accelY = (rand() % 100 - 50) * 0.005;  // ±0.25 m/s²
        ekf.updateWithAccelerometer(accelX, accelY);
        
        // Print status every 100 iterations (1 second)
        if (i % 100 == 0) {
            auto pose = ekf.getPose();
            auto velocities = ekf.getVelocities();
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "t=" << (i * 0.01) << "s: ";
            std::cout << "x=" << pose[0] << "m, y=" << pose[1] << "m, θ=" 
                      << (pose[2] * 180.0 / M_PI) << "°, ";
            std::cout << "vx=" << velocities[0] << "m/s, ω=" 
                      << (velocities[2] * 180.0 / M_PI) << "°/s" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // Faster simulation
    }
    
    // Test turning motion at 100Hz
    std::cout << "\nSimulating turning motion (100Hz telemetry)..." << std::endl;
    
    leftVel = 0.3;   // m/s
    rightVel = 0.7;  // m/s (turning left)
    
    for (int i = 0; i < 200; i++) {  // 2 seconds at 100Hz
        ekf.predict(leftVel, rightVel);
        
        // All sensor updates every iteration (100Hz I2C packet simulation)
        
        // IMU yaw for turning motion
        double expectedOmega = (rightVel - leftVel) / RobotUtils::WHEEL_BASE;
        double currentTime = i * 0.01;
        double imuYaw = expectedOmega * currentTime + (rand() % 100 - 50) * 0.01;
        ekf.updateWithIMU(imuYaw);
        
        // Encoder updates
        ekf.updateWithEncoders(leftVel, rightVel);
        
        // Gyroscope measurement (should match expected omega)
        double gyroZ = expectedOmega + (rand() % 100 - 50) * 0.02;
        ekf.updateWithGyro(gyroZ);
        
        // 2D accelerometer for turning motion (centripetal acceleration)
        double accelX = (rand() % 100 - 50) * 0.01;  // ±0.5 m/s²
        double accelY = (rand() % 100 - 50) * 0.01;  // ±0.5 m/s²
        ekf.updateWithAccelerometer(accelX, accelY);
        
        if (i % 50 == 0) {  // Print every 0.5 seconds
            auto pose = ekf.getPose();
            auto velocities = ekf.getVelocities();
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "t=" << (5.0 + i * 0.01) << "s: ";
            std::cout << "x=" << pose[0] << "m, y=" << pose[1] << "m, θ=" 
                      << (pose[2] * 180.0 / M_PI) << "°, ";
            std::cout << "vx=" << velocities[0] << "m/s, ω=" 
                      << (velocities[2] * 180.0 / M_PI) << "°/s" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    
    std::cout << "\nFinal EKF state:" << std::endl;
    ekf.printState();
    ekf.printCovariance();
}

void testEncoderConversions() {
    std::cout << "\n=== Testing Encoder Conversions ===" << std::endl;
    
    // Test encoder tick to distance conversion
    long ticks = 1000;
    double distance = RobotUtils::ticksToMeters(ticks);
    std::cout << ticks << " ticks = " << distance << " meters" << std::endl;
    
    // Test velocity conversion
    float tickDiff = 200.0f;  // ticks
    double dt = 0.01;         // 10ms (100Hz)
    double velocity = RobotUtils::tickDiffToVelocity(tickDiff, dt);
    std::cout << tickDiff << " tick difference in " << dt << "s = " 
              << velocity << " m/s" << std::endl;
    
    // Test angle conversions
    double angleDeg = 45.0;
    double angleRad = RobotUtils::degToRad(angleDeg);
    std::cout << angleDeg << "° = " << angleRad << " rad" << std::endl;
    
    double backToDeg = RobotUtils::radToDeg(angleRad);
    std::cout << angleRad << " rad = " << backToDeg << "°" << std::endl;
}

int main() {
    std::cout << "EKF Sensor Fusion Test Program" << std::endl;
    std::cout << "==============================\n" << std::endl;
    
    // Test encoder conversions
    testEncoderConversions();
    
    // Test sensor fusion
    testSensorFusion();
    
    std::cout << "\nTest completed successfully!" << std::endl;
    return 0;
}
