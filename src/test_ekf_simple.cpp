#include "localization/SensorFusion.hpp"
#include "localization/robot_utils.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

// Simple test data structure
struct TestSensorData {
    double leftVel, rightVel;
    double imuYaw;       // degrees
    double gyroZ;        // rad/s
    double accelX, accelY; // m/s²
    double dt;
    std::string description;
};

void printTestHeader(const std::string& testName) {
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << testName << "\n";
    std::cout << std::string(50, '=') << "\n";
}

void printEKFState(const SensorFusion& ekf, double time, const std::string& context = "") {
    auto pose = ekf.getPose();
    auto velocities = ekf.getVelocities();
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "t=" << std::setw(5) << time << "s ";
    if (!context.empty()) std::cout << "[" << context << "] ";
    std::cout << "Pose: (" << std::setw(6) << pose[0] << "," << std::setw(6) << pose[1] 
              << "," << std::setw(6) << pose[2]*180/M_PI << "°) ";
    std::cout << "Vel: (" << std::setw(5) << velocities[0] << "," << std::setw(5) << velocities[1] 
              << "," << std::setw(5) << velocities[2]*180/M_PI << "°/s)\n";
}

void runEKFStep(SensorFusion& ekf, const TestSensorData& data, bool verbose = false) {
    if (verbose) {
        std::cout << "  Input: L=" << data.leftVel << " R=" << data.rightVel 
                  << " accel=(" << data.accelX << "," << data.accelY << ") "
                  << data.description << "\n";
    }
    
    // Run EKF prediction
    ekf.predict(data.leftVel, data.rightVel);
    
    // Run updates based on available data
    if (data.imuYaw != 0.0 && std::abs(data.imuYaw) < 360.0) {
        ekf.updateWithIMU(data.imuYaw * M_PI / 180.0);
    }
    
    if (data.gyroZ != 0.0 && std::abs(data.gyroZ) < 10.0) {
        ekf.updateWithGyro(data.gyroZ);
    }
    
    // Always try accelerometer update (ZUPT logic is inside)
    ekf.updateWithAccelerometer(data.accelX, data.accelY);
    
    // Only use encoders if they show reasonable velocities
    if (std::abs(data.leftVel) < 2.0 && std::abs(data.rightVel) < 2.0) {
        if (std::abs(data.leftVel) > 0.01 || std::abs(data.rightVel) > 0.01) {
            ekf.updateWithEncoders(data.leftVel, data.rightVel);
        }
    }
}

void test1_FreeSpinningWheels() {
    printTestHeader("TEST 1: FREE SPINNING WHEELS");
    std::cout << "Problem: Encoders show 1m/s for 2 seconds, but robot is stationary\n";
    std::cout << "Expected: Position should stay near (0,0) due to ZUPT\n\n";
    
    SensorFusion ekf(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    // Simulate 200 steps (2 seconds at 100Hz)
    for (int i = 0; i < 200; i++) {
        TestSensorData data;
        data.leftVel = 1.0;      // Wheels spinning at 1 m/s
        data.rightVel = 1.0;     // Both wheels same speed
        data.imuYaw = 0.0;       // No rotation
        data.gyroZ = 0.0;        // No angular velocity
        data.accelX = 0.01;      // Very low acceleration (stationary)
        data.accelY = 0.01;      // Very low acceleration (stationary)
        data.dt = 0.01;
        data.description = "free spinning";
        
        runEKFStep(ekf, data);
        
        // Print every 50 steps (0.5 seconds)
        if (i % 50 == 0) {
            printEKFState(ekf, i * 0.01, "free spin");
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "\nFINAL RESULT:\n";
    std::cout << "Position: (" << final_pose[0] << ", " << final_pose[1] << ") meters\n";
    std::cout << "SUCCESS CRITERIA: |x| < 0.1 AND |y| < 0.1\n";
    
    bool success = (std::abs(final_pose[0]) < 0.1 && std::abs(final_pose[1]) < 0.1);
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << "\n";
}

void test2_NormalMovement() {
    printTestHeader("TEST 2: NORMAL MOVEMENT");
    std::cout << "Command: 0.5 m/s forward for 4 seconds\n";
    std::cout << "Expected: Position should reach approximately (2.0, 0.0)\n\n";
    
    SensorFusion ekf(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    // Simulate 400 steps (4 seconds at 100Hz)
    for (int i = 0; i < 400; i++) {
        TestSensorData data;
        data.leftVel = 0.5;      // 0.5 m/s forward
        data.rightVel = 0.5;     // 0.5 m/s forward
        data.imuYaw = 0.0;       // No rotation
        data.gyroZ = 0.0;        // No angular velocity
        data.accelX = 0.1;       // Some acceleration (moving)
        data.accelY = 0.05;      // Small Y acceleration
        data.dt = 0.01;
        data.description = "normal forward";
        
        runEKFStep(ekf, data);
        
        // Print every 100 steps (1 second)
        if (i % 100 == 0) {
            printEKFState(ekf, i * 0.01, "forward");
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "\nFINAL RESULT:\n";
    std::cout << "Position: (" << final_pose[0] << ", " << final_pose[1] << ") meters\n";
    std::cout << "SUCCESS CRITERIA: 1.8 < x < 2.2 AND |y| < 0.2\n";
    
    bool success = (final_pose[0] > 1.8 && final_pose[0] < 2.2 && std::abs(final_pose[1]) < 0.2);
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << "\n";
}

void test3_StopAndGo() {
    printTestHeader("TEST 3: STOP AND GO");
    std::cout << "Scenario: Move 1s, stop 1s, move 1s, stop 1s\n";
    std::cout << "Expected: Velocity should drop to zero during stops\n\n";
    
    SensorFusion ekf(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    // Simulate 400 steps (4 seconds)
    for (int i = 0; i < 400; i++) {
        double t = i * 0.01;
        bool moving = (fmod(t, 2.0) < 1.0);  // Move for 1s, stop for 1s
        
        TestSensorData data;
        data.leftVel = moving ? 0.5 : 0.0;
        data.rightVel = moving ? 0.5 : 0.0;
        data.imuYaw = 0.0;
        data.gyroZ = 0.0;
        data.accelX = moving ? 0.1 : 0.005;  // Low accel when stopped
        data.accelY = moving ? 0.05 : 0.005;
        data.dt = 0.01;
        data.description = moving ? "moving" : "stopped";
        
        runEKFStep(ekf, data);
        
        // Print at phase transitions
        if (i % 100 == 0 || i % 100 == 99) {
            printEKFState(ekf, t, data.description);
        }
    }
    
    auto final_velocities = ekf.getVelocities();
    std::cout << "\nFINAL RESULT:\n";
    std::cout << "Final velocity: (" << final_velocities[0] << ", " << final_velocities[1] << ") m/s\n";
    std::cout << "SUCCESS CRITERIA: |vx| < 0.1 AND |vy| < 0.1 (should be stopped)\n";
    
    bool success = (std::abs(final_velocities[0]) < 0.1 && std::abs(final_velocities[1]) < 0.1);
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << "\n";
}

void test4_ZUPTEffectiveness() {
    printTestHeader("TEST 4: ZUPT EFFECTIVENESS");
    std::cout << "Compare: High accel (no ZUPT) vs Low accel (ZUPT active)\n\n";
    
    // Test A: High acceleration (ZUPT should NOT trigger)
    std::cout << "4A: High acceleration (accel = 0.5 m/s²) - ZUPT should be INACTIVE\n";
    SensorFusion ekf_a(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    for (int i = 0; i < 100; i++) {
        TestSensorData data;
        data.leftVel = 0.5; data.rightVel = 0.5;
        data.imuYaw = 0.0; data.gyroZ = 0.0;
        data.accelX = 0.5; data.accelY = 0.3;  // High acceleration
        data.dt = 0.01;
        runEKFStep(ekf_a, data);
    }
    auto pose_a = ekf_a.getPose();
    std::cout << "Result: Position = (" << pose_a[0] << ", " << pose_a[1] << ")\n";
    
    // Test B: Low acceleration (ZUPT should trigger)
    std::cout << "\n4B: Low acceleration (accel = 0.01 m/s²) - ZUPT should be ACTIVE\n";
    SensorFusion ekf_b(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    for (int i = 0; i < 100; i++) {
        TestSensorData data;
        data.leftVel = 0.5; data.rightVel = 0.5;  // Same encoder readings
        data.imuYaw = 0.0; data.gyroZ = 0.0;
        data.accelX = 0.01; data.accelY = 0.01;  // Low acceleration
        data.dt = 0.01;
        runEKFStep(ekf_b, data);
    }
    auto pose_b = ekf_b.getPose();
    std::cout << "Result: Position = (" << pose_b[0] << ", " << pose_b[1] << ")\n";
    
    std::cout << "\nCOMPARISON:\n";
    std::cout << "High accel distance: " << sqrt(pose_a[0]*pose_a[0] + pose_a[1]*pose_a[1]) << "m\n";
    std::cout << "Low accel distance:  " << sqrt(pose_b[0]*pose_b[0] + pose_b[1]*pose_b[1]) << "m\n";
    std::cout << "SUCCESS: Low accel distance should be much smaller (ZUPT working)\n";
    
    double ratio = sqrt(pose_b[0]*pose_b[0] + pose_b[1]*pose_b[1]) / sqrt(pose_a[0]*pose_a[0] + pose_a[1]*pose_a[1]);
    bool success = (ratio < 0.5);  // ZUPT should reduce distance by at least 50%
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << " (ratio = " << ratio << ")\n";
}

void saveTestResults() {
    // Create a simple CSV for analysis
    std::ofstream file("ekf_test_results.csv");
    file << "test,step,time,x,y,theta,vx,vy,omega,left_vel,right_vel,accel_x,accel_y,notes\n";
    
    std::cout << "\nTest results framework created.\n";
    std::cout << "For detailed logging, modify the test functions to write to 'ekf_test_results.csv'\n";
    file.close();
}

int main() {
    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║           EKF VALIDATION TESTS               ║\n";
    std::cout << "║     Testing the issues you encountered      ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";
    
    // Run all tests
    test1_FreeSpinningWheels();
    test2_NormalMovement();
    test3_StopAndGo();
    test4_ZUPTEffectiveness();
    
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "ALL TESTS COMPLETED\n";
    std::cout << "Check the results above to verify EKF behavior\n";
    std::cout << std::string(60, '=') << "\n";
    
    saveTestResults();
    
    return 0;
}
