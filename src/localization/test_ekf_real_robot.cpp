#include "SensorFusion.hpp"
#include "robot_utils.hpp"
#include "serial_com.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <iomanip>
#include <memory>

class RobotEKFTester {
private:
    SensorFusion ekf_;
    std::unique_ptr<Serial_Com> serial_;
    std::ofstream logFile_;
    std::chrono::steady_clock::time_point startTime_;
    double dt_;
    
public:
    RobotEKFTester(const std::string& serialPort = "/dev/ttyUSB0", 
                   double wheelRadius = 0.05, double wheelBase = 0.3) 
        : ekf_(wheelRadius, wheelBase, 0.01,  // 100Hz update rate (10ms)
               1e-4,  // position noise
               1e-5,  // angular noise  
               1e-3,  // velocity noise
               1e-4,  // gyro noise
               1e-3,  // IMU noise
               1e-2), // accel noise
          logFile_("ekf_test_log.csv"),
          startTime_(std::chrono::steady_clock::now()),
          dt_(0.01) {
        
        // Initialize serial communication
        try {
            serial_ = std::make_unique<Serial_Com>(serialPort);
            std::cout << "Serial connection established on " << serialPort << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to connect to serial port " << serialPort 
                      << ": " << e.what() << std::endl;
            std::cerr << "Available ports:" << std::endl;
            auto ports = Serial_Com::getAvailablePorts();
            for (const auto& port : ports) {
                std::cerr << "  " << port << std::endl;
            }
            throw;
        }
        
        // Write CSV header
        logFile_ << "timestamp,x,y,theta,vx,vy,omega,"
                 << "leftVel,rightVel,gyroZ,imuYaw,accelX,accelY,"
                 << "encL,encR,dEncL,dEncR,yaw,roll,pitch,"
                 << "P_xx,P_yy,P_theta,P_vx,P_vy,P_omega,"
                 << "test_phase" << std::endl;
        
        std::cout << "EKF Robot Tester Initialized" << std::endl;
        std::cout << "Using robot parameters:" << std::endl;
        std::cout << "  Wheel radius: " << wheelRadius << " m" << std::endl;
        std::cout << "  Wheelbase: " << wheelBase << " m" << std::endl;
        std::cout << "  Update rate: " << 1.0/dt_ << " Hz" << std::endl;
        std::cout << "Log file: ekf_test_log.csv" << std::endl;
    }
    
    ~RobotEKFTester() {
        logFile_.close();
        std::cout << "EKF test completed. Serial connection closed." << std::endl;
    }
    
    void runTest(const SensorPacket& sensor, const std::string& testPhase = "") {
        // Convert sensor data to EKF inputs
        auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);
        double gyroZ = RobotUtils::degToRad(sensor.gyroZ);  // Convert to rad/s
        double imuYaw = RobotUtils::degToRad(sensor.yaw);   // Convert to rad
        double accelX = sensor.accelX;  // Already in m/s²
        double accelY = sensor.accelY;  // Already in m/s²
        
        // Check sensor availability (you may need to adjust these based on your robot)
        bool hasGyro = (sensor.valid && std::abs(sensor.gyroZ) < 1000.0);  // Reasonable gyro reading
        bool hasIMU = (sensor.valid && std::abs(sensor.yaw) < 360.0);      // Valid yaw reading
        bool hasAccel = (sensor.valid && std::abs(sensor.accelX) < 50.0 && std::abs(sensor.accelY) < 50.0);  // Reasonable accel
        bool hasEncoders = sensor.valid;  // Assume encoders always available if packet is valid
        
        // Run sensor fusion step
        ekf_.sensorFusionStep(leftVel, rightVel,
                             gyroZ, hasGyro,
                             imuYaw, hasIMU,
                             accelX, accelY, hasAccel,
                             hasEncoders);
        
        // Get current state
        auto pose = ekf_.getPose();
        auto velocities = ekf_.getVelocities();
        auto covariance = ekf_.getCovariance();
        
        // Calculate elapsed time
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime_).count();
        
        // Log data with more sensor information
        logFile_ << std::fixed << std::setprecision(6)
                 << elapsed << ","
                 << pose[0] << "," << pose[1] << "," << pose[2] << ","
                 << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
                 << leftVel << "," << rightVel << ","
                 << gyroZ << "," << imuYaw << ","
                 << accelX << "," << accelY << ","
                 << sensor.encL << "," << sensor.encR << ","
                 << sensor.dEncL << "," << sensor.dEncR << ","
                 << sensor.yaw << "," << sensor.roll << "," << sensor.pitch << ","
                 << covariance(0,0) << "," << covariance(1,1) << "," << covariance(2,2) << ","
                 << covariance(3,3) << "," << covariance(4,4) << "," << covariance(5,5) << ","
                 << testPhase << std::endl;
        
        // Print state every 10th iteration (1 second at 10Hz display)
        static int counter = 0;
        if (++counter % 10 == 0) {
            std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
            std::cout << "║                    ROBOT EKF TESTING                      ║" << std::endl;
            std::cout << "║              Real Hardware Validation                     ║" << std::endl;
            std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
            
            std::cout << "\nCurrent Test Phase: " << testPhase << std::endl;
            std::cout << "Time: " << std::fixed << std::setprecision(2) << elapsed << "s" << std::endl;
            
            // EKF State
            std::cout << "\n--- EKF State ---" << std::endl;
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "Position: (" << pose[0] << ", " << pose[1] << ") m" << std::endl;
            std::cout << "Heading:  " << pose[2] << " rad (" << pose[2]*180.0/M_PI << "°)" << std::endl;
            std::cout << "Velocity: (" << velocities[0] << ", " << velocities[1] << ") m/s" << std::endl;
            std::cout << "Ang. Vel: " << velocities[2] << " rad/s (" << velocities[2]*180.0/M_PI << "°/s)" << std::endl;
            
            // Raw sensor data
            std::cout << "\n--- Raw Sensor Data ---" << std::endl;
            std::cout << "Encoders: L=" << std::setprecision(2) << leftVel << " R=" << rightVel << " m/s" << std::endl;
            std::cout << "         (Δ: L=" << sensor.dEncL << " R=" << sensor.dEncR << " ticks)" << std::endl;
            std::cout << "IMU:      θ=" << std::setprecision(1) << sensor.yaw << "° roll=" << sensor.roll << "° pitch=" << sensor.pitch << "°" << std::endl;
            std::cout << "Gyro:     ωz=" << std::setprecision(2) << sensor.gyroZ << "°/s" << std::endl;
            std::cout << "Accel:    (" << sensor.accelX << ", " << sensor.accelY << ") m/s²" << std::endl;
            
            // Status flags
            std::cout << "\n--- Status ---" << std::endl;
            std::cout << "Packet Valid: " << (sensor.valid ? "YES" : "NO") << std::endl;
            std::cout << "Sensors: Gyro=" << (hasGyro ? "OK" : "NO") 
                      << " IMU=" << (hasIMU ? "OK" : "NO") 
                      << " Accel=" << (hasAccel ? "OK" : "NO") 
                      << " Enc=" << (hasEncoders ? "OK" : "NO") << std::endl;
            
            // Check for slip detection or other warnings
            if (detectSlipConditions(leftVel, rightVel, accelX, accelY)) {
                std::cout << "⚠️  WHEEL SLIP/LIFT DETECTED!" << std::endl;
            }
        }
    }
    
    // Helper function to detect slip conditions (similar to EKF internal logic)
    bool detectSlipConditions(double leftVel, double rightVel, double accelX, double accelY) const {
        double v_robot = (leftVel + rightVel) / 2.0;
        double accel_magnitude = std::sqrt(accelX*accelX + accelY*accelY);
        bool unreasonable_vel = (std::abs(leftVel) > 3.0 || std::abs(rightVel) > 3.0);
        bool high_accel_error = (accel_magnitude > 2.0 && std::abs(v_robot) < 0.1);
        return unreasonable_vel || high_accel_error;
    }
    
    void printDiagnostics() {
        std::cout << "\n=== FULL DIAGNOSTICS ===" << std::endl;
        ekf_.printDiagnostics();
    }
    
    void resetTest() {
        ekf_.resetState();
        std::cout << "EKF State Reset" << std::endl;
    }
    
    SensorPacket readSensors() {
        // Create a dummy command packet for spinOnce
        CommandPacket cmd;
        cmd.mode = AUTONOMOUS;
        cmd.cmdStatus = CMD_EMPTY;
        
        // Read latest sensor data from robot
        serial_->spinOnce(cmd);
        return serial_->getSensor();
    }
    
    bool isConnected() const {
        return serial_ && serial_->isConnected();
    }
    
    void testCommunication() {
        if (serial_) {
            serial_->testCommunication();
        }
    }
};

int main(int argc, char* argv[]) {
    std::string serialPort = "/dev/ttyUSB0";  // Default port
    
    // Allow command line port specification
    if (argc > 1) {
        serialPort = argv[1];
    }
    
    std::cout << "╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    ROBOT EKF TESTING                      ║" << std::endl;
    std::cout << "║              Real Hardware Validation                     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    try {
        // Use actual robot parameters - adjust these to match your robot!
        double wheelRadius = RobotUtils::WHEEL_RADIUS;  // Use from RobotUtils
        double wheelBase = RobotUtils::WHEEL_BASE;      // Use from RobotUtils
        
        RobotEKFTester tester(serialPort, wheelRadius, wheelBase);
        
        // Test communication first
        if (!tester.isConnected()) {
            std::cerr << "❌ Failed to connect to robot on " << serialPort << std::endl;
            return 1;
        }
        
        std::cout << "✅ Connected to robot on " << serialPort << std::endl;
        std::cout << "Testing communication..." << std::endl;
        tester.testCommunication();
        
        std::cout << "\nStarting EKF testing sequence..." << std::endl;
        std::cout << "Press Ctrl+C to stop at any time" << std::endl;
        std::cout << "Results will be logged to: ekf_test_log.csv" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        // Test sequence - each phase runs for specific duration
        const std::vector<std::pair<std::string, int>> testPhases = {
            {"initialization", 30},      // 3 seconds to settle
            {"stationary", 200},         // 20 seconds stationary test
            {"forward_motion", 300},     // 30 seconds forward motion
            {"constant_velocity", 300},  // 30 seconds constant velocity  
            {"turning", 200},            // 20 seconds turning
            {"stationary_again", 100},   // 10 seconds stationary again
            {"mixed_motion", 400}        // 40 seconds mixed motion
        };
        
        int totalIterations = 0;
        for (const auto& phase : testPhases) {
            totalIterations += phase.second;
        }
        
        std::cout << "Total test duration: " << totalIterations * 0.01 << " seconds" << std::endl;
        std::cout << "Test phases: " << testPhases.size() << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        int iteration = 0;
        for (const auto& [phaseName, duration] : testPhases) {
            std::cout << "\n🔄 Starting phase: " << phaseName 
                      << " (duration: " << duration * 0.01 << "s)" << std::endl;
            
            for (int i = 0; i < duration; ++i) {
                SensorPacket sensor = tester.readSensors();
                
                // Only process if we have valid data
                if (sensor.valid) {
                    tester.runTest(sensor, phaseName);
                } else {
                    std::cout << "⚠️  Invalid sensor packet received" << std::endl;
                }
                
                // Sleep for 10ms (100Hz)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                iteration++;
            }
            
            std::cout << "✅ Phase " << phaseName << " completed" << std::endl;
        }
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "🎉 All test phases completed successfully!" << std::endl;
        
        tester.printDiagnostics();
        
        std::cout << "\n📊 Test Results Summary:" << std::endl;
        std::cout << "- Total iterations: " << iteration << std::endl;
        std::cout << "- Total time: " << iteration * 0.01 << " seconds" << std::endl;
        std::cout << "- Log file: ekf_test_log.csv" << std::endl;
        std::cout << "- Use Python/Excel to analyze the CSV data" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        std::cerr << "\nTroubleshooting:" << std::endl;
        std::cerr << "1. Check serial port connection: " << serialPort << std::endl;
        std::cerr << "2. Verify robot is powered on and connected" << std::endl;
        std::cerr << "3. Check available ports with: ls /dev/tty*" << std::endl;
        std::cerr << "4. Try different port: ./test_ekf_real_robot /dev/ttyACM0" << std::endl;
        return 1;
    }
    
    return 0;
}
