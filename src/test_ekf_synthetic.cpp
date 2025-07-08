#include "localization/SensorFusion.hpp"
#include "localization/robot_utils.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>

// Synthetic sensor data generator
class SyntheticDataGenerator {
private:
    double time_;
    double true_x_, true_y_, true_theta_;
    double true_vx_, true_vy_, true_omega_;
    double wheelRadius_, wheelBase_;
    
    // Noise parameters
    double encoder_noise_std_;
    double imu_noise_std_;
    double gyro_noise_std_;
    double accel_noise_std_;
    
    std::default_random_engine generator_;
    
public:
    SyntheticDataGenerator(double wheelRadius = 0.065, double wheelBase = 0.3) 
        : time_(0.0), true_x_(0.0), true_y_(0.0), true_theta_(0.0),
          true_vx_(0.0), true_vy_(0.0), true_omega_(0.0),
          wheelRadius_(wheelRadius), wheelBase_(wheelBase),
          encoder_noise_std_(0.01), imu_noise_std_(0.02), 
          gyro_noise_std_(0.005), accel_noise_std_(0.02) {
        generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
    }
    
    struct SensorData {
        double leftVel, rightVel;
        double imuYaw;
        double gyroZ;
        double accelX, accelY;
        double dt;
        
        // Ground truth (for comparison)
        double true_x, true_y, true_theta;
        double true_vx, true_vy, true_omega;
    };
    
    // Generate one step of synthetic data
    SensorData generateStep(double dt, double cmd_forward_vel = 0.0, double cmd_angular_vel = 0.0) {
        // Update ground truth state
        updateGroundTruth(dt, cmd_forward_vel, cmd_angular_vel);
        
        SensorData data;
        data.dt = dt;
        
        // Store ground truth
        data.true_x = true_x_;
        data.true_y = true_y_;
        data.true_theta = true_theta_;
        data.true_vx = true_vx_;
        data.true_vy = true_vy_;
        data.true_omega = true_omega_;
        
        // Generate noisy sensor measurements
        generateEncoderData(data);
        generateIMUData(data);
        generateGyroData(data);
        generateAccelData(data);
        
        time_ += dt;
        return data;
    }
    
private:
    void updateGroundTruth(double dt, double cmd_forward_vel, double cmd_angular_vel) {
        // Perfect differential drive model
        true_vx_ = cmd_forward_vel * cos(true_theta_);
        true_vy_ = cmd_forward_vel * sin(true_theta_);
        true_omega_ = cmd_angular_vel;
        
        // Integrate position
        true_x_ += true_vx_ * dt;
        true_y_ += true_vy_ * dt;
        true_theta_ += true_omega_ * dt;
        
        // Wrap angle
        while (true_theta_ > M_PI) true_theta_ -= 2.0 * M_PI;
        while (true_theta_ < -M_PI) true_theta_ += 2.0 * M_PI;
    }
    
    void generateEncoderData(SensorData& data) {
        // Convert robot velocities to wheel velocities
        double v_robot = sqrt(true_vx_*true_vx_ + true_vy_*true_vy_);
        double true_left_vel = v_robot - (true_omega_ * wheelBase_ / 2.0);
        double true_right_vel = v_robot + (true_omega_ * wheelBase_ / 2.0);
        
        // Add noise
        std::normal_distribution<double> noise(0.0, encoder_noise_std_);
        data.leftVel = true_left_vel + noise(generator_);
        data.rightVel = true_right_vel + noise(generator_);
    }
    
    void generateIMUData(SensorData& data) {
        std::normal_distribution<double> noise(0.0, imu_noise_std_);
        data.imuYaw = (true_theta_ * 180.0 / M_PI) + noise(generator_); // Convert to degrees
    }
    
    void generateGyroData(SensorData& data) {
        std::normal_distribution<double> noise(0.0, gyro_noise_std_);
        data.gyroZ = true_omega_ + noise(generator_);
    }
    
    void generateAccelData(SensorData& data) {
        // Calculate acceleration from velocity change
        static double last_vx = 0.0, last_vy = 0.0;
        double accel_x = (true_vx_ - last_vx) / data.dt;
        double accel_y = (true_vy_ - last_vy) / data.dt;
        last_vx = true_vx_;
        last_vy = true_vy_;
        
        // Add noise
        std::normal_distribution<double> noise(0.0, accel_noise_std_);
        data.accelX = accel_x + noise(generator_);
        data.accelY = accel_y + noise(generator_);
    }
};

// Test scenarios
void testScenario1_FreSpinningWheels(SensorFusion& ekf, std::ofstream& logFile) {
    std::cout << "\n=== TEST 1: FREE SPINNING WHEELS ===\n";
    std::cout << "Simulating wheels spinning while robot is stationary\n";
    
    SyntheticDataGenerator generator;
    
    for (int i = 0; i < 200; ++i) {  // 2 seconds at 100Hz
        // Command 1 m/s forward but robot doesn't move (wheels spinning freely)
        auto data = generator.generateStep(0.01, 0.0, 0.0);  // No actual motion
        
        // But encoders think they're moving (simulate free spinning)
        data.leftVel = 1.0;   // 1 m/s
        data.rightVel = 1.0;  // 1 m/s
        
        // Accelerometer shows robot is stationary
        data.accelX = 0.01;   // Very low acceleration
        data.accelY = 0.01;
        
        // Run EKF
        ekf.predict(data.leftVel, data.rightVel);
        if (data.imuYaw != 0.0) ekf.updateWithIMU(data.imuYaw * M_PI / 180.0);
        if (data.gyroZ != 0.0) ekf.updateWithGyro(data.gyroZ);
        ekf.updateWithAccelerometer(data.accelX, data.accelY);  // Should trigger ZUPT
        ekf.updateWithEncoders(data.leftVel, data.rightVel);
        
        auto pose = ekf.getPose();
        auto velocities = ekf.getVelocities();
        
        // Log data
        logFile << "TEST1," << i*0.01 << "," 
                << pose[0] << "," << pose[1] << "," << pose[2] << ","
                << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
                << data.leftVel << "," << data.rightVel << ","
                << data.accelX << "," << data.accelY << ","
                << data.true_x << "," << data.true_y << "," << data.true_theta << "\n";
        
        if (i % 50 == 0) {
            std::cout << "t=" << std::fixed << std::setprecision(2) << i*0.01 
                      << "s: EKF pos=(" << std::setprecision(3) << pose[0] << "," << pose[1] 
                      << ")m, True pos=(" << data.true_x << "," << data.true_y << ")m\n";
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "Final EKF position: (" << final_pose[0] << ", " << final_pose[1] << ")m\n";
    std::cout << "Expected: (0.0, 0.0)m - Should be close to zero due to ZUPT!\n";
}

void testScenario2_NormalMovement(SensorFusion& ekf, std::ofstream& logFile) {
    std::cout << "\n=== TEST 2: NORMAL FORWARD MOVEMENT ===\n";
    std::cout << "Simulating 2m forward movement at 0.5 m/s\n";
    
    // Reset EKF
    ekf.resetState();
    
    SyntheticDataGenerator generator;
    
    for (int i = 0; i < 400; ++i) {  // 4 seconds at 100Hz
        // Command 0.5 m/s forward
        auto data = generator.generateStep(0.01, 0.5, 0.0);
        
        // Run EKF
        ekf.predict(data.leftVel, data.rightVel);
        if (std::abs(data.imuYaw) > 0.1) ekf.updateWithIMU(data.imuYaw * M_PI / 180.0);
        if (std::abs(data.gyroZ) > 0.001) ekf.updateWithGyro(data.gyroZ);
        ekf.updateWithAccelerometer(data.accelX, data.accelY);
        ekf.updateWithEncoders(data.leftVel, data.rightVel);
        
        auto pose = ekf.getPose();
        auto velocities = ekf.getVelocities();
        
        // Log data
        logFile << "TEST2," << i*0.01 << "," 
                << pose[0] << "," << pose[1] << "," << pose[2] << ","
                << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
                << data.leftVel << "," << data.rightVel << ","
                << data.accelX << "," << data.accelY << ","
                << data.true_x << "," << data.true_y << "," << data.true_theta << "\n";
        
        if (i % 100 == 0) {
            std::cout << "t=" << std::fixed << std::setprecision(2) << i*0.01 
                      << "s: EKF pos=(" << std::setprecision(3) << pose[0] << "," << pose[1] 
                      << ")m, True pos=(" << data.true_x << "," << data.true_y << ")m\n";
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "Final EKF position: (" << final_pose[0] << ", " << final_pose[1] << ")m\n";
    std::cout << "Expected: ~(2.0, 0.0)m\n";
}

void testScenario3_CircularMovement(SensorFusion& ekf, std::ofstream& logFile) {
    std::cout << "\n=== TEST 3: CIRCULAR MOVEMENT ===\n";
    std::cout << "Simulating circular movement with 0.5 m/s forward and 0.5 rad/s turning\n";
    
    // Reset EKF
    ekf.resetState();
    
    SyntheticDataGenerator generator;
    
    for (int i = 0; i < 628; ++i) {  // One full circle (2π/0.5 ≈ 12.56s)
        // Command circular motion
        auto data = generator.generateStep(0.01, 0.5, 0.5);
        
        // Run EKF
        ekf.predict(data.leftVel, data.rightVel);
        if (std::abs(data.imuYaw) > 0.1) ekf.updateWithIMU(data.imuYaw * M_PI / 180.0);
        if (std::abs(data.gyroZ) > 0.001) ekf.updateWithGyro(data.gyroZ);
        ekf.updateWithAccelerometer(data.accelX, data.accelY);
        ekf.updateWithEncoders(data.leftVel, data.rightVel);
        
        auto pose = ekf.getPose();
        auto velocities = ekf.getVelocities();
        
        // Log data
        logFile << "TEST3," << i*0.01 << "," 
                << pose[0] << "," << pose[1] << "," << pose[2] << ","
                << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
                << data.leftVel << "," << data.rightVel << ","
                << data.accelX << "," << data.accelY << ","
                << data.true_x << "," << data.true_y << "," << data.true_theta << "\n";
        
        if (i % 157 == 0) {  // Every quarter circle
            std::cout << "t=" << std::fixed << std::setprecision(2) << i*0.01 
                      << "s: EKF pos=(" << std::setprecision(3) << pose[0] << "," << pose[1] 
                      << ")m, True pos=(" << data.true_x << "," << data.true_y << ")m\n";
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "Final EKF position: (" << final_pose[0] << ", " << final_pose[1] << ")m\n";
    std::cout << "Expected: Close to (0.0, 0.0)m after full circle\n";
}

void testScenario4_StopAndGo(SensorFusion& ekf, std::ofstream& logFile) {
    std::cout << "\n=== TEST 4: STOP AND GO ===\n";
    std::cout << "Simulating alternating movement and stationary periods\n";
    
    // Reset EKF
    ekf.resetState();
    
    SyntheticDataGenerator generator;
    
    for (int i = 0; i < 600; ++i) {  // 6 seconds
        double t = i * 0.01;
        
        // Alternate between moving and stationary every 1 second
        double cmd_vel = (fmod(t, 2.0) < 1.0) ? 0.5 : 0.0;
        
        auto data = generator.generateStep(0.01, cmd_vel, 0.0);
        
        // When stationary, make sure accelerometer shows low values
        if (cmd_vel == 0.0) {
            data.accelX = 0.005;  // Very low noise
            data.accelY = 0.005;
            data.leftVel = 0.0;   // Perfect encoders when stopped
            data.rightVel = 0.0;
        }
        
        // Run EKF
        ekf.predict(data.leftVel, data.rightVel);
        if (std::abs(data.imuYaw) > 0.1) ekf.updateWithIMU(data.imuYaw * M_PI / 180.0);
        if (std::abs(data.gyroZ) > 0.001) ekf.updateWithGyro(data.gyroZ);
        ekf.updateWithAccelerometer(data.accelX, data.accelY);
        if (std::abs(data.leftVel) > 0.01 || std::abs(data.rightVel) > 0.01) {
            ekf.updateWithEncoders(data.leftVel, data.rightVel);
        }
        
        auto pose = ekf.getPose();
        auto velocities = ekf.getVelocities();
        
        // Log data
        logFile << "TEST4," << t << "," 
                << pose[0] << "," << pose[1] << "," << pose[2] << ","
                << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
                << data.leftVel << "," << data.rightVel << ","
                << data.accelX << "," << data.accelY << ","
                << data.true_x << "," << data.true_y << "," << data.true_theta << ","
                << cmd_vel << "\n";
        
        if (i % 100 == 0) {
            std::cout << "t=" << std::fixed << std::setprecision(2) << t 
                      << "s: CMD=" << cmd_vel << " m/s, EKF pos=(" 
                      << std::setprecision(3) << pose[0] << "," << pose[1] 
                      << ")m, EKF vel=" << velocities[0] << " m/s\n";
        }
    }
}

int main() {
    std::cout << "=== EKF SYNTHETIC DATA TESTING ===\n";
    std::cout << "This program tests the EKF with synthetic sensor data\n";
    std::cout << "to validate the behavior before real hardware testing.\n\n";
    
    // Initialize EKF with same parameters as RobotLocalization
    SensorFusion ekf(
        RobotUtils::WHEEL_RADIUS,     // wheelRadius
        RobotUtils::WHEEL_BASE,       // wheelBase  
        0.01,                         // dt (100Hz)
        1e-2,                         // processNoisePos
        1e-3,                         // processNoiseAng
        1e-2,                         // processNoiseVel
        1e-3,                         // gyroNoise
        1e-2,                         // imuNoise
        1e-1                          // accelNoise
    );
    
    // Create log file
    std::ofstream logFile("ekf_synthetic_test.csv");
    logFile << "test,time,ekf_x,ekf_y,ekf_theta,ekf_vx,ekf_vy,ekf_omega,";
    logFile << "left_vel,right_vel,accel_x,accel_y,true_x,true_y,true_theta,cmd_vel\n";
    
    // Run test scenarios
    testScenario1_FreSpinningWheels(ekf, logFile);
    testScenario2_NormalMovement(ekf, logFile);
    testScenario3_CircularMovement(ekf, logFile);
    testScenario4_StopAndGo(ekf, logFile);
    
    logFile.close();
    
    std::cout << "\n=== TESTING COMPLETE ===\n";
    std::cout << "Results saved to: ekf_synthetic_test.csv\n";
    std::cout << "You can analyze the data in Excel or Python to verify:\n";
    std::cout << "1. TEST1: Position should stay near (0,0) despite spinning wheels\n";
    std::cout << "2. TEST2: Position should reach ~(2,0) after 4 seconds\n";
    std::cout << "3. TEST3: Should return close to (0,0) after full circle\n";
    std::cout << "4. TEST4: Velocity should drop to zero during stop phases\n\n";
    
    return 0;
}
