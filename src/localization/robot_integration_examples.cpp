// ROBOT SENSOR INTEGRATION TEMPLATE
// Replace the placeholder readSensors() function with your actual sensor interfaces

// Example for common robot interfaces:

//=============================================================================
// 1. ROS-based Robot Integration
//=============================================================================
#ifdef USE_ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class ROSRobotInterface {
private:
    ros::Subscriber odom_sub_, imu_sub_;
    sensor_msgs::Imu latest_imu_;
    nav_msgs::Odometry latest_odom_;
    bool imu_received_, odom_received_;

public:
    ROSRobotInterface(ros::NodeHandle& nh) {
        odom_sub_ = nh.subscribe("/odom", 1, &ROSRobotInterface::odomCallback, this);
        imu_sub_ = nh.subscribe("/imu/data", 1, &ROSRobotInterface::imuCallback, this);
        imu_received_ = false;
        odom_received_ = false;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        latest_odom_ = *msg;
        odom_received_ = true;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        latest_imu_ = *msg;
        imu_received_ = true;
    }
    
    SensorData getSensorData() {
        SensorData data;
        
        if (odom_received_) {
            // Extract wheel velocities from odometry
            double linear_vel = latest_odom_.twist.twist.linear.x;
            double angular_vel = latest_odom_.twist.twist.angular.z;
            double wheelbase = 0.3; // Your robot's wheelbase
            
            data.leftVel = linear_vel - (angular_vel * wheelbase / 2.0);
            data.rightVel = linear_vel + (angular_vel * wheelbase / 2.0);
        }
        
        if (imu_received_) {
            data.gyroZ = latest_imu_.angular_velocity.z;
            data.accelX = latest_imu_.linear_acceleration.x;
            data.accelY = latest_imu_.linear_acceleration.y;
            
            // Convert quaternion to yaw
            auto q = latest_imu_.orientation;
            data.imuYaw = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));
            
            data.hasGyro = true;
            data.hasIMU = true;
            data.hasAccel = true;
        }
        
        return data;
    }
};
#endif

//=============================================================================
// 2. Direct Hardware Interface (Jetson GPIO/I2C/SPI)
//=============================================================================
#ifdef USE_DIRECT_HARDWARE
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>

class DirectHardwareInterface {
private:
    int i2c_fd_;
    // Add your specific hardware interface variables
    
public:
    DirectHardwareInterface() {
        // Initialize I2C for IMU
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            std::cerr << "Failed to open I2C device" << std::endl;
        }
        
        // Initialize your encoders, IMU, etc.
        initializeEncoders();
        initializeIMU();
    }
    
    void initializeEncoders() {
        // Setup encoder pins/interfaces
        // Example for GPIO-based encoders
    }
    
    void initializeIMU() {
        // Setup IMU communication (I2C/SPI)
        // Example for MPU6050/ICM20689/etc.
    }
    
    SensorData getSensorData() {
        SensorData data;
        
        // Read encoders
        data.leftVel = readLeftEncoder();
        data.rightVel = readRightEncoder();
        
        // Read IMU
        data.gyroZ = readGyroZ();
        data.imuYaw = readIMUYaw();
        data.accelX = readAccelX();
        data.accelY = readAccelY();
        
        data.hasGyro = true;
        data.hasIMU = true;
        data.hasAccel = true;
        
        return data;
    }
    
private:
    double readLeftEncoder() {
        // Implement your left encoder reading
        // Return velocity in m/s
        return 0.0;
    }
    
    double readRightEncoder() {
        // Implement your right encoder reading
        // Return velocity in m/s
        return 0.0;
    }
    
    double readGyroZ() {
        // Read gyroscope Z-axis (angular velocity)
        // Return in rad/s
        return 0.0;
    }
    
    double readIMUYaw() {
        // Read IMU yaw angle
        // Return in radians
        return 0.0;
    }
    
    double readAccelX() {
        // Read accelerometer X-axis
        // Return in m/s²
        return 0.0;
    }
    
    double readAccelY() {
        // Read accelerometer Y-axis  
        // Return in m/s²
        return 0.0;
    }
};
#endif

//=============================================================================
// 3. Custom Robot Driver Integration
//=============================================================================
#ifdef USE_CUSTOM_DRIVER
// If you have a custom robot driver/SDK
#include "your_robot_driver.h"

class CustomRobotInterface {
private:
    YourRobotDriver robot_;
    
public:
    CustomRobotInterface() : robot_("/dev/ttyUSB0") {
        robot_.initialize();
    }
    
    SensorData getSensorData() {
        SensorData data;
        
        // Use your robot's API
        auto encoders = robot_.getEncoderVelocities();
        data.leftVel = encoders.left;
        data.rightVel = encoders.right;
        
        auto imu = robot_.getIMUData();
        data.gyroZ = imu.gyro_z;
        data.imuYaw = imu.yaw;
        data.accelX = imu.accel_x;
        data.accelY = imu.accel_y;
        
        data.hasGyro = robot_.isGyroValid();
        data.hasIMU = robot_.isIMUValid();
        data.hasAccel = robot_.isAccelValid();
        
        return data;
    }
};
#endif

//=============================================================================
// 4. Integration into Main Test Function
//=============================================================================

// Update the main() function in test_ekf_real_robot.cpp:
/*
int main() {
    // Choose your interface type
    #ifdef USE_ROS
        ros::init(argc, argv, "ekf_tester");
        ros::NodeHandle nh;
        ROSRobotInterface robot_interface(nh);
    #elif USE_DIRECT_HARDWARE
        DirectHardwareInterface robot_interface;
    #elif USE_CUSTOM_DRIVER
        CustomRobotInterface robot_interface;
    #endif
    
    RobotEKFTester tester(0.05, 0.3);  // Your wheel radius & wheelbase
    
    std::cout << "Starting Robot EKF Test..." << std::endl;
    
    while (true) {  // Or your main loop condition
        #ifdef USE_ROS
            ros::spinOnce();
        #endif
        
        // Get sensor data from your robot
        SensorData data = robot_interface.getSensorData();
        
        // Run EKF test
        tester.runTest(data, "live_test");
        
        // Sleep for your desired update rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz
    }
    
    return 0;
}
*/

//=============================================================================
// 5. Compilation Instructions
//=============================================================================

/*
To compile with different interfaces:

1. ROS Interface:
   Add to CMakeLists.txt:
   find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs)
   target_compile_definitions(test_ekf_real_robot PRIVATE USE_ROS)
   target_link_libraries(test_ekf_real_robot ${catkin_LIBRARIES})

2. Direct Hardware:
   target_compile_definitions(test_ekf_real_robot PRIVATE USE_DIRECT_HARDWARE)

3. Custom Driver:
   target_compile_definitions(test_ekf_real_robot PRIVATE USE_CUSTOM_DRIVER)
   target_link_libraries(test_ekf_real_robot your_robot_driver_lib)
*/
