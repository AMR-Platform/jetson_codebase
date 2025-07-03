// main.cpp - Entry point for SLAM from scratch using IMU + Encoder + EKF

#include "serial_parser.hpp"
#include "kalman_filter.hpp"
#include "occupancy_grid.hpp"
#include "utils.hpp"
#include <iostream>

int main() {
    KalmanFilter ekf(0.05f); // 50 ms loop
    OccupancyGrid map(500, 500, 0.05f); // 25x25m grid

    SerialParser parser("/dev/ttyACM0", 115200);
    if (!parser.initialize()) {
        std::cerr << "Failed to open serial port\n";
        return -1;
    }

    std::cout << "[INFO] SLAM System Initialized\n";

    while (true) {
        parser.readPacket();

        float v = parser.getLinearVelocity();   // From encoder
        float w = parser.getYawRate();          // From IMU
        float yaw = parser.getYaw();            // From IMU

        ekf.predict(v, w);
        ekf.updateFromYaw(yaw);

        Pose2D pose = ekf.getPose();

        // Optional: mark path on occupancy grid
        map.markFree(pose.x, pose.y);

        std::cout << utils::timestamp() << " Pose: ["
                  << pose.x << ", " << pose.y << ", " << pose.theta << "]\n";

        utils::sleep_ms(50);
    }

    return 0;
}
