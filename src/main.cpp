#include "lidar_handler.hpp"
#include "ekf.hpp"
#include <iostream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    LidarHandler handler;
    EKF ekf(0.1f);  // Time step = 0.1s (100ms)
    Pose2D pose;

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "\n=== Straight Movement Test ===\n";
    for (int i = 0; i < 10; ++i) {
        ekf.predict(0.5f, 0.0f);        // 0.5 m/s forward, no rotation
        ekf.updateFromYaw(0.0f);        // IMU yaw remains 0
        pose = ekf.getPose();
        std::cout << "[Step " << i << "] x: " << pose.x << ", y: " << pose.y << ", θ: " << pose.theta << "\n";
    }

    std::cout << "\n=== Rotation In-Place Test ===\n";
    for (int i = 0; i < 10; ++i) {
        ekf.predict(0.0f, M_PI / 5);    // No linear velocity, 36°/s rotation
        ekf.updateFromYaw(pose.theta + M_PI / 5 * 0.1f);  // Simulated IMU yaw increase
        pose = ekf.getPose();
        std::cout << "[Step " << i << "] x: " << pose.x << ", y: " << pose.y << ", θ: " << pose.theta << "\n";
    }

    std::cout << "\n=== Forward + Rotation Test ===\n";
    for (int i = 0; i < 20; ++i) {
        ekf.predict(0.5f, M_PI / 10);   // Forward 0.5 m/s, turn 18°/s
        ekf.updateFromYaw(pose.theta + M_PI / 10 * 0.1f);
        pose = ekf.getPose();
        std::cout << "[Step " << i << "] x: " << pose.x << ", y: " << pose.y << ", θ: " << pose.theta << "\n";
    }

    // while (true) {
    //     handler.poll();
    //     auto scan = handler.getLatestScan();

    //     if (!scan.empty()) {
    //         std::cout << "Received " << scan.size() << " points.\n";
    //         for (size_t i = 0; i < std::min(scan.size(), size_t(5)); ++i) {
    //             std::cout << "  Point " << i << ": Azimuth = " << scan[i].azimuth
    //                       << "°, Distance = " << scan[i].distance << " m\n";
    //         }
    //     }

    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    return 0;
}
