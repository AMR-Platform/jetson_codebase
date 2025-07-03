#include "kalman_filter.hpp"
#include <iostream>
#include <iomanip>

int main() {
    KalmanFilter ekf(0.1f);  // Time step = 0.1s (100ms)
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

    return 0;
}
