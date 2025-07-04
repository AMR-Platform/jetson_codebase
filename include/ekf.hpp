// ekf.hpp - Extended Kalman Filter for dead reckoning

#pragma once
#include <Eigen/Dense>

struct Pose2D {
    float x;
    float y;
    float theta;
};

class EKF {
public:
    EKF(float dt);
    void predict(float linear_velocity, float angular_velocity);
    void updateFromYaw(float measured_yaw);
    Pose2D getPose() const;

private:
    Eigen::Vector3f state; // [x, y, theta]
    Eigen::Matrix3f P;     // Covariance
    float dt;

    Eigen::Matrix3f Q; // Process noise
    Eigen::Matrix3f R; // Measurement noise (yaw)

    float normalizeAngle(float angle);
};
