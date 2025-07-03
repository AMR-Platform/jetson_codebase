// kalman_filter.cpp - EKF implementation for pose estimation

#include "kalman_filter.hpp"
#include <cmath>

KalmanFilter::KalmanFilter(float dt) : dt(dt) {
    state << 0.0f, 0.0f, 0.0f; // x, y, theta
    P = Eigen::Matrix3f::Identity() * 0.01f;

    Q = Eigen::Matrix3f::Zero();
    Q(0,0) = 0.02f;
    Q(1,1) = 0.02f;
    Q(2,2) = 0.01f;

    R = Eigen::Matrix3f::Zero();
    R(2,2) = 0.05f; // Only yaw observed
}

void KalmanFilter::predict(float v, float omega) {
    float theta = state(2);

    float dx = v * dt * std::cos(theta);
    float dy = v * dt * std::sin(theta);
    float dtheta = omega * dt;

    state(0) += dx;
    state(1) += dy;
    state(2) = normalizeAngle(state(2) + dtheta);

    Eigen::Matrix3f J;
    J << 1, 0, -v * dt * std::sin(theta),
         0, 1,  v * dt * std::cos(theta),
         0, 0, 1;

    P = J * P * J.transpose() + Q;
}

void KalmanFilter::updateFromYaw(float measured_yaw) {
    Eigen::Vector3f z;
    z << 0.0f, 0.0f, normalizeAngle(measured_yaw);

    Eigen::Vector3f h;
    h << 0.0f, 0.0f, normalizeAngle(state(2));

    Eigen::Vector3f y = z - h;
    y(2) = normalizeAngle(y(2));

    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    H(2,2) = 1.0f;

    Eigen::Matrix3f S = H * P * H.transpose() + R;
    Eigen::Matrix3f K = P * H.transpose() * S.inverse();

    state += K * y;
    state(2) = normalizeAngle(state(2));
    P = (Eigen::Matrix3f::Identity() - K * H) * P;
}

Pose2D KalmanFilter::getPose() const {
    return {state(0), state(1), state(2)};
}

float KalmanFilter::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
