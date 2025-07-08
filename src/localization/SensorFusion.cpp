#include "SensorFusion.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

/* ———————————  ctor / dtor  ——————————— */
SensorFusion::SensorFusion(double wheelRadius,
                           double wheelBase,
                           double dt,
                           double processNoisePos,
                           double processNoiseAng,
                           double processNoiseVel,
                           double gyroNoise,
                           double imuNoise,
                           double accelNoise)
    : wheelRadius_(wheelRadius),
      wheelBase_(wheelBase),
      dt_(dt),
      initialized_(false) {
    
    // Initialize state vector [x, y, theta, vx, vy, omega]
    state_ = Eigen::VectorXd::Zero(6);
    
    // Initialize covariance matrix
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    
    // Process noise matrix Q
    Q_ = Eigen::MatrixXd::Zero(6, 6);
    Q_(0,0) = processNoisePos;  // x position noise
    Q_(1,1) = processNoisePos;  // y position noise
    Q_(2,2) = processNoiseAng;  // theta noise
    Q_(3,3) = processNoiseVel;  // vx noise
    Q_(4,4) = processNoiseVel;  // vy noise
    Q_(5,5) = processNoiseAng;  // omega noise
    
    // Measurement noise matrices
    R_gyro_ = Eigen::MatrixXd::Identity(1, 1) * gyroNoise;
    R_imu_ = Eigen::MatrixXd::Identity(1, 1) * imuNoise;
    R_accel_ = Eigen::MatrixXd::Identity(2, 2) * accelNoise;
    R_encoder_ = Eigen::MatrixXd::Identity(2, 2);
    R_encoder_(0,0) = processNoiseVel;  // Forward velocity noise
    R_encoder_(1,1) = processNoiseAng;  // Angular velocity noise
}

void SensorFusion::predict(double leftVelocity, double rightVelocity) {
    if (!initialized_) {
        initialized_ = true;
        return;
    }
    
    // Apply motion model
    Eigen::VectorXd predicted_state = motionModel(state_, leftVelocity, rightVelocity);
    
    // Get motion Jacobian
    Eigen::MatrixXd F = getMotionJacobian(state_, leftVelocity, rightVelocity);
    
    // Predict covariance
    Eigen::MatrixXd predicted_P = F * P_ * F.transpose() + Q_;
    
    // Update state and covariance
    state_ = wrapStateToPi(predicted_state);
    P_ = predicted_P;
}

Eigen::VectorXd SensorFusion::motionModel(const Eigen::VectorXd& state, 
                                         double vLeft, double vRight) const {
    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double vx = state(3);
    double vy = state(4);
    double omega = state(5);
    
    // Calculate robot velocities from wheel velocities
    double v_robot = (vLeft + vRight) / 2.0;
    double omega_robot = (vRight - vLeft) / wheelBase_;
    
    // Predict new state
    Eigen::VectorXd new_state(6);
    
    // Position update (integrate velocity in global frame)
    new_state(0) = x + (vx * cos(theta) - vy * sin(theta)) * dt_;
    new_state(1) = y + (vx * sin(theta) + vy * cos(theta)) * dt_;
    new_state(2) = theta + omega * dt_;
    
    // Velocity update (based on wheel commands with some smoothing)
    double alpha = 0.8;  // Smoothing factor
    new_state(3) = alpha * vx + (1.0 - alpha) * v_robot;      // Forward velocity
    new_state(4) = alpha * vy;                                // Lateral velocity (decay)
    new_state(5) = alpha * omega + (1.0 - alpha) * omega_robot; // Angular velocity
    
    return new_state;
}

Eigen::MatrixXd SensorFusion::getMotionJacobian(const Eigen::VectorXd& state,
                                               double vLeft, double vRight) const {
    double theta = state(2);
    double vx = state(3);
    double vy = state(4);
    
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    
    // Position derivatives
    F(0,2) = -(vx * sin(theta) + vy * cos(theta)) * dt_;  // dx/dtheta
    F(0,3) = cos(theta) * dt_;                            // dx/dvx
    F(0,4) = -sin(theta) * dt_;                           // dx/dvy
    
    F(1,2) = (vx * cos(theta) - vy * sin(theta)) * dt_;   // dy/dtheta
    F(1,3) = sin(theta) * dt_;                            // dy/dvx
    F(1,4) = cos(theta) * dt_;                            // dy/dvy
    
    F(2,5) = dt_;                                         // dtheta/domega
    
    // Velocity smoothing
    double alpha = 0.8;
    F(3,3) = alpha;    // dvx/dvx
    F(4,4) = alpha;    // dvy/dvy  
    F(5,5) = alpha;    // domega/domega
    
    return F;
}

void SensorFusion::updateWithGyro(double gyroZ) {
    // Measurement model: z = omega (angular velocity)
    double predicted = gyroMeasurementModel(state_);
    double innovation = gyroZ - predicted;
    
    // Measurement Jacobian
    Eigen::MatrixXd H = gyroMeasurementJacobian();
    
    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_gyro_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state and covariance
    state_ = state_ + K * innovation;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    
    state_ = wrapStateToPi(state_);
}

double SensorFusion::gyroMeasurementModel(const Eigen::VectorXd& state) const {
    return state(5);  // Return omega (angular velocity)
}

Eigen::MatrixXd SensorFusion::gyroMeasurementJacobian() const {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
    H(0,5) = 1.0;  // dh/domega = 1
    return H;
}

void SensorFusion::updateWithIMU(double imuYaw) {
    // Measurement model: z = theta (orientation)
    double predicted = imuMeasurementModel(state_);
    double innovation = wrapToPi(imuYaw - predicted);
    
    // Measurement Jacobian
    Eigen::MatrixXd H = imuMeasurementJacobian();
    
    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state and covariance
    Eigen::VectorXd state_update = K * innovation;
    state_ = state_ + state_update;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    
    state_ = wrapStateToPi(state_);
}

double SensorFusion::imuMeasurementModel(const Eigen::VectorXd& state) const {
    return state(2);  // Return theta (orientation)
}

Eigen::MatrixXd SensorFusion::imuMeasurementJacobian() const {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
    H(0,2) = 1.0;  // dh/dtheta = 1
    return H;
}

void SensorFusion::updateWithAccelerometer(double accelX, double accelY) {
    // Use accelerometer for Zero Velocity Update (ZUPT) in 2D
    // If total acceleration is close to gravity magnitude, assume robot is stationary
    double accel_magnitude = std::sqrt(accelX*accelX + accelY*accelY);
    
    // Apply ZUPT if acceleration is small (robot not accelerating much)
    // For 2D operation, we expect small X,Y accelerations when stationary
    if (accel_magnitude < 1.0) {  // Less than 1 m/s² indicates near-stationary
        // Measurement: velocities should be zero
        Eigen::Vector2d z_measured = Eigen::Vector2d::Zero();
        Eigen::Vector2d z_predicted;
        z_predicted(0) = state_(3);  // vx
        z_predicted(1) = state_(4);  // vy
        
        Eigen::Vector2d innovation = z_measured - z_predicted;
        
        // Measurement Jacobian for velocity
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
        H(0,3) = 1.0;  // dvx/dvx = 1
        H(1,4) = 1.0;  // dvy/dvy = 1
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_accel_;
        
        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update
        state_ = state_ + K * innovation;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    }
}

void SensorFusion::updateWithEncoders(double leftVel, double rightVel) {
    // Measurement model: velocities derived from encoders
    Eigen::Vector2d z_measured;
    z_measured(0) = (leftVel + rightVel) / 2.0;           // Forward velocity
    z_measured(1) = (rightVel - leftVel) / wheelBase_;    // Angular velocity
    
    Eigen::Vector2d z_predicted = encoderMeasurementModel(state_);
    Eigen::Vector2d innovation = z_measured - z_predicted;
    
    // Measurement Jacobian
    Eigen::MatrixXd H = encoderMeasurementJacobian();
    
    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_encoder_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update
    state_ = state_ + K * innovation;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
}

Eigen::Vector2d SensorFusion::encoderMeasurementModel(const Eigen::VectorXd& state) const {
    Eigen::Vector2d h;
    h(0) = state(3);  // Forward velocity
    h(1) = state(5);  // Angular velocity
    return h;
}

Eigen::MatrixXd SensorFusion::encoderMeasurementJacobian() const {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
    H(0,3) = 1.0;  // dh1/dvx = 1
    H(1,5) = 1.0;  // dh2/domega = 1
    return H;
}

double SensorFusion::wrapToPi(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::VectorXd SensorFusion::wrapStateToPi(const Eigen::VectorXd& state) const {
    Eigen::VectorXd wrapped_state = state;
    wrapped_state(2) = wrapToPi(state(2));  // Wrap theta
    return wrapped_state;
}

std::array<double, 3> SensorFusion::getPose() const {
    return {state_(0), state_(1), state_(2)};
}

std::array<double, 3> SensorFusion::getVelocities() const {
    return {state_(3), state_(4), state_(5)};
}

Eigen::VectorXd SensorFusion::getFullState() const {
    return state_;
}

Eigen::MatrixXd SensorFusion::getCovariance() const {
    return P_;
}

void SensorFusion::printState() const {
    std::cout << std::fixed << std::setprecision(4)
              << "EKF STATE -> x: " << state_(0) << " m, y: " << state_(1) 
              << " m, θ: " << state_(2) << " rad (" << (state_(2) * 180.0 / M_PI) << "°)" << std::endl;
    std::cout << "VELOCITIES -> vx: " << state_(3) << " m/s, vy: " << state_(4) 
              << " m/s, ω: " << state_(5) << " rad/s (" << (state_(5) * 180.0 / M_PI) << " °/s)" << std::endl;
}

void SensorFusion::printCovariance() const {
    std::cout << "COVARIANCE DIAGONAL -> [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::scientific << std::setprecision(2) << P_(i,i);
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

void SensorFusion::resetState() {
    state_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    initialized_ = false;
}
