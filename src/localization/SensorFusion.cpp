#include "SensorFusion.hpp"

#define _USE_MATH_DEFINES  // For M_PI on Windows
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
    
    // Initialize covariance matrix with realistic initial uncertainties
    P_ = Eigen::MatrixXd::Zero(6, 6);
    P_(0,0) = 0.1;    // x position uncertainty: 10cm
    P_(1,1) = 0.1;    // y position uncertainty: 10cm  
    P_(2,2) = 0.1;    // theta uncertainty: ~6 degrees
    P_(3,3) = 0.01;   // vx velocity uncertainty: 10cm/s
    P_(4,4) = 0.01;   // vy velocity uncertainty: 10cm/s
    P_(5,5) = 0.01;   // omega uncertainty: ~0.6 deg/s
    
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

void SensorFusion::predictWithZUPT(double leftVelocity, double rightVelocity, bool isStationary) {
    if (!initialized_) {
        initialized_ = true;
        return;
    }
    
    // If stationary (ZUPT active), ignore encoder velocities and use current velocities
    double effectiveLeftVel = isStationary ? 0.0 : leftVelocity;
    double effectiveRightVel = isStationary ? 0.0 : rightVelocity;
    
    // Apply motion model with potentially zeroed encoder inputs
    Eigen::VectorXd predicted_state = motionModel(state_, effectiveLeftVel, effectiveRightVel);
    
    // Get motion Jacobian
    Eigen::MatrixXd F = getMotionJacobian(state_, effectiveLeftVel, effectiveRightVel);
    
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
    
    // Position update (integrate velocity in robot frame to global frame)
    // Transform robot velocities to global frame and integrate
    double v_global_x = vx * cos(theta) - vy * sin(theta);
    double v_global_y = vx * sin(theta) + vy * cos(theta);
    
    new_state(0) = x + v_global_x * dt_;
    new_state(1) = y + v_global_y * dt_;
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
    
    // Kalman gain with regularization to avoid singular matrices
    if (S.determinant() < 1e-10) {
        S += Eigen::MatrixXd::Identity(1, 1) * 1e-6;  // Add small regularization
    }
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state and covariance using Joseph form for numerical stability
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
    state_ = state_ + K * innovation;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_gyro_ * K.transpose();
    
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
    
    // Kalman gain with regularization to avoid singular matrices
    if (S.determinant() < 1e-10) {
        S += Eigen::MatrixXd::Identity(1, 1) * 1e-6;  // Add small regularization
    }
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state and covariance using Joseph form for numerical stability
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
    Eigen::VectorXd state_update = K * innovation;
    state_ = state_ + state_update;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_imu_ * K.transpose();
    
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
    // If total acceleration is very small, assume robot is stationary
    double accel_magnitude = std::sqrt(accelX*accelX + accelY*accelY);
    
    // Apply ZUPT if acceleration is very small (robot not accelerating much)
    // Lowered threshold to be more sensitive to stationary conditions
    if (accel_magnitude < 0.05) {  // Less than 0.05 m/s² indicates near-stationary
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
        
        // Use very low noise for ZUPT - we're confident about zero velocity
        Eigen::MatrixXd R_zupt = Eigen::MatrixXd::Identity(2, 2) * 1e-4;  // Very confident in ZUPT
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_zupt;
        
        // Kalman gain with regularization to avoid singular matrices
        if (S.determinant() < 1e-10) {
            S += Eigen::MatrixXd::Identity(2, 2) * 1e-6;  // Add small regularization
        }
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update - force velocities towards zero using Joseph form
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
        state_ = state_ + K * innovation;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_zupt * K.transpose();
        
        // Additional: Directly constrain velocities for more aggressive ZUPT
        // This ensures that when ZUPT is active, velocities are truly forced to near-zero
        state_(3) = state_(3) * 0.1;  // Heavily damp vx
        state_(4) = state_(4) * 0.1;  // Heavily damp vy 
        state_(5) = state_(5) * 0.1;  // Heavily damp omega
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
    
    // Kalman gain with regularization to avoid singular matrices
    if (S.determinant() < 1e-10) {
        S += Eigen::MatrixXd::Identity(2, 2) * 1e-6;  // Add small regularization
    }
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update using Joseph form for numerical stability
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
    state_ = state_ + K * innovation;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_encoder_ * K.transpose();
    
    // Ensure angles are properly wrapped after update
    state_ = wrapStateToPi(state_);
}

void SensorFusion::updateWithEncodersEnhanced(double leftVel, double rightVel,
                                              double accelX, double accelY) {
    // Detect wheel slip/lift conditions before using encoder data
    if (detectWheelSlipOrLift(leftVel, rightVel, accelX, accelY)) {
        // Reduce trust in encoder measurements by increasing noise
        Eigen::MatrixXd R_reduced_trust = R_encoder_ * 10.0;  // 10x higher noise
        
        // Still update but with much lower confidence
        Eigen::Vector2d z_measured;
        z_measured(0) = (leftVel + rightVel) / 2.0;           // Forward velocity
        z_measured(1) = (rightVel - leftVel) / wheelBase_;    // Angular velocity
        
        Eigen::Vector2d z_predicted = encoderMeasurementModel(state_);
        Eigen::Vector2d innovation = z_measured - z_predicted;
        
        // Measurement Jacobian
        Eigen::MatrixXd H = encoderMeasurementJacobian();
        
        // Innovation covariance with increased noise
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_reduced_trust;
        
        // Regularization to avoid singular matrices
        if (S.determinant() < 1e-10) {
            S += Eigen::MatrixXd::Identity(2, 2) * 1e-6;
        }
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update state and covariance using Joseph form for numerical stability
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(6, 6) - K * H;
        state_ = state_ + K * innovation;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_reduced_trust * K.transpose();
        
    } else {
        // Normal encoder update with full trust
        updateWithEncoders(leftVel, rightVel);
    }
    
    // Ensure angles are properly wrapped after update
    state_ = wrapStateToPi(state_);
}

bool SensorFusion::detectWheelSlipOrLift(double leftVel, double rightVel, 
                                        double accelX, double accelY) const {
    // Calculate expected acceleration from encoder velocities
    double v_robot = (leftVel + rightVel) / 2.0;
    double omega_robot = (rightVel - leftVel) / wheelBase_;
    
    // Estimate expected acceleration in robot frame
    double expected_accel_forward = (v_robot - state_(3)) / dt_;  // Change in forward velocity
    double expected_accel_lateral = -state_(3) * omega_robot;     // Centripetal acceleration
    
    // Transform to global frame for comparison with IMU
    double cos_theta = std::cos(state_(2));
    double sin_theta = std::sin(state_(2));
    double expected_accel_x = expected_accel_forward * cos_theta - expected_accel_lateral * sin_theta;
    double expected_accel_y = expected_accel_forward * sin_theta + expected_accel_lateral * cos_theta;
    
    // Calculate acceleration discrepancy
    double accel_diff_x = std::abs(accelX - expected_accel_x);
    double accel_diff_y = std::abs(accelY - expected_accel_y);
    double total_accel_error = std::sqrt(accel_diff_x*accel_diff_x + accel_diff_y*accel_diff_y);
    
    // Thresholds for slip/lift detection
    double accel_error_threshold = 0.5;      // m/s² - significant discrepancy
    double velocity_unreasonable_threshold = 3.0;  // m/s - unreasonably high velocities
    
    // Detect slip/lift conditions
    bool high_accel_error = (total_accel_error > accel_error_threshold);
    bool unreasonable_velocities = (std::abs(leftVel) > velocity_unreasonable_threshold || 
                                   std::abs(rightVel) > velocity_unreasonable_threshold);
    bool velocity_mismatch = (std::abs(leftVel - rightVel) > 2.0 * std::abs(state_(5)) * wheelBase_);
    
    return (high_accel_error || unreasonable_velocities || velocity_mismatch);
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
    std::cout << "\n=== EKF COVARIANCE MATRIX ===\n";
    std::cout << "Diagonal elements (uncertainties):\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  σ_x²      = " << P_(0,0) << " m²\n";
    std::cout << "  σ_y²      = " << P_(1,1) << " m²\n";
    std::cout << "  σ_θ²      = " << P_(2,2) << " rad²\n";
    std::cout << "  σ_vx²     = " << P_(3,3) << " (m/s)²\n";
    std::cout << "  σ_vy²     = " << P_(4,4) << " (m/s)²\n";
    std::cout << "  σ_ω²      = " << P_(5,5) << " (rad/s)²\n";
    
    std::cout << "\nStandard deviations:\n";
    std::cout << "  σ_x       = " << std::sqrt(P_(0,0)) << " m\n";
    std::cout << "  σ_y       = " << std::sqrt(P_(1,1)) << " m\n";
    std::cout << "  σ_θ       = " << std::sqrt(P_(2,2)) << " rad (" << std::sqrt(P_(2,2)) * 180.0/M_PI << "°)\n";
    std::cout << "  σ_vx      = " << std::sqrt(P_(3,3)) << " m/s\n";
    std::cout << "  σ_vy      = " << std::sqrt(P_(4,4)) << " m/s\n";
    std::cout << "  σ_ω       = " << std::sqrt(P_(5,5)) << " rad/s\n";
    std::cout << "=================================\n";
}

void SensorFusion::printDiagnostics() const {
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║              EKF DIAGNOSTICS                 ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    
    // Current state
    std::cout << "║ STATE VECTOR:                                ║\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "║   Position:   (" << std::setw(8) << state_(0) << ", " 
              << std::setw(8) << state_(1) << ") m           ║\n";
    std::cout << "║   Heading:    " << std::setw(8) << state_(2) << " rad (" 
              << std::setw(6) << state_(2)*180.0/M_PI << "°)       ║\n";
    std::cout << "║   Velocity:   (" << std::setw(8) << state_(3) << ", " 
              << std::setw(8) << state_(4) << ") m/s         ║\n";
    std::cout << "║   Ang. Vel:   " << std::setw(8) << state_(5) << " rad/s           ║\n";
    
    // Uncertainty analysis
    std::cout << "║                                              ║\n";
    std::cout << "║ UNCERTAINTY (1σ):                           ║\n";
    std::cout << "║   Position:   (" << std::setw(6) << std::sqrt(P_(0,0)) << ", " 
              << std::setw(6) << std::sqrt(P_(1,1)) << ") m             ║\n";
    std::cout << "║   Heading:    " << std::setw(6) << std::sqrt(P_(2,2))*180.0/M_PI << "°                     ║\n";
    std::cout << "║   Velocity:   " << std::setw(6) << std::sqrt(P_(3,3)) << " m/s                  ║\n";
    
    // Filter health
    std::cout << "║                                              ║\n";
    std::cout << "║ FILTER HEALTH:                              ║\n";
    std::cout << "║   Initialized: " << (initialized_ ? "YES" : "NO ") << "                        ║\n";
    
    // Check for potential issues
    bool pos_uncertainty_high = (std::sqrt(P_(0,0)) > 1.0 || std::sqrt(P_(1,1)) > 1.0);
    bool heading_uncertainty_high = (std::sqrt(P_(2,2)) > 0.5);
    bool velocity_uncertainty_high = (std::sqrt(P_(3,3)) > 0.5);
    
    std::cout << "║   Pos. Uncert: " << (pos_uncertainty_high ? "HIGH" : "OK  ") << "                       ║\n";
    std::cout << "║   Head. Uncert:" << (heading_uncertainty_high ? "HIGH" : "OK  ") << "                       ║\n"; 
    std::cout << "║   Vel. Uncert: " << (velocity_uncertainty_high ? "HIGH" : "OK  ") << "                       ║\n";
    
    // Covariance matrix condition
    double cond_num = P_.norm() / (P_.inverse().norm());
    bool ill_conditioned = (cond_num > 1e12);
    std::cout << "║   Matrix Cond: " << (ill_conditioned ? "POOR" : "OK  ") << "                       ║\n";
    
    std::cout << "╚══════════════════════════════════════════════╝\n\n";
}

void SensorFusion::resetState() {
    state_ = Eigen::VectorXd::Zero(6);
    // Reset covariance with same initial uncertainties as constructor
    P_ = Eigen::MatrixXd::Zero(6, 6);
    P_(0,0) = 0.1;    // x position uncertainty: 10cm
    P_(1,1) = 0.1;    // y position uncertainty: 10cm  
    P_(2,2) = 0.1;    // theta uncertainty: ~6 degrees
    P_(3,3) = 0.01;   // vx velocity uncertainty: 10cm/s
    P_(4,4) = 0.01;   // vy velocity uncertainty: 10cm/s
    P_(5,5) = 0.01;   // omega uncertainty: ~0.6 deg/s
    initialized_ = false;
}

void SensorFusion::sensorFusionStep(double leftVel, double rightVel, 
                                   double gyroZ, bool hasGyro,
                                   double imuYaw, bool hasIMU,
                                   double accelX, double accelY, bool hasAccel,
                                   bool hasEncoders) {
    // Step 1: Prediction step (always first)
    predict(leftVel, rightVel);
    
    // Step 2: Update with measurements in order of reliability/frequency
    // Order: Gyro (high freq) -> Encoders (high freq) -> IMU (med freq) -> Accelerometer (ZUPT)
    
    if (hasGyro) {
        updateWithGyro(gyroZ);
    }
    
    if (hasEncoders) {
        // Use enhanced encoder update that detects wheel slip/lift
        if (hasAccel) {
            updateWithEncodersEnhanced(leftVel, rightVel, accelX, accelY);
        } else {
            updateWithEncoders(leftVel, rightVel);  // Fallback to normal update
        }
    }
    
    if (hasIMU) {
        updateWithIMU(imuYaw);
    }
    
    if (hasAccel) {
        updateWithAccelerometer(accelX, accelY);
    }
    
    // Ensure final angle wrapping after all updates
    state_ = wrapStateToPi(state_);
}

// ============= Missing Encoder Measurement Functions =============
Eigen::Vector2d SensorFusion::encoderMeasurementModel(const Eigen::VectorXd& state) const {
    // Extract velocities from state [x, y, theta, vx, vy, omega]
    double vx = state(3);
    double vy = state(4);
    double omega = state(5);
    double theta = state(2);
    
    // Convert from robot velocity to wheel velocities
    // Forward velocity in robot frame
    double v_forward = vx * cos(theta) + vy * sin(theta);
    
    // Calculate expected wheel velocities
    double leftVel = v_forward - (wheelBase_ / 2.0) * omega;
    double rightVel = v_forward + (wheelBase_ / 2.0) * omega;
    
    Eigen::Vector2d z_predicted;
    z_predicted << leftVel, rightVel;
    return z_predicted;
}

Eigen::MatrixXd SensorFusion::encoderMeasurementJacobian() const {
    // Jacobian matrix for encoder measurements
    // State: [x, y, theta, vx, vy, omega]
    // Measurement: [leftVel, rightVel]
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
    
    double theta = state_(2);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double vx = state_(3);
    double vy = state_(4);
    
    // Partial derivatives for left wheel velocity
    H(0, 2) = -vx * sin_theta + vy * cos_theta;  // d/dtheta
    H(0, 3) = cos_theta;                         // d/dvx
    H(0, 4) = sin_theta;                         // d/dvy
    H(0, 5) = -wheelBase_ / 2.0;                 // d/domega
    
    // Partial derivatives for right wheel velocity  
    H(1, 2) = -vx * sin_theta + vy * cos_theta;  // d/dtheta
    H(1, 3) = cos_theta;                         // d/dvx
    H(1, 4) = sin_theta;                         // d/dvy
    H(1, 5) = wheelBase_ / 2.0;                  // d/domega
    
    return H;
}
