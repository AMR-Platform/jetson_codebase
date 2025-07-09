#ifndef SENSOR_FUSION_HPP
#define SENSOR_FUSION_HPP

#include <array>
#include <vector>
#include <Eigen/Dense>

class SensorFusion {
public:
    // Constructor
    SensorFusion(double wheelRadius,
                 double wheelBase,
                 double dt = 0.01,                      // Default 100Hz (10ms)
                 double processNoisePos = 1e-4,
                 double processNoiseAng = 1e-5,
                 double processNoiseVel = 1e-3,
                 double gyroNoise = 1e-4,
                 double imuNoise = 1e-3,
                 double accelNoise = 1e-2);
    
    // Prediction step using encoder data
    void predict(double leftVelocity, double rightVelocity);
    void predictWithZUPT(double leftVelocity, double rightVelocity, bool isStationary);
    
    // Update steps with different sensors
    void updateWithGyro(double gyroZ);
    void updateWithIMU(double imuYaw);
    void updateWithAccelerometer(double accelX, double accelY);  // 2D accelerometer for ZUPT
    void updateWithEncoders(double leftVel, double rightVel);
    
    // Get current state estimates
    std::array<double, 3> getPose() const;           // [x, y, theta]
    std::array<double, 3> getVelocities() const;     // [vx, vy, omega]
    Eigen::VectorXd getFullState() const;            // Full 6D state
    Eigen::MatrixXd getCovariance() const;           // Covariance matrix
    
    // Get specific state components
    double getX() const { return state_(0); }
    double getY() const { return state_(1); }
    double getTheta() const { return state_(2); }
    double getVx() const { return state_(3); }
    double getVy() const { return state_(4); }
    double getOmega() const { return state_(5); }
    
    // Utility functions
    void printState() const;
    void printCovariance() const;
    void resetState();
    bool isInitialized() const { return initialized_; }

private:
    // Robot parameters
    double wheelRadius_;
    double wheelBase_;
    double dt_;
    
    // State vector: [x, y, theta, vx, vy, omega] (6D)
    Eigen::VectorXd state_;      // State estimate
    Eigen::MatrixXd P_;          // Covariance matrix
    
    // Noise matrices
    Eigen::MatrixXd Q_;          // Process noise
    Eigen::MatrixXd R_gyro_;     // Gyro measurement noise
    Eigen::MatrixXd R_imu_;      // IMU measurement noise  
    Eigen::MatrixXd R_accel_;    // Accelerometer noise
    Eigen::MatrixXd R_encoder_;  // Encoder noise
    
    // State tracking
    bool initialized_;
    
    // Motion model functions
    Eigen::VectorXd motionModel(const Eigen::VectorXd& state, 
                               double vLeft, double vRight) const;
    Eigen::MatrixXd getMotionJacobian(const Eigen::VectorXd& state,
                                     double vLeft, double vRight) const;
    
    // Measurement model functions
    double gyroMeasurementModel(const Eigen::VectorXd& state) const;
    Eigen::MatrixXd gyroMeasurementJacobian() const;
    
    double imuMeasurementModel(const Eigen::VectorXd& state) const;
    Eigen::MatrixXd imuMeasurementJacobian() const;
    
    Eigen::Vector2d encoderMeasurementModel(const Eigen::VectorXd& state) const;
    Eigen::MatrixXd encoderMeasurementJacobian() const;
    
    // Utility functions
    double wrapToPi(double angle) const;
    Eigen::VectorXd wrapStateToPi(const Eigen::VectorXd& state) const;
};

#endif // SENSOR_FUSION_HPP
