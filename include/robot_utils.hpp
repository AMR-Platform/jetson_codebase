#ifndef ROBOT_UTILS_HPP
#define ROBOT_UTILS_HPP

#include "serial_com.hpp"
#include <cmath>

class RobotUtils {
public:
    // Robot physical parameters
    static constexpr double WHEEL_DIAMETER = 0.2;       // 200mm = 0.2m
    static constexpr double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;  // 0.1m
    static constexpr double WHEEL_BASE = 0.5;            // 500mm = 0.5m
    static constexpr double TICKS_PER_REV = 40000.0;     // 4 × 1000 × 10 = 40,000 ticks
    static constexpr double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;  // π × 0.2m = 0.628m
    static constexpr double METERS_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
    
    // Convert encoder ticks to distance (meters)
    static double ticksToMeters(long ticks) {
        return ticks * METERS_PER_TICK;
    }
    
    // Convert encoder tick difference to velocity (m/s)
    static double tickDiffToVelocity(float tickDiff, double dt) {
        return (tickDiff * METERS_PER_TICK) / dt;
    }
    
    // Convert degrees to radians
    static double degToRad(double degrees) {
        return degrees * M_PI / 180.0;
    }
    
    // Convert radians to degrees
    static double radToDeg(double radians) {
        return radians * 180.0 / M_PI;
    }
    
    // Extract wheel velocities from SensorPacket
    static std::pair<double, double> getWheelVelocities(const SensorPacket& sensor, double dt) {
        double leftVel = tickDiffToVelocity(sensor.dEncL, dt);
        double rightVel = tickDiffToVelocity(sensor.dEncR, dt);
        return {leftVel, rightVel};
    }
    
    // Convert angular velocity from tick-difference to rad/s
    static double angularVelocityFromEncoders(const SensorPacket& sensor, double dt) {
        auto [leftVel, rightVel] = getWheelVelocities(sensor, dt);
        return (rightVel - leftVel) / WHEEL_BASE;
    }
    
    // Normalize angle to [-π, π]
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif // ROBOT_UTILS_HPP
