#pragma once
#include "SensorFusion.hpp"
#include "../communication/serial_com.hpp"
#include "robot_utils.hpp"
#include "../lidar/lidar_handler.hpp"
#include <fstream>
#include <chrono>
#include <array>
#include <string>

class RobotLocalization {
private:
    // default loop period = 10 ms
    static constexpr double DEFAULT_DT = 0.01;

    std::chrono::steady_clock::time_point lastUpdate_;
    double dt_;

    std::ofstream logFile_;
    bool enableLogging_;

public:
    RobotLocalization(bool enableLogging = true);
    ~RobotLocalization();

    void updateEKF(const SensorPacket& sensor);
    void printStatus(const SensorPacket& sensor);
    void logData   (const SensorPacket& sensor);

    std::array<double,3> getPose() const;
    std::array<double,3> getVelocities() const;

private:
    std::string getCurrentTimestamp();
};
