// RobotLocalization.hpp
#pragma once

#include "SensorFusion.hpp"
#include "../communication/serial_com.hpp"
#include "robot_utils.hpp"
#include <fstream>
#include <chrono>
#include <array>
#include <string>        // for std::string

/*** Wraps your EKF and logs once-per-tick into a single CSV file ***/
class RobotLocalization {
private:
    static constexpr double DEFAULT_DT = 0.01;

    std::chrono::steady_clock::time_point lastUpdate_;
    double dt_;

    std::ofstream logFile_;
    bool enableLogging_;

public:
    explicit RobotLocalization(bool enableLogging = true);
    ~RobotLocalization();

    void updateEKF(const SensorPacket& sensor);
    void printStatus(const SensorPacket& sensor);
    void logData   (uint32_t ts, const SensorPacket& sensor);

    std::array<double,3> getPose() const;
    std::array<double,3> getVelocities() const;

private:
    std::string getCurrentTimestamp();
};
