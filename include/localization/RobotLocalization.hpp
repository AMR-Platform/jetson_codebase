#pragma once

#include "SensorFusion.hpp"
#include "../communication/serial_com.hpp"
#include "robot_utils.hpp"
#include "../lidar/lidar_handler.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <memory>
#include <array>
#include <string>

class RobotLocalization {
private:
    std::unique_ptr<SensorFusion> ekf_;
    std::unique_ptr<Serial_Com> serial_;
    std::unique_ptr<LidarHandler> lidar_;
    
    // Timing
    std::chrono::steady_clock::time_point lastUpdate_;
    double dt_;
    static constexpr double DEFAULT_DT = 0.01;  // 100Hz default to match I2C telemetry
    
    // Data logging
    std::ofstream logFile_;
    bool enableLogging_;
    
    // Private helper methods
    void updateEKF(const SensorPacket& sensor);
    void printStatus(const SensorPacket& sensor);
    void logData(const SensorPacket& sensor);
    std::string getCurrentTimestamp();
    
public:
    explicit RobotLocalization(const std::string& serialPort, bool enableLogging = true);
    ~RobotLocalization();
    
    void spin(CommandPacket &cmd);
    void sendCommand(CommandPacket& cmd);
    std::array<double, 3> getPose() const;
    std::array<double, 3> getVelocities() const;
};
