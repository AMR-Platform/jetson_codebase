// lidar_handler.hpp - Header for LakiBeam 1S LiDAR handler

#pragma once
#include <vector>
#include <utility>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <string>
#include <netinet/in.h>

struct LidarPoint {
    float angle;    // in degrees
    float distance; // in meters
};

class LidarHandler {
public:
    LidarHandler(const std::string& ip, int port);
    ~LidarHandler();

    void poll();
    std::vector<LidarPoint> getLatestScan();

private:
    int sockfd;
    sockaddr_in lidarAddr;
    std::mutex scanMutex;
    std::vector<LidarPoint> latestScan;

    void parsePacket(const uint8_t* data, size_t length);
    float decodeDistance(uint16_t raw);
    float decodeAngle(uint16_t raw);
};
