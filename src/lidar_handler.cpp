// lidar_handler.cpp - Source for LakiBeam 1S LiDAR handler

#include "lidar_handler.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <iostream>

LidarHandler::LidarHandler(const std::string& ip, int port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&lidarAddr, 0, sizeof(lidarAddr));
    lidarAddr.sin_family = AF_INET;
    lidarAddr.sin_port = htons(port);
    lidarAddr.sin_addr.s_addr = inet_addr(ip.c_str());
    bind(sockfd, (struct sockaddr*)&lidarAddr, sizeof(lidarAddr));
}

LidarHandler::~LidarHandler() {
    close(sockfd);
}

void LidarHandler::poll() {
    uint8_t buffer[1500];
    ssize_t len = recv(sockfd, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (len > 0) {
        parsePacket(buffer, len);
    }
}

void LidarHandler::parsePacket(const uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(scanMutex);
    latestScan.clear();

    for (size_t i = 0; i + 4 <= length; i += 4) {
        uint16_t raw_dist = (data[i+1] << 8) | data[i];
        uint16_t raw_ang  = (data[i+3] << 8) | data[i+2];
        float dist = decodeDistance(raw_dist);
        float ang  = decodeAngle(raw_ang);
        latestScan.push_back({ang, dist});
    }
}

float LidarHandler::decodeDistance(uint16_t raw) {
    return raw / 1000.0f; // mm to meters
}

float LidarHandler::decodeAngle(uint16_t raw) {
    return raw / 100.0f; // hundredths of a degree
}

std::vector<LidarPoint> LidarHandler::getLatestScan() {
    std::lock_guard<std::mutex> lock(scanMutex);
    return latestScan;
}
