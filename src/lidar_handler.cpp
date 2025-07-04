#include "lidar_handler.hpp"
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <iostream>

LidarHandler::LidarHandler(int port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return;
    }

    memset(&lidarAddr, 0, sizeof(lidarAddr));
    lidarAddr.sin_family = AF_INET;
    lidarAddr.sin_port = htons(port);
    lidarAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&lidarAddr, sizeof(lidarAddr)) < 0) {
        perror("Socket bind failed");
        close(sockfd);
        sockfd = -1;
    }
}

LidarHandler::~LidarHandler() {
    if (sockfd >= 0) {
        close(sockfd);
    }
}

void LidarHandler::poll() {
    uint8_t buffer[2048];
    ssize_t len = recv(sockfd, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (len > 0) {
        parsePacket(buffer, len);
    }
}

void LidarHandler::parsePacket(const uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(scanMutex);
    latestScan.clear();

    const uint8_t* payload = data;
    size_t payload_len = length;

    if (length == 1248) {
        payload = data + 42;
        payload_len = 1206;
    } else if (length != 1206) {
        return;
    }

    parser.parsePacket(payload, payload_len, latestScan);
}

std::vector<LidarPoint> LidarHandler::getLatestScan() {
    std::lock_guard<std::mutex> lock(scanMutex);
    return latestScan;
}
