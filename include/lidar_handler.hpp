#ifndef LIDAR_HANDLER_HPP
#define LIDAR_HANDLER_HPP

#include <vector>
#include <mutex>
#include <string>
#include <netinet/in.h>

struct LidarPoint {
    float azimuth;   // in degrees
    float distance;  // in meters
};

class LidarHandler {
public:
    LidarHandler(const std::string& ip = "0.0.0.0", int port = 2368);
    ~LidarHandler();

    void poll();
    std::vector<LidarPoint> getLatestScan();

private:
    void parsePacket(const uint8_t* data, size_t length);
    float decodeDistance(uint16_t raw);
    float decodeAngle(uint16_t raw);

    int sockfd;
    struct sockaddr_in lidarAddr;

    std::vector<LidarPoint> latestScan;
    std::mutex scanMutex;
};

#endif // LIDAR_HANDLER_HPP
