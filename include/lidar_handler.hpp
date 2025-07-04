#ifndef LIDAR_HANDLER_HPP
#define LIDAR_HANDLER_HPP

#include "msop_parser.h"
#include <vector>
#include <mutex>
#include <string>
#include <netinet/in.h>

class LidarHandler {
public:
    LidarHandler(int port = 2368);
    ~LidarHandler();

    // Polls for one packet and parses it
    void poll();

    // Returns the most recently parsed scan
    std::vector<LidarPoint> getLatestScan();

private:
    void parsePacket(const uint8_t* data, size_t length);

    int sockfd;
    struct sockaddr_in lidarAddr;
    MSOPParser parser;

    std::vector<LidarPoint> latestScan;
    std::mutex scanMutex;
};

#endif // LIDAR_HANDLER_HPP
