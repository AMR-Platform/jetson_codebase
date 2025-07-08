#ifndef LIDAR_HANDLER_HPP
#define LIDAR_HANDLER_HPP

#include "LakiBeamUDP.h"               //  ←  SDK header
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

/*** A single polar sample from the LiDAR */
struct LidarPoint {
    float azimuth;     // deg   (0 … 360)
    float distance;    // m     (0 = invalid)
    uint8_t rssi;      // 0 … 255
};

/*** Thin wrapper around Richbeam’s LakiBeamUDP that:
 *   • runs the SDK in a background thread
 *   • converts each completed 360° frame into std::vector<LidarPoint>
 *   • offers getLatestScan() (thread-safe copy)                           */
class LidarHandler {
public:
    explicit LidarHandler(const std::string& local_ip   = "0.0.0.0",
                          const std::string& local_port = "2368",
                          const std::string& laser_ip   = "192.168.198.2",
                          const std::string& laser_port = "2368");
    ~LidarHandler();

    /** Non-blocking; returns an empty vector if no new scan is ready. */
    std::vector<LidarPoint> getLatestScan();

private:
    void worker();                      // background thread

    std::unique_ptr<LakiBeamUDP> driver_;
    std::thread                  th_;
    std::mutex                   mtx_;
    std::vector<LidarPoint>      latest_;
    std::atomic<bool>            running_{true};
};

#endif   // LIDAR_HANDLER_HPP
