#include "lidar_handler.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// helpers to convert SDK raw units
static inline float mm2m(uint16_t mm)         { return mm * 0.001f; }
static inline float hunDeg2deg(uint16_t hd)   { return hd * 0.01f;  }

LidarHandler::LidarHandler(const std::string& local_ip,
                           const std::string& local_port,
                           const std::string& laser_ip,
                           const std::string& laser_port)
{
    driver_ = std::make_unique<LakiBeamUDP>(local_ip, local_port,
                                            laser_ip,  laser_port);
    th_ = std::thread(&LidarHandler::worker, this);
}

LidarHandler::~LidarHandler()
{
    running_ = false;
    if (th_.joinable()) th_.join();
}

void LidarHandler::worker()
{
    repark_t frame;                                // SDK struct
    while (running_)
    {
        if (driver_->get_repackedpack(frame))      // = one 360° scan
        {
            std::vector<LidarPoint> tmp;
            tmp.reserve(frame.maxdots);

            for (uint16_t i = 0; i < frame.maxdots; ++i)
            {
                const auto& d = frame.dotcloud[i];
                if (d.distance == 0) continue;     // invalid / out-of-range

                tmp.push_back({ hunDeg2deg(d.angle),
                                 mm2m(d.distance),
                                 d.rssi });
            }

            // swap into shared buffer
            std::lock_guard<std::mutex> lk(mtx_);
            latest_.swap(tmp);
        }

        std::this_thread::sleep_for(2ms);          // avoid busy spin
    }
}

std::vector<LidarPoint> LidarHandler::getLatestScan()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return latest_;                                // copy (small: ≤3600 pts)
}
