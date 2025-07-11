#include "lidar_handler.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <algorithm>

using namespace std::chrono_literals;

// helpers to convert SDK raw units
static inline float mm2m(uint16_t mm) { return mm * 0.001f; }
static inline float hunDeg2deg(uint16_t hd) { return hd * 0.01f; }

LidarHandler::LidarHandler(const std::string &local_ip,
                           const std::string &local_port,
                           const std::string &laser_ip,
                           const std::string &laser_port)
{
    driver_ = std::make_unique<LakiBeamUDP>(local_ip, local_port,
                                            laser_ip, laser_port);
    th_ = std::thread(&LidarHandler::worker, this);
}

LidarHandler::~LidarHandler()
{
    running_ = false;
    if (th_.joinable())
        th_.join();
}

void LidarHandler::dumpNextScan(uint32_t ts, std::vector<LidarPoint> &scan)
{
    // 1) grab the latest scan
    scan = getLatestScan();
    if (scan.empty())
        return;

    // 2) sort by azimuth
    std::sort(scan.begin(), scan.end(),
              [](auto &a, auto &b)
              { return a.azimuth < b.azimuth; });

    // 3) open file in append mode
    auto ts_str = std::to_string(ts);
    std::string filename = "outputs/lidar.txt";
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs.is_open())
        return; // could log an error

    // 4) write distances, space-separated, one line per scan
    for (size_t i = 0; i < scan.size(); ++i)
    {
        ofs << scan[i].distance
            << (i + 1 < scan.size() ? ' ' : '\n');
    }
    // file closed automatically when ofs goes out of scope
}

void LidarHandler::worker()
{
    repark_t frame; // SDK struct
    while (running_)
    {
        if (driver_->get_repackedpack(frame)) // = one 360° scan
        {
            std::vector<LidarPoint> tmp;
            tmp.reserve(frame.maxdots);

            for (uint16_t i = 0; i < frame.maxdots; ++i)
            {
                const auto &d = frame.dotcloud[i];
                if (d.distance == 0)
                    continue; // invalid / out-of-range

                tmp.push_back({hunDeg2deg(d.angle),
                               mm2m(d.distance),
                               d.rssi});
            }

            // swap into shared buffer
            std::lock_guard<std::mutex> lk(mtx_);
            latest_.swap(tmp);
        }

        std::this_thread::sleep_for(2ms); // avoid busy spin
    }
}

std::vector<LidarPoint> LidarHandler::getLatestScan()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return latest_; // copy (small: ≤3600 pts)
}
