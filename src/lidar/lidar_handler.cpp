// lidar_handler.cpp
#include "lidar_handler.hpp"
#include <algorithm>
#include <boost/filesystem.hpp> // or <filesystem> if you prefer C++17
namespace fs = boost::filesystem;

// helpers to convert SDK raw units
static inline float mm2m(uint16_t mm) { return mm * 0.001f; }
static inline float hunDeg2deg(uint16_t hd) { return hd * 0.01f; }

LidarHandler::LidarHandler(const std::string &local_ip,
                           const std::string &local_port,
                           const std::string &laser_ip,
                           const std::string &laser_port)
{
    // ensure output directory exists
    fs::create_directories("outputs");

    // open single append-only file
    logFile_.open("outputs/lidar.txt", std::ios::app);

    driver_ = std::make_unique<LakiBeamUDP>(local_ip, local_port,
                                            laser_ip, laser_port);
    th_ = std::thread(&LidarHandler::worker, this);
}

LidarHandler::~LidarHandler()
{
    running_ = false;
    if (th_.joinable())
        th_.join();
    if (logFile_.is_open())
        logFile_.close();
}

void LidarHandler::dumpNextScan(uint32_t /*ts*/, const std::vector<LidarPoint> &scan)
{
    if (scan.empty() || !logFile_.is_open())
        return;

    // sort a local copy by azimuth
    std::vector<LidarPoint> sorted = scan;
    std::sort(sorted.begin(), sorted.end(),
              [](auto &a, auto &b)
              { return a.azimuth < b.azimuth; });

    // write one line: space-separated distances
    for (size_t i = 0; i < sorted.size(); ++i)
    {
        logFile_ << sorted[i].distance
                 << (i + 1 < sorted.size() ? ' ' : '\n');
    }
    logFile_.flush();
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

            std::lock_guard<std::mutex> lk(mtx_);
            latest_.swap(tmp);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

std::vector<LidarPoint> LidarHandler::getLatestScan()
{
    std::lock_guard<std::mutex> lk(mtx_);
    return latest_; // copy
}
