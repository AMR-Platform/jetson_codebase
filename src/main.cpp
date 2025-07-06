#include "lidar_handler.hpp"
#include <opencv2/opencv.hpp>
#include "serial_com.hpp"
#include "ekf.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/filesystem.hpp>
#include <chrono>

namespace fs = boost::filesystem;

/* ---------- misc helpers ------------------------------------------------ */

std::string timestamp()
{
    using clock = std::chrono::system_clock;
    auto  now   = clock::now();
    auto  t     = clock::to_time_t(now);
    auto  us    = std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch()).count() % 1'000'000;

    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << '_'
       << std::setw(6) << std::setfill('0') << us;
    return ss.str();
}

void scanToImage(const std::vector<LidarPoint>& scan,
                 cv::Mat& img,
                 float rangeMax,
                 int canvas = 600)          // new param, default old size
{
    img = cv::Mat::zeros(canvas, canvas, CV_8UC3);
    const cv::Point c(canvas/2, canvas/2);
    const float ppm = (canvas/2) / rangeMax;

    for (const auto& p : scan)
    {
        if (p.distance <= 0.01f || p.distance > rangeMax) continue;
        float rad = p.azimuth * CV_PI / 180.f;
        cv::Point pt(c.x + std::cos(rad)*p.distance*ppm,
                     c.y - std::sin(rad)*p.distance*ppm);
        cv::circle(img, pt, 0.5                //pixel_size
            , cv::Scalar(0,255,0), cv::FILLED);
    }
}

void saveCSV(const std::string& fname,
             const std::vector<LidarPoint>& scan)
{
    std::ofstream ofs(fname);
    ofs << "azimuth_deg,distance_m,rssi\n";
    for (auto& p : scan)
        ofs << p.azimuth << ',' << p.distance << ',' << unsigned(p.rssi) << '\n';
}

/* ---------- main -------------------------------------------------------- */

int main()
{
    fs::create_directories("scans");

    LidarHandler lidar;

    constexpr int   canvas   = 1000;                 // <— smaller window
    constexpr float rangeMax = 15.f;                // still 15 m radius
    constexpr auto  period   = std::chrono::milliseconds(100);  // <— 100 Hz

    cv::namedWindow("LakiBeam Viewer");

    auto next = std::chrono::steady_clock::now();
    while (true)
    {
        auto scan = lidar.getLatestScan();
        if (!scan.empty())
        {
            cv::Mat img;
            scanToImage(scan, img, rangeMax, canvas);   // pass new size

            cv::imshow("LakiBeam Viewer", img);
            cv::waitKey(1);

            std::string ts = timestamp();
            cv::imwrite("scans/" + ts + ".jpg", img);
            saveCSV      ("scans/" + ts + ".csv", scan);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
}
