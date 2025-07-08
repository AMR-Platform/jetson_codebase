#include "lidar_handler.hpp"
#include <opencv2/opencv.hpp>
#include "serial_com.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>
#include "config.hpp"

namespace fs = boost::filesystem;

/* ---------- LiDAR utility functions ------------------------------------------------ */
std::string timestamp()
{
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto t = clock::to_time_t(now);
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                  now.time_since_epoch())
                  .count() %
              1'000'000;
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << '_'
       << std::setw(6) << std::setfill('0') << us;
    return ss.str();
}

void scanToImage(const std::vector<LidarPoint> &scan,
                 cv::Mat &img,
                 float rangeMax,
                 int canvas = 600)
{
    img = cv::Mat::zeros(canvas, canvas, CV_8UC3);
    const cv::Point c(canvas / 2, canvas / 2);
    const float ppm = (canvas / 2) / rangeMax;
    for (const auto &p : scan)
    {
        if (p.distance <= 0.01f || p.distance > rangeMax)
            continue;
        float rad = p.azimuth * CV_PI / 180.f;
        cv::Point pt(c.x + std::cos(rad) * p.distance * ppm,
                     c.y - std::sin(rad) * p.distance * ppm);
        cv::circle(img, pt, 0.5, cv::Scalar(0, 255, 0), cv::FILLED);
    }
}

void saveCSV(const std::string &fname, const std::vector<LidarPoint> &scan)
{
    std::ofstream ofs(fname);
    ofs << "azimuth_deg,distance_m,rssi\n";
    for (auto &p : scan)
        ofs << p.azimuth << ',' << p.distance << ',' << unsigned(p.rssi) << '\n';
}

/* ---------- Main Function ------------------------------------------------ */
int main()
{
    std::unique_ptr<Serial_Com> serial;

    auto ports = Serial_Com::getAvailablePorts();

    if (ports.empty())
    {
        std::cout << "No serial ports found." << std::endl;
        return false;
    }

    for (const auto &port : ports)
    {
        try
        {
            std::cout << "Trying to open serial port: " << port << std::endl;
            serial = std::make_unique<Serial_Com>(port, DEFAULT_BAUD_RATE);

            // Set system state for debug + echo
            SystemState sys;
            sys.controlMode = AUTONOMOUS;
            sys.debugMode = MD_AND_ECHO;
            sys.updateExpectations();
            serial->setSystemState(sys);

            std::cout << "Successfully opened serial port: " << port << std::endl;
            break;
        }
        catch (const std::exception &e)
        {
            std::cout << "Failed to open " << port << ": " << e.what() << std::endl;
        }
    }

    if (!serial)
    {
        std::cerr << "Unable to connect to any serial port." << std::endl;
        return 1;
    }

    // Initialize LiDAR (commented out for now)
    // fs::create_directories("scans");
    // LidarHandler lidar;

    constexpr int canvas = 1000;
    constexpr float rangeMax = 15.f;
    constexpr auto period = std::chrono::milliseconds(LOOP_TIME);

    // cv::namedWindow("LakiBeam Viewer");
    auto next = std::chrono::steady_clock::now();

    int loop_count = 0;
    std::cout << "\n=== Starting Main Loop ===" << std::endl;

    while (true)
    {
        auto now = std::chrono::steady_clock::now();

        if (now >= next)
        {
            next += period;
            loop_count++;

            // Process LiDAR data (commented out for now)
            // auto scan = lidar.getLatestScan();
            // if (!scan.empty())
            // {
            //     cv::Mat img;
            //     scanToImage(scan, img, rangeMax, canvas);
            //     cv::imshow("LakiBeam Viewer", img);
            //     cv::waitKey(1);
            //     std::string ts = timestamp();
            //     cv::imwrite("scans/" + ts + ".jpg", img);
            //     saveCSV("scans/" + ts + ".csv", scan);
            // }

            // NEW: Listen to serial and update packets
            serial->spinOnce();

            SensorPacket sensor = serial->getSensor();
            MotionDebugPacket debug = serial->getDebug();
            CommandEchoPacket echo = serial->getCommandEcho();

            if (sensor.valid)
            {
                serial->printSensorData(sensor);
                std::cout << std::endl;
            }

            if (debug.valid)
            {
                serial->printDebugData(debug);
                std::cout << std::endl;
            }

            if (echo.valid)
            {
                serial->printCommandEcho(echo);
                std::cout << std::endl;
            }
        }

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
