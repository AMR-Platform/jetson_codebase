// main.cpp
#include "lidar/lidar_handler.hpp"
#include <opencv2/opencv.hpp>
#include "communication/serial_com.hpp"
#include "communication/udp_com.hpp"
#include "localization/RobotLocalization.hpp"
#include "localization/SensorFusion.hpp"
#include "localization/robot_utils.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include "core/config.hpp"

namespace fs = boost::filesystem;

// ───── Globals (used by both EKF and Basic modes) ────────────────────────────
SensorPacket g_sensor;
MotionDebugPacket g_debug;
CommandEchoPacket echo; // used for debugging

std::mutex g_cmd_mtx; // for thread‐safing g_cmd access
CommandPacket g_cmd;

std::vector<LidarPoint> g_scan; // holds the latest LiDAR scan data

// EKF instance for localization (tuned noise parameters)
SensorFusion g_ekf(
    RobotUtils::WHEEL_RADIUS,
    RobotUtils::WHEEL_BASE,
    0.01, // dt
    1e-2, // processNoisePos
    1e-3, // processNoiseAng
    1e-2, // processNoiseVel
    1e-3, // gyroNoise
    1e-2, // imuNoise
    1e-1  // accelNoise
);
// ──────────────────────────────────────────────────────────────────────────────

/* ---------- Global Access Functions ------------------------------------------- */
const SensorPacket &getCurrentSensorData() { return g_sensor; }
const MotionDebugPacket &getCurrentDebugData() { return g_debug; }
const CommandPacket &getCurrentCommand()
{
    std::lock_guard<std::mutex> lk(g_cmd_mtx);
    return g_cmd;
}

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
    ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
       << '_' << std::setw(6) << std::setfill('0') << us;
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
    for (auto &p : scan)
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
    fs::create_directories("outputs");

    std::unique_ptr<Serial_Com> serial;
    LidarHandler lidar;
    UDPCom udp(LOCAL_UDP_PORT, REMOTE_IP, REMOTE_UDP_PORT);
    RobotLocalization localize; // default-construct with logging on

    // ── NEW: record chosen serial port for RobotLocalization
    std::string serialPortString;

    auto ports = Serial_Com::getAvailablePorts();
    udp.start();
    // cv::namedWindow("Lidar Viewer");

    if (ports.empty())
    {
        std::cout << "No serial ports found." << std::endl;
        return 1;
    }

    for (const auto &port : ports)
    {
        try
        {
            std::cout << "Trying to open serial port: " << port << std::endl;
            serial = std::make_unique<Serial_Com>(port, DEFAULT_BAUD_RATE);

            // ── NEW: save the successful port name
            serialPortString = port;

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

    auto next = std::chrono::steady_clock::now();
    int loop_count = 0;

    // define your loop period explicitly
    constexpr auto period = std::chrono::milliseconds(10);

    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        if (now >= next)
        {
            next += period;
            loop_count++;

            // Handle remote commands from UDP
            if (g_cmd.cmdStatus == CMD_TOBE_WRITTEN)
            {
                serial->sendCommand(g_cmd);
                g_cmd.cmdStatus = CMD_JUST_WROTE; // mark as sent
                std::cout << "[SRL] Command sent: mode=" << int(g_cmd.mode);
            }

            // Read & parse serial
            serial->spinOnce(g_cmd);

            // Update globals
            g_sensor = serial->getSensor();
            g_debug = serial->getDebug();
            auto echoPkt = serial->getCommandEcho();

            // Send telemetry back over UDP
            if (g_sensor.valid)
                udp.sendSensor(g_sensor);
            if (g_debug.valid)
                udp.sendDebug(g_debug);

            if (echoPkt.valid)
            {
                serial->printCommandEcho(echoPkt);
                std::cout << std::endl;
            }

            localize.updateEKF(g_sensor);
            localize.printStatus(g_sensor);

            if (loop_count % 10 == 0)
            {
                auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now()
                                      .time_since_epoch())
                                  .count();
                uint32_t now_ts = static_cast<uint32_t>(now_ms & 0xFFFFFFFF);

                // Process LiDAR data
                g_scan = lidar.getLatestScan();

                lidar.dumpNextScan(now_ts, g_scan);
                localize.logData(now_ts, g_sensor);
                
                
                //     if (!g_scan.empty()) {
                //         cv::Mat img;
                //         scanToImage(g_scan, img, rangeMax, canvas);
                //         cv::imshow("LakiBeam Viewer", img);
                //         cv::waitKey(1);
                //         std::string ts = timestamp();
                //         //cv::imwrite("scans/" + ts + ".jpg", img);
                //         //saveCSV("scans/" + ts + ".csv", g_scan);
                //     }
            }
        }

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // never reached, but clean up if we ever break out
    udp.stop();
    return 0;
}
