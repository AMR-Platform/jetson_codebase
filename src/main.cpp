// ─────────────────────────── main.cpp ────────────────────────────
#include "lidar/lidar_handler.hpp"
#include <opencv2/opencv.hpp>

#include "communication/serial_com.hpp"
#include "communication/udp_com.hpp"
#include "localization/RobotLocalization.hpp"
#include "localization/SensorFusion.hpp"
#include "localization/robot_utils.hpp"
#include "core/config.hpp"
#include "mapping/occupancy_grid.hpp"

#include <boost/filesystem.hpp>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

/* ───── graceful Ctrl-C ─────────────────────────────────────────── */
static std::atomic<bool> g_keepRunning{true};
void sigHandler(int) { g_keepRunning = false; }

/* ───── globals shared between threads ─────────────────────────── */
SensorPacket  g_sensor;
MotionDebugPacket g_debug;
CommandPacket g_cmd;
std::mutex    g_cmd_mtx;

std::vector<LidarPoint> g_scan;         // latest LiDAR points
static OccupancyGrid g_occGrid(10.f, 10.f, 0.05f, -5.f, -5.f);
static std::vector<uint8_t> g_occBuf;   // reuse buffer

/* ───── EKF (unchanged tuning) ─────────────────────────────────── */
SensorFusion g_ekf(
    RobotUtils::WHEEL_RADIUS,
    RobotUtils::WHEEL_BASE,
    0.01,
    1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);

/* ───── main helper: quick LiDAR visual (optional) ─────────────── */
static void scanToImage(const std::vector<LidarPoint>& scan,
                        cv::Mat& img,
                        float rangeMax = 8.0f,
                        int canvas = 600)
{
    img = cv::Mat::zeros(canvas, canvas, CV_8UC3);
    const cv::Point c(canvas/2, canvas/2);
    const float ppm = (canvas/2) / rangeMax;

    for (const auto& p : scan) {
        if (p.distance <= 0.05f || p.distance > rangeMax) continue;
        float rad = p.azimuth * static_cast<float>(CV_PI) / 180.f;
        cv::Point pt(c.x + std::cos(rad)*p.distance*ppm,
                     c.y - std::sin(rad)*p.distance*ppm);
        cv::circle(img, pt, 0.8, cv::Scalar(0,255,0), cv::FILLED);
    }
}

/* ──────────────────────────  MAIN  ────────────────────────────── */
int main()
{
    namespace fs = boost::filesystem;
    fs::create_directories("scans");

    std::signal(SIGINT, sigHandler);

    /* ------------- Serial & UDP initialisation ------------------ */
    std::unique_ptr<Serial_Com> serial;
    UDPCom          udp(LOCAL_UDP_PORT, REMOTE_IP, REMOTE_UDP_PORT);
    RobotLocalization localize;      // wraps g_ekf + logging
    LidarHandler     lidar("192.168.198.10", 2368, 2369); // <-- adapt

    auto ports = Serial_Com::getAvailablePorts();
    if (ports.empty()) { std::cerr << "No serial ports.\n"; return 1; }

    for (const auto& p : ports) try {
        serial = std::make_unique<Serial_Com>(p, DEFAULT_BAUD_RATE);
        SystemState sys; sys.controlMode = AUTONOMOUS;
        sys.debugMode = MD_AND_ECHO; sys.updateExpectations();
        serial->setSystemState(sys);
        std::cout << "[Serial] Connected on " << p << '\n';
        break;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; }

    if (!serial) { std::cerr << "Serial-open failed.\n"; return 1; }

    udp.start();

    /* ------------- main periodic loop --------------------------- */
    constexpr auto period = std::chrono::milliseconds(10);
    auto next = std::chrono::steady_clock::now();
    long  loopCount = 0;

    while (g_keepRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        auto now = std::chrono::steady_clock::now();
        if (now < next) continue;
        next += period;
        ++loopCount;

        /* ---- handle outbound command --------------------------- */
        {
            std::lock_guard<std::mutex> lk(g_cmd_mtx);
            if (g_cmd.cmdStatus == CMD_TOBE_WRITTEN) {
                serial->sendCommand(g_cmd);
                g_cmd.cmdStatus = CMD_JUST_WROTE;
            }
        }

        /* ---- read inbound serial ------------------------------- */
        serial->spinOnce(g_cmd);
        g_sensor = serial->getSensor();
        g_debug  = serial->getDebug();

        /* ---- send telemetry over UDP --------------------------- */
        if (g_sensor.valid) udp.sendSensor(g_sensor);
        if (g_debug.valid)  udp.sendDebug(g_debug);

        /* ---- EKF update ---------------------------------------- */
        localize.updateEKF(g_sensor);

        /* ---- LiDAR & occupancy grid every 100 ms --------------- */
        if (loopCount % 10 == 0) {
            g_scan = lidar.getLatestScan();

            if (!g_scan.empty()) {
                RobotPose pose = localize.getPose();     // expose from class
                g_occGrid.updateGrid(g_scan, pose);

                if (g_occGrid.serialize(g_occBuf))
                    udp.sendOccupancy(g_occBuf.data(), g_occBuf.size(),
                                      g_occGrid.widthCells(),
                                      g_occGrid.heightCells(),
                                      g_occGrid.resolution(),
                                      g_occGrid.originX(),
                                      g_occGrid.originY());

                /* optional live viewer
                cv::imshow("OccGrid", g_occGrid.toImage());
                cv::waitKey(1);
                */
            }
        }
    } /* while */

    /* ------------- graceful shutdown --------------------------- */
    udp.stop();

    cv::Mat mapImg = g_occGrid.toImage();
    if (cv::imwrite("final_map.png", mapImg))
        std::cout << "Saved map to final_map.png\n";

    g_occGrid.saveToFile("final_map.map");      // binary dump (optional)
    std::cout << "Bye!\n";

    return 0;
}
