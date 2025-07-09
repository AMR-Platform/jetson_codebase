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

// ───── Configuration ─────────────────────────────────────
bool USE_ROBOT_LOCALIZATION = true; // Set to true to use EKF-based localization
// ──────────────────────────────────────────────────────────

// ───── Globals (used by both EKF and Basic modes) ────────────────────────────
SensorPacket g_sensor;
MotionDebugPacket g_debug;
CommandPacket g_cmd;
// ──────────────────────────────────────────────────────────────────────────────

/* ---------- Global Access Functions ------------------------------------------- */
// These functions provide access to current robot state from anywhere in the code
const SensorPacket &getCurrentSensorData() { return g_sensor; }
const MotionDebugPacket &getCurrentDebugData() { return g_debug; }
const CommandPacket &getCurrentCommand() { return g_cmd; }

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
    std::unique_ptr<Serial_Com> serial;
    std::unique_ptr<RobotLocalization> robot;

    // Check if we should use RobotLocalization (EKF-based localization)
    if (USE_ROBOT_LOCALIZATION)
    {
        std::cout << "=== STARTING EKF-BASED ROBOT LOCALIZATION MODE ===" << std::endl;
        std::cout << "This mode provides EKF localization with CSV logging and UDP support" << std::endl;
        std::cout << "=================================================" << std::endl;

        auto ports = Serial_Com::getAvailablePorts();
        if (ports.empty())
        {
            std::cerr << "No serial ports found." << std::endl;
            return 1;
        }

        std::string selectedPort;
        for (const auto &port : ports)
        {
            std::cout << "Trying to connect to: " << port << std::endl;
            try
            {
                // Test connection
                Serial_Com test(port, DEFAULT_BAUD_RATE);
                selectedPort = port;
                std::cout << "Successfully connected to: " << port << std::endl;
                break;
            }
            catch (const std::exception &e)
            {
                std::cout << "Failed to connect to " << port << ": " << e.what() << std::endl;
            }
        }

        if (selectedPort.empty())
        {
            std::cerr << "Unable to connect to any serial port." << std::endl;
            return 1;
        }

        try
        {
            robot = std::make_unique<RobotLocalization>(selectedPort, true); // Enable logging

            std::cout << "\n✓ Robot localization system started!" << std::endl;
            std::cout << "✓ Using EKF with wheel odometry, IMU, and gyroscope fusion" << std::endl;
            std::cout << "✓ Data logging enabled - CSV files will be saved" << std::endl;
            std::cout << "✓ Status updates every 1 second" << std::endl;
            std::cout << "✓ Global variables (g_sensor, g_debug, g_cmd) will be updated" << std::endl;
            std::cout << "✓ UDP communication enabled for remote commands" << std::endl;
            std::cout << "Press Ctrl+C to exit\n"
                      << std::endl;

            // Configure robot for EKF testing
            CommandPacket cmd;
            cmd.mode = AUTONOMOUS;  // Set to autonomous mode
            cmd.dbg = MOTION_DEBUG; // Enable motion debug for detailed output
            cmd.distance = 2000;
            cmd.maxVel = 300;
            cmd.linAcc = 100;
            cmd.lastVel = 0;
            robot->sendCommand(cmd);

            std::cout << "Command sent: mode=AUTONOMOUS, debug=MOTION_DEBUG" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
            return 1;
        }
    }
    else
    {
        // ===== ORIGINAL MAIN FUNCTIONALITY (BASIC SERIAL + LIDAR) =====
        std::cout << "=== STARTING BASIC SERIAL COMMUNICATION MODE ===" << std::endl;
        std::cout << "This mode provides basic serial communication and LiDAR support" << std::endl;
        std::cout << "No EKF or advanced localization features" << std::endl;
        std::cout << "=================================================" << std::endl;

        auto ports = Serial_Com::getAvailablePorts();

        if (ports.empty())
        {
            std::cout << "No serial ports found." << std::endl;
            return 1;
        }

        // Only set up basic serial if not using EKF mode
        if (!USE_ROBOT_LOCALIZATION)
        {
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
        }

        // Initialize LiDAR (commented out for now)
        // fs::create_directories("scans");
        // LidarHandler lidar;

        // ───── Setup UDP communication ───────────────────────────────────────────────
        constexpr uint16_t LOCAL_UDP_PORT = 9000;
        constexpr uint16_t REMOTE_UDP_PORT = 9001;
        const std::string REMOTE_IP = "192.168.1.68"; // adjust as needed
        UDPCom udp(LOCAL_UDP_PORT, REMOTE_IP, REMOTE_UDP_PORT);
        udp.start();
        // ──────────────────────────────────────────────────────────────────────────────

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

                // Handle remote commands from UDP
                if (g_cmd.cmdStatus == CMD_TOBE_WRITTEN)
                {
                    if (USE_ROBOT_LOCALIZATION && robot)
                    {
                        robot->sendCommand(g_cmd);
                    }
                    else if (serial)
                    {
                        serial->sendCommand(g_cmd);
                    }
                    g_cmd.cmdStatus = CMD_JUST_WROTE; // mark as sent
                    std::cout << "[CMD] Command sent: mode=" << int(g_cmd.mode) << std::endl;
                }

                // Handle different communication modes
                if (USE_ROBOT_LOCALIZATION && robot)
                {
                    // EKF mode: use RobotLocalization's spinOnce for single iteration
                    robot->spinOnce(g_cmd);

                    // Print EKF status every few loops
                    if (loop_count % 100 == 0)
                    { // Every ~1 second at 100Hz
                        auto pose = robot->getPose();
                        auto vel = robot->getVelocities();
                        std::cout << "[EKF] Pose: (" << std::fixed << std::setprecision(3)
                                  << pose[0] << "," << pose[1] << "," << pose[2] * 180 / M_PI << "°) "
                                  << "Vel: (" << vel[0] << "," << vel[1] << "," << vel[2] * 180 / M_PI << "°/s)" << std::endl;
                    }
                }
                else
                {
                    // Basic mode: use regular serial communication
                    serial->spinOnce(g_cmd);

                    // Update globals
                    g_sensor = serial->getSensor();
                    g_debug = serial->getDebug();
                    auto echo = serial->getCommandEcho();

                    // Print command echo if available
                    if (echo.valid)
                    {
                        serial->printCommandEcho(echo);
                        std::cout << std::endl;
                    }
                }

                // Send telemetry back over UDP (works for both modes)
                if (g_sensor.valid)
                    udp.sendSensor(g_sensor);
                if (g_debug.valid)
                    udp.sendDebug(g_debug);
            }

            // Small delay to prevent excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Clean up
        udp.stop();
        return 0;
    }
}
