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

std::mutex g_cmd_mtx; // for thread safing the g_cmd access
CommandPacket g_cmd;

std::vector<LidarPoint> g_scan; // holds the latest LiDAR scan data
// ──────────────────────────────────────────────────────────────────────────────

/* ---------- Global Access Functions ------------------------------------------- */
// These functions provide access to current robot state from anywhere in the code
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

// Add this function to your main.cpp file
// Calculates robot pose from encoder data and writes to pose.txt for Occupancy-SLAM

void writePoseData(const SensorPacket &sensor)
{
    static std::ofstream poseFile;
    static bool first_pose = true;
    static double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;
    static long prev_encL = 0, prev_encR = 0;
    
    // Robot physical parameters (adjust these to match your robot)
    const double WHEEL_BASE = 0.235;  // distance between wheels in meters (adjust for your robot)
    const double COUNTS_PER_METER = 1000.0; // encoder counts per meter (adjust for your robot)
    
    // Open file in append mode on first call
    if (!poseFile.is_open()) {
        poseFile.open("scans/pose.txt", std::ios::app);
        if (!poseFile.is_open()) {
            std::cerr << "Error: Cannot open pose.txt" << std::endl;
            return;
        }
    }
    
    // Only write if sensor data is valid
    if (!sensor.valid) return;
    
    if (first_pose) {
        // First pose: initialize position and write absolute pose
        robot_x = 0.0;
        robot_y = 0.0;
        robot_theta = sensor.yaw * M_PI / 180.0; // convert degrees to radians
        
        poseFile << robot_x << " " << robot_y << " " << robot_theta << std::endl;
        
        prev_encL = sensor.encL;
        prev_encR = sensor.encR;
        first_pose = false;
        
        std::cout << "Initialized pose tracking. First pose: " 
                  << robot_x << " " << robot_y << " " << robot_theta << std::endl;
    } else {
        // Calculate movement from encoders
        double delta_encL = (sensor.encL - prev_encL) / COUNTS_PER_METER;
        double delta_encR = (sensor.encR - prev_encR) / COUNTS_PER_METER;
        
        // Calculate robot movement
        double delta_distance = (delta_encL + delta_encR) / 2.0;
        double delta_angle = (delta_encR - delta_encL) / WHEEL_BASE;
        
        // Update robot pose
        double prev_x = robot_x;
        double prev_y = robot_y;
        double prev_theta = robot_theta;
        
        robot_x += delta_distance * cos(robot_theta + delta_angle / 2.0);
        robot_y += delta_distance * sin(robot_theta + delta_angle / 2.0);
        robot_theta += delta_angle;
        
        // Normalize angle to [-pi, pi]
        while (robot_theta > M_PI) robot_theta -= 2.0 * M_PI;
        while (robot_theta < -M_PI) robot_theta += 2.0 * M_PI;
        
        // Calculate incremental pose for Occupancy-SLAM
        double pose_delta_x = robot_x - prev_x;
        double pose_delta_y = robot_y - prev_y;
        double pose_delta_theta = robot_theta - prev_theta;
        
        // Normalize angle difference
        while (pose_delta_theta > M_PI) pose_delta_theta -= 2.0 * M_PI;
        while (pose_delta_theta < -M_PI) pose_delta_theta += 2.0 * M_PI;
        
        // Write incremental pose
        poseFile << pose_delta_x << " " << pose_delta_y << " " << pose_delta_theta << std::endl;
        
        // Update previous encoder values
        prev_encL = sensor.encL;
        prev_encR = sensor.encR;
        
        // Optional: print every 100th pose for monitoring
        static int pose_count = 0;
        pose_count++;
        if (pose_count % 100 == 0) {
            std::cout << "Pose #" << pose_count << " - Robot at: " 
                      << robot_x << ", " << robot_y << ", " << robot_theta * 180.0 / M_PI << "°" << std::endl;
        }
    }
    
    poseFile.flush();
}

// void scanToImage(const std::vector<LidarPoint> &scan,
//                  cv::Mat &img,
//                  float rangeMax,
//                  int canvas = 600)
// {
//     img = cv::Mat::zeros(canvas, canvas, CV_8UC3);
//     const cv::Point c(canvas / 2, canvas / 2);
//     const float ppm = (canvas / 2) / rangeMax;
//     for (auto &p : scan)
//     {
//         if (p.distance <= 0.01f || p.distance > rangeMax)
//             continue;
//         float rad = p.azimuth * CV_PI / 180.f;
//         cv::Point pt(c.x + std::cos(rad) * p.distance * ppm,
//                      c.y - std::sin(rad) * p.distance * ppm);
//         cv::circle(img, pt, 0.5, cv::Scalar(0, 255, 0), cv::FILLED);
//     }
// }

// void saveCSV(const std::string &fname, const std::vector<LidarPoint> &scan)
// {
//     std::ofstream ofs(fname);
//     ofs << "azimuth_deg,distance_m,rssi\n";
//     for (auto &p : scan)
//         ofs << p.azimuth << ',' << p.distance << ',' << unsigned(p.rssi) << '\n';
// }

/* ---------- Main Function ------------------------------------------------ */
int main()
{
    fs::create_directories("scans");

    std::unique_ptr<Serial_Com> serial;
    LidarHandler lidar;
    UDPCom udp(LOCAL_UDP_PORT, REMOTE_IP, REMOTE_UDP_PORT);

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
            serial->spinOnce(g_cmd);

            if (loop_count % 10 == 0)
            {
                // Process LiDAR data  
                g_scan = lidar.getLatestScan();
                lidar.dumpNextScan("scans/Range.txt", g_scan);

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
            }

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
            auto echo = serial->getCommandEcho();

            writePoseData(g_sensor);

            // Send telemetry back over UDP
            if (g_sensor.valid)
                udp.sendSensor(g_sensor);
            if (g_debug.valid)
                udp.sendDebug(g_debug);

            // Print or handle
            // if (g_sensor.valid)
            // {
            //     serial->printSensorData(g_sensor);
            //     std::cout << std::endl;
            // }
            // if (g_debug.valid)
            // {
            //     serial->printDebugData(g_debug);
            //     std::cout << std::endl;
            // }

            if (echo.valid)
            {
                serial->printCommandEcho(echo);
                std::cout << std::endl;
            }
        }

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // never reached, but clean up if we ever break out
    udp.stop();
    return 0;
}
