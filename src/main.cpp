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
SensorPacket            g_sensor;
MotionDebugPacket       g_debug;
CommandEchoPacket       echo;         // used for debugging

std::mutex              g_cmd_mtx;    // for thread‐safing g_cmd access
CommandPacket           g_cmd;

std::vector<LidarPoint> g_scan;       // holds the latest LiDAR scan data

// EKF instance for localization (tuned noise parameters)
SensorFusion g_ekf(
    RobotUtils::WHEEL_RADIUS,
    RobotUtils::WHEEL_BASE,
    0.01,    // dt
    1e-2,    // processNoisePos
    1e-3,    // processNoiseAng
    1e-2,    // processNoiseVel
    1e-3,    // gyroNoise
    1e-2,    // imuNoise
    1e-1     // accelNoise
);
// ──────────────────────────────────────────────────────────────────────────────
OccupancyGrid g_occupancy_grid(50.0f, 50.0f, 0.05f, -25.0f, -25.0f);  // 50x50m map, 5cm resolution
std::mutex    g_map_mtx;
int           g_map_update_counter = 0;

/* ---------- Global Access Functions ------------------------------------------- */
const SensorPacket &getCurrentSensorData()  { return g_sensor; }
const MotionDebugPacket &getCurrentDebugData() { return g_debug; }
const CommandPacket &getCurrentCommand() {
    std::lock_guard<std::mutex> lk(g_cmd_mtx);
    return g_cmd;
}

/* ---------- LiDAR utility functions ------------------------------------------------ */
std::string timestamp() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto t   = clock::to_time_t(now);
    auto us  = std::chrono::duration_cast<std::chrono::microseconds>(
                   now.time_since_epoch()).count() % 1'000'000;
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
    const cv::Point c(canvas/2, canvas/2);
    const float ppm = (canvas/2)/rangeMax;
    for (auto &p : scan) {
        if (p.distance <= 0.01f || p.distance > rangeMax) continue;
        float rad = p.azimuth * CV_PI/180.f;
        cv::Point pt(c.x + std::cos(rad)*p.distance*ppm,
                     c.y - std::sin(rad)*p.distance*ppm);
        cv::circle(img, pt, 0.5, cv::Scalar(0,255,0), cv::FILLED);
    }
}

void saveCSV(const std::string &fname, const std::vector<LidarPoint> &scan) {
    std::ofstream ofs(fname);
    ofs << "azimuth_deg,distance_m,rssi\n";
    for (auto &p : scan)
        ofs << p.azimuth << ',' << p.distance << ',' << unsigned(p.rssi) << '\n';
}


RobotPose getRobotPoseFromEKF(const SensorFusion& ekf) {
    // Extract pose from your EKF - you'll need to adapt this to your SensorFusion interface
    // This is a placeholder - replace with actual EKF state extraction
    RobotPose pose;
    
    // Assuming your EKF has methods to get state
    // You may need to modify this based on your SensorFusion implementation
    auto state = ekf.getState();  // You'll need to implement this getter
    pose.x = state.x;      // position x
    pose.y = state.y;      // position y  
    pose.theta = state.theta; // orientation
    
    return pose;
}

void updateOccupancyGrid(const std::vector<LidarPoint>& scan, const RobotPose& robot_pose) {
    std::lock_guard<std::mutex> lock(g_map_mtx);
    g_occupancy_grid.updateGrid(scan, robot_pose);
}

void saveMapPeriodically(int counter) {
    if (counter % 1000 == 0) {  // Save every 1000 updates (about every 10 seconds at 100Hz)
        std::lock_guard<std::mutex> lock(g_map_mtx);
        
        std::string ts = timestamp();
        std::string map_file = "maps/occupancy_grid_" + ts + ".map";
        std::string img_file = "maps/occupancy_grid_" + ts + ".png";
        
        g_occupancy_grid.saveToFile(map_file);
        
        cv::Mat map_img = g_occupancy_grid.toImage();
        cv::imwrite(img_file, map_img);
        
        std::cout << "Saved map: " << map_file << std::endl;
    }
}



/* ---------- Main Function ------------------------------------------------ */
int main()
{
    fs::create_directories("scans");

    std::unique_ptr<Serial_Com> serial;
    LidarHandler               lidar;
    UDPCom                     udp(LOCAL_UDP_PORT, REMOTE_IP, REMOTE_UDP_PORT);
    RobotLocalization          localize;    // default-construct with logging on

    // ── NEW: record chosen serial port for RobotLocalization
    std::string serialPortString;

    auto ports = Serial_Com::getAvailablePorts();
    udp.start();
    //cv::namedWindow("Lidar Viewer");
    cv::namedWindow("Lidar Viewer", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Occupancy Grid", cv::WINDOW_AUTOSIZE);

    if (ports.empty()) {
        std::cout << "No serial ports found." << std::endl;
        return 1;
    }

    for (const auto &port : ports) {
        try {
            std::cout << "Trying to open serial port: " << port << std::endl;
            serial = std::make_unique<Serial_Com>(port, DEFAULT_BAUD_RATE);

            // ── NEW: save the successful port name
            serialPortString = port;

            // Set system state for debug + echo
            SystemState sys;
            sys.controlMode = AUTONOMOUS;
            sys.debugMode   = MD_AND_ECHO;
            sys.updateExpectations();
            serial->setSystemState(sys);

            std::cout << "Successfully opened serial port: " << port << std::endl;
            break;
        }
        catch (const std::exception &e) {
            std::cout << "Failed to open " << port << ": " << e.what() << std::endl;
        }
    }

    if (!serial) {
        std::cerr << "Unable to connect to any serial port." << std::endl;
        return 1;
    }

    auto next       = std::chrono::steady_clock::now();
    int  loop_count = 0;
    // define your loop period explicitly
    constexpr auto period = std::chrono::milliseconds(10);

    std::cout << "Starting mapping loop..." << std::endl;

    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        if (now >= next)
        {
            next += period;
            ++loop_count;

            // if (loop_count % 10 == 0)
            // {
            //     // Process LiDAR data  
            //     g_scan = lidar.getLatestScan();
            //     lidar.dumpNextScan("scans/Range.txt", g_scan);

            //     if (!g_scan.empty()) {
            //         cv::Mat img;
            //         scanToImage(g_scan, img, rangeMax, canvas);
            //         cv::imshow("LakiBeam Viewer", img);
            //         cv::waitKey(1);
            //         std::string ts = timestamp();
            //         //cv::imwrite("scans/" + ts + ".jpg", img);
            //         //saveCSV("scans/" + ts + ".csv", g_scan);
            //     }
            // }

            g_scan = lidar.getLatestScan();
            
            if (loop_count % 5 == 0 && !g_scan.empty()) // Update visualization every 50ms
            {
                cv::Mat lidar_img;
                scanToImage(g_scan, lidar_img, 20.0f, 600);  // 20m range
                cv::imshow("Lidar Viewer", lidar_img);
                cv::waitKey(1);
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
            g_debug  = serial->getDebug();
            auto echoPkt = serial->getCommandEcho();

            // Send telemetry back over UDP
            if (g_sensor.valid)
                udp.sendSensor(g_sensor);
            if (g_debug.valid)
                udp.sendDebug(g_debug);

            // Run our localization/EKF, then status & logging
            localize.updateEKF(g_sensor);

                        if (g_sensor.valid && !g_scan.empty()) {
                try {
                    RobotPose robot_pose = getRobotPoseFromEKF(g_ekf);
                    updateOccupancyGrid(g_scan, robot_pose);
                    g_map_update_counter++;
                    
                    // Update map visualization every 100ms
                    if (loop_count % 10 == 0) {
                        std::lock_guard<std::mutex> lock(g_map_mtx);
                        cv::Mat map_img = g_occupancy_grid.toImage();
                        
                        // Draw robot position on map
                        int robot_grid_x, robot_grid_y;
                        if (g_occupancy_grid.worldToGrid(robot_pose.x, robot_pose.y, robot_grid_x, robot_grid_y)) {
                            cv::circle(map_img, cv::Point(robot_grid_x, robot_grid_y), 3, cv::Scalar(0, 0, 255), -1);
                            
                            // Draw robot orientation
                            float arrow_length = 5.0f;
                            int end_x = robot_grid_x + static_cast<int>(arrow_length * std::cos(robot_pose.theta));
                            int end_y = robot_grid_y + static_cast<int>(arrow_length * std::sin(robot_pose.theta));
                            cv::arrowedLine(map_img, cv::Point(robot_grid_x, robot_grid_y), 
                                          cv::Point(end_x, end_y), cv::Scalar(0, 0, 255), 2);
                        }
                        
                        cv::imshow("Occupancy Grid", map_img);
                        cv::waitKey(1);
                    }
                    
                    // Save map periodically
                    saveMapPeriodically(g_map_update_counter);
                    
                } catch (const std::exception& e) {
                    std::cerr << "Error updating occupancy grid: " << e.what() << std::endl;
                }
            }


            if (echoPkt.valid) {
                serial->printCommandEcho(echoPkt);
                std::cout << std::endl;
            }

            if (loop_count % 1000 == 0) {  // Every 10 seconds
                std::cout << "Mapping stats - Updates: " << g_map_update_counter 
                         << ", Grid size: " << g_occupancy_grid.getWidth() 
                         << "x" << g_occupancy_grid.getHeight() << std::endl;
            }


            localize.printStatus(g_sensor);
            localize.logData(g_sensor);
        }

         int key = cv::waitKey(1) & 0xFF;
        if (key == 27) { // ESC key
            std::cout << "Exiting..." << std::endl;
            break;
        } else if (key == 's') { // Save map manually
            std::lock_guard<std::mutex> lock(g_map_mtx);
            std::string ts = timestamp();
            std::string map_file = "maps/manual_save_" + ts + ".map";
            std::string img_file = "maps/manual_save_" + ts + ".png";
            
            g_occupancy_grid.saveToFile(map_file);
            cv::Mat map_img = g_occupancy_grid.toImage();
            cv::imwrite(img_file, map_img);
            std::cout << "Manually saved map: " << map_file << std::endl;
        } else if (key == 'c') { // Clear map
            std::lock_guard<std::mutex> lock(g_map_mtx);
            g_occupancy_grid.clear();
            g_map_update_counter = 0;
            std::cout << "Cleared occupancy grid" << std::endl;
        }

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // never reached, but clean up if we ever break out
    udp.stop();
    cv::destroyAllWindows();

    std::lock_guard<std::mutex> lock(g_map_mtx);
    std::string final_map = "maps/final_map_" + timestamp() + ".map";
    g_occupancy_grid.saveToFile(final_map);
    std::cout << "Final map saved: " << final_map << std::endl;
    
    
    return 0;
}
