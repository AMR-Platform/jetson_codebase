// main.cpp
#include "lidar/lidar_handler.hpp"
#include <opencv2/opencv.hpp>
#include "communication/serial_com.hpp"
#include "communication/udp_com.hpp"
#include "localization/SensorFusion.hpp"
#include "localization/robot_utils.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>
#include "core/config.hpp"

namespace fs = boost::filesystem;

// ───── Configuration ─────────────────────────────────────
bool USE_ROBOT_LOCALIZATION = true; // Set to true to use EKF-based localization
// ──────────────────────────────────────────────────────────

// ───── Globals (used by both EKF and Basic modes) ────────────────────────────
SensorPacket g_sensor;
MotionDebugPacket g_debug;
CommandPacket g_cmd;

// ───── EKF Globals ───────────────────────────────────────────────────────────
std::unique_ptr<SensorFusion> g_ekf;
std::ofstream g_ekf_log;
std::chrono::steady_clock::time_point g_start_time;
bool g_ekf_initialized = false;
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

/* ---------- EKF Functions ---------------------------------------------------- */
void initializeEKF()
{
    if (g_ekf_initialized)
        return;

    // Use robot parameters from RobotUtils
    double wheelRadius = RobotUtils::WHEEL_RADIUS;
    double wheelBase = RobotUtils::WHEEL_BASE;
    double dt = 0.01; // 100Hz

    // Initialize EKF with noise parameters
    g_ekf = std::make_unique<SensorFusion>(
        wheelRadius, wheelBase, dt,
        1e-4,  // position noise
        1e-5,  // angular noise
        1e-3,  // velocity noise
        1e-4,  // gyro noise
        1e-3,  // IMU noise
        1e-2   // accel noise
    );

    // Initialize CSV log file
    g_ekf_log.open("ekf_data_log.csv");
    g_ekf_log << "timestamp,x,y,theta,vx,vy,omega,"
              << "leftVel,rightVel,gyroZ,imuYaw,accelX,accelY,"
              << "encL,encR,dEncL,dEncR,yaw,roll,pitch,"
              << "P_xx,P_yy,P_theta,P_vx,P_vy,P_omega,"
              << "loop_count" << std::endl;

    g_start_time = std::chrono::steady_clock::now();
    g_ekf_initialized = true;

    std::cout << "✅ EKF initialized - logging to ekf_data_log.csv" << std::endl;
    std::cout << "   Wheel radius: " << wheelRadius << " m" << std::endl;
    std::cout << "   Wheelbase: " << wheelBase << " m" << std::endl;
    std::cout << "   Update rate: " << 1.0 / dt << " Hz" << std::endl;
}

void processEKFData(const SensorPacket &sensor, int loop_count)
{
    if (!g_ekf_initialized || !sensor.valid)
        return;

    double dt = 0.01; // 100Hz

    // Convert sensor data to EKF inputs
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt);
    double gyroZ = RobotUtils::degToRad(sensor.gyroZ);  // Convert to rad/s
    double imuYaw = RobotUtils::degToRad(sensor.yaw);   // Convert to rad
    double accelX = sensor.accelX;  // Already in m/s²
    double accelY = sensor.accelY;  // Already in m/s²

    // Check sensor availability
    bool hasGyro = (sensor.valid && std::abs(sensor.gyroZ) < 1000.0);
    bool hasIMU = (sensor.valid && std::abs(sensor.yaw) < 360.0);
    bool hasAccel = (sensor.valid && std::abs(sensor.accelX) < 50.0 && std::abs(sensor.accelY) < 50.0);
    bool hasEncoders = sensor.valid;

    // Run sensor fusion step
    g_ekf->sensorFusionStep(leftVel, rightVel,
                           gyroZ, hasGyro,
                           imuYaw, hasIMU,
                           accelX, accelY, hasAccel,
                           hasEncoders);

    // Get current state
    auto pose = g_ekf->getPose();
    auto velocities = g_ekf->getVelocities();
    auto covariance = g_ekf->getCovariance();

    // Calculate elapsed time
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - g_start_time).count();

    // Log data to CSV
    g_ekf_log << std::fixed << std::setprecision(6)
              << elapsed << ","
              << pose[0] << "," << pose[1] << "," << pose[2] << ","
              << velocities[0] << "," << velocities[1] << "," << velocities[2] << ","
              << leftVel << "," << rightVel << ","
              << gyroZ << "," << imuYaw << ","
              << accelX << "," << accelY << ","
              << sensor.encL << "," << sensor.encR << ","
              << sensor.dEncL << "," << sensor.dEncR << ","
              << sensor.yaw << "," << sensor.roll << "," << sensor.pitch << ","
              << covariance(0, 0) << "," << covariance(1, 1) << "," << covariance(2, 2) << ","
              << covariance(3, 3) << "," << covariance(4, 4) << "," << covariance(5, 5) << ","
              << loop_count << std::endl;

    // Print EKF state every 100 loops (~1 second at 100Hz)
    if (loop_count % 100 == 0)
    {
        std::cout << "\n--- EKF State (t=" << std::fixed << std::setprecision(1) << elapsed << "s) ---" << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Position: (" << pose[0] << ", " << pose[1] << ") m" << std::endl;
        std::cout << "Heading:  " << pose[2] << " rad (" << pose[2] * 180.0 / M_PI << "°)" << std::endl;
        std::cout << "Velocity: (" << velocities[0] << ", " << velocities[1] << ") m/s" << std::endl;
        std::cout << "Ang. Vel: " << velocities[2] << " rad/s (" << velocities[2] * 180.0 / M_PI << "°/s)" << std::endl;
        std::cout << "Sensors: Enc=" << (hasEncoders ? "OK" : "NO")
                  << " Gyro=" << (hasGyro ? "OK" : "NO")
                  << " IMU=" << (hasIMU ? "OK" : "NO")
                  << " Accel=" << (hasAccel ? "OK" : "NO") << std::endl;
    }
}

void cleanupEKF()
{
    if (g_ekf_log.is_open())
    {
        g_ekf_log.close();
        std::cout << "EKF log file closed." << std::endl;
    }
}

/* ---------- Main Function ------------------------------------------------ */
int main()
{
    std::unique_ptr<Serial_Com> serial;

    // Check if we should use RobotLocalization (EKF-based localization)
    // if (USE_ROBOT_LOCALIZATION) {

    //     auto ports = Serial_Com::getAvailablePorts();
    //     if (ports.empty()) {
    //         std::cerr << "No serial ports found." << std::endl;
    //         return 1;
    //     }

    //     std::string selectedPort;
    //     for (const auto &port : ports) {
    //         std::cout << "Trying to connect to: " << port << std::endl;
    //         try {
    //             // Test connection
    //             Serial_Com test(port, DEFAULT_BAUD_RATE);
    //             selectedPort = port;
    //             std::cout << "Successfully connected to: " << port << std::endl;
    //             break;
    //         } catch (const std::exception &e) {
    //             std::cout << "Failed to connect to " << port << ": " << e.what() << std::endl;
    //         }
    //     }

    //     if (selectedPort.empty()) {
    //         std::cerr << "Unable to connect to any serial port." << std::endl;
    //         return 1;
    //     }

    //     try {
    //         RobotLocalization robot(selectedPort, true);  // Enable logging

    //         std::cout << "\n✓ Robot localization system started!" << std::endl;
    //         std::cout << "✓ Using EKF with wheel odometry, IMU, and gyroscope fusion" << std::endl;
    //         std::cout << "✓ Data logging enabled - CSV files will be saved" << std::endl;
    //         std::cout << "✓ Status updates every 1 second" << std::endl;
    //         std::cout << "✓ Global variables (g_sensor, g_debug, g_cmd) will be updated" << std::endl;
    //         std::cout << "Press Ctrl+C to exit\n" << std::endl;

    //         // Configure robot for EKF testing
    //         CommandPacket cmd;
    //         cmd.mode = AUTONOMOUS;           // Set to autonomous mode
    //         cmd.dbg = MOTION_DEBUG;          // Enable motion debug for detailed output
    //         cmd.distance = 2000;
    //         cmd.maxVel =300;
    //         cmd.linAcc = 100;
    //         cmd.lastVel =0;
    //         robot.sendCommand(cmd);

    //         std::cout << "Command sent: mode=AUTONOMOUS, debug=MOTION_DEBUG" << std::endl;
    //         std::cout << "Starting main EKF loop...\n" << std::endl;

    //         // Main EKF loop - this will run indefinitely and update globals
    //         robot.spin(cmd);

    //     } catch (const std::exception& e) {
    //         std::cerr << "Error: " << e.what() << std::endl;
    //         return 1;
    //     }

    //     return 0;
    // }

    // // ===== ORIGINAL MAIN FUNCTIONALITY (BASIC SERIAL + LIDAR) =====
    // std::cout << "=== STARTING BASIC SERIAL COMMUNICATION MODE ===" << std::endl;
    // std::cout << "This mode provides basic serial communication and LiDAR support" << std::endl;
    // std::cout << "No EKF or advanced localization features" << std::endl;
    // std::cout << "=================================================" << std::endl;

    auto ports = Serial_Com::getAvailablePorts();

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
    std::cout << "\n=== Starting Main Loop with EKF ===" << std::endl;
    
    // Initialize EKF
    initializeEKF();

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

            // Process EKF data if sensor packet is valid
            if (g_sensor.valid) {
                processEKFData(g_sensor, loop_count);
            }

            // Send telemetry back over UDP
            if (g_sensor.valid)
                udp.sendSensor(g_sensor);
            if (g_debug.valid)
                udp.sendDebug(g_debug);

            // Print command echo if available
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
    cleanupEKF();
    udp.stop();
    return 0;
}