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
#include "core/config.hpp"

namespace fs = boost::filesystem;

// ───── Configuration ─────────────────────────────────────
bool USE_ROBOT_LOCALIZATION = true; // Set to true to use EKF-based localization
// ──────────────────────────────────────────────────────────

// ───── Globals (used by both EKF and Basic modes) ────────────────────────────
SensorPacket g_sensor;
MotionDebugPacket g_debug;
CommandPacket g_cmd;

// ───── EKF/RobotLocalization Globals ─────────────────────────────────────────
std::unique_ptr<RobotLocalization> g_robot_localization;
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

/* ---------- EKF/RobotLocalization Functions --------------------------------- */
void initializeRobotLocalization(const std::string& serialPort)
{
    if (g_ekf_initialized)
        return;

    try {
        // Initialize RobotLocalization with logging enabled
        g_robot_localization = std::make_unique<RobotLocalization>(serialPort, true);
        
        std::cout << "✅ RobotLocalization initialized on " << serialPort << std::endl;
        std::cout << "   EKF with wheel odometry, IMU, and gyroscope fusion" << std::endl;
        std::cout << "   Logging enabled - CSV files will be saved" << std::endl;
        std::cout << "   Integrated with UDP communication" << std::endl;
        
        g_start_time = std::chrono::steady_clock::now();
        g_ekf_initialized = true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize RobotLocalization: " << e.what() << std::endl;
        throw;
    }
}

void processRobotLocalizationStep(int loop_count)
{
    if (!g_ekf_initialized || !g_robot_localization)
        return;

    // Configure robot command
    CommandPacket cmd = g_cmd;  // Use the global command (potentially from UDP)
    cmd.mode = AUTONOMOUS;      // Ensure autonomous mode
    cmd.dbg = MOTION_DEBUG;     // Enable motion debug
    
    // Process one step of RobotLocalization
    // This updates the internal EKF and logs data
    g_robot_localization->processStep(cmd);
    
    // Get updated sensor and debug data from RobotLocalization
    g_sensor = g_robot_localization->getCurrentSensorData();
    g_debug = g_robot_localization->getCurrentDebugData();
    
    // Print EKF state every 100 loops (~1 second at 100Hz)
    if (loop_count % 100 == 0) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - g_start_time).count();
        
        std::cout << "\n--- RobotLocalization Status (t=" << std::fixed << std::setprecision(1) << elapsed << "s) ---" << std::endl;
        std::cout << "Loop: " << loop_count << " | Sensor valid: " << (g_sensor.valid ? "YES" : "NO") 
                  << " | Debug valid: " << (g_debug.valid ? "YES" : "NO") << std::endl;
        
        // Print brief EKF state if available
        if (g_sensor.valid) {
            std::cout << "Raw data: Enc L/R=(" << g_sensor.dEncL << "," << g_sensor.dEncR 
                      << ") | IMU yaw=" << std::setprecision(1) << g_sensor.yaw << "°" << std::endl;
        }
    }
}

void cleanupRobotLocalization()
{
    if (g_robot_localization) {
        std::cout << "RobotLocalization cleanup completed." << std::endl;
        g_robot_localization.reset();
    }
    g_ekf_initialized = false;
}

// Fallback EKF functions (if RobotLocalization fails)
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

    std::cout << "✅ Fallback EKF initialized - logging to ekf_data_log.csv" << std::endl;
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
        std::cout << "\n--- Fallback EKF State (t=" << std::fixed << std::setprecision(1) << elapsed << "s) ---" << std::endl;
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
        std::cout << "Fallback EKF log file closed." << std::endl;
    }
}

/* ---------- Main Function ------------------------------------------------ */
int main()
{
    std::unique_ptr<Serial_Com> serial;
    std::string selectedPort;
    bool useRobotLocalization = USE_ROBOT_LOCALIZATION;

    auto ports = Serial_Com::getAvailablePorts();

    if (ports.empty())
    {
        std::cout << "No serial ports found." << std::endl;
        return 1;
    }

    // Find and connect to a serial port
    for (const auto &port : ports)
    {
        try
        {
            std::cout << "Trying to open serial port: " << port << std::endl;
            
            if (useRobotLocalization) {
                // Test connection for RobotLocalization
                Serial_Com test(port, DEFAULT_BAUD_RATE);
                selectedPort = port;
                std::cout << "Successfully tested port: " << port << " for RobotLocalization" << std::endl;
                break;
            } else {
                // Use direct serial connection
                serial = std::make_unique<Serial_Com>(port, DEFAULT_BAUD_RATE);

                // Set system state for debug + echo
                SystemState sys;
                sys.controlMode = AUTONOMOUS;
                sys.debugMode = MD_AND_ECHO;
                sys.updateExpectations();
                serial->setSystemState(sys);

                std::cout << "Successfully opened serial port: " << port << std::endl;
                selectedPort = port;
                break;
            }
        }
        catch (const std::exception &e)
        {
            std::cout << "Failed to open " << port << ": " << e.what() << std::endl;
        }
    }

    if (selectedPort.empty())
    {
        std::cerr << "Unable to connect to any serial port." << std::endl;
        return 1;
    }

    // Initialize the localization system
    if (useRobotLocalization) {
        try {
            initializeRobotLocalization(selectedPort);
            std::cout << "✅ Using RobotLocalization with UDP integration" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "❌ RobotLocalization failed, falling back to direct EKF" << std::endl;
            useRobotLocalization = false;
            
            // Create direct serial connection as fallback
            serial = std::make_unique<Serial_Com>(selectedPort, DEFAULT_BAUD_RATE);
            SystemState sys;
            sys.controlMode = AUTONOMOUS;
            sys.debugMode = MD_AND_ECHO;
            sys.updateExpectations();
            serial->setSystemState(sys);
            
            initializeEKF();
        }
    } else {
        initializeEKF();
        std::cout << "✅ Using direct EKF with UDP integration" << std::endl;
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
    std::cout << "\n=== Starting Main Loop with " << (useRobotLocalization ? "RobotLocalization" : "Direct EKF") << " + UDP ===" << std::endl;

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
                if (useRobotLocalization && g_robot_localization) {
                    // Send command through RobotLocalization
                    g_robot_localization->sendCommand(g_cmd);
                } else if (serial) {
                    // Send command through direct serial
                    serial->sendCommand(g_cmd);
                }
                g_cmd.cmdStatus = CMD_JUST_WROTE; // mark as sent
                std::cout << "[UDP] Command sent: mode=" << int(g_cmd.mode) << std::endl;
            }

            // Process robot localization or direct serial
            if (useRobotLocalization) {
                // Use RobotLocalization (processes EKF internally)
                processRobotLocalizationStep(loop_count);
            } else {
                // Use direct serial + EKF
                serial->spinOnce(g_cmd);
                g_sensor = serial->getSensor();
                g_debug = serial->getDebug();
                
                // Process EKF data if sensor packet is valid
                if (g_sensor.valid) {
                    processEKFData(g_sensor, loop_count);
                }
            }

            // Send telemetry back over UDP (works with both approaches)
            if (g_sensor.valid)
                udp.sendSensor(g_sensor);
            if (g_debug.valid)
                udp.sendDebug(g_debug);

            // Print command echo if available (direct serial only)
            if (!useRobotLocalization && serial) {
                auto echo = serial->getCommandEcho();
                if (echo.valid)
                {
                    serial->printCommandEcho(echo);
                    std::cout << std::endl;
                }
            }
        }

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // never reached, but clean up if we ever break out
    if (useRobotLocalization) {
        cleanupRobotLocalization();
    } else {
        cleanupEKF();
    }
    udp.stop();
    return 0;
}