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
bool RUN_EKF_TESTS = false; // Set to true to run EKF validation tests
// ──────────────────────────────────────────────────────────

// ===== EKF TEST FUNCTIONS =====
// Simple test data structure
struct TestSensorData {
    double leftVel, rightVel;
    double imuYaw;       // degrees
    double gyroZ;        // rad/s
    double accelX, accelY; // m/s²
    double dt;
    std::string description;
};

void printTestHeader(const std::string& testName) {
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << testName << "\n";
    std::cout << std::string(50, '=') << "\n";
}

void printEKFState(const SensorFusion& ekf, double time, const std::string& context = "") {
    auto pose = ekf.getPose();
    auto velocities = ekf.getVelocities();
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "t=" << std::setw(5) << time << "s ";
    if (!context.empty()) std::cout << "[" << context << "] ";
    std::cout << "Pose: (" << std::setw(6) << pose[0] << "," << std::setw(6) << pose[1] 
              << "," << std::setw(6) << pose[2]*180/M_PI << "°) ";
    std::cout << "Vel: (" << std::setw(5) << velocities[0] << "," << std::setw(5) << velocities[1] 
              << "," << std::setw(5) << velocities[2]*180/M_PI << "°/s)\n";
}

void runEKFStep(SensorFusion& ekf, const TestSensorData& data, bool verbose = false) {
    if (verbose) {
        std::cout << "  Input: L=" << data.leftVel << " R=" << data.rightVel 
                  << " accel=(" << data.accelX << "," << data.accelY << ") "
                  << data.description << "\n";
    }
    
    // Determine if ZUPT should be active based on acceleration
    double accel_magnitude = std::sqrt(data.accelX * data.accelX + data.accelY * data.accelY);
    bool lowMotion = (std::abs(data.leftVel) < 0.05 && std::abs(data.rightVel) < 0.05);
    bool zuptCondition = (lowMotion && accel_magnitude < 0.05);  // Consistent threshold
    
    // Run EKF prediction with ZUPT awareness
    ekf.predictWithZUPT(data.leftVel, data.rightVel, zuptCondition);
    
    // Run updates based on available data
    if (data.imuYaw != 0.0 && std::abs(data.imuYaw) < 360.0) {
        ekf.updateWithIMU(data.imuYaw * M_PI / 180.0);
    }
    
    if (data.gyroZ != 0.0 && std::abs(data.gyroZ) < 10.0) {
        ekf.updateWithGyro(data.gyroZ);
    }
    
    // Apply ZUPT via accelerometer update when conditions are met
    if (zuptCondition) {
        ekf.updateWithAccelerometer(data.accelX, data.accelY);
    }
    
    // Only use encoders if they show reasonable velocities and ZUPT is not active
    if (std::abs(data.leftVel) < 2.0 && std::abs(data.rightVel) < 2.0 && !zuptCondition) {
        if (std::abs(data.leftVel) > 0.01 || std::abs(data.rightVel) > 0.01) {
            ekf.updateWithEncoders(data.leftVel, data.rightVel);
        }
    }
}

void test1_FreeSpinningWheels() {
    printTestHeader("TEST 1: FREE SPINNING WHEELS");
    std::cout << "Problem: Encoders show 1m/s for 2 seconds, but robot is stationary\n";
    std::cout << "Expected: Position should stay near (0,0) due to ZUPT\n\n";
    
    SensorFusion ekf(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    // Simulate 200 steps (2 seconds at 100Hz)
    for (int i = 0; i < 200; i++) {
        TestSensorData data;
        data.leftVel = 1.0;      // Wheels spinning at 1 m/s
        data.rightVel = 1.0;     // Both wheels same speed
        data.imuYaw = 0.0;       // No rotation
        data.gyroZ = 0.0;        // No angular velocity
        data.accelX = 0.01;      // Very low acceleration (stationary)
        data.accelY = 0.01;      // Very low acceleration (stationary)
        data.dt = 0.01;
        data.description = "free spinning";
        
        runEKFStep(ekf, data);
        
        // Print every 50 steps (0.5 seconds)
        if (i % 50 == 0) {
            printEKFState(ekf, i * 0.01, "free spin");
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "\nFINAL RESULT:\n";
    std::cout << "Position: (" << final_pose[0] << ", " << final_pose[1] << ") meters\n";
    std::cout << "SUCCESS CRITERIA: |x| < 0.1 AND |y| < 0.1\n";
    
    bool success = (std::abs(final_pose[0]) < 0.1 && std::abs(final_pose[1]) < 0.1);
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << "\n";
}

void test2_NormalMovement() {
    printTestHeader("TEST 2: NORMAL MOVEMENT");
    std::cout << "Command: 0.5 m/s forward for 4 seconds\n";
    std::cout << "Expected: Position should reach approximately (2.0, 0.0)\n\n";
    
    SensorFusion ekf(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    // Simulate 400 steps (4 seconds at 100Hz)
    for (int i = 0; i < 400; i++) {
        TestSensorData data;
        data.leftVel = 0.5;      // 0.5 m/s forward
        data.rightVel = 0.5;     // 0.5 m/s forward
        data.imuYaw = 0.0;       // No rotation
        data.gyroZ = 0.0;        // No angular velocity
        data.accelX = 0.1;       // Some acceleration (moving)
        data.accelY = 0.05;      // Small Y acceleration
        data.dt = 0.01;
        data.description = "normal forward";
        
        runEKFStep(ekf, data);
        
        // Print every 100 steps (1 second)
        if (i % 100 == 0) {
            printEKFState(ekf, i * 0.01, "forward");
        }
    }
    
    auto final_pose = ekf.getPose();
    std::cout << "\nFINAL RESULT:\n";
    std::cout << "Position: (" << final_pose[0] << ", " << final_pose[1] << ") meters\n";
    std::cout << "SUCCESS CRITERIA: 1.8 < x < 2.2 AND |y| < 0.2\n";
    
    bool success = (final_pose[0] > 1.8 && final_pose[0] < 2.2 && std::abs(final_pose[1]) < 0.2);
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << "\n";
}

void test3_StopAndGo() {
    printTestHeader("TEST 3: STOP AND GO");
    std::cout << "Scenario: Move 1s, stop 1s, move 1s, stop 1s\n";
    std::cout << "Expected: Velocity should drop to zero during stops\n\n";
    
    SensorFusion ekf(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    // Simulate 400 steps (4 seconds)
    for (int i = 0; i < 400; i++) {
        double t = i * 0.01;
        bool moving = (fmod(t, 2.0) < 1.0);  // Move for 1s, stop for 1s
        
        TestSensorData data;
        data.leftVel = moving ? 0.5 : 0.0;
        data.rightVel = moving ? 0.5 : 0.0;
        data.imuYaw = 0.0;
        data.gyroZ = 0.0;
        data.accelX = moving ? 0.1 : 0.005;  // Low accel when stopped
        data.accelY = moving ? 0.05 : 0.005;
        data.dt = 0.01;
        data.description = moving ? "moving" : "stopped";
        
        runEKFStep(ekf, data);
        
        // Print at phase transitions
        if (i % 100 == 0 || i % 100 == 99) {
            printEKFState(ekf, t, data.description);
        }
    }
    
    auto final_velocities = ekf.getVelocities();
    std::cout << "\nFINAL RESULT:\n";
    std::cout << "Final velocity: (" << final_velocities[0] << ", " << final_velocities[1] << ") m/s\n";
    std::cout << "SUCCESS CRITERIA: |vx| < 0.1 AND |vy| < 0.1 (should be stopped)\n";
    
    bool success = (std::abs(final_velocities[0]) < 0.1 && std::abs(final_velocities[1]) < 0.1);
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << "\n";
}

void test4_ZUPTEffectiveness() {
    printTestHeader("TEST 4: ZUPT EFFECTIVENESS");
    std::cout << "Compare: High accel (no ZUPT) vs Low accel (ZUPT active)\n\n";
    
    // Test A: High acceleration (ZUPT should NOT trigger)
    std::cout << "4A: High acceleration (accel = 0.5 m/s²) - ZUPT should be INACTIVE\n";
    SensorFusion ekf_a(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    for (int i = 0; i < 100; i++) {
        TestSensorData data;
        data.leftVel = 0.5; data.rightVel = 0.5;
        data.imuYaw = 0.0; data.gyroZ = 0.0;
        data.accelX = 0.5; data.accelY = 0.3;  // High acceleration
        data.dt = 0.01;
        runEKFStep(ekf_a, data);
    }
    auto pose_a = ekf_a.getPose();
    std::cout << "Result: Position = (" << pose_a[0] << ", " << pose_a[1] << ")\n";
    
    // Test B: Low acceleration (ZUPT should trigger)
    std::cout << "\n4B: Low acceleration (accel = 0.01 m/s²) - ZUPT should be ACTIVE\n";
    SensorFusion ekf_b(0.065, 0.3, 0.01, 1e-2, 1e-3, 1e-2, 1e-3, 1e-2, 1e-1);
    
    for (int i = 0; i < 100; i++) {
        TestSensorData data;
        data.leftVel = 0.5; data.rightVel = 0.5;  // Same encoder readings
        data.imuYaw = 0.0; data.gyroZ = 0.0;
        data.accelX = 0.01; data.accelY = 0.01;  // Low acceleration
        data.dt = 0.01;
        runEKFStep(ekf_b, data);
    }
    auto pose_b = ekf_b.getPose();
    std::cout << "Result: Position = (" << pose_b[0] << ", " << pose_b[1] << ")\n";
    
    std::cout << "\nCOMPARISON:\n";
    std::cout << "High accel distance: " << sqrt(pose_a[0]*pose_a[0] + pose_a[1]*pose_a[1]) << "m\n";
    std::cout << "Low accel distance:  " << sqrt(pose_b[0]*pose_b[0] + pose_b[1]*pose_b[1]) << "m\n";
    std::cout << "SUCCESS: Low accel distance should be much smaller (ZUPT working)\n";
    
    double ratio = sqrt(pose_b[0]*pose_b[0] + pose_b[1]*pose_b[1]) / sqrt(pose_a[0]*pose_a[0] + pose_a[1]*pose_a[1]);
    bool success = (ratio < 0.5);  // ZUPT should reduce distance by at least 50%
    std::cout << "RESULT: " << (success ? "✓ PASS" : "✗ FAIL") << " (ratio = " << ratio << ")\n";
}

void saveTestResults() {
    // Create a simple CSV for analysis
    std::ofstream file("ekf_test_results.csv");
    file << "test,step,time,x,y,theta,vx,vy,omega,left_vel,right_vel,accel_x,accel_y,notes\n";
    
    std::cout << "\nTest results framework created.\n";
    std::cout << "For detailed logging, modify the test functions to write to 'ekf_test_results.csv'\n";
    file.close();
}

void runEKFValidationTests() {
    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║           EKF VALIDATION TESTS               ║\n";
    std::cout << "║     Testing the issues you encountered      ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";
    
    // Run all tests
    test1_FreeSpinningWheels();
    test2_NormalMovement();
    test3_StopAndGo();
    test4_ZUPTEffectiveness();
    
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "ALL TESTS COMPLETED\n";
    std::cout << "Check the results above to verify EKF behavior\n";
    std::cout << std::string(60, '=') << "\n";
    
    saveTestResults();
}

// ===== END EKF TEST FUNCTIONS =====

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
    // Check if we should run EKF validation tests
    if (RUN_EKF_TESTS) {
        runEKFValidationTests();
        return 0;
    }

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
