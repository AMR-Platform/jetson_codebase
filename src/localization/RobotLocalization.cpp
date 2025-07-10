#include "RobotLocalization.hpp"
#include <cmath>

// use your global EKF instance
extern SensorFusion g_ekf;

RobotLocalization::RobotLocalization(bool enableLogging)
  : dt_(DEFAULT_DT),                // <-- use the class’s own DEFAULT_DT
    enableLogging_(enableLogging)
{
    lastUpdate_ = std::chrono::steady_clock::now();
    if (enableLogging_) {
        auto ts = getCurrentTimestamp();
        logFile_.open("outputs/robot_pose_" + ts + ".csv");
        logFile_ << "timestamp,x,y,theta,vx,vy,omega,leftVel,rightVel,imuYaw,gyroZ,accelX,accelY\n";
    }
}

RobotLocalization::~RobotLocalization() {
    if (logFile_.is_open())
        logFile_.close();
}

void RobotLocalization::updateEKF(const SensorPacket& sensor) {
    // compute loop dt
    auto now = std::chrono::steady_clock::now();
    double delta = std::chrono::duration<double>(now - lastUpdate_).count();
    lastUpdate_ = now;
    if      (delta > 0.1)   delta = dt_;   // reset if too large
    else if (delta < 0.001) delta = 0.001; // floor
    dt_ = delta;

    // wheel velocities
    auto [leftVel, rightVel] = RobotUtils::getWheelVelocities(sensor, dt_);

    // ZUPT condition
    double accelMag = std::sqrt(sensor.accelX*sensor.accelX + sensor.accelY*sensor.accelY);
    bool lowMotion      = (std::abs(leftVel) < 0.05 && std::abs(rightVel) < 0.05);
    bool reasonableAccel = (accelMag >= 0.0 && accelMag < 0.5);
    bool zuptActive     = (lowMotion && reasonableAccel && accelMag < 0.05);

    // 1) predict
    g_ekf.predictWithZUPT(leftVel, rightVel, zuptActive);

    // 2) IMU yaw
    if (sensor.yaw != 0.0f && std::abs(sensor.yaw) < 360.0f)
        g_ekf.updateWithIMU(RobotUtils::degToRad(sensor.yaw));

    // 3) gyro Z
    if (sensor.gyroZ != 0.0f && std::abs(sensor.gyroZ) < 10.0)
        g_ekf.updateWithGyro(sensor.gyroZ);

    // 4) ZUPT accelerometer
    if (zuptActive)
        g_ekf.updateWithAccelerometer(sensor.accelX, sensor.accelY);

    // 5) encoder update
    bool encOk = (std::abs(leftVel) < 2.0 && std::abs(rightVel) < 2.0);
    if (encOk && !zuptActive && (std::abs(leftVel)>0.01 || std::abs(rightVel)>0.01))
        g_ekf.updateWithEncoders(leftVel, rightVel);
}

void RobotLocalization::printStatus(const SensorPacket& sensor) {
    auto pose       = g_ekf.getPose();
    auto vel        = g_ekf.getVelocities();
    auto [lVel,rVel]= RobotUtils::getWheelVelocities(sensor, dt_);
    double accelMag = std::sqrt(sensor.accelX*sensor.accelX + sensor.accelY*sensor.accelY);
    double avgVel   = (lVel + rVel)/2.0;
    bool encOk      = (std::abs(lVel) < 2.0 && std::abs(rVel) < 2.0);
    bool lowMotion  = (std::abs(lVel)<0.05 && std::abs(rVel)<0.05);
    bool zuptActive = (lowMotion && accelMag < 0.05);

    std::cout << "\n========== ROBOT STATUS ==========\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "POSE: x=" << pose[0] << "m, y=" << pose[1] << "m, θ="
              << RobotUtils::radToDeg(pose[2]) << "°\n";
    std::cout << "VELOCITIES: vx=" << vel[0] << "m/s, vy=" << vel[1]
              << "m/s, ω=" << RobotUtils::radToDeg(vel[2]) << "°/s\n";
    std::cout << "ENCODERS: L=" << lVel << "m/s, R=" << rVel
              << "m/s (avg=" << avgVel << ")\n";
    std::cout << "SENSORS: yaw=" << sensor.yaw << "°, gyroZ=" << sensor.gyroZ << "rad/s\n";
    std::cout << "ACCEL: X=" << sensor.accelX << "m/s², Y=" << sensor.accelY
              << "m/s² (mag=" << accelMag << ")\n";
    std::cout << "DIAGNOSTICS: dt=" << dt_ << "s, Battery=" << sensor.vbat1 << "mV\n";
    std::cout << "STATUS: Encoders=" << (encOk ? "OK" : "UNRELIABLE")
              << ", Motion=" << (lowMotion ? "LOW" : "HIGH")
              << ", ZUPT="   << (zuptActive ? "ACTIVE" : "INACTIVE") << "\n";
    std::cout << "=================================\n" << std::endl;
}

void RobotLocalization::logData(uint32_t ts, const SensorPacket& sensor) {
    if (!logFile_.is_open()) return;

    auto pose        = g_ekf.getPose();
    auto vel         = g_ekf.getVelocities();
    auto [lVel,rVel] = RobotUtils::getWheelVelocities(sensor, dt_);


    logFile_ << ts << ","
             << pose[0] << "," << pose[1] << "," << pose[2] << ","
             << vel[0]  << "," << vel[1]  << "," << vel[2]  << ","
             << lVel    << "," << rVel    << ","
             << sensor.yaw << "," << sensor.gyroZ << ","
             << sensor.accelX << "," << sensor.accelY << "\n";
    logFile_.flush();
}

std::string RobotLocalization::getCurrentTimestamp() {
    auto t = std::chrono::system_clock::to_time_t(
                 std::chrono::system_clock::now());
    std::tm tm = *std::localtime(&t);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return ss.str();
}

std::array<double,3> RobotLocalization::getPose() const {
    return g_ekf.getPose();
}
std::array<double,3> RobotLocalization::getVelocities() const {
    return g_ekf.getVelocities();
}
