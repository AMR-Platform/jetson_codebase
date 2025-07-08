#include "serial_com.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <thread>
#include <chrono>
#include <dirent.h>

bool Serial_Com::deltaInit = false;
float Serial_Com::lastYaw = 0.0f;
long Serial_Com::lastEncL = 0;
long Serial_Com::lastEncR = 0;

/* ——————————— ctor / dtor ——————————— */
Serial_Com::Serial_Com(const std::string &port, int baud)
{
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("open");
        throw std::runtime_error("serial open failed");
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        ::close(fd);
        throw std::runtime_error("tcgetattr failed");
    }

    cfsetospeed(&tty, baudToTermios(baud));
    cfsetispeed(&tty, baudToTermios(baud));
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        ::close(fd);
        throw std::runtime_error("tcsetattr failed");
    }
    tcflush(fd, TCIOFLUSH);
    std::cout << "[SERIAL] Opened port " << port << " at " << baud << " baud\n";
}

Serial_Com::~Serial_Com()
{
    if (fd >= 0)
    {
        std::cout << "[SERIAL] Closing serial port\n";
        ::close(fd);
    }
}

/* ——————————— public API ——————————— */
void Serial_Com::spinOnce(CommandPacket &cmd)
{
    if (fd < 0)
        return;
    char buffer[256];
    int bytes = ::read(fd, buffer, sizeof(buffer) - 1);
    if (bytes > 0)
    {
        for (int i = 0; i < bytes; ++i)
        {
            char ch = buffer[i];
            if (ch == '\n' || ch == '\r')
            {
                if (!rxBuf.empty())
                {
                    std::cout << "[RAW] " << rxBuf << std::endl;
                    bool handled = false;

                    // 1) command-echo right after send
                    if (sysState.expectCommandEcho && cmd.cmdStatus == CMD_JUST_WROTE)
                    {
                        if (parseCommandEcho(rxBuf))
                        {
                            cmd.cmdStatus = CMD_EMPTY;
                            handled = true;
                        }
                    }
                    // 2) debug+telemetry combined
                    if (!handled && sysState.expectDebugData)
                    {
                        handled = parseCombinedLine(rxBuf);
                    }
                    // 3) echo fallback
                    if (!handled && sysState.expectCommandEcho)
                    {
                        handled = parseCommandEcho(rxBuf);
                        if (handled)
                            cmd.cmdStatus = CMD_EMPTY;
                    }
                    // 4) pure telemetry
                    if (!handled)
                        parseTelemetryOnly(rxBuf);

                    rxBuf.clear();
                }
            }
            else if (ch >= 32 && ch <= 126)
            {
                rxBuf += ch;
            }
        }
    }
    else if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        perror("read");
    }
}

void Serial_Com::sendCommand(CommandPacket &cmd)
{
    if (fd < 0)
        return;
    std::ostringstream oss;
    if (cmd.mode == AUTONOMOUS)
    {
        oss << int(cmd.mode) << ',' << int(cmd.dbg) << ','
            << cmd.distance << ',' << cmd.angle << ','
            << cmd.maxVel << ',' << cmd.maxOmega << ','
            << cmd.lastVel << ',' << cmd.lastOmega << ','
            << cmd.linAcc << ',' << cmd.angAcc << "\r\n";
    }
    else
    {
        oss << int(cmd.mode) << ',' << int(cmd.dbg) << ",0,0,"
            << int(cmd.f) << ',' << int(cmd.b) << ',' << int(cmd.l) << ',' << int(cmd.r) << ",0,0\r\n";
    }
    auto str = oss.str();
    std::cout << "[SEND] " << str;
    writeLine(str);
    cmd.cmdStatus = CMD_JUST_WROTE;

    {
        std::scoped_lock lk(mtx);
        sysState.controlMode = cmd.mode;
        sysState.debugMode = cmd.dbg;
        sysState.updateExpectations();
    }
}

SensorPacket Serial_Com::getSensor() const
{
    std::scoped_lock lk(mtx);
    return sensor;
}
MotionDebugPacket Serial_Com::getDebug() const
{
    std::scoped_lock lk(mtx);
    return debug;
}
CommandEchoPacket Serial_Com::getCommandEcho() const
{
    std::scoped_lock lk(mtx);
    return cmdEcho;
}

void Serial_Com::setSystemState(const SystemState &s)
{
    std::scoped_lock lk(mtx);
    sysState = s;
    sysState.updateExpectations();
}
SystemState Serial_Com::getSystemState() const
{
    std::scoped_lock lk(mtx);
    return sysState;
}

/* ——————————— parsers ——————————— */
bool Serial_Com::parseCombinedLine(const std::string &line)
{
    // 7 debug floats + 9 telem floats + 2 longs + 7 ints = 25
    float spdL, spdR, vel, omg, dist, ang, dt;
    float yaw, roll, pitch, gX, gY, gZ, aX, aY, aZ;
    long eL, eR;
    unsigned v1, v2, cL, cC, cR, em, pr;
    int p = std::sscanf(
        line.c_str(),
        "%f %f %f %f %f %f %f "
        "%f %f %f %f %f %f %f %f %f "
        "%ld %ld %u %u %u %u %u %u %u",
        &spdL, &spdR, &vel, &omg, &dist, &ang, &dt,
        &yaw, &roll, &pitch, &gX, &gY, &gZ, &aX, &aY, &aZ,
        &eL, &eR, &v1, &v2, &cL, &cC, &cR, &em, &pr);
    if (p != 25)
        return false;

    MotionDebugPacket d{spdL, spdR, vel, omg, dist, ang, dt, true};
    SensorPacket s;
    s.yaw = yaw;
    s.roll = roll;
    s.pitch = pitch;
    s.gyroX = gX;
    s.gyroY = gY;
    s.gyroZ = gZ;
    s.accelX = aX;
    s.accelY = aY;
    s.accelZ = aZ;
    s.encL = eL;
    s.encR = eR;
    s.vbat1 = v1;
    s.vbat2 = v2;
    s.cliffL = cL;
    s.cliffC = cC;
    s.cliffR = cR;
    s.emergency = uint8_t(em);
    s.profileDone = uint8_t(pr);
    s.valid = true;

    updateSensorDeltas(s);
    {
        std::scoped_lock lk(mtx);
        debug = d;
        sensor = s;
    }
    std::cout << "[PARSED] Combined debug+telemetry data\n";
    return true;
}

bool Serial_Com::parseTelemetryOnly(const std::string &line)
{
    // 18 fields: 9 floats, 2 longs, 7 ints
    float yaw, roll, pitch, gX, gY, gZ, aX, aY, aZ;
    long eL, eR;
    unsigned v1, v2, cL, cC, cR, em, pr;
    int p = std::sscanf(
        line.c_str(),
        "%f %f %f %f %f %f %f %f %f %ld %ld %u %u %u %u %u %u %u",
        &yaw, &roll, &pitch, &gX, &gY, &gZ, &aX, &aY, &aZ,
        &eL, &eR, &v1, &v2, &cL, &cC, &cR, &em, &pr);
    if (p != 18)
    {
        std::cout << "[PARSE] Failed telemetry: " << line << "\n";
        return false;
    }

    SensorPacket s;
    s.yaw = yaw;
    s.roll = roll;
    s.pitch = pitch;
    s.gyroX = gX;
    s.gyroY = gY;
    s.gyroZ = gZ;
    s.accelX = aX;
    s.accelY = aY;
    s.accelZ = aZ;
    s.encL = eL;
    s.encR = eR;
    s.vbat1 = v1;
    s.vbat2 = v2;
    s.cliffL = cL;
    s.cliffC = cC;
    s.cliffR = cR;
    s.emergency = uint8_t(em);
    s.profileDone = uint8_t(pr);
    s.valid = true;

    updateSensorDeltas(s);
    {
        std::scoped_lock lk(mtx);
        sensor = s;
    }
    std::cout << "[PARSED] Telemetry only\n";
    return true;
}

bool Serial_Com::parseDebugOnly(const std::string &line)
{
    float spdL, spdR, vel, omg, dist, ang, dt;
    if (std::sscanf(line.c_str(), "%f %f %f %f %f %f %f",
                    &spdL, &spdR, &vel, &omg, &dist, &ang, &dt) != 7)
        return false;
    MotionDebugPacket d{spdL, spdR, vel, omg, dist, ang, dt, true};
    {
        std::scoped_lock lk(mtx);
        debug = d;
    }
    std::cout << "[PARSED] Debug only\n";
    return true;
}

bool Serial_Com::parseCommandEcho(const std::string &line)
{
    if (line.rfind("CMD", 0) != 0)
        return false;
    CommandEchoPacket e{};
    if (std::sscanf(
            line.c_str(),
            "CMD d=%f a=%f vmax=%hu wmax=%hu vend=%hu wend=%hu acc=%f aacc=%f",
            &e.distance, &e.angle,
            &e.maxVel, &e.maxOmega,
            &e.lastVel, &e.lastOmega,
            &e.linAcc, &e.angAcc) != 8)
        return false;
    e.valid = true;
    {
        std::scoped_lock lk(mtx);
        cmdEcho = e;
    }
    std::cout << "[PARSED] Command echo\n";
    return true;
}

/* ——————————— utilities ——————————— */
void Serial_Com::updateSensorDeltas(SensorPacket &s)
{
    if (deltaInit)
    {
        s.dYaw = s.yaw - lastYaw;
        if (s.dYaw > 180)
            s.dYaw -= 360;
        if (s.dYaw < -180)
            s.dYaw += 360;
        s.dEncL = float(s.encL - lastEncL);
        s.dEncR = float(s.encR - lastEncR);
        s.linVel = (s.dEncL + s.dEncR) / 2;
        s.angVel = (s.dEncR - s.dEncL);
    }
    else
    {
        s.dYaw = s.dEncL = s.dEncR = s.linVel = s.angVel = 0;
        deltaInit = true;
    }
    lastYaw = s.yaw;
    lastEncL = s.encL;
    lastEncR = s.encR;
}

void Serial_Com::writeLine(const std::string &line)
{
    if (fd < 0)
        return;
    ::write(fd, line.data(), line.size());
    fsync(fd);
}

speed_t Serial_Com::baudToTermios(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        return B9600;
    }
}

std::vector<std::string> Serial_Com::getAvailablePorts()
{
    std::vector<std::string> ports;
    for (auto pat : {"/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyS"})
        for (int i = 0; i < 10; i++)
        {
            std::string p = pat + std::to_string(i);
            int fd = ::open(p.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0)
            {
                ports.push_back(p);
                ::close(fd);
            }
        }
    return ports;
}

/* ———————————  debug and testing functions  ——————————— */
void Serial_Com::testCommunication()
{
    std::cout << "\n=== Testing Serial Communication ===" << std::endl;

    // Test 1: Enable motion debug mode
    CommandPacket cmd;
    cmd.mode = AUTONOMOUS;
    cmd.dbg = MOTION_DEBUG;
    cmd.distance = 0;
    cmd.angle = 0;
    cmd.maxVel = 0;
    cmd.maxOmega = 0;
    cmd.lastVel = 0;
    cmd.lastOmega = 0;
    cmd.linAcc = 0;
    cmd.angAcc = 0;

    std::cout << "Sending command to enable MOTION_DEBUG..." << std::endl;
    sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Test 2: Send a simple movement command
    cmd.dbg = MD_AND_ECHO; // Enable both motion debug and echo
    cmd.distance = 1000.0f; // 1000mm forward
    cmd.angle = 0.0f;
    cmd.maxVel = 200; // 200 mm/s
    cmd.maxOmega = 0;
    cmd.lastVel = 0;
    cmd.lastOmega = 0;
    cmd.linAcc = 100.0f; // 100 mm/s²
    cmd.angAcc = 0.0f;

    std::cout << "Sending movement command (100mm forward)..." << std::endl;
    sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // Test 3: Send a rotation command
    cmd.distance = 0.0f;
    cmd.angle = 180.0f;  // 180 deg turn
    cmd.maxVel = 0;     // no linear
    cmd.maxOmega = 120; // 120 deg/s
    cmd.linAcc = 0.0f;
    cmd.angAcc = 360.0f; // 360 deg/s²

    std::cout << "Sending rotation command (90 degrees)..." << std::endl;
    sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // Test 4: Test teleoperator mode
    cmd.mode = TELEOPERATOR;
    cmd.dbg = RX_ECHO;
    cmd.f = 1; // forward
    cmd.b = 0;
    cmd.l = 0;
    cmd.r = 0;

    std::cout << "Sending teleoperator command (forward)..." << std::endl;
    sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "Communication test complete." << std::endl;
}

void Serial_Com::printSensorData(const SensorPacket &sensor) const
{
    if (!sensor.valid)
        return;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "IMU: Y:" << sensor.yaw << "° R:" << sensor.roll
              << "° P:" << sensor.pitch << "° ";
    std::cout << "Gyro: X:" << sensor.gyroX << " Y:" << sensor.gyroY
              << " Z:" << sensor.gyroZ << " ";
    std::cout << "Accel: X:" << sensor.accelX << " Y:" << sensor.accelY
              << " Z:" << sensor.accelZ << " ";
    std::cout << "Enc: L:" << sensor.encL << " R:" << sensor.encR << " ";
    std::cout << "Bat: " << sensor.vbat1 << "mV/" << sensor.vbat2 << "mV ";
    std::cout << "Cliff: L:" << sensor.cliffL << " C:" << sensor.cliffC
              << " R:" << sensor.cliffR << " ";
    std::cout << "Emerg:" << int(sensor.emergency)
              << " Done:" << int(sensor.profileDone);
}

void Serial_Com::printDebugData(const MotionDebugPacket &debug) const
{
    if (!debug.valid)
        return;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Motion: SpdL:" << debug.spdL << " SpdR:" << debug.spdR
              << " Vel:" << debug.vel << " Omg:" << debug.omg
              << " Dist:" << debug.dist << " Ang:" << debug.ang
              << " dt:" << debug.loopDt << "µs";
}

void Serial_Com::printCommandEcho(const CommandEchoPacket &echo) const
{
    if (!echo.valid)
        return;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Echo: d=" << echo.distance << " a=" << echo.angle
              << " vmax=" << echo.maxVel << " wmax=" << echo.maxOmega
              << " vend=" << echo.lastVel << " wend=" << echo.lastOmega
              << " acc=" << echo.linAcc << " aacc=" << echo.angAcc;
}