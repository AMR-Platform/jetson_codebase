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

// Static member initialization
bool Serial_Com::deltaInit = false;
float Serial_Com::lastYaw = 0.0f;
long Serial_Com::lastEncL = 0;
long Serial_Com::lastEncR = 0;

/* ———————————  ctor / dtor  ——————————— */
Serial_Com::Serial_Com(const std::string &port, int baud)
{
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("open");
        throw std::runtime_error("serial open failed for port: " + port);
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        ::close(fd);
        throw std::runtime_error("tcgetattr failed");
    }

    // Set baud rates
    cfsetospeed(&tty, baudToTermios(baud));
    cfsetispeed(&tty, baudToTermios(baud));

    // Configure 8N1
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;         // Clear data size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem lines

    // Input flags
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);           // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling

    // Output flags
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Local flags
    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    // VMIN and VTIME
    tty.c_cc[VMIN] = 0;  // Non-blocking read
    tty.c_cc[VTIME] = 1; // 0.1 second timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        ::close(fd);
        throw std::runtime_error("tcsetattr failed");
    }

    // Flush any existing data
    tcflush(fd, TCIOFLUSH);
    
    std::cout << "[SERIAL] Opened port " << port << " at " << baud << " baud" << std::endl;
}

Serial_Com::~Serial_Com()
{
    if (fd >= 0)
    {
        std::cout << "[SERIAL] Closing serial port" << std::endl;
        ::close(fd);
    }
}

/* ———————————  public API  ——————————— */
void Serial_Com::spinOnce()
{
    if (fd < 0) return;
    
    char buffer[256];
    int bytes_read = ::read(fd, buffer, sizeof(buffer) - 1);
    
    if (bytes_read > 0)
    {
        for (int i = 0; i < bytes_read; i++)
        {
            char ch = buffer[i];
            if (ch == '\n' || ch == '\r')
            {
                if (!rxBuf.empty())
                {
                    std::cout << "[RAW] " << rxBuf << std::endl;
                    
                    // Parse based on current system state
                    if (sysState.expectDebugData && sysState.expectCommandEcho)
                    {
                        // Could be debug+telemetry combined, command echo, or just telemetry
                        if (!parseCombinedLine(rxBuf) && !parseCommandEcho(rxBuf))
                        {
                            parseTelemetryOnly(rxBuf);
                        }
                    }
                    else if (sysState.expectDebugData)
                    {
                        // Could be debug+telemetry combined or just telemetry
                        if (!parseCombinedLine(rxBuf))
                        {
                            parseTelemetryOnly(rxBuf);
                        }
                    }
                    else if (sysState.expectCommandEcho)
                    {
                        // Could be command echo or just telemetry
                        if (!parseCommandEcho(rxBuf))
                        {
                            parseTelemetryOnly(rxBuf);
                        }
                    }
                    else
                    {
                        // Only expecting telemetry
                        parseTelemetryOnly(rxBuf);
                    }
                    
                    rxBuf.clear();
                }
            }
            else if (ch >= 32 && ch <= 126) // Only printable ASCII characters
            {
                rxBuf += ch;
            }
        }
    }
    else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        perror("read");
    }
}

void Serial_Com::sendCommand(const CommandPacket &cmd)
{
    if (fd < 0) return;
    
    std::ostringstream oss;
    if (cmd.mode == AUTONOMOUS)
    {
        oss << int(cmd.mode) << ','
            << int(cmd.dbg) << ','
            << cmd.distance << ','
            << cmd.angle << ','
            << cmd.maxVel << ','
            << cmd.maxOmega << ','
            << cmd.lastVel << ','
            << cmd.lastOmega << ','
            << cmd.linAcc << ','
            << cmd.angAcc << "\r\n";
    }
    else // TELEOPERATOR
    {
        oss << int(cmd.mode) << ','
            << int(cmd.dbg) << ','
            << "0,0," // distance, angle placeholders
            << int(cmd.f) << ',' << int(cmd.b) << ','
            << int(cmd.l) << ',' << int(cmd.r) << ','
            << "0,0\r\n"; // acc placeholders
    }
    
    std::string command = oss.str();
    std::cout << "[SEND] " << command;
    writeLine(command);
    
    // Update system state
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

void Serial_Com::setSystemState(const SystemState &state)
{
    std::scoped_lock lk(mtx);
    sysState = state;
    sysState.updateExpectations();
}

SystemState Serial_Com::getSystemState() const
{
    std::scoped_lock lk(mtx);
    return sysState;
}

/* ———————————  parsing functions  ——————————— */
bool Serial_Com::parseCombinedLine(const std::string &line)
{
    // Expected format: "SpdL: +0.0 SpdR: +0.0 Vel: +0.0 Omg: +0.0 dist: +0.0 ang: +0.0 dt: 10045.0 0.00 0.00 0.00 0 0 441 459 414 396 381 0 1"
    if (line.find("SpdL:") != 0)
        return false;

    MotionDebugPacket d{};
    float yaw, roll, pitch;
    long encL, encR;
    uint16_t vbat1, vbat2, cliffL, cliffC, cliffR;
    uint8_t emergency, profileDone;

    // Parse the combined line
    int parsed = sscanf(line.c_str(),
                       "SpdL: %f SpdR: %f Vel: %f Omg: %f dist: %f ang: %f dt: %f %f %f %f %ld %ld %hu %hu %hu %hu %hu %hhu %hhu",
                       &d.spdL, &d.spdR, &d.vel, &d.omg, &d.dist, &d.ang, &d.loopDt,
                       &yaw, &roll, &pitch, &encL, &encR,
                       &vbat1, &vbat2, &cliffL, &cliffC, &cliffR, &emergency, &profileDone);

    if (parsed == 19)
    {
        // Successfully parsed both debug and telemetry
        d.valid = true;
        
        SensorPacket s{};
        s.yaw = yaw;
        s.roll = roll;
        s.pitch = pitch;
        s.encL = encL;
        s.encR = encR;
        s.vbat1 = vbat1;
        s.vbat2 = vbat2;
        s.cliffL = cliffL;
        s.cliffC = cliffC;
        s.cliffR = cliffR;
        s.emergency = emergency;
        s.profileDone = profileDone;
        s.valid = true;
        
        updateSensorDeltas(s);
        
        {
            std::scoped_lock lk(mtx);
            debug = d;
            sensor = s;
        }
        
        std::cout << "[PARSED] Combined debug+telemetry data" << std::endl;
        return true;
    }
    
    return false;
}

bool Serial_Com::parseTelemetryOnly(const std::string &line)
{
    // Expected format: "0.00 0.00 0.00 0 0 441 459 414 396 381 0 1"
    std::istringstream iss(line);
    SensorPacket s{};
    
    if (!(iss >> s.yaw >> s.roll >> s.pitch >> s.encL >> s.encR 
          >> s.vbat1 >> s.vbat2 >> s.cliffL >> s.cliffC >> s.cliffR 
          >> s.emergency >> s.profileDone))
    {
        std::cout << "[PARSE] Failed to parse telemetry: " << line << std::endl;
        return false;
    }

    s.valid = true;
    updateSensorDeltas(s);
    
    {
        std::scoped_lock lk(mtx);
        sensor = s;
    }
    
    std::cout << "[PARSED] Telemetry only" << std::endl;
    return true;
}

bool Serial_Com::parseDebugOnly(const std::string &line)
{
    // Expected format: "SpdL: +0.0 SpdR: +0.0 Vel: +0.0 Omg: +0.0 dist: +0.0 ang: +0.0 dt: 10045.0"
    if (line.find("SpdL:") != 0)
        return false;

    MotionDebugPacket d{};
    if (sscanf(line.c_str(),
               "SpdL: %f SpdR: %f Vel: %f Omg: %f dist: %f ang: %f dt: %f",
               &d.spdL, &d.spdR, &d.vel, &d.omg, &d.dist, &d.ang, &d.loopDt) != 7)
    {
        std::cout << "[PARSE] Failed to parse debug: " << line << std::endl;
        return false;
    }

    d.valid = true;
    {
        std::scoped_lock lk(mtx);
        debug = d;
    }
    
    std::cout << "[PARSED] Debug only" << std::endl;
    return true;
}

bool Serial_Com::parseCommandEcho(const std::string &line)
{
    // Expected format: "CMD d=500.0 a=90.0 vmax=300 wmax=120 vend=0 wend=0 acc=500.0 aacc=360.0"
    if (line.find("CMD") != 0)
        return false;

    CommandEchoPacket e{};
    if (sscanf(line.c_str(),
               "CMD d=%f a=%f vmax=%hu wmax=%hu vend=%hu wend=%hu acc=%f aacc=%f",
               &e.distance, &e.angle, &e.maxVel, &e.maxOmega,
               &e.lastVel, &e.lastOmega, &e.linAcc, &e.angAcc) != 8)
    {
        std::cout << "[PARSE] Failed to parse command echo: " << line << std::endl;
        return false;
    }

    e.valid = true;
    {
        std::scoped_lock lk(mtx);
        cmdEcho = e;
    }
    
    std::cout << "[PARSED] Command echo" << std::endl;
    return true;
}

/* ———————————  utility functions  ——————————— */
void Serial_Com::updateSensorDeltas(SensorPacket &s)
{
    if (deltaInit)
    {
        s.dYaw = s.yaw - lastYaw;
        // Handle wraparound
        if (s.dYaw > 180.0f)
            s.dYaw -= 360.0f;
        if (s.dYaw < -180.0f)
            s.dYaw += 360.0f;
        
        s.dEncL = float(s.encL - lastEncL);
        s.dEncR = float(s.encR - lastEncR);
        s.linVel = (s.dEncL + s.dEncR) * 0.5f;
        s.angVel = (s.dEncR - s.dEncL); // tick-difference
    }
    else
    {
        s.dYaw = s.dEncL = s.dEncR = s.linVel = s.angVel = 0.0f;
        deltaInit = true;
    }
    
    lastYaw = s.yaw;
    lastEncL = s.encL;
    lastEncR = s.encR;
}

void Serial_Com::writeLine(const std::string &line)
{
    if (fd < 0) return;
    
    ssize_t bytes_written = ::write(fd, line.data(), line.size());
    if (bytes_written < 0)
    {
        perror("write");
    }
    else if (bytes_written != static_cast<ssize_t>(line.size()))
    {
        std::cout << "[WRITE] Warning: only wrote " << bytes_written 
                  << " of " << line.size() << " bytes" << std::endl;
    }
    
    // Force flush
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
        std::cout << "[WARN] Unsupported baud rate " << baud << ", using 9600" << std::endl;
        return B9600;
    }
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
    cmd.distance = 100.0f; // 100mm forward
    cmd.angle = 0.0f;      // No rotation
    cmd.maxVel = 200;      // 200 mm/s
    cmd.maxOmega = 0;
    cmd.lastVel = 0;
    cmd.lastOmega = 0;
    cmd.linAcc = 500.0f;   // 500 mm/s²
    cmd.angAcc = 0.0f;
    
    std::cout << "Sending movement command (100mm forward)..." << std::endl;
    sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Test 3: Send a rotation command
    cmd.distance = 0.0f;   // No linear movement
    cmd.angle = 90.0f;     // 90 degree turn
    cmd.maxVel = 0;
    cmd.maxOmega = 120;    // 120 deg/s
    cmd.lastVel = 0;
    cmd.lastOmega = 0;
    cmd.linAcc = 0.0f;
    cmd.angAcc = 360.0f;   // 360 deg/s²
    
    std::cout << "Sending rotation command (90 degrees)..." << std::endl;
    sendCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Test 4: Test teleoperator mode
    cmd.mode = TELEOPERATOR;
    cmd.dbg = RX_ECHO;
    cmd.f = 1; // Forward
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
    if (!sensor.valid) return;
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "IMU: Y:" << sensor.yaw << "° R:" << sensor.roll << "° P:" << sensor.pitch << "° ";
    std::cout << "Enc: L:" << sensor.encL << " R:" << sensor.encR << " ";
    std::cout << "Bat: " << sensor.vbat1 << "mV/" << sensor.vbat2 << "mV ";
    std::cout << "Cliff: L:" << sensor.cliffL << " C:" << sensor.cliffC << " R:" << sensor.cliffR << " ";
    std::cout << "Emerg:" << int(sensor.emergency) << " Done:" << int(sensor.profileDone);
}

void Serial_Com::printDebugData(const MotionDebugPacket &debug) const
{
    if (!debug.valid) return;
    
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Motion: SpdL:" << debug.spdL << " SpdR:" << debug.spdR << " ";
    std::cout << "Vel:" << debug.vel << " Omg:" << debug.omg << " ";
    std::cout << "Dist:" << debug.dist << " Ang:" << debug.ang << " ";
    std::cout << "dt:" << debug.loopDt << "µs";
}

void Serial_Com::printCommandEcho(const CommandEchoPacket &echo) const
{
    if (!echo.valid) return;
    
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Echo: d=" << echo.distance << " a=" << echo.angle << " ";
    std::cout << "vmax=" << echo.maxVel << " wmax=" << echo.maxOmega << " ";
    std::cout << "vend=" << echo.lastVel << " wend=" << echo.lastOmega << " ";
    std::cout << "acc=" << echo.linAcc << " aacc=" << echo.angAcc;
}

std::vector<std::string> Serial_Com::getAvailablePorts()
{
    std::vector<std::string> ports;
    
    // Common serial port patterns
    std::vector<std::string> patterns = {
        "/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyS"
    };
    
    for (const auto &pattern : patterns)
    {
        for (int i = 0; i < 10; i++)
        {
            std::string port = pattern + std::to_string(i);
            int test_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (test_fd >= 0)
            {
                ports.push_back(port);
                ::close(test_fd);
            }
        }
    }
    
    return ports;
}