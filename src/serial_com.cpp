#include "serial_com.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unistd.h>

/* ———————————  ctor / dtor  ——————————— */
Serial_Com::Serial_Com(const std::string &port, int baud)
{
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("open");
        throw std::runtime_error("serial open");
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        throw std::runtime_error("tcgetattr");
    }

    cfsetospeed(&tty, baudToTermios(baud));
    cfsetispeed(&tty, baudToTermios(baud));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-N-1
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    tty.c_cc[VMIN] = 1;  // read ≥1 byte
    tty.c_cc[VTIME] = 1; // 0.1 s timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        throw std::runtime_error("tcsetattr");
    }
}

Serial_Com::~Serial_Com()
{
    if (fd >= 0)
        ::close(fd);
}

/* ———————————  public API  ——————————— */
void Serial_Com::spinOnce()
{
    char ch;
    while (::read(fd, &ch, 1) == 1)
    {
        if (ch == '\n' || ch == '\r')
        {
            std::cerr << "[RAW] " << rxBuf << '\n';
            if (!rxBuf.empty())
            {
                /* decide which parser to call */
                if (!parseTelemetry(rxBuf))
                    parseDebug(rxBuf); // ignore failure
                rxBuf.clear();

                
            }
        }
        else
        {
            rxBuf += ch;
        }
    }
}

void Serial_Com::sendCommand(const CommandPacket &cmd)
{
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
    else
    { // TELEOPERATOR
        oss << int(cmd.mode) << ','
            << int(cmd.dbg) << ','
            << "0,0," // distance, angle placeholders
            << int(cmd.f) << ',' << int(cmd.b) << ','
            << int(cmd.l) << ',' << int(cmd.r) << ','
            << "0,0\r\n"; // acc placeholders
    }
    writeLine(oss.str());
    dbgMode.store(cmd.dbg); // let listeners know immediately
}

SensorPacket Serial_Com::getSensor() const
{
    std::scoped_lock lk(mtx);
    return sensor;
}

MotionDebugPacket Serial_Com::getDebug() const
{
    std::scoped_lock lk(mtx);
    return dbg;
}

/* ———————————  helpers  ——————————— */
bool Serial_Com::parseTelemetry(const std::string &line)
{
    /* matches send_telemetry() exactly */
    std::istringstream iss(line);
    SensorPacket s;
    
    // Parse basic telemetry first (matches current AVR format)
    if (!(iss >> s.yaw >> s.roll >> s.pitch >> s.encL >> s.encR >> s.vbat1 >> s.vbat2 >> s.cliffL >> s.cliffC >> s.cliffR >> s.emergency >> s.profileDone))
        return false;
    
    // Try to parse additional IMU data if available (optional - won't fail if not present)
    // Expected format extension: ... emergency profileDone accelX accelY gyroZ
    iss >> s.accelX >> s.accelY >> s.gyroZ;

    /* basic deltas / velocities */
    static bool init = false;
    static float lastYaw = 0.0f;
    static long lastL = 0, lastR = 0;

    if (init)
    {
        s.dYaw = s.yaw - lastYaw;
        if (s.dYaw > 180)
            s.dYaw -= 360;
        if (s.dYaw < -180)
            s.dYaw += 360;
        s.dEncL = float(s.encL - lastL);
        s.dEncR = float(s.encR - lastR);
        s.linVel = (s.dEncL + s.dEncR) * 0.5f;
        s.angVel = (s.dEncR - s.dEncL); // tick-difference (convert later)
    }
    init = true;
    lastYaw = s.yaw;
    lastL = s.encL;
    lastR = s.encR;

    /* store thread-safe */
    {
        std::scoped_lock lk(mtx);
        sensor = s;
    }
    return true;
}

bool Serial_Com::parseDebug(const std::string &line)
{
    /* expect line starting with "SpdL:" in MOTION_DEBUG / MD_AND_ECHO */
    if (line.rfind("SpdL:", 0) != 0)
        return false;

    MotionDebugPacket d{};
    if (sscanf(line.c_str(),
               "SpdL: %f SpdR: %f Vel: %f Omg: %f dist: %f ang: %f dt: %f",
               &d.spdL, &d.spdR, &d.vel, &d.omg, &d.dist, &d.ang, &d.loopDt) != 7)
        return false;

    {
        std::scoped_lock lk(mtx);
        dbg = d;
    }
    return true;
}

void Serial_Com::writeLine(const std::string &line)
{
    ::write(fd, line.data(), line.size());
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
    default:
        return B115200;
    }
}
