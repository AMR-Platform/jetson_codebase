#include "serial_parser.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <cmath>
#include <iostream>

SensorPacket GlobalSensorData;

SerialParser::SerialParser(const std::string& port_name, int baud_rate) {
    serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    // [same as before: setup termios...]
}

SerialParser::~SerialParser() {
    close(serial_fd);
}

void SerialParser::update() {
    char c;
    lastLine.clear();
    while (read(serial_fd, &c, 1) == 1) {
        if (c == '\n') {
            parseAndUpdate(lastLine);
            return;
        }
        lastLine += c;
    }
}

void SerialParser::parseAndUpdate(const std::string& line) {
    SensorPacket& s = GlobalSensorData;
    std::istringstream iss(line);

    if (!(iss >> s.yaw >> s.roll >> s.pitch >> s.encoderLeft >> s.encoderRight
              >> s.bat1Voltage >> s.bat2Voltage
              >> s.cliffLeft >> s.cliffCenter >> s.cliffRight >> s.emergencyFlag)) {
        std::cerr << "Malformed line: " << line << std::endl;
        return;
    }

    if (!initialized) {
        last_yaw = s.yaw;
        last_encL = s.encoderLeft;
        last_encR = s.encoderRight;
        initialized = true;
        return;
    }

    // Compute deltas
    s.deltaYaw = s.yaw - last_yaw;
    s.deltaEncL = s.encoderLeft - last_encL;
    s.deltaEncR = s.encoderRight - last_encR;

    last_yaw = s.yaw;
    last_encL = s.encoderLeft;
    last_encR = s.encoderRight;

    // Example: compute velocities (you can scale appropriately)
    s.linearVelocity = ((s.deltaEncR + s.deltaEncL) / 2.0f);
    s.angularVelocity = (s.deltaEncR - s.deltaEncL);

    // Optionally: accumulate position using odometry here too
}
