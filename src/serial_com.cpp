#include "serial_com.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <cstring>
#include <cmath>

SensorPacket GlobalSensorData;

Serial_Com::Serial_Com(const std::string& port_name, int baud_rate) {
    serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Failed to open serial port");
        exit(1);
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        exit(1);
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;    // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;    // No remapping, no delays
    tty.c_cc[VMIN]  = 1;  // Read at least 1 character
    tty.c_cc[VTIME] = 1;  // 0.1 seconds read timeout

    tty.c_cflag |= CREAD | CLOCAL;     // Turn on READ & ignore ctrl lines
    tty.c_cflag &= ~(PARENB | PARODD); // No parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        exit(1);
    }
}

Serial_Com::~Serial_Com() {
    close(serial_fd);
}

void Serial_Com::update() {
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

void Serial_Com::parseAndUpdate(const std::string& line) {
    SensorPacket& s = GlobalSensorData;
    std::istringstream iss(line);

    if (!(iss >> s.yaw >> s.roll >> s.pitch >> s.encoderLeft >> s.encoderRight
              >> s.bat1Voltage >> s.bat2Voltage
              >> s.cliffLeft >> s.cliffCenter >> s.cliffRight >> s.emergencyFlag)) {
        std::cerr << "[Serial_Com] Malformed line: " << line << std::endl;
        return;
    }

    if (!initialized) {
        last_yaw = s.yaw;
        last_encL = s.encoderLeft;
        last_encR = s.encoderRight;
        initialized = true;
        return;
    }

    // ΔYaw (wrapped for circular range)
    s.deltaYaw = s.yaw - last_yaw;
    if (s.deltaYaw > 180) s.deltaYaw -= 360;
    if (s.deltaYaw < -180) s.deltaYaw += 360;

    // ΔEncoder ticks
    s.deltaEncL = s.encoderLeft - last_encL;
    s.deltaEncR = s.encoderRight - last_encR;

    // Save previous for next cycle
    last_yaw = s.yaw;
    last_encL = s.encoderLeft;
    last_encR = s.encoderRight;

    // Basic velocity estimation (will scale later with wheel params)
    s.linearVelocity = (s.deltaEncR + s.deltaEncL) / 2.0f;
    s.angularVelocity = (s.deltaEncR - s.deltaEncL);
}
