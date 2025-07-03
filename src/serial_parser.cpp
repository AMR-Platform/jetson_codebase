#include "serial_parser.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <sstream>

SerialParser::SerialParser(const std::string& port_name, int baud_rate) {
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
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= ~(PARENB | PARODD);           // No parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        exit(1);
    }
}

SerialParser::~SerialParser() {
    close(serial_fd);
}

bool SerialParser::readLine(std::string& out_line) {
    char c;
    out_line.clear();
    while (read(serial_fd, &c, 1) == 1) {
        if (c == '\n') return true;
        out_line += c;
    }
    return false;
}

bool SerialParser::parsePacket(const std::string& line, SensorPacket& packet) {
    std::istringstream iss(line);
    return (iss >> packet.yaw
                >> packet.roll
                >> packet.pitch
                >> packet.encoderLeft
                >> packet.encoderRight
                >> packet.bat1Voltage
                >> packet.bat2Voltage
                >> packet.cliffLeft
                >> packet.cliffCenter
                >> packet.cliffRight
                >> packet.emergencyFlag);
}
