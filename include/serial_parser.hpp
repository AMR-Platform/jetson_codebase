#ifndef SERIAL_PARSER_HPP
#define SERIAL_PARSER_HPP

#include <string>

struct SensorPacket {
    float yaw;
    float roll;
    float pitch;
    long encoderLeft;
    long encoderRight;
    unsigned int bat1Voltage;
    unsigned int bat2Voltage;
    unsigned int cliffLeft;
    unsigned int cliffCenter;
    unsigned int cliffRight;
    unsigned int emergencyFlag;
};

class SerialParser {
public:
    SerialParser(const std::string& port_name, int baud_rate);
    ~SerialParser();

    bool readLine(std::string& out_line);
    bool parsePacket(const std::string& line, SensorPacket& packet);

private:
    int serial_fd;
};

#endif // SERIAL_PARSER_HPP
