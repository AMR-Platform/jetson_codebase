#ifndef SERIAL_PARSER_HPP
#define SERIAL_PARSER_HPP

#include <string>
extern SensorPacket GlobalSensorData;
struct SensorPacket {
    // Raw
    float yaw, roll, pitch;
    long encoderLeft, encoderRight;
    unsigned int bat1Voltage, bat2Voltage;
    unsigned int cliffLeft, cliffCenter, cliffRight;
    unsigned int emergencyFlag;

    // Derived
    float deltaYaw = 0.0f;
    float deltaEncL = 0.0f;
    float deltaEncR = 0.0f;

    float linearVelocity = 0.0f;
    float angularVelocity = 0.0f;

    // Optional: pose from odometry (for visualization)
    float x = 0.0f, y = 0.0f, theta = 0.0f;
};


class SerialParser {
public:
    SerialParser(const std::string& port_name, int baud_rate);
    ~SerialParser();

    void update();
private:
    int serial_fd;
    std::string lastLine;

    // Previous values to compute deltas
    float last_yaw = 0.0f;
    long last_encL = 0, last_encR = 0;
    bool initialized = false;

    void parseAndUpdate(const std::string& line);
};

#endif // SERIAL_PARSER_HPP
