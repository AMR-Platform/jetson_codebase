#ifndef SERIAL_PARSER_HPP
#define SERIAL_PARSER_HPP

#include <string>

struct SensorPacket {
    // Raw values from packet
    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    long encoderLeft = 0;
    long encoderRight = 0;
    unsigned int bat1Voltage = 0;
    unsigned int bat2Voltage = 0;
    unsigned int cliffLeft = 0;
    unsigned int cliffCenter = 0;
    unsigned int cliffRight = 0;
    unsigned int emergencyFlag = 0;

    // Derived values
    float deltaYaw = 0.0f;
    float deltaEncL = 0.0f;
    float deltaEncR = 0.0f;
    float linearVelocity = 0.0f;
    float angularVelocity = 0.0f;

    // Optional pose estimate (updated if needed)
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

// Global instance for access by other modules
extern SensorPacket GlobalSensorData;

class SerialParser {
public:
    SerialParser(const std::string& port_name, int baud_rate);
    ~SerialParser();

    void update(); // Reads a line and updates GlobalSensorData

private:
    int serial_fd;
    std::string lastLine;

    // For delta calculations
    float last_yaw = 0.0f;
    long last_encL = 0;
    long last_encR = 0;
    bool initialized = false;

    void parseAndUpdate(const std::string& line);
};

#endif // SERIAL_PARSER_HPP
