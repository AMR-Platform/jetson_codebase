#include "serial_parser.hpp"
#include <iostream>

int main() {
    SerialParser serial("/dev/ttyACM0", B115200);
    std::string line;
    SensorPacket packet;

    while (true) {
        if (serial.readLine(line)) {
            if (serial.parsePacket(line, packet)) {
                std::cout << "[YAW] " << packet.yaw
                          << " | [ENC L] " << packet.encoderLeft
                          << " | [ENC R] " << packet.encoderRight
                          << " | [EMERGENCY] " << packet.emergencyFlag
                          << std::endl;
            }
        }
    }
    return 0;
}
