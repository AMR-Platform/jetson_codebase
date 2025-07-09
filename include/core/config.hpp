#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>
#include <string>
#include <chrono>

// Timing configuration
constexpr int LOOP_TIME = 10; // Main loop time in milliseconds

// Serial communication configuration
constexpr int DEFAULT_BAUD_RATE = 9600;
constexpr int RX_BUFFER_SIZE = 512;

// UDP communication
constexpr uint16_t LOCAL_UDP_PORT = 9000;
constexpr uint16_t REMOTE_UDP_PORT = 9001;
const std::string REMOTE_IP = "192.168.1.68";

//LIDAR configuration
constexpr int canvas = 1000;
constexpr float rangeMax = 15.f;
constexpr auto period = std::chrono::milliseconds(LOOP_TIME);

// MCU-side enums brought to the host
enum ControlMode : uint8_t
{
    AUTONOMOUS = 0,
    TELEOPERATOR = 1
};

enum DebugMode : uint8_t
{
    DEBUG_OFF = 0,
    MOTION_DEBUG = 1,
    RX_ECHO = 2,
    MD_AND_ECHO = 3
};

enum CommandStatus : uint8_t
{
    CMD_TOBE_WRITTEN = 0,
    CMD_JUST_WROTE = 1,
    CMD_EMPTY = 2
};

// System state management
struct SystemState
{
    ControlMode controlMode{AUTONOMOUS};
    DebugMode debugMode{DEBUG_OFF};
    bool expectDebugData{false};
    bool expectCommandEcho{false};
    bool profileExecuted{true}; //Flip this flag to false immedieately upon received from UDP com or higher layer path planner class, make it true if it is given to mcu for execution
    bool profileOngoing{false}; //Flip this flag to true when the profile is being executed, make it false when the profile is finished as reported by the mcu's last received profile execution status bit
    bool emergencyStop{false}; // Set to true when emergency stop is triggered from MCU throguh the bit recieved from the telemetry data, set to false when the emergency stop is released from teleoperator  

    void updateExpectations()
    {
        expectDebugData = (debugMode == MOTION_DEBUG || debugMode == MD_AND_ECHO);
        expectCommandEcho = (debugMode == RX_ECHO || debugMode == MD_AND_ECHO);
    }
};

#endif // CONFIG_HPP