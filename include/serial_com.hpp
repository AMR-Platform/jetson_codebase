#ifndef SERIAL_COM_HPP
#define SERIAL_COM_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <termios.h>
#include <vector>
#include "config.hpp"

/* ——— inbound packets ——— */
struct SensorPacket
{
    float yaw{}, roll{}, pitch{};
    long encL{}, encR{};
    uint16_t vbat1{}, vbat2{}, cliffL{}, cliffC{}, cliffR{};
    uint8_t emergency{}, profileDone{};
    
    // Simple helpers filled by Serial_Com
    float dYaw{}, dEncL{}, dEncR{}, linVel{}, angVel{};
    
    // Validity flag
    bool valid{false};
};

struct MotionDebugPacket
{
    float spdL{}, spdR{}, vel{}, omg{}, dist{}, ang{}, loopDt{};
    bool valid{false};
};

struct CommandEchoPacket
{
    float distance{}, angle{};
    uint16_t maxVel{}, maxOmega{}, lastVel{}, lastOmega{};
    float linAcc{}, angAcc{};
    bool valid{false};
};

/* ——— outbound commands (host → MCU) ——— */
struct CommandPacket
{
    ControlMode mode{AUTONOMOUS};
    DebugMode dbg{DEBUG_OFF};
    
    // AUTONOMOUS fields
    float distance{}, angle{};
    uint16_t maxVel{}, maxOmega{}, lastVel{}, lastOmega{};
    float linAcc{}, angAcc{};
    
    // TELEOPERATOR fields (treated as bools 0/1)
    uint8_t f{}, b{}, l{}, r{};
};

class Serial_Com
{
public:
    explicit Serial_Com(const std::string &port, int baud = DEFAULT_BAUD_RATE);
    ~Serial_Com();
    
    // Non-blocking, call as often as you wish
    void spinOnce();
    
    // Host → MCU
    void sendCommand(const CommandPacket &cmd);
    
    // MCU → host : thread-safe getters
    SensorPacket getSensor() const;
    MotionDebugPacket getDebug() const;
    CommandEchoPacket getCommandEcho() const;
    
    // System state management
    void setSystemState(const SystemState &state);
    SystemState getSystemState() const;
    
    // Debug and testing functions
    void testCommunication();
    void printSensorData(const SensorPacket &sensor) const;
    void printDebugData(const MotionDebugPacket &debug) const;
    void printCommandEcho(const CommandEchoPacket &echo) const;
    
    // Connection status
    bool isConnected() const { return fd >= 0; }
    
    // Static utility functions
    static std::vector<std::string> getAvailablePorts();
    
private:
    int fd{-1};
    std::string rxBuf;
    mutable std::mutex mtx;
    
    // Data packets
    SensorPacket sensor{};
    MotionDebugPacket debug{};
    CommandEchoPacket cmdEcho{};
    
    // System state
    SystemState sysState{};
    
    // Parsing helpers
    bool parseCombinedLine(const std::string &line);
    bool parseTelemetryOnly(const std::string &line);
    bool parseDebugOnly(const std::string &line);
    bool parseCommandEcho(const std::string &line);
    
    // Utility functions
    static speed_t baudToTermios(int baud);
    void writeLine(const std::string &line);
    void updateSensorDeltas(SensorPacket &s);
    
    // Static variables for delta calculations
    static bool deltaInit;
    static float lastYaw;
    static long lastEncL, lastEncR;
};

#endif // SERIAL_COM_HPP