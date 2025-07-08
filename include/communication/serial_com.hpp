// serial_com.hpp
#ifndef SERIAL_COM_HPP
#define SERIAL_COM_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <termios.h>
#include <vector>
#include "../core/config.hpp"

/* ——— inbound packets ——— */
struct SensorPacket
{
    // Orientation (degrees)
    float yaw{}, roll{}, pitch{};

    // Gyroscope readings (deg/s) on X, Y, Z axes
    float gyroX{}, gyroY{}, gyroZ{};

    // Accelerometer readings (m/s²) on X, Y, Z axes
    float accelX{}, accelY{}, accelZ{};

    // Encoder counts
    long encL{}, encR{};

    // Battery voltages (mV) and cliff sensor readings
    uint16_t vbat1{}, vbat2{}, cliffL{}, cliffC{}, cliffR{};

    // Flags
    uint8_t emergency{}, profileDone{};

    // Deltas & derived velocities
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
    CommandStatus cmdStatus{CMD_EMPTY};

    // AUTONOMOUS fields
    float distance{}, angle{};
    uint16_t maxVel{}, maxOmega{}, lastVel{}, lastOmega{};
    float linAcc{}, angAcc{};

    // TELEOPERATOR fields (0/1)
    uint8_t f{}, b{}, l{}, r{};
};

class Serial_Com
{
public:
    explicit Serial_Com(const std::string &port, int baud = DEFAULT_BAUD_RATE);
    ~Serial_Com();

    // Non-blocking: read, parse, update internal packets
    void spinOnce(CommandPacket &cmd);

    // Send a new command (marks cmdStatus)
    void sendCommand(CommandPacket &cmd);

    // Thread-safe getters for the latest data
    SensorPacket getSensor() const;
    MotionDebugPacket getDebug() const;
    CommandEchoPacket getCommandEcho() const;

    // System state (toggles which parsers run)
    void setSystemState(const SystemState &state);
    SystemState getSystemState() const;

    // Debug/test
    void testCommunication();
    void printSensorData(const SensorPacket &sensor) const;
    void printDebugData(const MotionDebugPacket &debug) const;
    void printCommandEcho(const CommandEchoPacket &echo) const;

    bool isConnected() const { return fd >= 0; }

    // Find available /dev/tty* ports
    static std::vector<std::string> getAvailablePorts();

private:
    int fd{-1};
    std::string rxBuf;
    mutable std::mutex mtx;

    SensorPacket sensor{};
    MotionDebugPacket debug{};
    CommandEchoPacket cmdEcho{};
    SystemState sysState{};

    // Parsers
    bool parseCombinedLine(const std::string &line);
    bool parseTelemetryOnly(const std::string &line);
    bool parseDebugOnly(const std::string &line);
    bool parseCommandEcho(const std::string &line);

    // Helpers
    static speed_t baudToTermios(int baud);
    void writeLine(const std::string &line);
    void updateSensorDeltas(SensorPacket &s);

    static bool deltaInit;
    static float lastYaw;
    static long lastEncL, lastEncR;
};

#endif // SERIAL_COM_HPP
