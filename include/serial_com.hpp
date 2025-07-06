#ifndef SERIAL_COM_HPP
#define SERIAL_COM_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <termios.h>

/* ———  MCU-side enums brought to the host  ——— */
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

/* ———  inbound packets ——— */
struct SensorPacket
{
    float yaw{}, roll{}, pitch{};
    long encL{}, encR{};
    uint16_t vbat1{}, vbat2{}, cliffL{}, cliffC{}, cliffR{};
    uint8_t emergency{}, profileDone{};

    /* simple helpers filled by Serial_Com */
    float dYaw{}, dEncL{}, dEncR{}, linVel{}, angVel{};
};

struct MotionDebugPacket
{
    float spdL{}, spdR{}, vel{}, omg{}, dist{}, ang{}, loopDt{};
};

/* ———  outbound commands (host → MCU) ———  */
struct CommandPacket
{
    ControlMode mode{AUTONOMOUS};
    DebugMode dbg{DEBUG_OFF};

    /* AUTONOMOUS fields */
    float distance{}, angle{};
    uint16_t maxVel{}, maxOmega{}, lastVel{}, lastOmega{};
    float linAcc{}, angAcc{};

    /* TELEOPERATOR fields (treated as bools 0/1) */
    uint8_t f{}, b{}, l{}, r{};
};

class Serial_Com
{
public:
    explicit Serial_Com(const std::string &port, int baud = 115200);
    ~Serial_Com();

    /* non-blocking, call as often as you wish */
    void spinOnce();

    /* host → MCU */
    void sendCommand(const CommandPacket &cmd);

    /* MCU → host : thread-safe getters  */
    SensorPacket getSensor() const;
    MotionDebugPacket getDebug() const;
    DebugMode debugMode() const { return dbgMode.load(); }

private:
    int fd{-1};
    std::string rxBuf;
    mutable std::mutex mtx;

    SensorPacket sensor{};
    MotionDebugPacket dbg{};
    std::atomic<DebugMode> dbgMode{DEBUG_OFF};

    /* helpers */
    bool parseTelemetry(const std::string &line);
    bool parseDebug(const std::string &line);
    static speed_t baudToTermios(int baud);
    void writeLine(const std::string &line);
};

#endif // SERIAL_COM_HPP
