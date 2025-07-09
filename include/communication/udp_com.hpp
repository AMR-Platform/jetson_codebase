// udp_com.hpp
#ifndef UDP_COM_HPP
#define UDP_COM_HPP

#include "config.hpp"
#include "serial_com.hpp" // for CommandPacket, SensorPacket, MotionDebugPacket
#include <cstdint>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <netinet/in.h> // for sockaddr_in

class UDPCom
{
public:
    UDPCom(uint16_t localPort,
           const std::string &remoteIP,
           uint16_t remotePort);
    ~UDPCom();

    void start();
    void stop();

    void sendCommand(const CommandPacket &cmd);
    void sendSensor(const SensorPacket &s);
    void sendDebug(const MotionDebugPacket &d);
    void sendMap(const std::vector<uint8_t> &mapData);
    void sendPointCloud(const std::vector<uint8_t> &pcData);

private:
    void recvLoop();
    void handleIncoming(const std::string &msg);

    int sockfd_;
    sockaddr_in localAddr_, remoteAddr_;
    std::thread recvThread_;
    std::atomic<bool> running_{false};
};

#endif // UDP_COM_HPP
