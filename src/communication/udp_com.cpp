// udp_com.cpp
#include "communication/udp_com.hpp"
#include "lidar/lidar_handler.hpp"  // Include this for LidarPoint definition
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

// These globals are defined in main.cpp
extern SensorPacket g_sensor;
extern MotionDebugPacket g_debug;
extern CommandPacket g_cmd;
extern std::mutex g_cmd_mtx;

UDPCom::UDPCom(uint16_t localPort,
               const std::string &remoteIP,
               uint16_t remotePort)
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
        throw std::runtime_error("UDP socket creation failed");

    std::memset(&localAddr_, 0, sizeof(localAddr_));
    localAddr_.sin_family = AF_INET;
    localAddr_.sin_addr.s_addr = INADDR_ANY;
    localAddr_.sin_port = htons(localPort);
    if (bind(sockfd_, (sockaddr *)&localAddr_, sizeof(localAddr_)) < 0)
        perror("UDP bind");

    std::memset(&remoteAddr_, 0, sizeof(remoteAddr_));
    remoteAddr_.sin_family = AF_INET;
    remoteAddr_.sin_port = htons(remotePort);
    inet_pton(AF_INET, remoteIP.c_str(), &remoteAddr_.sin_addr);
}

UDPCom::~UDPCom()
{
    stop();
    close(sockfd_);
}

void UDPCom::start()
{
    running_ = true;
    recvThread_ = std::thread(&UDPCom::recvLoop, this);
}

void UDPCom::stop()
{
    running_ = false;
    if (recvThread_.joinable())
        recvThread_.join();
}

void UDPCom::recvLoop()
{
    char buf[2048];
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);
    while (running_)
    {
        int n = recvfrom(sockfd_, buf, sizeof(buf) - 1, 0,
                         (sockaddr *)&src, &srclen);
        if (n > 0)
        {
            buf[n] = '\0'; // Null terminate
            std::string msg(buf, n);
            handleIncoming(msg);
        }
    }
}

void UDPCom::handleIncoming(const std::string &msg)
{
    // Parse mode/debug changes and full CommandPacket
    if (msg.rfind("CMD,", 0) == 0)
    {
        // Format: CMD,<mode>,<dbg>,<distance>,<angle>,<maxVel>,<maxOmega>,
        //              <lastVel>,<lastOmega>,<linAcc>,<angAcc>
        std::istringstream ss(msg.substr(4));
        CommandPacket c{};
        char comma;
        int m, d;
        if (ss >> m >> comma >> d >> comma >> 
               c.distance >> comma >> c.angle >> comma >> 
               c.maxVel >> comma >> c.maxOmega >> comma >> 
               c.lastVel >> comma >> c.lastOmega >> comma >> 
               c.linAcc >> comma >> c.angAcc)
        {
            std::lock_guard<std::mutex> lk(g_cmd_mtx);
            if (g_cmd.cmdStatus != CMD_TOBE_WRITTEN)
            {
                c.mode = static_cast<ControlMode>(m);
                c.dbg = static_cast<DebugMode>(d);
                c.cmdStatus = CMD_TOBE_WRITTEN;
                g_cmd = c; // update global command
                std::cout << "[UDP] CMD received: mode=" << m << " dbg=" << d << "\n";
            }
        }
        return;
    }
    else if (msg.rfind("TELEOP,", 0) == 0)
    {
        std::istringstream ss(msg.substr(7));
        int f, b, l, r;
        char comma;
        if (ss >> f >> comma >> b >> comma >> l >> comma >> r)
        {
            std::lock_guard<std::mutex> lk(g_cmd_mtx);
            // fill the global command packet
            g_cmd.mode = TELEOPERATOR;
            g_cmd.dbg = RX_ECHO; // or whatever debug mode you prefer
            g_cmd.f = static_cast<uint8_t>(f);
            g_cmd.b = static_cast<uint8_t>(b);
            g_cmd.l = static_cast<uint8_t>(l);
            g_cmd.r = static_cast<uint8_t>(r);
            g_cmd.cmdStatus = CMD_TOBE_WRITTEN;
            std::cout << "[UDP] TELEOP RC: f=" << f
                      << " b=" << b
                      << " l=" << l
                      << " r=" << r << "\n";
        }
        return;
    }
}

void UDPCom::sendCommandEcho(const CommandPacket &cmdEcho)
{
    std::ostringstream ss;
    ss << "ECHO,"
       << cmdEcho.distance << "," << cmdEcho.angle << ","
       << cmdEcho.maxVel << "," << cmdEcho.maxOmega << ","
       << cmdEcho.lastVel << "," << cmdEcho.lastOmega << ","
       << cmdEcho.linAcc << "," << cmdEcho.angAcc;
    auto str = ss.str();
    sendto(sockfd_, str.data(), str.size(), 0,
           (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
}

void UDPCom::sendSensor(const SensorPacket &s)
{
    std::ostringstream ss;
    ss << "SENSOR,"
       << s.yaw << "," << s.roll << "," << s.pitch << ","
       << s.gyroX << "," << s.gyroY << "," << s.gyroZ << ","
       << s.accelX << "," << s.accelY << "," << s.accelZ << ","
       << s.encL << "," << s.encR << ","
       << s.vbat1 << "," << s.vbat2 << ","
       << s.cliffL << "," << s.cliffC << "," << s.cliffR << ","
       << int(s.emergency) << "," << int(s.profileDone);
    auto str = ss.str();
    sendto(sockfd_, str.data(), str.size(), 0,
           (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
}

void UDPCom::sendDebug(const MotionDebugPacket &d)
{
    std::ostringstream ss;
    ss << "DEBUG,"
       << d.spdL << "," << d.spdR << ","
       << d.vel << "," << d.omg << ","
       << d.dist << "," << d.ang << ","
       << d.loopDt;
    auto str = ss.str();
    sendto(sockfd_, str.data(), str.size(), 0,
           (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
}

void UDPCom::sendLidarScan(const std::vector<LidarPoint> &scan)
{
    if (scan.empty()) return;
    
    // Send LiDAR data in chunks due to UDP size limits
    const size_t MAX_POINTS_PER_PACKET = 100; // Adjust based on your needs
    size_t totalPoints = scan.size();
    size_t chunkID = 0;
    size_t totalChunks = (totalPoints + MAX_POINTS_PER_PACKET - 1) / MAX_POINTS_PER_PACKET;
    
    for (size_t offset = 0; offset < totalPoints; offset += MAX_POINTS_PER_PACKET)
    {
        size_t pointsInThisChunk = std::min(MAX_POINTS_PER_PACKET, totalPoints - offset);
        
        std::ostringstream ss;
        ss << "LIDAR," << chunkID << "," << totalChunks << "," << pointsInThisChunk;
        
        // Add each point: azimuth,distance,rssi
        for (size_t i = 0; i < pointsInThisChunk; ++i)
        {
            const auto &point = scan[offset + i];
            ss << "," << point.azimuth << "," << point.distance << "," << static_cast<int>(point.rssi);
        }
        
        auto str = ss.str();
        sendto(sockfd_, str.data(), str.size(), 0,
               (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
        
        ++chunkID;
        
        // Small delay between chunks to avoid overwhelming the receiver
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void UDPCom::sendVelocityProfile(const MotionDebugPacket &debug, const CommandPacket &cmd)
{
    // Send a combined packet with both actual and expected velocity data
    // This matches what the Python interface expects for velocity profile plotting
    std::ostringstream ss;
    ss << "VELOCITY_PROFILE,"
       << debug.vel << "," << debug.omg << ","           // actual velocity, omega
       << cmd.maxVel << "," << cmd.maxOmega << ","       // expected/command velocity, omega
       << debug.spdL << "," << debug.spdR << ","         // individual wheel speeds
       << cmd.distance << "," << cmd.angle << ","        // target distance, angle
       << debug.dist << "," << debug.ang << ","          // current distance, angle
       << debug.loopDt;                                  // loop timing
    
    auto str = ss.str();
    sendto(sockfd_, str.data(), str.size(), 0,
           (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
}

void UDPCom::sendMap(const std::vector<uint8_t> &mapData)
{
    size_t total = mapData.size(), offset = 0, chunkID = 0;
    size_t totalChunks = (total + MAX_UDP_PAYLOAD - 1) / MAX_UDP_PAYLOAD;
    while (offset < total)
    {
        size_t chunkSize = std::min(MAX_UDP_PAYLOAD, total - offset);
        std::ostringstream hdr;
        hdr << "MAP," << chunkID << "," << totalChunks << ",";
        auto h = hdr.str();
        sendto(sockfd_, h.data(), h.size(), 0,
               (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
        sendto(sockfd_,
               reinterpret_cast<const char *>(mapData.data() + offset),
               chunkSize, 0,
               (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
        offset += chunkSize;
        ++chunkID;
    }
}

void UDPCom::sendPointCloud(const std::vector<uint8_t> &pcData)
{
    size_t total = pcData.size(), offset = 0, chunkID = 0;
    size_t totalChunks = (total + MAX_UDP_PAYLOAD - 1) / MAX_UDP_PAYLOAD;
    while (offset < total)
    {
        size_t chunkSize = std::min(MAX_UDP_PAYLOAD, total - offset);
        std::ostringstream hdr;
        hdr << "PC," << chunkID << "," << totalChunks << ",";
        auto h = hdr.str();
        sendto(sockfd_, h.data(), h.size(), 0,
               (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
        sendto(sockfd_,
               reinterpret_cast<const char *>(pcData.data() + offset),
               chunkSize, 0,
               (sockaddr *)&remoteAddr_, sizeof(remoteAddr_));
        offset += chunkSize;
        ++chunkID;
    }
}