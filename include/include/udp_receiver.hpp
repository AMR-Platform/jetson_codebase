#ifndef UDP_RECEIVER_HPP
#define UDP_RECEIVER_HPP

#include <string>
#include <thread>
#include <atomic>

class UdpReceiver {
public:
    UdpReceiver(int port = 8888);
    ~UdpReceiver();

    void start();
    void stop();
    bool isRunning() const;

    std::string getLastMessage();

private:
    void receiveLoop();

    int sockfd;
    int port;
    std::thread receiver_thread;
    std::atomic<bool> running;
    std::string last_message;
};

#endif // UDP_RECEIVER_HPP
