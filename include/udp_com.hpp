#ifndef UDP_COM_HPP
#define UDP_COM_HPP

#include <string>
#include <thread>
#include <atomic>

class UDP_Com {
public:
    UDP_Com(int port = 8888);
    ~UDP_Com();

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

#endif // UDP_COM_HPP
