#include "udp_receiver.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <netinet/in.h>

UdpReceiver::UdpReceiver(int port) : port(port), running(false) {}

UdpReceiver::~UdpReceiver() {
    stop();
}

void UdpReceiver::start() {
    running = true;
    receiver_thread = std::thread(&UdpReceiver::receiveLoop, this);
}

void UdpReceiver::stop() {
    running = false;
    if (receiver_thread.joinable()) receiver_thread.join();
}

bool UdpReceiver::isRunning() const {
    return running;
}

std::string UdpReceiver::getLastMessage() {
    return last_message;
}

void UdpReceiver::receiveLoop() {
    struct sockaddr_in server_addr{}, client_addr{};
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("UDP socket creation failed");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("UDP bind failed");
        close(sockfd);
        return;
    }

    std::cout << "[UDP Receiver] Listening on port " << port << "...\n";

    char buffer[1024];
    socklen_t len = sizeof(client_addr);

    while (running) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                             (struct sockaddr *)&client_addr, &len);
        if (n > 0) {
            buffer[n] = '\0';
            last_message = std::string(buffer);
            std::cout << "[UDP Receiver] Message: " << last_message << std::endl;

            // TODO: Add behavior trigger here
        }
    }

    close(sockfd);
}
