#include "lidar_handler.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    LidarHandler handler;

    while (true) {
        handler.poll();
        auto scan = handler.getLatestScan();

        if (!scan.empty()) {
            std::cout << "Received " << scan.size() << " points.\n";
            for (size_t i = 0; i < std::min(scan.size(), size_t(5)); ++i) {
                std::cout << "  Point " << i << ": Azimuth = " << scan[i].azimuth
                          << "°, Distance = " << scan[i].distance << " m\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
