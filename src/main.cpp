#include "lidar_handler.hpp"
#include <iostream>
#include <thread>

int main() {
    LidarHandler handler;

    while (true) {
        handler.poll();
        auto scan = handler.getLatestScan();
        
        if (!scan.empty()) {
            std::cout << "Scan with " << scan.size() << " points received.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
