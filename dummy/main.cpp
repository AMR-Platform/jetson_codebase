#include "lidar_handler.hpp"
#include "utils.hpp"
#include <iostream>
#include <thread>

int main() {
    // Initialize LiDAR with IP and port
    LidarHandler lidar("192.168.198.2", 2368);

    // Start background polling thread
    std::thread lidarThread([&]() {
        lidar.poll();
    });

    std::cout << "[INFO] LakiBeam 1S LiDAR started. Printing scan data...\n";

    while (true) {
        auto scan = lidar.getLatestScan();

        if (!scan.empty()) {
            std::cout << utils::timestamp() << " - " << scan.size() << " points:\n";
            for (const auto& pt : scan) {
                std::cout << "  Angle: " << pt.angle << "°, Distance: " << pt.distance << " m\n";
            }
        } else {
            std::cout << "[WARN] No scan data received yet...\n";
        }

        utils::sleep_ms(500); // Print every 0.5s
    }

    lidarThread.join(); // Optional, for completeness

    return 0;
}
