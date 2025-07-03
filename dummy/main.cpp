#include "lidar_handler.hpp"
#include "utils.hpp"
#include <iostream>
#include <thread>

int main() {
    // Create LidarHandler for LakiBeam 1S (Ethernet IP and Port)
    LidarHandler lidar("192.168.198.2", 2368);

    // Start background polling thread
    std::thread lidarThread([&]() {
        lidar.poll();
    });

    std::cout << "[INFO] LiDAR polling started. Reading scans...\n";

    // Main loop: print latest scan periodically
    while (true) {
        auto scan = lidar.getLatestScan();

        std::cout << utils::timestamp() << " - Scan with " << scan.size() << " points\n";

        for (size_t i = 0; i < std::min(scan.size(), size_t(5)); ++i) {
            const auto& pt = scan[i];
            std::cout << "  [" << i << "] Angle: " << pt.angle << "°, Distance: " << pt.distance << " m\n";
        }

        utils::sleep_ms(500);
    }

    lidarThread.join(); // Optional: wait for thread (unreachable in current loop)

    return 0;
}
