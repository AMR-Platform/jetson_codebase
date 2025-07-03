#include "lidar_handler.hpp"
#include "utils.hpp"
#include <iostream>

int main() {
    LidarHandler lidar("192.168.198.2", 2368);  // IP and port of LakiBeam 1S

    if (!lidar.initialize()) {
        std::cerr << "[ERROR] Failed to initialize LidarHandler.\n";
        return -1;
    }

    std::cout << "[INFO] Listening for LiDAR data...\n";

    while (true) {
        std::vector<LidarPoint> points;
        if (lidar.receivePacket(points)) {
            std::cout << utils::timestamp() << " - Received " << points.size() << " points:\n";
            for (size_t i = 0; i < std::min(points.size(), size_t(5)); ++i) {
                std::cout << "  (" << points[i].x << ", " << points[i].y << ", " << points[i].z
                          << ") Intensity: " << static_cast<int>(points[i].intensity) << "\n";
            }
        } else {
            std::cerr << "[WARN] No packet received or parse failed.\n";
        }

        utils::sleep_ms(100); // Adjust based on LiDAR frequency
    }

    return 0;
}
