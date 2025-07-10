#include "SimpleSLAM.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>


enum class LidarCoordinateSystem {
    STANDARD,    // -180° to +180°
    POSITIVE,    // 0° to 360°
    ROTATED_90,  // Rotated 90 degrees
    FLIPPED      // Mirror image
};


std::vector<std::vector<LidarPoint>> loadRealLidarScans(const std::string& filename, 
                                                       LidarCoordinateSystem coord_system = LidarCoordinateSystem::STANDARD) {
    std::vector<std::vector<LidarPoint>> scans;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open Range.txt file: " << filename << std::endl;
        return scans;
    }
    
    std::string line;
    int scan_count = 0;
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::vector<LidarPoint> scan;
        std::istringstream iss(line);
        std::string range_str;
        int beam_index = 0;
        std::vector<double> ranges;
        
        // First, collect all ranges
        while (iss >> range_str) {
            try {
                double range = std::stod(range_str);
                ranges.push_back(range);
            } catch (const std::exception& e) {
                ranges.push_back(0.0); // Invalid range
            }
        }
        
        // Now convert to LidarPoints with correct angles
        for (size_t i = 0; i < ranges.size(); i++) {
            double angle;
            
            switch (coord_system) {
                case LidarCoordinateSystem::STANDARD:
                    angle = -M_PI + (2.0 * M_PI * i / ranges.size());
                    break;
                case LidarCoordinateSystem::POSITIVE:
                    angle = (2.0 * M_PI * i / ranges.size());
                    if (angle > M_PI) angle -= 2 * M_PI;
                    break;
                case LidarCoordinateSystem::ROTATED_90:
                    angle = -M_PI + (2.0 * M_PI * i / ranges.size()) + M_PI/2;
                    break;
                case LidarCoordinateSystem::FLIPPED:
                    angle = M_PI - (2.0 * M_PI * i / ranges.size());
                    break;
            }
            
            scan.emplace_back(angle, ranges[i], 100);
        }
        
        if (!scan.empty()) {
            scans.push_back(scan);
            scan_count++;
        }
        
        if (scan_count % 100 == 0) {
            std::cout << "Loaded " << scan_count << " scans..." << std::endl;
        }
    }
    
    file.close();
    std::cout << "Successfully loaded " << scans.size() << " LiDAR scans" << std::endl;
    return scans;
}

std::vector<Pose> loadRealPoses(const std::string& filename, bool convert_degrees = true) {
    std::vector<Pose> poses;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open Pose.txt file: " << filename << std::endl;
        return poses;
    }
    
    std::string line;
    int pose_count = 0;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        double x, y, theta;
        
        if (iss >> x >> y >> theta) {
            // Convert degrees to radians if needed
            if (convert_degrees) {
                theta = theta * M_PI / 180.0;
            }
            
            poses.emplace_back(x, y, theta);
            pose_count++;
        }
        
        if (pose_count % 100 == 0) {
            std::cout << "Loaded " << pose_count << " poses..." << std::endl;
        }
    }
    
    file.close();
    std::cout << "Successfully loaded " << poses.size() << " poses" << std::endl;
    return poses;
}

void analyzePoseVariation(const std::vector<Pose>& poses) {
    if (poses.size() < 2) return;
    
    double min_x = poses[0].x, max_x = poses[0].x;
    double min_y = poses[0].y, max_y = poses[0].y;
    double min_theta = poses[0].theta, max_theta = poses[0].theta;
    
    for (const auto& pose : poses) {
        min_x = std::min(min_x, pose.x);
        max_x = std::max(max_x, pose.x);
        min_y = std::min(min_y, pose.y);
        max_y = std::max(max_y, pose.y);
        min_theta = std::min(min_theta, pose.theta);
        max_theta = std::max(max_theta, pose.theta);
    }
    
    std::cout << "\n=== Pose Variation Analysis ===" << std::endl;
    std::cout << "X range: " << std::fixed << std::setprecision(3) 
              << min_x << " to " << max_x << " (variation: " << (max_x - min_x) << "m)" << std::endl;
    std::cout << "Y range: " << min_y << " to " << max_y << " (variation: " << (max_y - min_y) << "m)" << std::endl;
    std::cout << "Theta range: " << min_theta*180/M_PI << "° to " << max_theta*180/M_PI 
              << "° (variation: " << (max_theta - min_theta)*180/M_PI << "°)" << std::endl;
    
    if (std::abs(max_x - min_x) < 0.1 && std::abs(max_y - min_y) < 0.1) {
        std::cout << "WARNING: Robot movement is very small - might be stationary data!" << std::endl;
    }
}

void testRealDataWithOptions() {
    std::cout << "\n=== Testing Real LiDAR Data with Coordinate System Options ===" << std::endl;
    
    std::string data_directory = "/home/jeewantha/Desktop/SimpleSLAM/";
    std::string range_file = data_directory + "Range.txt";
    std::string pose_file = data_directory + "Pose.txt";
    
    // Try different coordinate systems
    std::vector<LidarCoordinateSystem> systems = {
        LidarCoordinateSystem::STANDARD,
        LidarCoordinateSystem::POSITIVE,
        LidarCoordinateSystem::ROTATED_90,
        LidarCoordinateSystem::FLIPPED
    };
    
    std::vector<std::string> system_names = {
        "Standard (-180° to +180°)",
        "Positive (0° to 360°)",
        "Rotated 90°",
        "Flipped"
    };
    
    for (size_t sys = 0; sys < systems.size(); sys++) {
        std::cout << "\n=== Trying coordinate system: " << system_names[sys] << " ===" << std::endl;
        
        auto real_scans = loadRealLidarScans(range_file, systems[sys]);
        auto real_poses = loadRealPoses(pose_file, true); // Convert degrees to radians
        
        if (real_scans.empty() || real_poses.empty()) {
            std::cout << "Failed to load data files!" << std::endl;
            continue;
        }
        
        analyzePoseVariation(real_poses);
        
        size_t min_size = std::min(real_scans.size(), real_poses.size());
        real_scans.resize(min_size);
        real_poses.resize(min_size);
        
        // Use smaller subset for testing
        size_t test_size = std::min(min_size, size_t(50));
        real_scans.resize(test_size);
        real_poses.resize(test_size);
        
        SimpleSLAM slam;
        
        // Calculate appropriate map size based on pose range
        double pose_range = 20.0; // Default
        if (!real_poses.empty()) {
            double min_x = real_poses[0].x, max_x = real_poses[0].x;
            double min_y = real_poses[0].y, max_y = real_poses[0].y;
            for (const auto& pose : real_poses) {
                min_x = std::min(min_x, pose.x);
                max_x = std::max(max_x, pose.x);
                min_y = std::min(min_y, pose.y);
                max_y = std::max(max_y, pose.y);
            }
            pose_range = std::max(max_x - min_x, max_y - min_y) + 20.0; // Add 20m buffer
        }
        
        if (!slam.initialize(pose_range, 0.05)) {
            std::cout << "Failed to initialize SLAM!" << std::endl;
            continue;
        }
        
        std::cout << "Processing " << real_scans.size() << " scans..." << std::endl;
        
        for (size_t i = 0; i < real_scans.size(); i++) {
            slam.addScan(real_scans[i], real_poses[i]);
            
            if ((i + 1) % 10 == 0) {
                std::cout << "Progress: " << (i + 1) << "/" << real_scans.size() << std::endl;
            }
        }
        
        // Save map with system identifier
        std::string map_filename = "map_system_" + std::to_string(sys) + ".pgm";
        slam.saveMap(map_filename);
        
        std::cout << "Saved map as: " << map_filename << std::endl;
        slam.printStats();
    }
    
    std::cout << "\n=== Compare the generated maps to see which coordinate system works best ===" << std::endl;
}

int main() {
    std::cout << "=== SimpleSLAM Coordinate System Diagnosis ===" << std::endl;
    
    try {
        testRealDataWithOptions();
        
        std::cout << "\nCheck the generated maps:" << std::endl;
        std::cout << "- map_system_0.pgm (Standard -180° to +180°)" << std::endl;
        std::cout << "- map_system_1.pgm (Positive 0° to 360°)" << std::endl;
        std::cout << "- map_system_2.pgm (Rotated 90°)" << std::endl;
        std::cout << "- map_system_3.pgm (Flipped)" << std::endl;
        std::cout << "\nThe correct map should show your actual environment structure!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}