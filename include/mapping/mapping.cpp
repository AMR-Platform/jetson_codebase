#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <cmath>

struct LidarPoint {
    float azimuth;    // degrees
    float distance;   // meters
    uint8_t rssi;
};

struct RobotPose {
    float x, y;       // meters
    float theta;      // radians
};

class OccupancyGrid {
public:
    OccupancyGrid(float width_m, float height_m, float resolution_m, 
                  float origin_x_m = 0.0f, float origin_y_m = 0.0f);
    
    // Update grid with LiDAR scan at given robot pose
    void updateGrid(const std::vector<LidarPoint>& scan, const RobotPose& robot_pose);
    
    // Get occupancy probability at world coordinates
    float getOccupancy(float x_m, float y_m) const;
    
    // Convert to OpenCV image for visualization
    cv::Mat toImage() const;
    
    // Save/load grid
    void saveToFile(const std::string& filename) const;
    void loadFromFile(const std::string& filename);
    
    // Clear the grid
    void clear();
    
    // Get grid dimensions
    int getWidth() const { return width_cells_; }
    int getHeight() const { return height_cells_; }
    float getResolution() const { return resolution_; }
    
private:
    // Grid parameters
    float width_m_, height_m_;          // physical dimensions
    float resolution_;                  // meters per cell
    float origin_x_, origin_y_;         // origin offset
    int width_cells_, height_cells_;    // grid dimensions in cells
    
    // Occupancy data: log-odds representation
    std::vector<std::vector<float>> log_odds_;
    mutable std::mutex grid_mutex_;
    
    // Log-odds parameters
    static constexpr float LOG_ODD_OCCUPIED = 0.85f;    // log(p/(1-p)) for occupied
    static constexpr float LOG_ODD_FREE = -0.4f;        // log(p/(1-p)) for free
    static constexpr float LOG_ODD_MAX = 10.0f;         // clamp maximum
    static constexpr float LOG_ODD_MIN = -10.0f;        // clamp minimum
    
    // Helper functions
    bool worldToGrid(float x_m, float y_m, int& grid_x, int& grid_y) const;
    void gridToWorld(int grid_x, int grid_y, float& x_m, float& y_m) const;
    float logOddsToProb(float log_odds) const;
    void raycast(const RobotPose& start, float end_x, float end_y, 
                 std::vector<std::pair<int, int>>& cells) const;
};
