
#include "occupancy_grid.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>

OccupancyGrid::OccupancyGrid(float width_m, float height_m, float resolution_m, 
                             float origin_x_m, float origin_y_m)
    : width_m_(width_m), height_m_(height_m), resolution_(resolution_m),
      origin_x_(origin_x_m), origin_y_(origin_y_m)
{
    width_cells_ = static_cast<int>(std::ceil(width_m_ / resolution_));
    height_cells_ = static_cast<int>(std::ceil(height_m_ / resolution_));
    
    // Initialize grid with unknown (log-odds = 0)
    log_odds_.resize(height_cells_, std::vector<float>(width_cells_, 0.0f));
    
    std::cout << "Created occupancy grid: " << width_cells_ << "x" << height_cells_ 
              << " cells, resolution: " << resolution_ << "m" << std::endl;
}

void OccupancyGrid::updateGrid(const std::vector<LidarPoint>& scan, const RobotPose& robot_pose)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    for (const auto& point : scan) {
        if (point.distance <= 0.05f || point.distance > 20.0f) continue; // filter invalid points
        
        // Convert LiDAR point to world coordinates
        float angle_rad = (point.azimuth * M_PI / 180.0f) + robot_pose.theta;
        float end_x = robot_pose.x + point.distance * std::cos(angle_rad);
        float end_y = robot_pose.y + point.distance * std::sin(angle_rad);
        
        // Raycast from robot to obstacle
        std::vector<std::pair<int, int>> ray_cells;
        raycast(robot_pose, end_x, end_y, ray_cells);
        
        // Mark cells along ray as free (except the last one)
        for (size_t i = 0; i < ray_cells.size() - 1; ++i) {
            int gx = ray_cells[i].first;
            int gy = ray_cells[i].second;
            if (gx >= 0 && gx < width_cells_ && gy >= 0 && gy < height_cells_) {
                log_odds_[gy][gx] += LOG_ODD_FREE;
                log_odds_[gy][gx] = std::max(LOG_ODD_MIN, std::min(LOG_ODD_MAX, log_odds_[gy][gx]));
            }
        }
        
        // Mark endpoint as occupied
        if (!ray_cells.empty()) {
            int gx = ray_cells.back().first;
            int gy = ray_cells.back().second;
            if (gx >= 0 && gx < width_cells_ && gy >= 0 && gy < height_cells_) {
                log_odds_[gy][gx] += LOG_ODD_OCCUPIED;
                log_odds_[gy][gx] = std::max(LOG_ODD_MIN, std::min(LOG_ODD_MAX, log_odds_[gy][gx]));
            }
        }
    }
}

float OccupancyGrid::getOccupancy(float x_m, float y_m) const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    int gx, gy;
    if (!worldToGrid(x_m, y_m, gx, gy)) return 0.5f; // unknown
    
    return logOddsToProb(log_odds_[gy][gx]);
}

cv::Mat OccupancyGrid::toImage() const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    cv::Mat img(height_cells_, width_cells_, CV_8UC3);
    
    for (int y = 0; y < height_cells_; ++y) {
        for (int x = 0; x < width_cells_; ++x) {
            float prob = logOddsToProb(log_odds_[y][x]);
            uint8_t gray = static_cast<uint8_t>((1.0f - prob) * 255.0f);
            
            // Color coding: white = free, black = occupied, gray = unknown
            if (std::abs(log_odds_[y][x]) < 0.1f) {
                // Unknown (gray)
                img.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
            } else if (prob > 0.7f) {
                // Occupied (black)
                img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            } else if (prob < 0.3f) {
                // Free (white)
                img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            } else {
                // Uncertain (gray scale)
                img.at<cv::Vec3b>(y, x) = cv::Vec3b(gray, gray, gray);
            }
        }
    }
    
    return img;
}

void OccupancyGrid::saveToFile(const std::string& filename) const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }
    
    // Write header
    file.write(reinterpret_cast<const char*>(&width_cells_), sizeof(width_cells_));
    file.write(reinterpret_cast<const char*>(&height_cells_), sizeof(height_cells_));
    file.write(reinterpret_cast<const char*>(&resolution_), sizeof(resolution_));
    file.write(reinterpret_cast<const char*>(&origin_x_), sizeof(origin_x_));
    file.write(reinterpret_cast<const char*>(&origin_y_), sizeof(origin_y_));
    
    // Write grid data
    for (const auto& row : log_odds_) {
        file.write(reinterpret_cast<const char*>(row.data()), row.size() * sizeof(float));
    }
}

void OccupancyGrid::loadFromFile(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return;
    }
    
    // Read header
    file.read(reinterpret_cast<char*>(&width_cells_), sizeof(width_cells_));
    file.read(reinterpret_cast<char*>(&height_cells_), sizeof(height_cells_));
    file.read(reinterpret_cast<char*>(&resolution_), sizeof(resolution_));
    file.read(reinterpret_cast<char*>(&origin_x_), sizeof(origin_x_));
    file.read(reinterpret_cast<char*>(&origin_y_), sizeof(origin_y_));
    
    // Resize grid
    log_odds_.resize(height_cells_, std::vector<float>(width_cells_, 0.0f));
    
    // Read grid data
    for (auto& row : log_odds_) {
        file.read(reinterpret_cast<char*>(row.data()), row.size() * sizeof(float));
    }
}

void OccupancyGrid::clear()
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    for (auto& row : log_odds_) {
        std::fill(row.begin(), row.end(), 0.0f);
    }
}

bool OccupancyGrid::worldToGrid(float x_m, float y_m, int& grid_x, int& grid_y) const
{
    grid_x = static_cast<int>((x_m - origin_x_) / resolution_);
    grid_y = static_cast<int>((y_m - origin_y_) / resolution_);
    
    return (grid_x >= 0 && grid_x < width_cells_ && grid_y >= 0 && grid_y < height_cells_);
}

void OccupancyGrid::gridToWorld(int grid_x, int grid_y, float& x_m, float& y_m) const
{
    x_m = origin_x_ + (grid_x + 0.5f) * resolution_;
    y_m = origin_y_ + (grid_y + 0.5f) * resolution_;
}

float OccupancyGrid::logOddsToProb(float log_odds) const
{
    return 1.0f / (1.0f + std::exp(-log_odds));
}

void OccupancyGrid::raycast(const RobotPose& start, float end_x, float end_y, 
                            std::vector<std::pair<int, int>>& cells) const
{
    cells.clear();
    
    int start_gx, start_gy, end_gx, end_gy;
    if (!worldToGrid(start.x, start.y, start_gx, start_gy) ||
        !worldToGrid(end_x, end_y, end_gx, end_gy)) {
        return;
    }
    
    // Bresenham's line algorithm
    int dx = std::abs(end_gx - start_gx);
    int dy = std::abs(end_gy - start_gy);
    int sx = (start_gx < end_gx) ? 1 : -1;
    int sy = (start_gy < end_gy) ? 1 : -1;
    int err = dx - dy;
    
    int x = start_gx;
    int y = start_gy;
    
    while (true) {
        cells.emplace_back(x, y);
        
        if (x == end_gx && y == end_gy) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}
