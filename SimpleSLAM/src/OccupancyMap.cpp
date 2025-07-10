#include "OccupancyMap.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <iomanip>

OccupancyMap::OccupancyMap(double width, double height, double resolution)
    : resolution_(resolution)
    , origin_x_(-width / 2.0)
    , origin_y_(-height / 2.0) {
    
    width_cells_ = static_cast<int>(std::ceil(width / resolution));
    height_cells_ = static_cast<int>(std::ceil(height / resolution));
    
    // Initialize grids
    grid_.resize(height_cells_, std::vector<double>(width_cells_, 0.5)); // Unknown = 0.5
    hits_.resize(height_cells_, std::vector<int>(width_cells_, 0));
    misses_.resize(height_cells_, std::vector<int>(width_cells_, 0));
    
    std::cout << "OccupancyMap created: " << width_cells_ << "x" << height_cells_ 
              << " cells, resolution: " << resolution_ << "m" << std::endl;
}

OccupancyMap::~OccupancyMap() {
}

void OccupancyMap::addScan(const std::vector<LidarPoint>& scan, const Pose& pose) {
    if (scan.empty()) return;
    
    // Convert robot pose to grid coordinates
    int robot_grid_x, robot_grid_y;
    worldToGrid(pose.x, pose.y, robot_grid_x, robot_grid_y);
    
    if (!isValidGrid(robot_grid_x, robot_grid_y)) {
        // Try to handle poses near map boundary gracefully
        std::cerr << "Warning: Robot pose near/outside map bounds: (" 
                  << pose.x << ", " << pose.y << ") -> grid (" 
                  << robot_grid_x << ", " << robot_grid_y << ")" << std::endl;
        
        // Clamp to map bounds
        robot_grid_x = std::max(0, std::min(width_cells_ - 1, robot_grid_x));
        robot_grid_y = std::max(0, std::min(height_cells_ - 1, robot_grid_y));
    }
    
    // Process each laser beam
    int valid_points = 0;
    for (const auto& point : scan) {
        if (point.distance <= 0.1 || point.distance > 30.0) {
            continue; // Skip invalid readings
        }
        
        // Calculate endpoint in world coordinates
        // Make sure angle transformation is correct for your LiDAR coordinate system
        double global_angle = pose.theta + point.angle;
        double end_x = pose.x + point.distance * std::cos(global_angle);
        double end_y = pose.y + point.distance * std::sin(global_angle);
        
        // Convert to grid coordinates
        int end_grid_x, end_grid_y;
        worldToGrid(end_x, end_y, end_grid_x, end_grid_y);
        
        // Skip points outside map bounds instead of clamping
        if (!isValidGrid(end_grid_x, end_grid_y)) {
            continue;
        }
        
        // Trace ray from robot to endpoint
        updateLine(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y);
        valid_points++;
    }
    
    // Debug output for first few scans
    static int debug_count = 0;
    if (debug_count < 3) {
        std::cout << "  Processed " << valid_points << "/" << scan.size() 
                  << " valid LiDAR points for scan " << debug_count << std::endl;
        debug_count++;
    }
}

void OccupancyMap::updateLine(int x0, int y0, int x1, int y1) {
    // Bresenham's line algorithm
    std::vector<std::pair<int, int>> line_points;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0, y = y0;
    
    while (true) {
        if (isValidGrid(x, y)) {
            line_points.push_back({x, y});
        }
        
        if (x == x1 && y == y1) break;
        
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
   
   // Update cells along the ray
   for (size_t i = 0; i < line_points.size(); i++) {
       int gx = line_points[i].first;
       int gy = line_points[i].second;
       
       if (i == line_points.size() - 1) {
           // Last point is occupied (obstacle)
           updateCell(gx, gy, true);
       } else {
           // Intermediate points are free space
           updateCell(gx, gy, false);
       }
   }
}

void OccupancyMap::updateCell(int grid_x, int grid_y, bool occupied) {
   if (!isValidGrid(grid_x, grid_y)) return;
   
   if (occupied) {
       hits_[grid_y][grid_x] += 3; // Stronger weight for obstacles
   } else {
       misses_[grid_y][grid_x]++;
   }
   
   int hits = hits_[grid_y][grid_x];
   int misses = misses_[grid_y][grid_x];
   int total = hits + misses;
   
   if (total >= 2) { // Need at least 2 observations
       double probability = static_cast<double>(hits) / total;
       
       // Apply sigmoid-like function for better contrast
       if (probability > 0.6) {
           probability = 0.8 + 0.15 * (probability - 0.6) / 0.4;
       } else if (probability < 0.4) {
           probability = 0.05 + 0.15 * probability / 0.4;
       } else {
           probability = 0.2 + 0.6 * (probability - 0.4) / 0.2;
       }
       
       grid_[grid_y][grid_x] = probability;
   }
}

void OccupancyMap::postProcessMap() {
   std::cout << "Post-processing map for noise reduction..." << std::endl;
   
   // Median filter to reduce noise
   std::vector<std::vector<double>> filtered_grid = grid_;
   
   int filter_count = 0;
   for (int y = 1; y < height_cells_ - 1; y++) {
       for (int x = 1; x < width_cells_ - 1; x++) {
           // Only filter cells that have been observed
           if (hits_[y][x] + misses_[y][x] > 0) {
               std::vector<double> neighbors;
               
               for (int dy = -1; dy <= 1; dy++) {
                   for (int dx = -1; dx <= 1; dx++) {
                       neighbors.push_back(grid_[y + dy][x + dx]);
                   }
               }
               
               std::sort(neighbors.begin(), neighbors.end());
               double old_value = filtered_grid[y][x];
               filtered_grid[y][x] = neighbors[4]; // Median of 9 values
               
               if (std::abs(old_value - filtered_grid[y][x]) > 0.1) {
                   filter_count++;
               }
           }
       }
   }
   
   grid_ = filtered_grid;
   std::cout << "  Filtered " << filter_count << " noisy cells" << std::endl;
}

void OccupancyMap::clear() {
   for (int y = 0; y < height_cells_; y++) {
       for (int x = 0; x < width_cells_; x++) {
           grid_[y][x] = 0.5;  // Unknown
           hits_[y][x] = 0;
           misses_[y][x] = 0;
       }
   }
}

double OccupancyMap::getOccupancy(double x, double y) const {
   int grid_x, grid_y;
   worldToGrid(x, y, grid_x, grid_y);
   return getOccupancyGrid(grid_x, grid_y);
}

double OccupancyMap::getOccupancyGrid(int grid_x, int grid_y) const {
   if (!isValidGrid(grid_x, grid_y)) {
       return 0.5; // Unknown for out-of-bounds
   }
   return grid_[grid_y][grid_x];
}

void OccupancyMap::worldToGrid(double x, double y, int& grid_x, int& grid_y) const {
   grid_x = static_cast<int>((x - origin_x_) / resolution_);
   grid_y = static_cast<int>((y - origin_y_) / resolution_);
}

void OccupancyMap::gridToWorld(int grid_x, int grid_y, double& x, double& y) const {
   x = origin_x_ + (grid_x + 0.5) * resolution_;
   y = origin_y_ + (grid_y + 0.5) * resolution_;
}

bool OccupancyMap::isValidGrid(int grid_x, int grid_y) const {
   return grid_x >= 0 && grid_x < width_cells_ && 
          grid_y >= 0 && grid_y < height_cells_;
}

void OccupancyMap::getBounds(double& min_x, double& min_y, double& max_x, double& max_y) const {
   min_x = origin_x_;
   min_y = origin_y_;
   max_x = origin_x_ + width_cells_ * resolution_;
   max_y = origin_y_ + height_cells_ * resolution_;
}

void OccupancyMap::saveToFile(const std::string& filename) const {
   std::ofstream file(filename);
   if (!file.is_open()) {
       std::cerr << "Failed to open file: " << filename << std::endl;
       return;
   }
   
   // Calculate statistics
   int occupied_cells = 0, free_cells = 0, unknown_cells = 0;
   for (int y = 0; y < height_cells_; y++) {
       for (int x = 0; x < width_cells_; x++) {
           double prob = grid_[y][x];
           if (prob > 0.6) occupied_cells++;
           else if (prob < 0.4) free_cells++;
           else unknown_cells++;
       }
   }
   
   // Write PGM header
   file << "P2" << std::endl;
   file << "# Created by SimpleSLAM - Advanced Occupancy Mapping" << std::endl;
   file << "# Map resolution: " << resolution_ << " m/cell" << std::endl;
   file << "# Map bounds: (" << origin_x_ << ", " << origin_y_ << ") to (" 
        << (origin_x_ + width_cells_ * resolution_) << ", " 
        << (origin_y_ + height_cells_ * resolution_) << ")" << std::endl;
   file << "# Statistics: " << occupied_cells << " occupied, " 
        << free_cells << " free, " << unknown_cells << " unknown cells" << std::endl;
   file << width_cells_ << " " << height_cells_ << std::endl;
   file << "255" << std::endl;
   
   // Write grid data (flip Y axis for standard image coordinates)
   for (int y = height_cells_ - 1; y >= 0; y--) {
       for (int x = 0; x < width_cells_; x++) {
           double prob = grid_[y][x];
           int total_obs = hits_[y][x] + misses_[y][x];
           int pixel_value;
           
           if (total_obs == 0) {
               // Unexplored - Medium gray
               pixel_value = 205;
           } else if (prob > 0.8) {
               // Strongly occupied - Black
               pixel_value = 0;
           } else if (prob > 0.6) {
               // Moderately occupied - Dark gray
               pixel_value = 64;
           } else if (prob < 0.2) {
               // Strongly free - White
               pixel_value = 255;
           } else if (prob < 0.4) {
               // Moderately free - Light gray
               pixel_value = 220;
           } else {
               // Uncertain - Medium gray
               pixel_value = 128;
           }
           
           file << pixel_value << " ";
       }
       file << std::endl;
   }
   
   file.close();
   std::cout << "Enhanced PGM map saved to: " << filename << std::endl;
   std::cout << "  Map contains: " << occupied_cells << " occupied, " 
             << free_cells << " free, " << unknown_cells << " unknown cells" << std::endl;
}

double OccupancyMap::logOdds(int hits, int misses) const {
   if (hits + misses == 0) return 0.0;
   
   double prob = static_cast<double>(hits) / (hits + misses);
   prob = std::max(0.01, std::min(0.99, prob)); // Clamp to avoid log(0)
   
   return std::log(prob / (1.0 - prob));
}