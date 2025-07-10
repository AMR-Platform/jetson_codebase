#pragma once
#include "SimpleSLAM.h"
#include <vector>
#include <string>

class OccupancyMap {
public:
    OccupancyMap(double width, double height, double resolution);
    ~OccupancyMap();
    
    // Main interface
    void addScan(const std::vector<LidarPoint>& scan, const Pose& pose);
    void clear();
    void saveToFile(const std::string& filename) const;
    void postProcessMap();  // MAKE SURE THIS LINE EXISTS
    
    // Rest of declarations...
    double getOccupancy(double x, double y) const;
    double getOccupancyGrid(int grid_x, int grid_y) const;
    int getWidth() const { return width_cells_; }
    int getHeight() const { return height_cells_; }
    double getResolution() const { return resolution_; }
    void getBounds(double& min_x, double& min_y, double& max_x, double& max_y) const;
    
    void worldToGrid(double x, double y, int& grid_x, int& grid_y) const;
    void gridToWorld(int grid_x, int grid_y, double& x, double& y) const;
    bool isValidGrid(int grid_x, int grid_y) const;
    
private:
    std::vector<std::vector<double>> grid_;
    std::vector<std::vector<int>> hits_;
    std::vector<std::vector<int>> misses_;
    
    int width_cells_, height_cells_;
    double resolution_;
    double origin_x_, origin_y_;
    
    void updateLine(int x0, int y0, int x1, int y1);
    void updateCell(int grid_x, int grid_y, bool occupied);
    double logOdds(int hits, int misses) const;
};