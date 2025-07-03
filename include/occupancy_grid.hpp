// occupancy_grid.hpp - Simple 2D occupancy grid map

#pragma once
#include <vector>
#include <mutex>
#include <cstdint>

class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, float resolution);
    void markOccupied(float x, float y);
    void markFree(float x, float y);
    const std::vector<std::vector<int8_t>>& getGrid();

private:
    int width, height;
    float resolution;
    std::vector<std::vector<int8_t>> grid;
    std::mutex gridMutex;

    void updateCell(int mx, int my, int8_t value);
    bool worldToMap(float x, float y, int& mx, int& my);
};
