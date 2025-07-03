// occupancy_grid.cpp - Simple 2D occupancy grid implementation

#include "occupancy_grid.hpp"
#include <cmath>
#include <algorithm>

OccupancyGrid::OccupancyGrid(int width, int height, float resolution)
    : width(width), height(height), resolution(resolution) {
    grid = std::vector<std::vector<int8_t>>(height, std::vector<int8_t>(width, 0));
}

void OccupancyGrid::markOccupied(float x, float y) {
    int mx, my;
    if (worldToMap(x, y, mx, my)) {
        updateCell(mx, my, 100); // Occupied
    }
}

void OccupancyGrid::markFree(float x, float y) {
    int mx, my;
    if (worldToMap(x, y, mx, my)) {
        updateCell(mx, my, 0); // Free
    }
}

void OccupancyGrid::updateCell(int mx, int my, int8_t value) {
    std::lock_guard<std::mutex> lock(gridMutex);
    if (mx >= 0 && mx < width && my >= 0 && my < height) {
        grid[my][mx] = value;
    }
}

bool OccupancyGrid::worldToMap(float x, float y, int& mx, int& my) {
    mx = static_cast<int>(std::round(x / resolution)) + width / 2;
    my = static_cast<int>(std::round(y / resolution)) + height / 2;
    return (mx >= 0 && mx < width && my >= 0 && my < height);
}

const std::vector<std::vector<int8_t>>& OccupancyGrid::getGrid() {
    return grid;
}
