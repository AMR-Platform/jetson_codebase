#include "occupancy_grid.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <cstdint>    // uint8_t

/* ───────────────────────── Constructor ──────────────────────────── */
OccupancyGrid::OccupancyGrid(float width_m, float height_m,
                             float resolution_m,
                             float origin_x_m, float origin_y_m)
    : width_m_(width_m),
      height_m_(height_m),
      resolution_(resolution_m),
      origin_x_(origin_x_m),
      origin_y_(origin_y_m)
{
    width_cells_  = static_cast<int>(std::ceil(width_m_  / resolution_));
    height_cells_ = static_cast<int>(std::ceil(height_m_ / resolution_));

    log_odds_.assign(height_cells_,
                     std::vector<float>(width_cells_, 0.0f));   // unknown

    std::cout << "[OccupancyGrid] "
              << width_cells_ << "×" << height_cells_
              << " cells  (res " << resolution_ << " m)\n";
}

/* ───────────────────────── Update from LiDAR ────────────────────── */
void OccupancyGrid::updateGrid(const std::vector<LidarPoint>& scan,
                               const RobotPose& robot_pose)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    for (const auto& p : scan)
    {
        if (p.distance <= 0.05f || p.distance > 20.0f) continue;

        float angle_rad = (p.azimuth * static_cast<float>(M_PI) / 180.0f)
                        + robot_pose.theta;

        float end_x = robot_pose.x + p.distance * std::cos(angle_rad);
        float end_y = robot_pose.y + p.distance * std::sin(angle_rad);

        std::vector<std::pair<int,int>> cells;
        raycast(robot_pose, end_x, end_y, cells);

        /* mark free along the ray (all but last) ------------------- */
        for (size_t i = 0; i + 1 < cells.size(); ++i) {
            int gx = cells[i].first, gy = cells[i].second;
            if (gx >= 0 && gx < width_cells_
             && gy >= 0 && gy < height_cells_)
            {
                float& lo = log_odds_[gy][gx];
                lo = std::clamp(lo + LOG_ODD_FREE,
                                LOG_ODD_MIN, LOG_ODD_MAX);
            }
        }

        /* mark endpoint occupied ---------------------------------- */
        if (!cells.empty()) {
            int gx = cells.back().first, gy = cells.back().second;
            if (gx >= 0 && gx < width_cells_
             && gy >= 0 && gy < height_cells_)
            {
                float& lo = log_odds_[gy][gx];
                lo = std::clamp(lo + LOG_ODD_OCCUPIED,
                                LOG_ODD_MIN, LOG_ODD_MAX);
            }
        }
    }
}

/* ───────────────────── Occupancy query (world) ──────────────────── */
float OccupancyGrid::getOccupancy(float x_m, float y_m) const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    int gx, gy;
    if (!worldToGrid(x_m, y_m, gx, gy))
        return 0.5f;                       // outside → unknown

    return logOddsToProb(log_odds_[gy][gx]);
}

/* ────────────────────────── Image export ────────────────────────── */
cv::Mat OccupancyGrid::toImage() const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    cv::Mat img(height_cells_, width_cells_, CV_8UC3);

    for (int y = 0; y < height_cells_; ++y)
        for (int x = 0; x < width_cells_; ++x)
        {
            float lo   = log_odds_[y][x];
            float prob = logOddsToProb(lo);

            cv::Vec3b col;                              // BGR
            if (std::abs(lo) < 0.1f)
                col = {128,128,128};                    // unknown
            else if (prob > 0.7f)
                col = {  0,  0,  0};                    // occupied
            else if (prob < 0.3f)
                col = {255,255,255};                    // free
            else {
                uint8_t g = static_cast<uint8_t>((1.0f - prob)*255.f);
                col = {g,g,g};                          // greyscale
            }
            img.at<cv::Vec3b>(y,x) = col;
        }
    cv::flip(img, img, 0);   // optional: (0,0) top-left → bottom-left
    return img;
}

/* ───────────────────────── Serialization ────────────────────────── */
bool OccupancyGrid::serialize(std::vector<uint8_t>& out) const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    try {
        out.resize(static_cast<size_t>(width_cells_ * height_cells_));
        uint8_t* dst = out.data();

        for (int y = 0; y < height_cells_; ++y)
            for (int x = 0; x < width_cells_; ++x)
            {
                float lo = log_odds_[y][x];
                if (std::abs(lo) < 0.1f)
                    *dst++ = 255;                       // unknown
                else
                    *dst++ =
                        static_cast<uint8_t>(std::round(
                            logOddsToProb(lo) * 100.f)); // 0-100 %
            }
        return true;
    }
    catch (...) { return false; }
}

/* ───────────────────────── File I/O (binary) ────────────────────── */
void OccupancyGrid::saveToFile(const std::string& filename) const
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    std::ofstream f(filename, std::ios::binary);
    if (!f) { std::cerr << "saveToFile: can't open " << filename << '\n'; return; }

    f.write(reinterpret_cast<const char*>(&width_cells_),  sizeof(width_cells_));
    f.write(reinterpret_cast<const char*>(&height_cells_), sizeof(height_cells_));
    f.write(reinterpret_cast<const char*>(&resolution_),   sizeof(resolution_));
    f.write(reinterpret_cast<const char*>(&origin_x_),     sizeof(origin_x_));
    f.write(reinterpret_cast<const char*>(&origin_y_),     sizeof(origin_y_));

    for (const auto& row : log_odds_)
        f.write(reinterpret_cast<const char*>(row.data()),
                row.size()*sizeof(float));
}

void OccupancyGrid::loadFromFile(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    std::ifstream f(filename, std::ios::binary);
    if (!f) { std::cerr << "loadFromFile: can't open " << filename << '\n'; return; }

    f.read(reinterpret_cast<char*>(&width_cells_),  sizeof(width_cells_));
    f.read(reinterpret_cast<char*>(&height_cells_), sizeof(height_cells_));
    f.read(reinterpret_cast<char*>(&resolution_),   sizeof(resolution_));
    f.read(reinterpret_cast<char*>(&origin_x_),     sizeof(origin_x_));
    f.read(reinterpret_cast<char*>(&origin_y_),     sizeof(origin_y_));

    log_odds_.assign(height_cells_,
                     std::vector<float>(width_cells_, 0.0f));

    for (auto& row : log_odds_)
        f.read(reinterpret_cast<char*>(row.data()),
               row.size()*sizeof(float));
}

/* ─────────────────────────── Utilities ──────────────────────────── */
void OccupancyGrid::clear()
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    for (auto& row : log_odds_)
        std::fill(row.begin(), row.end(), 0.f);
}

bool OccupancyGrid::worldToGrid(float x_m, float y_m,
                                int& gx, int& gy) const
{
    gx = static_cast<int>((x_m - origin_x_) / resolution_);
    gy = static_cast<int>((y_m - origin_y_) / resolution_);
    return (gx >= 0 && gx < width_cells_ &&
            gy >= 0 && gy < height_cells_);
}

void OccupancyGrid::gridToWorld(int gx, int gy,
                                float& x_m, float& y_m) const
{
    x_m = origin_x_ + (gx + 0.5f)*resolution_;
    y_m = origin_y_ + (gy + 0.5f)*resolution_;
}

float OccupancyGrid::logOddsToProb(float lo) const
{
    return 1.f / (1.f + std::exp(-lo));   // σ(lo)
}

/* Bresenham ray trace --------------------------------------------- */
void OccupancyGrid::raycast(const RobotPose& start,
                            float end_x, float end_y,
                            std::vector<std::pair<int,int>>& cells) const
{
    cells.clear();

    int x0, y0, x1, y1;
    if (!worldToGrid(start.x, start.y, x0, y0)
     || !worldToGrid(end_x,   end_y,   x1, y1))
        return;

    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        cells.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}
