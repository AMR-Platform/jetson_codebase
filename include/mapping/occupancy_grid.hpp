#pragma once
/********************************************************************
 * OccupancyGrid – 2-D log-odds map suitable for SLAM / navigation.
 *  - fixed-size, axis-aligned grid
 *  - Bresenham ray tracing update from 2-D LiDAR scans
 *  - thread-safe (internal mutex)
 *  - image export for quick visualisation
 *  - binary (de)serialisation helpers
 *  - NEW:  serialize() helper to publish the grid over UDP or ROS2.
 *******************************************************************/
#include <vector>
#include <mutex>
#include <cmath>
#include <opencv2/core.hpp>

#include "lidar/lidar_handler.hpp"       // LidarPoint
#include "localization/robot_utils.hpp"  // RobotPose

/* ───── Log-odds parameters (tweak to taste) ─────────────────────── */
constexpr float LOG_ODD_OCCUPIED =  3.5f;   // + → more certain occupied
constexpr float LOG_ODD_FREE     = -3.5f;   // − → more certain free
constexpr float LOG_ODD_MAX      = 10.0f;
constexpr float LOG_ODD_MIN      = -10.0f;
/* ─────────────────────────────────────────────────────────────────── */

class OccupancyGrid
{
public:
    /** width_m × height_m [metres] at given resolution [metres/cell].
        origin_{x,y}_m is the world-space position of cell (0,0) centre.             */
    OccupancyGrid(float width_m,
                  float height_m,
                  float resolution_m,
                  float origin_x_m = 0.f,
                  float origin_y_m = 0.f);

    /* ───── Updates & queries ─────────────────────────────────────── */
    void  updateGrid(const std::vector<LidarPoint>& scan,
                     const RobotPose& robot_pose);

    /** 0 → free, 1 → occupied, 0.5 → unknown.                        */
    float getOccupancy(float x_m, float y_m) const;

    /** BGR image: white = free, black = occupied, grey = unknown.     */
    cv::Mat toImage() const;

    /* ───── Persistence ───────────────────────────────────────────── */
    void saveToFile(const std::string& filename) const;
    void loadFromFile(const std::string& filename);
    void clear();   ///< reset to “all unknown”

    /* ───── New: helpers for map publishers ───────────────────────── */
    /** Packs the grid (row-major) into out as:
        - each byte 0–100 → occupancy probability [%]
        -     value 255   → unknown                                      */
    bool serialize(std::vector<uint8_t>& out) const;

    /* Lightweight metadata accessors (handy for headers) */
    inline int   widthCells()  const { return width_cells_;  }
    inline int   heightCells() const { return height_cells_; }
    inline float resolution()  const { return resolution_;   }
    inline float originX()     const { return origin_x_;     }
    inline float originY()     const { return origin_y_;     }

private:
    /* helpers ------------------------------------------------------ */
    bool  worldToGrid(float x_m, float y_m,
                      int& grid_x, int& grid_y) const;
    void  gridToWorld(int grid_x, int grid_y,
                      float& x_m, float& y_m) const;
    float logOddsToProb(float log_odds) const;

    void  raycast(const RobotPose& start,
                  float end_x, float end_y,
                  std::vector<std::pair<int,int>>& cells) const;

    /* geometry ----------------------------------------------------- */
    const float width_m_, height_m_, resolution_;
    const float origin_x_, origin_y_;
    int   width_cells_, height_cells_;

    /* storage ------------------------------------------------------ */
    mutable std::mutex grid_mutex_;
    std::vector<std::vector<float>> log_odds_;   // [row][col]
};
