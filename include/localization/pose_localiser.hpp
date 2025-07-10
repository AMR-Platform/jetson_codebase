#pragma once

#include <opencv2/opencv.hpp>
#include "lidar/lidar_handler.hpp"      // For LidarPoint
#include "localization/SensorFusion.hpp" // For SensorFusion
#include "localization/RobotLocalization.hpp" // For RobotLocalization

/**
 * @brief Simple 2D pose structure
 */
struct Pose2D {
    double x, y, theta;
    
    Pose2D(double x_ = 0.0, double y_ = 0.0, double theta_ = 0.0) 
        : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief Performs 360° spin localization using LiDAR scan matching
 * 
 * @param occ Occupancy grid map (binary image: 0=free, 255=occupied)
 * @param originW World coordinates of map origin (top-left corner)
 * @return Pose2D Estimated robot pose (x, y in meters, theta in radians)
 */
Pose2D runSpinLocalisation(const cv::Mat1b& occ, const cv::Point2d& originW);
