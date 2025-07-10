#include "ScanMatcher.h"
#include "OccupancyMap.h"
#include <iostream>
#include <algorithm>
#include <cmath>

ScanMatcher::ScanMatcher() 
    : map_(nullptr)
    , max_iterations_(15)
    , convergence_threshold_(0.001)
    , search_range_xy_(0.5)
    , search_range_theta_(0.2) {
}

ScanMatcher::~ScanMatcher() {
}

Pose ScanMatcher::matchScanToMap(const std::vector<LidarPoint>& scan,
                                const Pose& initial_guess,
                                const OccupancyMap* map) {
    if (!map || scan.empty()) {
        return initial_guess;
    }
    
    map_ = map;
    
    try {
        return optimizePose(scan, initial_guess);
    } catch (const std::exception& e) {
        std::cerr << "Scan matching failed: " << e.what() << std::endl;
        return initial_guess;
    }
}

Pose ScanMatcher::optimizePose(const std::vector<LidarPoint>& scan, 
                              const Pose& initial_pose) {
    Pose best_pose = initial_pose;
    double best_score = calculateMatchScore(scan, initial_pose);
    
    // Hierarchical search: coarse then fine
    std::vector<double> xy_steps = {0.2, 0.1, 0.05};  // Multiple resolution levels
    std::vector<double> theta_steps = {0.1, 0.05, 0.02};
    
    for (size_t level = 0; level < xy_steps.size(); level++) {
        double step_xy = xy_steps[level];
        double step_theta = theta_steps[level];
        double current_range_xy = search_range_xy_ / (level + 1);
        double current_range_theta = search_range_theta_ / (level + 1);
        
        Pose level_best = best_pose;
        double level_best_score = best_score;
        
        // Grid search around current best pose
        for (double dx = -current_range_xy; dx <= current_range_xy; dx += step_xy) {
            for (double dy = -current_range_xy; dy <= current_range_xy; dy += step_xy) {
                for (double dtheta = -current_range_theta; dtheta <= current_range_theta; dtheta += step_theta) {
                    
                    Pose test_pose = perturbPose(best_pose, dx, dy, dtheta);
                    double score = calculateMatchScore(scan, test_pose);
                    
                    if (score > level_best_score) {
                        level_best = test_pose;
                        level_best_score = score;
                    }
                }
            }
        }
        
        best_pose = level_best;
        best_score = level_best_score;
    }
    
    return best_pose;
}

double ScanMatcher::calculateMatchScore(const std::vector<LidarPoint>& scan, 
                                       const Pose& pose) {
    if (!map_) return 0.0;
    
    auto cartesian_points = scanToCartesian(scan, pose);
    double total_score = 0.0;
    int valid_points = 0;
    
    for (const auto& point : cartesian_points) {
        double x = point.first;
        double y = point.second;
        
        // Check map bounds
        double min_x, min_y, max_x, max_y;
        map_->getBounds(min_x, min_y, max_x, max_y);
        
        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
            double occupancy = map_->getOccupancy(x, y);
            
            // Improved scoring function
            if (occupancy > 0.7) {
                total_score += 2.0;  // Strong reward for hitting occupied areas
            } else if (occupancy > 0.5) {
                total_score += 1.0;  // Moderate reward for likely occupied areas
            } else if (occupancy < 0.3) {
                total_score -= 1.0;  // Penalty for hitting free space
            } else {
                total_score += 0.0;  // Neutral for unknown areas
            }
            
            valid_points++;
        }
    }
    
    if (valid_points == 0) return 0.0;
    
    // Normalize score and add bonus for more matched points
    double normalized_score = total_score / valid_points;
    double coverage_bonus = static_cast<double>(valid_points) / scan.size();
    
    return normalized_score + coverage_bonus;
}

std::vector<std::pair<double, double>> ScanMatcher::scanToCartesian(
    const std::vector<LidarPoint>& scan, const Pose& pose) {
    
    std::vector<std::pair<double, double>> points;
    points.reserve(scan.size());
    
    for (const auto& lidar_point : scan) {
        if (lidar_point.distance > 0.1 && lidar_point.distance < 30.0) {
            double global_angle = pose.theta + lidar_point.angle;
            double x = pose.x + lidar_point.distance * std::cos(global_angle);
            double y = pose.y + lidar_point.distance * std::sin(global_angle);
            points.emplace_back(x, y);
        }
    }
    
    return points;
}

Pose ScanMatcher::perturbPose(const Pose& pose, double dx, double dy, double dtheta) {
    return Pose(pose.x + dx, pose.y + dy, normalizeAngle(pose.theta + dtheta));
}

double ScanMatcher::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}