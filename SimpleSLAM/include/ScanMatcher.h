#pragma once
#include "SimpleSLAM.h"

class OccupancyMap;

class ScanMatcher {
public:
    ScanMatcher();
    ~ScanMatcher();
    
    // Main interface
    Pose matchScanToMap(const std::vector<LidarPoint>& scan,
                       const Pose& initial_guess,
                       const OccupancyMap* map);
    
    void setMap(const OccupancyMap* map) { map_ = map; }
    
    // Configuration
    void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
    void setConvergenceThreshold(double threshold) { convergence_threshold_ = threshold; }
    void setSearchRange(double xy_range, double theta_range) { 
        search_range_xy_ = xy_range; 
        search_range_theta_ = theta_range; 
    }
    
private:
    const OccupancyMap* map_;
    int max_iterations_;
    double convergence_threshold_;
    double search_range_xy_;
    double search_range_theta_;
    
    // Matching algorithms
    double calculateMatchScore(const std::vector<LidarPoint>& scan, 
                              const Pose& pose);
    Pose optimizePose(const std::vector<LidarPoint>& scan, 
                     const Pose& initial_pose);
    
    // Utility functions
    std::vector<std::pair<double, double>> scanToCartesian(
        const std::vector<LidarPoint>& scan, const Pose& pose);
    
    Pose perturbPose(const Pose& pose, double dx, double dy, double dtheta);
    double normalizeAngle(double angle);
};