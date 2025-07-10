#pragma once
#include <vector>
#include <array>
#include <string>
#include <memory>

#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <iostream>

struct LidarPoint {
    double angle;     // radians, relative to robot
    double distance;  // meters
    int intensity;    // optional
    
    LidarPoint() : angle(0), distance(0), intensity(0) {}
    LidarPoint(double a, double d, int i = 0) : angle(a), distance(d), intensity(i) {}
};

struct Pose {
    double x, y, theta;  // x, y in meters, theta in radians
    
    Pose() : x(0), y(0), theta(0) {}
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

// Forward declarations
class OccupancyMap;
class ScanMatcher;

class SimpleSLAM {
public:
    SimpleSLAM();
    ~SimpleSLAM();
    
    // Core interface
    bool initialize(double map_size = 50.0, double resolution = 0.1);
    bool addScan(const std::vector<LidarPoint>& scan, const Pose& initial_pose);
    bool optimizeTrajectory();
    
    // Results
    std::vector<Pose> getOptimizedPoses() const;
    void saveMap(const std::string& filename) const;
    void saveTrajectory(const std::string& filename) const;
    
    // Status
    int getNumScans() const;
    double getLastProcessingTime() const { return last_processing_time_; }
    bool isInitialized() const { return initialized_; }
    void printStats() const;
    void clear();
    
private:
    struct ScanData {
        std::vector<LidarPoint> points;
        Pose pose;
        int id;
        
        ScanData() : id(0) {}
        ScanData(const std::vector<LidarPoint>& p, const Pose& pose_, int id_) 
            : points(p), pose(pose_), id(id_) {}
    };
    
    std::vector<ScanData> scans_;
    std::vector<Pose> optimized_poses_;
    std::unique_ptr<OccupancyMap> map_;
    std::unique_ptr<ScanMatcher> matcher_;
    double last_processing_time_;
    bool initialized_;
    int next_scan_id_;
    
    // Internal methods
    Pose refinePoseWithScanMatching(const std::vector<LidarPoint>& scan, 
                                   const Pose& initial_pose, int scan_id);
    void updateMap();
    void detectAndCorrectLoops();
    void correctLoop(size_t start_idx, size_t end_idx);
    void smoothTrajectory();
    double getCurrentTime() const;
};