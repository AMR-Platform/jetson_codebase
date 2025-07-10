#include "SimpleSLAM.h"
#include "OccupancyMap.h"
#include "ScanMatcher.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>

SimpleSLAM::SimpleSLAM() 
    : last_processing_time_(0.0)
    , initialized_(false)
    , next_scan_id_(0) {
}

SimpleSLAM::~SimpleSLAM() {
}

bool SimpleSLAM::initialize(double map_size, double resolution) {
    try {
        // Create map
        map_ = std::make_unique<OccupancyMap>(map_size, map_size, resolution);
        
        // Create scan matcher
        matcher_ = std::make_unique<ScanMatcher>();
        matcher_->setMap(map_.get());
        matcher_->setSearchRange(0.5, 0.2); // 50cm, ~11 degrees
        
        // Clear any existing data
        scans_.clear();
        optimized_poses_.clear();
        next_scan_id_ = 0;
        
        initialized_ = true;
        
        std::cout << "SimpleSLAM initialized successfully!" << std::endl;
        std::cout << "Map size: " << map_size << "x" << map_size << "m" << std::endl;
        std::cout << "Resolution: " << resolution << "m/cell" << std::endl;
        std::cout << "Grid size: " << map_->getWidth() << "x" << map_->getHeight() << " cells" << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize SimpleSLAM: " << e.what() << std::endl;
        return false;
    }
}

bool SimpleSLAM::addScan(const std::vector<LidarPoint>& scan, const Pose& initial_pose) {
    if (!initialized_) {
        std::cerr << "SimpleSLAM not initialized!" << std::endl;
        return false;
    }
    
    if (scan.empty()) {
        std::cerr << "Empty scan provided!" << std::endl;
        return false;
    }
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Refine pose using scan matching (if we have previous scans)
        Pose refined_pose = initial_pose;
        if (!scans_.empty() && matcher_) {
            refined_pose = refinePoseWithScanMatching(scan, initial_pose, next_scan_id_);
        }
        
        // Store scan data
        ScanData scan_data(scan, refined_pose, next_scan_id_++);
        scans_.push_back(scan_data);
        optimized_poses_.push_back(refined_pose);
        
        // Add to map
        map_->addScan(scan, refined_pose);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        last_processing_time_ = duration.count() / 1000.0; // Convert to milliseconds
        
        if (next_scan_id_ % 10 == 0 || next_scan_id_ <= 5) {
            std::cout << "Added scan " << (next_scan_id_ - 1) 
                      << " at pose (" << std::fixed << std::setprecision(2)
                      << refined_pose.x << ", " << refined_pose.y 
                      << ", " << refined_pose.theta * 180.0 / M_PI << "°)" 
                      << " - " << last_processing_time_ << "ms" << std::endl;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to add scan: " << e.what() << std::endl;
        return false;
    }
}

Pose SimpleSLAM::refinePoseWithScanMatching(const std::vector<LidarPoint>& scan, 
                                           const Pose& initial_pose, int scan_id) {
    if (scans_.empty() || !matcher_) {
        return initial_pose;  // No previous scans to match against
    }
    
    try {
        // Use scan matcher to refine pose against map
        Pose refined_pose = matcher_->matchScanToMap(scan, initial_pose, map_.get());
        
        // Calculate improvement
        double dx = refined_pose.x - initial_pose.x;
        double dy = refined_pose.y - initial_pose.y;
        double dtheta = refined_pose.theta - initial_pose.theta;
        double improvement = std::sqrt(dx*dx + dy*dy) + std::abs(dtheta);
        
        if (improvement > 0.05 && scan_id < 10) {  // Only print significant improvements for first few scans
            std::cout << "  Scan matching improved pose by " << std::fixed << std::setprecision(3) 
                      << improvement << " (dx=" << dx << ", dy=" << dy << ", dθ=" << dtheta*180/M_PI << "°)" << std::endl;
        }
        
        return refined_pose;
    } catch (const std::exception& e) {
        std::cerr << "Scan matching failed: " << e.what() << std::endl;
        return initial_pose;  // Fall back to initial pose
    }
}

bool SimpleSLAM::optimizeTrajectory() {
    if (!initialized_ || scans_.size() < 5) {
        std::cout << "Not enough scans for optimization (need at least 5, have " 
                  << scans_.size() << ")" << std::endl;
        return false;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        std::cout << "\n=== Starting Advanced Trajectory Optimization ===" << std::endl;
        std::cout << "Processing " << scans_.size() << " scans..." << std::endl;
        
        // PHASE 1: Scan-to-scan matching for consecutive poses
        std::cout << "Phase 1: Scan-to-map matching refinement..." << std::endl;
        
        int improvements = 0;
        for (size_t i = 1; i < scans_.size(); i++) {
            if (matcher_) {
                Pose original_pose = optimized_poses_[i];
                
                // Match current scan against current map
                Pose refined_pose = matcher_->matchScanToMap(
                    scans_[i].points, 
                    scans_[i].pose,
                    map_.get()
                );
                
                // Apply gradual correction to avoid jumps
                double weight = 0.3; // 30% correction
                
                optimized_poses_[i].x = (1 - weight) * optimized_poses_[i].x + weight * refined_pose.x;
                optimized_poses_[i].y = (1 - weight) * optimized_poses_[i].y + weight * refined_pose.y;
                optimized_poses_[i].theta = (1 - weight) * optimized_poses_[i].theta + weight * refined_pose.theta;
                
                scans_[i].pose = optimized_poses_[i];
                
                // Count significant improvements
                double dx = optimized_poses_[i].x - original_pose.x;
                double dy = optimized_poses_[i].y - original_pose.y;
                if (std::sqrt(dx*dx + dy*dy) > 0.05) {
                    improvements++;
                }
            }
        }
        std::cout << "  Improved " << improvements << " poses" << std::endl;
        
        // PHASE 2: Loop closure detection
        std::cout << "Phase 2: Loop closure detection..." << std::endl;
        detectAndCorrectLoops();
        
        // PHASE 3: Global trajectory smoothing
        std::cout << "Phase 3: Global trajectory smoothing..." << std::endl;
        smoothTrajectory();
        
        // PHASE 4: Rebuild map with optimized poses
        std::cout << "Phase 4: Rebuilding map with optimized poses..." << std::endl;
        updateMap();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        last_processing_time_ = duration.count();
        
        std::cout << "=== Optimization completed in " 
                  << last_processing_time_ << "ms ===" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Advanced optimization failed: " << e.what() << std::endl;
        return false;
    }
}

void SimpleSLAM::detectAndCorrectLoops() {
    int loop_closures = 0;
    
    // Simple loop closure: check if robot returns to similar position
    for (size_t i = 0; i < optimized_poses_.size(); i++) {
        for (size_t j = i + 20; j < optimized_poses_.size(); j++) { // At least 20 scans apart
            
            double dx = optimized_poses_[i].x - optimized_poses_[j].x;
            double dy = optimized_poses_[i].y - optimized_poses_[j].y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < 1.0) { // Within 1.0m - potential loop
                std::cout << "  Loop closure detected between scans " << i << " and " << j 
                          << " (distance: " << std::fixed << std::setprecision(2) << distance << "m)" << std::endl;
                
                // Apply loop closure correction
                correctLoop(i, j);
                loop_closures++;
                
                if (loop_closures >= 3) break; // Limit corrections per optimization
            }
        }
        if (loop_closures >= 3) break;
    }
    
    if (loop_closures == 0) {
        std::cout << "  No loop closures detected" << std::endl;
    } else {
        std::cout << "  Applied " << loop_closures << " loop closure corrections" << std::endl;
    }
}

void SimpleSLAM::correctLoop(size_t start_idx, size_t end_idx) {
    // Distribute the error over the trajectory segment
    Pose error;
    error.x = optimized_poses_[end_idx].x - optimized_poses_[start_idx].x;
    error.y = optimized_poses_[end_idx].y - optimized_poses_[start_idx].y;
    error.theta = optimized_poses_[end_idx].theta - optimized_poses_[start_idx].theta;
    
    // Normalize angle
    while (error.theta > M_PI) error.theta -= 2*M_PI;
    while (error.theta < -M_PI) error.theta += 2*M_PI;
    
    size_t segment_length = end_idx - start_idx;
    
    for (size_t i = start_idx + 1; i < end_idx; i++) {
        double ratio = static_cast<double>(i - start_idx) / segment_length;
        
        optimized_poses_[i].x -= ratio * error.x * 0.5; // Apply 50% correction
        optimized_poses_[i].y -= ratio * error.y * 0.5;
        optimized_poses_[i].theta -= ratio * error.theta * 0.5;
        
        scans_[i].pose = optimized_poses_[i];
    }
}

void SimpleSLAM::smoothTrajectory() {
    if (optimized_poses_.size() < 3) return;
    
    // Multiple passes of smoothing
    for (int pass = 0; pass < 2; pass++) {
        std::vector<Pose> smoothed_poses = optimized_poses_;
        
        for (size_t i = 2; i < optimized_poses_.size() - 2; i++) {
            // 5-point smoothing
            smoothed_poses[i].x = 0.1 * optimized_poses_[i-2].x + 0.2 * optimized_poses_[i-1].x + 
                                 0.4 * optimized_poses_[i].x + 0.2 * optimized_poses_[i+1].x + 0.1 * optimized_poses_[i+2].x;
            smoothed_poses[i].y = 0.1 * optimized_poses_[i-2].y + 0.2 * optimized_poses_[i-1].y + 
                                 0.4 * optimized_poses_[i].y + 0.2 * optimized_poses_[i+1].y + 0.1 * optimized_poses_[i+2].y;
            
            // Smooth angle with circular mean
            double sin_sum = 0.1 * std::sin(optimized_poses_[i-2].theta) + 0.2 * std::sin(optimized_poses_[i-1].theta) + 
                            0.4 * std::sin(optimized_poses_[i].theta) + 0.2 * std::sin(optimized_poses_[i+1].theta) + 
                            0.1 * std::sin(optimized_poses_[i+2].theta);
            double cos_sum = 0.1 * std::cos(optimized_poses_[i-2].theta) + 0.2 * std::cos(optimized_poses_[i-1].theta) + 
                            0.4 * std::cos(optimized_poses_[i].theta) + 0.2 * std::cos(optimized_poses_[i+1].theta) + 
                            0.1 * std::cos(optimized_poses_[i+2].theta);
            smoothed_poses[i].theta = std::atan2(sin_sum, cos_sum);
        }
        
        optimized_poses_ = smoothed_poses;
        
        // Update scan poses
        for (size_t i = 0; i < scans_.size(); i++) {
            scans_[i].pose = optimized_poses_[i];
        }
    }
    
    std::cout << "  Applied trajectory smoothing" << std::endl;
}

void SimpleSLAM::updateMap() {
    if (!map_) return;
    
    // Clear and rebuild map with optimized poses
    map_->clear();
    
    for (const auto& scan_data : scans_) {
        map_->addScan(scan_data.points, scan_data.pose);
    }
    
    // Apply post-processing
    map_->postProcessMap();
}

std::vector<Pose> SimpleSLAM::getOptimizedPoses() const {
    return optimized_poses_;
}

void SimpleSLAM::saveMap(const std::string& filename) const {
    if (map_) {
        map_->saveToFile(filename);
    }
}

void SimpleSLAM::saveTrajectory(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    file << "# x y theta" << std::endl;
    for (const auto& pose : optimized_poses_) {
        file << std::fixed << std::setprecision(6) 
             << pose.x << " " << pose.y << " " << pose.theta << std::endl;
    }
    
    file.close();
    std::cout << "Trajectory saved to: " << filename << std::endl;
}

int SimpleSLAM::getNumScans() const {
    return static_cast<int>(scans_.size());
}

void SimpleSLAM::clear() {
    scans_.clear();
    optimized_poses_.clear();
    next_scan_id_ = 0;
    
    if (map_) {
        map_->clear();
    }
}

void SimpleSLAM::printStats() const {
    std::cout << "\n=== SimpleSLAM Statistics ===" << std::endl;
    std::cout << "Initialized: " << (initialized_ ? "Yes" : "No") << std::endl;
    std::cout << "Number of scans: " << scans_.size() << std::endl;
    std::cout << "Last processing time: " << last_processing_time_ << "ms" << std::endl;
    
    if (map_) {
        std::cout << "Map size: " << map_->getWidth() << "x" << map_->getHeight() 
                  << " cells" << std::endl;
        std::cout << "Map resolution: " << map_->getResolution() << "m/cell" << std::endl;
        
        double min_x, min_y, max_x, max_y;
        map_->getBounds(min_x, min_y, max_x, max_y);
        std::cout << "Map bounds: (" << min_x << ", " << min_y << ") to (" 
                  << max_x << ", " << max_y << ")" << std::endl;
    }
    
    if (!optimized_poses_.empty()) {
        const auto& first = optimized_poses_.front();
        const auto& last = optimized_poses_.back();
        std::cout << "Trajectory: (" << first.x << ", " << first.y << ") to (" 
                  << last.x << ", " << last.y << ")" << std::endl;
        
        double total_distance = 0.0;
        for (size_t i = 1; i < optimized_poses_.size(); i++) {
            double dx = optimized_poses_[i].x - optimized_poses_[i-1].x;
            double dy = optimized_poses_[i].y - optimized_poses_[i-1].y;
            total_distance += std::sqrt(dx*dx + dy*dy);
        }
        std::cout << "Total distance traveled: " << std::fixed << std::setprecision(2) 
                  << total_distance << "m" << std::endl;
    }
    std::cout << "==============================\n" << std::endl;
}