#include "pose_localiser.hpp"
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

// External references to your existing global variables
extern std::vector<LidarPoint> g_scan;       // from main.cpp
extern SensorFusion g_ekf;                   // from main.cpp
extern RobotLocalization localize;           // from main.cpp

/* ── Tunable parameters ─────────────────────────────────────────────── */
static double MAP_RES           = 0.05;   // metres per grid cell
static const double XY_RANGE    = 1.0;    // ± search window (m)
static const double XY_STEP     = 0.05;   // coarse grid step (m)
static const int    THETA_STEP  = 2;      // coarse step (deg)
static const double SIGMA_LASER = 0.10;   // likelihood-field σ (m)

/* ── Utility helpers ────────────────────────────────────────────────── */
static inline cv::Point w2m(const Eigen::Vector2d& w,
                            const cv::Point2d& origin)
{
    return { static_cast<int>(std::round((w.x() - origin.x) / MAP_RES)),
             static_cast<int>(std::round((w.y() - origin.y) / MAP_RES)) };
}

static inline bool inside(const cv::Point& p, const cv::Mat& I)
{ 
    return p.x >= 0 && p.y >= 0 && p.x < I.cols && p.y < I.rows; 
}

/* ── Main routine ───────────────────────────────────────────────────── */
Pose2D runSpinLocalisation(const cv::Mat1b& occ,
                           const cv::Point2d& originW)
{
    std::cout << "[SPIN_LOC] Starting 360° spin localization..." << std::endl;
    
    /* ---------------------------------------------------------------
     * 0.  Rotate robot 360°  
     * TODO: Replace with your actual motor control command
     * Example: serial->sendCommand(createRotationCommand(360.0));
     * ------------------------------------------------------------- */
    
    // For now, just wait for manual rotation or implement your rotation command
    std::cout << "[SPIN_LOC] Please rotate robot 360° or implement rotation command" << std::endl;
    std::cout << "[SPIN_LOC] Press Enter when rotation is complete..." << std::endl;
    std::cin.get(); // Wait for user input - replace with actual rotation command
    
    /* 1. Fetch the latest LiDAR scan (must be full revolution) */
    const auto& scan = g_scan;
    if (scan.empty()) {
        std::cout << "[SPIN_LOC] No LiDAR data available, using current EKF pose" << std::endl;
        
        // Get current EKF state - you'll need to add getter methods to your SensorFusion class
        // For now, return a default pose - you should replace this with actual EKF state access
        return { 0.0, 0.0, 0.0 }; // TODO: Get actual EKF state
    }
    
    std::cout << "[SPIN_LOC] Processing " << scan.size() << " LiDAR points" << std::endl;

    /* 2. Build (or reuse cached) distance + gradient images */
    static cv::Mat1f distImg, gradX, gradY;
    static bool cacheReady = false;
    
    if (!cacheReady) {
        std::cout << "[SPIN_LOC] Building distance transform and gradients..." << std::endl;
        cv::distanceTransform(255 - occ, distImg, cv::DIST_L2, 3);
        distImg *= MAP_RES;
        cv::Sobel(distImg, gradX, CV_32F, 1, 0, 3);
        cv::Sobel(distImg, gradY, CV_32F, 0, 1, 3);
        cacheReady = true;
        std::cout << "[SPIN_LOC] Distance transform complete" << std::endl;
    }

    /* 3. Get initial guess from EKF */
    // TODO: Replace with actual EKF state getter methods
    // You'll need to add methods to SensorFusion class like:
    // double getX() const { return state_(0); }
    // double getY() const { return state_(1); }  
    // double getYaw() const { return state_(2); }
    
    Pose2D best{ 0.0, 0.0, 0.0 }; // TODO: Get from g_ekf.getX(), g_ekf.getY(), g_ekf.getYaw()
    double bestScore = -std::numeric_limits<double>::infinity();
    
    std::cout << "[SPIN_LOC] Starting coarse grid search around EKF pose: (" 
              << best.x << ", " << best.y << ", " << best.theta << ")" << std::endl;

    /* 3-A. Coarse grid search around EKF pose */
    int totalSearches = 0;
    int validScores = 0;
    
    for (int dth = -180; dth < 180; dth += THETA_STEP) {
        double theta = best.theta + dth * M_PI / 180.0;
        Eigen::Rotation2D<double> Rtheta(theta);

        /* Pre-rotate scan points once for this θ */
        std::vector<Eigen::Vector2d> scanRot;
        scanRot.reserve(scan.size());
        for (const auto& b : scan) {
            // Convert azimuth from degrees to radians
            double a = b.azimuth * M_PI / 180.0;
            scanRot.emplace_back(Rtheta *
                Eigen::Vector2d(b.distance * std::cos(a),
                                b.distance * std::sin(a)));
        }

        for (double dx = -XY_RANGE; dx <= XY_RANGE; dx += XY_STEP) {
            for (double dy = -XY_RANGE; dy <= XY_RANGE; dy += XY_STEP) {
                totalSearches++;
                double score = 0.0;
                int validPoints = 0;
                
                for (const auto& p : scanRot) {
                    Eigen::Vector2d w = p + Eigen::Vector2d(best.x + dx, best.y + dy);
                    cv::Point mp = w2m(w, originW);
                    if (!inside(mp, occ)) continue;
                    
                    float d = distImg(mp);
                    score += std::exp(-d * d / (2 * SIGMA_LASER * SIGMA_LASER));
                    validPoints++;
                }
                
                if (validPoints > 0) {
                    validScores++;
                    score /= validPoints; // Normalize by number of valid points
                    
                    if (score > bestScore) {
                        bestScore = score;
                        best = { best.x + dx, best.y + dy, theta };
                    }
                }
            }
        }
    }
    
    std::cout << "[SPIN_LOC] Coarse search complete. Evaluated " << totalSearches 
              << " poses, " << validScores << " valid scores" << std::endl;
    std::cout << "[SPIN_LOC] Best coarse pose: (" << best.x << ", " << best.y 
              << ", " << best.theta << ") score: " << bestScore << std::endl;

    /* 3-B. Two Gauss–Newton steps to get sub-cell accuracy */
    std::cout << "[SPIN_LOC] Starting Gauss-Newton refinement..." << std::endl;
    
    for (int it = 0; it < 2; ++it) {
        Eigen::Vector3d JtF = Eigen::Vector3d::Zero();
        Eigen::Matrix3d H   = Eigen::Matrix3d::Zero();
        Eigen::Rotation2D<double> Rbest(best.theta);
        int validPoints = 0;

        for (const auto& b : scan) {
            double a  = b.azimuth * M_PI / 180.0;
            Eigen::Vector2d pL(b.distance * std::cos(a),
                               b.distance * std::sin(a));
            Eigen::Vector2d w  = Rbest * pL + Eigen::Vector2d(best.x, best.y);
            cv::Point mp = w2m(w, originW);
            if (!inside(mp, distImg)) continue;

            float d  = distImg(mp);
            float gx = gradX(mp), gy = gradY(mp);

            Eigen::Vector3d J(gx, gy, -gx * pL.y() + gy * pL.x());
            JtF += J * d;
            H   += J * J.transpose();
            validPoints++;
        }
        
        if (validPoints > 10) { // Ensure we have enough points for reliable estimation
            Eigen::Vector3d delta = -H.ldlt().solve(JtF);
            best.x     += delta(0);
            best.y     += delta(1);
            best.theta += delta(2);
            
            std::cout << "[SPIN_LOC] Gauss-Newton iteration " << (it+1) 
                      << ": delta = (" << delta(0) << ", " << delta(1) 
                      << ", " << delta(2) << ")" << std::endl;
        } else {
            std::cout << "[SPIN_LOC] Warning: Not enough valid points for Gauss-Newton iteration" << std::endl;
            break;
        }
    }

    std::cout << "[SPIN_LOC] Final refined pose: (" << best.x << ", " << best.y 
              << ", " << best.theta << ")" << std::endl;

    return best;
}
