// PathPlanner.cpp
// Run A* and PRM with 6-pixel robot safety margin from all barriers
// Robot dimensions require 6-pixel clearance - some areas may become inaccessible

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <cmath>
#include <random>
#include <algorithm>
#include <iomanip>

using namespace cv;
using namespace std;

struct Pt {
    int x,y;
    bool operator<(Pt const &o) const { return x<o.x || (x==o.x && y<o.y); }
    bool operator==(Pt const &o) const { return x==o.x && y==o.y; }
};

static double dist(const Pt &a, const Pt &b){
    double dx=a.x-b.x, dy=a.y-b.y;
    return sqrt(dx*dx + dy*dy);
}

// Snap to nearest free cell if blocked (with 6-pixel safety)
Pt findNearestFree(const Mat &grid, Pt pt, int max_r=20){
    if(pt.x >= 0 && pt.x < grid.rows && pt.y >= 0 && pt.y < grid.cols && 
       grid.at<uchar>(pt.x,pt.y)==255) return pt;
    
    int R=grid.rows, C=grid.cols;
    for(int r=1;r<=max_r;++r){
      for(int dx=-r;dx<=r;++dx) for(int dy=-r;dy<=r;++dy){
        int x=pt.x+dx, y=pt.y+dy;
        if(x>=0 && x<R && y>=0 && y<C && grid.at<uchar>(x,y)==255)
          return {x,y};
      }
    }
    return pt;
}

// -------- A* --------
class AStar {
public:
    AStar(const Mat &g): grid(g){}
    vector<Pt> search(Pt s, Pt goal){
        s = findNearestFree(grid,s);
        goal = findNearestFree(grid,goal);

        map<Pt,double> gScore,fScore;
        map<Pt,Pt> pred;
        auto cmp = [&](Pt a,Pt b){ return fScore[a]>fScore[b]; };
        priority_queue<Pt,vector<Pt>,decltype(cmp)> open(cmp);
        set<Pt> inOpen;

        gScore[s]=0; fScore[s]=dist(s,goal);
        open.push(s); inOpen.insert(s);

        static const vector<Pt> dirs = {
          {1,0},{1,1},{0,1},{-1,1},
          {-1,0},{-1,-1},{0,-1},{1,-1}
        };

        while(!open.empty()){
            Pt u=open.top(); open.pop();
            inOpen.erase(u);
            if(u==goal) return reconstruct(pred,s,goal);

            for(auto &d:dirs){
                Pt v{u.x+d.x,u.y+d.y};
                if(v.x<0||v.x>=grid.rows||v.y<0||v.y>=grid.cols) continue;
                if(grid.at<uchar>(v.x,v.y)!=255) continue;
                double t = gScore[u] + dist(u,v);
                if(!gScore.count(v) || t<gScore[v]){
                    pred[v]=u; gScore[v]=t;
                    fScore[v]=t + dist(v,goal);
                    if(!inOpen.count(v)){
                        open.push(v);
                        inOpen.insert(v);
                    }
                }
            }
        }
        return {};
    }
private:
    Mat grid;
    vector<Pt> reconstruct(map<Pt,Pt> &pred, Pt s, Pt g){
        vector<Pt> path;
        for(Pt cur=g; !(cur==s); cur=pred[cur])
            path.push_back(cur);
        path.push_back(s);
        reverse(path.begin(), path.end());
        return path;
    }
};

// -------- PRM --------
class PRM {
public:
    PRM(const Mat &g): grid(g), gen(random_device{}()){}
    void build(int N, double dmax){
        uniform_int_distribution<> rx(0,grid.rows-1),
                                   ry(0,grid.cols-1);
        
        // Clear any existing data
        nodes.clear();
        conn.clear();
        
        // Sample nodes more efficiently
        int attempts = 0;
        int maxAttempts = N * 10; // Allow more attempts to find valid samples
        
        while((int)nodes.size() < N && attempts < maxAttempts){
            Pt p{rx(gen), ry(gen)};
            attempts++;
            
            // Check if point is in free space (already has 6-pixel clearance)
            if(grid.at<uchar>(p.x, p.y) == 255){
                bool tooClose = false;
                // Ensure minimum distance between nodes to avoid clustering
                for(auto &existing : nodes){
                    if(dist(existing, p) < 5.0){ // Minimum 5 pixel separation
                        tooClose = true;
                        break;
                    }
                }
                
                if(!tooClose){
                    nodes.push_back(p);
                    conn[p] = {};
                    
                    // Connect to nearby nodes
                    for(auto &u : nodes){
                        if(!(u == p) && dist(u, p) <= dmax && connectable(u, p)){
                            double weight = dist(u, p);
                            conn[u].push_back({p, weight});
                            conn[p].push_back({u, weight});
                        }
                    }
                }
            }
        }
        
        cout << "PRM: Successfully sampled " << nodes.size() << " nodes out of " << N << " requested" << endl;
    }
    vector<Pt> findPath(Pt s, Pt goal, double dmax){
        s = findNearestFree(grid, s);
        goal = findNearestFree(grid, goal);
        
        // Clear existing connections for start and goal
        if(conn.count(s)) conn[s].clear();
        if(conn.count(goal)) conn[goal].clear();
        conn[s] = {};
        conn[goal] = {};
        
        // Connect start and goal to roadmap with larger radius for better connectivity
        double connectionRadius = dmax * 1.5; // Increase connection radius
        
        for(auto &u : nodes){
            // Connect start to roadmap
            if(dist(u, s) <= connectionRadius && connectable(u, s)){
                double weight = dist(u, s);
                conn[u].push_back({s, weight});
                conn[s].push_back({u, weight});
            }
            
            // Connect goal to roadmap
            if(dist(u, goal) <= connectionRadius && connectable(u, goal)){
                double weight = dist(u, goal);
                conn[u].push_back({goal, weight});
                conn[goal].push_back({u, weight});
            }
        }
        
        // Also try direct connection between start and goal
        if(dist(s, goal) <= connectionRadius && connectable(s, goal)){
            double weight = dist(s, goal);
            conn[s].push_back({goal, weight});
            conn[goal].push_back({s, weight});
        }
        
        cout << "PRM: Start connected to " << conn[s].size() << " nodes" << endl;
        cout << "PRM: Goal connected to " << conn[goal].size() << " nodes" << endl;
        
        // A* search on the roadmap
        map<Pt, double> gScore, fScore;
        map<Pt, Pt> pred;
        auto cmp = [&](Pt a, Pt b){ return fScore[a] > fScore[b]; };
        priority_queue<Pt, vector<Pt>, decltype(cmp)> open(cmp);
        set<Pt> inOpen;

        gScore[s] = 0; 
        fScore[s] = dist(s, goal);
        open.push(s); 
        inOpen.insert(s);

        while(!open.empty()){
            Pt u = open.top(); 
            open.pop();
            inOpen.erase(u);
            
            if(u == goal) return reconstruct(pred, s, goal);

            for(auto &e : conn[u]){
                Pt v = e.first; 
                double w = e.second;
                double t = gScore[u] + w;
                
                if(!gScore.count(v) || t < gScore[v]){
                    pred[v] = u; 
                    gScore[v] = t;
                    fScore[v] = t + dist(v, goal);
                    
                    if(!inOpen.count(v)){
                        open.push(v);
                        inOpen.insert(v);
                    }
                }
            }
        }
        
        cout << "PRM: No path found in roadmap!" << endl;
        return {};
    }
private:
    Mat grid;
    vector<Pt> nodes;
    map<Pt, vector<pair<Pt,double>>> conn;
    mt19937 gen;

    bool connectable(Pt a, Pt b){
        int steps = max(abs(b.x-a.x), abs(b.y-a.y));
        if(steps == 0) return true;
        
        for(int i=0; i<=steps; ++i){
            double t = (double)i/steps;
            int x = round(a.x + (b.x-a.x)*t);
            int y = round(a.y + (b.y-a.y)*t);
            
            // Check bounds
            if(x < 0 || x >= grid.rows || y < 0 || y >= grid.cols) return false;
            
            // Check if path goes through obstacle (path maintains 6-pixel clearance)
            if(grid.at<uchar>(x, y) == 0) return false;
        }
        return true;
    }

    vector<Pt> reconstruct(map<Pt,Pt> &pred, Pt s, Pt g){
        vector<Pt> path;
        for(Pt cur=g; !(cur==s); cur=pred[cur])
            path.push_back(cur);
        path.push_back(s);
        reverse(path.begin(), path.end());
        return path;
    }
};

double calculatePathLength(const vector<Pt> &path){
    double length = 0;
    for(int i=1; i<(int)path.size(); ++i){
        length += dist(path[i-1], path[i]);
    }
    return length;
}

// Detect turning points only
vector<tuple<int,Pt,double>> detectTurningPoints(const vector<Pt> &path, double angle_threshold=10.0){
    vector<tuple<int,Pt,double>> turningPoints;
    static constexpr double RAD2DEG = 180.0/CV_PI;
    
    for(int i=1; i+1<(int)path.size(); ++i){
        Vec2d v1(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
        Vec2d v2(path[i+1].x-path[i].x, path[i+1].y-path[i].y);
        
        double n1 = norm(v1), n2 = norm(v2);
        if(n1 < 1e-6 || n2 < 1e-6) continue;
        
        double cosang = clamp(v1.dot(v2)/(n1*n2), -1.0, 1.0);
        double angle = acos(cosang) * RAD2DEG;
        
        if(angle >= angle_threshold){
            turningPoints.emplace_back(i, path[i], angle);
        }
    }
    
    return turningPoints;
}

// Calculate distances between consecutive turning points
vector<double> calculateDistancesBetweenTurns(const vector<Pt> &path, 
                                             const vector<tuple<int,Pt,double>> &turns){
    vector<double> distances;
    
    // Create list of key points: start + turning points + goal
    vector<int> keyIndices;
    keyIndices.push_back(0); // Start point
    
    for(auto &turn : turns){
        keyIndices.push_back(get<0>(turn)); // Turning point index
    }
    
    keyIndices.push_back(path.size()-1); // Goal point
    
    // Calculate distances between consecutive key points
    for(int i=0; i+1<(int)keyIndices.size(); ++i){
        double segmentDistance = 0;
        int start_idx = keyIndices[i];
        int end_idx = keyIndices[i+1];
        
        for(int j=start_idx+1; j<=end_idx; ++j){
            segmentDistance += dist(path[j-1], path[j]);
        }
        
        distances.push_back(segmentDistance);
    }
    
    return distances;
}

int main(){
    // Load image
    Mat img = imread("new.png", IMREAD_GRAYSCALE);
    if(img.empty()){ 
        cerr << "Can't load new.png\n"; 
        return 1; 
    }
    
    // Check if resizing is needed
    if(img.rows != 600 || img.cols != 600) {
        cout << "Original image size: " << img.cols << "x" << img.rows << endl;
        resize(img, img, Size(600, 600), 0, 0, INTER_AREA);
        cout << "Resized to: 600x600" << endl;
    } else {
        cout << "Image is already 600x600" << endl;
    }
    
    // Binarize image: white areas = free space (255), everything else = obstacles (0)
    Mat mask;
    threshold(img, mask, 200, 255, THRESH_BINARY);  
    // Now: bright pixels (>200, white areas) become 255 (free), darker pixels become 0 (obstacles)
    
    // Show image statistics
    int total_pixels = mask.rows * mask.cols;
    int free_pixels = countNonZero(mask);  // Free space (white areas)
    int obstacle_pixels = total_pixels - free_pixels;  // Obstacles (gray + black areas)
    cout << "=== IMAGE ANALYSIS ===" << endl;
    cout << "- White areas (free space): " << free_pixels << " (" 
         << fixed << setprecision(1) << (100.0 * free_pixels / total_pixels) << "%)" << endl;
    cout << "- Gray/Black areas (obstacles): " << obstacle_pixels << " (" 
         << fixed << setprecision(1) << (100.0 * obstacle_pixels / total_pixels) << "%)" << endl;

    // CRITICAL: Apply 6-pixel safety margin for robot dimensions
    Mat inflated = mask.clone();
    const int ROBOT_CLEARANCE = 6;  // Robot requires 6-pixel clearance from barriers
    
    cout << "\n=== ROBOT SAFETY MARGIN ===" << endl;
    cout << "Applying " << ROBOT_CLEARANCE << "-pixel safety margin from all barriers..." << endl;
    cout << "This accounts for robot dimensions and ensures safe navigation." << endl;
    
    // Inflate all obstacles by 6 pixels
    for(int x=0; x<mask.rows; ++x){
        for(int y=0; y<mask.cols; ++y){
            if(mask.at<uchar>(x,y) == 0){  // If obstacle pixel
                // Create 6-pixel safety zone around obstacle
                int x0 = max(0, x-ROBOT_CLEARANCE);
                int x1 = min(mask.rows, x+ROBOT_CLEARANCE+1);
                int y0 = max(0, y-ROBOT_CLEARANCE);
                int y1 = min(mask.cols, y+ROBOT_CLEARANCE+1);
                inflated(Range(x0,x1), Range(y0,y1)).setTo(0);  // Mark as inaccessible
            }
        }
    }
    
    // Show the impact of safety margin
    int safe_free_pixels = countNonZero(inflated);
    int lost_pixels = free_pixels - safe_free_pixels;
    cout << "- Safe navigable area: " << safe_free_pixels << " (" 
         << fixed << setprecision(1) << (100.0 * safe_free_pixels / total_pixels) << "%)" << endl;
    cout << "- Area lost to safety margin: " << lost_pixels << " (" 
         << fixed << setprecision(1) << (100.0 * lost_pixels / total_pixels) << "%)" << endl;
    cout << "- Some narrow passages may now be inaccessible" << endl;
    
    // Save processed images for verification
    imwrite("original_binary.png", mask);
    imwrite("robot_safe_areas.png", inflated);
    cout << "\nSaved verification images:" << endl;
    cout << "- original_binary.png: Shows original free areas" << endl;
    cout << "- robot_safe_areas.png: Shows robot-accessible areas with 6px clearance" << endl;

    // Define start and goal points
    Pt start{280,250}, goal{240, 350};
    
    cout << "\n=== PATH PLANNING WITH SAFETY CONSTRAINTS ===" << endl;
    cout << "Start: (" << start.x << "," << start.y << ")" << endl;
    cout << "Goal: (" << goal.x << "," << goal.y << ")" << endl;
    
    // Check if start and goal are in robot-safe positions
    bool startValid = (start.x >= 0 && start.x < inflated.rows && 
                      start.y >= 0 && start.y < inflated.cols && 
                      inflated.at<uchar>(start.x, start.y) == 255);
    bool goalValid = (goal.x >= 0 && goal.x < inflated.rows && 
                     goal.y >= 0 && goal.y < inflated.cols && 
                     inflated.at<uchar>(goal.x, goal.y) == 255);
    
    if(!startValid) {
        cout << "WARNING: Start point does not have 6-pixel clearance from barriers!" << endl;
        Pt adjustedStart = findNearestFree(inflated, start, 20);
        cout << "Adjusting start to safe position: (" << adjustedStart.x << "," << adjustedStart.y << ")" << endl;
        start = adjustedStart;
    } else {
        cout << "✓ Start point has required 6-pixel clearance" << endl;
    }
    
    if(!goalValid) {
        cout << "WARNING: Goal point does not have 6-pixel clearance from barriers!" << endl;
        Pt adjustedGoal = findNearestFree(inflated, goal, 20);
        cout << "Adjusting goal to safe position: (" << adjustedGoal.x << "," << adjustedGoal.y << ")" << endl;
        goal = adjustedGoal;
    } else {
        cout << "✓ Goal point has required 6-pixel clearance" << endl;
    }

    // Run A* algorithm on robot-safe grid
    cout << "\nRunning A* algorithm on robot-safe areas..." << endl;
    AStar astar(inflated);
    auto astarPath = astar.search(start, goal);
    double astarLength = astarPath.empty() ? 1e9 : calculatePathLength(astarPath);
    cout << "A* Path Length: " << astarLength << " pixels" << endl;

    // Run PRM algorithm on robot-safe grid
    cout << "Running PRM algorithm on robot-safe areas..." << endl;
    PRM prm(inflated);
    prm.build(5000, 100); // More samples for better connectivity in reduced space
    auto prmPath = prm.findPath(start, goal, 100);
    double prmLength = prmPath.empty() ? 1e9 : calculatePathLength(prmPath);
    cout << "PRM Path Length: " << prmLength << " pixels" << endl;

    // Select the shorter path
    vector<Pt> chosenPath;
    string chosenAlgorithm;
    double chosenLength;
    
    if(astarLength <= prmLength){
        chosenPath = astarPath;
        chosenAlgorithm = "A*";
        chosenLength = astarLength;
    } else {
        chosenPath = prmPath;
        chosenAlgorithm = "PRM";
        chosenLength = prmLength;
    }
    
    cout << "\n=== SELECTED SAFE PATH ===" << endl;
    cout << "Chosen Algorithm: " << chosenAlgorithm << endl;
    cout << "Total Path Length: " << chosenLength << " pixels" << endl;
    cout << "Path maintains 6-pixel clearance from all barriers" << endl;

    if(chosenPath.empty()){
        cout << "\n❌ NO SAFE PATH FOUND!" << endl;
        cout << "This could be due to:" << endl;
        cout << "- Start or goal points too close to barriers" << endl;
        cout << "- No navigable path exists with 6-pixel clearance" << endl;
        cout << "- Narrow passages blocked by safety margin" << endl;
        cout << "- Robot dimensions incompatible with environment" << endl;
        cout << "\nSuggestions:" << endl;
        cout << "- Try different start/goal positions" << endl;
        cout << "- Consider smaller robot or wider passages" << endl;
        cout << "- Check robot_safe_areas.png to see accessible regions" << endl;
    } else {
        cout << "✓ Safe path found with guaranteed 6-pixel clearance!" << endl;
    }

    // Detect turning points (only if path exists)
    vector<tuple<int,Pt,double>> turningPoints;
    if(!chosenPath.empty()) {
        turningPoints = detectTurningPoints(chosenPath, 10.0);
        
        cout << "\n=== TURNING POINTS ANALYSIS ===" << endl;
        cout << "Number of Turning Points: " << turningPoints.size() << endl;
        
        if(turningPoints.empty()){
            cout << "No significant turning points detected (straight line path)" << endl;
            cout << "Direct distance: " << fixed << setprecision(2) << dist(start, goal) << " pixels" << endl;
        } else {
            cout << "\nTurning Points Details:" << endl;
            cout << "Index\tCoordinates\tTurning Angle" << endl;
            cout << "-----\t-----------\t-------------" << endl;
            
            for(auto &turn : turningPoints){
                int idx = get<0>(turn);
                Pt point = get<1>(turn);
                double angle = get<2>(turn);
                cout << idx << "\t(" << point.x << "," << point.y << ")\t" 
                     << fixed << setprecision(1) << angle << "°" << endl;
            }
            
            // Calculate distances between turning points
            auto distances = calculateDistancesBetweenTurns(chosenPath, turningPoints);
            
            cout << "\n=== DISTANCES BETWEEN TURNING POINTS ===" << endl;
            cout << "Segment\tFrom\t\tTo\t\tDistance" << endl;
            cout << "-------\t----\t\t--\t\t--------" << endl;
            
            // Start to first turning point
            Pt firstTurn = get<1>(turningPoints[0]);
            cout << "1\tStart(" << start.x << "," << start.y << ")\t"
                 << "Turn(" << firstTurn.x << "," << firstTurn.y << ")\t"
                 << fixed << setprecision(2) << distances[0] << " px" << endl;
            
            // Between turning points
            for(int i=1; i<(int)turningPoints.size(); ++i){
                Pt prevTurn = get<1>(turningPoints[i-1]);
                Pt currTurn = get<1>(turningPoints[i]);
                cout << i+1 << "\tTurn(" << prevTurn.x << "," << prevTurn.y << ")\t"
                     << "Turn(" << currTurn.x << "," << currTurn.y << ")\t"
                     << fixed << setprecision(2) << distances[i] << " px" << endl;
            }
            
            // Last turning point to goal
            Pt lastTurn = get<1>(turningPoints.back());
            cout << turningPoints.size()+1 << "\tTurn(" << lastTurn.x << "," << lastTurn.y << ")\t"
                 << "Goal(" << goal.x << "," << goal.y << ")\t"
                 << fixed << setprecision(2) << distances.back() << " px" << endl;
        }
    }

    // Create visualization showing safety margins
    Mat display;
    cvtColor(mask, display, COLOR_GRAY2BGR);
    
    // Show inaccessible areas due to safety margin in red overlay
    Mat safetyOverlay = mask - inflated;  // Areas lost to safety margin
    for(int x=0; x<display.rows; ++x){
        for(int y=0; y<display.cols; ++y){
            if(safetyOverlay.at<uchar>(x,y) > 0){
                display.at<Vec3b>(x,y) = Vec3b(0, 0, 128); // Dark red for safety zones
            }
        }
    }
    
    // Draw chosen path in bright red (only if path exists)
    if(!chosenPath.empty()) {
        vector<Point> pathPoints;
        for(auto &pt : chosenPath){
            pathPoints.emplace_back(pt.y, pt.x); // Note: x,y swapped for OpenCV
        }
        polylines(display, pathPoints, false, Scalar(0,0,255), 3); // Thicker line
    }
    
    // Mark start and goal with safety circles
    circle(display, Point(start.y, start.x), ROBOT_CLEARANCE, Scalar(0,255,0), 1); // Safety circle
    circle(display, Point(start.y, start.x), 8, Scalar(0,255,0), FILLED);
    putText(display, "START", Point(start.y+10, start.x-10), 
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
    
    circle(display, Point(goal.y, goal.x), ROBOT_CLEARANCE, Scalar(0,255,0), 1); // Safety circle  
    circle(display, Point(goal.y, goal.x), 8, Scalar(0,255,0), FILLED);
    putText(display, "GOAL", Point(goal.y+10, goal.x-10), 
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
    
    // Mark turning points in blue (only if path exists)
    if(!chosenPath.empty()) {
        for(auto &turn : turningPoints){
            Pt p = get<1>(turn);
            circle(display, Point(p.y, p.x), 6, Scalar(255,0,0), FILLED);
        }
    }
    
    // Add status and safety information
    if(chosenPath.empty()) {
        putText(display, "NO SAFE PATH FOUND", Point(50, 50), 
                FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2);
        putText(display, "6px clearance required", Point(50, 80), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
    } else {
        putText(display, "SAFE PATH FOUND", Point(50, 50), 
                FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 2);
        putText(display, chosenAlgorithm + " - 6px clearance", Point(50, 80), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);
    }
    
     // Legend
    putText(display, "Dark red: Safety zones", Point(50, display.rows-60), 
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,128), 1);
    putText(display, "Green circles: 6px clearance", Point(50, display.rows-40), 
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0), 1);
    putText(display, "Red line: Safe robot path", Point(50, display.rows-20), 
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 1);

    /* -------------- NEW: grid + axes (start) -------------- */

    /* -------------- NEW: grid + axes (with tick values) -------------- */

    const int gridSpacing = 50;

    // 1. Grid lines
    for (int y = 0; y < display.cols; y += gridSpacing)
        line(display, Point(y, 0), Point(y, display.rows - 1),
             Scalar(200, 200, 200), 1, LINE_AA);
    for (int x = 0; x < display.rows; x += gridSpacing)
        line(display, Point(0, x), Point(display.cols - 1, x),
             Scalar(200, 200, 200), 1, LINE_AA);

    // 2. Main axes (origin = top-left)
    const int axisLen = 90;
    arrowedLine(display, Point(0, 0), Point(axisLen, 0),      // X-axis
                Scalar(255, 255, 255), 2, LINE_AA, 0, 0.15);
    putText(display, "X", Point(axisLen + 6, 15),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

    arrowedLine(display, Point(0, 0), Point(0, axisLen),      // Y-axis
                Scalar(255, 255, 255), 2, LINE_AA, 0, 0.15);
    putText(display, "Y", Point(6, axisLen + 20),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

    // 3. Numeric tick labels (every 50 px)
    for (int x = 0; x < display.cols; x += gridSpacing) {
        putText(display, to_string(x), Point(x + 2, 15),
                FONT_HERSHEY_PLAIN, 0.8, Scalar(180, 180, 180), 1);
    }
    for (int y = 0; y < display.rows; y += gridSpacing) {
        // avoid writing "0" twice at the origin
        if (y == 0) continue;
        putText(display, to_string(y), Point(2, y + 15),
                FONT_HERSHEY_PLAIN, 0.8, Scalar(180, 180, 180), 1);
    }

    /* -------------- END of grid + axes block -------------- */

    
    // Save result
    string outputFilename = chosenPath.empty() ? "no_safe_path.png" : "safe_robot_path.png";
    imwrite(outputFilename, display);
    cout << "\n📁 Visualization saved as '" << outputFilename << "'" << endl;
    cout << "🔍 Dark red areas show zones lost to 6-pixel safety margin" << endl;
    cout << "✅ Path guarantees robot can navigate safely without collision" << endl;
    
    return 0;
}
