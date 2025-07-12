// Interactive PathPlanner.cpp
// Click to set start and goal points, real-time path planning with mouse control
// Left click: Set start point, Right click: Set goal point, ESC: Exit

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

// Global variables for interactive mode
Mat originalMask, inflatedMask, displayImage;
Pt currentStart{-1, -1}, currentGoal{-1, -1};
vector<Pt> currentPath;
bool startSet = false, goalSet = false;
string windowName = "Interactive Path Planner";
Point lastMousePos{-1, -1};
const int ROBOT_CLEARANCE = 6;  // Robot safety margin in pixels

// Forward declarations
class AStar;
class PRM;
vector<Pt> findBestPath(const Mat& grid, Pt start, Pt goal);
void updateDisplay();
void drawPath();

// Snap to nearest free cell if blocked
Pt findNearestFree(const Mat &grid, Pt pt, int max_r=20){
    if(pt.x < 0 || pt.x >= grid.rows || pt.y < 0 || pt.y >= grid.cols) return pt;
    if(grid.at<uchar>(pt.x,pt.y)==255) return pt;
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
        
        nodes.clear();
        conn.clear();
        
        int attempts = 0;
        int maxAttempts = N * 5;
        
        while((int)nodes.size() < N && attempts < maxAttempts){
            Pt p{rx(gen), ry(gen)};
            attempts++;
            
            if(grid.at<uchar>(p.x, p.y) == 255){
                bool tooClose = false;
                for(auto &existing : nodes){
                    if(dist(existing, p) < 3.0){
                        tooClose = true;
                        break;
                    }
                }
                
                if(!tooClose){
                    nodes.push_back(p);
                    conn[p] = {};
                    
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
    }
    
    vector<Pt> findPath(Pt s, Pt goal, double dmax){
        s = findNearestFree(grid, s);
        goal = findNearestFree(grid, goal);
        
        if(conn.count(s)) conn[s].clear();
        if(conn.count(goal)) conn[goal].clear();
        conn[s] = {};
        conn[goal] = {};
        
        double connectionRadius = dmax * 1.2;
        
        for(auto &u : nodes){
            if(dist(u, s) <= connectionRadius && connectable(u, s)){
                double weight = dist(u, s);
                conn[u].push_back({s, weight});
                conn[s].push_back({u, weight});
            }
            
            if(dist(u, goal) <= connectionRadius && connectable(u, goal)){
                double weight = dist(u, goal);
                conn[u].push_back({goal, weight});
                conn[goal].push_back({u, weight});
            }
        }
        
        if(dist(s, goal) <= connectionRadius && connectable(s, goal)){
            double weight = dist(s, goal);
            conn[s].push_back({goal, weight});
            conn[goal].push_back({s, weight});
        }
        
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
            
            if(x < 0 || x >= grid.rows || y < 0 || y >= grid.cols) return false;
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

// Global PRM instance for reuse
PRM* globalPRM = nullptr;

// Calculate path length
double calculatePathLength(const vector<Pt> &path){
    double length = 0;
    for(int i=1; i<(int)path.size(); ++i){
        length += dist(path[i-1], path[i]);
    }
    return length;
}

// Find best path using both algorithms
vector<Pt> findBestPath(const Mat& grid, Pt start, Pt goal) {
    // Run A*
    AStar astar(grid);
    auto astarPath = astar.search(start, goal);
    double astarLength = astarPath.empty() ? 1e9 : calculatePathLength(astarPath);
    
    // Run PRM (reuse existing roadmap for speed)
    auto prmPath = globalPRM->findPath(start, goal, 80);
    double prmLength = prmPath.empty() ? 1e9 : calculatePathLength(prmPath);
    
    // Choose better path
    if(astarLength <= prmLength && !astarPath.empty()) {
        cout << "Using A* (length: " << fixed << setprecision(1) << astarLength << ")" << endl;
        return astarPath;
    } else if(!prmPath.empty()) {
        cout << "Using PRM (length: " << fixed << setprecision(1) << prmLength << ")" << endl;
        return prmPath;
    }
    
    cout << "No path found!" << endl;
    return {};
}

// Update the display
void updateDisplay() {
    cvtColor(originalMask, displayImage, COLOR_GRAY2BGR);
    
    // Draw current path
    if(!currentPath.empty()) {
        vector<Point> pathPoints;
        for(auto &pt : currentPath) {
            pathPoints.emplace_back(pt.y, pt.x);
        }
        polylines(displayImage, pathPoints, false, Scalar(0, 0, 255), 2);
        
        // Show path length info
        double pathLength = calculatePathLength(currentPath);
        string lengthText = "Path Length: " + to_string((int)pathLength) + " pixels";
        putText(displayImage, lengthText, Point(10, 55), 
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
    }
    
    // Draw start point
    if(startSet) {
        circle(displayImage, Point(currentStart.y, currentStart.x), 8, Scalar(0, 255, 0), FILLED);
        circle(displayImage, Point(currentStart.y, currentStart.x), ROBOT_CLEARANCE, Scalar(0, 255, 0), 1);
        putText(displayImage, "START", Point(currentStart.y + 10, currentStart.x - 10), 
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
    }
    
    // Draw goal point
    if(goalSet) {
        circle(displayImage, Point(currentGoal.y, currentGoal.x), 8, Scalar(255, 0, 0), FILLED);
        circle(displayImage, Point(currentGoal.y, currentGoal.x), ROBOT_CLEARANCE, Scalar(255, 0, 0), 1);
        putText(displayImage, "GOAL", Point(currentGoal.y + 10, currentGoal.x - 10), 
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
    }
    
    // Show mouse hover preview with robot clearance
    if(lastMousePos.x >= 0 && lastMousePos.y >= 0) {
        Pt mouseGridPos = {lastMousePos.y, lastMousePos.x};
        bool validPos = (mouseGridPos.x >= 0 && mouseGridPos.x < inflatedMask.rows &&
                        mouseGridPos.y >= 0 && mouseGridPos.y < inflatedMask.cols &&
                        inflatedMask.at<uchar>(mouseGridPos.x, mouseGridPos.y) == 255);
        
        Scalar color = validPos ? Scalar(100, 255, 100) : Scalar(100, 100, 255);
        circle(displayImage, lastMousePos, ROBOT_CLEARANCE, color, 1);
        circle(displayImage, lastMousePos, 2, color, FILLED);
    }
    
    // Instructions with safety info
    putText(displayImage, "Left Click: Set Start | Right Click: Set Goal | 6px robot clearance", 
            Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
    
    imshow(windowName, displayImage);
}

// Mouse callback function
void onMouse(int event, int x, int y, int flags, void* userdata) {
    lastMousePos = Point(x, y);
    
    if (event == EVENT_LBUTTONDOWN) {
        // Left click - set start point
        Pt proposedStart = {y, x};  // Note: y,x because image coordinates
        Pt actualStart = findNearestFree(inflatedMask, proposedStart, 10);
        
        currentStart = actualStart;
        startSet = true;
        
        // Check if the position is valid (has 6-pixel clearance)
        bool validPosition = (inflatedMask.at<uchar>(actualStart.x, actualStart.y) == 255);
        if(validPosition) {
            cout << "Start set to: (" << currentStart.x << ", " << currentStart.y << ") ✓" << endl;
        } else {
            cout << "Start adjusted to: (" << currentStart.x << ", " << currentStart.y << ") - moved to safe area" << endl;
        }
        
        // Calculate path if both points are set
        if(startSet && goalSet) {
            currentPath = findBestPath(inflatedMask, currentStart, currentGoal);
        }
        updateDisplay();
    }
    else if (event == EVENT_RBUTTONDOWN) {
        // Right click - set goal point
        Pt proposedGoal = {y, x};  // Note: y,x because image coordinates
        Pt actualGoal = findNearestFree(inflatedMask, proposedGoal, 10);
        
        currentGoal = actualGoal;
        goalSet = true;
        
        // Check if the position is valid (has 6-pixel clearance)
        bool validPosition = (inflatedMask.at<uchar>(actualGoal.x, actualGoal.y) == 255);
        if(validPosition) {
            cout << "Goal set to: (" << currentGoal.x << ", " << currentGoal.y << ") ✓" << endl;
        } else {
            cout << "Goal adjusted to: (" << currentGoal.x << ", " << currentGoal.y << ") - moved to safe area" << endl;
        }
        
        // Calculate path if both points are set
        if(startSet && goalSet) {
            currentPath = findBestPath(inflatedMask, currentStart, currentGoal);
        }
        updateDisplay();
    }
    else if (event == EVENT_MOUSEMOVE) {
        // Show preview of where robot would be placed
        updateDisplay();
    }
}

int main() {
    // Load image
    Mat img = imread("new.png", IMREAD_GRAYSCALE);
    if(img.empty()) { 
        cerr << "Can't load new.png\n"; 
        return 1; 
    }
    
    // Resize if needed
    if(img.rows != 600 || img.cols != 600) {
        cout << "Original image size: " << img.cols << "x" << img.rows << endl;
        resize(img, img, Size(600, 600), 0, 0, INTER_AREA);
        cout << "Resized to: 600x600" << endl;
    }
    
    // Binarize image: white areas = free space (255), everything else = obstacles (0)
    threshold(img, originalMask, 200, 255, THRESH_BINARY);
    
    // Show image statistics
    int total_pixels = originalMask.rows * originalMask.cols;
    int free_pixels = countNonZero(originalMask);
    cout << "White areas (free space): " << free_pixels << " (" 
         << fixed << setprecision(1) << (100.0 * free_pixels / total_pixels) << "%)" << endl;
    
    // Inflate obstacles for robot safety (6-pixel margin)
    inflatedMask = originalMask.clone();
    int inflation_radius = 6;  // Robot requires 6-pixel clearance from barriers
    cout << "Inflating obstacles by " << inflation_radius << " pixels for robot safety..." << endl;
    
    for(int x=0; x<originalMask.rows; ++x){
        for(int y=0; y<originalMask.cols; ++y){
            if(originalMask.at<uchar>(x,y) == 0){  // If obstacle pixel
                int x0 = max(0, x-inflation_radius);
                int x1 = min(originalMask.rows, x+inflation_radius+1);
                int y0 = max(0, y-inflation_radius);
                int y1 = min(originalMask.cols, y+inflation_radius+1);
                inflatedMask(Range(x0,x1), Range(y0,y1)).setTo(0);  // Expand obstacle area
            }
        }
    }
    
    int inflated_free = countNonZero(inflatedMask);
    cout << "After 6-pixel inflation - Free space: " << inflated_free << " (" 
         << fixed << setprecision(1) << (100.0 * inflated_free / total_pixels) << "%)" << endl;
    cout << "Robot will maintain 6-pixel clearance from all barriers" << endl;
    
    // Build PRM roadmap once (for reuse)
    cout << "Building PRM roadmap..." << endl;
    globalPRM = new PRM(inflatedMask);
    globalPRM->build(3000, 80);
    cout << "PRM roadmap built!" << endl;
    
    // Create window and set mouse callback
    namedWindow(windowName, WINDOW_AUTOSIZE);
    setMouseCallback(windowName, onMouse, nullptr);
    
    // Initial display
    updateDisplay();
    
    cout << "\n=== INTERACTIVE MODE ===" << endl;
    cout << "Left click to set START point" << endl;
    cout << "Right click to set GOAL point" << endl;
    cout << "Press ESC to exit" << endl;
    
    // Main loop
    while(true) {
        int key = waitKey(30) & 0xFF;
        if(key == 27) break;  // ESC key
    }
    
    // Cleanup
    delete globalPRM;
    destroyAllWindows();
    
    return 0;
}
