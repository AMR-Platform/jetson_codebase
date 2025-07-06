// PathFinder.cpp
// A* and PRM path planning with turning point detection using OpenCV

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <cmath>
#include <random>

using namespace cv;
using namespace std;

struct Pt {
    int x, y;
    bool operator<(Pt const &o) const {
        return x<o.x || (x==o.x && y<o.y);
    }
    bool operator==(Pt const &o) const {
        return x==o.x && y==o.y;
    }
};

// hash for unordered_map if needed
struct PtHash { size_t operator()(Pt const &p) const { return (p.x<<16) ^ p.y; } };

// Euclidean distance
static double dist(Pt a, Pt b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// Find nearest free pixel if start/goal blocked
Pt findNearestFree(Mat const &grid, Pt pt, int max_radius=20) {
    if (grid.at<uchar>(pt.x, pt.y) == 255)
        return pt;
    int rows=grid.rows, cols=grid.cols;
    for(int r=1;r<=max_radius;++r){
        for(int dx=-r; dx<=r; ++dx){
            for(int dy=-r; dy<=r; ++dy){
                int x = pt.x+dx, y=pt.y+dy;
                if(x>=0&&x<rows&&y>=0&&y<cols && grid.at<uchar>(x,y)==255)
                    return Pt{x,y};
            }
        }
    }
    return pt;
}

// A* planner
class AStar {
public:
    AStar(Mat const &g): grid(g) {}

    vector<Pt> search(Pt s, Pt goal) {
        s = findNearestFree(grid,s);
        goal = findNearestFree(grid,goal);
        map<Pt,double> gScore, fScore;
        map<Pt,Pt> pred;
        auto cmp = [&](Pt a, Pt b){ return fScore[a] > fScore[b]; };
        priority_queue<Pt, vector<Pt>, decltype(cmp)> open(cmp);

        gScore[s]=0;
        fScore[s]=dist(s,goal);
        open.push(s);

        set<Pt> inOpen;
        inOpen.insert(s);

        vector<Pt> dirs={{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};

        while(!open.empty()){
            Pt u=open.top(); open.pop(); inOpen.erase(u);
            if(u==goal) return reconstruct(pred,s,goal);
            for(auto &d:dirs){
                Pt v{u.x+d.x,u.y+d.y};
                if(v.x<0||v.x>=grid.rows||v.y<0||v.y>=grid.cols)
                    continue;
                if(grid.at<uchar>(v.x,v.y)!=255) continue;
                double tentative = gScore[u] + dist(u,v);
                if(!gScore.count(v) || tentative < gScore[v]){
                    pred[v]=u;
                    gScore[v]=tentative;
                    fScore[v]=tentative + dist(v,goal);
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
    vector<Pt> reconstruct(map<Pt,Pt> &pred, Pt s, Pt g) {
        vector<Pt> path;
        Pt cur=g;
        while(!(cur==s)){
            path.push_back(cur);
            cur = pred[cur];
        }
        path.push_back(s);
        reverse(path.begin(), path.end());
        return path;
    }
};

// Simple PRM planner
class PRM {
public:
    PRM(Mat const &g): grid(g), gen(random_device{}()) {}

    void build(int N, double dmax) {
        uniform_int_distribution<> rx(0,grid.rows-1), ry(0,grid.cols-1);
        while(nodes.size()<N){
            Pt p{rx(gen), ry(gen)};
            if(grid.at<uchar>(p.x,p.y)==255 && !conn.count(p)){
                conn[p]={};
                nodes.push_back(p);
                for(auto &u: nodes){
                    if(!(u==p) && dist(u,p)<=dmax && connectable(u,p)){
                        conn[u].push_back({p,dist(u,p)});
                        conn[p].push_back({u,dist(u,p)});
                    }
                }
            }
        }
    }

    vector<Pt> findPath(Pt s, Pt goal, double dmax) {
        s = findNearestFree(grid,s);
        goal = findNearestFree(grid,goal);
        conn[s]={}; conn[goal]={};
        // connect s and goal
        for(auto &u: nodes){
            if(dist(u,s)<=dmax && connectable(u,s)) conn[u].push_back({s,dist(u,s)}), conn[s].push_back({u,dist(u,s)});
            if(dist(u,goal)<=dmax && connectable(u,goal)) conn[u].push_back({goal,dist(u,goal)}), conn[goal].push_back({u,dist(u,goal)});
        }
        // A* on roadmap
        map<Pt,double> gScore,fScore;
        map<Pt,Pt> pred;
        auto cmp=[&](Pt a, Pt b){ return fScore[a]>fScore[b]; };
        priority_queue<Pt, vector<Pt>, decltype(cmp)> open(cmp);
        gScore[s]=0; fScore[s]=dist(s,goal);
        open.push(s);
        set<Pt> inOpen; inOpen.insert(s);
        while(!open.empty()){
            Pt u=open.top(); open.pop(); inOpen.erase(u);
            if(u==goal) return reconstruct(pred,s,goal);
            for(auto &edge:conn[u]){
                Pt v=edge.first; double w=edge.second;
                double tentative = gScore[u]+w;
                if(!gScore.count(v)||tentative<gScore[v]){
                    pred[v]=u; gScore[v]=tentative;
                    fScore[v]=tentative+dist(v,goal);
                    if(!inOpen.count(v)) open.push(v), inOpen.insert(v);
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
        for(int i=0;i<=steps;++i){
            double t=(double)i/steps;
            int x=round(a.x+(b.x-a.x)*t), y=round(a.y+(b.y-a.y)*t);
            if(grid.at<uchar>(x,y)==0) return false;
        }
        return true;
    }

    vector<Pt> reconstruct(map<Pt,Pt> &pred, Pt s, Pt g){
        vector<Pt> path; Pt cur=g;
        while(!(cur==s)) { path.push_back(cur); cur=pred[cur]; }
        path.push_back(s); reverse(path.begin(),path.end());return path;
    }
};

int main(){
    // Load and preprocess image
    Mat img = imread("bxKHP.png", IMREAD_GRAYSCALE);
    resize(img, img, Size(600,600), 0,0, INTER_AREA);
    Mat mask = (img<128);
    mask *= 255;

    // Inflate obstacles
    Mat inflated = mask.clone();
    int r=6;
    for(int x=0;x<mask.rows;++x) for(int y=0;y<mask.cols;++y) if(mask.at<uchar>(x,y)==0){
        int x0=max(0,x-r), x1=min(mask.rows-1,x+r);
        int y0=max(0,y-r), y1=min(mask.cols-1,y+r);
        inflated(Range(x0,x1),Range(y0,y1)).setTo(0);
    }

    Pt start{595,140}, goal{208,386};
    // Run planners
    AStar astar(inflated);
    auto path1 = astar.search(start,goal);
    PRM prm(inflated);
    prm.build(2500,75);
    auto path2 = prm.findPath(start,goal,75);
    double l1=1e9,l2=1e9;
    for(int i=1;i<(int)path1.size();++i) l1+= dist(path1[i-1],path1[i]);
    for(int i=1;i<(int)path2.size();++i) l2+= dist(path2[i-1],path2[i]);
    auto chosen = l1<=l2?path1:path2;

    cout<<"Chosen: "<<(l1<=l2?"A*":"PRM")<<"\n";
    // Detect turns
    vector<tuple<int,Pt,double>> turns;
    for(int i=1;i+1<(int)chosen.size();++i){
        Vec2d v1(chosen[i].x-chosen[i-1].x, chosen[i].y-chosen[i-1].y);
        Vec2d v2(chosen[i+1].x-chosen[i].x, chosen[i+1].y-chosen[i].y);
        double n1=norm(v1), n2=norm(v2);
        if(n1<1e-6||n2<1e-6) continue;
        double angle=degrees(acos(clamp((v1.dot(v2))/(n1*n2), -1.0,1.0)));
        if(angle>=10) turns.emplace_back(i,chosen[i],angle);
    }
    // Include start/end
    vector<pair<Pt,double>> keyPts;
    keyPts.emplace_back(start,0);
    for(auto &t:turns){ keyPts.emplace_back(get<1>(t),get<2>(t)); }
    keyPts.emplace_back(goal,0);

    // Distances between keyPts
    cout<<"KeyPoint distances:\n";
    for(int i=0;i+1<(int)keyPts.size();++i){
        Pt p1=keyPts[i].first, p2=keyPts[i+1].first;
        // sum along chosen path between indices
        int idx1 = (i==0?0:get<0>(turns[i-1]));
        int idx2 = (i+1==(int)keyPts.size()-1?chosen.size()-1:get<0>(turns[i]));
        double d=0;
        for(int j=idx1+1;j<=idx2;++j) d+=dist(chosen[j-1],chosen[j]);
        cout<<"("<<p1.x<<","<<p1.y<<")->("<<p2.x<<","<<p2.y<<"): "<<d<<"\n";
    }
    return 0;
}

// End of PathFinder.cpp
