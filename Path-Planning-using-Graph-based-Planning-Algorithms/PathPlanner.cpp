// PathFinder.cpp
// Run A* and PRM, pick the shorter, then detect turns & visualize

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <cmath>
#include <random>
#include <algorithm>   // for std::clamp

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

// Snap to nearest free cell if blocked
Pt findNearestFree(const Mat &grid, Pt pt, int max_r=20){
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
        map<Pt,Pt>   pred;
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
        while((int)nodes.size()<N){
            Pt p{rx(gen),ry(gen)};
            if(grid.at<uchar>(p.x,p.y)==255 && !conn.count(p)){
                conn[p]={}; nodes.push_back(p);
                for(auto &u:nodes){
                    if(!(u==p) && dist(u,p)<=dmax && connectable(u,p)){
                        conn[u].push_back({p,dist(u,p)});
                        conn[p].push_back({u,dist(u,p)});
                    }
                }
            }
        }
    }
    vector<Pt> findPath(Pt s, Pt goal, double dmax){
        s = findNearestFree(grid,s);
        goal = findNearestFree(grid,goal);
        conn[s]={}; conn[goal]={};
        for(auto &u:nodes){
            if(dist(u,s)<=dmax && connectable(u,s))
                conn[u].push_back({s,dist(u,s)}),
                conn[s].push_back({u,dist(u,s)});
            if(dist(u,goal)<=dmax && connectable(u,goal))
                conn[u].push_back({goal,dist(u,goal)}),
                conn[goal].push_back({u,dist(u,goal)});
        }
        map<Pt,double> gScore,fScore;
        map<Pt,Pt>   pred;
        auto cmp=[&](Pt a,Pt b){ return fScore[a]>fScore[b]; };
        priority_queue<Pt,vector<Pt>,decltype(cmp)> open(cmp);
        set<Pt> inOpen;

        gScore[s]=0; fScore[s]=dist(s,goal);
        open.push(s); inOpen.insert(s);

        while(!open.empty()){
            Pt u=open.top(); open.pop();
            inOpen.erase(u);
            if(u==goal) return reconstruct(pred,s,goal);

            for(auto &e:conn[u]){
                Pt v=e.first; double w=e.second;
                double t=gScore[u]+w;
                if(!gScore.count(v) || t<gScore[v]){
                    pred[v]=u; gScore[v]=t;
                    fScore[v]=t+dist(v,goal);
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
        int steps=max(abs(b.x-a.x), abs(b.y-a.y));
        for(int i=0;i<=steps;++i){
            double t=(double)i/steps;
            int x=round(a.x+(b.x-a.x)*t),
                y=round(a.y+(b.y-a.y)*t);
            if(grid.at<uchar>(x,y)==0) return false;
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

int main(){
    // Load & binarize
    Mat img=imread("bxKHP.png",IMREAD_GRAYSCALE);
    if(img.empty()){ cerr<<"Can't load bxKHP.png\n"; return 1; }
    resize(img,img,Size(600,600),0,0,INTER_AREA);
    Mat mask=(img<128); mask*=255;

    // Inflate
    Mat inflated=mask.clone();
    int R=6;
    for(int x=0;x<mask.rows;++x)
      for(int y=0;y<mask.cols;++y)
        if(mask.at<uchar>(x,y)==0){
          int x0=max(0,x-R),x1=min(mask.rows-1,x+R),
              y0=max(0,y-R),y1=min(mask.cols-1,y+R);
          inflated(Range(x0,x1),Range(y0,y1)).setTo(0);
        }

    Pt start{595,140}, goal{208,386};
    // Run both
    auto aPath = AStar(inflated).search(start,goal);
    PRM prm(inflated); prm.build(2500,75);
    auto pPath = prm.findPath(start,goal,75);

    // Compute lengths
    auto pathLen=[&](const vector<Pt> &P){
      double L=0;
      for(int i=1;i<(int)P.size();++i) L+=dist(P[i-1],P[i]);
      return L;
    };
    double L1 = aPath.empty()?1e9:pathLen(aPath);
    double L2 = pPath.empty()?1e9:pathLen(pPath);

    // Select
    vector<Pt> chosen = (L1<=L2 ? aPath : pPath);
    cout<<"Chosen: "<<(L1<=L2?"A*":"PRM")<<"\n";

    // Turn detection
    static constexpr double RAD2DEG=180.0/CV_PI;
    vector<tuple<int,Pt,double>> turns;
    for(int i=1;i+1<(int)chosen.size();++i){
        Vec2d v1(chosen[i].x-chosen[i-1].x,chosen[i].y-chosen[i-1].y),
               v2(chosen[i+1].x-chosen[i].x,chosen[i+1].y-chosen[i].y);
        double n1=norm(v1), n2=norm(v2);
        if(n1<1e-6||n2<1e-6) continue;
        double cosang=clamp(v1.dot(v2)/(n1*n2),-1.0,1.0),
               ang=acos(cosang)*RAD2DEG;
        if(ang>=10.0) turns.emplace_back(i,chosen[i],ang);
    }
    cout<<"Turning points (idx,x,y,angle°):\n";
    for(auto &t:turns){
        int idx; Pt p; double a;
        tie(idx,p,a)=t;
        cout<<idx<<":("<<p.x<<","<<p.y<<")="<<a<<"°\n";
    }

    // Key-point distances
    vector<pair<Pt,double>> kpts;
    kpts.emplace_back(start,0);
    for(auto &t:turns) kpts.emplace_back(get<1>(t),get<2>(t));
    kpts.emplace_back(goal,0);
    cout<<"KeyPoint distances:\n";
    for(int i=0;i+1<(int)kpts.size();++i){
        Pt p1=kpts[i].first,p2=kpts[i+1].first;
        int i1=(i==0?0:get<0>(turns[i-1])),
            i2=(i+1==(int)kpts.size()-1?(int)chosen.size()-1:get<0>(turns[i]));
        double d=0;
        for(int j=i1+1;j<=i2;++j) d+=dist(chosen[j-1],chosen[j]);
        cout<<"("<<p1.x<<","<<p1.y<<")->("<<p2.x<<","<<p2.y<<")="<<d<<"\n";
    }

    // Visualization
    Mat disp; cvtColor(mask,disp,COLOR_GRAY2BGR);
    vector<Point> poly;
    for(auto &pt:chosen) poly.emplace_back(pt.y,pt.x);
    polylines(disp,poly,false,Scalar(0,0,255),2);
    circle(disp,Point(start.y,start.x),5,Scalar(0,255,0),FILLED);
    circle(disp,Point(goal.y,goal.x),5,Scalar(0,255,0),FILLED);
    for(auto &t:turns){
        Pt p=get<1>(t);
        circle(disp,Point(p.y,p.x),5,Scalar(255,0,0),FILLED);
    }
    imwrite("planned_path.png",disp);
    cout<<"Wrote planned_path.png\n";

    return 0;
}
