import numpy as np
import math
import matplotlib.pyplot as plt
import networkx as nx
import random as rn
from PIL import Image

# ── IMAGE LOADING & PREPROCESSING ─────────────────────────────────────────────
img = Image.open('bxKHP.png').convert('L')
try:
    resample_filter = Image.Resampling.LANCZOS
except AttributeError:
    resample_filter = Image.LANCZOS
img = img.resize((600, 600), resample_filter)
OG_array = np.array(img, dtype=np.uint8)
# Binarize so white=blocked (255), black=free (0)
mask = (OG_array < 128).astype(np.uint8) * 255

# ── INFLATE OBSTACLES BY 6 PIXELS ─────────────────────────────────────────────
inflation_radius = 6
inflated_mask = mask.copy()
obs = np.argwhere(mask == 0)
for x, y in obs:
    x_min = max(0, x - inflation_radius)
    x_max = min(mask.shape[0], x + inflation_radius + 1)
    y_min = max(0, y - inflation_radius)
    y_max = min(mask.shape[1], y + inflation_radius + 1)
    inflated_mask[x_min:x_max, y_min:y_max] = 0

# ── HELPERS ───────────────────────────────────────────────────────────────────
def find_nearest_free(grid, pt, max_radius=20):
    if grid[pt] == 255:
        return pt
    x0, y0 = pt
    rows, cols = grid.shape
    for r in range(1, max_radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                x, y = x0 + dx, y0 + dy
                if 0 <= x < rows and 0 <= y < cols and grid[x, y] == 255:
                    return (x, y)
    return pt

# ── A* SEARCH CLASS ────────────────────────────────────────────────────────────
class A_STAR:
    def __init__(self, grid):
        self.grid = grid
    def neighbors(self, v):
        offsets = [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
        return [(v[0]+dx, v[1]+dy) for dx,dy in offsets
                if 0 <= v[0]+dx < self.grid.shape[0]
                and 0 <= v[1]+dy < self.grid.shape[1]
                and self.grid[v[0]+dx, v[1]+dy] == 255]
    def cost(self, a, b): return math.dist(a, b)
    def heuristic(self, a, b): return math.dist(a, b)
    def reconstruct(self, pred, s, g):
        path=[g]
        while path[-1]!=s:
            path.append(pred[path[-1]])
        return path[::-1]
    def search(self, s, g):
        s, g = find_nearest_free(self.grid, s), find_nearest_free(self.grid, g)
        open_set={s:self.heuristic(s,g)}
        g_score={s:0}
        pred={}
        while open_set:
            current=min(open_set, key=open_set.get)
            open_set.pop(current)
            if current==g:
                return self.reconstruct(pred,s,g)
            for n in self.neighbors(current):
                t=g_score[current]+self.cost(current,n)
                if n not in g_score or t<g_score[n]:
                    g_score[n]=t; pred[n]=current
                    open_set[n]=t+self.heuristic(n,g)
        return []

# ── PROBABILISTIC ROADMAP (PRM) CLASS ────────────────────────────────────────
class PRM:
    def __init__(self, grid): self.grid=grid; self.G=nx.Graph()
    def sample(self):
        rows,cols=self.grid.shape
        while True:
            pt=(rn.randrange(rows), rn.randrange(cols))
            if self.grid[pt]==255 and pt not in self.G: return pt
    def connectable(self,v1,v2):
        steps=max(abs(v2[0]-v1[0]), abs(v2[1]-v1[1]))
        for i in range(steps+1):
            t=i/steps; x=int(round(v1[0]+(v2[0]-v1[0])*t)); y=int(round(v1[1]+(v2[1]-v1[1])*t))
            if self.grid[x,y]==0: return False
        return True
    def build(self,N,dmax):
        while self.G.number_of_nodes()<N:
            v=self.sample(); self.G.add_node(v)
            for u in list(self.G.nodes()):
                if u!=v and math.dist(u,v)<=dmax and self.connectable(u,v):
                    self.G.add_edge(u,v,weight=math.dist(u,v))
        return self.G
    def find_path(self,s,g,dmax):
        s,g=find_nearest_free(self.grid,s),find_nearest_free(self.grid,g)
        self.G.add_node(s); self.G.add_node(g)
        for v in list(self.G.nodes()):
            if v not in (s,g):
                if math.dist(v,s)<=dmax and self.connectable(v,s): self.G.add_edge(v,s,weight=math.dist(v,s))
                if math.dist(v,g)<=dmax and self.connectable(v,g): self.G.add_edge(v,g,weight=math.dist(v,g))
        try: return nx.astar_path(self.G,s,g,weight='weight')
        except nx.NetworkXNoPath: return []

# ── MAIN EXECUTION ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    start=(595,140); goal=(208,386)
    # run both planners
    astar=A_STAR(inflated_mask); path_astar=astar.search(start,goal)
    prm=PRM(inflated_mask); prm.build(N=2500,dmax=75)
    path_prm=prm.find_path(start,goal,dmax=75)
    # select shortest
    la=sum(math.dist(path_astar[i],path_astar[i-1]) for i in range(1,len(path_astar))) if path_astar else float('inf')
    lp=sum(math.dist(path_prm[i],path_prm[i-1]) for i in range(1,len(path_prm))) if path_prm else float('inf')
    chosen=path_astar if la<=lp else path_prm
    print("Chosen algorithm:","A*" if la<=lp else "PRM")

    # detect turning points and angles
    def detect_turns(path,angle_thresh=10):
        out=[]
        for i in range(1,len(path)-1):
            v1=np.subtract(path[i],path[i-1]); v2=np.subtract(path[i+1],path[i])
            n1,n2=np.linalg.norm(v1),np.linalg.norm(v2)
            if n1==0 or n2==0: continue
            cosang=np.clip(np.dot(v1,v2)/(n1*n2),-1,1)
            ang=math.degrees(math.acos(cosang))
            if ang>=angle_thresh: out.append((i,tuple(path[i]),ang))
        return out
    turns=detect_turns(chosen)
    print("Turning points (index,coord,angle_deg):")
    for idx,coord,ang in turns: print(idx,coord,f"{ang:.1f}")

        # include start and goal as turning points (with angle 0)
    key_points = [(0, start, 0.0)] + turns + [(len(chosen)-1, goal, 0.0)]

    # distances between successive key points
    dist_kp = []
    for k in range(len(key_points)-1):
        _, p1, _ = key_points[k]
        _, p2, _ = key_points[k+1]
        # compute path segment between indices
        idx1 = key_points[k][0]
        idx2 = key_points[k+1][0]
        d = sum(math.dist(chosen[j], chosen[j-1]) for j in range(idx1+1, idx2+1))
        dist_kp.append((p1, p2, d))

    print("Distances between key points (from, to, distance):")
    for p1, p2, d in dist_kp:
        print(f"{p1} -> {p2}: {d:.2f}")

    # optionally plot turning points and key segments
    plt.figure(figsize=(6,6))
    plt.imshow(mask, cmap='gray', origin='upper')
    if chosen:
        xs, ys = zip(*[(p[1], p[0]) for p in chosen])
        plt.plot(xs, ys, '-r')
    # plot key points
    for _, coord, _ in key_points:
        plt.scatter(coord[1], coord[0], c='blue', s=50)
    plt.scatter(start[1], start[0], c='green', s=50)
    plt.scatter(goal[1], goal[0], c='green', s=50)
    plt.title("Path & Key Points (incl. start/goal)")
    plt.axis('off')
    plt.show()

# end of script
    plt.figure(figsize=(6,6)); plt.imshow(mask,cmap='gray',origin='upper')
    if chosen:
        xs,ys=zip(*[(p[1],p[0]) for p in chosen]); plt.plot(xs,ys,'-r')
    for _,coord,_ in turns: plt.scatter(coord[1],coord[0],c='blue',s=50)
    plt.scatter(start[1],start[0],c='green',s=50); plt.scatter(goal[1],goal[0],c='green',s=50)
    plt.title("Path & Turning Points"); plt.axis('off'); plt.show()
