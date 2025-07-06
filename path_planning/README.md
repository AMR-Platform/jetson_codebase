# Route Planning in Occupancy Grid Maps

## Overview

This project explores route planning in robotic environments represented by occupancy grid maps, utilizing two prominent graph-based planning algorithms: **A\* Search** and **Probabilistic Road Maps (PRM)**. The project is structured in three main parts:

### Occupancy Grid Map

![alt text](https://github.com/nrjsbudhe/Path-Planning-using-Graph-based-Planning-Algorithms/blob/7aa40120af20c584bc75d273450dfc31c8c919de/occupancy_map.png)

1. **Implementation of A\* Search:**
   - A general version of the A\* search algorithm is implemented, which is abstracted from the specific graph representation. This allows it to be applied to both occupancy grids (considered as 8-connected graphs) and standard graphs, such as those used in probabilistic roadmaps.
   - The algorithm is designed to find the shortest path from a start node to a goal node within a graph.

2. **Route Planning in Occupancy Grids with A\* Search:**
   - This section applies the A\* search algorithm to an occupancy grid map, treated as an 8-connected graph. In this representation, each vertex corresponds to a cell in the grid, identified by its row and column coordinates.
   - The algorithm navigates through the grid, finding the optimal path from a start cell to a goal cell while avoiding obstacles.

3. **Route Planning with Probabilistic Roadmaps:**
   - Voxelized grids, like occupancy maps, are efficient for modeling robot configuration spaces, but they become impractical for high-dimensional spaces due to their exponential memory requirements. In such cases, sampling-based planners, like PRMs, offer a more feasible alternative.
   - This part of the project implements a sampling-based planner to construct a probabilistic roadmap (PRM) within the occupancy grid. The PRM is incrementally built by sampling new vertices from the configuration space and connecting them to nearby vertices using a local planner.
   - Ultimately A\* Algorithm is implemented on the constructed PRM to find a path from start to goal.

## Algorithms

### A\* Search
- **A\* Search** is a graph traversal and path search algorithm, which finds the shortest path between a start and goal node. It uses a heuristic to guide its search, combining the cost to reach a node and an estimate of the cost from that node to the goal.

### Probabilistic Roadmaps (PRM)
- **PRM** is a sampling-based path planning algorithm that approximates the configuration space by constructing a graph. Vertices in this graph are sampled points in the configuration space, and edges represent feasible paths between these points, determined by a local planner.

## Dependencies

- Python 3.x
- Libraries: NumPy, Matplotlib, heapq, networkx (for graph handling)

## Generated Paths

### A\* Star Search

![alt text](https://github.com/nrjsbudhe/Path-Planning-using-Graph-based-Planning-Algorithms/blob/main/Planned_Path_A_Star.png)

### PRM Based A\* Implementation

![alt text](https://github.com/nrjsbudhe/Path-Planning-using-Graph-based-Planning-Algorithms/blob/main/Planned_Path_PRM.png)

