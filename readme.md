# Jetson Codebase Scratch — From-Scratch SLAM on AMR 🚗📍

This project implements a **modular SLAM (Simultaneous Localization and Mapping)** system from scratch in **C++** for an **Autonomous Mobile Robot (AMR)**, without relying on frameworks like ROS. The system runs directly on a **Jetson platform** and supports modular pose estimation using:
- **IMU + Encoder fusion via EKF (Extended Kalman Filter)**
- **LiDAR scan processing** (for visual inspection and later use)

---

## How to run 

g++ main.cpp lidar_handler.cpp -o lidar_receiver -pthread
./lidar_receiver



cd Jetson_codebase_scratch
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./slam_exec

## 📁 Project Structure

Jetson_codebase_scratch/
├── include/
│ ├── kalman_filter.hpp # EKF for pose estimation
│ ├── occupancy_grid.hpp # Occupancy grid data structure
│ ├── lidar_handler.hpp # LakiBeam 1S UDP packet parsing
│ ├── serial_parser.hpp # Parses IMU + Encoder packets via /dev/ttyACM0
│ ├── udp_receiver.hpp # Remote control packet reception (optional)
│ └── utils.hpp # Common data structs (Pose2D, etc.)
├── src/
│ ├── kalman_filter.cpp
│ ├── occupancy_grid.cpp
│ ├── lidar_handler.cpp
│ ├── serial_parser.cpp
│ ├── udp_receiver.cpp
│ └── main.cpp # Entry point for SLAM loop or debug modes
├── build/ # CMake build folder
├── CMakeLists.txt
└── README.md

yaml
Copy
Edit

---

## 🛠️ Build Instructions

### 📦 Dependencies
- C++17 compiler
- CMake ≥ 3.10
- [Eigen3](https://eigen.tuxfamily.org/) (for matrix math)

Install dependencies on Jetson:
```bash
sudo apt update
sudo apt install cmake libeigen3-dev g++
🔨 Build
bash
Copy
Edit
cd Jetson_codebase_scratch
mkdir -p build && cd build
cmake ..
make -j$(nproc)
▶️ Run
bash
Copy
Edit
./slam_exec
🔧 Modules Description
Module	Purpose
serial_parser	Parses serial data from /dev/ttyACM0 containing IMU and encoder packets. Shared globally.
kalman_filter	Implements EKF to fuse encoder and yaw data into pose (x, y, θ).
lidar_handler	Receives and parses UDP packets from LakiBeam 1S LiDAR.
occupancy_grid	Maintains a 2D grid (for future SLAM mapping extensions).
udp_receiver	Handles remote commands or parameter updates via UDP (optional).
utils	Includes basic structs like Pose2D, angle normalization.

🧪 Debug Tools
Use the included main.cpp for:

Testing LiDAR parsing and angle/distance display

Debugging EKF with hardcoded velocity and yaw inputs

Printing estimated pose after each step

You can redirect output or log to CSV as needed for plotting.

🔌 Hardware Interfaces
Serial input: IMU + Encoder packet via /dev/ttyACM0 (USB)

Ethernet input: LiDAR packets from LakiBeam 1S over UDP

UDP optional: Remote control packets (via network)

📈 Future Work
Integrate occupancy grid with LiDAR updates

Loop closure and drift correction

Web-based telemetry dashboard

Map saving and visualization (PNG / CSV)

👨‍💻 Team
This project was developed by the AMR Platform Team
For inquiries, please contact the maintainers listed in the GitHub organization.

📜 License
This project is open-source under the MIT License.

yaml
Copy
Edit

---

Let me know if you'd like a second version for external documentation or contribution guidelines
