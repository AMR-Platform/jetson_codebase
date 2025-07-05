# ================================================================
#  jetson_codebase – SLAM-from-scratch with LakiBeam LiDAR
# ================================================================
cmake_minimum_required(VERSION 3.16)
project(slam_from_scratch LANGUAGES CXX)

# ---------- Global compile settings ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)      # VS Code / clangd use this

# ---------- Header-only deps ----------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------- Runtime / GUI deps ----------
find_package(OpenCV REQUIRED)              # imshow, imwrite
find_package(Boost REQUIRED COMPONENTS system thread)

# ---------- LakiBeam SDK (header + source) ----------
# Folder layout you already have:
#   third_party/lakibeam/include/  LakiBeamUDP.h
#   third_party/lakibeam/src/      LakiBeamUDP.cpp
set(LAKI_ROOT ${PROJECT_SOURCE_DIR}/third_party/lakibeam)
include_directories(${LAKI_ROOT}/include)

file(GLOB VENDOR_SRC CONFIGURE_DEPENDS
     ${LAKI_ROOT}/src/*.cpp)               # <- picks up LakiBeamUDP.cpp

# ---------- Your project sources ----------
file(GLOB_RECURSE USER_SRC CONFIGURE_DEPENDS src/*.cpp)

# ---------- Build target ----------
add_executable(slam_exec ${USER_SRC} ${VENDOR_SRC})

target_include_directories(slam_exec
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include      # your own public headers
        ${OpenCV_INCLUDE_DIRS} 
)

target_link_libraries(slam_exec
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        Boost::system
        Boost::thread
        pthread                            # Boost::thread & SDK expect it
)
