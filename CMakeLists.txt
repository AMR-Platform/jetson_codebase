cmake_minimum_required(VERSION 3.10)
project(slam_from_scratch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories
include_directories(include)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Executable
add_executable(slam_exec ${SOURCES})

# Link libraries (Eigen is header-only, so no linking needed)
