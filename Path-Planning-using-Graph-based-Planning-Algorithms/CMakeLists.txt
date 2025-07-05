cmake_minimum_required(VERSION 3.5)
project(PathPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point CMake at your OpenCV installation
set(OpenCV_DIR "C:/Users/USER/Desktop/EDR/Path-Planning-using-Graph-based-Planning-Algorithms-main/opencv/build")

find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "Could not find OpenCV. Make sure OpenCV_DIR is set correctly.")
endif()

message(STATUS "Found OpenCV ${OpenCV_VERSION} in ${OpenCV_DIR}")
include_directories(${OpenCV_INCLUDE_DIRS})

# Source files
set(SOURCES
    PathPlanner.cpp
)

# Build executable
add_executable(PathPlanner ${SOURCES})
target_link_libraries(PathPlanner PRIVATE ${OpenCV_LIBS})
