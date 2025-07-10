cmake_minimum_required(VERSION 3.10)
project(SimpleSLAM)
set(CMAKE_CXX_STANDARD 17)

# Windows-specific settings
if(WIN32 AND MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-DM_PI=3.14159265358979323846)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2")
endif()

# Include directories
include_directories(include)

# Source files
set(SLAM_SOURCES
    src/SimpleSLAM.cpp
    src/OccupancyMap.cpp
    src/ScanMatcher.cpp
)

# Create library
add_library(simple_slam ${SLAM_SOURCES})

# Main executable
add_executable(simple_slam_test src/main.cpp)
target_link_libraries(simple_slam_test simple_slam)

# Install
install(TARGETS simple_slam simple_slam_test
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)