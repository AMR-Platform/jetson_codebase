cmake_minimum_required(VERSION 3.10)
project(jetson_codebase LANGUAGES CXX)

# ---------- Global Settings ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------- Find Dependencies ----------
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)                       # OpenCV 4.1.1
find_package(Boost REQUIRED COMPONENTS system thread filesystem)  # Added filesystem

# ---------- Your Project Code ----------
# Recursively find all source files in subdirectories
file(GLOB_RECURSE USER_SRC 
    src/localization/*.cpp
    src/lidar/*.cpp
    src/communication/*.cpp
)

# Add main.cpp separately
list(APPEND USER_SRC src/main.cpp)

# ---------- Executables ----------
# Main executable (includes RobotLocalization functionality and LakiBeam files)
add_executable(main_exe ${USER_SRC})

target_include_directories(main_exe
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/core
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(main_exe
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        pthread
)
