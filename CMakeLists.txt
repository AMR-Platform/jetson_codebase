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
    src/mapping/*.cpp
    src/lidar/*.cpp
    src/communication/*.cpp
)

# Add main.cpp separately
list(APPEND USER_SRC src/main.cpp)

# Remove test files from main executable
list(REMOVE_ITEM USER_SRC ${PROJECT_SOURCE_DIR}/src/localization/test_ekf.cpp)
list(REMOVE_ITEM USER_SRC ${PROJECT_SOURCE_DIR}/src/localization/test_ekf_real_robot.cpp)

# ---------- Executables ----------
# Main executable (includes RobotLocalization functionality and LakiBeam files)
add_executable(main_exe ${USER_SRC})

# EKF test executable
add_executable(test_ekf
    src/localization/test_ekf.cpp
    src/localization/SensorFusion.cpp
)

# Real robot EKF test executable
add_executable(test_ekf_real_robot
    src/localization/test_ekf_real_robot.cpp
    src/localization/SensorFusion.cpp
    src/communication/serial_com.cpp
)

target_include_directories(main_exe
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/mapping
        ${PROJECT_SOURCE_DIR}/include/core
        ${OpenCV_INCLUDE_DIRS}
)


target_include_directories(test_ekf
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/mapping
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/core
)

target_include_directories(test_ekf_real_robot
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/mapping
        ${PROJECT_SOURCE_DIR}/include/core
)

target_link_libraries(main_exe
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        pthread
)


target_link_libraries(test_ekf
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

target_link_libraries(test_ekf_real_robot
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        pthread
)

add_executable(test_mapping
    src/mapping/occupancy_grid.cpp
    src/mapping/test_mapping.cpp                     # You could create this for testing
)

target_include_directories(test_mapping
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/mapping
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(test_mapping
    PUBLIC
        ${OpenCV_LIBS}
        pthread
)



