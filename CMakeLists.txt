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

# Remove test files from main executable
list(REMOVE_ITEM USER_SRC ${PROJECT_SOURCE_DIR}/src/localization/test_ekf.cpp)

# ---------- Executables ----------
# Main executable (includes RobotLocalization functionality and LakiBeam files)
add_executable(main_exe ${USER_SRC})

# EKF test executable
add_executable(test_ekf
    src/localization/test_ekf.cpp
    src/localization/SensorFusion.cpp
)

# EKF synthetic data test executable
add_executable(test_ekf_synthetic
    src/test_ekf_synthetic.cpp
    src/localization/SensorFusion.cpp
)

# EKF simple test executable  
add_executable(test_ekf_simple
    src/test_ekf_simple.cpp
    src/localization/SensorFusion.cpp
)

target_include_directories(main_exe
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/core
        ${OpenCV_INCLUDE_DIRS}
)


target_include_directories(test_ekf
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/core
)

target_include_directories(test_ekf_synthetic
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/core
)

target_include_directories(test_ekf_simple
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
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
)

target_link_libraries(test_ekf_synthetic
    PUBLIC
        Eigen3::Eigen
)

target_link_libraries(test_ekf_simple
    PUBLIC
        Eigen3::Eigen
)


