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

# ---------- Include LakiBeam SDK ----------
set(LAKI_ROOT ${PROJECT_SOURCE_DIR}/third_party/lakibeam)
include_directories(${LAKI_ROOT}/include)
file(GLOB VENDOR_SRC ${LAKI_ROOT}/src/*.cpp)

# ---------- Your Project Code ----------
file(GLOB_RECURSE USER_SRC src/*.cpp)

# Remove main_ekf.cpp and test_ekf.cpp from main_exe sources for separate compilation
list(REMOVE_ITEM USER_SRC ${PROJECT_SOURCE_DIR}/src/main_ekf.cpp)
list(REMOVE_ITEM USER_SRC ${PROJECT_SOURCE_DIR}/src/test_ekf.cpp)

# ---------- Executables ----------
# Original main executable (without EKF integration)
add_executable(main_exe ${USER_SRC} ${VENDOR_SRC})

# EKF-enabled executable
add_executable(robot_ekf 
    src/main_ekf.cpp
    src/SensorFusion.cpp
    src/serial_com.cpp
    src/lidar_handler.cpp
    src/utils.cpp
    ${VENDOR_SRC}
)

# EKF test executable
add_executable(test_ekf
    src/test_ekf.cpp
    src/SensorFusion.cpp
)



target_include_directories(main_exe
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(robot_ekf
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(test_ekf
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)



target_link_libraries(main_exe
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        pthread
)

target_link_libraries(robot_ekf
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


