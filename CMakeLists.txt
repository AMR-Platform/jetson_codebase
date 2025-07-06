cmake_minimum_required(VERSION 3.10)
project(slam_from_scratch LANGUAGES CXX)

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

# ---------- Executable ----------
add_executable(slam_exec ${USER_SRC} ${VENDOR_SRC})

target_include_directories(slam_exec
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_exec
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        pthread
)
