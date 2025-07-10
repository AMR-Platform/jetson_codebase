cmake_minimum_required(VERSION 3.10)
project(jetson_codebase LANGUAGES CXX)

# ---------- Global Settings -----------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------- Dependencies ---------------------------------------------------
find_package(Eigen3  REQUIRED)
find_package(OpenCV 4 REQUIRED)                       # 4.x recommended
find_package(Boost   REQUIRED COMPONENTS system thread filesystem)
find_package(Threads REQUIRED)

# ---------- Source Files ---------------------------------------------------
# Core, localization, lidar, communication
file(GLOB_RECURSE USER_SRC
    src/localization/*.cpp
    src/lidar/*.cpp
    src/communication/*.cpp
    src/mapping/*.cpp           # <─ NEW: occupancy_grid.cpp lives here
)

# Add the top-level main.cpp
list(APPEND USER_SRC src/main.cpp)

# ---------- Executable -----------------------------------------------------
add_executable(main_exe ${USER_SRC})

target_include_directories(main_exe
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/core
        ${PROJECT_SOURCE_DIR}/include/localization
        ${PROJECT_SOURCE_DIR}/include/lidar
        ${PROJECT_SOURCE_DIR}/include/communication
        ${PROJECT_SOURCE_DIR}/include/mapping   # <─ NEW: occupancy_grid.hpp
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(main_exe
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        Boost::system
        Boost::thread
        Boost::filesystem
        Threads::Threads                         # portable pthreads
)
