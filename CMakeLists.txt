cmake_minimum_required(VERSION 3.16)
project(jetson_codebase LANGUAGES CXX)

# ---------- Global compile settings ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # VS Code can consume this

# ---------- Dependencies ----------
# Eigen is header-only, but the imported target adds its include dir for us
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------- Sources ----------
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

# ---------- Executable ----------
add_executable(main_exe ${SOURCES})

# ---------- Include dirs & link libs ----------
# Your own headers
target_include_directories(main_exe
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

# Header-only libraries are still “linked” so their include dirs propagate
target_link_libraries(main_exe
    PUBLIC Eigen3::Eigen
)
