#!/bin/bash

# Clean previous build
rm -rf build

# Create new build directory
mkdir -p build
cd build

# Generate Makefiles with CMake
cmake ..

# Compile using all available cores
make -j$(nproc)

# Go back to project root
cd ..

# Run the executable with sudo
sudo ./build/main_exe