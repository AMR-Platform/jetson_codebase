// utils.hpp - Common utility functions

#pragma once
#include <chrono>
#include <thread>
#include <string>

namespace utils {
    void sleep_ms(int milliseconds);
    std::string timestamp();
}
