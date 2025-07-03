// utils.cpp - Common utility function implementations

#include "utils.hpp"
#include <ctime>
#include <iomanip>
#include <sstream>

namespace utils {

void sleep_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::string timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%F %T");
    return ss.str();
}

}
