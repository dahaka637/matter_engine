#include "Engine/Core/Log.hpp"
#include <iostream>

namespace MatterEngine {

void Log::info(std::string_view message) { std::cout << "[INFO] " << message << '\n'; }
void Log::warn(std::string_view message) { std::cout << "[WARN] " << message << '\n'; }
void Log::error(std::string_view message) { std::cerr << "[ERROR] " << message << '\n'; }

}
