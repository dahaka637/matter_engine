#pragma once

#include <string>

namespace MatterEngine::Platform {

// Absolute directory containing the executable, with a trailing separator.
[[nodiscard]] std::string executableBasePath();

} // namespace MatterEngine::Platform
