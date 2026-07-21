#pragma once

#include <cstdint>
#include <string_view>

namespace MatterEngine {

inline constexpr std::uint32_t VersionMajor = 0;
inline constexpr std::uint32_t VersionMinor = 1;
inline constexpr std::uint32_t VersionPatch = 0;
inline constexpr std::string_view Version = "0.1.0-alpha";

} // namespace MatterEngine
