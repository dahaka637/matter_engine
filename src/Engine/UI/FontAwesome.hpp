#pragma once

#include <string>
#include <string_view>

namespace MatterEngine::UI::FontAwesome {

// Font Awesome Free 7 solid icon codepoints. Keep UI code independent from
// the font backend by passing these through icon() or label().
inline constexpr char32_t MagnifyingGlass = 0xF002;
inline constexpr char32_t User = 0xF007;
inline constexpr char32_t Check = 0xF00C;
inline constexpr char32_t Xmark = 0xF00D;
inline constexpr char32_t Gear = 0xF013;
inline constexpr char32_t House = 0xF015;
inline constexpr char32_t VolumeHigh = 0xF028;
inline constexpr char32_t Camera = 0xF030;
inline constexpr char32_t Crosshairs = 0xF05B;
inline constexpr char32_t Play = 0xF04B;
inline constexpr char32_t Pause = 0xF04C;
inline constexpr char32_t ArrowLeft = 0xF060;
inline constexpr char32_t Users = 0xF0C0;
inline constexpr char32_t FloppyDisk = 0xF0C7;
inline constexpr char32_t Wrench = 0xF0AD;
inline constexpr char32_t Bolt = 0xF0E7;
inline constexpr char32_t Lightbulb = 0xF0EB;
inline constexpr char32_t Cube = 0xF1B2;
inline constexpr char32_t Sliders = 0xF1DE;
inline constexpr char32_t Futbol = 0xF1E3;
inline constexpr char32_t Hand = 0xF256;
inline constexpr char32_t DoorOpen = 0xF52B;
inline constexpr char32_t HandFist = 0xF6DE;

[[nodiscard]] inline std::string icon(char32_t codepoint) {
    std::string result;
    if (codepoint <= 0x7F) {
        result.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FF) {
        result.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
        result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else if (codepoint <= 0xFFFF) {
        result.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
        result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else {
        result.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
        result.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
        result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
    return result;
}

[[nodiscard]] inline std::string label(char32_t codepoint, std::string_view text) {
    std::string result = icon(codepoint);
    result.append("  ");
    result.append(text);
    return result;
}

} // namespace MatterEngine::UI::FontAwesome
