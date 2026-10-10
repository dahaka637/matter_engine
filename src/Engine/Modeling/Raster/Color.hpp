#pragma once

#include <algorithm>
#include <cstdint>

namespace MatterEngine::Modeling {

// Cor de trabalho (0..1, ja no espaco de exibicao) e alfa.
struct Color {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 1.0f;
};

[[nodiscard]] constexpr Color rgb(int r, int g, int b, float alpha = 1.0f) {
    return { static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f, static_cast<float>(b) / 255.0f,
        alpha };
}

[[nodiscard]] constexpr Color withAlpha(Color color, float alpha) {
    return { color.r, color.g, color.b, alpha };
}

[[nodiscard]] constexpr Color mix(Color a, Color b, float t) {
    return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t };
}

[[nodiscard]] constexpr Color scaled(Color color, float factor) {
    return { std::clamp(color.r * factor, 0.0f, 1.0f), std::clamp(color.g * factor, 0.0f, 1.0f),
        std::clamp(color.b * factor, 0.0f, 1.0f), color.a };
}

// Uma cor por indice, bem separadas no circulo (partes, componentes).
[[nodiscard]] Color categoricalColor(int index);
// Escala de calor 0..1 (azul, ciano, verde, amarelo, vermelho).
[[nodiscard]] Color heatColor(float value);

// A paleta das ferramentas de inspecao.
namespace Palette {
inline constexpr Color BackgroundTop = rgb(22, 26, 32);
inline constexpr Color BackgroundBottom = rgb(6, 8, 11);
inline constexpr Color Grid = rgb(70, 82, 96, 0.55f);
inline constexpr Color GridMajor = rgb(96, 112, 130, 0.8f);
inline constexpr Color Center = rgb(236, 236, 228);
inline constexpr Color Left = rgb(70, 170, 255);
inline constexpr Color Right = rgb(255, 146, 60);
inline constexpr Color Focus = rgb(255, 224, 70);
inline constexpr Color Text = rgb(232, 238, 244);
inline constexpr Color TextMuted = rgb(150, 164, 178);
inline constexpr Color LabelBackground = rgb(8, 12, 18, 0.82f);
inline constexpr Color Error = rgb(255, 54, 84);
inline constexpr Color Warning = rgb(255, 196, 40);
inline constexpr Color Info = rgb(90, 200, 255);
inline constexpr Color Surface = rgb(196, 200, 206);
inline constexpr Color Backface = rgb(230, 40, 200);
inline constexpr Color Wire = rgb(20, 24, 30, 0.85f);
inline constexpr Color XRay = rgb(120, 200, 255);
inline constexpr Color Reference = rgb(255, 255, 255, 0.5f);
} // namespace Palette

} // namespace MatterEngine::Modeling
