#pragma once

#include "Engine/Modeling/Raster/Color.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine::Modeling {

// RGBA8 contiguo: serve direto para PNG e para a textura da interface.
struct Pixel {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    std::uint8_t a = 255;
};

// Coordenada de textura: u para a direita, v para baixo na imagem (0 a 1).
struct Uv {
    float u = 0.0f;
    float v = 0.0f;
};

class Image {
public:
    Image() = default;
    Image(int width, int height, Color fill = { 0.0f, 0.0f, 0.0f, 1.0f });

    [[nodiscard]] int width() const { return m_width; }
    [[nodiscard]] int height() const { return m_height; }
    [[nodiscard]] bool empty() const { return m_pixels.empty(); }
    [[nodiscard]] bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < m_width && y < m_height; }

    [[nodiscard]] Color pixel(int x, int y) const;
    // A cor no ponto (u, v) da imagem inteira, bilinear (os centros dos
    // pixels em (i + 0,5) / largura; a borda se repete para fora).
    [[nodiscard]] Color sample(Uv uv) const;
    void setPixel(int x, int y, Color color);
    // Mistura por cima: alfa da cor vezes a cobertura.
    void blendPixel(int x, int y, Color color, float coverage = 1.0f);
    void fill(Color color);
    void fillVerticalGradient(Color top, Color bottom);
    // Retangulo misturado (cantos inclusivos/exclusivos como em [x0, x1)).
    void fillRect(int x0, int y0, int x1, int y1, Color color);
    void drawImage(const Image& source, int x, int y, float opacity = 1.0f);

    // Media de blocos factor x factor (o supersampling).
    [[nodiscard]] Image downsampled(int factor) const;
    // Bilinear (as imagens de referencia em outra escala).
    [[nodiscard]] Image resized(int width, int height) const;
    [[nodiscard]] Image cropped(int x, int y, int width, int height) const;

    [[nodiscard]] std::span<const Pixel> pixels() const { return m_pixels; }
    [[nodiscard]] std::span<Pixel> pixels() { return m_pixels; }
    [[nodiscard]] static Image fromRgba8(int width, int height, std::span<const std::uint8_t> rgba);

private:
    int m_width = 0;
    int m_height = 0;
    std::vector<Pixel> m_pixels;
};

} // namespace MatterEngine::Modeling
