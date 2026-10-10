#pragma once

#include "Engine/Modeling/Raster/Image.hpp"

#include <cstdint>

// A pintura em pixel art dos objetos procedurais: uma imagem pequena (cada
// pixel e um texel de verdade) com as primitivas da pintura a mao -
// retangulo, contorno, linha grossa, tom variando por pixel (pela semente: a
// mesma receita da a mesma imagem) - e a ampliacao sem filtro (cada texel
// vira um bloco), para quem amostra bilinear (a GPU, a inspecao) manter o
// pixel nitido.
namespace MatterEngine::Modeling {

// Um numero 0..1 fixo por (x, y, semente).
[[nodiscard]] float pixelNoise(int x, int y, std::uint32_t seed);

class PixelCanvas {
public:
    PixelCanvas(int width, int height, Color fill = { 0.0f, 0.0f, 0.0f, 1.0f });

    [[nodiscard]] int width() const { return m_image.width(); }
    [[nodiscard]] int height() const { return m_image.height(); }
    [[nodiscard]] Color at(int x, int y) const;
    void set(int x, int y, Color color);
    // Os retangulos vao de x0..x1 e y0..y1 (inclusive).
    void fillRect(int x0, int y0, int x1, int y1, Color color);
    void strokeRect(int x0, int y0, int x1, int y1, Color color);
    // Os pixels com o centro a ate thickness/2 do segmento (coordenadas em
    // pixels; o centro do pixel (x, y) e (x + 0,5, y + 0,5)).
    void line(float x0, float y0, float x1, float y1, float thickness, Color color);
    // Multiplica o tom (sombra < 1, realce > 1).
    void shade(int x, int y, float factor);
    void shadeRect(int x0, int y0, int x1, int y1, float factor);
    // Cada pixel vezes 1 +- amount (o ruido da semente).
    void jitter(int x0, int y0, int x1, int y1, float amount, std::uint32_t seed);
    void blit(const PixelCanvas& source, int x, int y);

    [[nodiscard]] const Image& image() const { return m_image; }
    // Cada pixel vira um bloco factor x factor.
    [[nodiscard]] Image upscaled(int factor) const;

private:
    Image m_image;
};

} // namespace MatterEngine::Modeling
