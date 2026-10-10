#pragma once

#include "Engine/Modeling/Raster/Color.hpp"
#include "Engine/Modeling/Raster/Image.hpp"

#include <vector>

// Uma imagem para a pintura (o escudo, o patrocinio em PNG): as cores com a
// transparencia pre-multiplicada (a beira de um desenho recortado nao fica
// com franja escura na media) e as reducoes pela metade - a amostra le o
// nivel do tamanho dela na superficie (a imagem grande num texel pequeno nao
// pisca nem serrilha).
namespace MatterEngine::Modeling {

class PaintImage {
public:
    explicit PaintImage(const Image& image);

    [[nodiscard]] int width() const { return m_levels.empty() ? 0 : m_levels.front().width; }
    [[nodiscard]] int height() const { return m_levels.empty() ? 0 : m_levels.front().height; }
    // A cor (com alfa, sem pre-multiplicar) em (u, v) de 0 a 1 (v = 0 em
    // cima, como no arquivo); "size": o tamanho da amostra em fracao da
    // largura. Fora da imagem, transparente.
    [[nodiscard]] Color sample(float u, float v, float size) const;

private:
    struct Level {
        int width = 0;
        int height = 0;
        std::vector<Color> texels;
    };
    std::vector<Level> m_levels;

    [[nodiscard]] Color bilinear(const Level& level, float u, float v) const;
};

} // namespace MatterEngine::Modeling
