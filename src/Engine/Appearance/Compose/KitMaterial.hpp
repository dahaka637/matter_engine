#pragma once

#include "Engine/Appearance/Compose/KitCompositor.hpp"

#include <array>
#include <cstddef>

// O material da roupa (o contrato das etapas 2 e 3): a conta que o shader do
// personagem faz (scene3d_mesh.frag com MATTER_CHARACTER, linha por linha) e
// que o render da CPU do inspetor usa. O uniforme trilinear ate o nivel seguro
// e, depois dele, suave ate a media da regiao; a identificacao por cima so
// dentro do retangulo interno da zona no UV1 (o limite exato), sumindo para
// alfa zero depois do nivel seguro dela. O resultado e pre-multiplicado: por
// cima do corpo, cor + corpo * (1 - alfa).
namespace MatterEngine::Appearance {

// O nivel pelas derivadas do UV num pixel (as da GPU: o maior dos dois
// comprimentos em texels, log2).
[[nodiscard]] float kitLevelOfDetail(std::array<float, 2> uvPerPixelX, std::array<float, 2> uvPerPixelY, int width, int height);

// O uniforme e a identificacao no ponto (b0, b1, b2) do triangulo, com os
// niveis dados. Fora da roupa (sem painel): alfa zero.
[[nodiscard]] Rgba kitMaterialColor(const ComposedKit& kit, const SurfaceTemplate& surface, std::size_t triangle,
    std::array<float, 3> bary, float kitLod, float identityLod);

// O bilinear (a borda repetida, como o sampler clamp) e o trilinear ate o
// nivel maximo dado.
[[nodiscard]] Rgba kitSample(const KitTexture& level, float u, float v);
[[nodiscard]] Rgba kitTrilinear(const std::vector<KitTexture>& levels, int maximumLevel, float lod, float u, float v);

} // namespace MatterEngine::Appearance
