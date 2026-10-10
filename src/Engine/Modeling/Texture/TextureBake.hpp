#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <functional>
#include <vector>

// O bake: a pintura e uma funcao do ponto da superficie (no espaco do modelo,
// na pose de ligacao) e o atlas guarda o resultado de cada texel, pelo mapa
// de textura da malha (UvAtlas). Varias amostras por texel (bordas lisas nos
// desenhos) e uma sangria em volta das ilhas (a filtragem nao puxa o fundo).
// Em paralelo (faixas de linhas: cada texel e de uma thread so) e adaptativo:
// uma amostra no centro de cada texel e todas so onde a cor muda para um
// vizinho (a borda de um desenho), na beira de uma ilha e nas partes de
// detalhe fino - o resto e liso e o centro ja e a media. A funcao da pintura
// e chamada de varias threads ao mesmo tempo (so leitura).
namespace MatterEngine::Modeling {

struct SurfaceSample {
    Vec3 position;
    // A normal do triangulo.
    Vec3 normal;
    int part = -1;
    int material = -1;
    std::size_t triangle = 0;
    // O lado da area que a amostra representa na superficie (metros): a
    // borda de um desenho fica lisa nesse tamanho (0: sem meio-termo).
    float footprint = 0.0f;
};

using SurfacePaint = std::function<Color(const SurfaceSample&)>;

struct TextureBakeSettings {
    int size = 2048;
    // Amostras por lado de cada texel (2 = quatro amostras).
    int samples = 2;
    // Texels de sangria em volta das ilhas.
    int bleed = 4;
    // A largura da borda de um desenho em cada amostra, em texels (0: o
    // pedaco do texel da amostra, 1 / samples - a borda de um texel). Mais
    // larga, a borda inclinada no atlas nao vira escada quando a textura e
    // ampliada (o filtro bilinear mostra os degraus de uma borda de um texel).
    float edgeTexels = 0.0f;
    // Adaptativo (desligado: todas as amostras em todo texel, o resultado de
    // referencia) e a diferenca de cor com um vizinho que pede todas.
    bool adaptive = true;
    float adaptiveThreshold = 2.0f / 255.0f;
    // As partes com detalhe mais fino que um texel (o rosto, o cadarco): todas
    // as amostras sempre.
    std::vector<int> fineParts;
};

// A imagem do atlas (o fundo fora das ilhas e da sangria fica transparente).
[[nodiscard]] Image bakeTexture(const Mesh& mesh, const SurfacePaint& paint, const TextureBakeSettings& settings = {});

} // namespace MatterEngine::Modeling
