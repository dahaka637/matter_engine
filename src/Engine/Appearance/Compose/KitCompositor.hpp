#pragma once

#include "Engine/Appearance/Compose/KitDesign.hpp"
#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

// O compositor do uniforme (o contrato das etapas 2 e 3, no vault; a convencao
// de cor e a de compatibilidade): o desenho e as cores do kit compostos no
// atlas do UV0 (o layout do template) e o numero na textura de identificacao.
// Os triangulos com painel sao rasterizados no espaco do atlas; cada camada e
// avaliada na coordenada dela (painel, carta ou referencia), com a cobertura
// analitica (fill, faixas, listras, degrade) ou por amostra (caminho, texto e
// toda camada com mascara: o produto por amostra). As margens (16 texels em
// cada nivel) sao avaliadas de novo: numa costura continua, do outro lado
// dela; numa abertura ou juncao, o proprio triangulo estendido. Os mips sao
// por ilha (a caixa 2x2 com as margens da propria ilha: a borda fica centrada
// no texel, como do outro lado da costura); o nivel seguro e o ultimo em
// que o filtro trilinear de todo texel de tecido so le a propria ilha. Igual
// bit a bit com qualquer numero de threads.
namespace MatterEngine::Appearance {

struct KitTexture {
    int width = 0;
    int height = 0;
    // RGBA8 pre-multiplicado (o atlas e a identificacao). Onde o desenho nao
    // pinta (alfa 0), o material mostra o corpo por baixo: o alto do meiao que
    // fica acima da faixa e pele (a pele e do jogador, nao do uniforme).
    std::vector<std::uint8_t> rgba;
};

struct ComposedKit {
    // Todos os niveis, ate 1x1.
    std::vector<KitTexture> kit;
    int kitSafeLevel = 0;
    // A media da cor do tecido por regiao (pre-multiplicada, na ordem do
    // template; alfa 0 = sem tinta): a transicao depois do nivel seguro.
    std::vector<Rgba> regionAverage;
    std::vector<KitTexture> identity;
    int identitySafeLevel = 0;
    // Por regiao, os texels de tecido sem tinta inteira (alfa < 1: o corpo
    // aparece) e os triangulos de roupa sem texel nenhum no nivel 0.
    std::vector<std::size_t> transparentTexels;
    std::size_t trianglesWithoutTexel = 0;
    struct Timings {
        double prepare = 0.0, compose = 0.0, margins = 0.0, mips = 0.0, identity = 0.0, total = 0.0;
    } milliseconds;
};

struct ComposeOptions {
    // 0: as threads da maquina.
    int threads = 0;
    int margin = 16;
};

// sourceTriangles: os triangulos do template pelos vertices da malha de origem
// (os das costuras), na ordem do template.
[[nodiscard]] std::optional<ComposedKit> composeKit(const SurfaceTemplate& surface,
    std::span<const std::array<std::uint32_t, 3>> sourceTriangles, const KitDesign& design, const Kit& kit,
    const ComposeOptions& options, std::string* error);

} // namespace MatterEngine::Appearance
