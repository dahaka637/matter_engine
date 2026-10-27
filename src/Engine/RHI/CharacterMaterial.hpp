#pragma once

#include "Engine/RHI/RHIHandles.hpp"
#include "Engine/RHI/RHITypes.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

// O material do personagem (o sistema de aparencia, etapa 3; o contrato no
// vault, 06_render/plano_sistema_de_aparencia.md): o uniforme composto (um
// texture array, uma camada por uniforme, os niveis prontos e o sampler
// proprio ate o nivel seguro), a identificacao (outro array), o atlas antigo
// do corpo (por baixo, onde o uniforme nao pinta) e, por vertice, o UV0, o
// UV1, a regiao e a zona. Estes layouts sao os mesmos do shader
// (scene3d_mesh.vert/.frag com MATTER_CHARACTER).
namespace MatterEngine::RHI {

// Sem regiao (a pele sem painel) ou sem zona.
inline constexpr std::uint32_t CharacterKitNone = 0xFFFFFFFFu;
inline constexpr std::size_t CharacterKitMaxRegions = 32;
inline constexpr std::size_t CharacterKitMaxZones = 4;
// As camadas do array do uniforme (os uniformes diferentes vivos ao mesmo tempo).
inline constexpr std::size_t CharacterKitMaxLayers = 8;

// Uma entrada por vertice final, na ordem do vertex buffer (std430). A entrada
// do vertice e gl_VertexIndex - baseVertex (parameters.info.x; hoje 0: o
// personagem e desenhado com vertexOffset 0).
struct CharacterKitVertexGpu {
    float kitUv[2] { 0.0f, 0.0f };
    float identityUv[2] { 0.0f, 0.0f };
    std::uint32_t region = CharacterKitNone;
    std::uint32_t zone = CharacterKitNone;
};
static_assert(sizeof(CharacterKitVertexGpu) == 24);

// Os parametros (std140).
struct CharacterKitParametersGpu {
    // x o nivel seguro do uniforme, y o da identificacao.
    float levels[4] { 0.0f, 0.0f, 0.0f, 0.0f };
    // A media do tecido por regiao (pre-multiplicada), por camada do uniforme:
    // a transicao depois do nivel seguro ([camada * 32 + regiao]).
    float regionAverage[CharacterKitMaxLayers * CharacterKitMaxRegions][4] {};
    // Por zona, o retangulo interno do bloco no UV1 (u0, v0, u1, v1): o limite
    // exato da identificacao por fragmento.
    float zoneRect[CharacterKitMaxZones][4] {};
    // x baseVertex.
    std::uint32_t info[4] { 0, 0, 0, 0 };
};
static_assert(sizeof(CharacterKitParametersGpu) == 16 + CharacterKitMaxLayers * CharacterKitMaxRegions * 16 + CharacterKitMaxZones * 16 + 16);

struct CharacterMaterialDesc {
    // Um array: as camadas, cada uma com os niveis do 0 ate levels-1 (largura e
    // altura do nivel l: max(1, x >> l)), RGBA8 pre-multiplicado, em sequencia
    // camada a camada e nivel a nivel. maxLod: o nivel seguro.
    struct Array {
        Extent2D extent {};
        std::uint32_t layers = 1;
        std::uint32_t levels = 1;
        float maxLod = 0.0f;
        std::span<const std::byte> pixels;
    };
    Array kit;
    Array identity;
    // O corpo (o atlas antigo, no UV antigo da malha).
    TextureHandle body;
    std::span<const CharacterKitVertexGpu> vertices;
    CharacterKitParametersGpu parameters;
};

// O tamanho em bytes de uma camada (todos os niveis) e o de um nivel.
[[nodiscard]] constexpr std::size_t characterLevelBytes(Extent2D extent, std::uint32_t level) {
    const std::size_t width = extent.width >> level > 0 ? extent.width >> level : 1;
    const std::size_t height = extent.height >> level > 0 ? extent.height >> level : 1;
    return width * height * 4;
}
[[nodiscard]] constexpr std::size_t characterLayerBytes(Extent2D extent, std::uint32_t levels) {
    std::size_t bytes = 0;
    for (std::uint32_t level = 0; level < levels; ++level) bytes += characterLevelBytes(extent, level);
    return bytes;
}

} // namespace MatterEngine::RHI
