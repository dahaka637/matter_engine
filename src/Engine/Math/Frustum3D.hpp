#pragma once

#include "Engine/Math/Mat4.hpp"

#include <array>

namespace MatterEngine {

struct Plane3D {
    Vec3 normal;
    float distance = 0.0f;
};

// Volume de visibilidade extraido de uma matriz Vulkan (profundidade 0..1).
// Testes conservadores por esfera permitem eliminar props inteiros antes de
// criar batches ou command buffers, sem depender de tipos do backend grafico.
class Frustum3D final {
public:
    // reversedDepth precisa bater com a convencao da matriz de origem: as
    // formulas dos planos proximo/distante trocam de lugar entre profundidade
    // padrao (perto=0, longe=1) e invertida (perto=1, longe=0, ver
    // Mat4::perspective). Sem default de proposito - a camera principal usa
    // Mat4::perspective (sempre invertida) e a camera de sombra usa
    // Mat4::orthographic (sempre padrao), entao nenhum valor seria seguro
    // pros dois casos ao mesmo tempo.
    [[nodiscard]] static Frustum3D fromViewProjection(
        const Mat4& viewProjection, bool reversedDepth);

    [[nodiscard]] bool intersectsSphere(Vec3 center,
        float radius) const;

private:
    std::array<Plane3D, 6> m_planes;
};

} // namespace MatterEngine
