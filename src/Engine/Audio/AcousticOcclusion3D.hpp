#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

namespace MatterEngine {

struct AcousticOcclusionSettings3D {
    // Raios extras amostrados ao redor do caminho direto ouvinte->fonte,
    // dispostos em circulo perpendicular a esse caminho.
    int crossSampleCount = 4;
    float crossSampleRadiusMeters = 0.35f;
};

// Fracao continua de oclusao entre ouvinte e fonte: 0 = linha totalmente
// livre, 1 = todos os raios amostrados encontraram geometria estatica no
// caminho. Um unico raycast central produz um "pop" audivel na borda de uma
// obstrucao (a fonte alterna entre livre/bloqueada de um frame pro outro);
// amostrar em cruz ao redor do raio central suaviza essa transicao em uma
// faixa continua, sem exigir nenhuma simulacao de difracao de verdade.
[[nodiscard]] float computeOcclusionFactor3D(const PhysicsScene3D& scene,
    Vec3 listenerPosition, Vec3 sourcePosition,
    const AcousticOcclusionSettings3D& settings = {});

} // namespace MatterEngine
