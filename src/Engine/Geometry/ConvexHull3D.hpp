#pragma once

#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// Uma casca convexa: os vertices e os triangulos (indices neles, no sentido
// anti-horario visto de fora).
struct ConvexHull3D {
    std::vector<Vec3> points;
    std::vector<std::array<std::uint16_t, 3>> faces;
};

// A casca convexa de uma nuvem de pontos (quickhull). Com tolerance > 0 ela
// para quando nenhum ponto fica mais que isso fora: fica no maximo tolerance
// por dentro da exata, com bem menos vertices (o colisor da chuteira). Ate
// maxVertices vertices (os mais longe entram primeiro). Pontos todos num plano
// (ou numa reta) nao tem casca: lanca.
[[nodiscard]] ConvexHull3D buildConvexHull3D(std::span<const Vec3> points, float tolerance = 0.0f,
    std::size_t maxVertices = 256);

// O quanto o ponto fica fora da casca (o maior dos planos das faces; negativo
// por dentro).
[[nodiscard]] float convexHullOutside3D(const ConvexHull3D& hull, Vec3 point);

} // namespace MatterEngine
