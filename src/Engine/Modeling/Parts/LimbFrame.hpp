#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <cmath>

namespace MatterEngine::Modeling {

// O referencial de um membro (braco): ao longo do osso (de uma junta para a
// seguinte), para a frente (+x do corpo) e para cima (+z), os dois
// perpendiculares ao osso. A manga e a pele do braco usam o mesmo.
struct LimbFrame {
    Vec3 origin;
    Vec3 along { 0.0f, -1.0f, 0.0f };
    Vec3 forward { 1.0f, 0.0f, 0.0f };
    Vec3 up { 0.0f, 0.0f, 1.0f };

    // O ponto na estacao s (metros desde a origem, no osso), deslocado para
    // a frente e para cima.
    [[nodiscard]] Vec3 at(float s, float forwardOffset = 0.0f, float liftOffset = 0.0f) const {
        return origin + along * s + forward * forwardOffset + up * liftOffset;
    }
    // A direcao no plano perpendicular ao osso (0 = frente, pi/2 = cima).
    [[nodiscard]] Vec3 direction(float angle) const { return forward * std::cos(angle) + up * std::sin(angle); }
    [[nodiscard]] float angleOf(Vec3 point, Vec3 center) const {
        const Vec3 d = point - center;
        return std::atan2(dot(d, up), dot(d, forward));
    }
    [[nodiscard]] float station(Vec3 point) const { return dot(point - origin, along); }
};

[[nodiscard]] inline LimbFrame limbFrame(Vec3 from, Vec3 to) {
    LimbFrame frame;
    frame.origin = from;
    frame.along = safeNormalized(to - from, { 0.0f, -1.0f, 0.0f });
    const Vec3 x { 1.0f, 0.0f, 0.0f }, z { 0.0f, 0.0f, 1.0f };
    frame.forward = safeNormalized(x - frame.along * dot(x, frame.along), x);
    frame.up = safeNormalized(z - frame.along * dot(z, frame.along) - frame.forward * dot(z, frame.forward), z);
    return frame;
}

} // namespace MatterEngine::Modeling
