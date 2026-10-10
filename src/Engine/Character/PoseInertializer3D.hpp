#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <vector>

namespace MatterEngine {

// Inercia de pose (a volta do fisico, como o inertial blending do Unreal): a
// diferenca entre a pose de partida e a pedida - juntas e raiz -, com a
// velocidade que ela tinha, se desfaz numa mola criticamente amortecida. A
// mistura linear partia de uma foto parada do corpo: na troca as pernas
// paravam e depois corriam atras da animacao.
class PoseInertializer3D {
public:
    // A pose de partida: juntas e as velocidades delas (rad, rad/s), a raiz
    // (mundo) e as velocidades dela.
    void start(const std::vector<Vec3>& coordinates, const std::vector<Vec3>& coordinateVelocities,
        Vec3 rootPosition, Quaternion rootOrientation, Vec3 rootVelocity, Vec3 rootAngularVelocity,
        float halfLifeSeconds, float durationSeconds);
    // Por cima da pose pedida deste tick (no lugar).
    void apply(std::vector<Vec3>& coordinates, Vec3& rootPosition, Quaternion& rootOrientation, float deltaTime);
    [[nodiscard]] bool active() const { return m_active; }
    void stop() { m_active = false; }

private:
    bool m_active = false;
    int m_ticks = 0;
    float m_elapsed = 0.0f, m_duration = 0.5f, m_omega = 12.0f;
    std::vector<Vec3> m_from, m_fromVelocity, m_offset, m_offsetVelocity, m_previousTarget;
    Vec3 m_fromRoot, m_fromRootVelocity, m_rootOffset, m_rootOffsetVelocity, m_previousRoot;
    Quaternion m_fromOrientation, m_previousOrientation;
    Vec3 m_fromAngularVelocity, m_turnOffset, m_turnOffsetVelocity;
};

} // namespace MatterEngine
