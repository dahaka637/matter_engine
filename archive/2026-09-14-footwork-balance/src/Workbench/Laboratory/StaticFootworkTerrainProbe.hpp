#pragma once

#include "Engine/Locomotion/FootworkTypes3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>

namespace MatterEngine::Workbench {

// Adaptador único do laboratório: planejadores de pé enxergam apenas terreno
// estático. Props e outros ragdolls não viram chão por acidente.
class StaticFootworkTerrainProbe final : public FootworkTerrainProbe3D {
public:
    explicit StaticFootworkTerrainProbe(const PhysicsScene3D& scene)
        : m_scene(scene) {
    }

    bool raycastGround(Vec3 candidatePosition,
        float distanceAboveMeters, float distanceBelowMeters,
        FootworkTerrainHit3D& hit) const override {
        const float above = std::max(0.01f, distanceAboveMeters);
        const float below = std::max(0.01f, distanceBelowMeters);
        PhysicsRayHit3D physicsHit;
        if (!m_scene.raycastStatic(
                { candidatePosition + Vec3 { 0.0f, 0.0f, above },
                    { 0.0f, 0.0f, -1.0f } },
                above + below, physicsHit)) {
            return false;
        }
        hit.position = physicsHit.position;
        hit.normal = physicsHit.normal;
        hit.distanceMeters = physicsHit.distance;
        return true;
    }

private:
    const PhysicsScene3D& m_scene;
};

} // namespace MatterEngine::Workbench
