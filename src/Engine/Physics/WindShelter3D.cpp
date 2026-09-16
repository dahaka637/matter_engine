#include "Engine/Physics/WindShelter3D.hpp"

#include "Engine/Math/Ray3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <algorithm>

namespace MatterEngine {

float windShelterExposure3D(const PhysicsScene3D& scene, Vec3 position,
    Vec3 windVelocity, float shelterDistanceMeters) {
    const float windSpeed = windVelocity.length();
    if (windSpeed <= 0.01f || shelterDistanceMeters <= 0.0f) return 1.0f;

    // O vento vem de fora - se houver parede/geometria estatica logo a
    // barlavento (entre a posicao e de onde o vento sopra), o ar ali fica
    // parado/turbulento, nao com a velocidade livre do vento aberto.
    const Vec3 upwindDirection = windVelocity * (-1.0f / windSpeed);
    const Ray3D upwindRay { position, upwindDirection };
    PhysicsRayHit3D shelterHit;
    if (!scene.raycastStatic(upwindRay, shelterDistanceMeters, shelterHit)) {
        return 1.0f;
    }

    // Falloff suave (smoothstep) em vez de um corte abrupto entre "vento
    // total" e "nada": parede colada bloqueia quase tudo, parede no limite
    // do alcance mal se nota.
    const float t = std::clamp(
        shelterHit.distance / shelterDistanceMeters, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace MatterEngine
