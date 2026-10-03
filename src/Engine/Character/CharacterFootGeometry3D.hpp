#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <cmath>

// Geometria do pe e rotacoes pequenas, comuns ao planejador de passos e a
// locomocao. Posicoes de pe aqui sao sempre a ORIGEM DO LINK do pe (o que o
// IK recebe), salvo quando o nome diz sola/bola.
namespace MatterEngine {

// Quanto o ponto mais baixo do colisor fica abaixo do centro dele, na
// orientacao em que esta. Tratar o pe como uma placa de halfExtents.z
// subestima isso assim que o tornozelo inclina - 12 graus num pe de 30 cm
// poem a ponta ~3 cm mais baixa.
inline float lowestColliderOffset3D(const RagdollLinkDefinition3D& link,
    Quaternion colliderWorldOrientation) {
    const Vec3 up = colliderWorldOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
    if (link.collider.shape == RagdollColliderShape3D::Box) {
        const Vec3 half = link.collider.boxHalfExtents;
        return std::abs(up.x) * half.x + std::abs(up.y) * half.y + std::abs(up.z) * half.z;
    }
    // Capsula: o cilindro corre ao longo do X local, com o raio nas pontas.
    return std::abs(up.x) * std::max(0.0f,
        link.collider.lengthMeters * 0.5f - link.collider.radiusMeters)
        + link.collider.radiusMeters;
}

// Altura da origem do link do pe para a sola (na orientacao dada) encostar
// num chao de altura `groundHeight`.
inline float footLockHeight3D(const RagdollLinkDefinition3D& link,
    float groundHeight, Quaternion orientation) {
    const Quaternion collider = (orientation * link.collider.localOrientation).normalized();
    return groundHeight + lowestColliderOffset3D(link, collider)
        - orientation.rotate(link.collider.localPosition).z + 0.001f;
}

// Para onde a ponta do pe aponta (mundo).
inline Vec3 footForwardWorld3D(const RagdollLinkDefinition3D& link, Quaternion orientation) {
    return orientation.rotate(link.modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
}

inline float footYaw3D(const RagdollLinkDefinition3D& link, Quaternion orientation) {
    const Vec3 forward = footForwardWorld3D(link, orientation);
    return std::atan2(forward.y, forward.x);
}

// A bola do pe (60% do caminho para a ponta, na sola), no referencial do link.
inline Vec3 footBallLocal3D(const RagdollLinkDefinition3D& link) {
    const Vec3 half = link.collider.shape == RagdollColliderShape3D::Box
        ? link.collider.boxHalfExtents : Vec3 {};
    return link.collider.localPosition
        + link.collider.localOrientation.rotate({ half.x * 0.6f, 0.0f, -half.z });
}

inline Quaternion rotationBetween3D(Vec3 from, Vec3 to) {
    constexpr float Pi = 3.14159265358979323846f;
    from = from.normalized();
    to = to.normalized();
    const float cosine = std::clamp(dot(from, to), -1.0f, 1.0f);
    const Vec3 axis = cross(from, to);
    const float sine = axis.length();
    if (sine < 0.000001f) {
        if (cosine > 0.0f) return {};
        const Vec3 other = std::abs(from.x) < 0.9f
            ? Vec3 { 1.0f, 0.0f, 0.0f } : Vec3 { 0.0f, 1.0f, 0.0f };
        return Quaternion::fromAxisAngle(cross(from, other).normalized(), Pi);
    }
    return Quaternion::fromAxisAngle(axis / sine, std::atan2(sine, cosine));
}

inline Quaternion slerp3D(Quaternion from, Quaternion to, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float cosine = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
    if (cosine < 0.0f) {
        to = { -to.x, -to.y, -to.z, -to.w };
        cosine = -cosine;
    }
    if (cosine > 0.9995f) {
        return Quaternion { from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t,
            from.z + (to.z - from.z) * t, from.w + (to.w - from.w) * t }.normalized();
    }
    const float angle = std::acos(std::clamp(cosine, -1.0f, 1.0f));
    const float sine = std::sin(angle);
    const float a = std::sin((1.0f - t) * angle) / sine;
    const float b = std::sin(t * angle) / sine;
    return Quaternion { from.x * a + to.x * b, from.y * a + to.y * b,
        from.z * a + to.z * b, from.w * a + to.w * b }.normalized();
}

} // namespace MatterEngine
