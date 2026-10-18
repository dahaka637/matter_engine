#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <vector>

namespace MatterEngine {

// Um instante de um clipe como referencia (DeepMimic): as coordenadas de junta,
// a raiz e os links no mundo com as velocidades.
struct TrackingClipFrame3D {
    std::vector<Vec3> coordinates;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    std::vector<PhysicsBodyState3D> links;
    Vec3 centerOfMass;
};

// Um clipe (os de levantar do Kimodo) ancorado no mundo: o ponto do chao e o
// rumo em que a pose do clipe (no espaco do modelo: a raiz no modelPosition da
// pelve mais o deslocamento do clipe) e posta. Amostra suave (Catmull-Rom, como
// o jogo) e as velocidades por diferenca finita (DeepMimic: a velocidade alvo
// vem do dado por diferenca finita).
class TrackingClipReference3D final {
public:
    TrackingClipReference3D(const RagdollProfile3D& profile, const AnimationClip3D& clip);

    [[nodiscard]] float durationSeconds() const { return m_clip->durationSeconds; }
    [[nodiscard]] const AnimationClip3D& clip() const { return *m_clip; }

    // A pose no tempo t (preso ao clipe), ancorada em (ground, yaw).
    void frame(float t, Vec3 ground, float yawRadians, TrackingClipFrame3D& out) const;

private:
    void pose(float t, Vec3 ground, float yawRadians, std::vector<Vec3>& coordinates, Vec3& root,
        Quaternion& orientation) const;

    const RagdollProfile3D* m_profile = nullptr;
    const AnimationClip3D* m_clip = nullptr;
    // A trilha de cada link (nullptr: sem trilha) e a da raiz.
    std::vector<const AnimationTrack3D*> m_tracks;
    std::size_t m_root = 0;
};

} // namespace MatterEngine
