#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/PoseInertializer3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <vector>

namespace MatterEngine {

// O levantar pelo padrao (28/10, item 16, pedido do usuario: "o ato de
// levantar-se agora vai ocorrer por locomocao default, para ser mais fluido...
// levantando-se com postura perfeita... desde que haja base solida"). O corpo
// guiado segue o clipe de levantar, ancorado no corpo deitado: o rumo pelo
// eixo do corpo no chao (da pelve a cabeca - deitado, a frente da pelve nao
// diz nada), a pelve do primeiro quadro no lugar da pelve dele e o chao do
// clipe no chao medido. A pose fisica de agora entra no clipe pela inercia.
// Quem chama decide quando (a base, o contato) e entrega ao padrao no fim.
class AnimatedGetUp3D {
public:
    // O corpo deitado: as juntas e as velocidades delas (rad, rad/s; o indice
    // do perfil), a pelve e a cabeca no mundo, a pelve (orientacao e
    // velocidades) e o chao embaixo da pelve.
    struct Body {
        const std::vector<Vec3>* coordinates = nullptr;
        const std::vector<Vec3>* coordinateVelocities = nullptr;
        Vec3 pelvisPosition;
        Quaternion pelvisOrientation;
        Vec3 pelvisVelocity;
        Vec3 pelvisAngularVelocity;
        Vec3 headPosition;
        float groundHeight = 0.0f;
    };

    void start(const RagdollProfile3D& profile, const AnimationClip3D& clip, const Body& body);
    // A pose do tick no mundo (para o corpo guiado), o tempo andando.
    void step(const RagdollProfile3D& profile, float deltaTime, RagdollAnimationPose3D& pose);
    // A pose do clipe num instante, so com a ancora (sem a inercia): o teste
    // da base olha o caminho do levantar inteiro antes de comecar.
    void poseAt(const RagdollProfile3D& profile, float timeSeconds, RagdollAnimationPose3D& pose) const;
    void stop() { m_active = false; }
    [[nodiscard]] bool active() const { return m_active; }
    [[nodiscard]] bool finished() const { return m_clip != nullptr && m_time >= m_clip->durationSeconds; }
    [[nodiscard]] float timeSeconds() const { return m_time; }
    [[nodiscard]] float durationSeconds() const { return m_clip != nullptr ? m_clip->durationSeconds : 0.0f; }

private:
    void clipRoot(const RagdollProfile3D& profile, float timeSeconds, std::vector<Vec3>& coordinates, Vec3& rootPosition,
        Quaternion& rootOrientation) const;

    const AnimationClip3D* m_clip = nullptr;
    bool m_active = false;
    float m_time = 0.0f;
    // A ancora: o giro (em z) e o pivo do clipe (a pelve do primeiro quadro
    // sobre o chao do modelo) levado a base no mundo.
    Quaternion m_turn;
    Vec3 m_pivot;
    Vec3 m_base;
    PoseInertializer3D m_inertia;
};

} // namespace MatterEngine
