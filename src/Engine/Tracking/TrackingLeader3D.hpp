#pragma once

#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/Reference/BodyForecast3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"

#include <vector>

namespace MatterEngine {

// O pedido ao lider num tick: a velocidade no plano (m/s, mundo) e para onde
// o corpo olha (rad).
struct TrackingCommand3D {
    Vec3 requestedVelocityWorld;
    float facingYawRadians = 0.0f;
};

// O lider num instante (o personagem cinematico do DReCon, vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secao 2).
struct TrackingLeaderState3D {
    bool valid = false;
    // A raiz (a origem da pelve) e as coordenadas de junta por link (a raiz,
    // indice 0, sem uso), na convencao dos alvos dos motores, e a variacao
    // delas por tick (diferenca finita).
    Vec3 rootPosition;
    Quaternion rootOrientation;
    std::vector<Vec3> coordinates;
    std::vector<Vec3> coordinateVelocities;
    // Os links no mundo, com as velocidades (a convencao do backend: a
    // velocidade linear e a do centro de massa do link).
    std::vector<PhysicsBodyState3D> links;
    Vec3 centerOfMass;
    Vec3 centerOfMassVelocity;
    // O rumo horizontal da pelve (o referencial F_kin).
    float headingYawRadians = 0.0f;
    // A velocidade pedida pelo jogador (v_des, no plano).
    Vec3 requestedVelocityWorld;
};

// O lider cinematico do rastreador: o modo padrao do jogo (DefaultLocomotion3D,
// com os ajustes do runtime) andando sobre uma capsula de dados com a regra
// dela (bodyForecastCapsuleStep3D: a mesma do controle de personagem no chao).
// Nao segue o corpo fisico: e ele quem manda (DReCon).
class TrackingLeader3D final {
public:
    TrackingLeader3D(const RagdollProfile3D& profile, const CharacterLocomotionAnimations3D& defaultClips,
        const AthleteRuntimeSettings3D& settings);

    // Recomeca parado num ponto do chao, olhando para facingYaw.
    void reset(Vec3 groundPosition, float facingYawRadians);
    // Um tick: a capsula anda pelo pedido e o padrao poe o corpo sobre ela.
    void step(const TrackingCommand3D& command, float deltaTime);

    [[nodiscard]] const TrackingLeaderState3D& state() const { return m_state; }
    [[nodiscard]] Vec3 capsulePosition() const { return m_capsulePosition; }
    [[nodiscard]] Vec3 capsuleVelocity() const { return m_capsuleVelocity; }
    [[nodiscard]] const DefaultLocomotion3D& locomotion() const { return m_locomotion; }

private:
    const RagdollProfile3D* m_profile = nullptr;
    const CharacterLocomotionAnimations3D* m_clips = nullptr;
    BodyForecastInput3D m_response;
    DefaultLocomotion3D m_locomotion;
    Vec3 m_capsulePosition;
    Vec3 m_capsuleVelocity;
    bool m_hasPrevious = false;
    Vec3 m_previousRoot;
    Quaternion m_previousRootOrientation;
    std::vector<Vec3> m_previousCoordinates;
    TrackingLeaderState3D m_state;
};

} // namespace MatterEngine
