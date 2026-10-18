#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingFeatures3D.hpp"
#include "Engine/Tracking/TrackingLeader3D.hpp"

#include <span>
#include <vector>

namespace MatterEngine {

// As parcelas da nota do DReCon (secao 6.5) num passo.
struct TrackingRewardTerms3D {
    float position = 0.0f;
    float velocity = 0.0f;
    float localPose = 0.0f;
    float centerOfMassVelocity = 0.0f;
    float fallFactor = 0.0f;
    float total = 0.0f;
    float headDistance = 0.0f;
    // A cabeca a mais de 1 m da do lider: o episodio acaba (com nota zero).
    bool terminate = false;
};

// A nota do rastreador, sem nenhum termo inventado (vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secao 5):
//   r_p      = exp(-10/N sum_i sum_j |p^_ij - p_ij|)   (6 pontos por corpo, F_kin x F_sim)
//   r_v      = exp(-1/N  sum_i sum_j |v^_ij - v_ij|)
//   r_local  = exp(-10/N sum_j |a^_j (-) a_j|_q)        (orientacao de cada corpo no pai)
//   r_vCM    = exp(-|v^_CM - v_CM|)
//   e_queda  = clamp(1,3 - 1,4 |p^_cabeca - p_cabeca|, 0, 1)
//   r        = e_queda (r_p + r_v + r_local + r_vCM)
// N = os corpos que entram no termo (18 nos pontos; 17 na pose local - a raiz
// nao tem pai: a orientacao dela em relacao ao rumo seria global, e o DReCon diz
// que este termo e so a pose local).
class TrackingReward3D final {
public:
    explicit TrackingReward3D(const RagdollProfile3D& profile);

    [[nodiscard]] TrackingRewardTerms3D evaluate(const TrackingLeaderState3D& leader,
        std::span<const PhysicsBodyState3D> bodyLinks, Vec3 bodyCenterOfMass, Vec3 bodyCenterOfMassVelocity);

private:
    const RagdollProfile3D* m_profile = nullptr;
    std::vector<TrackingSurfaceBox3D> m_boxes;
    std::uint32_t m_head = 0;
    // (Reaproveitados entre chamadas: a nota roda a cada decisao de cada mundo.)
    std::vector<Vec3> m_kinPoints, m_kinVelocities, m_simPoints, m_simVelocities;
};

} // namespace MatterEngine
