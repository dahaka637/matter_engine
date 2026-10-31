#pragma once

#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// O levantar livre (31/10): uma rede so, de qualquer queda, pelo AMP (Peng et al. 2021, "AMP: Adversarial Motion
// Priors for Stylized Physics-Based Character Control"): a nota da tarefa (ficar de pe) mais a do estilo, de um
// discriminador que separa as transicoes do corpo das dos clipes de levantar - sem seguir o clipe quadro a quadro.
//
// As caracteristicas de um instante que o discriminador ve (AMP, secao 6), no referencial do rumo do corpo:
// a altura da raiz; a orientacao da raiz (tangente e normal); as velocidades linear e angular da raiz; de cada junta
// a rotacao no pai (tangente e normal) e a velocidade angular no pai; a cabeca, as maos e os pes em relacao a raiz.
// O mesmo calculo vale para o corpo simulado e para os quadros de um clipe.
class TrackingAmpFeatures3D final {
public:
    explicit TrackingAmpFeatures3D(const RagdollProfile3D& profile);

    [[nodiscard]] std::size_t size() const { return m_size; }
    void compute(std::span<const PhysicsBodyState3D> links, std::span<float> out) const;

private:
    const RagdollProfile3D* m_profile = nullptr;
    std::uint32_t m_root = 0;
    std::vector<std::uint32_t> m_keyBodies;
    std::size_t m_size = 0;
};

// A situacao do corpo no levantar: as alturas da cabeca e da pelve (m), o alto da pelve (z do eixo de cima, 1 em pe)
// e a frente do peito (z: + de costas para o chao, - de brucos). De pe: cabeca acima de 1,4 m, pelve acima de
// 0,75 m e a pelve em pe (o alto acima de 0,8).
struct TrackingGetUpPosture3D {
    float headHeight = 0.0f;
    float pelvisHeight = 0.0f;
    float upright = 0.0f;
    float chestFacingUp = 0.0f;
    bool standing = false;
};

class TrackingGetUpPostureProbe3D final {
public:
    explicit TrackingGetUpPostureProbe3D(const RagdollProfile3D& profile);

    [[nodiscard]] TrackingGetUpPosture3D measure(std::span<const PhysicsBodyState3D> links) const;
    // A nota da tarefa (0 a 1): a cabeca subindo (0,15 m deitado a 1,5 m de pe) 0,6, a pelve em pe 0,2 e, de pe, o
    // centro de massa parado no chao 0,2 (exp(-2 v^2)).
    [[nodiscard]] float taskReward(const TrackingGetUpPosture3D& posture, std::span<const PhysicsBodyState3D> links) const;
    // O potencial da virada (Ng, Harada e Russell 1999: a nota e gamma Phi(s') - Phi(s), que paga a virada e nao o
    // ficar parado, e nao muda o melhor jeito de levantar): no chao, o peito para baixo vale ate 0,15, sumindo
    // conforme a cabeca sobe. (31/10: como nota de estado, ele aprendeu a ficar ajoelhado de cabeca no chao.)
    [[nodiscard]] float turnPotential(const TrackingGetUpPosture3D& posture) const;

private:
    const RagdollProfile3D* m_profile = nullptr;
    std::uint32_t m_root = 0;
    std::uint32_t m_head = 0;
    std::uint32_t m_chest = 0;
};

} // namespace MatterEngine
