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

    // floor: a altura do chao embaixo da pelve (m; a rampa do treino): as alturas da cabeca e da pelve contam dele.
    [[nodiscard]] TrackingGetUpPosture3D measure(std::span<const PhysicsBodyState3D> links, float floor = 0.0f) const;
    // Only standing. Functional balance earns up to 1 without pose constraints during perturbation.
    // Settled IDLE adds pose/spine/quietness (maximum 2), so fidgeting cannot evade the posture objective.
    [[nodiscard]] float standingReward(const TrackingGetUpPosture3D& posture,
        std::span<const PhysicsBodyState3D> links, bool quiet = true) const;
    // As poses do parado (um quadro de cada clipe parado; 31/10 18h, ele: o padrao de ficar de pe e o parado em alerta,
    // com o espelho): a rotacao de cada junta no pai. setIdlePose troca todas por uma; addIdlePose junta outra.
    void setIdlePose(std::span<const PhysicsBodyState3D> links);
    void addIdlePose(std::span<const PhysicsBodyState3D> links);
    // A semelhanca com a pose do parado mais proxima: metade exp(-0,5 soma dos angulos^2 de cada junta) - o largo, que
    // da gradiente longe (o -2 do DeepMimic sozinho dava 0,007 na pose de 07h50, soma 2,5) - e metade exp(-2 soma), o
    // estreito, que segue puxando perto (21h25: a soma media 1,26, o largo ja dava 0,57). 1 igual.
    [[nodiscard]] float idlePoseMatch(std::span<const PhysicsBodyState3D> links) const;
    // A coluna como a do parado mais proximo (o parado em alerta do clipe, reto de lado desde 23/10): o eixo de cima da
    // pelve, do abdomen, do peito, do alto do peito, do pescoco e da cabeca no referencial do rumo da pelve, exp(-soma
    // dos angulos^2 / 0,1) (~10 graus em cada elo da 0,18; 5, 0,63; 2, 0,93). 1 igual.
    [[nodiscard]] float idleSpineMatch(std::span<const PhysicsBodyState3D> links) const;
    // O potencial da subida (31/10 07h: como nota de estado - a cabeca subindo 0,6 e a pelve em pe 0,2 -, ele assentava
    // sentado ou ajoelhado em 1 s e colhia ~0,4 por passo ate o fim; como potencial, a subida paga ao subir, a descida
    // devolve e ficar parado no meio nao rende): 30 x (0,6 a cabeca subindo de 0,15 a 1,5 m + 0,2 a pelve em pe).
    [[nodiscard]] float risePotential(const TrackingGetUpPosture3D& posture) const;
    // O potencial da virada (Ng, Harada e Russell 1999: a nota e gamma Phi(s') - Phi(s), que paga a virada e nao o
    // ficar parado, e nao muda o melhor jeito de levantar): no chao, o peito para baixo vale ate 0,15, sumindo
    // conforme a cabeca sobe. (31/10: como nota de estado, ele aprendeu a ficar ajoelhado de cabeca no chao.)
    [[nodiscard]] float turnPotential(const TrackingGetUpPosture3D& posture) const;

private:
    const RagdollProfile3D* m_profile = nullptr;
    std::uint32_t m_root = 0;
    std::uint32_t m_head = 0;
    std::uint32_t m_chest = 0;
    std::vector<std::vector<Quaternion>> m_idle;
    std::vector<std::uint32_t> m_spine;
    std::vector<std::vector<Vec3>> m_idleSpine;
    void spineAxes(std::span<const PhysicsBodyState3D> links, std::vector<Vec3>& out) const;
};

} // namespace MatterEngine
