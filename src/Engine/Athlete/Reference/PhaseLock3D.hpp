#pragma once

#include <array>

namespace MatterEngine {

// O relogio da referencia segue o corpo (a licao do RL v1: o relogio do clipe
// nao esperava o corpo; vault 04_locomocao/rl_sobre_a_locomocao.md). A fase do
// corpo sai dos pousos de verdade - 0 no pouso do pe esquerdo, 0,5 no do
// direito; entre eles ela anda no ritmo da referencia - e o laco devolve o
// ritmo que puxa o relogio da referencia ate ela.
struct PhaseLockParams3D {
    // O pe no ar antes de um toque contar como pouso (s): um pisca de contato
    // nunca conta.
    float touchdownAirSeconds = 0.1f;
    // O ritmo: 1 + ganho x o erro de fase, entre o minimo e o maximo.
    float gain = 3.0f;
    float minimumRate = 0.5f;
    float maximumRate = 1.5f;
    // So puxa com um pouso recente (s) e com a referencia andando.
    float touchdownMemorySeconds = 1.0f;
    float movingShare = 0.5f;
};

class PhaseLock3D {
public:
    // Recomeca sem fase (a referencia recomecou: depois de uma queda).
    void reset();

    // Um tick, antes do passo da referencia: os pes encostando no chao agora,
    // a fase da referencia (0..1, do tick anterior), o quanto ela anda, se ela
    // esta ativa e se o laco puxa. Devolve o ritmo do relogio dela (1: nada).
    // Os dois pes pousando no mesmo tick (o pouso de um pulo) sao ambiguos: a
    // fase segue andando.
    float update(std::array<bool, 2> touching, float referencePhase, float referenceMovingShare, bool referenceActive,
        bool enabled, float deltaTime, const PhaseLockParams3D& params = {});

    [[nodiscard]] float bodyPhase() const { return m_bodyPhase; }
    [[nodiscard]] bool valid() const { return m_valid; }

private:
    float m_bodyPhase = 0.0f;
    bool m_valid = false;
    float m_sinceTouchdown = 10.0f;
    std::array<float, 2> m_footAir { 0.0f, 0.0f };
    float m_referencePhase = 0.0f;
};

} // namespace MatterEngine
