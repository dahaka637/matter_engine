#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

namespace MatterEngine {

// A ajuda do modo fisico depois de um impacto (pedido do usuario, 06/10): a
// ajuda escolhida (100% por padrao) cai pela forca do choque - ate a metade
// num choque forte (parede, mureta, um boneco, um objeto), ate 20% num
// carrinho levado de outro boneco - e volta aos poucos ate a inteira, num
// tempo que cresce com a queda; so entao ele volta a locomocao padrao.
struct ImpactAssistSettings3D {
    // Choque comum: o impulso (N.s) a partir do qual conta e o que leva a
    // queda inteira (maximumDrop: 0,5 = a ajuda vai a 50%).
    float minimumImpulse = 4.0f;
    float fullImpulse = 45.0f;
    float maximumDrop = 0.5f;
    // Carrinho levado: qualquer um tira tackleBaseDrop; o impulso ate
    // tackleFullImpulse leva a tackleMaximumDrop (0,8 = a ajuda a 20%).
    // (Os carrinhos medidos no boneco: ~6 a 13 N.s - a ajuda vai de ~45% a
    // ~25%.)
    float tackleMinimumImpulse = 2.0f;
    float tackleFullImpulse = 14.0f;
    float tackleBaseDrop = 0.45f;
    float tackleMaximumDrop = 0.8f;
    // A volta (s): recoverBaseSeconds + recoverSecondsPerDrop x a queda.
    float recoverBaseSeconds = 0.5f;
    float recoverSecondsPerDrop = 3.0f;
};

class ImpactAssist3D {
public:
    void reset();
    // O choque deste tick (impactImpulse3D) e se foi um carrinho levado
    // (com o impulso dele).
    void update(float impulse, bool tackled, float tackleImpulse, float deltaTime);
    // O quanto da ajuda escolhida vale agora (0..1).
    [[nodiscard]] float level() const { return m_level; }
    [[nodiscard]] bool recovering() const { return m_level < 0.995f; }
    [[nodiscard]] ImpactAssistSettings3D& settings() { return m_settings; }

private:
    ImpactAssistSettings3D m_settings;
    float m_floor = 1.0f;
    float m_level = 1.0f;
    float m_elapsed = 0.0f;
    float m_seconds = 0.0f;
};

// O choque de um tick no corpo (N.s): os contatos com o que nao e chao
// (paredes, muretas, outros bonecos, objetos); o chao (normal para cima)
// nao conta - pousos, quedas e o levantar tem as ajudas deles.
[[nodiscard]] float impactImpulse3D(const RagdollState3D& state);

} // namespace MatterEngine
