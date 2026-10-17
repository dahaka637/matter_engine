#pragma once

#include "Engine/Athlete/Training/LocomotionTask3D.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstddef>

namespace MatterEngine {

// Os termos da nota do treino da locomocao, na ordem das parcelas.
enum class LocomotionRewardTerm3D : std::size_t {
    Velocity = 0, Facing, Style, Contact, Slip, Work, Smooth, Assist, Posture, Residual, Fall, Count
};
inline constexpr std::size_t LocomotionRewardTermCount3D = static_cast<std::size_t>(LocomotionRewardTerm3D::Count);
inline constexpr std::array<const char*, LocomotionRewardTermCount3D> LocomotionRewardTermNames3D {
    "velocidade", "rumo", "estilo", "contato", "escorregar", "trabalho", "trancos", "ajuda", "postura", "residuo", "queda"
};
// Os quatro primeiros somam; os outros (custos e a queda) subtraem.
inline constexpr std::size_t LocomotionRewardPositiveTerms3D = 4;

// O que uma decisao mediu (a media dos ticks dela).
struct LocomotionStepMeasure3D {
    // O erro do centro de massa contra a velocidade viavel (m/s, RMS) e o do
    // peito contra a camera (rad, RMS).
    float velocityError = 0.0f;
    float facingError = 0.0f;
    // O erro das partes-chave contra a pose de estilo (m, RMS; so com a
    // referencia valida).
    bool styleValid = false;
    float styleError = 0.0f;
    // A soma dos nucleos dos pousos desta decisao (exp(-d^2/sigma^2) por pouso
    // com meta da referencia).
    float contactScore = 0.0f;
    // O pe apoiado (m/s), a potencia dos motores (W/kg), a mudanca da acao
    // (media dos quadrados), a forca da ajuda na pelve (fracao do peso), o
    // angulo fora do envelope da postura (rad) e o residuo da rede (o aplicado
    // sobre o teto, media dos quadrados).
    float slip = 0.0f;
    float power = 0.0f;
    float jerk = 0.0f;
    float assist = 0.0f;
    float posture = 0.0f;
    float residual = 0.0f;
    bool fell = false;
};

// As parcelas de uma decisao (ja vezes a duracao dela: a soma das quatro
// primeiras menos as outras e a nota).
[[nodiscard]] std::array<float, LocomotionRewardTermCount3D> locomotionRewardTerms3D(
    const LocomotionStepMeasure3D& measure, const LocomotionRewardWeights3D& weights, float decisionSeconds);
[[nodiscard]] float locomotionReward3D(const std::array<float, LocomotionRewardTermCount3D>& terms);

// A postura funcional (o envelope do Corpo v2, vault 04_locomocao/
// postura_funcional.md): o tronco inclina dentro de uma faixa que abre com a
// demanda - frear abre para tras, acelerar para a frente, curvar para dentro -
// e so o que passa dela custa. Nunca trava o uso funcional do tronco.
struct LocomotionPostureInput3D {
    // O eixo de cima do peito no referencial do rumo (x frente, y esquerda).
    Vec3 chestUp { 0.0f, 0.0f, 1.0f };
    // A aceleracao medida do centro de massa e a pedida pelo comando, no rumo
    // (m/s2; a maior das duas abre cada semieixo).
    Vec3 measuredAcceleration;
    Vec3 commandAcceleration;
    // A velocidade do corpo no plano (m/s).
    float speed = 0.0f;
};
// O angulo fora do envelope (rad, >= 0).
[[nodiscard]] float locomotionPostureExcess3D(const LocomotionPostureInput3D& input);

} // namespace MatterEngine
