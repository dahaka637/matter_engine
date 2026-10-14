#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyCommand3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// A nota do Corpo v2 (plano_claude_rl.md secao 7): por decisao,
//   r = dt x [tarefa - custos] + eventos - quedas
// e o treinador soma dt x beta x estilo (o juiz de estilo). A tarefa fica entre
// 0 e 1; os custos sao positivos e calibrados por pares de trajetorias.

struct BodyRewardSettings3D {
    // A forma do termo de velocidade (o Experimento B compara): "cauchy"
    // (cauda longa: 1 / (1 + (e/tol)^2), sinal em toda parte) ou "bell"
    // (exp(-e^2 / (2 tol^2))). A tolerancia e absoluta mais relativa ao
    // pedido e nunca vai a zero.
    std::string velocityForm = "cauchy";
    float velocityToleranceAbsolute = 0.2f;
    float velocityToleranceRelative = 0.1f;
    // O olhar: o sino do erro do peito contra a camera (rad).
    float facingSigma = 0.4f;
    // Os pesos dos custos.
    float work = 0.0f;
    float torque = 0.0f;
    float slip = 0.0f;
    float impact = 0.0f;
    float head = 0.0f;
    float smooth = 0.0f;
    float limits = 0.0f;
    float contacts = 0.0f;
    // A queda (um evento: paga uma vez).
    float fall = 1.0f;
};

// Os pesos da tarefa num exercicio (somam 1).
struct BodyTaskWeights3D {
    float velocity = 0.7f;
    float facing = 0.3f;
};

enum class BodyRewardTerm3D : std::size_t {
    Velocity = 0, Facing, Task, Work, Torque, Slip, Impact, Head, Smooth, Limits, Contacts, Fall, Count
};

inline constexpr std::array<const char*, static_cast<std::size_t>(BodyRewardTerm3D::Count)> BodyRewardTermNames3D {
    "velocity", "facing", "task", "work", "torque", "slip", "impact", "head", "smooth", "limits", "contacts", "fall"
};

// Os custos fisicos de uma decisao, antes dos pesos.
struct BodyCosts3D {
    // O trabalho positivo dos motores na decisao dividido pelo peso e pela
    // duracao (m/s: a potencia por peso).
    float work = 0.0f;
    // A media de (torque / teto)^2 dos eixos atuados nos passos da decisao.
    float torque = 0.0f;
    // A velocidade tangente no ponto de contato dos pes no chao, pesada pela
    // carga (m/s; um pe rolando nao escorrega).
    float slip = 0.0f;
    // A forca de cada pe acima de 2 pesos, na media da decisao (pesos).
    float impact = 0.0f;
    // O giro da cabeca para os lados e para frente e tras (rad/s)^2 mais o
    // tranco vertical dela.
    float head = 0.0f;
    // A mudanca do alvo de uma decisao para a outra (rad^2, media dos eixos)
    // mais 4 vezes as reversoes (ida e volta).
    float smooth = 0.0f;
    // A fatia dos eixos encostando no fim do curso (5%), na media dos passos.
    float limits = 0.0f;
    // 1 se algo que nao e pe tocou o chao.
    float contacts = 0.0f;
    // Medida, sem peso: a fatia dos eixos-passos com o torque a 98% do teto
    // ou mais (o tempo no teto).
    float saturation = 0.0f;
};

// Os custos somados nos passos de fisica de uma decisao.
class BodyCostAccumulator3D {
public:
    void begin(const RagdollState3D& state, const RLBody3D& body);
    void addSubstep(const RLBody3D& body, const RagdollState3D& state, float fixedStep, float bodyWeight);
    // A mudanca dos alvos entre decisoes (o fim da reta), pela decisao.
    void setTargetChange(std::span<const float> previousTargets, std::span<const float> previousChange,
        std::span<const float> targets);
    [[nodiscard]] BodyCosts3D finish(float decisionSeconds, float bodyWeight) const;

private:
    double m_work = 0.0;
    double m_slip = 0.0;
    double m_impact = 0.0;
    double m_head = 0.0;
    float m_smooth = 0.0f;
    float m_torque = 0.0f;
    float m_limits = 0.0f;
    float m_saturation = 0.0f;
    float m_contacts = 0.0f;
    float m_headVertical = 0.0f;
    std::uint32_t m_substeps = 0;
};

// O termo de velocidade: o vetor medido contra o pedido, no plano.
[[nodiscard]] float bodyVelocityScore3D(const BodyRewardSettings3D& settings, Vec3 measured, Vec3 wanted);
// O termo do olhar: o rumo do peito contra o da camera.
[[nodiscard]] float bodyFacingScore3D(const BodyRewardSettings3D& settings, float chestHeading, float faceHeading);

// A nota da decisao (sem o estilo) e as parcelas (BodyRewardTermNames3D).
[[nodiscard]] float bodyReward3D(const BodyRewardSettings3D& settings, const BodyTaskWeights3D& weights,
    float velocityScore, float facingScore, const BodyCosts3D& costs, bool fell, float decisionSeconds,
    float* components);

} // namespace MatterEngine
