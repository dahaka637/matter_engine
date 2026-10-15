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
    // (cauda longa: 1 / (1 + (e/tol)^2), sinal em toda parte), "bell"
    // (exp(-e^2 / (2 tol^2))) ou "mixed" (metade cauchy, metade linear:
    // max(0, 1 - e / (|pedido| + 1)), com inclinacao constante longe do
    // pedido: parado com pedido de andar paga pouco, meio caminho ja paga). A
    // tolerancia e absoluta mais relativa ao pedido e nunca vai a zero.
    std::string velocityForm = "cauchy";
    float velocityToleranceAbsolute = 0.2f;
    float velocityToleranceRelative = 0.1f;
    // "progress" (o plano de recuperacao, 5.4): com pedido de movimento, o
    // quanto o corpo se aproximou do pedido contra ficar parado,
    //   P = clamp((L(e0) - L(e)) / L(e0), -1, 1),  L(x) = sqrt(x^2 + d^2) - d,
    //   d = progressDeltaBase + progressDeltaPerSpeed x |pedido|,
    // 0 parado, 1 no pedido, negativo na direcao errada; com pedido de parar
    // (|pedido| < movingThreshold), exp(-(|v| / stopSigma)^2). O olhar so paga
    // junto com o progresso (max(0, P)).
    float movingThreshold = 0.3f;
    float progressDeltaBase = 0.1f;
    float progressDeltaPerSpeed = 0.1f;
    float stopSigma = 0.2f;
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
    float posture = 0.0f;
    // A queda (um evento: paga uma vez).
    float fall = 1.0f;
    // O total por decisao nunca fica negativo (antes da queda): tentar e errar
    // nunca vale menos que ficar parado (a aquisicao; sai na F1 com os custos).
    bool positiveOnly = false;
    // Multiplica a tarefa (0 = so o estilo e a muleta: o P2, onde o juiz sozinho
    // tem de ensinar a passada de um clipe).
    float taskScale = 1.0f;
    // A porta de "de pe" (5.4): multiplica a tarefa para tombar para a frente
    // nao render velocidade. Vale 1 numa faixa larga (inclinar para correr e
    // agachar para arrancar nao perdem nada); sobe de 0 a 1 entre os limites.
    struct StandGate {
        bool enabled = false;
        float heightFrom = 0.55f;
        float heightTo = 0.80f;
        float uprightFromDegrees = 65.0f;
        float uprightToDegrees = 40.0f;
    } standGate;
    // A muleta do pe no ar (o feetAirTime do v6; so a aquisicao): com um pe so
    // no chao e pedido de movimento, paga min(capSeconds, tempo do pe de apoio
    // plantado, tempo do pe de balanco no ar) / capSeconds, vezes o peso.
    struct Air {
        float weight = 0.0f;
        float capSeconds = 0.4f;
    } air;
};

// Os pesos da tarefa num exercicio (somam 1).
struct BodyTaskWeights3D {
    float velocity = 0.7f;
    float facing = 0.3f;
};

// As parcelas por decisao. Task e a tarefa em taxa por segundo, ja com a porta
// de pe (T); StandGate e a porta (g); StyleGate e a porta do estilo (G: o
// estilo so paga junto com o progresso e de pe); Air e a muleta do pe no ar (A,
// em taxa, ja com o peso e a porta); Fall e a queda ja aplicada (F). O
// treinador compoe  dt x max(0, T + beta x G x S + A - custos) - F  com o
// estilo S, que so ele tem. As tres ultimas vem depois da queda para as
// primeiras manterem o lugar.
enum class BodyRewardTerm3D : std::size_t {
    Velocity = 0, Facing, Task, Work, Torque, Slip, Impact, Head, Smooth, Limits, Contacts, Fall, StandGate, StyleGate,
    Air, Posture, Count
};

inline constexpr std::array<const char*, static_cast<std::size_t>(BodyRewardTerm3D::Count)> BodyRewardTermNames3D {
    "velocity", "facing", "task", "work", "torque", "slip", "impact", "head", "smooth", "limits", "contacts", "fall",
    "stand_gate", "style_gate", "air", "posture"
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
    // A postura: o quanto o peito e a cabeca saem da vertical, (1 - cos) de
    // cada um, somados (0 de pe reto; ~0,03 com 10 graus em cada; ~0,27 com 30).
    float posture = 0.0f;
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
    double m_posture = 0.0;
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
// A porta de pe (0..1): a altura da pelve sobre a de pe e o "para cima" da
// pelve contra a vertical (cosseno). 1 se a porta esta desligada.
[[nodiscard]] float bodyStandGate3D(const BodyRewardSettings3D& settings, float pelvisHeightRatio, float uprightCosine);
// A muleta do pe no ar (0..1, antes do peso): um pe so no chao, com pedido de
// movimento. contactSeconds/airSeconds: ha quanto tempo cada pe esta no estado
// de agora (no chao / no ar).
[[nodiscard]] float bodyAirCredit3D(const BodyRewardSettings3D& settings, bool moving, const bool (&contact)[2],
    const float (&seconds)[2]);

// A tarefa de uma decisao: as notas e as portas (so a forma "progress" mexe
// nas portas do olhar e do estilo).
struct BodyTaskTerms3D {
    float velocityScore = 0.0f;
    float facingScore = 0.0f;
    // g: a porta de pe; multiplica a tarefa.
    float standGate = 1.0f;
    // A porta do olhar: max(0, P) com pedido de movimento na forma "progress".
    float facingGate = 1.0f;
    // G: a porta do estilo, ja com g.
    float styleGate = 1.0f;
    // O credito da muleta do pe no ar (0..1).
    float airCredit = 0.0f;
};

// As notas de velocidade e do olhar de uma decisao e as portas, pela forma
// escolhida e pelo pedido.
[[nodiscard]] BodyTaskTerms3D bodyTaskTerms3D(const BodyRewardSettings3D& settings, Vec3 measured, Vec3 wanted,
    float facingScore, float standGate, float airCredit);

// A nota da decisao (sem o estilo) e as parcelas (BodyRewardTermNames3D).
[[nodiscard]] float bodyReward3D(const BodyRewardSettings3D& settings, const BodyTaskWeights3D& weights,
    const BodyTaskTerms3D& terms, const BodyCosts3D& costs, bool fell, float decisionSeconds, float* components);
// A mesma conta com as portas em 1 e sem a muleta (a tarefa dos testes e das
// formas antigas).
[[nodiscard]] float bodyReward3D(const BodyRewardSettings3D& settings, const BodyTaskWeights3D& weights,
    float velocityScore, float facingScore, const BodyCosts3D& costs, bool fell, float decisionSeconds,
    float* components);

} // namespace MatterEngine
