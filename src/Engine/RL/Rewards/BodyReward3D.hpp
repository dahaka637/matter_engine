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
    float still = 0.0f;
    float athletic = 0.0f;
    // A postura: a inclinacao do peito que nao custa cresce com a velocidade
    // e com a aceleracao do corpo (graus: a base, mais por m/s, mais atan(a/g):
    // o quanto a fisica pede para acelerar, frear, curvar ou segurar um
    // empurrao; ate o maximo). A cabeca so tem a base.
    float postureLeanDegrees = 0.0f;
    float postureLeanPerSpeed = 0.0f;
    float postureLeanMaxDegrees = 0.0f;
    // Mais folga quanto mais baixo o quadril (graus por fracao da altura de pe
    // abaixo dela): a postura de prontidao inclina o tronco.
    float postureLeanPerCrouch = 0.0f;
    // A postura funcional (postura_funcional.md): a linha do tronco inteiro
    // (pelve ate a cabeca) e a da parte baixa (pelve ate o UpperChest), pelos
    // centros dos colisores, no rumo do peito, dentro de uma faixa. Em ritmo
    // constante a faixa e estreita (nada para tras); ela se abre na direcao da
    // aceleracao fisica: frear abre para tras, acelerar para a frente, curvar
    // para dentro. Fora da faixa custa o angulo (rad, linear). Ligada, ela
    // substitui a folga do peito acima (a cabeca fica).
    struct FunctionalPosture {
        bool enabled = false;
        float fullWeight = 0.3f;
        float lowWeight = 0.7f;
        // Os pisos (graus), parado e em movimento (mistura linear ate 1 m/s).
        float fullFloorStanding = 0.0f;
        float fullFloorMoving = 3.0f;
        float lowFloorStanding = -6.0f;
        float lowFloorMoving = -3.0f;
        // Os tetos (graus): a base, mais por m/s, mais por fracao agachada (ate
        // ceilingCrouchMax).
        float fullCeiling = 15.0f;
        float lowCeiling = 8.0f;
        float ceilingPerSpeed = 1.5f;
        float ceilingPerCrouch = 30.0f;
        float ceilingCrouchMax = 8.0f;
        // Para cada lado (graus).
        float side = 5.0f;
        // A demanda: a aceleracao medida do centro de massa (suavizada em
        // accelSeconds) e a do comando ((pedido - corpo) / commandSeconds, com
        // zona morta e teto, suavizada em commandSeconds, vezes commandShare).
        // Por semieixo (frente, tras, esquerda, direita) vale a maior.
        float accelSeconds = 0.25f;
        float commandSeconds = 0.25f;
        float commandDeadband = 0.3f;
        float commandMaximum = 8.0f;
        float commandShare = 0.5f;
        // A abertura por semieixo: atan(a/g) em graus, ate o limite.
        float openForwardMax = 25.0f;
        float openBackMax = 18.0f;
        float openSideMax = 25.0f;
    } functionalPosture;
    // A postura atletica: o quadril acima de athleticHeight x a altura de pe
    // reto custa sempre; parado (o relogio do still), a base mais estreita que
    // athleticWidth (m, entre os pes de lado na pelve) tambem.
    float athleticHeight = 0.94f;
    float athleticWidth = 0.35f;
    // Parado (o relogio do still), com athleticStillKnee > 0: no lugar do
    // quadril baixo, os joelhos dobrados (o que falta ao minimo nos dois, em rad; a pose de
    // alerta do clipe; abrir as pernas com os joelhos retos baixava o quadril
    // sem dobrar nada), e a base mais larga que athleticMaxWidth (m) custa.
    float athleticStillKnee = 0.0f;
    float athleticMaxWidth = 0.0f;
    // Andando: a cada pisada, a base mais estreita que athleticWalkWidth (m)
    // tambem custa (fracao que falta; 0 = desligado).
    float athleticWalkWidth = 0.0f;
    // O pivo: o giro (rad/s) dos pes apoiados em torno da vertical (so com a
    // camera quieta: girar no lugar pode girar o pe).
    float pivot = 0.0f;
    // O parado de verdade: o custo still so vale com o pedido de parar ha mais
    // de stillGraceSeconds (os passos para frear nao pagam).
    float stillGraceSeconds = 1.0f;
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
    Air, Posture, Still, Athletic, Pivot, Count
};

inline constexpr std::array<const char*, static_cast<std::size_t>(BodyRewardTerm3D::Count)> BodyRewardTermNames3D {
    "velocity", "facing", "task", "work", "torque", "slip", "impact", "head", "smooth", "limits", "contacts", "fall",
    "stand_gate", "style_gate", "air", "posture", "still", "athletic", "pivot"
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
    // Com a postura funcional ligada, o peito sai e entram as duas linhas do
    // tronco: o angulo alem da faixa (rad), pesado por linha.
    float posture = 0.0f;
    // O parado: a velocidade media dos pes (m/s) quando o pedido e parar ha
    // mais que a folga (marchar no lugar paga; o lote o preenche).
    float still = 0.0f;
    // A postura atletica: o quanto o quadril passa da altura alvo (fracao da
    // altura de pe) mais, parado, o quanto a base falta (fracao da largura alvo).
    float athletic = 0.0f;
    // O pivo: o giro medio dos pes apoiados em torno da vertical (rad/s).
    float pivot = 0.0f;
    // Medida, sem peso: a fatia dos eixos-passos com o torque a 98% do teto
    // ou mais (o tempo no teto).
    float saturation = 0.0f;
};

// As linhas do tronco (rad), pelos centros dos colisores, no rumo do peito:
// o tronco inteiro (pelve ate a cabeca), a parte baixa (pelve ate o
// UpperChest) e o pescoco (UpperChest ate a cabeca, so medida). Para a frente
// positiva, para tras negativa; para o lado positiva para a esquerda.
struct BodyPostureLines3D {
    float fullForward = 0.0f;
    float fullSide = 0.0f;
    float lowForward = 0.0f;
    float lowSide = 0.0f;
    float neckForward = 0.0f;
    // O rumo do peito no plano (frente e esquerda, unitarios).
    Vec3 ahead { 1.0f, 0.0f, 0.0f };
    Vec3 left { 0.0f, 1.0f, 0.0f };
};
[[nodiscard]] BodyPostureLines3D bodyPostureLines3D(const RLBody3D& body, const RagdollState3D& state);

// A leitura da postura num passo: as linhas, a aceleracao suavizada do centro
// de massa (m/s2, no plano) e a velocidade dele.
struct BodyPostureReading3D {
    BodyPostureLines3D lines;
    Vec3 acceleration;
    Vec3 velocity;
};

// Os custos somados nos passos de fisica de uma decisao.
class BodyCostAccumulator3D {
public:
    // A tolerancia da inclinacao do peito (graus; 0 = a postura antiga, sem folga).
    void setLean(float baseDegrees, float perSpeed, float maximumDegrees, float perCrouch = 0.0f) {
        m_leanBase = baseDegrees;
        m_leanPerSpeed = perSpeed;
        m_leanMax = maximumDegrees;
        m_leanPerCrouch = perCrouch;
    }
    void setFunctionalPosture(const BodyRewardSettings3D::FunctionalPosture& posture) { m_functional = posture; }
    // O pedido de velocidade desta decisao (a demanda do comando).
    void setCommand(Vec3 moveVelocity) { m_command = moveVelocity; }
    // A ultima leitura das linhas do tronco e da aceleracao (sempre medida,
    // mesmo com a postura funcional desligada: as medidas por episodio).
    [[nodiscard]] const BodyPostureReading3D& postureReading() const { return m_reading; }
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
    // A aceleracao horizontal da pelve, suavizada (m/s2), e a velocidade de antes.
    float m_accel = 0.0f;
    Vec3 m_lastVelocity;
    bool m_haveVelocity = false;
    float m_leanBase = 0.0f;
    float m_leanPerSpeed = 0.0f;
    float m_leanMax = 0.0f;
    float m_leanPerCrouch = 0.0f;
    BodyRewardSettings3D::FunctionalPosture m_functional;
    // A postura funcional: o pedido, a velocidade do centro de massa de antes
    // e as duas aceleracoes suavizadas (seguem de uma decisao para a outra).
    Vec3 m_command;
    Vec3 m_comVelocity;
    bool m_haveCom = false;
    Vec3 m_accelSmooth;
    Vec3 m_commandSmooth;
    BodyPostureReading3D m_reading;
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
