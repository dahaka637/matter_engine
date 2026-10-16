#pragma once

#include "Engine/Athlete/Modes/ContactClasses3D.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace MatterEngine {

// As regras de troca do modo do corpo como uma tabela (vault:
// 04_locomocao/reescrita_da_locomocao.md, R3). O passo do atleta mede os
// sinais; a tabela decide o que acontece e por que; quem executa a troca
// (semente, inercia, guia) continua sendo o passo.
//
// Invariantes:
// - a mesma regra de impacto vale no animado, na volta e na ponte;
// - a ponte (a volta do fisico ao padrao) pode ser abortada e, depois de
//   abortada, ha uma trava antes de tentar voltar de novo (sem oscilar);
// - toda decisao diz o evento, o valor medido e o limiar.

enum class BodyModeKind3D : std::uint8_t { Animated, Physical, Bridge, Returning, Active };

enum class ModeEvent3D : std::uint8_t {
    None,
    Grab,
    Impact,
    Penetration,
    CapsuleWall,
    ActionContact,
    AirTackle,
    Jump,
    BridgePose,
    BridgeTilt,
    BridgeSupport,
    BridgeAssist,
};

[[nodiscard]] const char* modeEventName3D(ModeEvent3D event);
[[nodiscard]] const char* bodyModeName3D(BodyModeKind3D mode);

struct ModeRuleParams3D {
    // A capsula batendo (m/s; a de hoje).
    float capsuleWallImpact = 1.5f;
    // A ponte e abortada se o corpo sai da pose pedida: o pendulo (centro de
    // massa sobre o apoio) mais que isto (graus) fora do pendulo da pose, por
    // pelo menos bridgePoseSeconds (o pendulo cru passa de 25 graus de
    // proposito no impulso do sprint: so o desvio da pose conta).
    float bridgePoseDegrees = 20.0f;
    float bridgePoseSeconds = 0.3f;
    // E o pendulo do proprio corpo acima disto (graus): as duas condicoes,
    // como a regra da postura perdida (no fim do levantar a pose ja inclina
    // para a arrancada enquanto o corpo sobe - so o desvio nao e queda).
    float bridgePendulumDegrees = 25.0f;
    // O peito tombando fora do que a pose faz (rad/s; a velocidade do "para
    // cima" do peito do corpo menos a da pose - nao o giro do rumo, nem o
    // tronco que sobe de proposito no fim do levantar). 0 desliga. Desligado
    // por padrao: a pose de hoje salta na entrada da ponte (a historia dela e
    // descontinua) e o criterio abortava toda volta do levantar; volta quando a
    // referencia nova (R4) der a velocidade da pose.
    float bridgeTiltRate = 0.0f;
    // Sem apoio por mais que isto (s; o sprint voa ~0,14 s + folga).
    float bridgeNoSupportSeconds = 0.3f;
    // A ajuda do impacto abaixo disto.
    float bridgeAssist = 0.8f;
    // Depois de abortar a ponte, quanto esperar antes de tentar voltar (s).
    float abortLockoutSeconds = 0.5f;
    // A volta so com o ultimo impacto ha mais que isto (s).
    float returnQuietSeconds = 0.3f;

    [[nodiscard]] bool valid() const;
};

// O que o passo mede num tick.
struct ModeSignals3D {
    bool grabbed = false;
    ContactAssessment3D contacts;
    // O contato da acao (carrinho, voadora) acima do limiar dela.
    bool actionContact = false;
    // A voadora que vai ao fisico ja no comeco (a opcao antiga).
    bool airTackleStart = false;
    bool jumped = false;
    float capsuleWallImpact = 0.0f;
    // Para a ponte: o pendulo do corpo (graus da vertical) e o desvio dele
    // para o da pose pedida.
    float pendulumDegrees = 0.0f;
    float poseDeviationDegrees = 0.0f;
    float chestTiltRate = 0.0f;
    bool supported = true;
    bool airborneByDesign = false;
    float assistLevel = 1.0f;
};

struct ModeDecision3D {
    bool change = false;
    BodyModeKind3D to = BodyModeKind3D::Physical;
    ModeEvent3D event = ModeEvent3D::None;
    // O valor medido e o limiar (a telemetria do motivo).
    float value = 0.0f;
    float threshold = 0.0f;
    int priority = 0;
};

// Uma linha da tabela: de quais modos (bits de BodyModeKind3D), para qual, com
// que prioridade (menor vence) e por qual evento.
struct ModeRule3D {
    std::uint8_t from = 0;
    BodyModeKind3D to = BodyModeKind3D::Physical;
    int priority = 0;
    ModeEvent3D event = ModeEvent3D::None;
};

[[nodiscard]] constexpr std::uint8_t modeBit3D(BodyModeKind3D mode) {
    return static_cast<std::uint8_t>(1u << static_cast<unsigned>(mode));
}

// A tabela que vale (a mesma do vault).
[[nodiscard]] std::span<const ModeRule3D> modeRuleTable3D();

// O pendulo: o centro de massa sobre o apoio, como inclinacao no plano
// (deslocamento horizontal / altura). O desvio (graus) entre o pendulo do corpo
// e o da pose pedida - o pendulo cru do sprint passa de 25 graus de proposito,
// so o desvio da pose diz que o corpo saiu dela.
[[nodiscard]] float pendulumDeviationDegrees3D(Vec3 centerOfMass, Vec3 support, Vec3 poseCenterOfMass,
    Vec3 poseSupport);
// O pendulo sozinho (graus da vertical).
[[nodiscard]] float pendulumDegrees3D(Vec3 centerOfMass, Vec3 support);

class ModeRules3D {
public:
    void reset();
    // O tick: o modo atual e os sinais. Decide (sem executar) a troca de maior
    // prioridade que dispara; guarda os relogios (desvio da pose, sem apoio,
    // desde o ultimo impacto, a trava).
    ModeDecision3D decide(BodyModeKind3D current, const ModeSignals3D& signals, const ModeRuleParams3D& params,
        float deltaTime);
    // A troca foi executada (a ponte abortada liga a trava).
    void noteTransition(BodyModeKind3D from, const ModeDecision3D& decision, const ModeRuleParams3D& params);
    // O fisico pode comecar a voltar? (A trava depois de abortar e o ultimo
    // impacto recente seguram.)
    [[nodiscard]] bool returnAllowed(const ModeRuleParams3D& params) const;
    [[nodiscard]] float secondsSinceImpact() const { return m_sinceImpact; }
    [[nodiscard]] float lockoutLeft() const { return m_lockout; }

private:
    float m_poseSeconds = 0.0f;
    float m_noSupportSeconds = 0.0f;
    float m_sinceImpact = 10.0f;
    float m_lockout = 0.0f;
};

} // namespace MatterEngine
