#pragma once

#include "Engine/Athlete/AthleteState3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstdint>

namespace MatterEngine {

// A telemetria neutra de um atleta (a reescrita da locomocao, R0; vault
// 04_locomocao/reescrita_da_locomocao.md): o que a sonda, os testes e o
// laboratorio leem do controle, sem tocar no estado de uma implementacao. O
// caminho legado a publica hoje; as pecas novas publicam a mesma coisa, e a
// mesma bateria vale para as duas.
struct AthleteTelemetry3D {
    // O modo do corpo (a ponte separada do Ativo) e ha quanto tempo.
    BodyModeKind3D mode = BodyModeKind3D::Animated;
    float modeSeconds = 0.0f;
    // Os links seguem a animacao (o corpo guiado do padrao).
    bool guided = false;
    // A ajuda do impacto agora (0..1) e o tempo assentado no fisico.
    float assistLevel = 1.0f;
    float calmSeconds = 0.0f;
    // No chao, levantando, no carrinho; o cambaleio (0..1) e a perturbacao.
    bool fallen = false;
    bool gettingUp = false;
    bool tackling = false;
    float stagger = 0.0f;
    float perturbation = 0.0f;
    // A velocidade que o terreno a frente deixa (m/s; degrau, rampa).
    float terrainSpeedLimit = 1.0e9f;
    // Os contatos do ultimo tick e a ultima troca decidida pela tabela.
    ContactAssessment3D contacts;
    ModeDecision3D lastDecision;
    BodyModeKind3D lastDecisionFrom = BodyModeKind3D::Animated;
};

[[nodiscard]] AthleteTelemetry3D athleteTelemetry3D(const AthleteState3D& athlete);

// A impressao digital de um tick (a paridade, R1 da reescrita): o corpo
// publicado (posicao, orientacao e velocidades dos links), o que o controle
// mandou (os alvos dos motores, a guia da raiz e a forca dela), as forcas da
// ajuda (a pelve, os pes, as maos e a ajuda nos pes) e as decisoes (o modo e a
// ultima troca). Bit a bit: dois caminhos iguais dao os mesmos numeros, e o
// primeiro que difere diz onde eles se separaram.
struct AthleteTickDigest3D {
    std::uint64_t body = 0;
    std::uint64_t targets = 0;
    std::uint64_t assist = 0;
    std::uint64_t decisions = 0;
    friend bool operator==(const AthleteTickDigest3D&, const AthleteTickDigest3D&) = default;
};

[[nodiscard]] AthleteTickDigest3D athleteTickDigest3D(const AthleteState3D& athlete, const RagdollState3D& body);

// O codigo do modo na gravacao da sonda (o formato de antes: 0 animado, 1
// fisico, 2 volta, 3 ativo/ponte).
[[nodiscard]] int athleteDumpModeCode3D(BodyModeKind3D mode);

} // namespace MatterEngine
