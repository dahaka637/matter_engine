#pragma once

#include "Engine/Athlete/AthleteState3D.hpp"

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
    // Os contatos do ultimo tick e a ultima troca decidida pela tabela.
    ContactAssessment3D contacts;
    ModeDecision3D lastDecision;
    BodyModeKind3D lastDecisionFrom = BodyModeKind3D::Animated;
};

[[nodiscard]] AthleteTelemetry3D athleteTelemetry3D(const AthleteState3D& athlete);

// O codigo do modo na gravacao da sonda (o formato de antes: 0 animado, 1
// fisico, 2 volta, 3 ativo/ponte).
[[nodiscard]] int athleteDumpModeCode3D(BodyModeKind3D mode);

} // namespace MatterEngine
