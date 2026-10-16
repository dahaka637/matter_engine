#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace MatterEngine {

// As correcoes do RL nas costuras do executor fisico (vault
// 04_locomocao/rl_sobre_a_locomocao.md, secao 3): uma cabeca por modulo, cada
// uma multiplicada pela sua autoridade (0..1, o curriculo). Autoridade zero
// (o padrao) e exatamente o classico. A ajuda NAO e uma cabeca: ela e do
// ambiente (o orcamento do jogo), nunca uma saida da rede.
struct FootCorrection3D {
    // Onde o pe no balanco pousa, somado ao alvo do classico (m; no
    // referencial do rumo: x frente, y esquerda, z cima).
    Vec3 touchdownOffset;
};

struct PhysicalCorrections3D {
    // As autoridades por cabeca (0..1).
    float footworkAuthority = 0.0f;
    float balanceAuthority = 0.0f;
    float poseAuthority = 0.0f;
    float gainAuthority = 0.0f;
    // Jogo de pes.
    std::array<FootCorrection3D, 2> feet {};
    // Equilibrio: a inclinacao do tronco somada (rad: para a frente e para o
    // lado, os sinais do classico).
    float forwardLean = 0.0f;
    float lateralLean = 0.0f;
    // Pose: a correcao de cada eixo atuado (rad), indice link * 3 + eixo
    // (twist 0, swing1 1, swing2 2). Vazio: nenhuma.
    std::vector<float> jointResiduals;
    // Rigidez: a escala dos ganhos por grupo (0 pernas e pes, 1 tronco e
    // cabeca, 2 bracos e maos); 1 nao muda nada.
    std::array<float, 3> gainScale { 1.0f, 1.0f, 1.0f };

    // Alguma cabeca com autoridade (sem nenhuma, nem se passa adiante).
    [[nodiscard]] bool active() const;
};

// O grupo de ganho de um link pelo nome (0 pernas e pes, 1 tronco e cabeca,
// 2 bracos e maos).
[[nodiscard]] int physicalGainGroup3D(const std::string& linkId);

// O alvo do pe no IK com a correcao do jogo de pes: so no pe no balanco (no
// apoiado ela o arrastaria no chao).
[[nodiscard]] Vec3 correctedFootGoal3D(Vec3 goal, bool planted, Quaternion heading,
    const PhysicalCorrections3D* corrections, std::size_t side);

// A inclinacao com a correcao do equilibrio.
[[nodiscard]] float correctedForwardLean3D(float lean, const PhysicalCorrections3D* corrections);
[[nodiscard]] float correctedLateralLean3D(float lean, const PhysicalCorrections3D* corrections);

// Os alvos dos motores com as correcoes da pose (angulo) e da rigidez
// (escalas de rigidez, amortecimento e torque maximo).
void correctDriveTargets3D(std::vector<RagdollDriveTarget3D>& targets, const RagdollProfile3D& profile,
    const PhysicalCorrections3D* corrections);

} // namespace MatterEngine
