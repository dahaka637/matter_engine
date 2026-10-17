#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace MatterEngine {

// As correcoes do RL dentro dos modulos de locomocao (o consenso da etapa A,
// vault 04_locomocao/plano_fases_1_2.md): uma cabeca por modulo - os pes, o
// equilibrio, a pose e a rigidez. Os valores chegam prontos do contrato (a
// escala, o teto da fase, a autoridade e o limite de taxa ja aplicados); uma
// cabeca desligada nem e lida, e sem nenhuma o classico passa exato (bit a
// bit). A ajuda NAO e uma cabeca: ela e do ambiente (o orcamento do jogo),
// nunca uma saida da rede.
struct FootCorrection3D {
    // Onde o pe no balanco pousa, somado ao do classico (m; no referencial do
    // rumo: x frente, y esquerda; o z nao e usado).
    Vec3 touchdownOffset;
    // A altura a mais do balanco no meio dele (m; vezes sin(pi x progresso):
    // o pouso nao muda).
    float swingHeight = 0.0f;
    // O avanco (+) ou atraso (-) proprio desta perna no ciclo da passada
    // (fracao do ciclo): quando o pe sai e pousa.
    float phaseLead = 0.0f;
    // A urgencia de um passo de recuperacao com este pe (0..1; o planejador
    // decide com histerese).
    float recoveryUrgency = 0.0f;
};

struct PhysicalCorrections3D {
    // As cabecas ligadas (a autoridade de cada uma acima de zero).
    bool footwork = false;
    bool balance = false;
    bool pose = false;
    bool gain = false;
    // O referencial do rumo (o do contrato: o da pelve), posto pelo passo do
    // atleta a cada tick: e nele que a rede ve o corpo e age.
    Quaternion frame;
    // Os pes.
    std::array<FootCorrection3D, 2> feet {};
    // O ritmo da passada (oitavas; antes da compensacao: passos mais rapidos e
    // mais curtos na mesma velocidade) e o comprimento dela (oitavas; depois
    // da compensacao: passos mais longos no mesmo ritmo).
    float cadenceOctaves = 0.0f;
    float strideOctaves = 0.0f;
    // O equilibrio: a inclinacao do tronco somada (rad: para a frente e para
    // o lado, os sinais do classico); o alvo do centro de massa (a referencia
    // da raiz deslocada, m, x e y no rumo); a altura da pelve (m); e a
    // aceleracao a mais pedida as pernas pelo contato (m/s2, x e y no rumo -
    // o chao limita: centro de pressao nos pes e atrito).
    float forwardLean = 0.0f;
    float lateralLean = 0.0f;
    Vec3 comOffset;
    float pelvisHeight = 0.0f;
    Vec3 contactAcceleration;
    // A pose: a correcao de cada eixo atuado (rad), indice link * 3 + eixo
    // (twist 0, swing1 1, swing2 2). Vazio: nenhuma.
    std::vector<float> jointResiduals;
    // A rigidez: a escala dos ganhos por grupo (0 pernas e pes, 1 tronco e
    // cabeca, 2 bracos e maos); 1 nao muda nada.
    std::array<float, 3> gainScale { 1.0f, 1.0f, 1.0f };

    // Alguma cabeca ligada (sem nenhuma, nem se passa adiante).
    [[nodiscard]] bool active() const { return footwork || balance || pose || gain; }
};

// O rumo do corpo pela pelve (rad): a frente dela no plano e a perpendicular
// do eixo do lado dela, pesadas pelo quanto cada uma esta deitada no plano.
// O referencial da rede (a entrada e a saida).
[[nodiscard]] float locomotionFrameYaw3D(const RagdollProfile3D& profile, Quaternion pelvisOrientation);

// O grupo de ganho de um link pelo nome (0 pernas e pes, 1 tronco e cabeca,
// 2 bracos e maos).
[[nodiscard]] int physicalGainGroup3D(const std::string& linkId);

// O alvo do pe no IK com a correcao dos pes. Apoiado: nenhuma (ela o
// arrastaria no chao). No balanco: a altura (sin(pi x progresso)) e, se o pe
// e da passada (nao do planejador, que leva o pouso no plano dele), o pouso -
// entrando ao longo do balanco (no primeiro tick inteira, ela saltava o alvo).
[[nodiscard]] Vec3 correctedFootGoal3D(Vec3 goal, bool planted, bool plannerOwned, float swingProgress,
    const PhysicalCorrections3D* corrections, std::size_t side);
// O pouso a mais de um passo do planejador (no mundo; zero sem correcao).
[[nodiscard]] Vec3 plannerLandingOffset3D(const PhysicalCorrections3D* corrections, std::size_t side);
// O fator do ritmo e o do comprimento da passada (1 sem correcao).
[[nodiscard]] float cadenceFactor3D(const PhysicalCorrections3D* corrections);
[[nodiscard]] float strideFactor3D(const PhysicalCorrections3D* corrections);
// O avanco proprio de uma perna (fracao do ciclo; a perna que espera o toque
// so adianta: atrasada, ela ficaria no ar).
[[nodiscard]] float legPhaseLead3D(const PhysicalCorrections3D* corrections, std::size_t side, bool waitingTouchdown);
// A urgencia de recuperacao pedida para um pe (0 sem correcao).
[[nodiscard]] float recoveryUrgency3D(const PhysicalCorrections3D* corrections, std::size_t side);

// A inclinacao com a correcao do equilibrio.
[[nodiscard]] float correctedForwardLean3D(float lean, const PhysicalCorrections3D* corrections);
[[nodiscard]] float correctedLateralLean3D(float lean, const PhysicalCorrections3D* corrections);
// O deslocamento do alvo do centro de massa (no mundo, no plano), a altura a
// mais da pelve e a aceleracao a mais pelo contato (no mundo, no plano).
[[nodiscard]] Vec3 comOffsetWorld3D(const PhysicalCorrections3D* corrections);
[[nodiscard]] float pelvisHeightOffset3D(const PhysicalCorrections3D* corrections);
[[nodiscard]] Vec3 contactAccelerationWorld3D(const PhysicalCorrections3D* corrections);

// Os alvos dos motores com as correcoes da pose (angulo) e da rigidez
// (escalas de rigidez, amortecimento e torque maximo).
void correctDriveTargets3D(std::vector<RagdollDriveTarget3D>& targets, const RagdollProfile3D& profile,
    const PhysicalCorrections3D* corrections);

} // namespace MatterEngine
