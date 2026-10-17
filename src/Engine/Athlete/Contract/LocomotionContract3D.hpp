#pragma once

#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace MatterEngine {

// O que observe() fez: quantos numeros escreveu (o tamanho do layout, sempre)
// e quantos nao finitos virou zero (deveria ser sempre zero).
struct LocomotionObservation3D {
    std::size_t written = 0;
    std::size_t nonFinite = 0;
};

// O espelho de um layout (a simetria do treinador, tools/rl/matter_rl/
// symmetry.py): cada canal espelhado vem do canal source vezes scale mais
// offset.
struct LocomotionMirrorMap3D {
    std::vector<std::uint32_t> source;
    std::vector<float> scale;
    std::vector<float> offset;
};

struct LocomotionMirror3D {
    LocomotionMirrorMap3D observation;
    LocomotionMirrorMap3D action;
    // O pior alinhamento dos eixos espelhados (1: o eixo do outro lado e o
    // reflexo exato) e o eixo dele.
    float worstAxisAlignment = 1.0f;
    std::string worstAxis;
};

// Os tetos de uma fase (o curriculo do treino, nunca a rede), no valor
// fisico de cada saida: cortam o que a rede pede. Abrir um teto num portao
// nao muda o significado das acoes ja aprendidas (a escala da rede e a
// maxima, fixa). Os de fabrica sao os das fases 1 e 2 (o consenso da etapa A,
// vault 04_locomocao/plano_fases_1_2.md); full() abre tudo ate a escala.
struct LocomotionCaps3D {
    float touchdownMeters = 0.06f;
    float swingUpMeters = 0.05f;
    float swingDownMeters = 0.02f;
    float phaseLeadCycles = 0.05f;
    float recoveryUrgency = 0.5f;
    float cadenceOctaves = 0.2f;
    float strideOctaves = 0.2f;
    float leanRadians = 0.12f;
    float comMeters = 0.08f;
    float pelvisUpMeters = 0.02f;
    float pelvisDownMeters = 0.04f;
    float contactAcceleration = 1.0f;
    float jointRadians = 0.10f;
    float gainOctaves = 0.3f;

    [[nodiscard]] static LocomotionCaps3D full();
};

// As autoridades de cada cabeca (0..1: a rampa de entrada e o portao) e os
// tetos da fase: o curriculo do treino, nunca a rede.
struct LocomotionAuthority3D {
    float footwork = 0.0f;
    float balance = 0.0f;
    float pose = 0.0f;
    float gain = 0.0f;
    LocomotionCaps3D caps;
};

// O contrato da rede da locomocao fisica (o plano do RL, vault
// 04_locomocao/rl_sobre_a_locomocao.md, e o consenso da etapa A em
// plano_fases_1_2.md), versao 2. Feito do perfil do corpo; o mesmo no treino
// e no jogo.
//
// A entrada, no referencial do rumo da pelve (x a frente, y a esquerda, z
// para cima; deitado, o rumo vem do eixo de cima da pelve):
// - o corpo: a altura da pelve; cada link (a posicao em relacao a pelve, a
//   orientacao em 6D e as velocidades); as juntas atuadas (angulo e
//   velocidade); o contato de cada pe;
// - o comando: a velocidade pedida, o sprint, o olhar;
// - a referencia viavel (o padrao a sombra; zeros e a marca 0 sem ela);
// - o que o classico propos: por pe o pouso do balanco, e as partes-chave da
//   pose alvo dele;
// - o orcamento da ajuda e a forca que ela aplica;
// - o executor (v2, a entrada causal): o estado e o setor da marcha, o ciclo
//   e a fase de cada perna, o balanco, o contato do clipe, a espera do toque,
//   o ritmo e a passada, o planejador por pe (fase e dono), a urgencia de
//   recuperacao, o ponto de captura, a referencia da raiz, a ajuda efetiva e
//   o VMC das pernas (pedida, alocada, resto) com a margem na base;
// - as acoes das 3 decisoes anteriores.
// A saida (cada numero em -1..1, na escala maxima de cada canal; o teto da
// fase corta, a autoridade multiplica e o limite de taxa suaviza):
// - os pes: o pouso de cada um (x, y), a altura do balanco, o ritmo proprio
//   de cada perna, a urgencia de recuperacao de cada pe, a cadencia e o
//   comprimento da passada;
// - o equilibrio: a inclinacao (frente, lado), o centro de massa (x, y), a
//   altura da pelve, a aceleracao pelo contato (x, y);
// - a pose: a correcao de cada eixo atuado;
// - a rigidez: a escala dos ganhos por grupo (pernas, tronco, bracos).
class LocomotionContract3D {
public:
    static constexpr std::uint32_t Version = 2;
    static constexpr std::size_t HistoryDecisions = 3;
    // A decisao a 30 Hz (o seguimento a 120 Hz): a rampa entre decisoes.
    static constexpr int TicksPerDecision = 4;
    // As escalas maximas (o -1..1 da rede no corpo) e os limites de taxa
    // (por segundo), por canal.
    static constexpr float TouchdownMeters = 0.30f;
    static constexpr float SwingUpMeters = 0.10f;
    static constexpr float SwingDownMeters = 0.03f;
    static constexpr float PhaseLeadCycles = 0.10f;
    static constexpr float CadenceOctaves = 0.5f;
    static constexpr float StrideOctaves = 0.5f;
    static constexpr float LeanRadians = 0.30f;
    static constexpr float ComMeters = 0.20f;
    static constexpr float PelvisUpMeters = 0.05f;
    static constexpr float PelvisDownMeters = 0.15f;
    static constexpr float ContactAcceleration = 4.0f;
    static constexpr float JointRadians = 0.6f;
    static constexpr float GainOctaves = 1.0f;
    static constexpr float TouchdownRate = 1.5f;
    static constexpr float SwingRate = 0.5f;
    static constexpr float PhaseLeadRate = 0.5f;
    static constexpr float OctaveRate = 1.0f;
    static constexpr float LeanRate = 1.5f;
    static constexpr float ComRate = 0.6f;
    static constexpr float PelvisRate = 0.3f;
    static constexpr float ContactRate = 10.0f;
    static constexpr float JointRate = 2.0f;
    static constexpr float GainRate = 2.0f;

    explicit LocomotionContract3D(const RagdollProfile3D& profile);

    [[nodiscard]] const RLVectorLayout3D& observationLayout() const { return m_observation; }
    [[nodiscard]] const RLVectorLayout3D& actionLayout() const { return m_action; }
    [[nodiscard]] std::size_t observationSize() const { return m_observation.size(); }
    [[nodiscard]] std::size_t actionSize() const { return m_action.size(); }
    // A impressao do contrato (os nomes e unidades dos canais, a versao e o
    // corpo): vai no checkpoint; um checkpoint de outro contrato nao carrega.
    [[nodiscard]] std::uint64_t hash() const { return m_hash; }
    // Os eixos atuados (link * 3 + eixo), na ordem dos canais.
    [[nodiscard]] std::span<const std::uint32_t> actuatedAxes() const { return m_axes; }

    // O espelho pelo plano do corpo (y no referencial do rumo): os lados
    // trocam (Left/Right, esq/dir), os vetores trocam o y, as velocidades de
    // giro e o torque (pseudovetores) trocam o x e o z, as fases andam meio
    // ciclo e cada eixo de junta vai ao do outro lado com o sinal que os
    // quadros das juntas dao (o giro refletido por M = diag(1,-1,1) e o giro
    // pelo eixo M v ao contrario: s = -<M v, v'>).
    [[nodiscard]] LocomotionMirror3D mirror() const;

    // A entrada de um tick: o corpo publicado, o comando, a visao do controle
    // e o historico das acoes (o do atleta). `forecast` e o rascunho do plano
    // (um por atleta: nada e compartilhado entre threads). Sempre finita: um
    // numero nao finito vira zero, e a volta conta quantos foram (deveria ser
    // sempre zero; a paridade confere).
    LocomotionObservation3D observe(const RagdollState3D& body, const AthleteCommand3D& command,
        const AthleteControlView3D& view, std::span<const float> history, BodyForecast3D& forecast,
        std::span<float> out) const;

    // O valor fisico de cada canal para uma acao (-1..1): a escala, o teto da
    // fase e a autoridade da cabeca - antes do limite de taxa.
    [[nodiscard]] std::vector<float> physical(std::span<const float> action, const LocomotionAuthority3D& authority) const;
    // Os valores fisicos de cada canal nas correcoes do executor (uma cabeca
    // sem autoridade e sem valor fica desligada: o classico exato).
    [[nodiscard]] PhysicalCorrections3D corrections(std::span<const float> values,
        const LocomotionAuthority3D& authority) const;

    // Uma decisao nova (a cada TicksPerDecision ticks): guarda no historico do
    // atleta e comeca a rampa ate ela.
    void decide(AthleteState3D& athlete, std::span<const float> action) const;
    // Um tick: a rampa entre decisoes, o valor fisico e o limite de taxa de
    // cada canal (o aplicado mora no atleta), e as correcoes dele.
    void follow(AthleteState3D& athlete, const LocomotionAuthority3D& authority, float deltaTime) const;

private:
    // O que cada canal da saida e: o tipo, o pe (ou o eixo, ou o grupo).
    enum class Channel : std::uint8_t {
        TouchdownX, TouchdownY, SwingHeight, PhaseLead, Recovery, Cadence, Stride,
        LeanForward, LeanLateral, ComX, ComY, PelvisHeight, ContactX, ContactY, Joint, Gain
    };
    struct ChannelSpec {
        Channel kind = Channel::Joint;
        std::uint32_t index = 0;
    };
    const RagdollProfile3D* m_profile = nullptr;
    std::vector<ChannelSpec> m_specs;
    RLVectorLayout3D m_observation;
    RLVectorLayout3D m_action;
    std::vector<std::uint32_t> m_axes;
    std::array<int, 2> m_feet { -1, -1 };
    std::array<int, 2> m_toes { -1, -1 };
    // As partes-chave: pe esquerdo e direito, mao esquerda e direita, peito,
    // cabeca (-1 sem o link).
    std::array<int, 6> m_keyParts { -1, -1, -1, -1, -1, -1 };
    std::array<Vec3, 2> m_hipInBody {};
    std::uint64_t m_hash = 0;
};

} // namespace MatterEngine
