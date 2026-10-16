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

// As autoridades de cada cabeca (0..1): o curriculo do treino, nunca a rede.
struct LocomotionAuthority3D {
    float footwork = 0.0f;
    float balance = 0.0f;
    float pose = 0.0f;
    float gain = 0.0f;
};

// O contrato da rede da locomocao fisica (o plano do RL, vault
// 04_locomocao/rl_sobre_a_locomocao.md, secao 3), versao 1. Feito do perfil
// do corpo; o mesmo no treino e no jogo (o jogo o usa na L6).
//
// A entrada, no referencial do rumo da pelve (x a frente, y a esquerda, z
// para cima; deitado, o rumo vem do eixo de cima da pelve):
// - o corpo: a altura da pelve; cada link (a posicao em relacao a pelve, a
//   orientacao em 6D e as velocidades); as juntas atuadas (angulo e
//   velocidade); o contato de cada pe;
// - o comando: a velocidade pedida, o sprint, o olhar;
// - a referencia viavel (o padrao a sombra; zeros e a marca 0 sem ela): a
//   fase, por pe o apoio e a meta de contato, o plano a 0,1/0,25/0,5 s e o
//   erro das partes-chave (pes, maos, peito, cabeca) contra a pose de estilo;
// - o que o classico propos: por pe o pouso do balanco, e as partes-chave da
//   pose alvo dele;
// - o orcamento da ajuda e a forca que ela aplica;
// - as acoes das 3 decisoes anteriores.
// A saida, uma cabeca por modulo (cada numero em -1..1): o pouso de cada pe,
// a inclinacao do tronco, a correcao de cada eixo atuado e a escala dos
// ganhos por grupo (pernas, tronco, bracos).
class LocomotionContract3D {
public:
    static constexpr std::uint32_t Version = 1;
    static constexpr std::size_t HistoryDecisions = 3;
    // A decisao a 30 Hz (o seguimento a 120 Hz): a rampa entre decisoes.
    static constexpr int TicksPerDecision = 4;
    // As escalas da saida (o -1..1 da rede no corpo).
    static constexpr float TouchdownMeters = 0.15f;
    static constexpr float TouchdownHeightMeters = 0.05f;
    static constexpr float LeanRadians = 0.2f;
    static constexpr float JointRadians = 0.3f;
    static constexpr float GainOctaves = 1.0f;

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

    // A entrada de um tick: o corpo publicado, o comando, a visao do controle
    // e o historico das acoes (o do atleta). `forecast` e o rascunho do plano
    // (um por atleta: nada e compartilhado entre threads). Sempre finita: um
    // numero nao finito vira zero, e a volta conta quantos foram (deveria ser
    // sempre zero; a paridade confere).
    LocomotionObservation3D observe(const RagdollState3D& body, const AthleteCommand3D& command,
        const AthleteControlView3D& view, std::span<const float> history, BodyForecast3D& forecast,
        std::span<float> out) const;

    // A saida da rede nas correcoes do executor (cada cabeca vezes a sua
    // autoridade; autoridade zero e o classico exato).
    [[nodiscard]] PhysicalCorrections3D decode(std::span<const float> action,
        const LocomotionAuthority3D& authority) const;

    // Uma decisao nova (a cada TicksPerDecision ticks): guarda no historico do
    // atleta e comeca a rampa ate ela.
    void decide(AthleteState3D& athlete, std::span<const float> action) const;
    // Um tick da rampa: as correcoes do atleta no meio do caminho da acao de
    // antes ate a nova.
    void follow(AthleteState3D& athlete, const LocomotionAuthority3D& authority) const;

private:
    const RagdollProfile3D* m_profile = nullptr;
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
