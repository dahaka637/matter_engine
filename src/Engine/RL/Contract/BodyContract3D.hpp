#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyCommand3D.hpp"
#include "Engine/RL/Contract/PlayerAttributes3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <string_view>
#include <vector>

namespace MatterEngine {

// O alvo de velocidade dos motores (plano_claude_rl.md secoes 1.3 e 5.6; o
// Experimento A escolhe). O alvo de posicao anda em linha reta de uma decisao
// a outra nos passos de fisica; a velocidade pedida e:
//  - Zero: nenhuma (o amortecimento freia todo movimento da junta);
//  - SampledSlope: a da reta da acao sorteada (o ruido da exploracao vira
//    velocidade pedida);
//  - MeanSlope: a da reta da acao media da politica (o ruido mexe so na
//    posicao; no jogo a acao e a media e as duas coincidem).
enum class BodyVelocityTarget3D : std::uint8_t { Zero, SampledSlope, MeanSlope };

struct BodyContractSettings3D {
    // A memoria curta: quantas decisoes anteriores entram na entrada.
    std::uint32_t historyDecisions = 3;
    // Os passos de fisica de uma decisao e a duracao dela (s).
    std::uint32_t substepsPerDecision = 4;
    float decisionSeconds = 4.0f / 120.0f;
    BodyVelocityTarget3D velocityTarget = BodyVelocityTarget3D::MeanSlope;
    // A fracao da reta que vira velocidade pedida e o teto dela (rad/s).
    float velocityFraction = 1.0f;
    float maximumTargetVelocity = 20.0f;
};

// O estado do contrato de um jogador: a memoria curta, a acao anterior, a reta
// dos alvos e o resumo dos contatos da decisao.
struct BodyContractState3D {
    std::vector<float> previousAction;
    // historyDecisions quadros, o mais novo primeiro.
    std::vector<float> history;
    // O quadro da ultima observacao (entra na memoria na decisao seguinte).
    std::vector<float> lastFrame;
    std::vector<float> rampFrom;
    std::vector<float> rampTo;
    std::vector<float> velocityTarget;
    std::vector<float> previousMeanTarget;
    // O contato de cada lado (0 esquerdo, 1 direito; o pe e a ponta) nos
    // passos de fisica desde a ultima observacao.
    std::array<float, 2> contactSteps {};
    std::array<float, 2> toeSteps {};
    std::array<float, 2> forceSum {};
    std::array<float, 2> forcePeak {};
    std::array<bool, 2> touchdown {};
    std::array<bool, 2> liftoff {};
    std::array<bool, 2> lastContact {};
    std::uint32_t contactSubsteps = 0;
    // A primeira decisao depois do recomeco nao pede velocidade (a reta da
    // media ainda nao tem de onde partir).
    bool started = false;
};

// O contrato do Corpo v2: o que a rede ve e o que ela faz. O mesmo codigo no
// treino, na tela e no jogo. A entrada so tem o que o jogo tem: o proprio
// corpo (com a memoria curta), os comandos e os dados do jogador; nunca clipe,
// fase de clipe, destino ou nome de exercicio. A acao e o alvo absoluto de
// cada eixo atuado (a postura base mais a escala vezes a acao); os eixos
// travados ficam na postura base.
class BodyContract3D {
public:
    explicit BodyContract3D(const RLBody3D& body, BodyContractSettings3D settings = {});

    [[nodiscard]] const RLBody3D& body() const { return m_body; }
    [[nodiscard]] const BodyContractSettings3D& settings() const { return m_settings; }
    [[nodiscard]] const RLVectorLayout3D& observationLayout() const { return m_observation; }
    [[nodiscard]] const RLVectorLayout3D& actionLayout() const { return m_action; }
    [[nodiscard]] std::size_t observationSize() const { return m_observation.size(); }
    [[nodiscard]] std::size_t actionSize() const { return m_body.dofs.size(); }

    // Um jogador novo ou um recomeco: a memoria e a reta partem do corpo de agora.
    void reset(BodyContractState3D& state, const RagdollState3D& ragdoll) const;
    // Depois de cada passo de fisica: o contato de cada lado (bodyWeight em N).
    void accumulateContacts(BodyContractState3D& state, const RagdollState3D& ragdoll, float bodyWeight,
        float fixedStep) const;
    // A entrada da rede (observationSize floats em out). noise > 0 soma o ruido
    // de observacao do treino (rng obrigatorio); o jogo passa 0.
    void writeObservation(const RagdollState3D& ragdoll, const BodyCommand3D& command, const PlayerAttributes3D& player,
        BodyContractState3D& state, float* out, float noise = 0.0f, std::mt19937_64* rng = nullptr) const;
    // A decisao: a acao executada (sorteada no treino) e a media da politica.
    // Guarda a reta dos alvos, a velocidade pedida e a memoria.
    void beginDecision(BodyContractState3D& state, std::span<const float> action, std::span<const float> mean) const;
    // Os alvos dos motores no passo de fisica substep (0 a substepsPerDecision-1)
    // da decisao: os eixos atuados pela reta e os travados na postura base.
    // hiddenStrength: a variacao de forca do treino que a rede nao ve.
    void writeTargets(const BodyContractState3D& state, std::uint32_t substep, const PlayerAttributes3D& player,
        std::vector<RagdollDriveTarget3D>& out, float hiddenStrength = 1.0f) const;
    // Os alvos num ponto u (0 a 1) da reta da decisao: writeTargets e o fim de
    // cada passo de fisica; um u menor e o comando atrasado.
    void writeTargetsAt(const BodyContractState3D& state, float u, const PlayerAttributes3D& player,
        std::vector<RagdollDriveTarget3D>& out, float hiddenStrength = 1.0f) const;

    // O grupo de forca de um link (pelo id).
    [[nodiscard]] static StrengthGroup3D strengthGroup(std::string_view linkId);

private:
    [[nodiscard]] std::size_t frameSize() const;
    [[nodiscard]] float targetOf(std::size_t dof, float action) const;

    RLBody3D m_body;
    BodyContractSettings3D m_settings;
    RLVectorLayout3D m_observation;
    RLVectorLayout3D m_action;
    std::array<std::uint32_t, 5> m_keyBodies {};
    std::vector<StrengthGroup3D> m_dofGroup;
    std::vector<StrengthGroup3D> m_lockedGroup;
};

} // namespace MatterEngine
