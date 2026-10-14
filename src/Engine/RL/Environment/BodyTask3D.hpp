#pragma once

#include "Engine/RL/Commands/CommandPilot3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"
#include "Engine/RL/Contract/PlayerAttributes3D.hpp"
#include "Engine/RL/Rewards/BodyReward3D.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace MatterEngine {

// A tarefa do Corpo v2 (o esquema "matter-rl-task-2", plano_claude_rl.md
// secao 15.3): o corpo, a decisao do contrato, o jogador, os tetos, a nota, o
// comeco, o fim, a variacao do mundo e os exercicios como roteiros do piloto.
// Nada de clipe, alvo na entrada ou referencia: isso era o contrato 1.

// Um exercicio: o roteiro do piloto e os pesos da tarefa nele. O peso e a
// fatia dos mundos (as cotas por decisao: cada mundo fica no seu exercicio).
struct BodyDrill3D {
    std::string id;
    std::string displayName;
    float weight = 1.0f;
    PilotScript3D pilot;
    BodyTaskWeights3D task;
    // A duracao do episodio nele (s; 0 = a da tarefa).
    float episodeSeconds = 0.0f;
};

struct BodyTask3D {
    std::string id;
    std::string displayName;
    std::filesystem::path definitionPath;
    std::filesystem::path bodyPath;
    // FNV-1a de 64 bits do arquivo.
    std::uint64_t hash = 0;
    BodyContractSettings3D contract;
    float episodeSeconds = 20.0f;
    std::string groundMaterial = "grass";
    // A distancia entre os mundos de uma cena (m).
    float worldSpacingMeters = 250.0f;
    PlayerAttributes3D player;
    ControlSettings3D control;
    BodyRewardSettings3D reward;
    struct Termination {
        float minimumHeadHeightFraction = 0.6f;
        float minimumPelvisHeightFraction = 0.5f;
        // Um link que nao e apoio (os pes e as pontas) no chao e queda.
        bool nonSupportContact = true;
        // Longe demais do comeco: truncado, nao queda (m).
        float maximumDistanceFromStart = 120.0f;
    } termination;
    // O acervo do estilo (style.dataset no arquivo): o juiz do treinador e os
    // comecos pelo acervo usam. Vazio = sem acervo.
    std::filesystem::path styleDataset;
    struct InitialState {
        // A fatia dos comecos pelo acervo (a muleta do plano, secao 10.1): um
        // quadro de um clipe de treino com a pose e as velocidades dele, e o
        // primeiro pedido seguindo o movimento. So quadros de pe: a pelve a
        // pelo menos referenceMinimumPelvis da altura de pe da postura.
        float referenceFraction = 0.0f;
        float referenceMinimumPelvis = 0.6f;
        // De pe: a postura do corpo (o id), com ruido.
        std::string posture = "alerta";
        float jointNoiseRadians = 0.03f;
        float jointVelocityNoise = 0.2f;
        float rootVelocityNoise = 0.1f;
        float rootAngularVelocityNoise = 0.1f;
        bool randomYaw = true;
    } initialState;
    // A variacao do mundo que a rede nao ve (so no treino).
    struct Randomization {
        PilotRange3D friction { 0.7f, 1.1f };
        PilotRange3D mass { 0.95f, 1.05f };
        PilotRange3D strength { 0.9f, 1.1f };
        // A chance de um episodio com o comando atrasado um passo de fisica.
        float actionDelayProbability = 0.5f;
        // A escala do ruido de observacao do contrato.
        float observationNoise = 1.0f;
    } randomization;
    std::vector<BodyDrill3D> drills;
};

[[nodiscard]] BodyTask3D loadBodyTask3D(const std::filesystem::path& path);

// O indice de um exercicio pelo id (-1 se nao houver).
[[nodiscard]] int findBodyDrill3D(const BodyTask3D& task, const std::string& id);

} // namespace MatterEngine
