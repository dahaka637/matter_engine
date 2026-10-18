#pragma once

#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/RL/Commands/CommandPilot3D.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace MatterEngine {

// Os pesos da nota do treino da locomocao (vault 04_locomocao/cartao_L1.md e
// plano_fases_1_2.md). Os termos positivos sao taxas por segundo; os custos,
// medidas normalizadas vezes o peso (por segundo); a queda, uma vez.
struct LocomotionRewardWeights3D {
    // A velocidade viavel (o pedido no limite de aceleracao do fisico na
    // ajuda do episodio), o rumo (o peito para a camera), o estilo (as
    // partes-chave contra a pose do padrao a sombra) e as metas de contato
    // (cada pouso perto do pouso da referencia, por pouso).
    float velocity = 0.40f;
    float facing = 0.10f;
    float style = 0.20f;
    float contact = 0.10f;
    float velocitySigma = 0.5f;
    float facingSigma = 0.4f;
    float styleSigma = 0.15f;
    float contactSigma = 0.15f;
    // Os custos: o pe apoiado andando (m/s), o trabalho dos motores (por 50
    // W/kg), os trancos (a acao mudando de uma decisao para a outra, media dos
    // quadrados), o uso da ajuda (a forca na pelve, fracao do peso), a postura
    // funcional (o angulo fora do envelope, rad) e o tamanho do residuo da
    // rede (o aplicado sobre o teto, media dos quadrados).
    float slip = 0.05f;
    float work = 0.02f;
    float smooth = 0.05f;
    float assist = 0.05f;
    float posture = 0.20f;
    float residual = 0.02f;
    // A quietude: o tremor do corpo parado (a pelve, o peito e a aceleracao dele)
    // quando o comando pede para ficar quieto; 0 (o padrao): sem o custo.
    float still = 0.0f;
    float fall = 1.0f;
};

// Um exercicio: o roteiro do piloto (o mesmo formato do Corpo v2), a fase do
// plano e o peso na mistura.
struct LocomotionDrill3D {
    std::string id;
    std::string displayName;
    int phase = 1;
    float weight = 1.0f;
    PilotScript3D pilot;
    // O minimo do exercicio no portao (as medidas por exercicio): a fatia
    // das quedas e o erro de velocidade (m/s, RMS). Negativo: sem minimo.
    float maximumFallFraction = -1.0f;
    float maximumVelocityError = -1.0f;
};

// A mistura dos exercicios pela dificuldade (o plano antigo: as fatias seguem
// a dificuldade, com um piso). Quem calcula e o treinador; aqui so os ajustes.
struct LocomotionDrillWeights3D {
    bool automatic = true;
    float gain = 2.0f;
    float maximumFactor = 3.0f;
    float floor = 0.08f;
};

// A tarefa do treino da locomocao (schema "matter-rl-locomocao-1").
struct LocomotionTask3D {
    std::string id;
    std::string displayName;
    std::filesystem::path definitionPath;
    std::uint64_t hash = 0;
    float episodeSeconds = 20.0f;
    // A ajuda do episodio (%, os quatro controles do laboratorio): parado,
    // andando, nos pes e no levantar. O "0%" do usuario e tudo em zero.
    float standingAssistPercent = 0.0f;
    float walkingAssistPercent = 0.0f;
    float feetAssistPercent = 0.0f;
    float getUpAssistPercent = 0.0f;
    // A virada brusca (o professor, DynamicTurnMove3D): ligada na tarefa, o
    // episodio a usa (menos o sorteio do dropout: a fracao dos episodios em que
    // ela fica desligada, para a rede nao depender dela) e a intensidade dela.
    // Desligada (o padrao): nada muda, nem o sorteio.
    struct TurnMove {
        bool enabled = false;
        float dropout = 0.3f;
        float amount = 1.0f;
    } turnMove;
    // Os tetos da fase (as autoridades vem do treinador: a rampa de entrada).
    LocomotionCaps3D caps;
    LocomotionRewardWeights3D reward;
    // O fim: a pelve abaixo disto (m, sobre o chao) e queda; alem desta
    // distancia da origem (m) o episodio acaba sem fracasso (o chao tem 60 m
    // para cada lado).
    float fallPelvisHeight = 0.45f;
    float boundaryMeters = 50.0f;
    LocomotionDrillWeights3D drillWeights;
    std::vector<LocomotionDrill3D> drills;
};

// Le uma tarefa (std::runtime_error com o motivo, se o formato estiver errado).
[[nodiscard]] LocomotionTask3D loadLocomotionTask3D(const std::filesystem::path& path);

} // namespace MatterEngine
