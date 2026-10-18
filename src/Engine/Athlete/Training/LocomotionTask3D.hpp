#pragma once

#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/RL/Commands/CommandPilot3D.hpp"

#include <array>
#include <cstddef>
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
    // A cauda larga da velocidade e do rumo (vault 04_locomocao/referencia/
    // pesquisas/02_recompensa_locomocao.md): a fracao do termo que vem de
    // exp(-|e|/sigmaCauda) (a forma do DReCon e do Siekmann: o gradiente nao
    // some longe do alvo). 0 (o padrao): so o nucleo de sempre.
    float velocityTail = 0.0f;
    float velocityTailSigma = 1.5f;
    float facingTail = 0.0f;
    float facingTailSigma = 1.0f;
    // Os trancos na unidade do teto (a saida da rede, +-1 = o teto de cada
    // canal) em vez da escala do contrato. false (o padrao): a de sempre.
    bool smoothCapUnits = false;
    // O pescoco: a folga (graus) e se ele e medido contra a pose da sombra (a
    // cabeca no referencial do peito, no corpo e na referencia) em vez de contra
    // o proprio peito. O padrao (10 graus contra o peito) e o de 18/10.
    float neckSlackDegrees = 10.0f;
    bool neckRelative = false;
    // O custo da ajuda pelo esforco inteiro da regua (vault 04_locomocao/
    // ajuda_auxiliar.md): alem das forcas, o torque de fora e o par de postura
    // (que no regime B segue a regua), como forca equivalente num braco de 1 m.
    // false (o padrao): so as forcas, o de sempre.
    bool assistIncludesTorque = false;
    // O contato por ciclo (a medida M0.3 de 18/10: pagar por pouso premiava a
    // cadencia, correlacao 0,54-0,72): cada pouso vale no maximo a fracao do
    // meio ciclo da referencia que passou desde o pouso anterior (pisar mais
    // rapido que a referencia nao rende mais). false (o padrao): por pouso.
    bool contactPerCycle = false;
    // A velocidade da nota suavizada em ~uma passada (0,25 s) em vez da instantanea
    // (a de 18/10: o centro de massa oscila dentro da passada, ~0,25 m/s no trote, e a
    // nota instantanea punia a oscilacao natural - um empurrao para o passo curto).
    // false (o padrao): a instantanea.
    bool velocitySmoothed = false;
};

// O curriculo de velocidade por celulas (vault .../pesquisas/04_curriculo_e_ajuda.md):
// a celula e o rumo do pedido em relacao a camera (8 setores de 45 graus,
// 0 = frente, 2 = esquerda, 4 = costas) e o modo do peito (0 camera parada, 1
// camera girando ou acompanhando). O teto de cada celula corta a velocidade
// que o piloto pede (so no treino; a avaliacao pede a faixa inteira). Quem abre
// os tetos e o treinador.
inline constexpr std::size_t LocomotionCommandBins3D = 8;
inline constexpr std::size_t LocomotionCommandCells3D = LocomotionCommandBins3D * 2;
[[nodiscard]] std::size_t locomotionCommandCell3D(float directionRelativeToCamera, bool cameraTurning);

struct LocomotionSpeedCurriculum3D {
    bool enabled = false;
    // A fronteira da celula (m/s): o rastreio que abre o teto so conta as decisoes
    // com o pedido ate esta distancia abaixo do teto (a faixa de cima; a revisao de
    // 18/10: os pedidos baixos rastreados nao podem abrir o teto alto).
    float frontier = 0.25f;
    // Por exercicio (o indice do exercicio na tarefa), o teto de cada celula
    // (m/s; <= 0: sem teto).
    std::vector<std::array<float, LocomotionCommandCells3D>> caps;
};

// Os comecos em estados guardados (RSI/STI): instantaneos do treino em
// cruzeiro que sobreviveram surviveSeconds; a fracao dos episodios de cada
// exercicio que comeca deles fica no exercicio (savedStartFraction). Nunca na
// avaliacao.
struct LocomotionSavedStarts3D {
    std::uint32_t capacity = 128;
    float surviveSeconds = 1.0f;
    // O instante so entra com o pedido ao menos isto (m/s) e no cruzeiro.
    float minimumSpeed = 1.0f;
};

// A ajuda sorteada por episodio (so no treino): a fracao dos episodios a 0% e,
// nos outros, o nivel uniforme entre o minimo e o maximo (%), nos controles
// parado, andando e pes (o levantar fica no da tarefa). Desligada: a da tarefa.
struct LocomotionAssistSchedule3D {
    bool enabled = false;
    float zeroFraction = 0.6f;
    float minimumPercent = 5.0f;
    float maximumPercent = 30.0f;
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
    // A fracao dos episodios deste exercicio que comeca de um estado guardado
    // (0: sempre parado, o de sempre).
    float savedStartFraction = 0.0f;
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
    // O chao (meia-largura, m): 60 m cobre o limite de 50 m; a corrida a 8 m/s
    // pede mais (o limite sobe junto, sempre com folga dentro do chao).
    float groundHalfExtentMeters = 60.0f;
    // A tabela de velocidades do modo padrao que o piloto e o atleta usam (m/s;
    // as da aba Velocidades do laboratorio): o trote em todos os rumos e a
    // corrida para a frente, de lado e de costas. Padrao: o do motor (4,5 / 7,8 /
    // 5,2 / 5,2); a meta do usuario (18/10) e 4,5 / 8 / 6 / 6.
    struct Speeds {
        float trot = 4.5f;
        float sprintForward = 7.8f;
        float sprintSide = 5.2f;
        float sprintBackward = 5.2f;
    } speeds;
    LocomotionDrillWeights3D drillWeights;
    LocomotionSpeedCurriculum3D speedCurriculum;
    LocomotionSavedStarts3D savedStarts;
    LocomotionAssistSchedule3D assistSchedule;
    std::vector<LocomotionDrill3D> drills;
};

// Le uma tarefa (std::runtime_error com o motivo, se o formato estiver errado).
[[nodiscard]] LocomotionTask3D loadLocomotionTask3D(const std::filesystem::path& path);

} // namespace MatterEngine
