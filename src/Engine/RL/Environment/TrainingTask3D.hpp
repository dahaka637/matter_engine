#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace MatterEngine {

// Pesos da recompensa, por decisao (30 Hz). As parcelas positivas sao notas
// entre 0 e 1; as negativas, penalidades. Os nomes batem com os do arquivo da
// tarefa e com as colunas do treinador.
struct TrainingRewardWeights3D {
    float alive = 1.0f;
    float upright = 0.5f;
    float headHeight = 0.5f;
    float comOverSupport = 0.5f;
    float still = 0.3f;
    // O tronco girando (penalidade): a velocidade angular da pelve e do peito
    // ao quadrado. O "still" so ve o centro de massa, e girar o tronco quase
    // nao o move. Quadratica (e nao uma nota em sino) para ensinar tambem com
    // o ruido do treino, que faz o tronco girar a ~1,5 rad/s. Uma parcela nova
    // comeca desligada (as tarefas antigas nao mudam sem pedir).
    float trunkTurn = 0.0f;
    float footSlip = 0.5f;
    float footDrift = 1.0f;
    float effort = 0.1f;
    float actionRate = 0.2f;
    float jointLimit = 0.2f;
    float posePrior = 0.1f;
    // A postura pedida junta por junta (nota em sino do erro quadratico medio
    // das juntas, sigma ~10 graus): numa postura comandada os bracos tambem
    // contam. O posePrior, normalizado pelo curso, quase nao cobra um antebraco
    // 40 graus fora. Desligada se a tarefa nao pede.
    float poseMatch = 0.0f;
    // Andar (comando de velocidade): seguir a velocidade pedida no plano (no
    // referencial do rumo) e o giro pedido, notas em sino de sigma 0,5; e o
    // tempo de cada pe no ar (o passo de verdade, nao arrastar), so com
    // comando de andar. Desligadas nas tarefas de ficar de pe.
    float velocity = 0.0f;
    float yawRate = 0.0f;
    float feetAirTime = 0.0f;
    float fall = 10.0f;
};

inline constexpr std::array<const char*, 17> TrainingRewardComponentNames3D {
    "alive", "upright", "headHeight", "comOverSupport", "still", "trunkTurn", "footSlip", "footDrift", "effort",
    "actionRate", "jointLimit", "posePrior", "poseMatch", "velocity", "yawRate", "feetAirTime", "fall"
};

struct TrainingRange3D {
    float minimum = 0.0f;
    float maximum = 0.0f;
};

// Uma postura que a tarefa pede (pelo id no corpo) e a fatia dos episodios
// de treino com ela. Uma habilidade nova mantem uma fatia das antigas (o
// ensaio: aprende a nova sem esquecer).
struct TrainingPostureWeight3D {
    std::string id;
    float weight = 1.0f;
    // So parado nesta postura (o ensaio de uma habilidade de ficar de pe numa
    // tarefa de andar: pedidos de andar nela a fariam esquecer o parado).
    bool standingOnly = false;
};

// Um trecho do comando de velocidade numa bateria: a partir de seconds, a
// velocidade pedida (m/s, para frente e para a esquerda, no referencial do
// rumo) e o giro (rad/s, anti-horario).
struct TrainingCommandStep3D {
    float seconds = 0.0f;
    float forward = 0.0f;
    float lateral = 0.0f;
    float yawRate = 0.0f;
};

// Uma tarefa de treino (assets/rl/tasks/<id>.json): o que se pede ao corpo,
// como o episodio comeca e acaba e o quanto o mundo varia.
struct TrainingTask3D {
    std::string id;
    std::string displayName;
    std::filesystem::path definitionPath;
    std::filesystem::path bodyPath;
    std::uint32_t actionRepeat = 4;
    float episodeSeconds = 20.0f;
    std::string groundMaterial = "grass";
    // As posturas pedidas; sem a lista, so a pose base ("natural").
    std::vector<TrainingPostureWeight3D> postures { { "natural", 1.0f, false } };
    // A distancia entre os mundos de uma cena (e o chao cobre cada um com
    // metade dela em volta): andando, ninguem sai do chao nem tromba no vizinho.
    float worldSpacingMeters = 16.0f;
    TrainingRewardWeights3D reward;

    // O comando de velocidade (andar). Sem ele, so parado. Os maximos sao o
    // alvo de cada direcao; os limites de agora sao os do curriculo (o
    // treinador os sobe conforme a politica acompanha) e nunca passam do
    // maximo. Costas, lados e giro ficam prontos: basta um maximo acima de 0.
    struct Commands {
        TrainingRange3D resampleSeconds { 4.0f, 6.0f };
        // A fatia dos sorteios que pede ficar parado (comando zero).
        float standingFraction = 1.0f;
        float forward = 0.0f;
        float backward = 0.0f;
        float lateral = 0.0f;
        float yawRate = 0.0f;
        struct Limits {
            float forward = -1.0f;
            float backward = -1.0f;
            float lateral = -1.0f;
            float yawRate = -1.0f;
        } limits;
    } commands;

    struct Termination {
        float minimumHeadHeightFraction = 0.6f;
        float minimumPelvisHeightFraction = 0.5f;
        bool nonSupportContact = true;
        float maximumDistanceFromStart = 5.0f;  // m (o fim e truncado, nao queda)
    } termination;

    struct InitialState {
        float jointNoiseRadians = 0.05f;
        float jointVelocityNoise = 0.2f;
        float rootVelocityNoise = 0.1f;
        float rootAngularVelocityNoise = 0.1f;
        bool randomYaw = true;
    } initialState;

    struct Randomization {
        TrainingRange3D friction { 0.7f, 1.1f };
        TrainingRange3D mass { 0.95f, 1.05f };
        TrainingRange3D strength { 0.9f, 1.1f };
        float actionDelayProbability = 0.5f;
        float observationNoise = 1.0f;
    } randomization;

    struct Pushes {
        bool enabled = false;
        TrainingRange3D intervalSeconds { 2.0f, 4.0f };
        TrainingRange3D durationSeconds { 0.1f, 0.2f };
        TrainingRange3D forceNewtons { 50.0f, 150.0f };
        std::vector<std::string> links { "Pelvis", "UpperChest" };
    } pushes;

    std::uint64_t hash = 0;
};

[[nodiscard]] TrainingTask3D loadTrainingTask3D(const std::filesystem::path& definitionPath);

} // namespace MatterEngine
