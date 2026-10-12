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
    float footSlip = 0.5f;
    float footDrift = 1.0f;
    float effort = 0.1f;
    float actionRate = 0.2f;
    float jointLimit = 0.2f;
    float posePrior = 0.1f;
    float fall = 10.0f;
};

inline constexpr std::array<const char*, 12> TrainingRewardComponentNames3D {
    "alive", "upright", "headHeight", "comOverSupport", "still", "footSlip", "footDrift", "effort", "actionRate",
    "jointLimit", "posePrior", "fall"
};

struct TrainingRange3D {
    float minimum = 0.0f;
    float maximum = 0.0f;
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
    TrainingRewardWeights3D reward;

    struct Termination {
        float minimumHeadHeightFraction = 0.6f;
        float minimumPelvisHeightFraction = 0.5f;
        bool nonSupportContact = true;
        float maximumDistanceFromStart = 5.0f;
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
