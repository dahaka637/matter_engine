#pragma once

#include "CommandLine.hpp"

#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace MatterEngine::ModelInspector {

// O modelo que os comandos usam, carregado pelas opcoes comuns:
//   --modelo <model.json>   (padrao: assets/characters/player_model/model.json)
//   --malha montagem|procedural|nenhuma  (troca a origem da malha)
//   --pose <animacao>@<s>   (a malha e os ossos numa pose; ex.: sprint@0.3)
//   --saida-dir <pasta>     (onde gravar; padrao ./inspecao)
struct ModelContext {
    Modeling::LoadedPlayerModel loaded;
    std::filesystem::path outputDirectory;
    std::optional<Modeling::SkeletonPose> pose;
    std::vector<Vec3> posedPositions;
    std::string poseLabel;

    [[nodiscard]] const Modeling::InspectionModel& model() const { return *loaded.model; }
    // A cena (com a pose, se houver).
    [[nodiscard]] Modeling::InspectionScene scene() const;
    // A regiao pelo id (ou -1 com a mensagem).
    [[nodiscard]] int region(const std::string& id) const;
    // Caminho de saida: --saida, ou a pasta de saida + o nome padrao.
    [[nodiscard]] std::filesystem::path output(const CommandLine& line, const std::string& defaultName) const;
};

[[nodiscard]] std::optional<ModelContext> loadModelContext(const CommandLine& line, std::string* error);

// "sprint@0.3" -> a pose do esqueleto (clip do jogo no instante).
[[nodiscard]] std::optional<Modeling::SkeletonPose> poseFromSpec(const Modeling::LoadedPlayerModel& loaded,
    const std::string& spec, std::string* error);

} // namespace MatterEngine::ModelInspector
