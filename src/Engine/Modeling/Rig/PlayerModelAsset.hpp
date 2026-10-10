#pragma once

#include "Engine/Modeling/Inspection/InspectionSession.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace MatterEngine::Modeling {

struct ReferenceImage {
    // frente, costas, direita, esquerda (as vistas da camera) ou detalhe.
    std::string view;
    std::filesystem::path path;
    std::string label;
};

// assets/characters/player_model/model.json: de onde vem o esqueleto (o
// personagem do jogo), as regioes do menu, a malha e as referencias.
struct PlayerModelManifest {
    std::string id;
    std::string displayName;
    std::filesystem::path directory;
    std::filesystem::path characterPath;
    std::filesystem::path regionsPath;
    // "montagem" (o modelo novo: as malhas base do corpo e da cabeca
    // montadas pela receita em "assembly"), "procedural" (o corpo antigo, das
    // receitas em parts) ou "nenhuma" (so os ossos).
    std::string meshSource = "nenhuma";
    // As receitas das pecas procedurais (partes/<peca>.json).
    std::vector<std::filesystem::path> parts;
    // A receita da montagem (montagem.json).
    std::filesystem::path assemblyPath;
    // Os ajustes do esqueleto do modelo ("skeleton": linkShift e
    // mirrorFingersFrom, ou "fit": o arquivo com o esqueleto medido).
    GameRigAdjustments skeleton;
    std::filesystem::path skeletonFitPath;
    std::vector<ReferenceImage> references;
};

[[nodiscard]] std::filesystem::path defaultPlayerModelManifest();
[[nodiscard]] std::optional<PlayerModelManifest> loadPlayerModelManifest(const std::filesystem::path& path,
    std::string* error = nullptr);

struct PlayerModelOptions {
    // Troca a origem da malha ("" = a do manifesto).
    std::string meshSource;
};

// O modelo pronto para inspecionar, com o personagem do jogo junto (o rig e
// as animacoes, para as poses).
struct LoadedPlayerModel {
    PlayerModelManifest manifest;
    std::shared_ptr<const RagdollCharacter3D> character;
    std::shared_ptr<const GameRig> rig;
    std::shared_ptr<const InspectionModel> model;
    // Os arquivos que a montagem leu alem dos do manifesto (as malhas base, o
    // desenho da pintura, as fontes, os SVG e as imagens dele).
    std::vector<std::filesystem::path> sourceFiles;
    // O tempo de cada etapa da carga (milissegundos): o personagem do jogo, o
    // esqueleto e cada etapa da montagem.
    std::vector<std::pair<std::string, double>> timings;
};

[[nodiscard]] std::optional<LoadedPlayerModel> loadPlayerModel(const std::filesystem::path& manifestPath,
    const PlayerModelOptions& options = {}, std::string* error = nullptr);

// Os arquivos de que o modelo depende e a escrita mais nova entre eles
// (a tela recarrega quando mudam).
[[nodiscard]] std::vector<std::filesystem::path> playerModelFiles(const PlayerModelManifest& manifest);
[[nodiscard]] std::filesystem::file_time_type newestWrite(const std::vector<std::filesystem::path>& files);

// As animacoes do jogo (assets/animations/clips/<id>.matteranim.json).
[[nodiscard]] std::vector<std::string> availableClipIds();
[[nodiscard]] std::optional<AnimationClip3D> loadCharacterClip(const std::string& clipId, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
