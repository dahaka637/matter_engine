#pragma once

#include "Engine/Modeling/Inspection/InspectionSession.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"

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
    // "blender" (a malha modelada no Blender, em "blender"), "montagem" (as
    // malhas base do corpo e da cabeca montadas pela receita em "assembly"),
    // "procedural" (o corpo antigo, das receitas em parts) ou "nenhuma" (so
    // os ossos).
    std::string meshSource = "nenhuma";
    // As receitas das pecas procedurais (partes/<peca>.json).
    std::vector<std::filesystem::path> parts;
    // A receita da montagem (montagem.json).
    std::filesystem::path assemblyPath;
    // A malha modelada no Blender (mesh.blender: o JSON de troca), com
    // "source": "blender".
    std::filesystem::path blenderPath;
    // Os ajustes do esqueleto do modelo ("skeleton": linkShift e
    // mirrorFingersFrom, ou "fit": o arquivo com o esqueleto medido).
    GameRigAdjustments skeleton;
    std::filesystem::path skeletonFitPath;
    std::vector<ReferenceImage> references;
    // O sistema de aparencia: a pasta (assets/appearance) e o uniforme padrao.
    std::filesystem::path appearanceDirectory;
    std::string kit;
};

[[nodiscard]] std::filesystem::path defaultPlayerModelManifest();
[[nodiscard]] std::optional<PlayerModelManifest> loadPlayerModelManifest(const std::filesystem::path& path,
    std::string* error = nullptr);

struct PlayerModelOptions {
    // Troca a origem da malha ("" = a do manifesto).
    std::string meshSource;
    // O uniforme ("" = o do manifesto; "antigo" = a textura antiga).
    std::string kit;
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

// O modelo carregado (no corpo de hoje) num corpo (26/10): o personagem do
// jogo vira o do corpo (bodyRagdollCharacter3D, com o tonus na pele); cada
// vertice da modelagem anda o tonus do vertice da pele que saiu dele e, com o
// esqueleto e o rig, escala em volta do chao (o espaco do jogo, a pelve na
// origem, escala em volta dela: a pele do corpo deforma igual a modelagem). No
// corpo de hoje nada muda. Lanca se a malha nao for a do personagem exportado
// (outra --malha).
void applyPlayerBody(LoadedPlayerModel& loaded, const PlayerBody3D& body);

// Os arquivos de que o modelo depende e a escrita mais nova entre eles
// (a tela recarrega quando mudam).
[[nodiscard]] std::vector<std::filesystem::path> playerModelFiles(const PlayerModelManifest& manifest);
[[nodiscard]] std::filesystem::file_time_type newestWrite(const std::vector<std::filesystem::path>& files);

// As animacoes do jogo (assets/animations/clips/<id>.matteranim.json).
[[nodiscard]] std::vector<std::string> availableClipIds();
[[nodiscard]] std::optional<AnimationClip3D> loadCharacterClip(const std::string& clipId, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
