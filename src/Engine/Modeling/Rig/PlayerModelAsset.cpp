#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include "Engine/Modeling/Assembly/PlayerAssembly.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Parts/PartLibrary.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>

#ifndef MATTERENGINE_ASSETS_DIR
#define MATTERENGINE_ASSETS_DIR "assets"
#endif

namespace MatterEngine::Modeling {

namespace {

constexpr const char* ClipSuffix = ".matteranim.json";

std::filesystem::path clipsDirectory() {
    return std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "animations" / "clips";
}

// O personagem do jogo (o perfil fisico, os ossos e a malha antiga, 15 MB de
// json): lido uma vez por processo e de novo so quando um arquivo da pasta
// dele muda (o manifesto aponta para o perfil e para a pele ao lado).
std::shared_ptr<const RagdollCharacter3D> gameCharacter(const std::filesystem::path& manifest) {
    static std::mutex mutex;
    static std::map<std::string, std::pair<std::filesystem::file_time_type, std::shared_ptr<const RagdollCharacter3D>>> cache;
    std::filesystem::file_time_type newest {};
    std::error_code ignored;
    for (const auto& entry : std::filesystem::directory_iterator(manifest.parent_path(), ignored))
        if (entry.is_regular_file(ignored)) newest = std::max(newest, entry.last_write_time(ignored));
    const std::lock_guard lock(mutex);
    auto& [stamp, character] = cache[manifest.string()];
    if (!character || stamp != newest) {
        character = std::make_shared<const RagdollCharacter3D>(loadRagdollCharacter3D(manifest.string()));
        stamp = newest;
    }
    return character;
}

} // namespace

std::filesystem::path defaultPlayerModelManifest() {
    return std::filesystem::path(MATTERENGINE_ASSETS_DIR) / "characters" / "player_model" / "model.json";
}

std::optional<PlayerModelManifest> loadPlayerModelManifest(const std::filesystem::path& path, std::string* error) {
    try {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("nao abriu " + path.string());
        const nlohmann::json document = nlohmann::json::parse(file);
        PlayerModelManifest manifest;
        manifest.directory = path.parent_path();
        manifest.id = document.value("id", std::string("jogador"));
        manifest.displayName = document.value("displayName", manifest.id);
        manifest.characterPath = (manifest.directory / document.at("character").get<std::string>()).lexically_normal();
        manifest.regionsPath = (manifest.directory / document.value("regions", std::string("regions.json"))).lexically_normal();
        if (document.contains("mesh")) {
            const nlohmann::json& mesh = document.at("mesh");
            manifest.meshSource = mesh.value("source", manifest.meshSource);
            if (mesh.contains("parts"))
                for (const nlohmann::json& part : mesh.at("parts"))
                    manifest.parts.push_back((manifest.directory / part.get<std::string>()).lexically_normal());
            if (mesh.contains("assembly"))
                manifest.assemblyPath = (manifest.directory / mesh.at("assembly").get<std::string>()).lexically_normal();
        }
        if (document.contains("skeleton")) {
            const nlohmann::json& skeleton = document.at("skeleton");
            if (skeleton.contains("linkShift"))
                for (const auto& [link, offset] : skeleton.at("linkShift").items()) {
                    const std::vector<float> value = offset.get<std::vector<float>>();
                    if (value.size() != 3) throw std::runtime_error("skeleton.linkShift." + link + ": [x, y, z]");
                    manifest.skeleton.linkShifts.push_back({ link, { value[0], value[1], value[2] } });
                }
            manifest.skeleton.mirrorFingersFrom = skeleton.value("mirrorFingersFrom", std::string());
            if (skeleton.contains("ground")) manifest.skeleton.ground = skeleton.at("ground").get<float>();
            if (skeleton.contains("fit")) {
                manifest.skeletonFitPath = (manifest.directory / skeleton.at("fit").get<std::string>()).lexically_normal();
                std::ifstream fitFile(manifest.skeletonFitPath);
                if (!fitFile) throw std::runtime_error("nao abriu " + manifest.skeletonFitPath.string());
                const nlohmann::json fit = nlohmann::json::parse(fitFile);
                for (const auto& [name, value] : fit.at("joints").items()) {
                    const std::vector<float> xyz = value.get<std::vector<float>>();
                    if (xyz.size() != 3) throw std::runtime_error("esqueleto: " + name + " precisa de [x, y, altura]");
                    if (name.rfind("Left", 0) == 0) throw std::runtime_error("esqueleto: so o lado direito (o esquerdo e o espelho): " + name);
                    manifest.skeleton.joints.push_back({ name, { xyz[0], xyz[1], xyz[2] } });
                }
                if (!manifest.skeleton.linkShifts.empty())
                    throw std::runtime_error("skeleton: use linkShift ou fit, nao os dois");
            }
            const std::string& from = manifest.skeleton.mirrorFingersFrom;
            if (!from.empty() && from != "Right" && from != "Left")
                throw std::runtime_error("skeleton.mirrorFingersFrom: \"Right\" ou \"Left\"");
        }
        if (document.contains("references"))
            for (const nlohmann::json& entry : document.at("references"))
                manifest.references.push_back({ entry.value("view", std::string("detalhe")),
                    (manifest.directory / entry.at("image").get<std::string>()).lexically_normal(),
                    entry.value("label", std::string()) });
        return manifest;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("manifesto do jogador: ") + exception.what();
        return std::nullopt;
    }
}

std::optional<LoadedPlayerModel> loadPlayerModel(const std::filesystem::path& manifestPath,
    const PlayerModelOptions& options, std::string* error) {
    const std::optional<PlayerModelManifest> manifest = loadPlayerModelManifest(manifestPath, error);
    if (!manifest) return std::nullopt;
    LoadedPlayerModel loaded;
    loaded.manifest = *manifest;
    StageClock clock(loaded.timings);
    try {
        loaded.character = gameCharacter(manifest->characterPath);
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("personagem do jogo: ") + exception.what();
        return std::nullopt;
    }
    // O modelo montado: a receita dele (o pescoco mais curto leva o esqueleto
    // medido junto: as juntas sao as do corpo base).
    std::optional<AssemblyRecipe> recipe;
    if (!manifest->assemblyPath.empty()) {
        std::ifstream file(manifest->assemblyPath);
        std::stringstream text;
        text << file.rdbuf();
        recipe = parseAssemblyRecipe(text.str(), error);
        if (!file || !recipe) {
            if (error != nullptr && file) *error = "montagem: " + *error;
            else if (error != nullptr) *error = "montagem: nao abriu " + manifest->assemblyPath.string();
            return std::nullopt;
        }
    }
    GameRigAdjustments adjustments = manifest->skeleton;
    if (recipe)
        for (auto& [name, position] : adjustments.joints) {
            position.z = recipe->neck.liftedHeight(position.x, position.z);
            // A cabeca anda "headShift" sobre o pescoco do corpo: a junta dela
            // (o pivo) e o alto andam junto.
            if (name == "Head" || name == "HeadTop") position.x += recipe->headShift;
        }
    loaded.rig = std::make_shared<const GameRig>(buildGameRig(*loaded.character, adjustments));
    if (!loaded.rig->fitMissing.empty()) {
        if (error != nullptr) {
            *error = "esqueleto: faltam as juntas";
            for (const std::string& name : loaded.rig->fitMissing) *error += " " + name;
        }
        return std::nullopt;
    }
    std::optional<BodyRegions> regions = BodyRegions::load(manifest->regionsPath.string(), error);
    if (!regions) return std::nullopt;
    clock.mark("personagem do jogo e esqueleto");

    auto model = std::make_shared<InspectionModel>();
    model->skeleton = loaded.rig->skeleton;
    model->regions = std::move(*regions);
    model->jointRegion = model->regions.assign(model->skeleton);
    const std::string meshSource = options.meshSource.empty() ? manifest->meshSource : options.meshSource;
    if (meshSource == "montagem") {
        if (!recipe) {
            if (error != nullptr) *error = "montagem: o manifesto nao tem a receita (mesh.assembly)";
            return std::nullopt;
        }
        AssemblyResult assembled = assemblePlayer(*recipe, manifest->assemblyPath.parent_path(), model->skeleton);
        if (!assembled.ok) {
            if (error != nullptr) *error = "montagem: " + assembled.error;
            return std::nullopt;
        }
        model->mesh = std::move(assembled.mesh);
        loaded.sourceFiles = std::move(assembled.files);
        loaded.timings.insert(loaded.timings.end(), assembled.timings.begin(), assembled.timings.end());
        model->source = manifest->displayName + " (montagem, " + std::to_string(model->mesh.triangles.size()) + " triangulos";
        if (!model->mesh.textures.empty())
            model->source += ", textura " + std::to_string(model->mesh.textures.front().image.width()) + " px, "
                + std::to_string(static_cast<int>(std::lround(assembled.texelsPerMeter))) + " texels/m";
        model->source += ")";
    } else if (meshSource == "procedural") {
        std::optional<Mesh> mesh = buildProceduralParts(manifest->parts, model->skeleton, error);
        if (!mesh) return std::nullopt;
        model->mesh = std::move(*mesh);
        model->source = manifest->displayName + " (" + std::to_string(manifest->parts.size()) + " peca(s), "
            + std::to_string(model->mesh.triangles.size()) + " triangulos)";
    } else {
        model->source = manifest->displayName + " (so ossos)";
    }
    loaded.model = std::move(model);
    return loaded;
}

std::vector<std::filesystem::path> playerModelFiles(const PlayerModelManifest& manifest) {
    std::vector<std::filesystem::path> files { manifest.directory / "model.json", manifest.regionsPath,
        manifest.characterPath };
    if (!manifest.skeletonFitPath.empty()) files.push_back(manifest.skeletonFitPath);
    files.insert(files.end(), manifest.parts.begin(), manifest.parts.end());
    if (!manifest.assemblyPath.empty()) files.push_back(manifest.assemblyPath);
    return files;
}

std::filesystem::file_time_type newestWrite(const std::vector<std::filesystem::path>& files) {
    std::filesystem::file_time_type newest {};
    for (const std::filesystem::path& file : files) {
        std::error_code ignored;
        const auto time = std::filesystem::last_write_time(file, ignored);
        if (!ignored && time > newest) newest = time;
    }
    return newest;
}

std::vector<std::string> availableClipIds() {
    std::vector<std::string> ids;
    std::error_code ignored;
    for (const auto& entry : std::filesystem::directory_iterator(clipsDirectory(), ignored)) {
        const std::string name = entry.path().filename().string();
        if (name.size() > std::string_view(ClipSuffix).size() && name.ends_with(ClipSuffix))
            ids.push_back(name.substr(0, name.size() - std::string_view(ClipSuffix).size()));
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::optional<AnimationClip3D> loadCharacterClip(const std::string& clipId, std::string* error) {
    try {
        return loadAnimationClip3D((clipsDirectory() / (clipId + ClipSuffix)).string());
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = "animacao " + clipId + ": " + exception.what();
        return std::nullopt;
    }
}

} // namespace MatterEngine::Modeling
