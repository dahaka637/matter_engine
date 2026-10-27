#include "Commands.hpp"
#include "ModelContext.hpp"

#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <cstdio>
#include <iostream>

// exportar-blender: o jogador montado para o Blender (22/10/2026: a modelagem
// das juntas - a topologia do ombro e do quadril, os pesos - passa a ser feita
// la). Um JSON de troca exato (tools/blender/importar_troca.py monta o .blend):
// o esqueleto do jogo (as juntas, os pais e as regras dos auxiliares), a malha
// com as partes, os materiais, o mapa por canto e os pesos por osso, e as
// texturas em PNG. No espaco do modelo: x frente, y esquerda, z cima (metros).
namespace MatterEngine::ModelInspector {

using namespace Modeling;

int runBlenderExport(const ModelContext& context, const CommandLine& line) {
    // A troca e a fonte da malha: so o corpo de hoje (um corpo gravado nela trocaria o molde).
    if (line.has("corpo")) {
        std::cerr << "exportar-blender: so o corpo de hoje (sem --corpo)\n";
        return 2;
    }
    if (!line.has("saida")) {
        std::cerr << "exportar-blender: --saida <pasta> (grava jogador.troca.json e as texturas)\n";
        return 2;
    }
    const std::filesystem::path folder = line.text("saida");
    std::filesystem::create_directories(folder);
    const Mesh& mesh = context.model().mesh;
    const Skeleton& skeleton = context.model().skeleton;
    using Json = nlohmann::json;
    const auto vec = [](Vec3 v) { return Json::array({ v.x, v.y, v.z }); };
    Json joints = Json::array();
    for (const Joint& joint : skeleton.joints) {
        const Quaternion q = joint.bind.orientation;
        Json entry { { "name", joint.name }, { "parent", joint.parent }, { "kind", std::string(jointKindName(joint.kind)) },
            { "position", vec(joint.bind.position) }, { "orientation", Json::array({ q.x, q.y, q.z, q.w }) } };
        if (joint.kind == JointKind::Helper)
            entry["helper"] = { { "from", joint.derivedFrom }, { "to", joint.derivedTo }, { "fraction", joint.derivedFraction },
                { "axis", vec(joint.derivedAxis) }, { "sign", joint.derivedSign }, { "carry", joint.derivedCarry }, { "cap", joint.derivedCap },
                { "pivot", vec(joint.derivedPivot) } };
        joints.push_back(std::move(entry));
    }
    Json textures = Json::array();
    for (std::size_t t = 0; t < mesh.textures.size(); ++t) {
        const std::string name = "jogador_textura_" + std::to_string(t) + ".png";
        std::string error;
        if (!writePng(mesh.textures[t].image, (folder / name).string(), &error)) {
            std::cerr << "exportar-blender: " << error << '\n';
            return 1;
        }
        textures.push_back(name);
    }
    Json parts = Json::array(), materials = Json::array();
    for (const MeshPart& part : mesh.parts)
        parts.push_back({ { "name", part.name }, { "region", part.region }, { "connectedTo", part.connectedTo }, { "openings", part.openings },
            { "root", part.root } });
    for (const MeshMaterial& material : mesh.materials)
        materials.push_back({ { "name", material.name }, { "color", Json::array({ material.color.r, material.color.g, material.color.b }) },
            { "texture", material.texture } });
    Json positions = Json::array(), weights = Json::array();
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        positions.push_back(vec(mesh.positions[v]));
        Json pairs = Json::array();
        if (mesh.hasSkin())
            for (std::size_t i = 0; i < 4; ++i)
                if (mesh.skin[v].weights[i] > 0.0f) pairs.push_back(Json::array({ mesh.skin[v].joints[i], mesh.skin[v].weights[i] }));
        weights.push_back(std::move(pairs));
    }
    Json triangles = Json::array(), triangleUv = Json::array();
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        triangles.push_back(Json::array({ mesh.triangles[t][0], mesh.triangles[t][1], mesh.triangles[t][2] }));
        if (mesh.hasUv()) {
            Json corners = Json::array();
            for (const Uv& uv : mesh.triangleUv[t]) corners.push_back(Json::array({ uv.u, uv.v }));
            triangleUv.push_back(std::move(corners));
        }
    }
    const Json document { { "schema", "matter.troca_blender.v1" }, { "ground", skeleton.groundHeight }, { "joints", joints },
        { "parts", parts }, { "materials", materials }, { "textures", textures }, { "positions", positions }, { "weights", weights },
        { "triangles", triangles }, { "trianglePart", mesh.trianglePart }, { "triangleMaterial", mesh.triangleMaterial },
        { "triangleUv", triangleUv } };
    std::ofstream out(folder / "jogador.troca.json");
    out << document.dump();
    if (!out) {
        std::cerr << "exportar-blender: nao consegui gravar " << (folder / "jogador.troca.json").string() << '\n';
        return 1;
    }
    std::cout << "Troca para o Blender em " << folder.string() << ": " << mesh.positions.size() << " vertices, " << mesh.triangles.size()
              << " triangulos, " << skeleton.joints.size() << " juntas, " << mesh.textures.size() << " textura(s)\n";
    // --poses N: as poses dos clipes do jogo (N instantes cada, a mao solta),
    // com o skin de cada junta (os auxiliares pelas regras do GameRig): o que
    // o jogo faz com a malha, para ajustar os pesos no Blender. Um clipe em
    // cada cinco (pela ordem) fica de fora do treino ("teste").
    if (line.has("poses") && context.loaded.character && context.loaded.rig) {
        const int instants = std::max(1, line.integer("poses", 6));
        Json poses = Json::array();
        int clipIndex = 0;
        for (const std::string& id : availableClipIds()) {
            std::string error;
            const std::optional<AnimationClip3D> clip = loadCharacterClip(id, &error);
            if (!clip) continue;
            const bool test = clipIndex++ % 5 == 4;
            for (int k = 0; k < instants; ++k) {
                const float time = clip->durationSeconds * static_cast<float>(k) / static_cast<float>(instants);
                const SkeletonPose pose = sampleGameRigPose(*context.loaded.character, *context.loaded.rig, &*clip, time, RigHandPose::Relaxed, true);
                Json transforms = Json::array();
                for (const Transform& skin : skinningTransforms(skeleton, pose)) {
                    const Quaternion q = skin.orientation;
                    transforms.push_back(Json::array({ skin.position.x, skin.position.y, skin.position.z, q.x, q.y, q.z, q.w }));
                }
                char label[160];
                std::snprintf(label, sizeof label, "%s@%.2f", id.c_str(), time);
                poses.push_back({ { "name", label }, { "test", test }, { "skin", std::move(transforms) } });
            }
        }
        // --amplitude: e as poses da amplitude das juntas (os dois lados), no
        // treino: os clipes quase nao erguem o braco nem dobram o quadril ate o
        // fim, e o assado so acerta o que ve (como o assado sobre uma animacao
        // de amplitude nos estudios).
        if (line.has("amplitude")) {
            std::vector<std::string> specs;
            for (const char* side : { "Right", "Left" }) {
                const std::string arm = std::string(side) + "UpperArm", thigh = std::string(side) + "Thigh",
                                  shin = std::string(side) + "Shin", other = std::string(side) == "Right" ? "LeftThigh" : "RightThigh";
                const int open = std::string(side) == "Right" ? -1 : 1;
                for (const int degrees : { 60, 90, 120, 150 }) specs.push_back("dobra:" + arm + "=" + std::to_string(-degrees));
                for (const int degrees : { 45, 90, 135 }) specs.push_back("dobra:" + arm + "=" + std::to_string(open * degrees) + "x");
                specs.push_back("dobra:" + arm + "=40");
                specs.push_back("dobra:" + arm + "=-90+" + arm + "=" + std::to_string(open * 45) + "x");
                for (const int degrees : { 30, 60 }) specs.push_back("dobra:" + thigh + "=" + std::to_string(-degrees));
                specs.push_back("dobra:" + thigh + "=-90+" + shin + "=90");
                specs.push_back("dobra:" + thigh + "=-110+" + shin + "=110");
                for (const int degrees : { 20, 40 }) specs.push_back("dobra:" + thigh + "=" + std::to_string(degrees));
                specs.push_back("dobra:" + thigh + "=" + std::to_string(-open * 45) + "x");
                specs.push_back("dobra:" + thigh + "=-50+" + shin + "=40+" + other + "=25");
            }
            for (const char* spine : { "dobra:Abdomen=25+Chest=15", "dobra:Abdomen=-15+Chest=-10", "dobra:Abdomen=15x+Chest=10x",
                     "dobra:Abdomen=-15x+Chest=-10x", "dobra:Abdomen=20z+Chest=15z", "dobra:Abdomen=-20z+Chest=-15z" })
                specs.push_back(spine);
            // --extra a,b: e essas poses (dobras avulsas: as de um portao de teste).
            if (line.has("extra")) {
                std::string list = line.text("extra"), item;
                for (std::size_t k = 0; k <= list.size(); ++k) {
                    if (k == list.size() || list[k] == ',') {
                        if (!item.empty()) specs.push_back(item);
                        item.clear();
                    } else {
                        item += list[k];
                    }
                }
            }
            int added = 0;
            for (const std::string& spec : specs) {
                std::string error;
                const std::optional<SkeletonPose> pose = poseFromSpec(context.loaded, spec, &error);
                if (!pose) {
                    std::cerr << "exportar-blender: " << spec << ": " << error << '\n';
                    return 1;
                }
                Json transforms = Json::array();
                for (const Transform& skin : skinningTransforms(skeleton, *pose)) {
                    const Quaternion q = skin.orientation;
                    transforms.push_back(Json::array({ skin.position.x, skin.position.y, skin.position.z, q.x, q.y, q.z, q.w }));
                }
                poses.push_back({ { "name", spec }, { "test", false }, { "skin", std::move(transforms) } });
                ++added;
            }
            std::cout << "  amplitude: " << added << " poses\n";
        }
        std::ofstream file(folder / "jogador.poses.json");
        file << Json { { "schema", "matter.poses_blender.v1" }, { "poses", poses } }.dump();
        std::cout << "  poses: " << poses.size() << " (skin de cada junta: posicao + giro) em jogador.poses.json\n";
    }
    return 0;
}

} // namespace MatterEngine::ModelInspector
