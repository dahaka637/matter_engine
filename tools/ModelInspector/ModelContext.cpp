#include "ModelContext.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

InspectionScene ModelContext::scene() const {
    InspectionScene scene;
    const InspectionModel& m = model();
    scene.skeleton = &m.skeleton;
    scene.mesh = &m.mesh;
    scene.regions = &m.regions;
    scene.jointRegion = m.jointRegion;
    scene.surface = m.surface.get();
    scene.kit = m.kit.get();
    if (pose) {
        scene.pose = &*pose;
        if (!posedPositions.empty()) scene.meshPositions = posedPositions;
    }
    return scene;
}

int ModelContext::region(const std::string& id) const {
    const int found = model().regions.find(id);
    if (found < 0) {
        std::cerr << "regiao desconhecida: " << id << " (veja: MatterModelInspector info)\n";
    }
    return found;
}

std::filesystem::path ModelContext::output(const CommandLine& line, const std::string& defaultName) const {
    if (line.has("saida")) return line.text("saida");
    return outputDirectory / defaultName;
}

namespace {

// A mao do jogo pelo nome (reta = a da ligacao).
std::optional<RigHandPose> handFromName(const std::string& name) {
    if (name == "reta") return RigHandPose::Bind;
    if (name == "repouso") return RigHandPose::Relaxed;
    if (name == "aberta") return RigHandPose::Open;
    if (name == "punho") return RigHandPose::Fist;
    return std::nullopt;
}

// "dobra:RightShin=90+RightThigh=-60+LeftThigh=20x": a pose de partida com
// juntas giradas em graus em volta de um eixo do modelo (y, o lado, se nao
// disser; x ou z no fim do numero; "d" e a dobradica da propria junta na
// partida, jointHingeAxis), de cima para baixo na arvore (joelho: + dobra;
// quadril: - leva a coxa para a frente; em x, abre a perna para o lado). Os
// eixos sao os da partida e giram junto com o pai (como na tela).
// "RightFist=60" / "LeftFist=60": a mao fechada 60% da partida ate o punho
// do jogo (o controle "Dedos" da tela).
std::optional<SkeletonPose> bendSpec(const Skeleton& skeleton, const SkeletonPose& start, const SkeletonPose* fist,
    const std::string& terms, std::string* error) {
    std::vector<JointBend> bends;
    std::size_t cursor = 0;
    while (cursor <= terms.size()) {
        const std::size_t end = std::min(terms.find('+', cursor), terms.size());
        const std::string term = terms.substr(cursor, end - cursor);
        cursor = end + 1;
        if (term.empty()) continue;
        const std::size_t equals = term.find('=');
        if (const std::string name = term.substr(0, equals); equals != std::string::npos && (name == "RightFist" || name == "LeftFist")) {
            if (fist == nullptr) {
                if (error != nullptr) *error = "dobra: sem o punho do jogo (" + term + ")";
                return std::nullopt;
            }
            std::string value = term.substr(equals + 1);
            for (char& c : value)
                if (c == ',') c = '.';
            const std::vector<JointBend> closing = blendJointBends(skeleton, start, *fist,
                fingerJoints(skeleton, name == "RightFist" ? Side::Right : Side::Left, false), std::strtof(value.c_str(), nullptr) / 100.0f);
            bends.insert(bends.end(), closing.begin(), closing.end());
            continue;
        }
        const int joint = skeleton.find(term.substr(0, equals));
        if (equals == std::string::npos || joint < 0) {
            if (error != nullptr) *error = "dobra: esperado Junta=graus (" + term + ")";
            return std::nullopt;
        }
        std::string value = term.substr(equals + 1);
        JointBend bend;
        bend.joint = joint;
        if (!value.empty() && (value.back() == 'x' || value.back() == 'y' || value.back() == 'z' || value.back() == 'd')) {
            // "d": a dobradica da propria junta na partida (o dedo fecha para
            // a palma, o polegar atravessando ela; o cotovelo e o joelho no
            // plano do membro).
            bend.axis = value.back() == 'x' ? Vec3 { 1.0f, 0.0f, 0.0f }
                : value.back() == 'z'      ? Vec3 { 0.0f, 0.0f, 1.0f }
                : value.back() == 'd'      ? jointHingeAxis(skeleton, start, joint)
                                           : bend.axis;
            value.pop_back();
        }
        for (char& c : value)
            if (c == ',') c = '.';
        bend.radians = std::strtof(value.c_str(), nullptr) * DegreesToRadians;
        bend.followParent = true;
        bends.push_back(bend);
    }
    return bendPose(skeleton, start, bends);
}

} // namespace

std::optional<SkeletonPose> poseFromSpec(const LoadedPlayerModel& loaded, const std::string& fullSpec, std::string* error) {
    // A mao no fim ("sprint@0.3#punho"): reta (a da ligacao), repouso,
    // aberta ou punho (as do jogo); sem ela, a de repouso (numa dobra, a
    // reta: "dobra:RightThumb3=60d#punho" dobra a partir da mao do jogo, como
    // os controles da tela).
    std::string spec = fullSpec;
    const bool bend = spec.rfind("dobra:", 0) == 0;
    std::optional<RigHandPose> hand;
    if (const std::size_t mark = spec.find('#'); mark != std::string::npos) {
        hand = handFromName(spec.substr(mark + 1));
        if (!hand) {
            if (error != nullptr) *error = "mao: esperado #reta, #repouso, #aberta ou #punho (" + spec.substr(mark + 1) + ")";
            return std::nullopt;
        }
        spec = spec.substr(0, mark);
    }
    if (bend) {
        // A ligacao com os dedos da mao do jogo pedida (o corpo na ligacao).
        const SkeletonPose bind = bindPose(loaded.rig->skeleton);
        const SkeletonPose start = hand ? withGameHands(*loaded.character, *loaded.rig, bind, *hand) : bind;
        const SkeletonPose fist = withGameHands(*loaded.character, *loaded.rig, bind, RigHandPose::Fist);
        return bendSpec(loaded.rig->skeleton, start, &fist, spec.substr(6), error);
    }
    const std::size_t at = spec.find('@');
    const std::string clipId = spec.substr(0, at);
    float time = 0.0f;
    if (at != std::string::npos) {
        std::string value = spec.substr(at + 1);
        for (char& c : value)
            if (c == ',') c = '.';
        time = std::strtof(value.c_str(), nullptr);
    }
    if ((clipId == "ligacao" || clipId == "bind") && !hand) return bindPose(loaded.rig->skeleton);
    const RigHandPose gameHand = hand.value_or(RigHandPose::Relaxed);
    if (clipId == "ligacao" || clipId == "bind" || clipId == "repouso")
        return sampleGameRigPose(*loaded.character, *loaded.rig, nullptr, 0.0f, gameHand, true);
    const std::optional<AnimationClip3D> clip = loadCharacterClip(clipId, error);
    if (!clip) return std::nullopt;
    return sampleGameRigPose(*loaded.character, *loaded.rig, &*clip, time, gameHand, true);
}

std::optional<ModelContext> loadModelContext(const CommandLine& line, std::string* error) {
    PlayerModelOptions options;
    options.meshSource = line.text("malha");
    options.kit = line.text("uniforme");
    const std::filesystem::path manifest = line.has("modelo") ? std::filesystem::path(line.text("modelo"))
                                                              : defaultPlayerModelManifest();
    std::optional<LoadedPlayerModel> loaded = loadPlayerModel(manifest, options, error);
    if (!loaded) return std::nullopt;
    ModelContext context;
    context.loaded = std::move(*loaded);
    if (line.has("corpo")) {
        const std::optional<PlayerBody3D> body = parseBody3D(line.text("corpo"));
        if (!body) {
            if (error != nullptr) *error = "--corpo <cm>,<kg> (de 170 a 200 cm, de 60 a 100 kg): " + line.text("corpo");
            return std::nullopt;
        }
        try {
            applyPlayerBody(context.loaded, *body);
        } catch (const std::exception& exception) {
            if (error != nullptr) *error = std::string("corpo: ") + exception.what();
            return std::nullopt;
        }
    }
    context.outputDirectory = line.has("saida-dir") ? std::filesystem::path(line.text("saida-dir"))
                                                    : std::filesystem::current_path() / "inspecao";
    if (line.has("pose")) {
        const std::string spec = line.text("pose");
        context.pose = poseFromSpec(context.loaded, spec, error);
        if (!context.pose) return std::nullopt;
        context.poseLabel = spec;
        const InspectionModel& model = *context.loaded.model;
        if (model.mesh.hasSkin())
            context.posedPositions = model.mesh.posedPositions(skinningTransforms(model.skeleton, *context.pose));
    }
    return context;
}

} // namespace MatterEngine::ModelInspector
