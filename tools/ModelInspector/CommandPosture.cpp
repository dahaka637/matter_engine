#include "Commands.hpp"

#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

struct Range {
    float sum = 0.0f;
    float minimum = 1e9f;
    float maximum = -1e9f;
    int count = 0;
    void add(float value) {
        sum += value;
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
        ++count;
    }
    [[nodiscard]] float mean() const { return count > 0 ? sum / static_cast<float>(count) : 0.0f; }
};

// A postura numa pose, no plano do corpo (a frente da pelve, sem o rumo):
// graus para a frente desde a vertical (+ inclina para a frente) e o quanto
// a cabeca vai a frente da base do pescoco.
struct Posture {
    float torso = 0.0f;
    float neck = 0.0f;
    float neckOnTorso = 0.0f;
    float headForward = 0.0f;
    float gaze = 0.0f;
};

Posture measure(const Skeleton& skeleton, const SkeletonPose& pose) {
    const auto at = [&](const char* name) { return pose.joints[static_cast<std::size_t>(skeleton.find(name))]; };
    const auto bindAt = [&](const char* name) { return skeleton.joints[static_cast<std::size_t>(skeleton.find(name))].bind; };
    // A frente do corpo: o x da pelve no chao.
    const Quaternion pelvisTurn = (at("Pelvis").orientation * bindAt("Pelvis").orientation.conjugate()).normalized();
    Vec3 forward = pelvisTurn.rotate({ 1.0f, 0.0f, 0.0f });
    forward.z = 0.0f;
    forward = safeNormalized(forward, { 1.0f, 0.0f, 0.0f });
    const auto pitch = [&](Vec3 v) { return std::atan2(dot(v, forward), v.z) * RadiansToDegrees; };
    Posture posture;
    posture.torso = pitch(at("UpperChest").position - at("Pelvis").position);
    posture.neck = pitch(at("Head").position - at("Neck").position);
    posture.neckOnTorso = posture.neck - pitch(at("Neck").position - at("UpperChest").position);
    posture.headForward = dot(at("HeadTop").position - at("Neck").position, forward);
    // O olhar: a frente da cabeca desde a ligacao (+ para baixo).
    const Vec3 gaze = (at("Head").orientation * bindAt("Head").orientation.conjugate()).normalized().rotate({ 1.0f, 0.0f, 0.0f });
    posture.gaze = std::atan2(-gaze.z, std::max(1e-6f, dot(gaze, forward))) * RadiansToDegrees;
    return posture;
}

} // namespace

// A postura nas animacoes do jogo: em cada clipe, ao longo dele, a
// inclinacao do tronco e do pescoco (da vertical e sobre o tronco), o quanto
// a cabeca vai a frente da base do pescoco e o olhar - e a folha de lado.
//   --clipes a,b (padrao: o parado e as passadas do jogo)  --amostras 24
int runPosture(const ModelContext& context, const CommandLine& line) {
    const LoadedPlayerModel& loaded = context.loaded;
    if (!loaded.character || !loaded.rig) {
        std::cerr << "precisa do personagem do jogo\n";
        return 1;
    }
    std::vector<std::string> clips = line.list("clipes");
    if (clips.empty())
        clips = { "alert_idle", "jog", "run_backward", "sprint", "sprint_backward_long", "jog_strafe_left", "sprint_strafe_left",
            "idle_to_sprint", "crouched_to_sprint", "turn_left" };
    const int samples = std::max(2, line.integer("amostras", 24));
    const Skeleton& skeleton = loaded.rig->skeleton;
    {
        const Posture bind = measure(skeleton, bindPose(skeleton));
        std::printf("Ligacao: tronco %+.1f, pescoco %+.1f (sobre o tronco %+.1f), cabeca a frente %+.1f cm\n", bind.torso, bind.neck,
            bind.neckOnTorso, bind.headForward * 100.0f);
        // O repouso do perfil (as juntas todas em zero): de onde os clipes partem.
        const Posture rest = measure(skeleton, sampleGameRigPose(*loaded.character, *loaded.rig, nullptr, 0.0f));
        std::printf("Repouso do perfil: tronco %+.1f, pescoco %+.1f (sobre o tronco %+.1f), cabeca a frente %+.1f cm, olhar %+.1f\n\n",
            rest.torso, rest.neck, rest.neckOnTorso, rest.headForward * 100.0f, rest.gaze);
    }
    std::printf("Graus para a frente desde a vertical (media [min, max]); cabeca a frente da base do pescoco (cm); olhar + para baixo\n");
    std::printf("%-22s %-20s %-20s %-20s %-20s %-20s\n", "clipe", "tronco", "pescoco", "pescoco/tronco", "cabeca a frente", "olhar");
    std::vector<Image> images;
    std::vector<std::string> captions;
    const InspectionModel& model = context.model();
    const ViewPreset* side = findViewPreset("direita");
    for (const std::string& id : clips) {
        std::string error;
        const std::optional<AnimationClip3D> clip = loadCharacterClip(id, &error);
        if (!clip) {
            std::cerr << id << ": " << error << '\n';
            continue;
        }
        Range torso, neck, neckOnTorso, headForward, gaze;
        for (int k = 0; k < samples; ++k) {
            const float t = clip->durationSeconds * static_cast<float>(k) / static_cast<float>(samples);
            const Posture p = measure(skeleton, sampleGameRigPose(*loaded.character, *loaded.rig, &*clip, t));
            torso.add(p.torso);
            neck.add(p.neck);
            neckOnTorso.add(p.neckOnTorso);
            headForward.add(p.headForward * 100.0f);
            gaze.add(p.gaze);
        }
        const auto cell = [](const Range& r) {
            char text[48];
            std::snprintf(text, sizeof text, "%+5.1f [%+5.1f, %+5.1f]", r.mean(), r.minimum, r.maximum);
            return std::string(text);
        };
        std::printf("%-22s %-20s %-20s %-20s %-20s %-20s\n", id.c_str(), cell(torso).c_str(), cell(neck).c_str(), cell(neckOnTorso).c_str(),
            cell(headForward).c_str(), cell(gaze).c_str());
        // A folha: o clipe no meio, de lado.
        if (model.mesh.hasSkin()) {
            const SkeletonPose pose = sampleGameRigPose(*loaded.character, *loaded.rig, &*clip, clip->durationSeconds * 0.25f);
            const std::vector<Vec3> posed = model.mesh.posedPositions(skinningTransforms(model.skeleton, pose));
            InspectionScene scene = context.scene();
            scene.pose = &pose;
            scene.meshPositions = posed;
            InspectionStyle style;
            style.supersample = 2;
            style.hud = false;
            style.gizmoLegend = false;
            const OrbitCamera camera = frameRegion(scene, 0, 0.8f, side);
            images.push_back(renderInspection(scene, style, camera, 400, 500));
            captions.push_back(id);
        }
    }
    if (images.empty()) return 0;
    return saveImage(composeSheet(images, captions, 5, "Postura nas animacoes (de lado, no quarto do clipe)"),
               context.output(line, "postura.png"))
        ? 0
        : 1;
}

} // namespace MatterEngine::ModelInspector
