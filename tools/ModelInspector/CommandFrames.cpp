#include "Commands.hpp"

#include "Engine/Animation/RagdollVisualPose3D.hpp"
#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"
#if defined(MATTERENGINE_MODEL_INSPECTOR_RL)
#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// Folha de quadros de uma gravacao do jogo com o jogador de verdade (a pele, nao
// os colisores): o dump da MatterLocomotionProbe (--dump), do harness
// (XPOSEDUMP), da MatterAthleteProbe, das sondas de RL (MatterRLDynamics,
// MatterRLJointProbe) ou de um episodio do treino (matter-rl folha). Formato
// (linhas):
//   S <nome do cenario>
//   F <t> <estado> <fase>
//   C|B <link> <centro x y z> <orientacao x y z w> ...   (o colisor do link)
//   K <bola x y z> [velocidade x y z]
//   Q <lado> <x y z> ...    (alvo do pe que joga)
//   Z <lado> <x y z> <apoiado 0|1>   (pegada planejada)
//   T                        (tocou na bola neste quadro)
//   E <texto>                (evento: entra na legenda)
//   P <x y z> <link> [normal x y z] [impulso N.s]   (contato fisico)
// Linhas desconhecidas sao ignoradas.
//   --dump <arquivo> --cenario <nome ou indice>
//   --instantes t0,t1,...  ou  --de A --ate B --quadros K (padrao: 8 no cenario)
//   --vista <id> ou --yaw/--pitch   --seguir 0|1   --distancia <m>   --chao <z do mundo>
//   --colunas C   --largura/--altura (de cada quadro)   --modo <id>   --saida <png>
//   --colisores 1 (as shapes fisicas por cima, com BUILD_RL)
namespace {

struct DumpLink {
    std::string id;
    bool box = false;
    Vec3 center;
    Quaternion orientation;
};

struct DumpFrame {
    float t = 0.0f;
    int state = 0;
    std::vector<DumpLink> links;
    bool ballValid = false;
    Vec3 ball;
    std::vector<Vec3> footGoals;
    std::vector<std::pair<Vec3, bool>> footsteps;
    bool touched = false;
    std::string events;
    struct Contact {
        Vec3 position;
        Vec3 normal;
        std::uint32_t linkIndex = 0;
        float impulseNewtonSeconds = 0.0f;
        bool hasNormal = false;
    };
    std::vector<Contact> contacts;
};

struct DumpScenario {
    std::string name;
    std::vector<DumpFrame> frames;
};

std::vector<DumpScenario> readDump(const std::string& path) {
    std::vector<DumpScenario> scenarios;
    std::ifstream file(path);
    std::string line;
    DumpFrame* frame = nullptr;
    while (std::getline(file, line)) {
        std::istringstream in(line);
        std::string tag;
        if (!(in >> tag)) continue;
        if (tag == "S") {
            DumpScenario scenario;
            std::getline(in, scenario.name);
            if (!scenario.name.empty() && scenario.name.front() == ' ') scenario.name.erase(0, 1);
            scenarios.push_back(std::move(scenario));
            frame = nullptr;
        } else if (tag == "F") {
            if (scenarios.empty()) scenarios.push_back({ "?", {} });
            DumpFrame next;
            in >> next.t >> next.state;
            scenarios.back().frames.push_back(std::move(next));
            frame = &scenarios.back().frames.back();
        } else if (frame == nullptr) {
            continue;
        } else if (tag == "C" || tag == "B") {
            DumpLink link;
            link.box = tag == "B";
            in >> link.id >> link.center.x >> link.center.y >> link.center.z >> link.orientation.x >> link.orientation.y
                >> link.orientation.z >> link.orientation.w;
            frame->links.push_back(std::move(link));
        } else if (tag == "K") {
            in >> frame->ball.x >> frame->ball.y >> frame->ball.z;
            frame->ballValid = static_cast<bool>(in);
        } else if (tag == "Q") {
            int side = 0;
            Vec3 goal;
            in >> side >> goal.x >> goal.y >> goal.z;
            if (in) frame->footGoals.push_back(goal);
        } else if (tag == "Z") {
            int side = 0, planted = 0;
            Vec3 step;
            in >> side >> step.x >> step.y >> step.z >> planted;
            if (in) frame->footsteps.push_back({ step, planted != 0 });
        } else if (tag == "T") {
            frame->touched = true;
        } else if (tag == "E") {
            std::string text;
            std::getline(in, text);
            frame->events += text;
        } else if (tag == "P") {
            DumpFrame::Contact contact;
            in >> contact.position.x >> contact.position.y >> contact.position.z >> contact.linkIndex;
            if (!in) continue;
            if (in >> contact.normal.x >> contact.normal.y >> contact.normal.z >> contact.impulseNewtonSeconds)
                contact.hasNormal = contact.normal.lengthSquared() > 0.000001f;
            frame->contacts.push_back(contact);
        }
    }
    return scenarios;
}

// A pose dos links (origem e orientacao) a partir dos colisores do dump.
bool linksFromFrame(const RagdollProfile3D& profile, const DumpFrame& frame, RagdollAnimationPose3D& pose) {
    pose.linkPositions.assign(profile.links.size(), {});
    pose.linkOrientations.assign(profile.links.size(), {});
    std::size_t found = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        for (const DumpLink& dumped : frame.links) {
            if (dumped.id != link.id) continue;
            const Quaternion orientation =
                (dumped.orientation.normalized() * link.collider.localOrientation.conjugate()).normalized();
            pose.linkOrientations[i] = orientation;
            pose.linkPositions[i] = dumped.center - orientation.rotate(link.collider.localPosition);
            ++found;
            break;
        }
    }
    return found == profile.links.size();
}

Highlight circle(Vec3 center, float radius, Vec3 a, Vec3 b, Color color) {
    Highlight highlight;
    highlight.color = color;
    constexpr int Segments = 28;
    for (int k = 0; k < Segments; ++k) {
        const float u0 = 6.2831853f * static_cast<float>(k) / Segments;
        const float u1 = 6.2831853f * static_cast<float>(k + 1) / Segments;
        highlight.segments.push_back({ center + a * (radius * std::cos(u0)) + b * (radius * std::sin(u0)),
            center + a * (radius * std::cos(u1)) + b * (radius * std::sin(u1)) });
    }
    return highlight;
}

std::vector<float> chooseTimes(const DumpScenario& scenario, const CommandLine& line) {
    std::vector<float> times;
    for (const std::string& item : line.list("instantes")) times.push_back(std::strtof(item.c_str(), nullptr));
    if (!times.empty() || scenario.frames.empty()) return times;
    const float first = scenario.frames.front().t, last = scenario.frames.back().t;
    const float from = line.number("de", first), to = line.number("ate", last);
    const int count = std::clamp(line.integer("quadros", 8), 1, 64);
    for (int k = 0; k < count; ++k)
        times.push_back(count == 1 ? from : from + (to - from) * static_cast<float>(k) / static_cast<float>(count - 1));
    return times;
}

std::size_t nearestFrame(const DumpScenario& scenario, float t) {
    std::size_t best = 0;
    for (std::size_t i = 1; i < scenario.frames.size(); ++i)
        if (std::abs(scenario.frames[i].t - t) < std::abs(scenario.frames[best].t - t)) best = i;
    return best;
}

} // namespace

int runFrames(const ModelContext& context, const CommandLine& line) {
    if (!line.has("dump")) {
        std::cerr << "quadros: --dump <arquivo> (a gravacao da sonda ou do harness)\n";
        return 2;
    }
    const std::vector<DumpScenario> scenarios = readDump(line.text("dump"));
    if (scenarios.empty()) {
        std::cerr << "quadros: dump vazio ou ilegivel\n";
        return 2;
    }
    const DumpScenario* scenario = &scenarios.front();
    if (line.has("cenario")) {
        const std::string wanted = line.text("cenario");
        scenario = nullptr;
        for (const DumpScenario& item : scenarios)
            if (item.name == wanted || item.name.find(wanted) != std::string::npos) {
                scenario = &item;
                break;
            }
        if (scenario == nullptr) {
            const int index = std::atoi(wanted.c_str());
            if (index >= 0 && static_cast<std::size_t>(index) < scenarios.size()) scenario = &scenarios[static_cast<std::size_t>(index)];
        }
        if (scenario == nullptr) {
            std::cerr << "quadros: cenario nao encontrado: " << wanted << " (ha " << scenarios.size() << ")\n";
            for (const DumpScenario& item : scenarios) std::cerr << "  " << item.name << '\n';
            return 2;
        }
    }
    if (scenario->frames.empty()) {
        std::cerr << "quadros: cenario sem quadros\n";
        return 2;
    }

    const RagdollCharacter3D& character = *context.loaded.character;
    const GameRig& rig = *context.loaded.rig;
    const InspectionModel& model = context.model();
    const RagdollProfile3D& profile = character.profile;
    const int width = std::clamp(line.integer("largura", 420), 64, 2048);
    const int height = std::clamp(line.integer("altura", 520), 64, 2048);
    const std::string modeId = line.text("modo", model.mesh.empty() ? "ossos" : "solido");
    const std::optional<SurfaceMode> mode = parseSurfaceMode(modeId);
    if (!mode) {
        std::cerr << "modo desconhecido: " << modeId << '\n';
        return 2;
    }
    float yaw = 35.0f, pitch = 12.0f;
    if (line.has("vista")) {
        const ViewPreset* preset = findViewPreset(line.text("vista"));
        if (preset == nullptr) {
            std::cerr << "vista desconhecida: " << line.text("vista") << '\n';
            return 2;
        }
        yaw = preset->yawDegrees;
        pitch = preset->pitchDegrees;
    }
    yaw = line.number("yaw", yaw);
    pitch = line.number("pitch", pitch);
    const bool follow = line.integer("seguir", 1) != 0;
    const float distance = line.number("distancia", 4.4f);
    const float floorWorld = line.number("chao", 0.0f);
    // O chao do espaco do modelo do jogo (o perfil: a pelve do repouso na
    // posicao dela; o chao standingRootHeightMeters abaixo).
    const float gameFloor = profile.links.empty() ? 0.0f
        : profile.links.front().modelPosition.z - profile.standingRootHeightMeters;
#if defined(MATTERENGINE_MODEL_INSPECTOR_RL)
    const bool drawColliders = line.integer("colisores", 0) != 0;
    const std::optional<RagdollColliderWireframe3D> restColliders = drawColliders
        ? std::optional<RagdollColliderWireframe3D>(buildRagdollColliderWireframe3D(profile)) : std::nullopt;
#endif

    // O primeiro quadro fixa o referencial sem --seguir.
    RagdollAnimationPose3D firstLinks;
    if (!linksFromFrame(profile, scenario->frames.front(), firstLinks)) {
        std::cerr << "quadros: os links do dump nao batem com o perfil do personagem\n";
        return 2;
    }
    const Vec3 fixedPelvis = firstLinks.linkPositions.front();
    // Mundo do dump -> espaco do modelo do jogo -> espaco do esqueleto (a raiz
    // medida do modelo novo sobe/escala como em sampleGameRigPose).
    const auto toSkeleton = [&](Vec3 game) {
        if (!rig.fitted) return game;
        for (const Joint& joint : rig.skeleton.joints)
            if (joint.parent < 0)
                return joint.bind.position + Vec3 { 0.0f, 0.0f, rig.rootRestLift }
                    + (game - rig.rootGameBind.position) * rig.rootScale;
        return game;
    };

    RagdollVisualPose3D visual;
    std::vector<Image> images;
    std::vector<std::string> captions;
    float previousT = scenario->frames.front().t;
    for (const float wantedT : chooseTimes(*scenario, line)) {
        const DumpFrame& frame = scenario->frames[nearestFrame(*scenario, wantedT)];
        RagdollAnimationPose3D links;
        if (!linksFromFrame(profile, frame, links)) continue;
        const Vec3 pelvisWorld = links.linkPositions.front();
        const Vec3 anchor = follow ? pelvisWorld : fixedPelvis;
        const auto toGame = [&](Vec3 world) {
            return Vec3 { world.x - anchor.x + profile.links.front().modelPosition.x,
                world.y - anchor.y + profile.links.front().modelPosition.y, world.z - floorWorld + gameFloor };
        };
        for (Vec3& position : links.linkPositions) position = toGame(position);
        RagdollVisualPoseInput3D visualInput;
        visualInput.linkPositions = links.linkPositions;
        visualInput.linkOrientations = links.linkOrientations;
        visualInput.groundHeight = { gameFloor, gameFloor };
        visualInput.deltaTime = std::max(1.0f / 120.0f, frame.t - previousT);
        previousT = frame.t;
        const std::vector<Quaternion> rotations = visual.update(character, visualInput);
        const SkeletonPose pose = gameRigPoseFromLinks(character, rig, links, rotations, false);
        std::vector<Vec3> posed;
        if (model.mesh.hasSkin()) posed = model.mesh.posedPositions(skinningTransforms(model.skeleton, pose));

        std::vector<Highlight> highlights;
        if (frame.ballValid) {
            const Vec3 ball = toSkeleton(toGame(frame.ball));
            const float radius = 0.11f;
            const Color color = frame.touched ? Palette::Error : Palette::Focus;
            highlights.push_back(circle(ball, radius, { 1, 0, 0 }, { 0, 1, 0 }, color));
            highlights.push_back(circle(ball, radius, { 1, 0, 0 }, { 0, 0, 1 }, color));
            highlights.push_back(circle(ball, radius, { 0, 1, 0 }, { 0, 0, 1 }, color));
        }
        for (const Vec3 goal : frame.footGoals) {
            Highlight marker;
            marker.color = Palette::Info;
            marker.points.push_back(toSkeleton(toGame(goal)));
            highlights.push_back(std::move(marker));
        }
        for (const auto& [step, planted] : frame.footsteps) {
            const Vec3 at = toSkeleton(toGame(step));
            highlights.push_back(circle({ at.x, at.y, toSkeleton(Vec3 { 0, 0, gameFloor }).z + 0.004f }, 0.05f,
                { 1, 0, 0 }, { 0, 1, 0 }, planted ? Palette::Left : Palette::Right));
        }
        for (const DumpFrame::Contact& contact : frame.contacts) {
            const Vec3 at = toSkeleton(toGame(contact.position));
            Highlight marker;
            marker.color = Palette::Error;
            constexpr float Radius = 0.025f;
            marker.segments.push_back({ at - Vec3 { Radius, 0, 0 }, at + Vec3 { Radius, 0, 0 } });
            marker.segments.push_back({ at - Vec3 { 0, Radius, 0 }, at + Vec3 { 0, Radius, 0 } });
            marker.segments.push_back({ at - Vec3 { 0, 0, Radius }, at + Vec3 { 0, 0, Radius } });
            if (contact.hasNormal)
                marker.segments.push_back({ at, toSkeleton(toGame(contact.position + contact.normal.normalized() * 0.09f)) });
            highlights.push_back(std::move(marker));
        }
#if defined(MATTERENGINE_MODEL_INSPECTOR_RL)
        if (restColliders) {
            const RagdollColliderWireframe3D colliders = poseRagdollColliderWireframe3D(profile, *restColliders,
                links.linkPositions, links.linkOrientations, false);
            std::array<Highlight, 3> sides;
            sides[0].color = Palette::Center;
            sides[1].color = Palette::Left;
            sides[2].color = Palette::Right;
            for (const RagdollColliderWireframeLink3D& link : colliders.links) {
                const std::size_t side = link.side == RagdollColliderWireframeSide3D::Left ? 1
                    : link.side == RagdollColliderWireframeSide3D::Right ? 2 : 0;
                for (const auto& [a, b] : link.segments) sides[side].segments.push_back({ toSkeleton(a), toSkeleton(b) });
            }
            highlights.insert(highlights.end(), std::make_move_iterator(sides.begin()),
                std::make_move_iterator(sides.end()));
        }
#endif

        InspectionScene scene = context.scene();
        scene.pose = &pose;
        scene.meshPositions = posed;
        scene.highlights = highlights;
        InspectionStyle style;
        style.surface = *mode;
        style.skeleton = model.mesh.empty();
        style.joints = false;
        style.hud = false;
        style.gizmo = false;
        style.scaleBar = false;
        style.gridExtent = 3.0f;
        style.supersample = std::clamp(line.integer("ssaa", 2), 1, 4);
        OrbitCamera camera;
        const Vec3 pelvis = toSkeleton(toGame(pelvisWorld));
        camera.target = { pelvis.x, pelvis.y, pelvis.z - 0.04f };
        camera.yaw = yaw * DegreesToRadians;
        camera.pitch = pitch * DegreesToRadians;
        camera.distance = distance;
        images.push_back(renderInspection(scene, style, camera, width, height));
        char time[48];
        std::snprintf(time, sizeof time, "t %.2f s", frame.t);
        std::string caption = time;
        if (frame.touched) caption += "  TOQUE";
        if (!frame.events.empty()) caption += " " + frame.events;
        if (!frame.contacts.empty()) caption += "  contatos " + std::to_string(frame.contacts.size());
        captions.push_back(std::move(caption));
    }
    if (images.empty()) {
        std::cerr << "quadros: nenhum quadro desenhado\n";
        return 2;
    }
    const int columns = std::clamp(line.integer("colunas", std::min<int>(4, static_cast<int>(images.size()))), 1, 16);
    return saveImage(composeSheet(images, captions, columns, scenario->name), context.output(line, "quadros.png")) ? 0 : 1;
}

} // namespace MatterEngine::ModelInspector
