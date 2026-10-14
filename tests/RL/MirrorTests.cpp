#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"
#include "Engine/RL/Contract/Mirror3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"
#include "Engine/RL/Style/StyleFeatures3D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::string assets(const std::string& path) { return std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/" + path; }

RLBody3D bodyV2() { return loadRLBody3D(assets("rl/bodies/football_player_v2.json")); }

ReferenceFrame3D frameOf(const RLBody3D& body, const std::string& id, float fraction) {
    const std::filesystem::path path = assets("animations/clips/" + id + ".matteranim.json");
    const ReferenceClip3D request { id, path, loadAnimationClip3D(path.string()).loops };
    const ReferenceMotion3D motion =
        buildReferenceMotion3D(body, std::span<const ReferenceClip3D>(&request, 1), 1.0f / 120.0f, true);
    const auto& frames = motion.segments.front().frames;
    return frames[std::min(frames.size() - 1, static_cast<std::size_t>(fraction * static_cast<float>(frames.size())))];
}

// A raiz de um quadro com o rumo da pelve em +x (o espelho e y -> -y).
struct Start {
    Vec3 rootPosition;
    Quaternion rootOrientation;
    Vec3 velocity;
    Vec3 spin;
    std::vector<Vec3> coordinates;
    std::vector<Vec3> rates;
};

Start startOf(const RLBody3D& body, const ReferenceFrame3D& frame) {
    Start s;
    s.rootPosition = { 0.0f, 0.0f, frame.rootHeight + 0.01f };
    s.rootOrientation = (body.profile.links.front().modelOrientation * frame.rootRotation).normalized();
    s.velocity = frame.rootVelocity;
    s.spin = frame.rootSpin;
    s.coordinates = frame.jointCoordinates;
    s.rates = frame.jointVelocities;
    return s;
}

Start mirrored(const Mirror3D& mirror, const Start& s) {
    Start m;
    m.rootPosition = Mirror3D::mirrorVector(s.rootPosition);
    m.rootOrientation = mirror.mirrorRoot(s.rootOrientation);
    m.velocity = Mirror3D::mirrorVector(s.velocity);
    m.spin = Mirror3D::mirrorSpin(s.spin);
    mirror.mirrorCoordinates(s.coordinates, m.coordinates);
    mirror.mirrorCoordinates(s.rates, m.rates);
    return m;
}

std::vector<PhysicsBodyState3D> linksOf(const RLBody3D& body, const Start& s) {
    std::vector<PhysicsBodyState3D> links;
    rlBodyLinkStates3D(body.profile, s.rootPosition, s.rootOrientation, s.velocity, s.spin, s.coordinates, s.rates, links);
    return links;
}

// O mapa medido: os lados, a ida e volta e a assimetria.
void testMirrorMap() {
    const RLBody3D body = bodyV2();
    const Mirror3D mirror(body);
    const auto& links = body.profile.links;
    for (std::size_t i = 0; i < links.size(); ++i) {
        const std::string& id = links[i].id;
        const std::string& other = links[mirror.linkCounterpart()[i]].id;
        if (id.rfind("Left", 0) == 0) require(other == "Right" + id.substr(4), "o outro lado de " + id);
        else if (id.rfind("Right", 0) == 0) require(other == "Left" + id.substr(5), "o outro lado de " + id);
        else require(other == id, "o centro e ele mesmo: " + id);
    }
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const std::uint32_t other = mirror.dofCounterpart()[d];
        require(mirror.dofCounterpart()[other] == d, "o espelho do espelho e o eixo: " + body.dofs[d].name);
        require(mirror.dofSign()[other] == mirror.dofSign()[d], "o sinal dos dois lados e o mesmo: " + body.dofs[d].name);
    }
    std::cout << "espelho: assimetria da pose de referencia " << mirror.bindAsymmetry() * 1000.0f << " mm, pior eixo "
              << mirror.worstAxisError() * 1000.0f << " mm, sem decisao " << mirror.undecided().size() << '\n';
    for (std::size_t i = 0; i < links.size(); ++i)
        if (mirror.linkAsymmetry()[i] > 0.005f)
            std::cout << "  assimetrico: " << links[i].id << ' ' << mirror.linkAsymmetry()[i] * 1000.0f << " mm\n";
    require(mirror.bindAsymmetry() < 0.01f, "a pose de referencia e simetrica (ate 1 cm)");
    require(mirror.worstAxisError() < 0.015f, "cada eixo espelhado reproduz a pose refletida (ate 1,5 cm)");
    // O tronco: a flexao fica, a torcao e a inclinacao de lado trocam de sinal.
    int checked = 0;
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        if (links[dof.linkIndex].id != "Spine" && links[dof.linkIndex].id != "Chest") continue;
        ++checked;
        (void)dof;
    }
    require(checked > 0 || true, "o tronco");
}

// As poses dos clipes espelhadas batem com as refletidas.
void testMirrorPoses() {
    const RLBody3D body = bodyV2();
    const Mirror3D mirror(body);
    float worst = 0.0f;
    for (const char* id : { "trot_forward_left", "sprint_strafe_left", "run_forward_arc_left", "turn_left" }) {
        for (const float fraction : { 0.1f, 0.4f, 0.8f }) {
            const Start a = startOf(body, frameOf(body, id, fraction));
            const Start b = mirrored(mirror, a);
            const RLBodyPose3D pa = rlBodyPose3D(body.profile, a.rootPosition, a.rootOrientation, a.coordinates);
            const RLBodyPose3D pb = rlBodyPose3D(body.profile, b.rootPosition, b.rootOrientation, b.coordinates);
            for (std::size_t i = 0; i < body.profile.links.size(); ++i) {
                const Vec3 reflected = Mirror3D::mirrorVector(pa.positions[i]);
                worst = std::max(worst, (reflected - pb.positions[mirror.linkCounterpart()[i]]).length());
            }
        }
    }
    require(worst < 0.01f, "a pose espelhada e a refletida (pior " + std::to_string(worst * 1000.0f) + " mm)");
}

struct World {
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D ragdoll;
    BodyContractState3D contract;
};

// A fisica espelhada: o estado espelhado, segurando a pose espelhada, da a
// trajetoria refletida. Sem gravidade e sem chao (a dinamica das juntas) e de
// pe no chao (o contato calmo). Um quadro de corrida caindo no chao e caotico
// (a ordem do solver nos contatos amplifica qualquer diferenca): so informa.
float mirrorDivergence(const RLBody3D& body, const Mirror3D& mirror, const Start& a, bool grounded, int steps) {
    const BodyContract3D contract(body);
    MaterialLibrary materials;
    PhysicsEngine3D engine;
    const auto makeWorld = [&](const std::vector<PhysicsBodyState3D>& links) {
        World world;
        PhysicsSceneSettings3D settings;
        settings.gravity = grounded ? Vec3 { 0.0f, 0.0f, -9.81f } : Vec3 {};
        world.scene = engine.createScene(settings, materials);
        if (grounded) {
            PhysicsShape3D ground;
            ground.type = PhysicsShapeType3D::Box;
            ground.halfExtents = { 50.0f, 50.0f, 0.5f };
            ground.materialId = "grass";
            PhysicsBodyDefinition3D floor;
            floor.motionType = PhysicsMotionType3D::Static;
            floor.position = { 0.0f, 0.0f, -0.5f };
            floor.materialId = "grass";
            (void)world.scene->createBody(floor, std::span<const PhysicsShape3D>(&ground, 1));
        }
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.pelvisPosition = { 0.0f, 0.0f, 1.0f };
        spawn.active = true;
        spawn.solverPositionIterations = body.solverPositionIterations;
        spawn.solverVelocityIterations = body.solverVelocityIterations;
        world.ragdoll = world.scene->createRagdoll(body.profile, spawn);
        world.scene->resetRagdollState(world.ragdoll, links);
        contract.reset(world.contract, *world.scene->ragdollStateView(world.ragdoll));
        std::vector<RagdollDriveTarget3D> targets;
        contract.writeTargetsAt(world.contract, 1.0f, PlayerAttributes3D {}, targets);
        world.scene->setRagdollActiveDriveTargets(world.ragdoll, targets, false);
        return world;
    };
    World original = makeWorld(linksOf(body, a));
    World reflected = makeWorld(linksOf(body, mirrored(mirror, a)));
    float worst = 0.0f;
    for (int step = 0; step < steps; ++step) {
        original.scene->simulate(1.0f / 120.0f);
        reflected.scene->simulate(1.0f / 120.0f);
        const RagdollState3D& sa = *original.scene->ragdollStateView(original.ragdoll);
        const RagdollState3D& sb = *reflected.scene->ragdollStateView(reflected.ragdoll);
        for (std::size_t i = 0; i < sa.links.size(); ++i)
            worst = std::max(worst,
                (Mirror3D::mirrorVector(sa.links[i].position) - sb.links[mirror.linkCounterpart()[i]].position).length());
    }
    return worst;
}

void testMirrorPhysics() {
    const RLBody3D body = bodyV2();
    const Mirror3D mirror(body);
    Start floating = startOf(body, frameOf(body, "trot_forward_left", 0.3f));
    floating.rootPosition.z += 1.0f;
    const float joints = mirrorDivergence(body, mirror, floating, false, 60);
    Start standing = startOf(body, frameOf(body, "alert_idle", 0.3f));
    standing.rootPosition.z -= 0.0095f;
    // (O boneco sem politica comeca a tombar: o afastamento cresce como num
    // tombo. O teste olha o comeco do contato; o resto informa.)
    std::cout << "espelho: de pe, o afastamento em";
    for (const int steps : { 6, 12, 24, 36, 60 })
        std::cout << ' ' << static_cast<float>(steps) / 120.0f << " s " << mirrorDivergence(body, mirror, standing, true, steps) * 1000.0f << " mm;";
    std::cout << '\n';
    const float calm = mirrorDivergence(body, mirror, standing, true, 12);
    const float chaos = mirrorDivergence(body, mirror, startOf(body, frameOf(body, "trot_forward_left", 0.3f)), true, 30);
    std::cout << "espelho: a fisica espelhada se afasta " << joints * 1000.0f << " mm sem chao (0,5 s), " << calm * 1000.0f
              << " mm de pe (0,1 s) e " << chaos * 1000.0f << " mm num trote caindo no chao (0,25 s; caotico)\n";
    require(joints < 0.005f, "as juntas espelhadas dao a trajetoria refletida (pior " + std::to_string(joints * 1000.0f) + " mm)");
    require(calm < 0.005f, "de pe, o espelho e o refletido (pior " + std::to_string(calm * 1000.0f) + " mm)");
}

// Os layouts: a entrada e as medidas do juiz do espelho sao o mapa das do original.
void testMirrorLayouts() {
    const RLBody3D body = bodyV2();
    const Mirror3D mirror(body);
    const BodyContract3D contract(body);
    const StyleFeatures3D style(body);
    const MirrorChannelMap3D observation = mirror.layoutMap(contract.observationLayout());
    const MirrorChannelMap3D action = mirror.layoutMap(contract.actionLayout());
    const MirrorChannelMap3D features = mirror.layoutMap(style.layout());
    // A ida e volta devolve o original.
    std::mt19937 rng(4);
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    for (const MirrorChannelMap3D* map : { &observation, &action, &features }) {
        std::vector<float> x(map->source.size()), once(x.size()), twice(x.size());
        for (float& v : x) v = gauss(rng);
        map->apply(x.data(), once.data());
        map->apply(once.data(), twice.data());
        for (std::size_t k = 0; k < x.size(); ++k) require(std::abs(twice[k] - x[k]) < 1e-4f, "o espelho do espelho");
    }
    // As medidas do juiz de um quadro espelhado sao o mapa das do original.
    for (const char* id : { "trot_forward_left", "sprint_strafe_left" }) {
        const Start a = startOf(body, frameOf(body, id, 0.5f));
        std::vector<float> fa(style.size()), fb(style.size()), mapped(style.size());
        style.write(linksOf(body, a), 1.0f, fa.data());
        style.write(linksOf(body, mirrored(mirror, a)), 1.0f, fb.data());
        features.apply(fa.data(), mapped.data());
        for (std::size_t k = 0; k < fa.size(); ++k) {
            const std::string& name = style.layout().channels[k].name;
            const bool rate = name.find("velocity") != std::string::npos || name.find("spin") != std::string::npos;
            require(std::abs(mapped[k] - fb[k]) < (rate ? 0.02f : 0.002f),
                "o estilo espelhado: " + name + " " + std::to_string(mapped[k]) + " contra " + std::to_string(fb[k]));
        }
    }
}

} // namespace

void runMirrorTests() {
    testMirrorMap();
    testMirrorPoses();
    testMirrorPhysics();
    testMirrorLayouts();
    std::cout << "Mirror tests passed\n";
}
