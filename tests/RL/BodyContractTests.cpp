#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

RLBody3D loadBody(const char* name) {
    return loadRLBody3D(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/rl/bodies/" + name);
}

const RLBodyDof3D* findDof(const RLBody3D& body, const std::string& name) {
    for (const RLBodyDof3D& dof : body.dofs)
        if (dof.name == name) return &dof;
    return nullptr;
}

// O corpo v2: a junta dos dedos, os pulsos travados e as posturas como dados.
void testBodyV2() {
    const RLBody3D v2 = loadBody("football_player_v2.json");
    // O perfil do jogo, com o pe inteiro (o v2 o divide em calcanhar e ponta).
    const RagdollProfile3D game = loadRagdollProfile3D(v2.profilePath.string());
    const auto gameLink = [&](const std::string& id) -> const RagdollLinkDefinition3D& {
        for (const RagdollLinkDefinition3D& link : game.links)
            if (link.id == id) return link;
        throw std::runtime_error("o perfil do jogo nao tem " + id);
    };
    require(v2.profile.links.size() == game.links.size() + 2, "o v2 tem os dois links da ponta");
    require(v2.leftToe >= 0 && v2.rightToe >= 0, "o v2 acha as pontas");
    require(v2.footParts[0].size() == 2 && v2.footParts[1].size() == 2, "cada pe pisa com o calcanhar e a ponta");
    require(v2.dofs.size() == 37, "o v2 atua 37 eixos (39 + 2 das pontas - 4 dos pulsos), tem " + std::to_string(v2.dofs.size()));
    require(v2.lockedDofs.size() == 4, "os 4 eixos dos pulsos ficam travados");
    for (const RLBodyDof3D& dof : v2.lockedDofs)
        require(dof.name.find("Hand") != std::string::npos, "so os pulsos travam: " + dof.name);
    for (const RLBodyDof3D& dof : v2.dofs) require(dof.name.find("Hand") == std::string::npos, "pulso atuado: " + dof.name);
    const RLBodyDof3D* toe = findDof(v2, "LeftToe.twist");
    require(toe != nullptr, "a ponta e atuada");
    require(std::abs(toe->minimumRadians + 60.0f * 3.14159265f / 180.0f) < 1e-4f
            && std::abs(toe->maximumRadians - 25.0f * 3.14159265f / 180.0f) < 1e-4f,
        "o curso da ponta e -60..25 graus");
    require(std::abs(toe->baseRadians) < 1e-6f, "a ponta parte reta");
    require(toe->maximumTorque > 0.0f && std::abs(toe->maximumTorque - 30.0f) < 1e-3f, "o teto da ponta e 30 N.m");
    require(v2.supportLink[static_cast<std::size_t>(v2.leftToe)] && v2.supportLink[static_cast<std::size_t>(v2.rightToe)],
        "a ponta e apoio");
    require(v2.postures.size() == 2 && v2.postures[1].id == "alerta", "o v2 tem a postura de alerta");

    // A divisao: a massa e o centro de massa do pe inteiro se mantem; o calcanhar
    // e a ponta da chuteira ficam onde estavam.
    for (std::size_t side = 0; side < 2; ++side) {
        const std::uint32_t foot = side == 0 ? v2.leftFoot : v2.rightFoot;
        const auto& heelLink = v2.profile.links[foot];
        const auto& toeLink = v2.profile.links[static_cast<std::size_t>(side == 0 ? v2.leftToe : v2.rightToe)];
        const auto& wholeLink = gameLink(heelLink.id);
        require(std::abs(heelLink.massFraction + toeLink.massFraction - wholeLink.massFraction) < 1e-6f, "a massa do pe se mantem");
        const Vec3 heelCom = heelLink.modelPosition + heelLink.modelOrientation.rotate(heelLink.centerOfMassLocal);
        const Vec3 com = (heelCom * heelLink.massFraction + toeLink.modelPosition * toeLink.massFraction)
            / (heelLink.massFraction + toeLink.massFraction);
        require((com - wholeLink.modelPosition).length() < 1e-4f, "o centro de massa do pe se mantem");
        // Ao longo do eixo da chuteira (girada ~8 graus no chao), onde fica o corte.
        const Vec3 axis = wholeLink.modelOrientation.rotate(wholeLink.collider.localOrientation.rotate({ 1.0f, 0.0f, 0.0f }));
        const auto extreme = [&](const RagdollLinkDefinition3D& link, float sign) {
            float best = -1e9f;
            for (const Vec3& q : link.collider.hullPoints) {
                const Vec3 m = link.modelPosition
                    + link.modelOrientation.rotate(link.collider.localPosition + link.collider.localOrientation.rotate(q));
                best = std::max(best, sign * dot(m, axis));
            }
            return sign * best;
        };
        require(std::abs(extreme(heelLink, -1.0f) - extreme(wholeLink, -1.0f)) < 1e-3f, "o calcanhar fica onde estava");
        require(std::abs(extreme(toeLink, 1.0f) - extreme(wholeLink, 1.0f)) < 1e-3f, "a ponta da chuteira fica onde estava");
        require(extreme(heelLink, 1.0f) < extreme(toeLink, -1.0f) + 2e-3f, "o calcanhar e a ponta se encontram no corte");
    }

    // O sinal da junta: positivo dobra a ponta para baixo.
    std::vector<Vec3> coordinates(v2.profile.links.size());
    const RLBodyPose3D flat = rlBodyPose3D(v2.profile, {}, v2.profile.links.front().modelOrientation, coordinates);
    coordinates[static_cast<std::size_t>(v2.leftToe)].x = 0.4f;
    const RLBodyPose3D down = rlBodyPose3D(v2.profile, {}, v2.profile.links.front().modelOrientation, coordinates);
    require(down.positions[static_cast<std::size_t>(v2.leftToe)].z < flat.positions[static_cast<std::size_t>(v2.leftToe)].z - 0.005f,
        "twist positivo deveria baixar a ponta");
}

// A cinematica do v2 (com as pontas) e a do backend concordam.
void testBodyV2Backend() {
    const RLBody3D body = loadBody("football_player_v2.json");
    std::mt19937 rng(11);
    std::vector<Vec3> coordinates(body.profile.links.size());
    for (const RLBodyDof3D& dof : body.dofs) {
        const float value = std::uniform_real_distribution<float>(dof.minimumRadians, dof.maximumRadians)(rng) * 0.8f;
        Vec3& q = coordinates[dof.linkIndex];
        (dof.axis == RagdollAxis3D::Twist ? q.x : dof.axis == RagdollAxis3D::Swing1 ? q.y : q.z) = value;
    }
    const Quaternion root = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -0.4f)
        * body.profile.links.front().modelOrientation).normalized();
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D settings;
    settings.gravity = {};
    settings.runJobsInline = true;
    auto scene = engine.createScene(settings, materials);
    RagdollSpawnDefinition3D spawn;
    spawn.pelvisPosition = { 0.0f, 0.0f, 3.0f };
    const RagdollHandle3D ragdoll = scene->createRagdoll(body.profile, spawn);
    std::vector<PhysicsBodyState3D> links;
    std::vector<Vec3> rest(coordinates.size());
    rlBodyLinkStates3D(body.profile, { 0.0f, 0.0f, 2.0f }, root, {}, {}, coordinates, rest, links);
    scene->resetRagdollState(ragdoll, links);
    const RagdollState3D* state = scene->ragdollStateView(ragdoll);
    require(state != nullptr && state->links.size() == body.profile.links.size(), "o backend monta o v2");
    for (const RLBodyDof3D& dof : body.dofs) {
        const Vec3& expected = coordinates[dof.linkIndex];
        const float wanted = dof.axis == RagdollAxis3D::Twist ? expected.x
            : dof.axis == RagdollAxis3D::Swing1 ? expected.y : expected.z;
        const float measured = state->joints[dof.linkIndex].positionRadians[static_cast<std::size_t>(dof.axis)];
        require(std::abs(measured - wanted) < 2e-3f, "o backend le outra coordenada no v2 em " + dof.name);
    }
}

// O layout do contrato: so o corpo, os comandos, o jogador e a memoria.
void testContractLayout() {
    const RLBody3D body = loadBody("football_player_v2.json");
    const BodyContract3D contract(body);
    const RLVectorLayout3D& layout = contract.observationLayout();
    std::set<std::string> names;
    std::size_t commands = 0, player = 0, history = 0;
    for (const RLChannel3D& channel : layout.channels) {
        require(names.insert(channel.name).second, "canal repetido: " + channel.name);
        for (const char* banned : { "reference.", "target.", "drill", "clip", "jump.zone", "speed.now" })
            require(channel.name.find(banned) == std::string::npos, "canal proibido na entrada do Corpo: " + channel.name);
        if (channel.name.starts_with("command.")) ++commands;
        if (channel.name.starts_with("player.")) ++player;
        if (channel.name.starts_with("history")) ++history;
    }
    const std::size_t a = body.dofs.size();
    require(commands == 8 && player == 7, "8 comandos e 7 dados do jogador");
    require(history == 3 * (3 * a + 6), "a memoria tem 3 decisoes");
    require(layout.size() == 1 + 9 + 2 * a + 15 + 6 + 8 + 10 + 2 + a + 8 + 7 + history,
        "o tamanho da entrada: " + std::to_string(layout.size()));
    require(layout.version == 2 && contract.actionSize() == 37, "o contrato v2 atua 37 eixos");
}

// O estado de um corpo no backend (no ar, sem contato): a pose de base virada
// yaw, com a pelve andando.
struct Scene {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D ragdoll;
};

const RagdollState3D& placeBody(Scene& world, const RLBody3D& body, float yaw, Vec3 localVelocity, float bend) {
    if (!world.scene) {
        PhysicsSceneSettings3D settings;
        settings.gravity = {};
        settings.runJobsInline = true;
        world.scene = world.engine.createScene(settings, world.materials);
        RagdollSpawnDefinition3D spawn;
        spawn.pelvisPosition = { 0.0f, 0.0f, 3.0f };
        world.ragdoll = world.scene->createRagdoll(body.profile, spawn);
    }
    std::vector<Vec3> coordinates = body.baseJointCoordinates;
    coordinates[body.leftFoot].x += bend;
    std::vector<Vec3> rates(coordinates.size());
    rates[body.leftFoot].x = 2.0f;
    const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
    const Quaternion root = (heading * body.profile.links.front().modelOrientation * body.baseRootRotation).normalized();
    std::vector<PhysicsBodyState3D> links;
    rlBodyLinkStates3D(body.profile, { 5.0f, -3.0f, 2.0f }, root, heading.rotate(localVelocity), {}, coordinates, rates,
        links);
    world.scene->resetRagdollState(world.ragdoll, links);
    return *world.scene->ragdollStateView(world.ragdoll);
}

// A entrada nao depende do rumo do mundo: o mesmo corpo e o mesmo comando
// virados dao a mesma entrada.
void testContractHeadingInvariance() {
    const RLBody3D body = loadBody("football_player_v2.json");
    const BodyContract3D contract(body);
    PlayerAttributes3D player;
    std::vector<float> first(contract.observationSize()), second(contract.observationSize());
    for (int pass = 0; pass < 2; ++pass) {
        const float yaw = pass == 0 ? 0.3f : 2.1f;
        Scene world;
        const RagdollState3D& ragdoll = placeBody(world, body, yaw, { 1.5f, 0.4f, 0.0f }, 0.2f);
        BodyContractState3D state;
        contract.reset(state, ragdoll);
        BodyCommand3D command;
        const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
        command.moveVelocity = heading.rotate({ 3.0f, -1.0f, 0.0f });
        command.faceHeading = yaw + 0.5f;
        command.faceRate = 0.7f;
        contract.writeObservation(ragdoll, command, player, state, pass == 0 ? first.data() : second.data());
    }
    for (std::size_t i = 0; i < first.size(); ++i)
        require(std::abs(first[i] - second[i]) < 2e-3f, "a entrada muda com o rumo do mundo em "
            + contract.observationLayout().channels[i].name + ": " + std::to_string(first[i]) + " contra "
            + std::to_string(second[i]));
    // O comando chega no referencial do rumo: 3 m/s para a frente e 1 para a direita.
    const auto at = [&](const std::string& name) {
        const auto& channels = contract.observationLayout().channels;
        for (std::size_t i = 0; i < channels.size(); ++i)
            if (channels[i].name == name) return first[i];
        throw std::runtime_error("canal ausente: " + name);
    };
    require(std::abs(at("command.move.x") - 0.5f) < 0.02f && std::abs(at("command.move.y") + 1.0f / 6.0f) < 0.02f,
        "o comando de movimento no referencial do rumo");
}

// A reta dos alvos e a velocidade pedida, nos tres modos; os pulsos na base.
void testContractRamp() {
    const RLBody3D body = loadBody("football_player_v2.json");
    Scene world;
    const RagdollState3D& ragdoll = placeBody(world, body, 0.0f, {}, 0.0f);
    PlayerAttributes3D player;
    for (const BodyVelocityTarget3D mode :
        { BodyVelocityTarget3D::Zero, BodyVelocityTarget3D::SampledSlope, BodyVelocityTarget3D::MeanSlope }) {
        BodyContractSettings3D settings;
        settings.velocityTarget = mode;
        const BodyContract3D contract(body, settings);
        BodyContractState3D state;
        contract.reset(state, ragdoll);
        const std::size_t count = contract.actionSize();
        std::vector<float> action1(count, 1.0f), mean1(count, 0.5f), action2(count, -1.0f), mean2(count, 0.8f);
        contract.beginDecision(state, action1, mean1);
        std::vector<RagdollDriveTarget3D> targets;
        for (std::uint32_t s = 0; s < 4; ++s) {
            contract.writeTargets(state, s, player, targets);
            require(targets.size() == count + 4, "os alvos incluem os pulsos travados");
            for (std::size_t d = 0; d < count; ++d) {
                const RLBodyDof3D& dof = body.dofs[d];
                const float from = std::clamp(ragdoll.joints[dof.linkIndex].positionRadians[static_cast<std::size_t>(dof.axis)],
                    dof.minimumRadians, dof.maximumRadians);
                const float expected = from + (dof.target(1.0f) - from) * static_cast<float>(s + 1) / 4.0f;
                require(std::abs(targets[d].positionRadians - expected) < 1e-4f, "a reta da primeira decisao em " + dof.name);
                require(targets[d].velocityRadiansPerSecond == 0.0f, "a primeira decisao nao pede velocidade");
            }
            for (std::size_t l = 0; l < 4; ++l)
                require(std::abs(targets[count + l].positionRadians - body.lockedDofs[l].baseRadians) < 1e-6f
                        && targets[count + l].velocityRadiansPerSecond == 0.0f,
                    "o pulso travado fica na base");
        }
        contract.beginDecision(state, action2, mean2);
        contract.writeTargets(state, 3, player, targets);
        for (std::size_t d = 0; d < count; ++d) {
            const RLBodyDof3D& dof = body.dofs[d];
            float expected = 0.0f;
            if (mode == BodyVelocityTarget3D::SampledSlope)
                expected = (dof.target(-1.0f) - dof.target(1.0f)) / settings.decisionSeconds;
            else if (mode == BodyVelocityTarget3D::MeanSlope)
                expected = (dof.target(0.8f) - dof.target(0.5f)) / settings.decisionSeconds;
            expected = std::clamp(expected, -settings.maximumTargetVelocity, settings.maximumTargetVelocity);
            require(std::abs(targets[d].velocityRadiansPerSecond - expected) < 1e-3f, "a velocidade pedida em " + dof.name);
            require(std::abs(targets[d].positionRadians - dof.target(-1.0f)) < 1e-5f, "o fim da reta e o alvo da acao");
        }
    }
}

// A memoria curta: a observacao seguinte traz o quadro da anterior na frente.
void testContractHistory() {
    const RLBody3D body = loadBody("football_player_v2.json");
    const BodyContract3D contract(body);
    Scene world;
    PlayerAttributes3D player;
    BodyContractState3D state;
    contract.reset(state, placeBody(world, body, 0.0f, {}, 0.0f));
    const std::size_t count = contract.actionSize();
    std::vector<float> observation(contract.observationSize());
    const auto& channels = contract.observationLayout().channels;
    const auto index = [&](const std::string& name) {
        for (std::size_t i = 0; i < channels.size(); ++i)
            if (channels[i].name == name) return i;
        throw std::runtime_error("canal ausente: " + name);
    };
    contract.writeObservation(placeBody(world, body, 0.0f, {}, 0.15f), {}, player, state, observation.data());
    const float firstBend = observation[index("LeftFoot.twist.position_from_base")];
    std::vector<float> action(count, 0.3f);
    contract.beginDecision(state, action, action);
    contract.writeObservation(placeBody(world, body, 0.0f, {}, -0.1f), {}, player, state, observation.data());
    require(std::abs(observation[index("history1.LeftFoot.twist.position")] - firstBend) < 1e-6f,
        "a memoria traz a posicao da decisao anterior");
    require(std::abs(observation[index("LeftFoot.twist.previous_action")] - 0.3f) < 1e-6f, "a acao anterior");
    require(std::abs(observation[index("history1.LeftFoot.twist.action")]) < 1e-6f,
        "o quadro anterior traz a acao de antes dele (zero no comeco)");
}

} // namespace

void runBodyContractTests() {
    testBodyV2();
    testBodyV2Backend();
    testContractLayout();
    testContractHeadingInvariance();
    testContractRamp();
    testContractHistory();
    std::cout << "Body contract tests passed\n";
}
