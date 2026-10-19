#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

RLBody3D loadBody() {
    return loadRLBody3D(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/rl/bodies/football_player_v2.json");
}

void testDefinition() {
    const RLBody3D body = loadBody();
    require(body.dofs.size() + body.lockedDofs.size() == ragdollDegreesOfFreedom3D(body.profile),
        "o corpo de RL atua ou trava cada eixo");
    require(body.dofs.size() == 37, "o jogador tem 37 eixos atuados");
    for (const RLBodyDof3D& dof : body.dofs) {
        require(dof.baseRadians >= dof.minimumRadians && dof.baseRadians <= dof.maximumRadians,
            "pose base fora do limite em " + dof.name);
        require(std::abs(dof.target(0.0f) - dof.baseRadians) < 1e-6f, "acao zero deveria segurar a base: " + dof.name);
        const float step = std::min(dof.baseRadians + dof.actionScale, dof.maximumRadians);
        require(std::abs(dof.target(1.0f) - step) < 1e-5f, "acao +1 deveria somar actionScale: " + dof.name);
        require(std::abs(dof.target(RLActionLimit) - std::min(dof.maximumRadians, dof.baseRadians + dof.actionScale * RLActionLimit)) < 1e-5f,
            "o maior pedido vai ao limite: " + dof.name);
        require(dof.target(-1000.0f) >= dof.minimumRadians && dof.target(1000.0f) <= dof.maximumRadians,
            "o alvo fica no curso da junta: " + dof.name);
        require(dof.maximumTorque > 0.0f && dof.maximumTorqueScale > 0.0f, "teto de torque ausente em " + dof.name);
    }
    // Os tetos de atleta da definicao (o perfil do jogo segue com os dele).
    for (const RLBodyDof3D& dof : body.dofs)
        if (dof.name == "LeftShin.twist") require(std::abs(dof.maximumTorque - 400.0f) < 1e-3f, "teto do joelho");
    require(body.standingPelvisHeight > 0.85f && body.standingPelvisHeight < 1.1f, "altura da pelve de pe implausivel");
    require(body.standingHeadHeight > 1.5f && body.standingHeadHeight < 1.85f, "altura da cabeca de pe implausivel");
    require(body.hash != 0, "o corpo precisa de identidade");
}

// A cinematica do corpo de RL e a do backend concordam: o corpo posto numa
// pose qualquer le de volta as mesmas coordenadas de junta.
void testPoseMatchesBackend() {
    const RLBody3D body = loadBody();
    std::mt19937 rng(7);
    std::vector<Vec3> coordinates(body.profile.links.size());
    for (const RLBodyDof3D& dof : body.dofs) {
        const float value = std::uniform_real_distribution<float>(dof.minimumRadians, dof.maximumRadians)(rng) * 0.8f;
        Vec3& q = coordinates[dof.linkIndex];
        if (dof.axis == RagdollAxis3D::Twist) q.x = value;
        else if (dof.axis == RagdollAxis3D::Swing1) q.y = value;
        else q.z = value;
    }
    const Quaternion root = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 0.7f)
        * body.profile.links.front().modelOrientation).normalized();
    const RLBodyPose3D pose = rlBodyPose3D(body.profile, { 1.0f, 2.0f, 1.5f }, root, coordinates);
    for (std::size_t i = 1; i < body.profile.links.size(); ++i) {
        const Vec3 back = rlJointCoordinates3D(body.profile, i,
            pose.orientations[static_cast<std::size_t>(body.profile.links[i].parentIndex)], pose.orientations[i]);
        require((back - coordinates[i]).length() < 1e-3f, "a cinematica nao inverte em " + body.profile.links[i].id);
    }

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
    rlBodyLinkStates3D(body.profile, { 1.0f, 2.0f, 1.5f }, root, {}, {}, coordinates, rest, links);
    scene->resetRagdollState(ragdoll, links);
    const RagdollState3D* state = scene->ragdollStateView(ragdoll);
    require(state != nullptr, "o estado do ragdoll deveria existir");
    for (const RLBodyDof3D& dof : body.dofs) {
        const Vec3& expected = coordinates[dof.linkIndex];
        const float wanted = dof.axis == RagdollAxis3D::Twist ? expected.x
            : dof.axis == RagdollAxis3D::Swing1 ? expected.y : expected.z;
        const float measured = state->joints[dof.linkIndex].positionRadians[static_cast<std::size_t>(dof.axis)];
        require(std::abs(measured - wanted) < 2e-3f, "o backend le outra coordenada em " + dof.name + ": "
            + std::to_string(measured) + " contra " + std::to_string(wanted));
    }
}

} // namespace

// As posturas: a natural e a pose base; a de alerta (o parado do jogo) e
// outra pose parada, mais baixa (joelhos dobrados), dentro do curso das juntas.
void testPostures() {
    const RLBody3D body = loadBody();
    require(body.postures.size() == 2 && body.postures[0].id == "natural" && body.postures[1].id == "alerta",
        "o corpo deveria ter as posturas natural e alerta");
    require(findRLPosture3D(body, "alerta") == 1 && findRLPosture3D(body, "sentado") == -1, "busca de postura");
    const RLPosture3D& natural = body.postures[0];
    const RLPosture3D& alert = body.postures[1];
    float moved = 0.0f;
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const RLBodyDof3D& dof = body.dofs[d];
        require(natural.dofRadians[d] == dof.baseRadians, "a postura natural e a pose base");
        require(alert.dofRadians[d] >= dof.minimumRadians && alert.dofRadians[d] <= dof.maximumRadians,
            "a postura de alerta fica dentro do curso de " + dof.name);
        moved = std::max(moved, std::abs(alert.dofRadians[d] - natural.dofRadians[d]));
    }
    require(moved > 0.1f, "a postura de alerta deveria ser outra pose");
    require(natural.standingPelvisHeight == body.standingPelvisHeight, "as medidas da base sao as da postura natural");
    require(alert.standingPelvisHeight < natural.standingPelvisHeight - 0.005f, "o alerta e mais baixo (joelhos dobrados)");
    require(alert.standingHeadHeight > alert.standingPelvisHeight + 0.5f, "o alerta fica de pe");
    for (const Vec3 up : { alert.pelvisUp, alert.chestUp, alert.headUp })
        require(std::abs(up.length() - 1.0f) < 1e-4f && up.z > 0.8f, "o para cima do alerta e quase vertical");
}

void runRLBodyTests() {
    testDefinition();
    testPoseMatchesBackend();
    testPostures();
    std::cout << "RL body tests passed\n";
}
