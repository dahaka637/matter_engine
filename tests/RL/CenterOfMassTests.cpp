#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Rewards/BodyReward3D.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

float distance(Vec3 a, Vec3 b) {
    return (a - b).length();
}

RLBody3D loadBody() {
    return loadRLBody3D(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/rl/bodies/football_player_v2.json");
}

// Um pe rolando sem escorregar sobre o chao: o ponto de contato tem velocidade
// zero pelo centro de massa do link. Tratar a origem como centro de massa (a
// conta de antes) erra por omega x (R c) quando o centro de massa e deslocado.
void testRollingContactHasNoSlip() {
    RagdollLinkDefinition3D link;
    link.centerOfMassLocal = { 0.03f, 0.0f, -0.02f };
    PhysicsBodyState3D state;
    state.orientation = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, 0.4f);
    // O contato e o pivo de uma rotacao rigida em volta do eixo y por ele.
    const Vec3 contact { 0.0f, 0.0f, 0.0f };
    const Vec3 com { 0.0f, 0.0f, 0.1f };
    state.angularVelocity = { 0.0f, 6.0f, 0.0f };
    state.linearVelocity = cross(state.angularVelocity, com - contact);
    state.position = com - state.orientation.rotate(link.centerOfMassLocal);

    require(distance(rlLinkCenterOfMass3D(link, state), com) < 1e-6f, "o centro de massa do link");
    require(rlLinkPointVelocity3D(link, state, contact).length() < 1e-5f, "o pe rolando nao escorrega");
    const Vec3 oldWay = state.linearVelocity + cross(state.angularVelocity, contact - state.position);
    require(oldWay.length() > 0.05f, "a conta antiga (a origem como centro de massa) acusaria um escorregar falso");
}

// Deslocar o centro de massa mantendo o mesmo movimento rigido nao muda a
// velocidade de nenhum ponto: so a velocidade do centro de massa acompanha.
void testPointVelocityDoesNotDependOnTheCenterOfMass() {
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
    for (int trial = 0; trial < 20; ++trial) {
        const Quaternion orientation = Quaternion::fromAxisAngle({ unit(rng), unit(rng), unit(rng) + 2.0f }, unit(rng));
        const Vec3 origin { unit(rng), unit(rng), unit(rng) };
        const Vec3 originVelocity { unit(rng), unit(rng), unit(rng) };
        const Vec3 omega { 3.0f * unit(rng), 3.0f * unit(rng), 3.0f * unit(rng) };
        const Vec3 point { unit(rng), unit(rng), unit(rng) };
        const Vec3 expected = originVelocity + cross(omega, point - origin);
        for (const Vec3 c : { Vec3 {}, Vec3 { 0.05f, -0.02f, 0.03f }, Vec3 { -0.1f, 0.04f, 0.0f } }) {
            RagdollLinkDefinition3D link;
            link.centerOfMassLocal = c;
            PhysicsBodyState3D state;
            state.position = origin;
            state.orientation = orientation;
            state.angularVelocity = omega;
            state.linearVelocity = originVelocity + cross(omega, orientation.rotate(c));
            require(distance(rlLinkPointVelocity3D(link, state, point), expected) < 1e-4f,
                "a velocidade de um ponto muda com o centro de massa escolhido");
        }
    }
}

// O centro de massa do corpo e a soma ponderada dos centros de massa dos
// links, e o corpo v2 tem links de centro de massa deslocado (o calcanhar).
void testBodyCenterOfMass() {
    const RLBody3D body = loadBody();
    bool shifted = false;
    for (const RagdollLinkDefinition3D& link : body.profile.links) shifted = shifted || link.centerOfMassLocal.length() > 0.01f;
    require(shifted, "o corpo v2 tem links de centro de massa deslocado");

    std::vector<Vec3> joints(body.profile.links.size());
    std::vector<Vec3> rates(body.profile.links.size());
    std::vector<PhysicsBodyState3D> links;
    const Quaternion root = body.profile.links.front().modelOrientation;
    rlBodyLinkStates3D(body.profile, { 10.0f, 20.0f, 1.0f }, root, {}, {}, joints, rates, links);
    Vec3 manual {};
    float mass = 0.0f;
    for (std::size_t i = 0; i < links.size(); ++i) {
        const float m = body.profile.links[i].massFraction;
        manual += (links[i].position + links[i].orientation.rotate(body.profile.links[i].centerOfMassLocal)) * m;
        mass += m;
    }
    manual = manual / mass;
    require(distance(rlBodyCenterOfMass3D(body.profile, links), manual) < 1e-5f, "o centro de massa do corpo");
    // Um corpo parado tem velocidade zero em todos os pontos.
    require(rlBodyCenterOfMassVelocity3D(body.profile, links).length() < 1e-5f, "o corpo parado nao se move");
}

// As velocidades de um estado reconstruido sao as do centro de massa: tirando
// omega x (R c), sobra a derivada da posicao da origem, medida por fora com
// outro passo e diferenca central.
void testReconstructedVelocitiesAreCenterOfMass() {
    const RLBody3D body = loadBody();
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
    std::vector<Vec3> joints(body.profile.links.size());
    std::vector<Vec3> rates(body.profile.links.size());
    for (const RLBodyDof3D& dof : body.dofs) {
        Vec3& q = joints[dof.linkIndex];
        Vec3& qd = rates[dof.linkIndex];
        float* value = dof.axis == RagdollAxis3D::Twist ? &q.x : dof.axis == RagdollAxis3D::Swing1 ? &q.y : &q.z;
        float* rate = dof.axis == RagdollAxis3D::Twist ? &qd.x : dof.axis == RagdollAxis3D::Swing1 ? &qd.y : &qd.z;
        // No meio do curso: perto do fim a pose "a frente" seria cortada pela
        // junta e a "atras" nao, e a diferenca central nao mediria a velocidade.
        *value = dof.minimumRadians + (0.5f + 0.25f * unit(rng)) * (dof.maximumRadians - dof.minimumRadians);
        *rate = 2.0f * unit(rng);
    }
    const Quaternion root = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 0.9f)
        * body.profile.links.front().modelOrientation).normalized();
    const Vec3 rootVelocity { 1.5f, -0.7f, 0.2f };
    const Vec3 rootSpin { 0.4f, -1.1f, 2.0f };
    std::vector<PhysicsBodyState3D> links;
    rlBodyLinkStates3D(body.profile, {}, root, rootVelocity, rootSpin, joints, rates, links);

    constexpr float h = 2.0e-3f;
    std::vector<Vec3> ahead(joints), behind(joints);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        ahead[i] += rates[i] * h;
        behind[i] -= rates[i] * h;
    }
    const RLBodyPose3D forward = rlBodyPose3D(body.profile, rootVelocity * h, rlQuaternionExp3D(rootSpin * h) * root, ahead);
    const RLBodyPose3D backward =
        rlBodyPose3D(body.profile, rootVelocity * -h, rlQuaternionExp3D(rootSpin * -h) * root, behind);
    float worst = 0.0f;
    std::size_t worstLink = 0;
    for (std::size_t i = 0; i < links.size(); ++i) {
        const Vec3 originVelocity = (forward.positions[i] - backward.positions[i]) * (0.5f / h);
        const Vec3 fromState = links[i].linearVelocity
            - cross(links[i].angularVelocity, links[i].orientation.rotate(body.profile.links[i].centerOfMassLocal));
        const float error = distance(fromState, originVelocity);
        if (error > worst) {
            worst = error;
            worstLink = i;
        }
    }
    require(worst < 0.02f, "a velocidade reconstruida nao e a do centro de massa em "
        + body.profile.links[worstLink].id + " (" + std::to_string(worst) + " m/s)");
}

// O escorregar do corpo (o acumulador de custos) de um pe rolando sobre o
// calcanhar dividido e ~0; com o pe parado tambem.
void testSlipCostOfRollingHeel() {
    const RLBody3D body = loadBody();
    std::vector<Vec3> joints(body.profile.links.size());
    std::vector<Vec3> rates(body.profile.links.size());
    RagdollState3D state;
    rlBodyLinkStates3D(body.profile, { 0.0f, 0.0f, 1.0f }, body.profile.links.front().modelOrientation, {}, {},
        joints, rates, state.links);
    state.joints.resize(body.profile.links.size());

    const std::uint32_t heel = body.footParts[0][0];
    const RagdollLinkDefinition3D& link = body.profile.links[heel];
    PhysicsBodyState3D& foot = state.links[heel];
    // O pivo: um ponto 4 cm abaixo do centro de massa, no plano do pe; rotacao
    // rigida em volta do eixo y por ele.
    const Vec3 com = rlLinkCenterOfMass3D(link, foot);
    const Vec3 contact = com - Vec3 { 0.0f, 0.0f, 0.04f };
    foot.angularVelocity = { 0.0f, 5.0f, 0.0f };
    foot.linearVelocity = cross(foot.angularVelocity, com - contact);
    RagdollContactPoint3D point;
    point.linkIndex = heel;
    point.position = contact;
    point.normal = { 0.0f, 0.0f, 1.0f };
    point.normalImpulseNewtonSeconds = 1.0f;
    point.otherMotion = RagdollContactMotion3D::Static;
    state.contacts.push_back(point);

    BodyCostAccumulator3D costs;
    costs.begin(state, body);
    costs.addSubstep(body, state, 1.0f / 120.0f, 800.0f);
    const BodyCosts3D result = costs.finish(1.0f / 120.0f, 800.0f);
    require(result.slip < 1e-4f, "o calcanhar rolando nao escorrega (" + std::to_string(result.slip) + " m/s)");

    // O mesmo contato com o pe girando em volta de outro ponto escorrega.
    foot.linearVelocity = cross(foot.angularVelocity, com - contact) + Vec3 { 0.3f, 0.0f, 0.0f };
    BodyCostAccumulator3D sliding;
    sliding.begin(state, body);
    sliding.addSubstep(body, state, 1.0f / 120.0f, 800.0f);
    require(sliding.finish(1.0f / 120.0f, 800.0f).slip > 0.25f, "o pe deslizando escorrega");
}

} // namespace

void runCenterOfMassTests() {
    testRollingContactHasNoSlip();
    testPointVelocityDoesNotDependOnTheCenterOfMass();
    testBodyCenterOfMass();
    testReconstructedVelocitiesAreCenterOfMass();
    testSlipCostOfRollingHeel();
}
