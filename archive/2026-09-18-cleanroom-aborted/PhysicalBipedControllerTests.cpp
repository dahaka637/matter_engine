#include "Engine/Control/BipedBalanceController3D.hpp"
#include "Engine/Control/BipedMath3D.hpp"
#include "Engine/Control/BipedStateEstimator3D.hpp"
#include "Engine/Control/ResidualGetUpAssist3D.hpp"
#include "Engine/Control/PhysicalBipedController3D.hpp"
#include "Engine/Locomotion/BipedStepPlanner3D.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace MatterEngine;

namespace {

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(1);
}

void expect(bool value, const std::string& message) {
    if (!value) fail(message);
}

void expectNear(float actual, float expected, float epsilon,
                const std::string& message) {
    if (std::abs(actual - expected) > epsilon) {
        fail(message + " actual=" + std::to_string(actual)
            + " expected=" + std::to_string(expected));
    }
}

RagdollLinkDefinition3D link(std::string id, int parent, Vec3 pos,
                             float massFraction) {
    RagdollLinkDefinition3D result;
    result.id = std::move(id);
    result.parentIndex = parent;
    result.modelPosition = pos;
    result.modelOrientation = {};
    result.massFraction = massFraction;
    result.centerOfMassLocal = {};
    result.collider.shape = RagdollColliderShape3D::Capsule;
    result.collider.radiusMeters = 0.05f;
    result.collider.lengthMeters = 0.30f;
    result.inboundJoint.anchorModelPosition = pos;
    if (parent >= 0) {
        for (auto& axis : result.inboundJoint.axes) {
            axis.enabled = true;
            axis.minimumRadians = -1.5f;
            axis.maximumRadians = 1.5f;
            axis.stiffness = 100.0f;
            axis.damping = 20.0f;
            axis.maximumTorque = 200.0f;
        }
    }
    return result;
}

RagdollProfile3D makeProfile() {
    RagdollProfile3D p;
    p.id = "test-human";
    p.totalMassKg = 75.0f;
    p.standingRootHeightMeters = 1.0f;
    p.links.push_back(link("Pelvis", -1, {0, 0, 1.00f}, 0.22f));
    p.links.push_back(link("Chest", 0, {0, 0, 1.32f}, 0.22f));
    p.links.push_back(link("LeftThigh", 0, {0, 0.14f, 0.82f}, 0.12f));
    p.links.push_back(link("LeftCalf", 2, {0, 0.14f, 0.43f}, 0.08f));
    p.links.push_back(link("LeftFoot", 3, {0.08f, 0.14f, 0.08f}, 0.04f));
    p.links.push_back(link("RightThigh", 0, {0, -0.14f, 0.82f}, 0.12f));
    p.links.push_back(link("RightCalf", 5, {0, -0.14f, 0.43f}, 0.08f));
    p.links.push_back(link("RightFoot", 6, {0.08f, -0.14f, 0.08f}, 0.04f));
    p.links.push_back(link("LeftHand", 1, {0.0f, 0.45f, 1.10f}, 0.04f));
    p.links.push_back(link("RightHand", 1, {0.0f, -0.45f, 1.10f}, 0.04f));
    for (std::size_t i : {std::size_t{4}, std::size_t{7}}) {
        p.links[i].collider.shape = RagdollColliderShape3D::Box;
        p.links[i].collider.boxHalfExtents = {0.14f, 0.065f, 0.04f};
        p.links[i].collider.contactSensor = true;
    }
    return p;
}

BipedRig3D makeRig() {
    BipedRig3D rig;
    rig.valid = true;
    rig.pelvis = 0;
    rig.torso = 1;
    rig.upperLeg = {2, 5};
    rig.lowerLeg = {3, 6};
    rig.foot = {4, 7};
    rig.hand = {8, 9};
    rig.legLengthMeters = 0.90f;
    rig.nominalStepWidthMeters = 0.28f;
    return rig;
}

RagdollState3D makeState(const RagdollProfile3D& p) {
    RagdollState3D s;
    s.links.resize(p.links.size());
    s.joints.resize(p.links.size());
    for (std::size_t i = 0; i < p.links.size(); ++i) {
        s.links[i].position = p.links[i].modelPosition;
        s.links[i].orientation = p.links[i].modelOrientation;
    }
    s.active = true;
    return s;
}

BipedObservation3D standingObservation(const BipedRig3D& rig) {
    BipedObservation3D o;
    o.valid = true;
    o.centerOfMassWorld = {0.0f, 0.0f, 0.95f};
    o.centerOfMassHeightMeters = 0.95f;
    o.capturePointWorld = {0.0f, 0.0f, 0.0f};
    o.captureMarginMeters = 0.08f;
    o.torsoUpDot = 1.0f;
    o.support.mode = BipedSupportMode3D::DoubleSupport;
    o.support.centerWorld = {0, 0, 0};
    o.support.referenceHeightMeters = 0.0f;
    o.support.totalNormalLoadNewtons = 75.0f * 9.81f;
    o.feet[0].linkIndex = rig.foot[0];
    o.feet[1].linkIndex = rig.foot[1];
    o.feet[0].contact = true;
    o.feet[1].contact = true;
    o.feet[0].normalLoadNewtons = o.support.totalNormalLoadNewtons * 0.5f;
    o.feet[1].normalLoadNewtons = o.support.totalNormalLoadNewtons * 0.5f;
    o.feet[0].loadRatio = 0.5f;
    o.feet[1].loadRatio = 0.5f;
    o.feet[0].solePositionWorld = {0.0f, 0.14f, 0.0f};
    o.feet[1].solePositionWorld = {0.0f, -0.14f, 0.0f};
    o.feet[0].contactCenterWorld = o.feet[0].solePositionWorld;
    o.feet[1].contactCenterWorld = o.feet[1].solePositionWorld;
    return o;
}

void testEstimatorLoadRatio() {
    const auto profile = makeProfile();
    const auto rig = makeRig();
    auto state = makeState(profile);
    constexpr float dt = 0.01f;
    const float load = profile.totalMassKg * 9.81f * 0.5f;
    state.contacts.push_back({4, {0.08f, 0.14f, 0.0f}, {0,0,1}, load*dt, 0.0f});
    state.contacts.push_back({7, {0.08f,-0.14f, 0.0f}, {0,0,1}, load*dt, 0.0f});

    BipedStateEstimator3D estimator;
    RagdollDynamics3D dynamics;
    const auto o = estimator.observe(profile, rig, state, dynamics, dt);
    expect(o.valid, "state estimator must accept finite synthetic state");
    expect(o.support.mode == BipedSupportMode3D::DoubleSupport,
           "equal foot contacts must be double support");
    expectNear(o.feet[0].loadRatio, 0.5f, 1e-4f,
               "left load ratio must preserve measured 50/50 support");
    expectNear(o.feet[1].loadRatio, 0.5f, 1e-4f,
               "right load ratio must preserve measured 50/50 support");
}

void testNoLiftoffAt5050() {
    const auto profile = makeProfile();
    const auto rig = makeRig();
    auto o = standingObservation(rig);

    BipedStepPlanner3D planner;
    BipedIntent3D intent;
    intent.locomotionEnabled = true;
    intent.allowStepping = true;
    intent.desiredVelocityBody = {1.0f, 0.0f, 0.0f};

    BipedBalanceDecision3D balance;
    balance.strategy = BipedBalanceStrategy3D::AnkleHip;

    BipedTerrainQuery3D terrain;
    terrain.sampleGround = [](Vec3 p, float, float, BipedTerrainHit3D& hit) {
        hit.valid = true;
        hit.position = {p.x, p.y, 0.0f};
        hit.normal = {0,0,1};
        return true;
    };

    BipedStepPlan3D plan;
    for (int i = 0; i < 100; ++i) {
        plan = planner.update(profile, rig, o, intent, balance, terrain, 0.01f);
        expect(plan.phase != BipedStepPhase3D::Swing,
               "50/50 support must never authorize swing liftoff");
    }
    expect(plan.phase == BipedStepPhase3D::Transfer,
           "planner should remain in transfer while load is 50/50");

    // First selected swing foot is left; therefore right stance must receive load.
    o.feet[0].loadRatio = 0.15f;
    o.feet[1].loadRatio = 0.85f;
    o.feet[0].normalLoadNewtons = profile.totalMassKg*9.81f*0.15f;
    o.feet[1].normalLoadNewtons = profile.totalMassKg*9.81f*0.85f;
    plan = planner.update(profile, rig, o, intent, balance, terrain, 0.01f);
    expect(plan.phase == BipedStepPhase3D::Swing,
           "measured weight transfer must authorize the planned swing");
    expect(std::abs(plan.landingSoleWorld.y) >= rig.nominalStepWidthMeters*0.35f,
           "calm landing must preserve a functional lateral base width");
}

void testRecoveryRequestsStep() {
    const auto rig = makeRig();
    auto o = standingObservation(rig);
    o.captureMarginMeters = -0.10f;

    BipedIntent3D intent;
    intent.allowStepping = true;
    BipedBalanceController3D balance;
    const auto d = balance.assess(rig, o, intent);
    expect(d.strategy == BipedBalanceStrategy3D::Step && d.mustStep,
           "capture point outside the safe support region must request a step");

    intent.allowStepping = false;
    const auto noStep = balance.assess(rig, o, intent);
    expect(noStep.strategy == BipedBalanceStrategy3D::Fall,
           "uncapturable state with stepping disabled must not invent external support");
}

void testResidualAssistGates() {
    ResidualGetUpAssist3D residual;
    ResidualGetUpAssistInput3D in;
    in.inGetUp = true;
    in.eligibleStage = true;
    in.totalMassKg = 75.0f;
    in.supportLoadNewtons = 75.0f*9.81f;
    in.actuatorDemandPeak = 0.95f;
    in.centerOfMassVerticalVelocity = 0.0f;
    in.desiredVerticalVelocity = 0.5f;
    in.deltaTime = 0.10f;

    in.airborne = true;
    expect(residual.update(in).lengthSquared() == 0.0f,
           "airborne residual must be exactly zero");

    residual.reset();
    in.airborne = false;
    in.inGetUp = false;
    expect(residual.update(in).lengthSquared() == 0.0f,
           "normal standing/walking modes must never receive residual force");

    residual.reset();
    in.inGetUp = true;
    Vec3 f;
    for (int i = 0; i < 4; ++i) f = residual.update(in);
    expect(f.z > 0.0f && f.x == 0.0f && f.y == 0.0f,
           "eligible stalled get-up may receive vertical-only residual support");
}


void testControllerNeverUsesExternalBalanceForce() {
    auto profile = makeProfile();
    // Procedural get-up needs body contacts in the runtime; making all links
    // sensors here also exercises the same profile contract used by Workbench.
    for (auto& l : profile.links) l.collider.contactSensor = true;
    auto state = makeState(profile);
    constexpr float dt = 0.01f;
    const float each = profile.totalMassKg*9.81f*0.5f;
    state.contacts.push_back({4,{0.08f,0.14f,0},{0,0,1},each*dt,0});
    state.contacts.push_back({7,{0.08f,-0.14f,0},{0,0,1},each*dt,0});

    BipedTerrainQuery3D terrain;
    terrain.sampleGround = [](Vec3 p, float, float, BipedTerrainHit3D& hit) {
        hit.valid = true;
        hit.position = {p.x,p.y,0.0f};
        hit.normal = {0,0,1};
        return true;
    };
    RagdollDynamics3D dynamics;
    PhysicalBipedController3D controller;
    controller.reset(profile, state);
    controller.update(profile, state, dynamics, terrain, dt);
    expect(controller.output().externalWrenches.empty(),
           "standing controller must emit zero external balance wrenches");
    expect(controller.output().gravityCompensationEnabled,
           "normal controller should request joint-only gravity compensation");

    expect(controller.requestWalk(state, 1.0f),
           "walk request should be accepted in a standing state");
    for (int i=0;i<20;++i)
        controller.update(profile, state, dynamics, terrain, dt);
    expect(controller.output().externalWrenches.empty(),
           "walking/transfer controller must emit zero external balance wrenches");

    state.contacts.clear();
    for (int i=0;i<25;++i)
        controller.update(profile, state, dynamics, terrain, dt);
    expect(controller.output().externalWrenches.empty(),
           "airborne/falling controller must emit zero external balance wrenches");
}

void testCapturePointFormula() {
    auto rig = makeRig();
    auto profile = makeProfile();
    auto state = makeState(profile);
    constexpr float dt = 0.01f;
    const float each = profile.totalMassKg*9.81f*0.5f;
    state.contacts.push_back({4,{0.08f,0.14f,0},{0,0,1},each*dt,0});
    state.contacts.push_back({7,{0.08f,-0.14f,0},{0,0,1},each*dt,0});
    for (auto& b : state.links) b.linearVelocity = {1.0f,0,0};
    BipedStateEstimator3D estimator;
    RagdollDynamics3D dynamics;
    auto o = estimator.observe(profile, rig, state, dynamics, dt);
    const float expectedDelta = 1.0f/std::sqrt(9.81f/std::max(o.centerOfMassHeightMeters,0.15f));
    expectNear(o.capturePointWorld.x-o.centerOfMassWorld.x,
               expectedDelta, 1e-4f,
               "capture point must follow x + xdot/sqrt(g/h)");
}

} // namespace

int main() {
    testEstimatorLoadRatio();
    testNoLiftoffAt5050();
    testRecoveryRequestsStep();
    testResidualAssistGates();
    testControllerNeverUsesExternalBalanceForce();
    testCapturePointFormula();
    std::cout << "Matter physical biped tests: PASS\n";
    return 0;
}
