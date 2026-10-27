// A conta das forcas externas pela dinamica (G0, plano 6.2), em casos conhecidos:
// em queda livre o total e zero; um empurrao conhecido aparece inteiro; de pe e
// parado, os dois pes somam o peso e o centro de pressao fica na sola.
#include "Engine/BodyControl/Execution/Actuation3D.hpp"
#include "Engine/BodyControl/Observation/ContactWrench3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <string>

using namespace MatterEngine;

namespace {

int failures = 0;
constexpr double Dt = 1.0 / 120.0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::printf("FALHOU: %s\n", message.c_str());
    }
}

std::size_t findLink(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    return profile.links.size();
}

// O centro da sola no frame do link do pe.
Eigen::Vector3d soleCenter(const RagdollProfile3D& profile, std::size_t link) {
    const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    const std::vector<Vec3> points = ragdollColliderSolePoints3D(collider);
    for (const Vec3& p : points) mean += Eigen::Vector3d(p.x, p.y, p.z);
    mean /= std::max<std::size_t>(1, points.size());
    const Vec3 local = collider.localPosition + collider.localOrientation.rotate(Vec3 { float(mean.x()), float(mean.y()), float(mean.z()) });
    return { local.x, local.y, local.z };
}

// O torque muscular generalizado que o passo aplicou (motor + ff publicados).
Eigen::VectorXd appliedMuscle(const RagdollProfile3D& profile, const ArticulationModel3D& model,
    const ArticulationState3D& before, const RagdollState3D& after) {
    const ActuationDelivery3D delivery = actuationDelivery3D(profile, after);
    return articulationGeneralizedFromFeedforward3D(model, before, delivery.muscleTorque);
}

struct World {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D ragdoll;
    ArticulationModel3D model;
};

void makeWorld(World& world, const RagdollProfile3D& profile, bool ground, float pelvisHeight) {
    PhysicsSceneSettings3D settings;
    settings.gravity = { 0.0f, 0.0f, -9.81f };
    settings.workerThreadCount = 1;
    world.scene = world.engine.createScene(settings, world.materials);
    if (ground) {
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 20.0f, 20.0f, 0.25f };
        shape.materialId = "grass";
        PhysicsBodyDefinition3D body;
        body.motionType = PhysicsMotionType3D::Static;
        body.position = { 0.0f, 0.0f, -0.25f };
        body.materialId = "grass";
        (void)world.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
    }
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, pelvisHeight };
    spawn.active = true;
    world.ragdoll = world.scene->createRagdoll(profile, spawn);
    world.model = buildArticulationModel3D(profile, world.scene->ragdollEffectiveMass(world.ragdoll));
}

// Segura a pose de repouso com os motores do backend (rigidez do perfil, gravidade do backend).
void holdRestPose(World& world, const RagdollProfile3D& profile, float stiffness = 1.0f) {
    std::vector<RagdollDriveTarget3D> targets;
    for (std::size_t i = 1; i < profile.links.size(); ++i)
        for (int a = 0; a < 3; ++a) {
            if (!profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(i);
            target.axis = static_cast<RagdollAxis3D>(a);
            target.stiffnessScale = stiffness;
            target.dampingScale = stiffness;
            target.maximumTorqueScale = 2.0f;
            targets.push_back(target);
        }
    world.scene->setRagdollActiveDriveTargets(world.ragdoll, targets, true);
}

void freeFall(const RagdollProfile3D& profile) {
    World world;
    makeWorld(world, profile, false, 3.0f);
    holdRestPose(world, profile);
    const Eigen::Vector3d push(180.0, -90.0, 40.0);
    double worstQuiet = 0.0, worstPush = 0.0;
    for (int step = 0; step < 60; ++step) {
        const ArticulationState3D before = articulationStateFromRagdoll3D(world.model, *world.scene->ragdollStateView(world.ragdoll));
        const ArticulationKinematics3D k = computeArticulationKinematics3D(world.model, before);
        const ArticulationDynamics3D d = computeArticulationDynamics3D(world.model, k);
        const bool pushing = step >= 30;
        if (pushing) world.scene->applyRagdollRootForce(world.ragdoll, { float(push.x()), float(push.y()), float(push.z()) }, {});
        world.scene->simulate(static_cast<float>(Dt));
        const RagdollState3D& raw = *world.scene->ragdollStateView(world.ragdoll);
        const ArticulationState3D after = articulationStateFromRagdoll3D(world.model, raw);
        const ContactWrenchEstimate3D estimate = estimateContactWrench3D(world.model, before, k, d, after,
            appliedMuscle(profile, world.model, before, raw), {}, Dt);
        if (step < 5) continue;
        if (pushing && step > 31) worstPush = std::max(worstPush, (estimate.totalForce - push).norm());
        if (!pushing) worstQuiet = std::max(worstQuiet, estimate.totalForce.norm());
    }
    const double weight = world.model.totalMass * 9.81;
    std::printf("queda livre: forca externa total %.2f N (%.2f%% do peso); com empurrao de %.0f N: erro %.2f N\n", worstQuiet,
        100.0 * worstQuiet / weight, push.norm(), worstPush);
    check(worstQuiet < 0.02 * weight, "queda livre: forca externa total ~ 0");
    check(worstPush < 0.05 * push.norm(), "empurrao conhecido aparece inteiro");
}

void standing(const RagdollProfile3D& profile) {
    World world;
    makeWorld(world, profile, true, profile.standingRootHeightMeters + 0.01f);
    holdRestPose(world, profile, 2.0f);
    const std::size_t left = findLink(profile, "LeftFoot"), right = findLink(profile, "RightFoot");
    std::vector<ContactSupport3D> supports = {
        { static_cast<int>(left), soleCenter(profile, left), false },
        { static_cast<int>(right), soleCenter(profile, right), false },
    };
    double sumSupport = 0.0, sumTotal = 0.0, sumResidual = 0.0, worstCop = 0.0;
    int samples = 0;
    double pelvisStart = 0.0, pelvisEnd = 0.0;
    for (int step = 0; step < 300; ++step) {
        const RagdollState3D& rawBefore = *world.scene->ragdollStateView(world.ragdoll);
        const ArticulationState3D before = articulationStateFromRagdoll3D(world.model, rawBefore);
        const ArticulationKinematics3D k = computeArticulationKinematics3D(world.model, before);
        const ArticulationDynamics3D d = computeArticulationDynamics3D(world.model, k);
        world.scene->simulate(static_cast<float>(Dt));
        const RagdollState3D& raw = *world.scene->ragdollStateView(world.ragdoll);
        const ArticulationState3D after = articulationStateFromRagdoll3D(world.model, raw);
        for (ContactSupport3D& support : supports) {
            support.touching = false;
            for (const RagdollContactPoint3D& contact : raw.contacts)
                if (static_cast<int>(contact.linkIndex) == support.link) support.touching = true;
        }
        const ContactWrenchEstimate3D estimate = estimateContactWrench3D(world.model, before, k, d, after,
            appliedMuscle(profile, world.model, before, raw), supports, Dt);
        if (step == 180) pelvisStart = raw.links[0].position.z;
        pelvisEnd = raw.links[0].position.z;
        if (step < 180) continue;
        double vertical = 0.0;
        for (std::size_t s = 0; s < supports.size(); ++s) {
            vertical += estimate.supports[s].force.z();
            if (estimate.supports[s].hasCenterOfPressure && estimate.supports[s].force.z() > 0.1 * world.model.totalMass * 9.81) {
                const Eigen::Vector3d offset = estimate.supports[s].centerOfPressure - estimate.supports[s].pointWorld;
                worstCop = std::max(worstCop, std::hypot(offset.x(), offset.y()));
            }
        }
        sumSupport += vertical;
        sumTotal += estimate.totalForce.z();
        sumResidual += estimate.residualFraction;
        ++samples;
    }
    const double weight = world.model.totalMass * 9.81;
    std::printf("de pe parado: pes somam %.1f N, total %.1f N, peso %.1f N; residuo medio %.3f; CoP a no maximo %.3f m do centro da sola; pelve %.3f -> %.3f m\n",
        sumSupport / samples, sumTotal / samples, weight, sumResidual / samples, worstCop, pelvisStart, pelvisEnd);
    check(std::abs(pelvisEnd - pelvisStart) < 0.01, "o corpo ficou parado de pe (o caso conhecido vale)");
    check(std::abs(sumTotal / samples - weight) < 0.03 * weight, "de pe: forca externa total = peso");
    check(std::abs(sumSupport / samples - weight) < 0.05 * weight, "de pe: os dois pes somam o peso");
    check(worstCop < 0.16, "de pe: o centro de pressao fica na sola");
}

} // namespace

int main() {
    const RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    freeFall(profile);
    standing(profile);
    std::printf(failures ? "forcas de contato: %d falha(s)\n" : "forcas de contato: ok\n", failures);
    return failures ? 1 : 0;
}
