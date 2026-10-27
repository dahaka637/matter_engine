// A bancada de ficar de pe (G2 da locomocao fisica nova): o executor de corpo
// inteiro (QP) no laco, ajuda ZERO (nenhuma forca externa do controle), o
// atuador A (torque explicito), sobre o gramado. Mede a pelve, o escorregar
// dos pes, o QP (status, iteracoes, tempo) e, com --empurroes, o maior impulso
// aguentado sem cair em 8 direcoes (sem passo ainda).
//
// uso: MatterStandBench [--segundos 60] [--empurroes] [--dump arquivo --cada 4]
#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/BodyControl/Execution/Actuation3D.hpp"
#include "Engine/BodyControl/Execution/WholeBodyQP3D.hpp"
#include "Engine/BodyControl/Observation/ContactWrench3D.hpp"
#include "Engine/BodyControl/Planning/DcmWalker3D.hpp"
#include "Engine/BodyControl/Planning/HopPlanner3D.hpp"
#include "Engine/BodyControl/Planning/StepRecovery3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <fstream>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

constexpr double Dt = 1.0 / 120.0;
constexpr double Pi = 3.14159265358979323846;

Eigen::Vector3d toEigen(Vec3 v) { return { v.x, v.y, v.z }; }

std::size_t findLink(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    return profile.links.size();
}

// Os 4 cantos da sola (no frame do link do pe): o retangulo dos pontos da sola.
std::vector<Eigen::Vector3d> soleCorners(const RagdollProfile3D& profile, std::size_t link) {
    const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
    std::vector<Eigen::Vector3d> points;
    for (const Vec3& p : ragdollColliderSolePoints3D(collider)) {
        const Vec3 local = collider.localPosition + collider.localOrientation.rotate(p);
        points.emplace_back(local.x, local.y, local.z);
    }
    Eigen::Vector3d minimum = points.front(), maximum = points.front();
    for (const Eigen::Vector3d& p : points) {
        minimum = minimum.cwiseMin(p);
        maximum = maximum.cwiseMax(p);
    }
    // O pe: x para a frente no link? Usa os dois eixos de maior extensao.
    std::vector<Eigen::Vector3d> corners;
    double meanZ = 0.0;
    for (const Eigen::Vector3d& p : points) meanZ += p.z();
    meanZ /= double(points.size());
    (void)meanZ;
    const Eigen::Vector3d extent = maximum - minimum;
    int thin = 0;
    for (int a = 1; a < 3; ++a) if (extent[a] < extent[thin]) thin = a;
    const int first = (thin + 1) % 3, second = (thin + 2) % 3;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d corner = 0.5 * (minimum + maximum);
        corner[thin] = (thin == 2 || true) ? (std::abs(minimum[thin]) > std::abs(maximum[thin]) ? minimum[thin] : maximum[thin]) : corner[thin];
        corner[first] = (i & 1) ? maximum[first] : minimum[first];
        corner[second] = (i & 2) ? maximum[second] : minimum[second];
        corners.push_back(corner);
    }
    return corners;
}

struct Stand {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D ragdoll;
    ArticulationModel3D model;
    WholeBodyQP3D qp;
    std::size_t feet[2] {};
    std::vector<Eigen::Vector3d> corners[2];
    Eigen::Vector3d comTarget;
    Eigen::Matrix3d pelvisTarget, chestTarget, headTarget;
    std::size_t chest = 0, head = 0;
    Eigen::VectorXd torqueLimit;
    std::vector<Eigen::Vector3d> referencePose;
    Eigen::VectorXd predictedAcceleration, previousVelocity, previousTorque;
    std::optional<ArticulationState3D> previousState;
    std::optional<ArticulationState3D> positionHistory;
    Eigen::Vector3d plannedFootForce[2];
    Eigen::Vector3d plannedCop[2];
    int debugTicks = 0;
    int logTick = 0;
    // O passo de recuperacao (G3).
    bool steppingEnabled = false;
    bool stepping = false;
    int swingFoot = -1;
    int steps = 0;
    // A saida pela ponta (nota 29): quantas e a soma do angulo da sola na decolagem.
    int toeOffs = 0;
    double toeOffAngleSum = 0.0, toeOffLastAngle = 0.0;
    // Quanto o CoP pedido pelo DCM fica fora do apoio (o corte da projecao), na marcha.
    double copClampSum = 0.0, copClampWorst = 0.0;
    // O torque pedido no referencial do motor acima do teto (o backend recorta): ticks e pior razao.
    int motorClippedTicks = 0;
    double motorWorstRatio = 0.0;
    // A decolagem (nota 31): a forca planejada no pe no tick anterior e quanto ele ainda toca depois.
    int liftoffs = 0, separations = 0, separationTicks = 0, separationSum = 0;
    double liftoffForceSum = 0.0;
    bool separationPending = false;
    // O salto no lugar (G4, STANDHOP=voo,apoio,quando): 0 esperando, 1 apoio, 2 voo.
    bool hopping = false;
    HopSettings3D hopSettings;
    double hopStart = 1.0, hopTime = 0.0, hopEntrySpeed = 0.0, hopEntryHeight = 0.0;
    // 3: agachando antes do primeiro (de pe, a perna ja esta quase esticada: nao ha alcance para empurrar).
    int hopPhase = 0, hops = 0, airborneTicks = 0, flights = 0, missedLandings = 0, failedTakeoffs = 0;
    double hopCrouchFrom = 0.0;
    // A posicao de cada sola em relacao a pelve na decolagem (o voo a conserva, com um recolhimento).
    Eigen::Vector3d hopFootOffset[2];
    double flightSum = 0.0, hopPeakAsked = 0.0, hopPeakPlanned = 0.0, hopPeakRatioSum = 0.0;
    int copClampTicks = 0;
    double stepTime = 0.0, stepDuration = 0.0;
    Eigen::Vector3d swingStart, swingLanding;
    Eigen::Matrix3d swingRotation;
    Eigen::Vector3d soleCenterLocal[2];
    StepRecoverySettings3D stepSettings;
    // O passo voluntario (G3, degrau 1): transferencia, balanco e pouso.
    enum class Phase { Standing, Transfer, Swing } phase = Phase::Standing;
    double phaseTime = 0.0, transferDuration = 1.2, transferMove = 0.6, unloadDuration = 0.15, swingDuration = 0.4;
    Eigen::Vector3d comFeedforwardVelocity = Eigen::Vector3d::Zero(), comFeedforwardAcceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d transferFrom = Eigen::Vector3d::Zero(), transferTo = Eigen::Vector3d::Zero();
    // O DCM de referencia (plano) e a derivada dele.
    Eigen::Vector2d dcmReference = Eigen::Vector2d::Zero(), dcmReferenceRate = Eigen::Vector2d::Zero();
    bool dcmInitialized = false;
    bool recovering = false;
    Eigen::Vector2d recoveryCop = Eigen::Vector2d::Zero();
    int landedFoot = -1;
    // Depois do pouso: o pe novo entra aos poucos e o alvo do CoM migra.
    double landedTime = -1.0;
    Eigen::Vector3d comFrom = Eigen::Vector3d::Zero(), comTo = Eigen::Vector3d::Zero();
    int voluntaryFoot = -1;
    Eigen::Vector2d voluntaryOffset = Eigen::Vector2d::Zero();
    double voluntaryStart = -1.0, clock = 0.0;
    // A marcha (G3): pegadas e DCM pelo DcmWalker3D, um passo encadeado no pouso do outro.
    bool walking = false;
    Eigen::Vector2d walkVelocity = Eigen::Vector2d::Zero(), walkLeft = Eigen::Vector2d::UnitY();
    // A velocidade pedida; a do plano chega a ela em walkRampSteps passos (partida).
    Eigen::Vector2d walkTarget = Eigen::Vector2d::Zero();
    int walkRampSteps = 0;
    DcmWalkerSettings3D walkSettings;
    DcmWalkerPlan3D walkPlan;
    double walkDoubleSupport = 0.1;
    // O relogio do plano: comeca no pouso (o apoio duplo conta dentro do T do passo).
    double walkPlanTime = 0.0;
    // Diagnostico: a forca horizontal pedida, a planejada pelo QP e a medida (m c'').
    Eigen::Vector2d lastCommandedForce = Eigen::Vector2d::Zero(), lastPlannedForce = Eigen::Vector2d::Zero();
    Eigen::Vector3d lastComVelocity = Eigen::Vector3d::Zero();
};

// A sola plana no chao, com o rumo atual do pe.
Eigen::Matrix3d flatSoleRotation(const Stand& s, const ArticulationKinematics3D& k, int foot) {
    const std::vector<Eigen::Vector3d>& c = s.corners[foot];
    Eigen::Vector3d soleNormal = (c[1] - c[0]).cross(c[2] - c[0]).normalized();
    if (soleNormal.dot(s.soleCenterLocal[foot]) < 0.0) soleNormal = -soleNormal;
    const Eigen::Matrix3d current = k.rotation[s.feet[foot]];
    return Eigen::Quaterniond::FromTwoVectors(current * soleNormal, Eigen::Vector3d(0.0, 0.0, -1.0)).toRotationMatrix() * current;
}

void setup(Stand& s, const RagdollProfile3D& profile) {
    PhysicsSceneSettings3D settings;
    const bool free = std::getenv("STANDFREE") != nullptr;
    settings.gravity = free ? Vec3 {} : Vec3 { 0.0f, 0.0f, -9.81f };
    settings.workerThreadCount = 1;
    settings.collisionSteps = std::getenv("STANDSTEPS") ? static_cast<std::uint32_t>(std::atoi(std::getenv("STANDSTEPS"))) : 4;
    s.scene = s.engine.createScene(settings, s.materials);
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { 500.0f, 500.0f, 0.25f }; // a marcha de 60 s passava da borda de 40 m (nota 29)
    shape.materialId = "grass";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = { 0.0f, 0.0f, -0.25f };
    body.materialId = "grass";
    (void)s.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.pelvisPosition = { 0.0f, 0.0f, profile.standingRootHeightMeters + 0.005f };
    spawn.active = true;
    if (const char* iterations = std::getenv("STANDITER")) {
        spawn.solverVelocityIterations = static_cast<std::uint32_t>(std::atoi(iterations));
        spawn.solverPositionIterations = static_cast<std::uint32_t>(std::atoi(iterations));
    }
    s.ragdoll = s.scene->createRagdoll(profile, spawn);
    s.model = buildArticulationModel3D(profile, s.scene->ragdollEffectiveMass(s.ragdoll));
    if (free) s.model.gravity.setZero();
    if (const char* friction = std::getenv("STANDFRIC")) s.scene->setRagdollFrictionScale(s.ragdoll, static_cast<float>(std::atof(friction)));
    // A referencia: o primeiro quadro do idle alerta (a postura atletica aprovada),
    // nas coordenadas do modelo (maos e pes: swing-twist).
    const std::string referenceClip = std::getenv("STANDREF") ? std::getenv("STANDREF") : "alert_idle";
    const AnimationClip3D idle = loadAnimationClip3D(std::string(MATTERENGINE_ASSETS_DIR) + "/animations/clips/" + referenceClip + ".matteranim.json");
    s.referencePose.assign(s.model.links.size(), Eigen::Vector3d::Zero());
    Eigen::Quaterniond rootOrientation = Eigen::Quaterniond::Identity();
    for (const AnimationTrack3D& track : idle.tracks) {
        const std::size_t link = findLink(profile, track.targetLinkId);
        if (track.space != AnimationTrackSpace3D::Joint) {
            const Quaternion q = sampleAnimationTrackSmooth3D(track, 0.0f, idle.durationSeconds, idle.loops).rotationDelta;
            rootOrientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
            continue;
        }
        if (link == 0 || link >= profile.links.size()) continue;
        const Vec3 v = sampleAnimationTrackSmooth3D(track, 0.0f, idle.durationSeconds, idle.loops).jointPositionRadians;
        s.referencePose[link] = articulationJointCoordinates3D(s.model.links[link], exponentialRotation3D(Eigen::Vector3d(v.x, v.y, v.z)));
    }
    // As solas niveladas: o idle tem a sola inclinada alguns mm, e o pe pousava
    // numa quina (o Jolt o girava ate assentar enquanto o QP o tratava rigido).
    if (!std::getenv("STANDNOLEVEL")) {
        for (int pass = 0; pass < 3; ++pass) {
            ArticulationState3D pose;
            pose.rootOrientation = rootOrientation;
            pose.jointCoordinates = s.referencePose;
            pose.velocity = Eigen::VectorXd::Zero(s.model.dofCount);
            const ArticulationKinematics3D k0 = computeArticulationKinematics3D(s.model, pose);
            for (const char* footName : { "LeftFoot", "RightFoot" }) {
                const std::size_t foot = findLink(profile, footName);
                const std::vector<Eigen::Vector3d> corners = soleCorners(profile, foot);
                // A normal da sola (para baixo) pelo plano dos cantos.
                Eigen::Vector3d normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]).normalized();
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (const Eigen::Vector3d& c : corners) center += c / 4.0;
                if (normal.dot(center) < 0.0) normal = -normal;
                const Eigen::Vector3d down = k0.rotation[foot] * normal;
                const Eigen::Matrix3d fix = Eigen::Quaterniond::FromTwoVectors(down, Eigen::Vector3d(0.0, 0.0, -1.0)).toRotationMatrix();
                const Eigen::Matrix3d footWorld = fix * k0.rotation[foot];
                const ArticulationLink3D& link = s.model.links[foot];
                const Eigen::Matrix3d relative = link.jointInParent.transpose() * k0.rotation[static_cast<std::size_t>(link.parent)].transpose()
                    * footWorld * link.jointInChild;
                s.referencePose[foot] = articulationJointCoordinates3D(link, relative);
            }
        }
    }
    // O corpo nasce na referencia, com a sola mais baixa 2 mm acima do chao.
    {
        ArticulationState3D pose;
        pose.rootOrientation = rootOrientation;
        pose.jointCoordinates = s.referencePose;
        pose.velocity = Eigen::VectorXd::Zero(s.model.dofCount);
        const ArticulationKinematics3D k0 = computeArticulationKinematics3D(s.model, pose);
        double lowest = 1e9;
        for (const char* footName : { "LeftFoot", "RightFoot" }) {
            const std::size_t foot = findLink(profile, footName);
            for (const Eigen::Vector3d& corner : soleCorners(profile, foot))
                lowest = std::min(lowest, (k0.origin[foot] + k0.rotation[foot] * corner).z());
        }
        pose.rootPosition.z() = 0.0005 - lowest + (free ? 1.0 : 0.0);
        double feetLowest[2] = { 1e9, 1e9 };
        for (int f = 0; f < 2; ++f) {
            const std::size_t foot = findLink(profile, f ? "RightFoot" : "LeftFoot");
            for (const Eigen::Vector3d& corner : soleCorners(profile, foot))
                feetLowest[f] = std::min(feetLowest[f], (k0.origin[foot] + k0.rotation[foot] * corner).z());
        }
        std::printf("referencia: %s; a sola esquerda %.1f mm e a direita %.1f mm acima da mais baixa\n", referenceClip.c_str(),
            1000.0 * (feetLowest[0] - lowest), 1000.0 * (feetLowest[1] - lowest));
        const ArticulationKinematics3D k = computeArticulationKinematics3D(s.model, pose);
        std::vector<PhysicsBodyState3D> links(s.model.links.size());
        for (std::size_t i = 0; i < links.size(); ++i) {
            const Eigen::Quaterniond q(k.rotation[i]);
            links[i].position = { float(k.origin[i].x()), float(k.origin[i].y()), float(k.origin[i].z()) };
            links[i].orientation = Quaternion(float(q.x()), float(q.y()), float(q.z()), float(q.w()));
        }
        s.scene->resetRagdollState(s.ragdoll, links);
    }
    if (free) {
        // Um degrau na referencia para o QP gerar movimento (o ensaio da previsao).
        s.referencePose[findLink(profile, "LeftThigh")][1] += 0.4;
        s.referencePose[findLink(profile, "Chest")][1] += 0.3;
        s.referencePose[findLink(profile, "RightUpperArm")][2] += 0.5;
    }
    s.feet[0] = findLink(profile, "LeftFoot");
    s.feet[1] = findLink(profile, "RightFoot");
    s.chest = findLink(profile, "UpperChest");
    s.head = findLink(profile, "Head");
    for (int f = 0; f < 2; ++f) {
        s.corners[f] = soleCorners(profile, s.feet[f]);
        s.soleCenterLocal[f].setZero();
        for (const Eigen::Vector3d& corner : s.corners[f]) s.soleCenterLocal[f] += corner / 4.0;
    }
    s.steppingEnabled = std::getenv("STANDSTEP") != nullptr;
    if (const char* voluntary = std::getenv("STANDVSTEP")) {
        int foot = 1;
        double dx = 0.2, dy = 0.0, when = 1.0;
        std::sscanf(voluntary, "%d,%lf,%lf,%lf", &foot, &dx, &dy, &when);
        s.voluntaryFoot = foot;
        s.voluntaryOffset = { dx, dy };
        s.voluntaryStart = when;
    }
    const ArticulationState3D state = articulationStateFromRagdoll3D(s.model, *s.scene->ragdollStateView(s.ragdoll));
    const ArticulationKinematics3D k = computeArticulationKinematics3D(s.model, state);
    const ArticulationDynamics3D d = computeArticulationDynamics3D(s.model, k);
    // O centro de massa sobre o meio dos pes, na altura de agora.
    // A pose de repouso ja e de pe: o alvo e o centro de massa dela.
    s.comTarget = d.centerOfMass;
    s.stepSettings.comHeight = d.centerOfMass.z();
    if (const char* margin = std::getenv("STANDTRIGGER")) s.stepSettings.triggerMargin = std::atof(margin);
    if (const char* overshoot = std::getenv("STANDOVER")) s.stepSettings.overshoot = std::atof(overshoot);
    s.pelvisTarget = k.rotation[0];
    if (const char* hop = std::getenv("STANDHOP")) {
        double flight = 0.1, stance = 0.25, when = 1.0;
        std::sscanf(hop, "%lf,%lf,%lf", &flight, &stance, &when);
        s.hopping = true;
        s.hopSettings.flightTime = flight;
        s.hopSettings.stanceTime = stance;
        s.hopSettings.mass = s.model.totalMass;
        s.hopStart = when;
    }
    if (const char* walk = std::getenv("STANDWALK")) {
        double vx = 1.0, vy = 0.0, duration = 0.5, when = 1.0;
        std::sscanf(walk, "%lf,%lf,%lf,%lf", &vx, &vy, &duration, &when);
        const Eigen::Vector2d forward = s.pelvisTarget.col(0).head<2>().normalized();
        s.walkLeft = s.pelvisTarget.col(1).head<2>().normalized();
        s.walkTarget = vx * forward + vy * s.walkLeft;
        s.walkRampSteps = std::getenv("STANDWALKRAMP") ? std::atoi(std::getenv("STANDWALKRAMP")) : 0;
        s.walkVelocity = s.walkRampSteps > 0 ? Eigen::Vector2d(s.walkTarget / double(s.walkRampSteps)) : s.walkTarget;
        s.walking = true;
        s.walkSettings.stepDuration = duration;
        if (const char* ds = std::getenv("STANDWALKDS")) s.walkDoubleSupport = std::atof(ds);
        if (const char* travel = std::getenv("STANDCOPTRAVEL")) s.walkSettings.copTravel = std::atof(travel);
        s.walkSettings.comHeight = s.stepSettings.comHeight;
        s.swingDuration = duration;
        // Para a esquerda, sai o pe esquerdo (0); senao o direito (1).
        s.voluntaryFoot = vy > 0.0 ? 0 : 1;
        s.voluntaryStart = when;
    }
    s.chestTarget = k.rotation[s.chest];
    s.headTarget = k.rotation[s.head];
    s.torqueLimit = Eigen::VectorXd::Zero(s.model.dofCount - 6);
    for (std::size_t i = 1; i < s.model.links.size(); ++i)
        for (int a = 0; a < 3; ++a) {
            const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
            if (dof >= 0) s.torqueLimit[dof - 6] = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].maximumTorque;
        }
}

// O quadro no formato do MatterModelInspector quadros (os colisores dos links e os contatos).
void dumpFrame(FILE* file, const RagdollProfile3D& profile, const RagdollState3D& state, double t) {
    std::fprintf(file, "F %.4f 0 0\n", t);
    for (std::size_t i = 0; i < profile.links.size() && i < state.links.size(); ++i) {
        const auto& collider = profile.links[i].collider;
        const Vec3 center = state.links[i].position + state.links[i].orientation.rotate(collider.localPosition);
        const Quaternion q = (state.links[i].orientation * collider.localOrientation).normalized();
        if (ragdollColliderHasBox3D(collider)) {
            std::fprintf(file, "B %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(), center.x, center.y,
                center.z, q.x, q.y, q.z, q.w, collider.boxHalfExtents.x, collider.boxHalfExtents.y, collider.boxHalfExtents.z);
        } else {
            std::fprintf(file, "C %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(), center.x, center.y,
                center.z, q.x, q.y, q.z, q.w, ragdollCapsuleHalfSegment3D(collider), collider.radiusMeters, ragdollCapsuleRadiusAtPositiveX3D(collider));
        }
    }
    for (const RagdollContactPoint3D& c : state.contacts)
        std::fprintf(file, "P %.4f %.4f %.4f %s %.3f %.3f %.3f\n", c.position.x, c.position.y, c.position.z,
            profile.links[c.linkIndex].id.c_str(), c.normal.x, c.normal.y, c.normal.z);
}

const char* statusName(WholeBodyQPStatus3D status) {
    switch (status) {
        case WholeBodyQPStatus3D::Solved: return "resolvido";
        case WholeBodyQPStatus3D::MaximumIterations: return "teto de iteracoes";
        case WholeBodyQPStatus3D::Infeasible: return "inviavel";
        case WholeBodyQPStatus3D::NotFinite: return "nao finito";
        case WholeBodyQPStatus3D::InvalidInput: return "entrada invalida";
    }
    return "?";
}


struct StepReport {
    bool fallen = false;
    WholeBodyQPStatus3D status = WholeBodyQPStatus3D::Solved;
    WholeBodyQPPath3D path = WholeBodyQPPath3D::Single;
    double solveMicros = 0.0, slip = 0.0, pelvisSpeed = 0.0;
    int iterations = 0;
};

StepReport step(Stand& s, const RagdollProfile3D& profile, const Eigen::Vector3d& push, std::size_t pushLink) {
    StepReport report;
    const RagdollState3D& raw = *s.scene->ragdollStateView(s.ragdoll);
    ArticulationState3D state = articulationStateFromRagdoll3D(s.model, raw);
    if (std::getenv("STANDFDVEL") && s.positionHistory) {
        // A velocidade pelas posicoes (que o backend corrige), nao pela publicada.
        const ArticulationState3D& before = *s.positionHistory;
        Eigen::VectorXd velocity = Eigen::VectorXd::Zero(s.model.dofCount);
        velocity.head<3>() = (state.rootPosition - before.rootPosition) / Dt;
        const Eigen::AngleAxisd spin(state.rootOrientation * before.rootOrientation.conjugate());
        velocity.segment<3>(3) = spin.axis() * spin.angle() / Dt;
        for (std::size_t i = 1; i < s.model.links.size(); ++i)
            for (int a = 0; a < 3; ++a) {
                const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof >= 0) velocity[dof] = (state.jointCoordinates[i][a] - before.jointCoordinates[i][a]) / Dt;
            }
        s.positionHistory = state;
        state.velocity = velocity;
    } else {
        s.positionHistory = state;
    }
    const ArticulationKinematics3D k = computeArticulationKinematics3D(s.model, state);
    const ArticulationDynamics3D d = computeArticulationDynamics3D(s.model, k);
    report.pelvisSpeed = state.velocity.head<3>().norm();
    if (std::getenv("STANDLIMITS") && s.logTick % 20 == 0 && s.logTick < 200) {
        for (std::size_t i = 1; i < raw.joints.size(); ++i)
            for (int a = 0; a < 3; ++a) {
                const RagdollAxisDefinition3D& axis = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)];
                if (!axis.enabled) continue;
                const float c = raw.joints[i].constraintTorqueNewtonMeters[static_cast<std::size_t>(a)];
                const float q = raw.joints[i].positionRadians[static_cast<std::size_t>(a)];
                if (std::abs(c) > 2.0f || q < axis.minimumRadians + 0.03f || q > axis.maximumRadians - 0.03f)
                    std::printf("   t %.2f %s eixo %d: q %.3f (limites %.3f %.3f) restricao %.1f N.m\n", s.logTick * Dt, profile.links[i].id.c_str(), a, q,
                        axis.minimumRadians, axis.maximumRadians, c);
            }
    }
    if (std::getenv("STANDDELIV") && s.previousState && s.logTick < 8) {
        const ActuationDelivery3D delivery = actuationDelivery3D(profile, raw);
        const Eigen::VectorXd delivered = articulationGeneralizedFromFeedforward3D(s.model, *s.previousState, delivery.muscleTorque);
        double worst = 0.0;
        for (int j = 6; j < s.model.dofCount; ++j) worst = std::max(worst, std::abs(s.previousTorque[j] - delivered[j]));
        std::printf("   tick %d: maior diferenca pedido x entregue %.4f N.m\n", s.logTick, worst);
        for (std::size_t i = 1; i < s.model.links.size(); ++i)
            for (int a = 0; a < 3; ++a) {
                const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof < 0 || std::abs(s.previousTorque[dof] - delivered[dof]) < 0.5) continue;
                std::printf("   tick %d %s eixo %d: pedido %.2f entregue %.2f (motor %.2f ff %.2f, restricao %.2f)\n", s.logTick, profile.links[i].id.c_str(), a,
                    s.previousTorque[dof], delivered[dof], raw.joints[i].motorTorqueNewtonMeters[static_cast<std::size_t>(a)],
                    raw.joints[i].feedforwardTorqueNewtonMeters[static_cast<std::size_t>(a)], raw.joints[i].constraintTorqueNewtonMeters[static_cast<std::size_t>(a)]);
            }
    }
    if (std::getenv("STANDFEET") && s.logTick < 40) {
        for (int f = 0; f < 2; ++f) {
            const PhysicsBodyState3D& foot = raw.links[s.feet[f]];
            std::printf("   tick %d pe %d: v (%.3f %.3f %.3f) w (%.2f %.2f %.2f) z %.4f\n", s.logTick, f, foot.linearVelocity.x, foot.linearVelocity.y,
                foot.linearVelocity.z, foot.angularVelocity.x, foot.angularVelocity.y, foot.angularVelocity.z, foot.position.z);
        }
    }
    if (std::getenv("STANDPRED") && s.predictedAcceleration.size() && (s.logTick % 15 == 0 || s.logTick < 4)) {
        const Eigen::VectorXd measured = (state.velocity - s.previousVelocity) / Dt;
        std::printf("   pelve: alfa previsto (%.2f %.2f %.2f) medido (%.2f %.2f %.2f); a previsto (%.2f %.2f %.2f) medido (%.2f %.2f %.2f)\n",
            s.predictedAcceleration[3], s.predictedAcceleration[4], s.predictedAcceleration[5], measured[3], measured[4], measured[5],
            s.predictedAcceleration[0], s.predictedAcceleration[1], s.predictedAcceleration[2], measured[0], measured[1], measured[2]);
    }
    if (std::getenv("STANDPRED") && s.previousState && (s.logTick % 15 == 0 || s.logTick < 4)) {
        const ArticulationKinematics3D pk = computeArticulationKinematics3D(s.model, *s.previousState);
        const ArticulationDynamics3D pd = computeArticulationDynamics3D(s.model, pk);
        std::vector<ContactSupport3D> supports;
        for (int f = 0; f < 2; ++f) {
            Eigen::Vector3d sole = Eigen::Vector3d::Zero();
            for (const Eigen::Vector3d& corner : s.corners[f]) sole += corner / 4.0;
            supports.push_back({ static_cast<int>(s.feet[f]), sole, true });
        }
        const ContactWrenchEstimate3D estimate = estimateContactWrench3D(s.model, *s.previousState, pk, pd, state, s.previousTorque, supports, Dt);
        for (int f = 0; f < 2; ++f) {
            const Eigen::VectorXd footAcc = (state.velocity - s.previousVelocity) / Dt;
            const Eigen::Vector3d footLinear = pk.originJacobian[s.feet[f]] * footAcc + pk.originBias[s.feet[f]];
            const auto& real = estimate.supports[static_cast<std::size_t>(f)];
            if (real.hasCenterOfPressure)
                std::printf("   pe %d: CoP real (%.3f %.3f) planejado (%.3f %.3f)\n", f, real.centerOfPressure.x(), real.centerOfPressure.y(),
                    s.plannedCop[f].x(), s.plannedCop[f].y());
            std::printf("   pe %d: forca real (%.0f %.0f %.0f) planejada (%.0f %.0f %.0f); aceleracao do pe %.2f m/s2\n", f,
                estimate.supports[static_cast<std::size_t>(f)].force.x(), estimate.supports[static_cast<std::size_t>(f)].force.y(), estimate.supports[static_cast<std::size_t>(f)].force.z(),
                s.plannedFootForce[f].x(), s.plannedFootForce[f].y(), s.plannedFootForce[f].z(), footLinear.norm());
        }
    }
    s.previousVelocity = state.velocity;
    s.previousState = state;
    if (raw.links[0].position.z < 0.55f) {
        report.fallen = true;
        return report;
    }
    WholeBodyQPInput3D input;
    input.model = &s.model;
    input.kinematics = &k;
    input.dynamics = &d;
    input.state = &state;
    input.torqueLimit = s.torqueLimit;
    // A caixa do motor (nota 33): o backend recorta cada eixo no referencial dele (STANDNOMOTORBOX desliga).
    const Eigen::MatrixXd motorMap = articulationMotorTorqueMap3D(s.model, state);
    if (!std::getenv("STANDNOMOTORBOX")) input.motorTorqueMap = motorMap;
    if (!std::getenv("STANDNOGUARD")) {
        // A guarda dos batentes nas juntas de um eixo (joelhos e cotovelos).
        const double inf = std::numeric_limits<double>::infinity();
        input.jointLower = Eigen::VectorXd::Constant(s.model.dofCount - 6, -inf);
        input.jointUpper = Eigen::VectorXd::Constant(s.model.dofCount - 6, inf);
        input.jointMargin = Eigen::VectorXd::Constant(s.model.dofCount - 6, 0.015);
        const bool allJoints = !std::getenv("STANDGUARD1");
        for (std::size_t i = 1; i < s.model.links.size(); ++i) {
            int enabled = 0;
            for (int a = 0; a < 3; ++a) enabled += s.model.links[i].dof[static_cast<std::size_t>(a)] >= 0;
            if (enabled != 1 && !allJoints) continue;
            for (int a = 0; a < 3; ++a) {
                const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof < 0) continue;
                input.jointLower[dof - 6] = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].minimumRadians;
                input.jointUpper[dof - 6] = profile.links[i].inboundJoint.axes[static_cast<std::size_t>(a)].maximumRadians;
                // Juntas de varios eixos: a coordenada so aproxima o criterio do Jolt (pyramid swing-twist).
                if (enabled > 1) input.jointMargin[dof - 6] = 0.05;
            }
        }
    }
    const double friction = 0.75 * 0.8; // gramado x sola, com margem de 20%
    const Eigen::Vector3d comNow = d.centerOfMass;
    const Eigen::Vector3d comVelocityNow = d.centroidalMatrix.topRows(3) * state.velocity / s.model.totalMass;
    Eigen::Vector3d sole[2];
    bool footTouching[2] = { false, false };
    for (int f = 0; f < 2; ++f) {
        sole[f] = k.origin[s.feet[f]] + k.rotation[s.feet[f]] * s.soleCenterLocal[f];
        for (const RagdollContactPoint3D& contact : raw.contacts) footTouching[f] |= contact.linkIndex == s.feet[f];
    }
    if (s.hopping) {
        const double comZ = d.centerOfMass.z(), comVz = comVelocityNow.z();
        // O toque pela geometria: o backend publica contato especulativo ~10 ticks depois de separar.
        bool touching = false;
        for (int f = 0; f < 2; ++f)
            for (const Eigen::Vector3d& corner : s.corners[f])
                touching |= footTouching[f] && (k.origin[s.feet[f]] + k.rotation[s.feet[f]] * corner).z() < 0.004;
        const double crouchTime = 0.6;
        if (s.hopPhase == 0 && s.clock >= s.hopStart) {
            s.hopPhase = 3;
            s.hopTime = 0.0;
            s.hopCrouchFrom = s.comTarget.z();
        } else if (s.hopPhase == 3) {
            // Agacha v0 T / 2 + 2 cm (o que o apoio sobe partindo do repouso) por uma quintica.
            s.hopTime += Dt;
            const double depth = 0.5 * hopTakeoffSpeed3D(s.hopSettings) * s.hopSettings.stanceTime + (std::getenv("STANDHOPMARGIN") ? std::atof(std::getenv("STANDHOPMARGIN")) : 0.02);
            const double u = std::clamp(s.hopTime / crouchTime, 0.0, 1.0);
            s.comTarget.z() = s.hopCrouchFrom - depth * u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
            if (s.hopTime >= crouchTime + 0.2) s.hopPhase = 4;
        } else if (s.hopPhase == 4) {
            s.hopPhase = 1;
            s.hopTime = 0.0;
            s.hopEntrySpeed = comVz;
            s.hopEntryHeight = comZ;
            s.hopPeakAsked = hopPeakForce3D(s.hopSettings, comVz);
            s.hopPeakPlanned = 0.0;
        } else if (s.hopPhase == 1) {
            s.hopTime += Dt;
            s.hopPeakPlanned = std::max(s.hopPeakPlanned, s.plannedFootForce[0].z() + s.plannedFootForce[1].z());
            if (s.hopTime >= s.hopSettings.stanceTime) {
                s.hopPhase = 2;
                s.hopTime = 0.0;
                s.airborneTicks = 0;
                for (int f = 0; f < 2; ++f) s.hopFootOffset[f] = sole[f] - k.origin[0];
                ++s.hops;
                if (s.hopPeakAsked > 1.0) s.hopPeakRatioSum += s.hopPeakPlanned / s.hopPeakAsked;
            }
        } else if (s.hopPhase == 2) {
            s.hopTime += Dt;
            if (!touching) ++s.airborneTicks;
            // Sem separacao observada em 3 ticks nao houve decolagem: volta ao apoio (com a velocidade medida).
            const bool noTakeoff = s.airborneTicks == 0 && s.hopTime >= 3.0 * Dt;
            if (noTakeoff) ++s.failedTakeoffs;
            const bool landed = touching && s.airborneTicks > 0 && s.hopTime > 0.5 * s.hopSettings.flightTime;
            if (noTakeoff || landed || s.hopTime > s.hopSettings.flightTime + 0.3) {
                if (noTakeoff) {
                } else if (landed) {
                    s.flightSum += s.airborneTicks * Dt;
                    ++s.flights;
                } else {
                    ++s.missedLandings;
                }
                s.hopPhase = 1;
                s.hopTime = 0.0;
                s.hopEntrySpeed = comVz;
                s.hopEntryHeight = comZ;
                s.hopPeakAsked = hopPeakForce3D(s.hopSettings, comVz);
                s.hopPeakPlanned = 0.0;
            }
        }
    }
    if (s.hopping && std::getenv("STANDHOPDBG") && s.clock >= s.hopStart + 0.75 && s.clock < s.hopStart + 2.5)
        std::printf("   salto t %.3f fase %d: CoM z %.3f vz %.2f | toca %d %d | Fz planejada %.0f | pelve z %.3f\n", s.clock, s.hopPhase,
            d.centerOfMass.z(), comVelocityNow.z(), int(footTouching[0]), int(footTouching[1]),
            s.plannedFootForce[0].z() + s.plannedFootForce[1].z(), k.origin[0].z());
    if (s.separationPending && s.stepping) {
        if (footTouching[s.swingFoot]) {
            ++s.separationTicks;
        } else {
            s.separationSum += s.separationTicks;
            ++s.separations;
            s.separationPending = false;
        }
    }
    if (s.stepping && false) {
        s.stepTime += Dt;
        const bool landed = s.stepTime > 0.6 * s.stepDuration && footTouching[s.swingFoot];
        if (landed || s.stepTime > s.stepDuration + 0.2) {
            s.stepping = false;
            // O apoio novo: o centro de massa vai para o meio dos pes.
            const Eigen::Vector3d middle = 0.5 * (sole[0] + sole[1]);
            s.comTarget = { middle.x(), middle.y(), s.comTarget.z() };
        }
    }
    if (s.steppingEnabled && s.phase == Stand::Phase::Standing && (s.landedTime < 0.0 || s.landedTime > (std::getenv("STANDWAIT") ? std::atof(std::getenv("STANDWAIT")) : 0.05)) && footTouching[0] && footTouching[1]) {
        std::vector<Eigen::Vector2d> support;
        for (const RagdollContactPoint3D& contact : raw.contacts) support.emplace_back(contact.position.x, contact.position.y);
        const Eigen::Vector3d left = k.rotation[0] * Eigen::Vector3d::UnitY();
        std::array<StepRecoveryFoot3D, 2> feetInfo;
        for (int f = 0; f < 2; ++f) {
            feetInfo[static_cast<std::size_t>(f)].center = sole[f].head<2>();
            const Eigen::Vector2d side = Eigen::Vector2d(left.x(), left.y()).normalized();
            feetInfo[static_cast<std::size_t>(f)].outward = f == 0 ? side : Eigen::Vector2d(-side);
            for (const RagdollContactPoint3D& contact : raw.contacts)
                if (contact.linkIndex == s.feet[f]) feetInfo[static_cast<std::size_t>(f)].support.emplace_back(contact.position.x, contact.position.y);
        }
        const StepRecoveryPlan3D plan = planRecoveryStep3D(s.stepSettings, comNow.head<2>(), comVelocityNow.head<2>(), support, feetInfo);
        if (plan.step) {
            s.recoveryCop = plan.stanceCop;
            // O passo de recuperacao usa a mesma maquina do voluntario, direto no balanco.
            s.landedTime = -1.0;
            s.landedFoot = -1;
            ++s.steps;
            s.recovering = true;
            s.voluntaryFoot = plan.swingFoot;
            s.phase = Stand::Phase::Swing;
            s.phaseTime = 0.0;
            s.stepping = true;
            s.swingFoot = plan.swingFoot;
            s.stepTime = 0.0;
            s.stepDuration = plan.duration;
            s.swingDuration = plan.duration;
            s.swingStart = sole[plan.swingFoot];
            // O pouso mira 1 cm abaixo do chao (garante o contato) e o CoM pode descer nos passos grandes.
            s.swingLanding = { plan.landing.x(), plan.landing.y(), sole[plan.swingFoot].z() - 0.01 };
            const double width = (plan.landing - sole[1 - plan.swingFoot].head<2>()).norm();
            s.comTarget.z() = s.stepSettings.comHeight - 0.15 * std::max(0.0, width - 0.3);
            const std::vector<Eigen::Vector3d>& c = s.corners[plan.swingFoot];
            Eigen::Vector3d soleNormal = (c[1] - c[0]).cross(c[2] - c[0]).normalized();
            if (soleNormal.dot(s.soleCenterLocal[plan.swingFoot]) < 0.0) soleNormal = -soleNormal;
            const Eigen::Matrix3d current = k.rotation[s.feet[plan.swingFoot]];
            s.swingRotation = Eigen::Quaterniond::FromTwoVectors(current * soleNormal, Eigen::Vector3d(0.0, 0.0, -1.0)).toRotationMatrix() * current;
            if (std::getenv("STANDSTEPLOG"))
                std::printf("   passo de recuperacao %d: pe %d, DCM %.3f m fora, pouso (%.2f %.2f), %.2f s\n", s.steps, plan.swingFoot, plan.outside,
                    plan.landing.x(), plan.landing.y(), plan.duration);
        }
    }
    // O passo voluntario: Standing -> Transfer (descarrega o pe) -> Swing -> pouso.
    s.clock += Dt;
    s.comFeedforwardVelocity.setZero();
    s.comFeedforwardAcceleration.setZero();
    if (s.phase == Stand::Phase::Transfer) {
        // A transferencia: o CoM por uma quintica ate o pe de apoio (com alimentacao).
        const SwingSample3D path = sampleSwing3D(s.transferFrom, s.transferTo, 0.0, s.transferMove, s.phaseTime);
        if (std::getenv("STANDNODCM")) {
            s.comTarget = path.position;
            s.comFeedforwardVelocity = path.velocity;
            s.comFeedforwardAcceleration = path.acceleration;
        } else {
            s.dcmReference = path.position.head<2>();
            s.dcmReferenceRate = path.velocity.head<2>();
        }
    }
    if (s.walking) {
        s.walkPlanTime += Dt;
        if (s.phase == Stand::Phase::Transfer && s.steps > 0) {
            const DcmReference3D walk = sampleDcmWalk3D(s.walkSettings, s.walkPlan, s.walkPlanTime);
            s.dcmReference = walk.position;
            s.dcmReferenceRate = walk.rate;
        }
    }
    if (s.landedTime >= 0.0) {
        s.landedTime += Dt;
        const double blend = std::min(1.0, s.landedTime / 0.6);
        const double smooth = blend * blend * (3.0 - 2.0 * blend);
        s.comTarget = s.comFrom + smooth * (s.comTo - s.comFrom);
        if (blend >= 1.0) s.landedTime = -1.0;
    }
    if (s.voluntaryFoot >= 0) {
        const int swing = s.voluntaryFoot;
        const int stanceFoot = 1 - swing;
        if (s.phase == Stand::Phase::Standing && s.voluntaryStart >= 0.0 && s.clock >= s.voluntaryStart) {
            s.phase = Stand::Phase::Transfer;
            s.phaseTime = 0.0;
            s.voluntaryStart = -1.0;
            s.swingStart = sole[swing];
            s.swingLanding = { sole[swing].x() + s.voluntaryOffset.x(), sole[swing].y() + s.voluntaryOffset.y(), sole[swing].z() };
            const Eigen::Vector3d sc = sole[stanceFoot];
            s.transferFrom = s.comTarget;
            s.transferTo = { sc.x(), sc.y(), s.comTarget.z() };
            if (s.walking) {
                // A partida: o DCM vai ao comeco do plano do primeiro passo.
                s.walkPlan = planDcmWalk3D(s.walkSettings, sc.head<2>(), stanceFoot == 0 ? +1 : -1, s.walkLeft, s.walkVelocity);
                s.swingLanding = { s.walkPlan.footsteps[1].x(), s.walkPlan.footsteps[1].y(), sc.z() - 0.01 };
                const Eigen::Vector2d start = sampleDcmWalk3D(s.walkSettings, s.walkPlan, 0.0).position;
                s.transferTo = { start.x(), start.y(), s.comTarget.z() };
            }
            const std::vector<Eigen::Vector3d>& c = s.corners[swing];
            Eigen::Vector3d soleNormal = (c[1] - c[0]).cross(c[2] - c[0]).normalized();
            if (soleNormal.dot(s.soleCenterLocal[swing]) < 0.0) soleNormal = -soleNormal;
            const Eigen::Matrix3d current = k.rotation[s.feet[swing]];
            s.swingRotation = Eigen::Quaterniond::FromTwoVectors(current * soleNormal, Eigen::Vector3d(0.0, 0.0, -1.0)).toRotationMatrix() * current;
        } else if (s.phase != Stand::Phase::Standing) {
            s.phaseTime += Dt;
            bool ready = false;
            if (s.phase == Stand::Phase::Transfer) {
                // So tira o pe quando o DCM esta dentro do pe de apoio (com folga) ou no prazo.
                std::vector<Eigen::Vector2d> stancePoints;
                for (const RagdollContactPoint3D& contact : raw.contacts)
                    if (contact.linkIndex == s.feet[stanceFoot]) stancePoints.emplace_back(contact.position.x, contact.position.y);
                const Eigen::Vector2d capture = comNow.head<2>() + comVelocityNow.head<2>() / stepRecoveryOmega3D(s.stepSettings);
                ready = s.phaseTime >= s.transferMove + s.unloadDuration && stancePoints.size() >= 3
                    && polygonOutside3D(stancePoints, capture) < -0.02;
                if (s.walking) ready = s.steps > 0 ? s.phaseTime >= s.transferMove
                                                   : s.phaseTime >= s.transferMove && (capture - s.transferTo.head<2>()).norm() < 0.03;
            }
            if (s.phase == Stand::Phase::Transfer && (ready || s.phaseTime >= s.transferDuration)) {
                if (!ready && std::getenv("STANDSTEPLOG")) std::printf("   transferencia: prazo esgotado sem o DCM no pe de apoio\n");
                s.phase = Stand::Phase::Swing;
                s.phaseTime = 0.0;
                s.swingStart = sole[swing];
                s.stepping = true;
                if (s.walking && s.steps > 0) {
                    ++s.liftoffs;
                    s.liftoffForceSum += s.plannedFootForce[swing].z();
                    s.separationPending = true;
                    s.separationTicks = 0;
                }
                if (std::getenv("STANDTOEOFF") && s.walking && s.steps > 0) {
                    ++s.toeOffs;
                    s.toeOffAngleSum += s.toeOffLastAngle;
                }
                s.swingFoot = swing;
                s.stepTime = 0.0;
                s.stepDuration = s.swingDuration;
                if (s.walking) {
                    if (s.steps == 0) s.walkPlanTime = 0.0;
                    s.stepDuration = std::max(0.2, s.walkSettings.stepDuration - s.walkPlanTime);
                    s.swingDuration = s.stepDuration;
                }
            } else if (s.phase == Stand::Phase::Swing) {
                s.stepTime = s.phaseTime;
                double lowest = 1e9;
                for (const Eigen::Vector3d& corner : s.corners[swing])
                    lowest = std::min(lowest, (k.origin[s.feet[swing]] + k.rotation[s.feet[swing]] * corner).z());
                const bool landed = s.phaseTime > 0.6 * s.swingDuration && footTouching[swing] && lowest < 0.01;
                if (s.walking && landed) {
                    // O pe que pousou vira o apoio; um apoio duplo curto leva o DCM ao comeco do
                    // plano do proximo passo (a transferencia) e so entao o outro pe sai.
                    const int next = stanceFoot;
                    if (s.walkRampSteps > 0) s.walkVelocity = s.walkTarget * std::min(1.0, double(s.steps + 2) / double(s.walkRampSteps));
                    s.walkPlan = planDcmWalk3D(s.walkSettings, sole[swing].head<2>(), swing == 0 ? +1 : -1, s.walkLeft, s.walkVelocity);
                    ++s.steps;
                    if (std::getenv("STANDSTEPLOG"))
                        std::printf("   passo %d: pe %d pousou em (%.2f %.2f) aos %.2f s; sai o pe %d\n", s.steps, swing, sole[swing].x(), sole[swing].y(), s.phaseTime, next);
                    s.voluntaryFoot = next;
                    s.stepping = false;
                    s.phase = Stand::Phase::Transfer;
                    s.phaseTime = 0.0;
                    s.transferMove = s.walkDoubleSupport;
                    s.walkPlanTime = 0.0;
                    s.transferFrom = { s.dcmReference.x(), s.dcmReference.y(), s.comTarget.z() };
                    const Eigen::Vector2d start = sampleDcmWalk3D(s.walkSettings, s.walkPlan, 0.0).position;
                    s.transferTo = { start.x(), start.y(), s.comTarget.z() };
                    s.swingLanding = { s.walkPlan.footsteps[1].x(), s.walkPlan.footsteps[1].y(), sole[swing].z() - 0.01 };
                    s.swingRotation = flatSoleRotation(s, k, next);
                } else if (landed || s.phaseTime > s.swingDuration + 0.3) {
                    if (!landed && std::getenv("STANDSTEPLOG"))
                        std::printf("   pouso FALHOU (tempo esgotado): tocando %d, quina mais baixa %.3f m\n", int(footTouching[swing]), lowest);
                    s.walking = false;
                    s.phase = Stand::Phase::Standing;
                    s.stepping = false;
                    s.landedTime = 0.0;
                    if (s.recovering) { s.recovering = false; s.voluntaryFoot = -1; s.landedFoot = swing; }
                    s.comFrom = s.comTarget;
                    const Eigen::Vector3d middle = 0.5 * (sole[0] + sole[1]);
                    s.comTo = { middle.x(), middle.y(), s.recovering ? s.stepSettings.comHeight : s.comTarget.z() };
                }
            }
        }
    }
    // A saida pela ponta (STANDTOEOFF=graus): no apoio duplo da marcha a frente, o pe de tras
    // apoia so na linha da ponta (posto 5, sem minimo normal) e a arfagem segue uma quintica.
    const double toeOffDegrees = std::getenv("STANDTOEOFF") ? std::atof(std::getenv("STANDTOEOFF")) : 0.0;
    const Eigen::Vector2d walkForward = s.pelvisTarget.col(0).head<2>().normalized();
    const bool toeOff = toeOffDegrees > 0.0 && s.walking && s.phase == Stand::Phase::Transfer && s.steps > 0 && s.voluntaryFoot >= 0
        && s.walkVelocity.dot(walkForward) > 0.3;
    // A descarga na marcha (nota 31): no apoio duplo o pe que vai sair perde o minimo normal e o
    // teto cai a ~0 no fim, para o QP nao soltar um pe ainda carregado (antes: 144-244 N no tick
    // anterior). A aquisicao espelhada no pe que pousa foi pior (a esquerda caiu): ele carrega ja.
    const bool unloading = s.walking && s.phase == Stand::Phase::Transfer && s.steps > 0 && s.voluntaryFoot >= 0;
    const double unloadCap = s.model.totalMass * 9.81 * (1.0 - std::clamp(s.phaseTime / std::max(1e-3, s.transferMove), 0.0, 1.0));
    Eigen::Vector3d toeA = Eigen::Vector3d::Zero(), toeB = Eigen::Vector3d::Zero(), heelMiddle = Eigen::Vector3d::Zero();
    if (toeOff) {
        const int f = s.voluntaryFoot;
        std::vector<Eigen::Vector3d> corners;
        for (const Eigen::Vector3d& corner : s.corners[f]) corners.push_back(k.origin[s.feet[f]] + k.rotation[s.feet[f]] * corner);
        std::sort(corners.begin(), corners.end(), [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.head<2>().dot(walkForward) > b.head<2>().dot(walkForward); });
        toeA = corners[0];
        toeB = corners[1];
        heelMiddle = 0.5 * (corners[2] + corners[3]);
        for (int e = 0; e < 2; ++e) {
            WholeBodyContactPoint3D point;
            point.link = static_cast<int>(s.feet[f]);
            point.pointWorld = e == 0 ? toeA : toeB;
            point.normal = Eigen::Vector3d::UnitZ();
            point.friction = friction;
            point.constrained = true;
            if (e == 1) point.freeDirection = (toeB - toeA).normalized();
            if (unloading) point.maximumNormalForce = std::max(1.0, unloadCap);
            const Eigen::Vector3d velocity = articulationPointJacobian3D(k, point.link, point.pointWorld) * state.velocity;
            point.desiredAcceleration = -20.0 * velocity;
            report.slip = std::max(report.slip, std::hypot(velocity.x(), velocity.y()));
            input.contacts.push_back(point);
        }
    }
    for (int f = 0; f < 2; ++f) {
        if (s.stepping && f == s.swingFoot) continue;
        if (toeOff && f == s.voluntaryFoot) continue;
        // No voo do salto nenhum pe tem forca nem vinculo (balistico).
        if (s.hopping && s.hopPhase == 2) continue;
        bool touching = false;
        for (const RagdollContactPoint3D& contact : raw.contacts)
            if (contact.linkIndex == s.feet[f]) touching = true;
        // O apoio tambem pela geometria: a sola a menos de 4 mm do chao (o backend
        // so publica contatos depois do primeiro passo).
        if (!touching && std::getenv("STANDGEOM"))
            for (const Eigen::Vector3d& corner : s.corners[f])
                if ((k.origin[s.feet[f]] + k.rotation[s.feet[f]] * corner).z() < 0.004) touching = true;
        if (!touching) continue;
        // O pe apoiado e rigido: aceleracao 6D = - amortecimento da velocidade dele.
        WholeBodyContactLink3D rigid;
        rigid.link = static_cast<int>(s.feet[f]);
        const double footDamping = std::getenv("STANDFOOTD") ? std::atof(std::getenv("STANDFOOTD")) : 20.0;
        rigid.desiredAcceleration.head<3>() = -footDamping * k.angularVelocity[s.feet[f]];
        rigid.desiredAcceleration.tail<3>() = -footDamping * k.originVelocity[s.feet[f]];
        const bool pointContacts = std::getenv("STANDPOINTS") != nullptr;
        const bool justLanded = s.landedTime >= 0.0 && s.landedTime < 0.15 && (f == s.voluntaryFoot || f == s.landedFoot);
        if (!pointContacts && !justLanded) input.contactLinks.push_back(rigid);
        // Executar pelo apoio observado: os pontos de contato reais da bota (o
        // poligono do Jolt e menor que o retangulo da sola).
        std::vector<Eigen::Vector3d> observed, normals;
        bool anyObserved = false;
        for (const RagdollContactPoint3D& contact : raw.contacts) anyObserved |= contact.linkIndex == s.feet[f];
        if (!anyObserved)
            for (const Eigen::Vector3d& corner : s.corners[f]) {
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (const Eigen::Vector3d& c : s.corners[f]) center += c / 4.0;
                observed.push_back(k.origin[s.feet[f]] + k.rotation[s.feet[f]] * (center + 0.5 * (corner - center)));
                normals.push_back(Eigen::Vector3d::UnitZ());
            }
        for (const RagdollContactPoint3D& contact : raw.contacts) {
            if (contact.linkIndex != s.feet[f]) continue;
            observed.push_back(toEigen(contact.position));
            normals.push_back(toEigen(contact.normal));
        }
        if (!std::getenv("STANDRECT") && !observed.empty()) {
            // O apoio observado (o losango do Jolt) recuado para o centro: margem.
            const double shrink = std::getenv("STANDSHRINK") ? std::atof(std::getenv("STANDSHRINK")) : 0.7;
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const Eigen::Vector3d& point : observed) center += point / double(observed.size());
            for (Eigen::Vector3d& point : observed) point = center + shrink * (point - center);
        } else if (std::getenv("STANDRECT")) {
            // O poligono conservador: os cantos da sola recuados para o centro.
            const double shrink = std::getenv("STANDSHRINK") ? std::atof(std::getenv("STANDSHRINK")) : 0.6;
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const Eigen::Vector3d& corner : s.corners[f]) center += corner / 4.0;
            observed.clear();
            normals.clear();
            for (const Eigen::Vector3d& corner : s.corners[f]) {
                observed.push_back(k.origin[s.feet[f]] + k.rotation[s.feet[f]] * (center + shrink * (corner - center)));
                normals.push_back(Eigen::Vector3d::UnitZ());
            }
        }
        for (std::size_t c = 0; c < observed.size(); ++c) {
            WholeBodyContactPoint3D point;
            point.link = static_cast<int>(s.feet[f]);
            point.pointWorld = observed[c];
            point.normal = normals[c].normalized();
            point.friction = friction;
            point.constrained = pointContacts;
            point.minimumNormalForce = std::getenv("STANDMINN") ? std::atof(std::getenv("STANDMINN")) : 15.0;
            if (s.hopping && s.hopPhase == 1) point.minimumNormalForce = 0.0;
            if (s.landedTime >= 0.0 && s.landedTime < 0.15 && (f == s.voluntaryFoot || f == s.landedFoot)) point.minimumNormalForce = 0.0;
            if (unloading && f == s.voluntaryFoot) {
                point.minimumNormalForce = 0.0;
                point.maximumNormalForce = std::max(1.0, unloadCap);
            }
            if (s.phase == Stand::Phase::Transfer && f == s.voluntaryFoot && !s.walking) {
                // Descarrega o pe que vai subir so depois que o CoM chegou: sem minimo e
                // com teto decrescente no fim da transferencia.
                point.minimumNormalForce = 0.0;
                const double unload = std::clamp((s.phaseTime - s.transferMove) / s.unloadDuration, 0.0, 1.0);
                if (unload > 0.0) point.maximumNormalForce = std::max(1.0, 0.25 * s.model.totalMass * 9.81 * (1.0 - unload));
            }
            // Amortece qualquer escorregar do ponto.
            const Eigen::Vector3d velocity = articulationPointJacobian3D(k, point.link, point.pointWorld) * state.velocity;
            point.desiredAcceleration = -20.0 * velocity;
            report.slip = std::max(report.slip, std::hypot(velocity.x(), velocity.y()));
            input.contacts.push_back(point);
        }
    }
    const double m = s.model.totalMass;
    const double gain = std::getenv("STANDGAIN") ? std::atof(std::getenv("STANDGAIN")) : 1.0;
    const Eigen::Vector3d comVelocity = d.centroidalMatrix.topRows(3) * state.velocity / m;
    Eigen::Vector3d linearRate = m * (s.comFeedforwardAcceleration + 40.0 * gain * gain * (s.comTarget - d.centerOfMass)
        + 2.0 * std::sqrt(40.0) * gain * (s.comFeedforwardVelocity - comVelocity));
    if (!std::getenv("STANDNODCM") && (s.voluntaryFoot >= 0 || s.steppingEnabled)) {
        // O controle pelo DCM: xi' = omega (xi - p). Quer-se xi' = xi'_ref - K (xi - xi_ref);
        // o CoP desejado sai disso, preso ao apoio, e a aceleracao do CoM e omega^2 (c - p).
        const double omega = stepRecoveryOmega3D(s.stepSettings);
        const Eigen::Vector2d xi = d.centerOfMass.head<2>() + comVelocity.head<2>() / omega;
        if (!s.dcmInitialized) {
            s.dcmReference = s.comTarget.head<2>();
            s.dcmInitialized = true;
        }
        if (s.phase == Stand::Phase::Standing) {
            s.dcmReference = s.comTarget.head<2>();
            s.dcmReferenceRate.setZero();
        } else if (s.phase == Stand::Phase::Swing && s.walking) {
            const DcmReference3D walk = sampleDcmWalk3D(s.walkSettings, s.walkPlan, s.walkPlanTime);
            s.dcmReference = walk.position;
            s.dcmReferenceRate = walk.rate;
            // O pouso segue o DCM medido ate 70% do balanco (depois fica fixo).
            if (!std::getenv("STANDNOADJUST") && s.stepTime < 0.7 * s.stepDuration)
                s.swingLanding.head<2>() = adjustDcmLanding3D(s.walkSettings, s.walkPlan, DcmLandingLimits3D {}, xi,
                    s.voluntaryFoot == 1 ? +1 : -1, s.walkLeft, s.walkSettings.stepDuration - s.walkPlanTime);
        } else if (s.phase == Stand::Phase::Swing) {
            s.dcmReference = sole[1 - s.voluntaryFoot].head<2>();
            s.dcmReferenceRate.setZero();
        }
        if (s.recovering) {
            // No balanco da recuperacao o CoP fica no pe de apoio: o corpo cai para o pouso.
            s.dcmReference = xi;
            s.dcmReferenceRate = omega * (xi - s.recoveryCop);
        }
        const double gainXi = 3.0;
        Eigen::Vector2d cop = xi - (s.dcmReferenceRate - gainXi * (xi - s.dcmReference)) / omega;
        // Preso ao apoio (os contatos dos pes que apoiam), recuado 2 cm.
        std::vector<Eigen::Vector2d> support;
        if (std::getenv("STANDOLDPOLY")) {
            for (const RagdollContactPoint3D& contact : raw.contacts) {
                const bool swingFootContact = s.stepping && static_cast<int>(contact.linkIndex) == static_cast<int>(s.feet[s.swingFoot]);
                if (!swingFootContact) support.emplace_back(contact.position.x, contact.position.y);
            }
            if (!support.empty()) cop = projectToPolygon3D(support, cop, 0.02);
        } else {
            // O mesmo apoio que o QP recebe (os pontos ja recuados), com os minimos de forca normal:
            // o CoP possivel e sum(f_i/F) p_i + (1 - sum f_i/F) conv(p), com F o peso.
            const double weight = m * 9.81;
            double share = 0.0;
            Eigen::Vector2d fixed = Eigen::Vector2d::Zero();
            for (const WholeBodyContactPoint3D& point : input.contacts) {
                share += point.minimumNormalForce / weight;
                fixed += point.minimumNormalForce / weight * point.pointWorld.head<2>();
            }
            share = std::min(share, 0.9);
            for (const WholeBodyContactPoint3D& point : input.contacts) support.push_back(fixed + (1.0 - share) * point.pointWorld.head<2>());
            const Eigen::Vector2d wanted = cop;
            if (!support.empty()) cop = projectToPolygon3D(support, cop, 0.005);
            if (s.walking) {
                const double cut = (wanted - cop).norm();
                s.copClampSum += cut;
                s.copClampWorst = std::max(s.copClampWorst, cut);
                ++s.copClampTicks;
            }
        }
        if (std::getenv("STANDWALKDBG") && s.walking && s.logTick++ % 6 == 0)
            std::printf("   t %.2f fase %d pe %d: xi (%.3f %.3f) ref (%.3f %.3f) cop (%.3f %.3f) apoio (%.3f %.3f) pes L (%.3f %.3f) R (%.3f %.3f)\n", s.clock,
                int(s.phase), s.voluntaryFoot, xi.x(), xi.y(), s.dcmReference.x(), s.dcmReference.y(), cop.x(), cop.y(),
                s.voluntaryFoot >= 0 ? sole[1 - s.voluntaryFoot].x() : 0.0, s.voluntaryFoot >= 0 ? sole[1 - s.voluntaryFoot].y() : 0.0,
                sole[0].x(), sole[0].y(), sole[1].x(), sole[1].y());
        linearRate.x() = m * omega * omega * (d.centerOfMass.x() - cop.x());
        linearRate.y() = m * omega * omega * (d.centerOfMass.y() - cop.y());
    }
    if (s.stepping && s.voluntaryFoot < 0) {
        // No passo, o centro de massa segue o pendulo sobre o pe de apoio (nao briga).
        const double omega = stepRecoveryOmega3D(s.stepSettings);
        const Eigen::Vector3d stance = sole[1 - s.swingFoot];
        linearRate.x() = m * omega * omega * (d.centerOfMass.x() - stance.x());
        linearRate.y() = m * omega * omega * (d.centerOfMass.y() - stance.y());
    }
    if (s.hopping && s.hopPhase == 1) {
        // O apoio do salto: a vertical segue o meio seno, com realimentacao na trajetoria fechada.
        const HopSample3D hop = sampleHopStance3D(s.hopSettings, s.hopTime, s.hopEntrySpeed);
        linearRate.z() = m * (hop.acceleration + 100.0 * (s.hopEntryHeight + hop.displacement - d.centerOfMass.z())
            + 20.0 * (hop.velocity - comVelocity.z()));
    }
    const Eigen::Vector3d angularMomentum = d.centroidalMatrix.bottomRows(3) * state.velocity;
    const double angularWeight = std::getenv("STANDLW") ? std::atof(std::getenv("STANDLW")) : 0.1;
    const double angularGain = std::getenv("STANDLG") ? std::atof(std::getenv("STANDLG")) : 2.0;
    const bool hierarchy = std::getenv("STANDHQP") != nullptr;
    // No voo, nenhuma tarefa centroidal: o momento e balistico.
    if (!(s.hopping && s.hopPhase == 2)) {
        Eigen::Vector3d angularRate = -angularGain * angularMomentum;
        if (const char* pose = std::getenv("STANDANGPOSE")) {
            // O momento angular tambem endireita o tronco (Koolen): I kp theta, so rolagem e arfagem.
            const Eigen::AngleAxisd error(s.pelvisTarget * k.rotation[0].transpose());
            Eigen::Vector3d theta = error.axis() * error.angle();
            theta.z() = 0.0;
            angularRate += 10.0 * std::atof(pose) * theta;
        }
        WholeBodyTask3D centroidal = wholeBodyCentroidalTask3D(s.model, d, state, linearRate, angularRate,
            (std::getenv("STANDCW") ? std::atof(std::getenv("STANDCW")) : 1000.0), angularWeight * 1000.0);
        if (hierarchy) {
            // Dois niveis (nota 27): a forca linear do DCM em cima; o momento angular embaixo (livre para o quadril).
            WholeBodyTask3D linear = centroidal, angular = centroidal;
            linear.name = "centroidal linear";
            linear.matrix = centroidal.matrix.topRows(3);
            linear.target = centroidal.target.head(3);
            linear.priority = 0;
            angular.name = "centroidal angular";
            angular.matrix = centroidal.matrix.bottomRows(3);
            angular.target = centroidal.target.tail(3);
            if (std::getenv("STANDHQPANG")) angular.priority = 0;
            if (std::getenv("STANDHQPXY")) {
                // So a forca horizontal em cima; a vertical embaixo, com o angular.
                WholeBodyTask3D vertical = linear;
                vertical.name = "centroidal vertical";
                vertical.matrix = centroidal.matrix.row(2);
                vertical.target = centroidal.target.segment(2, 1);
                vertical.priority = s.hopping && s.hopPhase == 1 ? 0 : 1;
                linear.matrix = centroidal.matrix.topRows(2);
                linear.target = centroidal.target.head(2);
                input.tasks.push_back(vertical);
            }
            input.tasks.push_back(linear);
            input.tasks.push_back(angular);
        } else {
            input.tasks.push_back(centroidal);
        }
    }
    input.tasks.push_back(wholeBodyOrientationTask3D(k, 0, s.pelvisTarget, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 100.0 * gain * gain, 20.0 * gain, 5.0, "pelve"));
    input.tasks.push_back(wholeBodyOrientationTask3D(k, static_cast<int>(s.chest), s.chestTarget, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 100.0 * gain * gain, 20.0 * gain, 3.0, "peito"));
    input.tasks.push_back(wholeBodyOrientationTask3D(k, static_cast<int>(s.head), s.headTarget, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 100.0 * gain * gain, 20.0 * gain, 2.0, "cabeca"));
    if (s.stepping) {
        // O destino aceito fica fixo no balanco (nota 26: replanejar sem preservar derivadas derrubava).
        if (s.recovering && std::getenv("STANDREPLAN")) {
            const Eigen::Vector3d left = k.rotation[0] * Eigen::Vector3d::UnitY();
            Eigen::Vector2d outward = Eigen::Vector2d(left.x(), left.y()).normalized();
            if (s.swingFoot == 1) outward = -outward;
            const Eigen::Vector2d landing = replanLanding3D(s.stepSettings, comNow.head<2>(), comVelocityNow.head<2>(),
                sole[1 - s.swingFoot].head<2>(), outward, s.stepDuration - s.stepTime);
            if (s.stepTime < 0.8 * s.stepDuration) s.swingLanding.head<2>() = landing;
        }
        const SwingSample3D swing = sampleSwing3D(s.swingStart, s.swingLanding,
            s.recovering ? 0.025 : (std::getenv("STANDSWINGH") ? std::atof(std::getenv("STANDSWINGH")) : 0.06), s.stepDuration, s.stepTime);
        const int foot = static_cast<int>(s.feet[s.swingFoot]);
        if (std::getenv("STANDSTEPDBG")) {
            const Eigen::Vector3d up = k.rotation[0].col(2);
            std::printf("   passo t %.3f: pe (%.3f %.3f %.3f) alvo (%.3f %.3f %.3f) pouso (%.2f %.2f) | CoM (%.3f %.3f) apoio (%.3f %.3f) | pelve %.1f graus\n",
                s.stepTime, sole[s.swingFoot].x(), sole[s.swingFoot].y(), sole[s.swingFoot].z(), swing.position.x(), swing.position.y(), swing.position.z(),
                s.swingLanding.x(), s.swingLanding.y(), comNow.x(), comNow.y(), sole[1 - s.swingFoot].x(), sole[1 - s.swingFoot].y(),
                std::acos(std::clamp(up.dot(s.pelvisTarget.col(2)), -1.0, 1.0)) * 180.0 / Pi);
        }
        input.tasks.push_back(wholeBodyPointTask3D(k, foot, sole[s.swingFoot], swing.position, swing.velocity, swing.acceleration, 400.0, 40.0, 20.0, "pe no ar"));
        input.tasks.push_back(wholeBodyOrientationTask3D(k, foot, s.swingRotation, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 400.0, 40.0, 5.0, "pe no ar (orientacao)"));
        if (hierarchy) {
            input.tasks[input.tasks.size() - 2].priority = 0;
            input.tasks[input.tasks.size() - 1].priority = 0;
        }
    }
    if (toeOff) {
        // A arfagem sobre a linha da ponta: positiva ergue o calcanhar; quintica de 0 ao alvo no apoio duplo.
        const int foot = static_cast<int>(s.feet[s.voluntaryFoot]);
        const Eigen::Vector3d middle = 0.5 * (toeA + toeB), back = heelMiddle - middle;
        Eigen::Vector3d axis = (toeB - toeA).normalized();
        if (axis.dot(back.cross(Eigen::Vector3d::UnitZ())) < 0.0) axis = -axis;
        const double angle = std::asin(std::clamp(back.z() / back.norm(), -1.0, 1.0));
        const double rate = axis.dot(k.angularVelocity[foot]);
        const double duration = std::max(1e-3, s.transferMove), tau = std::clamp(s.phaseTime / duration, 0.0, 1.0);
        const double goal = toeOffDegrees * Pi / 180.0;
        const double shape = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
        const double shapeRate = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / duration;
        const double shapeAcceleration = 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau) / (duration * duration);
        WholeBodyTask3D pitch;
        pitch.name = "saida pela ponta";
        pitch.matrix = axis.transpose() * k.angularJacobian[static_cast<std::size_t>(foot)];
        pitch.target = Eigen::VectorXd::Constant(1, goal * shapeAcceleration + 100.0 * (goal * shape - angle) + 20.0 * (goal * shapeRate - rate)
            - axis.dot(k.angularBias[static_cast<std::size_t>(foot)]));
        pitch.weight = 5.0;
        pitch.priority = hierarchy ? 0 : 1;
        input.tasks.push_back(pitch);
        s.toeOffLastAngle = angle;
    }
    if (s.hopping && s.hopPhase == 2) {
        // No voo os pes seguem a pelve (sem a postura esticar a perna contra o chao), recolhidos 3 cm em meio seno.
        const double flight = std::max(0.05, s.hopSettings.flightTime), u = std::clamp(s.hopTime / flight, 0.0, 1.0);
        const double lift = 0.03 * std::sin(Pi * u), liftRate = 0.03 * Pi / flight * std::cos(Pi * u);
        const double liftAcceleration = -0.03 * (Pi / flight) * (Pi / flight) * std::sin(Pi * u);
        for (int f = 0; f < 2; ++f) {
            const int foot = static_cast<int>(s.feet[f]);
            const Eigen::Vector3d target = k.origin[0] + s.hopFootOffset[f] + Eigen::Vector3d(0.0, 0.0, lift);
            const Eigen::Vector3d targetVelocity = k.originVelocity[0] + Eigen::Vector3d(0.0, 0.0, liftRate);
            WholeBodyTask3D point = wholeBodyPointTask3D(k, foot, sole[f], target, targetVelocity, Eigen::Vector3d(0.0, 0.0, liftAcceleration - 9.81),
                400.0, 40.0, 20.0, "pe no voo");
            point.priority = hierarchy ? 0 : 1;
            input.tasks.push_back(point);
        }
    }
    const std::vector<Eigen::Vector3d> zero(s.model.links.size(), Eigen::Vector3d::Zero());
    {
        // A postura por regiao: bracos, coluna e pescoco firmes; pernas livres para equilibrar.
        WholeBodyTask3D posture = wholeBodyPostureTask3D(s.model, state, s.referencePose, zero, zero, 100.0 * gain * gain, 20.0 * gain, 1.0);
        // Bracos e coluna com ganhos proprios (omega 20 e 14 rad/s): o tronco empurrado nao os deixa para tras.
        const double armOmega = std::getenv("STANDARMW") ? std::atof(std::getenv("STANDARMW")) : 20.0;
        const double spineOmega = std::getenv("STANDSPINEW") ? std::atof(std::getenv("STANDSPINEW")) : 14.0;
        const WholeBodyTask3D armPosture = wholeBodyPostureTask3D(s.model, state, s.referencePose, zero, zero, armOmega * armOmega, 2.0 * armOmega, 1.0);
        const WholeBodyTask3D spinePosture = wholeBodyPostureTask3D(s.model, state, s.referencePose, zero, zero, spineOmega * spineOmega, 2.0 * spineOmega, 1.0);
        const double armWeight = std::getenv("STANDARM") ? std::atof(std::getenv("STANDARM")) : 3.0;
        const double spineWeight = std::getenv("STANDSPINE") ? std::atof(std::getenv("STANDSPINE")) : 1.5;
        const double legWeight = std::getenv("STANDLEG") ? std::atof(std::getenv("STANDLEG")) : 0.2;
        for (std::size_t i = 1; i < s.model.links.size(); ++i) {
            const std::string& id = profile.links[i].id;
            const bool arm = id.find("Arm") != std::string::npos || id.find("Hand") != std::string::npos;
            const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos;
            const double weight = arm ? armWeight : leg ? legWeight : spineWeight;
            for (int a = 0; a < 3; ++a) {
                const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof < 0) continue;
                if (arm) posture.target[dof - 6] = armPosture.target[dof - 6];
                else if (!leg) posture.target[dof - 6] = spinePosture.target[dof - 6];
                posture.matrix.row(dof - 6) *= std::sqrt(weight);
                posture.target[dof - 6] *= std::sqrt(weight);
            }
        }
        input.tasks.push_back(posture);
        if (s.walking && s.stepping && !std::getenv("STANDNOSWINGKNEE")) {
            // A perna no ar prefere o joelho dobrado (configuracao privilegiada): sem isso o QP,
            // que so ve o instante, ergue o pe pelo quadril com a perna reta contra o batente.
            const double peak = std::getenv("STANDSWINGKNEE") ? std::atof(std::getenv("STANDSWINGKNEE")) : 0.9;
            const double phase = std::clamp(s.stepTime / s.stepDuration, 0.0, 1.0);
            std::vector<Eigen::Vector3d> swingPose = s.referencePose;
            const std::size_t shin = static_cast<std::size_t>(s.model.links[s.feet[s.swingFoot]].parent);
            swingPose[shin].x() = 0.1 + peak * std::sin(Pi * phase);
            const WholeBodyTask3D knee = wholeBodyPostureTask3D(s.model, state, swingPose, zero, zero, 400.0, 40.0, 1.0);
            const int dof = s.model.links[shin].dof[0];
            WholeBodyTask3D row;
            row.name = "joelho no ar";
            row.matrix = knee.matrix.row(dof - 6);
            row.target = knee.target.segment(dof - 6, 1);
            row.weight = std::getenv("STANDSWINGKNEEW") ? std::atof(std::getenv("STANDSWINGKNEEW")) : 2.0;
            input.tasks.push_back(row);
        }
    }
    const WholeBodyQPResult3D result = s.qp.solve(input);
    if (const char* when = std::getenv("STANDTASKDBG"); when && result.usable() && std::abs(s.clock - std::atof(when)) < 0.5 * Dt && input.jointLower.size()) {
        for (std::size_t i = 1; i < s.model.links.size(); ++i) {
            const std::string& id = profile.links[i].id;
            if (id.find("Shin") == std::string::npos && id.find("Thigh") == std::string::npos && id.find("Foot") == std::string::npos) continue;
            for (int a = 0; a < 3; ++a) {
                const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof < 0) continue;
                std::printf("   junta %-10s eixo %d: q %.3f [%.3f %.3f] v %.2f a %.1f\n", id.c_str(), a, state.jointCoordinates[i][a],
                    input.jointLower[dof - 6], input.jointUpper[dof - 6], state.velocity[dof], result.acceleration[dof]);
            }
        }
    }
    if (const char* when = std::getenv("STANDTASKDBG"); when && result.usable() && std::abs(s.clock - std::atof(when)) < 0.5 * Dt) {
        for (const WholeBodyTask3D& task : input.tasks) {
            const Eigen::VectorXd residual = task.matrix * result.acceleration - task.target;
            std::printf("   tarefa %-22s custo %10.2f  |alvo| %8.2f  |residuo| %8.3f\n", task.name.c_str(), task.weight * residual.squaredNorm(),
                task.target.norm(), residual.norm());
            if (task.name.rfind("centroidal", 0) == 0 || task.name.rfind("centro", 0) == 0)
                std::printf("      residuo por linha: %s\n", [&] { std::string t; for (int r = 0; r < residual.size(); ++r) { char b[32]; std::snprintf(b, 32, "%.3f ", residual[r]); t += b; } return t; }().c_str());
        }
    }
    if (std::getenv("STANDFORCEDBG") && s.walking) {
        const Eigen::Vector2d measured = s.model.totalMass * (comVelocityNow - s.lastComVelocity).head<2>() / Dt;
        if (int(s.clock / Dt) % 6 == 0)
            std::printf("   t %.2f forca horizontal: pedida (%.0f %.0f) planejada (%.0f %.0f) medida (%.0f %.0f)\n", s.clock, s.lastCommandedForce.x(),
                s.lastCommandedForce.y(), s.lastPlannedForce.x(), s.lastPlannedForce.y(), measured.x(), measured.y());
        s.lastCommandedForce = linearRate.head<2>();
        s.lastPlannedForce.setZero();
        if (result.usable())
            for (std::size_t c = 0; c < input.contacts.size(); ++c) s.lastPlannedForce += result.contactForces.segment<2>(3 * static_cast<int>(c));
    }
    s.lastComVelocity = comVelocityNow;
    if (std::getenv("STANDLOG") && (s.logTick++ % 30 == 0 || !result.usable())) {
        double worstRatio = 0.0;
        if (result.torque.size())
            for (int j = 0; j < s.model.dofCount - 6; ++j) worstRatio = std::max(worstRatio, std::abs(result.torque[j + 6]) / s.torqueLimit[j]);
        const Eigen::Vector3d up = k.rotation[0].col(2);
        std::printf("   violacoes: igualdade %.2e, desigualdade %.2e\n", result.equalityViolation, result.inequalityViolation);
        std::printf("t %.2f: %s it %d, contatos %zu, CoM erro (%.3f %.3f %.3f), vel CoM %.3f, pelve inclinada %.1f graus, escorregar %.3f, torque %.2f\n",
            s.logTick * Dt, statusName(result.status), result.iterations, input.contacts.size(), (s.comTarget - d.centerOfMass).x(),
            (s.comTarget - d.centerOfMass).y(), (s.comTarget - d.centerOfMass).z(), (d.centroidalMatrix.topRows(3) * state.velocity / s.model.totalMass).norm(),
            std::acos(std::clamp(up.dot(s.pelvisTarget.col(2)), -1.0, 1.0)) * 180.0 / Pi, report.slip, worstRatio);
    }
    if (std::getenv("STANDMARGIN") && s.logTick % 30 == 1 && raw.contacts.size() >= 3) {
        // O centro de massa no chao contra o poligono dos contatos (fecho convexo 2D).
        std::vector<Eigen::Vector2d> points;
        for (const RagdollContactPoint3D& contact : raw.contacts) points.emplace_back(contact.position.x, contact.position.y);
        std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
        std::vector<Eigen::Vector2d> hull;
        const auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return (a - o).x() * (b - o).y() - (a - o).y() * (b - o).x(); };
        for (int pass = 0; pass < 2; ++pass) {
            const std::size_t start = hull.size();
            for (std::size_t i = 0; i < points.size(); ++i) {
                const Eigen::Vector2d& p = pass == 0 ? points[i] : points[points.size() - 1 - i];
                while (hull.size() >= start + 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0) hull.pop_back();
                hull.push_back(p);
            }
            hull.pop_back();
        }
        const Eigen::Vector2d com(d.centerOfMass.x(), d.centerOfMass.y());
        double margin = 1e9;
        for (std::size_t i = 0; i < hull.size(); ++i) {
            const Eigen::Vector2d a = hull[i], b = hull[(i + 1) % hull.size()];
            const Eigen::Vector2d edge = (b - a).normalized();
            margin = std::min(margin, edge.x() * (com - a).y() - edge.y() * (com - a).x());
        }
        std::printf("   margem do CoM dentro do apoio: %.3f m (%zu pontos, %zu no fecho)\n", margin, points.size(), hull.size());
    }
    if (std::getenv("STANDDBG") && s.debugTicks++ < std::atoi(std::getenv("STANDDBG"))) {
        double sumZ = 0.0;
        for (int c = 0; c < int(input.contacts.size()); ++c) sumZ += result.contactForces.size() ? result.contactForces[3 * c + 2] : 0.0;
        double worstRatio = 0.0;
        if (result.torque.size())
            for (int j = 0; j < s.model.dofCount - 6; ++j) worstRatio = std::max(worstRatio, std::abs(result.torque[j + 6]) / s.torqueLimit[j]);
        std::printf("tick %3d: contatos %zu, %s, it %d, %.0f us, soma Fz %.1f N (peso %.1f), torque max %.2f do teto, CoM erro %.3f m, pelve z %.3f, residuo %.2e\n",
            s.debugTicks, input.contacts.size(), statusName(result.status), result.iterations, result.solveMicros, sumZ, s.model.totalMass * 9.81,
            worstRatio, (s.comTarget - d.centerOfMass).norm(), double(raw.links[0].position.z), result.dynamicsResidual);
        if (s.debugTicks == 3) {
            for (const auto& p : input.contacts) std::printf("   canto suposto link %d em (%.3f %.3f %.3f)\n", p.link, p.pointWorld.x(), p.pointWorld.y(), p.pointWorld.z());
            for (const auto& c : raw.contacts) std::printf("   contato Jolt link %u em (%.3f %.3f %.3f) normal (%.2f %.2f %.2f) J %.3f\n", c.linkIndex, c.position.x, c.position.y, c.position.z, c.normal.x, c.normal.y, c.normal.z, c.normalImpulseNewtonSeconds);
        }
    }
    report.status = result.status;
    report.path = result.path;
    if (std::getenv("STANDSTEPDBG") && s.stepping) {
        const int foot = static_cast<int>(s.feet[s.swingFoot]);
        const Eigen::Vector3d predicted = result.usable() ? Eigen::Vector3d(articulationPointJacobian3D(k, foot, sole[s.swingFoot]) * result.acceleration
            + articulationPointBias3D(k, foot, sole[s.swingFoot])) : Eigen::Vector3d::Zero();
        double worstRatio = 0.0;
        if (result.torque.size()) for (int j = 0; j < s.model.dofCount - 6; ++j) worstRatio = std::max(worstRatio, std::abs(result.torque[j + 6]) / s.torqueLimit[j]);
        std::printf("      QP %s it %d viol %.1e/%.1e frio %d; acel. prevista do pe (%.1f %.1f %.1f); torque max %.2f\n", statusName(result.status), result.iterations,
            result.equalityViolation, result.inequalityViolation, int(result.coldRetry), predicted.x(), predicted.y(), predicted.z(), worstRatio);
    }
    if (std::getenv("STANDQPTIME") && s.logTick % 60 == 0)
        std::printf("   QP: montar %.0f us, resolver %.0f us (it %d, caminho %d)\n", result.buildMicros, result.innerSolveMicros, result.iterations, int(result.path));
    s.predictedAcceleration = result.usable() ? result.acceleration : Eigen::VectorXd();
    if (result.usable() && result.torque.size()) {
        const Eigen::VectorXd motor = motorMap * result.torque.tail(s.model.dofCount - 6);
        double worst = 0.0;
        for (int j = 0; j < s.model.dofCount - 6; ++j) worst = std::max(worst, std::abs(motor[j]) / s.torqueLimit[j]);
        s.motorWorstRatio = std::max(s.motorWorstRatio, worst);
        if (worst > 1.001) ++s.motorClippedTicks;
    }
    for (int f = 0; f < 2; ++f) { s.plannedFootForce[f].setZero(); s.plannedCop[f].setZero(); }
    if (result.usable())
        for (int f = 0; f < 2; ++f) {
            double total = 0.0;
            for (std::size_t c = 0; c < input.contacts.size(); ++c)
                if (input.contacts[c].link == static_cast<int>(s.feet[f])) {
                    const double fz = result.contactForces[3 * static_cast<int>(c) + 2];
                    s.plannedCop[f] += fz * input.contacts[c].pointWorld;
                    total += fz;
                }
            if (total > 1.0) s.plannedCop[f] /= total;
        }
    if (result.usable())
        for (std::size_t c = 0; c < input.contacts.size(); ++c)
            for (int f = 0; f < 2; ++f)
                if (input.contacts[c].link == static_cast<int>(s.feet[f])) s.plannedFootForce[f] += result.contactForces.segment<3>(3 * static_cast<int>(c));
    report.solveMicros = result.solveMicros;
    report.iterations = result.iterations;
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(s.model.dofCount);
    if (result.usable()) {
        torque = result.torque;
    } else {
        // A politica de falha: amortecimento articular limitado, nunca forca salvadora.
        torque.tail(s.model.dofCount - 6) = (-5.0 * state.velocity.tail(s.model.dofCount - 6)).cwiseMax(-s.torqueLimit).cwiseMin(s.torqueLimit);
    }
    ActuationSettings3D settings;
    settings.mode = ActuationMode3D::ExplicitTorque;
    ActuationReference3D reference;
    if (const char* act = std::getenv("STANDACT")) {
        if (std::string(act) == "B") settings.mode = ActuationMode3D::LowImpedance;
        if (std::string(act) == "D") settings.mode = ActuationMode3D::ExplicitTorqueImplicitDamping;
        settings.stiffnessPerInertia = std::getenv("STANDK") ? std::atof(std::getenv("STANDK")) : 150.0;
        settings.dampingPerInertia = std::getenv("STANDD") ? std::atof(std::getenv("STANDD")) : 20.0;
        settings.profileStiffnessFraction = std::getenv("STANDKP") ? std::atof(std::getenv("STANDKP")) : 0.0;
        // O alvo da mola: a configuracao prevista para o fim do passo (a mola nao briga com o QP).
        reference.coordinates.assign(s.model.links.size(), Eigen::Vector3d::Zero());
        reference.rates.assign(s.model.links.size(), Eigen::Vector3d::Zero());
        const Eigen::VectorXd planned = result.usable() ? Eigen::VectorXd(result.acceleration) : Eigen::VectorXd::Zero(s.model.dofCount);
        for (std::size_t i = 1; i < s.model.links.size(); ++i)
            for (int a = 0; a < 3; ++a) {
                const int dof = s.model.links[i].dof[static_cast<std::size_t>(a)];
                if (dof < 0) continue;
                reference.rates[i][a] = state.velocity[dof] + planned[dof] * Dt;
                reference.coordinates[i][a] = state.jointCoordinates[i][a] + reference.rates[i][a] * Dt;
            }
    }
    const std::vector<RagdollDriveTarget3D> targets = actuationTargets3D(profile, s.model, state, torque,
        reference, d.massMatrix.diagonal(), settings);
    std::vector<RagdollDriveTarget3D> finalTargets = targets;
    if (const char* ankle = std::getenv("STANDANKLE")) {
        // So os tornozelos com mola implicita rigida na configuracao prevista do fim do passo.
        const double fraction = std::atof(ankle);
        const Eigen::VectorXd planned = result.usable() ? Eigen::VectorXd(result.acceleration) : Eigen::VectorXd::Zero(s.model.dofCount);
        for (RagdollDriveTarget3D& target : finalTargets) {
            if (target.linkIndex != s.feet[0] && target.linkIndex != s.feet[1]) continue;
            const ArticulationLink3D& link = s.model.links[target.linkIndex];
            Eigen::Vector3d next = state.jointCoordinates[target.linkIndex];
            for (int a = 0; a < 3; ++a) {
                const int dof = link.dof[static_cast<std::size_t>(a)];
                if (dof >= 0) next[a] += (state.velocity[dof] + planned[dof] * Dt) * Dt;
            }
            const Eigen::Matrix3d rotation = articulationJointRotation3D(link, next);
            const Eigen::AngleAxisd axisAngle(rotation);
            const Eigen::Vector3d vector = axisAngle.axis() * axisAngle.angle();
            const int a = static_cast<int>(target.axis);
            const RagdollAxisDefinition3D& axis = profile.links[target.linkIndex].inboundJoint.axes[static_cast<std::size_t>(a)];
            target.positionRadians = static_cast<float>(vector[a]);
            target.stiffnessScale = static_cast<float>(std::clamp(fraction / 5.0, 0.0, 2.0));
            target.dampingScale = static_cast<float>(std::clamp(fraction / 2.0, 0.0, 2.5));
            (void)axis;
        }
    }
    s.scene->setRagdollActiveDriveTargets(s.ragdoll, finalTargets, false);
    s.previousTorque = torque;
    if (push.squaredNorm() > 0.0)
        s.scene->applyRagdollLinkForce(s.ragdoll, static_cast<std::uint32_t>(pushLink), { float(push.x()), float(push.y()), float(push.z()) }, {});
    s.scene->simulate(static_cast<float>(Dt));
    return report;
}

} // namespace

int main(int argc, char** argv) {
    double seconds = 60.0;
    bool pushes = false;
    std::string dumpPath;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--segundos" && i + 1 < argc) seconds = std::stod(argv[++i]);
        else if (arg == "--empurroes") pushes = true;
        else if (arg == "--dump" && i + 1 < argc) dumpPath = argv[++i];
        else {
            std::fprintf(stderr, "uso: MatterStandBench [--segundos 60] [--empurroes]\n");
            return 2;
        }
    }
    RagdollProfile3D profile = loadRagdollProfile3D(
        std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    if (std::getenv("STANDBOX")) {
        // Ensaio: o pe como caixa simples (a caixa justa da bota), no lugar da casca convexa.
        for (RagdollLinkDefinition3D& link : profile.links)
            if (link.id == "LeftFoot" || link.id == "RightFoot") link.collider.shape = RagdollColliderShape3D::Box;
    }
    {
        Stand s;
        setup(s, profile);
        const Vec3 startPelvis = s.scene->ragdollStateView(s.ragdoll)->links[0].position;
        const int steps = static_cast<int>(seconds / Dt);
        double pelvisSquared = 0.0, worstSlip = 0.0, micros = 0.0, worstMicros = 0.0;
        int failures = 0, samples = 0, iterations = 0, topOnly = 0, flat = 0, weighted = 0;
        bool fallen = false;
        WholeBodyQPStatus3D lastFailure = WholeBodyQPStatus3D::Solved;
        for (int i = 0; i < steps; ++i) {
            const StepReport r = step(s, profile, Eigen::Vector3d::Zero(), 0);
            if (r.fallen) { fallen = true; std::printf("caiu em t = %.2f s\n", i * Dt); break; }
            if (r.status != WholeBodyQPStatus3D::Solved) { ++failures; lastFailure = r.status; }
            topOnly += r.path == WholeBodyQPPath3D::TopOnly;
            flat += r.path == WholeBodyQPPath3D::FlatFallback;
            weighted += r.path == WholeBodyQPPath3D::WeightedPriority;
            if (i >= 120) {
                pelvisSquared += r.pelvisSpeed * r.pelvisSpeed;
                worstSlip = std::max(worstSlip, r.slip);
                micros += r.solveMicros;
                worstMicros = std::max(worstMicros, r.solveMicros);
                iterations += r.iterations;
                ++samples;
            }
        }
        std::printf("# De pe, ajuda zero, %.0f s\n", seconds);
        std::printf("- caiu: %s\n- pelve RMS: %.4f m/s (portao < 0,05)\n- escorregar maximo no apoio: %.4f m/s (portao <= 0,05)\n",
            fallen ? "SIM" : "nao", std::sqrt(pelvisSquared / std::max(1, samples)), worstSlip);
        std::printf("- QP: %d falhas (ultima: %s); %.0f us de media, %.0f us no pior; %.1f iteracoes de media\n", failures,
            statusName(lastFailure), micros / std::max(1, samples), worstMicros, double(iterations) / std::max(1, samples));
        if (topOnly + flat + weighted)
            std::printf("- QP em dois niveis: %d ticks com as de cima pesadas, %d so com o nivel de cima, %d no QP ponderado unico\n", weighted, topOnly, flat);
        std::printf("- torque no referencial do motor: %d ticks acima do teto (recortados), pior %.2f do teto\n", s.motorClippedTicks, s.motorWorstRatio);
        if (s.hopping)
            std::printf("- salto: %d apoios, %d voos medidos (%.0f ms de media, pedido %.0f), %d pousos perdidos, %d decolagens sem separacao; pico de forca planejado/pedido %.2f (media)\n",
                s.hops, s.flights, s.flights ? 1000.0 * s.flightSum / s.flights : 0.0, 1000.0 * s.hopSettings.flightTime, s.missedLandings, s.failedTakeoffs,
                s.hops ? s.hopPeakRatioSum / s.hops : 0.0);
        if (std::getenv("STANDWALK")) {
            const Vec3 end = s.scene->ragdollStateView(s.ragdoll)->links[0].position;
            std::printf("- marcha: %d passos, pelve andou (%.2f %.2f) m, %s\n", s.steps, end.x - startPelvis.x, end.y - startPelvis.y,
                s.walking ? "ainda andando" : "parou");
            if (s.liftoffs) std::printf("- decolagem: %d, forca planejada no pe no tick anterior %.0f N (media); ainda tocando %.0f ms depois (media)\n",
                s.liftoffs, s.liftoffForceSum / s.liftoffs, s.separations ? 1000.0 * Dt * s.separationSum / s.separations : 0.0);
            if (s.copClampTicks) std::printf("- CoP pedido fora do apoio: %.1f cm de media, %.1f cm no pior\n", 100.0 * s.copClampSum / s.copClampTicks, 100.0 * s.copClampWorst);
            if (s.toeOffs) std::printf("- saida pela ponta: %d, sola a %.1f graus na decolagem (media)\n", s.toeOffs, s.toeOffAngleSum / s.toeOffs * 180.0 / Pi);
        }
    }
    if (!dumpPath.empty()) {
        // Os cenarios das folhas: de pe 3 s; empurroes de frente, de lado e de costas.
        FILE* file = std::fopen(dumpPath.c_str(), "w");
        if (!file) { std::fprintf(stderr, "nao abriu %s\n", dumpPath.c_str()); return 1; }
        struct Shot { const char* name; double angle; double impulse; };
        std::vector<Shot> shots = { { "de_pe", 0.0, 0.0 }, { "empurrao_frente_30", 0.0, 30.0 }, { "empurrao_lado_60", 90.0, 60.0 },
            { "empurrao_225_120", 225.0, 120.0 }, { "empurrao_frente_60_cai", 0.0, 60.0 } };
        if (std::getenv("STANDWALK")) shots = { { "marcha", 0.0, 0.0 } };
        static std::string customName;
        if (const char* custom = std::getenv("STANDPUSH")) {
            double angle = 0.0, impulse = 0.0;
            std::sscanf(custom, "%lf,%lf", &angle, &impulse);
            customName = "empurrao_" + std::to_string(int(angle)) + "_" + std::to_string(int(impulse));
            shots = { { customName.c_str(), angle, impulse } };
        }
        for (const Shot& shot : shots) {
            Stand s;
            setup(s, profile);
            std::fprintf(file, "S %s\n", shot.name);
            const double angle = shot.angle * Pi / 180.0;
            const int ticks = std::getenv("STANDDUMPTICKS") ? std::atoi(std::getenv("STANDDUMPTICKS")) : 360;
            for (int i = 0; i < ticks; ++i) {
                const bool pushing = shot.impulse > 0.0 && i >= 120 && i < 132;
                if (i == 120 && shot.impulse > 0.0) std::fprintf(file, "E empurrao %.0f N.s\n", shot.impulse);
                const Eigen::Vector3d force = pushing ? Eigen::Vector3d(Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0) * (shot.impulse / 0.1))
                                                      : Eigen::Vector3d(Eigen::Vector3d::Zero());
                const bool fallen = step(s, profile, force, s.chest).fallen;
                if (i % 4 == 0) dumpFrame(file, profile, *s.scene->ragdollStateView(s.ragdoll), i * Dt);
                if (fallen && i > 200) {
                    std::printf("resultado %s: caiu em t = %.2f s (%d passos)\n", shot.name, i * Dt, s.steps);
                    break;
                }
                if (i + 1 == ticks) std::printf("resultado %s: de pe aos %.1f s (%d passos)\n", shot.name, ticks * Dt, s.steps);
            }
        }
        std::fclose(file);
        std::printf("dump: %s\n", dumpPath.c_str());
        return 0;
    }
    if (pushes) {
        std::printf("\n# Empurrao de 0,1 s no peito (sem passo), o maior impulso aguentado por direcao\n");
        for (int direction = 0; direction < 8; ++direction) {
            if (std::getenv("STANDENVDIR") && std::atoi(std::getenv("STANDENVDIR")) != direction) continue;
            const double angle = direction * Pi / 4.0;
            double held = 0.0;
            for (double impulse = 10.0; impulse <= 120.0; impulse += 10.0) {
                Stand s;
                setup(s, profile);
                bool fallen = false;
                for (int i = 0; i < 120 + 1200 && !fallen; ++i) {
                    const bool pushing = i >= 120 && i < 132;
                    const Eigen::Vector3d force = pushing ? Eigen::Vector3d(Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0) * (impulse / 0.1)) : Eigen::Vector3d(Eigen::Vector3d::Zero());
                    fallen = step(s, profile, force, s.chest).fallen;
                    if (fallen && std::getenv("STANDENVDIR")) std::printf("   %.0f N.s: caiu em t = %.2f s\n", impulse, i * Dt);
                }
                if (fallen) break;
                held = impulse;
            }
            std::printf("- direcao %3.0f graus: aguentou %.0f N.s\n", angle * 180.0 / Pi, held);
        }
    }
    return 0;
}
