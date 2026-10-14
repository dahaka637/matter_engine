// O Experimento A da reformulacao do RL (vault: Aprendizado_por_Reforco/
// novo_plano/plano_claude_rl.md, secao 14.1): os motores do corpo v2 pelo
// contrato de verdade (a reta entre decisoes e os alvos do treino).
//
//  1. O balanco do quadril a 5, 9 e 13 rad/s (o trote, a corrida e o sprint
//     dos clipes), sem gravidade, em cinco opcoes de alvo de velocidade, com
//     e sem o ruido de exploracao de hoje (~0,15 rad por junta): o quanto a
//     junta acompanha, o tempo no teto do motor e o torque aleatorio que o
//     ruido produz.
//  2. A ponta do pe: de pe na postura natural (o giro da ponta parada, as
//     trocas de sentido dele - o tremor numerico seria ~60/s - e o contato) e
//     depois os tornozelos estendendo, nos solvers 10/6 (treino) e 20/20
//     (jogo). Sem uma politica que equilibre, o corpo nao fica na ponta dos
//     pes (tomba para tras): a ponta com o peso em cima se ve no Experimento B.
//
// Imprime Markdown. Uso: MatterRLMotorProbe [corpo.json]
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/BodyContract3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

using namespace MatterEngine;

constexpr float FixedStep = 1.0f / 120.0f;
constexpr std::uint32_t Substeps = 4;
constexpr float Decision = FixedStep * Substeps;
constexpr float Pi = 3.14159265358979f;

struct World {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D ragdoll;
};

void makeWorld(World& world, const RLBody3D& body, bool gravity, std::uint32_t position, std::uint32_t velocity) {
    PhysicsSceneSettings3D settings;
    settings.gravity = gravity ? Vec3 { 0.0f, 0.0f, -9.81f } : Vec3 {};
    settings.workerThreadCount = 1;
    settings.runJobsInline = true;
    world.scene = world.engine.createScene(settings, world.materials);
    if (gravity) {
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 20.0f, 20.0f, 0.5f };
        shape.materialId = "grass";
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.5f };
        ground.materialId = "grass";
        (void)world.scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
    }
    RagdollSpawnDefinition3D spawn;
    spawn.pelvisPosition = { 0.0f, 0.0f, gravity ? body.standingPelvisHeight + 0.05f : 3.0f };
    spawn.active = true;
    spawn.rigidityPercent = 0.0f;
    spawn.solverPositionIterations = position;
    spawn.solverVelocityIterations = velocity;
    world.ragdoll = world.scene->createRagdoll(body.profile, spawn);
}

// O corpo numa postura (as coordenadas por link), apoiado (2 mm acima) ou no ar.
void placePosture(World& world, const RLBody3D& body, const RLPosture3D& posture, bool grounded) {
    const Quaternion root = (body.profile.links.front().modelOrientation * posture.rootRotation).normalized();
    float lift = 3.0f;
    if (grounded) {
        const RLBodyPose3D probe = rlBodyPose3D(body.profile, {}, root, posture.jointCoordinates);
        lift = 0.002f - rlBodyLowestFootPoint3D(body, probe);
    }
    std::vector<PhysicsBodyState3D> links;
    std::vector<Vec3> rest(posture.jointCoordinates.size());
    rlBodyLinkStates3D(body.profile, { 0.0f, 0.0f, lift }, root, {}, {}, posture.jointCoordinates, rest, links);
    world.scene->resetRagdollState(world.ragdoll, links);
}

std::size_t dofIndex(const RLBody3D& body, const std::string& name) {
    for (std::size_t d = 0; d < body.dofs.size(); ++d)
        if (body.dofs[d].name == name) return d;
    throw std::runtime_error("eixo ausente: " + name);
}

// A acao que pede um alvo (o inverso de RLBodyDof3D::target, sem o curso).
float actionFor(const RLBodyDof3D& dof, float target) {
    return (target - dof.baseRadians) / dof.actionScale;
}

struct Option {
    const char* name;
    BodyVelocityTarget3D velocity;
    bool correlated;
    float dampingScale;
};

struct SwingResult {
    float trackingRms = 0.0f;
    float peakSpeed = 0.0f;
    float saturated = 0.0f;
    float meanTorque = 0.0f;
    std::vector<float> torque;
};

// O quadril (LeftThigh.swing1) seguindo q*(t) = centro + A sin(wt), com o pico
// de velocidade A w, por 4 s; os outros eixos na postura base.
SwingResult swing(const RLBody3D& body, const Option& option, float peakSpeed, float noise, std::uint64_t seed) {
    World world;
    makeWorld(world, body, false, body.solverPositionIterations, body.solverVelocityIterations);
    placePosture(world, body, body.postures.front(), false);
    BodyContractSettings3D settings;
    settings.velocityTarget = option.velocity;
    const BodyContract3D contract(body, settings);
    BodyContractState3D state;
    contract.reset(state, *world.scene->ragdollStateView(world.ragdoll));
    const std::size_t hip = dofIndex(body, "LeftThigh.swing1");
    const RLBodyDof3D& dof = body.dofs[hip];
    const float amplitude = 0.6f, center = -0.5f, omega = peakSpeed / amplitude;
    std::mt19937_64 rng(seed);
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    // O ruido em unidades de acao: o desvio de hoje (0,15 rad) dividido pela
    // escala; correlacionado: Ornstein-Uhlenbeck com 0,1 s de memoria e o
    // mesmo desvio.
    const float sigma = 0.15f / dof.actionScale;
    std::vector<float> walk(body.dofs.size(), 0.0f);
    const float keep = std::exp(-Decision / 0.1f);
    std::vector<float> mean(body.dofs.size(), 0.0f), action(body.dofs.size(), 0.0f);
    std::vector<RagdollDriveTarget3D> targets;
    PlayerAttributes3D player;
    SwingResult result;
    double error2 = 0.0;
    std::uint32_t samples = 0, saturated = 0;
    double torqueSum = 0.0;
    const std::uint32_t decisions = static_cast<std::uint32_t>(4.0f / Decision);
    for (std::uint32_t k = 0; k < decisions; ++k) {
        const float tNext = static_cast<float>(k + 1) * Decision;
        std::fill(mean.begin(), mean.end(), 0.0f);
        mean[hip] = actionFor(dof, center + amplitude * std::sin(omega * tNext));
        for (std::size_t d = 0; d < mean.size(); ++d) {
            float e = noise > 0.0f ? gauss(rng) * sigma * noise : 0.0f;
            if (option.correlated && noise > 0.0f) {
                walk[d] = keep * walk[d] + std::sqrt(1.0f - keep * keep) * gauss(rng) * sigma * noise;
                e = walk[d];
            }
            action[d] = mean[d] + e;
        }
        contract.beginDecision(state, action, mean);
        for (std::uint32_t s = 0; s < Substeps; ++s) {
            contract.writeTargets(state, s, player, targets);
            for (RagdollDriveTarget3D& t : targets) t.dampingScale *= option.dampingScale;
            world.scene->setRagdollActiveDriveTargets(world.ragdoll, targets, false);
            world.scene->simulate(FixedStep);
            const RagdollState3D& now = *world.scene->ragdollStateView(world.ragdoll);
            const float t = (static_cast<float>(k) * Substeps + static_cast<float>(s + 1)) * FixedStep;
            const auto axis = static_cast<std::size_t>(dof.axis);
            const float q = now.joints[dof.linkIndex].positionRadians[axis];
            const float qd = now.joints[dof.linkIndex].velocityRadiansPerSecond[axis];
            const float tau = now.joints[dof.linkIndex].motorTorqueNewtonMeters[axis];
            const float limit = now.joints[dof.linkIndex].motorTorqueLimitNewtonMeters[axis];
            if (t > 1.0f) {
                const float wanted = center + amplitude * std::sin(omega * t);
                error2 += (q - wanted) * (q - wanted);
                result.peakSpeed = std::max(result.peakSpeed, std::abs(qd));
                if (limit > 0.0f && std::abs(tau) >= 0.98f * limit) ++saturated;
                torqueSum += std::abs(tau);
                ++samples;
            }
            result.torque.push_back(tau);
        }
    }
    result.trackingRms = static_cast<float>(std::sqrt(error2 / std::max(1u, samples)));
    result.saturated = static_cast<float>(saturated) / static_cast<float>(std::max(1u, samples));
    result.meanTorque = static_cast<float>(torqueSum / std::max(1u, samples));
    return result;
}

float noiseTorque(const SwingResult& noisy, const SwingResult& clean) {
    double sum = 0.0;
    std::size_t n = 0;
    for (std::size_t i = 120; i < std::min(noisy.torque.size(), clean.torque.size()); ++i) {
        const double d = noisy.torque[i] - clean.torque[i];
        sum += d * d;
        ++n;
    }
    return static_cast<float>(std::sqrt(sum / std::max<std::size_t>(1, n)));
}

struct ToeResult {
    float holdSeconds = 0.0f;
    // Parado (0,1 a 0,6 s): o giro da ponta e do tornozelo do outro lado, e o
    // tempo da ponta no chao.
    float quietToeSpeedRms = 0.0f;
    float quietAnkleSpeedRms = 0.0f;
    float quietToeContact = 0.0f;
    // As trocas de sentido do giro da ponta por segundo (o tremor e uma ida e
    // volta a cada passo de fisica: ~60/s; um corpo assentando, poucas).
    float quietToeReversals = 0.0f;
    float quietAnkleReversals = 0.0f;
    // Calcanhar subindo (0,6 a 1,4 s): a maior carga na ponta (pesos), o
    // angulo da ponta nessa hora (graus), o quanto o calcanhar subiu (cm), o
    // giro da ponta e as trocas de contato dela.
    float riseToeLoadMax = 0.0f;
    float riseToeAngle = 0.0f;
    float riseHeelLift = 0.0f;
    float riseToeSpeedRms = 0.0f;
    float riseToeFlips = 0.0f;
};

// De pe na postura natural (os pes chatos no chao; os alvos na postura, sem
// ruido); aos 0,6 s os dois tornozelos estendem 0,6 rad em 0,4 s (ele sobe na
// ponta dos pes e o peso vai para as pontas). Ate a queda ou 1,4 s.
ToeResult toes(const RLBody3D& body, std::uint32_t position, std::uint32_t velocity) {
    World world;
    makeWorld(world, body, true, position, velocity);
    const RLPosture3D& posture = body.postures.front();
    placePosture(world, body, posture, true);
    const BodyContract3D contract(body);
    BodyContractState3D state;
    contract.reset(state, *world.scene->ragdollStateView(world.ragdoll));
    const std::size_t ankle = dofIndex(body, "LeftFoot.twist");
    const RLBodyDof3D& toeDof = body.dofs[dofIndex(body, "LeftToe.twist")];
    const std::size_t otherAnkleIndex = dofIndex(body, "RightFoot.twist");
    const RLBodyDof3D& otherAnkle = body.dofs[otherAnkleIndex];
    std::vector<float> action(body.dofs.size());
    std::vector<RagdollDriveTarget3D> targets;
    PlayerAttributes3D player;
    ToeResult result;
    double quietToe2 = 0.0, quietAnkle2 = 0.0, riseToe2 = 0.0;
    std::uint32_t quietSamples = 0, quietContact = 0, riseSamples = 0, flips = 0;
    std::uint32_t toeReversals = 0, ankleReversals = 0;
    float lastToeSpeed = 0.0f, lastAnkleSpeed = 0.0f;
    bool lastToeContact = false;
    const float headFloor = 0.7f * posture.standingHeadHeight;
    const float heelStart = world.scene->ragdollStateView(world.ragdoll)->links[body.leftFoot].position.z;
    const std::uint32_t decisions = static_cast<std::uint32_t>(1.4f / Decision);
    for (std::uint32_t k = 0; k < decisions; ++k) {
        const float t0 = static_cast<float>(k) * Decision;
        for (std::size_t d = 0; d < body.dofs.size(); ++d) action[d] = actionFor(body.dofs[d], posture.dofRadians[d]);
        // Na ponta dos pes: os dois tornozelos estendem juntos.
        if (t0 > 0.6f)
            for (const std::size_t a : { ankle, otherAnkleIndex })
                action[a] = actionFor(body.dofs[a], posture.dofRadians[a] + 0.6f * std::min(1.0f, (t0 - 0.6f) / 0.4f));
        contract.beginDecision(state, action, action);
        bool fallen = false;
        for (std::uint32_t s = 0; s < Substeps; ++s) {
            contract.writeTargets(state, s, player, targets);
            world.scene->setRagdollActiveDriveTargets(world.ragdoll, targets, false);
            world.scene->simulate(FixedStep);
            const RagdollState3D& now = *world.scene->ragdollStateView(world.ragdoll);
            const float t = (static_cast<float>(k) * Substeps + static_cast<float>(s + 1)) * FixedStep;
            const float toeSpeed = now.joints[toeDof.linkIndex].velocityRadiansPerSecond[0];
            const float ankleSpeed = now.joints[otherAnkle.linkIndex].velocityRadiansPerSecond[0];
            bool toeContact = false;
            float toeForce = 0.0f;
            for (const RagdollContactPoint3D& c : now.contacts) {
                if (c.linkIndex == toeDof.linkIndex && c.otherMotion == RagdollContactMotion3D::Static) {
                    toeContact = true;
                    toeForce += c.normalImpulseNewtonSeconds / FixedStep;
                }
            }
            if (t > 0.1f && t <= 0.6f) {
                if (toeSpeed * lastToeSpeed < 0.0f && std::abs(toeSpeed) > 0.05f) ++toeReversals;
                if (ankleSpeed * lastAnkleSpeed < 0.0f && std::abs(ankleSpeed) > 0.05f) ++ankleReversals;
                quietToe2 += toeSpeed * toeSpeed;
                quietAnkle2 += ankleSpeed * ankleSpeed;
                quietContact += toeContact ? 1u : 0u;
                ++quietSamples;
            } else if (t > 0.6f) {
                riseToe2 += toeSpeed * toeSpeed;
                if (riseSamples > 0 && toeContact != lastToeContact) ++flips;
                ++riseSamples;
                const float load = toeForce / (body.profile.totalMassKg * 9.81f);
                if (load > result.riseToeLoadMax) {
                    result.riseToeLoadMax = load;
                    result.riseToeAngle = now.joints[toeDof.linkIndex].positionRadians[0] * 180.0f / Pi;
                }
                result.riseHeelLift = std::max(result.riseHeelLift, 100.0f * (now.links[body.leftFoot].position.z - heelStart));
            }
            lastToeContact = toeContact;
            lastToeSpeed = toeSpeed;
            lastAnkleSpeed = ankleSpeed;
            if (now.links[body.head].position.z < headFloor) fallen = true;
        }
        result.holdSeconds = t0 + Decision;
        if (fallen) break;
    }
    result.quietToeSpeedRms = static_cast<float>(std::sqrt(quietToe2 / std::max(1u, quietSamples)));
    result.quietAnkleSpeedRms = static_cast<float>(std::sqrt(quietAnkle2 / std::max(1u, quietSamples)));
    result.quietToeContact = static_cast<float>(quietContact) / static_cast<float>(std::max(1u, quietSamples));
    result.quietToeReversals = static_cast<float>(toeReversals) / 0.5f;
    result.quietAnkleReversals = static_cast<float>(ankleReversals) / 0.5f;
    result.riseToeSpeedRms = static_cast<float>(std::sqrt(riseToe2 / std::max(1u, riseSamples)));
    result.riseToeFlips = static_cast<float>(flips) / std::max(0.01f, static_cast<float>(riseSamples) * FixedStep);
    return result;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::string path = argc > 1 ? argv[1] : std::string(MATTERENGINE_ASSETS_DIR) + "/rl/bodies/football_player_v2.json";
        const RLBody3D body = loadRLBody3D(path);
        const std::size_t hip = dofIndex(body, "LeftThigh.swing1");
        const RLBodyDof3D& dof = body.dofs[hip];
        std::printf("# Experimento A: os motores do corpo %s\n\n", body.id.c_str());
        std::printf("Quadril (LeftThigh.swing1): rigidez %.0f N.m/rad, amortecimento %.1f N.m.s/rad, teto %.0f N.m; "
                    "ruido de exploracao 0,15 rad por decisao (%.2f em unidades de acao).\n\n",
            dof.stiffness, dof.damping, dof.maximumTorque, 0.15f / dof.actionScale);
        const Option options[] = {
            { "1. velocidade zero", BodyVelocityTarget3D::Zero, false, 1.0f },
            { "2. reta da acao sorteada", BodyVelocityTarget3D::SampledSlope, false, 1.0f },
            { "3. reta da acao media", BodyVelocityTarget3D::MeanSlope, false, 1.0f },
            { "4. reta sorteada, ruido correlacionado", BodyVelocityTarget3D::SampledSlope, true, 1.0f },
            { "5. velocidade zero, amortecimento x0,4", BodyVelocityTarget3D::Zero, false, 0.4f },
        };
        std::printf("## 1. O balanco do quadril (sem gravidade, 4 s; medido depois do 1o segundo)\n\n");
        std::printf("| Opcao | Pico pedido (rad/s) | Erro sem ruido (rad) | Pico alcancado (rad/s) | No teto sem ruido | "
                    "Erro com ruido (rad) | No teto com ruido | Torque aleatorio do ruido (N.m, RMS) |\n");
        std::printf("|---|---:|---:|---:|---:|---:|---:|---:|\n");
        for (const Option& option : options) {
            for (const float speed : { 5.0f, 9.0f, 13.0f }) {
                const SwingResult clean = swing(body, option, speed, 0.0f, 1);
                const SwingResult noisy = swing(body, option, speed, 1.0f, 1);
                std::printf("| %s | %.0f | %.3f | %.1f | %.0f%% | %.3f | %.0f%% | %.0f |\n", option.name, speed,
                    clean.trackingRms, clean.peakSpeed, 100.0f * clean.saturated, noisy.trackingRms,
                    100.0f * noisy.saturated, noiseTorque(noisy, clean));
            }
        }
        std::printf("\n## 2. A ponta do pe (de pe na postura natural; aos 0,6 s ele sobe na ponta dos pes)\n\n");
        std::printf("| Solver | De pe por (s) | Parado: giro da ponta / do tornozelo (rad/s, RMS) | Parado: trocas de sentido da ponta / do tornozelo (/s) | Parado: ponta no chao | "
                    "Subindo: maior carga na ponta (pesos) | Angulo da ponta nessa hora (graus) | Calcanhar subiu (cm) | "
                    "Subindo: giro da ponta (rad/s, RMS) | Trocas de contato da ponta (/s) |\n");
        std::printf("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n");
        for (const auto& solver : { std::pair<std::uint32_t, std::uint32_t> { 10, 6 }, { 20, 20 } }) {
            const ToeResult r = toes(body, solver.first, solver.second);
            std::printf("| %u/%u | %.2f | %.2f / %.2f | %.0f / %.0f | %.0f%% | %.2f | %.1f | %.1f | %.2f | %.1f |\n", solver.first,
                solver.second, r.holdSeconds, r.quietToeSpeedRms, r.quietAnkleSpeedRms, r.quietToeReversals,
                r.quietAnkleReversals, 100.0f * r.quietToeContact,
                r.riseToeLoadMax, r.riseToeAngle, r.riseHeelLift, r.riseToeSpeedRms, r.riseToeFlips);
        }
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "MatterRLMotorProbe: %s\n", error.what());
        return 1;
    }
}
