#include "Workbench/ReinforcementLearning/RlBodyBrains3D.hpp"

#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/Navigation/BodyNavigation3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace MatterEngine {
namespace {

float wrapAngle(float a) { return std::atan2(std::sin(a), std::cos(a)); }

const char* legacyModeName(LegacyBodyMode3D mode) {
    return mode == LegacyBodyMode3D::Animated ? "animado" : mode == LegacyBodyMode3D::Physical ? "FISICO"
        : mode == LegacyBodyMode3D::Returning ? "voltando" : "ativo";
}

float pelvisYaw(const RagdollProfile3D& profile, const PhysicsBodyState3D& pelvis) {
    const Vec3 forward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

std::unique_ptr<TrackingViewer3D> buildBrain(const std::filesystem::path& assets, const std::filesystem::path& run, PhysicsScene3D* scene,
    std::uint64_t seed) {
    auto brain = std::make_unique<TrackingViewer3D>(assets, seed, "lev_comando", std::filesystem::path {}, 1.0f, 0.2f,
        TrackingViewerConditions3D {}, run, scene);
    std::string label;
    brain->setPolicy(run / "exports" / "policy_latest.mrlpolicy", label);
    if (!brain->hasPolicy()) throw std::runtime_error("a rede exportada nao abriu: " + label);
    return brain;
}

} // namespace

RlBodyBrains3D::~RlBodyBrains3D() = default;

bool RlBodyBrains3D::drives(std::uint64_t entityId) const {
    const auto found = m_slots.find(entityId);
    return found != m_slots.end() && found->second.active;
}

void RlBodyBrains3D::clear() {
    m_slots.clear();
}

std::vector<std::uint64_t> RlBodyBrains3D::takeReturned() {
    std::vector<std::uint64_t> out;
    out.swap(m_returned);
    return out;
}

bool RlBodyBrains3D::adopt(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings) {
    TrackingViewer3D& brain = *slot.brain;
    const RagdollProfile3D& profile = brain.profile();
    const RagdollState3D was = scene.ragdollState(*body.handle);
    if (was.links.size() != profile.links.size()) {
        slot.failed = true;
        write("corpo " + std::to_string(body.entityId) + ": o corpo nao tem os elos do corpo do treino");
        return false;
    }
    // O corpo base sem a rigidez do painel ja e o corpo do treino: o mesmo ragdoll (as colisoes, a posse da bola e os contatos seguem com
    // ele). Outro corpo (so no Fisico com RL): o do treino no lugar dele, na mesma pose e velocidades.
    if (settings.recreateBody) {
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = body.entityId;
        spawn.active = true;
        spawn.pelvisPosition = was.links.front().position;
        scene.destroyRagdoll(*body.handle);
        *body.handle = scene.createRagdoll(profile, spawn);
        scene.resetRagdollState(*body.handle, was.links);
        *body.state = scene.ragdollState(*body.handle);
        if (body.previousState != nullptr) *body.previousState = *body.state;
        if (body.simulationPreviousState != nullptr) *body.simulationPreviousState = *body.state;
    }
    // O corpo exatamente como no treino: so os motores (sem a guia da animacao nem a restricao de pose do padrao), dinamico e ativo, o
    // atrito inteiro e a tensao dos limites do perfil - LIGADA no FootballPlayerV1, com ela o cerebro treinou inteiro. (16/11 01h50, ele: "a
    // fisica esta estranha... ele cai muito facil": a primeira versao a desligava; na sonda, sem ela, o empurrao de 400 N que ele segura
    // passou a derruba-lo.)
    scene.setRagdollGuided(*body.handle, false);
    scene.setRagdollKinematic(*body.handle, false);
    scene.setRagdollActive(*body.handle, true);
    scene.setRagdollAnimationConstraint(*body.handle, RagdollAnimationConstraint3D {});
    scene.setRagdollFrictionScale(*body.handle, 1.0f);
    scene.setRagdollLimitTension(*body.handle, profile.limitTension.enabled);
    body.athlete->legacy.guided = false;
    try {
        brain.adoptExternalBody(*body.handle);
    } catch (const std::exception& error) {
        slot.failed = true;
        write("corpo " + std::to_string(body.entityId) + ": " + error.what());
        return false;
    }
    slot.idleView = pelvisYaw(profile, body.state->links.front());
    slot.body = *body.handle;
    // (A navegacao fica no corpo enquanto o cerebro dirige - BodyNavigation3D.)
    anchorNavigation(scene, slot, body, settings);
    slot.tick = 0;
    slot.standingSeconds = slot.seconds = 0.0f;
    slot.fromGetUp = false;
    slot.active = true;
    slot.full = settings.full;
    const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, body.state->links);
    write(std::string("ASSUME ") + (body.controlled ? "o jogador" : "o boneco " + std::to_string(body.entityId))
        + (settings.full ? " (Fisico com RL)" : " (padrao -> fisico)") + " vel " + std::to_string(std::hypot(v.x, v.y)) + " | "
        + brain.externalState());
    return true;
}

// O cerebro larga o corpo. Saindo do Fisico com RL, o dono recomeca o corpo (events.releasedFromFull); no padrao que saiu do fisico
// sozinho (o T), nada.
void RlBodyBrains3D::release(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D* body, const RlBrainSettings3D& settings, const char* why,
    Events& events) {
    const bool wasFull = slot.full;
    slot.active = false;
    if ((slot.blending || slot.fading) && slot.brain) slot.brain->setExternalPoseBlend({}, 0.0f);
    slot.blending = false;
    slot.fading = false;
    write(std::string("PARA ") + (body != nullptr && body->controlled ? "o jogador" : "um corpo") + " (" + why + ")");
    if (body == nullptr || !scene.contains(*body->handle)) return;
    // (16/11, ele: "teleportando") O corpo recriado com o perfil do treino volta ao do jogo na mesma pose e nas mesmas velocidades -
    // antes o jogador era recriado de pe no lugar.
    if (wasFull && settings.recreateBody && settings.gameProfile != nullptr && body->state != nullptr && !body->state->links.empty()) {
        const RagdollState3D was = scene.ragdollState(*body->handle);
        if (was.links.size() == settings.gameProfile->links.size()) {
            RagdollSpawnDefinition3D spawn;
            spawn.entityId = body->entityId;
            spawn.active = true;
            spawn.rigidityPercent = settings.gameRigidityPercent;
            spawn.pelvisPosition = was.links.front().position;
            scene.destroyRagdoll(*body->handle);
            *body->handle = scene.createRagdoll(*settings.gameProfile, spawn);
            scene.resetRagdollState(*body->handle, was.links);
            *body->state = scene.ragdollState(*body->handle);
            if (body->previousState != nullptr) *body->previousState = *body->state;
            if (body->simulationPreviousState != nullptr) *body->simulationPreviousState = *body->state;
        }
    }
    scene.setRagdollLimitTension(*body->handle, settings.gameLimitTension);
    if (wasFull) {
        // (O atleta do zero no fisico, do corpo como esta: o padrao - ou o fisico de sempre - continua dele.)
        restartAthlete(scene, slot, *body, settings);
        events.releasedFromFull.push_back(body->entityId);
    }
}

void RlBodyBrains3D::anchorNavigation(PhysicsScene3D& scene, const Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings) const {
    if (settings.avatar == nullptr || body.state == nullptr || body.athlete == nullptr || body.state->links.empty() || !slot.brain) return;
    placeBodyNavigation3D(scene, *settings.avatar, body.controlled, body.athlete->legacy,
        bodyNavigationPoint3D(scene, slot.brain->profile(), *body.state));
}

void RlBodyBrains3D::restartAthlete(PhysicsScene3D& scene, const Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings) const {
    if (body.state == nullptr || body.athlete == nullptr || body.state->links.empty()) return;
    const RagdollProfile3D* profile = settings.gameProfile != nullptr ? settings.gameProfile : slot.brain ? &slot.brain->profile() : nullptr;
    if (profile == nullptr) return;
    AthleteState3D fresh;
    fresh.legacy.guideFacingYawRadians = body.commanded ? body.viewYaw : pelvisYaw(*profile, body.state->links.front());
    fresh.legacy.locomotion.reset(*profile, *body.state);
    fresh.legacy.adaptivePhysics.reset(*profile);
    *body.athlete = std::move(fresh);
    anchorNavigation(scene, slot, body, settings);
}

// (16/11, ele: "aumente... a % de guiagem dos ossos de 0 a 100 gradualmente, de uma forma harmonica com movimentos naturais, e ele so
// encaixe no modo locomocao default quando de fato a diferenca de pose for bem baixa") A volta harmonica: o padrao comeca a rodar por
// baixo - a sombra, semeada do corpo como na volta de sempre (a pelve no rumo dela, a velocidade, a fase pelos pes) -, os musculos vao
// a pose dele e so entao ele assume.
void RlBodyBrains3D::beginBlend(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings, float view) {
    const RagdollProfile3D& profile = *settings.gameProfile;
    const RagdollState3D& state = *body.state;
    const PhysicsBodyState3D& pelvis = state.links.front();
    // (A sombra olha para onde o corpo ja olha - os angulos das juntas nao giram a pelve inteira: virada para a camera, ela girava com
    // passos e o corpo ia atras, os pes a 20-30 cm. Depois do encaixe o padrao vira para a camera pelo giro dele.)
    (void)view;
    slot.shadowFacing = pelvisYaw(profile, pelvis);
    {
        const GroundProbeResult3D ground = scene.probeGround(pelvis.position + Vec3 { 0.0f, 0.0f, 0.3f }, 0.05f, 0.0f, 3.0f, 60.0f);
        slot.shadowRoot = { pelvis.position.x, pelvis.position.y, ground.hasSurface ? ground.pointWorld.z : 0.0f };
    }
    slot.shadow.reset(slot.shadowFacing);
    athleteConfigureDefaultLocomotion3D(slot.shadow.settings(), *settings.runtimeSettings);
    slot.shadow.settings().bodyScale = bodyScale3D(settings.body);
    DefaultLocomotionSeed3D seed;
    seed.pelvisYawRadians = pelvisYaw(profile, pelvis);
    seed.velocityWorld = { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f };
    seed.pelvisWorld = pelvis.position;
    seed.pelvisVelocityWorld = pelvis.linearVelocity;
    std::size_t feet[2] = { 0, 0 };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    seed.feetValid = feet[0] != 0 && feet[1] != 0 && feet[0] < state.links.size() && feet[1] < state.links.size();
    if (seed.feetValid)
        for (std::size_t side = 0; side < 2; ++side) {
            seed.feetWorld[side] = state.links[feet[side]].position;
            seed.feetVelocityWorld[side] = state.links[feet[side]].linearVelocity;
            seed.feetOrientationWorld[side] = state.links[feet[side]].orientation;
        }
    // (16/11) Os pes da sombra apoiados onde os do corpo estao: os musculos vao a postura dela sem arrastar um pe ate o lugar do
    // clipe (o encaixe saia com os pes a 13-44 cm e a volta os deslizava); depois do encaixe, o padrao os leva a pose parada por passos.
    seed.plantFeet = seed.feetValid;
    slot.shadow.seed(seed);
    slot.blending = true;
    slot.fading = false;
    slot.blendTime = 0.0f;
    slot.blendWeight = 0.0f;
    slot.yawGap = 0.0f;
    slot.jointError = slot.distance = 1.0e9f;
    write(std::string("MISTURA comeca ") + (body.controlled ? "no jogador" : "no boneco " + std::to_string(body.entityId)) + " | "
        + slot.brain->externalState());
}

bool RlBodyBrains3D::stepShadow(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings, float view,
    float deltaTime) {
    const RagdollState3D& state = *body.state;
    const PhysicsBodyState3D& pelvis = state.links.front();
    const GroundProbeResult3D ground = scene.probeGround(pelvis.position + Vec3 { 0.0f, 0.0f, 0.3f }, 0.05f, 0.0f, 3.0f, 60.0f);
    DefaultLocomotionInput3D input;
    const float floor = ground.hasSurface ? ground.pointWorld.z : 0.0f;
    const Vec3 v = rlBodyCenterOfMassVelocity3D(*settings.gameProfile, state.links);
    const bool still = (!body.commanded || body.requestedVelocity.lengthSquared() < 0.01f) && std::hypot(v.x, v.y) < 0.5f;
    // (A base do padrao sob a pelve, onde a capsula fica com o cerebro: no encaixe nada salta - posta sobre os pes, a raiz do padrao pulava
    // 30 cm entre o tick do encaixe e o seguinte e a mola lia o pulo como 10 m/s. Parado, sem velocidade: com o corpo parado e o rumo
    // dele, a sombra nao tem por que dar passos.)
    input.feetPositionWorld = still ? slot.shadowRoot : Vec3 { pelvis.position.x, pelvis.position.y, floor };
    if (still) input.velocityWorld = {};
    if (!still) input.velocityWorld = { v.x, v.y, 0.0f };
    input.grounded = true;
    input.requestedVelocityWorld = body.commanded ? body.requestedVelocity : Vec3 {};
    // (Parado, o rumo do corpo no comeco da mistura; andando, a vista - a passada vai para onde as teclas mandam.)
    input.facingYawRadians = still ? slot.shadowFacing : view;
    input.lookPitchRadians = body.lookPitch;
    PhysicsScene3D* where = &scene;
    input.groundAt = [where](Vec3 point) { return where->probeTerrain(point + Vec3 { 0.0f, 0.0f, 0.55f }, 0.03f, 0.0f, 1.5f, 60.0f); };
    input.deltaTime = deltaTime;
    // (Parado, ela conduz: o parado do padrao escolhe a base pela ultima passada - os pes do corpo -, e nao a do clipe, 17 cm mais
    // estreita; andando, segue por baixo.)
    input.leading = still;
    // (Os pes apoiados ficam onde o corpo os tem enquanto os musculos vao a postura - os do corpo a cada tick: o cerebro ainda pode
    // mexer um pe -; o passo de acomodacao e do padrao, depois do encaixe.)
    input.holdStance = true;
    if (still) {
        std::array<Vec3, 2> feetAt {};
        std::array<Quaternion, 2> feetTurn {};
        bool found[2] = { false, false };
        for (std::size_t i = 0; i < settings.gameProfile->links.size() && i < state.links.size(); ++i) {
            const std::string& id = settings.gameProfile->links[i].id;
            const int side = id == "LeftFoot" ? 0 : id == "RightFoot" ? 1 : -1;
            if (side < 0) continue;
            feetAt[side] = state.links[i].position;
            feetTurn[side] = state.links[i].orientation;
            found[side] = true;
        }
        if (found[0] && found[1]) slot.shadow.plantFeet(feetAt, feetTurn);
    }
    athleteConfigureDefaultLocomotion3D(slot.shadow.settings(), *settings.runtimeSettings);
    slot.shadow.settings().bodyScale = bodyScale3D(settings.body);
    slot.shadow.update(*settings.gameProfile, *settings.defaultClips, input);
    const DefaultLocomotionOutput3D& out = slot.shadow.output();
    if (!out.valid || out.coordinates.size() != state.links.size() || out.pose.linkPositions.size() != state.links.size()) return false;
    // As diferencas: as juntas (rms, dos elos alem da pelve) e o mais longe entre a pelve e os pes do corpo e os da sombra.
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 1; i < state.links.size() && i < state.joints.size(); ++i) {
        const auto& q = state.joints[i].positionRadians;
        const Vec3 d = Vec3 { q[0], q[1], q[2] } - out.coordinates[i];
        sum += static_cast<double>(d.lengthSquared());
        ++count;
    }
    slot.jointError = count > 0 ? static_cast<float>(std::sqrt(sum / static_cast<double>(count))) : 0.0f;
    float distance = (out.pose.linkPositions.front() - pelvis.position).length();
    slot.pelvisGap = out.pose.linkPositions.front() - pelvis.position;
    {
        const Vec3 forward = (out.pose.linkOrientations.front() * settings.gameProfile->links.front().modelOrientation.conjugate())
            .rotate({ 1.0f, 0.0f, 0.0f });
        slot.yawGap = wrapAngle(std::atan2(forward.y, forward.x) - pelvisYaw(*settings.gameProfile, pelvis));
    }
    slot.feetGap = 0.0f;
    for (std::size_t i = 0; i < settings.gameProfile->links.size() && i < state.links.size(); ++i) {
        const std::string& id = settings.gameProfile->links[i].id;
        if (id == "LeftFoot" || id == "RightFoot") {
            const float gap = (out.pose.linkPositions[i] - state.links[i].position).length();
            distance = std::max(distance, gap);
            slot.feetGap = std::max(slot.feetGap, gap);
        }
    }
    slot.distance = distance;
    return true;
}

void RlBodyBrains3D::update(PhysicsScene3D& scene, std::span<RlBrainBody3D> bodies, const RlBrainSettings3D& settings, float deltaTime,
    Events& events) {
    for (RlBrainBody3D& body : bodies) {
        auto found = m_slots.find(body.entityId);
        if (!body.usable || body.handle == nullptr || body.state == nullptr || body.athlete == nullptr || body.state->links.empty()
            || !scene.contains(*body.handle)) {
            if (found != m_slots.end() && found->second.active) release(scene, found->second, nullptr, settings, "corpo indisponivel", events);
            continue;
        }
        // (16/11, ele: "teleportando") Do Fisico com RL para o Padrao, com o fisico do padrao pelo cerebro: ele segue no corpo como no
        // fisico do padrao e o devolve pela volta de sempre quando assentar - o atleta recomeca no fisico, do corpo como esta. (Antes o
        // jogador era recriado de pe no lugar.)
        if (found != m_slots.end() && found->second.active && found->second.full && !settings.full && settings.transitions && body.inDefault) {
            Slot& switching = found->second;
            switching.full = false;
            switching.seconds = switching.standingSeconds = 0.0f;
            restartAthlete(scene, switching, body, settings);
            write(std::string("SEGUE ") + (body.controlled ? "o jogador" : "o boneco " + std::to_string(body.entityId))
                + " no padrao (fisico pelo cerebro ate assentar)");
        }
        const LegacyBodyMode3D mode = body.athlete->legacy.bodyMode;
        const bool legacyPhysical = mode == LegacyBodyMode3D::Physical;
        const bool want = settings.full || (settings.transitions && (!body.inDefault || legacyPhysical));
        if (found == m_slots.end()) {
            if (!want) continue;
            found = m_slots.emplace(body.entityId, Slot {}).first;
        }
        Slot& slot = found->second;
        if (mode != slot.legacyMode) {
            slot.legacyMode = mode;
            if (body.controlled && body.inDefault)
                write(std::string("padrao: modo do corpo ") + legacyModeName(mode)
                    + (legacyPhysical ? " (" + body.athlete->legacy.physicalReason + ")" : std::string()));
        }
        if (!want || slot.failed) {
            if (slot.active) release(scene, slot, &body, settings, slot.full ? "modo trocado" : "o padrao saiu do fisico", events);
            continue;
        }
        if (!slot.brain) {
            if (!settings.loadAsync) {
                try {
                    slot.brain = buildBrain(settings.assetsDirectory, settings.runDirectory, &scene, 61 + body.entityId);
                } catch (const std::exception& error) {
                    slot.failed = true;
                    m_label = std::string("cérebro único não carregou: ") + error.what();
                    write(m_label);
                    continue;
                }
            } else {
                // (Em segundo plano: os clipes e a rede levam ~0,3 s; ate la o corpo segue no que tinha.)
                if (!slot.loading.valid()) {
                    const std::filesystem::path assets = settings.assetsDirectory, run = settings.runDirectory;
                    PhysicsScene3D* where = &scene;
                    const std::uint64_t seed = 61 + body.entityId;
                    slot.loading = std::async(std::launch::async, [assets, run, where, seed]() { return buildBrain(assets, run, where, seed); });
                }
                if (slot.loading.wait_for(std::chrono::seconds(0)) != std::future_status::ready) continue;
                try {
                    slot.brain = slot.loading.get();
                } catch (const std::exception& error) {
                    slot.failed = true;
                    m_label = std::string("cérebro único não carregou: ") + error.what();
                    write(m_label);
                    continue;
                }
            }
            std::string label;
            slot.brain->setPolicy(settings.runDirectory / "exports" / "policy_latest.mrlpolicy", label);
            m_label = "cérebro único: " + label;
        }
        if (slot.active && slot.full != settings.full) {
            // (Do Fisico com RL para o Padrao, ou o contrario: recomeca do corpo como esta, no modo novo.)
            const bool wasFull = slot.full;
            release(scene, slot, &body, settings, "modo trocado", events);
            if (wasFull) continue;
        }
        if (slot.active && (body.handle->index != slot.body.index || body.handle->generation != slot.body.generation)) {
            slot.active = false;
            // (O corpo foi recriado - o T -: no Padrao, o padrao recomecou e decide de novo.)
            if (!settings.full && body.inDefault) continue;
        }
        TrackingViewer3D& brain = *slot.brain;
        // A entrada, antes de assumir (assumindo correndo sem ela, ele recebia "parado" a 4 m/s): o rumo contra a vista em 8 (0 frente, 2
        // esquerda, 4 tras, 6 direita), o shift e a vista. Sem entrada, parado olhando para onde estava.
        const float view = body.commanded ? body.viewYaw : slot.idleView;
        int key = -1;
        if (body.commanded && std::hypot(body.direction.x, body.direction.y) > 0.1f) {
            const float rel = wrapAngle(std::atan2(body.direction.y, body.direction.x) - view);
            key = ((static_cast<int>(std::lround(rel / (0.25f * 3.14159265f))) % 8) + 8) % 8;
        }
        const bool sprint = body.commanded && body.sprint;
        // (16/11, ele: "debrucado em alguma estrutura") O passo para longe do que ele encosta: a tecla no rumo de la, no pulso.
        if (key < 0 && slot.stepAwayLeft > 0.0f && slot.awayFrom.lengthSquared() > 0.01f) {
            const float rel = wrapAngle(std::atan2(slot.awayFrom.y, slot.awayFrom.x) - view);
            key = ((static_cast<int>(std::lround(rel / (0.25f * 3.14159265f))) % 8) + 8) % 8;
        }
        // (16/11, ele: "virado para outra direcao") Na mistura, o cerebro segura o rumo da pelve do comeco (o da pelve da sombra): seguindo
        // a camera, ele girava o corpo 20 graus durante a mistura e o padrao, no rumo do comeco, o desvirava no encaixe. (O cerebro vira a
        // pelve para a vista - sem a abertura de ~22 graus do parado do padrao: com o rumo do padrao, ele girava os mesmos 20 graus.) O giro
        // ate a camera e do padrao, depois, pela mola do rumo.
        float brainView = view;
        if (!settings.full && body.inDefault && key < 0 && slot.uprightStill && slot.active) {
            if (!slot.holdingView) {
                slot.heldView = pelvisYaw(brain.profile(), body.state->links.front());
                slot.holdingView = true;
            }
            brainView = slot.heldView;
        } else if (key >= 0 || !slot.uprightStill) {
            slot.holdingView = false;
        }
        if (slot.blending) brainView = slot.shadowFacing;
        if (slot.tick == 0 || !slot.active) {
            brain.setPlayerInput(key, sprint, wrapAngle(brainView));
            // (As reguas do painel AJUDAS - parado, andando, ao levantar - na mao do treino do cerebro; 0% e nenhuma forca de fora.)
            brain.setExternalAssist(settings.assistStanding, settings.assistWalking, settings.assistGetUp);
        }
        if (!slot.active && !adopt(scene, slot, body, settings)) continue;
        slot.seconds += deltaTime;
        const bool canShadow = settings.defaultClips != nullptr && settings.runtimeSettings != nullptr && settings.gameProfile != nullptr;
        bool calm = false, stillNow = false, keysNow = false, quickRiseNow = false;
        if (!settings.full && body.inDefault) {
            // (16/11, ele: "nao consegue voltar a locomocao default"; o registro: com 0,3 s "de pe" o padrao pegava o corpo cambaleando e o
            // vai e volta comecava; com a calma de parado ele nunca devolvia correndo.) Assentado: de pe na locomocao do cerebro (nem caindo
            // nem levantando), pelve e peito em pe, a cabeca alta, nada encostando alem do chao - parado, andando ou correndo.
            const RagdollProfile3D& profile = brain.profile();
            const RagdollState3D& state = *body.state;
            const auto upOf = [&](std::size_t k) {
                return (state.links[k].orientation * profile.links[k].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f }).z;
            };
            std::size_t chest = 0, head = 0;
            for (std::size_t k = 0; k < profile.links.size() && k < state.links.size(); ++k) {
                if (profile.links[k].id == "UpperChest") chest = k;
                if (profile.links[k].id == "Head") head = k;
            }
            bool touched = false;
            for (const RagdollInteraction3D& contact : state.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Static || contact.normalWorld.z < 0.6f) touched = true;
            const GroundProbeResult3D ground = scene.probeGround(state.links.front().position + Vec3 { 0.0f, 0.0f, 0.3f }, 0.05f, 0.0f, 3.0f,
                60.0f);
            const float floor = ground.hasSurface ? ground.pointWorld.z : 0.0f;
            // (Sem tecla, assentado e parado de verdade: o corpo ainda deslizando a ~0,7 m/s na volta - o padrao semeado com isso parava
            // dando passos, e o corpo ia atras.)
            const Vec3 com = rlBodyCenterOfMassVelocity3D(profile, state.links);
            const bool keys = body.commanded && std::hypot(body.direction.x, body.direction.y) > 0.1f;
            // (19h55, settings.quickGetUpReturn) Depois de um levantar, sem tecla: de pe ja basta - ver o .hpp.
            // (O levantar de verdade: o clipe do levantar com a cabeca no chao, abaixo de 0,9 m - no primeiro tick da posse o cerebro ainda
            // nao esta 'de pe', e encostado numa caixa ou parede o levantar pisca sem ele cair; ali, a volta de sempre, com a mistura.)
            if (brain.externalGettingUp() && state.links[head].position.z - floor < 0.9f) slot.fromGetUp = true;
            const bool quickRise = settings.quickGetUpReturn && slot.fromGetUp && !keys;
            const float stillSpeed = quickRise ? settings.getUpReturnSpeed : 0.3f;
            // (Na mistura, so a perda de equilibrio ou um contato a interrompe: a pelve subindo para a pose do padrao nao e "sair do
            // parado".)
            // (E sem tecla, com o giro do cerebro para a vista ja terminado: a mistura comecando no meio do giro, ele seguia girando.)
            const bool turning = !keys && !slot.blending && !quickRise && std::abs(state.links.front().angularVelocity.z) > 0.35f;
            slot.turning = turning;
            slot.uprightStill = brain.externalStanding() && upOf(0) > 0.85f && upOf(chest) > 0.8f
                && state.links[head].position.z - floor > 1.35f && !keys;
            slot.calmMissing.clear();
            if (!brain.externalStanding()) slot.calmMissing += " nao-de-pe";
            if (upOf(0) <= (keys ? 0.7f : 0.85f)) slot.calmMissing += " pelve-torta";
            if (upOf(chest) <= (keys ? 0.6f : 0.8f)) slot.calmMissing += " peito-torto";
            if (state.links[head].position.z - floor <= (keys ? 1.2f : 1.35f)) slot.calmMissing += " cabeca-baixa";
            if (touched) {
                slot.calmMissing += " encostado(";
                for (const RagdollInteraction3D& contact : state.interactions)
                    if (contact.otherMotion != RagdollContactMotion3D::Static || contact.normalWorld.z < 0.6f)
                        slot.calmMissing += (contact.linkIndex < profile.links.size() ? profile.links[contact.linkIndex].id : std::string("?"))
                            + (contact.otherMotion == RagdollContactMotion3D::Static ? "/parede " : "/corpo ");
                slot.calmMissing += ")";
            }
            if (!(keys || slot.blending || std::hypot(com.x, com.y) < stillSpeed)) slot.calmMissing += " andando";
            if (turning) slot.calmMissing += " girando";
            // (16/11, ele: "fica lento quando levanta... nao consigo usar o levantar correndo") Com tecla, a postura de quem corre: o
            // peito inclinado ate ~53 graus e a cabeca acima de 1,2 m - exigindo o tronco em pe, ele so devolvia depois de desacelerar ate
            // endireitar (o levantar correndo saia a 2,5 m/s e o padrao recomecava a arrancada dali).
            const float pelvisUpMin = keys ? 0.7f : 0.85f, chestUpMin = keys ? 0.6f : 0.8f, headMin = keys ? 1.2f : 1.35f;
            // (Com tecla, e indo para onde ela pede: o corpo virado ate ~50 graus da vista e, andando, o centro de massa ate ~40 graus do rumo
            // pedido - devolvendo logo depois de uma pancada, de lado e a 91 graus da camera, o padrao assumia num tranco.)
            bool heading = true;
            if (keys) {
                const float facing = std::abs(wrapAngle(pelvisYaw(profile, state.links.front()) - view));
                const float speed = std::hypot(com.x, com.y);
                const float along = speed > 1e-3f ? (com.x * body.direction.x + com.y * body.direction.y)
                        / (speed * std::max(1e-3f, std::hypot(body.direction.x, body.direction.y))) : 1.0f;
                heading = facing < 0.87f && (speed < 0.6f || along > 0.75f);
            }
            calm = brain.externalStanding() && upOf(0) > pelvisUpMin && upOf(chest) > chestUpMin && state.links[head].position.z - floor > headMin
                && !touched && heading && (keys || slot.blending || std::hypot(com.x, com.y) < stillSpeed) && !slot.fading;
            if (keys && !heading) slot.calmMissing += " fora-do-rumo";
            // De pe (o tronco em pe) e encostado - parede, caixa, outro corpo - sem tecla: o passo para longe (a soma das normais no plano,
            // que apontam do que encosta para ele), num pulso de 0,3 s a cada 0,9 s enquanto encostar. Sem enxergar o ambiente (o treino
            // nunca teve obstaculo), o cerebro ficava parado encostado - e o padrao nao volta com o corpo encostado.
            {
                Vec3 away;
                for (const RagdollInteraction3D& contact : state.interactions)
                    if (contact.otherMotion != RagdollContactMotion3D::Static || contact.normalWorld.z < 0.6f)
                        away += Vec3 { contact.normalWorld.x, contact.normalWorld.y, 0.0f };
                const bool upright = brain.externalStanding() && upOf(0) > 0.8f && upOf(chest) > 0.7f;
                const bool leaning = touched && upright && !keys && away.lengthSquared() > 0.04f;
                slot.leaningSeconds = leaning ? slot.leaningSeconds + deltaTime : 0.0f;
                slot.stepAwayLeft = std::max(0.0f, slot.stepAwayLeft - deltaTime);
                slot.stepAwayRest = std::max(0.0f, slot.stepAwayRest - deltaTime);
                if (leaning && slot.leaningSeconds > 0.25f && slot.stepAwayRest <= 0.0f && !slot.blending) {
                    slot.awayFrom = away.normalized();
                    slot.stepAwayLeft = 0.3f;
                    slot.stepAwayRest = 0.9f;
                    write(std::string("AFASTA ") + (body.controlled ? "o jogador" : "o boneco " + std::to_string(body.entityId))
                        + " do que encosta" + slot.calmMissing);
                }
            }
            keysNow = keys;
            quickRiseNow = quickRise;
            stillNow = !keys && std::hypot(com.x, com.y) < 0.3f;
            slot.comSpeed = std::hypot(com.x, com.y);
        }
        // (A volta harmonica, a cada tick: a sombra do padrao anda, os musculos vao a pose dela pela subida suave e a ajuda auxiliar sobe
        // junto - enquanto o cerebro cede, ela segura a pelve na altura e no rumo.)
        if (slot.blending) {
            slot.blendTime += deltaTime;
            if (stepShadow(scene, slot, body, settings, view, deltaTime)) {
                const float x = std::clamp(slot.blendTime / std::max(0.05f, settings.blendSeconds), 0.0f, 1.0f);
                const float weight = x * x * (3.0f - 2.0f * x);
                slot.blendWeight = weight;
                brain.setExternalPoseBlend(slot.shadow.output().coordinates, weight);
                brain.setExternalAssist(std::max(settings.assistStanding, settings.blendAssist * weight),
                    std::max(settings.assistWalking, settings.blendAssist * weight), settings.assistGetUp);
            }
        } else if (slot.fading) {
            // (A mistura desfeita sai em rampa: o peso cai ate zero em fadeSeconds, nos ultimos alvos da sombra.)
            slot.fadeTime += deltaTime;
            const float x = std::clamp(slot.fadeTime / std::max(0.05f, slot.fadeSeconds), 0.0f, 1.0f);
            const float weight = slot.fadeFrom * (1.0f - x * x * (3.0f - 2.0f * x));
            if (x >= 1.0f || slot.fadeCoordinates.empty()) {
                slot.fading = false;
                brain.setExternalPoseBlend({}, 0.0f);
                brain.setExternalAssist(settings.assistStanding, settings.assistWalking, settings.assistGetUp);
            } else {
                brain.setExternalPoseBlend(slot.fadeCoordinates, weight);
                brain.setExternalAssist(std::max(settings.assistStanding, settings.blendAssist * weight),
                    std::max(settings.assistWalking, settings.blendAssist * weight), settings.assistGetUp);
            }
        }
        const auto fadeBlend = [&](float seconds) {
            slot.blending = false;
            slot.fading = slot.blendWeight > 0.01f && slot.shadow.output().valid;
            slot.fadeFrom = slot.blendWeight;
            slot.fadeTime = 0.0f;
            slot.fadeSeconds = seconds;
            if (slot.fading) slot.fadeCoordinates = slot.shadow.output().coordinates;
            else brain.setExternalPoseBlend({}, 0.0f);
            slot.blendWeight = 0.0f;
        };
        if (slot.tick == 0) {
            const float decision = deltaTime * TrackingGetUpAmpTraining3D::TicksPerDecision;
            if (!settings.full && body.inDefault) {
                slot.standingSeconds = calm ? slot.standingSeconds + decision : 0.0f;
                bool handBack = false;
                const float pelvisGap = std::hypot(slot.pelvisGap.x, slot.pelvisGap.y);
                if (slot.blending && !calm) {
                    // (Uma pancada, um tropeco no meio: a mistura sai - numa rampa curta -, o cerebro inteiro de novo.)
                    fadeBlend(0.15f);
                    write("MISTURA abortada (o corpo saiu do assentado) | " + brain.externalState());
                } else if (slot.blending && !keysNow && slot.blendTime > 0.2f
                    && (pelvisGap > 0.15f || slot.feetGap > 0.12f || std::abs(slot.yawGap) > 0.35f || slot.comSpeed > 0.6f)) {
                    // (O corpo saindo da sombra - derivando, um pe saindo do lugar, girando: a mistura sai em rampa antes de o tirar do
                    // equilibrio; com o corpo embaixo de outro, ela seguia ate o limite e ele caia na saida.)
                    char numbers[160];
                    std::snprintf(numbers, sizeof numbers, "pelve %.3f m, pes %.3f m, rumo %.0f graus, %.2f m/s", pelvisGap, slot.feetGap,
                        slot.yawGap * 57.2958f, slot.comSpeed);
                    fadeBlend(0.3f);
                    slot.standingSeconds = 0.0f;
                    write(std::string("MISTURA desfeita (o corpo saiu da sombra: ") + numbers + ") | " + brain.externalState());
                } else if (slot.blending && keysNow) {
                    // (O jogador quer andar no meio da mistura: a volta direta na hora - a resposta primeiro.)
                    handBack = true;
                    slot.blending = false;
                    brain.setExternalPoseBlend({}, 0.0f);
                } else if (slot.blending) {
                    // (O encaixe: com os musculos inteiros na pose do padrao, as juntas perto e os pes no lugar; ou no limite do tempo.
                    // A pelve fica alguns cm da da sombra pela folga dos musculos - o encaixe a leva junto, acima. Com o limite de 5 cm
                    // nela, a mistura so encaixava no fim do tempo, 1,5 s.)
                    handBack = slot.blendTime >= settings.blendSeconds && slot.jointError < settings.commitJointError
                        && slot.feetGap < settings.commitDistance && pelvisGap < 0.15f && std::abs(slot.yawGap) < 0.14f;
                    // (No limite do tempo: perto o bastante, encaixa; longe - empurrado no meio, os pes saindo do lugar -, a mistura sai e o
                    // cerebro segue: encaixar com a pelve a 40 cm da sombra arrastava o corpo ate ela.)
                    if (!handBack && slot.blendTime >= settings.blendMaxSeconds) {
                        if (slot.jointError < 2.0f * settings.commitJointError && slot.feetGap < 2.0f * settings.commitDistance
                            && pelvisGap < 0.2f && std::abs(slot.yawGap) < 0.25f) {
                            handBack = true;
                        } else {
                            fadeBlend(0.4f);
                            slot.standingSeconds = 0.0f;
                            char numbers[160];
                            std::snprintf(numbers, sizeof numbers, "juntas %.3f rad, pelve %.3f m, pes %.3f m, rumo %.0f graus", slot.jointError,
                                pelvisGap, slot.feetGap, slot.yawGap * 57.2958f);
                            write(std::string("MISTURA desfeita no limite (") + numbers + ") | " + brain.externalState());
                        }
                    }
                } else if (slot.standingSeconds >= (quickRiseNow ? settings.getUpReturnCalmSeconds : keysNow ? 0.25f : 0.4f) && slot.seconds >= 0.6f
                    && legacyPhysical
                    && (keysNow || !slot.turning || slot.standingSeconds >= 1.2f)) {
                    // (16/11, ele: "virado para outra direcao") Parado, o cerebro ja segura o rumo (heldView) e a mistura espera o embalo do
                    // giro dele acabar (comecando girando, a pelve saia 20 graus da sombra e a mistura era desfeita); o giro ate a camera e do
                    // padrao depois da volta, pela mola do rumo (LegacyAthleteState3D::rlFacingYaw).
                    // (Parado, a mistura harmonica - os musculos a pose do padrao parado. Andando, a volta direta pela mola lenta: a passada
                    // da sombra e a do corpo saem de fase - o padrao corre em outra cadencia - e misturar duas passadas desencontradas
                    // derrubava o equilibrio; a mistura abortava e ele nunca voltava.)
                    // (19h55: depois de um levantar, a volta direta na hora - ver quickGetUpReturn.)
                    if (canShadow && stillNow && (!quickRiseNow || settings.getUpReturnBlend)) beginBlend(scene, slot, body, settings, view);
                    else handBack = true;
                }
                if (handBack && legacyPhysical) {
                    char numbers[200];
                    std::snprintf(numbers, sizeof numbers, " (mistura %.2f s, juntas %.3f rad, pelve (%.3f %.3f %.3f) m, pes %.3f m)",
                        slot.blendTime, slot.jointError, slot.pelvisGap.x, slot.pelvisGap.y, slot.pelvisGap.z, slot.feetGap);
                    char facing[64];
                    std::snprintf(facing, sizeof facing, " rumo do corpo - vista %.0f graus",
                        wrapAngle(pelvisYaw(*settings.gameProfile, body.state->links.front()) - view) * 57.2958f);
                    write(std::string("DEVOLVE ") + (body.controlled ? "o jogador" : "o boneco " + std::to_string(body.entityId))
                        + " ao padrao depois de " + std::to_string(slot.seconds) + " s" + (slot.blending ? numbers : " (direta)") + facing
                        + " | " + brain.externalState());
                    // (Os detectores do padrao - os passos, a base, o corpo ativo, o observador - ficaram parados enquanto o cerebro
                    // dirigia: com os pes de segundos atras ele ia ao fisico de novo no tick seguinte. Do zero, do corpo de agora; o padrao
                    // em si - o modo, a locomocao - fica, e a volta dele sai dali. A ajuda do impacto tambem: parada no tombo, a ponte da
                    // volta abortava por "ajuda caiu" - o choque ja foi gasto pelo cerebro, como na rede do levantar depois do chao, 02/11.)
                    const RagdollState3D& state = *body.state;
                    AthleteState3D fresh;
                    fresh.legacy = std::move(body.athlete->legacy);
                    if (settings.gameProfile != nullptr) {
                        fresh.legacy.locomotion.reset(*settings.gameProfile, state);
                        fresh.legacy.adaptivePhysics.reset(*settings.gameProfile);
                    }
                    fresh.legacy.impactAssist.reset();
                    fresh.legacy.rlReturn = true;
                    fresh.legacy.rlReturnHalfLife = slot.blending ? settings.blendCommitHalfLife : settings.directHalfLife;
                    fresh.legacy.rlReturnSeconds = slot.blending ? settings.blendCommitSeconds : settings.directSeconds;
                    // (O padrao continua da sombra, na fase em que o corpo ja esta.)
                    if (slot.blending && slot.shadow.output().valid) {
                        fresh.legacy.defaultLocomotion = slot.shadow;
                        fresh.legacy.rlShadowAdopted = true;
                    }
                    *body.athlete = std::move(fresh);
                    // (E a navegacao: a raiz do padrao comeca onde ele esta - a capsula do jogador e a virtual do boneco. Da mistura, a
                    // raiz da sombra levada o que falta a pelve dela para a do corpo: a pelve do padrao nasce na do corpo e os pes ficam
                    // apoiados onde estao - nada desliza; o resto da base, o padrao acerta pelo passo de acomodacao.)
                    if (slot.blending && settings.avatar != nullptr)
                        placeBodyNavigation3D(scene, *settings.avatar, body.controlled, body.athlete->legacy,
                            BodyNavigationPoint3D { slot.shadowRoot - Vec3 { slot.pelvisGap.x, slot.pelvisGap.y, 0.0f }, Vec3 {} });
                    else
                        anchorNavigation(scene, slot, body, settings);
                    slot.blending = false;
                    slot.fading = false;
                    brain.setExternalPoseBlend({}, 0.0f);
                    slot.active = false;
                    scene.setRagdollLimitTension(*body.handle, settings.gameLimitTension);
                    m_returned.push_back(body.entityId);
                    continue;
                }
            }
            brain.step();
            if (body.controlled && (slot.logSeconds += decision) >= 0.25f) {
                slot.logSeconds = 0.0f;
                const Vec3 v = rlBodyCenterOfMassVelocity3D(brain.profile(), body.state->links);
                float headZ = 0.0f;
                for (std::size_t k = 0; k < brain.profile().links.size() && k < body.state->links.size(); ++k)
                    if (brain.profile().links[k].id == "Head") headZ = body.state->links[k].position.z;
                char line[200];
                std::snprintf(line, sizeof line, "  tecla %d shift %d | vel %.2f cabeca %.2f | contatos %zu | calmo %.2f s%s | ", key, sprint ? 1 : 0,
                    std::hypot(v.x, v.y), headZ, body.state->interactions.size(), slot.standingSeconds,
                    slot.blending ? " | misturando" : "");
                write(line + brain.externalState() + (slot.calmMissing.empty() ? std::string() : " | falta:" + slot.calmMissing));
            }
        }
        brain.applyExternalDrives();
    }
    // (O cerebro de um corpo que saiu de cena vai junto.)
    for (auto it = m_slots.begin(); it != m_slots.end();) {
        const bool alive = std::any_of(bodies.begin(), bodies.end(), [&](const RlBrainBody3D& body) { return body.entityId == it->first; });
        it = alive ? std::next(it) : m_slots.erase(it);
    }
}

void RlBodyBrains3D::finish(PhysicsScene3D& scene, std::span<RlBrainBody3D> bodies, const RlBrainSettings3D& settings) {
    for (RlBrainBody3D& body : bodies) {
        const auto found = m_slots.find(body.entityId);
        if (found == m_slots.end() || !found->second.active || !found->second.brain) continue;
        Slot& slot = found->second;
        if (++slot.tick >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
            slot.tick = 0;
            slot.brain->finishExternalDecision();
        }
        // (A navegacao no corpo a cada tick - a capsula do jogador posta, nunca movida: nao empurra a bola nem os props.)
        anchorNavigation(scene, slot, body, settings);
    }
}

} // namespace MatterEngine
