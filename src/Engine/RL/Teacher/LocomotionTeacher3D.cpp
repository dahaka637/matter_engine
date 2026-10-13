#include "Engine/RL/Teacher/LocomotionTeacher3D.hpp"

#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>

namespace MatterEngine {
namespace {

constexpr float Pi = std::numbers::pi_v<float>;

TrainingRange3D readRange(const nlohmann::json& object, const char* key, TrainingRange3D fallback) {
    if (!object.contains(key)) return fallback;
    const nlohmann::json& values = object.at(key);
    if (!values.is_array() || values.size() != 2)
        throw std::runtime_error(std::string("professor de RL: faixa invalida em ") + key);
    TrainingRange3D range { values[0].get<float>(), values[1].get<float>() };
    if (!(range.minimum <= range.maximum))
        throw std::runtime_error(std::string("professor de RL: faixa invertida em ") + key);
    return range;
}

float uniform(std::mt19937_64& rng, TrainingRange3D range) {
    if (!(range.maximum > range.minimum)) return range.minimum;
    return std::uniform_real_distribution<float>(range.minimum, range.maximum)(rng);
}

} // namespace

LocomotionTeacherSettings3D loadLocomotionTeacherSettings3D(const std::filesystem::path& taskPath) {
    LocomotionTeacherSettings3D settings;
    std::ifstream stream(taskPath);
    if (!stream) throw std::runtime_error("professor de RL: tarefa nao encontrada: " + taskPath.string());
    const nlohmann::json document = nlohmann::json::parse(stream);
    if (!document.contains("teacher")) return settings;
    const nlohmann::json& t = document["teacher"];
    settings.standingAssistPercent = t.value("standingAssistPercent", settings.standingAssistPercent);
    settings.walkingAssistPercent = t.value("walkingAssistPercent", settings.walkingAssistPercent);
    settings.feetAssistPercent = t.value("feetAssistPercent", settings.feetAssistPercent);
    settings.segmentSeconds = readRange(t, "segmentSeconds", settings.segmentSeconds);
    settings.standingWeight = std::max(0.0f, t.value("standingWeight", settings.standingWeight));
    settings.turnWeight = std::max(0.0f, t.value("turnWeight", settings.turnWeight));
    settings.forwardWeight = std::max(0.0f, t.value("forwardWeight", settings.forwardWeight));
    settings.backwardWeight = std::max(0.0f, t.value("backwardWeight", settings.backwardWeight));
    settings.sideWeight = std::max(0.0f, t.value("sideWeight", settings.sideWeight));
    settings.diagonalWeight = std::max(0.0f, t.value("diagonalWeight", settings.diagonalWeight));
    settings.speedFraction = readRange(t, "speedFraction", settings.speedFraction);
    settings.turnRate = readRange(t, "turnRate", settings.turnRate);
    settings.curveFraction = std::clamp(t.value("curveFraction", settings.curveFraction), 0.0f, 1.0f);
    settings.curveRate = readRange(t, "curveRate", settings.curveRate);
    settings.pushes = t.value("pushes", settings.pushes);
    settings.pushInterval = readRange(t, "pushInterval", settings.pushInterval);
    settings.pushForce = readRange(t, "pushForce", settings.pushForce);
    settings.pushSeconds = readRange(t, "pushSeconds", settings.pushSeconds);
    return settings;
}

// O personagem do jogo, os clipes e os ajustes do runtime: um para todos os
// mundos (so leitura depois de montado; os clipes da locomocao apontam para a
// biblioteca, que nao muda).
struct LocomotionTeacher3D::Assets {
    RagdollCharacter3D character;
    std::vector<AnimationClip3D> library;
    CharacterLocomotionAnimations3D clips;
    CharacterLocomotionAnimations3D defaultClips;
    CharacterGaitSpeeds3D gaitSpeeds;
    CharacterMotorSettings3D avatar;
    AthleteRuntimeAssets3D runtime;
    AthleteRuntimeSettings3D settings;
};

// O estado do professor num mundo: o atleta, a capsula, o roteiro e os
// empurroes.
struct LocomotionTeacher3D::World {
    std::unique_ptr<AthleteState3D> athlete;
    AthleteRuntime3D runtime;
    std::mt19937_64 rng;
    // A capsula (como a sonda dos atletas): o quanto vai atras do corpo e o
    // quanto bateu numa parede.
    Vec3 followVelocity;
    Vec3 capsuleMoved;
    float capsuleRose = 0.0f;
    float wallImpact = 0.0f;
    // O roteiro: o trecho de agora, o olhar e as teclas (no referencial do
    // olhar: x frente, y esquerda, ate 1).
    float segmentLeft = 0.0f;
    float facingYaw = 0.0f;
    float facingRate = 0.0f;
    float keysForward = 0.0f;
    float keysLeft = 0.0f;
    // Empurroes.
    float time = 0.0f;
    float pushTimer = 0.0f;
    float pushLeft = 0.0f;
    Vec3 pushForce;
    // O rumo pedido no tick anterior e a taxa de giro dele, filtrada (o corte
    // da locomocao fisica v2).
    Vec3 requestDirection;
    bool requestValid = false;
    float requestTurnRate = 0.0f;
    // O que ele tenta (no mundo) e a fase no relogio do RL.
    Vec3 command;
    float phase = -1.0f;
    int lastSwing = -1;
};

LocomotionTeacher3D::LocomotionTeacher3D(
    const std::filesystem::path& assetsDirectory, const LocomotionTeacherSettings3D& settings)
    : m_settings(settings), m_assets(std::make_unique<Assets>()) {
    Assets& a = *m_assets;
    a.character = loadRagdollCharacter3D((assetsDirectory / "characters/football_player/character.json").string());
    a.library = loadAthleteClipLibrary3D((assetsDirectory / "animations/clips").string(), a.character.profile);
    a.clips = athleteLocomotionClips3D(a.character, a.library);
    a.defaultClips = defaultLocomotionAnimations3D(a.clips, a.character.defaultLocomotionClipIds, a.library);
    a.gaitSpeeds = characterGaitSpeeds3D(a.clips);
    a.avatar = athleteAvatarMotorSettings3D();
    if (a.gaitSpeeds.walk > 0.0f) {
        a.avatar.walkSpeed = a.gaitSpeeds.walk;
        a.avatar.sprintSpeed = a.gaitSpeeds.sprint;
    }
    a.runtime.profile = &a.character.profile;
    a.runtime.clips = &a.clips;
    a.runtime.defaultClips = &a.defaultClips;
    a.runtime.avatarSettings = a.avatar;
    // O modo fisico total, com as ajudas escolhidas.
    a.settings.defaultLocomotion = false;
    a.settings.e5StandingAssistPercent = settings.standingAssistPercent;
    a.settings.e5WalkingAssistPercent = settings.walkingAssistPercent;
    a.settings.physicalFeetAssistPercent = settings.feetAssistPercent;
}

LocomotionTeacher3D::~LocomotionTeacher3D() = default;

const LocomotionTeacherSettings3D& LocomotionTeacher3D::settings() const { return m_settings; }

void LocomotionTeacher3D::attach(const TrainingTeacherWorld3D& world) {
    const RagdollProfile3D& profile = m_assets->character.profile;
    const RagdollState3D state = world.scene->ragdollState(world.ragdoll);
    if (state.links.size() != profile.links.size())
        throw std::runtime_error("professor de RL: o corpo do treino nao e o do personagem do jogo");
    if (m_worlds.size() <= world.index) m_worlds.resize(world.index + 1);
    m_worlds[world.index] = std::make_unique<World>();
    world.scene->createCharacter({ world.origin.x, world.origin.y, 0.0f }, m_assets->avatar);
}

void LocomotionTeacher3D::beginEpisode(
    const TrainingTeacherWorld3D& world, const RagdollState3D& state, std::uint64_t seed) {
    World& w = *m_worlds.at(world.index);
    const RagdollProfile3D& profile = m_assets->character.profile;
    w.rng.seed(seed);
    w.athlete = std::make_unique<AthleteState3D>();
    LegacyAthleteState3D& legacy = w.athlete->legacy;
    legacy.guideFacingYawRadians = world.heading;
    legacy.locomotion.reset(profile, state);
    legacy.adaptivePhysics.reset(profile);
    // (Fisico total: o corpo comeca no fisico, sem guia.)
    legacy.bodyMode = LegacyBodyMode3D::Physical;
    legacy.guided = false;
    world.scene->setRagdollGuided(world.ragdoll, false);
    w.runtime = AthleteRuntime3D {};
    // A capsula embaixo da pelve, parada.
    const Vec3 pelvis = state.links.front().position;
    world.scene->placeCharacter({ pelvis.x, pelvis.y, 0.0f }, m_assets->avatar);
    world.scene->setCharacterVelocity({});
    w.followVelocity = {};
    w.capsuleMoved = {};
    w.capsuleRose = 0.0f;
    w.wallImpact = 0.0f;
    // Assenta parado um pouco antes do primeiro trecho do roteiro.
    w.segmentLeft = 0.6f;
    w.facingYaw = world.heading;
    w.facingRate = 0.0f;
    w.keysForward = w.keysLeft = 0.0f;
    w.time = 0.0f;
    w.pushTimer = 1.0f + uniform(w.rng, m_settings.pushInterval);
    w.pushLeft = 0.0f;
    w.pushForce = {};
    w.requestDirection = {};
    w.requestValid = false;
    w.requestTurnRate = 0.0f;
    w.command = {};
    w.phase = -1.0f;
    w.lastSwing = -1;
}

void LocomotionTeacher3D::control(const TrainingTeacherWorld3D& world, const RagdollState3D& state, float dt) {
    World& w = *m_worlds.at(world.index);
    const Assets& a = *m_assets;
    const LocomotionTeacherSettings3D& s = m_settings;
    PhysicsScene3D& scene = *world.scene;
    LegacyAthleteState3D& legacy = w.athlete->legacy;
    w.time += dt;

    // O roteiro: um trecho novo quando o de agora acaba.
    if ((w.segmentLeft -= dt) <= 0.0f) {
        w.segmentLeft = uniform(w.rng, s.segmentSeconds);
        w.facingRate = 0.0f;
        w.keysForward = w.keysLeft = 0.0f;
        const std::array<float, 6> weights { s.standingWeight, s.turnWeight, s.forwardWeight, s.backwardWeight,
            s.sideWeight, s.diagonalWeight };
        std::discrete_distribution<int> pick(weights.begin(), weights.end());
        const int kind = pick(w.rng);
        const float sign = std::uniform_int_distribution<int>(0, 1)(w.rng) == 0 ? -1.0f : 1.0f;
        const float fraction = uniform(w.rng, s.speedFraction);
        if (kind == 1) w.facingRate = sign * uniform(w.rng, s.turnRate);
        else if (kind == 2) w.keysForward = fraction;
        else if (kind == 3) w.keysForward = -fraction;
        else if (kind == 4) w.keysLeft = sign * fraction;
        else if (kind == 5) {
            const float back = std::uniform_int_distribution<int>(0, 2)(w.rng) == 0 ? -1.0f : 1.0f;
            w.keysForward = back * fraction * std::numbers::sqrt2_v<float> * 0.5f;
            w.keysLeft = sign * fraction * std::numbers::sqrt2_v<float> * 0.5f;
        }
        if (kind >= 2 && std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng) < s.curveFraction)
            w.facingRate = sign * uniform(w.rng, s.curveRate);
    }
    w.facingYaw = std::remainder(w.facingYaw + w.facingRate * dt, 2.0f * Pi);

    // As teclas e a capsula, como no laboratorio (a sonda dos atletas, modo
    // fisico total).
    const PhysicsCharacterState3D before = scene.characterState();
    const Vec3 forward { std::cos(w.facingYaw), std::sin(w.facingYaw), 0.0f };
    const Vec3 right { forward.y, -forward.x, 0.0f };
    const CharacterLocomotionState3D lastState = legacy.locomotion.telemetry().state;
    const bool down = lastState == CharacterLocomotionState3D::Fallen || lastState == CharacterLocomotionState3D::GettingUp
        || lastState == CharacterLocomotionState3D::Tackling;
    CharacterMotorCommand3D command;
    command.ignoreRagdolls = true;
    const Vec3 wanted = forward * w.keysForward - right * w.keysLeft;
    if (!down) command.moveDirection = wanted;
    CharacterIntent3D intent;
    intent.requestedFacingYawRadians = w.facingYaw;
    if (wanted.lengthSquared() > 1.0e-4f) {
        const Vec3 direction = wanted.normalized();
        const float travelRelative = std::atan2(direction.y, direction.x) - w.facingYaw;
        const float speed = characterGaitSpeed3D(a.gaitSpeeds, travelRelative, false);
        const float terrain = legacy.locomotion.telemetry().terrainSpeedLimit;
        if (speed > 0.0f && a.avatar.walkSpeed > 0.0f) command.speedScale = speed / a.avatar.walkSpeed;
        if (speed > 0.0f) command.speedScale *= std::clamp(terrain / speed, 0.2f, 1.0f);
        // A locomocao fisica v2, como no laboratorio: virando forte a
        // velocidade cai (o corte), e com pouca ajuda a capsula fica presa a
        // velocidade do corpo mais uma folga (a coleira; sem ela a capsula ia
        // metros a frente e o pe que pousa ia sempre para tras).
        const float rate = w.requestValid
            ? std::acos(std::clamp(dot(w.requestDirection, direction), -1.0f, 1.0f)) / std::max(dt, 1.0e-4f) : 0.0f;
        w.requestTurnRate += (rate - w.requestTurnRate) * (1.0f - std::exp(-dt / 0.25f));
        w.requestDirection = direction;
        w.requestValid = true;
        if (speed > 0.0f && before.grounded) {
            const float walkingAssist = legacy.adaptivePhysics.settings().walkingAssistScale;
            command.speedScale *= physicalNavigationTurnScale3D(before.velocity, direction, w.requestTurnRate, walkingAssist);
            command.speedScale = physicalNavigationSpeedScale3D(direction, command.speedScale, a.avatar.walkSpeed,
                legacy.locomotion.physicalState().centerOfMassVelocityWorld, physicalNavigationLead3D(walkingAssist));
        }
        intent.requestedDirectionWorld = direction;
        intent.requestedVelocityWorld = direction * std::min(speed * std::min(1.0f, wanted.length()), terrain);
    } else {
        w.requestValid = false;
        w.requestTurnRate = 0.0f;
    }
    CharacterMotorSettings3D active = adaptiveCharacterMotorSettings3D(a.avatar, before, command,
        physicalNavigationAcceleration3D(a.settings.e5WalkingAssistPercent * 0.01f));
    command.followVelocity = w.followVelocity;
    command.navigationProxy = true;
    scene.moveCharacter(command, active, dt);
    const PhysicsCharacterState3D& capsule = scene.characterState();
    {
        const Vec3 moved { (capsule.position.x - before.position.x) / dt, (capsule.position.y - before.position.y) / dt,
            0.0f };
        const float braking
            = std::max({ active.groundAcceleration, active.groundDeceleration, active.groundBrakeDeceleration }) * dt;
        const float rise = capsule.position.z - before.position.z;
        w.wallImpact = athleteCapsuleWallImpact3D(w.capsuleMoved, moved, braking, capsule.grounded, std::max(rise, w.capsuleRose));
        w.capsuleRose = rise;
        w.capsuleMoved = moved;
    }
    AthleteControlInput3D control;
    control.controlledEntityId = world.index + 1;
    control.intent = intent;
    control.facingYawRadians = w.facingYaw;
    control.lookPitchRadians = -0.2f;
    control.proxyVelocityWorld = { capsule.velocity.x, capsule.velocity.y, 0.0f };
    control.capsuleWallImpact = w.wallImpact;
    control.capsuleFollowVelocityWorld = w.followVelocity;
    AthleteSlot3D slot { world.index + 1, world.ragdoll, &state, true, false, w.athlete.get() };
    AthleteRuntimeOutput3D output;
    w.runtime.update(scene, a.runtime, std::span<AthleteSlot3D>(&slot, 1), {}, a.settings, control, dt, output);
    if (output.followVelocitySet) w.followVelocity = output.followVelocityWorld;

    // Empurroes no tronco (pelve, peito e alto do peito), depois de assentar.
    if (s.pushes) {
        if (w.pushLeft > 0.0f) {
            w.pushLeft -= dt;
            scene.applyRagdollLinkForce(world.ragdoll, 0, w.pushForce * 0.20f, {});
            scene.applyRagdollLinkForce(world.ragdoll, 2, w.pushForce * 0.32f, {});
            scene.applyRagdollLinkForce(world.ragdoll, 3, w.pushForce * 0.48f, {});
        } else if ((w.pushTimer -= dt) <= 0.0f) {
            const float angle = std::uniform_real_distribution<float>(-Pi, Pi)(w.rng);
            const float force = uniform(w.rng, s.pushForce);
            w.pushForce = { std::cos(angle) * force, std::sin(angle) * force, 0.0f };
            w.pushLeft = uniform(w.rng, s.pushSeconds);
            w.pushTimer = uniform(w.rng, s.pushInterval);
        }
    }

    // O que ele tenta: a capsula (o plano que o corpo segue, colado a ele com
    // pouca ajuda) e o giro do olhar.
    w.command = { capsule.velocity.x, capsule.velocity.y, w.facingRate };
    // A fase no relogio do RL pelo balanco de cada pe: o direito no ar de 0 a
    // 0,5 e o esquerdo de 0,5 a 1 (o mais recente, se os dois); no apoio
    // duplo, o fim do ultimo balanco.
    const CharacterLocomotionTelemetry3D& telemetry = legacy.locomotion.telemetry();
    const float left = telemetry.footSwingProgress[0], rightSwing = telemetry.footSwingProgress[1];
    if (telemetry.state == CharacterLocomotionState3D::Idle) {
        w.phase = -1.0f;
        w.lastSwing = -1;
    } else if (left >= 0.0f && (rightSwing < 0.0f || left <= rightSwing)) {
        w.phase = 0.5f + 0.5f * std::clamp(left, 0.0f, 0.999f);
        w.lastSwing = 0;
    } else if (rightSwing >= 0.0f) {
        w.phase = 0.5f * std::clamp(rightSwing, 0.0f, 0.999f);
        w.lastSwing = 1;
    } else {
        w.phase = w.lastSwing == 1 ? 0.5f : w.lastSwing == 0 ? 0.0f : -1.0f;
    }
}

std::string LocomotionTeacher3D::describe(std::size_t world) const {
    if (world >= m_worlds.size() || !m_worlds[world] || !m_worlds[world]->athlete) return "sem mundo";
    const World& w = *m_worlds[world];
    const LegacyAthleteState3D& legacy = w.athlete->legacy;
    const CharacterLocomotionTelemetry3D& t = legacy.locomotion.telemetry();
    char text[512];
    const Vec3 com = legacy.locomotion.physicalState().centerOfMassVelocityWorld;
    std::snprintf(text, sizeof(text),
        "locomocao %d modo %d teclas %.2f %.2f olhar %.2f fase %.2f capsula %.2f %.2f | cm %.2f %.2f | plantado %d %d "
        "balanco %.2f %.2f espera %.2f cambaleio %.2f",
        static_cast<int>(t.state), static_cast<int>(legacy.bodyMode), w.keysForward, w.keysLeft, w.facingYaw,
        w.phase, w.command.x, w.command.y, com.x, com.y, t.footPlanted[0] ? 1 : 0, t.footPlanted[1] ? 1 : 0,
        t.footSwingProgress[0], t.footSwingProgress[1], t.touchdownWaitSeconds, t.stagger);
    return text;
}

Vec3 LocomotionTeacher3D::command(std::size_t world) const {
    return world < m_worlds.size() && m_worlds[world] ? m_worlds[world]->command : Vec3 {};
}

float LocomotionTeacher3D::gaitPhase(std::size_t world) const {
    return world < m_worlds.size() && m_worlds[world] ? m_worlds[world]->phase : -1.0f;
}

} // namespace MatterEngine
