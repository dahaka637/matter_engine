#include "Engine/Character/BallTouchPreview3D.hpp"

#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <stdexcept>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Dt = 1.0f / 120.0f;

struct GaitSpec {
    const char* name;
    const char* key;
    // Movimento em relacao ao rumo do corpo (graus, + esquerda) e sprint.
    float travelDegrees;
    bool sprint;
    bool moving;
    // Ate a marcha firmar (s, contados do inicio do movimento).
    float warmupSeconds;
};

constexpr std::array<GaitSpec, static_cast<std::size_t>(BallPreviewGait3D::Count)> Gaits {{
    { "Parado", "parado", 0.0f, false, false, 1.2f },
    { "Trote", "trote", 0.0f, false, true, 2.0f },
    { "Sprint", "sprint", 0.0f, true, true, 2.6f },
    { "Lateral esq.", "lado-esq", 90.0f, false, true, 2.0f },
    { "Lateral dir.", "lado-dir", -90.0f, false, true, 2.0f },
    { "Recuo", "recuo", 180.0f, false, true, 2.0f },
    { "Sprint lat. esq.", "sprint-esq", 90.0f, true, true, 2.4f },
    { "Sprint lat. dir.", "sprint-dir", -90.0f, true, true, 2.4f },
}};

const GaitSpec& spec(BallPreviewGait3D gait) {
    return Gaits[std::min(static_cast<std::size_t>(gait), Gaits.size() - 1)];
}

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

PhysicsBodyHandle3D staticFloor(PhysicsScene3D& scene) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { 200.0f, 200.0f, 0.1f };
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = { 0.0f, 0.0f, -0.1f };
    body.materialId = "concrete";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

// O mesmo mapeamento do laboratorio (characterLocomotionAnimations).
CharacterLocomotionAnimations3D mapAnimations(const RagdollCharacter3D& c,
    const std::vector<AnimationClip3D>& clips) {
    CharacterLocomotionAnimations3D a;
    for (const auto& clip : clips) {
        if (clip.id == c.idleClipId) a.idle = &clip;
        if (clip.id == c.walkClipId) a.walk = &clip;
        if (clip.id == c.idleToSprintClipId) a.idleToSprint = &clip;
        if (clip.id == c.crouchToSprintClipId) a.crouchToSprint = &clip;
        if (clip.id == c.runForwardArcLeftClipId) a.runForwardArcLeft = &clip;
        if (clip.id == c.runForwardArcRightClipId) a.runForwardArcRight = &clip;
        if (clip.id == c.runBackwardArcRightClipId) a.runBackwardArcRight = &clip;
        if (clip.id == c.walkBackwardClipId) a.walkBackward = &clip;
        if (clip.id == c.sprintClipId) a.sprint = &clip;
        if (clip.id == c.sprintBackwardClipId) a.sprintBackward = &clip;
        if (clip.id == c.strafeLeftClipId) a.strafeLeft = &clip;
        if (clip.id == c.strafeRightClipId) a.strafeRight = &clip;
        if (clip.id == c.sprintStrafeLeftClipId) a.sprintStrafeLeft = &clip;
        if (clip.id == c.sprintStrafeRightClipId) a.sprintStrafeRight = &clip;
        if (clip.id == c.jumpStandingClipId) a.jumpStanding = &clip;
        if (clip.id == c.jumpForwardClipId) a.jumpForward = &clip;
        if (clip.id == c.jumpBackwardClipId) a.jumpBackward = &clip;
        if (clip.id == c.jumpLeftClipId) a.jumpLeft = &clip;
        if (clip.id == c.jumpRightClipId) a.jumpRight = &clip;
        if (clip.id == c.jumpPrepClipId) a.jumpPrep = &clip;
        if (clip.id == c.standUpBackClipId) a.standUpBack = &clip;
        if (clip.id == c.standUpFrontClipId) a.standUpFront = &clip;
        if (clip.id == c.turnLeftClipId) a.turnLeft = &clip;
        if (clip.id == c.turnRightClipId) a.turnRight = &clip;
        for (std::size_t k = 0; k < c.tackleClipIds.size() && k < a.tackles.size(); ++k)
            if (clip.id == c.tackleClipIds[k]) a.tackles[k] = &clip;
    }
    // Os levantares escolhidos por padrao no laboratorio (Kimodo).
    for (const auto& clip : clips) {
        if (clip.id == "stand_up_front_kimodo") a.standUpFront = &clip;
        if (clip.id == "stand_up_back_kimodo") a.standUpBack = &clip;
    }
    return a;
}

} // namespace

const char* ballPreviewGaitName(BallPreviewGait3D gait) { return spec(gait).name; }
const char* ballPreviewGaitKey(BallPreviewGait3D gait) { return spec(gait).key; }

bool parseBallPreviewGait(const std::string& key, BallPreviewGait3D& gait) {
    for (std::size_t i = 0; i < Gaits.size(); ++i) {
        if (key == Gaits[i].key) {
            gait = static_cast<BallPreviewGait3D>(i);
            return true;
        }
    }
    return false;
}

struct BallTouchPreview3D::Physics {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
};

BallTouchPreview3D::BallTouchPreview3D(const RagdollCharacter3D& character,
    std::span<const AnimationClip3D> clips)
    : m_profile(character.profile), m_physics(std::make_unique<Physics>()) {
    // (Todos os clipes: o modo padrao usa os papeis do manifesto por cima dos
    // da locomocao fisica. Copias: os ponteiros apontam para este vetor, que
    // nao muda mais.)
    m_clips.assign(clips.begin(), clips.end());
    m_animations = defaultLocomotionAnimations3D(mapAnimations(character, m_clips),
        character.defaultLocomotionClipIds, m_clips);
}

BallTouchPreview3D::~BallTouchPreview3D() = default;

const AnimationClip3D* BallTouchPreview3D::gaitClip(BallPreviewGait3D gait) const {
    switch (gait) {
    case BallPreviewGait3D::Idle: return m_animations.idle;
    case BallPreviewGait3D::Trot: return m_animations.walk;
    case BallPreviewGait3D::Sprint: return m_animations.sprint;
    case BallPreviewGait3D::StrafeLeft: return m_animations.strafeLeft;
    case BallPreviewGait3D::StrafeRight: return m_animations.strafeRight;
    case BallPreviewGait3D::Backpedal: return m_animations.walkBackward;
    case BallPreviewGait3D::SprintStrafeLeft: return m_animations.sprintStrafeLeft;
    case BallPreviewGait3D::SprintStrafeRight: return m_animations.sprintStrafeRight;
    default: return nullptr;
    }
}

BallTouchPreviewResult3D BallTouchPreview3D::run(const BallTouchPreviewSettings3D& request,
    const std::atomic<bool>* cancel, std::atomic<float>* progress) {
    // Pedido do usuario (06/10): o visualizador na locomocao padrao (o corpo
    // pela animacao, sem fisica), para ver o movimento limpo - o mesmo laco
    // da sonda MatterLocomotionProbe: a capsula com velocidade definida, a
    // DefaultLocomotion3D com o gesto do manejo e a bola fisica na cena.
    const auto wallStart = std::chrono::steady_clock::now();
    BallTouchPreviewResult3D result;
    const RagdollProfile3D& profile = m_profile;
    if (m_animations.idle == nullptr || m_animations.walk == nullptr) {
        result.error = "clipes de locomocao ausentes";
        return result;
    }
    const GaitSpec& gait = spec(request.gait);
    if (const AnimationClip3D* clip = gaitClip(request.gait)) {
        result.gaitClipId = clip->id;
        result.gaitClipFramesPerSecond = clip->sourceSampleRateHz > 0.0f ? clip->sourceSampleRateHz : 30.0f;
        result.gaitClipSeconds = clip->durationSeconds;
    }

    // A cena so com o chao e a bola (o corpo nao esta nela).
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.enableContinuousCollision = true;
    sceneSettings.enableStabilization = true;
    auto scene = m_physics->engine.createScene(sceneSettings, m_physics->materials);
    static_cast<void>(staticFloor(*scene));

    DefaultLocomotion3D locomotion;
    locomotion.reset(0.0f);
    DefaultLocomotionSettings3D& settings = locomotion.settings();
    BallControl3D ballControl;
    (void)request.areaTouch;

    std::array<std::size_t, 2> feet {};
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }

    InflatedBallProfile3D ballProfile;
    InflatedBallHandle3D ball {};
    const float facing = 0.0f;
    result.facingYawRadians = facing;
    const float travel = facing + gait.travelDegrees * Pi / 180.0f;
    const Vec3 moveWorld { std::cos(travel), std::sin(travel), 0.0f };
    const float directionYaw = facing + request.directionDegrees * Pi / 180.0f;
    result.directionWorld = { std::cos(directionYaw), std::sin(directionYaw), 0.0f };

    // Linha do tempo: 0,4 s parado, a marcha ate firmar, a fase pedida (ate
    // um ciclo a mais) e o depois.
    constexpr float MoveStart = 0.4f;
    const float requestEarliest = MoveStart + gait.warmupSeconds;
    const float requestLatest = requestEarliest + 2.5f;
    const float target = request.requestPhase - std::floor(request.requestPhase);
    const float after = std::max(0.2f, request.recordAfterSeconds);
    const std::size_t beforeTicks = static_cast<std::size_t>(std::max(0.0f, request.recordBeforeSeconds) / Dt);
    const int maximumTicks = static_cast<int>((requestLatest + after) / Dt) + 2;

    // A capsula (como na sonda: o motor do modo padrao - o rumo gira rapido,
    // a velocidade vai aos poucos; invertendo, freia antes).
    constexpr float Acceleration = 16.0f, Deceleration = 22.0f, TurnRate = 10.0f, Brake = 30.0f;
    Vec3 position, velocity;
    RagdollState3D bodyState;
    BallControlOutput3D touch;
    std::deque<BallTouchPreviewFrame3D> history;
    float requestAt = -1.0f;
    float previousCycle = -1.0f;
    BallActionKind3D pendingRequest = BallActionKind3D::None;
    bool requestOpen = false;
    int tick = 0;
    for (; tick < maximumTicks; ++tick) {
        if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) {
            result.error = "cancelado";
            return result;
        }
        const float t = static_cast<float>(tick) * Dt;
        if (requestAt >= 0.0f && t > requestAt + after) break;
        if (progress != nullptr) {
            const float total = requestAt >= 0.0f ? requestAt + after : requestEarliest + after + 0.5f;
            progress->store(std::clamp(t / std::max(0.1f, total), 0.0f, 0.99f), std::memory_order_relaxed);
        }
        const bool moving = gait.moving && t >= MoveStart;

        // O pedido: a fase da marcha passando pela pedida (ou o limite).
        const float cycle = locomotion.output().cyclePhase;
        if (requestAt < 0.0f && t >= requestEarliest && !bodyState.links.empty()) {
            bool reached = !gait.moving || t >= requestLatest;
            if (!reached && previousCycle >= 0.0f) {
                const float advance = std::fmod(cycle - previousCycle + 1.0f, 1.0f);
                const float toTarget = std::fmod(target - previousCycle + 1.0f, 1.0f);
                reached = advance < 0.5f && toTarget <= advance;
            }
            if (reached) {
                requestAt = t;
                result.cyclePhaseAtRequest = cycle;
                const Vec3 pelvis = bodyState.links.front().position;
                result.pelvisAtRequest = pelvis;
                const float c = std::cos(facing), s = std::sin(facing);
                const Vec3 offset = request.ballOffset;
                InflatedBallDefinition3D definition;
                definition.profile = ballProfile;
                definition.position = { pelvis.x + c * offset.x - s * offset.y,
                    pelvis.y + s * offset.x + c * offset.y,
                    std::max(ballProfile.radiusMeters, offset.z) };
                definition.entityId = 2;
                definition.collidesWithRagdolls = false;
                ball = scene->createInflatedBall(definition);
                result.ballAtRequest = definition.position;
                if (request.ballVelocitySet) {
                    const Vec3 v = request.ballVelocity;
                    const Vec3 ballVelocity { c * v.x - s * v.y, s * v.x + c * v.y, 0.0f };
                    const Vec3 spin = cross(Vec3 { 0.0f, 0.0f, 1.0f }, ballVelocity)
                        * (1.0f / ballProfile.radiusMeters);
                    scene->setInflatedBallMotion(ball, definition.position, ballVelocity, spin);
                } else if (request.ballRolling && moving) {
                    const Vec3 ballVelocity = planar(velocity);
                    const Vec3 spin = cross(Vec3 { 0.0f, 0.0f, 1.0f }, ballVelocity)
                        * (1.0f / ballProfile.radiusMeters);
                    scene->setInflatedBallMotion(ball, definition.position, ballVelocity, spin);
                }
                pendingRequest = request.kind;
                requestOpen = true;
            }
        }
        previousCycle = cycle;

        // A velocidade pedida (o chute correndo freia um pouco: a escala do
        // manejo) e a capsula.
        Vec3 wanted;
        if (moving)
            wanted = moveWorld * (defaultLocomotionSpeed3D(settings, travel - facing, gait.sprint) * touch.speedScale);
        {
            Vec3 planarVelocity { velocity.x, velocity.y, 0.0f };
            const float current = planarVelocity.length(), wantedSpeed = wanted.length();
            if (wantedSpeed < 1e-4f) {
                planarVelocity = current > 1e-4f
                    ? planarVelocity * (std::max(0.0f, current - Deceleration * Dt) / current) : Vec3 {};
            } else if (current < 0.3f) {
                const Vec3 change = wanted - planarVelocity;
                const float length = change.length(), limit = Acceleration * Dt;
                planarVelocity += length > limit ? change * (limit / length) : change;
            } else {
                const Vec3 from = planarVelocity * (1.0f / current), to = wanted * (1.0f / wantedSpeed);
                const float angle = std::atan2(from.x * to.y - from.y * to.x, dot(from, to));
                if (std::abs(angle) > 120.0f * Pi / 180.0f) {
                    planarVelocity = planarVelocity * (std::max(0.0f, current - Brake * Dt) / current);
                } else {
                    const float turn = std::clamp(angle, -TurnRate * Dt, TurnRate * Dt);
                    const Vec3 direction { from.x * std::cos(turn) - from.y * std::sin(turn),
                        from.x * std::sin(turn) + from.y * std::cos(turn), 0.0f };
                    const float rate = wantedSpeed > current ? Acceleration : Deceleration;
                    planarVelocity = direction * (current + std::clamp(wantedSpeed - current, -rate * Dt, rate * Dt));
                }
            }
            velocity = planarVelocity;
            position += velocity * Dt;
        }

        // O manejo de bola sobre o corpo do tick anterior (como no
        // laboratorio).
        touch = {};
        InflatedBallState3D ballState;
        if (!bodyState.links.empty()) {
            BallControlInput3D ballInput;
            ballInput.request = pendingRequest;
            ballInput.power = std::clamp(request.power, 0.0f, 1.0f);
            pendingRequest = BallActionKind3D::None;
            ballInput.direction = result.directionWorld;
            ballInput.lookYawRadians = request.kind == BallActionKind3D::Shot ? directionYaw : facing;
            ballInput.shotLift = request.shotLift;
            ballInput.shotCurve = request.shotCurve;
            if (ball && scene->contains(ball)) {
                ballState = scene->inflatedBallState(ball);
                ballInput.ballValid = true;
                ballInput.ballPosition = ballState.position;
                ballInput.ballVelocity = ballState.linearVelocity;
                ballInput.ballRadius = ballProfile.radiusMeters;
                ballInput.ballMass = ballProfile.massKg;
            }
            ballInput.profile = &profile;
            ballInput.state = &bodyState;
            ballInput.facingYawRadians = facing;
            ballInput.bodyYawRadians = locomotion.output().facingYawRadians;
            ballInput.bodyYawValid = true;
            ballInput.groundHeight = 0.0f;
            ballInput.requestedVelocity = wanted;
            // (A velocidade do corpo pela capsula, como no laboratorio.)
            ballInput.bodyVelocity = planar(velocity);
            ballInput.bodyVelocityValid = true;
            ballInput.contactTimingFactor = std::clamp(request.contactTimingFactor, 0.0f, 1.0f);
            for (std::size_t side = 0; side < 2; ++side) {
                ballInput.swingProgress[side] = locomotion.output().swingProgress[side];
                ballInput.footPlanted[side] = locomotion.output().footPlanted[side];
                ballInput.secondsToTouchdown[side] = locomotion.output().secondsToTouchdown[side];
            }
            ballInput.standing = planar(velocity).length() < 0.3f && wanted.lengthSquared() < 0.01f;
            ballInput.animatedBody = true;
            ballInput.deltaTime = Dt;
            ballControl.update(ballInput, touch);
            if (touch.setMotion && ball && scene->contains(ball))
                scene->setInflatedBallMotion(ball, ballState.position, touch.ballVelocity, touch.ballSpin);
            if (requestOpen && touch.touched && result.touchFrame < 0) {
                result.side = touch.side;
                result.surface = touch.surface;
                result.ballVelocityAfter = touch.ballVelocity;
                result.ballSpinAfter = touch.ballSpin;
                result.touchDelaySeconds = t - requestAt;
            }
            if (requestOpen && touch.missReason != 0 && result.touchDelaySeconds < 0.0f)
                result.missReason = touch.missReason;
        }

        // O corpo pela animacao (o gesto do manejo por cima).
        DefaultLocomotionInput3D input;
        input.feetPositionWorld = position;
        input.velocityWorld = velocity;
        input.grounded = true;
        input.requestedVelocityWorld = wanted;
        input.facingYawRadians = facing;
        input.deltaTime = Dt;
        input.groundAt = [](Vec3 at) {
            GroundProbeResult3D ground;
            ground.hasSurface = true;
            ground.pointWorld = { at.x, at.y, 0.0f };
            ground.normalWorld = { 0.0f, 0.0f, 1.0f };
            return ground;
        };
        input.ballGesture = touch.gesture;
        input.motionVariety = request.motionVariety ? 1.0f : 0.0f;
        input.varietySeed = 0x9E3779B9u;
        locomotion.update(profile, m_animations, input);
        const DefaultLocomotionOutput3D& out = locomotion.output();
        if (!out.valid) {
            result.error = "pose invalida";
            return result;
        }
        scene->simulate(Dt);

        // O corpo deste tick (para o manejo do proximo; velocidades pela
        // diferenca).
        {
            const bool first = bodyState.links.empty();
            bodyState.links.resize(out.pose.linkPositions.size());
            for (std::size_t i = 0; i < out.pose.linkPositions.size(); ++i) {
                PhysicsBodyState3D& link = bodyState.links[i];
                link.linearVelocity = first ? Vec3 {} : (out.pose.linkPositions[i] - link.position) * (1.0f / Dt);
                link.position = out.pose.linkPositions[i];
                link.orientation = out.pose.linkOrientations[i];
            }
        }

        // O quadro (a pose do tick e a bola depois do passo).
        BallTouchPreviewFrame3D frame;
        frame.timeSeconds = requestAt >= 0.0f ? t + Dt - requestAt : t + Dt;
        frame.linkPositions = out.pose.linkPositions;
        frame.linkOrientations = out.pose.linkOrientations;
        frame.targetLinkPositions = out.pose.linkPositions;
        frame.targetLinkOrientations = out.pose.linkOrientations;
        if (ball && scene->contains(ball)) {
            const InflatedBallState3D b = scene->inflatedBallState(ball);
            frame.ballPresent = true;
            frame.ballPosition = b.position;
            frame.ballOrientation = b.orientation;
            frame.ballVelocity = b.linearVelocity;
        }
        for (std::size_t side = 0; side < 2; ++side) {
            frame.feet[side] = out.pose.linkPositions[feet[side]];
            frame.planted[side] = out.footPlanted[side];
            frame.footPhase[side] = out.footPlanted[side] ? 0 : 1;
            frame.touching[side] = out.footContact[side] > 0.5f;
            frame.supporting[side] = out.footContact[side] > 0.5f;
        }
        frame.footGoal = touch.foot;
        frame.plant = touch.plant;
        frame.kick = touch.kick;
        frame.phase = touch.phase;
        frame.side = touch.side;
        frame.footGap = touch.footGap;
        frame.gaitRate = touch.gaitRate;
        frame.touched = touch.touched;
        frame.cyclePhase = out.cyclePhase;
        frame.locomotionState = 0;
        if (requestAt < 0.0f) {
            history.push_back(std::move(frame));
            while (history.size() > beforeTicks) history.pop_front();
        } else {
            if (result.frames.empty()) {
                // O que veio antes do pedido, com o tempo relativo a ele.
                for (auto& old : history) {
                    old.timeSeconds -= requestAt;
                    result.frames.push_back(std::move(old));
                }
                history.clear();
                result.requestFrame = result.frames.size();
            }
            if (frame.touched && result.touchFrame < 0)
                result.touchFrame = static_cast<int>(result.frames.size());
            result.frames.push_back(std::move(frame));
        }
    }
    if (requestAt < 0.0f) {
        result.error = "o pedido nao saiu (a marcha nao chegou na fase)";
        return result;
    }
    if (result.touchFrame < 0 && result.missReason == 0 && request.kind != BallActionKind3D::None)
        result.missReason = 3;
    result.simulatedSeconds = static_cast<float>(tick) * Dt;
    result.wallSeconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - wallStart).count();
    result.valid = true;
    if (progress != nullptr) progress->store(1.0f, std::memory_order_relaxed);
    return result;
}

CharacterLocomotionAnimations3D mapCharacterAnimations3D(const RagdollCharacter3D& character,
    const std::vector<AnimationClip3D>& clips) {
    return mapAnimations(character, clips);
}

BallPreviewAssets3D loadBallPreviewAssets3D(const std::string& assetsDirectory) {
    BallPreviewAssets3D assets;
    assets.character = loadRagdollCharacter3D(assetsDirectory + "/characters/football_player/character.json");
    const RagdollCharacter3D& c = assets.character;
    std::vector<std::string> ids { c.idleClipId, c.walkClipId, c.walkBackwardClipId, c.sprintClipId,
        c.sprintBackwardClipId, c.strafeLeftClipId, c.strafeRightClipId, c.sprintStrafeLeftClipId,
        c.sprintStrafeRightClipId, c.jumpStandingClipId, c.jumpForwardClipId, c.jumpBackwardClipId,
        c.jumpLeftClipId, c.jumpRightClipId, c.standUpBackClipId, c.standUpFrontClipId,
        c.idleToSprintClipId, c.crouchToSprintClipId, c.runForwardArcLeftClipId, c.runForwardArcRightClipId,
        c.runBackwardArcRightClipId, c.jumpPrepClipId, c.turnLeftClipId, c.turnRightClipId,
        "stand_up_front_kimodo", "stand_up_back_kimodo" };
    ids.insert(ids.end(), c.tackleClipIds.begin(), c.tackleClipIds.end());
    for (const auto& role : c.defaultLocomotionClipIds) ids.push_back(role.second);
    for (const auto& id : ids) {
        if (id.empty()) continue;
        const std::string path = assetsDirectory + "/animations/clips/" + id + ".matteranim.json";
        if (!std::filesystem::exists(path)) continue;
        assets.clips.push_back(loadAnimationClip3D(path));
    }
    return assets;
}

} // namespace MatterEngine
