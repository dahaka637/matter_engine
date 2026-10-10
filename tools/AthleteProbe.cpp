// Sonda dos atletas (rework de 10/10/2026): o MESMO runtime do laboratorio
// (AthleteRuntime3D) com a fisica de verdade (Jolt), sem janela. Roteiros de
// teclas, bola e empurroes; mede o que importa para o novo controle e grava o
// dump que o MatterModelInspector desenha com o jogador de verdade:
//   MatterModelInspector quadros --dump <arquivo> --cenario <nome>
//
// uso: MatterAthleteProbe [--cenario nome] [--dump arquivo] [--cada N] [--lista]
//      [--fisico] (comeca no modo fisico, como ao assumir no laboratorio)
//      [--ativo 0|1] (o Corpo Ativo; 1 padrao)  [--pegadas 0|1] (locomocao por
//      pegadas; 0 padrao)  [--teste] (os contratos, ctest)
// Sem --cenario roda todos.
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Dt = 1.0f / 120.0f;

// O que o "jogador" faz num instante: teclas no referencial da camera (x
// frente, y esquerda), o olhar e os pedidos.
struct Keys {
    float forward = 0.0f;
    float left = 0.0f;
    bool sprint = false;
    float cameraYaw = 0.0f;
    bool jump = false;
    bool crouch = false;
    BallActionKind3D ball = BallActionKind3D::None;
    float power = 0.5f;
};

// Empurrao: forca (N, no mundo) por um tempo, repartida em pelve, peito e
// alto do peito como o teste de impacto do laboratorio.
struct Push {
    float at = 0.0f;
    float seconds = 0.1f;
    Vec3 force;
};

// Um objeto arremessado no atleta (a pancada de verdade: contato, nao
// forca): caixa de massa e meia-dimensao dadas, saindo de `start` (em
// relacao aos pes) com `velocity` no instante `at`.
struct Throw {
    float at = 0.0f;
    float massKg = 20.0f;
    Vec3 halfExtents { 0.2f, 0.2f, 0.2f };
    Vec3 start;
    Vec3 velocity;
};

struct Scenario {
    const char* name;
    float seconds;
    std::function<Keys(float)> keys;
    bool ball = false;
    Vec3 ballStart;     // em relacao aos pes, no referencial inicial
    Vec3 ballVelocity;
    std::vector<Push> pushes;
    std::vector<Throw> throws;
    // Pedido automatico com a bola a menos disto da pelve (no plano).
    float autoDistance = 0.0f;
    BallActionKind3D autoKind = BallActionKind3D::Dribble;
};

std::vector<Scenario> scenarios() {
    std::vector<Scenario> list;
    list.push_back({ "parado", 3.0f, [](float) { return Keys {}; } });
    list.push_back({ "arranca", 3.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } });
    list.push_back({ "sprint", 4.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = true; return k; } });
    list.push_back({ "inverte", 4.0f, [](float t) {
        Keys k;
        k.forward = t < 0.3f ? 0.0f : t < 2.0f ? 1.0f : -1.0f;
        return k; } });
    list.push_back({ "zigue", 4.5f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.left = t < 0.3f ? 0.0f : std::fmod(t, 1.0f) < 0.5f ? 1.0f : -1.0f;
        k.sprint = true;
        return k; } });
    list.push_back({ "lados", 4.0f, [](float t) { Keys k; k.left = t < 0.3f ? 0.0f : t < 2.0f ? 1.0f : -1.0f; return k; } });
    list.push_back({ "corte", 4.0f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.sprint = true;
        if (t > 2.0f) { k.forward = 0.0f; k.left = 1.0f; }
        return k; } });
    list.push_back({ "para", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f && t < 1.8f ? 1.0f : 0.0f; return k; } });
    list.push_back({ "para_sprint", 3.5f, [](float t) {
        Keys k;
        k.forward = t > 0.3f && t < 2.0f ? 1.0f : 0.0f;
        k.sprint = true;
        return k; } });
    list.push_back({ "diagonal", 3.0f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.left = t > 0.3f ? 1.0f : 0.0f;
        return k; } });
    list.push_back({ "costas", 3.0f, [](float t) { Keys k; k.forward = t > 0.3f ? -1.0f : 0.0f; return k; } });
    list.push_back({ "gira", 3.0f, [](float t) { Keys k; k.cameraYaw = std::min(1.0f, std::max(0.0f, t - 0.5f)) * Pi * 0.75f; return k; } });
    list.push_back({ "pulo", 3.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.jump = t > 1.2f && t < 1.22f; return k; } });
    list.push_back({ "carrinho", 3.5f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.sprint = true;
        k.crouch = t > 1.6f && t < 1.7f;
        return k; } });
    {
        Scenario s { "toque_parado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.45f, -0.12f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "toque_lado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.15f, -0.45f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "toque_corre", 4.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.ball = true;
        s.ballStart = { 3.0f, -0.1f, 0.11f };
        s.autoDistance = 0.9f;
        list.push_back(s);
    }
    {
        Scenario s { "chute_parado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Shot; k.power = 0.8f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.55f, -0.15f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "chute_corre", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.ball = true;
        s.ballStart = { 3.2f, -0.12f, 0.11f };
        s.autoDistance = 1.0f;
        s.autoKind = BallActionKind3D::Shot;
        list.push_back(s);
    }
    {
        Scenario s { "caixa_leve", 3.0f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 6.0f, { 0.15f, 0.15f, 0.15f }, { 0.0f, 1.2f, 1.2f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "caixa_media", 3.0f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 20.0f, { 0.2f, 0.2f, 0.2f }, { 0.0f, 1.3f, 1.1f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "caixa_forte", 3.5f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 60.0f, { 0.25f, 0.25f, 0.25f }, { 0.0f, 1.4f, 1.1f }, { 0.0f, -6.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "caixa_correndo", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.throws.push_back({ 1.5f, 15.0f, { 0.2f, 0.2f, 0.2f }, { 5.5f, 1.3f, 1.1f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_leve", 3.0f, [](float) { return Keys {}; } };
        s.pushes.push_back({ 1.0f, 0.1f, { 0.0f, 350.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_medio", 3.0f, [](float) { return Keys {}; } };
        s.pushes.push_back({ 1.0f, 0.1f, { 0.0f, 900.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_forte", 3.5f, [](float) { return Keys {}; } };
        s.pushes.push_back({ 1.0f, 0.12f, { 0.0f, 2200.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_correndo", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.pushes.push_back({ 1.6f, 0.1f, { 0.0f, 500.0f, 0.0f } });
        list.push_back(s);
    }
    return list;
}

struct Metrics {
    std::vector<float> footSlide;
    float peakUpperAcceleration = 0.0f;
    float peakLegAcceleration = 0.0f;
    float worstPoseJump = 0.0f;
    int modeSwitches = 0;
    float animatedSeconds = 0.0f;
    float physicalSeconds = 0.0f;
    float returningSeconds = 0.0f;
    float activeSeconds = 0.0f;
    float assistImpulse = 0.0f;
    int requests = 0;
    int touches = 0;
    bool fell = false;
    float lowestPelvis = 10.0f;
    float ballSpeedAfter = 0.0f;
    // O pe apoiado (o estilo diz apoio, no modo padrao): a velocidade dele
    // no plano (m/s) - zero e o certo. Pegadas: a maior correcao na saida do
    // apoio (m), o maior erro de alcance da perna (m) e os passos de
    // acomodacao.
    std::vector<float> plantedSlide;
    float worstLiftCorrection = 0.0f;
    float worstReach = 0.0f;
    int settleSteps = 0;
    int earlyLifts = 0;
    int catchSteps = 0;
    int startSteps = 0;
    // Os modos por onde passou ("animado>ativo>volta>animado") e o do fim.
    std::string trail;
    LegacyBodyMode3D finalMode = LegacyBodyMode3D::Animated;
};

float percentile(std::vector<float> values, float q) {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    const std::size_t index = std::min(values.size() - 1, static_cast<std::size_t>(q * static_cast<float>(values.size() - 1)));
    return values[index];
}

void dumpFrame(FILE* file, const RagdollProfile3D& profile, const RagdollState3D& state, float t, int mode) {
    std::fprintf(file, "F %.4f %d 0\n", t, mode);
    for (std::size_t i = 0; i < profile.links.size() && i < state.links.size(); ++i) {
        const auto& collider = profile.links[i].collider;
        const Vec3 center = state.links[i].position + state.links[i].orientation.rotate(collider.localPosition);
        const Quaternion q = (state.links[i].orientation * collider.localOrientation).normalized();
        if (collider.shape == RagdollColliderShape3D::Box) {
            std::fprintf(file, "B %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w, collider.boxHalfExtents.x, collider.boxHalfExtents.y,
                collider.boxHalfExtents.z);
        } else {
            std::fprintf(file, "C %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w, ragdollCapsuleHalfSegment3D(collider),
                collider.radiusMeters, ragdollCapsuleRadiusAtPositiveX3D(collider));
        }
    }
}

const char* modeName(LegacyBodyMode3D mode) {
    switch (mode) {
    case LegacyBodyMode3D::Animated: return "animado";
    case LegacyBodyMode3D::Physical: return "fisico";
    case LegacyBodyMode3D::Returning: return "volta";
    case LegacyBodyMode3D::Active: return "ativo";
    }
    return "?";
}

} // namespace

namespace {

struct ProbeContext {
    const RagdollProfile3D* profile = nullptr;
    CharacterGaitSpeeds3D gaitSpeeds;
    CharacterMotorSettings3D avatar;
    AthleteRuntimeAssets3D runtimeAssets;
};

// Um cenario do comeco ao fim (a cena nova, o boneco, as teclas, as medidas).
Metrics runScenario(const ProbeContext& context, const Scenario& scenario, const AthleteRuntimeSettings3D& settings,
    bool startPhysical, FILE* dump, int every) {
    const RagdollProfile3D& profile = *context.profile;
    const AthleteFootGeometry3D footGeometry = resolveAthleteFootGeometry3D(profile, resolveAthleteLegs3D(profile), 0.08f);
    const CharacterGaitSpeeds3D& gaitSpeeds = context.gaitSpeeds;
    const CharacterMotorSettings3D& avatar = context.avatar;
    const AthleteRuntimeAssets3D& runtimeAssets = context.runtimeAssets;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    {
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 60.0f, 60.0f, 0.1f };
        shape.materialId = "concrete";
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.1f };
        ground.materialId = "concrete";
        (void)scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
    }
    scene->createCharacter({ 0.0f, 0.0f, 0.0f }, avatar);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.active = true;
    spawn.pelvisPosition = { 0.0f, 0.0f, profile.standingRootHeightMeters };
    const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
    RagdollState3D bodyState = scene->ragdollState(handle);
    AthleteState3D athlete;
    LegacyAthleteState3D& legacy = athlete.legacy;
    legacy.guideFacingYawRadians = 0.0f;
    legacy.locomotion.reset(profile, bodyState);
    legacy.adaptivePhysics.reset(profile);
    // Como "Restaurar pose" do laboratorio: ja no modo padrao. (--fisico:
    // como ao assumir o boneco, no fisico ate ele assentar.)
    if (!startPhysical) {
        legacy.defaultLocomotion.reset(0.0f);
        legacy.guided = true;
        legacy.bodyMode = LegacyBodyMode3D::Animated;
        scene->setRagdollGuided(handle, true);
    }
    InflatedBallHandle3D ballHandle;
    InflatedBallState3D ballState;
    BallOwnership3D ownership;
    if (dump != nullptr) std::fprintf(dump, "S %s\n", scenario.name);

    AthleteRuntime3D runtime;
    Vec3 followVelocity {};
    bool controlledAnimated = !startPhysical;
    Vec3 capsuleMoved {};
    float wallImpact = 0.0f;
    Metrics metrics;
    metrics.trail = modeName(legacy.bodyMode);
    LegacyBodyMode3D previousMode = legacy.bodyMode;
    std::vector<PhysicsBodyState3D> previousLinks = bodyState.links;
    std::vector<Vec3> previousVelocities(bodyState.links.size());
    bool previousJump = false;
    float jumpHeld = 0.0f;
    float autoCooldown = 0.0f;
    const int ticks = static_cast<int>(scenario.seconds / Dt);
    // Aquecimento (0,1 s antes do roteiro, sem teclas e sem pegadas): o corpo
    // nasce na pose de ligacao e assenta na do parado - com as pegadas, os
    // pes travados viriam dela. (Na volta do fisico, nao.)
    const int warmTicks = startPhysical ? 0 : 12;
    AthleteRuntimeSettings3D warmSettings = settings;
    warmSettings.footstepLocomotion = false;
    for (int step = -warmTicks; step < ticks; ++step) {
        const float t = static_cast<float>(step) * Dt;
        const Keys keys = step < 0 ? Keys {} : scenario.keys(t);
        if (step == 0 && scenario.ball) {
            InflatedBallDefinition3D definition;
            definition.position = scenario.ballStart;
            definition.linearVelocity = scenario.ballVelocity;
            definition.entityId = 2;
            ballHandle = scene->createInflatedBall(definition);
            ballState = scene->inflatedBallState(ballHandle);
        }
        // A capsula como no laboratorio (o personagem controlado no chao).
        const PhysicsCharacterState3D before = scene->characterState();
        const Vec3 forward { std::cos(keys.cameraYaw), std::sin(keys.cameraYaw), 0.0f };
        const Vec3 right { forward.y, -forward.x, 0.0f };
        // Caido, levantando ou no carrinho, a capsula fica sem teclas (como no
        // laboratorio: o corpo e quem a leva).
        const CharacterLocomotionState3D lastState = legacy.locomotion.telemetry().state;
        const bool down = controlledAnimated ? legacy.defaultLocomotion.output().tackling
            : lastState == CharacterLocomotionState3D::Fallen || lastState == CharacterLocomotionState3D::GettingUp
                || lastState == CharacterLocomotionState3D::Tackling;
        CharacterMotorCommand3D command;
        command.ignoreRagdolls = true;
        if (!down) command.moveDirection = forward * keys.forward - right * keys.left;
        command.sprint = keys.sprint;
        CharacterIntent3D intent;
        intent.requestedFacingYawRadians = keys.cameraYaw;
        intent.effort = keys.sprint ? 1.0f : 0.0f;
        intent.crouch = keys.crouch;
        if (command.moveDirection.lengthSquared() > 1e-4f) {
            const Vec3 direction = command.moveDirection.normalized();
            const float travelRelative = std::atan2(direction.y, direction.x) - keys.cameraYaw;
            const float wanted = controlledAnimated
                ? defaultLocomotionSpeed3D(legacy.defaultLocomotion.settings(), travelRelative, keys.sprint)
                : characterGaitSpeed3D(gaitSpeeds, travelRelative, keys.sprint);
            const float nominal = keys.sprint ? avatar.sprintSpeed : avatar.walkSpeed;
            if (wanted > 0.0f && nominal > 0.0f) command.speedScale = wanted / nominal;
            if (!controlledAnimated && wanted > 0.0f)
                command.speedScale *= std::clamp(legacy.locomotion.telemetry().terrainSpeedLimit / wanted, 0.2f, 1.0f);
            intent.requestedDirectionWorld = direction;
            float speed = wanted;
            if (!controlledAnimated) speed = std::min(speed, legacy.locomotion.telemetry().terrainSpeedLimit);
            intent.requestedVelocityWorld = direction * speed;
        }
        // Pulo ao soltar: segurar o espaco carrega.
        if (keys.jump) jumpHeld += Dt;
        command.jumpPressed = previousJump && !keys.jump;
        command.jumpChargeSeconds = jumpHeld;
        if (!keys.jump && previousJump) jumpHeld = 0.0f;
        previousJump = keys.jump;
        CharacterMotorSettings3D active = avatar;
        if (controlledAnimated) {
            active.groundAcceleration = 16.0f;
            active.groundDeceleration = 22.0f;
            active.groundTurnRate = 10.0f;
            active.groundBrakeDeceleration = 30.0f;
        } else {
            active = adaptiveCharacterMotorSettings3D(active, before, command,
                physicalNavigationAcceleration3D(settings.e5WalkingAssistPercent * 0.01f));
        }
        command.followVelocity = followVelocity;
        command.navigationProxy = true;
        scene->moveCharacter(command, active, Dt);
        const PhysicsCharacterState3D& capsule = scene->characterState();
        {
            const Vec3 moved { (capsule.position.x - before.position.x) / Dt,
                (capsule.position.y - before.position.y) / Dt, 0.0f };
            const float speed = capsuleMoved.length();
            const float braking = std::max({ active.groundAcceleration, active.groundDeceleration,
                active.groundBrakeDeceleration }) * Dt;
            wallImpact = speed > 0.5f && capsule.grounded
                ? std::max(0.0f, dot(capsuleMoved - moved, capsuleMoved * (1.0f / speed)) - braking) : 0.0f;
            capsuleMoved = moved;
        }
        AthleteControlInput3D control;
        control.controlledEntityId = 1;
        control.intent = intent;
        control.facingYawRadians = keys.cameraYaw;
        control.lookPitchRadians = -0.2f;
        control.sprinting = keys.sprint && Vec3 { capsule.velocity.x, capsule.velocity.y, 0.0f }.lengthSquared() > 0.01f;
        control.proxyVelocityWorld = { capsule.velocity.x, capsule.velocity.y, 0.0f };
        control.capsuleWallImpact = wallImpact;
        control.capsuleFollowVelocityWorld = followVelocity;
        control.jumpCharge = characterJumpCharge3D(active, command.jumpChargeSeconds);
        control.ballRequest = keys.ball;
        control.ballRequestPower = keys.power;
        control.ballRequestAimYaw = keys.cameraYaw;
        autoCooldown -= Dt;
        if (step >= 0 && scenario.autoDistance > 0.0f && scenario.ball && autoCooldown <= 0.0f && !bodyState.links.empty()) {
            const Vec3 pelvis = bodyState.links.front().position;
            const Vec3 offset = ballState.position - pelvis;
            if (std::hypot(offset.x, offset.y) < scenario.autoDistance) {
                control.ballRequest = scenario.autoKind;
                control.ballRequestPower = scenario.autoKind == BallActionKind3D::Shot ? 0.8f : 0.45f;
                autoCooldown = 10.0f;
            }
        }
        if (control.ballRequest != BallActionKind3D::None) ++metrics.requests;
        if (std::getenv("XPAREDE")) {
            float wallImpulse = 0.0f;
            for (const RagdollInteraction3D& contact : bodyState.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Dynamic && contact.normalWorld.z < 0.6f) {
                    wallImpulse += contact.normalImpulseNewtonSeconds;
                    if (contact.normalImpulseNewtonSeconds > 0.2f)
                        std::printf("  t %.3f link %s motion %d normal %.2f %.2f %.2f impulso %.2f\n", t,
                            profile.links[contact.linkIndex].id.c_str(), static_cast<int>(contact.otherMotion),
                            contact.normalWorld.x, contact.normalWorld.y, contact.normalWorld.z,
                            contact.normalImpulseNewtonSeconds);
                }
            if (wallImpact > 1.0f) std::printf("  t %.3f capsula bateu %.2f\n", t, wallImpact);
        }
        AthleteSlot3D slot { 1, handle, &bodyState, true, false, &athlete };
        std::vector<AthleteBall3D> balls;
        if (scenario.ball && step >= 0) balls.push_back({ ballHandle, &ballState, &ownership });
        AthleteRuntimeOutput3D output;
        runtime.update(*scene, runtimeAssets, std::span<AthleteSlot3D>(&slot, 1), balls, step < 0 ? warmSettings : settings,
            control, Dt, output);
        if (output.followVelocitySet) followVelocity = output.followVelocityWorld;
        if (output.controlledAnimatedSet) controlledAnimated = output.controlledAnimated;
        if (legacy.ballLastOutput.touched) {
            ++metrics.touches;
            if (dump != nullptr) std::fprintf(dump, "T\n");
        }
        for (const Throw& item : scenario.throws)
            if (t >= item.at && t < item.at + Dt) {
                PhysicsShape3D shape;
                shape.type = PhysicsShapeType3D::Box;
                shape.halfExtents = item.halfExtents;
                shape.materialId = "wood";
                PhysicsBodyDefinition3D box;
                box.position = item.start;
                box.linearVelocity = item.velocity;
                box.massKg = item.massKg;
                box.materialId = "wood";
                box.characteristicSizeMeters = 2.0f * std::max({ item.halfExtents.x, item.halfExtents.y, item.halfExtents.z });
                (void)scene->createBody(box, std::span<const PhysicsShape3D>(&shape, 1));
                if (dump != nullptr) std::fprintf(dump, "E caixa\n");
            }
        for (const Push& push : scenario.pushes)
            if (t >= push.at && t < push.at + push.seconds) {
                scene->applyRagdollLinkForce(handle, 0, push.force * 0.20f, {});
                scene->applyRagdollLinkForce(handle, 2, push.force * 0.32f, {});
                scene->applyRagdollLinkForce(handle, 3, push.force * 0.48f, {});
            }
        scene->simulate(Dt);
        bodyState = scene->ragdollState(handle);
        if (scenario.ball && step >= 0) ballState = scene->inflatedBallState(ballHandle);
        if (step < 0) {
            previousLinks = bodyState.links;
            for (std::size_t i = 0; i < bodyState.links.size() && i < previousVelocities.size(); ++i)
                previousVelocities[i] = bodyState.links[i].linearVelocity;
            previousMode = legacy.bodyMode;
            continue;
        }

        if (std::getenv("XATIVO") && legacy.bodyMode == LegacyBodyMode3D::Active && step % (std::getenv("XATIVO")[0] == '2' ? 1 : 6) == 0) {
            const ActiveBodyOutput3D& a = athlete.activeOutput;
            std::printf("  t %.2f captura fora %.3f | raiz %.3f m giro %.3f rad link %.3f m | apoio %d complac %.2f | base %.3f%s | pedido %.0f/%.0f/%.0f entregue %.0f/%.0f/%.0f ajuda %.0f/%.0f%s%s\n",
                t, athlete.stance.captureOutside(), a.rootError, a.orientationError, a.linkError, a.supportCount, a.compliance,
                athlete.stance.mismatch(), athlete.stance.settling() ? (athlete.stance.recovering() ? " RECUPERA" : " PASSO") : "", a.requestedForce.x,
                a.requestedForce.y, a.requestedForce.z, a.deliveredForce.x, a.deliveredForce.y, a.deliveredForce.z,
                a.assistForce.x, a.assistForce.y, a.knockout ? " NOCAUTE" : "", a.readyToAnimate ? " PRONTO" : "");
            if (std::getenv("XGEO")) {
                const BodyObservation3D o = athlete.observer.observe(profile, bodyState, Dt);
                const Vec3 mid = (o.feet[0].soleCenter + o.feet[1].soleCenter) * 0.5f;
                const Vec3 cap = scene->characterState().position;
                std::printf("      geo cm %+.3f %+.3f v %+.2f %+.2f | pelve %+.3f %+.3f | alvo %+.3f %+.3f (anc %+.3f %+.3f) z %.3f/%.3f | pes E %+.3f %+.3f %s%.0fN D %+.3f %+.3f %s%.0fN | capsula %+.3f %+.3f\n",
                    o.centerOfMass.x - mid.x, o.centerOfMass.y - mid.y, o.centerOfMassVelocity.x, o.centerOfMassVelocity.y,
                    o.pelvisPosition.x - mid.x, o.pelvisPosition.y - mid.y, a.targetRoot.x - mid.x, a.targetRoot.y - mid.y,
                    athlete.stance.baseShift().x, athlete.stance.baseShift().y, a.targetRoot.z, o.pelvisPosition.z,
                    o.feet[0].soleCenter.x - mid.x, o.feet[0].soleCenter.y - mid.y, o.feet[0].supporting ? "*" : "-", o.feet[0].loadNewtons,
                    o.feet[1].soleCenter.x - mid.x, o.feet[1].soleCenter.y - mid.y, o.feet[1].supporting ? "*" : "-", o.feet[1].loadNewtons,
                    cap.x - mid.x, cap.y - mid.y);
            }
        }
        if (std::getenv("XPEG") && settings.footstepLocomotion
            && (legacy.bodyMode == LegacyBodyMode3D::Animated || legacy.bodyMode == LegacyBodyMode3D::Returning)) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            const AthleteLegs3D& legs = athlete.footsteps.legs();
            std::printf("  t %.3f mov %.2f v %.2f |", t, o.movingShare, std::hypot(scene->characterState().velocity.x,
                scene->characterState().velocity.y));
            for (std::size_t side = 0; side < 2; ++side) {
                const char* mode = "LSWP" + static_cast<int>(athlete.footTargets[side].mode);
                const Vec3 stylePos = o.pose.linkPositions[legs.foot[side]];
                const Vec3 target = athlete.footTargets[side].position;
                std::printf(" %c %c%s d %.3f z %.3f alc %.3f lo %.2f td %.2f |", side == 0 ? 'E' : 'D',
                    "FLSP"[static_cast<int>(athlete.footTargets[side].mode)], o.footPlanted[side] ? "*" : " ",
                    std::hypot(target.x - stylePos.x, target.y - stylePos.y), target.z - stylePos.z,
                    athlete.legSynthesis.reachError[side], o.secondsToLiftoff[side], o.secondsToTouchdown[side]);
                (void)mode;
            }
            std::printf("\n");
        }
        // Medidas.
        if (settings.footstepLocomotion && t > 0.35f)
            for (const float reach : athlete.legSynthesis.reachError)
                metrics.worstReach = std::max(metrics.worstReach, reach);
        if (legacy.bodyMode != previousMode) {
            ++metrics.modeSwitches;
            metrics.trail += std::string(">") + modeName(legacy.bodyMode);
            if (dump != nullptr) std::fprintf(dump, "E %s\n", modeName(legacy.bodyMode));
        }
        previousMode = legacy.bodyMode;
        (legacy.bodyMode == LegacyBodyMode3D::Animated ? metrics.animatedSeconds
            : legacy.bodyMode == LegacyBodyMode3D::Physical ? metrics.physicalSeconds
            : legacy.bodyMode == LegacyBodyMode3D::Active ? metrics.activeSeconds : metrics.returningSeconds) += Dt;
        metrics.assistImpulse = athlete.activeOutput.assistImpulseTotal;
        for (std::size_t i = 0; i < bodyState.links.size() && i < profile.links.size(); ++i) {
            const PhysicsBodyState3D& link = bodyState.links[i];
            const std::string& id = profile.links[i].id;
            const Vec3 acceleration = (link.linearVelocity - previousVelocities[i]) * (1.0f / Dt);
            if (t > 0.35f) {
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (std::getenv("XPICO") && leg && acceleration.length() > std::atof(std::getenv("XPICO"))) {
                    const auto mode = [&](std::size_t side) { return "FLSP"[static_cast<int>(athlete.footTargets[side].mode)]; };
                    std::printf("  t %.3f %s acel %.0f | pes %c %c alcance %.3f %.3f\n", t, id.c_str(), acceleration.length(),
                        mode(0), mode(1), athlete.legSynthesis.reachError[0], athlete.legSynthesis.reachError[1]);
                }
                (leg ? metrics.peakLegAcceleration : metrics.peakUpperAcceleration) = std::max(
                    leg ? metrics.peakLegAcceleration : metrics.peakUpperAcceleration, acceleration.length());
                // Salto de pose: o link andou mais do que a velocidade dele explica.
                const Vec3 expected = previousLinks[i].position + link.linearVelocity * Dt;
                metrics.worstPoseJump = std::max(metrics.worstPoseJump, (link.position - expected).length());
            }
            previousVelocities[i] = link.linearVelocity;
            if ((id == "LeftFoot" || id == "RightFoot") && footGeometry.valid) {
                // O ponto que toca o chao (o mais baixo: calcanhar ou base dos
                // dedos) - o pe que rola gira em volta dele; a origem do link
                // anda ate com o pe parado no chao.
                const std::size_t side = id == "LeftFoot" ? 0 : 1;
                const Vec3 heel = link.position + link.orientation.rotate(footGeometry.heel[side]);
                const Vec3 toe = link.position + link.orientation.rotate(footGeometry.toeBase[side]);
                const Vec3 contact = heel.z < toe.z ? heel : toe;
                const Vec3 before = previousLinks[i].position
                    + previousLinks[i].orientation.rotate(heel.z < toe.z ? footGeometry.heel[side] : footGeometry.toeBase[side]);
                const bool planted = settings.footstepLocomotion
                    ? athlete.footTargets[side].mode == FootMode3D::Stance
                    : legacy.defaultLocomotion.output().footPlanted[side] && !legacy.defaultLocomotion.output().airborne;
                if (t > 0.35f && planted
                    && (legacy.bodyMode == LegacyBodyMode3D::Animated || legacy.bodyMode == LegacyBodyMode3D::Returning)) {
                    const float slide = std::hypot(contact.x - before.x, contact.y - before.y) / Dt;
                    metrics.plantedSlide.push_back(slide);
                    if (std::getenv("XDESLIZE") && slide > 0.3f)
                        std::printf("  t %.3f %s desliza %.2f m/s (%s) alcance %.3f\n", t, id.c_str(), slide,
                            heel.z < toe.z ? "calcanhar" : "dedos", athlete.legSynthesis.reachError[side]);
                }
            }
            if (id == "LeftFoot" || id == "RightFoot") {
                const auto& collider = profile.links[i].collider;
                const Vec3 center = link.position + link.orientation.rotate(collider.localPosition);
                if (t > 0.35f && center.z < collider.boxHalfExtents.z + 0.03f)
                    metrics.footSlide.push_back(std::hypot(link.linearVelocity.x, link.linearVelocity.y));
            }
        }
        previousLinks = bodyState.links;
        metrics.lowestPelvis = std::min(metrics.lowestPelvis, bodyState.links.front().position.z);
        if (bodyState.links.front().position.z < 0.5f) metrics.fell = true;
        if (scenario.ball) metrics.ballSpeedAfter = std::max(metrics.ballSpeedAfter, ballState.linearVelocity.length());
        if (dump != nullptr && step % every == 0) {
            dumpFrame(dump, profile, bodyState, t, static_cast<int>(legacy.bodyMode));
            if (scenario.ball)
                std::fprintf(dump, "K %.4f %.4f %.4f %.3f %.3f %.3f\n", ballState.position.x, ballState.position.y,
                    ballState.position.z, ballState.linearVelocity.x, ballState.linearVelocity.y,
                    ballState.linearVelocity.z);
        }
    }
    metrics.finalMode = legacy.bodyMode;
    metrics.worstLiftCorrection = athlete.footsteps.telemetry().worstLiftCorrection;
    metrics.settleSteps = athlete.footsteps.telemetry().settleSteps;
    metrics.earlyLifts = athlete.footsteps.telemetry().earlyLifts;
    metrics.catchSteps = athlete.footsteps.telemetry().catchSteps;
    metrics.startSteps = athlete.footsteps.telemetry().startSteps;
    return metrics;
}

void printMetrics(const Scenario& scenario, const Metrics& metrics, bool footsteps) {
    std::printf("%-18s | %5.2f / %5.2f / %5.2f | %7.1f / %7.1f m/s2 | %5.3f m | %d; %.1f/%.1f/%.1f/%.1f; %.0f | %s%d pedidos, %d toques, bola max %.1f m/s\n",
        scenario.name, percentile(metrics.footSlide, 0.5f), percentile(metrics.footSlide, 0.95f),
        percentile(metrics.footSlide, 1.0f), metrics.peakUpperAcceleration, metrics.peakLegAcceleration,
        metrics.worstPoseJump, metrics.modeSwitches, metrics.animatedSeconds, metrics.activeSeconds,
        metrics.physicalSeconds, metrics.returningSeconds, metrics.assistImpulse, metrics.fell ? "CAIU, " : "",
        metrics.requests, metrics.touches, metrics.ballSpeedAfter);
    std::printf("%-18s   apoio %5.2f / %5.2f / %5.2f m/s%s", "", percentile(metrics.plantedSlide, 0.5f),
        percentile(metrics.plantedSlide, 0.95f), percentile(metrics.plantedSlide, 1.0f), footsteps ? "" : "\n");
    if (footsteps)
        std::printf(" | saida do apoio max %.3f m | alcance max %.3f m | arranca %d, acomoda %d, antes %d, no ar %d\n",
            metrics.worstLiftCorrection, metrics.worstReach, metrics.startSteps, metrics.settleSteps, metrics.earlyLifts,
            metrics.catchSteps);
}

// Os contratos do runtime (ctest MatterEngine.Athlete): o que cada etapa do
// rework garante, medido na fisica de verdade.
int runChecks(const ProbeContext& context) {
    int failures = 0;
    const auto run = [&](const char* name, const AthleteRuntimeSettings3D& settings, bool startPhysical) {
        for (const Scenario& scenario : scenarios())
            if (std::string(scenario.name) == name) {
                const Metrics metrics = runScenario(context, scenario, settings, startPhysical, nullptr, 1);
                printMetrics(scenario, metrics, settings.footstepLocomotion);
                return metrics;
            }
        std::fprintf(stderr, "cenario sumiu: %s\n", name);
        ++failures;
        return Metrics {};
    };
    const auto expect = [&](bool ok, const char* what, const Metrics& metrics) {
        std::printf("  %s %s (%s)\n", ok ? "ok  " : "FALHOU", what, metrics.trail.c_str());
        if (!ok) ++failures;
    };
    const auto has = [](const Metrics& metrics, const char* mode) {
        return metrics.trail.find(mode) != std::string::npos;
    };
    const AthleteRuntimeSettings3D settings;
    AthleteRuntimeSettings3D legacyOnly;
    legacyOnly.activeBody = false;
    {
        const Metrics m = run("parado", settings, false);
        expect(m.modeSwitches == 0 && m.peakLegAcceleration < 5.0f, "parado: fica no padrao, quieto", m);
    }
    {
        const Metrics m = run("sprint", settings, false);
        expect(m.modeSwitches == 0, "sprint: fica no padrao", m);
    }
    {
        const Metrics m = run("caixa_leve", settings, false);
        expect(has(m, "ativo") && !has(m, "fisico"), "caixa leve: o Ativo segura, sem fisico pleno", m);
        expect(m.finalMode == LegacyBodyMode3D::Animated, "caixa leve: volta ao padrao", m);
        expect(m.assistImpulse < 5.0f, "caixa leve: quase sem ajuda de fora", m);
        expect(m.peakLegAcceleration < 300.0f && percentile(m.footSlide, 1.0f) < 1.0f,
            "caixa leve: sem estalo nas pernas nem pe escorregando", m);
    }
    {
        const Metrics m = run("caixa_leve", legacyOnly, false);
        expect(has(m, "fisico") && !has(m, "ativo"), "caixa leve sem o Ativo: o fisico de antes", m);
    }
    {
        const Metrics m = run("caixa_forte", settings, false);
        expect(has(m, "fisico"), "caixa forte: vai ao fisico pleno", m);
    }
    {
        const Metrics m = run("parado", settings, true);
        const std::size_t active = m.trail.find("ativo");
        expect(active != std::string::npos && m.trail.find("fisico", active) == std::string::npos
                && m.finalMode == LegacyBodyMode3D::Animated,
            "volta do fisico: pelo Ativo ate o padrao", m);
        expect(m.peakLegAcceleration < 300.0f, "volta do fisico: sem estalo na troca", m);
    }
    // Locomocao por pegadas (Etapa 3): o pe apoiado nao escorrega, sem
    // estalo, e as acoes com bola continuam saindo.
    AthleteRuntimeSettings3D footsteps;
    footsteps.footstepLocomotion = true;
    for (const char* name : { "arranca", "sprint", "inverte", "lados", "corte", "para_sprint", "diagonal", "costas" }) {
        const Metrics m = run(name, footsteps, false);
        char what[128];
        std::snprintf(what, sizeof(what), "pegadas, %s: o pe apoiado nao escorrega (p95 %.2f m/s)", name,
            percentile(m.plantedSlide, 0.95f));
        expect(percentile(m.plantedSlide, 0.95f) < 0.4f && percentile(m.plantedSlide, 1.0f) < 1.0f, what, m);
        std::snprintf(what, sizeof(what), "pegadas, %s: sem salto de pose (%.3f m)", name, m.worstPoseJump);
        expect(m.worstPoseJump < 0.03f, what, m);
    }
    {
        const Metrics m = run("zigue", footsteps, false);
        expect(percentile(m.plantedSlide, 0.95f) < 1.5f && m.worstPoseJump < 0.03f,
            "pegadas, zigue-zague: pouco deslize, sem salto de pose", m);
    }
    {
        const Metrics m = run("parado", footsteps, false);
        expect(m.settleSteps == 0 && percentile(m.plantedSlide, 1.0f) < 0.01f, "pegadas, parado: quieto", m);
    }
    for (const char* name : { "toque_parado", "chute_parado", "toque_corre" }) {
        const Metrics m = run(name, footsteps, false);
        char what[96];
        std::snprintf(what, sizeof(what), "pegadas, %s: a bola sai", name);
        expect(m.touches == 1 && m.ballSpeedAfter > 1.0f, what, m);
    }
    std::printf(failures == 0 ? "atleta: tudo certo\n" : "atleta: %d falha(s)\n", failures);
    return failures == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    std::string only, dumpPath;
    int every = 3;
    bool startPhysical = false;
    bool activeBody = true;
    bool footsteps = false;
    bool checks = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto need = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--cenario") only = need();
        else if (arg == "--dump") dumpPath = need();
        else if (arg == "--cada") every = std::max(1, std::atoi(need()));
        else if (arg == "--fisico") startPhysical = true;
        else if (arg == "--ativo") activeBody = std::atoi(need()) != 0;
        else if (arg == "--pegadas") footsteps = std::atoi(need()) != 0;
        else if (arg == "--teste") checks = true;
        else if (arg == "--lista") {
            for (const Scenario& scenario : scenarios()) std::printf("%s\n", scenario.name);
            return 0;
        } else {
            std::fprintf(stderr, "argumento desconhecido: %s\n", arg.c_str());
            return 2;
        }
    }
    const std::string assets = MATTERENGINE_ASSETS_DIR;
    const RagdollCharacter3D character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const RagdollProfile3D& profile = character.profile;
    std::vector<std::string> clipErrors;
    const std::vector<AnimationClip3D> clipLibrary =
        loadAthleteClipLibrary3D(assets + "/animations/clips", profile, &clipErrors);
    const CharacterLocomotionAnimations3D clips = athleteLocomotionClips3D(character, clipLibrary);
    const CharacterLocomotionAnimations3D defaultClips =
        defaultLocomotionAnimations3D(clips, character.defaultLocomotionClipIds, clipLibrary);
    const CharacterGaitSpeeds3D gaitSpeeds = characterGaitSpeeds3D(clips);
    CharacterMotorSettings3D avatar = athleteAvatarMotorSettings3D();
    if (gaitSpeeds.walk > 0.0f) {
        avatar.walkSpeed = gaitSpeeds.walk;
        avatar.sprintSpeed = gaitSpeeds.sprint;
    }
    AthleteRuntimeSettings3D settings;
    settings.activeBody = activeBody;
    settings.footstepLocomotion = footsteps;
    ProbeContext context;
    context.profile = &profile;
    context.gaitSpeeds = gaitSpeeds;
    context.avatar = avatar;
    context.runtimeAssets.profile = &profile;
    context.runtimeAssets.clips = &clips;
    context.runtimeAssets.defaultClips = &defaultClips;
    context.runtimeAssets.avatarSettings = avatar;
    if (checks) return runChecks(context);

    FILE* dump = dumpPath.empty() ? nullptr : std::fopen(dumpPath.c_str(), "w");
    std::printf("cenario            | deslize pe no chao med/p95/max m/s | pico acel tronco+bracos / pernas | salto pose | modos (trocas; animado/ativo/fisico/volta s; ajuda N.s) | bola\n");
    for (const Scenario& scenario : scenarios()) {
        if (!only.empty() && scenario.name != only) continue;
        printMetrics(scenario, runScenario(context, scenario, settings, startPhysical, dump, every), settings.footstepLocomotion);
    }
    if (dump != nullptr) std::fclose(dump);
    return 0;
}
