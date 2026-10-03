#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Character/CharacterControlApplication3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Control/CharacterWholeBody3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <stdexcept>

using namespace MatterEngine;
namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
// Pelve e pes, como no laboratorio.
void applyAdaptive(PhysicsScene3D& scene, RagdollHandle3D handle,
    const AdaptivePhysicalOutput3D& physical) {
    scene.applyRagdollControlRootForce(handle, physical.rootForceWorld, physical.rootTorqueWorld);
    for (std::size_t side = 0; side < 2; ++side) {
        if (physical.footLinkIndex[side] == RagdollDynamics3D::InvalidIndex) continue;
        scene.applyRagdollControlLinkForce(handle, physical.footLinkIndex[side],
            physical.footForceWorld[side], physical.footTorqueWorld[side]);
    }
    for (std::size_t side = 0; side < 2; ++side) {
        if (physical.handLinkIndex[side] == RagdollDynamics3D::InvalidIndex) continue;
        scene.applyRagdollControlLinkForce(handle, physical.handLinkIndex[side],
            {}, physical.handTorqueWorld[side]);
    }
}
void baseline(float speed) {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const auto& profile = character.profile;
    const auto idle = loadAnimationClip3D(assets + "/animations/clips/" + (std::getenv("XIDLE") ? std::string(std::getenv("XIDLE")) : character.idleClipId) + ".matteranim.json");
    const auto jog = loadAnimationClip3D(assets + "/animations/clips/" + character.walkClipId + ".matteranim.json");
    const auto sprint = loadAnimationClip3D(assets + "/animations/clips/" + character.sprintClipId + ".matteranim.json");
    CharacterLocomotionAnimations3D clips;
    clips.idle = &idle;
    clips.walk = &jog;
    clips.sprint = &sprint;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    PhysicsBodyDefinition3D floor;
    floor.motionType = PhysicsMotionType3D::Static;
    floor.position = {0, 0, -0.1f};
    floor.materialId = "concrete";
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = {100, 10, 0.1f};
    shape.materialId = "concrete";
    const auto floorHandle = scene->createBody(floor, std::span<const PhysicsShape3D>(&shape, 1));
    require(static_cast<bool>(floorHandle), "Missing floor");
    RagdollSpawnDefinition3D spawn;
    spawn.pelvisPosition = {0, 0, profile.standingRootHeightMeters};
    const auto handle = scene->createRagdoll(profile, spawn);
    CharacterLocomotion3D locomotion;
    locomotion.reset(profile, scene->ragdollState(handle));
    AdaptivePhysicalCharacter3D controller;
    controller.reset(profile);
    if (const char* v = std::getenv("XCORR")) controller.settings().correctionMode = std::atoi(v);
    if (const char* v = std::getenv("XPOST")) controller.settings().postureMode = std::atoi(v);
    if (const char* v = std::getenv("XVMC")) {
        // 1 peso pelas pernas, +2 equilibrio planar, +4 postura (parado).
        const int mode = std::atoi(v);
        controller.settings().jointSupport = (mode & 1) != 0;
        controller.settings().jointBalance = (mode & 2) != 0;
        controller.settings().jointPosture = (mode & 4) != 0;
        controller.settings().jointWhileWalking = (mode & 8) != 0;
        locomotion.footworkSettings().locomotionStepping = (mode & 16) != 0;
        locomotion.footworkSettings().recoveryCaptureTargeting = (mode & 2) != 0;
    }
    if (const char* v = std::getenv("XCAPT")) locomotion.footworkSettings().recoveryCaptureTargeting = std::atoi(v) != 0;
    if (const char* v = std::getenv("XTILTKG")) controller.settings().maximumTiltTorquePerKg = std::strtof(v, nullptr);
    float maxDrift = 0, lowest = 100, maxTilt = 0;
    float worstPosition = 0, worstAngle = 0, sumPosition = 0;
    int tracked = 0;
    int supportTicks = 0, fallingTicks = 0;
    constexpr float dt = 1.0f / 120;
    Vec3 navigation = spawn.pelvisPosition;
    float velocity = 0;
    for (int tick = 0; tick < 600; ++tick) {
        const auto state = scene->ragdollState(handle);
        CharacterLocomotionInput3D input;
        if (tick > 60) velocity = std::min(speed, velocity + 14 * dt);
        navigation.x += velocity * dt;
        input.rootPositionWorld = navigation;
        input.rootVelocityWorld = input.proxyVelocityWorld = {velocity, 0, 0};
        input.sprinting = speed > 4;
        input.grounded = true;
        input.controlled = true;
        input.forceDrivenRoot = true;
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            const auto& id = profile.links[i].id;
            const int side = id == "LeftFoot" ? 0 : id == "RightFoot" ? 1 : -1;
            if (side >= 0) input.footGround[side] = scene->probeGround(
                state.links[i].position + Vec3{0,0,0.1f}, 0.05f, 0, 0.4f, 48);
        }
        locomotion.update(profile, clips, state, {}, input, dt);
        const auto& pose = locomotion.output();
        require(pose.targetPose.linkPositions.size() == profile.links.size(), "Pose generator returned no reference");
        AdaptivePhysicalIntent3D intent;
        intent.navigationRootWorld = input.rootPositionWorld;
        intent.proxyVelocityWorld = input.proxyVelocityWorld;
        intent.footGround = input.footGround;
        intent.physicalState = &locomotion.physicalState();
        intent.contactPlan = &locomotion.contactPlan();
        controller.update(profile, state, pose.guide, intent, dt);
        const auto& physical = controller.output();
        if (std::getenv("XTRACE") && tick % 12 == 0) {
            std::cout << "t=" << tick * dt << " z=" << state.links[0].position.z
                      << " x=" << state.links[0].position.x << " y=" << state.links[0].position.y << " tilt=" << physical.bodyTiltRadians
                      << " sup=" << physical.supportAuthority << " cap=" << physical.captureErrorMeters
                      << " feet L " << state.links[14].position.x << "," << state.links[14].position.y << "," << state.links[14].position.z
                      << " R " << state.links[17].position.x << "," << state.links[17].position.y << "," << state.links[17].position.z
                      << " planted " << locomotion.telemetry().footPlanted[0] << locomotion.telemetry().footPlanted[1]
                      << " ancoraL " << locomotion.contactPlan().feet[0].anchorWorld.x << "," << locomotion.contactPlan().feet[0].anchorWorld.y
                      << " fase " << static_cast<int>(locomotion.contactPlan().feet[0].phase) << static_cast<int>(locomotion.contactPlan().feet[1].phase)
                      << " razao " << static_cast<int>(locomotion.contactPlan().feet[0].reason) << static_cast<int>(locomotion.contactPlan().feet[1].reason)
                      << " inalc " << locomotion.telemetry().footworkAnchorUnreachable[0] << locomotion.telemetry().footworkAnchorUnreachable[1]
                      << " rec " << locomotion.contactPlan().recovering << "\n";
        }
        if (tick % 120 == 0) std::cout << "speed=" << speed << " t=" << tick * dt
            << " z=" << state.links[0].position.z << " target=" << pose.guide.rootPositionWorld.z
            << " support=" << physical.supportCount << " Fz=" << physical.rootForceWorld.z
            << " tilt=" << physical.bodyTiltRadians << " blend=" << pose.physicsBlend
            << " phase=" << static_cast<int>(physical.phase) << std::endl;
        require(physical.valid, "Adaptive output invalid");
        if (physical.supportAuthority <= 0.0f) require(physical.rootForceWorld.length() == 0,
            "Root force without physical support");
        // Pelve: intencao do jogador (ate maximumPlanarAcceleration) mais a
        // correcao de desvio (ate o atrito).
        require(std::hypot(physical.rootForceWorld.x, physical.rootForceWorld.y)
            <= profile.totalMassKg * (controller.settings().maximumPlanarAcceleration
                + controller.settings().frictionAccelerationLimit) + 0.01f,
            "Planar force exceeded budget");
        {
            Vec3 feet = physical.footForceWorld[0] + physical.footForceWorld[1];
            require(std::hypot(feet.x, feet.y)
                <= profile.totalMassKg * controller.settings().frictionAccelerationLimit + 0.01f,
                "Foot traction exceeded friction budget");
        }
        require(pose.guide.rootTranslationAuthority == 0.0f && pose.guide.rootRotationAuthority == 0.0f
            && pose.guide.poseAuthority == 0.0f, "No modo fisico o guia escreveu transform");
        // A mesma aplicacao do jogo (motores, torque das pernas, forcas).
        applyCharacterControl3D(*scene, handle, pose,
            std::getenv("XNOASSIST") ? nullptr : &physical, false);
        scene->simulate(dt);
        if (tick > 120) {
            const auto root = scene->ragdollState(handle).links.front();
            // Pelve fisica contra a referencia do gerador de pose.
            const float error = (root.position - pose.guide.rootPositionWorld).length();
            auto q = (pose.guide.rootOrientationWorld * root.orientation.conjugate()).normalized();
            const float angle = 2 * std::acos(std::min(1.0f, std::abs(q.w))) * 57.2958f;
            worstPosition = std::max(worstPosition, error);
            worstAngle = std::max(worstAngle, angle);
            sumPosition += error;
            ++tracked;
            // Deriva so depois de a base inicial se refazer (dois passos de
            // acomodacao com troca de peso, ate ~1,3 s): a pelve vai de
            // proposito ate ~13 cm na direcao do pe de apoio. O que se mede
            // aqui e se a assistencia deixa o corpo vagar parado.
            if (tick > 180)
                maxDrift = std::max(maxDrift, std::hypot(root.position.x - navigation.x, root.position.y));
            lowest = std::min(lowest, root.position.z);
            maxTilt = std::max(maxTilt, physical.bodyTiltRadians);
            if (physical.supportAuthority > 0.99f) ++supportTicks;
            if (physical.phase == PhysicalControlPhase3D::Falling) ++fallingTicks;
        }
    }
    std::cout << "Floating baseline " << speed << ": drift " << maxDrift << " m, minimum height "
        << lowest << " m, tilt " << maxTilt << " rad, support ticks " << supportTicks
        << ", falling ticks " << fallingTicks << " | pelve x referencia: media "
        << sumPosition / std::max(1, tracked) * 1000 << " mm, pior " << worstPosition * 1000
        << " mm, " << worstAngle << " graus" << std::endl;
    require(fallingTicks == 0 && lowest > profile.standingRootHeightMeters * 0.70f
        && maxDrift < 0.12f, "Floating baseline is not ready");
    // Sem nenhum pe tocando nada, a assistencia atravessa o voo curto da
    // passada e depois some: nenhuma forca na pelve e so o torque de ar,
    // mesmo com a pose de pe sobre um chao de navegacao valido.
    auto airborne = scene->ragdollState(handle);
    airborne.contacts.clear();
    airborne.interactions.clear();
    AdaptivePhysicalIntent3D intent;
    intent.airborne = true;
    intent.navigationRootWorld = spawn.pelvisPosition;
    const float memory = controller.settings().supportMemorySeconds;
    for (float t = 0; t <= 2 * memory + dt; t += dt)
        controller.update(profile, airborne, locomotion.output().guide, intent, dt);
    require(controller.output().supportAuthority == 0
        && controller.output().rootForceWorld.length() == 0
        && controller.output().rootTorqueWorld.length()
            <= profile.totalMassKg * controller.settings().maximumAirTorquePerKg + 0.01f,
        "Aerial control exceeded its budget");
}
}

// ============================================================================
// Cenarios no laco do jogo (modo fisico adaptativo), com a capsula real: o
// mesmo que WorkbenchApp::updateActiveRagdolls faz com a opcao ligada.
// ============================================================================
struct ClipSet {
    std::vector<AnimationClip3D> storage;
    CharacterLocomotionAnimations3D animations;
};

ClipSet loadClips(const RagdollCharacter3D& character) {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    ClipSet set;
    const std::string ids[] = {character.idleClipId, character.walkClipId,
        character.walkBackwardClipId, character.sprintClipId, character.sprintBackwardClipId,
        character.strafeLeftClipId, character.strafeRightClipId,
        character.sprintStrafeLeftClipId, character.sprintStrafeRightClipId,
        character.jumpStandingClipId, character.jumpForwardClipId,
        character.jumpBackwardClipId, character.jumpLeftClipId, character.jumpRightClipId,
        character.standUpBackClipId, character.standUpFrontClipId,
        character.idleToSprintClipId, character.runForwardArcLeftClipId,
        character.runForwardArcRightClipId, character.runBackwardArcRightClipId};
    set.storage.reserve(std::size(ids));
    for (const auto& id : ids)
        set.storage.push_back(id.empty() ? AnimationClip3D{}
            : loadAnimationClip3D(assets + "/animations/clips/" + id + ".matteranim.json"));
    auto at = [&](std::size_t i) { return set.storage[i].id.empty() ? nullptr : &set.storage[i]; };
    auto& a = set.animations;
    a.idle = at(0); a.walk = at(1); a.walkBackward = at(2); a.sprint = at(3);
    a.sprintBackward = at(4); a.strafeLeft = at(5); a.strafeRight = at(6);
    a.sprintStrafeLeft = at(7); a.sprintStrafeRight = at(8); a.jumpStanding = at(9);
    a.jumpForward = at(10); a.jumpBackward = at(11); a.jumpLeft = at(12); a.jumpRight = at(13);
    a.standUpBack = at(14); a.standUpFront = at(15);
    a.idleToSprint = at(16); a.runForwardArcLeft = at(17);
    a.runForwardArcRight = at(18); a.runBackwardArcRight = at(19);
    return set;
}

PhysicsBodyHandle3D staticBox(PhysicsScene3D& scene, Vec3 position, Vec3 half) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = half;
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = position;
    body.materialId = "concrete";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

PhysicsBodyHandle3D dynamicBox(PhysicsScene3D& scene, Vec3 position, Vec3 half,
    float mass, Vec3 velocity) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = half;
    shape.materialId = "wood";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Dynamic;
    body.position = position;
    body.linearVelocity = velocity;
    body.massKg = mass;
    body.materialId = "wood";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

struct ScenarioCommand {
    Vec3 move;          // direcao no mundo (0 = parado)
    bool sprint = false;
    bool jump = false;               // soltou o pulo neste tick (o pulo sai ao soltar)
    float jumpChargeSeconds = 0;     // quanto tempo ficou segurado
    bool jumpCharging = false;       // segurando (agacha, preparando)
    float lookYaw = 0;  // para onde a camera aponta
};

struct Scenario {
    const char* name;
    float seconds = 4;
    std::function<void(PhysicsScene3D&)> build;
    std::function<ScenarioCommand(float)> script;
    // Chamado a cada tick antes de simular (empurroes, projeteis).
    std::function<void(PhysicsScene3D&, float, const RagdollState3D&)> disturb;
    // Outro boneco parado, no modo fisico, com a referencia no proprio
    // corpo (como os nao controlados no laboratorio).
    bool npc = false;
    Vec3 npcPosition {};
    float npcYaw = 0;
    // Nasce deitado: inclinacao da pelve (graus, + de bruços) e altura.
    float spawnPitchDegrees = 0;
    float spawnPelvisHeight = 0;
    // Modo antigo (raiz carregada; queda por impacto).
    bool legacy = false;
    // Arrastado pela PhysGun (o mesmo D6 do laboratorio) por este link, a
    // partir de `dragStart`, com velocidade constante do alvo por
    // `dragSeconds`. Vazio: nada.
    const char* dragLink = nullptr;
    float dragStart = 1.0f;
    float dragSeconds = 2.0f;
    // Forca continua como a do laboratorio (Ragdoll impulsos, impulso
    // continuo): a partir de 1 s, na direcao relativa ao corpo (0 frente, 90
    // esquerda), 20% na pelve, 32% no link 2 e 48% no link 3.
    float continuousForceNewtons = 0.0f;
    float continuousForceDegrees = 90.0f;
    // Janela da forca (s): comeca em forceStart e dura forceSeconds (um
    // empurrao no meio da marcha).
    float forceStart = 1.0f;
    float forceSeconds = 1.0e9f;
    Vec3 dragVelocity {};
    // Outro boneco nascendo deitado: inclinacao (graus, + de bruços, em torno
    // do lado dele), rolagem (de lado) e altura da pelve.
    float npcSpawnPitchDegrees = 0;
    float npcSpawnRollDegrees = 0;
    float npcSpawnPelvisHeight = 0;
    // Pressao continua: para onde o empurrao leva o corpo (unitario, no
    // plano). Com ele, o relatorio mede a reacao dos pes e a inclinacao
    // contra a forca.
    Vec3 pushDirection {};
    // Obstaculo que os pes nao veem: a sonda de terreno so ve o estatico
    // (para os cenarios de queda por tropeco - a caixa dinamica no chao,
    // vista, o passo passa por cima).
    bool unseenObstacles = false;
};

// Um passo do planejador, da soltura ao pouso (etapa E3: o pe descarrega,
// sai do chao, anda e pousa por contato medido).
struct PlannerStepTrace {
    std::uint32_t stepId = 0;
    int side = 0;
    StepReason3D reason = StepReason3D::None;      // na soltura
    StepReason3D finalReason = StepReason3D::None; // no pouso
    float releasedAt = 0;
    float landedAt = -1;
    bool releasedUnloaded = false;
    float maxSoleLift = 0;      // m, sola acima do chao de onde saiu
    float travel = 0;           // m no plano, soltura -> pouso (medido)
    float worstTargetJump = 0;  // m/s, maior mudanca da velocidade do alvo num tick no ar
    float searchSeconds = 0;    // procurando o chao depois do tempo do passo
    ContactEvidence3D evidence = ContactEvidence3D::None;
    bool early = false;
    bool dragged = false;
    Vec3 liftoff;
    Vec3 touchdown;
};

struct ScenarioResult {
    int falls = 0;
    float firstFallSeconds = -1;
    float worstTilt = 0;          // graus, pelve contra a vertical
    float worstBodyCapsule = 0;   // m, pelve fisica x pelve de referencia (plano)
    float meanTracking = 0;       // m
    float worstLinkSpeed = 0;
    float finalTilt = 0;
    float finalChestTilt = 0;
    int bothFeetAirTicks = 0;
    int steps = 0;                // pes que sairam do chao parado
    Vec3 finalBody;
    Vec3 finalCapsule;
    int npcFalls = 0;
    Vec3 npcFinal;
    // Outro boneco levantando: maior distancia ancora x pelve (plano), maior
    // velocidade da referencia (guia) e, no 1 s depois de cada levantar, o
    // membro mais rapido; quantas vezes levantou e se termina de pe.
    float npcAnchorErrorPeak = 0, npcGuideSpeedPeak = 0, npcAfterGetUpLinkPeak = 0;
    int npcGetUps = 0;
    bool npcStandingAtEnd = false;
    // Queda e levantar.
    float headImpactSpeed = 0;      // m/s da cabeca no primeiro toque no chao
    std::string firstGroundLink;    // primeiro segmento (fora os pes) a tocar o chao caindo
    float getUpSeconds = -1;        // da queda ate estar em pe de novo
    float getUpWorstRootSpeed = 0;  // pelve, no levantar
    float getUpWorstRootStep = 0;   // maior salto da pelve num tick (teleporte)
    float getUpWorstLinkSpeed = 0;
    float getUpAssistPeakShare = 0; // forca auxiliar de pico / m.g
    float getUpAssistMeanShare = 0;
    int relapses = 0;               // quedas depois de levantar
    bool standingAtEnd = false;
    // Cambalhotas: quanto a pelve gira (em torno de eixos horizontais) do
    // chao ate comecar a levantar.
    float settleTumbleDegrees = 0;
    float settleGatherDegrees = 0;
    // Troca do levantar para o controlador normal: pe mais rapido e quanto a
    // base (distancia entre os pes) abre, nos 0,6 s depois.
    float handoffFootSpeed = 0;
    float handoffSpreadGrowth = 0;
    // Forca das juntas que chega ao levantar (media dos links, 1 = inteira).
    float getUpJointStrength = 1;
    // Levantando ja alto (pelve > 0,6 m): o maior excesso de inclinacao
    // sobre o clipe com a ajuda de endireitar agindo (graus) - a prancha
    // "segurada" - e a distancia do centro de massa ao centro dos pes (m).
    float getUpHeldLean = 0;
    float getUpComOutside = 0;
    float getUpHeldLeanSeconds = 0;  // tempo com a pelve alta > 15 graus alem do clipe
    // Depois de levantar: inclinacao da pelve e do peito na troca e a maior
    // nos 2 s seguintes (graus) - "torto ate encaixar" e quase-quedas.
    float handoffTilt = -1;
    float handoffChestTilt = -1;
    float afterGetUpWorstTilt = 0;
    float worstUpwardSpeed = 0;
    // Erro de inclinacao da pelve contra a pose (graus), de pe: media e pior.
    double tiltErrorSum = 0;
    int tiltErrorCount = 0;
    float tiltErrorWorst = 0;
    int tiltErrorOver15 = 0;
    float balanceReactionPeak = 0;   // reacao de desequilibrio (0..1)
    float captureOutsidePeakAfterPush = 0; // ponto de captura fora do apoio, 1,5-1,8 s (m)
    // Marcha (E5): velocidade pedida e alcancada no rumo pedido, tempo ate 80%
    // dela, maior desvio de rumo do corpo andando, e quanto demorou a parar.
    float gaitRequestedSpeed = 0;
    float gaitBestSpeed = 0;
    float gaitTimeTo80 = -1;
    float gaitWorstYawDegrees = 0;
    float gaitMoveStart = -1, gaitMoveEnd = -1;
    float gaitStoppedAt = -1;
    // Regime (de 1 s depois de comecar ate o fim do pedido): media da
    // velocidade no rumo pedido e atravessada (com sinal: + a esquerda do
    // rumo), e a fase da queda (0 nenhuma, 1 arrancada, 2 regime, 3 parada).
    double gaitSustainedAlong = 0, gaitSustainedAcross = 0;
    int gaitSustainedSamples = 0;
    int gaitFallPhase = 0;
    // Inversao (XGAITINVERT): quando o pedido virou de sentido e quando o
    // corpo passou a ir a metade da nova velocidade no novo rumo.
    float gaitReverseAt = -1, gaitReversedAt = -1;
    // Telemetria do motor (XMOTORSUM): por junta (quadril/joelho/tornozelo
    // esquerdo e direito) e papel do pe (apoio / no ar).
    struct MotorSummary { int ticks = 0; int saturatedTicks = 0; float maxSaturation = 0;
        double sumAbsMotor = 0, sumAbsFeedforward = 0; };
    std::array<std::array<MotorSummary, 2>, 6> motorSummary {};
    // Transicoes da marcha (XTRANSITION, pacote do GPT doc 03 s6): velocidade
    // do COM no rumo pedido 0,1 s antes do toque do pe que vem (v0), no toque
    // (v1), 0,05 s depois (v2), na aceitacao da carga (v3) e na saida do
    // outro pe (v4). Somas para a media.
    int transitions = 0;
    double pushGain = 0, collisionLoss = 0, acceptanceLoss = 0, transitionNet = 0, earlySingleLoss = 0;
    // Balanco da marcha (XSWING, pacote do GPT fase 10): por passo de marcha
    // entre 2,5 e 5 s, quanto o alvo do pouso andou depois de 35% do
    // balanco (e recuou contra o rumo pedido), o pico de aceleracao no plano
    // do alvo do pe (um salto de velocidade vira um pico enorme), o erro
    // medio no plano do pe fisico ao alvo e o erro no pouso. Somas.
    int swings = 0, swingsReceding20 = 0;
    double swingRevision = 0, swingRecede = 0, swingAccelPeak = 0, swingTracking = 0, swingLandingError = 0;
    float swingRecedeWorst = 0, swingAccelWorst = 0;
    // Andando (2,5-5 s): ticks com os dois pes fora do chao (voo) e com um pe
    // de apoio no plano (apoio, descarga, carga) fora do chao (apoio ficticio).
    int flightTicks = 0, fictitiousSupportTicks = 0;
    // R1 (pacote v2): pelo apoio observado estrito (contactObserved), voo e
    // apoio ficticio por fase planejada (0 apoio, 1 descarga, 2 carga).
    int strictFlightTicks = 0;
    std::array<int, 3> strictFakeTicks {};
    double releaseJump = 0;   // soma de |pe medido na soltura - ancora| no plano (m)
    Vec3 gaitLastRequest {};
    Vec3 pelvisAt15 {};
    bool pelvisAt15Set = false;
    float standingTravelAfter15 = 0;       // pelve no plano, maior distancia de onde estava em 1,5 s (m)
    // Tremida (depois de 1,5 s): RMS e pico da velocidade vertical da pelve
    // (m/s) e quantas vezes um pe apoiado perdeu o contato (evento estrito).
    double idleVerticalSquared = 0;
    int idleVerticalSamples = 0;
    float idleVerticalPeak = 0;
    int idleContactLosses = 0;
    std::array<bool, 2> idleContactBefore { true, true };
    // Giro parado (varredura sweepturn): quando a pelve fisica ficou a menos
    // de 20 graus do olhar pela primeira vez depois de 1,5 s (s; -1 nunca) e
    // a maior inclinacao do tronco depois de 1,5 s (graus).
    float turnAlignedAt = -1;
    float turnWorstTilt = 0;
    // Rumo da pelve fisica e do olhar a cada tick depois de 1,5 s (rad).
    std::vector<std::pair<float, float>> turnYaw;
    // Pelve fisica (x) a cada tick: a velocidade atravessando um obstaculo.
    std::vector<std::pair<float, float>> pelvisX;
    // Toques do pe (sweepterrain): instante, x do pe e o angulo da sola contra
    // o chao embaixo dela no toque (graus; de frente + ponta para baixo, de
    // lado + borda esquerda para baixo).
    struct Touchdown { float time, x, pitch, roll, ankleError, progress; };
    std::vector<Touchdown> touchdowns;
    std::array<bool, 2> touchdownContact { true, true };
    std::array<float, 2> touchdownAirSeconds {};
    int touchdownScuffs = 0;   // contato de novo com menos de 0,10 s no ar
    std::array<float, 2> lastAir {};
    int balanceReactionTicks = 0;    // ticks com reacao > 0,3
    // Pouso: ticks com os dois pes no ar depois do toque (o quique antigo
    // erguia os dois a 30 cm) e a menor altura da pelve (agachou?).
    int landingBounceTicks = 0;
    float landingRebound = 0;
    // Pe arrastando: fora do apoio (no balanco) mas tocando o chao e
    // deslizando (s, por pe), e a maior altura de cada pe no balanco (m).
    std::array<float, 2> dragSeconds {};
    // Rumo da pelve contra a camera, andando (graus): no strafe puro ela
    // fica de frente; o giro para a diagonal na troca aparece aqui.
    float facingOffLookPeak = 0;
    // Ainda de pe (nao caido), com o reflexo de queda ativo (> 0,5): o "modo
    // rigido de queda" em que as pernas param de dar passos (s).
    float rigidSeconds = 0;
    float armsReflexSeconds = 0;
    // O quanto ele parece ragdoll: largado deitado (tonus baixo, antes de se
    // arrumar), a altura media das maos e o afastamento das juntas da pose
    // pedida; caindo sem volta, o mesmo afastamento.
    float restSeconds = 0, restHandHeight = 0, restJointError = 0, restHeadHeight = 0;
    float fallingSeconds = 0, fallingJointError = 0;
    // Pressao/empurrao: tempo sob contato externo (perturbacao > 0,5), tempo
    // desequilibrado (ponto de captura > 6 cm fora do apoio) com os dois pes
    // parados no chao - o "inclina sem dar passo" -, do primeiro contato ao
    // primeiro pe saindo do chao, pe "travado" escorregando, passos durante a
    // pressao e quanto o peito inclina contra a forca (graus, + = contra).
    float pressSeconds = 0, leanWithoutStepSeconds = 0, firstPressAt = -1, firstStepAfterPress = -1;
    float plantedSlideSeconds = 0, leanIntoSum = 0;
    // Segurado fora da base: parado (sem comando), o centro de massa mais de
    // 3 cm fora dos pes e o corpo sem tombar (inclinacao mudando menos de
    // 20 graus/s) - a pose que so uma forca invisivel sustenta.
    float heldOutsideSeconds = 0, previousTiltForHold = -1;
    // Arrastado: passos (pe saindo do chao), tempo com pe no chao escorregando
    // (> 0,25 m/s, os dois pes somados / 2) e quanto o corpo andou.
    int dragSteps = 0;
    float dragSlideSeconds = 0, dragDistance = 0;
    bool dragged = false;
    int pressSteps = 0;
    std::array<float, 2> swingPeak {};
    float highestPelvis = 0;     // pulo: pelve mais alta (m)
    float highestCapsuleFeet = 0; // e a base da capsula mais alta (m)   // maior subida da pelve nos 0,3 s depois do toque (m/s)
    // Inversao de direcao: da ordem ate o corpo andar a 1 m/s no sentido
    // novo (s), e a maior inclinacao da pelve nesse intervalo.
    float reverseSeconds = -1;
    float reverseWorstTilt = 0;
    float landingLowestPelvis = 100;
    bool landed = false;
    float worstVerticalStep = 0;
    // Planejador de passos no fim do cenario (contadores acumulados), cada
    // passo dele e as violacoes de contrato: pouso sem contato medido, pe que
    // voltou a apoiar sem passar pela carga, e o pe marcado apoiado diferente
    // da fase do plano (dois donos).
    CharacterContactPlan3D plan;
    std::uint32_t bodyReferenceTicks = 0;   // R0: referencia corporal ativa (ticks)
    std::vector<PlannerStepTrace> plannerSteps;
    int touchdownWithoutContact = 0;
    int skippedLoading = 0;
    int ownerMismatch = 0;
};

// Quanto um ponto (plano) esta FORA do fecho convexo dos pontos de apoio (m;
// 0 dentro). Com 1-2 pontos, a distancia ao ponto/segmento.
float outsideSupport(std::vector<std::array<float, 2>> points, float x, float y) {
    if (points.empty()) return 10.0f;
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    const auto cross = [](const std::array<float, 2>& o, const std::array<float, 2>& a,
        const std::array<float, 2>& b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    std::vector<std::array<float, 2>> hull;
    if (points.size() >= 3) {
        for (int pass = 0; pass < 2; ++pass) {
            const std::size_t start = hull.size();
            for (std::size_t i = 0; i < points.size(); ++i) {
                const auto& p = pass == 0 ? points[i] : points[points.size() - 1 - i];
                while (hull.size() >= start + 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0)
                    hull.pop_back();
                hull.push_back(p);
            }
            hull.pop_back();
        }
    } else {
        hull = points;
    }
    const auto segment = [&](const std::array<float, 2>& a, const std::array<float, 2>& b) {
        const float dx = b[0] - a[0], dy = b[1] - a[1];
        const float len = dx * dx + dy * dy;
        const float u = len > 1e-9f ? std::clamp(((x - a[0]) * dx + (y - a[1]) * dy) / len, 0.0f, 1.0f) : 0.0f;
        return std::hypot(x - (a[0] + u * dx), y - (a[1] + u * dy));
    };
    if (hull.size() == 1) return std::hypot(x - hull[0][0], y - hull[0][1]);
    bool inside = hull.size() >= 3;
    float nearest = 1e9f;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto& a = hull[i];
        const auto& b = hull[(i + 1) % hull.size()];
        if (cross(a, b, {x, y}) < 0) inside = false;
        nearest = std::min(nearest, segment(a, b));
    }
    return inside ? 0.0f : nearest;
}

ScenarioResult runScenario(const Scenario& scenario) {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const auto& profile = character.profile;
    const ClipSet clips = loadClips(character);
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    // Diagnostico (criterio da E4: menos atrito, menos capacidade): XFRICTION
    // escala o atrito do chao.
    if (const char* v = std::getenv("XFRICTION")) {
        if (const SurfaceMaterial* concrete = materials.find("concrete")) {
            SurfaceMaterial slippery = *concrete;
            const float scale = std::strtof(v, nullptr);
            slippery.contact.staticFriction *= scale;
            slippery.contact.dynamicFriction *= scale;
            materials.registerMaterial(slippery);
        }
    }
    auto scene = engine.createScene({}, materials);
    staticBox(*scene, {0, 0, -0.1f}, {60, 60, 0.1f});
    if (scenario.build) scenario.build(*scene);
    const CharacterGaitSpeeds3D speeds = characterGaitSpeeds3D(clips.animations);
    CharacterMotorSettings3D settings;
    settings.radius = 0.34f;
    settings.standingHeight = 1.83f;
    settings.crouchedHeight = 1.22f;
    settings.walkSpeed = speeds.walk;
    settings.sprintSpeed = speeds.sprint;
    settings.groundAcceleration = 14.0f;
    settings.groundDeceleration = 18.0f;
    settings.airAcceleration = 4.5f;
    settings.jumpSpeed = 4.6f;
    settings.maximumSlopeDegrees = 50.0f;
    settings.maximumStepHeight = 0.19f;
    settings.skinWidth = 0.018f;
    scene->createCharacter({0, 0, 0}, settings);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9101;
    spawn.active = true;
    spawn.pelvisPosition = {0, 0, scenario.spawnPelvisHeight > 0
        ? scenario.spawnPelvisHeight : profile.standingRootHeightMeters};
    if (scenario.spawnPitchDegrees != 0)
        spawn.orientation = Quaternion::fromAxisAngle({0, 1, 0},
            scenario.spawnPitchDegrees * 3.14159265f / 180);
    const auto handle = scene->createRagdoll(profile, spawn);
    CharacterLocomotion3D locomotion;
    locomotion.reset(profile, scene->ragdollState(handle));
    // E5-A (diagnostico): seguir um pe do comeco ao fim do passo - as quatro
    // posicoes do pe, a carga e as juntas da perna, capturadas em memoria e
    // impressas no fim (janela do primeiro passo de marcha desse pe).
    const int stepTraceSide = std::getenv("XSTEPTRACE") ? std::atoi(std::getenv("XSTEPTRACE")) : -1;
    if (stepTraceSide >= 0) locomotion.setFootTraceEnabled(true);
    struct StepTraceSample {
        float t; int phase; int reason; std::uint32_t stepId; float load; int supporting; int evidence; int memory;
        FootTrace3D trace;
        float hipTarget, hipMeasured, kneeTarget, kneeMeasured, kneeVelocityTarget, stiffness, torque;
        LegReachTelemetry3D reach;
        int otherPhase; int supportingStrict;
    };
    std::vector<StepTraceSample> stepTrace;
    stepTrace.reserve(4096);
    AdaptivePhysicalCharacter3D controller;
    controller.reset(profile);
    if (const char* v = std::getenv("XCORR")) controller.settings().correctionMode = std::atoi(v);
    if (const char* v = std::getenv("XPOST")) controller.settings().postureMode = std::atoi(v);
    if (const char* v = std::getenv("XVMC")) {
        // 1 peso pelas pernas, +2 equilibrio planar, +4 postura (parado).
        const int mode = std::atoi(v);
        controller.settings().jointSupport = (mode & 1) != 0;
        controller.settings().jointBalance = (mode & 2) != 0;
        controller.settings().jointPosture = (mode & 4) != 0;
        controller.settings().jointWhileWalking = (mode & 8) != 0;
        locomotion.footworkSettings().locomotionStepping = (mode & 16) != 0;
        locomotion.footworkSettings().recoveryCaptureTargeting = (mode & 2) != 0;
    }
    if (const char* v = std::getenv("XCAPT")) locomotion.footworkSettings().recoveryCaptureTargeting = std::atoi(v) != 0;
    if (const char* v = std::getenv("XTILTKG")) controller.settings().maximumTiltTorquePerKg = std::strtof(v, nullptr);
    if (std::getenv("XPLACE")) locomotion.footworkSettings().locomotionPlacementPredictor = true;
    // E5 nova (decisao do usuario, 30/09 noite): a animacao anda, os pes
    // corrigem o pouso pelo equilibrio, parado a E4 pelas pernas com a ajuda
    // da pelve presente (XRETAIN, padrao 0,5; andando XWALKASSIST, 0,6).
    // XWALKASSIST only has the advertised meaning in E5. Previously it could
    // be passed alone while jointWhileWalking stayed false; update() then
    // selected walkingAssist=1 and a purported 10% test actually ran at
    // 100%. Make the diagnostic impossible to invoke in that invalid mode.
    const bool e5Animation = std::getenv("XE5ANIM") != nullptr
        || std::getenv("XWALKASSIST") != nullptr;
    if (e5Animation) {
        controller.settings().jointSupport = true;
        controller.settings().jointBalance = true;
        controller.settings().jointPosture = true;
        controller.settings().jointWhileWalking = true;
        controller.settings().legsAssistRetained = 0.5f;
        controller.settings().walkingAssistScale = 0.6f;
        controller.settings().legsWeightFraction = 0.0f;
        locomotion.footworkSettings().recoveryCaptureTargeting = true;
    }
    if (const char* v = std::getenv("XRETAIN")) controller.settings().legsAssistRetained = std::strtof(v, nullptr);
    if (const char* v = std::getenv("XWALKASSIST")) controller.settings().walkingAssistScale = std::strtof(v, nullptr);
    if (const char* v = std::getenv("XLEGHF")) controller.settings().legHeightFrequency = std::strtof(v, nullptr);
    if (const char* v = std::getenv("XLEGWEIGHT")) controller.settings().legsWeightFraction = std::strtof(v, nullptr);
    if (std::getenv("XNOLEGBAL")) controller.settings().jointBalance = false;
    if (std::getenv("XNOLEGPOST")) controller.settings().jointPosture = false;
    if (const char* v = std::getenv("XHF")) controller.settings().heightFrequency = std::strtof(v, nullptr);
    // R0 (pacote v2): as mudancas que valem tambem no H7, isoladas.
    if (std::getenv("XNOACCEPT")) locomotion.footworkSettings().loadAcceptanceByEvent = false;
    if (std::getenv("XNOCOMPLY")) locomotion.footworkSettings().acceptanceCompliance = false;
    if (std::getenv("XNOH18")) locomotion.footworkSettings().transferGeometryRelease = false;
    if (std::getenv("XNOZMP")) locomotion.footworkSettings().zmpLoadEstimate = false;
    std::array<std::size_t, 2> feet{};
    std::size_t chest = 0;
    std::array<std::size_t, 2> hands{};
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
        if (profile.links[i].id == "UpperChest") chest = i;
        if (profile.links[i].id == "LeftHand") hands[0] = i;
        if (profile.links[i].id == "RightHand") hands[1] = i;
    }
    ScenarioResult result;
    Vec3 follow{};
    float heldPreviousExcess = 0;  // levantar: excesso de inclinacao do tick anterior
    float jumpCharge = 0;  // carga do ultimo pulo (0..1)
    float chargingSeconds = 0;  // segurando o pulo ha quanto tempo
    bool wasDown = false;
    std::array<bool, 2> wasPlanted{true, true};
    std::array<FootPhase3D, 2> previousPhase{FootPhase3D::Stance, FootPhase3D::Stance};
    std::array<bool, 2> previousOwned{};
    std::array<int, 2> openStep{-1, -1};
    std::array<Vec3, 2> previousTarget{};
    std::array<Vec3, 2> previousTargetVelocity{};
    std::array<bool, 2> previousTargetVelocityValid{};
    std::array<bool, 2> wasPlantedPress{true, true};
    std::array<bool, 2> wasPlantedDrag{true, true};
    Vec3 dragFrom {};
    bool dragging = false;
    double trackingSum = 0;
    int trackingCount = 0;
    unsigned dumpedTau = 0;
    float handoffAt = -1;
    float handoffSpread = 0;
    double assistSum = 0, strengthSum = 0;
    int assistCount = 0;
    // Outro boneco.
    RagdollHandle3D npcHandle {};
    CharacterLocomotion3D npcLocomotion;
    AdaptivePhysicalCharacter3D npcController;
    if (scenario.npc) {
        RagdollSpawnDefinition3D other;
        other.entityId = 9102;
        other.active = true;
        other.pelvisPosition = {scenario.npcPosition.x, scenario.npcPosition.y,
            scenario.npcSpawnPelvisHeight > 0 ? scenario.npcSpawnPelvisHeight
                : profile.standingRootHeightMeters};
        other.orientation = (Quaternion::fromAxisAngle({0, 0, 1}, scenario.npcYaw)
            * Quaternion::fromAxisAngle({0, 1, 0}, scenario.npcSpawnPitchDegrees * 3.14159265f / 180)
            * Quaternion::fromAxisAngle({1, 0, 0}, scenario.npcSpawnRollDegrees * 3.14159265f / 180))
            .normalized();
        npcHandle = scene->createRagdoll(profile, other);
        npcLocomotion.reset(profile, scene->ragdollState(npcHandle));
        npcController.reset(profile);
    }
    int npcFalls = 0;
    bool npcWasDown = false;
    float npcSinceGetUp = 10;
    Vec3 npcAnchor = scenario.npcPosition;
    Vec3 npcAnchorVelocity {};
    constexpr float dt = 1.0f / 120;
    const int ticks = static_cast<int>(scenario.seconds / dt);
    Vec3 previousMove {};
    float touchdownAt = -1;
    CharacterLocomotionState3D previousLocomotionState = CharacterLocomotionState3D::Idle;
    float flipAt = -1;
    Vec3 flipDirection {};
    for (int tick = 0; tick < ticks; ++tick) {
        const float t = tick * dt;
        // Bancada C (fase 4): parado, descarrega um pe sem solta-lo.
        if (const char* bench = std::getenv("XBENCHC")) {
            // XBENCHC=2: espelho (descarrega o direito primeiro).
            const int first = std::atoi(bench) == 2 ? 1 : 0;
            locomotion.footworkSettings().benchTransferFoot = t >= 2.5f && t < 4.0f ? first
                : t >= 5.0f && t < 6.5f ? 1 - first : -1;
        }
        const ScenarioCommand wanted = scenario.script ? scenario.script(t) : ScenarioCommand{};
        if (wanted.move.lengthSquared() > 1e-6f) {
            if (previousMove.lengthSquared() > 1e-6f && flipAt < 0
                && dot(previousMove.normalized(), wanted.move.normalized()) < -0.5f) {
                flipAt = t;
                flipDirection = wanted.move.normalized();
            }
            previousMove = wanted.move;
        }
        CharacterMotorCommand3D command;
        command.moveDirection = wanted.move;
        // E5 (passos pelo planejador): a capsula nao anda sozinha - so segue o
        // corpo; quem o leva sao as pernas.
        if (locomotion.footworkSettings().locomotionStepping) command.moveDirection = {};
        command.sprint = wanted.sprint;
        command.jumpPressed = wanted.jump;
        command.jumpChargeSeconds = wanted.jumpChargeSeconds;
        if (wanted.jump) jumpCharge = characterJumpCharge3D(settings, wanted.jumpChargeSeconds);
        command.ignoreRagdolls = true;
        command.followVelocity = follow;
        command.navigationProxy = true;
        if (wanted.move.lengthSquared() > 1e-6f) {
            const float travel = std::atan2(wanted.move.y, wanted.move.x) - wanted.lookYaw;
            const float want = characterGaitSpeed3D(speeds, travel, wanted.sprint);
            command.speedScale = want / (wanted.sprint ? settings.sprintSpeed : settings.walkSpeed);
        }
        command.speedScale = std::min(command.speedScale,
            locomotion.telemetry().terrainSpeedLimit / std::max(0.1f,
                wanted.sprint ? settings.sprintSpeed : settings.walkSpeed));
        const auto motorSettings = scenario.legacy ? settings
            : adaptiveCharacterMotorSettings3D(settings, scene->characterState(), command);
        scene->moveCharacter(command, motorSettings, dt);
        const PhysicsCharacterState3D& capsule = scene->characterState();
        const RagdollState3D state = scene->ragdollState(handle);
        const float feetHeight = capsule.position.z - settings.standingHeight * 0.5f;
        CharacterLocomotionInput3D input;
        input.rootPositionWorld = {capsule.position.x, capsule.position.y,
            feetHeight + profile.standingRootHeightMeters};
        input.rootVelocityWorld = capsule.velocity;
        input.proxyVelocityWorld = {capsule.velocity.x, capsule.velocity.y, 0};
        // O pedido puro do script (como as teclas no laboratorio), inclusive
        // caido: a velocidade da capsula e outra coisa (proxyVelocityWorld).
        input.intent.requestedFacingYawRadians = wanted.lookYaw;
        input.intent.effort = wanted.sprint ? 1.0f : 0.0f;
        if (!std::getenv("XNOINTENT") && wanted.move.lengthSquared() > 1e-6f) {
            const Vec3 direction = Vec3{wanted.move.x, wanted.move.y, 0}.normalized();
            input.intent.requestedDirectionWorld = direction;
            input.intent.requestedVelocityWorld = direction * std::min(characterGaitSpeed3D(speeds,
                std::atan2(direction.y, direction.x) - wanted.lookYaw, wanted.sprint),
                locomotion.telemetry().terrainSpeedLimit);
            // Diagnostico (E5): limita a velocidade pedida (andar devagar).
            if (const char* v = std::getenv("XGAITSPEED")) {
                const float cap = std::strtof(v, nullptr);
                const float speed = input.intent.requestedVelocityWorld.length();
                if (speed > cap) input.intent.requestedVelocityWorld = input.intent.requestedVelocityWorld * (cap / speed);
            }
        }
        input.facingYawRadians = wanted.lookYaw;
        input.grounded = capsule.grounded;
        input.sprinting = wanted.sprint;
        input.controlled = true;
        input.forceDrivenRoot = !scenario.legacy;
        // Diagnostico (criterio da E4: menos torque, menos capacidade).
        if (const char* v = std::getenv("XMUSCLE")) input.muscleAuthority = std::strtof(v, nullptr);
        chargingSeconds = wanted.jumpCharging ? chargingSeconds + dt : 0.0f;
        input.jumpCrouch = wanted.jumpCharging && capsule.grounded
            ? characterJumpCharge3D(settings, chargingSeconds) : 0.0f;
        if (!scenario.legacy && !std::getenv("XNOYIELD"))
            input.jointStrength = controller.output().jointStrength;
        if (!scenario.legacy) input.externalPerturbation = controller.output().perturbation;
        // E5: a base das pernas de apoio adiantada pelo que o controle do
        // corpo pediu as pernas no tick anterior (XNOLEAD desliga, XLEADSCALE
        // escala - diagnostico).
        input.bodyReference = std::getenv("XBODYREF") != nullptr;
        input.bodyReachProjection = std::getenv("XREACH") != nullptr;
        input.balanceFootPlacement = (e5Animation && !std::getenv("XNOFOOTFB")) || std::getenv("XFOOTFB");
        input.footSpring = !std::getenv("XNOANKLE");
        input.ankleBalance = !std::getenv("XNOANKLEBAL");
        // Variacao de movimento (vida, como no laboratorio): XVARIETY=intensidade,
        // XVARIETYSEED=semente (padrao: o nome do cenario).
        if (const char* v = std::getenv("XVARIETY")) {
            input.motionVariety = std::strtof(v, nullptr);
            std::uint32_t seed = 2166136261u;
            for (const char* c = scenario.name; c != nullptr && *c != 0; ++c) seed = (seed ^ static_cast<unsigned char>(*c)) * 16777619u;
            if (const char* s = std::getenv("XVARIETYSEED")) seed = static_cast<std::uint32_t>(std::strtoul(s, nullptr, 10));
            input.varietySeed = seed;
        }
        if (!scenario.legacy && !std::getenv("XNOLEAD")) {
            input.legBaseAccelerationWorld = controller.output().requestedComAccelerationWorld;
            if (const char* v = std::getenv("XLEADSCALE"))
                input.legBaseAccelerationWorld = input.legBaseAccelerationWorld * std::strtof(v, nullptr);
        }
        // Segurado pela PhysGun, como no laboratorio.
        input.manipulated = scene->grabbing() && scene->grabbedRagdoll() == handle;
        input.grabbedLink = input.manipulated ? scene->grabbedRagdollLink()
            : RagdollDynamics3D::InvalidIndex;
        PhysicsScene3D* query = scene.get();
        const bool unseen = scenario.unseenObstacles;
        input.groundAt = [query, unseen](Vec3 point) {
            return unseen ? query->probeGround(point + Vec3{0, 0, 0.55f}, 0.03f, 0, 1.5f, 60)
                          : query->probeTerrain(point + Vec3{0, 0, 0.55f}, 0.03f, 0, 1.5f, 60);
        };
        for (std::size_t side = 0; side < 2; ++side)
            input.footGround[side] = unseen
                ? scene->probeGround(state.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48)
                : scene->probeTerrain(state.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
        locomotion.update(profile, clips.animations, state,
            scene->ragdollDynamics(handle), input, dt);
        const auto& output = locomotion.output();
        if (stepTraceSide >= 0 && stepTraceSide < 2 && stepTrace.size() < 4096) {
            const std::size_t side = static_cast<std::size_t>(stepTraceSide);
            const auto& plan = locomotion.contactPlan();
            const auto& ps = locomotion.physicalState();
            StepTraceSample sample {};
            sample.t = t;
            sample.phase = static_cast<int>(plan.feet[side].phase);
            sample.reason = static_cast<int>(plan.feet[side].reason);
            sample.stepId = plan.feet[side].stepId;
            sample.load = plan.loadShare[side];
            sample.supporting = ps.feet[side].supporting ? 1 : 0;
            sample.evidence = static_cast<int>(ps.feet[side].evidence);
            sample.memory = ps.feet[side].heldFromMemory ? 1 : 0;
            sample.trace = locomotion.telemetry().footTrace[side];
            sample.reach = locomotion.telemetry().legReach[side];
            sample.otherPhase = static_cast<int>(plan.feet[1 - side].phase);
            sample.supportingStrict = ps.feet[side].contactObserved ? 1 : 0;
            const char* thigh = side == 0 ? "LeftThigh" : "RightThigh";
            const char* shin = side == 0 ? "LeftShin" : "RightShin";
            for (const auto& target : output.driveTargets) {
                const auto& id = profile.links[target.linkIndex].id;
                const float measured = state.joints[target.linkIndex].positionRadians[static_cast<std::size_t>(target.axis)];
                if (id == thigh && target.axis == static_cast<RagdollAxis3D>(1)) {
                    sample.hipTarget = target.positionRadians; sample.hipMeasured = measured;
                }
                if (id == shin && target.axis == static_cast<RagdollAxis3D>(0)) {
                    sample.kneeTarget = target.positionRadians; sample.kneeMeasured = measured;
                    sample.kneeVelocityTarget = target.velocityRadiansPerSecond;
                    sample.stiffness = target.stiffnessScale; sample.torque = target.maximumTorqueScale;
                }
            }
            stepTrace.push_back(sample);
        }
        const auto bodyState = locomotion.telemetry().state;
        const bool down = bodyState == CharacterLocomotionState3D::Fallen
            || bodyState == CharacterLocomotionState3D::GettingUp;
        AdaptivePhysicalIntent3D intent;
        intent.proxyVelocityWorld = input.proxyVelocityWorld;
        intent.navigationRootWorld = input.rootPositionWorld;
        intent.airborne = !input.grounded;
        intent.manipulated = input.manipulated;
        intent.suspended = down;
        intent.jumpCharge = jumpCharge;
        intent.terrainDemand = locomotion.telemetry().terrainDemand;
        intent.footGround = input.footGround;
        intent.followVelocityWorld = follow;
        intent.requestedVelocityWorld = input.intent.requestedVelocityWorld;
        intent.physicalState = &locomotion.physicalState();
        intent.contactPlan = &locomotion.contactPlan();
        controller.update(profile, state, output.guide, intent, dt);
        const auto& physical = controller.output();
        // Invariante do modo fisico: o guia nunca escreve transform (pose e
        // raiz com autoridade zero), nem caido, levantando ou segurado.
        if (!scenario.legacy)
            require(output.guide.poseAuthority == 0.0f && output.guide.rootTranslationAuthority == 0.0f
                && output.guide.rootRotationAuthority == 0.0f,
                "No modo fisico o guia escreveu transform (autoridade diferente de zero)");
        // A mesma aplicacao do laboratorio (motores, guia, forcas).
        if (std::getenv("XDIRECTFF") && !scenario.legacy && !down && physical.jointFeedforwardActive) {
            // Diagnostico: os torques de junta do corpo inteiro aplicados
            // direto nos links (+ filho, - pai), sem o orcamento do motor.
            AdaptivePhysicalOutput3D copy = physical;
            copy.jointFeedforwardActive = false;
            applyCharacterControl3D(*scene, handle, output, &copy, down);
            for (std::size_t i = 1; i < profile.links.size() && i < physical.jointFeedforward.size(); ++i) {
                const auto& link = profile.links[i];
                if (link.parentIndex < 0) continue;
                const Quaternion frame = state.links[i].orientation * link.modelOrientation.conjugate()
                    * link.inboundJoint.frameModelOrientation;
                const auto& ff = physical.jointFeedforward[i];
                const Vec3 torque = frame.rotate(Vec3 {ff[0], ff[1], ff[2]});
                if (torque.lengthSquared() < 1e-8f) continue;
                scene->applyRagdollControlLinkForce(handle, static_cast<std::uint32_t>(i), {}, torque);
                scene->applyRagdollControlLinkForce(handle, static_cast<std::uint32_t>(link.parentIndex), {}, torque * -1.0f);
            }
        } else
        // XNOASSIST (diagnostico, criterio da E4): so os motores seguindo a
        // pose, sem controle do corpo (nem ajuda na pelve, nem pernas).
        applyCharacterControl3D(*scene, handle, output,
            scenario.legacy || std::getenv("XNOASSIST") ? nullptr : &physical, down);
        // A capsula vai atras do corpo (mesma regra do laboratorio).
        follow = {};
        if (down) {
            const Vec3 root = state.links.front().position;
            scene->placeCharacter({root.x, root.y, 0}, settings);
        } else {
            follow = navigationFollowVelocity3D(state.links.front().position,
                output.guide.rootPositionWorld,
                0.10f * (1.0f - std::clamp(physical.perturbation, 0.0f, 1.0f)));
        }
        if (scenario.disturb) scenario.disturb(*scene, t, state);
        if (scenario.continuousForceNewtons > 0.0f && t >= scenario.forceStart
                && t < scenario.forceStart + scenario.forceSeconds && state.links.size() > 3) {
            // A direcao acompanha o corpo (como no laboratorio).
            Vec3 forward = state.links.front().orientation.rotate({1, 0, 0});
            forward.z = 0.0f;
            forward = forward.lengthSquared() < 1e-6f ? Vec3 {1, 0, 0} : forward.normalized();
            const Vec3 left {-forward.y, forward.x, 0.0f};
            const float a = scenario.continuousForceDegrees * 3.14159265f / 180;
            const Vec3 total = (forward * std::cos(a) + left * std::sin(a)) * scenario.continuousForceNewtons;
            scene->applyRagdollLinkForce(handle, 0, total * 0.20f, {});
            scene->applyRagdollLinkForce(handle, 2, total * 0.32f, {});
            scene->applyRagdollLinkForce(handle, 3, total * 0.48f, {});
        }
        if (scenario.dragLink != nullptr) {
            std::uint32_t link = 0;
            for (std::size_t i = 0; i < profile.links.size(); ++i)
                if (profile.links[i].id == scenario.dragLink) link = static_cast<std::uint32_t>(i);
            const float elapsed = t - scenario.dragStart;
            if (elapsed >= 0 && elapsed < scenario.dragSeconds) {
                PhysicsGrabTarget3D target;
                if (!dragging) {
                    dragFrom = state.links[link].position;
                    target.position = dragFrom;
                    dragging = scene->beginRagdollGrab(handle, link, {}, target, PhysicsHandleSettings3D {});
                } else {
                    target.position = dragFrom + scenario.dragVelocity * elapsed;
                    scene->updateGrabTarget(target, PhysicsHandleSettings3D {});
                }
            } else if (dragging) {
                scene->endGrab();
                dragging = false;
            }
            result.dragged = dragging;
        }
        if (scenario.npc) {
            const RagdollState3D other = scene->ragdollState(npcHandle);
            const auto& pelvis = other.links.front();
            CharacterLocomotionInput3D npcInput;
            npcInput.rootPositionWorld = {npcAnchor.x, npcAnchor.y,
                profile.standingRootHeightMeters};
            npcInput.rootVelocityWorld = {};
            npcInput.facingYawRadians = scenario.npcYaw;
            npcInput.grounded = true;
            npcInput.forceDrivenRoot = true;
            npcInput.jointStrength = npcController.output().jointStrength;
            npcInput.externalPerturbation = npcController.output().perturbation;
            for (std::size_t side = 0; side < 2; ++side)
                npcInput.footGround[side] = scene->probeTerrain(
                    other.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
            // Como no laboratorio: o boneco solto nao recebe RagdollDynamics3D.
            npcLocomotion.update(profile, clips.animations, other,
                RagdollDynamics3D {}, npcInput, dt);
            const auto& npcOutput = npcLocomotion.output();
            const auto npcState = npcLocomotion.telemetry().state;
            const bool npcDown = npcState == CharacterLocomotionState3D::Fallen
                || npcState == CharacterLocomotionState3D::GettingUp;
            if (npcDown && !npcWasDown) ++npcFalls;
            if (!npcDown && npcWasDown) { ++result.npcGetUps; npcSinceGetUp = 0; }
            npcSinceGetUp += dt;
            npcWasDown = npcDown;
            // Fora o 0,7 s do nascimento (nascer deitado e artificio do
            // teste: a referencia sobe do chao ate a altura de pe).
            if (t > 0.7f)
                result.npcGuideSpeedPeak = std::max(result.npcGuideSpeedPeak,
                    npcOutput.guide.rootLinearVelocityWorld.length());
            if (std::getenv("XNPC") && (tick % 12 == 0 || npcOutput.guide.rootLinearVelocityWorld.length() > 3.0f))
                std::printf("NPC t %.3f estado %d fase %d guia v %.2f (%+.2f %+.2f %+.2f) pelve z %.2f ancora erro %.3f tilt %.0f prog %.2f\n", t,
                    static_cast<int>(npcState), static_cast<int>(npcLocomotion.telemetry().getUpPhase),
                    npcOutput.guide.rootLinearVelocityWorld.length(), npcOutput.guide.rootLinearVelocityWorld.x,
                    npcOutput.guide.rootLinearVelocityWorld.y, npcOutput.guide.rootLinearVelocityWorld.z,
                    pelvis.position.z, std::hypot(pelvis.position.x - npcAnchor.x, pelvis.position.y - npcAnchor.y),
                    std::acos(std::clamp(pelvis.orientation.rotate({0, 0, 1}).z, -1.0f, 1.0f)) * 57.3f,
                    npcLocomotion.telemetry().getUpProgress);
            {
                Vec3 gap = pelvis.position - npcAnchor;
                gap.z = 0;
                result.npcAnchorErrorPeak = std::max(result.npcAnchorErrorPeak, gap.length());
            }
            if (npcSinceGetUp < 1.0f)
                for (const auto& link : other.links)
                    result.npcAfterGetUpLinkPeak = std::max(result.npcAfterGetUpLinkPeak,
                        link.linearVelocity.length());
            result.npcStandingAtEnd = !npcDown
                && pelvis.orientation.rotate({0, 0, 1}).z > 0.85f;
            AdaptivePhysicalIntent3D npcIntent;
            npcIntent.navigationRootWorld = npcInput.rootPositionWorld;
            npcIntent.suspended = npcDown;
            npcIntent.footGround = npcInput.footGround;
            npcIntent.followVelocityWorld = npcAnchorVelocity;
            npcIntent.physicalState = &npcLocomotion.physicalState();
            npcIntent.contactPlan = &npcLocomotion.contactPlan();
            npcController.update(profile, other, npcOutput.guide, npcIntent, dt);
            applyCharacterControl3D(*scene, npcHandle, npcOutput, &npcController.output(), npcDown);
            npcAnchorVelocity = navigationFollowVelocity3D(pelvis.position,
                npcAnchor, npcDown ? 0.0f : 0.10f * (1.0f - std::clamp(
                    npcController.output().perturbation, 0.0f, 1.0f)));
            npcAnchor += npcAnchorVelocity * dt;
            result.npcFalls = npcFalls;
            result.npcFinal = pelvis.position;
        }
        scene->simulate(dt);

        const RagdollState3D after = scene->ragdollState(handle);
        if (std::getenv("XTRACE") && tick % (std::getenv("XFINE") ? 3 : 12) == 0) {
            std::printf("TRACE %.2f erro %.1f parte %.2f apoio %.2f pert %.2f pes %d%d y %.2f vy %+.2f | x %.2f z %.3f vz %.2f refz %.3f refvz %.2f Fz %.0f tilt %.1f mode %d speed %.2f limit %.2f rf %.2f cm %.2f env %.2f\n",
                t, physical.tiltErrorDegrees, physical.uprightAuthorityShare, physical.supportAuthority,
                physical.perturbation, locomotion.telemetry().footPlanted[0] ? 1 : 0,
                locomotion.telemetry().footPlanted[1] ? 1 : 0, after.links[0].position.y, after.links[0].linearVelocity.y,
                after.links[0].position.x, after.links[0].position.z,
                after.links[0].linearVelocity.z, output.guide.rootPositionWorld.z,
                output.guide.rootLinearVelocityWorld.z, physical.rootForceWorld.z,
                physical.bodyTiltRadians * 57.3f, static_cast<int>(bodyState),
                std::hypot(capsule.velocity.x, capsule.velocity.y), locomotion.telemetry().terrainSpeedLimit,
                locomotion.telemetry().fallReflex, locomotion.telemetry().fallCommit,
                physical.assistEnvelope);
        }
        if (std::getenv("XLEANTRACE") && tick % 6 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 1.9f)
                && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 3.5f)) {
            // Inclinacao: peito e pelve fisicos (frente/lado no rumo da
            // pelve, graus), a inclinacao pedida a pelve e a velocidade.
            const auto tiltOf = [&](Quaternion q, float& forward, float& side) {
                const Vec3 up = q.rotate({ 0.0f, 0.0f, 1.0f });
                const Vec3 f = after.links[0].orientation.rotate({ 1.0f, 0.0f, 0.0f });
                const Vec3 fwd = Vec3 { f.x, f.y, 0.0f }.normalized();
                const Vec3 left { -fwd.y, fwd.x, 0.0f };
                forward = std::asin(std::clamp(dot(up, fwd), -1.0f, 1.0f)) * 57.3f;
                side = std::asin(std::clamp(dot(up, left), -1.0f, 1.0f)) * 57.3f;
            };
            float cf, cs, pf, ps;
            tiltOf(after.links[chest].orientation, cf, cs);
            tiltOf(after.links[0].orientation, pf, ps);
            const Vec3 v = after.links[0].linearVelocity;
            std::printf("INCLINA %.2f v %.2f | peito frente %+5.1f lado %+5.1f | pelve frente %+5.1f lado %+5.1f | pedido frente %+5.1f lado %+5.1f | olhar %+4.0f estado %d\n",
                t, std::hypot(v.x, v.y), cf, cs, pf, ps,
                locomotion.telemetry().forwardLeanRadians * 57.3f, locomotion.telemetry().lateralLeanRadians * 57.3f,
                wanted.lookYaw * 57.3f, static_cast<int>(locomotion.telemetry().state));
        }
        if (std::getenv("XJITTER") && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 2.0f)
                && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 2.6f)) {
            // Tremida parado: altura e velocidade vertical da pelve, contato
            // estrito dos pes, fracao das pernas, forcas de sustentacao.
            const auto& ps = locomotion.physicalState();
            std::printf("TREMIDA %.3f z %.4f vz %+.3f contato %d%d apoio %d%d | pernas %.2f/%.2f Fz-pernas %4.0f Fz-pelve %4.0f sup %+.2f | ref z %.4f vz %+.3f postura-pernas %+.0f %+.0f\n",
                t, after.links[0].position.z, after.links[0].linearVelocity.z,
                ps.feet[0].contactObserved ? 1 : 0, ps.feet[1].contactObserved ? 1 : 0,
                ps.feet[0].supporting ? 1 : 0, ps.feet[1].supporting ? 1 : 0,
                physical.legShare, physical.balanceShare, physical.jointSupportNewtons, physical.rootForceWorld.z,
                physical.supportAcceleration, output.guide.rootPositionWorld.z, output.guide.rootLinearVelocityWorld.z,
                physical.legPostureRequested.x, physical.legPostureRequested.y);
        }
        if (std::getenv("XVMCT") && tick % 3 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 1.0f) && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 2.0f)) {
            const auto& ps = locomotion.physicalState();
            const auto& plan = locomotion.contactPlan();
            std::printf("VMC %.3f fase %d%d segue %d%d apoio %d%d ev %d%d mem %d%d Fz %.0f %.0f Fx %.0f %.0f z %.3f tilt %.1f pelveF %.0f %.0f %.0f v %.2f\n",
                t, (int)plan.feet[0].phase, (int)plan.feet[1].phase, plan.feet[0].followingGait ? 1 : 0,
                plan.feet[1].followingGait ? 1 : 0, ps.feet[0].supporting ? 1 : 0, ps.feet[1].supporting ? 1 : 0,
                (int)ps.feet[0].evidence, (int)ps.feet[1].evidence, ps.feet[0].heldFromMemory ? 1 : 0,
                ps.feet[1].heldFromMemory ? 1 : 0, physical.contactWrench[0].forceWorld.z,
                physical.contactWrench[1].forceWorld.z, physical.contactWrench[0].forceWorld.x,
                physical.contactWrench[1].forceWorld.x, after.links[0].position.z,
                physical.bodyTiltRadians * 57.3f, physical.rootForceWorld.x, physical.rootForceWorld.y,
                physical.rootForceWorld.z, after.links[0].linearVelocity.x);
        }
        if (std::getenv("XPRESS") && tick % 6 == 0) {
            Vec3 external {};
            for (const auto& contact : after.interactions) {
                if ((contact.linkIndex == feet[0] || contact.linkIndex == feet[1]) && contact.normalWorld.z > 0.5f) continue;
                external += contact.normalWorld * (contact.normalImpulseNewtonSeconds / dt);
            }
            const auto& ps = locomotion.physicalState();
            const auto& plan = locomotion.contactPlan();
            std::printf("PRESS %.2f pert %.2f env %.2f cap %.3f com v %+.2f %+.2f tilt %.1f pes %d%d fase %d%d razao %d%d carga %.2f apoio %d ext %+.0f %+.0f %+.0f estado %d | reat %.2f fora %.3f vel %.2f\n",
                t, physical.perturbation, physical.assistEnvelope, physical.captureErrorMeters,
                ps.centerOfMassVelocityWorld.x, ps.centerOfMassVelocityWorld.y,
                physical.bodyTiltRadians * 57.3f,
                locomotion.telemetry().footPlanted[0] ? 1 : 0, locomotion.telemetry().footPlanted[1] ? 1 : 0,
                static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase),
                static_cast<int>(plan.feet[0].reason), static_cast<int>(plan.feet[1].reason),
                plan.loadShare[0], plan.recovering ? 10 + (plan.feet[0].followingGait ? 1 : 0) * 0 : (plan.feet[0].followingGait ? 1 : 0) * 1 + (plan.feet[1].followingGait ? 2 : 0), external.x, external.y, external.z,
                static_cast<int>(locomotion.telemetry().state), locomotion.telemetry().reactiveShare,
                locomotion.telemetry().captureOutsideMeters, locomotion.telemetry().speedMetersPerSecond);
        }
        if (std::getenv("XFEET") && tick % 3 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 1.0f) && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 2.5f)) {
            const auto& ps = locomotion.physicalState();
            const auto& plan = locomotion.contactPlan();
            const Vec3 fwd = after.links.front().orientation.rotate({1, 0, 0});
            std::printf("FEET %.3f [%d%d%d cap %.3f dir %+.2f %+.2f] rec %d urg %.2f fase %d%d | pe0 %+.2f %+.2f sola %.3f | pe1 %+.2f %+.2f sola %.3f | com %+.2f %+.2f v %+.2f %+.2f | cap %+.2f %+.2f | alvo0 %+.2f %+.2f alvo1 %+.2f %+.2f | frente %+.2f %+.2f carga %.2f | alvoz %.3f %.3f pez %.3f %.3f\n",
                t, locomotion.telemetry().footworkStanding ? 1 : 0, locomotion.telemetry().footworkAllowSteps ? 1 : 0,
                locomotion.telemetry().footworkAllowRecovery ? 1 : 0, locomotion.telemetry().captureOutsideMeters,
                locomotion.telemetry().captureDirectionWorld.x, locomotion.telemetry().captureDirectionWorld.y,
                plan.recoverySide, plan.recoveryUrgency, (int)plan.feet[0].phase, (int)plan.feet[1].phase,
                after.links[feet[0]].position.x, after.links[feet[0]].position.y, ps.feet[0].soleLowestHeight,
                after.links[feet[1]].position.x, after.links[feet[1]].position.y, ps.feet[1].soleLowestHeight,
                ps.centerOfMassWorld.x, ps.centerOfMassWorld.y, ps.centerOfMassVelocityWorld.x, ps.centerOfMassVelocityWorld.y,
                ps.capturePointWorld.x, ps.capturePointWorld.y,
                plan.feet[0].goalWorld.x, plan.feet[0].goalWorld.y, plan.feet[1].goalWorld.x, plan.feet[1].goalWorld.y,
                fwd.x, fwd.y, plan.loadShare[0], plan.feet[0].targetWorld.z, plan.feet[1].targetWorld.z,
                after.links[feet[0]].position.z, after.links[feet[1]].position.z);
        }
        if (std::getenv("XLEGT") && tick % 3 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 1.0f) && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 2.5f)) {
            std::printf("LEG %.3f", t);
            for (const auto& target : output.driveTargets) {
                const auto& link = profile.links[target.linkIndex];
                if (link.id != "RightThigh" && link.id != "RightShin" && link.id != "RightFoot") continue;
                const auto& axis = link.inboundJoint.axes[static_cast<std::size_t>(target.axis)];
                std::printf(" %s%d %+.2f[%+.2f,%+.2f] k%.2f T%.2f", link.id.c_str() + 5, static_cast<int>(target.axis),
                    target.positionRadians, axis.minimumRadians, axis.maximumRadians, target.stiffnessScale, target.maximumTorqueScale);
            }
            std::printf("\n");
        }
        if (std::getenv("XPOSTT") && tick % 3 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 1.0f) && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 2.5f)) {
            const auto& plan = locomotion.contactPlan();
            const auto& ps = locomotion.physicalState();
            const Vec3 up = after.links[0].orientation.rotate({0, 0, 1});
            const Vec3 chestUp = after.links[chest].orientation.rotate({0, 0, 1});
            std::printf("POST %.3f fase %d%d apoio %d%d pedido %+5.0f %+5.0f entregue %+5.0f %+5.0f pelveT %+5.0f %+5.0f | pelve up %+.2f %+.2f peito up %+.2f %+.2f | w %+.2f %+.2f | Fz %.0f %.0f leg %.2f post %.2f\n",
                t, (int)plan.feet[0].phase, (int)plan.feet[1].phase, ps.feet[0].supporting ? 1 : 0, ps.feet[1].supporting ? 1 : 0,
                physical.legPostureRequested.x, physical.legPostureRequested.y,
                physical.legPostureDelivered.x, physical.legPostureDelivered.y,
                physical.rootTorqueWorld.x, physical.rootTorqueWorld.y, up.x, up.y, chestUp.x, chestUp.y,
                after.links[0].angularVelocity.x, after.links[0].angularVelocity.y,
                physical.contactWrench[0].forceWorld.z, physical.contactWrench[1].forceWorld.z,
                physical.legShare, physical.balanceShare);
        }
        if (std::getenv("XFFCLAMP") && !down && physical.jointSupportNewtons > 0.0f) {
            static double asked = 0, over = 0;
            static int ticks = 0;
            for (const auto& target : output.driveTargets) {
                const auto& link = profile.links[target.linkIndex];
                if (link.id.find("Thigh") == std::string::npos && link.id.find("Shin") == std::string::npos
                    && link.id.find("Foot") == std::string::npos) continue;
                if (target.linkIndex >= physical.jointFeedforward.size()) continue;
                const float ff = physical.jointFeedforward[target.linkIndex][static_cast<std::size_t>(target.axis)];
                const float maximum = link.inboundJoint.axes[static_cast<std::size_t>(target.axis)].maximumTorque
                    * std::clamp(target.maximumTorqueScale, 0.0f, 3.0f);
                asked += std::abs(ff) * dt;
                over += std::max(0.0f, std::abs(ff) - maximum) * dt;
            }
            if (++ticks % 240 == 0)
                std::printf("FFCLAMP pedido %.0f acima do limite %.0f (N.m.s)\n", asked, over);
        }
        if (std::getenv("XPOSTURE") && !down) {
            static double requested = 0, delivered = 0, pelvis = 0, tickCount = 0;
            requested += physical.legPostureRequested.length() * dt;
            delivered += physical.legPostureDelivered.length() * dt;
            pelvis += physical.rootTorqueWorld.length() * dt;
            tickCount += 1;
            if (tick % 120 == 0)
                std::printf("POSTURE pedido %.0f entregue %.0f pelve %.0f (N.m.s acumulados)\n", requested, delivered, pelvis);
        }
        if (std::getenv("XSETTLE") && tick % 12 == 0 && down) {
            const auto& tel = locomotion.telemetry();
            std::printf("SETTLE t %.2f fase %d orient %d tonus %.2f erro %.1f tilt %.0f prog %.2f pelve z %.2f\n", t,
                static_cast<int>(tel.getUpPhase), static_cast<int>(tel.fallOrientation), tel.muscleTone,
                tel.jointErrorRmsDegrees, std::acos(std::clamp(after.links.front().orientation.rotate({0, 0, 1}).z, -1.0f, 1.0f)) * 57.3f,
                tel.getUpProgress, after.links.front().position.z);
        }
        if (std::getenv("XBASE") && tick % 12 == 0) {
            std::printf("BASE t %.2f margem %+.3f autoridade %.2f tilt %.1f apoio %.2f pes %d%d quedas %d\n", t,
                physical.balanceMargin, physical.balanceAuthority,
                physical.bodyTiltRadians * 57.3f, physical.supportAuthority,
                locomotion.telemetry().footPlanted[0] ? 1 : 0, locomotion.telemetry().footPlanted[1] ? 1 : 0, result.falls);
        }
        if (std::getenv("XREC") && (locomotion.telemetry().reactiveShare > 0.0f || tick % 24 == 0) && t < 2.0f) {
            const auto& ps = locomotion.physicalState();
            std::printf("REC t %.3f recuperando %.0f captura fora %.3f apoio %d%d evid %d%d com v %+.2f %+.2f estado %d\n", t,
                locomotion.telemetry().reactiveShare, ps.captureOutsideMeters,
                ps.feet[0].supporting ? 1 : 0, ps.feet[1].supporting ? 1 : 0,
                static_cast<int>(ps.feet[0].evidence), static_cast<int>(ps.feet[1].evidence),
                ps.centerOfMassVelocityWorld.x, ps.centerOfMassVelocityWorld.y,
                static_cast<int>(locomotion.telemetry().state));
        }
        if (std::getenv("XARMS") && tick % 12 == 0) {
            std::printf("ARMS %.2f fase %d tonus %.2f |", t, static_cast<int>(locomotion.telemetry().getUpPhase),
                locomotion.telemetry().muscleTone);
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                const auto& id = profile.links[i].id;
                if (id.find("Arm") != std::string::npos || id.find("Hand") != std::string::npos
                    || id == "Chest" || id == "Pelvis" || id == "Head")
                    std::printf(" %s %.2f", id.c_str(), after.links[i].position.z);
            }
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                const auto& id = profile.links[i].id;
                if (id != "LeftForearm" && id != "LeftUpperArm" && id != "LeftHand") continue;
                const auto& j = after.joints[i].positionRadians;
                std::printf(" | %s xyz %.2f %.2f %.2f j %.2f %.2f %.2f", id.c_str(), after.links[i].position.x,
                    after.links[i].position.y, after.links[i].position.z, j[0], j[1], j[2]);
                for (const auto& d : output.driveTargets)
                    if (d.linkIndex == i) std::printf(" alvo%d %.2f k %.2f c %.2f g %.2f", static_cast<int>(d.axis), d.positionRadians, d.stiffnessScale, d.dampingScale, d.gravityCompensationScale);
            }
            std::printf("\n");
        }
        if (std::getenv("XDBG") && tick % 3 == 0) {
            const Vec3 up = after.links.front().orientation.rotate({0, 0, 1});
            Vec3 com {}; float m = 0;
            for (std::size_t i = 0; i < after.links.size(); ++i) { com += after.links[i].position * profile.links[i].massFraction; m += profile.links[i].massFraction; }
            com = com * (1.0f / m);
            bool fc[2] {};
            for (const auto& contact : after.interactions)
                for (int s2 = 0; s2 < 2; ++s2) if (contact.linkIndex == feet[s2] && contact.normalWorld.z > 0.5f) fc[s2] = true;
            std::printf("DBG %.2f com x %.2f z %.2f vx %+.2f upx %+.2f | pe E x %.2f z %.2f %d | pe D x %.2f z %.2f %d | est %d cm %.2f rf %.2f plant %d%d\n", t,
                com.x, com.z, after.links[0].linearVelocity.x, up.x,
                after.links[feet[0]].position.x, after.links[feet[0]].position.z, fc[0] ? 1 : 0,
                after.links[feet[1]].position.x, after.links[feet[1]].position.z, fc[1] ? 1 : 0,
                static_cast<int>(locomotion.telemetry().state), locomotion.telemetry().fallCommit,
                locomotion.telemetry().fallReflex, locomotion.telemetry().footPlanted[0] ? 1 : 0,
                locomotion.telemetry().footPlanted[1] ? 1 : 0);
        }
        if (flipAt >= 0 && result.reverseSeconds < 0) {
            const Vec3 v = after.links.front().linearVelocity;
            result.reverseWorstTilt = std::max(result.reverseWorstTilt, std::acos(std::clamp(
                after.links.front().orientation.rotate({0, 0, 1}).z, -1.0f, 1.0f)) * 57.2958f);
            if (dot(Vec3{v.x, v.y, 0}, flipDirection) > 1.0f) result.reverseSeconds = t - flipAt;
            if (std::getenv("XB") && tick % 6 == 0)
                std::printf("XB t %.2f vx %+.2f cap %+.2f tilt %.0f estado %d setor %d facing %.0f pelve %.0f\n", t, v.x, capsule.velocity.x,
                    std::acos(std::clamp(after.links.front().orientation.rotate({0, 0, 1}).z, -1.0f, 1.0f)) * 57.3f,
                    static_cast<int>(locomotion.telemetry().state), static_cast<int>(locomotion.telemetry().gaitDirection),
                    locomotion.telemetry().facingYawRadians * 57.3f,
                    [&] { const Vec3 f = after.links.front().orientation.rotate({1, 0, 0}); return std::atan2(f.y, f.x) * 57.3f; }());
        }
        // Pouso: os 0,5 s depois do toque (saida do voo), com ou sem o estado
        // de pouso (pousando correndo a passada continua direto).
        if (previousLocomotionState == CharacterLocomotionState3D::Airborne
            && locomotion.telemetry().state != CharacterLocomotionState3D::Airborne && !down)
            touchdownAt = t;
        previousLocomotionState = locomotion.telemetry().state;
        if (touchdownAt >= 0 && t - touchdownAt < 0.5f) {
            bool footDown = false;
            for (const auto& contact : after.interactions)
                if (contact.normalWorld.z > 0.5f
                    && (contact.linkIndex == feet[0] || contact.linkIndex == feet[1])) footDown = true;
            if (footDown) result.landed = true;
            // Os dois pes claramente no ar (a sola a mais de ~7 cm): o
            // quique. Um pe saindo para o passo seguinte nao conta.
            else if (result.landed && std::min(after.links[feet[0]].position.z,
                    after.links[feet[1]].position.z) > 0.13f) ++result.landingBounceTicks;
            if (std::getenv("XJ"))
                std::printf("XL t %.3f pe %d | pelve z %.3f vz %+.2f | pes z %.3f %.3f\n", t, footDown ? 1 : 0,
                    after.links.front().position.z, after.links.front().linearVelocity.z,
                    after.links[feet[0]].position.z, after.links[feet[1]].position.z);
            if (result.landed)
                result.landingLowestPelvis = std::min(result.landingLowestPelvis,
                    after.links.front().position.z);
            if (result.landed && t - touchdownAt < 0.3f)
                result.landingRebound = std::max(result.landingRebound,
                    after.links.front().linearVelocity.z);
        }
        result.highestPelvis = std::max(result.highestPelvis, after.links.front().position.z);
        if (!down && locomotion.telemetry().fallCommit > 0.5f) result.rigidSeconds += dt;
        if (!down && locomotion.telemetry().fallReflex > 0.5f) result.armsReflexSeconds += dt;
        if (std::getenv("XDRAGT") && !down && t > 1.5f && t < 2.3f) {
            bool touching = false;
            for (const auto& contact : after.interactions)
                if (contact.linkIndex == feet[0] && contact.normalWorld.z > 0.5f) touching = true;
            const Vec3 v = after.links[feet[0]].linearVelocity;
            std::printf("XDRAGT t %.3f trav %d toca %d z %.3f vxy %.2f vz %+.2f\n", t,
                locomotion.telemetry().footPlanted[0] ? 1 : 0, touching ? 1 : 0,
                after.links[feet[0]].position.z, std::hypot(v.x, v.y), v.z);
        }
        if (!down && t > 0.5f && wanted.move.lengthSquared() > 1e-6f) {
            result.facingOffLookPeak = std::max(result.facingOffLookPeak, std::abs(std::remainder(
                locomotion.telemetry().facingYawRadians - wanted.lookYaw, 6.2831853f)) * 57.2958f);
            for (std::size_t side = 0; side < 2; ++side) {
                if (locomotion.telemetry().footPlanted[side]) continue;
                bool touching = false;
                for (const auto& contact : after.interactions)
                    if (contact.linkIndex == feet[side] && contact.normalWorld.z > 0.5f) touching = true;
                const Vec3 v = after.links[feet[side]].linearVelocity;
                if (touching && std::hypot(v.x, v.y) > 0.3f) result.dragSeconds[side] += dt;

                result.swingPeak[side] = std::max(result.swingPeak[side], after.links[feet[side]].position.z);
            }
        }
        result.highestCapsuleFeet = std::max(result.highestCapsuleFeet,
            capsule.position.z - settings.standingHeight * 0.5f);
        if (t > 0.5f && !down) {
            result.tiltErrorSum += physical.tiltErrorDegrees;
            ++result.tiltErrorCount;
            result.tiltErrorWorst = std::max(result.tiltErrorWorst, physical.tiltErrorDegrees);
            if (physical.tiltErrorDegrees > 15.0f) ++result.tiltErrorOver15;
            result.balanceReactionPeak = std::max(result.balanceReactionPeak,
                locomotion.telemetry().balanceReaction);
            {
                // Marcha: no rumo pedido, a velocidade do centro de massa.
                const Vec3 requested = input.intent.requestedVelocityWorld;
                const float requestedSpeed = std::hypot(requested.x, requested.y);
                const Vec3 comVelocity = locomotion.physicalState().centerOfMassVelocityWorld;
                if (requestedSpeed > 0.2f) {
                    if (result.gaitMoveStart < 0) result.gaitMoveStart = t;
                    result.gaitMoveEnd = t;
                    result.gaitStoppedAt = -1;
                    result.gaitRequestedSpeed = std::max(result.gaitRequestedSpeed, requestedSpeed);
                    const float along = (comVelocity.x * requested.x + comVelocity.y * requested.y) / requestedSpeed;
                    result.gaitBestSpeed = std::max(result.gaitBestSpeed, along);
                    if (result.gaitTimeTo80 < 0 && along >= 0.8f * requestedSpeed)
                        result.gaitTimeTo80 = t - result.gaitMoveStart;
                    if (dot(result.gaitLastRequest, requested) < 0.0f) {
                        result.gaitReverseAt = t;
                        result.gaitReversedAt = -1;
                    }
                    result.gaitLastRequest = requested;
                    if (result.gaitReverseAt >= 0 && result.gaitReversedAt < 0 && along >= 0.5f * requestedSpeed)
                        result.gaitReversedAt = t;
                    const Vec3 fwd = after.links.front().orientation.rotate({1, 0, 0});
                    const float bodyYaw = std::atan2(fwd.y, fwd.x);
                    float yawError = std::abs(std::remainder(bodyYaw - wanted.lookYaw, 2.0f * 3.14159265f)) * 57.2958f;
                    if (t - result.gaitMoveStart > 0.3f)
                        result.gaitWorstYawDegrees = std::max(result.gaitWorstYawDegrees, yawError);
                    if (t - result.gaitMoveStart >= 1.0f && result.falls == 0) {
                        result.gaitSustainedAlong += along;
                        result.gaitSustainedAcross += (-comVelocity.x * requested.y + comVelocity.y * requested.x) / requestedSpeed;
                        ++result.gaitSustainedSamples;
                    }
                } else if (result.gaitMoveEnd >= 0 && result.gaitStoppedAt < 0
                    && std::hypot(comVelocity.x, comVelocity.y) < 0.2f) {
                    result.gaitStoppedAt = t;
                }
            }
            result.pelvisX.emplace_back(t, after.links.front().position.x);
            {
                const auto& ps = locomotion.physicalState();
                for (std::size_t side = 0; side < 2; ++side) {
                    const bool seen = ps.feet[side].contactObserved;
                    result.touchdownAirSeconds[side] = seen ? 0.0f : result.touchdownAirSeconds[side] + dt;
                    if (seen && !result.touchdownContact[side] && t > 0.5f && result.lastAir[side] < 0.10f) ++result.touchdownScuffs;
                    if (seen && !result.touchdownContact[side] && t > 0.5f && result.lastAir[side] >= 0.10f) {
                        const auto& foot = after.links[feet[side]];
                        const GroundProbeResult3D ground = scene->probeTerrain(
                            foot.position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
                        if (ground.hasSurface) {
                            const Vec3 forward = foot.orientation.rotate({1, 0, 0});
                            const Vec3 left = foot.orientation.rotate({0, 1, 0});
                            // Alvo do tornozelo (flexao) menos o medido, no toque.
                            float ankleError = 0;
                            for (const auto& drive : locomotion.output().driveTargets)
                                if (drive.linkIndex == feet[side] && drive.axis == RagdollAxis3D::Twist)
                                    ankleError = (drive.positionRadians - after.joints[feet[side]].positionRadians[0]) * 57.2958f;
                            result.touchdowns.push_back({t, foot.position.x,
                                -std::asin(std::clamp(dot(forward, ground.normalWorld), -1.0f, 1.0f)) * 57.2958f,
                                std::asin(std::clamp(dot(left, ground.normalWorld), -1.0f, 1.0f)) * 57.2958f, ankleError,
                                locomotion.telemetry().footSwingProgress[side]});
                            // Erro (alvo - medido) do joelho e do quadril (3 eixos) e a
                            // inclinacao da pelve e da canela no mundo.
                            float kneeError = 0;
                            Vec3 hipError {};
                            const std::size_t shinLink = static_cast<std::size_t>(profile.links[feet[side]].parentIndex);
                            const std::size_t thighLink = static_cast<std::size_t>(profile.links[shinLink].parentIndex);
                            for (const auto& drive : locomotion.output().driveTargets) {
                                const std::size_t axis = static_cast<std::size_t>(drive.axis);
                                if (drive.linkIndex == shinLink && axis == 0)
                                    kneeError = (drive.positionRadians - after.joints[shinLink].positionRadians[0]) * 57.2958f;
                                if (drive.linkIndex == thighLink) {
                                    const float e = (drive.positionRadians - after.joints[thighLink].positionRadians[axis]) * 57.2958f;
                                    if (axis == 0) hipError.x = e; else if (axis == 1) hipError.y = e; else hipError.z = e;
                                }
                            }
                            const Vec3 pelvisForward = after.links.front().orientation.rotate({1, 0, 0});
                            const Vec3 shinAxis = after.links[shinLink].orientation.rotate({0, 0, 1});
                            if (std::getenv("XTOUCH"))
                                std::printf("  cadeia: joelho %+.1f quadril (%+.1f %+.1f %+.1f) pelve %+.1f canela %+.1f\n", kneeError,
                                    hipError.x, hipError.y, hipError.z,
                                    -std::asin(std::clamp(pelvisForward.z, -1.0f, 1.0f)) * 57.2958f,
                                    std::atan2(shinAxis.x, shinAxis.z) * 57.2958f);
                            if (std::getenv("XTOUCH"))
                                std::printf("TOQUE t %.2f lado %zu x %.2f sola %+.1f [clipe %+.1f alvo %+.1f mundo %+.1f] tornozelo %+.1f progresso %.2f no ar %.2f\n", t, side,
                                    foot.position.x, result.touchdowns.back().pitch,
                                    locomotion.telemetry().footClipPitch[side], locomotion.telemetry().footGoalPitch[side],
                                    -std::asin(std::clamp(forward.z, -1.0f, 1.0f)) * 57.2958f, ankleError,
                                    locomotion.telemetry().footSwingProgress[side], result.lastAir[side]);
                        }
                    }
                    result.touchdownContact[side] = seen;
                    if (!seen) result.lastAir[side] = result.touchdownAirSeconds[side];
                }
            }
            if (t >= 1.5f) {
                const Vec3 pelvisNow = after.links.front().position;
                const float vz = after.links.front().linearVelocity.z;
                {
                    const Vec3 forward = after.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                    float error = std::atan2(forward.y, forward.x) - wanted.lookYaw;
                    while (error > 3.14159265f) error -= 6.28318531f;
                    while (error < -3.14159265f) error += 6.28318531f;
                    if (result.turnAlignedAt < 0.0f && std::abs(error) < 0.35f && t > 1.55f)
                        result.turnAlignedAt = t;
                    result.turnYaw.emplace_back(t, std::atan2(forward.y, forward.x));
                    if (std::getenv("XTURNTRACE") && tick % 6 == 0 && t < 3.5f) {
                        const auto& tel = locomotion.telemetry();
                        const auto& plan = locomotion.contactPlan();
                        std::printf("GIRO %.2f olhar %+.0f pelve-fis %+.0f rumo-ref %+.0f taxa %+.2f estado %d | pes %d/%d razao %d/%d | recuperando %d lado %d urg %.2f captura-fora %.3f | base %+.0f torcao %+.0f falta %+.0f pendente?\n",
                            t, wanted.lookYaw * 57.3f, std::atan2(forward.y, forward.x) * 57.3f, tel.facingYawRadians * 57.3f,
                            tel.turningRateRadiansPerSecond, static_cast<int>(tel.state),
                            static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase),
                            static_cast<int>(plan.feet[0].reason), static_cast<int>(plan.feet[1].reason),
                            plan.recovering ? 1 : 0, plan.recoverySide, plan.recoveryUrgency, tel.captureOutsideMeters,
                            plan.standingStanceYaw * 57.3f, plan.standingTwist * 57.3f, plan.standingTurnRemaining * 57.3f);
                    }
                    const Vec3 up = after.links[chest].orientation.rotate({ 0.0f, 0.0f, 1.0f });
                    result.turnWorstTilt = std::max(result.turnWorstTilt,
                        std::acos(std::clamp(up.z, -1.0f, 1.0f)) * 57.2958f);
                }
                result.idleVerticalSquared += vz * vz;
                ++result.idleVerticalSamples;
                result.idleVerticalPeak = std::max(result.idleVerticalPeak, std::abs(vz));
                const auto& ps = locomotion.physicalState();
                const auto& plan = locomotion.contactPlan();
                for (std::size_t side = 0; side < 2; ++side) {
                    const bool seen = ps.feet[side].contactObserved;
                    const bool planned = plan.feet[side].phase == FootPhase3D::Stance;
                    if (planned && result.idleContactBefore[side] && !seen) ++result.idleContactLosses;
                    result.idleContactBefore[side] = seen;
                }
                if (!result.pelvisAt15Set) { result.pelvisAt15 = pelvisNow; result.pelvisAt15Set = true; }
                result.standingTravelAfter15 = std::max(result.standingTravelAfter15,
                    std::hypot(pelvisNow.x - result.pelvisAt15.x, pelvisNow.y - result.pelvisAt15.y));
            }
            if (t >= 1.5f && t <= 1.8f)
                result.captureOutsidePeakAfterPush = std::max(result.captureOutsidePeakAfterPush,
                    locomotion.telemetry().captureOutsideMeters);
            if (locomotion.telemetry().balanceReaction > 0.3f) ++result.balanceReactionTicks;
            result.worstUpwardSpeed = std::max(result.worstUpwardSpeed,
                after.links.front().linearVelocity.z);
            result.worstVerticalStep = std::max(result.worstVerticalStep,
                std::abs(after.links.front().position.z - state.links.front().position.z));
        }
        if (down && !wasDown) {
            ++result.falls;
            if (result.firstFallSeconds < 0) {
                result.firstFallSeconds = t;
                result.gaitFallPhase = result.gaitMoveStart < 0 ? 0
                    : result.gaitMoveEnd >= 0 && t > result.gaitMoveEnd + 0.05f ? 3
                    : result.gaitReverseAt >= 0 && t < result.gaitReverseAt + 1.0f ? 4
                    : t - result.gaitMoveStart < 1.0f ? 1 : 2;
            }
            else ++result.relapses;
        }
        if (!down && wasDown && result.getUpSeconds < 0 && result.firstFallSeconds >= 0) {
            result.getUpSeconds = t - result.firstFallSeconds;
            handoffAt = t;
            result.handoffTilt = std::acos(std::clamp(after.links.front().orientation.rotate({0, 0, 1}).z,
                -1.0f, 1.0f)) * 57.2958f;
            result.handoffChestTilt = std::acos(std::clamp(dot(after.links[chest].orientation.rotate({0, 0, 1}),
                output.targetPose.linkOrientations[chest].rotate({0, 0, 1})), -1.0f, 1.0f)) * 57.2958f;
            const Vec3 d = after.links[feet[0]].position - after.links[feet[1]].position;
            handoffSpread = std::hypot(d.x, d.y);
        }
        if (std::getenv("XH") && handoffAt >= 0 && t - handoffAt < 0.6f)
            std::printf("XH t %.3f pes %.2f %.2f | z %.2f %.2f | trav %d%d estado %d\n", t,
                after.links[feet[0]].linearVelocity.length(), after.links[feet[1]].linearVelocity.length(),
                after.links[feet[0]].position.z, after.links[feet[1]].position.z,
                locomotion.telemetry().footPlanted[0] ? 1 : 0, locomotion.telemetry().footPlanted[1] ? 1 : 0,
                static_cast<int>(locomotion.telemetry().state));
        if (handoffAt >= 0 && t - handoffAt < 2.0f && !down) {
            // Desvio da pose que ele deveria ter (pelve e peito), nao a
            // inclinacao absoluta: parado, o peito ja fica a ~25 graus.
            const auto offPose = [&](std::size_t link) {
                return std::acos(std::clamp(dot(after.links[link].orientation.rotate({0, 0, 1}),
                    output.targetPose.linkOrientations[link].rotate({0, 0, 1})), -1.0f, 1.0f)) * 57.2958f;
            };
            if (std::getenv("XPOS") && tick % 6 == 0)
                std::printf("XPOS t %.2f pelve %.1f peito %.1f desequilibrio %.2f pert %.2f estado %d pes %d%d\n", t,
                    offPose(0), offPose(chest), locomotion.telemetry().balanceReaction, physical.perturbation,
                    static_cast<int>(locomotion.telemetry().state), locomotion.telemetry().footPlanted[0] ? 1 : 0,
                    locomotion.telemetry().footPlanted[1] ? 1 : 0);
            result.afterGetUpWorstTilt = std::max({result.afterGetUpWorstTilt, offPose(0), offPose(chest)});
        }
        if (handoffAt >= 0 && t - handoffAt < 0.6f) {
            for (std::size_t side = 0; side < 2; ++side)
                result.handoffFootSpeed = std::max(result.handoffFootSpeed,
                    after.links[feet[side]].linearVelocity.length());
            const Vec3 d = after.links[feet[0]].position - after.links[feet[1]].position;
            result.handoffSpreadGrowth = std::max(result.handoffSpreadGrowth,
                std::hypot(d.x, d.y) - handoffSpread);
        }
        if (down && locomotion.telemetry().getUpPhase != CharacterGetUpPhase3D::Rising) {
            // Passivo (antes de se arrumar) conta como cambalhota; o rolar de
            // se arrumar para levantar (de lado para de costas) e a parte.
            // So o giro rapido (> 0,8 rad/s) e rolar: os movimentos lentos de
            // quem se mexe deitado (cabeca, cotovelos, joelhos) nao contam.
            Vec3 spin = after.links.front().angularVelocity;
            spin.z = 0;
            if (spin.length() > 0.8f)
                (locomotion.telemetry().settleGather < 0.05f ? result.settleTumbleDegrees
                    : result.settleGatherDegrees) += spin.length() * dt * 57.2958f;
        }
        wasDown = down;
        {
            // Caindo: o primeiro segmento (fora os pes) que toca o chao e a
            // velocidade da cabeca quando ela toca.
            const float uprightNow = after.links.front().orientation.rotate({0, 0, 1}).z;
            const bool falling = down || uprightNow < 0.8f;
            for (const auto& contact : after.interactions) {
                if (contact.normalWorld.z < 0.5f || contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id == "LeftFoot" || id == "RightFoot") continue;
                if (falling && result.firstGroundLink.empty()) result.firstGroundLink = id;
                if (id == "Head" && result.headImpactSpeed == 0)
                    result.headImpactSpeed = std::max(0.001f, after.links[contact.linkIndex].linearVelocity.length());
            }
            if (const char* dump = std::getenv("XDUMPTAU")) {
                // "cenario:tau1,tau2,..." -> estado fisico quando o tempo do
                // clipe de levantar passa por cada tau (s).
                const std::string spec = dump;
                const auto colon = spec.find(':');
                const auto& tel = locomotion.telemetry();
                if (colon != std::string::npos && spec.substr(0, colon) == scenario.name
                    && tel.getUpPhase == CharacterGetUpPhase3D::Rising) {
                const auto* recoveryClip = tel.fallOrientation == CharacterFallOrientation3D::FaceDown
                    ? clips.animations.standUpFront : clips.animations.standUpBack;
                const float tau = tel.getUpProgress * (recoveryClip ? recoveryClip->durationSeconds : 1.6f);
                    std::size_t start = colon + 1;
                    int index = 0;
                    while (start < spec.size()) {
                        const std::size_t comma = spec.find(',', start);
                        const float when = std::strtof(spec.substr(start, comma - start).c_str(), nullptr);
                        if (tau >= when && !(dumpedTau & (1u << index))) {
                            dumpedTau |= 1u << index;
                            std::printf("XDUMP %.2f", when);
                            for (const auto& link : after.links)
                                std::printf(" %.4f %.4f %.4f %.5f %.5f %.5f %.5f", link.position.x, link.position.y,
                                    link.position.z, link.orientation.x, link.orientation.y, link.orientation.z,
                                    link.orientation.w);
                            std::printf("\n");
                        }
                        ++index;
                        if (comma == std::string::npos) break;
                        start = comma + 1;
                    }
                }
            }
            if (const char* dump = std::getenv("XDUMP")) {
                // "cenario:t1,t2,..." -> estado fisico em JSON (posicoes e
                // orientacoes dos links no mundo) para desenhar.
                const std::string spec = dump;
                const auto colon = spec.find(':');
                if (colon != std::string::npos && spec.substr(0, colon) == scenario.name) {
                    std::size_t start = colon + 1;
                    while (start < spec.size()) {
                        const std::size_t comma = spec.find(',', start);
                        const float when = std::strtof(spec.substr(start, comma - start).c_str(), nullptr);
                        if (std::abs(t - when) < 0.5f / 120) {
                            std::printf("XDUMP %.2f", t);
                            for (const auto& link : after.links)
                                std::printf(" %.4f %.4f %.4f %.5f %.5f %.5f %.5f", link.position.x, link.position.y,
                                    link.position.z, link.orientation.x, link.orientation.y, link.orientation.z,
                                    link.orientation.w);
                            std::printf("\n");
                        }
                        if (comma == std::string::npos) break;
                        start = comma + 1;
                    }
                }
            }
            if (std::getenv("XA") && down && tick % 12 == 0) {
                // Erro das juntas dos bracos contra o alvo (graus), por eixo.
                std::printf("XA t %.2f tau %.2f", t, locomotion.telemetry().getUpProgress * 2.5f);
                for (const auto& target : output.driveTargets) {
                    const std::string& id = profile.links[target.linkIndex].id;
                    if (id != "LeftUpperArm" && id != "LeftForearm" && id != "RightUpperArm"
                        && id != "RightForearm") continue;
                    const float measured = after.joints[target.linkIndex].positionRadians[static_cast<int>(target.axis)];
                    std::printf(" %s%d %.0f/%.0f", id.substr(0, id.find("Arm") == std::string::npos ? 5 : 6).c_str(),
                        static_cast<int>(target.axis), target.positionRadians * 57.3f, measured * 57.3f);
                }
                std::printf("\n");
            }
            if (std::getenv("XC") && t > 1.85f && t < 1.97f) {
                for (const auto& c : state.contacts)
                    if (c.normal.z >= 0.4f && c.normalImpulseNewtonSeconds > 1e-5f)
                        std::printf("XC t %.3f link %s n %.2f %.2f %.2f imp %.3f din %d pos z %.3f\n", t,
                            profile.links[c.linkIndex].id.c_str(), c.normal.x, c.normal.y, c.normal.z,
                            c.normalImpulseNewtonSeconds, c.otherBodyDynamic ? 1 : 0, c.position.z);
            }
            if (std::getenv("XJ") && t > (std::getenv("XJFROM") ? std::atof(std::getenv("XJFROM")) : 1.4f)
                && t < (std::getenv("XJTO") ? std::atof(std::getenv("XJTO")) : 3.2f)
                && tick % (std::getenv("XFINE") ? 1 : 3) == 0) {
                // Pulo e aterrissagem.
                const auto& tel = locomotion.telemetry();
                Vec3 f0 = after.links[feet[0]].position, f1 = after.links[feet[1]].position;
                int footSensor = 0;
                for (const auto& c : after.contacts)
                    if (c.normal.z >= 0.4f && c.normalImpulseNewtonSeconds > 1e-5f
                        && (c.linkIndex == feet[0] || c.linkIndex == feet[1])) footSensor = 1;
                std::printf("XJ t %.3f sensor-pe %d estado %d cap %d | pelve z %.2f vz %+.2f vx %+.2f tilt %.0f | pes z %.2f %.2f x %+.2f %+.2f trav %d%d"
                    " | com-x %+.2f capt %.2f sup %.2f pert %.2f | ref-x %+.2f ref-z %+.2f fz %.0f fx %.0f | cap-vx %+.2f segue %+.2f refv %+.2f\n", t, footSensor,
                    static_cast<int>(tel.state), capsule.grounded ? 1 : 0, after.links.front().position.z,
                    after.links.front().linearVelocity.z, after.links.front().linearVelocity.x,
                    std::acos(std::clamp(uprightNow, -1.0f, 1.0f)) * 57.3f, f0.z, f1.z,
                    f0.x - after.links.front().position.x, f1.x - after.links.front().position.x,
                    tel.footPlanted[0] ? 1 : 0, tel.footPlanted[1] ? 1 : 0,
                    scene->ragdollDynamics(handle).centerOfMass.x - after.links.front().position.x,
                    physical.captureErrorMeters, physical.supportAuthority, physical.perturbation,
                    output.guide.rootPositionWorld.x - after.links.front().position.x,
                    output.guide.rootPositionWorld.z - after.links.front().position.z,
                    physical.rootForceWorld.z, physical.rootForceWorld.x,
                    capsule.velocity.x, follow.x, output.guide.rootLinearVelocityWorld.x);
            }
            if (std::getenv("XGU") && down && locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Rising
                && tick % 6 == 0) {
                const auto& tel = locomotion.telemetry();
                const Vec3 fwd3 = after.links.front().orientation.rotate({1, 0, 0});
                Vec3 fwd {fwd3.x, fwd3.y, 0};
                fwd = fwd.lengthSquared() > 1e-6f ? fwd.normalized() : Vec3{1, 0, 0};
                const Vec3 com = scene->ragdollDynamics(handle).centerOfMass;
                const Vec3 p = after.links.front().position;
                const auto along = [&](Vec3 v) { return dot(Vec3{v.x - p.x, v.y - p.y, 0}, fwd); };
                std::printf("XGU t %.2f prog %.2f ritmo %.2f | pelve z %.2f tilt %.0f excesso %.0f torque %.0f sobre-apoio %.2f"
                    " | frente da pelve: com %+.2f peito %+.2f pes %+.2f %+.2f maos %+.2f %+.2f\n", t,
                    tel.getUpProgress, tel.getUpRate, p.z,
                    std::acos(std::clamp(uprightNow, -1.0f, 1.0f)) * 57.3f, tel.getUpTiltExcessDegrees,
                    tel.getUpUprightTorque, tel.getUpOverSupport, along(com), along(after.links[chest].position),
                    along(after.links[feet[0]].position), along(after.links[feet[1]].position),
                    along(after.links[hands[0]].position), along(after.links[hands[1]].position));
            }
            if (std::getenv("XTRANSITION")) {
                // Historico curto da velocidade no rumo pedido e os eventos.
                static std::vector<float> history;
                static std::array<int, 2> previousPhase { 0, 0 };
                struct Pending { bool open = false; int side = 0; float v0 = 0, v1 = 0, v2 = -99, v3 = -99; float touchdownAt = 0; };
                static Pending pending;
                // Depois da saida: a velocidade 0,1 s mais tarde (comeco do
                // apoio simples).
                static float releaseAt = -1, releaseSpeed = 0;
                if (tick == 0) { history.clear(); previousPhase = { 0, 0 }; pending = {}; releaseAt = -1; }
                if (releaseAt >= 0 && t - releaseAt >= 0.10f) {
                    const Vec3 rq = input.intent.requestedVelocityWorld;
                    const float rs = std::hypot(rq.x, rq.y);
                    const Vec3 vv = locomotion.physicalState().centerOfMassVelocityWorld;
                    if (rs > 0.2f) {
                        result.earlySingleLoss += (vv.x * rq.x + vv.y * rq.y) / rs - releaseSpeed;
                    }
                    releaseAt = -1;
                }
                const Vec3 requested = input.intent.requestedVelocityWorld;
                const float speed = std::hypot(requested.x, requested.y);
                const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                const float along = speed > 0.2f ? (v.x * requested.x + v.y * requested.y) / speed : 0.0f;
                history.push_back(along);
                const auto& plan = locomotion.contactPlan();
                const auto& physicalNow = locomotion.physicalState();
                for (int side = 0; side < 2; ++side) {
                    const int phase = static_cast<int>(plan.feet[side].phase);
                    const int before = previousPhase[static_cast<std::size_t>(side)];
                    const bool locomotion = plan.feet[side].reason == StepReason3D::Locomotion && speed > 0.2f
                        && t > 3.0f && t < 5.0f;
                    // toque: Swing/Busca -> Carga
                    if (locomotion && (before == 2 || before == 3) && phase == 4) {
                        const int back = static_cast<int>(std::lround(0.10f / dt));
                        pending = {};
                        pending.open = true;
                        pending.side = side;
                        pending.v0 = history.size() > static_cast<std::size_t>(back) ? history[history.size() - 1 - static_cast<std::size_t>(back)] : along;
                        pending.v1 = along;
                        pending.touchdownAt = t;
                    }
                    // aceitacao: Carga -> Apoio (o pe que tocou)
                    if (pending.open && side == pending.side && before == 4 && phase == 0) pending.v3 = along;
                    // saida do outro pe: (Apoio|Descarga) -> Balanco. Com a
                    // dupla sustentacao curta ela pode vir antes dos 50 ms.
                    if (pending.open && side != pending.side && (before == 0 || before == 1) && phase == 2) {
                        if (pending.v2 < -90) pending.v2 = along;
                        if (pending.v3 < -90) pending.v3 = along;
                        {
                            ++result.transitions;
                            result.pushGain += pending.v1 - pending.v0;
                            result.collisionLoss += pending.v2 - pending.v1;
                            result.acceptanceLoss += pending.v3 - pending.v2;
                            result.transitionNet += along - pending.v0;
                            releaseAt = t;
                            releaseSpeed = along;
                        }
                        pending.open = false;
                    }
                    const float eventFrom = std::getenv("XEVTFROM") ? std::strtof(std::getenv("XEVTFROM"), nullptr) : 3.0f;
                    if (std::getenv("XTRANSITION")[0] == '2' && phase != before && t > eventFrom && t < 5.0f) {
                        // Pe (ancora) e DCM em relacao ao COM no rumo pedido (m).
                        const Vec3 com = physicalNow.centerOfMassWorld;
                        const float omega = std::clamp(physicalNow.captureOmega, 1.0f, 6.0f);
                        const Vec3 foot = plan.feet[side].anchorWorld;
                        const float footAlong = speed > 0.2f ? ((foot.x - com.x) * requested.x + (foot.y - com.y) * requested.y) / speed : 0.0f;
                        const float dcmAlong = speed > 0.2f ? (v.x * requested.x + v.y * requested.y) / speed / omega : 0.0f;
                        std::printf("EVT t %.3f pe %d %d->%d razao %d v %.2f | pe-COM %+.3f DCM-COM %+.3f carga %.2f\n", t, side, before, phase,
                            static_cast<int>(plan.feet[side].reason), along, footAlong, dcmAlong, plan.loadShare[static_cast<std::size_t>(side)]);
                    }
                    previousPhase[static_cast<std::size_t>(side)] = phase;
                }
                if (pending.open && pending.v2 < -90 && t - pending.touchdownAt >= 0.05f) pending.v2 = along;
            }
            if (std::getenv("XSWING")) {
                struct SwingWatch { bool open = false; std::uint32_t id = 0; bool has35 = false;
                    Vec3 goal35 {}, lastGoal {}, previousTarget {}, previousVelocity {};
                    float revision = 0, recede = 0, accelPeak = 0; double tracking = 0; int ticks = 0, samples = 0; };
                static std::array<SwingWatch, 2> watch;
                if (tick == 0) watch = {};
                const auto& plan = locomotion.contactPlan();
                const Vec3 requested = input.intent.requestedVelocityWorld;
                const float speed = std::hypot(requested.x, requested.y);
                if (t > 2.5f && t < 5.0f && speed > 0.2f) {
                    const auto& ps = locomotion.physicalState();
                    if (!ps.feet[0].supporting && !ps.feet[1].supporting) ++result.flightTicks;
                    if (!ps.feet[0].contactObserved && !ps.feet[1].contactObserved) ++result.strictFlightTicks;
                    // XSWING=3: cada voo inesperado (inicio e duracao), com a fase
                    // dos pes e o alcance das pernas no tick anterior.
                    static bool wasFlying = false;
                    static float flightStart = 0.0f;
                    static std::array<LegReachTelemetry3D, 2> lastReach {};
                    static std::array<int, 2> lastPhase {};
                    const bool flying = plan.observedSchedule == SupportSchedule3D::UnexpectedNoSupport;
                    if (std::getenv("XSWING")[0] == '3') {
                        if (flying && !wasFlying) {
                            flightStart = t;
                            std::printf("VOO t %.3f fases %d/%d (antes %d/%d) | alcance antes: E fis %.3f alvo %.3f estica %+.2f | D fis %.3f alvo %.3f estica %+.2f | pelve fisica z %.3f vz %+.2f\n",
                                t, static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase), lastPhase[0], lastPhase[1],
                                lastReach[0].physicalRatio, lastReach[0].referenceRatio, lastReach[0].referenceAxialSpeed,
                                lastReach[1].physicalRatio, lastReach[1].referenceRatio, lastReach[1].referenceAxialSpeed,
                                after.links.front().position.z, after.links.front().linearVelocity.z);
                        }
                        if (!flying && wasFlying) std::printf("VOO fim t %.3f duracao %.0f ms\n", t, (t - flightStart) * 1000.0f);
                    }
                    wasFlying = flying;
                    lastReach = locomotion.telemetry().legReach;
                    lastPhase = { static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase) };
                    for (std::size_t side = 0; side < 2; ++side) {
                        const FootPhase3D phase = plan.feet[side].phase;
                        const int slot = phase == FootPhase3D::Stance ? 0 : phase == FootPhase3D::Unloading ? 1
                            : phase == FootPhase3D::Loading ? 2 : -1;
                        if (slot >= 0 && !ps.feet[side].contactObserved) ++result.strictFakeTicks[static_cast<std::size_t>(slot)];
                    }
                    for (std::size_t side = 0; side < 2; ++side) {
                        const FootPhase3D phase = plan.feet[side].phase;
                        if ((phase == FootPhase3D::Stance || phase == FootPhase3D::Unloading
                                || phase == FootPhase3D::Loading) && !ps.feet[side].supporting)
                            ++result.fictitiousSupportTicks;
                    }
                }
                for (std::size_t side = 0; side < 2; ++side) {
                    const FootPlan3D& foot = plan.feet[side];
                    SwingWatch& w = watch[side];
                    const bool swinging = foot.reason == StepReason3D::Locomotion
                        && (foot.phase == FootPhase3D::Swing || foot.phase == FootPhase3D::TouchdownSearch);
                    if (swinging && w.id != foot.stepId) {
                        w = {};
                        w.open = t > 2.5f && t < 5.0f && speed > 0.2f;
                        w.id = foot.stepId;
                        if (w.open && std::getenv("XSWING")[0] == '2') {
                            const Vec3 d = foot.liftoffWorld - foot.anchorWorld;
                            std::printf("SOLTURA t %.3f pe %zu liftoff-ancora (%.3f %.3f %.3f) ao longo %+.3f\n", t, side,
                                d.x, d.y, d.z, speed > 0.2f ? (d.x * requested.x + d.y * requested.y) / speed : 0.0f);
                        }
                    }
                    const Vec3 physical = after.links[feet[side]].position;
                    if (w.open && swinging) {
                        const float progress = foot.swingSeconds / std::max(0.05f, foot.swingDurationSeconds);
                        const Vec3 target { foot.targetWorld.x, foot.targetWorld.y, 0.0f };
                        // O tick da soltura ainda tem o alvo do apoio (a
                        // ancora): o salto dali ate o pe medido e contado a
                        // parte; a aceleracao e so dentro do balanco.
                        if (w.samples == 0) {
                            result.releaseJump += std::hypot(foot.liftoffWorld.x - foot.anchorWorld.x,
                                foot.liftoffWorld.y - foot.anchorWorld.y);
                            ++w.samples;
                        } else if (w.samples == 1) {
                            w.previousTarget = target;
                            ++w.samples;
                        } else if (w.samples >= 2) {
                            const Vec3 velocity = (target - w.previousTarget) * (1.0f / dt);
                            if (w.samples >= 3) {
                                const float accel = (velocity - w.previousVelocity).length() / dt;
                                w.accelPeak = std::max(w.accelPeak, accel);
                                if (std::getenv("XSWING")[0] == '2' && accel > 300.0f)
                                    std::printf("SWINGJUMP t %.3f pe %zu fase %d via %d prog %.2f acel %.0f alvo (%.3f %.3f) meta (%.3f %.3f) v (%.2f %.2f) antes (%.2f %.2f)\n",
                                        t, side, static_cast<int>(foot.phase), static_cast<int>(foot.path), progress, accel,
                                        target.x, target.y, foot.goalWorld.x, foot.goalWorld.y,
                                        velocity.x, velocity.y, w.previousVelocity.x, w.previousVelocity.y);
                            }
                            w.previousVelocity = velocity;
                            w.previousTarget = target;
                            ++w.samples;
                        }
                        if (progress >= 0.35f && !w.has35) { w.goal35 = foot.goalWorld; w.has35 = true; }
                        if (w.has35) {
                            const Vec3 moved { foot.goalWorld.x - w.goal35.x, foot.goalWorld.y - w.goal35.y, 0.0f };
                            w.revision = std::max(w.revision, moved.length());
                            if (speed > 0.2f)
                                w.recede = std::max(w.recede, -(moved.x * requested.x + moved.y * requested.y) / speed);
                        }
                        w.lastGoal = foot.goalWorld;
                        w.tracking += std::hypot(physical.x - target.x, physical.y - target.y);
                        ++w.ticks;
                    }
                    if (w.open && !swinging) {
                        ++result.swings;
                        result.swingRevision += w.revision;
                        result.swingRecede += w.recede;
                        result.swingRecedeWorst = std::max(result.swingRecedeWorst, w.recede);
                        if (w.recede >= 0.20f) ++result.swingsReceding20;
                        result.swingAccelPeak += w.accelPeak;
                        result.swingAccelWorst = std::max(result.swingAccelWorst, w.accelPeak);
                        result.swingTracking += w.ticks ? w.tracking / w.ticks : 0.0;
                        result.swingLandingError += std::hypot(physical.x - w.lastGoal.x, physical.y - w.lastGoal.y);
                        w.open = false;
                    }
                }
            }
            if (std::getenv("XTIMEOUTLOG")) {
                static std::uint32_t lastTimeouts = 0;
                const auto& plan = locomotion.contactPlan();
                if (plan.locomotionTimeoutReleases < lastTimeouts) lastTimeouts = 0;
                if (plan.locomotionTimeoutReleases > lastTimeouts) {
                    const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                    std::printf("PRAZO t %.2f passo %u do pe %d | v (%+.2f %+.2f) | carga %.2f %.2f\n", t,
                        plan.feet[0].phase == FootPhase3D::Swing ? plan.feet[0].stepId : plan.feet[1].stepId,
                        plan.feet[0].phase == FootPhase3D::Swing ? 0 : 1, v.x, v.y, plan.loadShare[0], plan.loadShare[1]);
                    lastTimeouts = plan.locomotionTimeoutReleases;
                }
            }
            if (std::getenv("XBENCHC") && tick % 6 == 0 && t > 2.2f) {
                const auto& ps = locomotion.physicalState();
                const auto& plan = locomotion.contactPlan();
                Vec3 axis = ps.feet[1].soleCenterWorld - ps.feet[0].soleCenterWorld;
                axis.z = 0.0f;
                const Vec3 com = ps.centerOfMassWorld - ps.feet[0].soleCenterWorld;
                const float along = axis.lengthSquared() > 1e-4f ? (com.x * axis.x + com.y * axis.y) / axis.lengthSquared() : 0.5f;
                const float lift0 = ps.feet[0].soleLowestHeight - plan.feet[0].surface.height;
                const float lift1 = ps.feet[1].soleLowestHeight - plan.feet[1].surface.height;
                std::printf("BENCHC t %.2f banc %+d | fases %d%d apoiado %d%d | COM entre pes (0 = esq, 1 = dir) %.2f | carga esq %.2f | sola acima do chao %.3f %.3f | transf %d->%d %.2f | v (%+.2f %+.2f)\n",
                    t, locomotion.footworkSettings().benchTransferFoot,
                    static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase),
                    ps.feet[0].supporting ? 1 : 0, ps.feet[1].supporting ? 1 : 0, along, plan.loadShare[0], lift0, lift1,
                    plan.supportTransfer.outgoingFoot, plan.supportTransfer.incomingFoot, plan.supportTransfer.progress,
                    ps.centerOfMassVelocityWorld.x, ps.centerOfMassVelocityWorld.y);
            }
            // Telemetria do motor (E5, fase 1): quadril (flexao), joelho e
            // tornozelo das duas pernas - alvo/medido (posicao e velocidade),
            // torque do motor, feedforward, saturacao do orcamento que sobrou
            // (o ff consome o mesmo limite). XMOTORTRACE imprime por tick
            // (XMOTOREVERY, XGTFROM/XGTTO); XMOTORSUM resume por papel do pe.
            if (std::getenv("XMOTORTRACE") || std::getenv("XMOTORSUM")) {
                const auto& plan = locomotion.contactPlan();
                const char* names[2][3] = { { "LeftThigh", "LeftShin", "LeftFoot" }, { "RightThigh", "RightShin", "RightFoot" } };
                const int axes[3] = { 1, 0, 0 };
                const float from = std::getenv("XGTFROM") ? std::strtof(std::getenv("XGTFROM"), nullptr) : 1.8f;
                const float to = std::getenv("XGTTO") ? std::strtof(std::getenv("XGTTO"), nullptr) : 5.5f;
                const int every = std::getenv("XMOTOREVERY") ? std::atoi(std::getenv("XMOTOREVERY")) : 2;
                const bool print = std::getenv("XMOTORTRACE") && t >= from && t <= to && tick % every == 0;
                for (std::size_t side = 0; side < 2; ++side) {
                    const FootPlan3D& foot = plan.feet[side];
                    const bool supportingPhase = foot.phase == FootPhase3D::Stance || foot.phase == FootPhase3D::Loading
                        || foot.phase == FootPhase3D::Unloading;
                    char line[512];
                    int written = std::snprintf(line, sizeof line, "XMT %.3f pe %zu f%d r%d ap%d |", t, side,
                        static_cast<int>(foot.phase), static_cast<int>(foot.reason),
                        locomotion.physicalState().feet[side].supporting ? 1 : 0);
                    for (int j = 0; j < 3; ++j) {
                        std::size_t link = 0;
                        for (std::size_t i = 0; i < profile.links.size(); ++i)
                            if (profile.links[i].id == names[side][j]) link = i;
                        if (link == 0 || link >= after.joints.size()) continue;
                        const auto& js = after.joints[link];
                        const std::size_t a = static_cast<std::size_t>(axes[j]);
                        const float motor = js.motorTorqueNewtonMeters[a];
                        const float ff = js.feedforwardTorqueNewtonMeters[a];
                        const float limit = js.motorTorqueLimitNewtonMeters[a];
                        const float room = motor >= 0.0f ? limit - ff : -limit - ff;
                        const float saturation = std::abs(room) > 1e-3f ? std::clamp(motor / room, 0.0f, 9.99f) : 1.0f;
                        if (t > 2.0f && t < 5.0f) {
                            auto& summary = result.motorSummary[side * 3 + static_cast<std::size_t>(j)][supportingPhase ? 0 : 1];
                            summary.ticks += 1;
                            summary.maxSaturation = std::max(summary.maxSaturation, saturation);
                            summary.saturatedTicks += saturation > 0.95f ? 1 : 0;
                            summary.sumAbsMotor += std::abs(motor);
                            summary.sumAbsFeedforward += std::abs(ff);
                        }
                        if (print && written < static_cast<int>(sizeof line))
                            written += std::snprintf(line + written, sizeof line - static_cast<std::size_t>(written),
                                " %s %+.2f/%+.2f v %+5.1f/%+5.1f m %+5.0f ff %+5.0f sat %.2f |", j == 0 ? "Q" : j == 1 ? "J" : "T",
                                js.targetPositionRadians[a], js.positionRadians[a], js.targetVelocityRadiansPerSecond[a],
                                js.velocityRadiansPerSecond[a], motor, ff, saturation);
                    }
                    if (print) std::printf("%s\n", line);
                }
            }
            if (std::getenv("XGAITTRACE") && tick % (std::getenv("XGTEVERY") ? std::atoi(std::getenv("XGTEVERY")) : 6) == 0 && t > (std::getenv("XGTFROM") ? std::strtof(std::getenv("XGTFROM"), nullptr) : 1.8f) && t < 5.5f) {
                // Marcha (E5): rumo do corpo contra a referencia, velocidade do
                // COM e os dois pes (fase, razao, posicao em relacao a pelve).
                const auto& tel = locomotion.telemetry();
                const auto yawOf = [](Quaternion q) {
                    const Vec3 f = q.rotate({1, 0, 0});
                    return std::atan2(f.y, f.x) * 57.3f;
                };
                const Vec3 p = locomotion.physicalState().centerOfMassWorld;
                const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                const Vec3 f0 = after.links[feet[0]].position - p;
                const Vec3 f1 = after.links[feet[1]].position - p;
                std::printf("XGT t %.2f e%d | COM (%+.2f %+.2f %.2f) v (%+.2f %+.2f) | rumo corpo %+4.0f ref %+4.0f facing %+4.0f"
                    " | pe-COM: pe0 f%d r%d (%+.2f %+.2f %.2f) pe1 f%d r%d (%+.2f %+.2f %.2f)\n", t,
                    static_cast<int>(tel.state), p.x, p.y, p.z, v.x, v.y, yawOf(after.links.front().orientation),
                    yawOf(output.guide.rootOrientationWorld), tel.facingYawRadians * 57.3f,
                    static_cast<int>(locomotion.contactPlan().feet[0].phase), static_cast<int>(locomotion.contactPlan().feet[0].reason),
                    f0.x, f0.y, f0.z,
                    static_cast<int>(locomotion.contactPlan().feet[1].phase), static_cast<int>(locomotion.contactPlan().feet[1].reason),
                    f1.x, f1.y, f1.z);
                const Vec3 cap = locomotion.physicalState().capturePointWorld - p;
                const Vec3 w0 = physical.contactWrench[0].forceWorld;
                const Vec3 w1 = physical.contactWrench[1].forceWorld;
                {
                    const auto& ps = locomotion.physicalState();
                    float lo[2] = {1e9f, 1e9f}, hi[2] = {-1e9f, -1e9f};
                    for (std::uint32_t k = 0; k < ps.supportPointCount; ++k) {
                        const Vec3 d = ps.supportPointsWorld[k] - p;
                        lo[0] = std::min(lo[0], d.x); hi[0] = std::max(hi[0], d.x);
                        lo[1] = std::min(lo[1], d.y); hi[1] = std::max(hi[1], d.y);
                    }
                    const Vec3 chestUp = after.links[chest].orientation.rotate({0, 0, 1});
                    const Vec3 pelvisUp = after.links.front().orientation.rotate({0, 0, 1});
                    const auto& plan = locomotion.contactPlan();
                    const Vec3 g0 = plan.feet[0].goalWorld - p, g1 = plan.feet[1].goalWorld - p;
                    const Vec3 trueCom = scene->ragdollDynamics(handle).centerOfMass - p;
                    std::printf("XGT   COM fisica-estimado (%+.3f %+.3f %+.3f) contatos %zu:", trueCom.x, trueCom.y, trueCom.z, state.contacts.size());
                    for (const auto& c : state.contacts)
                        std::printf(" [L%u (%+.2f %+.2f) %.0fN]", c.linkIndex, c.position.x - p.x, c.position.y - p.y,
                            c.normalImpulseNewtonSeconds / dt);
                    std::printf(" | pes L%zu L%zu\n", feet[0], feet[1]);
                    std::printf("XGT   apoio-COM x [%+.2f %+.2f] y [%+.2f %+.2f] (%u cantos) | peito (%+.2f %+.2f) pelve (%+.2f %+.2f) inclinacao"
                        " | alvo-COM pe0 (%+.2f %+.2f) pe1 (%+.2f %+.2f)\n",
                        lo[0], hi[0], lo[1], hi[1], ps.supportPointCount, chestUp.x, chestUp.y, pelvisUp.x, pelvisUp.y,
                        g0.x, g0.y, g1.x, g1.y);
                }
                {
                    const auto& ps = locomotion.physicalState();
                    const auto& plan = locomotion.contactPlan();
                    {
                        // Pedida / alocada / observada (no plano) e a eficacia
                        // eta = obs.aloc/|aloc|^2 (so diagnostico).
                        const Vec3 req = physical.requestedComAccelerationWorld;
                        const Vec3 alloc = physical.allocatedComAccelerationWorld;
                        const Vec3 obs = physical.observedComAccelerationWorld;
                        const float allocSq = alloc.x * alloc.x + alloc.y * alloc.y;
                        const float eta = allocSq > 0.01f ? (obs.x * alloc.x + obs.y * alloc.y) / allocSq : 0.0f;
                        std::printf("XGT   COM a (m/s2) pedida (%+.2f %+.2f) alocada (%+.2f %+.2f) observada (%+.2f %+.2f) eta %+.2f\n",
                            req.x, req.y, alloc.x, alloc.y, obs.x, obs.y, std::clamp(eta, -5.0f, 5.0f));
                    }
                    // Divergencia medida em apoio simples: captura - centro da sola de apoio.
                    const auto& psx = locomotion.physicalState();
                    const int s0 = psx.feet[0].supporting ? 1 : 0, s1 = psx.feet[1].supporting ? 1 : 0;
                    if (s0 + s1 == 1) {
                        const Vec3 d = psx.capturePointWorld - psx.feet[s0 ? 0 : 1].soleCenterWorld;
                        std::printf("XGD t %.3f pe %d captura-sola (%+.3f %+.3f)\n", t, s0 ? 0 : 1, d.x, d.y);
                    }
                    std::printf("XGT   apoiado %d%d ev %d%d carga %.0f %.0f N | plano carga %.2f %.2f fase %.2f %.2f s\n",
                        ps.feet[0].supporting ? 1 : 0, ps.feet[1].supporting ? 1 : 0,
                        static_cast<int>(ps.feet[0].evidence), static_cast<int>(ps.feet[1].evidence),
                        ps.feet[0].loadNewtons, ps.feet[1].loadNewtons, plan.loadShare[0], plan.loadShare[1],
                        plan.feet[0].phaseSeconds, plan.feet[1].phaseSeconds);
                }
                std::printf("XGT   captura-COM (%+.2f %+.2f) fora %.2f | esforco pedido nos pes (N): pe0 (%+4.0f %+4.0f %4.0f) pe1 (%+4.0f %+4.0f %4.0f)"
                    " | torque pe0 (%+4.0f %+4.0f) pe1 (%+4.0f %+4.0f)\n",
                    cap.x, cap.y, locomotion.physicalState().captureOutsideMeters, w0.x, w0.y, w0.z, w1.x, w1.y, w1.z,
                    physical.contactWrench[0].torqueWorld.x, physical.contactWrench[0].torqueWorld.y,
                    physical.contactWrench[1].torqueWorld.x, physical.contactWrench[1].torqueWorld.y);
            }
            if (std::getenv("XLIFE") && tick % 6 == 0
                && (locomotion.telemetry().lifeFlail > 0.01f || (std::getenv("XLIFEALL") && t < 1.0f)))
                std::printf("XLIFE t %.2f espernear %.2f reflexo %.2f fase %d contatos %zu | pelve z %.2f tilt %.0f\n", t,
                    locomotion.telemetry().lifeFlail,
                    locomotion.telemetry().fallReflex, static_cast<int>(locomotion.telemetry().getUpPhase),
                    state.contacts.size(),
                    after.links.front().position.z, std::acos(std::clamp(uprightNow, -1.0f, 1.0f)) * 57.3f);
            if (std::getenv("XG") && (down || (handoffAt >= 0 && t - handoffAt < 1.5f)) && tick % 12 == 0) {
                const auto& tel = locomotion.telemetry();
                std::printf("XG t %.2f estado %d fase %d prog %.2f ritmo %.2f esforco %.2f ajuda %.0f N rampa %.2f tonus %.2f"
                    " | pelve z %.2f tilt %.0f | pes %d%d | rumo pelve %.0f giro %.2f | facing %.0f alvo %.0f taxa %.2f\n", t,
                    static_cast<int>(tel.state), static_cast<int>(tel.getUpPhase), tel.getUpProgress, tel.getUpRate,
                    tel.getUpStruggle, tel.getUpAssistNewtons, tel.getUpAssistRamp, tel.muscleTone,
                    after.links.front().position.z, std::acos(std::clamp(uprightNow, -1.0f, 1.0f)) * 57.3f,
                    tel.footPlanted[0] ? 1 : 0, tel.footPlanted[1] ? 1 : 0,
                    [&] { const Vec3 f = after.links.front().orientation.rotate({1, 0, 0});
                        return std::atan2(f.y, f.x) * 57.3f; }(),
                    physical.tiltErrorDegrees, tel.facingYawRadians * 57.3f,
                    [&] { const Vec3 f = output.guide.rootOrientationWorld.rotate({1, 0, 0});
                        return std::atan2(f.y, f.x) * 57.3f; }(),
                    tel.turningRateRadiansPerSecond);
            }
            if (bodyState == CharacterLocomotionState3D::GettingUp
                && locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Rising) {
                const float share = locomotion.telemetry().getUpAssistNewtons
                    / (profile.totalMassKg * 9.81f);
                result.getUpAssistPeakShare = std::max(result.getUpAssistPeakShare, share);
                const auto& tel = locomotion.telemetry();
                // "Segurada": a inclinacao alem do clipe parada ou crescendo -
                // a prancha mantida pela ajuda. Endireitando depressa (mais de
                // 20 graus/s) e o corpo saindo dela, nao ficando.
                const float excessRate = (heldPreviousExcess - tel.getUpTiltExcessDegrees) / dt;
                heldPreviousExcess = tel.getUpTiltExcessDegrees;
                const bool held = excessRate < 20.0f;
                if (after.links.front().position.z > 0.6f && tel.getUpTiltExcessDegrees > 15.0f && held)
                    result.getUpHeldLeanSeconds += dt;
                if (after.links.front().position.z > 0.6f && tel.getUpUprightTorque > 20.0f && held) {
                    result.getUpHeldLean = std::max(result.getUpHeldLean, tel.getUpTiltExcessDegrees);
                    const Vec3 com = scene->ragdollDynamics(handle).centerOfMass;
                    std::vector<std::array<float, 2>> supports;
                    for (const auto& contact : after.interactions)
                        if (contact.normalWorld.z > 0.5f)
                            supports.push_back({contact.positionWorld.x, contact.positionWorld.y});
                    if (!supports.empty())
                        result.getUpComOutside = std::max(result.getUpComOutside,
                            outsideSupport(supports, com.x, com.y));
                }
                assistSum += share;
                ++assistCount;
                double strength = 0;
                for (const float value : input.jointStrength) strength += value;
                strengthSum += input.jointStrength.empty() ? 1.0
                    : strength / input.jointStrength.size();
            }
            if (down && locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Rising) {
                // So o levantar (o teleporte seria dele); caindo pelo ar o corpo
                // pode estar rapido.
                const float speed = after.links.front().linearVelocity.length();
                result.getUpWorstRootSpeed = std::max(result.getUpWorstRootSpeed, speed);
                result.getUpWorstRootStep = std::max(result.getUpWorstRootStep,
                    (after.links.front().position - state.links.front().position).length());
                for (std::size_t i = 0; i < after.links.size(); ++i) {
                    const float v = after.links[i].linearVelocity.length();
                    if (std::getenv("XWHIP") && v > 9.0f)
                        std::printf("WHIP t %.3f %s %.1f m/s fase %.2f tonus %.2f\n", t, profile.links[i].id.c_str(), v,
                            locomotion.telemetry().getUpProgress, locomotion.telemetry().muscleTone);
                    result.getUpWorstLinkSpeed = std::max(result.getUpWorstLinkSpeed, v);
                }
            }
            result.standingAtEnd = !down && uprightNow > 0.85f;
            {
                const auto& tel = locomotion.telemetry();
                if (down && tel.getUpPhase == CharacterGetUpPhase3D::Settling && tel.muscleTone < 0.35f) {
                    result.restSeconds += dt;
                    result.restHandHeight += 0.5f * (after.links[hands[0]].position.z
                        + after.links[hands[1]].position.z) * dt;
                    result.restJointError += tel.jointErrorRmsDegrees * dt;
                    for (std::size_t i = 0; i < profile.links.size(); ++i)
                        if (profile.links[i].id == "Head") result.restHeadHeight += after.links[i].position.z * dt;
                }
                if (!down && tel.fallCommit > 0.5f) {
                    result.fallingSeconds += dt;
                    result.fallingJointError += tel.jointErrorRmsDegrees * dt;
                }
            }
        }
        const auto& root = after.links.front();
        const float tilt = std::acos(std::clamp(root.orientation.rotate({0, 0, 1}).z, -1.0f, 1.0f)) * 57.2958f;
        if (t > 0.5f) {
            result.worstTilt = std::max(result.worstTilt, tilt);
            Vec3 gap = root.position - output.guide.rootPositionWorld;
            gap.z = 0;
            result.worstBodyCapsule = std::max(result.worstBodyCapsule, gap.length());
            trackingSum += gap.length();
            ++trackingCount;
            for (std::size_t i = 0; i < after.links.size(); ++i) {
                const float v = after.links[i].linearVelocity.length();
                if (std::getenv("XV") && v > 25.0f)
                    std::printf("XV t %.3f link %zu v %.1f estado %d\n", t, i, v,
                        static_cast<int>(locomotion.telemetry().state));
                result.worstLinkSpeed = std::max(result.worstLinkSpeed, v);
            }
            const auto& telemetry = locomotion.telemetry();
            if (!down) {
                const bool pressing = physical.perturbation > 0.5f;
                if (pressing && result.firstPressAt < 0) result.firstPressAt = t;
                if (pressing) {
                    result.pressSeconds += dt;
                    if (scenario.pushDirection.lengthSquared() > 0.5f) {
                        const Vec3 up = after.links[chest].orientation.rotate({0, 0, 1});
                        result.leanIntoSum += std::asin(std::clamp(-dot(up, scenario.pushDirection),
                            -1.0f, 1.0f)) * 57.2958f * dt;
                    }
                }
                if (physical.captureErrorMeters > 0.06f && telemetry.footPlanted[0] && telemetry.footPlanted[1])
                    result.leanWithoutStepSeconds += dt;
                {
                    const float tiltNow = physical.bodyTiltRadians * 57.2958f;
                    const float rate = result.previousTiltForHold < 0 ? 0.0f
                        : std::abs(tiltNow - result.previousTiltForHold) / dt;
                    result.previousTiltForHold = tiltNow;
                    if (wanted.move.lengthSquared() < 1e-6f && physical.balanceMargin < -0.03f
                        && tiltNow > 10.0f && rate < 20.0f && capsule.grounded)
                        result.heldOutsideSeconds += dt;
                }
                for (std::size_t side = 0; side < 2; ++side) {
                    if (!telemetry.footPlanted[side]) continue;
                    bool touching = false;
                    for (const auto& contact : after.interactions)
                        if (contact.linkIndex == feet[side] && contact.normalWorld.z > 0.5f) touching = true;
                    const Vec3 v = after.links[feet[side]].linearVelocity;
                    if (touching && std::hypot(v.x, v.y) > 0.25f) result.plantedSlideSeconds += dt * 0.5f;
                }
                if (result.dragged) {
                    for (std::size_t side = 0; side < 2; ++side) {
                        bool touching = false;
                        for (const auto& contact : after.interactions)
                            if (contact.linkIndex == feet[side] && contact.normalWorld.z > 0.5f) touching = true;
                        const Vec3 v = after.links[feet[side]].linearVelocity;
                        if (touching && std::hypot(v.x, v.y) > 0.25f) result.dragSlideSeconds += dt * 0.5f;
                        if (wasPlantedDrag[side] && !telemetry.footPlanted[side]) ++result.dragSteps;
                    }
                    result.dragDistance += std::hypot(after.links.front().linearVelocity.x,
                        after.links.front().linearVelocity.y) * dt;
                }
                for (std::size_t side = 0; side < 2; ++side) wasPlantedDrag[side] = telemetry.footPlanted[side];
                for (std::size_t side = 0; side < 2; ++side) {
                    if (wasPlantedPress[side] && !telemetry.footPlanted[side] && result.firstPressAt >= 0) {
                        ++result.pressSteps;
                        if (result.firstStepAfterPress < 0) result.firstStepAfterPress = t - result.firstPressAt;
                    }
                    wasPlantedPress[side] = telemetry.footPlanted[side];
                }
            }
            if (!down && capsule.grounded && !telemetry.footPlanted[0] && !telemetry.footPlanted[1]
                && wanted.move.lengthSquared() < 1e-6f) ++result.bothFeetAirTicks;
            for (std::size_t side = 0; side < 2; ++side) {
                if (wasPlanted[side] && !telemetry.footPlanted[side]
                    && wanted.move.lengthSquared() < 1e-6f) ++result.steps;
                wasPlanted[side] = telemetry.footPlanted[side];
            }
        }
        {
            // Contrato do planejador de passos, a cada tick e em todo cenario.
            const CharacterContactPlan3D& plan = locomotion.contactPlan();
            const CharacterPhysicalState3D& measured = locomotion.physicalState();
            const auto& telemetry = locomotion.telemetry();
            for (std::size_t side = 0; side < 2; ++side) {
                const FootPlan3D& foot = plan.feet[side];
                const bool owned = !foot.followingGait;
                const bool inAir = foot.phase == FootPhase3D::Swing
                    || foot.phase == FootPhase3D::TouchdownSearch;
                const bool wasInAir = previousPhase[side] == FootPhase3D::Swing
                    || previousPhase[side] == FootPhase3D::TouchdownSearch;
                if (owned) {
                    const bool planted = foot.phase == FootPhase3D::Stance
                        || foot.phase == FootPhase3D::Unloading || foot.phase == FootPhase3D::Loading;
                    if (telemetry.footPlanted[side] != planted) ++result.ownerMismatch;
                }
                if (owned && previousOwned[side] && wasInAir && foot.phase == FootPhase3D::Stance)
                    ++result.skippedLoading;
                // Abriu um passo: o pe foi para o ar pelo planejador.
                if (owned && inAir && (!wasInAir || !previousOwned[side] || openStep[side] < 0)) {
                    PlannerStepTrace trace;
                    trace.stepId = foot.stepId;
                    trace.side = static_cast<int>(side);
                    trace.reason = trace.finalReason = foot.reason;
                    trace.releasedAt = t;
                    trace.releasedUnloaded = foot.releasedUnloaded;
                    trace.liftoff = foot.liftoffWorld;
                    result.plannerSteps.push_back(trace);
                    openStep[side] = static_cast<int>(result.plannerSteps.size()) - 1;
                    previousTarget[side] = foot.targetWorld;
                    previousTargetVelocityValid[side] = false;
                }
                if (openStep[side] >= 0) {
                    PlannerStepTrace& trace = result.plannerSteps[static_cast<std::size_t>(openStep[side])];
                    if (owned && inAir) {
                        trace.finalReason = foot.reason;
                        trace.maxSoleLift = std::max(trace.maxSoleLift,
                            measured.feet[side].soleLowestHeight - foot.liftoffGroundHeight);
                        // Continuidade: quanto a velocidade do alvo muda de um tick
                        // para o outro (m/s) - um passo longo e rapido nao e salto.
                        const Vec3 velocity = (foot.targetWorld - previousTarget[side]) * (1.0f / dt);
                        if (trace.searchSeconds >= 0.0f && previousTargetVelocityValid[side])
                            trace.worstTargetJump = std::max(trace.worstTargetJump,
                                (velocity - previousTargetVelocity[side]).length());
                        previousTargetVelocity[side] = velocity;
                        previousTargetVelocityValid[side] = true;
                        previousTarget[side] = foot.targetWorld;
                        if (foot.phase == FootPhase3D::TouchdownSearch) trace.searchSeconds += dt;
                    } else {
                        // Fechou: pousou (Loading) ou o planejador largou o pe.
                        if (owned && foot.phase == FootPhase3D::Loading) {
                            const FootSupportEstimate3D& f = measured.feet[side];
                            const bool contact = f.supporting && !f.heldFromMemory
                                && (f.evidence == ContactEvidence3D::EstimatedImpulse
                                    || f.evidence == ContactEvidence3D::SolvedImpulse);
                            if (!contact) ++result.touchdownWithoutContact;
                            trace.landedAt = t;
                            trace.evidence = f.evidence;
                            trace.early = foot.touchdownEarly;
                            trace.dragged = foot.dragged;
                            trace.touchdown = foot.anchorWorld;
                            trace.travel = std::hypot(foot.anchorWorld.x - trace.liftoff.x,
                                foot.anchorWorld.y - trace.liftoff.y);
                        }
                        openStep[side] = -1;
                    }
                }
                previousPhase[side] = foot.phase;
                previousOwned[side] = owned;
            }
        }
        result.plan = locomotion.contactPlan();
        result.bodyReferenceTicks = locomotion.telemetry().bodyReferenceTicks;
        result.finalTilt = tilt;
        result.finalChestTilt = std::acos(std::clamp(after.links[chest].orientation.rotate({0, 0, 1}).z,
            -1.0f, 1.0f)) * 57.2958f;
        result.finalBody = root.position;
        result.finalCapsule = capsule.position;
    }
    // Contrato do planejador unico de passos (E3), em todo cenario: pe so
    // pousa por contato medido, volta a apoiar passando pela carga, e a
    // marca de apoio da locomocao e a fase do plano nunca divergem.
    if (!scenario.legacy) {
        if (result.touchdownWithoutContact || result.skippedLoading || result.ownerMismatch)
            std::cerr << "Contrato dos pes em [" << scenario.name << "]: pouso sem contato "
                << result.touchdownWithoutContact << ", apoio sem carga " << result.skippedLoading
                << ", dono divergente " << result.ownerMismatch << "\n";
        require(result.touchdownWithoutContact == 0 && result.skippedLoading == 0
                && result.ownerMismatch == 0,
            "Planejador de passos: pouso sem contato, apoio sem carga ou dois donos do pe");
    }
    result.meanTracking = trackingCount ? static_cast<float>(trackingSum / trackingCount) : 0;
    result.getUpAssistMeanShare = assistCount ? static_cast<float>(assistSum / assistCount) : 0;
    result.getUpJointStrength = assistCount ? static_cast<float>(strengthSum / assistCount) : 1;
    if (std::getenv("XMOTORSUM")) {
        const char* names[6] = { "quadril E", "joelho E", "tornozelo E", "quadril D", "joelho D", "tornozelo D" };
        for (std::size_t j = 0; j < 6; ++j)
            for (std::size_t role = 0; role < 2; ++role) {
                const auto& m = result.motorSummary[j][role];
                if (m.ticks == 0) continue;
                std::printf("MOTOR %-11s %-6s ticks %4d | saturado (>95%%) %5.1f%% | pior %.2f | |motor| medio %5.0f N.m | |ff| medio %5.0f N.m\n",
                    names[j], role == 0 ? "apoio" : "no ar", m.ticks, 100.0 * m.saturatedTicks / m.ticks, m.maxSaturation,
                    m.sumAbsMotor / m.ticks, m.sumAbsFeedforward / m.ticks);
            }
    }
    if (stepTraceSide >= 0 && !stepTrace.empty()) {
        // Janela: 0,2 s antes da descarga do primeiro passo de marcha (razao
        // Locomotion) deste pe ate 0,2 s depois do pouso (ou XFROM/XTO).
        float from = -1, to = -1;
        std::uint32_t id = 0;
        for (const auto& sample : stepTrace) {
            if (id == 0 && sample.reason == static_cast<int>(StepReason3D::Locomotion) && sample.phase != 0) {
                id = sample.stepId; from = sample.t - 0.2f;
            }
            if (id != 0 && sample.stepId == id && (sample.phase == 4 || sample.phase == 0) && to < 0 && sample.t > from + 0.25f)
                to = sample.t + 0.2f;
        }
        if (const char* v = std::getenv("XFROM")) from = std::strtof(v, nullptr);
        if (const char* v = std::getenv("XTO")) to = std::strtof(v, nullptr);
        if (to < 0) to = from + 1.0f;
        std::printf("PASSO pe %d, janela %.3f-%.3f s. Fases: 0 apoio, 1 descarga, 2 balanco, 3 procurando, 4 carregando.\n"
            "  folga da sola ao chao (cm): alvo / IK antes do limitador / comando / comando pela pelve fisica / pe fisico\n"
            "  erros (cm): ik = IK-alvo, lim = comando-IK, frame = pelve fisica-IK, rastreio = pe fisico-comando pela pelve fisica\n", stepTraceSide, from, to);
        for (const auto& sample : stepTrace) {
            if (sample.t < from || sample.t > to || !sample.trace.valid) continue;
            const FootTrace3D& tr = sample.trace;
            // Decomposicao do erro de frame: translacao da pelve (plano e
            // altura) e rotacao (a mesma perna pendurada na pelve fisica).
            const Vec3 legLocal = tr.ikRootOrientation.conjugate().rotate(tr.command - tr.ikRoot);
            const Vec3 rotationPart = tr.physicalRootOrientation.rotate(legLocal) - (tr.command - tr.ikRoot);
            const Vec3 rootShift = tr.physicalRoot - tr.ikRoot;
            const auto tiltOf = [](Quaternion q) {
                const Vec3 up = q.rotate({0, 0, 1});
                return std::acos(std::clamp(up.z, -1.0f, 1.0f)) * 57.2958f;
            };
            if (std::getenv("XSTEPZ")) {
                const LegReachTelemetry3D& rc = sample.reach;
                std::printf("ALCANCE %.3f f%d outro f%d contato %d | pelve ik %.3f fisica %.3f | alcance fisico %.3f alvo %.3f | estica fisico %+.2f alvo %+.2f m/s | joelho %+.2f/%+.2f v %+.1f\n",
                    sample.t, sample.phase, sample.otherPhase, sample.supportingStrict,
                    rc.valid ? rc.ikRootHeight : tr.ikRoot.z, rc.valid ? rc.physicalRootHeight : tr.physicalRoot.z,
                    rc.physicalRatio, rc.referenceRatio, rc.physicalAxialSpeed, rc.referenceAxialSpeed,
                    sample.kneeTarget, sample.kneeMeasured, sample.kneeVelocityTarget);
            }
            std::printf("PASSO %.3f f%d r%d #%u carga %.2f ap%d ev%d m%d | folga %5.1f %5.1f %5.1f %5.1f %5.1f | erro ik %4.1f lim %4.1f frame %4.1f rastreio %4.1f | quadril %+.2f/%+.2f joelho %+.2f/%+.2f v %+5.1f k%.2f T%.2f | pelve dxy %4.1f dz %+5.1f rot %4.1f (rotz %+5.1f) incl ik %4.1f fis %4.1f\n",
                sample.t, sample.phase, sample.reason, sample.stepId, sample.load, sample.supporting, sample.evidence, sample.memory,
                tr.targetClearance * 100, tr.ikClearance * 100, tr.commandClearance * 100,
                tr.commandFromPhysicalPelvisClearance * 100, tr.physicalClearance * 100,
                (tr.ik - tr.target).length() * 100, (tr.command - tr.ik).length() * 100,
                (tr.commandFromPhysicalPelvis - tr.command).length() * 100,
                (tr.physical - tr.commandFromPhysicalPelvis).length() * 100,
                sample.hipTarget, sample.hipMeasured, sample.kneeTarget, sample.kneeMeasured, sample.kneeVelocityTarget,
                sample.stiffness, sample.torque,
                std::hypot(rootShift.x, rootShift.y) * 100, rootShift.z * 100,
                rotationPart.length() * 100, rotationPart.z * 100,
                tiltOf(tr.ikRootOrientation), tiltOf(tr.physicalRootOrientation));
        }
    }
    return result;
}

void report(const Scenario& scenario, const ScenarioResult& r) {
    std::cout << "Cenario [" << scenario.name << "]: quedas " << r.falls;
    if (r.firstFallSeconds >= 0) std::cout << " (em " << r.firstFallSeconds << " s)";
    std::cout << " | tilt max " << r.worstTilt << " graus, final " << r.finalTilt
        << " | corpo x referencia media " << r.meanTracking * 1000 << " mm, pior "
        << r.worstBodyCapsule * 1000 << " mm | peito final " << r.finalChestTilt
        << " graus | passos parado " << r.steps
        << " | membro " << r.worstLinkSpeed << " m/s | corpo final ("
        << r.finalBody.x << ", " << r.finalBody.y << ")";
    if (scenario.npc) std::cout << " | outro boneco: quedas " << r.npcFalls << ", final ("
        << r.npcFinal.x << ", " << r.npcFinal.y << ")";
    if (r.npcGetUps > 0 || scenario.npcSpawnPelvisHeight > 0)
        std::cout << "\n    outro boneco levantando: " << r.npcGetUps << " vezes, ancora ate "
            << r.npcAnchorErrorPeak << " m do corpo, referencia ate " << r.npcGuideSpeedPeak
            << " m/s, membro ate " << r.npcAfterGetUpLinkPeak << " m/s no 1 s seguinte, de pe no fim "
            << r.npcStandingAtEnd;
    if (r.firstFallSeconds >= 0)
        std::cout << "\n    queda: primeiro no chao " << (r.firstGroundLink.empty() ? "-" : r.firstGroundLink)
            << ", cabeca a " << r.headImpactSpeed << " m/s | levantar em " << r.getUpSeconds
            << " s, pelve ate " << r.getUpWorstRootSpeed << " m/s, salto por tick "
            << r.getUpWorstRootStep * 1000 << " mm, membro " << r.getUpWorstLinkSpeed
            << " m/s | ajuda media " << r.getUpAssistMeanShare * 100 << "% do peso, pico "
            << r.getUpAssistPeakShare * 100 << "% | recaidas " << r.relapses
            << " | em pe no fim " << r.standingAtEnd
            << "\n    no chao a pelve girou " << r.settleTumbleDegrees << " graus largada, "
            << r.settleGatherDegrees << " se arrumando | troca para o normal: pe a "
            << r.handoffFootSpeed << " m/s, base abriu " << r.handoffSpreadGrowth * 1000 << " mm"
            << " | forca das juntas no levantar " << r.getUpJointStrength * 100 << "%"
            << "\n    levantando (alto): inclinacao segurada pela ajuda ate " << r.getUpHeldLean
            << " graus (" << r.getUpHeldLeanSeconds << " s alem de 15), centro de massa ate "
            << r.getUpComOutside * 1000 << " mm fora dos apoios"
            << "\n    na troca: pelve a " << r.handoffTilt << " graus da vertical, peito a " << r.handoffChestTilt
            << " graus da pose | pior desvio da pose (pelve/peito) nos 2 s seguintes " << r.afterGetUpWorstTilt << " graus";
    std::cout << " | vertical: " << r.worstUpwardSpeed << " m/s, "
        << r.worstVerticalStep * 1000 << " mm/tick";
    if (std::string(scenario.name).find("pulo") != std::string::npos)
        std::cout << " | pelve ate " << r.highestPelvis << " m, capsula (base) ate " << r.highestCapsuleFeet << " m";
    if (r.restSeconds > 0)
        std::cout << " | largado: maos a " << r.restHandHeight / r.restSeconds * 1000
            << " mm do chao, cabeca a " << r.restHeadHeight / r.restSeconds * 1000
            << " mm, juntas " << r.restJointError / r.restSeconds << " graus da pose";
    if (r.fallingSeconds > 0)
        std::cout << " | caindo: juntas " << r.fallingJointError / r.fallingSeconds << " graus da pose";
    if (scenario.dragLink != nullptr)
        std::cout << "\n    arrastado: " << r.dragSteps << " passos, pe escorregando no chao "
            << r.dragSlideSeconds << " s, corpo andou " << r.dragDistance << " m";
    if (r.heldOutsideSeconds > 0.05f)
        std::cout << " | segurado fora da base " << r.heldOutsideSeconds << " s";
    if (r.pressSeconds > 0) {
        std::cout << "\n    pressao: " << r.pressSeconds << " s em contato, " << r.pressSteps
            << " passos, primeiro passo " << r.firstStepAfterPress << " s depois do contato, inclinado sem passo "
            << r.leanWithoutStepSeconds << " s, pe travado escorregando " << r.plantedSlideSeconds << " s";
        if (r.pressSeconds > 0 && std::abs(r.leanIntoSum) > 0)
            std::cout << ", peito contra a forca " << r.leanIntoSum / r.pressSeconds << " graus";
    }
    if (r.rigidSeconds > 0)
        std::cout << " | queda sem volta ainda de pe " << r.rigidSeconds << " s";
    if (r.armsReflexSeconds > 0)
        std::cout << " | bracos protegendo de pe " << r.armsReflexSeconds << " s";
    if (std::getenv("XFACING"))
        std::cout << " | pelve fora da camera ate " << r.facingOffLookPeak << " graus";
    if (std::getenv("XDRAG"))
        std::cout << " | pe arrastando: esq " << r.dragSeconds[0] << " s, dir " << r.dragSeconds[1]
            << " s; balanco ate " << r.swingPeak[0] << " / " << r.swingPeak[1] << " m";
    if (r.balanceReactionPeak > 0.05f)
        std::cout << " | desequilibrio: pico " << r.balanceReactionPeak << ", "
            << r.balanceReactionTicks / 120.0f << " s acima de 0,3";
    if (std::getenv("XTILTERR") && r.tiltErrorCount > 0)
        std::cout << " | erro de inclinacao: media " << r.tiltErrorSum / r.tiltErrorCount
            << ", pior " << r.tiltErrorWorst << ", acima de 15: " << r.tiltErrorOver15 * 100 / r.tiltErrorCount << "%";
    if (r.reverseSeconds >= 0 || r.reverseWorstTilt > 0)
        std::cout << " | inversao: " << r.reverseSeconds << " s, inclinacao ate " << r.reverseWorstTilt << " graus";
    if (r.landed) std::cout << " | pouso: pelve ate " << r.landingLowestPelvis
        << " m, ticks com os dois pes no ar " << r.landingBounceTicks
        << ", pelve voltando a subir a " << r.landingRebound << " m/s";
    if (std::getenv("XSTEPS")) {
        for (const auto& step : r.plannerSteps)
            std::printf("\n    passo %u pe %d razao %d->%d solto %.2f s%s, pousou %.2f s (evid %d%s%s), sola ate %.3f m, andou %.3f m, mudanca de velocidade do alvo ate %.2f m/s, busca %.2f s",
                step.stepId, step.side, static_cast<int>(step.reason), static_cast<int>(step.finalReason),
                step.releasedAt, step.releasedUnloaded ? "" : " com carga", step.landedAt,
                static_cast<int>(step.evidence), step.early ? ", cedo" : "", step.dragged ? ", arrastado" : "",
                step.maxSoleLift, step.travel, step.worstTargetJump, step.searchSeconds);
    }
    if (r.touchdownWithoutContact || r.skippedLoading || r.ownerMismatch)
        std::cout << "\n    CONTRATO DOS PES: pouso sem contato " << r.touchdownWithoutContact
            << ", apoio sem carga " << r.skippedLoading << ", dono divergente " << r.ownerMismatch;
    if (r.plan.stepsStarted > 0)
        std::cout << "\n    planejador: " << r.plan.stepsStarted << " passos, " << r.plan.touchdownsByContact
            << " pousos por contato (" << r.plan.earlyTouchdowns << " cedo), " << r.plan.touchdownSearches
            << " procurando o chao (ate " << r.plan.longestTouchdownSearchSeconds << " s), "
            << r.plan.forcedReleases << " soltos com carga, " << r.plan.draggedSteps << " arrastados, " << r.plan.abortedSteps << " desistidos, "
            << r.plan.blockedSwings << " esbarrados, " << r.plan.reasonChanges << " trocas de razao";
    std::cout << std::endl;
}

// Movimentos bruscos do jogador (zigue-zague, cortes, para-arranca, camera
// chicoteando) e pulos correndo: nada disso pode derrubar o personagem
// sozinho, no plano e sem obstaculo.
// Encostado num obstaculo (debrucado sobre uma caixa alta, maos em cima): ele
// tem de empurrar e voltar a ficar de pe, nao ficar parado ali.
void leanScenarios() {
    const Scenario scenarios[] = {
        {"trote contra caixa alta (0,9 m)", 7, [](PhysicsScene3D& s) {
            dynamicBox(s, {2.6f, 0, 0.45f}, {0.35f, 0.5f, 0.45f}, 300, {});
        }, [](float t) { ScenarioCommand c; if (t > 0.6f && t < 2.2f) c.move = {1, 0, 0}; return c; }, {}},
        // Pulando e batendo numa parede alta em pleno voo: ele cai (a queixa:
        // "ta dificil ele cair quando pula e esbarra em algo").
        {"pulo trotando contra parede alta (1,8 m)", 12, [](PhysicsScene3D& s) {
            staticBox(s, {3.8f, 0, 0.9f}, {0.15f, 1.5f, 0.9f});
        }, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 4.0f) c.move = {1, 0, 0};
            c.jump = t >= 1.55f && t < 1.56f;
            return c;
        }, {}},
        // Em sprint (7,5 m/s) contra uma caixa na cintura, tombar por cima dela
        // e fisico: aqui so nao pode ficar preso debrucado nela.
        {"sprint contra caixa alta (0,9 m)", 13, [](PhysicsScene3D& s) {
            dynamicBox(s, {4.0f, 0, 0.45f}, {0.35f, 0.5f, 0.45f}, 300, {});
        }, [](float t) { ScenarioCommand c; if (t > 0.6f && t < 2.2f) c.move = {1, 0, 0}; c.sprint = true; return c; }, {}},
    };
    for (const auto& scenario : scenarios) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(scenario.name).find(only) == std::string::npos) continue;
        const auto r = runScenario(scenario);
        report(scenario, r);
        if (std::string(scenario.name).find("parede alta") != std::string::npos) {
            require(r.falls >= 1, "Pulou e bateu numa parede alta e nao caiu (equilibrado demais no ar)");
            continue;
        }
        if (std::string(scenario.name).find("sprint") != std::string::npos) {
            require(r.standingAtEnd && r.finalTilt < 15.0f,
                "Debrucado num obstaculo, ele nao voltou a ficar de pe");
            continue;
        }
        require(r.falls == 0 && r.finalTilt < 15.0f,
            "Debrucado num obstaculo, ele nao voltou a ficar de pe");
    }
}

void agilityScenarios() {
    const auto zigzag = [](float period, bool sprint) {
        return [=](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) {
                const int leg = static_cast<int>((t - 0.6f) / period);
                c.move = {1, leg % 2 ? 0.8f : -0.8f, 0};
            }
            c.sprint = sprint;
            return c;
        };
    };
    const Scenario scenarios[] = {
        {"brusco zigue-zague sprint 0,4 s", 6.5f, {}, zigzag(0.4f, true), {}},
        {"brusco zigue-zague trote 0,3 s", 6.5f, {}, zigzag(0.3f, false), {}},
        {"brusco corte 90 no sprint", 6, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = t < 2.2f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            c.lookYaw = t < 2.2f ? 0.0f : 1.5708f;
            c.sprint = true;
            return c;
        }, {}},
        {"brusco para e arranca", 7, {}, [](float t) {
            ScenarioCommand c;
            const float phase = std::fmod(t, 1.2f);
            if (t > 0.6f && t < 6.0f && phase < 0.8f) c.move = {1, 0, 0};
            c.sprint = true;
            return c;
        }, {}},
        {"brusco camera chicote no sprint", 6.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) c.move = {1, 0, 0};
            c.lookYaw = t > 1.5f ? 1.2f * std::sin((t - 1.5f) * 6.0f) : 0.0f;
            c.sprint = true;
            return c;
        }, {}},
        {"brusco frente-tras trote", 7, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 6.0f) c.move = {static_cast<int>((t - 0.6f) / 0.7f) % 2 ? -1.0f : 1.0f, 0, 0};
            return c;
        }, {}},
        {"trote e troca para o lado", 5, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 4.5f) c.move = t < 2.0f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            return c;
        }, {}},
        {"sprint e troca para o lado", 5, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 4.5f) c.move = t < 2.0f ? Vec3{1, 0, 0} : Vec3{0, -1, 0};
            c.sprint = true;
            return c;
        }, {}},
        {"brusco strafe alternado", 6.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) c.move = {0, static_cast<int>((t - 0.6f) / 0.5f) % 2 ? -1.0f : 1.0f, 0};
            return c;
        }, {}},
        {"pulo parado tocado", 4, {}, [](float t) {
            ScenarioCommand c;
            c.jump = t >= 1.0f && t < 1.01f;
            return c;
        }, {}},
        {"pulo parado segurado", 4, {}, [](float t) {
            ScenarioCommand c;
            c.jumpCharging = t >= 1.0f && t < 1.6f;
            c.jump = t >= 1.6f && t < 1.61f;
            c.jumpChargeSeconds = 0.6f;
            return c;
        }, {}},
        {"pulo trotando segurado", 6, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = {1, 0, 0};
            c.jumpCharging = t >= 1.6f && t < 2.2f;
            c.jump = t >= 2.2f && t < 2.21f;
            c.jumpChargeSeconds = 0.6f;
            return c;
        }, {}},
        {"pulo correndo (sprint)", 6, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = {1, 0, 0};
            c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f;
            return c;
        }, {}},
        {"pulo correndo e para no ar", 6, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 2.3f) c.move = {1, 0, 0};
            c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f;
            return c;
        }, {}},
        {"pulo correndo e vira no ar", 6, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = t < 2.2f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            c.lookYaw = t < 2.2f ? 0.0f : 1.5708f;
            c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f;
            return c;
        }, {}},
        {"pulos seguidos trotando", 7, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 6.0f) c.move = {1, 0, 0};
            c.jump = (t >= 1.8f && t < 1.81f) || (t >= 3.3f && t < 3.31f) || (t >= 4.8f && t < 4.81f);
            return c;
        }, {}},
    };
    int fallen = 0;
    for (const auto& scenario : scenarios) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(scenario.name).find(only) == std::string::npos) continue;
        const auto r = runScenario(scenario);
        report(scenario, r);
        if (r.falls > 0) ++fallen;
        // Da frente para o lado (so a tecla do lado): a pelve vai direto para
        // o strafe, sem girar para a diagonal no caminho (era 49-69 graus).
        if (std::string(scenario.name).find("troca para o lado") != std::string::npos)
            require(r.facingOffLookPeak < 10.0f, "Trocando para o lado, a pelve girou para a diagonal");
    }
    if (!std::getenv("XAGILITYSOFT"))
        require(fallen == 0, "Movimento brusco ou pulo correndo derrubou o personagem sozinho");
}

void polishScenarios() {
    const auto laboratoryStairs = [](PhysicsScene3D& s) {
        for (int i = 0; i < 8; ++i) {
            const float h = (i + 1) * 0.18f;
            staticBox(s, {2.0f + (i + 0.5f) * 0.28f, 0, h * 0.5f}, {0.14f, 2, h * 0.5f});
        }
        staticBox(s, {14.24f, 0, 0.72f}, {10, 2, 0.72f});
    };
    const auto forward = [](float t) {
        ScenarioCommand c;
        if (t > 0.6f && t < 6.0f) c.move = {1, 0, 0};
        return c;
    };
    const Scenario scenarios[] = {
        {"polish trote", 5, {}, forward, {}},
        {"polish inversao sprint", 6, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 4.5f) c.move = {t < 2.0f ? 1.0f : -1.0f, 0, 0};
            c.sprint = true;
            return c;
        }, {}},
        {"polish camera 180 sprint", 6, {}, [](float t) {
            ScenarioCommand c;
            c.lookYaw = t < 2 ? 0.0f : 3.14159265f;
            if (t > 0.6f && t < 4.5f) c.move = {t < 2 ? 1.0f : -1.0f, 0, 0};
            c.sprint = true;
            return c;
        }, {}},
        {"polish escada 16 cm", 8, [](PhysicsScene3D& s) {
            for (int i = 0; i < 10; ++i) {
                const float h = (i + 1) * 0.16f;
                staticBox(s, {2.0f + i * 0.32f, 0, h * 0.5f}, {0.16f, 2, h * 0.5f});
            }
            staticBox(s, {15.04f, 0, 0.8f}, {10.0f, 2, 0.8f});
        }, forward, {}},
        {"polish rampa 20 graus", 8, [](PhysicsScene3D& s) {
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            body.position = {4.0f, 0, 0.65f};
            body.orientation = Quaternion::fromAxisAngle({0, 1, 0}, -0.349066f);
            body.materialId = "concrete";
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = {2.2f, 2, 0.1f};
            shape.materialId = "concrete";
            const auto ramp = s.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
            require(static_cast<bool>(ramp), "Missing ramp");
            staticBox(s, {16.0f, 0, 0.742f}, {10.0f, 2, 0.742f});
        }, forward, {}},
        {"polish escada laboratorio 18 cm", 9, laboratoryStairs, forward, {}},
        {"polish escada laboratorio sprint", 9, laboratoryStairs, [forward](float t) {
            auto c = forward(t);
            c.sprint = true;
            return c;
        }, {}},
    };
    bool passed = true;
    for (const auto& scenario : scenarios) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(scenario.name).find(only) == std::string::npos) continue;
        const auto r = runScenario(scenario);
        report(scenario, r);
        passed = passed && r.falls == 0 && r.worstLinkSpeed < 30;
        const std::string name = scenario.name;
        if (name.find("trote") != std::string::npos)
            passed = passed && r.worstUpwardSpeed < 0.65f;
        if (name.find("inversao") != std::string::npos || name.find("camera") != std::string::npos)
            passed = passed && r.worstUpwardSpeed < 2.1f
                // Do sprint para o sentido oposto: rapido (era 1,3 s), sem cair.
                && r.reverseSeconds >= 0 && r.reverseSeconds < 0.8f;
        if (name.find("escada") != std::string::npos || name.find("rampa") != std::string::npos)
            passed = passed && r.finalBody.x > 5.5f && r.finalBody.z > 1.9f;
    }
    require(passed, "Polimento: queda, instabilidade ou terreno nao atravessado");
    // Pouso ativo, em 8 instantes de pulo (um pulo so e caotico: o rebote de
    // um caso ia de 0,75 a 1,35 m/s com mudancas minimas antes da corrida):
    // todos pousam agachando (pelve abaixo de ~0,88 m), a mediana do rebote
    // fica abaixo de 1,4 m/s e nenhum passa de 1,8 (antes da primeira versao
    // do pouso: 2,2 m/s com os dois pes a 30 cm). E guarda de regressao, nao
    // aceitacao: o rebote de ~1,1-1,3 m/s e limitacao conhecida (pouso por
    // contatos, etapa E7). Medido na E2: mediana 1,15 no Release e 1,32 no
    // Debug (0,37 a 1,48 nos 8 instantes) - so a ordem das contas muda.
    std::vector<float> rebounds;
    bool landingsOk = true;
    for (const float jumpAt : { 1.40f, 1.45f, 1.50f, 1.55f, 1.60f, 1.65f, 1.70f, 1.75f }) {
        char name[64];
        std::snprintf(name, sizeof name, "polish aterrissagem (pulo em %.2f s)", jumpAt);
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(name).find(only) == std::string::npos) continue;
        const Scenario scenario {name, 6, {}, [jumpAt](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 3.5f) c.move = {1, 0, 0};
            c.jump = t >= jumpAt && t < jumpAt + 0.01f;
            return c;
        }, {}};
        const auto r = runScenario(scenario);
        report(scenario, r);
        landingsOk = landingsOk && r.falls == 0 && r.landed && r.landingLowestPelvis < 0.88f
            && r.landingRebound < 1.8f;
        rebounds.push_back(r.landingRebound);
    }
    if (!rebounds.empty()) {
        std::sort(rebounds.begin(), rebounds.end());
        const float median = rebounds.size() % 2 ? rebounds[rebounds.size() / 2]
            : 0.5f * (rebounds[rebounds.size() / 2 - 1] + rebounds[rebounds.size() / 2]);
        std::cout << "Pouso: rebote mediano " << median << " m/s, maior " << rebounds.back() << " m/s\n";
        require(landingsOk && median < 1.4f, "Pouso: quicou ou nao agachou");
    }
}

void gameLoopBaselines() {
    const auto forward = [](float from, float to, bool sprint) {
        return [=](float t) {
            ScenarioCommand c;
            if (t >= from && t < to) c.move = {1, 0, 0};
            c.sprint = sprint;
            return c;
        };
    };
    const Scenario scenarios[] = {
        {"parado", 4, {}, {}, {}},
        {"trote e para", 5, {}, forward(0.8f, 3.0f, false), {}},
        {"sprint e para", 5, {}, forward(0.8f, 3.2f, true), {}},
        {"giro de camera 90", 4, {}, [](float t) { ScenarioCommand c; c.lookYaw = t > 1 ? 1.5708f : 0; return c; }, {}},
        {"lado (strafe)", 4, {}, [](float t) { ScenarioCommand c; if (t > 0.8f && t < 3) c.move = {0, 1, 0}; return c; }, {}},
        {"recuo", 4, {}, [](float t) { ScenarioCommand c; if (t > 0.8f && t < 3) c.move = {-1, 0, 0}; return c; }, {}},
    };
    for (const auto& scenario : scenarios) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(scenario.name).find(only) == std::string::npos) continue;
        const auto r = runScenario(scenario);
        report(scenario, r);
        require(r.falls == 0, "Movimento normal derrubou o personagem no modo fisico");
        // No strafe o pe sai do chao (arrastava 1,3 s de 2,2 s andando).
        if (std::string(scenario.name).find("strafe") != std::string::npos)
            require(std::max(r.dragSeconds[0], r.dragSeconds[1]) < 0.8f,
                "No strafe um pe arrasta no chao");
    }
}

void fallScenarios() {
    const auto crate = [](Vec3 at) {
        return [=](PhysicsScene3D& s) { dynamicBox(s, at, {0.2f, 0.6f, 0.1f}, 80, {}); };
    };
    const auto move = [](Vec3 direction, float until) {
        return [=](float t) { ScenarioCommand c; if (t > 0.5f && t < until) c.move = direction; return c; };
    };
    const bool legacy = std::getenv("XLEGACY") != nullptr;
    // O mesmo tropeco, mas a caixa some depois da queda: levantar no plano.
    auto tripCrate = std::make_shared<PhysicsBodyHandle3D>();
    Scenario scenarios[] = {
        {"queda para a frente (tropeco trotando)", 16, crate({3.2f, 0, 0.1f}), move({1, 0, 0}, 2.2f), {}},
        {"queda para a frente, caixa retirada depois", 13,
            [tripCrate](PhysicsScene3D& s) { *tripCrate = dynamicBox(s, {3.2f, 0, 0.1f}, {0.2f, 0.6f, 0.1f}, 80, {}); },
            move({1, 0, 0}, 2.2f),
            [tripCrate](PhysicsScene3D& s, float t, const RagdollState3D&) {
                if (*tripCrate && std::abs(t - 2.6f) < 0.5f / 120) s.destroyBody(*tripCrate);
            }},
        // Em pe, ja desequilibrado 60 graus para tras: tomba de costas (com 35
        // ele se recupera dando passos).
        {"queda para tras (nasce desequilibrado)", 13, {}, {}, {}, false, {}, 0, -60, 0},
        // Includes the relaxed settling interval before muscular recovery.
        // Caixa de 40 cm: na de 20 cm ele tropeca de lado e se recupera; com
        // o estimador de estado (E2) a de 30 cm ficou na fronteira - derruba
        // ou nao conforme a fase do passo ao bater (2 de 6 posicoes). A de
        // 40 cm derruba em todas as posicoes testadas (2,4 a 2,8 m).
        {"queda de lado (strafe contra caixa)", 13, [](PhysicsScene3D& s) { dynamicBox(s, {0, 2.6f, 0.2f}, {0.6f, 0.2f, 0.2f}, 80, {}); },
            move({0, 1, 0}, 2.4f), {}},
        {"nasce deitado de costas", 13, {}, {}, {}, false, {}, 0, -90, 0.16f},
        {"nasce deitado de bruços", 13, {}, {}, {}, false, {}, 0, 90, 0.16f},
        // Cai de ~1,5 m ja tombado para tras (50 graus), sem tocar nada no
        // ar: espernea tentando se equilibrar, bate no chao e levanta.
        {"cai do alto tombado", 13, {}, {}, {}, false, {}, 0, -50, 2.4f},
        // Uma caixa arremessada nas costas: derruba nos dois modos (no
        // antigo, pelo impacto).
        {"atingido por caixa nas costas", 13, {}, {},
            [](PhysicsScene3D& s, float t, const RagdollState3D&) {
                if (std::abs(t - 1.0f) < 0.5f / 120)
                    dynamicBox(s, {-0.9f, 0, 1.2f}, {0.2f, 0.25f, 0.2f}, 70, {9, 0, 0});
            }},
    };
    for (auto& scenario : scenarios) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(scenario.name).find(only) == std::string::npos) continue;
        scenario.legacy = legacy;
        // Cenarios de queda: o obstaculo do tropeco e um que os pes nao veem.
        scenario.unseenObstacles = true;
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        if (legacy) continue;
        const std::string name = scenario.name;
        require(r.falls >= 1, "O cenario de queda nao derrubou (verificar o cenario)");
        // O levantar e do corpo: nada de teleporte (a pelve saltava 677 mm
        // num tick quando o clipe a carregava) e ele termina em pe.
        require(r.getUpWorstRootStep < 0.05f, "O levantar teleportou a pelve");
        require(r.getUpWorstLinkSpeed < 12.0f, "Um membro chicoteou no levantar");
        require(r.standingAtEnd, "O personagem nao conseguiu se levantar");
        // O levantar e dos musculos (a forca inteira das juntas; eram 10-30%
        // com a postura do controlador adaptativo), sem recaida logo depois,
        // sem cambalhota no chao (360 graus na queda de lado) e sem o pe
        // disparando na troca para o controle normal (7,8 m/s, "espacate").
        require(r.getUpJointStrength > 0.9f, "As juntas chegaram fracas ao levantar");
        require(r.relapses == 0, "Caiu de novo logo depois de levantar");
        require(r.settleTumbleDegrees < 180.0f, "O corpo rolou demais no chao antes de levantar");
        require(r.handoffFootSpeed < 6.0f, "Um pe disparou na troca do levantar para o normal");
        // Em pe, reto: sem ficar torto "encaixando" depois da troca (de bruços
        // desviava 14 graus da pose nos 2 s seguintes).
        // (12: com os musculos mais macios, no giro para a camera logo depois
        // o peito atrasa ~10 graus - movimento normal, nao o levantar.)
        require(r.afterGetUpWorstTilt < 12.0f, "Depois de levantar ele ficou torto (longe da pose)");
        // Levantando, nada de "prancha" inclinada segurada pela ajuda (a pelve
        // alta muito alem da inclinacao do clipe; era ate 39 graus por 2 s).
        require(r.getUpHeldLean < 25.0f && r.getUpHeldLeanSeconds < 0.8f,
            "Levantando, ele ficou inclinado segurado pela ajuda (evidente)");
        // (12 s: ele fica ~2,4 s largado se mexendo antes de se arrumar, a
        // pedido; um levantar sofrido - sentar devagar - cabe.)
        require(r.getUpSeconds > 0 && r.getUpSeconds < 12.0f,
            "Recuperacao demorou mais de doze segundos depois da queda");
        // A ajuda nao pode ser o que levanta: em media, uma fracao do peso.
        require(r.getUpAssistMeanShare < 0.35f, "A ajuda do levantar e forte demais (evidente)");
        // Reflexos: caindo de pe, a cabeca nao e a primeira a bater, e bate
        // devagar (medido: 0-2,6 m/s; sem reflexos 2,8 m/s de cabeca primeiro).
        if (name.find("nasce deitado") == std::string::npos) {
            require(r.firstGroundLink != "Head", "Caindo, a cabeca bateu primeiro no chao");
            require(r.headImpactSpeed < 3.0f, "A cabeca bateu forte demais no chao");
        }
    }
}

void pushScenarios() {
    // Bloco de 60 kg solto a 35 cm do peito (z 1,25), com a velocidade do
    // empurrao: momento conhecido (60 a 300 N.s), sem cair antes de chegar.
    const auto shove = [](float speed, float headingDegrees) {
        auto block = std::make_shared<PhysicsBodyHandle3D>();
        return [=](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
            if (std::abs(t - 1.5f) >= 0.5f / 120) return;
            const float a = headingDegrees * 3.14159265f / 180;
            const Vec3 from {std::cos(a), std::sin(a), 0};
            const Vec3 chest = state.links.front().position;
            *block = dynamicBox(scene, {chest.x + from.x * 0.55f, chest.y + from.y * 0.55f, 1.25f},
                {0.12f, 0.25f, 0.15f}, 60, from * -speed);
        };
    };
    struct Case { const char* name; float speed; float heading; };
    const Case cases[] = {
        {"empurrao 60 N.s pela frente", 1.0f, 0}, {"empurrao 120 N.s pela frente", 2.0f, 0},
        {"empurrao 210 N.s pela frente", 3.5f, 0}, {"empurrao 300 N.s pela frente", 5.0f, 0},
        {"empurrao 120 N.s pelo lado", 2.0f, 90}, {"empurrao 210 N.s pelo lado", 3.5f, 90},
        {"empurrao 120 N.s por tras", 2.0f, 180}, {"empurrao 210 N.s por tras", 3.5f, 180},
    };
    for (const auto& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(c.name).find(only) == std::string::npos) continue;
        const Scenario scenario {c.name, 4.5f, {}, {}, shove(c.speed, c.heading)};
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        require(r.worstLinkSpeed < 30.0f, "Um membro estourou no empurrao");
    }
}

void disturbanceScenarios() {
    // Caixa atirada no peito (z 1,3), vinda da frente, com o personagem
    // parado olhando para +X.
    const auto box = [](float mass, float speed, float atSeconds) {
        return [=](PhysicsScene3D& scene, float t, const RagdollState3D&) {
            if (std::abs(t - atSeconds) < 0.5f / 120) {
                const float edge = std::cbrt(mass / 600.0f) * 0.5f; // madeira ~600 kg/m3
                dynamicBox(scene, {0.9f, 0, 1.25f}, {edge, edge, edge}, mass, {-speed, 0, 0});
            }
        };
    };
    const auto runForward = [](float from, bool sprint) {
        return [=](float t) { ScenarioCommand c; if (t > from) c.move = {1, 0, 0}; c.sprint = sprint; return c; };
    };
    // Obstaculo baixo que nao e cenario: uma caixa pesada (dinamica) no
    // chao, no caminho - os pes a veem pela sonda de terreno.
    const auto lowCrate = [](float x, float height, float mass) {
        return [=](PhysicsScene3D& s) { dynamicBox(s, {x, 0, height * 0.5f}, {0.2f, 0.6f, height * 0.5f}, mass, {}); };
    };
    const Scenario scenarios[] = {
        {"trote contra caixa de 20 cm (80 kg) no chao", 8, lowCrate(3.2f, 0.2f, 80),
            [](float t) { ScenarioCommand c; if (t > 0.5f && t < 2.2f) c.move = {1, 0, 0}; return c; }, {}},
        {"sprint contra caixa de 20 cm (80 kg) no chao", 4, lowCrate(5.0f, 0.2f, 80), runForward(0.5f, true), {}},
        {"sprint contra caixa de 35 cm (120 kg) no chao", 4, lowCrate(5.0f, 0.35f, 120), runForward(0.5f, true), {}},
        {"outro boneco parado sozinho (controle)", 4, {}, {}, {}, true, {3.0f, 0, 0}, 3.14159265f},
        {"outro boneco parado sozinho, virado para +X (controle)", 4, {}, {}, {}, true, {3.0f, 0, 0}, 0},
        {"trote contra outro boneco parado", 4, {}, runForward(0.5f, false), {}, true, {3.0f, 0, 0}, 3.14159265f},
        {"sprint contra outro boneco parado", 4, {}, runForward(0.5f, true), {}, true, {5.0f, 0, 0}, 3.14159265f},
        {"sprint de raspao no ombro de outro boneco", 4, {}, runForward(0.5f, true), {}, true, {5.0f, 0.35f, 0}, 3.14159265f},
        {"caixa 15 kg a 3 m/s no peito", 4, {}, {}, box(15, 3, 1.0f)},
        {"caixa 15 kg a 6 m/s no peito", 4, {}, {}, box(15, 6, 1.0f)},
        {"caixa 15 kg a 10 m/s no peito", 4, {}, {}, box(15, 10, 1.0f)},
        {"caixa 40 kg a 5 m/s no peito", 4, {}, {}, box(40, 5, 1.0f)},
        {"trote contra parede", 4, [](PhysicsScene3D& s) { staticBox(s, {3, 0, 1}, {0.1f, 2, 1}); }, runForward(0.5f, false), {}},
        {"trote num meio-fio de 12 cm", 4, [](PhysicsScene3D& s) { staticBox(s, {3.2f, 0, 0.06f}, {0.15f, 2, 0.06f}); }, runForward(0.5f, false), {}},
        {"sprint num meio-fio de 25 cm", 4, [](PhysicsScene3D& s) { staticBox(s, {5, 0, 0.125f}, {0.15f, 2, 0.125f}); }, runForward(0.5f, true), {}},
        {"pulo curto contra plataforma de 1,1 m", 4, [](PhysicsScene3D& s) { staticBox(s, {4.5f, 0, 0.55f}, {1, 2, 0.55f}); },
            [](float t) { ScenarioCommand c; if (t > 0.5f && t < 3) c.move = {1, 0, 0}; c.jump = t > 1.35f && t < 1.4f; return c; }, {}},
    };
    for (const auto& scenario : scenarios) {
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        const std::string name = scenario.name;
        // O contrato: nada explode (o pe em balanco do sprint passa de 20 m/s
        // no mundo; 30 e estouro), e o que a fisica decide aparece.
        require(r.worstLinkSpeed < 30.0f, "Um membro estourou (velocidade absurda)");
        if (name.find("sozinho") != std::string::npos)
            require(r.npcFalls == 0, "O outro boneco caiu sozinho, parado");
        if (name.find("plataforma") != std::string::npos)
            require(r.falls >= 1, "Bater o corpo na plataforma no ar nao derrubou (raiz forte demais?)");
        // A caixa baixa no chao: ate 30/09 a sonda de terreno nao via corpos
        // dinamicos e ele tropecava nela (o teste exigia a queda). Agora os
        // pes a veem (probeTerrain) e o passo passa por cima.
        if (name.find("caixa de 20 cm (80 kg)") != std::string::npos && name.find("trote") != std::string::npos)
            require(r.falls == 0, "Trotando, a caixa baixa no chao (vista pelos pes) derrubou");
    }
}

// Estado fisico comum (etapa E2): o mesmo corpo da o mesmo estado para o
// personagem controlado (com RagdollDynamics3D) e para o boneco solto (sem);
// apoio medido nao e apoio planejado; a evidencia do contato e rotulada.
// Etapa E3 (planejador unico): o passo existe de verdade - o pe sai do chao,
// anda e pousa por contato medido - e o arrastado pela PhysGun da passos.
void plannerStepChecks() {
    const auto shove = [](float speed, float headingDegrees) {
        auto block = std::make_shared<PhysicsBodyHandle3D>();
        return [=](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
            if (std::abs(t - 1.5f) >= 0.5f / 120) return;
            const float a = headingDegrees * 3.14159265f / 180;
            const Vec3 from {std::cos(a), std::sin(a), 0};
            const Vec3 chest = state.links.front().position;
            *block = dynamicBox(scene, {chest.x + from.x * 0.55f, chest.y + from.y * 0.55f, 1.25f},
                {0.12f, 0.25f, 0.15f}, 60, from * -speed);
        };
    };
    // Um passo real: saiu do chao (sola 3 cm acima de onde estava), andou
    // 8 cm e pousou por contato medido.
    const auto isReal = [](const PlannerStepTrace& step) {
        return step.landedAt >= 0 && !step.dragged && step.maxSoleLift >= 0.03f
            && step.travel >= 0.08f && (step.evidence == ContactEvidence3D::EstimatedImpulse
                || step.evidence == ContactEvidence3D::SolvedImpulse);
    };
    struct Case { const char* name; float speed; float heading; };
    const Case cases[] = {
        {"passo: empurrao 210 N.s pela frente", 3.5f, 0},
        {"passo: empurrao 210 N.s pelo lado", 3.5f, 90},
        {"passo: empurrao 300 N.s pela frente", 5.0f, 0},
    };
    int withRealStep = 0;
    for (const auto& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(c.name).find(only) == std::string::npos) continue;
        const Scenario scenario {c.name, 4.5f, {}, {}, shove(c.speed, c.heading)};
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        float firstRecovery = -1;
        int real = 0, landed = 0;
        for (const auto& step : r.plannerSteps) {
            // Os passos depois do empurrao (no assentamento do nascimento pode
            // haver recuperacao tambem).
            // Um passo que ja estava no ar e virou recuperacao conta como
            // resposta imediata.
            if (step.finalReason != StepReason3D::Recovery) continue;
            if (step.releasedAt < 1.5f && !(step.landedAt > 1.55f)) continue;
            if (firstRecovery < 0) firstRecovery = std::max(1.5f, step.releasedAt);
            if (isReal(step)) ++real;
            if (step.landedAt >= 0) ++landed;
        }
        std::cout << "    recuperacao: primeiro passo " << (firstRecovery - 1.5f) << " s depois do empurrao, "
            << landed << " pousados por contato, " << real << " reais (sola > 3 cm, > 8 cm), captura ate "
            << r.captureOutsidePeakAfterPush << " m fora do apoio\n";
        // So quando o empurrao tirou o ponto de captura do apoio (mais de
        // 8 cm nos 0,3 s seguintes): o lateral de 210 N.s as vezes e
        // absorvido num pe so (Debug: o outro pe sobe 7 cm, a captura fica a
        // ate 6,6 cm da sola, inclinacao 6,9 graus) - ali nao dar passo e o
        // certo.
        if (r.captureOutsidePeakAfterPush > 0.08f)
            require(firstRecovery >= 1.5f && firstRecovery < 1.75f,
                "Empurrado, o pe nao saiu logo (mais de 0,25 s)");
        if (real > 0) ++withRealStep;
        // Dois passos de recuperacao seguidos, pousados por contato (sair do
        // chao em todos depende da troca de peso pelas juntas - E4; o Debug
        // deu 1 passo real de 3 onde o Release deu 4).
        if (c.speed >= 5.0f)
            require(landed >= 2, "Empurrao forte: menos de dois passos de recuperacao seguidos");
    }
    if (!std::getenv("XSCENARIO"))
        require(withRealStep >= 2, "Empurrado, o pe foi arrastado em vez de dar passo (2 de 3 casos)");
    // Arrastado pela PhysGun pelo peito: anda com o corpo dando passos (antes
    // do planejador unico ele ficava parado e caia).
    for (const auto& drag : { std::pair<const char*, Vec3> {"passo: arrastado pela PhysGun para a frente", {0.8f, 0, 0}},
             std::pair<const char*, Vec3> {"passo: arrastado pela PhysGun de lado", {0, 0.8f, 0}} }) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(drag.first).find(only) == std::string::npos) continue;
        Scenario scenario {drag.first, 5.0f, {}, {}, {}};
        scenario.dragLink = "UpperChest";
        scenario.dragVelocity = drag.second;
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        int landed = 0;
        for (const auto& step : r.plannerSteps)
            if (step.landedAt > 1.0f && step.landedAt >= 0) ++landed;
        std::cout << "    arrastado: " << landed << " passos pousados por contato, corpo andou "
            << r.dragDistance << " m" << (r.falls > 0 ? ", caiu" : "") << "\n";
        // O que a E3 corrigiu: arrastado, ele anda dando passos (antes ficava
        // parado com os pes no chao). Cair ou nao num caso so e moeda: a
        // varredura de arrasto (MATTERENGINE_TEST_FILTER=sweepdrag, 32 casos)
        // mede o padrao caindo em ~1/3 dos arrastos pelo peito (5 de 16) - o
        // Debug derrubou o "para a frente" logo depois de soltar, o Release
        // nao. Com as pernas carregando o corpo (E4, modo 7) sao 3 de 32.
        require(landed >= 3 && r.dragDistance > 0.9f, "Arrastado pela PhysGun, nao deu passos");
    }
}

// Etapa E4, bancada: o torque de contato (CharacterWholeBody3D) sustenta o
// corpo pelas pernas. Pernas macias (rigidez x0,2), nenhuma forca na pelve:
// com o torque de contato a pelve tem de ficar bem mais alta do que sem ele
// (sinal e grandeza do trabalho virtual no corpo inteiro).
float softLegsPelvisHeight(bool contactTorque) {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const auto& profile = character.profile;
    const ClipSet clips = loadClips(character);
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    staticBox(*scene, {0, 0, -0.1f}, {60, 60, 0.1f});
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9301;
    spawn.active = true;
    spawn.pelvisPosition = {0, 0, profile.standingRootHeightMeters};
    const auto handle = scene->createRagdoll(profile, spawn);
    CharacterLocomotion3D locomotion;
    locomotion.reset(profile, scene->ragdollState(handle));
    AdaptivePhysicalCharacter3D adaptive;
    adaptive.reset(profile);
    adaptive.settings().jointSupport = true;
    adaptive.settings().jointBalance = true;
    std::array<std::size_t, 2> feet {};
    std::vector<bool> leg(profile.links.size(), false);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id == "LeftFoot") feet[0] = i;
        if (id == "RightFoot") feet[1] = i;
        leg[i] = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
            || id.find("Foot") != std::string::npos;
    }
    constexpr float dt = 1.0f / 120;
    float height = 0.0f;
    for (int tick = 0; tick < 240; ++tick) {
        const RagdollState3D state = scene->ragdollState(handle);
        CharacterLocomotionInput3D input;
        input.rootPositionWorld = {0, 0, profile.standingRootHeightMeters};
        input.forceDrivenRoot = true;
        input.controlled = true;
        for (std::size_t side = 0; side < 2; ++side)
            input.footGround[side] = scene->probeGround(
                state.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
        locomotion.update(profile, clips.animations, state, scene->ragdollDynamics(handle), input, dt);
        AdaptivePhysicalIntent3D intent;
        intent.navigationRootWorld = input.rootPositionWorld;
        intent.footGround = input.footGround;
        intent.physicalState = &locomotion.physicalState();
        intent.contactPlan = &locomotion.contactPlan();
        adaptive.update(profile, state, locomotion.output().guide, intent, dt);
        const auto& out = adaptive.output();
        std::vector<RagdollDriveTarget3D> targets = locomotion.output().driveTargets;
        for (RagdollDriveTarget3D& target : targets) {
            if (!leg[target.linkIndex]) continue;
            target.stiffnessScale *= 0.2f;
            if (contactTorque && target.linkIndex < out.jointFeedforward.size())
                target.feedforwardTorqueNewtonMeters +=
                    out.jointFeedforward[target.linkIndex][static_cast<std::size_t>(target.axis)];
        }
        scene->setRagdollActiveDriveTargets(handle, targets, locomotion.output().gravityCompensationEnabled);
        scene->setRagdollAnimationConstraint(handle, locomotion.output().guide);
        scene->simulate(dt);
        if (tick >= 180) height += scene->ragdollState(handle).links.front().position.z / 60.0f;
    }
    return height;
}

AdaptivePhysicalOutput3D standingAssistOutput(float retained) {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto character = loadRagdollCharacter3D(
        assets + "/characters/football_player/character.json");
    const auto& profile = character.profile;
    const ClipSet clips = loadClips(character);
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    staticBox(*scene, {0, 0, -0.1f}, {20, 20, 0.1f});
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9304;
    spawn.active = true;
    spawn.pelvisPosition = {0, 0, profile.standingRootHeightMeters};
    const auto handle = scene->createRagdoll(profile, spawn);
    RagdollState3D state = scene->ragdollState(handle);
    CharacterLocomotion3D locomotion;
    locomotion.reset(profile, state);
    AdaptivePhysicalCharacter3D adaptive;
    adaptive.reset(profile);
    adaptive.settings().jointSupport = true;
    adaptive.settings().jointBalance = true;
    adaptive.settings().jointPosture = true;
    adaptive.settings().jointWhileWalking = true;
    adaptive.settings().legsAssistRetained = retained;
    adaptive.settings().legsWeightFraction = 0.0f; // E5 real do Workbench.
    std::array<std::size_t, 2> feet {};
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    constexpr float dt = 1.0f / 120.0f;
    CharacterPhysicalState3D physical;
    physical.valid = true;
    physical.massKilograms = 70.0f;
    physical.rootPositionWorld = state.links.front().position;
    physical.rootOrientationWorld = state.links.front().orientation;
    physical.centerOfMassWorld = state.links.front().position;
    physical.rootUpright = 1.0f;
    physical.supported = true;
    physical.supportHeight = 0.0f;
    physical.supportPointCount = 8;
    CharacterContactPlan3D plan;
    plan.valid = true;
    for (std::size_t side = 0; side < 2; ++side) {
        physical.feet[side].linkIndex = static_cast<std::uint32_t>(feet[side]);
        physical.feet[side].touching = true;
        physical.feet[side].supporting = true;
        physical.feet[side].contactObserved = true;
        physical.feet[side].loadNewtons = 0.5f * 70.0f * 9.81f;
        physical.feet[side].soleCenterWorld = state.links[feet[side]].position;
        plan.feet[side].followingGait = false;
        RagdollInteraction3D contact;
        contact.linkIndex = static_cast<std::uint32_t>(feet[side]);
        contact.normalWorld = {0, 0, 1};
        contact.positionWorld = state.links[feet[side]].position;
        contact.pointCount = 1;
        state.interactions.push_back(contact);
    }
    for (int tick = 0; tick < 180; ++tick) {
        CharacterLocomotionInput3D input;
        input.rootPositionWorld = {0, 0,
            profile.standingRootHeightMeters + 0.10f};
        input.forceDrivenRoot = true;
        input.controlled = true;
        for (std::size_t side = 0; side < 2; ++side)
            input.footGround[side] = scene->probeGround(
                state.links[feet[side]].position + Vec3 {0, 0, 0.10f},
                0.05f, 0, 0.40f, 48);
        locomotion.update(profile, clips.animations, state,
            scene->ragdollDynamics(handle), input, dt);
        AdaptivePhysicalIntent3D intent;
        intent.navigationRootWorld = input.rootPositionWorld;
        intent.footGround = input.footGround;
        intent.physicalState = &physical;
        intent.contactPlan = &plan;
        RagdollAnimationConstraint3D guide = locomotion.output().guide;
        guide.rootPositionWorld = state.links.front().position
            + Vec3 {0.10f, 0.0f, 0.10f};
        guide.rootOrientationWorld = (Quaternion::fromAxisAngle(
            {0.0f, 1.0f, 0.0f}, 0.10f)
            * state.links.front().orientation).normalized();
        adaptive.update(profile, state, guide, intent, dt);
    }
    return adaptive.output();
}

void standingAssistSemanticsChecks() {
    const auto zero = standingAssistOutput(0.0f);
    const auto thirty = standingAssistOutput(0.30f);
    const auto full = standingAssistOutput(1.0f);
    const auto jointEffort = [](const AdaptivePhysicalOutput3D& output) {
        float sum = 0.0f;
        for (const auto& link : output.jointFeedforward)
            for (float torque : link) sum += std::abs(torque);
        return sum;
    };
    std::printf("Ajuda da pelve: valid %d apoio %.3f vertical %.3f shares 0%% %.3f/%.3f | Fx 0/30/100 %.4f %.4f %.4f N | Fz %.4f %.4f %.4f N | torque 0/100 %.4f %.4f Nm | juntas em 0%% %.2f Nm\n",
        zero.valid, zero.supportAuthority, zero.uprightAuthorityShare,
        zero.legShare, zero.balanceShare, zero.rootForceWorld.x,
        thirty.rootForceWorld.x, full.rootForceWorld.x,
        zero.rootForceWorld.z, thirty.rootForceWorld.z, full.rootForceWorld.z,
        zero.rootTorqueWorld.length(), full.rootTorqueWorld.length(),
        jointEffort(zero));
    require(zero.legShare > 0.98f && zero.balanceShare > 0.98f,
        "Ajuda parado 0% nao transferiu suporte/equilibrio para as pernas");
    require(std::abs(zero.rootForceWorld.x) < 0.05f
            * std::abs(full.rootForceWorld.x),
        "Ajuda parado 0% ainda corrige a pelve pela raiz");
    require(std::abs(zero.rootForceWorld.z) < 0.05f
            * std::abs(full.rootForceWorld.z),
        "Ajuda parado 0% ainda sustenta a pelve pela raiz");
    require(zero.rootTorqueWorld.length() < 0.05f
            * full.rootTorqueWorld.length(),
        "Ajuda parado 0% ainda corrige a postura pela pelve");
    require(jointEffort(zero) > 1.0f,
        "Ajuda da pelve 0% deveria preservar o trabalho fisico das pernas");
    const float ratio = thirty.rootForceWorld.x
        / std::max(1.0f, full.rootForceWorld.x);
    require(ratio > 0.27f && ratio < 0.33f,
        "Ajuda parado 30% nao corresponde a 30% da sustentacao da pelve");
    std::printf("Ajuda da pelve parada: raiz planar 0%% %.2f N | 30%% %.2f N | 100%% %.2f N | razao %.3f\n",
        zero.rootForceWorld.x, thirty.rootForceWorld.x,
        full.rootForceWorld.x, ratio);
}

// Diagnostico E4: o mesmo torque de postura (mundo) aplicado como par
// pelve x pes (modo 1), como torques de junta da cadeia pe->pelve aplicados
// direto nos links (modo 2) ou pelos motores (modo 3). Devolve o giro da
// pelve (graus) e o deslocamento dos pes (mm) 0,15 s depois, descontado o
// caso sem torque (modo 0).
std::array<float, 3> postureTransmission(int mode, Vec3 torque, bool oneLeg = false) {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const auto& profile = character.profile;
    const ClipSet clips = loadClips(character);
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    staticBox(*scene, {0, 0, -0.1f}, {60, 60, 0.1f});
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9302;
    spawn.active = true;
    spawn.pelvisPosition = {0, 0, profile.standingRootHeightMeters};
    const auto handle = scene->createRagdoll(profile, spawn);
    CharacterLocomotion3D locomotion;
    locomotion.reset(profile, scene->ragdollState(handle));
    CharacterWholeBody3D body;
    body.prepare(profile);
    std::array<std::size_t, 2> feet {};
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    constexpr float dt = 1.0f / 120;
    Quaternion before;
    std::vector<Quaternion> linksBefore(profile.links.size());
    std::array<Vec3, 2> feetBefore {};
    std::vector<std::array<float, 3>> joint;
    for (int tick = 0; tick < 150 + 18; ++tick) {
        const RagdollState3D state = scene->ragdollState(handle);
        if (tick == 150) {
            before = state.links.front().orientation;
            for (std::size_t i = 0; i < state.links.size(); ++i) linksBefore[i] = state.links[i].orientation;
            for (std::size_t side = 0; side < 2; ++side) feetBefore[side] = state.links[feet[side]].position;
        }
        CharacterLocomotionInput3D input;
        input.rootPositionWorld = {0, 0, profile.standingRootHeightMeters};
        input.forceDrivenRoot = true;
        input.controlled = true;
        for (std::size_t side = 0; side < 2; ++side)
            input.footGround[side] = scene->probeGround(
                state.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
        locomotion.update(profile, clips.animations, state, scene->ragdollDynamics(handle), input, dt);
        std::vector<RagdollDriveTarget3D> targets = locomotion.output().driveTargets;
        const bool on = tick >= 150 && mode != 0;
        joint.assign(profile.links.size(), {0.0f, 0.0f, 0.0f});
        if (on && mode >= 2) {
            std::array<ContactWrench3D, 2> wrenches {};
            for (std::size_t side = 0; side < 2; ++side) {
                wrenches[side].linkIndex = static_cast<std::uint32_t>(feet[side]);
                wrenches[side].pointWorld = state.links[feet[side]].position;
                wrenches[side].torqueWorld = oneLeg ? (side == 0 ? torque : Vec3 {}) : torque * 0.5f;
            }
            body.addContactTorques(state, std::span<const ContactWrench3D>(wrenches.data(), 2), joint);
            if (mode == 3)
                for (RagdollDriveTarget3D& target : targets)
                    target.feedforwardTorqueNewtonMeters += joint[target.linkIndex][static_cast<std::size_t>(target.axis)];
        }
        scene->setRagdollActiveDriveTargets(handle, targets, locomotion.output().gravityCompensationEnabled);
        scene->setRagdollAnimationConstraint(handle, locomotion.output().guide);
        if (on && mode == 1) {
            scene->applyRagdollControlRootForce(handle, {}, torque);
            for (std::size_t side = 0; side < 2; ++side)
                scene->applyRagdollControlLinkForce(handle, static_cast<std::uint32_t>(feet[side]), {},
                    oneLeg ? (side == 0 ? torque * -1.0f : Vec3 {}) : torque * -0.5f);
        }
        if (on && mode == 2) {
            for (std::size_t i = 1; i < profile.links.size(); ++i) {
                const auto& link = profile.links[i];
                const Quaternion frame = state.links[i].orientation * link.modelOrientation.conjugate()
                    * link.inboundJoint.frameModelOrientation;
                const Vec3 applied = frame.rotate(Vec3 {joint[i][0], joint[i][1], joint[i][2]});
                if (applied.lengthSquared() < 1e-8f) continue;
                scene->applyRagdollControlLinkForce(handle, static_cast<std::uint32_t>(i), {}, applied);
                scene->applyRagdollControlLinkForce(handle, static_cast<std::uint32_t>(link.parentIndex), {}, applied * -1.0f);
            }
        }
        scene->simulate(dt);
    }
    const RagdollState3D state = scene->ragdollState(handle);
    if (std::getenv("XTRANSLINKS") && oneLeg) {
        std::printf("  modo %d:", mode);
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            const std::string& id = profile.links[i].id;
            if (i != 0 && id != "LeftThigh" && id != "LeftShin" && id != "LeftFoot") continue;
            const Quaternion delta = (state.links[i].orientation * linksBefore[i].conjugate()).normalized();
            const float angle = 2.0f * std::acos(std::clamp(std::abs(delta.w), 0.0f, 1.0f)) * 57.3f;
            const Vec3 v { delta.x, delta.y, delta.z };
            const Vec3 axis = v.lengthSquared() > 1e-12f ? v * ((delta.w < 0 ? -1.0f : 1.0f) / v.length()) : Vec3 {};
            std::printf(" %s (%+.2f %+.2f %+.2f)", id.c_str(), axis.x * angle, axis.y * angle, axis.z * angle);
        }
        std::printf("\n");
    }
    const Vec3 upBefore = before.rotate({0, 0, 1});
    const Vec3 upAfter = state.links.front().orientation.rotate({0, 0, 1});
    const Vec3 axis = cross(upBefore, upAfter);
    float moved = 0.0f;
    for (std::size_t side = 0; side < 2; ++side)
        moved = std::max(moved, (state.links[feet[side]].position - feetBefore[side]).length());
    return { axis.x * 57.3f, axis.y * 57.3f, moved * 1000.0f };
}

void postureTransmissionChecks() {
    {
        const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
        const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
        for (const auto& link : character.profile.links) {
            if (link.id.find("Left") != 0 || (link.id.find("Thigh") == std::string::npos
                && link.id.find("Shin") == std::string::npos && link.id.find("Foot") == std::string::npos)) continue;
            const Quaternion frame = link.inboundJoint.frameModelOrientation;
            std::printf("JUNTA %s:", link.id.c_str());
            for (int a = 0; a < 3; ++a) {
                const Vec3 axis = frame.rotate(a == 0 ? Vec3 {1, 0, 0} : a == 1 ? Vec3 {0, 1, 0} : Vec3 {0, 0, 1});
                std::printf(" eixo%d %s (%+.2f %+.2f %+.2f) [%.2f, %.2f]", a, link.inboundJoint.axes[a].enabled ? "livre" : "travado",
                    axis.x, axis.y, axis.z, link.inboundJoint.axes[a].minimumRadians, link.inboundJoint.axes[a].maximumRadians);
            }
            std::printf("\n");
        }
    }
    for (const bool oneLeg : { false, true })
    for (const Vec3 torque : { Vec3 {150, 0, 0}, Vec3 {0, 150, 0} }) {
        const auto base = postureTransmission(0, torque);
        for (int mode = 1; mode <= 3; ++mode) {
            const auto r = postureTransmission(mode, torque, oneLeg);
            std::printf("TRANSMISSAO %s torque (%.0f, %.0f) modo %d: giro da pelve (%+.2f, %+.2f) graus, pes %.1f mm (sem torque: %.1f)\n",
                oneLeg ? "uma perna" : "duas pernas", torque.x, torque.y, mode, r[0] - base[0], r[1] - base[1], r[2], base[2]);
        }
    }
}

void wholeBodyChecks() {
    const float with = softLegsPelvisHeight(true);
    const float without = softLegsPelvisHeight(false);
    std::cout << "Corpo inteiro: pernas macias, pelve a " << with << " m com o torque de contato, "
        << without << " m sem\n";
    require(with > without + 0.05f, "O torque de contato nao sustenta o corpo pelas pernas");
    // Sinal e grandeza do torque pelas juntas (E4): um torque de postura
    // levado do chao ate a pelve pelos motores das pernas gira a pelve como o
    // mesmo torque aplicado direto (par pelve x pes), com os dois pes no chao.
    for (const Vec3 torque : { Vec3 {150, 0, 0}, Vec3 {0, 150, 0} }) {
        const auto base = postureTransmission(0, torque);
        const auto pair = postureTransmission(1, torque);
        const auto chain = postureTransmission(3, torque);
        const float axisPair = torque.x != 0 ? pair[0] - base[0] : pair[1] - base[1];
        const float axisChain = torque.x != 0 ? chain[0] - base[0] : chain[1] - base[1];
        std::cout << "Corpo inteiro: torque de postura (" << torque.x << ", " << torque.y
            << ") gira a pelve " << axisChain << " graus pelas juntas, " << axisPair << " direto\n";
        require(axisPair > 1.0f && axisChain > 0.7f * axisPair && axisChain < 1.3f * axisPair,
            "O torque de postura pelas juntas nao chega a pelve com o sinal e a grandeza do par direto");
    }
}

void stateEstimatorChecks() {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const auto& profile = character.profile;
    const ClipSet clips = loadClips(character);
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    staticBox(*scene, {0, 0, -0.1f}, {60, 60, 0.1f});
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9201;
    spawn.active = true;
    spawn.pelvisPosition = {0, 0, profile.standingRootHeightMeters};
    const auto handle = scene->createRagdoll(profile, spawn);
    CharacterLocomotion3D controlled, loose;
    controlled.reset(profile, scene->ragdollState(handle));
    loose.reset(profile, scene->ragdollState(handle));
    // O corpo de pe como no jogo (com o adaptativo): sem ajuda nenhuma, so as
    // pernas, e a meta da etapa E4 - aqui se mede o estimador.
    AdaptivePhysicalCharacter3D adaptive;
    adaptive.reset(profile);
    std::array<std::size_t, 2> feet {};
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    constexpr float dt = 1.0f / 120;
    for (int tick = 0; tick < 360; ++tick) {
        const RagdollState3D state = scene->ragdollState(handle);
        CharacterLocomotionInput3D input;
        input.rootPositionWorld = {0, 0, profile.standingRootHeightMeters};
        input.forceDrivenRoot = true;
        for (std::size_t side = 0; side < 2; ++side)
            input.footGround[side] = scene->probeGround(
                state.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
        input.controlled = true;
        controlled.update(profile, clips.animations, state, scene->ragdollDynamics(handle), input, dt);
        input.controlled = false;
        loose.update(profile, clips.animations, state, RagdollDynamics3D {}, input, dt);
        const auto& a = controlled.physicalState();
        const auto& b = loose.physicalState();
        require(a.valid && b.valid, "Estado fisico invalido");
        require((a.centerOfMassWorld - b.centerOfMassWorld).length() < 1e-6f
            && (a.centerOfMassVelocityWorld - b.centerOfMassVelocityWorld).length() < 1e-6f
            && a.supportPointCount == b.supportPointCount
            && std::abs(a.captureOutsideMeters - b.captureOutsideMeters) < 1e-6f
            && (a.externalForceWorld - b.externalForceWorld).length() < 1e-4f,
            "Controlado e boneco solto deram estados fisicos diferentes para o mesmo corpo");
        AdaptivePhysicalIntent3D intent;
        intent.navigationRootWorld = input.rootPositionWorld;
        intent.footGround = input.footGround;
        intent.physicalState = &controlled.physicalState();
        intent.contactPlan = &controlled.contactPlan();
        adaptive.update(profile, state, controlled.output().guide, intent, dt);
        applyCharacterControl3D(*scene, handle, controlled.output(), &adaptive.output(), false);
        scene->simulate(dt);
    }
    // De pe, parado: os dois pes sustentam, por contato com impulso ESTIMADO
    // (Jolt), e o ponto de captura esta dentro do apoio.
    const RagdollState3D standing = scene->ragdollState(handle);
    CharacterStateEstimator3D estimator;
    std::array<GroundProbeResult3D, 2> ground {};
    for (std::size_t side = 0; side < 2; ++side)
        ground[side] = scene->probeGround(standing.links[feet[side]].position + Vec3{0, 0, 0.10f}, 0.05f, 0, 0.40f, 48);
    const auto& measured = estimator.update(profile, standing, ground, dt);
    std::cout << "Estado fisico de pe: apoio " << measured.feet[0].supporting << measured.feet[1].supporting
        << ", evidencia " << static_cast<int>(measured.feet[0].evidence) << "/" << static_cast<int>(measured.feet[1].evidence)
        << ", captura fora " << measured.captureOutsideMeters * 1000 << " mm, massa " << measured.massKilograms << " kg\n";
    require(measured.feet[0].supporting && measured.feet[1].supporting, "De pe, um pe nao conta como apoio");
    require(measured.feet[0].evidence == ContactEvidence3D::EstimatedImpulse
        || measured.feet[1].evidence == ContactEvidence3D::EstimatedImpulse,
        "Contato do Jolt sem o rotulo de impulso estimado");
    require(measured.captureOutsideMeters < 0.01f, "De pe e parado, o ponto de captura saiu do apoio");
    // O pe esquerdo 5 cm no ar e sem contato (o planejador pode ate te-lo como
    // travado): nao e apoio.
    RagdollState3D lifted = standing;
    lifted.links[feet[0]].position.z += 0.05f;
    std::erase_if(lifted.contacts, [&](const RagdollContactPoint3D& c) { return c.linkIndex == feet[0]; });
    CharacterStateEstimator3D liftedEstimator;
    const auto& air = liftedEstimator.update(profile, lifted, ground, dt);
    require(!air.feet[0].supporting && air.feet[1].supporting,
        "Pe no ar contado como apoio (apoio medido confundido com planejado)");
}

// Outro boneco (nao controlado) nascendo deitado, sem ninguem mexer, 20 s:
// o bug do laboratorio - ao levantar ele era arremessado. A ancora virtual
// dele ia atras do corpo comparando com a referencia da locomocao, que no
// levantar esta presa a outro ponto: acumulava metros de erro, e na saida a
// referencia saltava (milhares de m/s de velocidade).
void npcGetUpScenarios() {
    struct Case { const char* name; float pitch; float roll; };
    const Case cases[] = {
        {"outro boneco nasce de bruços e levanta", 90, 0},
        {"outro boneco nasce de costas e levanta", -90, 0},
        {"outro boneco nasce de lado e levanta", 0, 90},
    };
    for (const auto& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(c.name).find(only) == std::string::npos) continue;
        Scenario scenario {c.name, 20.0f, {}, {}, {}};
        scenario.npc = true;
        scenario.npcPosition = {0, 4, 0};
        scenario.npcSpawnPitchDegrees = c.pitch;
        scenario.npcSpawnRollDegrees = c.roll;
        scenario.npcSpawnPelvisHeight = 0.16f;
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        require(r.npcAnchorErrorPeak < 0.5f, "A ancora do outro boneco se afastou do corpo");
        require(r.npcGuideSpeedPeak < 5.0f, "A referencia do outro boneco saltou (velocidade absurda)");
        require(r.npcAfterGetUpLinkPeak < 6.0f, "O outro boneco foi arremessado ao levantar");
        // De lado ele ainda emperra na fase de sentar do levantar de costas e
        // tenta de novo (limitacao conhecida; o levantar por contatos e da
        // etapa E7 da reconstrucao). Aqui so os invariantes do arremesso.
        if (c.roll == 0) {
            require(r.npcGetUps >= 1, "O outro boneco nao levantou");
            require(r.npcStandingAtEnd, "O outro boneco nao terminou de pe");
        }
    }
}

// Pressao continua: uma caixa segura pela PhysGun (o mesmo D6 do laboratorio)
// e levada atraves do boneco - o "empurrar com um objeto" do usuario. O alvo
// sai 0,70 m do corpo, no peito, e anda `travel` m a `speed` m/s; depois a
// caixa e solta.
std::function<void(PhysicsScene3D&, float, const RagdollState3D&)> pressWithBox(
    float speed, float headingDegrees, float travel, float height = 1.15f, float start = 1.0f) {
    struct Press { PhysicsBodyHandle3D box; Vec3 origin; bool active = false; bool done = false; };
    auto press = std::make_shared<Press>();
    return [=](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
        if (press->done || t < start) return;
        const float a = headingDegrees * 3.14159265f / 180;
        const Vec3 from {std::cos(a), std::sin(a), 0};
        if (!press->active) {
            const Vec3 pelvis = state.links.front().position;
            press->origin = {pelvis.x + from.x * 0.70f, pelvis.y + from.y * 0.70f, height};
            press->box = dynamicBox(scene, press->origin, {0.18f, 0.28f, 0.22f}, 20, {});
            PhysicsGrabTarget3D target;
            target.position = press->origin;
            target.lockOrientation = true;
            press->active = scene.beginGrab(press->box, {}, target, PhysicsHandleSettings3D {});
            if (!press->active) press->done = true;
            return;
        }
        const float elapsed = t - start;
        if (elapsed * speed >= travel) {
            scene.endGrab();
            press->done = true;
            return;
        }
        PhysicsGrabTarget3D target;
        target.position = press->origin - from * (speed * elapsed);
        target.lockOrientation = true;
        scene.updateGrabTarget(target, PhysicsHandleSettings3D {});
    };
}

void pressureScenarios() {
    const auto walk = [](Vec3 direction) {
        return [=](float t) { ScenarioCommand c; if (t > 0.4f) c.move = direction; return c; };
    };
    struct Case { const char* name; float speed; float heading; float travel; bool walking; };
    int moderateFalls = 0;
    const Case cases[] = {
        {"caixa empurrando devagar pela frente (0,4 m/s)", 0.4f, 0, 1.6f, false},
        {"caixa empurrando pela frente (0,8 m/s)", 0.8f, 0, 2.0f, false},
        {"caixa empurrando pelas costas (0,8 m/s)", 0.8f, 180, 2.0f, false},
        {"caixa empurrando de lado (0,8 m/s)", 0.8f, 90, 2.0f, false},
        {"caixa empurrando rapido pelas costas (1,6 m/s)", 1.6f, 180, 2.4f, false},
        {"caixa empurrando de lado andando", 0.8f, 90, 2.0f, true},
    };
    // Arrastado pela PhysGun pelo peito (o outro relato do usuario: "so
    // ficava parado sem mover os pes").
    for (const auto& drag : { std::pair<const char*, Vec3> {"arrastado pela PhysGun pelo peito para a frente", {0.8f, 0, 0}},
             std::pair<const char*, Vec3> {"arrastado pela PhysGun pelo peito de lado", {0, 0.8f, 0}} }) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(drag.first).find(only) == std::string::npos) continue;
        Scenario scenario {drag.first, 5.0f, {}, {}, {}};
        scenario.dragLink = "UpperChest";
        scenario.dragVelocity = drag.second;
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
    }
    for (const auto& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"))
            if (std::string(c.name).find(only) == std::string::npos) continue;
        const float a = c.heading * 3.14159265f / 180;
        Scenario scenario {c.name, 6.0f, {}, c.walking ? std::function<ScenarioCommand(float)>(walk({1, 0, 0}))
            : std::function<ScenarioCommand(float)>{}, pressWithBox(c.speed, c.heading, c.travel,
                1.15f, c.walking ? 1.4f : 1.0f)};
        scenario.pushDirection = {-std::cos(a), -std::sin(a), 0};
        const ScenarioResult r = runScenario(scenario);
        report(scenario, r);
        require(r.worstLinkSpeed < 30.0f, "Um membro estourou sob pressao");
        // Ate 0,8 m/s: da passos logo (nada de ficar so inclinando). Cair ou
        // nao num caso isolado e caotico (a varredura de 216 empurroes da ~6%
        // de quedas): nos quatro moderados parados, no maximo uma queda. A
        // taxa de queda e acompanhada pela varredura (MATTERENGINE_TEST_FILTER
        // =sweep) nos gates. A 1,6 m/s pelas costas pode cair.
        if (c.speed <= 0.8f) {
            if (!c.walking) moderateFalls += r.falls > 0 ? 1 : 0;
            if (!c.walking && r.firstPressAt >= 0)
                require(r.firstStepAfterPress >= 0 && r.firstStepAfterPress < 0.6f,
                    "Empurrado, ele demorou a dar o primeiro passo");
        }
    }
    require(moderateFalls <= 1, "Empurrado por uma caixa (ate 0,8 m/s), caiu mais de uma vez");
}

// Varredura de pressao: 3 ritmos x 8 direcoes (XSWEEPPART=k/n divide entre
// processos). Resumo: quedas e inclinacao maxima media - para comparar
// variantes num sistema caotico, onde um cenario so engana.
void pressureSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0, falls = 0, count = 0;
    double tiltSum = 0;
    const auto run = [&](const char* kind, float speed, int heading, Scenario scenario) {
        const float a = heading * 3.14159265f / 180;
        scenario.pushDirection = {-std::cos(a), -std::sin(a), 0};
        const ScenarioResult r = runScenario(scenario);
        if (std::getenv("XSTEPS")) report(scenario, r);
        int lifted = 0, dragged = 0, recoverySteps = 0;
        for (const auto& step : r.plannerSteps) {
            if (step.finalReason != StepReason3D::Recovery) continue;
            ++recoverySteps;
            if (step.dragged) ++dragged;
            if (step.maxSoleLift >= 0.03f && step.landedAt >= 0) ++lifted;
        }
        std::printf("SWEEP %s %.1f %3d quedas %d tilt %.1f passos %d segurado %.2f recuperacao %d levantados %d arrastados %d\n",
            kind, speed, heading, r.falls, r.worstTilt, r.steps, r.heldOutsideSeconds, recoverySteps, lifted, dragged);
        falls += r.falls > 0 ? 1 : 0;
        tiltSum += std::min(90.0f, r.worstTilt);
        ++count;
    };
    // Tres instantes do empurrao (fases diferentes do balanco parado): o
    // resultado de um caso so muda com detalhes minimos; com 216 casos a
    // comparacao entre variantes deixa de ser ruido.
    for (const float delay : { 0.0f, 0.13f, 0.27f }) {
    for (const float height : { 1.15f, 0.95f }) {
        for (const float speed : { 0.6f, 0.8f, 1.0f }) {
            for (int heading = 0; heading < 360; heading += 45) {
                if (index++ % parts != part) continue;
                run(height > 1.0f ? "peito" : "cintura", speed, heading, Scenario {"varredura", 6.0f, {}, {},
                    pressWithBox(speed, static_cast<float>(heading), 2.0f, height, 1.0f + delay)});
            }
        }
    }
    }
    // Impulso: bloco de 60 kg solto a 35 cm do peito com a velocidade dada.
    for (const float delay : { 0.0f, 0.13f, 0.27f }) {
    for (const float speed : { 2.0f, 3.5f, 5.0f }) {
        for (int heading = 0; heading < 360; heading += 45) {
            if (index++ % parts != part) continue;
            auto block = std::make_shared<PhysicsBodyHandle3D>();
            const float a = heading * 3.14159265f / 180;
            const Vec3 from {std::cos(a), std::sin(a), 0};
            run("impulso", speed, heading, Scenario {"varredura", 4.5f, {}, {},
                [=](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
                    if (std::abs(t - 1.5f - delay) >= 0.5f / 120) return;
                    const Vec3 chest = state.links.front().position;
                    *block = dynamicBox(scene, {chest.x + from.x * 0.55f, chest.y + from.y * 0.55f, 1.25f},
                        {0.12f, 0.25f, 0.15f}, 60, from * -speed);
                }});
        }
    }
    }
    std::printf("SWEEPTOTAL casos %d quedas %d tilt medio %.1f\n", count, falls, count ? tiltSum / count : 0.0);
}

// Varredura de arrasto pela PhysGun (XSWEEPPART=k/n divide): 8 direcoes x
// 2 velocidades x peito/pelve. O relato do usuario (arrastado, so ficava
// parado) - um cenario so e caotico demais para decidir variantes.
void dragSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const char* link : { "UpperChest", "Pelvis" }) {
        for (const float speed : { 0.6f, 1.0f }) {
            for (int heading = 0; heading < 360; heading += 45) {
                if (index++ % parts != part) continue;
                const float a = heading * 3.14159265f / 180;
                Scenario scenario {"varredura de arrasto", 5.0f, {}, {}, {}};
                scenario.dragLink = link;
                scenario.dragVelocity = {std::cos(a) * speed, std::sin(a) * speed, 0};
                const ScenarioResult r = runScenario(scenario);
                if (std::getenv("XSTEPS")) report(scenario, r);
                int landed = 0;
                for (const auto& step : r.plannerSteps)
                    if (step.landedAt > 1.0f) ++landed;
                std::printf("DRAG %s %.1f %3d quedas %d tilt %.1f pousados %d andou %.2f\n",
                    link, speed, heading, r.falls, r.worstTilt, landed, r.dragDistance);
            }
        }
    }
}

// Varredura de parado: nascimentos levemente diferentes (inclinacao e
// altura), 6 s parado. Mede passos depois de 1,5 s (quem para quieto da 0) e
// o deslocamento do corpo nesse tempo - um nascimento so e caotico.
// Bancada C (fase 4 do pacote de revisao): parado, transfere o apoio para
// um pe e depois para o outro sem soltar nenhum (XBENCHC ligado aqui).
void transferBench() {
    if (!std::getenv("XBENCHC")) setenv("XBENCHC", "1", 1);
    Scenario scenario {"bancada de transferencia", 7.5f, {}, {}, {}};
    const ScenarioResult r = runScenario(scenario);
    std::printf("BENCHC fim: quedas %d\n", r.falls);
}

void standingSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const float pitch : { -4.0f, -2.0f, 0.0f, 2.0f, 4.0f }) {
        for (const float lift : { 0.0f, 0.02f, 0.05f }) {
            if (index++ % parts != part) continue;
            Scenario scenario {"varredura parado", 6.0f, {}, {}, {}};
            scenario.spawnPitchDegrees = pitch;
            scenario.spawnPelvisHeight = 0.98f + lift;
            const ScenarioResult r = runScenario(scenario);
            if (std::getenv("XSTEPS")) report(scenario, r);
            int late = 0;
            for (const auto& step : r.plannerSteps)
                if (step.releasedAt > 1.5f) ++late;
            std::printf("STAND %+.0f %.2f quedas %d passos %d andou %.3f vz-rms %.3f vz-pico %.3f contato-perdido %d\n", pitch, lift, r.falls, late,
                r.standingTravelAfter15, r.idleVerticalSamples ? std::sqrt(r.idleVerticalSquared / r.idleVerticalSamples) : 0.0,
                r.idleVerticalPeak, r.idleContactLosses);
        }
    }
}

// Terreno: meio-fio, degrau, escadas, rampas e uma caixa baixa (dinamica)
// no caminho, no trote e no sprint (andando em +X desde 0,5 s). Mede quedas,
// a velocidade atravessando (de 1,5 m antes a 1,5 m depois do obstaculo,
// contra a do plano) e a maior inclinacao do peito.
void terrainSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    struct Case { const char* name; float obstacleX; std::function<void(PhysicsScene3D&)> build; };
    const auto ramp = [](float degrees) {
        return [=](PhysicsScene3D& s) {
            const float angle = degrees * 3.14159265f / 180.0f;
            const float half = 2.2f;
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            body.position = {8.0f + half * std::cos(angle), 0, half * std::sin(angle) - 0.1f};
            body.orientation = Quaternion::fromAxisAngle({0, 1, 0}, -angle);
            body.materialId = "concrete";
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = {half, 2, 0.1f};
            shape.materialId = "concrete";
            s.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
            const float top = 2.0f * half * std::sin(angle);
            staticBox(s, {8.0f + 2.0f * half * std::cos(angle) + 10.0f, 0, top * 0.5f}, {10.0f, 2, top * 0.5f});
        };
    };
    const Case cases[] = {
        {"meio-fio 12 cm", 8.2f, [](PhysicsScene3D& s) { staticBox(s, {8.2f, 0, 0.06f}, {0.15f, 2, 0.06f}); }},
        {"meio-fio 25 cm", 8.2f, [](PhysicsScene3D& s) { staticBox(s, {8.2f, 0, 0.125f}, {0.15f, 2, 0.125f}); }},
        {"degrau 15 cm", 8.2f, [](PhysicsScene3D& s) { staticBox(s, {18.2f, 0, 0.075f}, {10.0f, 2, 0.075f}); }},
        {"escada 16 cm", 8.5f, [](PhysicsScene3D& s) {
            for (int i = 0; i < 10; ++i) {
                const float h = (i + 1) * 0.16f;
                staticBox(s, {8.0f + i * 0.32f, 0, h * 0.5f}, {0.16f, 2, h * 0.5f});
            }
            staticBox(s, {21.04f, 0, 0.8f}, {10.0f, 2, 0.8f});
        }},
        {"rampa 10 graus", 9.0f, ramp(10.0f)},
        {"rampa 20 graus", 9.0f, ramp(20.0f)},
        {"caixa 20 cm (80 kg)", 8.2f, [](PhysicsScene3D& s) { dynamicBox(s, {8.2f, 0, 0.1f}, {0.2f, 0.6f, 0.1f}, 80, {}); }},
    };
    int index = 0;
    for (const bool sprint : { false, true }) {
        for (const Case& c : cases) {
            if (index++ % parts != part) continue;
            Scenario scenario {c.name, sprint ? 5.0f : 7.0f, c.build, [=](float t) {
                ScenarioCommand command;
                if (t > 0.5f) command.move = {1, 0, 0};
                command.sprint = sprint;
                return command;
            }, {}};
            const ScenarioResult r = runScenario(scenario);
            // Tempo de (obstaculo - 1,5) a (obstaculo + 1,5) m, e a
            // velocidade no plano antes (de -3,5 a -1,5 m).
            const auto crossing = [&](float from, float to) {
                float start = -1.0f, end = -1.0f;
                for (const auto& [time, x] : r.pelvisX) {
                    if (start < 0.0f && x >= from) start = time;
                    if (end < 0.0f && x >= to) end = time;
                }
                return start >= 0.0f && end > start ? (to - from) / (end - start) : -1.0f;
            };
            const float flat = crossing(c.obstacleX - 3.5f, c.obstacleX - 1.5f);
            const float over = crossing(c.obstacleX - 1.5f, c.obstacleX + 1.5f);
            // A sola no toque: no plano (antes do obstaculo) e no obstaculo
            // (de 0,3 m antes a 2,5 m depois) - media do angulo de frente, do
            // de lado e o pior de frente.
            float pitchFlat = 0, pitchOver = 0, rollOver = 0, worstOver = 0;
            float ankleFlat = 0;
            int countFlat = 0, countOver = 0;
            for (const auto& touch : r.touchdowns) {
                if (touch.x < c.obstacleX - 2.0f && touch.x > c.obstacleX - 5.0f) {
                    pitchFlat += touch.pitch;
                    ankleFlat += touch.ankleError;
                    ++countFlat;
                } else if (touch.x > c.obstacleX - 0.3f && touch.x < c.obstacleX + 2.5f) {
                    pitchOver += touch.pitch;
                    rollOver += std::abs(touch.roll);
                    worstOver = std::max(worstOver, std::abs(touch.pitch - (countFlat ? pitchFlat / countFlat : 0.0f)));
                    ++countOver;
                }
            }
            std::printf("TERRENO %s %s | quedas %d | plano %.2f m/s atravessando %.2f m/s (%.0f%%) | inclinacao %.1f | sola no toque: plano %+.1f (tornozelo alvo-medido %+.1f) obstaculo %+.1f (pior desvio %.1f, lado %.1f, %d toques) raspadas %d\n",
                sprint ? "sprint" : "trote ", c.name, r.falls, flat, over,
                flat > 0.0f && over > 0.0f ? 100.0f * over / flat : -1.0f, r.turnWorstTilt,
                countFlat ? pitchFlat / countFlat : 0.0f, countFlat ? ankleFlat / countFlat : 0.0f,
                countOver ? pitchOver / countOver : 0.0f, worstOver, countOver ? rollOver / countOver : 0.0f, countOver, r.touchdownScuffs);
        }
    }
}

// Escadas (como a do laboratorio: 8 degraus e um patamar): espelho 15/18/20
// cm, piso 28/32 cm, comecando a 1,8-2,45 m, no trote e no sprint. Mede
// quedas, a maior inclinacao e se chegou ao patamar.
void stairsSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const bool sprint : { false, true })
        for (const float rise : { 0.15f, 0.18f, 0.20f })
            for (const float tread : { 0.28f, 0.32f })
                for (const float start : { 1.80f, 2.00f, 2.20f, 2.45f }) {
                    if (index++ % parts != part) continue;
                    const float top = 8.0f * rise;
                    Scenario scenario {"escadas", 9, [=](PhysicsScene3D& s) {
                        for (int i = 0; i < 8; ++i) {
                            const float h = (i + 1) * rise;
                            staticBox(s, {start + (i + 0.5f) * tread, 0, h * 0.5f}, {tread * 0.5f, 2, h * 0.5f});
                        }
                        staticBox(s, {start + 8.0f * tread + 10.0f, 0, top * 0.5f}, {10, 2, top * 0.5f});
                    }, [=](float t) {
                        ScenarioCommand c;
                        if (t > 0.6f && t < 6.0f) c.move = {1, 0, 0};
                        c.sprint = sprint;
                        return c;
                    }, {}};
                    const ScenarioResult r = runScenario(scenario);
                    const bool arrived = r.finalBody.x > start + 8.0f * tread && r.finalBody.z > top + 0.5f;
                    std::printf("ESCADA %s espelho %.0f piso %.0f inicio %.2f | quedas %d | inclinacao %.1f | %s\n",
                        sprint ? "sprint" : "trote ", rise * 100, tread * 100, start, r.falls, r.worstTilt,
                        arrived ? "chegou" : "NAO CHEGOU");
                }
}

// Giro parado: o olhar vira 45-180 graus para cada lado aos 1,5 s. Mede em
// quanto tempo a pelve fisica chega a 20 graus do olhar, os passos, a maior
// inclinacao do peito e as quedas.
void turnSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const float degrees : { 45.0f, 90.0f, 135.0f, 180.0f }) {
        for (const float sign : { 1.0f, -1.0f }) {
            if (index++ % parts != part) continue;
            const float yaw = sign * degrees * 3.14159265f / 180.0f * (degrees > 179.0f ? 0.999f : 1.0f);
            Scenario scenario {"giro parado", 4.5f, {}, [=](float t) {
                ScenarioCommand c;
                c.lookYaw = t > 1.5f ? yaw : 0.0f;
                return c;
            }, {}};
            const ScenarioResult r = runScenario(scenario);
            int steps = 0;
            for (const auto& step : r.plannerSteps)
                if (step.releasedAt > 1.5f) ++steps;
            // Assentou: o ultimo instante em que a pelve estava a mais de 10
            // graus do rumo final (menos 1,5 s); e quanto o rumo final fica
            // do olhar, e quanto ela girou em 0,5 s.
            const auto wrap = [](float a) {
                while (a > 3.14159265f) a -= 6.28318531f;
                while (a < -3.14159265f) a += 6.28318531f;
                return a;
            };
            float settled = -1.0f, half = 0.0f;
            const float startYaw = r.turnYaw.empty() ? 0.0f : r.turnYaw.front().second;
            const float finalYaw = r.turnYaw.empty() ? 0.0f : r.turnYaw.back().second;
            // Girado a partir do rumo aos 1,5 s, no sentido do pedido (para
            // 180 graus, qualquer sentido serve).
            const auto turned = [&](float value) {
                float a = wrap(value - startYaw);
                if (degrees > 179.0f && a * sign < -0.5f) a += sign * 6.28318531f;
                return a * sign;
            };
            for (const auto& [time, value] : r.turnYaw) {
                if (std::abs(wrap(value - finalYaw)) > 0.1745f) settled = time - 1.5f;
                if (time <= 2.0f) half = turned(value);
            }
            std::printf("TURN %+.0f quedas %d assentou %.2f passos %d girou %.0f em-0,5s %.0f\n", sign * degrees, r.falls,
                settled, steps, turned(finalYaw) * 57.2958f, half * 57.2958f);
        }
    }
}

// Varredura de marcha (E5): parte parado (depois de 2 s: o nascimento ainda
// da passos de acomodacao ate ~1,2 s), anda 3 s numa das 8 direcoes
// (olhando para a frente, +x), trote ou sprint, e para. Mede queda,
// velocidade alcancada no rumo pedido, tempo ate 80% dela, desvio de rumo e
// tempo para parar.
void gaitSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const bool sprint : { false, true }) {
        for (int heading = 0; heading < 360; heading += 45) {
            if (index++ % parts != part) continue;
            const float a = heading * 3.14159265f / 180;
            const Vec3 direction {std::cos(a), std::sin(a), 0};
            const bool invert = std::getenv("XGAITINVERT") != nullptr;
            // Camera balancando (XGAITWIGGLE graus, 0,7 Hz) com o movimento
            // relativo a ela (W no laboratorio): curvas para um lado e outro.
            const float wiggle = std::getenv("XGAITWIGGLE")
                ? std::strtof(std::getenv("XGAITWIGGLE"), nullptr) * 3.14159265f / 180.0f : 0.0f;
            Scenario scenario {"varredura de marcha", 7.0f, {}, [=](float t) {
                ScenarioCommand c;
                if (t >= 2.0f && t < 5.0f) c.move = invert && t >= 3.5f ? direction * -1.0f : direction;
                if (wiggle > 0.0f && t >= 2.0f) {
                    c.lookYaw = wiggle * std::sin(2.0f * 3.14159265f * 0.7f * (t - 2.0f));
                    if (t < 5.0f) c.move = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, c.lookYaw).rotate(direction);
                }
                c.sprint = sprint;
                return c;
            }, {}};
            // Empurrao no meio da marcha (XGAITPUSH N, XGAITPUSHDEG em relacao
            // ao corpo, 0,25 s a partir de 3,5 s).
            if (const char* v = std::getenv("XGAITPUSH")) {
                scenario.continuousForceNewtons = std::strtof(v, nullptr);
                scenario.continuousForceDegrees = std::getenv("XGAITPUSHDEG")
                    ? std::strtof(std::getenv("XGAITPUSHDEG"), nullptr) : 90.0f;
                scenario.forceStart = 3.5f;
                scenario.forceSeconds = 0.25f;
            }
            const ScenarioResult r = runScenario(scenario);
            if (std::getenv("XSTEPS")) report(scenario, r);
            const float stop = r.gaitStoppedAt >= 0 && r.gaitMoveEnd >= 0 ? r.gaitStoppedAt - r.gaitMoveEnd : -1;
            const float along = r.gaitSustainedSamples ? static_cast<float>(r.gaitSustainedAlong / r.gaitSustainedSamples) : -1.0f;
            const float across = r.gaitSustainedSamples ? static_cast<float>(r.gaitSustainedAcross / r.gaitSustainedSamples) : 0.0f;
            if (std::getenv("XGAITSTEPS")) {
                // Passos de marcha em regime (soltos entre 3 e 5 s): balanco,
                // intervalo entre solturas, percurso do pe e busca do chao.
                double swing = 0, travel = 0, search = 0, interval = 0;
                int count = 0, intervals = 0, dragged = 0;
                float previousRelease = -1;
                for (const auto& step : r.plannerSteps) {
                    if (step.reason != StepReason3D::Locomotion || step.releasedAt < 3.0f || step.releasedAt > 5.0f
                        || step.landedAt < 0) continue;
                    swing += step.landedAt - step.releasedAt;
                    travel += step.travel;
                    search += std::max(0.0f, step.searchSeconds);
                    dragged += step.dragged ? 1 : 0;
                    ++count;
                    if (previousRelease >= 0) { interval += step.releasedAt - previousRelease; ++intervals; }
                    previousRelease = step.releasedAt;
                }
                if (count > 0)
                    std::printf("PASSADA %s %3d passos %d balanco %.3f s intervalo %.3f s percurso %.2f m busca %.3f s arrastados %d\n",
                        sprint ? "sprint" : "trote", heading, count, swing / count,
                        intervals ? interval / intervals : -1.0, travel / count, search / count, dragged);
            }
            if (std::getenv("XTRANSITION") && r.transitions > 0)
                std::printf("TRANSICAO %s %3d n %d | ganho antes do toque %+.3f | colisao %+.3f | aceitacao %+.3f | saldo ate a saida %+.3f | 0,1 s depois da saida %+.3f m/s\n",
                    sprint ? "sprint" : "trote", heading, r.transitions, r.pushGain / r.transitions,
                    r.collisionLoss / r.transitions, r.acceptanceLoss / r.transitions, r.transitionNet / r.transitions,
                    r.earlySingleLoss / r.transitions);
            if (std::getenv("XSWING") && r.swings > 0)
                std::printf("BALANCO %s %3d n %d | revisao depois de 35%% %.3f | recuo %.3f pior %.3f (>=20 cm: %d) | pico de aceleracao do alvo %.0f pior %.0f m/s2 | rastreio %.3f | erro no pouso %.3f m | voo %d apoio-ficticio %d ticks | soltura fora da ancora %.3f m | revisoes limitadas %u recusadas %u | pe da frente pela agenda %u | estrito: voo %d ficticio apoio %d descarga %d carga %d\n",
                    sprint ? "sprint" : "trote", heading, r.swings, r.swingRevision / r.swings,
                    r.swingRecede / r.swings, r.swingRecedeWorst, r.swingsReceding20,
                    r.swingAccelPeak / r.swings, r.swingAccelWorst, r.swingTracking / r.swings,
                    r.swingLandingError / r.swings, r.flightTicks, r.fictitiousSupportTicks, r.releaseJump / r.swings, r.plan.swingRevisionsBounded, r.plan.swingRevisionsRefused, r.plan.locomotionLeadingReleases,
                    r.strictFlightTicks, r.strictFakeTicks[0], r.strictFakeTicks[1], r.strictFakeTicks[2]);
            if (std::getenv("XSWING")) {
                const auto& why = r.plan.releasesByReason;
                std::printf("APOIO %s %3d ficticio apoio %u aceitando %u saindo %u perdido %u voo-inesperado %u ticks %u eventos"
                    " | solturas carga %u parcial %u geometria %u agenda %u no-ar %u recuperacao %u falha %u\n",
                    sprint ? "sprint" : "trote", heading, r.plan.fakeSupportTicks[0], r.plan.fakeSupportTicks[1],
                    r.plan.fakeSupportTicks[2], r.plan.supportLostEvents, r.plan.unexpectedFlightTicks,
                    r.plan.unexpectedFlightEvents, why[1], why[2], why[3], why[4], why[5], why[6], why[7]);
            }
            std::printf("GAIT %s %3d quedas %d pedido %.2f alcancou %.2f em80 %.2f rumo %.0f parou %.2f regime %.2f lateral %+.2f fase %d inverteu %.2f soltos-carregados %u carga-por-prazo %u parcial %u prazo %u carga-falhou %u ref-corporal %u pouso-previsto %u inclinacao %.1f afastamento %.3f\n",
                sprint ? "sprint" : "trote", heading, r.falls, r.gaitRequestedSpeed, r.gaitBestSpeed,
                r.gaitTimeTo80, r.gaitWorstYawDegrees, stop, along, across, r.gaitFallPhase,
                r.gaitReverseAt >= 0 && r.gaitReversedAt >= 0 ? r.gaitReversedAt - r.gaitReverseAt : -1.0f,
                r.plan.locomotionForcedReleases, r.plan.loadingTimeouts,
                r.plan.locomotionPartialReleases, r.plan.locomotionTimeoutReleases, r.plan.loadAcceptanceFailures,
                r.bodyReferenceTicks, r.plan.placementPredictorCalls, r.worstTilt, r.worstBodyCapsule);
        }
    }
}

// Regressao da arrancada física. O FBX Idle To Sprint chegou a comandar a
// raiz/pernas inteiras também no trote: peito/pelve passavam de 45 graus, os
// dois pés perdiam o apoio e o corpo caía antes de alcançar a velocidade.
void startupGaitChecks() {
    setenv("XE5ANIM", "1", 1);
    if (std::getenv("XWALKASSIST") == nullptr)
        setenv("XWALKASSIST", "1", 1);
    for (const bool sprint : { false, true }) {
        Scenario scenario { sprint ? "arrancada sprint" : "arrancada trote",
            5.5f, {}, [=](float t) {
                ScenarioCommand command;
                if (t >= 2.0f && t < 5.0f) command.move = {1, 0, 0};
                command.sprint = sprint;
                return command;
            }, {}};
        const ScenarioResult result = runScenario(scenario);
        std::printf("START %s quedas %d tilt %.1f velocidade %.2f t80 %.2f\n",
            sprint ? "sprint" : "trote", result.falls, result.worstTilt,
            result.gaitBestSpeed, result.gaitTimeTo80);
        require(result.falls == 0,
            "A arrancada derrubou o personagem");
        require(result.worstTilt < (sprint ? 28.0f : 20.0f),
            "A arrancada voltou a dobrar o corpo para a frente");
        require(result.gaitTimeTo80 >= (sprint ? 0.24f : 0.10f)
                && result.gaitTimeTo80 < (sprint ? 0.90f : 0.70f),
            "A arrancada ficou instantanea ou lenta demais");
    }
}

// Varredura de forca continua (a meta do usuario: firme ate ~500 N). Tempo
// ate cair, ate 12 s de forca.
void continuousForceSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const float force : { 80.0f, 150.0f, 200.0f, 300.0f, 400.0f, 500.0f }) {
        for (const float degrees : { 0.0f, 90.0f, 180.0f, 270.0f }) {
            if (index++ % parts != part) continue;
            Scenario scenario {"forca continua", 13.0f, {}, {}, {}};
            scenario.continuousForceNewtons = force;
            scenario.continuousForceDegrees = degrees;
            const ScenarioResult r = runScenario(scenario);
            if (std::getenv("XSTEPS")) report(scenario, r);
            const float held = r.firstFallSeconds >= 0 ? r.firstFallSeconds - 1.0f : 12.0f;
            std::printf("FORCE %.0f %3.0f aguentou %.1f s tilt %.1f passos %zu\n", force, degrees, held,
                r.worstTilt, r.plannerSteps.size());
        }
    }
}

int main() {
    try {
        if (const char* only = std::getenv("XSPEED")) { baseline(std::strtof(only, nullptr)); return 0; }
        const char* filter = std::getenv("MATTERENGINE_TEST_FILTER");
        const std::string which = filter ? filter : "";
        if (which.empty() || which == "baseline") { baseline(0); baseline(3); baseline(7.5f); }
        if (which.empty() || which == "game") gameLoopBaselines();
        if (which.empty() || which == "polish") polishScenarios();
        if (which.empty() || which == "agility") agilityScenarios();
        if (which.empty() || which == "lean") leanScenarios();
        if (which.empty() || which == "disturb") disturbanceScenarios();
        if (which.empty() || which == "push") pushScenarios();
        if (which.empty() || which == "pressure") pressureScenarios();
        if (which.empty() || which == "npc") npcGetUpScenarios();
        if (which.empty() || which == "estado") stateEstimatorChecks();
        if (which.empty() || which == "passos") plannerStepChecks();
        if (which.empty() || which == "corpo") wholeBodyChecks();
        if (which.empty() || which == "assist") standingAssistSemanticsChecks();
        if (which == "transmissao") postureTransmissionChecks();
        if (which == "sweepdrag") dragSweep();
        if (which == "sweepstand") standingSweep();
        if (which == "sweepturn") turnSweep();
        if (which == "sweepterrain") terrainSweep();
        if (which == "sweepstairs") stairsSweep();
        if (which == "sweepforce") continuousForceSweep();
        if (which == "sweepgait") gaitSweep();
        if (which.empty() || which == "startup") startupGaitChecks();
        if (which == "benchtransfer") transferBench();
        if (which == "sweep") pressureSweep();
        if (which.empty() || which == "fall") fallScenarios();
        return 0; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
