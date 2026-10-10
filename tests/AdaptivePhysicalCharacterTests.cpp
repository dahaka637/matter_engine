#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Character/BallOwnership3D.hpp"
#include "Engine/Character/CharacterControlApplication3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/BallTouchPreview3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"
#include "Engine/Character/CharacterFootGeometry3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Control/ImpactAssist3D.hpp"
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
#include <map>
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
        character.standUpBackClipId,
        // XGUCLIP: outro clipe de levantar de bruços (comparacao).
        std::getenv("XGUCLIP") ? std::string(std::getenv("XGUCLIP")) : character.standUpFrontClipId,
        character.idleToSprintClipId, character.runForwardArcLeftClipId,
        character.runForwardArcRightClipId, character.runBackwardArcRightClipId,
        character.jumpPrepClipId, character.turnLeftClipId, character.turnRightClipId};
    // Carrinho: os dois (XCARRINHO=k deixa so o k).
    std::vector<std::string> tackles = character.tackleClipIds;
    if (const char* only = std::getenv("XCARRINHO")) {
        const std::size_t k = static_cast<std::size_t>(std::atoi(only));
        if (k < tackles.size()) tackles = { tackles[k] };
    }
    set.storage.reserve(std::size(ids) + tackles.size());
    for (const auto& id : ids)
        set.storage.push_back(id.empty() ? AnimationClip3D{}
            : loadAnimationClip3D(assets + "/animations/clips/" + id + ".matteranim.json"));
    for (const auto& id : tackles)
        set.storage.push_back(loadAnimationClip3D(assets + "/animations/clips/" + id + ".matteranim.json"));
    auto at = [&](std::size_t i) { return set.storage[i].id.empty() ? nullptr : &set.storage[i]; };
    auto& a = set.animations;
    a.idle = at(0); a.walk = at(1); a.walkBackward = at(2); a.sprint = at(3);
    a.sprintBackward = at(4); a.strafeLeft = at(5); a.strafeRight = at(6);
    a.sprintStrafeLeft = at(7); a.sprintStrafeRight = at(8); a.jumpStanding = at(9);
    a.jumpForward = at(10); a.jumpBackward = at(11); a.jumpLeft = at(12); a.jumpRight = at(13);
    a.standUpBack = at(14); a.standUpFront = at(15);
    a.idleToSprint = at(16); a.runForwardArcLeft = at(17);
    a.runForwardArcRight = at(18); a.runBackwardArcRight = at(19);
    a.jumpPrep = at(20);
    // XSEMGIROCLIPE: sem os clipes de giro parado (comparacao).
    if (!std::getenv("XSEMGIROCLIPE")) { a.turnLeft = at(21); a.turnRight = at(22); }
    for (std::size_t k = 0; k < tackles.size() && k < a.tackles.size(); ++k)
        a.tackles[k] = at(std::size(ids) + k);
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
    bool crouch = false;  // agachar segurado (correndo: o carrinho)
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
    // Rumo (graus) para onde a inclinacao do nascimento tomba (0 = frente).
    float spawnTiltHeadingDegrees = 0;
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
    // Resposta ao comando: a partir de probeStart, quando o centro de massa
    // passa a ir probeSpeed m/s em probeDirection (retomar no meio da freada).
    float probeStart = -1.0f;
    Vec3 probeDirection {};
    float probeSpeed = 1.5f;
    // Curva (sweepcurve): janela em que mede inclinacao, pelve e pes.
    float curveFrom = -1.0f;
    float curveTo = -1.0f;
    // Outro boneco nascendo deitado: inclinacao (graus, + de bruços, em torno
    // do lado dele), rolagem (de lado) e altura da pelve.
    // Ida do padrao ao fisico (sweepida): ate defaultUntil o corpo anda
    // cinematico pelo modo padrao sobre a capsula; ai fica dinamico e a
    // locomocao fisica assume (com a semente do corpo, sem XIDASEM).
    // Outro boneco com as ajudas do laboratorio; levando carrinho do jogador,
    // a ajuda dele vai a npcTackledPercent (< 0: nao muda) e volta em 2,5 s.
    bool npcLabAssist = false;
    float npcTackledPercent = -1.0f;
    float defaultUntil = -1.0f;
    const CharacterLocomotionAnimations3D* defaultAnimations = nullptr;
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
    // Transicoes (sweeptransicao): mede pes cruzados/embolados, saltinhos e a
    // resposta a cada troca de comando.
    bool transitionMetrics = false;
    // Curva fechada (sweepcurvafechada): a virada vai de turnStart a turnEnd
    // e termina no rumo turnDirection.
    float turnStart = -1.0f;
    float turnEnd = -1.0f;
    Vec3 turnDirection {};
    // Tropeco correndo (sweeptropeco): o obstaculo em x = stumbleX; mede a
    // partir de o centro de massa chegar 0,6 m antes dele.
    float stumbleX = -1.0f;
    // Manejo de bola (sweepmanejo): a bola (criada no build) e o roteiro dos
    // pedidos (devolve o tipo no tick do pedido; forca e direcao no mundo).
    std::shared_ptr<InflatedBallHandle3D> ball;
    // (Recebe a bola, a pelve e se o manejo esta livre: o jogador do teste
    // toca quando a bola volta para perto.)
    std::function<BallActionKind3D(float t, Vec3 ball, Vec3 pelvis, bool idle, float& power, Vec3& direction)> ballScript;
    // Gesto do chute (-1..1): altura e efeito.
    float shotLift = 0.0f, shotCurve = 0.0f;
    // Chute correndo (sweepchutecorre): em ballPlaceAt a bola vai para
    // ballPlaceOffset (no referencial do corpo: x a frente, y a esquerda) e o
    // toque e pelo pe.
    float ballPlaceAt = -1.0f;
    Vec3 ballPlaceOffset;
    Vec3 ballPlaceVelocity;
    // Estresse: a bola volta para perto do corpo a cada isto (s; 0 nunca).
    float ballReplaceEvery = 0.0f;
    int ballReplaceSeed = 0;
    // A bola colide com o corpo como no laboratorio: pe e canela deixam de
    // colidir no primeiro contato ou com acao e a bola a 0,4 m deles.
    bool ballCollides = false;
    // Repulsa: o corpo ja liberado desde o comeco; a bola posta nesta altura
    // (m, centro; <= 0 a do chao); e uma sonda por tick (bola e corpo).
    bool forcePassThrough = false;
    float ballPlaceHeight = -1.0f;
    std::function<void(float, Vec3, Vec3, const RagdollState3D&)> ballProbe;
    bool footTouch = false;
    // Arrancada: o comando de movimento comeca aqui (mede raspadas e o
    // tempo ate 80% a partir dele).
    float startAt = -1.0f;
    // Pe enganchado (sweeptropeco, modo puxao): a partir de snagAt, o pe que
    // estiver no ar (10-60% do balanco) e puxado para tras (contra o rumo)
    // com snagNewtons por snagSeconds - a ponta enganchando no chao.
    float snagAt = -1.0f;
    float snagNewtons = 0.0f;
    float snagSeconds = 0.08f;
    // Postura (sweeppostura): janela em que mede joelhos e altura da pelve.
    float postureFrom = -1.0f;
    float postureTo = -1.0f;
    // Fluidez (sweepfluidez): janela em que o corpo e comparado com a pose
    // da animacao, e a resposta a um comando (aos responseAt s o pedido vira
    // responseDirection na velocidade de trote/sprint, ou parar).
    // Giro parado: a partir de quando medir a torcao das pernas.
    float legTwistFrom = -1.0f;
    float fluidFrom = -1.0f;
    float fluidTo = -1.0f;
    float responseAt = -1.0f;
    Vec3 responseDirection;
    bool responseSprint = false;
    bool responseStop = false;
    // Pula (um toque) quando a capsula passa deste x (m; negativo: nunca).
    float jumpWhenX = -1.0f;
    // A bola posta (ballPlaceAt) ja com o jogador como dono.
    bool ballClaimAtPlace = false;
    // Velocidade media da pelve no rumo pedido entre estes instantes (s; <0
    // sem) - ScenarioResult::windowSpeedSum.
    float speedFrom = -1.0f, speedTo = -1.0f;
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
    // Fluidez: inclinacao do tronco e da pelve contra a pose da animacao
    // (graus, eixo de cima), giro da pelve, erro medio de todos os links,
    // pelve contra a raiz alvo (cm, no plano e na altura), tropecos (ticks
    // com recuperacao de tropeco ou risco alto) e o tempo de resposta (s).
    double fluTrunkSq = 0, fluPelvisSq = 0, fluYawSq = 0, fluAllSq = 0, fluPosHSq = 0, fluPosVSq = 0, fluHeadSq = 0;
    float fluTrunkMax = 0, fluPelvisMax = 0, fluYawMax = 0, fluAllMax = 0, fluPosHMax = 0, fluPosVMax = 0, fluHeadMax = 0;
    int fluSamples = 0, fluStumbleTicks = 0;
    double fluPosVSigned = 0;
    // Deslize de verdade: o pe no ponto de contato com o chao (velocidade do
    // corpo do pe naquele ponto, no plano; rolando e zero), integrado (m), e
    // o pior instante (m/s). Do inicio da janela de fluidez ou da arrancada.
    float slipMeters = 0, slipWorst = 0;
    // Torcao das pernas (graus): canela contra o pe e contra a pelve (rumo no
    // plano), o joelho apontando para dentro do corpo e a pelve atras do olhar.
    float legTwistFoot = 0, legTwistPelvis = 0, kneeInward = 0, pelvisLookLag = 0;
    // Pernas cruzadas (o esquerdo a direita do direito, no lado da pelve):
    // ticks e a pior invasao (cm); afundo: a maior distancia entre os
    // tornozelos (m).
    int crossTicks = 0;
    float crossWorst = 0, ankleSpreadWorst = 0;
    // Giro parado (sweepgiro): o quanto os pes apoiados esfregam no chao
    // (m, somado), a menor distancia entre os pes e os ticks com eles colados
    // (< 14 cm: embananado), a maior inclinacao da pelve (graus) e o ultimo
    // instante sem estar assentado (pelve a mais de 5 graus do olhar ou um pe
    // fora do chao).
    float footScrub = 0, feetMinDistance = 9, tiltWorst = 0, lastUnsettled = -1;
    int closeTicks = 0;
    // Pes em linha ou cruzados pelo rumo da BASE (a media dos rumos dos
    // dois pes - pela pelve, girando rapido, a regua mentia): ticks com o pe
    // esquerdo a menos de 8 cm a esquerda do direito, e o pior (cm, negativo
    // = cruzado).
    int baseTangleTicks = 0;
    float baseSeparationWorst = 9;
    std::array<Vec3, 2> previousPlantedFoot {};
    std::array<bool, 2> previousPlanted {};
    // (Filtrada: parado o pe vibra a ~60 Hz, <1 mm, e a velocidade crua no
    // contato trocava de sinal a cada tick - somava 0,9 m de "deslize" parado.)
    std::array<Vec3, 2> slipFiltered {};
    std::array<std::array<Vec3, 12>, 2> slipWindow {};
    std::array<int, 2> slipCount {};
    // Impacto: aceleracao da pelve menos a da raiz alvo (m/s2), rms e pico.
    double fluImpactSq = 0;
    float fluImpactMax = 0;
    Vec3 fluPrevPelvisVelocity, fluPrevTargetVelocity, fluPrevTargetPosition;
    bool fluPrevValid = false;
    float fluResponse = -1;
    int falls = 0;
    float firstFallSeconds = -1;
    // Postura perdida (pelve a mais de 25 graus da vertical, sem estar caido
    // nem no ar): o tempo somado ("pendurado" pela ajuda) e o maior episodio,
    // da perda ate voltar abaixo de 15 graus ou ir ao chao (s).
    float lostPostureSeconds = 0;
    float worstChestTilt = 0;   // graus, peito contra a vertical (de pe, fora do chao)
    // Ida ao fisico: o alvo dos links das pernas contra o corpo no primeiro
    // tick (graus, media), o pico de giro do alvo e do corpo nas pernas
    // (rad/s, 0,4 s depois; o do corpo tambem 0,4 s antes), a velocidade
    // antes e a menor em 0,6 s depois.
    float switchTargetGap = -1, switchTargetSpin = 0, switchBodySpin = 0, switchBodySpinBefore = 0;
    float switchSpeedBefore = 0, switchSpeedMin = 1e9f;
    // Giro da pelve (yaw) do comeco ao fim do voo (graus, o maior desvio).
    float airYawDrift = 0, airYawStart = 1e9f;
    float npcTackledAt = -1;
    // Velocidade media da pelve no rumo pedido (Scenario::speedFrom..speedTo).
    double windowSpeedSum = 0;
    int windowSamples = 0;
    // O menor nivel da ajuda depois de um impacto (ImpactAssist3D).
    float npcAssistLow = 1.0f, playerAssistLow = 1.0f;
    float defaultTackleAt = -1;
    // Corpo guiado no modo padrao (XGUIADO): o maior erro dos links contra a
    // pose no fim do passo (antes e depois da correcao), o maior pico de
    // aceleracao de um link na fase padrao (m/s2, pelas velocidades) e
    // quando a pancada (XGOLPE) levou ao fisico.
    float guideErrorPeak = 0, guidePoseErrorPeak = 0, defaultLinkAccelPeak = 0;
    float hitSwitchAt = -1;
    bool hitSwitchWall = false;
    // Boletim v2 (sweepboletim), de pe e fora da janela de um pulo (1,3 s):
    // voo sem pulo (s sem pe apoiado com a capsula no chao) - o maior, os que
    // passam de 0,14 s (saltinhos) e o tempo alem de 0,25 s (pendurado); a
    // pelve acima da altura de pe (m); e a ajuda na pelve (|forca| / m.g,
    // media; e so no plano).
    float bolFlightPeak = 0, bolSuspendedSeconds = 0, bolPelvisRisePeak = -1;
    int bolHops = 0;
    float bolFlightSeconds = 0, bolLastJumpAt = -10, bolFlightTakeoffSpeed = 0, bolFlightTakeoffHeight = 0, bolFlightApex = 0;
    // Velocidade do corpo no rumo pedido contra a pedida (so com pedido).
    double bolSpeedSum = 0, bolRequestedSum = 0;
    double bolAssistSum = 0, bolAssistPlanarSum = 0;
    int bolAssistSamples = 0;
    // Ticks com o jogador dono da bola e as trocas de dono (tempo, dono).
    int ballOwnedTicks = 0;
    std::vector<std::pair<float, std::uint64_t>> ballOwnerLog;
    // Cambaleio: o maior.
    float staggerPeak = 0;
    // Voadora (fisica ja no pedido): quando, a velocidade da capsula no pulo
    // e a da pelve na troca, no toque no chao (pelve abaixo de 0,55 m) e o
    // deslize dali com a pelve baixa (m, no plano).
    float voadoraAt = -1, voadoraSpeedTakeoff = 0, voadoraSpeedSwitch = 0, voadoraSpeedLanding = -1;
    float voadoraSlideMeters = 0;
    Vec3 voadoraLandingAt {};
    bool voadoraSlideDone = false;
    // O outro boneco: a maior velocidade da pelve dele (m/s).
    float npcPelvisSpeedPeak = 0;
    float risingSeconds = 0, hunchedSeconds = 0, risingEndAt = -1, risingPelvisPeak = 0;
    float getUpAirSeconds = 0, risingStartedAt = -1, risingPelvisRisePeak = 0, risingLinkSpeedPeak = 0;
    // O corpo no primeiro tick em que o levantar com pressa pode passar ao
    // modo padrao (getUpHandoffReady) e quando.
    RagdollState3D handoffState;
    float handoffAt = -1;
    // Pendulo: centro de massa contra o meio dos pes (graus), de pe e fora
    // do ar - o corpo inteiro fora dos pes, segurado pela ajuda.
    float worstPendulum = 0, pendulumHangSeconds = 0, longestPendulumEpisode = 0, pendulumEpisodeStart = -1;
    float longestLostEpisode = 0;
    float lostEpisodeStart = -1;
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
    // Queda: quando comecou (pelve a 25 graus) e o primeiro toque no chao das
    // maos/antebracos, da cabeca e do tronco (pelve, abdomen, peito), s.
    float fallOnsetAt = -1, handGroundAt = -1, headGroundAt = -1, torsoGroundAt = -1;
    float handGroundHeadHeight = -1;  // altura da cabeca quando a mao tocou
    float handGroundElbow = -1;       // flexao do cotovelo (graus) da mao que tocou primeiro
    std::string firstGroundLink;    // primeiro segmento (fora os pes) a tocar o chao caindo
    float getUpSeconds = -1;        // da queda ate estar em pe de novo
    int getUpAttemptsMax = 0;       // tentativas de levantar (a 1a conta)
    float risingFirstAt = -1;       // inicio da primeira subida
    // Carrinho (sweepcarrinho): quando comecou e terminou, a variacao, a
    // velocidade de entrada, quanto a pelve andou no carrinho, o maior desvio
    // de lado da direcao, a pelve mais baixa, o membro mais rapido e a
    // velocidade 0,5 s depois.
    float tackleStartedAt = -1, tackleEndedAt = -1, tackleEntrySpeed = 0;
    float tackleTravel = 0, tackleLateral = 0, tackleMinPelvis = 9, tackleWorstLinkSpeed = 0;
    float afterTackleSpeed = -1;
    // No chao (pelve abaixo de 0,35 m no carrinho): tempo e distancia, e a
    // velocidade ao tocar o chao.
    float slideGroundSeconds = 0, slideGroundDistance = 0, slideLandSpeed = -1;
    // Manejo de bola: pedidos, toques, os que nao tocaram (bola longe; o pe
    // nao chegou), a distancia da bola a pelve (maior e media com o corpo
    // andando), e o chute: a velocidade da bola e o erro de rumo (graus).
    int ballRequests = 0, ballTouches = 0, ballMissFar = 0, ballMissLate = 0;
    // Contatos do corpo com a bola que nao foram toques pedidos (a passada
    // chutando a bola sem querer).
    int ballAccidental = 0;
    bool ballContactPrevious = false;
    float ballTouchAt = -1;
    float ballMaxDistance = 0, ballDistanceSum = 0, shotSpeed = -1, shotYawError = 0, shotElevation = 0;
    int ballDistanceSamples = 0;
    float shotAt = -1;
    // Chute: onde estava o pe de apoio no toque (em relacao a bola, na mira:
    // a frente / de lado), do pedido ao toque (s), giro, altura maxima da
    // bola e o desvio lateral dela ao passar 20 m (ou no fim).
    float shotRequestAt = -1, shotPlantAlong = 0, shotPlantAcross = 0, shotDelay = -1, shotSpin = 0;
    float shotApex = 0, shotLateral20 = 0, shotTravel = 0;
    int shotKickSide = -1;
    bool shotLateralDone = false;
    Vec3 shotStart;
    std::array<int, 5> ballSurfaces {};
    // O maior reforco de estabilidade do gesto que valeu (V3).
    float ballStabilityPeak = 0.0f;
    Vec3 slidePrevious {};
    bool slideOnGround = false;
    float tackleArmedAt = -1;
    int tackleVariant = -1;
    Vec3 tackleStartPelvis;
    float risingStartAt = -1;       // quando comecou o levantar (clipe), s
    // Coluna (de pe, antes de cair): a maior dobra entre pelve e peito
    // superior e as maiores inclinacoes da pelve e do peito (graus).
    float spineBendPeak = 0, pelvisTiltPeak = 0, chestTiltPeak = 0;
    // A mesma dobra parado antes do empurrao (1,4 s): o pico menos ela e o
    // quanto a coluna se mexeu.
    float spineBendRest = -1, chestTiltRest = -1, pelvisTiltRest = -1;
    // Correndo em regime: a contra-rotacao (giro da linha dos ombros menos o
    // da linha dos quadris) e a inclinacao lateral entre elas, min e max
    // (graus) - a coluna trabalhando entre pelve e peito.
    float trunkYawMin = 1e9f, trunkYawMax = -1e9f, trunkRollMin = 1e9f, trunkRollMax = -1e9f;
    // Pouso (sweeppouso): o primeiro toque depois de um voo - quando, a
    // velocidade do centro de massa no fim do voo, o angulo da perna (do
    // centro de massa ao pe mais baixo) e o da velocidade a partir da
    // vertical (graus, + a frente no rumo horizontal do movimento) - e, no
    // 1 s seguinte, o quanto o peito tomba para a frente e para tras.
    float landingAt = -1; Vec3 landingVelocity {}; Vec3 airborneVelocity {};
    float landingLegAngle = 0, landingVelocityAngle = 0;
    float landingChestForward = -90, landingChestBack = 90, landingChestStart = 0;
    float landingSpeedAfter = -1;
    bool wasAirborne = false;
    // Serie por tick (sweeppulofrente): inclinacao do peito a frente (graus,
    // no rumo da velocidade), torque de postura da ajuda que endireita para
    // tras (N m, + endireitando) e velocidade.
    std::vector<float> seriesT, seriesChestPitch, seriesUprightTorque, seriesSpeed, seriesPelvisPitch;
    // Vizinho (sweepvizinho): quantas vezes a ajuda viu "batida externa", o
    // tempo perturbado, o menor envelope da ajuda, o maior impulso e em quem.
    int neighborHits = 0; float neighborPerturbedSeconds = 0, neighborEnvelopeMin = 1, neighborImpulseMax = 0;
    std::map<std::string, int> neighborHitLinks;
    bool neighborWasHit = false;
    // No chao (deitado, antes do levantar): agitacao dos membros - a
    // velocidade media e o pico dos alvos de quadril/joelho/ombro/cotovelo
    // (graus/s) - e quantas vezes o "se arrumar" voltou atras.
    double groundTargetSpeedSum = 0; int groundTargetSamples = 0; float groundTargetSpeedPeak = 0;
    double groundLimbSpeedSum = 0;
    int gatherReversals = 0; float lastGather = 0; bool gatherRising = false;
    double airTargetSpeedSum = 0, airLimbSpeedSum = 0; int airSamples = 0;
    std::vector<float> previousTargets;
    float getUpWorstRootSpeed = 0;  // pelve, no levantar
    float getUpWorstRootStep = 0;   // maior salto da pelve num tick (teleporte)
    float getUpWorstLinkSpeed = 0;
    float getUpAssistPeakShare = 0; // forca auxiliar de pico / m.g
    float getUpAssistMeanShare = 0;
    int relapses = 0;               // quedas depois de levantar
    int bodilyMaxStage = 0;         // levantar pelo corpo: maior etapa alcancada
    // Postura: joelho medido e pedido (graus) no apoio e no total, altura da
    // pelve (m).
    double postureKneeStanceSum = 0, postureKneeStanceTargetSum = 0, postureKneeAllSum = 0;
    double postureKneeTargetAllSum = 0, posturePelvisSum = 0;
    float postureKneeStanceMin = 1e9f, postureKneeStanceMax = 0;
    double postureGuideSum = 0;
    double postureStiffnessSum = 0, postureTorqueScaleSum = 0, postureMotorSum = 0, postureLimitSum = 0;
    int postureSaturated = 0;
    int postureStanceSamples = 0, postureSamples = 0;
    // Joelho no toque (o primeiro tick apoiado de cada pe) e as descidas da
    // pelve alvo por regra (m, medias).
    double postureTouchKneeSum = 0, postureTouchPelvisSum = 0, posturePelvisSquares = 0;
    double postureLiftKneeSum = 0;
    int postureLifts = 0;
    int postureTouches = 0;
    bool postureWasSupporting[2] {};
    double postureDropCurve = 0, postureDropBrake = 0, postureDropReach = 0, postureDropLanding = 0;
    double postureSupport = 0, postureUpright = 0, postureEnvelope = 0, postureLegShare = 0;
    // Passos (cada apoio fisico de um pe): o que ele fez com o centro de
    // massa (delta de velocidade vertical e ao longo do movimento, do toque a
    // saida), o amortecimento (do toque ao joelho mais dobrado: tempo e
    // graus) e o empurrao (dali a saida). Voos: altura da pelve no topo.
    struct StepTrack {
        bool on = false;
        float touchTime = 0, kneeAtTouch = 0, maxKnee = 0, maxKneeTime = 0;
        Vec3 velocityAtTouch;
        float aheadAtTouch = 0, hipAtTouch = 0, comHeightAtTouch = 0;
    } postureStep[2];
    int postureSteps = 0;
    double postureStepDvz = 0, postureStepDvh = 0, postureStepSeconds = 0;
    double postureAbsorbSeconds = 0, postureAbsorbDegrees = 0, posturePushSeconds = 0;
    std::vector<float> postureStepDvzList, postureFlightApex;
    // Suavidade: aceleracao angular (rad/s2, RMS) dos alvos e das juntas
    // medidas - braco (ombro, cotovelo) e perna (quadril, joelho) - e o
    // quanto o ritmo do ciclo varia de um tick para o outro.
    struct Smooth { float previous[2] {}; float velocity[2] {}; bool primed = 0; int count = 0; } postureSmoothState[8];
    double postureArmTargetAcc = 0, postureArmMeasuredAcc = 0, postureLegTargetAcc = 0, postureLegMeasuredAcc = 0;
    int postureArmSamples = 0, postureLegSamples = 0;
    float posturePreviousPhase = -1.0f, posturePreviousPhaseRate = 0.0f;
    double posturePhaseRateChange = 0;
    int posturePhaseSamples = 0;
    bool postureFlying = false;
    float postureFlightTop = 0;
    // Apoios do clipe (contato 0 -> 1 -> 0) por pe: quantos, em quantos o pe
    // fisico nao chegou a apoiar, o atraso do toque fisico depois do inicio
    // do apoio do clipe e quanto do apoio do clipe sobrou com o pe no chao.
    bool postureClipStance[2] {}, postureStanceTouched[2] {};
    float postureClipStanceStart[2] {};
    int postureClipStances = 0, postureMissedStances = 0, postureLateTouches = 0;
    double postureTouchDelaySum = 0, postureTouchedShareSum = 0;
    float postureTouchedSeconds[2] {};
    // Pelo travamento do pe (o que o corpo faz de fato): soltos sem ter
    // apoiado desde que travaram, e o apoio fisico de cada passo.
    bool posturePlanted[2] {}, posturePlantSupported[2] {};
    int posturePlants = 0, posturePlantsMissed = 0;
    double posturePlantSupportSum = 0;
    float posturePlantSupport[2] {};
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
    // No mesmo regime: o giro medio do corpo contra a camera (graus, com
    // sinal: + para a esquerda) - andando de lado ele ia girando.
    double gaitSignedYawSum = 0;
    // Peito no regime (graus): arfagem (+ para a frente) e rolagem (+ para a
    // esquerda) do fisico e do alvo - soma e soma dos quadrados - e a
    // velocidade angular (rad/s, quadrado) do peito fisico e do alvo.
    double chestPitch[2] = {0, 0}, chestPitchSq[2] = {0, 0}, chestRoll[2] = {0, 0}, chestRollSq[2] = {0, 0};
    double chestSpinSq[2] = {0, 0};
    double chestSpinAxisSq[2][3] = {{0, 0, 0}, {0, 0, 0}};  // rolagem, arfagem, guinada (no rumo)
    // Sentir as pernas: altura da pelve e velocidade no rumo (soma, quadrados).
    double pelvisZ = 0, pelvisZSq = 0, alongV = 0, alongVSq = 0;
    // Deslize: velocidade tangencial dos contatos dos pes com o chao (m/s).
    double footSlip = 0; int footSlipSamples = 0, footSlipOver = 0;
    // Transicoes: ticks com os pes cruzados (o esquerdo a direita do direito,
    // no referencial da pelve, fora do strafe) e embolados (a menos de 13 cm
    // um do outro, os dois baixos); saltinhos (o centro de massa subindo a
    // mais de 0,7 m/s sem pulo pedido); e a resposta a cada troca (s ate o
    // corpo ir a 60% do pedido no rumo novo; 2 s = nao chegou).
    int transCrossTicks = 0, transTouchTicks = 0, transHops = 0, transChanges = 0;
    float transHopArmed = 1.0f, transPeakUp = 0.0f, transSteadyUp = 0.0f, transWindowUntil = -1.0f;
    double transResponseSum = 0; float transResponseWorst = 0;
    float transChangeAt = -1; Vec3 transRequest {}; Vec3 transPreviousRequest {};
    // Deslize medido pelo caminho: quanto a sola anda no chao, no plano,
    // enquanto o pe apoia (do toque a saida), por apoio.
    std::array<bool, 2> slideSupported {};
    std::array<Vec3, 2> slideStart {}, slideLast {};
    std::array<float, 2> slideSeconds {};
    std::array<Vec3, 2> slideStartVelocity {};
    std::array<float, 2> slideStartLift {};
    double slideDistance = 0, slideTime = 0; int slideStances = 0;
    int chestSamples = 0;
    Quaternion previousChest[2] {};
    bool previousChestValid = false;
    int gaitFallPhase = 0;
    // Inversao (XGAITINVERT): quando o pedido virou de sentido e quando o
    // corpo passou a ir a metade da nova velocidade no novo rumo.
    float gaitReverseAt = -1, gaitReversedAt = -1;
    float probeReachedAt = -1;
    // Curva: medias na janela (graus; lado + = para a esquerda do rumo).
    int curveSamples = 0;
    double curveSpeed = 0, curveAcceleration = 0, curveTrunkSide = 0, curveTrunkForward = 0,
        curvePelvisLead = 0, curveLag = 0, curveRequestedSpeed = 0;
    std::array<int, 2> curveStance {};
    std::array<double, 2> curveFootYaw {}, curveLeg {}, curveHipHeight {}, curveBodyLean {};
    float curvePreviousHeading = 0.0f;
    bool curveHeadingValid = false;
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
    // Erro de pouso (XPOUSO): onde o pe tocou contra o alvo do IK logo antes
    // (m, no plano; com sinal no rumo pedido e atravessado, + a esquerda).
    double touchErrorSum = 0, touchErrorAlong = 0, touchErrorAcross = 0;
    int touchErrorCount = 0;
    std::array<Vec3, 2> lastSwingTarget {};
    std::array<bool, 2> lastSwingTargetValid {};
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
    // Curva fechada: velocidade do centro de massa no comeco da virada e a
    // menor ate 0,8 s depois dela; quando ele vai a 60% do pedido no rumo
    // final (s desde o comeco); o giro de cada pe no chao durante o apoio
    // (pivo, graus: soma, pior, apoios acima de 30); o deslize da sola
    // nesses apoios (m); ticks de freada e de pes cruzados na janela.
    float tcSpeedBefore = -1, tcMinSpeed = 1e9f, tcResponse = -1;
    std::array<bool, 2> tcSupported {};
    std::array<float, 2> tcStartYaw {}, tcPivotPeak {};
    std::array<Vec3, 2> tcStartSole {};
    std::array<float, 2> tcSoleTravel {};
    double tcPivotSum = 0, tcSlideSum = 0;
    float tcPivotWorst = 0;
    int tcStances = 0, tcPivotOver30 = 0, tcBrakeTicks = 0, tcCrossTicks = 0;
    // Tropeco: quando o corpo chegou ao obstaculo, a velocidade ali, a menor
    // no 1,5 s seguinte e a de 2 s depois; a maior inclinacao da pelve para a
    // frente (graus) e a pelve mais baixa (m).
    float stAt = -1, stSpeedBefore = -1, stSpeedMin = 1e9f, stSpeedAfter = -1, stPitchPeak = -90, stLowest = 10;
    // Alarmes de quase-queda (tropeco > 0,3) antes do obstaculo e em
    // qualquer cenario (ticks; eventos = subidas).
    int stumbleTicksBefore = 0, stumbleTicks = 0, stumbleEvents = 0;
    bool stumbleOn = false;
    int snagFoot = -1;
    float snagStartedAt = -1.0f;
    // Arrancada (sweeparrancada): raspadas (pe no meio do balanco tocando o
    // chao sem carga e deslizando - a mesma regra do tropeco da locomocao),
    // eventos e o primeiro, desde o comando.
    std::array<bool, 2> scuffOn {};
    // Bicudas em qualquer cenario (depois de 1 s, de pe): a mesma regra, por
    // pe, com a janela da curva (turnStart-0,1 .. turnEnd+1).
    std::array<bool, 2> kickOn {};
    int kicks = 0, kicksInTurn = 0;
    // Toques no balanco: o pe da passada (15-80% do balanco) encostando no
    // chao, com ou sem carga (eventos), e a menor folga da sola no meio do
    // balanco (30-70%) - balancos com menos de 2 cm. Separados pelo pe de
    // dentro/fora da curva (pelo giro do corpo; [0] dentro, [1] fora).
    std::array<bool, 2> swingTouchOn {};
    std::array<int, 2> swingTouches {}, lowSwings {}, swingsCounted {};
    int swingTouchesEarly = 0, swingTouchesLate = 0;  // < 35% / >= 50% do balanco
    std::array<float, 2> swingMinClearance { 1.0f, 1.0f };
    std::array<bool, 2> swingMidOn {};
    std::array<int, 2> swingSideAtStart {};
    float kickPreviousHeading = 0.0f, kickHeadingRate = 0.0f;
    // ... e o arrasto: quanto cada pe anda no plano encostado no chao (sola
    // a menos de 3 cm) nos 0,8 s depois do comando (m, o maior dos dois), e
    // quando o primeiro pe sai do chao (s).
    std::array<float, 2> startDrag {};
    std::array<Vec3, 2> startDragLast {};
    std::array<bool, 2> startDragValid {};
    float firstLiftAt = -1.0f;
    float startReachAt = -1.0f;  // corpo a 2,5 m/s (sprint) / 2,0 (trote) no rumo, s desde o comando
    float startNominal80At = -1.0f;  // corpo a 80% da velocidade nominal do comando, s desde ele
    int scuffs = 0;
    float firstScuffAt = -1.0f;
    float startAt = -1.0f;
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

// Pose do corpo para o renderizador do scratchpad (XPOSEDUMP=arquivo,
// XPOSEEVERY=ticks): por quadro, o colisor de cada link no mundo e os
// contatos com o chao.
void dumpPose(FILE* file, float t, int state, int phase, const RagdollProfile3D& profile,
    const RagdollState3D& body) {
    std::fprintf(file, "F %.4f %d %d\n", t, state, phase);
    for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
        const auto& collider = profile.links[i].collider;
        const Vec3 center = body.links[i].position + body.links[i].orientation.rotate(collider.localPosition);
        const Quaternion q = (body.links[i].orientation * collider.localOrientation).normalized();
        if (collider.shape == RagdollColliderShape3D::Box) {
            std::fprintf(file, "B %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w,
                collider.boxHalfExtents.x, collider.boxHalfExtents.y, collider.boxHalfExtents.z);
        } else {
            std::fprintf(file, "C %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w, ragdollCapsuleHalfSegment3D(collider),
                collider.radiusMeters, ragdollCapsuleRadiusAtPositiveX3D(collider));
        }
    }
    for (const auto& contact : body.interactions) {
        std::fprintf(file, "P %.4f %.4f %.4f %u\n", contact.positionWorld.x, contact.positionWorld.y,
            contact.positionWorld.z, contact.linkIndex);
    }
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
    BallControl3D ballControl;
    // O reforco de estabilidade do gesto com a bola (V3; XSEMREFORCO desliga).
    float ballActionStability = 0.0f;
    // O roteiro do pe do manejo no tick (rastro do contrato dos pes).
    BallFootScript3D lastBallScript;
    int lastBallPhase = 0;
    // Segurando o rumo (manejo automatico): a capsula vai assim no proximo
    // tick.
    bool autoSteerActive = false;
    Vec3 autoSteerVelocity {};
    // (O toque por area saiu no rework de 06/10: footTouch e o padrao.)
    bool ballPlaced = false;
    // O dono da bola (como no laboratorio; BallOwnership3D).
    BallOwnership3D ballOwnership;
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
    // (XPULOLAB: o pulo do laboratorio - o toque baixo, 0,72.)
    if (std::getenv("XPULOLAB")) {
        settings.jumpTapSeconds = 0.06f;
        settings.jumpFullChargeSeconds = 0.55f;
        settings.jumpTapSpeedScale = 0.72f;
    }
    settings.maximumSlopeDegrees = 50.0f;
    settings.maximumStepHeight = 0.19f;
    settings.skinWidth = 0.018f;
    scene->createCharacter({0, 0, 0}, settings);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 9101;
    spawn.active = true;
    spawn.pelvisPosition = {0, 0, scenario.spawnPelvisHeight > 0
        ? scenario.spawnPelvisHeight : profile.standingRootHeightMeters};
    if (scenario.spawnPitchDegrees != 0) {
        const float h = scenario.spawnTiltHeadingDegrees * 3.14159265f / 180;
        spawn.orientation = Quaternion::fromAxisAngle({-std::sin(h), std::cos(h), 0},
            scenario.spawnPitchDegrees * 3.14159265f / 180);
    }
    const auto handle = scene->createRagdoll(profile, spawn);
    CharacterLocomotion3D locomotion;
    locomotion.reset(profile, scene->ragdollState(handle));
    // (XCAMBALEIOINICIAL: comeca cambaleando, como logo depois de um choque.)
    if (const char* v = std::getenv("XCAMBALEIOINICIAL")) locomotion.debugSetStagger(std::strtof(v, nullptr));
    // E5-A (diagnostico): seguir um pe do comeco ao fim do passo - as quatro
    // posicoes do pe, a carga e as juntas da perna, capturadas em memoria e
    // impressas no fim (janela do primeiro passo de marcha desse pe).
    const int stepTraceSide = std::getenv("XSTEPTRACE") ? std::atoi(std::getenv("XSTEPTRACE")) : -1;
    if (stepTraceSide >= 0 || std::getenv("XPOSTRACE4") || std::getenv("XBRAKETRACE") || std::getenv("XPOUSO") || std::getenv("XTRANSTRACE") || std::getenv("XTCTRACE") || std::getenv("XSTTRACE")) locomotion.setFootTraceEnabled(true);
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
    // (O cambaleio escala estas por tick, como no laboratorio.)
    const float baseAssistRetained = controller.settings().legsAssistRetained;
    const float baseWalkingAssist = controller.settings().walkingAssistScale;
    if (const char* v = std::getenv("XLEGHF")) controller.settings().legHeightFrequency = std::strtof(v, nullptr);
    // (Experimentos do v2: limites da ajuda no plano, m/s2.)
    if (const char* v = std::getenv("XAPCATRITO")) controller.settings().frictionAccelerationLimit = std::strtof(v, nullptr);
    if (const char* v = std::getenv("XAPCPLANAR")) controller.settings().maximumPlanarAcceleration = std::strtof(v, nullptr);
    if (const char* v = std::getenv("XAPCFF")) controller.settings().maximumFeedforwardAcceleration = std::strtof(v, nullptr);
    if (const char* v = std::getenv("XTILTTORQUE")) {
        controller.settings().maximumTiltTorquePerKg = std::strtof(v, nullptr);
        controller.settings().v2TiltTorquePerKg = std::strtof(v, nullptr);
    }
    // Locomocao fisica v2 (XV2): o controle, a locomocao e a capsula.
    const bool physicalV2 = std::getenv("XV2") != nullptr && std::atoi(std::getenv("XV2")) != 0;
    controller.settings().v2 = physicalV2;
    if (const char* v = std::getenv("XLEGWEIGHT")) controller.settings().legsWeightFraction = std::strtof(v, nullptr);
    if (std::getenv("XNOLEGBAL")) controller.settings().jointBalance = false;
    if (std::getenv("XNOLEGPOST")) controller.settings().jointPosture = false;
    if (const char* v = std::getenv("XHF")) controller.settings().heightFrequency = std::strtof(v, nullptr);
    // R0 (pacote v2): as mudancas que valem tambem no H7, isoladas.
    if (std::getenv("XNOACCEPT")) locomotion.footworkSettings().loadAcceptanceByEvent = false;
    if (std::getenv("XNOCOMPLY")) locomotion.footworkSettings().acceptanceCompliance = false;
    if (std::getenv("XNOH18")) locomotion.footworkSettings().transferGeometryRelease = false;
    if (std::getenv("XNOZMP")) locomotion.footworkSettings().zmpLoadEstimate = false;
    if (std::getenv("XPROFILEDUMP")) {
        // Eixos das juntas (habilitado, limites em graus) e o quadro.
        for (const auto& link : profile.links) {
            std::printf("JUNTA %-14s pai %2d", link.id.c_str(), link.parentIndex);
            for (std::size_t a = 0; a < 3; ++a)
                std::printf(" | eixo%zu %s %+.0f..%+.0f", a, link.inboundJoint.axes[a].enabled ? "on " : "off",
                    link.inboundJoint.axes[a].minimumRadians * 57.3f, link.inboundJoint.axes[a].maximumRadians * 57.3f);
            const Quaternion f = link.inboundJoint.frameModelOrientation;
            const Vec3 x = f.rotate({1, 0, 0}), y = f.rotate({0, 1, 0});
            std::printf(" | quadro x(%+.2f %+.2f %+.2f) y(%+.2f %+.2f %+.2f)\n", x.x, x.y, x.z, y.x, y.y, y.z);
        }
    }
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
    // O peito (rastro XRASTROCSV).
    std::size_t chestIndexForTrace = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == "Chest") chestIndexForTrace = i;
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
        if (scenario.npcLabAssist) {
            // Como no laboratorio (E5, ajudas de 70%).
            npcController.settings() = controller.settings();
            npcLocomotion.footworkSettings().recoveryCaptureTargeting = true;
        }
    }
    int npcFalls = 0;
    float npcTackledRecovery = 1.0f;
    // A ajuda depois de um impacto (como no laboratorio desde 06/10:
    // ImpactAssist3D; XAJUDACAMBALEIO, a de antes - a ajuda caindo com o
    // cambaleio, e o carrinho levado a npcTackledPercent).
    const bool impactAssistModel = !std::getenv("XAJUDACAMBALEIO");
    ImpactAssist3D playerImpact, npcImpact;
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
    struct PoseDumpFile {
        FILE* file = nullptr;
        ~PoseDumpFile() { if (file) std::fclose(file); }
    } poseDump;
    if (const char* path = std::getenv("XPOSEDUMP")) {
        poseDump.file = std::fopen(path, "a");
        if (poseDump.file) std::fprintf(poseDump.file, "S %s\n", scenario.name);
    }
    FILE* poseFile = poseDump.file;
    const int poseEvery = std::getenv("XPOSEEVERY") ? std::max(1, std::atoi(std::getenv("XPOSEEVERY"))) : 6;
    DefaultLocomotion3D defaultLocomotion;
    if (const char* pitch = std::getenv("XVOADORADEITA")) defaultLocomotion.settings().airTacklePitchDegrees = std::strtof(pitch, nullptr);
    bool defaultKinematic = false, defaultCrouchHeld = false;
    // Corpo do modo padrao: o guiado, como no laboratorio (XGUIADO=0: o
    // cinematico de antes, para comparar); a saida do guiado com a
    // autoridade descendo em XGUIADOSAIDA s (padrao 0,12, o padrao rodando
    // junto) e a pancada que leva ao fisico (XGOLPE = impulso, como o
    // laboratorio: 4 N.s).
    const bool guidedBody = std::getenv("XGUIADO") == nullptr || std::atoi(std::getenv("XGUIADO")) != 0;
    const float guidedExit = std::getenv("XGUIADOSAIDA") ? std::strtof(std::getenv("XGUIADOSAIDA"), nullptr) : 0.12f;
    const float hitSwitch = std::getenv("XGOLPE") ? std::strtof(std::getenv("XGOLPE"), nullptr) : -1.0f;
    float defaultEnd = scenario.defaultUntil;
    float guidedExitLeft = 0.0f;
    std::vector<Vec3> defaultLinkVelocity;
    bool jumpedAtX = false;
    Vec3 capsuleVelocityBefore {}, capsulePositionBefore {}, capsuleMovedBefore {};
    // (v2: o rumo pedido no tick anterior e a taxa de giro dele, filtrada.)
    Vec3 previousRequestDirection {};
    bool previousRequestValid = false;
    float requestTurnRate = 0.0f;
    float switchAt = -1.0f;
    std::vector<Vec3> defaultCoordinates, defaultPrevious;
    std::vector<bool> legLink(profile.links.size(), false);
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        legLink[i] = profile.links[i].id.find("Thigh") != std::string::npos
            || profile.links[i].id.find("Shin") != std::string::npos || profile.links[i].id.find("Foot") != std::string::npos;
    std::vector<Quaternion> previousTargetOrientation;
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
        // Poses do jogador para a previa (XPOSESJSONL=arquivo, de XPOSEDE a
        // XPOSEATE s): origem e orientacao (xyzw) de cada link, uma linha por
        // tick.
        if (std::getenv("XRASTRO") && t > std::strtof(std::getenv("XRASTRO"), nullptr) && t < std::strtof(std::getenv("XRASTRO"), nullptr) + 0.6f) {
            const RagdollState3D now = scene->ragdollState(handle);
            std::printf("RASTRO t %.3f %s capsula x %.3f vx %.2f | pelve x %.3f vx %.2f z %.2f | assist %.2f cambaleio %.2f estado %d\n", t,
                defaultKinematic ? "padrao" : "fisico", scene->characterState().position.x, scene->characterState().velocity.x,
                now.links.front().position.x, now.links.front().linearVelocity.x, now.links.front().position.z,
                controller.output().assistEnvelope, locomotion.telemetry().stagger, static_cast<int>(locomotion.telemetry().state));
        }
        if (std::getenv("XIMPULSO") && t > 0.6f) {
            // Choques do jogador neste tick (fora o apoio dos pes): a soma e
            // o maior, por tipo de contato.
            const RagdollState3D now = scene->ragdollState(handle);
            float sum = 0.0f, biggest = 0.0f;
            int kind = -1;
            std::string link;
            for (const RagdollInteraction3D& contact : now.interactions) {
                if (contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if ((id == "LeftFoot" || id == "RightFoot") && contact.normalWorld.z > 0.62f) continue;
                sum += contact.normalImpulseNewtonSeconds;
                if (contact.normalImpulseNewtonSeconds > biggest) {
                    biggest = contact.normalImpulseNewtonSeconds;
                    kind = static_cast<int>(contact.otherMotion);
                    link = id;
                }
            }
            if (sum > 0.5f) std::printf("IMPULSO t %.3f soma %.1f maior %.1f (%s, %s) estado %d\n", t, sum, biggest, link.c_str(),
                kind == 0 ? "estatico" : kind == 1 ? "dinamico" : "cinematico", static_cast<int>(locomotion.telemetry().state));
        }
        if (const char* path = std::getenv("XPOSESJSONL")) {
            const float from = std::getenv("XPOSEDE") ? std::strtof(std::getenv("XPOSEDE"), nullptr) : 0.0f;
            const float to = std::getenv("XPOSEATE") ? std::strtof(std::getenv("XPOSEATE"), nullptr) : 1e9f;
            if (t >= from && t <= to)
                if (FILE* file = std::fopen(path, "a")) {
                    const RagdollState3D now = scene->ragdollState(handle);
                    std::fprintf(file, "{\"t\": %.4f, \"p\": [", t);
                    for (std::size_t i = 0; i < now.links.size(); ++i)
                        std::fprintf(file, "%s[%.4f, %.4f, %.4f]", i ? ", " : "", now.links[i].position.x,
                            now.links[i].position.y, now.links[i].position.z);
                    std::fprintf(file, "], \"q\": [");
                    for (std::size_t i = 0; i < now.links.size(); ++i)
                        std::fprintf(file, "%s[%.5f, %.5f, %.5f, %.5f]", i ? ", " : "", now.links[i].orientation.x,
                            now.links[i].orientation.y, now.links[i].orientation.z, now.links[i].orientation.w);
                    std::fprintf(file, "]}\n");
                    std::fclose(file);
                }
        }
        // O outro boneco (tambem com o jogador no modo padrao).
        const auto updateNpc = [&](const RagdollState3D& state, bool playerTackling) {
            const RagdollState3D other = scene->ragdollState(npcHandle);
            const auto& pelvis = other.links.front();
            CharacterLocomotionInput3D npcInput;
            npcInput.physicalV2 = physicalV2;
            npcInput.rootPositionWorld = {npcAnchor.x, npcAnchor.y,
                profile.standingRootHeightMeters};
            npcInput.rootVelocityWorld = {};
            npcInput.facingYawRadians = scenario.npcYaw;
            npcInput.grounded = true;
            npcInput.forceDrivenRoot = true;
            if (scenario.npcLabAssist) {
                // Levou carrinho (como no laboratorio): pancada de algo que se
                // move e uma perna do jogador em carrinho a menos de 45 cm.
                float impulse = 0.0f;
                for (const RagdollInteraction3D& contact : other.interactions)
                    if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
                bool tackled = false;
                if (impulse > 2.0f && playerTackling)
                    for (std::size_t i = 0; i < state.links.size() && !tackled; ++i)
                        if (legLink[i])
                            for (const PhysicsBodyState3D& link : other.links)
                                if ((link.position - state.links[i].position).lengthSquared() < 0.45f * 0.45f) { tackled = true; break; }
                if (tackled && scenario.npcTackledPercent >= 0.0f) {
                    if (npcTackledRecovery >= 1.0f) result.npcTackledAt = t;
                    npcTackledRecovery = 0.0f;
                } else {
                    npcTackledRecovery = std::min(1.0f, npcTackledRecovery + dt / 2.5f);
                }
                // (As do jogador sem o cambaleio dele; o do boneco abaixo.)
                npcController.settings().legsAssistRetained = baseAssistRetained;
                npcController.settings().walkingAssistScale = baseWalkingAssist;
                if (impactAssistModel) {
                    npcImpact.update(impactImpulse3D(other), tackled && scenario.npcTackledPercent >= 0.0f, impulse, dt);
                    npcController.settings().legsAssistRetained *= npcImpact.level();
                    npcController.settings().walkingAssistScale *= npcImpact.level();
                    result.npcAssistLow = std::min(result.npcAssistLow, npcImpact.level());
                } else if (npcTackledRecovery < 1.0f) {
                    const float back = npcTackledRecovery * npcTackledRecovery * (3.0f - 2.0f * npcTackledRecovery);
                    const float low = scenario.npcTackledPercent * 0.01f;
                    auto& assist = npcController.settings();
                    assist.legsAssistRetained = std::min(assist.legsAssistRetained,
                        low + (assist.legsAssistRetained - low) * back);
                    assist.walkingAssistScale = std::min(assist.walkingAssistScale,
                        low + (assist.walkingAssistScale - low) * back);
                }
            }
            npcInput.jointStrength = npcController.output().jointStrength;
            npcInput.externalPerturbation = npcController.output().perturbation;
            // O cambaleio do boneco (como no laboratorio, todos os bonecos).
            npcInput.staggerRecovery = !std::getenv("XSEMCAMBALEIO");
            if (scenario.npcLabAssist && !impactAssistModel) {
                const float stagger = npcLocomotion.telemetry().stagger;
                npcController.settings().legsAssistRetained *= 1.0f - 0.7f * stagger;
                npcController.settings().walkingAssistScale *= 1.0f - 0.7f * stagger;
            }
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
            npcIntent.brakeRunVelocityWorld = npcLocomotion.telemetry().brakeRunVelocityWorld;
            npcIntent.brakeRunShare = npcLocomotion.telemetry().brakeRunShare;
            npcController.update(profile, other, npcOutput.guide, npcIntent, dt);
            applyCharacterControl3D(*scene, npcHandle, npcOutput, &npcController.output(), npcDown);
            npcAnchorVelocity = navigationFollowVelocity3D(pelvis.position,
                npcAnchor, npcDown ? 0.0f : 0.10f * (1.0f - std::clamp(
                    npcController.output().perturbation, 0.0f, 1.0f)));
            npcAnchor += npcAnchorVelocity * dt;
            result.npcPelvisSpeedPeak = std::max(result.npcPelvisSpeedPeak, pelvis.linearVelocity.length());
            result.npcFalls = npcFalls;
            result.npcFinal = pelvis.position;
        };
        if (wanted.move.lengthSquared() > 1e-6f) {
            if (previousMove.lengthSquared() > 1e-6f && flipAt < 0
                && dot(previousMove.normalized(), wanted.move.normalized()) < -0.5f) {
                flipAt = t;
                flipDirection = wanted.move.normalized();
            }
            previousMove = wanted.move;
        }
        capsuleVelocityBefore = scene->characterState().velocity;
        capsulePositionBefore = scene->characterState().position;
        CharacterMotorCommand3D command;
        command.moveDirection = wanted.move;
        // E5 (passos pelo planejador): a capsula nao anda sozinha - so segue o
        // corpo; quem o leva sao as pernas.
        if (locomotion.footworkSettings().locomotionStepping) command.moveDirection = {};
        command.sprint = wanted.sprint;
        command.jumpPressed = wanted.jump;
        command.jumpChargeSeconds = wanted.jumpChargeSeconds;
        if (scenario.jumpWhenX > 0.0f && !jumpedAtX && scene->characterState().position.x >= scenario.jumpWhenX) {
            command.jumpPressed = true;
            jumpedAtX = true;
        }
        if (command.jumpPressed || wanted.jumpCharging) result.bolLastJumpAt = t;
        // (O pedido do jogador, antes de qualquer limite da capsula.)
        Vec3 boletimRequested {};
        if (wanted.move.lengthSquared() > 1e-6f) {
            const float travel = std::atan2(wanted.move.y, wanted.move.x) - wanted.lookYaw;
            boletimRequested = Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized()
                * characterGaitSpeed3D(speeds, travel, wanted.sprint);
        }
        if (wanted.jump) result.voadoraSpeedTakeoff = std::hypot(scene->characterState().velocity.x,
            scene->characterState().velocity.y);
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
        // Experimento (XCAPSULALIDER = m/s): a capsula no maximo isto a frente
        // da velocidade do corpo no rumo pedido - o corpo conduz.
        if (const char* lead = std::getenv("XCAPSULALIDER"); lead != nullptr && !defaultKinematic
            && wanted.move.lengthSquared() > 1e-6f) {
            const Vec3 body = locomotion.physicalState().centerOfMassVelocityWorld;
            const Vec3 direction = Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized();
            const float along = std::max(0.0f, body.x * direction.x + body.y * direction.y);
            const float nominal = wanted.sprint ? settings.sprintSpeed : settings.walkSpeed;
            command.speedScale = std::min(command.speedScale, (along + std::strtof(lead, nullptr)) / std::max(0.1f, nominal));
        }
        // Experimento v2 (XNAVLEASH = m/s): a velocidade pedida a capsula fica
        // a no maximo isto da velocidade real do corpo, em qualquer direcao
        // (a referencia conduzida pelo corpo: sem tranco depois de um pouso,
        // de um empurrao, de um zigue-zague que ele nao acompanhou).
        if (const char* leash = std::getenv("XNAVLEASH"); leash != nullptr && !defaultKinematic
            && wanted.move.lengthSquared() > 1e-6f) {
            const float nominal = wanted.sprint ? settings.sprintSpeed : settings.walkSpeed;
            Vec3 requested = wanted.move.lengthSquared() > 1e-6f
                ? Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized() * (nominal * command.speedScale) : Vec3 {};
            const Vec3 body = locomotion.physicalState().centerOfMassVelocityWorld;
            const Vec3 bodyPlanar { body.x, body.y, 0.0f };
            Vec3 difference = requested - bodyPlanar;
            const float limit = std::strtof(leash, nullptr);
            if (difference.length() > limit) difference = difference * (limit / difference.length());
            const Vec3 target = requested.lengthSquared() < 1e-6f && bodyPlanar.length() < limit ? Vec3 {} : bodyPlanar + difference;
            const float speed = target.length();
            command.moveDirection = speed > 1e-3f ? target * (1.0f / speed) : Vec3 {};
            command.speedScale = speed / std::max(0.1f, nominal);
        }
        // v2: virando forte, a velocidade cai como a de quem corre cortando
        // (physicalNavigationTurnScale3D); a taxa de giro do pedido, filtrada.
        if (wanted.move.lengthSquared() > 1e-6f && previousRequestValid) {
            const Vec3 b = Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized();
            const float rate = std::acos(std::clamp(previousRequestDirection.x * b.x + previousRequestDirection.y * b.y,
                -1.0f, 1.0f)) / dt;
            requestTurnRate += (rate - requestTurnRate) * (1.0f - std::exp(-dt / 0.25f));
        } else {
            requestTurnRate = 0.0f;
        }
        previousRequestValid = wanted.move.lengthSquared() > 1e-6f;
        if (previousRequestValid) previousRequestDirection = Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized();
        if (physicalV2 && !defaultKinematic && scene->characterState().grounded && !wanted.jump
            && wanted.move.lengthSquared() > 1e-6f && !std::getenv("XSEMCORTE"))
            command.speedScale *= physicalNavigationTurnScale3D(scene->characterState().velocity, wanted.move,
                requestTurnRate, controller.settings().walkingAssistScale);
        // v2: a capsula presa a velocidade do corpo (no chao, so ganhando
        // velocidade; ver physicalNavigationSpeedScale3D).
        if (physicalV2 && !defaultKinematic && scene->characterState().grounded && !wanted.jump
            && wanted.move.lengthSquared() > 1e-6f && !std::getenv("XSEMCOLEIRA")) {
            const float nominal = wanted.sprint ? settings.sprintSpeed : settings.walkSpeed;
            command.speedScale = physicalNavigationSpeedScale3D(wanted.move, command.speedScale, nominal,
                locomotion.physicalState().centerOfMassVelocityWorld,
                physicalNavigationLead3D(controller.settings().walkingAssistScale));
        }
        if (autoSteerActive && !std::getenv("XAUTONOHOLD")) {
            const float v = autoSteerVelocity.length();
            command.moveDirection = v > 1e-3f ? autoSteerVelocity * (1.0f / v) : Vec3 {};
            command.speedScale = v / (wanted.sprint ? settings.sprintSpeed : settings.walkSpeed);
        }
        // Experimento v2 (XNAVPOWER = W/kg): a velocidade da capsula vem de um
        // plano viavel - ganhar velocidade ate P/v (a potencia de quem corre:
        // ~3 m/s2 a 7 m/s), frear e virar ate XNAVACC (10) - e, andando, a no
        // maximo XNAVLEASH (0,4 m/s) a frente da velocidade real do corpo.
        // Assim, depois de um pouso que freou o corpo, a referencia freia
        // junto e volta a acelerar no ritmo de quem corre (sem o tranco).
        bool planned = false;
        if (const char* power = std::getenv("XNAVPOWER"); power != nullptr && !defaultKinematic
            && scene->characterState().grounded && !command.jumpPressed) {
            const float nominal = wanted.sprint ? settings.sprintSpeed : settings.walkSpeed;
            const float limitAcceleration = std::getenv("XNAVACC") ? std::strtof(std::getenv("XNAVACC"), nullptr) : 10.0f;
            Vec3 requested = command.moveDirection.lengthSquared() > 1e-6f
                ? Vec3 { command.moveDirection.x, command.moveDirection.y, 0.0f }.normalized() * (nominal * command.speedScale)
                : Vec3 {};
            const Vec3 body = locomotion.physicalState().centerOfMassVelocityWorld;
            const Vec3 bodyPlanar { body.x, body.y, 0.0f };
            if (requested.lengthSquared() > 1e-6f) {
                const float leash = std::getenv("XNAVLEASH") ? std::strtof(std::getenv("XNAVLEASH"), nullptr) : 0.4f;
                const float requestedSpeed = requested.length();
                const Vec3 direction = requested * (1.0f / requestedSpeed);
                const float along = bodyPlanar.x * direction.x + bodyPlanar.y * direction.y;
                // So a velocidade no rumo pedido fica presa ao corpo (virar
                // e frear seguem livres, pelo limite).
                if (requestedSpeed > std::max(0.0f, along) + leash)
                    requested = direction * (std::max(0.0f, along) + leash);
            }
            const Vec3 current { scene->characterState().velocity.x, scene->characterState().velocity.y, 0.0f };
            Vec3 change = requested - current;
            const float speed = current.length();
            Vec3 limited = change;
            if (speed > 0.3f) {
                const Vec3 unit = current * (1.0f / speed);
                float alongChange = change.x * unit.x + change.y * unit.y;
                Vec3 across = change - unit * alongChange;
                const float forward = std::min(limitAcceleration, std::strtof(power, nullptr) / std::max(1.0f, speed));
                alongChange = std::clamp(alongChange, -limitAcceleration * dt, forward * dt);
                if (across.length() > limitAcceleration * dt) across = across * (limitAcceleration * dt / across.length());
                limited = unit * alongChange + across;
            }
            if (limited.length() > limitAcceleration * dt) limited = limited * (limitAcceleration * dt / limited.length());
            const Vec3 next = current + limited;
            const float nextSpeed = next.length();
            command.moveDirection = nextSpeed > 1e-3f ? next * (1.0f / nextSpeed) : Vec3 {};
            command.speedScale = nextSpeed / std::max(0.1f, nominal);
            planned = true;
        }
        // Carrinho do padrao em andamento: como no laboratorio, o jogador conta
        // como caido e a capsula fica sem teclas; na voadora animada, ainda no
        // ar, ela segue no embalo do pulo (XVOADORAFREIA: sem isso).
        if (defaultKinematic && t < defaultEnd && defaultLocomotion.output().tackling) {
            command.moveDirection = {};
            const Vec3 flat { scene->characterState().velocity.x, scene->characterState().velocity.y, 0.0f };
            if (defaultLocomotion.output().airTackle && !scene->characterState().grounded && flat.length() > 0.05f
                && !std::getenv("XVOADORAFREIA")) {
                command.moveDirection = flat * (1.0f / flat.length());
                command.sprint = true;
                command.speedScale = flat.length() / settings.sprintSpeed;
            }
        }
        auto motorSettings = scenario.legacy ? settings
            : adaptiveCharacterMotorSettings3D(settings, scene->characterState(), command,
                defaultKinematic ? 0.0f : physicalV2
                    ? physicalNavigationAcceleration3D(controller.settings().walkingAssistScale) : 0.0f);
        if (planned) {
            motorSettings.groundAcceleration = motorSettings.groundDeceleration = 1000.0f;
            motorSettings.groundBrakeDeceleration = 1000.0f;
        }
        // Experimento v2 (XNAVACC = m/s2): a capsula acelera e freia no
        // maximo isto no modo fisico (o que o corpo consegue pelo chao).
        if (const char* v = std::getenv("XNAVACC"); v != nullptr && !defaultKinematic && !planned) {
            const float a = std::strtof(v, nullptr);
            motorSettings.groundAcceleration = std::min(motorSettings.groundAcceleration, a);
            motorSettings.groundDeceleration = std::min(motorSettings.groundDeceleration, a);
            motorSettings.groundBrakeDeceleration = std::min(motorSettings.groundBrakeDeceleration, a);
        }
        scene->moveCharacter(command, motorSettings, dt);
        const PhysicsCharacterState3D& capsule = scene->characterState();
        // A capsula batendo (como no laboratorio: medida logo depois de ela
        // andar e, no padrao, a troca ainda neste tick - o corpo segue com o
        // embalo dele, sem ir junto com a capsula parada).
        // (Pelo quanto ela andou de fato: a velocidade dela e a pedida.)
        const Vec3 capsuleMoved { (capsule.position.x - capsulePositionBefore.x) / dt,
            (capsule.position.y - capsulePositionBefore.y) / dt, 0.0f };
        if (hitSwitch > 0.0f && defaultKinematic && t < defaultEnd && result.hitSwitchAt < 0.0f && capsule.grounded) {
            const Vec3 before = capsuleMovedBefore;
            const Vec3 after = capsuleMoved;
            const float speed = before.length();
            const float braking = std::max({ motorSettings.groundAcceleration, motorSettings.groundDeceleration,
                motorSettings.groundBrakeDeceleration }) * dt;
            const float impact = speed > 0.5f ? std::max(0.0f, dot(before - after, before * (1.0f / speed)) - braking) : 0.0f;
            if (impact > 1.5f) {
                result.hitSwitchAt = t;
                result.hitSwitchWall = true;
                defaultEnd = t;
            }
        }
        capsuleMovedBefore = capsuleMoved;
        // A velocidade media da pelve no rumo pedido (speedFrom..speedTo).
        const auto measureWindowSpeed = [&](const RagdollState3D& body) {
            if (scenario.speedFrom < 0.0f || t < scenario.speedFrom || t > scenario.speedTo || body.links.empty()
                || wanted.move.lengthSquared() < 1e-6f) return;
            const Vec3 direction = Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized();
            result.windowSpeedSum += body.links.front().linearVelocity.x * direction.x
                + body.links.front().linearVelocity.y * direction.y;
            ++result.windowSamples;
        };
        // A voadora: a velocidade no toque (a pelve abaixo de 0,55 m) e o
        // deslize (o caminho da pelve baixa) - no padrao ou no fisico.
        const auto measureVoadora = [&](const RagdollState3D& body) {
            if (result.voadoraAt < 0.0f || result.voadoraSlideDone || body.links.empty()) return;
            const auto& pelvis = body.links.front();
            const float planar = std::hypot(pelvis.linearVelocity.x, pelvis.linearVelocity.y);
            if (result.voadoraSpeedLanding < 0.0f && pelvis.position.z < 0.55f) {
                result.voadoraSpeedLanding = planar;
                result.voadoraLandingAt = pelvis.position;
            } else if (result.voadoraSpeedLanding >= 0.0f) {
                if (pelvis.position.z < 0.55f)
                    result.voadoraSlideMeters = std::hypot(pelvis.position.x - result.voadoraLandingAt.x,
                        pelvis.position.y - result.voadoraLandingAt.y);
                else
                    result.voadoraSlideDone = true;
            }
        };
        // O padrao: a entrada da capsula e do script e o tick dele.
        const auto defaultStep = [&]() -> const DefaultLocomotionOutput3D& {
            DefaultLocomotionInput3D defaultInput;
            const float base = capsule.position.z - settings.standingHeight * 0.5f;
            defaultInput.feetPositionWorld = { capsule.position.x, capsule.position.y, base };
            defaultInput.velocityWorld = capsule.velocity;
            defaultInput.grounded = capsule.grounded;
            if (wanted.move.lengthSquared() > 1e-6f)
                defaultInput.requestedVelocityWorld = Vec3 { wanted.move.x, wanted.move.y, 0.0f }.normalized()
                    * Vec3 { capsule.velocity.x, capsule.velocity.y, 0.0f }.length();
            defaultInput.facingYawRadians = wanted.lookYaw;
            defaultInput.deltaTime = dt;
            defaultInput.tackleRequest = wanted.crouch && !defaultCrouchHeld;
            defaultInput.tackleHeld = wanted.crouch;
            // O chao (como no laboratorio: o pouso previsto no ar).
            PhysicsScene3D* groundScene = scene.get();
            defaultInput.groundAt = [groundScene](Vec3 point) {
                return groundScene->probeTerrain(point + Vec3 { 0.0f, 0.0f, 0.55f }, 0.03f, 0.0f, 1.5f, 60.0f);
            };
            defaultInput.airTackleCombo = wanted.sprint && wanted.move.lengthSquared() > 1e-6f
                && dot(wanted.move.normalized(), Vec3 { std::cos(wanted.lookYaw), std::sin(wanted.lookYaw), 0.0f }) > 0.7f;
            defaultCrouchHeld = wanted.crouch;
            defaultLocomotion.update(profile, *scenario.defaultAnimations, defaultInput);
            // Carrinho: a capsula vai embaixo da pelve do clipe e sai com o
            // embalo dela (como no laboratorio).
            const DefaultLocomotionOutput3D& out = defaultLocomotion.output();
            if (out.rootScripted)
                scene->placeCharacter({ out.tackleRootWorld.x, out.tackleRootWorld.y, 0.0f }, settings);
            if (out.tackleEnded)
                scene->setCharacterVelocity({ out.tackleVelocity.x, out.tackleVelocity.y, 0.0f });
            return out;
        };
        if (scenario.defaultUntil > 0.0f && scenario.defaultAnimations != nullptr && t < defaultEnd) {
            // Modo padrao: o corpo cinematico (ou guiado) na pose da animacao.
            if (!defaultKinematic) {
                if (guidedBody) scene->setRagdollGuided(handle, true);
                else scene->setRagdollKinematic(handle, true);
                defaultLocomotion.reset(wanted.lookYaw);
                defaultKinematic = true;
            }
            const DefaultLocomotionOutput3D& pose = defaultStep();
            if (defaultLocomotion.output().tackling && result.defaultTackleAt < 0.0f) result.defaultTackleAt = t;
            if (pose.valid) {
                if (guidedBody) scene->setRagdollGuidePose(handle, pose.pose.linkPositions, pose.pose.linkOrientations);
                else scene->setRagdollKinematicPose(handle, pose.pose.linkPositions, pose.pose.linkOrientations);
                defaultPrevious = defaultCoordinates.empty() ? pose.coordinates : defaultCoordinates;
                defaultCoordinates = pose.coordinates;
            }
            if (t > scenario.defaultUntil - 0.4f) {
                const RagdollState3D now = scene->ragdollState(handle);
                for (std::size_t i = 1; i < now.links.size(); ++i)
                    if (legLink[i]) result.switchBodySpinBefore = std::max(result.switchBodySpinBefore,
                        now.links[i].angularVelocity.length());
                result.switchSpeedBefore = std::hypot(now.links.front().linearVelocity.x,
                    now.links.front().linearVelocity.y);
            }
            if (scenario.npc) updateNpc(scene->ragdollState(handle), defaultLocomotion.output().tackling);
            scene->simulate(dt);
            {
                // O corpo contra a pose pedida, os picos de aceleracao e a
                // pancada (como no laboratorio).
                const RagdollState3D body = scene->ragdollState(handle);
                if (t > 0.6f) result.guideErrorPeak = std::max(result.guideErrorPeak, body.guideErrorMeters);
                if (pose.valid && t > 0.6f)
                    for (std::size_t i = 0; i < body.links.size() && i < pose.pose.linkPositions.size(); ++i)
                        result.guidePoseErrorPeak = std::max(result.guidePoseErrorPeak,
                            (body.links[i].position - pose.pose.linkPositions[i]).length());
                if (defaultLinkVelocity.size() == body.links.size() && t > 0.6f)
                    for (std::size_t i = 0; i < body.links.size(); ++i)
                        result.defaultLinkAccelPeak = std::max(result.defaultLinkAccelPeak,
                            (body.links[i].linearVelocity - defaultLinkVelocity[i]).length() / dt);
                defaultLinkVelocity.resize(body.links.size());
                for (std::size_t i = 0; i < body.links.size(); ++i) defaultLinkVelocity[i] = body.links[i].linearVelocity;
                if (hitSwitch > 0.0f) {
                    // (No carrinho, qualquer impacto: 0,4 N.s, como no
                    // laboratorio.)
                    float impulse = 0.0f;
                    for (const RagdollInteraction3D& contact : body.interactions)
                        if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
                    const float threshold = defaultLocomotion.output().tackling ? 0.4f : hitSwitch;
                    // Parede/mureta (contato com o que nao se move e nao e
                    // chao, 2,5 N.s) e a capsula batendo (como no
                    // laboratorio).
                    float wallImpulse = 0.0f;
                    for (const RagdollInteraction3D& contact : body.interactions)
                        if (contact.otherMotion != RagdollContactMotion3D::Dynamic && contact.normalWorld.z < 0.6f)
                            wallImpulse += contact.normalImpulseNewtonSeconds;
                    const bool wall = wallImpulse > (defaultLocomotion.output().tackling ? 0.4f : 2.5f);
                    if ((impulse > threshold || wall) && result.hitSwitchAt < 0.0f) {
                        result.hitSwitchAt = t;
                        result.hitSwitchWall = wall && impulse <= threshold;
                        defaultEnd = t + 0.5f * dt;
                    }
                }
                // A voadora: animada ate encostar em algo, como no laboratorio
                // (06/10); XVOADORAJA, ao fisico ja no comeco (o de antes).
                if (defaultLocomotion.output().airTackle && result.voadoraAt < 0.0f) {
                    result.voadoraAt = t;
                    result.voadoraSpeedSwitch = std::hypot(body.links.front().linearVelocity.x, body.links.front().linearVelocity.y);
                    if (std::getenv("XVOADORAJA")) defaultEnd = t + 0.5f * dt;
                }
                measureVoadora(body);
                measureWindowSpeed(body);
                if (impactAssistModel) playerImpact.update(impactImpulse3D(body), false, 0.0f, dt);
                // De pe no fim, no padrao: fora do carrinho e com a pelve em pe.
                if (!body.links.empty()) {
                    const float upright = (body.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                        .rotate({ 0.0f, 0.0f, 1.0f }).z;
                    result.standingAtEnd = !defaultLocomotion.output().tackling && upright > 0.85f;
                }
            }
            continue;
        }
        if (defaultKinematic) {
            // A ida ao fisico, como no laboratorio (toPhysical).
            defaultKinematic = false;
            switchAt = t;
            if (guidedBody) {
                scene->setRagdollGuided(handle, false);
                // (Na pancada, no carrinho e na voadora a troca e seca, como
                // no laboratorio.)
                guidedExitLeft = result.hitSwitchAt >= 0.0f || result.voadoraAt >= 0.0f
                    || defaultLocomotion.output().tackling ? 0.0f : guidedExit;
            } else {
                scene->setRagdollKinematic(handle, false);
            }
            if (!std::getenv("XSEMTIRAGIRO")) scene->removeRagdollVerticalSpin(handle);
            const RagdollState3D now = scene->ragdollState(handle);
            locomotion.reset(profile, now);
            controller.reset(profile);
            if (!std::getenv("XIDASEM")) {
                CharacterLocomotionSeed3D seed;
                seed.moving = wanted.move.lengthSquared() > 1e-6f;
                const DefaultLocomotion3D::Tackle tackle = defaultLocomotion.tackle();
                if (tackle.clip != nullptr && !std::getenv("XIDASEMCARRINHO")) {
                    seed.tackleClip = tackle.clip;
                    seed.tackleTime = tackle.time;
                    seed.tackleRate = tackle.rate;
                    seed.tackleScale = tackle.scale;
                    seed.tackleYaw = tackle.yaw;
                    seed.tackleEntrySpeed = tackle.entrySpeed;
                    seed.tackleStart = tackle.start;
                }
                locomotion.seedMotion(profile, now, seed);
            }
        }
        if (guidedExitLeft > 0.0f) {
            // Saida do guiado: o padrao segue rodando e a autoridade da pose
            // dele desce enquanto os motores do fisico assumem.
            const DefaultLocomotionOutput3D& pose = defaultStep();
            const float x = std::clamp(guidedExitLeft / std::max(1e-3f, guidedExit), 0.0f, 1.0f);
            const std::vector<float> authority(profile.links.size(), x * x * (3.0f - 2.0f * x));
            if (pose.valid) scene->setRagdollGuidePose(handle, pose.pose.linkPositions, pose.pose.linkOrientations, authority);
            guidedExitLeft -= dt;
            if (guidedExitLeft <= 0.0f && !std::getenv("XSEMTIRAGIRO")) scene->removeRagdollVerticalSpin(handle);
        }
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
        input.intent.crouch = wanted.crouch;
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
        input.walkingAssist = controller.settings().walkingAssistScale;
        input.physicalV2 = physicalV2;
        // Levantar (como os sliders do laboratorio): ajuda e rigidez.
        input.getUpAssist = std::getenv("XGUASSIST") ? std::strtof(std::getenv("XGUASSIST"), nullptr) : 1.3f;
        input.getUpStiffness = std::getenv("XGUTONE") ? std::strtof(std::getenv("XGUTONE"), nullptr) : 0.55f;
        input.getUpStrength = std::getenv("XGUFORCA") ? std::strtof(std::getenv("XGUFORCA"), nullptr) : 0.7f;
        input.getUpHurry = std::getenv("XLEVPRESSA") != nullptr;
        input.standingAssist = controller.settings().legsAssistRetained;
        if (scenario.ballScript && scenario.ball && scene->contains(*scenario.ball)) {
            if (!ballPlaced && scenario.ballPlaceAt >= 0.0f && t >= scenario.ballPlaceAt) {
                ballPlaced = true;
                if (scenario.ballClaimAtPlace) ballOwnership.interact(9101);
                const float yaw = locomotion.telemetry().facingYawRadians;
                const float c = std::cos(yaw), sn = std::sin(yaw);
                const Vec3 pelvis = state.links.front().position;
                const Vec3 o = scenario.ballPlaceOffset;
                const InflatedBallState3D was = scene->inflatedBallState(*scenario.ball);
                const Vec3 bv = scenario.ballPlaceVelocity;
                const Vec3 velocity { c * bv.x - sn * bv.y, sn * bv.x + c * bv.y, 0.0f };
                const Vec3 spin = cross(Vec3 { 0.0f, 0.0f, 1.0f }, velocity) * (1.0f / std::max(0.05f, was.position.z));
                scene->setInflatedBallMotion(*scenario.ball,
                    { pelvis.x + c * o.x - sn * o.y, pelvis.y + sn * o.x + c * o.y,
                        scenario.ballPlaceHeight > 0.0f ? scenario.ballPlaceHeight : was.position.z }, velocity, spin);
            }
            if (scenario.ballReplaceEvery > 0.0f && std::fmod(t, scenario.ballReplaceEvery) < dt) {
                const int k = static_cast<int>(t / scenario.ballReplaceEvery) + 31 * scenario.ballReplaceSeed;
                const float a = static_cast<float>((k * 2654435761u) % 6283u) / 1000.0f;
                const float d = 0.5f + static_cast<float>((k * 40503u) % 1500u) / 1000.0f;
                const Vec3 pelvis = state.links.front().position;
                scene->setInflatedBallMotion(*scenario.ball,
                    { pelvis.x + d * std::cos(a), pelvis.y + d * std::sin(a), 0.11f }, {}, {});
            }
            const InflatedBallState3D ball = scene->inflatedBallState(*scenario.ball);
            if (scenario.ballProbe) scenario.ballProbe(t, ball.position, ball.linearVelocity, state);
            if (scenario.ballCollides && !std::getenv("XSEMDONO")) {
                // Como no laboratorio: o dono da bola (quem a toca ou interage
                // com ela) nao colide com ela; longe dela, solta.
                if (ballControl.phase() != BallControlPhase3D::Idle || scenario.forcePassThrough)
                    ballOwnership.interact(9101);
                const Vec3 playerPelvis = state.links.front().position;
                Vec3 npcPelvis {};
                if (scenario.npc) npcPelvis = scene->ragdollState(npcHandle).links.front().position;
                const Vec3* ownerPelvis = ballOwnership.owner() == 9101 ? &playerPelvis
                    : ballOwnership.owner() == 9102 && scenario.npc ? &npcPelvis : nullptr;
                const std::uint64_t ownerBefore = ballOwnership.owner();
                ballOwnership.update(ball, ownerPelvis, dt);
                if (ballOwnership.owner() != ownerBefore || (result.ballOwnerLog.empty() && ownerBefore != 0))
                    result.ballOwnerLog.push_back({ t, ballOwnership.owner() });
                scene->setInflatedBallOwner(*scenario.ball, ballOwnership.owner() == 9101 ? handle
                    : ballOwnership.owner() == 9102 && scenario.npc ? npcHandle : RagdollHandle3D {});
                result.ballOwnedTicks += ballOwnership.owner() == 9101 ? 1 : 0;
            }
            BallControlInput3D request;
            float power = 0.0f;
            Vec3 direction {};
            const bool controlReady = ballControl.phase() == BallControlPhase3D::Idle
                || ballControl.phase() == BallControlPhase3D::FollowThrough;
            if (scenario.ballScript)
                request.request = scenario.ballScript(t, ball.position, state.links.front().position,
                    controlReady, power, direction);
            if (request.request != BallActionKind3D::None) ++result.ballRequests;
            if (request.request == BallActionKind3D::Shot) {
                request.shotLift = scenario.shotLift;
                request.shotCurve = scenario.shotCurve;
                result.shotRequestAt = t;
            }
            request.power = power;
            request.direction = direction;
            request.lookYawRadians = wanted.lookYaw;
            request.ballValid = true;
            request.ballPosition = ball.position;
            request.ballVelocity = ball.linearVelocity;
            request.profile = &profile;
            request.state = &state;
            request.facingYawRadians = wanted.lookYaw;
            request.bodyYawRadians = locomotion.telemetry().facingYawRadians;
            request.bodyYawValid = true;
            request.groundHeight = 0.0f;
            request.requestedVelocity = input.intent.requestedVelocityWorld;
            // (A velocidade do corpo: no fisico, a da pelve - como no
            // laboratorio; a capsula vai a frente do corpo que atrasa.)
            request.bodyVelocityValid = false;
            if (const char* factor = std::getenv("XFATOR")) request.contactTimingFactor = std::strtof(factor, nullptr);
            // (XSEMHITBOX: a forca pelo tempo da V3, sem a hitbox do pe.)
            request.hitboxContact = std::getenv("XSEMHITBOX") == nullptr;
            for (std::size_t side = 0; side < 2; ++side) {
                request.swingProgress[side] = locomotion.telemetry().footSwingProgress[side];
                request.footPlanted[side] = locomotion.telemetry().footPlanted[side];
                request.footLanding[side] = locomotion.telemetry().footLandingWorld[side];
                request.secondsToTouchdown[side] = locomotion.telemetry().footSecondsToTouchdown[side];
            }
            {
                const CharacterLocomotionState3D now = locomotion.telemetry().state;
                request.standing = (now == CharacterLocomotionState3D::Idle || now == CharacterLocomotionState3D::Turning)
                    && input.intent.requestedVelocityWorld.lengthSquared() < 0.01f;
            }
            request.deltaTime = dt;
            BallControlOutput3D touch;
            ballControl.update(request, touch);
            lastBallScript = touch.script;
            lastBallPhase = static_cast<int>(touch.phase);
            if (std::getenv("XMANEJOVEL") && tick % 12 == 0)
                std::printf("VEL %.2f pelve (%.2f %.2f %.2f) v (%.2f %.2f) bola (%.2f %.2f) vb %.2f fase %d estado %d\n", t,
                    state.links.front().position.x, state.links.front().position.y, state.links.front().position.z,
                    state.links.front().linearVelocity.x, state.links.front().linearVelocity.y, ball.position.x,
                    ball.position.y, ball.linearVelocity.length(), static_cast<int>(touch.phase),
                    static_cast<int>(locomotion.telemetry().state));
            // O reforco (como no laboratorio: cortado na volta de um impacto).
            ballActionStability = std::getenv("XSEMREFORCO") || (impactAssistModel && playerImpact.recovering())
                ? 0.0f : touch.stability;
            result.ballStabilityPeak = std::max(result.ballStabilityPeak, ballActionStability);
            // (O manejo nunca leva o corpo ate a bola - rework de 06/10; o
            // chute correndo so freia um pouco.)
            autoSteerActive = false;
            input.intent.requestedVelocityWorld = input.intent.requestedVelocityWorld * touch.speedScale;
            input.ballGesture = touch.gesture;
            input.ballFoot = touch.foot;
            input.ballScript = touch.script;
            input.ballBody = touch.body;
            input.ballRepel = touch.repel;
            input.ballAvoid = touch.avoid;
            input.ballPlant = touch.plant;
            input.ballGaitRate = touch.gaitRate;
            input.ballKick = touch.kick;
            if (touch.setMotion)
                scene->setInflatedBallMotion(*scenario.ball, ball.position, touch.ballVelocity, touch.ballSpin);
            {
                bool contact = false;
                for (std::uint32_t k = 0; k < ball.contactCount; ++k) contact = contact || ball.contacts[k].otherDynamic;
                if (contact && !result.ballContactPrevious && (result.ballTouchAt < 0.0f || t - result.ballTouchAt > 0.15f)
                    && !touch.touched)
                    ++result.ballAccidental;
                result.ballContactPrevious = contact;
            }
            if (touch.touched) {
                result.ballTouchAt = t;
                ++result.ballTouches;
                ++result.ballSurfaces[static_cast<std::size_t>(touch.surface)];
                if (touch.kind == BallActionKind3D::Shot) {
                    result.shotAt = t;
                    result.shotDelay = t - result.shotRequestAt;
                    result.shotSpin = touch.ballSpin.length();
                    result.shotKickSide = touch.side;
                    result.shotStart = ball.position;
                    if (touch.plant.active) {
                        const Vec3 plant = state.links[feet[static_cast<std::size_t>(touch.plant.side)]].position;
                        const Vec3 aim { std::cos(wanted.lookYaw), std::sin(wanted.lookYaw), 0.0f };
                        const Vec3 off { plant.x - ball.position.x, plant.y - ball.position.y, 0.0f };
                        result.shotPlantAlong = dot(off, aim);
                        result.shotPlantAcross = off.x * -aim.y + off.y * aim.x;
                    }
                }
            }
            if (touch.missReason == 1) ++result.ballMissFar;
            if (touch.missReason == 2) ++result.ballMissLate;
            if (std::getenv("XCHUTETRACE") && (tick % 3 == 0 || touch.touched) && touch.kind == BallActionKind3D::Shot)
                std::printf("CHU %.3f fase %d chute %d apoio %d(%d) estagio %.2f peso %.2f ritmo %.2f gap %.3f | prog %.2f %.2f plant %d %d | pouso apoio (%.2f %.2f) alvo (%.2f %.2f) | pe chute (%.2f %.2f %.2f) alvo (%.2f %.2f %.2f) | bola (%.2f %.2f) v %.1f%s\n",
                    t,
                    static_cast<int>(touch.phase), touch.side, touch.plant.side, touch.plant.active ? 1 : 0, touch.kick.stage,
                    touch.kick.weight, touch.gaitRate, touch.footGap, request.swingProgress[0], request.swingProgress[1],
                    request.footPlanted[0] ? 1 : 0, request.footPlanted[1] ? 1 : 0,
                    touch.plant.side >= 0 ? request.footLanding[static_cast<std::size_t>(touch.plant.side)].x : 0.0f,
                    touch.plant.side >= 0 ? request.footLanding[static_cast<std::size_t>(touch.plant.side)].y : 0.0f,
                    touch.plant.position.x, touch.plant.position.y,
                    touch.side >= 0 ? state.links[feet[static_cast<std::size_t>(touch.side)]].position.x : 0.0f,
                    touch.side >= 0 ? state.links[feet[static_cast<std::size_t>(touch.side)]].position.y : 0.0f,
                    touch.side >= 0 ? state.links[feet[static_cast<std::size_t>(touch.side)]].position.z : 0.0f,
                    touch.foot.position.x, touch.foot.position.y, touch.foot.position.z,
                    ball.position.x, ball.position.y, ball.linearVelocity.length(), touch.touched ? " TOQUE" : "");
            if (std::getenv("XGESTOTRACE") && touch.gesture.active && tick % 2 == 0) {
                const int side = touch.gesture.side;
                const Vec3 actual = state.links[feet[static_cast<std::size_t>(side)]].position;
                std::printf("GES %.3f fase %d pe %d peso %.2f alvo (%.2f %.2f %.2f) real (%.2f %.2f %.2f) erro %.2f gap %.3f planted %d %d prog %.2f %.2f%s\n",
                    t, static_cast<int>(touch.phase), side, touch.foot.weight, touch.foot.position.x, touch.foot.position.y,
                    touch.foot.position.z, actual.x, actual.y, actual.z, (touch.foot.position - actual).length(), touch.footGap,
                    request.footPlanted[0] ? 1 : 0, request.footPlanted[1] ? 1 : 0, request.swingProgress[0],
                    request.swingProgress[1], touch.touched ? " TOQUE" : "");
            }
            if (std::getenv("XMANEJOTRACE") && tick % 6 == 0)
                std::printf("MAN %.2f fase %d pe %d sup %d gap %.3f peso %.2f | desvio E (%.2f %.2f %.2f) D (%.2f %.2f %.2f) | prog %.2f %.2f | bola (%.2f %.2f %.2f) v %.2f a frente %.2f | corpo v %.2f | pe (%.2f %.2f %.2f)%s\n",
                    t, static_cast<int>(touch.phase), touch.side, static_cast<int>(touch.surface), touch.footGap, touch.foot.weight,
                    touch.avoid[0].x, touch.avoid[0].y, touch.avoid[0].z, touch.avoid[1].x, touch.avoid[1].y, touch.avoid[1].z,
                    request.swingProgress[0], request.swingProgress[1],
                    ball.position.x, ball.position.y, ball.position.z, ball.linearVelocity.length(),
                    std::hypot(ball.position.x - state.links.front().position.x, ball.position.y - state.links.front().position.y),
                    std::hypot(state.links.front().linearVelocity.x, state.links.front().linearVelocity.y),
                    touch.side >= 0 ? state.links[feet[static_cast<std::size_t>(touch.side)]].position.x : 0.0f,
                    touch.side >= 0 ? state.links[feet[static_cast<std::size_t>(touch.side)]].position.y : 0.0f,
                    touch.side >= 0 ? state.links[feet[static_cast<std::size_t>(touch.side)]].position.z : 0.0f,
                    touch.touched ? " TOQUE" : "");
            // Distancia da bola a pelve com o corpo andando; o chute.
            const Vec3 pelvis = state.links.front().position;
            if (std::hypot(state.links.front().linearVelocity.x, state.links.front().linearVelocity.y) > 1.0f
                && result.shotAt < 0.0f) {
                const float d = std::hypot(ball.position.x - pelvis.x, ball.position.y - pelvis.y);
                result.ballMaxDistance = std::max(result.ballMaxDistance, d);
                result.ballDistanceSum += d;
                ++result.ballDistanceSamples;
            }
            if (result.shotAt >= 0.0f) {
                const Vec3 aim { std::cos(wanted.lookYaw), std::sin(wanted.lookYaw), 0.0f };
                const Vec3 off = ball.position - result.shotStart;
                result.shotApex = std::max(result.shotApex, ball.position.z);
                result.shotTravel = dot(Vec3 { off.x, off.y, 0.0f }, aim);
                if (!result.shotLateralDone) {
                    result.shotLateral20 = off.x * -aim.y + off.y * aim.x;
                    result.shotLateralDone = result.shotTravel >= 20.0f;
                }
            }
            if (result.shotAt >= 0.0f && t - result.shotAt < 0.2f && ball.linearVelocity.length() > result.shotSpeed) {
                result.shotSpeed = ball.linearVelocity.length();
                const float yaw = std::atan2(ball.linearVelocity.y, ball.linearVelocity.x);
                result.shotYawError = std::remainder(yaw - wanted.lookYaw, 6.2831853f) * 57.2958f;
                result.shotElevation = std::atan2(ball.linearVelocity.z,
                    std::hypot(ball.linearVelocity.x, ball.linearVelocity.y)) * 57.2958f;
            }
        }
        // Cambaleio (como no laboratorio; XSEMCAMBALEIO desliga): a ajuda e
        // o pedido de andar caem com ele.
        input.staggerRecovery = !std::getenv("XSEMCAMBALEIO");
        {
            const float stagger = locomotion.telemetry().stagger;
            if (impactAssistModel) {
                playerImpact.update(impactImpulse3D(state), false, 0.0f, dt);
                controller.settings().legsAssistRetained = baseAssistRetained * playerImpact.level();
                controller.settings().walkingAssistScale = baseWalkingAssist * playerImpact.level();
                result.playerAssistLow = std::min(result.playerAssistLow, playerImpact.level());
            } else {
                controller.settings().legsAssistRetained = baseAssistRetained * (1.0f - 0.7f * stagger);
                controller.settings().walkingAssistScale = baseWalkingAssist * (1.0f - 0.7f * stagger);
            }
            input.intent.requestedVelocityWorld = input.intent.requestedVelocityWorld * (1.0f - stagger);
            result.staggerPeak = std::max(result.staggerPeak, stagger);
        }
        locomotion.update(profile, clips.animations, state,
            scene->ragdollDynamics(handle), input, dt);
        const auto& output = locomotion.output();
        if (switchAt >= 0.0f && t - switchAt < 0.4f
            && output.targetPose.linkOrientations.size() == state.links.size()) {
            const auto angle = [](Quaternion a, Quaternion b) {
                const float d = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
                return 2.0f * std::acos(std::min(1.0f, d));
            };
            if (result.switchTargetGap < 0.0f) {
                float sum = 0.0f;
                int count = 0;
                for (std::size_t i = 1; i < state.links.size(); ++i)
                    if (legLink[i]) { sum += angle(output.targetPose.linkOrientations[i], state.links[i].orientation); ++count; }
                result.switchTargetGap = sum / std::max(1, count) * 57.2958f;
            } else if (previousTargetOrientation.size() == state.links.size()) {
                for (std::size_t i = 1; i < state.links.size(); ++i)
                    if (legLink[i]) result.switchTargetSpin = std::max(result.switchTargetSpin,
                        angle(output.targetPose.linkOrientations[i], previousTargetOrientation[i]) / dt);
            }
            previousTargetOrientation = output.targetPose.linkOrientations;
        }
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
        // No carrinho o corpo tambem sai do controle do corpo em pe (como no
        // laboratorio), mas carrinho nao e queda.
        const bool tackling = bodyState == CharacterLocomotionState3D::Tackling;
        const bool suspended = down || tackling;
        AdaptivePhysicalIntent3D intent;
        intent.proxyVelocityWorld = input.proxyVelocityWorld;
        intent.navigationRootWorld = input.rootPositionWorld;
        intent.airborne = !input.grounded;
        intent.manipulated = input.manipulated;
        intent.suspended = suspended;
        intent.postureLost = locomotion.telemetry().postureLost;
        intent.braceRecovering = locomotion.telemetry().braceRecovering;
        intent.jumpCharge = jumpCharge;
        intent.footGround = input.footGround;
        intent.followVelocityWorld = follow;
        intent.requestedVelocityWorld = input.intent.requestedVelocityWorld;
        intent.brakeRunVelocityWorld = locomotion.telemetry().brakeRunVelocityWorld;
        intent.brakeRunShare = locomotion.telemetry().brakeRunShare;
        intent.physicalState = &locomotion.physicalState();
        intent.contactPlan = &locomotion.contactPlan();
        intent.targetPose = &output.targetPose;
        intent.footPlantedValid = true;
        intent.footPlanted = locomotion.telemetry().footPlanted;
        intent.actionStability = ballActionStability;
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
            scenario.legacy || std::getenv("XNOASSIST") ? nullptr : &physical, suspended);
        // A capsula vai atras do corpo (mesma regra do laboratorio).
        follow = {};
        if (suspended) {
            const Vec3 root = state.links.front().position;
            scene->placeCharacter({root.x, root.y, 0}, settings);
            if (tackling) {
                const Vec3 v = state.links.front().linearVelocity;
                scene->setCharacterVelocity({ v.x, v.y, 0.0f });
            }
        } else {
            follow = navigationFollowVelocity3D(state.links.front().position,
                output.navigationAnchorWorld,
                0.10f * (1.0f - std::clamp(physical.perturbation, 0.0f, 1.0f)));
        }
        if (scenario.disturb) scenario.disturb(*scene, t, state);
        if (scenario.snagAt > 0.0f && t >= scenario.snagAt && result.firstFallSeconds < 0.0f) {
            if (result.snagFoot < 0 && result.snagStartedAt < 0.0f) {
                for (int side = 0; side < 2; ++side) {
                    const float progress = locomotion.telemetry().footSwingProgress[side];
                    if (progress >= 0.10f && progress <= 0.60f) { result.snagFoot = side; break; }
                }
                if (result.snagFoot >= 0) result.snagStartedAt = t;
            }
            if (result.snagFoot >= 0 && t < result.snagStartedAt + scenario.snagSeconds) {
                Vec3 travel = state.links.front().linearVelocity;
                travel.z = 0.0f;
                travel = travel.lengthSquared() > 0.01f ? travel.normalized() : Vec3 { 1, 0, 0 };
                scene->applyRagdollLinkForce(handle, static_cast<std::uint32_t>(feet[static_cast<std::size_t>(result.snagFoot)]),
                    travel * -scenario.snagNewtons, {});
            }
        }
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
        if (scenario.npc) updateNpc(state, locomotion.telemetry().state == CharacterLocomotionState3D::Tackling);
        scene->simulate(dt);

        const RagdollState3D after = scene->ragdollState(handle);
        // A voadora do modo fisico: o carrinho comecando no ar.
        if (!defaultKinematic && result.voadoraAt < 0.0f && locomotion.telemetry().tackling
            && !scene->characterState().grounded && !after.links.empty()) {
            result.voadoraAt = t;
            result.voadoraSpeedSwitch = std::hypot(after.links.front().linearVelocity.x, after.links.front().linearVelocity.y);
        }
        measureVoadora(after);
        measureWindowSpeed(after);
        {
            const Vec3 forward = (after.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                .rotate({ 1.0f, 0.0f, 0.0f });
            const float yaw = std::atan2(forward.y, forward.x) * 57.2958f;
            if (!scene->characterState().grounded && switchAt >= 0.0f) {
                if (result.airYawStart > 1e8f) result.airYawStart = yaw;
                result.airYawDrift = std::max(result.airYawDrift, std::abs(std::remainder(yaw - result.airYawStart, 360.0f)));
            }
        }
        if (std::getenv("XYAWTRACE") && tick % (std::getenv("XYAWEVERY") ? std::atoi(std::getenv("XYAWEVERY")) : 6) == 0) {
            const Vec3 forward = (after.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                .rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 raw = after.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
            float spin = 0.0f, mass = 0.0f;
            for (std::size_t i = 0; i < after.links.size(); ++i) {
                spin += after.links[i].angularVelocity.z * profile.links[i].massFraction;
                mass += profile.links[i].massFraction;
            }
            std::printf("  giro z pelve %+.2f medio %+.2f cru %+.1f |", after.links.front().angularVelocity.z, spin / mass,
                std::atan2(raw.y, raw.x) * 57.2958f);
            std::printf("  t %.2f estado %d chao %d pelve %+.1f rumo %+.1f velocidade %+.2f %+.2f %+.2f\n", t,
                static_cast<int>(locomotion.telemetry().state), scene->characterState().grounded ? 1 : 0,
                std::atan2(forward.y, forward.x) * 57.2958f, locomotion.telemetry().facingYawRadians * 57.2958f,
                after.links.front().linearVelocity.x, after.links.front().linearVelocity.y, after.links.front().linearVelocity.z);
        }
        if (switchAt >= 0.0f && t - switchAt < 0.6f) {
            if (t - switchAt < 0.4f)
                for (std::size_t i = 1; i < after.links.size(); ++i)
                    if (legLink[i]) result.switchBodySpin = std::max(result.switchBodySpin, after.links[i].angularVelocity.length());
            result.switchSpeedMin = std::min(result.switchSpeedMin,
                std::hypot(after.links.front().linearVelocity.x, after.links.front().linearVelocity.y));
            if (std::getenv("XIDATRACE") && static_cast<int>((t - switchAt) * 120.0f + 0.5f) % 6 == 0)
                std::printf("  +%.2f v %.2f capsula %.2f estado %d pes %d%d fase %.2f\n", t - switchAt,
                    std::hypot(after.links.front().linearVelocity.x, after.links.front().linearVelocity.y),
                    std::hypot(scene->characterState().velocity.x, scene->characterState().velocity.y),
                    static_cast<int>(locomotion.telemetry().state), locomotion.telemetry().footPlanted[0] ? 1 : 0,
                    locomotion.telemetry().footPlanted[1] ? 1 : 0, locomotion.telemetry().cyclePhase);
        }
        if (scenario.fluidFrom >= 0.0f && t >= scenario.fluidFrom && t <= scenario.fluidTo
            && output.targetPose.linkOrientations.size() == after.links.size() && !down) {
            const auto angle = [](Quaternion a, Quaternion b) {
                const float d = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
                return 2.0f * std::acos(std::min(1.0f, d)) * 57.2958f;
            };
            const auto tilt = [](Quaternion a, Quaternion b) {
                const Vec3 ua = a.rotate({ 0.0f, 0.0f, 1.0f }), ub = b.rotate({ 0.0f, 0.0f, 1.0f });
                return std::acos(std::clamp(dot(ua, ub), -1.0f, 1.0f)) * 57.2958f;
            };
            const auto yawOf = [](Quaternion q) {
                const Vec3 f = q.rotate({ 1.0f, 0.0f, 0.0f });
                return std::atan2(f.y, f.x);
            };
            static std::size_t chest = 0, head = 0;
            if (chest == 0)
                for (std::size_t i = 0; i < profile.links.size(); ++i) {
                    if (profile.links[i].id == "UpperChest") chest = i;
                    if (profile.links[i].id == "Head") head = i;
                }
            const auto& target = output.targetPose;
            const float trunk = tilt(after.links[chest].orientation, target.linkOrientations[chest]);
            const float pelvisTilt = tilt(after.links[0].orientation, target.linkOrientations[0]);
            const float headTilt = tilt(after.links[head].orientation, target.linkOrientations[head]);
            const float yaw = std::abs(std::remainder(yawOf(after.links[0].orientation) - yawOf(target.linkOrientations[0]),
                6.2831853f)) * 57.2958f;
            float all = 0.0f;
            for (std::size_t i = 0; i < after.links.size(); ++i)
                all += angle(after.links[i].orientation, target.linkOrientations[i]);
            all /= static_cast<float>(after.links.size());
            const Vec3 off = after.links[0].position - target.linkPositions[0];
            const float h = std::hypot(off.x, off.y) * 100.0f, v = std::abs(off.z) * 100.0f;
            result.fluTrunkSq += trunk * trunk; result.fluTrunkMax = std::max(result.fluTrunkMax, trunk);
            result.fluPelvisSq += pelvisTilt * pelvisTilt; result.fluPelvisMax = std::max(result.fluPelvisMax, pelvisTilt);
            result.fluHeadSq += headTilt * headTilt; result.fluHeadMax = std::max(result.fluHeadMax, headTilt);
            result.fluYawSq += yaw * yaw; result.fluYawMax = std::max(result.fluYawMax, yaw);
            result.fluAllSq += all * all; result.fluAllMax = std::max(result.fluAllMax, all);
            result.fluPosHSq += h * h; result.fluPosHMax = std::max(result.fluPosHMax, h);
            result.fluPosVSq += v * v; result.fluPosVMax = std::max(result.fluPosVMax, v);
            result.fluPosVSigned += off.z * 100.0f;
            {
                const Vec3 pelvisVelocity = after.links[0].linearVelocity;
                const Vec3 targetVelocity = result.fluPrevValid
                    ? (target.linkPositions[0] - result.fluPrevTargetPosition) * (1.0f / dt) : Vec3 {};
                if (result.fluPrevValid) {
                    const Vec3 accel = (pelvisVelocity - result.fluPrevPelvisVelocity) * (1.0f / dt);
                    const Vec3 targetAccel = (targetVelocity - result.fluPrevTargetVelocity) * (1.0f / dt);
                    const float impact = (accel - targetAccel).length();
                    if (result.fluSamples > 2) {
                        result.fluImpactSq += impact * impact;
                        result.fluImpactMax = std::max(result.fluImpactMax, impact);
                    }
                }
                result.fluPrevPelvisVelocity = pelvisVelocity;
                result.fluPrevTargetVelocity = targetVelocity;
                result.fluPrevTargetPosition = target.linkPositions[0];
                result.fluPrevValid = true;
            }
            ++result.fluSamples;
            if (locomotion.telemetry().tripRecovery > 0.05f || locomotion.telemetry().stumbleRisk > 0.5f)
                ++result.fluStumbleTicks;
        }
        {
            const bool fluidWindow = scenario.fluidFrom >= 0.0f && t >= scenario.fluidFrom && t <= scenario.fluidTo;
            const bool startWindow = scenario.startAt > 0.0f && t >= scenario.startAt && t <= scenario.startAt + 0.8f;
            if ((fluidWindow || startWindow) && !down) {
                for (std::size_t side = 0; side < 2; ++side) {
                    const std::size_t link = feet[side];
                    Vec3 point {};
                    int count = 0;
                    for (const auto& contact : after.contacts)
                        if (contact.linkIndex == link && contact.normal.z > 0.6f && !contact.otherBodyDynamic) { point += contact.position; ++count; }
                    if (count == 0) { result.slipCount[side] = 0; continue; }
                    point = point * (1.0f / static_cast<float>(count));
                    const auto& body = after.links[link];
                    const Vec3 com = body.position + body.orientation.rotate(profile.links[link].centerOfMassLocal);
                    const Vec3 v = body.linearVelocity + cross(body.angularVelocity, point - com);
                    // Media de 12 ticks (0,1 s): zera a vibracao de 60 Hz.
                    auto& window = result.slipWindow[side];
                    window[static_cast<std::size_t>(result.slipCount[side] % 12)] = Vec3 { v.x, v.y, 0.0f };
                    ++result.slipCount[side];
                    if (result.slipCount[side] < 12) continue;
                    Vec3 mean {};
                    for (const Vec3& sample : window) mean += sample;
                    mean = mean * (1.0f / 12.0f);
                    const float speed = mean.length();
                    result.slipMeters += speed * dt;
                    result.slipWorst = std::max(result.slipWorst, speed);
                    static std::array<Vec3, 2> previousTarget {};
                    const Vec3 footTarget = output.targetPose.linkPositions[link];
                    const Vec3 targetVelocity = (footTarget - previousTarget[side]) * (1.0f / dt);
                    previousTarget[side] = footTarget;
                    if (std::getenv("XDESLIZETRACE") && speed > 0.15f)
                        std::printf("DESLIZE %.3f pe %zu %.2f m/s (%+.2f %+.2f) | alvo z %.3f v (%+.2f %+.2f %+.2f) pe z %.3f forca %.0f | plantado %d balanco %.2f | contato clipe %.2f | inalcancavel %d | pelve v %.2f\n",
                            t, side, speed, v.x, v.y, footTarget.z, targetVelocity.x, targetVelocity.y, targetVelocity.z, body.position.z,
                            0.0f,
                            locomotion.telemetry().footPlanted[side] ? 1 : 0,
                            locomotion.telemetry().footSwingProgress[side], locomotion.telemetry().clipFootContact[side],
                            locomotion.telemetry().footworkAnchorUnreachable[side] ? 1 : 0,
                            std::hypot(after.links[0].linearVelocity.x, after.links[0].linearVelocity.y));
                }
            }
        }
        if (scenario.legTwistFrom >= 0.0f && t >= scenario.legTwistFrom && !down) {
            const auto yawOf = [](Quaternion q, Vec3 axis) {
                const Vec3 f = q.rotate(axis);
                return std::atan2(f.y, f.x);
            };
            const auto wrapDeg = [](float a) { return std::abs(std::remainder(a, 6.2831853f)) * 57.2958f; };
            static std::array<std::size_t, 2> shins {}, thighs {};
            if (shins[0] == 0)
                for (std::size_t i = 0; i < profile.links.size(); ++i) {
                    if (profile.links[i].id == "LeftShin") shins[0] = i;
                    if (profile.links[i].id == "RightShin") shins[1] = i;
                    if (profile.links[i].id == "LeftThigh") thighs[0] = i;
                    if (profile.links[i].id == "RightThigh") thighs[1] = i;
                }
            // Rumo da frente de cada link no modelo: o eixo x local (o pe e a
            // pelve apontam +x; a canela, o joelho para a frente, pelo eixo de
            // flexao). Aproximacao: frente do link = rotacao do +x do modelo.
            const float pelvisYaw = yawOf(after.links[0].orientation * profile.links[0].modelOrientation.conjugate(), { 1, 0, 0 });
            for (std::size_t side = 0; side < 2; ++side) {
                const auto shinModel = profile.links[shins[side]].modelOrientation.conjugate();
                const auto footModel = profile.links[feet[side]].modelOrientation.conjugate();
                const float shinYaw = yawOf(after.links[shins[side]].orientation * shinModel, { 1, 0, 0 });
                const float footYaw = yawOf(after.links[feet[side]].orientation * footModel, { 1, 0, 0 });
                result.legTwistFoot = std::max(result.legTwistFoot, wrapDeg(shinYaw - footYaw));
                result.legTwistPelvis = std::max(result.legTwistPelvis, wrapDeg(shinYaw - pelvisYaw));
                // Joelho para dentro: o joelho (fim da coxa) do lado de dentro
                // da linha quadril-tornozelo, no plano lateral da pelve.
                const Vec3 hip = after.links[thighs[side]].position, knee = after.links[shins[side]].position,
                    ankle = after.links[feet[side]].position;
                const Vec3 pelvisLeft = (after.links[0].orientation * profile.links[0].modelOrientation.conjugate()).rotate({ 0, 1, 0 });
                const float outward = side == 0 ? 1.0f : -1.0f;
                const Vec3 mid = (hip + ankle) * 0.5f;
                const float inward = -dot(knee - mid, pelvisLeft) * outward * 100.0f;
                result.kneeInward = std::max(result.kneeInward, inward);
            }
            const float lookYaw = scenario.script(t).lookYaw;
            result.pelvisLookLag = std::max(result.pelvisLookLag, wrapDeg(pelvisYaw - lookYaw));
            {
                const Vec3 pelvisLeft = (after.links[0].orientation * profile.links[0].modelOrientation.conjugate()).rotate({ 0, 1, 0 });
                const Vec3 l = after.links[feet[0]].position, r = after.links[feet[1]].position;
                const float separation = dot(Vec3 { l.x - r.x, l.y - r.y, 0.0f }, Vec3 { pelvisLeft.x, pelvisLeft.y, 0.0f }.normalized());
                if (separation < 0.0f) { ++result.crossTicks; result.crossWorst = std::max(result.crossWorst, -separation * 100.0f); }
                result.ankleSpreadWorst = std::max(result.ankleSpreadWorst, std::hypot(l.x - r.x, l.y - r.y));
                const float distance = std::hypot(l.x - r.x, l.y - r.y);
                result.feetMinDistance = std::min(result.feetMinDistance, distance);
                if (distance < 0.14f) ++result.closeTicks;
                {
                    const auto footForward = [&](std::size_t side) {
                        const Vec3 f = (after.links[feet[side]].orientation * profile.links[feet[side]].modelOrientation.conjugate())
                            .rotate({ 1, 0, 0 });
                        return Vec3 { f.x, f.y, 0.0f }.normalized();
                    };
                    const Vec3 base = (footForward(0) + footForward(1)).normalized();
                    const Vec3 baseLeft { -base.y, base.x, 0.0f };
                    const float baseSeparation = dot(Vec3 { l.x - r.x, l.y - r.y, 0.0f }, baseLeft);
                    if (baseSeparation < 0.08f) ++result.baseTangleTicks;
                    result.baseSeparationWorst = std::min(result.baseSeparationWorst, baseSeparation);
                }
            }
            {
                // Esfregar: o deslocamento liquido de cada apoio (de quando o
                // pe planta ate sair), sem somar a vibracao de cada tick.
                bool allPlanted = true;
                for (std::size_t side = 0; side < 2; ++side) {
                    const bool planted = locomotion.telemetry().footPlanted[side];
                    const Vec3 at = after.links[feet[side]].position;
                    const float drift = std::hypot(at.x - result.previousPlantedFoot[side].x,
                        at.y - result.previousPlantedFoot[side].y);
                    if (!planted && result.previousPlanted[side]) result.footScrub += drift;
                    if (planted && !result.previousPlanted[side]) result.previousPlantedFoot[side] = at;
                    if (planted && t + 1.5f / 120.0f >= scenario.seconds) result.footScrub += drift;
                    result.previousPlanted[side] = planted;
                    allPlanted = allPlanted && planted;
                }
                const Vec3 up = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({ 0, 0, 1 }));
                result.tiltWorst = std::max(result.tiltWorst, std::acos(std::clamp(up.z, -1.0f, 1.0f)) * 57.2958f);
                // Assentado: o giro parou com o rumo dentro da zona do olhar
                // (o corpo para 18 graus antes dele e nem gira abaixo de 31;
                // a pelve da base em alerta fica ~20 graus virada) e os dois
                // pes apoiados.
                if (!allPlanted || std::abs(locomotion.telemetry().turningRateRadiansPerSecond) > 0.05f
                    || wrapDeg(locomotion.telemetry().facingYawRadians - lookYaw) > 33.0f)
                    result.lastUnsettled = t;
                if (std::getenv("XGIRODBG") && tick % 30 == 0)
                    std::printf("GDBG %.2f rumo %+.1f olhar %+.1f pelve %+.1f plantados %d%d estado %d\n", t,
                        locomotion.telemetry().facingYawRadians * 57.2958f, lookYaw * 57.2958f, pelvisYaw * 57.2958f,
                        locomotion.telemetry().footPlanted[0] ? 1 : 0, locomotion.telemetry().footPlanted[1] ? 1 : 0,
                        static_cast<int>(locomotion.telemetry().state));
            }
        }
        if (const char* v = std::getenv("XPARADOTRACE"); v != nullptr && t >= std::strtof(v, nullptr) && tick % 2 == 0) {
            const auto& tel = locomotion.telemetry();
            const Vec3 f = after.links[0].orientation.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 com = locomotion.physicalState().centerOfMassVelocityWorld;
            std::printf("GIROTRACE %.2f | modo %d torcao %+.1f falta %+.1f | olhar %+.1f pelve %+.1f | parado %d\n", t,
                locomotion.contactPlan().turnStance ? 1 : 0, locomotion.contactPlan().standingTwist * 57.2958f,
                locomotion.contactPlan().standingTurnRemaining * 57.2958f, scenario.script(t).lookYaw * 57.2958f,
                std::atan2(f.y, f.x) * 57.2958f, locomotion.telemetry().footworkStanding ? 1 : 0);
            std::printf("PARADO %.2f | ref (%.3f %.3f %.3f) v %.2f estado %d | captura fora %.3f | com v %.2f | pelve yaw %+.1f | cheia %.2f pert %.2f env %.2f | inclina %.1f | planejado (%d %d)\n",
                t, output.guide.rootPositionWorld.x, output.guide.rootPositionWorld.y, output.guide.rootPositionWorld.z,
                output.guide.rootLinearVelocityWorld.length(), static_cast<int>(tel.state),
                tel.captureOutsideMeters, std::hypot(com.x, com.y), std::atan2(f.y, f.x) * 57.2958f,
                0.0f, controller.output().perturbation, controller.output().assistEnvelope,
                controller.output().tiltErrorDegrees, tel.footPlanted[0] ? 1 : 0, tel.footPlanted[1] ? 1 : 0);
        }
        if (scenario.responseAt >= 0.0f && t >= scenario.responseAt && result.fluResponse < 0.0f) {
            const Vec3 v { after.links.front().linearVelocity.x, after.links.front().linearVelocity.y, 0.0f };
            bool reached = false;
            if (scenario.responseStop) {
                reached = v.length() < 0.3f;
            } else {
                const float speed = scenario.responseSprint ? settings.sprintSpeed : settings.walkSpeed;
                reached = dot(v, scenario.responseDirection) >= 0.9f * speed;
            }
            if (reached) result.fluResponse = t - scenario.responseAt;
        }
        if (scenario.postureFrom >= 0.0f && t >= scenario.postureFrom && t <= scenario.postureTo) {
            for (std::size_t side = 0; side < 2; ++side) {
                const char* shinId = side == 0 ? "LeftShin" : "RightShin";
                std::size_t shin = 0;
                for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == shinId) shin = i;
                const float measured = after.joints[shin].positionRadians[0] * 57.2958f;
                float target = measured;
                for (const auto& drive : output.driveTargets)
                    if (drive.linkIndex == shin && drive.axis == static_cast<RagdollAxis3D>(0)) target = drive.positionRadians * 57.2958f;
                result.postureKneeAllSum += measured;
                result.postureKneeTargetAllSum += target;
                const bool supporting = locomotion.physicalState().feet[side].supporting;
                {
                    auto& step = result.postureStep[side];
                    const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                    if (supporting && !step.on) {
                        step = { true, t, measured, measured, t, v };
                        {
                            const auto& ps = locomotion.physicalState();
                            const Vec3 hv { v.x, v.y, 0.0f };
                            const float hs = hv.length();
                            const Vec3 dir = hs > 0.5f ? hv * (1.0f / hs) : Vec3 { 1.0f, 0.0f, 0.0f };
                            step.aheadAtTouch = dot(Vec3 { ps.feet[side].soleCenterWorld.x - ps.centerOfMassWorld.x,
                                ps.feet[side].soleCenterWorld.y - ps.centerOfMassWorld.y, 0.0f }, dir);
                            std::size_t thighIndex = 0;
                            for (std::size_t i = 0; i < profile.links.size(); ++i)
                                if (profile.links[i].id == (side == 0 ? "LeftThigh" : "RightThigh")) thighIndex = i;
                            step.hipAtTouch = after.joints[thighIndex].positionRadians[1] * 57.2958f;
                            step.comHeightAtTouch = ps.centerOfMassWorld.z;
                        }
                    } else if (supporting) {
                        if (measured > step.maxKnee) { step.maxKnee = measured; step.maxKneeTime = t; }
                    } else if (step.on) {
                        step.on = false;
                        const float seconds = t - step.touchTime;
                        if (seconds > 0.03f) {
                            const Vec3 h0 { step.velocityAtTouch.x, step.velocityAtTouch.y, 0.0f };
                            const float speed0 = h0.length();
                            const Vec3 dir = speed0 > 0.5f ? h0 * (1.0f / speed0) : Vec3 { 1.0f, 0.0f, 0.0f };
                            const float dvz = v.z - step.velocityAtTouch.z;
                            const float dvh = dot(Vec3 { v.x, v.y, 0.0f }, dir) - speed0;
                            ++result.postureSteps;
                            result.postureStepDvz += dvz;
                            result.postureStepDvh += dvh;
                            result.postureStepSeconds += seconds;
                            result.postureAbsorbSeconds += step.maxKneeTime - step.touchTime;
                            result.postureAbsorbDegrees += step.maxKnee - step.kneeAtTouch;
                            result.posturePushSeconds += t - step.maxKneeTime;
                            result.postureStepDvzList.push_back(dvz);
                            if (std::getenv("XSTEPLOG"))
                                std::printf("STEP %.3f %s %3.0f ms | toque: pe a frente %+.2f m quadril %+5.1f cm %.3f vz %+.2f v %.2f | joelho %+4.1f -> max %4.1f | dvz %+.2f dvh %+.2f\n",
                                    t, side == 0 ? "E" : "D", 1000.0f * seconds, step.aheadAtTouch, step.hipAtTouch,
                                    step.comHeightAtTouch, step.velocityAtTouch.z, speed0, step.kneeAtTouch, step.maxKnee, dvz, dvh);
                        }
                    }
                }
                if (!supporting && result.postureWasSupporting[side]) {
                    result.postureLiftKneeSum += measured;
                    ++result.postureLifts;
                }
                if (supporting && !result.postureWasSupporting[side]) {
                    result.postureTouchKneeSum += measured;
                    result.postureTouchPelvisSum += after.links[0].position.z;
                    ++result.postureTouches;
                }
                result.postureWasSupporting[side] = supporting;
                {
                    const bool clipStance = locomotion.telemetry().clipFootContact[side] > 0.55f;
                    if (clipStance && !result.postureClipStance[side]) {
                        result.postureClipStanceStart[side] = t;
                        result.postureStanceTouched[side] = supporting;
                        result.postureTouchedSeconds[side] = 0.0f;
                        if (!supporting) ++result.postureLateTouches;
                    }
                    if (clipStance && supporting) {
                        if (!result.postureStanceTouched[side])
                            result.postureTouchDelaySum += t - result.postureClipStanceStart[side];
                        result.postureStanceTouched[side] = true;
                        result.postureTouchedSeconds[side] += dt;
                    }
                    if (!clipStance && result.postureClipStance[side]) {
                        ++result.postureClipStances;
                        if (!result.postureStanceTouched[side]) ++result.postureMissedStances;
                        const float length = t - result.postureClipStanceStart[side];
                        if (length > 0.0f) result.postureTouchedShareSum += result.postureTouchedSeconds[side] / length;
                    }
                    result.postureClipStance[side] = clipStance;
                    const bool planted = locomotion.telemetry().footPlanted[side];
                    if (planted && !result.posturePlanted[side]) {
                        result.posturePlantSupported[side] = false;
                        result.posturePlantSupport[side] = 0.0f;
                    }
                    if (planted && supporting) {
                        result.posturePlantSupported[side] = true;
                        result.posturePlantSupport[side] += dt;
                    }
                    if (!planted && result.posturePlanted[side]) {
                        ++result.posturePlants;
                        if (!result.posturePlantSupported[side]) ++result.posturePlantsMissed;
                        result.posturePlantSupportSum += result.posturePlantSupport[side];
                    }
                    result.posturePlanted[side] = planted;
                }
                if (supporting) {
                    for (const auto& drive : output.driveTargets)
                        if (drive.linkIndex == shin && drive.axis == static_cast<RagdollAxis3D>(0)) {
                            result.postureStiffnessSum += drive.stiffnessScale;
                            result.postureTorqueScaleSum += drive.maximumTorqueScale;
                        }
                    const float motor = std::abs(after.joints[shin].motorTorqueNewtonMeters[0]);
                    const float limit = after.joints[shin].motorTorqueLimitNewtonMeters[0];
                    result.postureMotorSum += motor;
                    result.postureLimitSum += limit;
                    if (motor > 0.95f * limit) ++result.postureSaturated;
                    result.postureKneeStanceSum += measured;
                    result.postureKneeStanceTargetSum += target;
                    result.postureKneeStanceMin = std::min(result.postureKneeStanceMin, measured);
                    result.postureKneeStanceMax = std::max(result.postureKneeStanceMax, measured);
                    ++result.postureStanceSamples;
                }
            }
            if (std::getenv("XPOSTRACE4")) {
                const auto& ps = locomotion.physicalState();
                const auto& tel = locomotion.telemetry();
                const Vec3 hv { ps.centerOfMassVelocityWorld.x, ps.centerOfMassVelocityWorld.y, 0.0f };
                const float hs = hv.length();
                const Vec3 dir = hs > 0.5f ? hv * (1.0f / hs) : Vec3 { 1.0f, 0.0f, 0.0f };
                std::printf("PF %.3f fase %.3f espera %.2f lead %.2f %.2f |", t, tel.cyclePhase, tel.touchdownWaitSeconds,
                    tel.legPhaseLead[0], tel.legPhaseLead[1]);
                for (std::size_t side = 0; side < 2; ++side) {
                    const float ahead = dot(Vec3 { ps.feet[side].soleCenterWorld.x - ps.centerOfMassWorld.x,
                        ps.feet[side].soleCenterWorld.y - ps.centerOfMassWorld.y, 0.0f }, dir);
                    const float targetAhead = tel.footTrace[side].valid
                        ? dot(Vec3 { tel.footTrace[side].target.x - ps.centerOfMassWorld.x,
                            tel.footTrace[side].target.y - ps.centerOfMassWorld.y, 0.0f }, dir) : 0.0f;
                    const Vec3 left { -dir.y, dir.x, 0.0f };
                    const float side_ = dot(Vec3 { ps.feet[side].soleCenterWorld.x - ps.centerOfMassWorld.x,
                        ps.feet[side].soleCenterWorld.y - ps.centerOfMassWorld.y, 0.0f }, left);
                    const float targetSide = tel.footTrace[side].valid
                        ? dot(Vec3 { tel.footTrace[side].target.x - ps.centerOfMassWorld.x,
                            tel.footTrace[side].target.y - ps.centerOfMassWorld.y, 0.0f }, left) : 0.0f;
                    std::printf(" %s %s%s clipe %.1f prog %+.2f frente %+.2f alvo %+.2f lado %+.2f alvo %+.2f z %.2f alvo %.2f ik %.2f raiz %.2f/%.2f |", side == 0 ? "E" : "D",
                        ps.feet[side].supporting ? "A" : "-", tel.footPlanted[side] ? "T" : "-",
                        tel.clipFootContact[side], tel.footSwingProgress[side], ahead, targetAhead, side_, targetSide,
                        ps.feet[side].soleLowestHeight, tel.footTrace[side].valid ? tel.footTrace[side].target.z : -1.0f,
                        tel.footTrace[side].ik.z, tel.footTrace[side].ikRoot.z, tel.footTrace[side].physicalRoot.z);
                }
                {
                    const auto tilt = [](const Quaternion& q) {
                        const Vec3 f = q.rotate(Vec3 { 1.0f, 0.0f, 0.0f });
                        const Vec3 l = q.rotate(Vec3 { 0.0f, 1.0f, 0.0f });
                        return std::pair<float, float> { std::asin(std::clamp(f.z, -1.0f, 1.0f)) * 57.2958f,
                            std::asin(std::clamp(l.z, -1.0f, 1.0f)) * 57.2958f };
                    };
                    const auto ik = tilt(tel.footTrace[0].ikRootOrientation);
                    const auto ph = tilt(tel.footTrace[0].physicalRootOrientation);
                    std::size_t chest = 0;
                    for (std::size_t i = 0; i < profile.links.size(); ++i)
                        if (profile.links[i].id == "UpperChest") chest = i;
                    const auto tr = tilt(after.links[chest].orientation);
                    std::printf(" pelve inclina frente %+.0f/%+.0f lado %+.0f/%+.0f tronco lado %+.0f", ik.first, ph.first, ik.second, ph.second, tr.second);
                }
                std::printf("\n");
            }
            if (std::getenv("XPOSTRACE3")) {
                // Quem empurra o corpo para cima: ajuda na pelve (aceleracao),
                // peso pelas juntas (N) e a forca de contato medida nos pes.
                const auto& pf = locomotion.physicalState().feet;
                std::printf("PV %.3f pelve %.3f guia %.3f vz %+.2f | E %s%s D %s%s | ajuda %+5.1f m/s2 juntas %4.0f N | carga E %4.0f D %4.0f N | empuxo %+.3f\n",
                    t, after.links[0].position.z, output.guide.rootPositionWorld.z,
                    locomotion.physicalState().centerOfMassVelocityWorld.z,
                    pf[0].supporting ? "A" : "-", locomotion.telemetry().footPlanted[0] ? "T" : "-",
                    pf[1].supporting ? "A" : "-", locomotion.telemetry().footPlanted[1] ? "T" : "-",
                    controller.output().supportAcceleration, controller.output().jointSupportNewtons,
                    pf[0].loadNewtons, pf[1].loadNewtons, locomotion.telemetry().thrustOffsetMeters);
            }
            if (std::getenv("XPOSTRACE2")) {
                // Empurrao: joelho de cada pe com motor, rigidez, trava e clipe.
                const auto& tel = locomotion.telemetry();
                std::printf("PK %.3f v %.2f fase %.3f espera %.2f lead %.2f %.2f pelve %.3f vz %+.2f", t,
                    std::hypot(after.links[0].linearVelocity.x, after.links[0].linearVelocity.y), tel.cyclePhase,
                    tel.touchdownWaitSeconds, tel.legPhaseLead[0], tel.legPhaseLead[1], after.links[0].position.z, after.links[0].linearVelocity.z);
                for (std::size_t side = 0; side < 2; ++side) {
                    std::size_t shin = 0, thigh = 0, foot = 0;
                    for (std::size_t i = 0; i < profile.links.size(); ++i) {
                        if (profile.links[i].id == (side == 0 ? "LeftShin" : "RightShin")) shin = i;
                        if (profile.links[i].id == (side == 0 ? "LeftThigh" : "RightThigh")) thigh = i;
                        if (profile.links[i].id == (side == 0 ? "LeftFoot" : "RightFoot")) foot = i;
                    }
                    float target = 0, stiff = 0, ankleT = 0, hipT = 0;
                    for (const auto& drive : output.driveTargets) {
                        if (drive.linkIndex == shin && drive.axis == static_cast<RagdollAxis3D>(0)) { target = drive.positionRadians * 57.2958f; stiff = drive.stiffnessScale; }
                        if (drive.linkIndex == foot && drive.axis == static_cast<RagdollAxis3D>(0)) ankleT = drive.positionRadians * 57.2958f;
                        if (drive.linkIndex == thigh && drive.axis == static_cast<RagdollAxis3D>(1)) hipT = drive.positionRadians * 57.2958f;
                    }
                    std::printf(" | %s %s%s%s clipe %.1f joelho %5.1f/%5.1f motor %4.0f/%3.0f rig %.2f quadril %5.1f/%5.1f m %4.0f/%3.0f ff %4.0f tornoz %5.1f/%5.1f",
                        side == 0 ? "E" : "D", locomotion.physicalState().feet[side].supporting ? "A" : "-",
                        tel.footPlanted[side] ? "T" : "-", tel.footSwingProgress[side] >= 0.0f ? "B" : "-",
                        tel.clipFootContact[side], after.joints[shin].positionRadians[0] * 57.2958f, target,
                        after.joints[shin].motorTorqueNewtonMeters[0], after.joints[shin].motorTorqueLimitNewtonMeters[0], stiff,
                        after.joints[thigh].positionRadians[1] * 57.2958f, hipT, after.joints[thigh].motorTorqueNewtonMeters[1], after.joints[thigh].motorTorqueLimitNewtonMeters[1], after.joints[thigh].feedforwardTorqueNewtonMeters[1], after.joints[foot].positionRadians[0] * 57.2958f, ankleT);
                }
                std::printf("\n");
            }
            if (std::getenv("XPOSTRACE") && tick % 3 == 0) {
                const auto& tel = locomotion.telemetry();
                float kneeM[2], kneeT[2];
                for (std::size_t side = 0; side < 2; ++side) {
                    std::size_t shin = 0;
                    for (std::size_t i = 0; i < profile.links.size(); ++i)
                        if (profile.links[i].id == (side == 0 ? "LeftShin" : "RightShin")) shin = i;
                    kneeM[side] = after.joints[shin].positionRadians[0] * 57.2958f;
                    kneeT[side] = kneeM[side];
                    for (const auto& drive : output.driveTargets)
                        if (drive.linkIndex == shin && drive.axis == static_cast<RagdollAxis3D>(0)) kneeT[side] = drive.positionRadians * 57.2958f;
                }
                std::printf("PT %.2f v %.2f pelve %.3f guia %.3f | E %d %5.1f/%5.1f D %d %5.1f/%5.1f | curva %.3f freada %.3f alcance %+.3f | sust %.1f ff %.0f N | estado %d\n",
                    t, std::hypot(after.links[0].linearVelocity.x, after.links[0].linearVelocity.y),
                    after.links[0].position.z, output.guide.rootPositionWorld.z,
                    locomotion.physicalState().feet[0].supporting ? 1 : 0, kneeM[0], kneeT[0],
                    locomotion.physicalState().feet[1].supporting ? 1 : 0, kneeM[1], kneeT[1],
                    tel.pelvisDropCurveMeters, tel.pelvisDropBrakeMeters, tel.pelvisDropReachMeters,
                    controller.output().supportAcceleration, controller.output().jointSupportNewtons, static_cast<int>(tel.state));
            }
            {
                const bool anySupport = locomotion.physicalState().feet[0].supporting
                    || locomotion.physicalState().feet[1].supporting;
                if (!anySupport) {
                    result.postureFlightTop = result.postureFlying
                        ? std::max(result.postureFlightTop, after.links[0].position.z) : after.links[0].position.z;
                    result.postureFlying = true;
                } else if (result.postureFlying) {
                    result.postureFlying = false;
                    result.postureFlightApex.push_back(result.postureFlightTop);
                }
            }
            {
                // Ombro (eixo 1, abaixar/levantar; eixo 2, frente/tras), cotovelo,
                // quadril (flexao) e joelho dos dois lados: alvo e medido.
                struct Probe { const char* id; int axis; bool arm; };
                static const Probe probes[8] = { { "LeftUpperArm", 2, true }, { "RightUpperArm", 2, true },
                    { "LeftForearm", 0, true }, { "RightForearm", 0, true }, { "LeftThigh", 1, false },
                    { "RightThigh", 1, false }, { "LeftShin", 0, false }, { "RightShin", 0, false } };
                for (int k = 0; k < 8; ++k) {
                    std::size_t link = 0;
                    for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == probes[k].id) link = i;
                    float target = after.joints[link].positionRadians[probes[k].axis];
                    for (const auto& drive : output.driveTargets)
                        if (drive.linkIndex == link && drive.axis == static_cast<RagdollAxis3D>(probes[k].axis)) target = drive.positionRadians;
                    const float values[2] = { target, after.joints[link].positionRadians[probes[k].axis] };
                    auto& st = result.postureSmoothState[k];
                    for (int j = 0; j < 2; ++j) {
                        const float velocity = st.primed ? (values[j] - st.previous[j]) / dt : 0.0f;
                        if (st.count >= 2) {
                            const float acc = (velocity - st.velocity[j]) / dt;
                            if (probes[k].arm) (j == 0 ? result.postureArmTargetAcc : result.postureArmMeasuredAcc) += acc * acc;
                            else (j == 0 ? result.postureLegTargetAcc : result.postureLegMeasuredAcc) += acc * acc;
                        }
                        st.velocity[j] = velocity;
                        st.previous[j] = values[j];
                    }
                    st.primed = true;
                    ++st.count;
                    if (st.count >= 3) { if (probes[k].arm) ++result.postureArmSamples; else ++result.postureLegSamples; }
                }
                const float phase = locomotion.telemetry().cyclePhase;
                if (result.posturePreviousPhase >= 0.0f) {
                    float step = phase - result.posturePreviousPhase;
                    if (step < -0.5f) step += 1.0f;
                    const float rate = step / dt;
                    if (result.posturePhaseSamples > 0) result.posturePhaseRateChange += std::abs(rate - result.posturePreviousPhaseRate);
                    result.posturePreviousPhaseRate = rate;
                    ++result.posturePhaseSamples;
                }
                result.posturePreviousPhase = phase;
            }
            result.posturePelvisSum += after.links[0].position.z;
            result.posturePelvisSquares += after.links[0].position.z * after.links[0].position.z;
            result.postureGuideSum += output.guide.rootPositionWorld.z;
            result.postureDropCurve += locomotion.telemetry().pelvisDropCurveMeters;
            result.postureDropBrake += locomotion.telemetry().pelvisDropBrakeMeters;
            result.postureDropReach += locomotion.telemetry().pelvisDropReachMeters;
            result.postureDropLanding += locomotion.telemetry().pelvisDropLandingMeters;
            result.postureSupport += controller.output().supportAcceleration;
            result.postureUpright += controller.output().uprightAuthorityShare;
            result.postureEnvelope += controller.output().assistEnvelope;
            result.postureLegShare += controller.output().legShare;
            ++result.postureSamples;
        }
        result.bodilyMaxStage = std::max(result.bodilyMaxStage,
            static_cast<int>(locomotion.telemetry().bodilyStage));
        result.getUpAttemptsMax = std::max(result.getUpAttemptsMax,
            static_cast<int>(locomotion.telemetry().getUpAttempt));
        if (result.risingFirstAt < 0 && locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Rising
            && !locomotion.telemetry().tackling)
            result.risingFirstAt = t;
        {
            const auto& tel = locomotion.telemetry();
            const Vec3 pelvisNow = after.links.front().position;
            if (tel.tackling && std::getenv("XCARTRACE") && tick % 6 == 0) {
                const Vec3 up = after.links.front().orientation.rotate({0, 0, 1});
                const Vec3 fw = after.links.front().orientation.rotate({1, 0, 0});
                const Vec3 tup = output.guide.rootOrientationWorld.rotate({0, 0, 1});
                const Vec3 tfw = output.guide.rootOrientationWorld.rotate({1, 0, 0});
                std::printf("CAR %.2f clipe %.2f | pelve %.2f alvo %.2f | cima (%+.2f %+.2f %+.2f) alvo (%+.2f %+.2f %+.2f) | frente (%+.2f %+.2f %+.2f) alvo (%+.2f %+.2f %+.2f) | v %.1f | F %.0f N\n",
                    t, tel.getUpProgress, pelvisNow.z, output.guide.rootPositionWorld.z, up.x, up.y, up.z, tup.x, tup.y, tup.z,
                    fw.x, fw.y, fw.z, tfw.x, tfw.y, tfw.z, std::hypot(after.links.front().linearVelocity.x, after.links.front().linearVelocity.y),
                    tel.getUpAssistNewtons);
            }
            if (tel.tacklePending && result.tackleArmedAt < 0) result.tackleArmedAt = t;
            if (tel.tackling) {
                if (result.tackleStartedAt < 0) {
                    result.tackleStartedAt = t;
                    result.tackleVariant = tel.tackleVariant;
                    result.tackleEntrySpeed = tel.tackleEntrySpeed;
                    result.tackleStartPelvis = pelvisNow;
                }
                const Vec3 dir { std::cos(wanted.lookYaw), std::sin(wanted.lookYaw), 0.0f };
                const Vec3 d { pelvisNow.x - result.tackleStartPelvis.x, pelvisNow.y - result.tackleStartPelvis.y, 0.0f };
                result.tackleTravel = std::max(result.tackleTravel, dot(d, dir));
                result.tackleLateral = std::max(result.tackleLateral, std::abs(d.x * dir.y - d.y * dir.x));
                result.tackleMinPelvis = std::min(result.tackleMinPelvis, pelvisNow.z);
                if (pelvisNow.z < 0.35f) {
                    if (result.slideOnGround) {
                        result.slideGroundDistance += std::hypot(pelvisNow.x - result.slidePrevious.x,
                            pelvisNow.y - result.slidePrevious.y);
                        result.slideGroundSeconds += dt;
                    } else if (result.slideLandSpeed < 0) {
                        result.slideLandSpeed = std::hypot(after.links.front().linearVelocity.x,
                            after.links.front().linearVelocity.y);
                    }
                    result.slideOnGround = true;
                    result.slidePrevious = pelvisNow;
                } else {
                    result.slideOnGround = false;
                }
                for (const auto& link : after.links)
                    result.tackleWorstLinkSpeed = std::max(result.tackleWorstLinkSpeed, link.linearVelocity.length());
            } else if (result.tackleStartedAt >= 0 && result.tackleEndedAt < 0) {
                result.tackleEndedAt = t;
            }
            if (result.tackleEndedAt >= 0 && result.afterTackleSpeed < 0 && t - result.tackleEndedAt >= 0.5f)
                result.afterTackleSpeed = std::hypot(after.links.front().linearVelocity.x, after.links.front().linearVelocity.y);
        }
        if (std::getenv("XGUTRACE") && tick % 6 == 0
            && locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Rising) {
            // Levantar: progresso do clipe, maos e pes (altura), e o centro de
            // massa contra os pes e as maos no rumo do corpo (+ a frente).
            const auto& tel = locomotion.telemetry();
            const auto linkOf = [&](const char* id) {
                for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == id) return i;
                return std::size_t { 0 };
            };
            const Vec3 com = locomotion.physicalState().centerOfMassWorld;
            const Vec3 fwd3 = after.links[0].orientation.rotate({ 1, 0, 0 });
            Vec3 fwd { fwd3.x, fwd3.y, 0 };
            if (fwd.length() < 0.2f) { const Vec3 up3 = after.links[0].orientation.rotate({ 0, 0, 1 }); fwd = { up3.x, up3.y, 0 }; }
            fwd = fwd.normalized();
            const Vec3 lf = after.links[linkOf("LeftFoot")].position, rf = after.links[linkOf("RightFoot")].position;
            const Vec3 lh = after.links[linkOf("LeftHand")].position, rh = after.links[linkOf("RightHand")].position;
            const Vec3 feet = (lf + rf) * 0.5f, hands = (lh + rh) * 0.5f;
            std::printf("GU %.2f prog %.2f ritmo %.2f | pelve %.2f peito %.2f | maos z %.2f %.2f pes z %.2f %.2f | cm-pes %+.2f cm-maos %+.2f | ajuda %.0f N\n",
                t, tel.getUpProgress, tel.getUpRate, after.links[0].position.z,
                after.links[linkOf("Chest")].position.z, lh.z, rh.z, lf.z, rf.z,
                dot(Vec3 { com.x - feet.x, com.y - feet.y, 0 }, fwd), dot(Vec3 { com.x - hands.x, com.y - hands.y, 0 }, fwd),
                tel.getUpAssistNewtons);
        }
        {
            // Levantar: a duracao da subida e o tempo "de pe e inclinado"
            // (pelve acima de 0,8 m com ela ou o peito alem do clipe).
            const auto& tel = locomotion.telemetry();
            if (tel.getUpPhase == CharacterGetUpPhase3D::Rising) result.risingEndAt = t;
            // Pulo no chao: caido ou levantando, nenhum link tocando o chao
            // (o tempo), e a maior velocidade de subida da pelve e de um link
            // no comeco da subida (0,8 s).
            if (tel.getUpPhase != CharacterGetUpPhase3D::None) {
                bool touching = false;
                for (const auto& contact : after.interactions) if (contact.normalWorld.z > 0.5f) touching = true;
                if (!touching && tel.getUpPhase == CharacterGetUpPhase3D::Rising) result.getUpAirSeconds += dt;
                if (std::getenv("XGUAR") && tick % 3 == 0)
                    std::printf("  GUAR t %.3f fase %d prog %.2f ritmo %.2f | pelve z %.2f vz %+.2f | toca %d | ajuda %.0f N | contatos %zu\n", t,
                        static_cast<int>(tel.getUpPhase), tel.getUpProgress, tel.getUpRate, after.links[0].position.z,
                        after.links[0].linearVelocity.z, touching ? 1 : 0, tel.getUpAssistNewtons, after.interactions.size());
                if (tel.getUpPhase == CharacterGetUpPhase3D::Rising) {
                    if (result.risingStartedAt < 0.0f) result.risingStartedAt = t;
                    if (t - result.risingStartedAt < 0.8f) {
                        result.risingPelvisRisePeak = std::max(result.risingPelvisRisePeak, after.links[0].linearVelocity.z);
                        for (const auto& link : after.links)
                            result.risingLinkSpeedPeak = std::max(result.risingLinkSpeedPeak, link.linearVelocity.length());
                    }
                } else {
                    result.risingStartedAt = -1.0f;
                }
            }
            if (tel.getUpPhase == CharacterGetUpPhase3D::Rising || (result.risingEndAt >= 0.0f && t - result.risingEndAt < 1.0f)) {
                if (tel.getUpPhase == CharacterGetUpPhase3D::Rising) result.risingSeconds += dt;
                result.risingPelvisPeak = std::max(result.risingPelvisPeak, after.links[0].position.z);
                std::size_t upper = 0;
                for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == "UpperChest") upper = i;
                const Vec3 pelvisUp = after.links[0].orientation.rotate({ 0, 0, 1 });
                const Vec3 chestUp = (after.links[upper].orientation * profile.links[upper].modelOrientation.conjugate())
                    .rotate({ 0, 0, 1 });
                const float pelvisTilt = std::acos(std::clamp(pelvisUp.z, -1.0f, 1.0f)) * 57.3f;
                const float chestTilt = std::acos(std::clamp(chestUp.z, -1.0f, 1.0f)) * 57.3f;
                if (after.links[0].position.z > 0.8f && (pelvisTilt > 20.0f || chestTilt > 35.0f))
                    result.hunchedSeconds += dt;
            }
        }
        if (result.handoffAt < 0.0f && locomotion.telemetry().getUpHandoffReady) {
            result.handoffAt = t;
            result.handoffState = after;
        }
        if (std::getenv("XGU2") && tick % 6 == 0) {
            // Levantar e o que vem depois: o estado, o progresso e o ritmo do
            // clipe, a pelve (altura, inclinacao) e o peito (inclinacao).
            static float risingEnded = -10.0f;
            const auto& tel = locomotion.telemetry();
            const bool rising = tel.getUpPhase == CharacterGetUpPhase3D::Rising;
            if (rising) risingEnded = t;
            if (rising || t - risingEnded < 2.0f) {
                std::size_t upper = 0;
                for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == "UpperChest") upper = i;
                const Vec3 pelvisUp = after.links[0].orientation.rotate({ 0, 0, 1 });
                const Vec3 chestUp = (after.links[upper].orientation * profile.links[upper].modelOrientation.conjugate())
                    .rotate({ 0, 0, 1 });
                if (std::getenv("XGU2SPINE")) {
                    for (const auto& target : output.driveTargets) {
                        if (static_cast<int>(target.axis) != 1 || target.linkIndex >= profile.links.size()) continue;
                        const std::string& id = profile.links[target.linkIndex].id;
                        if (id != "Abdomen" && id != "Chest" && id != "UpperChest" && id != "LeftThigh") continue;
                        std::printf("    %s alvo %+.1f medido %+.1f rig %.2f torque %.2f\n", id.c_str(), target.positionRadians * 57.3f,
                            after.joints[target.linkIndex].positionRadians[1] * 57.3f, target.stiffnessScale, target.maximumTorqueScale);
                    }
                }
                std::printf("GU2 t %.2f estado %d fase %d prog %.2f ritmo %.2f | pelve z %.2f incl %4.1f (clipe %4.1f) | peito incl %4.1f (clipe %4.1f)\n", t,
                    static_cast<int>(tel.state), static_cast<int>(tel.getUpPhase), tel.getUpProgress, tel.getUpRate,
                    after.links[0].position.z, std::acos(std::clamp(pelvisUp.z, -1.0f, 1.0f)) * 57.3f, tel.getUpExpectedTiltDegrees,
                    std::acos(std::clamp(chestUp.z, -1.0f, 1.0f)) * 57.3f, tel.getUpExpectedChestTiltDegrees);
            }
        }
        if (std::getenv("XGUTRACE") && tick % 12 == 0 && locomotion.telemetry().getUpRolling)
            std::printf("ROLA %.2f\n", t);
        if (std::getenv("XBODTRACE") && tick % 12 == 0 && locomotion.telemetry().bodilyStage != BodilyGetUpStage3D::None) {
            std::printf("BOD %.2f etapa %d prog %.2f erro %.3f pelve %.2f\n", t,
                static_cast<int>(locomotion.telemetry().bodilyStage), locomotion.telemetry().bodilyStageProgress,
                locomotion.telemetry().bodilyRootError, after.links[0].position.z);
        }
        if (poseFile && tick % poseEvery == 0) {
            dumpPose(poseFile, t, static_cast<int>(locomotion.telemetry().state),
                static_cast<int>(locomotion.telemetry().bodilyStage), profile, after);
            if (scenario.ball && scene->contains(*scenario.ball)) {
                const InflatedBallState3D ball = scene->inflatedBallState(*scenario.ball);
                std::fprintf(poseFile, "C Ball %.4f %.4f %.4f 0 0 0 1 0.0 0.11 0.11\n",
                    ball.position.x, ball.position.y, ball.position.z);
            }
            // Fantasma: a pose pedida pelo levantar pelo corpo (alvos das
            // juntas na pelve desejada).
            const auto& bodily = locomotion.bodilyGetUp();
            if (locomotion.telemetry().bodilyStage != BodilyGetUpStage3D::None
                && bodily.targetCoordinates().size() == profile.links.size()) {
                RagdollAnimationPose3D ghost;
                buildRagdollLocalPose3D(profile, bodily.targetCoordinates(), ghost);
                RagdollState3D ghostState = after;
                for (std::size_t i = 0; i < ghostState.links.size(); ++i) {
                    ghostState.links[i].position = bodily.desiredRootPosition()
                        + bodily.desiredRootOrientation().rotate(ghost.linkPositions[i]);
                    ghostState.links[i].orientation = (bodily.desiredRootOrientation() * ghost.linkOrientations[i]).normalized();
                }
                ghostState.interactions.clear();
                std::fprintf(poseFile, "G\n");
                dumpPose(poseFile, t, -1, -1, profile, ghostState);
            }
        }
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
        if (std::getenv("XBRAKETRACE") && tick % (std::getenv("XFINE") ? 1 : 3) == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 4.8f)
                && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 6.5f)) {
            // Freada: no rumo da velocidade do corpo, a posicao de cada pe e
            // do ponto de captura contra o centro de massa (m, + a frente),
            // a inclinacao da pelve e do peito (graus, + para a frente), o
            // estado, a capsula, a ajuda na pelve e o plano dos pes.
            const auto& ps = locomotion.physicalState();
            const auto& plan = locomotion.contactPlan();
            Vec3 v = ps.centerOfMassVelocityWorld; v.z = 0;
            const float speedNow = v.length();
            const Vec3 travel = result.gaitLastRequest.lengthSquared() > 1e-6f
                ? Vec3{result.gaitLastRequest.x, result.gaitLastRequest.y, 0}.normalized() : Vec3{1, 0, 0};
            const Vec3 com = ps.centerOfMassWorld;
            const auto ahead = [&](Vec3 p) { return dot(Vec3{p.x - com.x, p.y - com.y, 0}, travel); };
            const auto pitchOf = [&](Quaternion q) {
                const Vec3 up = q.rotate({0, 0, 1});
                return std::asin(std::clamp(dot(up, travel), -1.0f, 1.0f)) * 57.3f;
            };
            const float capture = dot(v, travel) / std::max(0.1f, ps.captureOmega);
            std::printf("REF %.3f fase-freada %d assentado %d ref-corpo %+.2f %+.2f capsula-corpo %+.2f %+.2f\n", t, locomotion.telemetry().brakePhase, locomotion.contactPlan().brakeSettled ? 1 : 0,
                output.guide.rootPositionWorld.x - after.links[0].position.x, output.guide.rootPositionWorld.y - after.links[0].position.y,
                capsule.position.x - after.links[0].position.x, capsule.position.y - after.links[0].position.y);
            if (std::getenv("XLATTRACE")) {
                // Lateral (m, + a esquerda do rumo): pes (* = apoio), captura e
                // velocidade do centro de massa, rolagem da pelve e do peito
                // (graus, + a esquerda para baixo... sinal do up . esquerda).
                const Vec3 left { -travel.y, travel.x, 0 };
                const auto lat = [&](Vec3 p) { return dot(Vec3{p.x - com.x, p.y - com.y, 0}, left); };
                const auto rollOf = [&](Quaternion q) {
                    const Vec3 up = q.rotate({0, 0, 1});
                    return std::asin(std::clamp(dot(up, left), -1.0f, 1.0f)) * 57.3f;
                };
                const float vl = dot(v, left);
                std::printf("LAT %.3f pe %+.2f%s %+.2f%s | cap %+.2f vlat %+.2f | rol pelve %+5.1f peito %+5.1f | frente pe %+.2f %+.2f cap %+.2f | com z %.2f vz %+.2f Fz %.0f T %.0f %.0f\n", t,
                    lat(after.links[feet[0]].position), ps.feet[0].supporting ? "*" : " ",
                    lat(after.links[feet[1]].position), ps.feet[1].supporting ? "*" : " ",
                    vl / std::max(0.1f, ps.captureOmega), vl,
                    rollOf(after.links[0].orientation * profile.links[0].modelOrientation.conjugate()),
                    rollOf(after.links[chest].orientation * profile.links[chest].modelOrientation.conjugate()),
                    ahead(after.links[feet[0]].position), ahead(after.links[feet[1]].position), capture, com.z,
                    ps.centerOfMassVelocityWorld.z, physical.rootForceWorld.z,
                    std::hypot(physical.rootTorqueWorld.x, physical.rootTorqueWorld.y), physical.rootTorqueWorld.z);
                {
                    const Quaternion rootEq = (after.links[0].orientation * profile.links[0].modelOrientation.conjugate()).normalized();
                    const Vec3 bf = rootEq.rotate({1, 0, 0});
                    const Vec3 gf = output.guide.rootOrientationWorld.rotate({1, 0, 0});
                    const Vec3 cf = (after.links[chest].orientation * profile.links[chest].modelOrientation.conjugate()).rotate({1, 0, 0});
                    std::printf("    giro pelve %+.0f peito %+.0f referencia %+.0f camera %+.0f | vel rumo %+.0f | Tz pelve %+.0f pernas %+.0f\n",
                        std::atan2(bf.y, bf.x) * 57.3f, std::atan2(cf.y, cf.x) * 57.3f, std::atan2(gf.y, gf.x) * 57.3f,
                        input.facingYawRadians * 57.3f, std::atan2(v.y, v.x) * 57.3f, physical.rootTorqueWorld.z, physical.legPostureDelivered.z);
                }
                std::printf("    marcha v-corpo %.2f capsula %.2f pedido %.2f | ritmo %.2f passo %.2f estado %d fase %.2f | Fpelve-frente %+.0f\n",
                    dot(v, travel), std::hypot(capsule.velocity.x, capsule.velocity.y), input.intent.requestedVelocityWorld.length(),
                    locomotion.telemetry().playbackRate, locomotion.telemetry().strideScale, static_cast<int>(locomotion.telemetry().state),
                    locomotion.telemetry().cyclePhase, dot(physical.rootForceWorld, travel));
                for (std::size_t side = 0; side < 2; ++side) {
                    const auto& trace = locomotion.telemetry().footTrace[side];
                    if (trace.valid) std::printf("    pe%zu altura %.2f alvo-lat %+.2f alvo-frente %+.2f alvo-altura %.2f | toca %d apoia %d carga %.0f N sola %.3f vsola %.2f\n", side,
                        trace.physicalClearance, lat(trace.target), ahead(trace.target), trace.targetClearance,
                        ps.feet[side].touching ? 1 : 0, ps.feet[side].supporting ? 1 : 0, ps.feet[side].loadNewtons,
                        ps.feet[side].soleLowestHeight, ps.feet[side].soleVelocityWorld.length());
                }
            }
            std::printf("FREADA %.3f est %d run %.2f v %.2f along %+.2f | pe %+.2f%s %+.2f%s captura %+.2f | pelve %+5.1f peito %+5.1f | Fpelve %+5.0f req %+5.1f aloc %+5.1f | fase %d%d razao %d%d segue %d%d | brake %.2f bal %.2f com z %.2f\n",
                t, static_cast<int>(bodyState), locomotion.telemetry().brakeRunShare, speedNow,
                dot(v, travel),
                ahead(after.links[feet[0]].position), ps.feet[0].supporting ? "*" : " ",
                ahead(after.links[feet[1]].position), ps.feet[1].supporting ? "*" : " ",
                capture, pitchOf(after.links[0].orientation), pitchOf(after.links[chest].orientation),
                dot(physical.rootForceWorld, travel),
                dot(physical.requestedComAccelerationWorld, travel), dot(physical.allocatedComAccelerationWorld, travel),
                static_cast<int>(plan.feet[0].phase), static_cast<int>(plan.feet[1].phase),
                static_cast<int>(plan.feet[0].reason), static_cast<int>(plan.feet[1].reason),
                plan.feet[0].followingGait ? 1 : 0, plan.feet[1].followingGait ? 1 : 0,
                locomotion.telemetry().brakeReaction, locomotion.telemetry().balanceReaction, com.z);
            if (std::getenv("XBRAKESPINE")) {
                // Coluna: alvo e medido (flexao, eixo 1) de abdomen, peito e
                // peito superior, e o quadril (coxa, eixo 1) dos dois lados.
                std::printf("   coluna");
                for (const char* id : {"Abdomen", "Chest", "UpperChest", "LeftThigh", "RightThigh", "LeftShin", "RightShin"}) {
                    for (const auto& target : output.driveTargets) {
                        if (profile.links[target.linkIndex].id != id) continue;
                        const int axis = std::string(id).find("Shin") != std::string::npos ? 0 : 1;
                        if (static_cast<int>(target.axis) != axis) continue;
                        std::printf(" %s %+.0f/%+.0f", id, target.positionRadians * 57.3f,
                            state.joints[target.linkIndex].positionRadians[static_cast<std::size_t>(axis)] * 57.3f);
                    }
                }
                for (std::size_t side = 0; side < 2; ++side) {
                    const auto& trace = locomotion.telemetry().footTrace[side];
                    if (trace.valid)
                        std::printf(" | pe%zu alvo %+.2f ik %+.2f cmd %+.2f cmdfis %+.2f (z %+.3f) fis %+.2f z %.2f raizik %+.2f %s%s", side, ahead(trace.target), ahead(trace.ik), ahead(trace.command),
                            ahead(trace.commandFromPhysicalPelvis), trace.commandFromPhysicalPelvisClearance, ahead(trace.physical), trace.physicalClearance, ahead(trace.ikRoot),
                            locomotion.telemetry().footPlanted[side] ? "T" : "-",
                            locomotion.telemetry().footworkAnchorUnreachable[side] ? "U" : "");
                }
                std::printf(" | lean %+.1f | pelve z %.3f alvo %.3f vz %+.2f com vz %+.2f\n", locomotion.telemetry().forwardLeanRadians * 57.3f,
                    after.links[0].position.z, output.guide.rootPositionWorld.z, after.links[0].linearVelocity.z,
                    ps.centerOfMassVelocityWorld.z);
            }
            for (std::size_t side = 0; side < 2; ++side) {
                const auto& foot = plan.feet[side];
                if (foot.followingGait || (foot.phase != FootPhase3D::Swing && foot.phase != FootPhase3D::TouchdownSearch)) continue;
                std::printf("   pe %zu razao %d balanco %.2f/%.2f alvo %+.2f agora %+.2f altura %.3f\n", side,
                    static_cast<int>(foot.reason), foot.swingSeconds, foot.swingDurationSeconds,
                    ahead(foot.goalWorld), ahead(after.links[feet[side]].position),
                    after.links[feet[side]].position.z);
            }
        }
        if (std::getenv("XPOSTURE") && tick % 60 == 0) {
            // Postura parado: inclinacao (graus, + para a frente) do peito
            // superior, pescoco e cabeca no plano da pelve, e a cabeca a
            // frente da base do pescoco (cm).
            const Vec3 f = after.links[0].orientation.rotate({1, 0, 0});
            const Vec3 fwd = Vec3 { f.x, f.y, 0.0f }.normalized();
            const auto tilt = [&](const char* id) {
                for (std::size_t i = 0; i < profile.links.size(); ++i)
                    if (profile.links[i].id == id) {
                        const Vec3 up = after.links[i].orientation.rotate({0, 0, 1});
                        return std::atan2(dot(up, fwd), up.z) * 57.3f;
                    }
                return 0.0f;
            };
            const auto at = [&](const char* id) {
                for (std::size_t i = 0; i < profile.links.size(); ++i)
                    if (profile.links[i].id == id) return after.links[i].position;
                return Vec3 {};
            };
            std::printf("POSTURA %.1f estado %d | pelve %+5.1f abdomen %+5.1f peito %+5.1f peito-sup %+5.1f pescoco %+5.1f cabeca %+5.1f | cabeca a frente do pescoco %+.1f cm, do peito-sup %+.1f cm\n",
                t, static_cast<int>(locomotion.telemetry().state), tilt("Pelvis"), tilt("Abdomen"), tilt("Chest"), tilt("UpperChest"),
                tilt("Neck"), tilt("Head"), dot(at("Head") - at("Neck"), fwd) * 100.0f, dot(at("Head") - at("UpperChest"), fwd) * 100.0f);
        }
        if (std::getenv("XFALLTRACE") && tick % 6 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 0.0f)
                && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 2.5f)) {
            // Queda: inclinacao da pelve, reflexo/entrega, alturas da cabeca,
            // maos e pelve, cotovelo e joelho (graus de flexao pelos pontos
            // dos links) e o que toca o chao.
            const auto at = [&](const char* id) {
                for (std::size_t i = 0; i < profile.links.size(); ++i)
                    if (profile.links[i].id == id) return after.links[i].position;
                return Vec3 {};
            };
            const auto bend = [](Vec3 a, Vec3 b, Vec3 c) {
                const Vec3 u = (a - b).normalized(), v = (c - b).normalized();
                return 180.0f - std::acos(std::clamp(dot(u, v), -1.0f, 1.0f)) * 57.3f;
            };
            const float upright = after.links.front().orientation.rotate({ 0, 0, 1 }).z;
            std::printf("CAI %.2f fase %d arruma %.2f tronco-v %.2f incl %3.0f reflexo %.2f entrega %.2f | cabeca %.2f v %+5.2f | maos %.2f %.2f | pelve %.2f | cotovelo %3.0f %3.0f | joelho %3.0f %3.0f | chao:",
                t, static_cast<int>(locomotion.telemetry().getUpPhase), locomotion.telemetry().settleGather,
                after.links.front().linearVelocity.length(),
                std::acos(std::clamp(upright, -1.0f, 1.0f)) * 57.3f, locomotion.telemetry().fallReflex,
                locomotion.telemetry().fallCommit, at("Head").z, [&] { for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == "Head") return after.links[i].linearVelocity.z; return 0.0f; }(),
                at("LeftHand").z, at("RightHand").z, after.links[0].position.z,
                bend(at("LeftUpperArm"), at("LeftForearm"), at("LeftHand")), bend(at("RightUpperArm"), at("RightForearm"), at("RightHand")),
                bend(at("LeftThigh"), at("LeftShin"), at("LeftFoot")), bend(at("RightThigh"), at("RightShin"), at("RightFoot")));
            for (const auto& contact : after.interactions)
                if (contact.normalWorld.z > 0.5f && contact.linkIndex < profile.links.size()
                    && profile.links[contact.linkIndex].id.find("Foot") == std::string::npos)
                    std::printf(" %s", profile.links[contact.linkIndex].id.c_str());
            std::printf("\n");
            if (std::getenv("XFALLJOINTS")) {
                // Alvo/medido (graus): quadril (coxa, eixo 1), joelho (canela,
                // eixo 0) e cotovelo (antebraco, eixo 0).
                std::printf("   juntas");
                for (const char* id : {"LeftThigh", "RightThigh", "LeftShin", "RightShin", "LeftForearm", "RightForearm"}) {
                    for (const auto& target : output.driveTargets) {
                        if (profile.links[target.linkIndex].id != id) continue;
                        const int axis = std::string(id).find("Thigh") != std::string::npos ? 1 : 0;
                        if (static_cast<int>(target.axis) != axis) continue;
                        std::printf(" %s %+.0f/%+.0f", id, target.positionRadians * 57.3f,
                            after.joints[target.linkIndex].positionRadians[static_cast<std::size_t>(axis)] * 57.3f);
                    }
                }
                std::printf("\n");
            }
        }
        if (std::getenv("XCUTTRACE") && tick % 4 == 0 && t > (std::getenv("XFROM") ? std::atof(std::getenv("XFROM")) : 3.4f)
                && t < (std::getenv("XTO") ? std::atof(std::getenv("XTO")) : 4.4f)) {
            // Virada: velocidade no rumo de antes (+X do cenario) e de lado,
            // giro da pelve, altura dela, peito, pes (apoio * e giro) e
            // a fase da freada.
            const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
            const Vec3 pf = after.links[0].orientation.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 cu = after.links[chest].orientation.rotate({ 0.0f, 0.0f, 1.0f });
            std::printf("VIRA %.3f v %+5.2f %+5.2f | pelve giro %+5.0f alt %.2f | peito frente %+5.1f lado %+5.1f |", t,
                v.x, v.y, std::atan2(pf.y, pf.x) * 57.3f, after.links[0].position.z,
                std::asin(std::clamp(cu.x, -1.0f, 1.0f)) * 57.3f, std::asin(std::clamp(cu.y, -1.0f, 1.0f)) * 57.3f);
            for (std::size_t side = 0; side < 2; ++side) {
                const Vec3 ff = footForwardWorld3D(profile.links[feet[side]], after.links[feet[side]].orientation);
                std::printf(" pe%zu%s %+5.0f", side, locomotion.physicalState().feet[side].supporting ? "*" : " ",
                    std::atan2(ff.y, ff.x) * 57.3f);
            }
            std::printf(" | freada %d estado %d\n", locomotion.telemetry().brakePhase,
                static_cast<int>(locomotion.telemetry().state));
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
                if (scenario.curveFrom >= 0.0f && t >= scenario.curveFrom - 0.1f && t <= scenario.curveTo) {
                    const float vSpeed = std::hypot(comVelocity.x, comVelocity.y);
                    const float heading = std::atan2(comVelocity.y, comVelocity.x);
                    const float rate = result.curveHeadingValid
                        ? std::remainder(heading - result.curvePreviousHeading, 6.28318531f) / dt : 0.0f;
                    result.curvePreviousHeading = heading;
                    result.curveHeadingValid = true;
                    if (t >= scenario.curveFrom && vSpeed > 0.5f) {
                        const Vec3 fwd { std::cos(heading), std::sin(heading), 0.0f };
                        const Vec3 left { -fwd.y, fwd.x, 0.0f };
                        const auto linkAt = [&](const char* id) {
                            for (std::size_t i = 0; i < profile.links.size(); ++i)
                                if (profile.links[i].id == id) return after.links[i].position;
                            return after.links[0].position;
                        };
                        const Vec3 trunk = linkAt("Neck") - after.links[0].position;
                        const Vec3 pelvisForward = after.links[0].orientation.rotate({ 1.0f, 0.0f, 0.0f });
                        const float pelvisYaw = std::atan2(pelvisForward.y, pelvisForward.x);
                        const float requestedHeading = std::atan2(requested.y, requested.x);
                        ++result.curveSamples;
                        result.curveSpeed += vSpeed;
                        result.curveRequestedSpeed += requestedSpeed;
                        result.curveAcceleration += vSpeed * rate;
                        result.curveTrunkSide += std::atan2(dot(trunk, left), trunk.z) * 57.2958f;
                        result.curveTrunkForward += std::atan2(dot(trunk, fwd), trunk.z) * 57.2958f;
                        result.curvePelvisLead += std::remainder(pelvisYaw - heading, 6.28318531f) * 57.2958f;
                        result.curveLag += requestedSpeed > 0.2f
                            ? std::remainder(requestedHeading - heading, 6.28318531f) * 57.2958f : 0.0f;
                        const auto& physicalFeet = locomotion.physicalState().feet;
                        for (std::size_t side = 0; side < 2; ++side) {
                            if (!physicalFeet[side].supporting) continue;
                            const Vec3 footForward = footForwardWorld3D(profile.links[feet[side]],
                                after.links[feet[side]].orientation);
                            const Vec3 hip = linkAt(side == 0 ? "LeftThigh" : "RightThigh");
                            ++result.curveStance[side];
                            result.curveFootYaw[side] += std::remainder(
                                std::atan2(footForward.y, footForward.x) - heading, 6.28318531f) * 57.2958f;
                            result.curveLeg[side] += (hip - after.links[feet[side]].position).length();
                            result.curveHipHeight[side] += hip.z - physicalFeet[side].soleLowestHeight;
                            const Vec3 body = after.links[0].position - physicalFeet[side].soleCenterWorld;
                            result.curveBodyLean[side] += std::atan2(dot(body, left), body.z) * 57.2958f;
                        }
                    }
                }
                if (scenario.probeStart >= 0.0f && t >= scenario.probeStart && result.probeReachedAt < 0.0f
                    && comVelocity.x * scenario.probeDirection.x + comVelocity.y * scenario.probeDirection.y
                        >= scenario.probeSpeed)
                    result.probeReachedAt = t;
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
                        result.gaitSignedYawSum += std::remainder(bodyYaw - wanted.lookYaw, 2.0f * 3.14159265f) * 57.2958f;
                        ++result.gaitSustainedSamples;
                        {
                            const Vec3 travel = requested * (1.0f / requestedSpeed);
                            const Vec3 left { -travel.y, travel.x, 0.0f };
                            const auto& target = locomotion.output().targetPose;
                            const Quaternion chestNow[2] = { after.links[chest].orientation,
                                chest < target.linkOrientations.size() ? target.linkOrientations[chest] : after.links[chest].orientation };
                            for (int k = 0; k < 2; ++k) {
                                const Vec3 up = (chestNow[k] * profile.links[chest].modelOrientation.conjugate()).rotate({0, 0, 1});
                                const double pitch = std::asin(std::clamp(dot(up, travel), -1.0f, 1.0f)) * 57.2958;
                                const double roll = std::asin(std::clamp(dot(up, left), -1.0f, 1.0f)) * 57.2958;
                                result.chestPitch[k] += pitch; result.chestPitchSq[k] += pitch * pitch;
                                result.chestRoll[k] += roll; result.chestRollSq[k] += roll * roll;
                                if (result.previousChestValid) {
                                    Quaternion delta = (chestNow[k] * result.previousChest[k].conjugate()).normalized();
                                    if (delta.w < 0.0f) delta = { -delta.x, -delta.y, -delta.z, -delta.w };
                                    const double angle = 2.0 * std::acos(std::min(1.0f, delta.w)) / dt;
                                    result.chestSpinSq[k] += angle * angle;
                                    const float sinHalf = std::sqrt(std::max(1e-12f, 1.0f - delta.w * delta.w));
                                    const Vec3 axis { delta.x / sinHalf, delta.y / sinHalf, delta.z / sinHalf };
                                    const Vec3 w = axis * static_cast<float>(angle);
                                    const double c[3] = { dot(w, travel), dot(w, left), w.z };
                                    for (int a = 0; a < 3; ++a) result.chestSpinAxisSq[k][a] += c[a] * c[a];
                                }
                                result.previousChest[k] = chestNow[k];
                            }
                            result.previousChestValid = true;
                            ++result.chestSamples;
                            result.pelvisZ += after.links[0].position.z; result.pelvisZSq += after.links[0].position.z * after.links[0].position.z;
                            result.alongV += along; result.alongVSq += along * along;
                            for (std::size_t side = 0; side < 2; ++side) {
                                const auto& foot = locomotion.physicalState().feet[side];
                                const bool down = foot.supporting && !foot.heldFromMemory;
                                const Vec3 sole = foot.soleCenterWorld;
                                if (down && !result.slideSupported[side]) {
                                    result.slideStart[side] = sole;
                                    result.slideSeconds[side] = 0.0f;
                                    result.slideStartVelocity[side] = foot.soleVelocityWorld;
                                    result.slideStartLift[side] = after.links[0].position.z - output.guide.rootPositionWorld.z;
                                }
                                if (down) {
                                    result.slideLast[side] = sole;
                                    result.slideSeconds[side] += dt;
                                }
                                if (std::getenv("XSLIDETICK") && side == 1 && t > 4.2f && t < 4.9f) {
                                    const Vec3 r = input.intent.requestedVelocityWorld;
                                    const float rs = std::max(0.1f, std::hypot(r.x, r.y));
                                    const auto al = [&](Vec3 p) { return (p.x * r.x + p.y * r.y) / rs; };
                                    const auto& tr = locomotion.telemetry().footTrace[1];
                                    std::printf("TICK %.3f %s%s sola %+.3f pe %+.3f alvo %+.3f pelve %+.3f ref %+.3f | z pe %.3f pelve %.3f ref %.3f | vsola %+.2f plantado %d\n", t,
                                        down ? "*" : " ", foot.touching ? "t" : " ", al(sole), al(after.links[feet[1]].position), tr.valid ? al(tr.target) : 0.0f,
                                        al(after.links[0].position), al(output.guide.rootPositionWorld), after.links[feet[1]].position.z, after.links[0].position.z,
                                        output.guide.rootPositionWorld.z, al(foot.soleVelocityWorld), locomotion.telemetry().footPlanted[1] ? 1 : 0);
                                }
                                if (!down && result.slideSupported[side] && result.slideSeconds[side] > 0.05f) {
                                    const Vec3 moved = result.slideLast[side] - result.slideStart[side];
                                    if (std::getenv("XSLIDETRACE") && side == 1) {
                                        const Vec3 r = input.intent.requestedVelocityWorld;
                                        const float rs = std::max(0.1f, std::hypot(r.x, r.y));
                                        const Vec3 v0 = result.slideStartVelocity[side];
                                        std::printf("SLIDE %.3f %3.0f ms anda %.3f frente %+.3f lado %+.3f | v no toque %+.2f | pelve-ref no toque %+.3f\n", t, result.slideSeconds[side] * 1000.0f,
                                            std::hypot(moved.x, moved.y), (moved.x * r.x + moved.y * r.y) / rs, (-moved.x * r.y + moved.y * r.x) / rs,
                                            (v0.x * r.x + v0.y * r.y) / rs, result.slideStartLift[side]);
                                    }
                                    result.slideDistance += std::hypot(moved.x, moved.y);
                                    result.slideTime += result.slideSeconds[side];
                                    ++result.slideStances;
                                }
                                result.slideSupported[side] = down;
                            }
                            for (const auto& contact : after.contacts) {
                                if (contact.linkIndex != feet[0] && contact.linkIndex != feet[1]) continue;
                                if (contact.normal.z < 0.4f || contact.normalImpulseNewtonSeconds <= 1e-5f) continue;
                                result.footSlip += contact.tangentialSpeedMetersPerSecond;
                                ++result.footSlipSamples;
                                if (contact.tangentialSpeedMetersPerSecond > 0.2f) ++result.footSlipOver;
                            }
                        }
                    }
                } else if (result.gaitMoveEnd >= 0 && result.gaitStoppedAt < 0
                    && std::hypot(comVelocity.x, comVelocity.y) < 0.2f) {
                    result.gaitStoppedAt = t;
                }
            }
            result.pelvisX.emplace_back(t, after.links.front().position.x);
            {
                const bool on = locomotion.telemetry().tripRecovery > 0.3f && result.firstFallSeconds < 0.0f;
                if (on) ++result.stumbleTicks;
                if (on && !result.stumbleOn) ++result.stumbleEvents;
                result.stumbleOn = on;
                if (on && (scenario.stumbleX > 0.0f || scenario.snagAt > 0.0f) && result.stAt < 0.0f) ++result.stumbleTicksBefore;
            }
            if (std::getenv("XSTTRACE") && std::atoi(std::getenv("XSTTRACE")) == 2 && t >= 0.4f
                && static_cast<int>(t * 120.0f + 0.5f) % 3 == 0) {
                const auto& tel = locomotion.telemetry();
                const Vec3 cv = locomotion.physicalState().centerOfMassVelocityWorld;
                const auto& pf = locomotion.physicalState().feet;
                std::printf("SIG %.3f v %.2f | excesso %+5.1f taxa %+5.2f freada %+6.1f | risco %.2f trip %.2f | est %d | espera %.2f clipe %.1f %.1f | E %d%d %.2f D %d%d %.2f | x %.2f z %.2f\n", t,
                    std::hypot(cv.x, cv.y), tel.stumbleExcess * 57.2958f, tel.stumbleRate, tel.stumbleDecel, tel.stumbleRisk,
                    tel.tripRecovery, static_cast<int>(tel.state), tel.touchdownWaitSeconds, tel.clipFootContact[0], tel.clipFootContact[1],
                    pf[0].touching ? 1 : 0, pf[0].supporting ? 1 : 0, pf[0].soleLowestHeight, pf[1].touching ? 1 : 0, pf[1].supporting ? 1 : 0,
                    pf[1].soleLowestHeight, state.links.front().position.x, state.links.front().position.z);
            }
            if (t > 1.0f && result.firstFallSeconds < 0.0f) {
                const auto& pfeet = locomotion.physicalState().feet;
                // Lado da curva pelo giro do caminho do corpo (rad/s).
                const Vec3 cvel = locomotion.physicalState().centerOfMassVelocityWorld;
                const float bodyHeading = std::atan2(cvel.y, cvel.x);
                const float rawRate = std::remainder(bodyHeading - result.kickPreviousHeading, 6.28318531f) / dt;
                result.kickPreviousHeading = bodyHeading;
                result.kickHeadingRate += (std::clamp(rawRate, -10.0f, 10.0f) - result.kickHeadingRate) * 0.05f;
                const float headingRate = result.kickHeadingRate;
                for (std::size_t side = 0; side < 2; ++side) {
                    const float progress = locomotion.telemetry().footSwingProgress[side];
                    // Dentro (0) se a curva vai para o lado deste pe.
                    const int curveSide = std::abs(headingRate) < 0.3f ? -1
                        : ((headingRate > 0.0f) == (side == 0) ? 0 : 1);
                    const bool inSwing = progress >= 0.15f && progress < 0.80f;
                    const bool touch = inSwing && pfeet[side].touching;
                    if (touch && !result.swingTouchOn[side] && curveSide >= 0
                        && (scenario.turnStart < 0.0f || (t >= scenario.turnStart - 0.1f && t <= scenario.turnEnd + 1.0f))) {
                        ++result.swingTouches[static_cast<std::size_t>(curveSide)];
                        if (progress < 0.35f) ++result.swingTouchesEarly;
                        if (progress >= 0.5f) ++result.swingTouchesLate;
                        if (std::getenv("XKICKTRACE")) {
                            const Vec3 sole = pfeet[side].soleCenterWorld;
                            const Vec3 com = locomotion.physicalState().centerOfMassWorld;
                            const Vec3 fwd { std::cos(bodyHeading), std::sin(bodyHeading), 0.0f };
                            const auto& trace = locomotion.telemetry().footTrace[side];
                            std::printf("TOQUE %.3f pe %zu %s progresso %.2f | carga %.0f N apoio %d | sola %+.2f do corpo, v %.2f | alvo folga %.2f | giro %.2f | v %.2f\n",
                                t, side, curveSide == 0 ? "dentro" : "fora", progress, pfeet[side].loadNewtons,
                                pfeet[side].supporting ? 1 : 0, dot(sole - com, fwd),
                                std::hypot(pfeet[side].soleVelocityWorld.x, pfeet[side].soleVelocityWorld.y),
                                trace.valid ? trace.targetClearance : -1.0f, headingRate, std::hypot(cvel.x, cvel.y));
                        }
                    }
                    result.swingTouchOn[side] = touch;
                    const bool mid = progress >= 0.30f && progress < 0.70f;
                    if (mid) {
                        if (!result.swingMidOn[side]) {
                            result.swingMidOn[side] = true;
                            result.swingMinClearance[side] = 1.0f;
                            result.swingSideAtStart[side] = curveSide;
                        }
                        result.swingMinClearance[side] = std::min(result.swingMinClearance[side], pfeet[side].soleLowestHeight);
                    } else if (result.swingMidOn[side]) {
                        result.swingMidOn[side] = false;
                        const int cs = result.swingSideAtStart[side];
                        if (cs >= 0 && (scenario.turnStart < 0.0f || (t >= scenario.turnStart - 0.1f && t <= scenario.turnEnd + 1.0f))) {
                            ++result.swingsCounted[static_cast<std::size_t>(cs)];
                            if (result.swingMinClearance[side] < 0.02f) ++result.lowSwings[static_cast<std::size_t>(cs)];
                        }
                    }
                    const float soleSpeed = std::hypot(pfeet[side].soleVelocityWorld.x, pfeet[side].soleVelocityWorld.y);
                    const bool kick = progress >= 0.12f && progress < 0.75f && pfeet[side].touching
                        && !pfeet[side].supporting && soleSpeed > 0.55f;
                    if (kick && !result.kickOn[side]) {
                        ++result.kicks;
                        if (scenario.turnStart >= 0.0f && t >= scenario.turnStart - 0.1f && t <= scenario.turnEnd + 1.0f)
                            ++result.kicksInTurn;
                        if (std::getenv("XKICKTRACE"))
                            std::printf("BICUDA %.3f pe %zu progresso %.2f v %.2f sola %.2f\n", t, side, progress,
                                std::hypot(locomotion.physicalState().centerOfMassVelocityWorld.x,
                                    locomotion.physicalState().centerOfMassVelocityWorld.y), soleSpeed);
                    }
                    result.kickOn[side] = kick;
                }
            }
            if (scenario.startAt > 0.0f && std::getenv("XARRTRACE") && t >= scenario.startAt - 0.1f
                && t <= scenario.startAt + 1.0f && static_cast<int>(t * 120.0f + 0.5f) % 3 == 0) {
                // No rumo do comando (x) e a esquerda dele (y), em relacao ao
                // centro de massa; * apoio; sw progresso do balanco.
                const auto wanted = scenario.script(t);
                const Vec3 fwd = wanted.move.lengthSquared() > 0.01f ? wanted.move.normalized() : Vec3 { 1, 0, 0 };
                const Vec3 lft { -fwd.y, fwd.x, 0.0f };
                const auto& pfeet = locomotion.physicalState().feet;
                const Vec3 com = locomotion.physicalState().centerOfMassWorld;
                const Vec3 cv = locomotion.physicalState().centerOfMassVelocityWorld;
                const auto& tel = locomotion.telemetry();
                const Vec3 up = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({0, 0, 1}));
                const auto rel = [&](Vec3 p) { return Vec3 { dot(p - com, fwd), dot(p - com, lft), 0.0f }; };
                const Vec3 l = rel(pfeet[0].soleCenterWorld), r = rel(pfeet[1].soleCenterWorld);
                std::printf("ART %.3f v %.2f | pelve fr %5.1f z %.2f | E %+.2f %+.2f h%.2f%s sw %+.2f | D %+.2f %+.2f h%.2f%s sw %+.2f | fase %.2f est %d setor %d inicio %d trip %.2f\n",
                    t - scenario.startAt, dot(cv, fwd), std::asin(std::clamp(dot(up, fwd), -1.0f, 1.0f)) * 57.2958f, after.links[0].position.z,
                    l.x, l.y, pfeet[0].soleLowestHeight, pfeet[0].supporting ? "*" : " ", tel.footSwingProgress[0],
                    r.x, r.y, pfeet[1].soleLowestHeight, pfeet[1].supporting ? "*" : " ", tel.footSwingProgress[1],
                    tel.cyclePhase, static_cast<int>(tel.state), static_cast<int>(tel.gaitDirection), tel.authoredStartActive ? 1 : 0, tel.tripRecovery);
            }
            if (scenario.startAt > 0.0f && t >= scenario.startAt && result.firstFallSeconds < 0.0f) {
                const auto& pfeet = locomotion.physicalState().feet;
                {
                    const auto wanted = scenario.script(t);
                    const Vec3 cv = locomotion.physicalState().centerOfMassVelocityWorld;
                    if (result.startReachAt < 0.0f && wanted.move.lengthSquared() > 0.01f
                        && dot(cv, wanted.move.normalized()) >= (wanted.sprint ? 2.5f : 2.0f))
                        result.startReachAt = t - scenario.startAt;
                    if (result.startNominal80At < 0.0f && wanted.move.lengthSquared() > 0.01f) {
                        const float nominal = characterGaitSpeed3D(speeds, std::atan2(wanted.move.y, wanted.move.x) - wanted.lookYaw, wanted.sprint);
                        if (dot(cv, wanted.move.normalized()) >= 0.8f * nominal) result.startNominal80At = t - scenario.startAt;
                    }
                }
                if (t <= scenario.startAt + 0.8f) {
                    for (std::size_t side = 0; side < 2; ++side) {
                        const Vec3 sole = pfeet[side].soleCenterWorld;
                        const bool grounded = pfeet[side].soleLowestHeight < 0.03f;
                        if (grounded && result.startDragValid[side])
                            result.startDrag[side] += std::hypot(sole.x - result.startDragLast[side].x, sole.y - result.startDragLast[side].y);
                        result.startDragValid[side] = grounded;
                        result.startDragLast[side] = sole;
                        if (!grounded && pfeet[side].soleLowestHeight > 0.05f && result.firstLiftAt < 0.0f)
                            result.firstLiftAt = t - scenario.startAt;
                    }
                }
                for (std::size_t side = 0; side < 2; ++side) {
                    const float progress = locomotion.telemetry().footSwingProgress[side];
                    const float soleSpeed = std::hypot(pfeet[side].soleVelocityWorld.x, pfeet[side].soleVelocityWorld.y);
                    const bool scuff = progress >= 0.12f && progress < 0.75f && pfeet[side].touching
                        && !pfeet[side].supporting && soleSpeed > 0.55f;
                    if (scuff && !result.scuffOn[side]) {
                        ++result.scuffs;
                        if (result.firstScuffAt < 0.0f) result.firstScuffAt = t - scenario.startAt;
                    }
                    result.scuffOn[side] = scuff;
                }
            }
            if (scenario.stumbleX > 0.0f || scenario.snagAt > 0.0f) {
                const Vec3 com = locomotion.physicalState().centerOfMassWorld;
                const Vec3 comV = locomotion.physicalState().centerOfMassVelocityWorld;
                const float planarSpeed = std::hypot(comV.x, comV.y);
                if (result.stAt < 0.0f && (scenario.stumbleX > 0.0f ? com.x >= scenario.stumbleX - 0.6f
                        : result.snagStartedAt >= 0.0f)) {
                    result.stAt = t;
                    result.stSpeedBefore = planarSpeed;
                }
                if (result.stAt >= 0.0f && result.firstFallSeconds < 0.0f) {
                    if (t <= result.stAt + 1.5f) result.stSpeedMin = std::min(result.stSpeedMin, planarSpeed);
                    if (result.stSpeedAfter < 0.0f && t >= result.stAt + 2.0f) result.stSpeedAfter = planarSpeed;
                    const Vec3 pf = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({0, 0, 1}));
                    result.stPitchPeak = std::max(result.stPitchPeak, std::asin(std::clamp(pf.x, -1.0f, 1.0f)) * 57.2958f);
                    result.stLowest = std::min(result.stLowest, after.links[0].position.z);
                    if (std::getenv("XSTTRACE") && std::atoi(std::getenv("XSTTRACE")) == 1 && t <= result.stAt + 1.6f && static_cast<int>(t * 120.0f + 0.5f) % 3 == 0) {
                        // Pes no rumo (+x) e altura, em relacao ao centro de massa.
                        const auto& pfeet = locomotion.physicalState().feet;
                        const auto& tel = locomotion.telemetry();
                        const Vec3 up = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({0, 0, 1}));
                        std::printf("STT %.3f v %.2f vz %+.2f | pelve %5.1f z %.2f | E %+.2f %.2f%s sw %+.2f D %+.2f %.2f%s sw %+.2f | trip %.2f reac %.2f queda %.2f est %d | risco %.2f passo %d fases %d%d razoes %d%d alvoE %+.2f %.2f alvoD %+.2f %.2f\n",
                            t, planarSpeed, comV.z, std::asin(std::clamp(up.x, -1.0f, 1.0f)) * 57.2958f, after.links[0].position.z,
                            pfeet[0].soleCenterWorld.x - com.x, pfeet[0].soleLowestHeight, pfeet[0].supporting ? "*" : " ", tel.footSwingProgress[0],
                            pfeet[1].soleCenterWorld.x - com.x, pfeet[1].soleLowestHeight, pfeet[1].supporting ? "*" : " ", tel.footSwingProgress[1],
                            tel.tripRecovery, tel.balanceReaction, tel.fallReflex, static_cast<int>(tel.state),
                            tel.stumbleRisk, tel.stumbleStepping ? 1 : 0,
                            static_cast<int>(locomotion.contactPlan().feet[0].phase), static_cast<int>(locomotion.contactPlan().feet[1].phase),
                            static_cast<int>(locomotion.contactPlan().feet[0].reason), static_cast<int>(locomotion.contactPlan().feet[1].reason),
                            tel.footTrace[0].valid ? tel.footTrace[0].target.x - com.x : 0.0f, tel.footTrace[0].valid ? tel.footTrace[0].target.z : 0.0f,
                            tel.footTrace[1].valid ? tel.footTrace[1].target.x - com.x : 0.0f, tel.footTrace[1].valid ? tel.footTrace[1].target.z : 0.0f);
                    }
                }
            }
            if (scenario.turnStart >= 0.0f && result.firstFallSeconds < 0) {
                const Vec3 comV = locomotion.physicalState().centerOfMassVelocityWorld;
                const float planarSpeed = std::hypot(comV.x, comV.y);
                const bool window = t >= scenario.turnStart - 0.1f && t <= scenario.turnEnd + 1.0f;
                if (result.tcSpeedBefore < 0.0f && t >= scenario.turnStart) result.tcSpeedBefore = planarSpeed;
                if (t >= scenario.turnStart && t <= scenario.turnEnd + 0.8f)
                    result.tcMinSpeed = std::min(result.tcMinSpeed, planarSpeed);
                const Vec3 request = input.intent.requestedVelocityWorld;
                const float requestSpeed = std::hypot(request.x, request.y);
                // Resposta: o corpo a 2,5 m/s (sprint) ou 1,8 m/s (trote) no rumo
                // final - fixa, para nao depender do teto da curva.
                if (t >= scenario.turnEnd && result.tcResponse < 0.0f && requestSpeed > 0.2f
                    && comV.x * scenario.turnDirection.x + comV.y * scenario.turnDirection.y
                        >= (input.sprinting ? 2.5f : 1.8f))
                    result.tcResponse = t - scenario.turnStart;
                if (window && locomotion.telemetry().brakePhase > 0) ++result.tcBrakeTicks;
                const auto& physicalFeet = locomotion.physicalState().feet;
                for (std::size_t side = 0; side < 2; ++side) {
                    const Vec3 forward = footForwardWorld3D(profile.links[feet[side]], after.links[feet[side]].orientation);
                    const float yaw = std::atan2(forward.y, forward.x);
                    const Vec3 sole = physicalFeet[side].soleCenterWorld;
                    // Apoio carregado: contato medido (nao a memoria do apoio) e
                    // ao menos 150 N - o pe encostando ou saindo nao conta.
                    const bool loaded = physicalFeet[side].contactObserved && physicalFeet[side].loadNewtons > 150.0f;
                    if (loaded && !result.tcSupported[side]) {
                        result.tcStartYaw[side] = yaw;
                        result.tcPivotPeak[side] = 0.0f;
                        result.tcStartSole[side] = sole;
                        result.tcSoleTravel[side] = 0.0f;
                    } else if (loaded) {
                        result.tcPivotPeak[side] = std::max(result.tcPivotPeak[side],
                            std::abs(std::remainder(yaw - result.tcStartYaw[side], 6.28318531f)) * 57.2958f);
                        result.tcSoleTravel[side] = std::max(result.tcSoleTravel[side],
                            std::hypot(sole.x - result.tcStartSole[side].x, sole.y - result.tcStartSole[side].y));
                    } else if (result.tcSupported[side] && window) {
                        if (std::getenv("XTCPIVOT") && result.tcPivotPeak[side] > 30.0f)
                            std::printf("PIVO %.3f pe %zu giro %.1f deslize %.3f v %.2f freada %d est %d setor %d\n", t, side,
                                result.tcPivotPeak[side], result.tcSoleTravel[side], planarSpeed, locomotion.telemetry().brakePhase,
                                static_cast<int>(locomotion.telemetry().state), static_cast<int>(locomotion.telemetry().gaitDirection));
                        ++result.tcStances;
                        result.tcPivotSum += result.tcPivotPeak[side];
                        result.tcPivotWorst = std::max(result.tcPivotWorst, result.tcPivotPeak[side]);
                        if (result.tcPivotPeak[side] > 30.0f) ++result.tcPivotOver30;
                        result.tcSlideSum += result.tcSoleTravel[side];
                    }
                    result.tcSupported[side] = loaded;
                }
                if (std::getenv("XTCTRACE") && t >= scenario.turnStart - 0.3f && t <= scenario.turnEnd + 1.5f
                    && static_cast<int>(t * 120.0f + 0.5f) % 3 == 0) {
                    // No referencial da velocidade do centro de massa (x no rumo,
                    // y a esquerda): pes (* apoio), capsula, pedido, pelve.
                    const float heading = std::atan2(comV.y, comV.x);
                    const Vec3 fwd { std::cos(heading), std::sin(heading), 0.0f };
                    const Vec3 lft { -fwd.y, fwd.x, 0.0f };
                    const Vec3 com = locomotion.physicalState().centerOfMassWorld;
                    const auto rel = [&](Vec3 p) { return Vec3 { dot(p - com, fwd), dot(p - com, lft), p.z }; };
                    const Vec3 pf = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({1, 0, 0}));
                    const auto ang = [&](Vec3 v) { return std::remainder(std::atan2(v.y, v.x) - heading, 6.28318531f) * 57.2958f; };
                    const Vec3 up = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({0, 0, 1}));
                    const Vec3 capV = input.rootVelocityWorld;
                    const auto& tel = locomotion.telemetry();
                    const Vec3 l = rel(physicalFeet[0].soleCenterWorld), r = rel(physicalFeet[1].soleCenterWorld);
                    std::printf("TCT %.3f v %.2f rumo %+6.1f | pede %+6.1f capsula %.2f %+6.1f | pelve %+6.1f lado %+5.1f frente %+5.1f | setor %d est %d freada %d curva %.2f giro %+.2f | E %+.2f %+.2f%s D %+.2f %+.2f%s | Fpelve %+5.0f %+5.0f | z %.2f | plant %d%d sw %+.2f %+.2f | folga E %+.2f/%+.2f/%+.2f D %+.2f/%+.2f/%+.2f | refLado %+.1f | emPe %.2f erro %.0f env %.2f calma %.2f trip %.2f\n",
                        t, planarSpeed, heading * 57.2958f, request.lengthSquared() > 0.01f ? ang(request) : 0.0f,
                        std::hypot(capV.x, capV.y), ang(capV), ang(pf), std::asin(std::clamp(dot(up, lft), -1.0f, 1.0f)) * 57.2958f,
                        std::asin(std::clamp(dot(up, fwd), -1.0f, 1.0f)) * 57.2958f,
                        static_cast<int>(tel.gaitDirection), static_cast<int>(tel.state), tel.brakePhase, tel.authoredCurveBlend,
                        tel.turningRateRadiansPerSecond, l.x, l.y, physicalFeet[0].supporting ? "*" : " ", r.x, r.y,
                        physicalFeet[1].supporting ? "*" : " ", dot(physical.rootForceWorld, fwd), dot(physical.rootForceWorld, lft),
                        after.links[0].position.z, tel.footPlanted[0] ? 1 : 0, tel.footPlanted[1] ? 1 : 0,
                        tel.footSwingProgress[0], tel.footSwingProgress[1],
                        tel.footTrace[0].targetClearance, tel.footTrace[0].commandClearance, tel.footTrace[0].physicalClearance,
                        tel.footTrace[1].targetClearance, tel.footTrace[1].commandClearance, tel.footTrace[1].physicalClearance,
                        tel.lateralLeanRadians * 57.2958f, physical.uprightAuthorityShare, physical.tiltErrorDegrees,
                        physical.assistEnvelope, controller.selfMotionCalm(), tel.tripRecovery);
                }
                if (window) {
                    const Vec3 left = after.links[feet[0]].position, right = after.links[feet[1]].position;
                    const Vec3 pelvisLeft = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({0, 1, 0}));
                    const auto sector = locomotion.telemetry().gaitDirection;
                    const bool strafing = sector == CharacterGaitDirection3D::Left || sector == CharacterGaitDirection3D::Right;
                    if (!strafing && (left.x - right.x) * pelvisLeft.x + (left.y - right.y) * pelvisLeft.y < 0.0f)
                        ++result.tcCrossTicks;
                }
            }
            if (scenario.transitionMetrics && result.firstFallSeconds < 0) {
                const Vec3 requestNow = input.intent.requestedVelocityWorld;
                const float speedNow = std::hypot(requestNow.x, requestNow.y);
                const float speedBefore = std::hypot(result.transPreviousRequest.x, result.transPreviousRequest.y);
                bool changed = false;
                if (speedNow > 0.3f && speedBefore > 0.3f) {
                    const float cosine = (requestNow.x * result.transPreviousRequest.x + requestNow.y * result.transPreviousRequest.y)
                        / (speedNow * speedBefore);
                    changed = cosine < 0.87f || std::abs(speedNow - speedBefore) > 1.0f;
                } else if (speedNow > 0.3f && speedBefore <= 0.3f) {
                    changed = true;
                }
                if ((changed || (speedNow <= 0.3f && speedBefore > 0.3f)) && t > 0.6f)
                    result.transWindowUntil = t + 0.8f;
                if (changed && t > 0.6f) {
                    if (result.transChangeAt >= 0) {  // a anterior nao chegou
                        result.transResponseSum += 2.0; result.transResponseWorst = std::max(result.transResponseWorst, 2.0f);
                        ++result.transChanges;
                    }
                    result.transChangeAt = t;
                    result.transRequest = requestNow;
                }
                result.transPreviousRequest = requestNow;
                const Vec3 comV = locomotion.physicalState().centerOfMassVelocityWorld;
                if (result.transChangeAt >= 0) {
                    const float reqSpeed = std::hypot(result.transRequest.x, result.transRequest.y);
                    const float along = (comV.x * result.transRequest.x + comV.y * result.transRequest.y) / std::max(0.1f, reqSpeed);
                    const float waited = t - result.transChangeAt;
                    if (along >= 0.6f * reqSpeed || waited >= 2.0f) {
                        const float response = std::min(waited, 2.0f);
                        result.transResponseSum += response;
                        result.transResponseWorst = std::max(result.transResponseWorst, response);
                        ++result.transChanges;
                        result.transChangeAt = -1;
                    }
                }
                // Saltinho: a pelve fisica sobe acima da de referencia (o clipe)
                // mais de 4 cm - a perna empurrou mais do que a passada pede
                // (rearma abaixo de 1,5 cm). Os cenarios de transicao nao pulam.
                {
                    // So nas trocas (0,8 s depois de cada mudanca de pedido ou
                    // parada); fora delas, o maximo da passada sem troca.
                    const float lift = after.links[0].position.z - output.guide.rootPositionWorld.z;
                    if (t < result.transWindowUntil) {
                        result.transPeakUp = std::max(result.transPeakUp, lift);
                        if (lift > 0.04f && result.transHopArmed > 0.5f) { ++result.transHops; result.transHopArmed = 0.0f; }
                    } else if (t > 1.6f) {
                        result.transSteadyUp = std::max(result.transSteadyUp, lift);
                    }
                    if (lift < 0.015f) result.transHopArmed = 1.0f;
                }
                // Pes cruzados / embolados.
                const Vec3 left = after.links[feet[0]].position, right = after.links[feet[1]].position;
                const Vec3 pelvisLeft = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({0, 1, 0}));
                const float lateral = (left.x - right.x) * pelvisLeft.x + (left.y - right.y) * pelvisLeft.y;
                const auto sector = locomotion.telemetry().gaitDirection;
                const bool strafing = sector == CharacterGaitDirection3D::Left || sector == CharacterGaitDirection3D::Right;
                if (!strafing && lateral < 0.0f) ++result.transCrossTicks;
                const float ground = std::min(left.z, right.z);
                if (std::hypot(left.x - right.x, left.y - right.y) < 0.13f && std::max(left.z, right.z) < ground + 0.15f)
                    ++result.transTouchTicks;
                if (std::getenv("XTRANSTRACE") && (static_cast<int>(t * 120.0f + 0.5f) % 2 == 0)) {
                    // Pes no referencial da pelve (x frente, y esquerda; * apoio,
                    // ~ planejador dono), alvo do IK, fase e setor.
                    const Vec3 pf = after.links[0].orientation.rotate(profile.links[0].modelOrientation.conjugate().rotate({1, 0, 0}));
                    const Vec3 pc = after.links[0].position;
                    const auto fr = [&](Vec3 p) { return (p.x - pc.x) * pf.x + (p.y - pc.y) * pf.y; };
                    const auto lt = [&](Vec3 p) { return (p.x - pc.x) * pelvisLeft.x + (p.y - pc.y) * pelvisLeft.y; };
                    const auto& tel = locomotion.telemetry();
                    const auto& ps = locomotion.physicalState();
                    std::printf("TT %.3f setor %d est %d fase %.2f | pE %+.2f %+.2f z%.2f%s sw%+.2f alvo %+.2f %+.2f | pD %+.2f %+.2f z%.2f%s sw%+.2f alvo %+.2f %+.2f | v %+.2f %+.2f pede %+.2f %+.2f | giro %+.0f | sobe %+.3f freada %d\n",
                        t, static_cast<int>(sector), static_cast<int>(tel.state), tel.cyclePhase,
                        fr(left), lt(left), left.z - ground, ps.feet[0].supporting ? "*" : " ", tel.footSwingProgress[0],
                        tel.footTrace[0].valid ? fr(tel.footTrace[0].target) : 0.0f, tel.footTrace[0].valid ? lt(tel.footTrace[0].target) : 0.0f,
                        fr(right), lt(right), right.z - ground, ps.feet[1].supporting ? "*" : " ", tel.footSwingProgress[1],
                        tel.footTrace[1].valid ? fr(tel.footTrace[1].target) : 0.0f, tel.footTrace[1].valid ? lt(tel.footTrace[1].target) : 0.0f,
                        comV.x * pf.x + comV.y * pf.y, comV.x * pelvisLeft.x + comV.y * pelvisLeft.y,
                        requestNow.x * pf.x + requestNow.y * pf.y, requestNow.x * pelvisLeft.x + requestNow.y * pelvisLeft.y,
                        std::atan2(pf.y, pf.x) * 57.3f, after.links[0].position.z - output.guide.rootPositionWorld.z, tel.brakePhase);
                    if (std::getenv("XTRANSWORLD")) {
                        const auto& f1 = ps.feet[1];
                        std::printf("   mundo D sola %+.3f %+.3f z %.3f pe %+.3f %+.3f vsola %+.2f %+.2f %s%s\n", f1.soleCenterWorld.x, f1.soleCenterWorld.y, f1.soleCenterWorld.z,
                            right.x, right.y, f1.soleVelocityWorld.x, f1.soleVelocityWorld.y, f1.supporting ? "*" : " ", f1.touching ? "t" : " ");
                    }
                    const auto& cp = locomotion.contactPlan();
                    std::printf("   plano E %d/%d%s D %d/%d%s | assume %d%d passos %d alcance %d%d\n",
                        static_cast<int>(cp.feet[0].phase), static_cast<int>(cp.feet[0].reason), cp.feet[0].followingGait ? "g" : "",
                        static_cast<int>(cp.feet[1].phase), static_cast<int>(cp.feet[1].reason), cp.feet[1].followingGait ? "g" : "",
                        tel.footworkStanding ? 1 : 0, tel.footworkAllowSteps ? 1 : 0, static_cast<int>(cp.stepsStarted),
                        tel.footworkAnchorUnreachable[0] ? 1 : 0, tel.footworkAnchorUnreachable[1] ? 1 : 0);
                }
            }
            {
                const auto& ps = locomotion.physicalState();
                for (std::size_t side = 0; side < 2; ++side) {
                    const bool seen = ps.feet[side].contactObserved;
                    result.touchdownAirSeconds[side] = seen ? 0.0f : result.touchdownAirSeconds[side] + dt;
                    if (seen && !result.touchdownContact[side] && t > 0.5f && result.lastAir[side] < 0.10f) ++result.touchdownScuffs;
                    const Vec3 requested = input.intent.requestedVelocityWorld;
                    const float requestedSpeed = std::hypot(requested.x, requested.y);
                    if (seen && !result.touchdownContact[side] && t > 1.5f && result.lastAir[side] >= 0.10f
                            && result.lastSwingTargetValid[side] && requestedSpeed > 0.2f) {
                        const Vec3 at = after.links[feet[side]].position;
                        const Vec3 error { at.x - result.lastSwingTarget[side].x, at.y - result.lastSwingTarget[side].y, 0.0f };
                        result.touchErrorSum += error.length();
                        result.touchErrorAlong += dot(error, requested) / requestedSpeed;
                        result.touchErrorAcross += (-error.x * requested.y + error.y * requested.x) / requestedSpeed;
                        ++result.touchErrorCount;
                    }
                    {
                        const auto& trace = locomotion.telemetry().footTrace[side];
                        if (!seen && trace.valid) {
                            result.lastSwingTarget[side] = trace.target;
                            result.lastSwingTargetValid[side] = true;
                        }
                        if (seen) result.lastSwingTargetValid[side] = result.lastSwingTargetValid[side] && !result.touchdownContact[side] ? true : false;
                    }
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
        if (const char* path = std::getenv("XRASTROCSV"); path != nullptr && !after.links.empty()) {
            // Rastro tick a tick (tools/analysis/locomotion_trace.py): capsula,
            // pedido, pelve, centro de massa e ponto de captura, pes (contato),
            // ajuda na pelve, inclinacoes, a forca do chao nos pes e o estado.
            static FILE* trace = nullptr;
            static std::string tracePath;
            if (trace == nullptr || tracePath != path) {
                if (trace) std::fclose(trace);
                trace = std::fopen(path, "w");
                tracePath = path;
                if (trace) std::fprintf(trace, "t,estado,cx,cy,cvx,cvy,rvx,rvy,px,py,pz,pvx,pvy,pvz,mx,my,mz,mvx,mvy,mvz,capx,capy,"
                    "f0x,f0y,f0z,f0c,f1x,f1y,f1z,f1c,ax,ay,az,incl,peito,chao,envelope,cambaleio,quedas,pend,desv,perdida,compromisso,refx,refy,refz,refvx,refvy,refvz,reqax,alcax,obsax,legshare,balshare,ffon,jsn,obsaz,apoio\n");
            }
            if (trace) {
                const auto& physicalState = locomotion.physicalState();
                const auto& pelvisNow = after.links.front();
                float ground = 0.0f;
                for (const RagdollInteraction3D& contact : after.interactions)
                    if ((contact.linkIndex == feet[0] || contact.linkIndex == feet[1]) && contact.normalWorld.z > 0.5f)
                        ground += contact.normalImpulseNewtonSeconds / dt;
                const Vec3 chestUp = (after.links[chestIndexForTrace].orientation
                    * profile.links[chestIndexForTrace].modelOrientation.conjugate()).rotate({ 0, 0, 1 });
                std::fprintf(trace, "%.4f,%d,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
                    "%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%.3f,%.3f,%d,%.1f,%.1f,%d,%.2f,"
                    "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.0f,%.2f,%d\n",
                    t, static_cast<int>(locomotion.telemetry().state), capsule.position.x, capsule.position.y,
                    capsule.velocity.x, capsule.velocity.y, wanted.move.x, wanted.move.y,
                    pelvisNow.position.x, pelvisNow.position.y, pelvisNow.position.z,
                    pelvisNow.linearVelocity.x, pelvisNow.linearVelocity.y, pelvisNow.linearVelocity.z,
                    physicalState.centerOfMassWorld.x, physicalState.centerOfMassWorld.y, physicalState.centerOfMassWorld.z,
                    physicalState.centerOfMassVelocityWorld.x, physicalState.centerOfMassVelocityWorld.y,
                    physicalState.centerOfMassVelocityWorld.z, physicalState.capturePointWorld.x, physicalState.capturePointWorld.y,
                    after.links[feet[0]].position.x, after.links[feet[0]].position.y, after.links[feet[0]].position.z,
                    physicalState.feet[0].contactObserved ? 1 : 0,
                    after.links[feet[1]].position.x, after.links[feet[1]].position.y, after.links[feet[1]].position.z,
                    physicalState.feet[1].contactObserved ? 1 : 0,
                    physical.rootForceWorld.x, physical.rootForceWorld.y, physical.rootForceWorld.z,
                    std::acos(std::clamp(pelvisNow.orientation.rotate({ 0, 0, 1 }).z, -1.0f, 1.0f)) * 57.2958f,
                    std::acos(std::clamp(chestUp.z, -1.0f, 1.0f)) * 57.2958f, ground, physical.assistEnvelope,
                    locomotion.telemetry().stagger, result.falls,
                    locomotion.telemetry().posturePendulum * 57.2958f, locomotion.telemetry().postureDeviation * 57.2958f,
                    locomotion.telemetry().postureLost ? 1 : 0, locomotion.telemetry().fallCommit,
                    output.guide.rootPositionWorld.x, output.guide.rootPositionWorld.y, output.guide.rootPositionWorld.z,
                    output.guide.rootLinearVelocityWorld.x, output.guide.rootLinearVelocityWorld.y,
                    output.guide.rootLinearVelocityWorld.z,
                    physical.requestedComAccelerationWorld.x, physical.allocatedComAccelerationWorld.x,
                    physical.observedComAccelerationWorld.x, physical.legShare, physical.balanceShare,
                    physical.jointFeedforwardActive ? 1 : 0, physical.jointSupportNewtons,
                    physical.observedComAccelerationWorld.z, locomotion.telemetry().braceRecovering ? 1 : 0);
                std::fflush(trace);
            }
        }
        if (!down && t > 0.6f && !after.links.empty()) {
            // Boletim v2 (ver bolFlightPeak).
            const bool jumpWindow = t - result.bolLastJumpAt < 1.3f;
            const auto& feetNow = locomotion.physicalState().feet;
            const bool supported = feetNow[0].contactObserved || feetNow[1].contactObserved;
            const float capsuleGround = capsule.position.z - settings.standingHeight * 0.5f;
            if (!jumpWindow && capsule.grounded && !supported) {
                if (result.bolFlightSeconds <= 0.0f) {
                    const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                    result.bolFlightTakeoffSpeed = std::hypot(v.x, v.y);
                    result.bolFlightTakeoffHeight = locomotion.physicalState().centerOfMassWorld.z - capsuleGround;
                    result.bolFlightApex = result.bolFlightTakeoffHeight;
                }
                // (Acima do chao da capsula: subindo uma escada o corpo sobe
                // com ela, e isso nao e saltinho.)
                result.bolFlightApex = std::max(result.bolFlightApex,
                    locomotion.physicalState().centerOfMassWorld.z - capsuleGround);
                result.bolFlightSeconds += dt;
                if (result.bolFlightSeconds > 0.25f) result.bolSuspendedSeconds += dt;
            } else {
                // Saltinho: voo bem mais longo que o da passada na mesma
                // velocidade (o clipe voa ~0,075 s no trote a 3 m/s e ~0,14 s
                // no sprint a 7,5) - 0,06 s a mais.
                // (Pela subida do centro de massa no voo: 2 cm alem da do voo
                // natural, g t^2/8. So pelo tempo, contava o pe pairando a
                // milimetros do chao numa parada ou inversao - nada visivel.)
                const float natural = 0.075f + std::clamp(result.bolFlightTakeoffSpeed - 3.0f, 0.0f, 4.5f) * 0.0145f;
                const float naturalRise = 9.81f * natural * natural / 8.0f;
                if (result.bolFlightSeconds > 0.0f && result.bolFlightApex - result.bolFlightTakeoffHeight > naturalRise + 0.02f)
                    ++result.bolHops;
                result.bolFlightPeak = std::max(result.bolFlightPeak, result.bolFlightSeconds);
                result.bolFlightSeconds = 0.0f;
            }
            if (!jumpWindow)
                result.bolPelvisRisePeak = std::max(result.bolPelvisRisePeak, after.links.front().position.z
                    - (capsule.position.z - settings.standingHeight * 0.5f) - profile.standingRootHeightMeters);
            if (boletimRequested.lengthSquared() > 0.25f) {
                const float wantedSpeed = boletimRequested.length();
                const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                result.bolSpeedSum += std::max(0.0f, (v.x * boletimRequested.x + v.y * boletimRequested.y) / wantedSpeed);
                result.bolRequestedSum += wantedSpeed;
            }
            const float weight = std::max(1.0f, profile.totalMassKg * 9.81f);
            result.bolAssistSum += physical.rootForceWorld.length() / weight;
            result.bolAssistPlanarSum += std::hypot(physical.rootForceWorld.x, physical.rootForceWorld.y) / weight;
            ++result.bolAssistSamples;
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
        {
            const bool airborneNow = locomotion.telemetry().state == CharacterLocomotionState3D::Airborne;
            const Vec3 comNow = locomotion.physicalState().centerOfMassWorld;
            if (airborneNow) result.airborneVelocity = locomotion.physicalState().centerOfMassVelocityWorld;
            if (result.wasAirborne && !airborneNow && result.landingAt < 0 && t > 0.5f) {
                result.landingAt = t;
                result.landingVelocity = result.airborneVelocity;
                Vec3 horizontal { result.landingVelocity.x, result.landingVelocity.y, 0.0f };
                const float hs = horizontal.length();
                const Vec3 dir = hs > 0.05f ? horizontal * (1.0f / hs) : Vec3 { 1.0f, 0.0f, 0.0f };
                Vec3 foot = after.links[feet[0]].position.z < after.links[feet[1]].position.z
                    ? after.links[feet[0]].position : after.links[feet[1]].position;
                const Vec3 leg = foot - comNow;
                result.landingLegAngle = std::atan2(dot(leg, dir), -leg.z) * 57.2958f;
                result.landingVelocityAngle = std::atan2(hs, std::max(0.01f, -result.landingVelocity.z)) * 57.2958f;
                const Vec3 up = after.links[chest].orientation.rotate({0, 0, 1});
                result.landingChestStart = std::atan2(dot(up, dir), up.z) * 57.2958f;
            }
            if (result.landingAt >= 0 && t - result.landingAt <= 1.0f) {
                Vec3 horizontal { result.landingVelocity.x, result.landingVelocity.y, 0.0f };
                const float hs = horizontal.length();
                const Vec3 dir = hs > 0.05f ? horizontal * (1.0f / hs) : Vec3 { 1.0f, 0.0f, 0.0f };
                const Vec3 up = after.links[chest].orientation.rotate({0, 0, 1});
                const float pitch = std::atan2(dot(up, dir), up.z) * 57.2958f;
                result.landingChestForward = std::max(result.landingChestForward, pitch);
                result.landingChestBack = std::min(result.landingChestBack, pitch);
                if (std::abs(t - result.landingAt - 0.5f) < 0.5f / 120) {
                    const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                    result.landingSpeedAfter = std::hypot(v.x, v.y);
                }
            }
            result.wasAirborne = airborneNow;
            if (std::getenv("XVIZ")) {
                const auto& o = controller.output();
                const bool hit = o.strongestImpulse > 2.0f;
                if (hit && !result.neighborWasHit) {
                    ++result.neighborHits;
                    if (o.strongestInteractionLink < profile.links.size())
                        ++result.neighborHitLinks[profile.links[o.strongestInteractionLink].id];
                }
                result.neighborWasHit = hit;
                if (o.perturbation > 0.1f) result.neighborPerturbedSeconds += dt;
                result.neighborEnvelopeMin = std::min(result.neighborEnvelopeMin, o.assistEnvelope);
                result.neighborImpulseMax = std::max(result.neighborImpulseMax, o.strongestImpulse);
            }
            if (std::getenv("XSERIES")) {
                const Vec3 v = locomotion.physicalState().centerOfMassVelocityWorld;
                const float hs = std::hypot(v.x, v.y);
                const Vec3 dir = hs > 0.5f ? Vec3 { v.x / hs, v.y / hs, 0.0f } : Vec3 { 1.0f, 0.0f, 0.0f };
                const Vec3 left { -dir.y, dir.x, 0.0f };
                const Vec3 up = after.links[chest].orientation.rotate({0, 0, 1});
                const Vec3 pelvisUp = after.links[0].orientation.rotate({0, 0, 1});
                result.seriesT.push_back(t);
                result.seriesChestPitch.push_back(std::atan2(dot(up, dir), up.z) * 57.2958f);
                result.seriesPelvisPitch.push_back(std::atan2(dot(pelvisUp, dir), pelvisUp.z) * 57.2958f);
                result.seriesUprightTorque.push_back(-dot(controller.output().rootTorqueWorld, left));
                result.seriesSpeed.push_back(hs);
            }
        }
        if (result.firstFallSeconds < 0 && result.gaitMoveStart >= 0 && t - result.gaitMoveStart > 1.2f
            && result.gaitMoveEnd >= t - 0.01f) {
            const auto joint = [&](const char* id) {
                for (std::size_t i = 0; i < profile.links.size(); ++i)
                    if (profile.links[i].id == id) {
                        const auto& link = profile.links[i];
                        return after.links[i].position + after.links[i].orientation.rotate(
                            link.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - link.modelPosition));
                    }
                return Vec3 {};
            };
            const Vec3 hips = joint("LeftThigh") - joint("RightThigh");
            const Vec3 shoulders = joint("LeftUpperArm") - joint("RightUpperArm");
            const float yaw = std::remainder(std::atan2(shoulders.y, shoulders.x) - std::atan2(hips.y, hips.x), 6.28318531f) * 57.2958f;
            const float roll = (std::atan2(shoulders.z, std::hypot(shoulders.x, shoulders.y))
                - std::atan2(hips.z, std::hypot(hips.x, hips.y))) * 57.2958f;
            result.trunkYawMin = std::min(result.trunkYawMin, yaw); result.trunkYawMax = std::max(result.trunkYawMax, yaw);
            result.trunkRollMin = std::min(result.trunkRollMin, roll); result.trunkRollMax = std::max(result.trunkRollMax, roll);
        }
        if (result.firstFallSeconds < 0 && t > 0.5f) {
            const Vec3 pelvisUp = after.links[0].orientation.rotate({0, 0, 1});
            const Vec3 chestUp = after.links[chest].orientation.rotate({0, 0, 1});
            result.spineBendPeak = std::max(result.spineBendPeak,
                std::acos(std::clamp(dot(pelvisUp, chestUp), -1.0f, 1.0f)) * 57.2958f);
            result.pelvisTiltPeak = std::max(result.pelvisTiltPeak, std::acos(std::clamp(pelvisUp.z, -1.0f, 1.0f)) * 57.2958f);
            result.chestTiltPeak = std::max(result.chestTiltPeak, std::acos(std::clamp(chestUp.z, -1.0f, 1.0f)) * 57.2958f);
            if (result.spineBendRest < 0 && t >= 1.4f) {
                result.spineBendRest = std::acos(std::clamp(dot(pelvisUp, chestUp), -1.0f, 1.0f)) * 57.2958f;
                result.chestTiltRest = std::acos(std::clamp(chestUp.z, -1.0f, 1.0f)) * 57.2958f;
                result.pelvisTiltRest = std::acos(std::clamp(pelvisUp.z, -1.0f, 1.0f)) * 57.2958f;
            }
        }
        if (result.risingStartAt < 0 && result.firstFallSeconds >= 0
            && locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Rising)
            result.risingStartAt = t;
        {
            // Agitacao no chao: so deitado (fase Settling).
            std::vector<float> targets;
            for (const auto& target : output.driveTargets) {
                const std::string& id = profile.links[target.linkIndex].id;
                if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("UpperArm") != std::string::npos || id.find("Forearm") != std::string::npos)
                    targets.push_back(target.positionRadians);
            }
            const bool lying = locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::Settling;
            const bool flying = locomotion.telemetry().lifeFlail > 0.3f;
            if (flying && result.previousTargets.size() == targets.size() && !targets.empty()) {
                double sum = 0;
                for (std::size_t i = 0; i < targets.size(); ++i)
                    sum += std::abs(targets[i] - result.previousTargets[i]) / dt * 57.2958;
                result.airTargetSpeedSum += sum / targets.size();
                double limb = 0; int limbs = 0;
                for (std::size_t i = 1; i < profile.links.size(); ++i) {
                    const std::string& id = profile.links[i].id;
                    if (id.find("Shin") != std::string::npos || id.find("Forearm") != std::string::npos) {
                        limb += after.links[i].angularVelocity.length() * 57.2958; ++limbs;
                    }
                }
                result.airLimbSpeedSum += limbs ? limb / limbs : 0;
                ++result.airSamples;
            }
            if (lying && result.previousTargets.size() == targets.size() && !targets.empty()) {
                double sum = 0;
                for (std::size_t i = 0; i < targets.size(); ++i)
                    sum += std::abs(targets[i] - result.previousTargets[i]) / dt * 57.2958;
                const float mean = static_cast<float>(sum / targets.size());
                result.groundTargetSpeedSum += mean;
                ++result.groundTargetSamples;
                result.groundTargetSpeedPeak = std::max(result.groundTargetSpeedPeak, mean);
                double limb = 0; int limbs = 0;
                for (std::size_t i = 1; i < profile.links.size(); ++i) {
                    const std::string& id = profile.links[i].id;
                    if (id.find("Shin") != std::string::npos || id.find("Forearm") != std::string::npos) {
                        limb += after.links[i].angularVelocity.length() * 57.2958; ++limbs;
                    }
                }
                result.groundLimbSpeedSum += limbs ? limb / limbs : 0;
                const float gather = locomotion.telemetry().settleGather;
                if (gather > result.lastGather + 0.01f) result.gatherRising = true;
                if (result.gatherRising && gather < result.lastGather - 0.05f) { ++result.gatherReversals; result.gatherRising = false; }
                result.lastGather = gather;
            }
            result.previousTargets = targets;
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
            const float uprightNow = after.links.front().orientation.rotate({0, 0, 1}).z;
            const bool airborneNow = locomotion.telemetry().state == CharacterLocomotionState3D::Airborne;
            if (!down && t > 0.6f) {
                std::size_t chest = 0;
                for (std::size_t i = 0; i < profile.links.size(); ++i) if (profile.links[i].id == "UpperChest") chest = i;
                const float chestUp = (after.links[chest].orientation * profile.links[chest].modelOrientation.conjugate())
                    .rotate({0, 0, 1}).z;
                result.worstChestTilt = std::max(result.worstChestTilt, std::acos(std::clamp(chestUp, -1.0f, 1.0f)) * 57.2958f);
            }
            if (!down && !airborneNow && uprightNow < 0.906f && t > 0.3f) {
                result.lostPostureSeconds += dt;
                if (result.lostEpisodeStart < 0) result.lostEpisodeStart = t;
            }
            const auto pendulumLean = [&]() {
                Vec3 base = (after.links[feet[0]].position + after.links[feet[1]].position) * 0.5f;
                const Vec3 arm = physical.centerOfMassWorld - base;
                return std::acos(std::clamp(arm.z / std::max(1e-3f, arm.length()), -1.0f, 1.0f)) * 57.2958f;
            };
            if (!down && !airborneNow && t > 0.6f) {
                const float lean = pendulumLean();
                result.worstPendulum = std::max(result.worstPendulum, lean);
                if (lean > 20.0f) {
                    result.pendulumHangSeconds += dt;
                    if (result.pendulumEpisodeStart < 0) result.pendulumEpisodeStart = t;
                }
            }
            if (result.pendulumEpisodeStart >= 0 && (down || pendulumLean() < 12.0f)) {
                result.longestPendulumEpisode = std::max(result.longestPendulumEpisode, t - result.pendulumEpisodeStart);
                result.pendulumEpisodeStart = -1;
            }
            if (std::getenv("XRESCUETRACE") && t > std::strtof(std::getenv("XRESCUETRACE"), nullptr) && static_cast<int>(t * 120.0f + 0.5f) % (std::getenv("XRESCUEEVERY") ? std::atoi(std::getenv("XRESCUEEVERY")) : 6) == 0)
                std::printf("  t %.2f pendulo %4.1f margem %5.2f inclinacao %4.1f peito %4.1f desvio %4.1f estado %d %.0f caiu %d pes %d%d fase %d recuperavel %.2f\n", t,
                    pendulumLean(), physical.balanceMargin,
                    std::acos(std::clamp(uprightNow, -1.0f, 1.0f)) * 57.2958f,
                    locomotion.telemetry().postureChestTilt * 57.2958f, locomotion.telemetry().postureDeviation * 57.2958f,
                    static_cast<int>(locomotion.telemetry().state),
                    0.0f, locomotion.telemetry().postureLost ? 1 : 0,
                    locomotion.telemetry().footPlanted[0] ? 1 : 0, locomotion.telemetry().footPlanted[1] ? 1 : 0,
                    static_cast<int>(controller.output().phase), controller.output().recoverability);
            if (result.lostEpisodeStart >= 0 && (down || uprightNow > 0.966f)) {
                result.longestLostEpisode = std::max(result.longestLostEpisode, t - result.lostEpisodeStart);
                result.lostEpisodeStart = -1;
            }
        }
        {
            // Caindo: o primeiro segmento (fora os pes) que toca o chao e a
            // velocidade da cabeca quando ela toca.
            const float uprightNow = after.links.front().orientation.rotate({0, 0, 1}).z;
            const bool falling = down || uprightNow < 0.8f;
            if (result.fallOnsetAt < 0 && uprightNow < 0.906f && t > 0.3f) result.fallOnsetAt = t;
            for (const auto& contact : after.interactions) {
                if (contact.normalWorld.z < 0.5f || contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id == "LeftFoot" || id == "RightFoot") continue;
                if (falling && result.firstGroundLink.empty()) result.firstGroundLink = id;
                if (falling && result.fallOnsetAt >= 0) {
                    const bool arm = id == "LeftHand" || id == "RightHand" || id == "LeftForearm" || id == "RightForearm";
                    const bool torso = id == "Pelvis" || id == "Abdomen" || id == "Chest" || id == "UpperChest";
                    if (arm && result.handGroundAt < 0) {
                        result.handGroundAt = t;
                        const std::size_t forearm = id.find("Forearm") != std::string::npos ? contact.linkIndex
                            : static_cast<std::size_t>(std::max(0, profile.links[contact.linkIndex].parentIndex));
                        if (forearm > 0) result.handGroundElbow = after.joints[forearm].positionRadians[0] * 57.2958f;
                        for (std::size_t i = 0; i < profile.links.size(); ++i)
                            if (profile.links[i].id == "Head") result.handGroundHeadHeight = after.links[i].position.z;
                    }
                    if (id == "Head" && result.headGroundAt < 0) result.headGroundAt = t;
                    if (torso && result.torsoGroundAt < 0) result.torsoGroundAt = t;
                }
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
                    if (telemetry.footPlanted[side] != planted) {
                        ++result.ownerMismatch;
                        if (std::getenv("XDONOTRACE"))
                            std::printf("DONO %.3f pe %zu fase %d plantado %d | bola fase %d roteiro %d pe %d swing %d\n", t,
                                side, static_cast<int>(foot.phase), telemetry.footPlanted[side] ? 1 : 0, lastBallPhase,
                                lastBallScript.active ? 1 : 0, lastBallScript.side, lastBallScript.swing ? 1 : 0);
                    }
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
                    if (std::getenv("XPASSOTRACE"))
                        std::printf("PASSO %.2f pe %zu motivo %d | torcao %+.1f falta %+.1f graus | de (%.2f %.2f) alvo (%.2f %.2f) | outro (%.2f %.2f)\n",
                            t, side, static_cast<int>(foot.reason), plan.standingTwist * 57.2958f,
                            plan.standingTurnRemaining * 57.2958f, foot.anchorWorld.x, foot.anchorWorld.y,
                            foot.targetWorld.x, foot.targetWorld.y, plan.feet[1 - side].anchorWorld.x,
                            plan.feet[1 - side].anchorWorld.y);
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
    std::cout << " | postura perdida " << r.lostPostureSeconds << " s (maior " << r.longestLostEpisode << " s)";
    std::cout << " | pendulo max " << r.worstPendulum << " graus, " << r.pendulumHangSeconds << " s acima de 20 (maior "
              << r.longestPendulumEpisode << " s)";
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
        std::printf("SWEEP %s %.1f %3d quedas %d tilt %.1f passos %d segurado %.2f recuperacao %d levantados %d arrastados %d | coluna dobra %.1f pelve %.1f peito %.1f\n",
            kind, speed, heading, r.falls, r.worstTilt, r.steps, r.heldOutsideSeconds, recoverySteps, lifted, dragged,
            r.spineBendPeak - r.spineBendRest, r.pelvisTiltPeak - r.pelvisTiltRest, r.chestTiltPeak - r.chestTiltRest);
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
            // Virada correndo (XGAITCUT graus, + para a esquerda): aos 3,5 s o
            // comando gira esse angulo; mede quando o corpo vai a 2,5 m/s no
            // rumo novo (a referencia Running Turn 180 chega a 2,4 m/s em
            // ~0,55 s).
            const float cutDegrees = std::getenv("XGAITCUT") ? std::strtof(std::getenv("XGAITCUT"), nullptr) : 0.0f;
            const Vec3 cutDirection = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                cutDegrees * 3.14159265f / 180.0f).rotate(direction);
            // Camera balancando (XGAITWIGGLE graus, 0,7 Hz) com o movimento
            // relativo a ela (W no laboratorio): curvas para um lado e outro.
            const float wiggle = std::getenv("XGAITWIGGLE")
                ? std::strtof(std::getenv("XGAITWIGGLE"), nullptr) * 3.14159265f / 180.0f : 0.0f;
            Scenario scenario {"varredura de marcha", 7.0f, {}, [=](float t) {
                ScenarioCommand c;
                if (t >= 2.0f && t < 5.0f) c.move = invert && t >= 3.5f ? direction * -1.0f : direction;
                if (cutDegrees != 0.0f && t >= 3.5f && t < 5.0f) {
                    c.move = cutDirection;
                    c.lookYaw = std::atan2(cutDirection.y, cutDirection.x);
                }
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
            if (wiggle > 0.0f) {
                // So para o rastreio da curva (XTCTRACE) na janela do zigue-zague.
                scenario.turnStart = 2.5f;
                scenario.turnEnd = 4.0f;
                scenario.turnDirection = direction;
            }
            if (cutDegrees != 0.0f) {
                scenario.probeStart = 3.5f;
                scenario.probeDirection = cutDirection;
                scenario.probeSpeed = 2.5f;
            }
            const ScenarioResult r = runScenario(scenario);
            if (cutDegrees != 0.0f)
                std::printf("CORTE %s %3d %+4.0f quedas %d resposta %.2f inclinacao %.1f\n", sprint ? "sprint" : "trote",
                    heading, cutDegrees, r.falls, r.probeReachedAt >= 0.0f ? r.probeReachedAt - 3.5f : -1.0f, r.worstTilt);
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
            if (std::getenv("XTRUNK"))
                std::printf("TRONCO %s %3d contra-rotacao +-%.1f inclinacao lateral +-%.1f\n", sprint ? "sprint" : "trote", heading,
                    (r.trunkYawMax - r.trunkYawMin) / 2, (r.trunkRollMax - r.trunkRollMin) / 2);
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

// Varredura de curva (sweepcurve): corre reto 1,5 s e entao a camera gira
// a taxa constante (graus/s) com o comando para a frente dela - uma curva
// de raio v/taxa, como a referencia Sprint Turn. Na janela da curva ja
// estabelecida, mede a inclinacao do tronco (pelve -> pescoco, lado + para
// a esquerda do rumo), a pelve adiantada no rumo, o atraso do rumo do corpo
// para o pedido e, nos pes apoiados, o giro do pe em relacao ao rumo, o
// comprimento da perna e a altura do quadril. Taxa 0: a corrida reta (a
// abertura natural dos pes).
void curveSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const bool sprint : { false, true }) {
        for (const float rate : { 0.0f, 45.0f, 90.0f, 135.0f, -45.0f, -90.0f, -135.0f }) {
            if (index++ % parts != part) continue;
            const float omega = rate * 3.14159265f / 180.0f;
            Scenario scenario {"varredura de curva", 6.0f, {}, [=](float t) {
                ScenarioCommand c;
                const float turn = t > 3.5f ? omega * (t - 3.5f) : 0.0f;
                c.lookYaw = turn;
                if (t >= 2.0f) c.move = Vec3 { std::cos(turn), std::sin(turn), 0.0f };
                c.sprint = sprint;
                return c;
            }, {}};
            scenario.curveFrom = rate == 0.0f ? 3.0f : 4.1f;
            scenario.curveTo = 5.6f;
            // Janela propria (XCURVEWIN "de,ate"): o comeco da curva, 3,5 s.
            if (const char* w = std::getenv("XCURVEWIN"))
                std::sscanf(w, "%f,%f", &scenario.curveFrom, &scenario.curveTo);
            const ScenarioResult r = runScenario(scenario);
            const double n = std::max(1, r.curveSamples);
            std::printf("CURVA %s %+4.0f quedas %d v %.2f pedido %.2f a_c %+.1f | tronco lado %+5.1f frente %+5.1f | pelve adiantada %+5.1f | atraso %+5.1f",
                sprint ? "sprint" : "trote", rate, r.falls, r.curveSpeed / n, r.curveRequestedSpeed / n,
                r.curveAcceleration / n, r.curveTrunkSide / n, r.curveTrunkForward / n, r.curvePelvisLead / n,
                r.curveLag / n);
            for (std::size_t side = 0; side < 2; ++side) {
                const double m = std::max(1, r.curveStance[side]);
                std::printf(" | pe %s apoio %3d giro %+5.1f corpo %+5.1f perna %.3f quadril %.3f", side == 0 ? "E" : "D",
                    r.curveStance[side], r.curveFootYaw[side] / m, r.curveBodyLean[side] / m, r.curveLeg[side] / m,
                    r.curveHipHeight[side] / m);
            }
            std::printf(" | bicudas %d | toques dentro %d fora %d | baixos dentro %d/%d fora %d/%d\n", r.kicks,
                r.swingTouches[0], r.swingTouches[1], r.lowSwings[0], r.swingsCounted[0], r.lowSwings[1], r.swingsCounted[1]);
        }
    }
}

// Varredura de quedas (sweepfall): derruba de verdade em 8 rumos - nasce
// tombado 60 graus para o rumo e empurrado por um bloco de 60 kg a 5 m/s
// (3 instantes) - e mede a protecao: o que toca o chao primeiro, quando as
// maos chegam (desde o inicio da queda, pelve a 25 graus) em relacao a
// cabeca e ao tronco, a altura da cabeca quando a mao toca e a velocidade
// da cabeca no impacto (referencia Tripping: maos no chao com a cabeca a
// ~0,5 m, cabeca so chega devagar no fim).
void fallSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    const auto print = [](const char* kind, int heading, float delay, const ScenarioResult& r) {
        const float onset = r.fallOnsetAt;
        const auto rel = [&](float at) { return at >= 0 && onset >= 0 ? at - onset : -1.0f; };
        std::printf("QUEDA %s %3d %.2f quedas %d primeiro %s maos %.2f cabeca %.2f tronco %.2f | cabeca quando a mao toca %.2f | cabeca %.2f m/s | comecou a levantar %.2f de pe %.2f recaidas %d em-pe-no-fim %d | cotovelo no toque %.0f\n",
            kind, heading, delay, r.falls, r.firstGroundLink.empty() ? "-" : r.firstGroundLink.c_str(),
            rel(r.handGroundAt), rel(r.headGroundAt), rel(r.torsoGroundAt), r.handGroundHeadHeight,
            r.headImpactSpeed, r.risingStartAt >= 0 && r.firstFallSeconds >= 0 ? r.risingStartAt - r.firstFallSeconds : -1.0f,
            r.getUpSeconds, r.relapses, r.standingAtEnd ? 1 : 0, r.handGroundElbow);
        std::printf("CHAO %s %3d %.2f alvo %.0f graus/s (pico %.0f) membros %.0f graus/s | arrumar voltou %d | ar %.2f s alvo %.0f membros %.0f graus/s\n", kind, heading, delay,
            r.groundTargetSamples ? r.groundTargetSpeedSum / r.groundTargetSamples : 0.0,
            r.groundTargetSpeedPeak, r.groundTargetSamples ? r.groundLimbSpeedSum / r.groundTargetSamples : 0.0,
            r.gatherReversals, r.airSamples / 120.0, r.airSamples ? r.airTargetSpeedSum / r.airSamples : 0.0,
            r.airSamples ? r.airLimbSpeedSum / r.airSamples : 0.0);
    };
    for (int heading = 0; heading < 360; heading += 45) {
        if (index++ % parts != part) continue;
        Scenario scenario {"queda tombado", 9.0f, {}, {}, {}};
        scenario.spawnPitchDegrees = 60.0f;
        scenario.spawnTiltHeadingDegrees = static_cast<float>(heading);
        print("tombado", heading, 0.0f, runScenario(scenario));
    }
    // Do alto (pelve a 3 m, tombado 30 graus): o tempo no ar.
    for (int heading = 0; heading < 360; heading += 90) {
        if (index++ % parts != part) continue;
        Scenario scenario {"queda do alto", 9.0f, {}, {}, {}};
        scenario.spawnPitchDegrees = 30.0f;
        scenario.spawnTiltHeadingDegrees = static_cast<float>(heading);
        scenario.spawnPelvisHeight = 3.0f;
        print("alto", heading, 0.0f, runScenario(scenario));
    }
    for (const float delay : { 0.0f, 0.13f, 0.27f }) {
        for (int heading = 0; heading < 360; heading += 45) {
            if (index++ % parts != part) continue;
            auto block = std::make_shared<PhysicsBodyHandle3D>();
            // O bloco vem do lado oposto e empurra para o rumo.
            const float a = heading * 3.14159265f / 180;
            const Vec3 from {-std::cos(a), -std::sin(a), 0};
            Scenario scenario {"queda empurrado", 9.5f, {}, {},
                [=](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
                    if (std::abs(t - 1.5f - delay) >= 0.5f / 120) return;
                    const Vec3 chest = state.links.front().position;
                    *block = dynamicBox(scene, {chest.x + from.x * 0.55f, chest.y + from.y * 0.55f, 1.25f},
                        {0.12f, 0.25f, 0.15f}, 60, from * -5.0f);
                }};
            print("empurrado", heading, delay, runScenario(scenario));
        }
    }
}

// Varredura de pouso (sweeppouso): sai correndo de uma plataforma de 1 m
// ou pula correndo no plano, em trote e sprint, de frente, de lado e de
// costas (a camera gira; o movimento e sempre +X). Mede o pouso: o angulo
// da perna e o da velocidade (a partir da vertical, no rumo do movimento),
// o quanto o peito tomba para a frente e para tras no 1 s seguinte, quedas
// e a velocidade 0,5 s depois. Perna alinhada com a velocidade: a forca do
// chao passa pelo centro de massa - nem joga para a frente nem para tras.
void landingSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const bool ledge : { true, false }) {
        for (const bool sprint : { false, true }) {
            for (const int view : { 0, 90, -90, 180 }) {
                if (index++ % parts != part) continue;
                const float yaw = view * 3.14159265f / 180.0f;
                Scenario scenario {"pouso", 6.0f,
                    [ledge](PhysicsScene3D& scene) {
                        if (ledge) staticBox(scene, {-3.0f, 0.0f, 0.5f}, {5.5f, 3.0f, 0.5f});
                    },
                    [=](float t) {
                        ScenarioCommand c;
                        c.lookYaw = yaw;
                        if (t > 0.6f && t < 4.5f) c.move = {1, 0, 0};
                        c.sprint = sprint;
                        if (!ledge) c.jump = t >= 2.2f && t < 2.21f;
                        return c;
                    }, {}};
                if (ledge) scenario.spawnPelvisHeight = 1.0f + 1.0f;
                const ScenarioResult r = runScenario(scenario);
                std::printf("POUSO %s %s %+4d quedas %d | velocidade %.1f m/s a %4.1f graus da vertical | perna %+5.1f graus | peito no toque %+5.1f, depois ate %+5.1f a frente e %+5.1f | 0,5 s depois %.1f m/s\n",
                    ledge ? "plataforma" : "pulo", sprint ? "sprint" : "trote", view, r.falls,
                    r.landingVelocity.length(), r.landingVelocityAngle, r.landingLegAngle, r.landingChestStart,
                    r.landingChestForward, r.landingChestBack, r.landingSpeedAfter);
            }
        }
    }
}

// Pulo forte para a frente e segue correndo (sweeppulofrente): sprint ou
// trote, pulo tocado ou carregado, e a corrida continua. Mede a inclinacao do
// peito antes do pulo e depois do pouso, quanto tempo ele leva para voltar a
// postura da corrida e o torque de postura da ajuda (o quanto ela o segura).
void forwardJumpSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    setenv("XSERIES", "1", 1);
    int index = 0;
    for (const bool sprint : { true, false }) {
        for (const float charge : { 0.0f, 0.35f, 0.7f }) {
            for (const int view : { 0, 30, -30 }) {
                if (index++ % parts != part) continue;
                const float yaw = view * 3.14159265f / 180.0f;
                const float jumpAt = 2.4f + charge;
                Scenario scenario {"pulofrente", jumpAt + 4.0f, {}, [=](float t) {
                    ScenarioCommand c;
                    c.lookYaw = yaw;
                    c.move = t > 0.6f ? Vec3 { std::cos(yaw), std::sin(yaw), 0.0f } : Vec3 {};
                    c.sprint = sprint;
                    if (charge > 0.0f) c.jumpCharging = t >= 2.4f && t < jumpAt;
                    c.jump = t >= jumpAt && t < jumpAt + 0.01f;
                    c.jumpChargeSeconds = charge;
                    return c;
                }, {}};
                if (std::getenv("XPULOSERIE")) { scenario.postureFrom = 2.0f; scenario.postureTo = jumpAt + 4.0f; }
                const ScenarioResult r = runScenario(scenario);
                double base = 0, baseTorque = 0; int nb = 0;
                double post = 0, postTorque = 0; int np = 0;
                double pelvisBase = 0, pelvisPost = 0;
                float worst = -90, recovered = -1, worstTorque = 0;
                float holdStart = -1;
                for (std::size_t i = 0; i < r.seriesT.size(); ++i) {
                    const float t = r.seriesT[i];
                    if (t >= 1.6f && t < 2.35f) {
                        base += r.seriesChestPitch[i]; baseTorque += r.seriesUprightTorque[i];
                        pelvisBase += r.seriesPelvisPitch[i]; ++nb;
                    }
                }
                if (nb > 0) { base /= nb; baseTorque /= nb; pelvisBase /= nb; }
                for (std::size_t i = 0; i < r.seriesT.size() && r.landingAt >= 0; ++i) {
                    const float since = r.seriesT[i] - r.landingAt;
                    if (since < 0.0f) continue;
                    if (since <= 1.5f) {
                        worst = std::max(worst, r.seriesChestPitch[i]);
                        worstTorque = std::max(worstTorque, r.seriesUprightTorque[i]);
                    }
                    if (since >= 0.2f && since <= 1.4f) {
                        post += r.seriesChestPitch[i]; postTorque += r.seriesUprightTorque[i];
                        pelvisPost += r.seriesPelvisPitch[i]; ++np;
                    }
                    if (since >= 0.15f && recovered < 0.0f) {
                        const bool near = std::abs(r.seriesChestPitch[i] - static_cast<float>(base)) < 5.0f;
                        if (near && holdStart < 0.0f) holdStart = since;
                        if (!near) holdStart = -1.0f;
                        if (holdStart >= 0.0f && since - holdStart >= 0.25f) recovered = holdStart;
                    }
                }
                if (np > 0) { post /= np; postTorque /= np; pelvisPost /= np; }
                if (std::getenv("XPULOSERIE"))
                    for (std::size_t i = 0; i < r.seriesT.size(); i += 3)
                        if (r.seriesT[i] > jumpAt - 0.3f && r.seriesT[i] < jumpAt + 2.6f)
                            std::printf("SERIE %.3f peito %+6.1f pelve %+6.1f ajuda %5.0f v %.2f\n", r.seriesT[i],
                                r.seriesChestPitch[i], r.seriesPelvisPitch[i], r.seriesUprightTorque[i], r.seriesSpeed[i]);
                std::printf("PULOFRENTE %s carga %.2f %+3d quedas %d | pouso %.2f s | peito antes %+5.1f depois %+5.1f (pior %+5.1f) pelve %+5.1f -> %+5.1f | volta em %.2f s | ajuda postura antes %4.0f depois %4.0f (pico %4.0f) N m\n",
                    sprint ? "sprint" : "trote", charge, view, r.falls, r.landingAt, base, post, worst,
                    pelvisBase, pelvisPost, recovered, baseTorque, postTorque, worstTorque);
            }
        }
    }
}

// Perto de outro corpo (sweepvizinho): passa correndo ao lado de outro
// boneco parado e de uma parede, a varias distancias, e fica parado ao lado
// do boneco. Mede quedas e o quanto a ajuda se ve "atingida" (contato
// externo), o tempo perturbado e o menor envelope da ajuda.
void neighborSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    setenv("XVIZ", "1", 1);
    struct Case { const char* kind; float gap; bool sprint; };
    std::vector<Case> cases;
    for (const float gap : { 0.20f, 0.30f, 0.45f })
        for (const bool sprint : { false, true })
            cases.push_back({ "boneco", gap, sprint });
    cases.push_back({ "parede", 0.25f, false });
    cases.push_back({ "parede", 0.25f, true });
    // Encosta: parado a 0,9 m, anda de lado devagar ate o outro e segue
    // empurrando um pouco.
    cases.push_back({ "encosta", 0.90f, false });
    int index = 0;
    for (const Case& item : cases) {
        if (index++ % parts != part) continue;
        const bool wall = std::string(item.kind) == "parede";
        const bool still = std::string(item.kind) == "encosta";
        const float gap = item.gap;
        Scenario scenario {item.kind, still ? 6.0f : 7.0f,
            [=](PhysicsScene3D& scene) {
                // Parede de 8 m ao lado do caminho (a face a `gap` do centro).
                if (wall) staticBox(scene, {6.0f, gap + 0.15f, 1.0f}, {4.0f, 0.15f, 1.0f});
            },
            [=](float t) {
                ScenarioCommand c;
                if (!still && t > 0.6f) c.move = {1, 0, 0};
                if (still && t > 1.0f && t < 4.5f) c.move = {0, 0.35f, 0};
                c.sprint = item.sprint;
                return c;
            }, {}};
        if (!wall) {
            scenario.npc = true;
            scenario.npcPosition = still ? Vec3 { 0.0f, gap, 0.0f } : Vec3 { item.sprint ? 9.0f : 5.0f, gap, 0.0f };
            scenario.npcYaw = 1.5708f;
        }
        const ScenarioResult r = runScenario(scenario);
        std::string links;
        for (const auto& [id, n] : r.neighborHitLinks) links += " " + id + ":" + std::to_string(n);
        std::printf("VIZINHO %-6s %.2f m %-6s quedas %d (outro %d) | batidas %d, perturbado %.2f s, envelope min %.2f, impulso max %.1f N s |%s\n",
            item.kind, gap, item.sprint ? "sprint" : "trote", r.falls, r.npcFalls, r.neighborHits,
            r.neighborPerturbedSeconds, r.neighborEnvelopeMin, r.neighborImpulseMax, links.c_str());
    }
}

// Transicoes intercaladas (sweeptransicao): sequencias bruscas de comando
// com a camera parada - frente, lados, costas, sprint, alternando - em 4
// rumos no mundo. Mede quedas, a pior inclinacao, pes cruzados e embolados,
// saltinhos e a resposta media/pior a cada troca. XTRANSCASO=nome roda so um.
void transitionSweep() {
    struct Segment { float degrees; float seconds; bool sprint; };
    struct Case { const char* name; std::vector<Segment> segments; };
    const std::vector<Case> cases = {
        {"frente>esquerda", {{0, 2.0f, false}, {90, 2.0f, false}}},
        {"frente>direita", {{0, 2.0f, false}, {-90, 2.0f, false}}},
        {"frente>costas", {{0, 2.0f, false}, {180, 2.0f, false}}},
        {"costas>frente", {{180, 2.0f, false}, {0, 2.0f, false}}},
        {"esquerda>direita", {{90, 1.5f, false}, {-90, 1.5f, false}, {90, 1.5f, false}}},
        {"lado>frente", {{90, 1.5f, false}, {0, 2.0f, false}}},
        {"diagonais", {{45, 1.0f, false}, {-45, 1.0f, false}, {135, 1.0f, false}, {-135, 1.0f, false}, {45, 1.0f, false}}},
        {"roda", {{0, 0.8f, false}, {90, 0.8f, false}, {180, 0.8f, false}, {-90, 0.8f, false}, {0, 0.8f, false}}},
        {"sprint>esquerda", {{0, 2.2f, true}, {90, 2.0f, false}}},
        {"sprint>costas", {{0, 2.2f, true}, {180, 2.0f, false}}},
        {"sprint>diagonal", {{0, 2.2f, true}, {-45, 2.0f, true}}},
        {"trote>sprint>lado", {{0, 1.2f, false}, {0, 1.5f, true}, {-90, 1.5f, false}}},
        {"vaivem", {{0, 0.6f, false}, {180, 0.6f, false}, {0, 0.6f, false}, {180, 0.6f, false}, {0, 0.8f, false}}},
        {"zigue", {{30, 0.7f, false}, {-30, 0.7f, false}, {30, 0.7f, false}, {-30, 0.7f, false}, {30, 0.7f, false}}},
    };
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const char* only = std::getenv("XTRANSCASO");
    int index = 0;
    for (const Case& item : cases) {
        if (only && std::string(only) != item.name) continue;
        for (int heading = 0; heading < 360; heading += 90) {
            if (index++ % parts != part) continue;
            const float base = heading * 3.14159265f / 180.0f;
            float total = 1.0f;
            for (const auto& s : item.segments) total += s.seconds;
            const std::vector<Segment> segments = item.segments;
            Scenario scenario {item.name, total + 2.0f, {}, [=](float t) {
                ScenarioCommand c;
                c.lookYaw = base;
                float start = 1.0f;
                for (const auto& s : segments) {
                    if (t >= start && t < start + s.seconds) {
                        const float a = base + s.degrees * 3.14159265f / 180.0f;
                        c.move = { std::cos(a), std::sin(a), 0.0f };
                        c.sprint = s.sprint;
                    }
                    start += s.seconds;
                }
                return c;
            }, {}};
            scenario.transitionMetrics = true;
            const ScenarioResult r = runScenario(scenario);
            std::printf("TRANS %-18s %3d quedas %d inclinacao %.1f | cruzados %d embolados %d ticks | saltinhos %d (acima da ref max %.3f m, fora das trocas %.3f) | resposta media %.2f pior %.2f s em %d trocas\n",
                item.name, heading, r.falls, r.worstTilt, r.transCrossTicks, r.transTouchTicks, r.transHops, r.transPeakUp, r.transSteadyUp,
                r.transChanges ? r.transResponseSum / r.transChanges : -1.0, r.transResponseWorst, r.transChanges);
        }
    }
}

// Curva fechada correndo (sweepcurvafechada): corre reto e aos 3 s o mouse
// gira a camera - e o comando para a frente dela - `degrees` graus a `rate`
// graus/s (a virada brusca do laboratorio); depois segue reto no rumo novo.
// Mede quedas, a menor velocidade na virada, quando o corpo vai a 60% do
// pedido no rumo novo, o giro do pe apoiado no chao (pivo), o deslize da sola
// e a freada. XTCCASO filtra um caso.
void tightCurveSweep() {
    struct Case { const char* name; float degrees; float rate; };
    const std::vector<Case> cases = {
        {"60@600", 60, 600}, {"90@360", 90, 360}, {"90@720", 90, 720}, {"135@450", 135, 450},
        {"180@600", 180, 600}, {"360@180", 360, 180}, {"360@300", 360, 300}};
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const char* only = std::getenv("XTCCASO");
    const char* onlyGait = std::getenv("XTCMARCHA");
    int index = 0;
    for (const bool sprint : { true, false }) {
        if (onlyGait && std::string(onlyGait) != (sprint ? "sprint" : "trote")) continue;
        for (const Case& item : cases) {
            if (only && std::string(only) != item.name) continue;
            for (const float sign : { 1.0f, -1.0f }) {
                const int headingStep = std::getenv("XTCRUMOS") ? std::atoi(std::getenv("XTCRUMOS")) : 120;
                for (int heading = 0; heading < 360; heading += headingStep) {
                    if (index++ % parts != part) continue;
                    const float base = heading * 3.14159265f / 180.0f;
                    const float span = item.degrees * 3.14159265f / 180.0f * sign;
                    const float omega = item.rate * 3.14159265f / 180.0f * sign;
                    const float start = 3.0f;
                    const float end = start + span / omega;
                    Scenario scenario {"curva fechada", end + 2.5f, {}, [=](float t) {
                        ScenarioCommand c;
                        const float turn = t < start ? 0.0f : t < end ? omega * (t - start) : span;
                        c.lookYaw = base + turn;
                        if (t >= 1.0f) c.move = { std::cos(c.lookYaw), std::sin(c.lookYaw), 0.0f };
                        c.sprint = sprint;
                        return c;
                    }, {}};
                    scenario.turnStart = start;
                    scenario.turnEnd = end;
                    scenario.turnDirection = { std::cos(base + span), std::sin(base + span), 0.0f };
                    const ScenarioResult r = runScenario(scenario);
                    std::printf("TC %-8s %-6s %+2.0f %3d quedas %d incl %4.1f | v antes %.2f min %.2f | resposta %.2f | pivo media %4.1f pior %4.1f (>30: %d de %d) | deslize %.3f | freada %d cruzados %d | alarmes %d em %d | bicudas %d | toques dentro %d fora %d | baixos dentro %d/%d fora %d/%d | saida %d fim %d\n",
                        item.name, sprint ? "sprint" : "trote", sign, heading, r.falls, r.worstTilt,
                        r.tcSpeedBefore, r.tcMinSpeed < 1e8f ? r.tcMinSpeed : -1.0f, r.tcResponse,
                        r.tcStances ? r.tcPivotSum / r.tcStances : 0.0, r.tcPivotWorst, r.tcPivotOver30, r.tcStances,
                        r.tcStances ? r.tcSlideSum / r.tcStances : 0.0, r.tcBrakeTicks, r.tcCrossTicks, r.stumbleTicks, r.stumbleEvents, r.kicksInTurn,
                        r.swingTouches[0], r.swingTouches[1], r.lowSwings[0], r.swingsCounted[0], r.lowSwings[1], r.swingsCounted[1],
                        r.swingTouchesEarly, r.swingTouchesLate);
                }
            }
        }
    }
}

// Tropeco correndo (sweeptropeco): corre para +x (trote ou sprint) e acha
// uma barra baixa atravessada que os pes nao veem (dinamica, 300 kg), em 8
// posicoes ao longo de uma passada (cada uma pega o pe numa fase) e duas
// alturas. Mede quedas, a maior inclinacao para a frente, a pelve mais baixa
// e a velocidade antes, na pior hora e 2 s depois. XTROPECOALT filtra a
// altura (cm) e XTROPECOMARCHA o ritmo.
void stumbleSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const char* onlyGait = std::getenv("XTROPECOMARCHA");
    const int onlyHeight = std::getenv("XTROPECOALT") ? std::atoi(std::getenv("XTROPECOALT")) : 0;
    // Modo puxao (XTROPECOMODO=puxao): o pe no ar enganchado (forca para
    // tras), em vez da barra; "altura" vira a forca (fraco/forte).
    const bool snag = std::getenv("XTROPECOMODO") && std::string(std::getenv("XTROPECOMODO")) == "puxao";
    int index = 0;
    for (const bool sprint : { false, true }) {
        if (onlyGait && std::string(onlyGait) != (sprint ? "sprint" : "trote")) continue;
        for (const int heightCm : { 20, 26 }) {
            if (onlyHeight && onlyHeight != heightCm) continue;
            const int positions = std::getenv("XTROPECOPOS") ? std::atoi(std::getenv("XTROPECOPOS")) : 16;
            for (int k = 0; k < positions * 2; ++k) {
                if (index++ % parts != part) continue;
                const float step = sprint ? 1.74f : 1.10f;
                const float x = (sprint ? 13.0f : 6.0f) + step * (k % positions) / static_cast<float>(positions);
                // Duas sementes de variacao (o nome do cenario e a semente).
                const char* name = k < positions ? "tropeco" : "tropeco b";
                const float h = heightCm * 0.01f;
                const bool bare = std::getenv("XTROPECOSEM") != nullptr;
                Scenario scenario {name, 5.5f, [=](PhysicsScene3D& scene) {
                    if (!bare && !snag) dynamicBox(scene, { x, 0.0f, h * 0.5f }, { 0.05f, 1.5f, h * 0.5f }, 300.0f, {});
                }, [=](float t) {
                    ScenarioCommand c;
                    if (t >= 0.5f) c.move = { 1.0f, 0.0f, 0.0f };
                    c.sprint = sprint;
                    return c;
                }, {}};
                scenario.unseenObstacles = true;
                if (snag) {
                    // Uma passada inteira (dois passos) em fases.
                    const float cycle = sprint ? 0.465f : 0.733f;
                    scenario.snagAt = (sprint ? 2.6f : 2.2f) + cycle * (k % positions) / static_cast<float>(positions);
                    scenario.snagNewtons = (sprint ? 1.4f : 1.0f) * (heightCm == 20 ? 1800.0f : 3000.0f) * (std::getenv("XPUXAO") ? std::strtof(std::getenv("XPUXAO"), nullptr) : 1.0f);
                } else {
                    scenario.stumbleX = x;
                }
                const ScenarioResult r = runScenario(scenario);
                std::printf("TROPECO %-6s %2dcm %d quedas %d | inclinacao %5.1f | pelve min %.2f | v antes %.2f min %.2f depois %.2f | alarmes antes %d ticks, total %d em %d | postura perdida %.2f s maior %.2f s | pendulo max %.1f, %.2f s acima de 20 (maior %.2f s)\n",
                    sprint ? "sprint" : "trote", heightCm, k, r.falls, r.stPitchPeak, r.stLowest, r.stSpeedBefore,
                    r.stSpeedMin < 1e8f ? r.stSpeedMin : -1.0f, r.stSpeedAfter, r.stumbleTicksBefore, r.stumbleTicks, r.stumbleEvents,
                    r.lostPostureSeconds, r.longestLostEpisode, r.worstPendulum, r.pendulumHangSeconds, r.longestPendulumEpisode);
                if (std::getenv("XTROPECOSEM")) std::printf("  peito max %.1f graus\n", r.worstChestTilt);
            }
        }
    }
}

// Volta do fisico ao padrao (sweepvolta): o corpo fisico andando, capturado
// em instantes ao longo de uma passada; em cada um, o padrao comeca (com a
// semente do corpo ou do zero, XVOLTASEM) e a pose dele e misturada a partir
// da do corpo em 0,35 s, como no laboratorio. Mede a distancia da primeira
// pose do padrao ao corpo (juntas das pernas, pes em relacao a pelve, rumo)
// e o pico de velocidade das juntas das pernas na mistura.
void modeSwitchSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const RagdollProfile3D& profile = assets.character.profile;
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    std::size_t feet[2] = { 0, 0 };
    std::vector<bool> leg(profile.links.size(), false);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id == "LeftFoot") feet[0] = i;
        if (id == "RightFoot") feet[1] = i;
        leg[i] = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
            || id.find("Foot") != std::string::npos;
    }
    struct Case { const char* name; Vec3 move; bool sprint; };
    const Case cases[] = { { "parado", {}, false }, { "trote", { 1, 0, 0 }, false },
        { "trote lado", { 0, 1, 0 }, false }, { "trote costas", { -1, 0, 0 }, false },
        { "sprint", { 1, 0, 0 }, true } };
    const bool seedOff = std::getenv("XVOLTASEM") != nullptr;
    for (const Case& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"); only && std::string(only) != c.name) continue;
        std::vector<std::pair<float, RagdollState3D>> captured;
        Scenario scenario { c.name, 3.2f, nullptr, [=](float t) {
            ScenarioCommand command;
            if (t >= 0.5f) command.move = c.move;
            command.sprint = c.sprint;
            return command;
        }, {}};
        scenario.disturb = [&](PhysicsScene3D&, float t, const RagdollState3D& state) {
            if (t >= 2.2f && t < 3.0f && static_cast<int>(t * 120.0f + 0.5f) % 4 == 0) captured.push_back({ t, state });
        };
        runScenario(scenario);
        double jointSum = 0, feetSum = 0, yawSum = 0;
        float jointWorst = 0, feetWorst = 0, speedWorst = 0, jerkWorst = 0;
        double speedSum = 0;
        for (const auto& [t, state] : captured) {
            DefaultLocomotion3D locomotion;
            locomotion.reset(0.0f);
            const PhysicsBodyState3D& pelvis = state.links.front();
            const Vec3 velocity { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f };
            if (!seedOff) {
                DefaultLocomotionSeed3D seed;
                const Vec3 forward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
                    .rotate({ 1.0f, 0.0f, 0.0f });
                seed.pelvisYawRadians = std::atan2(forward.y, forward.x);
                seed.velocityWorld = velocity;
                seed.pelvisWorld = pelvis.position;
                seed.pelvisVelocityWorld = pelvis.linearVelocity;
                seed.feetWorld = { state.links[feet[0]].position, state.links[feet[1]].position };
                seed.feetVelocityWorld = { state.links[feet[0]].linearVelocity, state.links[feet[1]].linearVelocity };
                seed.feetValid = true;
                locomotion.seed(seed);
            }
            std::vector<Vec3> physical(profile.links.size()), physicalVelocity(profile.links.size());
            for (std::size_t i = 1; i < physical.size() && i < state.joints.size(); ++i) {
                physical[i] = { state.joints[i].positionRadians[0], state.joints[i].positionRadians[1],
                    state.joints[i].positionRadians[2] };
                physicalVelocity[i] = { state.joints[i].velocityRadiansPerSecond[0],
                    state.joints[i].velocityRadiansPerSecond[1], state.joints[i].velocityRadiansPerSecond[2] };
            }
            PoseInertializer3D inertia;
            if (!seedOff)
                inertia.start(physical, physicalVelocity, pelvis.position, pelvis.orientation, pelvis.linearVelocity,
                    pelvis.angularVelocity, 0.10f, 0.45f);
            DefaultLocomotionInput3D input;
            input.feetPositionWorld = { pelvis.position.x, pelvis.position.y, 0.0f };
            input.velocityWorld = velocity;
            input.requestedVelocityWorld = velocity;
            input.deltaTime = 1.0f / 120.0f;
            std::vector<Vec3> previous = physical, previousTarget = physical;
            for (int tick = 0; tick < 48; ++tick) {
                locomotion.update(profile, animations, input);
                const DefaultLocomotionOutput3D& out = locomotion.output();
                if (!out.valid) break;
                if (tick == 0) {
                    float joints = 0;
                    int count = 0;
                    for (std::size_t i = 1; i < physical.size(); ++i)
                        if (leg[i]) { joints += (out.coordinates[i] - physical[i]).length(); ++count; }
                    joints = joints / std::max(1, count) * 57.2958f;
                    float feetError = 0;
                    for (std::size_t side = 0; side < 2; ++side) {
                        Vec3 a = out.pose.linkPositions[feet[side]] - out.pose.linkPositions.front();
                        Vec3 b = state.links[feet[side]].position - pelvis.position;
                        a.z = b.z = 0.0f;
                        feetError = std::max(feetError, (a - b).length() * 100.0f);
                    }
                    const Vec3 outForward = (out.rootOrientation * profile.links.front().modelOrientation.conjugate())
                        .rotate({ 1.0f, 0.0f, 0.0f });
                    const Vec3 bodyForward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
                        .rotate({ 1.0f, 0.0f, 0.0f });
                    const float yaw = std::abs(std::remainder(std::atan2(outForward.y, outForward.x)
                        - std::atan2(bodyForward.y, bodyForward.x), 6.2831853f)) * 57.2958f;
                    jointSum += joints; feetSum += feetError; yawSum += yaw;
                    jointWorst = std::max(jointWorst, joints);
                    feetWorst = std::max(feetWorst, feetError);
                }
                // A mistura do laboratorio: do zero, a linear de 0,35 s a
                // partir da foto do corpo; com a semente, a inercia.
                std::vector<Vec3> blended(physical.size());
                if (seedOff) {
                    const float x = std::clamp((tick + 1) / 120.0f / 0.35f, 0.0f, 1.0f);
                    const float share = x * x * (3.0f - 2.0f * x);
                    for (std::size_t i = 1; i < physical.size(); ++i)
                        blended[i] = physical[i] + (out.coordinates[i] - physical[i]) * share;
                } else {
                    blended = out.coordinates;
                    Vec3 root = out.rootPosition;
                    Quaternion orientation = out.rootOrientation;
                    inertia.apply(blended, root, orientation, input.deltaTime);
                }
                float fastest = 0;
                for (std::size_t i = 1; i < physical.size(); ++i) {
                    if (!leg[i]) continue;
                    const Vec3 velocity = (blended[i] - previous[i]) * 120.0f;
                    fastest = std::max(fastest, velocity.length());
                    if (tick == 0) jerkWorst = std::max(jerkWorst, (velocity - physicalVelocity[i]).length());
                    if (tick == 0 && std::getenv("XVOLTAVEL"))
                        std::printf("  %s v fisico %.1f v saida %.1f\n", profile.links[i].id.c_str(),
                            physicalVelocity[i].length(), velocity.length());
                }
                previous = blended;
                speedWorst = std::max(speedWorst, fastest);
                if (std::getenv("XVOLTATICK")) {
                    float targetFastest = 0;
                    std::string which;
                    for (std::size_t i = 1; i < physical.size(); ++i)
                        if (leg[i] && (out.coordinates[i] - previousTarget[i]).length() * 120.0f > targetFastest) {
                            targetFastest = (out.coordinates[i] - previousTarget[i]).length() * 120.0f;
                            which = profile.links[i].id;
                        }
                    std::printf("  t %.2f tick %2d saida %.1f alvo %.1f (%s)\n", t, tick, fastest, targetFastest, which.c_str());
                }
                previousTarget = out.coordinates;
                if (tick == 47) speedSum += fastest;
                input.feetPositionWorld += velocity * input.deltaTime;
            }
        }
        const float n = static_cast<float>(std::max<std::size_t>(1, captured.size()));
        std::printf("VOLTA %-12s %s | %zu instantes | juntas das pernas med %.1f pior %.1f graus | pes x pelve med %.1f pior %.1f cm"
            " | rumo med %.1f graus | pico das juntas na mistura %.1f rad/s (no fim %.1f) | tranco na troca %.1f rad/s\n",
            c.name, seedOff ? "do zero " : "semente", captured.size(), jointSum / n, jointWorst, feetSum / n, feetWorst,
            yawSum / n, speedWorst, speedSum / n, jerkWorst);
    }
}

// Ida do padrao ao fisico (sweepida): o corpo andando cinematico pelo
// padrao fica dinamico em instantes ao longo de uma passada (XIDASEM: sem a
// semente). Mede o alvo dos motores contra o corpo na troca, o giro do alvo
// e do corpo nas pernas depois (e o do corpo antes) e a velocidade perdida.
void switchToPhysicalSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    struct Case { const char* name; Vec3 move; bool sprint; };
    const Case cases[] = { { "trote", { 1, 0, 0 }, false }, { "trote lado", { 0, 1, 0 }, false },
        { "trote costas", { -1, 0, 0 }, false }, { "sprint", { 1, 0, 0 }, true } };
    const int instants = std::getenv("XIDAPOS") ? std::atoi(std::getenv("XIDAPOS")) : 8;
    for (const Case& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"); only && std::string(only) != c.name) continue;
        float gap = 0, targetSpin = 0, bodySpin = 0, before = 0, speedLoss = 0;
        float gapWorst = 0, targetSpinWorst = 0, bodySpinWorst = 0, speedLossWorst = 0;
        int falls = 0;
        for (int k = 0; k < instants; ++k) {
            Scenario scenario { c.name, 3.6f, nullptr, [=](float) {
                ScenarioCommand command;
                command.move = c.move;
                command.sprint = c.sprint;
                return command;
            }, {}};
            scenario.defaultUntil = 2.0f + 0.5f * static_cast<float>(k) / static_cast<float>(instants);
            scenario.defaultAnimations = &animations;
            const ScenarioResult r = runScenario(scenario);
            gap += r.switchTargetGap; targetSpin += r.switchTargetSpin; bodySpin += r.switchBodySpin;
            before += r.switchBodySpinBefore;
            const float loss = r.switchSpeedBefore - r.switchSpeedMin;
            speedLoss += loss;
            gapWorst = std::max(gapWorst, r.switchTargetGap);
            targetSpinWorst = std::max(targetSpinWorst, r.switchTargetSpin);
            bodySpinWorst = std::max(bodySpinWorst, r.switchBodySpin);
            speedLossWorst = std::max(speedLossWorst, loss);
            falls += r.falls;
        }
        const float n = static_cast<float>(std::max(1, instants));
        std::printf("IDA %-12s %s | alvo x corpo na troca med %.1f pior %.1f graus | giro do alvo med %.1f pior %.1f rad/s"
            " | giro das pernas depois med %.1f pior %.1f (antes %.1f) rad/s | velocidade perdida med %.2f pior %.2f m/s | quedas %d\n",
            c.name, std::getenv("XIDASEM") ? "do zero" : "semente", gap / n, gapWorst, targetSpin / n, targetSpinWorst,
            bodySpin / n, bodySpinWorst, before / n, speedLoss / n, speedLossWorst, falls);
    }
}

// Carrinho no padrao que vira fisico no meio (sweepcarrinhoida): a pancada
// ao acertar alguem; o fisico continua o carrinho (XIDASEMCARRINHO: comeca
// do zero, como antes). Mede se ele termina de pe, quando, e o membro mais
// rapido.
void tackleSwitchSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    int standing = 0, total = 0;
    for (const bool sprint : { true, false }) {
        for (const float after : { 0.08f, 0.2f, 0.35f, 0.5f, 0.7f }) {
            const float tackleAt = 1.6f;
            Scenario scenario { "carrinho ida", 7.0f, nullptr, [=](float t) {
                ScenarioCommand c;
                c.move = { 1, 0, 0 };
                c.sprint = sprint;
                c.crouch = t >= tackleAt && t < tackleAt + 0.05f;
                return c;
            }, {}};
            scenario.defaultUntil = tackleAt + after;
            scenario.defaultAnimations = &animations;
            const ScenarioResult r = runScenario(scenario);
            ++total;
            standing += r.standingAtEnd ? 1 : 0;
            std::printf("CARRINHOIDA %-6s troca %.2f s depois | carrinho no padrao %s | de pe no fim %d | quedas %d | membro max %.1f m/s\n",
                sprint ? "sprint" : "trote", after, r.defaultTackleAt >= 0.0f ? "sim" : "nao", r.standingAtEnd ? 1 : 0,
                r.falls, r.worstLinkSpeed);
        }
    }
    std::printf("CARRINHOIDA total %d, de pe no fim %d (%s)\n", total, standing,
        std::getenv("XIDASEMCARRINHO") ? "do zero" : "continuando o carrinho");
}

// Impacto contra o mundo (sweepimpacto): correndo contra uma parede e contra
// uma mureta de 70 cm (de frente e pulando perto - as canelas batem), no modo
// padrao (vai ao fisico no choque) e no fisico puro, para comparar (pedido do
// usuario, 05/10 noite: no padrao o choque nao era o do fisico). Mede quando
// troca, se cai, quando, e se termina de pe.
void impactSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    if (!std::getenv("XGOLPE")) setenv("XGOLPE", "4", 1);
    struct Case { const char* name; float jumpX; float height; };
    const Case cases[] = { { "parede", -1.0f, 1.0f }, { "mureta", -1.0f, 0.35f },
        { "mureta pulando 6.6", 6.6f, 0.35f }, { "mureta pulando 7.0", 7.0f, 0.35f } };
    int falls[2] = { 0, 0 }, standing[2] = { 0, 0 }, total = 0;
    for (const Case& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"); only && std::string(only) != c.name) continue;
        for (const bool sprint : { true, false }) {
            if (std::getenv("XSOSPRINT") && (std::atoi(std::getenv("XSOSPRINT")) != 0) != sprint) continue;
            ++total;
            for (const int mode : { 0, 1 }) {
                if (std::getenv("XSOMODO") && std::atoi(std::getenv("XSOMODO")) != mode) continue;
                // (XMURETAALT: a meia-altura da mureta, m - a de 0,35 tem 70 cm.)
                const float height = std::string(c.name) == "mureta" && std::getenv("XMURETAALT")
                    ? std::strtof(std::getenv("XMURETAALT"), nullptr) : c.height;
                const float seconds = std::getenv("XIMPACTOSEG") ? std::strtof(std::getenv("XIMPACTOSEG"), nullptr) : 6.0f;
                const float release = std::getenv("XIMPACTOSOLTA") ? std::strtof(std::getenv("XIMPACTOSOLTA"), nullptr) : 1e9f;
                Scenario scenario { c.name, seconds, [=](PhysicsScene3D& s) {
                    staticBox(s, { 8.0f, 0.0f, height }, { 0.1f, 2.0f, height });
                }, [=](float t) {
                    ScenarioCommand command;
                    // (XIMPACTOSOLTA: o jogador solta o comando neste tempo.)
                    if (t > 0.5f && t < release) command.move = { 1, 0, 0 };
                    command.sprint = sprint;
                    return command;
                }, {}};
                scenario.jumpWhenX = sprint ? c.jumpX : c.jumpX + 0.6f;
                if (mode == 0) {
                    scenario.defaultUntil = 100.0f;
                    scenario.defaultAnimations = &animations;
                }
                const ScenarioResult r = runScenario(scenario);
                falls[mode] += r.falls > 0 ? 1 : 0;
                standing[mode] += r.standingAtEnd ? 1 : 0;
                std::printf("IMPACTO %-19s %-6s %-7s | troca %s (%.2f s%s) | cambaleio %.2f | quedas %d (primeira %.2f s) | de pe no fim %d"
                    " | inclinacao max %.0f graus\n", c.name, sprint ? "sprint" : "trote", mode == 0 ? "padrao" : "fisico",
                    r.hitSwitchAt >= 0.0f ? "sim" : "nao", r.hitSwitchAt, r.hitSwitchWall ? ", parede" : "", r.staggerPeak,
                    r.falls, r.firstFallSeconds, r.standingAtEnd ? 1 : 0, r.worstTilt);
            }
        }
    }
    std::printf("IMPACTO total %d casos: padrao caiu em %d (de pe no fim %d), fisico caiu em %d (de pe no fim %d)\n",
        total, falls[0], standing[0], falls[1], standing[1]);
}

// Corpo guiado x cinematico no modo padrao (sweepguiado): os mesmos
// movimentos so no padrao, com o corpo cinematico (o de antes) e o guiado
// (dinamico levado a pose). Mede o erro dos links contra a pose (antes e
// depois da correcao do fim do passo) e o maior pico de aceleracao de um link.
void guidedBodySweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    struct Case { const char* name; std::function<ScenarioCommand(float)> script; };
    const Case cases[] = {
        { "sprint reto", [](float t) { ScenarioCommand c; if (t > 0.3f) c.move = { 1, 0, 0 }; c.sprint = true; return c; } },
        { "sprint curva", [](float t) { ScenarioCommand c; const float a = t > 1.0f ? 0.7f * (t - 1.0f) : 0.0f;
            if (t > 0.3f) c.move = { std::cos(a), std::sin(a), 0 }; c.sprint = true; c.lookYaw = a; return c; } },
        { "trote zigue", [](float t) { ScenarioCommand c; const int k = static_cast<int>(t / 0.8f) % 4;
            const Vec3 d[4] = { { 1, 0, 0 }, { 0, 1, 0 }, { -1, 0, 0 }, { 0, -1, 0 } };
            if (t > 0.3f) c.move = d[k]; return c; } },
        { "pulo", [](float t) { ScenarioCommand c; if (t > 0.3f) c.move = { 1, 0, 0 }; c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f; return c; } },
        { "carrinho", [](float t) { ScenarioCommand c; if (t > 0.3f) c.move = { 1, 0, 0 }; c.sprint = true;
            c.crouch = t >= 2.0f && t < 2.05f; return c; } },
        { "voadora", [](float t) { ScenarioCommand c; if (t > 0.3f) c.move = { 1, 0, 0 }; c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f; c.crouch = t >= 2.25f && t < 2.3f; return c; } },
        { "parada e giro", [](float t) { ScenarioCommand c; if (t > 0.3f && t < 2.0f) c.move = { 1, 0, 0 };
            c.sprint = true; c.lookYaw = t > 2.5f ? 1.2f * (t - 2.5f) : 0.0f; return c; } },
    };
    for (const Case& item : cases) {
        for (const int guided : { 0, 1 }) {
            setenv("XGUIADO", guided ? "1" : "0", 1);
            Scenario scenario { item.name, 4.5f, nullptr, item.script, {} };
            scenario.defaultUntil = 100.0f;
            scenario.defaultAnimations = &animations;
            const ScenarioResult r = runScenario(scenario);
            std::printf("GUIADO %-14s %-10s | erro antes %.4f m, depois %.4f m | pico acel link %7.1f m/s2\n", item.name,
                guided ? "guiado" : "cinematico", r.guideErrorPeak, r.guidePoseErrorPeak, r.defaultLinkAccelPeak);
        }
    }
    unsetenv("XGUIADO");
}

// Trombada no padrao (sweeptrombada): o jogador no modo padrao corre para
// dentro de um boneco parado (ajudas do laboratorio), de frente e de raspao;
// a pancada (impulso > XGOLPE, padrao 4 como no laboratorio) o leva ao
// fisico. Com o corpo guiado (o do laboratorio) ou o cinematico (XGUIADO=0,
// massa infinita).
void bumpSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    if (!std::getenv("XGOLPE")) setenv("XGOLPE", "4", 1);
    int playerFalls = 0, npcFalls = 0, hits = 0, total = 0;
    float npcSpeed = 0;
    // XTROMBADACARRINHO: o carrinho do padrao nele (Ctrl a ~2,5 m).
    const bool tackle = std::getenv("XTROMBADACARRINHO") != nullptr;
    // XTROMBADAVOADORA (= instante do pulo, s; so no sprint): pula e da a
    // voadora (Ctrl com Shift + W) 0,1 s depois, para acerta-lo no ar ou no
    // comeco do deslize.
    const float voadoraJump = std::getenv("XTROMBADAVOADORA") ? std::strtof(std::getenv("XTROMBADAVOADORA"), nullptr) : -1.0f;
    for (const bool sprint : { true, false }) {
        if (voadoraJump > 0.0f && !sprint) continue;
        for (const float side : { 0.0f, 0.2f, 0.35f }) {
            Scenario scenario { "trombada", 6.0f, {}, [=](float t) {
                ScenarioCommand c;
                if (t > 0.5f) c.move = { 1, 0, 0 };
                c.sprint = sprint;
                const float at = sprint ? 1.45f : 1.75f;
                c.crouch = tackle && t >= at && t < at + 0.05f;
                if (voadoraJump > 0.0f) {
                    c.jump = t >= voadoraJump && t < voadoraJump + 0.01f;
                    c.crouch = t >= voadoraJump + 0.1f && t < voadoraJump + 0.25f;
                }
                return c;
            }, {}};
            scenario.npc = true;
            scenario.npcLabAssist = true;
            scenario.npcTackledPercent = 30.0f;
            scenario.npcPosition = { sprint ? 9.0f : 5.0f, side, 0.0f };
            scenario.npcYaw = 3.14159f;
            // (XVOADORAFISICO: o jogador no modo fisico o tempo todo.)
            scenario.defaultUntil = std::getenv("XVOADORAFISICO") ? 0.0f : 100.0f;
            scenario.defaultAnimations = &animations;
            const ScenarioResult r = runScenario(scenario);
            ++total;
            playerFalls += r.falls > 0 ? 1 : 0;
            npcFalls += r.npcFalls > 0 ? 1 : 0;
            hits += r.hitSwitchAt >= 0.0f ? 1 : 0;
            npcSpeed += r.npcPelvisSpeedPeak;
            if (voadoraJump > 0.0f)
                std::printf("TROMBADA voadora %.2f s (toque no chao a %.2f m/s) | ", r.voadoraAt, r.voadoraSpeedLanding);
            std::printf("TROMBADA %-6s lado %.2f | carrinho %s (%.2f s) | pancada %s (%.2f s) | jogador quedas %d de pe %d"
                " | boneco quedas %d, pelve max %.2f m/s, levou carrinho %s | ajuda min jogador %.0f%% boneco %.0f%%\n",
                sprint ? "sprint" : "trote", side, r.defaultTackleAt >= 0.0f ? "sim" : "nao", r.defaultTackleAt,
                r.hitSwitchAt >= 0.0f ? "sim" : "nao", r.hitSwitchAt, r.falls,
                r.standingAtEnd ? 1 : 0, r.npcFalls, r.npcPelvisSpeedPeak, r.npcTackledAt >= 0.0f ? "sim" : "nao",
                100.0f * r.playerAssistLow, 100.0f * r.npcAssistLow);
        }
    }
    std::printf("TROMBADA total %d (%s): pancada em %d, jogador caiu em %d, boneco caiu em %d, pelve do boneco media %.2f m/s\n",
        total, std::getenv("XGUIADO") && !std::atoi(std::getenv("XGUIADO")) ? "cinematico" : "guiado", hits, playerFalls,
        npcFalls, npcSpeed / static_cast<float>(std::max(1, total)));
}

// Voadora (sweepvoadoraida): no ar, Ctrl com Shift + W em qualquer ordem
// (pedido do usuario, 05/10 noite), ela vai ao fisico ja e segue com o embalo
// do pulo; so o Ctrl no ar nao faz nada. Mede a velocidade no pulo, na troca e
// no toque no chao, o deslize e se ele termina de pe.
// Velocidades por rumo (sweepvelocidades, 06/10): trote e sprint a 0, 45,
// 90, 135 e 180 graus do olhar, a velocidade media da pelve no rumo (de 3 a
// 5 s) - no modo fisico (padrao) e no padrao (XVELPADRAO). Para calibrar as
// velocidades do padrao pelas do fisico.
void gaitSpeedSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    const bool defaultMode = std::getenv("XVELPADRAO") != nullptr;
    for (const bool sprint : { false, true }) {
        for (const float degrees : { 0.0f, 45.0f, 90.0f, 135.0f, 180.0f }) {
            const float a = degrees * 3.14159265f / 180.0f;
            Scenario scenario { "velocidade", 5.2f, nullptr, [=](float t) {
                ScenarioCommand c;
                if (t > 0.5f) c.move = { std::cos(a), std::sin(a), 0.0f };
                c.sprint = sprint;
                c.lookYaw = 0.0f;
                return c;
            }, {}};
            scenario.speedFrom = 3.0f;
            scenario.speedTo = 5.0f;
            scenario.defaultUntil = defaultMode ? 100.0f : 0.0f;
            scenario.defaultAnimations = &animations;
            const ScenarioResult r = runScenario(scenario);
            std::printf("VELOCIDADE %s %s %3.0f graus | corpo %.2f m/s | quedas %d\n", defaultMode ? "padrao" : "fisico",
                sprint ? "sprint" : "trote ", degrees,
                r.windowSamples > 0 ? r.windowSpeedSum / r.windowSamples : 0.0, r.falls);
        }
    }
}

void airTackleSwitchSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    struct Case { const char* name; bool sprint; float crouchAt, crouchUntil, sprintFrom; };
    constexpr float JumpAt = 1.6f;
    const Case cases[] = {
        { "voadora 0.08", true, JumpAt + 0.08f, JumpAt + 0.2f, 0.0f },
        { "voadora 0.15", true, JumpAt + 0.15f, JumpAt + 0.3f, 0.0f },
        { "voadora 0.25", true, JumpAt + 0.25f, JumpAt + 0.4f, 0.0f },
        { "voadora 0.35", true, JumpAt + 0.35f, JumpAt + 0.5f, 0.0f },
        { "ctrl e shift", false, JumpAt + 0.10f, JumpAt + 0.5f, JumpAt + 0.2f },
        // So o Ctrl no ar: nada (o carrinho do pouso saiu).
        { "so ctrl", true, -1.0f, -1.0f, 0.0f },
    };
    int standing = 0, total = 0;
    for (const Case& c : cases) {
        if (const char* only = std::getenv("XSCENARIO"); only && std::string(only) != c.name) continue;
        const bool landing = c.crouchAt < 0.0f;
        Scenario scenario { c.name, 7.0f, nullptr, [=](float t) {
            ScenarioCommand command;
            command.move = { 1, 0, 0 };
            command.sprint = c.sprintFrom > 0.0f ? t >= c.sprintFrom : c.sprint;
            command.jump = t >= JumpAt && t < JumpAt + 0.01f;
            // (So o Ctrl segurado no ar, sem o Shift + W junto.)
            if (landing) {
                command.crouch = t >= JumpAt + 0.1f && t < JumpAt + 1.2f;
                if (command.crouch) command.sprint = false;
            } else {
                command.crouch = t >= c.crouchAt && t < c.crouchUntil;
            }
            return command;
        }, {}};
        // XVOADORAFISICO: o cenario inteiro no modo fisico (a voadora dele, 06/10).
        scenario.defaultUntil = std::getenv("XVOADORAFISICO") ? 0.0f : 100.0f;
        scenario.defaultAnimations = &animations;
        const ScenarioResult r = runScenario(scenario);
        ++total;
        standing += r.standingAtEnd ? 1 : 0;
        if (landing)
            std::printf("VOADORA %-13s | carrinho %s (%.2f s) | voadora %s | quedas %d | de pe no fim %d\n", c.name,
                r.defaultTackleAt >= 0.0f ? "sim" : "nao", r.defaultTackleAt, r.voadoraAt >= 0.0f ? "sim" : "nao",
                r.falls, r.standingAtEnd ? 1 : 0);
        else
            std::printf("VOADORA %-13s | voadora %s (%.2f s) | velocidade pulo %.2f troca %.2f toque %.2f m/s | deslize %.2f m"
                " | quedas %d | de pe no fim %d | membro max %.1f m/s\n", c.name, r.voadoraAt >= 0.0f ? "sim" : "nao",
                r.voadoraAt, r.voadoraSpeedTakeoff, r.voadoraSpeedSwitch, r.voadoraSpeedLanding, r.voadoraSlideMeters,
                r.falls, r.standingAtEnd ? 1 : 0, r.worstLinkSpeed);
    }
    std::printf("VOADORA total %d, de pe no fim %d\n", total, standing);
}

// Pulo no padrao que vira fisico na decolagem (sweeppuloida): o giro da
// pelve no voo (XYAWTRACE para o trace).
void jumpSwitchSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    const float switchDelay = std::getenv("XPULOATRASO") ? std::strtof(std::getenv("XPULOATRASO"), nullptr) : 0.05f;
    for (const bool sprint : { false, true }) {
        for (int k = 0; k < 6; ++k) {
            const float jumpAt = 2.0f + 0.1f * static_cast<float>(k);
            Scenario scenario { sprint ? "pulo sprint" : "pulo trote", jumpAt + 2.0f, nullptr, [=](float t) {
                ScenarioCommand command;
                command.move = { 1, 0, 0 };
                command.sprint = sprint;
                command.jump = t >= jumpAt && t < jumpAt + 0.01f;
                return command;
            }, {}};
            scenario.defaultUntil = jumpAt + switchDelay;
            scenario.defaultAnimations = &animations;
            const ScenarioResult r = runScenario(scenario);
            std::printf("PULOIDA %-11s pulo %.1f | giro da pelve no voo %.1f graus | quedas %d\n", scenario.name, jumpAt,
                r.airYawDrift, r.falls);
        }
    }
}

// Carrinho num boneco (sweepcarrinhonpc): o jogador corre e da o carrinho
// num boneco parado no caminho (ajudas do laboratorio), em varias distancias
// e desvios de lado; com e sem a ajuda baixa de quem leva carrinho
// (XNPCCARRINHO=% , padrao 30; -1 desliga).
void tackledNpcSweep() {
    const float low = std::getenv("XNPCCARRINHO") ? std::strtof(std::getenv("XNPCCARRINHO"), nullptr) : 30.0f;
    int falls = 0, hits = 0, total = 0;
    for (const bool sprint : { true, false }) {
        for (const float side : { 0.0f, 0.25f, -0.25f }) {
            for (const float at : { 1.6f, 1.8f, 2.0f }) {
                Scenario scenario { "carrinho npc", 6.0f, {}, [=](float t) {
                    ScenarioCommand c;
                    if (t > 0.5f) c.move = { 1, 0, 0 };
                    c.sprint = sprint;
                    c.crouch = t >= at && t < at + 0.05f;
                    return c;
                }, {}};
                scenario.npc = true;
                scenario.npcLabAssist = true;
                scenario.npcTackledPercent = low;
                scenario.npcPosition = { (sprint ? 7.0f : 3.4f) * (at - 0.5f) * 0.85f + 1.6f, side, 0.0f };
                scenario.npcYaw = 3.14159f;
                const ScenarioResult r = runScenario(scenario);
                ++total;
                falls += r.npcFalls > 0 ? 1 : 0;
                hits += r.npcTackledAt >= 0.0f ? 1 : 0;
                std::printf("CARRINHONPC %-6s lado %+.2f aos %.1f s | acertou %s (%.2f s) | boneco caiu %d | jogador quedas %d\n",
                    sprint ? "sprint" : "trote", side, at, r.npcTackledAt >= 0.0f ? "sim" : "nao", r.npcTackledAt,
                    r.npcFalls, r.falls);
            }
        }
    }
    std::printf("CARRINHONPC total: %d casos, carrinho detectado em %d, boneco caiu em %d (ajuda baixa %.0f%%)\n",
        total, hits, falls, low);
}

// Levantar com pressa entregue ao padrao (sweepentrega): o corpo no ponto
// de entrega de cada levantar (XLEVTECLAS; XLEVSHIFT correndo) e o padrao
// assumindo com a semente e a inercia (0,16 s / 0,65 s), andando para
// frente, lado e tras do corpo. Mede o pico de giro das juntas das pernas e
// a pelve; XENTREGAPOSES=prefixo grava as poses (previa).
void getUpHandoffSweep() {
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_TEST_ASSETS_DIR);
    const RagdollProfile3D& profile = assets.character.profile;
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    const bool shift = std::getenv("XLEVSHIFT") != nullptr;
    setenv("XLEVTECLAS", "1", 1);
    std::size_t feet[2] = { 0, 0 };
    std::vector<bool> leg(profile.links.size(), false);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id == "LeftFoot") feet[0] = i;
        if (id == "RightFoot") feet[1] = i;
        leg[i] = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
            || id.find("Foot") != std::string::npos;
    }
    struct Fall { const char* name; float pitch; float heading; };
    const Fall falls[] = { { "bruços", 60.0f, 0.0f }, { "costas", 60.0f, 180.0f }, { "lado", 60.0f, 90.0f } };
    for (const Fall& fall : falls) {
        Scenario scenario { "entrega", 7.0f, {}, [shift](float t) {
            ScenarioCommand c;
            if (t > 0.8f) c.move = { 1, 0, 0 };
            c.sprint = shift;
            return c;
        }, {}};
        scenario.spawnPitchDegrees = fall.pitch;
        scenario.spawnTiltHeadingDegrees = fall.heading;
        const ScenarioResult r = runScenario(scenario);
        if (r.handoffAt < 0.0f) { std::printf("ENTREGA %-7s nao chegou ao ponto de entrega\n", fall.name); continue; }
        const RagdollState3D& state = r.handoffState;
        const PhysicsBodyState3D& pelvis = state.links.front();
        const Vec3 forward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).rotate({ 1, 0, 0 });
        const float bodyYaw = std::atan2(forward.y, forward.x);
        for (const float relative : { 0.0f, 90.0f, 180.0f }) {
            const float travelYaw = bodyYaw + relative * 3.14159265f / 180.0f;
            const Vec3 direction { std::cos(travelYaw), std::sin(travelYaw), 0.0f };
            DefaultLocomotion3D locomotion;
            locomotion.reset(travelYaw);
            DefaultLocomotionSeed3D seed;
            seed.pelvisYawRadians = bodyYaw;
            seed.pelvisWorld = pelvis.position;
            seed.pelvisVelocityWorld = pelvis.linearVelocity;
            seed.feetWorld = { state.links[feet[0]].position, state.links[feet[1]].position };
            seed.feetVelocityWorld = { state.links[feet[0]].linearVelocity, state.links[feet[1]].linearVelocity };
            seed.feetValid = true;
            locomotion.seed(seed);
            if (!std::getenv("XENTREGASEMCLIPE")) locomotion.startCrouchSprint(travelYaw, shift);
            std::vector<Vec3> coordinates(profile.links.size()), velocities(profile.links.size());
            for (std::size_t i = 1; i < coordinates.size() && i < state.joints.size(); ++i) {
                coordinates[i] = { state.joints[i].positionRadians[0], state.joints[i].positionRadians[1], state.joints[i].positionRadians[2] };
                velocities[i] = { state.joints[i].velocityRadiansPerSecond[0], state.joints[i].velocityRadiansPerSecond[1],
                    state.joints[i].velocityRadiansPerSecond[2] };
            }
            PoseInertializer3D inertia;
            inertia.start(coordinates, velocities, pelvis.position, pelvis.orientation, pelvis.linearVelocity,
                pelvis.angularVelocity, 0.16f, 0.65f);
            Vec3 base { pelvis.position.x, pelvis.position.y, 0.0f };
            Vec3 velocity {};
            const float topSpeed = shift ? 8.0f : 3.4f;
            std::vector<Vec3> previous = coordinates;
            float spinPeak = 0.0f, lowest = 9.0f, endJerk = 0.0f, endPelvisAccel = 0.0f, startPelvisAccel = 0.0f;
            std::vector<Vec3> previousVelocity(profile.links.size());
            Vec3 previousRoot = pelvis.position, previousRootVelocity = pelvis.linearVelocity;
            int endTick = -1;
            bool wasScripted = false;
            FILE* file = nullptr;
            if (const char* prefix = std::getenv("XENTREGAPOSES"))
                file = std::fopen((std::string(prefix) + "_" + fall.name + "_" + std::to_string(static_cast<int>(relative)) + ".jsonl").c_str(), "w");
            float speedAtHalf = 0.0f;
            for (int tick = 0; tick < 120; ++tick) {
                const float dt = 1.0f / 120.0f;
                // A capsula do laboratorio: acelera ate a velocidade pedida.
                const Vec3 wanted = direction * topSpeed;
                const Vec3 change = wanted - velocity;
                const float limit = (shift ? 14.0f : 10.0f) * dt;
                velocity += change.length() > limit ? change * (limit / change.length()) : change;
                base += velocity * dt;
                DefaultLocomotionInput3D input;
                input.feetPositionWorld = base;
                input.velocityWorld = velocity;
                input.requestedVelocityWorld = wanted;
                input.facingYawRadians = travelYaw;
                input.deltaTime = dt;
                locomotion.update(profile, animations, input);
                const DefaultLocomotionOutput3D& out = locomotion.output();
                // A capsula embaixo da pelve do clipe (como no laboratorio);
                // no fim, com o embalo dele.
                if (out.rootScripted) { base.x = out.tackleRootWorld.x; base.y = out.tackleRootWorld.y; }
                if (out.tackleEnded) velocity = { out.tackleVelocity.x, out.tackleVelocity.y, 0.0f };
                std::vector<Vec3> blended = out.coordinates;
                Vec3 root = out.rootPosition;
                Quaternion orientation = out.rootOrientation;
                inertia.apply(blended, root, orientation, dt);
                RagdollAnimationPose3D pose;
                defaultLocomotionPose3D(profile, root, orientation, blended, pose);
                if (wasScripted && !out.rootScripted) endTick = tick;
                wasScripted = out.rootScripted;
                const Vec3 rootVelocity = (root - previousRoot) * (1.0f / dt);
                const float rootAccel = (rootVelocity - previousRootVelocity).length() / dt;
                if (tick > 0 && tick < 12) startPelvisAccel = std::max(startPelvisAccel, rootAccel);
                if (endTick >= 0 && tick - endTick < 12) endPelvisAccel = std::max(endPelvisAccel, rootAccel);
                if (std::getenv("XENTREGATRACE") && relative == 0.0f && (tick < 14 || (endTick >= 0 && tick - endTick < 10) || (out.rootScripted && tick % 6 == 0)))
                    std::printf("   tick %3d %s raiz v %.2f %.2f %.2f acel %.0f\n", tick, out.rootScripted ? "clipe" : "passo",
                        rootVelocity.x, rootVelocity.y, rootVelocity.z, rootAccel);
                previousRoot = root;
                previousRootVelocity = rootVelocity;
                for (std::size_t i = 1; i < blended.size(); ++i) {
                    const Vec3 v = (blended[i] - previous[i]) * (1.0f / dt);
                    if (leg[i]) spinPeak = std::max(spinPeak, v.length());
                    if (endTick >= 0 && tick - endTick < 6 && tick > 0)
                        endJerk = std::max(endJerk, (v - previousVelocity[i]).length() / dt);
                    previousVelocity[i] = v;
                }
                previous = blended;
                lowest = std::min(lowest, root.z);
                if (tick == 60) speedAtHalf = velocity.length();
                if (file && tick % 3 == 0) {
                    std::fprintf(file, "{\"t\": %.4f, \"z\": 0.0, \"p\": [", tick * dt);
                    for (std::size_t i = 0; i < pose.linkPositions.size(); ++i)
                        std::fprintf(file, "%s[%.5f, %.5f, %.5f]", i ? ", " : "", pose.linkPositions[i].x, pose.linkPositions[i].y,
                            pose.linkPositions[i].z);
                    std::fprintf(file, "], \"q\": [");
                    for (std::size_t i = 0; i < pose.linkOrientations.size(); ++i)
                        std::fprintf(file, "%s[%.6f, %.6f, %.6f, %.6f]", i ? ", " : "", pose.linkOrientations[i].x,
                            pose.linkOrientations[i].y, pose.linkOrientations[i].z, pose.linkOrientations[i].w);
                    std::fprintf(file, "]}\n");
                }
            }
            if (file) std::fclose(file);
            std::printf("ENTREGA %-7s rumo %+4.0f | entrega aos %.2f s (pelve %.2f m) | giro das pernas pico %.1f rad/s | pelve mais baixa %.2f m"
                " | acel da pelve no comeco %.0f no fim do clipe %.0f m/s2 (fim aos %.2f s) | tranco das juntas no fim %.0f rad/s2\n",
                fall.name, relative, r.handoffAt, pelvis.position.z, spinPeak, lowest, startPelvisAccel, endPelvisAccel,
                endTick / 120.0f, endJerk);
        }
    }
}

// Postura (sweeppostura): parado, trote e sprint em linha reta. Mede o joelho
// medido e o pedido (no apoio e no ciclo todo) e a altura da pelve - a
// sensacao de "sempre semi-agachado".
void postureSweep() {
    struct Case { const char* name; bool sprint; float rate; float degrees; };
    // rate 0: reto; degrees 0: curva continua (rate graus/s a partir de 3,5 s);
    // degrees > 0: virada brusca de degrees a rate graus/s aos 3 s.
    const std::vector<Case> cases = {
        { "parado", false, 0, 0 }, { "trote", false, 0, 0 }, { "sprint", true, 0, 0 },
        { "trote-c90", false, 90, 0 }, { "sprint-c90", true, 90, 0 }, { "trote-c135", false, -135, 0 },
        { "sprint-c45", true, -45, 0 }, { "sprint-c45e", true, 45, 0 }, { "sprint-c90d", true, -90, 0 },
        { "sprint-c30", true, -30, 0 }, { "sprint-c60", true, -60, 0 },
        { "sprint-90@360", true, 360, 90 }, { "trote-90@360", false, -360, 90 }, { "sprint-180@600", true, 600, 180 } };
    const char* only = std::getenv("XPOSCASO");
    for (const Case& item : cases) {
        if (only && std::string(only) != item.name) continue;
        const bool still = std::string(item.name) == "parado";
        const float omega = item.rate * 3.14159265f / 180.0f;
        const float sign = item.rate < 0.0f ? -1.0f : 1.0f;
        const float turnEnd = item.degrees > 0.0f ? 3.0f + item.degrees / std::abs(item.rate) : 0.0f;
        const float runSeconds = std::getenv("XPOSDUR") && item.degrees <= 0.0f
            ? std::strtof(std::getenv("XPOSDUR"), nullptr) : 7.0f;
        Scenario scenario { item.name, item.degrees > 0.0f ? turnEnd + 2.0f : runSeconds, {}, [=](float t) {
            ScenarioCommand c;
            float turn = 0.0f;
            if (item.degrees > 0.0f)
                turn = t < 3.0f ? 0.0f : t < turnEnd ? omega * (t - 3.0f) : sign * item.degrees * 3.14159265f / 180.0f;
            else if (item.rate != 0.0f && t > 3.5f)
                turn = omega * (t - 3.5f);
            c.lookYaw = turn;
            if (t >= 0.5f && !still) c.move = { std::cos(turn), std::sin(turn), 0.0f };
            c.sprint = item.sprint;
            return c;
        }, {} };
        scenario.postureFrom = item.degrees > 0.0f ? 3.0f : item.rate != 0.0f ? 4.1f : 3.0f;
        scenario.postureTo = item.degrees > 0.0f ? turnEnd + 1.0f : runSeconds;
        const ScenarioResult r = runScenario(scenario);
        const double all = std::max(1, r.postureSamples * 2);
        const double stance = std::max(1, r.postureStanceSamples);
        const double samples = std::max(1, r.postureSamples);
        std::printf("POSTURA %-14s quedas %d | joelho apoio %.1f (%.1f..%.1f) pedido %.1f toque %.1f saida %.1f | ciclo %.1f pedido %.1f | pelve %.3f (+-%.1f cm, toque %.3f) guia %.3f | desce curva %.1f freada %.1f alcance %+.1f pouso %.1f cm | rigidez %.2f motor %.0f/%.0f sat %.0f%% | apc sust %.1f em-pe %.2f env %.2f pernas %.2f | apoios do clipe %d: sem tocar %d, tocou atrasado %d (%.0f ms), apoiado %.0f%% do apoio | passos %d: soltos sem apoiar %d, apoio medio %.0f ms\n",
            item.name, r.falls, r.postureKneeStanceSum / stance, r.postureKneeStanceMin, r.postureKneeStanceMax,
            r.postureKneeStanceTargetSum / stance, r.postureTouchKneeSum / std::max(1, r.postureTouches),
            r.postureLiftKneeSum / std::max(1, r.postureLifts),
            r.postureKneeAllSum / all, r.postureKneeTargetAllSum / all,
            r.posturePelvisSum / samples,
            100.0 * std::sqrt(std::max(0.0, r.posturePelvisSquares / samples - (r.posturePelvisSum / samples) * (r.posturePelvisSum / samples))),
            r.postureTouchPelvisSum / std::max(1, r.postureTouches), r.postureGuideSum / samples,
            100.0 * r.postureDropCurve / samples, 100.0 * r.postureDropBrake / samples,
            100.0 * r.postureDropReach / samples, 100.0 * r.postureDropLanding / samples,
            r.postureStiffnessSum / stance, r.postureMotorSum / stance, r.postureLimitSum / stance,
            100.0 * r.postureSaturated / stance, r.postureSupport / samples, r.postureUpright / samples,
            r.postureEnvelope / samples, r.postureLegShare / samples,
            r.postureClipStances, r.postureMissedStances, r.postureLateTouches,
            1000.0 * r.postureTouchDelaySum / std::max(1, r.postureLateTouches - r.postureMissedStances),
            100.0 * r.postureTouchedShareSum / std::max(1, r.postureClipStances),
            r.posturePlants, r.posturePlantsMissed, 1000.0 * r.posturePlantSupportSum / std::max(1, r.posturePlants));
        std::printf("SUAVE %-14s braco alvo %.0f medido %.0f rad/s2 | perna alvo %.0f medida %.0f rad/s2 | ritmo do ciclo muda %.2f ciclos/s por tick\n",
            item.name, std::sqrt(r.postureArmTargetAcc / std::max(1, r.postureArmSamples)),
            std::sqrt(r.postureArmMeasuredAcc / std::max(1, r.postureArmSamples)),
            std::sqrt(r.postureLegTargetAcc / std::max(1, r.postureLegSamples)),
            std::sqrt(r.postureLegMeasuredAcc / std::max(1, r.postureLegSamples)),
            r.posturePhaseRateChange / std::max(1, r.posturePhaseSamples - 1));
        if (r.postureSteps > 0) {
            const double n = r.postureSteps;
            double apexMean = 0, apexWorst = 0, apexSpread = 0;
            for (float a : r.postureFlightApex) apexMean += a;
            if (!r.postureFlightApex.empty()) apexMean /= r.postureFlightApex.size();
            for (float a : r.postureFlightApex) { apexWorst = std::max(apexWorst, a - apexMean); apexSpread += (a - apexMean) * (a - apexMean); }
            if (!r.postureFlightApex.empty()) apexSpread = std::sqrt(apexSpread / r.postureFlightApex.size());
            float dvzWorst = 0;
            for (float d : r.postureStepDvzList) dvzWorst = std::max(dvzWorst, d);
            std::printf("PASSOS %-14s %d apoios de %.0f ms | por apoio: vertical %+.2f m/s (pior %+.2f), a frente %+.2f m/s | amortece %.0f ms (+%.1f graus), empurra %.0f ms | voos %zu: topo %.3f m, irregular +-%.1f cm (pior +%.1f)\n",
                item.name, r.postureSteps, 1000.0 * r.postureStepSeconds / n, r.postureStepDvz / n, dvzWorst, r.postureStepDvh / n,
                1000.0 * r.postureAbsorbSeconds / n, r.postureAbsorbDegrees / n, 1000.0 * r.posturePushSeconds / n,
                r.postureFlightApex.size(), apexMean, 100.0 * apexSpread, 100.0 * apexWorst);
        }
    }
}

// Levantar (sweeplevantar): deitado em 8 rumos (de bruços, de costas, de lado
// e entre eles) e quedas de verdade (tombado em 8 rumos, do alto em 4). Mede
// se levanta, em quanto tempo, a maior etapa do levantar pelo corpo e a
// ajuda usada. XLEVCASO=k roda so o caso k.
void getUpSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const int only = std::getenv("XLEVCASO") ? std::atoi(std::getenv("XLEVCASO")) : -1;
    const float seconds = std::getenv("XLEVSEG") ? std::strtof(std::getenv("XLEVSEG"), nullptr) : 16.0f;
    int index = -1;
    const auto run = [&](const char* kind, int heading, Scenario scenario) {
        ++index;
        if (only >= 0 ? index != only : index % parts != part) return;
        // XLEVTECLAS: segurando para a frente desde o chao (XLEVSHIFT: com
        // Shift) - o levantar com pressa.
        if (std::getenv("XLEVTECLAS")) {
            const bool shift = std::getenv("XLEVSHIFT") != nullptr;
            scenario.script = [shift](float t) {
                ScenarioCommand c;
                if (t > 0.8f) c.move = { 1, 0, 0 };
                c.sprint = shift;
                return c;
            };
        }
        const ScenarioResult r = runScenario(scenario);
        std::printf("LEVANTAR %2d %-8s %3d quedas %d | levantou %s em %.2f s (comecou %.2f) | em pe no fim %d recaidas %d | etapa max %d | ajuda pico %.2f | subida %.2f s inclinado %.2f s pelve max %.2f | no ar %.2f s, pelve sobe %.2f m/s, membro %.1f m/s\n",
            index, kind, heading, r.falls, r.getUpSeconds >= 0 ? "sim" : "nao", r.getUpSeconds,
            r.risingStartAt >= 0 && r.firstFallSeconds >= 0 ? r.risingStartAt - r.firstFallSeconds : -1.0f,
            r.standingAtEnd ? 1 : 0, r.relapses, r.bodilyMaxStage, r.getUpAssistPeakShare, r.risingSeconds, r.hunchedSeconds, r.risingPelvisPeak,
            r.getUpAirSeconds, r.risingPelvisRisePeak, r.risingLinkSpeedPeak);
    };
    for (int heading = 0; heading < 360; heading += 45) {
        Scenario scenario {"deitado", seconds, {}, {}, {}};
        scenario.spawnPitchDegrees = 88.0f;
        scenario.spawnTiltHeadingDegrees = static_cast<float>(heading);
        scenario.spawnPelvisHeight = 0.22f;
        run("deitado", heading, scenario);
    }
    for (int heading = 0; heading < 360; heading += 45) {
        Scenario scenario {"queda tombado", seconds, {}, {}, {}};
        scenario.spawnPitchDegrees = 60.0f;
        scenario.spawnTiltHeadingDegrees = static_cast<float>(heading);
        run("tombado", heading, scenario);
    }
    for (int heading = 0; heading < 360; heading += 90) {
        Scenario scenario {"queda do alto", seconds, {}, {}, {}};
        scenario.spawnPitchDegrees = 30.0f;
        scenario.spawnTiltHeadingDegrees = static_cast<float>(heading);
        scenario.spawnPelvisHeight = 3.0f;
        run("alto", heading, scenario);
    }
}

// Levantar de bruços (sweepbrucos): deitado de bruços, virado de -45 a +45
// graus de lado. Mede se levantou, em quanto tempo (da queda e da primeira
// subida) e em quantas tentativas. XBRUCOSCASO=k roda so o caso k.
void proneGetUpSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const int only = std::getenv("XBRUCOSCASO") ? std::atoi(std::getenv("XBRUCOSCASO")) : -1;
    const float seconds = std::getenv("XLEVSEG") ? std::strtof(std::getenv("XLEVSEG"), nullptr) : 14.0f;
    int index = -1;
    for (const int side : { -45, -30, -15, 0, 15, 30, 45 }) {
        ++index;
        if (only >= 0 ? index != only : index % parts != part) continue;
        Scenario scenario {"brucos", seconds, {}, {}, {}};
        scenario.spawnPitchDegrees = 88.0f;
        scenario.spawnTiltHeadingDegrees = static_cast<float>(side);
        scenario.spawnPelvisHeight = 0.22f;
        const ScenarioResult r = runScenario(scenario);
        std::printf("BRUCOS %2d %+3d | levantou %s em %.2f s (subida %.2f s) | tentativas %d | em pe no fim %d\n",
            index, side, r.getUpSeconds >= 0 ? "sim" : "nao", r.getUpSeconds,
            r.getUpSeconds >= 0 && r.risingFirstAt >= 0 && r.firstFallSeconds >= 0
                ? r.getUpSeconds + r.firstFallSeconds - r.risingFirstAt : -1.0f,
            r.getUpAttemptsMax, r.standingAtEnd ? 1 : 0);
    }
}

// Carrinho (sweepcarrinho): correndo para a frente (trote e sprint, em 5
// rumos), agacha aos 2,5 s e segue segurando a frente. Mede se deu o
// carrinho, a variacao (XCARRINHO=k fixa uma), o deslize, se terminou de pe
// (sem virar queda) e se seguiu correndo. XCARRINHOCASO=k roda so o caso k;
// XCARRINHOSOLTA solta a frente no carrinho.
void tackleSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const int only = std::getenv("XCARRINHOCASO") ? std::atoi(std::getenv("XCARRINHOCASO")) : -1;
    const bool release = std::getenv("XCARRINHOSOLTA") != nullptr;
    const float seconds = std::getenv("XLEVSEG") ? std::strtof(std::getenv("XLEVSEG"), nullptr) : 6.5f;
    // Quando o Ctrl e apertado (s): muda o ponto da passada (e o pe que pousa).
    const float pressAt = std::getenv("XCARRINHOAOS") ? std::strtof(std::getenv("XCARRINHOAOS"), nullptr) : 2.5f;
    int index = -1;
    for (const bool sprint : { false, true }) {
        for (const int heading : { 0, 70, 145, 215, 290 }) {
            ++index;
            if (only >= 0 ? index != only : index % parts != part) continue;
            const float yaw = static_cast<float>(heading) * 3.14159265f / 180.0f;
            const Vec3 dir { std::cos(yaw), std::sin(yaw), 0.0f };
            Scenario scenario {"carrinho", seconds, {}, [=](float t) {
                ScenarioCommand c;
                c.move = release && t >= pressAt && t < pressAt + 2.0f ? Vec3 {} : dir;
                c.sprint = sprint;
                c.lookYaw = yaw;
                c.crouch = t >= pressAt && t < pressAt + 0.2f;
                return c;
            }, {}};
            const ScenarioResult r = runScenario(scenario);
            std::printf("CARRINHO %2d %-6s %3d | deu %s aos %.2f s (preparo %.2f s) var %d entrada %.1f m/s | deslize %.2f m lado %.2f m pelve min %.2f | no chao %.2f s %.2f m (toca a %.1f m/s) | durou %.2f s | quedas %d | depois %.1f m/s | membro max %.1f m/s\n",
                index, sprint ? "sprint" : "trote", heading, r.tackleStartedAt >= 0 ? "sim" : "nao",
                r.tackleStartedAt, r.tackleStartedAt >= 0 && r.tackleArmedAt >= 0 ? r.tackleStartedAt - r.tackleArmedAt : -1.0f,
                r.tackleVariant, r.tackleEntrySpeed, r.tackleTravel, r.tackleLateral,
                r.tackleMinPelvis, r.slideGroundSeconds, r.slideGroundDistance, r.slideLandSpeed,
                r.tackleEndedAt >= 0 ? r.tackleEndedAt - r.tackleStartedAt : -1.0f,
                r.falls, r.afterTackleSpeed, r.tackleWorstLinkSpeed);
        }
    }
}

// Manejo de bola (sweepmanejo): a bola a frente, ele corre (trote ou
// sprint) para a frente ou de lado, e dribla (um toque a cada intervalo, na
// direcao do movimento) ou chuta (forca 0,5 e 1) quando a bola esta a
// frente. Mede toques por pedido, quantas vezes a bola fugiu, a distancia
// dela, quedas e o chute. XMANEJOCASO=k roda so o caso k; XMANEJOTRACE
// imprime o toque.
void ballHandlingSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const int only = std::getenv("XMANEJOCASO") ? std::atoi(std::getenv("XMANEJOCASO")) : -1;
    struct Case { const char* name; bool sprint; float moveDegrees; float interval; float power; float shotPower; };
    const std::vector<Case> cases = {
        {"frente-trote", false, 0, 0.95f, 0.25f, -1}, {"frente-trote-forte", false, 0, 1.2f, 0.6f, -1},
        {"frente-sprint", true, 0, 0.95f, 0.3f, -1}, {"esquerda-trote", false, 90, 0.95f, 0.25f, -1},
        {"direita-trote", false, -90, 0.95f, 0.25f, -1}, {"diag-esq", false, 45, 0.95f, 0.25f, -1},
        {"chute-trote-50", false, 0, 0.0f, 0.0f, 0.5f}, {"chute-trote-100", false, 0, 0.0f, 0.0f, 1.0f},
        {"chute-sprint-100", true, 0, 0.0f, 0.0f, 1.0f}};
    int index = -1;
    for (const Case& c : cases) {
        ++index;
        if (only >= 0 ? index != only : index % parts != part) continue;
        const float yaw = c.moveDegrees * 3.14159265f / 180.0f;
        const Vec3 move { std::cos(yaw), std::sin(yaw), 0.0f };
        auto handle = std::make_shared<InflatedBallHandle3D>();
        auto lastRequest = std::make_shared<float>(-10.0f);
        // Driblando, a bola comeca perto; no chute, a frente (ele chega
        // correndo e o pedido sai antes).
        const float ahead = c.shotPower >= 0.0f ? (c.sprint ? 4.0f : 2.4f) : 0.75f;
        Scenario scenario {"manejo", 6.0f, [=](PhysicsScene3D& scene) {
            InflatedBallDefinition3D ball;
            ball.position = { ahead * move.x, ahead * move.y, ball.profile.radiusMeters };
            ball.entityId = 778;
            *handle = scene.createInflatedBall(ball);
        }, [=](float t) {
            ScenarioCommand command;
            if (t >= 0.6f) command.move = move;
            command.sprint = c.sprint;
            command.lookYaw = 0.0f;
            return command;
        }, {}};
        scenario.ball = handle;
        // (XDONO: a bola como no laboratorio - o dono atravessa com as pernas.
        // Sem ela, a bola colide com o corpo inteiro o tempo todo.)
        scenario.ballCollides = std::getenv("XDONO") != nullptr;
        scenario.ballScript = [=](float t, Vec3 ball, Vec3 pelvis, bool idle, float& power, Vec3& direction) {
            if (c.shotPower >= 0.0f) {
                // (Rework de 06/10: o pedido so vale com a bola no alcance -
                // o jogador aperta quando chega nela.)
                const float near = dot(Vec3 { ball.x - pelvis.x, ball.y - pelvis.y, 0.0f }, move);
                if (t >= 1.0f && *lastRequest < 0.0f && near < 0.85f && near > 0.2f) {
                    *lastRequest = t;
                    power = c.shotPower;
                    direction = { 1.0f, 0.0f, 0.0f };
                    return BallActionKind3D::Shot;
                }
                return BallActionKind3D::None;
            }
            // O jogador toca quando a bola esta a frente, perto (entre 0,35 e
            // 1,1 m da pelve na direcao do movimento), com o toque anterior
            // terminado.
            const float ahead = dot(Vec3 { ball.x - pelvis.x, ball.y - pelvis.y, 0.0f }, move);
            if (t >= 0.6f && idle && t - *lastRequest >= 0.25f && ahead > 0.35f && ahead < 0.8f) {
                *lastRequest = t;
                power = c.power;
                direction = move;
                return BallActionKind3D::Dribble;
            }
            return BallActionKind3D::None;
        };
        const ScenarioResult r = runScenario(scenario);
        std::printf("MANEJO %d %-18s | pedidos %d toques %d (longe %d atrasado %d) acidentais %d sup %d/%d/%d/%d | bola max %.2f m media %.2f m | quedas %d (%.2f s) | chute v %.1f m/s rumo %+.1f elev %.1f\n",
            index, c.name, r.ballRequests, r.ballTouches, r.ballMissFar, r.ballMissLate, r.ballAccidental,
            r.ballSurfaces[0], r.ballSurfaces[1], r.ballSurfaces[2], r.ballSurfaces[3],
            r.ballMaxDistance, r.ballDistanceSamples ? r.ballDistanceSum / r.ballDistanceSamples : 0.0f,
            r.falls, r.firstFallSeconds, r.shotSpeed, r.shotYawError, r.shotElevation);
    }
}

// Manejo como impacto (sweepmanejoimpacto, 06/10): o jogador parado, a bola
// vindo nele a 4 m/s; um toque contra ela (o rumo de onde ela vem) na forca
// do caso. Mede a velocidade dela 0,15 a 0,45 s depois do toque (+ = devolvida,
// - = ainda vindo).
void ballImpactSweep() {
    for (const float power : { 0.0f, 0.1f, 0.4f, 1.0f }) {
        auto handle = std::make_shared<InflatedBallHandle3D>();
        auto touchAt = std::make_shared<float>(-1.0f);
        auto samples = std::make_shared<std::vector<std::pair<float, Vec3>>>();
        Scenario scenario {"manejo impacto", 3.0f, [=](PhysicsScene3D& scene) {
            InflatedBallDefinition3D ball;
            ball.position = { 5.0f, 0.0f, ball.profile.radiusMeters };
            ball.entityId = 780;
            *handle = scene.createInflatedBall(ball);
            // (Rolando: giro = (cima x v) / R.)
            scene.setInflatedBallMotion(*handle, ball.position, { -4.0f, 0.0f, 0.0f },
                { 0.0f, -4.0f / ball.profile.radiusMeters, 0.0f });
        }, [=](float) { ScenarioCommand command; command.lookYaw = 0.0f; return command; }, {}};
        scenario.ball = handle;
        // (XDONO: a bola como no laboratorio - o dono atravessa com as pernas.)
        scenario.ballCollides = std::getenv("XDONO") != nullptr;
        scenario.ballScript = [=](float t, Vec3 ball, Vec3 pelvis, bool idle, float& p, Vec3& direction) {
            samples->push_back({ t, ball });
            if (*touchAt < 0.0f && idle && ball.x - pelvis.x < 1.0f) {
                *touchAt = t;
                p = power;
                direction = { 1.0f, 0.0f, 0.0f };
                return BallActionKind3D::Dribble;
            }
            return BallActionKind3D::None;
        };
        const ScenarioResult r = runScenario(scenario);
        float after = 0.0f;
        if (*touchAt > 0.0f) {
            const auto at = [&](float when) {
                Vec3 best {};
                float gap = 1e9f;
                for (const auto& [t, ball] : *samples)
                    if (std::abs(t - when) < gap) { gap = std::abs(t - when); best = ball; }
                return best;
            };
            after = (at(*touchAt + 0.45f).x - at(*touchAt + 0.15f).x) / 0.3f;
        }
        std::printf("MANEJOIMPACTO forca %.1f | toques %d | toque %.2f s | bola depois %+.2f m/s (+ devolvida)\n",
            power, r.ballTouches, *touchAt, after);
    }
}

// Recuo correndo (sweepmanejorecuo, 06/10): ele trota para a frente com a
// bola; com ela a frente (0,3 a 1 m), um toque para tras na forca do caso,
// o corpo ainda indo para a frente. Mede a velocidade da bola para tras (+)
// 0,15 a 0,45 s depois.
void ballRetractionSweep() {
    for (const float power : { 0.1f, 0.25f, 0.5f }) {
        auto handle = std::make_shared<InflatedBallHandle3D>();
        auto lastRequest = std::make_shared<float>(-10.0f);
        auto retractAt = std::make_shared<float>(-1.0f);
        auto samples = std::make_shared<std::vector<std::pair<float, Vec3>>>();
        Scenario scenario {"manejo recuo", 3.5f, [=](PhysicsScene3D& scene) {
            InflatedBallDefinition3D ball;
            ball.position = { 0.75f, 0.0f, ball.profile.radiusMeters };
            ball.entityId = 781;
            *handle = scene.createInflatedBall(ball);
        }, [=](float t) {
            ScenarioCommand command;
            if (t >= 0.6f && *retractAt < 0.0f) command.move = { 1, 0, 0 };
            command.lookYaw = 0.0f;
            return command;
        }, {}};
        scenario.ball = handle;
        // (XDONO: a bola como no laboratorio - o dono atravessa com as pernas.)
        scenario.ballCollides = std::getenv("XDONO") != nullptr;
        scenario.ballScript = [=](float t, Vec3 ball, Vec3 pelvis, bool idle, float& p, Vec3& direction) {
            samples->push_back({ t, ball });
            const float ahead = ball.x - pelvis.x;
            if (*retractAt >= 0.0f || !idle || t - *lastRequest < 0.3f) return BallActionKind3D::None;
            // (O primeiro toque fraco: desde 06/10 noite o toque andando empurra
            // mais pela velocidade e o de 0,25 deixava a bola alem do alcance do
            // recuo.)
            if (t >= 0.6f && t < 1.2f && ahead > 0.35f && ahead < 0.8f) {
                *lastRequest = t;
                p = 0.1f;
                direction = { 1, 0, 0 };
                return BallActionKind3D::Dribble;
            }
            if (t >= 1.2f && ahead > 0.3f && ahead < 0.95f) {
                *retractAt = t;
                p = power;
                direction = { -1, 0, 0 };
                return BallActionKind3D::Dribble;
            }
            return BallActionKind3D::None;
        };
        const ScenarioResult r = runScenario(scenario);
        float before = 0.0f, after = 0.0f;
        if (*retractAt > 0.0f) {
            const auto at = [&](float when) {
                Vec3 best {};
                float gap = 1e9f;
                for (const auto& [t, ball] : *samples)
                    if (std::abs(t - when) < gap) { gap = std::abs(t - when); best = ball; }
                return best;
            };
            before = (at(*retractAt).x - at(*retractAt - 0.2f).x) / 0.2f;
            after = -(at(*retractAt + 0.45f).x - at(*retractAt + 0.15f).x) / 0.3f;
        }
        std::printf("MANEJORECUO forca %.2f | toques %d | recuo %.2f s | bola antes %+.2f m/s (frente) depois %+.2f m/s"
            " (+ para tras)\n", power, r.ballTouches, *retractAt, before, after);
    }
}

// Fluidez (sweepfluidez, 04/10): a movimentacao com a ajuda do laboratorio
// (padrao 100%) - o corpo contra a pose da animacao em cada manobra (o tronco
// e a pelve nao podem envergar so por andar), a pelve contra a raiz alvo,
// tropecos, quedas e o tempo de resposta ao comando. Sementes da variacao:
// XFLUSEMENTES (padrao 4). XFLUCASO=nome roda so um caso.
void fluiditySweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const char* only = std::getenv("XFLUCASO");
    const int seeds = std::getenv("XFLUSEMENTES") ? std::atoi(std::getenv("XFLUSEMENTES")) : 4;
    struct Case {
        const char* name;
        float seconds;
        std::function<ScenarioCommand(float)> script;
        float from, to;
        float responseAt;
        Vec3 responseDirection;
        bool responseSprint, responseStop;
    };
    const Vec3 ahead { 1.0f, 0.0f, 0.0f }, back { -1.0f, 0.0f, 0.0f }, left { 0.0f, 1.0f, 0.0f }, right { 0.0f, -1.0f, 0.0f };
    const auto run = [](Vec3 direction, bool sprint, float from, float yaw = 0.0f) {
        return [=](float t) {
            ScenarioCommand c;
            c.lookYaw = yaw;
            if (t >= from) c.move = direction;
            c.sprint = sprint;
            return c;
        };
    };
    std::vector<Case> cases;
    cases.push_back({ "parado", 4.0f, run({}, false, 99.0f), 1.0f, 4.0f, -1.0f, {}, false, false });
    cases.push_back({ "arranca-trote", 3.5f, run(ahead, false, 0.5f), 0.5f, 3.5f, 0.5f, ahead, false, false });
    cases.push_back({ "arranca-sprint", 4.0f, run(ahead, true, 0.5f), 0.5f, 4.0f, 0.5f, ahead, true, false });
    cases.push_back({ "trote", 5.0f, run(ahead, false, 0.5f), 2.0f, 5.0f, -1.0f, {}, false, false });
    cases.push_back({ "sprint", 5.0f, run(ahead, true, 0.5f), 2.5f, 5.0f, -1.0f, {}, false, false });
    cases.push_back({ "freia-trote", 5.0f, [](float t) { ScenarioCommand c; if (t >= 0.5f && t < 2.5f) c.move = { 1, 0, 0 }; return c; },
        2.5f, 5.0f, 2.5f, {}, false, true });
    cases.push_back({ "freia-sprint", 6.0f, [](float t) { ScenarioCommand c; if (t >= 0.5f && t < 3.0f) c.move = { 1, 0, 0 }; c.sprint = true; return c; },
        3.0f, 6.0f, 3.0f, {}, true, true });
    cases.push_back({ "volta-tras", 5.0f, [](float t) { ScenarioCommand c; if (t >= 0.5f) c.move = t < 2.5f ? Vec3 { 1, 0, 0 } : Vec3 { -1, 0, 0 }; return c; },
        2.5f, 5.0f, 2.5f, back, false, false });
    cases.push_back({ "giro-180", 5.0f, [](float t) {
        ScenarioCommand c;
        const float yaw = t < 2.5f ? 0.0f : std::min(3.14159265f, (t - 2.5f) * 10.47f);
        c.lookYaw = yaw;
        if (t >= 0.5f) c.move = { std::cos(yaw), std::sin(yaw), 0.0f };
        return c; }, 2.5f, 5.0f, 2.5f, back, false, false });
    cases.push_back({ "curva90-sprint", 5.5f, [](float t) {
        ScenarioCommand c;
        const float yaw = t < 3.0f ? 0.0f : std::min(1.5708f, (t - 3.0f) * 6.283f);
        c.lookYaw = yaw;
        if (t >= 0.5f) c.move = { std::cos(yaw), std::sin(yaw), 0.0f };
        c.sprint = true;
        return c; }, 3.0f, 5.5f, 3.0f, left, true, false });
    cases.push_back({ "lado-esq", 4.5f, run(left, false, 0.5f), 0.5f, 4.5f, 0.5f, left, false, false });
    cases.push_back({ "lado-dir", 4.5f, run(right, false, 0.5f), 0.5f, 4.5f, 0.5f, right, false, false });
    cases.push_back({ "zigue-zague", 6.0f, [](float t) {
        ScenarioCommand c;
        if (t >= 0.5f) {
            const float a = (static_cast<int>((t - 0.5f) / 0.6f) % 2 == 0 ? 1.0f : -1.0f) * 0.785f;
            c.move = { std::cos(a), std::sin(a), 0.0f };
        }
        return c; }, 1.5f, 6.0f, -1.0f, {}, false, false });
    cases.push_back({ "trote-sprint-trote", 7.0f, [](float t) {
        ScenarioCommand c; if (t >= 0.5f) c.move = { 1, 0, 0 }; c.sprint = t >= 2.5f && t < 4.5f; return c; },
        2.5f, 7.0f, 2.5f, ahead, true, false });
    cases.push_back({ "para-volta", 6.0f, [](float t) {
        ScenarioCommand c; if ((t >= 0.5f && t < 2.0f) || t >= 2.6f) c.move = { 1, 0, 0 }; return c; },
        2.0f, 6.0f, 2.6f, ahead, false, false });
    int index = -1;
    std::printf("FLUIDEZ caso: quedas | tronco inclina rms/max | pelve inclina rms/max | cabeca rms/max | pelve giro rms/max | todos os links rms/max (graus) | pelve x alvo plano rms/max, altura rms/max (cm) | tropeco ticks | resposta s\n");
    for (const Case& item : cases) {
        if (only && std::string(only) != item.name) continue;
        double trunk = 0, pelvis = 0, head = 0, yaw = 0, all = 0, posH = 0, posV = 0, posVSigned = 0, impact = 0, slip = 0;
        float impactMax = 0, slipWorst = 0;
        float trunkMax = 0, pelvisMax = 0, headMax = 0, yawMax = 0, allMax = 0, posHMax = 0, posVMax = 0, response = 0;
        int falls = 0, samples = 0, stumbles = 0, responses = 0, missing = 0;
        for (int seed = 0; seed < seeds; ++seed) {
            ++index;
            if (index % parts != part) continue;
            char seedText[16];
            std::snprintf(seedText, sizeof seedText, "%d", 101 + seed * 7);
            setenv("XVARIETYSEED", seedText, 1);
            Scenario scenario { item.name, item.seconds, {}, item.script, {} };
            scenario.fluidFrom = item.from;
            scenario.fluidTo = item.to;
            scenario.responseAt = item.responseAt;
            scenario.responseDirection = item.responseDirection;
            scenario.responseSprint = item.responseSprint;
            scenario.responseStop = item.responseStop;
            const ScenarioResult r = runScenario(scenario);
            falls += r.falls;
            trunk += r.fluTrunkSq; pelvis += r.fluPelvisSq; head += r.fluHeadSq; yaw += r.fluYawSq; all += r.fluAllSq;
            posH += r.fluPosHSq; posV += r.fluPosVSq; posVSigned += r.fluPosVSigned; impact += r.fluImpactSq;
            impactMax = std::max(impactMax, r.fluImpactMax); samples += r.fluSamples;
            slip += r.slipMeters; slipWorst = std::max(slipWorst, r.slipWorst); stumbles += r.fluStumbleTicks;
            trunkMax = std::max(trunkMax, r.fluTrunkMax); pelvisMax = std::max(pelvisMax, r.fluPelvisMax);
            headMax = std::max(headMax, r.fluHeadMax); yawMax = std::max(yawMax, r.fluYawMax); allMax = std::max(allMax, r.fluAllMax);
            posHMax = std::max(posHMax, r.fluPosHMax); posVMax = std::max(posVMax, r.fluPosVMax);
            if (item.responseAt >= 0.0f) {
                if (r.fluResponse >= 0.0f) { response += r.fluResponse; ++responses; } else ++missing;
            }
        }
        if (samples == 0) continue;
        const double n = samples;
        char responseText[48] = "-";
        if (item.responseAt >= 0.0f)
            std::snprintf(responseText, sizeof responseText, "%.2f%s", responses ? response / responses : -1.0f,
                missing ? " (nao chegou em alguns)" : "");
        std::printf("FLUIDEZ %-18s quedas %d | tronco %.1f/%.1f | pelve %.1f/%.1f | cabeca %.1f/%.1f | giro %.1f/%.1f | links %.1f/%.1f | plano %.1f/%.1f altura %.1f/%.1f (%+.1f) cm | impacto %.1f/%.0f m/s2 | deslize %.3f m (pior %.2f m/s) | tropeco %d | resposta %s\n",
            item.name, falls, std::sqrt(trunk / n), trunkMax, std::sqrt(pelvis / n), pelvisMax, std::sqrt(head / n), headMax,
            std::sqrt(yaw / n), yawMax, std::sqrt(all / n), allMax, std::sqrt(posH / n), posHMax, std::sqrt(posV / n), posVMax, posVSigned / n,
            std::sqrt(impact / n), impactMax, slip / std::max(1, seeds), slipWorst, stumbles, responseText);
    }
    unsetenv("XVARIETYSEED");
}

// Giro parado com o mouse (sweepgiro, 04/10): a camera vira pouco (20-30
// graus), de uma vez, ou continua (45-180 graus/s por 2 s), para cada lado.
// Mede a pior torcao das pernas (a canela contra o pe e contra a pelve, em
// graus no plano), passos, quedas e o atraso da pelve. XGIROCASO=nome.
void standingTurnSweep() {
    // (vaivem: +graus e, 0,4 s depois, -graus; zigue: alterna +-graus a cada
    // 0,3 s por 1,8 s; cont: graus/s por `seconds`.)
    struct Case { const char* name; float degrees; float rate; float seconds = 2.0f; int pattern = 0; };
    const std::vector<Case> cases = { { "snap20", 20, 0 }, { "snap30", 30, 0 }, { "snap45", 45, 0 }, { "snap90", 90, 0 },
        { "snap180", 180, 0 }, { "cont45", 0, 45 }, { "cont90", 0, 90 }, { "cont180", 0, 180 },
        { "cont360", 0, 360, 1.0f }, { "vaivem60", 60, 0, 0.0f, 1 }, { "zigue45", 45, 0, 0.0f, 2 } };
    const char* only = std::getenv("XGIROCASO");
    for (const Case& item : cases) {
        for (const float sign : { 1.0f, -1.0f }) {
            if (only && std::string(only) != item.name) continue;
            Scenario scenario {"giro mouse", 5.5f, {}, [=](float t) {
                ScenarioCommand c;
                const float degrees = item.degrees * 3.14159265f / 180.0f;
                if (item.rate > 0.0f) c.lookYaw = sign * std::clamp(t - 1.5f, 0.0f, item.seconds) * item.rate * 3.14159265f / 180.0f;
                else if (item.pattern == 1) c.lookYaw = t < 1.5f ? 0.0f : t < 1.9f ? sign * degrees : -sign * degrees;
                else if (item.pattern == 2) c.lookYaw = t < 1.5f || t > 3.3f ? 0.0f
                    : ((static_cast<int>((t - 1.5f) / 0.3f) % 2) == 0 ? sign * degrees : -sign * degrees);
                else c.lookYaw = t > 1.5f ? sign * degrees : 0.0f;
                return c;
            }, {}};
            scenario.legTwistFrom = 1.5f;
            const ScenarioResult r = runScenario(scenario);
            int steps = 0;
            for (const auto& step : r.plannerSteps)
                if (step.releasedAt > 1.5f) ++steps;
            const float settle = item.pattern == 2 ? 3.3f : item.pattern == 1 ? 1.9f
                : item.rate > 0.0f ? 1.5f + item.seconds : 1.5f;
            std::printf("GIRO %-8s %+.0f quedas %d | passos %d | cruzado %d ticks (pior %.0f cm) | afastados pior %.2f m | canela x pelve pior %.0f | pelve atras do olhar pior %.0f"
                " | colados %d ticks (min %.0f cm) | esfrega %.2f m | inclina %.1f | assenta %.2f s | embolado %d ticks (pior %+.0f cm)\n",
                item.name, sign, r.falls, steps, r.crossTicks, r.crossWorst, r.ankleSpreadWorst, r.legTwistPelvis, r.pelvisLookLag,
                r.closeTicks, r.feetMinDistance * 100.0f, r.footScrub, r.tiltWorst, std::max(0.0f, r.lastUnsettled - settle),
                r.baseTangleTicks, r.baseSeparationWorst * 100.0f);
        }
    }
}

// Chute correndo (sweepchutecorre, pedido do usuario 04/10: "ele precisa
// acertar sempre, inclusive com a bola mais distante"): o jogador correndo
// de varios jeitos (reto, sprint, zigue, corte, de lado, curva, parando,
// virando do recuo); em 2 s a bola aparece num ponto em relacao ao corpo
// (0,8/1,6/2,6 m; 0, +-40, +-80 graus) e o chute e pedido na hora, pelo pe.
// Acerto: tocou em ate 1,5 s. XCORRECASO=padrao roda so ele.
void runningShotSweep() {
    const char* only = std::getenv("XCORRECASO");
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    constexpr float Pi = 3.14159265f;
    constexpr float Request = 2.0f;
    const std::vector<std::string> patterns = { "reto", "sprint", "zigue", "corte", "lado", "curva", "para", "vira",
        "sprintzigue" };
    // Variacoes: forca (XCORREFORCA), bola rolando no pedido (XCORREBOLAV
    // "vx,vy" no referencial do corpo) e distancias (XCORREDIST "a,b,c").
    const float shotPower = std::getenv("XCORREFORCA") ? std::strtof(std::getenv("XCORREFORCA"), nullptr) : 0.8f;
    Vec3 ballVelocityBody {};
    if (const char* v = std::getenv("XCORREBOLAV")) std::sscanf(v, "%f,%f", &ballVelocityBody.x, &ballVelocityBody.y);
    std::vector<float> distances { 0.8f, 1.6f, 2.6f };
    if (const char* v = std::getenv("XCORREDIST")) {
        distances.clear();
        std::string text = v;
        std::size_t at = 0;
        while (at < text.size()) {
            const std::size_t comma = text.find(',', at);
            distances.push_back(std::strtof(text.substr(at, comma - at).c_str(), nullptr));
            if (comma == std::string::npos) break;
            at = comma + 1;
        }
    }
    int index = -1, hits = 0, runs = 0, falls = 0;
    for (const std::string& pattern : patterns) {
        if (only && pattern != only) continue;
        for (const float distance : distances) {
            for (const float degrees : { 0.0f, 40.0f, -40.0f, 80.0f, -80.0f }) {
                ++index;
                if (index % parts != part) continue;
                auto handle = std::make_shared<InflatedBallHandle3D>();
                auto requested = std::make_shared<bool>(false);
                const float window = std::getenv("XCORREJANELA") ? std::strtof(std::getenv("XCORREJANELA"), nullptr) : 2.5f;
                Scenario scenario {"chutecorre", Request + window + 0.1f, [=](PhysicsScene3D& scene) {
                    InflatedBallDefinition3D ball;
                    ball.position = { -30.0f, -30.0f, ball.profile.radiusMeters };
                    ball.entityId = 779;
                    ball.collidesWithRagdolls = false;
                    *handle = scene.createInflatedBall(ball);
                }, [=](float t) {
                    ScenarioCommand c;
                    if (t < 0.6f) return c;
                    float yaw = 0.0f, look = 0.0f;
                    bool lookFollows = true, move = true;
                    if (pattern == "sprint") c.sprint = true;
                    else if (pattern == "sprintzigue") {
                        c.sprint = true;
                        yaw = t < 1.5f ? 0.0f : (static_cast<int>((t - 1.5f) / 0.5f) % 2 == 0 ? 0.6f : -0.6f);
                    } else if (pattern == "zigue") yaw = t < 1.5f ? 0.0f : (static_cast<int>((t - 1.5f) / 0.5f) % 2 == 0 ? 0.8f : -0.8f);
                    else if (pattern == "corte") yaw = t < Request ? 0.0f : -Pi / 2.0f;
                    else if (pattern == "lado") { yaw = Pi / 2.0f; lookFollows = false; }
                    else if (pattern == "curva") yaw = 0.9f * (t - 0.6f);
                    else if (pattern == "para") move = t < Request;
                    else if (pattern == "vira") { yaw = t < Request ? Pi : 0.0f; lookFollows = false; }
                    if (move) c.move = { std::cos(yaw), std::sin(yaw), 0.0f };
                    c.lookYaw = lookFollows ? yaw : look;
                    return c;
                }, {}};
                scenario.ball = handle;
                scenario.footTouch = true;
                scenario.ballPlaceAt = Request;
                scenario.ballPlaceOffset = { distance * std::cos(degrees * Pi / 180.0f),
                    distance * std::sin(degrees * Pi / 180.0f), 0.0f };
                scenario.ballPlaceVelocity = ballVelocityBody;
                scenario.ballScript = [=](float t, Vec3, Vec3, bool, float& power, Vec3& direction) {
                    if (*requested || t < Request) return BallActionKind3D::None;
                    *requested = true;
                    power = shotPower;
                    direction = {};
                    return BallActionKind3D::Shot;
                };
                const ScenarioResult r = runScenario(scenario);
                const bool hit = r.shotDelay >= 0.0f && r.shotDelay <= window;
                hits += hit ? 1 : 0;
                ++runs;
                falls += r.falls;
                std::printf("CHUTECORRE %-6s d %.1f a %+4.0f | %s em %.2f s pe %d | quedas %d\n", pattern.c_str(), distance,
                    degrees, hit ? "TOCOU" : "falhou", r.shotDelay, r.shotKickSide, r.falls);
                std::fflush(stdout);
            }
        }
    }
    std::printf("CHUTECORRE total: %d/%d, quedas %d\n", hits, runs, falls);
}

// Estresse do manejo (sweepbolaestresse, 04/10 - o crash ao chutar no
// laboratorio): movimento aleatorio (8 rumos, paradas, sprint), a bola
// reposta perto do corpo a cada ~1 s e pedidos de toque e chute a cada
// 0,1-0,5 s, um substituindo o outro (parado e correndo). Passa se nao cai
// nem quebra. XESTRESSESEMENTE escolhe a sequencia.
void ballStressSweep() {
    const int seed = std::getenv("XESTRESSESEMENTE") ? std::atoi(std::getenv("XESTRESSESEMENTE")) : 1;
    constexpr float Pi = 3.14159265f;
    struct Event { float at; int kind; float power; Vec3 place; bool movePlace; };
    struct Segment { float until; bool move; float yaw; bool sprint; };
    auto events = std::make_shared<std::vector<Event>>();
    auto segments = std::make_shared<std::vector<Segment>>();
    std::uint32_t state = 2166136261u ^ static_cast<std::uint32_t>(seed * 16777619u);
    const auto next = [&]() {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        return static_cast<float>(state % 10000u) / 10000.0f;
    };
    for (float at = 0.8f; at < 12.0f;) {
        Segment seg;
        at += 0.3f + 0.9f * next();
        seg.until = at;
        seg.move = next() > 0.2f;
        seg.yaw = std::floor(next() * 8.0f) * Pi / 4.0f;
        seg.sprint = next() < 0.3f;
        segments->push_back(seg);
    }
    for (float at = 1.0f; at < 12.0f;) {
        Event e;
        at += 0.1f + 0.4f * next();
        e.at = at;
        e.kind = next() < 0.45f ? 2 : 1;  // 2 chute, 1 toque
        e.power = next();
        e.movePlace = next() < 0.35f;
        const float a = next() * 2.0f * Pi, d = 0.4f + 1.6f * next();
        e.place = { d * std::cos(a), d * std::sin(a), 0.0f };
        events->push_back(e);
    }
    auto handle = std::make_shared<InflatedBallHandle3D>();
    auto cursor = std::make_shared<std::size_t>(0);
    auto requests = std::make_shared<int>(0);
    const bool collides = std::getenv("XESTRESSECOLIDE") != nullptr;
    Scenario scenario {"estresse", 12.5f, [=](PhysicsScene3D& scene) {
        InflatedBallDefinition3D ball;
        ball.position = { 0.6f, 0.0f, ball.profile.radiusMeters };
        ball.entityId = 779;
        ball.collidesWithRagdolls = collides;
        *handle = scene.createInflatedBall(ball);
    }, [=](float t) {
        ScenarioCommand c;
        if (t < 0.8f) return c;
        const Segment* seg = &segments->back();
        for (const Segment& item : *segments) if (t < item.until) { seg = &item; break; }
        if (seg->move) c.move = { std::cos(seg->yaw), std::sin(seg->yaw), 0.0f };
        c.lookYaw = seg->yaw;
        c.sprint = seg->sprint;
        return c;
    }, {}};
    scenario.ball = handle;
    scenario.footTouch = true;
    scenario.ballScript = [=](float t, Vec3, Vec3, bool, float& power, Vec3& direction) {
        if (*cursor >= events->size() || t < (*events)[*cursor].at) return BallActionKind3D::None;
        const Event e = (*events)[(*cursor)++];
        ++*requests;
        power = e.power;
        direction = { std::cos(e.place.x), std::sin(e.place.x), 0.0f };
        return e.kind == 2 ? BallActionKind3D::Shot : BallActionKind3D::Dribble;
    };
    // A bola reposta perto do corpo a cada ~1 s (o chute a manda longe).
    scenario.ballReplaceEvery = 1.0f;
    scenario.ballReplaceSeed = seed;
    scenario.ballCollides = collides;
    const ScenarioResult r = runScenario(scenario);
    std::printf("ESTRESSE semente %d | pedidos %d toques %d | quedas %d (%.2f s)\n", seed, *requests, r.ballTouches, r.falls,
        r.firstFallSeconds);
}

// Repulsao macia (sweeprepulsa, 04/10): o corpo liberado (como depois do
// primeiro contato) parado; a bola chega rolando em varias alturas de corpo
// (canela, coxa, quadril - quicando) e de lado. Mede a menor distancia da bola
// ao link mais perto (nao deve entrar fundo nem ficar presa), a velocidade
// dela 0,5 s depois (passou ou desviou, sem quique duro) e se o corpo caiu.
void ballRepelSweep() {
    constexpr float Pi = 3.14159265f;
    for (const float speed : { 2.0f, 5.0f, 9.0f }) {
        for (const float height : { 0.11f, 0.45f, 0.90f, 1.25f, 1.45f }) {
            for (const float side : { 0.0f, 0.15f }) {
                auto handle = std::make_shared<InflatedBallHandle3D>();
                auto worst = std::make_shared<float>(9.0f);
                auto after = std::make_shared<float>(-1.0f);
                // (Onde a bola foi parar no rumo dela, contra a pelve, 0,5 s
                // depois: negativo = atravessou o corpo.)
                auto beyond = std::make_shared<float>(0.0f);
                Scenario scenario {"repulsa", 3.0f, [=](PhysicsScene3D& scene) {
                    InflatedBallDefinition3D ball;
                    ball.position = { -30.0f, -30.0f, 0.11f };
                    ball.entityId = 779;
                    ball.collidesWithRagdolls = true;
                    *handle = scene.createInflatedBall(ball);
                }, [](float) { return ScenarioCommand {}; }, {}};
                scenario.ball = handle;
                scenario.footTouch = true;
                scenario.ballCollides = true;
                scenario.ballPlaceAt = 1.5f;
                scenario.ballPlaceOffset = { 1.2f, side, 0.0f };
                scenario.ballPlaceVelocity = { -speed, 0.0f, 0.0f };
                scenario.ballPlaceHeight = height;
                scenario.forcePassThrough = true;
                scenario.ballScript = [](float, Vec3, Vec3, bool, float&, Vec3&) { return BallActionKind3D::None; };
                scenario.ballProbe = [=](float t, Vec3 ball, Vec3 velocity, const RagdollState3D& state) {
                    if (t < 1.5f) return;
                    for (const auto& link : state.links) *worst = std::min(*worst, (link.position - ball).length());
                    if (*after < 0.0f && t >= 2.0f) {
                        *after = velocity.length();
                        *beyond = ball.x - state.links.front().position.x;
                    }
                };
                const ScenarioResult r = runScenario(scenario);
                std::printf("REPULSA v %.0f altura %.2f lado %.2f | menor distancia a um link %.2f m | bola 0,5 s depois %.1f m/s,"
                    " %+.2f m da pelve (%s) | quedas %d\n", speed, height, side, *worst, *after, *beyond,
                    *beyond < -0.2f ? "atravessou" : "ficou na frente", r.falls);
                (void)Pi;
            }
        }
    }
}

// Boletim da locomocao fisica (sweepboletim; v2 do equilibrio, pedido do
// usuario 05/10 noite): a bateria do modo fisico - corrida (reta, curva,
// cortes, zigue-zague, para-arranca, chicote, frente-tras, strafe, meia-volta
// e giro no sprint), pulos (parado, correndo e seguindo, virando, seguidos,
// corre-pula-corre), empurroes parado e correndo e uma sequencia mista - num
// nivel de ajuda (XBOLETIMNIVEL, 0..1, parado e andando; padrao 0,7). Uma
// linha por cenario: quedas, saltinhos (voos sem pulo em que o centro de
// massa sobe 2 cm alem do voo natural da passada), tempo
// pendurado (> 0,25 s sem pe apoiado), a pelve acima da de pe, inclinacoes e
// a ajuda usada. XBOLETIMCASO filtra por nome. tools/analysis/
// locomotion_report.py roda os niveis em paralelo e monta a tabela.
void locomotionReportSweep() {
    const float level = std::getenv("XBOLETIMNIVEL") ? std::strtof(std::getenv("XBOLETIMNIVEL"), nullptr) : 0.7f;
    char text[32];
    std::snprintf(text, sizeof(text), "%.3f", level);
    setenv("XE5ANIM", "1", 1);
    setenv("XRETAIN", text, 1);
    setenv("XWALKASSIST", text, 1);
    // O pulo do laboratorio (o toque a 0,72 da velocidade; carregado, 1,16).
    setenv("XPULOLAB", "1", 1);
    const auto run = [](float from, float until, Vec3 move, bool sprint) {
        return [=](float t) { ScenarioCommand c; if (t > from && t < until) c.move = move; c.sprint = sprint; return c; };
    };
    const auto zigzag = [](float period, bool sprint) {
        return [=](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) c.move = { 1, static_cast<int>((t - 0.6f) / period) % 2 ? 0.8f : -0.8f, 0 };
            c.sprint = sprint;
            return c;
        };
    };
    // Empurrao (bloco de 60 kg, como pushScenarios) parado ou correndo: o
    // bloco sai ao lado do peito com a velocidade do corpo mais a do empurrao.
    const auto shove = [](float at, float speed, float headingDegrees) {
        return [=](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
            if (std::abs(t - at) >= 0.5f / 120) return;
            const float a = headingDegrees * 3.14159265f / 180;
            const Vec3 from { std::cos(a), std::sin(a), 0 };
            const Vec3 chest = state.links.front().position;
            const Vec3 body { state.links.front().linearVelocity.x, state.links.front().linearVelocity.y, 0.0f };
            dynamicBox(scene, { chest.x + from.x * 0.55f, chest.y + from.y * 0.55f, 1.25f },
                { 0.12f, 0.25f, 0.15f }, 60, body + from * -speed);
        };
    };
    std::vector<Scenario> scenarios = {
        { "corrida trote reto", 5.5f, {}, run(0.5f, 4.5f, { 1, 0, 0 }, false), {} },
        { "corrida sprint reto e para", 6.0f, {}, run(0.5f, 4.5f, { 1, 0, 0 }, true), {} },
        { "corrida sprint curva", 6.0f, {}, [](float t) {
            ScenarioCommand c;
            const float yaw = t > 1.2f ? 0.9f * (t - 1.2f) : 0.0f;
            if (t > 0.5f && t < 5.5f) c.move = { std::cos(yaw), std::sin(yaw), 0 };
            c.lookYaw = yaw;
            c.sprint = true;
            return c; }, {} },
        { "corrida zigue sprint 0,4 s", 6.5f, {}, zigzag(0.4f, true), {} },
        { "corrida zigue trote 0,3 s", 6.5f, {}, zigzag(0.3f, false), {} },
        { "corrida corte 90 sprint", 6.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = t < 2.2f ? Vec3 { 1, 0, 0 } : Vec3 { 0, 1, 0 };
            c.lookYaw = t < 2.2f ? 0.0f : 1.5708f;
            c.sprint = true;
            return c; }, {} },
        { "corrida para e arranca", 7.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 6.0f && std::fmod(t, 1.2f) < 0.8f) c.move = { 1, 0, 0 };
            c.sprint = true;
            return c; }, {} },
        { "corrida chicote de camera", 6.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) c.move = { 1, 0, 0 };
            c.lookYaw = t > 1.5f ? 1.2f * std::sin((t - 1.5f) * 6.0f) : 0.0f;
            c.sprint = true;
            return c; }, {} },
        { "corrida frente-tras trote", 7.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 6.0f) c.move = { static_cast<int>((t - 0.6f) / 0.7f) % 2 ? -1.0f : 1.0f, 0, 0 };
            return c; }, {} },
        { "corrida strafe alternado", 6.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) c.move = { 0, static_cast<int>((t - 0.6f) / 0.5f) % 2 ? -1.0f : 1.0f, 0 };
            return c; }, {} },
        { "corrida meia-volta sprint", 6.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.5f && t < 5.0f) c.move = t < 2.5f ? Vec3 { 1, 0, 0 } : Vec3 { -1, 0, 0 };
            c.lookYaw = t < 2.5f ? 0.0f : 3.14159f;
            c.sprint = true;
            return c; }, {} },
        { "corrida giro 360 no sprint", 6.0f, {}, [](float t) {
            ScenarioCommand c;
            const float yaw = t > 1.8f && t < 3.3f ? 4.19f * (t - 1.8f) : t >= 3.3f ? 6.2832f : 0.0f;
            if (t > 0.5f && t < 5.0f) c.move = { std::cos(yaw), std::sin(yaw), 0 };
            c.lookYaw = yaw;
            c.sprint = true;
            return c; }, {} },
        { "pulo parado", 4.0f, {}, [](float t) { ScenarioCommand c; c.jump = t >= 1.0f && t < 1.01f; return c; }, {} },
        { "pulo sprint e segue", 6.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = { 1, 0, 0 };
            c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f;
            return c; }, {} },
        { "pulo sprint e vira", 6.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.0f) c.move = t < 2.2f ? Vec3 { 1, 0, 0 } : Vec3 { 0, 1, 0 };
            c.lookYaw = t < 2.2f ? 0.0f : 1.5708f;
            c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f;
            return c; }, {} },
        { "pulo carregado sprint e segue", 6.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 5.5f) c.move = { 1, 0, 0 };
            c.sprint = true;
            c.jump = t >= 2.0f && t < 2.01f;
            c.jumpChargeSeconds = 0.6f;
            return c; }, {} },
        { "pulos seguidos trote", 7.0f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.6f && t < 6.0f) c.move = { 1, 0, 0 };
            c.jump = (t >= 1.8f && t < 1.81f) || (t >= 3.3f && t < 3.31f) || (t >= 4.8f && t < 4.81f);
            return c; }, {} },
        { "pulo corre-pula-corre sprint", 7.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.5f && t < 6.5f) c.move = { 1, 0, 0 };
            c.sprint = true;
            c.jump = (t >= 1.8f && t < 1.81f) || (t >= 3.4f && t < 3.41f) || (t >= 5.0f && t < 5.01f);
            return c; }, {} },
        { "empurrao parado 120 frente", 4.5f, {}, {}, shove(1.5f, 2.0f, 0) },
        { "empurrao parado 210 frente", 4.5f, {}, {}, shove(1.5f, 3.5f, 0) },
        // (Pode cair: um bloco de 60 kg a 5 m/s no peito derruba uma pessoa
        // de verdade - o boletim o mostra a parte, fora do "zero quedas".)
        { "empurrao parado 300 frente (pode cair)", 4.5f, {}, {}, shove(1.5f, 5.0f, 0) },
        { "empurrao parado 210 lado", 4.5f, {}, {}, shove(1.5f, 3.5f, 90) },
        { "empurrao parado 210 tras", 4.5f, {}, {}, shove(1.5f, 3.5f, 180) },
        { "empurrao trote 120 lado", 5.0f, {}, run(0.5f, 4.5f, { 1, 0, 0 }, false), shove(2.2f, 2.0f, 90) },
        { "empurrao sprint 210 lado", 5.0f, {}, run(0.5f, 4.5f, { 1, 0, 0 }, true), shove(2.2f, 3.5f, -90) },
        { "empurrao sprint 120 tras", 5.0f, {}, run(0.5f, 4.5f, { 1, 0, 0 }, true), shove(2.2f, 2.0f, 180) },
        { "mista sequencia", 7.5f, {}, [](float t) {
            ScenarioCommand c;
            if (t > 0.5f && t < 1.6f) { c.move = { 1, 0, 0 }; c.sprint = true; }
            else if (t < 2.3f) { c.move = { 0.7f, 0.7f, 0 }; c.sprint = true; }
            else if (t < 2.9f) c.move = { -1, 0, 0 };
            else if (t < 3.5f) c.move = { 0, -1, 0 };
            else if (t < 4.8f) { c.move = { 1, 0, 0 }; c.sprint = true; }
            else if (t < 5.5f) { c.move = { 0.7f, -0.7f, 0 }; c.sprint = true; }
            else if (t < 6.2f) c.move = { 1, 0, 0 };
            c.jump = t >= 4.1f && t < 4.11f;
            return c; }, {} },
    };
    // Tropeco numa barra baixa que os pes nao veem (dinamica, 300 kg; como
    // sweeptropeco): achatar a corrida faz o pe passar mais baixo e enganchar
    // mais - o boletim tem de ver isso. As variantes trocam a fase do pe na
    // barra (cada uma chega nela 43 ms depois).
    for (const bool sprint : { false, true }) {
        const float x = sprint ? 13.0f : 6.0f, h = sprint ? 0.26f : 0.20f;
        Scenario trip { sprint ? "tropeco sprint barra 26" : "tropeco trote barra 20", 5.5f,
            [=](PhysicsScene3D& scene) { dynamicBox(scene, { x, 0.0f, h * 0.5f }, { 0.05f, 1.5f, h * 0.5f }, 300.0f, {}); },
            [=](float t) { ScenarioCommand c; if (t >= 0.5f) c.move = { 1.0f, 0.0f, 0.0f }; c.sprint = sprint; return c; }, {} };
        trip.unseenObstacles = true;
        trip.stumbleX = x;
        scenarios.push_back(trip);
    }
    // Escada (8 degraus de 18 x 28 cm, subindo) e meio-fio de 25 cm, como
    // sweepstairs e sweepterrain.
    for (const bool sprint : { false, true }) {
        scenarios.push_back({ sprint ? "escada sprint 18 cm" : "escada trote 18 cm", 7.0f, [](PhysicsScene3D& s) {
            for (int i = 0; i < 8; ++i) {
                const float h = (i + 1) * 0.18f;
                staticBox(s, { 2.4f + (i + 0.5f) * 0.28f, 0, h * 0.5f }, { 0.14f, 2, h * 0.5f });
            }
            staticBox(s, { 2.4f + 8.0f * 0.28f + 10.0f, 0, 0.72f }, { 10, 2, 0.72f });
        }, run(0.5f, 6.0f, { 1, 0, 0 }, sprint), {} });
    }
    scenarios.push_back({ "meio-fio sprint 25 cm", 5.0f, [](PhysicsScene3D& s) {
        staticBox(s, { 5.0f, 0, 0.125f }, { 0.15f, 2, 0.125f });
    }, run(0.5f, 4.5f, { 1, 0, 0 }, true), {} });
    int totalFalls = 0, fallenScenarios = 0, hops = 0, count = 0;
    float suspended = 0.0f;
    double assist = 0.0;
    // Variantes (XBOLETIMVARIANTES, padrao 3): os comandos e o empurrao
    // atrasados k x 43 ms - caem em outras fases da passada (a fisica e
    // caotica: uma execucao so nao mede nada).
    const int variants = std::max(1, std::getenv("XBOLETIMVARIANTES") ? std::atoi(std::getenv("XBOLETIMVARIANTES")) : 3);
    // Uma parte das rodadas (XBOLETIMPARTE = "k/N"): o script roda N
    // processos em paralelo, cada um com as rodadas de indice k mod N.
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XBOLETIMPARTE")) std::sscanf(p, "%d/%d", &part, &parts);
    parts = std::max(1, parts);
    int runIndex = -1;
    for (const auto& base : scenarios) {
        // (XBOLETIMCASO: um trecho do nome, ou varios separados por "|".)
        if (const char* only = std::getenv("XBOLETIMCASO")) {
            bool match = false;
            std::string filter = only;
            for (std::size_t from = 0; from <= filter.size();) {
                const std::size_t bar = std::min(filter.find('|', from), filter.size());
                if (bar > from && std::string(base.name).find(filter.substr(from, bar - from)) != std::string::npos) match = true;
                from = bar + 1;
            }
            if (!match) continue;
        }
      for (int variant = 0; variant < variants; ++variant) {
        if (++runIndex % parts != part) continue;
        Scenario scenario = base;
        const float offset = 0.043f * static_cast<float>(variant);
        if (base.script) scenario.script = [script = base.script, offset](float t) { return script(t - offset); };
        if (base.disturb)
            scenario.disturb = [disturb = base.disturb, offset](PhysicsScene3D& scene, float t, const RagdollState3D& state) {
                disturb(scene, t - offset, state);
            };
        std::string label = base.name;
        if (variants > 1) label += " #" + std::to_string(variant);
        const ScenarioResult r = runScenario(scenario);
        const float meanAssist = r.bolAssistSamples > 0 ? static_cast<float>(r.bolAssistSum / r.bolAssistSamples) : 0.0f;
        const float meanPlanar = r.bolAssistSamples > 0 ? static_cast<float>(r.bolAssistPlanarSum / r.bolAssistSamples) : 0.0f;
        std::printf("BOLETIM %.2f | %-40s | quedas %d (%5.2f s) | saltinhos %d voo max %.2f s | pendurado %.2f s"
            " | pelve acima %4.1f cm | incl %3.0f peito %3.0f | ajuda %3.0f%% plano %3.0f%% | vel %3.0f%%\n", level, label.c_str(),
            r.falls, r.firstFallSeconds, r.bolHops, r.bolFlightPeak, r.bolSuspendedSeconds,
            std::max(0.0f, r.bolPelvisRisePeak) * 100.0f, r.worstTilt, r.worstChestTilt, meanAssist * 100.0f,
            meanPlanar * 100.0f, r.bolRequestedSum > 0.0 ? r.bolSpeedSum / r.bolRequestedSum * 100.0 : 100.0);
        std::fflush(stdout);
        ++count;
        totalFalls += r.falls;
        fallenScenarios += r.falls > 0 ? 1 : 0;
        hops += r.bolHops;
        suspended += r.bolSuspendedSeconds;
        assist += meanAssist;
      }
    }
    std::printf("BOLETIM %.2f total | %d cenarios | caiu em %d (%d quedas) | saltinhos %d | pendurado %.2f s | ajuda media %.0f%%\n",
        level, count, fallenScenarios, totalFalls, hops, suspended, count > 0 ? assist / count * 100.0 : 0.0);
}

// Recuperacao apoiada (sweepapoio; pedido do usuario, 06/10): o corpo
// comeca debrucado sobre uma mureta (o tronco apoiado no topo, os pes no
// chao atras), como depois de bater correndo nela e amolecer. Com chance de
// se recompor, ele deve usar pernas, pes e coluna (sem forca de fora) para
// voltar a ficar em pe, em vez de escorregar e cair. Mede se ficou em pe, se
// caiu e a inclinacao. XAPOIOCASO filtra. Nivel de ajuda pelo ambiente
// (XRETAIN/XWALKASSIST, como no boletim).
void braceRecoverySweep() {
    struct Case { const char* name; float wallTop; float pitch; float pelvis; float wallX; float heading; };
    const Case cases[] = {
        { "mureta 85 inclinado 45", 0.85f, 45.0f, 0.66f, 0.30f, 0.0f },
        { "mureta 85 inclinado 55", 0.85f, 55.0f, 0.58f, 0.36f, 0.0f },
        { "mureta 95 inclinado 40", 0.95f, 40.0f, 0.72f, 0.28f, 0.0f },
        { "mureta 95 inclinado 50", 0.95f, 50.0f, 0.64f, 0.33f, 0.0f },
        { "mureta 75 inclinado 55", 0.75f, 55.0f, 0.58f, 0.36f, 0.0f },
        { "parede de lado 25", 2.0f, 25.0f, 0.86f, 0.42f, 90.0f },
    };
    int up = 0, fell = 0, total = 0;
    for (const Case& c : cases) {
        if (const char* only = std::getenv("XAPOIOCASO"); only && std::string(c.name).find(only) == std::string::npos) continue;
        for (const float yaw : { 0.0f, 20.0f, -20.0f }) {
            const float a = (c.heading + yaw) * 3.14159265f / 180.0f;
            const Vec3 out { std::cos(a), std::sin(a), 0.0f };
            const float top = c.wallTop, wx = c.wallX;
            Scenario scenario { c.name, 6.0f, [=](PhysicsScene3D& s) {
                // Mureta a frente (no rumo da inclinacao), larga.
                staticBox(s, { out.x * (wx + 0.15f), out.y * (wx + 0.15f), top * 0.5f },
                    { std::abs(out.x) > 0.5f ? 0.15f : 2.0f, std::abs(out.x) > 0.5f ? 2.0f : 0.15f, top * 0.5f });
            }, {}, {} };
            scenario.spawnPitchDegrees = c.pitch;
            scenario.spawnTiltHeadingDegrees = c.heading + yaw;
            scenario.spawnPelvisHeight = c.pelvis;
            const ScenarioResult r = runScenario(scenario);
            ++total;
            up += r.standingAtEnd ? 1 : 0;
            fell += r.falls > 0 ? 1 : 0;
            std::printf("APOIO %-26s giro %+4.0f | quedas %d (%.2f s) | de pe no fim %d | inclinacao max %.0f\n",
                c.name, yaw, r.falls, r.firstFallSeconds, r.standingAtEnd ? 1 : 0, r.worstTilt);
            std::fflush(stdout);
        }
    }
    std::printf("APOIO total %d | de pe %d | caiu %d\n", total, up, fell);
}

// Dono da bola (sweepdono; BallOwnership3D): o jogador com a bola, ela rola
// ate um boneco parado (o dono vai ao boneco); chutada para longe (a 5 m por
// 3 s solta); parada longe sem ninguem tocar (sem dono).
void ballOwnerSweep() {
    struct Case { const char* name; bool npc; Vec3 offset; float speed; bool claim; };
    const Case cases[] = {
        { "troca com o boneco", true, { 0.6f, 0.0f, 0.0f }, 4.0f, true },
        { "chutada para longe", false, { 0.6f, 0.0f, 0.0f }, 14.0f, true },
        { "parada sem toque", false, { 3.0f, 0.0f, 0.0f }, 0.0f, false },
    };
    for (const Case& c : cases) {
        auto handle = std::make_shared<InflatedBallHandle3D>();
        Scenario scenario { c.name, 5.5f, [=](PhysicsScene3D& scene) {
            InflatedBallDefinition3D ball;
            ball.position = { -30.0f, -30.0f, 0.11f };
            ball.entityId = 781;
            ball.collidesWithRagdolls = true;
            *handle = scene.createInflatedBall(ball);
        }, [](float) { return ScenarioCommand {}; }, {}};
        scenario.ball = handle;
        scenario.footTouch = true;
        scenario.ballCollides = true;
        scenario.ballPlaceAt = 1.0f;
        scenario.ballPlaceOffset = c.offset;
        scenario.ballPlaceVelocity = { c.speed, 0.0f, 0.0f };
        scenario.ballClaimAtPlace = c.claim;
        scenario.ballScript = [](float, Vec3, Vec3, bool, float&, Vec3&) { return BallActionKind3D::None; };
        if (c.npc) {
            scenario.npc = true;
            scenario.npcLabAssist = true;
            scenario.npcPosition = { 2.6f, 0.0f, 0.0f };
            scenario.npcYaw = 3.14159f;
        }
        const ScenarioResult r = runScenario(scenario);
        std::string log;
        for (const auto& [t, owner] : r.ballOwnerLog) {
            char text[64];
            std::snprintf(text, sizeof(text), " %.2f s: %s", t, owner == 9101 ? "jogador" : owner == 9102 ? "boneco" : "ninguem");
            log += text;
        }
        std::printf("DONO %-20s | trocas:%s | quedas %d\n", c.name, log.empty() ? " nenhuma" : log.c_str(), r.falls);
    }
}

// Chute (sweepchute): chegando na bola parada a frente (trote/sprint), o
// pedido sai cedo; a bola em distancias que caem em fases diferentes da
// passada e de lado. Mede o toque, onde o pe de apoio estava, do pedido ao
// toque, quedas, a bola (velocidade, rumo, elevacao, curva) e o gesto.
// XCHUTECASO=k roda so o caso k; XCHUTETRACE imprime o chute tick a tick.
void shotSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const int only = std::getenv("XCHUTECASO") ? std::atoi(std::getenv("XCHUTECASO")) : -1;
    struct Gesture { const char* name; float lift; float curve; };
    const Gesture gestures[] = { { "normal", 0.0f, 0.0f }, { "alta", 1.0f, 0.0f }, { "rasteira", -1.0f, 0.0f },
        { "curva-dir", 0.0f, 1.0f }, { "curva-esq", 0.0f, -1.0f } };
    int index = -1, touched = 0, runs = 0, falls = 0;
    for (const bool sprint : { false, true }) {
        for (const float power : { 0.5f, 1.0f }) {
            for (int phase = 0; phase < 3; ++phase) {
                for (const float lateral : { -0.12f, 0.0f, 0.12f }) {
                    for (int g = 0; g < 5; ++g) {
                        // O gesto so no caso do meio (sem desvio lateral, fase 0).
                        if (g > 0 && (lateral != 0.0f || phase != 0)) continue;
                        ++index;
                        if (only >= 0 ? index != only : index % parts != part) continue;
                        const Gesture gesture = gestures[g];
                        const float step = sprint ? 1.74f : 1.10f;
                        const float x = (sprint ? 4.0f : 2.4f) + step * phase / 3.0f;
                        auto handle = std::make_shared<InflatedBallHandle3D>();
                        auto requested = std::make_shared<bool>(false);
                        Scenario scenario {"chute", 4.5f, [=](PhysicsScene3D& scene) {
                            InflatedBallDefinition3D ball;
                            ball.position = { x, lateral, ball.profile.radiusMeters };
                            ball.entityId = 779;
                            *handle = scene.createInflatedBall(ball);
                        }, [=](float t) {
                            ScenarioCommand command;
                            if (t >= 0.6f) command.move = { 1.0f, 0.0f, 0.0f };
                            command.sprint = sprint;
                            command.lookYaw = 0.0f;
                            return command;
                        }, {}};
                        scenario.ball = handle;
                        scenario.shotLift = gesture.lift;
                        scenario.shotCurve = gesture.curve;
                        scenario.ballScript = [=](float t, Vec3, Vec3, bool, float& p, Vec3& direction) {
                            if (t >= 1.0f && !*requested) {
                                *requested = true;
                                p = power;
                                direction = { 1.0f, 0.0f, 0.0f };
                                return BallActionKind3D::Shot;
                            }
                            return BallActionKind3D::None;
                        };
                        const ScenarioResult r = runScenario(scenario);
                        ++runs;
                        falls += r.falls;
                        if (r.ballTouches > 0) ++touched;
                        std::printf("CHUTE %2d %-6s %3.0f%% fase %d y %+.2f %-9s | toque %d (longe %d tarde %d acid %d) pe %d em %.2f s | apoio frente %+.2f lado %+.2f | quedas %d | bola %.1f m/s rumo %+.1f elev %.1f giro %.0f alto %.2f m curva20 %+.2f m\n",
                            index, sprint ? "sprint" : "trote", power * 100.0f, phase, lateral, gesture.name,
                            r.ballTouches, r.ballMissFar, r.ballMissLate, r.ballAccidental, r.shotKickSide, r.shotDelay,
                            r.shotPlantAlong, r.shotPlantAcross, r.falls, r.shotSpeed, r.shotYawError, r.shotElevation,
                            r.shotSpin, r.shotApex, r.shotLateral20);
                    }
                }
            }
        }
    }
    std::printf("CHUTE total: %d/%d com toque, %d quedas\n", touched, runs, falls);
}

// Bola no caminho (sweepbola): correndo em linha reta (trote ou sprint) para
// uma bola de futebol parada, com desvios laterais e fases da passada. Mede
// quedas, a freada do corpo ao bater na bola (como no tropeco) e o que a bola
// fez: maior velocidade (o chute pelo pe) e onde parou. XBOLAMARCHA filtra.
void ballSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const char* onlyGait = std::getenv("XBOLAMARCHA");
    struct BallProbe {
        InflatedBallHandle3D ball;
        float maximumSpeed = 0.0f;
        float firstTouch = -1.0f;
        int touches = 0;
        bool touching = false;
        Vec3 position;
    };
    int index = 0, totalFalls = 0, runs = 0, kicked = 0;
    for (const bool sprint : { false, true }) {
        if (onlyGait && std::string(onlyGait) != (sprint ? "sprint" : "trote")) continue;
        for (const float lateral : { -0.24f, -0.12f, 0.0f, 0.12f, 0.24f }) {
            for (int phase = 0; phase < 3; ++phase) {
                if (index++ % parts != part) continue;
                const float step = sprint ? 1.74f : 1.10f;
                const float x = (sprint ? 11.0f : 5.0f) + step * phase / 3.0f;
                auto probe = std::make_shared<BallProbe>();
                Scenario scenario {"bola", sprint ? 5.5f : 7.0f, [=](PhysicsScene3D& scene) {
                    InflatedBallDefinition3D ball;
                    ball.position = { x, lateral, ball.profile.radiusMeters };
                    ball.entityId = 777;
                    probe->ball = scene.createInflatedBall(ball);
                }, [=](float t) {
                    ScenarioCommand c;
                    if (t >= 0.5f) c.move = { 1.0f, 0.0f, 0.0f };
                    c.sprint = sprint;
                    return c;
                }, [=](PhysicsScene3D& scene, float t, const RagdollState3D&) {
                    const InflatedBallState3D state = scene.inflatedBallState(probe->ball);
                    probe->position = state.position;
                    probe->maximumSpeed = std::max(probe->maximumSpeed, state.linearVelocity.length());
                    bool touching = false;
                    for (std::uint32_t k = 0; k < state.contactCount; ++k) touching = touching || state.contacts[k].otherDynamic;
                    if (touching && !probe->touching) {
                        ++probe->touches;
                        if (probe->firstTouch < 0.0f) probe->firstTouch = t;
                    }
                    probe->touching = touching;
                }};
                scenario.stumbleX = x;
                const ScenarioResult r = runScenario(scenario);
                ++runs;
                totalFalls += r.falls;
                if (probe->maximumSpeed > 1.0f) ++kicked;
                std::printf("BOLA %-6s y %+.2f fase %d: quedas %d | v antes %.2f min %.2f depois %.2f | bola vmax %.2f toques %d (1o %.2f s) parou x %+.2f y %+.2f\n",
                    sprint ? "sprint" : "trote", lateral, phase, r.falls, r.stSpeedBefore,
                    r.stSpeedMin < 1e8f ? r.stSpeedMin : -1.0f, r.stSpeedAfter, probe->maximumSpeed, probe->touches,
                    probe->firstTouch, probe->position.x - x, probe->position.y);
            }
        }
    }
    std::printf("BOLA total: %d quedas em %d corridas, bola chutada em %d\n", totalFalls, runs, kicked);
}

// Arrancada (sweeparrancada): parado e, num instante que varre a fase do
// idle (8), sai correndo (trote ou sprint) para um rumo em relacao ao olhar
// - com a camera parada (frente, diagonais, lados, costas) ou virando junto
// para o rumo (o mouse e a tecla ao mesmo tempo). Mede quedas, raspadas do
// pe no chao no meio do balanco (tropeco), a maior inclinacao e o tempo ate
// 80% da velocidade. XARRCASO filtra um caso.
void startSweep() {
    // kind 0: parado; 1: depois de trotar e parar (a parada termina ~1,2 s
    // antes); 2: no meio de um giro parado (a camera vira 100 graus 0,1-0,7 s
    // antes e o comando sai para a frente dela).
    struct Case { const char* name; float degrees; bool turnCamera; int kind; };
    const std::vector<Case> cases = {
        {"frente", 0, false, 0}, {"diag+45", 45, false, 0}, {"diag-45", -45, false, 0},
        {"lado+90", 90, false, 0}, {"lado-90", -90, false, 0}, {"costas", 180, false, 0},
        {"vira+90", 90, true, 0}, {"vira-90", -90, true, 0}, {"vira180", 180, true, 0},
        {"posparada", 0, false, 1}, {"posparada90", 90, true, 1}, {"nogiro", 100, true, 2}, {"nogiro-", -100, true, 2}};
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    const char* only = std::getenv("XARRCASO");
    int index = 0;
    for (const bool sprint : { true, false }) {
        for (const Case& item : cases) {
            if (only && std::string(only) != item.name) continue;
            for (int phase = 0; phase < 8; ++phase) {
                if (index++ % parts != part) continue;
                const int kind = item.kind;
                const float start = kind == 1 ? 3.4f + 0.8f * phase / 8.0f
                    : kind == 2 ? 2.1f + 0.6f * phase / 8.0f
                    : 1.6f + 0.6f * phase / 8.0f;
                const float a = item.degrees * 3.14159265f / 180.0f;
                const bool turnCamera = item.turnCamera;
                Scenario scenario {"arrancada", start + 2.5f, {}, [=](float t) {
                    ScenarioCommand c;
                    if (kind == 1 && t >= 0.5f && t < 2.0f) c.move = { 1.0f, 0.0f, 0.0f };
                    if (kind == 2 && t >= 2.0f) c.lookYaw = a;
                    if (t >= start) {
                        c.move = kind == 2 ? Vec3 { std::cos(a), std::sin(a), 0.0f } : Vec3 { std::cos(a), std::sin(a), 0.0f };
                        if (turnCamera) c.lookYaw = a;
                        c.sprint = sprint;
                    } else if (kind == 1) {
                        c.sprint = false;
                    }
                    return c;
                }, {}};
                scenario.startAt = start;
                const ScenarioResult r = runScenario(scenario);
                std::printf("ARR %-8s %-6s %d quedas %d | raspadas %d (primeira %.2f) | incl %4.1f | chega %.2f | arrasto %.3f sai %.2f | deslize %.3f pior %.2f\n",
                    item.name, sprint ? "sprint" : "trote", phase, r.falls, r.scuffs, r.firstScuffAt, r.worstTilt,
                    r.startReachAt, std::max(r.startDrag[0], r.startDrag[1]), r.firstLiftAt, r.slipMeters, r.slipWorst);
            }
        }
    }
}

// Varredura de andar para a frente (sweepandar): a camera no rumo do
// movimento (sempre o ciclo de frente), 8 rumos no mundo x 2 sementes de
// variacao. Anda 8 s (1 a 9 s) e solta. Mede quedas andando (e quando),
// a velocidade media nos ultimos 3 s andando e quedas na parada.
void walkSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    const bool sprint = std::getenv("XANDARSPRINT") != nullptr;
    // XANDARSEMENTES="7,19,31": as sementes de variacao (padrao 7 e 19).
    std::vector<std::uint32_t> seeds { 7u, 19u };
    if (const char* v = std::getenv("XANDARSEMENTES")) {
        seeds.clear();
        std::string text(v);
        std::size_t start = 0;
        while (start < text.size()) {
            const std::size_t comma = text.find(',', start);
            seeds.push_back(static_cast<std::uint32_t>(std::stoul(text.substr(start, comma - start))));
            if (comma == std::string::npos) break;
            start = comma + 1;
        }
    }
    for (const std::uint32_t seed : seeds) {
        for (int heading = 0; heading < 360; heading += 45) {
            if (index++ % parts != part) continue;
            const float a = heading * 3.14159265f / 180.0f;
            // XANDARLADO=graus: anda nesse angulo da camera (90 esquerda, -90
            // direita), a camera fixa no rumo `heading`.
            const float side = std::getenv("XANDARLADO") ? std::strtof(std::getenv("XANDARLADO"), nullptr) * 3.14159265f / 180.0f : 0.0f;
            const Vec3 direction { std::cos(a + side), std::sin(a + side), 0.0f };
            char seedText[16];
            std::snprintf(seedText, sizeof seedText, "%u", seed);
            setenv("XVARIETYSEED", seedText, 1);
            // XANDARSOLTA: quando solta o comando (padrao 9 s; o fim e 3 s depois).
            const float release = std::getenv("XANDARSOLTA") ? std::strtof(std::getenv("XANDARSOLTA"), nullptr) : 9.0f;
            Scenario scenario {"andar", release + 3.0f, {}, [=](float t) {
                ScenarioCommand c;
                c.lookYaw = a;
                if (t >= 1.0f && t < release) c.move = direction;
                c.sprint = sprint;
                return c;
            }, {}};
            scenario.probeStart = 6.0f;
            scenario.probeDirection = direction;
            scenario.probeSpeed = 100.0f;
            const ScenarioResult r = runScenario(scenario);
            const float fallAt = r.firstFallSeconds;
            const double samples = std::max(1, r.gaitSustainedSamples);
            if (r.chestSamples > 1 && std::getenv("XPEITO")) {
                const double n = r.chestSamples;
                const auto sd = [&](double sum, double sq) { return std::sqrt(std::max(0.0, sq / n - (sum / n) * (sum / n))); };
                std::printf("PEITO %3d semente %2u | arfagem fisico %+.1f +-%.1f alvo %+.1f +-%.1f | rolagem fisico +-%.1f alvo +-%.1f | giro rms fisico %.2f alvo %.2f rad/s\n",
                    heading, seed, r.chestPitch[0] / n, sd(r.chestPitch[0], r.chestPitchSq[0]), r.chestPitch[1] / n, sd(r.chestPitch[1], r.chestPitchSq[1]),
                    sd(r.chestRoll[0], r.chestRollSq[0]), sd(r.chestRoll[1], r.chestRollSq[1]),
                    std::sqrt(r.chestSpinSq[0] / n), std::sqrt(r.chestSpinSq[1] / n));
                std::printf("PERNAS %3d semente %2u | pelve sobe-desce +-%.1f cm | velocidade pulsa +-%.2f m/s | pe desliza %.3f m/s (%.0f%% dos contatos > 0,2) | sola anda %.1f cm por apoio de %.0f ms\n", heading, seed,
                    100.0 * sd(r.pelvisZ, r.pelvisZSq), sd(r.alongV, r.alongVSq),
                    r.footSlipSamples ? r.footSlip / r.footSlipSamples : 0.0,
                    r.footSlipSamples ? 100.0 * r.footSlipOver / r.footSlipSamples : 0.0,
                    r.slideStances ? 100.0 * r.slideDistance / r.slideStances : 0.0,
                    r.slideStances ? 1000.0 * r.slideTime / r.slideStances : 0.0);
                std::printf("PEITOEIXO %3d semente %2u | rolagem fis %.2f alvo %.2f | arfagem fis %.2f alvo %.2f | guinada fis %.2f alvo %.2f rad/s\n", heading, seed,
                    std::sqrt(r.chestSpinAxisSq[0][0] / n), std::sqrt(r.chestSpinAxisSq[1][0] / n),
                    std::sqrt(r.chestSpinAxisSq[0][1] / n), std::sqrt(r.chestSpinAxisSq[1][1] / n),
                    std::sqrt(r.chestSpinAxisSq[0][2] / n), std::sqrt(r.chestSpinAxisSq[1][2] / n));
            }
            if (r.touchErrorCount > 0)
                std::printf("POUSO %3d semente %2u toques %d erro %.3f m frente %+.3f lado %+.3f\n", heading, seed, r.touchErrorCount,
                    r.touchErrorSum / r.touchErrorCount, r.touchErrorAlong / r.touchErrorCount, r.touchErrorAcross / r.touchErrorCount);
            std::printf("ANDAR %3d semente %2u quedas %d caiu-em %.2f fase %s | alcancou %.2f m/s | regime %.2f m/s | atravessado %+.2f m/s | giro medio %+.1f pior %.1f graus\n",
                heading, seed, r.falls, fallAt, fallAt < 0 ? "-" : fallAt < release ? "andando" : "parando",
                r.gaitBestSpeed, r.gaitSustainedSamples ? r.gaitSustainedAlong / r.gaitSustainedSamples : -1.0,
                r.gaitSustainedAcross / samples, r.gaitSignedYawSum / samples, r.gaitWorstYawDegrees);
        }
    }
    unsetenv("XVARIETYSEED");
}

// Varredura de freada: corre numa das 8 direcoes (trote ou sprint) e solta o
// comando em 4 momentos diferentes da passada (2,6 a 3,35 s de corrida).
// Mede queda, tempo ate parar (corpo abaixo de 0,2 m/s), a pior inclinacao e
// a velocidade quando caiu. Paridade com o laboratorio: E5, ajuda parada e
// andando como nele (XRETAIN/XWALKASSIST), variacao ligada, sem XLEAD.
void brakeSweep() {
    int part = 0, parts = 1;
    if (const char* p = std::getenv("XSWEEPPART")) std::sscanf(p, "%d/%d", &part, &parts);
    int index = 0;
    for (const bool sprint : { false, true }) {
        for (int heading = 0; heading < 360; heading += 45) {
            for (const float run : { 2.6f, 2.85f, 3.1f, 3.35f }) {
                if (index++ % parts != part) continue;
                const float a = heading * 3.14159265f / 180;
                const Vec3 direction {std::cos(a), std::sin(a), 0};
                // Retomar no meio da freada (XBRAKERESUME "atraso,graus"): solto
                // o comando, depois de `atraso` s pede de novo, girado `graus`
                // do rumo da corrida, e mede quando o corpo vai a 1,5 m/s nele.
                float resumeDelay = -1.0f, resumeDegrees = 0.0f;
                if (const char* v = std::getenv("XBRAKERESUME"))
                    std::sscanf(v, "%f,%f", &resumeDelay, &resumeDegrees);
                const float resumeYaw = resumeDegrees * 3.14159265f / 180.0f;
                const Vec3 resumeDirection = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, resumeYaw).rotate(direction);
                const float resumeAt = 2.0f + run + resumeDelay;
                Scenario scenario {"varredura de freada", 2.0f + run + 2.5f, {}, [=](float t) {
                    ScenarioCommand c;
                    if (t >= 2.0f && t < 2.0f + run) c.move = direction;
                    if (resumeDelay >= 0.0f && t >= resumeAt) c.move = resumeDirection;
                    c.sprint = sprint;
                    return c;
                }, {}};
                if (resumeDelay >= 0.0f) {
                    scenario.probeStart = resumeAt;
                    scenario.probeDirection = resumeDirection;
                }
                const ScenarioResult r = runScenario(scenario);
                if (resumeDelay >= 0.0f) {
                    std::printf("RETOMA %s %3d %.2f quedas %d resposta %.2f\n", sprint ? "sprint" : "trote",
                        heading, run, r.falls, r.probeReachedAt >= 0.0f ? r.probeReachedAt - resumeAt : -1.0f);
                    continue;
                }
                const float stop = r.gaitStoppedAt >= 0 && r.gaitMoveEnd >= 0 ? r.gaitStoppedAt - r.gaitMoveEnd : -1;
                int brakeSteps = 0;
                std::array<int, 10> byReason {};
                for (const auto& step : r.plannerSteps)
                    if (step.releasedAt >= r.gaitMoveEnd) {
                        ++brakeSteps;
                        ++byReason[std::min<std::size_t>(9, static_cast<std::size_t>(step.reason))];
                    }
                std::printf("FREIO %s %3d %.2f quedas %d parou %.2f inclinacao %.1f queda-em %.2f passos %d freada %d recuperacao %d acomodacao %d giro %d outros %d\n",
                    sprint ? "sprint" : "trote", heading, run, r.falls, stop, r.worstTilt,
                    r.firstFallSeconds >= 0 && r.gaitMoveEnd >= 0 ? r.firstFallSeconds - r.gaitMoveEnd : -1.0f,
                    brakeSteps, byReason[static_cast<std::size_t>(StepReason3D::Brake)],
                    byReason[static_cast<std::size_t>(StepReason3D::Recovery)],
                    byReason[static_cast<std::size_t>(StepReason3D::Settle)],
                    byReason[static_cast<std::size_t>(StepReason3D::Turn)],
                    brakeSteps - byReason[static_cast<std::size_t>(StepReason3D::Brake)]
                        - byReason[static_cast<std::size_t>(StepReason3D::Recovery)]
                        - byReason[static_cast<std::size_t>(StepReason3D::Settle)]
                        - byReason[static_cast<std::size_t>(StepReason3D::Turn)]);
            }
        }
    }
}

// Regressao da freada (a configuracao do laboratorio: E5, ajuda parada em
// 0%, andando em 100%, variacao de movimento). Soltando o comando em trote
// ou sprint nas 8 direcoes, ele para sem cair: antes, com a ajuda parada em
// 0%, todo sprint caia (32 de 32 na varredura de freada) e 12 de 32 trotes.
// A ajuda nao arrasta mais a pelve: do sprint ele leva ~1,1 s, do trote
// ~0,7 s (nas referencias Running To Stop e Run To Stop, 1,6 e 0,55 s), com
// um passo de freada na maioria das paradas e no maximo dois.
void brakeChecks() {
    struct Saved { const char* name; std::string value; bool set; };
    std::vector<Saved> saved;
    const std::array<std::pair<const char*, const char*>, 5> lab {{
        {"XE5ANIM", "1"}, {"XRETAIN", "0"}, {"XWALKASSIST", "1"}, {"XVARIETY", "1"}, {"XNOLEAD", "1"}}};
    for (const auto& [name, value] : lab) {
        const char* old = std::getenv(name);
        saved.push_back({name, old ? old : "", old != nullptr});
        setenv(name, value, 1);
    }
    int falls = 0;
    float slowestSprint = 0.0f, slowestTrot = 0.0f;
    bool allStopped = true;
    int index = 0;
    const int only = std::getenv("XFREADACASE") ? std::atoi(std::getenv("XFREADACASE")) : -1;
    for (const bool sprint : { false, true }) {
        for (int heading = 0; heading < 360; heading += 45) {
            if (only >= 0 && index++ != only) continue;
            const float a = heading * 3.14159265f / 180;
            const Vec3 direction {std::cos(a), std::sin(a), 0};
            Scenario scenario {"freada", 7.6f, {}, [=](float t) {
                ScenarioCommand c;
                if (t >= 2.0f && t < 5.1f) c.move = direction;
                c.sprint = sprint;
                return c;
            }, {}};
            const ScenarioResult r = runScenario(scenario);
            const float stop = r.gaitStoppedAt >= 0 && r.gaitMoveEnd >= 0 ? r.gaitStoppedAt - r.gaitMoveEnd : -1;
            std::printf("FREADA %s %3d quedas %d parou em %.2f s inclinacao %.1f\n",
                sprint ? "sprint" : "trote", heading, r.falls, stop, r.worstTilt);
            falls += r.falls;
            if (stop < 0.0f) allStopped = false;
            (sprint ? slowestSprint : slowestTrot) = std::max(sprint ? slowestSprint : slowestTrot, stop);
        }
    }
    for (const Saved& entry : saved) {
        if (entry.set) setenv(entry.name, entry.value.c_str(), 1);
        else unsetenv(entry.name);
    }
    require(falls == 0, "Soltando o comando correndo (ajuda parada em 0%) ele caiu");
    require(allStopped && slowestSprint < 2.5f && slowestTrot < 1.5f,
        "A freada nao parou o corpo a tempo");
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
        // Ate 80% da velocidade nominal do comando (o pedido pode vir com teto
        // - escada, curva, ajuda dosada na arrancada - e 80% dele nao e a
        // arrancada).
        scenario.startAt = 2.0f;
        const ScenarioResult result = runScenario(scenario);
        std::printf("START %s quedas %d tilt %.1f velocidade %.2f t80 %.2f\n",
            sprint ? "sprint" : "trote", result.falls, result.worstTilt,
            result.gaitBestSpeed, result.startNominal80At);
        require(result.falls == 0,
            "A arrancada derrubou o personagem");
        require(result.worstTilt < (sprint ? 28.0f : 20.0f),
            "A arrancada voltou a dobrar o corpo para a frente");
        require(result.startNominal80At >= (sprint ? 0.24f : 0.10f)
                && result.startNominal80At < (sprint ? 0.90f : 0.70f),
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
        if (which == "sweepbrake") brakeSweep();
        if (which == "sweepcurve") curveSweep();
        if (which == "sweepfall") fallSweep();
        if (which == "sweeppouso") landingSweep();
        if (which == "sweeppulofrente") forwardJumpSweep();
        if (which == "sweepvizinho") neighborSweep();
        if (which == "sweepandar") walkSweep();
        if (which == "sweeptransicao") transitionSweep();
        if (which == "sweepcurvafechada") tightCurveSweep();
        if (which == "sweeptropeco") stumbleSweep();
        if (which == "sweepvolta") modeSwitchSweep();
        if (which == "sweepida") switchToPhysicalSweep();
        if (which == "sweeppuloida") jumpSwitchSweep();
        if (which == "sweepcarrinhonpc") tackledNpcSweep();
        if (which == "sweepentrega") getUpHandoffSweep();
        if (which == "sweepcarrinhoida") tackleSwitchSweep();
        if (which == "sweepvoadoraida") airTackleSwitchSweep();
        if (which == "sweepvelocidades") gaitSpeedSweep();
        if (which == "sweepmanejoimpacto") ballImpactSweep();
        if (which == "sweepmanejorecuo") ballRetractionSweep();
        if (which == "sweepguiado") guidedBodySweep();
        if (which == "sweepimpacto") impactSweep();
        if (which == "sweepdono") ballOwnerSweep();
        if (which == "sweepboletim") locomotionReportSweep();
        if (which == "sweepapoio") braceRecoverySweep();
        if (which == "sweeptrombada") bumpSweep();
        if (which == "sweeparrancada") startSweep();
        if (which == "sweepbola") ballSweep();
        if (which == "sweeplevantar") getUpSweep();
        if (which == "sweepbrucos") proneGetUpSweep();
        if (which == "sweepcarrinho") tackleSweep();
        if (which == "sweepmanejo") ballHandlingSweep();
        if (which == "sweepchute") shotSweep();
        if (which == "sweepchutecorre") runningShotSweep();
        if (which == "sweepbolaestresse") ballStressSweep();
        if (which == "sweeprepulsa") ballRepelSweep();
        if (which == "sweepfluidez") fluiditySweep();
        if (which == "sweepgiro") standingTurnSweep();
        if (which == "sweeppostura") postureSweep();
        if (which.empty() || which == "startup") startupGaitChecks();
        if (which.empty() || which == "freada") brakeChecks();
        if (which == "benchtransfer") transferBench();
        if (which == "sweep") pressureSweep();
        if (which.empty() || which == "fall") fallScenarios();
        return 0; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
