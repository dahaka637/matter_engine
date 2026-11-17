#include "Engine/Athlete/Legacy/LegacyAthleteStep3D.hpp"
#include "Engine/Athlete/Navigation/BodyNavigation3D.hpp"

#include "Engine/Athlete/Ball/BallContactPlanner3D.hpp"
#include "Engine/Gesture/GestureCatalog3D.hpp"
#include "Engine/Gesture/GestureChoice3D.hpp"
#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"
#include "Engine/Athlete/Body/GetUpPolicy3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"
#include "Engine/Athlete/Reference/BodyForecast3D.hpp"
#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"

#include "Engine/Character/CharacterControlApplication3D.hpp"
#include "Engine/Core/Log.hpp"

#include <algorithm>
#include <limits>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace MatterEngine {
namespace {
// Levantar com pressa entregue ao padrao: a meia-vida e a duracao da inercia
// (s) - o corpo agachado vai a pose de andar enquanto ja anda.
constexpr float GetUpHandoffHalfLife = 0.16f;
constexpr float GetUpHandoffSeconds = 0.65f;
// Do Ativo ao padrao: a meia-vida e a duracao da inercia (s) do corpo ate a
// pose do clipe.
constexpr float ActiveExitHalfLife = 0.15f;
constexpr float ActiveExitSeconds = 0.60f;
// A pose da volta do fisico pela inercia (plano v4, etapa C: com 0,10 s o
// tronco ia da postura do fisico a do clipe em ~0,15 s).
constexpr float ReturnHalfLife = 0.14f;
constexpr float ReturnSeconds = 0.60f;
// A rede do levantar junto da rede da locomocao (01/11): a pelve abaixo disto (m, na escala do corpo) e a queda do fim do
// episodio do treino da locomocao (a tarefa do B21) e, sem ficar de pe e firme neste tanto (s), o classico de volta.
constexpr float GetUpFallPelvisHeight = 0.45f;
constexpr float GetUpRecoveryLimitSeconds = 10.0f;
// Parada sem subir (o centro de massa abaixo de 0,15 m/s, sem ficar de pe) por este tanto (s), ela solta ao classico.
constexpr float GetUpStuckSeconds = 2.5f;

// No Ativo, o ponto de captura alem disto (m) fora da base leva ao fisico
// pleno: sem os passos de equilibrio, a pancada que pede passo; com eles, so
// a que um passo nao segura (ou depois dos passos que cabem).
float activeCaptureLimit(const AthleteRuntimeSettings3D& settings, const ActiveStance3D& stance) {
    if (!settings.activeStanceSettings.balanceSteps) return settings.activeHandoverCapture;
    if (!stance.stepping() && stance.stepsTaken() >= settings.activeStanceSettings.maximumSteps)
        return settings.activeHandoverCapture;
    return settings.activeHandoverCaptureStepping;
}

// (Os ajustes do padrao pelos do runtime: athleteConfigureDefaultLocomotion3D,
// no AthleteRuntime3D, que o rastreador tambem usa.)
void configureDefaultLocomotion(DefaultLocomotionSettings3D& defaults, const AthleteRuntimeSettings3D& settings,
    const PlayerBody3D& body) {
    athleteConfigureDefaultLocomotion3D(defaults, settings);
    defaults.bodyScale = bodyScale3D(body);
}
} // namespace

void stepLegacyAthlete3D(PhysicsScene3D& scene, const AthleteRuntimeAssets3D& assets,
    const AthleteRuntimeSettings3D& settings, AthleteControlInput3D& control, AthleteSlot3D& slot,
    std::span<AthleteBall3D> balls, std::span<const std::pair<std::uint64_t, Vec3>> tacklerLegs, float deltaTime,
    AthleteRuntimeOutput3D& output) {
    const RagdollProfile3D& profile = *assets.profile;
    const CharacterLocomotionAnimations3D& clips = *assets.clips;
    const CharacterLocomotionAnimations3D& defaultClips = *assets.defaultClips;
    const RagdollState3D& body = *slot.state;
    AthleteState3D& athlete = *slot.athlete;
    athlete.externalAssist = {};
    LegacyAthleteState3D& state = athlete.legacy;
    if (!athlete.observer.ready()) athlete.observer.prepare(profile);
    const auto groundHeight = [&](Vec3 position) { return athleteGroundHeight3D(scene, profile, position); };
    // A reescrita (R3): os contatos do tick classificados num lugar so (o
    // chao embaixo da base, o apoio dos pes, os raspoes e os impactos). A base
    // e o chao embaixo da pelve.
    const bool modeTable = settings.modeTable && !settings.activeBody;
    if (modeTable && !athlete.contactBody.valid()) athlete.contactBody = contactBody3D(profile);
    if (!body.links.empty()) athlete.groundHeight = groundHeight(body.links.front().position);
    const float contactBase = !modeTable || body.links.empty() ? 0.0f : athlete.groundHeight;
    ContactAssessment3D contacts;
    float pushWindowed = 0.0f;
    float impactWindowed = 0.0f;
    if (!modeTable) {
        // (Sem a tabela nada disto anda: religada, ela comeca do zero.)
        athlete.pushWindow = {};
        athlete.impactWindow = {};
        athlete.modeRules.reset();
    } else {
        pushWindowed = athlete.pushWindow.add(body.externalPushNewtonSeconds, deltaTime, settings.impactWindowSeconds);
        contacts = assessContacts3D(body.interactions, {}, athlete.contactBody, {}, contactBase,
            settings.contactClasses, pushWindowed);
        impactWindowed = athlete.impactWindow.add(contacts.impactImpulse + body.externalPushNewtonSeconds, deltaTime,
            settings.impactWindowSeconds);
    }
    // (Fora do bloco: a ponte de contato nao vale para quem levou carrinho.)
    bool tackled = false;
    {
        // Levou carrinho: pancada de algo que se move e uma perna de quem
        // da o carrinho a menos de 45 cm de um link dele.
        float impulse = 0.0f;
        for (const RagdollInteraction3D& contact : body.interactions)
            if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
        if (impulse > 2.0f)
            for (const auto& [entity, leg] : tacklerLegs) {
                if (entity == slot.entityId) continue;
                for (const PhysicsBodyState3D& link : body.links)
                    if ((link.position - leg).lengthSquared() < 0.45f * 0.45f) { tackled = true; break; }
                if (tackled) break;
            }
        if (tackled && settings.tackledAssistLow) {
            if (state.tackledRecovery >= 1.0f) Log::info("corpo: levou carrinho (ajuda baixa)");
            state.tackledRecovery = 0.0f;
        } else {
            state.tackledRecovery = std::min(1.0f, state.tackledRecovery
                + deltaTime / std::max(0.1f, settings.tackledRecoverSeconds));
        }
        // A ajuda depois do impacto (pedido do usuario, 06/10): cai pela
        // forca do choque - ate 50%, ou ate 20% num carrinho levado - e
        // volta aos poucos (no padrao tambem: o choque que o leva ao
        // fisico ja conta).
        // (Com a tabela, o choque e o dos impactos de verdade: inclui a perna
        // no topo de algo, e nao conta o raspao nem o apoio.)
        {
            ImpactAssistSettings3D& curve = state.impactAssist.settings();
            const ImpactAssistSettings3D plain;
            curve.kneeImpulse = modeTable ? settings.impactKneeImpulse : plain.kneeImpulse;
            curve.kneeDrop = modeTable ? settings.impactKneeDrop : plain.kneeDrop;
            curve.fullImpulse = modeTable ? settings.impactFullImpulse : plain.fullImpulse;
            curve.maximumDrop = modeTable ? settings.impactMaximumDrop : plain.maximumDrop;
        }
        // (28/10, pedido do usuario: "quando pela propria locomocao fisica o corpo
        // voltar a pose normal, voltar mais rapido". No padrao, assentado de pe
        // desde o tick anterior, a ajuda volta defaultCalmRecoverSpeedup vezes
        // mais rapido, vezes a rapidez dele; caindo ou cambaleando, no tempo de
        // sempre - mais rapida ali, ela segurava a queda e o levantar pelo clipe
        // nao comecava.)
        const float recoverSpeedup = settings.defaultLocomotion && state.calmSeconds > 0.0f
            ? settings.defaultCalmRecoverSpeedup * std::max(0.25f, settings.defaultReturnRate)
            : 1.0f;
        state.impactAssist.update(modeTable ? impactWindowed : impactImpulse3D(body),
            tackled && settings.tackledAssistLow, impulse, deltaTime * recoverSpeedup);
        // (Com a tabela: caido ou levantando, o choque ja foi gasto no tombo -
        // a ajuda volta inteira e, de pe, ele volta ao padrao como antes. A
        // pancada no levantar tem a regra dela, getUpHitWeakens.)
        if (modeTable && state.bodyMode == LegacyBodyMode3D::Physical) {
            const CharacterLocomotionState3D now = state.locomotion.telemetry().state;
            // (E a rede do levantar depois de ir ao chao: o choque tambem ja foi gasto - 02/11, sem isto a ponte
            // da volta abortava por "ajuda caiu".)
            if (now == CharacterLocomotionState3D::Fallen || now == CharacterLocomotionState3D::GettingUp
                || (athlete.getUpRecovery.active && athlete.getUpRecovery.down))
                state.impactAssist.reset();
        }
    }
    const bool grabbed = scene.grabbing() && scene.grabbedRagdoll() == slot.handle;
    const auto& root = body.links.front();
    std::array<GroundProbeResult3D, 2> footGround;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const auto& id = profile.links[i].id;
        const int foot = id == "LeftFoot" ? 0 : id == "RightFoot" ? 1 : -1;
        if (foot < 0 || i >= body.links.size()) continue;
        // O terreno dos pes ve tambem os corpos dinamicos (caixas, props).
        footGround[foot] = scene.probeTerrain(body.links[i].position + Vec3 { 0, 0, 0.10f }, 0.05f, 0.0f, 0.40f, 48.0f);
    }
    CharacterLocomotionInput3D command;
    // As correcoes do RL agem no referencial da rede (o rumo da pelve, como
    // a entrada dela), posto aqui a cada tick pelo corpo publicado.
    if (athlete.corrections.active())
        athlete.corrections.frame = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
            locomotionFrameYaw3D(profile, root.orientation));
    command.corrections = athlete.corrections.active() ? &athlete.corrections : nullptr;
    command.rootPositionWorld = { root.position.x, root.position.y,
        groundHeight(root.position) + profile.standingRootHeightMeters };
    command.rootVelocityWorld = root.linearVelocity;
    command.facingYawRadians = state.guideFacingYawRadians;
    command.runningTurn = settings.runningTurnClip;
    command.movingHipTurn = settings.movingHipTurn;
    command.grounded = true;
    command.controlled = slot.entityId == control.controlledEntityId;
    if (command.controlled && scene.hasCharacter()) {
        const PhysicsCharacterState3D& character = scene.characterState();
        const float bodyHeight = character.crouched
            ? assets.avatarSettings.crouchedHeight
            : assets.avatarSettings.standingHeight;
        const float feetHeight = character.position.z - bodyHeight * 0.5f;
        command.rootPositionWorld = { character.position.x, character.position.y,
            feetHeight + profile.standingRootHeightMeters };
        command.rootVelocityWorld = character.velocity;
        command.proxyVelocityWorld = control.proxyVelocityWorld;
        command.intent = control.intent;
        command.facingYawRadians = control.facingYawRadians;
        state.guideFacingYawRadians = control.facingYawRadians;
        command.lookPitchRadians = control.lookPitchRadians;
        command.grounded = character.grounded;
        command.crouched = character.crouched;
        command.sprinting = control.sprinting;
        // Chao em qualquer ponto: os pes consultam onde estao e onde vao
        // pisar (so o personagem controlado; os bonecos soltos usam a
        // sonda embaixo de cada pe).
        PhysicsScene3D* scenePointer = &scene;
        command.groundAt = [scenePointer](Vec3 point) {
            return scenePointer->probeTerrain(point + Vec3 { 0.0f, 0.0f, 0.55f }, 0.03f, 0.0f, 1.5f, 60.0f);
        };
    }
    // O boneco solto no padrao (looseDefaultLocomotion, 28/10): a capsula
    // virtual dele, de dados, anda aqui antes do passo, como a do jogador anda
    // antes dele - sem pedido, a velocidade so se gasta pela resposta da capsula
    // do jogador, mais o quanto ela foi atras do corpo no tick anterior; o chao
    // embaixo. O rumo e o do corpo quando ela nasce.
    // (O piloto, AthleteCommand3D::pilot, tambem: a capsula virtual dele anda no
    // pedido, com a resposta da do jogador no padrao e a escala da volta.)
    const bool piloted = !command.controlled && slot.command != nullptr && slot.command->pilot;
    const bool looseNavigation = !command.controlled && (slot.command == nullptr || piloted)
        && settings.looseDefaultLocomotion && settings.defaultLocomotion && settings.adaptivePhysics;
    // (16/11, AthleteRuntimeSettings3D::looseContactsToPhysical) Empurrado por quem se move: ao fisico, pelo cerebro.
    const bool shoveToPhysical = looseNavigation && settings.looseContactsToPhysical;
    bool looseShoved = false;
    if (looseNavigation) {
        PhysicsCharacterState3D& capsule = state.virtualCapsule;
        const float half = 0.5f * assets.avatarSettings.standingHeight;
        if (!state.virtualCapsuleValid) {
            capsule = {};
            capsule.position = { root.position.x, root.position.y, groundHeight(root.position) + half };
            state.virtualFollowVelocity = {};
            state.guideFacingYawRadians = locomotionFrameYaw3D(profile, root.orientation);
            state.virtualCapsuleValid = true;
        }
        BodyForecastInput3D response;
        response.acceleration = assets.avatarSettings.groundAcceleration;
        response.deceleration = assets.avatarSettings.groundDeceleration;
        response.brake = assets.avatarSettings.groundBrakeDeceleration;
        Vec3 wanted;
        if (piloted) {
            const AthleteCapsuleResponse3D lab =
                athleteCapsuleResponse3D(settings.capsuleResponsePercent, settings.capsuleReversalPercent);
            response.acceleration = lab.acceleration;
            response.deceleration = lab.deceleration;
            response.brake = lab.brake;
            response.turnRate = lab.turnRate;
            const Vec3 requested = slot.command->intent.requestedVelocityWorld;
            wanted = Vec3 { requested.x, requested.y, 0.0f } * state.returnSpeedScale;
        }
        if (piloted && state.bodyMode == LegacyBodyMode3D::Physical) {
            // (No fisico o piloto anda pela ancora dos comandados; a capsula dele
            // fica no corpo, com a velocidade dele - como a do jogador, que vai
            // atras do corpo -, e a volta comeca dali, nao de onde as teclas a
            // levariam.)
            capsule.position = { root.position.x, root.position.y, 0.0f };
            capsule.velocity = { root.linearVelocity.x, root.linearVelocity.y, 0.0f };
        } else {
            // Empurrado no padrao (02/11, ele: "se eu tento empurrar o corpo ele nao sai do lugar... o correto seria
            // empurrar a capsula"): o impulso de quem encosta e se move - outro jogador, um objeto -, no plano, sobre a
            // massa, entra na velocidade dela; a locomocao padrao anda junto, com os passos, e a resposta a gasta.
            // (O peso, 02/11, ele: "os bonecos tem PESO, nao deve ser tao facil assim empurrar um boneco de 80 quilos":
            // entre dois corpos de massa parecida quem empurra divide o embalo - metade - e o empurrado firma os pes
            // contra, PushResist; e a forca e a de um corpo empurrando, PushAcceleration (m/s2, ~300 N em 80 kg).)
            constexpr float VirtualPushSeconds = 0.35f, VirtualPushMaximum = 3.0f;
            constexpr float PushShare = 0.5f, PushResist = 0.25f, PushAcceleration = 4.0f;
            Vec3& pushVelocity = state.virtualPushVelocity;
            const Vec3 own = Vec3 { capsule.velocity.x, capsule.velocity.y, 0.0f } - pushVelocity;
            pushVelocity = pushVelocity * std::exp(-deltaTime / VirtualPushSeconds);
            if (state.bodyMode != LegacyBodyMode3D::Physical) {
                // (Como personagem empurrando personagem: na direcao do contato a capsula anda ao menos na velocidade
                // de quem empurra - pelo impulso ela ia a ~0,5 m/s com o outro a 1,4, e ele passava por cima.)
                for (const RagdollInteraction3D& contact : body.interactions) {
                    if (contact.otherMotion == RagdollContactMotion3D::Static || contact.linkIndex >= body.links.size())
                        continue;
                    Vec3 normal { contact.normalWorld.x, contact.normalWorld.y, 0.0f };
                    if (normal.lengthSquared() < 0.25f) continue;
                    normal = normal.normalized();
                    const Vec3 other = body.links[contact.linkIndex].linearVelocity - contact.relativeVelocityWorld;
                    const float pusher = dot(Vec3 { other.x, other.y, 0.0f }, normal) * PushShare * (1.0f - PushResist);
                    const float now = dot(own + pushVelocity, normal);
                    if (pusher > 0.05f && pusher > now) {
                        if (shoveToPhysical) {
                            looseShoved = true;
                            continue;
                        }
                        pushVelocity += normal * std::min(pusher - now, PushAcceleration * deltaTime);
                    }
                }
                if (pushVelocity.length() > VirtualPushMaximum) pushVelocity = pushVelocity * (VirtualPushMaximum / pushVelocity.length());
            } else {
                pushVelocity = {};
            }
            const Vec3 planar = bodyForecastCapsuleStep3D(own, wanted, response, deltaTime) + pushVelocity;
            capsule.velocity = { planar.x, planar.y, 0.0f };
            capsule.position += (capsule.velocity
                + Vec3 { state.virtualFollowVelocity.x, state.virtualFollowVelocity.y, 0.0f }) * deltaTime;
        }
        capsule.position.z = groundHeight(capsule.position) + half;
        capsule.grounded = true;
        capsule.crouched = false;
        state.virtualFollowVelocity = {};
        command.rootPositionWorld = { capsule.position.x, capsule.position.y,
            capsule.position.z - half + profile.standingRootHeightMeters };
        command.rootVelocityWorld = capsule.velocity;
        command.proxyVelocityWorld = capsule.velocity;
        command.facingYawRadians = state.guideFacingYawRadians;
        PhysicsScene3D* scenePointer = &scene;
        command.groundAt = [scenePointer](Vec3 point) {
            return scenePointer->probeTerrain(point + Vec3 { 0.0f, 0.0f, 0.55f }, 0.03f, 0.0f, 1.5f, 60.0f);
        };
    } else {
        state.virtualCapsuleValid = false;
    }
    // A navegacao do padrao: a capsula do jogador (o controlado) ou a virtual do
    // boneco solto.
    const auto nav = [&]() -> const PhysicsCharacterState3D& {
        return looseNavigation ? state.virtualCapsule : scene.characterState();
    };
    const auto placeNav = [&](Vec3 feet) {
        if (looseNavigation)
            state.virtualCapsule.position = { feet.x, feet.y, feet.z + 0.5f * assets.avatarSettings.standingHeight };
        else
            scene.placeCharacter(feet, assets.avatarSettings);
    };
    const auto setNavVelocity = [&](Vec3 velocity) {
        if (looseNavigation)
            state.virtualCapsule.velocity = velocity;
        else
            scene.setCharacterVelocity(velocity);
    };
    // (O quanto a capsula vai atras do corpo: a do jogador pela saida - quem a
    // move -, a virtual no proximo tick.)
    const auto setFollow = [&](Vec3 velocity) {
        if (looseNavigation) {
            state.virtualFollowVelocity = velocity;
            return;
        }
        output.followVelocitySet = true;
        output.followVelocityWorld = velocity;
    };
    command.manipulated = grabbed;
    // Comandado sem a capsula do jogador (o treino do RL, muitos atletas na
    // mesma cena): a intencao e o olhar vem do slot.
    const bool commanded = !command.controlled && slot.command != nullptr;
    command.rootAnchored = commanded;
    if (commanded) {
        command.intent = slot.command->intent;
        command.facingYawRadians = slot.command->facingYawRadians;
        state.guideFacingYawRadians = slot.command->facingYawRadians;
        command.lookPitchRadians = slot.command->lookPitchRadians;
        // (No padrao o sprint do piloto e o dele, como o do controlado; no
        // fisico, abaixo, so andando.)
        if (piloted) command.sprinting = slot.command->sprinting;
        // (O chao em qualquer ponto, como o controlado: o pe ve onde vai
        // pisar.)
        PhysicsScene3D* scenePointer = &scene;
        command.groundAt = [scenePointer](Vec3 point) {
            return scenePointer->probeTerrain(point + Vec3 { 0.0f, 0.0f, 0.55f }, 0.03f, 0.0f, 1.5f, 60.0f);
        };
    } else {
        // (Sem comando, a capsula virtual e a curva recomecam do zero: um
        // comando novo nao herda o giro de antes.)
        athlete.commandPlanVelocity = {};
        athlete.commandDirection = {};
        athlete.commandDirectionValid = false;
        athlete.commandTurnRate = 0.0f;
    }
    if (!command.controlled && settings.adaptivePhysics
        && !(looseNavigation && state.bodyMode != LegacyBodyMode3D::Physical)) {
        // Capsula virtual dos bonecos nao controlados (modo fisico).
        if (!state.navigationAnchorValid) {
            state.navigationAnchor = root.position;
            state.navigationAnchorVelocity = {};
            state.navigationAnchorValid = true;
            // (O piloto entra no fisico com o embalo da capsula dele, como a
            // do jogador, que segue com a velocidade que tinha.)
            if (piloted)
                athlete.commandPlanVelocity = { state.virtualCapsule.velocity.x, state.virtualCapsule.velocity.y, 0.0f };
        }
        command.rootPositionWorld.x = state.navigationAnchor.x;
        command.rootPositionWorld.y = state.navigationAnchor.y;
        command.rootVelocityWorld = {};
    } else {
        state.navigationAnchorValid = false;
        state.navigationAnchorVelocity = {};
    }
    // Todos os bonecos humanos ativos: o controlado segue a capsula; os
    // outros tem a referencia no proprio corpo (empurraveis, dao passo).
    const bool adaptive = settings.adaptivePhysics;
    command.forceDrivenRoot = adaptive;
    // As forcas das juntas deste tick, copiadas: a troca para o fisico (mais
    // abaixo, toPhysical) reinicia a ajuda adaptativa e solta o vetor dela, e
    // o controle fisico lia o span dele depois - memoria ja liberada (o
    // AddressSanitizer pegou nos contratos da sonda, 16/10). A copia guarda o
    // que ele lia quando a memoria ainda estava intacta: as forcas da pancada
    // que causou a troca.
    std::vector<float> jointStrengthTick;
    if (adaptive) {
        // As juntas perto de uma pancada cedem (do tick anterior).
        jointStrengthTick = state.adaptivePhysics.output().jointStrength;
        command.jointStrength = jointStrengthTick;
        command.externalPerturbation = state.adaptivePhysics.output().perturbation;
        if (command.controlled) command.jumpCrouch = control.jumpCrouch;
    }
    command.grabbedLink = grabbed ? scene.grabbedRagdollLink() : RagdollDynamics3D::InvalidIndex;
    command.poseAuthority = settings.ragdollForceControlOverride ? settings.ragdollAuxiliaryPercent * 0.01f : 1.0f;
    // (A rigidez muscular geral do menu do personagem; o painel Ragdoll
    // ainda pode forcar outra.)
    command.muscleAuthority = settings.ragdollForceControlOverride
        ? settings.ragdollMusclePercent * 0.01f : std::clamp(settings.characterMusclePercent * 0.01f, 0.0f, 1.0f);
    command.footGround = footGround;
    // Only the controlled character pays for the mass matrix/Jacobian;
    // both the biomechanical path and the hybrid balance assist below
    // need it, everyone else in the scene stays with an invalid one.
    const RagdollDynamics3D dynamics = command.controlled ? scene.ragdollDynamics(slot.handle) : RagdollDynamics3D {};
    if (command.controlled && settings.biomechanicalExperiment) {
        BiomechanicalBipedInput3D physicalInput;
        physicalInput.desiredVelocityWorld = control.biomechanicalRequestedVelocity;
        physicalInput.desiredFacingYawRadians = control.facingYawRadians;
        physicalInput.muscleAuthority = command.muscleAuthority;
        physicalInput.balanceAssistance = settings.biomechanicalBalanceAssistPercent * 0.01f;
        // Reference poses come only from clips already vetted against the
        // profile (see loadAnimationCatalog): the character's own idle
        // and walk.
        physicalInput.idleReferenceClip = assets.idleReferenceClip;
        physicalInput.locomotionReferenceClip = assets.walkReferenceClip;
        state.biomechanics.update(profile, body, dynamics, physicalInput, deltaTime);
        const auto& physicalOutput = state.biomechanics.output();
        scene.setRagdollActiveDriveTargets(slot.handle, physicalOutput.driveTargets,
            physicalOutput.gravityCompensationEnabled);
        scene.setRagdollAnimationConstraint(slot.handle, physicalOutput.guide);
        if (physicalOutput.balanceForceWorld.lengthSquared() > 0.000001f
            || physicalOutput.balanceTorqueWorld.lengthSquared() > 0.000001f) {
            scene.applyRagdollControlRootForce(slot.handle, physicalOutput.balanceForceWorld,
                physicalOutput.balanceTorqueWorld);
            // (O experimento biomecanico: fora da regua, medido a parte.)
            athlete.externalAssist.activeNewtons += physicalOutput.balanceForceWorld.length();
        }
        return;
    }
    // E4 (parado pelas pernas) e E5 hibrida (a animacao anda com os pes
    // conscientes e um envelope unico de ajuda; parado, a E4 com a
    // parcela escolhida - harness XE5ANIM=1).
    const bool e5 = settings.legsWalkingE5;
    const bool legsStanding = settings.legsOnlyStanding || e5;
    state.locomotion.footworkSettings().recoveryCaptureTargeting = legsStanding;
    state.locomotion.footworkSettings().locomotionStepping = false;
    state.locomotion.footworkSettings().locomotionPlacementPredictor = false;
    state.adaptivePhysics.settings().jointSupport = legsStanding;
    state.adaptivePhysics.settings().jointBalance = legsStanding;
    // Na E5 a postura fica na pelve (o par pelve x pes) e o peso com a pelve
    // e os motores: pelos tornozelos/juntas, somados a ajuda, o corpo
    // tremia parado (oscilacao vertical, pes perdendo o chao). As pernas
    // fazem o equilibrio (centro de pressao, passos de captura).
    state.adaptivePhysics.settings().jointPosture = settings.legsOnlyStanding && settings.legsPostureStanding;
    state.adaptivePhysics.settings().legsWeightFraction = e5 ? 0.0f : 1.0f;
    // A 10% de ajuda, a marcha precisa produzir momento pelo apoio real:
    // as pernas fazem a aceleracao planar limitada pelo centro de
    // pressao; a pelve conserva apenas o envelope auxiliar escolhido.
    state.adaptivePhysics.settings().jointWhileWalking = e5;
    state.adaptivePhysics.settings().v2 = settings.physicalLocomotionV2;
    // (O par na regua so com a regua no comando: fora da E5 a ajuda parada
    // retida e 0 por construcao, nao a escolhida.)
    state.adaptivePhysics.settings().postureCoupleFollowsAssist = e5 && settings.postureCoupleFollowsAssist;
    state.adaptivePhysics.settings().historicalAssistRuler = settings.historicalAssistRuler;
    command.physicalV2 = settings.physicalLocomotionV2;
    state.adaptivePhysics.settings().legsAssistRetained = e5
        ? std::clamp(settings.e5StandingAssistPercent * 0.01f, 0.0f, 1.0f) : 0.0f;
    state.adaptivePhysics.settings().walkingAssistScale = e5
        ? std::clamp(settings.e5WalkingAssistPercent * 0.01f, 0.0f, 1.0f) : 1.0f;
    // Voltando do fisico ao padrao: a ajuda sobe gradativamente ate a
    // cheia so na transicao (pedido do usuario, 05/10) - o corpo fisico
    // vai a pose da animacao antes de ela assumir.
    {
        // Depois de um impacto (e do carrinho levado): a ajuda escolhida
        // vezes o quanto ja voltou.
        const float level = state.impactAssist.level();
        auto& physical = state.adaptivePhysics.settings();
        // (O peso andando fica com a escolhida: o impacto tira o empurrao, nao a
        // sustentacao - ver walkingWeightAssistScale.)
        physical.walkingWeightAssistScale = physical.walkingAssistScale;
        physical.standingWeightAssistScale = physical.legsAssistRetained;
        physical.legsAssistRetained *= level;
        physical.walkingAssistScale *= level;
    }
    if (state.returnBoost > 0.0f) {
        // (A volta ao padrao, fora da regua: marcada aqui, onde ela sobe a
        // ajuda - o boost pode zerar antes da medida, no mesmo tick.)
        athlete.externalAssist.handover = true;
        const float boost = state.returnBoost * state.returnBoost * (3.0f - 2.0f * state.returnBoost);
        auto& physical = state.adaptivePhysics.settings();
        physical.legsAssistRetained += (1.0f - physical.legsAssistRetained) * boost;
        physical.walkingAssistScale += (1.0f - physical.walkingAssistScale) * boost;
        physical.walkingWeightAssistScale += (1.0f - physical.walkingWeightAssistScale) * boost;
        physical.standingWeightAssistScale += (1.0f - physical.standingWeightAssistScale) * boost;
    }
    // Cambaleio (do tick anterior): a ajuda cai com o tonus e volta com
    // ele; amolecido, ele nao segue o comando de andar (volta junto).
    command.staggerRecovery = settings.staggerRecovery;
    {
        // (So no fisico: no padrao a locomocao fisica nao roda e o valor
        // ficaria parado no ultimo.)
        const float stagger = state.bodyMode == LegacyBodyMode3D::Physical
            ? state.locomotion.telemetry().stagger : 0.0f;
        // (A ajuda nao cai mais pelo cambaleio: cai pelo impacto, acima.)
        if (stagger > 0.0f)
            command.intent.requestedVelocityWorld = command.intent.requestedVelocityWorld * (1.0f - stagger);
        // Voltando do fisico: a pedida sai da velocidade do corpo e vai a das
        // teclas numa aceleracao suave (plano v4, etapa C); a capsula vai
        // junto pela escala dela.
        state.returnSpeedScale = 1.0f;
        if (state.returnRampLeft > 0.0f) {
            state.returnRampLeft = std::max(0.0f, state.returnRampLeft - deltaTime);
            state.returnRampAge += deltaTime;
            const Vec3 wanted { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
            Vec3 change = wanted - state.returnVelocity;
            // (16/11, ele: "nao quero sentir lentidao na locomocao": a rampa mansa da volta pelo cerebro so freia ate parar, sem tecla; com
            // tecla, a da volta de sempre.)
            const bool gentle = state.rlRamp && wanted.length() < 0.05f;
            // (Com tecla, na volta pelo cerebro: a aceleracao do proprio padrao, com o comeco suave - ver rlReturnMovingAcceleration.)
            float moving = settings.rlReturnMovingAcceleration > 0.0f ? settings.rlReturnMovingAcceleration
                : athleteCapsuleResponse3D(settings.capsuleResponsePercent, settings.capsuleReversalPercent).acceleration;
            moving *= std::clamp(state.returnRampAge / std::max(0.01f, settings.rlReturnMovingOnsetSeconds), 0.15f, 1.0f);
            const float most = std::max(0.1f, gentle ? settings.rlReturnAcceleration
                    : state.rlRamp ? moving : settings.returnAcceleration) * deltaTime;
            if (change.length() > most) change = change * (most / change.length());
            state.returnVelocity += change;
            // (Na volta pelo cerebro sem tecla, a capsula tambem vai pela rampa: sem ela, freava na resposta do padrao.)
            if (state.rlRamp && wanted.length() < 0.05f)
                setNavVelocity({ state.returnVelocity.x, state.returnVelocity.y, nav().velocity.z });
            if ((wanted - state.returnVelocity).length() < 0.02f) state.returnRampLeft = 0.0f;
            if (state.returnRampLeft <= 0.0f) state.rlRamp = false;
            command.intent.requestedVelocityWorld = { state.returnVelocity.x, state.returnVelocity.y,
                command.intent.requestedVelocityWorld.z };
            const float wantedSpeed = wanted.length();
            if (wantedSpeed > 0.05f) state.returnSpeedScale = std::clamp(state.returnVelocity.length() / wantedSpeed, 0.0f, 1.0f);
        }
        // (So o controlado: com 2+ atletas o ultimo da lista apagava a rampa
        // de volta do controlado.)
        // (E quem empurra sente o peso - 02/11: no padrao, andando contra outro corpo de pe, a capsula dele vai na
        // velocidade dividida com o outro, PushingSpeedShare, subindo e descendo em ~0,15 s.)
        if (command.controlled) {
            constexpr float PushingSpeedShare = 0.45f;
            bool pushing = false;
            if (state.bodyMode != LegacyBodyMode3D::Physical)
                for (const RagdollInteraction3D& contact : body.interactions) {
                    if (contact.otherMotion == RagdollContactMotion3D::Static || contact.linkIndex >= athlete.contactBody.regions.size()
                        || athlete.contactBody.regions[contact.linkIndex] == BodyRegion3D::Foot)
                        continue;
                    Vec3 normal { contact.normalWorld.x, contact.normalWorld.y, 0.0f };
                    if (normal.lengthSquared() < 0.25f) continue;
                    // (So de frente contra ele - ate ~25 graus do contato e o toque a frente da pelve, nao ao lado -: de
                    // raspao, passando de ombro, nao freia; a 45 graus o passe de ombro saia a 72% da velocidade.)
                    const Vec3 wanted { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
                    const float speed = wanted.length();
                    const Vec3 offset = contact.positionWorld - body.links.front().position;
                    if (speed > 0.3f && -dot(wanted, normal.normalized()) > 0.9f * speed
                        && dot(Vec3 { offset.x, offset.y, 0.0f }, wanted) > 0.12f * speed)
                        pushing = true;
                }
            state.pushingShare += ((pushing ? 1.0f : 0.0f) - state.pushingShare) * std::min(1.0f, deltaTime / 0.15f);
            output.controlledSpeedScale = state.returnSpeedScale * (1.0f - (1.0f - PushingSpeedShare) * state.pushingShare);
        }
    }
    command.balanceFootPlacement = e5 && settings.e5FootPlacement;
    command.walkingAssist = state.adaptivePhysics.settings().walkingAssistScale;
    // (O piloto fora do fisico anda pela capsula virtual do padrao, acima.)
    if (commanded && settings.adaptivePhysics && !(looseNavigation && state.bodyMode != LegacyBodyMode3D::Physical)) {
        // A capsula virtual anda no pedido como a do laboratorio no fisico: a
        // aceleracao do fisico pela ajuda andando e, com pouca ajuda, a
        // coleira (no maximo uma folga acima da velocidade do corpo). Caido ou
        // levantando, sem teclas.
        const CharacterLocomotionState3D now = state.locomotion.telemetry().state;
        const bool lying = now == CharacterLocomotionState3D::Fallen || now == CharacterLocomotionState3D::GettingUp
            || now == CharacterLocomotionState3D::Tackling;
        Vec3 wanted { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
        if (lying) wanted = {};
        const float wantedSpeed = wanted.length();
        if (wantedSpeed > 1e-4f) {
            // A curva (como physicalNavigationTurnScale3D no laboratorio): o
            // giro do rumo pedido filtrado, e virando forte a velocidade cai.
            const Vec3 direction = wanted * (1.0f / wantedSpeed);
            const float rate = athlete.commandDirectionValid
                ? std::acos(std::clamp(dot(athlete.commandDirection, direction), -1.0f, 1.0f)) / std::max(deltaTime, 1e-4f)
                : 0.0f;
            athlete.commandTurnRate += (rate - athlete.commandTurnRate) * (1.0f - std::exp(-deltaTime / 0.25f));
            athlete.commandDirection = direction;
            athlete.commandDirectionValid = true;
            if (command.grounded)
                wanted = wanted * physicalNavigationTurnScale3D(athlete.commandPlanVelocity, direction,
                    athlete.commandTurnRate, command.walkingAssist);
            const Vec3 bodyVelocity = state.locomotion.physicalState().centerOfMassVelocityWorld;
            const float scale = physicalNavigationSpeedScale3D(wanted * (1.0f / wantedSpeed), 1.0f, wantedSpeed,
                bodyVelocity, physicalNavigationLead3D(command.walkingAssist));
            wanted = wanted * std::clamp(scale, 0.0f, 1.0f);
        }
        const float acceleration = physicalNavigationAcceleration3D(command.walkingAssist);
        Vec3 change = wanted - athlete.commandPlanVelocity;
        const float most = acceleration * deltaTime;
        if (change.length() > most && change.length() > 0.0f) change = change * (most / change.length());
        athlete.commandPlanVelocity += change;
        command.rootVelocityWorld = athlete.commandPlanVelocity;
        command.proxyVelocityWorld = athlete.commandPlanVelocity;
        // O sprint so vale andando (como o do laboratorio: a tecla e a
        // capsula acima de 0,1 m/s).
        command.sprinting = slot.command->sprinting
            && Vec3 { athlete.commandPlanVelocity.x, athlete.commandPlanVelocity.y, 0.0f }.lengthSquared() > 0.01f;
        // A ancora anda no plano antes do passo, como a capsula do jogador
        // (movida antes dele): andando no fim, ela ficava um tick atras (6 cm
        // no sprint) e o corpo puxado de menos.
        state.navigationAnchor += athlete.commandPlanVelocity * deltaTime;
        command.rootPositionWorld.x = state.navigationAnchor.x;
        command.rootPositionWorld.y = state.navigationAnchor.y;
    }
    command.standingAssist = state.adaptivePhysics.settings().legsAssistRetained;
    // A regua de 0 a 100 do levantar: 100% e a ajuda aprovada (1,3 da conta
    // do levantar; era o 130% da regua antiga de 0 a 200).
    constexpr float GetUpAssistFull = 130.0f * 0.01f;  // (a mesma conta da regua antiga: 100% da os mesmos bits)
    command.getUpAssist = std::clamp(settings.getUpAssistPercent * 0.01f, 0.0f, 1.0f) * GetUpAssistFull;
    {
        // Pancada enquanto levanta (pedido do usuario, 05/10 noite): a
        // ajuda do levantar cai a getUpHitAssistPercent e volta em
        // getUpHitRecoverSeconds - ele pode cair de novo.
        const CharacterLocomotionState3D now = state.locomotion.telemetry().state;
        const bool rising = now == CharacterLocomotionState3D::GettingUp || now == CharacterLocomotionState3D::Fallen;
        float impulse = 0.0f;
        for (const RagdollInteraction3D& contact : body.interactions)
            if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
        if (rising && impulse > 3.0f && settings.getUpHitWeakens) {
            if (state.getUpHitRecovery >= 1.0f) Log::info("corpo: pancada no levantar (ajuda baixa)");
            state.getUpHitRecovery = 0.0f;
        } else {
            state.getUpHitRecovery = std::min(1.0f, state.getUpHitRecovery
                + deltaTime / std::max(0.1f, settings.getUpHitRecoverSeconds));
        }
        if (state.getUpHitRecovery < 1.0f) {
            const float x = state.getUpHitRecovery;
            const float low = std::clamp(settings.getUpHitAssistPercent * 0.01f, 0.0f, 1.0f);
            command.getUpAssist *= low + (1.0f - low) * x * x * (3.0f - 2.0f * x);
        }
    }
    command.getUpStiffness = std::clamp(settings.getUpStiffnessPercent * 0.01f, 0.2f, 1.5f);
    command.getUpStrength = std::clamp(settings.getUpStrengthPercent * 0.01f, 0.2f, 1.5f);
    // (O espaco segurado tambem e pressa para levantar.)
    command.getUpHurry = command.controlled && control.spaceHeld;
    command.getUpRetryCrooked = settings.getUpRetryCrooked;
    command.tackleEnabled = settings.tackleEnabled;
    // A voadora tambem no fisico (06/10), pela mesma opcao do padrao.
    command.airTackleEnabled = settings.defaultAirTackle;
    command.motionVariety = settings.motionVariety ? settings.motionVarietyPercent * 0.01f : 0.0f;
    command.footSpring = settings.footSpring;
    command.ankleBalance = settings.ankleBalance;
    command.varietySeed = static_cast<std::uint32_t>(slot.entityId * 0x9E3779B97F4A7C15ull >> 32);
    // (16/11, ele: "a transicao... brusca... principalmente quando o boneco esta virado para outra direcao") O rumo pedido ao padrao
    // depois da volta pelo cerebro: do rumo em que ele assumiu ao da camera (ou do piloto) numa mola criticamente amortecida com o giro
    // limitado - na troca, a pelve, a coluna e a cabeca viravam para a camera de uma vez (a pelve a 160 graus/s).
    const auto easeFacing = [&]() {
        if (!state.rlFacingActive) return;
        constexpr float Omega = 1.0f / 0.16f, MaximumRate = 3.5f;
        const float error = std::remainder(command.facingYawRadians - state.rlFacingYaw, 6.28318531f);
        state.rlFacingRate = std::clamp(state.rlFacingRate + (Omega * Omega * error - 2.0f * Omega * state.rlFacingRate) * deltaTime,
            -MaximumRate, MaximumRate);
        state.rlFacingYaw = std::remainder(state.rlFacingYaw + state.rlFacingRate * deltaTime, 6.28318531f);
        if (std::abs(error) < 0.01f && std::abs(state.rlFacingRate) < 0.05f) {
            state.rlFacingActive = false;
            return;
        }
        command.facingYawRadians = state.rlFacingYaw;
    };
    if (state.rlFacingActive && (state.bodyMode == LegacyBodyMode3D::Physical || !settings.defaultLocomotion)) state.rlFacingActive = false;
    easeFacing();
    // Locomocao padrao: o modo do corpo do personagem controlado.
    using BodyMode = LegacyBodyMode3D;
    const bool defaultMode = settings.defaultLocomotion && adaptive
        && ((command.controlled && scene.hasCharacter()) || looseNavigation);
    const auto toPhysical = [&](const char* why) {
            if (state.bodyMode != BodyMode::Physical) {
                Log::info(std::string("corpo: modo fisico (") + why + ")");
                state.physicalReason = why;
            }
            if (state.guided) {
                // O corpo guiado ja e o fisico: so as juntas, a gravidade
                // e as colisoes voltam. Sem contato (pulo fisico, padrao
                // desligado), a pose da animacao segue segurando-o por
                // defaultGuidedExitSeconds com a autoridade descendo
                // (o padrao roda junto).
                scene.setRagdollGuided(slot.handle, false);
                scene.removeRagdollVerticalSpin(slot.handle);
                state.guided = false;
                // (So sem contato - o pulo fisico, o padrao desligado: num
                // choque, a pose seguraria o embalo e empurraria o corpo
                // contra o que ele acertou.)
                const std::string reason(why);
                state.guidedExitSeconds = (reason == "pulo" || reason == "padrao desligado")
                    && !state.defaultLocomotion.output().tackling ? settings.defaultGuidedExitSeconds : 0.0f;
                state.guidedExitLeft = state.guidedExitSeconds;
            }
            // O controle fisico comeca do corpo como esta (com o embalo
            // da animacao: os links saem com a velocidade que tinham).
            if (state.bodyMode != BodyMode::Physical) {
                state.rlReturn = false;
                state.locomotion.reset(profile, body);
                state.adaptivePhysics.reset(profile);
                // (Do pulo, ou de um esbarrao no ar: o pouso e fisico e a
                // volta vem ~1 s depois dele.)
                state.physicalFromJump = std::string(why) == "pulo" || !nav().grounded;
                if (settings.defaultSeedFromBody) {
                    // O fisico continua a marcha de onde o padrao estava
                    // (fase pelos pes, sem a arrancada).
                    CharacterLocomotionSeed3D seed;
                    const Vec3 velocity = nav().velocity;
                    seed.moving = std::hypot(velocity.x, velocity.y) > 0.3f
                        || command.intent.requestedVelocityWorld.lengthSquared() > 0.01f;
                    // No meio do carrinho (a pancada nele mesmo, ao
                    // acertar alguem): o fisico continua o carrinho, com a
                    // ajuda normal (pedido do usuario, 05/10 noite).
                    const DefaultLocomotion3D::Tackle tackle = state.defaultLocomotion.tackle();
                    if (tackle.clip != nullptr) {
                        seed.tackleClip = tackle.clip;
                        seed.tackleTime = tackle.time;
                        seed.tackleRate = tackle.rate;
                        seed.tackleScale = tackle.scale;
                        seed.tackleYaw = tackle.yaw;
                        seed.tackleEntrySpeed = tackle.entrySpeed;
                        seed.tackleStart = tackle.start;
                    }
                    state.locomotion.seedMotion(profile, body, seed);
                }
            }
            state.returnBoost = 0.0f;
            state.landedSeconds = 0.0f;
            state.bridge = false;
            state.contactBridge = false;
            state.contactKick = {};
            state.contactDecisionLeft = 0.0f;
            state.getUpSeen = false;
            state.bodyMode = BodyMode::Physical;
            state.bodyModeSeconds = 0.0f;
            state.calmSeconds = 0.0f;
        };
    // O Corpo Ativo (Etapa 2): o corpo fica fisico (gravidade, limites,
    // chao) seguindo a pose do padrao, que continua rodando; os links saem
    // do guiado com a velocidade da animacao.
    const auto toActive = [&](const char* why) {
        if (state.bodyMode != BodyMode::Active) Log::info(std::string("corpo: ativo (") + why + ")");
        if (state.guided) {
            scene.setRagdollGuided(slot.handle, false);
            state.guided = false;
        }
        state.guidedExitLeft = 0.0f;
        state.returnInertia.stop();
        athlete.active.enter();
        athlete.stance.enter();
        state.bridge = false;
        state.contactBridge = false;
        state.contactDecisionLeft = 0.0f;
        state.bodyMode = BodyMode::Active;
        state.bodyModeSeconds = 0.0f;
        state.calmSeconds = 0.0f;
    };
    // A ponte de contato (os contatos inteligentes, 28/10): o corpo fica fisico
    // seguindo o padrao, que continua rodando na mesma passada e na mesma
    // velocidade (sem recomecar nem semear). Ela entra solta
    // (bridgeAuthorityStart: o choque bate num corpo fisico, como no fisico
    // pleno) e, em contactDecisionSeconds - o choque ja medido inteiro -, ou
    // vai ao fisico (a pancada forte) ou a guia sobe a proporcao do choque: de
    // quase cheia no encostao a solta perto da pancada forte; dali ela sobe a
    // cheia num tempo que cresce com ele. Outro contato depois so a baixa
    // (nunca a sobe de uma vez) e recomeca a subida.
    const float bridgeStart = std::clamp(settings.bridgeAuthorityStart, 0.0f, 1.0f);
    const auto contactFloor = [&](float deltaV) {
        const float hard = std::max(0.1f, settings.contactBridgeHardDeltaV);
        return bridgeStart + (1.0f - bridgeStart) * std::clamp(1.0f - std::max(0.0f, deltaV) / hard, 0.0f, 1.0f);
    };
    const auto contactDuration = [&](float deltaV) {
        const float base = std::max(0.05f, settings.contactBridgeBaseSeconds);
        return std::clamp(base + settings.contactBridgeSecondsPerDeltaV * std::max(0.0f, deltaV), base,
            std::max(base, settings.contactBridgeMaxSeconds));
    };
    const auto toContactBridge = [&](float deltaV) {
        if (state.bridge && state.contactBridge && state.contactDecisionLeft > 0.0f) {
            // (Ainda decidindo: segue solta.)
        } else if (state.bridge) {
            // (A guia de agora: a mesma conta da ponte, abaixo.)
            const float from = state.contactBridge ? state.bridgeAuthorityFloor : bridgeStart;
            const float share = std::clamp(state.bridgeSeconds / std::max(0.05f, state.bridgeDuration), 0.0f, 1.0f);
            const float now = from + (1.0f - from) * share * share * (3.0f - 2.0f * share);
            state.bridgeAuthorityFloor = std::min(now, contactFloor(deltaV));
            state.bridgeDuration = contactDuration(deltaV);
            state.bridgeSeconds = 0.0f;
        } else {
            state.bridgeAuthorityFloor = bridgeStart;
            state.bridgeDuration = contactDuration(deltaV);
            state.bridgeSeconds = 0.0f;
            state.contactDecisionLeft = std::max(0.0f, settings.contactDecisionSeconds);
        }
        if (!state.bridge) {
            char why[96];
            std::snprintf(why, sizeof(why), "corpo: ponte de contato (Dv %.2f m/s)", deltaV);
            Log::info(why);
            if (state.guided) {
                scene.setRagdollGuided(slot.handle, false);
                state.guided = false;
            }
            state.guidedExitLeft = 0.0f;
            athlete.active.enter(false);
            athlete.stance.enter();
            state.bodyMode = BodyMode::Active;
            state.bodyModeSeconds = 0.0f;
            state.calmSeconds = 0.0f;
        }
        state.bridge = true;
        state.contactBridge = true;
    };
    {
        if (!defaultMode) {
            toPhysical("padrao desligado");
        } else {
            const PhysicsCharacterState3D& capsule = nav();
            // Pancada: contato com algo que se move (outro boneco, objeto)
            // acima de DefaultHitImpulse, ou agarrado.
            float hitImpulse = 0.0f;
            for (const RagdollInteraction3D& contact : body.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Static)
                    hitImpulse += contact.normalImpulseNewtonSeconds;
            constexpr float DefaultHitImpulse = 4.0f;
            const bool hit = hitImpulse > DefaultHitImpulse || grabbed;
            // No carrinho do padrao (o do tick anterior), qualquer impacto
            // leva ao fisico - um boneco ou uma parede (pedido do usuario,
            // 05/10 noite). A voadora tambem: animada ate encostar em algo
            // (06/10: "so alternar para o fisico no contato", mais
            // fluido); com defaultVoadoraContactOnly desligado, ela vai
            // ao fisico ja no comeco, como antes.
            const DefaultLocomotionOutput3D& defaultBefore = state.defaultLocomotion.output();
            const bool defaultTackling = state.bodyMode != BodyMode::Physical && defaultBefore.tackling;
            constexpr float TackleHitImpulse = 0.4f;
            const bool tackleHit = defaultTackling && hitImpulse > TackleHitImpulse;
            const bool airTackle = defaultTackling && defaultBefore.airTackle && !settings.defaultVoadoraContactOnly;
            // Parede, meio-fio, frente de mureta: o corpo guiado bate de
            // verdade (contato com o que nao se move e nao e chao) e, acima
            // de WallHitImpulse, vai ao fisico na hora com o choque (pedido
            // do usuario, 05/10 noite: como no modo fisico).
            // (O pe no terreno baixo - a quina de um degrau, uma rampa, o espelho
            // de um degrau, dois degraus da escada ou meio-fio (o contato ate
            // StepTerrainHeight acima da base dela) - nao e parede: o corpo
            // guiado bate de verdade e o pe sobe por ele; contado, subir escada
            // ia ao fisico a cada degrau (plano v2, fase E). O pe num obstaculo
            // mais alto conta.)
            constexpr float StepTerrainHeight = 0.40f;
            const AthleteLegs3D& stepLegs = athlete.footsteps.ready() ? athlete.footsteps.legs() : resolveAthleteLegs3D(profile);
            const float stepBase = capsule.position.z - profile.standingRootHeightMeters;
            const auto footOnStep = [&](std::uint32_t link, Vec3 normal, Vec3 point) {
                return stepLegs.valid && (link == stepLegs.foot[0] || link == stepLegs.foot[1])
                    && (normal.z > 0.2f || point.z < stepBase + StepTerrainHeight);
            };
            float wallImpulse = 0.0f;
            for (const RagdollInteraction3D& contact : body.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Dynamic && contact.normalWorld.z < 0.6f
                    && !footOnStep(contact.linkIndex, contact.normalWorld, contact.positionWorld))
                    wallImpulse += contact.normalImpulseNewtonSeconds;
            constexpr float WallHitImpulse = 2.5f;
            const bool wallHit = state.bodyMode != BodyMode::Physical
                && wallImpulse > (defaultTackling ? TackleHitImpulse : WallHitImpulse);
            state.bodyModeSeconds += deltaTime;
            if (state.bodyMode == BodyMode::Physical) {
                // (A tabela no fisico so anda os relogios: o ultimo impacto e a
                // trava depois de uma ponte abortada.)
                if (modeTable) {
                    ModeSignals3D signals;
                    signals.contacts = contacts;
                    signals.assistLevel = state.impactAssist.level();
                    athlete.modeRules.decide(BodyModeKind3D::Physical, signals, settings.modeRules, deltaTime);
                    athlete.lastContacts = contacts;
                }
                // De volta ao padrao com o corpo assentado: de pe, no chao,
                // sem pancada, por um tempo.
                const CharacterLocomotionState3D now = state.locomotion.telemetry().state;
                const bool standingState = now == CharacterLocomotionState3D::Idle
                    || now == CharacterLocomotionState3D::Turning || now == CharacterLocomotionState3D::Walking
                    || now == CharacterLocomotionState3D::Running
                    || (state.physicalFromJump && now == CharacterLocomotionState3D::Landing);
                // (Encostado numa parede - contato com o estatico que nao e
                // chao - ele fica no fisico: voltando, os membros entravam
                // nela de novo.)
                bool againstWall = false;
                for (const RagdollInteraction3D& contact : body.interactions)
                    if (contact.otherMotion == RagdollContactMotion3D::Static && contact.normalWorld.z < 0.6f)
                        againstWall = true;
                const PhysicsBodyState3D& pelvis = body.links.front();
                const Vec3 pelvisUp = (pelvis.orientation
                    * profile.links.front().modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                const bool upright = pelvisUp.z > 0.85f;
                // (Com a ponte, encostado na parede tambem volta: ela e fisica
                // e o encosto e de verdade - sem ela, ele ficava no fisico
                // para sempre encostado.)
                const bool calm = standingState && upright && capsule.grounded && !hit
                    && (!againstWall || settings.physicalBridge)
                    && state.locomotion.telemetry().stagger < 0.1f
                    && state.adaptivePhysics.output().perturbation < (state.physicalFromJump ? 0.6f : 0.25f);
                state.calmSeconds = calm ? state.calmSeconds + deltaTime : 0.0f;
                // (Do pulo: ~1 s de fisica depois do pouso - pedido do
                // usuario, 05/10 -, contado do toque, nao da decolagem.)
                state.landedSeconds = capsule.grounded ? state.landedSeconds + deltaTime : 0.0f;
                // (Levantou com pressa - as teclas: volta logo, ele sai
                // andando/correndo no padrao. Com a ponte, todo fim de
                // levantar volta logo: ela segura o corpo ate o padrao - de pe
                // no fisico ele esperava ~1 s, com a cabeca para tras.)
                if (now == CharacterLocomotionState3D::GettingUp) state.getUpSeen = true;
                const bool hurried = state.locomotion.telemetry().getUpHurried
                    || (settings.physicalBridge && state.getUpSeen);
                // (Com a ponte segurando a passagem, a espera pela calma e mais
                // curta: depois de um encostao, andar no fisico mais de 1 s era
                // onde o tombo acontecia.)
                const bool bridged = settings.physicalBridge;
                // (28/10, pedido do usuario - a volta ao padrao demorava: com a
                // ponte, 0,3 s no minimo e 0,15 de calma; eram 0,5 e 0,25. A
                // rapidez dele, defaultReturnRate, divide as esperas.)
                const float rate = std::max(0.25f, settings.defaultReturnRate);
                const float minimumSeconds = (state.physicalFromJump || hurried ? 0.0f : bridged ? 0.3f : 0.8f) / rate;
                const float calmSeconds
                    = (hurried ? 0.08f : state.physicalFromJump ? 0.15f : bridged ? 0.15f : 0.4f) / rate;
                const bool landedLongEnough = !state.physicalFromJump || state.landedSeconds > 1.0f;
                // Pronto para voltar: a ajuda sobe ate a cheia em
                // ReturnBoostSeconds e so entao ele volta (se deixar de
                // estar assentado no meio, a ajuda desce de novo).
                const float ReturnBoostSeconds = (hurried ? 0.15f : bridged ? 0.15f : 0.4f) / rate;
                // (E com a ajuda de volta inteira depois do impacto - 06/10.)
                const bool ready = state.bodyModeSeconds > minimumSeconds
                    && state.calmSeconds > calmSeconds && landedLongEnough
                    && !state.impactAssist.recovering()
                    && (!modeTable || athlete.modeRules.returnAllowed(settings.modeRules));
                state.returnBoost = ready
                    ? std::min(1.0f, state.returnBoost + deltaTime / ReturnBoostSeconds)
                    : std::max(0.0f, state.returnBoost - deltaTime / 0.2f);
                // Levantando com pressa (as teclas), ja sobre os pes: o
                // padrao assume ja, andando para onde elas pedem - o
                // resto do levantar sai pela inercia, mais longa (pedido
                // do usuario, 05/10 noite: "comecar a levantar e ja ir
                // indo para a direcao que quero").
                // (So com Shift - correndo; so com as teclas, o levantar
                // rapido e a volta logo ao padrao. O Shift pela tecla: o
                // sprint so liga com a capsula andando, e caido ela esta
                // parada embaixo do corpo.)
                const bool shiftHeld = command.intent.effort > 0.5f;
                const bool handoff = settings.defaultGetUpHandoff && settings.defaultSeedFromBody && !hit
                    && shiftHeld && state.locomotion.telemetry().getUpHandoffReady;
                // (02/11, ele: depois de levantar, "assim que ele recem se ergue" ja o padrao, sem esperar de pe -
                // se nada encosta: outro boneco, objeto ou parede.)
                bool touched = hit;
                for (const RagdollInteraction3D& contact : body.interactions)
                    if (contact.otherMotion != RagdollContactMotion3D::Static || contact.normalWorld.z < 0.6f) touched = true;
                const bool risenReturn = state.getUpSeen && standingState && upright && capsule.grounded && !touched
                    && now != CharacterLocomotionState3D::GettingUp;
                if (state.rlReturn || handoff || risenReturn || (ready && state.returnBoost >= 1.0f)) {
                    // (A volta pelo cerebro do RL: ver AthleteRuntimeSettings3D::rlReturnHalfLife.)
                    const bool rlHandback = state.rlReturn;
                    state.rlReturn = false;
                    state.returnBoost = 0.0f;
                    // (16/11, BodyNavigation3D) A volta comeca com a navegacao onde o corpo esta - a capsula, a virtual e a ancora - e a
                    // raiz do padrao deste tick ja dela: com a ancora do boneco parada onde ele caiu (o cerebro dirigindo), a volta o
                    // arrastava metros de volta ate ela.
                    // (Da mistura do cerebro, a navegacao ja esta na raiz da sombra - posta por ele -, de onde o padrao adotado continua.)
                    BodyNavigationPoint3D here;
                    if (rlHandback && state.rlShadowAdopted) {
                        const PhysicsCharacterState3D& at = nav();
                        const float half = 0.5f * (at.crouched ? assets.avatarSettings.crouchedHeight : assets.avatarSettings.standingHeight);
                        here.feet = { at.position.x, at.position.y, at.position.z - half };
                        here.velocity = { at.velocity.x, at.velocity.y, 0.0f };
                    } else {
                        here = bodyNavigationPoint3D(scene, profile, body);
                        placeBodyNavigation3D(scene, assets.avatarSettings, command.controlled, state, here);
                    }
                    command.rootPositionWorld = here.feet + Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
                    command.rootVelocityWorld = here.velocity;
                    // A volta: a pose fisica de agora (raiz em relacao a base
                    // da capsula, juntas) misturada ate a animada.
                    const Vec3 feet = here.feet;
                    state.returnRootOffset = pelvis.position - feet;
                    state.returnRootOrientation = pelvis.orientation;
                    state.returnCoordinates.assign(profile.links.size(), {});
                    for (std::size_t i = 1; i < state.returnCoordinates.size() && i < body.joints.size(); ++i) {
                        const auto& joint = body.joints[i].positionRadians;
                        state.returnCoordinates[i] = { joint[0], joint[1], joint[2] };
                    }
                    // (O boneco solto volta olhando para onde o corpo esta - da sombra do cerebro, o rumo dela, que ja e o do corpo com
                    // a abertura da pelve do parado -; o piloto, como o controlado, para onde ele olha.)
                    if (looseNavigation && !piloted) {
                        const DefaultLocomotionOutput3D& adopted = state.defaultLocomotion.output();
                        state.guideFacingYawRadians = rlHandback && state.rlShadowAdopted && adopted.valid ? adopted.facingYawRadians
                                                                                                     : locomotionFrameYaw3D(profile, pelvis.orientation);
                        command.facingYawRadians = state.guideFacingYawRadians;
                    }
                    // (Na volta pelo cerebro com a sombra, o padrao ja roda na fase do corpo: segue dela.)
                    const bool shadowAdopted = rlHandback && state.rlShadowAdopted;
                    state.rlShadowAdopted = false;
                    // (O rumo pedido comeca no do padrao adotado - o da sombra - ou, na volta direta, no da pelve, e vai ao da camera pela
                    // mola de easeFacing.)
                    if (rlHandback) {
                        const DefaultLocomotionOutput3D& adopted = state.defaultLocomotion.output();
                        state.rlFacingYaw = shadowAdopted && adopted.valid ? adopted.facingYawRadians
                                                                           : locomotionFrameYaw3D(profile, pelvis.orientation);
                        state.rlFacingRate = 0.0f;
                        state.rlFacingActive = true;
                        // (Ja neste tick: o recomeco e a semente do padrao, abaixo, leem o rumo pedido.)
                        easeFacing();
                    }
                    if (!shadowAdopted) state.defaultLocomotion.reset(command.facingYawRadians);
                    if (settings.defaultSeedFromBody) {
                        // O padrao comeca de onde o corpo esta: pelve no
                        // rumo dela, na velocidade e na fase dos pes.
                        DefaultLocomotionSeed3D seed;
                        const Vec3 forward = (pelvis.orientation
                            * profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                        seed.pelvisYawRadians = std::atan2(forward.y, forward.x);
                        // (A velocidade do corpo, nao a da capsula: no fisico
                        // ele anda mais devagar que ela.)
                        seed.velocityWorld = settings.returnSmoothing
                            ? Vec3 { pelvis.linearVelocity.x, pelvis.linearVelocity.y, capsule.velocity.z }
                            : capsule.velocity;
                        seed.pelvisWorld = pelvis.position;
                        std::size_t feetLinks[2] = { 0, 0 };
                        for (std::size_t i = 0; i < profile.links.size(); ++i) {
                            if (profile.links[i].id == "LeftFoot") feetLinks[0] = i;
                            if (profile.links[i].id == "RightFoot") feetLinks[1] = i;
                        }
                        seed.feetValid = feetLinks[0] != 0 && feetLinks[1] != 0
                            && feetLinks[0] < body.links.size() && feetLinks[1] < body.links.size();
                        seed.pelvisVelocityWorld = pelvis.linearVelocity;
                        if (seed.feetValid)
                            for (std::size_t side = 0; side < 2; ++side) {
                                seed.feetWorld[side] = body.links[feetLinks[side]].position;
                                seed.feetVelocityWorld[side] = body.links[feetLinks[side]].linearVelocity;
                                seed.feetOrientationWorld[side] = body.links[feetLinks[side]].orientation;
                            }
                        // (16/11, a volta pelo cerebro: os pes do padrao apoiados onde os do corpo estao - a acomodacao por passos, nunca
                        // deslizando ate o lugar do clipe.)
                        seed.plantFeet = rlHandback && seed.feetValid;
                        if (!shadowAdopted) state.defaultLocomotion.seed(seed);
                        // E a pose sai pela inercia: a diferenca para a
                        // animacao, com a velocidade que o corpo tinha.
                        std::vector<Vec3> velocities(state.returnCoordinates.size());
                        for (std::size_t i = 1; i < velocities.size() && i < body.joints.size(); ++i) {
                            const auto& joint = body.joints[i].velocityRadiansPerSecond;
                            velocities[i] = { joint[0], joint[1], joint[2] };
                        }
                        state.returnInertia.start(state.returnCoordinates, velocities, pelvis.position,
                            pelvis.orientation, pelvis.linearVelocity, pelvis.angularVelocity,
                            rlHandback ? (state.rlReturnHalfLife > 0.0f ? state.rlReturnHalfLife : settings.rlReturnHalfLife)
                                : handoff ? GetUpHandoffHalfLife : settings.returnSmoothing ? ReturnHalfLife : 0.10f,
                            rlHandback ? (state.rlReturnSeconds > 0.0f ? state.rlReturnSeconds : settings.rlReturnSeconds)
                                : handoff ? GetUpHandoffSeconds : settings.returnSmoothing ? ReturnSeconds : 0.45f);
                        state.rlResidualHalfLife = state.rlReturnHalfLife > 0.0f ? state.rlReturnHalfLife : settings.rlReturnHalfLife;
                        state.rlReturnHalfLife = state.rlReturnSeconds = -1.0f;
                    } else {
                        state.returnInertia.stop();
                    }
                    // A velocidade sai da do corpo: a capsula com ela, e a
                    // pedida indo as teclas aos poucos (plano v4, etapa C). (No
                    // levantar com pressa, a arrancada ja e a dele.)
                    // (A capsula ja esta embaixo do corpo, acima.)
                    if (!handoff && settings.returnSmoothing) {
                        state.returnVelocity = { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f };
                        state.returnRampLeft = rlHandback ? settings.rlReturnRampSeconds : settings.returnRampSeconds;
                        state.returnRampAge = 0.0f;
                        state.rlRamp = rlHandback;
                        setNavVelocity({ state.returnVelocity.x, state.returnVelocity.y, capsule.velocity.z });
                    }
                    if (rlHandback && state.returnInertia.active()) {
                        // (A volta pelo cerebro: direto ao guiado, da pose e das velocidades do corpo de agora.)
                        scene.setRagdollGuided(slot.handle, true);
                        state.rlResidualPending = true;
                        state.rlResidualAge = -1.0f;
                        state.guided = true;
                        state.bodyMode = BodyMode::Returning;
                        state.bridge = false;
                        state.contactBridge = false;
                    } else if (settings.activeBody) {
                        // A volta pelo Corpo Ativo: o corpo fisico vai a pose
                        // do padrao pelos musculos e pelos pes (sem mistura de
                        // poses); quando colar nela, o guiado assume.
                        athlete.active.enter(true);
                        athlete.stance.enter();
                        state.bodyMode = BodyMode::Active;
                        // A capsula no meio dos pes: a pose do padrao poe o
                        // centro de massa sobre a base de verdade (o fisico o
                        // segurava com a ajuda na pelve; a capsula e so
                        // navegacao).
                        Vec3 base = pelvis.position;
                        {
                            const std::uint32_t left = athlete.observer.footLink(0);
                            const std::uint32_t right = athlete.observer.footLink(1);
                            if (left > 0 && right > 0 && left < body.links.size() && right < body.links.size())
                                base = (body.links[left].position + body.links[right].position) * 0.5f;
                        }
                        placeNav({ base.x, base.y, groundHeight(pelvis.position) });
                    } else if (settings.physicalBridge) {
                        // A ponte fisico -> padrao (plano v2, fase D): o corpo
                        // segue fisico (gravidade, juntas, chao) por um tempo
                        // curto - os musculos indo a pose do padrao (que ja
                        // roda: anda e corre desde o primeiro tick), a ajuda de
                        // passagem e o guiamento dos ossos subindo de fraco a
                        // cheio -; no fim, o guiado.
                        athlete.active.enter(true);
                        athlete.stance.enter();
                        state.bodyMode = BodyMode::Active;
                        state.bridge = true;
                        state.contactBridge = false;
                        state.bridgeSeconds = 0.0f;
                        const bool moving = handoff || command.intent.requestedVelocityWorld.lengthSquared() > 0.25f;
                        // (A rapidez nao a encurta - 02/11: a ponte ja e o padrao rodando; curta, ela nao deixava a
                        // volta mais rapida, so o encaixe mais brusco: a 300%, 0,1 s e picos de 2000 m/s2.)
                        state.bridgeDuration = moving ? settings.bridgeMovingSeconds : settings.bridgeSeconds;
                        if (!settings.returnSmoothing) state.bridgeDuration = moving ? 0.30f : 0.45f;
                    } else {
                        scene.setRagdollGuided(slot.handle, true);
                        state.guided = true;
                        state.bodyMode = BodyMode::Returning;
                    }
                    state.guidedExitLeft = 0.0f;
                    state.bodyModeSeconds = 0.0f;
                    state.airSeconds = 0.0f;
                    if (handoff) {
                        // Do agachado, a arrancada (Crouched To Sprinting)
                        // para onde as teclas pedem.
                        const Vec3 keys = command.intent.requestedVelocityWorld;
                        const float travelYaw = keys.lengthSquared() > 1e-4f ? std::atan2(keys.y, keys.x)
                            : command.facingYawRadians;
                        state.defaultLocomotion.startCrouchSprint(travelYaw, true);
                    }
                    Log::info(handoff ? "corpo: levantando - arrancada agachada (padrao)" : "corpo: voltando ao padrao");
                }
            } else {
                // Animado: a fisica entra quando ele bate em algo fisico
                // (pedidos do usuario, 04-05/10) - outro boneco ou objeto
                // (pancada), a capsula batendo forte (parede), um membro
                // entrando no mundo estatico (parede, meio-fio: o pe que
                // tropeca) - e no pulo (o voo e o pouso com fisica; volta
                // logo depois do pouso).
                constexpr float WallImpact = 1.5f;
                const bool wall = (command.controlled ? control.capsuleWallImpact : 0.0f) > WallImpact;
                bool penetrating = false;
                std::string penetratingLink;
                // (No carrinho, qualquer encostada: 5 mm.)
                const float PenetrationDepth = defaultTackling ? 0.005f
                    : modeTable ? std::min(0.015f, settings.contactClasses.penetrationMeters) : 0.015f;
                std::vector<RagdollWorldContact3D> worldContacts;
                {
                    std::vector<Vec3> positions(body.links.size());
                    std::vector<Quaternion> orientations(body.links.size());
                    for (std::size_t i = 0; i < positions.size(); ++i) {
                        positions[i] = body.links[i].position;
                        orientations[i] = body.links[i].orientation;
                    }
                    worldContacts = scene.ragdollPoseWorldContacts(slot.handle, positions, orientations, PenetrationDepth);
                    for (const RagdollWorldContact3D& contact : worldContacts)
                        // (O chao - normal para cima - nao conta, nem o pe na
                        // quina do degrau.)
                        if (contact.normalWorld.z < 0.6f
                            && !footOnStep(contact.linkIndex, contact.normalWorld, contact.pointWorld)) {
                            penetrating = true;
                            if (contact.linkIndex < profile.links.size()) penetratingLink = profile.links[contact.linkIndex].id;
                        }
                }
                // Pulo: o voo e o pouso no fisico so com "Pulo fisico" no
                // laboratorio (o padrao e o pulo animado - pedido do usuario,
                // 05/10 noite: "para melhor estabilidade"); so um esbarrao no
                // ar (as pancadas acima) o leva ao fisico, e ai o pouso tambem
                // e fisico. A decolagem fica animada por JumpTakeoffSeconds:
                // trocando no primeiro tick no ar, o corpo ainda estava parado
                // no chao e nao levava o impulso.
                constexpr float JumpTakeoffSeconds = 0.05f;
                state.airSeconds = capsule.grounded ? state.airSeconds : state.airSeconds + deltaTime;
                const bool jumped = settings.defaultPhysicalJump
                    && ((!capsule.grounded && capsule.velocity.z > 1.0f && state.airSeconds >= JumpTakeoffSeconds)
                        || (capsule.grounded && state.airSeconds > 0.25f));
                if (capsule.grounded) state.airSeconds = 0.0f;
                if (modeTable) {
                    // A tabela (R3): a mesma regra de impacto no padrao, na
                    // volta e na ponte; a ponte abortada pelo corpo saindo da
                    // pose, pelo tronco tombando, pela falta de apoio ou pela
                    // ajuda caindo.
                    ContactClassParams3D classes = settings.contactClasses;
                    if (defaultTackling) classes.penetrationMeters = std::min(classes.penetrationMeters, 0.005f);
                    ModeSignals3D signals;
                    signals.grabbed = grabbed;
                    signals.contacts = assessContacts3D(body.interactions, worldContacts, athlete.contactBody, {},
                        contactBase, classes, pushWindowed);
                    // (A regua da pancada forte: a janela de impulsos sobre a massa
                    // e o golpe pendente - o impulso no plano dos contatos com o
                    // que se move, que se gasta em contactBridgeKickSeconds; um
                    // choque espalhado por varios ticks conta inteiro.)
                    const float bodyMass = std::max(1.0f, athlete.contactBody.massKg);
                    const float windowDeltaV = athlete.impactWindow.value / bodyMass;
                    state.contactKick = state.contactKick
                        * std::exp(-deltaTime / std::max(0.02f, settings.contactBridgeKickSeconds));
                    // (Na ponte de contato todo contato com o que se move empurra,
                    // tambem os ticks do mesmo choque abaixo do limiar.)
                    if ((signals.contacts.impact && signals.contacts.fromMoving) || state.contactBridge)
                        for (const RagdollInteraction3D& contact : body.interactions)
                            if (contact.otherMotion != RagdollContactMotion3D::Static)
                                state.contactKick += Vec3 { contact.normalImpulseWorld.x, contact.normalImpulseWorld.y, 0.0f }
                                    * (1.0f / bodyMass);
                    const float hitDeltaV = std::max(windowDeltaV, state.contactKick.length());
                    const bool belowHard = settings.contactBridge && hitDeltaV < settings.contactBridgeHardDeltaV;
                    // (No carrinho, o contato com quem se move so leva ao fisico na
                    // pancada forte - pedido do usuario, 28/10: "o carrinho deve
                    // ocorrer em locomocao default (ai entra em fisico se houver
                    // pancada forte o suficiente, o levantar dele tambem se estiver
                    // livre de contato)"; o mais leve vai a ponte de contato, e a
                    // parede, como antes. Varrer de sprint quem esta de pe ja e
                    // pancada forte: o fisico continua o carrinho com a massa de
                    // verdade (com uma regua mais alta, o deslize roteirizado
                    // carregava o outro 7 m).)
                    // (29/10, "ou trava ou joga longe": o carrinho que pega um corpo
                    // de verdade - chegando nele acima de TackleBodyApproach - vai ao
                    // fisico tambem abaixo da regua. No corpo animado o golpe medido
                    // e pequeno - Dv 0,2 m/s chegando a 5,5 m/s -, e na ponte a
                    // locomocao dele seguia correndo por cima do outro, levando-o
                    // 3,5 m. A bola nao entra aqui: ela empurra o corpo por fora dos
                    // contatos.)
                    constexpr float TackleBodyApproach = 1.5f;
                    float tackleApproach = 0.0f;
                    if (defaultTackling)
                        for (const RagdollInteraction3D& contact : body.interactions)
                            if (contact.otherMotion != RagdollContactMotion3D::Static)
                                tackleApproach = std::max(tackleApproach, contactApproachSpeed3D(contact));
                    const bool tackleBody = tackleApproach > TackleBodyApproach;
                    signals.actionContact = defaultTackling
                        && ((hitImpulse > TackleHitImpulse && (!belowHard || tackleBody)) || wallImpulse > TackleHitImpulse);
                    signals.airTackleStart = airTackle;
                    signals.jumped = jumped;
                    // (No carrinho do padrao o clipe poe a capsula embaixo da pelve
                    // dele: a "batida" que a capsula mede ali e a propria
                    // colocacao - 7,55 m/s no comeco de todo carrinho, 28/10 -, nao
                    // uma parede. A parede de verdade e o corpo que bate.)
                    signals.capsuleWallImpact = defaultTackling || !command.controlled ? 0.0f : control.capsuleWallImpact;
                    // (Na ponte de contato a guia nao cai com a ajuda do impacto:
                    // a regra "ponte: ajuda caiu" nao vale.)
                    signals.assistLevel = state.contactBridge ? 1.0f : state.impactAssist.level();
                    if (state.bridge) {
                        const BodyObservation3D observation = athlete.observer.observe(profile, body, deltaTime);
                        signals.supported = observation.supportCount > 0;
                        // (No carrinho o corpo desliza no chao: sem apoio por desenho.)
                        signals.airborneByDesign = !capsule.grounded || defaultTackling;
                        const RagdollAnimationPose3D& pose = state.animationPose;
                        // O peito tombando fora do que a pose faz (rad/s): a
                        // velocidade do "para cima" do peito do corpo menos a
                        // da pose (no fim do levantar o tronco sobe rapido de
                        // proposito).
                        const std::uint32_t chest = athlete.observer.chestLink();
                        if (chest < body.links.size() && chest < profile.links.size()) {
                            const Vec3 bodyUpRate = cross(body.links[chest].angularVelocity, observation.chestUp);
                            Vec3 poseUpRate;
                            const RagdollAnimationPose3D& before = state.previousAnimationPose;
                            if (chest < pose.linkOrientations.size() && chest < before.linkOrientations.size()
                                && deltaTime > 0.0f) {
                                const Quaternion model = profile.links[chest].modelOrientation.conjugate();
                                const Vec3 now = (pose.linkOrientations[chest] * model).rotate({ 0.0f, 0.0f, 1.0f });
                                const Vec3 then = (before.linkOrientations[chest] * model).rotate({ 0.0f, 0.0f, 1.0f });
                                poseUpRate = (now - then) * (1.0f / deltaTime);
                            }
                            signals.chestTiltRate = (bodyUpRate - poseUpRate).length();
                        }
                        if (observation.valid && pose.linkPositions.size() == profile.links.size()
                            && pose.linkOrientations.size() == profile.links.size()) {
                            Vec3 poseCenter;
                            float poseMass = 0.0f;
                            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                                const float m = std::max(0.0f, profile.links[i].massFraction);
                                poseCenter += (pose.linkPositions[i]
                                    + pose.linkOrientations[i].rotate(profile.links[i].centerOfMassLocal)) * m;
                                poseMass += m;
                            }
                            if (poseMass > 0.0f) poseCenter = poseCenter * (1.0f / poseMass);
                            Vec3 bodySupport;
                            Vec3 poseSupport;
                            int count = 0;
                            for (int pass = 0; pass < 2 && count == 0; ++pass)
                                for (const FootObservation3D& foot : observation.feet) {
                                    if (!foot.valid || foot.link >= body.links.size()) continue;
                                    if (pass == 0 && !foot.supporting) continue;
                                    bodySupport += body.links[foot.link].position;
                                    poseSupport += pose.linkPositions[foot.link];
                                    ++count;
                                }
                            if (count > 0) {
                                const float inverse = 1.0f / static_cast<float>(count);
                                signals.pendulumDegrees = pendulumDegrees3D(observation.centerOfMass,
                                    bodySupport * inverse);
                                signals.poseDeviationDegrees = pendulumDeviationDegrees3D(observation.centerOfMass,
                                    bodySupport * inverse, poseCenter, poseSupport * inverse);
                            }
                        }
                    }
                    const BodyModeKind3D kind = state.bridge ? BodyModeKind3D::Bridge
                        : state.bodyMode == BodyMode::Returning ? BodyModeKind3D::Returning : BodyModeKind3D::Animated;
                    // Os contatos inteligentes (28/10): o impacto do que se move
                    // abaixo da pancada forte - no chao, fora do carrinho e sem o
                    // mundo parado - vai a ponte de contato (ou a recomeca), nao ao
                    // fisico; no carrinho do padrao ele nao tira o corpo do
                    // padrao. A tabela decide o resto sem ele (a perda de base da
                    // ponte, o pulo, o agarrao).
                    const ContactAssessment3D& hitNow = signals.contacts;
                    const bool movingOnly = hitNow.impact && !hitNow.fromPenetration && hitNow.fromMoving
                        && !hitNow.fromStatic && hitNow.penetrationMeters <= 0.0f && !grabbed;
                    // (No carrinho o contato leve tambem vai a ponte - o corpo fisico
                    // seguindo o clipe -: guiado ele nao cede.)
                    const bool softContact = belowHard && settings.physicalBridge && movingOnly && !tackled
                        && (capsule.grounded || defaultTackling) && !tackleBody;
                    ModeSignals3D tableSignals = signals;
                    if (softContact) tableSignals.contacts.impact = false;
                    const ModeDecision3D decision = athlete.modeRules.decide(kind, tableSignals, settings.modeRules, deltaTime);
                    athlete.lastContacts = signals.contacts;
                    if (softContact && !decision.change) {
                        athlete.modeRules.noteImpact();
                        if (shoveToPhysical) toPhysical("encostao (o cerebro segura)");
                        else toContactBridge(hitDeltaV);
                    }
                    // (A ponte de contato decidindo: com o golpe medido, a pancada
                    // forte vai ao fisico; senao, a guia sobe a proporcao dele.)
                    if (state.contactBridge && state.contactDecisionLeft > 0.0f && !decision.change) {
                        if (!belowHard) {
                            char strong[96];
                            std::snprintf(strong, sizeof(strong), "pancada forte (golpe %.2f m/s)", hitDeltaV);
                            toPhysical(strong);
                        } else {
                            state.contactDecisionLeft -= deltaTime;
                            if (state.contactDecisionLeft <= 0.0f) {
                                state.contactDecisionLeft = 0.0f;
                                state.bridgeAuthorityFloor = contactFloor(hitDeltaV);
                                state.bridgeDuration = contactDuration(hitDeltaV);
                                state.bridgeSeconds = 0.0f;
                            }
                        }
                    }
                    if (decision.change) {
                        athlete.modeRules.noteTransition(kind, decision, settings.modeRules);
                        athlete.lastDecision = decision;
                        athlete.lastDecisionFrom = kind;
                        std::string why = decision.event == ModeEvent3D::Jump ? std::string("pulo")
                                                                             : std::string(modeEventName3D(decision.event));
                        if (decision.event == ModeEvent3D::Impact || decision.event == ModeEvent3D::Penetration) {
                            const std::uint32_t link = signals.contacts.link;
                            if (link < profile.links.size()) why += ": " + profile.links[link].id;
                            if (decision.event == ModeEvent3D::Impact) {
                                char detail[80];
                                std::snprintf(detail, sizeof(detail), " Dv %.2f m/s (janela %.0f N.s)",
                                    signals.contacts.bodyDeltaV, athlete.impactWindow.value);
                                why += detail;
                            }
                            // (A penetracao nao tem impulso medido: o que o
                            // mundo parado vai tirar da parte - a velocidade
                            // dela contra a superficie x a massa - entra na
                            // ajuda do impacto agora.)
                            if (decision.event == ModeEvent3D::Penetration && link < body.links.size()) {
                                Vec3 normal { 0.0f, 0.0f, 1.0f };
                                for (const RagdollWorldContact3D& contact : worldContacts)
                                    if (contact.linkIndex == link) normal = contact.normalWorld;
                                const float into = std::max(0.0f, -dot(body.links[link].linearVelocity, normal));
                                const float mass = link < athlete.contactBody.linkMassKg.size()
                                    ? athlete.contactBody.linkMassKg[link] : 0.0f;
                                athlete.impactWindow.add(into * mass, 0.0f, settings.impactWindowSeconds);
                                state.impactAssist.update(athlete.impactWindow.value, false, 0.0f, 0.0f);
                            }
                        } else if (decision.threshold > 0.0f) {
                            char detail[64];
                            std::snprintf(detail, sizeof(detail), " %.2f (limiar %.2f)", decision.value, decision.threshold);
                            why += detail;
                        }
                        toPhysical(why.c_str());
                    }
                } else if (!settings.activeBody && !state.bridge) {
                    if (hit || tackleHit || airTackle || wall || wallHit || penetrating || jumped)
                        toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : hit || tackleHit ? "pancada"
                            : wall || wallHit ? "parede" : penetrating ? ("membro no mundo: " + penetratingLink).c_str()
                            : jumped ? "pulo" : "pancada");
                } else if (state.bodyMode == BodyMode::Active) {
                    // No Ativo (e na ponte) o corpo ja e fisico (encostar na
                    // parede e contato de verdade): so o que derruba leva ao
                    // pleno.
                    if (grabbed || tackleHit || airTackle || hitImpulse > settings.fullPhysicalImpulse)
                        toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : "pancada forte");
                } else if (grabbed || tackleHit || airTackle || jumped || hitImpulse > settings.fullPhysicalImpulse
                    || (command.controlled ? control.capsuleWallImpact : 0.0f) > settings.fullPhysicalWallImpact) {
                    toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : tackleHit ? "pancada"
                        : jumped ? "pulo" : hitImpulse > settings.fullPhysicalImpulse ? "pancada forte" : "parede");
                } else if (hitImpulse > settings.activeEnterImpulse || wall || wallHit || penetrating) {
                    toActive(hitImpulse > settings.activeEnterImpulse ? "encostao"
                        : penetrating ? "membro no mundo" : "parede");
                }
            }
        }
    }
    // (O boneco solto empurrado por quem se move, com looseContactsToPhysical: o cerebro segura com passos de verdade.)
    if (looseShoved && defaultMode && state.bodyMode != BodyMode::Physical) toPhysical("empurrado (o cerebro segura)");
    const bool animated = defaultMode
        && (state.bodyMode == BodyMode::Animated || state.bodyMode == BodyMode::Returning);
    const bool active = defaultMode && state.bodyMode == BodyMode::Active;
    // (O gesto de bola anima o corpo do padrao e tambem o da ponte de contato,
    // que segue a mesma pose: a conducao na disputa de ombro, 28/10.)
    const bool gestureBody = animated || (active && state.contactBridge);
    if (command.controlled) {
        // (No Ativo a capsula segue como no padrao: a mesma tabela de
        // velocidades; ela vai atras do corpo quando ele se afasta.)
        output.controlledAnimatedSet = true;
        output.controlledAnimated = animated || active;
        output.controlledPassWalls = defaultMode && state.bodyMode == BodyMode::Animated;
    }
    // A bola (24/10/2026, a reescrita dos gestos): o pedido com a bola no
    // alcance aplica a forca na hora, sempre (computeBallOutcome3D, como
    // impulso; independe do gesto - regra do usuario) e, no corpo animado,
    // comeca o gesto - so animacao (Engine/Gesture): a posicao da bola escolhe
    // a perna, o alcance e o movimento, e depois nao conta mais. No fisico e no
    // Ativo, so a forca. Fora do alcance o pedido se perde. Interagindo (a
    // barra carregando ou o pedido), ele e o dono da bola: as pernas dele a
    // atravessam.
    GestureSpec3D gestureStart;
    bool gestureRequested = false;
    athlete.sinceGesture += deltaTime;
    // O toque de conducao guardado comeca quando a perna dele sai do chao (o
    // passo seguinte), ou em 0,4 s. (A saida do padrao e a do tick anterior.)
    if (athlete.pendingGesture.active) {
        AthleteState3D::PendingGesture& pending = athlete.pendingGesture;
        const DefaultLocomotionOutput3D& gait = state.defaultLocomotion.output();
        const std::size_t side = pending.spec.side == 1 ? 1 : 0;
        pending.waited += deltaTime;
        const bool planned = pending.startIn >= 0.0f;
        if (!gestureBody || (planned ? pending.waited >= pending.startIn
                                     : (!gait.valid || !gait.cycling || !gait.cyclePlanted[side] || pending.waited > 0.4f))) {
            pending.active = false;
            if (gestureBody) {
                gestureStart = pending.spec;
                // (17/11, o toque realista: a bola andou enquanto a perna esperava.)
                if (gestureStart.aimBall) gestureStart.ballAt = gestureStart.ballAt + gestureStart.ballDrift * pending.waited;
                // (O contato planejado: o que falta dele agora.)
                if (planned) gestureStart.contactAt = std::max(0.0f, gestureStart.contactAt - pending.waited);
                gestureRequested = true;
            }
        }
    }
    if ((command.controlled || piloted) && !body.links.empty()) {
        // As maos: as do jogador (o input do laboratorio, consumido aqui) ou as
        // do piloto (o comando dele, que so pede no tick do soltar).
        AthleteControlInput3D pilotHands;
        if (piloted) {
            pilotHands.ballRequest = slot.command->ballRequest;
            pilotHands.ballRequestPower = slot.command->ballRequestPower;
            pilotHands.ballRequestAimYaw = slot.command->ballRequestAimYaw;
            pilotHands.ballShotHeld = slot.command->ballShotHeld;
            pilotHands.ballDribbleHeld = slot.command->ballDribbleHeld;
        }
        AthleteControlInput3D& hands = command.controlled ? control : pilotHands;
        const Vec3 pelvis = body.links.front().position;
        AthleteBall3D* nearest = nullptr;
        float best = 3.0f;
        for (AthleteBall3D& ball : balls) {
            if (!scene.contains(ball.handle) || ball.state == nullptr) continue;
            const float distance = (ball.state->position - pelvis).length();
            if (distance < best) { best = distance; nearest = &ball; }
        }
        const BallActionKind3D pressed = nearest != nullptr ? hands.ballRequest : BallActionKind3D::None;
        hands.ballRequest = BallActionKind3D::None;
        // (17/11) Um clique novo substitui o toque ainda guardado do clique anterior:
        // a ultima intencao dele vale (o dominio depois de um drible, por exemplo).
        if (pressed != BallActionKind3D::None && athlete.pendingTouch.active && !athlete.pendingTouch.automatic) {
            athlete.pendingTouch.active = false;
            athlete.pendingGesture.active = false;
        }
        if (command.controlled) output.ballRequestConsumed = true;
        // (17/11, a conducao automatica - pedido dele) Liga no drible fraco (no
        // disparo, mais abaixo); um chute, um levantar ou um drible forte a
        // desligam, e a chave do laboratorio tambem.
        AthleteState3D::AutoConduction& autoState = athlete.autoConduction;
        if (!command.controlled || !settings.autoConduction
            || pressed == BallActionKind3D::Shot || pressed == BallActionKind3D::Raise
            || (pressed == BallActionKind3D::Dribble && hands.ballRequestPower > settings.autoConductionBar)) {
            autoState.active = false;
        }
        autoState.sinceTouch += deltaTime;
        {
            const Vec3 wanted { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
            const Vec3 last = autoState.lastWanted;
            const bool sharp = std::abs(wanted.length() - last.length()) > 1.5f
                || (wanted.length() > 0.5f && last.length() > 0.5f
                    && dot(wanted, last) < std::cos(0.44f) * wanted.length() * last.length());
            autoState.sinceSharp = sharp ? 0.0f : autoState.sinceSharp + deltaTime;
            autoState.lastWanted = wanted;
            if (wanted.length() > 0.5f) {
                const float yaw = std::atan2(wanted.y, wanted.x);
                if (autoState.wantedYawValid && deltaTime > 0.0f) {
                    float change = yaw - autoState.wantedYaw;
                    while (change > 3.14159265f) change -= 6.2831853f;
                    while (change < -3.14159265f) change += 6.2831853f;
                    const float rate = std::clamp(change / deltaTime, -8.0f, 8.0f);
                    autoState.wantedYawRate += (rate - autoState.wantedYawRate) * std::min(1.0f, deltaTime / 0.06f);
                }
                autoState.wantedYaw = yaw;
                autoState.wantedYawValid = true;
            } else {
                autoState.wantedYawValid = false;
                autoState.wantedYawRate = 0.0f;
            }
        }
        const bool autoOwns = autoState.active && nearest != nullptr && nearest->handle == autoState.ball;
        if (nearest != nullptr && nearest->ownership != nullptr
            && (hands.ballShotHeld || hands.ballDribbleHeld || hands.ballRaiseHeld || pressed != BallActionKind3D::None
                || autoOwns || athlete.pendingTouch.active)) {
            nearest->ownership->interact(slot.entityId);
            scene.setInflatedBallOwner(nearest->handle, slot.handle);
        }
        // A recepcao armada: sem aperto novo, o pedido guardado tenta de novo
        // (um aperto novo o troca).
        AthleteState3D::ArmedReception& armed = athlete.armedReception;
        if (pressed != BallActionKind3D::None || nearest == nullptr) armed.active = false;
        else if (armed.active && (armed.left -= deltaTime) <= 0.0f) armed.active = false;
        const bool fromArmed = pressed == BallActionKind3D::None && armed.active;
        BallActionKind3D request = fromArmed ? (armed.shot ? BallActionKind3D::Shot
                                                   : armed.raise ? BallActionKind3D::Raise
                                                                 : BallActionKind3D::Dribble)
                                             : pressed;
        float requestPower = fromArmed ? armed.power : hands.ballRequestPower;
        const float requestLift = fromArmed ? armed.lift : hands.ballRequestLift;
        const float requestCurve = fromArmed ? armed.curve : hands.ballRequestCurve;
        const float requestAimYaw = fromArmed ? armed.aimYaw : hands.ballRequestAimYaw;
        // O corpo: o rumo da pelve do padrao (o do fisico, fora dele) e a pelve
        // no chao - o referencial da escolha (e das areas que o laboratorio
        // desenha).
        const DefaultLocomotionOutput3D& gait = state.defaultLocomotion.output();
        const float bodyYaw = (animated || active) && gait.valid ? gait.facingYawRadians
            : state.locomotion.telemetry().facingYawRadians;
        // A base das areas dos pes e a dos pes do corpo (24/10, pedido do usuario:
        // as areas presas ao corpo): o pe mais baixo menos a altura dele de pe. No
        // chao, o chao; agachado tambem (os pes ficam); no pulo, sobe com eles.
        float feetBase = groundHeight(pelvis);
        {
            float lowest = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i)
                if (profile.links[i].id == "LeftFoot" || profile.links[i].id == "RightFoot")
                    lowest = std::min(lowest, body.links[i].position.z
                        - (profile.standingRootHeightMeters + profile.links[i].modelPosition.z));
            if (lowest < std::numeric_limits<float>::max()) feetBase = lowest;
        }
        if (command.controlled) {
            output.reachValid = true;
            output.reachOrigin = { pelvis.x, pelvis.y, feetBase };
            output.reachYaw = bodyYaw;
            output.reachPelvisHeight = pelvis.z - output.reachOrigin.z;
        }
        // O alcance deste corpo (os pes x s, o peito pelo tronco, a cabeca so na posicao).
        const GestureReach3D bodyReach = gestureReachForBody3D(settings.gestureReach, bodyScale3D(assets.body),
            bodyGirth3D(PhysiqueRegion3D::Chest, bodyTone3D(assets.body)) - 1.0f);
        // A conducao do chute (Ball/ShotConduction3D): a bola chutada ainda no
        // alcance dos pes gira com o olhar. Um aperto novo, a bola fora do
        // alcance ou sumida, ou a chave desligada a encerram.
        if (athlete.conduction.active) {
            const AthleteBall3D* kicked = nullptr;
            for (const AthleteBall3D& ball : balls)
                if (ball.handle == athlete.conduction.ball && scene.contains(ball.handle) && ball.state != nullptr)
                    kicked = &ball;
            bool inReach = false;
            if (kicked != nullptr) {
                const Vec3 forward { std::cos(bodyYaw), std::sin(bodyYaw), 0.0f };
                const Vec3 left { -forward.y, forward.x, 0.0f };
                const Vec3 at = kicked->state->position;
                const Vec3 offset { at.x - pelvis.x, at.y - pelvis.y, 0.0f };
                inReach = inGestureReach3D({ dot(offset, forward), dot(offset, left), at.z - feetBase }, bodyReach);
            }
            if (!inReach || pressed != BallActionKind3D::None || !settings.ballForce.conduction) {
                athlete.conduction.active = false;
            } else {
                const InflatedBallProfile3D ballProfile = scene.inflatedBallProfile(kicked->handle);
                const float mass = ballProfile.massKg;
                const float inertia = (2.0f / 3.0f) * mass * ballProfile.radiusMeters * ballProfile.radiusMeters;
                const ShotConductionStep3D next = stepShotConduction3D(athlete.conduction.shot,
                    kicked->state->linearVelocity, kicked->state->angularVelocity, command.facingYawRadians,
                    settings.ballForce);
                scene.applyInflatedBallImpulses(kicked->handle, (next.velocity - kicked->state->linearVelocity) * mass,
                    (next.spin - kicked->state->angularVelocity) * inertia);
            }
        }
        // A velocidade do corpo agora (no padrao, a da capsula; no fisico, a da pelve).
        const Vec3 bodyVelocityNow = animated || active ? Vec3 { nav().velocity.x, nav().velocity.y, 0.0f }
            : Vec3 { body.links.front().linearVelocity.x, body.links.front().linearVelocity.y, 0.0f };
        // A forca de um toque na bola (o resultado do pedido como impulso).
        // (ownVelocity: a do corpo que a forca considera - no encontro com o ajuste
        // curto, a das teclas: a bola nao sai com a rapidez do ajuste e foge dele.)
        const auto applyBallOutcome = [&](const AthleteBall3D& target, const BallOutcomeRequest3D& outcomeRequest,
                                          Vec3 aheadVelocity, const Vec3* ownVelocity = nullptr) {
            const InflatedBallProfile3D ballProfile = scene.inflatedBallProfile(target.handle);
            const BallOutcome3D outcome = computeBallOutcome3D(outcomeRequest, target.state->linearVelocity,
                ownVelocity != nullptr ? *ownVelocity : bodyVelocityNow, aheadVelocity, ballProfile.radiusMeters,
                settings.ballForce);
            const float mass = ballProfile.massKg;
            const float inertia = (2.0f / 3.0f) * mass * ballProfile.radiusMeters * ballProfile.radiusMeters;
            scene.applyInflatedBallImpulses(target.handle, (outcome.velocity - target.state->linearVelocity) * mass,
                (outcome.spin - target.state->angularVelocity) * inertia);
            if (command.controlled) output.ballTouched = true;
            if (std::getenv("XQT") != nullptr)
                std::printf("toque: bola %.2f -> %.2f m/s, corpo %.2f m/s, barra %.2f, velocidade pedida %.2f\n",
                    target.state->linearVelocity.length(), outcome.velocity.length(), bodyVelocityNow.length(),
                    outcomeRequest.power, outcomeRequest.touchSpeed);
        };
        // (17/11, o toque realista) A distancia da bola a chuteira de uma
        // perna: o colisor exato do pe (a casca convexa) contra o centro dela.
        const auto footBallGap = [&](int side, Vec3 ballCenter) {
            const char* id = side == 1 ? "RightFoot" : "LeftFoot";
            for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
                if (profile.links[i].id != id) continue;
                const RagdollCapsuleDefinition3D& collider = profile.links[i].collider;
                const Vec3 center = body.links[i].position + body.links[i].orientation.rotate(collider.localPosition);
                const Quaternion orientation = (body.links[i].orientation * collider.localOrientation).normalized();
                return ragdollColliderClosestPoint3D(collider, orientation.conjugate().rotate(ballCenter - center)).distance;
            }
            return 1.0e9f;
        };
        // (17/11, a conducao automatica - pedido dele: "sempre tentando deixa-la
        // logo a frente dos meus pes... apenas atraves do manejo REAL... o MINIMO
        // possivel de acoes") Os toques sao os do drible de verdade (o gesto, o
        // encosto, a mesma forca da barra), pedidos sozinhos so quando a bola vai
        // sair da faixa a frente dos pes. Cada toque a manda ate a ponta de longe
        // da faixa e ela volta aos pes pela desaceleracao dela: o maior intervalo
        // ate o proximo. Nunca puxa nem freia o corpo; a bola longe a frente so
        // espera ele chegar.
        BallActionKind3D autoRequest = BallActionKind3D::None;
        float autoPower = 0.0f;
        float autoSpeed = 0.0f;
        Vec3 autoDirection;
        bool autoReceive = false;
        // O planejador de um toque: so quando a bola vai sair da faixa (ou
        // sempre, com force: o drible que liga a conducao). valid sem toque.
        struct AutoTouchPlan {
            bool valid = false;
            bool receive = false;
            float power = 0.0f;
            float speed = 0.0f;
            Vec3 direction;
        };
        const auto planAutoTouch = [&](const AthleteBall3D& ball, bool force, bool atContact = false) {
            // (17/11 14h20, v3 - ele: "muito sofisticado, com predicts... dominio
            // perfeito com o minimo de movimentos"; "testar movimentos bruscos,
            // viradas fortes, paradas subitas") A bola fica logo a frente dos pes
            // (na frente do corpo) e sempre ao alcance; o corpo e previsto pela
            // resposta medida da locomocao (predictBodyMotion3D: o rumo gira a 10
            // rad/s, a rapidez vai a pedida em 0,25 s). Toca so quando o caminho
            // previsto da bola sai da faixa; logo depois de um toque, com as teclas
            // firmes, a faixa folga (sem toque de sobra).
            AutoTouchPlan plan;
            const Vec3 ballPosition = ball.state->position;
            const Vec3 ballVelocity { ball.state->linearVelocity.x, ball.state->linearVelocity.y, 0.0f };
            const Vec3 offset { ballPosition.x - pelvis.x, ballPosition.y - pelvis.y, 0.0f };
            const Vec3 forward { std::cos(bodyYaw), std::sin(bodyYaw), 0.0f };
            const Vec3 left { -forward.y, forward.x, 0.0f };
            const Vec3 inBody { dot(offset, forward), dot(offset, left), ballPosition.z - feetBase };
            // O alcance de verdade do pe (nao a caixa larga da escolha do gesto).
            const bool reachable = inBody.x > -0.30f && inBody.x < 0.64f && std::abs(inBody.y) < 0.48f
                && inBody.z <= bodyReach.highBall;
            // (No encosto o pe ja esta nela: o alcance nao conta.)
            if (!reachable && !atContact) return plan;
            // O que as teclas pedem; na virada em andamento (o mouse girando), o rumo
            // de 0,12 s adiante (o giro de agora, ate 0,5 rad).
            const float turning = std::clamp(autoState.wantedYawRate * 0.12f, -0.5f, 0.5f);
            const Vec3 wantedNow { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
            const Vec3 wanted = wantedNow.length() > 0.5f
                ? Vec3 { std::cos(autoState.wantedYaw + turning), std::sin(autoState.wantedYaw + turning), 0.0f } * wantedNow.length()
                : wantedNow;
            const float pace = std::max(bodyVelocityNow.length(), wanted.length());
            const float facingYaw = command.facingYawRadians + turning;
            const Vec3 facing { std::cos(facingYaw), std::sin(facingYaw), 0.0f };
            const Vec3 side { -facing.y, facing.x, 0.0f };
            const float nearEdge = 0.22f, farEdge = std::min(0.36f + 0.01f * pace, 0.42f);
            const float middle = 0.5f * (nearEdge + farEdge), sideEdge = 0.14f;
            const float deceleration = std::max(autoState.deceleration, 0.05f);
            const float ballSpeed = ballVelocity.length();
            const Vec3 rolling = ballSpeed > 0.05f ? ballVelocity * (1.0f / ballSpeed) : Vec3 {};
            const auto body = [&](float t) {
                return predictBodyMotion3D(bodyVelocityNow, wanted, 0.25f, 10.0f, t);
            };
            const auto ballAt = [&](float t) {
                const float slowed = std::min(t, ballSpeed / deceleration);
                return offset + ballVelocity * slowed - rolling * (0.5f * deceleration * slowed * slowed)
                    + ballVelocity * 0.0f;
            };
            // As teclas mudaram desde o ultimo toque (rumo 20 graus, rapidez 1 m/s)?
            const Vec3 then = autoState.plannedWanted;
            const bool changed = std::abs(wantedNow.length() - then.length()) > 1.0f
                || (wantedNow.length() > 0.5f && then.length() > 0.5f
                    && dot(wantedNow, then) < std::cos(0.35f) * wantedNow.length() * then.length());
            const float slack = !changed && autoState.sinceTouch < 0.45f ? 0.07f : 0.0f;
            const bool standing = wantedNow.length() < 0.3f && bodyVelocityNow.length() < 1.2f;
            if (standing) {
                const float along = dot(offset, facing), lateral = dot(offset, side);
                const Vec3 relative = ballVelocity - bodyVelocityNow;
                if (!force && relative.length() > 0.35f) {
                    // Parado: a bola rolando para longe leva o toque que a mata.
                    plan.valid = true;
                    plan.receive = true;
                } else if (force || along < 0.12f || along > 0.60f || std::abs(lateral) > 0.30f) {
                    // Girando parado (a bola saiu da frente dele): um toque leve a
                    // poe na frente dos pes.
                    constexpr float Settle = 0.7f;
                    const Vec3 meet = facing * middle;
                    Vec3 needed = (meet - offset) * (1.0f / Settle);
                    const float speed = needed.length();
                    if (speed > 0.05f) {
                        needed = needed * ((speed + 0.5f * deceleration * Settle) / speed);
                        plan.valid = true;
                        plan.direction = needed * (1.0f / needed.length());
                        plan.speed = needed.length();
                    }
                }
                return plan;
            }
            // O caminho dela em relacao a ele nos proximos 0,45 s.
            bool needed = force;
            for (float t = 0.0f; t <= 0.45f && !needed; t += 0.05f) {
                const Vec3 at = ballAt(t) - body(t).travel;
                const float along = dot(at, facing);
                needed = along < nearEdge - slack || along > farEdge + 0.08f + slack || std::abs(dot(at, side)) > sideEdge + slack;
            }
            if (!needed) return plan;
            const float along = dot(offset, facing), lateral = dot(offset, side);
            if (wantedNow.length() < 0.3f) {
                // Parando de repente: ele ainda desliza ate parar (do sprint, ~1 m).
                // O toque faz a bola rolar e parar logo a frente dos pes de onde ele
                // vai parar; ja passada desse ponto, ela morre ali.
                const Vec3 stop = predictBodyMotion3D(bodyVelocityNow, {}, 0.25f, 10.0f, 1.2f).travel + facing * middle;
                const Vec3 gap = stop - offset;
                if (dot(gap, facing) <= 0.0f || gap.length() < 0.05f) {
                    plan.valid = true;
                    plan.receive = true;
                    return plan;
                }
                plan.valid = true;
                plan.direction = gap * (1.0f / gap.length());
                plan.speed = std::sqrt(2.0f * deceleration * gap.length());
                return plan;
            }
            // O toque de encontro: a bola ate a frente dos pes de onde ele vai estar
            // (freando no caminho). Firme e no rumo, o encontro e mais longe - ela vai
            // ate a ponta e volta (o maior intervalo ate o proximo toque). Freando, o
            // encontro e quando ele ja desacelerou (0,7 s): antes, a bola ia na
            // rapidez da freada e fugia na frente dele.
            const bool steady = !changed && std::abs(autoState.wantedYawRate) < 1.0f && std::abs(lateral) < sideEdge
                && along > nearEdge - 0.05f;
            const bool braking = wantedNow.length() < bodyVelocityNow.length() - 1.0f;
            const float span = std::max(0.05f, farEdge - along);
            const float meetTime = braking ? 0.7f
                : steady ? std::clamp(std::sqrt(2.0f * span / deceleration), 0.4f, 1.2f) : 0.4f;
            const PredictedBody3D ahead = body(meetTime);
            const Vec3 meet = ahead.travel + facing * (steady ? farEdge : middle);
            Vec3 target = (meet - offset) * (1.0f / meetTime);
            const float targetSpeed = target.length();
            if (targetSpeed > 1e-3f) target = target * ((targetSpeed + 0.5f * deceleration * meetTime) / targetSpeed);
            const float speed = target.length();
            if (speed < 0.05f) {
                plan.valid = true;
                plan.receive = true;
                return plan;
            }
            plan.direction = target * (1.0f / speed);
            plan.speed = speed;
            // A barra do drible que da essa velocidade (o inverso de BallOutcome3D),
            // para o drible que liga a conducao sem o toque realista.
            const BallOutcomeSettings3D& f = settings.ballForce;
            const float carry = std::max(0.0f, std::max(dot(bodyVelocityNow, plan.direction),
                dot(wanted, plan.direction))) * std::max(0.0f, f.carry);
            const float push = std::max(0.0f, f.speedPush) * pace;
            const float strength = speed - carry - push - f.touchMinimum;
            plan.power = strength <= 0.0f ? 0.0f
                : std::pow(strength / std::max(f.touchPower, 1e-3f), 1.0f / std::max(0.05f, f.touchCurve));
            plan.power = std::clamp(plan.power, 0.0f, settings.autoConductionBar);
            plan.valid = true;
            return plan;
        };
        // (17/11) A consulta do planejador de contato (BallContactPlanner3D): a
        // bola, o corpo indo para o que as teclas pedem e a passada de cada perna.
        const auto contactQuery = [&](const AthleteBall3D& ball) {
            BallContactQuery3D q;
            q.ballPosition = ball.state->position;
            q.ballVelocity = ball.state->linearVelocity;
            q.ballRadius = scene.inflatedBallProfile(ball.handle).radiusMeters;
            q.rollingDeceleration = autoState.deceleration;
            q.pelvis = pelvis;
            q.feetBase = feetBase;
            q.yaw = bodyYaw;
            q.targetYaw = command.facingYawRadians;
            q.yawRate = 10.0f;
            q.bodyVelocity = bodyVelocityNow;
            q.wantedVelocity = { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
            const Vec3 forward { std::cos(bodyYaw), std::sin(bodyYaw), 0.0f };
            const Vec3 left { -forward.y, forward.x, 0.0f };
            for (std::size_t side = 0; side < 2; ++side) {
                BallContactLeg3D& leg = q.legs[side];
                leg.cycling = gait.valid && gait.cycling && gait.cycleSeconds > 0.0f;
                leg.forecast = gait.footForecast[side];
                leg.forecastStep = gait.forecastStep;
                leg.plantedNow = gait.cyclePlanted[side];
                leg.secondsToLiftoff = gait.secondsToLiftoff[side];
                leg.secondsToTouchdown = gait.cycleSecondsToTouchdown[side];
                leg.swingSeconds = gait.swingSeconds[side];
                leg.cycleSeconds = gait.cycleSeconds;
                for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i)
                    if (profile.links[i].id == (side == 0 ? "LeftFoot" : "RightFoot")) {
                        const Vec3 offset = body.links[i].position - pelvis;
                        leg.footNow = { dot(offset, forward), dot(offset, left), 0.0f };
                    }
            }
            return q;
        };
        // (17/11) Uma virada (mais de 30 graus do rumo pedido quando ele foi
        // planejado) cancela o toque automatico guardado: o proximo e planejado ja
        // para o rumo novo.
        if (autoState.active && ((athlete.pendingTouch.active && athlete.pendingTouch.automatic)
                || athlete.pendingGesture.active)) {
            const Vec3 now { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
            const Vec3 then = autoState.plannedWanted;
            const bool turned = now.length() > 0.5f && then.length() > 0.5f
                && dot(now, then) < std::cos(0.52f) * now.length() * then.length();
            const bool started = now.length() > 0.5f && then.length() <= 0.5f;
            if (turned || started) {
                athlete.pendingTouch.active = false;
                athlete.pendingGesture.active = false;
                autoState.sinceTouch = 1.0f;
            }
        }
        // O encontro guiado (a "puxada" suave): enquanto o toque planejado com o corpo
        // indo ate a bola espera, a velocidade que leva a pelve para atras da bola
        // prevista no instante do contato; o laboratorio a usa no lugar das teclas.
        // (17/11 ~15h, ele: "era coisa imediata e rapida.. nunca travar o jogador ou
        // ser algo demorado.. algo sutil") So ate o contato (no maximo ~0,36 s) e no
        // maximo 1 m/s em volta da velocidade dele: a das teclas de agora (elas
        // mandam) ou, no dominio, a do aperto.
        if (athlete.pendingTouch.active && athlete.pendingTouch.approach && command.controlled
            && athlete.pendingTouch.elapsed < athlete.pendingTouch.plannedContact) {
            const AthleteState3D::PendingTouch& touch = athlete.pendingTouch;
            const float remaining = std::max(0.06f, touch.plannedContact - touch.elapsed);
            Vec3 toBall { touch.plannedBall.x - pelvis.x, touch.plannedBall.y - pelvis.y, 0.0f };
            const float distance = toBall.length();
            if (distance > 1e-3f) {
                const Vec3 spot = Vec3 { touch.plannedBall.x, touch.plannedBall.y, 0.0f }
                    - toBall * (std::min(0.32f, distance) / distance);
                const Vec3 base = touch.trap ? touch.approachBase
                    : Vec3 { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
                Vec3 nudge = (spot - Vec3 { pelvis.x, pelvis.y, 0.0f }) * (1.0f / remaining) - base;
                if (nudge.length() > 1.0f) nudge = nudge * (1.0f / nudge.length());
                output.approachValid = true;
                output.approachVelocity = base + nudge;
            }
        }
        // (17/11, ele: "nao setar o movimento unico do ato pe-bola, mas ir ajustando
        // isso em tempo real conforme o movimento... se eu mudei de direcao no meio e
        // o pe nao iria coincidir... atualizar em tempo real a acao do pe... para
        // garantir o contato") A cada tick ate o contato o encontro e replanejado com
        // o estado de agora (as teclas novas, a bola, o corpo), na mesma perna. Mudou
        // (o ponto 5 cm ou o instante 30 ms): o gesto e corrigido no meio - recomeca de
        // onde a perna esta, sem armar de novo depois da armada, para o ponto novo no
        // instante novo - e a forca sai no contato novo.
        if (athlete.pendingTouch.active && !athlete.pendingTouch.automatic && athlete.pendingTouch.plannedContact >= 0.0f
            && gestureBody && settings.realisticTouch) {
            AthleteState3D::PendingTouch& touch = athlete.pendingTouch;
            touch.sinceRetarget += deltaTime;
            const float remaining = touch.plannedContact - touch.elapsed;
            const AthleteBall3D* aimed = nullptr;
            for (const AthleteBall3D& ball : balls)
                if (ball.handle == touch.ball && scene.contains(ball.handle) && ball.state != nullptr) aimed = &ball;
            if (aimed != nullptr && remaining > 0.08f && touch.sinceRetarget >= 0.06f) {
                BallContactQuery3D q = contactQuery(*aimed);
                q.onlySide = touch.side;
                q.naturalContact = remaining;
                q.maxDeviation = 0.9f;
                q.highestBall = 1.0f;
                // Primeiro no mesmo instante (so o ponto muda: o pe corrige o rumo
                // sem apressar a perna - o instante mais cedo comprimia o gesto e a
                // perna girava a 30 rad/s); sem alcance nele, a janela em volta.
                q.earliest = q.latest = remaining;
                BallContactPlan3D plan = planBallContact3D(q);
                if (!plan.valid) {
                    q.earliest = std::max(0.03f, remaining - 0.15f);
                    q.latest = remaining + 0.25f;
                    plan = planBallContact3D(q);
                }
                if (plan.valid && ((plan.ballWorld - touch.plannedBall).length() > 0.04f
                        || std::abs(plan.seconds - remaining) > 0.03f)) {
                    touch.plannedContact = touch.elapsed + plan.seconds;
                    touch.plannedBall = plan.ballWorld;
                    touch.sinceRetarget = 0.0f;
                    GestureSpec3D retarget = touch.gesture;
                    retarget.aimBall = true;
                    retarget.ballAt = plan.ballInBody;
                    retarget.ballDrift = {};
                    retarget.contactAt = plan.seconds;
                    retarget.skipWindup = touch.elapsed >= touch.windupEnd;
                    retarget.retarget = true;
                    if (athlete.pendingGesture.active) {
                        // (O gesto ainda nao comecou: a espera e o contato novos.)
                        athlete.pendingGesture.spec = retarget;
                        athlete.pendingGesture.spec.retarget = false;
                        athlete.pendingGesture.spec.contactAt = plan.seconds + athlete.pendingGesture.waited;
                        athlete.pendingGesture.startIn = athlete.pendingGesture.waited + std::max(0.0f, plan.seconds - 0.12f);
                    } else {
                        gestureStart = retarget;
                        gestureRequested = true;
                    }
                    touch.gesture = retarget;
                    if (std::getenv("XQT") != nullptr)
                        std::printf("ajuste: perna %d contato em %.2f s, bola %+.2f %+.2f\n", touch.side, plan.seconds,
                            plan.ballInBody.x, plan.ballInBody.y);
                }
            }
        }
        if (athlete.pendingTouch.active) {
            AthleteState3D::PendingTouch& touch = athlete.pendingTouch;
            const AthleteBall3D* touched = nullptr;
            for (const AthleteBall3D& ball : balls)
                if (ball.handle == touch.ball && scene.contains(ball.handle) && ball.state != nullptr) touched = &ball;
            touch.left -= deltaTime;
            touch.elapsed += deltaTime;
            bool assisted = false;
            if (touched != nullptr) {
                const float gap = footBallGap(touch.side, touched->state->position)
                    - scene.inflatedBallProfile(touched->handle).radiusMeters;
                touch.closest = std::min(touch.closest, gap);
                // (17/11, o contato por plano) No instante planejado o toque sai, com
                // a bola onde se previu (ate 35 cm: ninguem a desviou no caminho) - o
                // gesto levou o pe ate ela. Antes disso, so o encosto de verdade.
                // (E o pe pode estar nela: a bola ao alcance da perna dele agora - ele
                // pode ter parado ou virado desde o plano.)
                const Vec3 fromPelvis { touched->state->position.x - pelvis.x, touched->state->position.y - pelvis.y, 0.0f };
                assisted = touch.plannedContact >= 0.0f && touch.elapsed >= touch.plannedContact
                    && (touched->state->position - touch.plannedBall).length() <= 0.35f && fromPelvis.length() <= 0.85f;
                touch.previousGap = gap;
            }
            if ((touched == nullptr || touch.left <= 0.0f) && std::getenv("XQT") != nullptr)
                std::printf("toque realista: sem encosto (perna %d, mais perto %.3f m)\n", touch.side, touch.closest);
            if (touched == nullptr || touch.left <= 0.0f) {
                touch.active = false;
            } else if (assisted || (touch.automatic ? std::min(footBallGap(0, touched->state->position), footBallGap(1, touched->state->position))
                                        : footBallGap(touch.side, touched->state->position))
                <= scene.inflatedBallProfile(touched->handle).radiusMeters
                    + (!touch.automatic ? 0.15f
                        : std::abs(autoState.wantedYawRate) > 2.0f || bodyVelocityNow.length() > 6.0f
                            || autoState.sinceSharp < 0.6f ? 0.20f : 0.14f)) {
                touch.active = false;
                // (O gesto guardado deste toque ja nao serve: o pe ja encostou.)
                athlete.pendingGesture.active = false;
                // O toque automatico replaneja no encosto: o rumo e a velocidade
                // de saida que deixam a bola na faixa a frente dos pes agora.
                if (touch.automatic) {
                    const AutoTouchPlan plan = planAutoTouch(*touched, true, true);
                    if (plan.valid && !plan.receive) {
                        touch.request.receive = false;
                        touch.request.direction = plan.direction;
                        touch.request.touchSpeed = plan.speed;
                    } else if (plan.valid && plan.receive) {
                        // Parando: a bola morre na frente dos pes (a velocidade dele).
                        touch.request.receive = false;
                        const Vec3 wanted { command.intent.requestedVelocityWorld.x,
                            command.intent.requestedVelocityWorld.y, 0.0f };
                        touch.request.direction = wanted.length() > 0.05f ? wanted * (1.0f / wanted.length())
                            : Vec3 { std::cos(command.facingYawRadians), std::sin(command.facingYawRadians), 0.0f };
                        touch.request.touchSpeed = wanted.length();
                    }
                }
                // (O dominio: a bola rola e para logo a frente de onde ele vai parar - ele
                // vinha correndo e ainda desliza; parada na hora, ele passava por ela.)
                if (touch.trap && !touch.automatic) {
                    // (O pedido e parar - as teclas soltas; a velocidade de agora pode ser a
                    // da puxada ate a bola, nao a dele.)
                    const Vec3 wantedNow {};
                    const Vec3 slide = predictBodyMotion3D(bodyVelocityNow, wantedNow, 0.25f, 10.0f, 1.5f).travel;
                    const Vec3 facingNow { std::cos(command.facingYawRadians), std::sin(command.facingYawRadians), 0.0f };
                    const Vec3 rest = Vec3 { pelvis.x + slide.x, pelvis.y + slide.y, 0.0f } + facingNow * 0.32f;
                    const Vec3 gap { rest.x - touched->state->position.x, rest.y - touched->state->position.y, 0.0f };
                    touch.request.receive = false;
                    if (gap.length() > 0.05f && wantedNow.length() < 0.3f) {
                        touch.request.direction = gap * (1.0f / gap.length());
                        touch.request.touchSpeed = std::sqrt(2.0f * std::max(autoState.deceleration, 0.05f) * gap.length());
                    } else {
                        // (Com tecla, ou ja no ponto: a bola na velocidade dele.)
                        const float speed = bodyVelocityNow.length();
                        touch.request.direction = speed > 0.05f ? bodyVelocityNow * (1.0f / speed) : facingNow;
                        touch.request.touchSpeed = wantedNow.length() < 0.3f ? 0.0f : speed;
                    }
                }
                const Vec3 keysVelocity { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
                applyBallOutcome(*touched, touch.request, touch.ahead, touch.approach && !touch.trap ? &keysVelocity : nullptr);
                if (touch.shotConduction && settings.ballForce.conduction) {
                    athlete.conduction.active = true;
                    athlete.conduction.ball = touch.ball;
                    athlete.conduction.shot = startShotConduction3D(touch.shotYaw, touch.shotPower, settings.ballForce);
                }
                if (autoState.active && touch.ball == autoState.ball) autoState.sinceTouch = 0.0f;
            }
        }
        if (autoState.active) {
            const AthleteBall3D* ball = nullptr;
            for (const AthleteBall3D& candidate : balls)
                if (candidate.handle == autoState.ball && scene.contains(candidate.handle) && candidate.state != nullptr)
                    ball = &candidate;
            if (ball == nullptr) {
                autoState.active = false;
            } else {
                const Vec3 ballPosition = ball->state->position;
                const Vec3 ballVelocity { ball->state->linearVelocity.x, ball->state->linearVelocity.y, 0.0f };
                const Vec3 offset { ballPosition.x - pelvis.x, ballPosition.y - pelvis.y, 0.0f };
                const float distance = offset.length();
                const float radius = scene.inflatedBallProfile(ball->handle).radiusMeters;
                // A desaceleracao dela rolando solta (sem toque ha 0,3 s, no chao).
                const float ballSpeed = ballVelocity.length();
                if (autoState.sinceTouch > 0.3f && ballPosition.z < feetBase + radius + 0.04f && ballSpeed > 0.3f
                    && autoState.lastSpeed > 0.0f) {
                    const float observed = (autoState.lastSpeed - ballSpeed) / std::max(deltaTime, 1e-4f);
                    if (observed > 0.0f && observed < 5.0f)
                        autoState.deceleration = std::clamp(0.97f * autoState.deceleration + 0.03f * observed, 0.15f, 2.5f);
                }
                autoState.lastSpeed = ballSpeed;
                // (A bola que escapou numa virada seca: ele pode buscar - a conducao
                // so desliga com ela a mais de 3 m por 2 s.)
                autoState.away = distance > 3.0f ? autoState.away + deltaTime : 0.0f;
                if (autoState.away > 2.0f) autoState.active = false;
                if (autoState.active && distance < 1.6f && command.controlled) {
                    output.autoConductionGlow = true;
                    output.autoConductionBall = ball->handle;
                }
                if (autoState.active && request == BallActionKind3D::None && !athlete.pendingTouch.active
                    && !athlete.pendingGesture.active
                    && (autoState.sinceTouch > 0.12f || (autoState.sinceSharp < 0.05f && autoState.sinceTouch > 0.03f))) {
                    const AutoTouchPlan plan = planAutoTouch(*ball, false);
                    if (plan.valid) {
                        autoRequest = BallActionKind3D::Dribble;
                        autoPower = plan.power;
                        autoSpeed = plan.speed;
                        autoDirection = plan.direction;
                        autoReceive = plan.receive;
                    }
                }
            }
        }
        // O drible fraco que liga a conducao ja sai planejado: no rumo da
        // corrida, a bola ate a faixa a frente dos pes (andando; parado, o
        // toque dele como ele pediu).
        bool startsConduction = false;
        if (request == BallActionKind3D::Dribble && !fromArmed && pressed == BallActionKind3D::Dribble
            && command.controlled && settings.autoConduction && requestPower <= settings.autoConductionBar
            && nearest != nullptr) {
            startsConduction = true;
            const AutoTouchPlan plan = planAutoTouch(*nearest, true);
            if (plan.valid && !plan.receive) {
                autoDirection = plan.direction;
                autoSpeed = plan.speed;
                requestPower = plan.power;
            }
        }
        const bool automatic = request == BallActionKind3D::None && autoRequest != BallActionKind3D::None
            && nearest != nullptr && nearest->handle == autoState.ball;
        if (automatic) {
            request = autoRequest;
            requestPower = autoPower;
        }
        if (request != BallActionKind3D::None) {
            const Vec3 forward { std::cos(bodyYaw), std::sin(bodyYaw), 0.0f };
            const Vec3 left { -forward.y, forward.x, 0.0f };
            const Vec3 ballPosition = nearest->state->position;
            const Vec3 offset { ballPosition.x - pelvis.x, ballPosition.y - pelvis.y, 0.0f };
            const Vec3 ballInBody { dot(offset, forward), dot(offset, left), ballPosition.z - feetBase };
            const float look = command.facingYawRadians;
            const Vec3 keys = command.intent.requestedDirectionWorld;
            // (17/11 14h20, ele: "praticamente impossivel eu conduzir a bola para frente
            // e tentar frear ela subitamente") O drible sem tecla de movimento e o
            // dominio: a bola morre nos pes dele (o gesto de amortecer). Antes, o
            // toque sem tecla ia no rumo do olhar e a bola seguia.
            const bool stopTouch = request == BallActionKind3D::Dribble && !automatic && !startsConduction
                && keys.lengthSquared() < 1e-4f;
            const Vec3 touchDirection = ((automatic && !autoReceive) || (startsConduction && autoDirection.lengthSquared() > 0.5f))
                ? autoDirection
                : keys.lengthSquared() > 1e-4f ? keys : Vec3 { std::cos(look), std::sin(look), 0.0f };
            const float aimYaw = request == BallActionKind3D::Shot ? requestAimYaw : look;
            const Vec3 aimed = request == BallActionKind3D::Shot ? Vec3 { std::cos(aimYaw), std::sin(aimYaw), 0.0f }
                                                                : touchDirection;
            // (A bola no meio vai a perna que esta no ar na passada.)
            int freeSide = -1;
            if ((animated || active) && gait.valid && gait.cycling && gait.cyclePlanted[0] != gait.cyclePlanted[1])
                freeSide = gait.cyclePlanted[0] ? 1 : 0;
            const bool raising = request == BallActionKind3D::Raise;
            const GestureKind3D kind = request == BallActionKind3D::Shot ? GestureKind3D::Shot
                : raising ? GestureKind3D::Raise : GestureKind3D::Touch;
            // O toque ou o chute fraco: o de conduzir. (Levantar nunca e recepcao
            // nem toque de conducao.)
            const bool gentle = !raising && (kind == GestureKind3D::Touch || requestPower < 0.3f);
            // A velocidade do corpo: no padrao, a da capsula (a da pelve tem o
            // balanco da passada); no fisico, a da pelve.
            const Vec3 capsule = nav().velocity;
            const Vec3 bodyVelocity = animated || active ? Vec3 { capsule.x, capsule.y, 0.0f }
                : Vec3 { body.links.front().linearVelocity.x, body.links.front().linearVelocity.y, 0.0f };
            // A recepcao (24/10, pedido do usuario): a bola vindo para ele - mais
            // rapida que receiveClosing no rumo da pelve no mundo e em relacao a
            // ele (so no mundo, a bola que corre junto atras dele contava), ou a
            // alta caindo nele (o balao) - e o toque ou o chute fraco: a forca a
            // para junto do corpo e o gesto amortece.
            const bool high = ballInBody.z > bodyReach.highBall;
            const Vec3 ballVelocity = nearest->state->linearVelocity;
            const Vec3 flatBall { ballVelocity.x, ballVelocity.y, 0.0f };
            const Vec3 toward = offset.lengthSquared() > 1e-6f ? offset * (-1.0f / offset.length()) : forward * -1.0f;
            const float closing = std::max(std::min(dot(flatBall, toward), dot(flatBall - bodyVelocity, toward)),
                high ? -ballVelocity.z : 0.0f);
            // O chute na bola vindo so e recepcao abaixo de receiveShotBar (28/10,
            // pedido do usuario: o chute fraco de primeira sai como chute); o
            // pedido armado e o que era no aperto.
            const bool coming = closing > settings.ballForce.receiveClosing;
            const bool receive = fromArmed ? armed.receive
                : stopTouch || (automatic && autoReceive)
                    || (coming && !raising
                        && (kind == GestureKind3D::Touch || requestPower < settings.ballForce.receiveShotBar));
            // Conduzindo (o toque ou o chute fraco ate 2,5 s depois de outro),
            // as pernas se alternam (24/10, pedido do usuario). (Na recepcao, na
            // bola vindo, no pedido armado e na bola alta, nao.)
            const bool conducting = gentle && !coming && !fromArmed;
            // A perna do ultimo toque ainda no gesto, com a bola do lado dela,
            // continua (28/10, pedido do usuario: "aproveita a posicao da perna
            // para continuar o movimento"); nao alterna.
            const int lastSide = athlete.lastGestureSide;
            const bool continuing = lastSide >= 0 && (animated || active) && gait.valid
                && gait.gestureLegs[static_cast<std::size_t>(lastSide)]
                && (lastSide == 0 ? ballInBody.y > -bodyReach.centerBand : ballInBody.y < bodyReach.centerBand);
            // (17/11: no toque automatico, a perna e a do ar na passada ou a do lado
            // da bola - na curva, a alternada forcada ia na perna do outro lado.)
            const bool sequence = athlete.sinceGesture < 2.5f && lastSide >= 0 && conducting && !high && !continuing
                && !automatic;
            // (Na recepcao parado, a bola perto do meio vai ao pe da frente: o de
            // tras corria 75 cm ate ela.)
            int preferredSide = continuing ? lastSide : freeSide;
            // O pe mais a frente (front) ou mais atras (rear) agora.
            const auto footAt = [&](bool front) {
                int side = -1;
                float best = front ? -1e9f : 1e9f;
                for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
                    const int foot = profile.links[i].id == "LeftFoot" ? 0 : profile.links[i].id == "RightFoot" ? 1 : -1;
                    const float along = dot(body.links[i].position - pelvis, forward);
                    if (foot >= 0 && (front ? along > best : along < best)) {
                        best = along;
                        side = foot;
                    }
                }
                return side;
            };
            if (receive && preferredSide < 0) preferredSide = footAt(true);
            // O chute com a bola a frente vai com a perna de tras (28/10, pedido
            // do usuario: "a perna mais recuada... daria um movimento mais
            // bonito"): parado, a do pe de tras; andando, a do ar enquanto ela
            // ainda vem de tras (a primeira metade do balanco) e, com ela perto de
            // pousar a frente, a apoiada - a do ar pousa e apoia o chute.
            if (kind == GestureKind3D::Shot && !receive && ballInBody.x > bodyReach.shotAhead) {
                if (freeSide >= 0) {
                    const float swing = gait.swingSeconds[static_cast<std::size_t>(freeSide)];
                    const float landing = gait.cycleSecondsToTouchdown[static_cast<std::size_t>(freeSide)];
                    preferredSide = swing > 0.0f && landing < 0.5f * swing ? 1 - freeSide : freeSide;
                } else {
                    preferredSide = footAt(false);
                }
            }
            // (17/11) No toque automatico, a perna do ar pega a bola numa faixa
            // larga do meio (22 cm): a plantada esperava sair do chao e a bola
            // fugia na curva.
            GestureReach3D choiceReach = bodyReach;
            if (automatic || startsConduction) choiceReach.centerBand = std::max(choiceReach.centerBand, 0.22f);
            // (17/11, o planejador de contato - pedido dele: "o pe ir para onde a
            // bola VAI ESTAR no momento do contato... escolher a perna com base
            // nisso") Com o toque realista e o gesto do pe: a perna e o instante
            // em que a passada leva o pe ate onde a bola vai estar; a escolha do
            // gesto e o pe vao para essa bola.
            BallContactPlan3D contactPlan;
            bool approachPlanned = false;
            Vec3 approachBase;
            if (settings.realisticTouch && gestureBody) {
                BallContactQuery3D query = contactQuery(*nearest);
                const float power = std::clamp(requestPower, 0.0f, 1.0f);
                // (O instante natural do contato e o mais cedo possivel: o chute com a
                // armada inteira ou comprimida ate ~40% dela, o toque logo.)
                query.naturalContact = kind == GestureKind3D::Shot
                    ? std::clamp((0.10f + 0.12f * power) * settings.shotGestureSeconds, 0.05f, 0.25f) + 0.05f
                    : 0.07f;
                query.earliest = kind == GestureKind3D::Shot ? std::max(0.07f, 0.4f * query.naturalContact)
                    : automatic && autoState.sinceSharp < 0.3f ? 0.04f : 0.05f;
                // (A bola vindo: ele espera por ela. O clique guardado e o dominio nao
                // planejam longe - um toque 1,6 s depois do aperto nao e resposta.)
                query.latest = coming || (fromArmed && !armed.buffered) ? 1.6f : 0.6f;
                // (O voleio: o pe ate ~1 m no chute; o toque ate a altura do joelho e meio.)
                query.highestBall = kind == GestureKind3D::Shot ? 1.0f : bodyReach.highBall + 0.30f;
                if (!automatic) query.timeWeight = 2.5f;
                // (O pe leva tempo ate a bola: o toque a ~3 m/s, a batida do chute a ~5.)
                if (!automatic) query.footSpeed = kind == GestureKind3D::Shot ? 5.0f : 3.0f;
                // (O dominio tenta primeiro o ajuste curto, abaixo: ele segue um instante
                // ate a bola em vez de apostar no deslize da freada.)
                if (!stopTouch) contactPlan = planBallContact3D(query);
                // (Sem encontro natural - a virada ou a freada seca: o mais cedo
                // possivel, o pe saindo mais da passada e o apoiado tambem.)
                if (!contactPlan.valid && !stopTouch) {
                    query.maxDeviation = 0.80f;
                    query.latest = std::max(query.latest, 0.4f);
                    contactPlan = planBallContact3D(query);
                }
                // (17/11 14h45 e ~15h, ele: "pode reduzir a velocidade do jogador ou dar
                // uma puxada nele, desde que bem suave e harmonico" - "coisa imediata e
                // rapida.. nunca travar o jogador ou ser algo demorado.. algo sutil") Sem
                // encontro como as teclas pedem: um ajuste curto na velocidade dele - no
                // maximo 1 m/s a mais ou a menos, com o contato em ate 0,36 s - que poe a
                // bola no pe. No dominio a base e a velocidade de agora: ele segue um
                // instante ate a bola e para nela, em vez de frear antes dela.
                if (!contactPlan.valid && !automatic && command.controlled) {
                    const float radius = scene.inflatedBallProfile(nearest->handle).radiusMeters;
                    approachBase = stopTouch ? Vec3 { bodyVelocityNow.x, bodyVelocityNow.y, 0.0f }
                        : Vec3 { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
                    // (No dominio ate 0,48 s: ele segue no embalo dele ate a bola e para nela.)
                    for (const float meet : { 0.12f, 0.18f, 0.24f, 0.30f, 0.36f, 0.42f, 0.48f }) {
                        if (meet > 0.37f && !stopTouch) break;
                        const Vec3 ballThen = predictBallPosition3D(nearest->state->position, nearest->state->linearVelocity,
                            radius, feetBase, autoState.deceleration, meet);
                        Vec3 toBall { ballThen.x - pelvis.x, ballThen.y - pelvis.y, 0.0f };
                        const float distance = toBall.length();
                        if (distance < 1e-3f) continue;
                        const Vec3 spot = Vec3 { ballThen.x, ballThen.y, 0.0f } - toBall * (std::min(0.32f, distance) / distance);
                        Vec3 nudge = (spot - Vec3 { pelvis.x, pelvis.y, 0.0f }) * (1.0f / meet) - approachBase;
                        if (nudge.length() > 1.0f) nudge = nudge * (1.0f / nudge.length());
                        BallContactQuery3D assisted = query;
                        assisted.wantedVelocity = approachBase + nudge;
                        assisted.latest = meet + 0.06f;
                        const BallContactPlan3D plan = planBallContact3D(assisted);
                        if (plan.valid) {
                            contactPlan = plan;
                            approachPlanned = true;
                            break;
                        }
                    }
                }
                // (O dominio sem encontro curto: o plano com ele freando como as teclas
                // pedem - a bola perto.)
                if (!contactPlan.valid && stopTouch) {
                    query.latest = 0.5f;
                    contactPlan = planBallContact3D(query);
                }
                if (std::getenv("XQT") != nullptr)
                    std::printf("plano: %s perna %d em %.2f s | bola %+.2f %+.2f %.2f | desvio %.2f | agora %+.2f %+.2f\n",
                        contactPlan.valid ? "ok" : "NENHUM", contactPlan.side, contactPlan.seconds, contactPlan.ballInBody.x,
                        contactPlan.ballInBody.y, contactPlan.ballInBody.z, contactPlan.deviation, ballInBody.x, ballInBody.y);
            }
            const Vec3 choiceBall = contactPlan.valid ? contactPlan.ballInBody : ballInBody;
            if (contactPlan.valid) {
                preferredSide = contactPlan.side;
                choiceReach.centerBand = 1.0f;
            }
            const GestureChoice3D choice = chooseGesture3D(kind, choiceBall,
                { dot(aimed, forward), dot(aimed, left), 0.0f }, preferredSide, choiceReach,
                sequence && !contactPlan.valid ? 1 - athlete.lastGestureSide : -1, receive);
            // A recepcao armada (24/10, pedido do usuario): com a bola vindo e
            // ainda longe, o pedido espera ate receiveArmSeconds e a forca sai
            // quando ela chega - no alcance, a receiveArmDistance da pelve (onde
            // o gesto pronto a para) ou parando de vir. O chute na bola vindo
            // arma do mesmo jeito (28/10, pedido do usuario: o chute de
            // primeira). No manejo e no chute normais a forca sai no aperto (fora
            // do alcance, o pedido se perde); armado, se ela deixou de vir fora
            // do alcance, cai.
            // O peito e a cabeca (24/10, pedido do usuario): fora do alcance dos
            // pes, a bola na altura do tronco ou da cabeca (acima da pelve: sobe
            // com o pulo) e perto do corpo - o toque e o chute com uma fracao da
            // forca (BallOutcome3D). O levantar tambem vai de cabeca (28/10, pedido
            // do usuario: "levantar a bola com a cabeca quando ela esta no ar"), mas
            // nao de peito.
            const GestureReach3D& reach = bodyReach;
            const float aboveHip = ballPosition.z - pelvis.z;
            const bool free = !choice.inReach;
            const bool nearBody = free && !raising && ballInBody.x < reach.bodyFront && ballInBody.x > -reach.bodyBack
                && std::abs(ballInBody.y) < reach.bodySide;
            const bool nearHead = free && ballInBody.x < reach.headFront && ballInBody.x > -reach.headBack
                && std::abs(ballInBody.y) < reach.headSide;
            const BallBodyPart3D part = nearBody && aboveHip > reach.chestBottom && aboveHip <= reach.chestTop
                ? BallBodyPart3D::Chest
                : nearHead && aboveHip > reach.chestTop && aboveHip <= reach.headTop ? BallBodyPart3D::Head
                                                                                     : BallBodyPart3D::Foot;
            const bool upper = part != BallBodyPart3D::Foot;
            const bool waits = receive || fromArmed;
            // (O chute e o levantar guardados batem onde o chute bate - a bola do
            // balde dele, ao lado do pe de apoio -, nao onde a recepcao a para.)
            const bool kickWaits = fromArmed ? !armed.receive : !receive && (request == BallActionKind3D::Shot || raising);
            GestureSpec3D strike;
            strike.kind = GestureKind3D::Shot;
            strike.high = high;
            const Vec3 strikeBall = gestureNominalBall3D(strike);
            const float armDistance = bodyScale3D(assets.body)
                * (kickWaits ? std::hypot(strikeBall.x, strikeBall.y) : settings.ballForce.receiveArmDistance);
            bool fires = upper || (choice.inReach
                && (contactPlan.valid || !waits || !coming || offset.length() <= armDistance));
            // (17/11 14h20, o toque assistido) Com o gesto do pe e sem encontro
            // possivel agora (a bola ainda fora do alcance, longe ou no ar), o clique
            // fica guardado ate 0,5 s e sai no primeiro instante em que o pe pode
            // chegar nela; no ultimo tick, o toque de antes (o gesto e a forca no
            // encosto). Nada de clique perdido por meio passo.
            const bool bufferable = settings.realisticTouch && gestureBody && !upper && !automatic && !startsConduction
                && !contactPlan.valid && offset.length() < 2.5f;
            if (bufferable && !(fromArmed && armed.buffered && armed.left <= deltaTime * 1.5f)) {
                if (!fromArmed) {
                    armed = { true, request == BallActionKind3D::Shot, requestPower, requestLift, requestCurve, requestAimYaw,
                        0.5f, receive, false, raising };
                    armed.buffered = true;
                }
                fires = false;
            }
            // (O gesto do pedido guardado ja comecou: na chegada, so a forca.)
            const bool started = fromArmed && armed.started;
            // (O chute e o levantar guardados esperam tambem o caminho da bola da
            // distancia da recepcao ate a deles: a janela e a mesma da recepcao,
            // contada ate o ponto dela - com 0,3 s a partir do aperto, a bola a 4
            // m/s apertada a 1,5 m chegava depois e o chute se perdia.)
            if (!fires && !fromArmed && (receive || (coming && (request == BallActionKind3D::Shot || raising)))) {
                const float extra = kickWaits
                    ? std::max(0.0f, settings.ballForce.receiveArmDistance * bodyScale3D(assets.body) - armDistance)
                        / std::max(1.0f, closing)
                    : 0.0f;
                armed = { true, request == BallActionKind3D::Shot, requestPower, requestLift, requestCurve, requestAimYaw,
                    settings.ballForce.receiveArmSeconds + extra, receive, false, raising };
            }
            if (fromArmed && !choice.inReach && !coming && !armed.buffered) armed.active = false;
            const Vec3 aimedInBody { dot(aimed, forward), dot(aimed, left), 0.0f };
            // O gesto do pe de um balde: o tempo, a semente, o rumo em que a bola
            // vem (na recepcao) e o giro da cintura ate o rumo em que ela vai
            // (flight, no chute) e ate onde ela esta (at, no referencial do corpo).
            const auto footGesture = [&](const GestureSpec3D& spec, Vec3 flight, Vec3 at, Vec3 drift) {
                gestureStart = spec;
                // (17/11, o toque realista) O pe vai ate a bola de verdade.
                if (settings.realisticTouch) {
                    gestureStart.aimBall = true;
                    gestureStart.ballAt = at;
                    gestureStart.ballDrift = drift;
                    // (O toque automatico na curva: a bola foge de lado com o corpo girando.)
                    if (automatic || startsConduction) {
                        gestureStart.aimSlack = 0.10f;
                        gestureStart.aimTravel = 0.30f;
                    }
                    if (std::getenv("XQT") != nullptr && (automatic || startsConduction))
                        std::printf("toque auto: perna %d alcance %d prof %d | bola %+.2f %+.2f %.2f deriva %+.2f %+.2f | livre %d\n",
                            gestureStart.side, gestureStart.reach, gestureStart.depth, at.x, at.y, at.z, drift.x, drift.y, freeSide);
                }
                gestureStart.power = std::clamp(requestPower, 0.0f, 1.0f);
                // (Andando, o pe de apoio pousa perto do meio do corpo: o chute no
                // meio cruzava as pernas. So parado, com a base aberta.)
                if (gait.valid && gait.cycling) gestureStart.inward = 0.0f;
                // (A recepcao tem o tempo do manejo, mesmo no chute fraco; ela
                // e o toque na bola alta, um pouco mais: a perna sobe e cede -
                // 1,2 e 1,15 vezes.)
                gestureStart.seconds = kind == GestureKind3D::Shot && !receive ? settings.shotGestureSeconds
                    : settings.touchGestureSeconds * (receive ? 1.2f : spec.high ? 1.15f : 1.0f);
                if (receive) {
                    const Vec3 relative { ballVelocity.x - bodyVelocity.x, ballVelocity.y - bodyVelocity.y, 0.0f };
                    gestureStart.incoming = { dot(relative, forward), dot(relative, left), 0.0f };
                }
                gestureStart.seed = ++athlete.gestureCount * 2654435761u + static_cast<std::uint32_t>(slot.entityId);
                // A cintura gira ate o rumo em que a bola vai (no referencial
                // do corpo): andando de lado, o chute parecia ir para o lado
                // contrario (pedido do usuario).
                if (kind == GestureKind3D::Shot && !receive && flight.lengthSquared() > 1e-4f)
                    gestureStart.turn = std::atan2(dot(flight, left), dot(flight, forward));
                // O peito e a cintura viram para a bola de lado (28/10, pedido do
                // usuario: andando para a frente com ela de lado, o chute ficava
                // esquisito): 40% do angulo dela alem de 15 graus, ate 25 graus.
                // (So andando: parado, o giro a mais subiu o pico das pernas ate
                // 26% e o chute longe ganhou perna louca. Com a bola quase a
                // frente, 5 graus nao se viam e o pico do sprint subia 16%.)
                if (kind == GestureKind3D::Shot && !receive && gait.valid && gait.cycling && at.x * at.x + at.y * at.y > 1e-4f) {
                    const float angle = std::atan2(at.y, at.x);
                    gestureStart.face = std::clamp(0.4f * (angle - std::clamp(angle, -0.26f, 0.26f)), -0.44f, 0.44f);
                }
                gestureRequested = true;
                athlete.lastGestureSide = gestureStart.side;
                athlete.sinceGesture = 0.0f;
                athlete.pendingGesture.active = false;
            };
            // A previsao (28/10, pedido do usuario: o gesto coincidir com onde a
            // bola vai estar): o pedido guardado comeca o gesto ja, no balde de
            // onde ela vai chegar - andando no movimento relativo de agora ate
            // entrar a armDistance da pelve, ou passar mais perto -, com o
            // contato marcado na chegada: a batida coincide com a forca.
            if (gestureBody && armed.active && !armed.started && !fires && !upper) {
                const Vec3 relative { flatBall.x - bodyVelocity.x, flatBall.y - bodyVelocity.y, 0.0f };
                const float arrival = gestureBallArrival3D(offset, relative, armDistance);
                if (arrival > 0.0f && arrival <= armed.left) {
                    const Vec3 at = offset + relative * arrival;
                    const float radius = scene.inflatedBallProfile(nearest->handle).radiusMeters;
                    const float height
                        = std::max(radius, ballPosition.z + ballVelocity.z * arrival - 4.905f * arrival * arrival);
                    const GestureChoice3D early = chooseGesture3D(kind, { dot(at, forward), dot(at, left), height - feetBase },
                        aimedInBody, preferredSide, bodyReach, -1, receive);
                    if (early.inReach) {
                        footGesture(early.spec, aimed, { dot(at, forward), dot(at, left), height - feetBase }, {});
                        gestureStart.contactAt = arrival;
                        armed.started = true;
                    }
                }
            }
            if (fires) {
                armed.active = false;
                const InflatedBallProfile3D ballProfile = scene.inflatedBallProfile(nearest->handle);
                BallOutcomeRequest3D outcomeRequest;
                outcomeRequest.kind = request;
                outcomeRequest.power = requestPower;
                // (Na recepcao, o resto vai para onde as teclas mandam; sem
                // tecla, a bola para.)
                // (No peito, o toque e a parada: o resto no rumo das teclas.)
                // (Levantar: no rumo das teclas; sem tecla, reto para cima.)
                // (A cabecada com o manejo vai no rumo das teclas ou, sem tecla, no
                // do olhar, como o manejo - 28/10, pedido do usuario: na bola
                // vindo, ela ia so pelas teclas e, sem tecla, para um rumo fixo.)
                outcomeRequest.direction = (receive && part != BallBodyPart3D::Head) || raising
                        || (part == BallBodyPart3D::Chest && kind == GestureKind3D::Touch)
                    ? keys : aimed;
                outcomeRequest.receive = receive && !upper;
                // (17/11) O toque da conducao automatica sai na velocidade do plano.
                if (((automatic && !autoReceive) || startsConduction) && autoSpeed > 0.0f && !outcomeRequest.receive
                    && part == BallBodyPart3D::Foot)
                    outcomeRequest.touchSpeed = autoSpeed;
                outcomeRequest.part = part;
                outcomeRequest.aimYawRadians = aimYaw;
                outcomeRequest.lift = requestLift;
                outcomeRequest.curve = requestCurve;
                const Vec3 ahead { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
                const BallOutcome3D outcome = computeBallOutcome3D(outcomeRequest, nearest->state->linearVelocity,
                    bodyVelocity, ahead, ballProfile.radiusMeters, settings.ballForce);
                const bool shotConduction = request == BallActionKind3D::Shot && part == BallBodyPart3D::Foot
                    && !outcomeRequest.receive && settings.ballForce.conduction;
                // (17/11, o toque realista - pedido dele) Com o gesto do pe
                // rodando, a forca espera a chuteira encostar na bola (logo
                // abaixo, depois do gesto comecar). Sem gesto (o fisico, o
                // Ativo sem a ponte) e no peito e na cabeca, sai na hora.
                const bool footContact = settings.realisticTouch && gestureBody && part == BallBodyPart3D::Foot;
                if (!footContact) {
                    applyBallOutcome(*nearest, outcomeRequest, ahead);
                    if (shotConduction) {
                        athlete.conduction.active = true;
                        athlete.conduction.ball = nearest->handle;
                        athlete.conduction.shot = startShotConduction3D(command.facingYawRadians, requestPower, settings.ballForce);
                    }
                }
                // O drible fraco do jogador liga a conducao automatica (pedido dele: "ate uns 35%").
                if (startsConduction) {
                    autoState.active = true;
                    autoState.ball = nearest->handle;
                    autoState.away = 0.0f;
                    autoState.lastSpeed = -1.0f;
                }
                if (automatic) autoState.sinceTouch = 0.0f;
                if (automatic || startsConduction)
                    autoState.plannedWanted = { command.intent.requestedVelocityWorld.x,
                        command.intent.requestedVelocityWorld.y, 0.0f };
                if (started) {
                    // (O gesto guardado ja esta batendo: so a forca.)
                } else if (gestureBody && upper) {
                    // O gesto so do corpo, no lado da bola, virado ao rumo dela.
                    gestureStart = {};
                    gestureStart.kind = kind;
                    gestureStart.part = part == BallBodyPart3D::Chest ? GesturePart3D::Chest : GesturePart3D::Head;
                    gestureStart.side = ballInBody.y >= 0.0f ? 0 : 1;
                    gestureStart.power = std::clamp(requestPower, 0.0f, 1.0f);
                    gestureStart.lift = requestLift;
                    gestureStart.seconds = kind == GestureKind3D::Shot ? settings.shotGestureSeconds : settings.touchGestureSeconds;
                    const Vec3 flight { outcome.velocity.x - bodyVelocity.x, outcome.velocity.y - bodyVelocity.y, 0.0f };
                    if (flight.lengthSquared() > 0.25f) gestureStart.turn = std::atan2(dot(flight, left), dot(flight, forward));
                    gestureStart.seed = ++athlete.gestureCount * 2654435761u + static_cast<std::uint32_t>(slot.entityId);
                    gestureRequested = true;
                    athlete.pendingGesture.active = false;
                } else if (gestureBody) {
                    // Conduzindo (andando, o toque - ou o chute fraco - com a bola
                    // a frente), o toque e do passo seguinte da perna escolhida
                    // (24/10, pedido do usuario: "como se ele planejasse no proprio
                    // passo"): plantada, ela espera sair do chao; no ar, ja vai.
                    const bool carrying = gait.valid && gait.cycling && conducting && !high
                        && std::abs(ballInBody.y) <= bodyReach.alternateBand
                        && ballInBody.x > bodyReach.depthBack;
                    // (Na hora, o balde e o da bola de agora: a forca sai no
                    // aperto e o caminho dela depois e o do proprio toque. A
                    // previsao no toque na hora - o balde da bola quando o pe
                    // chega la - foi medida em 28/10 e piorou: o pe entrava mais
                    // na bola.)
                    if (contactPlan.valid) {
                        // O gesto planejado: o pe na bola prevista, no instante do
                        // contato; ele comeca so o tanto antes que o movimento pede
                        // (a passada leva a perna ate la), contado da armada do chute
                        // ou 0,12 s antes no toque.
                        footGesture(choice.spec, { outcome.velocity.x, outcome.velocity.y, 0.0f }, contactPlan.ballInBody, {});
                        gestureStart.contactAt = contactPlan.seconds;
                        const float lead = kind == GestureKind3D::Shot
                            ? std::clamp((0.10f + 0.12f * std::clamp(requestPower, 0.0f, 1.0f)) * settings.shotGestureSeconds,
                                  0.05f, 0.25f) + 0.06f
                            : 0.12f;
                        const float startIn = contactPlan.seconds - lead;
                        if (startIn > 0.02f) {
                            athlete.pendingGesture = { true, gestureStart, 0.0f, startIn };
                            gestureRequested = false;
                        }
                    } else {
                        footGesture(choice.spec, { outcome.velocity.x, outcome.velocity.y, 0.0f }, ballInBody,
                            { dot(flatBall - bodyVelocity, forward), dot(flatBall - bodyVelocity, left), 0.0f });
                        if (carrying && gait.cyclePlanted[gestureStart.side == 1 ? 1 : 0] && !automatic && !startsConduction) {
                            athlete.pendingGesture = { true, gestureStart, 0.0f };
                            gestureRequested = false;
                        }
                    }
                }
                if (footContact) {
                    // A espera: a perna que ainda vai sair do chao (ate 0,4 s), a
                    // armada e a batida do gesto e uma folga.
                    athlete.pendingTouch = { true, nearest->handle, outcomeRequest, ahead,
                        athlete.lastGestureSide == 1 ? 1 : 0,
                        // (O toque automatico espera pouco: errou, o proximo vem logo.)
                        contactPlan.valid ? contactPlan.seconds + 0.25f
                            : automatic || startsConduction ? 0.40f : request == BallActionKind3D::Shot ? 0.95f : 0.75f,
                        shotConduction, requestPower, command.facingYawRadians, 1.0e9f,
                        automatic || startsConduction,
                        Vec3 { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f },
                        contactPlan.valid ? contactPlan.seconds : -1.0f, 0.0f, 1.0e9f,
                        contactPlan.valid ? contactPlan.ballWorld : Vec3 {}, approachPlanned, stopTouch };
                    athlete.pendingTouch.gesture = athlete.pendingGesture.active ? athlete.pendingGesture.spec : gestureStart;
                    athlete.pendingTouch.approachBase = approachBase;
                    athlete.pendingTouch.windupEnd = kind == GestureKind3D::Shot && contactPlan.valid
                        ? std::max(0.0f, contactPlan.seconds - 0.06f) : 0.0f;
                }
            }
        }
    } else {
        athlete.armedReception.active = false;
    }
    // A entrada do modo padrao (capsula, teclas, manejo) e os ajustes do
    // laboratorio; tambem na saida do corpo guiado, com o padrao rodando
    // junto do fisico.
    const auto defaultLocomotionInput = [&]() {
        const PhysicsCharacterState3D& capsule = nav();
        DefaultLocomotionInput3D input;
        input.feetPositionWorld = command.rootPositionWorld - Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
        input.velocityWorld = capsule.velocity;
        input.grounded = capsule.grounded;
        input.requestedVelocityWorld = command.intent.requestedVelocityWorld;
        input.facingYawRadians = command.facingYawRadians;
        input.lookPitchRadians = command.lookPitchRadians;
        input.groundAt = command.groundAt;
        input.deltaTime = deltaTime;
        input.jumpCrouch = command.jumpCrouch;
        input.tackleRequest = command.intent.crouch && !state.crouchHeld;
        input.tackleEnabled = settings.tackleEnabled;
        // A voadora: Ctrl com Shift + W no ar (W: para a frente do olhar).
        input.tackleHeld = command.intent.crouch;
        {
            const Vec3 keys = command.intent.requestedDirectionWorld;
            const float forward = keys.x * std::cos(command.facingYawRadians)
                + keys.y * std::sin(command.facingYawRadians);
            input.airTackleCombo = command.intent.effort > 0.5f && forward > 0.7f;
        }
        state.crouchHeld = command.intent.crouch;
        if (gestureRequested) input.gesture = gestureStart;
        input.gestureRequested = gestureRequested;
        input.motionVariety = command.motionVariety;
        input.varietySeed = command.varietySeed;
        input.leading = defaultMode && state.bodyMode == BodyMode::Animated;
        configureDefaultLocomotion(state.defaultLocomotion.settings(), settings, assets.body);
        return input;
    };
    if (!animated && state.guidedExitLeft > 0.0f) {
        // Saida do corpo guiado: a pose do padrao com a autoridade
        // descendo enquanto os motores do fisico assumem.
        const DefaultLocomotionInput3D input = defaultLocomotionInput();
        state.defaultLocomotion.update(profile, defaultClips, input);
        const DefaultLocomotionOutput3D& out = state.defaultLocomotion.output();
        const float x = std::clamp(state.guidedExitLeft / std::max(1e-3f, state.guidedExitSeconds), 0.0f, 1.0f);
        const std::vector<float> authority(profile.links.size(), x * x * (3.0f - 2.0f * x));
        if (out.valid)
            scene.setRagdollGuidePose(slot.handle, out.pose.linkPositions, out.pose.linkOrientations, authority);
        state.guidedExitLeft -= deltaTime;
        // (No fim, de novo sem o giro em volta da vertical: a pose ainda
        // o dava - no ar, o voo girava ate 60 graus.)
        if (state.guidedExitLeft <= 0.0f) scene.removeRagdollVerticalSpin(slot.handle);
    }
    if (!animated) athlete.footsteps.clear();
    if (active) {
        // Corpo Ativo: a pose do padrao (a mesma do animado; vinda do fisico,
        // com a inercia da volta) executada pelo corpo fisico.
        const DefaultLocomotionInput3D input = defaultLocomotionInput();
        state.defaultLocomotion.update(profile, defaultClips, input);
        const DefaultLocomotionOutput3D& out = state.defaultLocomotion.output();
        if (out.rootScripted) {
            placeNav({ out.tackleRootWorld.x, out.tackleRootWorld.y, groundHeight(out.tackleRootWorld) });
        }
        if (out.tackleEnded) setNavVelocity({ out.tackleVelocity.x, out.tackleVelocity.y, 0.0f });
        state.defaultTackling = out.tackling;
        if (out.valid) {
            RagdollAnimationPose3D pose = out.pose;
            std::vector<Vec3> coordinates = out.coordinates;
            Vec3 rootPosition = out.rootPosition;
            Quaternion rootOrientation = out.rootOrientation;
            if (state.returnInertia.active()) {
                state.returnInertia.apply(coordinates, rootPosition, rootOrientation, deltaTime);
                defaultLocomotionPose3D(profile, rootPosition, rootOrientation, coordinates, pose);
            }
            const Vec3 plannedRoot = rootPosition;
            const BodyObservation3D observation = athlete.observer.observe(profile, body, deltaTime);
            // Os pes onde estao no chao (e a acomodacao de volta a base).
            const Vec3 capsuleVelocity = nav().velocity;
            std::array<bool, 2> planted = out.footPlanted;
            // (Na ponte parado tambem: o pe apoiado fica onde esta no chao - a
            // pose do padrao o arrastava ate o lugar do clipe, o atrito nao
            // deixava e no guiado ele saltava. Andando, os pes vao com a
            // passada do padrao: presos, na troca para o guiado o pe corria
            // ate o lugar do clipe.)
            const bool bridgeMoving = state.bridge
                && (std::hypot(command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y) > 0.5f
                    || std::hypot(capsuleVelocity.x, capsuleVelocity.y) > 0.5f);
            state.bridgeStance = state.bridge && !bridgeMoving;
            if (!bridgeMoving)
                athlete.stance.adapt(profile, body, observation, planted,
                    std::hypot(capsuleVelocity.x, capsuleVelocity.y), settings.activeStanceSettings, deltaTime,
                    rootPosition, rootOrientation, coordinates, pose);
            ActiveBodyTarget3D target;
            target.pose = &pose;
            target.coordinates = &coordinates;
            target.rootPosition = rootPosition;
            target.rootOrientation = rootOrientation;
            target.intendedSpeed = std::hypot(command.intent.requestedVelocityWorld.x,
                command.intent.requestedVelocityWorld.y);
            // (Na ponte de contato, o nocaute por pancada e a pancada forte dela:
            // o encostao abaixo disso fica na ponte.)
            ActiveBodySettings3D activeSettings = settings.activeBodySettings;
            if (state.contactBridge)
                activeSettings.knockoutImpulse = std::max(activeSettings.knockoutImpulse,
                    settings.contactBridgeHardDeltaV * std::max(1.0f, athlete.contactBody.massKg));
            athlete.active.update(profile, body, observation, target, activeSettings, deltaTime, athlete.activeOutput);
            const ActiveBodyOutput3D& activeOutput = athlete.activeOutput;
            scene.setRagdollActiveDriveTargets(slot.handle, activeOutput.drives, true);
            scene.setRagdollFrictionScale(slot.handle, 1.0f);
            // (Na ponte de contato decidindo, so os musculos: sem a ajuda nem a
            // guia o choque bate num corpo fisico de verdade, e a pancada forte
            // cai como no fisico pleno.)
            const bool deciding = state.contactBridge && state.contactDecisionLeft > 0.0f;
            for (const LinkWrench3D& wrench : activeOutput.wrenches) {
                if (deciding) break;
                scene.applyRagdollControlLinkForce(slot.handle, wrench.link, wrench.force, wrench.torque);
                athlete.externalAssist.activeNewtons += wrench.force.length();
            }
            if (!deciding && activeOutput.assistForce.lengthSquared() > 1e-6f) {
                scene.applyRagdollControlRootForce(slot.handle, activeOutput.assistForce, {});
                athlete.externalAssist.activeNewtons += activeOutput.assistForce.length();
            }
            // A ponte: o guiamento dos ossos ate a pose, a autoridade subindo
            // de bridgeAuthorityStart a cheia no tempo dela.
            float bridgeShare = 0.0f;
            if (state.bridge) {
                state.bridgeSeconds += deltaTime;
                bridgeShare = std::clamp(state.bridgeSeconds / std::max(0.05f, state.bridgeDuration), 0.0f, 1.0f);
                // (A de contato parte da guia que o choque deixou.)
                const float start = state.contactBridge ? std::clamp(state.bridgeAuthorityFloor, 0.0f, 1.0f)
                                                        : std::clamp(settings.bridgeAuthorityStart, 0.0f, 1.0f);
                // (Com a tabela, a guia cai junto com a ajuda do impacto: a
                // ponte nao segura um corpo que levou pancada. A de contato
                // segura: a pancada forte e a perda de base a levam ao fisico.)
                const float authority = deciding ? 0.0f
                    : (start + (1.0f - start) * bridgeShare * bridgeShare * (3.0f - 2.0f * bridgeShare))
                        * (modeTable && !state.contactBridge ? state.impactAssist.level() : 1.0f);
                const std::vector<float> authorities(profile.links.size(), authority);
                scene.setRagdollGuidePose(slot.handle, pose.linkPositions, pose.linkOrientations, authorities);
            }
            state.previousAnimationPose = state.animationPose.linkPositions.empty() ? pose : state.animationPose;
            state.animationPose = pose;
            state.physicsBlend = 1.0f;
            // A capsula vai atras do corpo quando ele se afasta da pose do
            // plano (pancada, parede): a pose ancora onde o corpo esta.
            // Parado, a raiz-alvo fica sobre os pes (ActiveStance3D): a
            // capsula vai para onde a base real a quer.
            // (A raiz indo para cima da base real no ritmo de uma troca de peso, BaseShiftSpeed: sem limite, depois de
            // levantar a capsula ia a 2,3 m/s para os pes e arrastava o corpo 0,3 m - o puxao de volta; 02/11.)
            constexpr float BaseShiftSpeed = 0.8f;
            const bool baseShift = athlete.stance.baseShift().lengthSquared() > 1e-8f;
            Vec3 follow = baseShift
                ? navigationFollowVelocity3D(rootPosition, plannedRoot, 0.0f)
                : navigationFollowVelocity3D(body.links.front().position, rootPosition,
                    state.contactBridge ? settings.contactBridgeFollowSlack : settings.activeFollowSlack);
            if (baseShift && follow.length() > BaseShiftSpeed) follow = follow * (BaseShiftSpeed / follow.length());
            // (Na ponte de contato a capsula leva o golpe pendente junto com o
            // corpo: fora da inercia do controlador, que o anularia num tick.)
            if (state.contactBridge) follow += Vec3 { state.contactKick.x, state.contactKick.y, 0.0f };
            setFollow(follow);
            if (activeOutput.knockout) {
                toPhysical(("nocaute: " + activeOutput.knockoutReason).c_str());
            } else if (!state.bridge && !athlete.active.handingOver()
                && athlete.stance.captureOutside() > activeCaptureLimit(settings, athlete.stance)) {
                toPhysical(("passo de equilibrio, captura " + std::to_string(athlete.stance.captureOutside()) + " m fora"
                    + (athlete.stance.stepping() ? " no passo" : "")).c_str());
            } else if ((state.bridge && bridgeShare >= 1.0f)
                || (!state.bridge && activeOutput.readyToAnimate && !athlete.stance.stepping()
                    && !state.returnInertia.active() && athlete.stance.baseShift().length() < 0.02f
                    && (athlete.stance.mismatch() < 0.04f
                        || athlete.active.secondsActive() > settings.activeReturnTimeout))) {
                // O guiado parte do corpo de agora (juntas e pelve) e vai a
                // pose do padrao pela inercia da volta: sem estalo nos pes
                // (a base do Ativo nao e a do clipe).
                // (02/11, ele: "tranco... como se realocasse o corpo para encaixar", pior a 300%: da ponte andando o
                // guiado ia direto a pose do clipe, e o pe que a ponte nao alcancara saltava ate ela - 0,23 m num tick,
                // 3360 m/s2. Agora pela inercia, do corpo de verdade, e com a primeira pose ja neste tick. A ponte de
                // contato andando segue a pose de antes: o corpo nunca saiu do padrao, e partir dele herdava o golpe.)
                if (!state.bridge || state.bridgeStance || !state.contactBridge) {
                    const PhysicsBodyState3D& pelvis = body.links.front();
                    state.returnCoordinates.assign(profile.links.size(), {});
                    std::vector<Vec3> velocities(profile.links.size());
                    for (std::size_t i = 1; i < profile.links.size() && i < body.joints.size(); ++i) {
                        const auto& joint = body.joints[i].positionRadians;
                        const auto& rate = body.joints[i].velocityRadiansPerSecond;
                        state.returnCoordinates[i] = Vec3 { joint[0], joint[1], joint[2] };
                        velocities[i] = { rate[0], rate[1], rate[2] };
                    }
                    state.returnInertia.start(state.returnCoordinates, velocities, pelvis.position,
                        pelvis.orientation, pelvis.linearVelocity, pelvis.angularVelocity,
                        settings.returnSmoothing ? ActiveExitHalfLife : 0.10f, settings.returnSmoothing ? ActiveExitSeconds : 0.40f);
                    // (E ja neste tick, sobre a pose crua do padrao - a de agora traz a inercia da entrada na ponte,
                    // que esta substitui -: o guiado seguia o ultimo alvo, o da ponte, e o pe ia ao clipe e voltava.)
                    std::vector<Vec3> first = out.coordinates;
                    Vec3 firstRoot = out.rootPosition;
                    Quaternion firstOrientation = out.rootOrientation;
                    state.returnInertia.apply(first, firstRoot, firstOrientation, deltaTime);
                    RagdollAnimationPose3D firstPose = pose;
                    defaultLocomotionPose3D(profile, firstRoot, firstOrientation, first, firstPose);
                    scene.setRagdollGuidePose(slot.handle, firstPose.linkPositions, firstPose.linkOrientations,
                        std::vector<float>(profile.links.size(), 1.0f));
                }
                scene.setRagdollGuided(slot.handle, true);
                state.guided = true;
                state.bodyMode = !state.returnInertia.active() ? BodyMode::Animated : BodyMode::Returning;
                state.bodyModeSeconds = 0.0f;
                Log::info(state.bridge ? "corpo: ponte -> padrao" : "corpo: ativo -> padrao");
                state.bridge = false;
                state.contactBridge = false;
                state.contactDecisionLeft = 0.0f;
            }
        }
        return;
    }
    if (animated) {
        // Modo padrao: a pose da animacao sobre a capsula (o corpo guiado
        // chega nela no fim do passo). Na volta do fisico, a pose fisica
        // do comeco (levada com a capsula) misturada ate ela.
        if (!state.guided) {
            scene.setRagdollGuided(slot.handle, true);
            state.guided = true;
        }
        const DefaultLocomotionInput3D input = defaultLocomotionInput();
        state.defaultLocomotion.update(profile, defaultClips, input);
        const DefaultLocomotionOutput3D& out = state.defaultLocomotion.output();
        // Carrinho: a capsula vai embaixo da pelve do clipe; no fim sai
        // com o embalo dela.
        if (out.rootScripted) {
            placeNav({ out.tackleRootWorld.x, out.tackleRootWorld.y, groundHeight(out.tackleRootWorld) });
        }
        if (out.tackleEnded) setNavVelocity({ out.tackleVelocity.x, out.tackleVelocity.y, 0.0f });
        if (out.tackling && !state.defaultTackling) Log::info("corpo: carrinho (padrao)");
        state.defaultTackling = out.tackling;
        if (out.valid) {
            RagdollAnimationPose3D pose = out.pose;
            std::vector<Vec3> coordinates = out.coordinates;
            Vec3 rootPosition = out.rootPosition;
            Quaternion rootOrientation = out.rootOrientation;
            if (state.bodyMode == BodyMode::Returning && state.returnInertia.active()) {
                state.returnInertia.apply(coordinates, rootPosition, rootOrientation, deltaTime);
                defaultLocomotionPose3D(profile, rootPosition, rootOrientation, coordinates, pose);
                if (!state.returnInertia.active()) {
                    state.bodyMode = BodyMode::Animated;
                    state.bodyModeSeconds = 0.0f;
                }
            } else if (state.bodyMode == BodyMode::Returning) {
                constexpr float ReturnSeconds = 0.60f;
                const float x = std::clamp(state.bodyModeSeconds / ReturnSeconds, 0.0f, 1.0f);
                const float share = x * x * (3.0f - 2.0f * x);
                for (std::size_t i = 0; i < coordinates.size() && i < state.returnCoordinates.size(); ++i)
                    coordinates[i] = state.returnCoordinates[i]
                        + (coordinates[i] - state.returnCoordinates[i]) * share;
                const Vec3 from = input.feetPositionWorld + state.returnRootOffset;
                Quaternion fromOrientation = state.returnRootOrientation;
                const Quaternion to = out.rootOrientation;
                if (fromOrientation.x * to.x + fromOrientation.y * to.y + fromOrientation.z * to.z
                        + fromOrientation.w * to.w < 0.0f)
                    fromOrientation = { -fromOrientation.x, -fromOrientation.y, -fromOrientation.z, -fromOrientation.w };
                rootOrientation = Quaternion {
                    fromOrientation.x + (to.x - fromOrientation.x) * share,
                    fromOrientation.y + (to.y - fromOrientation.y) * share,
                    fromOrientation.z + (to.z - fromOrientation.z) * share,
                    fromOrientation.w + (to.w - fromOrientation.w) * share }.normalized();
                rootPosition = from + (out.rootPosition - from) * share;
                defaultLocomotionPose3D(profile, rootPosition, rootOrientation, coordinates, pose);
                if (x >= 1.0f) {
                    state.bodyMode = BodyMode::Animated;
                    state.bodyModeSeconds = 0.0f;
                }
            }
            if (settings.footstepLocomotion) {
                // Locomocao por pegadas (Etapa 3): o clipe e o estilo; os pes
                // apoiados do corpo nao escorregam.
                FootstepPlanner3D& footsteps = athlete.footsteps;
                if (!footsteps.ready()) footsteps.reset(profile, settings.footsteps);
                const AthleteLegs3D& legs = footsteps.legs();
                if (legs.valid && !footsteps.live()) {
                    // Comecando do corpo (a volta do fisico ou do Ativo).
                    std::array<Vec3, 2> position {};
                    std::array<Quaternion, 2> orientation {};
                    std::array<bool, 2> grounded {};
                    for (std::size_t side = 0; side < 2; ++side) {
                        const PhysicsBodyState3D& link = body.links[legs.foot[side]];
                        position[side] = link.position;
                        orientation[side] = link.orientation;
                        grounded[side] = footsteps.soleHeight(static_cast<int>(side), link.position, link.orientation)
                            < groundHeight(link.position) + 0.03f;
                    }
                    footsteps.seed(position, orientation, grounded);
                }
                if (legs.valid) {
                    FootstepStyle3D style;
                    for (std::size_t side = 0; side < 2; ++side) {
                        style.position[side] = pose.linkPositions[legs.foot[side]];
                        style.orientation[side] = pose.linkOrientations[legs.foot[side]];
                        style.planted[side] = out.cycling ? out.cyclePlanted[side] : out.footPlanted[side];
                        style.secondsToTouchdown[side] = out.cycleSecondsToTouchdown[side];
                        style.secondsToLiftoff[side] = out.secondsToLiftoff[side];
                        style.swingSeconds[side] = out.swingSeconds[side];
                    }
                    style.rootPosition = rootPosition;
                    style.rootOrientation = rootOrientation;
                    style.velocity = nav().velocity;
                    style.grounded = !out.airborne;
                    style.scripted = out.rootScripted || out.tackling || out.airTackle || out.crouchStarting;
                    style.movingShare = out.movingShare;
                    // (Plano v3: o lado de cada pe pelo rumo do corpo e o chao de
                    // verdade para travar.)
                    {
                        // (O rumo do quadril do estilo: no strafe a pelve gira
                        // para o lado do movimento.)
                        const Vec3 hips = (rootOrientation * profile.links.front().modelOrientation.conjugate())
                                              .rotate({ 1.0f, 0.0f, 0.0f });
                        style.facingYaw = hips.x * hips.x + hips.y * hips.y > 1e-6f ? std::atan2(hips.y, hips.x)
                                                                                     : out.facingYawRadians;
                    }
                    if (input.groundAt) {
                        const auto probe = input.groundAt;
                        style.groundAt = [probe](Vec3 point) {
                            const GroundProbeResult3D hit = probe(point);
                            return FootGround3D { hit.hasSurface, hit.pointWorld.z };
                        };
                    }
                    athlete.footTargets = footsteps.plan(style, settings.footsteps, deltaTime);
                    athlete.legSynthesis = synthesizeAthleteLegs3D(profile, legs, footsteps.geometry(),
                        athlete.footTargets, rootPosition, rootOrientation, coordinates, pose);
                }
            } else {
                athlete.footsteps.clear();
                // As pernas nunca se atravessam (plano v4: sem as pegadas, os
                // clipes cruzavam uma perna por dentro da outra nas trocas de
                // rumo).
                // (O pe no balanco contorna.)
                std::array<bool, 2> swinging {};
                for (std::size_t side = 0; side < 2; ++side)
                    swinging[side] = out.cycling ? !out.cyclePlanted[side] : !out.footPlanted[side];
                if (!athlete.legClearance.ready()) athlete.legClearance.reset(profile);
                if (settings.legClearance.enabled)
                    athlete.legClearance.apply(profile, coordinates, rootPosition, rootOrientation, pose, swinging,
                        deltaTime, settings.legClearance);
            }
            // (16/11, a volta pelo cerebro do RL: ver LegacyAthleteState3D::rlResidualPosition.) No primeiro tick do guiado, a diferenca de cada
            // elo do corpo para a pose; dai ela soma, sumindo por (1 + w t) e^(-w t) - sem degrau de velocidade, com a meia-vida da volta.
            if (state.rlResidualPending && pose.linkPositions.size() == body.links.size()) {
                state.rlResidualPending = false;
                state.rlResidualAge = 0.0f;
                state.rlResidualPosition.assign(body.links.size(), {});
                state.rlResidualRotation.assign(body.links.size(), {});
                for (std::size_t i = 0; i < body.links.size(); ++i) {
                    state.rlResidualPosition[i] = body.links[i].position - pose.linkPositions[i];
                    Quaternion delta = (body.links[i].orientation * pose.linkOrientations[i].conjugate()).normalized();
                    if (delta.w < 0.0f) delta = { -delta.x, -delta.y, -delta.z, -delta.w };
                    const Vec3 axis { delta.x, delta.y, delta.z };
                    const float sine = axis.length();
                    state.rlResidualRotation[i] = sine > 1.0e-7f ? axis * (2.0f * std::atan2(sine, delta.w) / sine) : Vec3 {};
                }
            }
            if (state.rlResidualAge >= 0.0f && state.rlResidualPosition.size() == pose.linkPositions.size()) {
                const float omega = 1.678f / std::max(0.01f, state.rlResidualHalfLife);
                const float x = omega * state.rlResidualAge;
                const float keep = (1.0f + x) * std::exp(-x);
                for (std::size_t i = 0; i < pose.linkPositions.size(); ++i) {
                    pose.linkPositions[i] = pose.linkPositions[i] + state.rlResidualPosition[i] * keep;
                    const Vec3 r = state.rlResidualRotation[i] * keep;
                    const float angle = r.length();
                    if (angle > 1.0e-7f) {
                        const float sine = std::sin(0.5f * angle) / angle;
                        const Quaternion q { r.x * sine, r.y * sine, r.z * sine, std::cos(0.5f * angle) };
                        pose.linkOrientations[i] = (q * pose.linkOrientations[i]).normalized();
                    }
                }
                state.rlResidualAge += deltaTime;
                if (keep < 1.0e-3f) state.rlResidualAge = -1.0f;
            }
            // (16/11, a volta pelo cerebro: a autoridade do guiado sobe de 35% a cheia em 0,25 s - entrando cheia, as maos, leves, tremiam:
            // 100 m/s2 caindo em 0,15 s.)
            std::vector<float> rlAuthority;
            if (state.rlResidualAge >= 0.0f && state.rlResidualAge < 0.25f) {
                const float x = std::clamp(state.rlResidualAge / 0.25f, 0.0f, 1.0f);
                rlAuthority.assign(profile.links.size(), 0.35f + 0.65f * x * x * (3.0f - 2.0f * x));
            }
            if (!rlAuthority.empty() && settings.physicalFeel != 2) {
                scene.setRagdollGuidePose(slot.handle, pose.linkPositions, pose.linkOrientations, rlAuthority);
            } else if (settings.physicalFeel == 2) {
                // O corpo fisico (28/10, item 10): a coluna e os bracos com a guia
                // mais solta carregam o proprio embalo; as pernas, a pelve, o
                // pescoco e a cabeca exatos (solta, a cabeca ia de p95 80 a 109
                // graus/s no corte - a regra e a cabeca calma).
                athlete.guideAuthority.assign(profile.links.size(), 1.0f);
                const float upper = std::clamp(settings.physicalUpperAuthority, 0.0f, 1.0f);
                for (std::size_t i = 1; i < profile.links.size(); ++i) {
                    const std::string& id = profile.links[i].id;
                    if (id == "Abdomen" || id == "Chest" || id == "UpperChest" || id.find("Arm") != std::string::npos
                        || id.find("Hand") != std::string::npos)
                        athlete.guideAuthority[i] = upper;
                }
                for (std::size_t i = 0; i < rlAuthority.size() && i < athlete.guideAuthority.size(); ++i)
                    athlete.guideAuthority[i] = std::min(athlete.guideAuthority[i], rlAuthority[i]);
                scene.setRagdollGuidePose(slot.handle, pose.linkPositions, pose.linkOrientations, athlete.guideAuthority);
            } else {
                scene.setRagdollGuidePose(slot.handle, pose.linkPositions, pose.linkOrientations);
            }
            state.previousAnimationPose = state.animationPose.linkPositions.empty() ? pose : state.animationPose;
            state.animationPose = pose;
            state.physicsBlend = 0.0f;
        }
        // A capsula manda: nada de ir atras do corpo.
        setFollow({});
        return;
    }
    // A rede do levantar junto da rede da locomocao (01/11, ele: no "Fisico com RL" do laboratorio, "a troca harmonica
    // entre os modelos de RL de locomocao e levantar-se"; vault 04_locomocao/plano_interligacao_b21_levantar_01_11.md):
    // com as duas, no fisico, na queda iminente - o fim do episodio do treino da locomocao (a pelve baixa, algo que nao
    // os pes no chao, o classico caido) ou o tronco tombando (GetUpPolicy3D) - ela dirige o corpo como no treino dela: os
    // alvos a cada GetUpPolicy3D::TicksPerDecision passos, sem a compensacao da gravidade e sem forcas de fora, a capsula
    // embaixo da pelve. De pe e firme por GetUpPolicy3D::FirmSeconds, o classico recomeca no corpo, os alvos dele saem
    // dos dela (getUpHandoff, abaixo) e a rede da locomocao volta do zero.
    {
        auto& rl = athlete.getUpRecovery;
        const GetUpPolicy3D* policy = assets.getUpPolicy;
        const bool allowed = policy != nullptr && settings.getUpPolicy && settings.locomotionPolicy
            && state.bodyMode == BodyMode::Physical && body.links.size() == profile.links.size();
        const auto release = [&](const char* why) {
            rl.active = false;
            rl.trunkTiltValid = false;
            state.locomotion.reset(profile, body);
            state.adaptivePhysics.reset(profile);
            Log::info(std::string("corpo: rede do levantar -> rede da locomocao (") + why + ")");
        };
        const bool programmedFall = settings.protectiveFallBeforeGetUp;
        if (!allowed) {
            if (rl.active) release("desligada");
            rl.trunkTiltValid = false;
        } else if (!rl.active && !athlete.animatedGetUp.active()) {
            const PhysicsBodyState3D& pelvis = body.links.front();
            // (Depois de soltar travada ou pelo tempo, so de novo com ele de pe.)
            if (rl.waitStand) {
                const float up = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
                    .rotate({ 0.0f, 0.0f, 1.0f }).z;
                if (pelvis.position.z - groundHeight(pelvis.position) > 0.85f * profile.standingRootHeightMeters && up > 0.85f)
                    rl.waitStand = false;
            }
            bool imminent = !rl.waitStand
                && pelvis.position.z - groundHeight(pelvis.position) < GetUpFallPelvisHeight * bodyScale3D(assets.body);
            for (const RagdollInteraction3D& contact : body.interactions) {
                if (contact.linkIndex >= profile.links.size() || contact.otherMotion != RagdollContactMotion3D::Static
                    || contact.normalWorld.z <= 0.5f)
                    continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (!rl.waitStand && id.find("Foot") == std::string::npos && id.find("Toe") == std::string::npos)
                    imminent = true;
            }
            if (++rl.tiltTicks % GetUpPolicy3D::TicksPerDecision == 0 && !rl.waitStand) {
                if (state.locomotion.telemetry().state == CharacterLocomotionState3D::Fallen) imminent = true;
                const float tilt = GetUpPolicy3D::trunkTilt(profile, body.links);
                const float decision = static_cast<float>(GetUpPolicy3D::TicksPerDecision) * deltaTime;
                if (!programmedFall && rl.trunkTiltValid && tilt > GetUpPolicy3D::ImminentTiltDegrees
                    && (tilt - rl.trunkTilt) / decision > GetUpPolicy3D::ImminentTiltRate)
                    imminent = true;
                rl.trunkTilt = tilt;
                rl.trunkTiltValid = true;
            }
            // A queda da locomocao fisica antes (settings.protectiveFallBeforeGetUp; 02/11, ele: "nao da para usar apenas a
            // mesma que a locomocao fisica ja usa?"): caindo e deitado, o classico cuida de tudo, do jeito dele; a rede
            // assume so quando ele comeca a levantar.
            if (programmedFall) imminent = !rl.waitStand
                && state.locomotion.telemetry().state == CharacterLocomotionState3D::GettingUp;
            if (imminent) {
                rl.active = true;
                rl.ticks = 0;
                rl.firmSeconds = 0.0f;
                rl.seconds = 0.0f;
                rl.stuckSeconds = 0.0f;
                rl.down = false;
                if (state.guided) {
                    scene.setRagdollGuided(slot.handle, false);
                    state.guided = false;
                }
                Log::info(programmedFall ? "corpo: classico comeca a levantar -> rede do levantar"
                                         : "corpo: queda iminente -> rede do levantar");
            }
        }
        if (rl.active) {
            // (Com o pedido de deslocamento - o analogico apertado -, depois do chao ela levanta virando para la e a troca
            // vem de pe por GetUpPolicy3D::MovingHandoffSeconds, mesmo se mexendo: a rede da locomocao ja sai correndo.)
            const Vec3 wanted { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
            const bool moving = rl.down && std::hypot(wanted.x, wanted.y) > GetUpPolicy3D::MovingRequestSpeed;
            try {
                if (rl.ticks % GetUpPolicy3D::TicksPerDecision == 0)
                    policy->decide(profile, body, rl.drives, moving ? wanted : Vec3 {}, groundHeight(body.links.front().position),
                        &scene, slot.handle);
            } catch (const std::exception& error) {
                Log::warn(std::string("rede do levantar: ") + error.what());
                rl.drives.clear();
            }
            if (rl.drives.empty()) {
                release("erro");
            } else {
                ++rl.ticks;
                rl.seconds += deltaTime;
                scene.setRagdollActiveDriveTargets(slot.handle, rl.drives, false);
                scene.setRagdollFrictionScale(slot.handle, 1.0f);
                const PhysicsBodyState3D& pelvis = body.links.front();
                placeNav({ pelvis.position.x, pelvis.position.y, groundHeight(pelvis.position) });
                setNavVelocity({});
                setFollow({});
                // De pe e firme (o criterio do treino dela): a pelve perto da altura de pe, em pe, e o centro de massa
                // quase parado.
                Vec3 momentum;
                float mass = 0.0f;
                for (std::size_t i = 0; i < profile.links.size(); ++i) {
                    momentum += body.links[i].linearVelocity * profile.links[i].massFraction;
                    mass += profile.links[i].massFraction;
                }
                const Vec3 com = mass > 0.0f ? momentum * (1.0f / mass) : Vec3 {};
                const float upright = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
                    .rotate({ 0.0f, 0.0f, 1.0f }).z;
                const float height = pelvis.position.z - groundHeight(pelvis.position);
                rl.down = rl.down || height < GetUpPolicy3D::DownRootFraction * profile.standingRootHeightMeters;
                const bool firm = height > 0.85f * profile.standingRootHeightMeters && upright > 0.85f
                    && (moving || std::hypot(com.x, com.y) < 0.5f);
                rl.firmSeconds = firm ? rl.firmSeconds + deltaTime : 0.0f;
                // (Travada - presa num contato, sem subir -: o classico tenta.)
                const bool rising = height > 0.85f * profile.standingRootHeightMeters;
                rl.stuckSeconds = !rising && std::hypot(com.x, com.y) < 0.15f && std::abs(com.z) < 0.15f
                    ? rl.stuckSeconds + deltaTime : 0.0f;
                const bool stuck = rl.stuckSeconds >= GetUpStuckSeconds;
                // (02/11, ele: no padrao, "assim que ele recem se ergue" ja volta a ele: erguido - a pelve na altura e
                // o tronco em pe - basta, sem firmar.)
                const bool risen = defaultMode && rl.down && height > 0.85f * profile.standingRootHeightMeters && upright > 0.85f;
                const float needed = moving ? GetUpPolicy3D::MovingHandoffSeconds : GetUpPolicy3D::FirmSeconds;
                if (!risen && rl.firmSeconds < needed && rl.seconds < GetUpRecoveryLimitSeconds && !stuck) return;
                const bool stood = risen || rl.firmSeconds >= needed;
                athlete.getUpHandoff = { GetUpPolicy3D::BlendSeconds, rl.drives };
                state.getUpSeen = true;
                rl.waitStand = !stood;
                release(stood ? "de pe" : stuck ? "travada" : "tempo");
            }
        }
    }
    // O levantar pelo padrao (28/10, item 16; Athlete/Body/AnimatedGetUp3D):
    // caido e assentado no fisico, de brucos ou de costas, com base solida - o
    // chao plano embaixo do caminho do clipe inteiro, nada estatico no caminho
    // e nenhuma bola perto (guiado, o corpo atravessaria tudo) -, o corpo
    // guiado segue o clipe de levantar do corpo de hoje. Um contato ou um
    // empurrao no meio volta ao fisico; no fim ele entrega ao padrao de pe (a
    // volta de sempre). De lado, o fisico rola antes.
    {
        AnimatedGetUp3D& getUp = athlete.animatedGetUp;
        const bool allowed = settings.animatedGetUp && defaultMode && state.bodyMode == BodyMode::Physical
            && body.links.size() == profile.links.size() && body.joints.size() == profile.links.size();
        // (A pose do tick no corpo guiado, com a capsula parada embaixo da pelve.)
        const auto drive = [&](float pace) {
            RagdollAnimationPose3D pose;
            getUp.step(profile, deltaTime * std::max(0.1f, settings.animatedGetUpSpeed) * pace, pose);
            scene.setRagdollGuidePose(slot.handle, pose.linkPositions, pose.linkOrientations);
            const Vec3 pelvisNow = pose.linkPositions.front();
            placeNav({ pelvisNow.x, pelvisNow.y, groundHeight(pelvisNow) });
            setNavVelocity({});
            setFollow({});
        };
        // (O que encosta fora o chao firme, e o empurrao de fora: guiado, o
        // corpo nao bate no chao, entao qualquer contato e de outra coisa.)
        constexpr float AbortImpulse = 1.0f;
        float touched = body.externalPushNewtonSeconds;
        for (const RagdollInteraction3D& contact : body.interactions)
            if (contact.otherMotion != RagdollContactMotion3D::Static || contact.normalWorld.z < 0.6f)
                touched += contact.normalImpulseNewtonSeconds;
        if (!allowed) {
            // (Desligado no meio - a chave, o padrao desligado: o corpo guiado
            // volta ao fisico como no contato, se ninguem mais o guia.)
            if (getUp.active() && state.bodyMode == BodyMode::Physical && state.guided) {
                scene.setRagdollGuided(slot.handle, false);
                state.guided = false;
                state.locomotion.reset(profile, body);
                state.adaptivePhysics.reset(profile);
                Log::info("corpo: levantar pelo padrao -> fisico (desligado)");
            }
            getUp.stop();
            athlete.animatedGetUpCalm = 0.0f;
        } else if (getUp.active()) {
            if (touched > AbortImpulse) {
                getUp.stop();
                scene.setRagdollGuided(slot.handle, false);
                state.guided = false;
                state.locomotion.reset(profile, body);
                state.adaptivePhysics.reset(profile);
                // (E espera 1 s antes de tentar de novo: comecando e saindo a
                // cada tick, ele nunca levantava.)
                athlete.animatedGetUpCalm = -1.0f;
                Log::info("corpo: levantar pelo padrao -> fisico (contato)");
            } else {
                // Com pressa (28/10, pedido do usuario: o levantar rapido com
                // arrancada pela locomocao padrao): com as teclas o clipe anda
                // AnimatedGetUpHurry vezes mais rapido; com o Shift, quase de pe
                // (alem de 3/4 do clipe e a pelve acima de 0,7 m), o padrao assume
                // ja na arrancada agachada para onde elas pedem. Antes ele levantava o clipe
                // inteiro e so entao andava. (Passando antes - a um terco e com a
                // pelve a 0,5 m, o ponto do fisico - o tronco ia a 1.100 m/s2.)
                constexpr float AnimatedGetUpHurry = 2.0f;
                const Vec3 keys = command.intent.requestedVelocityWorld;
                const bool hurried = keys.lengthSquared() > 0.25f;
                drive(hurried ? AnimatedGetUpHurry : 1.0f);
                const PhysicsBodyState3D& pelvisNow = body.links.front();
                const bool sprintHandoff = hurried && command.intent.effort > 0.5f && settings.defaultGetUpHandoff
                    && !getUp.finished() && getUp.timeSeconds() > 0.75f * getUp.durationSeconds()
                    && pelvisNow.position.z - groundHeight(pelvisNow.position) > 0.70f;
                if (getUp.finished() || sprintHandoff) {
                    // De pe: a volta de sempre ao padrao (a pose guiada indo a
                    // dele pela inercia, o padrao comecando do corpo).
                    getUp.stop();
                    const PhysicsBodyState3D& pelvis = body.links.front();
                    state.returnCoordinates.assign(profile.links.size(), {});
                    std::vector<Vec3> velocities(profile.links.size());
                    for (std::size_t i = 1; i < profile.links.size(); ++i) {
                        const auto& joint = body.joints[i].positionRadians;
                        const auto& rate = body.joints[i].velocityRadiansPerSecond;
                        state.returnCoordinates[i] = { joint[0], joint[1], joint[2] };
                        velocities[i] = { rate[0], rate[1], rate[2] };
                    }
                    if (looseNavigation && !piloted) {
                        state.guideFacingYawRadians = locomotionFrameYaw3D(profile, pelvis.orientation);
                        command.facingYawRadians = state.guideFacingYawRadians;
                    }
                    state.defaultLocomotion.reset(command.facingYawRadians);
                    DefaultLocomotionSeed3D seed;
                    const Vec3 forward = (pelvis.orientation * profile.links.front().modelOrientation.conjugate())
                        .rotate({ 1.0f, 0.0f, 0.0f });
                    seed.pelvisYawRadians = std::atan2(forward.y, forward.x);
                    seed.pelvisWorld = pelvis.position;
                    state.defaultLocomotion.seed(seed);
                    // (Mais devagar que a volta de sempre: o clipe acaba com a
                    // cabeca um pouco para tras, e com 0,14 s ela voltava a 955
                    // graus/s.)
                    constexpr float GetUpExitHalfLife = 0.25f;
                    constexpr float GetUpExitSeconds = 0.90f;
                    state.returnInertia.start(state.returnCoordinates, velocities, pelvis.position, pelvis.orientation,
                        {}, {}, sprintHandoff ? GetUpHandoffHalfLife : GetUpExitHalfLife,
                        sprintHandoff ? GetUpHandoffSeconds : GetUpExitSeconds);
                    if (sprintHandoff) state.defaultLocomotion.startCrouchSprint(std::atan2(keys.y, keys.x), true);
                    state.bodyMode = BodyMode::Returning;
                    state.bodyModeSeconds = 0.0f;
                    state.guidedExitLeft = 0.0f;
                    state.airSeconds = 0.0f;
                    athlete.animatedGetUpCalm = 0.0f;
                    Log::info(sprintHandoff ? "corpo: levantar pelo padrao -> arrancada agachada" : "corpo: levantar pelo padrao -> padrao");
                }
                return;
            }
        } else {
            // Deitado de brucos ou de costas (o peito a menos de 40 graus), o
            // fisico ainda sem levantar - ou no comeco do levantar, com a pelve
            // ainda perto do chao (com as teclas ele comeca logo) - e o corpo
            // parado, por um tempo.
            const CharacterLocomotionTelemetry3D& telemetry = state.locomotion.telemetry();
            const bool low = body.links.front().position.z - groundHeight(body.links.front().position) < 0.30f;
            const bool down = telemetry.state == CharacterLocomotionState3D::Fallen
                || (telemetry.state == CharacterLocomotionState3D::GettingUp
                    && (telemetry.getUpPhase == CharacterGetUpPhase3D::Settling || low));
            std::size_t chest = 0, head = 0;
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                if (profile.links[i].id == "Chest") chest = i;
                if (profile.links[i].id == "Head") head = i;
            }
            const Vec3 chestForward = (body.links[chest].orientation * profile.links[chest].modelOrientation.conjugate())
                .rotate({ 1.0f, 0.0f, 0.0f });
            const bool prone = chestForward.z < -0.766f, supine = chestForward.z > 0.766f;
            // (Parado e o nucleo - a pelve e o peito: assentando, o fisico ja
            // mexe os bracos para levantar, e eles entram pela inercia.)
            const float fastest = body.links[chest].linearVelocity.length();
            const bool still = body.links.front().linearVelocity.length() < 0.4f && fastest < 0.6f;
            // (Com algo encostando - o obstaculo que o derrubou, outro jogador
            // -, nao comeca.)
            if (!down || !(prone || supine) || !still || touched > 0.1f)
                athlete.animatedGetUpCalm = std::min(athlete.animatedGetUpCalm, 0.0f);
            athlete.animatedGetUpCalm += deltaTime;
            const AnimationClip3D* clip = prone ? clips.standUpFrontGuided : clips.standUpBackGuided;
            constexpr float CalmSeconds = 0.15f;
            if (down && (prone || supine) && still && touched <= 0.1f && athlete.animatedGetUpCalm > CalmSeconds
                && clip != nullptr) {
                std::vector<Vec3> coordinates(profile.links.size()), velocities(profile.links.size());
                for (std::size_t i = 1; i < profile.links.size(); ++i) {
                    const auto& joint = body.joints[i].positionRadians;
                    const auto& rate = body.joints[i].velocityRadiansPerSecond;
                    coordinates[i] = { joint[0], joint[1], joint[2] };
                    velocities[i] = { rate[0], rate[1], rate[2] };
                }
                const PhysicsBodyState3D& pelvis = body.links.front();
                AnimatedGetUp3D::Body lying;
                lying.coordinates = &coordinates;
                lying.coordinateVelocities = &velocities;
                lying.pelvisPosition = pelvis.position;
                lying.pelvisOrientation = pelvis.orientation;
                lying.pelvisVelocity = pelvis.linearVelocity;
                lying.pelvisAngularVelocity = pelvis.angularVelocity;
                lying.headPosition = body.links[head].position;
                lying.groundHeight = groundHeight(pelvis.position);
                getUp.start(profile, *clip, lying);
                // A base: nenhuma bola perto; e no caminho do clipe inteiro, o
                // chao plano embaixo de cada link e nada entre a pelve e ele.
                bool solid = true;
                for (const AthleteBall3D& ball : balls)
                    if (ball.state != nullptr && std::hypot(ball.state->position.x - pelvis.position.x,
                            ball.state->position.y - pelvis.position.y) < 1.2f)
                        solid = false;
                const Vec3 center = pelvis.position + Vec3 { 0.0f, 0.0f, 0.25f };
                RagdollAnimationPose3D sample;
                constexpr int Samples = 8;
                for (int k = 0; k <= Samples && solid; ++k) {
                    getUp.poseAt(profile, getUp.durationSeconds() * static_cast<float>(k) / Samples, sample);
                    for (const Vec3& point : sample.linkPositions) {
                        if (std::abs(groundHeight(point) - lying.groundHeight) > 0.04f) { solid = false; break; }
                        // (O raio passa 15 cm do centro do link: a grossura do
                        // membro e a folga - so ate o centro, a parede a 40 cm do
                        // lado passava e o braco batia nela.)
                        const Vec3 toward = point - center;
                        const float length = toward.length();
                        PhysicsRayHit3D hit;
                        // (E o que se move parado no caminho: a caixa que o derrubou.)
                        const Ray3D ray { center, toward * (1.0f / std::max(length, 1e-4f)) };
                        if (length > 0.05f
                            && ((scene.raycastStatic(ray, length + 0.15f, hit) && hit.normal.z < 0.7f)
                                || scene.raycastDynamic(ray, length + 0.15f, hit))) {
                            solid = false;
                            break;
                        }
                    }
                }
                if (solid) {
                    scene.setRagdollGuided(slot.handle, true);
                    state.guided = true;
                    Log::info(prone ? "corpo: levantar pelo padrao (de brucos)" : "corpo: levantar pelo padrao (de costas)");
                    drive(1.0f);
                    return;
                }
                // (Sem base: o fisico levanta; olha de novo daqui a 1 s.)
                getUp.stop();
                athlete.animatedGetUpCalm = -1.0f;
            }
        }
    }
    // A virada brusca (experimento do laboratorio): desligada, nao roda e o
    // atleta passa exato.
    if (settings.dynamicTurnMove && adaptive) {
        TurnMoveInput3D turn;
        turn.requestedVelocity = command.intent.requestedVelocityWorld;
        turn.bodyVelocity = root.linearVelocity;
        turn.deltaTime = deltaTime;
        state.turnMove.settings() = settings.dynamicTurnMoveSettings;
        state.turnMove.update(turn);
        if (state.turnMove.output().weight > 0.0f) {
            state.turnCorrections = command.corrections != nullptr ? *command.corrections : PhysicalCorrections3D {};
            state.turnCorrections.frame = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                locomotionFrameYaw3D(profile, root.orientation));
            // (A rede ganha sobre o professor por familia: sem rede, 1.)
            const std::array<float, 4>& gate = athlete.corrections.turnGate;
            addTurnMoveCorrections3D(state.turnCorrections, gateTurnMoveCorrections3D(state.turnMove.corrections(), gate));
            command.corrections = &state.turnCorrections;
            command.intent.requestedVelocityWorld = command.intent.requestedVelocityWorld
                * gateTurnMoveSpeedScale3D(state.turnMove.output().speedScale, gate[3]);
        }
    } else if (state.turnMove.output().state != TurnMoveState3D::Idle || state.turnMove.output().weight > 0.0f) {
        state.turnMove.reset();
    }
    {
        const auto begin = std::chrono::steady_clock::now();
        state.locomotion.update(profile, clips, body, dynamics, command, deltaTime);
        athlete.locomotionSeconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - begin).count();
    }
    const auto& locomotionOutput = state.locomotion.output();
    // Fisica adaptativa: com a opcao ligada, a forca limitada da pelve e
    // aplicada no lugar do transporte pela capsula; desligada, so mede.
    const CharacterLocomotionState3D bodyState = state.locomotion.telemetry().state;
    // (No carrinho tambem: o corpo vai pelo clipe e pela propria ajuda.)
    const bool down = bodyState == CharacterLocomotionState3D::Fallen
        || bodyState == CharacterLocomotionState3D::GettingUp
        || bodyState == CharacterLocomotionState3D::Tackling;
    AdaptivePhysicalIntent3D physicalIntent;
    physicalIntent.proxyVelocityWorld = command.proxyVelocityWorld;
    // Sem a velocidade pedida, o controlador interpretava toda marcha
    // como estado parado: legsWalking ficava falso e walkingAssist era
    // forçado a 1, ignorando completamente o slider "Ajuda andando".
    physicalIntent.requestedVelocityWorld = command.intent.requestedVelocityWorld;
    // Freada correndo: a velocidade do perfil e a parcela da ajuda de
    // quem corre (da locomocao).
    physicalIntent.brakeRunVelocityWorld = state.locomotion.telemetry().brakeRunVelocityWorld;
    physicalIntent.brakeRunShare = state.locomotion.telemetry().brakeRunShare;
    physicalIntent.navigationRootWorld = command.rootPositionWorld;
    physicalIntent.airborne = !command.grounded;
    physicalIntent.manipulated = command.manipulated;
    physicalIntent.suspended = down;
    physicalIntent.postureLost = state.locomotion.telemetry().postureLost;
    physicalIntent.braceRecovering = state.locomotion.telemetry().braceRecovering;
    physicalIntent.jumpCharge = command.controlled ? control.jumpCharge : 0.0f;
    physicalIntent.footGround = command.footGround;
    // O que a capsula (ou a ancora virtual) andou para ir atras do corpo
    // neste tick nao e intencao do jogador.
    physicalIntent.followVelocityWorld = command.controlled
        ? control.capsuleFollowVelocityWorld : state.navigationAnchorVelocity;
    physicalIntent.physicalState = &state.locomotion.physicalState();
    physicalIntent.contactPlan = &state.locomotion.contactPlan();
    physicalIntent.targetPose = &state.locomotion.output().targetPose;
    physicalIntent.footPlantedValid = true;
    physicalIntent.footPlanted = state.locomotion.telemetry().footPlanted;
    if (command.corrections != nullptr && command.corrections->balance) {
        physicalIntent.rlContact = true;
        physicalIntent.rlContactAccelerationWorld = contactAccelerationWorld3D(command.corrections);
    }
    {
        const auto begin = std::chrono::steady_clock::now();
        state.adaptivePhysics.update(profile, body, locomotionOutput.guide, physicalIntent, deltaTime);
        athlete.assistSeconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - begin).count();
    }
    // A referencia de estilo do fisico (o contrato do RL): o padrao a sombra,
    // pelo comando de agora (a capsula ou a virtual), so calculando a pose.
    // Caido, ela para; de pe de novo, recomeca no rumo pedido.
    athlete.referenceSeconds = 0.0f;
    // (Com a rede ligada, so quem ela pode dirigir - o controlado e os
    // comandados - le a sombra; os bonecos soltos nao pagam por ela.)
    // (Os bonecos soltos tambem, 02/11: a rede agora os dirige.)
    const bool policyReads = settings.locomotionPolicy;
    if ((settings.physicalReference || policyReads) && !animated && !down) {
        const auto begin = std::chrono::steady_clock::now();
        if (!state.referenceShadowActive) {
            // (Recomecando - depois de uma queda: o laco de fase tambem, sem
            // a fase de antes dela.)
            state.referenceShadow.reset(command.facingYawRadians);
            state.referencePhaseLock.reset();
        }
        configureDefaultLocomotion(state.referenceShadow.settings(), settings, assets.body);
        state.referenceShadow.settings().footBentRetry = false;
        // (O parado pela ultima passada, a arrancada pela base dele, o trote de
        // costas novo e as molas do corpo tambem: o treino e o de sempre - 28/10.)
        state.referenceShadow.settings().idleFromLastStride = false;
        state.referenceShadow.settings().startFromStance = false;
        state.referenceShadow.settings().backTrotLikeForward = false;
        state.referenceShadow.settings().bodyReaction = false;
        DefaultLocomotionInput3D input;
        input.feetPositionWorld = command.rootPositionWorld - Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
        input.velocityWorld = command.rootVelocityWorld;
        input.grounded = command.grounded;
        input.requestedVelocityWorld = command.intent.requestedVelocityWorld;
        input.facingYawRadians = command.facingYawRadians;
        input.lookPitchRadians = command.lookPitchRadians;
        input.groundAt = command.groundAt;
        input.deltaTime = deltaTime;
        input.motionVariety = command.motionVariety;
        input.varietySeed = command.varietySeed;
        {
            // A fase do corpo pelos pousos de verdade (o pe que encosta no
            // chao depois de 0,1 s no ar) e o relogio da sombra puxado ate
            // ela pelo ritmo (PhaseLock3D: 1 + 3 x o erro, de 0,5 a 1,5
            // vezes; so andando, com um pouso no ultimo segundo). (Medido:
            // o erro dos pes contra o estilo em cruzeiro caiu de 0,13-0,70 m
            // para 0,07-0,20; com ganho 1,5, 0,10-0,24.)
            static const char* const feet[2] = { "LeftFoot", "RightFoot" };
            std::array<bool, 2> touching { false, false };
            for (std::size_t side = 0; side < 2; ++side)
                for (const RagdollInteraction3D& item : body.interactions)
                    if (item.linkIndex < profile.links.size() && profile.links[item.linkIndex].id == feet[side]
                        && item.normalWorld.z > 0.5f)
                        touching[side] = true;
            const DefaultLocomotionOutput3D& shadow = state.referenceShadow.output();
            input.phaseRateScale = state.referencePhaseLock.update(touching, shadow.cyclePhase, shadow.movingShare,
                state.referenceShadowActive, settings.physicalReferencePhaseLock, deltaTime);
        }
        state.referenceShadow.update(profile, defaultClips, input);
        state.referenceShadowActive = state.referenceShadow.output().valid;
        athlete.referenceSeconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - begin).count();
    } else {
        state.referenceShadowActive = false;
    }
    const float followSlack = 0.10f * (1.0f - std::clamp(state.adaptivePhysics.output().perturbation, 0.0f, 1.0f));
    if (adaptive && !command.controlled) {
        // A capsula virtual vai atras do corpo; caido, ela fica nele.
        // Comparada com ELA MESMA: no levantar a referencia da locomocao
        // fica presa ao ponto onde o corpo deitou, e comparando com ela a
        // ancora acumulava metros de erro - na saida, o boneco era
        // arremessado atras dela.
        if (commanded && down) {
            // Comandado, caido (levantando, no carrinho): a ancora fica no
            // corpo, como a capsula do jogador; no carrinho o plano leva o
            // embalo do corpo.
            const Vec3 pelvis = body.links.front().position;
            state.navigationAnchor = { pelvis.x, pelvis.y, state.navigationAnchor.z };
            state.navigationAnchorVelocity = {};
            const Vec3 velocity = body.links.front().linearVelocity;
            athlete.commandPlanVelocity = bodyState == CharacterLocomotionState3D::Tackling
                ? Vec3 { velocity.x, velocity.y, 0.0f } : Vec3 {};
        } else {
            // Comandado de pe: vai atras do corpo comparando com a pelve de
            // referencia da locomocao, como a capsula do jogador (com a
            // propria ancora, o corpo corria atras dela: o sprint andava 9%
            // menos).
            state.navigationAnchorVelocity = navigationFollowVelocity3D(body.links.front().position,
                commanded ? locomotionOutput.navigationAnchorWorld : state.navigationAnchor,
                down ? 0.0f : followSlack);
            // (O plano do comandado ja andou, antes do passo.)
            state.navigationAnchor += state.navigationAnchorVelocity * deltaTime;
        }
    }
    if (command.controlled || looseNavigation) {
        // A capsula vai atras do corpo quando ele se afasta da pelve de
        // referencia mais de 10 cm (empurrao, tropeco, parede): o corpo
        // e a verdade; depois de um contato externo, sem folga. Com a
        // opcao desligada o corpo e carregado e isso nunca acontece.
        // (E a virtual do boneco solto, 02/11: parada no lugar, empurrado ele era puxado de volta a ela.)
        setFollow(adaptive && !down
            ? navigationFollowVelocity3D(body.links.front().position,
                locomotionOutput.navigationAnchorWorld, followSlack)
            : Vec3 {});
    }
    state.previousAnimationPose = state.animationPose.linkPositions.empty()
        ? locomotionOutput.targetPose : state.animationPose;
    state.animationPose = locomotionOutput.targetPose;
    state.physicsBlend = locomotionOutput.physicsBlend;
    // While the body is down, the capsule goes where the body is. It is
    // what the camera follows, and leaving it standing where the fall
    // started is what made the character snap back across the floor the
    // moment a recovery finished.
    if ((command.controlled && scene.hasCharacter()) || looseNavigation) {
        const CharacterLocomotionState3D locomotionState = state.locomotion.telemetry().state;
        if (locomotionState == CharacterLocomotionState3D::Fallen
            || locomotionState == CharacterLocomotionState3D::GettingUp
            || locomotionState == CharacterLocomotionState3D::Tackling) {
            const Vec3 rootPosition = body.links.front().position;
            placeNav({ rootPosition.x, rootPosition.y, groundHeight(rootPosition) });
            // No carrinho a capsula vai com o embalo do corpo: ao sair
            // dele (correndo), ela continua com essa velocidade.
            if (locomotionState == CharacterLocomotionState3D::Tackling) {
                const Vec3 velocity = body.links.front().linearVelocity;
                setNavVelocity({ velocity.x, velocity.y, 0.0f });
            }
        }
    }
    // A guiagem de ossos (experimento do usuario, 11/10): no Fisico total, a
    // pose do modo padrao roda junto e os ossos vao ate ela nesta fracao (0: o
    // fisico de hoje; 100: como o padrao). Caido, levantando e no carrinho,
    // nada.
    if (command.controlled && !settings.defaultLocomotion && settings.physicalBoneGuidePercent > 0.0f && !down) {
        const DefaultLocomotionInput3D input = defaultLocomotionInput();
        state.defaultLocomotion.update(profile, defaultClips, input);
        const DefaultLocomotionOutput3D& out = state.defaultLocomotion.output();
        if (out.valid && out.pose.linkPositions.size() == profile.links.size()) {
            const std::vector<float> authority(profile.links.size(),
                std::clamp(settings.physicalBoneGuidePercent * 0.01f, 0.0f, 1.0f));
            scene.setRagdollGuidePose(slot.handle, out.pose.linkPositions, out.pose.linkOrientations, authority);
        }
    }
    // A entrega da rede do levantar (01/11, a interligacao das redes no treino da locomocao): por
    // GetUpPolicy3D::BlendSeconds os alvos do classico saem dos dela - a posicao, a rigidez, o amortecimento e a
    // compensacao da gravidade.
    const CharacterLocomotionOutput3D* applied = &locomotionOutput;
    CharacterLocomotionOutput3D blended;
    auto& handoff = athlete.getUpHandoff;
    if (handoff.blendLeft > 0.0f) {
        blended = locomotionOutput;
        const float w = std::clamp(handoff.blendLeft / GetUpPolicy3D::BlendSeconds, 0.0f, 1.0f);
        for (RagdollDriveTarget3D& drive : blended.driveTargets)
            for (const RagdollDriveTarget3D& from : handoff.drives)
                if (from.linkIndex == drive.linkIndex && from.axis == drive.axis) {
                    drive.positionRadians += (from.positionRadians - drive.positionRadians) * w;
                    drive.velocityRadiansPerSecond *= 1.0f - w;
                    drive.stiffnessScale += (from.stiffnessScale - drive.stiffnessScale) * w;
                    drive.dampingScale += (from.dampingScale - drive.dampingScale) * w;
                    drive.gravityCompensationScale *= 1.0f - w;
                    break;
                }
        handoff.blendLeft = std::max(0.0f, handoff.blendLeft - deltaTime);
        applied = &blended;
    }
    applyCharacterControl3D(scene, slot.handle, *applied, adaptive ? &state.adaptivePhysics.output() : nullptr,
        down);
    {
        // A medida da ajuda auxiliar (as mesmas condicoes da aplicacao acima).
        AthleteExternalAssist3D& measure = athlete.externalAssist;
        Vec3 torque {};
        if (adaptive && !down && state.adaptivePhysics.output().valid) {
            const AdaptivePhysicalOutput3D& a = state.adaptivePhysics.output();
            measure.pelvisNewtons += a.rootForceWorld.length();
            measure.pelvisVerticalNewtons += std::abs(a.rootForceWorld.z);
            measure.pelvisPlanarNewtons += std::hypot(a.rootForceWorld.x, a.rootForceWorld.y);
            measure.correctionTorque = a.correctionLeverWorld.length();
            measure.postureCoupleTorque = (a.rootTorqueWorld - a.correctionLeverWorld).length();
            torque += a.rootTorqueWorld;
            for (std::size_t side = 0; side < 2; ++side) {
                if (a.footLinkIndex[side] != RagdollDynamics3D::InvalidIndex) {
                    measure.feetNewtons += a.footForceWorld[side].length();
                    torque += a.footTorqueWorld[side];
                }
                if (a.handLinkIndex[side] != RagdollDynamics3D::InvalidIndex) torque += a.handTorqueWorld[side];
            }
            for (const Vec3& link : a.linkTorqueWorld) torque += link;
        } else if (!(adaptive && !down)) {
            Vec3 rootTorque = locomotionOutput.rootControlTorqueWorld;
            const float rootForce = locomotionOutput.rootControlForceWorld.length();
            if (bodyState == CharacterLocomotionState3D::Tackling) {
                measure.tackleNewtons += rootForce;
                measure.tackleTorque = rootTorque.length();
                rootTorque = {};
            } else {
                measure.getUpNewtons += rootForce;
            }
            torque += rootTorque;
        }
        if (locomotionOutput.chestAssistLink != RagdollDynamics3D::InvalidIndex) {
            measure.getUpNewtons += locomotionOutput.chestAssistForceWorld.length();
            torque += locomotionOutput.chestAssistTorqueWorld;
        }
        const std::size_t reactions = std::min(locomotionOutput.assistReactionLinks.size(),
            locomotionOutput.assistReactionTorquesWorld.size());
        for (std::size_t i = 0; i < reactions; ++i) torque += locomotionOutput.assistReactionTorquesWorld[i];
        measure.torqueWorld = torque;
        measure.netTorque = torque.length();
        measure.handover = measure.handover || state.returnBoost > 0.0f;
    }
    // A ajuda nos pes do corpo fisico (PhysicalFeetAssist3D): a forca que leva
    // o corpo a referencia passa pelos pes apoiados, no que a pelve nao deu.
    // Ela acompanha a ajuda andando e parado: cai com o impacto (o nivel do
    // ImpactAssist3D e o envelope de contato da fisica) e sobe na volta.
    // (Tambem no comandado sem capsula: sem ela, o sprint andava 9% menos.)
    athlete.feetAssistOutput = {};
    if ((command.controlled || commanded) && adaptive && !down && settings.physicalFeetAssistPercent > 0.0f
        && state.adaptivePhysics.output().valid) {
        const AdaptivePhysicalOutput3D& assist = state.adaptivePhysics.output();
        const RagdollAnimationConstraint3D& reference = locomotionOutput.guide;
        PhysicalFeetAssistInput3D feet;
        float share = std::clamp(settings.physicalFeetAssistPercent * 0.01f, 0.0f, 1.0f) * state.impactAssist.level();
        if (state.returnBoost > 0.0f) {
            const float boost = state.returnBoost * state.returnBoost * (3.0f - 2.0f * state.returnBoost);
            share += (1.0f - share) * boost;
        }
        feet.share = share * std::clamp(assist.assistEnvelope, 0.0f, 1.0f);
        feet.referencePosition = reference.rootPositionWorld;
        feet.referenceOrientation = reference.rootOrientationWorld;
        feet.referenceVelocity = reference.rootLinearVelocityWorld;
        feet.referenceAngularVelocity = reference.rootAngularVelocityWorld;
        feet.pelvisForce = assist.rootForceWorld;
        feet.pelvisTorque = assist.rootTorqueWorld;
        const BodyObservation3D observation = athlete.observer.observe(profile, body, deltaTime);
        athlete.feetAssistOutput = physicalFeetAssist3D(profile, body, observation, feet, settings.physicalFeetAssist);
        Vec3 feetTorque {};
        for (const LinkWrench3D& wrench : athlete.feetAssistOutput.wrenches) {
            scene.applyRagdollControlLinkForce(slot.handle, wrench.link, wrench.force, wrench.torque);
            athlete.externalAssist.feetNewtons += wrench.force.length();
            feetTorque += wrench.torque;
        }
        athlete.externalAssist.torqueWorld += feetTorque;
        athlete.externalAssist.netTorque = athlete.externalAssist.torqueWorld.length();
    }
}

} // namespace MatterEngine
