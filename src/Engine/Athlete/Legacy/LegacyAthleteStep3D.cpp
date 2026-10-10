#include "Engine/Athlete/Legacy/LegacyAthleteStep3D.hpp"

#include "Engine/Athlete/Legacy/LegacyBallBridge3D.hpp"
#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"

#include "Engine/Character/CharacterControlApplication3D.hpp"
#include "Engine/Core/Log.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr float ActiveExitHalfLife = 0.10f;
constexpr float ActiveExitSeconds = 0.40f;

// No Ativo, o ponto de captura alem disto (m) fora da base leva ao fisico
// pleno: sem os passos de equilibrio, a pancada que pede passo; com eles, so
// a que um passo nao segura (ou depois dos passos que cabem).
float activeCaptureLimit(const AthleteRuntimeSettings3D& settings, const ActiveStance3D& stance) {
    if (!settings.activeStanceSettings.balanceSteps) return settings.activeHandoverCapture;
    if (!stance.stepping() && stance.stepsTaken() >= settings.activeStanceSettings.maximumSteps)
        return settings.activeHandoverCapture;
    return settings.activeHandoverCaptureStepping;
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
    LegacyAthleteState3D& state = athlete.legacy;
    if (!athlete.observer.ready()) athlete.observer.prepare(profile);
    const auto groundHeight = [&](Vec3 position) { return athleteGroundHeight3D(scene, profile, position); };
    {
        // Levou carrinho: pancada de algo que se move e uma perna de quem
        // da o carrinho a menos de 45 cm de um link dele.
        float impulse = 0.0f;
        for (const RagdollInteraction3D& contact : body.interactions)
            if (contact.otherMotion != RagdollContactMotion3D::Static) impulse += contact.normalImpulseNewtonSeconds;
        bool tackled = false;
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
        state.impactAssist.update(impactImpulse3D(body), tackled && settings.tackledAssistLow, impulse, deltaTime);
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
    command.rootPositionWorld = { root.position.x, root.position.y,
        groundHeight(root.position) + profile.standingRootHeightMeters };
    command.rootVelocityWorld = root.linearVelocity;
    command.facingYawRadians = state.guideFacingYawRadians;
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
    command.manipulated = grabbed;
    if (!command.controlled && settings.adaptivePhysics) {
        // Capsula virtual dos bonecos nao controlados (modo fisico).
        if (!state.navigationAnchorValid) {
            state.navigationAnchor = root.position;
            state.navigationAnchorVelocity = {};
            state.navigationAnchorValid = true;
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
    if (adaptive) {
        // As juntas perto de uma pancada cedem (do tick anterior).
        command.jointStrength = state.adaptivePhysics.output().jointStrength;
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
        physical.legsAssistRetained *= level;
        physical.walkingAssistScale *= level;
    }
    if (state.returnBoost > 0.0f) {
        const float boost = state.returnBoost * state.returnBoost * (3.0f - 2.0f * state.returnBoost);
        auto& physical = state.adaptivePhysics.settings();
        physical.legsAssistRetained += (1.0f - physical.legsAssistRetained) * boost;
        physical.walkingAssistScale += (1.0f - physical.walkingAssistScale) * boost;
    }
    // Cambaleio (do tick anterior): a ajuda cai com o tonus e volta com
    // ele; amolecido, ele nao segue o comando de andar (volta junto).
    command.staggerRecovery = settings.staggerRecovery;
    {
        // (So no fisico: no padrao a locomocao fisica nao roda e o valor
        // ficaria parado no ultimo.)
        state.requestedVelocityRaw = command.intent.requestedVelocityWorld;
        const float stagger = state.bodyMode == LegacyBodyMode3D::Physical
            ? state.locomotion.telemetry().stagger : 0.0f;
        // (A ajuda nao cai mais pelo cambaleio: cai pelo impacto, acima.)
        if (stagger > 0.0f)
            command.intent.requestedVelocityWorld = command.intent.requestedVelocityWorld * (1.0f - stagger);
        // O chute correndo freia um pouco (a escala do manejo, do tick
        // anterior; rework de 06/10).
        if (command.controlled)
            command.intent.requestedVelocityWorld = command.intent.requestedVelocityWorld * state.ballSpeedScale;
    }
    command.balanceFootPlacement = e5 && settings.e5FootPlacement;
    command.walkingAssist = state.adaptivePhysics.settings().walkingAssistScale;
    command.standingAssist = state.adaptivePhysics.settings().legsAssistRetained;
    command.getUpAssist = std::clamp(settings.getUpAssistPercent * 0.01f, 0.0f, 2.0f);
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
    command.tackleEnabled = settings.tackleEnabled;
    // A voadora tambem no fisico (06/10), pela mesma opcao do padrao.
    command.airTackleEnabled = settings.defaultAirTackle;
    command.motionVariety = settings.motionVariety ? settings.motionVarietyPercent * 0.01f : 0.0f;
    command.footSpring = settings.footSpring;
    command.ankleBalance = settings.ankleBalance;
    command.varietySeed = static_cast<std::uint32_t>(slot.entityId * 0x9E3779B97F4A7C15ull >> 32);
    // Locomocao padrao: o modo do corpo do personagem controlado.
    using BodyMode = LegacyBodyMode3D;
    const bool defaultMode = settings.defaultLocomotion && command.controlled && adaptive && scene.hasCharacter();
    const auto toPhysical = [&](const char* why) {
            if (state.bodyMode != BodyMode::Physical) Log::info(std::string("corpo: modo fisico (") + why + ")");
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
                state.locomotion.reset(profile, body);
                state.adaptivePhysics.reset(profile);
                // (Do pulo, ou de um esbarrao no ar: o pouso e fisico e a
                // volta vem ~1 s depois dele.)
                state.physicalFromJump = std::string(why) == "pulo" || !scene.characterState().grounded;
                if (settings.defaultSeedFromBody) {
                    // O fisico continua a marcha de onde o padrao estava
                    // (fase pelos pes, sem a arrancada).
                    CharacterLocomotionSeed3D seed;
                    const Vec3 velocity = scene.characterState().velocity;
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
        state.bodyMode = BodyMode::Active;
        state.bodyModeSeconds = 0.0f;
        state.calmSeconds = 0.0f;
    };
    {
        if (!defaultMode) {
            toPhysical("padrao desligado");
        } else {
            const PhysicsCharacterState3D& capsule = scene.characterState();
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
            float wallImpulse = 0.0f;
            for (const RagdollInteraction3D& contact : body.interactions)
                if (contact.otherMotion != RagdollContactMotion3D::Dynamic && contact.normalWorld.z < 0.6f)
                    wallImpulse += contact.normalImpulseNewtonSeconds;
            constexpr float WallHitImpulse = 2.5f;
            const bool wallHit = state.bodyMode != BodyMode::Physical
                && wallImpulse > (defaultTackling ? TackleHitImpulse : WallHitImpulse);
            state.bodyModeSeconds += deltaTime;
            if (state.bodyMode == BodyMode::Physical) {
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
                const bool calm = standingState && upright && capsule.grounded && !hit && !againstWall
                    && state.locomotion.telemetry().stagger < 0.1f
                    && state.adaptivePhysics.output().perturbation < (state.physicalFromJump ? 0.6f : 0.25f);
                state.calmSeconds = calm ? state.calmSeconds + deltaTime : 0.0f;
                // (Do pulo: ~1 s de fisica depois do pouso - pedido do
                // usuario, 05/10 -, contado do toque, nao da decolagem.)
                state.landedSeconds = capsule.grounded ? state.landedSeconds + deltaTime : 0.0f;
                // (Levantou com pressa - as teclas: volta logo, ele sai
                // andando/correndo no padrao.)
                const bool hurried = state.locomotion.telemetry().getUpHurried;
                const float minimumSeconds = state.physicalFromJump || hurried ? 0.0f : 0.8f;
                const float calmSeconds = hurried ? 0.08f : state.physicalFromJump ? 0.15f : 0.4f;
                const bool landedLongEnough = !state.physicalFromJump || state.landedSeconds > 1.0f;
                // Pronto para voltar: a ajuda sobe ate a cheia em
                // ReturnBoostSeconds e so entao ele volta (se deixar de
                // estar assentado no meio, a ajuda desce de novo).
                const float ReturnBoostSeconds = hurried ? 0.15f : 0.4f;
                // (E com a ajuda de volta inteira depois do impacto - 06/10.)
                const bool ready = state.bodyModeSeconds > minimumSeconds
                    && state.calmSeconds > calmSeconds && landedLongEnough
                    && !state.impactAssist.recovering();
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
                if (handoff || (ready && state.returnBoost >= 1.0f)) {
                    state.returnBoost = 0.0f;
                    // A volta: a pose fisica de agora (raiz em relacao a base
                    // da capsula, juntas) misturada ate a animada.
                    const Vec3 feet = command.rootPositionWorld
                        - Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
                    state.returnRootOffset = pelvis.position - feet;
                    state.returnRootOrientation = pelvis.orientation;
                    state.returnCoordinates.assign(profile.links.size(), {});
                    for (std::size_t i = 1; i < state.returnCoordinates.size() && i < body.joints.size(); ++i) {
                        const auto& joint = body.joints[i].positionRadians;
                        state.returnCoordinates[i] = { joint[0], joint[1], joint[2] };
                    }
                    state.defaultLocomotion.reset(control.facingYawRadians);
                    if (settings.defaultSeedFromBody) {
                        // O padrao comeca de onde o corpo esta: pelve no
                        // rumo dela, na velocidade e na fase dos pes.
                        DefaultLocomotionSeed3D seed;
                        const Vec3 forward = (pelvis.orientation
                            * profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                        seed.pelvisYawRadians = std::atan2(forward.y, forward.x);
                        seed.velocityWorld = capsule.velocity;
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
                            }
                        state.defaultLocomotion.seed(seed);
                        // E a pose sai pela inercia: a diferenca para a
                        // animacao, com a velocidade que o corpo tinha.
                        std::vector<Vec3> velocities(state.returnCoordinates.size());
                        for (std::size_t i = 1; i < velocities.size() && i < body.joints.size(); ++i) {
                            const auto& joint = body.joints[i].velocityRadiansPerSecond;
                            velocities[i] = { joint[0], joint[1], joint[2] };
                        }
                        state.returnInertia.start(state.returnCoordinates, velocities, pelvis.position,
                            pelvis.orientation, pelvis.linearVelocity, pelvis.angularVelocity,
                            handoff ? GetUpHandoffHalfLife : 0.10f, handoff ? GetUpHandoffSeconds : 0.45f);
                    } else {
                        state.returnInertia.stop();
                    }
                    if (settings.activeBody) {
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
                        scene.placeCharacter({ base.x, base.y, groundHeight(pelvis.position) }, assets.avatarSettings);
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
                            : control.facingYawRadians;
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
                const bool wall = control.capsuleWallImpact > WallImpact;
                bool penetrating = false;
                {
                    std::vector<Vec3> positions(body.links.size());
                    std::vector<Quaternion> orientations(body.links.size());
                    for (std::size_t i = 0; i < positions.size(); ++i) {
                        positions[i] = body.links[i].position;
                        orientations[i] = body.links[i].orientation;
                    }
                    // (No carrinho, qualquer encostada: 5 mm.)
                    const float PenetrationDepth = defaultTackling ? 0.005f : 0.015f;
                    for (const RagdollWorldContact3D& contact : scene.ragdollPoseWorldContacts(
                            slot.handle, positions, orientations, PenetrationDepth))
                        // (O chao - normal para cima - nao conta.)
                        if (contact.normalWorld.z < 0.6f) penetrating = true;
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
                if (!settings.activeBody) {
                    if (hit || tackleHit || airTackle || wall || wallHit || penetrating || jumped)
                        toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : hit || tackleHit ? "pancada"
                            : wall || wallHit ? "parede" : penetrating ? "membro no mundo" : jumped ? "pulo" : "pancada");
                } else if (state.bodyMode == BodyMode::Active) {
                    // No Ativo o corpo ja e fisico (encostar na parede e
                    // contato de verdade): so o que derruba leva ao pleno.
                    if (grabbed || tackleHit || airTackle || hitImpulse > settings.fullPhysicalImpulse)
                        toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : "pancada forte");
                } else if (grabbed || tackleHit || airTackle || jumped || hitImpulse > settings.fullPhysicalImpulse
                    || control.capsuleWallImpact > settings.fullPhysicalWallImpact) {
                    toPhysical(grabbed ? "agarrado" : airTackle ? "voadora" : tackleHit ? "pancada"
                        : jumped ? "pulo" : hitImpulse > settings.fullPhysicalImpulse ? "pancada forte" : "parede");
                } else if (hitImpulse > settings.activeEnterImpulse || wall || wallHit || penetrating) {
                    toActive(hitImpulse > settings.activeEnterImpulse ? "encostao"
                        : penetrating ? "membro no mundo" : "parede");
                }
            }
        }
    }
    const bool animated = defaultMode
        && (state.bodyMode == BodyMode::Animated || state.bodyMode == BodyMode::Returning);
    const bool active = defaultMode && state.bodyMode == BodyMode::Active;
    if (command.controlled) {
        // (No Ativo a capsula segue como no padrao: a mesma tabela de
        // velocidades; ela vai atras do corpo quando ele se afasta.)
        output.controlledAnimatedSet = true;
        output.controlledAnimated = animated || active;
    }
    // Manejo novo (rework de 10/10/2026, Etapa 5): o pedido trava a bola, o
    // planejador escolhe pe, superficie e instante, o gesto leva o pe ate la
    // e o sensor do pe na bola da o evento unico - o resultado aprovado como
    // impulso, sem impor a velocidade nem segurar a bola.
    if (command.controlled && !body.links.empty() && settings.newBallHandlingActive()) {
        BallPlay3D& handling = athlete.ballPlay;
        if (!handling.ready()) handling.reset(profile);
        const Vec3 pelvis = body.links.front().position;
        // A bola do pedido em andamento; para um pedido novo, a mais perto
        // (ate 3 m).
        const std::uint64_t locked = handling.targetBallKey();
        AthleteBall3D* target = nullptr;
        AthleteBall3D* nearest = nullptr;
        float best = 3.0f;
        for (AthleteBall3D& ball : balls) {
            if (!scene.contains(ball.handle) || ball.state == nullptr) continue;
            if (locked != 0 && athleteBallKey3D(ball.handle) == locked) target = &ball;
            const float distance = (ball.state->position - pelvis).length();
            if (distance < best) { best = distance; nearest = &ball; }
        }
        BallHandlingInput3D in;
        in.request = control.ballRequest != BallActionKind3D::None && nearest != nullptr;
        if (in.request) {
            const float look = command.facingYawRadians;
            const Vec3 keys = command.intent.requestedDirectionWorld;
            in.intent.kind = control.ballRequest;
            in.intent.power = control.ballRequestPower;
            in.intent.direction = keys.lengthSquared() > 1e-4f ? keys : Vec3 { std::cos(look), std::sin(look), 0.0f };
            in.intent.aimYawRadians = control.ballRequest == BallActionKind3D::Shot ? control.ballRequestAimYaw : look;
            in.intent.lift = control.ballRequestLift;
            in.intent.curve = control.ballRequestCurve;
            in.requestBallKey = athleteBallKey3D(nearest->handle);
            if (target == nullptr && locked == 0) target = nearest;
        }
        control.ballRequest = BallActionKind3D::None;
        output.ballRequestConsumed = true;
        if (target == nullptr && in.request) target = nearest;
        if (target != nullptr) {
            const InflatedBallProfile3D ballProfile = scene.inflatedBallProfile(target->handle);
            in.ball.valid = true;
            in.ball.key = athleteBallKey3D(target->handle);
            in.ball.position = target->state->position;
            in.ball.velocity = target->state->linearVelocity;
            in.ball.spin = target->state->angularVelocity;
            in.ball.radius = ballProfile.radiusMeters;
            in.ball.mass = ballProfile.massKg;
            in.ball.sensor = target->state->sensorTouch;
        }
        // O corpo: a pose do tick anterior (a que o corpo seguiu), os pes
        // onde as pegadas os puseram.
        const RagdollAnimationPose3D& last = state.animationPose;
        const AthleteLegs3D& legs = athlete.footsteps.ready() ? athlete.footsteps.legs() : resolveAthleteLegs3D(profile);
        const bool havePose = last.linkPositions.size() == profile.links.size() && legs.valid;
        const Vec3 capsuleVelocity = scene.characterState().velocity;
        in.body.pelvis = havePose ? last.linkPositions.front() : pelvis;
        in.body.facingYaw = state.defaultLocomotion.output().valid ? state.defaultLocomotion.output().facingYawRadians
            : control.facingYawRadians;
        in.body.velocity = { capsuleVelocity.x, capsuleVelocity.y, 0.0f };
        in.body.groundHeight = groundHeight(pelvis);
        in.body.moving = std::hypot(capsuleVelocity.x, capsuleVelocity.y) > 0.5f;
        if (legs.valid) {
            const Quaternion pelvisOrientation = havePose ? last.linkOrientations.front() : body.links.front().orientation;
            for (std::size_t side = 0; side < 2; ++side) {
                in.body.hip[side] = in.body.pelvis + pelvisOrientation.rotate(legs.hipInPelvis[side]);
                const std::size_t foot = legs.foot[side];
                if (athlete.footsteps.live()) {
                    in.body.foot[side] = athlete.footTargets[side].position;
                    in.body.footOrientation[side] = athlete.footTargets[side].orientation;
                } else if (havePose) {
                    in.body.foot[side] = last.linkPositions[foot];
                    in.body.footOrientation[side] = last.linkOrientations[foot];
                } else {
                    in.body.foot[side] = body.links[foot].position;
                    in.body.footOrientation[side] = body.links[foot].orientation;
                }
                in.body.footVelocity[side] = body.links[foot].linearVelocity;
                if (havePose) {
                    const std::size_t thigh = legs.thigh[side];
                    in.body.knee[side] = (last.linkOrientations[thigh] * profile.links[thigh].modelOrientation.conjugate())
                                             .rotate({ 1.0f, 0.0f, 0.0f });
                }
                in.footLinks[side] = static_cast<std::uint32_t>(foot);
            }
        }
        // O centro de massa da pose anterior (o equilibrio no apoio).
        if (havePose) {
            Vec3 sum;
            float mass = 0.0f;
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                const float m = std::max(0.0f, profile.links[i].massFraction);
                sum += (last.linkPositions[i] + last.linkOrientations[i].rotate(profile.links[i].centerOfMassLocal)) * m;
                mass += m;
            }
            in.centerOfMass = mass > 0.0f ? sum * (1.0f / mass) : in.body.pelvis;
        } else {
            in.centerOfMass = pelvis;
        }
        // O ritmo da passada (o estilo do tick anterior).
        {
            const DefaultLocomotionOutput3D& style = state.defaultLocomotion.output();
            in.gait.cycling = style.valid && style.cycling && !style.airborne;
            in.gait.movingShare = style.valid ? style.movingShare : 0.0f;
            in.gait.planted = style.cyclePlanted;
            in.gait.secondsToLiftoff = style.secondsToLiftoff;
            in.gait.secondsToTouchdown = style.cycleSecondsToTouchdown;
            in.gait.swingSeconds = style.swingSeconds;
            in.gait.styleFoot = style.styleFootPosition;
            in.gait.styleFootOrientation = style.styleFootOrientation;
        }
        in.bodyVelocity = in.body.velocity;
        in.aheadVelocity = { command.intent.requestedVelocityWorld.x, command.intent.requestedVelocityWorld.y, 0.0f };
        in.requestedSpeed = std::hypot(state.requestedVelocityRaw.x, state.requestedVelocityRaw.y);
        // (No Ativo, so com o corpo assentado: o centro de massa andando como
        // o jogador pede. E o peso passa ao apoio antes do pe sair do chao:
        // no corpo fisico isso importa.)
        in.physicalBody = state.bodyMode == BodyMode::Active;
        if (state.bodyMode == BodyMode::Active) {
            Vec3 momentum;
            float mass = 0.0f;
            for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
                const float m = std::max(0.0f, profile.links[i].massFraction);
                momentum += body.links[i].linearVelocity * m;
                mass += m;
            }
            const Vec3 com = mass > 0.0f ? momentum * (1.0f / mass) : Vec3 {};
            in.bodyReady = std::hypot(com.x - state.requestedVelocityRaw.x, com.y - state.requestedVelocityRaw.y)
                < settings.ballActiveReadySpeed;
        }
        in.deltaTime = deltaTime;
        BallHandlingOutput3D& out = athlete.ballOutput;
        BallPlaySettings3D play = settings.ballPlay;
        play.contactTiming = std::clamp(settings.ballContactTiming, 0.0f, 1.0f);
        handling.update(in, settings.ballForce, play, out);
        // A fisica: o impulso do toque, o sensor do pe (so na bola-alvo) e o
        // dono (quem interage atravessa a bola com as pernas).
        for (AthleteBall3D& ball : balls) {
            if (!scene.contains(ball.handle)) continue;
            const std::uint64_t key = athleteBallKey3D(ball.handle);
            if (out.impulse && key == out.impulseBallKey)
                scene.applyInflatedBallImpulses(ball.handle, out.linearImpulse, out.angularImpulse);
            if (out.sensorActive && key == out.sensorBallKey)
                scene.setInflatedBallSensor(ball.handle, slot.handle, 1ull << out.sensorLink, out.sensorMargin);
            else
                scene.setInflatedBallSensor(ball.handle, {}, 0, 0.0f);
            if ((out.interacting && key == out.interactingBallKey)
                || ((control.ballShotHeld || control.ballDribbleHeld) && nearest == &ball)) {
                if (ball.ownership != nullptr) ball.ownership->interact(slot.entityId);
                scene.setInflatedBallOwner(ball.handle, slot.handle);
            }
        }
        command.ballGesture = legacyBallGesture3D(out.body);
        state.ballLastOutput = {};
        state.ballLastOutput.touched = out.touched;
        state.ballSpeedScale = out.speedScale;
        output.controlledSpeedScale = out.speedScale;
        state.ballStability = 0.0f;
    } else if (command.controlled && !body.links.empty()) {
        // (Manejo V3.)
        const Vec3 pelvis = body.links.front().position;
        AthleteBall3D* nearest = nullptr;
        float best = 3.0f;
        for (AthleteBall3D& ball : balls) {
            if (!scene.contains(ball.handle) || ball.state == nullptr) continue;
            const float distance = (ball.state->position - pelvis).length();
            if (distance < best) { best = distance; nearest = &ball; }
        }
        BallControlInput3D ballInput;
        ballInput.request = control.ballRequest;
        ballInput.power = control.ballRequestPower;
        control.ballRequest = BallActionKind3D::None;
        output.ballRequestConsumed = true;
        const float look = command.facingYawRadians;
        const Vec3 keys = command.intent.requestedDirectionWorld;
        ballInput.direction = keys.lengthSquared() > 1e-4f ? keys : Vec3 { std::cos(look), std::sin(look), 0.0f };
        ballInput.lookYawRadians = look;
        if (ballInput.request == BallActionKind3D::Shot) {
            ballInput.lookYawRadians = control.ballRequestAimYaw;
            ballInput.shotLift = control.ballRequestLift;
            ballInput.shotCurve = control.ballRequestCurve;
        }
        // Interagindo (barra carregando, pedido, toque em andamento): ele
        // vira o dono da bola - sem colisao com ela ja, mesmo sem encostar.
        if (nearest != nullptr && nearest->ownership != nullptr
            && (control.ballShotHeld || control.ballDribbleHeld || ballInput.request != BallActionKind3D::None
                || state.ballControl.phase() != BallControlPhase3D::Idle)) {
            nearest->ownership->interact(slot.entityId);
            scene.setInflatedBallOwner(nearest->handle, slot.handle);
        }
        if (nearest != nullptr) {
            const InflatedBallProfile3D ballProfile = scene.inflatedBallProfile(nearest->handle);
            ballInput.ballValid = true;
            ballInput.ballPosition = nearest->state->position;
            ballInput.ballVelocity = nearest->state->linearVelocity;
            ballInput.ballRadius = ballProfile.radiusMeters;
            ballInput.ballMass = ballProfile.massKg;
        }
        ballInput.profile = &profile;
        ballInput.state = &body;
        ballInput.facingYawRadians = look;
        ballInput.groundHeight = groundHeight(pelvis);
        ballInput.requestedVelocity = command.intent.requestedVelocityWorld;
        // A velocidade do corpo: no padrao, a da capsula (a da pelve tem o
        // balanco da passada e o proprio gesto); no fisico, a da pelve (a
        // capsula vai a frente do corpo que atrasa, e o gesto nao mexe
        // nela).
        ballInput.bodyVelocity = { command.rootVelocityWorld.x, command.rootVelocityWorld.y, 0.0f };
        ballInput.bodyVelocityValid = animated || active;
        ballInput.contactTimingFactor = std::clamp(settings.ballContactTiming, 0.0f, 1.0f);
        ballInput.hitboxContact = settings.ballHitboxContact;
        ballInput.force = &settings.ballForce;
        ballInput.forceOnly = settings.ballForceOnly;
        ballInput.bodyYawValid = true;
        ballInput.animatedBody = animated || active;
        if (animated || active) {
            // Os pes pela animacao do modo padrao.
            const DefaultLocomotionOutput3D& feet = state.defaultLocomotion.output();
            ballInput.bodyYawRadians = feet.facingYawRadians;
            for (std::size_t side = 0; side < 2; ++side) {
                ballInput.swingProgress[side] = feet.swingProgress[side];
                ballInput.footPlanted[side] = feet.footPlanted[side];
                ballInput.secondsToTouchdown[side] = feet.secondsToTouchdown[side];
            }
            const Vec3 velocity = scene.characterState().velocity;
            ballInput.standing = std::hypot(velocity.x, velocity.y) < 0.3f
                && command.intent.requestedVelocityWorld.lengthSquared() < 0.01f;
        } else {
            ballInput.bodyYawRadians = state.locomotion.telemetry().facingYawRadians;
            for (std::size_t side = 0; side < 2; ++side) {
                ballInput.swingProgress[side] = state.locomotion.telemetry().footSwingProgress[side];
                ballInput.footPlanted[side] = state.locomotion.telemetry().footPlanted[side];
                ballInput.footLanding[side] = state.locomotion.telemetry().footLandingWorld[side];
                ballInput.secondsToTouchdown[side] = state.locomotion.telemetry().footSecondsToTouchdown[side];
            }
            const CharacterLocomotionState3D now = state.locomotion.telemetry().state;
            ballInput.standing = (now == CharacterLocomotionState3D::Idle
                || now == CharacterLocomotionState3D::Turning)
                && command.intent.requestedVelocityWorld.lengthSquared() < 0.01f;
        }
        ballInput.deltaTime = deltaTime;
        BallControlOutput3D ballOutput;
        state.ballControl.update(ballInput, ballOutput);
        state.ballSpeedScale = ballOutput.speedScale;
        state.ballLastOutput = ballOutput;
        // O reforco de estabilidade do gesto (modo fisico; o controle o
        // corta com contato de fora - aqui, tambem na volta de um impacto).
        state.ballStability = settings.ballStabilityAssist && !state.impactAssist.recovering()
            ? ballOutput.stability : 0.0f;
        command.ballGesture = ballOutput.gesture;
        command.ballFoot = ballOutput.foot;
        command.ballScript = ballOutput.script;
        command.ballBody = ballOutput.body;
        command.ballRepel = ballOutput.repel;
        command.ballAvoid = ballOutput.avoid;
        command.ballPlant = ballOutput.plant;
        command.ballGaitRate = ballOutput.gaitRate;
        command.ballKick = ballOutput.kick;
        if (ballOutput.setMotion && nearest != nullptr)
            scene.setInflatedBallMotion(nearest->handle, nearest->state->position, ballOutput.ballVelocity,
                ballOutput.ballSpin);
    }
    // A entrada do modo padrao (capsula, teclas, manejo) e os ajustes do
    // laboratorio; tambem na saida do corpo guiado, com o padrao rodando
    // junto do fisico.
    const auto defaultLocomotionInput = [&]() {
        const PhysicsCharacterState3D& capsule = scene.characterState();
        DefaultLocomotionInput3D input;
        input.feetPositionWorld = command.rootPositionWorld - Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
        input.velocityWorld = capsule.velocity;
        input.grounded = capsule.grounded;
        input.requestedVelocityWorld = command.intent.requestedVelocityWorld;
        input.facingYawRadians = control.facingYawRadians;
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
            const float forward = keys.x * std::cos(control.facingYawRadians)
                + keys.y * std::sin(control.facingYawRadians);
            input.airTackleCombo = command.intent.effort > 0.5f && forward > 0.7f;
        }
        state.crouchHeld = command.intent.crouch;
        input.ballGesture = command.ballGesture;
        input.ballFoot = command.ballFoot;
        input.ballScript = command.ballScript;
        input.ballKick = command.ballKick;
        input.motionVariety = command.motionVariety;
        input.varietySeed = command.varietySeed;
        DefaultLocomotionSettings3D& defaults = state.defaultLocomotion.settings();
        defaults.adaptiveTransitions = settings.defaultAdaptiveTransitions;
        defaults.toeRoll = settings.toeRoll;
        defaults.smoothClipSampling = settings.defaultAdaptiveTransitions;
        defaults.trunkMotion = settings.defaultTrunkMotion;
        defaults.landingAbsorb = settings.defaultLandingAbsorb;
        defaults.curveLeanDeadZone = settings.defaultCurveLeanDeadZone;
        defaults.airTackle = settings.defaultAirTackle;
        defaults.airTacklePitchDegrees = settings.defaultAirTacklePitch;
        {
            // As velocidades da aba "Velocidades" (0, 45, 90, 135 e 180 graus).
            const float t = settings.defaultTrotSpeed;
            defaults.trotSpeeds = { t, t, t, t, t };
            defaults.sprintSpeeds = { settings.defaultSprintForwardSpeed, settings.defaultSprintForwardSpeed,
                settings.defaultSprintSideSpeed, settings.defaultSprintBackwardSpeed,
                settings.defaultSprintBackwardSpeed };
        }
        defaults.trunkMotionAmount = settings.defaultTrunkMotionPercent * 0.01f;
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
            scene.placeCharacter({ out.tackleRootWorld.x, out.tackleRootWorld.y,
                groundHeight(out.tackleRootWorld) }, assets.avatarSettings);
        }
        if (out.tackleEnded) scene.setCharacterVelocity({ out.tackleVelocity.x, out.tackleVelocity.y, 0.0f });
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
            const Vec3 capsuleVelocity = scene.characterState().velocity;
            // (O pe do gesto da bola sai do apoio: o clipe parado o dava
            // apoiado e o Ativo o prendia no chao, brigando com o gesto.)
            std::array<bool, 2> planted = out.footPlanted;
            if (settings.newBallHandlingActive() && command.controlled && athlete.ballOutput.actingSide >= 0)
                planted[static_cast<std::size_t>(athlete.ballOutput.actingSide)] = false;
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
            athlete.active.update(profile, body, observation, target, settings.activeBodySettings, deltaTime,
                athlete.activeOutput);
            const ActiveBodyOutput3D& activeOutput = athlete.activeOutput;
            scene.setRagdollActiveDriveTargets(slot.handle, activeOutput.drives, true);
            scene.setRagdollFrictionScale(slot.handle, 1.0f);
            for (const LinkWrench3D& wrench : activeOutput.wrenches)
                scene.applyRagdollControlLinkForce(slot.handle, wrench.link, wrench.force, wrench.torque);
            if (activeOutput.assistForce.lengthSquared() > 1e-6f)
                scene.applyRagdollControlRootForce(slot.handle, activeOutput.assistForce, {});
            state.previousAnimationPose = state.animationPose.linkPositions.empty() ? pose : state.animationPose;
            state.animationPose = pose;
            state.physicsBlend = 1.0f;
            // A capsula vai atras do corpo quando ele se afasta da pose do
            // plano (pancada, parede): a pose ancora onde o corpo esta.
            // Parado, a raiz-alvo fica sobre os pes (ActiveStance3D): a
            // capsula vai para onde a base real a quer.
            output.followVelocitySet = true;
            output.followVelocityWorld = athlete.stance.baseShift().lengthSquared() > 1e-8f
                ? navigationFollowVelocity3D(rootPosition, plannedRoot, 0.0f)
                : navigationFollowVelocity3D(body.links.front().position, rootPosition, settings.activeFollowSlack);
            if (activeOutput.knockout) {
                toPhysical(("nocaute: " + activeOutput.knockoutReason).c_str());
            } else if (!athlete.active.handingOver()
                && athlete.stance.captureOutside() > activeCaptureLimit(settings, athlete.stance)) {
                toPhysical(("passo de equilibrio, captura " + std::to_string(athlete.stance.captureOutside()) + " m fora"
                    + (athlete.stance.stepping() ? " no passo" : "")).c_str());
            } else if (activeOutput.readyToAnimate && !athlete.stance.stepping() && !state.returnInertia.active()
                && athlete.stance.baseShift().length() < 0.02f
                && (athlete.stance.mismatch() < 0.04f || athlete.active.secondsActive() > settings.activeReturnTimeout)) {
                // O guiado parte do corpo de agora (juntas e pelve) e vai a
                // pose do padrao pela inercia da volta: sem estalo nos pes
                // (a base do Ativo nao e a do clipe).
                const PhysicsBodyState3D& pelvis = body.links.front();
                state.returnCoordinates.assign(profile.links.size(), {});
                std::vector<Vec3> velocities(profile.links.size());
                for (std::size_t i = 1; i < profile.links.size() && i < body.joints.size(); ++i) {
                    const auto& joint = body.joints[i].positionRadians;
                    const auto& rate = body.joints[i].velocityRadiansPerSecond;
                    state.returnCoordinates[i] = { joint[0], joint[1], joint[2] };
                    velocities[i] = { rate[0], rate[1], rate[2] };
                }
                state.returnInertia.start(state.returnCoordinates, velocities, pelvis.position, pelvis.orientation,
                    pelvis.linearVelocity, pelvis.angularVelocity, ActiveExitHalfLife, ActiveExitSeconds);
                scene.setRagdollGuided(slot.handle, true);
                state.guided = true;
                state.bodyMode = BodyMode::Returning;
                state.bodyModeSeconds = 0.0f;
                Log::info("corpo: ativo -> padrao");
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
            scene.placeCharacter({ out.tackleRootWorld.x, out.tackleRootWorld.y,
                groundHeight(out.tackleRootWorld) }, assets.avatarSettings);
        }
        if (out.tackleEnded) scene.setCharacterVelocity({ out.tackleVelocity.x, out.tackleVelocity.y, 0.0f });
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
                constexpr float ReturnSeconds = 0.35f;
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
                    style.velocity = scene.characterState().velocity;
                    style.grounded = !out.airborne;
                    style.scripted = out.rootScripted || out.tackling || out.airTackle || out.crouchStarting;
                    if (settings.newBallHandlingActive() && command.controlled) {
                        style.actingSide = athlete.ballOutput.actingSide;
                        style.reservedSide = athlete.ballOutput.reservedSide;
                        style.landingSide = athlete.ballOutput.landingSide;
                        style.landingShift = athlete.ballOutput.landingShift;
                    } else {
                        style.actingSide = input.ballGesture.legWeight > 0.3f ? input.ballGesture.side : -1;
                    }
                    style.movingShare = out.movingShare;
                    athlete.footTargets = footsteps.plan(style, settings.footsteps, deltaTime);
                    athlete.legSynthesis = synthesizeAthleteLegs3D(profile, legs, footsteps.geometry(),
                        athlete.footTargets, rootPosition, rootOrientation, coordinates, pose);
                }
            } else {
                athlete.footsteps.clear();
            }
            scene.setRagdollGuidePose(slot.handle, pose.linkPositions, pose.linkOrientations);
            state.previousAnimationPose = state.animationPose.linkPositions.empty() ? pose : state.animationPose;
            state.animationPose = pose;
            state.physicsBlend = 0.0f;
        }
        // A capsula manda: nada de ir atras do corpo.
        output.followVelocitySet = true;
        output.followVelocityWorld = {};
        return;
    }
    state.locomotion.update(profile, clips, body, dynamics, command, deltaTime);
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
    physicalIntent.actionStability = command.controlled ? state.ballStability : 0.0f;
    state.adaptivePhysics.update(profile, body, locomotionOutput.guide, physicalIntent, deltaTime);
    const float followSlack = 0.10f * (1.0f - std::clamp(state.adaptivePhysics.output().perturbation, 0.0f, 1.0f));
    if (adaptive && !command.controlled) {
        // A capsula virtual vai atras do corpo; caido, ela fica nele.
        // Comparada com ELA MESMA: no levantar a referencia da locomocao
        // fica presa ao ponto onde o corpo deitou, e comparando com ela a
        // ancora acumulava metros de erro - na saida, o boneco era
        // arremessado atras dela.
        state.navigationAnchorVelocity = navigationFollowVelocity3D(
            body.links.front().position, state.navigationAnchor, down ? 0.0f : followSlack);
        state.navigationAnchor += state.navigationAnchorVelocity * deltaTime;
    }
    if (command.controlled) {
        // A capsula vai atras do corpo quando ele se afasta da pelve de
        // referencia mais de 10 cm (empurrao, tropeco, parede): o corpo
        // e a verdade; depois de um contato externo, sem folga. Com a
        // opcao desligada o corpo e carregado e isso nunca acontece.
        output.followVelocitySet = true;
        output.followVelocityWorld = adaptive && !down
            ? navigationFollowVelocity3D(body.links.front().position,
                locomotionOutput.navigationAnchorWorld, followSlack)
            : Vec3 {};
    }
    state.previousAnimationPose = state.animationPose.linkPositions.empty()
        ? locomotionOutput.targetPose : state.animationPose;
    state.animationPose = locomotionOutput.targetPose;
    state.physicsBlend = locomotionOutput.physicsBlend;
    // While the body is down, the capsule goes where the body is. It is
    // what the camera follows, and leaving it standing where the fall
    // started is what made the character snap back across the floor the
    // moment a recovery finished.
    if (command.controlled && scene.hasCharacter()) {
        const CharacterLocomotionState3D locomotionState = state.locomotion.telemetry().state;
        if (locomotionState == CharacterLocomotionState3D::Fallen
            || locomotionState == CharacterLocomotionState3D::GettingUp
            || locomotionState == CharacterLocomotionState3D::Tackling) {
            const Vec3 rootPosition = body.links.front().position;
            scene.placeCharacter({ rootPosition.x, rootPosition.y, groundHeight(rootPosition) },
                assets.avatarSettings);
            // No carrinho a capsula vai com o embalo do corpo: ao sair
            // dele (correndo), ela continua com essa velocidade.
            if (locomotionState == CharacterLocomotionState3D::Tackling) {
                const Vec3 velocity = body.links.front().linearVelocity;
                scene.setCharacterVelocity({ velocity.x, velocity.y, 0.0f });
            }
        }
    }
    applyCharacterControl3D(scene, slot.handle, locomotionOutput, adaptive ? &state.adaptivePhysics.output() : nullptr,
        down);
}

} // namespace MatterEngine
