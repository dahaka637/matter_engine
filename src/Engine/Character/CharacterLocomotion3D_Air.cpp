#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepAir(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const bool& StartOnlyFromRest = t.StartOnlyFromRest;
    const bool& LandingResync = t.LandingResync;
    const CharacterPhysicalState3D& physical = *t.physical;
    std::array<bool, 2>& feetGroundContact = t.feetGroundContact;
    const bool& getUpActive = t.getUpActive;
    Vec3& desiredPlanarVelocity = t.desiredPlanarVelocity;
    const float& speed = t.speed;
    const bool& moving = t.moving;
    bool& turningInPlace = t.turningInPlace;
    // Crouch and jump are deactivated: there is no authored clip for either
    // right now. Fallen/GettingUp preempt everything else — a real trip or
    // collision the reaction softening below could not absorb — so this
    // state machine reaches Idle, Turning, Walking, Running (a soccer
    // game's brisk jog and its sprint — see clip selection below),
    // Airborne, Landing, Fallen and GettingUp. Losing the ground (a fall, a
    // push, an edge) still needs a sane fallback pose regardless of
    // whether a jump was ever pressed, so Airborne/Landing stay
    // independent of the jump input itself.
    // Chao "de verdade" para os estados de voo. No modo fisico a verdade e o
    // corpo, nao a capsula: os dois decolam juntos, mas pousam em momentos
    // diferentes (medido: a capsula tocava o chao 0,28 s antes, e o corpo
    // chegava ao chao ainda com a pose de passada - pouso passivo). A
    // capsula no chao com o corpo ainda no ar segue em voo (pernas
    // procurando o chao); o corpo tocando o chao antes da capsula ja pousa.
    t.grounded = input.grounded;
    bool& grounded = t.grounded;
    if (input.forceDrivenRoot && !getUpActive && !state.links.empty()) {
        const PhysicsBodyState3D& pelvis = state.links.front();
        float floor = std::numeric_limits<float>::infinity();
        for (const GroundProbeResult3D& probe : input.footGround) {
            if (probe.hasSurface) floor = std::min(floor, probe.pointWorld.z);
        }
        const bool feetDown = feetGroundContact[0] || feetGroundContact[1];
        const bool bodyHigh = !std::isfinite(floor) || pelvis.position.z - floor
            > profile.standingRootHeightMeters + 0.06f;
        const bool flying = m_telemetry.state == CharacterLocomotionState3D::Airborne;
        if (flying && grounded && !feetDown && bodyHigh) {
            grounded = false;
        } else if (!grounded && feetDown && pelvis.linearVelocity.z < 0.5f
            && (!flying || m_stateSeconds > 0.15f)) {
            // Pes no chao e o corpo sem subir: chao, em qualquer estado. Na
            // decolagem, o voo so comeca quando o corpo sobe de fato; no
            // pouso antes da capsula, ele segue pousando (antes voltava a
            // "voar" no tick seguinte, com a capsula ainda no ar).
            grounded = true;
        }
    }
    t.desiredState = CharacterLocomotionState3D::Idle;
    CharacterLocomotionState3D& desiredState = t.desiredState;
    if (getUpActive) {
        desiredState = m_tackling ? CharacterLocomotionState3D::Tackling
            : m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? CharacterLocomotionState3D::Fallen
            : CharacterLocomotionState3D::GettingUp;
    } else if (!grounded) {
        desiredState = CharacterLocomotionState3D::Airborne;
    } else if ((!m_previousGrounded
        || (m_telemetry.state == CharacterLocomotionState3D::Landing
            && m_stateSeconds < m_landingSettleSeconds))
        // Pousando correndo, a passada continua: o estado de pouso trava os
        // dois pes onde tocaram, e a 7 m/s o corpo passava 1,7 m a frente
        // deles (35-50 graus de inclinacao). O amortecimento vale do mesmo
        // jeito (a pelve cede depois de qualquer toque, mais abaixo).
        && !(input.forceDrivenRoot && moving && speed > 2.5f)) {
        desiredState = CharacterLocomotionState3D::Landing;
    } else if (m_brakeStepping) {
        // Freando pelos passos: a passada de corrida para; os pes sao do
        // planejador (passos de freada) e o corpo fica na pose parada com a
        // reacao de freada por cima.
        desiredState = CharacterLocomotionState3D::Idle;
    } else if (moving) {
        // Sprint para a frente e de lado; recuando, so com o sprint de
        // costas (sem ele, segurar sprint nao corre). Correndo e freando, o
        // ciclo e o da velocidade do corpo (o sprint ate ~4,6 m/s).
        const bool sprintPace = m_brakeGaitActive
            ? m_brakeGaitSpeed > BrakeSprintClipSpeed : input.sprinting;
        desiredState = sprintPace
                && (m_gaitDirection != CharacterGaitDirection3D::Backward
                    || animations.sprintBackward != nullptr)
            ? CharacterLocomotionState3D::Running
            : CharacterLocomotionState3D::Walking;
    } else if (turningInPlace) {
        desiredState = CharacterLocomotionState3D::Turning;
    }
    if (desiredState == CharacterLocomotionState3D::Airborne
        && m_telemetry.state != CharacterLocomotionState3D::Airborne) {
        m_liftoffSpeed = speed;
        m_liftoffVerticalSpeed = input.rootVelocityWorld.z;
        // O pulo e escolhido na decolagem e vale ate o fim da aterrissagem:
        // parado se quase nao havia movimento, senao o do setor em que ele
        // corria (frente, costas, lados).
        const auto pick = [&](CharacterJumpKind3D kind,
            const AnimationClip3D* clip) {
            if (clip == nullptr) return false;
            m_jumpKind = kind;
            m_jumpClip = clip;
            return true;
        };
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
        bool picked = false;
        if (speed > 1.0f) {
            switch (m_gaitDirection) {
            case CharacterGaitDirection3D::Backward:
                picked = pick(CharacterJumpKind3D::Backward,
                    animations.jumpBackward);
                break;
            case CharacterGaitDirection3D::Left:
                picked = pick(CharacterJumpKind3D::Left, animations.jumpLeft);
                break;
            case CharacterGaitDirection3D::Right:
                picked = pick(CharacterJumpKind3D::Right, animations.jumpRight);
                break;
            default:
                picked = pick(CharacterJumpKind3D::Forward,
                    animations.jumpForward);
                break;
            }
        }
        if (!picked) {
            pick(CharacterJumpKind3D::Standing, animations.jumpStanding);
        }
        // v2: correndo para a frente, o voo e a passada aberta do proprio
        // ciclo - do ponto da decolagem ao toque seguinte -, esticada no tempo
        // do pulo: a perna de tras sai estendida, a da frente chega para o
        // pouso e a corrida segue. Com o pulo para a frente (pernas juntas
        // e dobradas, bracos a frente) o pulo correndo parecia o de quem
        // pula parado e pousava nos dois pes.
        m_leapClip = nullptr;
        // (No sprint e com ajuda: no trote, pulos seguidos giravam a pelve
        // ate ~45 graus no ar; com pouca ajuda, pousando num pe so de um pulo
        // alto, a perna cedia - pulo em sprint e vira a 30%, 0 -> 11 em 12.
        // Com a ajuda cheia ela leva o voo a ~7,6 m/s e, pousando num pe so
        // com um corte pedido, ele tombava - sequencia mista, 0 -> 2 em 12.)
        if (RunLeap && input.physicalV2 && m_jumpKind == CharacterJumpKind3D::Forward
            && speed > RunLeapMinimumSpeed && input.walkingAssist >= RunLeapMinimumAssist
            && input.walkingAssist <= RunLeapMaximumAssist
            && (m_telemetry.state == CharacterLocomotionState3D::Running
                || m_telemetry.state == CharacterLocomotionState3D::Walking)
            && m_previousClip != nullptr && m_previousClip != animations.idle
            && m_previousClip->loops && m_previousClip->durationSeconds > 0.05f) {
            const ClipFootEvents& events = footEventsFor(profile, *m_previousClip);
            const float duration = m_previousClip->durationSeconds;
            const float from = stanceAlignedPhase(*m_previousClip, m_cyclePhase);
            float best = 2.0f;
            for (std::size_t side = 0; side < 2; ++side)
                for (const FootTouchdown& touchdown : events.touchdowns[side]) {
                    const float ahead = std::fmod(touchdown.touchdown / duration - from + 2.0f, 1.0f);
                    if (ahead > 0.03f && ahead < best) best = ahead;
                }
            if (best <= 1.0f) {
                m_leapClip = m_previousClip;
                m_leapFrom = from;
                m_leapSpan = best;
            }
        }
    }
    // Fase do voo pela velocidade vertical: 0 na decolagem, 0,5 no apice, 1
    // ao voltar a altura de onde saiu. Vale para qualquer altura de pulo. Sem
    // impulso para cima (caiu de um degrau), o voo ja comeca descendo.
    if (desiredState == CharacterLocomotionState3D::Airborne) {
        const float verticalSpeed = input.forceDrivenRoot && !state.links.empty()
            ? state.links.front().linearVelocity.z : input.rootVelocityWorld.z;
        if (input.forceDrivenRoot) {
            // A subida do corpo e a que conta (ele ainda ganha velocidade
            // depois que a capsula decolou).
            m_liftoffVerticalSpeed = std::max(m_liftoffVerticalSpeed, verticalSpeed);
        }
        m_flightPhase = m_liftoffVerticalSpeed > 1.0f
            ? std::clamp((m_liftoffVerticalSpeed - verticalSpeed)
                / (2.0f * m_liftoffVerticalSpeed), 0.0f, 1.0f)
            : 0.5f + 0.5f * std::clamp(-verticalSpeed / 4.0f, 0.0f, 1.0f);
    } else if (desiredState != CharacterLocomotionState3D::Landing) {
        m_jumpKind = CharacterJumpKind3D::None;
        m_jumpClip = nullptr;
        m_leapClip = nullptr;
    }
    // Toque no chao depois de um voo (com ou sem o estado de pouso).
    const bool enteringLanding = m_telemetry.state
            == CharacterLocomotionState3D::Airborne
        && desiredState != CharacterLocomotionState3D::Airborne
        && !getUpActive;
    // v2 (T_LandResync): pousando de um pulo direto na passada, o ciclo
    // recomeca no toque do pe que chegou primeiro - ele em apoio, o outro ja
    // no balanco para o passo seguinte. Sem isso o ciclo voltava na fase da
    // decolagem: o clipe pedia o outro pe no chao, os dois ficavam plantados
    // e o corpo, a 7 m/s, girava para a frente por cima deles.
    if (LandingResync && enteringLanding && input.forceDrivenRoot
        && (desiredState == CharacterLocomotionState3D::Walking || desiredState == CharacterLocomotionState3D::Running)) {
        int first = -1;
        if (feetGroundContact[0] != feetGroundContact[1]) first = feetGroundContact[0] ? 0 : 1;
        else if (m_feet[0].valid && m_feet[1].valid && !state.links.empty()) {
            // Os dois (ou nenhum): o mais a frente no rumo do corpo, ou o
            // mais baixo.
            const Vec3 velocity { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
            const Vec3 a = state.links[m_feet[0].linkIndex].position, b = state.links[m_feet[1].linkIndex].position;
            first = feetGroundContact[0] && velocity.lengthSquared() > 0.25f
                ? (dot(a - b, velocity) > 0.0f ? 0 : 1) : (a.z < b.z ? 0 : 1);
        }
        m_landingResyncFoot = first;
    }
    const CharacterLocomotionState3D previousState = m_telemetry.state;
    enterState(desiredState);
    const bool enteredForwardGait = (previousState == CharacterLocomotionState3D::Idle
            || previousState == CharacterLocomotionState3D::Turning)
        && (desiredState == CharacterLocomotionState3D::Walking
            || desiredState == CharacterLocomotionState3D::Running)
        && m_gaitDirection == CharacterGaitDirection3D::Forward
        && animations.idleToSprint != nullptr;
    if (enteredForwardGait && StartStepEnabled) m_startStepRequested = true;
    // Ajuda dosada na arrancada parada: o teto comeca baixo e cresce sem
    // parar (sem janela: terminando de uma vez, o pedido saltava de ~3,5 para
    // 7,5 m/s no meio do sprint).
    // (Armado ja parado: o jogo le o teto do tick anterior, e armando so na
    // entrada da passada o primeiro tick saia com o pedido inteiro.)
    const bool idleLike = desiredState == CharacterLocomotionState3D::Idle
        || desiredState == CharacterLocomotionState3D::Turning;
    // (So com o corpo parado de fato: armado nas paradas curtas no meio de uma
    // troca - inversao, giro - ele atrasava a mudanca de direcao.)
    m_bodyRestSeconds = std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y) < 0.3f
        ? std::min(10.0f, m_bodyRestSeconds + deltaTime) : 0.0f;
    if (StartDoseSeconds > 0.0f && input.forceDrivenRoot && !getUpActive && idleLike && m_bodyRestSeconds > 0.3f)
        m_startDoseCap = StartSpeedBase;
    else if (m_startDoseCap < 100.0f)
        // Dosada so no comeco (StartDoseSeconds): depois o teto sobe rapido
        // ate sumir - devagar ate o fim, o sprint levava 0,93 s a 80%.
        m_startDoseCap += (m_startDoseCap < StartSpeedBase + StartSpeedRate * StartDoseSeconds
            ? StartSpeedRate : 30.0f) * deltaTime;

    if (enteredForwardGait) {
        m_startClipActive = true;
        m_startClipSprint = desiredState == CharacterLocomotionState3D::Running;
        m_startClipSeconds = 0.0f;
        m_cyclePhase = 0.0f;
        // O primeiro passo e do pe de tras (no rumo pedido), como na
        // referencia Idle To Sprint: o ciclo comeca quando o contato dele cai
        // no clipe. Na fase 0 o direito ja estava a 42% do balanco - o alvo o
        // queria no ar com o pe ainda carregado no chao, e ele saia
        // arrastado; com o direito na frente, o pe da frente dava o passo.
        const AnimationClip3D* startClip = desiredState == CharacterLocomotionState3D::Running
            && animations.sprint != nullptr && m_sprintClipAllowed ? animations.sprint : animations.walk;
        const Vec3 wanted { input.intent.requestedDirectionWorld.x, input.intent.requestedDirectionWorld.y, 0.0f };
        if (StartFromRearFoot && input.forceDrivenRoot && startClip != nullptr && startClip->durationSeconds > 0.0f
            && wanted.lengthSquared() > 0.01f && physical.valid) {
            const Vec3 direction = wanted.normalized();
            const float back[2] = {
                dot(physical.feet[0].soleCenterWorld - physical.centerOfMassWorld, direction),
                dot(physical.feet[1].soleCenterWorld - physical.centerOfMassWorld, direction) };
            // Quase lado a lado: sai o menos carregado.
            const int rear = std::abs(back[0] - back[1]) > 0.05f ? (back[0] < back[1] ? 0 : 1)
                : (physical.feet[0].loadNewtons < physical.feet[1].loadNewtons ? 0 : 1);
            const AnimationScalarTrack3D& contact = rear == 0 ? startClip->leftFootContact : startClip->rightFootContact;
            const float duration = startClip->durationSeconds;
            constexpr int Samples = 240;
            float liftoff = -1.0f;
            float previous = sampleAnimationScalarTrack3D(contact, duration * (Samples - 1) / Samples, duration, true);
            for (int i = 0; i < Samples; ++i) {
                const float now = sampleAnimationScalarTrack3D(contact, duration * i / Samples, duration, true);
                if (previous >= 0.5f && now < 0.5f) { liftoff = static_cast<float>(i) / Samples; break; }
                previous = now;
            }
            if (liftoff >= 0.0f) {
                // Fase do ciclo (a do clipe menos o deslocamento do apoio do
                // esquerdo - ver stanceAlignedPhase).
                const float offset = stanceAlignedPhase(*startClip, 0.0f);
                m_cyclePhase = std::fmod(liftoff - offset - StartPhaseLead + 2.0f, 1.0f);
            }
        }
    } else if (desiredState != CharacterLocomotionState3D::Walking
        && desiredState != CharacterLocomotionState3D::Running) {
        m_startClipActive = false;
    }
    // Pouso: a velocidade de queda do corpo no toque decide quanto ele
    // agacha e por quanto tempo (0,18 s num degrau; ~0,5 s caindo de um
    // pulo alto). Medida no voo, antes do toque (no tick do toque ela ja
    // caiu pela metade).
    if (desiredState == CharacterLocomotionState3D::Airborne
        && !state.links.empty()) {
        m_airborneFallSpeed = std::max(0.0f, input.forceDrivenRoot
            ? -state.links.front().linearVelocity.z
            : -input.rootVelocityWorld.z);
    }
    m_sinceTouchdownSeconds = enteringLanding ? 0.0f
        : std::min(10.0f, m_sinceTouchdownSeconds + deltaTime);
    if (enteringLanding) {
        m_landingImpactSpeed = m_airborneFallSpeed;
        m_landingSettleSeconds = input.forceDrivenRoot
            // (subida do agachamento em >= ~0,2 s: mais rapido, a ajuda de
            // altura ia ao maximo para acompanhar e o corpo quicava)
            ? 0.34f + 0.09f * std::clamp(m_landingImpactSpeed - 1.0f, 0.0f, 5.0f)
            : 0.18f;
    }
    m_previousGrounded = grounded;
    m_stateSeconds += deltaTime;
    if (m_startClipActive) {
        m_startClipSeconds += deltaTime;
        // A captura contem uma passada inteira de sprint, inclusive voo. No
        // trote usamos so a transferencia inicial; reproduzi-la inteira e
        // mais rapido deixava ambos os pes no ar por ~0,4 s.
        const float duration = m_startClipSprint ? 0.62f : 0.34f;
        if (animations.idleToSprint == nullptr
            || m_gaitDirection != CharacterGaitDirection3D::Forward
            || m_startClipSeconds >= std::min(
                duration, animations.idleToSprint->durationSeconds)) {
            m_startClipActive = false;
        }
    }
    m_transitionSeconds += deltaTime;
    m_idleSeconds += deltaTime;

    t.heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_facingYaw);
    const Quaternion& heading = t.heading;
    t.localVelocity = heading.conjugate().rotate(
        desiredPlanarVelocity);
    const Vec3& localVelocity = t.localVelocity;

    // Um ciclo por vez, sem mistura por direcao: a pelve ja se alinha com o
    // movimento (ou contra ele, recuando), entao o ciclo sempre anda para a
    // frente ou para tras em relacao a ela. Walking toca a caminhada rapida
    // (ou o recuo), Running o sprint quando existe - senao a caminhada.
    // Todo outro estado (giro parado, que nao tem clipe proprio, e
    // Airborne/Landing, procedurais - ver applyFlightPose) usa o idle.
    t.gaitMoving = desiredState == CharacterLocomotionState3D::Walking
        || desiredState == CharacterLocomotionState3D::Running;
    const bool& gaitMoving = t.gaitMoving;
    // Arrancada fisica em tres tempos, medida na referencia Idle To Sprint:
    // comprime primeiro (quase sem deslocamento), transfere peito/COM sobre o
    // apoio e so entao entrega o ciclo cheio. O sprint usa a leitura inteira;
    // o trote conserva a mesma mecanica com metade da amplitude e menos tempo.
    Vec3 startupDirection = desiredPlanarVelocity;
    startupDirection.z = 0.0f;
    const float startupWantedSpeed = startupDirection.length();
    if (startupWantedSpeed > 0.001f) startupDirection *= 1.0f / startupWantedSpeed;
    const float startupBodySpeed = startupWantedSpeed > 0.001f
        ? std::max(0.0f, dot(Vec3 { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f }, startupDirection)) : 0.0f;
    // v2 (T_StartFromRest): a arrancada so vale saindo de parado. Pousando
    // de um pulo em sprint o estado tambem recomeca (Airborne -> Running) com
    // o corpo abaixo de 85% do pedido, e ele corria o comeco do ciclo como
    // quem sai parado.
    if (enteredForwardGait) m_gaitStartFromRest = true;
    else if (desiredState != previousState) m_gaitStartFromRest = false;
    t.gaitStarting = input.forceDrivenRoot && gaitMoving
        && (!StartOnlyFromRest || m_gaitStartFromRest)
        && m_stateSeconds < 0.65f && startupWantedSpeed > 1.0f
        && startupBodySpeed < 0.85f * startupWantedSpeed;
    const bool& gaitStarting = t.gaitStarting;
    const float startupTime = gaitStarting
        ? std::clamp(m_stateSeconds / 0.65f, 0.0f, 1.0f) : 1.0f;
    const float startupRise = smoothStep(startupTime / 0.20f);
    // O primeiro quadro da referencia ja traz ~4,4 graus de avanco. Partir
    // de zero enquanto a perna comeca o ciclo deixa a massa atras dos pes;
    // ha pre-carga imediata, e o restante cresce com a compressao.
    t.startupProgress = gaitStarting
        ? smoothStep(m_stateSeconds / 0.55f) : 1.0f;
    const float& startupProgress = t.startupProgress;
    const bool sprintTier = desiredState == CharacterLocomotionState3D::Running
        && animations.sprint != nullptr;
    const bool forwardSprintStart = sprintTier
        && m_gaitDirection == CharacterGaitDirection3D::Forward;
    // Mantem a transferencia minima que ja sustentava o corpo no primeiro
    // quadro e cresce um pouco durante a compressao visual, sobretudo no
    // sprint. O envelope fisico termina quando o corpo atinge 85% do pedido.
    const float startupDrive = gaitStarting
        ? (1.0f - startupProgress) * (0.78f + 0.22f * startupRise) : 0.0f;
    const float startupCompression = gaitStarting
        ? smoothStep(startupTime / 0.16f)
            * (1.0f - smoothStep((startupTime - 0.56f) / 0.34f))
        : 0.0f;
    // A referencia chega perto de 38 graus porque raiz e coluna avancam
    // juntas. No ragdoll de jogo essa soma deslocaria o COM para fora do pe:
    // a leitura atletica fica principalmente na coluna, com a raiz menor.
    t.startupChestAmount =
        (forwardSprintStart ? 0.11f : 0.065f) * startupDrive;
    const float& startupChestAmount = t.startupChestAmount;
    t.startupRootLean = (forwardSprintStart ? 0.075f : 0.045f)
        * (1.0f - startupProgress);
    const float& startupRootLean = t.startupRootLean;
    t.startupPelvisDrop =
        (forwardSprintStart ? 0.025f : 0.010f) * startupCompression;
    const float& startupPelvisDrop = t.startupPelvisDrop;
    t.gaitCycleSpeed = gaitStarting
        ? std::min(speed, std::max(1.0f, startupBodySpeed + 0.85f)) : speed;
    const float& gaitCycleSpeed = t.gaitCycleSpeed;
    // Rising plays its own one-shot clip through the same path as every
    // other clip: sampled joint by joint, with the clip's own root track.
    t.standUpClip =
        m_getUpPhase == CharacterGetUpPhase3D::Rising
            ? (m_tackling ? m_tackleClip
                : m_fallOrientation == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack)
            : nullptr;
    const AnimationClip3D*& standUpClip = t.standUpClip;
    // Com pouca ajuda o clipe de sprint (feito para ~7,5 m/s) so entra com a
    // passada rapida: limitada pela inercia do corpo (2-4 m/s na arrancada),
    // ele tocava a 0,55 do ritmo - a cadencia lentissima nao trazia o pe a
    // frente a tempo e, soltando o comando, os dois pes ficavam atras do
    // corpo. Abaixo disso corre no ciclo de trote, no ritmo dele.
    {
        const float lowAssistClip = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        if (SprintClipSpeed <= 0.0f || (lowAssistClip < 0.5f && !SprintClipAllAssist)
            || desiredState != CharacterLocomotionState3D::Running)
            m_sprintClipAllowed = true;
        else if (speed > SprintClipSpeed + 0.3f)
            m_sprintClipAllowed = true;
        else if (speed < SprintClipSpeed - 0.3f)
            m_sprintClipAllowed = false;
    }
    const bool sprintingGait =
        desiredState == CharacterLocomotionState3D::Running && m_sprintClipAllowed;
    t.gaitClip = animations.walk;
    const AnimationClip3D*& gaitClip = t.gaitClip;
    switch (m_gaitDirection) {
    case CharacterGaitDirection3D::Backward:
        gaitClip = sprintingGait && animations.sprintBackward != nullptr
            ? animations.sprintBackward : animations.walkBackward;
        break;
    case CharacterGaitDirection3D::Left:
        gaitClip = sprintingGait && animations.sprintStrafeLeft != nullptr
            ? animations.sprintStrafeLeft : animations.strafeLeft;
        break;
    case CharacterGaitDirection3D::Right:
        gaitClip = sprintingGait && animations.sprintStrafeRight != nullptr
            ? animations.sprintStrafeRight : animations.strafeRight;
        break;
    default:
        gaitClip = sprintTier && m_sprintClipAllowed ? animations.sprint : animations.walk;
        break;
    }
    t.jumpPose = m_jumpClip != nullptr
        && (desiredState == CharacterLocomotionState3D::Airborne
            || desiredState == CharacterLocomotionState3D::Landing);
    const bool& jumpPose = t.jumpPose;
    t.leapPose = jumpPose && m_leapClip != nullptr;
    const bool& leapPose = t.leapPose;
    // (Com RunLeapHold: vai a meia passada no comeco do voo, segura e so
    // completa ate o toque no fim - as pernas balancando no ar inteiro giravam
    // a pelve ate ~45 graus na descida.)
    const auto leapProgress = [](float phase) {
        if (RunLeapHold <= 0.0f) return phase;
        return RunLeapHold * smoothStep(phase / 0.3f)
            + (1.0f - RunLeapHold) * smoothStep((phase - 0.7f) / 0.3f);
    };
    t.idleClip = animations.idle;
    const AnimationClip3D*& idleClip = t.idleClip;
    t.authoredStartActive = m_startClipActive
        && animations.idleToSprint != nullptr && gaitMoving;
    const bool& authoredStartActive = t.authoredStartActive;
}

} // namespace MatterEngine
