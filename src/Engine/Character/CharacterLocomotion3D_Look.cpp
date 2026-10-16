#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepLook(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const float& groundHeightGuess = t.groundHeightGuess;
    const bool& getUpActive = t.getUpActive;
    const float& impact = t.impact;
    Vec3& desiredPlanarVelocity = t.desiredPlanarVelocity;
    const float& speed = t.speed;
    const float& lookYaw = t.lookYaw;
    const float& pathSpeed = t.pathSpeed;
    const float& pathTurnRate = t.pathTurnRate;
    CharacterLocomotionState3D& desiredState = t.desiredState;
    const Quaternion& heading = t.heading;
    const bool& gaitMoving = t.gaitMoving;
    const float& startupProgress = t.startupProgress;
    const float& startupChestAmount = t.startupChestAmount;
    const float& startupRootLean = t.startupRootLean;
    const float& signedCurveAcceleration = t.signedCurveAcceleration;
    // O olhar. A pelve pode estar a ate ~105 graus da camera (andando de
    // lado, ou na borda do recuo); o personagem continua olhando para ela.
    // A coluna devolve a maior parte, ate 45 graus, e a cabeca completa, ate
    // 75 - o tronco fica perto da direcao do olhar e a cabeca nela. A
    // cabeca acompanha a camera rapido; o tronco vem atras num ritmo
    // proprio, mais lento, e um giro brusco de camera le como "a cabeca
    // olha, o corpo vem junto", nao como a coluna inteira girando de uma
    // vez no mesmo quadro.
    t.viewYawError = std::clamp(wrapAngle(lookYaw - m_facingYaw),
        -(TorsoTwistLimitRadians + HeadTurnLimitRadians),
        TorsoTwistLimitRadians + HeadTurnLimitRadians);
    const float& viewYawError = t.viewYawError;
    const float headLookBlend = 1.0f - std::exp(-15.0f * deltaTime);
    const float torsoLookRate = 4.2f;
    const float torsoLookBlend = 1.0f - std::exp(-torsoLookRate * deltaTime);
    const float torsoLimit = desiredState == CharacterLocomotionState3D::Running
        ? SprintTorsoTwistLimitRadians : TorsoTwistLimitRadians;
    const float torsoTarget = std::clamp(viewYawError * TorsoLookShare,
        -torsoLimit, torsoLimit);
    m_torsoTwist += (torsoTarget - m_torsoTwist) * torsoLookBlend;
    const float headTarget = std::clamp(viewYawError - m_torsoTwist,
        -HeadTurnLimitRadians, HeadTurnLimitRadians);
    m_headTurn += (headTarget - m_headTurn) * headLookBlend;
    m_filteredLookPitch = m_lookPitch;
    // Filters keep running so they are not stale on handoff, but the
    // look-at twist itself only applies to the normal locomotion pose —
    // piling it on top of the get-up sequence's own carefully phased
    // spine/neck targets would just fight them.
    if (!getUpActive) {
        // Twist + gira a frente do segmento para a esquerda, na coluna e no
        // pescoco (medido pela FK em tools/animation). Divisao pelos
        // limites de cada junta: coluna 30/35/35, pescoco 40 e cabeca 60.
        addJointCoordinate(profile, m_coordinates, "Abdomen", 0,
            m_torsoTwist * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 0,
            m_torsoTwist * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 0,
            m_torsoTwist * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Neck", 0,
            m_headTurn * 0.40f);
        addJointCoordinate(profile, m_coordinates, "Head", 0,
            m_headTurn * 0.60f);
        // Postura parado: a pose em alerta curva o peito superior ~23 graus e
        // o pescoco e a cabeca continuavam essa linha - a cabeca 7 cm a
        // frente da base do pescoco, corcunda. O pescoco e a cabeca estendem
        // (pescoco ~8 graus, cabeca quase reta, olhos na horizontal) e o
        // peito superior curva um pouco menos. Andando, metade.
        {
            const CharacterLocomotionState3D postureState = m_telemetry.state;
            const float uprightShare = postureState == CharacterLocomotionState3D::Idle
                    || postureState == CharacterLocomotionState3D::Turning ? 1.0f
                : postureState == CharacterLocomotionState3D::Walking
                    || postureState == CharacterLocomotionState3D::Running ? IdlePostureMovingShare : 0.0f;
            m_postureUpright += (uprightShare - m_postureUpright) * (1.0f - std::exp(-deltaTime / 0.3f));
            addJointCoordinate(profile, m_coordinates, "Neck", 1, -IdleNeckExtension * m_postureUpright);
            addJointCoordinate(profile, m_coordinates, "Head", 1, -IdleHeadExtension * m_postureUpright);
            addJointCoordinate(profile, m_coordinates, "UpperChest", 1, -IdleChestExtension * m_postureUpright);
            addJointCoordinate(profile, m_coordinates, "Chest", 1, -0.5f * IdleChestExtension * m_postureUpright);
        }
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            -m_filteredLookPitch * 0.14f);
        addJointCoordinate(profile, m_coordinates, "Neck", 1,
            -m_filteredLookPitch * 0.36f);
        addJointCoordinate(profile, m_coordinates, "Head", 1,
            -m_filteredLookPitch * 0.56f);
        // A coluna reage a pelve FISICA, nao apenas a inclinacao ideal da
        // animacao. Com ajuda parcial a pelve pode adiantar/tombar muito mais
        // que a referencia; a regra antiga via no maximo os poucos graus
        // pedidos pela arrancada e o tronco continuava soldado a ela.
        //
        // Esta e uma tarefa articular amortecida: a pelve continua livre e a
        // coluna devolve parte do erro para manter o peito proximo da vertical,
        // sem virar uma barra rigida nem apagar toda a inclinacao da corrida.
        Vec3 spineTarget {};
        if (input.forceDrivenRoot && !state.links.empty()) {
            const Vec3 pelvisUp = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 headingForward = heading.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 headingLeft = heading.rotate({ 0.0f, 1.0f, 0.0f });
            const float pelvisForward = std::asin(std::clamp(
                dot(pelvisUp, headingForward), -1.0f, 1.0f));
            const float pelvisSide = std::asin(std::clamp(
                dot(pelvisUp, headingLeft), -1.0f, 1.0f));
            spineTarget.x = std::clamp(-0.62f * pelvisForward, -0.24f, 0.24f)
                * startupProgress;
            spineTarget.y = std::clamp(-0.48f * pelvisSide, -0.20f, 0.20f);
            // Coluna viva na passada: os ombros inclinam ao contrario da pelve
            // (o tronco vai sobre a perna de apoio) e o tronco cede um pouco a
            // cada pisada, como mola. Os ciclos autorais tem a coluna quase
            // parada de lado (+-1 grau por segmento): medido correndo, ombros
            // x quadris inclinavam +-4,5 graus, contra +-8 (trote) e +-17
            // (sprint) nas referencias - parecia um bloco so.
            // So a parte rapida da rolagem da pelve (a da passada, ~2 Hz): a
            // lenta e a inclinacao de proposito (curva), que a coluna nao
            // pode desfazer - com ela toda o tronco da curva caia de 12 para
            // 2-7 graus.
            m_pelvisSideSlow += (pelvisSide - m_pelvisSideSlow) * (1.0f - std::exp(-deltaTime / 0.35f));
            if (gaitMoving) {
                const float pace = smoothStep((speed - 3.0f) / 3.5f);
                const float rollGain = SpineRollGainSlow + (SpineRollGainFast - SpineRollGainSlow) * pace;
                spineTarget.y = std::clamp(-0.48f * m_pelvisSideSlow
                    - rollGain * (pelvisSide - m_pelvisSideSlow), -SpineRollLimit, SpineRollLimit);
                // Na pisada a pelve freia para baixo (aceleracao para cima):
                // o tronco flexiona um pouco e volta.
                const float impact = std::max(0.0f, m_estimator.state().centerOfMassAccelerationWorld.z) / 9.81f;
                spineTarget.x += std::min(SpineImpactFlex * impact, 0.12f);
            }
        }
        const Vec3 spineError = m_spineBalance - spineTarget;
        const Vec3 spineImpulse = m_spineBalanceVelocity + spineError * 12.0f;
        const float spineDecay = std::exp(-12.0f * deltaTime);
        m_spineBalance = spineTarget
            + (spineError + spineImpulse * deltaTime) * spineDecay;
        m_spineBalanceVelocity = (m_spineBalanceVelocity
            - spineImpulse * (12.0f * deltaTime)) * spineDecay;
        const float spineForward = m_spineBalance.x;
        const float spineSide = m_spineBalance.y;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1, spineForward * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1, spineForward * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1, spineForward * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2, spineSide * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2, spineSide * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2, spineSide * 0.35f);
        // Na arrancada o peito acompanha a massa para a frente; a
        // contra-inclinacao que estabiliza a marcha entra conforme o corpo
        // ganha velocidade. No tropeco a inercia tambem continua levando o
        // tronco para a frente, como na referencia estudada: o passo longo e
        // a pelve baixa e que o capturam, nao uma extensao instantanea.
        const float startupChest = startupChestAmount;
        const Vec3 tripLocal = heading.conjugate().rotate(m_tripDirection);
        const float tripForward = 0.12f * StumbleTrunkScale * m_tripRecovery * tripLocal.x;
        const float tripSide = -0.10f * m_tripRecovery * tripLocal.y;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1,
            (startupChest + tripForward) * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1,
            (startupChest + tripForward) * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            (startupChest + tripForward) * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2, tripSide * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2, tripSide * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2, tripSide * 0.35f);
        if (m_tripFoot >= 0 && m_tripRecovery > 0.0f) {
            const char* thigh = m_tripFoot == 0 ? "LeftThigh" : "RightThigh";
            const char* shin = m_tripFoot == 0 ? "LeftShin" : "RightShin";
            const float elevation = m_tripElevating ? 1.0f : 0.45f;
            addJointCoordinate(profile, m_coordinates, thigh, 1,
                -0.24f * m_tripRecovery * elevation);
            addJointCoordinate(profile, m_coordinates, shin, 0,
                0.55f * m_tripRecovery * elevation);
        }
    }

    // A freada especial e uma recuperacao de emergencia, nao a animacao
    // normal de parar. Ela so aparece quando ainda existe momento fisico de
    // corrida maxima E a intencao desacelera forte. No trote e em trajetos
    // curtos, o ciclo apenas encurta os passos e assenta: projetar um pe
    // muito a frente nessas condicoes parece uma derrapagem sem causa.
    //
    // A reacao continua sendo uma mola de pose e nao segura a capsula. Vale
    // em qualquer direcao: o sentido vem do movimento de entrada.
    {
        if (speed > 0.3f) m_brakeDirection = desiredPlanarVelocity / speed;
        m_brakeEntrySpeed = std::max(speed,
            m_brakeEntrySpeed - 2.5f * deltaTime);
        const bool braceable = !getUpActive && input.grounded
            && desiredState != CharacterLocomotionState3D::Airborne
            && desiredState != CharacterLocomotionState3D::JumpStarting;
        const float deceleration = std::max(0.0f,
            -dot(m_smoothedAcceleration, m_brakeDirection));
        const Vec3 physicalPlanarVelocity { m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y, 0.0f };
        const float physicalMomentum = std::max(0.0f,
            dot(physicalPlanarVelocity, m_brakeDirection));
        // Comeca apenas perto da corrida maxima. Os dois portoes evitam que
        // uma oscilacao de velocidade do trote seja confundida com freada.
        const float sprintMomentum = smoothStep(
            (std::min(m_brakeEntrySpeed, physicalMomentum) - 5.5f) / 1.0f);
        const float hardDeceleration = smoothStep((deceleration - 8.0f) / 8.0f);
        // Freando pela passada: pela velocidade que ainda falta tirar (do
        // trote, metade; do sprint, inteira).
        const float gaitBrake = m_brakeGait * BrakeReactionScale
            * smoothStep((m_brakeGaitSpeed - 1.0f) / 4.5f);
        const float intensity = braceable
            ? std::max(sprintMomentum * hardDeceleration, gaitBrake)
            : 0.0f;
        const Vec3 drive = heading.conjugate().rotate(
            m_brakeDirection * intensity);
        constexpr float BrakeOmega = 11.0f, BrakeDamping = 0.55f;
        m_brakeReactionVelocity += (BrakeOmega * BrakeOmega
                * (Vec3 { drive.x, drive.y, 0.0f } - m_brakeReaction)
            - 2.0f * BrakeDamping * BrakeOmega * m_brakeReactionVelocity)
            * deltaTime;
        m_brakeReaction += m_brakeReactionVelocity * deltaTime;
        m_brakeReaction = clampMagnitude(m_brakeReaction, 1.2f);
        m_telemetry.brakeReaction = std::hypot(m_brakeReaction.x,
            m_brakeReaction.y);
        m_telemetry.brakeReactionForward = m_brakeReaction.x;
    }
    if (!getUpActive) {
        // Tronco para onde a inercia leva (flexao + para a frente,
        // inclinacao + para a esquerda), a cabeca devolvendo parte para o
        // olhar ficar no horizonte, e os cotovelos abrindo.
        // Freando: o tronco estende um pouco para tras junto com a pelve (a
        // cabeca devolve, o olhar fica no horizonte), nao cai para a frente.
        const float flexion = -0.15f * m_brakeReaction.x;
        const float lateral = -0.08f * m_brakeReaction.y;
        const float amount = m_telemetry.brakeReaction;
        addJointCoordinate(profile, m_coordinates, "Abdomen", 1,
            flexion * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 1,
            flexion * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 1,
            flexion * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Abdomen", 2,
            lateral * 0.30f);
        addJointCoordinate(profile, m_coordinates, "Chest", 2,
            lateral * 0.35f);
        addJointCoordinate(profile, m_coordinates, "UpperChest", 2,
            lateral * 0.35f);
        addJointCoordinate(profile, m_coordinates, "Neck", 1,
            -flexion * 0.40f);
        addJointCoordinate(profile, m_coordinates, "Head", 1,
            -flexion * 0.25f);
        addJointCoordinate(profile, m_coordinates, "LeftForearm", 0,
            -0.35f * std::min(1.0f, amount));
        addJointCoordinate(profile, m_coordinates, "RightForearm", 0,
            -0.35f * std::min(1.0f, amount));
    }
    // Reflexos de queda por cima da pose (e, no comeco do deitado, por cima
    // da pose deitada: as maos ainda seguram o impacto e depois relaxam).
    {
        float reflex = m_fallReflex;
        float legReflex = std::min(m_fallReflex, m_fallCommit);
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            // No chao os reflexos da queda (que miram o rumo dela, sem
            // sentido rolando no chao - os alvos pulavam e ele parecia
            // convulsionar) saem em 0,3 s; quem assume e a posicao fetal
            // (abaixo).
            reflex *= GroundAlive ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.3f)
                : 1.0f - smoothStep(m_getUpPhaseSeconds / 0.6f);
            legReflex = reflex;
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            reflex = 0.0f;
            legReflex = 0.0f;
        }
        applyFallReflexPose(profile, state, m_coordinates, m_handLinks,
            m_chestLink, m_fallDirection, groundHeightGuess, reflex, legReflex,
            m_handCatchSeconds, m_elapsedSeconds, input.varietySeed);
        // Vida: esperneando no ar caindo (sem tocar nada ha ~0,1 s).
        const float flail = m_getUpPhase == CharacterGetUpPhase3D::None
            ? m_fallReflex * smoothStep((m_noGroundSeconds - 0.10f) / 0.20f)
            : 0.0f;
        m_airBalance = AirBalanceNew ? flail : 0.0f;
        // O giro do corpo (no eixo de lado do peito): sentido e forca do moinho.
        float bodySpin = 0.0f;
        if (m_chestLink < state.links.size()) {
            const Vec3 chestLeft = state.links[m_chestLink].orientation.rotate({ 0.0f, 1.0f, 0.0f });
            bodySpin = dot(state.links[m_chestLink].angularVelocity, chestLeft);
        }
        if (AirBalanceNew) applyAirBalance(profile, m_coordinates, m_elapsedSeconds, flail, bodySpin);
        else applyAirborneFlail(profile, m_coordinates, m_elapsedSeconds, flail);
        // No chao: posicao fetal enquanto ainda nao se arruma para levantar
        // (entra em ~0,3 s e da lugar a pose de levantar quando ele se
        // arruma).
        // (De costas ou de lado; de bruços ele nao se enrola sobre a barriga
        // - protege a cabeca com os bracos e apoia as maos, que ja e a pose de
        // onde o levantar de bruços parte. Enrolado de bruços, o levantar
        // tinha de desmontar a pose e levava 8,6 s em vez de ~5.)
        const float fetalWanted = FetalEnabled && m_getUpPhase == CharacterGetUpPhase3D::Settling
                && m_fallOrientation == CharacterFallOrientation3D::FaceUp
            ? 1.0f - m_settleGather : 0.0f;
        m_fetal += (fetalWanted - m_fetal) * (1.0f - std::exp(-deltaTime / 0.12f));
        {
            float coreSpeedNow = state.links.empty() ? 0.0f : state.links.front().linearVelocity.length();
            applyFetalPose(profile, state, m_coordinates, m_handLinks, m_chestLink,
                smoothStep(m_fetal) * (m_getUpPhase == CharacterGetUpPhase3D::Settling ? 1.0f : 0.0f),
                m_elapsedSeconds, input.varietySeed, std::clamp(coreSpeedNow / 1.5f, 0.0f, 1.0f));
        }
        m_telemetry.lifeFlail = flail;
        // Perdendo o equilibrio, de pe: inclinado alem da pose (a partir de
        // ~12 graus) ou atingido (contato externo recente, o corpo indo para
        // onde o jogador nao pediu). Sai quando o reflexo de queda assume.
        float balanceWanted = 0.0f;
        Vec3 balanceDirection = m_balanceDirection;
        if (m_getUpPhase == CharacterGetUpPhase3D::None && !state.links.empty()
            && desiredState != CharacterLocomotionState3D::Airborne) {
            const Vec3 up = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            Vec3 lean = up - m_previousTargetUp;
            lean.z = 0.0f;
            const float tiltError = std::acos(std::clamp(
                dot(up, m_previousTargetUp), -1.0f, 1.0f));
            const float tiltTerm = smoothStep((tiltError - 0.21f) / 0.35f);
            // Movimento nao pedido: ir mais devagar (ou bloqueado) no sentido
            // pedido nao conta - correndo contra uma parede ele nao gira os
            // bracos; ser jogado de lado, para tras ou estando parado, conta.
            Vec3 bodyVelocity = m_filteredCenterOfMassVelocity;
            bodyVelocity.z = 0.0f;
            Vec3 wanted = input.proxyVelocityWorld;
            wanted.z = 0.0f;
            Vec3 drift = bodyVelocity;
            if (wanted.length() > 0.2f) {
                const Vec3 along = wanted.normalized();
                drift -= along * std::clamp(dot(bodyVelocity, along), 0.0f,
                    wanted.length() + 0.5f);
            }
            const float hitTerm = std::clamp(input.externalPerturbation, 0.0f, 1.0f)
                * smoothStep((drift.length() - 0.3f) / 1.0f);
            balanceWanted = std::max(tiltTerm, hitTerm) * (1.0f - m_fallReflex);
            Vec3 direction {};
            if (lean.lengthSquared() > 1e-6f) direction += lean.normalized() * tiltTerm;
            if (drift.lengthSquared() > 1e-6f) direction += drift.normalized() * hitTerm;
            if (direction.lengthSquared() > 1e-6f) balanceDirection = direction.normalized();
            // Tropeco: os bracos saem do balanco comum e abrem/giram para
            // administrar o momento. O reflexo existente gera essa resposta
            // e continua livre para virar protecao das maos se o passo falhar.
            if (m_tripRecovery > balanceWanted) {
                balanceWanted = std::max(balanceWanted,
                    0.75f * m_tripRecovery) * (1.0f - m_fallReflex);
                balanceDirection = m_tripDirection;
            }
            // Quase-queda em movimento: os bracos abrem para administrar o
            // momento (referencia Jogging Stumble: mao a altura do ombro, 37
            // cm para o lado).
            if (StumbleArmsScale * m_stumbleReaction > balanceWanted) {
                balanceWanted = StumbleArmsScale * m_stumbleReaction * (1.0f - m_fallReflex);
                balanceDirection = m_stumbleDirection;
            }
        }
        m_balanceReaction += (balanceWanted - m_balanceReaction) * (1.0f
            - std::exp(-deltaTime * (balanceWanted > m_balanceReaction ? 12.0f : 3.0f)));
        {
            const Vec3 blended = m_balanceDirection + (balanceDirection - m_balanceDirection)
                * (1.0f - std::exp(-8.0f * deltaTime));
            if (blended.lengthSquared() > 1e-6f) m_balanceDirection = blended.normalized();
        }
        std::array<bool, 2> braced {};
        for (const RagdollInteraction3D& contact : state.interactions) {
            if (contact.normalWorld.z <= 0.3f || contact.linkIndex >= profile.links.size()) continue;
            for (std::size_t side = 0; side < 2; ++side) {
                const std::size_t hand = m_handLinks[side];
                if (hand == 0 || hand >= profile.links.size()) continue;
                if (contact.linkIndex == hand || static_cast<int>(contact.linkIndex)
                        == profile.links[hand].parentIndex) braced[side] = true;
            }
        }
        const float gaitReaction = gaitMoving ? BalanceGaitShare : 1.0f;
        applyBalanceReaction(profile, state, m_coordinates, m_handLinks,
            m_chestLink, m_balanceDirection, m_balanceReaction * gaitReaction, m_elapsedSeconds,
            braced, m_balanceReaction * (gaitMoving && BalanceGaitNoLegs ? 0.0f : gaitReaction));
        m_telemetry.balanceReaction = m_balanceReaction;
    }
    // Chute: bracos e tronco do chute por cima.
    applyKickPose(profile, state, m_coordinates, m_handLinks, input.ballKick);
    // Manejo de bola (rework de 06/10): tronco, bracos e cabeca do gesto por
    // cima (os pes vao pelo planejador: ballFoot/ballScript).
    if (input.ballGesture.active && !state.links.empty()) {
        if (!m_gestureLegs.valid) m_gestureLegs = resolveBallGestureLegs3D(profile);
        Vec3 rootPosition = state.links.front().position;
        Quaternion rootOrientation = state.links.front().orientation;
        applyBallGesturePose3D(profile, m_gestureLegs, input.ballGesture, m_coordinates, rootPosition,
            rootOrientation, false, false);
    }

    // Banking into a turn: a runner carving a curve leans into it and
    // leans forward a little more, roughly in line with the lean/twist
    // magnitudes measured off a real running-turn reference (calibrated by
    // hand against that capture, not sampled from it — see
    // assets/animations/source/mixamo/README.md).
    // v * yawRate is centripetal acceleration. Small curves get a soft
    // onset, and rotating in place does not bank like a running turn.
    const float curveAcceleration = signedCurveAcceleration;
    t.curveShare = smoothStep(std::abs(curveAcceleration) / 3.0f);
    const float& curveShare = t.curveShare;
    const float turnBank = std::atan(curveAcceleration / 9.81f)
        * 0.72f * curveShare;
    // Freando, a pelve inclina para tras so 15% do que inclinaria (a
    // inclinacao satura: com 40%, uma freada de sprint ainda dava os 12,6
    // graus inteiros): o tronco vai para a frente pela inercia (bloco da
    // freada, acima).
    // Freando, a pelve inclina para tras na proporcao da desaceleracao (ate
    // ~9 graus): o corpo fica atras dos pes para frear (era 15% disso, e o
    // tronco ia para a frente pela inercia).
    const Vec3 brakingAcceleration = m_brakeDirection
        * std::min(0.0f, dot(m_smoothedAcceleration, m_brakeDirection));
    (void)brakingAcceleration;
    const Vec3 localAcceleration = heading.conjugate().rotate(m_smoothedAcceleration);
    // Arrancando, inclina menos (ate ~7 graus, era 12,6: somado a pose da
    // corrida, o peito passava de 30 graus no comeco do trote/sprint).
    const float wantedForwardLean = std::clamp(
        std::atan(localAcceleration.x / 9.81f) * 0.22f
            + std::abs(turnBank) * 0.08f
            + startupRootLean,
        -0.16f, 0.20f);
    // De lado (curva, troca de rumo) pela velocidade: andando e trotando,
    // pouco; so correndo rapido a inclinacao inteira (ate ~11 graus, era 15
    // em qualquer velocidade: mexendo a camera andando ele tombava de lado).
    const float bankSpeedShare = 0.25f + 0.75f * smoothStep((speed - 2.5f) / 4.0f);
    // Curva: o corpo inteiro inclina para dentro como a fisica pede, theta =
    // atan(a_c/g) pela aceleracao centripeta do caminho (na referencia
    // Sprint Turn, do pe de apoio ao quadril: 24 graus a 4,1 m/s2 - atan
    // 23). O pe pousa para fora (bloco do pouso), a pelve rola uma parte
    // (11-19 graus la) e o tronco vai junto com ela. Andando devagar quase
    // nada: girando a camera andando ele tombava de lado.
    // (Pela taxa do caminho, que so existe com pedido e sem freada: a
    // desaceleracao forte, um pouco desalinhada da velocidade, parecia curva
    // - 5 graus de 12 m/s2 sao ~1 m/s2 de lado - e ele inclinava e jogava o
    // pe para fora nos passos de freada.)
    m_bodyCurveLean = CurveNew ? std::clamp(std::atan(pathSpeed * pathTurnRate / 9.81f),
        -CurveLeanMaximum, CurveLeanMaximum) * smoothStep((speed - 1.2f) / 1.6f) : 0.0f;
    // (A pelve rola menos no sprint, TEMPTUNE CurveRootSprint: rolando, o
    // quadril de fora subia e a perna de fora nao alcancava o chao - a de
    // dentro fazia 70% dos apoios, galopando. A parte que a pelve deixa vai
    // pela coluna (bloco do tronco). No trote ela rola inteira: menos, o
    // joelho de apoio dobrava 7 graus a mais na curva.)
    m_curveSprintShare = smoothStep((speed - CurveRootSprintFrom) / std::max(0.1f, CurveRootSprintSpan));
    const float curveRootShare = CurveRootShare + (CurveRootSprint - CurveRootShare) * m_curveSprintShare;
    const float wantedLateralLean = CurveNew
        ? std::clamp(curveRootShare * m_bodyCurveLean, -0.30f, 0.30f)
        : std::clamp(
        (std::atan(localAcceleration.y / 9.81f) * 0.18f + turnBank) * bankSpeedShare,
        -0.19f, 0.19f);
    // Exact critically damped spring: continuous lean velocity on turn
    // reversal without delaying the capsule or changing running speed.
    const Vec3 leanTarget { wantedForwardLean, wantedLateralLean, 0.0f };
    const Vec3 leanError = m_lean - leanTarget;
    const Vec3 leanImpulse = m_leanVelocity + leanError * 22.0f;
    const float leanDecay = std::exp(-22.0f * deltaTime);
    m_lean = leanTarget + (leanError + leanImpulse * deltaTime) * leanDecay;
    m_leanVelocity = (m_leanVelocity - leanImpulse * (22.0f * deltaTime)) * leanDecay;
    // Longitudinal acceleration is already filtered upstream. A second
    // lag here moves the hip behind the foot plan when descending steps.
    t.forwardLean = wantedForwardLean;
    float& forwardLean = t.forwardLean;
    t.lateralLean = m_lean.y;
    float& lateralLean = t.lateralLean;
    // v2 (T_LeanNeed): com pouca ajuda quem acelera o corpo sao as pernas, e
    // o chao so empurra para a frente sem girar o corpo com o centro de massa
    // a frente do apoio (a inclinacao atan(a/g)). A referencia inclinava so
    // pela aceleracao da capsula (22% dela) - presa ao corpo, quase nada: o
    // tronco ficava reto e o corpo nao passava de ~1,5 m/s sem ajuda. Inclina
    // pela aceleracao que falta (o pedido menos o corpo, no que as pernas
    // conseguem), na parte das pernas (1 - ajuda andando).
    {
        static const float leanNeedGain = tuneTemp("T_LeanNeed", 0.0f);  // TEMPTUNE
        static const float leanNeedRate = tuneTemp("T_LeanNeedRate", 1.5f);  // TEMPTUNE (1/s)
        static const float leanNeedMaximum = tuneTemp("T_LeanNeedMax", 5.0f);  // TEMPTUNE (m/s2)
        static const float leanNeedPower = tuneTemp("T_LeanNeedPower", 18.0f);  // TEMPTUNE (W/kg)
        const bool gaitNow = desiredState == CharacterLocomotionState3D::Walking
            || desiredState == CharacterLocomotionState3D::Running;
        Vec3 wanted {};
        if (leanNeedGain > 0.0f && input.forceDrivenRoot && gaitNow && !getUpActive) {
            const Vec3 requested { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
            const Vec3 body { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
            Vec3 need = clampMagnitude((requested - body) * leanNeedRate, leanNeedMaximum);
            // Ganhar velocidade correndo rapido custa mais (a potencia): ~3
            // m/s2 a 6 m/s.
            const float speed = body.length();
            if (speed > 0.5f) {
                const Vec3 unit = body * (1.0f / speed);
                const float along = dot(need, unit);
                if (along > 0.0f) {
                    const float limit = leanNeedPower / std::max(1.0f, speed);
                    if (along > limit) need -= unit * (along - limit);
                }
            }
            const float legs = 1.0f - std::clamp(input.walkingAssist, 0.0f, 1.0f);
            const Vec3 local = heading.conjugate().rotate(need) * (legs * leanNeedGain);
            wanted = { std::atan(local.x / 9.81f), std::atan(local.y / 9.81f), 0.0f };
        }
        m_leanNeed += (wanted - m_leanNeed) * (1.0f - std::exp(-deltaTime / 0.12f));
        forwardLean = std::clamp(forwardLean + m_leanNeed.x, -0.30f, 0.45f);
        lateralLean = std::clamp(lateralLean + m_leanNeed.y, -0.35f, 0.35f);
    }

    // O gancho do equilibrio (o RL): a inclinacao do tronco somada (sem
    // correcao, a mesma).
    forwardLean = correctedForwardLean3D(forwardLean, input.corrections);
    lateralLean = correctedLateralLean3D(lateralLean, input.corrections);
}

} // namespace MatterEngine
