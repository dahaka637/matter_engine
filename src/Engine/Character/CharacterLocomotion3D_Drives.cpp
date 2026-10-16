#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepDrives(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const std::vector<Vec3>& previousCoordinates = t.previousCoordinates;
    const bool& getUpActive = t.getUpActive;
    const Quaternion& heading = t.heading;
    const bool& gaitMoving = t.gaitMoving;
    const float& transition = t.transition;
    const Quaternion& targetRootOrientation = t.targetRootOrientation;
    const float& lowAssist = t.lowAssist;
    Vec3& targetRootPosition = t.targetRootPosition;
    RagdollAnimationPose3D& localPose = t.localPose;
    const float& baseGround = t.baseGround;
    const bool& footworkEnabled = t.footworkEnabled;
    const bool& onGround = t.onGround;
    float& drop = t.drop;
    const auto groundUnder = [&](std::size_t side, Vec3 center,
        Vec3 forward) -> GroundSample {
        GroundSample result;
        if (input.groundAt) {
            // A sonda procura a partir da altura do chao da capsula (quem
            // implementa groundAt comeca um pouco acima dela): uma viga na
            // altura do peito nao vira chao.
            center.z = baseGround;
            forward.z = 0.0f;
            forward = forward.lengthSquared() > 0.000001f
                ? forward.normalized() : heading.rotate({ 1.0f, 0.0f, 0.0f });
            for (const float reach : { -0.09f, 0.0f, 0.09f }) {
                const GroundProbeResult3D probe =
                    input.groundAt(center + forward * reach);
                if (!probe.hasSurface || !probe.walkable) continue;
                // Evaluate the surface plane at the sole center: selecting
                // the highest toe sample adds artificial height on ramps.
                const float height = probe.pointWorld.z
                    - (probe.normalWorld.x * (center.x - probe.pointWorld.x)
                        + probe.normalWorld.y * (center.y - probe.pointWorld.y))
                        / std::max(0.2f, probe.normalWorld.z);
                if (!result.valid || height > result.height) {
                    result.valid = true;
                    result.height = height;
                    result.normal = probe.normalWorld;
                }
            }
            if (result.valid) return result;
        }
        const GroundProbeResult3D& legacy = input.footGround[side];
        result.valid = true;
        if (legacy.hasSurface) {
            result.height = legacy.pointWorld.z;
            result.normal = legacy.normalWorld;
        } else {
            result.height = baseGround;
        }
        return result;
    };
    const auto lockHeight = [&](const FootPlant& foot, float ground,
        Quaternion orientation) {
        const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
        const Quaternion collider = (orientation
            * link.collider.localOrientation).normalized();
        return ground + lowestColliderOffset(link, collider)
            - orientation.rotate(link.collider.localPosition).z + 0.001f;
    };
    const auto forwardOf = [&](const FootPlant& foot, Quaternion orientation) {
        return orientation.rotate(profile.links[foot.linkIndex]
            .modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
    };
    // Ground the authored pose before anything is driven from it. The root
    // height comes from the capsule plus the clip's own root track, and the
    // two do not have to agree: measured on the idle clip, the right sole
    // was being commanded 3.4 cm BELOW the floor. The floor answers that by
    // shoving the foot back up, and the leg above buckles to absorb it -
    // which is the crooked-legged stance, not weak motors. Raising the root
    // so the lowest authored sole rests exactly on the ground removes the
    // fight at its source; it never lowers the pose, so a genuine airborne
    // frame is untouched.
    if (!getUpActive && !input.forceDrivenRoot) {
        float deepest = 0.0f;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const FootPlant& foot = m_feet[side];
            if (!foot.valid || foot.linkIndex >= localPose.linkPositions.size()) {
                continue;
            }
            // Com os pes no terreno, so o pe de apoio conta: o pe em balanco
            // tem a propria folga sobre o chao (ver "Pes no terreno"). Contar
            // ele erguia o corpo inteiro quando passava rente a quina do
            // degrau seguinte, e o pe de apoio deixava de alcancar o dele
            // (medido: pelve 16 cm acima, pe de apoio pendurado 18 cm).
            if (footworkEnabled && onGround && !foot.planted) continue;
            // O chao alvo deste pe (terreno, ver "Pes no terreno"); fora do
            // chao, a base da capsula.
            const float floorZ = footworkEnabled && onGround
                ? foot.groundHeight : baseGround;
            const RagdollLinkDefinition3D& link = profile.links[foot.linkIndex];
            const Quaternion colliderWorld = (targetRootOrientation
                * localPose.linkOrientations[foot.linkIndex]
                * link.collider.localOrientation).normalized();
            const Vec3 colliderCentre = targetRootPosition
                + targetRootOrientation.rotate(
                    localPose.linkPositions[foot.linkIndex]
                    + localPose.linkOrientations[foot.linkIndex].rotate(
                        link.collider.localPosition));
            const float soleWorld = colliderCentre.z
                - lowestColliderOffset(link, colliderWorld);
            deepest = std::min(deepest, soleWorld - floorZ);
        }
        // Rede de seguranca, suavizada: sobe rapido e desce devagar. Aplicada
        // num tick so, qualquer diferenca entre a pose e o chao virava um
        // degrau na pelve (e o corpo inteiro tremia com ele).
        const float lift = std::max(0.0f, -deepest);
        const float response = lift > m_groundLift ? 40.0f : 10.0f;
        m_groundLift += (lift - m_groundLift)
            * (1.0f - std::exp(-response * deltaTime));
        targetRootPosition.z += m_groundLift;
    }

    m_output.targetPose = localPose;
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        m_output.targetPose.linkPositions[index] = targetRootPosition
            + targetRootOrientation.rotate(localPose.linkPositions[index]);
        m_output.targetPose.linkOrientations[index] =
            (targetRootOrientation * localPose.linkOrientations[index])
                .normalized();
    }
    m_referencePendulumValid = m_feet.size() >= 2 && m_feet[0].valid && m_feet[1].valid
        && m_feet[0].linkIndex < localPose.linkPositions.size() && m_feet[1].linkIndex < localPose.linkPositions.size();
    if (m_referencePendulumValid) {
        m_referencePendulum = targetRootOrientation.rotate(
            pendulumArm(profile, m_feet[0].linkIndex, m_feet[1].linkIndex,
                [&](std::size_t i) { return localPose.linkPositions[i]; }));
        float total = 0.0f;
        Vec3 center {};
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            center = center + m_output.targetPose.linkPositions[i] * profile.links[i].massFraction;
            total += profile.links[i].massFraction;
        }
        m_referenceCenterOfMassWorld = center * (1.0f / std::max(1e-4f, total));
        m_referenceFeetWorld = { m_output.targetPose.linkPositions[m_feet[0].linkIndex],
            m_output.targetPose.linkPositions[m_feet[1].linkIndex] };
    }

    t.muscle = std::clamp(input.muscleAuthority, 0.0f, 1.0f);
    const float& muscle = t.muscle;
    // A real hit lets the joints actually give, not just the root guide
    // above: this is what makes falling possible at all instead of the
    // legs snapping back to the walking pose no matter how hard something
    // hits the character. Get-up phases carry their own tuned profile
    // below instead (including a near-limp Resting), so this only softens
    // normal locomotion.
    // Cede na pancada, mas nunca vira corpo morto: minimo de 35% (era 3%).
    // Com a raiz livre (modo fisico) as pancadas nao passam por aqui (a
    // cedencia e local, por junta, do controlador fisico) - so a PhysGun
    // ainda subia esta mistura, e agarrado o corpo inteiro ia a 35% da
    // forca (e assim ficava ~1,5 s depois de soltar): arrastado, a perna
    // nao erguia o pe e ele caia (varredura de arrasto: 20 de 32).
    const float impactSoftening = getUpActive || input.forceDrivenRoot ? 1.0f
        : std::clamp(1.0f - m_reactionStrength * 0.97f, 0.35f, 1.0f);
    t.squaredError = 0.0f;
    float& squaredError = t.squaredError;
    t.errorCount = 0;
    std::uint32_t& errorCount = t.errorCount;
    std::array<bool, 2> physicalSupport {};
    for (const auto& contact : state.interactions) {
        if (contact.normalWorld.z < 0.5f) continue;
        for (std::size_t side = 0; side < m_feet.size(); ++side)
            if (contact.linkIndex == m_feet[side].linkIndex) physicalSupport[side] = true;
    }
    std::array<float, 2> touchdownShare {};
    if (input.forceDrivenRoot && !getUpActive) {
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const auto& foot = m_feet[side];
            if (!foot.valid || foot.linkIndex >= state.links.size()) continue;
            const auto& body = state.links[foot.linkIndex];
            const auto ground = groundUnder(side, body.position,
                forwardOf(foot, body.orientation));
            if (!ground.valid || body.linearVelocity.z > 0.15f) continue;
            // Um passo do planejador ainda no comeco do balanco nao esta
            // pousando: num passo de marcha baixo (8 cm) o corte valia o
            // balanco inteiro, a perna seguia so por posicao, atrasada, e o
            // pe subia metade do pedido e pousava curto.
            const FootPlan3D& planFoot = m_footwork.plan().feet[side];
            if (!planFoot.followingGait && planFoot.phase == FootPhase3D::Swing
                && planFoot.swingSeconds < 0.8f * planFoot.swingDurationSeconds) continue;
            // Passo de marcha (E5): a perna desce no tempo do plano ate o
            // contato - o amortecimento e da fase de carga, depois do toque.
            // Com o corte (velocidade-alvo zerada e mola 30% mais mole nos
            // ultimos 20%), no passo lateral o joelho estendia ~80 ms atrasado,
            // o pe ficava 3 cm acima com o alvo ja no chao e o pouso atrasava
            // 0,15 s procurando.
            if (!planFoot.followingGait && planFoot.reason == StepReason3D::Locomotion
                && (planFoot.phase == FootPhase3D::Swing || planFoot.phase == FootPhase3D::TouchdownSearch))
                continue;
            const float clearance = body.position.z
                - lockHeight(foot, ground.height, body.orientation);
            touchdownShare[side] = smoothStep((0.12f - clearance) / 0.12f);
        }
    }
    // Caindo e no chao, os alvos das juntas sao suavizados (~0,12 s): as
    // poses de queda trocam de alvo de repente (rumo da queda, mao tocando,
    // fetal, arrumar) e, sem isso, os membros davam trancos - parecia
    // convulsao. Sai aos poucos no comeco do levantar.
    {
        const float rising = m_getUpPhase == CharacterGetUpPhase3D::Rising
            ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.4f) : 0.0f;
        // (Pernas e tronco so com a queda sem volta: no desequilibrio os
        // passos que ainda salvam nao podem atrasar. Bracos e pescoco ja com
        // o reflexo.)
        // (Deitado o filtro sai conforme ele se arruma: o levantar parte dos
        // alvos exatos da pose de partida.)
        const float lying = std::max(m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f - m_settleGather : 0.0f, rising);
        if (m_fallSmoothCoordinates.size() != m_coordinates.size())
            m_fallSmoothCoordinates = m_coordinates;
        const float blend = 1.0f - std::exp(-deltaTime / std::max(FallSmoothSeconds, 1e-3f));
        for (std::size_t index = 1; index < m_coordinates.size(); ++index) {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos || id.find("Hand") != std::string::npos
                || id == "Neck" || id == "Head";
            // (Levantando, so a transicao do comeco: o resto do reflexo, que
            // leva ~1 s para sair, atrasava o clipe de levantar e o corpo
            // brigava com ele - de bruços ficava ~2 s sem sair do chao.)
            const float smoothing = FallSmoothSeconds <= 0.0f ? 0.0f
                : m_getUpPhase == CharacterGetUpPhase3D::Rising ? rising
                : std::max(lying, reflexLimb ? m_fallReflex : m_fallCommit);
            m_fallSmoothCoordinates[index] += (m_coordinates[index] - m_fallSmoothCoordinates[index]) * blend;
            if (smoothing > 0.0f)
                m_coordinates[index] += (m_fallSmoothCoordinates[index] - m_coordinates[index]) * smoothing;
            else
                m_fallSmoothCoordinates[index] = m_coordinates[index];
        }
    }
    // Levantar pelo corpo: as juntas sao as dele (BodilyGetUp3D).
    const bool bodilyRising = m_bodilyRising
        && m_getUpPhase == CharacterGetUpPhase3D::Rising
        && m_bodilyPose.coordinates.size() == m_coordinates.size();
    if (bodilyRising) {
        for (std::size_t index = 1; index < m_coordinates.size(); ++index)
            m_coordinates[index] = m_bodilyPose.coordinates[index];
    }
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        m_coordinateVelocities[index] =
            (m_coordinates[index] - previousCoordinates[index]) / deltaTime;
        const Vec3 measured {
            state.joints[index].positionRadians[0],
            state.joints[index].positionRadians[1],
            state.joints[index].positionRadians[2]
        };
        const Vec3 velocity = jointAngularVelocity(m_coordinates[index],
            m_coordinateVelocities[index], measured);
        const auto& linkId = profile.links[index].id;
        const bool leg = linkId.find("Thigh") != std::string::npos
            || linkId.find("Shin") != std::string::npos
            || linkId.find("Foot") != std::string::npos;
        const std::size_t side = linkId.starts_with("Left") ? 0 : 1;
        // On a loaded foot, discontinuities in IK velocity become a kick
        // against the floor. Position drives retain support; damping absorbs
        // landing energy instead of feeding it back through the velocity goal.
        // (A perna em balanco do planejador que ainda raspa o chao no comeco
        // do passo, solta SEM carga, nao esta apoiada: sem a velocidade-alvo
        // ela seguia so por posicao, atrasada. Solta com carga - a soltura de
        // emergencia da recuperacao -, a velocidade seria o coice contra o
        // chao: no modo so pelas pernas, 26 -> 50 quedas.)
        const FootPlan3D& legPlan = m_footwork.plan().feet[side];
        const bool locomotionStep = legPlan.reason == StepReason3D::Locomotion
            && (legPlan.phase == FootPhase3D::Swing || legPlan.phase == FootPhase3D::TouchdownSearch);
        // (R2 do pacote v2: na marcha pelo planejador, quem deveria liberar a
        // velocidade do balanco e o pe no ar de fato - physicalRelease -, nao
        // a carga estimada na soltura. Medido antes de corrigir o voo curto:
        // misto, inversao 1 -> 3 quedas. Volta depois da R4.)
        const bool releasedForSwing = legPlan.releasedUnloaded;
        const bool plannedSwing = leg && !legPlan.followingGait && releasedForSwing
            && ((legPlan.phase == FootPhase3D::Swing
                    && legPlan.swingSeconds < 0.8f * legPlan.swingDurationSeconds)
                || locomotionStep);
        // Empurrao antes do toque (E5, fase 7): com o pe que vem no ultimo
        // terco do balanco (ou procurando o chao), a perna de apoio segue a
        // velocidade-alvo da referencia em vez de zero - com zero, o motor
        // amortecia o proprio pendulo e freava o corpo justo antes do pouso.
        // Fora dessa janela o apoio segue com zero (o H17 tirava o freio no
        // apoio inteiro e o corpo passava da velocidade pedida).
        // (So empurra com o corpo abaixo da velocidade da referencia no rumo
        // dela: ja na velocidade, empurrar sempre o levava alem do que a
        // marcha segura - 1 m/s: 2 -> 4 quedas; 1,5 m/s: 6 -> 10.)
        const FootPlan3D& otherPlan = m_footwork.plan().feet[1 - side];
        bool belowReference = false;
        if (m_bodyRefValid) {
            const float referenceSpeed = std::hypot(m_bodyRefVelocity.x, m_bodyRefVelocity.y);
            const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
            belowReference = referenceSpeed > 0.1f
                && (com.x * m_bodyRefVelocity.x + com.y * m_bodyRefVelocity.y) / referenceSpeed < referenceSpeed;
        }
        const bool propulsive = leg && belowReference && m_footwork.settings().locomotionStepping && !legPlan.followingGait
            && legPlan.phase == FootPhase3D::Stance && otherPlan.reason == StepReason3D::Locomotion
            && (otherPlan.phase == FootPhase3D::TouchdownSearch
                || (otherPlan.phase == FootPhase3D::Swing
                    && otherPlan.swingSeconds >= LocomotionPushWindow * otherPlan.swingDurationSeconds));
        // Pe com mola: o tornozelo de um pe no ar (sem contato nenhum) so
        // carrega o proprio pe (~1 kg) - nada a amortecer contra o chao. Ele
        // segue o alvo (posicao e velocidade) ate o toque: com a perna toda
        // amolecida perto do chao e a velocidade-alvo cortada, o tornozelo
        // virava um freio de ~0,3 s e chegava 13-23 graus atrasado no sprint
        // (o pe nao pousava como a animacao e o terreno pediam).
        const FootSupportEstimate3D& footEstimate = m_estimator.state().feet[side];
        const bool ankle = linkId.find("Foot") != std::string::npos;
        const bool footInAir = input.footSpring && input.forceDrivenRoot && !getUpActive
            && !footEstimate.contactObserved && !footEstimate.supporting;
        const bool agileAnkle = footInAir && ankle && gaitMoving;
        // O joelho tambem: amolecido e sem velocidade-alvo nos ultimos 12 cm,
        // ele chegava ao toque 17-23 graus mais dobrado que o alvo - a canela
        // inclinada para tras e o pe pousando com a ponta para baixo mesmo
        // com o tornozelo certo. Ele so cede depois do contato (aceitacao de
        // peso, pouso forte).
        // (So na passada: caindo de um pulo o joelho chega macio - rigido,
        // o pouso quicava, rebote 1,07 -> 1,31 m/s.)
        const bool agileKnee = footInAir && gaitMoving && linkId.find("Shin") != std::string::npos;
        // TEMPTUNE: empurrao - o pe apoiado ja atras do corpo (a segunda
        // metade do apoio, ate a ponta sair). A perna estende com mais forca
        // e segue a velocidade do alvo: mole e freada (velocidade-alvo zero),
        // ela ficava 15-20 graus atras do pedido e saia do chao dobrada (o
        // joelho a ~34 graus no trote). O pouso continua macio: rigida nele,
        // a perna quicava nas curvas.
        bool pushPhase = false;
        float pushWeight = 0.0f;
        // (So na corrida estabelecida, com o jogador pedindo para correr:
        // freando, a perna de tras rigida empurrava o corpo por cima do pe
        // de freada - 5 -> 14 quedas em 256 paradas.)
        // (Nem logo depois do pouso de um pulo: o pe freia, o corpo gira
        // para a frente sobre ele e a perna rigida empurrando por tras o
        // jogava de cara no chao - pulo em sprint, 0 -> 5 quedas em 12.)
        const bool steadyRun = m_brakeGait <= 0.0f && !m_brakeStepping && !m_startClipActive
            && m_gaitChange <= 0.0f && m_tripRecovery <= 0.001f && !m_stumbleStepping
            && m_sinceTouchdownSeconds > PushAfterLandingSeconds
            && std::hypot(input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y) > 1.0f;
        if (leg && gaitMoving && input.forceDrivenRoot && !getUpActive && footEstimate.supporting
            && PushStiffness != 1.0f && steadyRun) {
            const CharacterPhysicalState3D& body = m_estimator.state();
            const Vec3 velocity { body.centerOfMassVelocityWorld.x, body.centerOfMassVelocityWorld.y, 0.0f };
            const float speedNow = velocity.length();
            if (speedNow > 1.0f) {
                const float behind = -dot(Vec3 { footEstimate.soleCenterWorld.x - body.centerOfMassWorld.x,
                    footEstimate.soleCenterWorld.y - body.centerOfMassWorld.y, 0.0f },
                    velocity * (1.0f / speedNow));
                pushPhase = behind > PushBehindMeters;
                // TEMPTUNE: em rampa (do pe embaixo do corpo ate 10 cm atras
                // do limite), nao num degrau: ligada de uma vez, a rigidez
                // dobrava num tick e a perna dava um tranco.
                pushWeight = PushRamp ? smoothStep((behind - (PushBehindMeters - 0.05f)) / 0.10f)
                    : (pushPhase ? 1.0f : 0.0f);
                pushPhase = pushWeight > 0.0f;
            }
        }
        const float velocityShare = input.forceDrivenRoot && !getUpActive && leg
            ? (agileAnkle || agileKnee || (pushPhase && PushVelocity) ? transition
                : physicalSupport[side] && !plannedSwing && !propulsive ? 0.0f
                : transition * (propulsive ? 1.0f : 1.0f - touchdownShare[side])) : 1.0f;
        // Same per-category get-up gains as
        // BiomechanicalBipedExperiment3D's: soft and passive while
        // Resting, firm (legs firmest) while actively recovering.
        // Default mode holds the pose as hard as the drives allow, and lets
        // go in step with physicsBlend. This is the "force the pose when
        // nothing is happening, yield when something is" dial: at blend 0
        // the motors run at their ceiling, at blend 1 they are back to the
        // profile's nominal strength and the body argues back.
        float poseHold = 1.0f + (1.0f - m_physicsBlend);
        // No modo fisico o corpo e sempre o fisico: sem "segurar a pose" no
        // dobro da forca (herdado do modo antigo). Mais macio, ele sente as
        // pancadas (esbarrar numa parede, num boneco) em vez de rebater duro.
        if (input.forceDrivenRoot) poseHold = UprightMuscleTone;
        // Caindo: tonus de quem se protege - firme o bastante para os
        // reflexos agirem, sem a rigidez de quem esta de pe. Bracos e
        // pescoco rapidos (as maos tem de chegar ao chao antes da cabeca).
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id == "Neck" || id == "Head";
            // Tronco e pernas amolecem so com a queda sem volta; os bracos
            // e o pescoco ja com o reflexo.
            const bool legLink = id.find("Thigh") != std::string::npos
                || id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos;
            const bool armLink = id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos;
            // A mao do lado segurando o chao: o braco firma um pouco para
            // amortecer (cede pelo alvo do cotovelo, nao mole).
            const bool catching = FallProtectNew && armLink
                && m_handCatchSeconds[id.rfind("Left", 0) == 0 ? 0 : 1] > 0.0f;
            const float fallTone = !FallProtectNew ? (reflexLimb ? FallReflexLimbTone : FallBodyTone)
                : catching ? FallCatchTone : reflexLimb ? FallReflexLimbTone
                : legLink ? (FallLegsNew ? FallLegTone : FallBodyTone) : FallTrunkTone;
            poseHold += (fallTone - poseHold) * (reflexLimb ? m_fallReflex : m_fallCommit);
            // No ar a pose e firme (os membros nao carregam peso): bracos e
            // pernas seguram a pose de equilibrio, sem chacoalhar.
            if ((armLink || legLink) && m_airBalance > 0.0f)
                poseHold += (std::max(poseHold, FallAirTone) - poseHold) * m_airBalance;
        }
        float stiffnessMultiplier = poseHold;
        float dampingMultiplier = std::sqrt(poseHold);
        float torqueMultiplier = poseHold;
        // Caindo, freio baixo tambem: a raiz quadrada do tonus deixava metade
        // do freio de pe e o corpo caia "posado", devagar nas juntas.
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id == "Neck" || id == "Head";
            const float falling = reflexLimb ? m_fallReflex : m_fallCommit;
            dampingMultiplier += ((reflexLimb ? 0.55f : 0.15f)
                - dampingMultiplier) * falling;
        }
        if (input.forceDrivenRoot && !getUpActive && leg) {
            // Changing a stride can ask for an incompatible extension on a
            // loaded leg. Let it yield while the new support is established.
            // (Caindo sem volta, a perna tem o tonus da queda, sem o
            // amolecer da passada: com os dois, ~14%, ela nem dobrava e ele
            // caia reto como uma tabua.)
            // TEMPTUNE LandingLegTone: no pouso de um pulo a troca de clipe
            // (pulo -> pouso -> corrida) recomecava a mistura e as pernas iam
            // a 35% da forca justo no impacto - o joelho ia a 130 graus, a
            // pelve afundava 30 cm e, agachado a 7,5 m/s, o tronco tombava para
            // a frente (correndo inclinado, segurado pela ajuda).
            const float transitionTone = std::max(transition,
                LandingLegTone > 0.0f && m_sinceTouchdownSeconds < LandingLegToneSeconds
                    && footEstimate.supporting ? LandingLegTone : 0.0f);
            const float transfer = FallProtectNew && FallLegsNew
                ? 0.35f + 0.65f * transitionTone + (1.0f / 0.55f - 0.35f - 0.65f * transitionTone) * m_fallCommit
                : 0.35f + 0.65f * transitionTone;
            if (agileAnkle) {
                // No ar: mola cheia e menos freio (ver agileAnkle).
                dampingMultiplier *= AnkleAirDamping;
            } else if (agileKnee) {
                // No ar: a mola da troca de apoio, sem o amolecer antes do toque.
                stiffnessMultiplier *= transfer * 0.55f;
                torqueMultiplier *= transfer * 0.75f;
            } else {
                // (Perna no chao: StanceLegStiffness - era 0,55 sempre, e o
                // joelho de apoio cedia 5-15 graus alem do pedido: "sempre
                // semi-agachado".)
                stiffnessMultiplier *= transfer * (footEstimate.supporting ? StanceLegStiffness : 0.55f);
                torqueMultiplier *= transfer * 0.75f;
                if (pushPhase) {
                    stiffnessMultiplier *= 1.0f + (PushStiffness - 1.0f) * pushWeight;
                    torqueMultiplier *= 1.0f + (std::max(1.0f, PushStiffness * 0.75f) - 1.0f) * pushWeight;
                }
                // Prepare compliance before impact, while the descending sole
                // approaches its measured surface. Retain damping through touch.
                stiffnessMultiplier *= 1.0f - 0.30f * touchdownShare[side];
                dampingMultiplier *= 1.0f + 0.30f * touchdownShare[side];
            }
            // Com pouca ajuda a perna no ar tem de chegar onde o pe vai pousar
            // (ninguem leva o corpo ate ela): amolecida como quando a pelve
            // levava o corpo, ela ficava 15-20 graus atras do alvo e o pe
            // tocava o chao antes, atras do corpo (TEMPTUNE: em teste).
            // (Tambem nos passos do planejador - freada, recuperacao: parado,
            // o passo de freada a 5% ficava 0,5 s no ar, e o corpo caia de
            // lado sobre o outro pe.)
            const bool plannerStepInAir = PlannerSwingLowAssist && footInAir && !legPlan.followingGait
                && (legPlan.phase == FootPhase3D::Swing || legPlan.phase == FootPhase3D::TouchdownSearch);
            if (lowAssist > 0.0f && ((footInAir && gaitMoving
                    && m_gaitDirection == CharacterGaitDirection3D::Forward) || plannerStepInAir)) {
                stiffnessMultiplier *= 1.0f + (SwingLowAssistStiffness - 1.0f) * lowAssist;
                torqueMultiplier *= 1.0f + (SwingLowAssistTorque - 1.0f) * lowAssist;
            }
            // O tornozelo apoiado continua sendo uma mola, nao uma junta
            // soldada: cede para a sola adaptar-se e devolve energia na
            // rolagem ate o antepe. O equilibrio ainda pode modular o alvo;
            // reduzimos a rigidez/torque, sem soltar o controle do pe.
            if (input.footSpring && ankle && !agileAnkle) {
                stiffnessMultiplier *= 0.82f;
                dampingMultiplier *= 0.92f;
                torqueMultiplier *= 0.86f;
            }
            // Mola do pe no toque: o tornozelo cede e amortece enquanto o peso
            // chega (o pe assenta no chao, o corpo nao bate na perna rigida)
            // e volta a firmar em ~0,15 s, a tempo do equilibrio e do impulso.
            if (input.footSpring && ankle && footEstimate.supporting
                && footEstimate.supportingSeconds < AnkleLandingSeconds) {
                const float settle = smoothStep(footEstimate.supportingSeconds / AnkleLandingSeconds);
                stiffnessMultiplier *= AnkleLandingStiffness + (1.0f - AnkleLandingStiffness) * settle;
                dampingMultiplier *= 1.0f + AnkleLandingDamping * (1.0f - settle);
            }
            // Pouso forte: a perna amortece (menos mola, mais freio) enquanto
            // a pelve desce, e volta a firmar ao subir.
            if (m_sinceTouchdownSeconds < m_landingSettleSeconds) {
                const float settle = std::clamp(m_sinceTouchdownSeconds
                    / m_landingSettleSeconds, 0.0f, 1.0f);
                const float absorb = smoothStep((m_landingImpactSpeed - 1.5f) / 2.5f)
                    * (1.0f - smoothStep((settle - 0.40f) / 0.60f));
                stiffnessMultiplier *= 1.0f - 0.35f * absorb;
                dampingMultiplier *= 1.0f + 0.80f * absorb;
            }
            // Aceitacao de peso (E5, fase 9): a perna que acabou de pousar
            // num passo de marcha e 25% mais mole durante a carga e os
            // primeiros 0,15 s de apoio - rigida, ela entrava como escora e
            // freava o corpo no toque. Grade medida (1 m/s, 16 corridas,
            // rigidez x amortecimento): 1,0 x 1,0 perdia 0,085 m/s na colisao
            // e 0,116 ate a saida do outro pe; 0,75 x 1,0, 0,046 e 0,069, mesmas
            // quedas, regime 76 -> 80%; mais mole (0,45-0,60) caia mais.
            if (m_footwork.settings().acceptanceCompliance
                && !legPlan.followingGait && legPlan.reason == StepReason3D::Locomotion
                && (legPlan.phase == FootPhase3D::Loading
                    || (legPlan.phase == FootPhase3D::Stance
                        && m_estimator.state().feet[side].supportingSeconds < 0.15f)))
                stiffnessMultiplier *= LocomotionAcceptanceStiffness;
        }
        if (getUpActive) {
            const std::string& id = profile.links[index].id;
            const bool isArm = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos;
            const bool isTorso = id == "Abdomen" || id == "Chest"
                || id == "UpperChest" || id == "Neck" || id == "Head";
            // Deitado com tonus: cabeca e tronco firmes (a cabeca nao bate
            // solta), bracos e pernas moderados - vezes o tonus do momento
            // (cede rolando, volta antes de levantar). O amortecimento fica:
            // cedendo, a junta ainda freia o movimento (nao e corpo morto).
            const float lyingStiffness = (isTorso ? 0.70f : isArm ? 0.50f : 0.55f)
                * m_settleTone;
            const float lyingTorque = (isTorso ? 0.90f : isArm ? 0.65f : 0.75f)
                * std::max(0.5f, m_settleTone);
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                // Largado, quase sem freio nem mola: o alvo persegue o corpo,
                // entao mola x atraso + amortecimento viram um freio viscoso.
                // Com ~5 N.m.s/rad um antebraco quase em pe (peso ~1,5 kg)
                // levava ~3 s para tombar; num ragdoll, ~0,3 s. Fica o teto
                // baixo de torque - o "pouquinho de forca". Se arrumando, tudo
                // volta ao nominal com o tonus.
                const float rest = 1.0f - m_settleGather;
                stiffnessMultiplier = lyingStiffness * (1.0f - 0.75f * rest);
                dampingMultiplier = 1.0f - 0.97f * rest;
                torqueMultiplier = lyingTorque * (1.0f - 0.60f * rest);
            } else {
                // Levantar: os bracos sao o apoio (a flexao de bracos de quem
                // levanta de bruços). Entra a partir do tonus deitado em
                // ~0,5 s, nao de uma vez.
                // (No carrinho ele vem correndo, firme: sem a rampa do deitado.)
                const float rising = m_tackling ? 1.0f : smoothStep(m_getUpPhaseSeconds / 0.5f);
                stiffnessMultiplier = lyingStiffness + ((isArm ? RisingArmStiffness
                    : isTorso ? RisingTorsoStiffness : RisingLegStiffness)
                    - lyingStiffness) * rising;
                dampingMultiplier = 1.0f + ((isArm ? 1.45f : isTorso ? 1.25f : 1.30f)
                    - 1.0f) * rising;
                torqueMultiplier = lyingTorque + ((isArm ? RisingArmTorque
                    : isTorso ? RisingTorsoTorque : RisingLegTorque)
                    - lyingTorque) * rising;
                // Rigidez ao levantar (slider do laboratorio): mais relaxado,
                // a mola cede mais; o freio acompanha pela raiz (mesma
                // proporcao de amortecimento) e a forca maxima fica.
                const float tone = m_tackling ? 1.0f : std::clamp(input.getUpStiffness, 0.2f, 1.5f);
                stiffnessMultiplier *= 1.0f + (tone - 1.0f) * rising;
                dampingMultiplier *= 1.0f + (std::sqrt(tone) - 1.0f) * rising;
                // E a forca (slider do laboratorio, pedido do usuario, 05/10
                // noite: no comeco do levantar ele fazia movimentos tao
                // fortes que pulava no chao).
                const float strength = m_tackling ? 1.0f : std::clamp(input.getUpStrength, 0.2f, 1.5f);
                torqueMultiplier *= 1.0f + (strength - 1.0f) * rising;
                // Carrinho: musculos de quem corre, nao os do levantar (com
                // eles a passada de entrada e a de saida viravam saltos - a
                // perna apoiada lancava o corpo por cima dela). Descendo para
                // o chao as pernas cedem (deixar-se cair no deslize); subindo
                // do chao elas ganham forca.
                if (m_tackling) {
                    const bool isLeg = !isArm && !isTorso;
                    const float drop = smoothStep((-m_tackleExpectedRise - 0.4f) / 0.8f);
                    const float push = smoothStep((m_tackleExpectedRise - 0.2f) / 0.6f)
                        * smoothStep((0.80f - m_tackleExpectedHeight) / 0.25f);
                    const float legShare = isLeg ? (1.0f - 0.75f * drop) * (1.0f + 0.5f * push) : 1.0f;
                    stiffnessMultiplier = (isArm ? 1.0f : isTorso ? 1.2f : 1.0f) * legShare;
                    dampingMultiplier = isTorso ? 1.2f : 1.1f;
                    torqueMultiplier = (isArm ? 1.0f : isTorso ? 1.3f : 1.1f) * legShare;
                }
            }
            if (bodilyRising && index < m_bodilyPose.stiffness.size()) {
                stiffnessMultiplier = m_bodilyPose.stiffness[index];
                dampingMultiplier = m_bodilyPose.damping[index];
                torqueMultiplier = m_bodilyPose.torque[index];
            }
            torqueMultiplier = std::min(torqueMultiplier, GetUpTorqueCap);
        }
        // Largado, os membros pesam. Com a gravidade compensada inteira eles
        // ficavam onde estavam, sem peso: o corpo caia e deitava "posado", de
        // costas com as maos a 25-37 cm do chao. Um resto sustenta (tonus);
        // se arrumando para levantar, a compensacao volta com o tonus.
        float gravityScale = 1.0f;
        {
            const std::string& id = profile.links[index].id;
            const bool reflexLimb = id.find("Arm") != std::string::npos
                || id.find("Forearm") != std::string::npos
                || id.find("Hand") != std::string::npos
                || id == "Neck" || id == "Head";
            if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
                gravityScale = LyingGravity + (1.0f - LyingGravity) * m_settleGather;
            } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising && m_tackling) {
                gravityScale = 1.0f;
            } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
                const float from = LyingGravity + (1.0f - LyingGravity) * m_settleGather;
                gravityScale = from + (1.0f - from)
                    * smoothStep(m_getUpPhaseSeconds / 0.5f);
            } else if (reflexLimb) {
                gravityScale = 1.0f - (1.0f - FallLimbGravity) * m_fallReflex;
            } else {
                gravityScale = 1.0f - (1.0f - FallBodyGravity) * m_fallCommit;
            }
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[index].inboundJoint.axes[axis].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(index);
            target.axis = static_cast<RagdollAxis3D>(axis);
            target.positionRadians = component(m_coordinates[index], axis);
            // Folga acima do joelho do sprint (~21 rad/s de pico).
            const float recoveryVelocityShare = m_tackling ? 1.0f
                : m_getUpPhase == CharacterGetUpPhase3D::Settling
                ? 0.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising
                    ? smoothStep(m_getUpPhaseSeconds / 0.5f) : 1.0f;
            // (O mergulho do carrinho e rapido: o limite do levantar o
            // atrasava.)
            const float velocityLimit = getUpActive && !m_tackling ? 6.0f : 30.0f;
            target.velocityRadiansPerSecond = std::clamp(
                component(velocity, axis) * velocityShare * recoveryVelocityShare,
                -velocityLimit, velocityLimit);
            // Modo fisico: a junta perto de uma pancada cede (controlador
            // fisico, por link).
            // Caido ou levantando, o tonus e o de cima (deitado/levantar).
            const float yielding = !getUpActive
                    && index < input.jointStrength.size()
                ? std::clamp(input.jointStrength[index], 0.05f, 1.0f) : 1.0f;
            // Cambaleio: o tonus de cada junta vai ao piso dela (pernas
            // seguram um pouco; tronco e bracos soltam, com menos gravidade
            // compensada) e volta com ele.
            float staggerTone = 1.0f, staggerGravity = 1.0f;
            if (m_stagger > 0.0f && !getUpActive) {
                if (m_staggerLimb.size() != profile.links.size()) {
                    m_staggerLimb.assign(profile.links.size(), 0);
                    for (std::size_t link = 0; link < profile.links.size(); ++link) {
                        const std::string& id = profile.links[link].id;
                        m_staggerLimb[link] = id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos
                                || id.find("Hand") != std::string::npos ? 2
                            : id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck" || id == "Head" ? 1 : 0;
                    }
                }
                const std::uint8_t limb = m_staggerLimb[index];
                // (v2: o piso das pernas sobe com menos ajuda - sem ela quem
                // segura o corpo amolecido sao as pernas, e a 20% de tonus
                // nao saia o passo que salva: empurrado parado a 0%, ele
                // sentava para tras com os dois pes plantados.)
                const float legFloor = input.physicalV2
                    ? StaggerLegTone + (0.6f - StaggerLegTone) * (1.0f - std::clamp(input.walkingAssist, 0.0f, 1.0f))
                    : StaggerLegTone;
                // (O tronco tambem: a 10% de tonus, sem ajuda o peito tombava
                // ~75 graus de lado num empurrao correndo.)
                // (Perto de 0% de ajuda, nao: la o tronco mole absorve o
                // empurrao parado - firme, o choque ia ao corpo inteiro e ele
                // caia mais.)
                static const float torsoLowAssist = tuneTemp("T_StaggerTorsoLow", 0.30f);  // TEMPTUNE
                const float walkAssist = std::clamp(input.walkingAssist, 0.0f, 1.0f);
                const float torsoShare = (1.0f - walkAssist) * smoothStep((walkAssist - 0.1f) / 0.15f);
                const float torsoFloor = input.physicalV2
                    ? StaggerTorsoTone + (torsoLowAssist - StaggerTorsoTone) * torsoShare
                    : StaggerTorsoTone;
                const float floor = limb == 2 ? StaggerArmTone : limb == 1 ? torsoFloor : legFloor;
                staggerTone = 1.0f - m_stagger * (1.0f - floor);
                if (limb != 0) staggerGravity = 1.0f - StaggerGravityDrop * m_stagger;
            }
            // v2: com pouca ajuda, a perna de apoio da passada para a frente
            // segura a pose com mais rigidez - ate StanceStiffness vezes perto
            // de 0% (1 - ajuda), sumindo de 30% a 50% (a 70%, 1,45x dobrava as
            // quedas do tropeco na barra escondida, 6 -> 12 em 128). E a mola da pose
            // que leva o corpo na passada (o esforco por J^T mudava pouco); com
            // a rigidez normal o corpo nao passava de ~4 m/s a 20% e o pe
            // escorregava para tras. Mais macia, mais lento; 4x ou mais
            // derrubava mais nas manobras rapidas (mais velocidade nelas). Fora
            // da passada para a frente (de lado, de costas, freando, tropecando
            // ou cambaleando) fica a normal: saindo do passo de lado para a
            // corrida, a perna rigida chutava o corpo.
            float stanceStiffnessScale = 1.0f;
            if (StanceStiffness != 1.0f && input.physicalV2 && input.forceDrivenRoot && !getUpActive
                && (m_telemetry.state == CharacterLocomotionState3D::Walking
                    || m_telemetry.state == CharacterLocomotionState3D::Running)
                && m_gaitDirection == CharacterGaitDirection3D::Forward
                && m_brakeGait <= 0.0f && !m_brakeStepping && m_tripRecovery < 0.05f
                && m_stagger < 0.05f) {
                for (std::size_t leg = 0; leg < m_feet.size(); ++leg) {
                    if (!m_feet[leg].valid || !m_feet[leg].planted) continue;
                    const int foot = static_cast<int>(m_feet[leg].linkIndex);
                    const int shin = profile.links[static_cast<std::size_t>(foot)].parentIndex;
                    const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
                    const int link = static_cast<int>(index);
                    const float assist = std::clamp(input.walkingAssist, 0.0f, 1.0f);
                    if (link == foot || link == shin || link == thigh)
                        stanceStiffnessScale = 1.0f + (StanceStiffness - 1.0f)
                            * (1.0f - assist) * smoothStep((0.5f - assist) / 0.2f);
                }
            }
            target.stiffnessScale =
                muscle * stiffnessMultiplier * impactSoftening * yielding * staggerTone * stanceStiffnessScale;
            target.dampingScale =
                std::sqrt(muscle) * dampingMultiplier * std::sqrt(staggerTone);
            target.maximumTorqueScale =
                muscle * torqueMultiplier * impactSoftening * yielding * staggerTone;
            target.gravityCompensationScale = gravityScale * staggerGravity;
            if (input.manipulated && index == input.grabbedLink) {
                target.stiffnessScale *= 0.20f;
                target.dampingScale *= 0.45f;
                target.maximumTorqueScale *= 0.20f;
            }
            m_output.driveTargets.push_back(target);
            const float error = target.positionRadians
                - component(measured, axis);
            squaredError += error * error;
            ++errorCount;
        }
    }

}

} // namespace MatterEngine
