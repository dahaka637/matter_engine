#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepLegs(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const float& StanceRootBand = t.StanceRootBand;
    const float& StanceDampCurveSpan = t.StanceDampCurveSpan;
    const std::vector<Vec3>& previousCoordinates = t.previousCoordinates;
    const CharacterPhysicalState3D& physical = *t.physical;
    const bool& getUpActive = t.getUpActive;
    CharacterLocomotionState3D& desiredState = t.desiredState;
    const Quaternion& heading = t.heading;
    const bool& gaitMoving = t.gaitMoving;
    const AnimationClip3D*& selectedClip = t.selectedClip;
    std::vector<Vec3>& sampled = t.sampled;
    const Quaternion& targetRootOrientation = t.targetRootOrientation;
    const float& lowAssist = t.lowAssist;
    Vec3& targetRootPosition = t.targetRootPosition;
    RagdollAnimationPose3D& localPose = t.localPose;
    const float& baseGround = t.baseGround;
    const bool& footworkEnabled = t.footworkEnabled;
    const bool& onGround = t.onGround;
    Vec3& planarVelocity = t.planarVelocity;
    std::array<FootGoal, 2>& goals = t.goals;
    const bool& movementRequested = t.movementRequested;
    Vec3& standingShift = t.standingShift;
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
    const auto forwardOf = [&](const FootPlant& foot, Quaternion orientation) {
        return orientation.rotate(profile.links[foot.linkIndex]
            .modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
    };
    // An uncontrolled root uses its physical XY as the next input anchor;
    // feeding the support shift back into it would integrate a displacement.
    if (!input.controlled) standingShift.x = standingShift.y = 0.0f;
    targetRootPosition += standingShift;
    // Toque parado na bola com passo de apoio: a pelve vai junto (absoluto,
    // com peso, suavizado). A ancora da capsula nao inclui isto: a capsula
    // segue o corpo ate la.
    {
        Vec3 wanted {};
        if (input.controlled && input.ballBody.active && input.ballBody.weight > 0.0f) {
            const float w = std::clamp(input.ballBody.weight, 0.0f, 1.0f);
            wanted = { (input.ballBody.position.x - targetRootPosition.x) * w,
                (input.ballBody.position.y - targetRootPosition.y) * w, 0.0f };
        }
        m_ballBodyDelta += (wanted - m_ballBodyDelta) * (1.0f - std::exp(-deltaTime / 0.10f));
        targetRootPosition += m_ballBodyDelta;
    }

    // Pelve pelo alcance das pernas: desce o que for preciso para o quadril
    // alcancar cada pe apoiado (ou pousando) com a perna a 94% de esticada -
    // nem mais, nem menos. Descer ate o pe mais baixo (a regra de antes)
    // agachava o corpo na escada sem precisar; e com a capsula subindo as
    // quinas de forma continua, 8-16 cm acima do degrau de apoio, o pe recem
    // pousado ficava pendurado. Com os dois pes apoiados acima da base da
    // capsula, a pelve sobe.
    t.drop = 0.0f;
    float& drop = t.drop;
    float raise = std::numeric_limits<float>::infinity();
    bool bothAbove = true;
    for (std::size_t side = 0; side < goals.size(); ++side) {
        const FootGoal& goal = goals[side];
        if (!goal.active || goal.reach <= 0.0f) {
            bothAbove = false;
            continue;
        }
        const RagdollLinkDefinition3D& footLink =
            profile.links[m_feet[side].linkIndex];
        const int shin = footLink.parentIndex;
        const int thigh = shin > 0
            ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) continue;
        const RagdollLinkDefinition3D& shinLink =
            profile.links[static_cast<std::size_t>(shin)];
        const RagdollLinkDefinition3D& thighLink =
            profile.links[static_cast<std::size_t>(thigh)];
        const Vec3 hipLocal = profile.links[0].modelOrientation.conjugate()
            .rotate(thighLink.inboundJoint.anchorModelPosition
                - profile.links[0].modelPosition);
        const float legLength = (shinLink.inboundJoint.anchorModelPosition
                - thighLink.inboundJoint.anchorModelPosition).length()
            + (footLink.inboundJoint.anchorModelPosition
                - shinLink.inboundJoint.anchorModelPosition).length();
        const Vec3 hip = targetRootPosition
            + targetRootOrientation.rotate(hipLocal);
        const Vec3 anchorInFoot = footLink.modelOrientation.conjugate()
            .rotate(footLink.inboundJoint.anchorModelPosition
                - footLink.modelPosition);
        const float comfortable = legLength * ReachComfort;
        const auto needAt = [&](float horizontal, float vertical) {
            return horizontal >= comfortable ? vertical
                : vertical - std::sqrt(comfortable * comfortable
                    - horizontal * horizontal);
        };
        const auto needFor = [&](Vec3 ankle) {
            return needAt(std::hypot(hip.x - ankle.x, hip.y - ankle.y),
                hip.z - ankle.z);
        };
        const float need = needFor(goal.position
            + goal.orientation.rotate(anchorInFoot));
        const float authored = needFor(goal.animatedPosition
            + goal.animatedOrientation.rotate(anchorInFoot));
        drop = std::max(drop, std::min(goal.dropCap, std::max(0.0f, need - std::max(0.0f, authored))
            * goal.reach));
        if (goal.anticipate > 0.0f) {
            const Vec3 hipThen = hip + goal.hipTravel;
            const Vec3 ankle = goal.landingPosition
                + goal.landingOrientation.rotate(anchorInFoot);
            const float needThen = needAt(std::hypot(hipThen.x - ankle.x,
                hipThen.y - ankle.y), hipThen.z - ankle.z);
            const float authoredThen = needAt(goal.authoredLandingHorizontal,
                goal.authoredLandingVertical);
            drop = std::max(drop, std::min(goal.dropCap, std::max(0.0f,
                needThen - std::max(0.0f, authoredThen)) * goal.anticipate * ReachAnticipateScale));
        }
        const float above = goal.pelvisGround - baseGround;
        if (goal.reach < 0.999f || above <= 0.0f) bothAbove = false;
        raise = std::min(raise, above);
    }
    float pelvisTarget = -drop;
    if (bothAbove && std::isfinite(raise)) pelvisTarget += raise;
    if (!onGround || !footworkEnabled) pelvisTarget = 0.0f;
    pelvisTarget = std::clamp(pelvisTarget, -0.35f, 0.25f);
    const float PelvisOmega = ReachPelvisOmega;
    m_pelvisOffsetVelocity += (PelvisOmega * PelvisOmega
            * (pelvisTarget - m_pelvisOffset)
        - 2.0f * PelvisOmega * m_pelvisOffsetVelocity) * deltaTime;
    m_pelvisOffset += m_pelvisOffsetVelocity * deltaTime;
    targetRootPosition.z += m_pelvisOffset;
    m_telemetry.pelvisDropReachMeters = -m_pelvisOffset;
    // Freando, as pernas de apoio cedem ou seguram, nunca estendem: a pelve
    // de referencia nao fica acima da medida. Com o pe de freada a frente a
    // pelve desce para alcanca-lo; quando o corpo passava por cima dele a
    // referencia voltava a subir, a perna de apoio estendia (joelho de 90
    // para 24 graus em 0,1 s) e lancava o corpo como uma vara de salto - 3 m/s
    // para cima e 0,8 s no ar a 4,6 m/s.
    // (So com o corpo ainda rapido: devagar, sem a ajuda, a pelve que cedia
    // um pouco nunca mais subia - a teto vinha junto - e em 0,8 s de passos
    // de lado o centro de massa descia de 0,98 para 0,77 m e ele desabava.)
    if (m_brakeAnchor > 0.0f && input.forceDrivenRoot && !getUpActive && !state.links.empty()) {
        const float ceiling = state.links.front().position.z + BrakeRiseMeters;
        const float fast = smoothStep((std::hypot(m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y) - BrakeCeilingSlowSpeed) / BrakeCeilingSpan);
        if (targetRootPosition.z > ceiling)
            targetRootPosition.z += (ceiling - targetRootPosition.z) * m_brakeAnchor * m_brakeGait * fast;
    }

    // Pernas por IK ate cada pe (posicao e orientacao), com a pelve final.
    // Mesmo com a raiz livre (forceDrivenRoot), o IK parte da pelve
    // DESEJADA: assim a perna de apoio empurra o corpo para onde ele deve
    // estar (o pe esta travado no chao) - e o musculo que sustenta. Pela
    // pelve medida, quando ela cedia o IK dobrava a perna junto e nada a
    // empurrava de volta (medido: parado sem ajuda, so em pe assim).
    // A perna que o planejador tem NO AR e outra coisa: ela precisa pousar o
    // pe no alvo do mundo, e as juntas dela partem da pelve de verdade. Pela
    // desejada (reta, na capsula), com o corpo inclinado 20-40 graus por um
    // empurrao o pe do passo de recuperacao ia girado junto e ficava no ar
    // "procurando o chao" ate o corpo cair. A troca e gradual (a perna nao
    // salta quando o passo comeca nem quando pousa).
    const PhysicsBodyState3D& measuredPelvis = state.links.front();
    {
        m_sinceGaitFlip = std::min(10.0f, m_sinceGaitFlip + deltaTime);
        const Vec3 requested { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
        const Vec3 body { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
        const bool changing = (requested - body).length() > GaitChangeVelocityError
            || m_brakeGait > 0.0f || m_sinceGaitFlip < 0.4f;
        m_gaitChange = changing ? 1.0f : std::max(0.0f, m_gaitChange - deltaTime / 0.3f);
    }
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPhase3D phase = m_footwork.plan().feet[side].phase;
        // O balanco da passada tambem (TEMPTUNE): pela pelve de referencia o
        // pe tocava em media 9 cm (trote) e 15 cm (sprint) a frente do alvo -
        // a pelve fisica inclinada alguns graus diferente da de referencia
        // leva o pe junto - e a trava, no alvo, ficava atras do pe real.
        const bool gaitSwing = GaitSwingMeasuredPelvis && input.forceDrivenRoot && footworkEnabled
            && !m_footwork.owns(side) && gaitMoving && m_feet[side].valid && !m_feet[side].planted
            && desiredState != CharacterLocomotionState3D::Airborne;
        const bool swinging = (input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side)
            && (phase == FootPhase3D::Swing || phase == FootPhase3D::TouchdownSearch)) || gaitSwing;
        float& weight = m_swingFromMeasuredPelvis[side];
        // (O da passada so com a ajuda andando alta: com pouca, a referencia
        // nao acompanha o corpo e o pe ia ao alvo dela, fora de baixo do
        // corpo - a 5%, 0 -> 9 quedas em 32 andando.)
        // (E so nas trocas de movimento: correndo estavel, pela altura medida a
        // perna pousava esticada com o corpo acima da referencia e o pe sem
        // carga escorregava - 2 -> 10-12 cm por apoio no sprint.)
        // (Decidido na saida do pe e mantido no balanco inteiro: ligando no
        // meio dele, a base do IK trocava com o pe no ar e o corte de 90
        // graus em sprint caia 3 em 8.)
        const bool gaitLegNow = !(input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side));
        if (gaitSwing && gaitLegNow && !m_gaitSwingLatched[side]) {
            m_gaitSwingLatched[side] = true;
            m_gaitSwingCeiling[side] = smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f) - 0.5f) / 0.4f)
                * (GaitSwingOnlyInChange ? m_gaitChange : 1.0f);
        }
        if (!gaitSwing) m_gaitSwingLatched[side] = false;
        const float ceiling = gaitSwing && gaitLegNow
            ? (GaitSwingLatch ? m_gaitSwingCeiling[side]
                : smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f) - 0.5f) / 0.4f)
                    * (GaitSwingOnlyInChange ? m_gaitChange : 1.0f)) : 1.0f;
        // (A perna da passada que acabou de pousar sai da pelve medida no
        // ritmo GaitSwingRelease: em 0,15 s o apoio inteiro do sprint, ~0,1
        // s, era a troca de base, e a perna re-mirada empurrava o pe para a
        // frente - 12 cm de deslize por apoio.)
        const bool gaitPlanted = !(input.forceDrivenRoot && footworkEnabled && m_footwork.owns(side))
            && m_feet[side].valid && m_feet[side].planted;
        weight = swinging ? std::min(ceiling, weight + deltaTime / 0.06f)
            : gaitPlanted ? (GaitSwingReleaseSeconds <= 0.0f ? 0.0f
                : GaitSwingReleaseSeconds >= 9.0f ? weight
                : std::max(0.0f, weight - deltaTime / GaitSwingReleaseSeconds))
            : std::max(0.0f, weight - deltaTime / 0.15f);
    }
    // E5 (passos pelo planejador, andando): a capsula so segue o corpo e
    // fica para tras dele - as pernas de apoio resolvidas pela pelve
    // desejada (la atras) puxavam o corpo de volta e freavam a marcha. Andando
    // nesse modo, a posicao da pelve e a medida (no plano). A troca de base
    // e gradual (~0,25 s): parado, a referencia fica a ~5 cm da pelve fisica,
    // e trocar num tick saltava os alvos do quadril (0,06-0,09 rad) na
    // arrancada - o corpo ia para o lado errado antes do primeiro passo.
    {
        // Com a referencia corporal, na parada ela desacelera ate o repouso
        // e so entao devolve a base a referencia parada: trocando logo, a
        // base ia para a pelve de referencia (presa a capsula, que vem atras
        // do corpo) e as pernas puxavam o corpo de volta (+7 m/s2 recuando a
        // 0,9 m/s; ele passava do ponto e caia).
        const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
        const bool stillMoving = input.bodyReference && m_bodyRefValid
            && (m_bodyRefVelocity.length() > 0.05f || std::hypot(com.x, com.y) > 0.25f);
        // Com a referencia corporal ela e sempre a base das pernas de apoio,
        // andando ou parado: devolver a base a referencia parada (presa a
        // capsula, que so segue o corpo com folga) puxava o corpo na parada
        // (recuando: +4 a +6 m/s2 para a frente, passava do ponto e caia).
        const bool measuredBase = input.forceDrivenRoot && m_footwork.settings().locomotionStepping
            && (movementRequested || input.bodyReference);
        (void)stillMoving;
        m_legRootFromMeasured = measuredBase ? std::min(1.0f, m_legRootFromMeasured + deltaTime / 0.25f)
                                             : std::max(0.0f, m_legRootFromMeasured - deltaTime / 0.25f);
    }
    constexpr float LegBaseLeadSecondsSquared = 1.0f / 19.0f;
    constexpr float BodyRefAcceleration = 4.0f;
    constexpr float BodyRefLeashMeters = 0.10f;
    constexpr float BodyRefWalkMaxSpeed = 1.2f;
    constexpr float LegBaseLeadLimitMeters = 0.10f;
    // Alcance das pernas de apoio (R4): a extensao pedida comeca a ser cortada
    // com a perna a 90% do comprimento esticado e e toda cortada a 96%.
    constexpr float ReachComfortRatio = 0.80f;
    constexpr float ReachCriticalRatio = 0.90f;
    constexpr float BodyRefMaximumDrop = 0.08f;
    constexpr float BodyRefVerticalSpeed = 0.15f;
    Vec3 legRootPosition = targetRootPosition;
    if (m_legRootFromMeasured > 0.0f && input.bodyReference) {
        ++m_telemetry.bodyReferenceTicks;
        // Referencia corporal com estado: a velocidade vai ate a pedida com
        // aceleracao limitada, a posicao a integra, e a coleira a prende a
        // pelve medida (corpo atrasado ou bloqueado: nao acumula).
        const Vec3 pelvis { state.links.front().position.x, state.links.front().position.y, 0.0f };
        if (!m_bodyRefValid) {
            const Vec3 com = m_estimator.state().centerOfMassVelocityWorld;
            m_bodyRefPosition = pelvis;
            m_bodyRefVelocity = { com.x, com.y, 0.0f };
            m_bodyRefValid = true;
        }
        // A referencia pede o factivel: andando, a passada satura no alcance
        // (~0,86 m de percurso do pe) e o teto medido da marcha e ~1,1-1,2
        // m/s - acima disso o corpo passava da marcha (1,7 m/s pedindo 1,5)
        // e nao havia passo que o parasse. Mais rapido e corrida (E5-F). O
        // pedido do jogador nao muda.
        const Vec3 wanted = movementRequested
            ? clampMagnitude(Vec3 { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f },
                BodyRefWalkMaxSpeed)
            : Vec3 {};
        m_bodyRefVelocity += clampMagnitude(wanted - m_bodyRefVelocity, BodyRefAcceleration * deltaTime);
        m_bodyRefPosition += m_bodyRefVelocity * deltaTime;
        const Vec3 error = m_bodyRefPosition - pelvis;
        if (error.length() > BodyRefLeashMeters)
            m_bodyRefPosition = pelvis + error * (BodyRefLeashMeters / error.length());
        // Transferencia de apoio (fase 4, do planejador): o centro de massa
        // vai a meta de apoio (o centro dos pes deslocado para o pe que
        // recebe - a mesma meta do esforco de contato). Na direcao da
        // transferencia a referencia de marcha e reancorada na pelve (senao,
        // parada, ela segurava o corpo: os dois se anulavam a 40% do caminho),
        // e a base ganha o que falta ate a meta menos a velocidade do centro de
        // massa (0,15 s) - zera na meta. (Como velocidade integrada, sem volta
        // de posicao, passava da meta, o outro pe saia do chao e ele caia.)
        Vec3 transferLead {};
        const auto& transfer = m_footwork.plan().supportTransfer;
        if (transfer.outgoingFoot >= 0) {
            const CharacterPhysicalState3D& physical = m_estimator.state();
            const Vec3 center = (physical.feet[0].soleCenterWorld + physical.feet[1].soleCenterWorld) * 0.5f;
            const Vec3 goal = center + (transfer.supportPointWorld - center) * transfer.progress;
            // A meta e do ponto de captura (DCM), nao do centro de massa:
            // andando o corpo ja vem com velocidade, e levar o COM a 85% do
            // caminho a cada passo balancava o corpo de lado (0,2 m/s) e
            // freava a marcha. O DCM inclui o amortecimento pela velocidade.
            const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
            const Vec3 dcm { physical.centerOfMassWorld.x + physical.centerOfMassVelocityWorld.x / omega,
                physical.centerOfMassWorld.y + physical.centerOfMassVelocityWorld.y / omega, 0.0f };
            Vec3 toGoal { goal.x - dcm.x, goal.y - dcm.y, 0.0f };
            // Andando, nunca contra o movimento pedido: para o pe da frente
            // sair (de lado, o do lado para onde vai) o apoio passa ao de tras
            // com o corpo acelerando para LONGE dele - no sentido da marcha -,
            // nao levando o corpo de volta ate ele. (Puxando para tras, a
            // transferencia brigava com a marcha e a descarga estourava o
            // prazo.)
            {
                const float wantedSpeed = std::hypot(wanted.x, wanted.y);
                if (wantedSpeed > 0.1f) {
                    const Vec3 forward = wanted * (1.0f / wantedSpeed);
                    const float backward = dot(toGoal, forward);
                    if (backward < 0.0f) toGoal -= forward * backward;
                }
            }
            if (toGoal.length() > 0.001f) {
                // Reancora a referencia de marcha na direcao da transferencia;
                // andando, so a parte atravessada ao movimento pedido (a
                // frente e da marcha).
                Vec3 along = toGoal * (1.0f / toGoal.length());
                const float wantedSpeed = std::hypot(wanted.x, wanted.y);
                if (wantedSpeed > 0.1f) {
                    const Vec3 forward = wanted * (1.0f / wantedSpeed);
                    along = along - forward * dot(along, forward);
                }
                if (along.length() > 0.01f) {
                    along = along * (1.0f / along.length());
                    m_bodyRefPosition -= along * dot(m_bodyRefPosition - pelvis, along);
                }
                transferLead = clampMagnitude(toGoal, BodyRefLeashMeters);
            }
        }
        legRootPosition.x += (m_bodyRefPosition.x + transferLead.x - legRootPosition.x) * m_legRootFromMeasured;
        legRootPosition.y += (m_bodyRefPosition.y + transferLead.y - legRootPosition.y) * m_legRootFromMeasured;
    } else if (m_legRootFromMeasured > 0.0f) {
        m_bodyRefValid = false;
        legRootPosition.x += (state.links.front().position.x - legRootPosition.x) * m_legRootFromMeasured;
        legRootPosition.y += (state.links.front().position.y - legRootPosition.y) * m_legRootFromMeasured;
        // A base prevista (guia 17): a pelve medida mais o que o controle do
        // corpo pediu de aceleracao ao centro de massa (antes do limite do
        // apoio: a queda passiva do pendulo ja acontece sozinha). Os motores das pernas
        // sao molas rigidas em volta do alvo; resolvida da pelve medida, a
        // perna so segurava o corpo onde ele estava - a troca de peso da
        // descarga e a velocidade pedida, so em torque de antecipacao, nao
        // moviam o corpo (medido: 170 N pedidos por 0,2 s, COM parado; a
        // mesma troca pelos alvos, 8 cm, levou o COM a 0,3 m/s). ~19 s^-2 e a
        // rigidez horizontal medida das pernas de apoio (0,7 Hz).
        Vec3 lead { input.legBaseAccelerationWorld.x, input.legBaseAccelerationWorld.y, 0.0f };
        lead = lead * LegBaseLeadSecondsSquared;
        if (lead.length() > LegBaseLeadLimitMeters) lead = lead * (LegBaseLeadLimitMeters / lead.length());
        legRootPosition += lead * m_legRootFromMeasured;
    } else {
        m_bodyRefValid = false;
    }
    // Alcance das pernas de apoio (R4 do pacote v2). A vantagem no plano da
    // referencia corporal, com a raiz do IK na mesma altura, alongava a
    // perna de tras (quadril a frente e acima do tornozelo): os motores a
    // esticavam ao longo do eixo, que e mais vertical que horizontal, e o
    // corpo subia 3,5 cm e saia do chao andando (voo de ate 67 ms). Aqui a
    // parte da extensao pedida a cada perna de apoio e cortada conforme ela
    // chega perto do alcance, e a raiz desce o que for preciso para o
    // comprimento ficar: a pelve gira em volta do tornozelo (o arco do
    // pendulo) em vez de a perna esticar. Com dois pes no chao vale a perna
    // que pede a raiz mais baixa. So o alvo das juntas muda; nada na pelve.
    const auto legGeometry = [&](std::size_t side, Vec3& hipLocal, Vec3& ankleLocal, float& nominal) {
        const FootPlant& plant = m_feet[side];
        const int shin = plant.valid ? profile.links[plant.linkIndex].parentIndex : -1;
        const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) return false;
        const RagdollLinkDefinition3D& footLink = profile.links[plant.linkIndex];
        const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
        const Vec3 kneeModel = profile.links[static_cast<std::size_t>(shin)].inboundJoint.anchorModelPosition;
        const Vec3 ankleModel = footLink.inboundJoint.anchorModelPosition;
        hipLocal = profile.links[0].modelOrientation.conjugate().rotate(hipModel - profile.links[0].modelPosition);
        ankleLocal = footLink.modelOrientation.conjugate().rotate(ankleModel - footLink.modelPosition);
        nominal = (kneeModel - hipModel).length() + (ankleModel - kneeModel).length();
        return nominal > 0.1f;
    };
    if (m_legRootFromMeasured > 0.0f && input.bodyReference && input.bodyReachProjection && m_bodyRefValid) {
        // A altura segue a da pelve de referencia a no maximo 0,15 m/s: da
        // base parada em alerta (0,80 m) para a de marcha (0,91) ela subia
        // em 0,3 s, a 0,5 m/s, e as pernas de apoio lancavam o corpo (voo de
        // 67 ms na arrancada).
        if (m_bodyRefHeight <= 0.0f) m_bodyRefHeight = legRootPosition.z;
        m_bodyRefHeight += std::clamp(legRootPosition.z - m_bodyRefHeight,
            -BodyRefVerticalSpeed * deltaTime, BodyRefVerticalSpeed * deltaTime);
        float heightLimit = m_bodyRefHeight;
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            LegReachTelemetry3D& reach = m_telemetry.legReach[side];
            reach.verticalCorrection = 0.0f;
            reach.projectionGain = 0.0f;
            Vec3 hipLocal, ankleLocal;
            float nominal = 0.0f;
            if (!goals[side].active || !m_feet[side].planted || !m_footwork.owns(side)
                || !legGeometry(side, hipLocal, ankleLocal, nominal)) continue;
            const Vec3 ankle = goals[side].position + goals[side].orientation.rotate(ankleLocal);
            const Vec3 hipOffset = targetRootOrientation.rotate(hipLocal);
            const Vec3 hip = Vec3 { legRootPosition.x, legRootPosition.y, heightLimit } + hipOffset;
            const float length = (hip - ankle).length();
            const float previous = m_previousLegLength[side][1];
            if (previous < 0.0f || length <= previous) continue;
            const float gain = smoothStep((length / nominal - ReachComfortRatio)
                / (ReachCriticalRatio - ReachComfortRatio));
            float allowed = previous + (length - previous) * (1.0f - gain);
            allowed = std::min(allowed, std::max(previous, ReachCriticalRatio * nominal));
            if (allowed >= length) continue;
            const float planar = std::hypot(hip.x - ankle.x, hip.y - ankle.y);
            if (planar >= allowed) continue;
            const float height = ankle.z - hipOffset.z + std::sqrt(allowed * allowed - planar * planar);
            reach.projectionGain = gain;
            if (height < heightLimit) {
                reach.verticalCorrection = heightLimit - height;
                heightLimit = height;
            }
        }
        // A restricao de alcance desce a altura na hora, nunca mais de 8 cm
        // abaixo da referencia (alem disso a perna esta no fim e o passo tem
        // de sair).
        m_bodyRefHeight = std::max(heightLimit, legRootPosition.z - BodyRefMaximumDrop);
        legRootPosition.z += (m_bodyRefHeight - legRootPosition.z) * m_legRootFromMeasured;
    } else {
        m_bodyRefHeight = 0.0f;
    }
    // Tornozelo contra a queda (pe com mola, modo fisico): o corpo saindo da
    // velocidade da referencia (empurrado, tropecando, passando do ponto)
    // leva o ponto de captura para fora. O pe de apoio pressiona do lado
    // para onde ele vai - a ponta caindo para a frente, o calcanhar para
    // tras, a borda de lado -: o alvo do pe gira em volta do tornozelo, o
    // chao nao deixa o pe girar e o motor empurra o corpo de volta (o centro
    // de pressao anda dentro do pe, a estrategia do tornozelo). A posicao ja
    // tem a mola das pernas de apoio (resolvidas da pelve de referencia);
    // isto antecipa pela velocidade. Fora da janela da mola do toque.
    {
        const CharacterPhysicalState3D& physical = m_estimator.state();
        Vec3 wanted {};
        if (input.footSpring && input.ankleBalance && input.forceDrivenRoot && !getUpActive && physical.valid) {
            const float omega = std::clamp(physical.captureOmega, 1.0f, 6.0f);
            wanted = { (physical.centerOfMassVelocityWorld.x - planarVelocity.x) / omega,
                (physical.centerOfMassVelocityWorld.y - planarVelocity.y) / omega, 0.0f };
        }
        m_ankleBalanceShift += (wanted - m_ankleBalanceShift) * (1.0f - std::exp(-deltaTime / 0.05f));
        // Zona morta: o balanco de quem esta parado (alguns cm/s) nao mexe o
        // tornozelo - reagindo a ele, o corpo parado oscilava (contato
        // perdido 0 -> 4 em 15 nascimentos).
        Vec3 pressShift = m_ankleBalanceShift;
        {
            const float size = pressShift.length();
            pressShift = size > AnkleBalanceDeadband
                ? pressShift * ((size - AnkleBalanceDeadband) / size) : Vec3 {};
        }
        m_telemetry.ankleBalancePitch = 0.0f;
        m_telemetry.ankleBalanceRoll = 0.0f;
        for (std::size_t side = 0; side < m_feet.size() && wanted.lengthSquared() > 0.0f; ++side) {
            FootGoal& goal = goals[side];
            const FootSupportEstimate3D& estimate = physical.feet[side];
            Vec3 hipLocal, ankleLocal;
            float nominal = 0.0f;
            if (!goal.active || !m_feet[side].planted || !estimate.supporting
                || !legGeometry(side, hipLocal, ankleLocal, nominal)) continue;
            const float engage = smoothStep((estimate.supportingSeconds - AnkleLandingSeconds) / 0.10f);
            Vec3 forward = goal.orientation.rotate({ 1.0f, 0.0f, 0.0f });
            forward.z = 0.0f;
            if (engage <= 0.0f || forward.lengthSquared() < 1e-4f) continue;
            forward = forward.normalized();
            const Vec3 left = cross(Vec3 { 0.0f, 0.0f, 1.0f }, forward);
            const float pitch = engage * std::clamp(AnkleBalanceGain * dot(pressShift, forward),
                -AnkleBalancePitchLimit, AnkleBalancePitchLimit);
            const float roll = engage * std::clamp(AnkleBalanceGain * dot(pressShift, left),
                -AnkleBalanceRollLimit, AnkleBalanceRollLimit);
            const Quaternion press = (Quaternion::fromAxisAngle(left, pitch)
                * Quaternion::fromAxisAngle(forward, -roll)).normalized();
            const Vec3 ankle = goal.position + goal.orientation.rotate(ankleLocal);
            goal.orientation = (press * goal.orientation).normalized();
            goal.position = ankle + press.rotate(goal.position - ankle);
            if (std::abs(pitch) > std::abs(m_telemetry.ankleBalancePitch)) m_telemetry.ankleBalancePitch = pitch;
            if (std::abs(roll) > std::abs(m_telemetry.ankleBalanceRoll)) m_telemetry.ankleBalanceRoll = roll;
        }
    }
    std::array<Vec3, 2> ikRootPosition { legRootPosition, legRootPosition };
    std::array<Quaternion, 2> ikRootOrientation { targetRootOrientation, targetRootOrientation };
    // Preparo do pouso: descendo, a perna aponta para onde o corpo vai bater
    // - na linha da velocidade (uma fracao do angulo dela com a vertical),
    // com o joelho um pouco dobrado para a mola. Com o pe embaixo do quadril
    // e o corpo ainda indo para a frente, ele girava sobre o pe e era jogado
    // para a frente (o peito ate 52 graus no pulo em sprint); alinhada, a
    // forca do chao passa pelo centro de massa. No referencial da pelve
    // fisica: no ar e a orientacao dela que vale.
    m_landingPrep = 0.0f;
    if (LandingPrepEnabled && input.forceDrivenRoot && desiredState == CharacterLocomotionState3D::Airborne
        && !state.links.empty()) {
        const Vec3 velocity = m_filteredCenterOfMassVelocity;
        const float horizontalSpeed = std::hypot(velocity.x, velocity.y);
        // So perto do toque (~0,25 s antes): no ar, levar as pernas a frente
        // gira o tronco para tras (o momento angular se conserva) - feito
        // cedo, numa queda de 1 m ele girava ate cair de costas.
        float soleHeight = std::numeric_limits<float>::infinity();
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const PhysicsBodyState3D& footBody = state.links[m_feet[side].linkIndex];
            const RagdollLinkDefinition3D& footLink = profile.links[m_feet[side].linkIndex];
            const GroundSample ground = groundUnder(side, footBody.position,
                footForwardWorld3D(footLink, footBody.orientation));
            if (ground.valid) soleHeight = std::min(soleHeight, footBody.position.z - ground.height);
        }
        const float fall = std::max(0.0f, -velocity.z);
        const float touchdown = std::isfinite(soleHeight)
            ? (-fall + std::sqrt(fall * fall + 2.0f * 9.81f * std::max(0.0f, soleHeight))) / 9.81f : 0.0f;
        const float prep = smoothStep((-velocity.z - 0.3f) / 1.2f)
            * smoothStep((LandingLeadSeconds - touchdown) / 0.15f);
        if (prep > 0.0f && horizontalSpeed > 0.3f) {
            m_landingPrep = prep;
            const Vec3 direction { velocity.x / horizontalSpeed, velocity.y / horizontalSpeed, 0.0f };
            const float alpha = std::atan2(horizontalSpeed, std::max(0.05f, -velocity.z));
            // TEMPTUNE LandRunLead: pousando para seguir correndo, so a perna
            // mais baixa (a da frente da tesoura do pulo) vai ao chao, menos
            // inclinada; a de tras fica atras, pronta para o passo seguinte.
            // As duas a frente, na linha da velocidade, ele pousava sentado
            // (coxas a -100 graus), freava e o tronco tombava para a frente.
            const float runningOn = LandRunLead
                ? smoothStep((horizontalSpeed - 2.5f) / 1.5f)
                    * smoothStep(std::hypot(input.intent.requestedVelocityWorld.x,
                        input.intent.requestedVelocityWorld.y) / 1.5f) : 0.0f;
            const float beta = std::min(LandingAlign * alpha,
                LandingMaxAngle + (LandRunMaxAngle - LandingMaxAngle) * runningOn);
            int leadSide = -1;
            if (runningOn > 0.5f) {
                float lowest = std::numeric_limits<float>::infinity();
                for (std::size_t side = 0; side < m_feet.size(); ++side) {
                    const float z = state.links[m_feet[side].linkIndex].position.z;
                    if (z < lowest) { lowest = z; leadSide = static_cast<int>(side); }
                }
            }
            const Vec3 legWorld = direction * std::sin(beta) + Vec3 { 0.0f, 0.0f, -std::cos(beta) };
            const Quaternion pelvisInverse = state.links.front().orientation.conjugate();
            const RagdollLinkDefinition3D& pelvisLink = profile.links[0];
            // A coluna na mesma linha: o tronco inclina para tras uma parte do
            // angulo da perna (o centro de massa fica sobre a linha do pe ao
            // longo da velocidade - a mola trabalha nela, sem girar o corpo).
            // Pela inclinacao real da pelve: a coluna e relativa a ela.
            if (LandingTrunkAlign > 0.0f) {
                const Vec3 pelvisUp = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
                const float pelvisAngle = std::atan2(dot(pelvisUp, direction), pelvisUp.z);
                // (De lado menos: o quadril abre menos do que flexiona e a perna
                // alinha menos - inclinado o mesmo, de lado ele caia de costas.)
                const Vec3 pelvisForward = state.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                const float forwardness = std::abs(dot(Vec3 { pelvisForward.x, pelvisForward.y, 0.0f }.normalized(), direction));
                const float wanted = -LandingTrunkAlign * beta
                    * (LandingTrunkSide + (1.0f - LandingTrunkSide) * forwardness);
                const float correction = std::clamp(wanted - pelvisAngle, -0.7f, 0.4f);
                // Rotacao em torno do eixo a esquerda do rumo, levada para a
                // pelve: arfagem (eixo 1 da coluna) e rolagem (eixo 2).
                const Vec3 axisLocal = pelvisInverse.rotate({ -direction.y, direction.x, 0.0f });
                const float pitch = correction * axisLocal.y;
                const float roll = LandingRollSign * correction * axisLocal.x;
                const float shares[3] = { 0.30f, 0.35f, 0.35f };
                const char* spine[3] = { "Abdomen", "Chest", "UpperChest" };
                for (std::size_t i = 0; i < 3; ++i) {
                    for (std::size_t index = 1; index < profile.links.size(); ++index) {
                        if (profile.links[index].id != spine[i]) continue;
                        blendJointCoordinate(profile, m_coordinates, index, 1, pitch * shares[i], prep);
                        blendJointCoordinate(profile, m_coordinates, index, 2, roll * shares[i], prep);
                    }
                }
            }
            for (std::size_t side = 0; side < m_feet.size(); ++side) {
                if (!goals[side].active) continue;
                if (leadSide >= 0 && static_cast<int>(side) != leadSide) continue;
                const RagdollLinkDefinition3D& footLink = profile.links[m_feet[side].linkIndex];
                const int shin = footLink.parentIndex;
                const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
                if (thigh <= 0) continue;
                const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
                const Vec3 hipLocal = pelvisLink.modelOrientation.conjugate().rotate(hipModel - pelvisLink.modelPosition);
                const float legLength = LandingLegShare * (hipModel - footLink.inboundJoint.anchorModelPosition).length();
                const Vec3 wantedLocal = hipLocal + pelvisInverse.rotate(legWorld * legLength);
                const Vec3 currentLocal = ikRootOrientation[side].conjugate().rotate(goals[side].position - ikRootPosition[side]);
                goals[side].position = ikRootPosition[side] + ikRootOrientation[side].rotate(
                    currentLocal + (wantedLocal - currentLocal) * prep);
            }
        }
    }
    // TEMPTUNE: altura do voo regulada passo a passo (Raibert). Correndo, o
    // corpo e uma massa numa mola: sem regulagem, o empurrao de cada apoio
    // dependia de quando o pe tocou no ciclo, e uns passos lancavam o dobro
    // de outros (sprint: voos +-5 cm, pior +9; em curva +-11). Cada voo e
    // comparado com o da referencia (a pelve guia no mesmo voo): subiu
    // demais, no apoio seguinte a perna de apoio estende menos (o alvo da
    // pelve dela desce); ficou baixo, estende mais. So pela perna - nenhuma
    // forca de fora.
    {
        const auto& feetNow = m_estimator.state().feet;
        const bool anySupport = feetNow.size() >= 2 && (feetNow[0].supporting || feetNow[1].supporting);
        if (ThrustGain > 0.0f && gaitMoving && input.forceDrivenRoot && !getUpActive) {
            if (!anySupport) {
                m_flightBodyApex = m_inFlight ? std::max(m_flightBodyApex, measuredPelvis.position.z) : measuredPelvis.position.z;
                m_flightGuideApex = m_inFlight ? std::max(m_flightGuideApex, targetRootPosition.z) : targetRootPosition.z;
                m_inFlight = true;
            } else if (m_inFlight) {
                m_inFlight = false;
                m_flightApexError = m_flightBodyApex - m_flightGuideApex;
            }
            const float wanted = anySupport
                ? std::clamp(-ThrustGain * (m_flightApexError - ThrustApexMargin), -ThrustMaximum, ThrustMaximum) : 0.0f;
            m_thrustOffset += (wanted - m_thrustOffset) * (1.0f - std::exp(-deltaTime / 0.03f));
        } else if (ThrustGain <= 0.0f) {
            m_thrustOffset = 0.0f;
        }
        // TEMPTUNE: amortecimento vertical pela perna de apoio. Subindo mais
        // rapido que a referencia, a perna estende menos (o alvo da pelve dela
        // desce); descendo mais rapido (o pouso), segura mais. Sem ele, na
        // curva de sprint a perna seguia estendendo com o corpo ja subindo e
        // ele decolava 15-20 cm acima da referencia, caia o voo inteiro e
        // pousava 15 cm abaixo: o corpo oscilava 40 cm (pelve de 0,69 a 1,08 m).
        {
            const float guideRate = m_stanceDampingValid
                ? (targetRootPosition.z - m_stanceDampingGuideZ) / std::max(1e-4f, deltaTime) : 0.0f;
            m_stanceDampingGuideZ = targetRootPosition.z;
            m_stanceDampingValid = true;
            float wanted = 0.0f;
            // (So na corrida livre: no tropeco o corpo cai e a perna, segurando
            // mais, o empurrava contra a barra - sprint na barra de 26 cm, 13 ->
            // 38 quedas em 96; freando, idem.)
            const bool stumblingNow = m_tripRecovery > 0.001f || m_stumbleRisk > 0.2f
                || m_stumbleStepping || m_footwork.plan().recovering;
            const bool brakingNow = m_brakeGait > 0.0f || m_brakeStepping;
            if (StanceVerticalDamping > 0.0f && anySupport && gaitMoving && input.forceDrivenRoot && !getUpActive
                && !(StanceDampFree && (stumblingNow || brakingNow)))
                wanted = std::clamp(-StanceVerticalDamping * (measuredPelvis.linearVelocity.z - guideRate),
                    -ThrustMaximum, ThrustMaximum)
                    // (So na curva, pela inclinacao: na reta a corrida ja nao
                    // saltava e, achatada, o pe passava mais baixo e enganchava
                    // mais na barra escondida - 13 -> 30 quedas em 192.)
                    * (StanceDampCurveSpan > 0.0f
                        ? smoothStep(std::abs(m_bodyCurveLean) / StanceDampCurveSpan) : 1.0f);
            m_stanceDampingOffset += (wanted - m_stanceDampingOffset) * (1.0f - std::exp(-deltaTime / 0.02f));
            m_thrustOffset = (ThrustGain > 0.0f ? m_thrustOffset : 0.0f) + m_stanceDampingOffset;
        }
        m_telemetry.thrustOffsetMeters = m_thrustOffset;
    }
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        if (m_thrustOffset != 0.0f && side < m_estimator.state().feet.size()
            && m_estimator.state().feet[side].supporting && !m_footwork.owns(side))
            ikRootPosition[side].z += m_thrustOffset;
        // TEMPTUNE: correndo, a perna de apoio pede no maximo StanceRootBand
        // acima (ou abaixo) da altura de verdade da pelve. Servo rigido da
        // referencia, depois de um pouso 20 cm abaixo dela a perna estendia os
        // 20 cm de uma vez (3 g) e passava do ponto: o voo seguinte ia 30 cm
        // acima, o pouso seguinte mais fundo - a curva de sprint oscilava
        // 50 cm. Assim ela devolve o corpo a altura em alguns passos.
        // Experimento v2 (T_StanceMeasured): com pouca ajuda a perna de apoio
        // da passada e resolvida pela pelve de VERDADE no plano (a altura e a
        // orientacao seguem as da referencia). Pela de referencia o motor da
        // perna e uma mola que prende o corpo nela: o impulso (o esforco das
        // pernas, por J^T) era anulado assim que o corpo saia dela, e sem
        // ajuda o corpo nao passava de ~2 m/s.
        {
            static const float stanceMeasured = tuneTemp("T_StanceMeasured", 0.0f);  // TEMPTUNE
            static const float stanceMeasuredFrom = tuneTemp("T_StanceMeasuredFrom", 0.2f);  // TEMPTUNE
            static const float stanceMeasuredSpan = tuneTemp("T_StanceMeasuredSpan", 0.4f);  // TEMPTUNE
            if (stanceMeasured > 0.0f && gaitMoving && input.forceDrivenRoot && !getUpActive
                && side < m_estimator.state().feet.size() && m_estimator.state().feet[side].supporting
                && !m_footwork.owns(side) && m_feet[side].planted) {
                const float w = stanceMeasured * (1.0f - smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f)
                    - stanceMeasuredFrom) / std::max(0.01f, stanceMeasuredSpan)));
                ikRootPosition[side].x += (measuredPelvis.position.x - ikRootPosition[side].x) * w;
                ikRootPosition[side].y += (measuredPelvis.position.y - ikRootPosition[side].y) * w;
            }
        }
        if (StanceRootBand > 0.0f && gaitMoving && input.forceDrivenRoot && !getUpActive
            && side < m_estimator.state().feet.size() && m_estimator.state().feet[side].supporting
            && !m_footwork.owns(side)) {
            ikRootPosition[side].z = std::clamp(ikRootPosition[side].z,
                measuredPelvis.position.z - StanceRootBand, measuredPelvis.position.z + StanceRootBand);
        }
        const float measured = m_swingFromMeasuredPelvis[side];
        if (measured > 0.0f) {
            // Da passada (modos 3 e 4): so no plano - a altura fica a da
            // referencia. Pela altura medida, com o corpo acima dela a perna
            // alcancava o chao esticada, pousava como vara e o devolvia para
            // cima a cada passada (sprint: pelve 8,5 cm acima da referencia
            // no toque, o pe quase sem carga escorregava 10 cm).
            const bool gaitLeg = !m_footwork.owns(side);
            const int mode = gaitLeg ? GaitSwingMeasuredMode : 0;
            if (mode != 2) {
                ikRootPosition[side] = legRootPosition
                    + (measuredPelvis.position - legRootPosition) * measured;
                if (mode == 3 || mode == 4) ikRootPosition[side].z = legRootPosition.z;
                // (Modo 5: medida, mas nunca acima da referencia - com o corpo
                // alto a perna nao estica ate o chao.)
                if (mode == 5) ikRootPosition[side].z = std::min(ikRootPosition[side].z, legRootPosition.z);
            }
            if (mode != 1 && mode != 4)
                ikRootOrientation[side] = slerpQuaternion(targetRootOrientation,
                    measuredPelvis.orientation, measured);
        }
        // Corpo mais baixo que a referencia (curva inclinada, freada, pouso):
        // a perna da passada no ar parte da altura de verdade da pelve. Pela
        // de referencia, o pe no fim do balanco ficava 10-15 cm abaixo do alvo
        // e batia a ponta no chao a 5-9 m/s ("bicuda") - na curva fechada, o
        // pe de dentro tocava o chao no meio do balanco ~2 vezes por curva.
        // Acima da referencia nada muda (a perna nao estica ate o chao).
        {
            const bool gaitSwingLeg = SwingFromLowBody > 0.0f && input.forceDrivenRoot && gaitMoving
                && !m_footwork.owns(side) && m_feet[side].valid && !m_feet[side].planted
                && desiredState != CharacterLocomotionState3D::Airborne;
            // (So com a ajuda andando alta: com pouca, a passada inteira mudava
            // - a 5% andava a 2,3 m/s em vez de 1,6 e caia parando 11 em 32.)
            const float lowCeiling = SwingFromLowBody
                * smoothStep((std::clamp(input.walkingAssist, 0.0f, 1.0f) - 0.5f) / 0.4f);
            float& low = m_swingLowWeight[side];
            low = gaitSwingLeg ? std::min(lowCeiling, low + deltaTime / 0.06f)
                : std::max(0.0f, low - deltaTime / std::max(0.01f, SwingFromLowBodyRelease));
            // TEMPTUNE: so na subida e no meio do balanco (a folga do pe); no
            // fim dele a perna volta a altura da referencia e pousa estendida.
            // Pela altura de verdade ate o toque, com o corpo baixo na curva a
            // perna pousava dobrada (quadril a 45, joelho a 50) e, apoiada, ainda
            // partia da pelve baixa por SwingFromLowBodyRelease - nada levava o
            // corpo de volta para cima: a curva inteira agachada.
            // TEMPTUNE: passada pelo contato - a perna que espera o toque busca
            // o chao a partir da altura de verdade do corpo (estende para
            // alcancar). Pela referencia, ~5 cm abaixo do corpo no voo do
            // sprint, ela esperava dobrada (joelho a 45) e, no toque, o apoio
            // puxava o corpo para baixo (joelho pedido 58). Depois do toque a
            // referencia volta aos poucos e a perna amortece.
            {
                float& reach = m_landingReach[side];
                reach = m_waitingLeg == static_cast<int>(side)
                    ? std::min(1.0f, reach + deltaTime / 0.05f)
                    : std::max(0.0f, reach - deltaTime / std::max(0.01f, ContactLandRelease));
                if (reach > 0.0f && ContactLandReach) {
                    // (Ate ContactLandReachMax acima da referencia: alcancando de
                    // qualquer altura, pousava esticada como vara e o devolvia
                    // mais alto a cada passo - sprint com a pelve a 1,01 m,
                    // acima da de pe.)
                    const float above = std::clamp(measuredPelvis.position.z - ikRootPosition[side].z,
                        0.0f, ContactLandReachMax);
                    ikRootPosition[side].z += above * reach;
                }
            }
            float& landing = m_swingLowLanding[side];
            {
                const float progress = m_telemetry.footSwingProgress[side];
                const float wanted = SwingLowLandingStart >= 1.0f ? 1.0f
                    : m_feet[side].planted || progress < 0.0f ? 0.0f
                    : 1.0f - smoothStep((progress - SwingLowLandingStart) / SwingLowLandingSpan);
                landing += (wanted - landing) * (1.0f - std::exp(-deltaTime / 0.04f));
            }
            if (low > 0.0f && measured <= 0.0f) {
                const float below = std::min(0.0f, measuredPelvis.position.z - ikRootPosition[side].z);
                ikRootPosition[side].z += below * low * landing;
                if (SwingFromBodyTilt > 0.0f) {
                    // O quanto a pelve de verdade esta inclinada alem da de
                    // referencia (sem o giro em torno da vertical).
                    const Vec3 upLocal = profile.links.front().modelOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
                    const Vec3 upReference = ikRootOrientation[side].rotate(upLocal);
                    const Vec3 upMeasured = measuredPelvis.orientation.rotate(upLocal);
                    const Quaternion tilt = rotationBetween(upReference, upMeasured);
                    const Quaternion partial = slerpQuaternion(Quaternion {}, tilt,
                        std::clamp(SwingFromBodyTilt * low / std::max(0.01f, SwingFromLowBody), 0.0f, 1.0f));
                    ikRootOrientation[side] = (partial * ikRootOrientation[side]).normalized();
                }
            }
        }
        // Quase-queda: a perna da passada no ar mira pela pelve de verdade
        // (so a orientacao). Com o tronco caindo 30-45 graus para a frente,
        // pela pelve de referencia (reta) o mesmo angulo de quadril deixava o
        // pe 0,7 m atras do corpo e ele nao vinha a frente.
        if (StumbleDetection && StumbleSwingMeasured > 0.0f && m_tripRecovery > 0.0f && input.forceDrivenRoot
            && gaitMoving && !m_footwork.owns(side) && m_feet[side].valid && !m_feet[side].planted) {
            const float w = std::clamp(StumbleSwingMeasured * m_tripRecovery, 0.0f, 1.0f);
            if (w > measured)
                ikRootOrientation[side] = slerpQuaternion(targetRootOrientation, measuredPelvis.orientation, w);
        }
        if (!goals[side].active) continue;
        // Balanco da passada em malha fechada no pe (TEMPTUNE): a perna no ar
        // e macia e, no fim do balanco, a canela seguia por inercia quando o
        // alvo freava e recuava - o joelho passava ~7 graus da extensao e o pe
        // tocava ate 6-15 cm a frente do alvo. O alvo do IK vai alem do alvo
        // de verdade o quanto o pe passou dele (o que um musculo faria
        // puxando a canela de volta), com limite.
        if (SwingTrackGain > 0.0f && input.forceDrivenRoot && gaitMoving && !m_feet[side].planted
            && !m_footwork.owns(side) && m_telemetry.footSwingProgress[side] >= 0.0f
            && desiredState != CharacterLocomotionState3D::Airborne) {
            const float late = smoothStep((m_telemetry.footSwingProgress[side] - 0.35f) / 0.30f);
            if (late > 0.0f) {
                m_trackedSwingGoal[side] = goals[side].position;
                m_trackedSwingGoalValid[side] = true;
                const Vec3 physicalFoot = state.links[m_feet[side].linkIndex].position;
                const Vec3 error = goals[side].position - physicalFoot;
                goals[side].position += clampMagnitude(error * (SwingTrackGain * late), SwingTrackMaximum);
            }
        }
        // Perna de apoio da passada complacente: ela nunca e mandada a
        // erguer a pelve mais que isto acima de onde a pelve esta. Pousando
        // com o corpo 6-7 cm abaixo da referencia (fim do voo da corrida, ou
        // o agachamento da freada), a perna resolvida pela pelve de
        // referencia pedia o pe 6 cm dentro do chao - os motores a esticavam
        // de uma vez e o corpo saltava (ate 11 cm acima da referencia,
        // invertendo a 3 m/s). Acima da referencia nada muda: a perna cede.
        if (StanceRiseLimit < 1.0f && input.forceDrivenRoot && gaitMoving && m_feet[side].planted
            && !m_footwork.owns(side) && measured <= 0.0f && physical.feet[side].supporting)
            ikRootPosition[side].z = std::min(ikRootPosition[side].z, measuredPelvis.position.z + StanceRiseLimit);
        // Manejo de bola: o pe no ar vai a bola (o alvo do toque por cima do
        // da passada, com o peso do toque).
        if (!m_feet[side].planted && input.ballAvoid[side].lengthSquared() > 1e-8f)
            goals[side].position += input.ballAvoid[side];
        if (input.ballFoot.active && input.ballFoot.side == static_cast<int>(side)
            && input.ballFoot.weight > 0.0f && !m_feet[side].planted
            && !(input.ballScript.active && input.ballScript.side == static_cast<int>(side))) {
            const float w = std::clamp(input.ballFoot.weight, 0.0f, 1.0f);
            goals[side].position += (input.ballFoot.position - goals[side].position) * w;
            goals[side].orientation = slerpQuaternion(goals[side].orientation, input.ballFoot.orientation, w);
        }
        // O pe no ar nunca entra na bola nem pousa nela (ela nao colide com o
        // pe): o alvo sai dela no plano. O pe que vai tocar fica de fora.
        if (input.ballRepel.active && !m_feet[side].planted && input.ballRepel.exemptSide != static_cast<int>(side))
            goals[side].position += footBallPushOut(profile.links[m_feet[side].linkIndex], goals[side].position,
                goals[side].orientation, input.ballRepel.position, input.ballRepel.radius, BallRepelMargin);
        // Continuidade do alvo: na troca de dono do pe no ar (o planejador
        // parado -> a passada, na arrancada) o alvo saltava ate 15 cm num
        // tick (18 m/s): o pe era jogado para o clipe e raspava. O alvo novo
        // entra misturado a partir do ultimo em GoalHandoffSeconds.
        {
            const bool plannerOwned = m_footwork.owns(side);
            if (GoalHandoffSeconds > 0.0f && m_goalOwnerValid[side] && m_goalPlannerOwned[side] != plannerOwned
                && !m_feet[side].planted) {
                m_goalHandoff[side] = 1.0f;
                m_goalHandoffPosition[side] = m_lastFootGoalPosition[side];
                m_goalHandoffOrientation[side] = m_lastFootGoalOrientation[side];
            }
            m_goalPlannerOwned[side] = plannerOwned;
            m_goalOwnerValid[side] = true;
            if (m_feet[side].planted) m_goalHandoff[side] = 0.0f;
            if (m_goalHandoff[side] > 0.0f) {
                const float hold = smoothStep(m_goalHandoff[side]);
                goals[side].position += (m_goalHandoffPosition[side] - goals[side].position) * hold;
                goals[side].orientation = slerpQuaternion(goals[side].orientation, m_goalHandoffOrientation[side], hold);
                m_goalHandoff[side] = std::max(0.0f, m_goalHandoff[side] - deltaTime / GoalHandoffSeconds);
            }
            m_lastFootGoalPosition[side] = goals[side].position;
            m_lastFootGoalOrientation[side] = goals[side].orientation;
        }
        const Quaternion rootInverse = ikRootOrientation[side].conjugate();
        // O pe da bola (toque e chute) sai longe da pose do clipe: o IK
        // limitado a ~52 graus por tick (7 iteracoes de 0,13 rad) deixava o
        // joelho do chute sem dobrar no armado e o pe 6-25 cm aquem do alvo.
        // So no chute: no drible as 7 de sempre (com mais, o pe chegava
        // diferente e 3 de 32 toques no trote se perdiam).
        const bool ballDriven = input.ballKick.active && input.ballKick.shot && input.ballKick.side == static_cast<int>(side)
            && ((input.ballScript.active && input.ballScript.side == static_cast<int>(side))
                || (input.ballFoot.active && input.ballFoot.side == static_cast<int>(side) && input.ballFoot.weight > 0.0f));
        solveFootPose(profile, m_coordinates, m_feet[side].linkIndex,
            rootInverse.rotate(goals[side].position - ikRootPosition[side]),
            (rootInverse * goals[side].orientation).normalized(), 1.0f,
            // Andando, o rumo do pe so vem da coxa quando ha giro de curva
            // nele (reto, o pe segue a pelve: ver solveFootPose).
            // Com pouca ajuda, o pe de apoio segura o rumo da trava (o da
            // referencia): seguindo a pelve, nada a trazia de volta ao rumo
            // pedido e ela ia girando (de lado a 5%: 30-50 graus, e a passada
            // de lado virava curva). Pela coxa, o quadril de apoio a gira de
            // volta.
            !gaitMoving || (CurveFootHoldYaw && std::abs(m_feet[side].curveYaw) > 0.03f)
                || (GaitStanceHoldYaw && m_feet[side].planted)
                || (GaitHoldYaw > 0.5f && lowAssist > GaitHoldYawFrom
                    && (m_feet[side].planted || GaitHoldYaw > 1.5f)),
            ballDriven ? BallFootIterations : 7);
    }
    // Alcance das pernas com o pe no chao (R3): quadril pela pelve (fisica
    // e do IK), tornozelo pelo pe (fisico e alvo).
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        LegReachTelemetry3D& reach = m_telemetry.legReach[side];
        const FootPlant& plant = m_feet[side];
        const bool planted = plant.valid && goals[side].active && plant.planted
            && plant.linkIndex < state.links.size();
        const int shin = planted ? profile.links[plant.linkIndex].parentIndex : -1;
        const int thigh = shin > 0 ? profile.links[static_cast<std::size_t>(shin)].parentIndex : -1;
        if (thigh <= 0) {
            reach = {};
            m_previousLegLength[side] = { -1.0f, -1.0f };
            continue;
        }
        const RagdollLinkDefinition3D& footLink = profile.links[plant.linkIndex];
        const Vec3 hipModel = profile.links[static_cast<std::size_t>(thigh)].inboundJoint.anchorModelPosition;
        const Vec3 kneeModel = profile.links[static_cast<std::size_t>(shin)].inboundJoint.anchorModelPosition;
        const Vec3 ankleModel = footLink.inboundJoint.anchorModelPosition;
        const Vec3 hipLocal = profile.links[0].modelOrientation.conjugate()
            .rotate(hipModel - profile.links[0].modelPosition);
        const Vec3 ankleLocal = footLink.modelOrientation.conjugate()
            .rotate(ankleModel - footLink.modelPosition);
        const float nominal = (kneeModel - hipModel).length() + (ankleModel - kneeModel).length();
        const PhysicsBodyState3D& footBody = state.links[plant.linkIndex];
        const Vec3 hipPhysical = measuredPelvis.position + measuredPelvis.orientation.rotate(hipLocal);
        const Vec3 anklePhysical = footBody.position + footBody.orientation.rotate(ankleLocal);
        const Vec3 hipReference = ikRootPosition[side] + ikRootOrientation[side].rotate(hipLocal);
        const Vec3 ankleReference = goals[side].position + goals[side].orientation.rotate(ankleLocal);
        const float physicalLength = (hipPhysical - anklePhysical).length();
        const float referenceLength = (hipReference - ankleReference).length();
        reach.valid = nominal > 0.1f;
        reach.nominalLengthMeters = nominal;
        reach.physicalRatio = reach.valid ? physicalLength / nominal : 0.0f;
        reach.referenceRatio = reach.valid ? referenceLength / nominal : 0.0f;
        reach.physicalAxialSpeed = m_previousLegLength[side][0] >= 0.0f
            ? (physicalLength - m_previousLegLength[side][0]) / deltaTime : 0.0f;
        reach.referenceAxialSpeed = m_previousLegLength[side][1] >= 0.0f
            ? (referenceLength - m_previousLegLength[side][1]) / deltaTime : 0.0f;
        reach.ikRootHeight = ikRootPosition[side].z;
        reach.physicalRootHeight = measuredPelvis.position.z;
        m_previousLegLength[side] = { physicalLength, referenceLength };
    }

    // Diagnostico E5-A: alvo e resultado do IK (antes do limitador).
    const auto footClearance = [&](std::size_t side, Vec3 position, Quaternion orientation, float ground) {
        return position.z - footLockHeight3D(profile.links[m_feet[side].linkIndex], ground, orientation);
    };
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootTrace3D& trace = m_telemetry.footTrace[side];
        trace = {};
        if (!m_footTraceEnabled || !goals[side].active || !m_feet[side].valid) continue;
        const std::size_t link = m_feet[side].linkIndex;
        const PhysicsBodyState3D& body = state.links[link];
        const GroundSample ground = groundUnder(side, body.position,
            forwardOf(m_feet[side], body.orientation));
        trace.groundHeight = ground.valid ? ground.height : 0.0f;
        trace.target = m_trackedSwingGoalValid[side] ? m_trackedSwingGoal[side] : goals[side].position;
        m_trackedSwingGoalValid[side] = false;
        trace.targetClearance = footClearance(side, goals[side].position, goals[side].orientation, trace.groundHeight);
        RagdollAnimationPose3D ikPose;
        buildLocalPose(profile, m_coordinates, ikPose);
        trace.ik = ikRootPosition[side] + ikRootOrientation[side].rotate(ikPose.linkPositions[link]);
        trace.ikClearance = footClearance(side, trace.ik,
            (ikRootOrientation[side] * ikPose.linkOrientations[link]).normalized(), trace.groundHeight);
        trace.physical = body.position;
        trace.ikRoot = ikRootPosition[side];
        trace.ikRootOrientation = ikRootOrientation[side];
        trace.physicalRoot = measuredPelvis.position;
        trace.physicalRootOrientation = measuredPelvis.orientation;
        trace.physicalClearance = footClearance(side, body.position, body.orientation, trace.groundHeight);
        trace.valid = true;
    }

    // Nothing this controller commands may ask a joint to move faster than
    // a body can. The motors are driven with a velocity target taken from
    // the per-tick change in these coordinates, so at 120 Hz any jump is
    // multiplied by 120: a pose-side discontinuity of a few degrees becomes
    // a limb travelling metres per second. Measured before this: feet
    // spiking past 4 m/s while flagged as planted, on a body turning at
    // 0.17 rad/s. Clamping the commanded change bounds that whole class of
    // glitch at the point where it enters the physics, whatever produced it
    // upstream - clip, procedural pose or the foot IK.
    //
    // O teto vale para o que NAO e o movimento do proprio clipe: IK de pe,
    // olhar e a mistura do crossfade. O que a amostra do clipe anda de um
    // tick para o outro passa por cima do teto - o joelho do sprint chega a
    // 20 rad/s por construcao, e cortar isso deformava o ciclo. A mistura
    // nao: medido, deixa-la passar levava o pe a 17 m/s na troca entre
    // recuar e andar para a frente, com a perna indo de uma pose a outra em
    // 0,14 s.
    constexpr float MaximumJointRateRadiansPerSecond = 12.0f;
    const float maximumJointStep =
        MaximumJointRateRadiansPerSecond * deltaTime;
    // A perna que o planejador tem no ar pode flexionar como a de uma
    // passada (o joelho do sprint chega a ~20 rad/s): a 12 rad/s o joelho que
    // saia esticado da descarga levava ~75 ms para dobrar e o pe subia tarde
    // e baixo (medido no passo lateral: 5 cm cortados pelo limitador). So o
    // joelho e o tornozelo: o quadril, com o pe a ~0,9 m, a 25 rad/s fazia a
    // perna do boneco empurrado chicotear (mediana 13,4 m/s no membro, contra
    // 9,5 com ele a 12).
    std::vector<bool> swingLeg(m_coordinates.size(), false);
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPlan3D& planFoot = m_footwork.plan().feet[side];
        if (!m_feet[side].valid || planFoot.followingGait || planFoot.phase != FootPhase3D::Swing) continue;
        int index = static_cast<int>(m_feet[side].linkIndex);
        for (int joint = 0; joint < 2 && index > 0; ++joint) {
            swingLeg[static_cast<std::size_t>(index)] = true;
            index = profile.links[static_cast<std::size_t>(index)].parentIndex;
        }
    }
    constexpr float SwingLegJointRateRadiansPerSecond = 25.0f;
    for (std::size_t index = 1; index < m_coordinates.size(); ++index) {
        const Vec3 previous = previousCoordinates[index];
        Vec3& current = m_coordinates[index];
        const float maximumJointStep = (swingLeg[index] ? SwingLegJointRateRadiansPerSecond
            : MaximumJointRateRadiansPerSecond) * deltaTime;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const float delta = component(current, axis)
                - component(previous, axis);
            const float clipStep = selectedClip == m_previousSampledClip
                ? std::abs(component(sampled[index], axis)
                    - component(m_previousClipCoordinates[index], axis))
                : 0.0f;
            const float allowed = maximumJointStep + clipStep;
            if (std::abs(delta) > allowed) {
                setComponent(current, axis, component(previous, axis)
                    + std::copysign(allowed, delta));
            }
        }
    }
    m_previousClipCoordinates = sampled;

    // Levantar de bruços: a partir dos quatro apoios os pes sao do
    // equilibrio, nao do clipe (TEMPTUNE GetUpFootwork). Ficam presos no chao
    // onde estao - IK a partir da pelve medida, adiantada um pouco para a do
    // clipe (o descompasso e o empurrao das pernas) - e, quando o centro de
    // massa passa da ponta do pe mais a frente, o pe de tras da um passo para
    // baixo dele. As maos so soltam (o clipe espera) com o corpo sobre os
    // pes. Sem isso, sem ajuda, ele soltava as maos com o centro de massa
    // 0,4-0,6 m a frente dos pes e caia de cabeca.
    m_telemetry.getUpBalanceExcess = 0.0f;
    if (GetUpFootworkEnabled && getUpActive && m_getUpPhase == CharacterGetUpPhase3D::Rising
        && !m_bodilyRising && m_fallOrientation == CharacterFallOrientation3D::FaceDown
        && input.forceDrivenRoot && animations.standUpFront != nullptr && !state.links.empty()) {
        const float duration = std::max(0.05f, animations.standUpFront->durationSeconds);
        const float progress = m_getUpClipSeconds / duration;
        const char* footIds[2] = { "LeftFoot", "RightFoot" };
        const char* shinIds[2] = { "LeftShin", "RightShin" };
        std::size_t foot[2] {}, shin[2] {};
        bool linksOk = true;
        for (std::size_t side = 0; side < 2; ++side) {
            foot[side] = findLink(profile, footIds[side]);
            shin[side] = findLink(profile, shinIds[side]);
            linksOk = linksOk && foot[side] > 0 && shin[side] > 0
                && foot[side] < state.links.size() && shin[side] < state.links.size();
        }
        if (linksOk && progress >= GetUpFootworkFrom) {
            const PhysicsBodyState3D& pelvis = state.links.front();
            const auto ankleInShin = [&](std::size_t side) {
                const RagdollLinkDefinition3D& link = profile.links[foot[side]];
                const RagdollLinkDefinition3D& parent = profile.links[shin[side]];
                return parent.modelOrientation.conjugate().rotate(
                    link.inboundJoint.anchorModelPosition - parent.modelPosition);
            };
            const auto ankleWorld = [&](std::size_t side) {
                return state.links[shin[side]].position
                    + state.links[shin[side]].orientation.rotate(ankleInShin(side));
            };
            const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
            // O tornozelo acima da sola com o pe no chao: a ancora do pe no
            // repouso do perfil (13,5 cm no corpo antigo, 10,7 no montado).
            const float ankleHeight = profile.links[foot[0]].inboundJoint.anchorModelPosition.z + profile.standingRootHeightMeters;
            if (!m_getUpFootwork) {
                m_getUpFootwork = true;
                for (std::size_t side = 0; side < 2; ++side) m_getUpFootWorld[side] = ankleWorld(side);
                m_getUpStepSide = -1;
                // A frente do levantar: dos pes para as maos (de bruços a
                // frente da pelve aponta para o chao).
                const std::size_t hands[2] = { findLink(profile, "LeftHand"), findLink(profile, "RightHand") };
                Vec3 handMid {}, footMid = (m_getUpFootWorld[0] + m_getUpFootWorld[1]) * 0.5f;
                int handCount = 0;
                for (const std::size_t hand : hands)
                    if (hand > 0 && hand < state.links.size()) { handMid += state.links[hand].position; ++handCount; }
                Vec3 toHands = handCount > 0 ? handMid * (1.0f / static_cast<float>(handCount)) - footMid : Vec3 {};
                toHands.z = 0.0f;
                m_getUpFootworkForward = toHands.length() > 0.1f ? toHands.normalized()
                    : Vec3 { std::cos(m_getUpAnchorYaw), std::sin(m_getUpAnchorYaw), 0.0f };
                // Um pe ja no ar (o passo do clipe) continua como passo nosso.
                for (std::size_t side = 0; side < 2; ++side) {
                    if (m_getUpFootWorld[side].z > ground + ankleHeight + 0.12f && m_getUpStepSide < 0) {
                        m_getUpStepSide = static_cast<int>(side);
                        m_getUpStepSeconds = GetUpStepSeconds * 0.5f;
                        m_getUpStepFrom = m_getUpFootWorld[side];
                        m_getUpStepFrom.z = ground + ankleHeight;
                        const Vec3 comNow = m_estimator.state().centerOfMassWorld;
                        const float sideSign = side == 0 ? 1.0f : -1.0f;
                        Vec3 to = comNow + m_getUpFootworkForward * GetUpStepAhead
                            + Vec3 { -m_getUpFootworkForward.y, m_getUpFootworkForward.x, 0.0f } * (sideSign * GetUpStepWidth);
                        to.z = ground + ankleHeight;
                        m_getUpStepTo = to;
                        ++m_getUpBalanceSteps;
                    }
                }
            }
            const Vec3 forward = m_getUpFootworkForward;
            const Vec3 leftDirection { -forward.y, forward.x, 0.0f };
            const Vec3 com = m_estimator.state().centerOfMassWorld;
            // Quanto o centro de massa passa da ponta do pe mais a frente (so
            // os pes no chao).
            float front = -1e9f;
            for (std::size_t side = 0; side < 2; ++side) {
                if (static_cast<int>(side) == m_getUpStepSide) continue;
                front = std::max(front, dot(m_getUpFootWorld[side] - com, forward));
            }
            const float excess = -(front + GetUpToeAhead);
            m_telemetry.getUpBalanceExcess = excess;
            if (m_getUpStepSide < 0 && excess > -GetUpStepMargin && m_getUpBalanceSteps < 6) {
                // O pe de tras vai para baixo do centro de massa, do seu lado.
                const int rear = dot(m_getUpFootWorld[0] - com, forward)
                    < dot(m_getUpFootWorld[1] - com, forward) ? 0 : 1;
                m_getUpStepSide = rear;
                m_getUpStepSeconds = 0.0f;
                m_getUpStepFrom = m_getUpFootWorld[rear];
                const float sideSign = rear == 0 ? 1.0f : -1.0f;
                Vec3 to = com + forward * GetUpStepAhead + leftDirection * (sideSign * GetUpStepWidth);
                to.z = ground + ankleHeight;
                Vec3 move = to - m_getUpStepFrom;
                const float planar = std::hypot(move.x, move.y);
                if (planar > GetUpStepMax) {
                    move.x *= GetUpStepMax / planar;
                    move.y *= GetUpStepMax / planar;
                }
                m_getUpStepTo = m_getUpStepFrom + move;
                ++m_getUpBalanceSteps;
            }
            if (m_getUpStepSide >= 0) {
                m_getUpStepSeconds += deltaTime;
                const float s = std::clamp(m_getUpStepSeconds / std::max(0.05f, GetUpStepSeconds), 0.0f, 1.0f);
                Vec3 position = m_getUpStepFrom + (m_getUpStepTo - m_getUpStepFrom) * smoothStep(s);
                position.z += GetUpStepLift * std::sin(Pi * s);
                m_getUpFootWorld[static_cast<std::size_t>(m_getUpStepSide)] = position;
                if (s >= 1.0f) {
                    m_getUpFootWorld[static_cast<std::size_t>(m_getUpStepSide)] = m_getUpStepTo;
                    m_getUpStepSide = -1;
                }
            }
            const bool needHold = m_getUpStepSide >= 0 || excess > -GetUpStepMargin;
            m_getUpBalanceHoldSeconds = needHold ? m_getUpBalanceHoldSeconds + deltaTime : 0.0f;
            m_getUpBalanceHold = needHold && m_getUpBalanceHoldSeconds < GetUpHoldMax;
            // Pelve do IK: a medida, adiantada para a do clipe (limitado).
            Vec3 rootPosition = pelvis.position;
            rootPosition.z += std::clamp(targetRootPosition.z - pelvis.position.z, -GetUpRootLead, GetUpRootLead);
            Quaternion rootOrientation = pelvis.orientation;
            {
                const Vec3 turn = rotationVector((targetRootOrientation * pelvis.orientation.conjugate()).normalized());
                const float angle = turn.length();
                if (angle > 1e-4f)
                    rootOrientation = (exponentialRotation(turn * (std::min(angle, GetUpTurnLead) / angle))
                        * pelvis.orientation).normalized();
            }
            const Quaternion rootInverse = rootOrientation.conjugate();
            const Vec3 upInRoot = rootInverse.rotate({ 0.0f, 0.0f, 1.0f });
            for (std::size_t side = 0; side < 2; ++side) {
                const Vec3 target = rootInverse.rotate(m_getUpFootWorld[side] - rootPosition);
                solveRagdollChainPoint3D(profile, m_coordinates, shin[side], ankleInShin(side), 2, target, 1.0f, 12);
                // O pe que deu passo pousa com a sola no chao; o que ainda nao
                // saiu do lugar fica como o clipe (na ponta dos pes).
                const bool stepped = m_getUpBalanceSteps > 0
                    && (static_cast<int>(side) == m_getUpStepSide
                        || std::abs(m_getUpFootWorld[side].z - (ground + ankleHeight)) < 0.02f);
                if (!stepped) continue;
                const RagdollLinkDefinition3D& link = profile.links[foot[side]];
                float best = -2.0f;
                Vec3 bestAngles = m_coordinates[foot[side]];
                RagdollAnimationPose3D frame;
                for (int i = 0; i <= 12; ++i) {
                    Vec3 angles = m_coordinates[foot[side]];
                    setComponent(angles, 0, std::clamp(-0.85f + 1.85f * static_cast<float>(i) / 12.0f,
                        link.inboundJoint.axes[0].minimumRadians, link.inboundJoint.axes[0].maximumRadians));
                    m_coordinates[foot[side]] = angles;
                    buildRagdollLocalPose3D(profile, m_coordinates, frame);
                    const float alignment = dot(frame.linkOrientations[foot[side]].rotate({ 0.0f, 0.0f, 1.0f }), upInRoot);
                    if (alignment > best) {
                        best = alignment;
                        bestAngles = angles;
                    }
                }
                m_coordinates[foot[side]] = bestAngles;
            }
        }
    }
    // Mao de apoio no levantar de bruços (TEMPTUNE GetUpHands, ideia do
    // usuario 04/10). So pelas juntas, o clipe dizia onde a mao fica em
    // relacao ao corpo do clipe; com o corpo fisico um pouco diferente a mao
    // de apoio ia parar para dentro, embaixo do peito, e nao sustentava -
    // ele caia de lado. Aqui, quando o clipe pede a mao no chao, ela vai a um
    // ponto do chao ao lado e a frente do ombro (no mundo), com a palma para
    // baixo, e fica presa ali (IK a partir da pelve e da coluna medidas)
    // enquanto pes e joelhos vem para a frente. Solta quando o clipe levanta
    // a mao ou o braco nao alcanca mais, voltando ao clipe aos poucos.
    if (GetUpHandsEnabled && getUpActive && m_getUpPhase == CharacterGetUpPhase3D::Rising
        && !m_bodilyRising && m_fallOrientation == CharacterFallOrientation3D::FaceDown
        && input.forceDrivenRoot && !state.links.empty()
        && state.joints.size() == m_coordinates.size()) {
        const char* upperIds[2] = { "LeftUpperArm", "RightUpperArm" };
        const char* foreIds[2] = { "LeftForearm", "RightForearm" };
        const char* handIds[2] = { "LeftHand", "RightHand" };
        const auto anchorIn = [&](std::size_t child) {
            const RagdollLinkDefinition3D& link = profile.links[child];
            const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
            return parent.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - parent.modelPosition);
        };
        const PhysicsBodyState3D& pelvis = state.links.front();
        const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
        // Rumo do corpo: frente + cima da pelve no plano (estavel de bruços,
        // quando a frente aponta para o chao, e de pe).
        Vec3 heading = pelvis.orientation.rotate({ 1.0f, 0.0f, 0.0f })
            + pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f });
        heading.z = 0.0f;
        heading = heading.length() > 0.1f ? heading.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
        const Vec3 leftward { -heading.y, heading.x, 0.0f };
        // A pose do clipe (alvos das juntas) no referencial da pelve: onde o
        // clipe quer a mao (altura do chao do clipe).
        RagdollAnimationPose3D clipFrame;
        buildRagdollLocalPose3D(profile, m_coordinates, clipFrame);
        // Para o IK, a coluna medida (o ombro de verdade).
        std::vector<Vec3> measuredSpine = m_coordinates;
        for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
            const std::size_t i = findLink(profile, id);
            if (i > 0 && i < state.joints.size())
                measuredSpine[i] = { state.joints[i].positionRadians[0], state.joints[i].positionRadians[1],
                    state.joints[i].positionRadians[2] };
        }
        for (std::size_t side = 0; side < 2; ++side) {
            GetUpHand& hand = m_getUpHands[side];
            const std::size_t upper = findLink(profile, upperIds[side]);
            const std::size_t fore = findLink(profile, foreIds[side]);
            const std::size_t palm = findLink(profile, handIds[side]);
            if (upper == 0 || fore == 0 || palm == 0 || palm >= state.links.size()) continue;
            const Vec3 wristInFore = anchorIn(palm);
            const float sideSign = side == 0 ? 1.0f : -1.0f;
            // Mao do clipe: altura do punho acima do chao do clipe.
            const Vec3 clipWristLocal = clipFrame.linkPositions[fore] + clipFrame.linkOrientations[fore].rotate(wristInFore);
            const float clipWrist = m_getUpExpectedHeight + targetRootOrientation.rotate(clipWristLocal).z;
            const bool clipDown = m_getUpExpectedValid && clipWrist < GetUpHandClipDown;
            const Vec3 wristNow = state.links[fore].position + state.links[fore].orientation.rotate(wristInFore);
            const std::size_t chest = static_cast<std::size_t>(profile.links[upper].parentIndex);
            const Vec3 shoulder = state.links[chest].position + state.links[chest].orientation.rotate(anchorIn(upper));
            const float armLength = (profile.links[fore].inboundJoint.anchorModelPosition
                    - profile.links[upper].inboundJoint.anchorModelPosition).length()
                + (profile.links[palm].inboundJoint.anchorModelPosition
                    - profile.links[fore].inboundJoint.anchorModelPosition).length();
            hand.clipUpSeconds = clipDown ? 0.0f : hand.clipUpSeconds + deltaTime;
            if (hand.stage == 0 && clipDown) {
                hand.stage = 1;
                hand.seconds = 0.0f;
                hand.from = wristNow;
                Vec3 spot = shoulder + heading * GetUpHandForward + leftward * (sideSign * GetUpHandOutward);
                spot.z = ground + GetUpHandWristHeight;
                hand.spot = spot;
            }
            if (hand.stage == 1 || hand.stage == 2) {
                hand.seconds += deltaTime;
                const float out = (hand.spot - shoulder).length();
                if (hand.clipUpSeconds > 0.1f || out > 0.98f * armLength) {
                    hand.stage = 3;
                    hand.seconds = 0.0f;
                }
            }
            if (hand.stage == 1 && hand.seconds >= GetUpHandReachSeconds) hand.stage = 2;
            // Encostou antes no chao, num lugar bom (para fora do ombro): fica ali.
            if (hand.stage == 1 && wristNow.z < ground + GetUpHandWristHeight + 0.03f
                && dot(wristNow - shoulder, leftward) * sideSign > 0.02f) {
                hand.stage = 2;
                hand.spot = { wristNow.x, wristNow.y, ground + GetUpHandWristHeight };
            }
            const float wanted = hand.stage == 1 || hand.stage == 2 ? 1.0f : 0.0f;
            hand.weight += std::clamp(wanted - hand.weight, -deltaTime / GetUpHandReleaseSeconds,
                deltaTime / 0.12f);
            if (hand.stage == 3 && hand.weight <= 0.0f) hand.stage = 0;
            if (hand.weight <= 0.0f) continue;
            Vec3 target = hand.spot;
            if (hand.stage == 1) {
                const float s = std::clamp(hand.seconds / GetUpHandReachSeconds, 0.0f, 1.0f);
                target = hand.from + (hand.spot - hand.from) * smoothStep(s);
                target.z += 0.08f * std::sin(Pi * s);
            }
            std::vector<Vec3> ik = measuredSpine;
            const Vec3 local = pelvis.orientation.conjugate().rotate(target - pelvis.position);
            solveRagdollChainPoint3D(profile, ik, fore, wristInFore, 2, local, 1.0f, 12);
            // Palma para baixo: procura nos dois eixos do punho.
            const Vec3 upInRoot = pelvis.orientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
            const RagdollLinkDefinition3D& link = profile.links[palm];
            float best = -2.0f;
            Vec3 bestAngles = ik[palm];
            RagdollAnimationPose3D frame;
            for (int i = -3; i <= 3; ++i) {
                for (int j = -2; j <= 2; ++j) {
                    Vec3 angles = ik[palm];
                    if (link.inboundJoint.axes[0].enabled)
                        setComponent(angles, 0, std::clamp(0.35f * static_cast<float>(i),
                            link.inboundJoint.axes[0].minimumRadians, link.inboundJoint.axes[0].maximumRadians));
                    if (link.inboundJoint.axes[1].enabled)
                        setComponent(angles, 1, std::clamp(0.25f * static_cast<float>(j),
                            link.inboundJoint.axes[1].minimumRadians, link.inboundJoint.axes[1].maximumRadians));
                    ik[palm] = angles;
                    buildRagdollLocalPose3D(profile, ik, frame);
                    const float alignment = dot(frame.linkOrientations[palm].rotate({ 0.0f, 0.0f, 1.0f }), upInRoot);
                    if (alignment > best) {
                        best = alignment;
                        bestAngles = angles;
                    }
                }
            }
            ik[palm] = bestAngles;
            const float w = smoothStep(hand.weight);
            for (const std::size_t i : { upper, fore, palm })
                m_coordinates[i] += (ik[i] - m_coordinates[i]) * w;
        }
    } else {
        for (GetUpHand& hand : m_getUpHands) hand = {};
    }
    for (std::size_t side = 0; side < 2; ++side) m_telemetry.getUpHandStage[side] = m_getUpHands[side].stage;
    m_telemetry.getUpFootwork = m_getUpFootwork;
    m_telemetry.getUpBalanceSteps = m_getUpBalanceSteps;
    m_telemetry.getUpBalanceHold = m_getUpBalanceHold;
    m_telemetry.getUpRolling = m_getUpRolling;
    m_telemetry.tackling = m_tackling;
    m_telemetry.tacklePending = m_tacklePending;
    m_telemetry.tackleVariant = m_tackling ? m_tackleVariant : -1;
    m_telemetry.tackleEntrySpeed = m_tackleEntrySpeed;
    m_telemetry.tackleRate = m_tackleRate;
    m_telemetry.tackleTravel = m_tackling && !state.links.empty()
        ? std::hypot(state.links.front().position.x - m_tackleStart.x,
            state.links.front().position.y - m_tackleStart.y) : 0.0f;
    m_previousSampledClip = selectedClip;

    buildLocalPose(profile, m_coordinates, localPose);
    // Diagnostico E5-A: o comando final (depois do limitador), no referencial
    // do IK e a partir da pelve fisica.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootTrace3D& trace = m_telemetry.footTrace[side];
        if (!trace.valid) continue;
        const std::size_t link = m_feet[side].linkIndex;
        trace.command = ikRootPosition[side] + ikRootOrientation[side].rotate(localPose.linkPositions[link]);
        trace.commandClearance = footClearance(side, trace.command,
            (ikRootOrientation[side] * localPose.linkOrientations[link]).normalized(), trace.groundHeight);
        trace.commandFromPhysicalPelvis = measuredPelvis.position
            + measuredPelvis.orientation.rotate(localPose.linkPositions[link]);
        trace.commandFromPhysicalPelvisClearance = footClearance(side, trace.commandFromPhysicalPelvis,
            (measuredPelvis.orientation * localPose.linkOrientations[link]).normalized(), trace.groundHeight);
    }

    // Quanto cada pe ficou longe do alvo depois do IK e do limitador: pe
    // travado que a perna nao alcanca (limite do quadril descendo escada)
    // solta no proximo tick.
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        FootPlant& foot = m_feet[side];
        if (!goals[side].active || !foot.valid) {
            foot.reachError = 0.0f;
            continue;
        }
        const Vec3 solved = ikRootPosition[side] + ikRootOrientation[side].rotate(
            localPose.linkPositions[foot.linkIndex]);
        foot.reachError = (solved - goals[side].position).length();
    }

}

} // namespace MatterEngine
