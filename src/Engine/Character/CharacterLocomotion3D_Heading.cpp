#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepHeading(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const CharacterPhysicalState3D& physical = *t.physical;
    const bool& getUpActive = t.getUpActive;
    Vec3& desiredPlanarVelocity = t.desiredPlanarVelocity;
    const float& speed = t.speed;
    const bool& moving = t.moving;
    const float& viewYaw = t.viewYaw;
    // O angulo da camera chega em degraus: muda uma vez por quadro da tela, e
    // a fisica roda a 120 Hz. Para o olhar (cabeca, tronco, pelve girando no
    // lugar), uma mola criticamente amortecida o alisa - velocidade continua,
    // ~65 ms de atraso. Com o filtro de primeira ordem de antes, a velocidade
    // pedida a cabeca e ao tronco virava um dente de serra a cada quadro. A
    // escolha do setor do movimento continua com o angulo cru.
    if (!m_lookValid) {
        m_lookYaw = viewYaw;
        m_lookYawVelocity = 0.0f;
        m_lookPitch = std::clamp(input.lookPitchRadians, -0.60f, 0.50f);
        m_lookPitchVelocity = 0.0f;
        m_lookValid = true;
    }
    constexpr float LookOmega = 30.0f;
    m_lookYawVelocity += (LookOmega * LookOmega
            * wrapAngle(viewYaw - m_lookYaw)
        - 2.0f * LookOmega * m_lookYawVelocity) * deltaTime;
    m_lookYaw = wrapAngle(m_lookYaw + m_lookYawVelocity * deltaTime);
    m_lookPitchVelocity += (LookOmega * LookOmega
            * (std::clamp(input.lookPitchRadians, -0.60f, 0.50f) - m_lookPitch)
        - 2.0f * LookOmega * m_lookPitchVelocity) * deltaTime;
    m_lookPitch += m_lookPitchVelocity * deltaTime;
    t.lookYaw = m_lookYaw;
    const float& lookYaw = t.lookYaw;
    // While fallen/getting up, pelvis yaw is meaningless — orientation is
    // physics', not this integrator's — so none of it runs, and the turn
    // step accumulator (a stationary-only concept) is kept at zero instead
    // of quietly building up a step to fire the instant recovery finishes.
    t.turningInPlace = false;
    bool& turningInPlace = t.turningInPlace;
    if (!moving || getUpActive) {
        m_gaitDirection = CharacterGaitDirection3D::Forward;
    }
    // Taxa de giro do caminho (rad/s): a velocidade de navegacao girando
    // pela aceleracao filtrada (lateral / velocidade).
    t.pathSpeed = std::hypot(desiredPlanarVelocity.x, desiredPlanarVelocity.y);
    const float& pathSpeed = t.pathSpeed;
    // So com o jogador pedindo movimento: parando (sem pedido), a
    // desaceleracao da navegacao um pouco desalinhada parecia curva e ele
    // inclinava e virava a pelve na parada. Freando pela passada, tambem nao
    // (o perfil de freada nao e curva).
    t.curveIntent = smoothStep(std::hypot(input.intent.requestedDirectionWorld.x,
        input.intent.requestedDirectionWorld.y) / 0.5f) * (1.0f - m_brakeGait);
    const float& curveIntent = t.curveIntent;
    t.pathTurnRate = pathSpeed > 0.8f
        ? (desiredPlanarVelocity.x * m_smoothedAcceleration.y
            - desiredPlanarVelocity.y * m_smoothedAcceleration.x) / (pathSpeed * pathSpeed) * curveIntent
        : 0.0f;
    const float& pathTurnRate = t.pathTurnRate;
    bool pivoting = false;
    if (!getUpActive) {
        const float viewErrorBeforeTurn = wrapAngle(lookYaw - m_facingYaw);
        float targetPelvisYaw = m_facingYaw;
        if (moving) {
            // Olhar x movimento. O personagem olha para onde a camera aponta
            // (viewYaw); o movimento cai num setor e o setor escolhe o ciclo:
            // para a frente corre, de lado faz o strafe (de frente para a
            // camera), para tras recua - nunca da meia-volta. Dentro do setor
            // a pelve gira para o movimento; o tronco e a cabeca devolvem o
            // giro para a camera (bloco do olhar, mais abaixo).
            // A direcao pedida (as teclas), quando ha: a da capsula gira aos
            // poucos e a pelve passava pela diagonal ao trocar de "frente"
            // para "lado" antes de assentar no strafe.
            // Freando pela passada, o rumo e o do corpo: o pedido (nenhum, ou
            // o contrario) giraria a pelve no meio da freada.
            // Invertendo, abaixo da velocidade de pivo a pelve ja gira para o
            // rumo novo (na referencia Running Turn 180 ela gira desde a
            // freada: 37 graus ate 1,2 m/s, 100 no meio da troca).
            pivoting = m_brakeReversing && m_brakeGaitActive
                && std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y) < PivotSpeed;
            // (So enquanto a freada dura: no fim dela, com a mistura ainda
            // acima de meio, o rumo voltava a ser o da capsula - ainda o da
            // corrida - e o setor piscava frente/costas/frente numa inversao,
            // cada troca recomecando o clipe e soltando o pe que freava.)
            const bool brakeTravel = m_brakeGait > 0.5f && !pivoting
                && (!SectorHoldWhileBraking || m_brakeGaitActive);
            const Vec3 intended = brakeTravel ? Vec3 {}
                : Vec3 { input.intent.requestedDirectionWorld.x,
                    input.intent.requestedDirectionWorld.y, 0.0f };
            const float travelYaw = intended.lengthSquared() > 0.01f
                ? std::atan2(intended.y, intended.x)
                : std::atan2(desiredPlanarVelocity.y, desiredPlanarVelocity.x);
            const float travelFromView = wrapAngle(travelYaw - viewYaw);
            const bool strafe = animations.strafeLeft != nullptr
                && animations.strafeRight != nullptr;
            const float hysteresis = strafe ? StrafeSectorHysteresisRadians
                : PlainSectorHysteresisRadians;
            const auto available = [&](CharacterGaitDirection3D direction) {
                switch (direction) {
                case CharacterGaitDirection3D::Left:
                case CharacterGaitDirection3D::Right: return strafe;
                case CharacterGaitDirection3D::Backward:
                    return animations.walkBackward != nullptr;
                default: return true;
                }
            };
            const auto overrun = [&](CharacterGaitDirection3D direction) {
                return gaitSectorOverrun(direction, travelFromView, strafe);
            };
            // Freando forte (o jogador pediu o sentido oposto), o corpo freia
            // de frente para onde vinha e so troca de setor devagar; e, rapido,
            // nunca troca para o setor oposto (meia-volta da pelve): com a
            // camera virada 180 graus no sprint, a velocidade ainda para a
            // frente caia no setor "de costas" e a pelve girava na hora - ele
            // "corria de costas" a 7 m/s e caia. Ele freia de frente e vira
            // quando estiver devagar. So no modo fisico (no antigo a pose nao
            // carrega o corpo).
            const bool hardBraking = !pivoting && input.forceDrivenRoot && speed > 1.8f
                && dot(m_smoothedAcceleration, desiredPlanarVelocity) < -3.0f * speed;
            // Freando, o setor fica o da corrida ate a freada acabar: o corpo
            // freia de frente para onde vinha (entrando na freada, a mistura
            // ainda abaixo de meio pegava o rumo pedido e trocava o ciclo).
            const bool brakeHold = SectorHoldWhileBraking && m_brakeGaitActive && !pivoting
                && available(m_gaitDirection);
            if (!hardBraking && !brakeHold && (!available(m_gaitDirection)
                || overrun(m_gaitDirection) > hysteresis)) {
                // O setor que contem a direcao (sem recuo, o mais proximo).
                float best = std::numeric_limits<float>::infinity();
                CharacterGaitDirection3D chosen = m_gaitDirection;
                for (const CharacterGaitDirection3D candidate : {
                        CharacterGaitDirection3D::Forward,
                        CharacterGaitDirection3D::Left,
                        CharacterGaitDirection3D::Right,
                        CharacterGaitDirection3D::Backward }) {
                    if (available(candidate) && overrun(candidate) < best) {
                        best = overrun(candidate);
                        chosen = candidate;
                    }
                }
                const bool halfTurn = std::abs(wrapAngle(gaitSectorCenter(chosen)
                    - gaitSectorCenter(m_gaitDirection))) > 2.0f;
                if (!(input.forceDrivenRoot && halfTurn && speed > 2.5f
                        && available(m_gaitDirection))) {
                    m_gaitDirection = chosen;
                }
            }
            // O ciclo do setor anda na direcao do centro dele em relacao a
            // pelve: a pelve fica no movimento menos esse centro.
            targetPelvisYaw = wrapAngle(travelYaw
                - gaitSectorCenter(m_gaitDirection));
            // Correndo para a frente, a pelve vai no caminho do corpo: no
            // maximo um adiantamento para o pedido, menor quanto mais rapido.
            // Seguindo o pedido (a camera girando a 180-300 graus/s no
            // sprint), ela ficava 80-90 graus a frente do rumo real, o ciclo
            // de sprint punha os pes de lado para o movimento e ele tombava
            // para fora da curva; e o pe de apoio girava junto com ela no chao.
            const auto boundToPath = [&](float yaw) {
                const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x,
                    m_filteredCenterOfMassVelocity.y);
                const float share = smoothStep((bodySpeed - 1.0f) / 1.0f);
                if (share <= 0.0f) return yaw;
                const float pathYaw = std::atan2(m_filteredCenterOfMassVelocity.y,
                    m_filteredCenterOfMassVelocity.x);
                const float lead = PathLeadSlow + (PathLeadFast - PathLeadSlow)
                    * smoothStep((bodySpeed - 2.0f) / std::max(0.5f, PathLeadFastSpeed - 2.0f));
                const float offset = wrapAngle(yaw - pathYaw);
                const float bounded = std::clamp(offset, -lead, lead);
                return wrapAngle(yaw + (bounded - offset) * share);
            };
            const bool pathBound = PathLeadSlow < 3.0f && input.forceDrivenRoot && !pivoting
                && m_gaitDirection == CharacterGaitDirection3D::Forward;
            if (pathBound && !PathLeadAfterCurve) {
                targetPelvisYaw = boundToPath(targetPelvisYaw);
            }
            // Correndo em curva a pelve vai a frente do rumo, para dentro
            // (na referencia Sprint Turn, ~13 graus a 43 graus/s): o quadril
            // ja aponta para onde o caminho vai.
            if (m_gaitDirection == CharacterGaitDirection3D::Forward)
                targetPelvisYaw = wrapAngle(targetPelvisYaw + std::clamp(PelvisCurveLeadSeconds * pathTurnRate,
                    -PelvisCurveLeadMaximum, PelvisCurveLeadMaximum) * smoothStep((speed - 1.5f) / 1.5f));
            if (pathBound && PathLeadAfterCurve) targetPelvisYaw = boundToPath(targetPelvisYaw);
        } else if (std::abs(viewErrorBeforeTurn) > 0.55f
            || (std::abs(m_turningRate) > 0.05f
                && std::abs(viewErrorBeforeTurn) > 0.32f)) {
            // Olhar/coluna conservam cerca de 18 graus. O restante é
            // resolvido pela base com um passo de rotação, sem colar a
            // pelve na câmera. Depois de comecar, o giro vai ate os 18
            // graus: parar ao entrar nos 31 (a regra de antes) deixava o
            // alvo do giro mudando no meio dos passos.
            targetPelvisYaw = lookYaw - std::copysign(0.31f,
                viewErrorBeforeTurn);
        }
        float pelvisYawError = wrapAngle(targetPelvisYaw - m_facingYaw);
        // Giro parado pelo clipe (ajuda cheia; pedido do usuario, 04/10): no
        // lugar dos passos de giro do planejador, o Left/Right Turn tocado na
        // velocidade do giro pedido - a pelve segue o giro do clipe (escalado
        // ao que falta, ate ~86 graus por ciclo; mais que isso, outro ciclo) e
        // as pernas e os pes sao os do clipe.
        bool clipTurnControl = false;
        {
            // (O giro pelo clipe so existia com a ajuda cheia, removida em
            // 04/10: o modo fisico gira pelos passos do planejador.)
            const bool allowed = false;
            if (!allowed) m_clipTurn.active = false;
            const auto begin = [&](float error) {
                m_clipTurn.active = true;
                m_clipTurn.sign = error >= 0.0f ? 1.0f : -1.0f;
                m_clipTurn.clip = error >= 0.0f ? animations.turnLeft : animations.turnRight;
                m_clipTurn.span = std::max(0.3f, std::abs(clipRootYaw(*m_clipTurn.clip, profile,
                    m_clipTurn.clip->durationSeconds)));
                m_clipTurn.total = std::min(std::abs(error), m_clipTurn.span);
                m_clipTurn.time = 0.0f;
                m_clipTurn.startYaw = m_facingYaw;
                m_clipTurn.startPelvisYaw = m_facingYaw + m_clipPelvisYaw;
            };
            if (allowed && !m_clipTurn.active && std::abs(pelvisYawError) > ClipTurnEnter)
                begin(pelvisYawError);
            if (m_clipTurn.active) {
                clipTurnControl = true;
                const AnimationClip3D& clip = *m_clipTurn.clip;
                const float duration = std::max(0.05f, clip.durationSeconds);
                // A camera seguiu girando no comeco do ciclo: ele gira mais
                // (nunca menos, nem para o outro lado).
                if (m_clipTurn.time < 0.5f * duration) {
                    const float wanted = wrapAngle(targetPelvisYaw - m_clipTurn.startYaw) * m_clipTurn.sign;
                    m_clipTurn.total = std::clamp(wanted, m_clipTurn.total, m_clipTurn.span);
                }
                const float cycleSeconds = std::clamp(m_clipTurn.total / std::max(0.5f, ClipTurnSpeed),
                    ClipTurnMinSeconds, duration);
                m_clipTurn.time = std::min(duration, m_clipTurn.time + deltaTime * duration / cycleSeconds);
                const float progress = std::clamp(std::abs(clipRootYaw(clip, profile, m_clipTurn.time))
                    / m_clipTurn.span, 0.0f, 1.0f);
                const float yaw = wrapAngle(m_clipTurn.startYaw + m_clipTurn.sign * m_clipTurn.total * progress);
                m_turningRate = wrapAngle(yaw - m_facingYaw) / deltaTime;
                m_facingYaw = yaw;
                // O deslocamento da raiz do clipe (no rumo da pelve no comeco),
                // na mesma escala do giro.
                const Vec3 planarOffset = clipRootPlanar(clip, profile, m_clipTurn.time) * (m_clipTurn.total / m_clipTurn.span);
                const float c = std::cos(m_clipTurn.startPelvisYaw), sn = std::sin(m_clipTurn.startPelvisYaw);
                m_clipTurnRootOffset = { planarOffset.x * c - planarOffset.y * sn, planarOffset.x * sn + planarOffset.y * c, 0.0f };
                if (m_clipTurn.time >= duration - 1e-4f) {
                    m_clipTurn.active = false;
                    m_clipTurnRootOffset = {};
                    // Falta giro: o proximo ciclo (a base no fim e a do comeco,
                    // girada).
                    const float rest = wrapAngle(targetPelvisYaw - m_facingYaw);
                    if (allowed && std::abs(rest) > ClipTurnEnter) begin(rest);
                }
                pelvisYawError = wrapAngle(targetPelvisYaw - m_facingYaw);
            } else {
                m_clipTurnRootOffset = {};
            }
            const float wantedWeight = m_clipTurn.active ? 1.0f : 0.0f;
            m_clipTurnWeight = wantedWeight > m_clipTurnWeight
                ? std::min(1.0f, m_clipTurnWeight + deltaTime / std::max(0.01f, ClipTurnBlendIn))
                : std::max(0.0f, m_clipTurnWeight - deltaTime / std::max(0.01f, ClipTurnBlendOut));
        }
        if (!clipTurnControl) {
        // Virada (pivo): o lado e escolhido uma vez - o do pe apoiado (na
        // referencia o direito planta e ele gira para a direita) - e mantido:
        // a 180 graus o erro alternava entre +180 e -180 e o giro trocava de
        // sentido sem sair do lugar.
        if (!pivoting) {
            m_pivotSign = 0.0f;
        } else {
            if (m_pivotSign == 0.0f) {
                const auto& feetNow = physical.feet;
                m_pivotSign = std::abs(pelvisYawError) < 2.6f ? (pelvisYawError >= 0.0f ? 1.0f : -1.0f)
                    : feetNow[0].supporting != feetNow[1].supporting ? (feetNow[0].supporting ? 1.0f : -1.0f)
                    : m_turningRate >= 0.0f ? 1.0f : -1.0f;
            }
            if (pelvisYawError * m_pivotSign < 0.0f && std::abs(pelvisYawError) > 2.0f)
                pelvisYawError += m_pivotSign * 2.0f * Pi;
        }
        // Recem-levantado, o corpo primeiro firma a base: parado, nao gira
        // para a camera no primeiro meio segundo, e o giro entra aos poucos
        // ate ~1,5 s. Girando 100 graus logo na troca (taxa cheia), com um pe
        // ainda solto, ele inclinava 20 graus e caia de novo.
        // Recem-freado tambem: primeiro assenta a base da freada.
        // (So parado: pedindo movimento, logo depois de uma inversao pela
        // freada, o giro a 35% deixava a pelve para tras e ele saia correndo
        // de costas. Sem pedido, terminando a parada, ele ainda conta como
        // andando por um instante - ali o giro fica contido.)
        const bool requestingMove = std::hypot(input.intent.requestedDirectionWorld.x,
            input.intent.requestedDirectionWorld.y) > 0.1f;
        const float footing = smoothStep((m_sinceGetUpSeconds - 0.5f) / 1.0f)
            * (moving && requestingMove && BrakeFootingStandingOnly ? 1.0f
                : smoothStep((m_sinceBrakeSeconds - 0.3f) / 0.6f));
        // Parado, o giro e rapido (5 rad/s, 30 rad/s^2): a 2,35 rad/s e 10
        // rad/s^2 a pelve fazia ~25 graus no primeiro meio segundo, e o giro
        // parado parecia preso. Quem limita o que passa do apoio e o limite de
        // torcao do quadril (abaixo) e os passos de giro.
        // Pivo ainda rapido: o giro vai entrando conforme o corpo freia.
        const float pivotRate = pivoting ? 0.3f + 0.7f * smoothStep((PivotFullRateBelow
            - std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y)) / 3.0f) : 1.0f;
        // Com a ajuda parado alta, o giro parado e quase imediato: a pelve tem
        // forca para girar e os pes acompanham pivotando (abaixo).
        const float turnAssist = std::clamp(input.standingAssist, 0.0f, 1.0f);
        const float standingTurnRate = 5.0f + (StandingTurnRateHigh - 5.0f) * turnAssist;
        // Andando com pouca ajuda, a pelve gira no ritmo em que os pes
        // conseguem curvar o corpo: a aceleracao centripeta que o chao da
        // (~atrito x g) sobre a velocidade - a 5,5 m/s, ~1 rad/s, contra os 4,8
        // do pedido. Mais rapido que isso o corpo nao acompanhava, a pelve
        // passava da referencia e ele caia (zigue-zague a 30%).
        const float movingTurnAssist = smoothStep((0.6f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.5f);
        const float bodyPlanarSpeed = std::hypot(m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y);
        const float physicalTurnRate = std::max(LowAssistTurnMinimum,
            LowAssistTurnCentripetal / std::max(0.5f, bodyPlanarSpeed));
        const float movingTurnRate = (input.sprinting ? 4.8f : 3.8f);
        const float maximumTurnRate = (moving
                ? movingTurnRate + (std::min(movingTurnRate, physicalTurnRate) - movingTurnRate) * movingTurnAssist
                : standingTurnRate)
            * (moving ? 0.35f + 0.65f * footing : footing) * pivotRate;
        // Correndo para a frente, a pelve gira junto com o caminho (a taxa
        // dele entra direto) e o ganho so corrige o resto: so pelo ganho ela
        // ficava atrasada taxa/7 numa curva (16 graus a 90 graus/s, apontando
        // para fora) e os pes pousavam virados para fora. (Nos outros setores
        // nao: trocando da frente para o strafe o caminho gira mas a pelve
        // fica, e ela ia para a diagonal.)
        const float wantedTurnRate = std::clamp(
            pelvisYawError * (moving ? 7.0f : 10.0f + (StandingTurnGainHigh - 10.0f) * turnAssist)
                + (moving && m_gaitDirection == CharacterGaitDirection3D::Forward
                    ? PelvisCurveFeedForward * pathTurnRate : 0.0f),
            -maximumTurnRate, maximumTurnRate);
        const float maximumTurnAcceleration = moving ? 18.0f
            : 30.0f + (StandingTurnAccelerationHigh - 30.0f) * turnAssist;
        m_turningRate += std::clamp(wantedTurnRate - m_turningRate,
            -maximumTurnAcceleration * deltaTime,
            maximumTurnAcceleration * deltaTime);
        float yawStep = m_turningRate * deltaTime;
        if (std::abs(yawStep) > std::abs(pelvisYawError)
            && yawStep * pelvisYawError > 0.0f) {
            yawStep = pelvisYawError;
            m_turningRate = 0.0f;
        }
        m_facingYaw = wrapAngle(m_facingYaw + yawStep);
        // Parado, a pelve nao gira alem do que o quadril gira sobre um pe
        // apoiado (45 graus; 40 com folga): o giro vem dos pes. Sem isso a
        // pelve seguia a camera a ate 135 graus/s, e o pe apoiado ia junto,
        // esfregando no chao, ou a perna torcia no limite. So segura o que
        // passaria do limite neste tick; nunca empurra a pelve de volta.
        if (!moving && std::abs(yawStep) > 0.0f) {
            // A base em alerta ja traz cerca de 18 graus de rotacao entre
            // pelve e pes. Este limite e para o giro ADICIONAL que ainda
            // cabe sobre o apoio; 28 + 18 mantem a perna abaixo do gate
            // anatomico de 50 graus enquanto o outro pe contorna a base.
            const float HipTwistLimit = 0.48f + (StandingHipTwistHigh - 0.48f) * turnAssist;
            for (std::size_t side = 0; side < m_feet.size(); ++side) {
                const FootPlant& foot = m_feet[side];
                if (!foot.valid || !foot.planted || foot.repositioning) {
                    continue;
                }
                const Vec3 forward = foot.lockedOrientationWorld.rotate(
                    profile.links[foot.linkIndex].modelOrientation
                        .conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
                const float twist = wrapAngle(m_facingYaw + m_clipPelvisYaw
                    - std::atan2(forward.y, forward.x)
                    - m_clipPelvisToFootYaw[side]);
                if (std::abs(twist) > HipTwistLimit && twist * yawStep > 0.0f) {
                    const float excess = std::min(std::abs(yawStep),
                        std::abs(twist) - HipTwistLimit);
                    // Com a ajuda parado alta o pe gira junto, pivotando no
                    // chao: o giro nao espera os passos.
                    const float pivot = excess * std::clamp(StandingFootPivot * turnAssist, 0.0f, 1.0f);
                    if (pivot > 0.0f && m_footwork.owns(side))
                        m_footwork.pivotAnchor(profile, side, std::copysign(pivot, twist));
                    m_facingYaw = wrapAngle(m_facingYaw
                        - std::copysign(excess - pivot, twist));
                    // O pe bloqueou o deslocamento deste tick, mas o giro
                    // continua solicitado. Preservar a taxa mantem a
                    // histerese de giro ativa e permite que o planejador
                    // solte o proximo passo; zerar aqui abandonava uma
                    // camera lenta ainda com ~30 graus por resolver.
                }
            }
        }
        }
        if (std::abs(pelvisYawError) < 0.001f
            && std::abs(m_turningRate) < 0.02f && !clipTurnControl) {
            m_turningRate = 0.0f;
        }
        // Com histerese: sem ela, girando devagar o estado alternava entre
        // parado e girando no lugar a cada tick.
        turningInPlace = !moving && std::abs(m_turningRate)
            > (m_telemetry.state == CharacterLocomotionState3D::Turning
                ? 0.03f : 0.10f);
        m_standingTurnTarget = targetPelvisYaw;
    } else {
        m_turningRate = 0.0f;
        m_standingTurnTarget = m_facingYaw;
    }

}

} // namespace MatterEngine
