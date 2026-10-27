#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepCycle(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const CharacterPhysicalState3D& physical = *t.physical;
    Vec3& brakeNavigationVelocity = t.brakeNavigationVelocity;
    const float& speed = t.speed;
    const Vec3& localVelocity = t.localVelocity;
    const bool& gaitMoving = t.gaitMoving;
    const float& gaitCycleSpeed = t.gaitCycleSpeed;
    const AnimationClip3D*& standUpClip = t.standUpClip;
    const AnimationClip3D*& gaitClip = t.gaitClip;
    const bool& jumpPose = t.jumpPose;
    const bool& leapPose = t.leapPose;
    const AnimationClip3D*& idleClip = t.idleClip;
    const bool& authoredStartActive = t.authoredStartActive;
    // Curva pelo movimento real, inclusive de costas: v longitudinal com
    // sinal vezes a taxa de giro da pelve. Usar apenas |v| invertia o lado da
    // inclinacao ao correr para tras.
    t.signedCurveAcceleration = localVelocity.x * m_turningRate;
    const float& signedCurveAcceleration = t.signedCurveAcceleration;
    const float curveSpeedShare = smoothStep((speed - 1.0f) / 2.5f);
    const float desiredCurveBlend = authoredStartActive ? 0.0f
        : std::clamp(smoothStep((std::abs(signedCurveAcceleration) - 0.35f)
            / 3.5f) * curveSpeedShare, 0.0f, 0.72f);
    int desiredCurveKind = 0;
    if (gaitMoving && desiredCurveBlend > 0.001f) {
        if (m_gaitDirection == CharacterGaitDirection3D::Forward) {
            if (signedCurveAcceleration > 0.0f) {
                desiredCurveKind = animations.runForwardArcLeft != nullptr ? 1 : 0;
            } else {
                desiredCurveKind = animations.runForwardArcRight != nullptr ? 2 : 0;
            }
        } else if (m_gaitDirection == CharacterGaitDirection3D::Backward
            && signedCurveAcceleration < 0.0f) {
            desiredCurveKind = animations.runBackwardArcRight != nullptr ? 3 : 0;
        }
    }
    // Nunca troca uma captura por outra com peso alto: primeiro sai da atual,
    // depois entra na nova. Isso evita um chicote de perna ao cruzar um setor
    // ou inverter a curva.
    const bool changingCurve = desiredCurveKind != m_authoredCurveKind;
    const float curveBlendTarget = changingCurve ? 0.0f : desiredCurveBlend;
    const float curveBlendRate = curveBlendTarget > m_authoredCurveBlend
        ? 2.5f : 4.0f;
    m_authoredCurveBlend += std::clamp(curveBlendTarget - m_authoredCurveBlend,
        -curveBlendRate * deltaTime, curveBlendRate * deltaTime);
    if (changingCurve && m_authoredCurveBlend <= 0.001f)
        m_authoredCurveKind = desiredCurveKind;
    t.authoredCurveBlend = m_authoredCurveBlend;
    const float& authoredCurveBlend = t.authoredCurveBlend;
    t.authoredCurveKind = m_authoredCurveKind;
    const int& authoredCurveKind = t.authoredCurveKind;
    t.authoredCurveClip = authoredCurveKind == 1
        ? animations.runForwardArcLeft : authoredCurveKind == 2
        ? animations.runForwardArcRight : authoredCurveKind == 3
        ? animations.runBackwardArcRight : nullptr;
    const AnimationClip3D*& authoredCurveClip = t.authoredCurveClip;
    t.selectedClip = standUpClip != nullptr ? standUpClip
        : jumpPose ? (leapPose ? m_leapClip : m_jumpClip)
        : !gaitMoving ? idleClip : gaitClip;
    const AnimationClip3D*& selectedClip = t.selectedClip;
    // Trocar de ciclo sem trocar de estado (troca de setor, passada <->
    // sprint) precisa do mesmo crossfade que a troca de estado faz em
    // enterState. Entre setores ele e mais longo: o pe muda de trajetoria
    // enquanto a pelve vira ~90 graus, e em 0,14 s o alvo do pe chegava a
    // 15 m/s (medido na varredura de direcao, entre frente e recuo).
    if (selectedClip != m_previousClip) {
        if (m_previousClip != nullptr) {
            m_transitionFrom = m_poseCoordinates.size()
                    == m_coordinates.size()
                ? m_poseCoordinates : m_coordinates;
            m_transitionFromRootTilt = m_rootTilt;
            m_transitionVelocity = m_poseVelocities;
            m_transitionRootTiltVelocity = m_rootTiltVelocity;
            m_transitionRootOffsetVelocity = m_rootOffsetVelocity;
            m_transitionFromRootOffset = m_rootOffset;
            m_transitionSeconds = deltaTime;
            // Troca de setor: o ciclo novo anda em outra direcao em relacao a
            // pelve (frente, lado, costas).
            const bool gaitFlip = gaitMoving && selectedClip != nullptr
                && selectedClip != idleClip
                && m_previousClip != idleClip
                && std::abs(wrapAngle(selectedClip->travelDirectionRadians
                    - m_previousClip->travelDirectionRadians)) > 0.1f;
            m_transitionDurationSeconds = gaitFlip
                ? GaitFlipTransitionSeconds : DefaultTransitionSeconds;
            if (gaitFlip) m_sinceGaitFlip = 0.0f;
        }
        m_previousClip = selectedClip;
    }

    // O ciclo anda com a capsula: ritmo dentro de uma faixa natural, e o
    // resto da velocidade vira passo mais curto ou mais longo (stride
    // warping, nos pes, mais abaixo). Com o ritmo sozinho travado em 0,35 o
    // pe andava mais que o corpo devagar, e o pe de apoio escorregava.
    t.playbackRate = 1.0f;
    float& playbackRate = t.playbackRate;
    t.strideScale = 1.0f;
    float& strideScale = t.strideScale;
    // Escada e ladeira: o chao a frente, na direcao do movimento. Quanto ele
    // sobe (ou desce) por metro limita o passo: cada passo sobe no maximo
    // StepRiseLimit (dois degraus), como quem sobe escada correndo. Na
    // passada inteira do trote (1,1 m) o pe pousava 4 degraus acima ou
    // abaixo - pernas contorcidas subindo, pe pendurado descendo.
    float terrainSlope = 0.0f;
    bool terrainIsRamp = false;
    float terrainDemand = 0.0f;
    Vec3 planarTravel { input.rootVelocityWorld.x,
        input.rootVelocityWorld.y, 0.0f };
    planarTravel += (brakeNavigationVelocity - planarTravel) * m_brakeGait;
    if (input.groundAt && gaitMoving && input.grounded
        && planarTravel.lengthSquared() > 0.01f) {
        const Vec3 ahead = planarTravel.normalized();
        const float base = input.rootPositionWorld.z
            - profile.standingRootHeightMeters;
        // A 35 cm um unico espelho de degrau ja dava inclinacao de 0,5 a 1:
        // medir mais longe mede a escada, nao o degrau.
        // Correndo rapido olha mais longe: do sprint a 7 m/s, vista a 2,1 m,
        // a escada chegava antes de o corpo conseguir frear.
        std::array<float, 7> rise {};
        std::array<float, 7> at {};
        std::size_t count = 0;
        int tilted = 0;
        for (const float distance : { 0.70f, 1.05f, 1.40f, 1.75f, 2.10f, 2.80f, 3.50f }) {
            if (distance > 1.40f && (!input.forceDrivenRoot || speed < 4.0f)) continue;
            if (distance > 2.10f && speed < 5.0f) continue;
            Vec3 point = input.rootPositionWorld + ahead * distance;
            point.z = base;
            const GroundProbeResult3D probe = input.groundAt(point);
            if (!probe.hasSurface) continue;
            rise[count] = probe.pointWorld.z - base;
            at[count] = distance;
            ++count;
            if (probe.slopeDegrees > 4.0f && probe.slopeDegrees < 45.0f) ++tilted;
        }
        // So a subida (ou descida) que continua - ladeira, escada - pede
        // passo curto e corpo mais devagar: a altura cresce com a distancia,
        // sempre no mesmo sentido. Um degrau so (meio-fio, calcada: sobe e
        // fica plano) ou uma barra no caminho nao: o pe passa por cima dele
        // no passo normal (a folga do balanco ve o obstaculo). Tratado como
        // ladeira, um meio-fio de 12 cm levava o sprint de 7,3 a 4,5 m/s.
        for (std::size_t i = 0; i < count; ++i)
            if (std::abs(rise[i]) > 0.06f) terrainDemand = 1.0f;
        // No modo cinematico (a capsula carrega o corpo), a regra de antes:
        // qualquer desnivel a frente encurta o passo.
        if (!input.forceDrivenRoot)
            for (std::size_t i = 0; i < count; ++i)
                terrainSlope = std::max(terrainSlope, std::abs(rise[i]) / at[i]);
        // So a subida (ou descida) que continua - ladeira, escada -: a altura
        // nunca volta contra o sentido e sobe (3 cm ou mais) em pelo menos
        // dois trechos entre sondas (numa escada um trecho pode cair num
        // mesmo piso). Um degrau so (sobe e fica plano), uma barra no caminho
        // ou um degrau que aparece so na sonda mais longe nao contam. A
        // inclinacao e a do trecho do primeiro degrau ate a sonda mais longe.
        if (count >= 3 && input.forceDrivenRoot) {
            const float direction = rise[count - 1] - rise[0];
            int rising = 0;
            bool against = false;
            std::size_t first = count;
            for (std::size_t i = 1; i < count; ++i) {
                const float step = rise[i] - rise[i - 1];
                if (step * direction < 0.0f && std::abs(step) > 0.02f) against = true;
                if (std::abs(step) >= 0.03f && step * direction > 0.0f) {
                    ++rising;
                    if (first == count) first = i - 1;
                }
            }
            if (!against && rising >= 2) {
                terrainSlope = std::abs(rise[count - 1] - rise[first])
                    / std::max(0.1f, at[count - 1] - at[first]);
            } else if (!against && rising >= 1 && std::abs(rise[0]) >= 0.30f) {
                // No meio da subida (o chao a 0,7 m ja 30 cm acima ou abaixo):
                // o fim da escada a vista nao a desfaz - os ultimos degraus
                // ainda sao escada.
                terrainSlope = std::abs(rise[count - 1]) / std::max(0.1f, at[count - 1]);
            }
        }
        // Reconhecida a subida continua, por 0,6 s vale a medida de antes (o
        // maior desnivel sobre a distancia): em cima da escada a sequencia
        // subindo se desfaz entre degraus, o teto subia no meio dela e o
        // corpo tombava.
        if (input.forceDrivenRoot) {
            m_sustainedTerrainHold = terrainSlope > 0.08f ? 0.6f
                : std::max(0.0f, m_sustainedTerrainHold - deltaTime);
            if (m_sustainedTerrainHold > 0.0f)
                for (std::size_t i = 0; i < count; ++i)
                    terrainSlope = std::max(terrainSlope, std::abs(rise[i]) / at[i]);
        }
        // Rampa (a superficie sondada inclinada, sem espelhos): o passo pode
        // subir mais que numa escada.
        terrainIsRamp = tilted * 2 >= static_cast<int>(count) && count > 0;
    }
    // Quanto um passo sobe no maximo: numa escada, um ou dois espelhos; numa
    // rampa (sem espelho), mais - no passo de escada, uma rampa de 20 graus
    // levava o sprint a 39% da velocidade.
    const float StepRiseLimit = !input.forceDrivenRoot ? 0.34f
        : (terrainIsRamp || m_terrainRampHold > 0.0f) ? 0.32f : 0.20f;
    // (A rampa segura a classificacao por um tempo: entre duas sondas o
    // plano da rampa some.)
    if (terrainSlope > 0.08f) terrainDemand = 1.0f;
    m_terrainDemand = terrainDemand > m_terrainDemand ? terrainDemand
        : std::max(0.0f, m_terrainDemand - deltaTime / 0.6f);
    m_telemetry.terrainDemand = m_terrainDemand;
    m_terrainRampHold = terrainIsRamp ? 0.3f : std::max(0.0f, m_terrainRampHold - deltaTime);
    m_terrainIsRamp = m_terrainRampHold > 0.0f;
    // Sobe rapido (o degrau aparece de uma vez) e solta devagar.
    m_terrainSlope += (terrainSlope - m_terrainSlope) * (1.0f - std::exp(
        -(terrainSlope > m_terrainSlope ? 20.0f : 4.0f) * deltaTime));
    t.terrainSpeedLimit = std::numeric_limits<float>::infinity();
    float& terrainSpeedLimit = t.terrainSpeedLimit;
    if (input.forceDrivenRoot && m_terrainSlope > 0.08f) {
        // Ladeira e escada: o corpo desacelera na proporcao da inclinacao
        // (10 graus ~90% da velocidade pedida, 20 graus ~70%, escada ~55%),
        // com um teto para escada ingreme. (Era um teto fixo de 0,48 /
        // inclinacao, ate 2,8 m/s: uma rampa de 10 graus levava o sprint de
        // 7,3 a 3,3 m/s.)
        // (Absoluto, sobre a velocidade de sprint: o jogo ja corta o pedido
        // por este teto, e calculado do pedido ele descia tick a tick.)
        constexpr float TerrainTopSpeed = 7.5f;
        const float share = std::clamp(1.0f - 1.1f * (m_terrainSlope - 0.08f), 0.55f, 1.0f);
        terrainSpeedLimit = TerrainTopSpeed * share;
        // Escada (subida continua sem ser rampa): cada degrau precisa de
        // tempo para o corpo subir o espelho - o teto de antes, continuo (so
        // acima de uma inclinacao ele chegava rapido demais e caia).
        if (!m_terrainIsRamp)
            terrainSpeedLimit = std::min(terrainSpeedLimit, std::max(0.85f, 0.48f / m_terrainSlope));
    }
    // Parado (sem passada), as pernas voltam ao mesmo ciclo.
    if (!gaitMoving) {
        m_armPhaseValid = false;
        m_phaseRateScale = 1.0f;
        m_contactSteady = false;
        m_legPhaseLead = {};
        m_waitingLeg = -1;
        m_touchdownWaitSeconds = 0.0f;
    }
    if (gaitMoving && selectedClip != nullptr) {
        const float authoredSpeed = authoredHorizontalSpeed(*selectedClip);
        if (authoredSpeed > 0.05f) {
            const float ratio = gaitCycleSpeed / authoredSpeed;
            // Passo do clipe (dois por ciclo) e o maior que o terreno deixa.
            const float authoredStep = authoredSpeed
                * selectedClip->durationSeconds * 0.5f;
            const float strideCap = m_terrainSlope > 0.05f
                ? std::clamp(StepRiseLimit / (m_terrainSlope * authoredStep),
                    0.25f, 1.0f)
                : 1.35f;
            // Com pouca ajuda o corpo acelera devagar e a passada fica bem
            // abaixo do clipe: com o ritmo caindo ate 0,55 a cadencia ficava
            // lentissima com a passada inteira (o pe nao chegava a frente a
            // tempo). Quem arranca da passos rapidos e curtos: o ritmo fica
            // alto e a passada encurta.
            const float lowAssistRate = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
            // So correndo (pedido acima do trote, como a folga da passada): no
            // trote, parando, o ritmo alto derrubava (5 -> 10-21 quedas em 32).
            const bool runRequest = std::hypot(input.intent.requestedVelocityWorld.x,
                input.intent.requestedVelocityWorld.y) > GaitLeadMinimumRequest;
            const float rateFloor = 0.55f + (std::max(0.55f, LowAssistRateFloor) - 0.55f) * lowAssistRate
                * (runRequest ? 1.0f : 0.0f) * (LowAssistRateFloorInBrake ? 1.0f : 1.0f - m_brakeGait);
            const float sprintFloor = selectedClip == animations.sprint
                ? SprintRateFloor * (1.0f - m_brakeGait) + 0.55f * m_brakeGait : 0.55f;
            playbackRate = std::clamp(ratio / std::min(1.0f, strideCap),
                std::max(rateFloor, sprintFloor), 1.45f);
            // Quase-queda: o passo seguinte sai e pousa mais cedo (passada
            // rapida); o comprimento fica o do pouso deslocado.
            if (StumbleDetection && m_tripRecovery > 0.0f)
                playbackRate *= 1.0f + StumbleCadence * m_tripRecovery;
            // O gancho do ritmo (o RL): antes da compensacao - a passada abaixo
            // encurta e a velocidade implicita fica a mesma.
            if (input.corrections != nullptr) playbackRate *= cadenceFactor3D(input.corrections);
            strideScale = std::clamp(ratio / playbackRate, 0.0f, strideCap);
            // O gancho do comprimento (o RL): depois da compensacao, no mesmo
            // ritmo (ate o teto do terreno).
            if (input.corrections != nullptr)
                strideScale = std::min(strideScale * strideFactor3D(input.corrections), strideCap);
            // Preparo do carrinho: o passo que pousa e o de firmar o pe.
            if (m_tacklePending)
                strideScale *= 1.0f + TacklePlantStride * smoothStep(m_tacklePendingSeconds / 0.10f);
            // Velocidade que cabe no terreno com o passo limitado e o ritmo
            // ate 15% acima do clipe; o jogo a usa como teto da capsula.
            if (strideCap < 1.0f) {
                terrainSpeedLimit = std::min(terrainSpeedLimit,
                    authoredSpeed * strideCap * 1.15f);
            }
            // TEMPTUNE: passada pelo contato (pedido do usuario, 04/10). Correndo,
            // a sequencia segue os pes, nao o relogio do clipe:
            //  - o pe que vai pousar (o clipe ja o poe no apoio) e ainda esta
            //    no ar, com o outro tambem fora do chao, ESPERA o toque: o
            //    ciclo para nesse ponto, a perna estendida para o chao;
            //  - a perna de tras continua recolhendo no seu ritmo (adianta-se
            //    ao ciclo, ate o meio do balanco). Congelar o ciclo inteiro a
            //    deixava baixa, atras, enquanto o corpo andava ~0,5 m: pegava
            //    a barra do tropeco (7 -> 36 quedas em 96);
            //  - tocou, o ciclo recupera a diferenca (a perna da frente anda
            //    mais rapido no apoio ate as duas casarem de novo);
            //  - o pe que encosta no fim do balanco, antes do toque do clipe,
            //    adianta o ciclo ate o apoio (em vez de arrastar a ponta).
            // A espera vai ate ContactWaitSeconds (0,08 s, o voo normal do
            // sprint): mais longa, depois de bater numa barra o ciclo ficava
            // esperando o equivalente a ~1-2 m de corrida e o pe descia em
            // cima dela (tropeco do sprint: 0 -> 17 quedas em 64 com 0,12 s).
            // So correndo para a frente, estabelecido: nada disso na freada,
            // na arrancada, no tropeco, em degrau ou trocando de passada.
            float phaseRate = playbackRate;
            const float cycleSeconds = std::max(0.1f, selectedClip->durationSeconds);
            const bool stumbling = m_tripRecovery > 0.001f || m_stumbleRisk > 0.2f
                || m_stumbleStepping || m_footwork.plan().recovering;
            // (No movimento livre tambem: a espera nao sustenta o corpo, so nao
            // deixa a perna esticar com o pe ainda chegando - sem ela a pelve
            // subia ate 12 cm acima da referencia no sprint, contra 6.)
            const bool contactSteady = ContactPhase && input.forceDrivenRoot && physical.feet.size() >= 2
                && m_feet.size() >= 2 && !stumbling
                && m_gaitDirection == CharacterGaitDirection3D::Forward
                && m_brakeGait <= 0.0f && !m_brakeStepping && !m_startClipActive && m_gaitChange <= 0.0f
                && m_terrainSlope < 0.05f
                && std::hypot(input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y) > 1.0f;
            int waitingLeg = -1;
            bool earlyTouch = false;
            for (std::size_t side = 0; side < 2 && side < physical.feet.size(); ++side) {
                const FootSupportEstimate3D& foot = physical.feet[side];
                const bool down = foot.touching || foot.supporting;
                const bool clipStance = m_telemetry.clipFootContact[side] > 0.55f;
                if (!clipStance) m_legTouchedInStance[side] = false;
                else if (down) m_legTouchedInStance[side] = true;
                if (!contactSteady) continue;
                if (clipStance && !m_legTouchedInStance[side] && !physical.feet[1 - side].supporting)
                    waitingLeg = static_cast<int>(side);
                if (!clipStance && m_telemetry.footSwingProgress[side] >= ContactEarlyProgress && down)
                    earlyTouch = true;
            }
            m_contactSteady = contactSteady;
            m_waitingLeg = waitingLeg >= 0 && m_touchdownWaitSeconds < ContactWaitSeconds ? waitingLeg : -1;
            if (waitingLeg >= 0 && m_touchdownWaitSeconds < ContactWaitSeconds) {
                m_touchdownWaitSeconds += deltaTime;
                // A perna de tras segue recolhendo, ate o meio do balanco.
                const std::size_t trailing = static_cast<std::size_t>(1 - waitingLeg);
                if (m_telemetry.footSwingProgress[trailing] >= 0.0f
                    && m_telemetry.footSwingProgress[trailing] < ContactTrailCap)
                    m_legPhaseLead[trailing] = std::min(ContactMaxLead,
                        m_legPhaseLead[trailing] + deltaTime * playbackRate / cycleSeconds);
                phaseRate = 0.0f;
            } else {
                if (waitingLeg < 0) m_touchdownWaitSeconds = 0.0f;
                if (earlyTouch) phaseRate *= ContactEarlyBoost;
            }
            const float phaseBefore = m_cyclePhase;
            // O ciclo recupera a diferenca das pernas: a parte comum vai para
            // ele de uma vez (nenhuma perna muda), o resto aos poucos.
            {
                const float common = std::min(m_legPhaseLead[0], m_legPhaseLead[1]);
                m_legPhaseLead[0] -= common;
                m_legPhaseLead[1] -= common;
                float extra = common;
                if (phaseRate > 0.0f) {
                    // (Fora da corrida estabelecida - freando, por exemplo - a
                    // diferenca sai mais rapido, mas continua: zerada de uma
                    // vez, a perna que pousava saltava de pose no comeco da
                    // freada.)
                    const float rate = ContactCatchUp * (contactSteady ? 1.0f : 3.0f);
                    for (std::size_t side = 0; side < 2; ++side) {
                        const float absorb = std::min(m_legPhaseLead[side],
                            deltaTime * std::max(playbackRate, 0.5f) / cycleSeconds * rate);
                        m_legPhaseLead[side] -= absorb;
                        extra += absorb;
                    }
                }
                m_cyclePhase = std::fmod(m_cyclePhase + extra + 1.0f, 1.0f);
            }
            m_telemetry.touchdownWaitSeconds = m_touchdownWaitSeconds;
            m_telemetry.legPhaseLead = m_legPhaseLead;
            // TEMPTUNE: o ritmo do ciclo muda aos poucos - freia rapido (a
            // perna da frente para na pose de toque) e retoma devagar. Em
            // degraus (parar, voltar, 3x no toque adiantado) braco e perna
            // davam trancos: aceleracao dos alvos do braco 4x maior no sprint.
            // (No tropeco e na quase-queda nao: ali o passo rapido que salva
            // precisa do ciclo na hora, e os bracos presos a ele - suavizados,
            // o sprint na barra escondida caia muito mais.)
            if (ContactRateSmooth && stumbling) {
                m_phaseRateScale = playbackRate > 1e-4f ? phaseRate / playbackRate : 1.0f;
            } else if (ContactRateSmooth) {
                const float wanted = playbackRate > 1e-4f ? phaseRate / playbackRate : 1.0f;
                const float tau = wanted < m_phaseRateScale ? ContactRateDownSeconds : ContactRateUpSeconds;
                m_phaseRateScale += (wanted - m_phaseRateScale) * (1.0f - std::exp(-deltaTime / std::max(0.005f, tau)));
                phaseRate = playbackRate * m_phaseRateScale;
            }
            m_cyclePhase = std::fmod(m_cyclePhase + deltaTime * phaseRate
                / std::max(0.1f, selectedClip->durationSeconds), 1.0f);
            // TEMPTUNE: os bracos andam numa fase propria, suave: seguem o
            // ritmo medio do ciclo e se prendem a ele devagar. Esperando o
            // toque o ciclo para, mas o corredor nao congela os bracos no ar.
            {
                float advanced = m_cyclePhase - phaseBefore;
                if (advanced < -0.5f) advanced += 1.0f;
                const float globalRate = advanced / std::max(1e-4f, deltaTime);
                if (!m_armPhaseValid || ArmPhaseSmoothSeconds <= 0.0f || stumbling) {
                    m_armPhase = m_cyclePhase;
                    m_armPhaseRate = globalRate;
                    m_armPhaseValid = ArmPhaseSmoothSeconds > 0.0f;
                } else {
                    m_armPhaseRate += (globalRate - m_armPhaseRate) * (1.0f - std::exp(-deltaTime / ArmPhaseSmoothSeconds));
                    float error = m_cyclePhase - m_armPhase;
                    error -= std::round(error);
                    m_armPhase = std::fmod(m_armPhase + (m_armPhaseRate + error / ArmPhaseLockSeconds) * deltaTime + 1.0f, 1.0f);
                }
            }
            m_varietyStrides += deltaTime * phaseRate
                / std::max(0.1f, selectedClip->durationSeconds);
        }
    }

    // Saindo de escada ou rampa o teto sobe aos poucos (2 m/s2) enquanto o
    // corpo ainda esta nela; entrando, baixa na hora. As sondas veem o
    // patamar ~0,7 m antes e o teto saltava com o corpo ainda no ultimo
    // degrau: de 0,55 a 3 m/s em 0,35 s, ele tombava para a frente no topo
    // da escada. (Com o corpo ainda no plano - chegando numa rampa, a
    // estimativa oscilando - o teto acompanha na hora: devagar, ficava preso
    // no menor valor e o sprint chegava a rampa a 5,3 m/s em vez de 7.)
    {
        bool incline = false;
        float lowest = std::numeric_limits<float>::infinity();
        float highest = -lowest;
        int planted = 0;
        for (const FootPlant& plant : m_feet) {
            if (!plant.valid || !plant.planted) continue;
            ++planted;
            lowest = std::min(lowest, plant.groundHeight);
            highest = std::max(highest, plant.groundHeight);
            if (plant.groundNormal.z < 0.9976f) incline = true;  // ~4 graus
        }
        if (planted == 2 && highest - lowest > 0.05f) incline = true;
        m_onInclineHold = incline ? 0.6f : std::max(0.0f, m_onInclineHold - deltaTime);
        constexpr float TerrainCapCeiling = 9.0f, TerrainCapRecovery = 2.0f;
        if (input.forceDrivenRoot) {
            const float wanted = std::min(terrainSpeedLimit, TerrainCapCeiling);
            m_terrainSpeedCap = wanted < m_terrainSpeedCap || m_onInclineHold <= 0.0f ? wanted
                : std::min(wanted, m_terrainSpeedCap + TerrainCapRecovery * deltaTime);
            if (m_terrainSpeedCap < TerrainCapCeiling) terrainSpeedLimit = m_terrainSpeedCap;
        } else {
            m_terrainSpeedCap = TerrainCapCeiling;
        }
    }
    t.sampled = std::vector<Vec3>(profile.links.size(), Vec3 {});
    std::vector<Vec3>& sampled = t.sampled;
    t.rootRotation = {};
    Quaternion& rootRotation = t.rootRotation;
    t.rootTranslation = {};
    Vec3& rootTranslation = t.rootTranslation;
    t.contactWeights = {};
    std::array<float, 2>& contactWeights = t.contactWeights;
    t.clipSampleTime = 0.0f;
    float& clipSampleTime = t.clipSampleTime;
    t.legSampleTime = {};
    std::array<float, 2>& legSampleTime = t.legSampleTime;
    t.armSampleTime = 0.0f;
    float& armSampleTime = t.armSampleTime;
}

} // namespace MatterEngine
