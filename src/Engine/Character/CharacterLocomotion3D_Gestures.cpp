#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepGestures(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const bool& getUpActive = t.getUpActive;
    CharacterLocomotionState3D& desiredState = t.desiredState;
    const Quaternion& heading = t.heading;
    const bool& gaitMoving = t.gaitMoving;
    const AnimationClip3D*& standUpClip = t.standUpClip;
    const bool& jumpPose = t.jumpPose;
    const bool& leapPose = t.leapPose;
    const bool& authoredStartActive = t.authoredStartActive;
    const float& authoredCurveBlend = t.authoredCurveBlend;
    const AnimationClip3D*& authoredCurveClip = t.authoredCurveClip;
    const AnimationClip3D*& selectedClip = t.selectedClip;
    std::vector<Vec3>& sampled = t.sampled;
    Quaternion& rootRotation = t.rootRotation;
    Vec3& rootTranslation = t.rootTranslation;
    std::array<float, 2>& contactWeights = t.contactWeights;
    float& clipSampleTime = t.clipSampleTime;
    std::array<float, 2>& legSampleTime = t.legSampleTime;
    float& armSampleTime = t.armSampleTime;
    const auto leapProgress = [](float phase) {
        if (RunLeapHold <= 0.0f) return phase;
        return RunLeapHold * smoothStep(phase / 0.3f)
            + (1.0f - RunLeapHold) * smoothStep((phase - 0.7f) / 0.3f);
    };
    // Variacao de movimento (vida), por cima do clipe e antes do crossfade:
    // a troca de estado a mistura como mistura o clipe.
    const float variety = std::clamp(input.motionVariety, 0.0f, 2.0f);
    const bool varietyActive = variety > 0.0f && selectedClip != nullptr
        && standUpClip == nullptr && !jumpPose && !getUpActive;
    // Parado, cada um respira o idle no seu ritmo e do seu ponto (sem
    // isso, varios personagens parados se mexiam em uníssono).
    const bool idleVariety = varietyActive && !gaitMoving
        && (desiredState == CharacterLocomotionState3D::Idle
            || desiredState == CharacterLocomotionState3D::Turning);
    if (m_landingResyncFoot >= 0) {
        if (gaitMoving && selectedClip != nullptr && selectedClip->durationSeconds > 0.0f && !jumpPose) {
            const AnimationScalarTrack3D& contact = m_landingResyncFoot == 0
                ? selectedClip->leftFootContact : selectedClip->rightFootContact;
            constexpr int Samples = 240;
            const float duration = selectedClip->durationSeconds;
            float touchdown = -1.0f;
            float previous = sampleAnimationScalarTrack3D(contact, duration * (Samples - 1) / Samples, duration, true);
            for (int i = 0; i < Samples; ++i) {
                const float now = sampleAnimationScalarTrack3D(contact, duration * i / Samples, duration, true);
                if (previous < 0.5f && now >= 0.5f) { touchdown = static_cast<float>(i) / Samples; break; }
                previous = now;
            }
            if (touchdown >= 0.0f) {
                m_cyclePhase = std::fmod(touchdown + 0.02f - stanceAlignedPhase(*selectedClip, 0.0f) + 2.0f, 1.0f);
                m_legPhaseLead = {};
            }
        }
        m_landingResyncFoot = -1;
    }
    if (m_motionSeedPending) {
        // Vindo do padrao (seedMotion): a fase em que os pes do clipe, em
        // relacao a pelve e no rumo dela, mais se parecem com os do corpo -
        // onde estao e para onde vao -, sem a arrancada; e a pose sai das
        // juntas do corpo com a velocidade delas.
        m_motionSeedPending = false;
        if (gaitMoving && selectedClip != nullptr && standUpClip == nullptr && !jumpPose
            && m_feet[0].valid && m_feet[1].valid && selectedClip->durationSeconds > 0.0f) {
            const AnimationClip3D& clip = *selectedClip;
            const auto clipFeet = [&](float fraction) {
                const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(profile, &clip,
                    std::fmod(fraction + 1.0f, 1.0f) * clip.durationSeconds, true);
                const Vec3 pelvisForward = (pose.linkOrientations.front() * profile.links.front().modelOrientation
                    .conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                const Quaternion back = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                    -std::atan2(pelvisForward.y, pelvisForward.x));
                std::array<Vec3, 2> out;
                for (std::size_t side = 0; side < 2; ++side)
                    out[side] = back.rotate(pose.linkPositions[m_feet[side].linkIndex] - pose.linkPositions.front());
                return out;
            };
            const float rate = std::clamp(m_seedSpeed / std::max(0.5f, clip.nominalSpeedMetersPerSecond), 0.5f, 2.0f)
                / clip.durationSeconds;
            constexpr int Samples = 48;
            constexpr float Step = 1.0f / 96.0f;
            constexpr float VelocityWeight = 0.08f;
            float best = 1e9f, bestFraction = 0.0f;
            for (int k = 0; k < Samples; ++k) {
                const float fraction = static_cast<float>(k) / static_cast<float>(Samples);
                const auto now = clipFeet(fraction);
                const auto next = clipFeet(fraction + Step);
                float error = 0.0f;
                for (std::size_t side = 0; side < 2; ++side)
                    error += (now[side] - m_seedFeet[side]).lengthSquared()
                        + ((next[side] - now[side]) * (rate / Step) - m_seedFeetVelocity[side]).lengthSquared()
                            * VelocityWeight * VelocityWeight;
                if (error < best) { best = error; bestFraction = fraction; }
            }
            m_cyclePhase = std::fmod(bestFraction - stanceAlignedPhase(clip, 0.0f) + 2.0f, 1.0f);
            m_startClipActive = false;
            m_startStepRequested = false;
        }
        if (m_seedJoints.size() == m_transitionFrom.size()) {
            // O crossfade sai das juntas do corpo, sem a tangente: ela estende
            // a velocidade das juntas por toda a troca (a 20 rad/s no sprint,
            // ~0,9 rad alem) e o sprint caia mais (5 em 8, contra 2). Pela
            // inercia (a da volta ao padrao) a inclinacao e o deslocamento da
            // raiz ficavam sem mistura (o alvo nascia a 18 graus do corpo no
            // trote de lado).
            m_transitionFrom = m_seedJoints;
            m_transitionVelocity.assign(m_transitionFrom.size(), {});
        }
    }
    if (selectedClip != nullptr) {
        const bool loopingState = desiredState == CharacterLocomotionState3D::Idle
            || desiredState == CharacterLocomotionState3D::Turning
            || desiredState == CharacterLocomotionState3D::Airborne;
        // O pulo e amostrado pela fase do voo; na aterrissagem fica na pose
        // de toque (o fim do clipe).
        const float sampleTime = standUpClip != nullptr
            ? std::min(m_getUpClipSeconds, selectedClip->durationSeconds)
            : leapPose
            ? std::fmod(m_leapFrom + m_leapSpan * (desiredState == CharacterLocomotionState3D::Landing
                ? 1.0f : leapProgress(m_flightPhase)), 1.0f) * selectedClip->durationSeconds
            : jumpPose
            ? (desiredState == CharacterLocomotionState3D::Landing ? 1.0f
                : m_flightPhase) * selectedClip->durationSeconds
            : gaitMoving
            ? stanceAlignedPhase(*selectedClip, m_cyclePhase)
                * selectedClip->durationSeconds
            : loopingState && idleVariety
                ? std::fmod(m_stateSeconds * (1.0f + 0.08f * varietyValue(
                    input.varietySeed, 5, 0)) + selectedClip->durationSeconds
                    * (0.5f + 0.5f * varietyValue(input.varietySeed, 6, 0)),
                    selectedClip->durationSeconds)
            : loopingState
                ? std::fmod(m_stateSeconds, selectedClip->durationSeconds)
                : std::min(m_stateSeconds, selectedClip->durationSeconds);
        clipSampleTime = sampleTime;
        // Cada perna no seu tempo (passada pelo contato: a de tras pode estar
        // adiantada enquanto a da frente espera o toque).
        armSampleTime = gaitMoving && standUpClip == nullptr && !jumpPose && m_armPhaseValid
            ? stanceAlignedPhase(*selectedClip, m_armPhase) * selectedClip->durationSeconds : sampleTime;
        for (std::size_t side = 0; side < 2; ++side) {
            const float cycle = std::max(0.05f, selectedClip->durationSeconds);
            if (!(gaitMoving && standUpClip == nullptr && !jumpPose)) {
                legSampleTime[side] = sampleTime;
            } else if (input.corrections == nullptr || !input.corrections->footwork) {
                legSampleTime[side] = std::fmod(sampleTime + m_legPhaseLead[side] * selectedClip->durationSeconds, cycle);
            } else {
                // O gancho do ritmo de cada perna (o RL): o avanco ou atraso
                // proprio dela, somado ao do contato (a perna que espera o
                // toque so adianta).
                const float own = legPhaseLead3D(input.corrections, side, m_waitingLeg == static_cast<int>(side));
                // (Sem avanco, a conta do classico exata: o ciclo somado mudava
                // o arredondamento e o sinal - a acao zero precisa ser o classico.)
                legSampleTime[side] = own == 0.0f
                    ? std::fmod(sampleTime + m_legPhaseLead[side] * selectedClip->durationSeconds, cycle)
                    : std::fmod(sampleTime + (m_legPhaseLead[side] + own) * selectedClip->durationSeconds + cycle, cycle);
            }
        }
        for (std::size_t side = 0; side < 2; ++side)
            m_telemetry.legPhase[side] = legSampleTime[side] / std::max(0.05f, selectedClip->durationSeconds);
        if (m_legSideOfLink.size() != profile.links.size()) {
            m_legSideOfLink.assign(profile.links.size(), -1);
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const std::string& id = profile.links[index].id;
                const bool legLink = id.find("Thigh") != std::string::npos
                    || id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos;
                if (legLink) m_legSideOfLink[index] = id.rfind("Left", 0) == 0 ? 0 : id.rfind("Right", 0) == 0 ? 1 : -1;
                else if (id.find("Arm") != std::string::npos || id.find("Forearm") != std::string::npos
                    || id.find("Hand") != std::string::npos) m_legSideOfLink[index] = -2;
            }
        }
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            const AnimationTrack3D* track = findAnimationTrack3D(
                *selectedClip, profile.links[index].id);
            if (track == nullptr) continue;
            const int legSide = m_legSideOfLink[index];
            sampled[index] = sampleAnimationTrack3D(*track,
                legSide >= 0 ? legSampleTime[static_cast<std::size_t>(legSide)]
                    : legSide == -2 ? armSampleTime : sampleTime,
                selectedClip->durationSeconds, selectedClip->loops)
                    .jointPositionRadians;
        }
        const AnimationTrack3D* rootTrack = findAnimationTrack3D(
            *selectedClip, profile.links.front().id);
        if (rootTrack != nullptr) {
            const AnimationTransformSample3D rootSample =
                sampleAnimationTrack3D(*rootTrack, sampleTime,
                    selectedClip->durationSeconds, selectedClip->loops);
            rootTranslation = rootSample.translationOffsetMeters;
            rootRotation = rootSample.rotationDelta;
        }
        contactWeights[0] = sampleAnimationScalarTrack3D(
            selectedClip->leftFootContact, legSampleTime[0],
            selectedClip->durationSeconds, selectedClip->loops);
        contactWeights[1] = sampleAnimationScalarTrack3D(
            selectedClip->rightFootContact, legSampleTime[1],
            selectedClip->durationSeconds, selectedClip->loops);
    }
    // Pulo parado: no chao, segurando o pulo, o preparo - o agachamento da
    // referencia (bracos para tras, tronco a frente), mais fundo com a carga.
    {
        const auto blendClip = [&](const AnimationClip3D& clip, float time, float weight, float trunkShare) {
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[index].id);
                if (track == nullptr) continue;
                const Vec3 pose = sampleAnimationTrack3D(*track, time, clip.durationSeconds, false).jointPositionRadians;
                const std::string& id = profile.links[index].id;
                const bool trunk = id == "Abdomen" || id == "Chest" || id == "UpperChest" || id == "Neck" || id == "Head";
                sampled[index] += (pose - sampled[index]) * weight * (trunk ? trunkShare : 1.0f);
            }
            if (const AnimationTrack3D* root = findAnimationTrack3D(clip, profile.links.front().id)) {
                const Quaternion tilt = sampleAnimationTrack3D(*root, time, clip.durationSeconds, false).rotationDelta;
                rootRotation = slerpQuaternion(rootRotation, tilt, weight).normalized();
            }
        };
        // Giro parado pelo clipe: as pernas sao as do clipe (o tronco fica com
        // a pose parada e o olhar).
        if (m_clipTurnWeight > 0.001f && m_clipTurn.clip != nullptr && !gaitMoving && !jumpPose
                && !getUpActive && standUpClip == nullptr) {
            const AnimationClip3D& clip = *m_clipTurn.clip;
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const std::string& id = profile.links[index].id;
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (!leg) continue;
                const AnimationTrack3D* track = findAnimationTrack3D(clip, id);
                if (track == nullptr) continue;
                const Vec3 pose = sampleAnimationTrack3D(*track, m_clipTurn.time, clip.durationSeconds, false)
                    .jointPositionRadians;
                sampled[index] += (pose - sampled[index]) * m_clipTurnWeight;
            }
        }
        if (!jumpPose && !getUpActive && standUpClip == nullptr && m_jumpCrouch > 0.001f
                && animations.jumpPrep != nullptr) {
            const float weight = smoothStep(m_jumpCrouch / 0.15f);
            const float depth = std::clamp(m_jumpCrouch, 0.0f, 1.0f);
            // O tronco so com parte da flexao da referencia: o impulso sai
            // pela pelve, e com o tronco dobrado 50 graus a frente (o centro de
            // massa a frente dela) ele saia girando de costas e caia deitado.
            blendClip(*animations.jumpPrep, depth * animations.jumpPrep->durationSeconds, weight, JumpPrepTrunkShare);
        }
    }
    // A arrancada capturada participa como DELTA de gesto, por cima do ciclo
    // de trote/sprint que continua dono das pernas, dos contatos e da raiz.
    // Copiar a pose absoluta do FBX trazia para o guia 33 graus de root pitch,
    // 10 cm de agachamento e uma fase de voo antes de o corpo acelerar.
    if (authoredStartActive && animations.idleToSprint != nullptr) {
        const float duration = m_startClipSprint ? 0.62f : 0.34f;
        const float u = std::clamp(m_startClipSeconds / duration, 0.0f, 1.0f);
        const float envelope = smoothStep(u / 0.20f)
            * (1.0f - smoothStep((u - 0.62f) / 0.38f));
        const float sourceTime = std::min(
            m_startClipSeconds * (m_startClipSprint ? 1.0f : 0.75f),
            animations.idleToSprint->durationSeconds);
        const auto segmentWeight = [&](const std::string& id) {
            const bool spine = id == "Abdomen" || id == "Chest"
                || id == "UpperChest";
            const bool arm = id == "LeftUpperArm" || id == "RightUpperArm"
                || id == "LeftForearm" || id == "RightForearm";
            const bool head = id == "Neck" || id == "Head";
            if (spine) return m_startClipSprint ? 0.55f : 0.28f;
            if (arm) return m_startClipSprint ? 0.62f : 0.30f;
            if (head) return m_startClipSprint ? 0.25f : 0.12f;
            return 0.0f;
        };
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            const float weight = segmentWeight(profile.links[index].id)
                * envelope;
            if (weight <= 0.0f) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(
                *animations.idleToSprint, profile.links[index].id);
            if (track == nullptr) continue;
            const Vec3 first = sampleAnimationTrack3D(*track, 0.0f,
                animations.idleToSprint->durationSeconds, false)
                    .jointPositionRadians;
            const Vec3 current = sampleAnimationTrack3D(*track, sourceTime,
                animations.idleToSprint->durationSeconds, false)
                    .jointPositionRadians;
            const float limit = m_startClipSprint ? 0.42f : 0.20f;
            const Vec3 delta {
                std::clamp(current.x - first.x, -limit, limit),
                std::clamp(current.y - first.y, -limit, limit),
                std::clamp(current.z - first.z, -limit, limit)
            };
            sampled[index] += delta * weight;
        }
    }
    // A curva real entra apenas nos segmentos que definem apoio e banco do
    // corpo. Cabeca e bracos ficam no ciclo normal; isso tambem exclui
    // explicitamente os canais defeituosos da referencia Arc Left.
    if (authoredCurveClip != nullptr && authoredCurveBlend > 0.0f) {
        const float curveTime = stanceAlignedPhase(*authoredCurveClip,
            m_cyclePhase) * authoredCurveClip->durationSeconds;
        const auto curveSegment = [](const std::string& id) {
            return id == "Abdomen" || id == "Chest" || id == "UpperChest"
                || id == "LeftThigh" || id == "LeftShin" || id == "LeftFoot"
                || id == "RightThigh" || id == "RightShin" || id == "RightFoot";
        };
        for (std::size_t index = 1; index < profile.links.size(); ++index) {
            if (!curveSegment(profile.links[index].id)) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(
                *authoredCurveClip, profile.links[index].id);
            if (track == nullptr) continue;
            const Vec3 curvePose = sampleAnimationTrack3D(*track, curveTime,
                authoredCurveClip->durationSeconds, false).jointPositionRadians;
            const bool spine = profile.links[index].id == "Abdomen"
                || profile.links[index].id == "Chest"
                || profile.links[index].id == "UpperChest";
            // A coluna pode ler a captura com peso alto. Nas pernas ela e
            // uma referencia de estilo menor: o alvo de pouso em arco e o
            // contato fisico continuam soberanos, evitando chicotear o pe ao
            // sincronizar dois ciclos gravados em fases diferentes.
            const float weight = authoredCurveBlend * (spine ? 1.0f : 0.15f);
            Vec3 wanted = curvePose;
            if (spine) {
                // A captura da curva esquerda traz ~30 graus de corcunda como
                // baseline. Curva deve fornecer banco/torcao; a flexao
                // sagital continua sendo a do trote ou sprint selecionado.
                wanted.y = sampled[index].y;
            }
            sampled[index] += (wanted - sampled[index]) * weight;
        }
    }
    if (varietyActive && gaitMoving) {
        applyGaitVariety(profile, *selectedClip, m_cyclePhase,
            m_armPhaseValid ? m_armPhase : m_cyclePhase,
            m_varietyStrides, input.varietySeed, variety, sampled);
    } else if (idleVariety) {
        applyIdleVariety(profile, m_elapsedSeconds, input.varietySeed,
            variety, sampled);
    }
    // TEMPTUNE CurveSpine: na curva o tronco inclina para dentro pela coluna
    // (a parte da inclinacao que a pelve nao rola; depois da variedade, que
    // reamostra o tronco do clipe). Rolando a pelve, o
    // quadril de fora subia e a perna de fora nao alcancava o chao: a de
    // dentro fazia 70% dos apoios, galopando - os saltinhos da curva.
    {
        const float wanted = gaitMoving ? CurveSpineShare * m_curveSprintShare * m_bodyCurveLean : 0.0f;
        m_spineCurveRoll += (wanted - m_spineCurveRoll) * (1.0f - std::exp(-deltaTime / 0.08f));
        if (std::abs(m_spineCurveRoll) > 1e-4f) {
            for (std::size_t index = 1; index < profile.links.size(); ++index) {
                const std::string& id = profile.links[index].id;
                if (id == "Abdomen" || id == "Chest" || id == "UpperChest")
                    sampled[index].z += CurveSpineSign * m_spineCurveRoll / 3.0f;
            }
        }
    }
    // Fim do levantar: coluna, pescoco e bracos vao aos poucos para a
    // postura parada (o primeiro quadro do idle, que e onde ele comeca). O
    // levantar de bruços termina ereto demais (tronco a ~2 graus, pes
    // juntos) e o idle tem o tronco a ~25 graus para a frente: depois da
    // troca o corpo ficava torto "encaixando" a diferenca. As pernas seguem
    // o clipe (os pes ficam onde estao e os passos refazem a base).
    if (standUpClip != nullptr && !m_tackling && animations.idle != nullptr) {
        const float duration = standUpClip->durationSeconds > 0.05f
            ? standUpClip->durationSeconds : 1.6f;
        const float toStance = smoothStep(
            (m_getUpClipSeconds / duration - 0.60f) / 0.40f);
        if (toStance > 0.0f) {
            for (std::size_t index = 1; index < profile.links.size()
                    && index < sampled.size(); ++index) {
                const std::string& id = profile.links[index].id;
                if (id.find("Thigh") != std::string::npos
                    || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) continue;
                const AnimationTrack3D* track =
                    findAnimationTrack3D(*animations.idle, id);
                if (track == nullptr) continue;
                const Vec3 stance = sampleAnimationTrack3D(*track, 0.0f,
                    animations.idle->durationSeconds, animations.idle->loops)
                        .jointPositionRadians;
                sampled[index] += (stance - sampled[index]) * toStance;
            }
        }
    }

    // Preparo do carrinho: armado, tronco, bracos e cabeca ja vao para a
    // pose do comeco do clipe escolhido - no pouso a troca e pequena e o
    // mergulho sai da passada, fluido. As pernas seguem a passada.
    if (m_tacklePending && m_tackleClip != nullptr && standUpClip == nullptr) {
        const float w = TacklePreBlend * smoothStep(m_tacklePendingSeconds / 0.12f);
        for (std::size_t index = 1; index < profile.links.size() && index < sampled.size(); ++index) {
            const std::string& id = profile.links[index].id;
            if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                || id.find("Foot") != std::string::npos) continue;
            const AnimationTrack3D* track = findAnimationTrack3D(*m_tackleClip, id);
            if (track == nullptr) continue;
            const Vec3 first = sampleAnimationTrack3D(*track, 0.0f, m_tackleClip->durationSeconds, false)
                .jointPositionRadians;
            sampled[index] += (first - sampled[index]) * w;
        }
    }

    // Quanto a pelve desce no pouso (modo antigo). No modo fisico o
    // agachamento vai direto na pelve alvo, mais abaixo.
    const auto landingSink = [&](float settle) {
        return input.forceDrivenRoot ? 0.0f : 0.08f * std::sin(Pi * settle);
    };
    t.landingBraceForward = 0.0f;
    float& landingBraceForward = t.landingBraceForward;
    t.landingBraceLateral = 0.0f;
    float& landingBraceLateral = t.landingBraceLateral;
    // Pulo autoral: o voo vem do clipe (acima). Na aterrissagem os pes sao
    // plantados pela trava de pe (contato forcado) e a pelve desce: e o IK
    // que dobra os joelhos, absorvendo, em vez de posicoes fixas de junta.
    // Sem clipes de pulo, o voo e a pose procedural de antes.
    if (jumpPose && desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / m_landingSettleSeconds, 0.0f, 1.0f);
        rootTranslation.z -= landingSink(settle);
        contactWeights = { 1.0f, 1.0f };
    } else if (jumpPose) {
        // Voo: nada a acrescentar.
    } else if (desiredState == CharacterLocomotionState3D::Airborne) {
        const float verticalVelocity = input.rootVelocityWorld.z;
        const float riseTuck = smoothStep(m_stateSeconds / 0.14f);
        // As descent speed builds, ease the tuck back open to meet the
        // ground with the legs already reaching for it.
        const float fallRelease = std::clamp(1.0f
            - std::max(0.0f, -verticalVelocity) * 0.10f, 0.30f, 1.0f);
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, riseTuck * fallRelease,
            runningShare);
    } else if (desiredState == CharacterLocomotionState3D::Landing) {
        const float settle = std::clamp(
            m_stateSeconds / m_landingSettleSeconds, 0.0f, 1.0f);
        const float compression = (1.0f - smoothStep(settle)) * 0.55f;
        const float runningShare = smoothStep(m_liftoffSpeed / 2.2f);
        applyFlightPose(profile, sampled, compression, runningShare);
    } else if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
        // Caido, em tres tempos. Primeiro ragdoll de verdade: o alvo das
        // juntas segue o proprio corpo com atraso (~0,2 s) e tonus baixo - ele
        // cede para onde a queda o leva, mas os musculos amortecem (nao e
        // corpo morto). Parado de vez, fica largado um instante. Depois, com o
        // tonus voltando, se arruma aos poucos ate a pose inicial do levantar
        // certo (de costas ou de bruços), de onde o levantar comeca sem salto.
        // Antes a pose de levantar era o alvo desde o primeiro instante: ele
        // caia ja "arrumado" para levantar.
        const AnimationClip3D* lying = m_fallOrientation
                == CharacterFallOrientation3D::FaceDown
            ? animations.standUpFront : animations.standUpBack;
        if (!m_settlePoseValid || m_settlePose.size() != sampled.size()) {
            m_settlePose.assign(sampled.size(), Vec3 {});
            m_settleGather = 0.0f;
            for (std::size_t link = 1;
                    link < sampled.size() && link < state.joints.size(); ++link) {
                m_settlePose[link] = { state.joints[link].positionRadians[0],
                    state.joints[link].positionRadians[1],
                    state.joints[link].positionRadians[2] };
            }
            m_settlePoseValid = true;
        }
        const bool gathering = m_settleRestSeconds > (m_getUpAttempt > 1
            ? RetryRestBeforeGather : SettleRestBeforeGather);
        // Atraso curto: o alvo que persegue o corpo atua como freio (mola x
        // atraso); com 0,2 s um antebraco em pe caia a ~0,1 rad/s.
        const float follow = gathering ? 0.0f
            : 1.0f - std::exp(-deltaTime / 0.04f);
        // Arrumando-se, acompanha o tonus; interrompido (um tranco no
        // tronco), volta a ceder em ~0,15 s, sem salto no alvo.
        const float gatherTarget = gathering ? smoothStep((m_settleTone
            - SettleRestingTone) / (1.0f - SettleRestingTone)) : 0.0f;
        m_settleGather = gatherTarget >= m_settleGather ? gatherTarget
            : m_settleGather + (gatherTarget - m_settleGather)
                * (1.0f - std::exp(-deltaTime / 0.15f));
        const float gather = m_settleGather;
        for (std::size_t link = 1;
                link < sampled.size() && link < state.joints.size(); ++link) {
            const Vec3 measured { state.joints[link].positionRadians[0],
                state.joints[link].positionRadians[1],
                state.joints[link].positionRadians[2] };
            m_settlePose[link] += (measured - m_settlePose[link]) * follow;
            // (Pelo corpo, deitado ele so recupera o tonus onde esta: quem
            // arruma a pose e o levantar.)
            const AnimationTrack3D* track = lying != nullptr && !GetUpBodily
                ? findAnimationTrack3D(*lying, profile.links[link].id) : nullptr;
            const Vec3 start = track != nullptr
                ? sampleAnimationTrack3D(*track, 0.0f, lying->durationSeconds,
                    false).jointPositionRadians
                : measured;
            sampled[link] = m_settlePose[link]
                + (start - m_settlePose[link]) * gather;
        }
        // Rolando (TEMPTUNE GetUpRoll): sobre a pose deitada, a perna de
        // cima dobra e cruza para o lado da barriga e o braco de cima alcanca
        // o chao por cima do peito - o peso vai para a frente e o corpo vira.
        // (No referencial da pelve: +x barriga, +y esquerda, +z cabeca.)
        if (m_getUpRolling && m_getUpRollTop >= 0) {
            const float weight = smoothStep(m_getUpRollSeconds / 0.4f);
            const int top = m_getUpRollTop;
            const float across = top == 0 ? -1.0f : 1.0f;  // para o outro lado
            const char* thighId = top == 0 ? "LeftThigh" : "RightThigh";
            const char* shinId = top == 0 ? "LeftShin" : "RightShin";
            const char* upperId = top == 0 ? "LeftUpperArm" : "RightUpperArm";
            const char* foreId = top == 0 ? "LeftForearm" : "RightForearm";
            const char* handId = top == 0 ? "LeftHand" : "RightHand";
            const std::size_t thigh = findLink(profile, thighId), shin = findLink(profile, shinId);
            const std::size_t upper = findLink(profile, upperId), fore = findLink(profile, foreId);
            const std::size_t hand = findLink(profile, handId);
            const auto anchorIn = [&](std::size_t child) {
                const RagdollLinkDefinition3D& link = profile.links[child];
                const RagdollLinkDefinition3D& parent = profile.links[static_cast<std::size_t>(link.parentIndex)];
                return parent.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - parent.modelPosition);
            };
            const std::size_t n = profile.links.size();
            // (findLink devolve size() quando nao acha: testar os dois lados.)
            if (thigh > 0 && shin > 0 && upper > 0 && fore > 0 && hand > 0
                && thigh < n && shin < n && upper < n && fore < n && hand < n) {
                std::vector<Vec3> roll = sampled;
                RagdollAnimationPose3D frame;
                buildRagdollLocalPose3D(profile, roll, frame);
                // Joelho de cima a frente e para o outro lado.
                const Vec3 hip = frame.linkPositions[thigh]
                    + frame.linkOrientations[thigh].rotate(profile.links[thigh].modelOrientation.conjugate().rotate(
                        profile.links[thigh].inboundJoint.anchorModelPosition - profile.links[thigh].modelPosition));
                const Vec3 knee = hip + Vec3 { 0.28f, across * 0.12f, -0.27f };
                solveRagdollChainPoint3D(profile, roll, thigh, anchorIn(shin), 1, knee, 1.0f, 10);
                setComponent(roll[shin], 0, std::clamp(1.3f,
                    profile.links[shin].inboundJoint.axes[0].minimumRadians,
                    profile.links[shin].inboundJoint.axes[0].maximumRadians));
                // Mao de cima no chao, a frente do peito, do outro lado.
                buildRagdollLocalPose3D(profile, roll, frame);
                const Vec3 shoulder = frame.linkPositions[upper];
                const Vec3 reach = shoulder + Vec3 { 0.40f, across * 0.30f, -0.05f };
                solveRagdollChainPoint3D(profile, roll, fore, anchorIn(hand), 2, reach, 1.0f, 10);
                for (const std::size_t link : { thigh, shin, upper, fore })
                    sampled[link] += (roll[link] - sampled[link]) * weight;
            }
        }
    }

    // Preparacao de impacto pela velocidade real. Durante a descida o peito
    // vai contra o movimento horizontal: aterrissando para a frente, fica
    // atras da pelve; para a esquerda, desloca-se para a direita. Assim a
    // resultante do contato atravessa melhor a base e os joelhos podem
    // comprimir sem o tronco continuar tombando por inercia.
    if (!getUpActive && (desiredState == CharacterLocomotionState3D::Airborne
            || desiredState == CharacterLocomotionState3D::Landing)) {
        const Vec3 bodyVelocity = input.forceDrivenRoot && !state.links.empty()
            ? state.links.front().linearVelocity : input.rootVelocityWorld;
        const Vec3 localImpact = heading.conjugate().rotate(
            {bodyVelocity.x, bodyVelocity.y, 0.0f});
        const float horizontalSpeed = std::hypot(localImpact.x, localImpact.y);
        const float fallSpeed = desiredState == CharacterLocomotionState3D::Airborne
            ? std::max(0.0f, -bodyVelocity.z) : m_landingImpactSpeed;
        const float descent = desiredState == CharacterLocomotionState3D::Airborne
            ? smoothStep((m_flightPhase - 0.48f) / 0.34f)
                * smoothStep((fallSpeed - 0.35f) / 1.8f)
            : 1.0f - smoothStep(m_stateSeconds
                / std::max(0.08f, m_landingSettleSeconds));
        const float brace = std::clamp(horizontalSpeed
            * (0.025f + 0.007f * std::min(fallSpeed, 5.0f)), 0.0f, 0.30f)
            * descent;
        if (horizontalSpeed > 0.05f) {
            landingBraceForward = -brace * localImpact.x / horizontalSpeed;
            landingBraceLateral = -brace * localImpact.y / horizontalSpeed;
            addJointCoordinate(profile, sampled, "Abdomen", 1,
                landingBraceForward * 0.30f);
            addJointCoordinate(profile, sampled, "Chest", 1,
                landingBraceForward * 0.35f);
            addJointCoordinate(profile, sampled, "UpperChest", 1,
                landingBraceForward * 0.35f);
            addJointCoordinate(profile, sampled, "Abdomen", 2,
                landingBraceLateral * 0.30f);
            addJointCoordinate(profile, sampled, "Chest", 2,
                landingBraceLateral * 0.35f);
            addJointCoordinate(profile, sampled, "UpperChest", 2,
                landingBraceLateral * 0.35f);
        }
    }

    // Fim do levantar (pedido do usuario, 05/10 noite): os clipes terminam
    // com a coluna curvada (de bruços, ~28 graus somando abdomen, peito e
    // peito alto, a pose de alerta) e as pernas ja esticadas - de pe e
    // dobrado, ate a postura de pe entrar so no fim, de uma vez. Ja de pe
    // (a pelve alta e o clipe quase reto), a coluna vai a GetUpEndSpineShare
    // da flexao do clipe e o pescoco e a cabeca a extensao da postura de pe.
    if (m_getUpPhase == CharacterGetUpPhase3D::Rising && !m_tackling && !state.links.empty()) {
        const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
        const float height = state.links.front().position.z - ground;
        const float upright = smoothStep((0.8f - m_getUpExpectedTilt) / 0.4f)
            * smoothStep((height - 0.55f) / 0.25f) * GetUpEndStraighten;
        if (upright > 0.0f) {
            for (const char* id : { "Abdomen", "Chest", "UpperChest" }) {
                const std::size_t index = findLink(profile, id);
                if (index == 0 || index >= sampled.size()) continue;
                sampled[index].y *= 1.0f - (1.0f - GetUpEndSpineShare) * upright;
            }
            addJointCoordinate(profile, sampled, "Neck", 1, -IdleNeckExtension * upright);
            addJointCoordinate(profile, sampled, "Head", 1, -IdleHeadExtension * upright);
        }
    }
    t.transition = smoothStep(
        m_transitionSeconds / m_transitionDurationSeconds);
    const float& transition = t.transition;
    const float transitionTime = std::clamp(
        m_transitionSeconds / m_transitionDurationSeconds, 0.0f, 1.0f);
    // Hermite outgoing tangent: preserve the motion at interruption and
    // release it continuously as the new clip takes over. Only authored
    // coordinates participate; IK and look must not feed back into it.
    t.tangentWeight = m_transitionDurationSeconds * transitionTime
        * (1.0f - transitionTime) * (1.0f - transitionTime);
    const float& tangentWeight = t.tangentWeight;
    // Levantar a partir de onde o corpo esta: nos primeiros 0,35 s os alvos
    // saem das juntas medidas ao comecar, nao de onde a pose de se arrumar
    // queria que estivessem.
    const float risingFromMeasured = m_getUpPhase == CharacterGetUpPhase3D::Rising
            && m_risingFromMeasured.size() == profile.links.size()
        ? 1.0f - smoothStep(m_getUpPhaseSeconds / 0.35f) : 0.0f;
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        Vec3 wanted = m_transitionFrom[index] * (1.0f - transition)
            + sampled[index] * transition
            + m_transitionVelocity[index] * tangentWeight;
        if (risingFromMeasured > 0.0f)
            wanted += (m_risingFromMeasured[index] - wanted) * risingFromMeasured;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit =
                profile.links[index].inboundJoint.axes[axis];
            setComponent(wanted, axis, limit.enabled
                ? std::clamp(component(wanted, axis),
                    limit.minimumRadians, limit.maximumRadians)
                : 0.0f);
        }
        m_coordinates[index] = wanted;
        m_poseVelocities[index] = m_poseCoordinates.size() == m_coordinates.size()
            ? clampMagnitude((wanted - m_poseCoordinates[index]) / deltaTime, 30.0f)
            : Vec3 {};
    }
    m_poseCoordinates = m_coordinates;

}

} // namespace MatterEngine
