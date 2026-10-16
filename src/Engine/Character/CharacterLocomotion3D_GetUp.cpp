#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepGetUp(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const bool& PostureSupportPendulum = t.PostureSupportPendulum;
    bool& anyGroundContact = t.anyGroundContact;
    std::array<bool, 2>& feetGroundContact = t.feetGroundContact;
    float& contactFloor = t.contactFloor;
    float& rootUpright = t.rootUpright;
    const float& groundHeightGuess = t.groundHeightGuess;
    const float& measuredComHeight = t.measuredComHeight;
    const bool& physicsOwnsBody = t.physicsOwnsBody;
    const float& fallenUpright = t.fallenUpright;
    const bool& fallenObservation = t.fallenObservation;
    const auto enterGetUpPhase = [&](CharacterGetUpPhase3D phase) {
        m_getUpPhase = phase;
        m_getUpPhaseSeconds = 0.0f;
        m_getUpFootwork = false;
        m_getUpStepSide = -1;
        m_getUpBalanceHold = false;
        m_getUpBalanceHoldSeconds = 0.0f;
        m_getUpBalanceSteps = 0;
        if (phase == CharacterGetUpPhase3D::Settling) {
            m_getUpRolling = false;
            m_getUpRollDone = false;
            m_getUpRollSeconds = 0.0f;
        }
    };
    const auto classifyFallOrientation = [&]() {
        const std::size_t reference = m_chestLink < state.links.size()
            ? m_chestLink : 0;
        const Vec3 bodyForward = state.links[reference]
            .orientation.rotate({ 1.0f, 0.0f, 0.0f });
        m_fallOrientation = bodyForward.z < 0.0f
            ? CharacterFallOrientation3D::FaceDown
            : CharacterFallOrientation3D::FaceUp;
    };
    // Pressa para levantar (as teclas de andar; com Shift, correndo): ver
    // GetUpSpeed.
    const bool getUpKeys = input.intent.requestedVelocityWorld.x * input.intent.requestedVelocityWorld.x
        + input.intent.requestedVelocityWorld.y * input.intent.requestedVelocityWorld.y > 0.01f;
    // (O Shift pela intencao: caido, a capsula esta parada e input.sprinting
    // - que so liga com ela andando - nao o via.)
    // Sem pressa, o levantar no ritmo dele, detalhado (pedido do usuario,
    // 05/10 noite); com as teclas de andar ou o espaco, rapido; andando com
    // Shift, correndo.
    const int getUpHurry = !GetUpHurryEnabled || (!getUpKeys && !input.getUpHurry) ? 0
        : getUpKeys && (input.sprinting || input.intent.effort > 0.5f) ? 2 : 1;
    const float getUpSpeed = getUpHurry == 2 ? GetUpSprintSpeed : getUpHurry == 1 ? GetUpHurrySpeed : GetUpSpeed;
    m_getUpSpeedNow = getUpSpeed;
    if (m_getUpPhase != CharacterGetUpPhase3D::Rising) m_getUpHandoffReady = false;
    if (m_getUpPhase == CharacterGetUpPhase3D::None) {
        if (fallenObservation && std::getenv("XQUEDADBG"))
            std::printf("QUEDA t %.2f upright %.2f (limite %.2f) com %.2f chao %.2f contato %d\n", m_elapsedSeconds,
                rootUpright, fallenUpright, measuredComHeight, contactFloor, anyGroundContact ? 1 : 0);
        if (fallenObservation) {
            m_getUpHurriedExit = false;
            classifyFallOrientation();
            enterGetUpPhase(CharacterGetUpPhase3D::Settling);
            ++m_getUpAttempt;
            m_recoveredStandingSeconds = 0.0f;
            m_settleTone = SettleImpactTone;
            m_settleRestSeconds = 0.0f;
            m_settlePoseValid = false;
            for (FootPlant& foot : m_feet) {
                foot.planted = false;
                foot.repositioning = false;
                foot.recoveryStep = false;
            }
        }
    } else if (!input.manipulated) {
        // Being held by the PhysGun freezes recovery instead of fighting the
        // player back toward a stand-up target.
        m_getUpPhaseSeconds += deltaTime;
        if (m_tackling) {
            // Carrinho: o clipe anda no tempo (o mergulho e o deslize sao o
            // embalo, nao esperam o corpo), no ritmo da entrada.
            const AnimationClip3D* clip = m_tackleClip;
            const float duration = clip != nullptr ? clip->durationSeconds : 1.0f;
            const float height = state.links.front().position.z - groundHeightGuess;
            // Deslizando (a pelve do clipe e a do corpo baixas, o clipe sem
            // subir ainda), a pose do deslize fica enquanto o corpo anda: o
            // comprimento do deslize e o do embalo e do atrito, nao o do
            // clipe. Devagar, o clipe segue para o levantar.
            const float bodySpeed = std::hypot(state.links.front().linearVelocity.x,
                state.links.front().linearVelocity.y);
            const bool sliding = m_tackleExpectedHeight < TackleSlideHeight
                && height < TackleSlideHeight + 0.10f && m_tackleExpectedRise < 0.4f;
            const float slideRate = sliding ? std::clamp(1.0f - (bodySpeed - TackleSlideRelease) / 1.5f,
                0.12f, 1.0f) : 1.0f;
            // Descendo, o clipe espera o corpo: o ator se joga ao chao mais
            // rapido que a queda livre (0,6 m em 0,15 s; caindo, 0,35 s) - o
            // clipe chegava a pose de deslize com o corpo ainda no alto, as
            // pernas iam a frente no ar e, quando ele tocava o chao, o clipe
            // ja estava levantando (quase nao deslizava).
            const float dropRate = m_tackleExpectedRise < 0.0f
                ? std::clamp(1.0f - (height - m_tackleExpectedHeight - 0.10f) / 0.20f, 0.20f, 1.0f) : 1.0f;
            const float paceShare = std::min(slideRate, dropRate);
            m_getUpClipSeconds = std::min(duration, m_getUpClipSeconds + deltaTime * m_tackleRate * paceShare);
            m_getUpRate = m_tackleRate * paceShare;
            const float expected = clip != nullptr ? profile.standingRootHeightMeters
                + clipRootAt(profile, *clip, m_getUpClipSeconds).translationOffsetMeters.z
                : profile.standingRootHeightMeters;
            // A altura da pelve no clipe e quanto ela sobe (m/s, no ritmo):
            // os musculos e a ajuda leem as fases (descendo, subindo).
            if (clip != nullptr) {
                const float ahead = std::min(duration, m_getUpClipSeconds + 1.0f / 30.0f);
                const float rise = profile.standingRootHeightMeters
                    + clipRootAt(profile, *clip, ahead).translationOffsetMeters.z - expected;
                m_tackleExpectedRise = ahead > m_getUpClipSeconds
                    ? rise * 30.0f * m_tackleRate : 0.0f;
            }
            m_tackleExpectedHeight = expected;
            // Na volta (da metade do clipe em diante) a pelve muito abaixo de
            // onde o clipe ja subiu, ou o fim sem estar de pe: nao deu - vira
            // queda e ele levanta pelo levantar normal.
            const bool recovering = m_getUpClipSeconds > 0.5f * duration;
            m_tackleBehindSeconds = recovering && height < expected - 0.40f
                ? m_tackleBehindSeconds + deltaTime
                : std::max(0.0f, m_tackleBehindSeconds - deltaTime);
            // De pe o bastante para a locomocao assumir: ela segura o peso e
            // termina de endireitar (no carrinho a ajuda de postura dela fica
            // suspensa - exigindo o corpo ereto, ele ficava meio agachado e
            // inclinado ~40 graus, sem forca para subir, ate virar queda).
            // (Ele termina de se endireitar ainda no carrinho - a ajuda de
            // levantar dele vale ate o corpo chegar a altura do clipe;
            // demorando, aceita mais inclinado em vez de virar queda.)
            const bool upright = (rootUpright > 0.82f
                    && height > profile.standingRootHeightMeters * 0.70f)
                || (m_tackleHoldSeconds > 0.4f && rootUpright > 0.65f
                    && height > profile.standingRootHeightMeters * 0.60f);
            // De volta em pe no clipe (passou do deslize e a pelve do clipe ja
            // na altura de pe) e no corpo: a corrida volta a ser da
            // locomocao - os passos e o equilibrio dela seguram a saida
            // correndo; presa ao fim do clipe, a 7 m/s o corpo pendia ~45
            // graus para a frente.
            // (Logo que ele levanta do deslize, antes da passada de saida do
            // clipe: ela nao se adapta - o pe pousava atras do centro de massa
            // e ele tombava para a frente. A locomocao poe o pe pelo ponto de
            // captura.)
            const bool clipStanding = m_getUpClipSeconds > 0.6f * duration
                && m_tackleExpectedHeight > profile.standingRootHeightMeters * TackleHandoffHeight;
            if (m_getUpClipSeconds >= duration && !upright) m_tackleHoldSeconds += deltaTime;
            if (m_tackleBehindSeconds > 1.0f || m_tackleHoldSeconds > 1.0f) {
                m_tackling = false;
                m_tackleClip = nullptr;
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_recoveredStandingSeconds = 0.0f;
                m_settleTone = SettleImpactTone;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                for (FootPlant& foot : m_feet) {
                    foot.planted = false;
                    foot.repositioning = false;
                    foot.recoveryStep = false;
                }
            } else if ((m_getUpClipSeconds >= duration || clipStanding) && upright) {
                m_tackling = false;
                m_tackleClip = nullptr;
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_getUpExpectedValid = false;
                m_tackleCooldown = 0.35f;
                // Segue para onde o corpo terminou virado; os pes ficam onde
                // estao (os passos refazem a base), o olhar volta aos poucos.
                const Vec3 forward = state.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                    ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw : m_tackleYaw;
                m_turningRate = 0.0f;
                m_plantFeetFromBody = true;
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Settling) {
            // Deitado: se rolou, a pose deitada (e o levantar) troca - com
            // folga, para nao ficar trocando de lado.
            if (m_chestLink < state.links.size()) {
                const float facing = state.links[m_chestLink].orientation
                    .rotate({ 1.0f, 0.0f, 0.0f }).z;
                if (m_fallOrientation == CharacterFallOrientation3D::FaceUp
                    && facing < -0.25f) {
                    m_fallOrientation = CharacterFallOrientation3D::FaceDown;
                } else if (m_fallOrientation
                        == CharacterFallOrientation3D::FaceDown
                    && facing > 0.25f) {
                    m_fallOrientation = CharacterFallOrientation3D::FaceUp;
                }
            }
            // Let the body actually come to rest before committing to a
            // recovery: which clip to play depends on which way it lands, and
            // that is not decided while it is still tumbling.
            float fastestLink = 0.0f;
            for (const PhysicsBodyState3D& link : state.links) {
                fastestLink = std::max(fastestLink,
                    link.linearVelocity.length());
            }
            // Tonus deitado ("ressustentacao"): enquanto o corpo ainda rola
            // ou desliza, os musculos cedem - a energia fica nas juntas em vez
            // de virar cambalhota do corpo inteiro, duro. Parado de vez, ele
            // fica largado ~0,7 s e o tonus volta aos poucos (~0,75 s),
            // enquanto ele se arruma para levantar (pose, mais abaixo); so
            // entao levanta.
            //
            // "Parado" e o tronco (pelve e peito), nao o membro mais rapido:
            // o proprio tonus voltando leva bracos e pernas para a pose
            // deitada, e medido pelos membros isso contava como "ainda
            // rolando" - o tonus desabava de novo, em ciclo, ate o limite de
            // tempo. Depois que ele comecou a voltar, so um tranco de verdade
            // no tronco o derruba de novo.
            float coreSpeed = state.links.front().linearVelocity.length();
            if (m_chestLink < state.links.size()) {
                coreSpeed = std::max(coreSpeed,
                    state.links[m_chestLink].linearVelocity.length());
            }
            // Na primeira vez fica largado mexendo-se; numa nova tentativa (ja
            // acordado, tentando) espera pouco.
            const float restBeforeGather = m_getUpAttempt > 1
                ? RetryRestBeforeGather : SettleRestBeforeGather;
            const bool recovering = m_settleRestSeconds > restBeforeGather;
            // Arrumando-se para levantar ele mesmo mexe o tronco (de lado para
            // de costas e um rolar): so um tranco de verdade interrompe.
            // Depois de parar, os proprios movimentos deitado (erguer a
            // cabeca, apoiar nos cotovelos) tambem nao contam como rolar.
            const float restLimit = recovering ? 1.50f
                : m_settleRestSeconds > 0.25f ? 0.80f : 0.35f;
            m_settleRestSeconds = coreSpeed < restLimit
                ? m_settleRestSeconds + deltaTime : 0.0f;
            m_settleTone = m_settleRestSeconds > restBeforeGather
                ? std::min(1.0f, m_settleTone + deltaTime * SettleToneRise * getUpSpeed)
                : std::max(SettleRestingTone, m_settleTone - deltaTime * 2.5f);
            // Rolar antes de levantar (TEMPTUNE GetUpRoll): de lado (ou de
            // costas, com GetUpRollFromBack), arrumando-se, ele rola ate
            // ficar de bruços - o levantar de bruços parte de uma pose
            // boa. Como no Prone Roll de referencia: a perna e o braco de
            // cima cruzam por cima para o lado da barriga e o quadril puxa o
            // tronco (uns 20 graus a frente dos ombros).
            // O tronco alinhado com o lado do levantar (peito e pelve de
            // costas para cima ou de brucos para baixo): de lado ou torcido
            // (o peito para cima e a pelve de lado), o clipe puxava um corpo
            // na postura errada (plano v2, fase D) - ele rola de brucos
            // antes, como de lado.
            bool trunkAligned = true;
            if (m_chestLink < state.links.size() && !state.links.empty()) {
                const float chestFacing = state.links[m_chestLink].orientation.rotate({ 1.0f, 0.0f, 0.0f }).z;
                const float pelvisFacing = (state.links.front().orientation
                    * profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f }).z;
                trunkAligned = m_fallOrientation == CharacterFallOrientation3D::FaceDown
                    ? chestFacing < -0.5f && pelvisFacing < -0.35f
                    : chestFacing > 0.5f && pelvisFacing > 0.35f;
            }
            if (GetUpRollEnabled && m_chestLink < state.links.size()) {
                const Vec3 chestForward = state.links[m_chestLink].orientation.rotate({ 1.0f, 0.0f, 0.0f });
                const Vec3 spine = (state.links[m_chestLink].position - state.links.front().position).normalized();
                // (Desalinhado so conta com o corpo ja pronto para levantar:
                // no meio do assentar ele ainda se arruma.)
                const bool readyButTwisted = !trunkAligned && m_getUpPhaseSeconds > GetUpSettleSeconds
                    && coreSpeed < 0.35f && m_settleTone > 0.97f;
                if (!m_getUpRolling && !m_getUpRollDone && recovering && m_settleTone > 0.6f
                    && (std::abs(chestForward.z) < GetUpRollSideFacing || readyButTwisted
                        || (GetUpRollFromBack && chestForward.z > 0.0f))) {
                    // Gira em torno da coluna no sentido que leva a barriga
                    // para o chao: d(frente.z)/d(angulo) = (coluna x frente).z.
                    const float lean = cross(spine, chestForward).z;
                    m_getUpRollSign = lean > 0.0f ? -1.0f : 1.0f;
                    // O lado de cima: o quadril que sobe com o giro.
                    const std::size_t thighs[2] = { findLink(profile, "LeftThigh"), findLink(profile, "RightThigh") };
                    m_getUpRollTop = -1;
                    float best = -1e9f;
                    for (int side = 0; side < 2; ++side) {
                        if (thighs[side] == 0 || thighs[side] >= state.links.size()) continue;
                        const Vec3 hip = state.links[thighs[side]].position - state.links.front().position;
                        const float rise = cross(spine * m_getUpRollSign, hip).z;
                        if (rise > best) { best = rise; m_getUpRollTop = side; }
                    }
                    m_getUpRolling = m_getUpRollTop >= 0;
                    m_getUpRollSeconds = 0.0f;
                }
                if (m_getUpRolling) {
                    m_getUpRollSeconds += deltaTime;
                    const bool prone = chestForward.z < -0.6f;
                    if (prone || m_getUpRollSeconds > GetUpRollTimeout) {
                        m_getUpRolling = false;
                        m_getUpRollDone = true;
                        if (prone) m_fallOrientation = CharacterFallOrientation3D::FaceDown;
                        else classifyFallOrientation();
                        // Arruma-se de novo a partir de onde rolou.
                        m_settlePoseValid = false;
                    }
                }
            }
            // Levantar so com o corpo deitado: invertido (pernas por cima da
            // cabeca, pelve alem de ~125 graus) o clipe comecava de uma pose
            // que o corpo nao tinha e ele rolava de novo.
            const float settledTilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
            // E com o corpo ja na pose de levantar (se arrumou): um braco
            // preso embaixo do corpo nao chegava, e ao levantar os ganhos
            // o arrancavam de la (mao a 14 m/s).
            // (Desalinhado e ainda sem ter rolado, espera a rolagem.)
            const bool settled = m_getUpPhaseSeconds > (getUpHurry > 0 ? GetUpHurrySettleSeconds : GetUpSettleSeconds)
                && !m_getUpRolling && (trunkAligned || m_getUpRollDone || !GetUpRollEnabled)
                && coreSpeed < 0.35f * (getUpHurry > 0 ? GetUpCalmScale : 1.0f)
                && fastestLink < 1.0f * (getUpHurry > 0 ? GetUpCalmScale : 1.0f)
                && m_settleTone > 0.97f && settledTilt < 2.2f
                && m_telemetry.jointErrorRmsDegrees < 12.0f;
            // No modo fisico o corpo cai com mais energia e desliza mais:
            // comecar o levantar com ele ainda andando (o limite de 2 s)
            // puxava a raiz de uma vez para o clipe - membros a 77 m/s.
            const float settleLimit = input.forceDrivenRoot ? 4.0f : 2.0f;
            if ((settled || m_getUpPhaseSeconds > settleLimit + (m_getUpRollDone ? 2.0f : 0.0f))
                && !m_getUpRolling) {
                // O lado (costas/bruços) e o do deitado, que ja tem folga:
                // reclassificar aqui, de lado (peito na vertical), trocava o
                // levantar para o outro clipe com o corpo arrumado para este
                // (mao a 14 m/s, levantar emperrado).
                m_getUpClipSeconds = 0.0f;
                if (!state.links.empty()) {
                    // The clip plays where the body is lying, not where the
                    // capsule happens to be standing.
                    const Vec3 root = state.links.front().position;
                    m_getUpAnchorWorld = { root.x, root.y, root.z };
                    const Vec3 forward = state.links.front()
                        .orientation.rotate({ 1.0f, 0.0f, 0.0f });
                    const float planar = std::hypot(forward.x, forward.y);
                    m_getUpAnchorYaw = planar > 0.05f
                        ? std::atan2(forward.y, forward.x) : m_facingYaw;
                }
                m_getUpRate = 1.0f;
                m_getUpStruggle = 0.0f;
                m_getUpStallSeconds = 0.0f;
                m_getUpHoldSeconds = 0.0f;
                m_getUpExpectedValid = false;
                m_risingFromMeasured.assign(state.joints.size(), Vec3 {});
                for (std::size_t link = 1; link < state.joints.size(); ++link) {
                    m_risingFromMeasured[link] = { state.joints[link].positionRadians[0],
                        state.joints[link].positionRadians[1],
                        state.joints[link].positionRadians[2] };
                }
                enterGetUpPhase(CharacterGetUpPhase3D::Rising);
                m_bodilyRising = GetUpBodily;
                if (m_bodilyRising) {
                    m_bodilyGetUp.begin(profile, state, std::isfinite(contactFloor)
                        ? contactFloor : groundHeightGuess);
                }
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising && m_bodilyRising) {
            // Levantar pelo corpo: as etapas decidem; nada carrega o corpo.
            const bool standing = m_bodilyGetUp.update(profile, state, std::isfinite(contactFloor)
                ? contactFloor : groundHeightGuess, deltaTime, m_bodilyPose);
            if (m_bodilyGetUp.failed()) {
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                m_bodilyRising = false;
            } else if (standing) {
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_bodilyRising = false;
                const Vec3 forward = state.links.front().orientation.rotate({ 1.0f, 0.0f, 0.0f });
                m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                    ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw : m_facingYaw;
                m_turningRate = 0.0f;
                m_plantFeetFromBody = true;
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising) {
            // Levantar biomecanico: o clipe e o roteiro das juntas; quem
            // levanta e o corpo. O tempo do clipe anda no ritmo que o corpo
            // acompanha (altura e inclinacao da pelve contra as do clipe):
            // atrasado, o clipe espera. Nada carrega a raiz - antes ela era
            // levada ao clipe, o "teleporte" (pelve a 677 mm num tick).
            if (m_getUpExpectedValid && !state.links.empty()) {
                const float ground = input.rootPositionWorld.z
                    - profile.standingRootHeightMeters;
                const float height = state.links.front().position.z - ground;
                const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
                const float lowBy = m_getUpExpectedHeight - height;
                const float tiltedBy = tilt - m_getUpExpectedTilt;
                const float heightRate = 1.0f - (lowBy - 0.06f) / 0.14f;
                // Na fase final (o clipe ja quase em pe) a inclinacao pesa
                // mais: com a pelve inclinada alem do clipe ele nao segue
                // subindo - primeiro endireita, depois fica de pe. Antes o
                // clipe seguia e ele subia como uma prancha inclinada.
                const bool finalPhase = m_getUpExpectedTilt < 0.8f
                    && height > 0.55f;
                // (Ja na altura de pe - pelve acima de GetUpLateHeight - o
                // clipe nao espera mais a inclinacao: esperando, os alvos do
                // quadril e da coluna ficavam dobrados e o corpo, de pe e
                // inclinado, ~0,55 s ate a ajuda o erguer. Seguindo, eles vao
                // a pose reta e erguem o tronco.)
                const float tiltRate = finalPhase && height > GetUpLateHeight ? 1.0f
                    : finalPhase
                    ? 1.0f - (tiltedBy - 0.12f) / 0.25f
                    : 1.0f - (tiltedBy - 0.21f) / 0.44f;
                // As pernas tambem: se coxas e canelas estao longe da pose
                // do clipe (os joelhos ainda nao vieram para baixo do corpo),
                // o clipe espera - subir a pelve com as pernas esticadas para
                // tras deixava o corpo numa ponte de cabeca no chao.
                float legsBehind = 0.0f;
                for (std::size_t index = 1; index < profile.links.size()
                        && index < state.joints.size()
                        && index < m_coordinates.size(); ++index) {
                    const std::string& id = profile.links[index].id;
                    if (id.find("Thigh") == std::string::npos
                        && id.find("Shin") == std::string::npos) continue;
                    const Vec3 measured { state.joints[index].positionRadians[0],
                        state.joints[index].positionRadians[1],
                        state.joints[index].positionRadians[2] };
                    legsBehind = std::max(legsBehind,
                        (m_coordinates[index] - measured).length());
                }
                const float legsRate = 1.0f - (legsBehind - 0.45f) / 0.50f;
                // O peito tambem: de bruços, a flexao de bracos (peito fora do
                // chao) vem antes de puxar os joelhos. Sem esperar por ela o
                // clipe seguia com o peito no chao e o corpo se encolhia de
                // cabeca no chao, arrastando-a.
                float chestRate = 1.0f;
                if (m_chestLink < state.links.size()) {
                    const float chest = state.links[m_chestLink].position.z - ground;
                    chestRate = 1.0f
                        - (m_getUpExpectedChestHeight - chest - 0.08f) / 0.15f;
                }
                // O ritmo acelerado (getUpSpeed) so onde o clipe nao sobe
                // rapido: onde a pelve dele sobe, no maximo GetUpMaxRise m/s
                // (pedido do usuario, 05/10 noite: a 1,9x o empurrao dos
                // bracos e o ficar de pe pediam ~2,1 m/s e o corpo saltava,
                // ~0,25 s fora do chao em cada um).
                float speedHere = getUpSpeed;
                {
                    const AnimationClip3D* riseClip = m_fallOrientation == CharacterFallOrientation3D::FaceDown
                        ? animations.standUpFront : animations.standUpBack;
                    if (riseClip != nullptr && riseClip->durationSeconds > 0.1f) {
                        const float now = std::min(m_getUpClipSeconds, riseClip->durationSeconds - 0.06f);
                        const float rise = (clipRootAt(profile, *riseClip, now + 0.06f).translationOffsetMeters.z
                            - clipRootAt(profile, *riseClip, now).translationOffsetMeters.z) / 0.06f;
                        if (rise > 1e-3f) speedHere = std::clamp(GetUpMaxRise / rise, 1.0f, getUpSpeed);
                        // E as pernas: esticando o joelho e o quadril rapido o
                        // corpo saltava (no fim, de pe, a 2,5x - a pelve a
                        // 1,05 m), mesmo com a pelve do clipe quase parada.
                        float legSpeed = 0.0f;
                        for (std::size_t index = 1; index < profile.links.size(); ++index) {
                            const std::string& id = profile.links[index].id;
                            if (id.find("Thigh") == std::string::npos && id.find("Shin") == std::string::npos) continue;
                            if (const AnimationTrack3D* track = findAnimationTrack3D(*riseClip, id)) {
                                const Vec3 a = sampleAnimationTrack3D(*track, now, riseClip->durationSeconds, false).jointPositionRadians;
                                const Vec3 b = sampleAnimationTrack3D(*track, now + 0.06f, riseClip->durationSeconds, false).jointPositionRadians;
                                legSpeed = std::max(legSpeed, (b - a).length() / 0.06f);
                            }
                        }
                        if (legSpeed > 1e-3f) speedHere = std::min(speedHere, std::max(1.0f, GetUpMaxLegSpeed / legSpeed));
                    }
                }
                m_getUpRate = std::clamp(std::min({ heightRate, tiltRate,
                    legsRate, chestRate }), 0.1f, 1.0f) * speedHere;
            } else {
                m_getUpRate = 1.0f;
            }
            // Emperrado, o esforco (e o teto da ajuda) sobe devagar.
            // (Sobe em ~0,5 s: mais devagar, o corpo desabava antes de a
            // ajuda responder - fase de sentar do levantar de costas.)
            m_getUpStruggle = m_getUpRate < 0.5f
                ? std::min(1.0f, m_getUpStruggle + deltaTime * 2.0f)
                : std::max(0.0f, m_getUpStruggle - deltaTime
                    * (m_getUpRate > 0.8f ? 1.0f : 0.0f));
            // Emperrado de verdade: o clipe esperando E o corpo sem subir nem
            // endireitar. Progredindo devagar (a pelve subindo, a inclinacao
            // caindo), ele nao desiste - antes desistia no meio da subida.
            bool progressing = false;
            if (!state.links.empty()) {
                const float tiltNow = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
                progressing = state.links.front().linearVelocity.z > 0.05f
                    || (m_previousTilt - tiltNow) / std::max(deltaTime, 1e-4f) > 0.08f;
            }
            m_getUpStallSeconds = m_getUpRate <= 0.2f && !progressing
                ? m_getUpStallSeconds + deltaTime : std::max(0.0f,
                    m_getUpStallSeconds - (progressing ? deltaTime : 3.0f));
            {
                const AnimationClip3D* frontClip = animations.standUpFront;
                const float release = GetUpHandRelease * (frontClip && frontClip->durationSeconds > 0.05f
                    ? frontClip->durationSeconds : 1.6f);
                const float before = m_getUpClipSeconds;
                m_getUpClipSeconds += deltaTime * m_getUpRate;
                // Os pes do equilibrio pedem: as maos so soltam com o corpo
                // sobre os pes (o clipe espera antes de solta-las).
                if (m_getUpBalanceHold && m_fallOrientation == CharacterFallOrientation3D::FaceDown)
                    m_getUpClipSeconds = std::max(before, std::min(m_getUpClipSeconds, release));
            }
            const AnimationClip3D* clip = m_fallOrientation
                    == CharacterFallOrientation3D::FaceDown
                ? animations.standUpFront : animations.standUpBack;
            const float duration = clip && clip->durationSeconds > 0.05f
                ? clip->durationSeconds : 1.6f;
            // Fim do clipe so conta com o corpo em pe de verdade; senao ele
            // segura a pose final (com a ajuda) um pouco, e desiste.
            const bool upright = rootUpright > 0.94f && !state.links.empty()
                && state.links.front().position.z
                    - (input.rootPositionWorld.z - profile.standingRootHeightMeters)
                    > profile.standingRootHeightMeters * 0.80f;
            // Perto da pose agachada do comeco da arrancada (Crouched To
            // Sprinting: pelve a 0,55 m e 45 graus, peito a 66, os pes no chao
            // embaixo da pelve): o padrao pode assumir com ela.
            m_getUpHandoffReady = false;
            if (getUpHurry > 0 && m_getUpClipSeconds >= GetUpHandoffProgress * duration && !state.links.empty()) {
                const float ground = input.rootPositionWorld.z - profile.standingRootHeightMeters;
                const Vec3 pelvisPosition = state.links.front().position;
                float chestTilt = 0.0f;
                if (m_chestLink < state.links.size()) {
                    const Vec3 chestUp = (state.links[m_chestLink].orientation
                        * profile.links[m_chestLink].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                    chestTilt = std::acos(std::clamp(chestUp.z, -1.0f, 1.0f));
                }
                const Vec3 forward { std::cos(m_getUpAnchorYaw), std::sin(m_getUpAnchorYaw), 0.0f };
                bool feetUnder = true;
                for (const FootPlant& foot : m_feet) {
                    if (!foot.valid || foot.linkIndex >= state.links.size()) { feetUnder = false; continue; }
                    const Vec3 at = state.links[foot.linkIndex].position;
                    feetUnder = feetUnder && at.z - ground < 0.22f
                        && std::abs(dot(Vec3 { at.x - pelvisPosition.x, at.y - pelvisPosition.y, 0.0f }, forward)) < GetUpHandoffFeet;
                }
                m_getUpHandoffReady = pelvisPosition.z - ground > GetUpHandoffHeight
                    && rootUpright > std::cos(GetUpHandoffTilt) && chestTilt < GetUpHandoffChestTilt && feetUnder;
            }
            // Com pressa: quase em pe ja sai (o resto do clipe e endireitar
            // parado, que a arrancada faz andando).
            const bool hurryExit = getUpHurry > 0 && m_getUpClipSeconds >= GetUpHurryExit * duration
                && rootUpright > GetUpHurryUpright && !state.links.empty()
                && state.links.front().position.z
                    - (input.rootPositionWorld.z - profile.standingRootHeightMeters)
                    > profile.standingRootHeightMeters * 0.75f;
            if (m_getUpClipSeconds >= duration && !upright) {
                m_getUpClipSeconds = duration;
                m_getUpHoldSeconds += deltaTime;
            }
            // (E um teto: oscilando para sempre sem terminar, tenta de novo.)
            if (m_getUpStallSeconds > 3.0f || m_getUpHoldSeconds > 1.5f
                || m_getUpPhaseSeconds > 9.0f) {
                // Nao deu: volta a deitar e tenta de novo (pode escolher o
                // outro clipe, se rolou).
                classifyFallOrientation();
                enterGetUpPhase(CharacterGetUpPhase3D::Settling);
                ++m_getUpAttempt;
                m_settleRestSeconds = 0.0f;
                m_settlePoseValid = false;
                m_getUpStallSeconds = 0.0f;
                m_getUpHoldSeconds = 0.0f;
                m_getUpStruggle = 0.0f;
            } else if ((m_getUpClipSeconds >= duration && upright) || hurryExit) {
                m_getUpPhase = CharacterGetUpPhase3D::None;
                m_fallOrientation = CharacterFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_getUpClipSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_getUpHoldSeconds = 0.0f;
                m_getUpStallSeconds = 0.0f;
                m_getUpStruggle = 0.0f;
                m_getUpExpectedValid = false;
                // O rumo e o do corpo em pe, onde ele terminou de levantar.
                {
                    const Vec3 forward = state.links.front().orientation
                        .rotate({ 1.0f, 0.0f, 0.0f });
                    m_facingYaw = std::hypot(forward.x, forward.y) > 0.2f
                        ? std::atan2(forward.y, forward.x) - m_clipPelvisYaw
                        : m_getUpAnchorYaw;
                }
                m_getUpHurriedExit = getUpHurry > 0;
                m_turningRate = 0.0f;
                // Os pes ficam onde estao, travados no chao (abaixo, onde a
                // sonda do chao existe). Soltos, o IK levava os dois de uma vez
                // para a base parada - o "espacate" da troca. Os passos de
                // acomodacao refazem a base, um pe de cada vez.
                m_plantFeetFromBody = true;
                // O olhar entra de novo aos poucos (o filtro continua de zero).
                m_torsoTwist = 0.0f;
                m_headTurn = 0.0f;
            }
        }
    }
    t.getUpActive = m_getUpPhase != CharacterGetUpPhase3D::None;
    const bool& getUpActive = t.getUpActive;
    m_sinceGetUpSeconds = getUpActive ? 0.0f
        : std::min(10.0f, m_sinceGetUpSeconds + deltaTime);
    // Cambaleio (ver StaggerImpulseMin): o choque deste tick e o tonus.
    {
        float impulse = 0.0f;
        if (input.staggerRecovery && physicsOwnsBody && !getUpActive)
            for (const RagdollInteraction3D& contact : state.interactions) {
                if (contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if ((id == "LeftFoot" || id == "RightFoot") && contact.normalWorld.z > 0.62f) continue;
                // (O chao logo depois de levantar ou do carrinho nao e choque.)
                if (contact.normalWorld.z > 0.62f && (m_sinceGetUpSeconds < 1.0f || m_sinceTackleSeconds < 0.8f))
                    continue;
                impulse += contact.normalImpulseNewtonSeconds;
            }
        if (!input.staggerRecovery || getUpActive) {
            m_stagger = 0.0f;
        } else {
            const float hit = std::clamp((impulse - StaggerImpulseMin)
                / std::max(1.0f, StaggerImpulseFull - StaggerImpulseMin), 0.0f, 1.0f);
            if (hit > m_stagger) {
                m_stagger = hit;
                m_staggerHold = StaggerHoldSeconds * hit;
            }
            const float upright = smoothStep((rootUpright - 0.60f) / 0.35f);
            if (m_staggerHold > 0.0f) m_staggerHold -= deltaTime;
            else m_stagger = std::max(0.0f, m_stagger - deltaTime * StaggerRecoverPerSecond * (0.35f + 0.65f * upright));
        }
        m_telemetry.stagger = m_stagger;
        m_telemetry.staggerImpulse = impulse;
    }
    // (Os limites da postura e da queda sem volta, mais tolerantes enquanto
    // ele esta amolecido: ele tenta se manter.)
    const float staggerRelief = StaggerTiltRelief * m_stagger;

    // Reflexos de queda: com o corpo tombando de verdade (pelve alem de ~40
    // graus da vertical; no modo antigo, a partir de 25 graus depois de uma
    // pancada) os reflexos entram e as juntas ficam com tonus - nao a 3% da
    // forca, que era o "corpo morto molengo". Deitado, fica inteiro; no
    // levantar, sai.
    {
        const float tilt = std::acos(std::clamp(rootUpright, -1.0f, 1.0f));
        // Tombando rapido (mais de 1,5 rad/s) os reflexos entram ja aos 25
        // graus: numa queda de lado o corpo vai ao chao em ~0,35 s, e
        // esperando os 40 graus as maos ainda estavam a 30 cm do chao quando
        // a cabeca batia. Inclinacao parada (a corrida inclina ate ~35) nao.
        const float tiltRate = (tilt - m_previousTilt) / std::max(deltaTime, 1e-4f);
        m_previousTilt = tilt;
        const float quickTipping = smoothStep((tilt - 0.35f) / 0.26f)
            * smoothStep((tiltRate - 1.0f) / 1.0f);
        const float tipping = !physicsOwnsBody ? 0.0f
            : input.forceDrivenRoot
                ? std::max(smoothStep((tilt - 0.70f) / 0.26f), quickTipping)
                : std::max(smoothStep((tilt - 0.44f) / 0.26f), quickTipping)
                    * smoothStep((m_physicsBlend - 0.3f) / 0.4f);
        const float wanted = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 0.0f
            : tipping;
        m_fallReflex += (wanted - m_fallReflex) * std::clamp(
            deltaTime * (wanted > m_fallReflex ? 18.0f : 3.0f), 0.0f, 1.0f);
        // Desequilibrado nao e caido: com o corpo fisico, os bracos protegem
        // ja no comeco (acima), mas as pernas seguem no passo, com o tonus de
        // pe, ate a queda ficar sem volta - tombado alem de ~50 graus e ainda
        // indo (ou 40 graus girando muito rapido). Ir logo para as pernas
        // moles tirava a chance de o passo salvar o equilibrio e o comando do
        // jogador parava de valer no meio do tropeco.
        const float committing = input.forceDrivenRoot
            ? std::max(smoothStep((tilt - 0.87f - staggerRelief) / 0.22f)
                    * smoothStep((tiltRate + 0.3f) / 0.6f),
                smoothStep((tilt - 0.70f - staggerRelief) / 0.26f)
                    * smoothStep((tiltRate - 2.5f) / 1.0f))
            : tipping;
        const float commitWanted = m_getUpPhase == CharacterGetUpPhase3D::Settling
            ? 1.0f : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 0.0f
            : std::max(committing, input.forceDrivenRoot && m_fallCommit > 0.5f
                ? smoothStep((tilt - 0.70f - staggerRelief) / 0.26f) : 0.0f);
        // De volta em pe (tronco a menos de ~25 graus), as pernas voltam logo
        // ao passo e ao tonus: com a volta lenta ele se empurrava de cima de
        // uma caixa, ficava reto, e caia de costas de pernas ainda moles.
        const float commitRelease = m_getUpPhase == CharacterGetUpPhase3D::None
            && tilt < 0.44f && (feetGroundContact[0] || feetGroundContact[1])
            ? 10.0f : 3.0f;
        // (Apoiado tentando se recompor: as pernas ficam com o tonus de pe.)
        const float commitTarget = m_braceRecovering ? 0.0f : commitWanted;
        m_fallCommit += (commitTarget - m_fallCommit) * std::clamp(
            deltaTime * (commitTarget > m_fallCommit ? 18.0f : (m_braceRecovering ? 10.0f : commitRelease)),
            0.0f, 1.0f);
        // Rumo da queda: para onde o centro de massa vai; devagar, para onde
        // o corpo tomba.
        Vec3 fall = m_filteredCenterOfMassVelocity;
        fall.z = 0.0f;
        if (fall.lengthSquared() < 0.16f && !state.links.empty()) {
            fall = state.links.front().orientation.rotate({ 0.0f, 0.0f, 1.0f });
            fall.z = 0.0f;
        }
        if (fall.lengthSquared() > 0.0004f) {
            const Vec3 unit = fall.normalized();
            m_fallDirection = (m_fallDirection + (unit - m_fallDirection)
                * std::clamp(deltaTime * 12.0f, 0.0f, 1.0f));
            if (m_fallDirection.lengthSquared() > 1e-6f)
                m_fallDirection = m_fallDirection.normalized();
        }
        // Postura perdida (ver PostureLossFall): com o corpo fisico, de pe
        // (sem levantar, carrinho nem pulo).
        const CharacterLocomotionState3D previousState = m_telemetry.state;
        const bool postureStanding = PostureLossFall && physicsOwnsBody && input.forceDrivenRoot
            && m_getUpPhase == CharacterGetUpPhase3D::None
            && previousState != CharacterLocomotionState3D::Tackling
            && previousState != CharacterLocomotionState3D::Fallen
            && previousState != CharacterLocomotionState3D::GettingUp
            && previousState != CharacterLocomotionState3D::Airborne
            && previousState != CharacterLocomotionState3D::JumpStarting
            && previousState != CharacterLocomotionState3D::Landing;
        // O pendulo (centro de massa contra o meio dos pes) da vertical e
        // contra o da pose pedida; e o peito da vertical.
        float deviation = 0.0f, pendulum = 0.0f, chestTilt = 0.0f;
        bool supportedPendulum = false;
        if (m_feet.size() >= 2 && m_feet[0].valid && m_feet[1].valid
            && m_feet[0].linkIndex < state.links.size() && m_feet[1].linkIndex < state.links.size()) {
            const Vec3 arm = pendulumArm(profile, m_feet[0].linkIndex, m_feet[1].linkIndex,
                [&](std::size_t i) { return state.links[i].position; });
            const Vec3 wanted = m_referencePendulumValid ? m_referencePendulum : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 armUnit = arm * (1.0f / std::max(1e-3f, arm.length()));
            const Vec3 wantedUnit = wanted * (1.0f / std::max(1e-3f, wanted.length()));
            deviation = std::acos(std::clamp(dot(armUnit, wantedUnit), -1.0f, 1.0f));
            pendulum = std::acos(std::clamp(armUnit.z, -1.0f, 1.0f));
            // v2 (T_PostureSupport): o pendulo e contra os pes APOIADOS (o do
            // alvo, contra os mesmos pes da pose alvo). Pelo meio dos dois,
            // correndo, o pe no ar - a 0,7 m para o lado no zigue-zague - o
            // inflava: a postura era dada como perdida com o corpo a 34 graus
            // e a ajuda saia de vez (a 100%, 15 de 174 rodadas caiam assim).
            // Sem pe apoiado (o voo da passada), o tempo segurado nao conta.
            // (So na passada - andando ou correndo, sem os passos de freada:
            // nos passos do planejador, parado, o pe no ar e o que vai segurar
            // o corpo - empurrado para tras, contra o pe apoiado, ele "perdia
            // a postura" no meio do passo que salvava.)
            const bool inGait = (previousState == CharacterLocomotionState3D::Walking
                || previousState == CharacterLocomotionState3D::Running) && !m_brakeStepping;
            if (PostureSupportPendulum && inGait) {
                const bool left = feetGroundContact[0], right = feetGroundContact[1];
                if (left || right) {
                    float total = 0.0f;
                    Vec3 center {}, wantedCenter {};
                    for (std::size_t i = 0; i < profile.links.size(); ++i) {
                        center = center + state.links[i].position * profile.links[i].massFraction;
                        total += profile.links[i].massFraction;
                    }
                    center = center * (1.0f / std::max(1e-4f, total));
                    const Vec3 support = left && right
                        ? (state.links[m_feet[0].linkIndex].position + state.links[m_feet[1].linkIndex].position) * 0.5f
                        : state.links[m_feet[left ? 0 : 1].linkIndex].position;
                    const Vec3 wantedSupport = left && right
                        ? (m_referenceFeetWorld[0] + m_referenceFeetWorld[1]) * 0.5f
                        : m_referenceFeetWorld[left ? 0 : 1];
                    const Vec3 supportArm = center - support;
                    const Vec3 wantedArm = m_referencePendulumValid ? m_referenceCenterOfMassWorld - wantedSupport
                        : Vec3 { 0.0f, 0.0f, 1.0f };
                    const Vec3 supportUnit = supportArm * (1.0f / std::max(1e-3f, supportArm.length()));
                    const Vec3 wantedSupportUnit = wantedArm * (1.0f / std::max(1e-3f, wantedArm.length()));
                    deviation = std::acos(std::clamp(dot(supportUnit, wantedSupportUnit), -1.0f, 1.0f));
                    pendulum = std::acos(std::clamp(supportUnit.z, -1.0f, 1.0f));
                    supportedPendulum = true;
                }
            }
        }
        if (m_chestLink != 0 && m_chestLink < state.links.size()) {
            const Quaternion bind = profile.links[m_chestLink].modelOrientation.conjugate();
            const Vec3 chestUp = (state.links[m_chestLink].orientation * bind).rotate({ 0.0f, 0.0f, 1.0f });
            chestTilt = std::acos(std::clamp(chestUp.z, -1.0f, 1.0f));
        }
        // O tempo segurado vaza devagar em vez de zerar: segurado em dois
        // lobos de ~0,3 s (39 graus, o peito a 48, sem pe no chao), com um
        // vale de 0,1 s entre eles, ele nunca contava.
        const bool pendulumCounts = !PostureSupportPendulum || supportedPendulum
            || !((previousState == CharacterLocomotionState3D::Walking
                || previousState == CharacterLocomotionState3D::Running) && !m_brakeStepping);
        if (!postureStanding) m_postureHeldSeconds = 0.0f;
        else if (!pendulumCounts) {}
        else if (pendulum > PostureHeldTilt + staggerRelief && deviation > PostureHeldDeviation + staggerRelief)
            m_postureHeldSeconds += deltaTime;
        else m_postureHeldSeconds = std::max(0.0f, m_postureHeldSeconds - deltaTime);
        if (m_braceRecovering) {
            // Apoiado e tentando se recompor: nao e postura perdida.
            m_postureLost = false;
            m_postureHeldSeconds = 0.0f;
        } else if (m_postureLost) {
            // Caiu de vez: ate ir ao chao (o levantar comeca) ou se, mesmo
            // assim, ficar de pe.
            if (m_getUpPhase != CharacterGetUpPhase3D::None || (pendulumCounts && pendulum < PostureRegainedTilt))
                m_postureLost = false;
        } else if (postureStanding && ((pendulumCounts && pendulum > PostureLostTilt + staggerRelief
                && !(PostureSupportPendulum && supportedPendulum))
            // (Na passada, pelo pendulo so o sustentado - abaixo: na hora,
            // um pe pousando longe a frente numa freada dava 43 graus com o
            // corpo indo para cima dele. Na hora vale a pelve tombada.)
            || (PostureSupportPendulum && supportedPendulum
                && std::acos(std::clamp(rootUpright, -1.0f, 1.0f)) > PostureGaitPelvisLost + staggerRelief)
            || chestTilt > PostureChestLost + staggerRelief
            || m_postureHeldSeconds > PostureHeldSeconds + 0.4f * m_stagger)) {
            m_postureLost = true;
        }
        if (m_postureLost) {
            // As pernas cedem e os reflexos entram ja (a ajuda sai pelo
            // controlador: telemetry.postureLost).
            m_fallCommit = 1.0f;
            m_fallReflex = 1.0f;
        }
        m_telemetry.postureDeviation = deviation;
        m_telemetry.posturePendulum = pendulum;
        m_telemetry.postureChestTilt = chestTilt;
        m_telemetry.postureLost = m_postureLost;
        m_telemetry.getUpHurried = m_getUpHurriedExit && m_sinceGetUpSeconds < 2.0f;
        m_telemetry.getUpHandoffReady = m_getUpHandoffReady && m_getUpPhase == CharacterGetUpPhase3D::Rising;
    }

}

} // namespace MatterEngine
