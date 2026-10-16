#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepRoot(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const bool& getUpActive = t.getUpActive;
    Vec3& brakeNavigationVelocity = t.brakeNavigationVelocity;
    const float& speed = t.speed;
    CharacterLocomotionState3D& desiredState = t.desiredState;
    const Quaternion& heading = t.heading;
    const bool& gaitMoving = t.gaitMoving;
    const float& startupPelvisDrop = t.startupPelvisDrop;
    const AnimationClip3D*& standUpClip = t.standUpClip;
    Quaternion& rootRotation = t.rootRotation;
    Vec3& rootTranslation = t.rootTranslation;
    const float& transition = t.transition;
    const float& tangentWeight = t.tangentWeight;
    float& forwardLean = t.forwardLean;
    float& lateralLean = t.lateralLean;
    // A direcao vem do referencial da locomocao. Dos ciclos (autorais, que
    // olham para +X) fica a rotacao inteira da pelve - inclinacao, queda do
    // quadril e o giro de ate 8 graus que acompanha a perna que avanca. Do
    // levantar (importado, com rumo arbitrario) sai o rumo da fonte e fica
    // so a inclinacao.
    // Rumo da fonte estavel deitado: frente + cima * (-frente.z). Em pe e a
    // frente; de bruços (frente para o chao) e de costas (frente para o
    // ceu) a frente no plano some e, passando de 90 graus, virava 180 graus
    // de um quadro para o outro - o cima completa (de costas, para os pes).
    const Vec3 sourceForward = rootRotation.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 sourceUp = rootRotation.rotate({ 0.0f, 0.0f, 1.0f });
    const Quaternion sourceHeading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, std::atan2(sourceForward.y - sourceUp.y * sourceForward.z,
            sourceForward.x - sourceUp.x * sourceForward.z));
    // (No carrinho o giro do clipe fica: ele desliza de lado em relacao a
    // direcao do carrinho, e o clipe ja vem no rumo do comeco.)
    const Quaternion clipRootTilt = standUpClip != nullptr && !m_tackling
        ? (sourceHeading.conjugate() * rootRotation).normalized()
        : rootRotation.normalized();
    // Mesmo crossfade das juntas (ver `transition` acima). Trocar de clipe
    // nunca pode saltar a pelve: o guia deriva velocidade por diferenca
    // finita, e um salto vira velocidade angular aplicada ao corpo inteiro.
    Quaternion tiltTarget = clipRootTilt;
    if (m_transitionFromRootTilt.x * tiltTarget.x
            + m_transitionFromRootTilt.y * tiltTarget.y
            + m_transitionFromRootTilt.z * tiltTarget.z
            + m_transitionFromRootTilt.w * tiltTarget.w < 0.0f) {
        tiltTarget = { -tiltTarget.x, -tiltTarget.y, -tiltTarget.z,
            -tiltTarget.w };
    }
    const Quaternion blendedRootTilt = Quaternion {
        m_transitionFromRootTilt.x
            + (tiltTarget.x - m_transitionFromRootTilt.x) * transition,
        m_transitionFromRootTilt.y
            + (tiltTarget.y - m_transitionFromRootTilt.y) * transition,
        m_transitionFromRootTilt.z
            + (tiltTarget.z - m_transitionFromRootTilt.z) * transition,
        m_transitionFromRootTilt.w
            + (tiltTarget.w - m_transitionFromRootTilt.w) * transition
    }.normalized();
    t.rootTilt = (exponentialRotation(
        m_transitionRootTiltVelocity * tangentWeight) * blendedRootTilt).normalized();
    const Quaternion& rootTilt = t.rootTilt;
    m_rootTiltVelocity = clampMagnitude(rotationVector(
        rootTilt * m_rootTilt.conjugate()) / deltaTime, 10.0f);
    m_rootTilt = rootTilt;
    {
        const Vec3 tiltForward = rootTilt.rotate({ 1.0f, 0.0f, 0.0f });
        m_clipPelvisYaw = std::atan2(tiltForward.y, tiltForward.x);
    }
    t.accelerationTilt = (
        Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, forwardLean)
        * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -lateralLean))
            .normalized();
    const Quaternion& accelerationTilt = t.accelerationTilt;
    // A recovery plays where the body is lying, not where the capsule is
    // standing: the anchor was captured when the body settled, and the
    // clip's own root track carries it from the floor back to its feet.
    const bool risingPose = m_getUpPhase == CharacterGetUpPhase3D::Rising;
    t.targetRootOrientation = risingPose
        ? (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_getUpAnchorYaw)
            * rootTilt).normalized()
        : (heading * accelerationTilt * rootTilt).normalized();
    const Quaternion& targetRootOrientation = t.targetRootOrientation;
    m_previousTargetUp = targetRootOrientation.rotate({ 0.0f, 0.0f, 1.0f });
    // Freando, a base da referencia no plano nao e a capsula: ela freia a
    // 18 m/s2 e vem arrastada ate 0,5-0,7 m atras do corpo, e a referencia
    // presa a ela fazia o pe pousar embaixo do corpo ou atras.
    //  - Correndo e freando: uma referencia com estado, que anda no perfil
    //    de freada e fica presa ao corpo por uma coleira - a capsula de uma
    //    freada que o corpo consegue cumprir.
    //  - Nos passos de freada: o proprio corpo (a pelve medida menos o
    //    balanco do clipe); quem freia sao os pes.
    t.navigationRoot = input.rootPositionWorld;
    Vec3& navigationRoot = t.navigationRoot;
    if (m_brakeGait > 0.0f && !state.links.empty()) {
        const Vec3 pelvis = state.links.front().position;
        const Vec3 sway = heading.rotate({ m_rootOffset.x, m_rootOffset.y, 0.0f });
        const Vec3 body { pelvis.x - sway.x, pelvis.y - sway.y, 0.0f };
        if (!m_brakeReferenceValid) {
            m_brakeReferencePosition = { input.rootPositionWorld.x, input.rootPositionWorld.y, 0.0f };
            m_brakeReferenceValid = true;
        }
        m_brakeReferencePosition += Vec3 { brakeNavigationVelocity.x, brakeNavigationVelocity.y, 0.0f }
            * deltaTime;
        const Vec3 away = m_brakeReferencePosition - body;
        // Com pouca ajuda o corpo nao volta para a referencia (ninguem o
        // freia na pelve): 30 cm atras dele, todos os alvos dos pes ficavam
        // atras e ele mergulhava por cima deles. Fica presa ao corpo.
        const float lowAssistReference = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
        const float leash = BrakeReferenceLeash + (LowAssistBrakeReferenceLeash - BrakeReferenceLeash) * lowAssistReference;
        if (away.length() > leash) m_brakeReferencePosition = body + away * (leash / away.length());
        m_brakeAnchor = m_brakeStepping ? std::min(1.0f, m_brakeAnchor + deltaTime / 0.15f)
            : std::max(0.0f, m_brakeAnchor - deltaTime / 0.15f);
        const Vec3 braking = m_brakeReferencePosition + (body - m_brakeReferencePosition) * m_brakeAnchor;
        navigationRoot.x += (braking.x - navigationRoot.x) * m_brakeGait;
        navigationRoot.y += (braking.y - navigationRoot.y) * m_brakeGait;
    } else {
        m_brakeReferenceValid = false;
        m_brakeAnchor = 0.0f;
    }
    // Andando, a referencia da passada fica presa ao corpo com uma folga
    // curta no rumo do movimento: com pouca ajuda o corpo ficava para tras e
    // a capsula corria ~0,3 m a frente - cada pe pousava (a partir dela) 0,3
    // m longe demais, freava o corpo, ele inclinava para tras e caia. A
    // folga e o que a perna de apoio empurra.
    // (Cresce conforme a ajuda cai: com a ajuda cheia a pelve leva o corpo
    // ate a capsula e nada muda.)
    t.lowAssist = smoothStep((0.5f - std::clamp(input.walkingAssist, 0.0f, 1.0f)) / 0.4f);
    const float& lowAssist = t.lowAssist;
    if (GaitLeashEnabled && lowAssist > 0.0f && input.forceDrivenRoot && gaitMoving && m_brakeGait <= 0.0f
        && !getUpActive && !state.links.empty()) {
        const Vec3 pelvis = state.links.front().position;
        const Vec3 sway = heading.rotate({ m_rootOffset.x, m_rootOffset.y, 0.0f });
        const Vec3 body { pelvis.x - sway.x, pelvis.y - sway.y, 0.0f };
        const Vec3 lead { navigationRoot.x - body.x, navigationRoot.y - body.y, 0.0f };
        Vec3 leashed = lead;
        if (leashed.length() > GaitLeashMeters) leashed = leashed * (GaitLeashMeters / leashed.length());
        navigationRoot.x = body.x + lead.x + (leashed.x - lead.x) * lowAssist;
        navigationRoot.y = body.y + lead.y + (leashed.y - lead.y) * lowAssist;
    }
    // Parado depois de uma freada, a base da referencia fica onde o corpo
    // parou: a capsula vem ate 10 cm atras dele (a folga com que ela segue o
    // corpo) e, com a referencia voltando para ela, as pernas puxavam o corpo
    // para tras ate ela - ele escorregava e a recuperacao de parado dava dois
    // ou tres passos para tras. Com o pedido de movimento (ou depois de um
    // empurrao) volta para a capsula.
    if (!state.links.empty()) {
        const Vec3 pelvis = state.links.front().position;
        const Vec3 sway = heading.rotate({ m_rootOffset.x, m_rootOffset.y, 0.0f });
        const Vec3 body { pelvis.x - sway.x, pelvis.y - sway.y, 0.0f };
        // Com os pes assentados (o planejador diz que o ponto de captura ja
        // cabe na base), a base passa a frear o resto da velocidade: as
        // pernas seguram o corpo sobre os pes (tornozelo e quadril), em vez
        // de deixa-lo seguir a 0,5 m/s ate um passo de recuperacao.
        const bool settled = m_brakeGaitActive && m_brakeStepping
            && m_footwork.plan().brakeSettled;
        const bool leaving = input.intent.requestedDirectionWorld.lengthSquared() > 0.01f
            || getUpActive || input.externalPerturbation > 0.2f
            || (m_brakeGaitActive && !settled);
        if (!leaving && !m_stopHolding
            && (settled || (m_wasBrakeStepping && !m_brakeGaitActive))) {
            m_stopRoot = body;
            m_stopVelocity = { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
            m_stopRootBlend = 1.0f;
            m_stopHolding = true;
        }
        if (leaving) m_stopHolding = false;
        m_wasBrakeStepping = m_brakeGaitActive && m_brakeStepping;
        m_stopRootBlend = leaving ? std::max(0.0f, m_stopRootBlend - deltaTime / 0.25f) : m_stopRootBlend;
        if (m_stopRootBlend > 0.0f) {
            const float speed = m_stopVelocity.length();
            const float slower = std::max(0.0f, speed - StopHoldDeceleration * deltaTime);
            m_stopVelocity = speed > 1e-4f ? m_stopVelocity * (slower / speed) : Vec3 {};
            m_stopRoot += m_stopVelocity * deltaTime;
            // Presa ao corpo com a mesma folga do proxy.
            const Vec3 away = m_stopRoot - body;
            if (away.length() > StopHoldLeash) m_stopRoot = body + away * (StopHoldLeash / away.length());
            navigationRoot.x += (m_stopRoot.x - navigationRoot.x) * m_stopRootBlend;
            navigationRoot.y += (m_stopRoot.y - navigationRoot.y) * m_stopRootBlend;
        }
    }
    // Giro parado pelo clipe: a raiz anda o que a do clipe anda.
    navigationRoot += m_clipTurnRootOffset * m_clipTurnWeight;
    t.targetRootPosition = risingPose
        ? Vec3 { m_getUpAnchorWorld.x, m_getUpAnchorWorld.y,
            input.rootPositionWorld.z }
        : navigationRoot;
    Vec3& targetRootPosition = t.targetRootPosition;
    // Tropecando, abaixa o centro de massa enquanto a perna livre encurta e
    // passa por cima. A referencia estudada faz isso de forma muito grande;
    // aqui usamos apenas o necessario para manter agilidade e alcance.
    if (!risingPose) {
        targetRootPosition.z -= (StumbleDetection ? StumbleDrop : 0.065f) * m_tripRecovery;
        // Passo de quase-queda: a pelve desce (referencia Jogging Stumble:
        // o quadril ~30 cm mais baixo no passo que salva).
        if (StumbleStepTrigger > 0.0f && !m_startStepActive)
            targetRootPosition.z -= StumbleStepDrop * (m_stumbleStepping ? std::max(0.5f, m_stumbleReaction) : 0.0f);
        // Arrancada pelo passo: agacha enquanto o pe de tras vem (referencia
        // Idle To Sprint: o quadril 13 cm mais baixo nos primeiros passos).
        if (m_startStepActive)
            targetRootPosition.z -= StartStepDrop * smoothStep(m_stumbleStepSeconds / 0.12f);
        // A compressao precede a arrancada: joelhos/quadris recebem carga
        // antes da extensao, como na referencia, em vez de o peito saltar
        // imediatamente para a inclinacao final. O trote usa menos da metade
        // da descida observada; o sprint ainda fica abaixo dos ~17 cm medidos.
        targetRootPosition.z -= startupPelvisDrop;
        // Preparo do carrinho: armado, ele baixa um pouco antes de mergulhar.
        if (m_tacklePending)
            targetRootPosition.z -= TackleGatherDrop * smoothStep(m_tacklePendingSeconds / 0.15f);
    }
    // Rebase: a base da raiz troca de referencial (levantar comeca/termina)
    // ou salta num tick (teletransporte, ancora refeita). A diferenca entre
    // as duas bases nao e movimento pedido - era ela que virava milhares de
    // m/s de "velocidade" na saida do levantar do boneco solto.
    // Toda troca de fase do deitado/levantar tambem: caindo, a altura da
    // referencia passa a ser a da pelve deitada (-98 m/s num tick).
    t.referenceRebase = risingPose != m_previousRisingPose
        || m_getUpPhase != m_previousReferencePhase
        || (Vec3 { targetRootPosition.x - m_previousRootBase.x,
               targetRootPosition.y - m_previousRootBase.y, 0.0f }).length()
            > 0.25f;
    const bool& referenceRebase = t.referenceRebase;
    m_previousRisingPose = risingPose;
    m_previousReferencePhase = m_getUpPhase;
    m_previousRootBase = targetRootPosition;
    t.airbornePose = desiredState == CharacterLocomotionState3D::JumpStarting
        || desiredState == CharacterLocomotionState3D::Airborne;
    const bool& airbornePose = t.airbornePose;
    // Deslocamento da raiz vindo do clipe: a altura (a pelve sobe e desce
    // com a passada) e o balanco para o pe de apoio. No referencial do clipe
    // e no MESMO tempo das pernas, com o crossfade das juntas nas trocas.
    //
    // Era filtrado (16/s). A altura ficava atrasada em relacao as pernas, o
    // pe de apoio do alvo afundava no chao e o aterramento (mais abaixo)
    // empurrava a pelve de volta num tick: dente de serra de ate 2,6 cm em
    // 8 ms no sprint - o tremor que se via no corpo andando.
    //
    // O personagem controlado tem a capsula como ancora. No boneco solto,
    // XY vem da pelve fisica: somar balanco ali realimentaria a posicao.
    // Sua altura de repouso usa a base nominal e o ajuste de alcance dos
    // pes. Levantar precisa da trilha vertical para chegar aos pes.
    const bool planarAllowed = !risingPose && !airbornePose
        && input.controlled;
    const bool verticalAllowed = risingPose
        || (input.controlled && !airbornePose);
    const Vec3 authoredOffset {
        planarAllowed ? rootTranslation.x : 0.0f,
        planarAllowed ? rootTranslation.y : 0.0f,
        verticalAllowed ? rootTranslation.z : 0.0f };
    const Vec3 nextRootOffset = m_transitionFromRootOffset
        + (authoredOffset - m_transitionFromRootOffset) * transition
        + m_transitionRootOffsetVelocity * tangentWeight;
    m_rootOffsetVelocity = clampMagnitude((nextRootOffset - m_rootOffset) / deltaTime, 3.0f);
    m_rootOffset = nextRootOffset;
    // Deitado (fisico), a altura da referencia e a do corpo: ela ficava na de
    // pe e, ao comecar o levantar, despencava ate a do clipe em ~0,1 s
    // (referencia a -7,6 m/s). Assim o levantar parte de onde a pelve esta.
    if (input.forceDrivenRoot && m_getUpPhase == CharacterGetUpPhase3D::Settling
        && !state.links.empty()) {
        m_rootOffset.z = state.links.front().position.z - input.rootPositionWorld.z;
        m_rootOffsetVelocity.z = state.links.front().linearVelocity.z;
    }
    targetRootPosition += heading.rotate({ m_rootOffset.x, m_rootOffset.y,
        0.0f });
    targetRootPosition.z += m_rootOffset.z;
    // Carrinho: no plano, o caminho do clipe (na escala da entrada) a partir
    // da ancora, no rumo do carrinho.
    if (m_tackling && standUpClip != nullptr) {
        const Vec3 path = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_tackleYaw).rotate({
            rootTranslation.x * m_tackleDistanceScale, rootTranslation.y * m_tackleDistanceScale, 0.0f });
        targetRootPosition.x = m_tackleStart.x + path.x;
        targetRootPosition.y = m_tackleStart.y + path.y;
    }
    // Postura mais ereta (pedido do usuario, 03/10): os clipes e o controle
    // deixavam a pelve 7-10 cm abaixo da de pe, "sempre semi-agachado". A
    // pelve-alvo sobe; os pes estao travados, entao o IK estica os joelhos.
    // So de pe e andando/correndo (agachado, no ar, aterrissando e
    // levantando, nao).
    // Preparo do pulo (agachar segurando o espaco): so parado. Em movimento
    // (passada ou movimento pedido) nada - correndo, o agachamento travava
    // as pernas; ao soltar, o pulo sai no padrao.
    const bool movingForJump = gaitMoving
        || std::hypot(input.intent.requestedDirectionWorld.x, input.intent.requestedDirectionWorld.y) > 0.1f;
    const float jumpCrouchInput = movingForJump ? 0.0f : std::clamp(input.jumpCrouch, 0.0f, 1.0f);
    {
        const float bodySpeed = std::hypot(m_filteredCenterOfMassVelocity.x,
            m_filteredCenterOfMassVelocity.y);
        const float running = smoothStep((bodySpeed - 0.6f) / 1.6f);
        const float sprinting = smoothStep((bodySpeed - 4.0f) / 2.0f);
        const bool upright = !getUpActive && !airbornePose
            && (desiredState == CharacterLocomotionState3D::Idle
                || desiredState == CharacterLocomotionState3D::Turning
                || desiredState == CharacterLocomotionState3D::Walking
                || desiredState == CharacterLocomotionState3D::Running);
        const float wanted = upright
            ? (PostureLiftIdle + (PostureLiftRun - PostureLiftIdle) * running
                + (PostureLiftSprint - PostureLiftRun) * sprinting)
                * (1.0f - jumpCrouchInput)
            : 0.0f;
        m_postureLift += (wanted - m_postureLift) * (1.0f - std::exp(-deltaTime / 0.25f));
        targetRootPosition.z += m_postureLift;
    }
    // Freada: a pelve desce com a reacao (as pernas dobram pelo IK dos pes).
    // Freando pelos passos, o agachamento vem antes do pe de freada pousar:
    // um pe a 0,4-0,5 m a frente do quadril so alcanca o chao com a pelve
    // 15-20 cm mais baixa (na referencia Run To Stop ela desce 25 cm). Sem
    // isso o pe ficava pendurado procurando o chao e pousava tarde e perto.
    {
        float wanted = m_brakeStepping && !getUpActive && !airbornePose
            ? BrakeCrouchMeters
                * smoothStep((m_brakeGaitSpeed - 0.6f) / BrakeCrouchSpeedSpan)
            : 0.0f;
        // Invertendo, ele agacha para frear e girar sobre o pe, ate a
        // arrancada para o outro lado (na referencia a pelve desce 33 cm e
        // fica embaixo enquanto a velocidade troca de sentido).
        if (m_brakeReversing && !getUpActive && !airbornePose)
            wanted = std::max(wanted, PivotCrouchMeters);
        const float omega = BrakeCrouchOmega;
        m_brakeCrouchVelocity += (omega * omega * (wanted - m_brakeCrouch)
            - 2.0f * omega * m_brakeCrouchVelocity) * deltaTime;
        m_brakeCrouch = std::max(0.0f, m_brakeCrouch + m_brakeCrouchVelocity * deltaTime);
    }
    if (!getUpActive && !airbornePose) {
        // Inclinado na curva, o quadril fica mais perto do chao.
        m_telemetry.pelvisDropCurveMeters = CurveDropShare * CurveLeanHeight * (1.0f - std::cos(m_bodyCurveLean));
        m_telemetry.pelvisDropBrakeMeters = std::max(m_brakeCrouch, 0.12f * std::min(1.0f,
            m_telemetry.brakeReaction));
        targetRootPosition.z -= m_telemetry.pelvisDropCurveMeters;
        targetRootPosition.z -= m_telemetry.pelvisDropBrakeMeters;
        // Carregando o pulo: agacha (as pernas dobram pelo IK dos pes). So
        // parado: em movimento nao ha preparo (o pulo sai no padrao).
        m_jumpCrouch += (jumpCrouchInput - m_jumpCrouch)
            * (1.0f - std::exp(-12.0f * deltaTime));
        // (Mais fundo carregado: as referencias descem 19 cm no pulo normal e
        // 45 no forte.)
        targetRootPosition.z -= JumpCrouchDepth * m_jumpCrouch;
    } else {
        m_jumpCrouch = 0.0f;
        m_telemetry.pelvisDropCurveMeters = m_telemetry.pelvisDropBrakeMeters = 0.0f;
    }
    // Pouso no modo fisico: a pelve de referencia cede junto com o corpo no
    // toque (nunca fica acima dele empurrando - era isso que o fazia quicar:
    // no toque ela estava 20 cm acima e a ajuda de altura ia ao maximo), vai
    // ao agachamento previsto para o impacto, segura e sobe devagar. Direto
    // na pelve alvo: pelo crossfade da pose ele chegava tarde.
    if (input.forceDrivenRoot && !getUpActive && !airbornePose
        && m_sinceTouchdownSeconds < m_landingSettleSeconds
        && !state.links.empty()) {
        const float settle = std::clamp(m_sinceTouchdownSeconds
            / m_landingSettleSeconds, 0.0f, 1.0f);
        // TEMPTUNE LandRun: pousando correndo (o jogador segue pedindo a
        // corrida), as pernas sao mola - agacham o previsto, menos, e nao
        // acompanham o corpo que afunda (a pelve alvo descendo com ele, nada
        // segurava: 30 cm abaixo, joelho a 130 graus, e o tronco tombava).
        const float runningLanding = LandRunYield > 0.0f ? smoothStep((std::hypot(
            m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y) - 2.5f) / 2.0f)
            * smoothStep(std::hypot(input.intent.requestedVelocityWorld.x,
                input.intent.requestedVelocityWorld.y) / 1.5f) : 0.0f;
        const float planned = std::clamp(0.035f * m_landingImpactSpeed,
            0.04f, 0.22f) * (1.0f - (1.0f - LandRunDepth) * runningLanding);
        const float yielded = std::min(std::clamp(targetRootPosition.z
            - state.links.front().position.z, 0.0f, 0.25f),
            0.25f + (planned * LandRunYield - 0.25f) * runningLanding);
        if (settle < 0.45f) {
            m_landingSinkPeak = std::max({ m_landingSinkPeak, yielded,
                planned * smoothStep(settle / 0.15f) });
        }
        m_telemetry.pelvisDropLandingMeters = m_landingSinkPeak
            * (1.0f - smoothStep((settle - 0.45f) / 0.55f));
        targetRootPosition.z -= m_telemetry.pelvisDropLandingMeters;
    } else {
        m_landingSinkPeak = 0.0f;
        m_telemetry.pelvisDropLandingMeters = 0.0f;
    }

    t.localPose = {};
    RagdollAnimationPose3D& localPose = t.localPose;
    buildLocalPose(profile, m_coordinates, localPose);
    if (m_feet.size() >= 2 && m_feet[0].valid && m_feet[1].valid
        && m_feet[0].linkIndex < localPose.linkPositions.size()
        && m_feet[1].linkIndex < localPose.linkPositions.size()) {
        m_poseFeetCenterLocal = (localPose.linkPositions[m_feet[0].linkIndex]
            + localPose.linkPositions[m_feet[1].linkIndex]) * 0.5f;
        m_poseFeetCenterValid = true;
    }
    for (std::size_t side = 0; side < m_feet.size(); ++side) {
        const FootPlant& foot = m_feet[side];
        if (!foot.valid
            || foot.linkIndex >= localPose.linkOrientations.size()) {
            continue;
        }
        const Quaternion footOrientation = (rootTilt
            * localPose.linkOrientations[foot.linkIndex]).normalized();
        const Vec3 footForward = footOrientation.rotate(
            profile.links[foot.linkIndex].modelOrientation
                .conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
        const float footYaw = std::atan2(footForward.y, footForward.x);
        m_clipPelvisToFootYaw[side] = wrapAngle(
            m_clipPelvisYaw - footYaw);
    }

}

} // namespace MatterEngine
