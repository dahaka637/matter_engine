#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepOutputs(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const Vec3& centerOfMass = t.centerOfMass;
    std::array<bool, 2>& feetGroundContact = t.feetGroundContact;
    float& rootUpright = t.rootUpright;
    const float& groundHeightGuess = t.groundHeightGuess;
    const bool& getUpActive = t.getUpActive;
    const float& mass = t.mass;
    const float& rootAuthority = t.rootAuthority;
    const float& speed = t.speed;
    const Vec3& localVelocity = t.localVelocity;
    const AnimationClip3D*& standUpClip = t.standUpClip;
    const bool& authoredStartActive = t.authoredStartActive;
    const float& authoredCurveBlend = t.authoredCurveBlend;
    const int& authoredCurveKind = t.authoredCurveKind;
    float& playbackRate = t.playbackRate;
    float& strideScale = t.strideScale;
    float& terrainSpeedLimit = t.terrainSpeedLimit;
    std::vector<Vec3>& sampled = t.sampled;
    Quaternion& rootRotation = t.rootRotation;
    Vec3& rootTranslation = t.rootTranslation;
    std::array<float, 2>& contactWeights = t.contactWeights;
    float& landingBraceForward = t.landingBraceForward;
    float& landingBraceLateral = t.landingBraceLateral;
    const float& viewYawError = t.viewYawError;
    float& forwardLean = t.forwardLean;
    float& lateralLean = t.lateralLean;
    const Quaternion& targetRootOrientation = t.targetRootOrientation;
    Vec3& navigationRoot = t.navigationRoot;
    Vec3& targetRootPosition = t.targetRootPosition;
    const bool& referenceRebase = t.referenceRebase;
    const bool& standing = t.standing;
    const bool& recovering = t.recovering;
    const float& muscle = t.muscle;
    float& squaredError = t.squaredError;
    std::uint32_t& errorCount = t.errorCount;
    // The joints are never written into the articulation: pose authority is
    // what makes the guide overwrite joint positions every tick, which is
    // both the visible "bones teleporting into place" when the body turns
    // and the reason limbs pass through walls - a written position never
    // meets the solver. The motors below do the work instead, so the body
    // tracks the animation AND still collides.
    //
    // What stays carried is the root: it follows the capsule, which is what
    // keeps the character upright and moving where gameplay says. Leaving
    // root rotation to physics is what made the spine lean.
    //
    // The residual on the joints is drift correction, not posing: a blend
    // toward the target each tick, never the hard assignment the guide does
    // at authority 1. Measured tracking error against the authored pose:
    // 9.7 deg RMS at 0.10, 6.1 at 0.35, 5.4 at 0.60 - the curve is flat past
    // 0.35, so this sits just high enough to hold the pose and no higher,
    // leaving contact to win. Raising the motor gains to their ceiling moved
    // it only 9.7 -> 9.0, so this is not a strength problem.
    //
    // Contact already relaxes it further on its own: the guide is staged
    // with releaseOnInteraction, and the PhysX side caps every authority by
    // 1 - 0.9*externalInterference, so touching a wall hands that limb back
    // to the solver instead of writing it through the wall.
    RagdollAnimationConstraint3D& guide = m_output.guide;
    // Never. Every joint the guide writes is a joint the solver did not
    // compute: it shivers, it ignores contact, and during a recovery - when
    // the root is free - writing joints is what threw the body across the
    // map. The motors below are the only thing that moves this character.
    guide.poseAuthority = 0.0f;
    guide.rootTranslationAuthority = rootAuthority;
    guide.rootRotationAuthority = rootAuthority;
    guide.releaseOnInteraction = true;
    guide.rootPositionWorld = targetRootPosition;
    m_output.navigationAnchorWorld = targetRootPosition - m_ballBodyDelta
        + Vec3 { input.rootPositionWorld.x - navigationRoot.x,
            input.rootPositionWorld.y - navigationRoot.y, 0.0f };
    guide.rootOrientationWorld = targetRootOrientation;
    const Vec3 finiteDifferenceVelocity =
        (targetRootPosition - m_previousRootPosition) / deltaTime;
    guide.rootLinearVelocityWorld = finite(finiteDifferenceVelocity)
        ? finiteDifferenceVelocity : input.rootVelocityWorld;
    // Nos passos de freada a raiz e o proprio corpo: a aceleracao dela e a
    // dele, e antecipada pela ajuda empurrava o corpo para onde ele ja ia -
    // freando sobre o pe da frente, ate 2700 N para tras, e ele voltava a
    // 1,2 m/s de costas depois de parar.
    guide.rootFeedforwardShare = BrakeAnchorNoFeedforward ? 1.0f - m_brakeGait * m_brakeAnchor : 1.0f;
    const Vec3 targetAngularVelocity = rotationVector(targetRootOrientation
        * m_previousRootOrientation.conjugate()) / deltaTime;
    guide.rootAngularVelocityWorld = targetAngularVelocity;
    if (referenceRebase) {
        guide.rootLinearVelocityWorld = m_previousGuideVelocity;
        guide.rootAngularVelocityWorld = m_previousGuideAngularVelocity;
    }
    m_previousGuideVelocity = guide.rootLinearVelocityWorld;
    m_previousGuideAngularVelocity = guide.rootAngularVelocityWorld;
    m_telemetry.getUpTiltExcessDegrees = 0.0f;
    m_telemetry.getUpUprightTorque = 0.0f;
    if (getUpActive && !state.links.empty()) {
        m_telemetry.getUpAssistNewtons = 0.0f;
        if (m_tackling && m_tackleClip != nullptr) {
            // Carrinho: nada empurra o corpo pelo espaco - o mergulho e o
            // deslize sao o embalo e o atrito (a ajuda que acompanhava o
            // caminho do clipe encurtava e "arrastava" o deslize). Fica so a
            // atitude: a pelve gira para a inclinacao do clipe (sem ela o
            // embalo capotava o corpo por cima do pe apoiado). E no levantar
            // do deslize, uma ajuda para cima.
            const float mass = std::max(1.0f, profile.totalMassKg);
            const auto& pelvis = state.links.front();
            const float duration = m_tackleClip->durationSeconds;
            constexpr float Step = 1.0f / 30.0f;
            const float t0 = std::min(m_getUpClipSeconds, duration);
            const float t1 = std::min(t0 + Step, duration);
            const AnimationTransformSample3D now = clipRootAt(profile, *m_tackleClip, t0);
            const AnimationTransformSample3D next = clipRootAt(profile, *m_tackleClip, t1);
            const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_tackleYaw);
            const Quaternion wanted = (turn * now.rotationDelta).normalized();
            const Quaternion wantedNext = (turn * next.rotationDelta).normalized();
            const float rate = t1 > t0 ? m_getUpRate / Step : 0.0f;
            const Vec3 wantedSpin = rotationVector(wantedNext * wanted.conjugate()) * rate;
            const Vec3 orientationError = rotationVector(wanted * pelvis.orientation.conjugate());
            m_output.rootControlTorqueWorld = clampMagnitude(orientationError * (mass * 9.0f)
                + (wantedSpin - pelvis.angularVelocity) * (mass * 1.2f), mass * 8.0f) * TackleAttitude;
            // O impulso do mergulho: o pe de apoio ainda no chao, o corpo no
            // alto e o clipe descendo (a saida), a perna empurra na direcao
            // do carrinho - sem ele o corpo chegava ao chao com metade da
            // velocidade de entrada (o carrinho lento e curto).
            {
                const std::size_t plant = static_cast<std::size_t>(m_tackleStartSide);
                const float bodyHeight = pelvis.position.z - groundHeightGuess;
                const bool takeoff = feetGroundContact[plant] && bodyHeight > 0.40f
                    && m_getUpClipSeconds < 0.45f * duration && m_tackleExpectedRise < 0.2f;
                if (takeoff) {
                    const Vec3 direction { std::cos(m_tackleYaw), std::sin(m_tackleYaw), 0.0f };
                    const float along = dot(Vec3 { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f }, direction);
                    const float wanted = m_tackleEntrySpeed * TackleImpulseShare;
                    const float push = std::clamp((wanted - along) * 8.0f, 0.0f, 9.81f * TackleImpulseForce);
                    m_output.rootControlForceWorld += direction * (push * mass);
                    m_telemetry.getUpAssistNewtons = m_output.rootControlForceWorld.length();
                }
            }
            const float standing = profile.standingRootHeightMeters;
            const float height = pelvis.position.z - groundHeightGuess;
            // Levantando do deslize (o clipe subindo) e, da metade em diante,
            // o corpo ainda abaixo da altura do clipe (ele termina de subir).
            const float late = m_getUpClipSeconds > 0.5f * duration
                ? smoothStep((m_tackleExpectedHeight - height - 0.04f) / 0.10f) : 0.0f;
            const float recovering = std::max(late, smoothStep((m_tackleExpectedRise - 0.05f) / 0.25f)
                * smoothStep((standing * 0.92f - m_tackleExpectedHeight) / 0.10f));
            if (recovering > 0.0f) {
                const float lift = std::clamp(36.0f * (m_tackleExpectedHeight - height)
                    + 12.0f * (m_tackleExpectedRise - pelvis.linearVelocity.z), 0.0f, 9.81f * 0.5f);
                // (Propria do carrinho, nao a do slider de levantar: com ela em
                // 0, 43% dos carrinhos nao levantavam do deslize.)
                const float assist = recovering * TackleRecoveryAssist;
                m_output.rootControlForceWorld = Vec3 { 0.0f, 0.0f, lift * mass } * assist;
                m_telemetry.getUpAssistNewtons = m_output.rootControlForceWorld.length();
            }
        } else if (m_getUpPhase == CharacterGetUpPhase3D::Rising && !m_bodilyRising) {
            // Ajuda discreta do levantar: quem levanta e o corpo (os motores
            // levando as juntas ao clipe, maos/joelhos/pes empurrando o
            // chao). A pelve so recebe o que falta para acompanhar o clipe -
            // altura e endireitar -, suave, pequena (30% do peso; ate 65% se
            // o corpo emperra) e so com apoio real no chao: sem mao, joelho
            // ou pe tocando, nada (nao levita).
            const float mass = std::max(1.0f, profile.totalMassKg);
            const auto& pelvis = state.links.front();
            const float ground = groundHeightGuess;
            const float expectedHeight = profile.standingRootHeightMeters
                + rootTranslation.z;
            const Vec3 clipUp = rootRotation.rotate({ 0.0f, 0.0f, 1.0f });
            const float expectedTilt = std::acos(std::clamp(clipUp.z, -1.0f, 1.0f));
            const float expectedRise = m_getUpExpectedValid
                ? (expectedHeight - m_getUpExpectedHeight) / deltaTime : 0.0f;
            m_getUpExpectedHeight = expectedHeight;
            m_getUpExpectedTilt = expectedTilt;
            m_getUpExpectedValid = true;
            bool support = false;
            for (const RagdollInteraction3D& contact : state.interactions) {
                if (contact.normalWorld.z < 0.5f
                    || contact.linkIndex >= profile.links.size()) continue;
                const std::string& id = profile.links[contact.linkIndex].id;
                if (id.find("Hand") != std::string::npos
                    || id.find("Forearm") != std::string::npos
                    || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos) {
                    support = true;
                }
            }
            m_getUpSupportAge = support ? 0.0f
                : std::min(5.0f, m_getUpSupportAge + deltaTime);
            // So com o centro de massa sobre os apoios de verdade (maos,
            // joelhos, pes no chao): fora deles (> ~8 cm) a ajuda de altura e
            // de endireitar some - com ela o corpo ficava "segurado" como uma
            // prancha, o centro de massa ate 0,6 m fora de qualquer apoio.
            float overSupport = 1.0f;
            {
                std::vector<std::array<float, 2>> points;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f) continue;
                    points.push_back({ contact.positionWorld.x,
                        contact.positionWorld.y });
                }
                if (!points.empty()) {
                    overSupport = 1.0f - smoothStep((planarDistanceOutsideHull3D(points,
                        centerOfMass.x, centerOfMass.y) - 0.08f) / 0.20f);
                }
            }
            m_telemetry.getUpOverSupport = overSupport;
            const float contactShare = 1.0f
                - smoothStep((m_getUpSupportAge - 0.15f) / 0.25f);
            // (overSupport so medido: cortar a ajuda por ele fazia o levantar
            // falhar - os pontos de contato do backend sao o ponto dominante
            // de cada segmento, a base medida fica menor que a real.)
            (void)overSupport;
            const float supportShare = contactShare;
            const float height = pelvis.position.z - ground;
            constexpr float LiftOmega = 6.0f;
            // Cada tentativa que falha, um pouco mais de esforco (ate o
            // dobro na terceira).
            // A ajuda entra aos poucos (~1 s): o corpo comeca sozinho e ela
            // vai chegando, em vez de pegar o corpo no primeiro tick.
            m_getUpAssistRamp = smoothStep(m_getUpPhaseSeconds * m_getUpSpeedNow / 1.0f);
            const float effort = std::min(2.0f, 1.0f
                + 0.5f * static_cast<float>(m_getUpAttempt > 0 ? m_getUpAttempt - 1 : 0))
                * m_getUpAssistRamp;
            const float liftCap = 9.81f * (0.30f + 0.35f * m_getUpStruggle) * effort;
            // (Perto da altura de pe ela tambem freia: com o clipe mais
            // rapido, o embalo da subida e as pernas esticando lancavam o
            // corpo - a pelve a 1,17 m, um pulinho no fim do levantar.)
            const float brakeShare = GetUpLiftBrake * smoothStep((height - 0.70f) / 0.15f);
            const float lift = std::clamp(
                LiftOmega * LiftOmega * (expectedHeight - height)
                    + 2.0f * LiftOmega * (expectedRise - pelvis.linearVelocity.z),
                -liftCap * brakeShare, liftCap) * supportShare;
            // Endireitar: gira a pelve no sentido de ficar em pe, ate a
            // inclinacao que o clipe tem agora.
            const Vec3 up = pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const float tilt = std::acos(std::clamp(up.z, -1.0f, 1.0f));
            Vec3 axis = cross(up, { 0.0f, 0.0f, 1.0f });
            Vec3 torque {};
            if (axis.lengthSquared() > 1e-6f) {
                axis = axis.normalized();
                const float excess = std::max(0.0f, tilt - expectedTilt);
                const float spin = dot(pelvis.angularVelocity, axis);
                // Fase final (o clipe ja quase em pe, a pelve alta): mais forca
                // para endireitar - como um par pelve x pes, e a extensao do
                // quadril que traz o tronco para cima e o peso para tras, sobre
                // os pes. Com o teto de antes (~130 N.m) ele ficava ~1 s como
                // uma prancha 20-27 graus alem do clipe, o centro de massa
                // 0,4-0,5 m a frente dos pes, segurado pela ajuda.
                const float late = smoothStep((0.8f - expectedTilt) / 0.4f)
                    * smoothStep((height - 0.55f) / 0.15f);
                const float cap = mass * (1.5f + 1.5f * m_getUpStruggle
                    + GetUpPelvisLateCap * late) * effort;
                torque = axis * std::clamp((excess * 22.0f * (1.0f + GetUpPelvisLateGain * late)
                    - spin * 4.5f * (1.0f + 0.5f * GetUpPelvisLateGain * late)) * mass,
                    -cap, cap) * supportShare;
                m_telemetry.getUpTiltExcessDegrees = excess * 57.2958f;
                m_telemetry.getUpExpectedTiltDegrees = expectedTilt * 57.2958f;
            }
            // O peito tambem: a altura dele no clipe agora (cinematica direta
            // da pose do clipe), com o mesmo teto dividido.
            float chestLift = 0.0f;
            if (m_chestLink < state.links.size()) {
                RagdollAnimationPose3D clipPose;
                buildLocalPose(profile, sampled, clipPose);
                const float expectedChest = profile.standingRootHeightMeters
                    + rootTranslation.z
                    + rootRotation.rotate(clipPose.linkPositions[m_chestLink]).z;
                const float chest = state.links[m_chestLink].position.z - ground;
                m_getUpExpectedChestHeight = expectedChest;
                // E a inclinacao do tronco: de bruços o clipe ergue o tronco
                // (cabeca para cima, maos no chao); sem isso o tronco
                // apontava para o chao e o corpo se encolhia de cabeca baixa.
                {
                    const Quaternion clipChest = (rootRotation
                        * clipPose.linkOrientations[m_chestLink]).normalized();
                    const Vec3 clipChestUp = clipChest.rotate({ 0.0f, 0.0f, 1.0f });
                    const float expectedChestTilt = std::acos(
                        std::clamp(clipChestUp.z, -1.0f, 1.0f));
                    m_telemetry.getUpExpectedChestTiltDegrees = expectedChestTilt * 57.2958f;
                    const auto& chestBody = state.links[m_chestLink];
                    const Vec3 chestUp = chestBody.orientation.rotate({ 0.0f, 0.0f, 1.0f });
                    Vec3 chestAxis = cross(chestUp, { 0.0f, 0.0f, 1.0f });
                    if (chestAxis.lengthSquared() > 1e-6f) {
                        chestAxis = chestAxis.normalized();
                        const float excess = std::max(0.0f, std::acos(
                            std::clamp(chestUp.z, -1.0f, 1.0f)) - expectedChestTilt);
                        const float spin = dot(chestBody.angularVelocity, chestAxis);
                        // (Na fase final, de pe, mais forca: o peito ficava ~25
                        // graus caido alem do clipe e o corpo, de pe e
                        // dobrado no quadril, ~0,55 s ate a ajuda o erguer -
                        // o levantar de bruços. Pedido do usuario, 05/10.)
                        const float chestLate = smoothStep((0.8f - expectedTilt) / 0.4f)
                            * smoothStep((height - 0.55f) / 0.15f);
                        const float cap = mass * (0.8f + 0.8f * m_getUpStruggle + GetUpChestLateCap * chestLate) * effort;
                        m_output.chestAssistTorqueWorld = chestAxis * std::clamp(
                            (excess * 14.0f * (1.0f + GetUpChestLateGain * chestLate) - spin * 3.0f) * mass, -cap, cap)
                            * supportShare;
                    }
                }
                chestLift = std::clamp(
                    LiftOmega * LiftOmega * (expectedChest - chest)
                        + 2.0f * LiftOmega * (expectedRise
                            - state.links[m_chestLink].linearVelocity.z),
                    0.0f, liftCap) * supportShare;
                m_output.chestAssistLink = static_cast<std::uint32_t>(m_chestLink);
            }
            // O teto vale para a soma: pelve e peito dividem a mesma ajuda.
            const float total = lift + chestLift;
            const float scale = total > liftCap && total > 0.0f ? liftCap / total : 1.0f;
            // Erguendo-se (o clipe ja quase em pe): o centro de massa vai para
            // cima dos pes que estao no chao, como quem se equilibra ao
            // levantar - de leve, ate 15% do peso.
            Vec3 balance {};
            if (expectedTilt < 0.8f) {
                Vec3 feet {};
                int touching = 0;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f
                        || contact.linkIndex >= profile.links.size()) continue;
                    if (profile.links[contact.linkIndex].id.find("Foot")
                        == std::string::npos) continue;
                    feet += state.links[contact.linkIndex].position;
                    ++touching;
                }
                if (touching > 0) {
                    feet *= 1.0f / static_cast<float>(touching);
                    Vec3 offset { feet.x - centerOfMass.x, feet.y - centerOfMass.y, 0.0f };
                    Vec3 velocity = m_filteredCenterOfMassVelocity;
                    velocity.z = 0.0f;
                    // (Esta continua fora da base: e ela que traz o centro de
                    // massa de volta para cima dos pes.)
                    balance = clampMagnitude(offset * 30.0f - velocity * 8.0f,
                        9.81f * 0.22f * effort) * (mass * contactShare);
                }
            }
            m_output.rootControlForceWorld = Vec3 { balance.x, balance.y,
                lift * scale * mass };
            m_output.chestAssistForceWorld = { 0.0f, 0.0f, chestLift * scale * mass };
            // Endireitar e um par interno, como um musculo: a pelve (e o
            // peito) giram para cima e a reacao vai para os segmentos que
            // estao apoiados no chao, pela forca de apoio de cada um. Assim
            // ele so sustenta o que os apoios sustentariam - com o centro de
            // massa fora deles, e o pe ou a mao que cede. Como torque externo
            // ele segurava o corpo como uma prancha inclinada ~35 graus: a
            // ajuda ficava evidente.
            {
                std::vector<std::pair<std::uint32_t, float>> supports;
                float supportSum = 0.0f;
                for (const RagdollInteraction3D& contact : state.interactions) {
                    if (contact.normalWorld.z < 0.5f || contact.linkIndex == 0
                        || contact.linkIndex == m_chestLink
                        || contact.linkIndex >= profile.links.size()) continue;
                    const float weight = std::max(0.001f,
                        contact.normalImpulseNewtonSeconds);
                    supportSum += weight;
                    auto existing = std::find_if(supports.begin(), supports.end(),
                        [&](const auto& item) { return item.first == contact.linkIndex; });
                    if (existing != supports.end()) existing->second += weight;
                    else supports.push_back({ contact.linkIndex, weight });
                }
                if (supportSum <= 0.0f) {
                    torque = {};
                    m_output.chestAssistTorqueWorld = {};
                } else {
                    const Vec3 reaction = -(torque + m_output.chestAssistTorqueWorld);
                    for (const auto& [link, weight] : supports) {
                        m_output.assistReactionLinks.push_back(link);
                        m_output.assistReactionTorquesWorld.push_back(
                            reaction * (weight / supportSum));
                    }
                }
            }
            m_output.rootControlTorqueWorld = torque;
            m_telemetry.getUpUprightTorque = torque.length();
            m_telemetry.getUpAssistNewtons = total * scale * mass;
            // A ajuda para levantar (slider do laboratorio, 0 = nenhuma; o
            // TEMPTUNE multiplica, para diagnostico).
            const float getUpAssistScale = GetUpAssistScale * std::clamp(input.getUpAssist, 0.0f, 2.0f);
            if (getUpAssistScale != 1.0f) {
                m_output.rootControlForceWorld *= getUpAssistScale;
                m_output.rootControlTorqueWorld *= getUpAssistScale;
                m_output.chestAssistForceWorld *= getUpAssistScale;
                m_output.chestAssistTorqueWorld *= getUpAssistScale;
                for (Vec3& reaction : m_output.assistReactionTorquesWorld) reaction *= getUpAssistScale;
                m_telemetry.getUpAssistNewtons *= getUpAssistScale;
            }
        }
        // Rolando: a ajuda gira o corpo em torno da coluna, no sentido da
        // barriga para o chao (vezes o slider da ajuda para levantar; a 0% so
        // a perna e o braco de cima rolam o corpo).
        if (m_getUpPhase == CharacterGetUpPhase3D::Settling && m_getUpRolling
            && m_chestLink < state.links.size()) {
            const float mass = std::max(1.0f, profile.totalMassKg);
            const Vec3 spine = (state.links[m_chestLink].position - state.links.front().position).normalized();
            const float spin = dot(state.links.front().angularVelocity, spine) * m_getUpRollSign;
            const float wanted = 2.0f;  // rad/s em torno da coluna
            const float ramp = smoothStep(m_getUpRollSeconds / 0.3f);
            const float torque = std::clamp((wanted - spin) * 0.6f, 0.0f, 1.0f) * GetUpRollTorque * mass * ramp
                * GetUpAssistScale * std::clamp(input.getUpAssist, 0.0f, 2.0f);
            m_output.rootControlTorqueWorld = spine * (m_getUpRollSign * torque);
        }
    } else if (!state.links.empty()) {
        // One path now: the body is always the physical one, so it always
        // gets a root controller. How much help it gets is the only thing
        // that changes, and that is what physicsBlend fades.
        Quaternion rotationError = targetRootOrientation
            * state.links.front().orientation.conjugate();
        const Vec3 orientationError = rotationVector(rotationError);
        const Vec3 angularVelocityError = targetAngularVelocity
            - state.links.front().angularVelocity;
        const float mass = std::max(1.0f, profile.totalMassKg);
        const Vec3 torque = orientationError * (mass * 10.0f)
            + angularVelocityError * (mass * 1.45f);
        m_output.rootControlTorqueWorld = clampMagnitude(
            torque, mass * 13.0f) * rootAuthority;

        // Capture-point balance assist, faded in by exactly how much the
        // body has been handed over to physics. Capped at half: while it is
        // going down it is supposed to be losing the argument at least
        // partly, and a fully assisted fall just looks like the character
        // refusing to fall.
        constexpr float MaximumAssistShare = 0.5f;
        const float assistShare = std::min(MaximumAssistShare,
            std::clamp(m_physicsBlend, 0.0f, 1.0f));
        Vec3 supportCenter {};
        float supportWeight = 0.0f;
        for (const FootPlant& foot : m_feet) {
            if (!foot.planted) continue;
            supportCenter += foot.lockedPositionWorld;
            supportWeight += 1.0f;
        }
        if (assistShare > 0.001f && supportWeight > 0.0f) {
            supportCenter *= 1.0f / supportWeight;
            const float comHeight = std::max(0.35f,
                centerOfMass.z - supportCenter.z);
            const float omega = std::sqrt(9.81f / comHeight);
            Vec3 capturePoint = centerOfMass
                + m_filteredCenterOfMassVelocity / omega;
            capturePoint.z = supportCenter.z;
            Vec3 assistForce = (supportCenter - capturePoint)
                * (mass * omega * 0.85f);
            assistForce.z = 0.0f;
            m_output.rootControlForceWorld = clampMagnitude(assistForce,
                mass * 9.81f * 0.16f) * assistShare;

            const Vec3 rootUp = state.links.front()
                .orientation.rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 tiltError = cross(rootUp, Vec3 { 0.0f, 0.0f, 1.0f });
            Vec3 planarAngularVelocity = state.links.front().angularVelocity;
            planarAngularVelocity.z = 0.0f;
            Vec3 tiltTorque = tiltError * (mass * 9.5f)
                - planarAngularVelocity * (mass * 0.85f);
            m_output.rootControlTorqueWorld += clampMagnitude(
                tiltTorque, mass * 1.05f) * assistShare;
        }
    }
    m_previousRootPosition = targetRootPosition;
    m_previousRootOrientation = targetRootOrientation;
    m_output.gravityCompensationEnabled = muscle > 0.0001f;
    m_output.physicsBlend = m_physicsBlend;
    // Carrinho: com o corpo baixo no clipe (mergulho e deslize) ele escorrega
    // na grama; subindo do chao, o atrito volta (pes e maos empurram).
    m_output.bodyFrictionScale = 1.0f;
    if (m_tackling && !state.links.empty()) {
        // A velocidade ao tocar o chao (a pelve chegando baixo); antes disso
        // (o joelho pode tocar antes), a de entrada com o impulso.
        const auto& pelvis = state.links.front();
        const float bodyHeight = pelvis.position.z - groundHeightGuess;
        if (m_tackleLandSpeed < 0.0f && bodyHeight < TackleSlideHeight)
            m_tackleLandSpeed = std::hypot(pelvis.linearVelocity.x, pelvis.linearVelocity.y);
        const float speed = std::max(0.5f, m_tackleLandSpeed >= 0.0f ? m_tackleLandSpeed
            : m_tackleEntrySpeed * TackleImpulseShare);
        const float distance = TackleSlideA * std::pow(speed, TackleSlideP);
        const float mu = speed * speed / (2.0f * 9.81f * std::max(0.1f, distance));
        const float slideScale = std::clamp(mu / TackleBaseFriction, 0.10f, 1.0f);
        const float slide = smoothStep((0.70f - m_tackleExpectedHeight) / 0.15f)
            * (1.0f - smoothStep((m_tackleExpectedRise - 0.10f) / 0.20f));
        m_output.bodyFrictionScale = 1.0f - (1.0f - slideScale) * slide;
    }

    m_telemetry.localVelocity = localVelocity;
    m_telemetry.speedMetersPerSecond = speed;
    m_telemetry.facingYawRadians = m_facingYaw;
    m_telemetry.viewYawErrorRadians = viewYawError;
    m_telemetry.gaitDirection = m_gaitDirection;
    m_telemetry.jumpKind = m_jumpKind;
    m_telemetry.flightPhase = m_flightPhase;
    m_telemetry.torsoTwistRadians = m_torsoTwist;
    m_telemetry.headTurnRadians = m_headTurn;
    m_telemetry.turningRateRadiansPerSecond = m_turningRate;
    m_telemetry.forwardLeanRadians = forwardLean;
    m_telemetry.lateralLeanRadians = lateralLean;
    m_telemetry.authoredCurveBlend = authoredCurveBlend;
    m_telemetry.clipFootContact = contactWeights;
    m_telemetry.authoredCurveKind = authoredCurveKind;
    m_telemetry.authoredStartActive = authoredStartActive;
    m_telemetry.landingBraceForwardRadians = landingBraceForward;
    m_telemetry.landingBraceLateralRadians = landingBraceLateral;
    m_telemetry.cyclePhase = m_cyclePhase;
    m_telemetry.waitingLeg = m_waitingLeg;
    m_telemetry.playbackRate = playbackRate;
    m_telemetry.strideScale = strideScale;
    m_telemetry.terrainSlope = m_terrainSlope;
    // Curva fechada: o teto de velocidade que cabe na curva pedida - a
    // aceleracao lateral que as pernas fazem sobre a taxa com que o pedido
    // gira (a camera girando com o comando para a frente). A 7,5 m/s, virar a
    // 180 graus/s pede 2,4 g: a ajuda puxava o corpo para o rumo novo, os pes
    // nao acompanhavam (o de dentro era arrastado no chao) e ele caia. Quem
    // corre de verdade desacelera para fechar a curva. O jogo corta o pedido
    // por este teto (o mesmo do terreno); ao sair da curva ele sobe no ritmo
    // de uma arrancada. Um degrau no pedido (troca de tecla) nao e curva: a
    // taxa crua e limitada antes do filtro.
    {
        const Vec3 request { input.intent.requestedDirectionWorld.x,
            input.intent.requestedDirectionWorld.y, 0.0f };
        const bool requesting = request.lengthSquared() > 0.01f;
        const float yaw = requesting ? std::atan2(request.y, request.x) : m_requestYaw;
        const float rate = requesting && m_requestYawValid
            ? std::clamp(wrapAngle(yaw - m_requestYaw) / std::max(1e-4f, deltaTime),
                -RequestTurnRateClamp, RequestTurnRateClamp) : 0.0f;
        m_requestYaw = yaw;
        m_requestYawValid = requesting;
        m_requestTurnRate += (rate - m_requestTurnRate)
            * (1.0f - std::exp(-deltaTime / std::max(0.01f, RequestTurnFilterSeconds)));
        float curveLimit = 1000.0f;
        const float lateral = CurveSpeedAccelerationLow + (CurveSpeedAcceleration - CurveSpeedAccelerationLow)
            * std::clamp(input.walkingAssist, 0.0f, 1.0f);
        if (CurveSpeedAcceleration > 0.0f && input.forceDrivenRoot && !getUpActive
            && std::abs(m_requestTurnRate) > 0.3f)
            curveLimit = std::max(CurveSpeedMinimum, lateral / std::abs(m_requestTurnRate));
        m_curveSpeedCap = curveLimit < m_curveSpeedCap ? curveLimit
            : std::min(curveLimit, m_curveSpeedCap + CurveSpeedRecovery * deltaTime);
        if (m_curveSpeedCap < 100.0f) terrainSpeedLimit = std::min(terrainSpeedLimit, m_curveSpeedCap);
    }
    // Arrancada pelo passo: a ajuda dosada - a capsula (e o pedido) sobe no
    // ritmo de quem arranca (referencia: 2,2 m/s aos 0,37 s), nao a 14 m/s2.
    if (m_startStepActive)
        terrainSpeedLimit = std::min(terrainSpeedLimit, StartSpeedBase + StartSpeedRate * m_stumbleStepSeconds);
    else if (m_startDoseCap < 100.0f)
        terrainSpeedLimit = std::min(terrainSpeedLimit, m_startDoseCap);
    m_telemetry.terrainSpeedLimit = terrainSpeedLimit;
    m_telemetry.tripRecovery = m_tripRecovery;
    m_telemetry.stumbleExcess = m_stumblePitch - m_stumbleBaseline;
    m_telemetry.stumbleRate = m_stumblePitchRate;
    m_telemetry.stumbleDecel = m_stumbleDecel;
    m_telemetry.stumbleRisk = m_stumbleRisk;
    m_telemetry.stumbleStepping = m_stumbleStepping;
    const float longitudinal = std::abs(localVelocity.x);
    const float lateral = std::abs(localVelocity.y);
    const float total = std::max(0.0001f, longitudinal + lateral);
    m_telemetry.forwardWeight = localVelocity.x >= 0.0f
        ? longitudinal / total : 0.0f;
    m_telemetry.backwardWeight = localVelocity.x < 0.0f
        ? longitudinal / total : 0.0f;
    m_telemetry.leftWeight = localVelocity.y >= 0.0f
        ? lateral / total : 0.0f;
    m_telemetry.rightWeight = localVelocity.y < 0.0f
        ? lateral / total : 0.0f;
    m_telemetry.poseAuthority = m_output.guide.poseAuthority;
    m_telemetry.rootAuthority = rootAuthority;
    m_telemetry.rootRotationAuthority = m_output.guide.rootRotationAuthority;
    m_telemetry.muscleAuthority = muscle;
    m_telemetry.reactionStrength = m_reactionStrength;
    m_telemetry.jointErrorRmsDegrees = errorCount > 0
        ? std::sqrt(squaredError / static_cast<float>(errorCount))
            * 180.0f / Pi
        : 0.0f;
    m_telemetry.rootUpright = rootUpright;
    m_telemetry.fallOrientation = m_fallOrientation;
    m_telemetry.getUpPhase = m_getUpPhase;
    m_telemetry.getUpProgress = m_getUpPhase
            == CharacterGetUpPhase3D::Rising
        ? std::clamp(m_getUpClipSeconds / std::max(0.05f,
            standUpClip ? standUpClip->durationSeconds : 1.6f), 0.0f, 1.0f) : 0.0f;
    m_telemetry.getUpAttempt = m_getUpAttempt;
    m_telemetry.fallReflex = m_fallReflex;
    m_telemetry.fallCommit = m_fallCommit;
    m_telemetry.getUpRate = m_getUpPhase == CharacterGetUpPhase3D::Rising
        ? m_getUpRate : 0.0f;
    m_telemetry.getUpStruggle = m_getUpStruggle;
    m_telemetry.muscleTone = m_getUpPhase == CharacterGetUpPhase3D::Settling
        ? m_settleTone : 1.0f;
    m_telemetry.getUpAssistRamp = m_getUpPhase == CharacterGetUpPhase3D::Rising
        ? m_getUpAssistRamp : 0.0f;
    m_telemetry.settleGather = m_getUpPhase == CharacterGetUpPhase3D::Settling
        ? m_settleGather : m_getUpPhase == CharacterGetUpPhase3D::Rising ? 1.0f : 0.0f;
    const bool bodilyTelemetry = m_bodilyRising && m_getUpPhase == CharacterGetUpPhase3D::Rising;
    m_telemetry.bodilyStage = bodilyTelemetry ? m_bodilyGetUp.telemetry().stage : BodilyGetUpStage3D::None;
    m_telemetry.bodilyStageProgress = bodilyTelemetry ? m_bodilyGetUp.telemetry().stageProgress : 0.0f;
    m_telemetry.bodilyRootError = bodilyTelemetry ? m_bodilyGetUp.telemetry().rootError : 0.0f;
}

} // namespace MatterEngine
