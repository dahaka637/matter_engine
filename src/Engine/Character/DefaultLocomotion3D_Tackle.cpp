#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

bool DefaultLocomotion3D::stepTackle(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const RagdollLinkDefinition3D& rootLink = *t.rootLink;
    const float& dt = t.dt;
    const std::size_t& links = t.links;
    const float& viewYaw = t.viewYaw;
    const Vec3& viewForward = t.viewForward;
    const Vec3& velocity = t.velocity;
    const float& speed = t.speed;
    // Arrancada agachada (startCrouchSprint): o clipe no rumo pedido, a raiz
    // dele comecando sobre a capsula (que esta embaixo do corpo agachado); a
    // pose do corpo entra pela inercia de quem chama.
    if (m_crouchStartPending) {
        m_crouchStartPending = false;
        if (animations.crouchToSprint != nullptr && animations.crouchToSprint->durationSeconds > 0.05f) {
            const AnimationClip3D& clip = *animations.crouchToSprint;
            m_tackleClip = &clip;
            m_crouchStarting = true;
            m_tackleFromAir = false;
            m_tackleRate = 1.0f;
            // (O clipe sai a ~4,4 m/s: trotando, o caminho vai a velocidade
            // do trote.)
            m_tackleScale = m_crouchStartSprint ? 1.0f
                : std::clamp(defaultLocomotionSpeed3D(s, 0.0f, false) / 4.4f, 0.5f, 1.0f);
            m_tackleYaw = m_crouchStartYaw;
            // O fim: no pico da velocidade da raiz (no plano).
            {
                float best = 0.0f;
                m_crouchExitTime = clip.durationSeconds;
                constexpr float Step = 1.0f / 60.0f;
                for (float t = Step; t <= clip.durationSeconds + 1e-4f; t += Step) {
                    const AnimationTransformSample3D a = rootAt(profile, clip, t - Step, false, s.smoothClipSampling);
                    const AnimationTransformSample3D b = rootAt(profile, clip, t, false, s.smoothClipSampling);
                    const float v = std::hypot(b.translationOffsetMeters.x - a.translationOffsetMeters.x,
                        b.translationOffsetMeters.y - a.translationOffsetMeters.y) / Step;
                    if (v >= best * 0.995f && t > 0.5f * clip.durationSeconds) { best = std::max(best, v); m_crouchExitTime = t; }
                    best = std::max(best, v);
                }
            }
            // O comeco: o tempo em que a pelve do clipe esta na altura da do
            // corpo (o levantar entrega ja subindo; no quadro 0 o clipe a
            // levava de volta ao agachado mais fundo).
            m_tackleTime = 0.0f;
            if (m_seed.feetValid || m_seedPending) {
                const float bodyHeight = m_seed.pelvisWorld.z - input.feetPositionWorld.z;
                for (float t = 0.0f; t <= std::min(CrouchStartLatest, m_crouchExitTime - 0.1f); t += 1.0f / 120.0f) {
                    const AnimationTransformSample3D r = rootAt(profile, clip, t, false, s.smoothClipSampling);
                    const float height = profile.standingRootHeightMeters + rootLink.modelPosition.z + r.translationOffsetMeters.z;
                    m_tackleTime = t;
                    if (height >= bodyHeight) break;
                }
            }
            const AnimationTransformSample3D r0 = rootAt(profile, clip, m_tackleTime, false, s.smoothClipSampling);
            const Vec3 first = yawRotation(m_tackleYaw).rotate({ r0.translationOffsetMeters.x * m_tackleScale,
                r0.translationOffsetMeters.y * m_tackleScale, 0.0f });
            m_tackleStart = { input.feetPositionWorld.x - first.x, input.feetPositionWorld.y - first.y, 0.0f };
            m_tacklePrevious = { input.feetPositionWorld.x, input.feetPositionWorld.y, input.feetPositionWorld.z };
            m_tackleFrom.clear();
            m_footLocks = {};
            m_tackleEntryVelocity = {};
            m_tackleExit = 1.0f;
            m_crouchPreviousFeet = {};
        }
    }
    // Carrinho no ar (a voadora): Ctrl com Shift + W, os tres juntos em
    // qualquer ordem (uma por voo). (O carrinho do pouso so com o Ctrl saiu
    // a pedido do usuario, 05/10 noite: a voadora ja o controla.)
    if (input.grounded) m_airTackleSpent = false;
    // O pouso previsto (balistico) da base da capsula.
    t.touchdown = 1e9f;
    float& touchdown = t.touchdown;
    if (!input.grounded && input.groundAt) {
        const GroundProbeResult3D probe = input.groundAt(input.feetPositionWorld);
        if (probe.hasSurface) {
            const float rise = input.velocityWorld.z;
            const float fall = std::sqrt(rise * rise
                + 2.0f * Gravity * std::max(0.0f, input.feetPositionWorld.z - probe.pointWorld.z));
            touchdown = (rise + fall) / Gravity;
        }
    }
    if (!input.grounded && input.tackleHeld && input.airTackleCombo && s.airTackle && !m_airTackleSpent
        && input.tackleEnabled && m_tackleClip == nullptr
        && m_airTackleClip == nullptr && m_tackleCooldown <= 0.0f && m_airTime > 0.05f) {
        m_airTackleSpent = true;
        const Vec3 flat = planar(input.velocityWorld);
        const float lateral = flat.lengthSquared() > 1e-4f ? std::sin(wrapAngle(std::atan2(flat.y, flat.x) - viewYaw)) : 0.0f;
        ++m_tackleCount;
        const int chosen = chooseTackleClip3D(m_tackles, m_tackleShape, m_viewTurnRate, lateral,
            (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u) ^ static_cast<std::uint32_t>(m_idleTime * 1000.0f));
        if (chosen >= 0) {
            const std::size_t k = static_cast<std::size_t>(chosen);
            m_airTackleClip = m_tackles[k];
            // Do mergulho do clipe (a pose do ar anda ate a fase deitada).
            m_airTackleTime = std::clamp(m_tackleShape[k].diveStart, 0.0f, m_airTackleClip->durationSeconds);
            m_airTackleTarget = std::clamp(std::max(m_tackleShape[k].diveStart, m_tackleShape[k].lowStart - 0.02f),
                0.0f, m_airTackleClip->durationSeconds);
            m_airTackleYaw = flat.length() > 1.0f ? std::atan2(flat.y, flat.x) : viewYaw;
            m_airTackleWeight = 0.0f;
            // Pedida quase pousando, a mistura cabe ate o toque (e o chao
            // continua o clipe de onde ela estiver). O mergulho no ritmo do
            // clipe esticado.
            m_airTackleBlendSeconds = std::clamp(0.85f * touchdown, AirTackleBlendMinimum, 0.12f);
            m_airTackleElapsed = 0.0f;
            m_airTackleDiveSeconds = std::max(0.05f, AirTackleDiveStretch * std::max(0.0f, m_airTackleTarget - m_airTackleTime));
            // Ritmo e percurso pela velocidade do voo (como no pouso; o
            // fisico os usa se ele bater em algo no ar).
            const float ratio = std::clamp(std::max(2.0f, flat.length()) / m_tackleClipSpeed[k], 0.4f, 1.3f);
            m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
            m_tackleScale = ratio / m_tackleRate;
            m_tackleEntryVelocity = flat;
        }
    }
    // O pouso da voadora: o deslize do carrinho a partir da fase deitada, com
    // a velocidade que ele trazia; a entrada mistura da pose do ar. Sem a
    // aterrissagem amortecida (o deslize e o pouso).
    if (m_airTackleClip != nullptr && input.grounded) {
        const AnimationClip3D& clip = *m_airTackleClip;
        std::size_t k = 0;
        for (std::size_t j = 0; j < m_tackles.size(); ++j) if (m_tackles[j] == &clip) k = j;
        m_airTackleClip = nullptr;
        m_airTackleWeight = 0.0f;
        m_tackleClip = &clip;
        m_tackleFromAir = true;
        const Vec3 flat = planar(input.velocityWorld);
        const float speedIn = std::max(2.0f, flat.length());
        const float ratio = std::clamp(speedIn / m_tackleClipSpeed[k], 0.4f, 1.3f);
        m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
        m_tackleScale = ratio / m_tackleRate;
        m_tackleYaw = m_airTackleYaw;
        // (Do tempo em que a pose do ar estava: o clipe continua.)
        m_tackleTime = std::clamp(m_airTackleTime, 0.0f, clip.durationSeconds);
        m_tackleEnterStart = m_tackleTime;
        {
            float seconds = 0.0f, drop = 0.0f;
            int count = 0;
            for (std::size_t j = 0; j < m_tackles.size(); ++j) {
                const TackleClipShape3D& c = m_tackleShape[j];
                if (m_tackles[j] == nullptr || c.lowEnd <= c.lowStart || c.lowEntrySpeed < 0.5f) continue;
                seconds += c.lowEnd - c.lowStart;
                drop += std::clamp(1.0f - c.lowExitSpeed / c.lowEntrySpeed, 0.0f, 0.95f);
                ++count;
            }
            m_tackleCommonLowSeconds = count > 0 ? seconds / static_cast<float>(count) : 0.0f;
            m_tackleCommonDrop = count > 0 ? drop / static_cast<float>(count) : 0.0f;
        }
        m_tackleEntrySpeedSlide = speedIn;
        m_tacklePathDirection = flat.length() > 0.5f ? flat.normalized() : Vec3 { std::cos(m_tackleYaw), std::sin(m_tackleYaw), 0.0f };
        const AnimationTransformSample3D r0 = rootAt(profile, clip, m_tackleTime, false, s.smoothClipSampling);
        const Vec3 first = yawRotation(m_tackleYaw).rotate({ r0.translationOffsetMeters.x * m_tackleScale,
            r0.translationOffsetMeters.y * m_tackleScale, 0.0f });
        // (A pose do tick anterior: m_output.valid ja foi zerado neste update.)
        const bool previous = m_output.coordinates.size() == links;
        const Vec3 from = previous ? m_output.rootPosition
            : input.feetPositionWorld + Vec3 { 0.0f, 0.0f, profile.standingRootHeightMeters };
        m_tackleStart = { from.x - first.x, from.y - first.y, 0.0f };
        m_tacklePlanar = first;
        m_tacklePrevious = from;
        m_tackleFrom.clear();
        m_footLocks = {};
        m_tackleFromRoot = from;
        m_tackleEntryVelocity = flat;
        m_tackleFromOrientation = m_output.rootOrientation;
        m_tackleExit = 1.0f;
        // A pose do voo e a velocidade dela (do tick anterior a ele).
        if (previous) {
            const float step = 1.0f / std::max(1e-4f, dt);
            const bool known = m_airPreviousCoordinates.size() == links;
            std::vector<Vec3> velocities(links);
            if (known)
                for (std::size_t i = 1; i < links; ++i)
                    velocities[i] = (m_output.coordinates[i] - m_airPreviousCoordinates[i]) * step;
            Vec3 rootVelocity = known ? (m_output.rootPosition - m_airPreviousRoot) * step : input.velocityWorld;
            Vec3 spin {};
            if (known) {
                const Quaternion turn = (m_output.rootOrientation * m_airPreviousOrientation.conjugate()).normalized();
                const float sine = std::sqrt(turn.x * turn.x + turn.y * turn.y + turn.z * turn.z);
                if (sine > 1e-6f)
                    spin = Vec3 { turn.x, turn.y, turn.z }
                        * (2.0f * std::atan2(sine, std::abs(turn.w)) / sine * (turn.w < 0.0f ? -1.0f : 1.0f) * step);
            }
            m_landTackleInertia.start(m_output.coordinates, velocities, m_output.rootPosition, m_output.rootOrientation,
                rootVelocity, spin, AirTackleLandHalfLife, AirTackleLandSeconds);
        }
        // (O pouso e o deslize: sem a aterrissagem amortecida depois.)
        m_airTime = 0.0f;
        m_landTime = -1.0f;
        m_landingPrep = 0.0f;
        m_landingPrepRate = 0.0f;
    }
    // Carrinho: correndo para a frente do olhar, o pedido o comeca ja, com o
    // clipe do pe que vai pousar.
    if (m_tackleClip == nullptr && input.tackleRequest && input.tackleEnabled && input.grounded
        && m_tackleCooldown <= 0.0f && !m_output.coordinates.empty()
        && dot(velocity, viewForward) >= s.tackleMinimumSpeed) {
        // Qual: o que vira o corpo para o lado do giro da camera; parada, para
        // o lado da corrida; senao, ao acaso (pedido do usuario, 05/10 noite).
        const Vec3 travel = planar(input.requestedVelocityWorld).lengthSquared() > 0.01f
            ? planar(input.requestedVelocityWorld) : velocity;
        const float lateral = travel.lengthSquared() > 1e-4f
            ? std::sin(wrapAngle(std::atan2(travel.y, travel.x) - viewYaw)) : 0.0f;
        ++m_tackleCount;
        const int chosen = chooseTackleClip3D(m_tackles, m_tackleShape, m_viewTurnRate, lateral,
            (input.varietySeed * 2654435761u) ^ (m_tackleCount * 40503u) ^ static_cast<std::uint32_t>(m_idleTime * 1000.0f));
        if (chosen >= 0) {
            const std::size_t k = static_cast<std::size_t>(chosen);
            m_tackleClip = m_tackles[k];
            // Ritmo e percurso pela velocidade de entrada (como no fisico).
            const float ratio = std::clamp(dot(velocity, viewForward) / m_tackleClipSpeed[k], 0.4f, 1.3f);
            m_tackleRate = std::clamp(std::sqrt(ratio), 0.75f, 1.12f);
            m_tackleScale = ratio / m_tackleRate;
            // Os dois deslizam o mesmo (pela velocidade de entrada): o
            // caminho de cada um vai ao deslize medio deles.
            {
                float sum = 0.0f;
                int count = 0;
                for (std::size_t j = 0; j < m_tackles.size(); ++j)
                    if (m_tackles[j] != nullptr && m_tackleShape[j].slideMeters > 0.5f) { sum += m_tackleShape[j].slideMeters; ++count; }
                if (count > 0 && m_tackleShape[k].slideMeters > 0.5f && s.tackleEqualSlide)
                    m_tackleScale *= (sum / static_cast<float>(count)) / m_tackleShape[k].slideMeters;
            }
            m_tackleYaw = viewYaw;
            m_tackleTime = 0.0f;
            m_tackleEnterStart = 0.0f;
            m_tackleFromAir = false;
            m_landTackleInertia.stop();
            // O deslize comum (ver o ramo do carrinho): a fase deitada media e
            // a fracao media que a velocidade cai nela.
            {
                float seconds = 0.0f, drop = 0.0f;
                int count = 0;
                for (std::size_t j = 0; j < m_tackles.size(); ++j) {
                    const TackleClipShape3D& c = m_tackleShape[j];
                    if (m_tackles[j] == nullptr || c.lowEnd <= c.lowStart || c.lowEntrySpeed < 0.5f) continue;
                    seconds += c.lowEnd - c.lowStart;
                    drop += std::clamp(1.0f - c.lowExitSpeed / c.lowEntrySpeed, 0.0f, 0.95f);
                    ++count;
                }
                m_tackleCommonLowSeconds = count > 0 ? seconds / static_cast<float>(count) : 0.0f;
                m_tackleCommonDrop = count > 0 ? drop / static_cast<float>(count) : 0.0f;
            }
            m_tackleEntrySpeedSlide = std::max(2.0f, dot(velocity, viewForward));
            m_tacklePathDirection = viewForward;
            const AnimationTransformSample3D r0 = rootAt(profile, *m_tackleClip, 0.0f, false, s.smoothClipSampling);
            const Vec3 first = yawRotation(m_tackleYaw).rotate({ r0.translationOffsetMeters.x * m_tackleScale,
                r0.translationOffsetMeters.y * m_tackleScale, 0.0f });
            m_tackleStart = { m_output.rootPosition.x - first.x, m_output.rootPosition.y - first.y, 0.0f };
            m_tacklePlanar = first;
            m_tacklePrevious = m_output.rootPosition;
            m_tackleFrom = m_output.coordinates;
            m_footLocks = {};
            m_tackleFromRoot = m_output.rootPosition;
            m_tackleEntryVelocity = velocity;
            m_tackleFromOrientation = m_output.rootOrientation;
            m_tackleExit = 1.0f;
        }
    }
    if (m_tackleClip != nullptr) {
        // O carrinho: o clipe no tempo (no ritmo da entrada), a pelve no
        // caminho da raiz dele, escalado; a capsula vai embaixo (laboratorio).
        const AnimationClip3D& clip = *m_tackleClip;
        // Os dois carrinhos deslizam igual (pedido do usuario, 05/10 noite):
        // o deslocamento no plano e o mesmo modelo para os dois, pela
        // velocidade de entrada - no mergulho ela fica; deitado freia a mesma
        // fracao no mesmo tempo (a pose do clipe estica/encolhe nessa fase);
        // levantando segue o perfil do clipe dali. O clipe da a pose, a altura
        // e o rumo do caminho.
        const TackleClipShape3D* shape = nullptr;
        for (std::size_t k = 0; k < m_tackles.size(); ++k) if (m_tackles[k] == &clip) shape = &m_tackleShape[k];
        const bool equalSlide = !m_crouchStarting && s.tackleEqualSlide && shape != nullptr
            && shape->lowEnd > shape->lowStart && m_tackleCommonLowSeconds > 0.05f;
        float timeRate = m_tackleRate;
        if (equalSlide && m_tackleTime >= shape->lowStart && m_tackleTime < shape->lowEnd)
            timeRate *= (shape->lowEnd - shape->lowStart) / m_tackleCommonLowSeconds;
        const float before = m_tackleTime;
        m_tackleTime = std::min(clip.durationSeconds, m_tackleTime + dt * timeRate);
        const AnimationTransformSample3D r = rootAt(profile, clip, m_tackleTime, false, s.smoothClipSampling);
        const Quaternion tackleYaw = yawRotation(m_tackleYaw);
        Vec3 path = tackleYaw.rotate({ r.translationOffsetMeters.x * m_tackleScale,
            r.translationOffsetMeters.y * m_tackleScale, 0.0f });
        if (equalSlide) {
            const AnimationTransformSample3D previous = rootAt(profile, clip, before, false, s.smoothClipSampling);
            Vec3 clipStep { r.translationOffsetMeters.x - previous.translationOffsetMeters.x,
                r.translationOffsetMeters.y - previous.translationOffsetMeters.y, 0.0f };
            const float clipSpeed = clipStep.length() / std::max(1e-5f, m_tackleTime - before);
            if (clipStep.lengthSquared() > 1e-10f) m_tacklePathDirection = tackleYaw.rotate(clipStep.normalized());
            float speed = m_tackleEntrySpeedSlide;
            if (m_tackleTime >= shape->lowStart && m_tackleTime < shape->lowEnd) {
                const float u = (m_tackleTime - shape->lowStart) / (shape->lowEnd - shape->lowStart);
                speed = m_tackleEntrySpeedSlide * (1.0f - m_tackleCommonDrop * u);
            } else if (m_tackleTime >= shape->lowEnd) {
                // Levantando: da velocidade comum ao sair do chao ate o perfil
                // do proprio clipe (escalado pela entrada) em ~0,3 s - o de
                // lado levanta devagar, o sentado sai correndo.
                const float common = m_tackleEntrySpeedSlide * (1.0f - m_tackleCommonDrop);
                const float natural = clipSpeed * m_tackleEntrySpeedSlide / std::max(0.5f, shape->lowEntrySpeed);
                speed = common + (natural - common) * smoothStep((m_tackleTime - shape->lowEnd) / 0.3f);
            }
            m_tacklePlanar += m_tacklePathDirection * (speed * dt);
            path = m_tacklePlanar;
        }
        Vec3 rootPosition { m_tackleStart.x + path.x, m_tackleStart.y + path.y,
            input.feetPositionWorld.z + profile.standingRootHeightMeters + rootLink.modelPosition.z
                + r.translationOffsetMeters.z };
        Quaternion rootOrientation = (tackleYaw * rootLink.modelOrientation * r.rotationDelta.normalized())
            .normalized();
        m_coordinates.assign(links, {});
        for (std::size_t i = 1; i < links; ++i)
            if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
                m_coordinates[i] = sampleTrack(*track, m_tackleTime, clip.durationSeconds, false, s.smoothClipSampling)
                    .jointPositionRadians;
        // Entrada: da pose da corrida (juntas e raiz) ate a do clipe.
        const float enter = smoothStep((m_tackleTime - m_tackleEnterStart) / std::max(0.01f, m_tackleRate * s.tackleBlendInSeconds));
        if (m_tackleFrom.size() == links && enter < 1.0f) {
            for (std::size_t i = 1; i < links; ++i)
                m_coordinates[i] = m_tackleFrom[i] + (m_coordinates[i] - m_tackleFrom[i]) * enter;
            // (A pose de partida segue com o embalo da entrada: parada, a
            // pelve freava de 7 m/s a zero num tick.)
            const Vec3 from = m_tackleFromRoot + m_tackleEntryVelocity * ((m_tackleTime - m_tackleEnterStart) / std::max(0.1f, m_tackleRate));
            rootPosition = from + (rootPosition - from) * enter;
            rootOrientation = nlerp(m_tackleFromOrientation, rootOrientation, enter);
        }
        if (m_landTackleInertia.active()) {
            m_landTackleInertia.apply(m_coordinates, rootPosition, rootOrientation, dt);
            // (A pelve do voo, mais baixa, com a perna do clipe esticada: o pe
            // entrava 18 cm no chao.)
            defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
            rootPosition.z += feetBelowGround(profile, m_output.pose, m_feet, input.feetPositionWorld.z);
        }
        defaultLocomotionPose3D(profile, rootPosition, rootOrientation, m_coordinates, m_output.pose);
        m_output.rootPosition = rootPosition;
        m_output.rootOrientation = rootOrientation;
        m_output.coordinates = m_coordinates;
        m_output.tackling = !m_crouchStarting;
        m_output.airTackle = m_tackleFromAir && !m_crouchStarting;
        m_output.crouchStarting = m_crouchStarting;
        m_output.rootScripted = true;
        m_output.tackleRootWorld = rootPosition;
        m_output.tackleVelocity = (rootPosition - m_tacklePrevious) * (1.0f / dt);
        m_tacklePrevious = rootPosition;
        m_output.facingYawRadians = m_tackleYaw;
        m_output.airborne = false;
        m_output.footContact = { 1.0f, 1.0f };
        m_output.footPlanted = { true, true };
        m_output.swingProgress = { -1.0f, -1.0f };
        m_output.secondsToTouchdown = { -1.0f, -1.0f };
        m_output.cycling = false;
        m_output.cyclePlanted = { true, true };
        m_output.secondsToLiftoff = { -1.0f, -1.0f };
        m_output.cycleSecondsToTouchdown = { -1.0f, -1.0f };
        m_output.swingSeconds = { -1.0f, -1.0f };
        m_output.valid = true;
        // Os pes do clipe (a arrancada termina passando para a passada).
        std::array<Vec3, 2> clipFeet {};
        if (m_feet[0] != 0 && m_feet[1] != 0)
            clipFeet = { m_output.pose.linkPositions[m_feet[0]], m_output.pose.linkPositions[m_feet[1]] };
        const bool clipEnded = m_crouchStarting ? m_tackleTime >= m_crouchExitTime - 1e-4f
            : m_tackleTime >= clip.durationSeconds - 1e-4f;
        if (clipEnded) {
            // Fim: de volta a locomocao, misturado a partir desta pose (a
            // raiz em relacao a capsula, que vai estar embaixo dela).
            const bool crouchStart = m_crouchStarting;
            m_tackleClip = nullptr;
            m_crouchStarting = false;
            if (!crouchStart) m_tackleCooldown = s.tackleCooldownSeconds;
            m_exitFrom = m_coordinates;
            m_exitRootOffset = { 0.0f, 0.0f, rootPosition.z - input.feetPositionWorld.z };
            m_exitRootOrientation = rootOrientation;
            m_tackleExit = 0.0f;
            m_yaw = viewYaw;
            if (crouchStart && m_feet[0] != 0 && m_feet[1] != 0) {
                // A arrancada: a passada continua do passo em que o clipe
                // terminou (a fase pelos pes dele), ja na velocidade dele.
                DefaultLocomotionSeed3D next;
                const Vec3 forward = (rootOrientation * rootLink.modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                next.pelvisYawRadians = std::atan2(forward.y, forward.x);
                next.velocityWorld = m_output.tackleVelocity;
                next.pelvisWorld = rootPosition;
                next.pelvisVelocityWorld = m_output.tackleVelocity;
                next.feetWorld = clipFeet;
                for (std::size_t side = 0; side < 2; ++side)
                    next.feetVelocityWorld[side] = (clipFeet[side] - m_crouchPreviousFeet[side]) * (1.0f / dt);
                next.feetValid = true;
                seed(next);
                m_started = true;
                // A saida pela inercia: a diferenca para a passada sai com a
                // velocidade que o clipe tinha (juntas e raiz), sem a foto
                // parada da saida do carrinho.
                std::vector<Vec3> velocities(links);
                const float back = std::max(0.0f, m_tackleTime - 1.0f / 60.0f);
                for (std::size_t i = 1; i < links; ++i)
                    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
                        velocities[i] = (m_coordinates[i] - sampleTrack(*track, back, clip.durationSeconds, false,
                            s.smoothClipSampling).jointPositionRadians) * (60.0f / std::max(0.01f, m_tackleRate));
                const Quaternion turn = (rootOrientation * m_crouchPreviousRoot.conjugate()).normalized();
                const float sine = std::sqrt(turn.x * turn.x + turn.y * turn.y + turn.z * turn.z);
                const Vec3 spin = sine > 1e-6f ? Vec3 { turn.x, turn.y, turn.z }
                    * (2.0f * std::atan2(sine, std::abs(turn.w)) / sine * (turn.w < 0.0f ? -1.0f : 1.0f) / dt) : Vec3 {};
                m_exitInertia.start(m_coordinates, velocities, rootPosition, rootOrientation, m_output.tackleVelocity, spin,
                    0.10f, 0.40f);
                m_tackleExit = 1.0f;
            }
            m_output.tackleEnded = true;
            m_output.tackling = false;
            m_output.airTackle = false;
            m_output.crouchStarting = false;
            m_tackleFromAir = false;
        }
        m_crouchPreviousFeet = clipFeet;
        m_crouchPreviousRoot = rootOrientation;
        return true;
    }
    m_output.tackling = false;
    m_output.crouchStarting = false;
    m_output.rootScripted = false;

    return false;
}

} // namespace MatterEngine
