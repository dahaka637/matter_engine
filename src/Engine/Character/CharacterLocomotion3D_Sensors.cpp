#include "Engine/Character/CharacterLocomotion3DInternal.hpp"

namespace MatterEngine {

void CharacterLocomotion3D::stepSensors(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const RagdollState3D& state = t.state;
    const CharacterLocomotionInput3D& input = t.input;
    float& deltaTime = t.deltaTime;
    const CharacterPhysicalState3D& physical = *t.physical;
    const bool& getUpActive = t.getUpActive;
    Vec3& desiredPlanarVelocity = t.desiredPlanarVelocity;
    // Sensor de equilibrio para os pes reagirem a empurrao, pressao e
    // tranco: o ponto de captura (centro de massa + velocidade/omega, onde o
    // corpo para se pisar ali) contra o apoio real (as solas no chao) e a
    // forca externa dos contatos. Parado, os passos de recuperacao (bloco
    // dos pes, mais abaixo) miram esse ponto; andando, o pouso do clipe e
    // deslocado para ele. Antes, parado, so havia o passo de acomodacao (o
    // pe 12 cm longe da pose, a pose presa a capsula que vem atras do
    // corpo): empurrado pelas costas a 0,8 m/s, os pes ficavam 0,8 s
    // travados e ele caia de frente. A passada dos clipes nao serve para
    // isso: todos sao de corrida (trote, recuo, strafe), com os dois pes no
    // ar a cada passada.
    float captureOutside = 0.0f;
    if (input.forceDrivenRoot && physical.valid) {
        // Apoio, ponto de captura e forca externa vem do estimador comum. Aqui
        // so a decisao de planejamento: com uma forca externa sustentada F o
        // ponto de equilibrio anda F/(m omega^2) = F h/(m g) no rumo dela
        // (150 N no peito: ~20 cm) - o passo sai antes e pousa onde segura.
        std::vector<std::array<float, 2>> support;
        float sumX = 0.0f, sumY = 0.0f;
        for (std::uint32_t i = 0; i < physical.supportPointCount; ++i) {
            const Vec3 p = physical.supportPointsWorld[i];
            support.push_back({ p.x, p.y });
            sumX += p.x;
            sumY += p.y;
        }
        Vec3 captureDirection {};
        if (!support.empty()) {
            const float omega = physical.captureOmega;
            const Vec3 forceShift = clampMagnitude(physical.externalForceWorld
                * (1.0f / (std::max(1.0f, profile.totalMassKg) * omega * omega)), 0.30f);
            // O planejamento usa a velocidade do centro de massa filtrada
            // (~60 ms): a crua do estimador reage ao tremor de cada tick e
            // disparava passos que tiravam a base (7 quedas na varredura de
            // 72 empurroes, contra 3).
            const float captureX = physical.centerOfMassWorld.x
                + m_filteredCenterOfMassVelocity.x / omega + forceShift.x;
            const float captureY = physical.centerOfMassWorld.y
                + m_filteredCenterOfMassVelocity.y / omega + forceShift.y;
            captureOutside = planarDistanceOutsideHull3D(support, captureX, captureY);
            const float count = static_cast<float>(support.size());
            captureDirection = { captureX - sumX / count, captureY - sumY / count, 0.0f };
            if (captureDirection.lengthSquared() > 1e-6f)
                captureDirection = captureDirection.normalized();
            m_capturePoint = { captureX, captureY, physical.supportHeight };
        }
        m_captureOutside = captureOutside;
        m_captureDirection = captureDirection;
    }
    m_telemetry.externalForceWorld = physical.externalForceWorld;
    m_telemetry.captureOutsideMeters = captureOutside;
    m_telemetry.capturePointWorld = m_capturePoint;

    t.speed = std::hypot(desiredPlanarVelocity.x,
        desiredPlanarVelocity.y);
    const float& speed = t.speed;
    t.moving = speed > 0.08f;
    const bool& moving = t.moving;
    // Tropeco de marcha: um pe que deveria estar no meio do balanco toca sem
    // sustentar carga. Nao confundir com o toque normal (ultimos 25% do
    // passo) nem com o pe de apoio. A fase e a do tick anterior, suficiente
    // para classificar cedo/meio contra tarde sem antecipar o solver.
    bool trippedThisTick = false;
    if (TripDetection && input.forceDrivenRoot && moving && input.grounded && !getUpActive
        && speed > 1.5f) {
        for (std::size_t side = 0; side < m_feet.size(); ++side) {
            const float phase = m_telemetry.footSwingProgress[side];
            const FootSupportEstimate3D& seen = physical.feet[side];
            const float soleSpeed = std::hypot(seen.soleVelocityWorld.x,
                seen.soleVelocityWorld.y);
            if (!m_feet[side].planted && phase >= 0.12f && phase < 0.75f
                && seen.touching && !seen.supporting && soleSpeed > 0.55f) {
                m_tripRecovery = 1.0f;
                m_tripRecoverySeconds = 0.0f;
                m_tripFoot = static_cast<int>(side);
                m_tripElevating = phase < 0.58f;
                m_tripDirection = desiredPlanarVelocity / speed;
                m_fallDirection = m_tripDirection;
                trippedThisTick = true;
                break;
            }
        }
    }
    if (!trippedThisTick) {
        m_tripRecoverySeconds += deltaTime;
        // A decisao dura aproximadamente uma passada; a intensidade cai
        // devagar o bastante para o pe seguinte terminar a recuperacao.
        m_tripRecovery = std::max(0.0f,
            m_tripRecovery - deltaTime / 0.48f);
        if (m_tripRecovery <= 0.001f) m_tripFoot = -1;
    }
    // Quase-queda em movimento: sem esperar o pe raspar no meio do balanco.
    // O corpo girando para a frente alem da corrida (a pelve inclinando mais
    // que a de regime e ainda subindo) ou freado de repente alem do que a
    // passada pede (o pe bateu em algo) entra na mesma recuperacao do
    // tropeco: pelve baixa, passada longa e rapida no rumo, bracos abertos.
    // Medido no tropeco num obstaculo a 3 m/s: o pe pegou aos 2,62 s, a pelve
    // foi de 6 a 45 graus em 0,33 s, e o tropeco so era visto aos 2,98 s -
    // com os dois pes 0,7 m atras do corpo, tarde demais.
    m_stumbleRisk = 0.0f;
    if (StumbleDetection && input.forceDrivenRoot && moving && input.grounded && !getUpActive
        && !state.links.empty() && m_brakeGait <= 0.0f) {
        const Vec3 bodyPlanar { m_filteredCenterOfMassVelocity.x, m_filteredCenterOfMassVelocity.y, 0.0f };
        const float bodySpeed = bodyPlanar.length();
        const Vec3 travel = bodySpeed > 0.8f ? bodyPlanar * (1.0f / bodySpeed)
            : speed > 0.1f ? desiredPlanarVelocity * (1.0f / speed) : Vec3 {};
        if (travel.lengthSquared() > 0.5f) {
            const Vec3 upModel = profile.links.front().modelOrientation.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
            const Vec3 up = state.links.front().orientation.rotate(upModel);
            // Contra a pelve de referencia do tick anterior: a inclinacao que a
            // passada pede (a arrancada inclina de proposito) nao e susto.
            const Vec3 referenceUp = m_output.guide.rootOrientationWorld.rotate(upModel);
            const float pitch = std::asin(std::clamp(dot(up, travel), -1.0f, 1.0f))
                - (StumbleAgainstReference ? std::asin(std::clamp(dot(referenceUp, travel), -1.0f, 1.0f)) : 0.0f);
            const float along = dot(bodyPlanar, travel);
            const float rawRate = m_stumbleValid ? (pitch - m_stumblePitch) / deltaTime : 0.0f;
            // Freada do corpo alem da da referencia (a capsula freando numa
            // curva ou parada nao conta).
            const float rawDecel = m_stumbleValid
                ? (m_stumbleAlong - along) / deltaTime + dot(m_smoothedAcceleration, travel) : 0.0f;
            if (!m_stumbleValid) m_stumbleBaseline = pitch;
            m_stumblePitch = pitch;
            m_stumbleAlong = along;
            m_stumblePitchRate += (rawRate - m_stumblePitchRate) * (1.0f - std::exp(-deltaTime / 0.03f));
            m_stumbleDecel += (rawDecel - m_stumbleDecel) * (1.0f - std::exp(-deltaTime / 0.06f));
            m_stumbleValid = true;
            const float excess = pitch - m_stumbleBaseline;
            // (So ja correndo: na arrancada a pelve inclina mais que a de
            // referencia enquanto o corpo ganha velocidade.)
            // Nem acelerando: arrancando, a pelve inclina ~18 graus alem da de
            // referencia enquanto a capsula acelera a 14 m/s2.
            const float running = smoothStep((bodySpeed - 1.3f) / 0.7f)
                * (1.0f - smoothStep((dot(m_smoothedAcceleration, travel) - 3.0f) / 3.0f));
            const float pitchRisk = smoothStep((excess - StumblePitchStart) / StumblePitchSpan)
                * smoothStep((m_stumblePitchRate - StumbleRateStart) / StumbleRateSpan) * running;
            const float deepRisk = smoothStep((excess - StumbleDeepStart) / 0.15f) * running;
            const Vec3 requested { input.intent.requestedVelocityWorld.x, input.intent.requestedVelocityWorld.y, 0.0f };
            const float wantsOn = smoothStep((dot(requested, travel) - 0.5f * bodySpeed) / 0.5f);
            const float decelRisk = smoothStep((m_stumbleDecel - StumbleDecelStart) / StumbleDecelSpan)
                * smoothStep((bodySpeed - 1.5f) / 1.0f) * wantsOn;
            m_stumbleRisk = std::max(std::max(pitchRisk, deepRisk), decelRisk);
            // A de regime acompanha a corrida devagar (~1 s), sempre: o susto e
            // rapido (0,2-0,3 s) e quase nao entra nela. (So fora do susto ela
            // congelava na arrancada e o alarme se realimentava.)
            m_stumbleBaseline += (pitch - m_stumbleBaseline)
                * (1.0f - std::exp(-deltaTime / std::max(0.05f, StumbleBaselineSeconds)));
            if (m_stumbleRisk > m_stumbleReaction) m_stumbleDirection = travel;
            // A recuperacao completa do tropeco (pelve baixa, pouso adiantado,
            // tronco) so com StumbleFullResponse: medido no pe enganchado em
            // sprint, ela derrubava mais (26 -> 36-50 em 128).
            if (StumbleFullResponse && m_stumbleRisk > m_tripRecovery) {
                m_tripRecovery = m_stumbleRisk;
                m_tripRecoverySeconds = 0.0f;
                m_tripDirection = travel;
            }
        }
    } else {
        m_stumbleValid = false;
        m_stumblePitchRate = 0.0f;
        m_stumbleDecel = 0.0f;
    }
    m_stumbleReaction = std::max(m_stumbleRisk, m_stumbleReaction - deltaTime / 0.5f);
    t.viewYaw = input.facingYawRadians;
    const float& viewYaw = t.viewYaw;
}

} // namespace MatterEngine
