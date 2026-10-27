#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepReadout(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const DefaultLocomotionInput3D& input = t.input;
    const RagdollLinkDefinition3D& rootLink = *t.rootLink;
    const float& dt = t.dt;
    const float& speed = t.speed;
    float& total = t.total;
    std::size_t& dominant = t.dominant;
    float& rate = t.rate;
    float& strideScale = t.strideScale;
    const bool& seedTick = t.seedTick;
    std::array<float, 2>& contact = t.contact;
    const Vec3& feet = t.feet;
    Quaternion& rootOrientation = t.rootOrientation;
    Vec3& rootPosition = t.rootPosition;
    const auto gaitAmplitude = [this](std::size_t g) { return gaitAmplitudeOf(g); };
    // Leitura: os pes pela animacao (o manejo de bola escolhe o pe por ela).
    m_output.valid = true;
    if (seedTick) {
        // Semente: o rumo pela pelve que saiu (o clipe tambem gira a pelve -
        // parado, ~22 graus) e a pose de novo com ele, num passe de 0,1 ms -
        // o rumo ja entra em muita coisa antes de a rotacao do clipe ser
        // conhecida, e a inercia mede a diferenca contra esta pose.
        const Vec3 out = (rootOrientation * rootLink.modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
        m_yaw = wrapAngle(m_yaw + wrapAngle(m_seed.pelvisYawRadians - std::atan2(out.y, out.x)));
        m_yawRate = 0.0f;
        DefaultLocomotionInput3D again = input;
        again.deltaTime = 1e-4f;
        m_exitInertiaDt = dt;
        again.tackleRequest = false;
        // (O gesto de bola ja comecou no primeiro passe: de novo, o tocador
        // recomecava.)
        again.gestureRequested = false;
        update(profile, animations, again);
        return;
    }
    m_output.facingYawRadians = m_yaw;
    m_output.cyclePhase = m_phase;
    for (std::size_t side = 0; side < 2; ++side) m_output.gestureLegs[side] = m_gesturePlayers[side].movesLeg();
    m_output.movingShare = total;
    m_output.landingIntensity = m_landTime >= 0.0f ? m_landIntensity : 0.0f;
    m_output.landingSink = m_landingSink;
    m_output.airborne = !input.grounded;
    m_output.lean = m_lean;
    for (std::size_t side = 0; side < 2; ++side) {
        m_output.footContact[side] = std::clamp(contact[side], 0.0f, 1.0f);
        const bool swinging = input.grounded && total > 0.3f && m_output.footContact[side] < 0.5f;
        m_output.footPlanted[side] = !swinging && input.grounded;
        const Gait& gait = m_gaits[dominant];
        const float span = std::max(0.05f, fraction(gait.land[side] - gait.liftOff[side]));
        const bool cycling = input.grounded && total > 0.3f && rate > 1e-3f;
        const bool inSwing = fraction(m_phase - gait.liftOff[side]) < span;
        m_output.cycling = cycling;
        m_output.cyclePlanted[side] = !cycling || !inSwing;
        m_output.swingSeconds[side] = cycling ? span / rate : -1.0f;
        m_output.secondsToLiftoff[side] = cycling && !inSwing ? fraction(gait.liftOff[side] - m_phase) / rate : -1.0f;
        m_output.cycleSecondsToTouchdown[side] = cycling && inSwing ? fraction(gait.land[side] - m_phase) / rate : -1.0f;
        if (!swinging || rate <= 1e-3f) {
            m_output.swingProgress[side] = -1.0f;
            m_output.secondsToTouchdown[side] = -1.0f;
            continue;
        }
        m_output.swingProgress[side] = std::clamp(fraction(m_phase - gait.liftOff[side]) / span, 0.0f, 1.0f);
        m_output.secondsToTouchdown[side] = fraction(gait.land[side] - m_phase) / rate;
    }
    // O ciclo e o pe no futuro (a referencia do contrato do RL).
    {
        const bool cycling = m_output.cycling && rate > 1e-3f;
        m_output.cycleSeconds = cycling ? 1.0f / rate : -1.0f;
        // O pe no futuro: a tabela das marchas (pelos pesos), com a passada de
        // agora (o comprimento no rumo de cada marcha) e a altura da pelve.
        const float height = rootPosition.z - feet.z;
        const float stride = (strideScale - 1.0f) * m_moving * smoothStep((speed - 0.3f) / 1.0f);
        float total = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) total += m_weights[g];
        for (int k = 0; k < DefaultLocomotionOutput3D::ForecastCount; ++k) {
            const float ahead = static_cast<float>(k) * m_output.forecastStep;
            const float phase = fraction(m_phase + rate * ahead);
            for (std::size_t side = 0; side < 2; ++side) {
                Vec3 position {};
                float used = 0.0f;
                for (std::size_t g = 0; g < GaitCount && total > 1e-3f; ++g) {
                    const float weight = m_weights[g] / total;
                    if (weight < 0.02f || m_gaits[g].clip == nullptr) continue;
                    const float u = phase * Gait::FeetSamples;
                    const int a = static_cast<int>(u) % Gait::FeetSamples;
                    const int b = (a + 1) % Gait::FeetSamples;
                    const float f = u - std::floor(u);
                    const auto& table = m_gaits[g].feet[side];
                    Vec3 foot = table[static_cast<std::size_t>(a)] * (1.0f - f) + table[static_cast<std::size_t>(b)] * f;
                    // (A amplitude da marcha: o pe em volta da media do ciclo.)
                    foot = m_gaits[g].feetMean[side] + (foot - m_gaits[g].feetMean[side]) * gaitAmplitude(g);
                    const Vec3 axis { std::cos(m_gaits[g].heading), std::sin(m_gaits[g].heading), 0.0f };
                    foot += axis * (dot(Vec3 { foot.x, foot.y, 0.0f }, axis) * stride);
                    position += foot * weight;
                    used += weight;
                }
                if (used > 1e-3f) {
                    position = position * (1.0f / used);
                    position.z += height;
                    m_output.footForecast[side][static_cast<std::size_t>(k)] = position;
                } else {
                    m_output.footForecast[side][static_cast<std::size_t>(k)] = {};
                }
            }
        }
    }
}

} // namespace MatterEngine
