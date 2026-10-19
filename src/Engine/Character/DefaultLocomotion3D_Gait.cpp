#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepGait(Tick& t) {
    const RagdollProfile3D& profile = t.profile;
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const RagdollLinkDefinition3D& rootLink = *t.rootLink;
    const float& dt = t.dt;
    const float& viewYaw = t.viewYaw;
    const Vec3& velocity = t.velocity;
    const float& speed = t.speed;
    float& touchdown = t.touchdown;
    // Requested travel: as teclas quando ha; senao, o movimento da capsula.
    t.requested = planar(input.requestedVelocityWorld);
    const Vec3& requested = t.requested;
    const bool moving = speed > 0.3f || requested.length() > 0.1f;
    t.travel = requested.length() > 0.1f ? requested : velocity;
    const Vec3& travel = t.travel;
    const float travelYaw = travel.lengthSquared() > 1e-4f ? std::atan2(travel.y, travel.x) : viewYaw;

    // O setor pela direcao em relacao a camera (oito, com folga).
    const float fromView = wrapAngle(travelYaw - viewYaw);
    const int sectorBefore = m_sector;
    if (moving) {
        const float step = Pi * 0.25f;
        if (std::abs(wrapAngle(fromView - static_cast<float>(m_sector) * step))
            > step * 0.5f + radians(s.sectorHysteresisDegrees))
            m_sector = (static_cast<int>(std::lround(fromView / step)) + static_cast<int>(Directions))
                % static_cast<int>(Directions);
    }

    // Os pesos: o setor (trote/sprint pela velocidade da tabela no rumo) na
    // mola da forma, o quanto anda na sua.
    t.wanted = {};
    std::array<float, GaitCount>& wanted = t.wanted;
    t.sprint = 0.0f;
    float& sprint = t.sprint;
    {
        const std::size_t d = static_cast<std::size_t>(m_sector);
        const float trotSpeed = defaultLocomotionSpeed3D(s, fromView, false);
        const float sprintSpeed = defaultLocomotionSpeed3D(s, fromView, true);
        sprint = sprintSpeed > trotSpeed + 0.1f
            ? std::clamp((speed - trotSpeed) / (sprintSpeed - trotSpeed), 0.0f, 1.0f) : 0.0f;
        wanted[d] = 1.0f - sprint;
        wanted[d + Directions] = sprint;
    }
    // O rumo dos clipes do setor, para a pelve.
    float clipHeading = 0.0f;
    {
        float x = 0.0f, y = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (wanted[g] <= 0.0f || m_gaits[g].clip == nullptr) continue;
            x += wanted[g] * std::cos(m_gaits[g].heading);
            y += wanted[g] * std::sin(m_gaits[g].heading);
        }
        if (x * x + y * y > 1e-6f) clipHeading = std::atan2(y, x);
    }

    // O rumo da pelve. Andando: o movimento menos o rumo do clipe (e, para a
    // frente, adiantado para dentro da curva). Parado: a coluna e a cabeca
    // olham primeiro; a pelve so gira quando o erro passa do que elas cobrem,
    // e para perto da camera.
    {
        float target = m_yaw;
        float seconds = s.turnSeconds;
        float springSeconds = s.turnSpringSeconds;
        if (moving) {
            m_standTurning = false;
            target = wrapAngle(travelYaw - clipHeading);
            if (frontSector(m_sector))
                target = wrapAngle(target + std::clamp(s.pelvisCurveLeadSeconds * m_pathTurnRate,
                    -s.pelvisCurveLeadMaximum, s.pelvisCurveLeadMaximum) * smoothStep((speed - 1.5f) / 1.5f));
        } else {
            const float error = wrapAngle(viewYaw - m_yaw);
            if (!m_standTurning && std::abs(error) > s.standTurnStart) m_standTurning = true;
            if (m_standTurning) {
                target = wrapAngle(viewYaw - std::copysign(s.standTurnKeep, error));
                if (std::abs(wrapAngle(target - m_yaw)) < 0.03f) m_standTurning = false;
            }
            seconds = s.standTurnSeconds;
            springSeconds = s.standTurnSpringSeconds;
        }
        if (s.adaptiveTransitions) {
            // Mola de segunda ordem com aceleracao e velocidade limitadas.
            const float omega = 1.0f / std::max(0.01f, springSeconds);
            const float acceleration = std::clamp(omega * omega * wrapAngle(target - m_yaw) - 2.0f * omega * m_yawRate,
                -s.maximumTurnAcceleration, s.maximumTurnAcceleration);
            m_yawRate = std::clamp(m_yawRate + acceleration * dt, -s.maximumTurnRateRadiansPerSecond,
                s.maximumTurnRateRadiansPerSecond);
            m_yaw = wrapAngle(m_yaw + m_yawRate * dt);
        } else {
            float step = wrapAngle(target - m_yaw) * (1.0f - std::exp(-dt / std::max(0.005f, seconds)));
            const float limit = s.maximumTurnRateRadiansPerSecond * dt;
            step = std::clamp(step, -limit, limit);
            m_yawRate = step / dt;
            m_yaw = wrapAngle(m_yaw + step);
        }
        // (O alvo do giro, para os passos do giro parado.)
        m_standTurnTarget = target;
    }

    // Transicoes adaptativas: frente, lados e costas pelo angulo entre o
    // movimento real e a pelve - entre os dois clipes vizinhos, pela fracao do
    // angulo (o setor so decidiu para onde a pelve vai).
    if (s.adaptiveTransitions) {
        const Vec3 motion = speed > 0.5f ? velocity : requested.length() > 0.1f ? requested : velocity;
        if (motion.lengthSquared() > 1e-4f) m_localTravel = wrapAngle(std::atan2(motion.y, motion.x) - m_yaw);
        const auto around = [](float angle) {
            const float turn = 2.0f * Pi;
            return angle - turn * std::floor(angle / turn);
        };
        wanted.fill(0.0f);
        for (std::size_t tier = 0; tier < 2; ++tier) {
            const float share = tier == 0 ? 1.0f - sprint : sprint;
            if (share <= 0.0f) continue;
            // O clipe logo antes e o logo depois do angulo (anti-horario).
            std::size_t below = GaitCount, above = GaitCount;
            float belowGap = 1e9f, aboveGap = 1e9f;
            for (std::size_t d = 0; d < Directions; ++d) {
                const std::size_t g = d + tier * Directions;
                if (m_gaits[g].clip == nullptr) continue;
                const float down = around(m_localTravel - m_gaits[g].heading);
                const float up = around(m_gaits[g].heading - m_localTravel);
                if (down < belowGap) { belowGap = down; below = g; }
                if (up < aboveGap) { aboveGap = up; above = g; }
            }
            if (below == GaitCount) continue;
            const float span = belowGap + aboveGap;
            if (above == GaitCount || span < 1e-4f || m_gaits[below].clip == m_gaits[above].clip) {
                wanted[below] += share;
                continue;
            }
            const float zone = std::clamp(s.directionBlendDeadZone, 0.0f, 0.45f);
            const float fraction = belowGap / span;
            const float toward = zone > 0.0f ? smoothStep((fraction - zone) / (1.0f - 2.0f * zone)) : fraction;
            wanted[below] += share * (1.0f - toward);
            wanted[above] += share * toward;
        }
    }
    if (m_seedPending) {
        // Semente: as misturas ja onde a velocidade as quer (sem nascer
        // parado e ir acelerando a pose).
        const float seedMoving = input.grounded ? smoothStep(speed / std::max(0.05f, s.idleSpeed)) : 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            m_shape[g] = m_upperShape[g] = moving ? wanted[g] : 0.0f;
            m_shapeRates[g] = m_upperShapeRates[g] = 0.0f;
        }
        m_moving = m_upperMoving = seedMoving;
        m_movingRate = m_upperMovingRate = 0.0f;
        m_startTime = 10.0f;
        m_stopTarget = -1.0f;
    }
    t.shapeTotal = 0.0f;
    float& shapeTotal = t.shapeTotal;
    for (std::size_t g = 0; g < GaitCount; ++g) {
        if (moving || m_moving > 0.01f) spring(m_shape[g], m_shapeRates[g], wanted[g], s.gaitBlendSeconds, dt);
        if (m_gaits[g].clip == nullptr) m_shape[g] = 0.0f;
        m_shape[g] = std::max(0.0f, m_shape[g]);
        shapeTotal += m_shape[g];
    }
    if (shapeTotal < 1e-4f) { m_shape[0] = 1.0f; shapeTotal = 1.0f; }
    const float movingWanted = input.grounded ? smoothStep(speed / std::max(0.05f, s.idleSpeed)) : 0.0f;
    spring(m_moving, m_movingRate, movingWanted, s.idleBlendSeconds, dt);
    m_moving = std::clamp(m_moving, 0.0f, 1.0f);
    t.total = 0.0f;
    float& total = t.total;
    for (std::size_t g = 0; g < GaitCount; ++g) {
        m_weights[g] = m_moving * m_shape[g] / shapeTotal;
        total += m_weights[g];
    }
    m_idleWeight = std::max(0.0f, 1.0f - total);
    t.dominant = 0;
    std::size_t& dominant = t.dominant;
    for (std::size_t g = 1; g < GaitCount; ++g)
        if (m_weights[g] > m_weights[dominant]) dominant = g;
    // O corpo de cima nas molas mais lentas (as pernas ja estao na pose nova,
    // o tronco e os bracos chegam depois).
    t.upperIdleWeight = 1.0f;
    float& upperIdleWeight = t.upperIdleWeight;
    {
        float upperShapeTotal = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (moving || m_upperMoving > 0.01f)
                spring(m_upperShape[g], m_upperShapeRates[g], wanted[g], s.upperGaitBlendSeconds, dt);
            if (m_gaits[g].clip == nullptr) m_upperShape[g] = 0.0f;
            m_upperShape[g] = std::max(0.0f, m_upperShape[g]);
            upperShapeTotal += m_upperShape[g];
        }
        if (upperShapeTotal < 1e-4f) { m_upperShape[0] = 1.0f; upperShapeTotal = 1.0f; }
        spring(m_upperMoving, m_upperMovingRate, movingWanted, s.upperIdleBlendSeconds, dt);
        m_upperMoving = std::clamp(m_upperMoving, 0.0f, 1.0f);
        for (std::size_t g = 0; g < GaitCount; ++g) {
            m_upperWeights[g] = m_upperMoving * m_upperShape[g] / upperShapeTotal;
            upperIdleWeight -= m_upperWeights[g];
        }
        upperIdleWeight = std::max(0.0f, upperIdleWeight);
    }
    // O peso afunda ao cortar de direcao correndo (mais quanto maior o corte).
    const float dipSpeed = s.transitionDipMeters * 2.71828f / std::max(0.01f, s.transitionDipSeconds);
    if (moving && m_sector != sectorBefore) {
        const float cut = std::abs(wrapAngle(static_cast<float>(m_sector - sectorBefore) * Pi * 0.25f)) / Pi;
        m_dipRate -= dipSpeed * cut * smoothStep((speed - 2.0f) / 3.0f);
    }
    t.yaw = yawRotation(m_yaw);
    const Quaternion& yaw = t.yaw;
    t.forward = Vec3 { std::cos(m_yaw), std::sin(m_yaw), 0.0f };
    const Vec3& forward = t.forward;
    t.left = Vec3 { -forward.y, forward.x, 0.0f };
    const Vec3& left = t.left;

    // A passada na velocidade da capsula: ritmo e comprimento (os pes nao
    // escorregam: quem anda mais rapido que o clipe da passos mais longos e
    // um pouco mais rapidos).
    // A amplitude de cada marcha (trote 0..7, corrida 8..15); de lado (2 e 6)
    // a parte trotAmplitudeSide / sprintAmplitudeSide.
    const auto gaitAmplitude = [this](std::size_t g) { return gaitAmplitudeOf(g); };
    t.rate = 0.0f;
    t.strideScale = 1.0f;
    float& rate = t.rate;
    float& strideScale = t.strideScale;
    if (total > 1e-3f) {
        float frequency = 0.0f, natural = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g) {
            if (m_weights[g] <= 0.0f) continue;
            frequency += m_weights[g] / std::max(0.05f, m_gaits[g].clip->durationSeconds);
            natural += m_weights[g] * m_gaits[g].speed;
        }
        frequency /= total;
        natural /= total;
        const float ratio = std::max(0.05f, speed / std::max(0.1f, natural));
        strideScale = std::clamp(std::sqrt(ratio), s.minimumStrideScale, s.maximumStrideScale);
        // O trote mais rapido que o clipe: o movimento cresce (ver
        // trotAmplitude) e o ritmo divide por ele, na parte do trote.
        // (Pela velocidade natural do trote, nao a da mistura com o sprint -
        // ela pulava na troca -, e suavizada no tempo: mudando de uma vez, a
        // pose das pernas e dos bracos saltava - picos de ~900 m/s2 contra
        // ~300 ao parar e ao trocar de marcha.)
        for (std::size_t kind = 0; kind < 2; ++kind) {
            float kindWeight = 0.0f, kindNatural = 0.0f;
            for (std::size_t d = 0; d < Directions; ++d) {
                const std::size_t g = kind * Directions + d;
                kindWeight += m_weights[g];
                kindNatural += m_weights[g] * m_gaits[g].speed;
            }
            const float kindRatio = kindWeight > 1e-3f ? speed / std::max(0.1f, kindNatural / kindWeight) : 1.0f;
            const float amplitudeTarget = s.gaitAmplitude
                ? std::clamp(std::pow(std::max(0.05f, kindRatio), s.gaitAmplitudeExponent),
                    std::min(1.0f, s.gaitAmplitudeMinimum), std::max(1.0f, s.gaitAmplitudeMaximum)) : 1.0f;
            m_gaitAmplitude[kind] += (amplitudeTarget - m_gaitAmplitude[kind])
                * (1.0f - std::exp(-dt / std::max(0.01f, s.gaitAmplitudeSeconds)));
        }
        float amplitudeShare = 0.0f;
        for (std::size_t g = 0; g < GaitCount; ++g)
            amplitudeShare += m_weights[g] * (gaitAmplitude(g) - 1.0f);
        amplitudeShare = 1.0f + amplitudeShare / total;
        // (A passada de conducao fora do limite do ritmo: a amplitude ja vem
        // dividida pela cadencia, que multiplica o ritmo depois do limite.)
        const float cadence = std::max(1.0f, m_dribbleCadence);
        const float playRate = std::clamp(ratio / (strideScale * amplitudeShare * cadence), s.minimumPlayRate,
            s.maximumPlayRate);
        rate = frequency * playRate * cadence;
        // Plano v5: a passada de conducao - a cadencia ate os pousos
        // alternados a dribbleHalfCycle (o ritmo de agora sem ela: meio ciclo
        // em 0,5 / ritmo).
        float wanted = 1.0f;
        const float style = std::clamp(input.dribbleStyle, 0.0f, 1.0f);
        if (style > 0.0f && frequency * playRate > 1e-3f) {
            const float half = 0.5f / (frequency * playRate);
            wanted = 1.0f + style * (std::clamp(half / std::max(0.1f, s.dribbleHalfCycle), 1.0f,
                std::max(1.0f, s.dribbleCadenceMaximum)) - 1.0f);
        }
        m_dribbleCadence += (wanted - m_dribbleCadence) * (1.0f - std::exp(-dt / std::max(0.01f, s.dribbleCadenceSeconds)));
    } else {
        m_dribbleCadence = 1.0f;
    }
    // Parando: o passo termina no proximo cruzamento das pernas, freando (o pe
    // apoiado fica travado; o de tras vem para baixo do corpo).
    if (s.adaptiveTransitions && s.settleStop && input.grounded && requested.length() < 0.1f && m_moving > 0.02f
        && speed < s.stopSettleSpeed && total > 1e-3f) {
        const Gait& gait = m_gaits[dominant];
        if (m_stopTarget < 0.0f) {
            const float first = fraction(gait.crossing - m_phase);
            const float second = fraction(gait.crossing + 0.5f - m_phase);
            m_stopTarget = fraction(m_phase + std::min(first, second));
        }
        float remaining = fraction(m_stopTarget - m_phase);
        if (remaining > 0.75f) remaining = 0.0f;
        rate = remaining / std::max(0.02f, s.stopSettleSeconds);
    } else if (requested.length() >= 0.1f || m_moving <= 0.02f) {
        m_stopTarget = -1.0f;
    }
    // Saindo do parado: o ciclo comeca no cruzamento dos pes (a pose mais
    // perto da parada), nao onde tinha parado.
    // Saindo do parado para a frente: o gesto da arrancada e o peso afundando.
    if (m_moving < 0.03f && movingWanted > 0.05f) {
        const int sector = std::clamp(m_sector, 0, static_cast<int>(Directions) - 1);
        m_phase = m_gaits[static_cast<std::size_t>(sector)].crossing;
        if (m_startTime < 0.0f && s.startGesture) {
            m_startTime = 0.0f;
            m_startSprint = requested.length() > defaultLocomotionSpeed3D(s, fromView, false) + 0.5f;
            m_dipRate -= dipSpeed * (m_startSprint ? 1.0f : 0.6f);
        }
    }
    if (m_moving < 0.03f && !moving) m_startTime = -1.0f;
    if (!frontSector(m_sector) && m_startTime >= 0.0f) m_startTime = 10.0f;
    spring(m_dip, m_dipRate, 0.0f, s.transitionDipSeconds, dt);
    m_dip = std::max(m_dip, -1.5f * s.transitionDipMeters);
    t.seedTick = m_seedPending;
    const bool& seedTick = t.seedTick;
    if (m_seedPending) {
        // Semente: a fase em que os pes das marchas (pelos pesos), em relacao
        // a pelve e no rumo dela, mais se parecem com os do corpo - onde
        // estao e para onde vao.
        m_seedPending = false;
        if (m_seed.feetValid && total > 0.05f && m_feet[0] != 0 && m_feet[1] != 0) {
            const Quaternion unyaw = yawRotation(-m_seed.pelvisYawRadians);
            std::array<Vec3, 2> body, bodyVelocity;
            for (std::size_t side = 0; side < 2; ++side) {
                body[side] = unyaw.rotate(m_seed.feetWorld[side] - m_seed.pelvisWorld);
                bodyVelocity[side] = unyaw.rotate(m_seed.feetVelocityWorld[side] - m_seed.pelvisVelocityWorld);
            }
            const auto clipFeet = [&](const AnimationClip3D& clip, float phase, float touchdown) {
                const RagdollAnimationPose3D pose = sampleRagdollAnimationPose3D(profile, &clip,
                    fraction(phase + touchdown) * clip.durationSeconds, true);
                const Vec3 pelvisForward = (pose.linkOrientations.front() * rootLink.modelOrientation.conjugate())
                    .rotate({ 1.0f, 0.0f, 0.0f });
                const Quaternion back = yawRotation(-std::atan2(pelvisForward.y, pelvisForward.x));
                std::array<Vec3, 2> out;
                for (std::size_t side = 0; side < 2; ++side)
                    out[side] = back.rotate(pose.linkPositions[m_feet[side]] - pose.linkPositions.front());
                return out;
            };
            constexpr int Samples = 48;
            constexpr float Step = 1.0f / 96.0f;
            constexpr float VelocityWeight = 0.08f;  // s: 1 m/s de diferenca pesa como 8 cm
            float best = 1e9f, bestPhase = m_phase;
            for (int k = 0; k < Samples; ++k) {
                const float phase = static_cast<float>(k) / static_cast<float>(Samples);
                std::array<Vec3, 2> feetAt {}, feetVelocity {};
                for (std::size_t g = 0; g < GaitCount; ++g) {
                    if (m_weights[g] < 0.05f || m_gaits[g].clip == nullptr) continue;
                    const float share = m_weights[g] / total;
                    const auto now = clipFeet(*m_gaits[g].clip, phase, m_gaits[g].touchdown);
                    const auto next = clipFeet(*m_gaits[g].clip, phase + Step, m_gaits[g].touchdown);
                    for (std::size_t side = 0; side < 2; ++side) {
                        feetAt[side] += now[side] * share;
                        feetVelocity[side] += (next[side] - now[side]) * (rate / Step * share);
                    }
                }
                float error = 0.0f;
                for (std::size_t side = 0; side < 2; ++side)
                    error += (feetAt[side] - body[side]).lengthSquared()
                        + (feetVelocity[side] - bodyVelocity[side]).lengthSquared() * VelocityWeight * VelocityWeight;
                if (error < best) { best = error; bestPhase = phase; }
            }
            m_phase = bestPhase;
        }
    }
    {
        // (O gesto com a bola acelera um pouco a passada: o pe do toque pousa
        // no clipe quando o gesto volta - plano v4; os tempos dos pes na
        // saida vem neste ritmo.)
        rate *= std::clamp(input.phaseRateScale, 0.3f, 3.0f);
        const float before = m_phase;
        m_phase = fraction(m_phase + rate * dt);
        m_strides += m_phase < before ? 1.0f : 0.0f;
    }
    m_idleTime += dt;

}

} // namespace MatterEngine
