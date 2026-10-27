#include "Engine/Gesture/GesturePlayer3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MatterEngine {
namespace {

// O sorteio do jeito do gesto (splitmix32: a mesma semente, o mesmo gesto).
struct Random {
    std::uint32_t state;
    float next() {
        std::uint32_t z = (state += 0x9E3779B9u);
        z = (z ^ (z >> 16)) * 0x85EBCA6Bu;
        z = (z ^ (z >> 13)) * 0xC2B2AE35u;
        z ^= z >> 16;
        return static_cast<float>(z >> 8) * (1.0f / 16777216.0f);
    }
    float range(float low, float high) { return low + (high - low) * next(); }
};

constexpr float VelocityLimit = 12.0f;
constexpr float Degree = 3.14159265358979323846f / 180.0f;

// O pulso da subida (0 e parado nas pontas, 1 no meio da batida) e as
// derivadas dele em s.
float liftBump(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    const float u = s * (1.0f - s);
    return 16.0f * u * u;
}
float liftBumpSlope(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return 32.0f * s * (1.0f - s) * (1.0f - 2.0f * s);
}
float liftBumpCurve(float s) {
    s = std::clamp(s, 0.0f, 1.0f);
    return 32.0f * (1.0f - 6.0f * s + 6.0f * s * s);
}

} // namespace

void GesturePlayer3D::start(const GestureSpec3D& spec, const GestureRig3D& rig, const GestureLegPose3D& current,
    const GestureLegPose3D& currentVelocity) {
    m_spec = spec;
    m_spec.side = spec.side == 1 ? 1 : 0;
    m_keys = gestureKeys3D(m_spec, rig);
    m_active = true;
    m_returning = false;
    m_lastValid = false;
    m_startLeg = current;
    m_startOffsetValid = false;
    m_fadeStart = m_fadeEnd = -1.0f;
    m_time = 0.0f;
    // A batida, o acompanhamento e a volta. A perna passa pela bola no mais
    // rapido do gesto (a velocidade das juntas pelo rumo do pe no catalogo;
    // parada ali, ela arrancava do zero depois - 2.300 m/s2 no chute
    // correndo); no fim do acompanhamento ela quase para e ja vira (parada e
    // plana ali, ficava pendurada no alto). A volta tem 35% do gesto, de 0,06 a
    // 0,12 s.
    // (A recepcao, mesmo do chute fraco, tem os limites do manejo. Ela e o
    // toque na bola alta sao calmos: a perna sobe ate a bola mais devagar, a
    // batida e a volta mais longas.)
    const bool shot = m_spec.kind == GestureKind3D::Shot && !m_spec.receive;
    const bool calm = m_spec.receive || (m_spec.high && !shot);
    const float limit = shot ? 26.0f : calm ? 10.0f : 14.0f;
    const float accelerationLimit = shot ? 900.0f : calm ? 260.0f : 450.0f;
    const float duration = m_keys.duration;
    const float back = calm ? std::clamp(0.35f * duration, 0.10f, 0.18f) : std::clamp(0.35f * duration, 0.06f, 0.12f);
    m_lift = {};
    const float nominalFollow = m_keys.followTime - m_keys.contactTime;
    // O chute arma primeiro: a perna sai de onde esta (com a velocidade que
    // tinha - correndo, ela vira ali) e para armada; a batida parte dali. No
    // tempo pela barra, ou no que a perna precisa para chegar la dentro de 70%
    // dos limites da batida (a perna da corrida la na frente volta mais
    // devagar: em 68 ms, o pe vinha para tras a 18 m/s).
    float windup = 0.0f;
    if (m_keys.windupTime > 0.0f) {
        const float nominal = std::min(m_keys.windupTime, 0.3f * duration);
        const float longest = std::max(nominal, std::min(0.22f, 0.4f * duration));
        windup = longest;
        for (float candidate = nominal; candidate <= longest + 1e-4f; candidate += 0.005f) {
            float worst = 0.0f;
            for (std::size_t i = 0; i < 4; ++i) {
                const GestureQuintic q { current.value[i], std::clamp(currentVelocity.value[i], -VelocityLimit, VelocityLimit),
                    0.0f, m_keys.windup.value[i], 0.0f, 0.0f, candidate };
                for (int n = 0; n <= 16; ++n) {
                    const float s = static_cast<float>(n) / 16.0f;
                    worst = std::max({ worst, std::abs(q.velocity(s)) / (0.7f * limit),
                        std::abs(q.acceleration(s)) / (0.7f * accelerationLimit) });
                }
            }
            if (worst <= 1.0f) {
                windup = candidate;
                break;
            }
        }
    }
    // (No alto da armada a perna para e ja acelera para a bola - parada e
    // plana ali, ela ficava pendurada uns 50 ms.)
    GestureLegPose3D from = current, fromVelocity = currentVelocity;
    if (windup > 0.0f) {
        from = m_keys.windup;
        fromVelocity = {};
    }
    const auto turnAcceleration = [&](std::size_t i, float strike) {
        return windup > 0.0f && i < 4 ? 1.5f * (m_keys.contact.value[i] - m_keys.windup.value[i]) / (strike * strike) : 0.0f;
    };
    const auto followFor = [&](float strike) {
        const float follow = std::clamp(nominalFollow, 0.03f, std::max(0.03f, duration - windup - strike - back));
        return std::max(1e-3f, std::min(windup + strike + follow, duration - 0.03f) - windup - strike);
    };
    // Monta os trechos para uma batida de strike segundos e da o quanto o
    // pior pico (velocidade ou aceleracao da coxa e do joelho) passa do limite.
    const auto build = [&](float strike) {
        const float follow = followFor(strike);
        const float speed = m_keys.contactSpeed * m_keys.followDistance / follow;
        float worst = 0.0f;
        for (std::size_t i = 0; i < 7; ++i) {
            const float p0 = from.value[i];
            const float v0 = std::clamp(fromVelocity.value[i], -VelocityLimit, VelocityLimit);
            const float k1 = m_keys.contact.value[i], k2 = m_keys.follow.value[i];
            const float v1 = i < 4 ? std::clamp(m_keys.contactTangent.value[i] * speed, -0.8f * limit, 0.8f * limit)
                                   : gestureMonotoneTangent((k1 - p0) / strike, (k2 - k1) / follow);
            const float v2 = 0.10f * (k2 - k1) / follow;
            const float a2 = -2.5f * (k2 - k1) / (follow * follow);
            m_strike[i] = { p0, v0, turnAcceleration(i, strike), k1, v1, 0.0f, strike };
            m_follow[i] = { k1, v1, 0.0f, k2, v2, a2, follow };
            // (A subida: a batida passa pela pose dela no meio, sem parar.)
            m_lift[i] = m_keys.liftValid ? m_keys.lift.value[i] - m_strike[i].position(0.5f) : 0.0f;
            if (i >= 4) continue;
            for (int n = 0; n <= 16; ++n) {
                const float s = static_cast<float>(n) / 16.0f;
                const float strikeVelocity = m_strike[i].velocity(s) + m_lift[i] * liftBumpSlope(s) / strike;
                const float strikeAcceleration
                    = m_strike[i].acceleration(s) + m_lift[i] * liftBumpCurve(s) / (strike * strike);
                worst = std::max({ worst, std::abs(strikeVelocity) / limit, std::abs(m_follow[i].velocity(s)) / limit,
                    std::abs(strikeAcceleration) / accelerationLimit,
                    std::abs(m_follow[i].acceleration(s)) / accelerationLimit });
            }
        }
        return worst;
    };
    // A batida mais curta que cabe nos limites (de 35 a 120 ms, ate 40% do
    // gesto): a perna que vem da corrida andando para o outro lado precisa de
    // tempo para virar. (Em tempo fixo, a sola por cima da bola passava de
    // 2.500 graus/s.)
    // (Na recepcao a bola ja parou no comando: o pe chega em ate 0,14 s - com
    // 0,2 s a bola ficava parada sozinha a espera dele.)
    const float longest = calm ? std::max(0.035f, std::min(m_spec.receive ? 0.14f : 0.22f, 0.5f * duration - windup))
                               : std::max(0.035f, std::min(0.12f, 0.4f * duration - windup));
    float strike = longest, best = 1e9f;
    for (float candidate = 0.035f; candidate <= longest + 1e-4f; candidate += 0.005f) {
        const float worst = build(candidate);
        if (worst < best) { best = worst; strike = candidate; }
        if (worst <= 1.0f) break;
    }
    build(strike);
    for (std::size_t i = 0; i < 7 && windup > 0.0f; ++i)
        m_windup[i] = { current.value[i], std::clamp(currentVelocity.value[i], -VelocityLimit, VelocityLimit), 0.0f,
            m_keys.windup.value[i], 0.0f, turnAcceleration(i, strike), windup };
    m_keys.windupTime = windup;
    m_keys.contactTime = windup + strike;
    m_keys.followTime = windup + strike + followFor(strike);
    const float t1 = m_keys.contactTime, t2 = m_keys.followTime;
    // O corpo: cada canal com o pico sorteado (80 a 120%), o instante do pico
    // e a largura um pouco deslocados; todos voltam a zero no fim.
    Random random { spec.seed * 2654435761u + 0x51ED270Bu };
    const GestureBodyPeaks3D& b = m_keys.body;
    const float end = m_keys.duration;
    // (Cada canal sobe em pelo menos 0,12 s: no instante do contato, o tronco
    // inclinava 9 cm em tres ticks.)
    const float rise = std::min(0.12f, 0.4f * end);
    const auto channel = [&](float peak, float top) {
        Channel c;
        c.peak = peak * random.range(0.8f, 1.2f);
        c.start = 0.0f;
        c.top = std::clamp(std::max(top, rise) + random.range(-0.02f, 0.02f), 0.02f, 0.8f * end);
        c.end = end * random.range(0.92f, 1.0f);
        return c;
    };
    const float mid = 0.5f * (t1 + t2);
    m_channels[0] = channel(b.drop, t1 + 0.3f * (t2 - t1));
    m_channels[1] = channel(b.forward, mid);
    m_channels[2] = channel(b.left, mid);
    m_channels[3] = channel(b.yaw, mid);
    m_channels[4] = channel(b.roll, mid);
    m_channels[5] = channel(b.spineTwist, mid);
    m_channels[6] = channel(b.spinePitch, t1);
    // (Para tras so no fim do acompanhamento: o chute forte joga o tronco.)
    m_channels[7] = channel(b.spineBack, t2);
    m_channels[7].start = t1;
    m_channels[8] = channel(b.spineRoll, mid);
    m_channels[9] = channel(b.headPitch, t1);
    // Os bracos com o jeito de cada um sorteado (24/10, pedido do usuario:
    // "procedural aleatorio, nao sempre a mesma coisa, sutil mas bonito"): o
    // quanto vai a frente e abre, o instante (um adianta, o outro atrasa), um
    // desvio de lado e o antebraco chegando um pouco depois do ombro.
    for (std::size_t arm = 0; arm < 2; ++arm) {
        const float lead = random.range(-0.25f, 0.25f) * std::max(0.1f, t2 - t1);
        m_channels[10 + arm] = channel(b.armForward[arm], t2 + lead);
        m_channels[10 + arm].peak = b.armForward[arm] * random.range(0.45f, 1.15f);
        m_channels[12 + arm] = channel(b.armOut[arm], mid + lead);
        m_channels[12 + arm].peak = b.armOut[arm] * random.range(0.6f, 1.2f);
        m_channels[14 + arm] = channel(b.elbow[arm] * random.range(0.6f, 1.5f) + random.range(0.0f, 0.25f),
            mid + lead + random.range(0.03f, 0.09f));
        m_channels[16 + arm] = channel(random.range(-10.0f, 10.0f) * Degree, mid + lead);
    }
    // (O pico dos bracos ate 55% do gesto: com o antebraco atrasado, a descida
    // ficava curta no fim do toque e a mao dava 400 m/s2.)
    for (std::size_t k = 10; k < 18; ++k)
        m_channels[k].top = std::max(0.02f, std::min(m_channels[k].top, 0.55f * m_channels[k].end));
    // O giro da cintura ate o rumo da bola: cheio no contato (o pe passa pela
    // bola), seguro no acompanhamento e de volta no fim - sem sorteio.
    m_channels[18] = { b.turn, 0.0f, std::max(0.02f, t1), end, std::max(t1, t2) };
    // O cabeceio: a armada do tronco e da cabeca (pico na armada, some no
    // contato) e a cabeca ao rumo da bola (cheia no contato, segura no
    // acompanhamento) - sem sorteio.
    const float armed = std::max(0.02f, m_keys.windupTime);
    m_channels[23] = { b.windupBack, 0.0f, armed, std::max(armed + 0.02f, t1), -1.0f };
    m_channels[24] = { b.windupHead, 0.0f, armed, std::max(armed + 0.02f, t1), -1.0f };
    m_channels[25] = { b.headYaw, 0.0f, std::max(0.02f, t1), end, std::max(t1, t2) };
    // A equilibrada (24/10, pedido do usuario: "como se ele quisesse dar uma
    // equilibrada, algo suave"): depois do pico, cada braco ainda faz um pulso
    // pequeno sorteado - para fora ou para dentro, a frente ou atras -, suave e
    // terminando com o gesto.
    for (std::size_t arm = 0; arm < 2; ++arm) {
        for (std::size_t part = 0; part < 2; ++part) {
            const float base = std::abs(part == 0 ? b.armOut[arm] : b.armForward[arm]) + 4.0f * Degree;
            Channel c;
            c.peak = random.range(-0.4f, 0.4f) * base;
            c.start = random.range(0.25f, 0.45f) * end;
            c.top = std::min(c.start + random.range(0.15f, 0.3f) * end, 0.75f * end);
            c.end = end;
            m_channels[19 + 2 * arm + part] = c;
        }
    }
}

void GesturePlayer3D::finishSoon(float seconds) {
    if (!m_active || !m_lastValid || m_keys.duration - m_time <= seconds) return;
    const float span = std::max(1e-3f, seconds);
    for (std::size_t i = 0; i < 7; ++i) {
        const float p = m_lastLeg.value[i], v = std::clamp(m_lastLegVelocity.value[i], -VelocityLimit, VelocityLimit);
        m_return[i] = { p, v, 0.0f, p + 0.3f * v * span, 0.0f, 0.0f, span };
    }
    m_returning = true;
    m_keys.windupTime = std::min(m_keys.windupTime, m_time);
    m_keys.contactTime = std::min(m_keys.contactTime, m_time);
    m_keys.followTime = m_time;
    m_keys.duration = m_time + span;
    m_fadeStart = m_time;
    m_fadeEnd = m_time + span;
}

GestureFrame3D GesturePlayer3D::update(float dt, const GestureLegPose3D& live, float moving) {
    GestureFrame3D frame;
    if (!m_active) return frame;
    m_time += std::max(0.0f, dt);
    const float t = m_time, t1 = m_keys.contactTime, t2 = m_keys.followTime, end = m_keys.duration;
    if (t >= end) {
        m_active = false;
        return frame;
    }
    frame.active = true;
    frame.side = m_spec.side;
    frame.time = t;
    frame.duration = end;
    const float tw = m_keys.windupTime;
    if (t < tw) {
        for (std::size_t i = 0; i < 7; ++i) frame.leg.value[i] = m_windup[i].position(t / m_windup[i].h);
    } else if (t < t1) {
        for (std::size_t i = 0; i < 7; ++i) {
            const float s = (t - tw) / m_strike[i].h;
            frame.leg.value[i] = m_strike[i].position(s) + m_lift[i] * liftBump(s);
        }
    } else if (t < t2) {
        for (std::size_t i = 0; i < 7; ++i) frame.leg.value[i] = m_follow[i].position((t - t1) / m_follow[i].h);
    } else {
        // A volta: a pose do gesto, parando, cede para a da locomocao numa
        // mistura de grau 5 (continua na posicao, na velocidade e na
        // aceleracao nas duas pontas; sempre entre duas poses validas).
        // (Somando a diferenca a pose da locomocao, que correndo anda muito
        // nesse tempo, o joelho chegava a -30 graus e o pe entrava 17 cm no
        // chao.)
        const float span = std::max(1e-3f, end - t2);
        if (!m_returning) {
            m_returning = true;
            for (std::size_t i = 0; i < 7; ++i) {
                const GestureQuintic& f = m_follow[i];
                m_return[i] = { f.p1, f.v1, f.a1, f.p1 + 0.3f * f.v1 * span, 0.0f, 0.0f, span };
            }
        }
        const float s = (t - t2) / span, w = gestureSmoothStep5(s);
        for (std::size_t i = 0; i < 7; ++i) {
            const float held = m_return[i].position(s);
            frame.leg.value[i] = held + (live.value[i] - held) * w;
        }
    }
    // (Conduzindo, a perna e do gesto so em parte e por pouco: o peso sobe ate
    // o contato e cede ate o fim - o pe toca e volta aos poucos ao passo.
    // Segura no gesto, a perna ficava parada no ar a frente enquanto a passada
    // a queria plantada e voltava 66 cm para tras em 67 ms no fim.)
    if (m_keys.legWeight < 1.0f) {
        // (A base parte da perna mostrada e chega na da locomocao: da pose crua
        // direto, o pe aberto pelo empurrao de nao cruzar saltava 35 cm.)
        if (!m_startOffsetValid) {
            for (std::size_t i = 0; i < 7; ++i) m_startOffset.value[i] = m_startLeg.value[i] - live.value[i];
            m_startOffsetValid = true;
        }
        const float weight = m_keys.legWeight * gestureBump(t, 0.0f, std::max(0.02f, t1), end);
        const float keep = 1.0f - gestureSmoothStep5(t / std::max(1e-3f, end));
        for (std::size_t i = 0; i < 7; ++i) {
            const float base = live.value[i] + m_startOffset.value[i] * keep;
            frame.leg.value[i] = base + (frame.leg.value[i] - base) * weight;
        }
    }
    if (m_lastValid && dt > 1e-5f)
        for (std::size_t i = 0; i < 7; ++i) m_lastLegVelocity.value[i] = (frame.leg.value[i] - m_lastLeg.value[i]) / dt;
    m_lastLeg = frame.leg;
    m_lastValid = true;
    // Andando, o corpo do gesto entra por cima do da passada, canal por canal
    // (24/10, pedido do usuario: andando faltavam bracos e inclinacao da
    // coluna). A inclinacao para o lado, o giro e os bracos abrindo cheios; o
    // recuo do tronco no fim do chute, os bracos para a frente e para tras, o
    // cotovelo e a pelve, em parte. (Por cima do balanco da corrida, cheios, o
    // tronco endireitava, o braco dobrava para tras e a mao do de fora cobria
    // o rosto.)
    const float go = std::clamp(moving, 0.0f, 1.0f);
    const auto dose = [go](float base) { return 1.0f - (1.0f - base) * go; };
    GestureBody3D& body = frame.body;
    body.pelvisDrop = m_channels[0].at(t) * dose(0.5f);
    body.pelvisForward = m_channels[1].at(t) * dose(0.5f);
    body.pelvisLeft = m_channels[2].at(t) * dose(0.5f);
    const float turn = m_channels[18].at(t);
    body.pelvisYaw = m_channels[3].at(t) * dose(0.8f) + turn;
    body.pelvisRoll = m_channels[4].at(t) * dose(0.8f);
    // A coluna repartida nas tres vertebras (a de cima mais); a cabeca
    // devolve metade da flexao dela (fica olhando a bola).
    // (O peito vai com a cintura no giro ate a bola, quase todo.)
    const float twist = m_channels[5].at(t) * dose(0.9f) - 0.2f * turn;
    const float pitch = m_channels[6].at(t) * dose(0.7f) - m_channels[7].at(t) * dose(0.4f) - m_channels[23].at(t);
    const float roll = m_channels[8].at(t);
    const float shares[3] = { 0.30f, 0.35f, 0.35f };
    for (std::size_t i = 0; i < 3; ++i) body.spine[i] = Vec3 { twist, pitch, roll } * shares[i];
    const float head = m_channels[9].at(t) * dose(0.8f) - m_channels[24].at(t);
    const float aim = m_channels[25].at(t);
    body.neck = { -0.4f * twist + 0.4f * aim, 0.4f * head - 0.3f * pitch, -0.3f * roll };
    body.head = { -0.4f * twist + 0.6f * aim, 0.6f * head - 0.3f * pitch, -0.3f * roll };
    // Os bracos: [0] o do lado da perna do gesto, [1] o outro.
    const std::size_t gestureArm = static_cast<std::size_t>(m_spec.side), otherArm = 1 - gestureArm;
    body.armForward[gestureArm] = (m_channels[10].at(t) + m_channels[20].at(t)) * dose(0.2f);
    body.armForward[otherArm] = (m_channels[11].at(t) + m_channels[22].at(t)) * dose(0.35f);
    body.armOut[gestureArm] = m_channels[12].at(t) + m_channels[19].at(t);
    body.armOut[otherArm] = m_channels[13].at(t) + m_channels[21].at(t);
    body.elbow[gestureArm] = m_channels[14].at(t) * dose(0.3f);
    body.elbow[otherArm] = m_channels[15].at(t) * dose(0.3f);
    body.armAcross[gestureArm] = m_channels[16].at(t) * dose(0.8f);
    body.armAcross[otherArm] = m_channels[17].at(t) * dose(0.8f);
    if (m_fadeStart >= 0.0f) {
        const float keep = 1.0f - gestureSmoothStep5((t - m_fadeStart) / std::max(1e-3f, m_fadeEnd - m_fadeStart));
        body.pelvisDrop *= keep;
        body.pelvisForward *= keep;
        body.pelvisLeft *= keep;
        body.pelvisYaw *= keep;
        body.pelvisRoll *= keep;
        for (Vec3& vertebra : body.spine) vertebra = vertebra * keep;
        body.neck = body.neck * keep;
        body.head = body.head * keep;
        for (std::size_t arm = 0; arm < 2; ++arm) {
            body.armForward[arm] *= keep;
            body.armOut[arm] *= keep;
            body.armAcross[arm] *= keep;
            body.elbow[arm] *= keep;
        }
    }
    return frame;
}

} // namespace MatterEngine
