#include "Engine/Athlete/Dribble/SwingTouch3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float smoother(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

Quaternion slerp(Quaternion a, Quaternion b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0f) {
        b = { -b.x, -b.y, -b.z, -b.w };
        d = -d;
    }
    if (d > 0.9995f)
        return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t }
            .normalized();
    const float angle = std::acos(std::clamp(d, -1.0f, 1.0f));
    const float ka = std::sin((1.0f - t) * angle) / std::sin(angle);
    const float kb = std::sin(t * angle) / std::sin(angle);
    return Quaternion { a.x * ka + b.x * kb, a.y * ka + b.y * kb, a.z * ka + b.z * kb, a.w * ka + b.w * kb }.normalized();
}

Vec3 blendDirection(Vec3 a, Vec3 b, float t) {
    const Vec3 v = a * (1.0f - t) + b * t;
    return v.lengthSquared() > 1e-8f ? v.normalized() : b;
}

Vec3 clampLength(Vec3 v, float most) {
    const float length = v.length();
    return length > most && length > 1e-6f ? v * (most / length) : v;
}

// Um passo da quintica (o minimo tranco) de (p, v, a) ate (target,
// targetVelocity, aceleracao zero) em T s, refeita a cada tick.
void quinticStep(Vec3& p, Vec3& v, Vec3& a, Vec3 target, Vec3 targetVelocity, float T, float dt) {
    if (T <= 1e-5f || dt >= T - 1e-6f) {
        p = target;
        v = targetVelocity;
        a = {};
        return;
    }
    const Vec3 d = target - p;
    const Vec3 v0 = v * T, v1 = targetVelocity * T, a0 = a * (T * T);
    const Vec3 c3 = d * 10.0f - v0 * 6.0f - v1 * 4.0f - a0 * 1.5f;
    const Vec3 c4 = d * -15.0f + v0 * 8.0f + v1 * 7.0f + a0 * 1.5f;
    const Vec3 c5 = d * 6.0f - v0 * 3.0f - v1 * 3.0f - a0 * 0.5f;
    const float s = dt / T;
    const float s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    p = p + v0 * s + a0 * (0.5f * s2) + c3 * s3 + c4 * s4 + c5 * s5;
    const Vec3 nextV = (v0 + a0 * s + c3 * (3.0f * s2) + c4 * (4.0f * s3) + c5 * (5.0f * s4)) * (1.0f / T);
    a = (a0 + c3 * (6.0f * s) + c4 * (12.0f * s2) + c5 * (20.0f * s3)) * (1.0f / (T * T));
    v = nextV;
}

// A quintica que leva uma diferenca (posicao, velocidade e aceleracao no
// comeco) a zero em T s, no instante u (0..1): o valor.
Vec3 quinticDecay(Vec3 offset, Vec3 velocity, Vec3 acceleration, float T, float u) {
    u = std::clamp(u, 0.0f, 1.0f);
    const Vec3 v0 = velocity * T, a0 = acceleration * (T * T);
    const Vec3 c3 = offset * -10.0f - v0 * 6.0f - a0 * 1.5f;
    const Vec3 c4 = offset * 15.0f + v0 * 8.0f + a0 * 1.5f;
    const Vec3 c5 = offset * -6.0f - v0 * 3.0f - a0 * 0.5f;
    const float u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    return offset + v0 * u + a0 * (0.5f * u2) + c3 * u3 + c4 * u4 + c5 * u5;
}

// A dica do joelho (a frente da patela) no plano do chao, pelo rumo do pe
// quando ela fica em pe: inclinada (a do clipe no comeco do balanco aponta
// para a frente e para baixo), ela ficava quase no eixo quadril-tornozelo com
// o pe a frente e o IK virava o joelho num tick (110-145 rad/s).
Vec3 kneeHint(Vec3 knee, float footYaw) {
    const Vec3 toe { std::cos(footYaw), std::sin(footYaw), 0.0f };
    const Vec3 flat = planar(knee);
    const float length = flat.length();
    if (length < 1e-4f) return toe;
    return blendDirection(toe, flat * (1.0f / length), smooth((length - 0.15f) / 0.25f));
}

constexpr float AccelerationMaximum = 60.0f;
// O peso da perna entra e sai nesse tempo (s).
constexpr float BlendSeconds = 0.05f;
// O pe gira no maximo isso do pe do clipe (rad).
constexpr float MaxTwist = 0.9f;
// O joelho gira no maximo isso (rad/s; uma rede: o planejador ja limita o
// giro da perna).
constexpr float KneeRate = 18.0f;

} // namespace

void SwingTouch3D::begin(const TouchPlan3D& plan, int request, float now, const SwingLive3D& live,
    const ContactFeet3D& feet, const DribbleSettings3D& s) {
    m_plan = plan;
    m_request = request;
    m_active = true;
    m_contacted = false;
    m_contactedNow = false;
    m_time = now;
    m_start = now;
    // A aproximacao parte do pe do clipe (posicao, velocidade e aceleracao).
    m_position = live.clipAnkle;
    m_velocity = live.clipVelocity;
    m_acceleration = clampLength(live.clipAcceleration, AccelerationMaximum);
    m_startOrientation = live.clipOrientation;
    m_contactOrientation = plan.pose.orientation;
    // O arco: pelo caminho ate o contato.
    m_lift = std::min(s.liftMaximum, s.liftPerMeter * (plan.pose.ankle - live.clipAnkle).length());
    m_ankle = live.clipAnkle;
    m_ankleVelocity = live.clipVelocity;
    m_orientation = live.clipOrientation;
    m_knee = kneeHint(live.clipKnee, athleteFootHeading3D(feet.geometry, plan.side, live.clipOrientation));
    m_lastUpdate = now;
    m_weight = 0.0f;
    m_lastWeightTime = now;
}

void SwingTouch3D::retarget(const TouchPlan3D& plan) {
    if (!m_active || m_contacted || plan.side != m_plan.side) return;
    m_plan.contact = plan.contact;
    m_plan.end = plan.end;
    m_plan.followEnd = plan.followEnd;
    m_plan.ball = plan.ball;
    m_plan.ballAfter = plan.ballAfter;
    m_plan.ballAfterKnown = plan.ballAfterKnown;
    m_plan.pose = plan.pose;
    m_plan.push = plan.push;
    m_plan.surface = plan.surface;
    m_plan.style = plan.style;
    m_plan.drop = plan.drop;
    m_plan.noShift = plan.noShift;
    m_plan.inArea = plan.inArea;
    m_contactOrientation = plan.pose.orientation;
}

void SwingTouch3D::abort(float now, const SwingLive3D& live, float follow) {
    if (!m_active || m_contacted) return;
    m_contacted = true;
    m_contactedNow = false;
    m_offset = m_ankle - live.clipAnkle;
    m_offsetVelocity = m_velocity - live.clipVelocity;
    m_offsetAcceleration = {};
    m_contactOrientation = m_orientation;
    m_plan.contact = now;
    m_plan.followEnd = std::max(m_plan.followEnd, now + follow);
    m_plan.end = std::max(m_plan.end, m_plan.followEnd);
    m_followSpan = m_plan.followEnd - now;
    m_time = now;
}

void SwingTouch3D::update(float now, const SwingLive3D& live, const ContactFeet3D& feet, const DribbleSettings3D& s,
    DribbleLeg3D& out) {
    m_contactedNow = false;
    if (!m_active) {
        out.weight = 0.0f;
        return;
    }
    const std::size_t side = static_cast<std::size_t>(std::clamp(m_plan.side, 0, 1));
    float left = std::max(0.0f, now - m_time);
    if (!m_contacted && left > 0.0f) {
        // Ate o contato: a quintica ate o pe na bola (o ponto do mundo),
        // chegando na velocidade do corpo no contato mais a da batida.
        const float T = std::max(1e-5f, m_plan.contact - m_time);
        const float ahead = std::max(0.0f, m_plan.contact - now);
        const float speed = m_plan.shot ? s.shotStrikeSpeed + s.shotStrikeSpeedPower * m_plan.power
                                        : s.strikeSpeed + s.strikeSpeedPower * m_plan.power;
        const Vec3 body = live.forecast != nullptr ? live.forecast->velocity(ahead) : live.bodyVelocity;
        // (Nunca mais rapido que a bola depois do toque, no rumo dele: no
        // acompanhamento o pe entrava ate 11 cm na bola.)
        const float ballAhead = dot(planar(m_plan.ballAfter) - planar(body), m_plan.push);
        const float pushSpeed = m_plan.ballAfterKnown ? std::min(speed, std::max(0.3f, ballAhead - 0.1f)) : speed;
        const Vec3 strike = planar(body) + m_plan.push * pushSpeed;
        const float step = std::min(left, T);
        quinticStep(m_position, m_velocity, m_acceleration, m_plan.pose.ankle, strike, T, step);
        m_time += step;
        left -= step;
        if (m_time >= m_plan.contact - 1e-6f) {
            m_contacted = true;
            m_contactedNow = true;
            // A diferenca para o pe do clipe no contato: ela some ate o pouso.
            // (No acompanhamento o pe nao passa da bola: o que ele tem a mais
            // que ela no rumo do toque sai.)
            if (m_plan.ballAfterKnown) {
                const float excess = dot(planar(m_velocity) - planar(m_plan.ballAfter), m_plan.push);
                if (excess > 0.0f) m_velocity -= m_plan.push * excess;
            }
            m_offset = m_position - live.clipAnkle;
            m_offsetVelocity = m_velocity - live.clipVelocity;
            m_offsetAcceleration = clampLength(m_acceleration - live.clipAcceleration, AccelerationMaximum);
            m_followSpan = std::max(0.04f, m_plan.followEnd - m_plan.contact);
            // (O pe passa da bola no maximo followThrough: a quintica que zera
            // a diferenca leva a velocidade da batida ~0,2 v T adiante - com
            // uma volta longa, 20 cm e a perna travada reta.)
            const float through = 0.2f * m_offsetVelocity.length() * m_followSpan;
            const float most = m_plan.shot ? s.shotFollowThrough : s.followThrough;
            if (through > most) m_offsetVelocity = m_offsetVelocity * (most / through);
        }
    }
    if (m_contacted) m_time += left;
    Vec3 ankle;
    float groundShare = 1.0f;
    float weight = 1.0f;
    // (0 no pe do clipe, 1 no contato.)
    float progress = 0.0f;
    if (!m_contacted) {
        const float u = std::clamp((m_time - m_start) / std::max(1e-3f, m_plan.contact - m_start), 0.0f, 1.0f);
        const float bump = m_lift * 16.0f * u * u * (1.0f - u) * (1.0f - u);
        ankle = m_position + Vec3 { 0.0f, 0.0f, bump };
        // O armado do chute parado: o pe vai para tras e para cima no
        // primeiro terco da ida (some no contato sem mudar a velocidade dele).
        if (m_plan.shot && m_plan.step) {
            const float windup = 6.75f * u * (1.0f - u) * (1.0f - u);
            ankle += m_plan.push * (-s.shotWindupBack * windup) + Vec3 { 0.0f, 0.0f, s.shotWindupLift * windup };
        }
        m_orientation = slerp(m_startOrientation, m_contactOrientation, smoother(u));
        weight = smooth((m_time - m_start) / BlendSeconds);
        progress = smoother(u);
    } else {
        const float u = std::clamp((m_time - m_plan.contact) / m_followSpan, 0.0f, 1.0f);
        ankle = live.clipAnkle + quinticDecay(m_offset, m_offsetVelocity, m_offsetAcceleration, m_followSpan, u);
        m_orientation = slerp(m_contactOrientation, live.clipOrientation, smoother(u));
        groundShare = 1.0f - smooth(u / 0.4f);
        progress = 1.0f - smoother(u);
        // (No fim a perna volta para a da locomocao pelo peso: o IK da mesma
        // pose nao e identico a ela.)
        weight = smooth((1.0f - u) * m_followSpan / std::min(BlendSeconds, 0.6f * m_followSpan));
        if (u >= 1.0f) {
            m_active = false;
            out.weight = 0.0f;
            m_ankle = live.clipAnkle;
            m_orientation = live.clipOrientation;
            return;
        }
    }
    // O pe nunca mais girado que MaxTwist do pe do clipe agora (o giro que a
    // coxa da): com o corpo virando depois do compromisso - a camera girando -
    // o pe do plano chegava a ficar de costas para o clipe e o joelho
    // apontava para tras, a perna esticada girando 200 rad/s.
    {
        const float clipYaw = athleteFootHeading3D(feet.geometry, m_plan.side, live.clipOrientation);
        const float off = athleteWrapAngle3D(athleteFootHeading3D(feet.geometry, m_plan.side, m_orientation) - clipYaw);
        const float excess = off - std::clamp(off, -MaxTwist, MaxTwist);
        if (std::abs(excess) > 1e-4f)
            m_orientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -excess) * m_orientation).normalized();
    }
    // O joelho: no rumo do pe, com a diferenca joelho-pe do clipe sumindo ate
    // o contato e voltando depois - e o do clipe no comeco e no fim e, no
    // contato, aponta para onde o pe aponta. (O tornozelo do boneco quase nao
    // gira de lado - so dobra e inclina -: o giro do pe e o da perna. Com o
    // joelho levando so parte do giro, o pe chegava ate 30 graus fora do
    // plano e a beira dele ficava 5 cm longe da bola.)
    // (Girando no maximo KneeRate: nada de virar o joelho num tick.)
    {
        const float clipYaw = athleteFootHeading3D(feet.geometry, m_plan.side, live.clipOrientation);
        const float footYaw = athleteFootHeading3D(feet.geometry, m_plan.side, m_orientation);
        const Vec3 clipKnee = kneeHint(live.clipKnee, clipYaw);
        const float offset = athleteWrapAngle3D(std::atan2(clipKnee.y, clipKnee.x) - clipYaw);
        const float yaw = footYaw + offset * (1.0f - progress);
        const Vec3 want { std::cos(yaw), std::sin(yaw), 0.0f };
        const float turn = athleteWrapAngle3D(std::atan2(want.y, want.x) - std::atan2(m_knee.y, m_knee.x));
        const float most = KneeRate * std::max(0.0f, now - m_lastUpdate);
        m_knee = rotateZ(planar(m_knee).normalized(), std::clamp(turn, -most, most));
    }
    m_lastUpdate = now;
    // A sola nunca abaixo do chao no ar (perto do pouso a trava some: ele pousa
    // com o clipe).
    if (groundShare > 0.0f && feet.valid) {
        const Vec3 foot = ankle - m_orientation.rotate(feet.legs.ankleInFoot[side]);
        const float lowest = std::min((foot + m_orientation.rotate(feet.geometry.heel[side])).z,
            (foot + m_orientation.rotate(feet.geometry.toeBase[side])).z);
        const float need = live.groundHeight + s.clearance - lowest;
        if (need > 0.0f) ankle.z += need * groundShare;
    }
    // A perna nunca mais esticada que a do clipe ou que swingReach (o que for
    // maior), cedendo perto do limite.
    if (feet.valid) {
        const float length = feet.legLength[side];
        const float most = std::max((live.clipAnkle - live.hip).length() + 0.01f * length, s.swingReach * length);
        const float band = std::max(1e-3f, s.reachBand * length);
        const Vec3 fromHip = ankle - live.hip;
        const float distance = fromHip.length();
        if (distance > most - band && distance > 1e-6f)
            ankle = live.hip + fromHip * ((most - band + band * (1.0f - std::exp(-(distance - most + band) / band))) / distance);
    }
    // (O peso so sobe na rampa: desistindo no comeco do balanco ele pulava
    // de 0,26 para 1.)
    weight = std::min(weight, m_weight + std::max(0.0f, now - m_lastWeightTime) / BlendSeconds);
    m_weight = weight;
    m_lastWeightTime = now;
    m_ankle = ankle;
    out.ankle = ankle;
    out.knee = m_knee;
    out.foot = m_orientation;
    out.weight = weight;
}

} // namespace MatterEngine
