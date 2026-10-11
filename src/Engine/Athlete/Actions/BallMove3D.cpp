#include "Engine/Athlete/Actions/BallMove3D.hpp"

#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

Quaternion slerp(Quaternion a, Quaternion b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0f) {
        b = { -b.x, -b.y, -b.z, -b.w };
        d = -d;
    }
    if (d > 0.9995f)
        return Quaternion { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
            a.w + (b.w - a.w) * t }.normalized();
    const float angle = std::acos(std::clamp(d, -1.0f, 1.0f));
    const float ka = std::sin((1.0f - t) * angle) / std::sin(angle);
    const float kb = std::sin(t * angle) / std::sin(angle);
    return Quaternion { a.x * ka + b.x * kb, a.y * ka + b.y * kb, a.z * ka + b.z * kb, a.w * ka + b.w * kb }
        .normalized();
}

// Um passo do Hermite cubico de (p, v) ate (target, targetVelocity) em T s.
void hermiteStep(Vec3& p, Vec3& v, Vec3 target, Vec3 targetVelocity, float T, float dt) {
    if (T <= dt + 1e-5f) {
        p = target;
        v = targetVelocity;
        return;
    }
    const float s = dt / T;
    const float s2 = s * s, s3 = s2 * s;
    const Vec3 nextP = p * (2.0f * s3 - 3.0f * s2 + 1.0f) + v * (T * (s3 - 2.0f * s2 + s))
        + target * (-2.0f * s3 + 3.0f * s2) + targetVelocity * (T * (s3 - s2));
    const Vec3 nextV = p * ((6.0f * s2 - 6.0f * s) / T) + v * (3.0f * s2 - 4.0f * s + 1.0f)
        + target * ((-6.0f * s2 + 6.0f * s) / T) + targetVelocity * (3.0f * s2 - 2.0f * s);
    p = nextP;
    v = nextV;
}

// Do referencial do corpo (x frente, y esquerda) ao mundo, e de volta.
Vec3 toWorld(Vec3 local, float yaw) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return { c * local.x - s * local.y, s * local.x + c * local.y, local.z };
}
Vec3 toLocal(Vec3 world, float yaw) { return toWorld(world, -yaw); }

// A forma de cada toque para a perna direita, em (frente, para fora, altura
// acima do tornozelo plano): a passagem e o rumo nela, o fim do
// acompanhamento, o pouso parado (no plano) e as orientacoes do pe (guinada
// para fora, arfagem com a ponta para baixo, inversao - a sola para dentro).
struct Shape {
    Vec3 strike;
    Vec3 direction;
    Vec3 follow;
    Vec3 land;
    std::array<float, 3> angles;
    bool over = false;
    Vec3 overPoint;
};

Shape shapeOf(BallMoveTouch3D touch) {
    switch (touch) {
    case BallMoveTouch3D::Front:
        // O peito do pe empurrando a frente.
        // (Com a ponta para baixo, o tornozelo mais alto: a ponta raspava.)
        return { { 0.28f, 0.11f, 0.09f }, { 1.0f, 0.0f, 0.05f }, { 0.36f, 0.11f, 0.10f }, { 0.18f, 0.12f, 0.0f },
            { 0.0f, 0.45f, 0.0f } };
    case BallMoveTouch3D::FrontOut:
        // A parte de fora, a frente e para fora.
        return { { 0.24f, 0.28f, 0.07f }, { 0.7f, 0.7f, 0.0f }, { 0.31f, 0.35f, 0.07f }, { 0.16f, 0.20f, 0.0f },
            { -0.15f, 0.30f, 0.25f } };
    case BallMoveTouch3D::Out:
        // A parte de fora, de lado: o pe abre para fora da perna.
        return { { 0.06f, 0.33f, 0.06f }, { 0.1f, 1.0f, 0.0f }, { 0.08f, 0.41f, 0.06f }, { 0.04f, 0.22f, 0.0f },
            { -0.20f, 0.10f, 0.30f } };
    case BallMoveTouch3D::BackOut: {
        // A sola por cima da bola, puxando para tras e para fora.
        Shape shape { { 0.00f, 0.24f, 0.07f }, { -0.8f, 0.6f, 0.0f }, { -0.07f, 0.28f, 0.05f }, { -0.04f, 0.18f, 0.0f },
            { 0.25f, -0.50f, 0.0f } };
        shape.over = true;
        shape.overPoint = { 0.20f, 0.16f, 0.19f };
        return shape;
    }
    case BallMoveTouch3D::Back: {
        // A sola por cima da bola, puxando para tras.
        Shape shape { { 0.02f, 0.11f, 0.07f }, { -1.0f, 0.0f, 0.0f }, { -0.07f, 0.11f, 0.05f }, { -0.04f, 0.12f, 0.0f },
            { 0.0f, -0.55f, 0.0f } };
        shape.over = true;
        shape.overPoint = { 0.22f, 0.11f, 0.20f };
        return shape;
    }
    case BallMoveTouch3D::Shot:
        // O chute: a perna vai direto para a frente (a finalizacao sai da
        // intensidade, no start).
        return { { 0.22f, 0.10f, 0.10f }, { 1.0f, 0.0f, 0.15f }, { 0.42f, 0.08f, 0.22f }, { 0.26f, 0.12f, 0.0f },
            { 0.0f, 0.60f, 0.0f } };
    }
    return {};
}

} // namespace

BallMoveChoice3D chooseBallMove3D(BallActionKind3D kind, Vec3 touchDirection, float facingYaw, float ballLateral,
    int freeSide) {
    BallMoveChoice3D choice;
    // A perna do lado da bola (perto do meio, a livre; senao a direita).
    const int ballSide = std::abs(ballLateral) >= 0.05f ? (ballLateral > 0.0f ? 0 : 1) : freeSide >= 0 ? freeSide : 1;
    if (kind == BallActionKind3D::Shot) {
        choice.side = ballSide;
        choice.touch = BallMoveTouch3D::Shot;
        choice.sector = 0;
        choice.surface = BallTouchSurface3D::Laces;
        return choice;
    }
    const Vec3 flat = planar(touchDirection);
    const Vec3 local = flat.lengthSquared() > 1e-6f ? toLocal(flat.normalized(), facingYaw) : Vec3 { 1.0f, 0.0f, 0.0f };
    const float angle = std::atan2(local.y, local.x);
    int sector = static_cast<int>(std::lround(angle / (Pi * 0.25f)));
    sector = ((sector % 8) + 8) % 8;
    choice.sector = sector;
    switch (sector) {
    case 0: choice.side = ballSide; choice.touch = BallMoveTouch3D::Front; break;
    case 1: choice.side = 0; choice.touch = BallMoveTouch3D::FrontOut; break;
    case 2: choice.side = 0; choice.touch = BallMoveTouch3D::Out; break;
    case 3: choice.side = 0; choice.touch = BallMoveTouch3D::BackOut; break;
    case 4: choice.side = ballSide; choice.touch = BallMoveTouch3D::Back; break;
    case 5: choice.side = 1; choice.touch = BallMoveTouch3D::BackOut; break;
    case 6: choice.side = 1; choice.touch = BallMoveTouch3D::Out; break;
    default: choice.side = 1; choice.touch = BallMoveTouch3D::FrontOut; break;
    }
    const bool sole = choice.touch == BallMoveTouch3D::Back || choice.touch == BallMoveTouch3D::BackOut;
    choice.surface = sole ? BallTouchSurface3D::Sole
        : choice.touch == BallMoveTouch3D::Front ? BallTouchSurface3D::Laces : BallTouchSurface3D::Outside;
    return choice;
}

Quaternion BallMove3D::localOrientation(float yaw, float pitch, float roll) const {
    // (Para a perna direita, para fora e a direita do corpo e a inversao
    // levanta a borda de dentro, a esquerda; espelhado na esquerda.)
    const float out = m_choice.side == 1 ? -1.0f : 1.0f;
    const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, m_facing + yaw * out);
    const Quaternion tilt = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, pitch);
    const Quaternion edge = Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -roll * out);
    return (turn * tilt * edge * m_flat).normalized();
}

void BallMove3D::start(const BallMoveStart3D& in, const BallMoveSettings3D& s) {
    m_choice = in.choice;
    m_power = std::clamp(in.power, 0.0f, 1.0f);
    m_flat = in.flatFoot;
    m_ankleInFoot = in.ankleInFoot;
    m_ankleHeight = in.ankleHeight;
    m_legLength = in.legLength;
    m_facing = in.facingYaw;
    m_time = 0.0f;
    m_phaseStart = 0.0f;
    m_landed = false;
    const bool shot = m_choice.touch == BallMoveTouch3D::Shot;
    const float p = m_power;
    Shape shape = shapeOf(m_choice.touch);
    if (shot) {
        // A intensidade: a finalizacao mais alta e mais longe.
        shape.follow.x = s.shotFollowAhead + s.shotFollowAheadPower * p;
        shape.follow.z = s.shotFollowHeight + s.shotFollowHeightPower * p;
        shape.angles[1] = 0.55f + 0.2f * p;
    }
    // Da forma (para fora) ao referencial do corpo (y esquerda) e a altura do
    // tornozelo.
    const float out = m_choice.side == 0 ? 1.0f : -1.0f;
    const auto place = [&](Vec3 v) { return Vec3 { v.x, v.y * out, m_ankleHeight + v.z }; };
    m_strike = place(shape.strike);
    m_follow = place(shape.follow);
    m_over = shape.over;
    m_overPoint = place(shape.overPoint);
    Vec3 direction { shape.direction.x, shape.direction.y * out, shape.direction.z };
    direction = direction.normalized();
    const float speed = shot ? s.shotStrikeSpeed + s.shotStrikeSpeedPower * p
                             : s.touchStrikeSpeed + s.touchStrikeSpeedPower * p;
    m_strikeVelocity = direction * speed;
    m_strikeAngles = shape.angles;
    // O arco do chute: ate quanto a perna sobe (rad a partir da vertical para
    // baixo, no plano do chute), pela intensidade.
    m_arcLive = false;
    m_arcEnd = s.shotArcEnd + s.shotArcEndPower * p;
    m_landAngles = { 0.05f, 0.0f, 0.0f };
    m_landTarget = place(shape.land);
    m_landTarget.z = 0.0f;
    m_landLocal = { m_landTarget.x, m_landTarget.y, m_ankleHeight };
    m_strikeAt = shot ? s.shotStrikeSeconds : s.touchStrikeSeconds;
    m_followAt = m_strikeAt + (shot ? s.shotFollowSeconds + s.shotFollowSecondsPower * p : s.touchFollowSeconds);
    m_landAt = m_followAt + (shot ? s.shotLandSeconds : s.touchLandSeconds);
    m_push = toWorld(Vec3 { direction.x, direction.y, 0.0f }, m_facing);
    // O estado de agora no referencial do corpo (a velocidade sem a do corpo).
    const Vec3 ankle = in.foot + in.footOrientation.rotate(m_ankleInFoot);
    m_local = toLocal(ankle - in.pelvisGround, m_facing);
    // (So uma parte da velocidade contra o corpo: o pe apoiado anda para tras
    // na velocidade da corrida, e com ela inteira ele ia mais para tras antes
    // de voltar - a perna ficava esticada atras. O pe que sai do chao ja vem.)
    m_localVelocity = toLocal(planar(in.footVelocity - in.bodyVelocity) * (in.footPlanted ? s.startVelocityShare : 1.0f)
            + Vec3 { 0.0f, 0.0f, in.footVelocity.z },
        m_facing);
    // (O chute sai do chao subindo: rente, o pe raspava no comeco.)
    if (shot) m_localVelocity.z = std::max(m_localVelocity.z, s.shotLiftSpeed);
    // O pe saindo de longe (atras do corpo, do outro lado) leva um pouco mais
    // ate a passagem: sem isso ele disparava rente ao chao.
    {
        const Vec3 way { m_strike.x - m_local.x, m_strike.y - m_local.y, 0.0f };
        const float most = shot ? s.shotTravelSpeed : s.touchTravelSpeed;
        const float extra = std::max(0.0f, way.length() / std::max(0.5f, most) - m_strikeAt);
        m_strikeAt += extra;
        m_followAt += extra;
        m_landAt += extra;
    }
    // Andando, a duracao e a da passada (o pe do toque pousa quando o clipe o
    // pousaria): os tempos todos na mesma proporcao.
    if (in.totalSeconds > 0.0f && m_landAt > 1e-3f) {
        const float scale = in.totalSeconds / m_landAt;
        m_strikeAt *= scale;
        m_followAt *= scale;
        m_landAt *= scale;
    }
    // O pe sobe antes da passagem (um arco, nao rente ao chao): no meio do
    // caminho, mais alto; nos toques de sola, por cima da bola.
    m_overAt = 0.45f * m_strikeAt;
    // (O chute nao sobe antes: o pe passa baixo pela bola e sobe na
    // finalizacao - subindo antes, era um joelho levantando.)
    if (!m_over && !shot) {
        m_over = true;
        // (Subindo primeiro: o alto a 30% do caminho no plano - mais longe, o
        // pe andava rente ao chao no comeco.)
        m_overPoint = { m_local.x + 0.3f * (m_strike.x - m_local.x), m_local.y + 0.3f * (m_strike.y - m_local.y),
            std::max(m_strike.z, m_ankleHeight + s.liftHeight) };
    }
    m_orientation = in.footOrientation;
    m_phaseOrientation = in.footOrientation;
    m_foot = in.foot;
    m_velocity = in.footVelocity;
    m_phase = BallMovePhase3D::Strike;
}

void BallMove3D::beginLand(const BallMoveLive3D& live, const BallMoveSettings3D& s) {
    // O pouso no mundo: parte de onde o pe esta (com a velocidade dele) e para
    // no chao, onde a passada continua.
    // (De onde o pe estava de fato - com o alcance aplicado -, nao do estado
    // por dentro: dele, o alvo saltava ate 9 cm na troca.)
    m_worldAnkle = ankle();
    m_worldVelocity = m_velocity;
    const float landSeconds = std::max(0.02f, m_landAt - m_time);
    Vec3 target = m_landTarget;
    // Andando: a frente no rumo do movimento.
    const Vec3 travel = toLocal(planar(live.bodyVelocity), live.facingYaw);
    const float speed = travel.length();
    if (speed > 0.3f) target += travel * (std::min(s.landLeadMaximum, s.landLeadSeconds * speed) / speed);
    // Do lado da perna e ao alcance do quadril (no plano).
    const float out = m_choice.side == 0 ? 1.0f : -1.0f;
    if (target.y * out < s.landSideMinimum) target.y = s.landSideMinimum * out;
    // (O alcance de verdade: com o quadril alto - em pe as pernas quase retas -
    // o tornozelo no chao so vai uns 15 cm para o lado; longe disso a perna
    // travava esticada.)
    const Vec3 hipLocal = toLocal(live.hip - live.pelvisGround, live.facingYaw);
    const float height = std::max(0.0f, hipLocal.z - m_ankleHeight);
    const float leg = s.landLegReach * m_legLength;
    const float across = std::sqrt(std::max(0.0f, leg * leg - height * height));
    const float most = std::min(s.landReach, std::max(0.06f, across));
    Vec3 fromHip { target.x - hipLocal.x, target.y - hipLocal.y, 0.0f };
    if (fromHip.length() > most) {
        target.x = hipLocal.x + fromHip.x * (most / fromHip.length());
        target.y = hipLocal.y + fromHip.y * (most / fromHip.length());
    }
    // Do lado dele contra o pe de apoio (as pernas nunca se cruzam, mesmo com
    // o corpo virando no meio do movimento).
    if (live.supportKnown) {
        const Vec3 support = toLocal(live.support - live.pelvisGround, live.facingYaw);
        const float lateral = (target.y - support.y) * out;
        if (lateral < s.landGap) target.y = support.y + s.landGap * out;
    }
    target.z = m_ankleHeight;
    // (O corpo anda ate o pouso: o alvo vai com ele.)
    m_landTarget = live.pelvisGround + planar(live.bodyVelocity) * landSeconds + toWorld(target, live.facingYaw);
    m_phase = BallMovePhase3D::Land;
    m_phaseStart = m_time;
    m_phaseOrientation = m_orientation;
}

void BallMove3D::hurry(const BallMoveLive3D& live, const BallMoveSettings3D& s) {
    if (m_phase != BallMovePhase3D::Strike && m_phase != BallMovePhase3D::Follow) return;
    // (O pouso no tempo dele a partir de agora.)
    const bool shot = m_choice.touch == BallMoveTouch3D::Shot;
    m_landAt = m_time + (shot ? s.shotLandSeconds : s.touchLandSeconds);
    beginLand(live, s);
}

void BallMove3D::update(float deltaTime, const BallMoveLive3D& live, const BallMoveSettings3D& s) {
    m_landed = false;
    if (m_phase == BallMovePhase3D::Idle) return;
    const float dt = std::max(deltaTime, 1e-4f);
    m_facing = live.facingYaw;
    const float before = m_time;
    m_time += dt;
    const auto orient = [&](std::array<float, 3> angles, float end) {
        const float span = std::max(1e-3f, end - m_phaseStart);
        m_orientation = slerp(m_phaseOrientation, localOrientation(angles[0], angles[1], angles[2]),
            smooth((m_time - m_phaseStart) / span));
    };
    switch (m_phase) {
    case BallMovePhase3D::Strike:
        if (m_over && before < m_overAt) {
            // A sola sobe por cima da bola antes de puxar.
            const Vec3 toward = (m_strike - m_overPoint) * (0.5f / std::max(0.02f, m_strikeAt - m_overAt));
            hermiteStep(m_local, m_localVelocity, m_overPoint, toward, m_overAt - before, dt);
        } else {
            hermiteStep(m_local, m_localVelocity, m_strike, m_strikeVelocity, m_strikeAt - before, dt);
        }
        orient(m_strikeAngles, m_strikeAt);
        if (m_time >= m_strikeAt) {
            m_phase = BallMovePhase3D::Follow;
            m_phaseStart = m_time;
            m_phaseOrientation = m_orientation;
        }
        break;
    case BallMovePhase3D::Follow:
        if (m_choice.touch == BallMoveTouch3D::Shot) {
            // O chute: a finalizacao e um arco da perna quase estendida em
            // volta do quadril (em linha reta, a linha passava perto do quadril
            // e o joelho dobrava - era um joelho levantando).
            const Vec3 hip = toLocal(live.hip - live.pelvisGround, live.facingYaw);
            if (!m_arcLive) {
                const Vec3 from = m_local - hip;
                m_arcSide = from.y;
                m_arcRadius = std::max(0.3f, std::sqrt(from.x * from.x + from.z * from.z));
                m_arcAngle = std::atan2(from.x, -from.z);
                // (A velocidade do angulo: a do pe no rumo do arco.)
                const Vec3 tangent { std::cos(m_arcAngle), 0.0f, std::sin(m_arcAngle) };
                m_arcRate = std::max(0.0f, dot(m_localVelocity, tangent)) / m_arcRadius;
                m_arcLive = true;
            }
            const float radiusEnd = s.shotArcReach * m_legLength;
            const float span = std::max(0.03f, m_followAt - m_strikeAt);
            const float u = smooth((m_time - m_strikeAt) / span);
            const float radius = m_arcRadius + (radiusEnd - m_arcRadius) * u;
            float rate = m_arcRate;
            float angle = m_arcAngle;
            // (O angulo num Hermite de 1 dimensao ate o fim, parado.)
            {
                const float T = m_followAt - before;
                if (T <= dt + 1e-5f) {
                    angle = m_arcEnd;
                    rate = 0.0f;
                } else {
                    const float s1 = dt / T, s2 = s1 * s1, s3 = s2 * s1;
                    const float next = angle * (2.0f * s3 - 3.0f * s2 + 1.0f) + rate * (T * (s3 - 2.0f * s2 + s1))
                        + m_arcEnd * (-2.0f * s3 + 3.0f * s2);
                    rate = angle * ((6.0f * s2 - 6.0f * s1) / T) + rate * (3.0f * s2 - 4.0f * s1 + 1.0f)
                        + m_arcEnd * ((-6.0f * s2 + 6.0f * s1) / T);
                    angle = next;
                }
            }
            m_arcAngle = angle;
            m_arcRate = rate;
            const Vec3 previous = m_local;
            m_local = hip + Vec3 { radius * std::sin(angle), m_arcSide, -radius * std::cos(angle) };
            m_localVelocity = (m_local - previous) * (1.0f / dt);
        } else {
            // (O acompanhamento acaba ja indo para o pouso: parando nele e
            // voltando, o pe dava o tranco na troca.)
            const float landSeconds = std::max(0.03f, m_landAt - m_followAt);
            const Vec3 toward { (m_landLocal.x - m_follow.x) / landSeconds * s.followIntoLand,
                (m_landLocal.y - m_follow.y) / landSeconds * s.followIntoLand, 0.0f };
            hermiteStep(m_local, m_localVelocity, m_follow, toward, m_followAt - before, dt);
        }
        orient(m_strikeAngles, m_followAt);
        if (m_time >= m_followAt) beginLand(live, s);
        break;
    case BallMovePhase3D::Land: {
        hermiteStep(m_worldAnkle, m_worldVelocity, m_landTarget, {}, m_landAt - before, dt);
        orient(m_landAngles, m_landAt);
        // (O tornozelo nunca abaixo do pe plano no chao.)
        if (m_time >= m_landAt) {
            m_worldAnkle = m_landTarget;
            m_worldVelocity = {};
            m_landed = true;
            m_phase = BallMovePhase3D::Idle;
        }
        m_local = toLocal(m_worldAnkle - live.pelvisGround, live.facingYaw);
        break;
    }
    case BallMovePhase3D::Idle:
        break;
    }
    // O tornozelo e o pe no mundo.
    Vec3 ankleWorld;
    Vec3 ankleVelocity;
    if (m_phase == BallMovePhase3D::Land || m_landed) {
        ankleWorld = m_worldAnkle;
        ankleVelocity = m_worldVelocity;
    } else {
        ankleWorld = live.pelvisGround + toWorld(m_local, live.facingYaw);
        ankleVelocity = planar(live.bodyVelocity) + toWorld(m_localVelocity, live.facingYaw);
    }
    ankleWorld.z = std::max(ankleWorld.z, live.pelvisGround.z + 0.8f * m_ankleHeight);
    {
        // Ao alcance do quadril (no ar; o pouso ja fica nele): o tornozelo vem
        // para o quadril o que passar.
        const Vec3 fromHip = ankleWorld - live.hip;
        const float most = s.reach * m_legLength;
        if (m_phase != BallMovePhase3D::Land && !m_landed && fromHip.length() > most)
            ankleWorld = live.hip + fromHip * (most / fromHip.length());
    }
    m_foot = ankleWorld - m_orientation.rotate(m_ankleInFoot);
    m_velocity = ankleVelocity;
}

} // namespace MatterEngine
