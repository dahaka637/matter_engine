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

// Um passo da quintica (o minimo tranco) de (p, v, a) ate (target,
// targetVelocity, aceleracao zero) em T s: refeita a cada tick, a aceleracao
// segue continua entre as fases (a cubica dava saltos nela - o tranco que o
// usuario via, plano v4).
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
    const Vec3 nextP = p + v0 * s + a0 * (0.5f * s2) + c3 * s3 + c4 * s4 + c5 * s5;
    const Vec3 nextV = (v0 + a0 * s + c3 * (3.0f * s2) + c4 * (4.0f * s3) + c5 * (5.0f * s4)) * (1.0f / T);
    const Vec3 nextA = (a0 + c3 * (6.0f * s) + c4 * (12.0f * s2) + c5 * (20.0f * s3)) * (1.0f / (T * T));
    p = nextP;
    v = nextV;
    a = nextA;
}

// A quintica que leva uma diferenca (posicao, velocidade e aceleracao no
// comeco) a zero em T s, no instante u (0..1): o valor e a velocidade.
void quinticDecay(Vec3 offset, Vec3 velocity, Vec3 acceleration, float T, float u, Vec3& value, Vec3& rate) {
    u = std::clamp(u, 0.0f, 1.0f);
    const Vec3 v0 = velocity * T, a0 = acceleration * (T * T);
    const Vec3 c3 = offset * -10.0f - v0 * 6.0f - a0 * 1.5f;
    const Vec3 c4 = offset * 15.0f + v0 * 8.0f + a0 * 1.5f;
    const Vec3 c5 = offset * -6.0f - v0 * 3.0f - a0 * 0.5f;
    const float u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    value = offset + v0 * u + a0 * (0.5f * u2) + c3 * u3 + c4 * u4 + c5 * u5;
    rate = (v0 + a0 * u + c3 * (3.0f * u2) + c4 * (4.0f * u3) + c5 * (5.0f * u4)) * (1.0f / std::max(1e-3f, T));
}

// A mistura suave de quinto grau (velocidade e aceleracao zero nas pontas).
float smoother(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
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
    m_heel = in.heel;
    m_toe = in.toe;
    m_ankleHeight = in.ankleHeight;
    m_legLength = in.legLength;
    m_facing = in.facingYaw;
    m_time = 0.0f;
    m_phaseStart = 0.0f;
    m_landed = false;
    m_joined = false;
    m_plantAfterLand = false;
    m_hurried = false;
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
    m_localAcceleration = {};
    m_startLocal = m_local;
    m_startReach = (ankle - in.hip).length() / std::max(0.1f, m_legLength);
    // (O chute sai do chao subindo: rente, o pe raspava no comeco.)
    if (shot) m_localVelocity.z = std::max(m_localVelocity.z, s.shotLiftSpeed);
    // O pe ja indo depressa no rumo do toque e alem da passagem (correndo, o
    // pe do clipe no fim do balanco, la na frente): a passagem vai adiante o
    // que ele ainda anda ate parar - voltando para ela, o pe freava de uma vez
    // e dava a volta.
    float coming = 0.0f, passing = 0.0f;
    {
        const Vec3 along { direction.x, direction.y, 0.0f };
        const float length = along.length();
        if (length > 0.3f) {
            const Vec3 unit = along * (1.0f / length);
            coming = std::max(0.0f, dot(planar(m_localVelocity), unit));
            passing = std::max(0.0f, dot(planar(m_strikeVelocity), unit));
            // (O que ele anda ate cair da velocidade dele para a da passagem,
            // freando em strikeBrake.)
            const float coast = coming > passing
                ? (coming * coming - passing * passing) / (2.0f * std::max(1.0f, s.strikeBrake)) : 0.0f;
            const float beyond = dot(planar(m_local - m_strike), unit) + coast;
            if (beyond > 0.0f) {
                m_strike += unit * beyond;
                m_follow += unit * beyond;
            }
        }
    }
    // O tempo ate a passagem: saindo de longe (atras do corpo, do outro lado),
    // o pe leva mais (sem isso ele disparava rente ao chao); ja vindo depressa
    // no rumo dela, ele chega antes (no ritmo dele - esperando o tempo
    // inteiro, freava e acelerava de novo).
    // (Nos toques de sola o caminho passa pelo alto, por cima da bola.)
    const float overWay = m_over ? (m_overPoint - m_local).length() : 0.0f;
    {
        const float way = m_over ? overWay + (m_strike - m_overPoint).length() : planar(m_strike - m_local).length();
        const float most = std::max(0.5f, shot ? s.shotTravelSpeed : s.touchTravelSpeed);
        const float average = 0.5f * (coming + passing);
        float strikeAt = std::max(m_strikeAt, way / most);
        if (average > most)
            strikeAt = std::clamp(way / average, std::min(s.strikeMinimumSeconds, m_strikeAt), strikeAt);
        const float change = strikeAt - m_strikeAt;
        m_strikeAt += change;
        m_followAt += change;
        m_landAt += change;
    }
    // Andando, com o pe do clipe no balanco: o gesto termina perto do pouso
    // dele (o pe nao pula o apoio dele - sem pe no chao, o corpo flutuava).
    if (!shot && in.clipTouchdownSeconds > 0.0f) {
        const float fit = std::max(in.clipTouchdownSeconds + s.fitTolerance, s.fitMinimumSeconds);
        if (m_followAt > fit) {
            const float scale = fit / m_followAt;
            const float land = m_landAt - m_followAt;
            m_strikeAt *= scale;
            m_followAt *= scale;
            m_landAt = m_followAt + land;
        }
    }
    // O pe sobe antes da passagem (um arco, nao rente ao chao): no meio do
    // caminho, mais alto; nos toques de sola, por cima da bola (o tempo de
    // cada trecho pelo tamanho dele).
    m_overAt = 0.45f * m_strikeAt;
    if (m_over) {
        const float total = overWay + (m_strike - m_overPoint).length();
        if (total > 1e-3f) m_overAt = std::clamp(overWay / total, 0.3f, 0.8f) * m_strikeAt;
    }
    // (O chute nao sobe antes: o pe passa baixo pela bola e sobe na
    // finalizacao - subindo antes, era um joelho levantando.)
    // (So com o pe abaixo dele: o pe ja no alto do balanco mergulhava ate ele.)
    if (!m_over && !shot && m_local.z < std::max(m_strike.z, m_ankleHeight + s.liftHeight) - 0.02f) {
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
    m_worldAcceleration = {};
    m_phase = BallMovePhase3D::Strike;
}

float BallMove3D::rejoinProgress() const {
    if (m_phase == BallMovePhase3D::Rejoin)
        return std::clamp((m_time - m_phaseStart) / std::max(1e-3f, m_rejoinSpan), 0.0f, 1.0f);
    return m_joined ? 1.0f : 0.0f;
}

float BallMove3D::soleLift(Vec3 ankle, Quaternion orientation, float ground, const BallMoveSettings3D& s) const {
    // O que falta para a sola (o calcanhar e a base dos dedos) ficar a
    // soleClearance do chao.
    const Vec3 foot = ankle - orientation.rotate(m_ankleInFoot);
    const float lowest = std::min((foot + orientation.rotate(m_heel)).z, (foot + orientation.rotate(m_toe)).z);
    return std::max(0.0f, ground + s.soleClearance - lowest);
}

void BallMove3D::worldState(const BallMoveLive3D& live, const BallMoveSettings3D& s, Quaternion orientation,
    Vec3& position, Vec3& velocity) const {
    // Do referencial do corpo ao mundo, com o chao e o alcance do quadril.
    position = live.pelvisGround + toWorld(m_local, live.facingYaw);
    velocity = planar(live.bodyVelocity) + toWorld(m_localVelocity, live.facingYaw);
    // (A sola nunca raspa o chao no gesto: com a ponta para baixo e o pe
    // indo depressa, os dedos arrastavam.)
    position.z += soleLift(position, orientation, live.pelvisGround.z, s);
    // Ao alcance do quadril: perto do limite o tornozelo vai cedendo para o
    // quadril, sem passar dele (cortado de uma vez, o pe dava um tranco ao
    // chegar no limite).
    // (O limite parte do alcance que o pe ja tinha - o pe do clipe atras, com
    // a perna esticada, subia de uma vez no primeiro tick - e desce ate o de
    // reach na passagem.)
    const Vec3 fromHip = position - live.hip;
    const float distance = fromHip.length();
    const float reach = s.reach
        + std::max(0.0f, m_startReach - s.reach) * (1.0f - smooth(m_time / std::max(0.02f, m_strikeAt)));
    const float most = reach * m_legLength;
    const float band = std::max(1e-3f, s.reachBand * m_legLength);
    const float start = most - band;
    if (distance > start && distance > 1e-6f)
        position = live.hip + fromHip * ((start + band * (1.0f - std::exp(-(distance - start) / band))) / distance);
    // Do lado dele contra o pe de apoio (as pernas nunca se cruzam): perto
    // dele no rumo do corpo, pelo menos moveGap de lado - andando de lado, o
    // corpo passa por cima do apoio plantado e o desenho, no referencial do
    // corpo, ia por cima dele. (Por ultimo: o alcance puxava o pe para dentro.)
    // (Entrando aos poucos na primeira parte da passagem: o pe que parte
    // cruzado - o passo cruzado andando de lado - saltava 40 cm de uma vez.)
    if (live.supportKnown) {
        const Vec3 local = toLocal(position - live.pelvisGround, live.facingYaw);
        const Vec3 support = toLocal(live.support - live.pelvisGround, live.facingYaw);
        const float out = m_choice.side == 0 ? 1.0f : -1.0f;
        const float near = 1.0f - smooth((std::abs(local.x - support.x) - s.moveGapZone) / std::max(0.01f, s.moveGapFade));
        const float need = (s.moveGap * near - (local.y - support.y) * out)
            * smooth(m_time / std::max(0.02f, s.moveGapEntry * m_strikeAt));
        if (need > 0.0f) position += toWorld(Vec3 { 0.0f, need * out, 0.0f }, live.facingYaw);
    }
}

Quaternion BallMove3D::orientationNow(const BallMoveLive3D& live) const {
    // A orientacao do pe na fase, pelo tempo dentro dela.
    const auto toward = [&](std::array<float, 3> angles, float end) {
        const float span = std::max(1e-3f, end - m_phaseStart);
        return slerp(m_phaseOrientation, localOrientation(angles[0], angles[1], angles[2]),
            smooth((m_time - m_phaseStart) / span));
    };
    switch (m_phase) {
    case BallMovePhase3D::Strike: return toward(m_strikeAngles, m_strikeAt);
    case BallMovePhase3D::Follow: return toward(m_strikeAngles, m_followAt);
    case BallMovePhase3D::Land:
        // (O pe fica plano antes de chegar ao chao.)
        return toward(m_landAngles, m_phaseStart + 0.6f * (m_landAt - m_phaseStart));
    case BallMovePhase3D::Rejoin:
        return slerp(m_phaseOrientation, live.clipOrientation,
            smoother((m_time - m_phaseStart) / std::max(1e-3f, m_rejoinSpan)));
    case BallMovePhase3D::Plant:
    case BallMovePhase3D::Idle: break;
    }
    return m_orientation;
}

bool BallMove3D::clipLeavingSoon(const BallMoveLive3D& live, const BallMoveSettings3D& s) const {
    // (Com o outro pe no chao, saindo antes do pouso do gesto; sendo o unico
    // apoio, so saindo ja.)
    const float land = m_choice.touch == BallMoveTouch3D::Shot ? s.shotLandSeconds : s.touchLandSeconds;
    const float soon = live.otherPlanted ? land * s.leavingShare : s.flightLeavingSeconds;
    return live.cycling && live.clipPlanted && live.clipSecondsToLiftoff >= 0.0f && live.clipSwingSeconds > 0.0f
        && live.clipSecondsToLiftoff < soon;
}

float BallMove3D::rejoinSpan(const BallMoveLive3D& live, const BallMoveSettings3D& s) const {
    const float base = m_choice.touch == BallMoveTouch3D::Shot ? s.shotRejoinSeconds : s.touchRejoinSeconds;
    // (Com o movimento mudando - parar, virar -, a perna sai no tempo dela.)
    if (m_hurried) return base;
    const float most = std::max(base, s.rejoinMaximumSeconds);
    if (clipLeavingSoon(live, s)) return std::clamp(live.clipSecondsToLiftoff + live.clipSwingSeconds, base, most);
    if (!live.cycling || live.clipPlanted || live.clipSecondsToTouchdown <= 0.0f) return base;
    return std::clamp(live.clipSecondsToTouchdown, base, most);
}

void BallMove3D::clampLandTarget(const BallMoveLive3D& live, const BallMoveSettings3D& s, float remaining) {
    // (O quadril anda de lado no passo e com o corpo do gesto: pelo do
    // comeco do pouso, a perna pousava esticada - 0,99.)
    const Vec3 hip = live.hip + planar(live.bodyVelocity) * remaining;
    const float height = std::max(0.0f, hip.z - m_landTarget.z);
    const float leg = s.landLegReach * m_legLength;
    const float most = std::min(s.landReach, std::max(0.06f, std::sqrt(std::max(0.0f, leg * leg - height * height))));
    const Vec3 fromHip = planar(m_landTarget - hip);
    const Vec3 travel = planar(live.bodyVelocity);
    if (m_plantAfterLand && travel.length() > 0.3f) {
        // O passo: no rumo do movimento so ate landReach a frente do quadril
        // (para tras nada: o corpo vem para cima dele); de lado, ao alcance.
        const Vec3 along = travel.normalized();
        const float ahead = std::min(dot(fromHip, along), s.landReach);
        Vec3 side = fromHip - along * dot(fromHip, along);
        if (side.length() > most) side = side * (most / side.length());
        m_landTarget = Vec3 { hip.x, hip.y, m_landTarget.z } + along * ahead + side;
        return;
    }
    if (fromHip.length() > most) m_landTarget = hip + fromHip * (most / fromHip.length()) + Vec3 { 0.0f, 0.0f, m_landTarget.z - hip.z };
}

void BallMove3D::beginReturn(const BallMoveLive3D& live, const BallMoveSettings3D& s, float ahead) {
    // Sem o pe do clipe (no fisico): o pouso parado de sempre.
    m_plantAfterLand = false;
    if (!live.clipKnown) {
        beginLand(live, s, ahead);
        return;
    }
    // Parado, ou com o pe do clipe no balanco - ou saindo do chao antes do
    // pouso (ele vem para a frente) -, ou o movimento mudando: a volta vai
    // direto a ele.
    const Vec3 travel = planar(live.bodyVelocity);
    const float speed = travel.length();
    if (m_hurried || !live.cycling || speed < 0.3f || !live.clipPlanted || clipLeavingSoon(live, s)) {
        beginRejoin(live, s, ahead);
        return;
    }
    // Com ele apoiado: so com ele a frente de onde o pe para naturalmente
    // (indo com o corpo, voltar a um pe parado atras era o "chicote" - ate
    // 10 m/s para tras); senao o passo: pousa a frente e sai do chao com ele.
    Vec3 position, velocity;
    worldState(live, s, orientationNow(live), position, velocity);
    position -= travel * ahead;
    const float base = m_choice.touch == BallMoveTouch3D::Shot ? s.shotRejoinSeconds : s.touchRejoinSeconds;
    const Vec3 stop = position + planar(velocity) * (0.5f * base);
    if (dot(planar(live.clipAnkle - stop), travel * (1.0f / speed)) >= -s.rejoinBehind) {
        beginRejoin(live, s, ahead);
        return;
    }
    m_plantAfterLand = true;
    beginLand(live, s, ahead);
}

void BallMove3D::beginLand(const BallMoveLive3D& live, const BallMoveSettings3D& s, float ahead) {
    // O pouso no mundo: parte de onde o pe esta (com a velocidade dele) e para
    // no chao, onde a passada continua.
    // (De onde o pe esta de fato - com o alcance aplicado -, nao do estado
    // por dentro: dele, o alvo saltava ate 9 cm na troca.)
    // (O corpo de live e o do fim do tick: a troca no meio dele volta o que
    // falta - ahead s.)
    const Vec3 back = planar(live.bodyVelocity) * ahead;
    if (m_phase == BallMovePhase3D::Strike || m_phase == BallMovePhase3D::Follow) {
        worldState(live, s, orientationNow(live), m_worldAnkle, m_worldVelocity);
        m_worldAnkle -= back;
    }
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
    m_landTarget = live.pelvisGround - back + planar(live.bodyVelocity) * landSeconds + toWorld(target, live.facingYaw);
    if (m_plantAfterLand) {
        // O passo: no rumo do movimento, onde o pe para naturalmente com o
        // embalo dele (metade do caminho na velocidade de agora) - puxado
        // para baixo do quadril, ele voltava para tras de uma vez (ate 7,8
        // m/s correndo: com o quadril alto, o alcance no chao e quase nenhum).
        // De lado, o do desenho.
        const Vec3 travel = planar(live.bodyVelocity);
        const float speed = travel.length();
        if (speed > 0.3f) {
            const Vec3 along = travel * (1.0f / speed);
            const Vec3 stop = m_worldAnkle + planar(m_worldVelocity) * (0.5f * landSeconds);
            m_landTarget += along * dot(planar(stop - m_landTarget), along);
        }
    }
    clampLandTarget(live, s, landSeconds);
    m_orientation = orientationNow(live);
    m_phase = BallMovePhase3D::Land;
    m_phaseStart = m_time;
    m_phaseOrientation = m_orientation;
}

void BallMove3D::beginRejoin(const BallMoveLive3D& live, const BallMoveSettings3D& s, float ahead) {
    // A diferenca do pe do gesto para o do clipe (de onde o pe esta de fato,
    // com a velocidade e a aceleracao dele) some numa quintica: o pe anda com
    // o do clipe e chega nele andando como ele - os dois juntos (perseguindo o
    // pe do clipe, o fim dava o tranco com ele rapido no balanco).
    // (O corpo e o pe do clipe de live sao os do fim do tick: a troca no meio
    // dele volta o que falta - ahead s; sem isso o pe andava para tras no
    // primeiro tick.)
    if (m_phase == BallMovePhase3D::Strike || m_phase == BallMovePhase3D::Follow) {
        worldState(live, s, orientationNow(live), m_worldAnkle, m_worldVelocity);
        m_worldAnkle -= planar(live.bodyVelocity) * ahead;
    }
    m_orientation = orientationNow(live);
    m_offset = m_worldAnkle - (live.clipAnkle - live.clipVelocity * ahead);
    m_offsetVelocity = m_worldVelocity - live.clipVelocity;
    m_offsetAcceleration = m_worldAcceleration - live.clipAcceleration;
    // (A aceleracao da diferenca no maximo rejoinAccelerationMaximum: com
    // mais, a quintica da volta disparava - o pe subia a 1,5 m.)
    // (O limite do alcance parte do da fase anterior: o da perna inteira no
    // pouso e apoiado.)
    m_rejoinLimitStart = m_phase == BallMovePhase3D::Land || m_phase == BallMovePhase3D::Plant ? 1.0f : s.rejoinReach;
    if (m_offsetAcceleration.length() > s.rejoinAccelerationMaximum)
        m_offsetAcceleration = m_offsetAcceleration * (s.rejoinAccelerationMaximum / m_offsetAcceleration.length());
    // (O pe do gesto abaixo do do clipe subindo - saindo do chao para ele no
    // balanco: sem isto a diferenca crescia antes de sumir e o alvo afundava
    // no chao, a perna travava reta. Ele sobe junto, ja.)
    if (m_offset.z < 0.0f) {
        m_offsetVelocity.z = std::max(m_offsetVelocity.z, 0.0f);
        m_offsetAcceleration.z = std::max(m_offsetAcceleration.z, 0.0f);
    }
    m_rejoinSpan = rejoinSpan(live, s);
    // Apoiado indo a um pe do clipe tambem apoiado e longe (parando, ou no
    // tempo maximo do apoio): um passo - o pe sobe num arco em vez de
    // deslizar.
    m_rejoinLift = 0.0f;
    if (m_phase == BallMovePhase3D::Plant && live.clipPlanted && planar(m_offset).length() > 0.05f)
        m_rejoinLift = s.liftHeight * std::min(1.0f, planar(m_offset).length() / 0.2f);
    m_phase = BallMovePhase3D::Rejoin;
    m_phaseStart = m_time;
    m_phaseOrientation = m_orientation;
}

void BallMove3D::hurry(float deltaTime, const BallMoveLive3D& live, const BallMoveSettings3D& s) {
    if (m_phase != BallMovePhase3D::Strike && m_phase != BallMovePhase3D::Follow) return;
    // (O pouso no tempo dele a partir de agora; o tick inteiro ainda vem.)
    const bool shot = m_choice.touch == BallMoveTouch3D::Shot;
    m_landAt = m_time + (shot ? s.shotLandSeconds : s.touchLandSeconds);
    m_hurried = true;
    beginReturn(live, s, std::max(deltaTime, 0.0f));
}

void BallMove3D::update(float deltaTime, const BallMoveLive3D& live, const BallMoveSettings3D& s) {
    m_landed = false;
    m_joined = false;
    if (m_phase == BallMovePhase3D::Idle) return;
    m_facing = live.facingYaw;
    // O tick inteiro passa pelas fases: o que sobra de uma entra na proxima
    // (parando na ponta da fase, o pe andava menos naquele tick - um tranco).
    float left = std::max(deltaTime, 1e-4f);
    for (int guard = 0; guard < 8 && left > 1e-6f; ++guard) {
        const float clock = m_time;
        if (m_phase == BallMovePhase3D::Strike) {
            if (m_over && clock < m_overAt) {
                // A sola sobe por cima da bola antes de puxar. (No alto, a
                // velocidade media dos dois trechos, deitada: com menos, o pe
                // parava ali e disparava depois.)
                Vec3 toward = ((m_overPoint - m_startLocal) * (1.0f / std::max(0.02f, m_overAt))
                                  + (m_strike - m_overPoint) * (1.0f / std::max(0.02f, m_strikeAt - m_overAt)))
                    * 0.5f;
                toward.z = 0.0f;
                const float step = std::min(left, m_overAt - clock);
                quinticStep(m_local, m_localVelocity, m_localAcceleration, m_overPoint, toward, m_overAt - clock, step);
                m_time += step;
                left -= step;
                continue;
            }
            const float step = std::min(left, std::max(0.0f, m_strikeAt - clock));
            quinticStep(m_local, m_localVelocity, m_localAcceleration, m_strike, m_strikeVelocity, m_strikeAt - clock,
                step);
            m_time += step;
            left -= step;
            if (m_time >= m_strikeAt - 1e-6f) {
                m_orientation = orientationNow(live);
                m_phase = BallMovePhase3D::Follow;
                m_phaseStart = m_time;
                m_phaseOrientation = m_orientation;
            }
            continue;
        }
        if (m_phase == BallMovePhase3D::Follow) {
            const float T = std::max(0.0f, m_followAt - clock);
            const float step = std::min(left, T);
            if (m_choice.touch == BallMoveTouch3D::Shot) {
                // O chute: a finalizacao e um arco da perna quase estendida em
                // volta do quadril (em linha reta, a linha passava perto do
                // quadril e o joelho dobrava - era um joelho levantando).
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
                const float along = std::clamp((clock + step - m_strikeAt) / span, 0.0f, 1.0f);
                const float u = smooth(along);
                const float radius = m_arcRadius + (radiusEnd - m_arcRadius) * u;
                const float radiusRate = (radiusEnd - m_arcRadius) * 6.0f * along * (1.0f - along) / span;
                // (O angulo num Hermite de 1 dimensao ate o fim, parado.)
                if (T <= step + 1e-6f) {
                    m_arcAngle = m_arcEnd;
                    m_arcRate = 0.0f;
                } else {
                    const float s1 = step / T, s2 = s1 * s1, s3 = s2 * s1;
                    const float next = m_arcAngle * (2.0f * s3 - 3.0f * s2 + 1.0f)
                        + m_arcRate * (T * (s3 - 2.0f * s2 + s1)) + m_arcEnd * (-2.0f * s3 + 3.0f * s2);
                    m_arcRate = m_arcAngle * ((6.0f * s2 - 6.0f * s1) / T) + m_arcRate * (3.0f * s2 - 4.0f * s1 + 1.0f)
                        + m_arcEnd * ((-6.0f * s2 + 6.0f * s1) / T);
                    m_arcAngle = next;
                }
                const Vec3 previousVelocity = m_localVelocity;
                m_local = hip + Vec3 { radius * std::sin(m_arcAngle), m_arcSide, -radius * std::cos(m_arcAngle) };
                // (A velocidade pela derivada do arco; a aceleracao pela
                // diferenca so em passos de verdade - na ponta da fase o passo
                // e uma fracao de milissegundo e ela dava 2500 m/s2.)
                const float sine = std::sin(m_arcAngle), cosine = std::cos(m_arcAngle);
                m_localVelocity = Vec3 { radiusRate * sine + radius * cosine * m_arcRate, 0.0f,
                    -radiusRate * cosine + radius * sine * m_arcRate };
                if (step > 2e-3f) m_localAcceleration = (m_localVelocity - previousVelocity) * (1.0f / step);
            } else {
                // (O acompanhamento acaba ja indo para o pouso: parando nele e
                // voltando, o pe dava o tranco na troca.)
                const float landSeconds = std::max(0.03f, m_landAt - m_followAt);
                const Vec3 toward { (m_landLocal.x - m_follow.x) / landSeconds * s.followIntoLand,
                    (m_landLocal.y - m_follow.y) / landSeconds * s.followIntoLand, 0.0f };
                quinticStep(m_local, m_localVelocity, m_localAcceleration, m_follow, toward, T, step);
            }
            m_time += step;
            left -= step;
            if (m_time >= m_followAt - 1e-6f) {
                m_worldAcceleration = toWorld(m_localAcceleration, live.facingYaw);
                beginReturn(live, s, left);
            }
            continue;
        }
        if (m_phase == BallMovePhase3D::Land) {
            clampLandTarget(live, s, std::max(0.0f, m_landAt - clock));
            const float step = std::min(left, std::max(0.0f, m_landAt - clock));
            quinticStep(m_worldAnkle, m_worldVelocity, m_worldAcceleration, m_landTarget, {}, m_landAt - clock, step);
            m_time += step;
            left -= step;
            if (m_time >= m_landAt - 1e-6f) {
                m_orientation = orientationNow(live);
                m_worldAnkle = m_landTarget;
                m_worldVelocity = {};
                m_worldAcceleration = {};
                if (m_plantAfterLand) {
                    m_phase = BallMovePhase3D::Plant;
                    m_phaseStart = m_time;
                    m_plantStart = m_time;
                    m_phaseOrientation = m_orientation;
                    m_plantAnkle = m_worldAnkle;
                    m_plantOrientation = m_orientation;
                    m_plantRoll = 0.0f;
                    continue;
                }
                m_landed = true;
                m_phase = BallMovePhase3D::Idle;
                break;
            }
            continue;
        }
        if (m_phase == BallMovePhase3D::Plant) {
            // Apoiado, parado no mundo (o passo), ate o pe do clipe sair do
            // chao (e pelo menos plantMinimumSeconds): a volta vai com ele no
            // balanco, sempre para a frente. Ficando atras do quadril, o
            // calcanhar sobe em volta da base dos dedos, como na saida do
            // passo. (Sai tambem parando, sem alcancar nem na ponta ou no tempo
            // maximo.)
            const float dt = left;
            const Vec3 before = m_worldAnkle;
            const float limit = s.plantReach * m_legLength;
            const Vec3 travel = planar(live.bodyVelocity);
            const Vec3 forward = planar((m_plantOrientation * m_flat.conjugate()).rotate({ 1.0f, 0.0f, 0.0f }));
            const Vec3 axis = forward.lengthSquared() > 1e-6f ? Vec3 { -forward.y, forward.x, 0.0f }.normalized()
                                                             : Vec3 { 0.0f, 1.0f, 0.0f };
            const Vec3 toe = m_plantAnkle - m_plantOrientation.rotate(m_ankleInFoot) + m_plantOrientation.rotate(m_toe);
            const auto rolled = [&](float angle) {
                return toe + Quaternion::fromAxisAngle(axis, angle).rotate(m_plantAnkle - toe);
            };
            float wanted = 0.0f;
            bool reachable = true;
            // (So com o pe embaixo ou atras do quadril: a frente, rolar afasta o
            // tornozelo.)
            const bool under = travel.length() < 0.3f || dot(planar(m_plantAnkle - live.hip), travel.normalized()) < 0.05f;
            if (under && (m_plantAnkle - live.hip).length() > limit) {
                if ((rolled(s.plantRollMaximum) - live.hip).length() > limit) {
                    reachable = false;
                    wanted = s.plantRollMaximum;
                } else {
                    float low = 0.0f, high = s.plantRollMaximum;
                    for (int i = 0; i < 10; ++i) {
                        const float middle = 0.5f * (low + high);
                        ((rolled(middle) - live.hip).length() > limit ? low : high) = middle;
                    }
                    wanted = high;
                }
            }
            const float most = s.plantRollRate * dt;
            m_plantRoll += std::clamp(wanted - m_plantRoll, -most, most);
            m_worldAnkle = rolled(m_plantRoll);
            m_orientation = (Quaternion::fromAxisAngle(axis, m_plantRoll) * m_plantOrientation).normalized();
            m_worldVelocity = dt > 1e-5f ? (m_worldAnkle - before) * (1.0f / dt) : Vec3 {};
            m_worldAcceleration = {};
            const bool stopped = !live.cycling || travel.length() < 0.3f;
            const bool lost = !reachable && m_plantRoll >= s.plantRollMaximum - 1e-3f;
            const float planted = m_time - m_plantStart;
            if ((!live.clipPlanted && planted >= s.plantMinimumSeconds) || stopped || lost
                || planted > s.plantMaximumSeconds) {
                beginRejoin(live, s, left);
                continue;
            }
            m_time += left;
            left = 0.0f;
            continue;
        }
        if (m_phase == BallMovePhase3D::Rejoin) {
            // O pe do clipe mais a diferenca sumindo: o pe anda com ele.
            m_time += left;
            left = 0.0f;
            const float u = std::clamp((m_time - m_phaseStart) / std::max(1e-3f, m_rejoinSpan), 0.0f, 1.0f);
            Vec3 offset, offsetRate;
            quinticDecay(m_offset, m_offsetVelocity, m_offsetAcceleration, m_rejoinSpan, u, offset, offsetRate);
            if (m_rejoinLift > 0.0f) {
                offset.z += m_rejoinLift * 16.0f * u * u * (1.0f - u) * (1.0f - u);
                offsetRate.z += m_rejoinLift * 32.0f * u * (1.0f - u) * (1.0f - 2.0f * u) / std::max(1e-3f, m_rejoinSpan);
            }
            m_worldAnkle = live.clipAnkle + offset;
            m_worldVelocity = live.clipVelocity + offsetRate;
            if (u >= 1.0f) {
                m_worldAnkle = live.clipAnkle;
                m_worldVelocity = live.clipVelocity;
                m_orientation = live.clipOrientation;
                m_joined = true;
                m_phase = BallMovePhase3D::Idle;
                break;
            }
            continue;
        }
        break;
    }
    if (m_phase != BallMovePhase3D::Idle) m_orientation = orientationNow(live);
    // O tornozelo e o pe no mundo.
    const bool inWorld = m_phase == BallMovePhase3D::Land || m_phase == BallMovePhase3D::Plant
        || m_phase == BallMovePhase3D::Rejoin || m_landed || m_joined;
    Vec3 ankleWorld;
    Vec3 ankleVelocity;
    if (inWorld) {
        ankleWorld = m_worldAnkle;
        ankleVelocity = m_worldVelocity;
        m_local = toLocal(m_worldAnkle - live.pelvisGround, live.facingYaw);
        // (No pouso, a sola longe do chao ate perto do fim: o pe ja plano.)
        if (m_phase == BallMovePhase3D::Land) {
            const float u = (m_time - m_phaseStart) / std::max(1e-3f, m_landAt - m_phaseStart);
            ankleWorld.z += soleLift(ankleWorld, m_orientation, live.pelvisGround.z, s) * (1.0f - smooth((u - 0.55f) / 0.35f));
        }
        // Ao alcance do quadril: no pouso e apoiado, no maximo a perna
        // inteira (alem disso o IK travava de qualquer jeito, e na troca para
        // a volta o pe saltava); na volta, nunca mais esticada que a do clipe
        // ou que rejoinReach, saindo do limite da fase anterior (a diferenca
        // sumindo podia passar do alcance e a perna travava reta; no fim os
        // dois coincidem).
        float most = m_legLength;
        if (m_phase == BallMovePhase3D::Rejoin || m_joined) {
            const float base = m_rejoinLimitStart + (s.rejoinReach - m_rejoinLimitStart) * smooth(rejoinProgress());
            most = std::max((live.clipAnkle - live.hip).length() + 0.01f * m_legLength, base * m_legLength);
        }
        const Vec3 fromHip = ankleWorld - live.hip;
        const float distance = fromHip.length();
        if (distance > most && distance > 1e-6f) ankleWorld = live.hip + fromHip * (most / distance);
    } else {
        worldState(live, s, m_orientation, ankleWorld, ankleVelocity);
        m_worldAcceleration = toWorld(m_localAcceleration, live.facingYaw);
    }
    m_foot = ankleWorld - m_orientation.rotate(m_ankleInFoot);
    m_velocity = ankleVelocity;
}

} // namespace MatterEngine
