#include "Engine/Athlete/Actions/BallGesture3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

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

Quaternion yawOnly(Vec3 heading) {
    return Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, std::atan2(heading.y, heading.x));
}

// Um passo do Hermite cubico de (p, v) ate (target, targetVelocity) em T
// segundos, avaliado em dt.
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

// O mesmo em uma dimensao, ate o alvo parado.
void hermiteStep1(float& p, float& v, float target, float T, float dt) {
    if (T <= dt + 1e-5f) {
        p = target;
        v = 0.0f;
        return;
    }
    const float s = dt / T;
    const float s2 = s * s, s3 = s2 * s;
    const float nextP = p * (2.0f * s3 - 3.0f * s2 + 1.0f) + v * (T * (s3 - 2.0f * s2 + s)) + target * (-2.0f * s3 + 3.0f * s2);
    const float nextV = p * ((6.0f * s2 - 6.0f * s) / T) + v * (3.0f * s2 - 4.0f * s + 1.0f)
        + target * ((-6.0f * s2 + 6.0f * s) / T);
    p = nextP;
    v = nextV;
}

// A altura da origem do link acima do chao com o pe plano nesta orientacao.
float flatHeight(const BallFootBox3D& box, int side, Quaternion orientation) {
    const std::size_t s = static_cast<std::size_t>(side);
    const Quaternion world = (orientation * box.orientation[s]).normalized();
    const Vec3 center = orientation.rotate(box.center[s]);
    const Vec3 axes[3] = { world.rotate({ 1.0f, 0.0f, 0.0f }), world.rotate({ 0.0f, 1.0f, 0.0f }),
        world.rotate({ 0.0f, 0.0f, 1.0f }) };
    const Vec3 half = box.half[s];
    const float low = center.z - std::abs(axes[0].z) * half.x - std::abs(axes[1].z) * half.y
        - std::abs(axes[2].z) * half.z;
    return -low;
}

} // namespace

void BallGesture3D::start(const BallGestureStart3D& in, const BallFootBox3D& box, const BallGestureSettings3D& s) {
    const BallPlan3D& plan = in.plan;
    const float power = in.power, lift = in.lift, groundHeight = in.groundHeight, legLength = in.legLength;
    const Vec3 footPosition = in.foot, hip = in.hip, ankleInFoot = in.ankleInFoot;
    const Quaternion footOrientation = in.footOrientation;
    m_frameKind = in.frame;
    m_moving = in.frame != BallGestureFrame3D::World;
    const bool swing = in.frame == BallGestureFrame3D::StyleFoot;
    m_hip = hip;
    m_bodyVelocity = planar(in.bodyVelocity);
    m_legLength = legLength * s.legReach;
    m_landReach = legLength * s.landReach;
    m_ankleInFoot = ankleInFoot;
    m_restHip = hip;
    const bool shot = in.kind == BallActionKind3D::Shot;
    m_shot = shot;
    const float p = std::clamp(power, 0.0f, 1.0f);
    m_side = plan.side;
    m_push = planar(plan.push).lengthSquared() > 1e-6f ? planar(plan.push).normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    m_time = 0.0f;
    m_contactAt = plan.contactTime;
    // (No balanco nao ha armado: o pe vem da passada direto ao contato e
    // volta a ela ate o pouso dela.)
    m_windupAt = swing ? 0.0f : std::max(0.02f, plan.contactTime - plan.strikeSeconds);
    m_followAt = m_contactAt + (shot ? s.shotFollowSeconds : s.touchFollowSeconds) * (0.8f + 0.4f * p);
    if (swing) m_followAt = std::max(m_contactAt + s.swingMinimumFollow, in.styleTouchdown + s.swingSettleSeconds);
    m_returnAt = m_followAt + s.returnSeconds + s.returnSecondsPower * p;
    m_position = footPosition;
    m_orientation = footOrientation;
    m_contact = plan.footPosition;
    m_contactOrientation = plan.footOrientation;
    // O referencial do gesto: andando, ele vai com o corpo (o armado, a
    // batida e o acompanhamento do toque sao nele - parado no mundo, o pe
    // ficava para tras do corpo correndo e a batida tinha de cobrir quase 1 m);
    // parado, fica parado.
    m_frame = {};
    m_styleStart = in.styleFoot;
    m_styleOrientation = footOrientation;
    m_frameVelocity = swing ? in.styleFootVelocity : m_moving ? m_bodyVelocity : Vec3 {};
    // (O pe parte da velocidade dele: no balanco da corrida ele anda mais
    // rapido que o corpo.)
    m_velocity = m_moving ? in.footVelocity : Vec3 {};
    const Vec3 contact = m_contact - m_frameVelocity * m_contactAt;
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    // A velocidade do pe no contato (no referencial): no rumo pedido.
    const float strike = shot ? s.shotStrikeSpeed + s.shotStrikeSpeedPower * p
        : s.touchStrikeSpeed + s.touchStrikeSpeedPower * p;
    m_contactVelocity = m_push * (strike * (swing ? s.swingStrikeShare : 1.0f));
    // O armado: para tras do contato e para cima, nunca abaixo do chao. Com
    // o pe ja mais para tras que isso (no rumo do toque), ele so sobe - nao
    // vem para a frente antes da batida.
    // (Com a sola o pe vem de cima da bola: atras dela nao alcanca.)
    const bool sole = plan.surface == BallTouchSurface3D::Sole;
    const float back = (shot ? s.shotBack + s.shotBackPower * p : s.touchBack + s.touchBackPower * p)
        * (sole ? s.soleBackShare : 1.0f);
    const float rise = std::max(shot ? s.shotUp + s.shotUpPower * p : s.touchUp + s.touchUpPower * p,
        sole ? s.soleUp : 0.0f);
    const float along = std::min(dot(planar(footPosition - contact), m_push), -back);
    Vec3 windup = contact + m_push * along + up * rise;
    windup.z = std::max(windup.z, std::max(contact.z, footPosition.z) + 0.5f * rise);
    m_windupOffset = windup - contact;
    // (No chute de peito do pe, a ponta mais para baixo no armado.)
    const float extraPitch = plan.surface == BallTouchSurface3D::Laces ? 0.35f : 0.1f;
    const Vec3 lateral = cross(up, m_push);
    m_windupOrientation = (Quaternion::fromAxisAngle(lateral, -extraPitch) * m_contactOrientation).normalized();
    // O armado ao alcance da perna: o tornozelo ate windupReach dela do
    // quadril (mais longe, o IK esticava a perna de uma vez no meio do
    // armado). A pelve anda no armado (o equilibrio): refeito a cada tick.
    m_windupReach = legLength * s.windupReach;
    const Vec3 reachableAtStart = reachableWindup(contact);
    // A velocidade no contato cabe no percurso da batida: um cubico que parte
    // parado do armado so chega sem recuar ate 3 D / T (com folga, 2,5).
    {
        const float distance = (contact - reachableAtStart).length();
        const float span = std::max(0.03f, m_contactAt - m_windupAt);
        const float most = 2.5f * distance / span;
        const float speed = m_contactVelocity.length();
        if (speed > most && speed > 1e-4f) m_contactVelocity = m_contactVelocity * (most / speed);
    }
    m_followBlend = s.followBlendSeconds;
    // O acompanhamento: o arco da perna em volta do quadril (comeca no
    // contato); o ponto do fim e so a estimativa do pouso.
    m_followArc = shot ? s.shotFollowArc + s.shotFollowArcPower * p * (1.0f + 0.5f * std::max(0.0f, lift))
        : s.touchFollowArc + s.touchFollowArcPower * p;
    m_followCeiling = shot ? s.shotFollowCeiling : 0.0f;
    // (No toque o pe segue baixo e para: um cubico que parte a v so para sem
    // passar e voltar com o caminho de pelo menos v T / 3 - com folga, 0,4.)
    const float followDistance = shot ? s.shotFollow + s.shotFollowPower * p
        : std::max(s.touchFollow + s.touchFollowPower * p,
            0.4f * dot(m_contactVelocity, m_push) * (m_followAt - m_contactAt));
    m_followOffset = m_push * followDistance
        + up * (shot ? s.shotFollowUp * p * (1.0f + std::max(0.0f, lift)) : 0.02f);
    const Vec3 contactToe = planar(m_contactOrientation.rotate({ 1.0f, 0.0f, 0.0f }));
    m_landOrientation = yawOnly(contactToe.lengthSquared() > 1e-6f ? contactToe : m_push);
    const float flat = groundHeight + flatHeight(box, m_side, m_landOrientation);
    // (O toque correndo acaba com o pe pousado e plano: ele vira o proximo
    // apoio e as pegadas o prendem ali, no ritmo da passada.)
    if (m_moving && !shot) m_followOffset.z = flat - contact.z;
    m_follow = m_contact + m_followOffset;
    m_land = (m_contact + m_follow) * 0.5f;
    m_land.z = flat;
    enter(swing ? BallGesturePhase3D::Strike : BallGesturePhase3D::Windup);
}

void BallGesture3D::retarget(Vec3 footPosition, Quaternion footOrientation, float deltaTime,
    const BallGestureSettings3D& s) {
    if (m_phase != BallGesturePhase3D::Windup && m_phase != BallGesturePhase3D::Strike) return;
    Vec3 change = footPosition - m_contact;
    const float most = s.correctionSpeed * std::max(deltaTime, 1e-4f);
    if (change.length() > most) change = change * (most / change.length());
    m_contact += change;
    m_follow += change;
    m_land += Vec3 { change.x, change.y, 0.0f };
    m_contactOrientation = slerp(m_contactOrientation, footOrientation, 0.2f);
}

void BallGesture3D::enter(BallGesturePhase3D phase) {
    m_phase = phase;
    m_phaseOrientation = m_orientation;
    m_phaseStart = m_time;
    if (phase == BallGesturePhase3D::Follow && m_shot) beginFollowArc();
    if (phase == BallGesturePhase3D::Return && m_shot) {
        // (A volta do chute continua o arco: o pendulo da perna esticada
        // descendo em volta do quadril ate o pouso; em linha reta a perna
        // dobrava e o joelho subia.)
        m_arcRate = 0.0f;
        m_arcRadiusRate = 0.0f;
        m_arcLateralRate = 0.0f;
    }
}

void BallGesture3D::beginFollowArc() {
    // O tornozelo em relacao ao quadril, no plano vertical do toque: o lado
    // fica, o raio e o angulo andam (a perna gira para cima e freia).
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const Vec3 side = cross(up, m_push);
    const Vec3 relative = m_position + m_orientation.rotate(m_ankleInFoot) - m_hip;
    m_arcLateral = dot(relative, side);
    const float along = dot(relative, m_push);
    const float height = relative.z;
    // (O raio comeca no de agora - sem salto. No chute a perna estende ate
    // quase esticada, como no chute de verdade; com o joelho dobrado o arco
    // parecia um joelho subindo. No toque ela fica como esta.)
    m_arcRadiusStart = std::max(0.2f, std::sqrt(along * along + height * height));
    m_arcRadius = m_arcRadiusStart;
    m_arcRadiusEnd = m_shot ? m_legLength : std::min(m_arcRadiusStart, m_legLength);
    m_arcAngle = std::atan2(height, along);
    // A velocidade do angulo: a do pe na tangente do arco (no plano).
    const Vec3 tangent = m_push * -std::sin(m_arcAngle) + up * std::cos(m_arcAngle);
    m_arcRate = std::max(0.0f, dot(m_velocity - m_bodyVelocity, tangent)) / m_arcRadius;
    // Quanto sobe: o pedido, e o bastante para frear sem passar e voltar
    // (o cubico que para precisa de w T / 3; com folga, 0,45 w T); no chute,
    // ate um pouco acima da horizontal.
    const float span = std::max(0.03f, m_followAt - m_contactAt);
    float rise = std::max(m_followArc, 0.45f * m_arcRate * span);
    m_arcEnd = m_arcAngle + rise;
    if (m_followCeiling > 0.0f) m_arcEnd = std::min(m_arcEnd, std::max(m_arcAngle + 0.2f, m_followCeiling));
    else m_arcEnd = std::min(m_arcEnd, std::max(m_arcAngle + 0.1f, -0.35f));
    // (Com o teto, o tempo encolhe para nao passar.)
    if (m_arcRate > 1e-3f) {
        const float most = 2.5f * (m_arcEnd - m_arcAngle) / m_arcRate;
        if (most < span) m_followAt = m_contactAt + std::max(0.04f, most);
    }
    // O que o arco nao leva da velocidade do pe (a parte radial - a perna
    // estendendo no contato - e a diferenca do corpo) sai aos poucos.
    m_followSlip = m_velocity - (tangent * (m_arcRate * m_arcRadius) + m_bodyVelocity);
}

Vec3 BallGesture3D::arcAnkle() const {
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    return m_hip + cross(up, m_push) * m_arcLateral + m_push * (m_arcRadius * std::cos(m_arcAngle))
        + up * (m_arcRadius * std::sin(m_arcAngle));
}

Vec3 BallGesture3D::reachableWindup(Vec3 contact) const {
    // (No referencial do gesto: o quadril vai com ele.)
    Vec3 windup = contact + m_windupOffset;
    const Vec3 reach = windup + m_windupOrientation.rotate(m_ankleInFoot) - (m_hip - m_frame);
    if (reach.length() > m_windupReach) windup = windup - reach * (1.0f - m_windupReach / reach.length());
    return windup;
}

void BallGesture3D::reachableLand() {
    // O tornozelo do pouso a no maximo m_landReach do quadril de repouso.
    m_land = m_landWanted;
    const Vec3 ankle = m_land + m_landOrientation.rotate(m_ankleInFoot);
    const float height = m_restHip.z - ankle.z;
    const float most = std::sqrt(std::max(0.0f, m_landReach * m_landReach - height * height));
    const Vec3 offset { ankle.x - m_restHip.x, ankle.y - m_restHip.y, 0.0f };
    if (offset.length() > most && offset.length() > 1e-5f) m_land = m_land - offset * (1.0f - most / offset.length());
}

void BallGesture3D::update(float deltaTime, const BallGestureLive3D& live, const BallGestureSettings3D& s) {
    (void)s;
    if (m_phase == BallGesturePhase3D::Idle) return;
    m_hip = live.hip;
    m_restHip = live.restHip;
    m_styleOrientation = live.styleFootOrientation;
    const float dt = std::max(deltaTime, 1e-4f);
    // O pe no referencial do gesto: andando, ele vai com o corpo (o chute) ou
    // com o pe do estilo (o toque no balanco); o pe guarda a velocidade dele
    // no mundo quando o referencial muda a dele.
    switch (m_frameKind) {
    case BallGestureFrame3D::World:
        m_frameVelocity = {};
        break;
    case BallGestureFrame3D::Body:
        m_frameVelocity = planar(live.bodyVelocity);
        break;
    case BallGestureFrame3D::StyleFoot:
        m_frame = live.styleFoot - m_styleStart;
        m_frameVelocity = live.styleFootVelocity;
        break;
    }
    Vec3 p = m_position - m_frame;
    Vec3 v = m_velocity - m_frameVelocity;
    // O contato no referencial (onde ele estara quando o pe chegar).
    const Vec3 contact = m_contact - (m_frame + m_frameVelocity * (m_contactAt - m_time));
    bool relative = false;
    m_time += dt;
    const auto orient = [&](Quaternion target, float end) {
        const float span = std::max(1e-3f, end - m_phaseStart);
        m_orientation = slerp(m_phaseOrientation, target, smooth((m_time - m_phaseStart) / span));
    };
    switch (m_phase) {
    case BallGesturePhase3D::Windup:
        hermiteStep(p, v, reachableWindup(contact), {}, m_windupAt - (m_time - dt), dt);
        relative = true;
        orient(m_windupOrientation, m_windupAt);
        break;
    case BallGesturePhase3D::Strike:
        hermiteStep(p, v, contact, m_contactVelocity, m_contactAt - (m_time - dt), dt);
        relative = true;
        orient(m_contactOrientation, m_contactAt);
        break;
    case BallGesturePhase3D::Follow: {
        const Vec3 previous = m_position;
        orient(m_frameKind == BallGestureFrame3D::StyleFoot ? m_styleOrientation
                : m_moving && !m_shot ? m_landOrientation : m_contactOrientation,
            m_followAt);
        if (m_shot) {
            // O arco: o angulo num Hermite de 1 dimensao ate o fim, parado.
            hermiteStep1(m_arcAngle, m_arcRate, m_arcEnd, m_followAt - (m_time - dt), dt);
            const float span = std::max(0.03f, m_followAt - m_contactAt);
            m_arcRadius = m_arcRadiusStart + (m_arcRadiusEnd - m_arcRadiusStart) * smooth((m_time - m_contactAt) / span);
            m_position = arcAnkle() - m_orientation.rotate(m_ankleInFoot);
            {
                // (Hermite: parte com a diferenca de velocidade e some parado.)
                const float tau = m_time - m_phaseStart;
                const float T = std::max(1e-3f, m_followBlend);
                if (tau < T) {
                    const float fall = 1.0f - tau / T;
                    m_position += m_followSlip * (tau * fall * fall);
                }
            }
            m_velocity = (m_position - previous) * (1.0f / dt);
        } else if (m_frameKind == BallGestureFrame3D::StyleFoot) {
            // O toque no balanco: o pe volta ao do estilo (parado nele) ate o
            // pouso dele.
            hermiteStep(p, v, m_styleStart, {}, m_followAt - (m_time - dt), dt);
            relative = true;
        } else {
            // O toque: o pe segue baixo no rumo e para (ao alcance; no
            // referencial do gesto).
            Vec3 follow = contact + m_followOffset;
            const Vec3 reach = follow + m_orientation.rotate(m_ankleInFoot) - (m_hip - m_frame);
            if (reach.length() > m_legLength) follow = follow - reach * (1.0f - m_legLength / reach.length());
            hermiteStep(p, v, follow, {}, m_followAt - (m_time - dt), dt);
            relative = true;
        }
        break;
    }
    case BallGesturePhase3D::Return: {
        // (O quadril de repouso anda com o corpo: o pouso vai junto.)
        reachableLand();
        const float T = m_returnAt - (m_time - dt);
        if (m_shot) {
            // O pendulo: angulo, raio e lado do tornozelo em volta do quadril
            // ate os do pouso, parados.
            const Vec3 previous = m_position;
            orient(m_landOrientation, m_returnAt);
            const Vec3 up { 0.0f, 0.0f, 1.0f };
            const Vec3 side = cross(up, m_push);
            const Vec3 land = m_land + m_landOrientation.rotate(m_ankleInFoot) - m_hip;
            const float along = dot(land, m_push);
            hermiteStep1(m_arcAngle, m_arcRate, std::atan2(land.z, along), T, dt);
            hermiteStep1(m_arcRadius, m_arcRadiusRate, std::sqrt(along * along + land.z * land.z), T, dt);
            hermiteStep1(m_arcLateral, m_arcLateralRate, dot(land, side), T, dt);
            m_position = arcAnkle() - m_orientation.rotate(m_ankleInFoot);
            if (T <= dt + 1e-5f) m_position = m_land;
            m_velocity = (m_position - previous) * (1.0f / dt);
        } else {
            hermiteStep(m_position, m_velocity, m_land, {}, T, dt);
            orient(m_landOrientation, m_returnAt);
        }
        if (m_time >= m_returnAt) m_phase = BallGesturePhase3D::Idle;
        break;
    }
    case BallGesturePhase3D::Idle:
        break;
    }
    m_frame += m_frameVelocity * dt;
    if (relative) {
        m_position = p + m_frame;
        m_velocity = v + m_frameVelocity;
    }
    if (m_phase == BallGesturePhase3D::Windup && m_time >= m_windupAt) {
        enter(BallGesturePhase3D::Strike);
    } else if (m_phase == BallGesturePhase3D::Strike && m_time >= m_contactAt) {
        enter(BallGesturePhase3D::Follow);
    } else if (m_phase == BallGesturePhase3D::Follow && m_time >= m_followAt) {
        m_follow = m_position;
        if (m_moving) {
            // Andando: o pe volta as pegadas no ar (elas o pousam no ritmo).
            m_phase = BallGesturePhase3D::Idle;
        } else {
            // O pouso: embaixo do fim do acompanhamento, ao alcance da perna
            // esticada no chao a partir do quadril de repouso (parado o corpo
            // nao vai junto; a 0,77 m a frente a perna nao chegava ao chao, o
            // pe ficava no ar e as pegadas nao o travavam).
            m_landWanted = Vec3 { m_position.x, m_position.y, m_land.z };
            reachableLand();
            if (m_shot) m_arcRadius = m_arcRadiusEnd;
            enter(BallGesturePhase3D::Return);
        }
    }
}

bool BallGesture3D::inContactWindow(const BallGestureSettings3D& s) const {
    if (m_phase == BallGesturePhase3D::Strike) return true;
    return m_phase == BallGesturePhase3D::Follow && m_time <= m_contactAt + s.contactLate;
}

float BallGesture3D::weight() const {
    switch (m_phase) {
    case BallGesturePhase3D::Idle: return 0.0f;
    case BallGesturePhase3D::Windup: return smooth(m_time / std::max(0.03f, 0.5f * m_windupAt));
    case BallGesturePhase3D::Strike:
    case BallGesturePhase3D::Follow: return 1.0f;
    case BallGesturePhase3D::Return:
        return 1.0f - smooth((m_time - m_followAt) / std::max(0.03f, m_returnAt - m_followAt));
    }
    return 0.0f;
}

} // namespace MatterEngine
