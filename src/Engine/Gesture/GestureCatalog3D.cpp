#include "Engine/Gesture/GestureCatalog3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Degree = Pi / 180.0f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 normalizedOr(Vec3 v, Vec3 fallback) {
    const float length = v.length();
    return length > 1e-6f ? v * (1.0f / length) : fallback;
}

// A rotacao que leva a base (a, b, a x b) na base (c, d, c x d), as duas
// ortonormais.
Quaternion rotationBetweenFrames(Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    const Vec3 e = cross(a, b), f = cross(c, d);
    // R = [c d f] [a b e]^T
    const float m00 = c.x * a.x + d.x * b.x + f.x * e.x, m01 = c.x * a.y + d.x * b.y + f.x * e.y,
                m02 = c.x * a.z + d.x * b.z + f.x * e.z;
    const float m10 = c.y * a.x + d.y * b.x + f.y * e.x, m11 = c.y * a.y + d.y * b.y + f.y * e.y,
                m12 = c.y * a.z + d.y * b.z + f.y * e.z;
    const float m20 = c.z * a.x + d.z * b.x + f.z * e.x, m21 = c.z * a.y + d.z * b.y + f.z * e.y,
                m22 = c.z * a.z + d.z * b.z + f.z * e.z;
    Quaternion q;
    const float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = { (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s };
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = { 0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s };
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = { (m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s };
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = { (m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s };
    }
    return q.normalized();
}

// O vetor de rotacao (o log), o das coordenadas de junta.
Vec3 rotationVector(Quaternion q) {
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const Vec3 v { q.x, q.y, q.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, q.w) / n);
}

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

// Um ponto no chao embaixo da pelve (x frente, "fora" para o lado da perna do
// gesto, z sobre o chao, no referencial do corpo) no referencial da pelve dela
// (descida e deslocamento do gesto ja contados).
Vec3 pelvisFrame(const GestureRig3D& rig, int side, float x, float out, float z, const GestureBodyPeaks3D& body,
    float turned = 0.0f) {
    const float y = side == 0 ? out : -out;
    // (turned: o quanto a cintura ja girou ali - o ponto fica no lugar dele.)
    return rig.bodyToPelvis.rotate(
        rotateZ({ x - body.forward, y - body.left, z - (rig.pelvisHeight - body.drop) }, -turned));
}

// A velocidade das juntas (a coxa e o joelho) por metro do tornozelo andando
// em direction: a diferenca central na perna de dois ossos.
GestureLegPose3D legTangent(const GestureLegRig3D& leg, int side, Vec3 ankle, Vec3 direction, float twist,
    float plantar, float inversion, Quaternion bodyToPelvis) {
    constexpr float Step = 0.01f;
    const GestureLegPose3D ahead
        = gestureLegPose3D(leg, side, ankle + direction * Step, twist, plantar, inversion, bodyToPelvis);
    const GestureLegPose3D behind
        = gestureLegPose3D(leg, side, ankle - direction * Step, twist, plantar, inversion, bodyToPelvis);
    GestureLegPose3D tangent;
    for (std::size_t i = 0; i < 4; ++i) tangent.value[i] = (ahead.value[i] - behind.value[i]) / (2.0f * Step);
    return tangent;
}

// As bolas de referencia (jogador de 1,80 m): o quanto ela esta para o lado
// da perna do gesto por alcance e a frente por frente-tras.
constexpr float TouchOut[3] = { 0.18f, 0.34f, 0.50f };
constexpr float TouchX[3] = { -0.16f, 0.12f, 0.36f };
constexpr float ShotOut[3] = { 0.17f, 0.34f, 0.52f };
constexpr float ShotX = 0.18f;
constexpr float BallZ = 0.11f;
// (O voleio: a bola de referencia mais a frente que a rasteira.)
constexpr float HighShotX = 0.30f;

// O tornozelo mais alto da bola alta: o pe na altura do joelho (24/10, pedido
// do usuario).
float kneeTop(const GestureLegRig3D& leg, float size) { return leg.ankleHeight + leg.shin + 0.05f * size; }

// A altura de referencia da bola alta: um gesto pronto so, o pe no joelho (a
// bola so escolhe o balde; 24/10, pedido do usuario: o pe nunca vai ate a bola).
float highBallZ(const GestureLegRig3D& leg, float size) { return std::max(0.40f, kneeTop(leg, size)) - 0.06f * size; }

// A recepcao (24/10, pedido do usuario: "quase o manejo, mas com movimento de
// parada"): a forca no comando para a bola contra o corpo e o gesto e pronto,
// como os outros - a bola so escolhe o balde (frente-tras, alcance) e a
// superficie: a sola na rasteira de frente no meio, a parte de fora na que vem
// de fora, a de dentro no resto, o peito do pe erguido na alta. O pe nunca
// segue a bola (ele via as pernas malucas); o acompanhamento cede no rumo da
// superficie, num movimento so. O corpo agacha um pouco, o tronco sobre a bola.
GestureKeys3D receptionKeys(const GestureSpec3D& spec, const GestureRig3D& rig) {
    GestureKeys3D keys;
    const int side = spec.side == 1 ? 1 : 0;
    const float s = side == 0 ? 1.0f : -1.0f;
    const float power = std::clamp(spec.power, 0.0f, 1.0f);
    const float seconds = std::max(0.12f, spec.seconds);
    const GestureLegRig3D& leg = rig.legs[static_cast<std::size_t>(side)];
    const float size = std::clamp((leg.thigh + leg.shin) / 0.832f, 0.75f, 1.3f);
    keys.contactTime = std::clamp(0.16f * seconds, 0.04f, 0.07f);
    keys.followTime = keys.contactTime + (spec.high ? 0.42f : 0.35f) * seconds;
    keys.duration = seconds;
    GestureBodyPeaks3D& body = keys.body;
    const int depth = std::clamp(spec.depth, 0, 2), reach = std::clamp(spec.reach, 0, 2);
    const float ballX = TouchX[depth] * size, ballOut = TouchOut[reach] * size;
    const float height = spec.high ? highBallZ(leg, size) : BallZ;
    // De onde ela vem (no "fora" da perna): so escolhe a superficie.
    const Vec3 incoming = normalizedOr(planar(spec.incoming), { -1.0f, 0.0f, 0.0f });
    const float withOut = s * incoming.y;
    // (lx, lout, lz: por onde o pe passa subindo antes, em relacao ao contato -
    // ele sobe e vai, nunca raspando o chao: reto, ia a 14 m/s rente a grama.
    // O tornozelo fica a um raio da bola mais meio pe da superficie. cx, cout:
    // o rumo em que a superficie cede.)
    struct Surface { float dx, dout, dz, twist, plantar, inversion, cushion, drop, lx, lout, lz, cx, cout; };
    Surface surface;
    if (spec.high)
        // O peito do pe por baixo da bola, a ponta para baixo; desce ate perto
        // do chao. (Com o tornozelo so 4 cm atras dela, a canela em pe ficava
        // dentro da bola.)
        surface = { -0.12f, 0.0f, -0.14f, 15.0f, 25.0f, 0.0f, 0.06f, -0.26f, -0.16f, 0.0f, -0.08f, -1.0f, 0.0f };
    else if (reach == 0 && depth == 2)
        // A sola por cima, a ponta para cima; cede para tras.
        surface = { -0.03f, 0.0f, 0.17f, 0.0f, -22.0f, 0.0f, 0.10f, -0.03f, -0.22f, 0.0f, 0.10f, -1.0f, 0.0f };
    else if (withOut < -0.4f)
        // Vindo de fora (para o meio): a parte de fora do pe, que da para ela;
        // cede para dentro. (O lado de dentro ficava de costas.)
        surface = { -0.05f, -0.15f, 0.05f, -20.0f, 15.0f, 20.0f, 0.10f, 0.01f, -0.06f, -0.08f, 0.06f, -0.3f, -0.95f };
    else
        // O lado de dentro, o pe virado para fora; recua.
        surface = { -0.09f, 0.12f, 0.05f, 40.0f, 5.0f, -5.0f, 0.12f, 0.01f, -0.08f, -0.10f, 0.07f, -1.0f, 0.0f };
    const float softness = 0.6f + 0.4f * power;
    body.drop = (spec.high ? 0.015f : 0.035f) * softness;
    body.forward = 0.02f * softness;
    body.roll = s * 2.0f * Degree;
    body.spineRoll = -s * (5.0f + (spec.high ? 6.0f : 0.0f)) * Degree;
    body.spinePitch = (spec.high ? 4.0f : 12.0f) * Degree;
    body.spineBack = (spec.high ? 8.0f : 0.0f) * Degree;
    body.headPitch = 18.0f * Degree;
    body.armForward = { -6.0f * softness * Degree, 10.0f * softness * Degree };
    body.armOut = { 14.0f * softness * Degree, 18.0f * softness * Degree };
    body.elbow = { 15.0f * Degree, 15.0f * Degree };
    const float contactX = ballX + surface.dx * size, contactOut = ballOut + surface.dout * size,
                contactZ = height + surface.dz * size;
    const Vec3 contactAnkle = pelvisFrame(rig, side, contactX, contactOut, contactZ, body);
    keys.contact = gestureLegPose3D(leg, side, contactAnkle, surface.twist * Degree, surface.plantar * Degree,
        surface.inversion * Degree, rig.bodyToPelvis);
    const Vec3 liftAnkle = pelvisFrame(rig, side, contactX + surface.lx * size,
        std::max(0.02f * size, contactOut + surface.lout * size), std::max(0.13f * size, contactZ + surface.lz * size), body);
    keys.lift = gestureLegPose3D(leg, side, liftAnkle, surface.twist * Degree, surface.plantar * Degree,
        surface.inversion * Degree, rig.bodyToPelvis);
    keys.liftValid = true;
    float followX = contactX + surface.cx * surface.cushion * size;
    float followOut = contactOut + surface.cout * surface.cushion * size;
    const float followZ = std::max(0.13f * size, contactZ + surface.drop * size);
    followOut = std::max(followOut, followX > 0.25f * size ? -0.05f * size : 0.02f * size);
    followX = std::clamp(followX, -0.40f * size, 0.65f * size);
    const Vec3 followAnkle = pelvisFrame(rig, side, followX, followOut, followZ, body);
    keys.follow = gestureLegPose3D(leg, side, followAnkle, surface.twist * Degree, surface.plantar * Degree,
        surface.inversion * Degree, rig.bodyToPelvis);
    keys.contactTangent = legTangent(leg, side, contactAnkle, normalizedOr(followAnkle - contactAnkle, { -1.0f, 0.0f, 0.0f }),
        surface.twist * Degree, surface.plantar * Degree, surface.inversion * Degree, rig.bodyToPelvis);
    keys.followDistance = (followAnkle - contactAnkle).length();
    // (O pe chega quase parado e cede com ela.)
    keys.contactSpeed = 0.4f;
    return keys;
}

// Levantar a bola (24/10, pedido do usuario: o botao do meio): a ponta do pe
// entra por baixo dela rente ao chao e sobe com ela, o joelho junto - gesto
// pronto pelo balde (a bola atras levanta do meio). O corpo inclina um pouco
// para tras e a cabeca olha a bola subir; mais pela barra.
GestureKeys3D raiseKeys(const GestureSpec3D& spec, const GestureRig3D& rig) {
    GestureKeys3D keys;
    const int side = spec.side == 1 ? 1 : 0;
    const float s = side == 0 ? 1.0f : -1.0f;
    const float power = std::clamp(spec.power, 0.0f, 1.0f);
    const float seconds = std::max(0.12f, spec.seconds);
    const GestureLegRig3D& leg = rig.legs[static_cast<std::size_t>(side)];
    const float size = std::clamp((leg.thigh + leg.shin) / 0.832f, 0.75f, 1.3f);
    const int depth = std::max(1, std::clamp(spec.depth, 0, 2)), reach = std::clamp(spec.reach, 0, 2);
    const float ballX = TouchX[depth] * size, ballOut = TouchOut[reach] * size;
    keys.contactTime = std::clamp(0.16f * seconds, 0.04f, 0.07f);
    keys.followTime = keys.contactTime + 0.32f * seconds;
    keys.duration = seconds;
    GestureBodyPeaks3D& body = keys.body;
    const float softness = 0.5f + 0.5f * power;
    body.drop = 0.015f * softness;
    body.spineBack = (4.0f + 6.0f * power) * Degree;
    body.spineRoll = -s * 6.0f * softness * Degree;
    body.roll = s * 2.0f * Degree;
    body.headPitch = 16.0f * Degree;
    body.armForward = { -6.0f * Degree, 10.0f * Degree };
    body.armOut = { (14.0f + 10.0f * power) * Degree, (18.0f + 10.0f * power) * Degree };
    body.elbow = { 12.0f * Degree, 16.0f * Degree };
    // O contato: o tornozelo atras da bola rente ao chao, a ponta para baixo
    // (por baixo dela); o fim: o pe subindo a frente, a ponta para cima.
    const Vec3 contactAnkle = pelvisFrame(rig, side, ballX - 0.13f * size, ballOut, 0.09f * size, body);
    keys.contact = gestureLegPose3D(leg, side, contactAnkle, 5.0f * Degree, 30.0f * Degree, 0.0f, rig.bodyToPelvis);
    // (O pe sobe menos e fica atras e um pouco para fora da linha dela: subindo
    // na frente, o joelho atravessava a bola que subia.)
    const float rise = (0.10f + 0.10f * power) * size;
    const Vec3 followAnkle = pelvisFrame(rig, side, ballX - 0.11f * size, ballOut + 0.05f * size, 0.09f * size + rise, body);
    keys.follow = gestureLegPose3D(leg, side, followAnkle, 5.0f * Degree, -15.0f * Degree, 0.0f, rig.bodyToPelvis);
    keys.contactTangent = legTangent(leg, side, contactAnkle, normalizedOr(followAnkle - contactAnkle, { 0.0f, 0.0f, 1.0f }),
        5.0f * Degree, 30.0f * Degree, 0.0f, rig.bodyToPelvis);
    keys.followDistance = (followAnkle - contactAnkle).length();
    keys.contactSpeed = 1.2f;
    return keys;
}

// O peito e a cabeca (24/10, pedido do usuario): gestos so do corpo, as
// pernas seguindo a locomocao (legWeight 0; a perna das chaves e a de pe, que
// nao aparece). O dominio no peito (o toque): o peito cede para tras na bola e
// a cabeca olha ela descer, os bracos abertos. O passe de peito (o chute): o
// peito empurra a frente, girando ate o rumo da bola. O cabeceio: o tronco e a
// cabeca armam para tras e batem a frente no rumo da bola (a cabeca vai la),
// mais pela barra; a bola para baixo (o gesto do mouse) e um aceno maior, a
// alta, menor.
GestureKeys3D bodyPartKeys(const GestureSpec3D& spec, const GestureRig3D& rig) {
    GestureKeys3D keys;
    const int side = spec.side == 1 ? 1 : 0;
    const float power = std::clamp(spec.power, 0.0f, 1.0f);
    const float lift = std::clamp(spec.lift, -1.0f, 1.0f);
    const float seconds = std::max(0.12f, spec.seconds);
    const GestureLegRig3D& leg = rig.legs[static_cast<std::size_t>(side)];
    const Vec3 standing = leg.hip + rig.bodyToPelvis.rotate({ 0.0f, 0.0f, -0.97f * (leg.thigh + leg.shin) });
    keys.windup = keys.contact = keys.follow = gestureLegPose3D(leg, side, standing, 0.0f, 0.0f, 0.0f, rig.bodyToPelvis);
    keys.legWeight = 0.0f;
    keys.contactSpeed = 0.0f;
    keys.duration = seconds;
    GestureBodyPeaks3D& body = keys.body;
    const float aim = std::clamp(spec.turn, -60.0f * Degree, 60.0f * Degree);
    if (spec.part == GesturePart3D::Chest) {
        const bool pass = spec.kind == GestureKind3D::Shot;
        keys.contactTime = std::clamp(0.18f * seconds, 0.05f, 0.09f);
        keys.followTime = keys.contactTime + 0.35f * seconds;
        body.spinePitch = (pass ? 6.0f + 10.0f * power : -(12.0f + 8.0f * power)) * Degree;
        body.drop = pass ? 0.01f + 0.01f * power : 0.03f;
        body.forward = pass ? 0.02f * power : -0.02f;
        body.headPitch = 22.0f * Degree;
        body.turn = pass ? 0.6f * aim : 0.0f;
        // (Os bracos abertos para os lados e um pouco para tras, o cotovelo
        // quase reto: a frente e dobrados, parecia que ia pegar a bola com a mao.)
        body.armForward = { -12.0f * Degree, -12.0f * Degree };
        body.armOut = { 50.0f * Degree, 50.0f * Degree };
        body.elbow = { 10.0f * Degree, 10.0f * Degree };
        return keys;
    }
    const float down = std::max(0.0f, -lift), up = std::max(0.0f, lift);
    keys.windupTime = std::clamp((0.08f + 0.10f * power) * seconds, 0.04f, 0.14f);
    keys.contactTime = keys.windupTime + std::clamp(0.10f * seconds, 0.04f, 0.07f);
    keys.followTime = keys.contactTime + 0.30f * seconds;
    body.windupBack = (6.0f + 14.0f * power) * Degree;
    body.windupHead = (8.0f + 12.0f * power) * Degree;
    body.spinePitch = (6.0f + 14.0f * power + 8.0f * down - 6.0f * up) * Degree;
    body.headPitch = (10.0f + 20.0f * power + 12.0f * down - 10.0f * up) * Degree;
    body.turn = 0.4f * aim;
    body.headYaw = 0.6f * aim;
    body.drop = 0.02f + 0.02f * power;
    body.armForward = { (10.0f + 15.0f * power) * Degree, (10.0f + 15.0f * power) * Degree };
    body.armOut = { (20.0f + 15.0f * power) * Degree, (20.0f + 15.0f * power) * Degree };
    body.elbow = { 30.0f * Degree, 30.0f * Degree };
    return keys;
}

} // namespace

Vec3 gestureNominalBall3D(const GestureSpec3D& spec) {
    const int reach = std::clamp(spec.reach, 0, 2);
    const float sign = spec.side == 1 ? -1.0f : 1.0f;
    if (spec.kind == GestureKind3D::Shot) return { spec.high ? HighShotX : ShotX, sign * ShotOut[reach], BallZ };
    return { TouchX[std::clamp(spec.depth, 0, 2)], sign * TouchOut[reach], BallZ };
}

GestureLegPose3D gestureLegPose3D(const GestureLegRig3D& leg, int side, Vec3 ankle, float twist, float plantar,
    float inversion, Quaternion bodyToPelvis) {
    const float l1 = leg.thigh, l2 = leg.shin;
    const Vec3 hip = leg.hip;
    Vec3 reach = ankle - hip;
    const float wanted = reach.length();
    const Vec3 along = normalizedOr(reach, { 0.0f, 0.0f, -1.0f });
    const float distance = std::clamp(wanted, std::abs(l1 - l2) + 0.02f, 0.995f * (l1 + l2));
    reach = along * distance;
    // O joelho: para a frente girado pelo twist (a ponta do pe para fora gira
    // o joelho junto) e para cima com a perna indo a frente.
    const float sideTwist = side == 0 ? twist : -twist;
    const float forwardness = bodyToPelvis.conjugate().rotate(along).x;
    Vec3 pole = bodyToPelvis.rotate(
        rotateZ({ 1.0f, 0.0f, 0.0f }, sideTwist) + Vec3 { 0.0f, 0.0f, 1.0f } * (1.5f * std::max(0.0f, forwardness)));
    Vec3 bend = pole - along * dot(pole, along);
    const Vec3 up = bodyToPelvis.rotate({ 0.0f, 0.0f, 1.0f });
    bend = normalizedOr(bend, normalizedOr(up - along * dot(up, along), bodyToPelvis.rotate({ 1.0f, 0.0f, 0.0f })));
    const float cosine = std::clamp((l1 * l1 + distance * distance - l2 * l2) / (2.0f * l1 * distance), -1.0f, 1.0f);
    const float sine = std::sqrt(std::max(0.0f, 1.0f - cosine * cosine));
    const Vec3 knee = hip + along * (l1 * cosine) + bend * (l1 * sine);
    const Vec3 thighDirection = normalizedOr(knee - hip, along);
    const Vec3 shinDirection = normalizedOr(hip + reach - knee, along);
    // A frente do joelho (perpendicular a coxa, para o lado da dobra).
    const Vec3 front = normalizedOr(bend - thighDirection * dot(bend, thighDirection), { 1.0f, 0.0f, 0.0f });
    // A coxa de repouso: o osso para baixo, a frente do joelho para a frente.
    const Quaternion turn = rotationBetweenFrames({ 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f }, front, thighDirection);
    const Vec3 thigh = rotationVector((leg.hipFrame.conjugate() * turn * leg.hipFrame).normalized());
    const float kneeAngle = std::acos(std::clamp(dot(thighDirection, shinDirection), -1.0f, 1.0f));
    GestureLegPose3D pose;
    pose.set(thigh, std::max(3.0f * Degree, kneeAngle), { plantar, side == 0 ? inversion : -inversion, 0.0f });
    return pose;
}

GestureKeys3D gestureKeys3D(const GestureSpec3D& spec, const GestureRig3D& rig) {
    if (spec.part != GesturePart3D::Foot) return bodyPartKeys(spec, rig);
    // (Levantar a bola alta e o toque alto.)
    if (spec.kind == GestureKind3D::Raise && !spec.high) return raiseKeys(spec, rig);
    if (spec.receive) return receptionKeys(spec, rig);
    GestureKeys3D keys;
    const int side = spec.side == 1 ? 1 : 0;
    const float s = side == 0 ? 1.0f : -1.0f;
    const int reach = std::clamp(spec.reach, 0, 2);
    const float r = static_cast<float>(reach);
    const float power = std::clamp(spec.power, 0.0f, 1.0f);
    const float lift = std::clamp(spec.lift, -1.0f, 1.0f);
    const float seconds = std::max(0.12f, spec.seconds);
    GestureBodyPeaks3D& body = keys.body;
    const GestureLegRig3D& leg = rig.legs[static_cast<std::size_t>(side)];
    // (As medidas do catalogo sao do jogador de 1,80 m: as pernas de outro
    // tamanho escalam.)
    const float size = std::clamp((leg.thigh + leg.shin) / 0.832f, 0.75f, 1.3f);
    // A bola alta (24/10, pedido do usuario): o pe na altura do joelho e o corpo
    // inclinando mais - para o lado do apoio e para tras (para erguer a perna),
    // a pelve caindo para o lado dele. Gesto pronto: a altura e a de referencia
    // (a bola so escolhe o balde). high: 1 na bola alta.
    const float top = std::max(0.40f, kneeTop(leg, size));
    const float ballHeight = spec.high ? highBallZ(leg, size) : BallZ;
    const float high = spec.high ? 1.0f : 0.0f;

    if (spec.kind == GestureKind3D::Shot) {
        // O chute: o contato logo (a forca ja saiu no comando) e o
        // acompanhamento pela barra - mais longo e mais alto, o corpo junto.
        keys.contactTime = std::clamp(0.10f * seconds, 0.035f, 0.06f);
        // (O acompanhamento curto e a volta longa: a perna sobe e ja desce; com
        // 40% do tempo ela ficava parada no alto.)
        keys.followTime = keys.contactTime + 0.28f * seconds;
        keys.duration = seconds;
        const float twists[3] = { 8.0f, 16.0f, 26.0f };
        body.drop = 0.020f + 0.020f * power + 0.012f * r;
        body.left = s * 0.020f * r;
        body.yaw = -s * (6.0f + 10.0f * power) * Degree;
        // A cintura gira ate o rumo da bola (24/10, pedido do usuario: andando
        // de lado, o chute parecia ir para o lado contrario). A perna vai com
        // ela: a batida e o acompanhamento saem no rumo da bola; o contato e a
        // armada ficam onde estavam (girados de volta pelo quanto ela ja girou).
        body.turn = std::clamp(spec.turn, -70.0f * Degree, 70.0f * Degree);
        body.roll = s * 2.5f * r * Degree;
        body.spineTwist = -0.6f * body.yaw;
        // (A inclinacao para o lado do apoio maior: 24/10, ele sentiu falta.)
        body.spineRoll = -s * (6.0f + 6.0f * r + 4.0f * power) * Degree;
        body.spinePitch = (4.0f + 4.0f * std::max(0.0f, -lift)) * Degree;
        body.spineBack = (4.0f + 8.0f * power + 6.0f * std::max(0.0f, lift)) * Degree;
        body.headPitch = 10.0f * Degree;
        // (O braco de fora mais para o lado que para a frente e o cotovelo
        // dobrado: para a frente demais, a mao parava na frente do rosto.)
        body.armForward = { -(25.0f + 20.0f * power) * Degree, (20.0f + 20.0f * power) * Degree };
        body.armOut = { (18.0f + 12.0f * power) * Degree, (35.0f + 25.0f * power) * Degree };
        body.elbow = { 15.0f * Degree, 30.0f * Degree };
        // (O chute fraco - conduzindo, um passe curto - com o corpo contido:
        // 45% no mais fraco, cheio no forte.)
        {
            const float held = 0.45f + 0.55f * power;
            body.drop *= held;
            body.left *= held;
            body.yaw *= held;
            body.roll *= held;
            body.spineTwist *= held;
            body.spineRoll *= held;
            body.spinePitch *= held;
            body.spineBack *= held;
            for (std::size_t arm = 0; arm < 2; ++arm) {
                body.armForward[arm] *= held;
                body.armOut[arm] *= held;
                body.elbow[arm] *= held;
            }
        }
        const float ballOut = ShotOut[reach] * size;
        const float ballX = (spec.high ? HighShotX : ShotX) * size;
        // A pegada de impulso (24/10, pedido do usuario: "deve existir, de
        // acordo com a potencia", sem exagero): a perna arma para tras na linha
        // da bola - fraco, um recuo curto com o joelho meio dobrado; cheio, o
        // calcanhar sobe atras. (A forca ja saiu no comando: o armar e curto.)
        keys.windupTime = std::clamp((0.10f + 0.12f * power) * seconds, 0.05f, 0.25f);
        // O chute curto e um chute menor: abaixo de 0,45 s, a armada e o
        // acompanhamento encolhem com o tempo (a forca na bola e a mesma). Em
        // 0,25 s, do tamanho cheio, o pe passava de 5.000 m/s2.
        const float reachOfMotion = power * std::clamp(seconds / 0.45f, 0.4f, 1.0f);
        {
            // (No voleio a perna arma mais alta.)
            const float back = (0.12f + 0.22f * reachOfMotion) * size,
                        below = (0.78f - 0.28f * reachOfMotion - 0.15f * high) * size;
            const float windupOut = 0.3f * (ballOut - s * leg.hip.y);
            keys.windup = gestureLegPose3D(leg, side,
                leg.hip + rig.bodyToPelvis.rotate(rotateZ({ -back, s * windupOut, -below }, -0.5f * body.turn)), 0.0f,
                40.0f * Degree, 0.0f, rig.bodyToPelvis);
        }
        // (O voleio: o contato na altura da bola; a inclinacao e a pelve maiores.)
        if (spec.high) {
            body.spineRoll *= 1.0f + high;
            body.spineBack += 8.0f * high * Degree;
            body.roll += s * 6.0f * high * Degree;
        }
        const Vec3 contactAnkle = pelvisFrame(rig, side, ballX - 0.09f * size, ballOut,
            spec.high ? std::min(ballHeight + 0.06f * size, top) : 0.21f * size, body, body.turn);
        keys.contact = gestureLegPose3D(leg, side, contactAnkle, twists[reach] * Degree, 45.0f * Degree, 0.0f,
            rig.bodyToPelvis);
        // O fim do acompanhamento: a perna quase esticada passando a frente e
        // subindo pela barra (fraco, 50 graus abaixo do horizontal; cheio, 10
        // acima) - com o joelho dobrado ali, parecia marcha. Reto no rumo do
        // chute: guarda 35% do quanto o contato ficou para o lado do quadril
        // (24/10, pedido do usuario: cruzando para a linha do corpo, ate 32
        // graus, a perna ia demais para o outro lado).
        const float elevation = (-50.0f + 60.0f * reachOfMotion + 8.0f * lift + 18.0f * high) * Degree;
        const float length = 0.985f * (leg.thigh + leg.shin), horizontal = std::cos(elevation) * length;
        const float outward = std::clamp(0.15f * (ballOut - s * leg.hip.y), -0.3f * horizontal, 0.3f * horizontal);
        // (No referencial da pelve: o quadril nao anda com a descida dela. E
        // contra o giro da pelve no gesto, quase no pico ali: girando junto, a
        // perna cruzava para o outro lado.)
        const Vec3 followAnkle = leg.hip
            + rig.bodyToPelvis.rotate(rotateZ({ std::sqrt(std::max(0.0f, horizontal * horizontal - outward * outward)),
                s * outward, std::sin(elevation) * length }, -0.9f * body.yaw));
        keys.follow = gestureLegPose3D(leg, side, followAnkle, 4.0f * Degree, 35.0f * Degree, 0.0f, rig.bodyToPelvis);
        // O pe passa pela bola no mais rapido do gesto, em arco em volta do
        // quadril (para a frente) e com o joelho esticando - a chicotada da
        // canela. (Parado na pose de contato, ele arrancava do zero depois:
        // 2.300 m/s2. Indo para cima, a perna encurtava: o joelho dobrava a 90
        // graus depois da bola, a canela pendurada.)
        const Vec3 reachBody = rig.bodyToPelvis.conjugate().rotate(contactAnkle - leg.hip);
        const Vec3 arc = normalizedOr({ -reachBody.z, 0.0f, reachBody.x }, { 1.0f, 0.0f, 0.0f });
        const Vec3 drive = rig.bodyToPelvis.rotate(normalizedOr(arc + normalizedOr(reachBody, {}) * 0.35f, arc));
        keys.contactTangent = legTangent(leg, side, contactAnkle, drive, twists[reach] * Degree, 45.0f * Degree, 0.0f,
            rig.bodyToPelvis);
        keys.followDistance = (followAnkle - contactAnkle).length();
        keys.contactSpeed = 1.6f;
        // (O chute muito fraco - conduzindo, um passe curto - fica dentro do
        // passo: a perna 55% do gesto no mais fraco, cheia a partir de 0,3. A
        // bola alta nao: ela nunca chegaria la.)
        if (!spec.high) keys.legWeight = 0.55f + 0.45f * std::clamp(power / 0.3f, 0.0f, 1.0f);
        return keys;
    }

    // O manejo: o toque logo e o acompanhamento curto no rumo do movimento; o
    // corpo vai junto com intensidade parecida com a do chute (pedido do
    // usuario).
    keys.contactTime = std::clamp(0.16f * seconds, 0.04f, 0.07f);
    keys.followTime = keys.contactTime + 0.30f * seconds;
    keys.duration = seconds;
    const int depth = std::clamp(spec.depth, 0, 2);
    const bool in = spec.move == GestureMove3D::In;
    const float ballOut = TouchOut[reach] * size;
    const float ballX = TouchX[depth] * size;
    const float ballZ = ballHeight;
    // O corpo pela forca do toque (24/10, pedido do usuario: o toque suave -
    // conduzindo - suave, os bracos sutis: 35% no mais fraco, cheio no forte)
    // e pelo alcance (a perna longe pede mais equilibrio). A cabeca olha a
    // bola sempre.
    const float softness = 0.35f + 0.65f * power;
    body.drop = (0.012f + 0.018f * r + (depth == 2 && in ? 0.01f : 0.0f)) * softness;
    body.forward = (depth == 2 ? 0.02f : depth == 0 ? -0.02f : 0.0f) * softness;
    body.left = s * 0.015f * r * softness;
    // (O toque curto tambem com o corpo: so a perna mexendo, ele sumia.)
    body.roll = s * 2.0f * r * softness * Degree;
    body.spineRoll = -s * (6.0f + 8.0f * r) * softness * Degree;
    body.spinePitch = (8.0f + 4.0f * r) * (0.5f + 0.5f * softness) * Degree;
    body.headPitch = 14.0f * Degree;
    body.armForward = { -10.0f * softness * Degree, 12.0f * softness * Degree };
    body.armOut = { (10.0f + 9.0f * r) * softness * Degree, (14.0f + 12.0f * r) * softness * Degree };
    body.elbow = { 10.0f * softness * Degree, 14.0f * softness * Degree };
    // O tornozelo em relacao a bola no contato, a pose da perna e o
    // acompanhamento (x frente, "fora", z), por frente-tras e movimento.
    struct Touch { float dx, dout, dz, twist, plantar, inversion, fx, fout, fz, yaw; };
    static const Touch touches[3][2] = {
        // atras: puxar (a sola traz de tras para a frente e para dentro), jogar (o calcanhar)
        { { -0.07f, 0.05f, 0.09f, 20.0f, -10.0f, 0.0f, 0.18f, -0.08f, -0.02f, -4.0f },
          { 0.10f, -0.02f, 0.07f, 15.0f, -10.0f, 0.0f, -0.14f, 0.06f, 0.05f, 4.0f } },
        // no meio: puxar (a parte de dentro varre), jogar (a parte de fora)
        { { -0.02f, 0.11f, 0.05f, 35.0f, 5.0f, -5.0f, 0.08f, -0.18f, 0.02f, -6.0f },
          { -0.03f, -0.10f, 0.05f, -20.0f, 15.0f, 20.0f, 0.06f, 0.14f, 0.03f, 5.0f } },
        // a frente: puxar (a sola por cima traz para tras), jogar (o peito do pe empurra)
        { { 0.02f, 0.00f, 0.21f, 0.0f, -18.0f, 0.0f, -0.22f, -0.06f, -0.06f, 0.0f },
          { -0.11f, 0.00f, 0.08f, 5.0f, 35.0f, 0.0f, 0.16f, 0.05f, 0.04f, 5.0f } },
    };
    // (A bola alta: o lado de dentro ou de fora na altura dela - a sola por
    // cima e o calcanhar nao servem la.)
    const Touch& t = touches[spec.high ? 1 : depth][in ? 0 : 1];
    body.yaw = s * t.yaw * Degree;
    body.spineTwist = -0.5f * body.yaw + s * 0.6f * t.yaw * Degree;
    if (spec.high) {
        body.spineRoll *= 1.0f + 1.2f * high;
        body.spineBack = (depth == 2 ? 8.0f : 4.0f) * high * Degree;
        body.roll += s * 5.0f * high * Degree;
        body.drop += 0.015f * high;
    }
    // (O pe um pouco acima no contato: rente ao chao, o toque curto sumia. Na
    // bola alta, o pe no meio dela.)
    const float contactX = ballX + t.dx * size, contactOut = ballOut + t.dout * size,
                contactZ = spec.high ? ballZ - 0.02f * size : ballZ + (t.dz + 0.02f) * size;
    const Vec3 contactAnkle = pelvisFrame(rig, side, contactX, contactOut, contactZ, body);
    keys.contact = gestureLegPose3D(leg, side, contactAnkle, t.twist * Degree, t.plantar * Degree, t.inversion * Degree,
        rig.bodyToPelvis);
    // (Na bola alta o joelho sobe primeiro e a canela estica ate ela: reto, o pe
    // ia rente ao chao a 18 m/s e so subia no fim.)
    if (spec.high) {
        const Vec3 liftAnkle = pelvisFrame(rig, side, 0.45f * contactX, std::max(0.02f * size, 0.85f * contactOut),
            contactZ - 0.06f * size, body);
        keys.lift = gestureLegPose3D(leg, side, liftAnkle, t.twist * Degree, t.plantar * Degree, t.inversion * Degree,
            rig.bodyToPelvis);
        keys.liftValid = true;
    }
    // (O toque curto e menor: abaixo de 0,28 s o acompanhamento encolhe.)
    const float k = (0.9f + 0.7f * power) * std::clamp(seconds / 0.28f, 0.45f, 1.0f);
    float followX = contactX + t.fx * k * size;
    float followOut = contactOut + t.fout * k * size;
    const float followZ = std::max(0.13f * size, contactZ + t.fz * k * size);
    // As pernas nao se cruzam: ao lado do apoio o pe do gesto para na linha
    // do corpo; a frente dele pode passar um pouco.
    followOut = std::max(followOut, followX > 0.25f * size ? -0.05f * size : 0.02f * size);
    followX = std::clamp(followX, -0.40f * size, 0.65f * size);
    const Vec3 followAnkle = pelvisFrame(rig, side, followX, followOut, followZ, body);
    keys.follow = gestureLegPose3D(leg, side, followAnkle, t.twist * Degree, t.plantar * Degree, t.inversion * Degree,
        rig.bodyToPelvis);
    // O pe passa pela bola andando no rumo do acompanhamento.
    keys.contactTangent = legTangent(leg, side, contactAnkle, normalizedOr(followAnkle - contactAnkle, { 1.0f, 0.0f, 0.0f }),
        t.twist * Degree, t.plantar * Degree, t.inversion * Degree, rig.bodyToPelvis);
    keys.followDistance = (followAnkle - contactAnkle).length();
    keys.contactSpeed = 1.4f;
    // (O toque a frente fraco - conduzindo - fica dentro do passo: a perna 55%
    // do gesto no mais fraco, cheia a partir de 0,3; 24/10, pedido do usuario.)
    if (depth == 2 && !spec.high) keys.legWeight = 0.55f + 0.45f * std::clamp(power / 0.3f, 0.0f, 1.0f);
    return keys;
}

} // namespace MatterEngine
