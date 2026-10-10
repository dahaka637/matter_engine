#include "Engine/Character/BallControl3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace MatterEngine {

namespace {

float tune(const char* name, float fallback) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::strtof(value, nullptr) : fallback;
}

constexpr float Pi = 3.14159265358979323846f;

// ---------------------------------------------------------------------------
// A forca na bola (os padroes de BallForceSettings3D; o laboratorio tem uma aba
// com todos, 07/10). Os T_* no ambiente mudam os padroes.
// O resultado na bola (aprovado pelo usuario em 06/10):
//  - Toque: define a velocidade da bola no rumo dele - a do corpo nesse rumo
//    (carry) mais a forca da barra - e leva um pouco da que ela trazia. A
//    forca e uma curva: fraca embaixo e bem mais forte em cima (o cheio e um
//    passe forte): minimo + forca x barra^curva. A sola (puxar e rolar), a
//    mesma conta mais fraca.
//  - A forca pela velocidade (06/10 noite, pedido do usuario: "falta de forca
//    quando tento fazer a bola ir em direcao contraria quando em movimento" e
//    "levar em consideracao a velocidade do meu boneco - correndo fica mais
//    forte"): a velocidade do corpo no rumo do toque e a de agora ou a que ele
//    vai ter em carryAhead s, indo a pedida (arrancando ou mudando de rumo, a
//    bola ia na de antes e o corpo passava por ela) - tambem na sola; so no
//    corpo animado (o fisico parte devagar demais para a previsao). Correndo,
//    o toque empurra mais: speedPush (m/s por m/s do corpo) somado a forca da
//    barra (no sprint o toque rapido deixava a bola nos pes). Contra o
//    movimento, o corpo empresta reverseBoost da velocidade dele no rumo
//    contrario (e a bola o mesmo do que trazia contra o toque - o quique do
//    modelo aprovado, que era 0,3): a bola volta de verdade.
//  - O golpe com atrito (Pro Soccer Online): no rumo do gesto a bola sai na
//    velocidade que ele define mais o quique do que ela trazia contra ele; de
//    lado, o atrito tira parte do movimento e fica sideKeep; ja indo no rumo
//    mais rapido que ele, fica aheadKeep do excesso.
//  - Chute: velocidade pela barra; elevacao (graus) padrao e os extremos do
//    gesto do mouse (alta e rasteira); efeito: giro de lado no gesto cheio
//    (rad/s), quanto a bola sai para fora da mira (graus) para a curva voltar
//    nela; giro para tras (alta) e para a frente (rasteira).
BallForceSettings3D buildDefaultForce() {
    BallForceSettings3D f;
    f.touchMinimum = tune("T_BallBase", f.touchMinimum);
    f.touchPower = tune("T_BallPower", f.touchPower);
    f.touchCurve = tune("T_BallCurve", f.touchCurve);
    f.soleMinimum = tune("T_BallSoleBase", f.soleMinimum);
    f.solePower = tune("T_BallSolePower", f.solePower);
    f.soleCurve = tune("T_BallSoleCurve", f.soleCurve);
    f.carry = tune("T_BallCarry", f.carry);
    f.carryAhead = tune("T_BallCarryAhead", f.carryAhead);
    f.speedPush = tune("T_BallSpeedPush", f.speedPush);
    f.reverseBoost = tune("T_BallReverseBoost", f.reverseBoost);
    f.touchSideKeep = tune("T_BallKeepSide", f.touchSideKeep);
    f.touchAheadKeep = tune("T_BallKeepAhead", f.touchAheadKeep);
    f.shotMinimum = tune("T_ShotMin", f.shotMinimum);
    f.shotPower = tune("T_ShotPower", f.shotPower);
    f.shotCurve = tune("T_ShotCurve", f.shotCurve);
    f.shotElevationMinimum = tune("T_ShotElevMin", f.shotElevationMinimum);
    f.shotElevationPower = tune("T_ShotElev", f.shotElevationPower);
    f.shotHighElevation = tune("T_ShotHigh", f.shotHighElevation);
    f.shotLowElevation = tune("T_ShotLow", f.shotLowElevation);
    f.shotCurveSpin = tune("T_ShotCurveSpin", f.shotCurveSpin);
    f.shotCurveLaunch = tune("T_ShotCurveLaunch", f.shotCurveLaunch);
    f.shotBackSpin = tune("T_ShotBackSpin", f.shotBackSpin);
    f.shotTopSpin = tune("T_ShotTopSpin", f.shotTopSpin);
    f.shotSideKeep = tune("T_ShotKeepSide", tune("T_BallKeepSide", f.shotSideKeep));
    f.shotBounce = tune("T_BallKeepBounce", f.shotBounce);
    f.shotAheadKeep = tune("T_ShotKeepAhead", tune("T_BallKeepAhead", f.shotAheadKeep));
    f.kickRunSlowdown = tune("T_KickRunSlowdown", f.kickRunSlowdown);
    f.holdSeconds = tune("T_BallHold", f.holdSeconds);
    f.hitboxMargin = tune("T_BallHitbox", f.hitboxMargin);
    return f;
}

const BallForceSettings3D& defaultForce() {
    static const BallForceSettings3D force = buildDefaultForce();
    return force;
}

// ---------------------------------------------------------------------------
// TEMPTUNE: o gesto (rework de 06/10), calibrado pelas previas.
// Area (pedido do usuario): da cintura ate a base dos pes, raio de uma perna
// esticada em volta da pelve.
const float ReachRadius = tune("T_BallReach", 0.95f);
const float WaistAbovePelvis = tune("T_BallWaist", 0.05f);
// O armado: a perna no maximo a isto (rad/s do balanco em volta do quadril).
const float WindupSwingRate = tune("T_BallWindupRate", 14.0f);
// Entre dois pedidos aceitos (s, desde o toque).
const float RequestCooldown = tune("T_BallCooldown", 0.25f);
// Andando: a acao espera o pe escolhido sair do chao no maximo isto (s).
const float MovingWait = tune("T_BallMovingWait", 0.45f);
// A perna inteira esticada (fracao de coxa + canela) no contato.
const float ContactStretch = tune("T_BallStretch", 0.985f);
// O peito do pe no chute: a ponta para baixo (rad do horizontal; + forca).
const float LacesPitch = tune("T_KickLacesPitch", 0.80f);
const float LacesPitchPower = tune("T_KickLacesPitchPower", 0.15f);
// O pe virado para fora no lado de dentro (rad do rumo do corpo, maximo) e
// para dentro no lado de fora.
const float InsideTurnMaximum = tune("T_BallInsideTurn", 1.25f);
const float OutsideTurnMaximum = tune("T_BallOutsideTurn", 0.80f);
// Chute: armado (coxa para tras e joelho dobrado, rad: fraco + cheio x
// forca), acompanhamento (coxa para cima, joelho quase esticado) e tempos (s).
const float KickWindupThigh = tune("T_KickWindupThigh", 0.12f);
const float KickWindupThighPower = tune("T_KickWindupThighPower", 0.40f);
const float KickWindupKnee = tune("T_KickWindupKnee", 0.85f);
const float KickWindupKneePower = tune("T_KickWindupKneePower", 1.00f);
const float KickWindupOut = tune("T_KickWindupOut", 0.10f);
const float KickFollowThigh = tune("T_KickFollowThigh", 0.50f);
const float KickFollowThighPower = tune("T_KickFollowThighPower", 0.75f);
const float KickFollowKnee = tune("T_KickFollowKnee", 0.35f);
const float KickFollowKneePower = tune("T_KickFollowKneePower", -0.15f);
const float KickFollowIn = tune("T_KickFollowIn", 0.10f);
const float KickWindupTime = tune("T_KickWindupTime", 0.15f);
const float KickWindupTimePower = tune("T_KickWindupTimePower", 0.12f);
const float KickStrikeTime = tune("T_KickStrikeTime", 0.10f);
const float KickStrikeTimePower = tune("T_KickStrikeTimePower", -0.02f);
const float KickFollowTime = tune("T_KickFollowTime", 0.12f);
const float KickFollowTimePower = tune("T_KickFollowTimePower", 0.05f);
const float KickRecoverTime = tune("T_KickRecoverTime", 0.22f);
const float KickRecoverTimePower = tune("T_KickRecoverTimePower", 0.08f);
// A pelve desce a mais (afundo) para o pe alcancar a bola, ate isto (m); e,
// no chute com a bola a frente do apoio, vai sobre ele ate isto (m).
const float LungeDropMaximum = tune("T_BallLungeDrop", 0.12f);
const float LungeForwardMaximum = tune("T_BallLungeForward", 0.12f);
// Na volta, o joelho dobra no meio do caminho (m a menos na perna).
const float RecoverKneeBend = tune("T_BallRecoverBend", 0.14f);
// A batida acelera ate a bola: velocidade no contato em relacao a media da
// batida (balanco da coxa e o joelho esticando - o chicote).
const float StrikeSwingRate = tune("T_StrikeSwingRate", 1.5f);
const float StrikeLengthRate = tune("T_StrikeLengthRate", 2.2f);
// Toques (lado de dentro/fora, peito do pe): armado atras e acima do contato
// e acompanhamento (m, fraco + cheio x forca) e tempos (s).
const float TouchBack = tune("T_TouchBack", 0.06f);
const float TouchBackPower = tune("T_TouchBackPower", 0.30f);
const float TouchLift = tune("T_TouchLift", 0.045f);
const float TouchLiftPower = tune("T_TouchLiftPower", 0.10f);
const float TouchAhead = tune("T_TouchAhead", 0.06f);
const float TouchAheadPower = tune("T_TouchAheadPower", 0.28f);
const float TouchWindupTime = tune("T_TouchWindupTime", 0.08f);
const float TouchWindupTimePower = tune("T_TouchWindupTimePower", 0.10f);
const float TouchStrikeTime = tune("T_TouchStrikeTime", 0.07f);
const float TouchStrikeTimePower = tune("T_TouchStrikeTimePower", 0.04f);
const float TouchFollowTime = tune("T_TouchFollowTime", 0.08f);
const float TouchFollowTimePower = tune("T_TouchFollowTimePower", 0.10f);
const float TouchRecoverTime = tune("T_TouchRecoverTime", 0.16f);
const float TouchRecoverTimePower = tune("T_TouchRecoverTimePower", 0.10f);
// O toque fraco chega na bola freando (a velocidade no contato em relacao a
// media da batida, fraco + cheio x forca: o forte acelera como o chute).
const float TouchStrikeRate = tune("T_TouchStrikeRate", 0.55f);
const float TouchStrikeRatePower = tune("T_TouchStrikeRatePower", 1.0f);
// Corpo: quanto do desenho (fraco + cheio x forca) no chute e no toque.
const float KickBodyBase = tune("T_KickBody", 0.35f);
const float TouchBodyBase = tune("T_TouchBody", 0.45f);

// ---------------------------------------------------------------------------
// V3 (06/10 noite, pedido do usuario).
// A forca sai nesta fracao do tempo estimado ate o contato visual (0 na
// hora, 1 no contato) - o padrao (1,0 desde 06/10 noite, pedido do usuario:
// "por enquanto"); o laboratorio tem o controle dela.
const float ContactTimingDefault = tune("T_BallContactTiming", 1.0f);
// O pedido guardado esperando dar para comecar (s); passado, descartado - a
// nao ser com a bola na area (a garantia: sai ja).
const float RequestBuffer = tune("T_BallRequestBuffer", 0.25f);
// A garantia (06/10 noite): a bola saindo da area nos proximos ticks - a
// forca ja; e a bola que acabou de sair (a previsao errou por pouco) ainda
// vale por este tempo (s).
const float LeavingTicks = tune("T_BallLeavingTicks", 2.5f);
const float GuaranteeGrace = tune("T_BallGuaranteeGrace", 0.05f);
// A bola lancada pela propria forca (chute, passe: mais que isto em relacao
// ao corpo, m/s): o pedido guardado durante a acao nao a pega na saida.
const float LaunchSpeed = tune("T_BallLaunchSpeed", 3.0f);
// Ate onde a predicao do encontro olha (s).
const float MeetHorizon = tune("T_BallMeetHorizon", 1.0f);
// Parado com a bola andando: o contato cai na chegada dela - o gesto comeca
// quando o encontro bom esta a no maximo isto vezes o tempo natural dele
// (mais cedo, o pedido fica guardado) e encolhe ate isto.
const float AnticipateStretch = tune("T_BallAnticipateStretch", 1.2f);
const float AnticipateCompress = tune("T_BallAnticipateCompress", 0.6f);
// As pernas nunca se cruzam: o tornozelo que joga fica pelo menos isto (m)
// do lado dele da linha do meio do corpo e, lado a lado com o de apoio,
// isto longe dele (m, de lado); a correcao entra suave nesta faixa (m).
const float OwnSideMargin = tune("T_BallOwnSide", 0.03f);
const float FootSideGap = tune("T_BallFootGap", 0.15f);
const float OwnSideSoftness = tune("T_BallOwnSideSoft", 0.03f);
// O contato tirado da bola para nao cruzar: ate isto (m) - alem, a perna
// nao serve (a outra faz).
const float VirtualShiftMaximum = tune("T_BallVirtualShift", 0.16f);
// A bola um pouco atras (ate isto, m, atras da pelve) tocada para a frente:
// o gesto de frente com ela imaginada logo a frente (m a frente da pelve).
const float BehindNear = tune("T_BallBehindNear", 0.30f);
const float VirtualAhead = tune("T_BallVirtualAhead", 0.14f);
// A forca ainda sai ate isto (s) depois do acompanhamento comecar (a bola
// entrando tarde na area).
const float ForceLate = tune("T_BallForceLate", 0.06f);
// O contato pela hitbox (09/10): no corpo fisico o pe de verdade atrasa o
// alvo - a folga vezes isto (como na V2); a varredura entre dois ticks em
// tantos pedacos.
const float PhysicalHitboxScale = tune("T_BallHitboxPhysical", 1.6f);
constexpr int HitboxSweepSteps = 4;
// Correndo (corpo animado), o contato no minimo a isto (s) do pedido; com a
// hitbox, 0,12 (a batida de 0,10 s numa bola longe no sprint saltava a
// canela 35 graus num tick; com 0,12, no maximo 28 na sonda, e toca mais).
const float RunningSoonest = tune("T_BallRunSoonest", 0.10f);
const float RunningSoonestHitbox = tune("T_BallRunSoonestHitbox", 0.12f);
// Modo fisico: o pe chega no alvo com atraso (s, somado ao tempo estimado) e
// o alvo fica no maximo isto (m) a frente do pe de verdade.
const float PhysicalLag = tune("T_BallPhysLag", 0.04f);
const float PhysicalReach = tune("T_BallPhysReach", 0.35f);
// A bola rolando freia (m/s2) e o corpo acelera para a velocidade pedida
// (m/s2, a capsula) - a predicao do encontro.
const float RollDeceleration = tune("T_BallRollDecel", 0.6f);
const float BodyAcceleration = tune("T_BallBodyAccel", 16.0f);
const float PhysicalBodyAcceleration = tune("T_BallPhysBodyAccel", 4.0f);
// O tempo ate o contato pelo caminho do pe (parado): a velocidade media do
// tornozelo no armado e na batida (m/s, fraco + cheio x forca), limitada em
// volta dos tempos de referencia (vezes eles).
const float TouchWindupSpeed = tune("T_TouchWindupSpeed", 2.4f);
const float TouchWindupSpeedPower = tune("T_TouchWindupSpeedPower", 0.4f);
const float TouchStrikeSpeed = tune("T_TouchStrikeSpeed", 2.2f);
const float TouchStrikeSpeedPower = tune("T_TouchStrikeSpeedPower", 1.3f);
const float KickWindupSpeed = tune("T_KickWindupSpeed", 2.7f);
const float KickWindupSpeedPower = tune("T_KickWindupSpeedPower", 0.7f);
const float KickStrikeSpeed = tune("T_KickStrikeSpeed", 6.9f);
const float KickStrikeSpeedPower = tune("T_KickStrikeSpeedPower", 5.3f);
const float TimingShortest = tune("T_BallTimeMin", 0.75f);
const float TimingLongest = tune("T_BallTimeMax", 1.4f);
// O reforco de estabilidade do modo fisico: entra e sai (s).
const float StabilityIn = tune("T_BallStabilityIn", 0.06f);
const float StabilityOut = tune("T_BallStabilityOut", 0.20f);
// Um gesto cortado por outro: a pose dele sai neste tempo (s).
const float BlendOldSeconds = tune("T_BallBlendOld", 0.12f);
// Depois da forca, o contato acompanha a bola lenta ate isto (m) alem do
// planejado (correndo, mais que isso o pe voava ate ela).
const float FollowMaximum = tune("T_BallFollowMax", 0.15f);

float smooth(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float smoother(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }

Vec3 rotateZ(Vec3 v, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

float signedAngle(Vec3 from, Vec3 to) {
    return std::atan2(from.x * to.y - from.y * to.x, from.x * to.x + from.y * to.y);
}

// Hermite cubico entre p0 e p1 (tangentes m0, m1 ja em unidades de valor por
// segmento).
float hermite(float p0, float p1, float m0, float m1, float u) {
    const float u2 = u * u, u3 = u2 * u;
    return (2.0f * u3 - 3.0f * u2 + 1.0f) * p0 + (u3 - 2.0f * u2 + u) * m0 + (-2.0f * u3 + 3.0f * u2) * p1
        + (u3 - u2) * m1;
}

// A velocidade da bola depois do chute: a definida por ele mais o que fica da
// que ela trazia (o atrito: de lado, o quique contra e o excesso no rumo).
Vec3 keepBallMotion(Vec3 defined, Vec3 ballVelocity, float sideKeep, float bounce, float aheadKeep) {
    const float speed = defined.length();
    if (speed < 1e-4f) return defined + ballVelocity * sideKeep;
    const Vec3 normal = defined * (1.0f / speed);
    const float along = dot(ballVelocity, normal);
    const Vec3 side = ballVelocity - normal * along;
    const float extra = along < 0.0f ? bounce * -along : aheadKeep * std::max(0.0f, along - speed);
    return defined + normal * extra + side * sideKeep;
}

// O toque: a forca da barra (e a da velocidade, correndo), a velocidade do
// corpo no rumo dele (a de agora ou a que ele vai ter), o que o corpo indo
// para o outro lado empresta, e o que fica da que a bola trazia.
Vec3 dribbleTouchVelocity(Vec3 direction, float power, Vec3 bodyVelocity, Vec3 aheadVelocity, Vec3 ballVelocity,
    bool sole, const BallForceSettings3D& f) {
    direction = planar(direction);
    if (direction.lengthSquared() < 1e-6f) return planar(ballVelocity);
    direction = direction.normalized();
    const float bar = std::clamp(power, 0.0f, 1.0f);
    const Vec3 body = planar(bodyVelocity), ahead = planar(aheadVelocity);
    const float speed = std::max(body.length(), ahead.length());
    const float force = (sole ? f.soleMinimum + f.solePower * std::pow(bar, std::max(0.05f, f.soleCurve))
        : f.touchMinimum + f.touchPower * std::pow(bar, std::max(0.05f, f.touchCurve)))
        + std::max(0.0f, f.speedPush) * speed;
    const float along = dot(body, direction);
    const float carry = std::max(0.0f, std::max(along, dot(ahead, direction))) * std::max(0.0f, f.carry);
    // Contra o rumo do toque: o quique do que a bola trazia contra ele (o
    // KeepBounce do modelo aprovado, 0,3) e o que o corpo indo para o outro
    // lado empresta - os dois por reverseBoost (o padrao 0,3 e o quique de
    // antes: parado, o toque e o mesmo).
    const Vec3 ball = planar(ballVelocity);
    const float ballAlong = dot(ball, direction);
    const float reverse = std::max(0.0f, f.reverseBoost) * (std::max(0.0f, -along) + std::max(0.0f, -ballAlong));
    const float defined = carry + reverse + force;
    // O que fica da que a bola trazia (keepBallMotion), mas de lado so o que
    // ela anda em relacao ao corpo: a bola que ia junto com ele vai limpa no
    // rumo do toque (de lado correndo, ela guardava 40% da corrida e saia na
    // diagonal, para longe do caminho dele). Parado, igual.
    const Vec3 relative = ball - body;
    const Vec3 side = relative - direction * dot(relative, direction);
    const float extra = f.touchAheadKeep * std::max(0.0f, ballAlong - defined);
    return direction * (defined + extra) + side * f.touchSideKeep;
}

Quaternion slerpQuat(Quaternion from, Quaternion to, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float d = from.w * to.w + from.x * to.x + from.y * to.y + from.z * to.z;
    if (d < 0.0f) {
        to = Quaternion { -to.x, -to.y, -to.z, -to.w };
        d = -d;
    }
    if (d > 0.9995f) {
        return Quaternion { from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t,
            from.z + (to.z - from.z) * t, from.w + (to.w - from.w) * t }.normalized();
    }
    const float angle = std::acos(std::clamp(d, -1.0f, 1.0f));
    const float a = std::sin((1.0f - t) * angle) / std::sin(angle);
    const float b = std::sin(t * angle) / std::sin(angle);
    return Quaternion { from.x * a + to.x * b, from.y * a + to.y * b,
        from.z * a + to.z * b, from.w * a + to.w * b }.normalized();
}

Vec3 nlerpDirection(Vec3 a, Vec3 b, float t) {
    const Vec3 v = a + (b - a) * std::clamp(t, 0.0f, 1.0f);
    return v.lengthSquared() > 1e-8f ? v.normalized() : b;
}

Vec3 orthogonal(Vec3 v, Vec3 axis) {
    const Vec3 out = v - axis * dot(v, axis);
    return out.lengthSquared() > 1e-10f ? out.normalized() : Vec3 {};
}

// Quaternion de uma base ortonormal (colunas x, y, z).
Quaternion fromBasis(Vec3 x, Vec3 y, Vec3 z) {
    const float m00 = x.x, m01 = y.x, m02 = z.x;
    const float m10 = x.y, m11 = y.y, m12 = z.y;
    const float m20 = x.z, m21 = y.z, m22 = z.z;
    const float trace = m00 + m11 + m22;
    Quaternion q;
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

Quaternion yawPitch(Vec3 heading, float pitchDown) {
    const float yaw = std::atan2(heading.y, heading.x);
    return (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw)
        * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, pitchDown)).normalized();
}

// Curva C1 pelos valores nas chaves do corpo (0 no comeco e no fim):
// Catmull-Rom em tempos dados.
float bodyCurve(const float* times, const float* values, int count, float t) {
    if (t <= times[0]) return values[0];
    if (t >= times[count - 1]) return values[count - 1];
    int k = 0;
    while (k + 1 < count - 1 && t > times[k + 1]) ++k;
    const float h = std::max(1e-4f, times[k + 1] - times[k]);
    const float u = (t - times[k]) / h;
    const auto tangent = [&](int i) {
        if (i <= 0 || i >= count - 1) return 0.0f;
        return (values[i + 1] - values[i - 1]) / std::max(1e-4f, times[i + 1] - times[i - 1]);
    };
    return hermite(values[k], values[k + 1], tangent(k) * h, tangent(k + 1) * h, u);
}

// x com um piso suave: igual acima de floor + width e chegando no piso sem
// quina (nunca abaixo dele).
float softFloor(float x, float floor, float width) {
    if (width <= 1e-4f) return std::max(x, floor);
    if (x >= floor + width) return x;
    return floor + width * std::exp((x - floor - width) / width);
}

// Os tempos de referencia do gesto (s desde o comeco: armado, contato,
// acompanhamento e fim) pela familia e pela forca.
struct GestureTimes {
    float windup = 0.1f, contact = 0.2f, follow = 0.35f, end = 0.6f;
};

GestureTimes referenceTimes(bool kickLike, bool soleOrHeel, bool behind, float p, float q) {
    GestureTimes g;
    if (kickLike) {
        g.windup = KickWindupTime + KickWindupTimePower * p;
        g.contact = g.windup + KickStrikeTime + KickStrikeTimePower * p;
        g.follow = g.contact + KickFollowTime + KickFollowTimePower * p;
        g.end = g.follow + KickRecoverTime + KickRecoverTimePower * p;
    } else if (behind) {
        // A bola atras tocada para o lado (V3).
        g.windup = 0.12f + 0.06f * q;
        g.contact = g.windup + 0.07f + 0.03f * q;
        g.follow = g.contact + 0.09f + 0.06f * q;
        g.end = g.follow + 0.20f + 0.06f * q;
    } else if (soleOrHeel) {
        g.windup = 0.11f;
        g.contact = 0.17f;
        g.follow = 0.17f + 0.10f + 0.06f * p;
        g.end = g.follow + 0.18f;
    } else {
        // (O chute de lado de dentro/fora tem o tempo de um passe forte.)
        g.windup = TouchWindupTime + TouchWindupTimePower * q;
        g.contact = g.windup + TouchStrikeTime + TouchStrikeTimePower * q;
        g.follow = g.contact + TouchFollowTime + TouchFollowTimePower * q;
        g.end = g.follow + TouchRecoverTime + TouchRecoverTimePower * q;
    }
    return g;
}

// O giro entre duas orientacoes (rad).
float turnBetween(Quaternion a, Quaternion b) {
    const Quaternion turn = (b * a.conjugate()).normalized();
    return 2.0f * std::acos(std::clamp(std::abs(turn.w), 0.0f, 1.0f));
}

} // namespace

BallForceSettings3D defaultBallForceSettings3D() {
    return defaultForce();
}

void BallControl3D::reset() {
    m_phase = BallControlPhase3D::Idle;
    m_kind = BallActionKind3D::None;
    m_gesture = {};
    m_waiting = false;
    m_pending = false;
    m_requestCommitted = false;
    m_action = false;
    m_forced = false;
    m_actionCommitted = false;
    m_bareForce = false;
    m_sinceInArea = 10.0f;
    m_virtualBall = false;
    m_imagined = false;
    m_stability = 0.0f;
    m_carry = {};
    m_carryTime = 10.0f;
    m_previousFootValid = false;
}

void BallControl3D::resolve(const RagdollProfile3D& profile) {
    const char* feet[2] = { "LeftFoot", "RightFoot" };
    const char* thighs[2] = { "LeftThigh", "RightThigh" };
    const char* shins[2] = { "LeftShin", "RightShin" };
    const auto findLink = [&](const char* id) -> std::size_t {
        for (std::size_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        return 0;
    };
    const auto anchorIn = [&](std::size_t link) {
        const auto& def = profile.links[link];
        return def.modelOrientation.conjugate().rotate(def.inboundJoint.anchorModelPosition - def.modelPosition);
    };
    for (std::size_t side = 0; side < 2; ++side) {
        FootGeometry geometry;
        geometry.link = findLink(feet[side]);
        geometry.hip = findLink(thighs[side]);
        const std::size_t shin = findLink(shins[side]);
        if (geometry.link != 0 && geometry.hip != 0 && shin != 0) {
            const auto& collider = profile.links[geometry.link].collider;
            geometry.center = collider.localPosition;
            geometry.half = collider.boxHalfExtents;
            geometry.boxOrientation = collider.localOrientation;
            geometry.valid = geometry.half.x > 0.0f;
            geometry.hipAnchor = anchorIn(geometry.hip);
            geometry.ankleAnchor = anchorIn(geometry.link);
            const Vec3 hip = profile.links[geometry.hip].inboundJoint.anchorModelPosition;
            const Vec3 knee = profile.links[shin].inboundJoint.anchorModelPosition;
            const Vec3 ankle = profile.links[geometry.link].inboundJoint.anchorModelPosition;
            geometry.thigh = (knee - hip).length();
            geometry.shin = (ankle - knee).length();
        }
        m_feet[side] = geometry;
    }
    m_legs = resolveBallGestureLegs3D(profile);
    m_zone.radius = ReachRadius;
    m_zone.waistAbovePelvis = WaistAbovePelvis;
    m_resolved = true;
}

Vec3 BallControl3D::hipJoint(const RagdollState3D& state, int side) const {
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    const auto& link = state.links[f.hip];
    return link.position + link.orientation.rotate(f.hipAnchor);
}

Vec3 BallControl3D::ankleJoint(const RagdollState3D& state, int side) const {
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    const auto& link = state.links[f.link];
    return link.position + link.orientation.rotate(f.ankleAnchor);
}

bool BallControl3D::footBox(const RagdollState3D& state, int side, Vec3& center, Quaternion& orientation,
    Vec3& half) const {
    if (!m_resolved || side < 0 || side > 1) return false;
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    if (!f.valid || f.link >= state.links.size()) return false;
    const auto& body = state.links[f.link];
    orientation = (body.orientation * f.boxOrientation).normalized();
    center = body.position + body.orientation.rotate(f.center);
    half = f.half;
    return true;
}

bool BallControl3D::inArea(Vec3 ball, float radius, Vec3 pelvis) const {
    // A base da bola abaixo da cintura (a pelve, um pouco acima) e no raio.
    if (ball.z - radius > pelvis.z + WaistAbovePelvis) return false;
    return planar(ball - pelvis).length() <= ReachRadius;
}

void BallControl3D::ballNow(const BallControlInput3D& input, Vec3& position, Vec3& velocity) const {
    if (m_virtualBall) {
        // (A forca ja saiu, o gesto ainda nao comecou: a bola de antes dela.)
        velocity = m_virtualVelocity;
        position = m_virtualPosition + planar(m_virtualVelocity) * m_virtualClock;
        return;
    }
    position = input.ballPosition;
    velocity = input.ballVelocity;
}

Vec3 BallControl3D::predictBall(const BallControlInput3D& input, float seconds) const {
    Vec3 position, velocity;
    ballNow(input, position, velocity);
    return predictBallFrom(input, position, velocity, seconds);
}

Vec3 BallControl3D::predictBallFrom(const BallControlInput3D& input, Vec3 position, Vec3 velocity, float seconds) {
    const float t = std::max(0.0f, seconds);
    const Vec3 flat = planar(velocity);
    const float speed = flat.length();
    const bool airborne = position.z > input.groundHeight + input.ballRadius + 0.02f || velocity.z > 0.1f;
    // Rolando, freia; voando, a gravidade (o quique nao entra).
    float travel = speed * t;
    if (!airborne && speed > 1e-4f) {
        const float rolling = std::min(t, speed / std::max(1e-3f, RollDeceleration));
        travel = speed * rolling - 0.5f * RollDeceleration * rolling * rolling;
    }
    Vec3 at = position + (speed > 1e-4f ? flat * (travel / speed) : Vec3 {});
    if (airborne) {
        at.z = position.z + velocity.z * t - 4.905f * t * t;
        at.z = std::max(at.z, input.groundHeight + input.ballRadius);
    }
    return at;
}

Vec3 BallControl3D::predictPelvis(const BallControlInput3D& input, float seconds) const {
    const Vec3 pelvis = input.state->links.front().position;
    if (input.standing) return pelvis;
    const float t = std::max(0.0f, seconds);
    const Vec3 v0 = bodyVelocityOf(input);
    const Vec3 v1 = planar(input.requestedVelocity);
    const Vec3 change = v1 - v0;
    const float amount = change.length();
    if (amount < 1e-3f) return pelvis + v0 * t;
    // A velocidade vai a pedida (a capsula acelera) e segue nela. (No fisico
    // o corpo atrasa e as vezes mal anda: acelera bem menos.)
    const float acceleration = std::max(0.5f, input.animatedBody ? BodyAcceleration : PhysicalBodyAcceleration);
    const float reach = amount / acceleration;
    const Vec3 a = change * (acceleration / amount);
    if (t <= reach) return pelvis + v0 * t + a * (0.5f * t * t);
    return pelvis + v0 * reach + a * (0.5f * reach * reach) + v1 * (t - reach);
}

Vec3 BallControl3D::predictBodyVelocity(const BallControlInput3D& input, float seconds) const {
    const Vec3 v0 = bodyVelocityOf(input);
    if (input.standing) return v0;
    const Vec3 v1 = planar(input.requestedVelocity);
    const Vec3 change = v1 - v0;
    const float amount = change.length();
    if (amount < 1e-3f) return v1;
    // (A mesma aceleracao da previsao da pelve: a capsula, ou o corpo fisico
    // que atrasa.)
    const float acceleration = std::max(0.5f, input.animatedBody ? BodyAcceleration : PhysicalBodyAcceleration);
    return v0 + change * std::min(1.0f, acceleration * std::max(0.0f, seconds) / amount);
}

Vec3 BallControl3D::carryVelocity(const BallControlInput3D& input) const {
    // (So no corpo animado: o fisico parte devagar demais para a previsao - o
    // toque saia a 2 m/s com ele mal andando e a bola escapava.)
    return input.animatedBody ? predictBodyVelocity(input, m_force.carryAhead) : bodyVelocityOf(input);
}

float BallControl3D::bodyHeightShare(const BallControlInput3D& input) {
    const float standing = input.profile != nullptr ? input.profile->standingRootHeightMeters : 0.0f;
    if (standing <= 0.1f) return 1.0f;
    return (input.state->links.front().position.z - input.groundHeight) / standing;
}

Vec3 BallControl3D::bodyVelocityOf(const BallControlInput3D& input) {
    return planar(input.bodyVelocityValid ? input.bodyVelocity : input.state->links.front().linearVelocity);
}

void BallControl3D::bodyFrame(const BallControlInput3D& input, Vec3& forward, Vec3& left) const {
    const float yaw = input.bodyYawValid ? input.bodyYawRadians : input.facingYawRadians;
    forward = { std::cos(yaw), std::sin(yaw), 0.0f };
    left = { -forward.y, forward.x, 0.0f };
}

Vec3 BallControl3D::requestPush(Vec3 bodyForward) const {
    Vec3 push = m_kind == BallActionKind3D::Shot ? Vec3 { std::cos(m_lookYaw), std::sin(m_lookYaw), 0.0f }
        : planar(m_direction);
    return push.lengthSquared() > 1e-6f ? push.normalized() : bodyForward;
}

float BallControl3D::footGroundDeficit(int side, Vec3 position, Quaternion orientation, float groundHeight,
    float clearance) const {
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    const Quaternion box = (orientation * f.boxOrientation).normalized();
    const Vec3 center = position + orientation.rotate(f.center);
    const Vec3 axes[3] = { box.rotate({ 1.0f, 0.0f, 0.0f }), box.rotate({ 0.0f, 1.0f, 0.0f }),
        box.rotate({ 0.0f, 0.0f, 1.0f }) };
    const float low = center.z - std::abs(axes[0].z) * f.half.x - std::abs(axes[1].z) * f.half.y
        - std::abs(axes[2].z) * f.half.z;
    return std::max(0.0f, groundHeight + clearance - low);
}

float BallControl3D::boxGap(int side, Vec3 footPosition, Quaternion footOrientation, Vec3 ball,
    float radius, float margin) const {
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    const Quaternion box = (footOrientation * f.boxOrientation).normalized();
    const Vec3 center = footPosition + footOrientation.rotate(f.center);
    const Vec3 half = f.half + Vec3 { margin, margin, margin };
    const Vec3 local = box.conjugate().rotate(ball - center);
    const Vec3 clamped { std::clamp(local.x, -half.x, half.x), std::clamp(local.y, -half.y, half.y),
        std::clamp(local.z, -half.z, half.z) };
    return (local - clamped).length() - radius;
}

Vec3 BallControl3D::keepOwnSide(int side, Vec3 ankle, Vec3 pelvis, Vec3 bodyForward, Vec3 supportAnkle,
    float supportWeight, bool soft, float sideGap) const {
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const float out = side == 0 ? 1.0f : -1.0f;
    // Para o lado desta perna.
    const Vec3 outward = cross(up, bodyForward) * out;
    const float width = soft ? OwnSideSoftness : 0.0f;
    // Do lado dela da linha do meio do corpo.
    const float fromMiddle = dot(planar(ankle - pelvis), outward);
    float shift = softFloor(fromMiddle, OwnSideMargin, width) - fromMiddle;
    // Lado a lado com o tornozelo de apoio: longe dele de lado (pelo peso).
    if (supportWeight > 0.0f) {
        const Vec3 rel = planar(ankle - supportAnkle);
        const float along = std::abs(dot(rel, bodyForward));
        const float overlap = 1.0f - smooth((along - 0.22f) / 0.16f);
        if (overlap > 0.0f) {
            const float lateral = dot(rel, outward);
            const float gap = sideGap > 0.0f ? sideGap : FootSideGap;
            shift = std::max(shift, (softFloor(lateral, gap, width) - lateral) * overlap * std::min(1.0f, supportWeight));
        }
    }
    return shift > 0.0f ? ankle + outward * shift : ankle;
}

void BallControl3D::surfaceLocal(int side, BallTouchSurface3D surface, Vec3& point, Vec3& normal) const {
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    // Lado de dentro do pe: o esquerdo para -y (a direita do corpo), o
    // direito para +y.
    const float medial = side == 0 ? -1.0f : 1.0f;
    Vec3 local;
    switch (surface) {
    case BallTouchSurface3D::Laces:
        // O peito do pe: em cima, um pouco a frente do meio.
        local = { f.half.x * 0.25f, 0.0f, f.half.z };
        normal = { 0.0f, 0.0f, 1.0f };
        break;
    case BallTouchSurface3D::Inside:
        // O arco: do lado de dentro, no meio do pe.
        local = { 0.0f, medial * f.half.y, -f.half.z * 0.15f };
        normal = { 0.0f, medial, 0.0f };
        break;
    case BallTouchSurface3D::Outside:
        // Do lado de fora, mais para a frente (o quinto metatarso).
        local = { f.half.x * 0.35f, -medial * f.half.y, -f.half.z * 0.15f };
        normal = { 0.0f, -medial, 0.0f };
        break;
    case BallTouchSurface3D::Sole:
        // A sola sob a planta do pe.
        local = { f.half.x * 0.35f, 0.0f, -f.half.z };
        normal = { 0.0f, 0.0f, -1.0f };
        break;
    case BallTouchSurface3D::Heel:
        // O calcanhar: atras, na altura da sola.
        local = { -f.half.x, 0.0f, -f.half.z * 0.35f };
        normal = { -1.0f, 0.0f, 0.0f };
        break;
    }
    point = f.center + f.boxOrientation.rotate(local);
    normal = f.boxOrientation.rotate(normal);
}

BallTouchSurface3D BallControl3D::chooseSurface(int side, Vec3 push, Vec3 bodyForward, BallActionKind3D kind,
    bool moving, bool ballBehind) const {
    // O angulo do empurrao no corpo, + para fora da perna que joga.
    const float out = side == 0 ? 1.0f : -1.0f;
    const float angle = signedAngle(bodyForward, push) * out;
    // A bola atras do corpo: para tras, de calcanhar; para a frente, a sola
    // por cima dela, trazendo-a.
    if (ballBehind) return std::abs(angle) > 2.0f ? BallTouchSurface3D::Heel : BallTouchSurface3D::Sole;
    if (kind == BallActionKind3D::Shot) {
        // Com efeito: curva para o lado de dentro do pe, a parte de dentro
        // (o direito curvando para a esquerda); para o de fora, a trivela.
        if (std::abs(m_curve) >= 0.3f) {
            const bool towardLeft = m_curve < 0.0f;
            return towardLeft == (side == 1) ? BallTouchSurface3D::Inside : BallTouchSurface3D::Outside;
        }
        if (angle > 1.0f) return BallTouchSurface3D::Outside;
        if (angle < -1.0f) return BallTouchSurface3D::Inside;
        return BallTouchSurface3D::Laces;
    }
    const float back = moving ? 1.95f : 2.05f;
    if (std::abs(angle) > back) return BallTouchSurface3D::Sole;
    if (moving && std::abs(angle) <= 0.42f) return BallTouchSurface3D::Laces;
    if (angle > 0.5f) return BallTouchSurface3D::Outside;
    return BallTouchSurface3D::Inside;
}

bool BallControl3D::styleFor(int side, Vec3 ball, Vec3 pelvis, Vec3 forward, Vec3 push, bool moving, Style& style,
    BallTouchSurface3D& surface, bool& virtualFront) const {
    const float out = side == 0 ? 1.0f : -1.0f;
    const bool shot = m_kind == BallActionKind3D::Shot;
    const float along = dot(planar(ball - pelvis), forward);
    const float angle = signedAngle(forward, push) * out;
    style = Style::Front;
    virtualFront = false;
    bool behind = false;
    if (along < -0.06f) {
        // Atras: para tras, o calcanhar (aprovado); para fora desta perna, ela
        // vai para tras e para fora; para dentro, a sola curta; para a frente,
        // um pouco atras e o gesto de frente (a bola imaginada a frente), bem
        // atras a sola trazendo-a.
        if (std::abs(angle) > 2.2f) behind = true;
        else if (angle >= 0.9f) style = Style::BehindOut;
        else if (angle <= -0.9f) style = Style::BehindIn;
        else if (along >= -BehindNear) virtualFront = true;
        else behind = true;
    }
    if (shot && style != Style::Front) return false;
    surface = chooseSurface(side, push, forward, m_kind, moving, behind);
    if (style == Style::BehindOut) surface = BallTouchSurface3D::Outside;
    if (style == Style::BehindIn) surface = BallTouchSurface3D::Sole;
    // O chute com a bola atras so de calcanhar (mira para tras).
    if (behind && shot && surface != BallTouchSurface3D::Heel) return false;
    return true;
}

void BallControl3D::contactPose(int side, BallTouchSurface3D surface, Style style, Vec3 ball, float radius,
    Vec3 push, Vec3 bodyForward, float power, float groundHeight, Vec3& footPosition,
    Quaternion& footOrientation) const {
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const float out = side == 0 ? 1.0f : -1.0f;
    push = planar(push).normalized();
    bodyForward = planar(bodyForward).normalized();
    // O rumo da ponta do pe pela superficie, nos limites do quadril (em
    // relacao ao rumo do corpo: + para fora).
    const auto turned = [&](Vec3 toe, float outward, float inward) {
        const float angle = std::clamp(signedAngle(bodyForward, toe) * out, -inward, outward);
        return rotateZ(bodyForward, angle * out);
    };
    Vec3 toe = push;
    float pitch = 0.0f;
    switch (surface) {
    case BallTouchSurface3D::Laces:
        toe = push;
        // (Bola alta: o pe mais deitado, por baixo dela.)
        pitch = LacesPitch + LacesPitchPower * power - 0.25f * std::max(0.0f, m_lift);
        break;
    case BallTouchSurface3D::Inside:
        // O lado de dentro encara o empurrao: a ponta virada para fora.
        toe = turned(rotateZ(push, out * Pi * 0.5f), InsideTurnMaximum, 0.6f);
        pitch = -0.06f;
        break;
    case BallTouchSurface3D::Outside:
        // O de fora encara o empurrao: a ponta virada para dentro e um pouco
        // para baixo.
        toe = turned(rotateZ(push, -out * Pi * 0.5f), 0.35f, OutsideTurnMaximum);
        pitch = 0.28f;
        break;
    case BallTouchSurface3D::Sole:
        toe = bodyForward;
        pitch = -0.22f;
        break;
    case BallTouchSurface3D::Heel:
        // A ponta para a frente e um pouco para baixo; o calcanhar empurra
        // para tras.
        toe = bodyForward;
        pitch = 0.15f;
        break;
    }
    if (style == Style::BehindOut) {
        // Atras, para fora (Pro Soccer Online): a perna para tras, o pe de
        // ponta para baixo (a sola para tras) e o lado de fora dele encarando
        // o empurrao.
        toe = turned(rotateZ(push, -out * Pi * 0.5f), 0.45f, 0.30f);
        pitch = 0.85f;
    } else if (style == Style::BehindIn) {
        // Atras, para dentro: a sola por cima dela, o pe quase deitado.
        toe = bodyForward;
        pitch = 0.05f;
    }
    footOrientation = yawPitch(toe, pitch);
    Vec3 point, normal;
    surfaceLocal(side, surface, point, normal);
    const Vec3 pointWorld = footOrientation.rotate(point);
    Vec3 contact;
    if (surface == BallTouchSurface3D::Sole) {
        // A sola em cima da bola, um pouco para o lado contrario ao puxar.
        contact = ball + (up * 0.94f - push * 0.34f) * radius;
    } else if (surface == BallTouchSurface3D::Laces) {
        // O peito do pe atras da bola, na altura do centro dela (na bola alta,
        // um pouco abaixo: o pe passa por baixo).
        contact = ball - push * radius - up * (0.04f * std::max(0.0f, m_lift));
    } else {
        // A face do pe encostada na bola, no ponto mais perto do centro.
        Vec3 n = planar(footOrientation.rotate(normal));
        n = n.lengthSquared() > 1e-6f ? n.normalized() : push;
        contact = ball - n * radius;
    }
    footPosition = contact - pointWorld;
    // A caixa do pe nunca dentro do chao: o pe sobe (o contato sobe na bola).
    footPosition.z += footGroundDeficit(side, footPosition, footOrientation, groundHeight, 0.008f);
}

BallControl3D::LegKey BallControl3D::legKeyFrom(Vec3 v) const {
    LegKey key;
    key.length = std::max(0.05f, v.length());
    const float along = dot(v, m_frameSwing), side = dot(v, m_frameOut);
    key.lateral = std::asin(std::clamp(side / key.length, -1.0f, 1.0f));
    key.swing = std::atan2(along, -v.z);
    return key;
}

Vec3 BallControl3D::legKeyVector(const LegKey& key) const {
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    return (m_frameSwing * (std::cos(key.lateral) * std::sin(key.swing))
        - up * (std::cos(key.lateral) * std::cos(key.swing)) + m_frameOut * std::sin(key.lateral)) * key.length;
}

BallControl3D::LegKey BallControl3D::legKeyAngles(int side, float thigh, float knee, float lateral) const {
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    const float a = f.thigh, b = f.shin;
    LegKey key;
    key.length = std::sqrt(a * a + b * b + 2.0f * a * b * std::cos(knee));
    key.swing = thigh - std::atan2(b * std::sin(knee), a + b * std::cos(knee));
    key.lateral = lateral;
    return key;
}

Quaternion BallControl3D::footAlongShin(int side, const LegKey& key, float pitch) const {
    // A mesma conta do IK analitico (BallGesturePose3D): a coxa e a canela
    // pelo tornozelo, o joelho apontando para key.knee; o pe neutro aponta
    // para a frente da canela e o tornozelo o gira na dobradica (+ ponta para
    // baixo).
    const FootGeometry& f = m_feet[static_cast<std::size_t>(side)];
    const float a = f.thigh, b = f.shin;
    const Vec3 v = legKeyVector(key);
    const float D = std::clamp(v.length(), std::abs(a - b) + 0.03f, (a + b) * 0.9995f);
    const Vec3 u = v.normalized();
    const float cosHip = std::clamp((a * a + D * D - b * b) / (2.0f * a * D), -1.0f, 1.0f);
    const float sinHip = std::sqrt(std::max(0.0f, 1.0f - cosHip * cosHip));
    Vec3 pole = orthogonal(key.knee, u);
    if (pole.lengthSquared() < 0.5f) pole = orthogonal(m_frameSwing, u);
    const Vec3 thighDirection = u * cosHip + pole * sinHip;
    const Vec3 shinDirection = (u * D - thighDirection * a).normalized();
    const Vec3 kneeForward = orthogonal(pole, thighDirection);
    const Vec3 hinge = cross(thighDirection, kneeForward);
    const Vec3 shinForward = orthogonal(cross(hinge, shinDirection), shinDirection);
    const Vec3 toe = shinForward * std::cos(pitch) + shinDirection * std::sin(pitch);
    const Vec3 footUp = shinForward * std::sin(pitch) - shinDirection * std::cos(pitch);
    return fromBasis(toe, cross(footUp, toe), footUp);
}

float BallControl3D::movingContact(const BallControlInput3D& input, int side, bool kick, float& waitLift,
    float* score) const {
    // A janela do contato deste pe (s a partir de agora): no balanco, do
    // ponto em que ele passa a frente do quadril (~62% dele) ate o pouso;
    // apoiado, o mesmo no proximo balanco (sai depois do resto do apoio). O
    // chute bate logo depois do outro pe pousar (no apoio dele). Dentro dela,
    // o instante com a bola dentro da area e a uma distancia boa a frente do
    // quadril (cedo e melhor), com o pe do lado dele (V3: as pernas nunca
    // cruzam - medido no corpo, nao no rumo do movimento).
    const RagdollState3D& state = *input.state;
    const std::size_t s = static_cast<std::size_t>(side);
    const std::size_t o = static_cast<std::size_t>(1 - side);
    Vec3 bodyForward, bodyLeft;
    bodyFrame(input, bodyForward, bodyLeft);
    const Vec3 bodyVelocity = bodyVelocityOf(input);
    // A frente e o rumo em que o corpo anda (de lado ou de costas, a bola
    // "a frente" e a do movimento).
    const Vec3 forward = bodyVelocity.length() > 1.0f ? bodyVelocity.normalized() : bodyForward;
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const float out = side == 0 ? 1.0f : -1.0f;
    const Vec3 pelvisNow = state.links.front().position;
    const Vec3 hip = hipJoint(state, side);
    // O empurrao em relacao a esta perna (+ para fora dela): o pe fica do
    // lado contrario da bola.
    const float pushOut = dot(requestPush(bodyForward), bodyLeft) * out;
    const bool pulling = !kick && std::abs(signedAngle(bodyForward, requestPush(bodyForward))) > 1.95f;
    const bool swinging = !input.footPlanted[s] && input.swingProgress[s] >= 0.0f;
    float lift = 0.0f, swing = m_swingSeconds[s], progress = 0.0f;
    if (swinging) {
        const float touchdown = input.secondsToTouchdown[s] > 0.0f ? input.secondsToTouchdown[s] : m_swingSeconds[s];
        progress = std::clamp(input.swingProgress[s], 0.0f, 0.95f);
        // (Logo na saida do chao a passada ainda pode dizer o fim do anterior.)
        if (progress > 0.85f && touchdown > 0.2f) progress = 0.0f;
        swing = touchdown / std::max(0.05f, 1.0f - progress);
        lift = -progress * swing;
    } else {
        lift = std::clamp(m_stanceSeconds[s] - m_footClock[s], 0.0f, 0.3f);
    }
    waitLift = std::max(0.0f, lift);
    // A janela: o toque do ponto em que o pe passa a frente do quadril no
    // balanco natural (~62%) ate o pouso, podendo apressar o balanco (o pe
    // chega na metade do tempo; custa um pouco). O chute: no apoio do outro
    // pe (o certo) ou, sem tempo para isso, rapido (custa mais).
    const float natural = lift + 0.62f * swing;
    // (No fisico o pe precisa de tempo para chegar: nada antes de 0,14 s.)
    // A bola imaginada (a garantia: so o visual) vai no tempo natural da
    // passada - no fim do balanco a batida durava 0,03 s e a perna saltava.
    const bool imaginedBall = m_virtualBall && m_imagined;
    // (O toque correndo, no minimo 0,1 s: com 0,05 o armado durava 0,03 s e,
    // virando junto, a canela saltava 36 graus num quadro.)
    const float soonest = std::max(input.animatedBody ? (m_hitbox ? RunningSoonestHitbox : RunningSoonest) : 0.14f,
        imaginedBall ? 0.15f : 0.0f);
    const float from = std::max(soonest, kick ? lift + 0.12f : lift + 0.30f * swing);
    const float to = std::max(from, lift + swing + 0.01f);
    float plant = -1.0f;
    if (kick) {
        if (!input.footPlanted[o] && input.swingProgress[o] >= 0.0f) {
            plant = input.secondsToTouchdown[o] > 0.0f ? input.secondsToTouchdown[o] : 0.1f;
            if (plant < lift) plant += m_swingSeconds[o] + m_stanceSeconds[o];
        } else {
            plant = std::max(0.0f, lift) + 0.12f;
        }
    }
    float best = 1e9f, bestAt = -1.0f;
    const float ideal = (kick ? 0.30f : 0.40f) - (input.animatedBody ? 0.0f : 0.08f);
    // O toque nao alcanca longe num instante: o tornozelo indo de onde esta
    // ate a bola (em volta do quadril) mais rapido que a passada leva custa
    // (0,6 m em 0,03 s - a bola correndo junto na borda da frente - saltava a
    // canela 35 graus num quadro).
    const Vec3 ankleFromHip = planar(ankleJoint(state, side) - hip);
    const float paceLimit = 4.0f + 0.8f * bodyVelocity.length();
    for (float at = from; at <= to + 1e-4f; at += 0.01f) {
        const Vec3 pelvisAt = predictPelvis(input, at);
        const Vec3 ballAt = predictBall(input, at);
        const Vec3 rel = planar(ballAt - hip - (pelvisAt - pelvisNow));
        const float along = dot(rel, forward);
        const float lateral = dot(rel, left) * out;
        float cost = std::abs(along - ideal) + 0.25f * at;
        // Longe demais a frente nao da; atras do quadril, so um pouco (V3: o
        // gesto de frente com a bola imaginada logo a frente - o pedido tarde,
        // com a bola ja na area, ainda sai), e custa; muito para o lado custa.
        if (along > 0.85f) cost += 10.0f;
        else if (along < 0.0f) cost += along >= -BehindNear ? 0.8f + 2.0f * -along : 10.0f;
        cost += 0.6f * std::max(0.0f, std::abs(lateral) - 0.45f);
        // V3: a bola dentro da area no contato.
        if (!inArea(ballAt, input.ballRadius, Vec3 { pelvisAt.x, pelvisAt.y, pelvisNow.z })) cost += 10.0f;
        // V3: as pernas nao cruzam. O pe no contato (do lado contrario ao
        // empurrao) do lado dele do corpo; tirado da bola ate o limite, alem
        // disso a perna nao serve. A bola do lado da outra perna custa.
        const float ballSide = dot(planar(ballAt - pelvisAt), bodyLeft) * out;
        // (Puxando - a sola, o empurrao para tras -, o pe vai por cima da bola,
        // nao do lado dela: a perna de la cruzava na diagonal para tras.)
        const float footSide = pulling ? ballSide : ballSide - pushOut * (input.ballRadius + 0.06f);
        const float shift = std::max(0.0f, OwnSideMargin - footSide);
        cost += shift > VirtualShiftMaximum ? 10.0f : 2.0f * shift;
        cost += 0.6f * std::max(0.0f, -ballSide);
        if (!kick) {
            const float travel = std::max(0.0f, (rel - ankleFromHip).length() - 0.15f);
            cost += 0.1f * std::max(0.0f, travel / std::max(0.03f, at) - paceLimit);
        }
        if (kick) {
            // Fora do apoio do outro pe (o chute rapido) e o balanco cedo demais.
            const bool supported = at >= plant + 0.04f && at <= plant + m_stanceSeconds[o] - 0.01f;
            if (!supported) cost += 0.35f;
            if (swinging && progress > 0.45f) cost += 0.3f + (progress - 0.45f) * 1.5f;
        } else {
            cost += 0.6f * (imaginedBall ? std::abs(natural - at) : std::max(0.0f, natural - at));
        }
        if (cost < best) { best = cost; bestAt = at; }
    }
    if (score != nullptr) *score = best;
    if (std::getenv("XBALLPLAN"))
        std::printf("OPCAO pe %d chute %d balanco %d prog %.2f (passada %.2f, pousa %.2f, relogio %.2f) sai %.2f janela %.2f..%.2f melhor %.2f custo %.2f\n", side,
            kick ? 1 : 0, swinging ? 1 : 0, progress, input.swingProgress[s], input.secondsToTouchdown[s], m_footClock[s], lift, from, to, bestAt, best);
    return best < 10.0f ? bestAt : -1.0f;
}

float BallControl3D::naturalContact(const BallControlInput3D& input, int side) const {
    Vec3 forward, left;
    bodyFrame(input, forward, left);
    Vec3 position, velocity;
    ballNow(input, position, velocity);
    Style style = Style::Front;
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    bool virtualFront = false;
    if (!styleFor(side, position, input.state->links.front().position, forward, requestPush(forward), false, style,
            surface, virtualFront))
        style = Style::Front;
    const bool shot = m_kind == BallActionKind3D::Shot;
    const float p = std::clamp(m_power, 0.0f, 1.0f);
    const bool kickLike = shot && surface != BallTouchSurface3D::Heel && surface != BallTouchSurface3D::Sole;
    return referenceTimes(kickLike, surface == BallTouchSurface3D::Sole || surface == BallTouchSurface3D::Heel,
        style != Style::Front, p, shot ? std::max(p, 0.7f) : p).contact;
}

float BallControl3D::standingMeeting(const BallControlInput3D& input, int side, float natural, bool& early,
    float* later) const {
    // Parado: quando o gesto deste pe encontra a bola. Parada, no tempo dele
    // (com ela na area); andando (vindo, cruzando, indo), o instante com ela
    // a uma distancia boa a frente do quadril, dentro da area - cedo demais
    // para o gesto (alem do quanto ele aguenta esperar), o pedido fica
    // guardado.
    early = false;
    const RagdollState3D& state = *input.state;
    const Vec3 pelvis = state.links.front().position;
    Vec3 position, velocity;
    ballNow(input, position, velocity);
    const bool airborne = position.z > input.groundHeight + input.ballRadius + 0.02f || velocity.z > 0.1f;
    if (planar(velocity).length() < 0.3f && !airborne)
        return inArea(position, input.ballRadius, pelvis) ? natural : -1.0f;
    Vec3 forward, left;
    bodyFrame(input, forward, left);
    const float out = side == 0 ? 1.0f : -1.0f;
    const Vec3 hip = hipJoint(state, side);
    float best = 1e9f, bestAt = -1.0f;
    for (float at = natural * AnticipateCompress; at <= MeetHorizon + 1e-4f; at += 0.01f) {
        const Vec3 ball = predictBall(input, at);
        if (!inArea(ball, input.ballRadius, pelvis)) continue;
        const Vec3 rel = planar(ball - hip);
        const float along = dot(rel, forward), lateral = dot(rel, left) * out;
        float cost = std::abs(along - 0.30f) + 0.4f * std::max(0.0f, std::abs(lateral) - 0.45f)
            + 0.15f * std::abs(at - natural) / std::max(0.05f, natural);
        // (A bola ja passando do quadril: o gesto de frente nao a pega bem.)
        if (along < -0.05f) cost += 0.6f;
        if (cost < best) { best = cost; bestAt = at; }
    }
    // (A bola na area agora mas saindo antes do gesto encontra-la: o gesto
    // sai no tempo dele e a forca vem antes de ela sair - beginGesture.)
    if (bestAt < 0.0f) return inArea(position, input.ballRadius, pelvis) ? natural : -1.0f;
    if (bestAt > natural * AnticipateStretch) {
        early = true;
        if (later != nullptr) *later = bestAt;
        return -1.0f;
    }
    return bestAt;
}

float BallControl3D::areaExit(const BallControlInput3D& input, float horizon) const {
    // (A bola de verdade - nunca a imaginada: a forca e nela.)
    const Vec3 pelvisNow = input.state->links.front().position;
    for (float at = 0.0f; at <= horizon + 1e-4f; at += 0.01f) {
        const Vec3 pelvis = predictPelvis(input, at);
        const Vec3 ball = predictBallFrom(input, input.ballPosition, input.ballVelocity, at);
        if (!inArea(ball, input.ballRadius, Vec3 { pelvis.x, pelvis.y, pelvisNow.z })) return at;
    }
    return -1.0f;
}

bool BallControl3D::plan(const BallControlInput3D& input, int side, bool moving, float contactAt, Gesture& result) {
    const RagdollState3D& state = *input.state;
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    Vec3 forward, leftAxis;
    bodyFrame(input, forward, leftAxis);
    const float out = side == 0 ? 1.0f : -1.0f;
    const FootGeometry& foot = m_feet[static_cast<std::size_t>(side)];
    const int supportSide = 1 - side;
    const bool shot = m_kind == BallActionKind3D::Shot;
    const float p = std::clamp(m_power, 0.0f, 1.0f);
    const float q = shot ? std::max(p, 0.7f) : p;
    const Vec3 push = requestPush(forward);
    const Vec3 pelvisNow = state.links.front().position;
    const std::size_t s = static_cast<std::size_t>(side);
    // Andando: o instante do contato pela passada (movingContact).
    float movingAt = -1.0f;
    if (moving) {
        float waitLift = 0.0f;
        movingAt = contactAt >= 0.0f ? contactAt : movingContact(input, side, shot, waitLift, nullptr);
        if (movingAt < 0.0f) return false;
    }
    // O estilo e a superficie pela bola no contato (prevista).
    const float lead0 = moving ? movingAt : (contactAt >= 0.0f ? contactAt : naturalContact(input, side));
    Style style = Style::Front;
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    bool virtualFront = false;
    if (!styleFor(side, predictBall(input, lead0), moving ? predictPelvis(input, lead0) : pelvisNow, forward, push,
            moving, style, surface, virtualFront))
        return false;
    Gesture g;
    g.kind = m_kind;
    g.surface = surface;
    g.style = style;
    g.side = side;
    g.power = p;
    g.push = push;
    g.contactPush = push;
    g.bodyForward = forward;
    g.moving = moving;
    g.virtualFront = virtualFront;
    g.midline = pelvisNow;
    // O referencial do gesto: o pe passa no rumo do empurrao.
    m_frameSwing = push;
    m_frameOut = cross(up, push) * out;
    // Tempos.
    // (Todo chute tem o gesto de chute - o de efeito so muda a superficie no
    // contato; o calcanhar e a sola nao.)
    const bool kickLike = shot && surface != BallTouchSurface3D::Heel && surface != BallTouchSurface3D::Sole;
    const bool behindStyle = style != Style::Front;
    {
        const GestureTimes times = referenceTimes(kickLike,
            surface == BallTouchSurface3D::Sole || surface == BallTouchSurface3D::Heel, behindStyle, p, q);
        g.windupAt = times.windup;
        g.contactAt = times.contact;
        g.followAt = times.follow;
        g.endAt = times.end;
    }
    // Andando: no ritmo da passada (o pe que joga esta no balanco; o instante
    // do contato vem da escolha do pe - movingContact). O chute arma ate o
    // apoio pousar, bate logo depois (no apoio) e acompanha no voo; o toque sai
    // perto do fim do balanco, com a bola a uma distancia boa a frente.
    if (moving) {
        const float touchdown = input.secondsToTouchdown[s] > 0.0f ? input.secondsToTouchdown[s] : m_swingSeconds[s];
        if (kickLike) {
            g.contactAt = std::max(0.10f, movingAt);
            g.windupAt = std::max(0.05f, g.contactAt - (0.07f + 0.02f * p));
            g.followAt = g.contactAt + 0.09f + 0.04f * p;
            // (A volta para a passada sem pressa: a perna desce do alto.)
            g.endAt = std::max(g.followAt + 0.16f, touchdown + 0.04f);
        } else {
            g.contactAt = std::max(0.06f, movingAt);
            g.windupAt = g.contactAt * 0.5f;
            g.followAt = g.contactAt + 0.05f + 0.04f * q;
            g.endAt = std::max(g.followAt + 0.07f, touchdown + 0.03f);
        }
    } else if (contactAt >= 0.0f) {
        // Parado, a bola andando (V3): o contato na chegada dela - o gesto
        // encolhe (ate AnticipateCompress) ou o armado espera (mais devagar).
        const float natural = g.contactAt;
        const float follow = g.followAt - g.contactAt, end = g.endAt - g.followAt;
        if (contactAt < natural) {
            const float k = std::max(AnticipateCompress, contactAt / natural);
            g.windupAt *= k;
            g.contactAt = natural * k;
        } else {
            g.windupAt += contactAt - natural;
            g.contactAt = contactAt;
        }
        g.followAt = g.contactAt + follow;
        g.endAt = g.followAt + end;
    }
    // O corpo nas chaves (mais desenho com mais forca).
    const float amount = shot ? KickBodyBase + (1.0f - KickBodyBase) * p : TouchBodyBase + (1.0f - TouchBodyBase) * q;
    BodyKey& bw = g.bodyWindup;
    BodyKey& bc = g.bodyContact;
    BodyKey& bf = g.bodyFollow;
    if (kickLike) {
        // Armando: o quadril do chute vai para tras (a pelve gira para o lado
        // dele), o tronco vai para tras e para o lado do apoio, os ombros
        // ficam (giram contra a pelve), o braco do apoio abre para o lado e
        // o do chute vem a frente. Batendo: a pelve gira para a bola, o tronco
        // vem por cima dela, o braco do apoio cruza a frente e o do chute vai
        // para tras. O peso desce na perna de apoio.
        bw = { 0.025f, out * 0.30f, -0.04f, -out * 0.05f, 0.45f, -out * 0.40f, -0.10f, -out * 0.12f,
            { 0.16f, 0.12f, -0.38f }, { -0.06f, 0.44f, -0.06f }, 1.0f };
        bc = { 0.065f, -out * 0.06f, 0.05f, -out * 0.06f, 0.55f, out * 0.02f, 0.14f, -out * 0.16f,
            { -0.18f, 0.20f, -0.36f }, { 0.20f, 0.34f, -0.18f }, 1.0f };
        bf = { 0.040f, -out * 0.26f, 0.06f, -out * 0.04f, 0.40f, out * 0.06f, 0.20f, -out * 0.10f,
            { -0.30f, 0.16f, -0.30f }, { 0.30f, -0.02f, -0.30f }, 1.0f };
    } else if (style == Style::BehindOut) {
        // Atras, para fora (a referencia do Pro Soccer Online): a perna vai
        // para tras e para fora, o tronco inclina para a frente (contrapeso)
        // e para o lado do apoio, a pelve abre para o lado da perna e sobe
        // dele; os bracos abrem bem para os lados.
        bw = { 0.050f, out * 0.10f, 0.12f, -out * 0.05f, 0.50f, -out * 0.05f, 0.26f, -out * 0.10f,
            { 0.10f, 0.34f, -0.24f }, { 0.10f, 0.38f, -0.20f }, 1.0f };
        bc = { 0.090f, out * 0.16f, 0.22f, -out * 0.08f, 0.55f, -out * 0.08f, 0.46f, -out * 0.14f,
            { 0.08f, 0.42f, -0.14f }, { 0.14f, 0.42f, -0.14f }, 1.0f };
        bf = { 0.070f, out * 0.10f, 0.16f, -out * 0.06f, 0.45f, -out * 0.05f, 0.34f, -out * 0.10f,
            { 0.08f, 0.38f, -0.18f }, { 0.12f, 0.38f, -0.18f }, 1.0f };
    } else if (surface == BallTouchSurface3D::Sole || surface == BallTouchSurface3D::Heel) {
        // A sola (e o calcanhar): o corpo um pouco para tras, os bracos a
        // frente e abertos.
        bw = { 0.02f, 0.0f, -0.02f, 0.0f, 0.35f, 0.0f, -0.05f, -out * 0.04f,
            { 0.12f, 0.16f, -0.40f }, { 0.14f, 0.24f, -0.34f }, 1.0f };
        bc = { 0.04f, 0.0f, -0.03f, 0.0f, 0.45f, 0.0f, -0.08f, -out * 0.06f,
            { 0.16f, 0.18f, -0.38f }, { 0.16f, 0.28f, -0.30f }, 1.0f };
        bf = { 0.03f, 0.0f, -0.02f, 0.0f, 0.35f, 0.0f, -0.04f, -out * 0.04f,
            { 0.08f, 0.16f, -0.42f }, { 0.10f, 0.24f, -0.36f }, 1.0f };
        if (style == Style::BehindIn) {
            // (A perna para tras: o tronco vem para a frente.)
            bw.trunkBend = 0.10f;
            bc.trunkBend = 0.16f;
            bf.trunkBend = 0.10f;
            bw.pelvisPitch = bc.pelvisPitch = bf.pelvisPitch = 0.04f;
        }
    } else if (surface == BallTouchSurface3D::Outside) {
        // De fora: a pelve fecha um pouco, o corpo inclina para o lado do
        // apoio, o braco do apoio abre.
        bw = { 0.015f, -out * 0.06f, 0.0f, -out * 0.03f, 0.35f, out * 0.04f, 0.03f, -out * 0.06f,
            { 0.04f, 0.20f, -0.40f }, { 0.04f, 0.32f, -0.30f }, 1.0f };
        bc = { 0.030f, -out * 0.10f, 0.03f, -out * 0.05f, 0.45f, out * 0.06f, 0.07f, -out * 0.10f,
            { -0.06f, 0.24f, -0.36f }, { 0.10f, 0.36f, -0.24f }, 1.0f };
        bf = { 0.020f, -out * 0.08f, 0.03f, -out * 0.03f, 0.35f, out * 0.05f, 0.06f, -out * 0.07f,
            { -0.08f, 0.22f, -0.38f }, { 0.12f, 0.30f, -0.30f }, 1.0f };
    } else {
        // Lado de dentro (e o peito do pe no toque): a pelve abre (o quadril
        // gira para fora com o pe), os ombros ficam na bola, o tronco vem por
        // cima dela e inclina para o apoio; os bracos abrem para equilibrar.
        const float open = surface == BallTouchSurface3D::Inside ? 1.0f : 0.3f;
        bw = { 0.020f, out * 0.26f * open, 0.02f, -out * 0.03f, 0.35f, -out * 0.18f * open, 0.04f, -out * 0.07f,
            { -0.04f, 0.26f, -0.36f }, { 0.08f, 0.32f, -0.28f }, 1.0f };
        bc = { 0.040f, out * 0.20f * open, 0.04f, -out * 0.04f, 0.45f, -out * 0.14f * open, 0.10f, -out * 0.09f,
            { -0.10f, 0.28f, -0.34f }, { 0.14f, 0.34f, -0.24f }, 1.0f };
        bf = { 0.030f, out * 0.08f * open, 0.04f, -out * 0.03f, 0.35f, -out * 0.06f * open, 0.10f, -out * 0.06f,
            { -0.14f, 0.24f, -0.34f }, { 0.18f, 0.26f, -0.28f }, 1.0f };
    }
    for (BodyKey* key : { &bw, &bc, &bf }) {
        key->pelvisDrop *= amount;
        key->pelvisYaw *= amount;
        key->pelvisPitch *= amount;
        key->pelvisRoll *= amount;
        key->trunkTwist *= amount;
        key->trunkBend *= amount;
        key->trunkSide *= amount;
        key->arms *= amount;
    }
    // Previsoes: a pelve, o quadril e a bola no contato.
    const float lead = g.contactAt;
    const Vec3 hipNow = hipJoint(state, side);
    const auto pelvisAt = [&](float seconds) {
        return moving ? predictPelvis(input, seconds) : pelvisNow;
    };
    const auto hipAt = [&](float seconds, float drop) {
        return hipNow + planar(pelvisAt(seconds) - pelvisNow) - up * drop;
    };
    const Vec3 pelvisAtContact = pelvisAt(lead);
    Vec3 hipAtContact = hipAt(lead, bc.pelvisDrop);
    const Vec3 ballReal = predictBall(input, lead);
    Vec3 ball = ballReal;
    // A bola um pouco atras tocada para a frente (V3): imaginada logo a
    // frente dele (o gesto bonito vale mais que o ponto exato).
    float virtualShift = 0.0f;
    if (virtualFront) {
        const float along = dot(planar(ball - pelvisAtContact), forward);
        if (along < VirtualAhead) {
            virtualShift = VirtualAhead - along;
            ball += forward * virtualShift;
        }
    }
    g.ballAtContact = ball;
    // A bola a frente do apoio no chute: a pelve vai sobre ele (o afundo).
    const Vec3 supportNow = ankleJoint(state, supportSide);
    if (kickLike) {
        const float ahead = dot(planar(ball - supportNow), push);
        const float lunge = std::clamp((ahead - 0.12f) * 0.5f, 0.0f, LungeForwardMaximum);
        bw.pelvisForward = lunge * 0.3f;
        bc.pelvisForward = lunge;
        bf.pelvisForward = lunge * 0.8f;
        bc.trunkBend += lunge * 0.8f;
        hipAtContact += push * lunge;
    }
    // O contato numa superficie: a perna alcanca; se nao, a pelve desce (o
    // joelho de apoio dobra, ate LungeDropMaximum); se ainda nao, a perna
    // estica ate onde da (a bola esta na area: o toque sai pelo tempo, mesmo
    // com o pe chegando so perto dela).
    const float legLength = (foot.thigh + foot.shin) * ContactStretch;
    Vec3 footAt, ankleAt;
    Quaternion footOrientation;
    float drop = 0.0f, reachGap = 0.0f;
    const auto fit = [&](BallTouchSurface3D trySurface, Vec3 tryPush, bool anyGap) {
        contactPose(side, trySurface, style, ball, input.ballRadius, tryPush, forward, p, input.groundHeight, footAt,
            footOrientation);
        ankleAt = footAt + footOrientation.rotate(foot.ankleAnchor);
        Vec3 hip = hipAtContact;
        drop = 0.0f;
        reachGap = 0.0f;
        const Vec3 reach = ankleAt - hip;
        const float flat = planar(reach).length();
        if (reach.length() > legLength) {
            const float height = flat < legLength ? std::sqrt(legLength * legLength - flat * flat) : 0.0f;
            drop = std::clamp(hip.z - (ankleAt.z + height), 0.0f, moving ? 0.05f : LungeDropMaximum);
            hip.z -= drop;
        }
        const Vec3 again = ankleAt - hip;
        if (again.length() > legLength) {
            const Vec3 stretched = hip + again * (legLength / again.length());
            const Vec3 stretchedFoot = stretched - footOrientation.rotate(foot.ankleAnchor);
            reachGap = std::max(0.0f, boxGap(side, stretchedFoot, footOrientation, ball, input.ballRadius, 0.0f));
            if (!anyGap && reachGap > 0.06f) return false;
            ankleAt = stretched;
            footAt = stretchedFoot;
        }
        return true;
    };
    BallTouchSurface3D used = surface;
    const bool pulling = surface == BallTouchSurface3D::Sole || surface == BallTouchSurface3D::Heel;
    if (!fit(surface, push, shot || behindStyle || pulling)) {
        // A superficie de lado nao chega: o toque esticado, de peito/ponta
        // do pe apontando para a bola (alcanca ~20 cm a mais). (Puxando - a
        // sola e o calcanhar -, a mesma superficie esticada: o pe indo para a
        // frente na bola longe empurrava-a para a frente depois da forca.)
        const Vec3 toward = planar(ball - hipNow);
        if (toward.lengthSquared() < 1e-6f) return false;
        fit(BallTouchSurface3D::Laces, toward.normalized(), true);
        used = BallTouchSurface3D::Laces;
        g.contactPush = toward.normalized();
    }
    if (drop > 0.0f) {
        bc.pelvisDrop += drop;
        bw.pelvisDrop += drop * 0.4f;
        bf.pelvisDrop += drop * 0.7f;
        bc.trunkBend += drop * 1.5f;
        hipAtContact.z -= drop;
    }
    g.surface = used;
    // As pernas nunca se cruzam (V3): o pe do contato do lado dele da linha
    // do meio do corpo e longe do tornozelo de apoio (andando, so com ele
    // plantado - de lado, o corpo passa por cima dele e ele fica do outro
    // lado da pelve; a folga e menor, os pes da corrida passam perto). Tirado
    // da bola ate VirtualShiftMaximum (a forca vem do tempo); alem disso, a
    // outra perna faz.
    const Vec3 supportAnkle = supportNow;
    const bool useSupport = !moving || input.footPlanted[static_cast<std::size_t>(supportSide)];
    const float supportGap = moving ? FootSideGap * 0.8f : FootSideGap;
    float crossShift = 0.0f;
    {
        const Vec3 clean = keepOwnSide(side, ankleAt, pelvisAtContact, forward, supportAnkle, useSupport ? 1.0f : 0.0f, false,
            supportGap);
        const Vec3 delta = clean - ankleAt;
        crossShift = planar(delta).length();
        if (crossShift > VirtualShiftMaximum) {
            if (std::getenv("XBALLPLAN"))
                std::printf("PLAN pe %d: cruzaria (%.2f m) - descartado\n", side, crossShift);
            return false;
        }
        ankleAt = clean;
        footAt = ankleAt - footOrientation.rotate(foot.ankleAnchor);
        g.contactOffset = (ball - ballReal) + delta;
    }
    // As chaves da perna (polares em volta do quadril).
    const Vec3 ankleNow = ankleJoint(state, side);
    g.start = legKeyFrom(ankleNow - hipNow);
    g.start.foot = state.links[foot.link].orientation;
    g.start.knee = state.links[foot.hip].orientation.rotate({ 1.0f, 0.0f, 0.0f });
    g.startAnkleWorld = ankleNow;
    g.startFootWorld = g.start.foot;
    g.supportFootWorld = supportAnkle;
    g.contact = legKeyFrom(ankleAt - hipAtContact);
    g.contact.foot = footOrientation;
    g.contactAnkle = ankleAt;
    // O joelho aponta para onde vai a ponta do pe (a perna gira inteira pelo
    // quadril) e um pouco para a frente do corpo; nunca muito para dentro
    // (as coxas nao se cruzam).
    const Vec3 outward = leftAxis * out;
    const auto kneeNotInward = [&](Vec3 knee) {
        const float inward = -dot(knee, outward);
        if (inward > 0.3f) knee += outward * (inward - 0.3f);
        return knee.lengthSquared() > 1e-6f ? knee.normalized() : forward;
    };
    const Vec3 toe = planar(footOrientation.rotate({ 1.0f, 0.0f, 0.0f }));
    g.contact.knee = kneeNotInward(nlerpDirection(toe.lengthSquared() > 1e-6f ? toe.normalized() : forward, forward, 0.25f));
    if (g.surface == BallTouchSurface3D::Heel) g.contact.knee = forward;
    // (A perna para tras: o joelho dobra para a frente - o tornozelo nao
    // gira, a ponta do pe vai para onde o joelho aponta: para fora, ela virava
    // para a bola e o pe entrava nela.)
    if (behindStyle) g.contact.knee = forward;
    const auto clean = [&](Vec3 ankleFromHip, Vec3 hip, float seconds) {
        // (No plano: o pe que joga do lado dele e longe do de apoio.)
        const Vec3 at = keepOwnSide(side, hip + ankleFromHip, pelvisAt(seconds), forward, supportAnkle,
            useSupport ? 1.0f : 0.0f, false,
            supportGap);
        return at - hip;
    };
    const Vec3 hipAtWindup = hipAt(g.windupAt, bw.pelvisDrop);
    const Vec3 hipAtFollow = hipAt(g.followAt, bf.pelvisDrop);
    if (kickLike) {
        LegKey windup = legKeyAngles(side, -(KickWindupThigh + KickWindupThighPower * p),
            KickWindupKnee + KickWindupKneePower * p, KickWindupOut);
        // O joelho: no peito do pe, para a frente e um pouco para fora; com o
        // lado de dentro (a curva), bem para fora (o quadril gira); na
        // trivela, para dentro.
        const float kneeOut = g.surface == BallTouchSurface3D::Inside ? 0.40f
            : g.surface == BallTouchSurface3D::Outside ? -0.15f : 0.18f;
        windup.knee = kneeNotInward(kneeOut >= 0.0f ? nlerpDirection(push, m_frameOut, kneeOut)
            : nlerpDirection(push, m_frameOut * -1.0f, -kneeOut));
        const Quaternion windupFoot = footAlongShin(side, windup, 0.55f);
        g.windup = legKeyFrom(clean(legKeyVector(windup), hipAtWindup, g.windupAt));
        g.windup.foot = windupFoot;
        g.windup.knee = windup.knee;
        if (moving) {
            // Correndo, o balanco ja leva o pe para tras e para cima: o armado
            // vai so parte do caminho a partir dele (sem o pe voar), e o pe
            // gira so metade.
            const LegKey from = legKeyFrom(ankleJoint(state, side) - hipNow);
            g.windup.swing = from.swing + (g.windup.swing - from.swing) * 0.7f;
            g.windup.length = from.length + (g.windup.length - from.length) * 0.8f;
            g.windup.lateral = from.lateral + (g.windup.lateral - from.lateral) * 0.7f;
            g.windup.foot = slerpQuat(state.links[foot.link].orientation, g.windup.foot, 0.5f);
        }
        // O acompanhamento: mais alto na bola alta, mais baixo na rasteira; a
        // curva vai para a frente do corpo, a trivela abre para fora (nunca
        // cruzando o apoio).
        const float high = std::max(0.0f, m_lift), low = std::max(0.0f, -m_lift);
        const float followLateral = g.surface == BallTouchSurface3D::Inside ? -(0.25f + 0.15f * p)
            : g.surface == BallTouchSurface3D::Outside ? 0.12f : -KickFollowIn * (0.5f + p);
        LegKey follow = legKeyAngles(side, KickFollowThigh + KickFollowThighPower * p + 0.25f * high - 0.20f * low,
            KickFollowKnee + KickFollowKneePower * p, followLateral);
        follow.knee = g.surface == BallTouchSurface3D::Inside ? nlerpDirection(push, m_frameOut, 0.3f)
            : nlerpDirection(push, m_frameOut * -1.0f, g.surface == BallTouchSurface3D::Outside ? 0.3f : 0.12f);
        follow.knee = kneeNotInward(follow.knee);
        const Quaternion followFoot = g.surface == BallTouchSurface3D::Laces ? footAlongShin(side, follow, 0.70f)
            : slerpQuat(footOrientation, footAlongShin(side, follow, 0.4f), 0.4f);
        g.follow = legKeyFrom(clean(legKeyVector(follow), hipAtFollow, g.followAt));
        g.follow.foot = followFoot;
        g.follow.knee = follow.knee;
        // O tronco: para tras na bola alta, por cima dela na rasteira.
        bc.trunkBend += (-0.20f * high + 0.08f * low) * amount;
        bf.trunkBend += (-0.15f * high + 0.06f * low) * amount;
    } else {
        const Vec3 swing = m_frameSwing;
        Vec3 windupAnkle, followAnkle;
        if (style == Style::BehindOut) {
            // Atras, para fora: o pe sobe por tras dela, do lado do corpo, de
            // ponta para baixo, e sai para fora passando por ela.
            windupAnkle = ankleAt - swing * (0.09f + 0.08f * q) + up * (0.06f + 0.04f * q) + forward * 0.02f;
            followAnkle = ankleAt + swing * (0.10f + 0.14f * q) + up * (0.03f + 0.03f * q) - forward * 0.02f;
        } else if (style == Style::BehindIn) {
            // Atras, para dentro: a sola por cima dela puxa curto (nunca passa
            // do meio do corpo - keepOwnSide).
            windupAnkle = ankleAt - swing * 0.06f + up * 0.04f;
            followAnkle = ankleAt + swing * (0.08f + 0.08f * q) + up * 0.015f;
        } else if (g.surface == BallTouchSurface3D::Sole) {
            // Por cima da bola (do lado de la) e puxando para o lado do
            // empurrao.
            windupAnkle = ankleAt - swing * 0.07f + up * 0.035f;
            followAnkle = ankleAt + swing * (0.14f + 0.16f * q) + up * 0.015f;
        } else if (g.surface == BallTouchSurface3D::Heel) {
            // O calcanhar: o pe vem da frente da bola (do lado do corpo) e
            // bate para tras, continuando um pouco.
            windupAnkle = ankleAt - swing * (0.14f + 0.12f * q) + up * 0.06f;
            followAnkle = ankleAt + swing * (0.06f + 0.10f * q) + up * 0.05f;
        } else {
            windupAnkle = ankleAt - swing * (TouchBack + TouchBackPower * q) + up * (TouchLift + TouchLiftPower * q)
                + m_frameOut * (0.02f + 0.04f * q);
            followAnkle = ankleAt + swing * (TouchAhead + TouchAheadPower * q) + up * (0.015f + 0.06f * q)
                - m_frameOut * (0.02f * q);
        }
        g.windup = legKeyFrom(clean(windupAnkle - hipAtContact, hipAtWindup, g.windupAt));
        if (moving && g.surface != BallTouchSurface3D::Sole && !behindStyle) {
            // Andando: o pe passa por baixo do quadril, dobrado (a passada),
            // e vai a bola - se ja nao estiver a frente (vai direto).
            const float legs = foot.thigh + foot.shin;
            Vec3 passing = planar(ankleAt - hipAtContact) * 0.35f;
            passing.z = -(legs - 0.20f);
            g.windup = legKeyFrom(clean(passing, hipAtWindup, g.windupAt));
            if (g.start.swing > g.windup.swing) {
                const LegKey direct = g.start;
                g.windup.swing = direct.swing + (g.contact.swing - direct.swing) * 0.4f;
                g.windup.length = std::min(direct.length, g.contact.length) - 0.04f;
                g.windup.lateral = direct.lateral + (g.contact.lateral - direct.lateral) * 0.4f;
            }
        }
        g.windup.foot = g.surface == BallTouchSurface3D::Sole || behindStyle ? footOrientation
            : slerpQuat(g.start.foot, footOrientation, moving ? 0.6f : 0.85f);
        g.windup.knee = g.contact.knee;
        g.follow = legKeyFrom(clean(followAnkle - hipAtContact, hipAtFollow, g.followAt));
        g.follow.foot = footOrientation;
        g.follow.knee = g.contact.knee;
    }
    // O tempo ate o contato pelo caminho do pe (V3, parado e no tempo dele):
    // o armado e a batida duram o caminho do tornozelo pela velocidade do
    // gesto, em volta dos tempos de referencia.
    if (!moving && contactAt < 0.0f) {
        const Vec3 a = legKeyVector(g.start), b = legKeyVector(g.windup), c = legKeyVector(g.contact);
        const float windupTravel = (b - a).length() + 0.06f * turnBetween(g.start.foot, g.windup.foot);
        const float strikeTravel = (c - b).length() + 0.03f * turnBetween(g.windup.foot, g.contact.foot);
        const float windupSpeed = kickLike ? KickWindupSpeed + KickWindupSpeedPower * p
            : TouchWindupSpeed + TouchWindupSpeedPower * q;
        const float strikeSpeed = kickLike ? KickStrikeSpeed + KickStrikeSpeedPower * p
            : TouchStrikeSpeed + TouchStrikeSpeedPower * q;
        const float referenceWindup = g.windupAt, referenceStrike = g.contactAt - g.windupAt;
        const float windup = std::clamp(windupTravel / std::max(0.1f, windupSpeed),
            referenceWindup * TimingShortest, referenceWindup * TimingLongest);
        const float strike = std::clamp(strikeTravel / std::max(0.1f, strikeSpeed),
            referenceStrike * TimingShortest, referenceStrike * TimingLongest);
        const float shift = windup + strike - g.contactAt;
        if (std::getenv("XBALLPLAN"))
            std::printf("TEMPO pe %d sup %d caminho armado %.3f m batida %.3f m | ref %.3f %.3f -> %.3f %.3f s\n", side,
                static_cast<int>(g.surface), windupTravel, strikeTravel, referenceWindup, referenceStrike, windup,
                strike);
        g.windupAt = windup;
        g.contactAt = windup + strike;
        g.followAt += shift;
        g.endAt += shift;
    }
    // O armado no tempo que ha: a perna nao vai mais rapido que isto (rad/s do
    // balanco; o comprimento pesa em dobro) - sem tempo, arma so parte do
    // caminho (correndo, o pe a frente do corpo nao volta inteiro para tras).
    {
        const float footTurn = turnBetween(g.start.foot, g.windup.foot);
        const float travel = std::abs(g.windup.swing - g.start.swing) + 2.0f * std::abs(g.windup.length - g.start.length)
            + std::abs(g.windup.lateral - g.start.lateral) + 0.5f * footTurn;
        const float need = travel / WindupSwingRate;
        if (need > g.windupAt && need > 1e-3f) {
            const float k = std::clamp(g.windupAt / need, 0.2f, 1.0f);
            g.windup.swing = g.start.swing + (g.windup.swing - g.start.swing) * k;
            g.windup.length = g.start.length + (g.windup.length - g.start.length) * k;
            g.windup.lateral = g.start.lateral + (g.windup.lateral - g.start.lateral) * k;
            g.windup.foot = slerpQuat(g.start.foot, g.windup.foot, k);
            g.windup.knee = nlerpDirection(g.start.knee, g.windup.knee, k);
        }
    }
    // O fim: parado, o pe volta para onde estava; andando, a passada o leva
    // (o peso sai): o toque fica no acompanhamento; o chute desce a perna para
    // a frente e baixo, perto de onde a passada pousa.
    g.recover = moving ? g.follow : g.start;
    if (moving && kickLike) {
        const float legs = foot.thigh + foot.shin;
        LegKey landing = legKeyFrom(push * 0.30f - up * (legs - 0.10f));
        landing.foot = yawPitch(push, 0.15f);
        landing.knee = push;
        g.recover = landing;
    }
    // A batida acelera ate a bola (balanco e joelho), sem passar do
    // acompanhamento.
    const float strike = std::max(0.02f, g.contactAt - g.windupAt);
    const float follow = std::max(0.02f, g.followAt - g.contactAt);
    const auto limitRate = [](float rate, float span, float h) {
        if (rate * span <= 0.0f) return 0.0f;
        const float most = 2.5f * std::abs(span) / h;
        return std::clamp(rate, -most, most);
    };
    // (O toque fraco freia: a perna guia a bola; o forte e o chute aceleram.)
    const float rateScale = kickLike ? 1.0f
        : std::min(1.0f, (TouchStrikeRate + TouchStrikeRatePower * q) / StrikeSwingRate);
    g.swingRate = limitRate(StrikeSwingRate * rateScale * (g.contact.swing - g.windup.swing) / strike,
        g.follow.swing - g.contact.swing, follow);
    g.lengthRate = limitRate(StrikeLengthRate * rateScale * (g.contact.length - g.windup.length) / strike,
        g.follow.length - g.contact.length, follow);
    // O custo da solucao (a escolha da perna): cruzar (o contato tirado da
    // bola), a bola imaginada a frente, o pe esticado sem chegar, o afundo,
    // a bola do lado da outra perna e as superficies menos naturais.
    const float ballSide = dot(planar(ballReal - pelvisAtContact), leftAxis) * out;
    float cost = 1.5f * crossShift + 0.4f * virtualShift + 1.2f * reachGap + 0.8f * drop
        + 0.8f * std::max(0.0f, -ballSide);
    if (used != surface) cost += 0.25f;
    if (style == Style::Front && used == BallTouchSurface3D::Outside) cost += 0.06f;
    if (style == Style::BehindIn) cost += 0.2f;
    g.cost = cost;
    if (std::getenv("XBALLPLAN"))
        std::printf("PLAN pe %d sup %d estilo %d mov %d | cruza %.2f virtual %.2f falta %.2f afundo %.2f bola-lado %+.2f"
            " custo %.2f | armar %.2f contato %.2f acomp %.2f fim %.2f\n",
            side, static_cast<int>(g.surface), static_cast<int>(style), moving ? 1 : 0, crossShift, virtualShift,
            reachGap, drop, ballSide, cost, g.windupAt, g.contactAt, g.followAt, g.endAt);
    g.active = true;
    g.time = 0.0f;
    g.touched = false;
    result = g;
    return true;
}

void BallControl3D::imagineBallAhead(const BallControlInput3D& input, int side) {
    Vec3 forward, left;
    bodyFrame(input, forward, left);
    const Vec3 velocity = bodyVelocityOf(input);
    const float out = side == 0 ? 1.0f : -1.0f;
    const Vec3 pelvis = input.state->links.front().position;
    // (A frente do corpo, do lado da perna - as pernas nunca cruzam -, um
    // pouco adiantada no rumo do movimento; andando junto com o corpo.)
    const Vec3 travel = velocity.length() > 1.0f ? velocity.normalized() : forward;
    m_virtualPosition = Vec3 { pelvis.x, pelvis.y, input.groundHeight + input.ballRadius } + forward * 0.32f
        + left * (0.14f * out) + travel * 0.12f;
    m_virtualVelocity = velocity;
    m_virtualClock = 0.0f;
    m_virtualBall = true;
}

void BallControl3D::startAction() {
    m_action = true;
    m_actionClock = 0.0f;
    m_forceAt = 0.0f;
    m_forced = false;
    m_virtualBall = false;
    m_imagined = false;
    m_actionCommitted = false;
    m_bareForce = false;
}

void BallControl3D::endAction() {
    m_action = false;
    m_forced = false;
    m_virtualBall = false;
    m_imagined = false;
    m_waiting = false;
    m_actionCommitted = false;
    m_bareForce = false;
}

void BallControl3D::scheduleForce(const BallControlInput3D& input, float secondsToContact) {
    // A forca: na fracao do tempo estimado ate o contato visual, contado do
    // comeco da acao (o pedido aceito; andando, com a espera do pe). A bola
    // saindo da area antes disso: a forca vem antes, com ela ainda dentro (so
    // dentro da area ela sai) - o tempo encurta sozinho, ate zero (pedido do
    // usuario, 06/10 noite).
    const float ahead = std::max(0.0f, secondsToContact);
    m_forceAt = m_timingFactor * (m_actionClock + ahead);
    const float exit = areaExit(input, ahead);
    if (exit > 0.0f) m_forceAt = std::min(m_forceAt, m_actionClock + std::max(0.0f, exit - 0.02f));
    if (std::getenv("XBALLPLAN"))
        std::printf("INICIO acao %.3f contato em %.3f saida %.3f forca em %.3f | bola a %.2f m\n", m_actionClock, ahead,
            exit, m_forceAt, planar(input.ballPosition - input.state->links.front().position).length());
}

void BallControl3D::beginGesture(const BallControlInput3D& input, const Gesture& gesture) {
    // O gesto anterior (ja depois da forca) cortado por este, ou o preparo
    // da espera do pe: a pose dele sai suave.
    m_carry = m_gesture.active || (m_waiting && m_lastPose.active) ? m_lastPose : BallGesturePose3D {};
    m_carryTime = 0.0f;
    m_gesture = gesture;
    m_gesture.active = true;
    m_gesture.time = 0.0f;
    m_gesture.touched = m_forced;
    // (Planejado na bola imaginada: o gesto inteiro e nela.)
    m_gesture.imagined = gesture.imagined || m_imagined;
    m_side = gesture.side;
    m_surface = gesture.surface;
    m_waiting = false;
    // (A bola de antes da forca so serviu para planejar: dali em diante o
    // gesto acompanha a bola de verdade, se ela saiu devagar.)
    m_virtualBall = false;
    m_imagined = false;
    if (!m_forced) scheduleForce(input, gesture.contactAt + (input.animatedBody ? 0.0f : PhysicalLag));
    m_previousFootValid = false;
    m_physicalTrust = 1.0f;
    m_hitPrevious = false;
}

bool BallControl3D::busy() const {
    // A acao anterior ainda esperando o pe ou sem a forca, o gesto anterior
    // antes do contato visual (com a forca cedo, o pedido seguinte cortava a
    // batida no meio), ou o intervalo entre pedidos.
    return m_waiting || (m_action && !m_forced) || m_requestCooldown > 0.0f
        || (m_gesture.active && (!m_gesture.touched || m_gesture.time < m_gesture.contactAt));
}

void BallControl3D::takeRequest() {
    m_kind = m_request.kind;
    m_power = std::clamp(m_request.power, 0.0f, 1.0f);
    m_direction = m_request.direction;
    m_lookYaw = m_request.lookYaw;
    m_lift = m_request.lift;
    m_curve = m_request.curve;
    m_touchSpeed = m_request.touchSpeed;
}

int BallControl3D::preferredSide(const BallControlInput3D& input) const {
    // O desempate (o da V2): o pe do lado da bola; perto do meio, o que
    // empurra com o lado de dentro (ou o direito, para a frente).
    Vec3 forward, left;
    bodyFrame(input, forward, left);
    const float lateral = dot(planar(input.ballPosition - input.state->links.front().position), left);
    if (std::abs(lateral) >= 0.07f) return lateral >= 0.0f ? 0 : 1;
    const float across = dot(requestPush(forward), left);
    return std::abs(across) > 0.3f ? (across > 0.0f ? 1 : 0) : 1;
}

void BallControl3D::startBare(const BallControlInput3D& input) {
    startAction();
    m_waiting = false;
    m_bareForce = true;
    m_forceAt = m_actionClock;
    Vec3 forward, left;
    bodyFrame(input, forward, left);
    const Vec3 push = requestPush(forward);
    const bool moving = !input.standing && bodyVelocityOf(input).length() > 0.6f;
    const bool backward = std::abs(signedAngle(forward, push)) > (moving ? 1.95f : 2.05f);
    const bool ballBehind = dot(planar(input.ballPosition - input.state->links.front().position), forward) < -0.06f;
    m_surface = m_kind == BallActionKind3D::Shot ? BallTouchSurface3D::Laces
        : backward ? (ballBehind ? BallTouchSurface3D::Heel : BallTouchSurface3D::Sole) : BallTouchSurface3D::Inside;
}

bool BallControl3D::tryStart(const BallControlInput3D& input) {
    if (busy()) return false;
    if (!input.ballValid) return false;
    // Sem gesto: so a forca, com a bola na area (fora dela, o pedido espera
    // ela entrar no tempo guardado).
    if (input.forceOnly) {
        if (!inArea(input.ballPosition, input.ballRadius, input.state->links.front().position)) return false;
        takeRequest();
        startBare(input);
        return true;
    }
    // So com o corpo de pe na altura dele (caido, levantando ou agachado, o
    // gesto pela pelve que ainda sobe levava o pe a 0,6-0,8 m do chao).
    if (bodyHeightShare(input) < 0.85f) return false;
    takeRequest();
    const float speed = bodyVelocityOf(input).length();
    const bool moving = !input.standing && speed > 0.6f;
    // Arrancando (pedindo para andar com o corpo ainda parado): espera a
    // passada comecar - o gesto vai no ritmo dela; parado, o corpo saia
    // correndo com a perna do gesto presa (no fisico, tropecava). No fim do
    // pedido guardado, vai o gesto parado mesmo.
    // (Com a hitbox, o gesto parado ja: a capsula acelera uns 0,3 s com os
    // dois pes no chao antes da passada comecar, e o corpo passava da bola.)
    const bool starting = !input.standing && !moving && planar(input.requestedVelocity).length() > 1.0f;
    if (starting && !m_hitbox && m_pendingSeconds < RequestBuffer - 0.05f) return false;
    return moving ? startMoving(input, false) : startStanding(input, false, false);
}

bool BallControl3D::startStanding(const BallControlInput3D& input, bool late, bool imagined) {
    // Parado: as duas pernas planejadas, fica a mais limpa (menor custo); o
    // desempate pela perna do lado da bola.
    const int preferred = preferredSide(input);
    Vec3 position, velocity;
    ballNow(input, position, velocity);
    const bool ballMoving = planar(velocity).length() >= 0.3f
        || position.z > input.groundHeight + input.ballRadius + 0.02f || velocity.z > 0.1f;
    Gesture best;
    bool found = false, early = false;
    for (int side = 0; side < 2; ++side) {
        float at = -1.0f;
        if (imagined) {
            // (A bola imaginada logo a frente, do lado desta perna, no tempo
            // natural do gesto.)
            imagineBallAhead(input, side);
        } else {
            const float natural = naturalContact(input, side);
            bool tooEarly = false;
            float later = -1.0f;
            at = standingMeeting(input, side, natural, tooEarly, &later);
            early = early || tooEarly;
            // (A garantia: o armado espera a bola chegar.)
            if (at < 0.0f && late && tooEarly) at = later;
            if (at < 0.0f) continue;
        }
        Gesture g;
        if (!plan(input, side, false, !imagined && ballMoving ? at : -1.0f, g)) continue;
        g.imagined = imagined;
        if (side != preferred) g.cost += 0.01f;
        if (!found || g.cost < best.cost) {
            best = g;
            found = true;
        }
    }
    if (imagined) {
        m_virtualBall = false;
        m_imagined = false;
    }
    if (std::getenv("XBALLPLAN") && !found)
        std::printf("PEDIDO guardado (parado%s): %s\n", imagined ? ", imaginada" : "",
            early ? "cedo demais" : "sem encontro");
    if (!found) return false;
    startAction();
    beginGesture(input, best);
    return true;
}

bool BallControl3D::startMoving(const BallControlInput3D& input, bool imagined) {
    // Andando: o pe e o instante pela passada e pela bola - o que esta no
    // balanco (comeca ja) ou o proximo a sair do chao (a acao espera ele).
    const bool kick = m_kind == BallActionKind3D::Shot;
    int bestSide = -1;
    float bestScore = 1e9f, bestAt = -1.0f;
    for (int s = 0; s < 2; ++s) {
        // (Imaginada: a bola logo a frente, do lado desta perna.)
        if (imagined) {
            imagineBallAhead(input, s);
            m_imagined = true;
        }
        float wait = 0.0f, score = 1e9f;
        const float at = movingContact(input, s, kick, wait, &score);
        if (at >= 0.0f && score < bestScore) {
            bestScore = score;
            bestSide = s;
            bestAt = at;
        }
    }
    if (imagined) {
        m_virtualBall = false;
        m_imagined = false;
    }
    if (bestSide < 0) {
        if (std::getenv("XBALLPLAN"))
            std::printf("PEDIDO guardado (andando%s): sem janela\n", imagined ? ", imaginada" : "");
        return false;
    }
    const std::size_t i = static_cast<std::size_t>(bestSide);
    const bool swingingNow = !input.footPlanted[i] && input.swingProgress[i] >= 0.0f;
    startAction();
    // (A espera do pe e o gesto dela vao na bola imaginada.)
    if (imagined) {
        imagineBallAhead(input, bestSide);
        m_imagined = true;
    }
    if (swingingNow) {
        Gesture g;
        if (plan(input, bestSide, true, bestAt, g)) {
            beginGesture(input, g);
            return true;
        }
    }
    m_waiting = true;
    m_waitSide = swingingNow ? -1 : bestSide;
    m_waitSeconds = 0.0f;
    m_waitContact = bestAt;
    m_surface = kick ? BallTouchSurface3D::Laces : BallTouchSurface3D::Inside;
    scheduleForce(input, bestAt + (input.animatedBody ? 0.0f : PhysicalLag));
    return true;
}

bool BallControl3D::startCommitted(const BallControlInput3D& input, bool leaving) {
    // A garantia (06/10 noite): o pedido com a bola na area sai agora, o que
    // o corpo estiver fazendo. (A acao anterior ainda sem a forca e ela mesma
    // garantida - este espera ela sair.)
    if (m_action && !m_forced) return false;
    takeRequest();
    // (A acao anterior - ja com a forca - fica para tras: a bola dela, a de
    // antes da forca, nao serve mais.)
    m_virtualBall = false;
    m_imagined = false;
    const bool moving = !input.standing && bodyVelocityOf(input).length() > 0.6f;
    bool started = false;
    const char* how = "gesto";
    if (bodyHeightShare(input) >= 0.85f && !input.forceOnly) {
        // O gesto ate a bola de verdade; impossivel (atras correndo, saindo
        // da area, cruzando...), o gesto numa bola imaginada logo a frente.
        started = moving ? startMoving(input, false) : startStanding(input, true, false);
        if (!started) {
            started = moving ? startMoving(input, true) : startStanding(input, false, true);
            how = "gesto imaginado";
        }
    }
    if (!started) {
        // Sem gesto (o corpo baixo - caido, levantando, agachado -, nenhum
        // plano ou o laboratorio pedindo so a forca): so a forca, ja.
        startBare(input);
        how = "so a forca";
    }
    m_actionCommitted = true;
    if (leaving || m_bareForce) m_forceAt = m_actionClock;
    if (std::getenv("XBALLPLAN"))
        std::printf("GARANTIA %s%s - forca em %.3f s\n", how, leaving ? " (bola saindo da area)" : "", m_forceAt);
    return true;
}

void BallControl3D::evaluate(const BallControlInput3D& input, BallControlOutput3D& output) {
    Gesture& g = m_gesture;
    const RagdollState3D& state = *input.state;
    const Vec3 up { 0.0f, 0.0f, 1.0f };
    const float dt = std::max(1e-4f, input.deltaTime);
    const float out = g.side == 0 ? 1.0f : -1.0f;
    const FootGeometry& foot = m_feet[static_cast<std::size_t>(g.side)];
    g.time += dt;
    const float t = g.time;
    m_frameSwing = g.push;
    m_frameOut = cross(up, g.push) * out;
    const Vec3 bodyVelocity = g.moving ? bodyVelocityOf(input) : Vec3 {};
    // A linha do meio do corpo anda com ele (as pernas nao cruzam).
    g.midline += bodyVelocity * dt;
    // O quadril neste tick (o do tick anterior, andando).
    const Vec3 hip = hipJoint(state, g.side) + bodyVelocity * dt;
    const Vec3 supportAnkle = ankleJoint(state, 1 - g.side);
    // (O apoio na regra das pernas: parado sempre; andando, plantado - entra e
    // sai suave: o pe que joga preso ao lado do apoio plantado saltava 34 cm
    // num tick quando ele saia do chao.)
    const bool useSupport = !g.moving || input.footPlanted[static_cast<std::size_t>(1 - g.side)];
    if (t <= dt * 1.5f) m_supportHold = useSupport ? 1.0f : 0.0f;
    m_supportHold = std::clamp(m_supportHold + (useSupport ? dt / 0.08f : -dt / 0.12f), 0.0f, 1.0f);
    const float supportWeight = smooth(m_supportHold);
    const float supportGap = g.moving ? FootSideGap * 0.8f : FootSideGap;
    // Antes do contato, o contato segue a bola (e o corpo) previstos - sem a
    // forca ainda; e depois dela, se a bola sai devagar em relacao ao corpo
    // (o toque, nao o chute nem o passe forte): o pe chega nela e a empurra,
    // em vez de bater onde ela estaria (com a forca antes do contato visual,
    // correndo, ela ja ia 20-30 cm a frente). (O gesto imaginado fica na bola
    // dele: a de verdade esta onde o pe nao chega.)
    // (Sem a forca, so ate a metade do tempo ate o contato - onde a forca saia
    // com o fator 0,5 da V3 aprovada: o gesto e o dela, qualquer que seja o
    // fator. Seguindo a bola ate o ultimo tick, a batida curta saltava 30
    // graus num quadro e o chute correndo do fisico derrubava; parado no fim
    // do armado, o fisico chutava o ar. Dali em diante, como depois da forca:
    // so a bola lenta, ate FollowMaximum.)
    const float freezeAt = 0.5f * (g.contactAt + (input.animatedBody ? 0.0f : PhysicalLag));
    const bool freeTrack = !g.touched && t < freezeAt;
    const bool followBall = !g.imagined && !freeTrack && g.style == Style::Front && !g.virtualFront && input.ballValid
        && planar(input.ballVelocity - bodyVelocityOf(input)).length() < 3.0f;
    const bool trackBall = freeTrack || followBall;
    if (!g.imagined && trackBall && t < g.contactAt && input.ballValid) {
        const float left = g.contactAt - t;
        const Vec3 midlineAtContact = g.midline + bodyVelocity * left;
        Vec3 ball = predictBall(input, left);
        if (g.virtualFront) {
            const float along = dot(planar(ball - midlineAtContact), g.bodyForward);
            if (along < VirtualAhead) ball += g.bodyForward * (VirtualAhead - along);
        }
        g.ballAtContact = ball;
        Vec3 footAt;
        Quaternion footOrientation;
        contactPose(g.side, g.surface, g.style, ball, input.ballRadius, g.contactPush, g.bodyForward, g.power,
            input.groundHeight, footAt, footOrientation);
        // (A pelve ainda desce o que falta ate o contato.)
        const float times[5] = { 0.0f, g.windupAt, g.contactAt, g.followAt, g.endAt };
        const float drops[5] = { 0.0f, g.bodyWindup.pelvisDrop, g.bodyContact.pelvisDrop, g.bodyFollow.pelvisDrop, 0.0f };
        const float dropNow = bodyCurve(times, drops, 5, t - dt);
        const Vec3 hipAtContact = hip + bodyVelocity * left - up * (g.bodyContact.pelvisDrop - dropNow);
        Vec3 ankleAt = footAt + footOrientation.rotate(foot.ankleAnchor);
        ankleAt = keepOwnSide(g.side, ankleAt, midlineAtContact, g.bodyForward, supportAnkle, supportWeight, false,
            supportGap);
        // (Antes da forca, o contato e este; depois, acompanha a bola so ate
        // FollowMaximum alem dele - o corpo andando junto.)
        if (freeTrack) {
            g.contactAnkle = ankleAt;
        } else {
            const Vec3 shift = ankleAt - g.contactAnkle;
            const float distance = shift.length();
            if (distance > FollowMaximum) ankleAt = g.contactAnkle + shift * (FollowMaximum / distance);
        }
        LegKey contact = legKeyFrom(ankleAt - hipAtContact);
        contact.foot = footOrientation;
        contact.knee = g.contact.knee;
        g.contact = contact;
    }
    // A perna: as chaves polares no tempo do gesto.
    const auto blendKeys = [](const LegKey& a, const LegKey& b, float u, float uFoot) {
        LegKey k;
        k.swing = a.swing + (b.swing - a.swing) * u;
        k.lateral = a.lateral + (b.lateral - a.lateral) * u;
        k.length = a.length + (b.length - a.length) * u;
        k.foot = slerpQuat(a.foot, b.foot, uFoot);
        k.knee = nlerpDirection(a.knee, b.knee, u);
        return k;
    };
    LegKey key;
    float legWeight = 1.0f;
    if (t <= g.windupAt) {
        const float u = smoother(t / std::max(1e-3f, g.windupAt));
        key = blendKeys(g.start, g.windup, u, u);
        // O joelho dobra antes (o calcanhar sobe primeiro, depois a coxa vai
        // para tras): o comprimento chega no armado na metade do tempo.
        const float early = g.moving ? u : smooth(t / std::max(1e-3f, g.windupAt * 0.55f));
        key.length = g.start.length + (g.windup.length - g.start.length) * early;
        legWeight = smooth(t / 0.05f);
    } else if (t <= g.contactAt) {
        const float h = g.contactAt - g.windupAt;
        const float u = (t - g.windupAt) / h;
        key = blendKeys(g.windup, g.contact, smooth(u), std::pow(u, 1.4f));
        key.swing = hermite(g.windup.swing, g.contact.swing, 0.0f, g.swingRate * h, u);
        key.length = hermite(g.windup.length, g.contact.length, 0.0f, g.lengthRate * h, u);
    } else if (t <= g.followAt) {
        const float h = g.followAt - g.contactAt;
        const float u = (t - g.contactAt) / h;
        key = blendKeys(g.contact, g.follow, smooth(u), smooth(u * 1.5f));
        key.swing = hermite(g.contact.swing, g.follow.swing, g.swingRate * h, 0.0f, u);
        key.length = hermite(g.contact.length, g.follow.length, g.lengthRate * h, 0.0f, u);
    } else {
        const float u = smoother((t - g.followAt) / std::max(1e-3f, g.endAt - g.followAt));
        key = blendKeys(g.follow, g.recover, u, u);
        // Descendo, o joelho dobra no meio do caminho (nao cai esticada).
        key.length -= RecoverKneeBend * std::sin(Pi * u) * std::min(1.0f, g.power + 0.4f);
        // Andando, a passada leva o pe de volta; parado, o fim e onde ele
        // estava (o peso sai so no final). (O gesto parado com o corpo ja
        // andando - arrancando com a bola - devolve a perna a passada na volta
        // inteira: em 0,05 s ela saltava para onde a passada estava.)
        const bool walking = g.moving || bodyVelocityOf(input).length() > 0.6f;
        legWeight = walking ? 1.0f - u : 1.0f - smooth((t - (g.endAt - 0.05f)) / 0.05f);
    }
    // O comprimento nunca alem da perna.
    key.length = std::min(key.length, (foot.thigh + foot.shin) * 0.999f);
    Vec3 ankle = hip + legKeyVector(key);
    // Depois da forca o pe acompanha a bola sem passar por dentro dela (a
    // bola fraca sai devagar e o pe, mais rapido, entrava nela) - nos gestos
    // de frente com a bola de verdade (a imaginada a frente fica de fora); a
    // sola fica em cima dela de proposito. Ao longo do empurrao: na faixa da
    // bola (de lado e de altura), a frente do pe nao passa da parte de tras
    // dela (o pe a empurra). (A trava pela distancia caixa-bola soltava
    // quando a chave do pe ja tinha passado a bola: o pe saltava 30 cm.)
    if (g.touched && input.ballValid && g.surface != BallTouchSurface3D::Sole && g.style == Style::Front
        && !g.virtualFront && !g.imagined) {
        const Vec3 ballNext = input.ballPosition + input.ballVelocity * dt;
        const Vec3 footPosition = ankle - key.foot.rotate(foot.ankleAnchor);
        const Quaternion box = (key.foot * foot.boxOrientation).normalized();
        const Vec3 center = footPosition + key.foot.rotate(foot.center);
        const Vec3 axes[3] = { box.rotate({ 1.0f, 0.0f, 0.0f }), box.rotate({ 0.0f, 1.0f, 0.0f }),
            box.rotate({ 0.0f, 0.0f, 1.0f }) };
        const Vec3 across = cross(up, g.push);
        const auto extent = [&](Vec3 direction) {
            return std::abs(dot(axes[0], direction)) * foot.half.x + std::abs(dot(axes[1], direction)) * foot.half.y
                + std::abs(dot(axes[2], direction)) * foot.half.z;
        };
        const Vec3 rel = center - ballNext;
        const bool inBand = std::abs(dot(rel, across)) < input.ballRadius + extent(across) - 0.02f
            && rel.z - extent(up) < input.ballRadius;
        // (So vindo de tras: a correcao engata pequena e cresce aos poucos;
        // muito alem - a bola ja ficou para tras do pe, o corpo mais rapido
        // que ela - nao puxa o pe de volta, e solta devagar.)
        const float wanted = inBand ? dot(rel, g.push) + extent(g.push) + input.ballRadius * 0.9f : 0.0f;
        // (A trava cresce no maximo 1,2 cm por tick: engatando inteira, o pe
        // que descia na volta para a faixa da bola recuava 8 cm num tick.)
        float correction = 0.0f;
        if (wanted > 0.0f && wanted <= g.behindBall + 0.08f) correction = std::min(wanted, g.behindBall + 0.012f);
        else if (wanted > 0.0f) correction = std::max(0.0f, g.behindBall - 0.03f);
        g.behindBall = correction;
        if (correction > 0.0f) ankle -= g.push * correction;
    } else {
        g.behindBall = 0.0f;
    }
    // As pernas nunca se cruzam (V3): em todo instante o pe que joga fica do
    // lado dele do corpo e longe do de apoio (suave perto do limite).
    ankle = keepOwnSide(g.side, ankle, g.midline, g.bodyForward, supportAnkle, supportWeight, true, supportGap);
    // O pe nunca dentro do chao.
    {
        const Vec3 footPosition = ankle - key.foot.rotate(foot.ankleAnchor);
        ankle.z += footGroundDeficit(g.side, footPosition, key.foot, input.groundHeight, 0.004f);
    }
    // O corpo nas chaves (Catmull-Rom; zero no comeco e no fim).
    const float times[5] = { 0.0f, g.windupAt, g.contactAt, g.followAt, g.endAt };
    const auto body = [&](float BodyKey::*field) {
        const float values[5] = { 0.0f, g.bodyWindup.*field, g.bodyContact.*field, g.bodyFollow.*field, 0.0f };
        return bodyCurve(times, values, 5, t);
    };
    const auto bodyVector = [&](Vec3 BodyKey::*field) {
        // (Constante antes da primeira e depois da ultima chave: o peso do
        // braco faz a mistura com a pose da locomocao.)
        const Vec3 a = g.bodyWindup.*field, b = g.bodyContact.*field, c = g.bodyFollow.*field;
        const float xs[5] = { a.x, a.x, b.x, c.x, c.x };
        const float ys[5] = { a.y, a.y, b.y, c.y, c.y };
        const float zs[5] = { a.z, a.z, b.z, c.z, c.z };
        return Vec3 { bodyCurve(times, xs, 5, t), bodyCurve(times, ys, 5, t), bodyCurve(times, zs, 5, t) };
    };
    BallGesturePose3D& pose = output.gesture;
    pose.active = true;
    pose.side = g.side;
    pose.weight = 1.0f;
    pose.ankle = ankle;
    pose.knee = key.knee;
    pose.foot = key.foot;
    pose.legWeight = legWeight;
    pose.holdSupport = true;
    pose.pelvisDrop = body(&BodyKey::pelvisDrop);
    pose.pelvisYaw = body(&BodyKey::pelvisYaw);
    pose.pelvisPitch = body(&BodyKey::pelvisPitch);
    pose.pelvisRoll = body(&BodyKey::pelvisRoll);
    {
        // Para o apoio: a fracao do caminho da pelve ate o tornozelo de apoio.
        const Vec3 pelvis = state.links.front().position;
        pose.pelvisShift = planar(supportAnkle - pelvis) * body(&BodyKey::pelvisToSupport)
            + g.push * body(&BodyKey::pelvisForward);
    }
    pose.trunkTwist = body(&BodyKey::trunkTwist);
    pose.trunkBend = body(&BodyKey::trunkBend);
    pose.trunkSide = body(&BodyKey::trunkSide);
    // Bracos: frente/fora/cima no rumo do corpo -> mundo; os alvos so nas
    // chaves (fora delas, o peso sai e o braco e o da locomocao).
    {
        const Vec3 forward = g.bodyForward;
        const Vec3 left = cross(up, forward);
        const float arms = body(&BodyKey::arms);
        const Vec3 kickArm = bodyVector(&BodyKey::kickArm);
        const Vec3 supportArm = bodyVector(&BodyKey::supportArm);
        pose.wrist[static_cast<std::size_t>(g.side)] = forward * kickArm.x + left * (kickArm.y * out) + up * kickArm.z;
        pose.wrist[static_cast<std::size_t>(1 - g.side)] = forward * supportArm.x - left * (supportArm.y * out)
            + up * supportArm.z;
        // (Os bracos entram em no minimo 0,12 s: no chute rapido o desenho
        // deles chegava em 0,05 s, a mao a 18 m/s. No toque correndo, so um
        // pouco: o balanco da corrida continua - inteiro, os bracos paravam
        // abertos a cada toque.)
        const float runningTouch = g.moving && g.kind != BallActionKind3D::Shot ? 0.35f : 1.0f;
        const float armsIn = arms * smooth(t / 0.12f) * runningTouch;
        pose.armWeight = { armsIn, armsIn };
    }
    // A cabeca olha a bola ate o contato; parado, fica no ponto dele depois (a
    // cabeca baixa sobre a bola, sem seguir a bola que voa); andando, segue a
    // bola (ela vai a frente) e o peso sai logo.
    if (input.ballValid) {
        // (O gesto imaginado olha a bola dele.)
        const Vec3 target = g.imagined || (g.touched && (!g.moving || g.kind == BallActionKind3D::Shot))
            ? g.ballAtContact : input.ballPosition;
        // (O ponto suavizado: a bola passando perto do corpo varria o olhar
        // rapido demais; e a bola ja nos pes, a cabeca nao vai tao para baixo.)
        // (No comeco, de onde a cabeca ja olhava - o preparo da espera ou o
        // gesto cortado -, sem saltar.)
        if (g.time <= dt * 1.5f) m_lookPoint = m_carry.active && m_carry.lookWeight > 0.001f ? m_carry.look : target;
        m_lookPoint += (target - m_lookPoint) * (1.0f - std::exp(-dt / 0.07f));
        pose.look = m_lookPoint;
        const float near = planar(target - state.links.front().position).length();
        // (Andando, depois do toque ele volta a olhar para a frente logo.)
        const float hold = g.moving ? 0.0f : 0.10f;
        const float release = g.moving ? 0.15f : 0.25f;
        pose.lookWeight = 0.75f * (1.0f - smooth((t - g.contactAt - hold) / release)) * smooth(t / 0.08f)
            * (g.moving ? 0.4f + 0.6f * smooth((near - 0.25f) / 0.35f) : 1.0f);
    }
    // O gesto anterior cortado por este (V3): a pose dele sai suave - corpo,
    // bracos e cabeca vao dela ate a deste; a perna dele volta do alvo dele
    // (a outra) ou e o comeco desta (a mesma).
    if (m_carry.active && m_carryTime < BlendOldSeconds) {
        const float w = 1.0f - smooth(m_carryTime / BlendOldSeconds);
        // (Para onde a cabeca olhava anda com o corpo: parado no mundo, com
        // ele correndo, o ponto passava para baixo e para tras dela.)
        m_carry.look += bodyVelocityOf(input) * dt;
        const auto mix = [w](float now, float old) { return now + (old - now) * w; };
        pose.pelvisDrop = mix(pose.pelvisDrop, m_carry.pelvisDrop);
        pose.pelvisShift = pose.pelvisShift + (m_carry.pelvisShift - pose.pelvisShift) * w;
        pose.pelvisYaw = mix(pose.pelvisYaw, m_carry.pelvisYaw);
        pose.pelvisPitch = mix(pose.pelvisPitch, m_carry.pelvisPitch);
        pose.pelvisRoll = mix(pose.pelvisRoll, m_carry.pelvisRoll);
        pose.trunkTwist = mix(pose.trunkTwist, m_carry.trunkTwist);
        pose.trunkBend = mix(pose.trunkBend, m_carry.trunkBend);
        pose.trunkSide = mix(pose.trunkSide, m_carry.trunkSide);
        for (std::size_t k = 0; k < 2; ++k) {
            const float now = pose.armWeight[k], old = m_carry.armWeight[k] * w;
            const float total = now + old * (1.0f - now);
            if (total > 1e-4f) pose.wrist[k] = m_carry.wrist[k] + (pose.wrist[k] - m_carry.wrist[k]) * (now / total);
            pose.armWeight[k] = total;
        }
        if (m_carry.lookWeight > 0.001f) {
            const float now = pose.lookWeight, old = m_carry.lookWeight * w;
            const float total = now + old * (1.0f - now);
            if (total > 1e-4f) pose.look = m_carry.look + (pose.look - m_carry.look) * (now / total);
            pose.lookWeight = total;
        }
        if (m_carry.side != g.side && m_carry.legWeight > 0.001f) {
            pose.releaseSide = m_carry.side;
            pose.releaseAnkle = m_carry.ankle;
            pose.releaseKnee = m_carry.knee;
            pose.releaseFoot = m_carry.foot;
            pose.releaseWeight = m_carry.legWeight * w;
        } else if (m_carry.side == g.side) {
            pose.legWeight = std::max(pose.legWeight, m_carry.legWeight * w);
            legWeight = pose.legWeight;
        }
    }
    // As pernas nunca se cruzam (V3): a de apoio no ar (a passada cruzada de
    // lado) fica do lado dela da que joga; plantada, quem cede e a que joga
    // (acima). A separacao entra e sai suave (sem saltar o pe que pousa).
    {
        const bool supportAirborne = !input.footPlanted[static_cast<std::size_t>(1 - g.side)];
        const float target = supportAirborne ? 1.0f : 0.0f;
        const float rate = supportAirborne ? dt / 0.05f : dt / 0.15f;
        m_separate += std::clamp(target - m_separate, -rate, rate);
        pose.separateGap = supportGap;
        pose.separateForward = g.bodyForward;
        pose.separateWeight = smooth(m_separate) * legWeight;
    }
    m_lastPose = pose;
    if (std::getenv("XBALLTRACE"))
        std::printf("TRACO t %.3f pe %d peso %.2f tornozelo (%.2f %.2f %.2f) chave sw %.2f lat %.2f len %.2f | tempos %.2f %.2f %.2f %.2f"
            " | carrega %.2f sep %.2f solta %d | quadril (%.2f %.2f %.2f) pelve (%.2f %.2f) linha (%.2f %.2f)\n", t, g.side,
            legWeight, ankle.x, ankle.y, ankle.z, key.swing, key.lateral, key.length, g.windupAt, g.contactAt, g.followAt,
            g.endAt, m_carryTime, pose.separateWeight, pose.releaseSide, hip.x, hip.y, hip.z,
            state.links.front().position.x, state.links.front().position.y, g.midline.x, g.midline.y);
    // Modo fisico: o pe (origem do link) pelo tornozelo. Se o pe de verdade
    // ficou longe demais do alvo (o corpo nao acompanhou o gesto: tropeco,
    // freada), o gesto solta a perna para a passada - puxada para um alvo a
    // mais de um metro, ela subia atras do corpo e o derrubava.
    Vec3 footPosition = ankle - key.foot.rotate(foot.ankleAnchor);
    float follow = 1.0f;
    if (!input.animatedBody) {
        const Vec3 realFoot = state.links[foot.link].position;
        const float error = (footPosition - realFoot).length();
        m_physicalTrust = std::min(m_physicalTrust, 1.0f - smooth((error - 0.6f) / 0.4f));
        follow = m_physicalTrust;
        // (O alvo nunca muito a frente do pe de verdade: a perna e conduzida,
        // nao arrancada - a batida andando corria 0,5-0,7 m a frente dele e
        // o corpo tropecava. A forca nao depende de o pe chegar na bola.)
        if (error > PhysicalReach) footPosition = realFoot + (footPosition - realFoot) * (PhysicalReach / error);
    }
    output.foot.active = legWeight * follow > 0.001f;
    output.foot.side = g.side;
    output.foot.position = footPosition;
    output.foot.orientation = key.foot;
    output.foot.weight = legWeight * follow;
    // Modo fisico: o planejador segue o pe dirigido (tira do chao e o leva
    // pelo gesto). Parado, no fim ele pousa onde estava; andando - ou o corpo
    // saindo a andar no meio do gesto parado (pousando onde estava, o pe
    // ficava para tras e ele tropecava) -, depois do acompanhamento ele volta
    // para a passada (sem roteiro).
    const bool walkingNow = g.moving || bodyVelocityOf(input).length() > 0.6f
        || planar(input.requestedVelocity).length() > 0.1f || !input.standing;
    if (!input.animatedBody && follow > 0.5f && (!walkingNow || t < g.followAt + 0.04f)) {
        output.script.active = true;
        output.script.side = g.side;
        // (Pousa so com o pe de verdade ja descido perto do ponto: mandado
        // pousar ainda alto, o planejador e a locomocao discordavam do pe
        // apoiado - o contrato dos pes quebrava no sweepbolaestresse.)
        const Vec3 landing = g.startAnkleWorld - g.startFootWorld.rotate(foot.ankleAnchor);
        const Vec3 realFoot = state.links[foot.link].position;
        g.footDown = realFoot.z < landing.z + 0.05f && planar(realFoot - landing).length() < 0.12f;
        const bool swing = walkingNow ? true : t < g.endAt - 0.06f || !g.footDown;
        output.script.swing = swing;
        output.script.position = swing ? footPosition : landing;
        output.script.orientation = swing ? key.foot : g.startFootWorld;
        output.script.supportShift = walkingNow ? 0.0f : 0.35f;
    }
    output.repel.exemptSide = g.side;
    // Chute correndo: o corpo freia um pouco (a passada desacelera junto e o
    // pe que chutou pousa depois do acompanhamento, em vez de a passada
    // pousa-lo no meio dele).
    if (g.moving && g.kind == BallActionKind3D::Shot) {
        const float slow = std::clamp(m_force.kickRunSlowdown, 0.0f, 0.9f) * (0.6f + 0.4f * g.power);
        const float in = smooth(t / 0.08f), fade = 1.0f - smooth((t - g.followAt) / std::max(0.05f, g.endAt - g.followAt));
        output.speedScale = 1.0f - slow * std::min(in, fade);
    }
    // Fases (pelo tempo do gesto).
    m_phase = t < g.windupAt ? BallControlPhase3D::Windup
        : t < g.contactAt ? BallControlPhase3D::Reaching : BallControlPhase3D::FollowThrough;
    // Telemetria: o pe (a caixa de verdade, do corpo) ate a bola.
    if (input.ballValid) {
        const PhysicsBodyState3D& footBody = state.links[foot.link];
        output.footGap = boxGap(g.side, footBody.position, footBody.orientation, input.ballPosition,
            input.ballRadius, 0.0f);
    }
    // O reforco de estabilidade (modo fisico) entra suave no comeco.
    m_stability = std::max(m_stability, smooth(t / std::max(0.01f, StabilityIn)));
}

float BallControl3D::hitboxMargin(const BallControlInput3D& input) const {
    return std::max(0.0f, m_force.hitboxMargin) * (input.animatedBody ? 1.0f : PhysicalHitboxScale);
}

bool BallControl3D::hitboxTouch(const BallControlInput3D& input, const BallControlOutput3D& output) {
    const Gesture& g = m_gesture;
    const FootGeometry& f = m_feet[static_cast<std::size_t>(g.side)];
    const float dt = std::max(1e-4f, input.deltaTime);
    // O pe agora: no corpo animado, o do gesto neste tick (o que vai ser
    // desenhado); no fisico, o de verdade (o alvo vai a frente dele).
    Vec3 center;
    Quaternion orientation;
    const BallGesturePose3D& pose = output.gesture;
    if (input.animatedBody && pose.active && pose.legWeight > 0.5f) {
        const Vec3 footPosition = pose.ankle - pose.foot.rotate(f.ankleAnchor);
        center = footPosition + pose.foot.rotate(f.center);
        orientation = (pose.foot * f.boxOrientation).normalized();
    } else {
        const PhysicsBodyState3D& body = input.state->links[f.link];
        center = body.position + body.orientation.rotate(f.center);
        orientation = (body.orientation * f.boxOrientation).normalized();
    }
    const float margin = hitboxMargin(input);
    const Vec3 half = f.half + Vec3 { margin, margin, margin };
    const Vec3 ball = input.ballPosition;
    {
        const Vec3 local = orientation.conjugate().rotate(ball - center);
        const Vec3 clamped { std::clamp(local.x, -f.half.x, f.half.x), std::clamp(local.y, -f.half.y, f.half.y),
            std::clamp(local.z, -f.half.z, f.half.z) };
        m_hitGap = (local - clamped).length() - input.ballRadius;
    }
    if (!m_hitPrevious) {
        m_hitCenter = center;
        m_hitOrientation = orientation;
        m_hitBall = ball - input.ballVelocity * dt;
    }
    // Vindo para a bola: o pe em relacao a ela anda no rumo do empurrao ou na
    // direcao dela (no armado o pe vai para tras e nao conta; a bola rolando
    // contra o pe parado conta).
    const Vec3 relative = (center - m_hitCenter) / dt - input.ballVelocity;
    const bool approaching = dot(relative, g.push) > 0.0f || dot(relative, ball - center) > 0.0f;
    bool hit = false;
    for (int k = 1; k <= HitboxSweepSteps && !hit; ++k) {
        const float u = static_cast<float>(k) / static_cast<float>(HitboxSweepSteps);
        const Vec3 c = m_hitCenter + (center - m_hitCenter) * u;
        const Quaternion q = slerpQuat(m_hitOrientation, orientation, u);
        const Vec3 b = m_hitBall + (ball - m_hitBall) * u;
        const Vec3 local = q.conjugate().rotate(b - c);
        const Vec3 clamped { std::clamp(local.x, -half.x, half.x), std::clamp(local.y, -half.y, half.y),
            std::clamp(local.z, -half.z, half.z) };
        hit = (local - clamped).length() <= input.ballRadius;
    }
    m_hitPrevious = true;
    m_hitCenter = center;
    m_hitOrientation = orientation;
    m_hitBall = ball;
    return hit && approaching;
}

void BallControl3D::applyForce(const BallControlInput3D& input, BallControlOutput3D& output) {
    Vec3 velocity, spin;
    if (m_kind == BallActionKind3D::Shot) shotMotion(velocity, spin);
    else touchMotion(input, velocity, spin);
    output.touched = true;
    output.setMotion = true;
    output.ballVelocity = velocity;
    output.ballSpin = spin;
    m_touchVelocity = velocity;
    m_touchSpin = spin;
    m_sinceTouch = 0.0f;
    m_requestCooldown = RequestCooldown;
    m_forced = true;
    // (Um pedido guardado durante esta acao: o tempo guardado dele conta
    // daqui. Com a bola lancada para longe por esta forca - chute, passe -, a
    // saida da area nao dispara a garantia dele: a forca seguinte trocava o
    // chute por um toque. A velocidade de lancamento e em relacao a que o
    // corpo vai ter - a que o toque leva: arrancando, o toque parecia um
    // chute.)
    m_launchSpeed = planar(velocity - carryVelocity(input)).length();
    if (m_pending) m_pendingSeconds = 0.0f;
    if (m_waiting) {
        // (O gesto ainda nao comecou - andando, esperando o pe: ele vai onde
        // a bola estaria sem a forca; a imaginada fica.)
        if (!m_imagined) {
            m_virtualBall = true;
            m_virtualPosition = input.ballPosition;
            m_virtualVelocity = input.ballVelocity;
            m_virtualClock = 0.0f;
        }
    } else if (m_gesture.active) {
        m_gesture.touched = true;
    }
    if (std::getenv("XBALLPLAN")) {
        const Vec3 body = bodyVelocityOf(input);
        std::printf("FORCA acao %.3f s (fator %.2f, tempo %.3f) gesto %.3f contato %.3f espera %d garantia %d imaginado %d"
            " | corpo v (%.2f %.2f) pedida (%.2f %.2f) bola v %.2f\n", m_actionClock, m_timingFactor, m_forceAt,
            m_gesture.active ? m_gesture.time : -1.0f, m_gesture.active ? m_gesture.contactAt : -1.0f, m_waiting ? 1 : 0,
            m_actionCommitted ? 1 : 0, (m_gesture.active && m_gesture.imagined) || m_imagined ? 1 : 0, body.x, body.y,
            input.requestedVelocity.x, input.requestedVelocity.y, velocity.length());
    }
}

void BallControl3D::touchMotion(const BallControlInput3D& input, Vec3& velocity, Vec3& spin) const {
    const Vec3 bodyVelocity = bodyVelocityOf(input);
    const bool sole = m_surface == BallTouchSurface3D::Sole;
    velocity = m_touchSpeed >= 0.0f ? m_direction * m_touchSpeed
        : dribbleTouchVelocity(m_direction, m_power, bodyVelocity, carryVelocity(input), input.ballVelocity, sole,
            m_force);
    spin = cross(Vec3 { 0.0f, 0.0f, 1.0f }, planar(velocity)) * (1.0f / std::max(0.05f, input.ballRadius));
}

void BallControl3D::shotMotion(Vec3& velocity, Vec3& spin) const {
    const BallForceSettings3D& f = m_force;
    const float bar = std::clamp(m_power, 0.0f, 1.0f);
    const float base = f.shotElevationMinimum + f.shotElevationPower * bar;
    const float elevation = (m_lift >= 0.0f ? base + (f.shotHighElevation - base) * m_lift
        : base + (f.shotLowElevation - base) * -m_lift) * Pi / 180.0f;
    const float speed = f.shotMinimum + f.shotPower * std::pow(bar, std::max(0.05f, f.shotCurve));
    // Curva para a direita sai um pouco para a esquerda (e o contrario).
    const float yaw = m_lookYaw + m_curve * f.shotCurveLaunch * Pi / 180.0f;
    const Vec3 ahead { std::cos(yaw), std::sin(yaw), 0.0f };
    velocity = keepBallMotion(ahead * (speed * std::cos(elevation)) + Vec3 { 0.0f, 0.0f, speed * std::sin(elevation) },
        m_ballVelocityNow, f.shotSideKeep, f.shotBounce, f.shotAheadKeep);
    // Giro: de lado (Magnus para o lado: giro -z leva para a direita) e por
    // cima (cima x frente e o giro de quem rola para a frente).
    const Vec3 topAxis = cross(Vec3 { 0.0f, 0.0f, 1.0f }, ahead);
    const float top = m_lift >= 0.0f ? -f.shotBackSpin * m_lift : f.shotTopSpin * -m_lift;
    spin = Vec3 { 0.0f, 0.0f, -f.shotCurveSpin * m_curve } + topAxis * top;
}

void BallControl3D::update(const BallControlInput3D& input, BallControlOutput3D& output) {
    output = {};
    m_force = input.force != nullptr ? *input.force : defaultForce();
    m_ballVelocityNow = input.ballValid ? input.ballVelocity : Vec3 {};
    if (input.profile == nullptr || input.state == nullptr || input.state->links.empty()) return;
    if (!m_resolved) resolve(*input.profile);
    if (!m_feet[0].valid || !m_feet[1].valid) return;
    const float dt = std::max(1e-4f, input.deltaTime);
    const RagdollState3D& state = *input.state;
    const Vec3 pelvis = state.links.front().position;
    m_requestCooldown = std::max(0.0f, m_requestCooldown - dt);
    m_timingFactor = std::clamp(input.contactTimingFactor >= 0.0f ? input.contactTimingFactor : ContactTimingDefault,
        0.0f, 1.0f);
    // O contato pela hitbox do pe (so a forca, sem gesto, fica de fora: nao
    // ha pe).
    m_hitbox = input.hitboxContact && !input.forceOnly;
    output.hitboxMargin = m_hitbox ? hitboxMargin(input) : 0.0f;

    // A passada medida: quanto dura o apoio e o balanco de cada pe e ha
    // quanto tempo ele esta na fase atual.
    // (So andando: parado, o "apoio" e o tempo todo em pe.)
    const bool striding = bodyVelocityOf(input).length() > 1.0f && !input.standing;
    for (std::size_t side = 0; side < 2; ++side) {
        m_footClock[side] += dt;
        if (input.footPlanted[side] != m_lastPlanted[side]) {
            const float spent = m_footClock[side];
            if (striding && m_lastPlanted[side] && spent < 0.4f)
                m_stanceSeconds[side] += (std::clamp(spent, 0.08f, 0.3f) - m_stanceSeconds[side]) * 0.5f;
            else if (striding && !m_lastPlanted[side] && spent < 0.8f)
                m_swingSeconds[side] += (std::clamp(spent, 0.2f, 0.6f) - m_swingSeconds[side]) * 0.5f;
            m_footClock[side] = 0.0f;
        }
    }
    // Repulsao dos pes pela bola perto do corpo (o pe que joga fica de fora).
    if (input.ballValid && planar(input.ballPosition - pelvis).length() < 2.0f) {
        output.repel.active = true;
        output.repel.position = input.ballPosition;
        output.repel.radius = input.ballRadius;
    }
    // Logo depois do toque: o pe ainda empurrando nao muda a bola.
    m_sinceTouch += dt;
    if (m_sinceTouch < m_force.holdSeconds && input.ballValid) {
        // (Mais rapida ou desviada: o pe fisico ainda encostando mudava a
        // direcao do chute.)
        const float planned = m_touchVelocity.length();
        if (input.ballVelocity.length() > planned * 1.08f + 0.2f
            || (input.ballVelocity - m_touchVelocity).length() > planned * 0.12f + 0.3f) {
            output.setMotion = true;
            output.ballVelocity = m_touchVelocity;
            output.ballSpin = m_touchSpin;
        }
    }
    // Os relogios da acao (V3).
    if (m_action) m_actionClock += dt;
    if (m_virtualBall) m_virtualClock += dt;
    m_carryTime += dt;
    // A garantia: a bola de verdade na area agora, ou ha pouco (a previsao da
    // saida errou por pouco). A acao sem a forca com a bola na area passa a
    // ser garantida.
    const bool inside = input.ballValid && inArea(input.ballPosition, input.ballRadius, pelvis);
    m_sinceInArea = inside ? 0.0f : m_sinceInArea + dt;
    const bool reachable = input.ballValid && (inside || m_sinceInArea <= GuaranteeGrace);
    if (m_action && !m_forced && inside && !m_hitbox) m_actionCommitted = true;
    // Sem gesto ligado com um gesto ou a espera do pe em curso: acabam ja (a
    // forca que faltava sai).
    if (input.forceOnly && (m_gesture.active || m_waiting)) {
        if (m_action && !m_forced && reachable) applyForce(input, output);
        m_gesture.active = false;
        endAction();
    }

    // O pedido (V3): guardado ate RequestBuffer s; a cada tick, se da para
    // comecar (o gesto, ou andando a espera do pe) - senao, descartado no fim
    // (com a bola na area, a garantia: nunca). Um pedido novo substitui o
    // guardado.
    if (input.request != BallActionKind3D::None) {
        Request request;
        request.kind = input.request;
        request.power = std::clamp(input.power, 0.0f, 1.0f);
        Vec3 d = planar(input.direction);
        if (d.lengthSquared() < 1e-6f) d = { std::cos(input.lookYawRadians), std::sin(input.lookYawRadians), 0.0f };
        request.direction = d.normalized();
        request.lookYaw = input.lookYawRadians;
        const bool shot = request.kind == BallActionKind3D::Shot;
        request.lift = shot ? std::clamp(input.shotLift, -1.0f, 1.0f) : 0.0f;
        request.curve = shot ? std::clamp(input.shotCurve, -1.0f, 1.0f) : 0.0f;
        request.touchSpeed = input.touchSpeed;
        m_request = request;
        m_pending = true;
        m_pendingSeconds = 0.0f;
        m_requestCommitted = false;
    }
    if (m_pending) {
        // A garantia (06/10 noite): o pedido que viu a bola na area sai - pelo
        // caminho normal quando der; com a bola saindo da area (ou ja fora,
        // por pouco) ou no fim do tempo guardado, ja (startCommitted).
        // (Com a hitbox, sem garantia: o pedido que nao alcanca a bola com um
        // gesto e descartado - no fim do tempo guardado, ainda o encontro
        // mais tarde que o gesto aguenta, com o armado esperando a bola.)
        if (inside && !m_hitbox) m_requestCommitted = true;
        bool started = tryStart(input);
        const bool launched = m_sinceTouch < RequestCooldown && m_launchSpeed > LaunchSpeed;
        if (started) {
            m_actionCommitted = m_requestCommitted;
        } else if (m_hitbox && inside && !launched && m_pendingSeconds + dt > RequestBuffer && !busy()
            && bodyHeightShare(input) >= 0.85f) {
            takeRequest();
            const bool moving = !input.standing && bodyVelocityOf(input).length() > 0.6f;
            started = moving ? startMoving(input, false) : startStanding(input, true, false);
        } else if (m_requestCommitted && reachable && !launched) {
            const bool leaving = !inside || areaExit(input, LeavingTicks * dt) > 0.0f;
            if (leaving || m_pendingSeconds >= RequestBuffer) started = startCommitted(input, leaving);
        }
        if (started) {
            m_pending = false;
            m_requestCommitted = false;
        } else {
            m_pendingSeconds += dt;
            if (m_pendingSeconds > RequestBuffer && !(m_requestCommitted && reachable)) {
                m_pending = false;
                m_requestCommitted = false;
                output.missReason = 1;
                if (std::getenv("XBALLPLAN")) std::printf("PEDIDO descartado (%.2f s guardado)\n", m_pendingSeconds);
            }
        }
    }
    // Andando: um pe saiu do chao (comeca um balanco novo) - o gesto vai com
    // ele; nao dando, espera o proximo; parou, o gesto parado.
    if (m_waiting) {
        m_waitSeconds += dt;
        int lifted = -1;
        for (int s = 0; s < 2; ++s) {
            const std::size_t i = static_cast<std::size_t>(s);
            if (m_lastPlanted[i] && !input.footPlanted[i]) lifted = s;
        }
        Gesture g;
        const auto standingPlan = [&]() {
            Gesture best;
            bool found = false;
            for (int side = 0; side < 2; ++side) {
                Gesture candidate;
                if (plan(input, side, false, -1.0f, candidate) && (!found || candidate.cost < best.cost)) {
                    best = candidate;
                    found = true;
                }
            }
            if (found) beginGesture(input, best);
            return found;
        };
        const bool liftedHere = lifted >= 0 && (m_waitSide < 0 || lifted == m_waitSide);
        // (Com a hitbox, qualquer pe que sair do chao e alcancar a bola serve:
        // o esperado pode dar um passo curto demais - o primeiro da arrancada
        // pousa em 0,06 s - e o proximo dele chegava com o corpo ja passado
        // da bola.)
        const bool liftedAny = lifted >= 0 && m_hitbox;
        if (std::getenv("XBALLPLAN") && lifted >= 0)
            std::printf("ESPERA pe %d saiu (esperando %d) forca %d parado %d\n", lifted, m_waitSide, m_forced ? 1 : 0,
                input.standing ? 1 : 0);
        // (A garantia: a forca ainda por sair - o gesto sai mesmo sem caber
        // na bola de verdade, e a espera nunca acaba sem ela.)
        const bool guaranteed = m_actionCommitted && !m_forced;
        const auto giveUp = [&]() {
            if (!m_forced) {
                if (guaranteed && reachable) applyForce(input, output);
                else output.missReason = 1;
            }
            endAction();
        };
        if (input.standing && (input.ballValid || m_forced)) {
            // Parou: o gesto parado (a perna mais limpa); com a forca ja dada
            // (ou garantida) e sem gesto que caiba, com a bola imaginada a
            // frente.
            if (!standingPlan() && (m_forced || guaranteed)) {
                imagineBallAhead(input, std::max(0, m_waitSide));
                m_imagined = !m_forced;
                standingPlan();
            }
            if (m_waiting) giveUp();
        } else if ((liftedHere || liftedAny) && input.ballValid && plan(input, lifted, true, -1.0f, g)) {
            beginGesture(input, g);
        } else if (liftedHere && (m_forced || guaranteed)) {
            // A forca ja saiu (ou e garantida) e o gesto planejado nao cabe
            // mais (a bola de antes ficou para tras): o gesto com ela
            // imaginada a frente.
            imagineBallAhead(input, lifted);
            m_imagined = !m_forced;
            if (plan(input, lifted, true, -1.0f, g)) beginGesture(input, g);
        } else if (m_waitSeconds > MovingWait || !input.ballValid) {
            giveUp();
        }
    }
    m_lastPlanted = input.footPlanted;
    m_phase = m_waiting || m_pending ? BallControlPhase3D::Waiting : BallControlPhase3D::Idle;
    // O corpo caindo (a pelve bem abaixo da altura de pe): o gesto e a acao
    // acabam ja - a queda e da fisica.
    if ((m_gesture.active || m_waiting) && bodyHeightShare(input) < 0.65f) {
        // (A forca garantida sai assim mesmo.)
        if (m_action && !m_forced && m_actionCommitted && reachable) applyForce(input, output);
        m_gesture.active = false;
        endAction();
    }
    if (m_gesture.active) evaluate(input, output);
    // Andando, esperando o pe sair do chao: o corpo ja se prepara (a
    // antecipacao) - a cabeca vai na bola, o tronco inclina um pouco sobre
    // ela e os bracos comecam a abrir; o gesto continua dali (sai suave).
    bool preparing = false;
    if (m_waiting && !m_gesture.active && input.ballValid) {
        preparing = true;
        m_prepare = std::min(1.0f, m_prepare + dt / 0.12f);
        const float w = smooth(m_prepare);
        Vec3 forward, left;
        bodyFrame(input, forward, left);
        const Vec3 up { 0.0f, 0.0f, 1.0f };
        BallGesturePose3D& pose = output.gesture;
        pose = {};
        pose.active = true;
        pose.side = std::max(0, m_waitSide);
        pose.weight = 1.0f;
        pose.legWeight = 0.0f;
        pose.holdSupport = false;
        pose.trunkBend = 0.07f * w;
        pose.pelvisDrop = 0.01f * w;
        const float out = pose.side == 0 ? 1.0f : -1.0f;
        pose.wrist[static_cast<std::size_t>(pose.side)] = forward * 0.02f + left * (0.24f * out) + up * -0.38f;
        pose.wrist[static_cast<std::size_t>(1 - pose.side)] = forward * 0.06f - left * (0.30f * out) + up * -0.30f;
        pose.armWeight = { 0.25f * w, 0.25f * w };
        // (Esperando com a bola imaginada, olha para ela.)
        Vec3 lookAt = input.ballPosition, lookVelocity = input.ballVelocity;
        if (m_imagined) ballNow(input, lookAt, lookVelocity);
        if (m_prepare <= dt / 0.12f * 1.5f) m_lookPoint = lookAt;
        // (O ponto anda com a bola e so a diferenca e suavizada: correndo, o
        // suavizado ficava meio metro atras dela, embaixo da cabeca.)
        m_lookPoint += planar(lookVelocity) * dt;
        m_lookPoint += (lookAt - m_lookPoint) * (1.0f - std::exp(-dt / 0.07f));
        pose.look = m_lookPoint;
        pose.lookWeight = 0.45f * w;
        m_lastPose = pose;
    } else {
        m_prepare = 0.0f;
    }
    // A forca pela hitbox (09/10): o pe do gesto encostando na bola, vindo
    // para ela, ate pouco depois do acompanhamento comecar (o gesto na bola
    // imaginada nunca). O rumo da bola e o do pedido.
    if (m_hitbox && !m_bareForce) {
        if (m_action && !m_forced && input.ballValid && m_gesture.active && !m_gesture.imagined) {
            const bool touch = hitboxTouch(input, output);
            // (A sola e o calcanhar comecam com o pe sobre a bola: so depois do
            // armado - a sola saia no primeiro tick, antes de o pe subir nela.)
            const bool pulling = m_gesture.surface == BallTouchSurface3D::Sole || m_gesture.surface == BallTouchSurface3D::Heel;
            const bool armed = !pulling || m_gesture.time >= m_gesture.windupAt;
            if (touch && armed && m_gesture.time <= m_gesture.followAt + ForceLate) {
                applyForce(input, output);
                output.footGap = m_hitGap;
            }
        }
    } else if (m_action && !m_forced && input.ballValid) {
        // A forca (V3, sem hitbox): na fracao do tempo estimado ate o
        // contato, com a bola dentro da area (fora dela, quando entrar) - na
        // espera do pe ou no gesto ate pouco depois do contato; garantida, no
        // gesto inteiro e tambem com a bola que acabou de sair.
        const bool window = m_bareForce || m_waiting
            || (m_gesture.active && (m_actionCommitted || m_gesture.time <= m_gesture.followAt + ForceLate));
        if (window && (inside || (m_actionCommitted && reachable))) {
            // No tempo do fator - ou ja, se a bola vai sair da area nos
            // proximos ticks (so dentro dela a forca sai).
            const bool leaving = !inside || areaExit(input, LeavingTicks * dt) > 0.0f;
            if (m_actionClock >= m_forceAt || leaving) applyForce(input, output);
        }
    }
    // (So a forca: a acao acaba nela.)
    if (m_bareForce) {
        if (!m_forced) output.missReason = 2;
        endAction();
    }
    // O fim do gesto (o da acao: andando, uma acao nova pode estar esperando
    // o pe enquanto o anterior acaba).
    // (No fisico, o gesto parado espera o pe pousar ate 0,3 s a mais.)
    const bool landed = input.animatedBody || m_gesture.moving || m_gesture.footDown
        || m_gesture.time >= m_gesture.endAt + 0.3f;
    if (m_gesture.active && m_gesture.time >= m_gesture.endAt && landed) {
        m_gesture.active = false;
        if (!m_waiting) {
            if (m_action && !m_forced) {
                // (Garantida: sai no fim, se ainda nao saiu.)
                if (m_actionCommitted && reachable) {
                    applyForce(input, output);
                } else {
                    output.missReason = 2;
                    if (std::getenv("XBALLPLAN"))
                        std::printf(m_hitbox ? "GESTO sem forca (a hitbox nao encostou na bola)\n"
                                             : "GESTO sem forca (a bola nao entrou na area)\n");
                }
            }
            endAction();
        }
        m_phase = m_waiting || m_pending ? BallControlPhase3D::Waiting : BallControlPhase3D::Idle;
    }
    // O reforco de estabilidade sai suave depois do gesto.
    if (!m_gesture.active) m_stability = std::max(0.0f, m_stability - dt / std::max(0.01f, StabilityOut));
    output.stability = smooth(m_stability);
    if (!m_gesture.active && !preparing) m_lastPose = {};
    output.phase = m_phase;
    output.kind = m_kind;
    output.surface = m_surface;
    output.side = m_gesture.active ? m_gesture.side : m_waiting ? m_waitSide : -1;
    output.pending = m_pending;
    if (m_gesture.active && !m_gesture.touched) {
        output.contactValid = true;
        output.contactPoint = m_gesture.ballAtContact;
    } else if (m_waiting && !m_forced && m_waitContact > 0.0f) {
        output.contactValid = true;
        output.contactPoint = predictBall(input, std::max(0.0f, m_waitContact - m_waitSeconds));
    }
    output.secondsToForce = m_action && !m_forced ? std::max(0.0f, m_forceAt - m_actionClock) : -1.0f;
    // (Com a hitbox, o tempo ate o contato planejado.)
    if (m_hitbox && m_action && !m_forced)
        output.secondsToForce = m_gesture.active ? std::max(0.0f, m_gesture.contactAt - m_gesture.time)
            : m_waiting && m_waitContact > 0.0f ? std::max(0.0f, m_waitContact - m_waitSeconds) : 0.0f;
}

} // namespace MatterEngine
