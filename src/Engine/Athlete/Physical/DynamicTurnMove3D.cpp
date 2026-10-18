#include "Engine/Athlete/Physical/DynamicTurnMove3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

constexpr float Pi = 3.14159265359f;

float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }
float smooth(float v) {
    v = saturate(v);
    return v * v * (3.0f - 2.0f * v);
}
// 0 antes de a, 1 depois de b.
float ramp(float t, float a, float b) { return b > a ? smooth((t - a) / (b - a)) : (t >= b ? 1.0f : 0.0f); }
Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }
float length2(Vec3 v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// O quanto os pesos acompanham o alvo (s): a manobra sobe e desce sem salto
// mesmo quando o pedido muda no meio ou ela aborta.
constexpr float FollowSeconds = 0.04f;

} // namespace

void DynamicTurnMove3D::reset() {
    m_output = {};
    m_corrections = {};
    m_clock = 0.0f;
    m_persist = 0;
    m_smoothLean = m_smoothSide = m_smoothBrake = m_smoothFoot = m_smoothExit = 0.0f;
}

void DynamicTurnMove3D::start(const TurnMoveInput3D& input, float degrees, float side) {
    m_output.turnDegrees = degrees;
    m_output.side = side;
    m_output.entrySpeed = length2(input.bodyVelocity);
    m_output.elapsedSeconds = 0.0f;
    m_clock = 0.0f;
    m_output.state = TurnMoveState3D::Anticipate;
}

void DynamicTurnMove3D::update(const TurnMoveInput3D& input) {
    const TurnMoveSettings3D& s = m_settings;
    m_followBlend = 1.0f - std::exp(-std::max(1.0e-4f, input.deltaTime) / FollowSeconds);
    const Vec3 request = planar(input.requestedVelocity);
    const Vec3 body = planar(input.bodyVelocity);
    const float requestSpeed = length2(request);
    const float bodySpeed = length2(body);
    float degrees = 0.0f;
    float side = 0.0f;
    if (requestSpeed > 1.0e-3f && bodySpeed > 1.0e-3f) {
        const float cosine = std::clamp((request.x * body.x + request.y * body.y) / (requestSpeed * bodySpeed), -1.0f, 1.0f);
        degrees = std::acos(cosine) * 180.0f / Pi;
        side = (body.x * request.y - body.y * request.x) >= 0.0f ? 1.0f : -1.0f;
    }

    if (m_output.state == TurnMoveState3D::Idle) {
        const bool wants = s.amount > 0.0f && requestSpeed >= s.minRequestedSpeed && bodySpeed >= s.minBodySpeed
            && degrees >= s.triggerDegrees;
        m_persist = wants ? m_persist + 1 : 0;
        if (m_persist >= std::max(1, s.persistTicks)) {
            m_persist = 0;
            start(input, degrees, side);
        }
    } else {
        // O pedido ainda manda: o maior angulo visto e o lado se atualizam
        // (uma virada que continua girando, ou muda de lado).
        if (degrees > m_output.turnDegrees) {
            m_output.turnDegrees = degrees;
            m_output.entrySpeed = std::max(m_output.entrySpeed, bodySpeed);
        }
        if (degrees > s.releaseDegrees) m_output.side = side;
        m_clock += input.deltaTime;
        m_output.elapsedSeconds = m_clock;
        // Termina quando o corpo ja esta no rumo (ou o pedido caiu, ou o teto).
        const bool done = degrees < s.releaseDegrees || requestSpeed < s.abortRequestedSpeed
            || m_clock >= s.maximumSeconds;
        if (done) {
            m_output.state = TurnMoveState3D::Idle;
            m_clock = 0.0f;
        }
    }
    synthesize(degrees);
}

void DynamicTurnMove3D::synthesize(float degreesNow) {
    const TurnMoveSettings3D& s = m_settings;
    const bool running = m_output.state != TurnMoveState3D::Idle;
    float targetLean = 0.0f, targetBrake = 0.0f, targetFoot = 0.0f, targetExit = 0.0f, targetSide = 0.0f;
    float done = 0.0f;
    if (running) {
        const float c = m_clock;
        // O que falta virar (0..1: 1 = a virada inteira por fazer).
        const float left = ramp(degreesNow, s.emptyDegrees, s.fullDegrees);
        const float onset = ramp(c, 0.0f, s.onsetSeconds);
        done = m_output.turnDegrees > 1.0f ? saturate(1.0f - degreesNow / m_output.turnDegrees) : 1.0f;
        if (c < s.onsetSeconds) m_output.state = TurnMoveState3D::Anticipate;
        else if (done < 0.35f) m_output.state = TurnMoveState3D::Plant;
        else if (done < 0.7f) m_output.state = TurnMoveState3D::Cut;
        else m_output.state = TurnMoveState3D::Exit;
        // Agachar, encurtar a passada e a inclinacao: entram logo e valem
        // enquanto falta virar.
        targetLean = onset * left;
        // A inclinacao para o lado e o centro de massa para dentro: um pouco
        // depois (o pe de fora firma primeiro), e seguem enquanto falta virar.
        targetSide = ramp(c, 0.5f * s.onsetSeconds, 2.0f * s.onsetSeconds) * left;
        // A freada do plantio e o pe de fora: so no comeco, enquanto o
        // corpo ainda vai no rumo velho.
        const float early = 1.0f - ramp(done, 0.3f, 0.65f);
        targetBrake = onset * left * early;
        targetFoot = onset * left * early;
        // O empurrao da saida (para a frente): quando o corpo ja virou a maior parte.
        targetExit = ramp(done, 0.5f, 0.8f) * (1.0f - ramp(done, 0.97f, 1.0f)) * ramp(degreesNow, 4.0f, s.emptyDegrees + 8.0f);
    }
    m_corrections = {};
    m_smoothLean += (targetLean - m_smoothLean) * m_followBlend;
    m_smoothSide += (targetSide - m_smoothSide) * m_followBlend;
    m_smoothBrake += (targetBrake - m_smoothBrake) * m_followBlend;
    m_smoothFoot += (targetFoot - m_smoothFoot) * m_followBlend;
    m_smoothExit += (targetExit - m_smoothExit) * m_followBlend;
    const float peak = std::max({ m_smoothLean, m_smoothSide, m_smoothBrake, m_smoothFoot, m_smoothExit });
    m_output.weight = peak < 1.0e-3f ? 0.0f : peak;
    m_output.progress = running ? done : 0.0f;
    if (m_output.weight <= 0.0f) {
        m_smoothLean = m_smoothSide = m_smoothBrake = m_smoothFoot = m_smoothExit = 0.0f;
        m_output.speedScale = 1.0f;
        return;
    }
    const float v = m_output.entrySpeed;
    const float degrees = m_output.turnDegrees;
    const float side = m_output.side;
    // O quanto o angulo pesa: a 90 graus o valor cheio; menos nas viradas
    // suaves, e um pouco mais nas fortes.
    const float angle = std::clamp(degrees / 90.0f, 0.4f, 1.3f);
    const float amount = s.amount;
    const float lean = std::min(s.lateralLeanMaximum, (s.lateralLeanBase + s.lateralLeanPerSpeed * v) * angle);
    const float inside = std::min(s.comInsideMaximum, (s.comInsideBase + s.comInsidePerSpeed * v) * angle);
    const float crouch = std::min(s.crouchMaximum, (s.crouchBase + s.crouchPerSpeed * v) * angle);
    PhysicalCorrections3D& out = m_corrections;
    out.balance = true;
    out.footwork = true;
    out.lateralLean = side * lean * m_smoothSide * amount;
    out.forwardLean = (-s.brakeLean * m_smoothBrake + s.exitLean * m_smoothExit) * amount;
    out.comOffset = { 0.0f, side * inside * m_smoothSide * amount, 0.0f };
    out.pelvisHeight = -crouch * m_smoothLean * amount;
    out.cadenceOctaves = s.cadenceOctaves * m_smoothLean * amount;
    out.strideOctaves = s.strideOctaves * m_smoothLean * amount;
    // O pe de fora: o do lado oposto ao centro da curva (esquerda: o direito).
    const std::size_t outside = side > 0.0f ? 1 : 0;
    const float sine = std::sin(std::min(degrees, 150.0f) * Pi / 180.0f);
    out.feet[outside].touchdownOffset = { -s.outsideShort * m_smoothFoot * amount,
        -side * (s.outsideSideBase + s.outsideSidePerSin * sine) * m_smoothFoot * amount, 0.0f };
    // A freada do plantio: a velocidade no corte e a fracao da de entrada.
    const float cut = std::clamp(std::max(s.cutSpeedFraction, v > 1.0e-3f ? s.cutSpeedFloor / v : 1.0f), 0.0f, 1.0f);
    m_output.speedScale = 1.0f - (1.0f - cut) * m_smoothBrake * amount;
}

void addTurnMoveCorrections3D(PhysicalCorrections3D& target, const PhysicalCorrections3D& move) {
    if (!move.active()) return;
    target.balance = target.balance || move.balance;
    target.footwork = target.footwork || move.footwork;
    target.forwardLean += move.forwardLean;
    target.lateralLean += move.lateralLean;
    target.comOffset += move.comOffset;
    target.pelvisHeight += move.pelvisHeight;
    target.cadenceOctaves += move.cadenceOctaves;
    target.strideOctaves += move.strideOctaves;
    for (std::size_t side = 0; side < 2; ++side) target.feet[side].touchdownOffset += move.feet[side].touchdownOffset;
}

} // namespace MatterEngine
