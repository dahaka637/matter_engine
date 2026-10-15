#include "Engine/RL/Rewards/BodyReward3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

bool isFootPart(const RLBody3D& body, std::uint32_t link) {
    for (const auto& side : body.footParts)
        for (const std::uint32_t part : side)
            if (part == link) return true;
    return false;
}

} // namespace

void BodyCostAccumulator3D::begin(const RagdollState3D& state, const RLBody3D& body) {
    *this = {};
    m_headVertical = state.links[body.head].linearVelocity.z;
}

void BodyCostAccumulator3D::addSubstep(const RLBody3D& body, const RagdollState3D& state, float fixedStep,
    float bodyWeight) {
    const float weight = std::max(1.0f, bodyWeight);
    // O trabalho positivo, o uso do motor e os limites.
    float torque = 0.0f, limits = 0.0f, saturated = 0.0f;
    for (const RLBodyDof3D& dof : body.dofs) {
        const RagdollJointState3D& joint = state.joints[dof.linkIndex];
        const auto axis = static_cast<std::size_t>(dof.axis);
        const float tau = joint.motorTorqueNewtonMeters[axis];
        m_work += std::max(0.0f, tau * joint.velocityRadiansPerSecond[axis]) * fixedStep;
        const float cap = joint.motorTorqueLimitNewtonMeters[axis];
        if (cap > 0.0f) {
            torque += (tau / cap) * (tau / cap);
            if (std::abs(tau) >= 0.98f * cap) saturated += 1.0f;
        }
        const float range = std::max(0.01f, dof.maximumRadians - dof.minimumRadians);
        const float x = (joint.positionRadians[axis] - dof.minimumRadians) / range;
        if (x < 0.05f || x > 0.95f) limits += 1.0f;
    }
    const float count = static_cast<float>(std::max<std::size_t>(1, body.dofs.size()));
    m_torque += torque / count;
    m_limits += limits / count;
    m_saturation += saturated / count;
    // O escorregar no ponto de contato, pesado pela carga; a forca dos pes.
    double slipSum = 0.0, loadSum = 0.0;
    float sideForce[2] {};
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.otherMotion != RagdollContactMotion3D::Static) continue;
        if (!isFootPart(body, contact.linkIndex)) {
            if (contact.linkIndex < body.supportLink.size() && !body.supportLink[contact.linkIndex]) m_contacts = 1.0f;
            continue;
        }
        // A velocidade do ponto de contato pelo centro de massa do link (a
        // origem do calcanhar dividido nao e o centro de massa dele).
        const Vec3 point = rlLinkPointVelocity3D(body.profile.links[contact.linkIndex],
            state.links[contact.linkIndex], contact.position);
        const Vec3 tangent = point - contact.normal * dot(contact.normal, point);
        const double load = std::max(0.0f, contact.normalImpulseNewtonSeconds);
        slipSum += static_cast<double>(tangent.length()) * load;
        loadSum += load;
        for (std::size_t side = 0; side < 2; ++side)
            for (const std::uint32_t part : body.footParts[side])
                if (part == contact.linkIndex) sideForce[side] += contact.normalImpulseNewtonSeconds / fixedStep;
    }
    if (loadSum > 0.0) m_slip += slipSum / loadSum;
    for (const float force : sideForce) m_impact += std::max(0.0f, force / weight - 2.0f) * fixedStep;
    // A cabeca: o giro para os lados e para frente e tras, e o tranco vertical.
    const PhysicsBodyState3D& head = state.links[body.head];
    const float vertical = (head.linearVelocity.z - m_headVertical) / fixedStep;
    m_headVertical = head.linearVelocity.z;
    m_head += head.angularVelocity.x * head.angularVelocity.x + head.angularVelocity.y * head.angularVelocity.y
        + 0.0025f * vertical * vertical;
    ++m_substeps;
}

void BodyCostAccumulator3D::setTargetChange(std::span<const float> previousTargets, std::span<const float> previousChange,
    std::span<const float> targets) {
    // A reversao (ida e volta de uma decisao para a outra) pesa 4 vezes: e o
    // tremor; a mudanca sozinha e o movimento.
    float change = 0.0f, reversal = 0.0f;
    const std::size_t count = std::min({ previousTargets.size(), previousChange.size(), targets.size() });
    for (std::size_t d = 0; d < count; ++d) {
        const float delta = targets[d] - previousTargets[d];
        change += delta * delta;
        reversal += std::max(0.0f, -delta * previousChange[d]);
    }
    m_smooth = count > 0 ? (change + 4.0f * reversal) / static_cast<float>(count) : 0.0f;
}

BodyCosts3D BodyCostAccumulator3D::finish(float decisionSeconds, float bodyWeight) const {
    BodyCosts3D costs;
    const float substeps = static_cast<float>(std::max(1u, m_substeps));
    costs.work = static_cast<float>(m_work / (std::max(1.0f, bodyWeight) * std::max(1.0e-4f, decisionSeconds)));
    costs.torque = m_torque / substeps;
    costs.slip = static_cast<float>(m_slip / substeps);
    costs.impact = static_cast<float>(m_impact / std::max(1.0e-4f, decisionSeconds));
    costs.head = static_cast<float>(m_head / substeps);
    costs.smooth = m_smooth;
    costs.limits = m_limits / substeps;
    costs.saturation = m_saturation / substeps;
    costs.contacts = m_contacts;
    return costs;
}

namespace {

float pseudoHuber(float x, float delta) {
    return std::sqrt(x * x + delta * delta) - delta;
}

float smoothstep(float from, float to, float x) {
    if (!(to > from)) return x >= to ? 1.0f : 0.0f;
    const float t = std::clamp((x - from) / (to - from), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

float bodyVelocityScore3D(const BodyRewardSettings3D& settings, Vec3 measured, Vec3 wanted) {
    const float error = std::hypot(measured.x - wanted.x, measured.y - wanted.y);
    if (settings.velocityForm == "progress") {
        const float requested = std::hypot(wanted.x, wanted.y);
        if (requested < settings.movingThreshold) {
            const float u = std::hypot(measured.x, measured.y) / std::max(1.0e-3f, settings.stopSigma);
            return std::exp(-u * u);
        }
        const float delta = settings.progressDeltaBase + settings.progressDeltaPerSpeed * requested;
        const float still = pseudoHuber(requested, delta);
        return std::clamp((still - pseudoHuber(error, delta)) / std::max(1.0e-6f, still), -1.0f, 1.0f);
    }
    const float tolerance = settings.velocityToleranceAbsolute
        + settings.velocityToleranceRelative * std::hypot(wanted.x, wanted.y);
    const float u = error / std::max(1.0e-3f, tolerance);
    if (settings.velocityForm == "bell") return std::exp(-0.5f * u * u);
    const float cauchy = 1.0f / (1.0f + u * u);
    if (settings.velocityForm == "mixed")
        return 0.5f * cauchy + 0.5f * std::max(0.0f, 1.0f - error / (std::hypot(wanted.x, wanted.y) + 1.0f));
    return cauchy;
}

float bodyFacingScore3D(const BodyRewardSettings3D& settings, float chestHeading, float faceHeading) {
    const float error = std::atan2(std::sin(faceHeading - chestHeading), std::cos(faceHeading - chestHeading));
    const float sigma = std::max(1.0e-3f, settings.facingSigma);
    return std::exp(-(error * error) / (sigma * sigma));
}

float bodyStandGate3D(const BodyRewardSettings3D& settings, float pelvisHeightRatio, float uprightCosine) {
    const BodyRewardSettings3D::StandGate& gate = settings.standGate;
    if (!gate.enabled) return 1.0f;
    constexpr float Degrees = 3.14159265358979f / 180.0f;
    return smoothstep(gate.heightFrom, gate.heightTo, pelvisHeightRatio)
        * smoothstep(std::cos(gate.uprightFromDegrees * Degrees), std::cos(gate.uprightToDegrees * Degrees),
            uprightCosine);
}

float bodyAirCredit3D(const BodyRewardSettings3D& settings, bool moving, const bool (&contact)[2],
    const float (&seconds)[2]) {
    if (!(settings.air.weight > 0.0f) || !moving || contact[0] == contact[1]) return 0.0f;
    return std::min(settings.air.capSeconds, std::min(seconds[0], seconds[1]))
        / std::max(1.0e-3f, settings.air.capSeconds);
}

BodyTaskTerms3D bodyTaskTerms3D(const BodyRewardSettings3D& settings, Vec3 measured, Vec3 wanted, float facingScore,
    float standGate, float airCredit) {
    BodyTaskTerms3D terms;
    terms.velocityScore = bodyVelocityScore3D(settings, measured, wanted);
    terms.facingScore = facingScore;
    terms.standGate = standGate;
    terms.airCredit = airCredit;
    const bool moving = std::hypot(wanted.x, wanted.y) >= settings.movingThreshold;
    if (settings.velocityForm == "progress" && moving) {
        // O olhar e o estilo so pagam junto com o progresso: parado com pedido
        // de andar nao rende nada.
        terms.facingGate = std::max(0.0f, terms.velocityScore);
        terms.styleGate = standGate * std::max(0.0f, terms.velocityScore);
    } else {
        terms.styleGate = standGate;
    }
    return terms;
}

float bodyReward3D(const BodyRewardSettings3D& settings, const BodyTaskWeights3D& weights, float velocityScore,
    float facingScore, const BodyCosts3D& costs, bool fell, float decisionSeconds, float* components) {
    BodyTaskTerms3D terms;
    terms.velocityScore = velocityScore;
    terms.facingScore = facingScore;
    return bodyReward3D(settings, weights, terms, costs, fell, decisionSeconds, components);
}

float bodyReward3D(const BodyRewardSettings3D& settings, const BodyTaskWeights3D& weights,
    const BodyTaskTerms3D& task_, const BodyCosts3D& costs, bool fell, float decisionSeconds, float* components) {
    const float velocityScore = task_.velocityScore;
    const float facingScore = task_.facingScore;
    const float task = settings.taskScale * task_.standGate * (weights.velocity * velocityScore + weights.facing * facingScore * task_.facingGate);
    const float air = settings.air.weight * task_.standGate * task_.airCredit;
    const float terms[8] = { settings.work * costs.work, settings.torque * costs.torque, settings.slip * costs.slip,
        settings.impact * costs.impact, settings.head * costs.head, settings.smooth * costs.smooth,
        settings.limits * costs.limits, settings.contacts * costs.contacts };
    float cost = 0.0f;
    for (const float term : terms) cost += term;
    const float fall = fell ? settings.fall : 0.0f;
    if (components) {
        components[static_cast<std::size_t>(BodyRewardTerm3D::Velocity)] = velocityScore;
        components[static_cast<std::size_t>(BodyRewardTerm3D::Facing)] = facingScore;
        components[static_cast<std::size_t>(BodyRewardTerm3D::Task)] = task;
        for (std::size_t i = 0; i < 8; ++i) components[static_cast<std::size_t>(BodyRewardTerm3D::Work) + i] = terms[i];
        components[static_cast<std::size_t>(BodyRewardTerm3D::Fall)] = fall;
        components[static_cast<std::size_t>(BodyRewardTerm3D::StandGate)] = task_.standGate;
        components[static_cast<std::size_t>(BodyRewardTerm3D::StyleGate)] = task_.styleGate;
        components[static_cast<std::size_t>(BodyRewardTerm3D::Air)] = air;
    }
    const float rate = task + air - cost;
    return decisionSeconds * (settings.positiveOnly ? std::max(0.0f, rate) : rate) - fall;
}

} // namespace MatterEngine
