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
        const PhysicsBodyState3D& link = state.links[contact.linkIndex];
        const Vec3 point = link.linearVelocity + cross(link.angularVelocity, contact.position - link.position);
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

float bodyVelocityScore3D(const BodyRewardSettings3D& settings, Vec3 measured, Vec3 wanted) {
    const float error = std::hypot(measured.x - wanted.x, measured.y - wanted.y);
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

float bodyReward3D(const BodyRewardSettings3D& settings, const BodyTaskWeights3D& weights, float velocityScore,
    float facingScore, const BodyCosts3D& costs, bool fell, float decisionSeconds, float* components) {
    const float task = weights.velocity * velocityScore + weights.facing * facingScore;
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
    }
    return decisionSeconds * (task - cost) - fall;
}

} // namespace MatterEngine
