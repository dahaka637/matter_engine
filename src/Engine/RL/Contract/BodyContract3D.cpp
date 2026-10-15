#include "Engine/RL/Contract/BodyContract3D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace MatterEngine {
namespace {

constexpr std::uint32_t ObservationVersion = 2;

float gaussian(std::mt19937_64& rng, float sigma) {
    return sigma > 0.0f ? std::normal_distribution<float>(0.0f, sigma)(rng) : 0.0f;
}

void vec3(RLVectorLayout3D& layout, const std::string& name, const char* unit) {
    for (const char* axis : { ".x", ".y", ".z" }) layout.channels.push_back({ name + axis, unit });
}

float wrapAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

float jointValue(const RagdollState3D& ragdoll, const RLBodyDof3D& dof) {
    return ragdoll.joints[dof.linkIndex].positionRadians[static_cast<std::size_t>(dof.axis)];
}

} // namespace

StrengthGroup3D BodyContract3D::strengthGroup(std::string_view linkId) {
    for (const char* leg : { "Thigh", "Shin", "Foot", "Toe" })
        if (linkId.find(leg) != std::string_view::npos) return StrengthGroup3D::Legs;
    for (const char* arm : { "Arm", "Hand" })
        if (linkId.find(arm) != std::string_view::npos) return StrengthGroup3D::Arms;
    return StrengthGroup3D::Trunk;
}

BodyContract3D::BodyContract3D(const RLBody3D& body, BodyContractSettings3D settings)
    : m_body(body), m_settings(settings) {
    if (m_settings.substepsPerDecision == 0 || !(m_settings.decisionSeconds > 0.0f))
        throw std::runtime_error("contrato do Corpo: decisao sem passos de fisica");
    m_keyBodies = { m_body.head, m_body.leftHand, m_body.rightHand, m_body.leftFoot, m_body.rightFoot };
    for (const RLBodyDof3D& dof : m_body.dofs) m_dofGroup.push_back(strengthGroup(m_body.profile.links[dof.linkIndex].id));
    for (const RLBodyDof3D& dof : m_body.lockedDofs)
        m_lockedGroup.push_back(strengthGroup(m_body.profile.links[dof.linkIndex].id));

    RLVectorLayout3D& o = m_observation;
    o.id = "matter.rl.observation";
    o.version = ObservationVersion;
    // O proprio corpo.
    o.channels.push_back({ "pelvis.height_error", "m/0.3" });
    vec3(o, "pelvis.gravity_local", "unit");
    vec3(o, "pelvis.linear_velocity_heading", "m/s/2");
    vec3(o, "pelvis.angular_velocity_local", "rad/s/4");
    for (const RLBodyDof3D& dof : m_body.dofs) o.channels.push_back({ dof.name + ".position_from_base", "rad" });
    for (const RLBodyDof3D& dof : m_body.dofs) o.channels.push_back({ dof.name + ".velocity", "rad/s/10" });
    for (const std::uint32_t link : m_keyBodies) vec3(o, m_body.profile.links[link].id + ".position_heading", "m");
    vec3(o, "LeftFoot.gravity_local", "unit");
    vec3(o, "RightFoot.gravity_local", "unit");
    vec3(o, "com.position_heading", "m");
    vec3(o, "com.velocity_heading", "m/s/2");
    o.channels.push_back({ "com.support_offset.x", "m/0.2" });
    o.channels.push_back({ "com.support_offset.y", "m/0.2" });
    // O contato de cada lado nos passos de fisica da decisao.
    for (const char* side : { "LeftFoot", "RightFoot" }) {
        const std::string s = side;
        o.channels.push_back({ s + ".contact_fraction", "0..1" });
        o.channels.push_back({ s + ".force_mean", "peso" });
        o.channels.push_back({ s + ".force_peak", "peso" });
        o.channels.push_back({ s + ".touchdown", "0/1" });
        o.channels.push_back({ s + ".liftoff", "0/1" });
    }
    if (m_body.leftToe >= 0) o.channels.push_back({ "LeftToe.contact_fraction", "0..1" });
    if (m_body.rightToe >= 0) o.channels.push_back({ "RightToe.contact_fraction", "0..1" });
    for (const RLBodyDof3D& dof : m_body.dofs) o.channels.push_back({ dof.name + ".previous_action", "acao" });
    // Os comandos (passam crus pela normalizacao do treinador: "command.").
    o.channels.push_back({ "command.move.x", "m/s/6" });
    o.channels.push_back({ "command.move.y", "m/s/6" });
    o.channels.push_back({ "command.face.sin", "1" });
    o.channels.push_back({ "command.face.cos", "1" });
    o.channels.push_back({ "command.face.rate", "rad/s/2" });
    o.channels.push_back({ "command.jump.pending", "0/1" });
    o.channels.push_back({ "command.jump.rise", "m" });
    o.channels.push_back({ "command.jump.age", "s" });
    // Os dados do jogador (tambem crus: "player.").
    o.channels.push_back({ "player.height", "m/1.8" });
    o.channels.push_back({ "player.mass", "kg/80" });
    o.channels.push_back({ "player.strength.legs", "x" });
    o.channels.push_back({ "player.strength.trunk", "x" });
    o.channels.push_back({ "player.strength.arms", "x" });
    o.channels.push_back({ "player.cap.sprint", "m/s/6" });
    o.channels.push_back({ "player.cap.now", "m/s/6" });
    // A memoria curta: as decisoes anteriores, a mais nova primeiro.
    for (std::uint32_t k = 1; k <= m_settings.historyDecisions; ++k) {
        const std::string h = "history" + std::to_string(k) + ".";
        for (const RLBodyDof3D& dof : m_body.dofs) o.channels.push_back({ h + dof.name + ".position", "rad" });
        for (const RLBodyDof3D& dof : m_body.dofs) o.channels.push_back({ h + dof.name + ".velocity", "rad/s/10" });
        vec3(o, h + "pelvis.gravity_local", "unit");
        vec3(o, h + "pelvis.angular_velocity_local", "rad/s/4");
        for (const RLBodyDof3D& dof : m_body.dofs) o.channels.push_back({ h + dof.name + ".action", "acao" });
    }

    m_action.id = "matter.rl.action";
    m_action.version = ObservationVersion;
    for (const RLBodyDof3D& dof : m_body.dofs) m_action.channels.push_back({ dof.name + ".target", "acao" });
}

std::size_t BodyContract3D::frameSize() const {
    return 3 * m_body.dofs.size() + 6;
}

float BodyContract3D::targetOf(std::size_t dof, float action) const {
    return m_body.dofs[dof].target(action);
}

void BodyContract3D::reset(BodyContractState3D& state, const RagdollState3D& ragdoll) const {
    const std::size_t count = m_body.dofs.size();
    state.previousAction.assign(count, 0.0f);
    state.rampFrom.assign(count, 0.0f);
    state.rampTo.assign(count, 0.0f);
    state.velocityTarget.assign(count, 0.0f);
    state.previousMeanTarget.assign(count, 0.0f);
    for (std::size_t d = 0; d < count; ++d) {
        const RLBodyDof3D& dof = m_body.dofs[d];
        const float q = std::clamp(jointValue(ragdoll, dof), dof.minimumRadians, dof.maximumRadians);
        state.rampFrom[d] = state.rampTo[d] = state.previousMeanTarget[d] = q;
    }
    // O quadro de agora em toda a memoria (sem ruido, sem acao anterior).
    const PhysicsBodyState3D& pelvis = ragdoll.links.front();
    const Quaternion pelvisFrame =
        (pelvis.orientation * m_body.profile.links.front().modelOrientation.conjugate()).normalized();
    std::vector<float> frame;
    frame.reserve(frameSize());
    for (const RLBodyDof3D& dof : m_body.dofs) frame.push_back(jointValue(ragdoll, dof) - dof.baseRadians);
    for (const RLBodyDof3D& dof : m_body.dofs)
        frame.push_back(ragdoll.joints[dof.linkIndex].velocityRadiansPerSecond[static_cast<std::size_t>(dof.axis)] * 0.1f);
    const Vec3 down = pelvisFrame.conjugate().rotate({ 0.0f, 0.0f, -1.0f });
    const Vec3 spin = pelvisFrame.conjugate().rotate(pelvis.angularVelocity) * 0.25f;
    for (const float v : { down.x, down.y, down.z, spin.x, spin.y, spin.z }) frame.push_back(v);
    for (std::size_t d = 0; d < count; ++d) frame.push_back(0.0f);
    state.lastFrame = frame;
    state.history.clear();
    for (std::uint32_t k = 0; k < m_settings.historyDecisions; ++k)
        state.history.insert(state.history.end(), frame.begin(), frame.end());
    state.contactSteps = state.toeSteps = state.forceSum = state.forcePeak = {};
    state.touchdown = state.liftoff = {};
    state.contactSubsteps = 0;
    for (std::size_t side = 0; side < 2; ++side) {
        state.lastContact[side] = false;
        for (const RagdollContactPoint3D& contact : ragdoll.contacts) {
            if (contact.otherMotion != RagdollContactMotion3D::Static) continue;
            for (const std::uint32_t part : m_body.footParts[side])
                if (contact.linkIndex == part) state.lastContact[side] = true;
        }
    }
    state.started = false;
}

void BodyContract3D::accumulateContacts(BodyContractState3D& state, const RagdollState3D& ragdoll, float bodyWeight,
    float fixedStep) const {
    const float weight = std::max(1.0f, bodyWeight);
    const std::array<int, 2> toes { m_body.leftToe, m_body.rightToe };
    for (std::size_t side = 0; side < 2; ++side) {
        bool contact = false, toe = false;
        float force = 0.0f;
        for (const RagdollContactPoint3D& c : ragdoll.contacts) {
            if (c.otherMotion != RagdollContactMotion3D::Static) continue;
            bool mine = false;
            for (const std::uint32_t part : m_body.footParts[side]) mine = mine || c.linkIndex == part;
            if (!mine) continue;
            contact = true;
            if (toes[side] >= 0 && c.linkIndex == static_cast<std::uint32_t>(toes[side])) toe = true;
            force += c.normalImpulseNewtonSeconds / std::max(1.0e-6f, fixedStep);
        }
        if (contact) state.contactSteps[side] += 1.0f;
        if (toe) state.toeSteps[side] += 1.0f;
        state.forceSum[side] += force / weight;
        state.forcePeak[side] = std::max(state.forcePeak[side], force / weight);
        if (contact && !state.lastContact[side]) state.touchdown[side] = true;
        if (!contact && state.lastContact[side]) state.liftoff[side] = true;
        state.lastContact[side] = contact;
    }
    ++state.contactSubsteps;
}

void BodyContract3D::writeObservation(const RagdollState3D& ragdoll, const BodyCommand3D& command,
    const PlayerAttributes3D& player, BodyContractState3D& state, float* out, float noise, std::mt19937_64* rng) const {
    const RagdollProfile3D& profile = m_body.profile;
    const std::size_t count = m_body.dofs.size();
    if (state.previousAction.size() != count) throw std::runtime_error("contrato do Corpo: estado sem reset");
    const auto n = [&](float sigma) { return noise > 0.0f && rng ? gaussian(*rng, sigma * noise) : 0.0f; };
    const auto put3 = [&](float*& p, Vec3 v, float sigma) {
        *p++ = v.x + n(sigma);
        *p++ = v.y + n(sigma);
        *p++ = v.z + n(sigma);
    };

    const PhysicsBodyState3D& pelvis = ragdoll.links.front();
    const Quaternion pelvisFrame = (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
    const Quaternion headingInverse =
        Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -std::atan2(forward.y, forward.x));
    // O centro de massa de verdade: a posicao publicada e a da origem do corpo
    // (a velocidade ja e a do centro de massa; ver RLBody3D.hpp).
    const Vec3 com = rlBodyCenterOfMass3D(profile, ragdoll.links);
    const Vec3 comVelocity = rlBodyCenterOfMassVelocity3D(profile, ragdoll.links);

    float* p = out;
    *p++ = (pelvis.position.z - m_body.standingPelvisHeight) / 0.3f + n(0.005f);
    float* gravityAt = p;
    put3(p, pelvisFrame.conjugate().rotate({ 0.0f, 0.0f, -1.0f }), 0.02f);
    put3(p, headingInverse.rotate(pelvis.linearVelocity) * 0.5f, 0.02f);
    float* spinAt = p;
    put3(p, pelvisFrame.conjugate().rotate(pelvis.angularVelocity) * 0.25f, 0.02f);
    float* jointsAt = p;
    for (const RLBodyDof3D& dof : m_body.dofs) *p++ = jointValue(ragdoll, dof) - dof.baseRadians + n(0.01f);
    for (const RLBodyDof3D& dof : m_body.dofs)
        *p++ = ragdoll.joints[dof.linkIndex].velocityRadiansPerSecond[static_cast<std::size_t>(dof.axis)] * 0.1f + n(0.015f);
    for (const std::uint32_t link : m_keyBodies)
        put3(p, headingInverse.rotate(ragdoll.links[link].position - pelvis.position), 0.005f);
    std::array<Vec3, 2> footCenter {};
    for (std::size_t f = 0; f < 2; ++f) {
        const std::uint32_t foot = f == 0 ? m_body.leftFoot : m_body.rightFoot;
        const Quaternion frame = ragdoll.links[foot].orientation * profile.links[foot].modelOrientation.conjugate();
        put3(p, frame.conjugate().rotate({ 0.0f, 0.0f, -1.0f }), 0.02f);
        footCenter[f] = ragdoll.links[foot].position
            + ragdoll.links[foot].orientation.rotate(profile.links[foot].collider.localPosition);
    }
    put3(p, headingInverse.rotate(com - pelvis.position), 0.005f);
    put3(p, headingInverse.rotate(comVelocity) * 0.5f, 0.02f);
    const Vec3 support = (footCenter[0] + footCenter[1]) * 0.5f;
    const Vec3 offset = headingInverse.rotate(Vec3 { com.x - support.x, com.y - support.y, 0.0f });
    *p++ = offset.x * 5.0f + n(0.02f);
    *p++ = offset.y * 5.0f + n(0.02f);
    // O contato da decisao (desde a ultima observacao); depois zera.
    const float steps = static_cast<float>(std::max(1u, state.contactSubsteps));
    for (std::size_t side = 0; side < 2; ++side) {
        const bool any = state.contactSubsteps > 0;
        *p++ = any ? state.contactSteps[side] / steps : (state.lastContact[side] ? 1.0f : 0.0f);
        *p++ = std::min(3.0f, state.forceSum[side] / steps);
        *p++ = std::min(3.0f, state.forcePeak[side]);
        *p++ = state.touchdown[side] ? 1.0f : 0.0f;
        *p++ = state.liftoff[side] ? 1.0f : 0.0f;
    }
    if (m_body.leftToe >= 0) *p++ = state.toeSteps[0] / steps;
    if (m_body.rightToe >= 0) *p++ = state.toeSteps[1] / steps;
    state.contactSteps = state.toeSteps = state.forceSum = state.forcePeak = {};
    state.touchdown = state.liftoff = {};
    state.contactSubsteps = 0;
    float* actionAt = p;
    for (const float a : state.previousAction) *p++ = a;
    // Os comandos, no referencial do rumo; o olhar contra o rumo do peito.
    const Vec3 move = headingInverse.rotate(Vec3 { command.moveVelocity.x, command.moveVelocity.y, 0.0f });
    *p++ = move.x / 6.0f;
    *p++ = move.y / 6.0f;
    const Quaternion chest = (ragdoll.links[m_body.upperChest].orientation
        * profile.links[m_body.upperChest].modelOrientation.conjugate()).normalized();
    const Vec3 chestForward = chest.rotate({ 1.0f, 0.0f, 0.0f });
    const float faceError = wrapAngle(command.faceHeading - std::atan2(chestForward.y, chestForward.x));
    *p++ = std::sin(faceError);
    *p++ = std::cos(faceError);
    *p++ = command.faceRate / 2.0f;
    *p++ = command.jumpPending ? 1.0f : 0.0f;
    *p++ = command.jumpPending ? command.jumpRise : 0.0f;
    *p++ = command.jumpPending ? std::min(command.jumpAge, 2.0f) : 0.0f;
    *p++ = player.heightMeters / 1.8f;
    *p++ = player.massKg / 80.0f;
    for (const float s : player.strength) *p++ = s;
    *p++ = player.sprintCap / 6.0f;
    *p++ = player.capNow / 6.0f;
    std::copy(state.history.begin(), state.history.end(), p);
    p += state.history.size();
    if (static_cast<std::size_t>(p - out) != m_observation.size())
        throw std::runtime_error("contrato do Corpo: a entrada escrita nao bate com o layout");

    // O quadro desta observacao (o que a rede viu), para a memoria.
    state.lastFrame.assign(frameSize(), 0.0f);
    std::copy(jointsAt, jointsAt + 2 * count, state.lastFrame.begin());
    std::copy(gravityAt, gravityAt + 3, state.lastFrame.begin() + static_cast<std::ptrdiff_t>(2 * count));
    std::copy(spinAt, spinAt + 3, state.lastFrame.begin() + static_cast<std::ptrdiff_t>(2 * count + 3));
    std::copy(actionAt, actionAt + count, state.lastFrame.begin() + static_cast<std::ptrdiff_t>(2 * count + 6));
}

void BodyContract3D::beginDecision(BodyContractState3D& state, std::span<const float> action,
    std::span<const float> mean) const {
    const std::size_t count = m_body.dofs.size();
    if (action.size() != count || mean.size() != count)
        throw std::runtime_error("contrato do Corpo: acao com tamanho errado");
    // A memoria: o quadro da ultima observacao entra na frente.
    const std::size_t frame = frameSize();
    if (m_settings.historyDecisions > 0 && state.lastFrame.size() == frame) {
        std::copy_backward(state.history.begin(), state.history.end() - static_cast<std::ptrdiff_t>(frame),
            state.history.end());
        std::copy(state.lastFrame.begin(), state.lastFrame.end(), state.history.begin());
    }
    const float slope = m_settings.velocityFraction / m_settings.decisionSeconds;
    for (std::size_t d = 0; d < count; ++d) {
        const float a = std::clamp(action[d], -RLActionLimit, RLActionLimit);
        state.previousAction[d] = a;
        state.rampFrom[d] = state.rampTo[d];
        state.rampTo[d] = targetOf(d, a);
        const float meanTarget = targetOf(d, std::clamp(mean[d], -RLActionLimit, RLActionLimit));
        float velocity = 0.0f;
        if (state.started) {
            if (m_settings.velocityTarget == BodyVelocityTarget3D::SampledSlope)
                velocity = (state.rampTo[d] - state.rampFrom[d]) * slope;
            else if (m_settings.velocityTarget == BodyVelocityTarget3D::MeanSlope)
                velocity = (meanTarget - state.previousMeanTarget[d]) * slope;
        }
        state.velocityTarget[d] = std::clamp(velocity, -m_settings.maximumTargetVelocity, m_settings.maximumTargetVelocity);
        state.previousMeanTarget[d] = meanTarget;
    }
    state.started = true;
}

void BodyContract3D::writeTargets(const BodyContractState3D& state, std::uint32_t substep,
    const PlayerAttributes3D& player, std::vector<RagdollDriveTarget3D>& out, float hiddenStrength) const {
    writeTargetsAt(state,
        static_cast<float>(std::min(substep, m_settings.substepsPerDecision - 1) + 1)
            / static_cast<float>(m_settings.substepsPerDecision),
        player, out, hiddenStrength);
}

void BodyContract3D::writeTargetsAt(const BodyContractState3D& state, float u, const PlayerAttributes3D& player,
    std::vector<RagdollDriveTarget3D>& out, float hiddenStrength) const {
    const std::size_t count = m_body.dofs.size();
    out.resize(count + m_body.lockedDofs.size());
    u = std::clamp(u, 0.0f, 1.0f);
    const auto fill = [&](RagdollDriveTarget3D& target, const RLBodyDof3D& dof, StrengthGroup3D group, float position,
                          float velocity) {
        target.linkIndex = dof.linkIndex;
        target.axis = dof.axis;
        target.positionRadians = position;
        target.velocityRadiansPerSecond = velocity;
        target.feedforwardTorqueNewtonMeters = 0.0f;
        target.stiffnessScale = dof.stiffnessScale;
        target.dampingScale = dof.dampingScale;
        target.maximumTorqueScale =
            dof.maximumTorqueScale * player.strength[static_cast<std::size_t>(group)] * hiddenStrength;
        target.gravityCompensationScale = 0.0f;
    };
    for (std::size_t d = 0; d < count; ++d)
        fill(out[d], m_body.dofs[d], m_dofGroup[d], state.rampFrom[d] + (state.rampTo[d] - state.rampFrom[d]) * u,
            state.velocityTarget[d]);
    for (std::size_t l = 0; l < m_body.lockedDofs.size(); ++l)
        fill(out[count + l], m_body.lockedDofs[l], m_lockedGroup[l], m_body.lockedDofs[l].baseRadians, 0.0f);
}

} // namespace MatterEngine
