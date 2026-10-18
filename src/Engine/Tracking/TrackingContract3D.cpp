#include "Engine/Tracking/TrackingContract3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingFeatures3D.hpp"

#include <algorithm>
#include <stdexcept>

namespace MatterEngine {
namespace {

std::uint32_t findLink(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<std::uint32_t>(i);
    throw std::runtime_error("rastreador: o perfil nao tem o link " + id);
}

float component(Vec3 v, std::size_t axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

void setComponent(Vec3& v, std::size_t axis, float value) {
    if (axis == 0) v.x = value;
    else if (axis == 1) v.y = value;
    else v.z = value;
}

const char* const AxisNames[3] = { "twist", "swing1", "swing2" };

} // namespace

TrackingContract3D::TrackingContract3D(const RagdollProfile3D& profile) : m_profile(&profile) {
    // A ordem da acao (DReCon 6.4, no corpo do jogo: sem dedo, pe de 2 eixos).
    const char* const actionLinks[] = { "LeftThigh", "LeftShin", "LeftFoot", "RightThigh", "RightShin", "RightFoot",
        "Abdomen", "LeftUpperArm", "RightUpperArm" };
    m_actionOfLink.assign(profile.links.size(), -1);
    std::uint32_t first = 0;
    for (const char* id : actionLinks) {
        ActionLink entry;
        entry.link = findLink(profile, id);
        entry.first = first;
        const RagdollJointDefinition3D& joint = profile.links[entry.link].inboundJoint;
        for (std::uint8_t axis = 0; axis < 3; ++axis)
            if (joint.axes[axis].enabled) entry.axes.push_back(axis);
        entry.composed = entry.axes.size() == 3;
        for (std::uint8_t axis : entry.axes) m_actionNames.push_back(std::string(id) + "." + AxisNames[axis]);
        first += static_cast<std::uint32_t>(entry.axes.size());
        m_actionOfLink[entry.link] = static_cast<int>(m_actionLinks.size());
        m_actionLinks.push_back(std::move(entry));
    }
    if (first != ActionSize)
        throw std::runtime_error("rastreador: a acao do perfil tem " + std::to_string(first) + " canais (esperava 21)");

    // Os 6 corpos da observacao (DReCon: os dedos, o Spine, a cabeca e os
    // antebracos; aqui os pes, o abdomen, a cabeca e os antebracos).
    const char* const keys[KeyBodyCount] = { "LeftFoot", "RightFoot", "Abdomen", "Head", "LeftForearm", "RightForearm" };
    for (std::size_t k = 0; k < KeyBodyCount; ++k) m_keyBodies[k] = findLink(profile, keys[k]);

    const auto vec = [this](const std::string& name, int n) {
        const char* const xyz[3] = { ".x", ".y", ".z" };
        for (int i = 0; i < n; ++i) m_observationNames.push_back(name + xyz[i]);
    };
    vec("v_kin", 3);
    vec("v_sim", 3);
    vec("v_sim_menos_kin", 3);
    vec("v_des", 2);
    vec("v_diff", 2);
    for (const char* block : { "s_sim", "s_sim_menos_kin" })
        for (const char* key : keys) {
            vec(std::string(block) + "." + key + ".pos", 3);
            vec(std::string(block) + "." + key + ".vel", 3);
        }
    for (const std::string& action : m_actionNames) m_observationNames.push_back("y_ant." + action);
    if (m_observationNames.size() != ObservationSize) throw std::logic_error("rastreador: observacao com tamanho errado");
}

void TrackingContract3D::driveTargets(const TrackingLeaderState3D& leader, const RagdollState3D& body,
    std::span<const float> filteredAction, std::vector<RagdollDriveTarget3D>& out) const {
    const RagdollProfile3D& profile = *m_profile;
    out.clear();
    for (std::size_t i = 1; i < profile.links.size(); ++i) {
        const RagdollJointDefinition3D& joint = profile.links[i].inboundJoint;
        Vec3 target = i < leader.coordinates.size() ? leader.coordinates[i] : Vec3 {};
        const Vec3 coordinateVelocity = i < leader.coordinateVelocities.size() ? leader.coordinateVelocities[i] : Vec3 {};
        const int action = m_actionOfLink[i];
        if (action >= 0 && filteredAction.size() >= ActionSize) {
            const ActionLink& entry = m_actionLinks[static_cast<std::size_t>(action)];
            if (entry.composed) {
                // R(alvo) = R(corr) * R(lider), no espaco de restricao da junta.
                const Vec3 correction { filteredAction[entry.first], filteredAction[entry.first + 1],
                    filteredAction[entry.first + 2] };
                target = rlQuaternionLog3D((rlQuaternionExp3D(correction) * rlQuaternionExp3D(target)).normalized());
            } else {
                for (std::size_t k = 0; k < entry.axes.size(); ++k)
                    setComponent(target, entry.axes[k], component(target, entry.axes[k]) + filteredAction[entry.first + k]);
            }
        }
        Vec3 measured;
        if (i < body.joints.size())
            measured = { body.joints[i].positionRadians[0], body.joints[i].positionRadians[1],
                body.joints[i].positionRadians[2] };
        const Vec3 velocity = trackingJointAngularVelocity3D(target, coordinateVelocity, measured);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!joint.axes[axis].enabled) continue;
            RagdollDriveTarget3D drive;
            drive.linkIndex = static_cast<std::uint32_t>(i);
            drive.axis = static_cast<RagdollAxis3D>(axis);
            drive.positionRadians = component(target, axis);
            // (O mesmo limite do modo fisico: folga acima do joelho do sprint.)
            drive.velocityRadiansPerSecond = std::clamp(component(velocity, axis), -30.0f, 30.0f);
            out.push_back(drive);
        }
    }
}

void TrackingContract3D::observe(const TrackingLeaderState3D& leader, std::span<const PhysicsBodyState3D> bodyLinks,
    Vec3 bodyCenterOfMass, Vec3 bodyCenterOfMassVelocity, std::span<const float> previousAction,
    std::span<float> out) const {
    const RagdollProfile3D& profile = *m_profile;
    const TrackingHeadingFrame3D kin = TrackingHeadingFrame3D::at(leader.centerOfMass, leader.headingYawRadians);
    const TrackingHeadingFrame3D sim = TrackingHeadingFrame3D::at(bodyCenterOfMass, leader.headingYawRadians);
    std::size_t n = 0;
    const auto put = [&out, &n](Vec3 v, int count) {
        out[n++] = v.x;
        out[n++] = v.y;
        if (count == 3) out[n++] = v.z;
    };
    const Vec3 vKin = kin.direction(leader.centerOfMassVelocity);
    const Vec3 vSim = sim.direction(bodyCenterOfMassVelocity);
    const Vec3 vDes = kin.direction(leader.requestedVelocityWorld);
    put(vKin, 3);
    put(vSim, 3);
    put(vSim - vKin, 3);
    put({ vDes.x, vDes.y, 0.0f }, 2);
    put({ vDes.x - vSim.x, vDes.y - vSim.y, 0.0f }, 2);
    std::array<Vec3, KeyBodyCount> simPosition {}, simVelocity {}, kinPosition {}, kinVelocity {};
    for (std::size_t k = 0; k < KeyBodyCount; ++k) {
        const std::uint32_t link = m_keyBodies[k];
        if (link < bodyLinks.size()) {
            simPosition[k] = sim.point(rlLinkCenterOfMass3D(profile.links[link], bodyLinks[link]));
            simVelocity[k] = sim.direction(bodyLinks[link].linearVelocity);
        }
        if (link < leader.links.size()) {
            kinPosition[k] = kin.point(rlLinkCenterOfMass3D(profile.links[link], leader.links[link]));
            kinVelocity[k] = kin.direction(leader.links[link].linearVelocity);
        }
    }
    for (std::size_t k = 0; k < KeyBodyCount; ++k) {
        put(simPosition[k], 3);
        put(simVelocity[k], 3);
    }
    for (std::size_t k = 0; k < KeyBodyCount; ++k) {
        put(simPosition[k] - kinPosition[k], 3);
        put(simVelocity[k] - kinVelocity[k], 3);
    }
    for (std::size_t a = 0; a < ActionSize; ++a) out[n++] = a < previousAction.size() ? previousAction[a] : 0.0f;
}

} // namespace MatterEngine
