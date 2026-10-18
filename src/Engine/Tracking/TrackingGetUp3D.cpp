#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingFeatures3D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {

const char* const AxisNames[3] = { "twist", "swing1", "swing2" };

float rotationAngle(Quaternion q) {
    const float w = std::min(1.0f, std::abs(q.normalized().w));
    return 2.0f * std::acos(w);
}

// A velocidade angular de uma junta: a do filho menos a do pai (no mundo); a raiz, a dela.
Vec3 jointAngularVelocity(std::span<const PhysicsBodyState3D> links, const RagdollProfile3D& profile, std::size_t j) {
    const int parent = profile.links[j].parentIndex;
    return parent < 0 ? links[j].angularVelocity : links[j].angularVelocity - links[static_cast<std::size_t>(parent)].angularVelocity;
}

Quaternion jointOrientation(std::span<const PhysicsBodyState3D> links, const RagdollProfile3D& profile, std::size_t j) {
    const int parent = profile.links[j].parentIndex;
    return parent < 0 ? links[j].orientation
                      : links[static_cast<std::size_t>(parent)].orientation.conjugate() * links[j].orientation;
}

} // namespace

float trackingBodyYaw3D(const RagdollLinkDefinition3D& pelvis, const PhysicsBodyState3D& state) {
    const Quaternion r = (state.orientation * pelvis.modelOrientation.conjugate()).normalized();
    const Vec3 forward = r.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 up = r.rotate({ 0.0f, 0.0f, 1.0f });
    const float x = forward.x + up.x, y = forward.y + up.y;
    return std::abs(x) + std::abs(y) > 1e-6f ? std::atan2(y, x) : 0.0f;
}

TrackingGetUpContract3D::TrackingGetUpContract3D(const RagdollProfile3D& profile) : m_profile(&profile) {
    bool head = false;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        if (link.parentIndex < 0) m_root = static_cast<std::uint32_t>(i);
        if (link.id == "Head") {
            m_head = static_cast<std::uint32_t>(i);
            head = true;
        }
        if (link.id == "LeftHand" || link.id == "RightHand" || link.id == "LeftFoot" || link.id == "RightFoot")
            m_endEffectors.push_back(static_cast<std::uint32_t>(i));
        if (link.parentIndex < 0) continue;
        for (std::uint8_t axis = 0; axis < 3; ++axis)
            if (link.inboundJoint.axes[axis].enabled) {
                m_axes.push_back({ static_cast<std::uint32_t>(i), axis });
                m_actionNames.push_back(link.id + "." + AxisNames[axis]);
            }
    }
    if (!head || m_endEffectors.size() != 4) throw std::runtime_error("levantar: o perfil nao tem cabeca, maos e pes");
    m_observationNames.push_back("raiz.altura");
    for (const RagdollLinkDefinition3D& link : profile.links) {
        for (const char* part : { ".pos.x", ".pos.y", ".pos.z", ".rot.x", ".rot.y", ".rot.z", ".rot.w", ".vel.x",
                 ".vel.y", ".vel.z", ".ang.x", ".ang.y", ".ang.z" })
            m_observationNames.push_back(link.id + part);
    }
    m_observationNames.push_back("fase");
}

void TrackingGetUpContract3D::driveTargets(std::span<const float> action, std::vector<RagdollDriveTarget3D>& out) const {
    out.clear();
    for (std::size_t k = 0; k < m_axes.size(); ++k) {
        RagdollDriveTarget3D drive;
        drive.linkIndex = m_axes[k].link;
        drive.axis = static_cast<RagdollAxis3D>(m_axes[k].axis);
        drive.positionRadians = k < action.size() && std::isfinite(action[k]) ? action[k] : 0.0f;
        drive.velocityRadiansPerSecond = 0.0f;
        out.push_back(drive);
    }
}

void TrackingGetUpContract3D::actionOf(std::span<const Vec3> coordinates, std::span<float> out) const {
    for (std::size_t k = 0; k < m_axes.size() && k < out.size(); ++k) {
        const Vec3 c = m_axes[k].link < coordinates.size() ? coordinates[m_axes[k].link] : Vec3 {};
        out[k] = m_axes[k].axis == 0 ? c.x : m_axes[k].axis == 1 ? c.y : c.z;
    }
}

void TrackingGetUpContract3D::observe(std::span<const PhysicsBodyState3D> links, float phase, std::span<float> out) const {
    const RagdollProfile3D& profile = *m_profile;
    const PhysicsBodyState3D& root = links[m_root];
    const float yaw = trackingBodyYaw3D(profile.links[m_root], root);
    const TrackingHeadingFrame3D frame = TrackingHeadingFrame3D::at(root.position, yaw);
    const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw).conjugate();
    std::size_t n = 0;
    out[n++] = root.position.z;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const PhysicsBodyState3D& s = links[i];
        const Vec3 p = frame.point(rlLinkCenterOfMass3D(profile.links[i], s));
        Quaternion q = (heading * s.orientation).normalized();
        if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
        const Vec3 v = frame.direction(s.linearVelocity);
        const Vec3 w = frame.direction(s.angularVelocity);
        for (float value : { p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z }) out[n++] = value;
    }
    out[n++] = phase;
}

TrackingGetUpContract3D::Terms TrackingGetUpContract3D::reward(std::span<const PhysicsBodyState3D> body,
    const TrackingClipFrame3D& reference) const {
    const RagdollProfile3D& profile = *m_profile;
    const std::span<const PhysicsBodyState3D> ref(reference.links);
    Terms terms;
    double pose = 0.0, velocity = 0.0, end = 0.0;
    for (std::size_t j = 0; j < profile.links.size(); ++j) {
        const float angle = rotationAngle(jointOrientation(ref, profile, j).conjugate() * jointOrientation(body, profile, j));
        pose += static_cast<double>(angle) * angle;
        velocity += (jointAngularVelocity(ref, profile, j) - jointAngularVelocity(body, profile, j)).lengthSquared();
    }
    for (std::uint32_t e : m_endEffectors)
        end += (rlLinkCenterOfMass3D(profile.links[e], ref[e]) - rlLinkCenterOfMass3D(profile.links[e], body[e])).lengthSquared();
    const Vec3 com = rlBodyCenterOfMass3D(profile, body);
    terms.pose = static_cast<float>(std::exp(-2.0 * pose));
    terms.velocity = static_cast<float>(std::exp(-0.1 * velocity));
    terms.endEffector = static_cast<float>(std::exp(-40.0 * end));
    terms.centerOfMass = std::exp(-10.0f * (reference.centerOfMass - com).lengthSquared());
    terms.total = 0.65f * terms.pose + 0.1f * terms.velocity + 0.15f * terms.endEffector + 0.1f * terms.centerOfMass;
    return terms;
}

} // namespace MatterEngine
