#include "Engine/Tracking/TrackingGetUpAmp3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingFeatures3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace MatterEngine {
namespace {

std::uint32_t linkIndex(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<std::uint32_t>(i);
    throw std::runtime_error("levantar livre: o perfil nao tem o link " + id);
}

std::uint32_t rootIndex(const RagdollProfile3D& profile) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].parentIndex < 0) return static_cast<std::uint32_t>(i);
    throw std::runtime_error("levantar livre: o perfil nao tem raiz");
}

// A orientacao de um link sem a do modelo (os eixos do corpo: x a frente, z o alto).
Quaternion bodyOrientation(const RagdollLinkDefinition3D& link, const PhysicsBodyState3D& state) {
    return (state.orientation * link.modelOrientation.conjugate()).normalized();
}

} // namespace

TrackingAmpFeatures3D::TrackingAmpFeatures3D(const RagdollProfile3D& profile)
    : m_profile(&profile), m_root(rootIndex(profile)) {
    for (const char* id : { "Head", "LeftHand", "RightHand", "LeftFoot", "RightFoot" })
        m_keyBodies.push_back(linkIndex(profile, id));
    m_size = 1 + 6 + 3 + 3 + 9 * (profile.links.size() - 1) + 3 * m_keyBodies.size();
}

void TrackingAmpFeatures3D::compute(std::span<const PhysicsBodyState3D> links, std::span<float> out) const {
    const RagdollProfile3D& profile = *m_profile;
    const PhysicsBodyState3D& root = links[m_root];
    const float yaw = trackingBodyYaw3D(profile.links[m_root], root);
    const TrackingHeadingFrame3D frame = TrackingHeadingFrame3D::at(root.position, yaw);
    const Quaternion heading = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw).conjugate();
    std::size_t n = 0;
    const auto put = [&](Vec3 v) {
        out[n++] = v.x;
        out[n++] = v.y;
        out[n++] = v.z;
    };
    // (A tangente e a normal de uma rotacao: os eixos x e z girados, continuos ao contrario do quaternion.)
    const auto tangentNormal = [&](Quaternion q) {
        put(q.rotate({ 1.0f, 0.0f, 0.0f }));
        put(q.rotate({ 0.0f, 0.0f, 1.0f }));
    };
    out[n++] = root.position.z;
    tangentNormal((heading * root.orientation).normalized());
    put(frame.direction(root.linearVelocity));
    put(frame.direction(root.angularVelocity));
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const int parent = profile.links[i].parentIndex;
        if (parent < 0) continue;
        const PhysicsBodyState3D& p = links[static_cast<std::size_t>(parent)];
        const PhysicsBodyState3D& s = links[i];
        tangentNormal((p.orientation.conjugate() * s.orientation).normalized());
        put(p.orientation.conjugate().rotate(s.angularVelocity - p.angularVelocity));
    }
    for (std::uint32_t k : m_keyBodies) put(frame.point(rlLinkCenterOfMass3D(profile.links[k], links[k])));
}

TrackingGetUpPostureProbe3D::TrackingGetUpPostureProbe3D(const RagdollProfile3D& profile)
    : m_profile(&profile), m_root(rootIndex(profile)), m_head(linkIndex(profile, "Head")),
      m_chest(linkIndex(profile, "UpperChest")) {}

TrackingGetUpPosture3D TrackingGetUpPostureProbe3D::measure(std::span<const PhysicsBodyState3D> links) const {
    const RagdollProfile3D& profile = *m_profile;
    TrackingGetUpPosture3D out;
    out.headHeight = rlLinkCenterOfMass3D(profile.links[m_head], links[m_head]).z;
    out.pelvisHeight = links[m_root].position.z;
    out.upright = bodyOrientation(profile.links[m_root], links[m_root]).rotate({ 0.0f, 0.0f, 1.0f }).z;
    out.chestFacingUp = bodyOrientation(profile.links[m_chest], links[m_chest]).rotate({ 1.0f, 0.0f, 0.0f }).z;
    out.standing = out.headHeight > 1.4f && out.pelvisHeight > 0.75f && out.upright > 0.8f;
    return out;
}

float TrackingGetUpPostureProbe3D::standingReward(const TrackingGetUpPosture3D& posture,
    std::span<const PhysicsBodyState3D> links) const {
    if (!posture.standing) return 0.0f;
    const Vec3 v = rlBodyCenterOfMassVelocity3D(*m_profile, links);
    return 0.3f + 0.1f * std::exp(-2.0f * (v.x * v.x + v.y * v.y)) + 0.6f * idlePoseMatch(links);
}

void TrackingGetUpPostureProbe3D::setIdlePose(std::span<const PhysicsBodyState3D> links) {
    m_idle.clear();
    addIdlePose(links);
}

void TrackingGetUpPostureProbe3D::addIdlePose(std::span<const PhysicsBodyState3D> links) {
    std::vector<Quaternion> pose(links.size(), Quaternion {});
    for (std::size_t i = 0; i < links.size(); ++i) {
        const int parent = m_profile->links[i].parentIndex;
        if (parent >= 0)
            pose[i] = (links[static_cast<std::size_t>(parent)].orientation.conjugate() * links[i].orientation).normalized();
    }
    m_idle.push_back(std::move(pose));
}

float TrackingGetUpPostureProbe3D::idlePoseMatch(std::span<const PhysicsBodyState3D> links) const {
    if (m_idle.empty()) return 1.0f;
    float best = 0.0f;
    for (const std::vector<Quaternion>& idle : m_idle) {
        float sum = 0.0f;
        for (std::size_t i = 0; i < links.size() && i < idle.size(); ++i) {
            const int parent = m_profile->links[i].parentIndex;
            if (parent < 0) continue;
            const Quaternion relative =
                (links[static_cast<std::size_t>(parent)].orientation.conjugate() * links[i].orientation).normalized();
            const Vec3 error = rlQuaternionLog3D((idle[i].conjugate() * relative).normalized());
            sum += error.x * error.x + error.y * error.y + error.z * error.z;
        }
        best = std::max(best, 0.5f * std::exp(-0.5f * sum) + 0.5f * std::exp(-2.0f * sum));
    }
    return best;
}

float TrackingGetUpPostureProbe3D::risePotential(const TrackingGetUpPosture3D& posture) const {
    const float rise = std::clamp((posture.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
    return 30.0f * (0.6f * rise + 0.2f * std::max(0.0f, posture.upright));
}

float TrackingGetUpPostureProbe3D::turnPotential(const TrackingGetUpPosture3D& posture) const {
    const float rise = std::clamp((posture.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
    return 0.15f * std::max(0.0f, -posture.chestFacingUp) * (1.0f - rise);
}

} // namespace MatterEngine
