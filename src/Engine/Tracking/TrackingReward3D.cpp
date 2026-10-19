#include "Engine/Tracking/TrackingReward3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {

// O angulo da rotacao de um quaternion unitario (rad, 0..pi).
float rotationAngle(Quaternion q) {
    const float w = std::min(1.0f, std::abs(q.normalized().w));
    return 2.0f * std::acos(w);
}

} // namespace

TrackingReward3D::TrackingReward3D(const RagdollProfile3D& profile)
    : m_profile(&profile), m_boxes(trackingSurfaceBoxes3D(profile)) {
    bool found = false;
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == "Head") {
            m_head = static_cast<std::uint32_t>(i);
            found = true;
        }
    if (!found) throw std::runtime_error("rastreador: o perfil nao tem a cabeca");
    m_leg.resize(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        m_leg[i] = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
            || id.find("Foot") != std::string::npos;
    }
}

TrackingRewardTerms3D TrackingReward3D::evaluate(const TrackingLeaderState3D& leader,
    std::span<const PhysicsBodyState3D> bodyLinks, Vec3 bodyCenterOfMass, Vec3 bodyCenterOfMassVelocity,
    float legWeight) {
    const RagdollProfile3D& profile = *m_profile;
    TrackingRewardTerms3D terms;
    const std::size_t count = std::min({ profile.links.size(), leader.links.size(), bodyLinks.size() });
    if (count == 0) return terms;
    const TrackingHeadingFrame3D kin = TrackingHeadingFrame3D::at(leader.centerOfMass, leader.headingYawRadians);
    const TrackingHeadingFrame3D sim = TrackingHeadingFrame3D::at(bodyCenterOfMass, leader.headingYawRadians);

    trackingSurfacePoints3D(profile, m_boxes, std::span(leader.links).first(count), m_kinPoints, m_kinVelocities);
    trackingSurfacePoints3D(profile, m_boxes, bodyLinks.first(count), m_simPoints, m_simVelocities);
    double positionSum = 0.0, velocitySum = 0.0;
    for (std::size_t p = 0; p < m_kinPoints.size(); ++p) {
        // (6 pontos por link, na ordem dos links.)
        const double w = m_leg[p / 6] ? legWeight : 1.0;
        positionSum += w * (kin.point(m_kinPoints[p]) - sim.point(m_simPoints[p])).length();
        velocitySum += w * (kin.direction(m_kinVelocities[p]) - sim.direction(m_simVelocities[p])).length();
    }
    const double bodies = static_cast<double>(count);
    terms.position = static_cast<float>(std::exp(-10.0 / bodies * positionSum));
    terms.velocity = static_cast<float>(std::exp(-1.0 / bodies * velocitySum));

    double localSum = 0.0;
    std::size_t localCount = 0;
    for (std::size_t j = 0; j < count; ++j) {
        const int parent = profile.links[j].parentIndex;
        if (parent < 0 || static_cast<std::size_t>(parent) >= count) continue;
        const std::size_t p = static_cast<std::size_t>(parent);
        const Quaternion kinLocal = leader.links[p].orientation.conjugate() * leader.links[j].orientation;
        const Quaternion simLocal = bodyLinks[p].orientation.conjugate() * bodyLinks[j].orientation;
        localSum += (m_leg[j] ? legWeight : 1.0) * rotationAngle(kinLocal.conjugate() * simLocal);
        ++localCount;
    }
    terms.localPose = localCount > 0
        ? static_cast<float>(std::exp(-10.0 / static_cast<double>(localCount) * localSum)) : 0.0f;
    terms.centerOfMassVelocity = std::exp(-(leader.centerOfMassVelocity - bodyCenterOfMassVelocity).length());

    const Vec3 kinHead = rlLinkCenterOfMass3D(profile.links[m_head], leader.links[m_head]);
    const Vec3 simHead = rlLinkCenterOfMass3D(profile.links[m_head], bodyLinks[m_head]);
    terms.headDistance = (kinHead - simHead).length();
    terms.fallFactor = std::clamp(1.3f - 1.4f * terms.headDistance, 0.0f, 1.0f);
    terms.terminate = terms.headDistance > 1.0f;
    terms.total = terms.terminate
        ? 0.0f
        : terms.fallFactor * (terms.position + terms.velocity + terms.localPose + terms.centerOfMassVelocity);
    return terms;
}

} // namespace MatterEngine
