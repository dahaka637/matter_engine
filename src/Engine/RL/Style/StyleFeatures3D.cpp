#include "Engine/RL/Style/StyleFeatures3D.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace MatterEngine {
namespace {

float component(Vec3 v, RagdollAxis3D axis) {
    return axis == RagdollAxis3D::Twist ? v.x : axis == RagdollAxis3D::Swing1 ? v.y : v.z;
}

} // namespace

StyleFeatures3D::StyleFeatures3D(const RLBody3D& body) : m_profile(body.profile), m_allDofs(body.dofs) {
    for (std::size_t d = 0; d < body.dofs.size(); ++d) {
        const auto link = static_cast<int>(body.dofs[d].linkIndex);
        if (link == body.leftToe || link == body.rightToe) continue;
        m_dofs.push_back(static_cast<std::uint32_t>(d));
    }
    m_keyBodies = { body.head, body.leftHand, body.rightHand, body.leftFoot, body.rightFoot };
    RLVectorLayout3D& l = m_layout;
    l.id = "matter.rl.style";
    l.version = 1;
    const auto vec3 = [&](const std::string& name, const char* unit) {
        for (const char* axis : { ".x", ".y", ".z" }) l.channels.push_back({ name + axis, unit });
    };
    l.channels.push_back({ "pelvis.height", "m/escala" });
    vec3("pelvis.up_local", "unit");
    vec3("pelvis.velocity_heading", "m/s/froude");
    vec3("pelvis.spin_local", "rad/s*froude");
    for (const std::uint32_t d : m_dofs) l.channels.push_back({ body.dofs[d].name + ".position", "rad" });
    for (const std::uint32_t d : m_dofs) l.channels.push_back({ body.dofs[d].name + ".velocity", "rad/s*froude" });
    for (const std::uint32_t link : m_keyBodies) vec3(body.profile.links[link].id + ".offset_heading", "m/escala");
}

void StyleFeatures3D::write(std::span<const PhysicsBodyState3D> links, float heightScale, float* out) const {
    const RagdollProfile3D& profile = m_profile;
    if (links.size() != profile.links.size()) throw std::runtime_error("estilo: o estado nao e deste corpo");
    const float scale = heightScale > 0.0f ? heightScale : 1.0f;
    const float froude = std::sqrt(scale);
    const PhysicsBodyState3D& pelvis = links.front();
    const Quaternion pelvisFrame = (pelvis.orientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = pelvisFrame.rotate({ 1.0f, 0.0f, 0.0f });
    const Quaternion headingInverse =
        Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -std::atan2(forward.y, forward.x));
    float* p = out;
    const auto put = [&p](Vec3 v) {
        *p++ = v.x;
        *p++ = v.y;
        *p++ = v.z;
    };
    *p++ = pelvis.position.z / scale;
    put(pelvisFrame.conjugate().rotate({ 0.0f, 0.0f, 1.0f }));
    put(headingInverse.rotate(pelvis.linearVelocity) * (1.0f / froude));
    put(pelvisFrame.conjugate().rotate(pelvis.angularVelocity) * froude);
    // Os angulos e as velocidades dos eixos, pela conta do backend.
    for (const std::uint32_t d : m_dofs) {
        const RLBodyDof3D& dof = m_allDofs[d];
        const RagdollLinkDefinition3D& link = profile.links[dof.linkIndex];
        const PhysicsBodyState3D& child = links[dof.linkIndex];
        const PhysicsBodyState3D& parent = links[static_cast<std::size_t>(link.parentIndex)];
        *p++ = component(rlJointCoordinates3D(profile, dof.linkIndex, parent.orientation, child.orientation), dof.axis);
    }
    for (const std::uint32_t d : m_dofs) {
        const RLBodyDof3D& dof = m_allDofs[d];
        const RagdollLinkDefinition3D& link = profile.links[dof.linkIndex];
        const PhysicsBodyState3D& child = links[dof.linkIndex];
        const PhysicsBodyState3D& parent = links[static_cast<std::size_t>(link.parentIndex)];
        const Quaternion frame = child.orientation * link.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation;
        *p++ = component(frame.conjugate().rotate(child.angularVelocity - parent.angularVelocity), dof.axis) * froude;
    }
    for (const std::uint32_t link : m_keyBodies) put(headingInverse.rotate(links[link].position - pelvis.position) * (1.0f / scale));
    if (static_cast<std::size_t>(p - out) != m_layout.size()) throw std::runtime_error("estilo: o layout nao bate");
}

} // namespace MatterEngine
