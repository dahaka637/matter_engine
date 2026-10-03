#include "Engine/Control/CharacterWholeBody3D.hpp"

namespace MatterEngine {

void CharacterWholeBody3D::prepare(const RagdollProfile3D& profile) {
    m_links.assign(profile.links.size(), LinkConstants {});
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        LinkConstants& constants = m_links[index];
        constants.parentIndex = link.parentIndex;
        if (link.parentIndex < 0) continue;
        // Ancoradouro e frame articular no espaco local do proprio link (o
        // perfil os descreve em espaco de modelo), como o compensador.
        const Quaternion inverseModel = link.modelOrientation.conjugate();
        constants.anchorLocal = inverseModel.rotate(
            link.inboundJoint.anchorModelPosition - link.modelPosition);
        constants.jointFrameLocal = inverseModel * link.inboundJoint.frameModelOrientation;
        for (std::size_t axis = 0; axis < 3; ++axis)
            constants.axisEnabled[axis] = link.inboundJoint.axes[axis].enabled;
    }
}

void CharacterWholeBody3D::addContactTorques(const RagdollState3D& state,
    std::span<const ContactWrench3D> wrenches,
    std::vector<std::array<float, 3>>& torques) const {
    const std::size_t linkCount = m_links.size();
    if (torques.size() != linkCount) torques.assign(linkCount, { 0.0f, 0.0f, 0.0f });
    if (state.links.size() != linkCount) return;
    for (const ContactWrench3D& wrench : wrenches) {
        if (wrench.linkIndex >= linkCount) continue;
        // Da junta do link apoiado ate a raiz (exclusive): cada junta segura
        // a subarvore abaixo dela contra o esforco do chao. Equilibrio da
        // subarvore: tau + (p - a) x f + t = 0 -> tau = -((p - a) x f + t).
        for (int index = static_cast<int>(wrench.linkIndex);
             index > 0 && m_links[static_cast<std::size_t>(index)].parentIndex >= 0;
             index = m_links[static_cast<std::size_t>(index)].parentIndex) {
            const LinkConstants& constants = m_links[static_cast<std::size_t>(index)];
            const PhysicsBodyState3D& body = state.links[static_cast<std::size_t>(index)];
            const Vec3 anchor = body.position + body.orientation.rotate(constants.anchorLocal);
            const Vec3 moment = cross(wrench.pointWorld - anchor, wrench.forceWorld)
                + wrench.torqueWorld;
            const Quaternion frame = body.orientation * constants.jointFrameLocal;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (!constants.axisEnabled[axis]) continue;
                const Vec3 axisLocal = axis == 0 ? Vec3 { 1.0f, 0.0f, 0.0f }
                    : axis == 1 ? Vec3 { 0.0f, 1.0f, 0.0f } : Vec3 { 0.0f, 0.0f, 1.0f };
                torques[static_cast<std::size_t>(index)][axis] -= dot(frame.rotate(axisLocal), moment);
            }
        }
    }
}

} // namespace MatterEngine
