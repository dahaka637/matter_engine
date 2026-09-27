#include "Engine/Physics/Articulation/ArticulationGravity3D.hpp"

namespace MatterEngine {

void ArticulationGravityCompensator3D::prepare(
    const RagdollProfile3D& profile) {
    const std::size_t linkCount = profile.links.size();
    m_links.assign(linkCount, LinkConstants {});
    m_jointTorques.assign(linkCount, std::array<float, 3> { 0.0f, 0.0f, 0.0f });
    m_subtreeForce.assign(linkCount, Vec3 {});
    m_subtreeMoment.assign(linkCount, Vec3 {});

    for (std::size_t index = 0; index < linkCount; ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        LinkConstants& constants = m_links[index];
        constants.parentIndex = link.parentIndex;
        constants.massKg = link.massFraction * profile.totalMassKg;
        constants.centerOfMassLocal = link.centerOfMassLocal;
        constants.hasInboundJoint = link.parentIndex >= 0;
        if (!constants.hasInboundJoint) continue;

        // O perfil descreve ancoradouro e frame articular em espaco de modelo.
        // Convertemos os dois para o espaco local do proprio link uma unica
        // vez: assim, em tempo de execucao, basta compor com a pose atual.
        const Quaternion inverseModel = link.modelOrientation.conjugate();
        constants.anchorLocal = inverseModel.rotate(
            link.inboundJoint.anchorModelPosition - link.modelPosition);
        constants.jointFrameLocal =
            inverseModel * link.inboundJoint.frameModelOrientation;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            constants.axisEnabled[axis] =
                link.inboundJoint.axes[axis].enabled;
        }
    }
}

void ArticulationGravityCompensator3D::compute(
    std::span<const ArticulationLinkPose3D> linkPoses, Vec3 gravity) {
    const std::size_t linkCount = m_links.size();
    for (auto& torques : m_jointTorques) {
        torques = { 0.0f, 0.0f, 0.0f };
    }
    if (linkCount == 0 || linkPoses.size() < linkCount) return;

    // Peso de cada link e seu momento em torno da origem do mundo. Somar
    // momentos em torno de um ponto comum e o que permite acumular subarvores
    // sem refazer produto vetorial por junta.
    for (std::size_t index = 0; index < linkCount; ++index) {
        const ArticulationLinkPose3D& pose = linkPoses[index];
        const Vec3 centerOfMassWorld = pose.position
            + pose.orientation.rotate(m_links[index].centerOfMassLocal);
        const Vec3 weight = gravity * m_links[index].massKg;
        m_subtreeForce[index] = weight;
        m_subtreeMoment[index] = cross(centerOfMassWorld, weight);
    }

    // Passada de tras para frente. O contrato do perfil garante pai antes de
    // filho, portanto ao alcancar `index` toda a sua subarvore ja somou nele.
    for (std::size_t index = linkCount; index-- > 0;) {
        const LinkConstants& constants = m_links[index];
        if (!constants.hasInboundJoint) continue;

        const ArticulationLinkPose3D& pose = linkPoses[index];
        const Vec3 anchorWorld =
            pose.position + pose.orientation.rotate(constants.anchorLocal);
        // Momento da subarvore transportado da origem do mundo para o
        // ancoradouro da junta.
        const Vec3 momentAboutAnchor = m_subtreeMoment[index]
            - cross(anchorWorld, m_subtreeForce[index]);

        const Quaternion jointFrameWorld =
            pose.orientation * constants.jointFrameLocal;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!constants.axisEnabled[axis]) continue;
            // X = Twist, Y = Swing1, Z = Swing2 - a mesma base anatomica que o
            // perfil declara e que os motores por eixo consomem.
            const Vec3 axisLocal = axis == 0 ? Vec3 { 1.0f, 0.0f, 0.0f }
                : axis == 1 ? Vec3 { 0.0f, 1.0f, 0.0f }
                : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 axisWorld = jointFrameWorld.rotate(axisLocal);
            // Sinal: a gravidade produz `dot(eixo, momento)` nesse eixo, e o
            // motor precisa aplicar o oposto para sustentar.
            m_jointTorques[index][axis] =
                -dot(axisWorld, momentAboutAnchor);
        }

        const std::size_t parentIndex =
            static_cast<std::size_t>(constants.parentIndex);
        if (parentIndex >= linkCount) continue;
        m_subtreeForce[parentIndex] += m_subtreeForce[index];
        m_subtreeMoment[parentIndex] += m_subtreeMoment[index];
    }
}

} // namespace MatterEngine
