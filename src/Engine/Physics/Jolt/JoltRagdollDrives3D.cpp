#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
bool finite(Vec3 value) { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z); }
bool finite(Quaternion value) { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) && std::isfinite(value.w)
    && value.x*value.x + value.y*value.y + value.z*value.z + value.w*value.w > 0.001f; }
}
void PhysicsScene3D::setRagdollRigidity(RagdollHandle3D h, float v) { if (std::isfinite(v)) m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetRigidity, std::clamp(v,0.0f,100.0f)}); }
void PhysicsScene3D::captureRagdollPose(RagdollHandle3D h) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::CapturePose, 0}); }
void PhysicsScene3D::setRagdollNeutralPose(RagdollHandle3D h) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::NeutralPose, 0}); }
void PhysicsScene3D::releaseRagdollDrives(RagdollHandle3D h) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::Release, 0}); }
void PhysicsScene3D::setRagdollActive(RagdollHandle3D h, bool v) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetActive, v ? 1.0f : 0.0f}); }
void PhysicsScene3D::setRagdollFrozen(RagdollHandle3D h, bool v) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetFrozen, v ? 1.0f : 0.0f}); }
void PhysicsScene3D::setRagdollActiveDriveTargets(RagdollHandle3D h, std::span<const RagdollDriveTarget3D> targets, bool gravity) {
    auto* r = m_impl->ragdoll(h);
    if (!r) return;
    r->stagedActiveTargets.clear();
    for (const auto& target : targets) {
        if (!target.linkIndex || target.linkIndex >= r->links.size() || static_cast<unsigned>(target.axis) >= 3
            || !std::isfinite(target.positionRadians) || !std::isfinite(target.velocityRadiansPerSecond)
            || !std::isfinite(target.feedforwardTorqueNewtonMeters) || !std::isfinite(target.stiffnessScale)
            || !std::isfinite(target.dampingScale) || !std::isfinite(target.maximumTorqueScale)) continue;
        r->stagedActiveTargets.push_back(target);
    }
    r->gravityCompensationEnabled = gravity;
    r->stagedActiveTargetsDirty = true;
}
void PhysicsScene3D::setRagdollAnimationConstraint(RagdollHandle3D h, const RagdollAnimationConstraint3D& constraint) {
    auto* r = m_impl->ragdoll(h);
    if (!r) return;
    r->animationConstraintStaged = false;
    if (!finite(constraint.rootPositionWorld) || !finite(constraint.rootOrientationWorld)
        || !finite(constraint.rootLinearVelocityWorld) || !finite(constraint.rootAngularVelocityWorld)
        || !std::isfinite(constraint.rootTranslationAuthority) || !std::isfinite(constraint.rootRotationAuthority)) return;
    r->animationConstraint = constraint;
    r->animationConstraintStaged = true;
}
void PhysicsScene3D::Impl::applyPendingRagdollCommands() {
    for (const auto& command : pendingRagdollCommands) {
        auto* r = ragdoll(command.handle);
        if (!r) continue;
        switch (command.type) {
        case RagdollCommandType::SetRigidity:
            r->rigidityPercent = r->passiveRigidityPercent = command.value;
            break;
        case RagdollCommandType::CapturePose:
            for (std::size_t i = 1; i < r->joints.size(); ++i) r->joints[i].targets = r->state.joints[i].positionRadians;
            break;
        case RagdollCommandType::NeutralPose:
            for (auto& joint : r->joints) joint.targets = {};
            break;
        case RagdollCommandType::Release:
            r->rigidityPercent = r->passiveRigidityPercent = 0;
            r->active = false;
            r->stagedActiveTargets.clear();
            r->gravityCompensationEnabled = false;
            break;
        case RagdollCommandType::SetActive: r->active = command.value != 0; break;
        case RagdollCommandType::SetFrozen: {
            const bool freeze = command.value != 0;
            if (freeze == r->frozen || (freeze && grabRagdoll == r->handle)) break;
            if (freeze) {
                for (auto id : r->links) std::erase_if(pendingBodyWrenches, [id](const auto& w) { return w.body == id; });
                r->ragdoll->RemoveFromPhysicsSystem();
            }
            else r->ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);
            r->frozen = freeze;
            break;
        }
        }
        if (!r->frozen) r->ragdoll->Activate();
    }
    pendingRagdollCommands.clear();
}
void PhysicsScene3D::Impl::applyRagdollDrives() {
    auto& bi = system->GetBodyInterface();
    for (auto& slot : ragdollSlots) {
        if (!slot.record || slot.record->frozen) continue;
        auto& r = *slot.record;
        // A raiz NAO e cinematica aqui. Ela participa do solver como qualquer
        // outro link, com massa finita, e o guia de animacao e resolvido
        // depois do passo (resolveRagdollGuides). Ver o comentario la.
        for (std::size_t i = 0; i < r.links.size(); ++i)
            r.gravityPoses[i] = {fromJolt(bi.GetPosition(r.links[i])), fromJolt(bi.GetRotation(r.links[i]))};
        if (r.gravityCompensationEnabled) r.gravityCompensator.compute(r.gravityPoses, settings.gravity);
        for (std::size_t i = 1; i < r.links.size(); ++i) {
            auto& runtime = r.joints[i];
            auto& constraint = *runtime.constraint;
            const auto& link = r.profile.links[i];
            JPH::Vec3 angles = JPH::Vec3::sZero(), velocity = JPH::Vec3::sZero(), feedforward = JPH::Vec3::sZero();
            for (int a = 0; a < 3; ++a) {
                const auto axis = static_cast<JPH::SixDOFConstraint::EAxis>(a+3);
                const auto& limit = link.inboundJoint.axes[a];
                if (!limit.enabled) continue;
                RagdollDriveTarget3D target;
                target.positionRadians = runtime.targets[a];
                const float rigidity = r.passiveRigidityPercent / 100;
                target.stiffnessScale = std::pow(rigidity, 2.2f);
                target.dampingScale = std::pow(rigidity, 1.4f);
                target.maximumTorqueScale = std::pow(rigidity, 1.25f);
                if (r.active) {
                    target.stiffnessScale = target.dampingScale = target.maximumTorqueScale = 0;
                    for (const auto& t : r.stagedActiveTargets)
                        if (t.linkIndex == i && static_cast<int>(t.axis) == a) { target = t; break; }
                }
                angles.SetComponent(a, std::clamp(target.positionRadians, limit.minimumRadians, limit.maximumRadians));
                velocity.SetComponent(a, target.velocityRadiansPerSecond);
                auto& motor = constraint.GetMotorSettings(axis);
                motor.mSpringSettings = {JPH::ESpringMode::StiffnessAndDamping,
                    limit.stiffness * std::clamp(target.stiffnessScale,0.0f,2.0f) * (r.active ? 5.0f : 1.0f),
                    limit.damping * std::clamp(target.dampingScale,0.0f,2.5f) * (r.active ? 2.0f : 1.0f)};
                const float maximum = limit.maximumTorque * std::clamp(target.maximumTorqueScale,0.0f,3.0f);
                motor.SetTorqueLimit(maximum);
                constraint.SetMotorState(axis, maximum > 0 ? JPH::EMotorState::PositionAndVelocity : JPH::EMotorState::Off);
                const float gravity = r.gravityCompensationEnabled ? r.gravityCompensator.jointTorques()[i][a] : 0;
                const float ff = std::clamp(target.feedforwardTorqueNewtonMeters + gravity, -maximum, maximum);
                feedforward.SetComponent(a, ff);
                // Feedforward consumes the same muscle budget as the implicit motor.
                motor.mMinTorqueLimit = -maximum - ff;
                motor.mMaxTorqueLimit = maximum - ff;
            }
            const float angle = angles.Length();
            const auto orientation = angle > 1e-7f ? JPH::Quat::sRotation(angles/angle, angle) : JPH::Quat::sIdentity();
            constraint.SetTargetOrientationCS(orientation);
            // Both APIs express angular motor velocity in the child joint frame.
            constraint.SetTargetAngularVelocityCS(velocity);
            if (!feedforward.IsNearZero()) {
                const auto frame = r.gravityPoses[i].orientation * link.modelOrientation.conjugate() * link.inboundJoint.frameModelOrientation;
                const Vec3 torque = frame.rotate(fromJolt(feedforward));
                pendingBodyWrenches.push_back({r.links[i], {}, torque});
                pendingBodyWrenches.push_back({r.links[static_cast<std::size_t>(link.parentIndex)], {}, -torque});
            }
        }
        if (r.stagedActiveTargetsDirty) r.ragdoll->Activate();
        r.stagedActiveTargetsDirty = false;
    }
}

namespace {
// Mesma curva do backend PhysX: autoridade 1 e escrita exata; abaixo disso, a
// fracao do caminho percorrida neste passo cresce com a autoridade e nunca
// chega a 1, de modo que uma autoridade parcial segue o guia sem teleportar.
float guideBlend(float authority, float dt) {
    authority = std::clamp(authority, 0.0f, 1.0f);
    if (authority >= 1.0f) return 1.0f;
    return 1.0f - std::exp(-dt * 12.0f * authority / std::max(0.001f, 1.0f - authority));
}
} // namespace

// Guia de animacao da raiz, resolvido DEPOIS do solver - a semantica exata do
// backend PhysX (resolveRagdollAnimationConstraint), portada.
//
// A primeira versao Jolt tornava a raiz carregada CINEMATICA durante o passo.
// Cinematico tem massa infinita: dois cinematicos nem colidem entre si, e um
// link dinamico de outro boneco preso contra a pelve fica espremido entre ela
// e as proprias juntas. Medido com o jogador andando para dentro de um boneco
// parado: 240 m/s em membro e ancoras separadas em 11,8 cm - os bonecos
// "tortos" e o tremor que o usuario via ao coloca-los em contato.
//
// Aqui a raiz fica dinamica durante o solver (contatos negociam com massas
// reais) e so depois o boneco inteiro e reposicionado sobre o guia. No PhysX
// escrever a transformacao da raiz em coordenadas reduzidas movia a arvore
// inteira rigidamente; em coordenadas maximas isso precisa ser explicito: a
// mesma transformacao rigida e aplicada a todos os links, e o campo de
// velocidades troca de referencial preservando a parte induzida pelas juntas.
//
// A interferencia externa tambem volta a valer como no PhysX: com
// releaseOnInteraction, contato com o mundo externo limita a autoridade a
// 1 - 0,9 * interferencia, e o boneco cede em vez de ser uma parede.
void PhysicsScene3D::Impl::resolveRagdollGuides() {
    auto& bi = system->GetBodyInterface();
    for (auto& slot : ragdollSlots) {
        if (!slot.record) continue;
        RagdollRecord& r = *slot.record;

        // Interferencia externa deste passo: contatos dos sensores mais
        // forcas externas explicitas. Pe apoiado em chao estatico e suporte,
        // nao interferencia.
        float impulse = r.externalForceMagnitude * stepDeltaTime;
        r.externalForceMagnitude = 0.0f;
        for (const RagdollContactPoint3D& point : r.stepContacts) {
            if (point.linkIndex >= r.profile.links.size()) continue;
            const std::string& id = r.profile.links[point.linkIndex].id;
            if ((id == "LeftFoot" || id == "RightFoot")
                && point.normal.z > 0.62f && !point.otherBodyDynamic) {
                continue;
            }
            impulse += std::max(0.0f, point.normalImpulseNewtonSeconds);
        }
        const float weight = std::max(1.0f, r.profile.totalMassKg * 9.81f);
        const float pressure = impulse / (weight * stepDeltaTime);
        r.state.externalInterference = pressure > 0.04f
            ? std::clamp(pressure / 0.5f, 0.0f, 1.0f) : 0.0f;
        r.state.animationAuthority = 0.0f;

        const bool staged = r.animationConstraintStaged;
        r.animationConstraintStaged = false;
        if (r.frozen || !r.active || !staged || grabRagdoll == r.handle) {
            continue;
        }

        RagdollAnimationConstraint3D target = r.animationConstraint;
        if (target.releaseOnInteraction && r.state.externalInterference > 0.0f) {
            const float cap = 1.0f - 0.90f * r.state.externalInterference;
            target.poseAuthority = std::min(target.poseAuthority, cap);
            target.rootTranslationAuthority =
                std::min(target.rootTranslationAuthority, cap);
            target.rootRotationAuthority =
                std::min(target.rootRotationAuthority, cap);
        }
        // Sem consumidor fora dos backends; formula identica a do PhysX.
        r.state.animationAuthority = std::min({ target.poseAuthority,
            target.rootTranslationAuthority, target.rootRotationAuthority });
        // poseAuthority (escrita direta de juntas) nao e suportada: nenhum
        // controlador a usa - o de locomocao a fixa em zero por decisao
        // registrada (juntas escritas tremiam, atravessavam parede e
        // arremessavam o corpo no levantar). Os motores fazem esse trabalho.

        const float translationBlend =
            guideBlend(target.rootTranslationAuthority, stepDeltaTime);
        const float rotationBlend =
            guideBlend(target.rootRotationAuthority, stepDeltaTime);
        if (translationBlend <= 0.0f && rotationBlend <= 0.0f) continue;

        const JPH::BodyID rootId = r.links.front();
        JPH::RVec3 rootPosition;
        JPH::Quat rootRotation;
        bi.GetPositionAndRotation(rootId, rootPosition, rootRotation);
        const JPH::Vec3 rootCenter = bi.GetCenterOfMassPosition(rootId);
        const JPH::Vec3 rootLinear = bi.GetLinearVelocity(rootId);
        const JPH::Vec3 rootAngular = bi.GetAngularVelocity(rootId);

        const JPH::Vec3 desiredPosition = toJolt(target.rootPositionWorld);
        JPH::Quat desiredRotation = toJolt(target.rootOrientationWorld);
        if (rootRotation.Dot(desiredRotation) < 0.0f) {
            desiredRotation = -desiredRotation;
        }
        const JPH::Vec3 desiredLinear = toJolt(target.rootLinearVelocityWorld);
        const JPH::Vec3 desiredAngular = toJolt(target.rootAngularVelocityWorld);

        const JPH::Vec3 newPosition = target.rootTranslationAuthority >= 1.0f
            ? desiredPosition
            : rootPosition + (desiredPosition - rootPosition) * translationBlend;
        const JPH::Vec3 newLinear = target.rootTranslationAuthority >= 1.0f
            ? desiredLinear
            : rootLinear + (desiredLinear - rootLinear) * translationBlend;
        const JPH::Quat newRotation = target.rootRotationAuthority >= 1.0f
            ? desiredRotation
            : (rootRotation * (1.0f - rotationBlend)
                + desiredRotation * rotationBlend).Normalized();
        const JPH::Vec3 newAngular = target.rootRotationAuthority >= 1.0f
            ? desiredAngular
            : rootAngular + (desiredAngular - rootAngular) * rotationBlend;

        // Transformacao rigida que leva a raiz atual a raiz resolvida.
        const JPH::Quat delta = (newRotation * rootRotation.Conjugated()).Normalized();
        for (const JPH::BodyID id : r.links) {
            JPH::RVec3 position;
            JPH::Quat rotation;
            bi.GetPositionAndRotation(id, position, rotation);
            const JPH::Vec3 center = bi.GetCenterOfMassPosition(id);
            // Velocidades do Jolt sao do centro de massa. A parte induzida
            // pelas juntas e o que sobra depois de remover o movimento rigido
            // da raiz; ela gira junto com o corpo e e preservada.
            const JPH::Vec3 jointLinear = bi.GetLinearVelocity(id)
                - rootLinear - rootAngular.Cross(center - rootCenter);
            const JPH::Vec3 jointAngular = bi.GetAngularVelocity(id) - rootAngular;
            const JPH::Vec3 movedCenterOffset = delta * (center - rootCenter);
            bi.SetPositionRotationAndVelocity(id,
                newPosition + delta * (position - rootPosition),
                (delta * rotation).Normalized(),
                newLinear + newAngular.Cross(movedCenterOffset)
                    + delta * jointLinear,
                newAngular + delta * jointAngular);
        }
    }
}
void PhysicsScene3D::applyRagdollRootForce(RagdollHandle3D h, Vec3 force, Vec3 torque) {
    applyRagdollLinkForce(h, 0, force, torque);
}
void PhysicsScene3D::applyRagdollControlRootForce(RagdollHandle3D h, Vec3 force, Vec3 torque) {
    auto* r = m_impl->ragdoll(h);
    if (r && !r->frozen && finite(force) && finite(torque))
        m_impl->pendingBodyWrenches.push_back({r->links.front(), force, torque});
}
void PhysicsScene3D::applyRagdollLinkForce(RagdollHandle3D h, std::uint32_t link, Vec3 force, Vec3 torque) {
    auto* r = m_impl->ragdoll(h);
    if (!r || r->frozen || link >= r->links.size() || !finite(force) || !finite(torque)) return;
    r->externalForceMagnitude += force.length() + torque.length() / std::max(0.2f, r->profile.standingRootHeightMeters);
    m_impl->pendingBodyWrenches.push_back({r->links[link], force, torque});
}
} // namespace MatterEngine
