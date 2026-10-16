#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

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
void PhysicsScene3D::setRagdollFrictionScale(RagdollHandle3D h, float v) { if (std::isfinite(v)) m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetFrictionScale, std::clamp(v,0.0f,2.0f)}); }
void PhysicsScene3D::setRagdollFrozen(RagdollHandle3D h, bool v) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetFrozen, v ? 1.0f : 0.0f}); }
void PhysicsScene3D::setRagdollKinematic(RagdollHandle3D h, bool v) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetKinematic, v ? 1.0f : 0.0f}); }
void PhysicsScene3D::removeRagdollVerticalSpin(RagdollHandle3D h) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::RemoveVerticalSpin, 0.0f}); }
void PhysicsScene3D::setRagdollGuided(RagdollHandle3D h, bool v) { m_impl->pendingRagdollCommands.push_back({h, Impl::RagdollCommandType::SetGuided, v ? 1.0f : 0.0f}); }
void PhysicsScene3D::setRagdollGuidePose(RagdollHandle3D h, std::span<const Vec3> positions,
    std::span<const Quaternion> orientations, std::span<const float> authority) {
    auto* r = m_impl->ragdoll(h);
    if (!r || positions.size() != r->links.size() || orientations.size() != r->links.size()
        || (!authority.empty() && authority.size() != r->links.size())) return;
    for (std::size_t i = 0; i < positions.size(); ++i)
        if (!finite(positions[i]) || !finite(orientations[i]) || (!authority.empty() && !std::isfinite(authority[i]))) return;
    r->guidePositions.assign(positions.begin(), positions.end());
    r->guideOrientations.assign(orientations.begin(), orientations.end());
    r->guideAuthority.assign(r->links.size(), 1.0f);
    for (std::size_t i = 0; i < authority.size(); ++i) r->guideAuthority[i] = std::clamp(authority[i], 0.0f, 1.0f);
    r->guidePoseStaged = true;
}
void PhysicsScene3D::setRagdollKinematicPose(RagdollHandle3D h, std::span<const Vec3> positions,
    std::span<const Quaternion> orientations) {
    auto* r = m_impl->ragdoll(h);
    if (!r || positions.size() != r->links.size() || orientations.size() != r->links.size()) return;
    for (std::size_t i = 0; i < positions.size(); ++i)
        if (!finite(positions[i]) || !finite(orientations[i])) return;
    r->kinematicPositions.assign(positions.begin(), positions.end());
    r->kinematicOrientations.assign(orientations.begin(), orientations.end());
    r->kinematicPoseStaged = true;
}
void PhysicsScene3D::setRagdollActiveDriveTargets(RagdollHandle3D h, std::span<const RagdollDriveTarget3D> targets, bool gravity) {
    auto* r = m_impl->ragdoll(h);
    if (!r) return;
    r->stagedActiveTargets.clear();
    for (const auto& target : targets) {
        if (!target.linkIndex || target.linkIndex >= r->links.size() || static_cast<unsigned>(target.axis) >= 3
            || !std::isfinite(target.positionRadians) || !std::isfinite(target.velocityRadiansPerSecond)
            || !std::isfinite(target.feedforwardTorqueNewtonMeters) || !std::isfinite(target.stiffnessScale)
            || !std::isfinite(target.dampingScale) || !std::isfinite(target.maximumTorqueScale)
            || !std::isfinite(target.gravityCompensationScale)) continue;
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
namespace {
// Giro maximo dos links guiados (rad/s): o do perfil (24) cortava o chute e o
// carrinho - o link ficava para tras da pose.
constexpr float GuidedMaxAngularVelocity = 120.0f;
constexpr float PhysicalMaxAngularVelocity = 24.0f;
}
// O modo guiado liga/desliga: gravidade, giro maximo, limites das juntas (so
// o ponto preso; na volta, os do perfil) e motores (desligados; o modo fisico
// os religa em applyRagdollDrives).
static void setRagdollGuidedState(PhysicsScene3D::Impl& impl, PhysicsScene3D::Impl::RagdollRecord& r, bool guided) {
    if (guided == r.guided) return;
    auto& bi = impl.system->GetBodyInterface();
    for (const JPH::BodyID id : r.links) {
        bi.SetGravityFactor(id, guided ? 0.0f : 1.0f);
        bi.SetMaxAngularVelocity(id, guided ? GuidedMaxAngularVelocity : PhysicalMaxAngularVelocity);
    }
    if (guided)
        for (auto id : r.links) std::erase_if(impl.pendingBodyWrenches, [id](const auto& w) { return w.body == id; });
    for (std::size_t i = 1; i < r.joints.size(); ++i) {
        auto& constraint = *r.joints[i].constraint;
        const auto& link = r.profile.links[i];
        JPH::Vec3 low = JPH::Vec3::sZero(), high = JPH::Vec3::sZero();
        for (int a = 0; a < 3; ++a) {
            const auto& limit = link.inboundJoint.axes[a];
            low.SetComponent(a, guided ? -JPH::JPH_PI : limit.enabled ? limit.minimumRadians : 0.0f);
            high.SetComponent(a, guided ? JPH::JPH_PI : limit.enabled ? limit.maximumRadians : 0.0f);
            if (guided) constraint.SetMotorState(static_cast<JPH::SixDOFConstraint::EAxis>(a + 3), JPH::EMotorState::Off);
        }
        constraint.SetRotationLimits(low, high);
    }
    r.guided = guided;
    r.state.guided = guided;
    r.guideTouched = false;
}
void PhysicsScene3D::Impl::applyPendingRagdollCommands() {
    for (const auto& command : pendingRagdollCommands) {
        auto* r = ragdoll(command.handle);
        if (!r) continue;
        // (A escala entra no atrito de cada contato do ragdoll, no listener
        // de contatos - ver recordContact.)
        if (command.type == RagdollCommandType::SetFrictionScale) {
            r->frictionScale = command.value;
            continue;
        }
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
        case RagdollCommandType::SetKinematic: {
            const bool kinematic = command.value != 0;
            if (kinematic == r->kinematic || r->frozen) break;
            if (kinematic) setRagdollGuidedState(*this, *r, false);
            auto& bi = system->GetBodyInterface();
            for (const JPH::BodyID id : r->links)
                bi.SetMotionType(id, kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic,
                    JPH::EActivation::Activate);
            // (Cinematico: forcas pendentes nao valem; a pose antiga tambem nao.)
            if (kinematic)
                for (auto id : r->links) std::erase_if(pendingBodyWrenches, [id](const auto& w) { return w.body == id; });
            r->kinematic = kinematic;
            r->kinematicPoseStaged = false;
            r->state.kinematic = kinematic;
            break;
        }
        case RagdollCommandType::RemoveVerticalSpin: {
            if (r->kinematic || r->frozen) break;
            auto& bi = system->GetBodyInterface();
            struct Link { JPH::BodyID id; JPH::Vec3 position, velocity, angular; float mass; JPH::Mat44 inertia; };
            std::vector<Link> links;
            JPH::Vec3 center = JPH::Vec3::sZero(), momentum = JPH::Vec3::sZero();
            float total = 0.0f;
            for (const JPH::BodyID id : r->links) {
                JPH::BodyLockRead lock(system->GetBodyLockInterface(), id);
                if (!lock.Succeeded() || !lock.GetBody().IsDynamic()) continue;
                const JPH::Body& body = lock.GetBody();
                const float mass = 1.0f / std::max(1e-6f, body.GetMotionProperties()->GetInverseMass());
                const JPH::Mat44 inverse = body.GetInverseInertia();
                links.push_back({ id, JPH::Vec3(body.GetCenterOfMassPosition()), body.GetLinearVelocity(),
                    body.GetAngularVelocity(), mass, inverse.Inversed3x3() });
                center += links.back().position * mass;
                momentum += links.back().velocity * mass;
                total += mass;
            }
            if (links.empty() || total <= 0.0f) break;
            center /= total;
            const JPH::Vec3 drift = momentum / total;
            // Momento angular vertical no centro de massa e a inercia do
            // corpo inteiro em volta da vertical.
            float spin = 0.0f, inertia = 0.0f;
            for (const Link& link : links) {
                const JPH::Vec3 arm = link.position - center;
                const JPH::Vec3 relative = link.velocity - drift;
                spin += link.mass * (arm.GetX() * relative.GetY() - arm.GetY() * relative.GetX())
                    + (link.inertia * link.angular).GetZ();
                inertia += link.mass * (arm.GetX() * arm.GetX() + arm.GetY() * arm.GetY())
                    + (link.inertia * JPH::Vec3(0.0f, 0.0f, 1.0f)).GetZ();
            }
            if (inertia <= 1e-6f) break;
            const float omega = spin / inertia;
            for (const Link& link : links) {
                const JPH::Vec3 arm = link.position - center;
                bi.SetLinearVelocity(link.id, link.velocity - JPH::Vec3(-arm.GetY() * omega, arm.GetX() * omega, 0.0f));
                bi.SetAngularVelocity(link.id, link.angular - JPH::Vec3(0.0f, 0.0f, omega));
            }
            break;
        }
        case RagdollCommandType::SetGuided: {
            const bool guided = command.value != 0;
            if (guided == r->guided || r->frozen) break;
            if (guided && r->kinematic) {
                // (Do cinematico: dinamico com a velocidade que ele tinha.)
                auto& bi = system->GetBodyInterface();
                for (const JPH::BodyID id : r->links)
                    bi.SetMotionType(id, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
                r->kinematic = false;
                r->kinematicPoseStaged = false;
                r->state.kinematic = false;
            }
            setRagdollGuidedState(*this, *r, guided);
            break;
        }
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
        // (Guiado: os motores ficam desligados - a pose vem das velocidades.)
        if (!slot.record || slot.record->frozen || slot.record->kinematic || slot.record->guided) continue;
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
                const float gravity = r.gravityCompensationEnabled
                    ? r.gravityCompensator.jointTorques()[i][a]
                        * std::clamp(target.gravityCompensationScale, 0.0f, 1.0f)
                    : 0;
                const float ff = std::clamp(target.feedforwardTorqueNewtonMeters + gravity, -maximum, maximum);
                feedforward.SetComponent(a, ff);
                runtime.appliedTargets[static_cast<std::size_t>(a)] = angles[a];
                runtime.appliedTargetVelocities[static_cast<std::size_t>(a)] = target.velocityRadiansPerSecond;
                runtime.appliedFeedforward[static_cast<std::size_t>(a)] = ff;
                runtime.appliedTorqueLimit[static_cast<std::size_t>(a)] = maximum;
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
// O amortecimento da guia de autoridade parcial (ver applyRagdollGuidePoses):
// 0,7 quase nao passa do alvo e deixa as pernas acompanharem mais que 1.
constexpr float GuideDampingRatio = 0.7f;

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
        r.state.externalPushNewtonSeconds = impulse;
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
        if (r.frozen || r.kinematic || !r.active || !staged || grabRagdoll == r.handle) {
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
// Modo animado: cada link cinematico vai a pose do tick no fim do passo (o
// Jolt da a velocidade que chega la - os contatos com o que e dinamico usam
// essa velocidade). Sem pose nova, o link para onde esta.
void PhysicsScene3D::Impl::applyRagdollKinematics(float dt) {
    auto& bi = system->GetBodyInterface();
    for (auto& slot : ragdollSlots) {
        if (!slot.record || slot.record->frozen || !slot.record->kinematic) continue;
        RagdollRecord& r = *slot.record;
        if (!r.kinematicPoseStaged) {
            for (const JPH::BodyID id : r.links) {
                bi.SetLinearVelocity(id, JPH::Vec3::sZero());
                bi.SetAngularVelocity(id, JPH::Vec3::sZero());
            }
            continue;
        }
        r.kinematicPoseStaged = false;
        for (std::size_t i = 0; i < r.links.size(); ++i)
            bi.MoveKinematic(r.links[i], toJolt(r.kinematicPositions[i]),
                toJolt(r.kinematicOrientations[i]).Normalized(), dt);
    }
}
namespace {
// Vetor de rotacao (eixo * angulo) de um quaternion.
JPH::Vec3 rotationVector(JPH::Quat q) {
    q = q.Normalized();
    if (q.GetW() < 0.0f) q = -q;
    const JPH::Vec3 v = q.GetXYZ();
    const float n = v.Length();
    return n < 1e-7f ? JPH::Vec3::sZero() : v * (2.0f * std::atan2(n, q.GetW()) / n);
}
} // namespace
// Corpo guiado, antes do passo: cada link recebe a velocidade (do centro de
// massa e de giro) que o leva a pose pedida no fim do passo - com autoridade
// 1 exatamente (o MoveKinematic do Jolt, mas com o corpo dinamico); abaixo
// disso, a fracao do caminho de guideBlend (a mesma curva do guia da raiz).
// Fora do modo guiado a gravidade do passo e descontada.
void PhysicsScene3D::Impl::applyRagdollGuidePoses(float dt) {
    auto& bi = system->GetBodyInterface();
    const JPH::Vec3 gravity = system->GetGravity();
    for (auto& slot : ragdollSlots) {
        if (!slot.record) continue;
        RagdollRecord& r = *slot.record;
        const bool staged = r.guidePoseStaged;
        r.guidePoseStaged = false;
        r.guidePoseApplied = false;
        r.guideTouched = false;
        if (!staged || r.frozen || r.kinematic || grabRagdoll == r.handle) {
            r.guidePreviousKnown = false;
            continue;
        }
        r.guidePoseApplied = true;
        if (r.guidePreviousCenters.size() != r.links.size()) {
            r.guidePreviousCenters.assign(r.links.size(), JPH::Vec3::sZero());
            r.guidePreviousRotations.assign(r.links.size(), JPH::Quat::sIdentity());
            r.guidePreviousKnown = false;
        }
        for (std::size_t i = 0; i < r.links.size(); ++i) {
            const float authority = r.guideAuthority[i];
            const JPH::BodyID id = r.links[i];
            JPH::RVec3 position;
            JPH::Quat rotation;
            bi.GetPositionAndRotation(id, position, rotation);
            const JPH::Vec3 center = bi.GetCenterOfMassPosition(id);
            const JPH::Vec3 local = rotation.Conjugated() * (center - position);
            const JPH::Quat targetRotation = toJolt(r.guideOrientations[i]).Normalized();
            const JPH::Vec3 targetCenter = toJolt(r.guidePositions[i]) + targetRotation * local;
            const JPH::Vec3 previousCenter = r.guidePreviousCenters[i];
            const JPH::Quat previousRotation = r.guidePreviousRotations[i];
            r.guidePreviousCenters[i] = targetCenter;
            r.guidePreviousRotations[i] = targetRotation;
            if (authority <= 0.0f) continue;
            const float blend = guideBlend(authority, dt);
            JPH::Vec3 linear, angular;
            if (authority >= 0.999f) {
                // Inteira: o alvo num passo (o corpo da animacao).
                linear = (targetCenter - center) / dt;
                angular = rotationVector(targetRotation * rotation.Conjugated()) / dt;
            } else {
                // Parcial (a ponte, a saida do guiado, a guiagem de ossos):
                // a velocidade da pose mais a correcao do erro num tempo que
                // deixa a mistura amortecida - fechando o erro todo num passo,
                // a mistura parcial era uma mola quase sem amortecimento e o
                // corpo passava da pose (a ponte correndo lancava a pelve a 7
                // m/s; plano v4, etapa C). (tau = 4 zeta^2 dt / mistura; com a
                // velocidade da pose inteira e um tempo fixo, os saltos da pose
                // viravam picos de 2000 m/s2.)
                const float tau = std::max(dt, GuideDampingRatio * GuideDampingRatio * 4.0f * dt / std::max(1e-3f, blend));
                JPH::Vec3 poseVelocity = JPH::Vec3::sZero(), poseSpin = JPH::Vec3::sZero();
                if (r.guidePreviousKnown) {
                    poseVelocity = (targetCenter - previousCenter) / dt;
                    poseSpin = rotationVector(targetRotation * previousRotation.Conjugated()) / dt;
                }
                const float carry = 1.0f - dt / tau;
                linear = poseVelocity * carry + (targetCenter - center) / tau;
                angular = poseSpin * carry + rotationVector(targetRotation * rotation.Conjugated()) / tau;
            }
            if (!r.guided) linear -= gravity * dt;
            const JPH::Vec3 v = bi.GetLinearVelocity(id);
            const JPH::Vec3 w = bi.GetAngularVelocity(id);
            bi.SetLinearAndAngularVelocity(id, v + (linear - v) * blend, w + (angular - w) * blend);
        }
        r.guidePreviousKnown = true;
        r.ragdoll->Activate();
    }
}
// Corpo guiado, depois do passo: o quanto os links ficaram da pose e, no modo
// guiado sem nada de fora encostando (nem forca externa), a pose exata - a
// integracao do giro e o solver deixam milimetros, e livre o corpo tem de ser
// o da animacao. Encostando, o que o passo deu fica: e a reacao dele.
void PhysicsScene3D::Impl::finishRagdollGuidePoses() {
    auto& bi = system->GetBodyInterface();
    for (auto& slot : ragdollSlots) {
        if (!slot.record) continue;
        RagdollRecord& r = *slot.record;
        r.state.guideErrorMeters = 0.0f;
        if (!r.guidePoseApplied || r.frozen) continue;
        float error = 0.0f;
        for (std::size_t i = 0; i < r.links.size(); ++i)
            if (r.guideAuthority[i] > 0.0f)
                error = std::max(error, (fromJolt(bi.GetPosition(r.links[i])) - r.guidePositions[i]).length());
        r.state.guideErrorMeters = error;
        if (!r.guided || r.guideTouched || r.externalForceMagnitude > 0.0f || error > 0.05f) continue;
        for (std::size_t i = 0; i < r.links.size(); ++i)
            if (r.guideAuthority[i] >= 0.999f)
                bi.SetPositionAndRotation(r.links[i], toJolt(r.guidePositions[i]),
                    toJolt(r.guideOrientations[i]).Normalized(), JPH::EActivation::DontActivate);
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
void PhysicsScene3D::applyRagdollControlLinkForce(RagdollHandle3D h, std::uint32_t link, Vec3 force, Vec3 torque) {
    auto* r = m_impl->ragdoll(h);
    if (r && !r->frozen && link < r->links.size() && finite(force) && finite(torque))
        m_impl->pendingBodyWrenches.push_back({r->links[link], force, torque});
}
void PhysicsScene3D::applyRagdollLinkForce(RagdollHandle3D h, std::uint32_t link, Vec3 force, Vec3 torque) {
    auto* r = m_impl->ragdoll(h);
    if (!r || r->frozen || link >= r->links.size() || !finite(force) || !finite(torque)) return;
    r->externalForceMagnitude += force.length() + torque.length() / std::max(0.2f, r->profile.standingRootHeightMeters);
    m_impl->pendingBodyWrenches.push_back({r->links[link], force, torque});
}
} // namespace MatterEngine
