#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace MatterEngine {
PhysicsScene3D::Impl::RagdollRecord* PhysicsScene3D::Impl::ragdoll(RagdollHandle3D h) {
    return h.index < ragdollSlots.size() && ragdollSlots[h.index].generation == h.generation ? ragdollSlots[h.index].record.get() : nullptr;
}
const PhysicsScene3D::Impl::RagdollRecord* PhysicsScene3D::Impl::ragdoll(RagdollHandle3D h) const {
    return h.index < ragdollSlots.size() && ragdollSlots[h.index].generation == h.generation ? ragdollSlots[h.index].record.get() : nullptr;
}
PhysicsScene3D::Impl::RagdollRecord* PhysicsScene3D::Impl::ragdollFromId(JPH::BodyID id, std::uint32_t& link) {
    auto identity = JoltDetail::unpackBodyIdentity(system->GetBodyInterface().GetUserData(id));
    if (identity.kind != JoltDetail::BodyKind::RagdollLink || identity.slotIndex >= ragdollSlots.size()) return nullptr;
    link = identity.linkIndex;
    auto* record = ragdollSlots[identity.slotIndex].record.get();
    return record && link < record->links.size() && record->links[link] == id ? record : nullptr;
}
namespace {

// Autocolisao de um ragdoll: quem colide com quem dentro do mesmo corpo.
//
//   1. Pai-filho direto nunca colide (a junta ja os prende).
//   2. Par que se sobrepoe na pose de repouso nao colide: nao tem como ser
//      separado sem deformar o proprio repouso, so injeta forca a cada passo.
//   3. Par ancestral-descendente na mesma cadeia que fica a menos de 1 cm em
//      repouso tambem nao colide. Na coluna, segmentos vizinhos-de-vizinho
//      quase se tocam (Abdomen x UpperChest a 2,5 mm): flexionar o tronco os
//      faria bater depois de poucos graus, um batente artificial - quem
//      governa esse movimento e o elo entre eles e os limites de junta.
//   4. Pares declarados no perfil (selfCollisionIgnoredPairs), cada um com o
//      motivo. Sao contatos falsos da forma aproximada dos colisores dentro
//      da amplitude normal do corpo - no ALS, a capsula do abdomen desce ate
//      o quadril e a coxa bateria nela com 45 graus de flexao.
//
// O que continua colidindo, de proposito: Chest x UpperArm (ancestral-
// descendente, mas longe em repouso porque o bind e pose em T: e a barreira
// que impede o braco de atravessar o peito, protegida pelo teste adversarial)
// e coxa x coxa (irmas, nao ancestrais - as pernas nao podem se atravessar).
//
// A funcao pronta do Jolt, DisableParentChildCollisions(pose, distancia),
// aplica a mesma distancia a TODOS os pares e desligaria coxa x coxa, entao a
// tabela e montada aqui. O espelho em Python e tools/animation/validate.py.
constexpr float ChainRestClearanceMeters = 0.01f;

bool isAncestor(const RagdollProfile3D& profile, std::size_t ancestor,
    std::size_t link) {
    for (int current = profile.links[link].parentIndex; current >= 0;
            current = profile.links[static_cast<std::size_t>(current)].parentIndex) {
        if (static_cast<std::size_t>(current) == ancestor) return true;
    }
    return false;
}

bool shapesWithin(const JPH::RagdollSettings::Part& first,
    const JPH::RagdollSettings::Part& second, float distance) {
    const JPH::Shape* firstShape = first.GetShape();
    const JPH::Shape* secondShape = second.GetShape();
    JPH::Vec3 firstScale;
    JPH::Vec3 secondScale;
    const JPH::Mat44 firstTransform = JPH::Mat44::sRotationTranslation(
        first.mRotation, first.mPosition).PreTranslated(
            firstShape->GetCenterOfMass()).Decompose(firstScale);
    const JPH::Mat44 secondTransform = JPH::Mat44::sRotationTranslation(
        second.mRotation, second.mPosition).PreTranslated(
            secondShape->GetCenterOfMass()).Decompose(secondScale);
    JPH::CollideShapeSettings settings;
    settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
    settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
    settings.mMaxSeparationDistance = distance;
    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
    JPH::CollisionDispatch::sCollideShapeVsShape(firstShape, secondShape,
        firstScale, secondScale, firstTransform, secondTransform,
        JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(), settings,
        collector);
    return collector.HadHit();
}

// As partes ja estao na pose de spawn, que e o bind posicionado no mundo; as
// consultas so dependem das poses relativas.
bool declaredIgnored(const RagdollProfile3D& profile, std::size_t a,
    std::size_t b) {
    const std::string& first = profile.links[a].id;
    const std::string& second = profile.links[b].id;
    for (const RagdollIgnoredCollisionPair3D& pair
            : profile.selfCollisionIgnoredPairs) {
        if ((pair.firstLinkId == first && pair.secondLinkId == second)
            || (pair.firstLinkId == second && pair.secondLinkId == first)) {
            return true;
        }
    }
    return false;
}

void buildSelfCollisionFilter(JPH::RagdollSettings& settings,
    const RagdollProfile3D& profile) {
    const std::size_t count = settings.mParts.size();
    JPH::Ref<JPH::GroupFilterTable> table =
        new JPH::GroupFilterTable(static_cast<JPH::uint>(count));
    for (std::size_t a = 0; a < count; ++a) {
        for (std::size_t b = a + 1; b < count; ++b) {
            const bool parentChild =
                profile.links[b].parentIndex == static_cast<int>(a)
                || profile.links[a].parentIndex == static_cast<int>(b);
            const bool sameChain = isAncestor(profile, a, b)
                || isAncestor(profile, b, a);
            const float restDistance = sameChain ? ChainRestClearanceMeters
                                                 : 0.0f;
            if (parentChild || declaredIgnored(profile, a, b)
                || shapesWithin(settings.mParts[a], settings.mParts[b],
                    restDistance)) {
                table->DisableCollision(static_cast<JPH::CollisionGroup::SubGroupID>(a),
                    static_cast<JPH::CollisionGroup::SubGroupID>(b));
            }
        }
    }
    for (std::size_t index = 0; index < count; ++index) {
        settings.mParts[index].mCollisionGroup.SetSubGroupID(
            static_cast<JPH::CollisionGroup::SubGroupID>(index));
        settings.mParts[index].mCollisionGroup.SetGroupFilter(table);
    }
}

} // namespace

RagdollHandle3D PhysicsScene3D::createRagdoll(const RagdollProfile3D& profile, const RagdollSpawnDefinition3D& spawn) {
    validateRagdollProfileOrThrow3D(profile);
    if (!std::isfinite(spawn.rigidityPercent)) throw std::invalid_argument("Invalid ragdoll rigidity");
    const auto index = m_impl->freeRagdollSlots.empty() ? static_cast<std::uint32_t>(m_impl->ragdollSlots.size()) : m_impl->freeRagdollSlots.back();
    auto record = std::make_unique<Impl::RagdollRecord>();
    record->profile = profile;
    record->stepInteractions.resize(profile.links.size() * 6);
    record->state.interactions.reserve(profile.links.size() * 6);
    record->entityId = spawn.entityId;
    record->active = spawn.active;
    record->rigidityPercent = record->passiveRigidityPercent = std::clamp(spawn.rigidityPercent, 0.0f, 100.0f);
    record->indexing = buildArticulationIndexing3D(profile);
    record->gravityCompensator.prepare(profile);
    record->gravityPoses.resize(profile.links.size());
    record->joints.resize(profile.links.size());
    record->state.links.resize(profile.links.size());
    record->state.joints.resize(profile.links.size());
    record->settings = new JPH::RagdollSettings;
    auto& settings = *record->settings;
    settings.mSkeleton = new JPH::Skeleton;
    const auto orientation = spawn.orientation.normalized();
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const auto& link = profile.links[i];
        settings.mSkeleton->AddJoint(link.id.c_str(), link.parentIndex);
        PhysicsShape3D collider;
        collider.type = link.collider.shape == RagdollColliderShape3D::Box ? PhysicsShapeType3D::Box : PhysicsShapeType3D::Capsule;
        collider.halfExtents = link.collider.boxHalfExtents;
        collider.radius = link.collider.radiusMeters;
        // Conica: a tampa +X do perfil vira a +Y do Jolt (a rotacao abaixo).
        collider.capsuleTopRadius = link.collider.radiusAtPositiveXMeters;
        collider.capsuleHalfHeight = std::max(0.001f, ragdollCapsuleHalfSegment3D(link.collider));
        collider.localPosition = link.collider.localPosition;
        collider.localOrientation = link.collider.localOrientation;
        // Ragdoll assets use longitudinal X, unlike the general PhysicsShape capsule (Y).
        if (collider.type == PhysicsShapeType3D::Capsule)
            collider.localOrientation = collider.localOrientation * Quaternion::fromAxisAngle({0,0,1}, -JPH::JPH_PI * 0.5f);
        collider.materialId = link.collider.materialId;
        auto shape = createJoltShape(collider, false);
        const auto comOffset = toJolt(link.centerOfMassLocal) - shape->GetCenterOfMass();
        if (!comOffset.IsNearZero()) {
            auto result = JPH::OffsetCenterOfMassShapeSettings(comOffset, shape).Create();
            if (result.HasError()) throw std::runtime_error(result.GetError().c_str());
            shape = result.Get();
        }
        JPH::RagdollSettings::Part part;
        part.SetShape(shape);
        part.mPosition = toJolt(spawn.pelvisPosition + orientation.rotate(link.modelPosition));
        part.mRotation = toJolt(orientation * link.modelOrientation);
        part.mMotionType = JPH::EMotionType::Dynamic;
        part.mObjectLayer = JoltObjectLayers::Ragdoll;
        part.mLinearDamping = 0.04f;
        part.mAngularDamping = 0.16f;
        part.mMaxAngularVelocity = 24;
        part.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        part.mMassPropertiesOverride.mMass = profile.totalMassKg * link.massFraction;
        const auto& material = m_impl->contactMaterial(collider.materialId);
        part.mFriction = material.dynamicFriction;
        part.mRestitution = material.restitution;
        if (link.parentIndex >= 0) {
            auto* joint = new JPH::SixDOFConstraintSettings;
            const auto frame = toJolt(orientation * link.inboundJoint.frameModelOrientation);
            joint->mPosition1 = joint->mPosition2 = toJolt(spawn.pelvisPosition + orientation.rotate(link.inboundJoint.anchorModelPosition));
            joint->mAxisX1 = joint->mAxisX2 = frame * JPH::Vec3::sAxisX();
            joint->mAxisY1 = joint->mAxisY2 = frame * JPH::Vec3::sAxisY();
            // Iteracoes de solver das juntas, com 1 collision step por tick
            // (ver JoltScene3D). Escolhidas por estudo de variancia, nao por
            // uma execucao: a pilha de 22 ragdolls e caotica e o pior caso de
            // separacao de ancora muda com qualquer perturbacao (Debug e
            // RelWithDebInfo divergem). Pior caso em 8 sementes com spawn
            // perturbado em +/-1 mm (media entre parenteses):
            //   12/20  1,25 cm (1,16)   - reprovou o gate de 1,5 cm em Debug
            //   20/20  1,09 cm (0,90)   - escolhido: 1,66 ms por passo
            //   24/20  1,00 cm (0,90)   - posicao satura; 32 nao melhora
            //   24/12  1,79 cm (1,16)   - cortar velocidade piora muito
            //   4 subpassos 8/20  0,84 cm (0,72) - primeira versao, 3,87 ms
            // A ilha inteira paga a maior contagem pedida por um constraint,
            // entao isto vale onde ha ragdoll, nao para a cena toda.
            joint->mNumPositionStepsOverride = 20;
            joint->mNumVelocityStepsOverride = 20;
            joint->mSwingType = JPH::ESwingType::Pyramid;
            for (int a = 0; a < 3; ++a) {
                joint->MakeFixedAxis(static_cast<JPH::SixDOFConstraintSettings::EAxis>(a));
                const auto axis = static_cast<JPH::SixDOFConstraintSettings::EAxis>(a+3);
                const auto& limit = link.inboundJoint.axes[a];
                if (limit.enabled) joint->SetLimitedAxis(axis, limit.minimumRadians, limit.maximumRadians);
                else joint->MakeFixedAxis(axis);
                joint->mMotorSettings[axis].mSpringSettings = {JPH::ESpringMode::StiffnessAndDamping, limit.stiffness, limit.damping};
                joint->mMotorSettings[axis].SetTorqueLimit(limit.maximumTorque);
            }
            part.mToParent = joint;
        }
        settings.mParts.push_back(part);
    }
    if (!settings.Stabilize()) throw std::runtime_error("Jolt: invalid ragdoll inertia");
    // Stabilize conditions inertias for a constraint chain. Preserve authored
    // masses so gravity feedforward and gameplay momentum retain their units.
    for (std::size_t i=0; i<settings.mParts.size(); ++i)
        settings.mParts[i].mMassPropertiesOverride.ScaleToMass(profile.totalMassKg * profile.links[i].massFraction);
    settings.CalculateConstraintPriorities();
    // Autocolisao: ver buildSelfCollisionFilter.
    buildSelfCollisionFilter(settings, profile);
    settings.CalculateBodyIndexToConstraintIndex();
    settings.CalculateConstraintIndexToBodyIdxPair();
    record->ragdoll = settings.CreateRagdoll(index + 1, 0, m_impl->system.get());
    if (!record->ragdoll) throw std::runtime_error("Jolt: ragdoll body capacity exceeded");
    const auto& ids = record->ragdoll->GetBodyIDs();
    record->links.assign(ids.begin(), ids.end());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        m_impl->system->GetBodyInterface().SetUserData(ids[i], JoltDetail::packBodyIdentity({JoltDetail::BodyKind::RagdollLink, index, static_cast<std::uint32_t>(i)}));
        if (i) record->joints[i].constraint = static_cast<JPH::SixDOFConstraint*>(record->ragdoll->GetConstraint(settings.GetConstraintIndexForBodyIndex(static_cast<int>(i))));
    }
    if (index == m_impl->ragdollSlots.size()) m_impl->ragdollSlots.emplace_back();
    else m_impl->freeRagdollSlots.pop_back();
    record->handle = {index, m_impl->ragdollSlots[index].generation};
    const auto handle = record->handle;
    record->ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);
    m_impl->publishRagdoll(*record);
    m_impl->ragdollSlots[index].record = std::move(record);
    return handle;
}
void PhysicsScene3D::destroyRagdoll(RagdollHandle3D h) {
    auto* r = m_impl->ragdoll(h);
    if (!r) return;
    if (m_impl->grabRagdoll == h) endGrab();
    for (auto id : r->links) std::erase_if(m_impl->pendingBodyWrenches, [id](const auto& w) { return w.body == id; });
    if (!r->frozen) r->ragdoll->RemoveFromPhysicsSystem();
    auto& slot = m_impl->ragdollSlots[h.index];
    slot.record.reset();
    ++slot.generation;
    m_impl->freeRagdollSlots.push_back(h.index);
}
bool PhysicsScene3D::contains(RagdollHandle3D h) const { return m_impl->ragdoll(h) != nullptr; }
} // namespace MatterEngine
