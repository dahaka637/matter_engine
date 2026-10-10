#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"
#include "Engine/Physics/RagdollInteractions3D.hpp"
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <algorithm>
#include <tuple>

namespace MatterEngine {
// O corpo guiado nao colide com os proprios links (a pose da animacao cruza
// bracos e coxas), nem com a capsula, nem com o chao: superficie que nao e
// dinamica com a normal para cima (piso, topo de degrau e de mureta) - a pose
// atravessa o chao alguns milimetros e os pes deslizam. Com parede, meio-fio
// e a frente de uma mureta ele bate de verdade (pedido do usuario, 05/10
// noite: no padrao o corpo entrava na parede e so virava fisico ja dentro
// dela, sem o choque). normalAB: de a para b.
bool PhysicsScene3D::Impl::guidedIgnores(const JPH::Body& a, const JPH::Body& b, JPH::Vec3Arg normalAB) const {
    const auto ia = JoltDetail::unpackBodyIdentity(a.GetUserData());
    const auto ib = JoltDetail::unpackBodyIdentity(b.GetUserData());
    const auto guided = [&](const JoltDetail::BodyIdentity& id) {
        if (id.kind != JoltDetail::BodyKind::RagdollLink || id.slotIndex >= ragdollSlots.size()) return false;
        const auto* r = ragdollSlots[id.slotIndex].record.get();
        return r != nullptr && r->guided;
    };
    const bool ga = guided(ia), gb = guided(ib);
    if (!ga && !gb) return false;
    if (ia.kind == JoltDetail::BodyKind::RagdollLink && ib.kind == ia.kind && ia.slotIndex == ib.slotIndex) return true;
    const JPH::Body& other = ga ? b : a;
    const auto otherKind = (ga ? ib : ia).kind;
    if (otherKind == JoltDetail::BodyKind::Character) return true;
    if (other.IsDynamic()) return false;
    // A normal da superficie do outro, apontando para o link guiado.
    const float length = normalAB.Length();
    if (length < 1e-6f) return true;
    const JPH::Vec3 surface = (ga ? -normalAB : normalAB) / length;
    return surface.GetZ() > GuidedFloorNormal;
}
JPH::ValidateResult PhysicsScene3D::Impl::OnContactValidate(const JPH::Body& a, const JPH::Body& b,
    JPH::RVec3Arg, const JPH::CollideShapeResult& collision) {
    return guidedIgnores(a, b, collision.mPenetrationAxis) ? JPH::ValidateResult::RejectContact
        : JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
}
void PhysicsScene3D::Impl::OnContactAdded(const JPH::Body& a, const JPH::Body& b,
    const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) {
    try { recordContact(a,b,manifold,settings,true); } catch (...) { callbackFailed = true; }
}
void PhysicsScene3D::Impl::OnContactPersisted(const JPH::Body& a, const JPH::Body& b,
    const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) {
    try { recordContact(a,b,manifold,settings,false); } catch (...) { callbackFailed = true; }
}
void PhysicsScene3D::Impl::recordContact(const JPH::Body& a, const JPH::Body& b,
    const JPH::ContactManifold& manifold, JPH::ContactSettings& contact, bool added) {
    ++allContactPairs;
    // (Um contato guardado de antes de o corpo virar guiado.)
    if (guidedIgnores(a, b, manifold.mWorldSpaceNormal)) {
        contact.mIsSensor = true;
        return;
    }
    const auto ia = JoltDetail::unpackBodyIdentity(a.GetUserData());
    const auto ib = JoltDetail::unpackBodyIdentity(b.GetUserData());
    const bool characterPair = ia.kind == JoltDetail::BodyKind::Character || ib.kind == JoltDetail::BodyKind::Character;
    // Capsula de navegacao pura: o corpo interno nao toca nada que se move.
    const bool proxyPair = characterPair && characterNavigationProxy
        && (!a.IsStatic() && !b.IsStatic());
    if (proxyPair || (characterPair && (characterState.flying || (characterIgnoreRagdolls
        && (ia.kind == JoltDetail::BodyKind::RagdollLink || ib.kind == JoltDetail::BodyKind::RagdollLink))))) {
        contact.mIsSensor = true;
        return;
    }
    const std::string ma(joltShapeMaterialId(a.GetShape()->GetMaterial(manifold.mSubShapeID1)));
    const std::string mb(joltShapeMaterialId(b.GetShape()->GetMaterial(manifold.mSubShapeID2)));
    const auto& materialA = contactMaterial(ma);
    const auto& materialB = contactMaterial(mb);
    contact.mCombinedFriction = (materialA.dynamicFriction + materialB.dynamicFriction) * 0.5f;
    // Escala de atrito do ragdoll (setRagdollFrictionScale: o carrinho
    // escorrega na grama). O atrito do contato e o dos materiais - o do corpo
    // no Jolt nao conta aqui.
    const auto frictionScale = [&](JoltDetail::BodyIdentity id) {
        if (id.kind != JoltDetail::BodyKind::RagdollLink || id.slotIndex >= ragdollSlots.size()) return 1.0f;
        const auto* r = ragdollSlots[id.slotIndex].record.get();
        return r != nullptr ? r->frictionScale : 1.0f;
    };
    contact.mCombinedFriction *= frictionScale(ia) * frictionScale(ib);
    contact.mCombinedRestitution = std::max(materialA.restitution, materialB.restitution);
    const auto record = [&](JoltDetail::BodyIdentity id) -> const BodyRecord* {
        return id.kind == JoltDetail::BodyKind::Body && id.slotIndex < bodySlots.size() ? bodySlots[id.slotIndex].record.get() : nullptr;
    };
    const auto ragdoll = [&](JoltDetail::BodyIdentity id) -> RagdollRecord* {
        if (id.kind != JoltDetail::BodyKind::RagdollLink || id.slotIndex >= ragdollSlots.size()) return nullptr;
        auto* r = ragdollSlots[id.slotIndex].record.get();
        return r && id.linkIndex < r->profile.links.size() ? r : nullptr;
    };
    const auto* ra = record(ia);
    const auto* rb = record(ib);
    auto* interactionA = ragdoll(ia);
    auto* interactionB = ragdoll(ib);
    auto* sa = interactionA && interactionA->profile.links[ia.linkIndex].collider.contactSensor ? interactionA : nullptr;
    auto* sb = interactionB && interactionB->profile.links[ib.linkIndex].collider.contactSensor ? interactionB : nullptr;
    // Own limbs still collide, but are not external support/impacts. Otherwise
    // bending a knee raises physicsBlend and repeatedly triggers recovery.
    if (ia.kind == JoltDetail::BodyKind::RagdollLink && ib.kind == ia.kind
        && ia.slotIndex == ib.slotIndex) {
        sa = nullptr; sb = nullptr; interactionA = nullptr; interactionB = nullptr;
    }
    // Ragdolls never feed prop acoustics, even on contact with the floor.
    const bool acoustic = ia.kind != JoltDetail::BodyKind::RagdollLink && ib.kind != JoltDetail::BodyKind::RagdollLink
        && ((ra && a.IsDynamic()) || (rb && b.IsDynamic()));
    if (!acoustic && !interactionA && !interactionB) return;
    JPH::CollisionEstimationResult response;
    JPH::EstimateCollisionResponse(a,b,manifold,response,contact.mCombinedFriction,contact.mCombinedRestitution);
    const auto mass = [](const JPH::Body& body) {
        return body.IsDynamic() ? 1.0f / body.GetMotionProperties()->GetInverseMass() : 0.0f;
    };
    const float massA = mass(a), massB = mass(b);
    const float effective = massA > 0 && massB > 0 ? massA*massB/(massA+massB) : std::max(massA,massB);
    // Only event storage is serialized. Never lock a Jolt body from its callback.
    const std::lock_guard lock(contactMutex);
    // O corpo guiado encostou em algo de fora (o passo nao termina na pose).
    if (interactionA != nullptr && interactionA->guided) interactionA->guideTouched = true;
    if (interactionB != nullptr && interactionB->guided) interactionB->guideTouched = true;
    if (acoustic) { ++reportedContactPairs; reportedContactPoints += manifold.mRelativeContactPointsOn1.size(); }
    for (JPH::uint i = 0; i < manifold.mRelativeContactPointsOn1.size(); ++i) {
        const auto point = manifold.GetWorldSpaceContactPointOn1(i);
        const auto normal = manifold.mWorldSpaceNormal;
        const auto relative = a.GetPointVelocity(point) - b.GetPointVelocity(point);
        const float approach = std::max(0.0f, relative.Dot(normal));
        const float tangential = (relative - normal * relative.Dot(normal)).Length();
        const float impulse = i < response.mContactImpulse.size() ? response.mContactImpulse[i] : 0;
        const auto motion = [](const JPH::Body& body) {
            return body.IsDynamic() ? RagdollContactMotion3D::Dynamic
                : body.IsKinematic() ? RagdollContactMotion3D::Kinematic
                : RagdollContactMotion3D::Static;
        };
        if (interactionA) accumulateRagdollInteraction3D(interactionA->stepInteractions,
            ia.linkIndex, motion(b), fromJolt(point), fromJolt(-normal),
            fromJolt(relative), impulse, true, true);
        if (interactionB) accumulateRagdollInteraction3D(interactionB->stepInteractions,
            ib.linkIndex, motion(a), fromJolt(point), fromJolt(normal),
            fromJolt(-relative), impulse, true, true);
        if (sa) sa->stepContacts.push_back({ia.linkIndex, fromJolt(point), fromJolt(-normal), impulse, tangential,
            b.IsDynamic(), motion(b), true});
        if (sb) sb->stepContacts.push_back({ib.linkIndex, fromJolt(point), fromJolt(normal), impulse, tangential,
            a.IsDynamic(), motion(a), true});
        if (!acoustic) continue;
        if (added) {
            ContactImpactEvent3D e;
            e.bodyA = ra ? ra->handle.index : InvalidPhysicsBodyIndex;
            e.bodyB = rb ? rb->handle.index : InvalidPhysicsBodyIndex;
            e.bodyIdA = ra ? ra->definition.entityId : 0;
            e.bodyIdB = rb ? rb->definition.entityId : 0;
            e.materialA = ma; e.materialB = mb;
            e.position = fromJolt(point); e.normal = fromJolt(normal);
            e.normalImpulseNewtonSeconds = impulse; e.approachSpeedMetersPerSecond = approach;
            e.effectiveMassKg = effective; e.transferredEnergyJoules = 0.5f * effective * approach * approach;
            e.massA = massA; e.massB = massB; e.staticA = !a.IsDynamic(); e.staticB = !b.IsDynamic();
            if (ra) { const auto& d=ra->definition; e.characteristicSizeA=d.characteristicSizeMeters; e.acousticGainA=d.acousticGain; e.acousticDampingA=d.acousticDamping; e.structureA=d.acousticStructure; }
            if (rb) { const auto& d=rb->definition; e.characteristicSizeB=d.characteristicSizeMeters; e.acousticGainB=d.acousticGain; e.acousticDampingB=d.acousticDamping; e.structureB=d.acousticStructure; }
            contactImpacts.push_back(std::move(e));
        } else {
            ContactSlideEvent3D e;
            e.bodyA = ra ? ra->handle.index : InvalidPhysicsBodyIndex;
            e.bodyB = rb ? rb->handle.index : InvalidPhysicsBodyIndex;
            e.bodyIdA = ra ? ra->definition.entityId : 0; e.bodyIdB = rb ? rb->definition.entityId : 0;
            e.materialA=ma; e.materialB=mb; e.position=fromJolt(point);
            e.normalImpulseNewtonSeconds=impulse; e.tangentialSpeedMetersPerSecond=tangential;
            e.effectiveMassKg=effective; e.massA=massA; e.massB=massB; e.staticA=!a.IsDynamic(); e.staticB=!b.IsDynamic();
            contactSlides.push_back(std::move(e));
        }
    }
}
void PhysicsScene3D::Impl::consolidateContacts() {
    const auto less = [](const auto& a, const auto& b) {
        return std::tie(a.bodyA,a.bodyB,a.position.x,a.position.y,a.position.z) < std::tie(b.bodyA,b.bodyB,b.position.x,b.position.y,b.position.z);
    };
    std::sort(contactImpacts.begin(),contactImpacts.end(),less);
    std::sort(contactSlides.begin(),contactSlides.end(),less);
    for (auto& slot : ragdollSlots) if (slot.record) {
        auto& points = slot.record->stepContacts;
        std::sort(points.begin(),points.end(),[](const auto& a,const auto& b) {
            return std::tie(a.linkIndex,a.position.x,a.position.y,a.position.z) < std::tie(b.linkIndex,b.position.x,b.position.y,b.position.z);
        });
    }
}
} // namespace MatterEngine
