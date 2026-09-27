#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Physics/Jolt/JoltShapes3D.hpp"
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace MatterEngine {
namespace {
class Layers final : public JPH::ObjectLayerFilter {
public:
    explicit Layers(unsigned mask) : mask(mask) {}
    bool ShouldCollide(JPH::ObjectLayer layer) const override { return (mask & (1u << layer)) != 0; }
    unsigned mask;
};
struct Hit { JPH::BodyID id; JPH::SubShapeID sub; Vec3 point; Vec3 normal; float distance; };
bool cast(const PhysicsScene3D::Impl& impl, const Ray3D& ray, float distance,
    float radius, unsigned mask, Hit& out) {
    if (distance <= 0 || ray.direction.lengthSquared() < 1e-12f) return false;
    const auto direction = toJolt(ray.direction.normalized() * distance);
    if (radius > 0) {
        JPH::SphereShape shape(radius);
        JPH::RShapeCast query(&shape, JPH::Vec3::sReplicate(1),
            JPH::RMat44::sTranslation(toJolt(ray.origin)), direction);
        JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
        impl.system->GetNarrowPhaseQuery().CastShape(query, {}, JPH::RVec3::sZero(), collector, {}, Layers(mask));
        if (!collector.HadHit()) return false;
        const auto& hit = collector.mHit;
        out = {hit.mBodyID2, hit.mSubShapeID2, fromJolt(hit.mContactPointOn2),
            fromJolt(-hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisZ())), hit.mFraction * distance};
    } else {
        JPH::RayCastResult hit;
        if (!impl.system->GetNarrowPhaseQuery().CastRay({toJolt(ray.origin), direction}, hit, {}, Layers(mask))) return false;
        const auto point = toJolt(ray.origin) + hit.mFraction * direction;
        JPH::BodyLockRead lock(impl.system->GetBodyLockInterface(), hit.mBodyID);
        if (!lock.Succeeded()) return false;
        out = {hit.mBodyID, hit.mSubShapeID2, fromJolt(point),
            fromJolt(lock.GetBody().GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, point)), hit.mFraction * distance};
    }
    return true;
}
bool ordinary(const PhysicsScene3D::Impl& impl, const Ray3D& ray, float distance,
    float radius, unsigned mask, PhysicsRayHit3D& output) {
    Hit hit;
    if (!cast(impl, ray, distance, radius, mask, hit)) return false;
    JPH::BodyLockRead lock(impl.system->GetBodyLockInterface(), hit.id);
    if (!lock.Succeeded()) return false;
    const auto& body = lock.GetBody();
    const auto identity = JoltDetail::unpackBodyIdentity(body.GetUserData());
    output = {};
    if (identity.kind == JoltDetail::BodyKind::Body && identity.slotIndex < impl.bodySlots.size())
        output.body = impl.bodySlots[identity.slotIndex].record->handle;
    output.position = hit.point;
    output.normal = hit.normal;
    output.distance = hit.distance;
    output.materialId = joltShapeMaterialId(body.GetShape()->GetMaterial(hit.sub));
    return true;
}
bool ragdollQuery(const PhysicsScene3D::Impl& impl, const Ray3D& ray, float distance,
    float radius, PhysicsRagdollRayHit3D& output) {
    Hit hit;
    if (!cast(impl, ray, distance, radius, 1u << JoltObjectLayers::Ragdoll, hit)) return false;
    const auto identity = JoltDetail::unpackBodyIdentity(impl.system->GetBodyInterface().GetUserData(hit.id));
    if (identity.kind != JoltDetail::BodyKind::RagdollLink || identity.slotIndex >= impl.ragdollSlots.size()) return false;
    output = {impl.ragdollSlots[identity.slotIndex].record->handle, identity.linkIndex, hit.point, hit.normal, hit.distance};
    return true;
}
bool sweepCapsule(const PhysicsScene3D::Impl& impl, Vec3 center, float radius,
    float halfHeight, Vec3 displacement, Hit& out) {
    if (displacement.lengthSquared() < 1e-12f) return false;
    JPH::CapsuleShape shape(std::max(halfHeight, 0.001f), radius);
    JPH::RShapeCast query(&shape, JPH::Vec3::sReplicate(1),
        JPH::RMat44::sRotationTranslation(JPH::Quat::sRotation(JPH::Vec3::sAxisX(), JPH::JPH_PI * 0.5f), toJolt(center)), toJolt(displacement));
    JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
    impl.system->GetNarrowPhaseQuery().CastShape(query, {}, JPH::RVec3::sZero(), collector, {}, Layers(1));
    if (!collector.HadHit()) return false;
    const auto& h = collector.mHit;
    out = {h.mBodyID2, h.mSubShapeID2, fromJolt(h.mContactPointOn2),
        fromJolt(-h.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisZ())), h.mFraction * displacement.length()};
    return true;
}
} // namespace
bool PhysicsScene3D::raycast(const Ray3D& ray, float d, PhysicsRayHit3D& hit) const { return ordinary(*m_impl, ray, d, 0, 3, hit); }
bool PhysicsScene3D::raycastDynamic(const Ray3D& ray, float d, PhysicsRayHit3D& hit) const { return ordinary(*m_impl, ray, d, 0, 2, hit); }
bool PhysicsScene3D::raycastStatic(const Ray3D& ray, float d, PhysicsRayHit3D& hit) const { return ordinary(*m_impl, ray, d, 0, 1, hit); }
bool PhysicsScene3D::sweepSphere(const Ray3D& ray, float r, float d, PhysicsRayHit3D& hit) const { return ordinary(*m_impl, ray, d, r, 3, hit); }
bool PhysicsScene3D::sweepSphereDynamic(const Ray3D& ray, float r, float d, PhysicsRayHit3D& hit) const { return ordinary(*m_impl, ray, d, r, 2, hit); }
bool PhysicsScene3D::raycastRagdoll(const Ray3D& ray, float d, PhysicsRagdollRayHit3D& hit) const { return ragdollQuery(*m_impl, ray, d, 0, hit); }
bool PhysicsScene3D::sweepSphereRagdoll(const Ray3D& ray, float r, float d, PhysicsRagdollRayHit3D& hit) const { return ragdollQuery(*m_impl, ray, d, r, hit); }
bool PhysicsScene3D::overlapsBox(Vec3 center, Vec3 halfExtents, Quaternion orientation) const {
    JPH::BoxShape shape(toJolt(halfExtents), 0);
    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
    m_impl->system->GetNarrowPhaseQuery().CollideShape(&shape, JPH::Vec3::sReplicate(1),
        JPH::RMat44::sRotationTranslation(toJolt(orientation), toJolt(center)), {}, JPH::RVec3::sZero(), collector);
    return collector.HadHit();
}
GroundProbeResult3D PhysicsScene3D::probeGround(Vec3 center, float radius,
    float halfHeight, float distance, float slope) const {
    GroundProbeResult3D result;
    Hit hit;
    if (radius <= 0 || halfHeight < 0 || distance <= 0 || !sweepCapsule(*m_impl, center, radius, halfHeight, {0,0,-distance}, hit)) return result;
    result.hasSurface = true;
    result.pointWorld = hit.point;
    result.normalWorld = hit.normal;
    result.distanceMeters = hit.distance;
    result.slopeDegrees = std::acos(std::clamp(hit.normal.z, -1.0f, 1.0f)) * 180 / std::numbers::pi_v<float>;
    result.walkable = result.slopeDegrees <= slope;
    return result;
}
CapsuleTraversalResult3D PhysicsScene3D::solveCapsuleTraversal(
    const CapsuleTraversalQuery3D& source) const {
    CapsuleTraversalResult3D result;
    result.requestedDisplacementWorld = source.desiredDisplacementWorld;
    if (!std::isfinite(source.centerWorld.x)
        || !std::isfinite(source.centerWorld.y)
        || !std::isfinite(source.centerWorld.z)
        || !std::isfinite(source.desiredDisplacementWorld.x)
        || !std::isfinite(source.desiredDisplacementWorld.y)
        || !std::isfinite(source.desiredDisplacementWorld.z)) {
        return result;
    }

    const float radius = std::clamp(source.radiusMeters, 0.05f, 2.0f);
    const float halfHeight = std::clamp(
        source.cylinderHalfHeightMeters, 0.0f, 3.0f);
    const float skin = std::clamp(source.skinWidthMeters, 0.001f, radius * 0.4f);
    const float step = std::clamp(source.stepOffsetMeters, 0.0f, 0.8f);
    Vec3 position = source.centerWorld;
    Vec3 remaining = source.desiredDisplacementWorld;
    remaining.z = 0.0f;
    const std::uint32_t iterations = std::clamp(
        source.maximumIterations, 1u, 8u);

    for (std::uint32_t iteration = 0;
            iteration < iterations && remaining.lengthSquared() > 1.0e-10f;
            ++iteration) {
        Hit hit;
        if (!sweepCapsule(*m_impl, position, radius, halfHeight,
                remaining, hit)) {
            position += remaining;
            remaining = {};
            break;
        }

        const float requestedDistance = remaining.length();
        const Vec3 direction = remaining / requestedDistance;
        const float advance = std::max(0.0f, hit.distance - skin);
        position += direction * advance;
        Vec3 left = remaining - direction * advance;
        const Vec3 normal = hit.normal.normalized();

        // A low obstacle may be traversable. Test the same remainder with the
        // query capsule lifted by stepOffset; ground probing later brings it
        // back to the actual support surface.
        if (step > 0.0f && normal.z < 0.35f) {
            Hit raisedHit;
            if (!sweepCapsule(*m_impl,
                    position + Vec3 { 0.0f, 0.0f, step }, radius,
                    halfHeight, left, raisedHit)) {
                position.z += step;
                position += left;
                remaining = {};
                continue;
            }
        }

        result.blocked = true;
        const float intoSurface = dot(left, normal);
        if (intoSurface < 0.0f) left -= normal * intoSurface;
        left.z = std::max(0.0f, left.z);
        result.slideDisplacementWorld += left;
        remaining = left;
    }

    const float downDistance = step
        + std::max(source.groundProbeDistanceMeters,
            source.groundAdhesionDistanceMeters) + skin;
    const Vec3 raised = position + Vec3 { 0.0f, 0.0f, step };
    result.ground = probeGround(raised, radius, halfHeight,
        downDistance, source.maximumSlopeDegrees);
    if (result.ground.hasSurface && result.ground.walkable) {
        const float descend = result.ground.distanceMeters - step;
        const bool withinAdhesion = descend <=
            source.groundAdhesionDistanceMeters + skin;
        if (descend <= source.groundProbeDistanceMeters + skin
            || descend < 0.0f) {
            position.z -= descend;
            result.mode = RagdollTraversalMode3D::Grounded;
            result.groundAdhesionActive = descend > skin && withinAdhesion;
        } else {
            result.mode = RagdollTraversalMode3D::Airborne;
        }
    } else if (result.ground.hasSurface) {
        result.mode = RagdollTraversalMode3D::SlidingSteep;
    } else {
        result.mode = RagdollTraversalMode3D::Airborne;
    }
    result.allowedDisplacementWorld = position - source.centerWorld;
    return result;
}

} // namespace MatterEngine
