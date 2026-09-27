#include "Engine/Physics/Jolt/JoltInternals3D.hpp"
#include "Engine/Environment/OceanSurface.hpp"
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <algorithm>
#include <cmath>

namespace MatterEngine {
void PhysicsScene3D::Impl::OnStep(const JPH::PhysicsStepListenerContext& context) {
    try {
        if (context.mIsFirstStep) {
            applyPendingWrenches();
            applyAerodynamicDrag(stepDeltaTime);
        }
        applyBuoyancy(context.mDeltaTime);
    } catch (...) { callbackFailed = true; }
}
void PhysicsScene3D::Impl::applyPendingWrenches() {
    auto& bi = system->GetBodyInterfaceNoLock();
    for (const auto& w : pendingBodyWrenches) {
        if (bi.GetMotionType(w.body) != JPH::EMotionType::Dynamic) continue;
        bi.AddForce(w.body, toJolt(w.force), JPH::EActivation::DontActivate);
        bi.AddTorque(w.body, toJolt(w.torque), JPH::EActivation::DontActivate);
    }
    pendingBodyWrenches.clear();
}
void PhysicsScene3D::Impl::applyAerodynamicDrag(float dt) {
    auto& bi = system->GetBodyInterfaceNoLock();
    const auto count = system->GetNumActiveBodies(JPH::EBodyType::RigidBody);
    const auto* ids = system->GetActiveBodiesUnsafe(JPH::EBodyType::RigidBody);
    for (JPH::uint i=0; i<count; ++i) {
        const auto id=ids[i];
        const auto identity=JoltDetail::unpackBodyIdentity(bi.GetUserData(id));
        if (identity.kind != JoltDetail::BodyKind::Body) continue;
        const auto& r=*bodySlots[identity.slotIndex].record;
        const auto& d=r.definition;
        if (!d.aerodynamicDragEnabled || r.frozen || r.motionType != JPH::EMotionType::Dynamic || d.aerodynamicReferenceAreaSquareMeters <= 0) continue;
        Vec3 wind=settings.airVelocity;
        const float windSpeed=wind.length();
        if (windSpeed > 1e-4f && settings.windShelterDistanceMeters > 0) {
            JPH::RayCastResult hit;
            const float range=settings.windShelterDistanceMeters;
            if (system->GetNarrowPhaseQueryNoLock().CastRay({bi.GetPosition(id),toJolt(wind * (-range/windSpeed))},hit,{},JPH::SpecifiedObjectLayerFilter(JoltObjectLayers::NonMoving))) {
                const float t=std::clamp(hit.mFraction,0.0f,1.0f);
                wind *= t*t*(3-2*t);
            }
        }
        const Vec3 relative=wind-fromJolt(bi.GetLinearVelocity(id));
        const float speed=relative.length();
        if (speed < 1e-5f) continue;
        const float magnitude=std::min(0.5f*settings.airDensityKgPerCubicMeter*d.aerodynamicDragCoefficient*d.aerodynamicReferenceAreaSquareMeters*speed*speed,d.massKg*speed/dt);
        bi.AddForce(id,toJolt(relative*(magnitude/speed)),JPH::EActivation::DontActivate);
    }
}
void PhysicsScene3D::Impl::applyBuoyancy(float dt) {
    if (!ocean) return;
    const auto& water=*ocean;
    const auto count=system->GetNumActiveBodies(JPH::EBodyType::RigidBody);
    const auto* ids=system->GetActiveBodiesUnsafe(JPH::EBodyType::RigidBody);
    auto& bi=system->GetBodyInterfaceNoLock();
    for (JPH::uint i=0;i<count;++i) {
        const auto id=ids[i];
        const auto identity=JoltDetail::unpackBodyIdentity(bi.GetUserData(id));
        if (identity.kind != JoltDetail::BodyKind::Body) continue;
        const auto& r=*bodySlots[identity.slotIndex].record;
        if (r.frozen || r.motionType != JPH::EMotionType::Dynamic || r.definition.bodyVolumeCubicMeters <= 0) continue;
        const auto position=fromJolt(bi.GetPosition(id));
        if (std::abs(position.x-water.center.x)>water.halfExtents.x || std::abs(position.y-water.center.y)>water.halfExtents.y || position.z<water.meanSeaLevelMeters-water.depthMeters) continue;
        const auto surface=evaluateOceanSurface({position.x,position.y},water.meanSeaLevelMeters,oceanTimeSeconds);
        const float buoyancy=water.densityKgPerCubicMeter*r.definition.bodyVolumeCubicMeters/r.definition.massKg;
        bi.ApplyBuoyancyImpulse(id,toJolt({position.x,position.y,surface.heightMeters}),toJolt(surface.normal),buoyancy,0.6f,0.3f,toJolt({0,0,surface.verticalSpeedMetersPerSecond}),toJolt(settings.gravity),dt);
    }
}
} // namespace MatterEngine
