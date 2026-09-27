#include "Engine/Control/HybridRagdollAssist3D.hpp"
#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
Vec3 limited(Vec3 v, float limit) {
    const float n = v.length();
    return n > limit && n > 0 ? v * (limit / n) : v;
}
bool foot(const RagdollLinkDefinition3D& l) { return l.id == "LeftFoot" || l.id == "RightFoot"; }
float priority(const RagdollLinkDefinition3D& l) {
    return l.id == "Pelvis" || l.id == "Abdomen" || l.id == "Chest" || l.id == "UpperChest" ? 2.5f : 1.0f;
}
Vec3 rotationError(Quaternion wanted, Quaternion current) {
    auto q = (wanted * current.conjugate()).normalized();
    if(q.w < 0) { q.x=-q.x; q.y=-q.y; q.z=-q.z; q.w=-q.w; }
    const Vec3 v{q.x,q.y,q.z};
    const float n=v.length();
    return n>1e-6f ? v*(2*std::atan2(n,std::max(0.0f,q.w))/n) : v*2;
}
}
void HybridRagdollAssist3D::reset() {
    m_authority=m_contactSeverity=m_contactHold=0;
    m_localSeverity.clear();
}
HybridRagdollAssistOutput3D HybridRagdollAssist3D::update(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    const RagdollAnimationPose3D& pose, const std::vector<Vec3>& velocities,
    const ContactFootworkOutput3D& support, Vec3 desiredVelocity,
    float requested, float dt, bool inhibited) {
    HybridRagdollAssistOutput3D result;
    const auto n=profile.links.size();
    if(!n || state.links.size()!=n || pose.linkPositions.size()!=n ||
       pose.linkOrientations.size()!=n || !std::isfinite(dt) || dt<=0) {
        reset(); return result;
    }
    dt=std::min(dt,0.05f);
    requested=std::isfinite(requested)?std::clamp(requested,0.0f,1.0f):0;
    m_localSeverity.resize(n,0);
    std::vector<float> impulses(n,0);
    for(const auto& c:state.contacts) {
        if(c.linkIndex>=n || (foot(profile.links[c.linkIndex]) && c.normal.z>0.62f)) continue;
        impulses[c.linkIndex]+=std::max(0.0f,c.normalImpulseNewtonSeconds);
    }
    const float mass=std::max(0.1f,profile.totalMassKg), weight=mass*9.81f;
    float severity=0;
    for(std::size_t i=0;i<n;++i) {
        // Sustained pressure and impact both yield. Ordinary sole support is excluded.
        const float hit=std::clamp(impulses[i]/(weight*dt*0.6f),0.0f,1.0f);
        severity=std::max(severity,hit);
        m_localSeverity[i]=std::max(hit,m_localSeverity[i]*std::exp(-dt/0.45f));
    }
    if(severity>0.05f) { m_contactHold=0.18f; m_contactSeverity=std::max(m_contactSeverity,severity); }
    else if(m_contactHold>0) m_contactHold=std::max(0.0f,m_contactHold-dt);
    else m_contactSeverity*=std::exp(-dt/0.55f);
    // No grace interval can apply forces during flight or a physgun grab.
    const bool eligible=!inhibited && support.valid && support.supportCount>0 &&
        !support.likelyFallen && support.uprightDot>0.3f;
    if(!eligible || requested==0) { m_authority=0; return result; }
    const float target=requested*(1-0.65f*m_contactSeverity);
    m_authority+=(target-m_authority)*(1-std::exp(-dt/(target<m_authority?0.035f:0.30f)));
    m_authority=std::min(m_authority,requested);
    const auto& root=state.links.front();
    const Quaternion frame=support.heading;
    float fractions=0, priorities=0;
    Vec3 comVelocity;
    for(std::size_t i=0;i<n;++i) {
        fractions+=profile.links[i].massFraction;
        priorities+=profile.links[i].massFraction*priority(profile.links[i]);
        comVelocity+=state.links[i].linearVelocity*profile.links[i].massFraction;
    }
    fractions=std::max(0.001f,fractions); priorities=std::max(0.001f,priorities);
    comVelocity=comVelocity/fractions;
    Vec3 acceleration=(frame.rotate(desiredVelocity)-comVelocity)*7.0f;
    acceleration.z=0;
    // Shift onto the measured stance contact, not a saved world location.
    if(support.phase==ContactFootworkPhase3D::WeightShift)
        acceleration+=frame.rotate(support.desiredComCorrectionHeading)*12.0f;
    acceleration=limited(acceleration,7.0f);
    // Height is measured from CURRENT support geometry. Leave a load on soles.
    acceleration.z=std::clamp(0.70f*9.81f+
        100.0f*(profile.standingRootHeightMeters-0.018f-support.rootHeightAboveSupport)
        -16.0f*comVelocity.z,-9.81f,19.62f);
    result.movementForceWorld=Vec3{acceleration.x,acceleration.y,0}*(mass*m_authority);
    result.liftForceWorld=Vec3{0,0,acceleration.z}*(mass*m_authority);
    std::vector<Vec3> shape(n);
    Vec3 shapeSum;
    // Targets are rebuilt in the current pelvis frame on every tick.
    for(std::size_t i=0;i<n;++i) {
        const auto& def=profile.links[i]; const auto& body=state.links[i];
        const Vec3 desiredRelative=pose.linkPositions[i]+pose.linkOrientations[i].rotate(def.centerOfMassLocal);
        const Vec3 measuredRelative=body.position-root.position+body.orientation.rotate(def.centerOfMassLocal);
        const Vec3 targetVelocity=i<velocities.size()?frame.rotate(velocities[i]):Vec3{};
        shape[i]=limited((frame.rotate(desiredRelative)-measuredRelative)*180.0f+
            (targetVelocity-(body.linearVelocity-root.linearVelocity))*24.0f,45.0f)*
            (mass*def.massFraction/fractions)/(1+24.0f*dt+180.0f*dt*dt);
        shapeSum+=shape[i];
    }
    float forceSum=0,torqueSum=0;
    const float height=std::max(0.3f,profile.standingRootHeightMeters);
    const Vec3 up=root.orientation.rotate(profile.links.front().modelOrientation.conjugate().rotate({0,0,1}));
    const Vec3 tiltVelocity{root.angularVelocity.x,root.angularVelocity.y,0};
    // Core righting uses whole-body inertia: it supports the connected body,
    // whereas local pose damping below uses each collider's own inertia.
    const float bodyInertia=mass*height*height*0.25f;
    const Vec3 balanceTorque=limited((cross(up,Vec3{0,0,1})*100.0f-tiltVelocity*20.0f)*
        (bodyInertia/(1+20.0f*dt+100.0f*dt*dt)),weight*height);

    for(std::size_t i=0;i<n;++i) {
        const auto& def=profile.links[i]; const auto& body=state.links[i];
        const float share=def.massFraction/fractions;
        const float coreShare=def.massFraction*priority(def)/priorities;
        // Yield the entire immediate joint chain when a link is constrained.
        float local=m_localSeverity[i];
        if(def.parentIndex>=0 && static_cast<std::size_t>(def.parentIndex)<n)
            local=std::max(local,m_localSeverity[def.parentIndex]*0.8f);
        for(std::size_t j=1;j<n;++j)
            if(profile.links[j].parentIndex==static_cast<int>(i)) local=std::max(local,m_localSeverity[j]*0.6f);
        const float a=m_authority*(1-0.85f*local);
        RagdollAssistWrench3D w;
        w.linkIndex=static_cast<std::uint32_t>(i);
        // Shape forces have zero net force before contact attenuation.
        w.forceNewtons=((shape[i]-shapeSum*share)+acceleration*(mass*coreShare))*a;
        Vec3 error=rotationError(frame*pose.linkOrientations[i],body.orientation);
        Vec3 omega=body.angularVelocity;
        // Follow current heading: damp yaw rate, never restore an old yaw.
        const float kp=weight*height*coreShare*2.8f;
        const float kd=weight*height*coreShare*0.38f;
        const auto& c=def.collider;
        float inertia=mass*share*0.4f*c.radiusMeters*c.radiusMeters;
        if(c.shape==RagdollColliderShape3D::Box) {
            const auto h=c.boxHalfExtents;
            inertia=mass*share*std::min({h.x*h.x+h.y*h.y,h.x*h.x+h.z*h.z,h.y*h.y+h.z*h.z})/3.0f;
        }
        // Implicit PD denominator keeps tiny hands/feet from receiving an
        // explicit damping impulse larger than their angular momentum per tick.
        const float denominator=1+(kd*dt+kp*dt*dt)/std::max(0.0001f,inertia);
        w.torqueNewtonMeters=limited((error*kp-omega*kd)/denominator,
            weight*height*coreShare*2.0f)*a;
        w.torqueNewtonMeters+=balanceTorque*(coreShare*a);
        forceSum+=w.forceNewtons.length();torqueSum+=w.torqueNewtonMeters.length();
        result.wrenches.push_back(w);
    }
    // Whole-body budgets do not grow with skeleton link count.
    const float fs=std::min(1.0f,weight*4.0f*m_authority/std::max(0.001f,forceSum));
    const float ts=std::min(1.0f,weight*height*1.5f*m_authority/std::max(0.001f,torqueSum));
    for(auto& w:result.wrenches) {
        w.forceNewtons=w.forceNewtons*fs;w.torqueNewtonMeters=w.torqueNewtonMeters*ts;
        result.netForceWorld+=w.forceNewtons;result.netTorqueWorld+=w.torqueNewtonMeters;
        result.forceSumNewtons+=w.forceNewtons.length();result.torqueSumNewtonMeters+=w.torqueNewtonMeters.length();
    }
    result.supportAuthority=m_authority;
    return result;
}
} // namespace MatterEngine
