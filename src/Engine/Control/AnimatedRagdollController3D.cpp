#include "Engine/Control/AnimatedRagdollController3D.hpp"
#include "Engine/Control/RagdollPoseMotor3D.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {
using Phase=AnimatedRagdollPhase3D;
constexpr float Pi=3.14159265358979323846f;
float component(Vec3 v,int a) { return a==0?v.x:a==1?v.y:v.z; }
void component(Vec3& v,int a,float x) { if(a==0)v.x=x;else if(a==1)v.y=x;else v.z=x; }
float smooth(float t) { t=std::clamp(t,0.0f,1.0f);return t*t*(3-2*t); }
Quaternion blendRotation(Quaternion a,Quaternion b,float t) {
    if(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w<0)b={-b.x,-b.y,-b.z,-b.w};
    return Quaternion{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t,a.w+(b.w-a.w)*t}.normalized();
}
bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
bool moving(Phase p) { return p==Phase::Walking||p==Phase::Running; }
Quaternion heading(Quaternion q) {
    Vec3 f=q.rotate({1,0,0});
    if(f.x*f.x+f.y*f.y<1e-5f) { auto l=q.rotate({0,1,0});f={l.y,-l.x,0}; }
    return Quaternion::fromAxisAngle({0,0,1},std::atan2(f.y,f.x));
}
bool validClip(const AnimationClip3D* c,const RagdollProfile3D& p) {
    return c&&c->targetRigId==p.id&&std::isfinite(c->durationSeconds)&&c->durationSeconds>0;
}
}
bool AnimatedRagdollClips3D::compatible(const RagdollProfile3D& p) const {
    if(!validClip(idle,p)) return false;
    for(const auto* c:{idle,run,stop,walkForward,walkBackward,walkLeft,walkRight,runBackward})
        if(c&&(!validClip(c,p)||!validateAnimationClipForRagdoll3D(*c,p).empty())) return false;
    return true;
}
void AnimatedRagdollController3D::reset(const RagdollProfile3D& p,const RagdollState3D& state) {
    m_initialized=!p.links.empty()&&state.links.size()==p.links.size()&&state.joints.size()==p.links.size();
    m_output={};m_telemetry={};m_phaseTime=m_blendTime=m_gaitClock=m_speed=0;
    m_gaitSource=nullptr;m_requestedSeconds=0;m_wasManipulated=m_stoppingWalk=false;
    m_previousRootTilt={};m_blendRootRotationFrom={};
    m_previousRootTranslation=m_blendRootTranslationFrom={};m_directionBody={1,0,0};
    m_hybridAssist.reset();m_footwork.reset(p);
    if(!m_initialized)return;
    m_heading=heading(state.links.front().orientation);m_commandOrigin=state.links.front().position;
    m_coordinates.assign(p.links.size(),{});m_coordinateVelocities.assign(p.links.size(),{});
    for(std::size_t i=1;i<p.links.size();++i)
        m_coordinates[i]={state.joints[i].positionRadians[0],state.joints[i].positionRadians[1],state.joints[i].positionRadians[2]};
    m_blendFrom=m_coordinates;
}
Vec3 AnimatedRagdollController3D::desiredPlanarVelocityWorld(
    const RagdollState3D& state) const {
    if(state.links.empty())return {};
    const auto frame=heading(state.links.front().orientation*
        m_previousRootTilt.conjugate());
    return frame.rotate(m_directionBody*m_speed);
}
void AnimatedRagdollController3D::transition(Phase phase) {
    m_telemetry.phase=phase;m_phaseTime=m_blendTime=0;m_blendFrom=m_coordinates;
    m_blendRootRotationFrom=m_previousRootTilt;m_blendRootTranslationFrom=m_previousRootTranslation;
}
bool AnimatedRagdollController3D::requestRun(const RagdollState3D& s,float seconds) {
    if(!m_initialized||s.links.empty()||m_telemetry.phase!=Phase::Idle||m_telemetry.manipulated||
        !std::isfinite(seconds)||seconds<=0)return false;
    m_requestedSeconds=std::clamp(seconds,0.1f,30.0f);m_directionBody={1,0,0};
    m_heading=heading(s.links.front().orientation);m_commandOrigin=s.links.front().position;
    m_gaitClock=0;m_gaitSource=nullptr;m_stoppingWalk=false;m_telemetry.blocked=false;
    transition(Phase::Running);return true;
}
bool AnimatedRagdollController3D::requestWalk(const RagdollState3D& s,float seconds,Vec3 direction) {
    if(!finite(direction))return false;
    direction.z=0;
    if(direction.lengthSquared()<1e-6f||!requestRun(s,seconds))return false;
    m_directionBody=direction.normalized();transition(Phase::Walking);return true;
}
void AnimatedRagdollController3D::update(const RagdollProfile3D& p,const AnimatedRagdollClips3D& clips,
    const RagdollState3D& state,const CapsuleTraversalResult3D& traversal,
    float dt,const std::array<GroundProbeResult3D,2>& footGround,
    bool manipulated,std::uint32_t grabbedLink) {
    m_output={};
    if(!std::isfinite(dt)||dt<=0||state.links.size()!=p.links.size()||
        state.joints.size()!=p.links.size()||p.links.empty()||!validClip(clips.idle,p))return;
    if(!m_initialized)reset(p,state);
    dt=std::min(dt,0.05f);
    const auto& root=state.links.front();
    if(!finite(root.position)||!finite(root.linearVelocity))return;
    const auto frame=heading(root.orientation*m_previousRootTilt.conjugate());
    const float up=root.orientation.rotate({0,0,1}).z;
    m_telemetry.manipulated=manipulated;
    if(manipulated!=m_wasManipulated) {
        for(std::size_t i=1;i<p.links.size();++i)
            m_coordinates[i]={state.joints[i].positionRadians[0],state.joints[i].positionRadians[1],state.joints[i].positionRadians[2]};
        transition(Phase::Idle);m_speed=0;m_wasManipulated=manipulated;
    }
    auto effectiveTraversal=traversal;
    if(manipulated)effectiveTraversal.mode=RagdollTraversalMode3D::PhysicalOverride;
    const auto channels=m_hybridAssist.update(state,effectiveTraversal,
        m_settings.assistanceEnabled?m_settings.auxiliaryAuthority:0,dt,manipulated);
    // A default animated body cannot autonomously decide to fall or take a
    // recovery step because the solver deviated from its previous pose.
    if(channels.pose==0&&!manipulated&&up<0.35f) {
        if(m_telemetry.phase!=Phase::Fallen)transition(Phase::Fallen);
        m_speed=0;
    } else if(m_telemetry.phase==Phase::Fallen&&channels.pose>0)transition(Phase::Idle);
    m_phaseTime+=dt;m_blendTime+=dt;
    if(moving(m_telemetry.phase)&&m_phaseTime>=m_requestedSeconds) {
        m_stoppingWalk=m_telemetry.phase==Phase::Walking;transition(Phase::Stopping);
    }
    if(m_telemetry.phase==Phase::Stopping&&m_speed<0.001f)transition(Phase::Idle);
    const bool locomotion=moving(m_telemetry.phase)||(m_telemetry.phase==Phase::Stopping&&m_speed>0.001f);
    const bool running=m_telemetry.phase==Phase::Running||
        (m_telemetry.phase==Phase::Stopping&&!m_stoppingWalk);
    const AnimationClip3D* clip=clips.idle;
    if(locomotion) {
        if(m_telemetry.phase==Phase::Running||(m_telemetry.phase==Phase::Stopping&&!m_stoppingWalk))clip=clips.run;
        else if(std::abs(m_directionBody.y)>std::abs(m_directionBody.x))clip=m_directionBody.y>0?clips.walkLeft:clips.walkRight;
        else clip=m_directionBody.x<0?clips.walkBackward:clips.walkForward;
        if(!validClip(clip,p))clip=validClip(clips.walkForward,p)?clips.walkForward:clips.idle;
    }
    // Root Z bob is not travel. Use the selected directional clip's horizontal
    // displacement, so forward/back/strafe each retain their authored timing.
    const auto travel=clip->sourceRootDisplacementMeters;
    const float authoredSpeed=std::hypot(travel.x,travel.y)/clip->durationSeconds;
    const float sourceSpeed=std::isfinite(authoredSpeed)&&authoredSpeed>0.05f
        ? authoredSpeed : (running?5.2f:1.4f);
    const float overrideSpeed=running?m_settings.runSpeedMetersPerSecond:m_settings.walkSpeedMetersPerSecond;
    const bool overridden=std::isfinite(overrideSpeed)&&overrideSpeed>0;
    const float cruiseSpeed=overridden?std::clamp(overrideSpeed,sourceSpeed*0.35f,sourceSpeed*2.0f):sourceSpeed;
    const float desiredSpeed=moving(m_telemetry.phase)?cruiseSpeed:0;
    const float acceleration=desiredSpeed<m_speed?m_settings.stoppingDecelerationMetersPerSecondSquared:
        running?m_settings.runAccelerationMetersPerSecondSquared:m_settings.walkAccelerationMetersPerSecondSquared;
    m_speed+=std::clamp(desiredSpeed-m_speed,-acceleration*dt,acceleration*dt);
    // Acceleration changes root travel, not the clock of the authored motion.
    // In the default mode even startup and braking play at 1x, never slow-mo.
    const float cadence=locomotion&&overridden?std::clamp(cruiseSpeed/sourceSpeed,0.35f,2.0f):1.0f;
    m_telemetry.animationPlaybackRate=cadence;
    m_telemetry.commandedSpeedMetersPerSecond=m_speed;
    m_gaitClock+=dt*cadence;
    const float time=locomotion?m_gaitClock:m_phaseTime;
    const float blend=smooth(m_blendTime/0.20f);
    std::vector<Vec3> wanted(p.links.size());
    std::array<std::size_t,2> feet{p.links.size(),p.links.size()};
    for(std::size_t i=1;i<p.links.size();++i) {
        const auto* track=findAnimationTrack3D(*clip,p.links[i].id);
        const auto sample=track?sampleAnimationTrack3D(*track,time,clip->durationSeconds,clip->loops):AnimationTransformSample3D{};
        wanted[i]=m_blendFrom[i]*(1-blend)+sample.jointPositionRadians*blend;
        if(p.links[i].id=="LeftFoot")feet[0]=i;
        if(p.links[i].id=="RightFoot")feet[1]=i;
    }
    const auto* rootTrack=findAnimationTrack3D(*clip,p.links.front().id);
    const auto rootSample=rootTrack?sampleAnimationTrack3D(*rootTrack,time,clip->durationSeconds,clip->loops):AnimationTransformSample3D{};
    // Preserve authored pelvis rotation without accumulating its yaw every tick.
    // The base frame above removes the previous local animation rotation.
    const Quaternion tilt=blendRotation(m_blendRootRotationFrom,rootSample.rotationDelta,blend);
    const Vec3 rootTranslation=m_blendRootTranslationFrom*(1-blend)+rootSample.translationOffsetMeters*blend;
    ContactFootworkIntent3D footIntent;
    footIntent.desiredVelocityHeading=m_directionBody*m_speed;
    footIntent.allowSteps=m_settings.footworkEnabled&&m_settings.steppingEnabled&&
        effectiveTraversal.mode==RagdollTraversalMode3D::Grounded&&!manipulated;
    footIntent.locomotionEnabled=locomotion;
    footIntent.balanceRecoveryEnabled=false;
    footIntent.ground=effectiveTraversal.ground;
    footIntent.footGround=footGround;
    m_footwork.update(p,state,dt,footIntent);
    const auto& plannedFeet=m_footwork.output();
    m_telemetry.swingFoot=plannedFeet.swingFoot;
    m_telemetry.footworkPhase=plannedFeet.phase;
    m_telemetry.footstepReason=plannedFeet.stepReason;
    m_telemetry.footworkStepUrgency=plannedFeet.stepUrgency;
    m_telemetry.plannedSwingDurationSeconds=plannedFeet.plannedSwingDurationSeconds;
    m_telemetry.plannedStepDirectionHeading=plannedFeet.plannedStepDirectionHeading;
    m_telemetry.centerOfMassHeading=plannedFeet.centerOfMassHeading;
    m_telemetry.supportCenterHeading=plannedFeet.supportCenterHeading;
    m_telemetry.captureErrorMeters=plannedFeet.supportErrorHeading.length();
    m_telemetry.waitingForClearance=plannedFeet.waitingForClearance;
    m_telemetry.swingClearanceMeters=plannedFeet.swingClearanceMeters;
    m_telemetry.desiredSwingClearanceMeters=plannedFeet.desiredSwingClearanceMeters;
    if(locomotion&&m_settings.footworkEnabled&&plannedFeet.valid&&
       effectiveTraversal.mode==RagdollTraversalMode3D::Grounded) {
        for(std::size_t side=0;side<plannedFeet.feet.size();++side) {
            const auto& foot=plannedFeet.feet[side];
            if(foot.linkIndex>=p.links.size())continue;
            const bool swing=static_cast<int>(side)==plannedFeet.swingFoot&&
                (plannedFeet.phase==ContactFootworkPhase3D::Swing||
                 plannedFeet.phase==ContactFootworkPhase3D::Touchdown);
            // Preserve the authored gait as the dominant reference. The
            // planner corrects contact geometry without rebuilding the legs
            // from a noisy measured pose.
            const float proceduralWeight=swing?0.58f:0.24f;
            fitRagdollFootTarget3D(p,wanted,foot.linkIndex,
                foot.targetPositionHeading,
                p.links[foot.linkIndex].modelOrientation,
                blend*proceduralWeight);
        }
    }
    const float muscle=std::isfinite(m_settings.muscleAuthority)?std::clamp(m_settings.muscleAuthority,0.0f,1.0f):0;
    float errorSquared=0;unsigned dofs=0;
    for(std::size_t i=1;i<p.links.size();++i) {
        const Vec3 previous=m_coordinates[i];
        for(int a=0;a<3;++a) {
            const auto& axis=p.links[i].inboundJoint.axes[a];
            const float q=axis.enabled?std::clamp(component(wanted[i],a),axis.minimumRadians,axis.maximumRadians):0;
            component(m_coordinates[i],a,q);
            component(m_coordinateVelocities[i],a,(q-component(previous,a))/dt);
        }
        const auto& measured=state.joints[i].positionRadians;
        const Vec3 velocity=ragdollJointTargetVelocity3D(m_coordinates[i],m_coordinateVelocities[i],{measured[0],measured[1],measured[2]});
        for(int a=0;a<3;++a) {
            if(!p.links[i].inboundJoint.axes[a].enabled)continue;
            RagdollDriveTarget3D drive;
            drive.linkIndex=static_cast<std::uint32_t>(i);drive.axis=static_cast<RagdollAxis3D>(a);
            drive.positionRadians=component(m_coordinates[i],a);
            drive.velocityRadiansPerSecond=std::clamp(component(velocity,a),-14.0f,14.0f);
            drive.stiffnessScale=2*muscle;drive.dampingScale=2*std::sqrt(muscle);drive.maximumTorqueScale=3*muscle;
            if(manipulated&&i==grabbedLink) {drive.stiffnessScale*=0.25f;drive.maximumTorqueScale*=0.25f;}
            m_output.driveTargets.push_back(drive);
            const float error=component(previous,a)-measured[a];errorSquared+=error*error;++dofs;
        }
    }
    buildRagdollJointPose3D(p,m_coordinates,m_output.targetPose);
    auto& constraint=m_output.animationConstraint;
    constraint.channels=channels;
    constraint.rootOrientationWorld=(frame*tilt).normalized();
    constraint.allowedPlanarDisplacementWorld=effectiveTraversal.allowedDisplacementWorld;
    constraint.allowedPlanarDisplacementWorld.z=0;
    constraint.desiredPlanarVelocityWorld=dt>0
        ? constraint.allowedPlanarDisplacementWorld/dt : Vec3{};
    constraint.desiredPlanarVelocityWorld.z=0;
    m_previousRootTilt=tilt;m_previousRootTranslation=rootTranslation;

    // Vertical support is a finite, distributed force relative to the current
    // walkable support. It vanishes in the first airborne/steep/override tick.
    const float mass=std::max(0.1f,p.totalMassKg);
    const float weight=mass*9.81f;
    Vec3 supportForce;
    if(channels.verticalSupport>0&&effectiveTraversal.ground.hasSurface&&
       effectiveTraversal.ground.walkable&&
       effectiveTraversal.mode==RagdollTraversalMode3D::Grounded) {
        const float measured=root.position.z-effectiveTraversal.ground.pointWorld.z;
        const float desired=p.standingRootHeightMeters+
            std::clamp(rootTranslation.z,-0.06f,0.06f);
        const float acceleration=std::clamp(9.81f+100.0f*(desired-measured)-
            20.0f*root.linearVelocity.z,-9.81f,28.0f);
        supportForce={0,0,mass*acceleration*channels.verticalSupport};
    }
    const Vec3 targetUp=constraint.rootOrientationWorld.rotate({0,0,1});
    const Vec3 currentUp=root.orientation.rotate({0,0,1});
    const Vec3 tiltOmega=root.angularVelocity-Vec3{0,0,1}*root.angularVelocity.z;
    Vec3 uprightTorque=(cross(currentUp,targetUp)*(weight*p.standingRootHeightMeters*6.0f)-
        tiltOmega*(weight*p.standingRootHeightMeters*1.25f))*channels.upright;
    const float torqueLimit=weight*p.standingRootHeightMeters*3.0f*
        channels.upright;
    if(uprightTorque.length()>torqueLimit&&torqueLimit>0)
        uprightTorque=uprightTorque.normalized()*torqueLimit;
    std::array<float,4> coreWeights{0.48f,0.24f,0.18f,0.10f};
    std::size_t coreSlot=0;
    for(std::size_t i=0;i<p.links.size()&&coreSlot<coreWeights.size();++i) {
        const auto& id=p.links[i].id;
        if(id!="Pelvis"&&id!="Abdomen"&&id!="Chest"&&id!="UpperChest")continue;
        AnimatedRagdollForce3D wrench;
        wrench.linkIndex=static_cast<std::uint32_t>(i);
        wrench.forceNewtons=supportForce*coreWeights[coreSlot];
        wrench.torqueNewtonMeters=uprightTorque*coreWeights[coreSlot];
        m_output.assistance.push_back(wrench);
        ++coreSlot;
    }
    m_output.gravityCompensationEnabled=muscle>0;
    for(std::size_t i=0;i<p.links.size();++i) {
        m_output.targetPose.linkPositions[i]=root.position+
            constraint.rootOrientationWorld.rotate(m_output.targetPose.linkPositions[i]);
        m_output.targetPose.linkOrientations[i]=(constraint.rootOrientationWorld*m_output.targetPose.linkOrientations[i]).normalized();
    }
    m_telemetry.assistanceAuthority=channels.pose;
    m_telemetry.standingPostureAuthority=channels.upright;
    m_telemetry.assistanceForceNewtons=supportForce.length();
    m_telemetry.assistanceTorqueNewtonMeters=uprightTorque.length();
    m_telemetry.assistanceNetForceWorld=supportForce;
    m_telemetry.assistanceBudgetWeightFraction=supportForce.length()/weight;
    m_telemetry.traversalMode=effectiveTraversal.mode;
    m_telemetry.ground=effectiveTraversal.ground;
    m_telemetry.assistChannels=channels;
    m_telemetry.proxyRequestedDisplacement=effectiveTraversal.requestedDisplacementWorld;
    m_telemetry.proxyAllowedDisplacement=effectiveTraversal.allowedDisplacementWorld;
    m_telemetry.physicalRootVelocity=root.linearVelocity;
    m_telemetry.proxyDesiredVelocity=constraint.desiredPlanarVelocityWorld;
    m_telemetry.proxyBlocked=effectiveTraversal.blocked;
    m_telemetry.groundAdhesionActive=effectiveTraversal.groundAdhesionActive;
    m_telemetry.blocked=effectiveTraversal.blocked;
    m_telemetry.muscleTorqueScale=3*muscle;m_telemetry.phaseSeconds=m_phaseTime;m_telemetry.bodyUpDot=up;
    m_telemetry.movementSecondsRemaining=moving(m_telemetry.phase)?std::max(0.0f,m_requestedSeconds-m_phaseTime):0;
    m_telemetry.runSecondsRemaining=m_telemetry.phase==Phase::Running?m_telemetry.movementSecondsRemaining:0;
    m_telemetry.speedMetersPerSecond=dot(root.linearVelocity,frame.rotate(m_directionBody));
    m_telemetry.travelledMeters=dot(root.position-m_commandOrigin,m_heading.rotate(m_directionBody));
    m_telemetry.jointRmsDegrees=std::sqrt(errorSquared/std::max(1u,dofs))*180/Pi;
    m_telemetry.supportLoadNewtons=0;m_telemetry.supportFootCount=0;
    for(const auto i:feet)if(i<p.links.size()) {
        bool touching=false;
        for(const auto& c:state.contacts)if(c.linkIndex==i&&c.normal.z>0.62f) {
            touching=true;m_telemetry.supportLoadNewtons+=c.normalImpulseNewtonSeconds/dt;
        }
        if(touching)++m_telemetry.supportFootCount;
    }
}
} // namespace MatterEngine
