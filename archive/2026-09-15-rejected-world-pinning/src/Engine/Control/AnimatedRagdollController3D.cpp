#include "Engine/Control/AnimatedRagdollController3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
constexpr float Pi = 3.14159265358979323846f;
Vec3 limited(Vec3 value, float maximum) {
    const float length=value.length();
    return length>maximum ? value*(maximum/length) : value;
}
float smooth(float t) { t=std::clamp(t,0.0f,1.0f); return t*t*(3-2*t); }
Vec3 rotationVector(Quaternion q) {
    q=q.normalized();
    if (q.w<0) q={-q.x,-q.y,-q.z,-q.w};
    const Vec3 v{q.x,q.y,q.z};
    const float n=v.length();
    return n<1e-6f ? Vec3{} : v*(2*std::atan2(n,q.w)/n);
}
Quaternion rotation(Vec3 v) {
    const float n=v.length();
    return n<1e-6f ? Quaternion{} : Quaternion::fromAxisAngle(v/n,n);
}
Quaternion heading(Quaternion q) {
    const Vec3 forward=q.rotate({1,0,0});
    return Quaternion::fromAxisAngle({0,0,1},std::atan2(forward.y,forward.x));
}
float component(Vec3 v, std::size_t i) {return i==0 ? v.x : i==1 ? v.y : v.z;}
void setComponent(Vec3& v,std::size_t i,float f) {if(i==0)v.x=f;else if(i==1)v.y=f;else v.z=f;}
}

bool AnimatedRagdollClips3D::compatible(const RagdollProfile3D& profile) const {
    return idle&&run&&stop&&idle->loops&&run->loops&&!stop->loops
        &&validateAnimationClipForRagdoll3D(*idle,profile).empty()
        &&validateAnimationClipForRagdoll3D(*run,profile).empty()
        &&validateAnimationClipForRagdoll3D(*stop,profile).empty();
}

void AnimatedRagdollController3D::reset(const RagdollProfile3D& profile,
    const RagdollState3D& state,float groundHeight) {
    m_initialized=state.links.size()==profile.links.size()&&!state.links.empty();
    m_output={}; m_telemetry={}; m_phaseTime=0; m_blendTime=0;
    m_distance=0; m_speed=0; m_previousValid=false; m_wasManipulated=false;
    if(!m_initialized)return;
    m_routeOrigin=state.links.front().position;
    m_commandOrigin=m_routeOrigin;
    m_heading=heading(state.links.front().orientation);
    m_groundHeight=groundHeight;
    m_coordinates.assign(profile.links.size(),{});
    for(std::size_t i=1;i<state.joints.size();++i)
        m_coordinates[i]={state.joints[i].positionRadians[0],state.joints[i].positionRadians[1],state.joints[i].positionRadians[2]};
    m_blendFrom=m_coordinates;
}

void AnimatedRagdollController3D::transition(AnimatedRagdollPhase3D phase) {
    m_telemetry.phase=phase; m_phaseTime=0; m_blendTime=0;
    m_blendFrom=m_coordinates;
}

bool AnimatedRagdollController3D::requestRun(const RagdollState3D& state,float seconds) {
    if(!m_initialized||state.links.empty()||m_telemetry.phase!=AnimatedRagdollPhase3D::Idle
        ||m_telemetry.manipulated||!std::isfinite(seconds)||seconds<=0)return false;
    m_requestedSeconds=std::clamp(seconds,0.1f,30.0f);
    m_heading=heading(state.links.front().orientation);
    m_routeOrigin=state.links.front().position; m_commandOrigin=m_routeOrigin;
    m_distance=0; m_speed=0; m_telemetry.blocked=false;
    transition(AnimatedRagdollPhase3D::Running);
    return true;
}

void AnimatedRagdollController3D::update(const RagdollProfile3D& profile,
    const AnimatedRagdollClips3D& clips,const RagdollState3D& state,
    float dt,float groundHeight,bool manipulated,bool obstacleAhead) {
    m_output.driveTargets.clear(); m_output.assistance.clear();
    if(!std::isfinite(dt)||dt<=0||dt>0.05f||state.links.size()!=profile.links.size()
        ||!clips.idle||!clips.run||!clips.stop)return;
    if(!m_initialized)reset(profile,state,groundHeight);
    m_telemetry.manipulated=manipulated;
    if(manipulated) {
        // Yield to the Physgun, including joint motors. Re-anchor on release.
        m_wasManipulated=true; m_previousValid=false;
        for(std::size_t i=1;i<profile.links.size();++i)
            for(std::size_t a=0;a<3;++a)if(profile.links[i].inboundJoint.axes[a].enabled) {
                RagdollDriveTarget3D drive;
                drive.linkIndex=static_cast<std::uint32_t>(i); drive.axis=static_cast<RagdollAxis3D>(a);
                drive.stiffnessScale=drive.dampingScale=drive.maximumTorqueScale=0;
                m_output.driveTargets.push_back(drive);
            }
        return;
    }
    if(m_wasManipulated)reset(profile,state,groundHeight);
    m_groundHeight+=std::clamp(groundHeight-m_groundHeight,-dt*1.0f,dt*1.0f);
    m_phaseTime+=dt; m_blendTime+=dt;
    if(m_telemetry.phase==AnimatedRagdollPhase3D::Running
        &&(m_phaseTime>=m_requestedSeconds||obstacleAhead||m_telemetry.rootErrorMeters>0.8f)) {
        m_telemetry.blocked=obstacleAhead||m_telemetry.rootErrorMeters>0.8f;
        m_stopStartSpeed=m_speed;
        if(m_telemetry.blocked) {
            m_stopStartSpeed=0;
            m_routeOrigin=state.links.front().position; m_distance=0;
        }
        const auto* root=findAnimationTrack3D(*clips.stop,profile.links.front().id);
        const float interval=1.0f/clips.stop->sourceSampleRateHz;
        const float sourceSpeed=root?(sampleAnimationTrack3D(*root,interval,clips.stop->durationSeconds,false).translationOffsetMeters.x
            -sampleAnimationTrack3D(*root,0,clips.stop->durationSeconds,false).translationOffsetMeters.x)/interval:0;
        m_stopRate=sourceSpeed>0.1f?std::clamp(m_stopStartSpeed/sourceSpeed,0.5f,2.5f):1;
        m_stopStartDistance=m_distance;
        transition(AnimatedRagdollPhase3D::Stopping);
    }
    if(m_telemetry.phase==AnimatedRagdollPhase3D::Stopping&&m_phaseTime*m_stopRate>=clips.stop->durationSeconds)
        transition(AnimatedRagdollPhase3D::Idle);
    const AnimationClip3D* clip=clips.idle;
    if(m_telemetry.phase==AnimatedRagdollPhase3D::Running)clip=clips.run;
    if(m_telemetry.phase==AnimatedRagdollPhase3D::Stopping)clip=clips.stop;
    const float sourceRunSpeed=std::max(0.5f,clips.run->sourceRootDisplacementMeters.x/clips.run->durationSeconds);
    const float runSpeed=m_settings.runSpeedMetersPerSecond>0?m_settings.runSpeedMetersPerSecond:sourceRunSpeed;
    const float previousSpeed=m_speed;
    m_speed=m_telemetry.phase==AnimatedRagdollPhase3D::Running
        ?std::clamp(runSpeed,0.5f,8.0f)*smooth(m_phaseTime/0.35f):0;
    const float sampleTime=m_phaseTime*(m_telemetry.phase==AnimatedRagdollPhase3D::Stopping?m_stopRate:
        m_telemetry.phase==AnimatedRagdollPhase3D::Running?runSpeed/sourceRunSpeed:1.0f);
    if(m_telemetry.phase==AnimatedRagdollPhase3D::Stopping) {
        const auto* root=findAnimationTrack3D(*clips.stop,profile.links.front().id);
        const float travel=root?sampleAnimationTrack3D(*root,sampleTime,clips.stop->durationSeconds,false).translationOffsetMeters.x
            -sampleAnimationTrack3D(*root,0,clips.stop->durationSeconds,false).translationOffsetMeters.x:0;
        const float distance=m_stopStartDistance+(m_telemetry.blocked?0:travel);
        m_speed=(distance-m_distance)/dt;m_distance=distance;
    } else if(m_telemetry.phase==AnimatedRagdollPhase3D::Running)
        m_distance+=(previousSpeed+m_speed)*0.5f*dt;
    const Vec3 forward=m_heading.rotate({1,0,0});
    m_output.targetPose.linkPositions.resize(profile.links.size());
    m_output.targetPose.linkOrientations.resize(profile.links.size());
    auto& positions=m_output.targetPose.linkPositions;
    auto& orientations=m_output.targetPose.linkOrientations;
    const float blend=smooth(m_blendTime/(m_telemetry.phase==AnimatedRagdollPhase3D::Stopping?0.12f:0.35f));
    float jointError=0; std::size_t dofs=0;
    for(std::size_t i=0;i<profile.links.size();++i) {
        const auto& link=profile.links[i];
        const auto* track=findAnimationTrack3D(*clip,link.id);
        const auto sample=track?sampleAnimationTrack3D(*track,sampleTime,clip->durationSeconds,clip->loops):AnimationTransformSample3D{};
        if(i==0) {
            // Preserve pitch/roll from the source, but no animated heading or
            // sideways root drift: this command explicitly means straight ahead.
            const auto lean=heading(sample.rotationDelta).conjugate()*sample.rotationDelta;
            orientations[i]=(m_heading*lean*link.modelOrientation).normalized();
            positions[i]=m_routeOrigin+forward*m_distance;
            positions[i].z=0;
            continue;
        }
        Vec3 coordinates;
        for(std::size_t a=0;a<3;++a) {
            const auto& axis=link.inboundJoint.axes[a];
            const float old=component(m_coordinates[i],a);
            const float wanted=component(m_blendFrom[i],a)*(1-blend)+component(sample.jointPositionRadians,a)*blend;
            const float value=axis.enabled?std::clamp(wanted,axis.minimumRadians,axis.maximumRadians):0;
            setComponent(coordinates,a,value);
            if(!axis.enabled)continue;
            RagdollDriveTarget3D drive;
            drive.linkIndex=static_cast<std::uint32_t>(i);drive.axis=static_cast<RagdollAxis3D>(a);
            drive.positionRadians=value; drive.velocityRadiansPerSecond=std::clamp((value-old)/dt,-20.0f,20.0f);
            drive.stiffnessScale=2;drive.dampingScale=1.1f;drive.maximumTorqueScale=2;
            m_output.driveTargets.push_back(drive);
            if(i<state.joints.size())jointError+=std::pow(value-state.joints[i].positionRadians[a],2.0f);
            ++dofs;
        }
        m_coordinates[i]=coordinates;
        const auto parent=static_cast<std::size_t>(link.parentIndex);
        const auto& parentLink=profile.links[parent];
        const auto frame=link.inboundJoint.frameModelOrientation;
        const auto parentFrame=parentLink.modelOrientation.conjugate()*frame;
        const auto childFrame=link.modelOrientation.conjugate()*frame;
        orientations[i]=(orientations[parent]*parentFrame*rotation(coordinates)*childFrame.conjugate()).normalized();
        const auto anchor=link.inboundJoint.anchorModelPosition;
        positions[i]=positions[parent]+orientations[parent].rotate(parentLink.modelOrientation.conjugate().rotate(anchor-parentLink.modelPosition))
            -orientations[i].rotate(link.modelOrientation.conjugate().rotate(anchor-link.modelPosition));
    }
    // Fit the soles to the sampled floor using physical contact geometry, not
    // guessed mesh dimensions. No procedural footsteps are generated here.
    float minimumSole=0;
    for(std::size_t i=0;i<profile.links.size();++i) {
        const auto& c=profile.links[i].collider;
        if(c.shape!=RagdollColliderShape3D::Box)continue;
        const auto q=orientations[i]*c.localOrientation;
        const Vec3 center=positions[i]+orientations[i].rotate(c.localPosition);
        const float extent=std::abs(q.rotate({c.boxHalfExtents.x,0,0}).z)
            +std::abs(q.rotate({0,c.boxHalfExtents.y,0}).z)+std::abs(q.rotate({0,0,c.boxHalfExtents.z}).z);
        minimumSole=std::min(minimumSole,center.z-extent);
    }
    // The support target is the surface itself, not an 8 mm hover margin.
    // PhysX contact/rest offsets provide collision tolerance independently.
    for(auto& p:positions)p.z+=m_groundHeight-minimumSole;
    m_telemetry.assistanceForceNewtons=0;
    for(std::size_t i=0;i<positions.size();++i) {
        const auto& body=state.links[i];
        const Vec3 targetVelocity=m_previousValid?(positions[i]-m_previousTarget.linkPositions[i])/dt:Vec3{};
        const Vec3 angularVelocity=m_previousValid?rotationVector(orientations[i]*m_previousTarget.linkOrientations[i].conjugate())/dt:Vec3{};
        if(!m_settings.assistanceEnabled)continue;
        const float mass=profile.totalMassKg*profile.links[i].massFraction;
        Vec3 acceleration=(positions[i]-body.position)*180.0f+(limited(targetVelocity,12)-body.linearVelocity)*24.0f+Vec3{0,0,9.81f*0.9f};
        const auto force=limited(acceleration,std::clamp(m_settings.maximumAssistAcceleration,0.0f,120.0f))*mass;
        const auto angularError=rotationVector(orientations[i]*body.orientation.conjugate());
        const float angularGain=i==0?800.0f:180.0f;
        const float angularDamping=i==0?55.0f:24.0f;
        const float inertiaScale=i==0?0.05f:0.025f;
        const auto torque=limited((angularError*angularGain+(limited(angularVelocity,20)-body.angularVelocity)*angularDamping)
            *(mass*inertiaScale),mass*(i==0?30.0f:12.0f));
        m_output.assistance.push_back({static_cast<std::uint32_t>(i),force,torque});
        m_telemetry.assistanceForceNewtons+=force.length();
    }
    m_previousTarget=m_output.targetPose; m_previousValid=true;
    m_telemetry.phaseSeconds=m_phaseTime;
    m_telemetry.runSecondsRemaining=m_telemetry.phase==AnimatedRagdollPhase3D::Running?std::max(0.0f,m_requestedSeconds-m_phaseTime):0;
    m_telemetry.rootErrorMeters=(positions.front()-state.links.front().position).length();
    m_telemetry.speedMetersPerSecond=dot(state.links.front().linearVelocity,forward);
    m_telemetry.travelledMeters=dot(state.links.front().position-m_commandOrigin,forward);
    m_telemetry.jointRmsDegrees=std::sqrt(jointError/static_cast<float>(std::max(std::size_t{1},dofs)))*180/Pi;
}
} // namespace MatterEngine
