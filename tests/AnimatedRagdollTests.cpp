#include "Engine/Control/AnimatedRagdollController3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace MatterEngine;
void testBodyRelativeRagdollAssistance(const RagdollProfile3D&, const AnimatedRagdollClips3D&,
    PhysicsEngine3D&, MaterialLibrary&);
namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
void ground(PhysicsEngine3D& engine,PhysicsScene3D& scene) {
    MeshData3D mesh;
    for(auto p:{Vec3{-80,-80,0},Vec3{80,-80,0},Vec3{80,80,0},Vec3{-80,80,0}}) {
        MeshVertex3D vertex;vertex.position=p;mesh.vertices.push_back(vertex);
    }
    mesh.indices={0,1,2,0,2,3};
    PhysicsShape3D shape;shape.type=PhysicsShapeType3D::TriangleMesh;
    shape.mesh=engine.cookStaticTriangleMesh(mesh);shape.materialId="concrete";
    PhysicsBodyDefinition3D body;body.motionType=PhysicsMotionType3D::Static;
    (void)scene.createBody(body,std::span<const PhysicsShape3D>(&shape,1));
}
}
int main(int argc,char** argv) {try {
    const bool idleDiagnostic=argc>1&&std::string(argv[1])=="--diagnostic-idle";
    const bool diagnostic=idleDiagnostic||(argc>1&&std::string(argv[1])=="--diagnostic");
    const std::string assets=MATTERENGINE_TEST_ASSETS_DIR;
    const auto character=loadRagdollCharacter3D(assets+"/characters/crash_test_dummy/character.json");
    const auto idle=loadAnimationClip3D(assets+"/animations/clips/idle_dummy.matteranim.json");
    const auto run=loadAnimationClip3D(assets+"/animations/clips/run_forward_dummy.matteranim.json");
    const auto stop=loadAnimationClip3D(assets+"/animations/clips/run_to_stop_dummy.matteranim.json");
    AnimatedRagdollClips3D clips{&idle,&run,&stop};
    const auto& profile=character.profile;
    require(clips.compatible(profile),"Invalid locomotion clips");
    PhysicsEngine3D engine;MaterialLibrary materials;
    if(argc>1&&std::string(argv[1])=="--assistance-contracts") {
        testBodyRelativeRagdollAssistance(profile,clips,engine,materials);
        return 0;
    }
    for(float yaw:{0.0f,1.1f,2.9f}) {
        auto scene=engine.createScene({},materials);ground(engine,*scene);
        RagdollSpawnDefinition3D spawn;spawn.entityId=80001;
        spawn.pelvisPosition={0,0,profile.standingRootHeightMeters+0.055f};
        spawn.orientation=Quaternion::fromAxisAngle({0,0,1},yaw);spawn.active=true;
        const auto handle=scene->createRagdoll(profile,spawn);
        AnimatedRagdollController3D controller;
        controller.reset(profile,scene->ragdollState(handle),0);
        float maxJointError=0,maxGap=0,minUp=1,maxLimitViolation=0;
        bool sawRunning=false,sawStopping=false;
        float maximumVisualSoleGap=0, runningSoleGapSum=0, idleSoleGapSum=0;
        int soleSamples=0,idleSoleSamples=0,groundedSamples=0;
        int runningTicks=0;
        Vec3 runOrigin,runForward;
        for(int step=0;step<120*10;++step) {
            const auto state=scene->ragdollState(handle);
            if(step==240&&!idleDiagnostic) {
                const bool accepted=controller.requestRun(state);
                if(!diagnostic)require(accepted,"Idle did not accept run command");
                require(!controller.requestRun(state),"Repeated click restarted running");
                runOrigin=state.links.front().position;
                runForward=state.links.front().orientation.rotate({1,0,0});runForward.z=0;runForward=runForward.normalized();
            }
            controller.update(profile,clips,state,1.0f/120,0);
            const auto& output=controller.output();const auto& t=controller.telemetry();
            if(diagnostic&&yaw==0&&step%60==0&&step<240) {
                for(std::size_t i=12;i<profile.links.size();++i)
                    std::cout<<profile.links[i].id<<" pos "<<state.links[i].position.x<<","<<state.links[i].position.z
                        <<" angles "<<state.joints[i].positionRadians[0]<<","<<state.joints[i].positionRadians[1]
                        <<" up "<<state.links[i].orientation.rotate({0,0,1}).x<<std::endl;
            }
            if(step>120&&step%4==0) {
                std::vector<Vec3> p;std::vector<Quaternion> q;
                for(const auto& link:state.links){p.push_back(link.position);q.push_back(link.orientation);}
                const auto palette=buildRagdollSkinMatrices3D(character,p,q);
                float visualSole=100, colliderSole=100;
                for(const auto& vertex:character.mesh.vertices)visualSole=std::min(visualSole,skinVertexPosition3D(vertex,palette).z);
                for(std::size_t i=0;i<profile.links.size();++i) {
                    const auto& c=profile.links[i].collider;if(!c.contactSensor)continue;
                    const auto r=q[i]*c.localOrientation;
                    const float extent=std::abs(r.rotate({c.boxHalfExtents.x,0,0}).z)
                        +std::abs(r.rotate({0,c.boxHalfExtents.y,0}).z)+std::abs(r.rotate({0,0,c.boxHalfExtents.z}).z);
                    colliderSole=std::min(colliderSole,p[i].z+q[i].rotate(c.localPosition).z-extent);
                }
                if(t.phase==AnimatedRagdollPhase3D::Running) {
                    maximumVisualSoleGap=std::max(maximumVisualSoleGap,visualSole);
                    runningSoleGapSum+=visualSole;++soleSamples;
                    // Speculative CCD can report contacts before actual touch.
                    groundedSamples+=colliderSole<0.005f;
                } else if(t.phase==AnimatedRagdollPhase3D::Idle) {idleSoleGapSum+=visualSole;++idleSoleSamples;}
                if(step%120==0)std::cout<<"SOLE visual="<<visualSole<<" collider="<<colliderSole<<" contacts="<<state.contacts.size()<<std::endl;
            }
            sawRunning|=t.phase==AnimatedRagdollPhase3D::Running;
            sawStopping|=t.phase==AnimatedRagdollPhase3D::Stopping;
            runningTicks+=t.phase==AnimatedRagdollPhase3D::Running;
            require(t.assistanceForceNewtons<=profile.totalMassKg*9.81f*0.15f+0.01f,"Excessive assistance");
            for(const auto& drive:output.driveTargets) {
                const auto& axis=profile.links[drive.linkIndex].inboundJoint.axes[static_cast<std::size_t>(drive.axis)];
                require(drive.positionRadians>=axis.minimumRadians-1e-5f&&drive.positionRadians<=axis.maximumRadians+1e-5f,"Illegal joint target");
                const auto angle=state.joints[drive.linkIndex].positionRadians[static_cast<std::size_t>(drive.axis)];
                maxLimitViolation=std::max({maxLimitViolation,axis.minimumRadians-angle,angle-axis.maximumRadians});
            }
            scene->setRagdollActiveDriveTargets(handle,output.driveTargets,output.gravityCompensationEnabled);
            for(const auto& force:output.assistance) {
                require(std::isfinite(force.forceNewtons.length())&&std::isfinite(force.torqueNewtonMeters.length()),"Non finite assistance");
                scene->applyRagdollLinkForce(handle,force.linkIndex,force.forceNewtons,force.torqueNewtonMeters);
            }
            scene->simulate(1.0f/120);
            for(std::size_t i=1;i<state.links.size();++i) {
                const auto& link=profile.links[i];const auto p=static_cast<std::size_t>(link.parentIndex);
                const auto anchor=link.inboundJoint.anchorModelPosition;
                const Vec3 a=state.links[p].position+state.links[p].orientation.rotate(profile.links[p].modelOrientation.conjugate().rotate(anchor-profile.links[p].modelPosition));
                const Vec3 b=state.links[i].position+state.links[i].orientation.rotate(link.modelOrientation.conjugate().rotate(anchor-link.modelPosition));
                maxGap=std::max(maxGap,(a-b).length());
            }
            if(step>120) {
                maxJointError=std::max(maxJointError,t.jointRmsDegrees);
                minUp=std::min(minUp,state.links.front().orientation.rotate({0,0,1}).z);
            }
            if(step%(diagnostic?30:120)==0)std::cout<<"yaw="<<yaw<<" t="<<step/120.0f<<" phase="<<static_cast<int>(t.phase)
                <<" distance="<<t.travelledMeters<<" speed="<<t.speedMetersPerSecond
                <<" jointRMS="<<t.jointRmsDegrees<<" up="<<state.links.front().orientation.rotate({0,0,1}).z
                <<" height="<<state.links.front().position.z<<" load="<<t.supportLoadNewtons<<" aid="<<t.assistanceForceNewtons<<std::endl;
        }
        const auto state=scene->ragdollState(handle);
        const auto travel=state.links.front().position-runOrigin;
        const auto side=cross(Vec3{0,0,1},runForward);
        std::cout<<"RESULT yaw="<<yaw<<" maxJointRMS="<<maxJointError<<" minUp="<<minUp<<" gap="<<maxGap
            <<" limitViolationRadians="<<maxLimitViolation<<std::endl;
        std::cout<<"SOLE RESULT mean running="<<runningSoleGapSum/soleSamples<<" max="<<maximumVisualSoleGap
            <<" mean idle="<<idleSoleGapSum/idleSoleSamples<<" grounded="<<groundedSamples<<"/"<<soleSamples<<std::endl;
        if(diagnostic)continue;
        require(sawRunning&&sawStopping&&controller.telemetry().phase==AnimatedRagdollPhase3D::Idle,"Sequence did not complete");
        require(runningTicks>=598&&runningTicks<=601,"Run was not five seconds");
        require(dot(travel,runForward)>20&&std::abs(dot(travel,side))<0.3f,"Run did not follow body heading");
        require(state.links.front().linearVelocity.length()<0.3f,"Did not stop after braking");
        require(minUp>0.9f&&maxGap<0.005f,"Physical tracking unstable");
        require(maxJointError<12.0f,"Physical pose drifted from the animation");
        require(maxLimitViolation<0.025f,"Physical joint exceeded solver limit tolerance");
        controller.settings().assistanceEnabled=false;
        controller.update(profile,clips,state,1.0f/120,0);
        require(controller.output().assistance.empty(),"Assistance toggle ignored");
        controller.update(profile,clips,state,1.0f/120,0,true);
        require(controller.output().assistance.empty(),"Assistance fights Physgun");
        for(const auto& drive:controller.output().driveTargets)require(drive.maximumTorqueScale==0,"Drives fight Physgun");
        controller.update(profile,clips,state,1.0f/120,0,false);
        require(controller.telemetry().phase==AnimatedRagdollPhase3D::Idle,"Release did not return to Idle");
        require(controller.requestRun(state),"Could not run again after completion/release");
        controller.update(profile,clips,state,1.0f/120,0,false,true);
        require(controller.telemetry().blocked&&controller.telemetry().phase==AnimatedRagdollPhase3D::Stopping,
            "Obstacle did not interrupt running");
    }
    std::cout<<(diagnostic?"Diagnostic finished (not acceptance)\n":"Animated ragdoll tests passed\n");return 0;
}catch(const std::exception& e){std::cerr<<"Animated ragdoll failure: "<<e.what()<<'\n';return 1;}}
