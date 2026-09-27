#include "Engine/Control/BipedController3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace MatterEngine;
int main(int argc,char** argv){
    try {
    auto profile=loadRagdollProfile3D(std::string(MATTERENGINE_TEST_ASSETS_DIR)+"/characters/crash_test_dummy/CrashTestDummyV1.ragdoll.json");
    for(auto& l:profile.links)l.collider.contactSensor=true;
    PhysicsEngine3D engine;MaterialLibrary materials;PhysicsSceneSettings3D settings;
    auto scene=engine.createScene(settings,materials);
    PhysicsBodyDefinition3D ground;ground.motionType=PhysicsMotionType3D::Static;ground.position={0,0,-.1f};ground.materialId="concrete";
    PhysicsShape3D shape;shape.type=PhysicsShapeType3D::Box;shape.halfExtents={100,100,.1f};shape.materialId="concrete";
    auto floor=scene->createBody(ground,std::span<const PhysicsShape3D>(&shape,1));(void)floor;
    RagdollSpawnDefinition3D spawn;spawn.pelvisPosition={0,0,profile.standingRootHeightMeters+.002f};
    auto body=scene->createRagdoll(profile,spawn);auto state=scene->ragdollState(body);
    BipedController3D ctrl;ctrl.reset(profile,state);constexpr float dt=1.f/120;
    float duration=argc>1?std::stof(argv[1]):10;Vec3 command={argc>2?std::stof(argv[2]):0,argc>3?std::stof(argv[3]):0,0};ctrl.setVelocity(command);int unsolved=0;float minUp=1,minHeight=10;
    for(int tick=0;tick<int(duration/dt);++tick){
        ctrl.update(profile,state,scene->ragdollDynamics(body),dt);
        const auto& t=ctrl.telemetry();unsolved+=!t.solved;minUp=std::min(minUp,t.upright);minHeight=std::min(minHeight,t.height);
        if(tick%120==0)std::cout<<"t="<<tick*dt<<" phase="<<int(t.phase)<<" steps="<<t.steps<<" com="<<t.com.x<<","<<t.com.y<<","<<t.com.z<<" v="<<t.velocity.length()<<" up="<<t.upright<<" load="<<t.footLoad[0]<<","<<t.footLoad[1]<<" feet="<<state.links[14].position.x<<","<<state.links[14].position.y<<","<<state.links[14].position.z<<" / "<<state.links[17].position.x<<","<<state.links[17].position.y<<","<<state.links[17].position.z<<" solve="<<t.solved<<" it="<<t.solverIterations<<" err="<<t.dynamicsResidual<<" viol="<<t.constraintViolation<<std::endl;
        scene->setRagdollActiveDriveTargets(body,ctrl.targets(),false);scene->simulate(dt);state=scene->ragdollState(body);
    }
    if(command.length()>.01f && (ctrl.telemetry().steps<4 || dot(ctrl.telemetry().com,command.normalized())<.20f))throw std::runtime_error("Walking did not make physical progress");
    std::cout<<"minimum up="<<minUp<<" height="<<minHeight<<" unsolved="<<unsolved<<std::endl;
    if(minUp<.9 || minHeight<.75 || unsolved>10)throw std::runtime_error("Standing physical gate failed");
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}return 0;
}
