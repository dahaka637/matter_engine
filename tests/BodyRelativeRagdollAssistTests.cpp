#include "Engine/Control/AnimatedRagdollController3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace MatterEngine;
namespace {
constexpr float Dt = 1.0f / 120.0f;
void check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
void near(Vec3 actual, Vec3 expected, const char *message,
          float tolerance = 0.004f) {
  check((actual - expected).length() < tolerance, message);
}
Vec3 center(const RagdollProfile3D &profile, const RagdollState3D &state) {
  Vec3 com;
  float sum = 0;
  for (std::size_t i = 0; i < state.links.size(); ++i) {
    const float fraction = profile.links[i].massFraction;
    com += (state.links[i].position + state.links[i].orientation.rotate(
                                          profile.links[i].centerOfMassLocal)) *
           fraction;
    sum += fraction;
  }
  return com / sum;
}
RagdollState3D translated(RagdollState3D state, Vec3 offset) {
  for (auto &link : state.links)
    link.position += offset;
  for (auto &contact : state.contacts)
    contact.position += offset;
  return state;
}
void sameWrenches(const std::vector<RagdollAssistWrench3D> &a,
                  const std::vector<RagdollAssistWrench3D> &b,
                  Quaternion rotation = {}) {
  check(a.size() == b.size(), "Different assistance after a rigid transform");
  for (std::size_t i = 0; i < a.size(); ++i) {
    check(a[i].linkIndex == b[i].linkIndex, "Mismatched force link");
    near(b[i].forceNewtons, rotation.rotate(a[i].forceNewtons),
         "Force remembers world position/frame");
    near(b[i].torqueNewtonMeters, rotation.rotate(a[i].torqueNewtonMeters),
         "Torque remembers world position/frame");
  }
}
void floor(PhysicsEngine3D &engine, PhysicsScene3D &scene) {
  MeshData3D mesh;
  for (Vec3 p : {Vec3{-20, -20, 0}, Vec3{20, -20, 0}, Vec3{20, 20, 0},
                 Vec3{-20, 20, 0}}) {
    MeshVertex3D vertex;
    vertex.position = p;
    mesh.vertices.push_back(vertex);
  }
  mesh.indices = {0, 1, 2, 0, 2, 3};
  PhysicsShape3D shape;
  shape.type = PhysicsShapeType3D::TriangleMesh;
  shape.mesh = engine.cookStaticTriangleMesh(mesh);
  shape.materialId = "concrete";
  PhysicsBodyDefinition3D body;
  body.motionType = PhysicsMotionType3D::Static;
  (void)scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}
void tick(PhysicsScene3D &scene, RagdollHandle3D handle,
          AnimatedRagdollController3D &controller,
          const RagdollProfile3D &profile,
          const AnimatedRagdollClips3D &clips) {
  controller.update(profile, clips, scene.ragdollState(handle), Dt, 0);
  const auto &out = controller.output();
  scene.setRagdollActiveDriveTargets(handle, out.driveTargets,
                                     out.gravityCompensationEnabled);
  for (const auto &wrench : out.assistance)
    scene.applyRagdollLinkForce(handle, wrench.linkIndex, wrench.forceNewtons,
                                wrench.torqueNewtonMeters);
  scene.simulate(Dt);
}
} // namespace

void testBodyRelativeRagdollAssistance(const RagdollProfile3D &profile,
                                       const AnimatedRagdollClips3D &clips,
                                       PhysicsEngine3D &engine,
                                       MaterialLibrary &materials) {
  auto fixture = engine.createScene({}, materials);
  RagdollSpawnDefinition3D spawn;
  spawn.entityId = 94001;
  spawn.pelvisPosition = {0, 0, profile.standingRootHeightMeters + 0.055f};
  const auto handle = fixture->createRagdoll(profile, spawn);
  auto state = fixture->ragdollState(handle);
  const auto &root = state.links.front();
  const Vec3 rootCom =
      root.position +
      root.orientation.rotate(profile.links.front().centerOfMassLocal);
  BodyRelativeRagdollReference3D reference;
  for (std::size_t i = 0; i < state.links.size(); ++i) {
    reference.positionsBody.push_back(root.orientation.conjugate().rotate(
        state.links[i].position +
        state.links[i].orientation.rotate(profile.links[i].centerOfMassLocal) -
        rootCom));
    reference.velocitiesBody.push_back({});
    if (profile.links[i].collider.contactSensor)
      state.contacts.push_back(
          {static_cast<std::uint32_t>(i),
           {state.links[i].position.x, state.links[i].position.y, 0},
           {0, 0, 1},
           profile.totalMassKg * 9.81f * Dt * 0.5f,
           0});
  }
  BodyRelativeRagdollAssistSettings3D settings;
  reference.balanceAuthority = reference.movementAuthority = 0;
  // A pure probe of the pose helper, not a new animation/retarget pose.
  reference.positionsBody.back() += Vec3{0.03f, -0.02f, 0.07f};
  const auto shape =
      computeBodyRelativeRagdollAssist3D(profile, state, reference, settings);
  check(shape.forceSumNewtons > 0.1f, "Vacuous pose-assistance test");
  Vec3 moment;
  const Vec3 com = center(profile, state);
  float vertical = 0;
  for (const auto &wrench : shape.wrenches) {
    const auto i = wrench.linkIndex;
    const Vec3 point =
        state.links[i].position +
        state.links[i].orientation.rotate(profile.links[i].centerOfMassLocal);
    moment +=
        cross(point - com, wrench.forceNewtons) + wrench.torqueNewtonMeters;
    vertical += std::abs(wrench.forceNewtons.z);
  }
  near(shape.netForceWorld, {},
       "Pose assistance translates/lifts the entire body");
  near(moment, {}, "Pose assistance secretly rotates the whole body");
  check(vertical > 0.1f, "Vertical shape assistance was incorrectly forbidden");

  const Vec3 offset{7, -9, 4};
  sameWrenches(shape.wrenches,
               computeBodyRelativeRagdollAssist3D(
                   profile, translated(state, offset), reference, settings)
                   .wrenches);

  // Rigid advection/rotation is not articulation velocity. Damping must not
  // fight a carried body merely because it moved through the world.
  auto moving = state;
  const Vec3 omega{0.3f, -0.4f, 0.5f};
  for (std::size_t i = 0; i < moving.links.size(); ++i) {
    const Vec3 point =
        moving.links[i].position +
        moving.links[i].orientation.rotate(profile.links[i].centerOfMassLocal);
    moving.links[i].linearVelocity =
        Vec3{5, -3, 2} + cross(omega, point - rootCom);
    moving.links[i].angularVelocity = omega;
  }
  sameWrenches(shape.wrenches, computeBodyRelativeRagdollAssist3D(
                                   profile, moving, reference, settings)
                                   .wrenches);

  reference.movementAuthority = reference.balanceAuthority = 1;
  reference.velocityHeading = {2, 0.5f, 0.8f};
  reference.upHeading = Vec3{0.1f, 0, 1}.normalized();
  const auto all =
      computeBodyRelativeRagdollAssist3D(profile, state, reference, settings);
  auto turned = state;
  const auto turn = Quaternion::fromAxisAngle({0, 0, 1}, 0.9f);
  for (auto &link : turned.links) {
    link.position = turn.rotate(link.position);
    link.orientation = turn * link.orientation;
    link.linearVelocity = turn.rotate(link.linearVelocity);
    link.angularVelocity = turn.rotate(link.angularVelocity);
  }
  sameWrenches(
      all.wrenches,
      computeBodyRelativeRagdollAssist3D(profile, turned, reference, settings)
          .wrenches,
      turn);

  reference.poseAuthority = reference.balanceAuthority = 0;
  for (float desiredZ : {-0.8f, 0.8f}) {
    reference.velocityHeading = {0, 0, desiredZ};
    const auto verticalAid =
        computeBodyRelativeRagdollAssist3D(profile, state, reference, settings);
    near(verticalAid.netForceWorld,
         {0, 0, profile.totalMassKg * 0.5f * desiredZ},
         "XYZ movement intent was projected onto the horizontal plane");
  }
  reference.poseAuthority = reference.balanceAuthority = 1;
  reference.velocityHeading = {100, -100, 100};
  for (auto &p : reference.positionsBody)
    p += Vec3{100, 100, -100};
  settings.maximumAssistWeightFraction =
      settings.maximumAssistTorqueWeightLengthFraction = 1000;
  const auto capped =
      computeBodyRelativeRagdollAssist3D(profile, state, reference, settings);
  check(capped.forceSumNewtons <= profile.totalMassKg * 9.81f * 0.15f + 0.001f,
        "Per-link assistance bypasses whole-body force budget");
  check(capped.torqueSumNewtonMeters <=
            profile.totalMassKg * 9.81f * profile.standingRootHeightMeters *
                    0.05f +
                0.001f,
        "Reaction torque bypasses whole-body torque budget");
  reference.poseAuthority = reference.movementAuthority = 0;
  reference.balanceAuthority = 1;
  reference.upHeading = {1, 0, 0};
  const auto torqueCapped =
      computeBodyRelativeRagdollAssist3D(profile, state, reference, settings);
  check(std::abs(torqueCapped.torqueSumNewtonMeters -
                 profile.totalMassKg * 9.81f *
                     profile.standingRootHeightMeters * 0.05f) < 0.002f,
        "Torque saturation was not exercised");
  settings.assistanceEnabled = false;
  check(computeBodyRelativeRagdollAssist3D(profile, state, reference, settings)
            .wrenches.empty(),
        "Disabled assistance emitted wrenches");

  // Control-law regression: identical pose/phase, different position history.
  // Synthetic snapshots deliberately isolate the contract from gait stability.
  AnimatedRagdollController3D original;
  original.reset(profile, state, 0);
  check(original.requestRun(state), "Run command refused in unit fixture");
  for (int i = 0; i < 80; ++i)
    original.update(profile, clips, state, Dt, 0);
  auto displaced = original;
  const auto farState = translated(state, offset);
  bool sawStopping = false;
  for (int i = 0; i < 900; ++i) {
    original.update(profile, clips, state, Dt, 0);
    displaced.update(profile, clips, farState, Dt, offset.z);
    sameWrenches(original.output().assistance, displaced.output().assistance);
    const auto &a = original.output().driveTargets;
    const auto &b = displaced.output().driveTargets;
    check(a.size() == b.size(), "World displacement changed motor count");
    for (std::size_t j = 0; j < a.size(); ++j) {
      check(std::abs(a[j].positionRadians - b[j].positionRadians) < 1e-6f,
            "Motor target depends on world position");
      const auto &axis =
          profile.links[a[j].linkIndex]
              .inboundJoint.axes[static_cast<std::size_t>(a[j].axis)];
      check(a[j].positionRadians >= axis.minimumRadians - 1e-6f &&
                a[j].positionRadians <= axis.maximumRadians + 1e-6f,
            "Illegal motor target");
    }
    near(displaced.output().targetPose.linkPositions.front(),
         farState.links.front().position,
         "Animation root did not follow displaced body");
    sawStopping |=
        original.telemetry().phase == AnimatedRagdollPhase3D::Stopping;
  }
  check(sawStopping &&
            original.telemetry().phase == AnimatedRagdollPhase3D::Idle,
        "State machine did not complete in the isolated contract fixture");
  displaced.update(profile, clips, farState, Dt, offset.z, true);
  check(displaced.output().assistance.empty() &&
            !displaced.output().gravityCompensationEnabled,
        "Controller fought manipulation");
  for (const auto &drive : displaced.output().driveTargets)
    check(drive.maximumTorqueScale == 0, "Motor fought manipulation");
  displaced.update(profile, clips, farState, Dt, offset.z, false);
  AnimatedRagdollController3D fresh;
  fresh.reset(profile, farState, offset.z);
  fresh.update(profile, clips, farState, Dt, offset.z);
  sameWrenches(fresh.output().assistance, displaced.output().assistance);
  displaced.update(profile, clips, farState,
                   std::numeric_limits<float>::quiet_NaN(), offset.z);
  check(displaced.output().assistance.empty() &&
            displaced.output().driveTargets.empty(),
        "Invalid step reused previous forces");

  // Real physical collision against a shin, compared with an unhit control.
  // This is a compliance test, not proof that the gait can yet balance.
  auto baselineScene = engine.createScene({}, materials);
  auto struckScene = engine.createScene({}, materials);
  floor(engine, *baselineScene);
  floor(engine, *struckScene);
  const auto baselineHandle = baselineScene->createRagdoll(profile, spawn);
  const auto struckHandle = struckScene->createRagdoll(profile, spawn);
  AnimatedRagdollController3D baselineController, struckController;
  for (int i = 0; i < 12; ++i) {
    tick(*baselineScene, baselineHandle, baselineController, profile, clips);
    tick(*struckScene, struckHandle, struckController, profile, clips);
  }
  const auto beforeHit = struckScene->ragdollState(struckHandle);
  std::size_t shin = 0;
  for (std::size_t i = 0; i < profile.links.size(); ++i)
    if (profile.links[i].collider.contactSensor)
      shin = static_cast<std::size_t>(profile.links[i].parentIndex);
  check(shin > 0, "No shin fixture found");
  PhysicsBodyDefinition3D box;
  box.entityId = 94002;
  box.massKg = 25;
  box.position = beforeHit.links[shin].position + Vec3{0, -0.45f, 0.05f};
  box.linearVelocity = {0, 8, 0};
  box.collisionMode = PhysicsCollisionMode3D::Continuous;
  PhysicsShape3D boxShape;
  boxShape.type = PhysicsShapeType3D::Box;
  boxShape.halfExtents = {0.12f, 0.15f, 0.12f};
  const auto projectile = struckScene->createBody(
      box, std::span<const PhysicsShape3D>(&boxShape, 1));
  for (int i = 0; i < 60; ++i) {
    tick(*baselineScene, baselineHandle, baselineController, profile, clips);
    tick(*struckScene, struckHandle, struckController, profile, clips);
  }
  const auto baseline = baselineScene->ragdollState(baselineHandle);
  const auto struck = struckScene->ragdollState(struckHandle);
  const float lateralDisplacement =
      center(profile, struck).y - center(profile, baseline).y;
  std::cout << "Shin collision: COM displacement versus control = "
            << lateralDisplacement << " m; body up = "
            << struck.links.front().orientation.rotate({0, 0, 1}).z
            << "; control up = "
            << baseline.links.front().orientation.rotate({0, 0, 1}).z
            << "; projectile y = "
            << struckScene->bodyState(projectile).position.y << '\n';
  check(lateralDisplacement > 0.08f,
        "Object impact was resisted like a world anchor");
  check(baseline.links.front().orientation.rotate({0, 0, 1}).z > 0.9f,
        "Compliance fixture fell before it could distinguish an impact");
  check(struck.links.front().orientation.rotate({0, 0, 1}).z < 0.65f &&
            struckController.telemetry().phase ==
                AnimatedRagdollPhase3D::Falling &&
            struckController.output().assistance.empty(),
        "Assistance prevents yielding to a knockdown");

  // Shape assistance and internal gravity feedforward cannot provide hidden
  // floating-base lift. Deliberate vertical movement assistance is covered
  // separately above and is allowed, rather than banned from this system.
  auto airborneScene = engine.createScene({}, materials);
  spawn.pelvisPosition.z += 3;
  const auto airborneHandle = airborneScene->createRagdoll(profile, spawn);
  AnimatedRagdollController3D airborneController;
  const float initialHeight =
      center(profile, airborneScene->ragdollState(airborneHandle)).z;
  for (int i = 0; i < 60; ++i)
    tick(*airborneScene, airborneHandle, airborneController, profile, clips);
  const float drop =
      initialHeight -
      center(profile, airborneScene->ragdollState(airborneHandle)).z;
  std::cout << "Airborne COM drop in 0.5 s = " << drop << " m\n";
  check(std::abs(drop - 0.5f * 9.81f * 0.5f * 0.5f) < 0.06f,
        "Shape/internal feedforward is secretly supporting body weight");
  std::cout << "Body-relative assistance contracts passed (not locomotion "
               "acceptance)\n";
}
