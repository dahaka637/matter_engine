# MatterEngine development rules

## Project continuity

- Read `DESENVOLVIMENTO_ATUAL.md` before resuming material engine work.
- Keep it updated when a development stage, decision, known problem,
  validation result, blocker, or next step changes.
- Record limitations honestly; passing an automated test is not equivalent to
  visual approval by the user.

## Architectural boundaries

- `src/Workbench` is the active application layer. It must not include SDL,
  Vulkan, Volk, VMA, or native platform headers.
- Vulkan types and calls belong only in `src/Engine/RHI/Vulkan`.
- Jolt headers, types and calls belong only in `src/Engine/Physics/Jolt`
  (the active backend). PhysX headers, types and calls belong only in
  `src/Engine/Physics/PhysX`, which stays selectable as a behavior reference
  until the migration is visually approved; a study copy lives in
  `archive/2026-09-25-physx-backend/`.
- Never `dynamic_cast` a Jolt type: the SDK is built without RTTI. Callbacks the
  SDK calls back into (contact/step listeners, trace/assert, jobs) must not
  throw.
- Public physics headers must remain backend-neutral.
- Public RHI headers must remain backend-neutral.
- Engine code must never include Workbench headers.
- The former SoccerFall runtime, legacy Footwork rig and animation editor were
  removed. They must not be reintroduced as dependencies or copied back into
  active code. A new backend-neutral locomotion module is allowed when built
  from the current engine contracts rather than copied from that prototype.
- The clean-room `PhysicalBipedController3D` is a new implementation; its name
  does not revive the retired `PhysicalBiped` prototype. Check its provenance
  by source and dependencies, not by a broad name match.
- Prefer the smallest engine capability required by a real Workbench feature;
  avoid speculative abstractions.
- Preserve the fixed 120 Hz simulation order: controller writes forces/motor
  targets, `PhysicsScene3D` advances the backend and fetches its results, then
  systems read contact telemetry. Controllers must not write body transforms.
- A carried ragdoll root is resolved by the backend AFTER the solver (the whole
  body is re-framed onto the animation guide). It must never be a kinematic
  body during the solve: two infinite-mass pelvises tear the limbs caught
  between them (measured 240 m/s and 11.8 cm joint separation).

## Verification

After engine or Workbench changes, run:

```powershell
.\tools\check-architecture.cmd
.\tools\build.cmd -Configuration Debug
ctest --test-dir build-modern -C Debug --output-on-failure
```

On Linux, run the equivalent native workflow:

```bash
./tools/check-architecture.sh
./tools/build.sh Debug
ctest --test-dir build-linux --output-on-failure
```

For Vulkan changes, also smoke-test startup, graceful close, resize,
minimize/restore and fullscreen. With the Vulkan SDK installed, validation
errors are release blockers.

## Source checks

These searches should remain empty:

```powershell
rg "#include <SDL|SDL_[A-Za-z]|SDL_Event" src/Workbench
rg "\\bVk[A-Z]|<volk|<vulkan|vk_mem" src -g "!src/Engine/RHI/Vulkan/**"
rg "\\bphysx::|#include.*(Px|characterkinematic|cooking/)" src -g "!src/Engine/Physics/PhysX/**"
rg "\\bJPH::|#include.*<Jolt/" src -g "!src/Engine/Physics/Jolt/**"
rg "PhysicsWorld3D|RigidBody3D|StaticCollisionWorld3D|TriangleMeshCollider3D|KinematicCharacter3D|PhysicsHandle3D|JointConstraint3D" CMakeLists.txt src tests
rg "SDL_opengl|\\bgl(Begin|End|Vertex|Color|LineWidth)|opengl32" src CMakeLists.txt
rg "#include.*Game/|src[/\\\\]Game" CMakeLists.txt src tests
rg "SoccerFall|SOCCERFALL|LegacyFootworkRig|AnimatorScreen" CMakeLists.txt src tests
```
