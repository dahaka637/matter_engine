# MatterEngine development rules

## Architectural boundaries

- `src/Workbench` is the active application layer. It must not include SDL,
  Vulkan, Volk, VMA, or native platform headers.
- Vulkan types and calls belong only in `src/Engine/RHI/Vulkan`.
- PhysX headers, types and calls belong only in `src/Engine/Physics/PhysX`.
- Public physics headers must remain backend-neutral.
- Public RHI headers must remain backend-neutral.
- Engine code must never include Workbench headers.
- The former SoccerFall runtime, Footwork rig and animation editor were removed.
  They must not be reintroduced as dependencies or copied back into active code.
- Prefer the smallest engine capability required by a real Workbench feature;
  avoid speculative abstractions.
- Preserve the fixed 120 Hz simulation order: controller writes forces/motor
  targets, `PhysicsScene3D` advances PhysX and fetches its results, then systems
  read contact telemetry. Controllers must not write body transforms.

## Verification

After engine or Workbench changes, run:

```powershell
.\tools\check-architecture.cmd
.\tools\build.cmd -Configuration Debug
ctest --test-dir build-modern -C Debug --output-on-failure
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
rg "PhysicsWorld3D|RigidBody3D|StaticCollisionWorld3D|TriangleMeshCollider3D|KinematicCharacter3D|PhysicsHandle3D|JointConstraint3D" CMakeLists.txt src tests
rg "SDL_opengl|\\bgl(Begin|End|Vertex|Color|LineWidth)|opengl32" src CMakeLists.txt
rg "#include.*Game/|src[/\\\\]Game" CMakeLists.txt src tests
rg "SoccerFall|SOCCERFALL|Footwork|PhysicalBiped|AnimatorScreen" CMakeLists.txt src tests
```
