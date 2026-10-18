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
  (the only backend). PhysX was removed on 2026-10-02 after the Jolt
  migration; the final backend lives only as a study copy in
  `archive/2026-10-02-physx-backend-final/` and must not come back into
  `src`, CMake or the tools.
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
rg "\\bphysx::|#include.*(Px|characterkinematic|cooking/)" src
rg "\\bJPH::|#include.*<Jolt/" src -g "!src/Engine/Physics/Jolt/**"
rg "PhysicsWorld3D|RigidBody3D|StaticCollisionWorld3D|TriangleMeshCollider3D|KinematicCharacter3D|PhysicsHandle3D|JointConstraint3D" CMakeLists.txt src tests
rg "SDL_opengl|\\bgl(Begin|End|Vertex|Color|LineWidth)|opengl32" src CMakeLists.txt
rg "#include.*Game/|src[/\\\\]Game" CMakeLists.txt src tests
rg "SoccerFall|SOCCERFALL|LegacyFootworkRig|AnimatorScreen" CMakeLists.txt src tests
```

<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:46cd31e7 -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/core-concepts/sync-concepts.md for details and anti-patterns.

## Agent Context Profiles

The managed Beads block is task-tracking guidance, not permission to override repository, user, or orchestrator instructions.

- **Conservative (default)**: Use `bd` for task tracking. Do not run git commits, git pushes, or Dolt remote sync unless explicitly asked. At handoff, report changed files, validation, and suggested next commands.
- **Minimal**: Keep tool instruction files as pointers to `bd prime`; use the same conservative git policy unless active instructions say otherwise.
- **Team-maintainer**: Only when the repository explicitly opts in, agents may close beads, run quality gates, commit, and push as part of session close. A current "do not commit" or "do not push" instruction still wins.

## Session Completion

This protocol applies when ending a Beads implementation workflow. It is subordinate to explicit user, repository, and orchestrator instructions.

1. **File issues for remaining work** - Create beads for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **Handle git/sync by active profile**:
   ```bash
   # Conservative/minimal/default: report status and proposed commands; wait for approval.
   git status

   # Team-maintainer opt-in only, unless current instructions forbid it:
   git pull --rebase
   bd dolt push
   git push
   git status
   ```
5. **Hand off** - Summarize changes, validation, issue status, and any blocked sync/commit/push step

**Critical rules:**
- Explicit user or orchestrator instructions override this Beads block.
- Do not commit or push without clear authority from the active profile or the current user request.
- If a required sync or push is blocked, stop and report the exact command and error.
<!-- END BEADS INTEGRATION -->

<!-- BEGIN BEADS CODEX SETUP: generated by bd setup codex -->
## Beads Issue Tracker

Use Beads (`bd`) for durable task tracking in repositories that include it. Use the `beads` skill at `.agents/skills/beads/SKILL.md` (project install) or `~/.agents/skills/beads/SKILL.md` (global install) for Beads workflow guidance, then use the `bd` CLI for issue operations.

### Quick Reference

```bash
bd ready                # Find available work
bd show <id>            # View issue details
bd update <id> --claim  # Claim work
bd close <id>           # Complete work
bd prime                # Refresh Beads context
```

### Rules

- Use `bd` for all task tracking; do not create markdown TODO lists.
- Run `bd prime` when Beads context is missing or stale. Codex 0.129.0+ can load Beads context automatically through native hooks; use `/hooks` to inspect or toggle them.
- Keep persistent project memory in Beads via `bd remember`; do not create ad hoc memory files.

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/core-concepts/sync-concepts.md for details and anti-patterns.
<!-- END BEADS CODEX SETUP -->
