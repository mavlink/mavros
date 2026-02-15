# AGENTS.md

Guidance for coding agents in this repository.

## Critical Rules

- Keep changes minimal and scoped; avoid unrelated churn.
- Source ROS setup before build/test:
  - `source /opt/ros/<distro>/setup.bash`
  - `source <ws>/install/setup.bash` (if present)
- CI `colcon test` is a merge gate and must pass before merge.
- Follow generated-code workflows (`cog`, docs extractors/templates); do not hand-edit generated regions.
- Use commit subject format: `<component>: ...` with short component names (for example: `msgs`, `extras`, `tools`).

## Repository Scope

- Monorepo packages: `mavros`, `mavros_extras`, `mavros_msgs`, `libmavconn`, `tools`.
- Respect package boundaries:
  - runtime/plugins: `mavros`, `mavros_extras`
  - interfaces: `mavros_msgs`
  - transport/link: `libmavconn`

## Build And Test

- Run build/test commands from workspace root (`/ws` in devcontainer), not from `/ws/src/mavros`.
- Build (workspace root):
  - `colcon build --packages-up-to mavros mavros_extras mavros_msgs`
- Local test runs are not required on every iteration, but are recommended for risky changes:
  - `colcon test --packages-select mavros mavros_extras mavros_msgs`
  - `colcon test-result --verbose`
- Tool-only extractor changes:
  - `cmake -S tools -B tools/build`
  - `cmake --build tools/build -j$(nproc)`
- When tests fail, report first failing target and log path; distinguish env/setup issues from regressions.

## Devcontainer

- Prefer devcontainer for reproducible local build/test when available.
- Default config: `.devcontainer/devcontainer.json`
  - Scope-limited mount to `/ws/src/mavros` (only this repository).
  - Run `colcon` from `/ws`.
- Optional full-workspace config: `.devcontainer/whole-workspace/devcontainer.json`
  - Mounts `${localWorkspaceFolder}/../..` to `/ws`.
  - Requires repository path `<workspace>/src/mavros`.

## Style And Compatibility

- C++ style is enforced by `ament_uncrustify` (latest/current supported ROS 2 baseline).
- Python style is enforced with `ruff`, while staying compatible with ROS 2 conventions used here.
- Avoid style-only edits outside touched scope.
- Keep compatibility across supported ROS 2 distros; prefer conditional handling over distro-specific breakage.
- Treat plugin API surface as stable: topic/service/parameter names and defaults should not change unless intentional.

## Generated Code (Cog)

- Cog blocks are authoritative for generated regions in CMake/XML/C++ files.
- Do not edit generated body text directly; edit the cog snippet and regenerate.
- Regenerate:
  - single file: `mr-cog -cr <file>`
  - all files: `./tools/cogall.sh`
- If checksum-protected output was edited, remove checksum suffix from `[[[end]]]` first, then regenerate.
- Some checksum drift after `ament_uncrustify` is acceptable; avoid checksum/style-only churn when possible.
- Known non-fatal `cogall` message is expected from `mavros/src/lib/enum_sensor_orientation.cpp` (`Parse Error ... desc: Custom orientation`).

## Plugin Docs Tooling

- Extractor binary: `tools/build/plugin_doc_extract`
- Frontend: `tools/plugin_doc_gen.py` (`mr-plugin-doc-gen`)
- Preferred workspace command:
  - `./src/mavros/tools/gendoc.sh`
  - subcommands: `index`, `markdown`, default `all`
- If changing extractor/template/generator logic, regenerate affected outputs in:
  - `docs/plugins/std/index.json`
  - `docs/plugins/extras/index.json`
  - `docs/plugins/std/*.md`
  - `docs/plugins/extras/*.md`
- Prefer split commits for clarity:
  - commit 1: generator/template/extractor logic changes
  - commit 2: regenerated outputs
- Keep both commits in the same PR/branch so review and CI cover them together.

## Commit Scopes

- Format: `<component>: ...`
- `component` should usually be plugin name or a short scope like:
  - `mavros`, `extras`, `msgs`, `libmavconn`, `tools`, `docs`
