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
- Never print, echo, export, or otherwise output the value of any token/secret environment variable
  (e.g. `*_TOKEN`, `*_SECRET`, `*_KEY`, `*_USERNAME`, `*_PASSWORD`, `GH_TOKEN`, `GITHUB_*`, `GITLAB_*`).
  Reference them only by variable name (e.g. `"$GITHUB_MCP_RO_TOKEN"`) so the value is
  never captured in tool output, logs, or transcripts. Prefer MCP/tooling that handles auth internally
  over shelling out with an explicit secret value.

## Repository Scope

- ROS packages (each has `package.xml`): `mavros`, `mavros_extras`, `mavros_msgs`, `libmavconn`, `mavros_examples`, `test_mavros`.
- `tools/` is NOT a colcon package — it is a standalone `uv`-managed tool dir (`uv tool install ./tools` provides `mr-cog`, `mr-plugin-doc-gen`).
- Respect package boundaries:
  - runtime/plugins: `mavros`, `mavros_extras`
  - interfaces: `mavros_msgs`
  - transport/link: `libmavconn`
  - CI covers packages: `libmavconn`, `mavros`, `mavros_extras`

## Build And Test

- Run build/test commands from workspace root (`/ws` in devcontainer / podman container), not from `/ws/src/mavros`.
- Always `cd /ws` before `colcon` so it finds `install/` and `build/` correctly.
- Build (from `/ws`):
  - `colcon build --packages-up-to mavros mavros_extras mavros_msgs`
- Do NOT use `--base-paths /ws/src/mavros` — it breaks dependency resolution
  (mavlink/libmavconn packages live in sibling dirs under `/ws/src/`).
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

## Local Container (Podman)

As an alternative to the devcontainer, use a plain ROS container with full-workspace mount.

### Setup

Container creation (one-time):

    podman run -it --name ros2-<distro> \
      -v <workspace>:/ws \
      -v $HOME/.gitconfig:/root/.gitconfig \
      -v $HOME/.ssh:/root/.ssh \
      -v $HOME/.gnupg:/root/.gnupg \
      -v $HOME/.config:/root/.config \
      -p 14540:14540/udp -p 14545:14545/udp -p 5761:5761 \
      ros:<distro>

Restart after host reboot:

    podman start ros2-<distro>

First-run dependencies (inside container):

    apt-get update && rosdep update && rosdep install -y -i --from-paths /ws/src
    ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

Remove system mavlink if building from source (avoids duplicate-package error):

    apt-get remove -y ros-<distro>-mavlink

### Build and Test

Run commands on the host via `podman exec <name> bash -lc '...'`.
Prepend `source /opt/ros/<distro>/setup.bash` before every command.
Always `cd /ws` before `colcon` so it finds `install/` and `build/` correctly.

Build:

    podman exec ros2-<distro> bash -lc 'source /opt/ros/<distro>/setup.bash && cd /ws && colcon build'

If extra source packages cause duplicate-package errors, scope discovery:

    colcon build --packages-up-to mavros

Test (after source of install; `install/setup.bash` includes the system setup):

    podman exec ros2-<distro> bash -lc '\
      source /ws/install/setup.bash \
      && cd /ws/build/mavros && ctest --output-on-failure'

`ctest` from the build directory is preferred over `colcon test` — it avoids workspace-scanning issues.

To reformat with uncrustify:

    podman exec ros2-<distro> bash -lc '\
      source /opt/ros/<distro>/setup.bash \
      && ament_uncrustify --reformat /ws/src/mavros/mavros'

## Style And Compatibility

- C++ style is enforced by `ament_uncrustify` (latest/current supported ROS 2 baseline).
- Python style is enforced with `ruff`, while staying compatible with ROS 2 conventions used here.
- Avoid style-only edits outside touched scope.
- Keep compatibility across supported ROS 2 distros; prefer conditional handling over distro-specific breakage.
- CI matrix runs `humble`, `jazzy`, `kilted`, `lyrical`, `rolling`; build against the oldest to catch regressions.
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
