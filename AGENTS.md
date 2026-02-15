# AGENTS.md

Guidance for coding agents working in this repository.

## Scope

- Repository root for `mavros` monorepo (`mavros`, `mavros_extras`, `mavros_msgs`, `libmavconn`, `tools`).
- Prefer minimal, targeted changes. Do not reformat unrelated files.

## Build And Test

- Typical environment is ROS 2 workspace (often in container `ros2-kilted`).
- Before running build/test commands, source ROS environment:
  - `source /opt/ros/<distro>/setup.bash`
  - and if present: `source <ws>/install/setup.bash`
- Build with colcon from workspace root:
  - `colcon build --packages-up-to mavros mavros_extras mavros_msgs`
- `colcon test` is a CI merge gate and must pass before merge.
- Local runs on every iteration are not required; run tests when risk/impact justifies it.
- Recommended local commands (at minimum for touched packages):
  - `colcon test --packages-select mavros mavros_extras mavros_msgs`
  - `colcon test-result --verbose`
- For tool-only C++ extractor changes:
  - `cmake -S tools -B tools/build`
  - `cmake --build tools/build -j$(nproc)`

## ROS 2 Codebase Guidelines

- Follow ROS 2 package boundaries:
  - Runtime/plugin code in `mavros` and `mavros_extras`
  - Interface definitions in `mavros_msgs`
  - Transport/link logic in `libmavconn`
- Keep plugin API behavior stable. Treat topic/service/parameter names as compatibility surface.
- Prefer existing helpers and wrappers in this repo (`plugin.hpp`, `plugin_filter.hpp`, UAS helpers) over ad-hoc node logic.
- For parameters:
  - Use declaration/watch helpers already used in plugins.
  - Preserve default values and parameter names unless change is explicitly requested.
- For ROS interfaces:
  - Reuse existing `mavros_msgs` types where possible.
  - If adding/changing `.msg/.srv/.action`, update all callers and docs, and regenerate as required by build.
- For QoS/topic changes:
  - Keep QoS explicit and conservative; avoid silent behavior changes.
  - Validate remap/namespace behavior in plugin context.
- For logging:
  - Use ROS logging macros consistently (`RCLCPP_DEBUG/INFO/WARN/ERROR`).
  - Avoid noisy logs on hot paths.
- For cross-distro compatibility:
  - Keep CMake/package logic compatible with older ROS 2 distros where this repo supports them.
  - Prefer feature detection and conditional dependency handling over distro-specific breakage.

## C++ Style Expectations

- Code style is enforced with `ament_uncrustify`.
- Treat the current/latest supported ROS 2 release as the formatting baseline.
- Expect minor formatting differences on older distros; avoid churn-only edits to satisfy old-distro style drift.
- Match surrounding style and include ordering in each package.
- Keep headers minimal and avoid unnecessary template/metaprogramming complexity.
- Favor explicit types and clear control flow over clever compact code.
- Add comments only where behavior is non-obvious (protocol constraints, MAVLink quirks, ROS integration details).

## Python Style Expectations

- Use `ruff` for linting/formatting of Python code in this repository.
- Keep Python changes compatible with ROS 2 ecosystem conventions used in this codebase (packaging, entrypoints, logging style, dependency expectations).
- Prefer clear, typed code and avoid introducing style-only churn in unrelated files.

## Plugin Docs Pipeline

- Main extraction binary: `tools/build/plugin_doc_extract`
- Python frontend: `tools/plugin_doc_gen.py` (entrypoint `mr-plugin-doc-gen`)
- Workspace helper script (run from workspace root):
  - `./src/mavros/tools/gendoc.sh` (all)
  - `./src/mavros/tools/gendoc.sh index`
  - `./src/mavros/tools/gendoc.sh markdown`
- Script uses `uv sync`/`uv run`; prefer it over ad-hoc command variants.

## Generated Files

- Index JSON outputs:
  - `docs/plugins/std/index.json`
  - `docs/plugins/extras/index.json`
- Per-plugin markdown outputs:
  - `docs/plugins/std/*.md`
  - `docs/plugins/extras/*.md`
- If extractor/template logic changes, regenerate affected outputs.

## Cog-Generated Code

- This repository uses `cog` blocks (`[[[cog: ... ]]] ... [[[end]]]`) in CMake, XML, and C++ sources.
- Many blocks include checksum markers on the end tag, e.g. `[[[end]]] (sum: ...)`.
- Do not hand-edit generated text inside cog output regions; edit the generator snippet and re-run cog.
- Regenerate:
  - Single file: `mr-cog -cr <file>`
  - All files: `./tools/cogall.sh`
- Commit updated generated content and checksum markers together with generator changes.
- If a changed cog block no longer matches checksum, first remove the checksum suffix from its `[[[end]]]` marker, then run cog regeneration.
- Some blocks may not keep checksum parity after `ament_uncrustify`; this is acceptable in this repo.
- If needed, regenerate with cog and then apply `ament_uncrustify`, but prefer to avoid churn-only checksum/style updates.
- During `./tools/cogall.sh`, the message from `mavros/src/lib/enum_sensor_orientation.cpp` about `Parse Error ... desc: Custom orientation` is expected and non-fatal.

## Extractor Notes

- `plugin_doc_extract.cpp` uses explicit allowlists for special mixin/base context extraction.
- Keep mission and setpoint special handling conservative; avoid broad heuristics that impact unrelated plugins.
- Prefer deterministic output and human-readable JSON formatting.

## Editing Expectations

- Preserve existing style and naming conventions.
- Do not introduce destructive git operations.
- If unexpected repository changes appear, stop and ask before proceeding.

## PR Checklist

- Build succeeds for affected packages.
- CI-gated tests pass (`colcon test` in CI).
- Regenerated artifacts are updated when generators/templates change.
- No unrelated formatting or file churn in the diff.

## Change Scope And Regeneration

- If changing code generators/templates/extractors, regenerate only affected outputs.
- In commit/PR notes, state which generated files were refreshed and why.
- Keep generator changes and regenerated output in the same change set.

## Container Command Convention

- Prefer running build/test in the project container environment when available.
- Typical sequence:
  - `source /opt/ros/<distro>/setup.bash`
  - `source <ws>/install/setup.bash` (if exists)
  - run `colcon build` / `colcon test` from workspace root

## Compatibility Policy

- For new dependencies or APIs, confirm compatibility with supported ROS 2 distros.
- Prefer conditional handling over distro-specific breakage.

## Performance Guardrails

- For tooling/scripts, avoid full-repo scans when package/file-scoped work is sufficient.
- Prefer incremental/regional regeneration where practical.

## Logging Policy

- Keep default logs concise; avoid high-volume logs in hot paths.
- Use existing ROS log levels and plugin parameters for verbosity control.

## Failure Triage

- Distinguish environment/setup failures from code regressions.
- Report first failing test target and log path when tests fail.

## Commit Prefixes

- Use commit subject format: `<component>: ...`
- `component` should usually be:
  - plugin name (when change is plugin-scoped), or
  - short package/scope name (e.g. `mavros`, `extras`, `msgs`, `libmavconn`, `tools`, `docs`).
- Prefer short scopes to reduce `mavros_*` prefix noise in commit history.
