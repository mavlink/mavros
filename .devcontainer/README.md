# Devcontainer Modes

## Default (`.devcontainer/devcontainer.json`)
- Scope-limited mode.
- Mounts only this repository to `/ws/src/mavros`.

## Whole Workspace (`.devcontainer/whole-workspace/devcontainer.json`)
- Mounts `${localWorkspaceFolder}/../..` to `/ws`.
- Expects this repository to be located at `src/mavros` in that workspace.
- In other words, host layout should look like:
  - `<workspace>/src/mavros`
