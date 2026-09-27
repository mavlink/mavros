#!/usr/bin/env bash
set -euo pipefail

# Run from colcon workspace root:
#   ./src/mavros/tools/gendoc.sh
#   ./src/mavros/tools/gendoc.sh index
#   ./src/mavros/tools/gendoc.sh markdown
# Optional env overrides:
#   WS_ROOT=/ws MAVROS_REPO=/ws/src/mavros CPP_BIN=/ws/src/mavros/tools/build/plugin_doc_extract
#   TEMPLATE=/ws/src/mavros/tools/templates/plugin.md.j2
#
# Requires clang-tooling (clang + libclang-dev) installed; the extractor parses
# each plugin with the clang AST using the compile_commands.json produced by the
# mavros build (with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON).

SUBCMD="${1:-all}"
if [[ "${SUBCMD}" != "all" && "${SUBCMD}" != "index" && "${SUBCMD}" != "markdown" ]]; then
  echo "Usage: $0 [all|index|markdown]"
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "${MAVROS_REPO:-}" ]]; then
  if [[ -n "${WS_ROOT:-}" && -d "${WS_ROOT}/src/mavros" ]]; then
    MAVROS_REPO="${WS_ROOT}/src/mavros"
  elif [[ -d "${PWD}/src/mavros" ]]; then
    MAVROS_REPO="${PWD}/src/mavros"
  else
    MAVROS_REPO="$(cd "${SCRIPT_DIR}/.." && pwd)"
  fi
fi

# Workspace root: the parent of src/ holding build/ and install/.
if [[ -z "${WS_ROOT:-}" ]]; then
  if [[ -d "$(dirname "$(dirname "${MAVROS_REPO}")")/build" ]]; then
    WS_ROOT="$(dirname "$(dirname "${MAVROS_REPO}")")"
  else
    WS_ROOT="$(dirname "${MAVROS_REPO}")"
  fi
fi

TOOLS_DIR="${MAVROS_REPO}/tools"
CPP_BIN="${CPP_BIN:-${TOOLS_DIR}/build/plugin_doc_extract}"
TEMPLATE="${TEMPLATE:-${TOOLS_DIR}/templates/plugin.md.j2}"
COMPILE_COMMANDS="${COMPILE_COMMANDS:-${WS_ROOT}/build/compile_commands.json}"
MAVROS_CC="${WS_ROOT}/build/mavros/compile_commands.json"
EXTRAS_CC="${WS_ROOT}/build/mavros_extras/compile_commands.json"

STD_INDEX="${MAVROS_REPO}/docs/plugins/std/index.json"
EXTRAS_INDEX="${MAVROS_REPO}/docs/plugins/extras/index.json"
STD_MD_DIR="${MAVROS_REPO}/docs/plugins/std"
EXTRAS_MD_DIR="${MAVROS_REPO}/docs/plugins/extras"
PLUGINS_INDEX="${MAVROS_REPO}/docs/plugins/index.md"
QOS_APPENDIX="${MAVROS_REPO}/docs/plugins/qos.md"

# Locate the LLVM root (for clang-tooling CMake config).
detect_llvm_root() {
  if [[ -n "${LLVM_ROOT:-}" ]]; then
    echo "${LLVM_ROOT}"
    return
  fi
  for d in /usr/lib/llvm-*; do
    if [[ -d "${d}/lib/cmake/llvm" ]]; then
      echo "${d}"
      return
    fi
  done
  echo ""
}

ensure_cpp_bin() {
  if [[ ! -x "${CPP_BIN}" ]]; then
    local llvm_root
    llvm_root="$(detect_llvm_root)"
    if [[ -z "${llvm_root}" ]]; then
      echo "ERROR: clang-tooling not found. Install clang + libclang-dev." >&2
      exit 1
    fi
    echo "Building extractor: ${CPP_BIN}"
    cmake -S "${TOOLS_DIR}" -B "${TOOLS_DIR}/build" -DLLVM_ROOT="${llvm_root}"
    cmake --build "${TOOLS_DIR}/build" -j"$(nproc)" --target plugin_doc_extract
    CPP_BIN="${TOOLS_DIR}/build/plugin_doc_extract"
  fi
}

# Return 0 when the compilation database exists and actually contains
# translation units for both mavros and mavros_extras plugin sources.
# A plain `colcon build` (without -DCMAKE_EXPORT_COMPILE_COMMANDS=ON) silently
# drops those entries, leaving a stale aggregate that would make the clang
# extractor fail to find include paths (fatal error: file not found).
compile_commands_complete() {
  [[ -f "${COMPILE_COMMANDS}" ]] || return 1
  python3 - "${COMPILE_COMMANDS}" <<'PY'
import json
import sys

try:
    entries = json.load(open(sys.argv[1]))
except (OSError, ValueError):
    sys.exit(1)

files = {entry.get("file", "") for entry in entries}
has_std = any("/mavros/src/plugins/" in f for f in files)
has_extras = any("/mavros_extras/src/plugins/" in f for f in files)
sys.exit(0 if has_std and has_extras else 1)
PY
}

# Ensure a merged compile_commands.json covering both mavros and mavros_extras
# plugin files exists (build both packages with the export flag).
ensure_compile_commands() {
  if compile_commands_complete; then
    return
  fi
  echo "Compilation database missing or incomplete: ${COMPILE_COMMANDS}"
  echo "Building mavros + mavros_extras with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..."
  # Run from the workspace root so colcon can discover sibling packages.
  (cd "${WS_ROOT}" && colcon build --packages-up-to mavros_extras \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
  MAVROS_CC="${MAVROS_CC}" EXTRAS_CC="${EXTRAS_CC}" \
    COMPILE_COMMANDS="${COMPILE_COMMANDS}" python3 - <<'PY'
import json, os, sys
cc = []
for f in (os.environ["MAVROS_CC"], os.environ["EXTRAS_CC"]):
    try:
        cc += json.load(open(f))
    except FileNotFoundError:
        print(f"WARNING: missing {f}", file=sys.stderr)
json.dump(cc, open(os.environ["COMPILE_COMMANDS"], "w"), indent=2)
PY
  if ! compile_commands_complete; then
    echo "ERROR: ${COMPILE_COMMANDS} still lacks mavros/mavros_extras plugin" >&2
    echo "       entries. Run: colcon build --packages-up-to mavros_extras" >&2
    echo "       --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON" >&2
    exit 1
  fi
}

run_extract() {
  ensure_compile_commands
  "${CPP_BIN}" \
    --compile-commands-dir="$(dirname "${COMPILE_COMMANDS}")" \
    --plugin-dir "${1}" \
    --output "${2}"
}

run_index() {
  ensure_cpp_bin
  ensure_compile_commands
  local cc_dir
  cc_dir="$(dirname "${COMPILE_COMMANDS}")"
  uv run mr-plugin-doc-gen \
    --compile-commands-dir "${cc_dir}" \
    --cpp-bin "${CPP_BIN}" \
    --format json \
    --plugin-dir "${MAVROS_REPO}/mavros/src/plugins" \
    --output "${STD_INDEX}"
  uv run mr-plugin-doc-gen \
    --compile-commands-dir "${cc_dir}" \
    --cpp-bin "${CPP_BIN}" \
    --format json \
    --plugin-dir "${MAVROS_REPO}/mavros_extras/src/plugins" \
    --output "${EXTRAS_INDEX}"
}

# Regenerate the markdown changelogs from each package's CHANGELOG.rst.
run_changelogs() {
  python3 "${TOOLS_DIR}/convert_changelogs.py"
}

run_markdown() {
  if [[ ! -f "${STD_INDEX}" || ! -f "${EXTRAS_INDEX}" ]]; then
    echo "Missing index files, run '$0 index' first (or '$0 all')."
    exit 1
  fi
  if [[ ! -f "${TEMPLATE}" ]]; then
    echo "Template not found: ${TEMPLATE}"
    exit 1
  fi

  uv run mr-plugin-doc-gen \
    --format markdown \
    --template "${TEMPLATE}" \
    --input-json "${STD_INDEX}" \
    --output-dir "${STD_MD_DIR}"

  uv run mr-plugin-doc-gen \
    --format markdown \
    --template "${TEMPLATE}" \
    --input-json "${EXTRAS_INDEX}" \
    --output-dir "${EXTRAS_MD_DIR}"

  uv run mr-plugin-doc-gen \
    --plugin-index "${PLUGINS_INDEX}" \
    --input-json "${STD_INDEX}" \
    --input-json "${EXTRAS_INDEX}"

  uv run mr-plugin-doc-gen \
    --qos-appendix "${QOS_APPENDIX}" \
    --input-json "${STD_INDEX}" \
    --input-json "${EXTRAS_INDEX}"
}

pushd "${TOOLS_DIR}" >/dev/null
uv sync --reinstall-package mavros-tools

case "${SUBCMD}" in
  index)
    run_index
    ;;
  markdown)
    run_changelogs
    run_markdown
    ;;
  all)
    run_index
    run_changelogs
    run_markdown
    ;;
esac
popd >/dev/null

echo "Done: ${SUBCMD}"
echo "Indexes:"
echo "  ${STD_INDEX}"
echo "  ${EXTRAS_INDEX}"
echo "Markdown dirs:"
echo "  ${STD_MD_DIR}"
echo "  ${EXTRAS_MD_DIR}"
echo "Plugin index:"
echo "  ${PLUGINS_INDEX}"
echo "QoS appendix:"
echo "  ${QOS_APPENDIX}"