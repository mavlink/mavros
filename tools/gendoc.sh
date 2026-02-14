#!/usr/bin/env bash
set -euo pipefail

# Run from colcon workspace root:
#   ./src/mavros/tools/gendoc.sh
#   ./src/mavros/tools/gendoc.sh index
#   ./src/mavros/tools/gendoc.sh markdown
# Optional env overrides:
#   WS_ROOT=/ws MAVROS_REPO=/ws/src/mavros CPP_BIN=/ws/src/mavros/tools/build/plugin_doc_gen_cpp
#   TEMPLATE=/ws/src/mavros/tools/templates/plugin.md.j2

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

TOOLS_DIR="${MAVROS_REPO}/tools"
CPP_BIN="${CPP_BIN:-${TOOLS_DIR}/build/plugin_doc_gen_cpp}"
TEMPLATE="${TEMPLATE:-${TOOLS_DIR}/templates/plugin.md.j2}"

STD_INDEX="${MAVROS_REPO}/docs/plugins/std/index.json"
EXTRAS_INDEX="${MAVROS_REPO}/docs/plugins/extras/index.json"
STD_MD_DIR="${MAVROS_REPO}/docs/plugins/std"
EXTRAS_MD_DIR="${MAVROS_REPO}/docs/plugins/extras"

ensure_cpp_bin() {
  if [[ ! -x "${CPP_BIN}" ]]; then
    echo "Building extractor: ${CPP_BIN}"
    cmake -S "${TOOLS_DIR}" -B "${TOOLS_DIR}/build"
    cmake --build "${TOOLS_DIR}/build" -j"$(nproc)"
  fi
}

run_index() {
  uv run mr-plugin-doc-gen \
    --cpp-bin "${CPP_BIN}" \
    --format json \
    --plugin-dir "${MAVROS_REPO}/mavros/src/plugins" \
    --output "${STD_INDEX}"

  uv run mr-plugin-doc-gen \
    --cpp-bin "${CPP_BIN}" \
    --format json \
    --plugin-dir "${MAVROS_REPO}/mavros_extras/src/plugins" \
    --output "${EXTRAS_INDEX}"
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
}

pushd "${TOOLS_DIR}" >/dev/null
uv sync --reinstall-package mavros-tools
ensure_cpp_bin

case "${SUBCMD}" in
  index)
    run_index
    ;;
  markdown)
    run_markdown
    ;;
  all)
    run_index
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
