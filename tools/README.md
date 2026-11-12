MAVROS Tools
============

Some common development scripts.

## Setup

```shell
uv tool install ./
```

The C++ plugin API extractor is built with **clang-tooling**, so install clang
and the LLVM development packages first:

```shell
# Debian/Ubuntu
sudo apt install clang libclang-dev
```

The changelog converter uses the **pandoc** binary (install it with
`apt install pandoc`) to convert `CHANGELOG.rst` to markdown.

## C++ plugin API extractor

```shell
# requires clang-tooling (see apt install above) and yaml-cpp
cmake -S tools -B tools/build -DLLVM_ROOT=/usr/lib/llvm-21
cmake --build tools/build -j
tools/build/plugin_doc_extract \
  --compile-commands-dir /ws/build \
  --plugin-dir mavros/src/plugins \
  --output /tmp/mavros_plugin_api.json
```

## Generate docs

The generator covers `mavros`, `mavros_extras`, and `mavros_px4_events`, writing
separate `std`, `extras`, and `px4_events` groups under `docs/plugins/`. Initialize
the libevents submodule as described in the PX4 events package README first.

From the workspace root, with the ROS environment sourced:

```shell
colcon build --packages-up-to mavros_extras mavros_px4_events \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Then generate the documentation:

```shell
./src/mavros/tools/gendoc.sh
./src/mavros/tools/gendoc.sh index
./src/mavros/tools/gendoc.sh markdown
```

`gendoc.sh` builds the extractor if missing, (re)builds the compile database if incomplete,
extracts the plugin API, regenerates the per-package changelogs from the RST
sources, and renders the plugin pages, index, QoS appendix and changelog pages.

## Render plugin markdown manually

```shell
cd tools
uv run mr-plugin-doc-gen --format markdown --input-json ../docs/plugins/std/index.json --output-dir /tmp/std-md
```
