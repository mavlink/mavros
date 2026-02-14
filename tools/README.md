MAVROS Tools
============

Some common development scripts.

```shell
uv tool install ./
```

Fast C++ plugin API extractor:

```shell
# requires yaml-cpp development package in build environment
cmake -S tools -B tools/build
cmake --build tools/build -j
tools/build/plugin_doc_gen_cpp --jobs 6 --output /tmp/mavros_plugin_api.json
```

Generate docs from workspace root (after `colcon build`):

```shell
./src/mavros/tools/gendoc.sh
./src/mavros/tools/gendoc.sh index
./src/mavros/tools/gendoc.sh markdown
```

Generate one markdown file per plugin with Jinja2 template:

```shell
cd tools
uv run mr-plugin-doc-gen --format markdown --input-json ../docs/plugins/std/index.json --output-dir /tmp/std-md
```
