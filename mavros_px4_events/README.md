# mavros_px4_events

Optional PX4 EVENT decoding for MAVROS. The `px4_events` plugin translates
firmware event IDs and arguments into readable log messages using libevents.

Core `sys_status` still publishes raw `status_event` messages and logs raw EVENT
messages. This plugin adds decoded logging; it does not replace that publication
or suppress the raw logs. Unknown events remain visible through core logging.

## Build and load

From a ROS workspace containing this repository, initialize the pinned libevents
submodule before building. libevents is built from the submodule under
`vendor/libevents`; XZ decompression uses system liblzma (`liblzma-dev`).

```sh
cd /ws/src/mavros
git submodule update --init --recursive mavros_px4_events/vendor/libevents
source /opt/ros/<distro>/setup.bash
cd /ws
colcon build --packages-up-to mavros_px4_events
source install/setup.bash
```

MAVROS discovers the installed plugin through pluginlib. If you use a plugin
allowlist, add `px4_events` to it. Automatic mode also needs `command` and `ftp`;
keep `sys_status` enabled for normal connection tracking and raw event output.
An empty allowlist permits all installed plugins, unless denied explicitly.

Start MAVROS with your usual FCU URL and PX4 configuration, for example:

```sh
ros2 run mavros mavros_node --ros-args \
  --params-file "$(ros2 pkg prefix mavros)/share/mavros/launch/px4_config.yaml" \
  -p 'fcu_url:=udp://:14540@'
```

Building core or extras with (for example):
`--packages-up-to mavros mavros_extras` also does not select this package.

## Configuration

Parameters belonging to the plugin: 
| Parameter | Default | Meaning |
|---|---|---|
| `event_decoding_mode` | `auto` | `auto`: acquire definitions from a connected PX4 autopilot; `local`: load a local file; `disabled`: no decoding or metadata acquisition. |
| `event_metadata_file` | `""` | Firmware-matching JSON or JSON.xz definitions; valid only in `local` mode. |
| `event_metadata_download_delay` | `30.0` | Seconds after FCU connection before FTP downloads are allowed. Must be finite and nonnegative. Cache lookup can complete before this delay. |
| `event_metadata_cache_enabled` | `true` | Enable persistent metadata reuse in automatic mode. |
| `event_metadata_cache_dir` | `""` | Empty selects `$HOME/.cache/mavros/px4_events`; otherwise use this directory. |
| `event_metadata_cache_max_bytes` | `16777216` (16 MiB) | Best-effort finalized cache size budget. Nonpositive values disable persistence with a diagnostic; automatic decoding continues. |

For local mode, create a parameter file such as `/tmp/px4-events-local.yaml`:

```yaml
/**/px4_events:
  ros__parameters:
    event_decoding_mode: local
    event_metadata_file: /absolute/path/to/all_events.json.xz
```

Add `--params-file /tmp/px4-events-local.yaml` to the command above. Use event
definitions from the exact firmware being tested. Local mode does not discover
or download metadata and does not use the persistent cache.

## Automatic acquisition and cache

Automatic mode checks for PX4, discovers COMPONENT_METADATA (with legacy
COMPONENT_INFORMATION fallback), and acquires general metadata followed by event
definitions through MAVROS `ftp/open`, `ftp/read`, and `ftp/close` services.
Discovery uses `cmd/command`. These clients live in helper classes and are not
currently included in the generated plugin reference's client list.

Definitions are validated before use or persistence. A complete cache hit enables
decoding without FTP; a partial hit downloads only the missing file. Corrupt
entries are rejected. An unwritable cache directory or failed save does not
prevent validated definitions from being used in memory. Disconnect cancels
acquisition and clears automatically acquired definitions; reconnect starts a
new discovery. Local definitions remain loaded across reconnects.

The cache stores validated original file bytes, keyed by metadata role, advertised
URI and checksum. Eviction is best-effort, based on save time. In a container,
mount the configured cache directory on persistent storage to retain entries
across container replacement. Set `event_metadata_cache_enabled: false` to avoid
persistent reads and writes. To clear it, stop MAVROS and remove the contents of
the configured cache directory.

Each metadata file is limited to 1 MiB of transferred input and 8 MiB of decoded
JSON. liblzma has a separate 128 MiB decoder memory limit: the tested PX4 metadata
advertises a 64 MiB dictionary even though its output is much smaller. This memory
limit is separate from the output cap. Acquisition or decoding failures leave
core raw logging available.

## Check operation


Look for `PX4 event metadata ready; decoded logging enabled`, followed by readable
messages from `mavros.px4_events`. To inspect discovery, cache hits and events at
DEBUG severity without enabling every plugin's debug logs, append:

```sh
--log-level info --log-level mavros.px4_events:=debug
```

The logger prefix follows the MAVROS namespace. Decoded messages preserve event
severity; internal summary events can therefore appear only at DEBUG level.
