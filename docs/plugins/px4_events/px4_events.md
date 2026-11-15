# px4_events

- File: `mavros_px4_events/src/plugins/px4_events.cpp`
- Class: `mavros::px4_events::PX4EventsPlugin`
- Namespace: `mavros::px4_events`
- Brief: Optional PX4 event decoding plugin.


Decode PX4 EVENT messages using local or automatically acquired metadata.
Automatic acquisition reuses validated cached files before downloading over FTP.
Core sys_status raw event logging and publication remain unchanged.


See [package usage and configuration](../../../mavros_px4_events/README.md).

## Publishers
- None

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- `event_decoding_mode` [type: string, default: `"auto"`] - PX4 event logging: auto (default), local, or disabled; changes require restart.
- `event_metadata_file` [type: string, default: `""`] - Firmware-matching local JSON or JSON.xz definitions; valid only in local mode.
- `event_metadata_download_delay` [type: double, default: `30.0`] - Automatic metadata download delay from FCU connection, in seconds.
- `event_metadata_cache_enabled` [type: bool, default: `true`] - Reuse validated persistent definitions in automatic mode.
- `event_metadata_cache_dir` [type: string, default: `""`] - Empty selects $HOME/.cache/mavros/px4_events; otherwise use this directory.
- `event_metadata_cache_max_bytes` [type: integer, default: `MetadataCache::DEFAULT_MAX_BYTES`] - Best-effort finalized cache size limit; nonpositive values disable persistence.


## MAVLink Subscriptions
- [`EVENT`](https://mavlink.io/en/messages/common.html#EVENT) [handler: handle_event, dialect: common, msg_id: 410, id: `mavlink::common::msg::EVENT::MSG_ID`]
- [`COMPONENT_METADATA`](https://mavlink.io/en/messages/common.html#COMPONENT_METADATA) [handler: handle_component_metadata, dialect: common, msg_id: 397, id: `mavlink::common::msg::COMPONENT_METADATA::MSG_ID`]
- [`COMPONENT_INFORMATION`](https://mavlink.io/en/messages/common.html#COMPONENT_INFORMATION) [handler: handle_component_information, dialect: common, msg_id: 395, id: `mavlink::common::msg::COMPONENT_INFORMATION::MSG_ID`]


## MAVLink Publications
- None
