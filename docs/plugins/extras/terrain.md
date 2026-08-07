# terrain

- File: `mavros_extras/src/plugins/terrain.cpp`
- Class: `mavros::extra_plugins::TerrainPlugin`
- Namespace: `terrain`
- Brief: Terrain plugin.


Bridges the MAVLink terrain protocol between the FCU and a companion terrain_server node that serves SRTM elevation data. Protocol spec: https://mavlink.io/en/services/terrain.html TERRAIN_REQUEST from the FCU is forwarded to the server for SRTM lookup; the server responds with TERRAIN_DATA blocks that are sent back to the FCU.  TERRAIN_CHECK point queries are handled via a service call to the server, which responds with elevation data that is returned to the FCU as TERRAIN_REPORT.

## Publishers
- `~/report` ([mavros_msgs::msg::TerrainReport](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/TerrainReport.html)) - Terrain height reports from FCU and check-service responses Publish terrain reports from MAVLink TERRAIN_REPORT.
- `~/request` ([mavros_msgs::msg::TerrainRequest](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/TerrainRequest.html)) - Grid data requests forwarded from FCU for SRTM lookup Publish terrain requests from MAVLink TERRAIN_REQUEST.


## Subscribers
- `~/data` ([mavros_msgs::msg::TerrainData](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/TerrainData.html)) - Filled terrain grid blocks from terrain_server Subscribe to TerrainData to send as TERRAIN_DATA to the FCU.


## Services
- None


## Clients
- `~/check` ([mavros_msgs::srv::TerrainCheck](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/TerrainCheck.html)) - Point elevation query handled by terrain_server Client to query terrain elevation (terrain/check).


## Parameters
- None


## MAVLink Subscriptions
- [`TERRAIN_REPORT`](https://mavlink.io/en/messages/common.html#TERRAIN_REPORT) [handler: handle_terrain_report, dialect: common, msg_id: 136, id: `mavlink::common::msg::TERRAIN_REPORT::MSG_ID`]
- [`TERRAIN_REQUEST`](https://mavlink.io/en/messages/common.html#TERRAIN_REQUEST) [handler: handle_terrain_request, dialect: common, msg_id: 133, id: `mavlink::common::msg::TERRAIN_REQUEST::MSG_ID`]
- [`TERRAIN_CHECK`](https://mavlink.io/en/messages/common.html#TERRAIN_CHECK) [handler: handle_terrain_check, dialect: common, msg_id: 135, id: `mavlink::common::msg::TERRAIN_CHECK::MSG_ID`]


## MAVLink Publications
- [`TERRAIN_REPORT`](https://mavlink.io/en/messages/common.html#TERRAIN_REPORT) [arg: `rpt`, dialect: common, msg_id: 136, id: `mavlink::common::msg::TERRAIN_REPORT::MSG_ID`]
- [`TERRAIN_DATA`](https://mavlink.io/en/messages/common.html#TERRAIN_DATA) [arg: `td`, dialect: common, msg_id: 134, id: `mavlink::common::msg::TERRAIN_DATA::MSG_ID`]
