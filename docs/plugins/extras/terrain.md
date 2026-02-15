# terrain

- File: `mavros_extras/src/plugins/terrain.cpp`
- Class: `mavros::extra_plugins::TerrainPlugin`
- Namespace: `terrain`
- Brief: Terrain height plugin.


This plugin allows publishing of terrain height estimate from FCU to ROS.

## Publishers
- `~/report` ([mavros_msgs::msg::TerrainReport](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/TerrainReport.html))


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`TERRAIN_REPORT`](https://mavlink.io/en/messages/common.html#TERRAIN_REPORT) [handler: handle_terrain_report, dialect: common, msg_id: 136, id: `mavlink::common::msg::TERRAIN_REPORT::MSG_ID`]


## MAVLink Publications
- None
