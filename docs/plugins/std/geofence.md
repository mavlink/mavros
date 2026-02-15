# geofence

- File: `mavros/src/plugins/geofence.cpp`
- Class: `mavros::std_plugins::GeofencePlugin`
- Namespace: `geofence`
- Brief: Geofence manipulation plugin


## Publishers
- `~/fences` ([mavros_msgs::msg::WaypointList](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/WaypointList.html))


## Subscribers
- None


## Services
- `~/pull` ([mavros_msgs::srv::WaypointPull](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointPull.html))
- `~/push` ([mavros_msgs::srv::WaypointPush](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointPush.html))
- `~/clear` ([mavros_msgs::srv::WaypointClear](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointClear.html))


## Clients
- None


## Parameters
- `pull_after_gcs` [type: bool, default: `true`] - NOTE(vooon): I'm not quite sure that this option would work with mavros router
- `use_mission_item_int` [type: bool, default: `true`]


## MAVLink Subscriptions
- [`MISSION_ITEM`](https://mavlink.io/en/messages/common.html#MISSION_ITEM) [handler: handle_mission_item, dialect: common, msg_id: 39, id: `mavlink::common::msg::MISSION_ITEM::MSG_ID`]
- [`MISSION_ITEM_INT`](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT) [handler: handle_mission_item_int, dialect: common, msg_id: 73, id: `mavlink::common::msg::MISSION_ITEM_INT::MSG_ID`]
- [`MISSION_REQUEST`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST) [handler: handle_mission_request, dialect: common, msg_id: 40, id: `mavlink::common::msg::MISSION_REQUEST::MSG_ID`]
- [`MISSION_REQUEST_INT`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT) [handler: handle_mission_request_int, dialect: common, msg_id: 51, id: `mavlink::common::msg::MISSION_REQUEST_INT::MSG_ID`]
- [`MISSION_COUNT`](https://mavlink.io/en/messages/common.html#MISSION_COUNT) [handler: handle_mission_count, dialect: common, msg_id: 44, id: `mavlink::common::msg::MISSION_COUNT::MSG_ID`]
- [`MISSION_ACK`](https://mavlink.io/en/messages/common.html#MISSION_ACK) [handler: handle_mission_ack, dialect: common, msg_id: 47, id: `mavlink::common::msg::MISSION_ACK::MSG_ID`]


## MAVLink Publications
- [`MISSION_ITEM`](https://mavlink.io/en/messages/common.html#MISSION_ITEM) [arg: `wpi`, dialect: common, msg_id: 39, id: `mavlink::common::msg::MISSION_ITEM::MSG_ID`]
- [`MISSION_ITEM_INT`](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT) [arg: `wpi`, dialect: common, msg_id: 73, id: `mavlink::common::msg::MISSION_ITEM_INT::MSG_ID`]
- [`MISSION_REQUEST`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST) [arg: `mrq`, dialect: common, msg_id: 40, id: `mavlink::common::msg::MISSION_REQUEST::MSG_ID`]
- [`MISSION_REQUEST_INT`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT) [arg: `mrq`, dialect: common, msg_id: 51, id: `mavlink::common::msg::MISSION_REQUEST_INT::MSG_ID`]
- [`MISSION_SET_CURRENT`](https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT) [arg: `msc`, dialect: common, msg_id: 41, id: `mavlink::common::msg::MISSION_SET_CURRENT::MSG_ID`] - msc.mission_type = enum_value(mission_type);
- [`MISSION_REQUEST_LIST`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST) [arg: `mrl`, dialect: common, msg_id: 43, id: `mavlink::common::msg::MISSION_REQUEST_LIST::MSG_ID`]
- [`MISSION_COUNT`](https://mavlink.io/en/messages/common.html#MISSION_COUNT) [arg: `mcnt`, dialect: common, msg_id: 44, id: `mavlink::common::msg::MISSION_COUNT::MSG_ID`]
- [`MISSION_WRITE_PARTIAL_LIST`](https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST) [arg: `mwpl`, dialect: common, msg_id: 38, id: `mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST::MSG_ID`]
- [`MISSION_CLEAR_ALL`](https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL) [arg: `mclr`, dialect: common, msg_id: 45, id: `mavlink::common::msg::MISSION_CLEAR_ALL::MSG_ID`]
- [`MISSION_ACK`](https://mavlink.io/en/messages/common.html#MISSION_ACK) [arg: `mack`, dialect: common, msg_id: 47, id: `mavlink::common::msg::MISSION_ACK::MSG_ID`]
