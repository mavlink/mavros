# waypoint

- File: `mavros/src/plugins/waypoint.cpp`
- Class: `mavros::std_plugins::WaypointPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Mission manipulation plugin


Implements the
[MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html).

## Publishers
- `~/waypoints` [type: [mavros_msgs::msg::WaypointList](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/WaypointList.html), qos: [LatchedStateQoS](../qos.md#latchedstateqos "LatchedStateQoS QoS profile")] - The current mission waypoint list.
- `~/reached` [type: [mavros_msgs::msg::WaypointReached](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/WaypointReached.html), qos: [LatchedStateQoS](../qos.md#latchedstateqos "LatchedStateQoS QoS profile")] - Notifies when a mission waypoint is reached.

## Subscribers
- None

## Services
- `~/pull` [type: [mavros_msgs::srv::WaypointPull](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointPull.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Pull the mission from the FCU (MISSION_REQUEST_LIST).
- `~/push` [type: [mavros_msgs::srv::WaypointPush](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointPush.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Push a mission to the FCU (full or partial).
- `~/clear` [type: [mavros_msgs::srv::WaypointClear](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointClear.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Clear the mission on the FCU (MISSION_CLEAR_ALL).
- `~/set_current` [type: [mavros_msgs::srv::WaypointSetCurrent](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/WaypointSetCurrent.html), qos: [ServicesQoS](../qos.md#servicesqos "ServicesQoS QoS profile")] - Set the active/current mission waypoint.

## Clients
- None


## Parameters
- `pull_after_gcs` [type: bool, default: `true`] - Re-pull the mission automatically after a GCS pushes it.
- `use_mission_item_int` [type: bool, default: `true`] - Use MISSION_ITEM_INT instead of MISSION_ITEM when supported.
- `enable_partial_push` [type: integer, default: `2`] - Enable partial mission push (0/off, 1/on, 2/auto-detect).


## MAVLink Subscriptions
- [`MISSION_ACK`](https://mavlink.io/en/messages/common.html#MISSION_ACK) [handler: handle_mission_ack, dialect: common, msg_id: 47, id: `mavlink::common::msg::MISSION_ACK::MSG_ID`]
- [`MISSION_COUNT`](https://mavlink.io/en/messages/common.html#MISSION_COUNT) [handler: handle_mission_count, dialect: common, msg_id: 44, id: `mavlink::common::msg::MISSION_COUNT::MSG_ID`]
- [`MISSION_CURRENT`](https://mavlink.io/en/messages/common.html#MISSION_CURRENT) [handler: handle_mission_current, dialect: common, msg_id: 42, id: `mavlink::common::msg::MISSION_CURRENT::MSG_ID`]
- [`MISSION_ITEM`](https://mavlink.io/en/messages/common.html#MISSION_ITEM) [handler: handle_mission_item, dialect: common, msg_id: 39, id: `mavlink::common::msg::MISSION_ITEM::MSG_ID`]
- [`MISSION_ITEM_INT`](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT) [handler: handle_mission_item_int, dialect: common, msg_id: 73, id: `mavlink::common::msg::MISSION_ITEM_INT::MSG_ID`]
- [`MISSION_ITEM_REACHED`](https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED) [handler: handle_mission_item_reached, dialect: common, msg_id: 46, id: `mavlink::common::msg::MISSION_ITEM_REACHED::MSG_ID`]
- [`MISSION_REQUEST`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST) [handler: handle_mission_request, dialect: common, msg_id: 40, id: `mavlink::common::msg::MISSION_REQUEST::MSG_ID`]
- [`MISSION_REQUEST_INT`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT) [handler: handle_mission_request_int, dialect: common, msg_id: 51, id: `mavlink::common::msg::MISSION_REQUEST_INT::MSG_ID`]


## MAVLink Publications
- [`MISSION_ACK`](https://mavlink.io/en/messages/common.html#MISSION_ACK) [arg: `msg`, dialect: common, msg_id: 47, id: `mavlink::common::msg::MISSION_ACK::MSG_ID`]
- [`MISSION_CLEAR_ALL`](https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL) [arg: `msg`, dialect: common, msg_id: 45, id: `mavlink::common::msg::MISSION_CLEAR_ALL::MSG_ID`]
- [`MISSION_COUNT`](https://mavlink.io/en/messages/common.html#MISSION_COUNT) [arg: `msg`, dialect: common, msg_id: 44, id: `mavlink::common::msg::MISSION_COUNT::MSG_ID`]
- [`MISSION_ITEM`](https://mavlink.io/en/messages/common.html#MISSION_ITEM) [arg: `wpi`, dialect: common, msg_id: 39, id: `mavlink::common::msg::MISSION_ITEM::MSG_ID`]
- [`MISSION_ITEM_INT`](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT) [arg: `wpi`, dialect: common, msg_id: 73, id: `mavlink::common::msg::MISSION_ITEM_INT::MSG_ID`]
- [`MISSION_REQUEST`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST) [arg: `msg`, dialect: common, msg_id: 40, id: `mavlink::common::msg::MISSION_REQUEST::MSG_ID`]
- [`MISSION_REQUEST_INT`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT) [arg: `msg`, dialect: common, msg_id: 51, id: `mavlink::common::msg::MISSION_REQUEST_INT::MSG_ID`]
- [`MISSION_REQUEST_LIST`](https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST) [arg: `msg`, dialect: common, msg_id: 43, id: `mavlink::common::msg::MISSION_REQUEST_LIST::MSG_ID`]
- [`MISSION_SET_CURRENT`](https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT) [arg: `msg`, dialect: common, msg_id: 41, id: `mavlink::common::msg::MISSION_SET_CURRENT::MSG_ID`]
- [`MISSION_WRITE_PARTIAL_LIST`](https://mavlink.io/en/messages/common.html#MISSION_WRITE_PARTIAL_LIST) [arg: `msg`, dialect: common, msg_id: 38, id: `mavlink::common::msg::MISSION_WRITE_PARTIAL_LIST::MSG_ID`]
