# guided_target

- File: `mavros_extras/src/plugins/guided_target.cpp`
- Class: `mavros::extra_plugins::GuidedTargetPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: guided target plugin


Send and receive setpoint positions from FCU controller.

## Publishers
- `/move_base_simple/goal` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish guided target from MAVLink POSITION_TARGET_GLOBAL_INT.

## Subscribers
- `global_position/gp_origin` [type: [geographic_msgs::msg::GeoPointStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPointStamped.html), qos: [LatchedStateQoS](../qos.md#latchedstateqos "LatchedStateQoS QoS profile")] - Subscribe to global position origin (map origin).

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"map"`] - frame params: Frame id used for the published target.


## MAVLink Subscriptions
- [`POSITION_TARGET_GLOBAL_INT`](https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT) [handler: handle_position_target_global_int, dialect: common, msg_id: 87, id: `mavlink::common::msg::POSITION_TARGET_GLOBAL_INT::MSG_ID`]


## MAVLink Publications
- None
