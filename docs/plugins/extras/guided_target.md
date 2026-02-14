# guided_target

- File: `mavros_extras/src/plugins/guided_target.cpp`
- Class: `mavros::extra_plugins::GuidedTargetPlugin`
- Namespace: `guided_target`
- Brief: guided target plugin


Send and receive setpoint positions from FCU controller.

## Publishers
- `/move_base_simple/goal` (geometry_msgs::msg::PoseStamped) - Publish targets received from FCU


## Subscribers
- `global_position/gp_origin` (geographic_msgs::msg::GeoPointStamped) - Subscriber for global origin (aka map origin).


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`] - frame params:


## MAVLink Subscriptions
- `POSITION_TARGET_GLOBAL_INT` [handler: handle_position_target_global_int, dialect: common, msg_id: 87, id: `mavlink::common::msg::POSITION_TARGET_GLOBAL_INT::MSG_ID`]


## MAVLink Publications
- None
