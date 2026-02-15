# setpoint_position

- File: `mavros/src/plugins/setpoint_position.cpp`
- Class: `mavros::std_plugins::SetpointPositionPlugin`
- Namespace: `setpoint_position`
- Brief: Setpoint position plugin


Send setpoint positions to FCU controller.

## Publishers
- None


## Subscribers
- `~/local` (geometry_msgs::msg::PoseStamped)
- `~/global` (geographic_msgs::msg::GeoPoseStamped)
- `~/global_to_local` (geographic_msgs::msg::GeoPoseStamped)
- `global_position/global` (sensor_msgs::msg::NavSatFix)
- `local_position/pose` (geometry_msgs::msg::PoseStamped)


## Services
- None


## Clients
- None


## Parameters
- `mav_frame` [default: `"LOCAL_NED"`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- `SET_POSITION_TARGET_LOCAL_NED` [arg: `sp`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
- `SET_POSITION_TARGET_GLOBAL_INT` [arg: `sp`, dialect: common, msg_id: 86, id: `mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT::MSG_ID`]
