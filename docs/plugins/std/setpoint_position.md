# setpoint_position

- File: `mavros/src/plugins/setpoint_position.cpp`
- Class: `mavros::std_plugins::SetpointPositionPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Setpoint position plugin


Send setpoint positions to FCU controller. Uses the
[MAVLink Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html).

## Publishers
- None

## Subscribers
- `~/local` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local position setpoint (SET_POSITION_TARGET_LOCAL_NED).
- `~/global` [type: [geographic_msgs::msg::GeoPoseStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Global position setpoint (SET_POSITION_TARGET_GLOBAL_INT).
- `~/global_to_local` [type: [geographic_msgs::msg::GeoPoseStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Global setpoint converted to local setpoint (SET_POSITION_TARGET_LOCAL_NED).
- `global_position/global` [type: [sensor_msgs::msg::NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Current global position used for conversion.
- `local_position/pose` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Current local position used for conversion.

## Services
- None

## Clients
- None


## Parameters
- `mav_frame` [type: string, default: `"LOCAL_NED"`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_POSITION_TARGET_GLOBAL_INT`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT) [arg: `msg`, dialect: common, msg_id: 86, id: `mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT::MSG_ID`]
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `msg`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
