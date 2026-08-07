# setpoint_accel

- File: `mavros/src/plugins/setpoint_accel.cpp`
- Class: `mavros::std_plugins::SetpointAccelerationPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Setpoint acceleration/force plugin


Send setpoint accelerations/forces to FCU controller. Uses the
[MAVLink Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html).

## Publishers
- None

## Subscribers
- `~/accel` [type: [geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Setpoint acceleration/force (SET_POSITION_TARGET_LOCAL_NED).

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `msg`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
