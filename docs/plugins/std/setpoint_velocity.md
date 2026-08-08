# setpoint_velocity

- File: `mavros/src/plugins/setpoint_velocity.cpp`
- Class: `mavros::std_plugins::SetpointVelocityPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Setpoint velocity plugin


Send setpoint velocities to FCU controller. Uses the
[MAVLink Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html).

## Publishers
- None

## Subscribers
- `~/cmd_vel` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Velocity setpoint (SET_POSITION_TARGET_LOCAL_NED).
- `~/cmd_vel_unstamped` [type: [geometry_msgs::msg::Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Velocity setpoint without timestamp (SET_POSITION_TARGET_LOCAL_NED).

## Services
- None

## Clients
- None


## Parameters
- `mav_frame` [type: string, default: `"LOCAL_NED"`] - Coordinate frame of the velocity setpoints.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `msg`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
