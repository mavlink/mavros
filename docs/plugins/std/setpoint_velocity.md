# setpoint_velocity

- File: `mavros/src/plugins/setpoint_velocity.cpp`
- Class: `mavros::std_plugins::SetpointVelocityPlugin`
- Namespace: `setpoint_velocity`
- Brief: Setpoint velocity plugin


Send setpoint velocities to FCU controller. Uses the [MAVLink Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html).

## Publishers
- None


## Subscribers
- `~/cmd_vel` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)) - Velocity setpoint (SET_POSITION_TARGET_LOCAL_NED).
- `~/cmd_vel_unstamped` ([geometry_msgs::msg::Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)) - Velocity setpoint without timestamp (SET_POSITION_TARGET_LOCAL_NED).


## Services
- None


## Clients
- None


## Parameters
- `mav_frame` [default: `"LOCAL_NED"`] - Coordinate frame of the velocity setpoints.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `sp`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
