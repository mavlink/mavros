# setpoint_attitude

- File: `mavros/src/plugins/setpoint_attitude.cpp`
- Class: `mavros::std_plugins::SetpointAttitudePlugin`
- Namespace: `setpoint_attitude`
- Brief: Setpoint attitude plugin


Send setpoint attitude/orientation/thrust to FCU controller.

## Publishers
- None


## Subscribers
- `~/attitude` (geometry_msgs::msg::PoseStamped) - Use message_filters to sync attitude and thrust msg coming from different topics
- `~/cmd_vel` (geometry_msgs::msg::TwistStamped)
- `~/thrust` (mavros_msgs::msg::Thrust) - thrust msg subscriber to sync


## Services
- None


## Clients
- None


## Parameters
- `reverse_thrust` [type: bool, default: `false`]
- `use_quaternion` [type: bool, default: `false`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_ATTITUDE_TARGET`](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) [arg: `sp`, dialect: common, msg_id: 82, id: `mavlink::common::msg::SET_ATTITUDE_TARGET::MSG_ID`]
