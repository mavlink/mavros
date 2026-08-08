# setpoint_attitude

- File: `mavros/src/plugins/setpoint_attitude.cpp`
- Class: `mavros::std_plugins::SetpointAttitudePlugin`
- Namespace: `mavros::std_plugins`
- Brief: Setpoint attitude plugin


Send setpoint attitude/orientation/thrust to FCU controller. Uses the
[MAVLink Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html).

## Publishers
- None

## Subscribers
- `th_sub` [type: [mavros_msgs::msg::Thrust](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Thrust.html)]
- `pose_sub` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)]
- `twist_sub` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)]

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
- [`SET_ATTITUDE_TARGET`](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) [arg: `msg`, dialect: common, msg_id: 82, id: `mavlink::common::msg::SET_ATTITUDE_TARGET::MSG_ID`]
