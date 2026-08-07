# wheel_odometry

- File: `mavros_extras/src/plugins/wheel_odometry.cpp`
- Class: `mavros::extra_plugins::WheelOdometryPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Wheel odometry plugin.


This plugin allows computing and publishing wheel odometry coming from FCU wheel encoders.
Can use either wheel's RPM or WHEEL_DISTANCE messages (the latter gives better accuracy).

## Publishers
- `~/rpm` [type: [mavros_msgs::msg::WheelOdomStamped](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/WheelOdomStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish raw wheel RPM from MAVLink RPM.
- `~/distance` [type: [mavros_msgs::msg::WheelOdomStamped](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/WheelOdomStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish raw wheel distances from MAVLink WHEEL_DISTANCE.
- `~/velocity` [type: [geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish computed wheel odometry as TwistWithCovarianceStamped.
- `~/odom` [type: [nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish computed wheel odometry as nav_msgs/Odometry.

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- `send_raw` [type: bool, default: `false`] - General params Publish raw wheel RPM and distance data.
- `count` [type: integer, default: `2`] - Number of wheels used for odometry computation.
- `use_rpm` [type: bool, default: `false`] - Use wheel RPM (true) or cumulative distance (false) for odometry.
- `send_twist` [type: bool, default: `false`] - Odometry params Publish TwistWithCovarianceStamped instead of Odometry.
- `frame_id` [default: `uas_->get_odom_frame_id()`] - Frame id for published odometry.
- `child_frame_id` [default: `uas_->get_base_link_frame_id()`] - Child frame id for published odometry.
- `vel_error` [type: double, default: `0.1`] - Wheel velocity measurement error (std) [m/s].
- `tf.frame_id` [default: `uas_->get_odom_frame_id()`] - TF subsection TF frame id for published odometry.
- `tf.child_frame_id` [default: `uas_->get_base_link_frame_id()`] - TF child frame id for published odometry.
- `tf.send` [type: bool, default: `false`] - Enable publishing odometry transform to TF.
- `utils::format("wheel%i.x", wi)` [type: double, default: `0.0`] - Build the string in the form "wheelX", where X is the wheel number. Check if we have "wheelX" parameter. Indices starts from 0 and should increase without gaps.
- `utils::format("wheel%i.y", wi)` [type: double, default: `0.0`] - Wheel Y offset from vehicle origin [m].
- `utils::format("wheel%i.radius", wi)` [type: double, default: `0.05`] - Wheel radius [m].


## MAVLink Subscriptions
- [`RPM`](https://mavlink.io/en/messages/ardupilotmega.html#RPM) [handler: handle_rpm, dialect: ardupilotmega, msg_id: 226, id: `mavlink::ardupilotmega::msg::RPM::MSG_ID`]
- [`WHEEL_DISTANCE`](https://mavlink.io/en/messages/common.html#WHEEL_DISTANCE) [handler: handle_wheel_distance, dialect: common, msg_id: 9000, id: `mavlink::common::msg::WHEEL_DISTANCE::MSG_ID`]


## MAVLink Publications
- None
