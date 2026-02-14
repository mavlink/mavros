# wheel_odometry

- File: `mavros_extras/src/plugins/wheel_odometry.cpp`
- Class: `mavros::extra_plugins::WheelOdometryPlugin`
- Namespace: `wheel_odometry`
- Brief: Wheel odometry plugin.


This plugin allows computing and publishing wheel odometry coming from FCU wheel encoders. Can use either wheel's RPM or WHEEL_DISTANCE messages (the latter gives better accuracy).

## Publishers
- `~/rpm` (mavros_msgs::msg::WheelOdomStamped)
- `~/distance` (mavros_msgs::msg::WheelOdomStamped)
- `~/velocity` (geometry_msgs::msg::TwistWithCovarianceStamped)
- `~/odom` (nav_msgs::msg::Odometry)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `send_raw` [type: bool, default: `false`] - General params
- `count` [type: integer, default: `2`]
- `use_rpm` [type: bool, default: `false`]
- `send_twist` [type: bool, default: `false`] - Odometry params
- `frame_id` [default: `uas_->get_odom_frame_id()`]
- `child_frame_id` [default: `uas_->get_base_link_frame_id()`]
- `vel_error` [type: double, default: `0.1`]
- `tf.frame_id` [default: `uas_->get_odom_frame_id()`] - TF subsection
- `tf.child_frame_id` [default: `uas_->get_base_link_frame_id()`]
- `tf.send` [type: bool, default: `false`]


## MAVLink Subscriptions
- `RPM` [handler: handle_rpm, dialect: ardupilotmega, msg_id: 226, id: `mavlink::ardupilotmega::msg::RPM::MSG_ID`]
- `WHEEL_DISTANCE` [handler: handle_wheel_distance, dialect: common, msg_id: 9000, id: `mavlink::common::msg::WHEEL_DISTANCE::MSG_ID`]


## MAVLink Publications
- None
