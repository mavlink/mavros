# local_position

- File: `mavros/src/plugins/local_position.cpp`
- Class: `mavros::std_plugins::LocalPositionPlugin`
- Namespace: `local_position`
- Brief: Local position plugin.


Publish local position to TF, PositionStamped, TwistStamped and Odometry

## Publishers
- `~/pose` ([geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html))
- `~/pose_cov` ([geometry_msgs::msg::PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html))
- `~/velocity_local` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html))
- `~/velocity_body` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html))
- `~/velocity_body_cov` ([geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html))
- `~/accel` ([geometry_msgs::msg::AccelWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/AccelWithCovarianceStamped.html))
- `~/odom` ([nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html))


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`] - header frame_id. default to map (world-fixed, ENU as per REP-105).
- `tf.send` [type: bool, default: `false`] - Important tf subsection Report the transform from world to base_link here.
- `tf.frame_id` [default: `"map"`]
- `tf.child_frame_id` [default: `"base_link"`]


## MAVLink Subscriptions
- [`LOCAL_POSITION_NED`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED) [handler: handle_local_position_ned, dialect: common, msg_id: 32, id: `mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID`]
- [`LOCAL_POSITION_NED_COV`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_COV) [handler: handle_local_position_ned_cov, dialect: common, msg_id: 64, id: `mavlink::common::msg::LOCAL_POSITION_NED_COV::MSG_ID`]


## MAVLink Publications
- None
