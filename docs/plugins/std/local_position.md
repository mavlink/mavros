# local_position

- File: `mavros/src/plugins/local_position.cpp`
- Class: `mavros::std_plugins::LocalPositionPlugin`
- Namespace: `local_position`
- Brief: Local position plugin.


Publish local position to TF, PositionStamped, TwistStamped and Odometry

## Publishers
- `~/pose` ([geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)) - Local position (LOCAL_POSITION_NED).
- `~/pose_cov` ([geometry_msgs::msg::PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html)) - Local position with covariance (LOCAL_POSITION_NED_COV).
- `~/velocity_local` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)) - Local velocity in NED frame (LOCAL_POSITION_NED).
- `~/velocity_body` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)) - Local velocity in body frame (LOCAL_POSITION_NED).
- `~/velocity_body_cov` ([geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html)) - Local velocity with covariance (LOCAL_POSITION_NED_COV).
- `~/accel` ([geometry_msgs::msg::AccelWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/AccelWithCovarianceStamped.html)) - Local acceleration with covariance (LOCAL_POSITION_NED_COV).
- `~/odom` ([nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html)) - Local odometry (LOCAL_POSITION_NED / LOCAL_POSITION_NED_COV).


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`] - header frame_id. default to map (world-fixed, ENU as per REP-105). Set the frame id for the published messages.
- `tf.send` [type: bool, default: `false`] - Important tf subsection Report the transform from world to base_link here. Enable publishing of the world to base_link TF tree.
- `tf.frame_id` [default: `"map"`] - World frame id for the published TF.
- `tf.child_frame_id` [default: `"base_link"`] - Body frame id for the published TF.


## MAVLink Subscriptions
- [`LOCAL_POSITION_NED`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED) [handler: handle_local_position_ned, dialect: common, msg_id: 32, id: `mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID`]
- [`LOCAL_POSITION_NED_COV`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_COV) [handler: handle_local_position_ned_cov, dialect: common, msg_id: 64, id: `mavlink::common::msg::LOCAL_POSITION_NED_COV::MSG_ID`]


## MAVLink Publications
- None
