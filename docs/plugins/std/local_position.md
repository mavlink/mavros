# local_position

- File: `mavros/src/plugins/local_position.cpp`
- Class: `mavros::std_plugins::LocalPositionPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Local position plugin.


Publish local position to TF, PositionStamped, TwistStamped
and Odometry

## Publishers
- `~/pose` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local position (LOCAL_POSITION_NED).
- `~/pose_cov` [type: [geometry_msgs::msg::PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local position with covariance (LOCAL_POSITION_NED_COV).
- `~/velocity_local` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local velocity in NED frame (LOCAL_POSITION_NED).
- `~/velocity_body` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local velocity in body frame (LOCAL_POSITION_NED).
- `~/velocity_body_cov` [type: [geometry_msgs::msg::TwistWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistWithCovarianceStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local velocity with covariance (LOCAL_POSITION_NED_COV).
- `~/accel` [type: [geometry_msgs::msg::AccelWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/AccelWithCovarianceStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local acceleration with covariance (LOCAL_POSITION_NED_COV).
- `~/odom` [type: [nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Local odometry (LOCAL_POSITION_NED / LOCAL_POSITION_NED_COV).

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"map"`] - header frame_id. default to map (world-fixed, ENU as per REP-105). Set the frame id for the published messages.
- `tf.send` [type: bool, default: `false`] - Important tf subsection Report the transform from world to base_link here. Enable publishing of the world to base_link TF tree.
- `tf.frame_id` [type: string, default: `"map"`] - World frame id for the published TF.
- `tf.child_frame_id` [type: string, default: `"base_link"`] - Body frame id for the published TF.


## MAVLink Subscriptions
- [`LOCAL_POSITION_NED`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED) [handler: handle_local_position_ned, dialect: common, msg_id: 32, id: `mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID`]
- [`LOCAL_POSITION_NED_COV`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_COV) [handler: handle_local_position_ned_cov, dialect: common, msg_id: 64, id: `mavlink::common::msg::LOCAL_POSITION_NED_COV::MSG_ID`]


## MAVLink Publications
- None
