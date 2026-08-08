# odometry

- File: `mavros_extras/src/plugins/odom.cpp`
- Class: `mavros::extra_plugins::OdometryPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Odometry plugin


Sends odometry data to the FCU estimator and
publishes odometry data that comes from FCU.



This plugin is following ROS REP 147. Pose is expressed in parent frame.
(Quaternion rotates from child to parent)
The twist is expressed in the child frame.



## Publishers
- `~/in` [type: [nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish odometry from MAVLink ODOMETRY.

## Subscribers
- `~/out` [type: [nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html), qos: [QoS(1)](../qos.md#qos_1_)] - subscribers Subscribe to odometry to send as ODOMETRY to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `fcu.odom_parent_id_des` [default: `uas_->get_odom_frame_id()`] - frame params: Desired parent frame id for odometry from the FCU.
- `fcu.odom_child_id_des` [default: `uas_->get_base_link_frame_id()`] - Desired child frame id for odometry from the FCU.
- `fcu.map_id_des` [default: `uas_->get_map_frame_id()`] - Desired map frame id for odometry from the FCU.


## MAVLink Subscriptions
- [`ODOMETRY`](https://mavlink.io/en/messages/common.html#ODOMETRY) [handler: handle_odom, dialect: common, msg_id: 331, id: `mavlink::common::msg::ODOMETRY::MSG_ID`]


## MAVLink Publications
- [`ODOMETRY`](https://mavlink.io/en/messages/common.html#ODOMETRY) [arg: `msg`, dialect: common, msg_id: 331, id: `mavlink::common::msg::ODOMETRY::MSG_ID`]
