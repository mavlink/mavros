# odometry

- File: `mavros_extras/src/plugins/odom.cpp`
- Class: `mavros::extra_plugins::OdometryPlugin`
- Namespace: `odometry`
- Brief: Odometry plugin


Sends odometry data to the FCU estimator and publishes odometry data that comes from FCU. This plugin is following ROS REP 147. Pose is expressed in parent frame. (Quaternion rotates from child to parent) The twist is expressed in the child frame. @see odom_cb()	transforming and sending odometry to fcu @see handle_odom()	receiving and transforming odometry from fcu

## Publishers
- `~/in` (nav_msgs::msg::Odometry) - publishers


## Subscribers
- `~/out` (nav_msgs::msg::Odometry)


## Services
- None


## Clients
- None


## Parameters
- `fcu.odom_parent_id_des` [default: `uas_->get_odom_frame_id()`] - frame params:
- `fcu.odom_child_id_des` [default: `uas_->get_base_link_frame_id()`]
- `fcu.map_id_des` [default: `uas_->get_map_frame_id()`]


## MAVLink Subscriptions
- `ODOMETRY` [handler: handle_odom, dialect: common, msg_id: 331, id: `mavlink::common::msg::ODOMETRY::MSG_ID`]


## MAVLink Publications
- `ODOMETRY` [arg: `msg`, dialect: common, msg_id: 331, id: `mavlink::common::msg::ODOMETRY::MSG_ID`] - send ODOMETRY msg
