# trajectory

- File: `mavros_extras/src/plugins/trajectory.cpp`
- Class: `mavros::extra_plugins::TrajectoryPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Trajectory plugin to receive planned path from the FCU and


send back to the FCU a corrected path (collision free, smoothed)



## Publishers
- `~/desired` [type: [mavros_msgs::msg::Trajectory](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Trajectory.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish desired trajectory from MAVLink TRAJECTORY.

## Subscribers
- `~/generated` [type: [mavros_msgs::msg::Trajectory](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Trajectory.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to Trajectory to send as TRAJECTORY to the FCU.
- `~/path` [type: [nav_msgs::msg::Path](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to nav_msgs/Path to send as TRAJECTORY to the FCU.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`TRAJECTORY_REPRESENTATION_WAYPOINTS`](https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_WAYPOINTS) [handler: handle_trajectory, dialect: common, msg_id: 332, id: `mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS::MSG_ID`]


## MAVLink Publications
- [`TRAJECTORY_REPRESENTATION_BEZIER`](https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_BEZIER) [arg: `msg`, dialect: common, msg_id: 333, id: `mavlink::common::msg::TRAJECTORY_REPRESENTATION_BEZIER::MSG_ID`]
- [`TRAJECTORY_REPRESENTATION_WAYPOINTS`](https://mavlink.io/en/messages/common.html#TRAJECTORY_REPRESENTATION_WAYPOINTS) [arg: `msg`, dialect: common, msg_id: 332, id: `mavlink::common::msg::TRAJECTORY_REPRESENTATION_WAYPOINTS::MSG_ID`]
