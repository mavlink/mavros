# setpoint_trajectory

- File: `mavros/src/plugins/setpoint_trajectory.cpp`
- Class: `mavros::std_plugins::SetpointTrajectoryPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Setpoint TRAJECTORY plugin


Receive trajectory setpoints and send setpoint_raw setpoints along the trajectory. Uses the
[MAVLink Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html).

## Publishers
- `~/desired` [type: [nav_msgs::msg::Path](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish the desired trajectory path.

## Subscribers
- `~/local` [type: [trajectory_msgs::msg::MultiDOFJointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/MultiDOFJointTrajectory.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Trajectory setpoints (SET_POSITION_TARGET_LOCAL_NED).

## Services
- `~/reset` [type: [std_srvs::srv::Trigger](https://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html)] - Reset the current trajectory.

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"map"`] - Frame id for the published path.
- `mav_frame` [type: string, default: `"LOCAL_NED"`] - Coordinate frame of the trajectory setpoints.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `msg`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
