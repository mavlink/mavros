# setpoint_trajectory

- File: `mavros/src/plugins/setpoint_trajectory.cpp`
- Class: `mavros::std_plugins::SetpointTrajectoryPlugin`
- Namespace: `setpoint_trajectory`
- Brief: Setpoint TRAJECTORY plugin


Receive trajectory setpoints and send setpoint_raw setpoints along the trajectory.

## Publishers
- `~/desired` ([nav_msgs::msg::Path](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html))


## Subscribers
- `~/local` ([trajectory_msgs::msg::MultiDOFJointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/MultiDOFJointTrajectory.html))


## Services
- `~/reset` ([std_srvs::srv::Trigger](https://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html))


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`]
- `mav_frame` [default: `"LOCAL_NED"`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`SET_POSITION_TARGET_LOCAL_NED`](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) [arg: `sp`, dialect: common, msg_id: 84, id: `mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED::MSG_ID`]
