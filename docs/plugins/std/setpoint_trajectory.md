# setpoint_trajectory

- File: `mavros/src/plugins/setpoint_trajectory.cpp`
- Class: `mavros::std_plugins::SetpointTrajectoryPlugin`
- Namespace: `setpoint_trajectory`
- Brief: Setpoint TRAJECTORY plugin


Receive trajectory setpoints and send setpoint_raw setpoints along the trajectory.

## Publishers
- `~/desired` (nav_msgs::msg::Path)


## Subscribers
- `~/local` (trajectory_msgs::msg::MultiDOFJointTrajectory)


## Services
- `~/reset` (std_srvs::srv::Trigger)


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`]
- `mav_frame` [default: `"LOCAL_NED"`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- None
