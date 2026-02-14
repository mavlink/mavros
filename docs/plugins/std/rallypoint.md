# rallypoint

- File: `mavros/src/plugins/rallypoint.cpp`
- Class: `mavros::std_plugins::RallypointPlugin`
- Namespace: `<unknown>`
- Brief: Rallypoint manipulation plugin


## Publishers
- `~/rallypoints` (mavros_msgs::msg::WaypointList)


## Subscribers
- None


## Services
- `~/pull` (mavros_msgs::srv::WaypointPull)
- `~/push` (mavros_msgs::srv::WaypointPush)
- `~/clear` (mavros_msgs::srv::WaypointClear)


## Clients
- None


## Parameters
- `pull_after_gcs` [type: bool, default: `true`] - NOTE(vooon): I'm not quite sure that this option would work with mavros router
- `use_mission_item_int` [type: bool, default: `true`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- None
