# waypoint

- File: `mavros/src/plugins/waypoint.cpp`
- Class: `mavros::std_plugins::WaypointPlugin`
- Namespace: `<unknown>`
- Brief: Mission manipulation plugin


## Publishers
- `~/waypoints` (mavros_msgs::msg::WaypointList)
- `~/reached` (mavros_msgs::msg::WaypointReached)


## Subscribers
- None


## Services
- `~/pull` (mavros_msgs::srv::WaypointPull)
- `~/push` (mavros_msgs::srv::WaypointPush)
- `~/clear` (mavros_msgs::srv::WaypointClear)
- `~/set_current` (mavros_msgs::srv::WaypointSetCurrent)


## Clients
- None


## Parameters
- `pull_after_gcs` [type: bool, default: `true`] - NOTE(vooon): I'm not quite sure that this option would work with mavros router
- `use_mission_item_int` [type: bool, default: `true`]
- `enable_partial_push` [type: integer, default: `2`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- None
