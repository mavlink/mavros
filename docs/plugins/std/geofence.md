# geofence

- File: `mavros/src/plugins/geofence.cpp`
- Class: `mavros::std_plugins::GeofencePlugin`
- Namespace: `<unknown>`
- Brief: Geofence manipulation plugin


## Publishers
- `~/fences` (mavros_msgs::msg::WaypointList)


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
