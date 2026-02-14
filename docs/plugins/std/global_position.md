# global_position

- File: `mavros/src/plugins/global_position.cpp`
- Class: `mavros::std_plugins::GlobalPositionPlugin`
- Namespace: `global_position`
- Brief: Global position plugin.


Publishes global position. Conversion from GPS LLA to ECEF allows publishing local position to TF and PoseWithCovarianceStamped.

## Publishers
- `~/raw/fix` (sensor_msgs::msg::NavSatFix) - gps data
- `~/raw/gps_vel` (geometry_msgs::msg::TwistStamped)
- `~/raw/satellites` (std_msgs::msg::UInt32)
- `~/global` (sensor_msgs::msg::NavSatFix) - fused global position
- `~/local` (nav_msgs::msg::Odometry)
- `~/rel_alt` (std_msgs::msg::Float64)
- `~/compass_hdg` (std_msgs::msg::Float64)
- `~/gp_origin` (geographic_msgs::msg::GeoPointStamped) - global origin
- `~/gp_lp_offset` (geometry_msgs::msg::PoseStamped) - offset from local position to the global origin ("earth")


## Subscribers
- `~/set_gp_origin` (geographic_msgs::msg::GeoPointStamped)
- `home_position/home` (mavros_msgs::msg::HomePosition) - home position subscriber to set "map" origin TODO(vooon): use UAS


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`] - general params
- `child_frame_id` [default: `"base_link"`]
- `rot_covariance` [type: double, default: `99999.0`]
- `gps_uere` [type: double, default: `1.0`]
- `use_relative_alt` [type: bool, default: `true`]
- `tf.send` [type: bool, default: `false`] - tf subsection
- `tf.frame_id` [default: `"map"`]
- `tf.global_frame_id` [default: `"earth"`] - The global_origin should be represented as "earth" coordinate frame (ECEF) (REP 105)
- `tf.child_frame_id` [default: `"base_link"`]


## MAVLink Subscriptions
- `GPS_RAW_INT` [handler: handle_gps_raw_int, dialect: common, msg_id: 24, id: `mavlink::common::msg::GPS_RAW_INT::MSG_ID`]
- `GLOBAL_POSITION_INT` [handler: handle_global_position_int, dialect: standard, msg_id: 33, id: `mavlink::standard::msg::GLOBAL_POSITION_INT::MSG_ID`] - GPS_STATUS: there no corresponding ROS message, and it is not supported by APM
- `GPS_GLOBAL_ORIGIN` [handler: handle_gps_global_origin, dialect: common, msg_id: 49, id: `mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID`]
- `LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET` [handler: handle_lpned_system_global_offset, dialect: common, msg_id: 89, id: `mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET::MSG_ID`]


## MAVLink Publications
- `SET_GPS_GLOBAL_ORIGIN` [arg: `gpo`, dialect: common, msg_id: 48, id: `mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN::MSG_ID`]
