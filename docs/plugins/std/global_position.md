# global_position

- File: `mavros/src/plugins/global_position.cpp`
- Class: `mavros::std_plugins::GlobalPositionPlugin`
- Namespace: `global_position`
- Brief: Global position plugin.


Publishes global position. Conversion from GPS LLA to ECEF allows publishing local position to TF and PoseWithCovarianceStamped.

## Publishers
- `~/raw/fix` ([sensor_msgs::msg::NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)) - gps data
- `~/raw/gps_vel` ([geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html))
- `~/raw/satellites` ([std_msgs::msg::UInt32](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt32.html))
- `~/global` ([sensor_msgs::msg::NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)) - fused global position
- `~/local` ([nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html))
- `~/rel_alt` ([std_msgs::msg::Float64](https://docs.ros.org/en/rolling/p/std_msgs/msg/Float64.html))
- `~/compass_hdg` ([std_msgs::msg::Float64](https://docs.ros.org/en/rolling/p/std_msgs/msg/Float64.html))
- `~/gp_origin` ([geographic_msgs::msg::GeoPointStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPointStamped.html)) - global origin
- `~/gp_lp_offset` ([geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)) - offset from local position to the global origin ("earth")


## Subscribers
- `~/set_gp_origin` ([geographic_msgs::msg::GeoPointStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPointStamped.html))
- `home_position/home` ([mavros_msgs::msg::HomePosition](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HomePosition.html)) - home position subscriber to set "map" origin TODO(vooon): use UAS


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
- [`GPS_RAW_INT`](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) [handler: handle_gps_raw_int, dialect: common, msg_id: 24, id: `mavlink::common::msg::GPS_RAW_INT::MSG_ID`]
- [`GLOBAL_POSITION_INT`](https://mavlink.io/en/messages/standard.html#GLOBAL_POSITION_INT) [handler: handle_global_position_int, dialect: standard, msg_id: 33, id: `mavlink::standard::msg::GLOBAL_POSITION_INT::MSG_ID`] - GPS_STATUS: there no corresponding ROS message, and it is not supported by APM
- [`GPS_GLOBAL_ORIGIN`](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN) [handler: handle_gps_global_origin, dialect: common, msg_id: 49, id: `mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID`]
- [`LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) [handler: handle_lpned_system_global_offset, dialect: common, msg_id: 89, id: `mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET::MSG_ID`]


## MAVLink Publications
- [`SET_GPS_GLOBAL_ORIGIN`](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN) [arg: `gpo`, dialect: common, msg_id: 48, id: `mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN::MSG_ID`]
