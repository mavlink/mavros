# global_position

- File: `mavros/src/plugins/global_position.cpp`
- Class: `mavros::std_plugins::GlobalPositionPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Global position plugin.


Publishes global position. Conversion from GPS LLA to ECEF allows
publishing local position to TF and PoseWithCovarianceStamped.

## Publishers
- `~/raw/fix` [type: [sensor_msgs::msg::NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish raw GPS fix (GPS_RAW_INT).
- `~/raw/gps_vel` [type: [geometry_msgs::msg::TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish raw GPS velocity (GPS_RAW_INT).
- `~/raw/satellites` [type: [std_msgs::msg::UInt32](https://docs.ros.org/en/rolling/p/std_msgs/msg/UInt32.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish number of visible GPS satellites (GPS_RAW_INT).
- `~/global` [type: [sensor_msgs::msg::NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish fused global position fix (GLOBAL_POSITION_INT).
- `~/local` [type: [nav_msgs::msg::Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish fused local position as odometry (GLOBAL_POSITION_INT).
- `~/rel_alt` [type: [std_msgs::msg::Float64](https://docs.ros.org/en/rolling/p/std_msgs/msg/Float64.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish fused relative altitude (GLOBAL_POSITION_INT).
- `~/compass_hdg` [type: [std_msgs::msg::Float64](https://docs.ros.org/en/rolling/p/std_msgs/msg/Float64.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish fused compass heading (GLOBAL_POSITION_INT).
- `~/gp_origin` [type: [geographic_msgs::msg::GeoPointStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPointStamped.html), qos: [LatchedStateQoS](../qos.md#latchedstateqos "LatchedStateQoS QoS profile")] - Publish the global origin (GPS_GLOBAL_ORIGIN).
- `~/gp_lp_offset` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish the offset from the local position to the global origin.

## Subscribers
- `~/set_gp_origin` [type: [geographic_msgs::msg::GeoPointStamped](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeoPointStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Set the global origin (SET_GPS_GLOBAL_ORIGIN).
- `home_position/home` [type: [mavros_msgs::msg::HomePosition](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/HomePosition.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Set the "map" origin from the home position (HOME_POSITION).

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"map"`] - general params Coordinate frame used for the global position topics.
- `child_frame_id` [type: string, default: `"base_link"`] - Child body-fixed frame used for the global position topics.
- `rot_covariance` [type: double, default: `99999.0`] - Rotation covariance for the odometry pose.
- `gps_uere` [type: double, default: `1.0`] - GPS user range error used to approximate position covariance.
- `use_relative_alt` [type: bool, default: `true`] - Use relative altitude instead of the geocentric altitude.
- `tf.send` [type: bool, default: `false`] - tf subsection Enable publishing of the global position transform.
- `tf.frame_id` [type: string, default: `"map"`] - Map frame used for the published transform.
- `tf.global_frame_id` [type: string, default: `"earth"`] - The global_origin should be represented as "earth" coordinate frame (ECEF) (REP 105) Global reference frame (ECEF) for the transform.
- `tf.child_frame_id` [type: string, default: `"base_link"`] - Child body-fixed frame used for the transform.


## MAVLink Subscriptions
- [`GPS_RAW_INT`](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) [handler: handle_gps_raw_int, dialect: common, msg_id: 24, id: `mavlink::common::msg::GPS_RAW_INT::MSG_ID`]
- [`GLOBAL_POSITION_INT`](https://mavlink.io/en/messages/standard.html#GLOBAL_POSITION_INT) [handler: handle_global_position_int, dialect: standard, msg_id: 33, id: `mavlink::standard::msg::GLOBAL_POSITION_INT::MSG_ID`]
- [`GPS_GLOBAL_ORIGIN`](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN) [handler: handle_gps_global_origin, dialect: common, msg_id: 49, id: `mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID`]
- [`LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) [handler: handle_lpned_system_global_offset, dialect: common, msg_id: 89, id: `mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET::MSG_ID`]


## MAVLink Publications
- [`SET_GPS_GLOBAL_ORIGIN`](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN) [arg: `msg`, dialect: common, msg_id: 48, id: `mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN::MSG_ID`]
