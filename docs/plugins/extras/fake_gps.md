# fake_gps

- File: `mavros_extras/src/plugins/fake_gps.cpp`
- Class: `mavros::extra_plugins::FakeGPSPlugin`
- Namespace: `fake_gps`
- Brief: Fake GPS plugin.


Sends fake GPS from local position estimation source data (motion capture, vision) to FCU - processed in HIL mode or out of it if parameter MAV_USEHILGPS is set on PX4 Pro Autopilot Firmware; Ardupilot Firmware already supports it without a flag set.

## Publishers
- None


## Subscribers
- `~/mocap/tf` (geometry_msgs::msg::TransformStamped)
- `~/mocap/pose_cov` (geometry_msgs::msg::PoseWithCovarianceStamped)
- `~/mocap/pose` (geometry_msgs::msg::PoseStamped)
- `~/vision` (geometry_msgs::msg::PoseStamped)


## Services
- None


## Clients
- None


## Parameters
- `gps_id` [type: integer, default: `0`] - general params
- `fix_type` [default: `utils::enum_value(GPS_FIX_TYPE::NO_GPS)`]
- `gps_rate` [type: double, default: `5.0`]
- `eph` [type: double, default: `2.0`]
- `epv` [type: double, default: `2.0`]
- `horiz_accuracy` [type: double, default: `0.0`]
- `vert_accuracy` [type: double, default: `0.0`]
- `speed_accuracy` [type: double, default: `0.0`]
- `satellites_visible` [type: integer, default: `5`]
- `geo_origin.lat` [type: double, default: `47.3667`] - default origin/starting point: Zürich geodetic coordinates
- `geo_origin.lon` [type: double, default: `8.5500`]
- `geo_origin.alt` [type: double, default: `408.0`]
- `use_hil_gps` [type: bool, default: `false`]
- `tf.frame_id` [default: `"map"`] - tf params
- `tf.child_frame_id` [default: `"base_link"`]
- `tf.rate_limit` [type: double, default: `10.0`]
- `tf.listen` [type: bool, default: `false`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- `HIL_GPS` [arg: `hil_gps`, dialect: common, msg_id: 113, id: `mavlink::common::msg::HIL_GPS::MSG_ID`]
- `GPS_INPUT` [arg: `gps_input`, dialect: common, msg_id: 232, id: `mavlink::common::msg::GPS_INPUT::MSG_ID`]
