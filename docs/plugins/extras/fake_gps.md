# fake_gps

- File: `mavros_extras/src/plugins/fake_gps.cpp`
- Class: `mavros::extra_plugins::FakeGPSPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Fake GPS plugin.


Sends fake GPS from local position estimation source data (motion capture,
vision) to FCU - processed in HIL mode or out of it if parameter MAV_USEHILGPS
is set on PX4 Pro Autopilot Firmware; Ardupilot Firmware already supports it
without a flag set.

## Publishers
- None

## Subscribers
- `~/mocap/tf` [type: [geometry_msgs::msg::TransformStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TransformStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - MoCap pose source as TransformStamped.
- `~/mocap/pose_cov` [type: [geometry_msgs::msg::PoseWithCovarianceStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseWithCovarianceStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - MoCap pose source as PoseWithCovarianceStamped.
- `~/mocap/pose` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - MoCap pose source as PoseStamped.
- `~/vision` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Vision pose source as PoseStamped.

## Services
- None

## Clients
- None


## Parameters
- `gps_id` [type: integer, default: `0`] - general params GPS receiver id in GPS_INPUT messages.
- `fix_type` [default: `utils::enum_value(GPS_FIX_TYPE::NO_GPS)`] - GPS fix type reported in fake GPS messages.
- `gps_rate` [type: double, default: `5.0`] - Rate at which fake GPS messages are sent.
- `eph` [type: double, default: `2.0`] - Horizontal position error [m].
- `epv` [type: double, default: `2.0`] - Vertical position error [m].
- `horiz_accuracy` [type: double, default: `0.0`] - Horizontal position accuracy [m].
- `vert_accuracy` [type: double, default: `0.0`] - Vertical position accuracy [m].
- `speed_accuracy` [type: double, default: `0.0`] - Speed accuracy [m/s].
- `satellites_visible` [type: integer, default: `5`] - Number of visible satellites reported in fake GPS.
- `geo_origin.lat` [type: double, default: `47.3667`] - default origin/starting point: Zürich geodetic coordinates Geodetic origin latitude [deg].
- `geo_origin.lon` [type: double, default: `8.5500`] - Geodetic origin longitude [deg].
- `geo_origin.alt` [type: double, default: `408.0`] - Geodetic origin altitude [m].
- `use_mocap` [type: bool, default: `true`] - source set params Listen to MoCap source.
- `mocap_transform` [type: bool, default: `true`] - Use TransformStamped if true, PoseStamped if false.
- `mocap_withcovariance` [type: bool, default: `false`] - ~mocap/pose uses PoseWithCovarianceStamped Message.
- `use_vision` [type: bool, default: `false`] - Listen to Vision source.
- `use_hil_gps` [type: bool, default: `false`] - Send HIL_GPS if true, GPS_INPUT if false.
- `tf.frame_id` [type: string, default: `"map"`] - tf params TF frame id for pose source.
- `tf.child_frame_id` [type: string, default: `"base_link"`] - TF child frame id for pose source.
- `tf.rate_limit` [type: double, default: `10.0`] - TF rate limit [Hz].
- `tf.listen` [type: bool, default: `false`] - Listen to TF pose source.


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`GPS_INPUT`](https://mavlink.io/en/messages/common.html#GPS_INPUT) [arg: `msg`, dialect: common, msg_id: 232, id: `mavlink::common::msg::GPS_INPUT::MSG_ID`]
- [`HIL_GPS`](https://mavlink.io/en/messages/common.html#HIL_GPS) [arg: `msg`, dialect: common, msg_id: 113, id: `mavlink::common::msg::HIL_GPS::MSG_ID`]
