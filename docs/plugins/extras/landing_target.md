# landing_target

- File: `mavros_extras/src/plugins/landing_target.cpp`
- Class: `mavros::extra_plugins::LandingTargetPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Landing Target plugin


This plugin is intended to publish the location of a landing area captured from a downward facing camera
to the FCU and/or receive landing target tracking data coming from the FCU. Implements the
[MAVLink Landing Target Protocol](https://mavlink.io/en/services/landing_target.html).

## Publishers
- `~/pose_in` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish landing target pose from MAVLink LANDING_TARGET.
- `~/lt_marker` [type: [geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish landing target size as Vector3Stamped.

## Subscribers
- `~/raw` [type: [mavros_msgs::msg::LandingTarget](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/LandingTarget.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to raw LandingTarget messages to send to the FCU.
- `~/pose` [type: [geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to landing target pose to send as LANDING_TARGET to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"landing_target_1"`] - general params Frame id for published landing target messages.
- `listen_lt` [type: bool, default: `false`] - Enable listening to raw LandingTarget messages.
- `mav_frame` [type: string, default: `"LOCAL_NED"`] - MAVLink MAV_FRAME used when sending LANDING_TARGET.
- `land_target_type` [type: string, default: `"VISION_FIDUCIAL"`] - MAVLink LANDING_TARGET_TYPE used when sending LANDING_TARGET.
- `target_size.x` [type: double, default: `1.0`] - target size Landing target size on X axis [m].
- `target_size.y` [type: double, default: `1.0`] - Landing target size on Y axis [m].
- `image.width` [type: integer, default: `640`] - image size Image width in pixels.
- `image.height` [type: integer, default: `480`] - Image height in pixels.
- `camera.fov_x` [type: double, default: `2.0071286398`] - camera field-of-view -> should be precised using the calibrated camera intrinsics Camera field-of-view on X axis [rad].
- `camera.fov_y` [type: double, default: `2.0071286398`] - Camera field-of-view on Y axis [rad].
- `camera.focal_length` [type: double, default: `2.8`] - camera focal length Camera focal length [mm].
- `tf.rate_limit` [type: double, default: `50.0`] - tf subsection Landing target transform rate limit [Hz].
- `tf.send` [type: bool, default: `true`] - Enable sending landing target transform to TF.
- `tf.frame_id` [default: `frame_id`] - TF frame id for landing target.
- `tf.child_frame_id` [type: string, default: `"camera_center"`] - TF child frame id for landing target.
- `tf.listen` [type: bool, default: `false`] - Listen to landing target pose from TF.


## MAVLink Subscriptions
- [`LANDING_TARGET`](https://mavlink.io/en/messages/common.html#LANDING_TARGET) [handler: handle_landing_target, dialect: common, msg_id: 149, id: `mavlink::common::msg::LANDING_TARGET::MSG_ID`]


## MAVLink Publications
- [`LANDING_TARGET`](https://mavlink.io/en/messages/common.html#LANDING_TARGET) [arg: `msg`, dialect: common, msg_id: 149, id: `mavlink::common::msg::LANDING_TARGET::MSG_ID`]
