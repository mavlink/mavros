# landing_target

- File: `mavros_extras/src/plugins/landing_target.cpp`
- Class: `mavros::extra_plugins::LandingTargetPlugin`
- Namespace: `landing_target`
- Brief: Landing Target plugin


This plugin is intended to publish the location of a landing area captured from a downward facing camera to the FCU and/or receive landing target tracking data coming from the FCU.

## Publishers
- `~/pose_in` ([geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html))
- `~/lt_marker` ([geometry_msgs::msg::Vector3Stamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Vector3Stamped.html))


## Subscribers
- `~/raw` ([mavros_msgs::msg::LandingTarget](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/LandingTarget.html))
- `~/pose` ([geometry_msgs::msg::PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html))


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"landing_target_1"`] - general params
- `listen_lt` [type: bool, default: `false`]
- `mav_frame` [default: `"LOCAL_NED"`]
- `land_target_type` [default: `"VISION_FIDUCIAL"`]
- `target_size.x` [type: double, default: `1.0`] - target size
- `target_size.y` [type: double, default: `1.0`]
- `image.width` [type: integer, default: `640`] - image size
- `image.height` [type: integer, default: `480`]
- `camera.fov_x` [type: double, default: `2.0071286398`] - camera field-of-view -> should be precised using the calibrated camera intrinsics
- `camera.fov_y` [type: double, default: `2.0071286398`]
- `camera.focal_length` [type: double, default: `2.8`] - camera focal length
- `tf.rate_limit` [type: double, default: `50.0`] - tf subsection
- `tf.send` [type: bool, default: `true`]
- `tf.frame_id` [default: `frame_id`]
- `tf.child_frame_id` [default: `"camera_center"`]
- `tf.listen` [type: bool, default: `false`]


## MAVLink Subscriptions
- [`LANDING_TARGET`](https://mavlink.io/en/messages/common.html#LANDING_TARGET) [handler: handle_landing_target, dialect: common, msg_id: 149, id: `mavlink::common::msg::LANDING_TARGET::MSG_ID`]


## MAVLink Publications
- [`LANDING_TARGET`](https://mavlink.io/en/messages/common.html#LANDING_TARGET) [arg: `lt`, dialect: common, msg_id: 149, id: `mavlink::common::msg::LANDING_TARGET::MSG_ID`]
