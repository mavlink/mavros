# camera

- File: `mavros_extras/src/plugins/camera.cpp`
- Class: `mavros::extra_plugins::CameraPlugin`
- Namespace: `camera`
- Brief: Camera plugin plugin


Plugin for interfacing on the mavlink camera protocol @see command_cb()

## Publishers
- `~/image_captured` (mavros_msgs::msg::CameraImageCaptured)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`CAMERA_IMAGE_CAPTURED`](https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED) [handler: handle_camera_image_captured, dialect: common, msg_id: 263, id: `mavlink::common::msg::CAMERA_IMAGE_CAPTURED::MSG_ID`]


## MAVLink Publications
- None
