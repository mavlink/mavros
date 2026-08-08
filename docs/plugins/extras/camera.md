# camera

- File: `mavros_extras/src/plugins/camera.cpp`
- Class: `mavros::extra_plugins::CameraPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Camera plugin plugin


Plugin for interfacing on the mavlink camera protocol. Implements the
[MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html).

## Publishers
- `~/image_captured` [type: [mavros_msgs::msg::CameraImageCaptured](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/CameraImageCaptured.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish camera image capture info from MAVLink CAMERA_IMAGE_CAPTURED.

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
