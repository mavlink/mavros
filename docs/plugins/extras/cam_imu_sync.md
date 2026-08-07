# cam_imu_sync

- File: `mavros_extras/src/plugins/cam_imu_sync.cpp`
- Class: `mavros::extra_plugins::CamIMUSyncPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Camera IMU synchronisation plugin


This plugin publishes a timestamp for when a external camera system was
triggered by the FCU. Sequence ID from the message and the image sequence from
camera can be correlated to get the exact shutter trigger time.

## Publishers
- `~/cam_imu_stamp` [type: [mavros_msgs::msg::CamIMUStamp](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/CamIMUStamp.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish camera IMU trigger timestamp from MAVLink CAMERA_TRIGGER.

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`CAMERA_TRIGGER`](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) [handler: handle_cam_trig, dialect: common, msg_id: 112, id: `mavlink::common::msg::CAMERA_TRIGGER::MSG_ID`]


## MAVLink Publications
- None
