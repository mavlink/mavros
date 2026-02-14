# cam_imu_sync

- File: `mavros_extras/src/plugins/cam_imu_sync.cpp`
- Class: `mavros::extra_plugins::CamIMUSyncPlugin`
- Namespace: `cam_imu_sync`
- Brief: Camera IMU synchronisation plugin


This plugin publishes a timestamp for when a external camera system was triggered by the FCU. Sequence ID from the message and the image sequence from camera can be corellated to get the exact shutter trigger time.

## Publishers
- `~/cam_imu_stamp` (mavros_msgs::msg::CamIMUStamp)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `CAMERA_TRIGGER` [handler: handle_cam_trig, dialect: common, msg_id: 112, id: `mavlink::common::msg::CAMERA_TRIGGER::MSG_ID`]


## MAVLink Publications
- None
