# vibration

- File: `mavros_extras/src/plugins/vibration.cpp`
- Class: `mavros::extra_plugins::VibrationPlugin`
- Namespace: `vibration`
- Brief: Vibration plugin


This plugin is intended to publish MAV vibration levels and accelerometer clipping from FCU.

## Publishers
- `~/raw/vibration` ([mavros_msgs::msg::Vibration](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Vibration.html)) - Publish vibration levels from MAVLink VIBRATION.


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"base_link"`] - Frame id for published vibration messages.


## MAVLink Subscriptions
- [`VIBRATION`](https://mavlink.io/en/messages/common.html#VIBRATION) [handler: handle_vibration, dialect: common, msg_id: 241, id: `mavlink::common::msg::VIBRATION::MSG_ID`]


## MAVLink Publications
- None
