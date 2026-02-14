# vibration

- File: `mavros_extras/src/plugins/vibration.cpp`
- Class: `mavros::extra_plugins::VibrationPlugin`
- Namespace: `vibration`
- Brief: Vibration plugin


This plugin is intended to publish MAV vibration levels and accelerometer clipping from FCU.

## Publishers
- `~/raw/vibration` (mavros_msgs::msg::Vibration)


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"base_link"`]


## MAVLink Subscriptions
- `VIBRATION` [handler: handle_vibration, dialect: common, msg_id: 241, id: `mavlink::common::msg::VIBRATION::MSG_ID`]


## MAVLink Publications
- None
