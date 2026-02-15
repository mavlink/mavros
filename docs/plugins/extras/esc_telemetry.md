# esc_telemetry

- File: `mavros_extras/src/plugins/esc_telemetry.cpp`
- Class: `mavros::extra_plugins::ESCTelemetryPlugin`
- Namespace: `esc_telemetry`
- Brief: ESC telemetry plugin


APM specific plugin.

## Publishers
- `~/telemetry` ([mavros_msgs::msg::ESCTelemetry](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ESCTelemetry.html))


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`ESC_TELEMETRY_1_TO_4`](https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4) [handler: handle_esc_telemetry_1_to_4, dialect: ardupilotmega, msg_id: 11030, id: `mavlink::ardupilotmega::msg::ESC_TELEMETRY_1_TO_4::MSG_ID`]
- [`ESC_TELEMETRY_5_TO_8`](https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_5_TO_8) [handler: handle_esc_telemetry_5_to_8, dialect: ardupilotmega, msg_id: 11031, id: `mavlink::ardupilotmega::msg::ESC_TELEMETRY_5_TO_8::MSG_ID`]
- [`ESC_TELEMETRY_9_TO_12`](https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_9_TO_12) [handler: handle_esc_telemetry_9_to_12, dialect: ardupilotmega, msg_id: 11032, id: `mavlink::ardupilotmega::msg::ESC_TELEMETRY_9_TO_12::MSG_ID`]


## MAVLink Publications
- None
