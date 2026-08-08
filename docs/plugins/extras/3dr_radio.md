# tdr_radio

- File: `mavros_extras/src/plugins/3dr_radio.cpp`
- Class: `mavros::extra_plugins::TDRRadioPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: 3DR Radio plugin.


## Publishers
- `radio_status` [type: [mavros_msgs::msg::RadioStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RadioStatus.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish radio status from MAVLink RADIO_STATUS/RADIO.

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- `low_rssi` [type: integer, default: `40`] - Low RSSI threshold for diagnostics [dB].


## MAVLink Subscriptions
- [`RADIO_STATUS`](https://mavlink.io/en/messages/common.html#RADIO_STATUS) [handler: handle_radio_status, dialect: common, msg_id: 109, id: `mavlink::common::msg::RADIO_STATUS::MSG_ID`]
- [`RADIO`](https://mavlink.io/en/messages/ardupilotmega.html#RADIO) [handler: handle_radio, dialect: ardupilotmega, msg_id: 166, id: `mavlink::ardupilotmega::msg::RADIO::MSG_ID`]


## MAVLink Publications
- None
