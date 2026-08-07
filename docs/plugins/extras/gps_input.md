# gps_input

- File: `mavros_extras/src/plugins/gps_input.cpp`
- Class: `mavros::extra_plugins::GpsInputPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: GPS_INPUT GPS plugin.


Sends <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink messages</a>

## Publishers
- None

## Subscribers
- `~/gps_input` [type: [mavros_msgs::msg::GPSINPUT](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GPSINPUT.html), qos: [QoS(1)](../qos.md#qos_1_)] - Subscribe to GPSINPUT to send as GPS_INPUT to the FCU.

## Services
- None

## Clients
- None


## Parameters
- `gps_rate` [type: double, default: `5.0`] - Rate at which GPS_INPUT messages are sent [Hz].


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`GPS_INPUT`](https://mavlink.io/en/messages/common.html#GPS_INPUT) [arg: `msg`, dialect: common, msg_id: 232, id: `mavlink::common::msg::GPS_INPUT::MSG_ID`]
