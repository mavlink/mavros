# gps_input

- File: `mavros_extras/src/plugins/gps_input.cpp`
- Class: `mavros::extra_plugins::GpsInputPlugin`
- Namespace: `gps_input`
- Brief: GPS_INPUT GPS plugin.


Sends <a href="https://mavlink.io/en/messages/common.html#GPS_INPUT">GPS_INPUT MAVLink messages</a>

## Publishers
- None


## Subscribers
- `~/gps_input` ([mavros_msgs::msg::GPSINPUT](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/GPSINPUT.html))


## Services
- None


## Clients
- None


## Parameters
- `gps_rate` [type: double, default: `5.0`]


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`GPS_INPUT`](https://mavlink.io/en/messages/common.html#GPS_INPUT) [arg: `gps_input`, dialect: common, msg_id: 232, id: `mavlink::common::msg::GPS_INPUT::MSG_ID`]
