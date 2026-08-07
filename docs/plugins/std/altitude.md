# altitude

- File: `mavros/src/plugins/altitude.cpp`
- Class: `mavros::std_plugins::AltitudePlugin`
- Namespace: `altitude`
- Brief: Altitude plugin.


Publish altitude data.

## Publishers
- `altitude` ([mavros_msgs::msg::Altitude](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Altitude.html)) - ALTITUDE data Publish altitude data (ALTITUDE).


## Subscribers
- None


## Services
- None


## Clients
- None


## Parameters
- `frame_id` [default: `"map"`] - Coordinate frame used for the altitude topic headers.


## MAVLink Subscriptions
- [`ALTITUDE`](https://mavlink.io/en/messages/common.html#ALTITUDE) [handler: handle_altitude, dialect: common, msg_id: 141, id: `mavlink::common::msg::ALTITUDE::MSG_ID`]


## MAVLink Publications
- None
