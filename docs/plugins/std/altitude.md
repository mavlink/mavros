# altitude

- File: `mavros/src/plugins/altitude.cpp`
- Class: `mavros::std_plugins::AltitudePlugin`
- Namespace: `mavros::std_plugins`
- Brief: Altitude plugin.


Publish altitude data.

## Publishers
- `altitude` [type: [mavros_msgs::msg::Altitude](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Altitude.html), qos: [SensorDataQoS](../qos.md#sensordataqos "SensorDataQoS QoS profile")] - Publish altitude data (ALTITUDE).

## Subscribers
- None

## Services
- None

## Clients
- None


## Parameters
- `frame_id` [type: string, default: `"map"`] - Coordinate frame used for the altitude topic headers.


## MAVLink Subscriptions
- [`ALTITUDE`](https://mavlink.io/en/messages/common.html#ALTITUDE) [handler: handle_altitude, dialect: common, msg_id: 141, id: `mavlink::common::msg::ALTITUDE::MSG_ID`]


## MAVLink Publications
- None
