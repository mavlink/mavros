# tunnel

- File: `mavros_extras/src/plugins/tunnel.cpp`
- Class: `mavros::extra_plugins::TunnelPlugin`
- Namespace: `tunnel`
- Brief: Tunnel plugin


Implements the [MAVLink Tunnel Protocol](https://mavlink.io/en/services/tunnel.html).

## Publishers
- `~/out` ([mavros_msgs::msg::Tunnel](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Tunnel.html)) - Publish TUNNEL messages received from the FCU.


## Subscribers
- `~/in` ([mavros_msgs::msg::Tunnel](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/Tunnel.html)) - Subscribe to Tunnel to send as TUNNEL to the FCU.


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`TUNNEL`](https://mavlink.io/en/messages/common.html#TUNNEL) [handler: mav_callback, dialect: common, msg_id: 385, id: `mavlink::common::msg::TUNNEL::MSG_ID`]


## MAVLink Publications
- [`TUNNEL`](https://mavlink.io/en/messages/common.html#TUNNEL) [arg: `mav_tunnel`, dialect: common, msg_id: 385, id: `mavlink::common::msg::TUNNEL::MSG_ID`]
