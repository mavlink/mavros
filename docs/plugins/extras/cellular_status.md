# cellular_status

- File: `mavros_extras/src/plugins/cellular_status.cpp`
- Class: `mavros::extra_plugins::CellularStatusPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Cellular status plugin.


Users must publish to the topic the CellularStatus message and it
will be relayed to the mavlink components.

## Publishers
- None

## Subscribers
- `~/status` [type: [mavros_msgs::msg::CellularStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/CellularStatus.html), qos: [QoS(1)](../qos.md#qos_1_)] - Subscribe to CellularStatus messages to send as CELLULAR_STATUS to the FCU.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`CELLULAR_STATUS`](https://mavlink.io/en/messages/common.html#CELLULAR_STATUS) [arg: `msg`, dialect: common, msg_id: 334, id: `mavlink::common::msg::CELLULAR_STATUS::MSG_ID`]
