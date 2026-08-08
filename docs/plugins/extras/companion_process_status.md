# companion_process_status

- File: `mavros_extras/src/plugins/companion_process_status.cpp`
- Class: `mavros::extra_plugins::CompanionProcessStatusPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Obstacle companion process status plugin


Publishes the status of components running on the companion computer

## Publishers
- None

## Subscribers
- `~/status` [type: [mavros_msgs::msg::CompanionProcessStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/CompanionProcessStatus.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to CompanionProcessStatus to send as HEARTBEAT to the FCU.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`HEARTBEAT`](https://mavlink.io/en/messages/minimal.html#HEARTBEAT) [arg: `msg`, dialect: minimal, msg_id: 0, id: `mavlink::minimal::msg::HEARTBEAT::MSG_ID`]
