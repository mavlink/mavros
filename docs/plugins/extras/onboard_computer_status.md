# onboard_computer_status

- File: `mavros_extras/src/plugins/onboard_computer_status.cpp`
- Class: `mavros::extra_plugins::OnboardComputerStatusPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: Onboard Computer Status plugin


Publishes the status of the onboard computer

## Publishers
- None

## Subscribers
- `~/status` [type: [mavros_msgs::msg::OnboardComputerStatus](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OnboardComputerStatus.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to OnboardComputerStatus to send as ONBOARD_COMPUTER_STATUS.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- None


## MAVLink Publications
- [`ONBOARD_COMPUTER_STATUS`](https://mavlink.io/en/messages/common.html#ONBOARD_COMPUTER_STATUS) [arg: `msg`, dialect: common, msg_id: 390, id: `mavlink::common::msg::ONBOARD_COMPUTER_STATUS::MSG_ID`]
