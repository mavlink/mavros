# manual_control

- File: `mavros/src/plugins/manual_control.cpp`
- Class: `mavros::std_plugins::ManualControlPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Manual Control plugin


Implements the
[MAVLink Manual Control (Joystick) Protocol](https://mavlink.io/en/services/manual_control.html).

## Publishers
- `~/control` [type: [mavros_msgs::msg::ManualControl](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ManualControl.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish manual control input from FCU (MANUAL_CONTROL).

## Subscribers
- `~/send` [type: [mavros_msgs::msg::ManualControl](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ManualControl.html), qos: [QoS(10)](../qos.md#qos_10_)] - Send manual control commands to FCU (MANUAL_CONTROL).

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`MANUAL_CONTROL`](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL) [handler: handle_manual_control, dialect: common, msg_id: 69, id: `mavlink::common::msg::MANUAL_CONTROL::MSG_ID`]


## MAVLink Publications
- [`MANUAL_CONTROL`](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL) [arg: `msg`, dialect: common, msg_id: 69, id: `mavlink::common::msg::MANUAL_CONTROL::MSG_ID`]
