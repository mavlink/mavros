# rc_io

- File: `mavros/src/plugins/rc_io.cpp`
- Class: `mavros::std_plugins::RCIOPlugin`
- Namespace: `mavros::std_plugins`
- Brief: RC IO plugin


## Publishers
- `~/in` [type: [mavros_msgs::msg::RCIn](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RCIn.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish RC input (RC_CHANNELS / RC_CHANNELS_RAW).
- `~/out` [type: [mavros_msgs::msg::RCOut](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/RCOut.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish RC servo output (SERVO_OUTPUT_RAW).

## Subscribers
- `~/override` [type: [mavros_msgs::msg::OverrideRCIn](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/OverrideRCIn.html), qos: [QoS(10)](../qos.md#qos_10_)] - Override RC input on the FCU (RC_CHANNELS_OVERRIDE).

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`RC_CHANNELS_RAW`](https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW) [handler: handle_rc_channels_raw, dialect: common, msg_id: 35, id: `mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID`]
- [`RC_CHANNELS`](https://mavlink.io/en/messages/common.html#RC_CHANNELS) [handler: handle_rc_channels, dialect: common, msg_id: 65, id: `mavlink::common::msg::RC_CHANNELS::MSG_ID`]
- [`SERVO_OUTPUT_RAW`](https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW) [handler: handle_servo_output_raw, dialect: common, msg_id: 36, id: `mavlink::common::msg::SERVO_OUTPUT_RAW::MSG_ID`]


## MAVLink Publications
- [`RC_CHANNELS_OVERRIDE`](https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE) [arg: `msg`, dialect: common, msg_id: 70, id: `mavlink::common::msg::RC_CHANNELS_OVERRIDE::MSG_ID`]
